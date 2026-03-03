"""
==============================================================================
PICOBOT HOST - Real-time Visualization with Keyboard Control
==============================================================================

This visualizer receives IMU data from the Pico robot and displays:
1. Raw sensor vectors (accelerometer, gyroscope)
2. Robot orientation (3D rotating box)
3. Estimated path/movement on a grid with ultrasonic cone
4. Time-series plots of all sensor channels

NEW FEATURES:
- Keyboard control (arrow keys to drive robot)
- Ultrasonic sensor visualization
- Better stationary detection to reduce drift
- AI motion detection (classifies: STOP, FWD, BWD, T_L, T_R, PUSH)
- CSV export for ML training data

CONTROLS:
  Arrow Up/Down    - Forward/Backward
  Arrow Left/Right - Turn Left/Right
  Space            - Stop
  R                - Reset position
  F                - Toggle filters
  A                - Toggle AI prediction display
  E                - Export raw CSV data
  Q/Esc            - Quit

TRAINING MODE (collect labeled data for ML):
  T                - Toggle training mode ON/OFF
  0-5              - Set current label:
                     0=STOP, 1=FWD, 2=BWD, 3=T_L, 4=T_R, 5=PUSH
  X                - Export training data (features + labels)
  C                - Clear training data

==============================================================================
"""

# Note: If you see OpenGL shader errors, they're warnings and can be ignored
# The visualization should still work
import os

import socket
import struct
import sys
import time
import numpy as np
import warnings
from collections import deque
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem
import math
import datetime

# Ensure data directory exists for exports
DATA_DIR = os.path.join(os.path.dirname(__file__), 'data')
os.makedirs(DATA_DIR, exist_ok=True)

# Import analyzers (optional - for advanced analysis)
try:
    from analyzers import BumpDetector, VibrationAnalyzer, MotionClassifier, FFTAnalyzer
    ANALYZERS_AVAILABLE = True
except ImportError:
    ANALYZERS_AVAILABLE = False
    print("Note: Analyzers not available. Install with: pip install sklearn")

# Suppress OpenGL warnings (common on some systems)
warnings.filterwarnings('ignore', category=RuntimeWarning)
import OpenGL
OpenGL.ERROR_CHECKING = False  # Disable OpenGL error checking for performance

# ==============================================================================
# AI MOTION DETECTION
# ==============================================================================

# Movement class labels
AI_CLASSES = ['STOP', 'FWD', 'BWD', 'T_L', 'T_R', 'PUSH']
AI_STOPPED, AI_FORWARD, AI_BACKWARD, AI_TURN_LEFT, AI_TURN_RIGHT, AI_PUSHED = range(6)


class FeatureExtractor:
    """Extract statistical features from sensor data windows."""

    def __init__(self, window_size=50):
        self.window_size = window_size
        self.ax_buf, self.ay_buf, self.az_buf = [], [], []
        self.gx_buf, self.gy_buf, self.gz_buf = [], [], []

    def add_sample(self, ax, ay, az, gx, gy, gz):
        for buf, val in [(self.ax_buf, ax), (self.ay_buf, ay), (self.az_buf, az),
                         (self.gx_buf, gx), (self.gy_buf, gy), (self.gz_buf, gz)]:
            buf.append(val)
            if len(buf) > self.window_size:
                buf.pop(0)

    def is_ready(self):
        return len(self.ax_buf) >= self.window_size // 2

    @staticmethod
    def _var(data):
        if len(data) < 2:
            return 0
        m = sum(data) / len(data)
        return sum((x - m) ** 2 for x in data) / len(data)

    @staticmethod
    def _zc(data):
        if len(data) < 2:
            return 0
        mean = sum(data) / len(data)
        crossings = 0
        above = data[0] > mean
        for x in data[1:]:
            now = x > mean
            if now != above:
                crossings += 1
                above = now
        return crossings

    def extract_features(self):
        if not self.is_ready():
            return None
        features = []
        for buf in [self.ax_buf, self.ay_buf, self.az_buf]:
            features.extend([sum(buf)/len(buf), self._var(buf)*100, self._zc(buf)])
        for buf in [self.gx_buf, self.gy_buf, self.gz_buf]:
            features.extend([sum(buf)/len(buf), self._var(buf)])
        mag = [math.sqrt(x*x+y*y+z*z) for x,y,z in zip(self.ax_buf, self.ay_buf, self.az_buf)]
        features.extend([sum(mag)/len(mag), self._var(mag)*100])
        return features


class RuleBasedClassifier:
    """Simple rule-based movement classifier."""

    def __init__(self):
        self.gyro_turn = 15.0      # deg/s for turn detection
        self.accel_move = 0.08    # g for movement detection
        self.vibration = 3.0      # variance for wheel motion
        self.stationary_var = 1.0

    def predict(self, features):
        if features is None or len(features) < 20:
            return AI_STOPPED

        ax_mean, ax_var = features[0], features[1]
        gz_mean, gz_var = features[12], features[13]
        mag_var = features[19]

        # Turning detection
        if abs(gz_mean) > self.gyro_turn:
            return AI_TURN_LEFT if gz_mean > 0 else AI_TURN_RIGHT

        # Stationary check
        if mag_var < self.stationary_var and gz_var < 5.0:
            return AI_STOPPED

        # Push detection (high variance spike)
        if mag_var > self.vibration * 5:
            return AI_PUSHED

        # Linear motion
        if mag_var > self.vibration:
            return AI_FORWARD if ax_mean > self.accel_move else (AI_BACKWARD if ax_mean < -self.accel_move else AI_FORWARD)

        return AI_STOPPED


# Initialize AI components
ai_feature_extractor = FeatureExtractor(window_size=50)
ai_classifier = RuleBasedClassifier()
ai_prediction = AI_STOPPED
ai_enabled = True  # Toggle with 'A' key

# ==============================================================================
# TRAINING DATA COLLECTION
# ==============================================================================

training_mode = False          # Toggle with 'T' key
training_label = AI_STOPPED    # Current label (0-5 keys)
training_data = []             # List of (features, label) tuples
training_sample_count = 0      # Samples collected in current session

FEATURE_NAMES = [
    'ax_mean', 'ax_var', 'ax_zc',
    'ay_mean', 'ay_var', 'ay_zc',
    'az_mean', 'az_var', 'az_zc',
    'gx_mean', 'gx_var',
    'gy_mean', 'gy_var',
    'gz_mean', 'gz_var',
    'mag_mean', 'mag_var'
]


def export_training_data(filename=None):
    """Export training data to CSV for ML frameworks."""
    import datetime
    if filename is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"training_data_{timestamp}.csv"

    if not training_data:
        print("\n>>> No training data to export!")
        return None

    with open(filename, 'w') as f:
        # Header compatible with sklearn, TensorFlow, Edge Impulse
        f.write(','.join(FEATURE_NAMES + ['label', 'label_name']) + '\n')

        for features, label in training_data:
            feat_str = ','.join(f"{v:.6f}" for v in features)
            f.write(f"{feat_str},{label},{AI_CLASSES[label]}\n")

    print(f"\n>>> Exported {len(training_data)} samples to {filename}")

    # Also print class distribution
    counts = [0] * 6
    for _, label in training_data:
        counts[label] += 1
    print("    Class distribution:")
    for i, name in enumerate(AI_CLASSES):
        if counts[i] > 0:
            print(f"  {name}: {counts[i]}")

    return filename


def get_training_stats():
    """Get statistics about collected training data."""
    counts = [0] * 6
    for _, label in training_data:
        counts[label] += 1
    return counts


# ==============================================================================
# CONFIGURATION
# ==============================================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# Control port - send commands back to robot
ROBOT_IP = None  # Set when first packet received
ROBOT_CONTROL_PORT = 5006
DISCOVERY_ADDR = "255.255.255.255"
DISCOVERY_INTERVAL_S = 1.0

# Batch configuration (must match main.py)
BATCH_SIZE = 20  # Increased for 500Hz sampling
SAMPLE_SIZE_V1 = struct.calcsize("<Q6f")   # 32 bytes per sample (no ultrasonic)
SAMPLE_SIZE_V2 = struct.calcsize("<Q7f")   # 36 bytes per sample (with ultrasonic)
SAMPLE_SIZE_V3 = struct.calcsize("<Q13f")  # 60 bytes per sample (with debug data)

# Packet types
PACKET_TYPE_IMU_ULTRASONIC = 0x02
PACKET_TYPE_IMU_DEBUG = 0x03  # New v3 format with debug data

# ==============================================================================
# DEAD RECKONING PARAMETERS
# ==============================================================================

GRAVITY_G = 1.0
ACCEL_THRESHOLD_G = 0.03     # Threshold for forward motion detection
GYRO_THRESHOLD_DPS = 1.0     # Threshold for rotation detection
VELOCITY_DAMPING = 0.90      # Velocity decay factor (more aggressive to prevent drift)
POSITION_SCALE = 2.0         # Scale factor for visualization
MAX_VELOCITY = 0.5           # Maximum realistic velocity in m/s (clamp to this)

# IMU axis configuration - adjust based on how IMU is mounted
# Forward axis: 'x' or 'y', and sign: 1 or -1
FORWARD_AXIS = 'x'           # X-axis is forward (based on debug output)
FORWARD_SIGN = -1            # Flipped: was 1, now -1 (track was inverted)
YAW_SIGN = 1                 # Flipped: was -1, now +1

# Complementary filter for orientation
ALPHA = 0.98

# Speed/position fusion (ultrasonic + IMU accel)
SPEED_FUSION_ALPHA = 0.85
US_SPEED_MAX_CM_S = 200.0
US_SPEED_MIN_DT = 0.02
IMU_ACCEL_TO_MS2 = 9.81
US_JUMP_MAX_CM = 30.0
BUMP_CALIB_MIN_SPEED = 5.0
BUMP_BAND_MIN_HZ = 5.0
BUMP_BAND_MAX_HZ = 60.0
DEFAULT_BUMP_PER_CM = 0.45

# Stationary detection (tighter thresholds for clean BMI160 data)
STATIONARY_ACCEL_VAR_THRESHOLD = 0.0005  # Tighter — BMI160 is cleaner
STATIONARY_GYRO_VAR_THRESHOLD = 0.3      # Tighter for faster ZUPT trigger
STATIONARY_WINDOW = 25                   # Slightly shorter window

# Acceleration smoothing (simple moving average)
ACCEL_SMOOTH_WINDOW = 5  # Increased for better vibration filtering

# Dead reckoning from IMU
DR_ACCEL_THRESHOLD_G = 0.02    # Ignore accel below this (noise floor)
DR_VELOCITY_DAMPING = 0.98     # Gentle damping when accel below threshold
DR_HP_ALPHA = 0.98             # High-pass filter to remove gravity residual
DR_ONSET_SAMPLES = 10          # Samples to accumulate at motion start for direction

# ==============================================================================
# CHART CONFIG
# ==============================================================================

CHART_LABELS = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'US', 'M_L', 'M_R', 'Bumps', 'Vib']
CHART_COLORS = ['r', 'g', 'b', 'm', 'y', 'c', (255, 128, 0), (0, 200, 0), (0, 100, 200), (255, 0, 255), (100, 255, 100)]
CHART_Y_RANGES = [
    (-2, 2),       # ax: ±2g
    (-2, 2),       # ay: ±2g
    (-2, 2),       # az: ±2g
    (-100, 100),   # gx: ±100°/s
    (-100, 100),   # gy: ±100°/s
    (-100, 100),   # gz: ±100°/s
    (0, 100),      # US: 0-100cm
    (-150, 150),   # M_L: motor left speed
    (-150, 150),   # M_R: motor right speed
    (0, 500),      # Bumps: cumulative count
    (0, 50),       # Vib: vibration variance
]
CHART_FIELD_KEYS = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'us', 'motor_l', 'motor_r', 'bumps', 'vib']

# ==============================================================================
# INITIALIZE QT APPLICATION
# ==============================================================================

app = QtWidgets.QApplication(sys.argv)

# ==============================================================================
# UDP SOCKETS
# ==============================================================================

# Receive socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"Listening on UDP port {UDP_PORT}...")

# Control socket (for sending commands to robot)
control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Discovery timing
last_discovery_time = 0.0

# Log receive socket (for robot debug messages)
LOG_PORT = 5008
log_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
log_sock.bind((UDP_IP, LOG_PORT))
log_sock.setblocking(False)
print(f"Log listener on UDP port {LOG_PORT}...")

# ==============================================================================
# MAIN WINDOW CLASS
# ==============================================================================

class MainWindow(QtWidgets.QMainWindow):
    """Main application window with proper layout."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PicoBot Controller - IMU Visualization & Control")
        self.setGeometry(50, 50, 1400, 900)
        self._recording_mode = None
        self._ml_flow_active = False
        self._vibration_flow_active = False

        # Create menu bar
        self._create_menus()

        # Create toolbar with control buttons
        self._create_toolbar()

        # Create status bar
        self.status_bar = self.statusBar()
        self.status_label = QtWidgets.QLabel("No robot connected")
        self.status_bar.addPermanentWidget(self.status_label)

        # Create central widget with splitter layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(5, 5, 5, 5)

        # Main splitter: left (charts) | middle (speed/position) | right (3D + path)
        self.splitter = QtWidgets.QSplitter(Qt.Horizontal)
        layout.addWidget(self.splitter)

        # Left panel: Time series charts
        self.charts_widget = pg.GraphicsLayoutWidget()
        self.charts_panel = self._create_charts_panel()
        self.splitter.addWidget(self.charts_panel)

        # Middle panel: speed/position
        self.speed_panel = self._create_speed_panel()
        self.speed_panel.setVisible(False)
        self.splitter.addWidget(self.speed_panel)

        # Right panel: 3D view + 2D path (stacked vertically)
        right_splitter = QtWidgets.QSplitter(Qt.Vertical)
        self.splitter.addWidget(right_splitter)

        # 3D Robot view
        self.view_3d = gl.GLViewWidget()
        right_splitter.addWidget(self.view_3d)

        # 2D Path view
        self.path_widget = pg.PlotWidget()
        self.path_widget.setAspectLocked(True)
        self.path_widget.setXRange(-10, 10)
        self.path_widget.setYRange(-10, 10)
        self.path_widget.showGrid(x=True, y=True, alpha=0.3)
        right_splitter.addWidget(self.path_widget)

        # Set splitter sizes (left 40%, middle 0%, right 60%)
        self.splitter.setSizes([560, 0, 840])
        right_splitter.setSizes([450, 450])

        # Chart controls dialog
        self.chart_controls_dialog = QtWidgets.QDialog(self)
        self.chart_controls_dialog.setWindowTitle("Chart Controls")
        self.chart_controls_dialog.setLayout(QtWidgets.QVBoxLayout())
        self.chart_controls = self._create_chart_controls_widget()
        self.chart_controls_dialog.layout().addWidget(self.chart_controls)

    def _create_charts_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(self.charts_widget)
        return panel

    def _create_chart_controls_widget(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)

        fields_box = QtWidgets.QGroupBox("Fields (plot + export)")
        grid = QtWidgets.QGridLayout(fields_box)
        self.chart_checks = []
        for i, label in enumerate(CHART_LABELS):
            chk = QtWidgets.QCheckBox(label)
            chk.setChecked(True)
            chk.toggled.connect(self._apply_chart_visibility)
            self.chart_checks.append(chk)
            grid.addWidget(chk, i // 4, i % 4)
        layout.addWidget(fields_box)

        view_box = QtWidgets.QGroupBox("View Range")
        vlayout = QtWidgets.QVBoxLayout(view_box)
        row = QtWidgets.QHBoxLayout()
        self.chk_follow_live = QtWidgets.QCheckBox("Follow live")
        self.chk_follow_live.setChecked(True)
        self.chk_follow_live.toggled.connect(self._update_view_controls)
        row.addWidget(self.chk_follow_live)
        row.addWidget(QtWidgets.QLabel("Window (s):"))
        self.spin_view_window = QtWidgets.QDoubleSpinBox()
        self.spin_view_window.setRange(1.0, 60.0)
        self.spin_view_window.setValue(10.0)
        self.spin_view_window.valueChanged.connect(self._update_view_controls)
        row.addWidget(self.spin_view_window)
        vlayout.addLayout(row)

        self.slider_view_offset = QtWidgets.QSlider(Qt.Horizontal)
        self.slider_view_offset.setRange(0, 1000)
        self.slider_view_offset.setValue(1000)
        self.slider_view_offset.valueChanged.connect(self._update_view_controls)
        vlayout.addWidget(self.slider_view_offset)

        self.lbl_view_range = QtWidgets.QLabel("Range: --")
        vlayout.addWidget(self.lbl_view_range)

        self.chk_export_view = QtWidgets.QCheckBox("Export view range only")
        self.chk_export_view.setChecked(True)
        vlayout.addWidget(self.chk_export_view)

        layout.addWidget(view_box)

        self.view_start_s = None
        self.view_end_s = None
        self._update_view_controls()
        return panel

    def _apply_chart_visibility(self):
        pass

    def _update_view_controls(self):
        follow = self.chk_follow_live.isChecked()
        self.slider_view_offset.setEnabled(not follow)
        if follow:
            self.slider_view_offset.blockSignals(True)
            self.slider_view_offset.setValue(1000)
            self.slider_view_offset.blockSignals(False)
    def _create_speed_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)

        controls = QtWidgets.QHBoxLayout()
        self.chk_speed_us = QtWidgets.QCheckBox("US speed")
        self.chk_speed_imu = QtWidgets.QCheckBox("IMU speed")
        self.chk_speed_fused = QtWidgets.QCheckBox("Fused speed")
        self.chk_speed_bump = QtWidgets.QCheckBox("Bump speed")
        for chk in (self.chk_speed_us, self.chk_speed_imu, self.chk_speed_fused):
            chk.blockSignals(True)
            chk.setChecked(True)
            chk.blockSignals(False)
            controls.addWidget(chk)
        self.chk_speed_bump.setChecked(True)
        controls.addWidget(self.chk_speed_bump)
        layout.addLayout(controls)

        method_row = QtWidgets.QHBoxLayout()
        method_row.addWidget(QtWidgets.QLabel("Bump method:"))
        self.cmb_bump_method = QtWidgets.QComboBox()
        self.cmb_bump_method.addItems(["Zero-Cross", "FFT", "Variance"])
        self.cmb_bump_method.currentTextChanged.connect(self._set_bump_method)
        method_row.addWidget(self.cmb_bump_method)
        layout.addLayout(method_row)

        est_row = QtWidgets.QHBoxLayout()
        self.chk_speed_est = QtWidgets.QCheckBox("Enable speed/pos")
        self.chk_speed_est.setChecked(False)
        self.chk_speed_est.toggled.connect(self._toggle_speed_est)
        est_row.addWidget(self.chk_speed_est)
        layout.addLayout(est_row)

        band_row = QtWidgets.QHBoxLayout()
        band_row.addWidget(QtWidgets.QLabel("Band (Hz):"))
        self.spin_band_min = QtWidgets.QDoubleSpinBox()
        self.spin_band_min.setRange(0.1, 200.0)
        self.spin_band_min.setValue(BUMP_BAND_MIN_HZ)
        self.spin_band_min.valueChanged.connect(self._set_bump_band)
        self.spin_band_max = QtWidgets.QDoubleSpinBox()
        self.spin_band_max.setRange(1.0, 300.0)
        self.spin_band_max.setValue(BUMP_BAND_MAX_HZ)
        self.spin_band_max.valueChanged.connect(self._set_bump_band)
        band_row.addWidget(self.spin_band_min)
        band_row.addWidget(QtWidgets.QLabel("to"))
        band_row.addWidget(self.spin_band_max)
        layout.addLayout(band_row)

        self.speed_plot = pg.PlotWidget(title="Speed (cm/s)")
        self.speed_plot.addLegend()
        self.speed_plot.setLabel('bottom', 'Time', 's')
        self.speed_plot.setYRange(0, 100)
        layout.addWidget(self.speed_plot)

        self.speed_curve_us = self.speed_plot.plot(pen=pg.mkPen((255, 128, 0), width=2), name="US")
        self.speed_curve_imu = self.speed_plot.plot(pen=pg.mkPen((0, 200, 200), width=2), name="IMU")
        self.speed_curve_fused = self.speed_plot.plot(pen=pg.mkPen((0, 255, 0), width=2), name="Fused")
        self.speed_curve_bump = self.speed_plot.plot(pen=pg.mkPen((200, 100, 255), width=2), name="Bump")

        self.pos_plot = pg.PlotWidget(title="Position (cm)")
        self.pos_plot.addLegend()
        self.pos_plot.setLabel('bottom', 'Time', 's')
        self.pos_plot.setYRange(-200, 200)
        layout.addWidget(self.pos_plot)

        self.pos_curve_us = self.pos_plot.plot(pen=pg.mkPen((255, 128, 0), width=2), name="US")
        self.pos_curve_imu = self.pos_plot.plot(pen=pg.mkPen((0, 200, 200), width=2), name="IMU")
        self.pos_curve_fused = self.pos_plot.plot(pen=pg.mkPen((0, 255, 0), width=2), name="Fused")
        self.pos_curve_bump = self.pos_plot.plot(pen=pg.mkPen((200, 100, 255), width=2), name="Bump")

        self.err_plot = pg.PlotWidget(title="Speed Error vs US (cm/s)")
        self.err_plot.addLegend()
        self.err_plot.setLabel('bottom', 'Time', 's')
        self.err_plot.setYRange(-50, 50)
        layout.addWidget(self.err_plot)

        self.err_curve_imu = self.err_plot.plot(pen=pg.mkPen((0, 200, 200), width=2), name="IMU")
        self.err_curve_fused = self.err_plot.plot(pen=pg.mkPen((0, 255, 0), width=2), name="Fused")
        self.err_curve_bump = self.err_plot.plot(pen=pg.mkPen((200, 100, 255), width=2), name="Bump")

        calib_row = QtWidgets.QHBoxLayout()
        self.btn_calib_start = QtWidgets.QPushButton("Start Cal")
        self.btn_calib_start.clicked.connect(self._start_bump_calibration)
        self.btn_calib_stop = QtWidgets.QPushButton("Stop Cal")
        self.btn_calib_stop.clicked.connect(self._stop_bump_calibration)
        self.btn_calib_reset = QtWidgets.QPushButton("Reset Cal")
        self.btn_calib_reset.clicked.connect(self._reset_bump_calibration)
        calib_row.addWidget(self.btn_calib_start)
        calib_row.addWidget(self.btn_calib_stop)
        calib_row.addWidget(self.btn_calib_reset)
        layout.addLayout(calib_row)

        self.lbl_bump_cm = QtWidgets.QLabel("Bump/cm: --")
        self.lbl_bump_cm_avg = QtWidgets.QLabel("Avg: -- (0)")
        self.lbl_var_fit = QtWidgets.QLabel("Var fit: --")
        self.lbl_bump_cm_warn = QtWidgets.QLabel("")
        layout.addWidget(self.lbl_bump_cm)
        layout.addWidget(self.lbl_bump_cm_avg)
        layout.addWidget(self.lbl_var_fit)
        layout.addWidget(self.lbl_bump_cm_warn)

        for chk in (self.chk_speed_us, self.chk_speed_imu, self.chk_speed_fused, self.chk_speed_bump):
            chk.toggled.connect(self._update_speed_curve_visibility)
        self._update_speed_curve_visibility()

        return panel

    def _update_speed_curve_visibility(self):
        self.speed_curve_us.setVisible(self.chk_speed_us.isChecked())
        self.speed_curve_imu.setVisible(self.chk_speed_imu.isChecked())
        self.speed_curve_fused.setVisible(self.chk_speed_fused.isChecked())
        self.speed_curve_bump.setVisible(self.chk_speed_bump.isChecked())
        self.pos_curve_us.setVisible(self.chk_speed_us.isChecked())
        self.pos_curve_imu.setVisible(self.chk_speed_imu.isChecked())
        self.pos_curve_fused.setVisible(self.chk_speed_fused.isChecked())
        self.pos_curve_bump.setVisible(self.chk_speed_bump.isChecked())
        self.err_curve_imu.setVisible(self.chk_speed_imu.isChecked())
        self.err_curve_fused.setVisible(self.chk_speed_fused.isChecked())
        self.err_curve_bump.setVisible(self.chk_speed_bump.isChecked())

    def _set_bump_method(self, method):
        global bump_method
        bump_method = method
        self.status_bar.showMessage(f"Bump method: {method}", 2000)

    def _toggle_speed_est(self):
        global speed_est_enabled
        speed_est_enabled = self.chk_speed_est.isChecked()
        self.status_bar.showMessage(
            "Speed/position enabled" if speed_est_enabled else "Speed/position disabled",
            2000,
        )

    def _set_bump_band(self):
        global bump_band_min_hz, bump_band_max_hz
        bump_band_min_hz = float(self.spin_band_min.value())
        bump_band_max_hz = float(self.spin_band_max.value())
        if bump_band_min_hz > bump_band_max_hz:
            bump_band_min_hz, bump_band_max_hz = bump_band_max_hz, bump_band_min_hz
            self.spin_band_min.setValue(bump_band_min_hz)
            self.spin_band_max.setValue(bump_band_max_hz)

    def _toggle_pause_updates(self):
        global pause_updates
        pause_updates = not pause_updates
        self.btn_pause.setChecked(pause_updates)
        self.btn_pause.setText("▶ Resume" if pause_updates else "⏸ Pause")
        self.status_bar.showMessage("Updates paused" if pause_updates else "Updates resumed", 2000)

    def _toggle_speed_panel(self):
        show = self.btn_speed_panel.isChecked()
        self.speed_panel.setVisible(show)
        sizes = self.splitter.sizes()
        if show:
            if len(sizes) >= 3 and sizes[1] == 0:
                total = sum(sizes) or 1750
                self.splitter.setSizes([int(total * 0.4), int(total * 0.2), int(total * 0.4)])
        else:
            if len(sizes) >= 3:
                self.splitter.setSizes([sizes[0] + sizes[1], 0, sizes[2]])

    def _show_chart_controls(self):
        self.chart_controls_dialog.show()
        self.chart_controls_dialog.raise_()
        self.chart_controls_dialog.activateWindow()

    def _start_bump_calibration(self):
        global calib_active, calib_bump_cm_samples, calib_bump_cm_avg, calib_invalid_count
        global calib_var_pairs, calib_var_slope, calib_var_intercept
        msg = (
            "Bump/cm Calibration\n\n"
            "1) Face a wall with ultrasonic in range\n"
            "2) Drive at steady speed for 5-10s\n"
            "3) Press Stop to lock average\n"
        )
        if QtWidgets.QMessageBox.question(self, "Start Bump Calibration?", msg) != QtWidgets.QMessageBox.Yes:
            return
        calib_active = True
        calib_bump_cm_samples = []
        calib_bump_cm_avg = None
        calib_invalid_count = 0
        calib_var_pairs = []
        calib_var_slope = None
        calib_var_intercept = None
        self.lbl_var_fit.setText("Var fit: --")
        self.status_bar.showMessage("Bump/cm calibration running...", 2000)

    def _stop_bump_calibration(self):
        global calib_active, calib_bump_cm_samples, calib_bump_cm_avg
        calib_active = False
        if calib_bump_cm_samples:
            calib_bump_cm_avg = sum(calib_bump_cm_samples) / len(calib_bump_cm_samples)
            QtWidgets.QMessageBox.information(
                self,
                "Bump Calibration",
                f"Avg bump/cm: {calib_bump_cm_avg:.4f}\nSamples: {len(calib_bump_cm_samples)}",
            )
        else:
            QtWidgets.QMessageBox.information(self, "Bump Calibration", "No valid samples collected.")
        self.status_bar.showMessage("Bump/cm calibration stopped", 2000)

    def _reset_bump_calibration(self):
        global calib_active, calib_bump_cm_samples, calib_bump_cm_avg, calib_invalid_count
        global calib_var_pairs, calib_var_slope, calib_var_intercept
        calib_active = False
        calib_bump_cm_samples = []
        calib_bump_cm_avg = None
        calib_invalid_count = 0
        calib_var_pairs = []
        calib_var_slope = None
        calib_var_intercept = None
        self.lbl_bump_cm.setText("Bump/cm: --")
        self.lbl_bump_cm_avg.setText("Avg: -- (0)")
        self.lbl_var_fit.setText("Var fit: --")
        self.lbl_bump_cm_warn.setText("")
        self.status_bar.showMessage("Bump/cm calibration reset", 2000)

    def _create_menus(self):
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        open_data = QtWidgets.QAction("Open &Data Folder...", self)
        open_data.triggered.connect(self._open_data_folder)
        file_menu.addAction(open_data)

        export_action = QtWidgets.QAction("&Export CSV...", self)
        export_action.setShortcut("Ctrl+E")
        export_action.triggered.connect(lambda: export_csv())
        file_menu.addAction(export_action)

        file_menu.addSeparator()

        quit_action = QtWidgets.QAction("&Quit", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # Recording menu
        rec_menu = menubar.addMenu("&Record")

        rec_toggle = QtWidgets.QAction("Start/Stop &Recording", self)
        rec_toggle.setShortcut("Ctrl+R")
        rec_toggle.triggered.connect(self._toggle_recording)
        rec_menu.addAction(rec_toggle)

        rec_clear = QtWidgets.QAction("&Clear Recording", self)
        rec_clear.triggered.connect(self._clear_recording)
        rec_menu.addAction(rec_clear)

        rec_export = QtWidgets.QAction("&Export Recording...", self)
        rec_export.setShortcut("Ctrl+S")
        rec_export.triggered.connect(self._export_recording)
        rec_menu.addAction(rec_export)

        rec_menu.addSeparator()

        analyze_action = QtWidgets.QAction("&Analyze Data (B/cm)...", self)
        analyze_action.setShortcut("Ctrl+A")
        analyze_action.triggered.connect(self._analyze_data)
        rec_menu.addAction(analyze_action)

        # Analysis menu (advanced analyzers)
        analysis_menu = menubar.addMenu("A&nalysis")

        bump_analyze = QtWidgets.QAction("&Bump Detection (B/cm)...", self)
        bump_analyze.triggered.connect(self._run_bump_analyzer)
        analysis_menu.addAction(bump_analyze)

        vib_analyze = QtWidgets.QAction("&Vibration Analysis...", self)
        vib_analyze.triggered.connect(self._run_vibration_analyzer)
        analysis_menu.addAction(vib_analyze)

        fft_analyze = QtWidgets.QAction("&FFT Frequency Analysis...", self)
        fft_analyze.triggered.connect(self._run_fft_analyzer)
        analysis_menu.addAction(fft_analyze)

        analysis_menu.addSeparator()

        train_ai = QtWidgets.QAction("&Train Motion Classifier...", self)
        train_ai.triggered.connect(self._train_motion_classifier)
        analysis_menu.addAction(train_ai)

        eval_ai = QtWidgets.QAction("&Evaluate Classifier...", self)
        eval_ai.triggered.connect(self._evaluate_classifier)
        analysis_menu.addAction(eval_ai)

        analysis_menu.addSeparator()

        open_analyzers = QtWidgets.QAction("Open &Analyzers Folder...", self)
        open_analyzers.triggered.connect(self._open_analyzers_folder)
        analysis_menu.addAction(open_analyzers)

        # AI Data menu
        ai_menu = menubar.addMenu("AI &Data")

        ai_toggle = QtWidgets.QAction("Toggle Auto-&Label Collection", self)
        ai_toggle.setShortcut("Ctrl+L")
        ai_toggle.triggered.connect(self._toggle_ai_collect)
        ai_menu.addAction(ai_toggle)

        ai_stats = QtWidgets.QAction("Show &Stats", self)
        ai_stats.triggered.connect(self._show_ai_stats)
        ai_menu.addAction(ai_stats)

        ai_export = QtWidgets.QAction("&Export AI Dataset...", self)
        ai_export.triggered.connect(self._export_ai_data)
        ai_menu.addAction(ai_export)

        ai_clear = QtWidgets.QAction("&Clear AI Data", self)
        ai_clear.triggered.connect(self._clear_ai_data)
        ai_menu.addAction(ai_clear)

        # View menu
        view_menu = menubar.addMenu("&View")

        reset_action = QtWidgets.QAction("&Reset Position", self)
        reset_action.setShortcut("R")
        reset_action.triggered.connect(lambda: reset_position())
        view_menu.addAction(reset_action)

        filter_action = QtWidgets.QAction("Toggle &Filters", self)
        filter_action.setShortcut("F")
        filter_action.triggered.connect(self._toggle_filters)
        view_menu.addAction(filter_action)

        ai_action = QtWidgets.QAction("Toggle &AI Display", self)
        ai_action.setShortcut("I")
        ai_action.triggered.connect(self._toggle_ai)
        view_menu.addAction(ai_action)

        # Calibration menu
        cal_menu = menubar.addMenu("&Calibration")

        cal_action = QtWidgets.QAction("Start &Calibration", self)
        cal_action.setShortcut("C")
        cal_action.triggered.connect(lambda: send_command(b'C'))
        cal_menu.addAction(cal_action)

        verify_action = QtWidgets.QAction("Run &Verification", self)
        verify_action.setShortcut("V")
        verify_action.triggered.connect(lambda: send_command(b'V'))
        cal_menu.addAction(verify_action)

        # Training menu
        train_menu = menubar.addMenu("&Training")

        train_toggle = QtWidgets.QAction("Toggle Training &Mode", self)
        train_toggle.setShortcut("T")
        train_toggle.triggered.connect(self._toggle_training)
        train_menu.addAction(train_toggle)

        export_train = QtWidgets.QAction("&Export Training Data", self)
        export_train.setShortcut("X")
        export_train.triggered.connect(lambda: export_training_data())
        train_menu.addAction(export_train)

        clear_train = QtWidgets.QAction("&Clear Training Data", self)
        clear_train.setShortcut("Z")
        clear_train.triggered.connect(self._clear_training)
        train_menu.addAction(clear_train)

        # Playback menu
        play_menu = menubar.addMenu("&Playback")

        play_toggle = QtWidgets.QAction("Toggle &Playback Mode", self)
        play_toggle.setShortcut("P")
        play_toggle.triggered.connect(self._toggle_playback)
        play_menu.addAction(play_toggle)

        play_pause = QtWidgets.QAction("Pause/Resume", self)
        play_pause.setShortcut("Space")
        play_pause.triggered.connect(self._toggle_playback_pause)
        play_menu.addAction(play_pause)

        play_back = QtWidgets.QAction("Step &Back (10 samples)", self)
        play_back.setShortcut("[")
        play_back.triggered.connect(lambda: self._playback_step(-10))
        play_menu.addAction(play_back)

        play_fwd = QtWidgets.QAction("Step &Forward (10 samples)", self)
        play_fwd.setShortcut("]")
        play_fwd.triggered.connect(lambda: self._playback_step(10))
        play_menu.addAction(play_fwd)

        play_start = QtWidgets.QAction("Go to &Start", self)
        play_start.setShortcut("Home")
        play_start.triggered.connect(self._playback_start)
        play_menu.addAction(play_start)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        controls_action = QtWidgets.QAction("Show &Controls", self)
        controls_action.setShortcut("H")
        controls_action.triggered.connect(lambda: print_help())
        help_menu.addAction(controls_action)

        about_action = QtWidgets.QAction("&About", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

        # View menu
        view_menu = menubar.addMenu("&View")
        chart_controls_action = QtWidgets.QAction("Chart &Controls", self)
        chart_controls_action.triggered.connect(self._show_chart_controls)
        view_menu.addAction(chart_controls_action)

        # Workflow menu
        flow_menu = menubar.addMenu("&Workflow")

        quick_rec = QtWidgets.QAction("Record + &Export", self)
        quick_rec.setShortcut("Ctrl+R")
        quick_rec.triggered.connect(self._toggle_quick_record)
        flow_menu.addAction(quick_rec)

        ml_flow = QtWidgets.QAction("&ML Capture Flow", self)
        ml_flow.setShortcut("Ctrl+M")
        ml_flow.triggered.connect(self._toggle_ml_flow)
        flow_menu.addAction(ml_flow)

        vib_flow = QtWidgets.QAction("&Vibration Report Pipeline", self)
        vib_flow.setShortcut("Ctrl+B")
        vib_flow.triggered.connect(self._toggle_vibration_flow)
        flow_menu.addAction(vib_flow)

    def _create_toolbar(self):
        """Create toolbar with control buttons."""
        toolbar = self.addToolBar("Controls")
        toolbar.setMovable(False)

        # Recording button
        self.btn_record = QtWidgets.QPushButton("⏺ Record")
        self.btn_record.setCheckable(True)
        self.btn_record.setStyleSheet("QPushButton:checked { background-color: #ff4444; color: white; }")
        self.btn_record.clicked.connect(self._toggle_recording)
        toolbar.addWidget(self.btn_record)

        # Quick record + export button
        self.btn_quick_record = QtWidgets.QPushButton("⏺ Rec+Export")
        self.btn_quick_record.setCheckable(True)
        self.btn_quick_record.setStyleSheet("QPushButton:checked { background-color: #44aa44; color: white; }")
        self.btn_quick_record.clicked.connect(self._toggle_quick_record)
        toolbar.addWidget(self.btn_quick_record)

        toolbar.addSeparator()

        # AI Collection button
        self.btn_ai_collect = QtWidgets.QPushButton("🤖 AI Collect")
        self.btn_ai_collect.setCheckable(True)
        self.btn_ai_collect.setStyleSheet("QPushButton:checked { background-color: #4444ff; color: white; }")
        self.btn_ai_collect.clicked.connect(self._toggle_ai_collect)
        toolbar.addWidget(self.btn_ai_collect)

        # AI Stats button
        btn_ai_stats = QtWidgets.QPushButton("📊 Stats")
        btn_ai_stats.clicked.connect(self._show_ai_stats)
        toolbar.addWidget(btn_ai_stats)

        # ML capture flow button
        self.btn_ml_flow = QtWidgets.QPushButton("🧠 ML Flow")
        self.btn_ml_flow.setCheckable(True)
        self.btn_ml_flow.setStyleSheet("QPushButton:checked { background-color: #5555aa; color: white; }")
        self.btn_ml_flow.clicked.connect(self._toggle_ml_flow)
        toolbar.addWidget(self.btn_ml_flow)

        toolbar.addSeparator()

        # Calibration button
        btn_cal = QtWidgets.QPushButton("📏 Calibrate")
        btn_cal.clicked.connect(lambda: send_command(b'C'))
        toolbar.addWidget(btn_cal)

        # Verify button
        btn_verify = QtWidgets.QPushButton("✓ Verify")
        btn_verify.clicked.connect(lambda: send_command(b'V'))
        toolbar.addWidget(btn_verify)

        toolbar.addSeparator()

        # Analyze button
        btn_analyze = QtWidgets.QPushButton("📈 Analyze")
        btn_analyze.clicked.connect(self._analyze_data)
        toolbar.addWidget(btn_analyze)

        # Export button
        btn_export = QtWidgets.QPushButton("💾 Export")
        btn_export.clicked.connect(self._export_recording)
        toolbar.addWidget(btn_export)

        # Speed panel toggle
        self.btn_speed_panel = QtWidgets.QPushButton("📌 Speed Panel")
        self.btn_speed_panel.setCheckable(True)
        self.btn_speed_panel.clicked.connect(self._toggle_speed_panel)
        toolbar.addWidget(self.btn_speed_panel)

        # Chart controls popup
        btn_chart_controls = QtWidgets.QPushButton("📊 Chart Controls")
        btn_chart_controls.clicked.connect(self._show_chart_controls)
        toolbar.addWidget(btn_chart_controls)

        # Pause updates button
        self.btn_pause = QtWidgets.QPushButton("⏸ Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self._toggle_pause_updates)
        toolbar.addWidget(self.btn_pause)

        # Vibration report pipeline button
        self.btn_vib_flow = QtWidgets.QPushButton("📡 Vib Report")
        self.btn_vib_flow.setCheckable(True)
        self.btn_vib_flow.setStyleSheet("QPushButton:checked { background-color: #aa7755; color: white; }")
        self.btn_vib_flow.clicked.connect(self._toggle_vibration_flow)
        toolbar.addWidget(self.btn_vib_flow)

        toolbar.addSeparator()

        # Reset button
        btn_reset = QtWidgets.QPushButton("🔄 Reset")
        btn_reset.clicked.connect(lambda: reset_position())
        toolbar.addWidget(btn_reset)

    def _toggle_filters(self):
        global filters_enabled
        filters_enabled = not filters_enabled
        self.status_bar.showMessage(f"Filters: {'ON' if filters_enabled else 'OFF'}", 2000)

    def _toggle_ai(self):
        global ai_enabled
        ai_enabled = not ai_enabled
        self.status_bar.showMessage(f"AI Display: {'ON' if ai_enabled else 'OFF'}", 2000)

    def _toggle_training(self):
        global training_mode, training_sample_count
        training_mode = not training_mode
        if training_mode:
            training_sample_count = 0
        self.status_bar.showMessage(f"Training Mode: {'ON' if training_mode else 'OFF'}", 2000)

    def _clear_training(self):
        global training_data
        training_data = []
        self.status_bar.showMessage("Training data cleared", 2000)

    def _toggle_playback(self):
        global playback_mode, playback_index, playback_paused
        playback_mode = not playback_mode
        if playback_mode:
            playback_index = 0
            playback_paused = True
            n = len(playback_buffer)
            self.status_bar.showMessage(f"Playback Mode ON - {n} samples, press Space to play", 3000)
        else:
            self.status_bar.showMessage("Playback Mode OFF - live data", 2000)

    def _toggle_playback_pause(self):
        global playback_paused
        if playback_mode:
            playback_paused = not playback_paused
            status = "PAUSED" if playback_paused else "PLAYING"
            self.status_bar.showMessage(f"Playback: {status}", 1000)

    def _playback_step(self, delta):
        global playback_index
        if playback_mode:
            playback_index = max(0, min(len(playback_buffer)-1, playback_index + delta))
            self.status_bar.showMessage(f"Playback: {playback_index}/{len(playback_buffer)}", 1000)

    def _playback_start(self):
        global playback_index
        if playback_mode:
            playback_index = 0
            self.status_bar.showMessage("Playback: Start", 1000)

    def _toggle_recording(self):
        if self._recording_mode == "quick":
            self.btn_quick_record.setChecked(False)
        if self._recording_mode == "vibration":
            self.btn_vib_flow.setChecked(False)
        if not is_recording:
            self._start_recording("manual")
            return
        self._stop_recording()

    def _start_recording(self, mode):
        global is_recording, recording_buffer
        is_recording = True
        recording_buffer = []  # Clear on start
        self._recording_mode = mode
        self.btn_record.setChecked(True)
        self.btn_record.setText("⏹ Stop")
        if mode == "quick":
            self.btn_quick_record.setChecked(True)
        elif mode == "vibration":
            self.btn_vib_flow.setChecked(True)
        self.status_bar.showMessage("⏺ RECORDING STARTED", 2000)
        print("\n>>> Recording started")

    def _stop_recording(self, auto_export=False, run_vibration=False):
        global is_recording
        n = len(recording_buffer)
        is_recording = False
        self._recording_mode = None
        self.btn_record.setChecked(False)
        self.btn_record.setText("⏺ Record")
        self.btn_quick_record.setChecked(False)
        self.btn_vib_flow.setChecked(False)
        self.status_bar.showMessage(f"⏹ Recording stopped - {n} samples", 2000)
        print(f"\n>>> Recording stopped - {n} samples")

        filename = None
        if auto_export or run_vibration:
            filename = self._export_recording()
        if run_vibration and filename:
            self._run_vibration_report(filename, recording_buffer)

    def _toggle_quick_record(self):
        if not is_recording:
            if self._recording_mode == "vibration":
                self.btn_vib_flow.setChecked(False)
            msg = (
                "Quick Record + Export\n\n"
                "1) Place robot on flat surface\n"
                "2) Drive to capture data\n"
                "3) Press again to stop + export CSV\n"
            )
            if QtWidgets.QMessageBox.question(self, "Start Quick Record?", msg) != QtWidgets.QMessageBox.Yes:
                self.btn_quick_record.setChecked(False)
                return
            self._start_recording("quick")
            self.status_bar.showMessage("Quick record ON - press again to export", 2000)
            return
        if self._recording_mode == "quick":
            self._stop_recording(auto_export=True)
            return
        self.btn_quick_record.setChecked(False)
        self.status_bar.showMessage("Recording in progress (manual/vibration)", 2000)

    def _toggle_vibration_flow(self):
        if not is_recording:
            if self._recording_mode == "quick":
                self.btn_quick_record.setChecked(False)
            msg = (
                "Vibration Report Flow\n\n"
                "1) Place robot on flat surface\n"
                "2) Drive at a steady speed for ~10s\n"
                "3) Press again to stop, export CSV, and generate report\n"
            )
            if QtWidgets.QMessageBox.question(self, "Start Vibration Flow?", msg) != QtWidgets.QMessageBox.Yes:
                self.btn_vib_flow.setChecked(False)
                return
            self._start_recording("vibration")
            self._vibration_flow_active = True
            self.status_bar.showMessage("Vibration flow ON - press again for report", 2000)
            return
        if self._recording_mode == "vibration":
            self._vibration_flow_active = False
            self._stop_recording(auto_export=True, run_vibration=True)
            return
        self.btn_vib_flow.setChecked(False)
        self.status_bar.showMessage("Recording in progress (manual/quick)", 2000)

    def _toggle_ml_flow(self):
        global ai_collect_enabled, ai_data_collection
        if not self._ml_flow_active:
            msg = (
                "ML Capture Flow\n\n"
                "1) Clear floor and give robot space\n"
                "2) Drive with WASD/Arrow keys to label data\n"
                "3) Press again to stop and export dataset\n"
            )
            if QtWidgets.QMessageBox.question(self, "Start ML Flow?", msg) != QtWidgets.QMessageBox.Yes:
                self.btn_ml_flow.setChecked(False)
                return
            for k in ai_data_collection:
                ai_data_collection[k] = []
            ai_collect_enabled = True
            self.btn_ai_collect.setChecked(True)
            self.btn_ai_collect.setText("🤖 Collecting...")
            self.btn_ml_flow.setChecked(True)
            self._ml_flow_active = True
            self.status_bar.showMessage("ML flow ON - drive with WASD to label", 3000)
            print("\n>>> ML flow started - data auto-labeled by keyboard commands")
            return

        self._ml_flow_active = False
        ai_collect_enabled = False
        self.btn_ai_collect.setChecked(False)
        self.btn_ai_collect.setText("🤖 AI Collect")
        self.btn_ml_flow.setChecked(False)
        self._export_ai_data()
        self.status_bar.showMessage("ML flow OFF - dataset exported", 3000)

    def _run_vibration_report(self, csv_filename, data):
        try:
            from analyzers.vibration_analyzer import VibrationAnalyzer
            from analyzers.fft_analyzer import FFTAnalyzer
        except ImportError:
            self.status_bar.showMessage("Vibration analyzers not available", 2000)
            return None

        if not data:
            self.status_bar.showMessage("No data for vibration report", 2000)
            return None

        speed_samples = []
        speed_indices = []
        last_us = None
        last_t = None
        for idx, sample in enumerate(data):
            us = sample.get('us', -1)
            t = sample.get('time_s', None)
            if us is None or us <= 0 or t is None:
                last_us = us
                last_t = t
                continue
            if last_us is None or last_us <= 0 or last_t is None:
                last_us = us
                last_t = t
                continue
            dt = t - last_t
            if dt <= 0:
                last_us = us
                last_t = t
                continue
            speed_cm_s = abs(us - last_us) / dt
            if speed_cm_s <= 200:
                speed_samples.append(speed_cm_s)
                speed_indices.append(idx)
            last_us = us
            last_t = t

        vib = VibrationAnalyzer(window_size=50)
        vib.load_data(data)
        vib_results = vib.analyze()

        fft = FFTAnalyzer(sample_rate=500)
        fft_data = {
            'time_s': [s['time_s'] for s in data],
            'ax': [s['ax'] for s in data],
            'ay': [s['ay'] for s in data],
            'az': [s['az'] for s in data],
            'gx': [s['gx'] for s in data],
            'gy': [s['gy'] for s in data],
            'gz': [s['gz'] for s in data],
            'us': [s['us'] for s in data],
        }
        fft.load_data(fft_data)
        fft.compute_resultant()
        fft_results = fft.analyze(channel='ax')

        speed_stats = {}
        if speed_samples:
            sorted_speeds = sorted(speed_samples)
            mid = len(sorted_speeds) // 2
            if len(sorted_speeds) % 2 == 0:
                median_speed = (sorted_speeds[mid - 1] + sorted_speeds[mid]) / 2
            else:
                median_speed = sorted_speeds[mid]
            avg_speed = sum(speed_samples) / len(speed_samples)
            speed_stats = {
                'count': len(speed_samples),
                'avg_speed_cm_s': avg_speed,
                'median_speed_cm_s': median_speed,
                'min_speed_cm_s': min(speed_samples),
                'max_speed_cm_s': max(speed_samples),
            }

        fit_stats = {}
        if speed_stats:
            variances = vib_results.get('variances', [])
            pairs = []
            for i, s in zip(speed_indices, speed_samples):
                if i < len(variances):
                    pairs.append((variances[i], s))
            if len(pairs) >= 5:
                xs = [p[0] for p in pairs]
                ys = [p[1] for p in pairs]
                x_mean = sum(xs) / len(xs)
                y_mean = sum(ys) / len(ys)
                denom = sum((x - x_mean) ** 2 for x in xs)
                if denom > 0:
                    slope = sum((x - x_mean) * (y - y_mean) for x, y in pairs) / denom
                    intercept = y_mean - slope * x_mean
                    fit_stats = {'slope': slope, 'intercept': intercept}

        report_path = csv_filename.replace(".csv", "_vibration_report.txt")
        with open(report_path, 'w') as f:
            f.write("VIBRATION REPORT\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Source: {os.path.basename(csv_filename)}\n")
            f.write(f"Samples: {vib_results.get('total_samples', 0)}\n")
            f.write(f"Duration: {vib_results.get('duration', 0):.2f} s\n\n")
            f.write("Vibration Stats:\n")
            f.write(f"  Avg variance: {vib_results.get('avg_variance', 0):.6f}\n")
            f.write(f"  Max variance: {vib_results.get('max_variance', 0):.6f}\n")
            f.write(f"  Avg robot vib: {vib_results.get('avg_robot_vib', 0):.2f}\n\n")
            f.write("Ultrasonic Speed Estimates:\n")
            if speed_stats:
                f.write(f"  Samples: {speed_stats['count']}\n")
                f.write(f"  Avg speed: {speed_stats['avg_speed_cm_s']:.2f} cm/s\n")
                f.write(f"  Median speed: {speed_stats['median_speed_cm_s']:.2f} cm/s\n")
                f.write(f"  Min/Max: {speed_stats['min_speed_cm_s']:.2f} / {speed_stats['max_speed_cm_s']:.2f} cm/s\n")
                if fit_stats:
                    f.write("  Vibration->Speed fit:\n")
                    f.write(f"    speed = {fit_stats['slope']:.3f} * variance + {fit_stats['intercept']:.3f}\n")
            else:
                f.write("  No valid ultrasonic speed samples\n")
            f.write("\n")
            f.write("Motion State Percentages:\n")
            for state, pct in vib_results.get('state_percentages', {}).items():
                f.write(f"  {state:8s}: {pct:5.1f}%\n")
            f.write("\nFFT Peak (ax):\n")
            if 'error' in fft_results:
                f.write(f"  FFT error: {fft_results['error']}\n")
            else:
                f.write(f"  Peak frequency: {fft_results['peak_frequency']:.2f} Hz\n")
                f.write(f"  Peak magnitude: {fft_results['peak_magnitude']:.4f}\n")
                f.write(f"  Sample rate: {fft_results['sample_rate']:.1f} Hz\n")
                if speed_stats and speed_stats['avg_speed_cm_s'] > 0:
                    bump_per_cm = fft_results['peak_frequency'] / speed_stats['avg_speed_cm_s']
                    f.write(f"  Bump/cm (calibrated): {bump_per_cm:.4f}\n")
            f.write("\n")

        self.status_bar.showMessage(f"Vibration report saved: {report_path}", 3000)
        print(f"\n>>> Vibration report saved to {report_path}")
        summary_lines = [
            "Vibration Report",
            "=" * 30,
            f"File: {os.path.basename(report_path)}",
            f"Samples: {vib_results.get('total_samples', 0)}",
            f"Duration: {vib_results.get('duration', 0):.2f} s",
        ]
        if speed_stats:
            summary_lines.append(f"US avg speed: {speed_stats['avg_speed_cm_s']:.2f} cm/s")
        if fit_stats:
            summary_lines.append(f"Fit: speed = {fit_stats['slope']:.3f} * var + {fit_stats['intercept']:.3f}")
        if speed_stats and 'error' not in fft_results and speed_stats['avg_speed_cm_s'] > 0:
            bump_per_cm = fft_results['peak_frequency'] / speed_stats['avg_speed_cm_s']
            summary_lines.append(f"Bump/cm: {bump_per_cm:.4f}")
        QtWidgets.QMessageBox.information(self, "Vibration Report", "\n".join(summary_lines))
        return report_path

    def _clear_recording(self):
        global recording_buffer
        recording_buffer = []
        self.status_bar.showMessage("Recording cleared", 2000)

    def _export_recording(self):
        """Export recording to CSV file in data/ directory."""
        global recording_buffer
        if len(recording_buffer) == 0:
            self.status_bar.showMessage("No recorded data to export!", 2000)
            return None

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(DATA_DIR, f"recording_{timestamp}.csv")

        selected_raw = {k for k, chk in zip(CHART_FIELD_KEYS, self.chart_checks) if chk.isChecked()}
        include_us = self.chk_speed_us.isChecked()
        include_imu = self.chk_speed_imu.isChecked()
        include_fused = self.chk_speed_fused.isChecked()
        include_bump = self.chk_speed_bump.isChecked()

        fields = ['time_s', 'ts_us']
        for key in CHART_FIELD_KEYS:
            if key in selected_raw:
                fields.append(key)
        if include_us:
            fields += ['speed_us', 'pos_us']
        if include_imu:
            fields += ['speed_imu', 'pos_imu']
        if include_fused:
            fields += ['speed_fused', 'pos_fused']
        if include_bump:
            fields += ['speed_bump', 'pos_bump', 'bump_per_cm', 'bump_method', 'bump_freq_hz',
                       'bump_var', 'var_slope', 'var_intercept']
        if include_us and (include_imu or include_fused or include_bump):
            fields += ['err_imu', 'err_fused', 'err_bump']

        view_start = self.view_start_s
        view_end = self.view_end_s
        if self.chk_export_view.isChecked() and view_start is not None and view_end is not None:
            export_rows = [s for s in recording_buffer if view_start <= s['time_s'] <= view_end]
        else:
            export_rows = recording_buffer

        with open(filename, 'w') as f:
            f.write(",".join(fields) + "\n")
            for sample in export_rows:
                row = []
                for key in fields:
                    val = sample.get(key, "")
                    if isinstance(val, float):
                        if key in ('time_s',):
                            row.append(f"{val:.6f}")
                        elif key in ('ax', 'ay', 'az', 'gx', 'gy', 'gz'):
                            row.append(f"{val:.4f}")
                        elif key in ('speed_us', 'speed_imu', 'speed_fused', 'speed_bump',
                                     'pos_us', 'pos_imu', 'pos_fused', 'pos_bump',
                                     'err_imu', 'err_fused', 'err_bump'):
                            row.append(f"{val:.3f}")
                        elif key in ('bump_per_cm', 'bump_var', 'var_slope'):
                            row.append(f"{val:.6f}")
                        elif key in ('var_intercept',):
                            row.append(f"{val:.3f}")
                        else:
                            row.append(f"{val:.3f}")
                    else:
                        row.append(str(val))
                f.write(",".join(row) + "\n")

        self.status_bar.showMessage(f"Exported {len(recording_buffer)} samples to {filename}", 3000)
        print(f"\n>>> Exported {len(recording_buffer)} samples to {filename}")
        return filename

    def _analyze_data(self):
        """
        Analyze recorded data to calculate B/cm and other metrics.
        Uses BumpDetector analyzer if available for advanced analysis.
        """
        global recording_buffer
        if len(recording_buffer) < 100:
            self.status_bar.showMessage("Need more data (at least 100 samples)", 2000)
            return

        data = recording_buffer
        n = len(data)

        # Use BumpDetector analyzer if available
        if ANALYZERS_AVAILABLE:
            detector = BumpDetector(threshold=0.015, hp_alpha=0.95)
            detector.load_data(data)
            results = detector.analyze()

            # Build analysis text from analyzer results
            analysis = (
                f"Recording Analysis (using BumpDetector)\n"
                f"{'='*45}\n\n"
                f"Duration: {results['duration']:.2f} s\n"
                f"Samples: {results['total_samples']}\n\n"
                f"Ultrasonic Distance:\n"
                f"  Start: {results['us_start']:.1f} cm\n"
                f"  End: {results['us_end']:.1f} cm\n"
                f"  Travel: {results['us_distance']:.1f} cm\n\n"
                f"Bumps:\n"
                f"  Detected (accel): {results['detected_bumps']}\n"
                f"  Robot counter:    {results['robot_bumps']}\n"
                f"  Used:             {results['total_bumps']}\n\n"
            )

            # Add segment analysis if available
            if results['segments']:
                analysis += f"Motion Segments ({len(results['segments'])}):\n"
                for i, seg in enumerate(results['segments'][:3]):  # Show first 3
                    analysis += f"  #{i+1}: {seg['duration']:.1f}s, {seg['us_distance']:.0f}cm, B/cm={seg['bumps_per_cm']:.2f}\n"
                analysis += "\n"

            analysis += (
                f"{'='*45}\n"
                f"B/cm = {results['bumps_per_cm']:.3f}\n"
                f"{'='*45}\n\n"
                f"Tip: Run standalone for detailed report:\n"
                f"  python analyzers/bump_detector.py data/file.csv"
            )

            bumps_per_cm = results['bumps_per_cm']
            total_bumps = results['total_bumps']
            us_distance = results['us_distance']
        else:
            # Fallback: simple inline analysis
            start_bumps = data[0]['bumps']
            end_bumps = data[-1]['bumps']
            total_bumps = end_bumps - start_bumps

            start_us = data[0]['us']
            end_us = data[-1]['us']
            us_distance = abs(end_us - start_us)

            duration = data[-1]['time_s'] - data[0]['time_s']

            motor_samples = [s for s in data if abs(s['motor_l']) > 10 or abs(s['motor_r']) > 10]
            moving_pct = 100 * len(motor_samples) / n if n > 0 else 0

            bumps_per_cm = total_bumps / us_distance if us_distance > 1 else 0

            analysis = (
                f"Recording Analysis\n"
                f"{'='*40}\n\n"
                f"Duration: {duration:.2f} s\n"
                f"Samples: {n}\n"
                f"Moving: {moving_pct:.1f}% of time\n\n"
                f"Ultrasonic Distance:\n"
                f"  Start: {start_us:.1f} cm\n"
                f"  End: {end_us:.1f} cm\n"
                f"  Travel: {us_distance:.1f} cm\n\n"
                f"Bumps:\n"
                f"  Start: {start_bumps}\n"
                f"  End: {end_bumps}\n"
                f"  Total: {total_bumps}\n\n"
                f"{'='*40}\n"
                f"B/cm = {bumps_per_cm:.3f}\n"
                f"{'='*40}"
            )

        QtWidgets.QMessageBox.information(self, "Data Analysis", analysis)
        print(f"\n>>> Analysis: B/cm = {bumps_per_cm:.3f} ({total_bumps} bumps / {us_distance:.1f} cm)")

    def _toggle_ai_collect(self):
        global ai_collect_enabled
        ai_collect_enabled = not ai_collect_enabled
        self.btn_ai_collect.setChecked(ai_collect_enabled)
        if ai_collect_enabled:
            self.btn_ai_collect.setText("🤖 Collecting...")
            self.status_bar.showMessage("🤖 AI Auto-Label Collection ON", 2000)
            print("\n>>> AI auto-label collection ON - data tagged by keyboard commands")
        else:
            self.btn_ai_collect.setText("🤖 AI Collect")
            self.status_bar.showMessage("AI Collection OFF", 2000)

    def _show_ai_stats(self):
        global ai_data_collection
        stats = "\n".join([f"  {k}: {len(v)} samples" for k, v in ai_data_collection.items()])
        total = sum(len(v) for v in ai_data_collection.values())
        msg = f"AI Dataset Statistics\n{'='*30}\n{stats}\n{'='*30}\nTotal: {total} samples"
        QtWidgets.QMessageBox.information(self, "AI Data Stats", msg)

    def _export_ai_data(self):
        """Export AI labeled data to data/ directory as JSON and CSV."""
        global ai_data_collection
        import json

        total = sum(len(v) for v in ai_data_collection.values())
        if total == 0:
            self.status_bar.showMessage("No AI data to export!", 2000)
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        json_filename = os.path.join(DATA_DIR, f"ai_dataset_{timestamp}.json")
        csv_filename = os.path.join(DATA_DIR, f"ai_dataset_{timestamp}.csv")

        # Export as JSON for easy loading
        with open(json_filename, 'w') as f:
            json.dump(ai_data_collection, f, indent=2)

        # Also export as CSV for ML tools (compatible with MotionClassifier)
        with open(csv_filename, 'w') as f:
            f.write("label,ax,ay,az,gx,gy,gz,us,motor_l,motor_r,bumps,vib\n")
            for label, samples in ai_data_collection.items():
                for s in samples:
                    f.write(f"{label},{s['ax']:.4f},{s['ay']:.4f},{s['az']:.4f},"
                            f"{s['gx']:.4f},{s['gy']:.4f},{s['gz']:.4f},"
                            f"{s['us']:.1f},{s['motor_l']},{s['motor_r']},"
                            f"{s['bumps']},{s['vib']:.2f}\n")

        self.status_bar.showMessage(f"Exported {total} samples to data/", 3000)
        print(f"\n>>> Exported AI dataset to:")
        print(f"    {json_filename}")
        print(f"    {csv_filename}")
        print(f"\n>>> Train model with:")
        print(f"    python analyzers/motion_classifier.py train {csv_filename}")

    def _clear_ai_data(self):
        global ai_data_collection
        for k in ai_data_collection:
            ai_data_collection[k] = []
        self.status_bar.showMessage("AI data cleared", 2000)

    def _show_about(self):
        QtWidgets.QMessageBox.about(self, "About PicoBot Controller",
            "PicoBot Controller v1.0\n\n"
            "Real-time IMU visualization and robot control.\n\n"
            "Controls:\n"
            "  WASD/Arrows - Drive robot\n"
            "  R - Reset position\n"
            "  C - Calibration\n"
            "  V - Verification\n"
        )

    # =========================================================================
    # DATA FOLDER AND ANALYZER METHODS
    # =========================================================================

    def _open_data_folder(self):
        """Open the data folder in file manager."""
        import subprocess
        import platform

        if platform.system() == 'Windows':
            os.startfile(DATA_DIR)
        elif platform.system() == 'Darwin':  # macOS
            subprocess.run(['open', DATA_DIR])
        else:  # Linux
            subprocess.run(['xdg-open', DATA_DIR])

        self.status_bar.showMessage(f"Opened {DATA_DIR}", 2000)

    def _open_analyzers_folder(self):
        """Open the analyzers folder in file manager."""
        import subprocess
        import platform

        analyzers_dir = os.path.join(os.path.dirname(__file__), 'analyzers')
        if platform.system() == 'Windows':
            os.startfile(analyzers_dir)
        elif platform.system() == 'Darwin':
            subprocess.run(['open', analyzers_dir])
        else:
            subprocess.run(['xdg-open', analyzers_dir])

        self.status_bar.showMessage(f"Opened {analyzers_dir}", 2000)

    def _run_bump_analyzer(self):
        """Run bump analyzer on a selected CSV file."""
        if not ANALYZERS_AVAILABLE:
            QtWidgets.QMessageBox.warning(self, "Analyzers Not Available",
                "Analyzers require sklearn. Install with:\n\npip install scikit-learn")
            return

        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Recording File", DATA_DIR,
            "CSV Files (*.csv);;All Files (*)"
        )
        if not filename:
            return

        detector = BumpDetector()
        detector.load_csv(filename)
        results = detector.analyze()

        # Show results dialog
        msg = (
            f"Bump Detection Analysis\n"
            f"{'='*45}\n\n"
            f"File: {os.path.basename(filename)}\n"
            f"Samples: {results['total_samples']}\n"
            f"Duration: {results['duration']:.2f} s\n\n"
            f"Ultrasonic Travel: {results['us_distance']:.1f} cm\n"
            f"Robot Bumps: {results['robot_bumps']}\n"
            f"Detected Bumps: {results['detected_bumps']}\n\n"
            f"{'='*45}\n"
            f"B/cm = {results['bumps_per_cm']:.3f}\n"
            f"{'='*45}\n\n"
            f"For detailed report, run:\n"
            f"  python analyzers/bump_detector.py {filename}"
        )
        QtWidgets.QMessageBox.information(self, "Bump Analysis", msg)

    def _run_vibration_analyzer(self):
        """Run vibration analyzer on a selected CSV file."""
        if not ANALYZERS_AVAILABLE:
            QtWidgets.QMessageBox.warning(self, "Analyzers Not Available",
                "Analyzers require sklearn. Install with:\n\npip install scikit-learn")
            return

        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Recording File", DATA_DIR,
            "CSV Files (*.csv);;All Files (*)"
        )
        if not filename:
            return

        analyzer = VibrationAnalyzer()
        analyzer.load_csv(filename)
        results = analyzer.analyze()

        # Format state distribution
        state_lines = []
        for state, pct in results['state_percentages'].items():
            bar = '#' * int(pct / 5)  # Scale bar
            state_lines.append(f"  {state:8s}: {pct:5.1f}% {bar}")
        state_text = '\n'.join(state_lines)

        msg = (
            f"Vibration Analysis\n"
            f"{'='*45}\n\n"
            f"File: {os.path.basename(filename)}\n"
            f"Samples: {results['total_samples']}\n"
            f"Duration: {results['duration']:.2f} s\n\n"
            f"Motion State Distribution:\n{state_text}\n\n"
            f"Avg Variance: {results['avg_variance']:.6f}\n"
            f"Max Variance: {results['max_variance']:.6f}\n"
            f"Motion Periods: {len(results['periods'])}\n\n"
            f"For detailed report, run:\n"
            f"  python analyzers/vibration_analyzer.py {filename}"
        )
        QtWidgets.QMessageBox.information(self, "Vibration Analysis", msg)

    def _run_fft_analyzer(self):
        """Run FFT frequency analysis on a selected CSV file."""
        if not ANALYZERS_AVAILABLE:
            QtWidgets.QMessageBox.warning(self, "Analyzers Not Available",
                "Analyzers require numpy. Install with:\n\npip install numpy")
            return

        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Recording File", DATA_DIR,
            "CSV Files (*.csv);;All Files (*)"
        )
        if not filename:
            return

        # Choose channel
        channels = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
        channel, ok = QtWidgets.QInputDialog.getItem(
            self, "Select Channel", "Analyze which sensor channel:",
            channels, 0, False
        )
        if not ok:
            return

        analyzer = FFTAnalyzer(sample_rate=500)
        analyzer.load_csv(filename)
        results = analyzer.analyze(channel)

        # Format harmonics
        harmonic_lines = []
        for h in results.get('harmonics', [])[:5]:
            harmonic_lines.append(f"  {h['frequency']:6.1f} Hz  ({h['relative']:5.1f}%)")
        harmonic_text = '\n'.join(harmonic_lines) if harmonic_lines else "  None detected"

        msg = (
            f"FFT Frequency Analysis\n"
            f"{'='*45}\n\n"
            f"File: {os.path.basename(filename)}\n"
            f"Channel: {channel}\n"
            f"Samples: {results['n_samples']}\n"
            f"Sample Rate: {results['sample_rate']} Hz\n\n"
            f"Frequency Resolution: {results['freq_resolution']:.2f} Hz\n"
            f"Max Frequency (Nyquist): {results['max_frequency']:.1f} Hz\n\n"
            f"Dominant Frequency:\n"
            f"  {results['peak_frequency']:.1f} Hz\n\n"
            f"Significant Peaks:\n{harmonic_text}\n\n"
            f"For plot, run:\n"
            f"  python analyzers/fft_analyzer.py {filename} -c {channel} --plot"
        )
        QtWidgets.QMessageBox.information(self, "FFT Analysis", msg)

    def _train_motion_classifier(self):
        """Train motion classifier on AI dataset."""
        if not ANALYZERS_AVAILABLE:
            QtWidgets.QMessageBox.warning(self, "Analyzers Not Available",
                "Analyzers require sklearn. Install with:\n\npip install scikit-learn")
            return

        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select AI Dataset", DATA_DIR,
            "CSV Files (*.csv);;JSON Files (*.json);;All Files (*)"
        )
        if not filename:
            return

        # Choose model type
        model_types = ['Random Forest (Recommended)', 'Decision Tree', 'KNN', 'Rule-based']
        model_map = {'Random Forest (Recommended)': 'forest', 'Decision Tree': 'tree',
                    'KNN': 'knn', 'Rule-based': 'rule'}

        choice, ok = QtWidgets.QInputDialog.getItem(
            self, "Select Model", "Choose classifier type:",
            model_types, 0, False
        )
        if not ok:
            return

        model_type = model_map[choice]
        clf = MotionClassifier(model_type=model_type)

        # Train
        if filename.endswith('.json'):
            features, labels = clf.load_training_json(filename)
        else:
            features, labels = clf.load_training_data(filename)

        if not features:
            QtWidgets.QMessageBox.warning(self, "Training Failed", "No valid training data found.")
            return

        clf.train(features, labels)

        # Evaluate
        results = clf.evaluate(features, labels)

        # Save model
        model_file = os.path.join(DATA_DIR, f"motion_model_{model_type}.pkl")
        clf.save_model(model_file)

        msg = (
            f"Motion Classifier Training\n"
            f"{'='*45}\n\n"
            f"Model: {choice}\n"
            f"Training Samples: {len(features)}\n"
            f"Accuracy: {results['accuracy']*100:.1f}%\n\n"
            f"Model saved to:\n  {model_file}\n\n"
            f"Use for predictions:\n"
            f"  python analyzers/motion_classifier.py predict {model_file} data.csv"
        )
        QtWidgets.QMessageBox.information(self, "Training Complete", msg)

    def _evaluate_classifier(self):
        """Evaluate trained classifier on test data."""
        if not ANALYZERS_AVAILABLE:
            QtWidgets.QMessageBox.warning(self, "Analyzers Not Available",
                "Analyzers require sklearn. Install with:\n\npip install scikit-learn")
            return

        # Select model file
        model_file, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Model File", DATA_DIR,
            "Model Files (*.pkl);;All Files (*)"
        )
        if not model_file:
            return

        # Select test data
        data_file, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Test Data", DATA_DIR,
            "CSV Files (*.csv);;JSON Files (*.json);;All Files (*)"
        )
        if not data_file:
            return

        clf = MotionClassifier()
        clf.load_model(model_file)

        if data_file.endswith('.json'):
            features, labels = clf.load_training_json(data_file)
        else:
            features, labels = clf.load_training_data(data_file)

        results = clf.evaluate(features, labels)

        # Format per-class stats
        class_lines = []
        for label in clf.label_names:
            stats = results['class_stats'][label]
            class_lines.append(f"  {label:12s} P:{stats['precision']:4.0%} R:{stats['recall']:4.0%} F1:{stats['f1']:4.0%}")
        class_text = '\n'.join(class_lines)

        msg = (
            f"Classifier Evaluation\n"
            f"{'='*45}\n\n"
            f"Model: {clf.model_type}\n"
            f"Test Samples: {results['total']}\n\n"
            f"Overall Accuracy: {results['accuracy']*100:.1f}%\n"
            f"({results['correct']}/{results['total']} correct)\n\n"
            f"Per-Class Performance:\n{class_text}"
        )
        QtWidgets.QMessageBox.information(self, "Evaluation Results", msg)

    def update_status(self, text):
        self.status_label.setText(text)

    def closeEvent(self, event):
        # Send quit command to robot
        if ROBOT_IP:
            try:
                control_sock.sendto(b'Q', (ROBOT_IP, ROBOT_CONTROL_PORT))
            except:
                pass
        print("\n>>> Closing...")
        event.accept()


def send_command(cmd):
    """Send command to robot."""
    if ROBOT_IP:
        try:
            control_sock.sendto(cmd, (ROBOT_IP, ROBOT_CONTROL_PORT))
        except:
            pass

# ==============================================================================
# KEYBOARD CONTROL STATE
# ==============================================================================

key_state = {
    'forward': False,
    'backward': False,
    'left': False,
    'right': False,
}
filters_enabled = True

def send_control_command():
    """Send motor control command to robot based on key state."""
    global ROBOT_IP, cmd_left_speed, cmd_right_speed
    if ROBOT_IP is None:
        return

    # Determine motor speeds from key state
    left_speed = 0
    right_speed = 0
    speed = 120       # Base speed for forward/backward
    turn_speed = 100  # Speed for turning

    if key_state['forward']:
        left_speed = speed
        right_speed = speed
    elif key_state['backward']:
        left_speed = -speed
        right_speed = -speed

    if key_state['left']:
        left_speed -= turn_speed
        right_speed += turn_speed
    elif key_state['right']:
        left_speed += turn_speed
        right_speed -= turn_speed

    # Clamp speeds
    left_speed = max(-255, min(255, left_speed))
    right_speed = max(-255, min(255, right_speed))

    # Store for command-based velocity estimation
    cmd_left_speed = left_speed
    cmd_right_speed = right_speed

    # Send command packet: "M" + left_speed (2 bytes) + right_speed (2 bytes)
    # Use '<' for little-endian, no padding
    try:
        cmd = struct.pack("<chh", b'M', left_speed, right_speed)
        control_sock.sendto(cmd, (ROBOT_IP, ROBOT_CONTROL_PORT))
        # Debug: uncomment to see sent values
        # print(f"Sent: L={left_speed} R={right_speed}")
    except Exception as e:
        pass  # Ignore send errors

# ==============================================================================
# CREATE MAIN WINDOW AND SETUP VISUALIZATIONS
# ==============================================================================

main_window = MainWindow()

# Setup 3D Robot view (inside main window)
win_robot = main_window.view_3d
win_robot.setCameraPosition(distance=10, elevation=30, azimuth=45)

grid_robot = gl.GLGridItem()
grid_robot.scale(2, 2, 1)
win_robot.addItem(grid_robot)

# Trail behind robot
robot_trail = gl.GLLinePlotItem(pos=np.array([[0, 0, 0]]), color=(0, 0.8, 0, 0.7), width=2, antialias=False)
win_robot.addItem(robot_trail)
trail_points_3d = deque(maxlen=500)


def make_box_mesh(sx=1.0, sy=0.6, sz=0.3):
    """Create a robot-shaped box mesh."""
    verts = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ]) / 2
    faces = np.array([
        [0, 1, 2], [0, 2, 3],  # bottom
        [4, 6, 5], [4, 7, 6],  # top
        [0, 4, 5], [0, 5, 1],  # front (+X)
        [2, 6, 7], [2, 7, 3],  # back (-X)
        [0, 3, 7], [0, 7, 4],  # left (+Y)
        [1, 5, 6], [1, 6, 2],  # right (-Y)
    ])
    colors = np.array([
        [0.2, 0.2, 0.8, 1], [0.2, 0.2, 0.8, 1],  # bottom - blue
        [0.8, 0.2, 0.2, 1], [0.8, 0.2, 0.2, 1],  # top - red
        [0.2, 0.8, 0.2, 1], [0.2, 0.8, 0.2, 1],  # front - green
        [0.8, 0.8, 0.2, 1], [0.8, 0.8, 0.2, 1],  # back - yellow
        [0.8, 0.2, 0.8, 1], [0.8, 0.2, 0.8, 1],  # left - magenta
        [0.2, 0.8, 0.8, 1], [0.2, 0.8, 0.8, 1],  # right - cyan
    ])
    return verts, faces, colors


box_verts, box_faces, box_colors = make_box_mesh()
robot_mesh = GLMeshItem(vertexes=box_verts, faces=box_faces, faceColors=box_colors,
                        smooth=False, drawEdges=True, edgeColor=(0, 0, 0, 1))
win_robot.addItem(robot_mesh)

# Forward direction arrow
arrow_line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]), color=(0, 1, 0, 1), width=4, antialias=False)
win_robot.addItem(arrow_line)

# ==============================================================================
# PATH VIEW (Top-Down 2D) with Ultrasonic - inside main window
# ==============================================================================

win_path = main_window.path_widget

MAX_PATH_POINTS = 1000
path_points_x = deque(maxlen=MAX_PATH_POINTS)
path_points_y = deque(maxlen=MAX_PATH_POINTS)

# Path trail (green line)
path_curve = win_path.plot(pen=pg.mkPen('g', width=2))

# Robot marker (red dot)
robot_marker_2d = win_path.plot(pen=None, symbol='o', symbolSize=12, symbolBrush='r')

# Heading arrow (red line from robot)
heading_line = win_path.plot(pen=pg.mkPen('r', width=3))

# Ultrasonic cone (orange)
ultrasonic_cone_2d = win_path.plot(pen=pg.mkPen(color=(255, 128, 0), width=2), fillLevel=None)

# Ultrasonic obstacle point (orange dot)
ultrasonic_marker = win_path.plot(pen=None, symbol='o', symbolSize=10, symbolBrush=(255, 128, 0))

# Ultrasonic distance text
ultrasonic_text = pg.TextItem(text="US: --", color='orange', anchor=(0, 1))
win_path.addItem(ultrasonic_text)
ultrasonic_text.setPos(-9, 9)

# ==============================================================================
# 2D PLOTS (Time Series) - Left panel in main window
# ==============================================================================

win2d = main_window.charts_widget
plots, curves, buffers = [], [], []
max_len = 2000
for i, label in enumerate(CHART_LABELS):
    p = win2d.addPlot(title=label)
    p.setYRange(CHART_Y_RANGES[i][0], CHART_Y_RANGES[i][1])
    p.enableAutoRange(axis='y', enable=False)  # Disable auto-range
    p.setLabel('bottom', 'Time', 's')  # X-axis label
    c = p.plot(pen=CHART_COLORS[i], name=label)
    plots.append(p)
    curves.append(c)
    buffers.append(deque(maxlen=max_len))
    if i < len(CHART_LABELS) - 1:
        win2d.nextRow()

# Timestamp buffer for time-based X-axis (in seconds, relative to first sample)
time_buffer = deque(maxlen=max_len)
first_timestamp = None  # Will be set on first sample after calibration

# Recording state - start/pause data logging
is_recording = False
recording_buffer = []  # Separate buffer for recorded data

# AI labeled data collection - auto-tagged by keyboard/command
ai_data_collection = {
    'STOPPED': [],
    'FORWARD': [],
    'BACKWARD': [],
    'TURN_LEFT': [],
    'TURN_RIGHT': [],
    'CALIBRATE': [],
}
ai_collect_enabled = False  # Toggle with Ctrl+L
current_motion_label = 'STOPPED'  # Current label based on key state

# Show main window
main_window.show()

# ==============================================================================
# STATE VARIABLES
# ==============================================================================

frame_count = 0
packet_count = 0
prev_pico_ts = None

# Calibration
calibration_samples = []
CALIBRATION_COUNT = 50
is_calibrated = False
accel_bias = np.array([0.0, 0.0, 0.0])
gyro_bias_z = 0.0

# Dead reckoning state
pitch = 0.0
roll = 0.0
yaw = 0.0
position = np.array([0.0, 0.0, 0.0])

# Motion detection state
motion_active = False
motion_velocity = 0.0
motion_direction = 1  # +1 forward, -1 backward (detected from accel)
motion_start_pos = np.array([0.0, 0.0, 0.0])
no_accel_count = 0  # Count samples with no acceleration

# Direction detection from acceleration slope
prev_forward_accel = 0.0
accel_slope_buffer = deque(maxlen=10)  # Track acceleration changes

# Dead reckoning state (IMU integration with ZUPT)
dr_velocity = 0.0              # Forward velocity from accel integration (m/s)
dr_hp_prev_ax = 0.0            # High-pass filter state for forward accel
dr_hp_prev_out = 0.0           # High-pass filter output
dr_motion_state = 'STOPPED'    # STOPPED → ONSET → MOVING (direction state machine)
dr_onset_accum = 0.0           # Accumulated accel during onset phase
dr_onset_count = 0             # Sample count during onset phase

# Ultrasonic
ultrasonic_cm = -1.0

# Stationary detection
accel_history = deque(maxlen=STATIONARY_WINDOW)
gyro_history = deque(maxlen=STATIONARY_WINDOW)
is_stationary = False
is_turning = False

# Low-pass filter state
lpf_ax, lpf_ay, lpf_az, lpf_gz = 0.0, 0.0, 0.0, 0.0
LPF_ALPHA = 0.25  # Balanced smoothing

# Moving average buffer for forward acceleration (extra smoothing)
fwd_accel_buffer = deque(maxlen=ACCEL_SMOOTH_WINDOW)

# Vibration detection buffers (for wheel bump detection experiment)
VIBRATION_WINDOW = 100  # Increased: ~200ms at 500Hz for better averaging
accel_x_raw_buffer = deque(maxlen=VIBRATION_WINDOW)
accel_y_raw_buffer = deque(maxlen=VIBRATION_WINDOW)
accel_z_raw_buffer = deque(maxlen=VIBRATION_WINDOW)

# Speed/position estimation buffers (cm/s, cm)
speed_us_buffer = deque(maxlen=max_len)
speed_imu_buffer = deque(maxlen=max_len)
speed_fused_buffer = deque(maxlen=max_len)
speed_bump_buffer = deque(maxlen=max_len)
pos_us_buffer = deque(maxlen=max_len)
pos_imu_buffer = deque(maxlen=max_len)
pos_fused_buffer = deque(maxlen=max_len)
pos_bump_buffer = deque(maxlen=max_len)
speed_err_imu_buffer = deque(maxlen=max_len)
speed_err_fused_buffer = deque(maxlen=max_len)
speed_err_bump_buffer = deque(maxlen=max_len)

speed_us_cm_s = 0.0
speed_imu_cm_s = 0.0
speed_fused_cm_s = 0.0
speed_bump_cm_s = 0.0
speed_dir_sign = 1
pos_us_cm = 0.0
pos_imu_cm = 0.0
pos_fused_cm = 0.0
pos_bump_cm = 0.0
last_us_cm = None
last_us_time_s = None
last_us_jump_bad = False

accel_mag_buffer = deque(maxlen=400)
accel_mag_time_buffer = deque(maxlen=400)

calib_active = False
calib_bump_cm_samples = []
calib_bump_cm_avg = DEFAULT_BUMP_PER_CM
calib_invalid_count = 0
calib_var_pairs = []
calib_var_slope = None
calib_var_intercept = None

fft_peak_hz = 0.0
last_fft_update_s = 0.0
bump_method = "Zero-Cross"
bump_band_min_hz = BUMP_BAND_MIN_HZ
bump_band_max_hz = BUMP_BAND_MAX_HZ
speed_est_enabled = False
pause_updates = False
last_udp_log_time = 0.0
udp_rx_count = 0

# CSV export buffer - store more samples for analysis
CSV_BUFFER_SIZE = 5000  # ~10 seconds at 500Hz
csv_buffer = deque(maxlen=CSV_BUFFER_SIZE)

# Last commanded motor speeds (for command-based velocity)
cmd_left_speed = 0
cmd_right_speed = 0

# Debug data from robot (v3 packet format)
robot_heading = 0.0      # Heading integrated on robot
robot_vib = 0.0          # Vibration variance from robot
robot_bumps = 0          # Total bump count from robot
robot_drift_corr = 0.0   # Drift correction being applied
robot_motor_l = 0        # Left motor speed
robot_motor_r = 0        # Right motor speed

# Playback feature
playback_mode = False
playback_buffer = deque(maxlen=50000)  # Store all received data
playback_index = 0
playback_paused = False


def rotation_matrix(pitch, roll, yaw):
    """Create rotation matrix from Euler angles (radians)."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def reset_position():
    """Reset position tracking to origin."""
    global position, motion_velocity, motion_direction, motion_active, no_accel_count
    global yaw, path_points_x, path_points_y, trail_points_3d
    global prev_forward_accel, accel_slope_buffer
    global dr_velocity, dr_hp_prev_ax, dr_hp_prev_out
    global dr_motion_state, dr_onset_accum, dr_onset_count
    global first_timestamp, time_buffer
    position = np.array([0.0, 0.0, 0.0])
    motion_velocity = 0.0
    motion_direction = 1
    motion_active = False
    no_accel_count = 0
    prev_forward_accel = 0.0
    accel_slope_buffer.clear()
    dr_velocity = 0.0
    dr_hp_prev_ax = 0.0
    dr_hp_prev_out = 0.0
    dr_motion_state = 'STOPPED'
    dr_onset_accum = 0.0
    dr_onset_count = 0
    yaw = 0.0
    path_points_x.clear()
    path_points_y.clear()
    trail_points_3d.clear()
    first_timestamp = None  # Reset time reference
    time_buffer.clear()
    for buf in buffers:
        buf.clear()
    print("\n>>> Position reset to origin")


def export_csv():
    """Export collected sensor data to CSV file."""
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"imu_data_{timestamp}.csv"

    if len(csv_buffer) == 0:
        print("\n>>> No data to export!")
        return

    with open(filename, 'w') as f:
        # Header
        f.write("timestamp_us,ax,ay,az,gx,gy,gz,cmd_left,cmd_right,ultrasonic_cm\n")

        # Data
        for sample in csv_buffer:
            f.write(f"{sample['ts']},{sample['ax']:.6f},{sample['ay']:.6f},{sample['az']:.6f},"
                    f"{sample['gx']:.4f},{sample['gy']:.4f},{sample['gz']:.4f},"
                    f"{sample['cmd_l']},{sample['cmd_r']},{sample['us']:.1f}\n")

    print(f"\n>>> Exported {len(csv_buffer)} samples to {filename}")


def detect_stationary(accel_mag, gyro_z):
    """Detect if robot is stationary using variance."""
    global is_stationary

    accel_history.append(accel_mag)
    gyro_history.append(gyro_z)

    if len(accel_history) < STATIONARY_WINDOW // 2:
        return False

    # Calculate variance
    accel_arr = np.array(accel_history)
    gyro_arr = np.array(gyro_history)

    accel_var = np.var(accel_arr)
    gyro_var = np.var(gyro_arr)

    is_stationary = (accel_var < STATIONARY_ACCEL_VAR_THRESHOLD and
                    gyro_var < STATIONARY_GYRO_VAR_THRESHOLD)

    return is_stationary


# ==============================================================================
# UPDATE FUNCTION
# ==============================================================================

def update():
    global frame_count, packet_count, prev_pico_ts, ROBOT_IP
    global last_discovery_time
    global is_calibrated, calibration_samples, accel_bias, gyro_bias_z
    global pitch, roll, yaw, position
    global motion_active, motion_velocity, motion_direction, no_accel_count
    global prev_forward_accel, accel_slope_buffer
    global dr_velocity, dr_hp_prev_ax, dr_hp_prev_out
    global dr_motion_state, dr_onset_accum, dr_onset_count
    global training_sample_count
    global ultrasonic_cm, is_stationary, is_turning
    global lpf_ax, lpf_ay, lpf_az, lpf_gz, fwd_accel_buffer
    global robot_heading, robot_vib, robot_bumps, robot_drift_corr, robot_motor_l, robot_motor_r
    global playback_mode, playback_index, playback_paused
    global first_timestamp, time_buffer
    global is_recording, recording_buffer
    global ai_collect_enabled, ai_data_collection, current_motion_label
    global speed_us_cm_s, speed_imu_cm_s, speed_fused_cm_s, speed_dir_sign
    global speed_bump_cm_s
    global pos_us_cm, pos_imu_cm, pos_fused_cm, pos_bump_cm
    global last_us_cm, last_us_time_s, last_us_jump_bad
    global accel_mag_buffer, accel_mag_time_buffer
    global calib_active, calib_bump_cm_samples, calib_bump_cm_avg, calib_invalid_count
    global calib_var_pairs, calib_var_slope, calib_var_intercept
    global fft_peak_hz, last_fft_update_s
    global bump_method, bump_band_min_hz, bump_band_max_hz
    global speed_est_enabled, pause_updates
    global last_udp_log_time, udp_rx_count

    if pause_updates:
        try:
            while True:
                sock.recvfrom(2048)
        except BlockingIOError:
            pass
        try:
            while True:
                log_sock.recvfrom(1024)
        except BlockingIOError:
            pass
        return

    if ROBOT_IP is None:
        now = time.time()
        if now - last_discovery_time >= DISCOVERY_INTERVAL_S:
            try:
                control_sock.sendto(b'H', (DISCOVERY_ADDR, ROBOT_CONTROL_PORT))
            except OSError:
                pass
            last_discovery_time = now

    # Check for log messages from robot
    try:
        log_data, _ = log_sock.recvfrom(1024)
        log_msg = log_data.decode().strip()
        print(f"\n[ROBOT] {log_msg}")
    except BlockingIOError:
        pass

    try:
        packet, addr = sock.recvfrom(2048)
        udp_rx_count += 1
        packet_count += 1

        # Save robot IP for control commands
        if ROBOT_IP is None:
            ROBOT_IP = addr[0]
            print(f"\n>>> Robot connected from {ROBOT_IP}")
            main_window.status_bar.showMessage(f"Robot connected from {ROBOT_IP}", 3000)

        # Detect packet format
        has_ultrasonic = False
        has_debug = False
        if len(packet) >= 2:
            first_byte = packet[0]
            if first_byte == PACKET_TYPE_IMU_DEBUG:
                # V3 format with debug data
                has_ultrasonic = True
                has_debug = True
                batch_size = packet[1]
                packet = packet[2:]  # Skip header
                sample_size = SAMPLE_SIZE_V3
            elif first_byte == PACKET_TYPE_IMU_ULTRASONIC:
                has_ultrasonic = True
                batch_size = packet[1]
                packet = packet[2:]  # Skip header
                sample_size = SAMPLE_SIZE_V2
            else:
                sample_size = SAMPLE_SIZE_V1

        num_samples = len(packet) // sample_size

        if packet_count == 1:
            fmt_name = "v3 (debug)" if has_debug else ("v2 (US)" if has_ultrasonic else "v1")
            print(f">>> Packet format: {fmt_name}, {len(packet)} bytes, {num_samples} samples/packet")
            print(f">>> Debug data: {'Enabled' if has_debug else 'Disabled'}")
            print(">>> Calibrating IMU... keep robot stationary!\n")

        # Process all samples
        for i in range(num_samples):
            offset = i * sample_size

            if has_debug:
                # V3 format: raw sensors + debug data
                sample_data = packet[offset:offset + sample_size]
                (pico_ts_us, ax, ay, az, gx, gy, gz, us,
                 r_heading, r_vib, r_bumps, r_drift, r_ml, r_mr) = struct.unpack("<Q13f", sample_data)  # explicit LE
                ultrasonic_cm = us
                robot_heading = r_heading
                robot_vib = r_vib
                robot_bumps = int(r_bumps)
                robot_drift_corr = r_drift
                robot_motor_l = int(r_ml)
                robot_motor_r = int(r_mr)
                # Store for playback
                playback_buffer.append((
                    pico_ts_us, ax, ay, az, gx, gy, gz, us,
                    r_heading, r_vib, r_bumps, r_drift, r_ml, r_mr
                ))
            elif has_ultrasonic:
                sample_data = packet[offset:offset + sample_size]
                pico_ts_us, ax, ay, az, gx, gy, gz, us = struct.unpack("<Q7f", sample_data)
                ultrasonic_cm = us
            else:
                sample_data = packet[offset:offset + sample_size]
                pico_ts_us, ax, ay, az, gx, gy, gz = struct.unpack("<Q6f", sample_data)

            # Auto-calibration
            if not is_calibrated:
                calibration_samples.append((ax, ay, az, gz))
                prev_pico_ts = pico_ts_us  # Track timestamp even during calibration
                if len(calibration_samples) >= CALIBRATION_COUNT:
                    samples = np.array(calibration_samples)
                    accel_bias[0] = np.mean(samples[:, 0])
                    accel_bias[1] = np.mean(samples[:, 1])
                    accel_bias[2] = np.mean(samples[:, 2]) - GRAVITY_G
                    gyro_bias_z = np.mean(samples[:, 3])
                    is_calibrated = True
                    print(f"Calibration complete!")
                    print(f"  Accel bias: ({accel_bias[0]:.4f}, {accel_bias[1]:.4f}, {accel_bias[2]:.4f})")
                    print(f"  Gyro Z bias: {gyro_bias_z:.4f}")
                continue

            # Time delta (Pico ticks_us wraps at 2^30)
            if prev_pico_ts is not None:
                dt_us = pico_ts_us - prev_pico_ts
                if dt_us < 0:
                    dt_us += 2**30  # Pico ticks_us wraparound
                dt = dt_us / 1_000_000.0
            else:
                dt = 0.005
            prev_pico_ts = pico_ts_us

            # Flag for unreasonable dt values (don't skip, just don't use for physics)
            bad_dt = dt > 0.5 or dt < 0.0005
            if bad_dt:
                dt = 0.002  # Use nominal dt for visualization continuity

            # Apply calibration
            ax_cal = ax - accel_bias[0]
            ay_cal = ay - accel_bias[1]
            az_cal = az - accel_bias[2]
            gz_cal = gz - gyro_bias_z

            # Apply low-pass filter if enabled
            if filters_enabled:
                lpf_ax = LPF_ALPHA * ax_cal + (1 - LPF_ALPHA) * lpf_ax
                lpf_ay = LPF_ALPHA * ay_cal + (1 - LPF_ALPHA) * lpf_ay
                lpf_az = LPF_ALPHA * az_cal + (1 - LPF_ALPHA) * lpf_az
                lpf_gz = LPF_ALPHA * gz_cal + (1 - LPF_ALPHA) * lpf_gz
                ax_cal, ay_cal, az_cal, gz_cal = lpf_ax, lpf_ay, lpf_az, lpf_gz

            # Stationary detection
            accel_mag = math.sqrt(ax_cal**2 + ay_cal**2 + az_cal**2)
            detect_stationary(accel_mag, gz_cal)

            # Collect raw samples for vibration analysis (BEFORE filtering)
            accel_x_raw_buffer.append(ax)  # Use raw, uncalibrated values
            accel_y_raw_buffer.append(ay)
            accel_z_raw_buffer.append(az)

            # Feed AI feature extractor (calibrated values)
            if ai_enabled or training_mode:
                ai_feature_extractor.add_sample(ax_cal, ay_cal, az_cal, gx, gy, gz_cal)

            # Collect training data when in training mode
            if training_mode and ai_feature_extractor.is_ready():
                features = ai_feature_extractor.extract_features()
                if features is not None:
                    training_data.append((features, training_label))
                    training_sample_count += 1

            # Store sample for CSV export
            csv_buffer.append({
                'ts': pico_ts_us,
                'ax': ax, 'ay': ay, 'az': az,
                'gx': gx, 'gy': gy, 'gz': gz,
                'cmd_l': cmd_left_speed, 'cmd_r': cmd_right_speed,
                'us': ultrasonic_cm
            })

            # Orientation from complementary filter
            accel_pitch = math.atan2(ax_cal, math.sqrt(ay_cal**2 + (az_cal + GRAVITY_G)**2))
            accel_roll = math.atan2(ay_cal, math.sqrt(ax_cal**2 + (az_cal + GRAVITY_G)**2))

            gx_rad = math.radians(gx)
            gy_rad = math.radians(gy)
            gz_rad = math.radians(gz_cal)

            pitch = ALPHA * (pitch + gx_rad * dt) + (1 - ALPHA) * accel_pitch
            roll = ALPHA * (roll + gy_rad * dt) + (1 - ALPHA) * accel_roll

            # Yaw from gyro (with configurable sign)
            if not is_stationary and abs(gz_cal) > GYRO_THRESHOLD_DPS:
                yaw += YAW_SIGN * gz_rad * dt

            # Check if robot is turning significantly
            is_turning = abs(gz_cal) > GYRO_THRESHOLD_DPS * 1.5

            # =============================================================
            # IMU DEAD RECKONING WITH ZUPT
            # =============================================================
            #
            # Direction: detected at motion ONSET from initial accel spike
            #   - Differential drive robot can only move along body axis
            #   - Starting forward → positive accel spike
            #   - Starting backward → negative accel spike
            #   - Direction locked until next ZUPT (stop)
            #
            # Speed: HP-filtered accel → integrate → |velocity|
            # ZUPT: stationary → reset velocity & state machine
            #

            # Forward acceleration in body frame (after bias calibration)
            if FORWARD_AXIS == 'x':
                forward_accel_raw = ax_cal * FORWARD_SIGN
            else:
                forward_accel_raw = ay_cal * FORWARD_SIGN

            # High-pass filter: removes slow drift/gravity residual
            dr_hp_prev_out = DR_HP_ALPHA * (dr_hp_prev_out + forward_accel_raw - dr_hp_prev_ax)
            dr_hp_prev_ax = forward_accel_raw
            forward_accel_hp = dr_hp_prev_out

            # Dead zone: ignore noise-level accelerations
            if abs(forward_accel_hp) < DR_ACCEL_THRESHOLD_G:
                forward_accel_hp = 0.0
                dr_velocity *= DR_VELOCITY_DAMPING
            else:
                dr_velocity += forward_accel_hp * 9.81 * dt

            # Clamp speed to physically reasonable range
            dr_velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, dr_velocity))

            # --- Direction state machine ---
            # STOPPED: waiting for motion
            # ONSET:   accumulating initial accel spike to determine direction
            # MOVING:  direction locked, using |velocity| only
            #
            # The first few samples after motion starts carry the clearest
            # directional signal — the impulse that gets the robot moving.
            # After that, accel oscillates (constant speed → ~0, decel → flips sign).

            if is_stationary:
                # ZUPT: reset everything
                dr_motion_state = 'STOPPED'
                dr_velocity = 0.0
                dr_onset_accum = 0.0
                dr_onset_count = 0

            elif dr_motion_state == 'STOPPED':
                # Transition: just left stationary
                if abs(forward_accel_hp) > DR_ACCEL_THRESHOLD_G:
                    dr_motion_state = 'ONSET'
                    dr_onset_accum = forward_accel_hp
                    dr_onset_count = 1

            elif dr_motion_state == 'ONSET':
                # Accumulate initial accel samples for direction
                dr_onset_accum += forward_accel_hp
                dr_onset_count += 1
                if dr_onset_count >= DR_ONSET_SAMPLES:
                    # Lock direction from the onset impulse
                    motion_direction = 1 if dr_onset_accum > 0 else -1
                    dr_motion_state = 'MOVING'

            # In MOVING state: direction stays locked until next ZUPT

            # Combine: magnitude from integration, sign from onset detection
            motion_velocity = abs(dr_velocity) * motion_direction

            # Update position based on velocity and heading
            if abs(motion_velocity) > 0.002:
                velocity_x = motion_velocity * math.cos(yaw)
                velocity_y = motion_velocity * math.sin(yaw)
                position[0] += velocity_x * dt
                position[1] += velocity_y * dt

            # Update 2D history with robot timestamps
            # Set first_timestamp on first sample after calibration
            if first_timestamp is None:
                first_timestamp = pico_ts_us

            # Convert timestamp to seconds relative to start
            time_s = (pico_ts_us - first_timestamp) / 1_000_000.0
            if time_s < 0:  # Handle wraparound
                time_s += (2**30) / 1_000_000.0
            time_buffer.append(time_s)

            # All chart values: sensors + motor + debug data from robot
            chart_values = [
                ax, ay, az, gx, gy, gz, ultrasonic_cm,
                robot_motor_l, robot_motor_r, robot_bumps, robot_vib
            ]
            for buf, val in zip(buffers, chart_values):
                buf.append(val)

            # Speed/position estimation (cm/s, cm)
            forward_accel_g = ax_cal if FORWARD_AXIS == 'x' else ay_cal
            forward_accel_g *= FORWARD_SIGN
            accel_cm_s2 = forward_accel_g * IMU_ACCEL_TO_MS2 * 100.0

            accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
            accel_mag_buffer.append(accel_mag)
            accel_mag_time_buffer.append(time_s)

            if not speed_est_enabled:
                speed_us_cm_s *= 0.98
                speed_imu_cm_s *= 0.98
                speed_fused_cm_s *= 0.98
                speed_bump_cm_s *= 0.98
                speed_us_buffer.append(speed_us_cm_s)
                speed_imu_buffer.append(speed_imu_cm_s)
                speed_fused_buffer.append(speed_fused_cm_s)
                speed_bump_buffer.append(speed_bump_cm_s)
                pos_us_buffer.append(pos_us_cm)
                pos_imu_buffer.append(pos_imu_cm)
                pos_fused_buffer.append(pos_fused_cm)
                pos_bump_buffer.append(pos_bump_cm)
                speed_err_imu_buffer.append(0.0)
                speed_err_fused_buffer.append(0.0)
                speed_err_bump_buffer.append(0.0)
                if is_recording:
                    recording_buffer.append({
                        'time_s': time_s,
                        'ts_us': pico_ts_us,
                        'ax': ax, 'ay': ay, 'az': az,
                        'gx': gx, 'gy': gy, 'gz': gz,
                        'us': ultrasonic_cm,
                        'motor_l': robot_motor_l, 'motor_r': robot_motor_r,
                        'bumps': robot_bumps, 'vib': robot_vib,
                        'heading': robot_heading, 'drift': robot_drift_corr,
                        'speed_us': speed_us_cm_s,
                        'speed_imu': speed_imu_cm_s,
                        'speed_fused': speed_fused_cm_s,
                        'speed_bump': speed_bump_cm_s,
                        'pos_us': pos_us_cm,
                        'pos_imu': pos_imu_cm,
                        'pos_fused': pos_fused_cm,
                        'pos_bump': pos_bump_cm,
                        'bump_per_cm': calib_bump_cm_avg if calib_bump_cm_avg else -1,
                        'bump_method': bump_method,
                        'bump_freq_hz': 0.0,
                        'bump_var': 0.0,
                        'var_slope': calib_var_slope if calib_var_slope is not None else -1,
                        'var_intercept': calib_var_intercept if calib_var_intercept is not None else -1,
                        'err_imu': 0.0,
                        'err_fused': 0.0,
                        'err_bump': 0.0,
                    })
                continue

            if is_stationary:
                speed_imu_cm_s *= 0.98
            else:
                speed_imu_cm_s += accel_cm_s2 * dt

            us_speed_valid = False
            if ultrasonic_cm > 0:
                if last_us_cm is not None and last_us_time_s is not None:
                    dt_us = time_s - last_us_time_s
                    if dt_us >= US_SPEED_MIN_DT:
                        us_delta = ultrasonic_cm - last_us_cm
                        speed_us_cm_s = -us_delta / dt_us
                        if abs(speed_us_cm_s) <= US_SPEED_MAX_CM_S:
                            us_speed_valid = True
                        last_us_jump_bad = abs(us_delta) > US_JUMP_MAX_CM
                last_us_cm = ultrasonic_cm
                last_us_time_s = time_s
            if not us_speed_valid:
                speed_us_cm_s *= 0.95

            pred_speed = speed_fused_cm_s + accel_cm_s2 * dt
            if us_speed_valid:
                speed_fused_cm_s = SPEED_FUSION_ALPHA * pred_speed + (1 - SPEED_FUSION_ALPHA) * speed_us_cm_s
            else:
                speed_fused_cm_s = pred_speed * 0.98
            if is_stationary and not us_speed_valid:
                speed_fused_cm_s *= 0.98

            cmd_avg = 0.5 * (cmd_left_speed + cmd_right_speed)
            if us_speed_valid and abs(speed_us_cm_s) > 0.5:
                speed_dir_sign = 1 if speed_us_cm_s > 0 else -1
            elif abs(cmd_avg) > 10:
                speed_dir_sign = 1 if cmd_avg > 0 else -1
            elif abs(forward_accel_g) > 0.05:
                speed_dir_sign = 1 if forward_accel_g > 0 else -1

            vib_freq_hz = 0.0
            if len(accel_mag_buffer) >= 30 and len(accel_mag_time_buffer) >= 2:
                duration_s = accel_mag_time_buffer[-1] - accel_mag_time_buffer[0]
                if duration_s > 0:
                    mean_mag = sum(accel_mag_buffer) / len(accel_mag_buffer)
                    crossings = 0
                    above = accel_mag_buffer[0] > mean_mag
                    for v in list(accel_mag_buffer)[1:]:
                        now_above = v > mean_mag
                        if now_above != above:
                            crossings += 1
                            above = now_above
                    vib_freq_hz = (crossings / 2) / duration_s
                    if vib_freq_hz < bump_band_min_hz or vib_freq_hz > bump_band_max_hz:
                        vib_freq_hz = 0.0

            if time_s - last_fft_update_s > 0.4 and len(accel_mag_buffer) >= 128:
                duration_s = accel_mag_time_buffer[-1] - accel_mag_time_buffer[0]
                if duration_s > 0:
                    sample_rate = (len(accel_mag_buffer) - 1) / duration_s
                    try:
                        import numpy as _np
                        signal = _np.array(accel_mag_buffer, dtype=float)
                        signal -= _np.mean(signal)
                        window = _np.hanning(len(signal))
                        fft_vals = _np.fft.rfft(signal * window)
                        freqs = _np.fft.rfftfreq(len(signal), d=1.0 / sample_rate)
                        mags = _np.abs(fft_vals)
                        band_mask = (freqs >= bump_band_min_hz) & (freqs <= bump_band_max_hz)
                        if band_mask.any():
                            band_freqs = freqs[band_mask]
                            band_mags = mags[band_mask]
                            fft_peak_hz = float(band_freqs[int(_np.argmax(band_mags))])
                        else:
                            fft_peak_hz = 0.0
                    except Exception:
                        fft_peak_hz = 0.0
                last_fft_update_s = time_s

            if calib_active and us_speed_valid and not last_us_jump_bad and speed_us_cm_s >= BUMP_CALIB_MIN_SPEED and vib_freq_hz > 0:
                bump_cm = vib_freq_hz / speed_us_cm_s
                calib_bump_cm_samples.append(bump_cm)
                calib_bump_cm_avg = sum(calib_bump_cm_samples) / len(calib_bump_cm_samples)
                main_window.lbl_bump_cm.setText(f"Bump/cm: {bump_cm:.4f}")
                main_window.lbl_bump_cm_avg.setText(f"Avg: {calib_bump_cm_avg:.4f} ({len(calib_bump_cm_samples)})")
                main_window.lbl_bump_cm_warn.setText("")
            elif calib_active and last_us_jump_bad:
                calib_invalid_count += 1
                main_window.lbl_bump_cm_warn.setText(f"US jump rejected ({calib_invalid_count})")

            variance_val = 0.0
            if len(accel_mag_buffer) >= 20:
                mean_mag = sum(accel_mag_buffer) / len(accel_mag_buffer)
                variance_val = sum((x - mean_mag) ** 2 for x in accel_mag_buffer) / len(accel_mag_buffer)

            if calib_active and us_speed_valid and not last_us_jump_bad and speed_us_cm_s >= BUMP_CALIB_MIN_SPEED:
                calib_var_pairs.append((variance_val, speed_us_cm_s))
                if len(calib_var_pairs) >= 5:
                    xs = [p[0] for p in calib_var_pairs]
                    ys = [p[1] for p in calib_var_pairs]
                    x_mean = sum(xs) / len(xs)
                    y_mean = sum(ys) / len(ys)
                    denom = sum((x - x_mean) ** 2 for x in xs)
                    if denom > 0:
                        calib_var_slope = sum((x - x_mean) * (y - y_mean) for x, y in calib_var_pairs) / denom
                        calib_var_intercept = y_mean - calib_var_slope * x_mean
                        main_window.lbl_var_fit.setText(
                            f"Var fit: {calib_var_slope:.3f}*var+{calib_var_intercept:.2f}"
                        )

            bump_per_cm = calib_bump_cm_avg
            if bump_method == "Variance":
                if calib_var_slope is not None and calib_var_intercept is not None:
                    speed_bump_cm_s = calib_var_slope * variance_val + calib_var_intercept
                else:
                    speed_bump_cm_s *= 0.95
            else:
                method_freq = vib_freq_hz if bump_method == "Zero-Cross" else fft_peak_hz
                if bump_per_cm and bump_per_cm > 0 and method_freq > 0:
                    speed_bump_cm_s = speed_dir_sign * (method_freq / bump_per_cm)
                else:
                    speed_bump_cm_s *= 0.95
            if is_stationary:
                speed_bump_cm_s *= 0.98

            if not us_speed_valid and abs(speed_fused_cm_s) < 0.5:
                speed_fused_cm_s *= 0.98

            pos_us_cm += speed_us_cm_s * dt
            pos_imu_cm += speed_imu_cm_s * dt
            pos_fused_cm += speed_fused_cm_s * dt
            pos_bump_cm += speed_bump_cm_s * dt

            speed_us_buffer.append(speed_us_cm_s)
            speed_imu_buffer.append(speed_imu_cm_s)
            speed_fused_buffer.append(speed_fused_cm_s)
            speed_bump_buffer.append(speed_bump_cm_s)
            pos_us_buffer.append(pos_us_cm)
            pos_imu_buffer.append(pos_imu_cm)
            pos_fused_buffer.append(pos_fused_cm)
            pos_bump_buffer.append(pos_bump_cm)
            if us_speed_valid:
                speed_err_imu_buffer.append(speed_imu_cm_s - speed_us_cm_s)
                speed_err_fused_buffer.append(speed_fused_cm_s - speed_us_cm_s)
                speed_err_bump_buffer.append(speed_bump_cm_s - speed_us_cm_s)
            else:
                speed_err_imu_buffer.append(0.0)
                speed_err_fused_buffer.append(0.0)
                speed_err_bump_buffer.append(0.0)

            # Record data if recording is active
            if is_recording:
                recording_buffer.append({
                    'time_s': time_s,
                    'ts_us': pico_ts_us,
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'us': ultrasonic_cm,
                    'motor_l': robot_motor_l, 'motor_r': robot_motor_r,
                    'bumps': robot_bumps, 'vib': robot_vib,
                    'heading': robot_heading, 'drift': robot_drift_corr,
                    'speed_us': speed_us_cm_s,
                    'speed_imu': speed_imu_cm_s,
                    'speed_fused': speed_fused_cm_s,
                    'speed_bump': speed_bump_cm_s,
                    'pos_us': pos_us_cm,
                    'pos_imu': pos_imu_cm,
                    'pos_fused': pos_fused_cm,
                    'pos_bump': pos_bump_cm,
                    'bump_per_cm': calib_bump_cm_avg if calib_bump_cm_avg else -1,
                    'bump_method': bump_method,
                    'bump_freq_hz': fft_peak_hz if bump_method == "FFT" else vib_freq_hz,
                    'bump_var': variance_val,
                    'var_slope': calib_var_slope if calib_var_slope is not None else -1,
                    'var_intercept': calib_var_intercept if calib_var_intercept is not None else -1,
                    'err_imu': speed_imu_cm_s - speed_us_cm_s if us_speed_valid else 0.0,
                    'err_fused': speed_fused_cm_s - speed_us_cm_s if us_speed_valid else 0.0,
                    'err_bump': speed_bump_cm_s - speed_us_cm_s if us_speed_valid else 0.0,
                })

            # AI auto-labeled data collection
            if ai_collect_enabled:
                # Determine label from keyboard state
                if key_state['forward']:
                    current_motion_label = 'FORWARD'
                elif key_state['backward']:
                    current_motion_label = 'BACKWARD'
                elif key_state['left']:
                    current_motion_label = 'TURN_LEFT'
                elif key_state['right']:
                    current_motion_label = 'TURN_RIGHT'
                else:
                    current_motion_label = 'STOPPED'

                # Collect sample with auto-tag
                sample = {
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'us': ultrasonic_cm,
                    'motor_l': robot_motor_l, 'motor_r': robot_motor_r,
                    'bumps': robot_bumps, 'vib': robot_vib
                }
                ai_data_collection[current_motion_label].append(sample)

            frame_count += 1

        # ======================================================================
        # UPDATE VISUALIZATIONS
        # ======================================================================

        if not is_calibrated:
            # Skip visualization but still run UDP diagnostics below
            now = time.time()
            if now - last_udp_log_time >= 1.0:
                last_udp_log_time = now
                print(f"[UDP] rx={udp_rx_count} calib={len(calibration_samples)}/{CALIBRATION_COUNT} bind={sock.getsockname()}")
                udp_rx_count = 0
            return

        # 2D plots with robot timestamps as X-axis
        t = list(time_buffer)
        if t:
            window_s = main_window.spin_view_window.value()
            if main_window.chk_follow_live.isChecked():
                view_end = t[-1]
                view_start = max(t[0], view_end - window_s)
            else:
                max_offset = max(0.0, (t[-1] - t[0]) - window_s)
                offset = (main_window.slider_view_offset.value() / 1000.0) * max_offset
                view_start = t[0] + offset
                view_end = view_start + window_s
            main_window.view_start_s = view_start
            main_window.view_end_s = view_end
            main_window.lbl_view_range.setText(f"Range: {view_start:.2f}s – {view_end:.2f}s")

            idxs = []
            for i, tv in enumerate(t):
                if view_start <= tv <= view_end:
                    idxs.append(i)
            max_idx = min(len(t), min(len(b) for b in buffers)) if buffers else 0
            if not idxs:
                idxs = list(range(max_idx))
            else:
                idxs = [i for i in idxs if i < max_idx]
            t_view = [t[i] for i in idxs]

            for i, (c, buf, p) in enumerate(zip(curves, buffers, plots)):
                visible = main_window.chart_checks[i].isChecked()
                p.setVisible(visible)
                if not visible or not idxs:
                    c.setData(x=[], y=[])
                    continue
                y_view = [buf[j] for j in idxs]
                c.setData(x=t_view, y=y_view)

            def _slice(buf):
                return [buf[j] for j in idxs] if idxs else []

            main_window.speed_curve_us.setData(x=t_view, y=_slice(speed_us_buffer))
            main_window.speed_curve_imu.setData(x=t_view, y=_slice(speed_imu_buffer))
            main_window.speed_curve_fused.setData(x=t_view, y=_slice(speed_fused_buffer))
            main_window.speed_curve_bump.setData(x=t_view, y=_slice(speed_bump_buffer))
            main_window.pos_curve_us.setData(x=t_view, y=_slice(pos_us_buffer))
            main_window.pos_curve_imu.setData(x=t_view, y=_slice(pos_imu_buffer))
            main_window.pos_curve_fused.setData(x=t_view, y=_slice(pos_fused_buffer))
            main_window.pos_curve_bump.setData(x=t_view, y=_slice(pos_bump_buffer))
            main_window.err_curve_imu.setData(x=t_view, y=_slice(speed_err_imu_buffer))
            main_window.err_curve_fused.setData(x=t_view, y=_slice(speed_err_fused_buffer))
            main_window.err_curve_bump.setData(x=t_view, y=_slice(speed_err_bump_buffer))

        # Robot orientation + position
        R = rotation_matrix(pitch, roll, yaw)
        rotated_verts = (R @ box_verts.T).T
        robot_pos = position * POSITION_SCALE
        translated_verts = rotated_verts + robot_pos
        robot_mesh.setMeshData(vertexes=translated_verts, faces=box_faces, faceColors=box_colors)

        # Forward arrow
        forward = R @ np.array([1, 0, 0])
        arrow_line.setData(pos=np.array([robot_pos, robot_pos + forward]))

        # 3D trail
        trail_points_3d.append(robot_pos.copy())
        if len(trail_points_3d) > 1:
            robot_trail.setData(pos=np.array(trail_points_3d))

        # Path view (2D top-down)
        path_pos = position[:2] * POSITION_SCALE
        path_points_x.append(path_pos[0])
        path_points_y.append(path_pos[1])

        # Update path trail
        if len(path_points_x) > 1:
            path_curve.setData(x=list(path_points_x), y=list(path_points_y))

        # Update robot marker position
        robot_marker_2d.setData(x=[path_pos[0]], y=[path_pos[1]])

        # Heading arrow on path (2D)
        heading_len = 1.5
        heading_end_x = path_pos[0] + math.cos(yaw) * heading_len
        heading_end_y = path_pos[1] + math.sin(yaw) * heading_len
        heading_line.setData(x=[path_pos[0], heading_end_x], y=[path_pos[1], heading_end_y])

        # Ultrasonic cone visualization (2D)
        if ultrasonic_cm > 0:
            # Draw cone showing ultrasonic beam (15 degree spread)
            dist_scaled = min(ultrasonic_cm / 100.0 * POSITION_SCALE * 2, 5.0)
            cone_angle = math.radians(15)

            # Cone points (triangle: robot -> left -> tip -> right -> robot)
            cone_x = [
                path_pos[0],
                path_pos[0] + math.cos(yaw - cone_angle) * dist_scaled,
                path_pos[0] + math.cos(yaw) * dist_scaled,
                path_pos[0] + math.cos(yaw + cone_angle) * dist_scaled,
                path_pos[0]
            ]
            cone_y = [
                path_pos[1],
                path_pos[1] + math.sin(yaw - cone_angle) * dist_scaled,
                path_pos[1] + math.sin(yaw) * dist_scaled,
                path_pos[1] + math.sin(yaw + cone_angle) * dist_scaled,
                path_pos[1]
            ]
            ultrasonic_cone_2d.setData(x=cone_x, y=cone_y)

            # Obstacle point at tip
            obstacle_x = path_pos[0] + math.cos(yaw) * dist_scaled
            obstacle_y = path_pos[1] + math.sin(yaw) * dist_scaled
            ultrasonic_marker.setData(x=[obstacle_x], y=[obstacle_y])

            # Update text
            ultrasonic_text.setText(f"US: {ultrasonic_cm:.0f}cm")
        else:
            # No reading - clear cone
            ultrasonic_cone_2d.setData(x=[], y=[])
            ultrasonic_marker.setData(x=[], y=[])
            ultrasonic_text.setText("US: --")

        # Status output with all parameters
        if packet_count % 40 == 0:
            heading_deg = math.degrees(yaw)
            avg_cmd = (cmd_left_speed + cmd_right_speed) / 2.0
            mode = "CMD" if abs(avg_cmd) > 10 else ("EXT" if motion_active else "---")

            # Calculate vibration (variance) for each axis
            if len(accel_x_raw_buffer) >= 20:
                vib_x = np.var(list(accel_x_raw_buffer)) * 1000  # Scale up for readability
                vib_y = np.var(list(accel_y_raw_buffer)) * 1000
                vib_z = np.var(list(accel_z_raw_buffer)) * 1000
                vib_total = vib_x + vib_y + vib_z
            else:
                vib_x = vib_y = vib_z = vib_total = 0

            # AI prediction
            ai_pred_str = "OFF"
            if ai_enabled:
                features = ai_feature_extractor.extract_features()
                ai_pred = ai_classifier.predict(features)
                ai_pred_str = AI_CLASSES[ai_pred]

            # Build comprehensive status line with robot debug data
            vel_cms = motion_velocity * 100  # m/s to cm/s
            us_str = f"{ultrasonic_cm:3.0f}" if ultrasonic_cm > 0 else " --"

            # Use robot-provided motor speeds if available, else command speeds
            ml = robot_motor_l if robot_motor_l != 0 else cmd_left_speed
            mr = robot_motor_r if robot_motor_r != 0 else cmd_right_speed

            # Build status indicators
            rec_ind = "⏺" if is_recording else ""
            ai_ind = "🤖" if ai_collect_enabled else ""
            indicators = f"{rec_ind}{ai_ind}" if (rec_ind or ai_ind) else ""

            if training_mode:
                stats = get_training_stats()
                total = sum(stats)
                status = (f"{indicators}[REC:{AI_CLASSES[training_label]:4s}] "
                          f"X:{position[0]:+5.2f} Y:{position[1]:+5.2f} "
                          f"H:{heading_deg:+6.1f}° "
                          f"M:{ml:+4d}/{mr:+4d} "
                          f"V:{robot_vib:5.1f} "
                          f"US:{us_str}cm "
                          f"N:{total:4d}")
            else:
                # Show robot debug data with recording/AI indicators
                status = (f"{indicators}[{mode:3s}] "
                          f"Pos:({position[0]:+.2f},{position[1]:+.2f}) "
                          f"H:{robot_heading:+6.1f}° "
                          f"M:{ml:+4d}/{mr:+4d} "
                          f"V:{robot_vib:5.1f} B:{robot_bumps:4d} "
                          f"D:{robot_drift_corr:+5.1f} "
                          f"US:{us_str}cm")

            # Update status bar in main window (clean display)
            status_clean = status.replace('\r', '').strip()
            main_window.update_status(status_clean)

    except BlockingIOError:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    now = time.time()
    if now - last_udp_log_time >= 1.0:
        last_udp_log_time = now
        buf_len = len(buffers[0]) if buffers else 0
        print(f"[UDP] rx={udp_rx_count} pause={pause_updates} bind={sock.getsockname()} t={len(time_buffer)} b0={buf_len}")
        udp_rx_count = 0


# ==============================================================================
# KEYBOARD EVENT HANDLER
# ==============================================================================

class KeyboardHandler(QtCore.QObject):
    def __init__(self):
        super().__init__()

    def eventFilter(self, obj, event):
        global filters_enabled, ai_enabled, training_mode, training_sample_count
        global training_label, training_data

        if event.type() == QtCore.QEvent.KeyPress:
            key = event.key()

            # Movement: Arrow keys OR WASD
            if key in (Qt.Key_Up, Qt.Key_W):
                key_state['forward'] = True
                key_state['backward'] = False
            elif key in (Qt.Key_Down, Qt.Key_S):
                key_state['backward'] = True
                key_state['forward'] = False
            elif key in (Qt.Key_Left, Qt.Key_A):
                key_state['left'] = True
                key_state['right'] = False
            elif key in (Qt.Key_Right, Qt.Key_D):
                key_state['right'] = True
                key_state['left'] = False
            elif key == Qt.Key_Space:
                # Stop all
                key_state['forward'] = False
                key_state['backward'] = False
                key_state['left'] = False
                key_state['right'] = False
            elif key == Qt.Key_R:
                reset_position()
            elif key == Qt.Key_F:
                filters_enabled = not filters_enabled
                print(f"\n>>> Filters: {'ON' if filters_enabled else 'OFF'}")
            elif key == Qt.Key_E:
                export_csv()
            elif key == Qt.Key_I:  # Changed from A to I for AI toggle
                ai_enabled = not ai_enabled
                print(f"\n>>> AI Detection: {'ON' if ai_enabled else 'OFF'}")
            elif key == Qt.Key_T:
                training_mode = not training_mode
                if training_mode:
                    training_sample_count = 0
                    print(f"\n>>> TRAINING MODE ON - Label: {AI_CLASSES[training_label]} (0-5 to change)")
                else:
                    print(f"\n>>> TRAINING MODE OFF - Collected {len(training_data)} samples")
            elif key in (Qt.Key_0, Qt.Key_1, Qt.Key_2, Qt.Key_3, Qt.Key_4, Qt.Key_5):
                training_label = key - Qt.Key_0
                print(f"\n>>> Label: {training_label} = {AI_CLASSES[training_label]}")
            elif key == Qt.Key_X:
                export_training_data()
            elif key == Qt.Key_Z:
                training_data = []
                print("\n>>> Training data cleared!")
            elif key == Qt.Key_C:
                # Send calibration command to robot
                try:
                    control_sock.sendto(b'C', (ROBOT_IP, ROBOT_CONTROL_PORT))
                    print("\n>>> Sent CALIBRATE command to robot")
                except:
                    print("\n>>> No robot connected")
            elif key == Qt.Key_V:
                # Send verify command to robot
                try:
                    control_sock.sendto(b'V', (ROBOT_IP, ROBOT_CONTROL_PORT))
                    print("\n>>> Sent VERIFY command to robot")
                except:
                    print("\n>>> No robot connected")
            elif key == Qt.Key_H:
                print("\n")  # New line before help
                print_help()
            elif key in (Qt.Key_Q, Qt.Key_Escape):
                # Send quit command to robot before closing
                try:
                    control_sock.sendto(b'Q', (ROBOT_IP, ROBOT_CONTROL_PORT))
                    print("\n>>> Sent QUIT to robot, closing...")
                except:
                    pass
                app.quit()

            send_control_command()
            return True

        elif event.type() == QtCore.QEvent.KeyRelease:
            key = event.key()

            # Movement release: Arrow keys OR WASD
            if key in (Qt.Key_Up, Qt.Key_W):
                key_state['forward'] = False
            elif key in (Qt.Key_Down, Qt.Key_S):
                key_state['backward'] = False
            elif key in (Qt.Key_Left, Qt.Key_A):
                key_state['left'] = False
            elif key in (Qt.Key_Right, Qt.Key_D):
                key_state['right'] = False

            send_control_command()
            return True

        return False


# Install keyboard handler
keyboard_handler = KeyboardHandler()
app.installEventFilter(keyboard_handler)

# ==============================================================================
# MAIN
# ==============================================================================

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1)

def print_help():
    """Print usage instructions."""
    print()
    print("="*60)
    print("PICOBOT VISUALIZER v2.0")
    print("="*60)
    print()
    print("MOVEMENT:")
    print("  W / Arrow Up      - Drive forward")
    print("  S / Arrow Down    - Drive backward")
    print("  A / Arrow Left    - Turn left")
    print("  D / Arrow Right   - Turn right")
    print("  Space             - Stop")
    print()
    print("GENERAL:")
    print("  R                 - Reset position to origin")
    print("  F                 - Toggle low-pass filters")
    print("  E                 - Export IMU data to CSV")
    print("  H                 - Show this help")
    print("  Q / Esc           - Quit (sends stop to robot)")
    print()
    print("WORKFLOWS:")
    print("  Toolbar buttons   - Rec+Export, ML Flow, Vib Report")
    print()
    print("CALIBRATION (vibration lab):")
    print("  C                 - Run calibration (fwd+bwd 30cm)")
    print("  V                 - Run verification test")
    print()
    print("AI/ML TRAINING:")
    print("  I                 - Toggle AI motion detection")
    print("  T                 - Toggle training mode")
    print("  0-5               - Select training label:")
    print("                      0=STOP 1=FWD 2=BWD 3=T_L 4=T_R 5=PUSH")
    print("  X                 - Export training data to CSV")
    print("  Z                 - Clear training data")
    print()
    print("="*60)
    print()


if __name__ == '__main__':
    print_help()
    print("Waiting for data from robot...")
    print()
    sys.exit(app.exec_())
