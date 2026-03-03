#!/usr/bin/env python3
"""
==============================================================================
HOST DATA COLLECTOR - Simple Educational Data Collection
==============================================================================

This is a simplified host application for collecting sensor data from the
PicoBot. It pairs with pico_collector.py on the robot.

PURPOSE:
--------
Collect sensor data with manual keyboard control for later analysis.
This is the "data collection" phase of the workflow:

  1. COLLECT: Run this script, drive robot, record data
  2. ANALYZE: Use analyzers (bump_detector, fft_analyzer, etc.)
  3. APPLY: Use insights to improve robot control algorithms

FEATURES:
---------
  - Real-time sensor display
  - Keyboard control (WASD/arrows)
  - Recording start/stop
  - CSV export for analysis
  - Simple PyQt5 UI with charts

USAGE:
------
  1. Upload pico_collector.py to robot
  2. Run: python host_collector.py
  3. Drive robot with WASD or arrow keys
  4. Press R to start/stop recording
  5. Press E to export recorded data
  6. Analyze with: python analyzers/fft_analyzer.py data/recording.csv

CONTROLS:
---------
  W / Up      - Forward
  S / Down    - Backward
  A / Left    - Turn left
  D / Right   - Turn right
  Space       - Stop
  R           - Start/Stop recording
  E           - Export to CSV
  Q / Esc     - Quit

==============================================================================
"""

import socket
import struct
import sys
import os
import time
import datetime
from collections import deque

# PyQt5 for GUI
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QTimer

# pyqtgraph for real-time charts
import pyqtgraph as pg

# NumPy for FFT
import numpy as np

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Network settings
ROBOT_PORT_DATA = 5005      # Receive sensor data
ROBOT_PORT_COMMAND = 5006   # Send motor commands

# Motor speed settings
MOTOR_SPEED = 70            # Default motor speed (0-100)
TURN_SPEED = 50             # Speed for turning

# Display settings
CHART_HISTORY = 500         # Number of samples to show in charts
SAMPLE_RATE = 500           # Expected samples per second

# Data directory
DATA_DIR = os.path.join(os.path.dirname(__file__), 'data')
os.makedirs(DATA_DIR, exist_ok=True)


# ==============================================================================
# PACKET PARSING
# ==============================================================================

# DataLogger packet flags
FLAG_MOTORS = 0x01
FLAG_ULTRASONIC = 0x02
FLAG_OPTO = 0x04
FLAG_IMU = 0x08
FLAG_VARS = 0x10
FLAG_AUDIO = 0x20  # Audio ADC samples (8-bit, 8kHz)

# Var schema storage (populated from schema packets)
var_schema = []  # List of (name, type_char, struct_size)

def parse_sensor_packet(data):
    """
    Parse sensor data packet from robot.

    Supports two formats:
      1. Legacy (pico_collector.py): type=0x01, batched samples
      2. DataLogger (picobot.py): magic=0xABCD, version=0x02

    Args:
        data: Raw bytes received from robot

    Returns:
        list: List of sample dictionaries, or empty list if invalid
    """
    if len(data) < 4:
        return []

    # Check for DataLogger format (magic bytes 0xAB 0xCD)
    if data[0] == 0xAB and data[1] == 0xCD:
        return _parse_datalogger_packet(data)

    # Legacy format
    return _parse_legacy_packet(data)


def _parse_legacy_packet(data):
    """Parse legacy pico_collector.py format."""
    if len(data) < 2:
        return []

    packet_type = data[0]
    sample_count = data[1]

    if packet_type != 0x01:
        return []

    SAMPLE_SIZE = 36
    expected_size = 2 + (sample_count * SAMPLE_SIZE)

    if len(data) < expected_size:
        return []

    samples = []
    offset = 2

    for _ in range(sample_count):
        sample_data = data[offset:offset + SAMPLE_SIZE]
        try:
            values = struct.unpack('<Q7f', sample_data)
            sample = {
                'timestamp_us': values[0],
                'ax': values[1],
                'ay': values[2],
                'az': values[3],
                'gx': values[4],
                'gy': values[5],
                'gz': values[6],
                'us': values[7],
            }
            samples.append(sample)
        except struct.error:
            break
        offset += SAMPLE_SIZE

    return samples


def _parse_datalogger_packet(data):
    """
    Parse DataLogger format (picobot.py).

    Header (4 bytes): magic(2) + version(1) + flags(1)
    Payload varies based on flags.
    """
    global var_schema

    version = data[2]
    flags = data[3]

    # Schema packet (version 0x00)
    if version == 0x00:
        _parse_schema_packet(data)
        return []

    # Data packet (version 0x02)
    if version != 0x02:
        return []

    sample = {}
    offset = 4

    # Timestamp (uint32, µs)
    if offset + 4 > len(data):
        return []
    sample['timestamp_us'] = struct.unpack_from('<I', data, offset)[0]
    offset += 4

    # Motors (2x uint8)
    if flags & FLAG_MOTORS:
        if offset + 2 > len(data):
            return [sample]
        sample['motor_l'] = data[offset]
        sample['motor_r'] = data[offset + 1]
        offset += 2

    # Ultrasonic (uint16, cm * 10)
    if flags & FLAG_ULTRASONIC:
        if offset + 2 > len(data):
            return [sample]
        us_raw = struct.unpack_from('<H', data, offset)[0]
        if us_raw == 0xFFFF:
            sample['us'] = -1.0
        else:
            sample['us'] = us_raw / 10.0
        offset += 2

    # Opto (uint8 bitmask)
    if flags & FLAG_OPTO:
        if offset + 1 > len(data):
            return [sample]
        opto = data[offset]
        sample['opto_x1'] = 1 if (opto & 0x01) else 0
        sample['opto_x2'] = 1 if (opto & 0x02) else 0
        sample['opto_x3'] = 1 if (opto & 0x04) else 0
        sample['opto_x4'] = 1 if (opto & 0x08) else 0
        offset += 1

    # IMU (uint32 hw_ts + 6x int16 raw)
    if flags & FLAG_IMU:
        if offset + 16 > len(data):
            return [sample]
        imu_data = struct.unpack_from('<I6h', data, offset)
        sample['imu_ts_us'] = imu_data[0]
        # Convert raw to physical units
        sample['ax'] = imu_data[1] / 16384.0  # g
        sample['ay'] = imu_data[2] / 16384.0
        sample['az'] = imu_data[3] / 16384.0
        sample['gx'] = imu_data[4] / 131.2    # °/s
        sample['gy'] = imu_data[5] / 131.2
        sample['gz'] = imu_data[6] / 131.2
        offset += 16

    # Vars (uint8 count + packed values)
    if flags & FLAG_VARS:
        if offset + 1 > len(data):
            return [sample]
        var_count = data[offset]
        offset += 1

        # Parse using schema if available
        for i, (name, type_char, size) in enumerate(var_schema[:var_count]):
            if offset + size > len(data):
                break
            try:
                value = struct.unpack_from('<' + type_char, data, offset)[0]
                sample[name] = value
            except:
                pass
            offset += size

    # Audio (uint16 count + 8-bit samples)
    if flags & FLAG_AUDIO:
        if offset + 2 > len(data):
            return [sample]
        audio_count = struct.unpack_from('<H', data, offset)[0]
        offset += 2

        if offset + audio_count <= len(data):
            # Store as list of 8-bit samples (0-255, centered at ~128)
            sample['audio'] = list(data[offset:offset + audio_count])
            sample['audio_count'] = audio_count
            offset += audio_count

    return [sample]


def _parse_schema_packet(data):
    """
    Parse schema packet describing var names and types.

    Format: magic(2) + version=0x00(1) + count(1) + [name_len(1) + name + type(1)]...
    """
    global var_schema

    if len(data) < 4:
        return

    var_count = data[3]
    var_schema = []
    offset = 4

    type_sizes = {'B': 1, 'b': 1, 'H': 2, 'h': 2, 'I': 4, 'i': 4, 'f': 4}

    for _ in range(var_count):
        if offset >= len(data):
            break

        name_len = data[offset]
        offset += 1

        if offset + name_len + 1 > len(data):
            break

        name = data[offset:offset + name_len].decode('utf-8', errors='ignore')
        offset += name_len

        type_char = chr(data[offset])
        offset += 1

        size = type_sizes.get(type_char, 4)
        var_schema.append((name, type_char, size))

    print(f"Schema received: {[n for n, _, _ in var_schema]}")


# ==============================================================================
# MAIN WINDOW
# ==============================================================================

class CollectorWindow(QtWidgets.QMainWindow):
    """
    Main window for data collection.

    Layout:
      +----------------------------------+
      |  Status Bar (connection, etc.)   |
      +----------------------------------+
      |     Real-time Charts             |
      |  (accelerometer, gyroscope)      |
      +----------------------------------+
      |  Info: Recording status, samples |
      +----------------------------------+
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PicoBot Data Collector")
        self.setGeometry(100, 100, 1000, 700)

        # State
        self.robot_ip = None
        self.is_recording = False
        self.recording_buffer = []
        self.first_timestamp = None

        # Data buffers for charts
        self.time_buffer = deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY)
        self.data_buffers = {
            # Motors
            'motor_l': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'motor_r': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            # Sensors
            'us': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            # IMU (optional)
            'ax': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'ay': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'az': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'gx': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'gy': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
            'gz': deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY),
        }

        # Audio buffer (larger - 8kHz sample rate)
        AUDIO_BUFFER_SIZE = 4096  # ~0.5 seconds at 8kHz
        self.audio_buffer = deque([128] * AUDIO_BUFFER_SIZE, maxlen=AUDIO_BUFFER_SIZE)
        self.audio_time_buffer = deque([0] * AUDIO_BUFFER_SIZE, maxlen=AUDIO_BUFFER_SIZE)
        self.audio_sample_count = 0  # Continuous counter for proper timing
        self.audio_sample_rate = 16000  # Hz - must match Pico

        # Audio filtering (Pico ADC is noisy)
        self.audio_filter_enabled = True
        self.audio_dc_offset = 128.0  # Running DC estimate
        self.audio_dc_alpha = 0.001   # DC tracking speed (slow)
        self.audio_lp_alpha = 0.3     # Low-pass filter (0=heavy, 1=none)
        self.audio_last_filtered = 0.0

        # Audio recording (WAV)
        self.is_recording_audio = False
        self.audio_recording_buffer = []
        self.audio_packets_received = 0  # Debug counter

        # Dynamic buffers for vars (added when schema received)
        self.var_buffers = {}

        # Keyboard state
        self.key_state = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False,
        }

        # Setup UI
        self._setup_ui()

        # Setup network
        self._setup_network()

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update)
        self.update_timer.start(20)  # 50 Hz update

    def _setup_ui(self):
        """Create the user interface."""
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Status bar
        self.status_label = QtWidgets.QLabel("Waiting for robot connection...")
        self.status_label.setStyleSheet("font-size: 14px; padding: 5px;")
        layout.addWidget(self.status_label)

        # Charts widget
        self.charts_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.charts_widget, stretch=4)

        # Create charts
        self._create_charts()

        # Info bar
        info_layout = QtWidgets.QHBoxLayout()

        self.recording_label = QtWidgets.QLabel("Recording: OFF")
        self.recording_label.setStyleSheet("font-weight: bold;")
        info_layout.addWidget(self.recording_label)

        self.audio_rec_label = QtWidgets.QLabel("Audio: OFF")
        self.audio_rec_label.setStyleSheet("font-weight: bold;")
        info_layout.addWidget(self.audio_rec_label)

        self.samples_label = QtWidgets.QLabel("Samples: 0")
        info_layout.addWidget(self.samples_label)

        self.time_label = QtWidgets.QLabel("Time: 0.0s")
        info_layout.addWidget(self.time_label)

        # View All button - shows entire recording
        view_all_btn = QtWidgets.QPushButton("View All")
        view_all_btn.setToolTip("Zoom out to show entire timeframe (V)")
        view_all_btn.clicked.connect(self._view_all)
        info_layout.addWidget(view_all_btn)

        info_layout.addStretch()

        # Control hints
        hints = QtWidgets.QLabel("WASD: Drive | R: Record | G: Audio | E: Export | Q: Quit")
        hints.setStyleSheet("color: gray;")
        info_layout.addWidget(hints)

        layout.addLayout(info_layout)

    def _create_charts(self):
        """Create real-time sensor charts."""
        self.plots = {}
        self.curves = {}
        self.first_plot = None
        self.chart_row = 0

        # Fixed charts: motors, ultrasonic
        # (name, y_range, color, label)
        fixed_charts = [
            ('motor_l', (0, 255), 'r', 'Motor L'),
            ('motor_r', (0, 255), 'b', 'Motor R'),
            ('us', (0, 100), (255, 128, 0), 'Dist (cm)'),
        ]

        for name, y_range, color, label in fixed_charts:
            self._add_chart(name, y_range, color, label)

        # Audio waveform chart (separate - uses its own time buffer)
        # Hidden by default - press 'A' to toggle
        self.audio_plot = self.charts_widget.addPlot(row=self.chart_row, col=0)
        self.audio_plot.setLabel('left', 'Audio')
        self.audio_plot.setYRange(0, 255)
        self.audio_plot.showGrid(x=True, y=True, alpha=0.3)
        self.audio_plot.getAxis('left').setWidth(60)
        self.audio_plot.hideAxis('bottom')
        self.audio_curve = self.audio_plot.plot(pen=pg.mkPen(color='g', width=1))
        self.audio_plot.hide()  # Hidden by default
        self.chart_row += 1

        # Audio FFT spectrum chart
        self.fft_plot = self.charts_widget.addPlot(row=self.chart_row, col=0)
        self.fft_plot.setLabel('left', 'FFT (dB)')
        self.fft_plot.setLabel('bottom', 'Freq (Hz)')
        self.fft_plot.setYRange(-60, 60)
        self.fft_plot.setXRange(0, self.audio_sample_rate // 2)  # Nyquist
        self.fft_plot.showGrid(x=True, y=True, alpha=0.3)
        self.fft_plot.getAxis('left').setWidth(60)
        self.fft_curve = self.fft_plot.plot(pen=pg.mkPen(color='c', width=1))
        # Fill under curve for better visualization
        self.fft_fill = pg.FillBetweenItem(self.fft_curve, self.fft_curve, brush=(0, 255, 255, 50))
        self.fft_plot.addItem(self.fft_fill)
        self.fft_plot.hide()  # Hidden by default
        self.chart_row += 1

        # Audio charts visibility
        self.audio_charts_visible = False

        # FFT parameters
        self.fft_size = 1024  # FFT window size

    def _add_chart(self, name, y_range, color, label=None):
        """Add a chart for a data field."""
        plot = self.charts_widget.addPlot(row=self.chart_row, col=0)
        plot.setLabel('left', label or name)
        plot.setYRange(*y_range)
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.getAxis('left').setWidth(60)

        if self.first_plot is None:
            self.first_plot = plot
        else:
            plot.setXLink(self.first_plot)

        plot.hideAxis('bottom')  # Will show on last chart

        curve = plot.plot(pen=pg.mkPen(color=color, width=1))

        self.plots[name] = plot
        self.curves[name] = curve
        self.chart_row += 1

        # Ensure buffer exists
        if name not in self.data_buffers:
            self.data_buffers[name] = deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY)

    def _add_var_charts(self):
        """Add charts for schema vars (called when schema received)."""
        colors = ['c', 'm', 'y', 'g', (100, 200, 255), (255, 100, 200)]

        for i, (name, type_char, _) in enumerate(var_schema):
            if name not in self.curves:
                color = colors[i % len(colors)]
                # Guess y_range based on type
                if type_char in ('B', 'b'):
                    y_range = (0, 255) if type_char == 'B' else (-128, 127)
                elif type_char in ('H', 'h'):
                    y_range = (-1000, 1000)
                else:
                    y_range = (-100, 100)
                self._add_chart(name, y_range, color)

        # Show time axis on bottom chart
        if self.plots:
            last_plot = list(self.plots.values())[-1]
            last_plot.showAxis('bottom')
            last_plot.setLabel('bottom', 'Time (s)')

    def _setup_network(self):
        """Initialize network sockets."""
        # Receive socket (sensor data from robot)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('', ROBOT_PORT_DATA))
        self.recv_sock.setblocking(False)

        # Send socket (commands to robot)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _update(self):
        """Main update loop - called by timer."""
        # Receive sensor data
        self._receive_data()

        # Update motor commands based on keyboard
        self._send_motor_command()

        # Update charts
        self._update_charts()

    def _receive_data(self):
        """Receive and process sensor data from robot."""
        try:
            while True:
                data, addr = self.recv_sock.recvfrom(1024)

                # Remember robot IP
                if self.robot_ip is None:
                    self.robot_ip = addr[0]
                    self.status_label.setText(f"Connected to robot: {self.robot_ip}")
                    self.status_label.setStyleSheet(
                        "font-size: 14px; padding: 5px; color: green;")

                # Check for schema packet and add charts
                if len(data) >= 4 and data[0] == 0xAB and data[1] == 0xCD and data[2] == 0x00:
                    # Schema packet - will be parsed by parse_sensor_packet
                    parse_sensor_packet(data)
                    self._add_var_charts()
                    continue

                # Parse data packet
                samples = parse_sensor_packet(data)

                for sample in samples:
                    self._process_sample(sample)

        except BlockingIOError:
            pass  # No data available

    def _filter_audio_sample(self, raw_val):
        """
        Filter a single audio sample to reduce noise.

        Applies:
        1. DC offset tracking (high-pass effect)
        2. Low-pass filter for smoothing
        3. Re-centers around 128

        Args:
            raw_val: Raw 8-bit sample (0-255)

        Returns:
            Filtered sample (0-255 range)
        """
        # Track DC offset with slow-moving average
        self.audio_dc_offset = (
            self.audio_dc_alpha * raw_val +
            (1 - self.audio_dc_alpha) * self.audio_dc_offset
        )

        # Remove DC offset (center signal around 0)
        centered = raw_val - self.audio_dc_offset

        # Low-pass filter (exponential moving average)
        self.audio_last_filtered = (
            self.audio_lp_alpha * centered +
            (1 - self.audio_lp_alpha) * self.audio_last_filtered
        )

        # Re-center to 128 and clamp to 0-255
        result = self.audio_last_filtered + 128
        return max(0, min(255, result))

    def _process_sample(self, sample):
        """Process a single sensor sample."""
        # Calculate time in seconds
        if self.first_timestamp is None:
            self.first_timestamp = sample['timestamp_us']

        time_s = (sample['timestamp_us'] - self.first_timestamp) / 1_000_000

        # Update buffers (always, for live chart display)
        self.time_buffer.append(time_s)

        # Update known buffers
        for key in self.data_buffers:
            if key in sample:
                self.data_buffers[key].append(sample[key])
            else:
                # Keep buffer length consistent
                self.data_buffers[key].append(self.data_buffers[key][-1] if self.data_buffers[key] else 0)

        # Handle dynamic vars from schema
        for name, _, _ in var_schema:
            if name in sample:
                if name not in self.data_buffers:
                    self.data_buffers[name] = deque([0] * CHART_HISTORY, maxlen=CHART_HISTORY)
                self.data_buffers[name].append(sample[name])

        # Handle audio samples (batched) - use continuous sample counter for proper timing
        if 'audio' in sample and sample.get('audio'):
            audio_samples = sample['audio']
            self.audio_packets_received += 1
            sample_period = 1.0 / self.audio_sample_rate
            for val in audio_samples:
                audio_time = self.audio_sample_count * sample_period

                # Apply filtering if enabled
                if self.audio_filter_enabled:
                    val = self._filter_audio_sample(val)

                self.audio_buffer.append(val)
                self.audio_time_buffer.append(audio_time)
                self.audio_sample_count += 1

                # Record to WAV buffer if recording
                if self.is_recording_audio:
                    self.audio_recording_buffer.append(int(val))

        # Always show current time (so user knows data is flowing)
        self.time_label.setText(f"Time: {time_s:.1f}s")

        # Recording - also store to buffer
        if self.is_recording:
            self.recording_buffer.append({
                'time_s': time_s,
                'ts_us': sample['timestamp_us'],
                **sample
            })
            self.samples_label.setText(f"Recorded: {len(self.recording_buffer)}")

    def _update_charts(self):
        """Update chart displays."""
        time_data = list(self.time_buffer)

        for name, curve in self.curves.items():
            if name in self.data_buffers:
                curve.setData(x=time_data, y=list(self.data_buffers[name]))

        # Update audio chart with its own time buffer
        audio_time = list(self.audio_time_buffer)
        audio_data = list(self.audio_buffer)
        if audio_time and audio_data:
            self.audio_curve.setData(x=audio_time, y=audio_data)

        # Update FFT spectrum
        if len(audio_data) >= self.fft_size:
            self._update_fft(audio_data)

    def _update_fft(self, audio_data):
        """Compute and display FFT spectrum."""
        # Get last fft_size samples
        samples = np.array(audio_data[-self.fft_size:], dtype=np.float32)

        # Remove DC offset (center around 0)
        samples = samples - np.mean(samples)

        # Apply Hanning window to reduce spectral leakage
        window = np.hanning(len(samples))
        samples = samples * window

        # Compute FFT
        fft_result = np.fft.rfft(samples)
        fft_magnitude = np.abs(fft_result)

        # Convert to dB (with floor to avoid log(0))
        fft_db = 20 * np.log10(fft_magnitude + 1e-10)

        # Frequency axis
        freq_axis = np.fft.rfftfreq(len(samples), 1.0 / self.audio_sample_rate)

        # Update plot
        self.fft_curve.setData(x=freq_axis, y=fft_db)

        # Update fill (create new baseline at -60 dB)
        baseline = np.full_like(fft_db, -60)
        self.fft_fill.setCurves(
            pg.PlotDataItem(freq_axis, fft_db),
            pg.PlotDataItem(freq_axis, baseline)
        )

    def _send_motor_command(self):
        """Send motor command based on current keyboard state."""
        if not self.robot_ip:
            return

        # Calculate motor speeds from key state
        left = 0
        right = 0

        if self.key_state['forward']:
            left = MOTOR_SPEED
            right = MOTOR_SPEED
        elif self.key_state['backward']:
            left = -MOTOR_SPEED
            right = -MOTOR_SPEED

        if self.key_state['left']:
            left -= TURN_SPEED
            right += TURN_SPEED
        elif self.key_state['right']:
            left += TURN_SPEED
            right -= TURN_SPEED

        # Clamp to valid range
        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        # Send command: 'M' + left_byte + right_byte
        # Convert signed to unsigned byte
        left_byte = left if left >= 0 else 256 + left
        right_byte = right if right >= 0 else 256 + right

        cmd = bytes([ord('M'), left_byte, right_byte])

        try:
            self.send_sock.sendto(cmd, (self.robot_ip, ROBOT_PORT_COMMAND))
        except Exception as e:
            print(f"Send error: {e}")

    def keyPressEvent(self, event):
        """Handle key press events."""
        key = event.key()

        # Movement keys
        if key in (Qt.Key_W, Qt.Key_Up):
            self.key_state['forward'] = True
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.key_state['backward'] = True
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.key_state['left'] = True
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.key_state['right'] = True
        elif key == Qt.Key_Space:
            # Stop all movement
            for k in self.key_state:
                self.key_state[k] = False

        # Control keys
        elif key == Qt.Key_R:
            self._toggle_recording()
        elif key == Qt.Key_T:
            self._toggle_audio_recording()
        elif key == Qt.Key_F:
            self._toggle_audio_filter()
        elif key == Qt.Key_G:
            self._toggle_audio_charts()
        elif key == Qt.Key_E:
            self._export_data()
        elif key == Qt.Key_V:
            self._view_all()
        elif key in (Qt.Key_Q, Qt.Key_Escape):
            self._quit()

    def keyReleaseEvent(self, event):
        """Handle key release events."""
        key = event.key()

        if key in (Qt.Key_W, Qt.Key_Up):
            self.key_state['forward'] = False
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.key_state['backward'] = False
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.key_state['left'] = False
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.key_state['right'] = False

    def _toggle_recording(self):
        """Start or stop recording."""
        self.is_recording = not self.is_recording

        if self.is_recording:
            # Start recording
            self.recording_buffer = []
            self.recording_label.setText("Recording: ON")
            self.recording_label.setStyleSheet("font-weight: bold; color: red;")
            print("Recording started...")
        else:
            # Stop recording
            self.recording_label.setText("Recording: OFF")
            self.recording_label.setStyleSheet("font-weight: bold; color: black;")
            print(f"Recording stopped. {len(self.recording_buffer)} samples captured.")

    def _toggle_audio_recording(self):
        """Start or stop audio WAV recording."""
        self.is_recording_audio = not self.is_recording_audio

        if self.is_recording_audio:
            # Start audio recording
            self.audio_recording_buffer = []
            self.audio_packets_received = 0  # Reset counter
            self.audio_rec_label.setText("Audio: REC")
            self.audio_rec_label.setStyleSheet("font-weight: bold; color: red;")
            print("Audio recording started (press T to stop)...")
        else:
            # Stop and save WAV
            self.audio_rec_label.setText("Audio: OFF")
            self.audio_rec_label.setStyleSheet("font-weight: bold; color: black;")
            print(f"Audio packets received during recording: {self.audio_packets_received}")
            if self.audio_recording_buffer:
                self._export_wav()
            else:
                print("No audio recorded.")

    def _toggle_audio_filter(self):
        """Toggle audio filtering on/off."""
        self.audio_filter_enabled = not self.audio_filter_enabled
        state = "ON" if self.audio_filter_enabled else "OFF"
        print(f"Audio filter: {state}")

    def _toggle_audio_charts(self):
        """Toggle audio waveform and FFT chart visibility."""
        self.audio_charts_visible = not self.audio_charts_visible
        if self.audio_charts_visible:
            self.audio_plot.show()
            self.fft_plot.show()
            print("Audio charts: visible")
        else:
            self.audio_plot.hide()
            self.fft_plot.hide()
            print("Audio charts: hidden")

    def _export_wav(self):
        """Export recorded audio to WAV file with post-processing."""
        import wave
        from scipy import signal

        if not self.audio_recording_buffer:
            print("No audio to export!")
            return

        # Convert to numpy array and normalize to -1..1
        audio = np.array(self.audio_recording_buffer, dtype=np.float32)
        audio = (audio - 128) / 128.0  # Center and normalize

        # === STEP 1: Pre-emphasis filter (boost high frequencies for speech clarity) ===
        pre_emphasis = 0.97
        audio_preemph = np.append(audio[0], audio[1:] - pre_emphasis * audio[:-1])

        # === STEP 2: Bandpass filter for voice (300Hz - 3400Hz telephony standard) ===
        nyquist = self.audio_sample_rate / 2
        low_cut = 300 / nyquist   # Remove low rumble/hum
        high_cut = min(3400, nyquist - 100) / nyquist  # Voice range

        b, a = signal.butter(4, [low_cut, high_cut], btype='band')
        audio_filtered = signal.filtfilt(b, a, audio_preemph)

        # === STEP 3: Spectral subtraction noise reduction ===
        # Estimate noise from quietest 20% of frames
        frame_size = 256
        hop_size = 128
        n_frames = (len(audio_filtered) - frame_size) // hop_size + 1

        if n_frames > 5:
            # Calculate frame energies
            frame_energies = []
            for i in range(n_frames):
                start = i * hop_size
                frame = audio_filtered[start:start + frame_size]
                frame_energies.append(np.sum(frame ** 2))

            # Find threshold for quietest 20% (likely noise)
            sorted_energies = sorted(frame_energies)
            noise_threshold_idx = max(1, int(0.2 * len(sorted_energies)))
            noise_energy_threshold = sorted_energies[noise_threshold_idx]

            # Estimate noise spectrum from quiet frames
            noise_frames = []
            for i, energy in enumerate(frame_energies):
                if energy <= noise_energy_threshold:
                    start = i * hop_size
                    frame = audio_filtered[start:start + frame_size]
                    noise_frames.append(np.abs(np.fft.rfft(frame * np.hanning(frame_size))))

            if noise_frames:
                noise_spectrum = np.mean(noise_frames, axis=0)

                # Apply spectral subtraction
                output = np.zeros_like(audio_filtered)
                window = np.hanning(frame_size)

                for i in range(n_frames):
                    start = i * hop_size
                    frame = audio_filtered[start:start + frame_size]

                    # FFT
                    spectrum = np.fft.rfft(frame * window)
                    magnitude = np.abs(spectrum)
                    phase = np.angle(spectrum)

                    # Subtract noise (with floor to avoid negative values)
                    clean_magnitude = np.maximum(magnitude - 1.5 * noise_spectrum, 0.1 * magnitude)

                    # Reconstruct
                    clean_spectrum = clean_magnitude * np.exp(1j * phase)
                    clean_frame = np.fft.irfft(clean_spectrum, n=frame_size)

                    # Overlap-add
                    output[start:start + frame_size] += clean_frame * window

                audio_filtered = output

        # === STEP 4: Noise gate with smooth attack/release ===
        # Calculate envelope
        envelope = np.abs(audio_filtered)
        # Smooth envelope
        b_env, a_env = signal.butter(2, 50 / nyquist, btype='low')
        envelope_smooth = signal.filtfilt(b_env, a_env, envelope)

        # Gate threshold (adaptive based on signal level)
        gate_threshold = np.percentile(envelope_smooth, 30) * 2
        gate_gain = np.where(envelope_smooth > gate_threshold, 1.0,
                            envelope_smooth / gate_threshold)
        audio_filtered = audio_filtered * gate_gain

        # === STEP 5: Normalize with compression ===
        # Light compression to reduce dynamic range
        threshold_db = -20
        ratio = 3.0
        threshold = 10 ** (threshold_db / 20)

        # Apply compression
        sign = np.sign(audio_filtered)
        magnitude = np.abs(audio_filtered)
        above_threshold = magnitude > threshold
        magnitude[above_threshold] = threshold + (magnitude[above_threshold] - threshold) / ratio
        audio_filtered = sign * magnitude

        # Final normalization
        max_val = np.max(np.abs(audio_filtered))
        if max_val > 0:
            audio_filtered = audio_filtered / max_val * 0.85  # Leave headroom

        # Convert to 16-bit PCM
        audio_16bit = (audio_filtered * 32767).astype(np.int16)

        # Generate filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(DATA_DIR, f"audio_{timestamp}.wav")

        # Write WAV file (16-bit for better quality)
        with wave.open(filename, 'wb') as wav:
            wav.setnchannels(1)  # Mono
            wav.setsampwidth(2)  # 16-bit
            wav.setframerate(self.audio_sample_rate)
            wav.writeframes(audio_16bit.tobytes())

        # Also save raw version for comparison
        raw_filename = os.path.join(DATA_DIR, f"audio_{timestamp}_raw.wav")
        audio_8bit = bytes(self.audio_recording_buffer)
        with wave.open(raw_filename, 'wb') as wav:
            wav.setnchannels(1)
            wav.setsampwidth(1)
            wav.setframerate(self.audio_sample_rate)
            wav.writeframes(audio_8bit)

        duration = len(self.audio_recording_buffer) / self.audio_sample_rate
        print(f"\nExported {duration:.2f}s audio to:")
        print(f"  {filename} (processed, 16-bit)")
        print(f"  {raw_filename} (raw, 8-bit)")
        print(f"  Processing: pre-emphasis, bandpass 300-3400Hz, spectral subtraction,")
        print(f"              noise gate, compression, normalization")

    def _export_data(self):
        """Export recorded data to CSV file."""
        if not self.recording_buffer:
            print("No data to export!")
            return

        # Generate filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(DATA_DIR, f"recording_{timestamp}.csv")

        # Collect all fields from samples
        all_fields = set()
        for sample in self.recording_buffer:
            all_fields.update(sample.keys())

        # Order fields sensibly
        priority_fields = ['time_s', 'ts_us', 'motor_l', 'motor_r', 'us',
                           'opto_x1', 'opto_x2', 'opto_x3', 'opto_x4',
                           'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'imu_ts_us']
        ordered_fields = [f for f in priority_fields if f in all_fields]
        ordered_fields += sorted([f for f in all_fields if f not in priority_fields])

        # Write CSV
        with open(filename, 'w') as f:
            # Header
            f.write(','.join(ordered_fields) + '\n')

            # Data
            for sample in self.recording_buffer:
                values = []
                for field in ordered_fields:
                    val = sample.get(field, '')
                    if isinstance(val, float):
                        values.append(f"{val:.4f}")
                    else:
                        values.append(str(val))
                f.write(','.join(values) + '\n')

        print(f"\nExported {len(self.recording_buffer)} samples to:")
        print(f"  {filename}")
        print(f"Fields: {', '.join(ordered_fields)}")
        print(f"\nAnalyze with:")
        print(f"  python analyzers/fft_analyzer.py {filename}")
        print(f"  python analyzers/bump_detector.py {filename} --plot")

    def _view_all(self):
        """Zoom out all charts to show entire timeframe."""
        if not self.time_buffer or len(self.time_buffer) < 2:
            return

        # Get time range from buffer
        times = list(self.time_buffer)
        t_min = min(times)
        t_max = max(times)

        # Add small margin
        margin = (t_max - t_min) * 0.02
        t_min -= margin
        t_max += margin

        # Apply to first plot (others are linked via setXLink)
        first_plot = list(self.plots.values())[0]
        first_plot.setXRange(t_min, t_max, padding=0)

        # Enable auto-range for Y on all plots
        for plot in self.plots.values():
            plot.enableAutoRange(axis='y')

    def _quit(self):
        """Send quit command and exit."""
        if self.robot_ip:
            try:
                cmd = bytes([ord('Q')])
                self.send_sock.sendto(cmd, (self.robot_ip, ROBOT_PORT_COMMAND))
            except:
                pass

        print("\nExiting...")
        QtWidgets.QApplication.instance().quit()

    def closeEvent(self, event):
        """Handle window close."""
        self._quit()
        event.accept()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    """Main entry point."""
    print("\n" + "=" * 60)
    print("PICOBOT DATA COLLECTOR")
    print("Educational Sensor Data Collection")
    print("=" * 60)
    print("\nControls:")
    print("  WASD / Arrows  - Drive robot")
    print("  Space          - Stop")
    print("  R              - Start/Stop recording")
    print("  E              - Export to CSV")
    print("  Q / Esc        - Quit")
    print("\n" + "-" * 60)
    print(f"Waiting for robot on port {ROBOT_PORT_DATA}...")
    print("-" * 60 + "\n")

    app = QtWidgets.QApplication(sys.argv)
    window = CollectorWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
