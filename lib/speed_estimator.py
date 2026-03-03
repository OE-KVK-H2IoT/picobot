"""
==============================================================================
SPEED ESTIMATOR - Multi-Method Robot Speed Estimation
==============================================================================

Estimate robot speed without wheel encoders using multiple methods:

1. OPTO PATTERN COUNTING (most accurate with calibration track)
   - Count line crossings on calibration track
   - Known line spacing = ground truth distance
   - Best for calibration and validation

2. ULTRASONIC RATE OF CHANGE
   - Measure distance change over time
   - Requires wall/obstacle in front
   - Good for validation

3. IMU VIBRATION ANALYSIS
   - Motor vibration frequency/amplitude correlates with speed
   - Works without external references
   - Needs training data

4. MOTOR COMMAND MODEL
   - Estimate from PWM commands
   - Requires calibration curve
   - Primary method for real-time use

CALIBRATION WORKFLOW:
--------------------
1. Create calibration track (see docs/calibration-track.md)
2. Run calibration collection script
3. Train speed model on PC
4. Deploy coefficients to Pico

==============================================================================
"""

import time
import math


class OptoSpeedEstimator:
    """
    Estimate speed by counting line crossings on calibration track.

    CALIBRATION TRACK DESIGN:
    -------------------------
    Print evenly spaced lines on paper/tape:
      - Line width: 5mm black
      - Gap width: 5mm white
      - Total pattern: 10mm per cycle
      - At least 20 lines (200mm track)

    The opto sensor will see alternating high/low readings as robot
    crosses each line. Count crossings × line spacing = distance.

    Usage:
        opto = OptoSpeedEstimator(line_spacing_cm=1.0)

        # In main loop:
        opto.update(line_sensor.value())  # 0 or 1

        speed = opto.get_speed()       # cm/s
        distance = opto.get_distance() # cm
    """

    def __init__(self, line_spacing_cm=1.0, debounce_ms=5):
        """
        Initialize opto speed estimator.

        Args:
            line_spacing_cm: Distance between line centers (typically 1.0 cm)
            debounce_ms: Minimum time between edge detections
        """
        self.line_spacing = line_spacing_cm
        self.debounce_us = debounce_ms * 1000

        # State
        self.last_state = None
        self.last_edge_us = 0
        self.edge_count = 0
        self.distance = 0.0

        # Speed calculation (sliding window of edge times)
        self.edge_times = []  # List of (timestamp_us, edge_count)
        self.window_size = 10  # Edges to average over
        self.speed = 0.0

        # Calibration data collection
        self.calibration_data = []  # (pwm, measured_speed)

    def update(self, sensor_value):
        """
        Update with new sensor reading.

        Args:
            sensor_value: Digital sensor value (0 or 1, or True/False)

        Returns:
            True if edge detected, False otherwise
        """
        now_us = time.ticks_us()
        state = 1 if sensor_value else 0

        # Initialize on first call
        if self.last_state is None:
            self.last_state = state
            return False

        # Detect edge (change from 0→1 or 1→0)
        edge_detected = False
        if state != self.last_state:
            # Debounce check
            if time.ticks_diff(now_us, self.last_edge_us) > self.debounce_us:
                self.edge_count += 1
                self.last_edge_us = now_us
                edge_detected = True

                # Record edge time for speed calculation
                self.edge_times.append((now_us, self.edge_count))
                if len(self.edge_times) > self.window_size:
                    self.edge_times.pop(0)

                # Update distance (each edge = half a line spacing)
                self.distance += self.line_spacing / 2.0

                # Calculate speed from edge timing
                self._update_speed()

        self.last_state = state
        return edge_detected

    def _update_speed(self):
        """Calculate speed from recent edge timings."""
        if len(self.edge_times) < 2:
            self.speed = 0.0
            return

        # Use oldest and newest edge
        t1, c1 = self.edge_times[0]
        t2, c2 = self.edge_times[-1]

        dt_s = time.ticks_diff(t2, t1) / 1_000_000.0
        if dt_s > 0:
            # Each edge = half line spacing
            distance_cm = (c2 - c1) * (self.line_spacing / 2.0)
            self.speed = distance_cm / dt_s
        else:
            self.speed = 0.0

    def get_speed(self):
        """Get current speed estimate in cm/s."""
        # Decay speed if no edges for a while
        if self.edge_times:
            last_edge_us = self.edge_times[-1][0]
            elapsed_ms = time.ticks_diff(time.ticks_us(), last_edge_us) / 1000
            if elapsed_ms > 100:  # No edges for 100ms
                self.speed *= 0.9  # Decay
                if elapsed_ms > 500:
                    self.speed = 0.0
        return self.speed

    def get_distance(self):
        """Get total distance traveled in cm."""
        return self.distance

    def reset(self):
        """Reset distance and edge count."""
        self.distance = 0.0
        self.edge_count = 0
        self.edge_times = []
        self.speed = 0.0


class UltrasonicSpeedEstimator:
    """
    Estimate speed from ultrasonic distance rate of change.

    Only works when approaching or receding from a wall/obstacle.
    Good for validation, not primary estimation.

    Usage:
        us_speed = UltrasonicSpeedEstimator()

        # In main loop:
        distance = ultrasonic.distance_cm()
        us_speed.update(distance)

        speed = us_speed.get_speed()  # cm/s (positive = approaching)
    """

    def __init__(self, filter_alpha=0.3):
        """
        Initialize ultrasonic speed estimator.

        Args:
            filter_alpha: Low-pass filter coefficient (0-1, higher = less filtering)
        """
        self.filter_alpha = filter_alpha

        # State
        self.last_distance = None
        self.last_time_us = None
        self.speed = 0.0
        self.filtered_speed = 0.0

        # History for validation
        self.history = []
        self.history_size = 20

    def update(self, distance_cm):
        """
        Update with new distance reading.

        Args:
            distance_cm: Current distance in cm (-1 if invalid)

        Returns:
            Estimated speed in cm/s (positive = approaching wall)
        """
        now_us = time.ticks_us()

        # Skip invalid readings
        if distance_cm < 0 or distance_cm > 400:
            return self.filtered_speed

        # Initialize on first valid reading
        if self.last_distance is None:
            self.last_distance = distance_cm
            self.last_time_us = now_us
            return 0.0

        # Calculate speed
        dt_s = time.ticks_diff(now_us, self.last_time_us) / 1_000_000.0
        if dt_s > 0.01:  # At least 10ms between updates
            delta_cm = self.last_distance - distance_cm  # Positive = approaching
            self.speed = delta_cm / dt_s

            # Low-pass filter
            self.filtered_speed = (
                self.filter_alpha * self.speed +
                (1 - self.filter_alpha) * self.filtered_speed
            )

            # Update state
            self.last_distance = distance_cm
            self.last_time_us = now_us

            # Record history
            self.history.append((now_us, distance_cm, self.filtered_speed))
            if len(self.history) > self.history_size:
                self.history.pop(0)

        return self.filtered_speed

    def get_speed(self):
        """Get current speed estimate in cm/s."""
        return self.filtered_speed


class IMUSpeedEstimator:
    """
    Estimate speed from IMU vibration patterns.

    Theory: Motor vibration creates characteristic accelerometer patterns
    that correlate with wheel speed. This class:
    1. Extracts vibration features (amplitude, variance, zero crossings)
    2. Applies a trained model to estimate speed

    REQUIRES CALIBRATION:
    - Collect data at known speeds (using opto or ultrasonic)
    - Train linear regression on PC
    - Load coefficients here

    Usage:
        imu_speed = IMUSpeedEstimator()
        imu_speed.load_calibration(slope=0.15, intercept=0.5, var_threshold=0.5)

        # In main loop:
        accel = imu.get_accel()
        imu_speed.update(accel['x'], accel['y'], accel['z'])

        speed = imu_speed.get_speed()  # cm/s
    """

    def __init__(self, window_size=50, sample_rate_hz=100):
        """
        Initialize IMU speed estimator.

        Args:
            window_size: Samples for vibration analysis
            sample_rate_hz: Expected IMU sample rate
        """
        self.window_size = window_size
        self.sample_rate = sample_rate_hz

        # Vibration history
        self.accel_history = []  # Raw magnitude
        self.variance_history = []  # For smoothing

        # Calibration parameters (must be set via load_calibration)
        self.calibrated = False
        self.var_slope = 0.0      # speed = slope * variance + intercept
        self.var_intercept = 0.0
        self.var_threshold = 0.5  # Below this = stopped

        # State
        self.speed = 0.0
        self.vibration_level = 0.0
        self.is_moving = False

    def load_calibration(self, slope, intercept, threshold=0.5):
        """
        Load calibration parameters from training.

        Args:
            slope: Coefficient for variance → speed mapping
            intercept: Offset for mapping
            threshold: Variance below which robot is considered stopped
        """
        self.var_slope = slope
        self.var_intercept = intercept
        self.var_threshold = threshold
        self.calibrated = True

    def update(self, ax, ay, az):
        """
        Update with new accelerometer reading.

        Args:
            ax, ay, az: Accelerometer values in g

        Returns:
            Estimated speed in cm/s
        """
        # Calculate magnitude (remove gravity bias approximately)
        magnitude = math.sqrt(ax*ax + ay*ay + az*az)

        # Store in history
        self.accel_history.append(magnitude)
        if len(self.accel_history) > self.window_size:
            self.accel_history.pop(0)

        # Need enough samples
        if len(self.accel_history) < 20:
            return 0.0

        # Calculate variance (vibration level)
        mean_mag = sum(self.accel_history) / len(self.accel_history)
        variance = sum((x - mean_mag)**2 for x in self.accel_history) / len(self.accel_history)
        self.vibration_level = variance * 1000  # Scale for readability

        # Determine if moving
        self.is_moving = self.vibration_level > self.var_threshold

        # Estimate speed
        if not self.is_moving:
            self.speed = 0.0
        elif self.calibrated:
            self.speed = max(0, self.var_slope * self.vibration_level + self.var_intercept)
        else:
            # Uncalibrated fallback (rough approximation)
            self.speed = self.vibration_level * 0.5

        return self.speed

    def get_speed(self):
        """Get current speed estimate in cm/s."""
        return self.speed

    def get_vibration(self):
        """Get current vibration level."""
        return self.vibration_level

    def is_stationary(self):
        """Check if robot appears stationary."""
        return not self.is_moving


class BumpFrequencyEstimator:
    """
    Estimate vibration frequency from accelerometer magnitude.

    Uses zero-crossings around the mean to estimate dominant frequency.
    """

    def __init__(self, sample_rate_hz=100, window_size=100):
        self.sample_rate = sample_rate_hz
        self.window_size = window_size
        self.samples = []

    def update(self, ax, ay, az):
        magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        self.samples.append(magnitude)
        if len(self.samples) > self.window_size:
            self.samples.pop(0)

    def estimate_frequency(self):
        if len(self.samples) < 20:
            return 0.0
        mean = sum(self.samples) / len(self.samples)
        crossings = 0
        above = self.samples[0] > mean
        for s in self.samples[1:]:
            now_above = s > mean
            if now_above != above:
                crossings += 1
                above = now_above
        duration_s = len(self.samples) / self.sample_rate
        if duration_s <= 0:
            return 0.0
        return (crossings / 2) / duration_s


class SpeedEstimator:
    """
    Combined speed estimator using multiple methods.

    Fuses estimates from:
    - Opto (if on calibration track)
    - Ultrasonic (if approaching wall)
    - IMU vibration (always available)
    - Motor command model (primary)

    Also collects training data for ML calibration.

    Usage:
        from speed_estimator import SpeedEstimator

        speed_est = SpeedEstimator(imu, opto_pin=2)

        # In main loop:
        speed_est.update_motor_command(left_pwm, right_pwm)
        speed_est.update()

        speed = speed_est.get_speed()

        # For ML training:
        speed_est.start_recording()
        # ... run robot at various speeds ...
        speed_est.stop_recording()
        data = speed_est.get_training_data()
    """

    # Default motor model (uncalibrated)
    DEFAULT_PWM_TO_SPEED = 0.15  # cm/s per PWM unit
    DEFAULT_DEAD_ZONE = 30      # PWM below this = no movement

    def __init__(self, imu=None, opto_pin=None, ultrasonic=None,
                 line_spacing_cm=1.0):
        """
        Initialize combined speed estimator.

        Args:
            imu: IMU sensor instance (MPU6050 or BMI160)
            opto_pin: GPIO pin for opto sensor (for calibration track)
            ultrasonic: Ultrasonic sensor instance
            line_spacing_cm: Calibration track line spacing
        """
        self.imu = imu
        self.ultrasonic = ultrasonic

        # Sub-estimators
        self.imu_est = IMUSpeedEstimator() if imu else None
        self.opto_est = OptoSpeedEstimator(line_spacing_cm) if opto_pin else None
        self.us_est = UltrasonicSpeedEstimator() if ultrasonic else None
        self.bump_est = BumpFrequencyEstimator() if imu else None

        # Opto sensor
        self._opto_pin = None
        if opto_pin is not None:
            from machine import Pin
            self._opto_pin = Pin(opto_pin, Pin.IN)

        # Motor model parameters
        self.pwm_to_speed = self.DEFAULT_PWM_TO_SPEED
        self.dead_zone = self.DEFAULT_DEAD_ZONE

        # Motor command state
        self.cmd_left = 0
        self.cmd_right = 0

        # Combined estimate
        self.speed = 0.0
        self.speed_source = 'none'
        self.bump_per_cm = None
        self.bump_speed = 0.0

        # Distance integration
        self.distance_cm = 0.0
        self.last_update_us = None

        # Training data collection
        self.is_recording = False
        self.training_data = []
        self.last_record_us = 0
        self.record_interval_ms = 50  # Record every 50ms

    def load_motor_calibration(self, pwm_to_speed, dead_zone=30):
        """
        Load motor calibration parameters.

        Args:
            pwm_to_speed: cm/s per PWM unit (above dead zone)
            dead_zone: PWM value below which motors don't move
        """
        self.pwm_to_speed = pwm_to_speed
        self.dead_zone = dead_zone

    def load_imu_calibration(self, slope, intercept, threshold=0.5):
        """Load IMU vibration calibration."""
        if self.imu_est:
            self.imu_est.load_calibration(slope, intercept, threshold)

    def load_bump_calibration(self, bump_per_cm):
        """
        Load bump/cm calibration from host analysis.

        Args:
            bump_per_cm: bumps per centimeter (frequency / speed)
        """
        if bump_per_cm and bump_per_cm > 0:
            self.bump_per_cm = bump_per_cm

    def update_motor_command(self, left_pwm, right_pwm):
        """Update current motor commands."""
        self.cmd_left = left_pwm
        self.cmd_right = right_pwm

    def update(self):
        """
        Update all estimators and fuse estimates.

        Call this in main loop at regular intervals.
        """
        now_us = time.ticks_us()
        if self.last_update_us is None:
            self.last_update_us = now_us

        # Update opto if available
        opto_speed = None
        if self.opto_est and self._opto_pin:
            self.opto_est.update(self._opto_pin.value())
            opto_speed = self.opto_est.get_speed()

        # Update ultrasonic if available
        us_speed = None
        if self.us_est and self.ultrasonic:
            dist = self.ultrasonic.distance_cm()
            if dist > 0:
                us_speed = abs(self.us_est.update(dist))

        # Update IMU if available
        imu_speed = None
        imu_vibration = 0.0
        bump_freq = 0.0
        if self.imu_est and self.imu:
            accel = self.imu.get_accel()
            imu_speed = self.imu_est.update(accel['x'], accel['y'], accel['z'])
            imu_vibration = self.imu_est.get_vibration()
            if self.bump_est:
                self.bump_est.update(accel['x'], accel['y'], accel['z'])
                bump_freq = self.bump_est.estimate_frequency()
                if self.bump_per_cm:
                    self.bump_speed = bump_freq / self.bump_per_cm
                else:
                    self.bump_speed = 0.0

        # Motor command estimate
        avg_pwm = (abs(self.cmd_left) + abs(self.cmd_right)) / 2.0
        if avg_pwm > self.dead_zone:
            motor_speed = (avg_pwm - self.dead_zone) * self.pwm_to_speed
        else:
            motor_speed = 0.0

        # Fuse estimates (priority: opto > motor > imu)
        if opto_speed is not None and opto_speed > 0.5:
            self.speed = opto_speed
            self.speed_source = 'opto'
        elif us_speed is not None and us_speed > 0.5:
            self.speed = us_speed
            self.speed_source = 'ultrasonic'
        elif self.bump_speed > 0:
            self.speed = self.bump_speed
            self.speed_source = 'bump'
        elif motor_speed > 0:
            self.speed = motor_speed
            self.speed_source = 'motor'
        elif imu_speed is not None and imu_speed > 0:
            self.speed = imu_speed
            self.speed_source = 'imu'
        else:
            self.speed = 0.0
            self.speed_source = 'none'

        dt_s = time.ticks_diff(now_us, self.last_update_us) / 1_000_000.0
        self.last_update_us = now_us
        if dt_s > 0 and dt_s < 0.5:
            self.distance_cm += self.speed * dt_s

        # Record training data if enabled
        if self.is_recording:
            if time.ticks_diff(now_us, self.last_record_us) > self.record_interval_ms * 1000:
                self.last_record_us = now_us

                # Record all available measurements
                sample = {
                    'timestamp_us': now_us,
                    'pwm_left': self.cmd_left,
                    'pwm_right': self.cmd_right,
                    'pwm_avg': avg_pwm,
                    'imu_vibration': imu_vibration,
                    'opto_speed': opto_speed if opto_speed else -1,
                    'opto_distance': self.opto_est.get_distance() if self.opto_est else -1,
                    'us_speed': us_speed if us_speed else -1,
                    'bump_speed': self.bump_speed if self.bump_speed else -1,
                    'bump_freq': bump_freq if bump_freq else -1,
                    'bump_per_cm': self.bump_per_cm if self.bump_per_cm else -1,
                }

                # Add raw IMU if available
                if self.imu:
                    accel = self.imu.get_accel()
                    gyro = self.imu.get_gyro()
                    sample.update({
                        'ax': accel['x'],
                        'ay': accel['y'],
                        'az': accel['z'],
                        'gx': gyro['x'],
                        'gy': gyro['y'],
                        'gz': gyro['z'],
                    })

                self.training_data.append(sample)

        return self.speed

    def get_speed(self):
        """Get fused speed estimate in cm/s."""
        return self.speed

    def get_bump_speed(self):
        """Get bump-frequency speed estimate in cm/s."""
        return self.bump_speed

    def get_speed_source(self):
        """Get which sensor provided the speed estimate."""
        return self.speed_source

    def get_distance(self):
        """Get total distance from opto (if available)."""
        if self.opto_est:
            return self.opto_est.get_distance()
        return -1

    def get_fused_distance(self):
        """Get fused distance estimate in cm."""
        return self.distance_cm

    def reset_distance(self):
        """Reset fused distance counter."""
        self.distance_cm = 0.0

    # === Training Data Collection ===

    def start_recording(self):
        """Start recording training data."""
        self.training_data = []
        self.is_recording = True
        self.last_record_us = time.ticks_us()
        print("Recording started...")

    def stop_recording(self):
        """Stop recording training data."""
        self.is_recording = False
        print(f"Recording stopped. {len(self.training_data)} samples collected.")

    def get_training_data(self):
        """Get collected training data."""
        return self.training_data

    def reset_opto(self):
        """Reset opto distance counter (call at start of calibration run)."""
        if self.opto_est:
            self.opto_est.reset()


# ==============================================================================
# CALIBRATION TRACK SPECIFICATIONS
# ==============================================================================

CALIBRATION_TRACK_SPEC = """
CALIBRATION TRACK DESIGN
========================

PURPOSE:
--------
Provide ground truth speed measurement for training the speed estimation model.
The robot drives over evenly-spaced lines; counting line crossings gives
precise distance traveled.

CONSTRUCTION:
-------------
Materials:
  - White paper or white tape (background)
  - Black electrical tape or printed black lines

Dimensions:
  - Line width: 5mm black
  - Gap width: 5mm white
  - Total pattern period: 10mm (1cm)
  - Track length: At least 50cm (50 lines)
  - Track width: 3-4cm (covers one opto sensor)

Layout:
  +------------------------------------------+
  |  ████  ████  ████  ████  ████  ████  ... |
  |  5mm  5mm   5mm   5mm                    |
  |  <-- 10mm --> = 1 line crossing          |
  +------------------------------------------+
         ^ Robot drives this direction ^

CALIBRATION PROCEDURE:
----------------------
1. Place robot at start of track
2. Reset distance counter: speed_est.reset_opto()
3. Run robot at constant PWM for known time
4. Record: PWM value, time, opto distance
5. Calculate actual speed: distance / time
6. Repeat at different PWM values (30, 40, 50, ... 100)

SENSOR PLACEMENT:
-----------------
The opto sensor should be positioned:
  - Centered over the line pattern
  - ~3-5mm above the surface
  - Perpendicular to the lines

VALIDATION:
-----------
After calibration, validate with ultrasonic:
  1. Place robot facing wall at 100cm
  2. Drive toward wall for 5 seconds
  3. Compare:
     - Opto measured distance
     - Ultrasonic delta distance
     - Motor model predicted distance

  All three should agree within ~10%

DATA FORMAT:
------------
Training data CSV columns:
  timestamp_us, pwm_left, pwm_right, pwm_avg,
  imu_vibration, ax, ay, az, gx, gy, gz,
  opto_speed, opto_distance, us_speed

For each calibration run, record:
  - Starting position
  - PWM value used
  - Run duration
  - Final opto distance
  - Calculated speed

EXPECTED RESULTS:
-----------------
After training, you should have:
  1. Motor calibration: speed = k * (PWM - dead_zone)
  2. IMU calibration: speed = m * vibration + b
  3. Validation error < 10%
"""

def print_calibration_guide():
    """Print calibration track specifications."""
    print(CALIBRATION_TRACK_SPEC)
