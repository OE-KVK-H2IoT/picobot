"""
==============================================================================
ODOMETRY - Robot Speed and Distance Estimation
==============================================================================

Multiple approaches to estimate robot motion without wheel encoders:

1. MOTOR COMMAND ESTIMATION (most reliable)
   - Estimate velocity from PWM commands sent to motors
   - Requires calibration: PWM value → actual speed

2. VIBRATION-BASED DETECTION
   - Wheel bumps create vibrations when spinning
   - Vibration magnitude correlates with wheel speed
   - Works even without knowing motor commands

3. ACCELEROMETER INTEGRATION (least reliable)
   - Traditional dead reckoning
   - Suffers from drift, only good for short bursts

4. ZERO VELOCITY UPDATE (ZUPT)
   - Detect when robot is truly stationary
   - Reset velocity to zero to prevent drift

==============================================================================
"""

import time
import math


class Odometry:
    """
    Multi-method odometry for PicoBot.

    Usage:
        from odometry import Odometry
        from mpu6050 import MPU6050
        from picobot import Motors

        motors = Motors()
        imu = MPU6050(i2c)
        odom = Odometry(imu)
        odom.calibrate()

        # In main loop:
        motors.forward(100)
        odom.update_motor_command(100, 100)  # Tell odometry what motors are doing

        speed = odom.get_speed()       # m/s
        distance = odom.get_distance() # m
        heading = odom.get_heading()   # degrees
    """

    # Calibration constants (adjust for your robot!)
    # These convert motor PWM (0-255) to actual speed (m/s)
    PWM_TO_SPEED = 0.002  # At PWM=100, speed ≈ 0.2 m/s
    WHEEL_BASE = 0.10    # Distance between wheels (meters)

    # Vibration detection thresholds
    VIBRATION_MOVING_THRESHOLD = 5.0    # Variance threshold for "moving"
    VIBRATION_STOPPED_THRESHOLD = 1.0   # Below this = definitely stopped

    # ZUPT (Zero Velocity Update) settings
    ZUPT_ACCEL_THRESHOLD = 0.02   # g - below this, might be stationary
    ZUPT_GYRO_THRESHOLD = 0.5     # deg/s - below this, not rotating
    ZUPT_SAMPLES_REQUIRED = 50    # Need this many consecutive "still" samples

    def __init__(self, imu, vibration_window=50):
        """
        Initialize odometry.

        Parameters:
            imu: MPU6050 sensor instance
            vibration_window: Samples to use for vibration analysis
        """
        self.imu = imu

        # State
        self.speed = 0.0           # Current speed (m/s)
        self.distance = 0.0        # Total distance traveled (m)
        self.heading_deg = 0.0     # Heading (degrees)
        self.heading_rad = 0.0
        self.position_x = 0.0      # X position (m)
        self.position_y = 0.0      # Y position (m)

        # Motor command state
        self.cmd_left = 0
        self.cmd_right = 0

        # Calibration
        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.is_calibrated = False

        # Vibration detection
        self.vibration_window = vibration_window
        self.accel_history = []  # Store recent raw accelerations
        self.vibration_level = 0.0

        # ZUPT state
        self.zupt_counter = 0
        self.is_stationary = True

        # Timing
        self.last_update_us = None

        # Method selection
        self.use_motor_commands = True  # Primary method
        self.use_vibration = True       # Secondary/validation
        self.use_accelerometer = False  # Tertiary (disabled by default)

    def calibrate(self, num_samples=100, delay_ms=10):
        """
        Calibrate sensor biases. Robot must be STATIONARY!
        """
        print("Calibrating odometry... keep robot still!")

        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]

        for _ in range(num_samples):
            accel = self.imu.get_accel()
            gyro = self.imu.get_gyro()

            accel_sum[0] += accel['x']
            accel_sum[1] += accel['y']
            accel_sum[2] += accel['z']
            gyro_sum[0] += gyro['x']
            gyro_sum[1] += gyro['y']
            gyro_sum[2] += gyro['z']

            time.sleep_ms(delay_ms)

        n = num_samples
        self.accel_bias = [accel_sum[0]/n, accel_sum[1]/n, accel_sum[2]/n - 1.0]
        self.gyro_bias = [gyro_sum[0]/n, gyro_sum[1]/n, gyro_sum[2]/n]
        self.is_calibrated = True

        print(f"Calibration done. Gyro Z bias: {self.gyro_bias[2]:.2f} deg/s")

    def update_motor_command(self, left_pwm, right_pwm):
        """
        Tell odometry what motor commands are being sent.
        Call this whenever you change motor speeds.

        Parameters:
            left_pwm: Left motor PWM (-255 to 255)
            right_pwm: Right motor PWM (-255 to 255)
        """
        self.cmd_left = left_pwm
        self.cmd_right = right_pwm

    def update(self):
        """
        Update odometry with new sensor reading.
        Call this frequently (e.g., every 5-10ms).

        Returns:
            Dict with current estimates
        """
        now_us = time.ticks_us()

        # Calculate dt
        if self.last_update_us is None:
            self.last_update_us = now_us
            return self.get_state()

        dt_us = time.ticks_diff(now_us, self.last_update_us)
        dt = dt_us / 1_000_000.0
        self.last_update_us = now_us

        if dt > 0.1 or dt < 0.001:
            return self.get_state()

        # Read sensors
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()

        # Apply calibration
        ax = accel['x'] - self.accel_bias[0]
        ay = accel['y'] - self.accel_bias[1]
        az = accel['z'] - self.accel_bias[2]
        gz = gyro['z'] - self.gyro_bias[2]

        # === UPDATE HEADING FROM GYRO ===
        if abs(gz) > self.ZUPT_GYRO_THRESHOLD:
            self.heading_rad += math.radians(gz) * dt
            self.heading_deg = math.degrees(self.heading_rad)
            # Normalize to -180..180
            while self.heading_deg > 180:
                self.heading_deg -= 360
                self.heading_rad -= 2 * math.pi
            while self.heading_deg < -180:
                self.heading_deg += 360
                self.heading_rad += 2 * math.pi

        # === UPDATE VIBRATION ANALYSIS ===
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        self.accel_history.append(accel_mag)
        if len(self.accel_history) > self.vibration_window:
            self.accel_history.pop(0)

        if len(self.accel_history) >= 20:
            # Calculate variance (vibration level)
            mean_a = sum(self.accel_history) / len(self.accel_history)
            variance = sum((x - mean_a)**2 for x in self.accel_history) / len(self.accel_history)
            self.vibration_level = variance * 1000  # Scale for readability

        # === ZUPT: Zero Velocity Update ===
        accel_horizontal = math.sqrt(ax*ax + ay*ay)
        if accel_horizontal < self.ZUPT_ACCEL_THRESHOLD and abs(gz) < self.ZUPT_GYRO_THRESHOLD:
            self.zupt_counter += 1
            if self.zupt_counter >= self.ZUPT_SAMPLES_REQUIRED:
                self.is_stationary = True
                self.speed = 0.0  # Force velocity to zero
        else:
            self.zupt_counter = 0
            self.is_stationary = False

        # === ESTIMATE SPEED ===
        if self.is_stationary:
            self.speed = 0.0
        elif self.use_motor_commands and (self.cmd_left != 0 or self.cmd_right != 0):
            # Method 1: Motor command-based estimation
            avg_pwm = (self.cmd_left + self.cmd_right) / 2.0
            self.speed = avg_pwm * self.PWM_TO_SPEED
        elif self.use_vibration and self.vibration_level > self.VIBRATION_MOVING_THRESHOLD:
            # Method 2: Vibration-based estimation
            # Map vibration level to speed (needs calibration!)
            # This is a rough approximation
            self.speed = min(0.3, self.vibration_level * 0.01)
        elif self.use_accelerometer:
            # Method 3: Accelerometer integration (least reliable)
            forward_accel = ax * 9.81  # Assuming X is forward
            self.speed += forward_accel * dt
            self.speed *= 0.95  # Decay to prevent runaway

        # === UPDATE POSITION ===
        if abs(self.speed) > 0.01:
            self.position_x += self.speed * math.cos(self.heading_rad) * dt
            self.position_y += self.speed * math.sin(self.heading_rad) * dt
            self.distance += abs(self.speed) * dt

        return self.get_state()

    def get_state(self):
        """Get current odometry state as dict."""
        return {
            'speed': self.speed,
            'distance': self.distance,
            'heading_deg': self.heading_deg,
            'position_x': self.position_x,
            'position_y': self.position_y,
            'vibration': self.vibration_level,
            'is_stationary': self.is_stationary,
        }

    def get_speed(self):
        """Get current speed in m/s."""
        return self.speed

    def get_distance(self):
        """Get total distance traveled in meters."""
        return self.distance

    def get_heading(self):
        """Get heading in degrees."""
        return self.heading_deg

    def get_position(self):
        """Get (x, y) position in meters."""
        return (self.position_x, self.position_y)

    def reset(self):
        """Reset distance and position to zero (keeps heading)."""
        self.distance = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.speed = 0.0


class VibrationAnalyzer:
    """
    Analyze accelerometer vibration to detect wheel rotation.

    Theory:
    - Wheels have bumps/texture
    - When spinning, bumps create periodic vibrations
    - Vibration frequency ∝ wheel RPM
    - Vibration magnitude ∝ wheel speed (roughly)

    This class provides tools to analyze vibration patterns
    and potentially extract wheel speed from them.
    """

    def __init__(self, sample_rate_hz=500, window_size=100):
        self.sample_rate = sample_rate_hz
        self.window_size = window_size
        self.samples = []

    def add_sample(self, accel_x, accel_y, accel_z):
        """Add a new accelerometer sample."""
        # Use magnitude or specific axis
        magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        self.samples.append(magnitude)
        if len(self.samples) > self.window_size:
            self.samples.pop(0)

    def get_variance(self):
        """Get variance of recent samples (vibration level)."""
        if len(self.samples) < 10:
            return 0.0
        mean = sum(self.samples) / len(self.samples)
        variance = sum((x - mean)**2 for x in self.samples) / len(self.samples)
        return variance

    def get_rms(self):
        """Get RMS of deviation from mean (another vibration metric)."""
        if len(self.samples) < 10:
            return 0.0
        mean = sum(self.samples) / len(self.samples)
        rms = math.sqrt(sum((x - mean)**2 for x in self.samples) / len(self.samples))
        return rms

    def count_zero_crossings(self):
        """
        Count zero crossings (crossings of the mean).
        Higher count = higher frequency vibration = faster wheel speed.
        """
        if len(self.samples) < 20:
            return 0

        mean = sum(self.samples) / len(self.samples)
        crossings = 0
        above = self.samples[0] > mean

        for s in self.samples[1:]:
            now_above = s > mean
            if now_above != above:
                crossings += 1
                above = now_above

        return crossings

    def estimate_frequency(self):
        """
        Estimate dominant vibration frequency from zero crossings.
        Returns frequency in Hz.
        """
        crossings = self.count_zero_crossings()
        # Each complete cycle has 2 crossings
        duration_s = len(self.samples) / self.sample_rate
        if duration_s > 0:
            freq = (crossings / 2) / duration_s
            return freq
        return 0.0
