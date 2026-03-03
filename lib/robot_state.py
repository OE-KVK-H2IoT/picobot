"""
==============================================================================
ROBOT STATE - Dead Reckoning and Motion Estimation
==============================================================================

This module handles the estimation of robot motion (position, velocity,
heading) using only IMU (Inertial Measurement Unit) data.

WHAT IS DEAD RECKONING?
-----------------------
Dead reckoning is estimating your current position based on:
  1. A known starting position
  2. Measurements of how you've moved since then

For a robot with an IMU, this means:
  - Integrate gyroscope readings to get heading (rotation)
  - Integrate accelerometer readings to get velocity
  - Integrate velocity to get position

WHY DOES IT DRIFT?
------------------
Dead reckoning always drifts over time because:
  1. Sensor noise gets integrated (small errors accumulate)
  2. Sensor bias (offset) causes constant drift
  3. Accelerometer measures gravity + motion (hard to separate perfectly)

In practice, you need external references to correct drift:
  - Wheel encoders (odometry)
  - GPS
  - Visual landmarks
  - SLAM (Simultaneous Localization and Mapping)

But for short-term motion estimation, IMU-only dead reckoning works!

COORDINATE SYSTEM:
------------------
We use a right-handed coordinate system:
  - X axis: Points FORWARD (robot's front)
  - Y axis: Points LEFT
  - Z axis: Points UP

  - Positive rotation around Z (yaw) = turning LEFT
  - Heading 0° = facing initial forward direction

==============================================================================
"""

import time
import math


# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Gravity in g units (accelerometer reads ~1g when stationary)
GRAVITY_G = 1.0

# Noise thresholds - ignore small values (sensor noise)
ACCEL_THRESHOLD_G = 0.05     # ~0.5 m/s² - typical noise floor
GYRO_THRESHOLD_DPS = 1.0     # 1 deg/s - typical gyro noise

# Velocity damping - reduces drift when robot appears stationary
# When acceleration is below threshold, multiply velocity by this factor
# 0.95 means velocity decays to ~5% after 60 samples of no acceleration
VELOCITY_DAMPING = 0.95

# Minimum velocity to track (below this, set to zero)
MIN_VELOCITY_MS = 0.01  # 1 cm/s


# ==============================================================================
# FILTER CLASSES
# ==============================================================================

class LowPassFilter:
    """
    Simple low-pass filter to smooth sensor readings.

    Formula: output = alpha * input + (1 - alpha) * prev_output

    Higher alpha = more responsive but noisier
    Lower alpha = smoother but more lag

    Typical values:
      - alpha = 0.1 for very smooth (lots of lag)
      - alpha = 0.3 for balanced
      - alpha = 0.5 for responsive
    """

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


class HighPassFilter:
    """
    High-pass filter to remove slow drift/bias from signals.

    Useful for accelerometer to remove gravity component.
    Only passes through rapid changes (actual motion).

    Formula: output = alpha * (prev_output + input - prev_input)

    Higher alpha = passes more low frequencies
    Lower alpha = blocks more low frequencies
    """

    def __init__(self, alpha=0.95):
        self.alpha = alpha
        self.prev_input = None
        self.prev_output = 0.0

    def update(self, new_value):
        if self.prev_input is None:
            self.prev_input = new_value
            return 0.0

        self.prev_output = self.alpha * (self.prev_output + new_value - self.prev_input)
        self.prev_input = new_value
        return self.prev_output

    def reset(self):
        self.prev_input = None
        self.prev_output = 0.0


class MovingAverage:
    """
    Moving average filter for smoothing.

    Stores last N samples and returns their average.
    Good for reducing noise while maintaining responsiveness.
    """

    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []

    def update(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

    def reset(self):
        self.values = []


class StationaryDetector:
    """
    Detects when the robot is stationary using multiple sensor axes.

    Uses variance of recent samples - if all axes are stable,
    robot is probably not moving.
    """

    def __init__(self, window_size=20, accel_var_threshold=0.002, gyro_var_threshold=0.5):
        self.window_size = window_size
        self.accel_var_threshold = accel_var_threshold
        self.gyro_var_threshold = gyro_var_threshold
        self.accel_history = []
        self.gyro_history = []

    def update(self, accel_magnitude, gyro_z):
        """
        Update with new readings and return True if stationary.

        Parameters:
            accel_magnitude: Total acceleration magnitude (should be ~1g when still)
            gyro_z: Z-axis gyroscope reading
        """
        self.accel_history.append(accel_magnitude)
        self.gyro_history.append(gyro_z)

        if len(self.accel_history) > self.window_size:
            self.accel_history.pop(0)
            self.gyro_history.pop(0)

        if len(self.accel_history) < self.window_size // 2:
            return False

        # Calculate variance
        accel_mean = sum(self.accel_history) / len(self.accel_history)
        accel_var = sum((x - accel_mean)**2 for x in self.accel_history) / len(self.accel_history)

        gyro_mean = sum(self.gyro_history) / len(self.gyro_history)
        gyro_var = sum((x - gyro_mean)**2 for x in self.gyro_history) / len(self.gyro_history)

        is_stationary = (accel_var < self.accel_var_threshold and
                        gyro_var < self.gyro_var_threshold)

        return is_stationary

    def reset(self):
        self.accel_history = []
        self.gyro_history = []


# ==============================================================================
# ROBOT STATE CLASS
# ==============================================================================

class RobotState:
    """
    Tracks robot motion state using IMU dead reckoning.

    This class estimates:
      - velocity_ms:    Forward velocity in meters/second
      - distance_m:     Total distance traveled in meters
      - heading_deg:    Heading/yaw angle in degrees
      - position:       (x, y) position in meters

    Usage:
        state = RobotState()
        state.calibrate(sensor)  # Call once at startup, robot must be still!

        # In main loop:
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()

        motion = state.update(accel, gyro, timestamp)
        print(f"Velocity: {motion['velocity_ms']:.2f} m/s")
        print(f"Heading: {motion['heading_deg']:.1f} degrees")
    """

    def __init__(self, enable_filters=True, filter_alpha=0.3):
        """
        Initialize state to zero.

        Parameters:
            enable_filters: Enable low-pass filtering on sensors (default True)
            filter_alpha:   Filter responsiveness 0.1-0.9 (default 0.3)
                           Lower = smoother but more lag
                           Higher = more responsive but noisier
        """
        # Motion estimates
        self.velocity_ms = 0.0          # Forward velocity (m/s)
        self.distance_m = 0.0           # Odometer - total distance (m)
        self.heading_deg = 0.0          # Heading angle (degrees)
        self.heading_rad = 0.0          # Heading angle (radians)

        # Position in 2D (x, y) - world frame
        self.position_x = 0.0           # meters
        self.position_y = 0.0           # meters

        # Angular velocity (for external use)
        self.angular_velocity_dps = 0.0  # degrees/second

        # Calibration values (set by calibrate())
        self.gyro_bias = [0.0, 0.0, 0.0]     # deg/s bias for x, y, z
        self.accel_bias = [0.0, 0.0, 0.0]    # g bias for x, y, z
        self.is_calibrated = False

        # Timing
        self._prev_time_us = None

        # Filtering options
        self.enable_filters = enable_filters
        self.filter_alpha = filter_alpha

        # Initialize filters
        self._accel_filter_x = LowPassFilter(filter_alpha)
        self._accel_filter_y = LowPassFilter(filter_alpha)
        self._accel_filter_z = LowPassFilter(filter_alpha)
        self._gyro_filter_z = LowPassFilter(filter_alpha)

        # High-pass filter for acceleration (removes gravity drift)
        self._accel_hp_x = HighPassFilter(0.98)

        # Stationary detector
        self._stationary_detector = StationaryDetector(
            window_size=20,
            accel_var_threshold=0.003,
            gyro_var_threshold=1.0
        )
        self.is_stationary = False

    def calibrate(self, sensor, num_samples=100, delay_ms=10):
        """
        Calibrate sensor biases while robot is STATIONARY.

        IMPORTANT: Robot must not be moving during calibration!

        This measures the average sensor readings when stationary and
        stores them as bias values to subtract from future readings.

        For the accelerometer, we expect:
          - ax ≈ 0 (no sideways tilt)
          - ay ≈ 0 (no forward/back tilt)
          - az ≈ 1g (gravity pointing up)

        For the gyroscope, we expect:
          - All axes ≈ 0 (no rotation)

        Parameters:
            sensor:      MPU6050 sensor instance
            num_samples: Number of samples to average (default 100)
            delay_ms:    Delay between samples in ms (default 10)
        """
        print("="*50)
        print("CALIBRATION")
        print("Keep robot STATIONARY for", num_samples * delay_ms / 1000, "seconds...")
        print("="*50)

        # Accumulate readings
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]

        for i in range(num_samples):
            accel = sensor.get_accel()
            gyro = sensor.get_gyro()

            accel_sum[0] += accel['x']
            accel_sum[1] += accel['y']
            accel_sum[2] += accel['z']

            gyro_sum[0] += gyro['x']
            gyro_sum[1] += gyro['y']
            gyro_sum[2] += gyro['z']

            time.sleep_ms(delay_ms)

        # Calculate averages
        n = num_samples

        # Gyro bias = average reading (should be ~0)
        self.gyro_bias[0] = gyro_sum[0] / n
        self.gyro_bias[1] = gyro_sum[1] / n
        self.gyro_bias[2] = gyro_sum[2] / n

        # Accel bias = average reading - expected value
        # Expected: (0, 0, 1g) when flat
        self.accel_bias[0] = accel_sum[0] / n - 0.0
        self.accel_bias[1] = accel_sum[1] / n - 0.0
        self.accel_bias[2] = accel_sum[2] / n - GRAVITY_G

        self.is_calibrated = True

        print("Calibration complete!")
        print(f"  Accel bias: ({self.accel_bias[0]:.4f}, {self.accel_bias[1]:.4f}, {self.accel_bias[2]:.4f}) g")
        print(f"  Gyro bias:  ({self.gyro_bias[0]:.2f}, {self.gyro_bias[1]:.2f}, {self.gyro_bias[2]:.2f}) deg/s")
        print("="*50)

    def update(self, accel, gyro, timestamp_us):
        """
        Update state with new sensor readings.

        Call this function every time you read the IMU sensor.
        The function uses the timestamp to calculate the time delta
        for integration.

        Parameters:
            accel:        Dict with 'x', 'y', 'z' in g units
            gyro:         Dict with 'x', 'y', 'z' in deg/s
            timestamp_us: Timestamp in microseconds (from time.ticks_us())

        Returns:
            Dict with current motion estimates:
              - velocity_ms: Forward velocity (m/s)
              - distance_m:  Total distance traveled (m)
              - heading_deg: Heading angle (degrees)
              - position_x:  X position (m)
              - position_y:  Y position (m)
              - angular_velocity_dps: Rotation rate (deg/s)
        """

        # =====================================================================
        # STEP 1: Calculate time delta (dt)
        # =====================================================================

        if self._prev_time_us is None:
            self._prev_time_us = timestamp_us
            return self.get_state()

        # Calculate dt in microseconds
        dt_us = timestamp_us - self._prev_time_us

        # Handle timer wraparound (ticks_us wraps at 2^30 on Pico ≈ 17 minutes)
        if dt_us < 0:
            dt_us += 2**30

        # Convert to seconds
        dt_s = dt_us / 1_000_000.0
        self._prev_time_us = timestamp_us

        # Skip if dt is unreasonable (glitch protection)
        if dt_s > 0.1 or dt_s < 0.001:
            return self.get_state()

        # =====================================================================
        # STEP 2: Apply calibration (remove bias)
        # =====================================================================

        ax = accel['x'] - self.accel_bias[0]
        ay = accel['y'] - self.accel_bias[1]
        az = accel['z'] - self.accel_bias[2]

        gx = gyro['x'] - self.gyro_bias[0]
        gy = gyro['y'] - self.gyro_bias[1]
        gz = gyro['z'] - self.gyro_bias[2]

        # =====================================================================
        # STEP 2.5: Apply optional filtering
        # =====================================================================

        if self.enable_filters:
            ax = self._accel_filter_x.update(ax)
            ay = self._accel_filter_y.update(ay)
            az = self._accel_filter_z.update(az)
            gz = self._gyro_filter_z.update(gz)

        # Calculate acceleration magnitude for stationary detection
        accel_magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        self.is_stationary = self._stationary_detector.update(accel_magnitude, gz)

        # =====================================================================
        # STEP 3: Update heading from gyroscope Z
        # =====================================================================
        #
        # The gyroscope measures angular velocity (how fast we're rotating).
        # To get the angle, we INTEGRATE:
        #
        #   heading = heading + angular_velocity * dt
        #
        # We use the Z-axis gyro because that measures rotation around the
        # vertical axis (yaw), which is how a ground robot turns.

        if abs(gz) > GYRO_THRESHOLD_DPS:
            self.angular_velocity_dps = gz

            # Convert deg/s to rad/s and integrate
            gz_rad = math.radians(gz)
            self.heading_rad += gz_rad * dt_s
            self.heading_deg = math.degrees(self.heading_rad)

            # Keep heading in [-180, 180] range
            while self.heading_deg > 180:
                self.heading_deg -= 360
                self.heading_rad -= 2 * math.pi
            while self.heading_deg < -180:
                self.heading_deg += 360
                self.heading_rad += 2 * math.pi
        else:
            self.angular_velocity_dps = 0.0

        # =====================================================================
        # STEP 4: Update velocity from accelerometer X
        # =====================================================================
        #
        # For a differential drive robot on flat ground, forward/backward
        # acceleration shows up primarily on the X-axis.
        #
        # To get velocity, we INTEGRATE acceleration:
        #
        #   velocity = velocity + acceleration * dt
        #
        # We convert from g units to m/s²: 1g = 9.81 m/s²

        # Use stationary detector for better drift control
        if self.is_stationary:
            # Robot is definitely stationary - aggressively zero velocity
            self.velocity_ms *= 0.8  # Fast decay
            if abs(self.velocity_ms) < MIN_VELOCITY_MS * 10:
                self.velocity_ms = 0.0
        elif abs(ax) > ACCEL_THRESHOLD_G:
            # Convert g to m/s²
            forward_accel_ms2 = ax * 9.81

            # Integrate: v = v + a*dt
            self.velocity_ms += forward_accel_ms2 * dt_s
        else:
            # Below threshold - robot is probably stationary or constant velocity
            # Apply damping to reduce drift
            self.velocity_ms *= VELOCITY_DAMPING

            # Zero out very small velocities
            if abs(self.velocity_ms) < MIN_VELOCITY_MS:
                self.velocity_ms = 0.0

        # =====================================================================
        # STEP 5: Update position from velocity and heading
        # =====================================================================
        #
        # Position in 2D is updated using:
        #   x = x + velocity * cos(heading) * dt
        #   y = y + velocity * sin(heading) * dt
        #
        # This converts from the robot's local velocity (forward) to
        # global position (x, y world coordinates).

        if abs(self.velocity_ms) > MIN_VELOCITY_MS:
            # Update position in world frame
            self.position_x += self.velocity_ms * math.cos(self.heading_rad) * dt_s
            self.position_y += self.velocity_ms * math.sin(self.heading_rad) * dt_s

            # Update odometer (total distance traveled)
            self.distance_m += abs(self.velocity_ms) * dt_s

        return self.get_state()

    def get_state(self):
        """Return current state as a dictionary."""
        return {
            'velocity_ms': self.velocity_ms,
            'distance_m': self.distance_m,
            'heading_deg': self.heading_deg,
            'position_x': self.position_x,
            'position_y': self.position_y,
            'angular_velocity_dps': self.angular_velocity_dps,
            'is_stationary': self.is_stationary,
        }

    def reset(self):
        """Reset all motion estimates to zero (keeps calibration)."""
        self.velocity_ms = 0.0
        self.distance_m = 0.0
        self.heading_deg = 0.0
        self.heading_rad = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.angular_velocity_dps = 0.0
        self._prev_time_us = None
        self.is_stationary = False

        # Reset filters
        self._accel_filter_x.reset()
        self._accel_filter_y.reset()
        self._accel_filter_z.reset()
        self._gyro_filter_z.reset()
        self._accel_hp_x.reset()
        self._stationary_detector.reset()

    def print_state(self):
        """Print current state in a readable format."""
        print(f"Position: ({self.position_x:.2f}, {self.position_y:.2f}) m")
        print(f"Heading:  {self.heading_deg:.1f} deg")
        print(f"Velocity: {self.velocity_ms:.2f} m/s")
        print(f"Distance: {self.distance_m:.2f} m (odometer)")


# ==============================================================================
# STREAMING HELPER
# ==============================================================================

class IMUStreamer:
    """
    Helper class to read IMU data and optionally send it over UDP.

    This separates the "data collection" logic from the main student code.

    Usage:
        streamer = IMUStreamer(sensor, udp_ip="192.168.1.100")
        streamer.start()  # Begins streaming in background

        # Or for manual control:
        streamer = IMUStreamer(sensor)
        while True:
            data = streamer.read_sample()
            # Process data...

        # With ultrasonic:
        streamer = IMUStreamer(sensor, ultrasonic=ultrasonic_sensor)
    """

    # Packet types for protocol versioning
    PACKET_TYPE_IMU = 0x01
    PACKET_TYPE_IMU_ULTRASONIC = 0x02

    def __init__(self, sensor, robot_state=None, ultrasonic=None, udp_ip=None, udp_port=5005,
                 ultrasonic_interval_ms=100):
        """
        Initialize streamer.

        Parameters:
            sensor:      MPU6050 sensor instance
            robot_state: RobotState instance for dead reckoning (optional)
            ultrasonic:  Ultrasonic sensor instance (optional)
            udp_ip:      IP address to stream to (None = don't stream)
            udp_port:    UDP port (default 5005)
        """
        self.sensor = sensor
        self.robot_state = robot_state
        self.ultrasonic = ultrasonic
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.ultrasonic_interval_ms = ultrasonic_interval_ms

        self._sock = None
        if udp_ip:
            import socket
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Last ultrasonic reading (don't read every sample - too slow)
        self._last_ultrasonic_cm = -1.0
        self._last_ultrasonic_ms = time.ticks_ms()
        self._last_sample = None
        self._imu_error_count = 0
        self._last_imu_error_ms = 0

    def read_sample(self):
        """
        Read one sample from IMU.

        Returns:
            Dict with 'timestamp_us', 'accel', 'gyro', and optionally 'state', 'ultrasonic_cm'
        """
        timestamp_us = time.ticks_us()
        try:
            accel = self.sensor.get_accel()
            gyro = self.sensor.get_gyro()
        except Exception as exc:
            self._imu_error_count += 1
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, self._last_imu_error_ms) > 1000:
                print("IMU read error:", exc)
                self._last_imu_error_ms = now_ms
            if self._last_sample:
                # Reuse last good sample to keep stream alive.
                fallback = dict(self._last_sample)
                fallback['timestamp_us'] = timestamp_us
                fallback['imu_error'] = True
                return fallback
            return {
                'timestamp_us': timestamp_us,
                'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'imu_error': True,
            }

        result = {
            'timestamp_us': timestamp_us,
            'accel': accel,
            'gyro': gyro,
        }

        # Update robot state if available
        if self.robot_state:
            state = self.robot_state.update(accel, gyro, timestamp_us)
            result['state'] = state

        # Read ultrasonic occasionally (time-based to reduce blocking)
        if self.ultrasonic:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, self._last_ultrasonic_ms) >= self.ultrasonic_interval_ms:
                self._last_ultrasonic_ms = now_ms
                self._last_ultrasonic_cm = self.ultrasonic.distance_cm()
            result['ultrasonic_cm'] = self._last_ultrasonic_cm

        self._last_sample = result
        return result

    def read_batch(self, batch_size=10, delay_ms=5):
        """
        Read a batch of samples.

        Parameters:
            batch_size: Number of samples per batch
            delay_ms:   Delay between samples

        Returns:
            List of sample dicts
        """
        batch = []
        for _ in range(batch_size):
            sample = self.read_sample()
            batch.append(sample)
            time.sleep_ms(delay_ms)
        return batch

    def send_batch(self, batch):
        """
        Send a batch of samples over UDP.

        Packet format v2 (with ultrasonic):
          Header: 1 byte packet type + 1 byte batch size
          Per sample: timestamp(8) + accel(12) + gyro(12) + ultrasonic(4) = 36 bytes

        Parameters:
            batch: List of sample dicts from read_batch()
        """
        if not self._sock or not self.udp_ip:
            return

        import struct

        has_ultrasonic = self.ultrasonic is not None

        if has_ultrasonic:
            # New format with ultrasonic
            data = struct.pack("<BB", self.PACKET_TYPE_IMU_ULTRASONIC, len(batch))
            for sample in batch:
                ts = sample['timestamp_us']
                a = sample['accel']
                g = sample['gyro']
                us = sample.get('ultrasonic_cm', -1.0)
                data += struct.pack("<Q7f", ts, a['x'], a['y'], a['z'],
                                   g['x'], g['y'], g['z'], us)
        else:
            # Legacy format (no header, for backward compatibility)
            data = b''
            for sample in batch:
                ts = sample['timestamp_us']
                a = sample['accel']
                g = sample['gyro']
                data += struct.pack("<Q6f", ts, a['x'], a['y'], a['z'],
                                   g['x'], g['y'], g['z'])

        self._sock.sendto(data, (self.udp_ip, self.udp_port))
