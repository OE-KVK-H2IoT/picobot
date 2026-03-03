"""
IMU (Inertial Measurement Unit) module for BMI160.

Provides gyroscope-based heading tracking and accelerometer readings.
Students discover calibration and drift in Lab 05.
"""

from machine import I2C, Pin
import time
from .config import PINS, CONTROL, HARDWARE

# BMI160 registers
_REG_CHIP_ID = 0x00
_REG_DATA_GYR_Z_L = 0x10
_REG_DATA_ACC_X_L = 0x12
_REG_ACC_CONF = 0x40
_REG_ACC_RANGE = 0x41
_REG_GYR_CONF = 0x42
_REG_GYR_RANGE = 0x43
_REG_CMD = 0x7E

_CHIP_ID = 0xD1

# Sensitivity for ±2g range: 16384 LSB/g
_ACC_SENSITIVITY = 16384.0
# Sensitivity for ±125°/s range: 262.4 LSB/°/s
_GYR_SENSITIVITY = 262.4


class IMU:
    """
    BMI160-based heading tracker and accelerometer.

    The gyroscope measures angular velocity (deg/s).
    We integrate over time to get heading (degrees).

    Configured for maximum sensitivity:
      - Accelerometer: ±2g range (16384 LSB/g)
      - Gyroscope: ±125°/s range (262.4 LSB/°/s)

    Access via robot.imu
    """

    def __init__(self):
        """Initialize IMU."""
        self._i2c = I2C(
            PINS.IMU_I2C_BUS,
            sda=Pin(PINS.I2C_SDA),
            scl=Pin(PINS.I2C_SCL),
            freq=HARDWARE.IMU_I2C_FREQ
        )

        # Try to detect IMU
        self._address = self._detect_imu()
        if self._address:
            self._init_imu()
            print(f"BMI160 found at 0x{self._address:02x}")
        else:
            print("WARNING: No BMI160 detected!")

        self._gyro_bias = 0.0
        self._heading = 0.0
        self._last_update = time.ticks_us()

    def _detect_imu(self):
        """Detect BMI160 on I2C bus."""
        devices = self._i2c.scan()

        for addr in [0x68, 0x69]:
            if addr in devices:
                try:
                    chip_id = self._i2c.readfrom_mem(addr, _REG_CHIP_ID, 1)[0]
                    if chip_id == _CHIP_ID:
                        return addr
                except:
                    pass

        return None

    def _init_imu(self):
        """Initialize BMI160 registers for maximum sensitivity."""
        # Wake accelerometer (CMD_ACC_NORMAL)
        self._i2c.writeto_mem(self._address, _REG_CMD, bytes([0x11]))
        time.sleep(0.05)
        # Wake gyroscope (CMD_GYR_NORMAL)
        self._i2c.writeto_mem(self._address, _REG_CMD, bytes([0x15]))
        time.sleep(0.1)

        # Accel config: ODR 100Hz, normal bandwidth
        self._i2c.writeto_mem(self._address, _REG_ACC_CONF, bytes([0x28]))
        # Accel range: ±2g (most sensitive, 16384 LSB/g)
        self._i2c.writeto_mem(self._address, _REG_ACC_RANGE, bytes([0x03]))

        # Gyro config: ODR 100Hz, normal bandwidth
        self._i2c.writeto_mem(self._address, _REG_GYR_CONF, bytes([0x28]))
        # Gyro range: ±125°/s (most sensitive, 262.4 LSB/°/s)
        self._i2c.writeto_mem(self._address, _REG_GYR_RANGE, bytes([0x04]))

    def gyro_z(self):
        """
        Read raw gyroscope Z-axis (rotation speed).

        Returns:
            Angular velocity in degrees/second
            Positive = counter-clockwise (left turn)
            Negative = clockwise (right turn)
        """
        if not self._address:
            return 0.0

        try:
            data = self._i2c.readfrom_mem(self._address, _REG_DATA_GYR_Z_L, 2)
            raw = data[0] | (data[1] << 8)
            if raw > 32767:
                raw -= 65536
            return raw / _GYR_SENSITIVITY
        except:
            return 0.0

    def calibrate(self, samples=None):
        """
        Calibrate gyroscope bias.

        IMPORTANT: Keep robot stationary during calibration!

        Args:
            samples: Number of samples (default from config)

        Returns:
            Measured bias value
        """
        if samples is None:
            samples = CONTROL.GYRO_CALIBRATION_SAMPLES

        print("Calibrating gyro... DON'T MOVE!")

        total = 0.0
        for i in range(samples):
            total += self.gyro_z()
            time.sleep(0.02)

            # Progress indicator
            if i % 25 == 0:
                print(".", end="")

        self._gyro_bias = total / samples
        self._heading = 0.0
        self._last_update = time.ticks_us()

        print(f"\nBias: {self._gyro_bias:.4f} deg/s")
        return self._gyro_bias

    def update(self):
        """
        Update heading from gyroscope.

        Call this regularly (every loop iteration) for accurate tracking.
        """
        now = time.ticks_us()
        dt = time.ticks_diff(now, self._last_update) / 1_000_000  # seconds
        self._last_update = now

        gz = self.gyro_z() - self._gyro_bias
        self._heading += gz * dt

    @property
    def heading(self):
        """
        Current heading in degrees.

        Positive = turned left from start
        Negative = turned right from start
        """
        return self._heading

    def reset_heading(self):
        """Reset heading to zero."""
        self._heading = 0.0
        self._last_update = time.ticks_us()

    @property
    def bias(self):
        """Current calibrated bias value."""
        return self._gyro_bias

    def accel(self):
        """
        Read accelerometer.

        Returns:
            Tuple (ax, ay, az) in g units, or (0, 0, 0) if unavailable
        """
        if not self._address:
            return (0.0, 0.0, 0.0)

        try:
            data = self._i2c.readfrom_mem(self._address, _REG_DATA_ACC_X_L, 6)
            ax = data[0] | (data[1] << 8)
            ay = data[2] | (data[3] << 8)
            az = data[4] | (data[5] << 8)
            if ax > 32767: ax -= 65536
            if ay > 32767: ay -= 65536
            if az > 32767: az -= 65536
            return (ax / _ACC_SENSITIVITY, ay / _ACC_SENSITIVITY, az / _ACC_SENSITIVITY)
        except:
            return (0.0, 0.0, 0.0)
