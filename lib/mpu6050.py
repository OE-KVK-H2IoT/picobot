"""
==============================================================================
MPU6050 IMU SENSOR DRIVER
==============================================================================

This module provides a simple driver for the MPU6050 6-axis IMU sensor.

WHAT IS AN IMU?
---------------
IMU = Inertial Measurement Unit

The MPU6050 contains two sensors:
  1. 3-axis Accelerometer - measures acceleration (including gravity)
  2. 3-axis Gyroscope     - measures angular velocity (rotation speed)

Together, these can estimate orientation and motion.

ACCELEROMETER:
--------------
Measures acceleration in g units (1g = 9.81 m/s²).

When the sensor is:
  - Flat on table:     reads approximately (0, 0, +1) g
  - Tilted forward:    X becomes positive
  - Tilted left:       Y becomes positive
  - Upside down:       Z becomes negative

The accelerometer "feels" gravity as an upward force (like standing in
an elevator accelerating upward). This is why it reads +1g on Z when
stationary - it's measuring the force that prevents it from falling.

GYROSCOPE:
----------
Measures angular velocity in degrees per second (°/s).

When the sensor is:
  - Stationary:        reads approximately (0, 0, 0) °/s
  - Rotating around X: X value increases (pitch rotation)
  - Rotating around Y: Y value increases (roll rotation)
  - Rotating around Z: Z value increases (yaw rotation)

The gyroscope has "bias" - it reads a small non-zero value even when
stationary. This bias should be calibrated out.

I2C COMMUNICATION:
------------------
The MPU6050 communicates via I2C:
  - Default address: 0x68
  - Data registers: 0x3B-0x48 (accelerometer + gyroscope)
  - Each axis uses 2 bytes (16-bit signed integer)

SENSITIVITY:
------------
The sensor has configurable sensitivity ranges. This driver uses:
  - Accelerometer: ±2g  → sensitivity = 16384 LSB/g
  - Gyroscope:     ±250°/s → sensitivity = 131 LSB/(°/s)

==============================================================================
"""


class MPU6050:
    """
    Simple driver for MPU6050 6-axis IMU sensor.

    Usage:
        from machine import Pin, I2C
        from mpu6050 import MPU6050

        i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
        sensor = MPU6050(i2c)

        accel = sensor.get_accel()
        print(f"Acceleration: X={accel['x']:.2f}g, Y={accel['y']:.2f}g, Z={accel['z']:.2f}g")

        gyro = sensor.get_gyro()
        print(f"Gyroscope: X={gyro['x']:.1f}°/s, Y={gyro['y']:.1f}°/s, Z={gyro['z']:.1f}°/s")
    """

    # I2C address (AD0 pin low = 0x68, AD0 pin high = 0x69)
    MPU_ADDR = 0x68

    # Register addresses
    REG_PWR_MGMT_1 = 0x6B    # Power management register
    REG_SMPLRT_DIV = 0x19    # Sample rate divider
    REG_CONFIG = 0x1A        # Configuration (DLPF)
    REG_GYRO_CONFIG = 0x1B   # Gyroscope configuration
    REG_ACCEL_CONFIG = 0x1C  # Accelerometer configuration
    REG_ACCEL_XOUT_H = 0x3B  # First accelerometer data register
    REG_GYRO_XOUT_H = 0x43   # First gyroscope data register

    # Sensitivity scale factors (for default ±2g and ±250°/s ranges)
    ACCEL_SCALE = 16384.0   # LSB per g
    GYRO_SCALE = 131.0      # LSB per °/s

    # DLPF bandwidth settings (REG_CONFIG bits 0-2)
    # Lower bandwidth = more filtering = less noise but more lag
    DLPF_260HZ = 0  # Accel 260Hz, Gyro 256Hz, 8kHz sample rate
    DLPF_184HZ = 1  # Accel 184Hz, Gyro 188Hz, 1kHz sample rate
    DLPF_94HZ = 2   # Accel 94Hz, Gyro 98Hz, 1kHz sample rate
    DLPF_44HZ = 3   # Accel 44Hz, Gyro 42Hz, 1kHz sample rate
    DLPF_21HZ = 4   # Accel 21Hz, Gyro 20Hz, 1kHz sample rate
    DLPF_10HZ = 5   # Accel 10Hz, Gyro 10Hz, 1kHz sample rate
    DLPF_5HZ = 6    # Accel 5Hz, Gyro 5Hz, 1kHz sample rate

    def __init__(self, i2c, address=0x68, dlpf=2):
        """
        Initialize MPU6050 sensor.

        Parameters:
            i2c:     I2C bus object (from machine.I2C)
            address: I2C address (default 0x68)
            dlpf:    Digital Low Pass Filter setting 0-6 (default 2 = 94Hz)
                     Higher values = more filtering, good for vibration
        """
        self.i2c = i2c
        self.address = address

        # Wake up the sensor (it starts in sleep mode)
        self._write_register(self.REG_PWR_MGMT_1, 0x00)

        # Configure Digital Low Pass Filter
        # DLPF=2 gives 94Hz bandwidth - less filtering, captures faster motion
        self._write_register(self.REG_CONFIG, dlpf)

        # Set sample rate divider (Sample Rate = 1kHz / (1 + divider))
        # With divider=1, sample rate = 500Hz
        self._write_register(self.REG_SMPLRT_DIV, 1)

        # Configure gyro range (±250°/s for precision)
        self._write_register(self.REG_GYRO_CONFIG, 0x00)

        # Configure accel range (±2g for precision)
        self._write_register(self.REG_ACCEL_CONFIG, 0x00)

    def _write_register(self, register, value):
        """Write a single byte to a register."""
        self.i2c.writeto_mem(self.address, register, bytes([value]))

    def _read_register(self, register, num_bytes=1):
        """Read bytes from a register."""
        return self.i2c.readfrom_mem(self.address, register, num_bytes)

    def _read_word(self, register):
        """
        Read a 16-bit signed value from two consecutive registers.

        The MPU6050 stores data as big-endian (high byte first).
        We need to combine the two bytes and handle sign.
        """
        data = self._read_register(register, 2)

        # Combine high byte and low byte
        value = (data[0] << 8) | data[1]

        # Convert to signed (two's complement)
        if value > 32767:
            value -= 65536

        return value

    def get_accel_raw(self):
        """
        Read raw accelerometer values.

        Returns:
            Tuple (ax, ay, az) - raw 16-bit signed integers
        """
        ax = self._read_word(0x3B)
        ay = self._read_word(0x3D)
        az = self._read_word(0x3F)
        return (ax, ay, az)

    def get_gyro_raw(self):
        """
        Read raw gyroscope values.

        Returns:
            Tuple (gx, gy, gz) - raw 16-bit signed integers
        """
        gx = self._read_word(0x43)
        gy = self._read_word(0x45)
        gz = self._read_word(0x47)
        return (gx, gy, gz)

    def get_accel(self):
        """
        Read accelerometer values in g units.

        Returns:
            Dict with 'x', 'y', 'z' keys, values in g (1g = 9.81 m/s²)

        Example:
            accel = sensor.get_accel()
            # When flat: accel ≈ {'x': 0.0, 'y': 0.0, 'z': 1.0}
        """
        ax, ay, az = self.get_accel_raw()

        return {
            'x': ax / self.ACCEL_SCALE,
            'y': ay / self.ACCEL_SCALE,
            'z': az / self.ACCEL_SCALE
        }

    def get_gyro(self):
        """
        Read gyroscope values in degrees per second.

        Returns:
            Dict with 'x', 'y', 'z' keys, values in °/s

        Example:
            gyro = sensor.get_gyro()
            # When stationary: gyro ≈ {'x': 0.0, 'y': 0.0, 'z': 0.0}
            # (may have small bias offset)
        """
        gx, gy, gz = self.get_gyro_raw()

        return {
            'x': gx / self.GYRO_SCALE,
            'y': gy / self.GYRO_SCALE,
            'z': gz / self.GYRO_SCALE
        }

    def get_all(self):
        """
        Read all sensor values at once.

        Returns:
            Dict with 'accel' and 'gyro' sub-dicts

        Example:
            data = sensor.get_all()
            print(f"Accel Z: {data['accel']['z']:.2f} g")
            print(f"Gyro Z: {data['gyro']['z']:.1f} °/s")
        """
        return {
            'accel': self.get_accel(),
            'gyro': self.get_gyro()
        }

    def get_temp(self):
        """
        Read the internal temperature sensor.

        Returns:
            Temperature in degrees Celsius

        Note: This is the chip temperature, not ambient temperature.
        It's useful for temperature compensation but not as a thermometer.
        """
        raw_temp = self._read_word(0x41)
        # Formula from datasheet
        temp_c = (raw_temp / 340.0) + 36.53
        return temp_c
