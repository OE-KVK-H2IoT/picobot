"""
==============================================================================
BMI160 IMU SENSOR DRIVER
==============================================================================

Drop-in replacement for MPU6050 with better performance.

BMI160 vs MPU6050 COMPARISON:
-----------------------------
| Feature              | MPU6050      | BMI160         | Improvement    |
|----------------------|--------------|----------------|----------------|
| Accel Noise Density  | 400 µg/√Hz   | 180 µg/√Hz     | 2.2x better    |
| Gyro Noise Density   | 0.005 °/s/√Hz| 0.007 °/s/√Hz  | Similar        |
| Max Sample Rate      | 1 kHz        | 1.6 kHz        | 1.6x faster    |
| Current (normal)     | 3.9 mA       | 0.95 mA        | 4x lower power |
| FIFO                 | 1024 bytes   | 1024 bytes     | Same           |
| Built-in Features    | None         | Step counter,  | More features  |
|                      |              | tap, motion    |                |

USAGE (same as MPU6050):
------------------------
    from machine import Pin, I2C
    from bmi160 import BMI160

    i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
    imu = BMI160(i2c)

    accel = imu.get_accel()  # or imu.read_accel()
    gyro = imu.get_gyro()    # or imu.read_gyro()

WIRING:
-------
    BMI160    Pico
    ------    ----
    VCC   ->  3.3V
    GND   ->  GND
    SCL   ->  GP5 (I2C0 SCL)
    SDA   ->  GP4 (I2C0 SDA)

    Optional:
    INT1  ->  Any GPIO (interrupt)
    INT2  ->  Any GPIO (interrupt)

==============================================================================
"""

import time


class BMI160:
    """
    BMI160 6-axis IMU driver - API compatible with MPU6050.

    Features:
    - Lower noise accelerometer (180 µg/√Hz vs 400 µg/√Hz)
    - Lower power consumption (0.95mA vs 3.9mA)
    - Configurable output data rate up to 1600Hz
    - Built-in step counter, tap detection, motion detection
    """

    # I2C addresses (depends on SDO pin: GND=0x68, VCC=0x69)
    BMI160_ADDR = 0x68
    BMI160_ADDR_ALT = 0x69

    # Chip ID (read from 0x00, should be 0xD1)
    CHIP_ID = 0xD1

    # ==========================================
    # Register addresses
    # ==========================================
    REG_CHIP_ID = 0x00
    REG_ERR_REG = 0x02
    REG_PMU_STATUS = 0x03

    # Data registers (little-endian, unlike MPU6050!)
    REG_DATA_MAG_X_L = 0x04
    REG_DATA_GYR_X_L = 0x0C
    REG_DATA_GYR_X_H = 0x0D
    REG_DATA_GYR_Y_L = 0x0E
    REG_DATA_GYR_Y_H = 0x0F
    REG_DATA_GYR_Z_L = 0x10
    REG_DATA_GYR_Z_H = 0x11
    REG_DATA_ACC_X_L = 0x12
    REG_DATA_ACC_X_H = 0x13
    REG_DATA_ACC_Y_L = 0x14
    REG_DATA_ACC_Y_H = 0x15
    REG_DATA_ACC_Z_L = 0x16
    REG_DATA_ACC_Z_H = 0x17

    REG_SENSORTIME_0 = 0x18
    REG_STATUS = 0x1B
    REG_TEMPERATURE_L = 0x20
    REG_TEMPERATURE_H = 0x21

    # Configuration registers
    REG_ACC_CONF = 0x40
    REG_ACC_RANGE = 0x41
    REG_GYR_CONF = 0x42
    REG_GYR_RANGE = 0x43

    # Interrupt registers
    REG_INT_EN_0 = 0x50
    REG_INT_EN_1 = 0x51
    REG_INT_EN_2 = 0x52
    REG_INT_OUT_CTRL = 0x53
    REG_INT_MAP_0 = 0x55
    REG_INT_MAP_1 = 0x56
    REG_INT_MAP_2 = 0x57

    # FIFO registers
    REG_FIFO_LENGTH_0 = 0x22
    REG_FIFO_DATA = 0x24
    REG_FIFO_CONFIG_0 = 0x46
    REG_FIFO_CONFIG_1 = 0x47

    # Command register
    REG_CMD = 0x7E

    # ==========================================
    # Commands (write to REG_CMD)
    # ==========================================
    CMD_SOFT_RESET = 0xB6
    CMD_STEP_CNT_CLR = 0xB2
    CMD_FIFO_FLUSH = 0xB0
    CMD_INT_RESET = 0xB1

    # Power mode commands
    CMD_ACC_SUSPEND = 0x10
    CMD_ACC_NORMAL = 0x11
    CMD_ACC_LOW_POWER = 0x12
    CMD_GYR_SUSPEND = 0x14
    CMD_GYR_NORMAL = 0x15
    CMD_GYR_FAST_STARTUP = 0x17

    # ==========================================
    # Output Data Rate (ODR) settings
    # ==========================================
    # Accelerometer ODR (REG_ACC_CONF bits 3:0)
    ACC_ODR_25HZ = 0x06
    ACC_ODR_50HZ = 0x07
    ACC_ODR_100HZ = 0x08
    ACC_ODR_200HZ = 0x09
    ACC_ODR_400HZ = 0x0A
    ACC_ODR_800HZ = 0x0B
    ACC_ODR_1600HZ = 0x0C

    # Gyroscope ODR (REG_GYR_CONF bits 3:0)
    GYR_ODR_25HZ = 0x06
    GYR_ODR_50HZ = 0x07
    GYR_ODR_100HZ = 0x08
    GYR_ODR_200HZ = 0x09
    GYR_ODR_400HZ = 0x0A
    GYR_ODR_800HZ = 0x0B
    GYR_ODR_1600HZ = 0x0C
    GYR_ODR_3200HZ = 0x0D

    # ==========================================
    # Bandwidth (filter) settings
    # ==========================================
    # Accelerometer BWP (REG_ACC_CONF bits 6:4)
    ACC_BWP_OSR4 = 0x00 << 4   # Over-sampling rate 4 (most filtering)
    ACC_BWP_OSR2 = 0x01 << 4   # Over-sampling rate 2
    ACC_BWP_NORMAL = 0x02 << 4  # Normal mode (recommended)

    # Gyroscope BWP (REG_GYR_CONF bits 5:4)
    GYR_BWP_OSR4 = 0x00 << 4
    GYR_BWP_OSR2 = 0x01 << 4
    GYR_BWP_NORMAL = 0x02 << 4

    # ==========================================
    # Range settings
    # ==========================================
    # Accelerometer range (REG_ACC_RANGE)
    ACC_RANGE_2G = 0x03   # ±2g  (16384 LSB/g)
    ACC_RANGE_4G = 0x05   # ±4g  (8192 LSB/g)
    ACC_RANGE_8G = 0x08   # ±8g  (4096 LSB/g)
    ACC_RANGE_16G = 0x0C  # ±16g (2048 LSB/g)

    # Gyroscope range (REG_GYR_RANGE)
    GYR_RANGE_2000 = 0x00  # ±2000°/s (16.4 LSB/°/s)
    GYR_RANGE_1000 = 0x01  # ±1000°/s (32.8 LSB/°/s)
    GYR_RANGE_500 = 0x02   # ±500°/s  (65.6 LSB/°/s)
    GYR_RANGE_250 = 0x03   # ±250°/s  (131.2 LSB/°/s)
    GYR_RANGE_125 = 0x04   # ±125°/s  (262.4 LSB/°/s)

    # Scale factors for default ranges
    ACCEL_SCALE_2G = 16384.0
    ACCEL_SCALE_4G = 8192.0
    ACCEL_SCALE_8G = 4096.0
    ACCEL_SCALE_16G = 2048.0

    GYRO_SCALE_2000 = 16.4
    GYRO_SCALE_1000 = 32.8
    GYRO_SCALE_500 = 65.6
    GYRO_SCALE_250 = 131.2
    GYRO_SCALE_125 = 262.4

    # DLPF-equivalent mapping (for MPU6050 compatibility)
    # Maps MPU6050 DLPF values to BMI160 ODR + BWP settings
    DLPF_MAP = {
        0: (ACC_ODR_1600HZ, GYR_ODR_3200HZ),  # 260Hz
        1: (ACC_ODR_800HZ, GYR_ODR_800HZ),    # 184Hz
        2: (ACC_ODR_400HZ, GYR_ODR_400HZ),    # 94Hz
        3: (ACC_ODR_200HZ, GYR_ODR_200HZ),    # 44Hz
        4: (ACC_ODR_100HZ, GYR_ODR_100HZ),    # 21Hz
        5: (ACC_ODR_50HZ, GYR_ODR_50HZ),      # 10Hz
        6: (ACC_ODR_25HZ, GYR_ODR_25HZ),      # 5Hz
    }

    def __init__(self, i2c, address=0x68, dlpf=2, accel_range=None, gyro_range=None):
        """
        Initialize BMI160 sensor.

        Parameters:
            i2c:          I2C bus object
            address:      I2C address (0x68 or 0x69)
            dlpf:         Digital Low Pass Filter setting 0-6 (MPU6050 compatible)
                          0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
            accel_range:  Accelerometer range (2, 4, 8, or 16 g). Default: 2g
            gyro_range:   Gyroscope range (125, 250, 500, 1000, 2000 °/s). Default: 250°/s
        """
        self.i2c = i2c
        self.address = address

        # Default ranges (same as MPU6050 defaults)
        self.accel_range = accel_range or 2
        self.gyro_range = gyro_range or 250

        # Set scale factors based on range
        self._set_scales()

        # Last-known-good readings (returned when I2C fails after retries)
        self._last_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._last_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._i2c_errors = 0  # Running count for diagnostics

        # Initialize sensor
        self._init_sensor(dlpf)

    def _set_scales(self):
        """Set scale factors based on current range settings."""
        scale_map_acc = {2: self.ACCEL_SCALE_2G, 4: self.ACCEL_SCALE_4G,
                         8: self.ACCEL_SCALE_8G, 16: self.ACCEL_SCALE_16G}
        scale_map_gyr = {125: self.GYRO_SCALE_125, 250: self.GYRO_SCALE_250,
                         500: self.GYRO_SCALE_500, 1000: self.GYRO_SCALE_1000,
                         2000: self.GYRO_SCALE_2000}

        self.ACCEL_SCALE = scale_map_acc.get(self.accel_range, self.ACCEL_SCALE_2G)
        self.GYRO_SCALE = scale_map_gyr.get(self.gyro_range, self.GYRO_SCALE_250)

    def _init_sensor(self, dlpf):
        """Initialize the BMI160 sensor."""
        # Soft reset
        self._write_register(self.REG_CMD, self.CMD_SOFT_RESET)
        time.sleep_ms(100)  # Wait for reset

        # Check chip ID
        chip_id = self._read_register(self.REG_CHIP_ID)[0]
        if chip_id != self.CHIP_ID:
            raise RuntimeError(f"BMI160 not found! Got chip ID: 0x{chip_id:02X}, expected 0xD1")

        # Set accelerometer to normal mode
        self._write_register(self.REG_CMD, self.CMD_ACC_NORMAL)
        time.sleep_ms(10)

        # Set gyroscope to normal mode
        self._write_register(self.REG_CMD, self.CMD_GYR_NORMAL)
        time.sleep_ms(100)  # Gyro takes longer to start

        # Configure accelerometer ODR and bandwidth
        acc_odr, gyr_odr = self.DLPF_MAP.get(dlpf, (self.ACC_ODR_400HZ, self.GYR_ODR_400HZ))
        self._write_register(self.REG_ACC_CONF, acc_odr | self.ACC_BWP_NORMAL)
        self._write_register(self.REG_GYR_CONF, gyr_odr | self.GYR_BWP_NORMAL)

        # Configure ranges
        range_map_acc = {2: self.ACC_RANGE_2G, 4: self.ACC_RANGE_4G,
                         8: self.ACC_RANGE_8G, 16: self.ACC_RANGE_16G}
        range_map_gyr = {125: self.GYR_RANGE_125, 250: self.GYR_RANGE_250,
                         500: self.GYR_RANGE_500, 1000: self.GYR_RANGE_1000,
                         2000: self.GYR_RANGE_2000}

        self._write_register(self.REG_ACC_RANGE, range_map_acc.get(self.accel_range, self.ACC_RANGE_2G))
        self._write_register(self.REG_GYR_RANGE, range_map_gyr.get(self.gyro_range, self.GYR_RANGE_250))

        time.sleep_ms(10)

    # I2C retry settings (handles transient EIO errors from vibration/loose wires)
    I2C_RETRIES = 3
    I2C_RETRY_DELAY_MS = 2

    def _write_register(self, register, value):
        """Write a single byte to a register (with retry on I2C error)."""
        for attempt in range(self.I2C_RETRIES):
            try:
                self.i2c.writeto_mem(self.address, register, bytes([value]))
                return
            except OSError:
                if attempt < self.I2C_RETRIES - 1:
                    time.sleep_ms(self.I2C_RETRY_DELAY_MS)
        # All retries failed — let the last error propagate
        self.i2c.writeto_mem(self.address, register, bytes([value]))

    def _read_register(self, register, num_bytes=1):
        """Read bytes from a register (with retry on I2C error)."""
        for attempt in range(self.I2C_RETRIES):
            try:
                return self.i2c.readfrom_mem(self.address, register, num_bytes)
            except OSError:
                if attempt < self.I2C_RETRIES - 1:
                    time.sleep_ms(self.I2C_RETRY_DELAY_MS)
        # All retries failed — let the last error propagate
        return self.i2c.readfrom_mem(self.address, register, num_bytes)

    def _read_word_le(self, register):
        """
        Read a 16-bit signed value (little-endian).

        BMI160 uses little-endian format (unlike MPU6050 which is big-endian).
        """
        data = self._read_register(register, 2)
        value = data[0] | (data[1] << 8)  # Little-endian
        if value > 32767:
            value -= 65536
        return value

    # ==========================================
    # Raw data methods
    # ==========================================

    def get_accel_raw(self):
        """Read raw accelerometer values."""
        ax = self._read_word_le(self.REG_DATA_ACC_X_L)
        ay = self._read_word_le(self.REG_DATA_ACC_Y_L)
        az = self._read_word_le(self.REG_DATA_ACC_Z_L)
        return (ax, ay, az)

    def get_gyro_raw(self):
        """Read raw gyroscope values."""
        gx = self._read_word_le(self.REG_DATA_GYR_X_L)
        gy = self._read_word_le(self.REG_DATA_GYR_Y_L)
        gz = self._read_word_le(self.REG_DATA_GYR_Z_L)
        return (gx, gy, gz)

    # ==========================================
    # Converted data methods (MPU6050 compatible)
    # ==========================================

    def get_accel(self):
        """
        Read accelerometer values in g units.

        Returns last-known-good values if I2C fails after retries.

        Returns:
            Dict with 'x', 'y', 'z' keys, values in g
        """
        try:
            ax, ay, az = self.get_accel_raw()
            self._last_accel = {
                'x': ax / self.ACCEL_SCALE,
                'y': ay / self.ACCEL_SCALE,
                'z': az / self.ACCEL_SCALE
            }
        except OSError:
            self._i2c_errors += 1
        return self._last_accel

    def get_gyro(self):
        """
        Read gyroscope values in degrees per second.

        Returns last-known-good values if I2C fails after retries.

        Returns:
            Dict with 'x', 'y', 'z' keys, values in °/s
        """
        try:
            gx, gy, gz = self.get_gyro_raw()
            self._last_gyro = {
                'x': gx / self.GYRO_SCALE,
                'y': gy / self.GYRO_SCALE,
                'z': gz / self.GYRO_SCALE
            }
        except OSError:
            self._i2c_errors += 1
        return self._last_gyro

    def get_all(self):
        """Read all sensor values at once."""
        return {
            'accel': self.get_accel(),
            'gyro': self.get_gyro()
        }

    # Aliases for MPU6050 compatibility
    read_accel = get_accel
    read_gyro = get_gyro

    def get_temp(self):
        """
        Read temperature sensor.

        Returns:
            Temperature in degrees Celsius
        """
        raw = self._read_word_le(self.REG_TEMPERATURE_L)
        # BMI160 formula: T = raw / 512 + 23
        return raw / 512.0 + 23.0

    # ==========================================
    # Bulk read (more efficient)
    # ==========================================

    def get_all_raw(self):
        """
        Read all 6 axes in one I2C transaction (more efficient).

        Returns last-known-good scaled values (as tuple) if I2C fails.

        Returns:
            Tuple (ax, ay, az, gx, gy, gz) - raw values
        """
        try:
            # Read 12 bytes starting from gyro X (0x0C to 0x17)
            data = self._read_register(self.REG_DATA_GYR_X_L, 12)

            # Parse gyro (first 6 bytes)
            gx = data[0] | (data[1] << 8)
            gy = data[2] | (data[3] << 8)
            gz = data[4] | (data[5] << 8)

            # Parse accel (next 6 bytes)
            ax = data[6] | (data[7] << 8)
            ay = data[8] | (data[9] << 8)
            az = data[10] | (data[11] << 8)

            # Convert to signed
            if ax > 32767: ax -= 65536
            if ay > 32767: ay -= 65536
            if az > 32767: az -= 65536
            if gx > 32767: gx -= 65536
            if gy > 32767: gy -= 65536
            if gz > 32767: gz -= 65536

            self._last_all_raw = (ax, ay, az, gx, gy, gz)
            return self._last_all_raw
        except OSError:
            self._i2c_errors += 1
            return getattr(self, '_last_all_raw', (0, 0, 0, 0, 0, 0))

    # ==========================================
    # Configuration methods
    # ==========================================

    def set_accel_range(self, range_g):
        """
        Set accelerometer range.

        Parameters:
            range_g: 2, 4, 8, or 16 (g)
        """
        range_map = {2: self.ACC_RANGE_2G, 4: self.ACC_RANGE_4G,
                     8: self.ACC_RANGE_8G, 16: self.ACC_RANGE_16G}
        if range_g not in range_map:
            raise ValueError("Accel range must be 2, 4, 8, or 16")

        self._write_register(self.REG_ACC_RANGE, range_map[range_g])
        self.accel_range = range_g
        self._set_scales()

    def set_gyro_range(self, range_dps):
        """
        Set gyroscope range.

        Parameters:
            range_dps: 125, 250, 500, 1000, or 2000 (°/s)
        """
        range_map = {125: self.GYR_RANGE_125, 250: self.GYR_RANGE_250,
                     500: self.GYR_RANGE_500, 1000: self.GYR_RANGE_1000,
                     2000: self.GYR_RANGE_2000}
        if range_dps not in range_map:
            raise ValueError("Gyro range must be 125, 250, 500, 1000, or 2000")

        self._write_register(self.REG_GYR_RANGE, range_map[range_dps])
        self.gyro_range = range_dps
        self._set_scales()

    def set_odr(self, accel_hz, gyro_hz=None):
        """
        Set Output Data Rate.

        Parameters:
            accel_hz: 25, 50, 100, 200, 400, 800, or 1600 Hz
            gyro_hz:  25, 50, 100, 200, 400, 800, 1600, or 3200 Hz (default: same as accel)
        """
        if gyro_hz is None:
            gyro_hz = accel_hz

        odr_map_acc = {25: self.ACC_ODR_25HZ, 50: self.ACC_ODR_50HZ,
                       100: self.ACC_ODR_100HZ, 200: self.ACC_ODR_200HZ,
                       400: self.ACC_ODR_400HZ, 800: self.ACC_ODR_800HZ,
                       1600: self.ACC_ODR_1600HZ}
        odr_map_gyr = {25: self.GYR_ODR_25HZ, 50: self.GYR_ODR_50HZ,
                       100: self.GYR_ODR_100HZ, 200: self.GYR_ODR_200HZ,
                       400: self.GYR_ODR_400HZ, 800: self.GYR_ODR_800HZ,
                       1600: self.GYR_ODR_1600HZ, 3200: self.GYR_ODR_3200HZ}

        if accel_hz in odr_map_acc:
            self._write_register(self.REG_ACC_CONF, odr_map_acc[accel_hz] | self.ACC_BWP_NORMAL)
        if gyro_hz in odr_map_gyr:
            self._write_register(self.REG_GYR_CONF, odr_map_gyr[gyro_hz] | self.GYR_BWP_NORMAL)

    # ==========================================
    # Power management
    # ==========================================

    def sleep(self):
        """Put sensor into sleep mode (low power)."""
        self._write_register(self.REG_CMD, self.CMD_ACC_SUSPEND)
        time.sleep_ms(5)
        self._write_register(self.REG_CMD, self.CMD_GYR_SUSPEND)

    def wake(self):
        """Wake sensor from sleep mode."""
        self._write_register(self.REG_CMD, self.CMD_ACC_NORMAL)
        time.sleep_ms(10)
        self._write_register(self.REG_CMD, self.CMD_GYR_NORMAL)
        time.sleep_ms(100)

    def low_power_mode(self):
        """Put accelerometer into low power mode (gyro suspended)."""
        self._write_register(self.REG_CMD, self.CMD_GYR_SUSPEND)
        time.sleep_ms(5)
        self._write_register(self.REG_CMD, self.CMD_ACC_LOW_POWER)

    # ==========================================
    # FIFO methods (for high-speed buffered reads)
    # ==========================================

    def enable_fifo(self, accel=True, gyro=True):
        """
        Enable FIFO buffering.

        Parameters:
            accel: Include accelerometer data
            gyro:  Include gyroscope data
        """
        config = 0x00
        if accel:
            config |= 0x40  # ACC_EN
        if gyro:
            config |= 0x80  # GYR_EN

        self._write_register(self.REG_FIFO_CONFIG_1, config)

    def disable_fifo(self):
        """Disable FIFO buffering."""
        self._write_register(self.REG_FIFO_CONFIG_1, 0x00)

    def flush_fifo(self):
        """Flush FIFO buffer."""
        self._write_register(self.REG_CMD, self.CMD_FIFO_FLUSH)

    def get_fifo_length(self):
        """Get number of bytes in FIFO."""
        data = self._read_register(self.REG_FIFO_LENGTH_0, 2)
        return data[0] | ((data[1] & 0x07) << 8)

    # ==========================================
    # Status and diagnostics
    # ==========================================

    def get_status(self):
        """
        Get sensor status.

        Returns:
            Dict with status flags
        """
        status = self._read_register(self.REG_STATUS)[0]
        pmu = self._read_register(self.REG_PMU_STATUS)[0]

        return {
            'drdy_acc': bool(status & 0x80),   # Accel data ready
            'drdy_gyr': bool(status & 0x40),   # Gyro data ready
            'drdy_mag': bool(status & 0x20),   # Mag data ready
            'nvm_rdy': bool(status & 0x10),    # NVM ready
            'foc_rdy': bool(status & 0x08),    # FOC ready
            'acc_pmu': (pmu >> 4) & 0x03,      # Accel power mode
            'gyr_pmu': (pmu >> 2) & 0x03,      # Gyro power mode
        }

    def self_test(self):
        """
        Run self-test and return results.

        Returns:
            Dict with test results
        """
        # This is a simplified self-test
        # Full self-test requires comparing readings with/without self-test enabled

        # Check chip ID
        chip_id = self._read_register(self.REG_CHIP_ID)[0]
        id_ok = chip_id == self.CHIP_ID

        # Check error register
        err = self._read_register(self.REG_ERR_REG)[0]
        no_errors = err == 0

        # Check PMU status (should be in normal mode)
        status = self.get_status()
        acc_normal = status['acc_pmu'] == 1
        gyr_normal = status['gyr_pmu'] == 1

        return {
            'chip_id_ok': id_ok,
            'no_errors': no_errors,
            'accel_normal': acc_normal,
            'gyro_normal': gyr_normal,
            'passed': id_ok and no_errors and acc_normal and gyr_normal
        }


# ==============================================================================
# Factory function for easy switching between IMU types
# ==============================================================================

def create_imu(i2c, imu_type='auto', **kwargs):
    """
    Factory function to create IMU instance.

    Parameters:
        i2c:      I2C bus object
        imu_type: 'mpu6050', 'bmi160', or 'auto' (auto-detect)
        **kwargs: Additional arguments passed to constructor

    Returns:
        IMU instance (MPU6050 or BMI160)
    """
    if imu_type == 'auto':
        # Try BMI160 first (check for chip ID 0xD1)
        try:
            chip_id = i2c.readfrom_mem(0x68, 0x00, 1)[0]
            if chip_id == 0xD1:
                imu_type = 'bmi160'
            else:
                imu_type = 'mpu6050'
        except:
            imu_type = 'mpu6050'

    if imu_type == 'bmi160':
        return BMI160(i2c, **kwargs)
    else:
        from mpu6050 import MPU6050
        return MPU6050(i2c, **kwargs)
