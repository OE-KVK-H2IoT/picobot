"""
==============================================================================
PICO DATA COLLECTOR - Educational Sensor Data Collection
==============================================================================

This is a simple, educational MicroPython script for Raspberry Pi Pico W that:
  1. Reads sensor data (IMU accelerometer/gyroscope, ultrasonic)
  2. Receives motor control commands from host
  3. Streams raw sensor data to host via UDP

PURPOSE:
--------
This script focuses on DATA COLLECTION only. All analysis (bump detection,
FFT, speed estimation, ML) is done on the host computer. This separation:
  - Keeps robot code simple and reliable
  - Allows experimenting with analysis algorithms on PC
  - Makes debugging easier (can replay recorded data)

NOTE ON CODE STRUCTURE:
-----------------------
The BMI160 class is included inline for educational clarity - you can see
exactly what each register does. For production code, use the full library:
    from bmi160 import BMI160  # src/libs/bmi160.py
which has additional features (FIFO, power management, self-test).

HARDWARE SETUP:
---------------
  Raspberry Pi Pico W (PicoCar platform) with:
  - BMI160 IMU (I2C: SDA=GP14, SCL=GP15) - lower noise than MPU6050
  - HC-SR04 Ultrasonic sensor (TRIG=GP0, ECHO=GP1)
  - Two DC motors (LEFT: GP12/13, RIGHT: GP10/11)

NETWORK:
--------
  - Robot IP: Assigned by DHCP (printed on startup)
  - Host receives data on port 5005 (UDP)
  - Host sends commands on port 5006 (UDP)

DATA PACKET FORMAT:
-------------------
  Each packet contains multiple samples for efficiency.

  Header (2 bytes):
    - Byte 0: Packet type (0x01 = sensor data)
    - Byte 1: Number of samples in packet

  Per sample (36 bytes):
    - timestamp (8 bytes, uint64): microseconds since boot
    - ax, ay, az (3 × 4 bytes, float): accelerometer in g
    - gx, gy, gz (3 × 4 bytes, float): gyroscope in °/s
    - ultrasonic (4 bytes, float): distance in cm

  Total packet size: 2 + (N × 36) bytes

COMMAND FORMAT:
---------------
  Commands are single bytes or short strings:
    - 'M' + left_byte + right_byte: Set motor speeds (-100 to +100)
    - 'S': Stop motors
    - 'Q': Quit (stop motors and halt)

==============================================================================
"""

import network
import socket
import struct
import time
from machine import Pin, I2C, PWM

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# WiFi credentials - CHANGE THESE for your network
WIFI_SSID = "NeoNexus"
WIFI_PASSWORD = "userC013$"

# Network ports
DATA_PORT = 5005      # Robot sends sensor data here
COMMAND_PORT = 5006   # Robot receives commands here

# Sensor settings
SAMPLE_RATE_HZ = 500  # How many samples per second
BATCH_SIZE = 10       # Samples per UDP packet (reduces network overhead)

# ==============================================================================
# HARDWARE PIN DEFINITIONS (PicoCar configuration)
# ==============================================================================

# I2C for BMI160 IMU sensor
I2C_SDA = 14
I2C_SCL = 15
BMI160_ADDR = 0x68  # Default address (SDO=GND)

# Ultrasonic sensor (HC-SR04)
ULTRASONIC_TRIG = 0
ULTRASONIC_ECHO = 1

# Motor driver pins (PicoCar configuration)
MOTOR_LEFT_FWD = 13   # L_B - forward
MOTOR_LEFT_BWD = 12   # L_A - backward
MOTOR_RIGHT_FWD = 10  # R_A - forward
MOTOR_RIGHT_BWD = 11  # R_B - backward

# PWM frequency for motors
MOTOR_PWM_FREQ = 1000


# ==============================================================================
# BMI160 IMU SENSOR CLASS
# ==============================================================================

class BMI160:
    """
    Driver for BMI160 6-axis IMU (Inertial Measurement Unit).

    The BMI160 is a high-performance, low-power IMU containing:
      - 3-axis accelerometer: measures acceleration (including gravity)
      - 3-axis gyroscope: measures rotation speed

    Advantages over MPU6050:
      - Lower noise: 180 µg/√Hz vs 400 µg/√Hz (2.2x better)
      - Lower power: 0.95mA vs 3.9mA (4x better)
      - Faster startup time
      - Better temperature stability

    Communication: I2C protocol at 400kHz

    Data format (default ranges):
      - Accelerometer: ±2g range, 16384 LSB/g sensitivity
      - Gyroscope: ±250°/s range, 131.2 LSB/(°/s) sensitivity

    NOTE: Uses LITTLE-ENDIAN byte order (unlike MPU6050 which is big-endian)
    """

    # Chip ID (read from 0x00, should be 0xD1)
    CHIP_ID = 0xD1

    # Register addresses (from BMI160 datasheet)
    REG_CHIP_ID = 0x00       # Chip ID register
    REG_DATA_GYR = 0x0C      # First gyro data register (little-endian)
    REG_DATA_ACC = 0x12      # First accel data register (little-endian)
    REG_CMD = 0x7E           # Command register
    REG_ACC_CONF = 0x40      # Accelerometer config (ODR, bandwidth)
    REG_ACC_RANGE = 0x41     # Accelerometer range
    REG_GYR_CONF = 0x42      # Gyroscope config (ODR, bandwidth)
    REG_GYR_RANGE = 0x43     # Gyroscope range

    # Power mode commands (write to REG_CMD)
    CMD_SOFT_RESET = 0xB6    # Soft reset
    CMD_ACC_NORMAL = 0x11    # Accelerometer normal mode
    CMD_GYR_NORMAL = 0x15    # Gyroscope normal mode

    # Range settings
    ACC_RANGE_2G = 0x03      # ±2g (16384 LSB/g)
    GYR_RANGE_250 = 0x03     # ±250°/s (131.2 LSB/°/s)

    # ODR settings (bits 3:0 of ACC_CONF/GYR_CONF)
    ODR_800HZ = 0x0B         # 800 Hz output data rate
    BWP_NORMAL = 0x20        # Normal bandwidth mode (bits 5:4)

    def __init__(self, i2c, addr=0x68):
        """
        Initialize the BMI160 sensor.

        Args:
            i2c: I2C bus object
            addr: I2C address (0x68 if SDO=GND, 0x69 if SDO=VCC)
        """
        self.i2c = i2c
        self.addr = addr

        # Soft reset to ensure clean state
        self.i2c.writeto_mem(self.addr, self.REG_CMD, bytes([self.CMD_SOFT_RESET]))
        time.sleep_ms(100)  # Wait for reset to complete

        # Verify chip ID
        chip_id = self.i2c.readfrom_mem(self.addr, self.REG_CHIP_ID, 1)[0]
        if chip_id != self.CHIP_ID:
            raise RuntimeError(f"BMI160 not found! Chip ID: 0x{chip_id:02X} (expected 0xD1)")

        # Put accelerometer in normal mode
        self.i2c.writeto_mem(self.addr, self.REG_CMD, bytes([self.CMD_ACC_NORMAL]))
        time.sleep_ms(10)  # Wait for accel power-up

        # Put gyroscope in normal mode
        self.i2c.writeto_mem(self.addr, self.REG_CMD, bytes([self.CMD_GYR_NORMAL]))
        time.sleep_ms(100)  # Gyro needs longer to stabilize

        # Configure accelerometer: 800Hz ODR, normal bandwidth
        self.i2c.writeto_mem(self.addr, self.REG_ACC_CONF, bytes([self.ODR_800HZ | self.BWP_NORMAL]))
        self.i2c.writeto_mem(self.addr, self.REG_ACC_RANGE, bytes([self.ACC_RANGE_2G]))

        # Configure gyroscope: 800Hz ODR, normal bandwidth
        self.i2c.writeto_mem(self.addr, self.REG_GYR_CONF, bytes([self.ODR_800HZ | self.BWP_NORMAL]))
        self.i2c.writeto_mem(self.addr, self.REG_GYR_RANGE, bytes([self.GYR_RANGE_250]))

        time.sleep_ms(10)  # Let settings take effect

        print(f"BMI160 initialized (chip ID: 0x{chip_id:02X})")

    def read_raw(self):
        """
        Read raw 16-bit values from all 6 axes in one I2C transaction.

        BMI160 data layout (starting at 0x0C):
          0x0C-0x0D: Gyro X (low, high) - little-endian
          0x0E-0x0F: Gyro Y
          0x10-0x11: Gyro Z
          0x12-0x13: Accel X
          0x14-0x15: Accel Y
          0x16-0x17: Accel Z

        Returns:
            tuple: (ax, ay, az, gx, gy, gz) as raw signed integers
        """
        # Read all 12 bytes in one transaction (more efficient)
        data = self.i2c.readfrom_mem(self.addr, self.REG_DATA_GYR, 12)

        # Unpack as 6 little-endian signed 16-bit integers
        gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', data)

        return ax, ay, az, gx, gy, gz

    def read_scaled(self):
        """
        Read accelerometer and gyroscope with physical units.

        Scaling factors (for ±2g and ±250°/s ranges):
          - Accelerometer: raw / 16384.0 = g
          - Gyroscope: raw / 131.2 = °/s

        Returns:
            tuple: (ax, ay, az, gx, gy, gz)
                   accelerometer in g, gyroscope in °/s
        """
        ax, ay, az, gx, gy, gz = self.read_raw()

        return (
            ax / 16384.0,  # g
            ay / 16384.0,  # g
            az / 16384.0,  # g
            gx / 131.2,    # °/s
            gy / 131.2,    # °/s
            gz / 131.2     # °/s
        )


# ==============================================================================
# ULTRASONIC SENSOR CLASS
# ==============================================================================

class UltrasonicSensor:
    """
    Driver for HC-SR04 ultrasonic distance sensor.

    How it works:
      1. Send a 10µs pulse on TRIG pin
      2. Sensor emits 8 ultrasonic pulses at 40kHz
      3. Pulses reflect off obstacle and return
      4. ECHO pin goes HIGH for duration proportional to distance
      5. Distance = (echo_time × speed_of_sound) / 2

    Speed of sound: ~343 m/s at 20°C = 0.0343 cm/µs
    Distance = echo_µs × 0.0343 / 2 = echo_µs × 0.01715 cm

    Range: 2cm - 400cm (practical: 2cm - 200cm)
    """

    def __init__(self, trig_pin, echo_pin):
        """
        Initialize ultrasonic sensor.

        Args:
            trig_pin: GPIO number for TRIG (output)
            echo_pin: GPIO number for ECHO (input)
        """
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)  # Start with trigger low

        print(f"Ultrasonic sensor initialized (TRIG={trig_pin}, ECHO={echo_pin})")

    def measure_cm(self, timeout_us=30000):
        """
        Measure distance in centimeters.

        Args:
            timeout_us: Maximum time to wait for echo (default 30ms ≈ 5m range)

        Returns:
            float: Distance in cm, or -1 if timeout/error
        """
        # Ensure trigger is low
        self.trig.value(0)
        time.sleep_us(2)

        # Send 10µs trigger pulse
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        # Wait for echo to go HIGH (start of echo)
        start_wait = time.ticks_us()
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start_wait) > timeout_us:
                return -1  # Timeout waiting for echo start

        # Record start time
        echo_start = time.ticks_us()

        # Wait for echo to go LOW (end of echo)
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), echo_start) > timeout_us:
                return -1  # Timeout waiting for echo end

        # Calculate echo duration
        echo_duration = time.ticks_diff(time.ticks_us(), echo_start)

        # Convert to distance
        # Distance = time × speed / 2 (divide by 2 for round trip)
        # Speed of sound ≈ 343 m/s = 0.0343 cm/µs
        distance_cm = echo_duration * 0.01715

        return distance_cm


# ==============================================================================
# MOTOR CONTROLLER CLASS
# ==============================================================================

class MotorController:
    """
    Controls two DC motors using PWM.

    Each motor has two pins:
      - Forward pin: PWM for forward rotation
      - Backward pin: PWM for backward rotation

    Speed control:
      - Speed range: -100 to +100
      - Positive = forward, Negative = backward
      - PWM duty cycle: |speed| × 655.35 (0-65535 range)
    """

    def __init__(self, left_fwd, left_bwd, right_fwd, right_bwd, freq=1000):
        """
        Initialize motor controller.

        Args:
            left_fwd: GPIO for left motor forward
            left_bwd: GPIO for left motor backward
            right_fwd: GPIO for right motor forward
            right_bwd: GPIO for right motor backward
            freq: PWM frequency in Hz
        """
        # Create PWM objects for each motor pin
        self.left_fwd = PWM(Pin(left_fwd))
        self.left_bwd = PWM(Pin(left_bwd))
        self.right_fwd = PWM(Pin(right_fwd))
        self.right_bwd = PWM(Pin(right_bwd))

        # Set PWM frequency
        for pwm in [self.left_fwd, self.left_bwd, self.right_fwd, self.right_bwd]:
            pwm.freq(freq)
            pwm.duty_u16(0)  # Start stopped

        # Track current speeds
        self.left_speed = 0
        self.right_speed = 0

        print(f"Motors initialized (L:{left_fwd}/{left_bwd}, R:{right_fwd}/{right_bwd})")

    def set_speed(self, left, right):
        """
        Set motor speeds.

        Args:
            left: Left motor speed (-100 to +100)
            right: Right motor speed (-100 to +100)
        """
        # Clamp to valid range
        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        self.left_speed = left
        self.right_speed = right

        # Convert speed to PWM duty (0-65535)
        # |speed| / 100 × 65535 = duty
        self._set_motor(self.left_fwd, self.left_bwd, left)
        self._set_motor(self.right_fwd, self.right_bwd, right)

    def _set_motor(self, fwd_pwm, bwd_pwm, speed):
        """Set individual motor speed."""
        duty = int(abs(speed) * 655.35)

        if speed > 0:
            fwd_pwm.duty_u16(duty)
            bwd_pwm.duty_u16(0)
        elif speed < 0:
            fwd_pwm.duty_u16(0)
            bwd_pwm.duty_u16(duty)
        else:
            fwd_pwm.duty_u16(0)
            bwd_pwm.duty_u16(0)

    def stop(self):
        """Stop both motors immediately."""
        self.set_speed(0, 0)

    def cleanup(self):
        """Release PWM resources."""
        for pwm in [self.left_fwd, self.left_bwd, self.right_fwd, self.right_bwd]:
            pwm.duty_u16(0)
            pwm.deinit()


# ==============================================================================
# WIFI CONNECTION
# ==============================================================================

def connect_wifi(ssid, password, timeout=10):
    """
    Connect to WiFi network.

    Args:
        ssid: Network name
        password: Network password
        timeout: Max seconds to wait for connection

    Returns:
        str: IP address if connected, None if failed
    """
    print(f"Connecting to WiFi: {ssid}")

    # Create WiFi interface in station (client) mode
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # Start connection
    wlan.connect(ssid, password)

    # Wait for connection
    start = time.time()
    while not wlan.isconnected():
        if time.time() - start > timeout:
            print("WiFi connection timeout!")
            return None
        print(".", end="")
        time.sleep(0.5)

    # Get IP address
    ip = wlan.ifconfig()[0]
    print(f"\nConnected! IP: {ip}")
    return ip


# ==============================================================================
# DATA STREAMING
# ==============================================================================

class DataStreamer:
    """
    Streams sensor data to host computer via UDP.

    Data is batched into packets for efficiency:
      - Sending one sample at a time = high overhead
      - Batching 10 samples = 10× less overhead

    Packet structure:
      [type:1][count:1][sample:36][sample:36]...
    """

    # Packet type identifier
    PACKET_TYPE_SENSOR = 0x01

    def __init__(self, host_ip, host_port):
        """
        Initialize data streamer.

        Args:
            host_ip: IP address of host computer
            host_port: UDP port to send data to
        """
        self.host_ip = host_ip
        self.host_port = host_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Buffer for batching samples
        self.batch = []

        print(f"Data streamer ready -> {host_ip}:{host_port}")

    def add_sample(self, timestamp, ax, ay, az, gx, gy, gz, ultrasonic):
        """
        Add a sensor sample to the batch.

        Args:
            timestamp: Microseconds since boot (from time.ticks_us())
            ax, ay, az: Accelerometer in g
            gx, gy, gz: Gyroscope in °/s
            ultrasonic: Distance in cm
        """
        # Pack sample as binary data
        # '<' = little-endian, 'Q' = uint64, 'f' = float32
        sample = struct.pack('<Q7f',
            timestamp,
            ax, ay, az,
            gx, gy, gz,
            ultrasonic
        )
        self.batch.append(sample)

    def send_batch(self):
        """Send accumulated samples as one UDP packet."""
        if not self.batch:
            return

        # Build packet: header + samples
        header = struct.pack('BB', self.PACKET_TYPE_SENSOR, len(self.batch))
        packet = header + b''.join(self.batch)

        # Send to host
        try:
            self.sock.sendto(packet, (self.host_ip, self.host_port))
        except Exception as e:
            print(f"Send error: {e}")

        # Clear batch
        self.batch = []

    def close(self):
        """Close socket."""
        self.sock.close()


# ==============================================================================
# COMMAND RECEIVER
# ==============================================================================

class CommandReceiver:
    """
    Receives motor commands from host via UDP.

    Uses non-blocking socket to check for commands without waiting.

    Command format:
      - 'M' + left_byte + right_byte: Set motor speeds
        (speeds are signed bytes: -100 to +100)
      - 'S': Stop motors
      - 'Q': Quit
    """

    def __init__(self, port):
        """
        Initialize command receiver.

        Args:
            port: UDP port to listen on
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.setblocking(False)  # Non-blocking mode

        self.quit_requested = False

        print(f"Command receiver listening on port {port}")

    def check_commands(self, motors):
        """
        Check for and process any pending commands.

        Args:
            motors: MotorController instance to control

        Returns:
            bool: True if should continue, False if quit requested
        """
        try:
            # Try to receive command (non-blocking)
            data, addr = self.sock.recvfrom(64)

            if len(data) >= 1:
                cmd = chr(data[0])

                if cmd == 'M' and len(data) >= 3:
                    # Motor command: 'M' + left + right
                    # Convert unsigned bytes to signed (-100 to +100)
                    left = data[1] if data[1] < 128 else data[1] - 256
                    right = data[2] if data[2] < 128 else data[2] - 256
                    motors.set_speed(left, right)

                elif cmd == 'S':
                    # Stop motors
                    motors.stop()

                elif cmd == 'Q':
                    # Quit
                    motors.stop()
                    self.quit_requested = True
                    return False

        except OSError:
            # No data available (expected in non-blocking mode)
            pass

        return True

    def close(self):
        """Close socket."""
        self.sock.close()


# ==============================================================================
# MAIN LOOP
# ==============================================================================

def main():
    """
    Main data collection loop.

    This function:
      1. Initializes all hardware and network connections
      2. Reads sensors at fixed rate (SAMPLE_RATE_HZ)
      3. Sends batched data to host (every BATCH_SIZE samples)
      4. Processes motor commands from host
      5. Continues until quit command received
    """
    print("\n" + "=" * 50)
    print("PICO DATA COLLECTOR")
    print("Educational Sensor Data Collection")
    print("=" * 50 + "\n")

    # -------------------------------------------------------------------------
    # STEP 1: Connect to WiFi
    # -------------------------------------------------------------------------
    ip = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    if not ip:
        print("Failed to connect to WiFi. Check credentials.")
        return

    # Calculate broadcast IP for network discovery
    # Robot will broadcast data until host responds with a command
    broadcast_ip = '.'.join(ip.split('.')[:-1]) + '.255'
    print(f"Will broadcast to {broadcast_ip} until host connects")

    # For direct connection (faster), uncomment and set your host IP:
    # broadcast_ip = "192.168.1.100"  # Replace with your computer's IP

    # -------------------------------------------------------------------------
    # STEP 2: Initialize hardware
    # -------------------------------------------------------------------------
    print("\nInitializing hardware...")

    # I2C bus for IMU (I2C1 for GP14/GP15)
    # Note: Pico I2C pin mapping:
    #   I2C0: SDA=GP0,4,8,12,16,20  SCL=GP1,5,9,13,17,21
    #   I2C1: SDA=GP2,6,10,14,18,26 SCL=GP3,7,11,15,19,27
    i2c = I2C(1, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)

    # Initialize sensors
    imu = BMI160(i2c, BMI160_ADDR)
    ultrasonic = UltrasonicSensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO)

    # Initialize motors
    motors = MotorController(
        MOTOR_LEFT_FWD, MOTOR_LEFT_BWD,
        MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD,
        MOTOR_PWM_FREQ
    )

    # -------------------------------------------------------------------------
    # STEP 3: Initialize network
    # -------------------------------------------------------------------------
    print("\nInitializing network...")

    # Note: In real use, host_ip should be set to actual host computer IP
    # For now, we'll need the host to tell us its IP
    streamer = DataStreamer("192.168.28.104", DATA_PORT)  # Will be updated
    receiver = CommandReceiver(COMMAND_PORT)

    # -------------------------------------------------------------------------
    # STEP 4: Main sensor loop
    # -------------------------------------------------------------------------
    print("\n" + "-" * 50)
    print(f"Starting data collection at {SAMPLE_RATE_HZ} Hz")
    print(f"Batch size: {BATCH_SIZE} samples per packet")
    print("Waiting for host connection...")
    print("-" * 50 + "\n")

    # Calculate delay between samples
    sample_interval_us = 1_000_000 // SAMPLE_RATE_HZ

    # For ultrasonic timing (measure less frequently - it's slow)
    ultrasonic_interval_ms = 50  # 20Hz
    last_ultrasonic_time = 0
    last_ultrasonic_value = 0

    # Timing
    last_sample_time = time.ticks_us()
    sample_count = 0

    # Track if host responded (for status display)
    host_responded = False

    try:
        while True:
            # Check timing
            now = time.ticks_us()
            elapsed = time.ticks_diff(now, last_sample_time)

            if elapsed >= sample_interval_us:
                last_sample_time = now

                # Read IMU (fast - can do every sample)
                ax, ay, az, gx, gy, gz = imu.read_scaled()

                # Read ultrasonic (slow - do less frequently)
                now_ms = time.ticks_ms()
                if time.ticks_diff(now_ms, last_ultrasonic_time) >= ultrasonic_interval_ms:
                    last_ultrasonic_time = now_ms
                    us_reading = ultrasonic.measure_cm()
                    if us_reading > 0:
                        last_ultrasonic_value = us_reading

                # Add sample to batch (always send - host IP set at startup)
                streamer.add_sample(
                    now,
                    ax, ay, az,
                    gx, gy, gz,
                    last_ultrasonic_value
                )

                # Send batch when full
                if len(streamer.batch) >= BATCH_SIZE:
                    streamer.send_batch()

                sample_count += 1

                # Print status occasionally
                if sample_count % 500 == 0:
                    print(f"Samples: {sample_count}, "
                          f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}), "
                          f"US: {last_ultrasonic_value:.1f}cm, "
                          f"Motors: {motors.left_speed}/{motors.right_speed}")

            # Check for commands (non-blocking)
            try:
                data, addr = receiver.sock.recvfrom(64)

                # First packet confirms host is listening
                if not host_responded:
                    host_responded = True
                    print(f"\nHost connected: {addr[0]}")

                # Process command
                if len(data) >= 1:
                    cmd = chr(data[0])

                    if cmd == 'M' and len(data) >= 3:
                        left = data[1] if data[1] < 128 else data[1] - 256
                        right = data[2] if data[2] < 128 else data[2] - 256
                        motors.set_speed(left, right)

                    elif cmd == 'S':
                        motors.stop()

                    elif cmd == 'Q':
                        print("\nQuit command received")
                        break

            except OSError:
                pass  # No command waiting

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Cleanup
        print("\nCleaning up...")
        motors.stop()
        motors.cleanup()
        streamer.close()
        receiver.close()
        print("Done.")


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == '__main__':
    main()
