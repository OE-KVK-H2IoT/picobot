"""
==============================================================================
LAB 7: Full System - WiFi, Streaming, Ultrasonic & Remote Control
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Connect robot to WiFi network
  2. Stream IMU + ultrasonic data over UDP
  3. Receive remote control commands from host PC
  4. Visualize everything in real-time

SYSTEM ARCHITECTURE:
--------------------

    ┌──────────────────────────────────────────────────────┐
    │                PICO W (Robot)                         │
    │  ┌─────────┐   ┌─────────┐   ┌─────────────────┐    │
    │  │BMI160 or│──>│ Robot   │──>│ UDP Streamer    │───────> WiFi (port 5005)
    │  │ MPU6050 │   │ State   │   │ + Ultrasonic    │    │
    │  └─────────┘   └─────────┘   └─────────────────┘    │
    │       │              │                               │
    │       v              v              ┌─────────────┐  │
    │  ┌─────────┐   ┌─────────┐    <─────│ Control     │<──── WiFi (port 5006)
    │  │ Motors  │<──│  LEDs   │          │ Receiver    │  │
    │  └─────────┘   └─────────┘          └─────────────┘  │
    └──────────────────────────────────────────────────────┘
                           │                    ^
                      UDP packets          Commands
                           │                    │
                           v                    │
    ┌──────────────────────────────────────────────────────┐
    │                    HOST PC                            │
    │  ┌─────────────────────────────────────────────────┐ │
    │  │               host.py (PyQt5)                    │ │
    │  │  - 3D visualization                              │ │
    │  │  - Path tracking with ultrasonic cone            │ │
    │  │  - Keyboard control (arrow keys)                 │ │
    │  └─────────────────────────────────────────────────┘ │
    └──────────────────────────────────────────────────────┘

HOW TO RUN:
-----------
  1. Edit WIFI_SSID, WIFI_PASSWORD, HOST_IP below
  2. Deploy libs/ + this main.py to Pico
  3. On host PC: cd src/host && python host.py
  4. Power on robot - streams data + accepts keyboard control

KEYBOARD CONTROLS (on host):
----------------------------
  Arrow Up/Down    - Forward/Backward
  Arrow Left/Right - Turn Left/Right
  Space            - Stop
  R                - Reset position display
  F                - Toggle filters
  Q/Esc            - Quit

==============================================================================
"""

import time
import network
import socket
import struct
from machine import Pin, I2C

from picobot import PicoBot
from robot_state import RobotState, IMUStreamer
from speed_estimator import SpeedEstimator

# IMU type: 'auto', 'mpu6050', or 'bmi160'
IMU_TYPE = 'auto'


# ==============================================================================
# CONFIGURATION - EDIT THESE FOR YOUR SETUP
# ==============================================================================

# WiFi credentials
WIFI_SSID = 'NeoNexus'        # <-- Change this!
WIFI_PASSWORD = 'userC013$' # <-- Change this!

# Host PC IP address (run 'ipconfig' or 'ifconfig' on your PC to find it)
HOST_IP = '192.168.28.175'           # <-- Change this! Use "auto" for discovery.
HOST_PORT = 5005                     # UDP port for streaming data
BUMP_PER_CM = None                   # Set from host calibration (e.g., 0.45)

# Control receiver port (host sends commands here)
CONTROL_PORT = 5006

# Yaw hold (keep straight using gyro heading)
YAW_KP = 1.2
YAW_MAX_CORRECTION = 40
YAW_HOLD_DEADBAND = 8
YAW_CAPTURE_SPEED = 20

# I2C pins for IMU
I2C_SCL_PIN = 15
I2C_SDA_PIN = 14

# Sampling configuration
SAMPLE_RATE_HZ = 500      # Increased to match sensor's 500Hz output
BATCH_SIZE = 20           # More samples per packet
SAMPLE_DELAY_MS = 1000 // SAMPLE_RATE_HZ  # = 2ms

# Filtering (reduce drift)
ENABLE_FILTERS = True
FILTER_ALPHA = 0.3        # 0.1=smooth, 0.5=responsive


# ==============================================================================
# WIFI FUNCTIONS
# ==============================================================================

def connect_wifi(ssid, password, timeout_s=15):
    """Connect to WiFi network."""
    print(f"Connecting to WiFi: {ssid}")

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    for i in range(timeout_s):
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print(f"Connected! Robot IP: {ip}")
            return wlan
        print(f"  Waiting... ({i+1}/{timeout_s})")
        time.sleep(1)

    print("WiFi connection failed!")
    return None


# ==============================================================================
# CONTROL RECEIVER
# ==============================================================================

class ControlReceiver:
    """
    Receives motor control commands from host PC over UDP.

    Protocol:
      - Packet: 'M' (1 byte) + left_speed (2 bytes) + right_speed (2 bytes)
      - Speeds are signed 16-bit integers (-255 to 255)
    """

    def __init__(self, port=5006):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

        self.left_speed = 0
        self.right_speed = 0
        self.last_command_time = 0
        self.command_timeout_ms = 500  # Stop if no command for 500ms
        self.host_address = None

    def check_commands(self):
        """
        Check for incoming control commands.
        Returns (left_speed, right_speed) or None if no new command.
        """
        try:
            data, addr = self.sock.recvfrom(64)
            if self.host_address is None:
                self.host_address = addr
                print(f"Control connected from {addr[0]}:{addr[1]}")

            if len(data) >= 5 and data[0:1] == b'M':
                left, right = struct.unpack("<hh", data[1:5])
                self.left_speed = max(-255, min(255, left))
                self.right_speed = max(-255, min(255, right))
                self.last_command_time = time.ticks_ms()
                return (self.left_speed, self.right_speed)

        except OSError:
            pass  # No data available

        return None

    def get_speeds(self):
        """
        Get current motor speeds.
        Returns (0, 0) if commands have timed out.
        """
        # Check for timeout
        if time.ticks_diff(time.ticks_ms(), self.last_command_time) > self.command_timeout_ms:
            self.left_speed = 0
            self.right_speed = 0

        return (self.left_speed, self.right_speed)

    def is_active(self):
        """Return True if receiving commands (not timed out)."""
        return time.ticks_diff(time.ticks_ms(), self.last_command_time) < self.command_timeout_ms


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*60)
    print("LAB 7: Full System with Remote Control")
    print("="*60)

    # -------------------------------------------------------------------------
    # Step 1: Initialize hardware
    # -------------------------------------------------------------------------
    print("\n[1/6] Initializing hardware...")
    robot = PicoBot(num_leds=4)

    robot.leds.fill(255, 255, 0)  # Yellow = starting
    robot.leds.show()

    # Initialize IMU (auto-detect BMI160 vs MPU6050)
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    devices = i2c.scan()
    print(f"  I2C devices: {[hex(d) for d in devices]}")

    if 0x68 not in devices:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        print("ERROR: IMU not found at 0x68!")
        return

    # Auto-detect IMU type
    sensor = None
    imu_name = "Unknown"

    if IMU_TYPE == 'auto':
        # Check chip ID to detect BMI160 (0xD1) vs MPU6050
        chip_id = i2c.readfrom_mem(0x68, 0x00, 1)[0]
        if chip_id == 0xD1:
            detected_type = 'bmi160'
        else:
            detected_type = 'mpu6050'
    else:
        detected_type = IMU_TYPE

    if detected_type == 'bmi160':
        from bmi160 import BMI160
        sensor = BMI160(i2c, dlpf=2)
        imu_name = "BMI160"
    else:
        from mpu6050 import MPU6050
        sensor = MPU6050(i2c, dlpf=2)
        imu_name = "MPU6050"

    print(f"  {imu_name} OK!")

    # -------------------------------------------------------------------------
    # Step 2: Connect to WiFi
    # -------------------------------------------------------------------------
    print("\n[2/6] Connecting to WiFi...")
    robot.leds.fill(0, 255, 255)  # Cyan = connecting
    robot.leds.show()

    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)

    if not wlan:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        print("Cannot continue without WiFi!")
        return

    # -------------------------------------------------------------------------
    # Step 3: Initialize control receiver
    # -------------------------------------------------------------------------
    print("\n[3/6] Starting control receiver...")
    print(f"  Listening on UDP port {CONTROL_PORT}")
    controller = ControlReceiver(CONTROL_PORT)
    auto_host = HOST_IP in (None, "", "auto")
    speed_est = SpeedEstimator(imu=sensor, ultrasonic=robot.ultrasonic)
    if BUMP_PER_CM:
        speed_est.load_bump_calibration(BUMP_PER_CM)

    # -------------------------------------------------------------------------
    # Step 4: Calibrate IMU
    # -------------------------------------------------------------------------
    print("\n[4/6] Calibrating IMU...")
    robot.leds.fill(255, 165, 0)  # Orange = calibrating
    robot.leds.show()

    state = RobotState(enable_filters=ENABLE_FILTERS, filter_alpha=FILTER_ALPHA)

    print("*** KEEP ROBOT STILL! ***")
    time.sleep(1)
    state.calibrate(sensor, num_samples=100, delay_ms=10)

    # -------------------------------------------------------------------------
    # Step 5: Initialize streamer with ultrasonic
    # -------------------------------------------------------------------------
    print("\n[5/6] Initializing data streamer...")
    if auto_host:
        print("  Target: auto-discovery")
    else:
        print(f"  Target: {HOST_IP}:{HOST_PORT}")
    print(f"  Ultrasonic: Enabled")
    print(f"  Filters: {'Enabled' if ENABLE_FILTERS else 'Disabled'}")

    streamer = IMUStreamer(
        sensor,
        robot_state=state,
        ultrasonic=robot.ultrasonic,  # Include ultrasonic!
        udp_ip=None if auto_host else HOST_IP,
        udp_port=HOST_PORT
    )

    # -------------------------------------------------------------------------
    # Step 6: Main loop
    # -------------------------------------------------------------------------
    print("\n[6/6] Starting main loop!")
    print("="*60)
    print("ROBOT READY")
    if auto_host:
        print("  Data streaming: waiting for host discovery...")
    else:
        print(f"  Data streaming to {HOST_IP}:{HOST_PORT}")
    print(f"  Control listening on port {CONTROL_PORT}")
    print("  Run host.py on your PC to visualize and control")
    print("="*60)

    robot.leds.fill(0, 255, 0)  # Green = ready
    robot.leds.show()

    loop_count = 0
    remote_control_active = False
    target_heading = None

    while True:
        # --- Read IMU batch and update state ---
        batch = streamer.read_batch(batch_size=BATCH_SIZE, delay_ms=SAMPLE_DELAY_MS)

        # --- Send data to host ---
        streamer.send_batch(batch)

        # --- Latest state ---
        motion = state.get_state()

        # --- Check for remote control commands ---
        cmd = controller.check_commands()
        if cmd is not None:
            if not remote_control_active:
                print("Remote control ACTIVE")
                remote_control_active = True
        if auto_host and controller.host_address and streamer.udp_ip is None:
            streamer.udp_ip = controller.host_address[0]
            if streamer._sock is None:
                streamer._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"Host detected: {streamer.udp_ip}. Streaming enabled.")

        # --- Apply motor control ---
        if controller.is_active():
            # Remote control mode
            left, right = controller.get_speeds()
            heading = motion.get('heading_deg', 0.0)

            if abs(left - right) <= YAW_HOLD_DEADBAND and (abs(left) + abs(right)) / 2 >= YAW_CAPTURE_SPEED:
                if target_heading is None:
                    target_heading = heading
                error = target_heading - heading
                while error > 180:
                    error -= 360
                while error < -180:
                    error += 360
                direction = 1 if (left + right) >= 0 else -1
                correction = max(-YAW_MAX_CORRECTION, min(YAW_MAX_CORRECTION, YAW_KP * error))
                correction *= direction
                left = int(left + correction)
                right = int(right - correction)
            else:
                target_heading = None
            robot.motors.set_speeds(left, right)
            speed_est.update_motor_command(left, right)

            # LED feedback: Blue when remote controlled
            if loop_count % 10 == 0:
                robot.leds.fill(0, 100, 255)
                robot.leds.show()
        else:
            if remote_control_active:
                print("Remote control INACTIVE (timeout)")
                remote_control_active = False
                robot.motors.stop()
                robot.leds.fill(0, 255, 0)  # Back to green
                robot.leds.show()
            speed_est.update_motor_command(0, 0)
            target_heading = None

        # --- Update estimators ---
        speed_est.update()

        # --- Print status periodically ---
        loop_count += 1
        if loop_count % 40 == 0:
            us_dist = batch[-1].get('ultrasonic_cm', -1)
            us_str = f"{us_dist:.0f}cm" if us_dist > 0 else "N/A"
            stat_str = "STILL" if motion.get('is_stationary', False) else "MOVE"

            speed_cm_s = speed_est.get_speed()
            print(f"[{stat_str}] Head:{motion['heading_deg']:+6.1f}° "
                  f"Pos:({motion['position_x']:+.2f},{motion['position_y']:+.2f}) "
                  f"US:{us_str} Speed:{speed_cm_s:5.1f}cm/s")

        # =====================================================================
        # YOUR AUTONOMOUS BEHAVIOR CODE HERE
        # =====================================================================
        #
        # The remote control takes priority, but you can add autonomous
        # behaviors that activate when remote control is not active.
        #
        # Example: Autonomous obstacle avoidance when not remote controlled
        # -----------------------------------------------------------------
        # if not controller.is_active():
        #     distance = batch[-1].get('ultrasonic_cm', -1)
        #     if distance > 0 and distance < 20:
        #         robot.motors.backward(100)
        #         time.sleep(0.3)
        #         robot.motors.turn_right(100)
        #         time.sleep(0.3)
        #     elif distance > 0:
        #         robot.motors.forward(100)
        #     else:
        #         robot.motors.stop()
        #
        # =====================================================================


# ==============================================================================
# RUN
# ==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import sys
        sys.print_exception(e)
    finally:
        # Safety: stop motors
        from picobot import Motors
        Motors().stop()
