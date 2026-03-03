"""
Full Feature Demo (Lab 7 Based) - Host Viewer + Remote Control
==============================================================

This demo pairs with the full-feature host viewer to show:
  - IMU + ultrasonic streaming over WiFi (UDP)
  - 3D orientation + path visualization in host/viewer.py
  - Keyboard remote control from the host

USAGE
-----
1) Edit WIFI_SSID, WIFI_PASSWORD, HOST_IP for your network.
2) Upload libs + this file to the Pico:
   ampy --port /dev/ttyACM0 put src/pico/lib/picobot.py picobot.py
   ampy --port /dev/ttyACM0 put src/pico/lib/bmi160.py bmi160.py
   ampy --port /dev/ttyACM0 put src/pico/lib/mpu6050.py mpu6050.py
   ampy --port /dev/ttyACM0 put src/pico/lib/robot_state.py robot_state.py
   ampy --port /dev/ttyACM0 put src/pico/lib/control_receiver.py control_receiver.py
   ampy --port /dev/ttyACM0 put src/examples/full_feature_demo.py main.py
3) On the host PC:
   python src/host/viewer.py

HOST CONTROLS (viewer.py)
-------------------------
  Arrow Keys  - Drive
  Space       - Stop
  R           - Reset path
  F           - Toggle filters
  A           - Toggle AI prediction
  Q/Esc       - Quit
"""

import time
import network
import socket
from machine import Pin, I2C

from picobot import PicoBot
from oled import OLEDStatus
from robot_state import RobotState, IMUStreamer
from speed_estimator import SpeedEstimator
from control_receiver import ControlReceiver

# IMU type: 'auto', 'mpu6050', or 'bmi160'
IMU_TYPE = "auto"

# ==============================================================================
# CONFIGURATION - EDIT THESE
# ==============================================================================

WIFI_SSID = "NeoNexus"
WIFI_PASSWORD = "userC013$"

HOST_IP = "192.168.28.102"  # Use "auto" for discovery
HOST_PORT = 5005
CONTROL_PORT = 5006
BUMP_PER_CM = None  # Set from host calibration (e.g., 0.45)

# Yaw hold (keep straight using gyro heading)
YAW_KP = 1.2
YAW_MAX_CORRECTION = 40
YAW_HOLD_DEADBAND = 8
YAW_CAPTURE_SPEED = 20

I2C_SCL_PIN = 15
I2C_SDA_PIN = 14

SAMPLE_RATE_HZ = 500
BATCH_SIZE = 20
SAMPLE_DELAY_MS = 1000 // SAMPLE_RATE_HZ
CONTROL_INTERVAL_MS = 20  # Control loop rate (~50 Hz)

ENABLE_FILTERS = True
FILTER_ALPHA = 0.3

# ==============================================================================
# WIFI
# ==============================================================================

def connect_wifi(ssid, password, timeout_s=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    print(f"Connecting to {ssid}...")
    for i in range(timeout_s):
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print(f"Connected! Robot IP: {ip}")
            return wlan
        print(f"  Waiting... ({i + 1}/{timeout_s})")
        time.sleep(1)

    print("WiFi connection failed!")
    return None

# ==============================================================================
# IMU SETUP
# ==============================================================================

def init_imu(i2c):
    devices = i2c.scan()
    print(f"I2C devices: {[hex(d) for d in devices]}")

    if 0x68 not in devices:
        print("ERROR: IMU not found at 0x68!")
        return None, "None"

    if IMU_TYPE == "auto":
        chip_id = i2c.readfrom_mem(0x68, 0x00, 1)[0]
        detected = "bmi160" if chip_id == 0xD1 else "mpu6050"
    else:
        detected = IMU_TYPE

    if detected == "bmi160":
        from bmi160 import BMI160
        sensor = BMI160(i2c, dlpf=2)
        return sensor, "BMI160"

    from mpu6050 import MPU6050
    sensor = MPU6050(i2c, dlpf=2)
    return sensor, "MPU6050"

# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 60)
    print("FULL FEATURE DEMO (LAB 7 BASED)")
    print("=" * 60)

    robot = PicoBot(num_leds=4)
    robot.leds.fill(255, 255, 0)  # Yellow = booting
    robot.leds.show()

    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    oled = OLEDStatus(i2c)
    if oled.available:
        oled.show_two_lines("BOOT", "Init...")
    sensor, imu_name = init_imu(i2c)
    if not sensor:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        if oled.available:
            oled.show_two_lines("IMU FAIL", "0x68?")
        return
    print(f"{imu_name} OK")
    if oled.available:
        oled.show_two_lines("IMU OK", imu_name)

    robot.leds.fill(0, 255, 255)  # Cyan = WiFi
    robot.leds.show()
    if oled.available:
        oled.show_two_lines("WIFI", "Connecting")
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    if not wlan:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        if oled.available:
            oled.show_two_lines("WIFI FAIL", "Check SSID")
        return
    if oled.available:
        oled.show_two_lines("WIFI OK", wlan.ifconfig()[0])

    robot.leds.fill(255, 165, 0)  # Orange = calibrating
    robot.leds.show()
    if oled.available:
        oled.show_two_lines("IMU", "Calibrating")

    state = RobotState(enable_filters=ENABLE_FILTERS, filter_alpha=FILTER_ALPHA)
    print("Calibrating IMU - keep robot still...")
    time.sleep(1)
    state.calibrate(sensor, num_samples=100, delay_ms=10)
    if oled.available:
        oled.show_two_lines("CAL DONE", "Ready")

    auto_host = HOST_IP in (None, "", "auto")
    controller = ControlReceiver(port=CONTROL_PORT, timeout_ms=500)
    speed_est = SpeedEstimator(imu=sensor, ultrasonic=robot.ultrasonic)
    if BUMP_PER_CM:
        speed_est.load_bump_calibration(BUMP_PER_CM)
    if auto_host:
        ip = wlan.ifconfig()[0]
        broadcast_ip = ".".join(ip.split(".")[:-1]) + ".255"
        udp_ip = broadcast_ip
        print(f"  Auto-host: broadcasting to {broadcast_ip}")
    else:
        udp_ip = HOST_IP
    streamer = IMUStreamer(
        sensor,
        robot_state=state,
        ultrasonic=robot.ultrasonic,
        udp_ip=udp_ip,
        udp_port=HOST_PORT,
    )
    print(f"  UDP target: {streamer.udp_ip}:{HOST_PORT}")

    print("Robot ready")
    if auto_host:
        print("  Streaming: waiting for host discovery...")
    else:
        print(f"  Streaming to {HOST_IP}:{HOST_PORT}")
    print(f"  Control on port {CONTROL_PORT}")
    print("  Run: python src/host/viewer.py")

    robot.leds.fill(0, 255, 0)  # Green = ready
    robot.leds.show()
    if oled.available:
        oled.show_two_lines("READY", "Streaming")

    loop_count = 0
    remote_active = False
    target_heading = None

    sample_interval_us = 1_000_000 // SAMPLE_RATE_HZ
    next_sample_us = time.ticks_us()
    next_control_ms = time.ticks_ms()
    batch = []
    last_us = -1
    packet_count = 0
    last_report_ms = time.ticks_ms()

    while True:
        # --- Sensor/logging loop (high rate) ---
        now_us = time.ticks_us()
        if time.ticks_diff(now_us, next_sample_us) >= 0:
            sample = streamer.read_sample()
            batch.append(sample)
            if len(batch) >= BATCH_SIZE:
                streamer.send_batch(batch)
                packet_count += 1
                batch = []
            next_sample_us = time.ticks_add(next_sample_us, sample_interval_us)
            last_us = sample.get("ultrasonic_cm", last_us)

        # --- Control loop (lower rate) ---
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, next_control_ms) < 0:
            continue
        next_control_ms = time.ticks_add(next_control_ms, CONTROL_INTERVAL_MS)

        if controller.check_commands() is not None and not remote_active:
            print("Remote control ACTIVE")
            remote_active = True
        if auto_host and controller.host_address and streamer.udp_ip is None:
            streamer.udp_ip = controller.host_address[0]
            if streamer._sock is None:
                streamer._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"Host detected: {streamer.udp_ip}. Streaming enabled.")

        motion = state.get_state()
        if time.ticks_diff(now_ms, last_report_ms) > 1000:
            last_report_ms = now_ms
            print(f"Packets/s: {packet_count} -> {streamer.udp_ip}")
            packet_count = 0

        if controller.is_active():
            left, right = controller.get_speeds()
            heading = motion.get("heading_deg", 0.0)

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
            if loop_count % 2 == 0:
                robot.leds.fill(0, 100, 255)  # Blue = remote
                robot.leds.show()
        else:
            if remote_active:
                print("Remote control INACTIVE (timeout)")
                remote_active = False
            robot.motors.stop()
            speed_est.update_motor_command(0, 0)
            target_heading = None
            if loop_count % 10 == 0:
                robot.leds.fill(0, 255, 0)
                robot.leds.show()

        speed_est.update()

        loop_count += 1
        if loop_count % 10 == 0:
            us_str = f"{last_us:.0f}cm" if last_us > 0 else "N/A"
            stat_str = "STILL" if motion.get("is_stationary", False) else "MOVE"
            speed_cm_s = speed_est.get_speed()
            print(
                f"[{stat_str}] Head:{motion['heading_deg']:+6.1f}° "
                f"Pos:({motion['position_x']:+.2f},{motion['position_y']:+.2f}) "
                f"US:{us_str} Speed:{speed_cm_s:5.1f}cm/s"
            )
            if oled.available:
                mode_str = "RC ON" if controller.is_active() else "RC OFF"
                us_line = f"US {last_us:.0f}cm" if last_us > 0 else "US ---"
                oled.show_two_lines(mode_str, us_line)

        # ------------------------------------------------------------------
        # Add autonomous behavior here when remote control is not active.
        # Example (commented):
        #
        # if not controller.is_active():
        #     distance = batch[-1].get("ultrasonic_cm", -1)
        #     if distance > 0 and distance < 20:
        #         robot.motors.backward(100)
        #         time.sleep(0.3)
        #         robot.motors.turn_right(100)
        #         time.sleep(0.3)
        #     elif distance > 0:
        #         robot.motors.forward(100)
        #     else:
        #         robot.motors.stop()
        # ------------------------------------------------------------------


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as exc:
        print(f"\nError: {exc}")
        import sys
        sys.print_exception(exc)
    finally:
        from picobot import Motors
        Motors().stop()
