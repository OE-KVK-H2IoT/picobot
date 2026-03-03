"""



TODO

modify the repl so at boot wait like 2 sec if something want to connect
or some UDP message for host app oto enable repl when its opened/enabled?


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

"""
==============================================================================
LAB 7: Full System - WiFi, Streaming, Ultrasonic & Remote Control
==============================================================================

RUN MODE:
- Robot runs
- IMU + ultrasonic streamed
- Motors controlled by UDP

DEV MODE:
- mpremote connects over WiFi
- Motors are stopped
- REPL + file upload available
- On disconnect, robot reboots into RUN MODE

This is how real robots are programmed.
==============================================================================
"""

import time
import socket
import struct
import os
import machine
import network
from machine import Pin, I2C

from picobot import PicoBot
from robot_state import RobotState, IMUStreamer


# ==============================================================================
# CONFIGURATION
# ==============================================================================

REPL_PORT = 8266

HOST_IP = "192.168.28.102"
HOST_PORT = 5005
CONTROL_PORT = 5006

I2C_SCL_PIN = 15
I2C_SDA_PIN = 14

SAMPLE_RATE_HZ = 500
BATCH_SIZE = 20
SAMPLE_DELAY_MS = 1000 // SAMPLE_RATE_HZ

ENABLE_FILTERS = True
FILTER_ALPHA = 0.3


# ==============================================================================
# DEV MODE (WiFi REPL)
# ==============================================================================

def wait_for_repl():
    """
    Blocks until mpremote connects.
    When connected:
      - Motors stop
      - REPL moves to WiFi
      - When mpremote disconnects → reboot → RUN mode
    """
    print("DEV MODE: waiting for WiFi REPL on port", REPL_PORT)
    import machine
    machine.disable_irq() 

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", REPL_PORT))
    s.listen(1)

    conn, addr = s.accept()
    print("DEV MODE: client", addr)

    # Safety: stop motors
    try:
        from picobot import Motors
        Motors().stop()
    except:
        pass
    import usb_cdc
    usb_cdc.disable() 
    # Redirect REPL to WiFi
    os.dupterm(None) 
    os.dupterm(conn)
    import sys
    sys.stdin.write("\x01")
    sys.stdin.flush()

    # Block until mpremote disconnects
    try:
        while True:
            data = conn.recv(1)
            if not data:
                break
    except:
        pass

    print("DEV MODE ended, rebooting...")
    usb_cdc.enable()

    os.dupterm(None)
    try:
        conn.close()
    except:
        pass
    try:
        s.close()
    except:
        pass

    machine.reset()


# ==============================================================================
# CONTROL RECEIVER
# ==============================================================================

class ControlReceiver:
    def __init__(self, port=5006):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.left = 0
        self.right = 0
        self.last = time.ticks_ms()

    def check(self):
        try:
            data, _ = self.sock.recvfrom(64)
            if len(data) >= 5 and data[0:1] == b"M":
                self.left, self.right = struct.unpack("<hh", data[1:5])
                self.last = time.ticks_ms()
        except:
            pass

    def get(self):
        if time.ticks_diff(time.ticks_ms(), self.last) > 500:
            return 0, 0
        return max(-255, min(255, self.left)), max(-255, min(255, self.right))

    def active(self):
        return time.ticks_diff(time.ticks_ms(), self.last) < 500


# ==============================================================================
# RUN MODE
# ==============================================================================

def run_robot():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        print("WiFi not connected – check boot.py")
        return
    print("WiFi OK:", wlan.ifconfig())

    print("\n[RUN] Initializing robot")
    robot = PicoBot(num_leds=4)
    robot.leds.fill(0, 255, 0)
    robot.leds.show()

    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    if 0x68 not in i2c.scan():
        print("IMU not found")
        return

    chip = i2c.readfrom_mem(0x68, 0x00, 1)[0]
    if chip == 0xD1:
        from bmi160 import BMI160
        sensor = BMI160(i2c, dlpf=2)
    else:
        from mpu6050 import MPU6050
        sensor = MPU6050(i2c, dlpf=2)

    state = RobotState(enable_filters=ENABLE_FILTERS, filter_alpha=FILTER_ALPHA)

    print("[RUN] Calibrating IMU")
    time.sleep(1)
    state.calibrate(sensor, 100, 10)

    controller = ControlReceiver(CONTROL_PORT)
    streamer = IMUStreamer(sensor, state, robot.ultrasonic, HOST_IP, HOST_PORT)

    print("[RUN] Robot active")

    while True:
        batch = streamer.read_batch(BATCH_SIZE, SAMPLE_DELAY_MS)
        streamer.send_batch(batch)

        controller.check()
        if controller.active():
            l, r = controller.get()
            robot.motors.set_speeds(l, r)
        else:
            robot.motors.stop()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    # First allow programming
    wait_for_repl()

    # After mpremote disconnects, we reboot and start RUN mode
    # So run_robot() is reached only after reset

if __name__ == "__main__":
    main()
