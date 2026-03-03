# cal_servo.py — Lightweight servo calibration listener
# No ToF sensor needed. Just WiFi + servo + UDP.
# Run with: mpremote connect /dev/ttyACM0 run pico/cal_servo.py
# Then run on PC: python pc/servo_cal.py

import time
import network
import socket
from machine import Pin, PWM

# --- Settings (match your main_spad.py) ---
WIFI_SSID = "Flying-013"
WIFI_PASS = "userC013"
CMD_PORT = 5007

SERVO_PIN = 18
SERVO_FREQ = 50
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

def set_servo(pwm, angle, reverse=False):
    angle = max(0, min(180, angle))
    if reverse:
        angle = 180 - angle
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180
    period_us = 1_000_000 / SERVO_FREQ
    duty = int(pulse_us / period_us * 65535)
    pwm.duty_u16(duty)
    return pulse_us

# WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print("Connecting to WiFi '{}'...".format(WIFI_SSID))
    wlan.connect(WIFI_SSID, WIFI_PASS)
    deadline = time.ticks_add(time.ticks_ms(), 15000)
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), deadline) > 0:
            raise RuntimeError("WiFi timeout")
        time.sleep_ms(250)
print("WiFi:", wlan.ifconfig())

# Servo
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(SERVO_FREQ)
reverse = False
angle = 90
pulse = set_servo(pwm, angle, reverse)
print("Servo on GP{} at {}Hz, angle={}, pulse={:.0f}us".format(
    SERVO_PIN, SERVO_FREQ, angle, pulse))

# UDP listener
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", CMD_PORT))
sock.setblocking(False)

print("Listening on UDP port {}...".format(CMD_PORT))
print("Run: python pc/servo_cal.py")

while True:
    try:
        data, addr = sock.recvfrom(64)
        cmd = data.decode("ascii").strip()

        if cmd.startswith("A"):
            angle = int(cmd[1:])
            angle = max(0, min(180, angle))
            pulse = set_servo(pwm, angle, reverse)
            # Send back confirmation
            resp = "OK,{},{:.0f},{}\n".format(angle, pulse, int(reverse))
            sock.sendto(resp.encode(), addr)
            print("angle={:3d}  pulse={:.0f}us  rev={}".format(angle, pulse, reverse))

        elif cmd == "REV":
            reverse = not reverse
            pulse = set_servo(pwm, angle, reverse)
            resp = "OK,{},{:.0f},{}\n".format(angle, pulse, int(reverse))
            sock.sendto(resp.encode(), addr)
            print("REVERSE={} angle={} pulse={:.0f}us".format(reverse, angle, pulse))

        elif cmd.startswith("MINPW"):
            SERVO_MIN_US = int(cmd[5:])
            pulse = set_servo(pwm, angle, reverse)
            print("MIN_US={}".format(SERVO_MIN_US))

        elif cmd.startswith("MAXPW"):
            SERVO_MAX_US = int(cmd[5:])
            pulse = set_servo(pwm, angle, reverse)
            print("MAX_US={}".format(SERVO_MAX_US))

    except OSError:
        pass
    time.sleep_ms(5)
