# radar_sweep.py — Pico 2 W: VL53L1X ToF radar sweep over UDP
# Sweeps a servo-mounted VL53L1X sensor and sends angle+distance to a PC.
#
# Copy vl53l1x.py to lib/ on the Pico 2 W filesystem.
# Run with: mpremote connect /dev/ttyACM0 run radar_sweep.py

import time
import network
import socket
from machine import Pin, I2C, PWM
from vl53l1x import VL53L1X

# ===========================================================
# USER SETTINGS — edit these to match your setup
# ===========================================================
WIFI_SSID = "YOUR_SSID"
WIFI_PASS = "YOUR_PASS"

PC_IP = "192.168.1.100"      # your PC's LAN IP
PC_PORT = 5005

# I2C pins for VL53L1X (bus 1: GP14=SDA, GP15=SCL)
I2C_ID = 1
I2C_SDA = 14                 # GP14
I2C_SCL = 15                 # GP15
I2C_FREQ = 400_000

# Servo — direct PWM (50Hz, standard hobby servo signal)
SERVO_PIN = 18                # GP18
SERVO_FREQ = 50               # 50Hz standard (20ms period)
SERVO_MIN_US = 500             # 0 degrees pulse width
SERVO_MAX_US = 2500            # 180 degrees pulse width
SERVO_REVERSE = False          # Set True if servo direction is inverted

# Sweep parameters
ANGLE_MIN = 10               # degrees (avoid servo mechanical limits)
ANGLE_MAX = 170
ANGLE_STEP = 2               # degrees per step
SETTLE_MS = 25               # servo settle time between steps
TOF_TIMEOUT_MS = 100         # max time to wait for a ToF reading

# ToF sensor config
DISTANCE_MODE = 2            # 1=SHORT (up to ~1.3m), 2=LONG (up to ~4m)
TIMING_BUDGET_MS = 33        # 15/20/33/50/100/200/500 — lower = faster but noisier

# Status LED (onboard LED on Pico W)
LED_PIN = "LED"

# ===========================================================
# Servo helper
# ===========================================================
def servo_init(pin, freq=50):
    pwm = PWM(Pin(pin))
    pwm.freq(freq)
    return pwm

def servo_angle(pwm, angle):
    """Set servo to angle (0-180) using standard 50Hz PWM."""
    angle = max(0, min(180, angle))
    if SERVO_REVERSE:
        angle = 180 - angle
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180
    period_us = 1_000_000 / SERVO_FREQ
    duty = int(pulse_us / period_us * 65535)
    pwm.duty_u16(duty)

# ===========================================================


def wifi_connect(ssid, password, timeout_s=15):
    """Connect to WiFi, return WLAN object."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        print("WiFi already connected:", wlan.ifconfig())
        return wlan

    print("Connecting to WiFi '{}'...".format(ssid))
    wlan.connect(ssid, password)

    deadline = time.ticks_add(time.ticks_ms(), timeout_s * 1000)
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), deadline) > 0:
            raise RuntimeError("WiFi connection timeout after {}s".format(timeout_s))
        time.sleep_ms(250)

    print("WiFi connected:", wlan.ifconfig())
    return wlan


def main():
    # Status LED
    led = Pin(LED_PIN, Pin.OUT)
    led.on()

    # WiFi
    wifi_connect(WIFI_SSID, WIFI_PASS)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # I2C + ToF sensor
    i2c = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_FREQ)
    devices = i2c.scan()
    print("I2C devices found:", [hex(d) for d in devices])

    tof = VL53L1X(i2c)
    tof.init()
    tof.set_distance_mode(DISTANCE_MODE)
    tof.set_timing_budget_ms(TIMING_BUDGET_MS)
    print("VL53L1X initialized (mode={}, budget={}ms)".format(DISTANCE_MODE, TIMING_BUDGET_MS))

    # Servo — direct 50Hz PWM
    servo_pwm = servo_init(SERVO_PIN, SERVO_FREQ)
    servo_angle(servo_pwm, (ANGLE_MIN + ANGLE_MAX) // 2)
    time.sleep_ms(500)  # let servo reach center

    angle = ANGLE_MIN
    direction = 1  # +1 = sweeping up, -1 = sweeping down
    sweep_id = 0

    print("Starting radar sweep: {} -> {} deg, step {}, servo GP{}".format(
        ANGLE_MIN, ANGLE_MAX, ANGLE_STEP, SERVO_PIN))

    while True:
        servo_angle(servo_pwm, angle)
        time.sleep_ms(SETTLE_MS)

        dist_mm = tof.read_mm(timeout_ms=TOF_TIMEOUT_MS)

        # Toggle LED each reading for visual heartbeat
        led.toggle()

        # Packet: "angle,distance_mm,sweep_id\n"
        msg = "{},{},{}\n".format(angle, dist_mm, sweep_id)
        try:
            sock.sendto(msg.encode(), (PC_IP, PC_PORT))
        except OSError as e:
            print("UDP send error:", e)

        # Advance sweep
        angle += direction * ANGLE_STEP
        if angle >= ANGLE_MAX:
            angle = ANGLE_MAX
            direction = -1
            sweep_id += 1
        elif angle <= ANGLE_MIN:
            angle = ANGLE_MIN
            direction = +1
            sweep_id += 1


try:
    main()
except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    import sys
    sys.print_exception(e)
    print("Fatal error. Resetting in 5s...")
    time.sleep(5)
    import machine
    machine.reset()
