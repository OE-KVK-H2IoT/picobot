# main_spad.py — Pico 2 W: VL53L1X 4x4 multizone SPAD scanning + servo
# Scans 16 zones (4x4 ROI grid) across the 16x16 SPAD array.
# Sends frame data over UDP; accepts servo commands from PC.
#
# Copy vl53l1x.py to the Pico 2 W filesystem.
# Run with: mpremote connect /dev/ttyACM0 run pico/main_spad.py

import time
import network
import socket
from machine import Pin, I2C, PWM
from vl53l1x import VL53L1X

# ===========================================================
# USER SETTINGS
# ===========================================================
WIFI_SSID = "Flying-013"
WIFI_PASS = "userC013"

PC_IP = "192.168.6.71"
DATA_PORT = 5006              # Pico sends zone frames here
CMD_PORT = 5007               # Pico listens for servo commands here

# I2C (bus 1: GP14=SDA, GP15=SCL)
I2C_ID = 1
I2C_SDA = 14
I2C_SCL = 15
I2C_FREQ = 400_000

# Servo — direct PWM (50Hz, standard hobby servo signal)
SERVO_PIN = 18                # GP18 = pico_car S1 header
SERVO_FREQ = 50               # 50Hz standard (20ms period)
# Pulse width range in microseconds (adjust if your servo differs)
SERVO_MIN_US = 500             # 0 degrees
SERVO_MAX_US = 2500            # 180 degrees
SERVO_REVERSE = True           # Verified with cal_servo.py

# ToF config
DISTANCE_MODE = 2             # 1=SHORT, 2=LONG
TIMING_BUDGET_MS = 20         # 20ms for faster scanning

# Sweep defaults
SWEEP_MIN = 0
SWEEP_MAX = 180
SWEEP_STEP = 2

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
# Zone grids — precomputed SPAD center numbers
# Sensor is mounted UPSIDE DOWN (~15cm from ground).
#   SPAD row 0 = physically bottom (ground side)
#   SPAD row 15 = physically top
#   SPAD col 0 = physically right
#   SPAD col 15 = physically left
# Zone rows: high SPAD rows first (physically top on display).
# Zone cols: high SPAD cols first (physically left on display).
# Bottom zone row avoids lowest SPAD rows to skip floor.
# ===========================================================
def _spad(row, col):
    if row < 8:
        return 128 + col * 8 + row
    return 127 - col * 8 - (row - 8)

# Normal: 16 zones (4x4 grid, 4x4 ROI each) — ~3 FPS, full detail
# Rows inverted + ground avoidance: display top→bottom = SPAD [13,11,8,5]
_ROWS16 = [13, 11, 8, 5]
_COLS16 = [13, 10, 6, 2]
ZONE_SPADS_16 = [_spad(_ROWS16[zr], _COLS16[zc])
                 for zr in range(4) for zc in range(4)]

# Turbo: 4 zones (2x2 grid, 4x4 ROI each) — ~12 FPS
# Upper two rows only (inverted sensor: high rows = top)
_ROWS4 = [11, 8]
_COLS4 = [10, 5]
ZONE_SPADS_4 = [_spad(_ROWS4[zr], _COLS4[zc])
                for zr in range(2) for zc in range(2)]
# Map 4 zones → 16 cells
_MAP_4_TO_16 = [
    [0, 1, 4, 5],      # top-left
    [2, 3, 6, 7],      # top-right
    [8, 9, 12, 13],    # bottom-left
    [10, 11, 14, 15],  # bottom-right
]

# Fast: 1 zone (4x4 ROI, upper-center) — ~50 FPS
SPAD_SINGLE = _spad(10, 8)


# ===========================================================


def wifi_connect(ssid, password, timeout_s=15):
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
            raise RuntimeError("WiFi timeout")
        time.sleep_ms(250)
    print("WiFi connected:", wlan.ifconfig())
    return wlan


def main():
    led = Pin(LED_PIN, Pin.OUT)
    led.on()

    wifi_connect(WIFI_SSID, WIFI_PASS)

    # Data socket (send frames to PC)
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Command socket (receive from PC, non-blocking)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(("0.0.0.0", CMD_PORT))
    cmd_sock.setblocking(False)

    # I2C + ToF
    i2c = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_FREQ)
    print("I2C devices:", [hex(d) for d in i2c.scan()])

    tof = VL53L1X(i2c)
    tof.init()
    tof.set_distance_mode(DISTANCE_MODE)
    tof.set_timing_budget_ms(TIMING_BUDGET_MS)

    # Scan mode: N=16 zones, T=4 zones, F=1 zone
    scan_mode = "T"
    tof.set_roi(4, 4)
    sweep_mode = "T"
    print("VL53L1X ready (mode={}, budget={}ms)".format(DISTANCE_MODE, TIMING_BUDGET_MS))

    # Quick driver test
    dist = tof.read_mm(timeout_ms=500)
    print("Sensor test: {} mm".format(dist))

    # Servo — direct 50Hz PWM, start at sweep min (sweep on by default)
    servo_pwm = servo_init(SERVO_PIN, SERVO_FREQ)
    cur_angle = SWEEP_MIN
    servo_angle(servo_pwm, cur_angle)
    time.sleep_ms(300)
    print("Servo on GP{} at 90 deg (center)".format(SERVO_PIN))

    # State — sweep enabled by default
    sweeping = True
    sweep_once = False
    sweep_dir = 1
    sweep_step = SWEEP_STEP * 3
    distances = [0] * 16

    print("SPAD scanner ready. Listening for commands on port {}.".format(CMD_PORT))

    frame_count = 0
    t_start = time.ticks_ms()

    while True:
        # --- Scan zones ---
        try:
            if scan_mode == "N":
                # Normal: 16 zones
                for i, spad in enumerate(ZONE_SPADS_16):
                    tof.set_roi_center(spad)
                    distances[i] = tof.read_mm(timeout_ms=TIMING_BUDGET_MS + 10)
            elif scan_mode == "T":
                # Turbo: 4 zones → replicate to 16
                for i, spad in enumerate(ZONE_SPADS_4):
                    tof.set_roi_center(spad)
                    d = tof.read_mm(timeout_ms=TIMING_BUDGET_MS + 10)
                    for cell in _MAP_4_TO_16[i]:
                        distances[cell] = d
            else:
                # Fast: 1 zone → fill all 16
                tof.set_roi_center(SPAD_SINGLE)
                d = tof.read_mm(timeout_ms=TIMING_BUDGET_MS + 10)
                for i in range(16):
                    distances[i] = d
        except OSError as e:
            print("I2C error:", e)
            time.sleep_ms(50)
            continue

        led.toggle()
        frame_count += 1

        # --- Send frame (include sweep direction: 1=fwd, -1=rev, 0=static) ---
        sdir = sweep_dir if sweeping else 0
        parts = ["F", str(cur_angle), str(sdir)]
        parts.extend(str(d) for d in distances)
        msg = ",".join(parts) + "\n"
        try:
            data_sock.sendto(msg.encode(), (PC_IP, DATA_PORT))
        except OSError as e:
            print("UDP send:", e)

        # --- FPS + debug (every 20 frames) ---
        if frame_count % 20 == 0:
            elapsed = time.ticks_diff(time.ticks_ms(), t_start)
            fps = 20000 / elapsed if elapsed > 0 else 0
            valid = sum(1 for d in distances if d > 0)
            print("[step{}] FPS:{:.1f} angle:{} valid:{}/16".format(
                sweep_step, fps, cur_angle, valid))
            t_start = time.ticks_ms()

        # --- Check for commands from PC ---
        try:
            data, addr = cmd_sock.recvfrom(64)
            cmd = data.decode("ascii").strip()
            if cmd.startswith("A"):
                cur_angle = int(cmd[1:])
                cur_angle = max(0, min(180, cur_angle))
                servo_angle(servo_pwm, cur_angle)
                sweeping = False
                print("Servo -> {}".format(cur_angle))
            elif cmd == "W":
                sweeping = True
                sweep_once = False
                sweep_dir = 1
                cur_angle = SWEEP_MIN
                servo_angle(servo_pwm, cur_angle)
                print("Sweep started")
            elif cmd == "X":
                sweeping = False
                sweep_once = False
                cur_angle = 90
                servo_angle(servo_pwm, cur_angle)
                print("Sweep stopped, servo -> 90")
            elif cmd == "R":
                sweeping = False
                sweep_once = False
                cur_angle = 90
                servo_angle(servo_pwm, cur_angle)
                print("Reset servo -> 90")
            elif cmd in ("MN", "MT", "MF"):
                scan_mode = cmd[1]
                sweep_step = {"N": SWEEP_STEP, "T": SWEEP_STEP * 3, "F": SWEEP_STEP * 3}[scan_mode]
                tof.set_roi(4, 4)
                names = {"N": "Normal 16z", "T": "Turbo 4z", "F": "Fast 1z"}
                print("{} step{}".format(names[scan_mode], sweep_step))
            elif cmd == "T":
                sweeping = True
                sweep_dir = 1
                cur_angle = SWEEP_MIN
                servo_angle(servo_pwm, cur_angle)
                sweep_once = True
                print("Quick sweep started")
            elif cmd == "1":
                sweeping = False
                sweep_once = False
                cur_angle = 0
                servo_angle(servo_pwm, cur_angle)
                print("Preset LEFT -> 0")
            elif cmd == "2":
                sweeping = False
                sweep_once = False
                cur_angle = 90
                servo_angle(servo_pwm, cur_angle)
                print("Preset CENTER -> 90")
            elif cmd == "3":
                sweeping = False
                sweep_once = False
                cur_angle = 180
                servo_angle(servo_pwm, cur_angle)
                print("Preset RIGHT -> 180")
        except OSError:
            pass

        # --- Advance sweep ---
        if sweeping:
            cur_angle += sweep_dir * sweep_step
            if cur_angle >= SWEEP_MAX:
                cur_angle = SWEEP_MAX
                sweep_dir = -1
                if sweep_once:
                    # Quick sweep: reached end, return to center
                    sweeping = False
                    sweep_once = False
                    cur_angle = 90
                    print("Quick sweep done")
            elif cur_angle <= SWEEP_MIN:
                cur_angle = SWEEP_MIN
                sweep_dir = 1
            servo_angle(servo_pwm, cur_angle)
            # Let servo settle — ~3ms per degree of movement
            time.sleep_ms(sweep_step * 3)


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
