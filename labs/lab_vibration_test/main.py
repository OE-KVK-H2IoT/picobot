"""
==============================================================================
PICOBOT SPEED CALIBRATION LAB
==============================================================================

Automatic calibration process using ultrasonic as ground truth.

CALIBRATION PROCESS:
1. Place robot 50cm from wall
2. Robot drives forward 30cm (using ultrasonic)
3. Measures bumps and vibration during movement
4. Calculates bumps/cm and vib-to-speed ratios
5. Verification: drives 30cm using calibrated values
6. Compares estimated vs actual distance, shows error
7. Repeats to refine calibration

==============================================================================
"""

import time
import math
import network
import socket
import struct
from machine import Pin, I2C

from picobot import PicoBot, UltrasonicIRQ
from oled import OLEDStatus

# IMU type: 'auto', 'mpu6050', or 'bmi160'
IMU_TYPE = 'auto'

# ==============================================================================
# CONFIGURATION
# ==============================================================================

WIFI_SSID = 'NeoNexus'
WIFI_PASSWORD = 'userC013$'

HOST_IP = '192.168.28.104'
HOST_PORT = 5005
CONTROL_PORT = 5006

I2C_SCL_PIN = 15
I2C_SDA_PIN = 14

# Calibration settings
CALIBRATION_DISTANCE = 20.0  # cm - target distance for calibration runs
MOTOR_SPEED = 70             # PWM speed (slow for safety)
MIN_WALL_DISTANCE = 8.0      # cm - don't get closer than this
MAX_WALL_DISTANCE = 80.0     # cm - don't start further than this

SAMPLE_RATE_HZ = 500
SAMPLE_DELAY_MS = 1000 // SAMPLE_RATE_HZ


# ==============================================================================
# WIFI
# ==============================================================================

def connect_wifi(ssid, password, timeout_s=15):
    print(f"Connecting to WiFi: {ssid}")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    for i in range(timeout_s):
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print(f"Connected! IP: {ip}")
            return wlan
        print(f"  Waiting... ({i+1}/{timeout_s})")
        time.sleep(1)

    print("WiFi failed!")
    return None


# ==============================================================================
# CONTROL RECEIVER
# ==============================================================================

class UDPLogger:
    """Send log messages to host via UDP."""

    def __init__(self, host_ip, port=5008):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host_ip, port)

    def log(self, msg):
        """Send log message to host."""
        try:
            self.sock.sendto(msg.encode() if isinstance(msg, str) else msg, self.addr)
        except:
            pass

    def __call__(self, msg):
        """Allow using logger as function: log("message")"""
        self.log(msg)


class ControlReceiver:
    def __init__(self, port=5006):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.left_speed = 0
        self.right_speed = 0

    def check_commands(self):
        try:
            data, addr = self.sock.recvfrom(64)
            if len(data) >= 1:
                if data[0:1] == b'Q':
                    return 'QUIT'
                elif data[0:1] == b'C':  # Start calibration
                    return 'CALIBRATE'
                elif data[0:1] == b'V':  # Verify calibration
                    return 'VERIFY'
                elif data[0:1] == b'M' and len(data) >= 5:
                    left, right = struct.unpack("<hh", data[1:5])
                    self.left_speed = max(-255, min(255, left))
                    self.right_speed = max(-255, min(255, right))
                    return ('MOTOR', self.left_speed, self.right_speed)
        except:
            pass
        return None


# ==============================================================================
# BUMP COUNTER
# ==============================================================================

class BumpCounter:
    def __init__(self, threshold=0.015):
        self.threshold = threshold
        self.bump_count = 0
        self.last_above = False

        # High-pass filter
        self.hp_alpha = 0.95
        self.hp_prev_in = 0.0
        self.hp_prev_out = 0.0

        # Calibration
        self.bumps_per_cm = None
        self.run_bumps = 0

    def reset_run(self):
        """Reset for new run."""
        self.run_bumps = 0

    def add_sample(self, accel_mag):
        # High-pass filter
        hp_out = self.hp_alpha * (self.hp_prev_out + accel_mag - self.hp_prev_in)
        self.hp_prev_in = accel_mag
        self.hp_prev_out = hp_out

        is_above = hp_out > self.threshold
        if is_above and not self.last_above:
            self.bump_count += 1
            self.run_bumps += 1
        self.last_above = is_above

    def calibrate(self, distance_cm):
        """Set calibration from measured run."""
        if distance_cm > 1 and self.run_bumps > 5:
            self.bumps_per_cm = self.run_bumps / distance_cm
            return self.bumps_per_cm
        return None

    def estimate_distance(self):
        """Estimate distance from bumps using calibration."""
        if self.bumps_per_cm and self.bumps_per_cm > 0:
            return self.run_bumps / self.bumps_per_cm
        return None


# ==============================================================================
# VIBRATION ANALYZER
# ==============================================================================

class VibrationAnalyzer:
    def __init__(self, window_size=50):
        self.window_size = window_size
        self.samples = []
        self.vib_to_speed = None  # cm/s per unit vibration
        self.baseline = 2.0
        self.run_vib_sum = 0
        self.run_vib_count = 0

    def reset_run(self):
        self.run_vib_sum = 0
        self.run_vib_count = 0

    def add_sample(self, ax, ay, az):
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        self.samples.append(mag)
        if len(self.samples) > self.window_size:
            self.samples.pop(0)

    def get_variance(self):
        if len(self.samples) < 10:
            return 0.0
        mean = sum(self.samples) / len(self.samples)
        return sum((x - mean)**2 for x in self.samples) / len(self.samples) * 1000

    def accumulate(self):
        """Accumulate vibration for averaging."""
        vib = self.get_variance()
        self.run_vib_sum += vib
        self.run_vib_count += 1
        return vib

    def get_run_average(self):
        if self.run_vib_count > 0:
            return self.run_vib_sum / self.run_vib_count
        return 0

    def calibrate(self, distance_cm, time_s):
        """Calibrate vib-to-speed from measured run."""
        avg_vib = self.get_run_average()
        if time_s > 0.1 and avg_vib > self.baseline:
            actual_speed = distance_cm / time_s
            self.vib_to_speed = actual_speed / (avg_vib - self.baseline)
            return self.vib_to_speed
        return None

    def estimate_speed(self):
        """Estimate speed from current vibration."""
        if self.vib_to_speed is None:
            return None
        vib = self.get_variance()
        if vib > self.baseline:
            return (vib - self.baseline) * self.vib_to_speed
        return 0.0


# ==============================================================================
# DRIFT COMPENSATOR
# ==============================================================================

class DriftCompensator:
    """
    Keeps robot driving straight using gyroscope feedback.

    Integrates gyro to track absolute heading deviation from start.
    Uses PID to correct both heading drift and rotation rate.
    """

    def __init__(self):
        self.correction = 0.0
        self.heading = 0.0       # Integrated heading deviation (degrees)
        self.kp_heading = 8.0    # Correct heading error
        self.kp_rate = 2.0       # Dampen rotation rate
        self.dt = 0.002          # 2ms sample time
        self.max_correction = 60

    def update(self, gyro_z):
        """
        Update correction based on gyro rotation.

        Args:
            gyro_z: Rotation rate in deg/s (positive = turning left)

        Returns:
            (left_adj, right_adj): Motor speed adjustments
        """
        # Integrate heading
        self.heading += gyro_z * self.dt

        # Correction: fix heading error + dampen rotation rate
        # Heading error: if heading > 0, we've turned left, need to turn right
        # Rate damping: if gyro_z > 0, we're turning left, resist it
        self.correction = (self.kp_heading * self.heading +
                          self.kp_rate * gyro_z)
        self.correction = max(-self.max_correction, min(self.max_correction, self.correction))

        # If correction > 0 (drifted left), speed up left / slow right
        left_adj = int(self.correction / 2)
        right_adj = int(-self.correction / 2)

        return left_adj, right_adj

    def reset(self):
        self.correction = 0
        self.heading = 0.0


# ==============================================================================
# CALIBRATION RUN
# ==============================================================================

class CalibrationRun:
    """Manages a single calibration or verification run."""

    def __init__(self, robot, imu, ultrasonic, bump_counter, vib_analyzer, drift_comp):
        self.robot = robot
        self.imu = imu
        self.ultrasonic = ultrasonic
        self.bump = bump_counter
        self.vib = vib_analyzer
        self.drift = drift_comp

        # Run state
        self.running = False
        self.direction = 1  # 1=forward, -1=backward
        self.start_distance = 0
        self.target_distance = 0
        self.start_time = 0
        self.last_valid_distance = 0  # Track last good US reading

        # Results
        self.actual_distance = 0
        self.elapsed_time = 0
        self.avg_vibration = 0
        self.bump_count = 0
        self.estimated_distance = None
        self.estimated_speed = None

        # Motor tracking (for streaming)
        self.motor_left = 0
        self.motor_right = 0

    def start(self, direction, target_cm):
        """Start a calibration run."""
        self.direction = direction
        self.target_distance = target_cm

        # Get start distance (auto-triggered in background)
        time.sleep_ms(100)  # Wait for a couple measurements
        dist = self.ultrasonic.distance_cm_valid()
        if dist > 0:
            self.start_distance = dist
            self.last_valid_distance = dist
        else:
            self.start_distance = 50  # Default if no reading
            self.last_valid_distance = 50

        self.start_time = time.ticks_ms()

        self.bump.reset_run()
        self.vib.reset_run()
        self.drift.reset()

        self.running = True
        self.estimated_distance = 0

        dir_str = "FORWARD" if direction > 0 else "BACKWARD"
        print(f"\n>>> Starting {dir_str} run: {target_cm}cm at speed {MOTOR_SPEED} (start: {self.start_distance:.1f}cm)")

    def update(self, ax, ay, az, gz):
        """Update during run. Returns True when complete."""
        if not self.running:
            return False

        # Add samples
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az) - 1.0
        self.bump.add_sample(abs(accel_mag))
        self.vib.add_sample(ax, ay, az)
        self.vib.accumulate()

        # Drift compensation
        left_adj, right_adj = self.drift.update(gz)

        # Motor control
        base_speed = MOTOR_SPEED * self.direction
        left = max(-255, min(255, base_speed + left_adj))
        right = max(-255, min(255, base_speed + right_adj))
        self.motor_left = int(left)
        self.motor_right = int(right)
        self.robot.motors.set_speeds(left, right)

        # Check distance (auto-triggered in background, just read latest)
        current_dist = self.ultrasonic.distance_cm_valid()
        if current_dist > 0:
            self.last_valid_distance = current_dist
        else:
            current_dist = self.last_valid_distance  # Use last valid

        traveled = abs(current_dist - self.start_distance)

        # Estimate distance from bumps (for verification)
        if self.bump.bumps_per_cm:
            self.estimated_distance = self.bump.estimate_distance()

        # Check if target reached or safety limits
        if traveled >= self.target_distance:
            self._finish(current_dist)
            return True

        if self.direction > 0 and current_dist < MIN_WALL_DISTANCE:
            print(f"\n>>> Safety stop: too close ({current_dist}cm)")
            self._finish(current_dist)
            return True

        if self.direction < 0 and current_dist > MAX_WALL_DISTANCE:
            print(f"\n>>> Safety stop: too far ({current_dist}cm)")
            self._finish(current_dist)
            return True

        return False

    def _finish(self, end_distance):
        """Finish the run and calculate results."""
        self.robot.motors.stop()
        self.running = False

        self.actual_distance = abs(end_distance - self.start_distance)
        self.elapsed_time = time.ticks_diff(time.ticks_ms(), self.start_time) / 1000.0
        self.avg_vibration = self.vib.get_run_average()
        self.bump_count = self.bump.run_bumps

        if self.bump.bumps_per_cm:
            self.estimated_distance = self.bump.estimate_distance()

    def get_status(self):
        """Get current status string."""
        current = self.last_valid_distance
        traveled = abs(current - self.start_distance) if self.start_distance > 0 else 0
        vib = self.vib.get_variance()

        est_str = ""
        if self.estimated_distance is not None:
            est_str = f" Est:{self.estimated_distance:5.1f}cm"

        return (f"[RUN ] US:{current:5.1f}cm Trav:{traveled:5.1f}cm "
                f"V:{vib:5.1f} B:{self.bump.run_bumps:4d}{est_str}")


# ==============================================================================
# MAIN CALIBRATION SYSTEM
# ==============================================================================

def main():
    print("\n" + "="*50)
    print("     PICOBOT SPEED CALIBRATION LAB")
    print("="*50)

    # Initialize hardware
    print("\n[1/5] Initializing hardware...")
    robot = PicoBot()
    robot.leds.fill(255, 255, 0)
    robot.leds.show()

    # Auto-triggered ultrasonic with hard IRQ (immune to WiFi/I2C interrupts)
    ultrasonic = UltrasonicIRQ(interval_ms=60)  # Auto-triggers every 60ms
    time.sleep_ms(100)  # Wait for first measurement
    test_dist = ultrasonic.distance_cm_valid()
    print(f"  Ultrasonic: {test_dist:.1f}cm (auto-trigger + hard IRQ)")

    # Initialize I2C
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    devices = i2c.scan()
    print(f"  I2C devices: {[hex(d) for d in devices]}")

    # Initialize OLED (128x32)
    oled = OLEDStatus(i2c)
    if oled.available:
        print("  OLED: OK (128x32)")
        oled.show_message("PICOBOT", "Calibration")
    else:
        print("  OLED: Not found")

    # Initialize IMU
    if 0x68 not in devices:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        print("ERROR: IMU not found!")
        return

    # Auto-detect IMU
    if IMU_TYPE == 'auto':
        chip_id = i2c.readfrom_mem(0x68, 0x00, 1)[0]
        detected = 'bmi160' if chip_id == 0xD1 else 'mpu6050'
    else:
        detected = IMU_TYPE

    if detected == 'bmi160':
        from bmi160 import BMI160
        imu = BMI160(i2c, dlpf=1)
        print(f"  IMU: BMI160")
    else:
        from mpu6050 import MPU6050
        imu = MPU6050(i2c, dlpf=1)
        print(f"  IMU: MPU6050")

    # Connect WiFi
    print("\n[2/5] Connecting to WiFi...")
    robot.leds.fill(0, 255, 255)
    robot.leds.show()
    oled.show_message("WiFi...", WIFI_SSID[:16])

    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    if not wlan:
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        oled.show_message("WiFi", "FAILED!")
        return

    oled.show_message("Connected!", wlan.ifconfig()[0])

    # Setup networking
    print("\n[3/5] Starting network...")
    controller = ControlReceiver(CONTROL_PORT)
    log = UDPLogger(HOST_IP, 5008)  # UDP logger for wireless debugging

    # UDP socket for streaming to host
    stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream_addr = (HOST_IP, HOST_PORT)
    print(f"  Streaming to {HOST_IP}:{HOST_PORT}")
    log(f"Robot started, streaming to {HOST_IP}")

    # Calibrate IMU
    print("\n[4/5] Calibrating IMU... keep still!")
    robot.leds.fill(255, 165, 0)
    robot.leds.show()
    time.sleep(0.5)

    bias_ax, bias_ay, bias_az, bias_gz = 0, 0, 0, 0
    for _ in range(100):
        a = imu.get_accel()
        g = imu.get_gyro()
        bias_ax += a['x']
        bias_ay += a['y']
        bias_az += a['z']
        bias_gz += g['z']
        time.sleep_ms(10)
    bias_ax /= 100
    bias_ay /= 100
    bias_az /= 100
    bias_gz /= 100

    # Initialize analyzers
    print("\n[5/5] Initializing analyzers...")
    bump_counter = BumpCounter(threshold=0.012)
    vib_analyzer = VibrationAnalyzer(window_size=50)
    drift_comp = DriftCompensator()

    # Measure baseline vibration
    for _ in range(100):
        a = imu.get_accel()
        vib_analyzer.add_sample(a['x'], a['y'], a['z'])
        time.sleep_ms(SAMPLE_DELAY_MS)
    vib_analyzer.baseline = vib_analyzer.get_variance()
    print(f"  Baseline vibration: {vib_analyzer.baseline:.2f}")

    # Create calibration runner
    cal_run = CalibrationRun(robot, imu, ultrasonic, bump_counter, vib_analyzer, drift_comp)

    # Ready!
    robot.leds.fill(0, 255, 0)
    robot.leds.show()

    print("\n" + "="*50)
    print("CALIBRATION READY")
    print("="*50)
    print()
    print("Place robot 40-50cm from wall")
    print()
    print("COMMANDS (from host.py):")
    print("  C - Run calibration (forward + backward)")
    print("  V - Run verification test")
    print("  W/S - Manual forward/backward")
    print("  Q - Quit and show results")
    print()
    print("="*50)

    oled.show_idle(test_dist, None)

    # State
    calibration_count = 0
    verification_count = 0
    results = []
    running = True
    manual_mode = False
    last_oled_update = 0
    OLED_UPDATE_MS = 100  # Update OLED max 10 times/sec

    # Pickup detection state - uses NO BUMPS + tilt/rotation
    pickup_count = 0
    PICKUP_SAMPLES = 20  # Require ~40ms of sustained condition
    no_bump_count = 0    # Count samples with no new bumps
    last_bump_total = 0  # Track bumps to detect new ones

    # Streaming batch - new format with debug data
    BATCH_SIZE = 20
    batch = []
    motor_left, motor_right = 0, 0
    us_cm = test_dist if test_dist > 0 else 50.0  # Last valid ultrasonic reading

    # Debug state for streaming
    heading = 0.0  # Integrated heading from gyro

    try:
        while running:
            # Read IMU
            accel = imu.get_accel()
            gyro = imu.get_gyro()
            ax = accel['x'] - bias_ax
            ay = accel['y'] - bias_ay
            az = accel['z'] - bias_az
            gz = gyro['z'] - bias_gz
            gx = gyro['x']
            gy = gyro['y']

            # Pickup detection - uses NO BUMPS + (tilt OR rotation)
            # When picked up: wheels leave ground = no bumps, but tilted/rotating
            raw_az = accel['z']
            gyro_mag = math.sqrt(gx*gx + gy*gy)  # Tilt rotation (not yaw)

            # Track if we got new bumps
            current_bumps = bump_counter.bump_count
            got_bump = current_bumps > last_bump_total
            last_bump_total = current_bumps

            if got_bump:
                no_bump_count = 0  # Reset - wheels are on ground
            else:
                no_bump_count += 1

            # Pickup = no bumps for a while AND (tilted OR rotating)
            # This avoids triggering during normal accel/decel (which have bumps)
            is_tilted = raw_az < 0.7  # Significantly tilted
            is_rotating = gyro_mag > 20  # Rotating in pitch/roll
            no_wheel_contact = no_bump_count > 30  # ~60ms without bumps

            if no_wheel_contact and (is_tilted or is_rotating):
                pickup_count += 1
            else:
                pickup_count = 0

            # Trigger after sustained pickup condition
            if pickup_count >= PICKUP_SAMPLES:
                if cal_run.running or manual_mode:
                    robot.motors.stop()
                    if cal_run.running:
                        cal_run.running = False
                        log(f"PICKUP! az={raw_az:.2f} gyro={gyro_mag:.1f}")
                        print(f"\n>>> PICKUP DETECTED (az={raw_az:.2f}, gyro={gyro_mag:.1f})")
                        oled.show_message("PICKUP!", "Motors off")
                    manual_mode = False
                pickup_count = 0

            # Stream to host (batch samples with debug data)
            ts = time.ticks_us()

            # Integrate heading from gyro (for debug)
            heading += gz * 0.002  # dt = 2ms

            # Ultrasonic: read every 25th sample (~50ms at 500Hz)
            if len(batch) % 25 == 0:
                us_reading = ultrasonic.distance_cm_valid()
                if us_reading > 0:
                    us_cm = us_reading

            # Get current vibration variance
            vib_var = vib_analyzer.get_variance()

            # Get drift correction (if running)
            drift_corr = drift_comp.correction if cal_run.running or manual_mode else 0.0

            # Estimated speed from vibration
            est_speed = vib_analyzer.estimate_speed() or 0.0

            # New packet format v3: raw sensors + debug data
            # Header: [0x03][batch_size]
            # Per sample: Q 7f 6f = ts(8) + raw(28) + debug(24) = 60 bytes
            #   Raw: ax, ay, az, gx, gy, gz, us
            #   Debug: heading, vib, bumps, drift_corr, motor_l, motor_r
            batch.append((
                ts,
                accel['x'], accel['y'], accel['z'], gx, gy, gyro['z'], us_cm,
                heading, vib_var, float(bump_counter.bump_count), drift_corr,
                float(motor_left), float(motor_right)
            ))

            if len(batch) >= BATCH_SIZE:
                # Pack and send batch with debug data
                # Header: [0x03][batch_size] = v3 format
                data = struct.pack('BB', 0x03, BATCH_SIZE)
                for sample in batch:
                    data += struct.pack('<Q13f', *sample)
                try:
                    stream_sock.sendto(data, stream_addr)
                except:
                    pass
                batch = []

            # Check commands
            cmd = controller.check_commands()

            if cmd == 'QUIT':
                print("\n\n>>> QUIT received")
                running = False
                continue

            elif cmd == 'CALIBRATE' and not cal_run.running:
                # Start calibration sequence
                calibration_count += 1
                print(f"\n{'='*50}")
                print(f"CALIBRATION RUN #{calibration_count}")
                print(f"{'='*50}")

                log(f"CAL START #{calibration_count}")
                oled.show_message("CALIBRATE", f"Run #{calibration_count}")
                time.sleep(0.3)

                # Forward run
                cal_run.start(1, CALIBRATION_DISTANCE)
                robot.leds.fill(0, 0, 255)  # Blue = running
                robot.leds.show()

            elif cmd == 'VERIFY' and not cal_run.running:
                if bump_counter.bumps_per_cm is None:
                    log("VERIFY ERROR: Need calibration first")
                    print("\n>>> Need calibration first! Press C")
                    oled.show_message("ERROR", "Cal first!")
                else:
                    verification_count += 1
                    log(f"VERIFY START #{verification_count} B/cm={bump_counter.bumps_per_cm:.3f}")
                    print(f"\n{'='*50}")
                    print(f"VERIFICATION RUN #{verification_count}")
                    print(f"{'='*50}")

                    oled.show_message("VERIFY", f"Run #{verification_count}")
                    time.sleep(0.3)

                    cal_run.start(1, CALIBRATION_DISTANCE)
                    robot.leds.fill(255, 255, 0)  # Yellow = verify
                    robot.leds.show()

            elif isinstance(cmd, tuple) and cmd[0] == 'MOTOR':
                _, left, right = cmd
                if not cal_run.running:
                    manual_mode = abs(left) > 10 or abs(right) > 10
                    if manual_mode:
                        # Apply drift compensation in manual mode too
                        l_adj, r_adj = drift_comp.update(gz)
                        motor_left = int(left + l_adj)
                        motor_right = int(right + r_adj)
                        robot.motors.set_speeds(motor_left, motor_right)
                    else:
                        motor_left, motor_right = 0, 0
                        robot.motors.stop()
                        drift_comp.reset()

            # Update calibration run
            if cal_run.running:
                # Add samples
                vib_analyzer.add_sample(ax, ay, az)

                # Track motor speeds from cal_run
                motor_left = cal_run.motor_left
                motor_right = cal_run.motor_right

                if cal_run.update(ax, ay, az, gz):
                    # Run complete
                    result = {
                        'type': 'cal' if bump_counter.bumps_per_cm is None else 'verify',
                        'direction': 'fwd' if cal_run.direction > 0 else 'bwd',
                        'actual_cm': cal_run.actual_distance,
                        'time_s': cal_run.elapsed_time,
                        'bumps': cal_run.bump_count,
                        'avg_vib': cal_run.avg_vibration,
                        'est_cm': cal_run.estimated_distance,
                    }

                    # Calculate calibration or error
                    if result['type'] == 'cal':
                        # First calibration - calculate ratios
                        b_per_cm = bump_counter.calibrate(result['actual_cm'])
                        v_to_spd = vib_analyzer.calibrate(result['actual_cm'], result['time_s'])

                        result['bumps_per_cm'] = b_per_cm
                        result['vib_to_speed'] = v_to_spd

                        bpcm_str = f"{b_per_cm:.3f}" if b_per_cm else "0"
                        msg = f"CAL: Dist={result['actual_cm']:.1f}cm Bumps={result['bumps']} B/cm={bpcm_str}"
                        log(msg)

                        print(f"\n>>> RUN COMPLETE")
                        print(f"    Distance:  {result['actual_cm']:.1f} cm")
                        print(f"    Time:      {result['time_s']:.2f} s")
                        print(f"    Speed:     {result['actual_cm']/result['time_s']:.1f} cm/s")
                        print(f"    Bumps:     {result['bumps']}")
                        print(f"    Avg Vib:   {result['avg_vib']:.1f}")
                        print()
                        print(f">>> CALIBRATION RESULT:")
                        print(f"    Bumps/cm:    {b_per_cm:.3f}" if b_per_cm else "    Bumps/cm:    FAILED")
                        print(f"    Vib-to-spd:  {v_to_spd:.4f}" if v_to_spd else "    Vib-to-spd:  FAILED")

                        # Show on OLED
                        oled.show_result(result['actual_cm'])

                    else:
                        # Verification - use BUMPS ONLY (compare to target, not ultrasonic)
                        target_cm = CALIBRATION_DISTANCE
                        est_cm = result['est_cm']

                        if est_cm is not None:
                            error_cm = est_cm - target_cm
                            error_pct = (error_cm / target_cm) * 100
                            result['error_cm'] = error_cm
                            result['error_pct'] = error_pct

                            msg = f"VERIFY: Est={est_cm:.1f}cm Target={target_cm:.0f}cm Err={error_pct:+.1f}% Bumps={result['bumps']}"
                            log(msg)

                            print(f"\n>>> VERIFICATION COMPLETE (bumps only)")
                            print(f"    Target:    {target_cm:.0f} cm")
                            print(f"    Estimated: {est_cm:.1f} cm (from bumps)")
                            print(f"    Error:     {error_cm:+.1f} cm ({error_pct:+.1f}%)")
                            print(f"    Bumps:     {result['bumps']}")
                            print(f"    Time:      {result['time_s']:.2f} s")

                            # Show on OLED with error
                            oled.show_result(target_cm, est_cm, error_pct)
                        else:
                            msg = f"VERIFY: No estimate - Bumps={result['bumps']} B/cm={bump_counter.bumps_per_cm}"
                            log(msg)
                            print(f"\n>>> VERIFICATION (no estimate - run calibration first)")
                            print(f"    Bumps:     {result['bumps']}")

                    results.append(result)

                    # Check if we should do backward run
                    if cal_run.direction > 0:
                        time.sleep(0.5)
                        log("BACKWARD RUN")
                        print("\n>>> Now backward...")
                        oled.show_message("BACKWARD", f"{CALIBRATION_DISTANCE:.0f}cm")
                        time.sleep(0.3)
                        cal_run.start(-1, CALIBRATION_DISTANCE)
                    else:
                        if bump_counter.bumps_per_cm:
                            log(f"DONE B/cm={bump_counter.bumps_per_cm:.3f}")
                        else:
                            log("DONE")
                        robot.leds.fill(0, 255, 0)
                        robot.leds.show()
                        print("\n>>> Sequence complete. Press C for more calibration, V to verify")

                        # Show summary on OLED
                        oled.show_idle(ultrasonic.distance_cm_valid(), bump_counter.bumps_per_cm)

            # Status output (when not in calibration)
            if not cal_run.running and not manual_mode:
                dist = ultrasonic.distance_cm_valid()
                vib = vib_analyzer.get_variance()

                # Only print occasionally (every ~500ms)
                if time.ticks_ms() % 500 < 10:
                    status = f"\r[IDLE] US:{dist:5.1f}cm V:{vib:5.1f}"
                    if bump_counter.bumps_per_cm:
                        status += f" B/cm:{bump_counter.bumps_per_cm:.2f}"
                    status += "  "
                    print(status, end='')

                    # Update OLED
                    oled.show_idle(dist, bump_counter.bumps_per_cm)

            elif cal_run.running:
                # Print run status (throttled)
                now = time.ticks_ms()
                if time.ticks_diff(now, last_oled_update) >= OLED_UPDATE_MS:
                    last_oled_update = now

                    status = cal_run.get_status()
                    print(f"\r{status}  ", end='')

                    # Send status via UDP every 500ms
                    if now % 500 < OLED_UPDATE_MS:
                        log(f"RUN: H={drift_comp.heading:.1f}° B={bump_counter.run_bumps} V={vib_analyzer.get_variance():.1f}")

                    # Update OLED during run
                    traveled = bump_counter.run_bumps / bump_counter.bumps_per_cm if bump_counter.bumps_per_cm else 0
                    oled.show_run_status(cal_run.direction, traveled, bump_counter.run_bumps)

            time.sleep_ms(SAMPLE_DELAY_MS)

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        robot.motors.stop()
        robot.leds.fill(0, 0, 0)
        robot.leds.show()
        oled.show_message("STOPPED")

        # Print final summary
        print("\n" + "="*50)
        print("CALIBRATION SUMMARY")
        print("="*50)

        if bump_counter.bumps_per_cm:
            print(f"\nFinal Calibration:")
            print(f"  Bumps per cm:  {bump_counter.bumps_per_cm:.3f}")
            print(f"  Bumps per m:   {bump_counter.bumps_per_cm * 100:.1f}")
            if vib_analyzer.vib_to_speed:
                print(f"  Vib-to-speed:  {vib_analyzer.vib_to_speed:.4f} cm/s per unit")

        if results:
            print(f"\nAll Runs ({len(results)} total):")
            print(f"{'#':>3} {'Type':>6} {'Dir':>4} {'Actual':>8} {'Est':>8} {'Error':>8} {'Bumps':>6}")
            print("-" * 50)
            for i, r in enumerate(results):
                est_str = f"{r['est_cm']:.1f}cm" if r.get('est_cm') else "---"
                err_str = f"{r['error_cm']:+.1f}cm" if r.get('error_cm') is not None else "---"
                print(f"{i+1:>3} {r['type']:>6} {r['direction']:>4} "
                      f"{r['actual_cm']:>6.1f}cm {est_str:>8} {err_str:>8} {r['bumps']:>6}")

        print("\n" + "="*50)


if __name__ == "__main__":
    main()
