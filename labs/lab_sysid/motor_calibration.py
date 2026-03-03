"""
==============================================================================
MOTOR SYSTEM IDENTIFICATION - CALIBRATION SCRIPT
==============================================================================

Runs on the Pico to collect data for motor model identification.

WHAT THIS MEASURES:
-------------------
1. Steady-state response: PWM duty → actual angular velocity
2. Step response: Time constant τ (how fast motor reaches final speed)
3. Startup threshold: Minimum PWM to overcome static friction
4. Left/Right difference: Motor imbalance for calibration

HOW IT WORKS:
-------------
1. Apply known PWM values to motors
2. Measure angular velocity via IMU gyroscope (during tank turns)
3. Stream data to host via UDP for analysis
4. Host fits model parameters using regression

MEASUREMENT APPROACH:
--------------------
- Tank turn (one motor fwd, one bwd) → robot rotates in place
- Gyro Z measures rotation rate in °/s
- Wheel angular velocity ∝ robot rotation rate (geometry factor)

DATA COLLECTED:
--------------
- timestamp_ms: Time since start
- pwm_left: PWM command to left motor (0-255)
- pwm_right: PWM command to right motor (0-255)
- gyro_z: Angular velocity from IMU (°/s)
- test_type: 'sweep', 'step', 'startup', 'balance'

USAGE:
------
1. Place robot on smooth surface with wheels free to spin
2. Run this script
3. Robot will perform calibration sequence (~60 seconds)
4. Data streams to host for analysis

==============================================================================
"""

import time
from machine import Pin, I2C

# Import robot libraries
import sys
sys.path.append('/lib')

from picobot import Motors
from bmi160 import BMI160

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Network settings (change to your host IP)
HOST_IP = "192.168.1.255"  # Broadcast - host will receive
UDP_PORT = 5010

# Calibration parameters
PWM_MIN = 0
PWM_MAX = 255
PWM_STEP = 15        # Steps for sweep
SWEEP_DWELL_MS = 800  # Time at each PWM level (ms)
STEP_DURATION_MS = 1500  # Duration for step response
STARTUP_MAX_PWM = 80  # Max PWM to test for startup

# IMU settings
IMU_SAMPLE_RATE_HZ = 100
I2C_SCL = 5
I2C_SDA = 4

# Battery monitoring (optional - set to None to disable)
BATTERY_ADC_PIN = 28   # GP28 = ADC2
BATTERY_DIVIDER = 2.0  # Voltage divider ratio (V_bat / V_adc)

# ==============================================================================
# DATA LOGGER (SIMPLE UDP)
# ==============================================================================

class CalibrationLogger:
    """Simple UDP logger for calibration data."""

    def __init__(self, host_ip, port):
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = (host_ip, port)
        self.start_ms = time.ticks_ms()

    def log(self, pwm_left, pwm_right, gyro_z, test_type):
        """Send calibration data point."""
        t_ms = time.ticks_diff(time.ticks_ms(), self.start_ms)
        # CSV format: timestamp,pwm_left,pwm_right,gyro_z,test_type
        msg = f"{t_ms},{pwm_left},{pwm_right},{gyro_z:.3f},{test_type}\n"
        try:
            self.sock.sendto(msg.encode(), self.host)
        except:
            pass

    def send_header(self):
        """Send CSV header."""
        header = "timestamp_ms,pwm_left,pwm_right,gyro_z,test_type\n"
        try:
            self.sock.sendto(header.encode(), self.host)
        except:
            pass

    def send_marker(self, marker):
        """Send a marker/comment line."""
        msg = f"# {marker}\n"
        try:
            self.sock.sendto(msg.encode(), self.host)
        except:
            pass

# ==============================================================================
# CALIBRATION ROUTINES
# ==============================================================================

def wait_for_settle(imu, threshold=2.0, timeout_ms=2000):
    """Wait for robot to stop moving."""
    start = time.ticks_ms()
    stable_count = 0

    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        gyro = imu.get_gyro()
        if abs(gyro['z']) < threshold:
            stable_count += 1
            if stable_count > 10:
                return True
        else:
            stable_count = 0
        time.sleep_ms(10)

    return False


def calibrate_steady_state(motors, imu, logger):
    """
    Measure steady-state response: PWM → angular velocity.

    Uses tank turn (opposite motor directions) so robot spins in place.
    Gyro Z measures the resulting angular velocity.
    """
    logger.send_marker("STEADY_STATE_SWEEP_START")
    print("\n=== Steady-State Calibration ===")
    print("Robot will spin at different speeds...")

    # Sweep from low to high PWM
    pwm_values = list(range(PWM_MIN, PWM_MAX + 1, PWM_STEP))

    for pwm in pwm_values:
        print(f"  PWM={pwm}...", end=" ")

        # Tank turn: left forward, right backward
        motors.set_speeds(pwm, -pwm)

        # Wait for speed to stabilize
        time.sleep_ms(300)

        # Collect samples at steady state
        samples = []
        for _ in range(int(SWEEP_DWELL_MS / 10)):
            gyro = imu.get_gyro()
            samples.append(gyro['z'])
            logger.log(pwm, -pwm, gyro['z'], 'sweep')
            time.sleep_ms(10)

        avg_gyro = sum(samples) / len(samples)
        print(f"gyro_z = {avg_gyro:.1f} °/s")

        motors.stop()
        time.sleep_ms(200)

    motors.stop()
    wait_for_settle(imu)
    logger.send_marker("STEADY_STATE_SWEEP_END")


def calibrate_step_response(motors, imu, logger):
    """
    Measure step response: How fast motor reaches final speed.

    This reveals the time constant τ = L/R (electrical) and J/B (mechanical).
    """
    logger.send_marker("STEP_RESPONSE_START")
    print("\n=== Step Response Calibration ===")
    print("Robot will do sudden speed changes...")

    test_pwm = 150  # Mid-range PWM for step test

    for direction in ['CW', 'CCW']:
        print(f"  Step response {direction}...")

        # Ensure stopped
        motors.stop()
        wait_for_settle(imu)
        time.sleep_ms(500)

        # Log baseline (stopped)
        for _ in range(20):
            gyro = imu.get_gyro()
            logger.log(0, 0, gyro['z'], f'step_{direction}_baseline')
            time.sleep_ms(10)

        # Apply step
        if direction == 'CW':
            motors.set_speeds(test_pwm, -test_pwm)
        else:
            motors.set_speeds(-test_pwm, test_pwm)

        # Log response
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < STEP_DURATION_MS:
            gyro = imu.get_gyro()
            logger.log(test_pwm if direction == 'CW' else -test_pwm,
                      -test_pwm if direction == 'CW' else test_pwm,
                      gyro['z'], f'step_{direction}')
            time.sleep_ms(10)

        motors.stop()

        # Log settling
        for _ in range(50):
            gyro = imu.get_gyro()
            logger.log(0, 0, gyro['z'], f'step_{direction}_settle')
            time.sleep_ms(10)

    motors.stop()
    logger.send_marker("STEP_RESPONSE_END")


def calibrate_startup_threshold(motors, imu, logger):
    """
    Find minimum PWM to overcome static friction.

    Slowly ramp up PWM until motor starts moving.
    """
    logger.send_marker("STARTUP_THRESHOLD_START")
    print("\n=== Startup Threshold Calibration ===")
    print("Finding minimum PWM to start motors...")

    threshold_left = 0
    threshold_right = 0

    # Test left motor (tank turn right)
    print("  Testing left motor...", end=" ")
    motors.stop()
    wait_for_settle(imu)

    for pwm in range(0, STARTUP_MAX_PWM, 2):
        motors.set_speeds(pwm, -pwm)  # Left fwd, right bwd
        time.sleep_ms(100)

        gyro = imu.get_gyro()
        logger.log(pwm, -pwm, gyro['z'], 'startup_left')

        if abs(gyro['z']) > 5.0:  # Movement detected
            threshold_left = pwm
            print(f"starts at PWM={pwm}")
            break

    motors.stop()
    wait_for_settle(imu)

    # Test right motor (tank turn left)
    print("  Testing right motor...", end=" ")

    for pwm in range(0, STARTUP_MAX_PWM, 2):
        motors.set_speeds(-pwm, pwm)  # Left bwd, right fwd
        time.sleep_ms(100)

        gyro = imu.get_gyro()
        logger.log(-pwm, pwm, gyro['z'], 'startup_right')

        if abs(gyro['z']) > 5.0:  # Movement detected
            threshold_right = pwm
            print(f"starts at PWM={pwm}")
            break

    motors.stop()
    logger.send_marker(f"STARTUP_THRESHOLD_END left={threshold_left} right={threshold_right}")

    return threshold_left, threshold_right


def calibrate_motor_balance(motors, imu, logger):
    """
    Measure left vs right motor difference.

    Same PWM to both motors should go straight, but usually curves.
    This measures the imbalance for software compensation.
    """
    logger.send_marker("MOTOR_BALANCE_START")
    print("\n=== Motor Balance Calibration ===")
    print("Measuring left/right motor difference...")

    test_pwm = 120

    # Forward motion - both motors same speed
    print(f"  Forward at PWM={test_pwm}...")
    motors.forward(test_pwm)
    time.sleep_ms(200)  # Let it stabilize

    samples = []
    for _ in range(100):
        gyro = imu.get_gyro()
        samples.append(gyro['z'])
        logger.log(test_pwm, test_pwm, gyro['z'], 'balance_fwd')
        time.sleep_ms(10)

    motors.stop()
    drift_fwd = sum(samples) / len(samples)
    print(f"    Drift: {drift_fwd:.2f} °/s (+ = curves left, - = curves right)")

    wait_for_settle(imu)

    # Backward motion
    print(f"  Backward at PWM={test_pwm}...")
    motors.backward(test_pwm)
    time.sleep_ms(200)

    samples = []
    for _ in range(100):
        gyro = imu.get_gyro()
        samples.append(gyro['z'])
        logger.log(-test_pwm, -test_pwm, gyro['z'], 'balance_bwd')
        time.sleep_ms(10)

    motors.stop()
    drift_bwd = sum(samples) / len(samples)
    print(f"    Drift: {drift_bwd:.2f} °/s")

    logger.send_marker(f"MOTOR_BALANCE_END drift_fwd={drift_fwd:.2f} drift_bwd={drift_bwd:.2f}")

    return drift_fwd, drift_bwd


# ==============================================================================
# MAIN CALIBRATION ROUTINE
# ==============================================================================

def run_calibration():
    """Run full motor calibration sequence."""
    print("\n" + "="*60)
    print("MOTOR SYSTEM IDENTIFICATION")
    print("="*60)

    # Initialize hardware
    print("\nInitializing hardware...")

    # Motors
    motors = Motors()
    print("  Motors: OK")

    # IMU
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    imu = BMI160(i2c)
    print("  BMI160 IMU: OK")

    # Battery monitor (optional)
    battery = None
    V_cal = 3.7  # Default nominal
    if BATTERY_ADC_PIN is not None:
        try:
            from motor_model import BatteryMonitor
            battery = BatteryMonitor(adc_pin=BATTERY_ADC_PIN, divider_ratio=BATTERY_DIVIDER)
            V_cal = battery.voltage
            print(f"  Battery: {battery.status_string()}")
        except Exception as e:
            print(f"  Battery: Not available ({e})")

    # Data logger
    logger = CalibrationLogger(HOST_IP, UDP_PORT)
    logger.send_header()
    print(f"  Logger: -> {HOST_IP}:{UDP_PORT}")

    # Send battery voltage for calibration record
    logger.send_marker(f"V_CALIBRATION={V_cal:.2f}")

    # Wait for user
    print("\n" + "-"*60)
    print("READY TO START CALIBRATION")
    print("Place robot on smooth surface with wheels free to rotate.")
    print("Press ENTER or wait 5 seconds to begin...")
    print("-"*60)
    time.sleep(5)

    try:
        # Run calibration routines
        calibrate_startup_threshold(motors, imu, logger)
        calibrate_steady_state(motors, imu, logger)
        calibrate_step_response(motors, imu, logger)
        calibrate_motor_balance(motors, imu, logger)

        print("\n" + "="*60)
        print("CALIBRATION COMPLETE!")
        print("Data has been streamed to host for analysis.")
        print("="*60)

    except KeyboardInterrupt:
        print("\nCalibration interrupted!")
    finally:
        motors.stop()
        print("Motors stopped.")


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
    run_calibration()
