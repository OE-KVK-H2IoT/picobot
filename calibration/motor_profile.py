"""
Motor characterization — dead zone, speed curve, and balance profiling.

Tests each motor independently and together to build a per-robot profile.
Results are saved to calibration.json.

Run on Pico:  import calibration.motor_profile
"""

from picobot import Robot
import time
from .cal_utils import save_cal, banner, result

robot = Robot(encoders=True)

motor_data = {
    "left": {"dead_zone": 0, "speed_curve": []},
    "right": {"dead_zone": 0, "speed_curve": []},
    "balance_drift_dps": 0.0,
}


# ─── Dead Zone Detection ───────────────────────────────────────

def test_dead_zone(name, motor, encoder):
    """Ramp PWM from 0 until encoder detects motion.

    Args:
        name: 'left' or 'right'
        motor: Motor instance
        encoder: Encoder instance
    """
    banner(f"Dead Zone: {name} motor")
    print(f"Ramping {name} motor PWM from 0 to 80...")

    dead_zone = 0
    encoder.reset()

    for pwm in range(0, 82, 2):
        motor.set_speed(pwm)
        time.sleep(0.2)
        encoder.update()

        if abs(encoder.speed_tps) > 5:
            dead_zone = pwm
            print(f"  Motion detected at PWM {pwm} ({encoder.speed_tps:.0f} tps)")
            break
    else:
        dead_zone = 80
        print("  No motion detected up to PWM 80!")

    motor.stop()
    time.sleep(0.3)

    motor_data[name]["dead_zone"] = dead_zone
    result(f"{name} dead zone", dead_zone, "PWM")


# ─── Speed Curve ────────────────────────────────────────────────

def test_speed_curve(name, motor, encoder):
    """Measure ticks/s at multiple PWM levels.

    Args:
        name: 'left' or 'right'
        motor: Motor instance
        encoder: Encoder instance
    """
    dz = motor_data[name]["dead_zone"]
    levels = [dz + 10, 60, 80, 100, 120, 160, 200, 255]
    # Remove levels below dead zone + 10
    levels = sorted(set(l for l in levels if l >= dz + 10))

    banner(f"Speed Curve: {name} motor")
    print(f"  {'PWM':>5}  {'tps':>8}")
    print(f"  {'-'*5}  {'-'*8}")

    curve = []
    for pwm in levels:
        encoder.reset()
        motor.set_speed(pwm)
        time.sleep(0.3)  # Let motor reach steady state

        # Sample speed for 1 second
        samples = []
        for _ in range(20):
            encoder.update()
            samples.append(abs(encoder.speed_tps))
            time.sleep(0.05)

        motor.stop()
        time.sleep(0.2)

        avg_tps = sum(samples) / len(samples)
        curve.append([pwm, round(avg_tps, 1)])
        print(f"  {pwm:>5}  {avg_tps:>8.1f}")

    motor_data[name]["speed_curve"] = curve


# ─── Balance Test ───────────────────────────────────────────────

def test_balance():
    """Drive both motors at equal PWM and measure heading drift."""
    banner("Balance Test")
    print("Calibrating IMU...")
    robot.imu.calibrate()

    print(f"Driving both motors at PWM 120 for 3 seconds...")
    robot.imu.reset_heading()

    robot.motors.set_speed(120, 120)
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < 3000:
        robot.imu.update()
        time.sleep(0.02)

    robot.motors.stop()
    robot.imu.update()
    time.sleep(0.3)

    drift = robot.imu.heading
    drift_dps = drift / 3.0

    result("Total heading drift", drift, "deg")
    result("Drift rate", drift_dps, "deg/s")

    if abs(drift_dps) < 1:
        print("  Excellent balance!")
    elif abs(drift_dps) < 3:
        print("  Acceptable — small trim may help.")
    else:
        direction = "left" if drift > 0 else "right"
        print(f"  Significant drift to {direction}.")
        print(f"  Try reducing {direction} motor PWM by ~{abs(drift_dps) * 2:.0f}")

    motor_data["balance_drift_dps"] = round(drift_dps, 2)


# ─── Main ───────────────────────────────────────────────────────

def main():
    """Run all motor characterization tests."""
    banner("Motor Characterization")
    print("This script profiles each motor individually,")
    print("then tests left-right balance.")
    print()
    input("Place robot on a flat surface. Press Enter to start...")

    enc_left = robot.encoders.left
    enc_right = robot.encoders.right

    # Dead zone detection
    test_dead_zone("left", robot.motors.left, enc_left)
    if enc_right:
        test_dead_zone("right", robot.motors.right, enc_right)

    # Speed curves
    test_speed_curve("left", robot.motors.left, enc_left)
    if enc_right:
        test_speed_curve("right", robot.motors.right, enc_right)

    # Balance test
    test_balance()

    # Summary
    banner("Motor Profile Summary")
    for name in ("left", "right"):
        dz = motor_data[name]["dead_zone"]
        n_pts = len(motor_data[name]["speed_curve"])
        print(f"  {name}: dead_zone={dz} PWM, {n_pts} speed-curve points")
    print(f"  balance drift: {motor_data['balance_drift_dps']} deg/s")

    # Save
    save_cal({"motors": motor_data})
    print("\nDone!")


try:
    main()
finally:
    robot.motors.stop()
