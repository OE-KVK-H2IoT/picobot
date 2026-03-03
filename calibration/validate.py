"""
Calibration validation — statistical accuracy tests.

Requires calibration.json with distance.mm_per_tick from distance_cal.py.
Runs repeated distance and turn trials, computes error statistics,
and assigns a letter grade.

Run on Pico:  import calibration.validate
"""

from picobot import Robot
import time
from .cal_utils import (
    load_cal, save_cal, stats, median5, try_tof,
    banner, result,
)

NUM_TRIALS = 5
TARGET_DIST_MM = 200
TARGET_TURN_DEG = 90

# Straight-line correction
_KP_STRAIGHT = 1
_MAX_CORRECTION = 40

robot = Robot(encoders=True)


def _read_distance_mm(tof):
    """Read distance in mm using ToF if available, else ultrasonic."""
    if tof:
        return median5(tof.read_mm)
    d_cm = median5(robot.sensors.distance.read_cm)
    if d_cm < 0:
        return -1
    return d_cm * 10  # Convert to mm


def _drive_by_ticks(target_mm, mm_per_tick, speed=100):
    """Drive forward until encoder reaches target distance.

    Uses encoder P-correction to maintain straight line.
    """
    target_ticks = target_mm / mm_per_tick
    robot.encoders.reset()
    has_right = robot.encoders.right is not None

    while True:
        left_t = robot.encoders.left.ticks
        right_t = robot.encoders.right.ticks if has_right else left_t
        avg = (abs(left_t) + abs(right_t)) / 2
        if avg >= target_ticks:
            break
        # Straight-line correction
        error = abs(left_t) - abs(right_t)
        correction = int(_KP_STRAIGHT * error)
        correction = max(-_MAX_CORRECTION, min(_MAX_CORRECTION, correction))
        robot.motors.set_speed(speed - correction, speed + correction)
        time.sleep(0.01)

    robot.motors.stop()
    time.sleep(0.3)


def _drive_back_straight(duration_s, speed=100):
    """Drive backward for duration_s with encoder correction."""
    robot.encoders.reset()
    has_right = robot.encoders.right is not None
    end = time.ticks_ms() + int(duration_s * 1000)

    while time.ticks_diff(end, time.ticks_ms()) > 0:
        left_t = robot.encoders.left.ticks
        right_t = robot.encoders.right.ticks if has_right else left_t
        error = abs(left_t) - abs(right_t)
        correction = int(_KP_STRAIGHT * error)
        correction = max(-_MAX_CORRECTION, min(_MAX_CORRECTION, correction))
        robot.motors.set_speed(-speed + correction, -speed - correction)
        time.sleep(0.02)

    robot.motors.stop()
    time.sleep(0.3)


def _turn_by_imu(target_deg):
    """Turn by target degrees using IMU feedback."""
    robot.imu.reset_heading()
    speed = 80

    if target_deg > 0:
        robot.motors.set_speed(-speed, speed)
    else:
        robot.motors.set_speed(speed, -speed)

    while abs(robot.imu.heading) < abs(target_deg) - 2:
        robot.imu.update()
        time.sleep(0.01)

    robot.motors.stop()
    robot.imu.update()
    time.sleep(0.3)
    return robot.imu.heading


# ─── Distance Test ──────────────────────────────────────────────

def test_distance(mm_per_tick, tof):
    """Run repeated distance trials, measure actual vs commanded."""
    banner(f"Distance Accuracy ({NUM_TRIALS} trials)")
    print(f"Commanded: {TARGET_DIST_MM} mm per trial")
    print("Face robot toward a flat wall, ~60 cm away.")
    input("Press Enter to start...")

    errors = []
    print(f"\n  {'Trial':>5}  {'Start':>7}  {'End':>7}  {'Actual':>7}  {'Error':>7}")
    print(f"  {'-'*5}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*7}")

    for i in range(NUM_TRIALS):
        d_start = _read_distance_mm(tof)
        if d_start < 0:
            print(f"  {i+1:>5}  sensor error — skipped")
            continue

        _drive_by_ticks(TARGET_DIST_MM, mm_per_tick)

        d_end = _read_distance_mm(tof)
        if d_end < 0:
            print(f"  {i+1:>5}  sensor error — skipped")
            continue

        actual = d_start - d_end  # Closer to wall = smaller
        error_mm = actual - TARGET_DIST_MM
        error_pct = (error_mm / TARGET_DIST_MM) * 100
        errors.append(abs(error_pct))

        print(f"  {i+1:>5}  {d_start:>6.0f}  {d_end:>6.0f}  "
              f"{actual:>6.0f}  {error_pct:>+6.1f}%")

        # Drive back to start position
        _drive_back_straight(1.5)

    return errors


# ─── Turn Test ──────────────────────────────────────────────────

def test_turns():
    """Run repeated turn trials, measure actual vs commanded."""
    banner(f"Turn Accuracy ({NUM_TRIALS} trials)")
    print(f"Commanded: {TARGET_TURN_DEG} deg per trial")
    print("Calibrating IMU...")
    robot.imu.calibrate()
    input("Press Enter to start...")

    errors = []
    print(f"\n  {'Trial':>5}  {'Actual':>8}  {'Error':>8}")
    print(f"  {'-'*5}  {'-'*8}  {'-'*8}")

    for i in range(NUM_TRIALS):
        actual = _turn_by_imu(TARGET_TURN_DEG)
        error = actual - TARGET_TURN_DEG
        errors.append(abs(error))

        print(f"  {i+1:>5}  {actual:>7.1f}°  {error:>+7.1f}°")

        # Turn back to start
        _turn_by_imu(-TARGET_TURN_DEG)
        time.sleep(0.5)

    return errors


# ─── Grading ────────────────────────────────────────────────────

def grade(dist_errors, turn_errors):
    """Assign letter grade based on error statistics."""
    if not dist_errors or not turn_errors:
        return "N/A", 0, 0

    d_mean = sum(dist_errors) / len(dist_errors)
    t_mean = sum(turn_errors) / len(turn_errors)

    if d_mean < 2 and t_mean < 2:
        return "A", d_mean, t_mean
    elif d_mean < 5 and t_mean < 5:
        return "B", d_mean, t_mean
    elif d_mean < 10 and t_mean < 10:
        return "C", d_mean, t_mean
    else:
        return "F", d_mean, t_mean


# ─── Main ───────────────────────────────────────────────────────

def main():
    """Run full validation suite."""
    banner("Calibration Validation")

    cal = load_cal()
    if "distance" not in cal or "mm_per_tick" not in cal["distance"]:
        print("ERROR: No distance calibration found!")
        print("Run distance_cal.py first.")
        return

    mm_per_tick = cal["distance"]["mm_per_tick"]
    print(f"Using mm_per_tick = {mm_per_tick:.4f} "
          f"(method: {cal['distance'].get('method', '?')})")

    tof = try_tof(robot.imu._i2c)

    # Distance test
    dist_errors = test_distance(mm_per_tick, tof)

    # Turn test
    turn_errors = test_turns()

    # Summary
    banner("Validation Summary")

    if dist_errors:
        d_mean, d_std, d_min, d_max = stats(dist_errors)
        print("  Distance:")
        result("    Mean error", d_mean, "%")
        result("    Std dev", d_std, "%")
        result("    Max error", d_max, "%")
    else:
        d_mean = 99
        print("  Distance: no valid trials")

    if turn_errors:
        t_mean, t_std, t_min, t_max = stats(turn_errors)
        print("  Turns:")
        result("    Mean error", t_mean, "deg")
        result("    Std dev", t_std, "deg")
        result("    Max error", t_max, "deg")
    else:
        t_mean = 99
        print("  Turns: no valid trials")

    letter, d_pct, t_deg = grade(dist_errors, turn_errors)
    print(f"\n  Grade: {letter}")
    print(f"  (distance {d_pct:.1f}%, turn {t_deg:.1f}°)")

    # Save
    save_cal({
        "validation": {
            "distance_error_pct": round(d_pct, 2),
            "turn_error_deg": round(t_deg, 2),
            "grade": letter,
            "trials": NUM_TRIALS,
        }
    })


try:
    main()
finally:
    robot.motors.stop()
