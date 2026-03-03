"""
Distance & PPR calibration — four methods to determine mm_per_tick.

Methods:
  A — Ruler: drive, measure with ruler, compute
  B — Ultrasonic wall: measure delta distance with HC-SR04
  C — Opto track: count edges on striped calibration track
  D — ToF: measure delta distance with VL53L1X (optional)

Run on Pico:  import calibration.distance_cal
"""

from picobot import Robot
import time
from .cal_utils import (
    load_cal, save_cal, stats, median5, try_tof,
    banner, result,
)

# Drive parameters
DRIVE_PWM = 120
DRIVE_TIME = 3  # seconds

robot = Robot(encoders=True)
results = {}  # method_name -> mm_per_tick


# Straight-line correction gain — matches left/right encoder ticks
_KP_STRAIGHT = 1
_MAX_CORRECTION = 40  # Never adjust more than this from base speed


def _drive_straight(base_speed, duration_s):
    """Drive straight for duration_s using encoder P-correction.

    Adjusts left/right PWM to keep encoder ticks balanced.
    Returns average encoder ticks.
    """
    robot.encoders.reset()
    has_right = robot.encoders.right is not None
    end = time.ticks_ms() + int(duration_s * 1000)

    while time.ticks_diff(end, time.ticks_ms()) > 0:
        left_t = robot.encoders.left.ticks
        right_t = robot.encoders.right.ticks if has_right else left_t
        # Positive error = left is ahead → slow left, speed up right
        error = abs(left_t) - abs(right_t)
        correction = int(_KP_STRAIGHT * error)
        # Clamp so correction never reverses a motor
        correction = max(-_MAX_CORRECTION, min(_MAX_CORRECTION, correction))
        if base_speed > 0:
            robot.motors.set_speed(base_speed - correction,
                                   base_speed + correction)
        else:
            robot.motors.set_speed(base_speed + correction,
                                   base_speed - correction)
        time.sleep(0.02)

    robot.motors.stop()
    time.sleep(0.3)

    left = abs(robot.encoders.left.ticks)
    right = abs(robot.encoders.right.ticks) if has_right else left
    return (left + right) / 2


def _drive_and_count():
    """Drive forward for DRIVE_TIME seconds, return average encoder ticks."""
    return _drive_straight(DRIVE_PWM, DRIVE_TIME)


def _drive_backward_and_count():
    """Drive backward for DRIVE_TIME seconds, return average encoder ticks."""
    return _drive_straight(-DRIVE_PWM, DRIVE_TIME)


# ─── Method A: Ruler ────────────────────────────────────────────

def method_ruler():
    """Calibrate using a ruler — simplest, always available."""
    banner("Method A: Ruler")
    print("Place robot at a marked start position.")
    print(f"It will drive forward for {DRIVE_TIME}s at PWM {DRIVE_PWM}.")
    input("Press Enter when ready...")

    avg_ticks = _drive_and_count()
    print(f"\nAverage ticks: {avg_ticks:.0f}")

    dist_str = input("Measure distance traveled (mm): ")
    try:
        dist_mm = float(dist_str)
    except ValueError:
        print("Invalid input — skipping.")
        return

    mpt = dist_mm / avg_ticks
    results["ruler"] = mpt
    result("mm_per_tick (ruler)", mpt, "mm/tick")


# ─── Method B: Ultrasonic Wall ──────────────────────────────────

def method_ultrasonic():
    """Calibrate using ultrasonic distance to a wall."""
    banner("Method B: Ultrasonic Wall")
    print("Face the robot toward a flat wall, ~80 cm away.")
    print("The robot will drive BACKWARD to increase distance.")
    input("Press Enter when ready...")

    d_start = median5(robot.sensors.distance.read_cm)
    if d_start < 0:
        print("Ultrasonic error — skipping.")
        return
    print(f"Start distance: {d_start:.1f} cm")

    avg_ticks = _drive_backward_and_count()

    d_end = median5(robot.sensors.distance.read_cm)
    if d_end < 0:
        print("Ultrasonic error — skipping.")
        return
    print(f"End distance:   {d_end:.1f} cm")

    delta_mm = (d_end - d_start) * 10
    if delta_mm <= 0:
        print("Distance didn't increase — check orientation.")
        return

    mpt = delta_mm / avg_ticks
    results["ultrasonic"] = mpt
    result("Delta distance", delta_mm, "mm")
    result("Average ticks", avg_ticks)
    result("mm_per_tick (ultrasonic)", mpt, "mm/tick")


# ─── Method C: Opto Track ──────────────────────────────────────

def method_opto():
    """Calibrate using a striped calibration track (5mm black/5mm white)."""
    banner("Method C: Opto Track")
    print("Place robot on a striped calibration track.")
    print("Track spec: 5mm black / 5mm white (10mm per cycle).")
    print("Using first line sensor (GP2) to count edges.")
    input("Press Enter when ready...")

    robot.encoders.reset()
    has_right = robot.encoders.right is not None
    edges = 0
    last_val = robot.sensors.line.read_raw()[0]
    opto_speed = 80  # Slower for edge accuracy

    start = time.ticks_ms()
    end = start + DRIVE_TIME * 1000

    while time.ticks_diff(end, time.ticks_ms()) > 0:
        # Count edges
        val = robot.sensors.line.read_raw()[0]
        if val != last_val:
            edges += 1
            last_val = val
        # Straight-line correction
        left_t = robot.encoders.left.ticks
        right_t = robot.encoders.right.ticks if has_right else left_t
        error = abs(left_t) - abs(right_t)
        correction = int(_KP_STRAIGHT * error)
        correction = max(-_MAX_CORRECTION, min(_MAX_CORRECTION, correction))
        robot.motors.set_speed(opto_speed - correction,
                               opto_speed + correction)
        time.sleep_ms(2)  # Poll fast

    robot.motors.stop()
    time.sleep(0.3)

    left = abs(robot.encoders.left.ticks)
    right = abs(robot.encoders.right.ticks) if has_right else left
    avg_ticks = (left + right) / 2

    if edges < 4:
        print(f"Only {edges} edges detected — not enough. Check track and sensor.")
        return

    dist_mm = edges * 5  # Each edge = 5mm (half cycle)
    mpt = dist_mm / avg_ticks
    results["opto"] = mpt
    result("Edges detected", edges)
    result("Track distance", dist_mm, "mm")
    result("Average ticks", avg_ticks)
    result("mm_per_tick (opto)", mpt, "mm/tick")


# ─── Method D: ToF ─────────────────────────────────────────────

def method_tof():
    """Calibrate using VL53L1X ToF sensor (mm precision)."""
    banner("Method D: ToF (VL53L1X)")

    tof = try_tof(robot.imu._i2c)
    if tof is None:
        print("No VL53L1X detected — skipping.")
        return

    print("Face the robot toward a flat wall, ~80 cm away.")
    print("The robot will drive BACKWARD to increase distance.")
    input("Press Enter when ready...")

    d_start = median5(tof.read_mm)
    if d_start < 0:
        print("ToF read error — skipping.")
        return
    print(f"Start distance: {d_start} mm")

    avg_ticks = _drive_backward_and_count()

    d_end = median5(tof.read_mm)
    if d_end < 0:
        print("ToF read error — skipping.")
        return
    print(f"End distance:   {d_end} mm")

    delta_mm = d_end - d_start
    if delta_mm <= 0:
        print("Distance didn't increase — check orientation.")
        return

    mpt = delta_mm / avg_ticks
    results["tof"] = mpt
    result("Delta distance", delta_mm, "mm")
    result("Average ticks", avg_ticks)
    result("mm_per_tick (tof)", mpt, "mm/tick")


# ─── Interactive Menu ───────────────────────────────────────────

def show_comparison():
    """Print comparison table if multiple methods were run."""
    if len(results) < 2:
        return
    banner("Method Comparison")
    print(f"  {'Method':<12} {'mm/tick':>10}")
    print(f"  {'-'*12} {'-'*10}")
    for name, mpt in sorted(results.items()):
        print(f"  {name:<12} {mpt:>10.4f}")
    vals = list(results.values())
    mean, std, lo, hi = stats(vals)
    print(f"\n  Mean: {mean:.4f}  Std: {std:.4f}  Range: {hi - lo:.4f}")


def save_best():
    """Save the best result (prefer opto > tof > ultrasonic > ruler)."""
    if not results:
        print("No results to save.")
        return

    priority = ["opto", "tof", "ultrasonic", "ruler"]
    for method in priority:
        if method in results:
            mpt = results[method]
            save_cal({
                "distance": {
                    "mm_per_tick": round(mpt, 4),
                    "method": method,
                    "trials": len(results),
                }
            })
            return


def main():
    """Interactive calibration menu."""
    banner("Distance Calibration")
    print("Choose calibration method(s):")
    print("  A — Ruler (simplest, always works)")
    print("  B — Ultrasonic wall (no ruler needed)")
    print("  C — Opto track (most accurate)")
    print("  D — ToF sensor (optional, mm precision)")
    print("  Q — Done / show results")
    print()

    while True:
        choice = input("Method [A/B/C/D/Q]: ").strip().upper()
        if choice == "A":
            method_ruler()
        elif choice == "B":
            method_ultrasonic()
        elif choice == "C":
            method_opto()
        elif choice == "D":
            method_tof()
        elif choice == "Q":
            break
        else:
            print("Unknown choice. Enter A, B, C, D, or Q.")

    show_comparison()
    save_best()
    print("\nDone! Use calibration.json in your scripts.")


try:
    main()
finally:
    robot.motors.stop()
