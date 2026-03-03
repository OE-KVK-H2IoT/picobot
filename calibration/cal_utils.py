"""
Calibration utilities — shared helpers for all calibration scripts.

Provides JSON load/save, basic statistics, sensor helpers,
and formatted output for the calibration workflow.
"""

import json
import time


def load_cal(path="calibration.json"):
    """Load calibration data from JSON file.

    Returns empty dict if file doesn't exist or is corrupt.
    """
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_cal(data, path="calibration.json"):
    """Merge data into existing calibration file and save.

    Loads existing file, updates with new keys, writes back.
    Prints a summary of what was saved.
    """
    cal = load_cal(path)
    cal.update(data)
    cal["timestamp"] = _timestamp()
    with open(path, "w") as f:
        json.dump(cal, f)
    print(f"\nSaved to {path}:")
    for key in data:
        print(f"  {key}: {data[key]}")


def stats(values):
    """Compute (mean, std, min, max) from a list of numbers.

    Pure Python — no numpy required.
    """
    n = len(values)
    mean = sum(values) / n
    variance = sum((x - mean) ** 2 for x in values) / n
    std = variance ** 0.5
    return (mean, std, min(values), max(values))


def median5(sensor_fn):
    """Take 5 readings from sensor_fn(), return the median.

    Useful for filtering ultrasonic noise.
    """
    readings = sorted(sensor_fn() for _ in range(5))
    return readings[2]


def try_tof(i2c):
    """Try to initialize VL53L1X ToF sensor.

    Returns sensor instance or None if not available.
    """
    try:
        from vl53l1x import VL53L1X
        tof = VL53L1X(i2c)
        tof.init()
        tof.set_timing_budget_ms(50)
        print("VL53L1X ToF sensor detected!")
        return tof
    except Exception:
        return None


def banner(title):
    """Print a formatted section banner."""
    line = "=" * 40
    print(f"\n{line}")
    print(f"  {title}")
    print(f"{line}\n")


def result(label, value, unit=""):
    """Print a formatted result line."""
    if isinstance(value, float):
        print(f"  {label}: {value:.3f} {unit}")
    else:
        print(f"  {label}: {value} {unit}")


def _timestamp():
    """Return ISO-style timestamp string from RTC."""
    t = time.localtime()
    return f"{t[0]:04d}-{t[1]:02d}-{t[2]:02d}T{t[3]:02d}:{t[4]:02d}:{t[5]:02d}"
