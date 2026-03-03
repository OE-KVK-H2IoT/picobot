"""
==============================================================================
IMU COMPARISON LAB - Compare MPU6050 vs BMI160
==============================================================================

This lab compares the two IMU sensors:
- MPU6050: Older, more common, noisier
- BMI160:  Newer, lower noise, lower power

Run this on your Pico to see the difference in noise levels.

WIRING (same for both sensors):
    IMU     Pico
    ---     ----
    VCC ->  3.3V
    GND ->  GND
    SCL ->  GP5
    SDA ->  GP4

==============================================================================
"""

import time
from machine import I2C, Pin
import math

# I2C setup
I2C_SDA = 4
I2C_SCL = 5

print("\n" + "=" * 60)
print("IMU COMPARISON LAB")
print("=" * 60)

# Initialize I2C
i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)

# Scan for devices
devices = i2c.scan()
print(f"\nI2C devices found: {[hex(d) for d in devices]}")

# Try to detect which IMU is connected
imu = None
imu_type = "Unknown"

if 0x68 in devices:
    # Check chip ID to distinguish MPU6050 from BMI160
    try:
        chip_id = i2c.readfrom_mem(0x68, 0x00, 1)[0]
        print(f"Chip ID at 0x00: 0x{chip_id:02X}")

        if chip_id == 0xD1:
            # BMI160
            from bmi160 import BMI160
            imu = BMI160(i2c, dlpf=2)
            imu_type = "BMI160"
            print("Detected: BMI160")
        else:
            # MPU6050 (WHO_AM_I at 0x75 returns 0x68)
            who_am_i = i2c.readfrom_mem(0x68, 0x75, 1)[0]
            print(f"WHO_AM_I at 0x75: 0x{who_am_i:02X}")
            if who_am_i == 0x68:
                from mpu6050 import MPU6050
                imu = MPU6050(i2c, dlpf=2)
                imu_type = "MPU6050"
                print("Detected: MPU6050")
    except Exception as e:
        print(f"Error detecting IMU: {e}")

if imu is None:
    print("ERROR: No compatible IMU found!")
    print("Check wiring and try again.")
    raise SystemExit

print(f"\nUsing: {imu_type}")
print("=" * 60)


# ==============================================================================
# NOISE ANALYSIS
# ==============================================================================

def analyze_noise(imu, num_samples=500, delay_ms=2):
    """
    Analyze sensor noise by collecting samples while stationary.

    Returns:
        Dict with noise statistics
    """
    print(f"\nCollecting {num_samples} samples (keep sensor still)...")

    ax_samples, ay_samples, az_samples = [], [], []
    gx_samples, gy_samples, gz_samples = [], [], []

    for i in range(num_samples):
        accel = imu.get_accel()
        gyro = imu.get_gyro()

        ax_samples.append(accel['x'])
        ay_samples.append(accel['y'])
        az_samples.append(accel['z'])
        gx_samples.append(gyro['x'])
        gy_samples.append(gyro['y'])
        gz_samples.append(gyro['z'])

        if i % 100 == 0:
            print(f"  {i}/{num_samples}...")

        time.sleep_ms(delay_ms)

    # Calculate statistics
    def stats(data):
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        std = math.sqrt(variance)
        min_v = min(data)
        max_v = max(data)
        return {'mean': mean, 'std': std, 'min': min_v, 'max': max_v, 'range': max_v - min_v}

    return {
        'accel': {
            'x': stats(ax_samples),
            'y': stats(ay_samples),
            'z': stats(az_samples),
        },
        'gyro': {
            'x': stats(gx_samples),
            'y': stats(gy_samples),
            'z': stats(gz_samples),
        }
    }


def print_noise_report(results, imu_type):
    """Print noise analysis report."""
    print("\n" + "=" * 60)
    print(f"NOISE ANALYSIS: {imu_type}")
    print("=" * 60)

    print("\nAccelerometer (units: g)")
    print("-" * 50)
    print(f"{'Axis':<6} {'Mean':>10} {'Std Dev':>10} {'Range':>10}")
    print("-" * 50)
    for axis in ['x', 'y', 'z']:
        s = results['accel'][axis]
        print(f"{axis.upper():<6} {s['mean']:>10.4f} {s['std']:>10.4f} {s['range']:>10.4f}")

    # Calculate total accel noise (RMS of std devs)
    accel_noise = math.sqrt(sum(results['accel'][a]['std']**2 for a in 'xyz'))
    print(f"\nTotal accel noise (RMS): {accel_noise:.4f} g = {accel_noise*1000:.2f} mg")

    print("\nGyroscope (units: °/s)")
    print("-" * 50)
    print(f"{'Axis':<6} {'Mean':>10} {'Std Dev':>10} {'Range':>10}")
    print("-" * 50)
    for axis in ['x', 'y', 'z']:
        s = results['gyro'][axis]
        print(f"{axis.upper():<6} {s['mean']:>10.4f} {s['std']:>10.4f} {s['range']:>10.4f}")

    # Calculate total gyro noise
    gyro_noise = math.sqrt(sum(results['gyro'][a]['std']**2 for a in 'xyz'))
    print(f"\nTotal gyro noise (RMS): {gyro_noise:.4f} °/s")

    # Gyro bias (mean when stationary should be ~0)
    gyro_bias = math.sqrt(sum(results['gyro'][a]['mean']**2 for a in 'xyz'))
    print(f"Gyro bias magnitude: {gyro_bias:.4f} °/s")

    return accel_noise, gyro_noise


# ==============================================================================
# MAIN
# ==============================================================================

print("\n" + "=" * 60)
print("STEP 1: Keep sensor completely still for noise measurement")
print("=" * 60)
time.sleep(2)

# Analyze noise
results = analyze_noise(imu, num_samples=500, delay_ms=2)
accel_noise, gyro_noise = print_noise_report(results, imu_type)

# Expected values for comparison
print("\n" + "=" * 60)
print("COMPARISON WITH DATASHEET SPECS")
print("=" * 60)
print(f"\n{'Sensor':<12} {'Accel Noise':<15} {'Gyro Noise':<15}")
print("-" * 45)
print(f"{'MPU6050':<12} {'~4 mg RMS':<15} {'~0.05 °/s RMS':<15}")
print(f"{'BMI160':<12} {'~1.8 mg RMS':<15} {'~0.07 °/s RMS':<15}")
print(f"{imu_type:<12} {f'{accel_noise*1000:.2f} mg':<15} {f'{gyro_noise:.3f} °/s':<15}")

if imu_type == "BMI160":
    improvement = 4.0 / (accel_noise * 1000)
    print(f"\nBMI160 accel noise is ~{improvement:.1f}x better than MPU6050 spec")

# ==============================================================================
# LIVE DEMO
# ==============================================================================

print("\n" + "=" * 60)
print("STEP 2: Live sensor readings (move the sensor)")
print("=" * 60)
print("Press Ctrl+C to stop\n")

try:
    while True:
        accel = imu.get_accel()
        gyro = imu.get_gyro()
        temp = imu.get_temp()

        # Calculate magnitude
        accel_mag = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
        gyro_mag = math.sqrt(gyro['x']**2 + gyro['y']**2 + gyro['z']**2)

        print(f"\rA: X={accel['x']:+.3f} Y={accel['y']:+.3f} Z={accel['z']:+.3f} |{accel_mag:.2f}|g  "
              f"G: X={gyro['x']:+6.1f} Y={gyro['y']:+6.1f} Z={gyro['z']:+6.1f} |{gyro_mag:5.1f}|°/s  "
              f"T:{temp:.1f}°C  ", end='')

        time.sleep_ms(50)

except KeyboardInterrupt:
    print("\n\nStopped.")

print("\n" + "=" * 60)
print("LAB COMPLETE")
print("=" * 60)
print(f"IMU tested: {imu_type}")
print(f"Measured accel noise: {accel_noise*1000:.2f} mg")
print(f"Measured gyro noise: {gyro_noise:.3f} °/s")
