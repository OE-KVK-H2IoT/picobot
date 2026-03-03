"""
==============================================================================
LAB 4: IMU Basics - Accelerometer and Gyroscope
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Understand what an IMU (Inertial Measurement Unit) is
  2. Read and interpret accelerometer data
  3. Read and interpret gyroscope data
  4. Understand sensor noise and bias

HARDWARE:
---------
  - MPU6050 6-axis IMU
  - I2C connection: SCL=GPIO15, SDA=GPIO14

THEORY - ACCELEROMETER:
-----------------------
  Measures acceleration in 3 axes (X, Y, Z) in units of "g" (gravity).
  1g = 9.81 m/s²

  IMPORTANT: The accelerometer always "sees" gravity!
  When the robot is stationary on a flat surface:
    - X ≈ 0g (no forward/backward tilt)
    - Y ≈ 0g (no left/right tilt)
    - Z ≈ +1g (gravity pointing "up" from sensor's perspective)

  The sensor measures the force that prevents it from falling
  (like feeling weight when standing in an elevator).

  Orientation from accelerometer:
    - Tilt forward  → X increases (positive)
    - Tilt backward → X decreases (negative)
    - Tilt left     → Y increases
    - Tilt right    → Y decreases
    - Flip upside   → Z becomes negative

THEORY - GYROSCOPE:
-------------------
  Measures angular velocity (rotation speed) in degrees per second (°/s).

  When the robot is stationary: X, Y, Z ≈ 0 °/s
  (In practice, there's always some "bias" - a small offset)

  When rotating:
    - Rotate around X axis (pitch/nod) → X value changes
    - Rotate around Y axis (roll)      → Y value changes
    - Rotate around Z axis (yaw/spin)  → Z value changes

  The gyroscope is great for measuring rotation, but:
    - It has bias (offset when stationary)
    - It drifts over time when integrated

==============================================================================
"""

import time
from machine import Pin, I2C
from mpu6050 import MPU6050
from picobot import LEDStrip


# ==============================================================================
# CONFIGURATION
# ==============================================================================

I2C_SCL_PIN = 15
I2C_SDA_PIN = 14
I2C_FREQ = 400000    # 400 kHz (fast mode)

SAMPLE_DELAY_MS = 100  # Time between readings


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 4: IMU Basics")
    print("="*50)

    # Initialize I2C and IMU
    print("\nInitializing I2C bus...")
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

    # Scan for I2C devices
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(d) for d in devices]}")

    if 0x68 not in devices:
        print("ERROR: MPU6050 not found at address 0x68!")
        print("Check wiring: SCL->GPIO15, SDA->GPIO14")
        return

    sensor = MPU6050(i2c)
    print("MPU6050 initialized!")

    # LEDs for visual feedback
    leds = LEDStrip(num_leds=4, brightness=30)

    # -------------------------------------------------------------------------
    # DEMO 1: Raw sensor reading
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Single Reading ---")
    print("Keep robot stationary and flat...\n")
    time.sleep(1)

    accel = sensor.get_accel()
    gyro = sensor.get_gyro()
    temp = sensor.get_temp()

    print("Accelerometer (g):")
    print(f"  X: {accel['x']:+.3f} g")
    print(f"  Y: {accel['y']:+.3f} g")
    print(f"  Z: {accel['z']:+.3f} g")

    print("\nGyroscope (°/s):")
    print(f"  X: {gyro['x']:+.2f} °/s")
    print(f"  Y: {gyro['y']:+.2f} °/s")
    print(f"  Z: {gyro['z']:+.2f} °/s")

    print(f"\nChip temperature: {temp:.1f} °C")

    # -------------------------------------------------------------------------
    # DEMO 2: Understanding gravity
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Gravity Vector ---")
    print("Tilt the robot and watch the accelerometer values change")
    print("(Ctrl+C to continue)\n")

    try:
        while True:
            accel = sensor.get_accel()

            # Calculate total acceleration magnitude
            # Should be ≈1.0g when stationary (gravity only)
            magnitude = (accel['x']**2 + accel['y']**2 + accel['z']**2) ** 0.5

            # Visualize tilt with LEDs
            # LED 0-1: X-axis (forward/backward tilt)
            # LED 2-3: Y-axis (left/right tilt)
            leds.clear()

            if accel['x'] > 0.2:
                leds.set_pixel(0, 255, 0, 0)    # Tilted forward = red on LED 0
            elif accel['x'] < -0.2:
                leds.set_pixel(1, 255, 0, 0)    # Tilted backward = red on LED 1

            if accel['y'] > 0.2:
                leds.set_pixel(2, 0, 255, 0)    # Tilted left = green on LED 2
            elif accel['y'] < -0.2:
                leds.set_pixel(3, 0, 255, 0)    # Tilted right = green on LED 3

            leds.show()

            print(f"  X:{accel['x']:+.2f}g  Y:{accel['y']:+.2f}g  Z:{accel['z']:+.2f}g  |Mag|:{magnitude:.2f}g")
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass

    leds.clear()

    # -------------------------------------------------------------------------
    # DEMO 3: Gyroscope rotation detection
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: Rotation Detection ---")
    print("Rotate the robot and watch the gyroscope values")
    print("(Ctrl+C to continue)\n")

    try:
        while True:
            gyro = sensor.get_gyro()

            # Color based on rotation axis
            leds.clear()

            if abs(gyro['z']) > 20:
                # Significant yaw rotation (spinning)
                if gyro['z'] > 0:
                    leds.fill(0, 0, 255)   # Blue = spinning left
                else:
                    leds.fill(255, 0, 255) # Magenta = spinning right

            leds.show()

            print(f"  X:{gyro['x']:+6.1f}°/s  Y:{gyro['y']:+6.1f}°/s  Z:{gyro['z']:+6.1f}°/s")
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    leds.clear()

    # -------------------------------------------------------------------------
    # DEMO 4: Gyroscope bias measurement
    # -------------------------------------------------------------------------
    print("\n--- Demo 4: Gyroscope Bias ---")
    print("Keep robot PERFECTLY STILL for bias measurement...")
    time.sleep(1)

    # Collect samples
    NUM_SAMPLES = 100
    gx_sum, gy_sum, gz_sum = 0, 0, 0

    print(f"Collecting {NUM_SAMPLES} samples...")
    for i in range(NUM_SAMPLES):
        gyro = sensor.get_gyro()
        gx_sum += gyro['x']
        gy_sum += gyro['y']
        gz_sum += gyro['z']
        time.sleep(0.01)

    # Calculate average (this is the bias)
    bias_x = gx_sum / NUM_SAMPLES
    bias_y = gy_sum / NUM_SAMPLES
    bias_z = gz_sum / NUM_SAMPLES

    print("\nGyroscope Bias (offset when stationary):")
    print(f"  X bias: {bias_x:+.2f} °/s")
    print(f"  Y bias: {bias_y:+.2f} °/s")
    print(f"  Z bias: {bias_z:+.2f} °/s")
    print("\nSubtract these values from readings to get true rotation!")

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Level indicator
    # ---------------------------
    # Use the accelerometer to create a "bubble level" indicator.
    # Light up LEDs to show which direction to tilt to level the robot.
    #
    # Hint: When level, X and Y should both be close to 0

    # EXERCISE 2: Shake detection
    # ---------------------------
    # Detect when the robot is being shaken.
    # Calculate total acceleration magnitude: sqrt(x² + y² + z²)
    # If magnitude is significantly different from 1.0g, it's being shaken!
    #
    # Your code here:
    # magnitude = (accel['x']**2 + accel['y']**2 + accel['z']**2) ** 0.5
    # if abs(magnitude - 1.0) > 0.3:
    #     print("SHAKE DETECTED!")

    # EXERCISE 3: Simple tilt angle
    # -----------------------------
    # Calculate the tilt angle from accelerometer data.
    # For small angles: angle ≈ arcsin(accel_x) in radians
    #
    # import math
    # tilt_rad = math.asin(max(-1, min(1, accel['x'])))
    # tilt_deg = math.degrees(tilt_rad)

    # EXERCISE 4: Gyroscope integration
    # ---------------------------------
    # Integrate gyroscope Z to track heading (cumulative rotation).
    # Note: This will drift over time!
    #
    # heading = 0
    # last_time = time.ticks_ms()
    # while True:
    #     gyro = sensor.get_gyro()
    #     now = time.ticks_ms()
    #     dt = time.ticks_diff(now, last_time) / 1000.0
    #     heading += (gyro['z'] - bias_z) * dt
    #     last_time = now
    #     print(f"Heading: {heading:.1f}°")

    # =========================================================================

    print("\n--- Lab 4 Complete ---")
    leds.clear()


# ==============================================================================
# RUN
# ==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
