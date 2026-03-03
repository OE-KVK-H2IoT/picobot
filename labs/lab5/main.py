"""
==============================================================================
LAB 5: Dead Reckoning - Position and Heading Estimation
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Understand dead reckoning (position estimation from motion)
  2. Integrate gyroscope data to track heading
  3. Understand sensor drift and its causes
  4. Observe how errors accumulate over time

HARDWARE:
---------
  - MPU6050 IMU (I2C: SCL=GPIO15, SDA=GPIO14)
  - Motors for movement testing

THEORY - DEAD RECKONING:
------------------------
  Dead reckoning estimates position by integrating motion measurements:

    Position = Starting_Position + Sum(Velocity × Time)
    Heading = Starting_Heading + Sum(Angular_Velocity × Time)

  For our robot:
    1. Gyroscope Z-axis → Angular velocity (°/s) → Integrate → Heading (°)
    2. Accelerometer X-axis → Acceleration → Integrate → Velocity → Position

  THE DRIFT PROBLEM:
  ------------------
  Dead reckoning always drifts because:
    - Sensor noise gets integrated (random walk)
    - Sensor bias causes linear drift
    - Small errors compound over time

  Example: If gyroscope has 1°/s bias:
    - After 1 second:  1° error
    - After 1 minute:  60° error
    - After 1 hour:    3600° error (10 full rotations!)

  This is why real robots use additional sensors (GPS, encoders, cameras)
  to correct drift periodically.

==============================================================================
"""

import time
import math
from machine import Pin, I2C
from mpu6050 import MPU6050
from robot_state import RobotState
from picobot import Motors, LEDStrip


# ==============================================================================
# CONFIGURATION
# ==============================================================================

I2C_SCL_PIN = 15
I2C_SDA_PIN = 14
I2C_FREQ = 400000

SAMPLE_RATE_HZ = 100      # How often to read the IMU
SAMPLE_DELAY_MS = 1000 // SAMPLE_RATE_HZ


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 5: Dead Reckoning")
    print("="*50)

    # Initialize hardware
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    sensor = MPU6050(i2c)
    motors = Motors()
    leds = LEDStrip(num_leds=4, brightness=30)

    print("Hardware initialized")

    # Initialize robot state tracker
    state = RobotState()

    # -------------------------------------------------------------------------
    # CALIBRATION (Critical for accuracy!)
    # -------------------------------------------------------------------------
    print("\n*** KEEP ROBOT STILL FOR CALIBRATION ***")
    leds.fill(255, 255, 0)  # Yellow = calibrating
    leds.show()

    time.sleep(1)
    state.calibrate(sensor, num_samples=100, delay_ms=10)

    leds.fill(0, 255, 0)  # Green = ready
    leds.show()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 1: Heading tracking (gyroscope integration)
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Heading Tracking ---")
    print("Rotate the robot by hand and watch heading change")
    print("Try to return to starting orientation (0°)")
    print("(Ctrl+C to continue)\n")

    try:
        while True:
            # Read sensors and update state
            accel = sensor.get_accel()
            gyro = sensor.get_gyro()
            timestamp = time.ticks_us()

            motion = state.update(accel, gyro, timestamp)

            # Display heading with visual indicator
            heading = motion['heading_deg']

            # Create simple compass visualization
            compass = ["-"] * 36  # 10-degree increments
            idx = int((heading + 180) / 10) % 36
            compass[idx] = "*"

            print(f"  Heading: {heading:+7.1f}°  |{''.join(compass)}|  AngVel: {motion['angular_velocity_dps']:+6.1f}°/s")

            time.sleep_ms(SAMPLE_DELAY_MS)

    except KeyboardInterrupt:
        pass

    # -------------------------------------------------------------------------
    # DEMO 2: Observe gyroscope drift
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Drift Observation ---")
    print("Leave robot COMPLETELY STILL for 30 seconds")
    print("Watch how heading drifts even without movement!\n")

    state.reset()  # Reset heading to 0
    start_time = time.ticks_ms()

    leds.fill(0, 0, 255)  # Blue = measuring
    leds.show()

    drift_data = []

    for i in range(300):  # 30 seconds at 100ms intervals
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()

        motion = state.update(accel, gyro, timestamp)

        elapsed_s = (time.ticks_ms() - start_time) / 1000.0
        drift_data.append((elapsed_s, motion['heading_deg']))

        if i % 10 == 0:  # Print every second
            print(f"  t={elapsed_s:5.1f}s  Heading: {motion['heading_deg']:+.2f}°")

        time.sleep_ms(100)

    # Calculate drift rate
    total_drift = drift_data[-1][1] - drift_data[0][1]
    drift_rate = total_drift / 30.0

    print(f"\n  Total drift in 30s: {total_drift:+.2f}°")
    print(f"  Drift rate: {drift_rate:+.3f} °/s")
    print(f"  Projected drift in 1 min: {drift_rate * 60:+.1f}°")

    leds.clear()

    # -------------------------------------------------------------------------
    # DEMO 3: Position tracking with motors
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: Position Tracking ---")
    print("Robot will drive in a pattern, tracking position")
    print("Place robot on ground. Press Enter to start...")
    input()

    state.reset()

    leds.fill(0, 255, 0)
    leds.show()

    print("\nDriving forward 1 second...")
    motors.forward(100)

    # Track position while moving
    for _ in range(100):  # 1 second
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()
        state.update(accel, gyro, timestamp)
        time.sleep_ms(10)

    motors.stop()

    print(f"  Position: ({state.position_x:.3f}, {state.position_y:.3f}) m")
    print(f"  Distance: {state.distance_m:.3f} m")
    print(f"  Heading: {state.heading_deg:.1f}°")

    time.sleep(0.5)

    print("\nTurning right 90 degrees...")
    motors.turn_right(80)

    start_heading = state.heading_deg
    while state.heading_deg > start_heading - 90:
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()
        state.update(accel, gyro, timestamp)
        time.sleep_ms(10)

    motors.stop()

    print(f"  Position: ({state.position_x:.3f}, {state.position_y:.3f}) m")
    print(f"  Heading: {state.heading_deg:.1f}°")

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Calibration quality
    # --------------------------------
    # Modify the calibration to use more samples (500 instead of 100).
    # Compare the drift rate with the original calibration.
    # Does more samples = less drift?

    # EXERCISE 2: Return to start
    # ---------------------------
    # Program the robot to:
    # 1. Drive forward for 2 seconds
    # 2. Turn 180 degrees (using heading tracking)
    # 3. Drive forward for 2 seconds
    # How close does it get to the starting position?

    # EXERCISE 3: Drive a measured square
    # ------------------------------------
    # Drive a 50cm x 50cm square using heading for turns.
    # At end, check position_x and position_y.
    # The robot should end up at approximately (0, 0).

    # EXERCISE 4: Drift compensation
    # ------------------------------
    # Measure drift rate while stationary.
    # Then subtract (drift_rate * elapsed_time) from heading readings.
    # Does this improve accuracy?

    # =========================================================================

    print("\n--- Lab 5 Complete ---")
    motors.stop()
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
    finally:
        Motors().stop()
