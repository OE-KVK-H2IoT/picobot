"""
==============================================================================
LAB 6: IMU-Based Motion Control
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Use gyroscope feedback for precise turns
  2. Implement closed-loop heading control
  3. Introduction to PID-like control concepts
  4. Combine IMU with motor control

HARDWARE:
---------
  - MPU6050 IMU (I2C: SCL=GPIO15, SDA=GPIO14)
  - Motors for movement

THEORY - FEEDBACK CONTROL:
--------------------------
  In Lab 2, we used "open-loop" motor control:
    - Turn right for 0.5 seconds
    - Hope it's approximately 90 degrees!

  Problems with open-loop:
    - Motor speed varies with battery voltage
    - Different surfaces = different friction
    - Inconsistent results

  With IMU feedback, we use "closed-loop" control:
    1. Measure current heading (from gyroscope)
    2. Compare to target heading
    3. Adjust motors based on error
    4. Repeat until error is small enough

  This is the foundation of PID control (Proportional-Integral-Derivative).

PROPORTIONAL CONTROL:
---------------------
  The simplest feedback control:

    motor_speed = K_p × error

  Where:
    - K_p is the "proportional gain" (how aggressively to correct)
    - error = target_value - current_value

  If error is large → correct strongly
  If error is small → correct gently

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

# Control parameters
TURN_KP = 2.0           # Proportional gain for turning
TURN_MIN_SPEED = 50     # Minimum motor speed (below this, motors don't move)
TURN_MAX_SPEED = 150    # Maximum motor speed
HEADING_TOLERANCE = 2.0 # Acceptable error in degrees


# ==============================================================================
# CONTROL FUNCTIONS
# ==============================================================================

def turn_to_heading(motors, state, sensor, target_heading, timeout_s=5.0):
    """
    Turn robot to a specific heading using IMU feedback.

    This is a simple proportional (P) controller:
      speed = Kp × error

    Parameters:
        motors:         Motors instance
        state:          RobotState instance (calibrated)
        sensor:         MPU6050 sensor instance
        target_heading: Target heading in degrees
        timeout_s:      Maximum time to attempt turn

    Returns:
        True if reached target, False if timed out
    """
    print(f"  Turning to {target_heading:.1f}°...")

    start_time = time.ticks_ms()

    while True:
        # Check timeout
        elapsed = (time.ticks_ms() - start_time) / 1000.0
        if elapsed > timeout_s:
            print(f"    Timeout! Current: {state.heading_deg:.1f}°")
            motors.stop()
            return False

        # Update state from IMU
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()
        state.update(accel, gyro, timestamp)

        # Calculate error (how far from target?)
        error = target_heading - state.heading_deg

        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        # Check if we've reached the target
        if abs(error) < HEADING_TOLERANCE:
            motors.stop()
            print(f"    Reached! Final: {state.heading_deg:.1f}° (error: {error:.1f}°)")
            return True

        # Proportional control: speed = Kp × error
        speed = TURN_KP * abs(error)

        # Clamp speed to valid range
        speed = max(TURN_MIN_SPEED, min(TURN_MAX_SPEED, speed))

        # Turn in correct direction
        if error > 0:
            # Need to turn left (positive heading)
            motors.set_speeds(-speed, speed)
        else:
            # Need to turn right (negative heading)
            motors.set_speeds(speed, -speed)

        time.sleep_ms(10)


def drive_straight(motors, state, sensor, distance_m, speed=100, timeout_s=10.0):
    """
    Drive straight for a specified distance, using gyroscope to maintain heading.

    Uses proportional control to correct drift while driving.

    Parameters:
        motors:     Motors instance
        state:      RobotState instance
        sensor:     MPU6050 sensor instance
        distance_m: Distance to drive in meters
        speed:      Base motor speed
        timeout_s:  Maximum time

    Returns:
        True if completed, False if timed out
    """
    print(f"  Driving {distance_m:.2f}m at speed {speed}...")

    # Remember starting heading - we want to maintain this
    target_heading = state.heading_deg
    start_distance = state.distance_m

    start_time = time.ticks_ms()

    while True:
        # Check timeout
        elapsed = (time.ticks_ms() - start_time) / 1000.0
        if elapsed > timeout_s:
            motors.stop()
            print(f"    Timeout!")
            return False

        # Update state
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        timestamp = time.ticks_us()
        state.update(accel, gyro, timestamp)

        # Check if we've driven far enough
        traveled = state.distance_m - start_distance
        if traveled >= distance_m:
            motors.stop()
            print(f"    Done! Traveled: {traveled:.3f}m")
            return True

        # Calculate heading error
        heading_error = target_heading - state.heading_deg

        # Normalize to [-180, 180]
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360

        # Adjust motor speeds to correct heading drift
        # If drifting left (positive error), slow left motor
        # If drifting right (negative error), slow right motor
        correction = TURN_KP * heading_error * 0.5  # Gentler correction while driving

        left_speed = speed - correction
        right_speed = speed + correction

        # Clamp speeds
        left_speed = max(0, min(255, left_speed))
        right_speed = max(0, min(255, right_speed))

        motors.set_speeds(int(left_speed), int(right_speed))

        time.sleep_ms(10)


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 6: IMU-Based Motion Control")
    print("="*50)

    # Initialize hardware
    i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    sensor = MPU6050(i2c)
    motors = Motors()
    leds = LEDStrip(num_leds=4, brightness=30)

    state = RobotState()

    # Calibration
    print("\n*** KEEP ROBOT STILL FOR CALIBRATION ***")
    leds.fill(255, 255, 0)
    leds.show()
    time.sleep(1)

    state.calibrate(sensor, num_samples=100, delay_ms=10)

    leds.fill(0, 255, 0)
    leds.show()

    # -------------------------------------------------------------------------
    # DEMO 1: Precise turns
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Precise Turns ---")
    print("Place robot on ground. Press Enter to start...")
    input()

    print("\nTurning +90° (left)...")
    turn_to_heading(motors, state, sensor, 90.0)
    time.sleep(1)

    print("\nTurning +180°...")
    turn_to_heading(motors, state, sensor, 180.0)
    time.sleep(1)

    print("\nTurning -90° (right from 180 = facing backwards-right)...")
    turn_to_heading(motors, state, sensor, -90.0)
    time.sleep(1)

    print("\nReturning to 0°...")
    turn_to_heading(motors, state, sensor, 0.0)
    time.sleep(1)

    # -------------------------------------------------------------------------
    # DEMO 2: Drive a square using IMU
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Precise Square ---")
    print("Robot will drive a square using IMU feedback")
    print("Press Enter to start...")
    input()

    state.reset()
    leds.fill(0, 0, 255)
    leds.show()

    print("\nDriving square pattern:")

    for i in range(4):
        print(f"\n  Side {i+1}/4:")

        # Drive forward (using time-based for now, or distance if odometry works)
        motors.forward(100)
        time.sleep(0.8)
        motors.stop()
        time.sleep(0.3)

        # Turn exactly 90 degrees right
        target = state.heading_deg - 90
        turn_to_heading(motors, state, sensor, target)
        time.sleep(0.3)

    print(f"\nFinal position: ({state.position_x:.3f}, {state.position_y:.3f}) m")
    print(f"Final heading: {state.heading_deg:.1f}° (should be ~0° or ~-360°)")

    motors.stop()

    # -------------------------------------------------------------------------
    # DEMO 3: Tuning the P-gain
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: P-Gain Demonstration ---")
    print("Watch how different gains affect turn behavior")
    print("Press Enter for each test...\n")

    for kp in [0.5, 1.0, 2.0, 4.0]:
        input(f"Test with Kp = {kp} (press Enter)...")

        state.reset()

        # Temporarily change gain
        global TURN_KP
        old_kp = TURN_KP
        TURN_KP = kp

        turn_to_heading(motors, state, sensor, 90.0)
        print(f"  Final error: {90.0 - state.heading_deg:.1f}°\n")

        TURN_KP = old_kp  # Restore
        time.sleep(1)

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Turn to any angle
    # -----------------------------
    # Modify the code to accept user input for target angle.
    # Let users type an angle and watch the robot turn to it.

    # EXERCISE 2: Multi-point navigation
    # -----------------------------------
    # Program a sequence of headings: 45°, 135°, 225°, 315°, 0°
    # This makes the robot point to each corner of a square.

    # EXERCISE 3: PID controller
    # --------------------------
    # Add integral (I) and derivative (D) terms:
    #   I: Accumulate error over time (helps eliminate steady-state error)
    #   D: React to rate of change (reduces overshoot)
    #
    # speed = Kp×error + Ki×integral(error) + Kd×d(error)/dt

    # EXERCISE 4: Obstacle avoidance with heading recovery
    # ----------------------------------------------------
    # When obstacle detected:
    #   1. Remember current heading
    #   2. Back up
    #   3. Turn to avoid
    #   4. Return to original heading
    #   5. Continue forward

    # =========================================================================

    print("\n--- Lab 6 Complete ---")
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
