"""
==============================================================================
LAB 2: Motor Control and Differential Drive
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Understand differential drive robot kinematics
  2. Control motor speed with PWM
  3. Implement basic movements (forward, backward, turn, curve)

HARDWARE:
---------
  - 2 DC motors with differential drive
  - Left motor:  GPIO 12 (forward), GPIO 13 (backward)
  - Right motor: GPIO 10 (forward), GPIO 11 (backward)

THEORY - DIFFERENTIAL DRIVE:
----------------------------
  A differential drive robot has two independently controlled wheels.
  Movement is achieved by varying the speed of each wheel:

    Left Wheel | Right Wheel | Result
    -----------|-------------|------------------
    Forward    | Forward     | Move forward
    Backward   | Backward    | Move backward
    Backward   | Forward     | Spin left (pivot)
    Forward    | Backward    | Spin right (pivot)
    Fast       | Slow        | Curve right
    Slow       | Fast        | Curve left

THEORY - PWM:
-------------
  PWM (Pulse Width Modulation) controls motor speed by rapidly switching
  the motor on/off. The "duty cycle" (% of time ON) determines speed:
    - 0% duty cycle   = motor stopped
    - 50% duty cycle  = half speed
    - 100% duty cycle = full speed

  We use 0-255 scale for speed (0 = stop, 255 = full speed)

==============================================================================
"""

import time
from picobot import Motors, LEDStrip


# ==============================================================================
# CONFIGURATION
# ==============================================================================

DEFAULT_SPEED = 120   # Default motor speed (0-255)
TURN_SPEED = 100      # Speed for turning
MOVE_TIME = 1.0       # Seconds to move for each demo


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 2: Motor Control")
    print("="*50)

    # Initialize hardware
    motors = Motors()
    leds = LEDStrip(num_leds=4, brightness=30)

    print("Motors initialized")
    print(f"Default speed: {DEFAULT_SPEED}")
    print("\n*** PLACE ROBOT ON THE GROUND ***")
    print("Starting in 3 seconds...")

    # Countdown with LEDs
    for i in range(3, 0, -1):
        print(f"  {i}...")
        leds.fill(255, 255, 0)  # Yellow = waiting
        leds.show()
        time.sleep(0.5)
        leds.clear()
        time.sleep(0.5)

    leds.fill(0, 255, 0)  # Green = running
    leds.show()

    # -------------------------------------------------------------------------
    # DEMO 1: Forward and backward
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Forward & Backward ---")

    print("  Forward...")
    motors.forward(DEFAULT_SPEED)
    time.sleep(MOVE_TIME)

    motors.stop()
    time.sleep(0.5)

    print("  Backward...")
    motors.backward(DEFAULT_SPEED)
    time.sleep(MOVE_TIME)

    motors.stop()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 2: Turning (spin in place)
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Turns ---")

    print("  Turn left (spin)...")
    motors.turn_left(TURN_SPEED)
    time.sleep(0.5)

    motors.stop()
    time.sleep(0.5)

    print("  Turn right (spin)...")
    motors.turn_right(TURN_SPEED)
    time.sleep(0.5)

    motors.stop()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 3: Curves (one wheel faster than other)
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: Curves ---")

    print("  Curve left...")
    motors.curve_left(DEFAULT_SPEED, ratio=0.3)
    time.sleep(MOVE_TIME)

    motors.stop()
    time.sleep(0.5)

    print("  Curve right...")
    motors.curve_right(DEFAULT_SPEED, ratio=0.3)
    time.sleep(MOVE_TIME)

    motors.stop()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 4: Direct motor control
    # -------------------------------------------------------------------------
    print("\n--- Demo 4: Direct Control (set_speeds) ---")

    print("  Left=100, Right=200 (curve right)...")
    motors.set_speeds(100, 200)
    time.sleep(MOVE_TIME)

    print("  Left=200, Right=100 (curve left)...")
    motors.set_speeds(200, 100)
    time.sleep(MOVE_TIME)

    motors.stop()

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Drive a square
    # --------------------------
    # Make the robot drive in a square pattern:
    # Forward -> Turn 90° right -> Forward -> Turn 90° right -> ...
    #
    # Hint: Experiment with turn time to get approximately 90 degrees
    #
    # Your code here:
    # for _ in range(4):
    #     motors.forward(150)
    #     time.sleep(1.0)
    #     motors.turn_right(100)
    #     time.sleep(0.5)  # Adjust this for 90° turn
    # motors.stop()

    # EXERCISE 2: Figure-8 pattern
    # ----------------------------
    # Drive in a figure-8 using curves
    # Hint: Curve left, then curve right, repeat
    #
    # Your code here:

    # EXERCISE 3: Speed ramp
    # ----------------------
    # Gradually increase speed from 0 to 200, then back to 0
    # This creates smooth acceleration/deceleration
    #
    # Your code here:
    # for speed in range(0, 200, 10):
    #     motors.forward(speed)
    #     time.sleep(0.1)
    # for speed in range(200, 0, -10):
    #     motors.forward(speed)
    #     time.sleep(0.1)
    # motors.stop()

    # EXERCISE 4: Dance routine
    # -------------------------
    # Create a fun dance routine combining movements
    # Be creative!
    #
    # Your code here:

    # =========================================================================

    print("\n--- Lab 2 Complete ---")
    motors.stop()
    leds.fill(0, 0, 255)  # Blue = done
    leds.show()


# ==============================================================================
# RUN
# ==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
        # Safety: stop motors on interrupt
        motors = Motors()
        motors.stop()
    except Exception as e:
        print(f"\nError: {e}")
        # Safety: stop motors on error
        motors = Motors()
        motors.stop()
