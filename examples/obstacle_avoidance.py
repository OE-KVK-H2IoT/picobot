"""
==============================================================================
EXAMPLE: Obstacle Avoidance
==============================================================================

A complete obstacle avoidance robot that:
  - Drives forward when path is clear
  - Backs up and turns when obstacle detected
  - Uses LED feedback to show state

Deploy this along with libs/ to run standalone.
==============================================================================
"""

import time
from picobot import PicoBot


# Configuration
DANGER_DISTANCE = 15
WARNING_DISTANCE = 30
DRIVE_SPEED = 120
TURN_SPEED = 100


def main():
    print("Obstacle Avoidance Robot")
    print("========================")

    robot = PicoBot(num_leds=4)

    print("Place on ground. Starting in 3s...")
    for i in range(3, 0, -1):
        robot.leds.fill(255, 255, 0)
        robot.leds.show()
        time.sleep(0.5)
        robot.leds.clear()
        time.sleep(0.5)

    print("Running! (Ctrl+C to stop)")

    try:
        while True:
            distance = robot.ultrasonic.distance_cm_filtered(3)

            if distance < 0:
                # No reading - stop
                robot.leds.fill(255, 255, 255)  # White = error
                robot.leds.show()
                robot.motors.stop()

            elif distance < DANGER_DISTANCE:
                # Too close! Back up and turn
                robot.leds.fill(255, 0, 0)  # Red
                robot.leds.show()
                robot.motors.backward(DRIVE_SPEED)
                time.sleep(0.4)
                robot.motors.turn_right(TURN_SPEED)
                time.sleep(0.4)

            elif distance < WARNING_DISTANCE:
                # Getting close - slow turn
                robot.leds.fill(255, 165, 0)  # Orange
                robot.leds.show()
                robot.motors.curve_right(DRIVE_SPEED, 0.3)

            else:
                # Clear - drive forward
                robot.leds.fill(0, 255, 0)  # Green
                robot.leds.show()
                robot.motors.forward(DRIVE_SPEED)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        robot.motors.stop()
        robot.leds.clear()


if __name__ == "__main__":
    main()
