"""
==============================================================================
EXAMPLE: IMU-Controlled Square
==============================================================================

Drives a precise square using IMU feedback for turns.
Demonstrates closed-loop heading control.

Deploy this along with libs/ to run standalone.
==============================================================================
"""

import time
from machine import Pin, I2C
from picobot import Motors, LEDStrip
from mpu6050 import MPU6050
from robot_state import RobotState


# Configuration
SIDE_TIME_S = 1.0       # How long to drive each side
DRIVE_SPEED = 100
TURN_KP = 2.0           # Proportional gain for turning
TURN_TOLERANCE = 2.0    # Degrees


def turn_to_heading(motors, state, sensor, target):
    """Turn to specific heading using P-control."""
    while True:
        # Update state
        accel = sensor.get_accel()
        gyro = sensor.get_gyro()
        state.update(accel, gyro, time.ticks_us())

        error = target - state.heading_deg

        # Normalize error
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        if abs(error) < TURN_TOLERANCE:
            motors.stop()
            return

        speed = min(150, max(50, TURN_KP * abs(error)))

        if error > 0:
            motors.set_speeds(-speed, speed)
        else:
            motors.set_speeds(speed, -speed)

        time.sleep_ms(10)


def main():
    print("IMU-Controlled Square")
    print("=====================")

    # Initialize
    motors = Motors()
    leds = LEDStrip(num_leds=4, brightness=30)

    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
    sensor = MPU6050(i2c)
    state = RobotState()

    # Calibrate
    print("Calibrating IMU (keep still)...")
    leds.fill(255, 255, 0)
    leds.show()
    time.sleep(1)
    state.calibrate(sensor, num_samples=100, delay_ms=10)

    print("Place on ground. Press Enter...")
    input()

    leds.fill(0, 255, 0)
    leds.show()

    print("Driving square...")

    try:
        for i in range(4):
            print(f"  Side {i+1}/4")

            # Drive forward
            leds.fill(0, 255, 0)
            leds.show()
            motors.forward(DRIVE_SPEED)
            time.sleep(SIDE_TIME_S)
            motors.stop()
            time.sleep(0.3)

            # Turn 90 degrees right
            leds.fill(0, 0, 255)
            leds.show()
            target = state.heading_deg - 90
            turn_to_heading(motors, state, sensor, target)
            time.sleep(0.3)

        print(f"\nFinal heading: {state.heading_deg:.1f}° (should be ~-360° or 0°)")
        print("Square complete!")

    finally:
        motors.stop()
        leds.clear()


if __name__ == "__main__":
    main()
