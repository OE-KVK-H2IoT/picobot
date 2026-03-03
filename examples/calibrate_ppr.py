"""
Calibrate encoder PPR (pulses per wheel revolution).

Drives straight for 2 seconds, counts ticks.
Measure distance with ruler, then compute PPR.
"""
from picobot.motors import Motors
from picobot.encoders import Encoders
import time

motors = Motors()
encoders = Encoders()

SPEED = 160

import sys
print("Place robot at start mark. Starting in 3 seconds...")
time.sleep(3)

encoders.reset()

motors.set_speed(SPEED, SPEED)
time.sleep(2)
motors.stop()

left_ticks = encoders.left.ticks
right_ticks = encoders.right.ticks if encoders.right else 0

print(f"\nLeft ticks:  {left_ticks}")
print(f"Right ticks: {right_ticks}")
print()
print("Now measure:")
print("  1. Distance traveled (mm): ___")
print("  2. Wheel circumference (mm): ___  (or diameter x 3.14)")
print()
print("Then calculate:")
print("  revolutions = distance / circumference")
print("  PPR = ticks / revolutions")
