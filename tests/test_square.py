from picobot import Robot
import time

robot = Robot()

# Countdown — gives you time to place the robot and unplug USB


for i in range(4):
    robot.forward(speed=180, duration=1.6)
    time.sleep(0.2)
    robot.turn_right(speed=180, duration=1.35)
    time.sleep(0.2)

robot.stop()
robot.set_leds((0, 0, 255))         # Blue = done
