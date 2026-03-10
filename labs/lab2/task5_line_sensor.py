from picobot import Robot
import time

robot = Robot()

while True:
    values = robot.sensors.line.read_raw()
    pattern = robot.sensors.line.get_pattern()

    print(f"Sensors: {values}  |{pattern}|")
    time.sleep(0.1)
