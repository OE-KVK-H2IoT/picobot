from picobot import Robot
import time

robot = Robot(encoders=True)

robot.encoders.right.reset()
print("RIGHT wheel — watch the mark, Ctrl+C after exactly 1 revolution")

try:
    robot.set_motors(0, 160)
    while True:
        print(f"Ticks: {robot.encoders.right.ticks}    ", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    robot.stop()
    print(f"\nRIGHT PPR: {robot.encoders.right.ticks}")
