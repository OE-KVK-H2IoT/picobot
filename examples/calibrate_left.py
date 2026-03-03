from picobot import Robot
import time

robot = Robot(encoders=True)

robot.encoders.left.reset()
print("LEFT wheel — watch the mark, Ctrl+C after exactly 1 revolution")

try:
    robot.set_motors(160, 0)
    while True:
        print(f"Ticks: {robot.encoders.left.ticks}    ", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    robot.stop()
    print(f"\nLEFT PPR: {robot.encoders.left.ticks}")
