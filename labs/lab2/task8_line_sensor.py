from picobot import Robot
import time

robot = Robot()

count = 0
was_on_line = False

print("Slide robot across black lines — press Ctrl+C to stop")

try:
    while True:
        values = robot.sensors.line.read_raw()
        on_line = any(v == 1 for v in values)

        if on_line and not was_on_line:  # Transition: off → on
            count += 1
            print(f"Line #{count} detected!")
            robot.beep(440, 50)

        was_on_line = on_line
        time.sleep(0.02)

except KeyboardInterrupt:
    print(f"\nTotal lines crossed: {count}")
