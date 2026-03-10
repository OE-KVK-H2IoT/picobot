from picobot import Robot
import time

robot = Robot()

SENSOR_LEDS = [1, 3, 4, 6]  # Map sensors to LED positions

while True:
    values = robot.sensors.line.read_raw()

    for i, val in enumerate(values):
        led_idx = SENSOR_LEDS[i]
        if val:  # Black detected
            robot.set_led(led_idx, (255, 0, 0))  # Red
        else:
            robot.set_led(led_idx, (0, 50, 0))   # Dim green

    time.sleep(0.05)
