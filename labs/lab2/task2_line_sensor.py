from machine import Pin
import time

# Sensor X1 is on GP2
sensor_x1 = Pin(2, Pin.IN)

print("Hold robot over different surfaces")
print("0 = reflective (white), 1 = non-reflective (black)")
print()

while True:
    val = sensor_x1.value()
    indicator = "██ BLACK" if val else "░░ white"
    print(f"X1 (GP2): {val}  {indicator}")
    time.sleep(0.2)
