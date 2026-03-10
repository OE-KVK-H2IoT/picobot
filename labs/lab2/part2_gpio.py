from machine import Pin
import time

# Configure GP9 as input — no pull resistor
sensor = Pin(9, Pin.IN)

while True:
    print(f"GP9: {sensor.value()}")
    time.sleep(0.2)
