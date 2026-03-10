from machine import ADC, Pin
import time

# Light Sensor 2 on GP26 (ADC0)
light = ADC(Pin(26))

while True:
    raw = light.read_u16()
    print(f"Light: {raw}")
    time.sleep(0.2)
