from machine import Pin
import time

sensors = [
    Pin(2, Pin.IN),  # X1 - far left
    Pin(3, Pin.IN),  # X2 - left center
    Pin(4, Pin.IN),  # X3 - right center
    Pin(5, Pin.IN),  # X4 - far right
]

print("Slide robot over black tape")
print()

while True:
    values = [s.value() for s in sensors]

    pattern = ""
    for v in values:
        pattern += "██" if v else "░░"

    print(f"X1-X4: {values}  |{pattern}|")
    time.sleep(0.1)
