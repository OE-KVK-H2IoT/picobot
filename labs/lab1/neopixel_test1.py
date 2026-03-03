from machine import Pin
import time

# Method 1: Try built-in neopixel with custom timing
print("Test 1: Built-in neopixel with custom timing...")
try:
    from neopixel import NeoPixel
    pin = Pin(6)
    leds = NeoPixel(pin, 8)

    # Set rainbow colors
    colors = [(50,0,0), (50,25,0), (50,50,0), (0,50,0),
            (0,50,50), (0,0,50), (25,0,50), (50,0,25)]
    for i, c in enumerate(colors):
        leds[i] = c

    # Try custom timing (works on Pico 2 W)
    pin.bitstream(leds.buf, timing=(350, 900, 800, 450))
    print("  Custom timing: OK!")
    time.sleep(2)
except Exception as e:
    print(f"  Failed: {e}")
