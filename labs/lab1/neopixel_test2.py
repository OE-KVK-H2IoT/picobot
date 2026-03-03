# Method 2: Try updated picobot LEDStrip
import time
colors = [(50,0,0), (50,25,0), (50,50,0), (0,50,0),
                (0,50,50), (0,0,50), (25,0,50), (50,0,25)]

print("\nTest 2: picobot LEDStrip (PIO-based)...")
try:
    from picobot import LEDStrip

    strip = LEDStrip(num_leds=8, pin=6, brightness=50)

    # Rainbow
    for i, c in enumerate(colors):
        strip.set_pixel(i, c[0], c[1], c[2])
    strip.show()
    print("  PIO driver: OK!")
    time.sleep(2)

    # Blink test
    for _ in range(5):
        strip.fill(250, 0, 0)  # Red
        strip.show()
        time.sleep(0.3)
        strip.fill(0, 0, 0)   # Off
        strip.show()
        time.sleep(0.3)
    print("  Blink test: OK!")

except Exception as e:
    print(f"  Failed: {e}")

print("\nDone!")
