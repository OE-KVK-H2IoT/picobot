from machine import ADC, Pin
import time

light = ADC(Pin(26))

# --- CALIBRATION ---
# Replace these with YOUR measured values!
min_val = 500     # Bright light (lowest reading)
max_val = 55000   # Complete darkness (highest reading)

while True:
    raw = light.read_u16()

    # Clamp to calibration range
    clamped = max(min_val, min(raw, max_val))

    # Convert: 100% = bright, 0% = dark
    percent = ((max_val - clamped) / (max_val - min_val)) * 100

    bars = int(percent / 5)
    bar = "█" * bars + "░" * (20 - bars)

    print(f"Raw: {raw:5d}  Light: {percent:5.1f}% |{bar}|")
    time.sleep(0.2)
