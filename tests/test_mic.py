"""Mic diagnostic — amplitude-based beat detection (matches picobot library)."""
from machine import Pin, ADC
import time

MIC_PIN = 27
THRESHOLD = 80  # Same as demo_dj_live default sensitivity

adc = ADC(Pin(MIC_PIN))

# Calibrate
print("Calibrating (keep quiet)...")
time.sleep_ms(100)
samples = sorted(adc.read_u16() for _ in range(200))
trimmed = samples[20:180]
center = sum(trimmed) // len(trimmed)
noise = sum(abs(s - center) for s in trimmed) // len(trimmed)
if noise < 100:
    noise = 100

# Compute trigger threshold (same formula as picobot)
multiplier = 2 + THRESHOLD // 5
min_amp = max(2000, noise * multiplier)

print(f"  Center: {center}")
print(f"  Noise floor: {noise}")
print(f"  Raw range in silence: {min(samples)}-{max(samples)}")
print(f"  Threshold: {THRESHOLD} -> {multiplier}x noise -> amp > {min_amp}")
print()
print("Now make noise! (Ctrl+C to stop)")
print("raw       | amp   | min_amp | beat?")
print("----------|-------|---------|------")

last_beat = 0

while True:
    raw = adc.read_u16()
    amp = abs(raw - center)
    now = time.ticks_ms()

    is_beat = amp > min_amp and time.ticks_diff(now, last_beat) > 200
    if is_beat:
        last_beat = now

    print(f"{raw:9d} | {amp:5d} | {min_amp:7d} | {'BEAT!' if is_beat else ''}")
    time.sleep(0.02)
