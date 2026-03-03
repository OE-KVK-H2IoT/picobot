# timing_demo.py - Show WiFi impact on timing
from machine import Pin
import network
import time

led = Pin("LED", Pin.OUT)

def measure_jitter(iterations=1000):
    """Measure timing consistency of a tight loop."""
    times = []

    for _ in range(iterations):
        start = time.ticks_us()

        # Simulate timing-critical operation (like sending LED data)
        for _ in range(100):
            led.on()
            led.off()

        elapsed = time.ticks_diff(time.ticks_us(), start)
        times.append(elapsed)

    avg = sum(times) // len(times)
    min_t = min(times)
    max_t = max(times)
    jitter = max_t - min_t

    # Count "glitches" (iterations that took >2x average)
    glitches = sum(1 for t in times if t > avg * 2)

    return {
        'avg': avg,
        'min': min_t,
        'max': max_t,
        'jitter': jitter,
        'glitches': glitches
    }

# Test with WiFi ON
print("=" * 50)
print("TEST 1: WiFi ENABLED")
print("=" * 50)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
time.sleep(0.5)  # Let WiFi driver stabilize

results_on = measure_jitter()
print(f"  Average:  {results_on['avg']} µs")
print(f"  Min:      {results_on['min']} µs")
print(f"  Max:      {results_on['max']} µs")
print(f"  Jitter:   {results_on['jitter']} µs")
print(f"  Glitches: {results_on['glitches']} / 1000")

# Test with WiFi OFF
print()
print("=" * 50)
print("TEST 2: WiFi DISABLED")
print("=" * 50)
wlan.active(False)
time.sleep(0.5)

results_off = measure_jitter()
print(f"  Average:  {results_off['avg']} µs")
print(f"  Min:      {results_off['min']} µs")
print(f"  Max:      {results_off['max']} µs")
print(f"  Jitter:   {results_off['jitter']} µs")
print(f"  Glitches: {results_off['glitches']} / 1000")

# Summary
print()
print("=" * 50)
print("SUMMARY")
print("=" * 50)
print(f"  Jitter reduction: {results_on['jitter']} → {results_off['jitter']} µs")
print(f"  Glitch reduction: {results_on['glitches']} → {results_off['glitches']}")
print()
print("  WS2812B tolerance: ~150 ns (0.15 µs)")
print(f"  WiFi jitter:       ~{results_on['jitter']} µs = {results_on['jitter'] * 1000} ns")
print()
if results_on['jitter'] > 150:
    print("  ⚠ WiFi jitter EXCEEDS LED timing tolerance!")
    print("  → That's why software timing breaks with WiFi on")
    print("  → PIO runs independently, not affected by interrupts")
