from machine import Pin
import time

led = Pin("LED", Pin.OUT)

print("=" * 50)
print("PYTHON INTERPRETER SPEED TEST")
print("=" * 50)

# Test 1: Python pin toggle speed
print("\n1. Python led.on()/led.off() speed:")
start = time.ticks_us()
for _ in range(1000):
    led.on()
    led.off()
elapsed = time.ticks_diff(time.ticks_us(), start)
per_toggle = elapsed / 1000
print(f"   1000 toggles: {elapsed} µs")
print(f"   Per toggle:   {per_toggle:.1f} µs = {per_toggle * 1000:.0f} ns")

# Test 2: Python overhead (empty loop)
print("\n2. Python empty loop overhead:")
start = time.ticks_us()
for _ in range(1000):
    pass
elapsed = time.ticks_diff(time.ticks_us(), start)
print(f"   1000 iterations: {elapsed} µs")
print(f"   Per iteration:   {elapsed / 1000:.1f} µs")

# Test 3: Direct register access (faster but still Python)
print("\n3. Direct register toggle:")
from machine import mem32
SIO_BASE = 0xd0000000
GPIO_OUT_XOR = SIO_BASE + 0x01c
LED_BIT = 1 << 25  # Onboard LED - may vary on Pico 2 W

start = time.ticks_us()
for _ in range(1000):
    mem32[GPIO_OUT_XOR] = LED_BIT
elapsed = time.ticks_diff(time.ticks_us(), start)
print(f"   1000 toggles: {elapsed} µs")
print(f"   Per toggle:   {elapsed / 1000:.1f} µs")

# Summary
print("\n" + "=" * 50)
print("WS2812B TIMING REQUIREMENTS")
print("=" * 50)
print("""
'0' bit: 350ns HIGH, 900ns LOW  (total: 1250ns)
'1' bit: 800ns HIGH, 450ns LOW  (total: 1250ns)

Tolerance: ~150ns
""")

print("=" * 50)
print("THE PROBLEM")
print("=" * 50)
print(f"""
Python toggle speed:     ~{per_toggle * 1000:.0f} ns
WS2812B bit time:        ~1250 ns
WS2812B tolerance:       ~150 ns

Python is {per_toggle * 1000 / 1250:.0f}x SLOWER than needed!

Even without WiFi, Python cannot:
- Execute fast enough
- Guarantee consistent timing
- Avoid garbage collection pauses
""")

print("=" * 50)
print("THE SOLUTION: PIO")
print("=" * 50)
print("""
PIO (Programmable I/O) is HARDWARE, not software:
- Runs at 10 MHz = 100ns per instruction
- Not affected by Python speed
- Not affected by garbage collection
- Not affected by ANY interrupts (WiFi, timers, etc.)

That's why NeoPixels need PIO on Pico!
""")
