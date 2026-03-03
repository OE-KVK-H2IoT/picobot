"""Minimal NeoPixel test — run directly on Pico to verify PIO works."""
import array, time
from machine import Pin, disable_irq, enable_irq
import rp2

LED_PIN = 6
NUM_LEDS = 8

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")           .side(1)    [T2 - 1]
    label("do_zero")
    nop()                    .side(0)    [T2 - 1]
    wrap()

sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(LED_PIN))
sm.active(1)

pixels = array.array("I", [0] * NUM_LEDS)

def show(px):
    irq = disable_irq()
    sm.put(px, 8)
    enable_irq(irq)
    time.sleep_us(300)

# Test 1: All green
print("All GREEN...")
for i in range(NUM_LEDS):
    pixels[i] = (50 << 16) | (0 << 8) | 0  # GRB: G=50, R=0, B=0
show(pixels)
time.sleep(2)

# Test 2: All red
print("All RED...")
for i in range(NUM_LEDS):
    pixels[i] = (0 << 16) | (50 << 8) | 0  # GRB: G=0, R=50, B=0
show(pixels)
time.sleep(2)

# Test 3: All blue
print("All BLUE...")
for i in range(NUM_LEDS):
    pixels[i] = (0 << 16) | (0 << 8) | 50  # GRB: G=0, R=0, B=50
show(pixels)
time.sleep(2)

# Test 4: Off
print("OFF")
for i in range(NUM_LEDS):
    pixels[i] = 0
show(pixels)
