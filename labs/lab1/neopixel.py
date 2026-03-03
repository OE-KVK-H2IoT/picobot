from machine import Pin
import rp2, time

LED_PIN = 6

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def ws2812():
    T1 = 3
    T2 = 6
    T3 = 4
    wrap_target()
    label("bitloop")
    out(x, 1)              .side(0) [T3 - 1]
    jmp(not_x, "do_zero")  .side(1) [T1 - 1]
    jmp("bitloop")         .side(1) [T2 - 1]
    label("do_zero")
    nop()                  .side(0) [T2 - 1]
    wrap()

sm = rp2.StateMachine(
    0,
    ws2812,
    freq=10_000_000,
    sideset_base=Pin(LED_PIN),
    out_base=Pin(LED_PIN)   # ← DO NOT OMIT
)

sm.active(1)

def put_grb(g, r, b):
    sm.put(((g << 16) | (r << 8) | b)<<8)

put_grb(0, 50, 0)   # GREEN
time.sleep(1)
put_grb(50, 0, 0)   # RED
time.sleep(1)
put_grb(0, 0, 50)   # BLUE
time.sleep(1)
put_grb(0, 0, 0)

