"""Direct motor pin test — bypasses picobot to verify wiring."""
from machine import Pin, PWM
import time

# From config.py
LEFT_A, LEFT_B = 13, 12
RIGHT_A, RIGHT_B = 10, 11

print("=== Motor Pin Test ===")
print(f"Left:  A={LEFT_A}, B={LEFT_B}")
print(f"Right: A={RIGHT_A}, B={RIGHT_B}")
print()

def test_pin(name, gpio, speed_pct=50):
    pwm = PWM(Pin(gpio))
    pwm.freq(1000)
    duty = int(speed_pct / 100 * 65535)
    print(f"  {name} (GPIO {gpio}) at {speed_pct}%...", end=" ")
    pwm.duty_u16(duty)
    time.sleep(1)
    pwm.duty_u16(0)
    pwm.deinit()
    print("done")

print("Test 1: Left forward (A=13)")
test_pin("LEFT_A", LEFT_A)
time.sleep(0.5)

print("Test 2: Left backward (B=12)")
test_pin("LEFT_B", LEFT_B)
time.sleep(0.5)

print("Test 3: Right forward (A=10)")
test_pin("RIGHT_A", RIGHT_A)
time.sleep(0.5)

print("Test 4: Right backward (B=11)")
test_pin("RIGHT_B", RIGHT_B)
time.sleep(0.5)

print("\nDone. Which tests moved a motor?")
