"""Motor test with proper pin pairing — drive the 'other' pin LOW explicitly."""
from machine import Pin, PWM
import time

print("=== Motor Test (paired pins) ===\n")

# Test right motor forward: PWM on A=10, force B=11 LOW
print("Right FORWARD (PWM=10, LOW=11)...")
pin_b = Pin(11, Pin.OUT, value=0)       # B explicitly LOW
pwm_a = PWM(Pin(10))
pwm_a.freq(1000)
pwm_a.duty_u16(50000)                   # ~76%
time.sleep(2)
pwm_a.duty_u16(0)
pwm_a.deinit()
pin_b.init(Pin.IN)                      # release
print("  done\n")

time.sleep(0.5)

# Test right motor backward: PWM on B=11, force A=10 LOW
print("Right BACKWARD (PWM=11, LOW=10)...")
pin_a = Pin(10, Pin.OUT, value=0)       # A explicitly LOW
pwm_b = PWM(Pin(11))
pwm_b.freq(1000)
pwm_b.duty_u16(50000)
time.sleep(2)
pwm_b.duty_u16(0)
pwm_b.deinit()
pin_a.init(Pin.IN)
print("  done\n")

time.sleep(0.5)

# Test right motor forward at max duty
print("Right FORWARD full power (PWM=10, LOW=11)...")
pin_b = Pin(11, Pin.OUT, value=0)
pwm_a = PWM(Pin(10))
pwm_a.freq(1000)
pwm_a.duty_u16(65535)                   # 100%
time.sleep(2)
pwm_a.duty_u16(0)
pwm_a.deinit()
pin_b.init(Pin.IN)
print("  done\n")

print("Which tests rotated?")
