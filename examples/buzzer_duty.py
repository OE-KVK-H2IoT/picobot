from machine import Pin, PWM
import time

buzzer = PWM(Pin(22))  # buzzer pin — check your pinout

# Sweep duty cycle at fixed frequency (2 kHz)
buzzer.freq(4000)
# for duty in range(0, 65536, 1024):
buzzer.duty_u16(55535)
time.sleep_ms(2000)
buzzer.duty_u16(0)               # silence
