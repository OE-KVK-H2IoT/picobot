from machine import Pin, PWM
import time

buzzer = PWM(Pin(22))  # buzzer pin — check your pinout

# Sweep frequency at 50% duty
buzzer.duty_u16(32768)           # 50% duty cycle
for freq in range(100, 5001, 100):
    buzzer.freq(freq)
    time.sleep_ms(100)
buzzer.duty_u16(0)               # silence
time.sleep(1)
