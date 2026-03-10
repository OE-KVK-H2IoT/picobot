from machine import Pin
import time

# Configure GP25 as an output (on-board LED on some Pico boards)
led = Pin("LED", Pin.OUT)

print("LED On")
led.on()       # Pin goes HIGH (3.3V)
time.sleep(1)

print("LED Off")
led.off()      # Pin goes LOW (0V)
time.sleep(1)

led.toggle()   # Flip state
print("LED Toggle")
