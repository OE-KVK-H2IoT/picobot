import machine
import time

led = machine.Pin("LED", machine.Pin.OUT)
adcpin = 4
temp_sensor = machine.ADC(adcpin)
  
def ReadTemperature():
    adc_val = temp_sensor.read_u16()
    um = (3.3/65535) * adc_val
    temp = 27 - (um - 0.706)/0.001721
    return round(temp, 1)
  
while True:
    temperature = ReadTemperature()
    print(temperature)
    time.sleep(2)
