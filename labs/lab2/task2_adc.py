from machine import ADC
import time

temp_adc = ADC(ADC.CORE_TEMP)

while True:
    raw = temp_adc.read_u16()
    voltage = (3.3/ 65535) * raw 
    temperature = 27 - (voltage - 0.706) / 0.001721

    print(f"Raw: {raw}  Voltage: {voltage:.3f}V  Temp: {temperature:.1f}°C")
    time.sleep(1)
