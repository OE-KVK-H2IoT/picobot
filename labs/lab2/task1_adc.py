from machine import ADC

# Internal temperature sensor — no external pin needed
temp_adc = ADC(ADC.CORE_TEMP)

# Read raw 16-bit value
raw = temp_adc.read_u16()
print(f"Raw ADC value: {raw}")
