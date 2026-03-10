from machine import ADC, Pin
import time

# Battery voltage through voltage divider on GP28 (ADC2)
battery_adc = ADC(Pin(28))

while True:
    raw = battery_adc.read_u16()
    v_adc = (raw / 65535) * 3.3
    v_battery = v_adc * 3  # Voltage divider ratio

    print(f"Raw: {raw}  V_adc: {v_adc:.2f}V  V_battery: {v_battery:.2f}V")
    time.sleep(1)
