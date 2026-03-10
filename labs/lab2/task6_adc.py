from machine import ADC, Pin
import time

battery_adc = ADC(Pin(28))

while True:
    raw = battery_adc.read_u16()
    v_adc = (raw / 65535) * 3.3
    v_battery = v_adc * 3

    # State of charge (linear estimate)
    soc = ((v_battery - 3.0) / (4.2 - 3.0)) * 100
    soc = max(0, min(100, soc))  # Clamp to 0-100%

    # Visual battery bar
    bars = int(soc / 10)
    bar = "█" * bars + "░" * (10 - bars)

    print(f"Battery: {v_battery:.2f}V  SoC: {soc:.0f}% [{bar}]")
    time.sleep(2)
