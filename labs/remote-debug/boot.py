# boot.py
import network
import webrepl
import time

WIFI_SSID = "Flying-013"
WIFI_PASSWORD = "userC013"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

if not wlan.isconnected():
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    for _ in range(10):
        if wlan.isconnected():
            break
        time.sleep(1)

if wlan.isconnected():
    print("WiFi:", wlan.ifconfig())
    webrepl.start(password="robot")
else:
    print("WiFi failed – USB REPL only")
