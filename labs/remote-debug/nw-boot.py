# boot.py
import network
import socket
import time
import ubinascii
from machine import Pin

# ===============================
# USER SETTINGS
# ===============================
WIFI_SSID = "NeoNexus"
WIFI_PASSWORD = "userC013$"

ROBOT_NAME = "Pico2-Car"
DISCOVERY_PORT = 37020

WIFI_TIMEOUT_S = 15

LED = Pin("LED", Pin.OUT)

# ===============================
# LED PATTERNS
# ===============================

def led_off():
    LED.off()

def led_connecting_step():
    # slow blink
    LED.on()
    time.sleep(0.15)
    LED.off()
    time.sleep(0.35)

def led_connected_heartbeat_once():
    # two quick blips then pause (run once)
    LED.on();  time.sleep(0.05)
    LED.off(); time.sleep(0.12)
    LED.on();  time.sleep(0.05)
    LED.off(); time.sleep(0.45)

def led_wifi_error_forever():
    # triple blink then pause
    while True:
        for _ in range(3):
            LED.on();  time.sleep(0.12)
            LED.off(); time.sleep(0.12)
        time.sleep(0.8)

# ===============================
# WIFI + DISCOVERY
# ===============================

def connect_wifi(timeout_s=WIFI_TIMEOUT_S):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Connecting to WiFi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    t0 = time.ticks_ms()
    while not wlan.isconnected():
        led_connecting_step()
        if time.ticks_diff(time.ticks_ms(), t0) > int(timeout_s * 1000):
            return None

    return wlan


def send_discovery_once(wlan):
    raw_mac = wlan.config("mac")
    mac = ubinascii.hexlify(raw_mac, b":").decode()
    ip = wlan.ifconfig()[0]

    msg = "ROBOT,{},{},{}".format(ROBOT_NAME, mac, ip)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.sendto(msg.encode(), ("255.255.255.255", DISCOVERY_PORT))
    s.close()

    print("Discovery sent:", msg)


# ===============================
# BOOT SEQUENCE
# ===============================

led_off()

wlan = connect_wifi()
if wlan is None:
    print("WiFi FAILED -> USB REPL stays available. Fix SSID/PASS in boot.py and reboot.")
    led_wifi_error_forever()

print("WiFi OK:", wlan.ifconfig())
send_discovery_once(wlan)

# show connected heartbeat a few times, then leave LED on steady
for _ in range(2):
    led_connected_heartbeat_once()

LED.on()  # steady ON = connected
time.sleep(1)
LED.off()  # turn it off to start main