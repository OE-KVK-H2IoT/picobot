from picobot import PicoBot
from robot_state import RobotState, IMUStreamer
import network
import time

HOST_IP = "192.168.28.102"
HOST_PORT = 5005
CONTROL_PORT = 5006

print("RUN MODE")

wlan = network.WLAN(network.STA_IF)
if not wlan.isconnected():
    print("WiFi not connected")
else:
    print("WiFi OK:", wlan.ifconfig())

# robot = PicoBot(num_leds=4)
# robot.leds.fill(0,255,0)
# robot.leds.show()

# init IMU, streamer, etc
# ...

while True:
    # read sensors
    # send UDP
    # receive commands
    print("Working....")
    time.sleep(1)
    pass
