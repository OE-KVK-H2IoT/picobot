"""
==============================================================================
EXAMPLE: Remote Control Robot
==============================================================================

Simple remote-controlled robot using host.py keyboard commands.
No IMU streaming - just motor control over WiFi.

This is a minimal example showing how to use the ControlReceiver.

USAGE:
------
  1. Edit WIFI_SSID, WIFI_PASSWORD below
  2. Deploy to Pico along with libs/
  3. Run host.py on your PC (it will auto-detect robot)
  4. Use arrow keys to drive!

==============================================================================
"""

import time
import network
import socket
from picobot import PicoBot
from control_receiver import ControlReceiver


# ==============================================================================
# CONFIGURATION
# ==============================================================================

WIFI_SSID = 'NeoNexus'
WIFI_PASSWORD = 'userC013$'
CONTROL_PORT = 5006


# ==============================================================================
# MAIN
# ==============================================================================

def connect_wifi(ssid, password, timeout_s=15):
    """Connect to WiFi."""
    print(f"Connecting to {ssid}...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    for i in range(timeout_s):
        if wlan.isconnected():
            print(f"Connected! IP: {wlan.ifconfig()[0]}")
            return wlan
        time.sleep(1)

    print("WiFi failed!")
    return None


def main():
    print("="*40)
    print("Remote Control Robot")
    print("="*40)

    # Initialize
    robot = PicoBot(num_leds=4)
    robot.leds.fill(255, 255, 0)  # Yellow
    robot.leds.show()

    # Connect WiFi
    if not connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        robot.leds.fill(255, 0, 0)
        robot.leds.show()
        return

    # Start control receiver
    print(f"Listening for commands on port {CONTROL_PORT}")
    controller = ControlReceiver(CONTROL_PORT)

    # Ready!
    robot.leds.fill(0, 255, 0)  # Green
    robot.leds.show()
    print("Ready! Use arrow keys on host.py")

    was_active = False

    while True:
        # Check for commands
        controller.check_commands()

        if controller.is_active():
            # Remote control active
            left, right = controller.get_speeds()
            robot.motors.set_speeds(left, right)

            # Blue LED when controlled
            robot.leds.fill(0, 100, 255)
            robot.leds.show()

            if not was_active:
                print("Remote control ACTIVE")
                was_active = True

        else:
            # No commands - stop
            robot.motors.stop()

            if was_active:
                print("Remote control stopped")
                robot.leds.fill(0, 255, 0)
                robot.leds.show()
                was_active = False

        time.sleep_ms(10)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        from picobot import Motors
        Motors().stop()
