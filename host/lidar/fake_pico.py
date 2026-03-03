#!/usr/bin/env python3
"""
fake_pico.py — Simulates the Pico 2 W sending radar data over UDP.
Use this to test radar.py without actual hardware.

Usage:
    python fake_pico.py
"""

import socket
import time
import math
import random

PC_IP = "127.0.0.1"
PC_PORT = 5005

ANGLE_MIN = 10
ANGLE_MAX = 170
ANGLE_STEP = 2
DELAY_S = 0.03  # time between readings


def fake_environment(angle_deg):
    """
    Simulate a room with some objects.
    Returns distance in mm.
    """
    rad = math.radians(angle_deg)

    # Simulate walls: a rectangular room roughly 3m x 2m
    # Sensor at center of one wall, facing into the room.
    distances = []

    # Back wall at ~3000mm
    if 20 < angle_deg < 160:
        wall_dist = 3000 / max(0.01, math.sin(rad))
        distances.append(min(wall_dist, 4000))

    # Left wall
    if angle_deg > 90:
        left_dist = 1500 / max(0.01, abs(math.cos(rad)))
        distances.append(min(left_dist, 4000))

    # Right wall
    if angle_deg < 90:
        right_dist = 1500 / max(0.01, abs(math.cos(rad)))
        distances.append(min(right_dist, 4000))

    # Object 1: a box at ~60 degrees, 800mm away
    if 55 < angle_deg < 68:
        distances.append(800 + random.randint(-20, 20))

    # Object 2: a cylinder at ~120 degrees, 1200mm away
    if 115 < angle_deg < 128:
        distances.append(1200 + random.randint(-15, 15))

    # Object 3: person walking at ~90 degrees, distance varies
    t = time.time()
    person_dist = 1800 + 400 * math.sin(t * 0.5)
    if 85 < angle_deg < 95:
        distances.append(int(person_dist) + random.randint(-30, 30))

    if distances:
        # Return the nearest object (like real ToF)
        dist = int(min(distances))
        # Add some noise
        dist += random.randint(-10, 10)
        return max(30, dist)
    else:
        return -1


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Fake Pico sending to {}:{}".format(PC_IP, PC_PORT))
    print("Press Ctrl+C to stop.")

    angle = ANGLE_MIN
    direction = 1
    sweep_id = 0

    while True:
        dist = fake_environment(angle)
        msg = "{},{},{}\n".format(angle, dist, sweep_id)
        sock.sendto(msg.encode(), (PC_IP, PC_PORT))

        angle += direction * ANGLE_STEP
        if angle >= ANGLE_MAX:
            angle = ANGLE_MAX
            direction = -1
            sweep_id += 1
        elif angle <= ANGLE_MIN:
            angle = ANGLE_MIN
            direction = +1
            sweep_id += 1

        time.sleep(DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
