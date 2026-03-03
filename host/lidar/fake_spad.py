#!/usr/bin/env python3
"""
fake_spad.py — Simulates the Pico sending SPAD zone frames over UDP.
Use to test spad_viewer.py without hardware.

Also accepts servo commands on port 5007.

Usage:
    python fake_spad.py
"""

import socket
import time
import math
import random
import threading

PC_IP = "127.0.0.1"
DATA_PORT = 5006
CMD_PORT = 5007

SWEEP_MIN = 10
SWEEP_MAX = 170
SWEEP_STEP = 2


def fake_zone_distance(zr, zc, servo_angle):
    """Simulate distance for zone (zr, zc) at given servo angle."""
    t = time.time()

    # Base distance: walls of a room
    # Back wall distance varies with servo angle (wider = farther at edges)
    angle_rad = math.radians(servo_angle)
    base = 2500 / max(0.05, math.sin(angle_rad))
    base = min(base, 3800)

    # Zone affects distance slightly (perspective)
    zone_offset = (zr - 1.5) * 80 + (zc - 1.5) * 40

    dist = int(base + zone_offset)

    # Object 1: box at ~60 degrees, 800mm, visible in zones (1,1) and (1,2)
    if 55 < servo_angle < 70:
        if zr in (1, 2) and zc in (1, 2):
            dist = 800 + random.randint(-20, 20) + (zr - 1) * 30

    # Object 2: person at ~110 degrees, walking closer/farther
    person_dist = 1500 + 500 * math.sin(t * 0.4)
    if 105 < servo_angle < 120:
        if zc in (1, 2):
            dist = int(person_dist) + random.randint(-25, 25) + zr * 50

    # Object 3: small post at ~90 degrees, 600mm, only in bottom zones
    if 85 < servo_angle < 95:
        if zr >= 2 and zc in (1, 2):
            dist = 600 + random.randint(-15, 15)

    # Add noise
    dist += random.randint(-10, 10)
    return max(30, min(dist, 3900))


def command_listener(state):
    """Listen for commands from the PC app."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", CMD_PORT))
    sock.settimeout(0.5)
    print("Command listener on port {}".format(CMD_PORT))
    while state["running"]:
        try:
            data, addr = sock.recvfrom(64)
            cmd = data.decode("ascii").strip()
            if cmd.startswith("A"):
                angle = int(cmd[1:])
                state["servo_angle"] = max(0, min(180, angle))
                state["sweeping"] = False
                print("  CMD: servo -> {}".format(state["servo_angle"]))
            elif cmd == "W":
                state["sweeping"] = True
                state["servo_angle"] = SWEEP_MIN
                state["sweep_dir"] = 1
                print("  CMD: sweep start")
            elif cmd == "X":
                state["sweeping"] = False
                print("  CMD: sweep stop at {}".format(state["servo_angle"]))
        except socket.timeout:
            pass
    sock.close()


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Fake SPAD scanner sending to {}:{}".format(PC_IP, DATA_PORT))
    print("Press Ctrl+C to stop.")

    state = {
        "servo_angle": 90,
        "sweeping": False,
        "sweep_dir": 1,
        "running": True,
    }

    # Start command listener thread
    cmd_thread = threading.Thread(target=command_listener, args=(state,),
                                 daemon=True)
    cmd_thread.start()

    while state["running"]:
        angle = state["servo_angle"]

        # Generate 16 zone distances
        distances = []
        for zr in range(4):
            for zc in range(4):
                distances.append(fake_zone_distance(zr, zc, angle))

        # Build frame packet
        parts = ["F", str(angle)] + [str(d) for d in distances]
        msg = ",".join(parts) + "\n"
        sock.sendto(msg.encode(), (PC_IP, DATA_PORT))

        # Advance sweep
        if state["sweeping"]:
            angle += state["sweep_dir"] * SWEEP_STEP
            if angle >= SWEEP_MAX:
                angle = SWEEP_MAX
                state["sweep_dir"] = -1
            elif angle <= SWEEP_MIN:
                angle = SWEEP_MIN
                state["sweep_dir"] = 1
            state["servo_angle"] = angle

        # Simulate ~2.5 FPS (16 zones at 25ms each)
        time.sleep(0.4)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
