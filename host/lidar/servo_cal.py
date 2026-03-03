#!/usr/bin/env python3
"""
servo_cal.py — Interactive servo calibration tool

Controls the servo in real-time via UDP.
First run on Pico: mpremote connect /dev/ttyACM0 run pico/cal_servo.py
Then run this:     python pc/servo_cal.py

Keys:
    Left/Right   -1 / +1 degree
    Down/Up      -10 / +10 degrees
    Home/End     Go to 0 / 180
    R            Reverse direction
    1/2/3        Save current angle as Left/Center/Right preset
    P            Print saved config for main_spad.py
    Space        Slow sweep 0->180->0
    Q/Esc        Quit
"""

import socket
import sys
import pygame

# Pico address — will auto-detect or set manually
PICO_IP = None  # auto-detect from first response, or set e.g. "192.168.6.245"
CMD_PORT = 5007

WINDOW_W = 520
WINDOW_H = 400

BG = (20, 20, 25)
GAUGE_COLOR = (50, 55, 65)
NEEDLE_COLOR = (0, 220, 160)
TEXT_COLOR = (190, 200, 210)
DIM_COLOR = (100, 110, 120)
ACCENT = (0, 180, 140)
WARN = (220, 80, 60)
PRESET_COLORS = [(255, 100, 100), (100, 255, 100), (100, 100, 255)]


def send_cmd(sock, cmd):
    global PICO_IP
    if PICO_IP:
        sock.sendto(cmd.encode(), (PICO_IP, CMD_PORT))


def main():
    global PICO_IP

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Servo Calibration")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    big_font = pygame.font.SysFont("consolas", 36, bold=True)
    small_font = pygame.font.SysFont("consolas", 12)
    title_font = pygame.font.SysFont("consolas", 20, bold=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", CMD_PORT + 100))  # listen for responses
    sock.setblocking(False)

    angle = 90
    pulse_us = 1500
    reverse = False
    presets = [None, None, None]  # left, center, right
    preset_names = ["Left", "Center", "Right"]
    sweep_active = False
    sweep_angle = 0
    sweep_dir = 1

    # Try to find Pico
    if not PICO_IP:
        # Try broadcast or common IPs
        print("Waiting for Pico... (make sure cal_servo.py is running on Pico)")
        print("Trying to reach Pico on port {}...".format(CMD_PORT))
        # Try sending to broadcast
        for attempt_ip in ["192.168.6.245", "192.168.6.255", "255.255.255.255"]:
            try:
                sock.sendto("A90".encode(), (attempt_ip, CMD_PORT))
            except OSError:
                pass

    running = True
    send_timer = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                sweep_active = False
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_LEFT:
                    angle = max(0, angle - 1)
                elif event.key == pygame.K_RIGHT:
                    angle = min(180, angle + 1)
                elif event.key == pygame.K_DOWN:
                    angle = max(0, angle - 10)
                elif event.key == pygame.K_UP:
                    angle = min(180, angle + 10)
                elif event.key == pygame.K_HOME:
                    angle = 0
                elif event.key == pygame.K_END:
                    angle = 180
                elif event.key == pygame.K_r:
                    send_cmd(sock, "REV")
                elif event.key == pygame.K_1:
                    presets[0] = angle
                elif event.key == pygame.K_2:
                    presets[1] = angle
                elif event.key == pygame.K_3:
                    presets[2] = angle
                elif event.key == pygame.K_p:
                    print("\n--- Config for main_spad.py ---")
                    print("SERVO_REVERSE = {}".format(reverse))
                    for i, name in enumerate(preset_names):
                        if presets[i] is not None:
                            print("# {} position: {} deg".format(name, presets[i]))
                    print("-------------------------------\n")
                elif event.key == pygame.K_SPACE:
                    sweep_active = True
                    sweep_angle = 0
                    sweep_dir = 1

        # Hold arrow keys for continuous movement
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            angle = max(0, angle - 1)
        if keys[pygame.K_RIGHT]:
            angle = min(180, angle + 1)

        # Sweep
        if sweep_active:
            sweep_angle += sweep_dir * 1
            if sweep_angle >= 180:
                sweep_dir = -1
            elif sweep_angle <= 0:
                sweep_active = False
            angle = max(0, min(180, sweep_angle))

        # Send angle to Pico (throttled to ~30 Hz)
        send_timer += 1
        if send_timer >= 2:
            send_timer = 0
            send_cmd(sock, "A{}".format(angle))

        # Receive responses
        try:
            data, addr = sock.recvfrom(128)
            if PICO_IP is None:
                PICO_IP = addr[0]
                print("Pico found at {}".format(PICO_IP))
            line = data.decode("ascii").strip()
            if line.startswith("OK,"):
                parts = line.split(",")
                angle = int(parts[1])
                pulse_us = float(parts[2])
                reverse = parts[3] == "1"
        except (OSError, BlockingIOError):
            pass

        # --- Draw ---
        screen.fill(BG)

        # Title
        t = title_font.render("Servo Calibration", True, ACCENT)
        screen.blit(t, (20, 15))

        conn_text = "Connected: {}".format(PICO_IP) if PICO_IP else "Waiting for Pico..."
        conn_color = ACCENT if PICO_IP else WARN
        t = small_font.render(conn_text, True, conn_color)
        screen.blit(t, (WINDOW_W - t.get_width() - 20, 20))

        # Big angle display
        t = big_font.render("{:3d}".format(angle), True, TEXT_COLOR)
        screen.blit(t, (WINDOW_W // 2 - t.get_width() // 2, 55))
        t = font.render("degrees", True, DIM_COLOR)
        screen.blit(t, (WINDOW_W // 2 - t.get_width() // 2, 95))

        # Pulse width
        t = font.render("Pulse: {:.0f} us".format(pulse_us), True, DIM_COLOR)
        screen.blit(t, (WINDOW_W // 2 - t.get_width() // 2, 118))

        # Reverse indicator
        rev_text = "Direction: REVERSED" if reverse else "Direction: NORMAL"
        rev_color = WARN if reverse else ACCENT
        t = font.render(rev_text, True, rev_color)
        screen.blit(t, (WINDOW_W // 2 - t.get_width() // 2, 142))

        # Horizontal gauge bar
        gy = 175
        gx = 40
        gw = WINDOW_W - 80
        gh = 30

        pygame.draw.rect(screen, GAUGE_COLOR, (gx, gy, gw, gh), border_radius=4)

        # Needle position
        nx = gx + int(angle / 180 * gw)
        pygame.draw.rect(screen, NEEDLE_COLOR, (nx - 3, gy - 5, 6, gh + 10),
                         border_radius=2)

        # Tick marks and labels
        for deg in range(0, 181, 30):
            tx = gx + int(deg / 180 * gw)
            pygame.draw.line(screen, DIM_COLOR, (tx, gy + gh), (tx, gy + gh + 8))
            t = small_font.render(str(deg), True, DIM_COLOR)
            screen.blit(t, (tx - t.get_width() // 2, gy + gh + 10))

        # Preset markers on gauge
        for i, preset in enumerate(presets):
            if preset is not None:
                px = gx + int(preset / 180 * gw)
                pygame.draw.polygon(screen, PRESET_COLORS[i],
                                    [(px, gy - 8), (px - 5, gy - 16), (px + 5, gy - 16)])

        # Presets display
        py = 240
        t = font.render("Saved Presets:", True, TEXT_COLOR)
        screen.blit(t, (40, py))
        for i, name in enumerate(preset_names):
            val = "{}".format(presets[i]) if presets[i] is not None else "---"
            color = PRESET_COLORS[i] if presets[i] is not None else DIM_COLOR
            t = font.render("[{}] {}: {} deg".format(i + 1, name, val), True, color)
            screen.blit(t, (60, py + 25 + i * 22))

        # Controls help
        controls = [
            "Left/Right: +/-1 deg    Up/Down: +/-10 deg",
            "Home/End: 0/180    Space: Sweep    R: Reverse",
            "[1][2][3]: Save preset    [P]: Print config    [Q]: Quit",
        ]
        for i, line in enumerate(controls):
            t = small_font.render(line, True, DIM_COLOR)
            screen.blit(t, (40, WINDOW_H - 60 + i * 16))

        pygame.display.flip()
        clock.tick(60)

    # Print final config
    print("\n=== Final Calibration ===")
    print("SERVO_REVERSE = {}".format(reverse))
    for i, name in enumerate(preset_names):
        if presets[i] is not None:
            print("{} = {} deg".format(name, presets[i]))
    print("=========================")

    pygame.quit()
    sock.close()


if __name__ == "__main__":
    main()
