#!/usr/bin/env python3
"""
spad_viewer.py — VL53L1X SPAD Array Viewer + Panoramic Scanner

Displays the 16x16 SPAD array with 4x4 zone distance measurements.
Optionally sweeps a servo for 180-degree panoramic depth scanning.

Listens for UDP frames from Pico on port 5006.
Sends servo commands to Pico on port 5007.

Usage:
    pip install pygame
    python spad_viewer.py
"""

import socket
import math
import sys
import pygame

# ===========================================================
# SETTINGS
# ===========================================================
DATA_PORT = 5006          # receive zone frames from Pico
CMD_PORT = 5007           # send servo commands to Pico

WINDOW_W = 1050
WINDOW_H = 850
FPS = 60

R_MAX_MM = 4000           # max distance for color scale
SWEEP_MIN = 0
SWEEP_MAX = 180
SWEEP_STEP = 2

# Colors
BG = (15, 15, 20)
GRID_LINE = (40, 45, 50)
ZONE_BORDER = (100, 110, 120)
TEXT_COLOR = (180, 190, 200)
TITLE_COLOR = (0, 200, 160)
LABEL_COLOR = (120, 130, 140)
HIGHLIGHT = (0, 255, 180)
WARN_COLOR = (200, 60, 60)
BTN_COLOR = (40, 50, 60)
BTN_HOVER = (60, 70, 85)
BTN_ACTIVE = (0, 150, 120)

# Layout
GRID_X, GRID_Y = 40, 50
GRID_PX = 400              # 16x16 grid is 400x400 pixels
SPAD_PX = GRID_PX // 16    # 25 px per SPAD cell
ZONE_PX = GRID_PX // 4     # 100 px per zone block

INFO_X = GRID_X + GRID_PX + 30
INFO_Y = GRID_Y

PANO_X = 40
PANO_Y = GRID_Y + GRID_PX + 60
PANO_W = WINDOW_W - 80
PANO_2D_H = 120             # 4-row 2D panorama
PANO_AVG_Y = PANO_Y + PANO_2D_H + 40  # averaged strip below
PANO_AVG_H = 60             # single strip height

SCALE_X = INFO_X
SCALE_Y = INFO_Y + 260
SCALE_W = 30
SCALE_H = 180

# ===========================================================


def dist_to_color(dist_mm, max_mm=4000):
    """Map distance to heatmap color (close=red, far=blue)."""
    if dist_mm <= 0:
        return (40, 40, 40)
    t = min(1.0, dist_mm / max_mm)
    if t < 0.25:
        r, g, b = 255, int(t / 0.25 * 255), 0
    elif t < 0.5:
        r, g, b = int((1 - (t - 0.25) / 0.25) * 255), 255, 0
    elif t < 0.75:
        r, g, b = 0, 255, int((t - 0.5) / 0.25 * 255)
    else:
        r, g, b = 0, int((1 - (t - 0.75) / 0.25) * 255), 255
    return (r, g, b)


def draw_spad_grid(screen, font, distances):
    """Draw the 16x16 SPAD array with 4x4 zone coloring."""
    # Fill each zone block
    for zr in range(4):
        for zc in range(4):
            idx = zr * 4 + zc
            dist = distances[idx]
            color = dist_to_color(dist, R_MAX_MM)

            # Zone rectangle
            x = GRID_X + zc * ZONE_PX
            y = GRID_Y + zr * ZONE_PX
            pygame.draw.rect(screen, color, (x, y, ZONE_PX, ZONE_PX))

            # Draw individual SPAD cell outlines (subtle)
            for sr in range(4):
                for sc in range(4):
                    sx = x + sc * SPAD_PX
                    sy = y + sr * SPAD_PX
                    pygame.draw.rect(screen, GRID_LINE,
                                     (sx, sy, SPAD_PX, SPAD_PX), 1)

            # Distance label in zone center
            if dist > 0:
                if dist >= 1000:
                    label = "{:.1f}m".format(dist / 1000)
                else:
                    label = "{}".format(dist)
            else:
                label = "---"
            text = font.render(label, True, (0, 0, 0))
            tx = x + ZONE_PX // 2 - text.get_width() // 2
            ty = y + ZONE_PX // 2 - text.get_height() // 2
            # Shadow for readability
            shadow = font.render(label, True, (255, 255, 255))
            screen.blit(shadow, (tx + 1, ty + 1))
            screen.blit(text, (tx, ty))

    # Zone borders (thick)
    for i in range(5):
        # Vertical
        x = GRID_X + i * ZONE_PX
        pygame.draw.line(screen, ZONE_BORDER, (x, GRID_Y),
                         (x, GRID_Y + GRID_PX), 2)
        # Horizontal
        y = GRID_Y + i * ZONE_PX
        pygame.draw.line(screen, ZONE_BORDER, (GRID_X, y),
                         (GRID_X + GRID_PX, y), 2)

    # Row/col labels
    small = pygame.font.SysFont("consolas", 10)
    for i in range(16):
        # Top (column)
        t = small.render(str(i), True, LABEL_COLOR)
        screen.blit(t, (GRID_X + i * SPAD_PX + SPAD_PX // 2 - t.get_width() // 2,
                         GRID_Y - 14))
        # Left (row)
        t = small.render(str(i), True, LABEL_COLOR)
        screen.blit(t, (GRID_X - 16, GRID_Y + i * SPAD_PX + SPAD_PX // 2 - 5))


def draw_color_scale(screen, font):
    """Draw a vertical color scale bar."""
    x, y, w, h = SCALE_X + 180, SCALE_Y, SCALE_W, SCALE_H

    # Title
    t = font.render("Distance", True, TEXT_COLOR)
    screen.blit(t, (x, y - 20))

    # Gradient bar
    for py in range(h):
        t = py / h
        color = dist_to_color(int(t * R_MAX_MM), R_MAX_MM)
        pygame.draw.line(screen, color, (x, y + py), (x + w, y + py))

    # Border
    pygame.draw.rect(screen, ZONE_BORDER, (x, y, w, h), 1)

    # Labels
    small = pygame.font.SysFont("consolas", 11)
    for frac, label in [(0, "0m"), (0.25, "{:.1f}m".format(R_MAX_MM * 0.25 / 1000)),
                         (0.5, "{:.1f}m".format(R_MAX_MM * 0.5 / 1000)),
                         (0.75, "{:.1f}m".format(R_MAX_MM * 0.75 / 1000)),
                         (1.0, "{:.1f}m".format(R_MAX_MM / 1000))]:
        ly = y + int(frac * h)
        t = small.render(label, True, LABEL_COLOR)
        screen.blit(t, (x + w + 5, ly - 6))


def _draw_pano_empty(screen, font, y, h, label):
    """Draw empty panorama placeholder."""
    pygame.draw.rect(screen, (25, 25, 30), (PANO_X, y, PANO_W, h))
    pygame.draw.rect(screen, GRID_LINE, (PANO_X, y, PANO_W, h), 1)
    t = font.render(label, True, LABEL_COLOR)
    screen.blit(t, (PANO_X + PANO_W // 2 - t.get_width() // 2,
                     y + h // 2 - 8))


def _draw_angle_labels(screen, sweep_angles, cell_w, y):
    """Draw angle tick labels below a panorama."""
    small = pygame.font.SysFont("consolas", 10)
    n = len(sweep_angles)
    step = max(1, n // 10)
    for ai in range(0, n, step):
        t = small.render("{}".format(sweep_angles[ai]), True, LABEL_COLOR)
        px = PANO_X + ai * cell_w
        screen.blit(t, (px, y))


def draw_panorama_2d(screen, font, pano_2d, sweep_angles):
    """Draw 4-row 2D panoramic view."""
    if not sweep_angles:
        _draw_pano_empty(screen, font, PANO_Y, PANO_2D_H,
                         "2D Panorama (press S to sweep)")
        return

    n_angles = len(sweep_angles)
    cell_w = max(2, PANO_W // n_angles)
    cell_h = PANO_2D_H // 4

    for ai, angle in enumerate(sweep_angles):
        if angle not in pano_2d:
            continue
        row_dists = pano_2d[angle]
        for row in range(4):
            dist = row_dists[row]
            color = dist_to_color(dist, R_MAX_MM)
            px = PANO_X + ai * cell_w
            py = PANO_Y + row * cell_h
            pygame.draw.rect(screen, color, (px, py, cell_w, cell_h))

    total_w = n_angles * cell_w

    # Border + row separators
    pygame.draw.rect(screen, ZONE_BORDER,
                     (PANO_X, PANO_Y, total_w, PANO_2D_H), 1)
    for row in range(1, 4):
        y = PANO_Y + row * cell_h
        pygame.draw.line(screen, GRID_LINE,
                         (PANO_X, y), (PANO_X + total_w, y))

    # Row labels
    small = pygame.font.SysFont("consolas", 10)
    for row in range(4):
        t = small.render("R{}".format(row), True, LABEL_COLOR)
        screen.blit(t, (PANO_X - 22,
                         PANO_Y + row * cell_h + cell_h // 2 - 5))

    _draw_angle_labels(screen, sweep_angles, cell_w,
                       PANO_Y + PANO_2D_H + 3)


def draw_panorama_avg(screen, font, pano_avg, sweep_angles):
    """Draw single-strip averaged panorama."""
    if not sweep_angles:
        _draw_pano_empty(screen, font, PANO_AVG_Y, PANO_AVG_H,
                         "Average")
        return

    n_angles = len(sweep_angles)
    cell_w = max(2, PANO_W // n_angles)

    for ai, angle in enumerate(sweep_angles):
        dist = pano_avg.get(angle, -1)
        color = dist_to_color(dist, R_MAX_MM)
        px = PANO_X + ai * cell_w
        pygame.draw.rect(screen, color,
                         (px, PANO_AVG_Y, cell_w, PANO_AVG_H))

    total_w = n_angles * cell_w
    pygame.draw.rect(screen, ZONE_BORDER,
                     (PANO_X, PANO_AVG_Y, total_w, PANO_AVG_H), 1)

    _draw_angle_labels(screen, sweep_angles, cell_w,
                       PANO_AVG_Y + PANO_AVG_H + 3)


def draw_info(screen, font, small_font, servo_angle, sweeping, pico_ip,
              frame_count, fps_val, scan_mode_name="Normal 16z"):
    """Draw info panel."""
    x, y = INFO_X, INFO_Y
    lines = [
        ("Servo", "{}deg".format(servo_angle)),
        ("Sweep", "ON" if sweeping else "OFF"),
        ("Scan", scan_mode_name),
        ("Pico", pico_ip if pico_ip else "waiting..."),
        ("Frames", str(frame_count)),
        ("FPS", "{:.1f}".format(fps_val)),
        ("Max range", "{:.1f}m".format(R_MAX_MM / 1000)),
    ]
    for i, (label, value) in enumerate(lines):
        t = font.render("{}: {}".format(label, value), True, TEXT_COLOR)
        screen.blit(t, (x, y + i * 22))

    # Servo gauge
    gy = y + len(lines) * 22 + 15
    gw = 200
    t = small_font.render("Servo position:", True, LABEL_COLOR)
    screen.blit(t, (x, gy))
    gy += 18
    pygame.draw.rect(screen, GRID_LINE, (x, gy, gw, 14))
    fill_w = int((servo_angle / 180) * gw)
    pygame.draw.rect(screen, HIGHLIGHT, (x, gy, fill_w, 14))
    # Tick marks
    for deg in (0, 45, 90, 135, 180):
        tx = x + int(deg / 180 * gw)
        pygame.draw.line(screen, ZONE_BORDER, (tx, gy), (tx, gy + 14))
    t = small_font.render("{}".format(servo_angle), True, TEXT_COLOR)
    screen.blit(t, (x + fill_w - t.get_width() // 2, gy - 14))


def send_command(sock, pico_ip, cmd):
    """Send a command string to the Pico."""
    if pico_ip:
        try:
            sock.sendto(cmd.encode(), (pico_ip, CMD_PORT))
        except OSError:
            pass


def main():
    # UDP sockets
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind(("0.0.0.0", DATA_PORT))
    data_sock.setblocking(False)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Listening for SPAD data on UDP port {}...".format(DATA_PORT))
    print("Commands will be sent to Pico port {}.".format(CMD_PORT))

    # Pygame
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("VL53L1X SPAD Array Viewer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 14)
    small_font = pygame.font.SysFont("consolas", 11)
    title_font = pygame.font.SysFont("consolas", 18, bold=True)
    zone_font = pygame.font.SysFont("consolas", 13, bold=True)

    # State
    distances = [-1] * 16
    servo_angle = 0
    sweeping = True
    pico_ip = None
    scan_modes = ["MN", "MT", "MF"]
    scan_mode_names = ["Normal 16z", "Turbo 4z", "Fast 1z"]
    scan_mode_idx = 1
    frame_count = 0
    fps_val = 0.0
    fps_timer = pygame.time.get_ticks()
    fps_frames = 0

    # Panorama data per angle
    pano_2d = {}    # {angle: [row0_avg, row1_avg, row2_avg, row3_avg]}
    pano_avg = {}   # {angle: single_avg}
    sweep_angles = []  # built dynamically from received angles

    running = True
    while running:
        # --- Events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_s:
                    # Toggle sweep
                    sweeping = not sweeping
                    if sweeping:
                        send_command(cmd_sock, pico_ip, "W")
                        pano_2d.clear()
                        pano_avg.clear()
                        sweep_angles = []
                    else:
                        send_command(cmd_sock, pico_ip, "X")
                elif event.key == pygame.K_1:
                    send_command(cmd_sock, pico_ip, "1")
                    servo_angle = 0
                    sweeping = False
                elif event.key == pygame.K_2:
                    send_command(cmd_sock, pico_ip, "2")
                    servo_angle = 90
                    sweeping = False
                elif event.key == pygame.K_3:
                    send_command(cmd_sock, pico_ip, "3")
                    servo_angle = 180
                    sweeping = False
                elif event.key == pygame.K_t:
                    # Quick sweep: one full pass then return to 0
                    send_command(cmd_sock, pico_ip, "T")
                    sweeping = True
                    pano_2d.clear()
                    pano_avg.clear()
                    sweep_angles = []
                elif event.key == pygame.K_f:
                    # Cycle scan mode: Normal → Turbo → Fast
                    scan_mode_idx = (scan_mode_idx + 1) % 3
                    send_command(cmd_sock, pico_ip, scan_modes[scan_mode_idx])
                elif event.key == pygame.K_r:
                    # Reset servo to center (90)
                    send_command(cmd_sock, pico_ip, "R")
                    servo_angle = 90
                    sweeping = False
                elif event.key == pygame.K_c:
                    pano_2d.clear()
                    pano_avg.clear()
                    sweep_angles = []
                elif event.key == pygame.K_LEFT:
                    servo_angle = max(0, servo_angle - 5)
                    send_command(cmd_sock, pico_ip,
                                "A{}".format(servo_angle))
                    sweeping = False
                elif event.key == pygame.K_RIGHT:
                    servo_angle = min(180, servo_angle + 5)
                    send_command(cmd_sock, pico_ip,
                                "A{}".format(servo_angle))
                    sweeping = False
                elif event.key == pygame.K_UP:
                    servo_angle = min(180, servo_angle + 1)
                    send_command(cmd_sock, pico_ip,
                                "A{}".format(servo_angle))
                    sweeping = False
                elif event.key == pygame.K_DOWN:
                    servo_angle = max(0, servo_angle - 1)
                    send_command(cmd_sock, pico_ip,
                                "A{}".format(servo_angle))
                    sweeping = False

        # --- Receive data ---
        while True:
            try:
                data, addr = data_sock.recvfrom(512)
            except BlockingIOError:
                break
            except OSError:
                break

            if pico_ip is None:
                pico_ip = addr[0]
                print("Pico found at {}".format(pico_ip))

            try:
                line = data.decode("ascii").strip()
                if not line.startswith("F,"):
                    continue
                parts = line.split(",")
                # F,angle,dir,d0,...,d15
                servo_angle = int(parts[1])
                sweep_dir = int(parts[2])
                for i in range(16):
                    distances[i] = int(parts[3 + i])

                frame_count += 1
                fps_frames += 1

                # Store panorama only on forward sweep (dir=1)
                # to avoid misalignment from servo backlash
                if sweeping and sweep_dir == 1:
                    row_avgs = []
                    all_vals = []
                    for zr in range(4):
                        vals = [distances[zr * 4 + zc]
                                for zc in range(4)
                                if distances[zr * 4 + zc] > 0]
                        avg = sum(vals) // len(vals) if vals else -1
                        row_avgs.append(avg)
                        all_vals.extend(vals)
                    pano_2d[servo_angle] = row_avgs
                    pano_avg[servo_angle] = (sum(all_vals) // len(all_vals)
                                             if all_vals else -1)
                    sweep_angles = sorted(pano_2d.keys())

            except (ValueError, IndexError):
                pass

        # FPS calculation
        now = pygame.time.get_ticks()
        if now - fps_timer >= 1000:
            fps_val = fps_frames / ((now - fps_timer) / 1000)
            fps_frames = 0
            fps_timer = now

        # --- Draw ---
        screen.fill(BG)

        # Title
        t = title_font.render("VL53L1X SPAD Array Viewer", True, TITLE_COLOR)
        screen.blit(t, (GRID_X, 15))

        # SPAD grid
        draw_spad_grid(screen, zone_font, distances)

        # Info panel
        draw_info(screen, font, small_font, servo_angle, sweeping,
                  pico_ip, frame_count, fps_val,
                  scan_mode_names[scan_mode_idx])

        # Color scale
        draw_color_scale(screen, small_font)

        # Panorama labels + draw
        t = font.render("Panoramic 2D ({} angles)".format(
            len(pano_2d)), True, TEXT_COLOR)
        screen.blit(t, (PANO_X, PANO_Y - 20))
        draw_panorama_2d(screen, font, pano_2d, sweep_angles)

        t = font.render("Average", True, TEXT_COLOR)
        screen.blit(t, (PANO_X, PANO_AVG_Y - 20))
        draw_panorama_avg(screen, font, pano_avg, sweep_angles)

        # Controls help
        help_text = "[1]Left [2]Mid [3]Right [F]Speed [S]Sweep [T]Scan [R]Reset [Arrows]Servo [C]Clear [Q]Quit"
        t = small_font.render(help_text, True, LABEL_COLOR)
        screen.blit(t, (PANO_X, WINDOW_H - 22))

        # Connection warning
        if pico_ip is None:
            warn = font.render(
                "Waiting for Pico data on UDP:{}...".format(DATA_PORT),
                True, WARN_COLOR)
            screen.blit(warn, (GRID_X + GRID_PX // 2 - warn.get_width() // 2,
                               GRID_Y + GRID_PX // 2))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    data_sock.close()
    cmd_sock.close()
    print("Done.")


if __name__ == "__main__":
    main()
