#!/usr/bin/env python3
"""
radar.py — ToF Radar Display
Listens for UDP packets from Pico 2 W and renders a real-time radar view.

Packet format (ASCII): "angle_deg,distance_mm,sweep_id\n"

Usage:
    pip install pygame
    python radar.py
"""

import socket
import math
import sys
import pygame

# ===========================================================
# SETTINGS
# ===========================================================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

WINDOW_W = 1000
WINDOW_H = 560
FPS = 60

R_MAX_MM = 4000          # max distance on display (mm) — adjust to your environment
TRAIL_FADE_SPEED = 3     # lower = longer trails
POINT_RADIUS = 3

# Colors (R, G, B)
BG_COLOR = (10, 10, 10)
GRID_COLOR = (0, 40, 0)
GRID_TEXT_COLOR = (0, 80, 0)
SWEEP_LINE_COLOR = (0, 100, 0)
POINT_COLOR = (0, 255, 50)
LABEL_COLOR = (0, 150, 0)
TITLE_COLOR = (0, 200, 0)
WARNING_COLOR = (200, 50, 50)

# ===========================================================

CENTER_X = WINDOW_W // 2
CENTER_Y = WINDOW_H - 40
RADAR_R = WINDOW_H - 70  # max pixel radius of the radar arc


def angle_to_rad(deg):
    """Convert sensor angle (0=left, 180=right) to screen radians."""
    return math.radians(180 - deg)


def polar_to_screen(angle_deg, dist_mm):
    """Convert (angle, distance) to pixel coordinates."""
    theta = angle_to_rad(angle_deg)
    r_px = (dist_mm / R_MAX_MM) * RADAR_R
    x = CENTER_X + r_px * math.cos(theta)
    y = CENTER_Y - r_px * math.sin(theta)
    return int(x), int(y)


def draw_grid(screen, font):
    """Draw range arcs, angle lines, and labels."""
    # Range arcs at 25%, 50%, 75%, 100%
    for frac in (0.25, 0.5, 0.75, 1.0):
        r = int(RADAR_R * frac)
        rect = pygame.Rect(CENTER_X - r, CENTER_Y - r, 2 * r, 2 * r)
        pygame.draw.arc(screen, GRID_COLOR, rect, 0, math.pi, 1)

        # Distance label
        dist_label = "{:.1f}m".format(R_MAX_MM * frac / 1000)
        text = font.render(dist_label, True, GRID_TEXT_COLOR)
        screen.blit(text, (CENTER_X + r + 4, CENTER_Y - 14))

    # Angle lines every 30 degrees
    for deg in range(0, 181, 30):
        theta = angle_to_rad(deg)
        ex = CENTER_X + RADAR_R * math.cos(theta)
        ey = CENTER_Y - RADAR_R * math.sin(theta)
        pygame.draw.line(screen, GRID_COLOR, (CENTER_X, CENTER_Y), (int(ex), int(ey)), 1)

        # Angle label
        lx = CENTER_X + (RADAR_R + 12) * math.cos(theta)
        ly = CENTER_Y - (RADAR_R + 12) * math.sin(theta)
        label = font.render("{}".format(deg), True, GRID_TEXT_COLOR)
        screen.blit(label, (int(lx) - label.get_width() // 2,
                            int(ly) - label.get_height() // 2))

    # Baseline
    pygame.draw.line(screen, GRID_COLOR,
                     (CENTER_X - RADAR_R - 20, CENTER_Y),
                     (CENTER_X + RADAR_R + 20, CENTER_Y), 1)

    # Center dot
    pygame.draw.circle(screen, GRID_TEXT_COLOR, (CENTER_X, CENTER_Y), 3)


def draw_sweep_line(screen, angle_deg):
    """Draw the current sweep line."""
    theta = angle_to_rad(angle_deg)
    ex = CENTER_X + RADAR_R * math.cos(theta)
    ey = CENTER_Y - RADAR_R * math.sin(theta)
    pygame.draw.line(screen, SWEEP_LINE_COLOR,
                     (CENTER_X, CENTER_Y), (int(ex), int(ey)), 2)


def draw_hud(screen, font, stats):
    """Draw heads-up display with stats."""
    y = 8
    for label, value in stats:
        text = font.render("{}: {}".format(label, value), True, LABEL_COLOR)
        screen.blit(text, (10, y))
        y += 18


def main():
    # UDP socket (non-blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    print("Listening on UDP port {}...".format(UDP_PORT))

    # Pygame init
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("ToF Lidar Radar")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 13)
    title_font = pygame.font.SysFont("consolas", 16, bold=True)

    # State
    points = []  # [(x, y, brightness)]
    current_angle = 90
    last_dist = 0
    packet_count = 0
    sweep_count = 0
    last_sweep_id = -1
    connected = False

    running = True
    while running:
        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_c:
                    points.clear()
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    R_MAX_MM_ref = globals()
                    pass  # could add zoom

        # Read all pending UDP packets
        while True:
            try:
                data, addr = sock.recvfrom(256)
            except BlockingIOError:
                break
            except OSError:
                break

            connected = True
            try:
                line = data.decode("ascii").strip()
                parts = line.split(",")
                angle = int(parts[0])
                dist = int(parts[1])
                sid = int(parts[2]) if len(parts) > 2 else 0

                current_angle = angle
                last_dist = dist
                packet_count += 1

                if sid != last_sweep_id:
                    last_sweep_id = sid
                    sweep_count += 1

                # Only plot valid readings
                if 0 < dist < R_MAX_MM:
                    x, y = polar_to_screen(angle, dist)
                    points.append([x, y, 255])

            except (ValueError, IndexError):
                pass

        # Fade old points
        new_points = []
        for p in points:
            p[2] -= TRAIL_FADE_SPEED
            if p[2] > 0:
                new_points.append(p)
        points = new_points

        # --- Draw ---
        screen.fill(BG_COLOR)
        draw_grid(screen, font)
        draw_sweep_line(screen, current_angle)

        # Draw points with fade
        for x, y, brightness in points:
            b = max(0, min(255, int(brightness)))
            color = (0, b, b // 4)
            pygame.draw.circle(screen, color, (x, y), POINT_RADIUS)

        # HUD
        stats = [
            ("Angle", "{}deg".format(current_angle)),
            ("Distance", "{}mm".format(last_dist) if last_dist > 0 else "---"),
            ("Packets", str(packet_count)),
            ("Sweeps", str(sweep_count)),
            ("Max range", "{}m".format(R_MAX_MM / 1000)),
        ]
        draw_hud(screen, font, stats)

        # Title
        title = title_font.render("ToF Lidar Radar", True, TITLE_COLOR)
        screen.blit(title, (WINDOW_W - title.get_width() - 10, 8))

        # Connection status
        if not connected:
            warn = title_font.render("Waiting for data on UDP:{}...".format(UDP_PORT),
                                     True, WARNING_COLOR)
            screen.blit(warn, (WINDOW_W // 2 - warn.get_width() // 2, WINDOW_H // 2))

        # Controls hint
        hint = font.render("[Q] Quit  [C] Clear", True, GRID_TEXT_COLOR)
        screen.blit(hint, (WINDOW_W - hint.get_width() - 10, WINDOW_H - 22))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sock.close()
    print("Done.")


if __name__ == "__main__":
    main()
