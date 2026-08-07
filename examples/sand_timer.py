"""
Sand Timer — physics-based sand in a diamond hourglass.

Two 8×8 LED panels mounted at 45° form a double-diamond shape.
Grains accumulate velocity from the accelerometer and move to
adjacent cells when velocity exceeds a threshold.

    mpremote run sand_timer.py
"""

# ═══════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════
PIN = 6
BRIGHTNESS = 60
NUM_GRAINS = 40
GRAVITY = 0.5
MOVE_THRESHOLD = 0.2
DAMPING = 0.5
NECK_COOLDOWN = 5       # Frames between grains passing through neck
GRAIN_COLOR = (150, 35, 5)
# ═══════════════════════════════════════════════════════════════════

import time
from picobot.led_panel import DiamondPanel
from picobot.imu import IMU

imu = IMU()
dp = DiamondPanel(pin=PIN, brightness=BRIGHTNESS)

W = dp.width
H = dp.height
valid = dp.valid_cells
virt_to_raw = dp.cell_to_index

print(f"Diamond Sand Timer: {W}x{H}, {len(valid)} pixels")

# ═══════════════════════════════════════════════════════════════════
# NEIGHBOR MAP
# ═══════════════════════════════════════════════════════════════════

OFFSETS = [(1,1),(1,-1),(-1,1),(-1,-1),(0,1),(0,-1),(0,2),(0,-2),(2,0),(-2,0)]

neighbors = {}
for cell in valid:
    nbrs = []
    for dx, dy in OFFSETS:
        n = (cell[0] + dx, cell[1] + dy)
        if n in valid:
            nbrs.append(n)
    neighbors[cell] = nbrs

# ═══════════════════════════════════════════════════════════════════
# GRAINS — live on grid cells, accumulate float velocity
# ═══════════════════════════════════════════════════════════════════

_seed = time.ticks_us()
def rng(n):
    global _seed
    _seed = (_seed * 1103515245 + 12345) & 0x7FFFFFFF
    return _seed % n

top_cells = sorted([c for c in valid if c[1] < H // 2])
grains = []  # Each: [cell_x, cell_y, vx, vy]
placed = set()

for _ in range(min(NUM_GRAINS, len(top_cells))):
    idx = rng(len(top_cells))
    for _att in range(len(top_cells)):
        cell = top_cells[(idx + _att) % len(top_cells)]
        if cell not in placed:
            grains.append([cell[0], cell[1], 0.0, 0.0])
            placed.add(cell)
            break

print(f"Placed {len(grains)} grains")
print("Tilt to move sand! Ctrl-C to exit.\n")

# Pre-pack colors for direct buffer writes
bright = BRIGHTNESS
outline_packed = ((4 * bright // 255) << 16) | ((4 * bright // 255) << 8) | (6 * bright // 255)
gr, gg, gb = GRAIN_COLOR
grain_packed = ((gg * bright // 255) << 16) | ((gr * bright // 255) << 8) | (gb * bright // 255)

# ═══════════════════════════════════════════════════════════════════
# SIMULATION
# ═══════════════════════════════════════════════════════════════════

# Neck rate limiter: only let a grain enter the neck cells every N frames
neck_cells = {(7, 14), (7, 15)}
neck_last_pass = 0
frame_count = 0

try:
    while True:
        frame_count += 1
        ax, ay, az = imu.accel()
        grav_x = -ax * GRAVITY
        grav_y = ay * GRAVITY

        # Build occupancy
        occupied = set()
        for g in grains:
            occupied.add((g[0], g[1]))

        # Sort by gravity direction
        grav_mag = (grav_x * grav_x + grav_y * grav_y) ** 0.5
        if grav_mag > 0.01:
            grains.sort(key=lambda g: -(g[0] * grav_x + g[1] * grav_y))

        for g in grains:
            pos = (g[0], g[1])

            # Accumulate velocity from gravity
            g[2] += grav_x
            g[3] += grav_y

            # Check if velocity is enough to move
            speed = (g[2] * g[2] + g[3] * g[3]) ** 0.5
            if speed < MOVE_THRESHOLD:
                continue

            # Find best neighbor in velocity direction
            occupied.discard(pos)
            nbrs = neighbors.get(pos, [])

            # Score all free neighbors by alignment with velocity.
            # Collect ALL positive-scoring moves, sorted best first.
            scored = []
            for n in nbrs:
                if n in occupied:
                    continue
                dx = n[0] - pos[0]
                dy = n[1] - pos[1]
                dist = (dx * dx + dy * dy) ** 0.5
                if dist > 0:
                    score = (dx * g[2] + dy * g[3]) / (dist * speed)
                    scored.append((score, n))

            scored.sort(key=lambda s: -s[0])

            # Filter out neck cells if cooldown hasn't expired
            # (grains already IN the neck can move freely)
            if pos not in neck_cells and (frame_count - neck_last_pass) < NECK_COOLDOWN:
                scored = [(s, n) for s, n in scored if n not in neck_cells]

            moved = False
            if scored and scored[0][0] > 0.1:
                target = scored[0][1]
                # Track neck passage
                if target in neck_cells and pos not in neck_cells:
                    neck_last_pass = frame_count
                g[0] = target[0]
                g[1] = target[1]
                g[2] *= DAMPING
                g[3] *= DAMPING
                occupied.add(target)
                moved = True
            elif scored:
                # No move aligned with velocity, but free neighbors exist.
                # "Settle" — pick any move that goes toward the gravity
                # low point. This makes grains slide down walls and
                # level out in piles instead of getting stuck.
                for sc, n in scored:
                    dx = n[0] - pos[0]
                    dy = n[1] - pos[1]
                    dist = (dx * dx + dy * dy) ** 0.5
                    # Score against gravity direction (not velocity)
                    grav_score = (dx * grav_x + dy * grav_y) / (dist * max(grav_mag, 0.001))
                    if grav_score > 0.1:
                        g[0] = n[0]
                        g[1] = n[1]
                        # Reset velocity to gravity direction
                        g[2] = grav_x * 0.3
                        g[3] = grav_y * 0.3
                        occupied.add(n)
                        moved = True
                        break

            if not moved:
                # Truly stuck — bounce velocity
                g[2] *= -0.3
                g[3] *= -0.3
                occupied.add(pos)

        # Render
        buf = dp._raw._buf
        for i in range(len(buf)):
            buf[i] = 0

        for cell in valid:
            buf[virt_to_raw[cell]] = outline_packed

        for g in grains:
            key = (g[0], g[1])
            if key in virt_to_raw:
                buf[virt_to_raw[key]] = grain_packed

        dp.show()
        time.sleep_ms(10)

except KeyboardInterrupt:
    print("\nExited.")
    dp.clear()
    dp.show()
