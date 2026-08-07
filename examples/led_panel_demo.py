"""
LED Panel Demos — animations for WS2812 8×8 and multi-panel displays.

This file demonstrates common animation techniques used in LED art,
game graphics, and embedded displays. Each demo teaches a different
concept — read the comments to understand the algorithms.

Edit the configuration below to match your hardware, then:
    mpremote run led_panel_demo.py

Controls: Ctrl-C skips to next demo, double Ctrl-C exits.
"""

# ═══════════════════════════════════════════════════════════════════
# CONFIGURATION — edit these to match your setup
# ═══════════════════════════════════════════════════════════════════
PIN = 6          # GPIO pin connected to DIN
PANELS = 2       # Number of chained 8×8 panels (1, 2, 3, ...)
PANEL_WIDTH = 8  # Width of each physical panel
PANEL_HEIGHT = 8 # Height of each physical panel
BRIGHTNESS = 80  # 0-255 (keep low to reduce power and avoid blinding)
# ═══════════════════════════════════════════════════════════════════

import time
import math

from picobot import LEDPanel

# Calculate total display dimensions from panel configuration
WIDTH = PANEL_WIDTH * PANELS   # e.g. 2 panels × 8 = 16 columns
HEIGHT = PANEL_HEIGHT          # rows stay the same (panels are side-by-side)

# Create the panel with the right geometry.
# panel_width tells the driver where one physical panel ends and the
# next begins in the data chain — essential for correct 2D mapping.
if PANELS > 1:
    panel = LEDPanel(pin=PIN, num_leds=WIDTH * HEIGHT, width=WIDTH,
                 panel_width=PANEL_WIDTH, brightness=BRIGHTNESS)
else:
    panel = LEDPanel(pin=PIN, num_leds=WIDTH * HEIGHT, width=WIDTH,
                 brightness=BRIGHTNESS)

print(f"LED Panel Demo: {WIDTH}x{HEIGHT} on pin {PIN}")
print(f"Ctrl-C to skip, double Ctrl-C to exit\n")


def run(name, func, duration=8):
    """Run a demo for `duration` seconds, Ctrl-C skips to next."""
    print(f"  {name}...")
    try:
        func(duration)
    except KeyboardInterrupt:
        pass
    panel.clear()
    panel.show()
    time.sleep(0.3)


# ═══════════════════════════════════════════════════════════════════
# RGB DEMOS
# ═══════════════════════════════════════════════════════════════════

def rainbow_columns(duration):
    """
    Rainbow sweep — each column gets a different hue, scrolling over time.

    TECHNIQUE: HSV color space
    --------------------------
    Instead of mixing R/G/B manually, we use HSV (Hue, Saturation, Value):
      - Hue: 0.0-1.0 = color wheel (0=red, 0.33=green, 0.67=blue, 1.0=red)
      - Saturation: 0=gray, 1=vivid
      - Value: 0=black, 1=bright

    By mapping the x position to hue, each column gets a different color.
    Adding a time-based offset makes the rainbow scroll.
    """
    print("    Each column gets a hue based on its x position.")
    print("    Adding a time offset makes the rainbow scroll.")
    print("    hue = (x / width + time * speed) mod 1.0")

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        offset = time.ticks_diff(time.ticks_ms(), start_time) / 1000

        for x in range(WIDTH):
            hue = ((x / WIDTH) + offset * 0.3) % 1.0
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            # Fill the entire column with this color
            for y in range(HEIGHT):
                panel.pixel(x, y, (r, g, b))

        panel.show()
        time.sleep_ms(30)  # ~33 FPS


def plasma(duration):
    """
    Plasma effect — classic demoscene algorithm from the 1990s.

    TECHNIQUE: Additive sine waves
    --------------------------------
    Plasma works by combining multiple sine waves at different scales
    and speeds. At each pixel, we compute:

      v = sin(x) + sin(y) + sin(x+y) + sin(distance_from_center)

    The result is a smoothly varying value that we map to a color.
    Different frequencies and phase speeds create organic, flowing
    patterns that never repeat exactly.

    This is the same math behind many screensavers, shader effects,
    and generative art. The key insight: adding sine waves at different
    frequencies creates complex patterns from simple math.
    """
    print("    4 sine waves at different scales are added per pixel:")
    print("    v = sin(x) + sin(y) + sin(x+y) + sin(distance)")
    print("    The sum becomes a color. Shifting phase over time animates it.")

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000

        for y in range(HEIGHT):
            for x in range(WIDTH):
                wave_sum = math.sin(x * 0.5 + elapsed * 2)           # Vertical bands
                wave_sum += math.sin(y * 0.5 + elapsed * 1.5)        # Horizontal bands
                wave_sum += math.sin((x + y) * 0.3 + elapsed)        # Diagonal bands
                wave_sum += math.sin(                                 # Radial ripple
                    math.sqrt(x * x + y * y) * 0.4
                )

                # wave_sum ranges from -4 to +4 — normalize to 0.0-1.0 for hue
                hue = (wave_sum / 4.0 + 0.5) % 1.0
                r, g, b = hsv_to_rgb(hue, 1.0, 0.8)
                panel.pixel(x, y, (r, g, b))

        panel.show()
        time.sleep_ms(30)


def color_wipe(duration):
    """
    Color wipe — fills one LED at a time, then switches color.

    TECHNIQUE: Sequential fill
    ---------------------------
    The simplest possible animation: light up pixels one by one.
    This is useful for testing your wiring order and panel mapping,
    and it demonstrates the show()-per-pixel approach (slow but
    visually satisfying).
    """
    colors = [
        (60, 0, 0),   # Red
        (0, 60, 0),   # Green
        (0, 0, 60),   # Blue
        (60, 60, 0),  # Yellow
        (0, 60, 60),  # Cyan
        (60, 0, 60),  # Magenta
    ]
    color_index = 0
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        color = colors[color_index % len(colors)]
        for y in range(HEIGHT):
            for x in range(WIDTH):
                panel.pixel(x, y, color)
                panel.show()           # Show after EACH pixel (intentionally slow)
                time.sleep_ms(15)
        color_index += 1


def fire(duration):
    """
    Fire simulation — heat rises and dissipates.

    TECHNIQUE: Cellular automaton with heat diffusion
    --------------------------------------------------
    Each pixel stores a "heat" value (0-255). Every frame:

    1. Cool down: subtract random small amount (simulates heat loss)
    2. Diffuse upward: each pixel averages its value with the three
       pixels below it (heat rises in real fires)
    3. Ignite: randomly add heat to the bottom row (the "fuel")
    4. Render: map heat value to a color gradient:
       0-84: black to red        (cold to warm)
       85-169: red to yellow     (warm to hot)
       170-255: yellow to white  (hot to white-hot)

    This is a simplified version of the "doom fire" algorithm
    (used in the Doom PSX port's title screen, 1995).
    """
    print("    Each pixel has a 'heat' value (0-255). Every frame:")
    print("    1. Cool down randomly    2. Heat rises (average from below)")
    print("    3. Ignite bottom row     4. Map heat to black>red>yellow>white")

    heat = [0] * (WIDTH * HEIGHT)
    start_time = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        # Step 1: Cool down each pixel slightly
        for i in range(len(heat)):
            cool = _rng() % 3     # Random cooling: 0, 1, or 2
            heat[i] = max(0, heat[i] - cool)

        # Step 2: Heat rises — each pixel becomes the average of
        # the three pixels below it (y+1 row, columns x-1, x, x+1).
        # This creates the upward-flowing "flames" shape.
        for y in range(HEIGHT - 1):
            for x in range(WIDTH):
                below_left  = heat[min(HEIGHT-1, y+1) * WIDTH + max(0, x-1)]
                below       = heat[min(HEIGHT-1, y+1) * WIDTH + x]
                below_right = heat[min(HEIGHT-1, y+1) * WIDTH + min(WIDTH-1, x+1)]
                heat[y * WIDTH + x] = (below_left + below + below_right) // 3

        # Step 3: Randomly ignite the bottom row
        for x in range(WIDTH):
            if _rng() % 3 == 0:   # 33% chance per pixel per frame
                idx = (HEIGHT - 1) * WIDTH + x
                heat[idx] = min(255, heat[idx] + 80 + _rng() % 80)

        # Step 4: Render heat values as colors
        for y in range(HEIGHT):
            for x in range(WIDTH):
                heat_value = heat[y * WIDTH + x]
                # Three-zone color gradient: black > red > yellow > white
                if heat_value < 85:
                    panel.pixel(x, y, (heat_value * 3 // 4, 0, 0))
                elif heat_value < 170:
                    panel.pixel(x, y, (63, (heat_value - 85) * 3 // 4, 0))
                else:
                    panel.pixel(x, y, (63, 63, (heat_value - 170) * 3 // 4))

        panel.show()
        time.sleep_ms(50)  # ~20 FPS


def rotating_gradient(duration):
    """
    Rotating color gradient — a color plane that spins.

    TECHNIQUE: Dot product for directional gradient
    -------------------------------------------------
    A gradient in a specific direction can be computed using the
    dot product of each pixel's position with a direction vector.

    The dot product (x*dx + y*dy) gives a scalar that increases
    as you move in the direction (dx, dy). By rotating (dx, dy)
    over time, the gradient rotates.

    This is the same math used in shader programming (GLSL/HLSL)
    for directional lighting and gradient effects.
    """
    start_time = time.ticks_ms()
    center_x, center_y = WIDTH / 2, HEIGHT / 2  # Center of display

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        # Rotating direction vector
        angle = time.ticks_diff(time.ticks_ms(), start_time) / 2000 * math.pi
        dir_x = math.cos(angle)
        dir_y = math.sin(angle)

        for y in range(HEIGHT):
            for x in range(WIDTH):
                # Dot product: how far along the gradient direction
                dot = (x - center_x) * dir_x + (y - center_y) * dir_y
                # Normalize to 0.0-1.0 and use as hue
                hue = (dot / max(WIDTH, HEIGHT) + 0.5) % 1.0
                r, g, b = hsv_to_rgb(hue, 1.0, 0.8)
                panel.pixel(x, y, (r, g, b))

        panel.show()
        time.sleep_ms(30)


# ═══════════════════════════════════════════════════════════════════
# MONOCHROME DEMOS
# ═══════════════════════════════════════════════════════════════════

def scanner(duration):
    """
    Knight Rider / Cylon scanner — a bright bar sweeps back and forth.

    TECHNIQUE: Sine-based ping-pong with distance falloff
    ------------------------------------------------------
    sin(t) naturally oscillates between -1 and +1, which maps perfectly
    to back-and-forth motion. We scale it to the display width.

    Each column's brightness is based on its distance from the current
    position — closer = brighter, creating a soft glow effect instead
    of a single hard pixel. This is a simple form of anti-aliasing.
    """
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
        # sin() gives smooth ping-pong between 0 and WIDTH-1
        pos = (math.sin(elapsed * 3) + 1) / 2 * (WIDTH - 1)

        panel.clear()
        for x in range(WIDTH):
            dist = abs(x - pos)
            if dist < 3:
                # Linear falloff: brightness decreases with distance
                bright = int(max(0, (3 - dist) / 3 * 60))
                for y in range(HEIGHT):
                    panel.pixel(x, y, (bright, 0, 0))

        panel.show()
        time.sleep_ms(20)


def game_of_life(duration):
    """
    Conway's Game of Life — cellular automaton.

    TECHNIQUE: Cellular automaton
    ------------------------------
    Each cell is either alive or dead. Every generation, four rules
    determine the next state based on the count of live neighbors:

      1. Live cell with <2 neighbors dies       (underpopulation)
      2. Live cell with 2-3 neighbors survives
      3. Live cell with >3 neighbors dies        (overcrowding)
      4. Dead cell with exactly 3 neighbors born (reproduction)

    These four simple rules produce astonishingly complex behavior:
    gliders, oscillators, spaceships, and even Turing-complete
    computation. Invented by mathematician John Conway in 1970.

    The grid wraps around (toroidal topology) so patterns that leave
    one edge reappear on the opposite side.
    """
    print("    4 rules determine life and death each generation:")
    print("    - <2 neighbors: dies (lonely)  - 2-3 neighbors: survives")
    print("    - >3 neighbors: dies (crowded) - exactly 3: born!")
    print("    Watch for stable patterns, oscillators, and gliders.")

    grid = [[(_rng() % 4 == 0) for _ in range(WIDTH)] for _ in range(HEIGHT)]
    start_time = time.ticks_ms()
    generation = 0

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        # Render current generation
        for y in range(HEIGHT):
            for x in range(WIDTH):
                panel.pixel(x, y, (0, 40, 0) if grid[y][x] else (0, 0, 0))
        panel.show()

        # Compute next generation
        new_grid = [[False] * WIDTH for _ in range(HEIGHT)]
        for y in range(HEIGHT):
            for x in range(WIDTH):
                # Count live neighbors (8-connected, wrapping)
                neighbor_count = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue  # Don't count self
                        neighbor_y = (y + dy) % HEIGHT  # Wrap vertically
                        neighbor_x = (x + dx) % WIDTH   # Wrap horizontally
                        if grid[neighbor_y][neighbor_x]:
                            neighbor_count += 1

                # Apply Conway's rules
                if grid[y][x]:
                    new_grid[y][x] = neighbor_count in (2, 3)  # Survive
                else:
                    new_grid[y][x] = neighbor_count == 3       # Born

        grid = new_grid
        generation += 1

        # Restart with fresh random state if population dies or stalls
        alive_count = sum(sum(row) for row in grid)
        if alive_count == 0 or generation > 200:
            grid = [[(_rng() % 4 == 0) for _ in range(WIDTH)] for _ in range(HEIGHT)]
            generation = 0

        time.sleep_ms(150)


def rain(duration):
    """
    Falling rain — blue drops with fading trails.

    TECHNIQUE: Particle system
    ---------------------------
    Each raindrop is a "particle" with position (x, y) and a brightness
    value. Every frame:
      1. Randomly spawn new drops at the top
      2. Move each drop down by 0.5 pixels
      3. Draw the drop and a dimmer trail pixel above it
      4. Remove drops that fall off the bottom

    Particle systems are used everywhere in games: rain, snow, sparks,
    explosions, fire. The key idea is managing a list of independent
    objects that are created, updated, and destroyed each frame.
    """
    print("    Each raindrop is a 'particle' with position and brightness.")
    print("    Spawn at top, move down 0.5px/frame, remove at bottom.")
    print("    A dim trail pixel above each drop creates the streak effect.")

    drops = []  # Each drop: [x_pos, y_pos, brightness]
    start_time = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        # Randomly spawn new drops at the top
        if _rng() % 3 == 0:
            drops.append([_rng() % WIDTH, 0, 20 + _rng() % 40])

        panel.clear()
        surviving_drops = []
        for drop in drops:
            drop_x, drop_y, drop_bright = drop
            if drop_y < HEIGHT:
                # Draw the drop
                panel.pixel(drop_x, int(drop_y), (0, 0, drop_bright))
                # Draw a dimmer trail above it
                if drop_y - 1 >= 0:
                    panel.pixel(drop_x, int(drop_y - 1), (0, 0, drop_bright // 3))
                drop[1] += 0.5  # Move down (sub-pixel movement for smoothness)
                surviving_drops.append(drop)
            # Drops past the bottom are simply not kept

        drops = surviving_drops
        panel.show()
        time.sleep_ms(60)


def breathe(duration):
    """
    Breathing light — the entire panel smoothly pulses white.

    TECHNIQUE: Sine-based brightness modulation
    ---------------------------------------------
    sin(t) oscillates between -1 and +1. We remap it to 0-40 for
    brightness. The sine curve gives a natural "ease in, ease out"
    feel — the same acceleration curve used in CSS animations and
    motion design (it's technically a cosine-based ease).
    """
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
        # Map sin (range -1 to 1) to brightness (range 0 to 40)
        brightness_level = int((math.sin(elapsed * 2) + 1) / 2 * 40)
        panel.fill((brightness_level, brightness_level, brightness_level))
        panel.show()
        time.sleep_ms(30)


def maze_gen(duration):
    """
    Animated maze generation using recursive backtracker (DFS).

    TECHNIQUE: Depth-first search with backtracking
    -------------------------------------------------
    This is the classic maze generation algorithm:

    1. Start at a random cell, mark it visited, push to stack
    2. Look at the current cell's unvisited neighbors
    3. If there are unvisited neighbors:
       - Pick one randomly
       - Mark it visited, push to stack
       - Light it up (bright green = frontier)
    4. If ALL neighbors are visited (dead end):
       - Dim the current cell (dark green = explored)
       - Pop from stack (backtrack)
    5. Repeat until stack is empty (all cells visited)

    The stack acts as a "breadcrumb trail" — when we hit a dead end,
    we backtrack along the trail until we find a cell with unvisited
    neighbors. This guarantees every cell is visited exactly once.

    This same algorithm is used in game level generation, circuit
    routing, and pathfinding.
    """
    print("    Depth-first search: explore → hit dead end → backtrack.")
    print("    Bright green = frontier, dim = explored, very dim = backtracked.")
    print("    The stack acts as a breadcrumb trail back to unexplored cells.")

    visited = [[False] * WIDTH for _ in range(HEIGHT)]
    stack = [(0, 0)]
    visited[0][0] = True

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        if not stack:
            # Maze complete — restart from a random position
            visited = [[False] * WIDTH for _ in range(HEIGHT)]
            start_x, start_y = _rng() % WIDTH, _rng() % HEIGHT
            stack = [(start_x, start_y)]
            visited[start_y][start_x] = True
            panel.clear()

        x, y = stack[-1]  # Current cell (top of stack)

        # Find unvisited neighbors (4-connected: up, right, down, left)
        neighbors = []
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT and not visited[ny][nx]:
                neighbors.append((nx, ny))

        if neighbors:
            # Pick a random unvisited neighbor and move there
            nx, ny = neighbors[_rng() % len(neighbors)]
            visited[ny][nx] = True
            stack.append((nx, ny))
            panel.pixel(nx, ny, (0, 30, 0))   # Bright green = frontier
            panel.pixel(x, y, (0, 10, 0))     # Dim the cell we came from
        else:
            # Dead end — backtrack
            panel.pixel(x, y, (0, 5, 0))      # Very dim = fully explored
            stack.pop()

        panel.show()
        time.sleep_ms(30)


# ═══════════════════════════════════════════════════════════════════
# UTILITY FUNCTIONS
# ═══════════════════════════════════════════════════════════════════

# Simple pseudo-random number generator (PRNG).
# MicroPython has no random module on all ports, so we use a
# "xorshift32" algorithm — three XOR-and-shift operations that
# produce a well-distributed sequence from any non-zero seed.
# This is NOT cryptographically secure, but perfect for animations.
_rng_state = time.ticks_us()  # Seed from current time (varies each run)

def _rng():
    """Xorshift32 PRNG — returns a pseudo-random 32-bit integer."""
    global _rng_state
    _rng_state ^= (_rng_state << 13) & 0xFFFFFFFF
    _rng_state ^= (_rng_state >> 17)
    _rng_state ^= (_rng_state << 5) & 0xFFFFFFFF
    return _rng_state & 0xFFFFFFFF


def hsv_to_rgb(hue, saturation, value):
    """
    Convert HSV color to RGB.

    HSV (Hue-Saturation-Value) is much easier to work with for
    animations than RGB:
      - hue: 0.0-1.0, the color wheel position
      - saturation: 0.0 = gray, 1.0 = vivid color
      - value: 0.0 = black, 1.0 = full brightness

    Returns (red, green, blue) in 0-63 range (dimmed for LED panels —
    full 255 on all LEDs would draw too much current).

    The algorithm divides the hue circle into 6 sectors of 60 degrees
    and linearly interpolates between the primary/secondary colors
    in each sector. This is the standard HSV-to-RGB conversion
    used in image processing and computer graphics.
    """
    if saturation == 0:
        # No saturation = grayscale
        gray = int(value * 63)
        return (gray, gray, gray)

    # Determine which 60-degree sector of the color wheel we're in
    sector = int(hue * 6)           # Sector index (0-5)
    fraction = hue * 6 - sector     # Fractional position within sector

    # Pre-compute the three interpolated components.
    # These formulas come from linearly blending between the two
    # primary/secondary colors at the edges of each sector.
    min_component = int(value * (1 - saturation) * 63)
    decreasing   = int(value * (1 - fraction * saturation) * 63)
    increasing   = int(value * (1 - (1 - fraction) * saturation) * 63)
    max_component = int(value * 63)

    sector %= 6  # Wrap sector index
    if sector == 0: return (max_component, increasing, min_component)      # Red to Yellow
    if sector == 1: return (decreasing, max_component, min_component)      # Yellow to Green
    if sector == 2: return (min_component, max_component, increasing)      # Green to Cyan
    if sector == 3: return (min_component, decreasing, max_component)      # Cyan to Blue
    if sector == 4: return (increasing, min_component, max_component)      # Blue to Magenta
    return (max_component, min_component, decreasing)                      # Magenta to Red


# ═══════════════════════════════════════════════════════════════════
# RUN ALL DEMOS
# ═══════════════════════════════════════════════════════════════════

try:
    print("=== RGB Demos ===")
    run("Rainbow columns", rainbow_columns)
    run("Plasma", plasma)
    run("Color wipe", color_wipe, 6)
    run("Fire", fire, 10)
    run("Rotating gradient", rotating_gradient)

    print("\n=== Monochrome Demos ===")
    run("Scanner", scanner)
    run("Game of Life", game_of_life, 12)
    run("Rain", rain)
    run("Breathe", breathe, 6)
    run("Maze generator", maze_gen, 10)

    print("\nAll demos complete!")

except KeyboardInterrupt:
    print("\nExited.")

panel.clear()
panel.show()
