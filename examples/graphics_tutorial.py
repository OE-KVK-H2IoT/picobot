"""
Graphics Tutorial — learn to draw shapes from scratch on LED panels.

This tutorial builds up from single pixels to a rotating 3D cube,
teaching the math and techniques behind computer graphics. Each step
is a self-contained demo with explanations.

The same concepts apply to OLED displays, TFT screens, or any
pixel-based display — only the driver changes.

Edit configuration below, then:
    mpremote run graphics_tutorial.py

Controls: Ctrl-C skips to next step, double Ctrl-C exits.
"""

# ═══════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════
PIN = 6
PANELS = 2
PANEL_WIDTH = 8
PANEL_HEIGHT = 8
BRIGHTNESS = 60
# ═══════════════════════════════════════════════════════════════════

import time
import math
from picobot import LEDPanel

WIDTH = PANEL_WIDTH * PANELS
HEIGHT = PANEL_HEIGHT

if PANELS > 1:
    panel = LEDPanel(pin=PIN, num_leds=WIDTH * HEIGHT, width=WIDTH,
                     panel_width=PANEL_WIDTH, brightness=BRIGHTNESS)
else:
    panel = LEDPanel(pin=PIN, num_leds=WIDTH * HEIGHT, width=WIDTH,
                     brightness=BRIGHTNESS)


def run_step(name, func, duration=8):
    """Run a tutorial step, Ctrl-C skips to next."""
    print(f"\n{'='*50}")
    print(f"  {name}")
    print(f"{'='*50}")
    try:
        func(duration)
    except KeyboardInterrupt:
        pass
    panel.clear()
    panel.show()
    time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════════
# STEP 0: Why framebuffers matter
# ═══════════════════════════════════════════════════════════════════

def step0_immediate_vs_framebuffer(duration):
    """
    THE MOST IMPORTANT LESSON IN DISPLAY PROGRAMMING

    There are two ways to draw on a display:

    Method 1 — "Immediate mode" (show after each pixel):
      Set pixel → show → set pixel → show → ...
      You SEE each pixel appear one by one.
      This is SLOW because sending data to the display takes time.

    Method 2 — "Framebuffer mode" (draw everything, then show once):
      Set pixel → set pixel → set pixel → ... → show (once)
      The entire frame appears at once — no flicker, no partial updates.
      This is FAST because you only send data once per frame.

    Watch the difference:
    """
    total_pixels = WIDTH * HEIGHT
    print(f"  Your display has {WIDTH}x{HEIGHT} = {total_pixels} pixels.")
    print()
    print("  METHOD 1: Immediate mode")
    print("  -------------------------")
    print(f"  We'll draw {total_pixels} pixels, calling show() after EACH one.")
    print("  Each show() sends ALL pixel data to the LED chain.")
    print("  Watch the panel — you'll see it fill in slowly...\n")
    time.sleep(1)

    # --- Method 1: Immediate mode (slow, visible drawing) ---
    panel.clear()
    panel.show()

    immediate_start = time.ticks_ms()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            brightness = int(x / WIDTH * 50)
            panel.pixel(x, y, (0, brightness, 0))
            panel.show()  # <-- SHOW AFTER EVERY PIXEL (slow!)

    immediate_time = time.ticks_diff(time.ticks_ms(), immediate_start)
    print(f"  Done! Immediate mode took: {immediate_time} ms")
    print(f"  That's {total_pixels} × show() calls = {total_pixels} full data transfers.")
    time.sleep(1.5)

    # --- Method 2: Framebuffer mode (fast, instant appearance) ---
    print()
    print("  METHOD 2: Framebuffer mode")
    print("  ---------------------------")
    print(f"  Same {total_pixels} pixels, but we write to a memory buffer first,")
    print("  then send everything with ONE show() call.")
    print("  Watch — it will appear instantly...\n")
    time.sleep(1)

    panel.clear()
    panel.show()
    time.sleep(0.3)

    framebuffer_start = time.ticks_ms()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            brightness = int(x / WIDTH * 50)
            panel.pixel(x, y, (0, brightness, 0))
            # NO show() here — just writing to memory (nanoseconds)

    panel.show()  # <-- ONE show() for the entire frame
    framebuffer_time = time.ticks_diff(time.ticks_ms(), framebuffer_start)

    speedup = immediate_time // max(framebuffer_time, 1)
    print(f"  Done! Framebuffer mode took: {framebuffer_time} ms")
    print(f"  That's 1 × show() call = 1 data transfer.")
    print()
    print(f"  ┌─────────────────────────────────────┐")
    print(f"  │  Immediate: {immediate_time:>6} ms  ({total_pixels} transfers) │")
    print(f"  │  Framebuf:  {framebuffer_time:>6} ms  (1 transfer)    │")
    print(f"  │  Speedup:   {speedup:>5}×                    │")
    print(f"  └─────────────────────────────────────┘")
    print()
    print("  KEY INSIGHT: pixel() writes to memory (fast).")
    print("  show() sends data to the LEDs (slow).")
    print("  Minimize show() calls = maximize frame rate.")
    print()
    print("  Every demo after this uses framebuffer mode.")
    time.sleep(4)


# ═══════════════════════════════════════════════════════════════════
# STEP 1: Points and lines
# ═══════════════════════════════════════════════════════════════════

def draw_line(x0, y0, x1, y1, color):
    """
    Draw a line from (x0,y0) to (x1,y1) using linear interpolation.

    HOW IT WORKS:
    A line between two points can be described parametrically:
      x = x0 + t * (x1 - x0)
      y = y0 + t * (y1 - y0)
    where t goes from 0.0 (start point) to 1.0 (end point).

    We divide the line into `steps` segments and draw a pixel at
    each position. More steps = smoother line.

    This is simpler than Bresenham's algorithm (which uses only
    integer math) but easier to understand. On a small LED panel,
    the performance difference doesn't matter.
    """
    # Calculate how many pixels we need to draw.
    # Use the longer axis to avoid gaps in the line.
    delta_x = abs(x1 - x0)
    delta_y = abs(y1 - y0)
    steps = max(delta_x, delta_y, 1)

    for step in range(steps + 1):
        # t goes from 0.0 to 1.0 along the line
        t = step / steps
        # Interpolate x and y positions
        pixel_x = int(x0 + t * (x1 - x0) + 0.5)  # +0.5 for rounding
        pixel_y = int(y0 + t * (y1 - y0) + 0.5)
        panel.pixel(pixel_x, pixel_y, color)


def step1_lines(duration):
    """
    DRAWING LINES — the foundation of all vector graphics.

    Every shape (triangle, rectangle, circle) is made of lines.
    A line between two points is the simplest geometric primitive.

    We'll draw lines in different directions to show that the
    parametric approach works at any angle.
    """
    print("  A line from A to B is computed with parametric interpolation:")
    print("    x = x0 + t * (x1 - x0)")
    print("    y = y0 + t * (y1 - y0)")
    print("  where t goes from 0.0 (start) to 1.0 (end).")
    print()
    print("  Watch: 12 lines radiate from the center like clock hands.")
    print("  The bright yellow one rotates — same math, changing angle.")
    time.sleep(2)

    start_time = time.ticks_ms()
    angle = 0
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    radius = min(WIDTH, HEIGHT) // 2 - 1

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        panel.clear()

        # Draw lines radiating from center, like a clock hand
        for line_angle_deg in range(0, 360, 30):
            line_angle = math.radians(line_angle_deg + angle)
            end_x = int(center_x + math.cos(line_angle) * radius)
            end_y = int(center_y + math.sin(line_angle) * radius)
            draw_line(center_x, center_y, end_x, end_y, (0, 30, 0))

        # Highlight the "minute hand" in bright color
        highlight_angle = math.radians(angle)
        end_x = int(center_x + math.cos(highlight_angle) * radius)
        end_y = int(center_y + math.sin(highlight_angle) * radius)
        draw_line(center_x, center_y, end_x, end_y, (50, 50, 0))

        panel.show()
        angle += 3
        time.sleep_ms(50)


# ═══════════════════════════════════════════════════════════════════
# STEP 2: Circles
# ═══════════════════════════════════════════════════════════════════

def draw_circle(center_x, center_y, radius, color, filled=False):
    """
    Draw a circle using trigonometry.

    HOW IT WORKS:
    A circle is defined as all points at distance `radius` from the center.
    Using trigonometry, each point on the circle is:
      x = center_x + radius * cos(angle)
      y = center_y + radius * sin(angle)
    where angle goes from 0 to 2*pi (full rotation).

    For a filled circle, we draw horizontal lines (scanlines) at each
    row. At each y, we calculate how wide the circle is using the
    circle equation: x_offset = sqrt(radius^2 - y_offset^2)
    """
    if filled:
        # Filled circle: draw horizontal lines at each row
        for row_offset in range(-radius, radius + 1):
            if center_y + row_offset < 0 or center_y + row_offset >= HEIGHT:
                continue
            # Circle equation: x^2 + y^2 = r^2
            # Solving for x: x = sqrt(r^2 - y^2)
            x_span_squared = radius * radius - row_offset * row_offset
            if x_span_squared < 0:
                continue
            x_span = int(math.sqrt(x_span_squared))
            for col_offset in range(-x_span, x_span + 1):
                panel.pixel(center_x + col_offset, center_y + row_offset, color)
    else:
        # Outline only: plot points around the circumference
        # More points = smoother circle. We use circumference / 1.5
        # as the step count — enough for small radii on LED panels.
        num_points = max(int(2 * math.pi * radius / 1.5), 12)
        for point_index in range(num_points):
            angle = 2 * math.pi * point_index / num_points
            pixel_x = int(center_x + radius * math.cos(angle) + 0.5)
            pixel_y = int(center_y + radius * math.sin(angle) + 0.5)
            panel.pixel(pixel_x, pixel_y, color)


def step2_circles(duration):
    """
    DRAWING CIRCLES — trigonometry in action.

    A circle is just cos() and sin() plotted at every angle.
    Watch how the radius grows and shrinks smoothly.
    """
    print("  A circle is all points at distance R from the center:")
    print("    x = center_x + radius * cos(angle)")
    print("    y = center_y + radius * sin(angle)")
    print()
    print("  For a FILLED circle, we use the circle equation:")
    print("    x² + y² = r²  →  x = sqrt(r² - y²)")
    print("  This tells us how wide the circle is at each row.")
    print()
    print("  Watch: the radius pulses using sin(time) — smooth grow/shrink.")
    time.sleep(2)

    start_time = time.ticks_ms()
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    max_radius = min(WIDTH, HEIGHT) // 2 - 1

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
        panel.clear()

        # Pulsing radius using sine wave (smooth grow/shrink)
        current_radius = int((math.sin(elapsed * 2) + 1) / 2 * max_radius)

        if current_radius > 0:
            # Draw filled circle with color based on size
            hue = (elapsed * 0.2) % 1.0
            red = int((math.sin(hue * 6.28) + 1) / 2 * 40)
            blue = int((math.cos(hue * 6.28) + 1) / 2 * 40)
            draw_circle(center_x, center_y, current_radius, (red, 10, blue), filled=True)

            # Draw outline in brighter color
            draw_circle(center_x, center_y, current_radius, (50, 50, 50), filled=False)

        panel.show()
        time.sleep_ms(40)


# ═══════════════════════════════════════════════════════════════════
# STEP 3: Triangles and rotation
# ═══════════════════════════════════════════════════════════════════

def rotate_point(point_x, point_y, angle, center_x, center_y):
    """
    Rotate a point around a center by the given angle (radians).

    HOW 2D ROTATION WORKS:
    The rotation matrix transforms (x, y) to (x', y'):
      x' = cos(angle) * (x - cx) - sin(angle) * (y - cy) + cx
      y' = sin(angle) * (x - cx) + cos(angle) * (y - cy) + cy

    This comes from the unit circle: if you think of the point as
    being at some angle from the center, rotation just adds to
    that angle. The cos/sin formulas convert between polar and
    Cartesian coordinates.

    The (x - cx) / (y - cy) terms shift the coordinate system so
    the rotation happens around (cx, cy) instead of the origin.
    """
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)

    # Translate to origin, rotate, translate back
    relative_x = point_x - center_x
    relative_y = point_y - center_y

    rotated_x = cos_angle * relative_x - sin_angle * relative_y + center_x
    rotated_y = sin_angle * relative_x + cos_angle * relative_y + center_y

    return rotated_x, rotated_y


def draw_triangle(vertices, color):
    """
    Draw a triangle by connecting three vertices with lines.

    A triangle is the simplest polygon — just 3 lines connecting
    3 points. All 3D graphics ultimately decompose complex shapes
    into triangles (this is why GPUs are "triangle rasterizers").
    """
    for edge_index in range(3):
        start_vertex = vertices[edge_index]
        end_vertex = vertices[(edge_index + 1) % 3]  # Wrap: 0→1, 1→2, 2→0
        draw_line(
            int(start_vertex[0]), int(start_vertex[1]),
            int(end_vertex[0]), int(end_vertex[1]),
            color
        )


def step3_rotating_triangle(duration):
    """
    ROTATION — making shapes move.

    To rotate a shape, we rotate each of its vertices (corner points)
    around a center point, then redraw the edges. This is the
    foundation of all 2D animation and game graphics.

    The rotation matrix:
      x' = cos(a) * x - sin(a) * y
      y' = sin(a) * x + cos(a) * y

    Watch how the triangle spins smoothly. Each frame:
    1. Calculate new vertex positions using rotation
    2. Clear the display
    3. Draw lines between the rotated vertices
    4. Show the frame
    """
    print("  To rotate a point around a center, use the rotation matrix:")
    print("    x' = cos(a) * (x - cx) - sin(a) * (y - cy) + cx")
    print("    y' = sin(a) * (x - cx) + cos(a) * (y - cy) + cy")
    print()
    print("  A triangle is just 3 vertices connected by 3 lines.")
    print("  To rotate it, rotate each vertex, then redraw the edges.")
    print()
    print("  Watch: the yellow dots are the 3 vertices being rotated.")
    print("  The cyan lines are drawn between them each frame.")
    time.sleep(2)

    center_x = WIDTH / 2
    center_y = HEIGHT / 2
    triangle_radius = min(WIDTH, HEIGHT) / 2 - 1

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
        rotation_angle = elapsed * 1.5  # Radians per second

        panel.clear()

        # Generate 3 vertices equally spaced around a circle (120° apart),
        # then rotate them all by the current angle
        vertices = []
        for vertex_index in range(3):
            # Base angle for this vertex (0°, 120°, 240°)
            base_angle = vertex_index * (2 * math.pi / 3)
            # Starting position (unrotated)
            vertex_x = center_x + triangle_radius * math.cos(base_angle)
            vertex_y = center_y + triangle_radius * math.sin(base_angle)
            # Apply rotation
            rotated_x, rotated_y = rotate_point(
                vertex_x, vertex_y, rotation_angle, center_x, center_y
            )
            vertices.append((rotated_x, rotated_y))

        # Draw the triangle edges
        draw_triangle(vertices, (0, 40, 40))

        # Draw a small dot at each vertex to highlight the corners
        for vertex_x, vertex_y in vertices:
            panel.pixel(int(vertex_x), int(vertex_y), (50, 50, 0))

        panel.show()
        time.sleep_ms(40)


# ═══════════════════════════════════════════════════════════════════
# STEP 4: 3D Cube — the classic
# ═══════════════════════════════════════════════════════════════════

def step4_rotating_cube(duration):
    """
    3D WIREFRAME CUBE — from 3D coordinates to 2D screen.

    HOW 3D PROJECTION WORKS:
    A 3D object exists in (x, y, z) space, but your screen is 2D.
    To display 3D objects, we need to "project" them onto the screen.

    The simplest projection is "orthographic" (parallel):
      screen_x = 3d_x
      screen_y = 3d_y
      (z is ignored — no depth perception)

    Better is "perspective" (objects shrink with distance):
      screen_x = 3d_x * focal_length / (3d_z + focal_length)
      screen_y = 3d_y * focal_length / (3d_z + focal_length)

    The division by z makes distant objects smaller — this is
    why railroad tracks appear to converge in the distance.

    ROTATION IN 3D:
    We rotate around two axes (Y and X) using the same cos/sin
    formula as 2D rotation, applied to different pairs of coordinates:
      Rotate around Y axis: affects (x, z)
      Rotate around X axis: affects (y, z)

    The cube has 8 vertices and 12 edges. We rotate all vertices,
    project them to 2D, then draw lines between connected vertices.
    """
    print("  A 3D object has (x, y, z) coordinates, but the screen is 2D.")
    print("  To display it, we PROJECT 3D points onto the screen:")
    print()
    print("    screen_x = x * focal_length / (z + focal_length)")
    print("    screen_y = y * focal_length / (z + focal_length)")
    print()
    print("  The division by z makes distant objects smaller — this is")
    print("  perspective. Same reason railroad tracks look like they converge.")
    print()
    print("  The cube has 8 vertices and 12 edges. Each frame:")
    print("  1. Rotate all 8 vertices in 3D (around Y and X axes)")
    print("  2. Project each 3D point to 2D screen coordinates")
    print("  3. Draw lines between connected vertices")
    print()
    print("  Watch: brighter edges are closer, dimmer edges are further away.")
    time.sleep(3)

    # Define the 8 vertices of a unit cube centered at origin.
    # Each vertex is at (+/-1, +/-1, +/-1).
    cube_vertices = [
        (-1, -1, -1), ( 1, -1, -1), ( 1,  1, -1), (-1,  1, -1),  # Back face
        (-1, -1,  1), ( 1, -1,  1), ( 1,  1,  1), (-1,  1,  1),  # Front face
    ]

    # Define the 12 edges as pairs of vertex indices.
    # 4 edges on back face + 4 on front face + 4 connecting them.
    cube_edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Back face
        (4, 5), (5, 6), (6, 7), (7, 4),  # Front face
        (0, 4), (1, 5), (2, 6), (3, 7),  # Connecting edges
    ]

    # Display center and scale
    center_x = WIDTH / 2
    center_y = HEIGHT / 2
    scale = min(WIDTH, HEIGHT) / 4  # How big the cube appears
    focal_length = 4.0  # Controls perspective strength (higher = less distortion)

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000

        # Rotation angles (different speeds for each axis = tumbling)
        angle_y = elapsed * 1.2  # Rotation around vertical axis
        angle_x = elapsed * 0.8  # Rotation around horizontal axis

        # Pre-compute sin/cos (used for every vertex)
        cos_y = math.cos(angle_y)
        sin_y = math.sin(angle_y)
        cos_x = math.cos(angle_x)
        sin_x = math.sin(angle_x)

        # Transform each 3D vertex to 2D screen coordinates
        projected_vertices = []
        for vertex_x, vertex_y, vertex_z in cube_vertices:
            # Step 1: Rotate around Y axis (affects x and z)
            rotated_x = cos_y * vertex_x - sin_y * vertex_z
            rotated_z = sin_y * vertex_x + cos_y * vertex_z

            # Step 2: Rotate around X axis (affects y and z)
            rotated_y = cos_x * vertex_y - sin_x * rotated_z
            final_z   = sin_x * vertex_y + cos_x * rotated_z

            # Step 3: Perspective projection
            # Objects further away (larger z) appear smaller.
            # We add focal_length to z to prevent division by zero
            # and to control how strong the perspective effect is.
            perspective_scale = focal_length / (final_z + focal_length)
            screen_x = int(center_x + rotated_x * scale * perspective_scale)
            screen_y = int(center_y + rotated_y * scale * perspective_scale)

            projected_vertices.append((screen_x, screen_y))

        # Draw the wireframe: connect projected vertices with lines
        panel.clear()
        for start_index, end_index in cube_edges:
            start_point = projected_vertices[start_index]
            end_point = projected_vertices[end_index]

            # Color edges by depth (front = bright, back = dim)
            # This gives a simple depth cue without full z-buffering
            avg_depth = (cube_vertices[start_index][2] + cube_vertices[end_index][2]) / 2
            edge_brightness = int(30 + avg_depth * 10)
            draw_line(
                start_point[0], start_point[1],
                end_point[0], end_point[1],
                (0, edge_brightness, edge_brightness)
            )

        panel.show()
        time.sleep_ms(40)


# ═══════════════════════════════════════════════════════════════════
# STEP 5: Combining shapes — animated scene
# ═══════════════════════════════════════════════════════════════════

def step5_combined_scene(duration):
    """
    PUTTING IT ALL TOGETHER — a scene with multiple animated shapes.

    Real graphics combine many primitives:
    - Background (filled rectangle)
    - Bouncing circle (position + velocity)
    - Rotating triangle (rotation matrix)
    - Static decorations (individual pixels)

    Each frame follows the same pattern:
    1. Clear the framebuffer
    2. Draw all shapes into the buffer
    3. Call show() once

    This is exactly how game engines work — just at higher
    resolution and with hardware acceleration.
    """
    print("  Real graphics combine many primitives in one frame.")
    print("  This scene has:")
    print("    - A border (individual pixels)")
    print("    - A rotating triangle (rotation matrix + line drawing)")
    print("    - A bouncing ball (position + velocity + collision)")
    print()
    print("  Every frame follows the 'game loop' pattern:")
    print("    1. Update physics (move ball, check collisions)")
    print("    2. Clear the framebuffer")
    print("    3. Draw ALL shapes into the buffer")
    print("    4. Call show() ONCE")
    print()
    print("  This is exactly how game engines work!")
    time.sleep(2)

    # Bouncing ball state
    ball_x = WIDTH / 4
    ball_y = HEIGHT / 2
    ball_velocity_x = 0.15
    ball_velocity_y = 0.1
    ball_radius = 2

    start_time = time.ticks_ms()
    center_x = WIDTH / 2
    center_y = HEIGHT / 2

    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000

        # --- Update physics ---

        # Move the ball
        ball_x += ball_velocity_x
        ball_y += ball_velocity_y

        # Bounce off edges
        if ball_x <= ball_radius or ball_x >= WIDTH - 1 - ball_radius:
            ball_velocity_x = -ball_velocity_x
            ball_x = max(ball_radius, min(WIDTH - 1 - ball_radius, ball_x))
        if ball_y <= ball_radius or ball_y >= HEIGHT - 1 - ball_radius:
            ball_velocity_y = -ball_velocity_y
            ball_y = max(ball_radius, min(HEIGHT - 1 - ball_radius, ball_y))

        # --- Draw frame ---
        panel.clear()

        # Draw border
        for border_x in range(WIDTH):
            panel.pixel(border_x, 0, (10, 10, 10))
            panel.pixel(border_x, HEIGHT - 1, (10, 10, 10))
        for border_y in range(HEIGHT):
            panel.pixel(0, border_y, (10, 10, 10))
            panel.pixel(WIDTH - 1, border_y, (10, 10, 10))

        # Draw rotating triangle
        triangle_angle = elapsed * 2
        triangle_radius = min(WIDTH, HEIGHT) / 4
        vertices = []
        for vertex_index in range(3):
            base_angle = vertex_index * (2 * math.pi / 3)
            vertex_x = center_x + triangle_radius * math.cos(base_angle + triangle_angle)
            vertex_y = center_y + triangle_radius * math.sin(base_angle + triangle_angle)
            vertices.append((vertex_x, vertex_y))
        draw_triangle(vertices, (0, 0, 25))

        # Draw bouncing ball (on top of triangle)
        draw_circle(int(ball_x), int(ball_y), ball_radius, (40, 0, 0), filled=True)

        panel.show()
        time.sleep_ms(30)


# ═══════════════════════════════════════════════════════════════════
# RUN ALL STEPS
# ═══════════════════════════════════════════════════════════════════

print(f"Graphics Tutorial: {WIDTH}x{HEIGHT} on pin {PIN}")
print("Each step teaches a new concept. Ctrl-C to skip.\n")

try:
    run_step("Step 0: Immediate vs Framebuffer", step0_immediate_vs_framebuffer, 15)
    run_step("Step 1: Drawing Lines", step1_lines)
    run_step("Step 2: Drawing Circles", step2_circles)
    run_step("Step 3: Rotating Triangle (2D rotation)", step3_rotating_triangle)
    run_step("Step 4: Rotating Cube (3D projection)", step4_rotating_cube, 12)
    run_step("Step 5: Combined Scene", step5_combined_scene, 10)

    print("\n" + "="*50)
    print("  Tutorial complete!")
    print("  You now know: lines, circles, rotation, 3D projection,")
    print("  and why framebuffers exist.")
    print("="*50)

except KeyboardInterrupt:
    print("\nExited.")

panel.clear()
panel.show()
