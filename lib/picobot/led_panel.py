"""
WS2812 LED Panel — framebuffer-based NeoPixel driver for strips and matrices.

HOW WS2812 LEDs WORK
--------------------
WS2812 (NeoPixel) LEDs are "addressable" — each LED has a tiny chip inside
that receives 24 bits of color data (8 bits each for Green, Red, Blue in
that order), then passes the remaining data downstream to the next LED.

The protocol is timing-critical: a '1' bit is a long HIGH pulse followed
by a short LOW, a '0' bit is a short HIGH followed by a long LOW. The
exact timing (measured in nanoseconds) must be precise or LEDs misinterpret
the data. This is why we use the RP2350's PIO (Programmable I/O) hardware
instead of software bit-banging — PIO guarantees exact cycle timing
regardless of what the CPU is doing.

FRAMEBUFFER PATTERN
-------------------
Instead of sending data to the LEDs every time you change a pixel
(which would be slow and cause flicker), we use a "framebuffer" pattern:

  1. Write pixel colors into an in-memory buffer (fast, no I/O)
  2. When the full frame is ready, call show() to push it all at once

This is the same pattern used by game engines, OLED displays, and
professional LED controllers. It gives consistent frame timing and
prevents partial updates from being visible.

CHAINED PANELS
--------------
Multiple 8x8 panels can be daisy-chained (DOUT of panel 1 → DIN of
panel 2). The data flows through each panel completely before reaching
the next one. So for two 8x8 panels:

  Panel 1 (indices 0-63)        Panel 2 (indices 64-127)
  row 0: [0  1  2  ... 7]       row 0: [64 65 66 ... 71]
  row 1: [8  9  10 ... 15]      row 1: [72 73 74 ... 79]
  ...                            ...

The _xy_to_index() method handles this mapping so you can address
the combined display with simple (x, y) coordinates.

Usage:
    from picobot import LEDPanel

    # Simple strip (30 LEDs on pin 2)
    strip = LEDPanel(pin=2, num_leds=30)
    strip[0] = (255, 0, 0)       # Red — first LED
    strip[10] = (0, 255, 0)      # Green — eleventh LED
    strip.show()                  # Push to hardware

    # Single 8x8 matrix
    panel = LEDPanel(pin=2, num_leds=64, width=8)
    panel.pixel(3, 4, (0, 0, 255))  # Blue at column 3, row 4
    panel.show()

    # Two chained 8x8 panels as one 16x8 display
    big = LEDPanel(pin=2, num_leds=128, width=16, panel_width=8)
    big.pixel(0, 0, (20, 0, 0))     # Top-left of panel 1
    big.pixel(8, 0, (0, 20, 0))     # Top-left of panel 2
    big.show()
"""

import array
import time
from machine import Pin, disable_irq, enable_irq
import rp2


# ═══════════════════════════════════════════════════════════════════════════
# PIO PROGRAM — WS2812B bit-level protocol
# ═══════════════════════════════════════════════════════════════════════════
#
# The RP2350 has PIO (Programmable I/O) — tiny state machines that run
# independently of the CPU. We program one to generate the exact pulse
# timing that WS2812 LEDs require.
#
# At 8 MHz, each PIO cycle = 125 ns.
# One bit takes T1 + T2 + T3 = 2 + 5 + 3 = 10 cycles = 1250 ns
#
#   '1' bit: HIGH for T1+T2 = 875 ns, then LOW for T3 = 375 ns
#   '0' bit: HIGH for T1    = 250 ns, then LOW for T2+T3 = 1000 ns
#
# The [T3-1] etc. are delay cycles — PIO instructions take 1 cycle
# by default, so we add extra cycles to reach the target timing.
#
# "sideset" controls the output pin as a side-effect of each instruction,
# so we can change the pin state while also doing logic (out, jmp, nop).
#
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,    # Pin starts LOW
    out_shiftdir=rp2.PIO.SHIFT_LEFT, # Send MSB first (WS2812 requirement)
    autopull=True,                    # Auto-load next word when shift reg empty
    pull_thresh=24,                   # Each pixel = 24 bits (GRB)
)
def _ws2812():
    T1 = 2   # Initial HIGH time (both '0' and '1' bits)
    T2 = 5   # Extended HIGH for '1', or LOW start for '0'
    T3 = 3   # Final LOW time
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]  # Shift 1 bit into X, pin LOW
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]  # Pin HIGH; if bit=0, jump
    jmp("bitloop")           .side(1)    [T2 - 1]  # Bit=1: stay HIGH longer
    label("do_zero")
    nop()                    .side(0)    [T2 - 1]  # Bit=0: go LOW early
    wrap()


# ═══════════════════════════════════════════════════════════════════════════
# LED PANEL CLASS
# ═══════════════════════════════════════════════════════════════════════════

class LEDPanel:
    """
    Framebuffer-based WS2812 driver for LED strips and 2D matrix panels.

    Design decisions:
    - Framebuffer + explicit show(): avoids per-pixel I/O, gives consistent
      frame timing, and lets you compose a full frame before displaying it.
    - Brightness is applied at write time (in __setitem__ and fill), not
      stored separately. This keeps the buffer compact (one 32-bit word
      per pixel) and avoids floating-point math during show().
    - PIO state machine runs independently of CPU, so show() is fast
      and timing-accurate regardless of Python overhead.
    """

    def __init__(self, pin, num_leds, width=None, sm_id=1, brightness=255,
                 layout='progressive', panel_width=None, rotation=0):
        """
        Initialize the LED panel driver.

        Args:
            pin:         GPIO pin number connected to DIN (data input)
            num_leds:    Total number of LEDs in the entire chain
            width:       Total display width in pixels.
                         For a 1D strip, leave as None (defaults to num_leds).
                         For a matrix, set to the number of columns.
            sm_id:       PIO state machine ID (0-7). Default 1 to avoid
                         conflict with the robot's built-in LEDStrip (SM 0).
                         Each PIO SM is independent — you can run multiple
                         LED chains on different pins simultaneously.
            brightness:  Global brightness limit 0-255. Applied when writing
                         pixels, not during show(). This lets you cap power
                         draw without changing your color values.
                         At brightness=40, (255,0,0) becomes (40,0,0).
            layout:      How LEDs are wired within each panel:
                         'progressive' — all rows left-to-right (default)
                         'serpentine'  — even rows L→R, odd rows R→L
                         (zigzag wiring, common on flexible LED matrices)
            panel_width: Width of each physical panel in the chain.
                         Only needed when chaining multiple panels.
                         Example: two 8×8 panels as 16×8 display:
                           width=16, panel_width=8
                         The driver maps (x, y) coordinates across the
                         combined display to the correct linear index.
            rotation:    Rotate the virtual display coordinate system.
                         0   = no rotation (default)
                         90  = 90° clockwise (virtual x,y → physical y, W-1-x)
                         180 = 180° (upside down)
                         270 = 90° counter-clockwise
                         With rotation=90, a 16×8 physical panel becomes
                         an 8×16 virtual display (portrait mode). The width
                         and height properties return the virtual dimensions.

        Example configurations:
            LEDPanel(pin=2, num_leds=30)
                → 30-LED strip, 1D addressing only

            LEDPanel(pin=2, num_leds=64, width=8)
                → Single 8×8 panel, pixel(x, y) addressing

            LEDPanel(pin=2, num_leds=128, width=16, panel_width=8)
                → Two chained 8×8 panels as one 16×8 display

            LEDPanel(pin=2, num_leds=128, width=16, panel_width=8, rotation=90)
                → Two 8×8 panels as 8×16 portrait display
        """
        self._pin = pin
        self._count = num_leds
        self._phys_width = width or num_leds
        self._layout = layout
        self._panel_width = panel_width or self._phys_width
        self._brightness = brightness
        self._rotation = rotation

        # Calculate panel geometry for chained panel support.
        # _leds_per_panel: how many LEDs in one physical panel.
        # For a single panel, this equals num_leds.
        # For chained panels: total LEDs / number of panels.
        num_panels = self._phys_width // self._panel_width
        self._leds_per_panel = num_leds // num_panels if num_panels > 1 else num_leds
        self._panel_height = self._leds_per_panel // self._panel_width
        self._phys_height = self._panel_height

        # Virtual dimensions after rotation.
        # 90° and 270° swap width and height.
        if rotation in (90, 270):
            self._width = self._phys_height
            self._height = self._phys_width
        else:
            self._width = self._phys_width
            self._height = self._phys_height

        # Framebuffer: one 32-bit word per LED, packed as GRB.
        # Using array.array("I", ...) gives us a flat C-level array
        # that PIO can DMA directly — much faster than a Python list.
        self._buf = array.array("I", [0] * num_leds)

        # Initialize PIO state machine for this pin.
        # freq=8_000_000 gives 125ns per PIO cycle, which produces
        # the exact WS2812 timing (1250ns per bit = 800 kHz data rate).
        self._sm = rp2.StateMachine(
            sm_id, _ws2812,
            freq=8_000_000,
            sideset_base=Pin(pin),
        )
        self._sm.active(1)
        self.show()  # Clear all LEDs on startup

    # ═══════════════════════════════════════════════════════════════════
    # FRAMEBUFFER WRITES — fast, no hardware I/O
    # ═══════════════════════════════════════════════════════════════════
    #
    # These methods modify the in-memory buffer only. Nothing is sent
    # to the LEDs until you call show(). This lets you build up a
    # complete frame without flicker.

    def __setitem__(self, index, color):
        """
        Set LED by linear index: panel[i] = (r, g, b)

        The color is scaled by the global brightness setting and packed
        into a single 32-bit word in GRB order (what WS2812 expects).

        The bit layout of each 32-bit word:
          bits 23-16: Green (8 bits)
          bits 15-8:  Red   (8 bits)
          bits  7-0:  Blue  (8 bits)
        """
        if 0 <= index < self._count:
            r, g, b = color
            # Apply brightness scaling using integer math (no floats).
            # brightness=255 means full brightness (no change).
            # brightness=128 halves all values. brightness=0 = off.
            r = r * self._brightness // 255
            g = g * self._brightness // 255
            b = b * self._brightness // 255
            # Pack into GRB format (WS2812 expects Green first!)
            self._buf[index] = (g << 16) | (r << 8) | b

    def __getitem__(self, index):
        """
        Read back pixel color as (r, g, b).

        Note: returns the brightness-scaled value (what's actually in
        the buffer), not the original color you set.
        """
        if 0 <= index < self._count:
            w = self._buf[index]
            g = (w >> 16) & 0xFF
            r = (w >> 8) & 0xFF
            b = w & 0xFF
            return (r, g, b)
        return (0, 0, 0)

    def _rotate(self, x, y):
        """
        Transform virtual (x, y) to physical coordinates based on rotation.

        Rotation is applied BEFORE the panel index mapping. This lets you
        work in a rotated coordinate system (e.g. portrait mode on a
        landscape panel) without manually swapping coordinates everywhere.

          rotation=0:   physical = (x, y)                — no change
          rotation=90:  physical = (y, phys_height-1-x)  — 90° clockwise
          rotation=180: physical = (phys_width-1-x, phys_height-1-y)
          rotation=270: physical = (phys_width-1-y, x)   — 90° counter-clockwise
        """
        if self._rotation == 90:
            return (y, self._phys_height - 1 - x)
        elif self._rotation == 180:
            return (self._phys_width - 1 - x, self._phys_height - 1 - y)
        elif self._rotation == 270:
            return (self._phys_width - 1 - y, x)
        return (x, y)

    def pixel(self, x, y, color):
        """
        Set pixel by 2D coordinate (in virtual/rotated space).

        Args:
            x:     Column (0 = left edge)
            y:     Row (0 = top edge)
            color: (R, G, B) tuple, each 0-255

        Coordinates outside the display are silently ignored.
        If rotation is set, coordinates are transformed automatically.
        """
        if 0 <= x < self._width and 0 <= y < self._height:
            px, py = self._rotate(x, y)
            self[self._xy_to_index(px, py)] = color

    def get_pixel(self, x, y):
        """Read pixel color at 2D coordinate (in virtual/rotated space)."""
        if 0 <= x < self._width and 0 <= y < self._height:
            px, py = self._rotate(x, y)
            return self[self._xy_to_index(px, py)]
        return (0, 0, 0)

    def _xy_to_index(self, x, y):
        """
        Convert (x, y) screen coordinate to linear index in the LED chain.

        This is the key mapping that makes chained panels work as one display.

        For a single panel (panel_width == width), it's simple:
          index = y * width + x

        For chained panels, the data fills one panel completely (all rows)
        before continuing to the next panel. So the first panel owns
        indices 0 to leds_per_panel-1, the second owns leds_per_panel to
        2*leds_per_panel-1, etc.

        Example: two 8×8 panels, pixel(10, 3):
          panel = 10 // 8 = 1  (second panel)
          lx    = 10 % 8  = 2  (column 2 within that panel)
          index = 64 + 3*8 + 2 = 90

        Serpentine layout (zigzag wiring):
          Even rows: left to right (index increases with x)
          Odd rows:  right to left (index decreases with x)
          This is common on flexible LED matrices where the data line
          snakes back and forth to minimize wiring.
        """
        # Which physical panel does this x coordinate land on?
        panel = x // self._panel_width
        # X position within that panel (local x)
        lx = x % self._panel_width

        # Starting index of this panel's data in the chain
        panel_offset = panel * self._leds_per_panel

        # Handle serpentine (zigzag) wiring within the panel
        if self._layout == 'serpentine' and y % 2 == 1:
            # Odd rows are wired right-to-left
            return panel_offset + y * self._panel_width + (self._panel_width - 1 - lx)

        # Progressive (normal) wiring: left-to-right
        return panel_offset + y * self._panel_width + lx

    # ═══════════════════════════════════════════════════════════════════
    # DRAWING PRIMITIVES
    # ═══════════════════════════════════════════════════════════════════

    def fill(self, color):
        """
        Fill the entire display with one color.

        Faster than setting pixels individually because it pre-computes
        the packed GRB value once and writes it to every buffer position.
        """
        r, g, b = color
        r = r * self._brightness // 255
        g = g * self._brightness // 255
        b = b * self._brightness // 255
        w = (g << 16) | (r << 8) | b
        for i in range(self._count):
            self._buf[i] = w

    def clear(self):
        """Set all pixels to black (off). Call show() after to display."""
        for i in range(self._count):
            self._buf[i] = 0

    def fill_rect(self, x, y, w, h, color):
        """
        Fill a rectangle.

        Args:
            x, y:  Top-left corner
            w, h:  Width and height in pixels
            color: (R, G, B) tuple
        """
        for dy in range(h):
            for dx in range(w):
                self.pixel(x + dx, y + dy, color)

    def hline(self, x, y, length, color):
        """Draw a horizontal line starting at (x, y)."""
        for dx in range(length):
            self.pixel(x + dx, y, color)

    def vline(self, x, y, length, color):
        """Draw a vertical line starting at (x, y)."""
        for dy in range(length):
            self.pixel(x, y + dy, color)

    # ═══════════════════════════════════════════════════════════════════
    # HARDWARE OUTPUT
    # ═══════════════════════════════════════════════════════════════════

    def show(self):
        """
        Push the framebuffer to the LED chain.

        This sends all pixel data via PIO. The entire chain is updated
        every time — WS2812 has no way to update individual LEDs.

        Interrupts are briefly disabled during the transfer to prevent
        timing glitches. The 300µs sleep at the end is the WS2812
        "reset" signal — a long LOW that tells the first LED "the next
        data is for you again" (rather than passing it downstream).
        """
        irq_state = disable_irq()
        self._sm.put(self._buf, 8)  # 8 = bit shift for 24-bit data in 32-bit words
        enable_irq(irq_state)
        time.sleep_us(300)  # WS2812 reset pulse (>280µs required)

    # ═══════════════════════════════════════════════════════════════════
    # PROPERTIES
    # ═══════════════════════════════════════════════════════════════════

    @property
    def width(self):
        """Total display width in pixels (across all panels)."""
        return self._width

    @property
    def height(self):
        """Display height in pixels."""
        return self._height

    @property
    def count(self):
        """Total number of LEDs in the chain."""
        return self._count

    @property
    def brightness(self):
        """Global brightness limit (0-255)."""
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        """
        Set global brightness. Takes effect on the next pixel write
        or fill() call — already-written buffer contents are not changed.
        Call fill() or re-draw after changing brightness to see the effect.
        """
        self._brightness = max(0, min(255, value))

    def deinit(self):
        """
        Release the PIO state machine.

        Call this if you want to reuse the PIO SM for something else,
        or when switching between different LED configurations.
        """
        self._sm.active(0)


# ═══════════════════════════════════════════════════════════════════════════
# DIAMOND PANEL — two 8×8 panels physically rotated 45°
# ═══════════════════════════════════════════════════════════════════════════

class DiamondPanel:
    """
    Two 8×8 WS2812 panels mounted at 45° to form a diamond/hourglass.

    Physical mounting (top view):
        ◇◇        Each 8×8 panel is rotated 45° so its diagonal
       ◇◇◇◇       is vertical. Two panels touching at tips form
      ◇◇◇◇◇◇      an hourglass / double-diamond shape.
     ◇◇◇◇◇◇◇◇
      ◇◇◇◇◇◇
       ◇◇◇◇
        ◇◇

    COORDINATE MAPPING
    ------------------
    When a square grid is rotated 45°, the pixel at physical (row, col)
    appears at a different position. The mapping uses the standard
    isometric transform:

      virtual_x = col - row   (diagonal axis)
      virtual_y = col + row   (other diagonal axis)

    This creates a diamond-shaped coordinate space where:
    - The virtual grid has gaps (checkerboard pattern — not every
      integer coordinate has a physical pixel)
    - The virtual dimensions are ~15×15 for an 8×8 panel

    For two panels joined at tips (hourglass), the second panel's
    coordinates are offset so the bottom tip of panel 1 meets the
    top tip of panel 2.

    Usage:
        from picobot.led_panel import DiamondPanel

        dp = DiamondPanel(pin=6, sm_id=1, brightness=60)
        dp.pixel(7, 0, (50, 0, 0))   # Top tip
        dp.pixel(7, 14, (0, 50, 0))  # Center (where diamonds meet)
        dp.pixel(7, 28, (0, 0, 50))  # Bottom tip
        dp.fill_diamond((10, 10, 10))
        dp.show()
    """

    def __init__(self, pin, panel_size=8, sm_id=1, brightness=255,
                 layout='progressive'):
        """
        Args:
            pin:        GPIO pin connected to DIN
            panel_size: Size of each square panel (default 8 for 8×8)
            sm_id:      PIO state machine ID
            brightness: Global brightness 0-255
            layout:     'progressive' or 'serpentine' within each panel
        """
        self._panel_size = panel_size
        self._brightness = brightness
        num_leds = panel_size * panel_size * 2  # Two panels

        # The underlying LEDPanel handles the raw pixel buffer and PIO
        self._raw = LEDPanel(
            pin=pin, num_leds=num_leds, width=panel_size * 2,
            panel_width=panel_size, sm_id=sm_id, brightness=brightness,
            layout=layout,
        )

        # Build the mapping tables.
        # For each physical (panel, row, col) → virtual (vx, vy)
        # and reverse: virtual (vx, vy) → physical linear index.
        #
        # Panel 0 (top diamond): tip at top, widest in middle
        # Panel 1 (bottom diamond): widest at top, tip at bottom
        # They share a row at the junction.

        size = panel_size
        # Virtual width = 2*size - 1 (the diagonal of one panel)
        self._vwidth = 2 * size - 1
        # Virtual height = two full diamonds stacked with no overlap.
        # Each diamond is (2*size - 1) rows tall. No shared row.
        self._vheight = 2 * (2 * size - 1)

        # Build reverse lookup: (vx, vy) → linear buffer index
        # Only valid diamond positions have entries
        self._virt_to_raw = {}
        self._valid_cells = set()

        for panel_id in range(2):
            for row in range(size):
                for col in range(size):
                    # Isometric transform within one panel.
                    # Rotating a square grid 45° maps (row, col) to:
                    #   vx = (col - row)  along one diagonal
                    #   vy = (col + row)  along the other diagonal
                    # We add (size-1) to vx to center horizontally.
                    local_vx = (col - row) + (size - 1)
                    local_vy = col + row

                    # Panel 1 is placed directly below panel 0.
                    # The bottom tip of panel 0 is at vy = 2*(size-1).
                    # Panel 1's top tip starts one row below that — no shared pixel.
                    if panel_id == 0:
                        vx = local_vx
                        vy = local_vy
                    else:
                        vx = local_vx
                        vy = local_vy + (2 * size - 1)  # Gap of 1 row

                    # Physical linear index in the LED chain
                    raw_index = panel_id * (size * size) + row * size + col

                    self._virt_to_raw[(vx, vy)] = raw_index
                    self._valid_cells.add((vx, vy))

    def pixel(self, x, y, color):
        """Set pixel in virtual diamond coordinates."""
        key = (x, y)
        if key in self._virt_to_raw:
            self._raw[self._virt_to_raw[key]] = color

    def get_pixel(self, x, y):
        """Read pixel from virtual diamond coordinates."""
        key = (x, y)
        if key in self._virt_to_raw:
            return self._raw[self._virt_to_raw[key]]
        return (0, 0, 0)

    def is_valid(self, x, y):
        """Check if (x, y) is a valid diamond pixel position."""
        return (x, y) in self._valid_cells

    def clear(self):
        """Clear all pixels."""
        self._raw.clear()

    def fill_diamond(self, color):
        """Fill all valid diamond pixels with one color."""
        for vx, vy in self._valid_cells:
            self._raw[self._virt_to_raw[(vx, vy)]] = color

    def show(self):
        """Push framebuffer to LEDs."""
        self._raw.show()

    @property
    def width(self):
        """Virtual display width."""
        return self._vwidth

    @property
    def height(self):
        """Virtual display height."""
        return self._vheight

    @property
    def valid_cells(self):
        """Set of all valid (x, y) positions in the diamond."""
        return self._valid_cells

    @property
    def cell_to_index(self):
        """Dict mapping (vx, vy) → raw buffer index."""
        return self._virt_to_raw

    @property
    def brightness(self):
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        self._brightness = max(0, min(255, value))
        self._raw.brightness = self._brightness

    def deinit(self):
        """Release PIO state machine."""
        self._raw.deinit()
