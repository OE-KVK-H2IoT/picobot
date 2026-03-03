"""
LED control module - NeoPixel RGB LEDs via PIO.

Uses PIO state machine for deterministic WS2812B timing.
The built-in neopixel module uses machine.bitstream() (CPU bit-banging)
which has unreliable timing on RP2350 (Pico 2) due to Cortex-M33
pipeline differences and XIP cache jitter.
"""

import array
import time
from machine import Pin, disable_irq, enable_irq
import rp2
from .config import PINS


# --- PIO program for WS2812B protocol ---
# At 8 MHz each cycle = 125 ns.
# Bit period = T1+T2+T3 = 2+5+3 = 10 cycles = 1250 ns
#   '1' bit: HIGH for T1+T2 = 875 ns, LOW for T3 = 375 ns
#   '0' bit: HIGH for T1    = 250 ns, LOW for T2+T3 = 1000 ns
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24,
)
def _ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")           .side(1)    [T2 - 1]
    label("do_zero")
    nop()                    .side(0)    [T2 - 1]
    wrap()


class LEDStrip:
    """
    WS2812 NeoPixel LED strip controller using PIO.

    Access via robot.leds
    """

    # Predefined colors (dim to save power)
    OFF = (0, 0, 0)
    RED = (50, 0, 0)
    GREEN = (0, 50, 0)
    BLUE = (0, 0, 50)
    YELLOW = (50, 50, 0)
    CYAN = (0, 50, 50)
    MAGENTA = (50, 0, 50)
    WHITE = (30, 30, 30)
    ORANGE = (50, 25, 0)

    def __init__(self):
        """Initialize LED strip with PIO-based driver."""
        self._count = PINS.NEOPIXEL_COUNT
        # Pre-packed GRB pixel data (32-bit words)
        self._pixels = array.array("I", [0] * self._count)
        self._sm = rp2.StateMachine(
            0, _ws2812,
            freq=8_000_000,
            sideset_base=Pin(PINS.NEOPIXEL),
        )
        self._sm.active(1)
        self.off()

    def _write(self):
        """Push pixel data to strip via PIO with IRQs disabled."""
        irq_state = disable_irq()
        self._sm.put(self._pixels, 8)
        enable_irq(irq_state)
        time.sleep_us(300)

    def _pack(self, index, color):
        """Pack (R,G,B) tuple into GRB 32-bit word at index."""
        r, g, b = color
        self._pixels[index] = (g << 16) | (r << 8) | b

    def set(self, index, color):
        """
        Set a single LED.

        Args:
            index: LED index (0 to count-1)
            color: (R, G, B) tuple (0-255 each)
        """
        if 0 <= index < self._count:
            self._pack(index, color)
            self._write()

    def set_all(self, colors):
        """
        Set all LEDs.

        Args:
            colors: List of (R, G, B) tuples
        """
        for i, color in enumerate(colors):
            if i < self._count:
                self._pack(i, color)
        self._write()

    def fill(self, color):
        """
        Fill all LEDs with one color.

        Args:
            color: (R, G, B) tuple
        """
        for i in range(self._count):
            self._pack(i, color)
        self._write()

    def off(self):
        """Turn off all LEDs."""
        self.fill(self.OFF)

    def show_state(self, state_name):
        """
        Show robot state with predefined color.

        Args:
            state_name: One of IDLE, FOLLOW, TURNING, etc.
        """
        state_colors = {
            'IDLE': self.YELLOW,
            'FOLLOW': self.GREEN,
            'FOLLOW_LINE': self.GREEN,
            'TURNING': self.MAGENTA,
            'TURN': self.MAGENTA,
            'JUNCTION': self.BLUE,
            'AT_JUNCTION': self.BLUE,
            'OBSTACLE': self.ORANGE,
            'AVOIDING': self.ORANGE,
            'DELIVERING': self.CYAN,
            'RETURNING': self.GREEN,
            'ERROR': self.RED,
            'LOST': self.RED,
        }

        color = state_colors.get(state_name.upper(), self.WHITE)
        self.fill(color)

    def show_error(self, error):
        """
        Visualize line following error on LEDs.

        Args:
            error: -1.5 to +1.5 (from line sensors)
        """
        if error is None:
            self.fill(self.RED)
            return

        # Map error to LED position
        # error -1.5 = leftmost LEDs, +1.5 = rightmost LEDs
        center = self._count / 2
        position = center + (error * center / 1.5)

        for i in range(self._count):
            distance = abs(i - position)
            if distance < 1:
                self._pack(i, self.GREEN)
            elif distance < 2:
                self._pack(i, (0, 20, 0))
            else:
                self._pack(i, self.OFF)

        self._write()

    def show_distance(self, distance_cm):
        """
        Visualize distance on LEDs.

        Args:
            distance_cm: Distance in cm
        """
        if distance_cm < 0:
            self.fill(self.MAGENTA)  # Error
            return

        # Map distance to number of lit LEDs
        # 0-10cm = 1 LED (red), 10-80cm = proportional (yellow to green)
        if distance_cm < 10:
            lit = 1
            color = self.RED
        elif distance_cm < 80:
            lit = int((distance_cm - 10) / 70 * self._count)
            if distance_cm < 30:
                color = self.ORANGE
            else:
                color = self.GREEN
        else:
            lit = self._count
            color = self.GREEN

        for i in range(self._count):
            if i < lit:
                self._pack(i, color)
            else:
                self._pack(i, self.OFF)

        self._write()

    @property
    def count(self):
        """Number of LEDs in strip."""
        return self._count
