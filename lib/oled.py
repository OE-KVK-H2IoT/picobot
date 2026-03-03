"""
Simple SSD1306 OLED Display Driver for MicroPython
128x32 I2C OLED display with large text support
"""

from machine import I2C
import framebuf


class SSD1306:
    """SSD1306 OLED display driver."""

    def __init__(self, i2c, width=128, height=32, addr=0x3C):
        self.i2c = i2c
        self.width = width
        self.height = height
        self.addr = addr
        self.pages = height // 8

        # Frame buffer
        self.buffer = bytearray(self.pages * width)
        self.fb = framebuf.FrameBuffer(self.buffer, width, height, framebuf.MONO_VLSB)

        self._init_display()

    def _init_display(self):
        """Initialize display."""
        init_cmds = [
            0xAE,  # Display off
            0xD5, 0x80,  # Clock divide
            0xA8, self.height - 1,  # Multiplex ratio
            0xD3, 0x00,  # Display offset
            0x40,  # Start line
            0x8D, 0x14,  # Charge pump
            0x20, 0x00,  # Memory mode
            0xA1,  # Segment remap
            0xC8,  # COM scan direction
            0xDA, 0x02 if self.height == 32 else 0x12,  # COM pins config
            0x81, 0xCF,  # Contrast
            0xD9, 0xF1,  # Pre-charge
            0xDB, 0x40,  # VCOMH
            0xA4,  # Display from RAM
            0xA6,  # Normal display
            0xAF,  # Display on
        ]
        for cmd in init_cmds:
            self._cmd(cmd)

    def _cmd(self, cmd):
        try:
            self.i2c.writeto(self.addr, bytes([0x00, cmd]))
        except OSError:
            pass  # Ignore I2C timeout (hard IRQ interference)

    def show(self):
        """Update display."""
        try:
            self._cmd(0x21); self._cmd(0); self._cmd(self.width - 1)
            self._cmd(0x22); self._cmd(0); self._cmd(self.pages - 1)
            self.i2c.writeto(self.addr, bytes([0x40]) + self.buffer)
        except OSError:
            pass  # Ignore I2C timeout (hard IRQ interference)

    def fill(self, c):
        self.fb.fill(c)

    def clear(self):
        self.fill(0)
        self.show()

    def text(self, txt, x, y, c=1):
        self.fb.text(txt, x, y, c)

    def pixel(self, x, y, c=1):
        self.fb.pixel(x, y, c)

    def hline(self, x, y, w, c=1):
        self.fb.hline(x, y, w, c)

    def rect(self, x, y, w, h, c=1):
        self.fb.rect(x, y, w, h, c)

    def fill_rect(self, x, y, w, h, c=1):
        self.fb.fill_rect(x, y, w, h, c)


# Large digit font (10x16 pixels each)
# Each digit is stored as list of horizontal line segments
LARGE_DIGITS = {
    '0': [(1,0,8,0),(0,1,0,14),(9,1,9,14),(1,15,8,15)],
    '1': [(4,0,4,15)],
    '2': [(0,0,9,0),(9,1,9,7),(0,8,9,8),(0,9,0,15),(0,15,9,15)],
    '3': [(0,0,9,0),(9,1,9,15),(0,7,8,7),(0,15,8,15)],
    '4': [(0,0,0,7),(0,8,9,8),(9,0,9,15)],
    '5': [(0,0,9,0),(0,1,0,7),(0,7,9,7),(9,8,9,15),(0,15,8,15)],
    '6': [(1,0,9,0),(0,1,0,15),(1,15,8,15),(9,8,9,14),(1,7,8,7)],
    '7': [(0,0,9,0),(9,1,9,15)],
    '8': [(1,0,8,0),(0,1,0,15),(9,1,9,15),(1,7,8,7),(1,15,8,15)],
    '9': [(1,0,8,0),(0,1,0,7),(9,1,9,15),(1,7,8,7),(1,15,9,15)],
    '.': [(4,13,5,15)],
    '-': [(1,7,8,7)],
    '+': [(4,4,4,11),(1,7,8,7)],
    ' ': [],
    ':': [(4,4,5,5),(4,11,5,12)],
    '%': [(0,0,2,2),(7,13,9,15),(9,0,0,15)],
}


class OLEDStatus:
    """Status display for 128x32 OLED with large text."""

    def __init__(self, i2c, addr=0x3C):
        try:
            self.oled = SSD1306(i2c, width=128, height=32, addr=addr)
            self.available = True
        except:
            self.oled = None
            self.available = False
            print("OLED not found")

    def _draw_large_char(self, char, x, y, scale=1):
        """Draw a large character at position."""
        if char not in LARGE_DIGITS:
            return 10 * scale

        for seg in LARGE_DIGITS[char]:
            if len(seg) == 4:
                x1, y1, x2, y2 = seg
                # Draw thick line
                for dx in range(scale):
                    for dy in range(scale):
                        if x1 == x2:  # Vertical line
                            for yy in range(y1, y2+1):
                                self.oled.pixel(x + x1*scale + dx, y + yy*scale + dy, 1)
                        elif y1 == y2:  # Horizontal line
                            for xx in range(x1, x2+1):
                                self.oled.pixel(x + xx*scale + dx, y + y1*scale + dy, 1)
                        else:  # Diagonal
                            for i in range(max(abs(x2-x1), abs(y2-y1))+1):
                                xx = x1 + i * (1 if x2 > x1 else -1)
                                yy = y1 + i * (1 if y2 > y1 else -1)
                                self.oled.pixel(x + xx*scale + dx, y + yy*scale + dy, 1)
        return 10 * scale

    def _draw_large_text(self, text, x, y, scale=1):
        """Draw text using large digits."""
        for char in text:
            w = self._draw_large_char(char, x, y, scale)
            x += w + scale

    def show_two_lines(self, line1, line2):
        """Show 2 lines of regular text (16 chars each)."""
        if not self.available:
            return
        self.oled.fill(0)
        self.oled.text(line1[:16], 0, 4)
        self.oled.text(line2[:16], 0, 20)
        self.oled.show()

    def show_state_value(self, state, value, unit=""):
        """Show state on top, large value below.

        state: 4-5 char status like "IDLE", "RUN", "CAL"
        value: number to show large
        unit: optional unit like "cm" or "%"
        """
        if not self.available:
            return

        self.oled.fill(0)

        # State in top-left with box
        self.oled.rect(0, 0, len(state)*8 + 4, 11, 1)
        self.oled.text(state, 2, 2)

        # Large value
        val_str = f"{value:.1f}" if isinstance(value, float) else str(value)
        self._draw_large_text(val_str, 4, 14)

        # Unit on right
        if unit:
            self.oled.text(unit, 128 - len(unit)*8, 20)

        self.oled.show()

    def show_run_status(self, direction, distance, bumps):
        """Show calibration run status."""
        if not self.available:
            return

        self.oled.fill(0)

        # Direction indicator
        arrow = ">>>" if direction > 0 else "<<<"
        self.oled.text(arrow, 0, 0)

        # Distance (large)
        dist_str = f"{distance:.0f}"
        self._draw_large_text(dist_str, 30, 8)
        self.oled.text("cm", 90, 14)

        # Bumps count
        self.oled.text(f"B:{bumps}", 0, 24)

        self.oled.show()

    def show_result(self, actual, estimated=None, error_pct=None):
        """Show calibration result."""
        if not self.available:
            return

        self.oled.fill(0)

        if error_pct is not None:
            # Verification result
            self.oled.text("RESULT", 0, 0)
            self.oled.text(f"Act:{actual:.1f}", 0, 12)
            self.oled.text(f"Est:{estimated:.1f}" if estimated else "Est:---", 0, 22)

            # Error on right side (large if error > 5%)
            err_str = f"{error_pct:+.0f}%"
            if abs(error_pct) > 5:
                self.oled.fill_rect(80, 0, 48, 32, 1)
                self.oled.text(err_str, 84, 12, 0)  # Inverted
            else:
                self.oled.text(err_str, 80, 12)
        else:
            # Calibration result
            self.oled.text("CALIBRATED", 0, 0)
            self.oled.text(f"Dist: {actual:.1f}cm", 0, 14)

        self.oled.show()

    def show_idle(self, distance, bumps_per_cm=None):
        """Show idle status with distance and calibration."""
        if not self.available:
            return

        self.oled.fill(0)

        # Line 1: Status
        self.oled.text("READY", 0, 0)

        # Line 2: US distance with 1 decimal
        dist_str = f"{distance:.1f}" if distance > 0 else "--.-"
        self.oled.text(f"US: {dist_str} cm", 0, 12)

        # Line 3: b/cm calibration value
        if bumps_per_cm:
            self.oled.text(f"B/cm: {bumps_per_cm:.3f}", 0, 24)
        else:
            self.oled.text("B/cm: ---", 0, 24)

        self.oled.show()

    def show_message(self, line1, line2=""):
        """Show simple message."""
        if not self.available:
            return

        self.oled.fill(0)

        # Center line1
        x1 = max(0, (128 - len(line1) * 8) // 2)
        y1 = 4 if line2 else 12
        self.oled.text(line1, x1, y1)

        if line2:
            x2 = max(0, (128 - len(line2) * 8) // 2)
            self.oled.text(line2, x2, 20)

        self.oled.show()

    def clear(self):
        if self.available:
            self.oled.clear()
