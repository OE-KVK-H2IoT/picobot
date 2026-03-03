"""
Quadrature encoder support for hall-effect wheel encoders.

Each encoder has two channels (A, B) offset by 90 degrees.
An ISR on channel A's rising edge reads channel B to determine
direction: B=0 means forward, B=1 means reverse (or vice versa
depending on wiring — swap A/B pins in config to fix).

Speed is computed in update() from tick deltas, NOT inside the ISR.
"""

from machine import Pin, disable_irq, enable_irq
import time


class Encoder:
    """Single quadrature encoder using pin interrupts."""

    def __init__(self, pin_a, pin_b):
        """
        Args:
            pin_a: GPIO number for channel A (interrupt source)
            pin_b: GPIO number for channel B (direction sense)
        """
        self._pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)

        self._ticks = 0
        self._last_ticks = 0
        self._last_time_us = time.ticks_us()
        self._speed_tps = 0.0

        # ISR on channel A rising edge
        self._pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)

    def _irq_handler(self, pin):
        """ISR: minimal work — read direction, update counter."""
        if self._pin_b.value():
            self._ticks -= 1
        else:
            self._ticks += 1

    @property
    def ticks(self):
        """Raw tick count (signed). Atomic read via IRQ disable."""
        irq_state = disable_irq()
        val = self._ticks
        enable_irq(irq_state)
        return val

    def reset(self):
        """Zero the tick counter."""
        irq_state = disable_irq()
        self._ticks = 0
        enable_irq(irq_state)
        self._last_ticks = 0
        self._last_time_us = time.ticks_us()
        self._speed_tps = 0.0

    @property
    def speed_tps(self):
        """Ticks per second (updated by calling update())."""
        return self._speed_tps

    def update(self):
        """
        Recompute speed from tick delta. Call this from
        your main loop (every 20-50 ms), NOT from an ISR.
        """
        now = time.ticks_us()
        current_ticks = self.ticks
        dt_us = time.ticks_diff(now, self._last_time_us)

        if dt_us > 0:
            delta = current_ticks - self._last_ticks
            self._speed_tps = delta * 1_000_000 / dt_us

        self._last_ticks = current_ticks
        self._last_time_us = now


class Encoders:
    """Paired encoders for differential-drive robot."""

    def __init__(self):
        from .config import PINS

        if PINS.ENCODER_LEFT_A is None or PINS.ENCODER_LEFT_B is None:
            raise ValueError("Left encoder pins not configured in config.PINS")

        self.left = Encoder(PINS.ENCODER_LEFT_A, PINS.ENCODER_LEFT_B)

        self.right = None
        if PINS.ENCODER_RIGHT_A is not None and PINS.ENCODER_RIGHT_B is not None:
            self.right = Encoder(PINS.ENCODER_RIGHT_A, PINS.ENCODER_RIGHT_B)

    def reset(self):
        """Reset both encoder counters."""
        self.left.reset()
        if self.right:
            self.right.reset()

    def update(self):
        """Update speed calculations for both encoders."""
        self.left.update()
        if self.right:
            self.right.update()
