"""
Buzzer module - Sound output via PWM.
"""

from machine import Pin, PWM
from .config import PINS


class Buzzer:
    """
    PWM buzzer controller.

    Access via robot.buzzer
    """

    # Musical note frequencies (Hz)
    NOTES = {
        'C4': 262, 'D4': 294, 'E4': 330, 'F4': 349,
        'G4': 392, 'A4': 440, 'B4': 494,
        'C5': 523, 'D5': 587, 'E5': 659, 'F5': 698,
        'G5': 784, 'A5': 880, 'B5': 988,
        'C6': 1047,
    }

    def __init__(self):
        """Initialize buzzer."""
        self._pwm = PWM(Pin(PINS.BUZZER))
        self._pwm.duty_u16(0)

    def tone(self, frequency, volume=50):
        """
        Play a tone.

        Args:
            frequency: Hz (20-20000)
            volume: 0-100 percent
        """
        if frequency < 20:
            self.off()
            return

        self._pwm.freq(int(frequency))
        duty = int(volume / 100 * 32768)  # 50% max for square wave
        self._pwm.duty_u16(duty)

    def note(self, note_name, volume=50):
        """
        Play a musical note.

        Args:
            note_name: e.g., 'C4', 'A4', 'G5'
            volume: 0-100 percent
        """
        freq = self.NOTES.get(note_name.upper(), 440)
        self.tone(freq, volume)

    def off(self):
        """Stop sound."""
        self._pwm.duty_u16(0)

    def beep(self, frequency=440, duration_ms=100, volume=50):
        """
        Play a short beep.

        Args:
            frequency: Hz
            duration_ms: Duration in milliseconds
            volume: 0-100 percent
        """
        import time
        self.tone(frequency, volume)
        time.sleep(duration_ms / 1000)
        self.off()

    def play_sequence(self, sequence):
        """
        Play a sequence of (frequency, duration_ms) pairs.

        Args:
            sequence: List of (freq, duration) tuples
        """
        import time
        for freq, duration in sequence:
            if freq > 0:
                self.tone(freq)
            else:
                self.off()
            time.sleep(duration / 1000)
        self.off()
