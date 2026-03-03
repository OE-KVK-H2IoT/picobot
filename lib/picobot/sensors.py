"""
Sensor modules - Line sensors and ultrasonic distance.

Students explore this in Lab 08 to understand how sensor
reading and error calculation actually works.
"""

from machine import Pin, ADC, time_pulse_us
import time
from .config import PINS, CONTROL, HARDWARE


class LineSensors:
    """
    Four-channel infrared line sensor array.

    Sensors return:
        0 = black (line detected)
        1 = white (no line)

    Access via robot.sensors.line
    """

    def __init__(self):
        """Initialize line sensor array."""
        self._sensors = [Pin(p, Pin.IN) for p in PINS.LINE_SENSORS]
        self._weights = CONTROL.SENSOR_WEIGHTS

    def read_raw(self):
        """
        Read raw sensor values.

        Returns:
            List of 4 integers [0 or 1]
            0 = black (on line), 1 = white (off line)
        """
        return [s.value() for s in self._sensors]

    def get_error(self):
        """
        Calculate line position error.

        Returns:
            Float from -1.5 (line far left) to +1.5 (line far right)
            None if line is lost (all sensors white)

        The error can be used directly for P-control:
            correction = Kp * error
            left_motor = base_speed + correction
            right_motor = base_speed - correction
        """
        readings = self.read_raw()

        # Find which sensors see black
        active_weights = []
        for i, reading in enumerate(readings):
            if reading == 0:  # Black detected
                active_weights.append(self._weights[i])

        if not active_weights:
            return None  # Line lost!

        # Average position of black readings
        return sum(active_weights) / len(active_weights)

    def at_junction(self):
        """
        Check if robot is at a junction.

        Returns:
            True if all sensors see black
        """
        return all(s.value() == 0 for s in self._sensors)

    def line_lost(self):
        """
        Check if line is lost.

        Returns:
            True if no sensors see black
        """
        return all(s.value() == 1 for s in self._sensors)

    def get_pattern(self):
        """
        Get visual representation of sensor readings.

        Returns:
            String like "█░░█" where █=black, ░=white
        """
        readings = self.read_raw()
        return "".join("█" if r == 0 else "░" for r in readings)


class Ultrasonic:
    """
    HC-SR04 ultrasonic distance sensor.

    WARNING: read_cm() takes ~25ms to complete!
    This is a "blocking" operation that will disrupt
    fast control loops.

    Access via robot.sensors.distance
    """

    def __init__(self):
        """Initialize ultrasonic sensor."""
        self._trig = Pin(PINS.ULTRASONIC_TRIG, Pin.OUT)
        self._echo = Pin(PINS.ULTRASONIC_ECHO, Pin.IN)
        self._trig.off()

        self._last_reading = 100  # Default: far away

    def read_cm(self):
        """
        Measure distance in centimeters.

        WARNING: This takes ~25ms! Don't call every loop iteration.

        Returns:
            Distance in cm (2-400 range)
            -1 if timeout/error
        """
        # Send trigger pulse
        self._trig.off()
        time.sleep_us(2)
        self._trig.on()
        time.sleep_us(10)
        self._trig.off()

        # Wait for echo
        try:
            pulse_us = time_pulse_us(
                self._echo,
                1,  # Wait for HIGH
                HARDWARE.ULTRASONIC_TIMEOUT_US
            )

            if pulse_us < 0:
                return -1

            # Calculate distance
            distance = (pulse_us * HARDWARE.SPEED_OF_SOUND_CM_US) / 2
            self._last_reading = distance
            return distance

        except OSError:
            return -1

    @property
    def last_reading(self):
        """Get last successful reading without blocking."""
        return self._last_reading

    def read_raw_us(self):
        """
        Get raw echo pulse duration in microseconds.
        Useful for debugging/learning.
        """
        self._trig.off()
        time.sleep_us(2)
        self._trig.on()
        time.sleep_us(10)
        self._trig.off()

        try:
            return time_pulse_us(self._echo, 1, HARDWARE.ULTRASONIC_TIMEOUT_US)
        except OSError:
            return -1


class Microphone:
    """
    Electret microphone via ADC.

    The mic outputs voltage centered at ~1.65V (half of 3.3V).
    Silence reads ~32768 on the 16-bit ADC.
    Sound swings above and below that center value.

    Access via robot.mic
    """

    def __init__(self):
        """Initialize microphone ADC."""
        self._adc = ADC(Pin(PINS.MIC_PIN))
        self._center = 32768
        self._noise = 200
        self._last_beat = 0
        self._history = [0] * 5
        self._idx = 0
        self.calibrate()

    def calibrate(self):
        """
        Calibrate DC offset and noise floor from silence.
        Call this when the environment is quiet and motors are off.
        """
        time.sleep_ms(100)
        # Take 200 samples, sort to reject outliers
        samples = sorted(self._adc.read_u16() for _ in range(200))
        # Trim top/bottom 10% (20 each)
        trimmed = samples[20:180]
        self._center = sum(trimmed) // len(trimmed)
        # Noise floor from trimmed set
        self._noise = sum(abs(s - self._center) for s in trimmed) // len(trimmed)
        if self._noise < 100:
            self._noise = 100
        # Reset history
        self._history = [0] * len(self._history)
        self._last_beat = 0

    def read_raw(self):
        """Read raw ADC value (0-65535, ~32768 = silence)."""
        return self._adc.read_u16()

    def amplitude(self):
        """Get current amplitude (distance from silence, 0-32768)."""
        return abs(self.read_raw() - self._center)

    def level(self):
        """
        Get smoothed sound level (0-100).

        Scaled relative to noise floor: silence ≈ 0, loud clap ≈ 100.
        """
        amp = self.amplitude()
        self._history[self._idx] = amp
        self._idx = (self._idx + 1) % len(self._history)
        avg = sum(self._history) // len(self._history)
        # 0 = at noise floor, 100 = 5x noise floor
        return min(100, max(0, (avg - self._noise) * 100 // (self._noise * 5)))

    def beat(self, threshold=40, cooldown_ms=200):
        """
        Detect impulsive sounds (claps, loud beats).

        Uses instantaneous amplitude vs. calibrated noise floor.
        Rejects continuous noise — only sharp transients trigger.

        Args:
            threshold: 0-100, higher = needs louder sound.
                30 = sensitive (8x noise floor)
                50 = moderate (12x noise floor)
                70 = loud claps only (16x noise floor)
            cooldown_ms: Minimum gap between beats in ms

        Returns:
            True if beat detected
        """
        amp = self.amplitude()
        now = time.ticks_ms()

        if time.ticks_diff(now, self._last_beat) < cooldown_ms:
            return False

        # Map threshold to noise multiplier (higher threshold = louder needed)
        # Floor of 2000 prevents false triggers when noise calibration is very low
        min_amp = max(2000, self._noise * (2 + threshold // 5))

        if amp > min_amp:
            self._last_beat = now
            return True
        return False
