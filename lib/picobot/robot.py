"""
Robot - Main robot control class

This is the entry point for students. They start with high-level methods
like robot.line_follow() and gradually discover the lower layers.
"""

from machine import Pin, PWM, I2C
import time

from .motors import Motors
from .sensors import LineSensors, Ultrasonic, Microphone
from .imu import IMU
from .leds import LEDStrip
from .buzzer import Buzzer
from .config import PINS, CONTROL


class Robot:
    """
    High-level robot control interface.

    Usage (Layer 0 - Magic):
        robot = Robot()
        robot.demo_dance()
        robot.play_melody("twinkle")

    Usage (Layer 1 - Control):
        robot.line_follow(speed=80, kp=30)
        robot.set_motors(100, 100)
        distance = robot.read_distance()

    Usage (Layer 2 - Direct Access):
        robot.motors.left.set_pwm(50)
        robot.sensors.line.read_raw()
        robot.imu.gyro_z()
    """

    def __init__(self, encoders=False):
        """
        Initialize all robot hardware.

        Args:
            encoders: If True, initialize quadrature wheel encoders.
                      Requires encoder-equipped motors and configured pins.
        """
        print("Initializing robot...")

        # Hardware modules (Layer 2 access)
        self.motors = Motors()
        self.sensors = _SensorGroup()
        self.imu = IMU()
        self.leds = LEDStrip()
        self.buzzer = Buzzer()
        self.mic = Microphone()

        # Optional: wheel encoders (deferred import saves memory)
        self.encoders = None
        if encoders:
            from .encoders import Encoders
            self.encoders = Encoders()

        # Internal state
        self._heading = 0.0
        self._last_heading_update = time.ticks_us()

        # Boot indication
        self.leds.fill((0, 50, 0))
        time.sleep(0.2)
        self.leds.off()

        print("Robot ready!")

    # ═══════════════════════════════════════════════════════════════
    # LAYER 0: MAGIC METHODS (Labs 01-02)
    # These "just work" - students don't need to understand how
    # ═══════════════════════════════════════════════════════════════

    def demo_lightshow(self, duration=5):
        """Run a colorful LED demonstration."""
        start = time.ticks_ms()
        hue = 0

        while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
            colors = []
            for i in range(8):
                h = (hue + i * 32) % 256
                colors.append(self._hsv_to_rgb(h, 255, 50))
            self.leds.set_all(colors)
            hue = (hue + 5) % 256
            time.sleep(0.05)

        self.leds.off()

    def demo_dance(self, duration=5):
        """Make the robot dance!"""
        moves = [
            (100, 100, 300),   # Forward
            (-100, -100, 300), # Back
            (-80, 80, 400),    # Spin left
            (80, -80, 400),    # Spin right
            (100, 50, 300),    # Curve right
            (50, 100, 300),    # Curve left
        ]

        start = time.ticks_ms()
        move_idx = 0

        while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
            left, right, dur = moves[move_idx % len(moves)]
            self.motors.set_speed(left, right)
            time.sleep(dur / 1000)
            move_idx += 1

        self.motors.stop()

    def play_melody(self, name="twinkle"):
        """Play a preset melody."""
        melodies = {
            "twinkle": [
                (262, 400), (262, 400), (392, 400), (392, 400),
                (440, 400), (440, 400), (392, 800),
                (349, 400), (349, 400), (330, 400), (330, 400),
                (294, 400), (294, 400), (262, 800),
            ],
            "scale": [
                (262, 250), (294, 250), (330, 250), (349, 250),
                (392, 250), (440, 250), (494, 250), (523, 500),
            ],
            "alert": [
                (880, 100), (0, 50), (880, 100), (0, 50), (880, 100),
            ],
            "success": [
                (523, 150), (659, 150), (784, 300),
            ],
            "error": [
                (200, 300), (150, 500),
            ],
        }

        if name not in melodies:
            print(f"Unknown melody: {name}")
            print(f"Available: {list(melodies.keys())}")
            return

        for freq, duration in melodies[name]:
            if freq > 0:
                self.buzzer.tone(freq)
            else:
                self.buzzer.off()
            time.sleep(duration / 1000)

        self.buzzer.off()

    def demo_dj(self, duration=10, bpm=128, max_speed=80):
        """
        Techno DJ mode: melodic riff, four-on-the-floor, build-ups, drops!

        Args:
            duration: Total duration in seconds
            bpm: Beats per minute (default 128 for techno)
            max_speed: Maximum motor speed 0-255 (default 80)
        """
        s = max_speed
        COLORS = [
            (255, 0, 50), (0, 50, 255), (255, 0, 255),
            (0, 255, 200), (255, 80, 0), (150, 0, 255),
        ]
        # Melodic riff: minor-key techno arpeggio (A minor / E minor vibe)
        RIFF = [330, 392, 494, 392, 330, 294, 330, 262]  # E4 G4 B4 G4 E4 D4 E4 C4
        MOVES = [
            (s, -s),             # spin right
            (-s, s),             # spin left
            (s, s * 30 // 100),  # hard arc right
            (s * 30 // 100, s),  # hard arc left
            (-s * 80 // 100, -s * 80 // 100),  # reverse
            (s, s),              # charge forward
            (-s, s),             # spin left
            (s, s * 40 // 100),  # wide arc right
        ]

        beat_ms = 60000 // bpm
        half = beat_ms // 2
        quarter = beat_ms // 4
        start = time.ticks_ms()
        beat = 0

        try:
            while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
                phrase_beat = beat % 16
                color = COLORS[(beat // 4) % len(COLORS)]
                note = RIFF[beat % len(RIFF)]

                if phrase_beat < 12:
                    # ── GROOVE: kick + melody + hat ──

                    # KICK on the beat: bass thump + bright flash + move
                    self.leds.fill(color)
                    self.buzzer.tone(80)
                    left, right = MOVES[beat % len(MOVES)]
                    self.motors.set_speed(left, right)
                    time.sleep_ms(30)

                    # Melodic note on top of kick
                    self.buzzer.tone(note)
                    time.sleep_ms(quarter - 30)
                    self.buzzer.off()

                    # Off-beat hi-hat: dim flash
                    dim = (color[0] // 5, color[1] // 5, color[2] // 5)
                    self.leds.fill(dim)
                    self.buzzer.tone(1400)
                    time.sleep_ms(15)
                    self.buzzer.off()
                    self.leds.off()
                    time.sleep_ms(quarter - 15)

                    # Second hi-hat (16th note feel)
                    self.leds.fill(dim)
                    self.buzzer.tone(1800)
                    time.sleep_ms(10)
                    self.buzzer.off()
                    self.leds.off()
                    time.sleep_ms(half - quarter - 10)

                elif phrase_beat < 15:
                    # ── BUILD-UP: accelerating hats + rising pitch + spin ──
                    self.motors.set_speed(-s, s)

                    step = phrase_beat - 12
                    n_hits = 4 + step * 4
                    gap = beat_ms // n_hits
                    for i in range(n_hits):
                        self.leds.fill((255, 255, 255))
                        self.buzzer.tone(600 + i * 150)
                        time.sleep_ms(min(15, gap // 2))
                        self.buzzer.off()
                        self.leds.off()
                        time.sleep_ms(max(5, gap - 15))

                else:
                    # ── DROP: silence → bass sweep → charge ──
                    self.motors.stop()
                    self.leds.off()
                    self.buzzer.off()
                    time.sleep_ms(60)

                    self.leds.fill((255, 255, 255))
                    self.motors.set_speed(s, s)
                    for f in (150, 120, 90, 70, 55):
                        self.buzzer.tone(f)
                        time.sleep_ms(beat_ms // 5)
                    self.buzzer.off()

                    self.motors.set_speed(s, -s)
                    self.leds.fill(color)
                    time.sleep_ms(beat_ms // 2)
                    self.motors.stop()
                    self.leds.off()
                    time.sleep_ms(40)

                beat += 1

            # Finale: rapid strobe + descending bass
            self.motors.set_speed(s, -s)
            for i in range(12):
                self.leds.fill((255, 255, 255))
                self.buzzer.tone(200 - i * 12)
                time.sleep_ms(30)
                self.leds.off()
                self.buzzer.off()
                time.sleep_ms(20)
            self.leds.fill((255, 0, 255))
            self.buzzer.tone(55)
            self.motors.set_speed(s, s)
            time.sleep_ms(300)

        except KeyboardInterrupt:
            pass

        self.motors.stop()
        self.buzzer.off()
        self.leds.off()

    def demo_dj_live(self, duration=60, sensitivity=80, max_speed=80):
        """
        Live DJ mode: robot reacts to music from your phone!

        Play music near the microphone and watch the robot
        flash colors and dance to the beat.

        Args:
            duration: How long to run in seconds (default 60)
            sensitivity: 10-100, higher = needs louder sound (default 80)
            max_speed: Maximum motor speed 0-255 (default 80)
        """
        s = max_speed
        COLORS = [
            (255, 0, 50), (0, 50, 255), (255, 0, 255),
            (0, 255, 200), (255, 80, 0), (150, 0, 255),
        ]
        MOVES = [
            (s, -s),             # spin right
            (-s, s),             # spin left
            (s, s * 30 // 100),  # hard arc right
            (s * 30 // 100, s),  # hard arc left
            (-s * 80 // 100, -s * 80 // 100),  # reverse burst
            (s, s),              # charge forward
            (-s, s),             # spin left
            (s, s * 40 // 100),  # wide arc right
        ]

        print("Live DJ Mode!")
        print("Calibrating mic (keep quiet)...")
        self.motors.stop()
        self.buzzer.off()
        time.sleep(0.5)
        self.mic.calibrate()
        min_amp = max(2000, self.mic._noise * (2 + sensitivity // 5))
        print(f"  Center: {self.mic._center}  Noise: {self.mic._noise}")
        print(f"  Trigger threshold: amplitude > {min_amp}")
        print("Play music near the robot...")
        print(f"Sensitivity: {sensitivity}  Duration: {duration}s")
        print("Press Ctrl+C to stop")
        print()

        start = time.ticks_ms()
        move_idx = 0
        beat_count = 0
        flash_until = 0

        try:
            while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
                now = time.ticks_ms()

                if self.mic.beat(threshold=sensitivity, cooldown_ms=300):
                    beat_count += 1
                    color = COLORS[beat_count % len(COLORS)]

                    # Bright flash
                    self.leds.fill(color)
                    flash_until = time.ticks_add(now, 250)

                    # Full-speed dance move
                    left, right = MOVES[move_idx % len(MOVES)]
                    self.motors.set_speed(left, right)
                    time.sleep_ms(250)
                    self.motors.stop()
                    move_idx += 1

                else:
                    # Dim glow between beats
                    if time.ticks_diff(now, flash_until) > 0:
                        color = COLORS[beat_count % len(COLORS)]
                        self.leds.fill((color[0] // 10, color[1] // 10, color[2] // 10))

                time.sleep_ms(10)

        except KeyboardInterrupt:
            pass

        self.motors.stop()
        self.buzzer.off()
        self.leds.off()
        print(f"Detected {beat_count} beats!")

    def drive_square(self, side_cm=30, speed=80):
        """Drive in a square pattern (approximate, time-based)."""
        duration = side_cm / 20  # Rough cm to seconds conversion

        for _ in range(4):
            self.forward(speed, duration)
            time.sleep(0.2)
            self.turn_left()
            time.sleep(0.2)

    # ═══════════════════════════════════════════════════════════════
    # LAYER 1: CONTROL METHODS (Labs 03-06)
    # Students use these and start to understand parameters
    # ═══════════════════════════════════════════════════════════════

    def forward(self, speed=80, duration=None):
        """Drive forward."""
        self.motors.set_speed(speed, speed)
        if duration:
            time.sleep(duration)
            self.motors.stop()

    def backward(self, speed=80, duration=None):
        """Drive backward."""
        self.motors.set_speed(-speed, -speed)
        if duration:
            time.sleep(duration)
            self.motors.stop()

    def turn_left(self, speed=80, duration=0.5):
        """Turn left in place.

        Args:
            speed: Motor speed (0-255)
            duration: Seconds to spin. Adjust to calibrate!
                      Try: robot.turn_left(speed=80, duration=0.3)
        """
        self.motors.set_speed(-speed, speed)
        time.sleep(duration)
        self.motors.stop()

    def turn_right(self, speed=80, duration=0.5):
        """Turn right in place.

        Args:
            speed: Motor speed (0-255)
            duration: Seconds to spin. Adjust to calibrate!
                      Try: robot.turn_right(speed=80, duration=0.3)
        """
        self.motors.set_speed(speed, -speed)
        time.sleep(duration)
        self.motors.stop()

    def turn_degrees(self, degrees, speed=80):
        """
        Turn by exact degrees using gyroscope.
        Positive = left, negative = right.
        """
        self.imu.reset_heading()
        target = degrees

        if degrees > 0:
            self.motors.set_speed(-speed, speed)
        else:
            self.motors.set_speed(speed, -speed)

        while abs(self.imu.heading) < abs(target) - 5:
            self.imu.update()
            time.sleep(0.01)

        self.motors.stop()
        return self.imu.heading

    def stop(self):
        """Stop all motors."""
        self.motors.stop()

    def set_motors(self, left, right):
        """Set individual motor speeds (-255 to 255)."""
        self.motors.set_speed(left, right)

    def line_follow(self, speed=80, kp=30, duration=None):
        """
        Follow a line continuously.

        Args:
            speed: Base forward speed (0-255)
            kp: Proportional gain for steering
            duration: Seconds to run (None = forever until Ctrl+C)
        """
        start = time.ticks_ms()

        try:
            while True:
                if duration and time.ticks_diff(time.ticks_ms(), start) > duration * 1000:
                    break

                self.line_follow_step(speed, kp)
                time.sleep(0.02)

        except KeyboardInterrupt:
            pass
        finally:
            self.motors.stop()

    def line_follow_step(self, speed=80, kp=30):
        """
        Single step of line following.
        Call this in your own loop for more control.

        Returns:
            error value, or None if line lost
        """
        error = self.sensors.line.get_error()

        if error is None:
            self.motors.stop()
            return None

        correction = kp * error
        left = int(speed + correction)
        right = int(speed - correction)

        self.motors.set_speed(left, right)
        return error

    def read_distance(self):
        """
        Read ultrasonic distance in cm.

        WARNING: This is SLOW (~25ms)!
        Don't call it every loop iteration.
        """
        return self.sensors.distance.read_cm()

    def get_heading(self):
        """Get current heading from IMU (degrees)."""
        self.imu.update()
        return self.imu.heading

    def reset_heading(self):
        """Reset heading to zero."""
        self.imu.reset_heading()

    def at_junction(self):
        """Check if robot is at a junction (all sensors see black)."""
        return self.sensors.line.at_junction()

    def line_lost(self):
        """Check if line is lost (no sensors see black)."""
        return self.sensors.line.line_lost()

    # ═══════════════════════════════════════════════════════════════
    # LED AND SOUND (All layers)
    # ═══════════════════════════════════════════════════════════════

    def set_leds(self, colors):
        """
        Set LED colors.

        Args:
            colors: Single (r,g,b) tuple for all, or list of 8 tuples
        """
        if isinstance(colors, tuple):
            self.leds.fill(colors)
        else:
            self.leds.set_all(colors)

    def set_led(self, index, color):
        """Set single LED color."""
        self.leds.set(index, color)

    def leds_off(self):
        """Turn off all LEDs."""
        self.leds.off()

    def beep(self, freq=440, duration=100):
        """Play a beep."""
        self.buzzer.tone(freq)
        time.sleep(duration / 1000)
        self.buzzer.off()

    # ═══════════════════════════════════════════════════════════════
    # HELPERS (Internal)
    # ═══════════════════════════════════════════════════════════════

    def _hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB (simple version)."""
        if s == 0:
            return (v, v, v)

        region = h // 43
        remainder = (h - (region * 43)) * 6

        p = (v * (255 - s)) >> 8
        q = (v * (255 - ((s * remainder) >> 8))) >> 8
        t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8

        if region == 0:
            return (v, t, p)
        elif region == 1:
            return (q, v, p)
        elif region == 2:
            return (p, v, t)
        elif region == 3:
            return (p, q, v)
        elif region == 4:
            return (t, p, v)
        else:
            return (v, p, q)


class _SensorGroup:
    """Groups all sensors for robot.sensors access."""

    def __init__(self):
        self.line = LineSensors()
        self.distance = Ultrasonic()
