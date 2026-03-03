"""
Motor control module.

DRV8833 / NMS1100A H-bridge: each motor has two inputs (A and B).
  - Forward:  PWM on pin A, pin B held LOW (GPIO)
  - Backward: pin A held LOW (GPIO), PWM on pin B
  - Stop:     both pins LOW (coast)

Only one pin per motor is in PWM mode at a time.
The other is a plain GPIO output driven LOW.
This avoids RP2350 issues where PWM duty=0 may not produce
a clean LOW signal, causing the motor driver to glitch.
"""

from machine import Pin, PWM
from .config import PINS, HARDWARE


class Motor:
    """
    Single motor controller (two-pin H-bridge PWM).

    Students can access this directly via robot.motors.left
    """

    def __init__(self, pin_a, pin_b):
        """
        Initialize a motor.

        Args:
            pin_a: GPIO for forward / INA
            pin_b: GPIO for backward / INB
        """
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._pwm = None
        self._active_pin = None
        self._speed = 0
        # Both pins LOW (coast)
        Pin(pin_a, Pin.OUT, value=0)
        Pin(pin_b, Pin.OUT, value=0)

    def set_speed(self, speed):
        """
        Set motor speed.

        Args:
            speed: -255 (full backward) to 255 (full forward)
        """
        speed = max(-255, min(255, int(speed)))
        self._speed = speed

        if speed == 0:
            self.stop()
            return

        if speed > 0:
            target, other = self._pin_a, self._pin_b
        else:
            target, other = self._pin_b, self._pin_a

        duty = int(abs(speed) / 255 * HARDWARE.MOTOR_MAX_DUTY)

        # Switch PWM pin only when direction changes
        if self._active_pin != target:
            if self._pwm:
                self._pwm.deinit()
            Pin(other, Pin.OUT, value=0)
            self._pwm = PWM(Pin(target))
            self._pwm.freq(HARDWARE.MOTOR_PWM_FREQ)
            self._active_pin = target

        self._pwm.duty_u16(duty)

    def stop(self):
        """Stop the motor (coast)."""
        if self._pwm:
            self._pwm.deinit()
            self._pwm = None
        self._active_pin = None
        Pin(self._pin_a, Pin.OUT, value=0)
        Pin(self._pin_b, Pin.OUT, value=0)
        self._speed = 0

    @property
    def speed(self):
        """Current speed setting."""
        return self._speed

    def set_pwm(self, duty_percent):
        """
        Set raw PWM duty cycle on pin A (for learning/debugging).

        Args:
            duty_percent: 0-100
        """
        self.set_speed(int(duty_percent / 100 * 255))


class Motors:
    """
    Dual motor controller for differential drive.

    Access via robot.motors
    """

    def __init__(self):
        """Initialize both motors."""
        self.left = Motor(PINS.MOTOR_LEFT_A, PINS.MOTOR_LEFT_B)
        self.right = Motor(PINS.MOTOR_RIGHT_A, PINS.MOTOR_RIGHT_B)

    def set_speed(self, left, right):
        """
        Set both motor speeds.

        Args:
            left: -255 to 255
            right: -255 to 255
        """
        self.left.set_speed(left)
        self.right.set_speed(right)

    def stop(self):
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def forward(self, speed):
        """Drive forward."""
        self.set_speed(speed, speed)

    def backward(self, speed):
        """Drive backward."""
        self.set_speed(-speed, -speed)

    def turn_left(self, speed):
        """Spin left in place."""
        self.set_speed(-speed, speed)

    def turn_right(self, speed):
        """Spin right in place."""
        self.set_speed(speed, -speed)
