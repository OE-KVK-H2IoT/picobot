"""
picobot - High-level robot control library for ES101

Designed for discovery-based learning:
- Layer 0: Magic methods (demo_dance, play_melody)
- Layer 1: Control methods (line_follow, set_motors)
- Layer 2: Direct hardware access (robot.motors.left.set_pwm)
- Layer 3: Source code (students read/modify this file)

Students start with high-level methods, then "peel back" layers
as they encounter problems and need to understand what's happening.
"""

from .robot import Robot
from .motors import Motors
from .sensors import LineSensors, Ultrasonic, Microphone
from .leds import LEDStrip
from .buzzer import Buzzer
from .utils import DataLogger, Timer
from .state_machine import StateMachine
from .encoders import Encoder, Encoders

# Legacy alias: old monolithic picobot.py used PicoBot as the main class
PicoBot = Robot

__all__ = [
    'Robot', 'PicoBot',
    'Motors', 'LineSensors', 'Ultrasonic', 'Microphone',
    'LEDStrip', 'Buzzer',
    'DataLogger', 'Timer', 'StateMachine',
    'Encoder', 'Encoders',
]
__version__ = '1.0.0'
