"""
==============================================================================
CONTROL RECEIVER - Receive Remote Commands over UDP
==============================================================================

This module allows the robot to receive motor control commands from a host PC.
Used with host.py for keyboard-based remote control.

PROTOCOL:
---------
  Motor Command: 'M' (1 byte) + left_speed (2 bytes) + right_speed (2 bytes)
  - Speeds are signed 16-bit integers (-255 to 255)
  - Positive = forward, negative = backward

USAGE:
------
    from control_receiver import ControlReceiver

    controller = ControlReceiver(port=5006)

    while True:
        # Check for new commands (non-blocking)
        cmd = controller.check_commands()

        # Apply to motors if active
        if controller.is_active():
            left, right = controller.get_speeds()
            motors.set_speeds(left, right)
        else:
            motors.stop()

SAFETY:
-------
  - Commands timeout after 500ms if no new command received
  - Robot automatically stops when host disconnects
  - Speeds are clamped to -255 to 255

==============================================================================
"""

import socket
import struct
import time


class ControlReceiver:
    """
    Receives motor control commands from host PC over UDP.

    The host (running host.py) sends keyboard commands that this class
    receives and converts to motor speeds.
    """

    def __init__(self, port=5006, timeout_ms=500):
        """
        Initialize control receiver.

        Parameters:
            port:       UDP port to listen on (default 5006)
            timeout_ms: Stop motors if no command for this many ms (default 500)
        """
        self.port = port
        self.command_timeout_ms = timeout_ms

        # Create non-blocking UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

        # Current command state
        self.left_speed = 0
        self.right_speed = 0
        self.last_command_time = 0

        # Statistics
        self.commands_received = 0
        self.host_address = None

    def check_commands(self):
        """
        Check for incoming control commands (non-blocking).

        Returns:
            Tuple (left_speed, right_speed) if new command received,
            None if no new command.
        """
        try:
            data, addr = self.sock.recvfrom(64)

            # Save host address for debugging
            if self.host_address is None:
                self.host_address = addr
                print(f"Control connected from {addr[0]}:{addr[1]}")

            # Parse motor command: 'M' + left (2 bytes) + right (2 bytes)
            # Use '<' for little-endian to match host
            if len(data) >= 5 and data[0:1] == b'M':
                left, right = struct.unpack("<hh", data[1:5])

                # Clamp speeds to valid range
                self.left_speed = max(-255, min(255, left))
                self.right_speed = max(-255, min(255, right))
                self.last_command_time = time.ticks_ms()
                self.commands_received += 1

                # Debug: print received speeds
                if self.commands_received % 20 == 1:
                    print(f"Motor cmd: L={self.left_speed} R={self.right_speed}")

                return (self.left_speed, self.right_speed)

        except OSError:
            # No data available (expected for non-blocking socket)
            pass
        except Exception as e:
            print(f"Control receiver error: {e}")

        return None

    def get_speeds(self):
        """
        Get current motor speeds.

        Returns (0, 0) if commands have timed out.

        Returns:
            Tuple (left_speed, right_speed)
        """
        if self._is_timed_out():
            self.left_speed = 0
            self.right_speed = 0

        return (self.left_speed, self.right_speed)

    def _is_timed_out(self):
        """Check if commands have timed out."""
        if self.last_command_time == 0:
            return True
        elapsed = time.ticks_diff(time.ticks_ms(), self.last_command_time)
        return elapsed > self.command_timeout_ms

    def is_active(self):
        """
        Return True if actively receiving commands (not timed out).

        Use this to determine if remote control is active.
        """
        return not self._is_timed_out()

    def get_stats(self):
        """Return statistics about received commands."""
        return {
            'commands_received': self.commands_received,
            'host_address': self.host_address,
            'is_active': self.is_active(),
            'current_speeds': (self.left_speed, self.right_speed),
        }
