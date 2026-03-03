"""
Line Following with PID Control + Data Logging

Demonstrates the measurement-driven workflow:
  1. MEASURE: Collect sensor data via DataLogger
  2. MODEL: Host analyzes data to understand system behavior
  3. DESIGN: Choose Kp, Ki, Kd based on measurements
  4. IMPLEMENT: Run PID controller on robot
  5. VALIDATE: Log control variables, compare with model
  6. COMPARE: Adjust parameters based on real vs expected

USAGE:
------
1. Upload to Pico:
   ampy --port /dev/ttyACM0 put src/libs/picobot.py picobot.py
   ampy --port /dev/ttyACM0 put src/examples/line_follow_pid.py main.py

2. Run host collector:
   python src/host/host_collector.py

3. Robot streams data, host displays and records for analysis.

HARDWARE:
---------
- 4 optocouplers (X1-X4) for line position
- 2 DC motors for differential drive
- Optional: IMU for heading correction
"""

import time
import network
from machine import Pin, I2C
from picobot import PicoBot, DataLogger

# ==============================================================================
# CONFIGURATION
# ==============================================================================

WIFI_SSID = "YourNetwork"
WIFI_PASSWORD = "YourPassword"
HOST_IP = "192.168.1.100"  # Your computer's IP

# PID parameters (tune these based on measurements!)
KP = 30.0   # Proportional gain
KI = 0.0    # Integral gain (start with 0)
KD = 5.0    # Derivative gain

# Motor settings
BASE_SPEED = 120   # 0-255
MAX_SPEED = 200

# Line position weights (X1 is left, X4 is right)
# Position = weighted sum, 0 = centered
SENSOR_WEIGHTS = [-1.5, -0.5, 0.5, 1.5]  # X1, X2, X3, X4

# ==============================================================================
# WIFI CONNECTION
# ==============================================================================

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    print(f"Connecting to {WIFI_SSID}...", end="")
    for _ in range(20):
        if wlan.isconnected():
            print(f"\nConnected! IP: {wlan.ifconfig()[0]}")
            return True
        print(".", end="")
        time.sleep(0.5)

    print("\nFailed to connect!")
    return False

# ==============================================================================
# LINE POSITION CALCULATION
# ==============================================================================

def get_line_position(opto_pins):
    """
    Calculate line position from optocoupler readings.

    Returns:
        float: Position from -1.5 (far left) to +1.5 (far right)
               0 = centered on line
               None = no line detected
    """
    readings = [pin.value() for pin in opto_pins]
    active_count = sum(readings)

    if active_count == 0:
        return None  # No line detected

    # Weighted average of active sensors
    position = 0.0
    for i, (reading, weight) in enumerate(zip(readings, SENSOR_WEIGHTS)):
        if reading:
            position += weight

    return position / active_count

# ==============================================================================
# PID CONTROLLER
# ==============================================================================

class PIDController:
    """Simple PID controller for line following."""

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time_us = time.ticks_us()

    def update(self, error):
        """
        Compute PID output.

        Args:
            error: Current error (line position)

        Returns:
            float: Control output (steering correction)
        """
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, self.last_time_us) / 1_000_000.0
        self.last_time_us = now_us

        if dt <= 0:
            dt = 0.01

        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))
        i_term = self.ki * self.integral

        # Derivative
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        self.last_error = error

        return p_term + i_term + d_term

    def reset(self):
        """Reset integral and derivative state."""
        self.integral = 0.0
        self.last_error = 0.0

# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("\n" + "="*50)
    print("LINE FOLLOWING WITH PID + DATA LOGGING")
    print("="*50 + "\n")

    # Connect to WiFi
    if not connect_wifi():
        return

    # Initialize robot hardware
    robot = PicoBot()

    # Optocoupler pins (GP2-5)
    opto_pins = [Pin(i, Pin.IN) for i in range(2, 6)]

    # Initialize data logger
    logger = DataLogger(robot, host_ip=HOST_IP, enable_imu=False)
    logger.set_interval_hz(50)  # 50 Hz logging

    # Register PID variables for logging
    logger.add_var("line_pos", "f")     # Line position (-1.5 to +1.5)
    logger.add_var("pid_error", "f")    # Error (same as line_pos for now)
    logger.add_var("pid_p", "f")        # P term
    logger.add_var("pid_i", "f")        # I term
    logger.add_var("pid_d", "f")        # D term
    logger.add_var("pid_output", "f")   # Total output
    logger.add_var("motor_l", "B")      # Left motor (0-255)
    logger.add_var("motor_r", "B")      # Right motor (0-255)

    # Send schema so host knows what vars are available
    logger.send_schema()
    time.sleep(0.1)
    logger.send_schema()  # Send twice for reliability

    # Initialize PID controller
    pid = PIDController(KP, KI, KD)

    # State
    STATE_FOLLOW = 0
    STATE_SEARCH = 1
    state = STATE_FOLLOW
    last_known_pos = 0.0

    print(f"\nPID: Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"Base speed: {BASE_SPEED}")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            # Read line position
            line_pos = get_line_position(opto_pins)

            if line_pos is not None:
                # Line detected - follow it
                state = STATE_FOLLOW
                last_known_pos = line_pos
                error = line_pos

                # Compute PID output
                output = pid.update(error)

                # Calculate motor speeds
                left_speed = BASE_SPEED + output
                right_speed = BASE_SPEED - output

                # Clamp to valid range
                left_speed = max(0, min(MAX_SPEED, int(left_speed)))
                right_speed = max(0, min(MAX_SPEED, int(right_speed)))

            else:
                # Line lost - search by turning toward last known position
                state = STATE_SEARCH
                error = last_known_pos
                output = 0

                if last_known_pos < 0:
                    # Line was on left, turn left
                    left_speed = BASE_SPEED // 2
                    right_speed = BASE_SPEED
                else:
                    # Line was on right, turn right
                    left_speed = BASE_SPEED
                    right_speed = BASE_SPEED // 2

            # Apply motor speeds
            robot.motors.set_speeds(left_speed, right_speed)

            # Update logger with current values
            logger.set_motors(left_speed, right_speed)
            logger.set_vars(
                line_pos=line_pos if line_pos else 0.0,
                pid_error=error,
                pid_p=KP * error,
                pid_i=pid.ki * pid.integral,
                pid_d=pid.kd * (error - pid.last_error),
                pid_output=output,
                motor_l=left_speed,
                motor_r=right_speed
            )

            # Send data to host
            logger.log()

            # Small delay
            time.sleep_ms(20)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        robot.motors.stop()
        logger.close()
        print(f"Packets sent: {logger.packets_sent}, errors: {logger.errors}")

if __name__ == '__main__':
    main()
