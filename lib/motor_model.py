"""
==============================================================================
MODEL-BASED MOTOR CONTROL
==============================================================================

Uses identified motor model for better control:
- Feedforward: Predicts PWM needed for desired speed
- Deadband compensation: Overcomes static friction
- Motor balancing: Corrects left/right differences
- Battery compensation: Adjusts for voltage drop

IDENTIFIED MODEL PARAMETERS:
---------------------------
- K_motor: Angular velocity per PWM unit (°/s / PWM)
- K_inverse: PWM per angular velocity (PWM / °/s)
- deadband: Minimum PWM to overcome static friction
- pwm_min_left/right: Startup threshold per motor
- pwm_balance: Correction for motor imbalance
- tau_ms: Motor time constant (for feedforward)
- V_calibration: Battery voltage during calibration

BATTERY VOLTAGE EFFECT:
----------------------
Motor speed ∝ Voltage. If calibrated at 4.0V but running at 3.5V:
  - Actual speed = (3.5/4.0) × calibrated speed = 87.5%
  - To maintain speed, increase PWM by 4.0/3.5 = 1.14×

Li-ion voltage curve (1S cell):
  - Full:    4.2V (100%)
  - Nominal: 3.7V (50%)
  - Low:     3.3V (10%)
  - Empty:   3.0V (0%) - stop using!

CONTROL MODES:
--------------
1. Open-loop feedforward: target_speed → PWM (using inverse model)
2. Closed-loop feedback: PID on speed error
3. Hybrid: Feedforward + feedback (best performance)

USAGE:
------
    from motor_model import MotorController
    from picobot import Motors
    from bmi160 import BMI160

    motors = Motors()
    imu = BMI160(i2c)

    # Load calibration
    controller = MotorController(motors)
    controller.load_calibration('motor_model.json')

    # Or set parameters manually
    controller.set_params(K_motor=0.5, deadband=25)

    # Open-loop control (feedforward only)
    controller.set_turn_rate(50)  # Request 50°/s turn

    # Closed-loop control
    controller.set_turn_rate(50, use_feedback=True, imu=imu)

    # In main loop:
    while True:
        controller.update()
        time.sleep_ms(10)

==============================================================================
"""

import json
import time
from machine import ADC, Pin


# =============================================================================
# BATTERY MONITOR
# =============================================================================

class BatteryMonitor:
    """
    Monitor Li-ion battery voltage for model compensation.

    WIRING (voltage divider required!):
    ----------------------------------
    Battery+ ──┬── R1 (10kΩ) ──┬── ADC pin (e.g., GP28)
               │               │
               └── R2 (10kΩ) ──┴── GND

    With 10k/10k divider: V_adc = V_bat / 2
    Pico ADC: 0-3.3V → 0-65535

    Li-ion voltage ranges (1S = 1 cell):
    - 1S: 3.0V (empty) to 4.2V (full)
    - 2S: 6.0V to 8.4V (need divider for Pico's 3.3V ADC!)
    """

    # Voltage thresholds (1S Li-ion)
    V_FULL = 4.2
    V_NOMINAL = 3.7
    V_LOW = 3.4      # Warning threshold
    V_CRITICAL = 3.2  # Stop motors!
    V_EMPTY = 3.0

    def __init__(self, adc_pin=28, divider_ratio=2.0, cell_count=1):
        """
        Initialize battery monitor.

        Args:
            adc_pin: ADC pin number (GP26=ADC0, GP27=ADC1, GP28=ADC2)
            divider_ratio: Voltage divider ratio (V_bat / V_adc)
            cell_count: Number of Li-ion cells in series (1S, 2S, etc.)
        """
        self._adc = ADC(Pin(adc_pin))
        self._divider = divider_ratio
        self._cells = cell_count

        # Calibration offset (measure and adjust if needed)
        self._offset = 0.0

        # Smoothing filter
        self._voltage = 0
        self._alpha = 0.1  # Low-pass filter coefficient

        # Thresholds scaled by cell count
        self.v_full = self.V_FULL * cell_count
        self.v_nominal = self.V_NOMINAL * cell_count
        self.v_low = self.V_LOW * cell_count
        self.v_critical = self.V_CRITICAL * cell_count

        # Initial reading
        self.update()

    def update(self):
        """Read and filter battery voltage."""
        # Read ADC (16-bit)
        raw = self._adc.read_u16()

        # Convert to voltage: (raw / 65535) * 3.3V * divider_ratio
        v_raw = (raw / 65535.0) * 3.3 * self._divider + self._offset

        # Low-pass filter
        if self._voltage == 0:
            self._voltage = v_raw
        else:
            self._voltage = self._alpha * v_raw + (1 - self._alpha) * self._voltage

        return self._voltage

    @property
    def voltage(self):
        """Get filtered battery voltage."""
        return self._voltage

    @property
    def percent(self):
        """
        Estimate battery percentage (rough, Li-ion curve is non-linear).

        Uses simplified linear approximation between low and full.
        """
        v = self._voltage
        if v >= self.v_full:
            return 100
        elif v <= self.v_critical:
            return 0
        else:
            return int(100 * (v - self.v_critical) / (self.v_full - self.v_critical))

    @property
    def is_low(self):
        """Check if battery is low (warning level)."""
        return self._voltage < self.v_low

    @property
    def is_critical(self):
        """Check if battery is critical (stop motors!)."""
        return self._voltage < self.v_critical

    def get_compensation_factor(self, v_calibration):
        """
        Get PWM compensation factor for current voltage.

        If calibrated at 4.0V and now at 3.5V, need to increase
        PWM by 4.0/3.5 = 1.14 to maintain same speed.

        Args:
            v_calibration: Voltage during calibration

        Returns:
            Multiplier for PWM (>1 if voltage dropped)
        """
        if self._voltage < 0.1 or v_calibration < 0.1:
            return 1.0

        factor = v_calibration / self._voltage

        # Clamp to reasonable range
        return max(0.8, min(1.5, factor))

    def status_string(self):
        """Get human-readable status."""
        v = self._voltage
        p = self.percent

        if self.is_critical:
            return f"CRITICAL! {v:.2f}V ({p}%)"
        elif self.is_low:
            return f"LOW {v:.2f}V ({p}%)"
        else:
            return f"OK {v:.2f}V ({p}%)"


class MotorController:
    """
    Model-based motor controller with feedforward and feedback.
    """

    def __init__(self, motors, battery=None):
        """
        Initialize controller.

        Args:
            motors: Motors instance from picobot
            battery: BatteryMonitor instance (optional, for voltage compensation)
        """
        self.motors = motors
        self.battery = battery

        # Model parameters (defaults, should be calibrated)
        self.K_motor = 0.5       # °/s per PWM unit
        self.K_inverse = 2.0     # PWM per °/s
        self.deadband = 25       # PWM units
        self.pwm_min_left = 30
        self.pwm_min_right = 30
        self.pwm_balance = 0     # Add to left motor for straight driving
        self.tau_ms = 100        # Time constant in ms

        # Battery compensation
        self.V_calibration = 3.7  # Voltage during calibration (nominal Li-ion)
        self._voltage_compensation = True

        # Control state
        self._target_left_speed = 0   # °/s
        self._target_right_speed = 0  # °/s
        self._target_turn_rate = 0    # °/s
        self._target_forward = 0      # PWM-like (0-255)

        # Feedback control
        self._use_feedback = False
        self._imu = None
        self._kp = 1.0           # Proportional gain
        self._ki = 0.1           # Integral gain
        self._kd = 0.05          # Derivative gain
        self._integral = 0
        self._last_error = 0
        self._last_update_ms = 0

        # Limits
        self.pwm_max = 255
        self.pwm_min = 0

    # =========================================================================
    # CALIBRATION
    # =========================================================================

    def load_calibration(self, filename='motor_model.json'):
        """
        Load calibration from JSON file.

        Args:
            filename: Path to calibration file
        """
        try:
            with open(filename, 'r') as f:
                params = json.load(f)

            self.K_motor = params.get('K_motor', self.K_motor)
            self.K_inverse = params.get('K_inverse', 1.0 / self.K_motor)
            self.deadband = params.get('deadband', self.deadband)
            self.pwm_min_left = params.get('pwm_min_left', self.pwm_min_left)
            self.pwm_min_right = params.get('pwm_min_right', self.pwm_min_right)
            self.pwm_balance = params.get('pwm_balance', self.pwm_balance)
            self.tau_ms = params.get('tau_ms', self.tau_ms)
            self.V_calibration = params.get('V_calibration', self.V_calibration)

            print(f"Loaded calibration from {filename}")
            print(f"  K_motor={self.K_motor:.3f}, deadband={self.deadband}")
            if self.V_calibration != 3.7:
                print(f"  V_calibration={self.V_calibration:.2f}V")
            return True
        except Exception as e:
            print(f"Failed to load calibration: {e}")
            return False

    def save_calibration(self, filename='motor_model.json'):
        """
        Save current calibration to JSON file.

        Args:
            filename: Path to save calibration
        """
        params = {
            'K_motor': self.K_motor,
            'K_inverse': self.K_inverse,
            'deadband': self.deadband,
            'pwm_min_left': self.pwm_min_left,
            'pwm_min_right': self.pwm_min_right,
            'pwm_balance': self.pwm_balance,
            'tau_ms': self.tau_ms,
            'V_calibration': self.V_calibration,
        }
        try:
            with open(filename, 'w') as f:
                json.dump(params, f)
            print(f"Saved calibration to {filename}")
            return True
        except Exception as e:
            print(f"Failed to save calibration: {e}")
            return False

    def set_calibration_voltage(self, voltage=None):
        """
        Set or measure the calibration voltage.

        Call this during calibration to record the battery voltage.
        If battery monitor is available and no voltage given, measures current.

        Args:
            voltage: Battery voltage during calibration (or None to measure)
        """
        if voltage is not None:
            self.V_calibration = voltage
        elif self.battery is not None:
            self.battery.update()
            self.V_calibration = self.battery.voltage
        print(f"Calibration voltage set to {self.V_calibration:.2f}V")

    def set_params(self, **kwargs):
        """
        Set model parameters manually.

        Args:
            K_motor: °/s per PWM unit
            deadband: Minimum PWM to start
            pwm_balance: Left motor correction
            tau_ms: Time constant
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

        # Update inverse
        if 'K_motor' in kwargs and self.K_motor > 0:
            self.K_inverse = 1.0 / self.K_motor

    # =========================================================================
    # INVERSE MODEL (Speed → PWM)
    # =========================================================================

    def speed_to_pwm(self, speed_dps, motor='left'):
        """
        Convert desired angular velocity to PWM command.

        Uses inverse model: PWM = speed / K_motor + deadband
        Applies battery voltage compensation if available.

        Args:
            speed_dps: Desired speed in °/s (positive = forward)
            motor: 'left' or 'right'

        Returns:
            PWM value (-255 to 255)
        """
        if abs(speed_dps) < 0.1:
            return 0

        # Direction
        sign = 1 if speed_dps >= 0 else -1
        abs_speed = abs(speed_dps)

        # Inverse model
        pwm = abs_speed * self.K_inverse + self.deadband

        # Motor-specific minimum
        pwm_min = self.pwm_min_left if motor == 'left' else self.pwm_min_right
        pwm = max(pwm, pwm_min)

        # Battery voltage compensation
        # If voltage dropped, increase PWM proportionally
        if self._voltage_compensation and self.battery is not None:
            self.battery.update()  # Refresh voltage reading
            comp_factor = self.battery.get_compensation_factor(self.V_calibration)
            pwm = pwm * comp_factor

        # Clamp to max
        pwm = min(pwm, self.pwm_max)

        return int(sign * pwm)

    def turn_rate_to_pwm(self, turn_rate_dps):
        """
        Convert desired turn rate to motor PWMs for tank turn.

        Args:
            turn_rate_dps: Desired rotation rate in °/s
                          (positive = counter-clockwise / left turn)

        Returns:
            (pwm_left, pwm_right)
        """
        # For tank turn: left and right motors run opposite directions
        # Robot geometry factor (depends on wheel base and wheel radius)
        # For now, assume gyro measures robot rotation directly

        speed = abs(turn_rate_dps)
        pwm = self.speed_to_pwm(speed)

        if turn_rate_dps >= 0:
            # Turn left: left backward, right forward
            return (-pwm, pwm)
        else:
            # Turn right: left forward, right backward
            return (pwm, -pwm)

    # =========================================================================
    # FORWARD MODEL (PWM → Speed, for prediction)
    # =========================================================================

    def pwm_to_speed(self, pwm):
        """
        Predict speed from PWM command.

        Uses forward model: speed = K_motor * (PWM - deadband)

        Args:
            pwm: PWM command (0-255)

        Returns:
            Predicted speed in °/s
        """
        if abs(pwm) <= self.deadband:
            return 0

        sign = 1 if pwm >= 0 else -1
        effective_pwm = abs(pwm) - self.deadband

        return sign * self.K_motor * effective_pwm

    # =========================================================================
    # HIGH-LEVEL CONTROL
    # =========================================================================

    def set_turn_rate(self, turn_rate_dps, use_feedback=False, imu=None):
        """
        Set desired turn rate.

        Args:
            turn_rate_dps: Target rotation rate in °/s
            use_feedback: Enable closed-loop control
            imu: BMI160 instance for feedback (required if use_feedback=True)
        """
        self._target_turn_rate = turn_rate_dps
        self._use_feedback = use_feedback and imu is not None
        self._imu = imu

        if use_feedback:
            self._integral = 0
            self._last_error = 0

        # Apply immediately with feedforward
        pwm_left, pwm_right = self.turn_rate_to_pwm(turn_rate_dps)
        self.motors.set_speeds(pwm_left, pwm_right)

    def set_drive(self, forward, turn_rate=0, use_feedback=False, imu=None):
        """
        Set forward speed and turn rate (differential drive).

        Args:
            forward: Forward PWM-like value (-255 to 255)
            turn_rate: Turn rate in °/s (positive = left)
            use_feedback: Enable feedback on turn rate
            imu: BMI160 instance
        """
        self._target_forward = forward
        self._target_turn_rate = turn_rate
        self._use_feedback = use_feedback and imu is not None
        self._imu = imu

        # Convert turn rate to differential
        turn_pwm = self.speed_to_pwm(abs(turn_rate)) if turn_rate != 0 else 0
        turn_diff = turn_pwm if turn_rate >= 0 else -turn_pwm

        # Apply with balance correction
        pwm_left = forward - turn_diff + self.pwm_balance / 2
        pwm_right = forward + turn_diff - self.pwm_balance / 2

        # Clamp
        pwm_left = max(-self.pwm_max, min(self.pwm_max, int(pwm_left)))
        pwm_right = max(-self.pwm_max, min(self.pwm_max, int(pwm_right)))

        self.motors.set_speeds(pwm_left, pwm_right)

    def update(self):
        """
        Update controller (call regularly, ~10-50 Hz).

        Applies feedback correction if enabled.
        """
        if not self._use_feedback or self._imu is None:
            return

        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, self._last_update_ms) / 1000.0
        self._last_update_ms = now_ms

        if dt <= 0 or dt > 0.5:  # Skip if dt invalid
            return

        # Read actual turn rate
        gyro = self._imu.get_gyro()
        actual = gyro['z']  # °/s

        # PID on turn rate error
        error = self._target_turn_rate - actual

        # Proportional
        p_term = self._kp * error

        # Integral (with anti-windup)
        self._integral += error * dt
        self._integral = max(-50, min(50, self._integral))  # Clamp
        i_term = self._ki * self._integral

        # Derivative
        d_term = self._kd * (error - self._last_error) / dt if dt > 0 else 0
        self._last_error = error

        # Total correction (in °/s)
        correction = p_term + i_term + d_term

        # Apply correction to motors
        # Feedforward + feedback
        target_corrected = self._target_turn_rate + correction
        pwm_left, pwm_right = self.turn_rate_to_pwm(target_corrected)

        # Add forward component if driving
        if self._target_forward != 0:
            pwm_left += self._target_forward
            pwm_right += self._target_forward

        self.motors.set_speeds(pwm_left, pwm_right)

    def stop(self):
        """Stop motors."""
        self._target_turn_rate = 0
        self._target_forward = 0
        self.motors.stop()

    # =========================================================================
    # PID TUNING
    # =========================================================================

    def set_pid(self, kp=None, ki=None, kd=None):
        """
        Set PID gains.

        Args:
            kp: Proportional gain (default 1.0)
            ki: Integral gain (default 0.1)
            kd: Derivative gain (default 0.05)
        """
        if kp is not None:
            self._kp = kp
        if ki is not None:
            self._ki = ki
        if kd is not None:
            self._kd = kd

    def reset_pid(self):
        """Reset PID state (integral, derivative)."""
        self._integral = 0
        self._last_error = 0

    # =========================================================================
    # BATTERY COMPENSATION
    # =========================================================================

    def enable_voltage_compensation(self, enable=True):
        """
        Enable or disable battery voltage compensation.

        Args:
            enable: True to enable compensation
        """
        self._voltage_compensation = enable
        status = "enabled" if enable else "disabled"
        print(f"Voltage compensation {status}")

    def get_battery_status(self):
        """
        Get current battery status.

        Returns:
            dict with voltage, percent, compensation_factor, or None if no battery
        """
        if self.battery is None:
            return None

        self.battery.update()
        return {
            'voltage': self.battery.voltage,
            'percent': self.battery.percent,
            'is_low': self.battery.is_low,
            'is_critical': self.battery.is_critical,
            'compensation_factor': self.battery.get_compensation_factor(self.V_calibration),
            'V_calibration': self.V_calibration,
        }

    def check_battery(self):
        """
        Check battery and warn if low. Returns True if OK to continue.

        Use this before starting motors to avoid damage from low voltage.

        Returns:
            True if battery OK, False if critical (should stop)
        """
        if self.battery is None:
            return True  # No monitor, assume OK

        self.battery.update()

        if self.battery.is_critical:
            print(f"!!! BATTERY CRITICAL: {self.battery.voltage:.2f}V - STOP MOTORS !!!")
            self.stop()
            return False
        elif self.battery.is_low:
            print(f"WARNING: Battery low: {self.battery.voltage:.2f}V ({self.battery.percent}%)")

        return True


# =============================================================================
# SIMPLE CALIBRATION-FREE CONTROL
# =============================================================================

class SimpleMotorControl:
    """
    Simple motor control without calibration.

    Uses basic model with reasonable defaults.
    Good for quick prototyping before full calibration.
    """

    def __init__(self, motors, deadband=30, max_speed=200):
        """
        Initialize simple controller.

        Args:
            motors: Motors instance
            deadband: Minimum PWM to overcome friction
            max_speed: Maximum PWM to use
        """
        self.motors = motors
        self.deadband = deadband
        self.max_speed = max_speed

    def drive(self, forward, turn):
        """
        Simple drive control.

        Args:
            forward: -1.0 to 1.0 (backward to forward)
            turn: -1.0 to 1.0 (right to left)
        """
        # Scale to PWM
        fwd_pwm = forward * (self.max_speed - self.deadband)
        turn_pwm = turn * (self.max_speed - self.deadband)

        # Differential
        left = fwd_pwm - turn_pwm
        right = fwd_pwm + turn_pwm

        # Add deadband for non-zero commands
        if abs(left) > 1:
            left = left + (self.deadband if left > 0 else -self.deadband)
        if abs(right) > 1:
            right = right + (self.deadband if right > 0 else -self.deadband)

        # Clamp
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))

        self.motors.set_speeds(left, right)

    def stop(self):
        """Stop motors."""
        self.motors.stop()


# =============================================================================
# UTILITY: AUTO-CALIBRATE ON STARTUP
# =============================================================================

def quick_calibrate(motors, imu, battery=None, duration_ms=2000):
    """
    Quick calibration: Measure motor constant with single test.

    Args:
        motors: Motors instance
        imu: BMI160 instance
        battery: BatteryMonitor instance (optional)
        duration_ms: Test duration

    Returns:
        dict with K_motor, deadband, and V_calibration estimates
    """
    print("Quick calibration - robot will spin briefly...")

    # Record battery voltage at calibration time
    V_cal = 3.7  # Default nominal
    if battery is not None:
        battery.update()
        V_cal = battery.voltage
        print(f"  Battery: {battery.status_string()}")

    # Find startup threshold
    print("  Finding startup threshold...")
    threshold = 0
    for pwm in range(0, 80, 5):
        motors.set_speeds(pwm, -pwm)
        time.sleep_ms(100)
        gyro = imu.get_gyro()
        if abs(gyro['z']) > 5:
            threshold = pwm
            break
    motors.stop()
    time.sleep_ms(200)

    # Measure at known PWM
    test_pwm = 150
    print(f"  Measuring at PWM={test_pwm}...")
    motors.set_speeds(test_pwm, -test_pwm)
    time.sleep_ms(500)  # Let it stabilize

    samples = []
    for _ in range(int(duration_ms / 10)):
        gyro = imu.get_gyro()
        samples.append(abs(gyro['z']))
        time.sleep_ms(10)

    motors.stop()

    avg_speed = sum(samples) / len(samples)
    effective_pwm = test_pwm - threshold

    K_motor = avg_speed / effective_pwm if effective_pwm > 0 else 0.5

    result = {
        'K_motor': K_motor,
        'K_inverse': 1.0 / K_motor if K_motor > 0 else 2.0,
        'deadband': threshold,
        'pwm_min_left': threshold,
        'pwm_min_right': threshold,
        'V_calibration': V_cal,
    }

    print(f"  K_motor = {K_motor:.4f} °/s per PWM")
    print(f"  Deadband = {threshold} PWM")
    print(f"  V_calibration = {V_cal:.2f}V")

    return result
