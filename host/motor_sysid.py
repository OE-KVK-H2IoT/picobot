#!/usr/bin/env python3
"""
==============================================================================
MOTOR SYSTEM IDENTIFICATION - ANALYSIS SCRIPT
==============================================================================

Runs on the host PC to analyze calibration data and fit motor model.

WHAT THIS DOES:
---------------
1. Receives calibration data from Pico via UDP
2. Fits motor model parameters using regression
3. Generates calibration file for robot control
4. Creates visualization plots

MODEL PARAMETERS IDENTIFIED:
---------------------------
1. Motor curve: PWM → angular velocity (linear or polynomial fit)
   - K_motor: Motor constant (°/s per PWM unit)
   - PWM_offset: Deadband offset

2. Time constant: τ (from step response)
   - τ_rise: Time to reach 63% of final speed

3. Startup threshold: T_static
   - PWM_min_left: Minimum PWM to start left motor
   - PWM_min_right: Minimum PWM to start right motor

4. Motor balance:
   - K_balance: Left/right speed ratio for straight driving

USAGE:
------
1. Start this script: python motor_sysid.py
2. Run motor_calibration.py on the Pico
3. Wait for calibration to complete
4. Review results and save calibration file

OUTPUT FILES:
-------------
- motor_calibration.json: Model parameters for robot use
- motor_calibration.csv: Raw calibration data
- motor_sysid_plots.png: Visualization of fits

==============================================================================
"""

import socket
import numpy as np
import json
import time
import sys
from datetime import datetime
from collections import defaultdict

# ==============================================================================
# CONFIGURATION
# ==============================================================================

UDP_PORT = 5010
SAVE_RAW_DATA = True

# ==============================================================================
# DATA COLLECTOR
# ==============================================================================

class CalibrationReceiver:
    """Receives calibration data from Pico via UDP."""

    def __init__(self, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', port))
        self.sock.settimeout(0.1)

        self.data = defaultdict(list)  # test_type -> list of samples
        self.markers = []
        self.raw_lines = []
        self.metadata = {}  # For V_CALIBRATION etc.

        print(f"Listening on UDP port {port}...")

    def receive(self, timeout_sec=120):
        """Receive data until timeout with no new data."""
        last_data_time = time.time()
        sample_count = 0

        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                line = data.decode().strip()

                if not line:
                    continue

                self.raw_lines.append(line)

                # Skip header
                if line.startswith('timestamp_ms'):
                    print("Received header, calibration started...")
                    continue

                # Marker/comment
                if line.startswith('#'):
                    marker = line[2:].strip()
                    self.markers.append(marker)
                    print(f"  {line}")

                    # Parse metadata markers like V_CALIBRATION=3.85
                    if '=' in marker:
                        key, val = marker.split('=', 1)
                        try:
                            self.metadata[key] = float(val)
                        except ValueError:
                            self.metadata[key] = val

                    continue

                # Parse data line: timestamp,pwm_left,pwm_right,gyro_z,test_type
                parts = line.split(',')
                if len(parts) >= 5:
                    try:
                        sample = {
                            'timestamp_ms': int(parts[0]),
                            'pwm_left': int(parts[1]),
                            'pwm_right': int(parts[2]),
                            'gyro_z': float(parts[3]),
                            'test_type': parts[4]
                        }
                        self.data[sample['test_type']].append(sample)
                        sample_count += 1

                        if sample_count % 100 == 0:
                            print(f"  Received {sample_count} samples...")

                    except ValueError:
                        pass

                last_data_time = time.time()

            except socket.timeout:
                if time.time() - last_data_time > timeout_sec:
                    print(f"\nNo data for {timeout_sec}s, stopping collection.")
                    break

        print(f"\nTotal samples received: {sample_count}")
        return sample_count > 0


# ==============================================================================
# MODEL FITTING
# ==============================================================================

class MotorModelFitter:
    """Fits motor model parameters from calibration data."""

    def __init__(self, data, metadata=None):
        self.data = data
        self.metadata = metadata or {}
        self.params = {}

        # Copy metadata into params
        if 'V_CALIBRATION' in self.metadata:
            self.params['V_calibration'] = self.metadata['V_CALIBRATION']

    def fit_steady_state(self):
        """
        Fit PWM → angular velocity curve.

        Uses linear regression: gyro_z = K * PWM + offset
        """
        samples = self.data.get('sweep', [])
        if not samples:
            print("No sweep data available!")
            return

        # Group by PWM value and average
        pwm_to_gyro = defaultdict(list)
        for s in samples:
            pwm = abs(s['pwm_left'])  # Use absolute PWM
            gyro = abs(s['gyro_z'])   # Use absolute gyro
            if pwm > 0:  # Skip stopped samples
                pwm_to_gyro[pwm].append(gyro)

        # Average at each PWM level
        pwm_values = sorted(pwm_to_gyro.keys())
        gyro_means = [np.mean(pwm_to_gyro[p]) for p in pwm_values]
        gyro_stds = [np.std(pwm_to_gyro[p]) for p in pwm_values]

        if len(pwm_values) < 3:
            print("Not enough data points for regression!")
            return

        # Linear regression: gyro = K * PWM + b
        X = np.array(pwm_values)
        y = np.array(gyro_means)

        # Use numpy polyfit for linear fit
        K, b = np.polyfit(X, y, 1)

        # Calculate R-squared
        y_pred = K * X + b
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

        self.params['K_motor'] = float(K)  # °/s per PWM unit
        self.params['pwm_offset'] = float(-b / K) if K != 0 else 0  # Deadband
        self.params['r_squared'] = float(r_squared)

        print(f"\n=== Steady-State Fit ===")
        print(f"  K_motor = {K:.4f} °/s per PWM")
        print(f"  Deadband = {-b/K:.1f} PWM units")
        print(f"  R² = {r_squared:.4f}")

        # Store raw data for plotting
        self.params['_sweep_pwm'] = pwm_values
        self.params['_sweep_gyro'] = gyro_means
        self.params['_sweep_std'] = gyro_stds

    def fit_step_response(self):
        """
        Fit time constant from step response.

        Model: gyro(t) = gyro_final * (1 - exp(-t/τ))
        """
        for direction in ['CW', 'CCW']:
            samples = self.data.get(f'step_{direction}', [])
            if not samples:
                continue

            # Extract time and gyro values
            t0 = samples[0]['timestamp_ms']
            times = np.array([s['timestamp_ms'] - t0 for s in samples]) / 1000.0  # seconds
            gyros = np.array([abs(s['gyro_z']) for s in samples])

            if len(times) < 10:
                continue

            # Find steady-state value (last 20% of samples)
            n_steady = max(5, len(gyros) // 5)
            gyro_final = np.mean(gyros[-n_steady:])

            if gyro_final < 5:  # Too slow to fit
                continue

            # Normalize response
            gyro_norm = gyros / gyro_final

            # Find time to reach 63.2% (1 - 1/e)
            target = 0.632
            idx_63 = np.argmax(gyro_norm >= target)
            if idx_63 > 0:
                tau = times[idx_63]
            else:
                # Fit exponential: gyro_norm = 1 - exp(-t/τ)
                # Linearize: -ln(1 - gyro_norm) = t/τ
                mask = (gyro_norm > 0.1) & (gyro_norm < 0.95)
                if np.sum(mask) > 5:
                    t_fit = times[mask]
                    y_fit = -np.log(1 - gyro_norm[mask])
                    tau = np.mean(t_fit / y_fit)
                else:
                    tau = 0.1  # Default

            self.params[f'tau_{direction}'] = float(tau)
            self.params[f'gyro_final_{direction}'] = float(gyro_final)

            print(f"\n=== Step Response ({direction}) ===")
            print(f"  τ = {tau*1000:.1f} ms")
            print(f"  Final speed = {gyro_final:.1f} °/s")

        # Average time constant
        taus = [v for k, v in self.params.items() if k.startswith('tau_')]
        if taus:
            self.params['tau'] = float(np.mean(taus))

    def fit_startup_threshold(self):
        """Find minimum PWM to start each motor."""
        for motor in ['left', 'right']:
            samples = self.data.get(f'startup_{motor}', [])
            if not samples:
                continue

            # Find first sample where gyro exceeds threshold
            threshold_gyro = 5.0  # °/s

            for s in samples:
                if abs(s['gyro_z']) > threshold_gyro:
                    pwm_min = abs(s['pwm_left']) if motor == 'left' else abs(s['pwm_right'])
                    self.params[f'pwm_min_{motor}'] = int(pwm_min)
                    print(f"\n=== Startup Threshold ({motor}) ===")
                    print(f"  PWM_min = {pwm_min}")
                    break

    def fit_motor_balance(self):
        """Calculate motor balance from drift during straight driving."""
        samples = self.data.get('balance_fwd', [])
        if samples:
            drift = np.mean([s['gyro_z'] for s in samples])
            self.params['drift_fwd'] = float(drift)

            # Positive drift = curving left = right motor faster
            # To compensate: reduce right motor or increase left
            if abs(drift) > 0.5:
                # Simple compensation factor
                # drift °/s at PWM 120 → need K_balance adjustment
                K_motor = self.params.get('K_motor', 1.0)
                if K_motor > 0:
                    pwm_diff = drift / K_motor  # PWM difference to add to left
                    self.params['pwm_balance'] = float(pwm_diff)

            print(f"\n=== Motor Balance ===")
            print(f"  Forward drift = {drift:.2f} °/s")
            if 'pwm_balance' in self.params:
                print(f"  Compensation: add {self.params['pwm_balance']:.1f} PWM to left motor")

    def fit_all(self):
        """Run all fitting routines."""
        print("\n" + "="*60)
        print("FITTING MOTOR MODEL")
        print("="*60)

        self.fit_startup_threshold()
        self.fit_steady_state()
        self.fit_step_response()
        self.fit_motor_balance()

        return self.params

    def get_model_for_robot(self):
        """
        Generate simplified model for robot use.

        Returns dict that can be saved as JSON and loaded on Pico.
        """
        model = {
            'version': 1,
            'created': datetime.now().isoformat(),

            # Motor curve: gyro = K * (PWM - deadband)
            'K_motor': self.params.get('K_motor', 1.0),
            'deadband': self.params.get('pwm_offset', 20),

            # Time constant (for feedforward)
            'tau_ms': self.params.get('tau', 0.1) * 1000,

            # Startup thresholds
            'pwm_min_left': self.params.get('pwm_min_left', 30),
            'pwm_min_right': self.params.get('pwm_min_right', 30),

            # Motor balance
            'pwm_balance': self.params.get('pwm_balance', 0),

            # Inverse model: PWM = gyro / K + deadband
            'K_inverse': 1.0 / self.params.get('K_motor', 1.0),

            # Battery voltage during calibration (default nominal Li-ion)
            # Update this manually or add battery streaming from Pico
            'V_calibration': self.params.get('V_calibration', 3.7),
        }

        return model


# ==============================================================================
# VISUALIZATION
# ==============================================================================

def plot_results(fitter, save_path='motor_sysid_plots.png'):
    """Create visualization plots."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plots")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Plot 1: Steady-state curve
    ax = axes[0, 0]
    if '_sweep_pwm' in fitter.params:
        pwm = fitter.params['_sweep_pwm']
        gyro = fitter.params['_sweep_gyro']
        std = fitter.params['_sweep_std']

        ax.errorbar(pwm, gyro, yerr=std, fmt='bo', label='Measured', capsize=3)

        # Fitted line
        K = fitter.params['K_motor']
        b = -K * fitter.params['pwm_offset']
        x_fit = np.linspace(0, max(pwm), 100)
        y_fit = K * x_fit + b
        ax.plot(x_fit, y_fit, 'r-', label=f'Fit: K={K:.3f}')

        ax.set_xlabel('PWM')
        ax.set_ylabel('Angular Velocity (°/s)')
        ax.set_title('Motor Curve: PWM → Speed')
        ax.legend()
        ax.grid(True)

    # Plot 2: Step response
    ax = axes[0, 1]
    for direction in ['CW', 'CCW']:
        samples = fitter.data.get(f'step_{direction}', [])
        if samples:
            t0 = samples[0]['timestamp_ms']
            times = [(s['timestamp_ms'] - t0) / 1000.0 for s in samples]
            gyros = [abs(s['gyro_z']) for s in samples]
            ax.plot(times, gyros, label=direction)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (°/s)')
    ax.set_title('Step Response')
    ax.legend()
    ax.grid(True)

    # Plot 3: Startup threshold
    ax = axes[1, 0]
    for motor, color in [('left', 'blue'), ('right', 'orange')]:
        samples = fitter.data.get(f'startup_{motor}', [])
        if samples:
            pwm = [abs(s['pwm_left']) if motor == 'left' else abs(s['pwm_right']) for s in samples]
            gyro = [abs(s['gyro_z']) for s in samples]
            ax.plot(pwm, gyro, f'{color[0]}o-', label=f'{motor.capitalize()} motor')

            # Mark threshold
            if f'pwm_min_{motor}' in fitter.params:
                thresh = fitter.params[f'pwm_min_{motor}']
                ax.axvline(thresh, color=color, linestyle='--', alpha=0.5)

    ax.set_xlabel('PWM')
    ax.set_ylabel('Angular Velocity (°/s)')
    ax.set_title('Startup Threshold')
    ax.legend()
    ax.grid(True)

    # Plot 4: Motor balance
    ax = axes[1, 1]
    samples = fitter.data.get('balance_fwd', [])
    if samples:
        t0 = samples[0]['timestamp_ms']
        times = [(s['timestamp_ms'] - t0) / 1000.0 for s in samples]
        gyros = [s['gyro_z'] for s in samples]
        ax.plot(times, gyros, 'g-')
        ax.axhline(0, color='black', linestyle='-', alpha=0.3)

        drift = fitter.params.get('drift_fwd', 0)
        ax.axhline(drift, color='red', linestyle='--', label=f'Mean: {drift:.2f}°/s')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Drift (°/s)')
    ax.set_title('Motor Balance (+ = curves left)')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"\nPlots saved to {save_path}")
    plt.show()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("\n" + "="*60)
    print("MOTOR SYSTEM IDENTIFICATION - HOST")
    print("="*60)
    print("\nWaiting for calibration data from Pico...")
    print("Run motor_calibration.py on the robot to start.")
    print("Press Ctrl+C to stop.\n")

    # Receive data
    receiver = CalibrationReceiver()
    try:
        if not receiver.receive(timeout_sec=10):
            print("No data received!")
            return
    except KeyboardInterrupt:
        print("\nStopped by user.")

    # Save raw data
    if SAVE_RAW_DATA and receiver.raw_lines:
        filename = f"motor_calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w') as f:
            f.write('\n'.join(receiver.raw_lines))
        print(f"\nRaw data saved to {filename}")

    # Fit model
    fitter = MotorModelFitter(receiver.data, receiver.metadata)
    params = fitter.fit_all()

    # Generate robot model
    model = fitter.get_model_for_robot()

    print("\n" + "="*60)
    print("MODEL FOR ROBOT")
    print("="*60)
    print(json.dumps(model, indent=2))

    # Save model
    model_file = 'motor_model.json'
    with open(model_file, 'w') as f:
        json.dump(model, f, indent=2)
    print(f"\nModel saved to {model_file}")

    # Plot results
    try:
        plot_results(fitter)
    except Exception as e:
        print(f"Plotting failed: {e}")


if __name__ == '__main__':
    main()
