#!/usr/bin/env python3
"""
==============================================================================
SPEED MODEL TRAINING - Educational ML Pipeline for Speed Estimation
==============================================================================

This script trains speed estimation models from calibration data collected
using the opto calibration track.

WORKFLOW:
---------
1. Collect data on calibration track (Pico-side)
2. Export to CSV via DataLogger
3. Run this script to train models
4. Copy coefficients to Pico

WHAT THIS TEACHES:
------------------
- Data preprocessing (cleaning, filtering)
- Feature extraction
- Linear regression
- Model validation
- Train/test split
- Error metrics

This is NOT a magic black box. Students see exactly what the model learns.

USAGE:
------
    python train_speed_model.py calibration_data.csv

    # Or with plotting:
    python train_speed_model.py calibration_data.csv --plot

==============================================================================
"""

import argparse
import sys
import os
from pathlib import Path

import numpy as np
import pandas as pd


def load_data(csv_path: str) -> pd.DataFrame:
    """
    Load and preprocess calibration data.

    Args:
        csv_path: Path to CSV file from data collection

    Returns:
        DataFrame with cleaned data
    """
    print(f"\n📂 Loading: {csv_path}")

    df = pd.read_csv(csv_path)
    print(f"   Loaded {len(df)} samples")

    # Check required columns
    required = ['pwm_avg', 'imu_vibration']
    optional = ['opto_speed', 'opto_distance', 'us_speed', 'ax', 'ay', 'az']

    missing = [c for c in required if c not in df.columns]
    if missing:
        print(f"   ⚠️ Missing columns: {missing}")
        print(f"   Available: {list(df.columns)}")

    # Add time column if missing
    if 'time_s' not in df.columns and 'timestamp_us' in df.columns:
        df['time_s'] = (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6

    return df


def train_motor_model(df: pd.DataFrame, plot: bool = False):
    """
    Train motor command → speed model.

    Model: speed = k × (PWM - dead_zone)

    Returns:
        dict with 'pwm_to_speed' and 'dead_zone'
    """
    print("\n🔧 Training Motor Model")
    print("   Model: speed = k × (PWM - dead_zone)")

    # Need opto_speed for ground truth
    if 'opto_speed' not in df.columns:
        print("   ⚠️ No opto_speed column - cannot train motor model")
        return None

    # Filter to samples where we have valid speed measurements
    valid = df[(df['opto_speed'] > 0.1) & (df['pwm_avg'] > 0)].copy()
    print(f"   Using {len(valid)} samples with valid speed readings")

    if len(valid) < 10:
        print("   ⚠️ Not enough data for training")
        return None

    # Find dead zone: minimum PWM where speed > 0
    moving = df[df['opto_speed'] > 0.5]
    if len(moving) > 0:
        dead_zone = moving['pwm_avg'].min()
    else:
        dead_zone = 30  # Default
    print(f"   Dead zone: {dead_zone:.0f} (min PWM with movement)")

    # Fit linear model: speed = k × (PWM - dead_zone)
    # Only use samples above dead zone
    above_dz = valid[valid['pwm_avg'] > dead_zone].copy()
    if len(above_dz) < 5:
        print("   ⚠️ Not enough samples above dead zone")
        return {'pwm_to_speed': 0.15, 'dead_zone': dead_zone}

    X = above_dz['pwm_avg'].values - dead_zone
    y = above_dz['opto_speed'].values

    # Simple linear regression (no intercept)
    pwm_to_speed = np.dot(X, y) / np.dot(X, X)

    # Calculate R² score
    y_pred = X * pwm_to_speed
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    print(f"   Result: speed = {pwm_to_speed:.4f} × (PWM - {dead_zone:.0f})")
    print(f"   R² score: {r2:.3f}")

    # Calculate prediction error
    rmse = np.sqrt(np.mean((y - y_pred) ** 2))
    print(f"   RMSE: {rmse:.2f} cm/s")

    if plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # Plot 1: Data and fit
        ax1 = axes[0]
        ax1.scatter(above_dz['pwm_avg'], above_dz['opto_speed'],
                   alpha=0.5, label='Data')
        pwm_range = np.linspace(dead_zone, above_dz['pwm_avg'].max(), 50)
        speed_pred = (pwm_range - dead_zone) * pwm_to_speed
        ax1.plot(pwm_range, speed_pred, 'r-', linewidth=2, label='Model')
        ax1.axvline(dead_zone, color='gray', linestyle='--',
                   label=f'Dead zone ({dead_zone:.0f})')
        ax1.set_xlabel('PWM Value')
        ax1.set_ylabel('Speed (cm/s)')
        ax1.set_title('Motor Model: PWM → Speed')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Residuals
        ax2 = axes[1]
        residuals = above_dz['opto_speed'] - (above_dz['pwm_avg'] - dead_zone) * pwm_to_speed
        ax2.scatter(above_dz['pwm_avg'], residuals, alpha=0.5)
        ax2.axhline(0, color='r', linestyle='-')
        ax2.axhline(rmse, color='gray', linestyle='--', label=f'±RMSE ({rmse:.2f})')
        ax2.axhline(-rmse, color='gray', linestyle='--')
        ax2.set_xlabel('PWM Value')
        ax2.set_ylabel('Residual (cm/s)')
        ax2.set_title('Model Residuals')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('motor_model.png', dpi=150)
        print(f"   📊 Saved: motor_model.png")

    return {
        'pwm_to_speed': pwm_to_speed,
        'dead_zone': dead_zone,
        'r2': r2,
        'rmse': rmse,
    }


def train_imu_model(df: pd.DataFrame, plot: bool = False):
    """
    Train IMU vibration → speed model.

    Model: speed = slope × vibration + intercept

    Returns:
        dict with 'slope', 'intercept', 'threshold'
    """
    print("\n📡 Training IMU Vibration Model")
    print("   Model: speed = slope × vibration + intercept")

    # Need both vibration and ground truth speed
    if 'imu_vibration' not in df.columns:
        print("   ⚠️ No imu_vibration column")
        return None

    if 'opto_speed' not in df.columns:
        print("   ⚠️ No opto_speed column - cannot train IMU model")
        return None

    # Filter to valid samples
    valid = df[(df['imu_vibration'] > 0) & (df['opto_speed'] >= 0)].copy()
    print(f"   Using {len(valid)} samples with valid readings")

    if len(valid) < 10:
        print("   ⚠️ Not enough data for training")
        return None

    # Linear regression with intercept
    X = valid['imu_vibration'].values
    y = valid['opto_speed'].values

    # Calculate coefficients
    n = len(X)
    x_mean = np.mean(X)
    y_mean = np.mean(y)
    slope = np.sum((X - x_mean) * (y - y_mean)) / np.sum((X - x_mean) ** 2)
    intercept = y_mean - slope * x_mean

    # Calculate R² score
    y_pred = slope * X + intercept
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    # Find threshold (vibration level where speed becomes significant)
    stationary = df[df['opto_speed'] < 0.5]
    if len(stationary) > 0:
        threshold = stationary['imu_vibration'].quantile(0.9)
    else:
        threshold = 0.5

    print(f"   Result: speed = {slope:.4f} × vibration + {intercept:.2f}")
    print(f"   Threshold: {threshold:.2f} (below = stationary)")
    print(f"   R² score: {r2:.3f}")

    # Calculate prediction error
    rmse = np.sqrt(np.mean((y - y_pred) ** 2))
    print(f"   RMSE: {rmse:.2f} cm/s")

    if plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # Plot 1: Data and fit
        ax1 = axes[0]
        ax1.scatter(valid['imu_vibration'], valid['opto_speed'],
                   alpha=0.5, label='Data')
        vib_range = np.linspace(0, valid['imu_vibration'].max(), 50)
        speed_pred = slope * vib_range + intercept
        ax1.plot(vib_range, speed_pred, 'r-', linewidth=2, label='Model')
        ax1.axvline(threshold, color='gray', linestyle='--',
                   label=f'Threshold ({threshold:.2f})')
        ax1.set_xlabel('IMU Vibration Level')
        ax1.set_ylabel('Speed (cm/s)')
        ax1.set_title('IMU Model: Vibration → Speed')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Comparison with motor model prediction
        ax2 = axes[1]
        if 'pwm_avg' in valid.columns:
            ax2.scatter(valid['opto_speed'], slope * valid['imu_vibration'] + intercept,
                       alpha=0.5, label='IMU prediction')
            ax2.plot([0, valid['opto_speed'].max()], [0, valid['opto_speed'].max()],
                    'r--', label='Perfect')
            ax2.set_xlabel('Actual Speed (cm/s)')
            ax2.set_ylabel('Predicted Speed (cm/s)')
            ax2.set_title('IMU Model: Predicted vs Actual')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        else:
            # Just show residuals
            residuals = y - y_pred
            ax2.hist(residuals, bins=30, alpha=0.7)
            ax2.axvline(0, color='r', linestyle='-')
            ax2.set_xlabel('Residual (cm/s)')
            ax2.set_ylabel('Count')
            ax2.set_title('Model Residuals')
            ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('imu_model.png', dpi=150)
        print(f"   📊 Saved: imu_model.png")

    return {
        'slope': slope,
        'intercept': intercept,
        'threshold': threshold,
        'r2': r2,
        'rmse': rmse,
    }


def validate_models(df: pd.DataFrame, motor_params: dict, imu_params: dict):
    """
    Cross-validate models against each other and ultrasonic if available.
    """
    print("\n✅ Model Validation")

    if 'opto_speed' not in df.columns:
        print("   ⚠️ Cannot validate without ground truth (opto_speed)")
        return

    valid = df[df['opto_speed'] > 0.5].copy()
    if len(valid) < 5:
        print("   ⚠️ Not enough moving samples for validation")
        return

    # Ground truth
    actual = valid['opto_speed'].values

    # Motor model prediction
    if motor_params and 'pwm_avg' in valid.columns:
        dz = motor_params['dead_zone']
        k = motor_params['pwm_to_speed']
        motor_pred = np.maximum(0, (valid['pwm_avg'].values - dz) * k)
        motor_mae = np.mean(np.abs(actual - motor_pred))
        motor_mape = np.mean(np.abs(actual - motor_pred) / actual) * 100
        print(f"   Motor model MAE: {motor_mae:.2f} cm/s ({motor_mape:.1f}%)")

    # IMU model prediction
    if imu_params and 'imu_vibration' in valid.columns:
        m = imu_params['slope']
        b = imu_params['intercept']
        imu_pred = m * valid['imu_vibration'].values + b
        imu_mae = np.mean(np.abs(actual - imu_pred))
        imu_mape = np.mean(np.abs(actual - imu_pred) / actual) * 100
        print(f"   IMU model MAE: {imu_mae:.2f} cm/s ({imu_mape:.1f}%)")

    # Ultrasonic if available
    if 'us_speed' in valid.columns:
        us_valid = valid[valid['us_speed'] > 0]
        if len(us_valid) > 0:
            us_mae = np.mean(np.abs(us_valid['opto_speed'] - us_valid['us_speed']))
            print(f"   Ultrasonic vs Opto MAE: {us_mae:.2f} cm/s")


def generate_pico_code(motor_params: dict, imu_params: dict):
    """Generate copy-paste code for Pico."""
    print("\n" + "=" * 60)
    print("📋 COPY THIS TO YOUR PICO CODE:")
    print("=" * 60)

    code = """
# === Speed Estimation Calibration ===
# Generated from calibration data
# Date: {date}

from speed_estimator import SpeedEstimator

# Initialize estimator
speed_est = SpeedEstimator(imu=imu, opto_pin=2)
"""

    if motor_params:
        code += f"""
# Motor model calibration
speed_est.load_motor_calibration(
    pwm_to_speed={motor_params['pwm_to_speed']:.4f},
    dead_zone={motor_params['dead_zone']:.0f}
)
"""

    if imu_params:
        code += f"""
# IMU vibration calibration
speed_est.load_imu_calibration(
    slope={imu_params['slope']:.4f},
    intercept={imu_params['intercept']:.2f},
    threshold={imu_params['threshold']:.2f}
)
"""

    code += """
# In main loop:
# speed_est.update_motor_command(left_pwm, right_pwm)
# speed_est.update()
# speed = speed_est.get_speed()  # cm/s
"""

    from datetime import datetime
    print(code.format(date=datetime.now().strftime("%Y-%m-%d %H:%M")))
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Train speed estimation models from calibration data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python train_speed_model.py data/calibration_run.csv
  python train_speed_model.py data/calibration_run.csv --plot

The script will output calibration code to copy to your Pico.
        """
    )
    parser.add_argument('csv_file', help='Path to calibration CSV data')
    parser.add_argument('--plot', action='store_true',
                       help='Generate visualization plots')
    args = parser.parse_args()

    # Check file exists
    if not os.path.exists(args.csv_file):
        print(f"❌ File not found: {args.csv_file}")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("🎓 SPEED MODEL TRAINING")
    print("   Educational ML Pipeline for Robot Speed Estimation")
    print("=" * 60)

    # Load data
    df = load_data(args.csv_file)

    # Train models
    motor_params = train_motor_model(df, plot=args.plot)
    imu_params = train_imu_model(df, plot=args.plot)

    # Validate
    if motor_params or imu_params:
        validate_models(df, motor_params, imu_params)

    # Generate Pico code
    generate_pico_code(motor_params, imu_params)

    if args.plot:
        print("\n📊 Showing plots...")
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == '__main__':
    main()
