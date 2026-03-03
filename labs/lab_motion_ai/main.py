"""
==============================================================================
MOTION AI - Interactive Training Lab
==============================================================================

This lab teaches machine learning through hands-on robot training!

WORKFLOW:
1. Collect training data by performing movements
2. Train classifier (KNN learns on device, NN needs PC training)
3. Test real-time predictions
4. Export data for more advanced PC-based training

CONTROLS:
    Serial commands:
    'r' = Start RECORDING with current label
    's' = STOP recording
    '0'-'5' = Select movement CLASS to record
    't' = TEST mode (real-time predictions)
    'k' = Switch to KNN classifier
    'u' = Switch to RULES classifier
    'c' = Show sample COUNTS per class
    'e' = EXPORT data as CSV
    'x' = Clear all data
    'h' = Help

MOVEMENT CLASSES:
    0 = STOPPED     (keep robot still)
    1 = FORWARD     (push robot forward)
    2 = BACKWARD    (push robot backward)
    3 = TURN_LEFT   (rotate robot left)
    4 = TURN_RIGHT  (rotate robot right)
    5 = PUSHED      (sudden push from side)

==============================================================================
"""

import time
from machine import I2C, Pin

# Import our modules
from mpu6050 import MPU6050
from motion_ai import (
    MotionDetector, TrainingDataCollector, FeatureExtractor,
    CLASS_NAMES, STOPPED, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, PUSHED
)


# ==============================================================================
# CONFIGURATION
# ==============================================================================

# I2C pins for MPU6050
I2C_SDA = 4
I2C_SCL = 5

# Sampling
SAMPLE_RATE_HZ = 100  # Samples per second
SAMPLE_DELAY_MS = int(1000 / SAMPLE_RATE_HZ)
WINDOW_SIZE = 50  # Samples for feature extraction (~0.5 sec)

# Status display rate
DISPLAY_INTERVAL = 20  # Show status every N samples


# ==============================================================================
# MAIN APPLICATION
# ==============================================================================

class MotionAILab:
    def __init__(self):
        # Initialize I2C and IMU
        print("Initializing IMU...")
        self.i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
        self.imu = MPU6050(self.i2c, dlpf=2)

        # Initialize AI components
        self.detector = MotionDetector(method='rules', window_size=WINDOW_SIZE)
        self.collector = TrainingDataCollector()
        self.feature_extractor = FeatureExtractor(window_size=WINDOW_SIZE)

        # State
        self.current_label = STOPPED
        self.recording = False
        self.test_mode = False
        self.sample_count = 0

        print("Motion AI Lab initialized!")
        self.print_help()

    def print_help(self):
        """Print command help."""
        print("\n" + "=" * 60)
        print("MOTION AI LAB COMMANDS")
        print("=" * 60)
        print("  0-5  Select class to record:")
        for i, name in enumerate(CLASS_NAMES):
            print(f"        {i} = {name}")
        print("  r    Start RECORDING current class")
        print("  s    STOP recording")
        print("  t    Toggle TEST mode (predictions)")
        print("  k    Switch to KNN classifier")
        print("  u    Switch to RULES classifier")
        print("  c    Show sample COUNTS")
        print("  e    EXPORT data as CSV")
        print("  x    CLEAR all data")
        print("  h    Show this HELP")
        print("=" * 60 + "\n")

    def process_command(self, cmd):
        """Process serial command."""
        cmd = cmd.lower().strip()

        if cmd in '012345':
            self.current_label = int(cmd)
            print(f"Selected class: {CLASS_NAMES[self.current_label]}")

        elif cmd == 'r':
            if not self.recording:
                self.recording = True
                self.collector.start_recording(self.current_label)
            else:
                print("Already recording! Press 's' to stop first.")

        elif cmd == 's':
            if self.recording:
                self.recording = False
                self.collector.stop_recording()
            else:
                print("Not recording.")

        elif cmd == 't':
            self.test_mode = not self.test_mode
            if self.test_mode:
                print("\n>>> TEST MODE ON - showing predictions <<<")
                # Train KNN with collected data
                if self.collector.data:
                    print(f"Training KNN with {len(self.collector.data)} samples...")
                    for features, label in self.collector.data:
                        self.detector.train_knn(features, label)
                    print("KNN trained!")
            else:
                print("\n>>> TEST MODE OFF <<<")

        elif cmd == 'k':
            self.detector.set_method('knn')
            print("Switched to KNN classifier")
            print(f"  Training samples: {self.detector.classifiers['knn'].get_training_size()}")

        elif cmd == 'u':
            self.detector.set_method('rules')
            print("Switched to RULES classifier")

        elif cmd == 'c':
            counts = self.collector.get_sample_counts()
            print("\nSample counts:")
            print("-" * 30)
            for i, name in enumerate(CLASS_NAMES):
                bar = '#' * (counts[i] // 5)
                print(f"  {name:12s}: {counts[i]:4d} {bar}")
            print(f"  {'TOTAL':12s}: {sum(counts):4d}")
            print()

        elif cmd == 'e':
            csv_data = self.collector.export_csv()
            if csv_data:
                # Print header first (for easy copy-paste)
                print("\n" + "=" * 60)
                print("CSV DATA (copy everything below):")
                print("=" * 60)
                print(csv_data)
                print("=" * 60 + "\n")

                # Also try to save to file
                try:
                    filename = f"motion_data.csv"
                    with open(filename, 'w') as f:
                        f.write(csv_data)
                    print(f"Saved to {filename}")
                except:
                    print("(Could not save to file)")
            else:
                print("No data to export!")

        elif cmd == 'x':
            self.collector.clear()
            self.detector.classifiers['knn'].training_data.clear()
            print("All data cleared!")

        elif cmd == 'h':
            self.print_help()

        else:
            print(f"Unknown command: '{cmd}'")

    def run(self):
        """Main loop."""
        print("\nStarting sensor loop...")
        print("Calibrating (keep robot still)...")

        # Calibration - collect bias
        time.sleep(0.5)
        ax_bias, ay_bias, az_bias = 0, 0, 0
        gx_bias, gy_bias, gz_bias = 0, 0, 0
        cal_samples = 50

        for _ in range(cal_samples):
            accel = self.imu.read_accel()
            gyro = self.imu.read_gyro()
            ax_bias += accel['x']
            ay_bias += accel['y']
            az_bias += accel['z'] - 1.0  # Remove gravity
            gx_bias += gyro['x']
            gy_bias += gyro['y']
            gz_bias += gyro['z']
            time.sleep_ms(10)

        ax_bias /= cal_samples
        ay_bias /= cal_samples
        az_bias /= cal_samples
        gx_bias /= cal_samples
        gy_bias /= cal_samples
        gz_bias /= cal_samples

        print(f"Calibration done! Bias: ax={ax_bias:.3f} ay={ay_bias:.3f}")
        print("Ready! Enter commands or just observe.\n")

        last_display = 0
        import sys

        while True:
            # Check for serial input (non-blocking)
            if hasattr(sys.stdin, 'read'):
                try:
                    import select
                    if select.select([sys.stdin], [], [], 0)[0]:
                        cmd = sys.stdin.readline()
                        if cmd:
                            self.process_command(cmd)
                except:
                    pass

            # Read IMU
            accel = self.imu.read_accel()
            gyro = self.imu.read_gyro()

            # Apply calibration
            ax = accel['x'] - ax_bias
            ay = accel['y'] - ay_bias
            az = accel['z'] - az_bias - 1.0
            gx = gyro['x'] - gx_bias
            gy = gyro['y'] - gy_bias
            gz = gyro['z'] - gz_bias

            # Feed to detector
            self.detector.add_sample(ax, ay, az, gx, gy, gz)

            # Also feed to collector's feature extractor
            self.feature_extractor.add_sample(ax, ay, az, gx, gy, gz)

            # If recording, add to collector
            if self.recording:
                features = self.feature_extractor.extract_features()
                self.collector.add_sample(features)

            self.sample_count += 1

            # Display status
            if self.sample_count - last_display >= DISPLAY_INTERVAL:
                last_display = self.sample_count

                if self.test_mode:
                    # Show prediction
                    prediction = self.detector.predict()
                    proba = self.detector.predict_proba()
                    conf = max(proba) if proba else 0

                    # Confidence bar
                    bar = '█' * int(conf * 20)

                    print(f"[{self.detector.current_method.upper():5s}] "
                          f"Prediction: {CLASS_NAMES[prediction]:10s} "
                          f"Conf: {conf:.0%} {bar}")

                elif self.recording:
                    # Show recording progress
                    counts = self.collector.get_sample_counts()
                    n = counts[self.current_label]
                    print(f"[REC] Recording {CLASS_NAMES[self.current_label]:10s} "
                          f"Samples: {n}")

            time.sleep_ms(SAMPLE_DELAY_MS)


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("   MOTION AI LAB - Learn ML with Robot Movement Detection")
    print("=" * 60 + "\n")

    lab = MotionAILab()

    try:
        lab.run()
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
        print("Sample counts:")
        counts = lab.collector.get_sample_counts()
        for i, name in enumerate(CLASS_NAMES):
            print(f"  {name}: {counts[i]}")
