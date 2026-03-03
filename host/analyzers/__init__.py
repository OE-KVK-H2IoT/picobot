"""
==============================================================================
PICOBOT DATA ANALYZERS
==============================================================================

This package contains modular data analysis tools for PicoBot sensor data.

EDUCATIONAL WORKFLOW:
--------------------
  1. COLLECT: Use host_collector.py + pico_collector.py to gather data
  2. ANALYZE: Run analyzers on recorded CSV files
  3. UNDERSTAND: Study the algorithms in each analyzer
  4. APPLY: Use insights to improve robot control

AVAILABLE ANALYZERS:
-------------------
  BumpDetector      - Wheel bump detection, B/cm calibration
                      Uses: High-pass filter + threshold crossing
                      Output: Bump count, distance estimation

  VibrationAnalyzer - Speed estimation from vibration intensity
                      Uses: Sliding window variance
                      Output: Motion states (STOPPED, SLOW, FAST)

  FFTAnalyzer       - Frequency analysis of sensor signals
                      Uses: Fast Fourier Transform
                      Output: Frequency spectrum, dominant frequencies

  MotionClassifier  - AI-based motion classification
                      Uses: Rule-based, KNN, Decision Tree, Random Forest
                      Output: Motion labels (FORWARD, BACKWARD, etc.)

  MotionDetector    - IMU-only movement detection (no ultrasonic!)
                      Uses: Gyro Z for turns, accel variance for motion
                      Output: FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STOPPED

  PartitionTool     - Interactive manual data partitioning
                      Uses: Click to select regions, keyboard to label
                      Output: Statistics across multiple samples per direction

USAGE - Standalone:
------------------
  python analyzers/bump_detector.py data/recording.csv --plot
  python analyzers/fft_analyzer.py data/recording.csv --bump
  python analyzers/vibration_analyzer.py data/recording.csv
  python analyzers/motion_classifier.py train data/ai_dataset.csv
  python analyzers/partition_tool.py data/recording.csv  # Interactive

USAGE - As Module:
-----------------
  from analyzers import BumpDetector, FFTAnalyzer

  detector = BumpDetector()
  detector.load_csv('data/recording.csv')
  results = detector.analyze()
  print(f"B/cm = {results['bumps_per_cm']:.3f}")

DATA FORMAT:
-----------
  All analyzers expect CSV files with columns:
    time_s, ax, ay, az, gx, gy, gz, us [, motor_l, motor_r, bumps, vib]

  Minimum required: time_s, ax, ay, az, gx, gy, gz, us
  Optional (from full host.py): motor_l, motor_r, bumps, vib, heading, drift

ALGORITHM COMPARISON:
--------------------
  Bump Detection:
    - Time domain: Count threshold crossings (simple, real-time)
    - Frequency domain: Find peak in FFT (more accurate, needs window)

  Speed Estimation:
    - From bumps: speed = bump_rate / bumps_per_rotation × circumference
    - From vibration: speed ∝ sqrt(variance) (needs calibration)
    - From FFT: speed = (peak_freq / bumps_per_rotation) × circumference

==============================================================================
"""

from .bump_detector import BumpDetector
from .vibration_analyzer import VibrationAnalyzer
from .motion_classifier import MotionClassifier
from .fft_analyzer import FFTAnalyzer
from .motion_detector import MotionDetector
from .partition_tool import PartitionTool

__all__ = ['BumpDetector', 'VibrationAnalyzer', 'MotionClassifier', 'FFTAnalyzer', 'MotionDetector', 'PartitionTool']
