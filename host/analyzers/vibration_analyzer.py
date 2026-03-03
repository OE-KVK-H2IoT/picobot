#!/usr/bin/env python3
"""
==============================================================================
VIBRATION ANALYZER - Speed Estimation from Vibration Patterns
==============================================================================

This analyzer estimates robot speed from vibration patterns in accelerometer
data. Different speeds produce different vibration intensities.

THEORY:
-------
When a robot moves:
  - Faster speed = more vibration = higher variance in accelerometer
  - Slower speed = less vibration = lower variance
  - Stopped = minimal vibration (just sensor noise)

By measuring the variance (spread) of accelerometer readings over a
sliding window, we can estimate relative speed.

KEY CONCEPTS:
-------------
1. Variance: Measures how spread out values are from their mean
   variance = sum((x - mean)²) / n

2. Sliding Window: Analyze a small window of recent samples (e.g., 50 samples)
   to get real-time vibration measurement

3. Speed Correlation: Experimentally determine the relationship between
   variance and actual speed, then use it for estimation

ALGORITHM:
----------
1. Compute acceleration magnitude per sample
2. Apply sliding window variance calculation
3. Map variance to speed using calibrated relationship
4. Detect motion state (stopped, slow, fast)

USAGE:
------
Standalone:
    python vibration_analyzer.py recording.csv

As module:
    from analyzers import VibrationAnalyzer
    analyzer = VibrationAnalyzer(window_size=50)
    analyzer.load_csv('recording.csv')
    results = analyzer.analyze()

==============================================================================
"""

import csv
import math
import argparse
from collections import deque


class VibrationAnalyzer:
    """
    Analyzes vibration patterns for speed estimation.

    Uses sliding window variance of accelerometer data to estimate
    robot motion intensity and speed.

    Attributes:
        window_size (int): Number of samples in sliding window
        stopped_threshold (float): Variance below this = stopped
        speed_scale (float): Scaling factor for variance to speed
    """

    def __init__(self, window_size=50, stopped_threshold=0.001, speed_scale=100.0):
        """
        Initialize the vibration analyzer.

        Args:
            window_size: Number of samples for variance calculation.
                        Larger = smoother but slower response
                        Smaller = faster response but noisier
                        Typical: 30-100 (at 500Hz, this is 60-200ms)

            stopped_threshold: Variance threshold for "stopped" detection.
                              Typical: 0.0005 - 0.002

            speed_scale: Multiplier to convert variance to speed estimate.
                        This needs calibration for your specific robot.
        """
        self.window_size = window_size
        self.stopped_threshold = stopped_threshold
        self.speed_scale = speed_scale

        # Data storage
        self.data = []
        self.results = None

    def load_csv(self, filename):
        """
        Load data from CSV file.

        Args:
            filename: Path to CSV file

        Returns:
            int: Number of samples loaded
        """
        self.data = []

        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    sample = {
                        'time_s': float(row.get('time_s', 0)),
                        'ax': float(row.get('ax', 0)),
                        'ay': float(row.get('ay', 0)),
                        'az': float(row.get('az', 0)),
                        'vib': float(row.get('vib', 0)),
                        'motor_l': int(float(row.get('motor_l', 0))),
                        'motor_r': int(float(row.get('motor_r', 0))),
                    }
                    self.data.append(sample)
                except (ValueError, KeyError):
                    continue

        print(f"Loaded {len(self.data)} samples from {filename}")
        return len(self.data)

    def load_data(self, data_list):
        """Load data from a list of dictionaries."""
        self.data = data_list

    def _calculate_variance(self, values):
        """
        Calculate variance of a list of values.

        Variance measures the "spread" of data from the mean.

        Formula: variance = sum((x - mean)²) / n

        Args:
            values: List of float values

        Returns:
            float: Variance of the values
        """
        if len(values) < 2:
            return 0.0

        # Calculate mean
        mean = sum(values) / len(values)

        # Calculate variance
        squared_diffs = [(x - mean) ** 2 for x in values]
        variance = sum(squared_diffs) / len(values)

        return variance

    def _sliding_window_variance(self, values):
        """
        Calculate variance over a sliding window.

        For each position, compute variance of the surrounding window
        of samples. This gives a time-series of "vibration intensity".

        Args:
            values: Full list of values

        Returns:
            list: Variance at each position
        """
        variances = []
        window = deque(maxlen=self.window_size)

        for val in values:
            window.append(val)

            if len(window) >= self.window_size // 2:
                var = self._calculate_variance(list(window))
                variances.append(var)
            else:
                variances.append(0.0)

        return variances

    def _detect_motion_states(self, variances):
        """
        Classify motion state based on variance.

        States:
          - STOPPED: Variance below stopped_threshold
          - SLOW: Variance 1-3x threshold
          - MEDIUM: Variance 3-10x threshold
          - FAST: Variance above 10x threshold

        Args:
            variances: List of variance values

        Returns:
            list: Motion state string for each sample
        """
        states = []
        t = self.stopped_threshold

        for var in variances:
            if var < t:
                states.append('STOPPED')
            elif var < t * 3:
                states.append('SLOW')
            elif var < t * 10:
                states.append('MEDIUM')
            else:
                states.append('FAST')

        return states

    def _estimate_speed(self, variances):
        """
        Estimate speed from variance values.

        This is a simple linear mapping. For more accuracy, you would
        calibrate this with actual speed measurements.

        Speed estimate = sqrt(variance) * speed_scale

        The sqrt helps linearize the relationship since variance
        grows quadratically with amplitude.

        Args:
            variances: List of variance values

        Returns:
            list: Estimated speed at each point
        """
        speeds = []

        for var in variances:
            # Square root to linearize, then scale
            speed = math.sqrt(var) * self.speed_scale
            speeds.append(speed)

        return speeds

    def _find_motion_periods(self, states):
        """
        Find continuous periods of motion.

        Returns list of periods with start/end times and average intensity.

        Args:
            states: List of motion state strings

        Returns:
            list: List of motion period dictionaries
        """
        periods = []
        in_motion = False
        start_idx = 0
        current_states = []

        for i, state in enumerate(states):
            is_moving = state != 'STOPPED'

            if is_moving and not in_motion:
                # Motion started
                start_idx = i
                in_motion = True
                current_states = [state]
            elif is_moving and in_motion:
                current_states.append(state)
            elif not is_moving and in_motion:
                # Motion ended
                if i - start_idx > 10:  # Minimum period length
                    # Find most common state
                    state_counts = {}
                    for s in current_states:
                        state_counts[s] = state_counts.get(s, 0) + 1
                    dominant = max(state_counts, key=state_counts.get)

                    periods.append({
                        'start_idx': start_idx,
                        'end_idx': i,
                        'samples': i - start_idx,
                        'dominant_state': dominant,
                        'start_time': self.data[start_idx]['time_s'],
                        'end_time': self.data[i]['time_s'],
                    })
                in_motion = False
                current_states = []

        # Handle motion continuing to end
        if in_motion and len(self.data) - start_idx > 10:
            state_counts = {}
            for s in current_states:
                state_counts[s] = state_counts.get(s, 0) + 1
            dominant = max(state_counts, key=state_counts.get)

            periods.append({
                'start_idx': start_idx,
                'end_idx': len(states) - 1,
                'samples': len(states) - start_idx,
                'dominant_state': dominant,
                'start_time': self.data[start_idx]['time_s'],
                'end_time': self.data[-1]['time_s'],
            })

        return periods

    def analyze(self):
        """
        Perform full vibration analysis.

        Returns:
            dict: Analysis results including:
                - variances: Time-series of variance values
                - states: Motion state at each sample
                - speeds: Estimated speed at each sample
                - periods: Detected motion periods
                - statistics: Overall statistics
        """
        if not self.data:
            return {'error': 'No data loaded'}

        # Step 1: Calculate acceleration magnitude
        magnitudes = []
        for sample in self.data:
            mag = math.sqrt(sample['ax']**2 + sample['ay']**2 + sample['az']**2)
            magnitudes.append(mag)

        # Step 2: Calculate sliding window variance
        variances = self._sliding_window_variance(magnitudes)

        # Step 3: Classify motion states
        states = self._detect_motion_states(variances)

        # Step 4: Estimate speed from variance
        speeds = self._estimate_speed(variances)

        # Step 5: Find motion periods
        periods = self._find_motion_periods(states)

        # Step 6: Calculate statistics
        state_counts = {'STOPPED': 0, 'SLOW': 0, 'MEDIUM': 0, 'FAST': 0}
        for s in states:
            state_counts[s] = state_counts.get(s, 0) + 1

        total = len(states)
        state_percentages = {k: v / total * 100 for k, v in state_counts.items()}

        avg_variance = sum(variances) / len(variances) if variances else 0
        max_variance = max(variances) if variances else 0
        avg_speed = sum(speeds) / len(speeds) if speeds else 0
        max_speed = max(speeds) if speeds else 0

        # Compare with robot's vibration values if available
        robot_vibs = [s['vib'] for s in self.data if s['vib'] > 0]
        avg_robot_vib = sum(robot_vibs) / len(robot_vibs) if robot_vibs else 0

        self.results = {
            'total_samples': len(self.data),
            'duration': self.data[-1]['time_s'] - self.data[0]['time_s'],
            'variances': variances,
            'states': states,
            'speeds': speeds,
            'periods': periods,
            'state_counts': state_counts,
            'state_percentages': state_percentages,
            'avg_variance': avg_variance,
            'max_variance': max_variance,
            'avg_speed': avg_speed,
            'max_speed': max_speed,
            'avg_robot_vib': avg_robot_vib,
        }

        return self.results

    def print_report(self):
        """Print a formatted analysis report."""
        if not self.results:
            self.analyze()

        r = self.results

        print("\n" + "=" * 60)
        print("VIBRATION ANALYSIS REPORT")
        print("=" * 60)

        print(f"\nData Overview:")
        print(f"  Samples:     {r['total_samples']}")
        print(f"  Duration:    {r['duration']:.2f} seconds")

        print(f"\nMotion State Distribution:")
        for state, pct in r['state_percentages'].items():
            count = r['state_counts'][state]
            bar = '#' * int(pct / 2)
            print(f"  {state:8s}: {count:5d} ({pct:5.1f}%) {bar}")

        print(f"\nVibration Statistics:")
        print(f"  Average variance:  {r['avg_variance']:.6f}")
        print(f"  Maximum variance:  {r['max_variance']:.6f}")
        print(f"  Avg robot vib:     {r['avg_robot_vib']:.2f}")

        print(f"\nSpeed Estimates:")
        print(f"  Average speed:     {r['avg_speed']:.1f} units")
        print(f"  Maximum speed:     {r['max_speed']:.1f} units")

        if r['periods']:
            print(f"\nMotion Periods ({len(r['periods'])} detected):")
            for i, period in enumerate(r['periods']):
                duration = period['end_time'] - period['start_time']
                print(f"\n  Period {i+1}:")
                print(f"    Time:     {period['start_time']:.2f} - {period['end_time']:.2f} s")
                print(f"    Duration: {duration:.2f} s")
                print(f"    State:    {period['dominant_state']}")

        print("\n" + "=" * 60)


def main():
    """Command-line interface for vibration analysis."""
    parser = argparse.ArgumentParser(
        description='Analyze vibration patterns for speed estimation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python vibration_analyzer.py recording.csv
  python vibration_analyzer.py recording.csv --window 30
  python vibration_analyzer.py data/*.csv --threshold 0.002
        """
    )

    parser.add_argument('csv_file', help='CSV file with recorded data')
    parser.add_argument('--window', '-w', type=int, default=50,
                       help='Sliding window size (default: 50)')
    parser.add_argument('--threshold', '-t', type=float, default=0.001,
                       help='Stopped detection threshold (default: 0.001)')
    parser.add_argument('--scale', '-s', type=float, default=100.0,
                       help='Speed scale factor (default: 100.0)')

    args = parser.parse_args()

    # Create analyzer
    analyzer = VibrationAnalyzer(
        window_size=args.window,
        stopped_threshold=args.threshold,
        speed_scale=args.scale
    )
    analyzer.load_csv(args.csv_file)
    analyzer.analyze()
    analyzer.print_report()

    return analyzer.results


if __name__ == '__main__':
    main()
