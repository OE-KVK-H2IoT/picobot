#!/usr/bin/env python3
"""
==============================================================================
BUMP DETECTOR - Wheel Bump Detection and B/cm Calibration
==============================================================================

This analyzer detects wheel bumps from accelerometer data and calculates
the bumps-per-centimeter (B/cm) calibration value.

THEORY:
-------
When a wheeled robot moves, small imperfections in the wheels and surface
create periodic vibrations (bumps) that appear in the accelerometer data.
By counting these bumps and comparing to the actual distance traveled
(from ultrasonic sensor), we can calculate a calibration ratio (B/cm).

Once calibrated, we can estimate distance traveled just from bump count:
    distance_cm = bump_count / bumps_per_cm

ALGORITHM:
----------
1. High-pass filter to remove DC offset and low-frequency drift
2. Calculate acceleration magnitude: mag = sqrt(ax² + ay² + az²)
3. Detect zero-crossings or threshold crossings as "bumps"
4. Compare bump count to ultrasonic distance for B/cm

USAGE:
------
Standalone:
    python bump_detector.py recording.csv

As module:
    from analyzers import BumpDetector
    detector = BumpDetector(threshold=0.015)
    detector.load_csv('recording.csv')
    results = detector.analyze()
    print(f"B/cm = {results['bumps_per_cm']:.3f}")

==============================================================================
"""

import csv
import math
import argparse
from collections import deque

# Optional: matplotlib for visualization
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class BumpDetector:
    """
    Detects wheel bumps from accelerometer data.

    The detector uses a high-pass filter to isolate the bump vibrations
    from the accelerometer signal, then counts threshold crossings.

    Attributes:
        threshold (float): Minimum magnitude change to count as bump (in g)
        hp_alpha (float): High-pass filter coefficient (0.9-0.99)
    """

    def __init__(self, threshold=0.015, hp_alpha=0.95, mag_mode='xy', window_ms=500):
        """
        Initialize the bump detector.

        Args:
            threshold: Sensitivity threshold in g units.
                      Lower = more sensitive, may detect noise as bumps
                      Higher = less sensitive, may miss small bumps
                      Typical range: 0.01 - 0.05

            hp_alpha: High-pass filter coefficient.
                     Closer to 1.0 = slower response, better DC rejection
                     Closer to 0.5 = faster response, less DC rejection
                     Typical range: 0.90 - 0.99

            mag_mode: Which magnitude to use for bump detection:
                     'xyz' - Full 3D magnitude sqrt(ax²+ay²+az²)-1
                     'xy'  - Horizontal plane sqrt(ax²+ay²) - better for wheels!
                     'x'   - Forward axis only (ax)
                     'y'   - Side axis only (ay)

                     'xy' is recommended because:
                     - Wheel bumps primarily cause horizontal vibrations
                     - Z axis is dominated by gravity (1g constant)
                     - When two wheels are in sync, X-Y resultant captures both

            window_ms: Window size in milliseconds for direction detection.
                      Larger = detects longer movements, fewer segments
                      Smaller = detects shorter movements, more segments
                      Default: 500ms (good for typical robot movements)
                      Try 1000-2000ms if movements are longer

            us_smooth: Low-pass filter alpha for ultrasonic smoothing.
                      0.0 = no smoothing (use raw values)
                      0.1 = heavy smoothing (slow response)
                      0.3 = moderate smoothing (recommended)
                      1.0 = no smoothing
                      Smoothing helps ignore ultrasonic noise spikes.
        """
        self.threshold = threshold
        self.hp_alpha = hp_alpha
        self.mag_mode = mag_mode
        self.window_ms = window_ms
        self.us_smooth = 0.2  # Default smoothing for ultrasonic
        self.sample_rate = 500  # Default, will be auto-detected from data

        # Data storage
        self.data = []
        self.results = None

    def load_csv(self, filename):
        """
        Load data from CSV file.

        Expected columns: time_s, ax, ay, az, ... , us, bumps, ...

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
                        'gx': float(row.get('gx', 0)),
                        'gy': float(row.get('gy', 0)),
                        'gz': float(row.get('gz', 0)),
                        'us': float(row.get('us', 0)),
                        'bumps': int(float(row.get('bumps', 0))),
                        'motor_l': int(float(row.get('motor_l', 0))),
                        'motor_r': int(float(row.get('motor_r', 0))),
                    }
                    self.data.append(sample)
                except (ValueError, KeyError) as e:
                    continue  # Skip invalid rows

        n = len(self.data)
        print(f"Loaded {n} samples from {filename}")

        # Auto-detect sample rate from timestamps
        if n > 10:
            duration = self.data[-1]['time_s'] - self.data[0]['time_s']
            if duration > 0:
                self.sample_rate = (n - 1) / duration
                print(f"Sample rate: {self.sample_rate:.1f} Hz (auto-detected)")

        return n

    def load_data(self, data_list):
        """
        Load data from a list of dictionaries.

        Args:
            data_list: List of dicts with keys: ax, ay, az, us, etc.
        """
        self.data = data_list

    def _calc_magnitude(self, sample):
        """
        Calculate magnitude based on selected mode.

        Different modes capture wheel bumps differently:

        'xyz' mode: sqrt(ax² + ay² + az²) - 1.0
            - Full 3D magnitude minus gravity
            - Simple but Z axis adds noise from gravity variations

        'xy' mode: sqrt(ax² + ay²)  [RECOMMENDED]
            - Horizontal plane only
            - Ignores gravity (Z axis)
            - Captures wheel bumps from both wheels
            - When wheels are in sync: bumps reinforce in same direction
            - When out of sync: bumps create rotation (still captured)

        'x' mode: ax (forward axis)
            - Only forward/backward component
            - May miss bumps that cause sideways motion

        'y' mode: ay (side axis)
            - Only left/right component
            - Captures differential wheel effects

        Args:
            sample: Dict with 'ax', 'ay', 'az' keys

        Returns:
            float: Magnitude value for bump detection
        """
        ax = sample['ax']
        ay = sample['ay']
        az = sample['az']

        if self.mag_mode == 'xyz':
            # Full 3D minus gravity
            return math.sqrt(ax**2 + ay**2 + az**2) - 1.0

        elif self.mag_mode == 'xy':
            # Horizontal plane resultant (recommended for wheels)
            return math.sqrt(ax**2 + ay**2)

        elif self.mag_mode == 'x':
            # Forward axis only
            return ax

        elif self.mag_mode == 'y':
            # Side axis only
            return ay

        else:
            # Default to xy
            return math.sqrt(ax**2 + ay**2)

    def _low_pass_filter(self, values, alpha=None):
        """
        Apply low-pass filter to smooth signal.

        A low-pass filter removes high-frequency noise, keeping the
        slow-changing trend. Good for smoothing ultrasonic readings.

        The filter equation (exponential moving average):
            y[n] = alpha * x[n] + (1 - alpha) * y[n-1]

        Where:
            x[n] = current input
            y[n] = current output (smoothed)
            alpha = smoothing factor (smaller = more smoothing)

        Args:
            values: List of float values to filter
            alpha: Smoothing factor (0.1=heavy, 0.5=moderate, 1.0=none)

        Returns:
            list: Smoothed values
        """
        if alpha is None:
            alpha = self.us_smooth

        if not values or alpha >= 1.0:
            return values

        smoothed = [values[0]]
        for i in range(1, len(values)):
            # Exponential moving average
            new_val = alpha * values[i] + (1 - alpha) * smoothed[-1]
            smoothed.append(new_val)

        return smoothed

    def _high_pass_filter(self, values):
        """
        Apply high-pass filter to remove DC offset.

        A high-pass filter removes slow-changing (low frequency) components,
        keeping only the rapid changes (high frequency) like bumps.

        The filter equation:
            y[n] = alpha * (y[n-1] + x[n] - x[n-1])

        Where:
            x[n] = current input
            y[n] = current output
            alpha = filter coefficient (0.95 typical)

        Args:
            values: List of float values to filter

        Returns:
            list: Filtered values
        """
        if not values:
            return []

        filtered = [0.0]
        for i in range(1, len(values)):
            # High-pass filter formula
            hp_out = self.hp_alpha * (filtered[-1] + values[i] - values[i-1])
            filtered.append(hp_out)

        return filtered

    def _count_bumps(self, filtered_mag):
        """
        Count threshold crossings in filtered signal.

        A "bump" is detected when the signal crosses above the threshold
        after being below it. This prevents counting the same bump twice.

        Args:
            filtered_mag: High-pass filtered acceleration magnitude

        Returns:
            tuple: (bump_count, bump_indices)
        """
        bumps = 0
        bump_indices = []
        was_above = False

        for i, val in enumerate(filtered_mag):
            is_above = val > self.threshold

            # Rising edge detection: was below, now above
            if is_above and not was_above:
                bumps += 1
                bump_indices.append(i)

            was_above = is_above

        return bumps, bump_indices

    def _detect_direction(self, us_start, us_end, threshold=2.0):
        """
        Detect movement direction from ultrasonic change.

        Simple logic based on distance to wall:
          - Distance decreased = moved FORWARD (toward wall)
          - Distance increased = moved BACKWARD (away from wall)
          - Little change = STOPPED

        Args:
            us_start: Ultrasonic reading at start (cm)
            us_end: Ultrasonic reading at end (cm)
            threshold: Minimum change to count as movement (cm)

        Returns:
            str: 'FORWARD', 'BACKWARD', or 'STOPPED'
        """
        delta = us_end - us_start

        if delta < -threshold:
            return 'FORWARD'   # Got closer to wall
        elif delta > threshold:
            return 'BACKWARD'  # Got further from wall
        else:
            return 'STOPPED'

    def _partition_by_ultrasonic(self):
        """
        Partition data by movement direction using ultrasonic sensor.

        Uses sliding window to detect direction changes.
        Simple approach: just look at ultrasonic distance trend.

        Window size is calculated from self.window_ms and actual sample rate.

        Returns:
            list: Segments with 'direction', 'start', 'end', 'us_delta', 'duration'
        """
        # Calculate window size from actual data timing
        if len(self.data) < 10:
            return []

        # Estimate sample rate from data
        duration = self.data[-1]['time_s'] - self.data[0]['time_s']
        if duration <= 0:
            sample_rate = 500  # Default
        else:
            sample_rate = len(self.data) / duration

        # Convert window_ms to samples
        window_size = max(10, int(self.window_ms * sample_rate / 1000))

        # Apply low-pass filter to smooth ultrasonic readings
        us_raw = [s['us'] for s in self.data]
        us_smooth = self._low_pass_filter(us_raw)

        if len(self.data) < window_size * 2:
            # Not enough data - treat as single segment
            return [{
                'direction': self._detect_direction(us_smooth[0], us_smooth[-1]),
                'start': 0,
                'end': len(self.data),
                'us_delta': us_smooth[-1] - us_smooth[0],
                'duration': self.data[-1]['time_s'] - self.data[0]['time_s']
            }]

        segments = []
        i = 0

        while i < len(self.data) - window_size:
            # Look at this window (using smoothed data)
            window_start = us_smooth[i]
            window_end = us_smooth[i + window_size - 1]
            direction = self._detect_direction(window_start, window_end)

            # Find how long this direction continues
            segment_start = i
            while i < len(self.data) - window_size:
                next_start = us_smooth[i]
                next_end = us_smooth[min(i + window_size - 1, len(us_smooth) - 1)]
                next_dir = self._detect_direction(next_start, next_end)

                if next_dir != direction:
                    break
                i += window_size // 2  # Overlap windows

            segment_end = min(i + window_size, len(self.data))

            # Only save if segment is meaningful
            if segment_end - segment_start >= window_size:
                # Use smoothed values for delta calculation too
                us_delta = us_smooth[segment_end - 1] - us_smooth[segment_start]
                seg_data = self.data[segment_start:segment_end]
                segments.append({
                    'direction': direction,
                    'start': segment_start,
                    'end': segment_end,
                    'us_delta': us_delta,
                    'duration': seg_data[-1]['time_s'] - seg_data[0]['time_s']
                })

        return segments

    def _find_motion_segments(self):
        """
        Find segments where the robot is moving (legacy method).

        Uses motor values if available, falls back to ultrasonic.

        Returns:
            list: List of (start, end) index tuples
        """
        segments = []
        in_motion = False
        start_idx = 0
        motor_threshold = 10

        for i, sample in enumerate(self.data):
            # Check if motors are running (if data available)
            motor_l = sample.get('motor_l', 0)
            motor_r = sample.get('motor_r', 0)
            is_moving = abs(motor_l) > motor_threshold or abs(motor_r) > motor_threshold

            if is_moving and not in_motion:
                start_idx = i
                in_motion = True
            elif not is_moving and in_motion:
                if i - start_idx > 10:
                    segments.append((start_idx, i))
                in_motion = False

        if in_motion and len(self.data) - start_idx > 10:
            segments.append((start_idx, len(self.data) - 1))

        return segments

    def analyze(self):
        """
        Perform full bump analysis on loaded data.

        This method:
        1. Calculates acceleration magnitude for each sample
        2. Applies high-pass filter to isolate bumps
        3. Counts threshold crossings as bumps
        4. Compares to ultrasonic distance for B/cm calculation
        5. Analyzes motion segments separately

        Returns:
            dict: Analysis results including:
                - total_bumps: Total bumps detected
                - duration: Total time in seconds
                - us_start/us_end: Ultrasonic readings
                - us_distance: Distance from ultrasonic (cm)
                - bumps_per_cm: Calibration ratio
                - segments: Per-segment analysis
        """
        if not self.data:
            return {'error': 'No data loaded'}

        # Step 1: Calculate acceleration magnitude based on mode
        # Mode 'xy' (default): sqrt(ax² + ay²) - horizontal plane resultant
        # Mode 'xyz': sqrt(ax² + ay² + az²) - 1.0 (full 3D minus gravity)
        # Mode 'x'/'y': single axis only
        magnitudes = [self._calc_magnitude(sample) for sample in self.data]

        # Also compute individual components for comparison
        ax_vals = [s['ax'] for s in self.data]
        ay_vals = [s['ay'] for s in self.data]
        xy_mags = [math.sqrt(s['ax']**2 + s['ay']**2) for s in self.data]

        # Step 2: Apply high-pass filter
        filtered = self._high_pass_filter(magnitudes)

        # Step 3: Count bumps (threshold crossings)
        detected_bumps, bump_indices = self._count_bumps(filtered)

        # Step 4: Get ultrasonic distance
        us_start = self.data[0]['us']
        us_end = self.data[-1]['us']
        us_distance = abs(us_end - us_start)

        # Step 5: Use robot's bump counter if available
        robot_bumps_start = self.data[0]['bumps']
        robot_bumps_end = self.data[-1]['bumps']
        robot_bumps = robot_bumps_end - robot_bumps_start

        # Step 6: Calculate B/cm
        # Use robot's counter (more accurate) if available
        total_bumps = robot_bumps if robot_bumps > 0 else detected_bumps
        bumps_per_cm = total_bumps / us_distance if us_distance > 1 else 0

        # Step 7: Calculate duration
        duration = self.data[-1]['time_s'] - self.data[0]['time_s']

        # Step 8: Partition by direction (using ultrasonic)
        direction_segments = self._partition_by_ultrasonic()

        # Gyro threshold for turn detection (deg/s)
        # If average |gz| exceeds this, segment is marked as turning
        TURN_THRESHOLD = 30.0  # degrees/second

        # Analyze each direction segment
        direction_results = []
        for seg in direction_segments:
            start, end = seg['start'], seg['end']
            seg_data = self.data[start:end]

            # Count bumps in this segment
            seg_mags = magnitudes[start:end]
            seg_filtered = self._high_pass_filter(seg_mags)
            seg_bumps, _ = self._count_bumps(seg_filtered)

            # Also check robot counter if available
            robot_seg_bumps = seg_data[-1].get('bumps', 0) - seg_data[0].get('bumps', 0)
            if robot_seg_bumps > 0:
                seg_bumps = robot_seg_bumps

            seg_us_dist = abs(seg['us_delta'])
            seg_duration = seg['duration']

            # Check gyro for turning - use gz (yaw rate)
            gz_values = [abs(s.get('gz', 0)) for s in seg_data]
            avg_gz = sum(gz_values) / len(gz_values) if gz_values else 0
            max_gz = max(gz_values) if gz_values else 0
            is_turning = avg_gz > TURN_THRESHOLD

            # Segment is valid for B/cm if:
            # - Moving (FORWARD or BACKWARD)
            # - Not turning significantly
            # - Has meaningful distance (> 2cm)
            is_valid = (
                seg['direction'] in ('FORWARD', 'BACKWARD') and
                not is_turning and
                seg_us_dist > 2.0
            )

            direction_results.append({
                'direction': seg['direction'],
                'start_idx': start,
                'end_idx': end,
                'duration': seg_duration,
                'us_distance': seg_us_dist,
                'bumps': seg_bumps,
                'bumps_per_cm': seg_bumps / seg_us_dist if seg_us_dist > 1 else 0,
                'speed_cms': seg_us_dist / seg_duration if seg_duration > 0.1 else 0,
                'bump_rate_hz': seg_bumps / seg_duration if seg_duration > 0.1 else 0,
                'avg_gz': avg_gz,
                'max_gz': max_gz,
                'is_turning': is_turning,
                'is_valid': is_valid,
            })

        # Aggregate by direction type (only valid segments for stats)
        by_direction = {'FORWARD': [], 'BACKWARD': [], 'STOPPED': []}
        for seg in direction_results:
            by_direction[seg['direction']].append(seg)

        # Calculate average B/cm per direction (only valid segments!)
        direction_stats = {}
        for direction, segs in by_direction.items():
            if segs:
                # Filter to valid segments only for B/cm calculation
                valid_segs = [s for s in segs if s['is_valid']]
                all_count = len(segs)
                valid_count = len(valid_segs)

                # Stats from ALL segments (including invalid)
                total_bumps = sum(s['bumps'] for s in segs)
                total_dist = sum(s['us_distance'] for s in segs)
                total_time = sum(s['duration'] for s in segs)

                # B/cm only from VALID segments (not turning, moving)
                valid_bumps = sum(s['bumps'] for s in valid_segs)
                valid_dist = sum(s['us_distance'] for s in valid_segs)
                valid_time = sum(s['duration'] for s in valid_segs)

                direction_stats[direction] = {
                    'count': all_count,
                    'valid_count': valid_count,
                    'total_bumps': total_bumps,
                    'total_distance': total_dist,
                    'total_time': total_time,
                    # Use valid segments for accurate B/cm
                    'valid_bumps': valid_bumps,
                    'valid_distance': valid_dist,
                    'valid_time': valid_time,
                    'avg_bumps_per_cm': valid_bumps / valid_dist if valid_dist > 1 else 0,
                    'avg_speed_cms': valid_dist / valid_time if valid_time > 0.1 else 0,
                }

        # Calculate overall B/cm from valid FORWARD + BACKWARD segments only
        valid_moving = [s for s in direction_results
                       if s['is_valid'] and s['direction'] in ('FORWARD', 'BACKWARD')]
        valid_total_bumps = sum(s['bumps'] for s in valid_moving)
        valid_total_dist = sum(s['us_distance'] for s in valid_moving)
        valid_bumps_per_cm = valid_total_bumps / valid_total_dist if valid_total_dist > 1 else 0

        self.results = {
            'total_samples': len(self.data),
            'duration': duration,
            'detected_bumps': detected_bumps,
            'robot_bumps': robot_bumps,
            'total_bumps': total_bumps,
            'us_start': us_start,
            'us_end': us_end,
            'us_distance': us_distance,
            'bumps_per_cm': bumps_per_cm,  # Raw (all data)
            # Valid B/cm (only straight-line FWD/BACK segments)
            'valid_bumps_per_cm': valid_bumps_per_cm,
            'valid_total_bumps': valid_total_bumps,
            'valid_total_dist': valid_total_dist,
            'valid_segment_count': len(valid_moving),
            'direction_segments': direction_results,
            'direction_stats': direction_stats,
            'filtered_signal': filtered,
            'bump_indices': bump_indices,
            'magnitudes': magnitudes,
            # Individual axis data for comparison
            'mag_mode': self.mag_mode,
            'ax_vals': ax_vals,
            'ay_vals': ay_vals,
            'xy_mags': xy_mags,
        }

        return self.results

    def print_report(self):
        """Print a formatted analysis report."""
        if not self.results:
            self.analyze()

        r = self.results

        print("\n" + "=" * 60)
        print("BUMP DETECTION ANALYSIS REPORT")
        print("=" * 60)

        mode_desc = {
            'xy': 'Horizontal resultant sqrt(ax² + ay²)',
            'xyz': 'Full 3D sqrt(ax² + ay² + az²) - 1',
            'x': 'Forward axis (ax) only',
            'y': 'Side axis (ay) only',
        }
        mag_mode = r.get('mag_mode', 'xy')

        print(f"\nData Overview:")
        print(f"  Samples:     {r['total_samples']}")
        print(f"  Duration:    {r['duration']:.2f} seconds")
        print(f"  Sample Rate: {self.sample_rate:.1f} Hz (auto-detected)")
        print(f"  Mag Mode:    {mag_mode} - {mode_desc.get(mag_mode, 'unknown')}")
        print(f"  Window:      {self.window_ms} ms = {int(self.window_ms * self.sample_rate / 1000)} samples")

        print(f"\nUltrasonic Distance:")
        print(f"  Start:       {r['us_start']:.1f} cm")
        print(f"  End:         {r['us_end']:.1f} cm")
        print(f"  Travel:      {r['us_distance']:.1f} cm")

        print(f"\nBump Count:")
        print(f"  Detected:    {r['detected_bumps']} (from accelerometer)")
        print(f"  Robot:       {r['robot_bumps']} (from robot counter)")
        print(f"  Used:        {r['total_bumps']}")

        print(f"\n{'=' * 60}")
        print(f"  RAW B/cm     = {r['bumps_per_cm']:.3f}  (all data)")
        print(f"  VALID B/cm   = {r['valid_bumps_per_cm']:.3f}  (straight FWD/BACK only)")
        print(f"{'=' * 60}")
        print(f"  Valid segments: {r['valid_segment_count']}")
        print(f"  Valid distance: {r['valid_total_dist']:.1f} cm")
        print(f"  Valid bumps:    {r['valid_total_bumps']}")

        # Direction-based analysis
        if r.get('direction_stats'):
            print(f"\n" + "-" * 60)
            print("ANALYSIS BY DIRECTION")
            print("-" * 60)

            for direction in ['FORWARD', 'BACKWARD', 'STOPPED']:
                if direction in r['direction_stats']:
                    stats = r['direction_stats'][direction]
                    valid_info = f" ({stats['valid_count']} valid)" if stats['valid_count'] < stats['count'] else ""
                    print(f"\n  {direction}:")
                    print(f"    Segments:  {stats['count']}{valid_info}")
                    print(f"    Distance:  {stats['valid_distance']:.1f} cm (valid)")
                    print(f"    Time:      {stats['valid_time']:.2f} s (valid)")
                    print(f"    Bumps:     {stats['valid_bumps']} (valid)")
                    print(f"    B/cm:      {stats['avg_bumps_per_cm']:.3f}")
                    print(f"    Avg Speed: {stats['avg_speed_cms']:.1f} cm/s")

        if r.get('direction_segments'):
            print(f"\n" + "-" * 60)
            print(f"SEGMENT DETAILS ({len(r['direction_segments'])} segments)")
            print("-" * 60)
            print(f"  {'#':>2} {'Dir':>8} {'Time':>6} {'Dist':>6} {'Bumps':>6} {'B/cm':>6} {'Gz':>5} {'Valid':>5}")
            print("  " + "-" * 56)
            for i, seg in enumerate(r['direction_segments']):
                valid_mark = "✓" if seg['is_valid'] else "✗"
                turn_mark = "↻" if seg['is_turning'] else ""
                print(f"  {i+1:2} {seg['direction']:>8} "
                      f"{seg['duration']:5.2f}s {seg['us_distance']:5.1f}cm "
                      f"{seg['bumps']:5} {seg['bumps_per_cm']:5.2f} "
                      f"{seg['avg_gz']:4.0f}° {valid_mark:>3}{turn_mark}")

    def plot(self, save_path=None):
        """
        Visualize the signal processing pipeline with direction segments.

        Shows 4 subplots:
        1. Original acceleration magnitude with direction colors
        2. High-pass filtered signal with threshold
        3. Detected bumps overlay
        4. Ultrasonic distance with direction segments

        Args:
            save_path: If provided, save figure to this path instead of showing
        """
        if not MATPLOTLIB_AVAILABLE:
            print("matplotlib not installed. Run: pip install matplotlib")
            return

        if not self.results:
            self.analyze()

        # Prepare data
        times = [s['time_s'] for s in self.data]
        magnitudes = self.results.get('magnitudes', [
            math.sqrt(s['ax']**2 + s['ay']**2 + s['az']**2) - 1.0
            for s in self.data
        ])
        filtered = self.results['filtered_signal']
        bump_indices = self.results['bump_indices']
        us_data = [s['us'] for s in self.data]

        # Direction colors
        dir_colors = {
            'FORWARD': '#2ecc71',   # Green
            'BACKWARD': '#e74c3c',  # Red
            'STOPPED': '#95a5a6',   # Gray
        }

        # Create figure with 4 subplots
        fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
        fig.suptitle(f"Bump Detection Analysis - Overall B/cm = {self.results['bumps_per_cm']:.3f}",
                    fontsize=14, fontweight='bold')

        # Plot 1: Original magnitude with direction segments
        ax1 = axes[0]
        ax1.plot(times, magnitudes, 'b-', linewidth=0.5, alpha=0.4)

        # Color background by direction
        for seg in self.results.get('direction_segments', []):
            t_start = times[seg['start_idx']] if seg['start_idx'] < len(times) else times[-1]
            t_end = times[min(seg['end_idx']-1, len(times)-1)]
            color = dir_colors.get(seg['direction'], 'gray')
            ax1.axvspan(t_start, t_end, alpha=0.2, color=color)

        ax1.set_ylabel('Magnitude (g)')
        ax1.set_title('Step 1: Raw Acceleration Magnitude (colored by direction)')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

        # Plot 2: High-pass filtered signal
        ax2 = axes[1]
        ax2.plot(times, filtered, 'g-', linewidth=0.5, alpha=0.7)
        ax2.axhline(y=self.threshold, color='r', linestyle='--',
                   label=f'Threshold = {self.threshold}', linewidth=1.5)
        ax2.axhline(y=-self.threshold, color='r', linestyle='--', linewidth=1.5)
        ax2.set_ylabel('Filtered (g)')
        ax2.set_title(f'Step 2: High-Pass Filtered (α = {self.hp_alpha})')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Filtered signal with bump markers
        ax3 = axes[2]
        ax3.plot(times, filtered, 'g-', linewidth=0.5, alpha=0.5)
        bump_times = [times[i] for i in bump_indices if i < len(times)]
        bump_values = [filtered[i] for i in bump_indices if i < len(filtered)]
        ax3.scatter(bump_times, bump_values, c='red', s=20, marker='v',
                   label=f'Bumps detected: {len(bump_indices)}', zorder=5)
        ax3.axhline(y=self.threshold, color='r', linestyle='--', linewidth=1)
        ax3.set_ylabel('Filtered (g)')
        ax3.set_title(f'Step 3: Threshold Crossing Detection ({len(bump_indices)} bumps)')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)

        # Plot 4: Ultrasonic distance with direction annotations
        ax4 = axes[3]
        ax4.plot(times, us_data, 'b-', linewidth=1.5)

        # Color and annotate segments
        for seg in self.results.get('direction_segments', []):
            t_start = times[seg['start_idx']] if seg['start_idx'] < len(times) else times[-1]
            t_end = times[min(seg['end_idx']-1, len(times)-1)]
            color = dir_colors.get(seg['direction'], 'gray')
            ax4.axvspan(t_start, t_end, alpha=0.3, color=color)

            # Add label at segment center
            t_mid = (t_start + t_end) / 2
            us_mid = us_data[min((seg['start_idx'] + seg['end_idx']) // 2, len(us_data)-1)]
            if seg['us_distance'] > 3:  # Only label significant segments
                ax4.annotate(f"{seg['direction']}\n{seg['bumps_per_cm']:.2f} B/cm",
                           xy=(t_mid, us_mid), fontsize=7, ha='center',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

        ax4.set_ylabel('Distance (cm)')
        ax4.set_xlabel('Time (s)')
        ax4.set_title(f'Ultrasonic Distance (Green=FWD, Red=BWD, Gray=STOPPED)')
        ax4.grid(True, alpha=0.3)

        # Add legend for directions
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=dir_colors['FORWARD'], alpha=0.5, label='FORWARD'),
            Patch(facecolor=dir_colors['BACKWARD'], alpha=0.5, label='BACKWARD'),
            Patch(facecolor=dir_colors['STOPPED'], alpha=0.5, label='STOPPED'),
        ]
        ax4.legend(handles=legend_elements, loc='upper right', fontsize=8)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved plot to: {save_path}")
        else:
            plt.show()

        return fig


def main():
    """Command-line interface for bump detection."""
    parser = argparse.ArgumentParser(
        description='Analyze wheel bumps and calculate B/cm calibration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Magnitude Modes:
  xy   - Horizontal resultant sqrt(ax² + ay²) [RECOMMENDED]
         Best for two-wheel robots: captures both wheels regardless of sync
  xyz  - Full 3D sqrt(ax² + ay² + az²) - 1 (removes gravity)
  x    - Forward axis (ax) only
  y    - Side axis (ay) only

Examples:
  python bump_detector.py recording.csv --plot
  python bump_detector.py recording.csv --mode xy --plot
  python bump_detector.py recording.csv --mode xyz --threshold 0.02
  python bump_detector.py data/calibration_run.csv --alpha 0.98 --save plot.png
        """
    )

    parser.add_argument('csv_file', help='CSV file with recorded data')
    parser.add_argument('--threshold', '-t', type=float, default=0.015,
                       help='Bump detection threshold in g (default: 0.015)')
    parser.add_argument('--alpha', '-a', type=float, default=0.95,
                       help='High-pass filter alpha (default: 0.95)')
    parser.add_argument('--mode', '-m', default='xy',
                       choices=['xy', 'xyz', 'x', 'y'],
                       help='Magnitude mode: xy (horizontal, recommended), xyz (3D), x, y (default: xy)')
    parser.add_argument('--window', '-w', type=int, default=500,
                       help='Window size in ms for direction detection (default: 500, try 1000-2000 for longer movements)')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Show visualization of signal processing steps')
    parser.add_argument('--save', '-s', type=str, default=None,
                       help='Save plot to file instead of showing')

    args = parser.parse_args()

    # Create detector and analyze
    detector = BumpDetector(threshold=args.threshold, hp_alpha=args.alpha,
                           mag_mode=args.mode, window_ms=args.window)
    detector.load_csv(args.csv_file)
    detector.analyze()
    detector.print_report()

    # Show or save plot
    if args.plot or args.save:
        detector.plot(save_path=args.save)

    return detector.results


if __name__ == '__main__':
    main()
