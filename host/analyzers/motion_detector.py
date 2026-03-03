#!/usr/bin/env python3
"""
==============================================================================
MOTION DETECTOR - IMU-Only Movement Classification
==============================================================================

Detects robot movement direction using ONLY accelerometer and gyroscope data.
No ultrasonic sensor needed!

DETECTION APPROACH:
-------------------
1. TURNING detection (gyroscope Z):
   - High |gz| = robot is rotating
   - gz > 0 = turning LEFT (counter-clockwise)
   - gz < 0 = turning RIGHT (clockwise)

2. MOTION detection (accelerometer variance):
   - High variance = robot is moving (wheels vibrating)
   - Low variance = robot is stopped

3. DIRECTION detection (acceleration patterns):
   - Forward/Backward harder to distinguish without reference
   - Use acceleration bias during motion start/stop
   - Or use heading integration to track orientation

CLASSIFICATION LOGIC:
--------------------
  if |gz| > TURN_THRESHOLD:
      if gz > 0: TURN_LEFT
      else: TURN_RIGHT
  elif variance > MOTION_THRESHOLD:
      if has forward acceleration pattern: FORWARD
      else: BACKWARD (or just MOVING if can't distinguish)
  else:
      STOPPED

USAGE:
------
Standalone:
    python motion_detector.py recording.csv
    python motion_detector.py recording.csv --plot

As module:
    from analyzers import MotionDetector
    detector = MotionDetector()
    detector.load_csv('recording.csv')
    segments = detector.analyze()

==============================================================================
"""

import csv
import math
import argparse
import os


class MotionDetector:
    """
    Detects motion state from IMU data only.

    KEY INSIGHT: Wheel bumps create high-frequency vibration noise.
    Useful motion signals (body acceleration, yaw rate) are LOW frequency.

    FILTERING STRATEGY:
    ------------------
    1. Low-pass filter gyro Z at ~20 Hz - removes bump noise from yaw rate
    2. Low-pass filter accel at ~5 Hz - extracts body motion from vibration
    3. High-pass filter accel - isolates vibration energy
    4. Quality gating - only trust accel when vibration is LOW

    TRUST HIERARCHY:
    ---------------
    - Gyro Z (yaw rate): Most reliable - less affected by gravity/bumps
    - Vibration level: Good for detecting motion vs stopped
    - Accel direction: LEAST reliable - only use when vibration is calm

    Motion states:
        STOPPED    - No movement (low vibration, low rotation)
        FORWARD    - Moving forward (calm accel shows forward bias)
        BACKWARD   - Moving backward (calm accel shows backward bias)
        TURN_LEFT  - Rotating counter-clockwise (high +gz)
        TURN_RIGHT - Rotating clockwise (high -gz)
        MOVING     - Motion detected but direction unclear (high vibration)
    """

    # Motion state constants
    STOPPED = 'STOPPED'
    FORWARD = 'FORWARD'
    BACKWARD = 'BACKWARD'
    TURN_LEFT = 'TURN_LEFT'
    TURN_RIGHT = 'TURN_RIGHT'
    MOVING = 'MOVING'  # When can't distinguish FWD/BACK (high vibration)

    def __init__(self, turn_threshold=25.0, motion_threshold=0.02,
                 window_ms=200, accel_bias_threshold=0.05,
                 gyro_lp_cutoff=20.0, accel_lp_cutoff=5.0):
        """
        Initialize motion detector.

        Args:
            turn_threshold: Gyro Z threshold for turn detection (deg/s)
                           Lower = more sensitive to turns
                           Default 25 deg/s works well for typical robots

            motion_threshold: Vibration RMS threshold for motion detection
                             This is HIGH-FREQUENCY energy, not raw variance
                             Lower = more sensitive to motion
                             Default 0.02 works for most surfaces

            window_ms: Analysis window size in milliseconds
                      Larger = smoother, but slower response
                      Default 200ms is good balance

            accel_bias_threshold: Acceleration bias for FWD/BACK detection
                                 ONLY used when vibration is LOW (calm IMU)
                                 During acceleration, robot tilts slightly
                                 Forward motion = positive ax bias (tilts back)
                                 Backward motion = negative ax bias (tilts forward)

            gyro_lp_cutoff: Low-pass filter cutoff for gyro (Hz)
                           Default 20 Hz - removes high-freq bump noise
                           Gyro is more reliable than accel for direction

            accel_lp_cutoff: Low-pass filter cutoff for accel (Hz)
                            Default 5 Hz - extracts body motion from vibration
                            Lower = smoother but slower response
        """
        self.turn_threshold = turn_threshold
        self.motion_threshold = motion_threshold
        self.window_ms = window_ms
        self.accel_bias_threshold = accel_bias_threshold
        self.gyro_lp_cutoff = gyro_lp_cutoff
        self.accel_lp_cutoff = accel_lp_cutoff

        self.sample_rate = 500  # Auto-detected from data
        self.data = []
        self.filtered_data = {}  # Stores filtered signals
        self.results = None

    def load_csv(self, filename):
        """Load data from CSV file."""
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
                    }
                    self.data.append(sample)
                except (ValueError, KeyError):
                    continue

        n = len(self.data)
        print(f"Loaded {n} samples from {filename}")

        # Auto-detect sample rate
        if n > 10:
            duration = self.data[-1]['time_s'] - self.data[0]['time_s']
            if duration > 0:
                self.sample_rate = (n - 1) / duration
                print(f"Sample rate: {self.sample_rate:.1f} Hz (auto-detected)")

        return n

    def _lowpass_alpha(self, cutoff_hz):
        """
        Calculate exponential filter alpha for given cutoff frequency.

        The relationship between cutoff frequency and alpha:
            alpha = dt / (RC + dt)
            where RC = 1 / (2 * pi * cutoff_hz)

        Args:
            cutoff_hz: Desired cutoff frequency in Hz

        Returns:
            float: Alpha value for exponential filter (0 < alpha < 1)
        """
        if self.sample_rate <= 0 or cutoff_hz <= 0:
            return 0.5
        dt = 1.0 / self.sample_rate
        rc = 1.0 / (2 * math.pi * cutoff_hz)
        alpha = dt / (rc + dt)
        return min(1.0, max(0.01, alpha))

    def _apply_lowpass(self, values, cutoff_hz):
        """
        Apply low-pass filter to extract slow-changing component.

        Removes high-frequency noise (wheel bumps), keeps body motion.

        Args:
            values: List of sample values
            cutoff_hz: Cutoff frequency in Hz

        Returns:
            list: Filtered values
        """
        if not values:
            return []

        alpha = self._lowpass_alpha(cutoff_hz)
        filtered = [values[0]]

        for i in range(1, len(values)):
            # y[n] = alpha * x[n] + (1-alpha) * y[n-1]
            filtered.append(alpha * values[i] + (1 - alpha) * filtered[-1])

        return filtered

    def _apply_highpass(self, values, cutoff_hz):
        """
        Apply high-pass filter to extract fast-changing component (vibration).

        HP = signal - LP(signal)

        Args:
            values: List of sample values
            cutoff_hz: Cutoff frequency in Hz

        Returns:
            list: High-frequency component (vibration)
        """
        lowpassed = self._apply_lowpass(values, cutoff_hz)
        return [v - lp for v, lp in zip(values, lowpassed)]

    def _calc_rms(self, values):
        """Calculate RMS (Root Mean Square) of values."""
        if not values:
            return 0
        return math.sqrt(sum(v * v for v in values) / len(values))

    def auto_detect_threshold(self, baseline_seconds=0.5):
        """
        Auto-detect motion threshold from stationary baseline at start of recording.

        Assumes robot is stationary for first `baseline_seconds`.
        Sets threshold = mean + 3*std of vibration during baseline.

        Args:
            baseline_seconds: How many seconds at start to use (default 0.5)

        Returns:
            float: Detected motion threshold
        """
        if not self.data:
            return 0.02  # Default

        # Calculate how many samples = baseline
        baseline_samples = int(baseline_seconds * self.sample_rate)
        baseline_samples = min(baseline_samples, len(self.data) // 2)  # Max half of data

        if baseline_samples < 10:
            print(f"Warning: Not enough baseline samples ({baseline_samples}), using default threshold")
            return 0.02

        # Calculate vibration magnitude for baseline period
        vib_baseline = []
        for i in range(baseline_samples):
            s = self.data[i]
            # High-pass estimate: deviation from mean
            ax, ay = s['ax'], s['ay']
            vib_baseline.append(math.sqrt(ax*ax + ay*ay))

        # Stats
        mean_vib = sum(vib_baseline) / len(vib_baseline)
        variance = sum((v - mean_vib)**2 for v in vib_baseline) / len(vib_baseline)
        std_vib = math.sqrt(variance)

        # Threshold = mean + 3*std (99.7% of stationary noise below this)
        threshold = mean_vib + 3 * std_vib

        # Clamp to reasonable range
        threshold = max(0.005, min(0.2, threshold))

        print(f"Auto-detected motion threshold:")
        print(f"  Baseline: {baseline_seconds}s ({baseline_samples} samples)")
        print(f"  Stationary vib: {mean_vib:.4f} ± {std_vib:.4f}")
        print(f"  Threshold: {threshold:.4f} (mean + 3σ)")

        self.motion_threshold = threshold
        return threshold

    def preprocess(self):
        """
        Apply filtering to separate useful signals from vibration noise.

        Creates:
            - gz_lp: Low-pass filtered gyro Z (clean yaw rate)
            - ax_lp, ay_lp: Low-pass filtered accel (body motion)
            - ax_hp, ay_hp: High-pass filtered accel (vibration)
            - vib_rms: Vibration RMS over sliding window (IMU quality metric)

        The vibration RMS is KEY for "quality gating":
            - High vib_rms = bumpy, don't trust accel for direction
            - Low vib_rms = calm, can use accel to detect FWD/BACK
        """
        if not self.data:
            return

        # Extract raw signals
        ax_raw = [s['ax'] for s in self.data]
        ay_raw = [s['ay'] for s in self.data]
        gz_raw = [s['gz'] for s in self.data]

        # Low-pass filter: extract body motion (slow changes)
        self.filtered_data['gz_lp'] = self._apply_lowpass(gz_raw, self.gyro_lp_cutoff)
        self.filtered_data['ax_lp'] = self._apply_lowpass(ax_raw, self.accel_lp_cutoff)
        self.filtered_data['ay_lp'] = self._apply_lowpass(ay_raw, self.accel_lp_cutoff)

        # High-pass filter: extract vibration (fast changes = wheel bumps)
        self.filtered_data['ax_hp'] = self._apply_highpass(ax_raw, self.accel_lp_cutoff)
        self.filtered_data['ay_hp'] = self._apply_highpass(ay_raw, self.accel_lp_cutoff)

        # Compute vibration magnitude: sqrt(ax_hp² + ay_hp²)
        self.filtered_data['vib_mag'] = [
            math.sqrt(ax**2 + ay**2)
            for ax, ay in zip(self.filtered_data['ax_hp'], self.filtered_data['ay_hp'])
        ]

        # Sliding window RMS of vibration (quality metric)
        window_samples = max(10, int(0.1 * self.sample_rate))  # 100ms window
        vib_rms = []
        for i in range(len(self.data)):
            start = max(0, i - window_samples // 2)
            end = min(len(self.data), i + window_samples // 2)
            window = self.filtered_data['vib_mag'][start:end]
            vib_rms.append(self._calc_rms(window))
        self.filtered_data['vib_rms'] = vib_rms

        # Get filtered signals as local vars
        dt = 1.0 / self.sample_rate
        ax_lp = self.filtered_data['ax_lp']
        gz_lp = self.filtered_data['gz_lp']
        vib_mag = self.filtered_data['vib_mag']

        # Compute jerk (derivative of low-pass accel) for direction at transitions
        ax_jerk = [0]
        for i in range(1, len(ax_lp)):
            ax_jerk.append((ax_lp[i] - ax_lp[i-1]) / dt)
        self.filtered_data['ax_jerk'] = self._apply_lowpass(ax_jerk, 10.0)

        # DIRECTION LATCH: Capture ax direction at motion START, hold during motion
        # Key insight: ax is only reliable at the moment vibration starts rising
        # After that it's too noisy - so we "latch" the direction and hold it

        # Low-pass filter vib_rms FIRST to detect slower motion starts
        # Without this, fast bumps cause false "motion start" triggers
        vib_rms_smooth = self._apply_lowpass(vib_rms, 2.0)  # 2Hz = catches ~0.5s ramps
        self.filtered_data['vib_rms_smooth'] = vib_rms_smooth

        # Compute vibration derivative from SMOOTHED vib (detects motion start/stop)
        vib_deriv = [0]
        for i in range(1, len(vib_rms_smooth)):
            vib_deriv.append((vib_rms_smooth[i] - vib_rms_smooth[i-1]) / dt)
        # Smooth derivative too for cleaner trigger
        vib_deriv_smooth = self._apply_lowpass(vib_deriv, 1.0)  # 1Hz = very smooth
        self.filtered_data['vib_deriv'] = vib_deriv_smooth

        # Direction latch logic:
        # - When vib crosses threshold AND vib_deriv > 0 (rising) → sample ax, latch direction
        # - Hold latched direction while vib > threshold
        # - When vib drops below threshold → release latch
        latched_direction = 0  # -1=BACK, 0=NONE, +1=FWD
        direction_latch = []   # Per-sample latched direction
        start_samples = 10     # How many samples to average at motion start

        in_motion = False
        start_idx = 0

        for i in range(len(self.data)):
            vib = vib_rms_smooth[i]  # Use smoothed for stable threshold
            vib_rising = vib_deriv_smooth[i] > 0.005  # Lower threshold for smoother signal

            if not in_motion and vib > self.motion_threshold and vib_rising:
                # Motion just started! Capture direction from next few samples
                in_motion = True
                start_idx = i
                # Average ax over the start window (use more samples for smoother estimate)
                end_idx = min(i + start_samples * 2, len(ax_lp))
                ax_at_start = self._calc_mean(ax_lp[i:end_idx])
                # Latch the direction
                # Note: When accelerating FORWARD, robot tilts BACK → negative ax
                #       When accelerating BACKWARD, robot tilts FORWARD → positive ax
                if ax_at_start < -0.015:  # Negative ax = forward acceleration
                    latched_direction = 1  # FORWARD
                elif ax_at_start > 0.015:  # Positive ax = backward acceleration
                    latched_direction = -1  # BACKWARD
                else:
                    latched_direction = 0  # Can't determine

            elif in_motion and vib < self.motion_threshold * 0.5:
                # Motion stopped - release latch
                in_motion = False
                latched_direction = 0

            direction_latch.append(latched_direction)

        self.filtered_data['direction_latch'] = direction_latch
        self.filtered_data['direction_metric'] = vib_deriv_smooth  # For visualization
        self.filtered_data['cumulative_direction'] = direction_latch  # Reuse for compat

        # Heading from integrated gyro Z (for tracking rotation)
        heading = [0]
        for i in range(1, len(gz_lp)):
            heading.append(heading[-1] + gz_lp[i] * dt)
        self.filtered_data['heading'] = heading

        print(f"Preprocessing complete:")
        print(f"  Gyro LP cutoff:  {self.gyro_lp_cutoff} Hz")
        print(f"  Accel LP cutoff: {self.accel_lp_cutoff} Hz")
        print(f"  Vibration RMS range: {min(vib_rms):.4f} - {max(vib_rms):.4f}")
        print(f"  Heading range: {min(heading):.1f}° to {max(heading):.1f}°")

    def _calc_variance(self, values):
        """Calculate variance of a list of values."""
        if len(values) < 2:
            return 0
        mean = sum(values) / len(values)
        return sum((x - mean) ** 2 for x in values) / len(values)

    def _calc_mean(self, values):
        """Calculate mean of a list of values."""
        if not values:
            return 0
        return sum(values) / len(values)

    def _classify_window(self, start_idx, end_idx):
        """
        Classify motion state for a window of samples.

        Uses FILTERED signals for classification:
        - gz_lp (low-pass gyro) for turn detection
        - vib_rms (high-pass accel RMS) for motion detection
        - ax_lp (low-pass accel) for direction ONLY when vib is LOW

        Quality gating: Don't trust accel direction when vibration is high!

        Args:
            start_idx: Start index in data
            end_idx: End index in data

        Returns:
            tuple: (state, confidence, details)
        """
        if end_idx - start_idx < 5:
            return self.STOPPED, 0, {}

        # Use filtered signals
        gz_lp = self.filtered_data['gz_lp'][start_idx:end_idx]
        vib_rms = self.filtered_data['vib_rms'][start_idx:end_idx]
        dir_latch = self.filtered_data['direction_latch'][start_idx:end_idx]

        # Statistics on filtered data
        gz_mean = self._calc_mean(gz_lp)
        gz_abs_mean = self._calc_mean([abs(g) for g in gz_lp])
        vib_mean = self._calc_mean(vib_rms)

        # Latched direction: majority vote in this window
        # +1 = FWD, -1 = BACK, 0 = unknown
        latch_sum = sum(dir_latch)
        latched_dir = 1 if latch_sum > len(dir_latch) * 0.3 else (-1 if latch_sum < -len(dir_latch) * 0.3 else 0)

        # IMU quality: inverse of vibration (high vib = low quality for raw ax)
        imu_quality = max(0, 1.0 - vib_mean / (self.motion_threshold * 5))

        details = {
            'gz_mean': gz_mean,
            'gz_abs_mean': gz_abs_mean,
            'latched_dir': latched_dir,
            'latch_sum': latch_sum,
            'dir_metric': latch_sum,  # For compat
            'cumul_dir': latched_dir,  # For compat
            'vib_rms': vib_mean,
            'imu_quality': imu_quality,
        }

        # Classification logic

        # 1. Check for turning (MOST RELIABLE - gyro less affected by bumps)
        if gz_abs_mean > self.turn_threshold:
            if gz_mean > 0:
                confidence = min(1.0, gz_abs_mean / (self.turn_threshold * 2))
                return self.TURN_LEFT, confidence, details
            else:
                confidence = min(1.0, gz_abs_mean / (self.turn_threshold * 2))
                return self.TURN_RIGHT, confidence, details

        # 2. Check for motion using VIBRATION RMS (high-freq energy)
        if vib_mean > self.motion_threshold:
            # Robot is moving - use LATCHED direction (captured at motion start)

            if latched_dir > 0:
                confidence = 0.8  # Good confidence - captured at clean moment
                return self.FORWARD, confidence, details
            elif latched_dir < 0:
                confidence = 0.8
                return self.BACKWARD, confidence, details
            else:
                # Couldn't determine direction at start
                confidence = min(1.0, vib_mean / (self.motion_threshold * 3))
                return self.MOVING, confidence, details

        # 3. No significant motion or rotation = STOPPED
        confidence = 1.0 - min(1.0, vib_mean / self.motion_threshold)
        return self.STOPPED, confidence, details

    def analyze(self):
        """
        Analyze motion throughout the recording.

        Steps:
        1. Preprocess: Apply LP/HP filters, compute vibration RMS
        2. Classify: Sliding window classification with quality gating
        3. Merge: Combine consecutive same-state segments
        4. Statistics: Compute per-state summaries

        Returns:
            dict: Analysis results with segments and statistics
        """
        if not self.data:
            return {'error': 'No data loaded'}

        # Step 1: Preprocess - apply filters
        self.preprocess()

        # Calculate window size in samples
        window_size = max(10, int(self.window_ms * self.sample_rate / 1000))
        step_size = window_size // 2  # 50% overlap

        # Step 2: Classify each window
        classifications = []
        i = 0
        while i < len(self.data) - window_size:
            end_idx = i + window_size
            state, confidence, details = self._classify_window(i, end_idx)

            classifications.append({
                'start_idx': i,
                'end_idx': end_idx,
                'start_time': self.data[i]['time_s'],
                'end_time': self.data[end_idx - 1]['time_s'],
                'state': state,
                'confidence': confidence,
                **details
            })
            i += step_size

        # Step 3: Merge consecutive same-state segments
        segments = []
        if classifications:
            current = {
                'state': classifications[0]['state'],
                'start_idx': classifications[0]['start_idx'],
                'start_time': classifications[0]['start_time'],
                'confidences': [classifications[0]['confidence']],
                'gz_means': [classifications[0]['gz_mean']],
                'dir_metrics': [classifications[0]['dir_metric']],
                'cumul_dirs': [classifications[0]['cumul_dir']],
                'vib_rms_vals': [classifications[0]['vib_rms']],
                'imu_qualities': [classifications[0]['imu_quality']],
            }

            for c in classifications[1:]:
                if c['state'] == current['state']:
                    # Extend current segment
                    current['confidences'].append(c['confidence'])
                    current['gz_means'].append(c['gz_mean'])
                    current['dir_metrics'].append(c['dir_metric'])
                    current['cumul_dirs'].append(c['cumul_dir'])
                    current['vib_rms_vals'].append(c['vib_rms'])
                    current['imu_qualities'].append(c['imu_quality'])
                else:
                    # Save current and start new
                    prev_class = classifications[classifications.index(c) - 1]
                    current['end_idx'] = prev_class['end_idx']
                    current['end_time'] = prev_class['end_time']
                    current['duration'] = current['end_time'] - current['start_time']
                    current['avg_confidence'] = self._calc_mean(current['confidences'])
                    current['avg_gz'] = self._calc_mean(current['gz_means'])
                    current['total_dir_metric'] = sum(current['dir_metrics'])
                    current['final_cumul_dir'] = current['cumul_dirs'][-1] if current['cumul_dirs'] else 0
                    current['avg_vib_rms'] = self._calc_mean(current['vib_rms_vals'])
                    current['avg_imu_quality'] = self._calc_mean(current['imu_qualities'])
                    del current['confidences'], current['gz_means'], current['dir_metrics'], current['cumul_dirs'], current['vib_rms_vals'], current['imu_qualities']
                    segments.append(current)

                    current = {
                        'state': c['state'],
                        'start_idx': c['start_idx'],
                        'start_time': c['start_time'],
                        'confidences': [c['confidence']],
                        'gz_means': [c['gz_mean']],
                        'dir_metrics': [c['dir_metric']],
                        'cumul_dirs': [c['cumul_dir']],
                        'vib_rms_vals': [c['vib_rms']],
                        'imu_qualities': [c['imu_quality']],
                    }

            # Don't forget last segment
            current['end_idx'] = classifications[-1]['end_idx']
            current['end_time'] = classifications[-1]['end_time']
            current['duration'] = current['end_time'] - current['start_time']
            current['avg_confidence'] = self._calc_mean(current['confidences'])
            current['avg_gz'] = self._calc_mean(current['gz_means'])
            current['total_dir_metric'] = sum(current['dir_metrics'])
            current['final_cumul_dir'] = current['cumul_dirs'][-1] if current['cumul_dirs'] else 0
            current['avg_vib_rms'] = self._calc_mean(current['vib_rms_vals'])
            current['avg_imu_quality'] = self._calc_mean(current['imu_qualities'])
            del current['confidences'], current['gz_means'], current['dir_metrics'], current['cumul_dirs'], current['vib_rms_vals'], current['imu_qualities']
            segments.append(current)

        # Calculate statistics per state
        stats = {}
        for state in [self.STOPPED, self.FORWARD, self.BACKWARD,
                      self.TURN_LEFT, self.TURN_RIGHT, self.MOVING]:
            state_segs = [s for s in segments if s['state'] == state]
            if state_segs:
                stats[state] = {
                    'count': len(state_segs),
                    'total_time': sum(s['duration'] for s in state_segs),
                    'avg_duration': self._calc_mean([s['duration'] for s in state_segs]),
                    'avg_confidence': self._calc_mean([s['avg_confidence'] for s in state_segs]),
                }

        # Total duration
        total_duration = self.data[-1]['time_s'] - self.data[0]['time_s']

        self.results = {
            'total_samples': len(self.data),
            'total_duration': total_duration,
            'sample_rate': self.sample_rate,
            'window_ms': self.window_ms,
            'segments': segments,
            'classifications': classifications,  # Per-window results
            'stats': stats,
        }

        return self.results

    def get_state_at_time(self, time_s):
        """Get motion state at a specific time."""
        if not self.results:
            self.analyze()

        for seg in self.results['segments']:
            if seg['start_time'] <= time_s <= seg['end_time']:
                return seg['state']
        return self.STOPPED

    def get_timeline(self):
        """
        Get simplified timeline of states.

        Returns:
            list: [(start_time, end_time, state), ...]
        """
        if not self.results:
            self.analyze()

        return [(s['start_time'], s['end_time'], s['state'])
                for s in self.results['segments']]

    def print_report(self):
        """Print formatted analysis report."""
        if not self.results:
            self.analyze()

        r = self.results

        print("\n" + "=" * 60)
        print("IMU MOTION DETECTION REPORT")
        print("=" * 60)

        print(f"\nData Overview:")
        print(f"  Samples:      {r['total_samples']}")
        print(f"  Duration:     {r['total_duration']:.2f} seconds")
        print(f"  Sample Rate:  {r['sample_rate']:.1f} Hz")
        print(f"  Window:       {r['window_ms']} ms")

        print(f"\nFiltering (separating motion from vibration):")
        print(f"  Gyro LP:      {self.gyro_lp_cutoff:.0f} Hz (removes bump noise from yaw)")
        print(f"  Accel LP:     {self.accel_lp_cutoff:.0f} Hz (extracts body motion)")
        print(f"  Vib RMS:      High-pass accel RMS (wheel bump energy)")

        print(f"\nDetection Thresholds:")
        print(f"  Turn:         {self.turn_threshold:.1f} deg/s (gyro Z)")
        print(f"  Motion:       {self.motion_threshold:.4f} vib RMS")
        print(f"  Accel bias:   {self.accel_bias_threshold:.3f} g (only when IMU calm)")

        print(f"\n" + "-" * 60)
        print("MOTION STATISTICS")
        print("-" * 60)

        state_colors = {
            'STOPPED': '⬜', 'FORWARD': '🟢', 'BACKWARD': '🔴',
            'TURN_LEFT': '🔵', 'TURN_RIGHT': '🟣', 'MOVING': '🟡'
        }

        for state in [self.FORWARD, self.BACKWARD, self.MOVING,
                      self.TURN_LEFT, self.TURN_RIGHT, self.STOPPED]:
            if state in r['stats']:
                s = r['stats'][state]
                icon = state_colors.get(state, '')
                pct = s['total_time'] / r['total_duration'] * 100
                print(f"\n  {icon} {state}:")
                print(f"      Segments:   {s['count']}")
                print(f"      Total time: {s['total_time']:.2f}s ({pct:.1f}%)")
                print(f"      Avg length: {s['avg_duration']:.2f}s")
                print(f"      Confidence: {s['avg_confidence']:.1%}")

        print(f"\n" + "-" * 60)
        print(f"SEGMENT TIMELINE ({len(r['segments'])} segments)")
        print("-" * 60)
        print(f"  {'#':>3} {'State':>12} {'Time':>12} {'Dur':>6} {'VibRMS':>7} {'IMU Q':>6} {'Conf':>5}")
        print("  " + "-" * 58)

        for i, seg in enumerate(r['segments']):
            time_range = f"{seg['start_time']:.1f}-{seg['end_time']:.1f}s"
            print(f"  {i+1:3} {seg['state']:>12} "
                  f"{time_range:>12} {seg['duration']:5.2f}s "
                  f"{seg['avg_vib_rms']:7.4f} {seg['avg_imu_quality']:5.0%} "
                  f"{seg['avg_confidence']:4.0%}")

        print("\n" + "=" * 60)

    def plot(self, save_path=None):
        """
        Visualize motion detection with filtering pipeline.

        Shows:
        1. Raw vs Low-pass filtered acceleration (body motion extraction)
        2. High-pass accel = Vibration energy (wheel bumps)
        3. Gyroscope Z (raw vs filtered) with turn threshold
        4. Motion state timeline with IMU quality coloring
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
        except ImportError:
            print("matplotlib not available. Install with: pip install matplotlib")
            return

        if not self.results:
            self.analyze()

        # Extract data
        times = [s['time_s'] for s in self.data]
        ax_raw = [s['ax'] for s in self.data]
        gz_raw = [s['gz'] for s in self.data]

        # Filtered data
        ax_lp = self.filtered_data['ax_lp']
        gz_lp = self.filtered_data['gz_lp']
        vib_rms = self.filtered_data['vib_rms']
        vib_deriv = self.filtered_data['vib_deriv']
        dir_latch = self.filtered_data['direction_latch']

        # Create figure with 5 subplots
        fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
        fig.suptitle('IMU Motion Detection with Direction Metric', fontsize=14, fontweight='bold')

        # Color map for states
        state_colors = {
            'STOPPED': '#95a5a6',    # Gray
            'FORWARD': '#2ecc71',    # Green
            'BACKWARD': '#e74c3c',   # Red
            'TURN_LEFT': '#3498db',  # Blue
            'TURN_RIGHT': '#9b59b6', # Purple
            'MOVING': '#f1c40f',     # Yellow
        }

        # Plot 1: Raw vs Low-pass filtered acceleration
        ax1 = axes[0]
        ax1.plot(times, ax_raw, 'b-', linewidth=0.3, alpha=0.5, label='Raw ax')
        ax1.plot(times, ax_lp, 'r-', linewidth=1.5, label=f'LP filtered ({self.accel_lp_cutoff}Hz)')
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax1.set_ylabel('Accel X (g)')
        ax1.set_title('Acceleration: Raw (blue) vs Low-pass Filtered (red) = Body Motion')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)

        # Add colored backgrounds for motion states
        for seg in self.results['segments']:
            color = state_colors.get(seg['state'], '#cccccc')
            ax1.axvspan(seg['start_time'], seg['end_time'], alpha=0.2, color=color)

        # Plot 2: Vibration RMS (high-frequency energy)
        ax2 = axes[1]
        vib_rms_smooth = self.filtered_data['vib_rms_smooth']
        ax2.fill_between(times, vib_rms, alpha=0.3, color='orange', label='Vibration RMS (raw)')
        ax2.plot(times, vib_rms_smooth, 'darkorange', linewidth=2, label='Vib RMS smoothed (2Hz LP)')
        ax2.axhline(y=self.motion_threshold, color='r', linestyle='--',
                   linewidth=2, label=f'Motion threshold ({self.motion_threshold})')
        ax2.set_ylabel('Vib RMS (g)')
        ax2.set_title('Vibration Energy: Raw (fill) vs Smoothed (line) for Motion Detection')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

        for seg in self.results['segments']:
            color = state_colors.get(seg['state'], '#cccccc')
            ax2.axvspan(seg['start_time'], seg['end_time'], alpha=0.2, color=color)

        # Plot 3: Direction latch and vibration derivative
        ax3 = axes[2]
        # Show vib derivative (triggers latch capture)
        ax3.plot(times, vib_deriv, 'orange', linewidth=0.8, alpha=0.7, label='Vib derivative (motion start detector)')
        ax3.axhline(y=0.01, color='orange', linestyle='--', linewidth=1, alpha=0.5)
        # Show latched direction
        ax3.fill_between(times, dir_latch, alpha=0.5, color='green',
                        where=[d > 0 for d in dir_latch], label='Latched FWD')
        ax3.fill_between(times, dir_latch, alpha=0.5, color='red',
                        where=[d < 0 for d in dir_latch], label='Latched BACK')
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax3.set_ylabel('Direction Latch')
        ax3.set_ylim(-1.5, 1.5)
        ax3.set_title('Direction Latch: Captured at Motion START (when vib rising), Held During Motion')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)

        for seg in self.results['segments']:
            color = state_colors.get(seg['state'], '#cccccc')
            ax3.axvspan(seg['start_time'], seg['end_time'], alpha=0.15, color=color)

        # Plot 4: Gyroscope Z (raw vs filtered)
        ax4 = axes[3]
        ax4.plot(times, gz_raw, 'purple', linewidth=0.3, alpha=0.5, label='Raw gz')
        ax4.plot(times, gz_lp, 'darkviolet', linewidth=1.5, label=f'LP filtered ({self.gyro_lp_cutoff}Hz)')
        ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax4.axhline(y=self.turn_threshold, color='r', linestyle='--',
                   linewidth=1, label=f'Turn threshold (±{self.turn_threshold}°/s)')
        ax4.axhline(y=-self.turn_threshold, color='r', linestyle='--', linewidth=1)
        ax4.set_ylabel('Gyro Z (°/s)')
        ax4.set_title('Gyroscope Z: Turn Detection (more reliable than accel)')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)

        for seg in self.results['segments']:
            color = state_colors.get(seg['state'], '#cccccc')
            ax4.axvspan(seg['start_time'], seg['end_time'], alpha=0.2, color=color)

        # Plot 5: State timeline
        ax5 = axes[4]
        state_order = ['STOPPED', 'MOVING', 'FORWARD', 'BACKWARD', 'TURN_LEFT', 'TURN_RIGHT']
        state_y = {s: i for i, s in enumerate(state_order)}

        for seg in self.results['segments']:
            y = state_y.get(seg['state'], 0)
            color = state_colors.get(seg['state'], '#cccccc')
            ax5.barh(y, seg['duration'], left=seg['start_time'],
                    height=0.8, color=color, alpha=0.8,
                    edgecolor='black', linewidth=0.5)

        ax5.set_yticks(range(len(state_order)))
        ax5.set_yticklabels(state_order)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Motion State')
        ax5.set_title('Detected Motion Timeline (Direction from Vibration-Weighted Accel)')
        ax5.grid(True, alpha=0.3, axis='x')

        # Legend
        patches = [mpatches.Patch(color=color, label=state, alpha=0.8)
                  for state, color in state_colors.items()]
        ax5.legend(handles=patches, loc='upper right', ncol=3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to {save_path}")
        else:
            plt.show()


def main():
    """Command-line interface."""
    parser = argparse.ArgumentParser(
        description='IMU-only motion detection with filtering',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python motion_detector.py recording.csv              # Auto-detect threshold
  python motion_detector.py recording.csv --plot       # With visualization
  python motion_detector.py recording.csv -m 0.05      # Manual threshold
  python motion_detector.py recording.csv --baseline-seconds 1.0  # Longer baseline
  python motion_detector.py recording.csv --gyro-lp 15 --accel-lp 3 --plot
        """
    )

    parser.add_argument('csv_file', help='CSV file with IMU data')
    parser.add_argument('--turn-threshold', '-t', type=float, default=25.0,
                       help='Gyro Z threshold for turn detection in deg/s (default: 25)')
    parser.add_argument('--motion-threshold', '-m', type=float, default=None,
                       help='Vibration RMS threshold for motion detection (default: auto)')
    parser.add_argument('--auto-threshold', '-a', action='store_true', default=True,
                       help='Auto-detect motion threshold from stationary baseline (default: on)')
    parser.add_argument('--baseline-seconds', type=float, default=0.5,
                       help='Seconds at start to use for baseline (default: 0.5)')
    parser.add_argument('--window', '-w', type=int, default=200,
                       help='Analysis window in ms (default: 200)')
    parser.add_argument('--gyro-lp', type=float, default=20.0,
                       help='Low-pass filter cutoff for gyro in Hz (default: 20)')
    parser.add_argument('--accel-lp', type=float, default=5.0,
                       help='Low-pass filter cutoff for accel in Hz (default: 5)')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Show visualization')
    parser.add_argument('--save', '-s', type=str,
                       help='Save plot to file')

    args = parser.parse_args()

    # Use default threshold temporarily if auto-detect enabled
    initial_threshold = args.motion_threshold if args.motion_threshold else 0.02

    # Create detector
    detector = MotionDetector(
        turn_threshold=args.turn_threshold,
        motion_threshold=initial_threshold,
        window_ms=args.window,
        gyro_lp_cutoff=args.gyro_lp,
        accel_lp_cutoff=args.accel_lp
    )

    detector.load_csv(args.csv_file)

    # Auto-detect threshold if not manually specified
    if args.motion_threshold is None and args.auto_threshold:
        detector.auto_detect_threshold(baseline_seconds=args.baseline_seconds)

    detector.analyze()
    detector.print_report()

    if args.plot or args.save:
        detector.plot(save_path=args.save)


if __name__ == '__main__':
    main()
