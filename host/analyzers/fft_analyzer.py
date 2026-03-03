#!/usr/bin/env python3
"""
==============================================================================
FFT ANALYZER - Frequency Analysis of Sensor Data
==============================================================================

This analyzer uses Fast Fourier Transform (FFT) to reveal the frequency
content of sensor signals. This is useful for:
  - Finding wheel bump frequency (for speed estimation)
  - Detecting motor vibration frequencies
  - Identifying resonance issues
  - Filtering design (know what frequencies to keep/remove)

WHAT IS FFT?
------------
FFT converts a time-domain signal to frequency-domain:

  Time domain:    Shows how signal changes over TIME
                  Example: accelerometer reading every 2ms

  Frequency domain: Shows WHAT FREQUENCIES are present
                    Example: 50Hz vibration, 120Hz motor noise

Think of it like:
  - Time domain = sheet music (notes over time)
  - Frequency domain = list of instruments playing (what sounds)

HOW FFT WORKS (Simplified):
---------------------------
1. Take N samples of the signal (N should be power of 2: 256, 512, 1024...)
2. FFT computes correlation with sine waves at different frequencies
3. Output shows magnitude at each frequency bin

Key terms:
  - Sample rate (Fs): How many samples per second (e.g., 500 Hz)
  - Nyquist frequency: Maximum detectable = Fs/2 (e.g., 250 Hz)
  - Frequency resolution: Fs/N (e.g., 500/512 ≈ 1 Hz per bin)

INTERPRETING FFT OUTPUT:
------------------------
  - X-axis: Frequency (Hz)
  - Y-axis: Magnitude (how strong that frequency is)
  - Peak at 25Hz: Something vibrating 25 times per second
  - Multiple peaks: Multiple vibration sources

PRACTICAL EXAMPLE - Wheel Bumps:
--------------------------------
  Robot moving at 20 cm/s with wheel circumference 10cm:
    - Wheel rotation rate: 20cm/s ÷ 10cm = 2 rotations/s = 2 Hz
    - If wheel has 10 bumps per rotation: 2 × 10 = 20 Hz bump frequency

  So if you see a peak at 20 Hz in the accelerometer FFT,
  that's likely the wheel bumps!

  Speed estimation: bump_frequency / bumps_per_rotation = rotation_rate
                   rotation_rate × wheel_circumference = speed

USAGE:
------
Standalone:
    python fft_analyzer.py recording.csv --channel ax
    python fft_analyzer.py recording.csv --channel ay --plot

As module:
    from analyzers import FFTAnalyzer
    analyzer = FFTAnalyzer(sample_rate=500)
    analyzer.load_csv('recording.csv')
    results = analyzer.analyze('ax')
    print(f"Dominant frequency: {results['peak_frequency']:.1f} Hz")

==============================================================================
"""

import csv
import math
import argparse
import os


class FFTAnalyzer:
    """
    Analyzes frequency content of sensor data using FFT.

    Attributes:
        sample_rate (float): Samples per second (Hz)
        window_type (str): Window function ('hann', 'hamming', 'none')
    """

    def __init__(self, sample_rate=500, window_type='hann', window_ms=500, us_smooth=0.2):
        """
        Initialize FFT analyzer.

        Args:
            sample_rate: Samples per second. Must match your data!
                        Pico collector default is 500 Hz.

            window_type: Window function to reduce spectral leakage.
                        'hann': Good general purpose (recommended)
                        'hamming': Slightly different shape
                        'none': No windowing (causes leakage)

            window_ms: Window size in ms for direction detection (default 500)

            us_smooth: Low-pass filter alpha for ultrasonic smoothing (0.2)
        """
        self.sample_rate = sample_rate
        self.window_type = window_type
        self.window_ms = window_ms
        self.us_smooth = us_smooth
        self.data = {}
        self.results = {}
        self.segments = []

    def load_csv(self, filename):
        """
        Load sensor data from CSV file.

        Args:
            filename: Path to CSV file

        Returns:
            int: Number of samples loaded
        """
        self.data = {'time_s': [], 'ax': [], 'ay': [], 'az': [],
                     'gx': [], 'gy': [], 'gz': [], 'us': []}

        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    for key in self.data:
                        if key in row:
                            self.data[key].append(float(row[key]))
                except ValueError:
                    continue

        n = len(self.data.get('ax', []))
        print(f"Loaded {n} samples from {filename}")

        # Auto-detect sample rate from data timestamps (CRITICAL for FFT accuracy!)
        if len(self.data['time_s']) > 10:
            duration = self.data['time_s'][-1] - self.data['time_s'][0]
            if duration > 0:
                detected_rate = (n - 1) / duration

                # Use detected rate instead of default
                old_rate = self.sample_rate
                self.sample_rate = detected_rate

                print(f"Sample rate: {detected_rate:.1f} Hz (auto-detected from timestamps)")

                if abs(detected_rate - old_rate) > 50:
                    print(f"  WARNING: Detected rate differs from default ({old_rate} Hz)")
                    print(f"  FFT frequency resolution: {detected_rate/n:.2f} Hz per bin")
                    print(f"  Nyquist limit: {detected_rate/2:.1f} Hz")

        return n

    def load_data(self, data_dict):
        """Load data from dictionary."""
        self.data = data_dict

    def compute_resultant(self):
        """
        Compute resultant vector magnitudes from accelerometer axes.

        Creates two new channels:
          - 'xy': Horizontal plane resultant sqrt(ax² + ay²)
          - 'xyz': Full 3D resultant sqrt(ax² + ay² + az²) - 1 (minus gravity)

        These are useful for analyzing overall vibration regardless of direction.
        """
        import math

        if 'ax' not in self.data or 'ay' not in self.data:
            return

        n = len(self.data['ax'])

        # Horizontal plane resultant (recommended for wheel analysis)
        self.data['xy'] = []
        for i in range(n):
            ax = self.data['ax'][i] if i < len(self.data['ax']) else 0
            ay = self.data['ay'][i] if i < len(self.data['ay']) else 0
            self.data['xy'].append(math.sqrt(ax**2 + ay**2))

        # Full 3D resultant (minus gravity)
        self.data['xyz'] = []
        if 'az' in self.data:
            for i in range(n):
                ax = self.data['ax'][i] if i < len(self.data['ax']) else 0
                ay = self.data['ay'][i] if i < len(self.data['ay']) else 0
                az = self.data['az'][i] if i < len(self.data['az']) else 0
                # Subtract 1g (gravity) from total magnitude
                mag = math.sqrt(ax**2 + ay**2 + az**2) - 1.0
                self.data['xyz'].append(abs(mag))

        print(f"Computed resultant vectors: xy (horizontal), xyz (3D)")

    def _low_pass_filter(self, values, alpha=None):
        """Apply low-pass filter (exponential moving average) to smooth signal."""
        if alpha is None:
            alpha = self.us_smooth
        if not values or alpha >= 1.0:
            return values
        smoothed = [values[0]]
        for i in range(1, len(values)):
            smoothed.append(alpha * values[i] + (1 - alpha) * smoothed[-1])
        return smoothed

    def _detect_direction(self, us_start, us_end, threshold=2.0):
        """Detect direction from ultrasonic change."""
        delta = us_end - us_start
        if delta < -threshold:
            return 'FORWARD'
        elif delta > threshold:
            return 'BACKWARD'
        return 'STOPPED'

    def partition_by_direction(self):
        """
        Partition data by movement direction using ultrasonic sensor.

        Same algorithm as BumpDetector - segments by FORWARD/BACKWARD/STOPPED.
        Also checks gyro to invalidate turning segments.

        Returns:
            list: Segments with direction, indices, validity info
        """
        if 'us' not in self.data or len(self.data['us']) < 10:
            return []

        n = len(self.data['us'])

        # Estimate sample rate
        if 'time_s' in self.data and len(self.data['time_s']) > 1:
            duration = self.data['time_s'][-1] - self.data['time_s'][0]
            sample_rate = n / duration if duration > 0 else self.sample_rate
        else:
            sample_rate = self.sample_rate

        # Calculate window size from ms
        window_size = max(10, int(self.window_ms * sample_rate / 1000))

        # Smooth ultrasonic data
        us_smooth = self._low_pass_filter(self.data['us'])

        if n < window_size * 2:
            return [{
                'direction': self._detect_direction(us_smooth[0], us_smooth[-1]),
                'start': 0, 'end': n, 'is_valid': True
            }]

        segments = []
        i = 0
        TURN_THRESHOLD = 30.0  # deg/s

        while i < n - window_size:
            window_start = us_smooth[i]
            window_end = us_smooth[i + window_size - 1]
            direction = self._detect_direction(window_start, window_end)

            segment_start = i
            while i < n - window_size:
                next_start = us_smooth[i]
                next_end = us_smooth[min(i + window_size - 1, n - 1)]
                if self._detect_direction(next_start, next_end) != direction:
                    break
                i += window_size // 2

            segment_end = min(i + window_size, n)

            if segment_end - segment_start >= window_size:
                # Check gyro for turning
                gz_vals = []
                if 'gz' in self.data:
                    gz_vals = [abs(self.data['gz'][j]) for j in range(segment_start, segment_end)
                              if j < len(self.data['gz'])]
                avg_gz = sum(gz_vals) / len(gz_vals) if gz_vals else 0
                is_turning = avg_gz > TURN_THRESHOLD

                us_delta = us_smooth[segment_end - 1] - us_smooth[segment_start]
                is_valid = (
                    direction in ('FORWARD', 'BACKWARD') and
                    not is_turning and
                    abs(us_delta) > 2.0
                )

                segments.append({
                    'direction': direction,
                    'start': segment_start,
                    'end': segment_end,
                    'us_delta': us_delta,
                    'avg_gz': avg_gz,
                    'is_turning': is_turning,
                    'is_valid': is_valid,
                })

        self.segments = segments
        return segments

    def analyze_by_direction(self, channel='ax'):
        """
        Analyze FFT for each direction segment separately.

        This allows comparing frequency content when moving forward vs backward
        vs stopped. Turning segments are marked but still analyzed.

        Args:
            channel: Data channel to analyze

        Returns:
            dict: Results organized by direction with FFT for each segment
        """
        if not self.segments:
            self.partition_by_direction()

        results_by_dir = {'FORWARD': [], 'BACKWARD': [], 'STOPPED': []}

        for i, seg in enumerate(self.segments):
            start, end = seg['start'], seg['end']

            # Extract segment data
            if channel not in self.data:
                continue

            segment_signal = self.data[channel][start:end]
            if len(segment_signal) < 16:  # Need minimum samples for FFT
                continue

            # Analyze this segment
            n = len(segment_signal)
            mean_val = sum(segment_signal) / n
            centered = [x - mean_val for x in segment_signal]
            windowed = self._apply_window(centered)

            fft_result = self._fft_numpy(windowed)
            n_fft = len(fft_result)
            magnitudes = []
            for j in range(n_fft // 2):
                mag = abs(fft_result[j]) / n_fft * 2
                magnitudes.append(mag)

            freq_res = self.sample_rate / n_fft
            frequencies = [j * freq_res for j in range(len(magnitudes))]

            # Find peak
            peak_idx = 1
            for j in range(2, len(magnitudes)):
                if magnitudes[j] > magnitudes[peak_idx]:
                    peak_idx = j

            # Calculate speed from ultrasonic (cm/s)
            us_dist = abs(seg.get('us_delta', 0))
            duration = n / self.sample_rate
            speed_cms = us_dist / duration if duration > 0.1 else 0

            # Calculate B/cm from frequency and speed
            # B/cm = frequency (bumps/sec) / speed (cm/sec)
            peak_freq = frequencies[peak_idx] if magnitudes else 0
            bumps_per_cm = peak_freq / speed_cms if speed_cms > 0.5 else 0

            seg_result = {
                'segment_idx': i,
                'direction': seg['direction'],
                'is_valid': seg['is_valid'],
                'is_turning': seg.get('is_turning', False),
                'avg_gz': seg.get('avg_gz', 0),
                'n_samples': n,
                'duration': duration,
                'us_distance': us_dist,
                'speed_cms': speed_cms,
                'frequencies': frequencies,
                'magnitudes': magnitudes,
                'peak_frequency': peak_freq,
                'peak_magnitude': magnitudes[peak_idx] if magnitudes else 0,
                'bumps_per_cm': bumps_per_cm,  # Calculated from FFT!
            }

            results_by_dir[seg['direction']].append(seg_result)

        # Calculate average peak frequency and B/cm per direction (valid segments only)
        summary = {}
        for direction, segs in results_by_dir.items():
            valid_segs = [s for s in segs if s['is_valid'] and s['speed_cms'] > 0.5]
            if valid_segs:
                avg_peak = sum(s['peak_frequency'] for s in valid_segs) / len(valid_segs)
                avg_speed = sum(s['speed_cms'] for s in valid_segs) / len(valid_segs)
                avg_bcm = sum(s['bumps_per_cm'] for s in valid_segs) / len(valid_segs)
                summary[direction] = {
                    'count': len(segs),
                    'valid_count': len(valid_segs),
                    'avg_peak_frequency': avg_peak,
                    'avg_speed_cms': avg_speed,
                    'avg_bumps_per_cm': avg_bcm,
                    'segments': segs,
                }
            elif segs:
                summary[direction] = {
                    'count': len(segs),
                    'valid_count': 0,
                    'avg_peak_frequency': 0,
                    'avg_speed_cms': 0,
                    'avg_bumps_per_cm': 0,
                    'segments': segs,
                }

        self.results['by_direction'] = summary
        return summary

    def _apply_window(self, signal):
        """
        Apply window function to signal.

        Windowing reduces "spectral leakage" - false frequencies that appear
        because FFT assumes the signal repeats forever. The window tapers
        the signal to zero at the edges.

        Args:
            signal: List of sample values

        Returns:
            list: Windowed signal
        """
        n = len(signal)

        if self.window_type == 'none':
            return signal

        windowed = []
        for i in range(n):
            if self.window_type == 'hann':
                # Hann window: 0.5 * (1 - cos(2π * i / (n-1)))
                w = 0.5 * (1 - math.cos(2 * math.pi * i / (n - 1)))
            elif self.window_type == 'hamming':
                # Hamming window: 0.54 - 0.46 * cos(2π * i / (n-1))
                w = 0.54 - 0.46 * math.cos(2 * math.pi * i / (n - 1))
            else:
                w = 1.0

            windowed.append(signal[i] * w)

        return windowed

    def _fft(self, signal):
        """
        Compute FFT of signal.

        This is a simple (educational) implementation. For production,
        use numpy.fft.fft() which is much faster.

        The FFT uses the Cooley-Tukey algorithm:
        1. Recursively split signal into even and odd indices
        2. Compute FFT of each half
        3. Combine using "butterfly" operations

        Args:
            signal: List of sample values (length should be power of 2)

        Returns:
            list: Complex FFT coefficients
        """
        n = len(signal)

        # Base case: single sample
        if n == 1:
            return [complex(signal[0], 0)]

        # Pad to next power of 2 if needed
        if n & (n - 1) != 0:
            # Find next power of 2
            next_pow2 = 1
            while next_pow2 < n:
                next_pow2 *= 2
            # Pad with zeros
            signal = signal + [0] * (next_pow2 - n)
            n = next_pow2

        # Recursive case: divide and conquer
        even = self._fft(signal[0::2])  # Even indices
        odd = self._fft(signal[1::2])   # Odd indices

        # Combine halves using butterfly operations
        result = [0] * n
        for k in range(n // 2):
            # "Twiddle factor" - rotation in complex plane
            angle = -2 * math.pi * k / n
            twiddle = complex(math.cos(angle), math.sin(angle))

            # Butterfly: combine even and odd
            result[k] = even[k] + twiddle * odd[k]
            result[k + n // 2] = even[k] - twiddle * odd[k]

        return result

    def _fft_numpy(self, signal):
        """
        Compute FFT using numpy (much faster).

        Args:
            signal: List or array of samples

        Returns:
            numpy array of complex FFT coefficients
        """
        try:
            import numpy as np
            return np.fft.fft(signal)
        except ImportError:
            print("NumPy not available, using slow FFT implementation")
            return self._fft(list(signal))

    def analyze(self, channel='ax', use_numpy=True):
        """
        Perform FFT analysis on specified channel.

        Args:
            channel: Data channel to analyze ('ax', 'ay', 'az', 'gx', etc.)
            use_numpy: Use numpy for faster FFT (recommended)

        Returns:
            dict: Analysis results including:
                - frequencies: Array of frequency values (Hz)
                - magnitudes: Array of magnitude values
                - peak_frequency: Frequency of largest peak (Hz)
                - peak_magnitude: Magnitude of largest peak
                - harmonics: List of significant peaks
        """
        if channel not in self.data or not self.data[channel]:
            return {'error': f'Channel {channel} not found'}

        signal = self.data[channel]
        n = len(signal)

        # Remove DC offset (mean)
        mean_val = sum(signal) / n
        signal_centered = [x - mean_val for x in signal]

        # Apply window function
        windowed = self._apply_window(signal_centered)

        # Compute FFT
        if use_numpy:
            fft_result = self._fft_numpy(windowed)
        else:
            fft_result = self._fft(windowed)

        # Convert to magnitude spectrum
        # Only use first half (positive frequencies)
        n_fft = len(fft_result)
        magnitudes = []
        for i in range(n_fft // 2):
            if hasattr(fft_result[i], 'real'):
                mag = abs(fft_result[i]) / n_fft * 2  # Normalize
            else:
                # Complex from pure Python implementation
                mag = math.sqrt(fft_result[i].real**2 + fft_result[i].imag**2) / n_fft * 2
            magnitudes.append(mag)

        # Calculate frequency axis
        # Frequency resolution = sample_rate / n_fft
        freq_resolution = self.sample_rate / n_fft
        frequencies = [i * freq_resolution for i in range(len(magnitudes))]

        # Find peak frequency (ignore DC at index 0)
        if len(magnitudes) > 1:
            peak_idx = 1  # Start from index 1 to skip DC
            for i in range(2, len(magnitudes)):
                if magnitudes[i] > magnitudes[peak_idx]:
                    peak_idx = i
            peak_frequency = frequencies[peak_idx]
            peak_magnitude = magnitudes[peak_idx]
        else:
            peak_frequency = 0
            peak_magnitude = 0

        # Find significant peaks (harmonics)
        # A peak is significant if it's higher than neighbors and above threshold
        threshold = peak_magnitude * 0.1  # 10% of max
        harmonics = []

        for i in range(2, len(magnitudes) - 2):
            is_peak = (magnitudes[i] > magnitudes[i-1] and
                      magnitudes[i] > magnitudes[i+1] and
                      magnitudes[i] > magnitudes[i-2] and
                      magnitudes[i] > magnitudes[i+2])
            if is_peak and magnitudes[i] > threshold:
                harmonics.append({
                    'frequency': frequencies[i],
                    'magnitude': magnitudes[i],
                    'relative': magnitudes[i] / peak_magnitude * 100
                })

        # Sort harmonics by magnitude
        harmonics.sort(key=lambda x: x['magnitude'], reverse=True)

        self.results[channel] = {
            'channel': channel,
            'n_samples': n,
            'sample_rate': self.sample_rate,
            'frequencies': frequencies,
            'magnitudes': magnitudes,
            'peak_frequency': peak_frequency,
            'peak_magnitude': peak_magnitude,
            'harmonics': harmonics[:10],  # Top 10 peaks
            'freq_resolution': freq_resolution,
            'max_frequency': self.sample_rate / 2,  # Nyquist
        }

        return self.results[channel]

    def estimate_bump_frequency(self):
        """
        Estimate wheel bump frequency from accelerometer data.

        Analyzes all accelerometer axes and finds the most prominent
        frequency that's likely wheel bumps.

        Typical wheel bump frequencies: 10-100 Hz depending on speed

        Returns:
            dict: Bump frequency estimation
        """
        results = {}

        for channel in ['ax', 'ay', 'az']:
            if channel in self.data and self.data[channel]:
                analysis = self.analyze(channel)
                results[channel] = {
                    'peak_freq': analysis['peak_frequency'],
                    'peak_mag': analysis['peak_magnitude'],
                }

        # Find channel with strongest signal in typical bump range (10-100 Hz)
        best_channel = None
        best_score = 0

        for channel, res in results.items():
            freq = res['peak_freq']
            if 10 <= freq <= 100:  # Likely bump frequency range
                if res['peak_mag'] > best_score:
                    best_score = res['peak_mag']
                    best_channel = channel

        if best_channel:
            return {
                'bump_frequency': results[best_channel]['peak_freq'],
                'channel': best_channel,
                'confidence': 'high' if best_score > 0.1 else 'low',
                'all_channels': results,
            }
        else:
            return {
                'bump_frequency': None,
                'channel': None,
                'confidence': 'none',
                'all_channels': results,
            }

    def print_report(self, channel='ax'):
        """Print formatted FFT analysis report."""
        if channel not in self.results:
            self.analyze(channel)

        r = self.results[channel]

        print("\n" + "=" * 60)
        print(f"FFT ANALYSIS REPORT - Channel: {channel}")
        print("=" * 60)

        print(f"\nSignal Information:")
        print(f"  Samples:          {r['n_samples']}")
        print(f"  Sample rate:      {r['sample_rate']} Hz")
        print(f"  Duration:         {r['n_samples']/r['sample_rate']:.2f} s")

        print(f"\nFrequency Analysis:")
        print(f"  Resolution:       {r['freq_resolution']:.2f} Hz per bin")
        print(f"  Max frequency:    {r['max_frequency']:.1f} Hz (Nyquist limit)")

        print(f"\nDominant Frequency:")
        print(f"  Peak at:          {r['peak_frequency']:.1f} Hz")
        print(f"  Magnitude:        {r['peak_magnitude']:.4f}")

        if r['harmonics']:
            print(f"\nSignificant Peaks (top 5):")
            print(f"  {'Frequency':>10s}  {'Magnitude':>10s}  {'Relative':>8s}")
            print("  " + "-" * 32)
            for h in r['harmonics'][:5]:
                print(f"  {h['frequency']:10.1f}  {h['magnitude']:10.4f}  {h['relative']:7.1f}%")

        # Interpretation hints
        print(f"\nInterpretation:")
        freq = r['peak_frequency']
        if freq < 5:
            print("  Low frequency peak - could be slow swaying or drift")
        elif 10 <= freq <= 100:
            print("  Mid-range frequency - likely wheel bumps or vibration")
            print(f"  If bumps_per_rotation=10, rotation rate = {freq/10:.1f} Hz")
        elif freq > 100:
            print("  High frequency - motor vibration or electrical noise")

        print("\n" + "=" * 60)

    def print_direction_report(self, channel='ax'):
        """Print FFT analysis report organized by direction."""
        if 'by_direction' not in self.results:
            self.analyze_by_direction(channel)

        summary = self.results['by_direction']

        print("\n" + "=" * 60)
        print(f"FFT ANALYSIS BY DIRECTION - Channel: {channel}")
        print("=" * 60)
        print(f"\nSample Rate: {self.sample_rate:.1f} Hz")
        print(f"Nyquist Limit: {self.sample_rate/2:.1f} Hz (max detectable frequency)")
        print(f"\nB/cm = Peak Frequency (Hz) / Speed (cm/s)")

        for direction in ['FORWARD', 'BACKWARD', 'STOPPED']:
            if direction not in summary:
                continue
            stats = summary[direction]
            valid_info = f" ({stats['valid_count']} valid)" if stats['valid_count'] < stats['count'] else ""

            print(f"\n{direction}:{valid_info}")
            print(f"  Segments: {stats['count']}")
            if stats['valid_count'] > 0:
                print(f"  Avg Peak Frequency: {stats['avg_peak_frequency']:.1f} Hz")
                print(f"  Avg Speed:          {stats['avg_speed_cms']:.1f} cm/s")
                print(f"  Avg B/cm (from FFT): {stats['avg_bumps_per_cm']:.3f}")

            # Show individual segment details
            print(f"\n  {'#':>3} {'Valid':>5} {'Speed':>6} {'Peak Hz':>8} {'B/cm':>7}")
            print("  " + "-" * 36)
            for seg in stats['segments']:
                valid_mark = "✓" if seg['is_valid'] else "✗"
                turn_mark = "↻" if seg['is_turning'] else " "
                print(f"  {seg['segment_idx']+1:3} {valid_mark:>4}{turn_mark} "
                      f"{seg['speed_cms']:5.1f} "
                      f"{seg['peak_frequency']:8.1f} "
                      f"{seg['bumps_per_cm']:7.3f}")

        print("\n" + "=" * 60)

    def plot(self, channel='ax', save_path=None, show_pipeline=True):
        """
        Plot frequency spectrum with signal processing pipeline.

        Args:
            channel: Channel to plot
            save_path: If provided, save plot to file instead of showing
            show_pipeline: Show all processing steps (True) or simple view (False)
        """
        try:
            import matplotlib.pyplot as plt
            import numpy as np
        except ImportError:
            print("matplotlib not available. Install with: pip install matplotlib")
            return

        if channel not in self.results:
            self.analyze(channel)

        r = self.results[channel]

        # Prepare data
        signal = self.data[channel]
        n = len(signal)
        if 'time_s' in self.data and self.data['time_s']:
            time = self.data['time_s']
        else:
            time = [i / self.sample_rate for i in range(n)]

        # Compute intermediate steps for visualization
        mean_val = sum(signal) / n
        signal_centered = [x - mean_val for x in signal]
        signal_windowed = self._apply_window(signal_centered)

        if show_pipeline:
            # 4-panel view showing pipeline
            fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=False)
            fig.suptitle(f'FFT Signal Processing Pipeline - {channel}',
                        fontsize=14, fontweight='bold')

            # Panel 1: Original signal
            ax1 = axes[0]
            ax1.plot(time, signal, 'b-', linewidth=0.5, alpha=0.7)
            ax1.axhline(y=mean_val, color='r', linestyle='--',
                       label=f'Mean = {mean_val:.3f}', linewidth=1)
            ax1.set_ylabel('Value')
            ax1.set_title('Step 1: Original Signal')
            ax1.legend(loc='upper right')
            ax1.grid(True, alpha=0.3)

            # Panel 2: DC removed (centered)
            ax2 = axes[1]
            ax2.plot(time, signal_centered, 'g-', linewidth=0.5, alpha=0.7)
            ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            ax2.set_ylabel('Value')
            ax2.set_title('Step 2: DC Removed (Mean Subtracted)')
            ax2.grid(True, alpha=0.3)

            # Panel 3: Windowed signal
            ax3 = axes[2]
            ax3.plot(time, signal_windowed, 'm-', linewidth=0.5, alpha=0.7)
            # Show window shape
            window_shape = self._apply_window([1.0] * n)
            max_sig = max(abs(min(signal_centered)), abs(max(signal_centered)))
            window_scaled = [w * max_sig for w in window_shape]
            ax3.fill_between(time, [-w for w in window_scaled], window_scaled,
                            alpha=0.2, color='orange', label=f'{self.window_type} window')
            ax3.set_ylabel('Value')
            ax3.set_title(f'Step 3: Windowed ({self.window_type.title()} Window Applied)')
            ax3.legend(loc='upper right')
            ax3.grid(True, alpha=0.3)

            # Panel 4: Frequency spectrum
            ax4 = axes[3]
            ax4.plot(r['frequencies'], r['magnitudes'], 'b-', linewidth=1)
            ax4.axvline(x=r['peak_frequency'], color='r', linestyle='--',
                       label=f'Peak: {r["peak_frequency"]:.1f} Hz', linewidth=1.5)
            # Mark harmonics
            for i, h in enumerate(r['harmonics'][:3]):
                if h['frequency'] != r['peak_frequency']:
                    ax4.axvline(x=h['frequency'], color='orange', linestyle=':',
                               alpha=0.7, linewidth=1)
            ax4.set_xlabel('Frequency (Hz)')
            ax4.set_ylabel('Magnitude')
            ax4.set_title(f'Step 4: FFT Result (Peak at {r["peak_frequency"]:.1f} Hz)')
            ax4.set_xlim(0, min(200, r['max_frequency']))
            ax4.legend(loc='upper right')
            ax4.grid(True, alpha=0.3)

        else:
            # Simple 2-panel view
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

            ax1.plot(time, signal, 'b-', linewidth=0.5)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel(f'{channel} value')
            ax1.set_title(f'Time Domain - {channel}')
            ax1.grid(True, alpha=0.3)

            ax2.plot(r['frequencies'], r['magnitudes'], 'b-', linewidth=1)
            ax2.axvline(x=r['peak_frequency'], color='r', linestyle='--',
                       label=f'Peak: {r["peak_frequency"]:.1f} Hz')
            ax2.set_xlabel('Frequency (Hz)')
            ax2.set_ylabel('Magnitude')
            ax2.set_title(f'Frequency Domain (FFT) - {channel}')
            ax2.set_xlim(0, min(200, r['max_frequency']))
            ax2.grid(True, alpha=0.3)
            ax2.legend()

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to {save_path}")
        else:
            plt.show()

        return fig

    def plot_all_channels(self, save_path=None):
        """
        Plot FFT for all accelerometer channels side by side.

        Useful for comparing which axis shows strongest bump signal.
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available. Install with: pip install matplotlib")
            return

        channels = ['ax', 'ay', 'az']
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('FFT Analysis - All Accelerometer Channels', fontsize=14, fontweight='bold')

        for i, ch in enumerate(channels):
            if ch not in self.data or not self.data[ch]:
                continue

            if ch not in self.results:
                self.analyze(ch)

            r = self.results[ch]

            # Time domain
            ax_time = axes[i, 0]
            if 'time_s' in self.data:
                time = self.data['time_s']
            else:
                time = [j / self.sample_rate for j in range(len(self.data[ch]))]
            ax_time.plot(time, self.data[ch], linewidth=0.5)
            ax_time.set_ylabel(ch)
            ax_time.set_title(f'{ch} - Time Domain')
            ax_time.grid(True, alpha=0.3)
            if i == 2:
                ax_time.set_xlabel('Time (s)')

            # Frequency domain
            ax_freq = axes[i, 1]
            ax_freq.plot(r['frequencies'], r['magnitudes'], linewidth=1)
            ax_freq.axvline(x=r['peak_frequency'], color='r', linestyle='--',
                           label=f'{r["peak_frequency"]:.1f} Hz')
            ax_freq.set_ylabel('Magnitude')
            ax_freq.set_title(f'{ch} - FFT (Peak: {r["peak_frequency"]:.1f} Hz)')
            ax_freq.set_xlim(0, min(150, r['max_frequency']))
            ax_freq.legend(loc='upper right', fontsize=8)
            ax_freq.grid(True, alpha=0.3)
            if i == 2:
                ax_freq.set_xlabel('Frequency (Hz)')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to {save_path}")
        else:
            plt.show()

        return fig


def main():
    """Command-line interface for FFT analysis."""
    parser = argparse.ArgumentParser(
        description='FFT frequency analysis of sensor data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python fft_analyzer.py recording.csv
  python fft_analyzer.py recording.csv --channel ay
  python fft_analyzer.py recording.csv --channel ax --plot
  python fft_analyzer.py recording.csv --rate 500 --bump
        """
    )

    parser.add_argument('csv_file', help='CSV file with recorded data')
    parser.add_argument('--channel', '-c', default='ax',
                       choices=['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'xy', 'xyz'],
                       help='Channel to analyze: ax,ay,az,gx,gy,gz or xy/xyz for resultant (default: ax)')
    parser.add_argument('--rate', '-r', type=float, default=500,
                       help='Sample rate in Hz (default: 500, auto-detected from data)')
    parser.add_argument('--window', '-w', default='hann',
                       choices=['hann', 'hamming', 'none'],
                       help='Window function (default: hann)')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Show frequency plot')
    parser.add_argument('--save', '-s', type=str,
                       help='Save plot to file')
    parser.add_argument('--bump', '-b', action='store_true',
                       help='Estimate bump frequency from all axes')
    parser.add_argument('--all', '-a', action='store_true',
                       help='Plot all accelerometer channels (ax, ay, az)')
    parser.add_argument('--simple', action='store_true',
                       help='Simple 2-panel plot instead of full pipeline')
    parser.add_argument('--direction', '-d', action='store_true',
                       help='Analyze by direction (FWD/BACK/STOP), invalidate turns')
    parser.add_argument('--window-ms', type=int, default=500,
                       help='Window size in ms for direction detection (default: 500)')

    args = parser.parse_args()

    # Create analyzer
    analyzer = FFTAnalyzer(sample_rate=args.rate, window_type=args.window,
                          window_ms=args.window_ms)
    analyzer.load_csv(args.csv_file)

    # Compute resultant vectors if needed
    if args.channel in ('xy', 'xyz'):
        analyzer.compute_resultant()

    if args.direction:
        # Direction-based analysis
        analyzer.partition_by_direction()
        analyzer.analyze_by_direction(args.channel)
        analyzer.print_direction_report(args.channel)
    elif args.bump:
        # Bump frequency estimation
        result = analyzer.estimate_bump_frequency()
        print("\n" + "=" * 50)
        print("BUMP FREQUENCY ESTIMATION")
        print("=" * 50)
        if result['bump_frequency']:
            print(f"\nEstimated bump frequency: {result['bump_frequency']:.1f} Hz")
            print(f"Detected on channel: {result['channel']}")
            print(f"Confidence: {result['confidence']}")
        else:
            print("\nNo clear bump frequency detected")
        print("\nPer-channel peaks:")
        for ch, res in result['all_channels'].items():
            print(f"  {ch}: {res['peak_freq']:.1f} Hz (mag: {res['peak_mag']:.4f})")
    else:
        # Single channel analysis
        analyzer.analyze(args.channel)
        analyzer.print_report(args.channel)

    if args.all:
        analyzer.plot_all_channels(save_path=args.save)
    elif args.plot or args.save:
        analyzer.plot(args.channel, save_path=args.save, show_pipeline=not args.simple)


if __name__ == '__main__':
    main()
