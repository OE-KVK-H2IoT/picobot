#!/usr/bin/env python3
"""
==============================================================================
INTERACTIVE PARTITION TOOL
==============================================================================

Manually select and label regions of recorded data for analysis.
Useful when you have multiple forward/backward runs in one recording
and want to analyze them separately with statistics.

USAGE:
------
    python partition_tool.py recording.csv

CONTROLS:
---------
    Click       - Add partition boundary at cursor position
    1-5         - Label current region (1=FWD, 2=BWD, 3=STOP, 4=LEFT, 5=RIGHT)
    Backspace   - Remove last boundary
    Enter       - Finish and show analysis
    S           - Save partitions to JSON
    Q/Escape    - Quit

==============================================================================
"""

import csv
import json
import math
import argparse
import os
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, SpanSelector
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("matplotlib required: pip install matplotlib")


class PartitionTool:
    """Interactive tool for manually partitioning sensor data."""

    LABELS = {
        '1': 'FORWARD',
        '2': 'BACKWARD',
        '3': 'STOPPED',
        '4': 'TURN_LEFT',
        '5': 'TURN_RIGHT',
    }

    COLORS = {
        'FORWARD': '#2ecc71',
        'BACKWARD': '#e74c3c',
        'STOPPED': '#95a5a6',
        'TURN_LEFT': '#3498db',
        'TURN_RIGHT': '#9b59b6',
        'UNLABELED': '#f39c12',
    }

    def __init__(self):
        self.data = []
        self.times = []
        self.us_data = []
        self.magnitudes = []

        # Partition state
        self.boundaries = [0]  # Start boundary
        self.labels = []  # Label for each region
        self.current_region = 0

        # Analysis results
        self.partitions = []
        self.stats = {}

    def load_csv(self, filename):
        """Load sensor data from CSV."""
        self.data = []
        self.filename = filename

        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    sample = {
                        'time_s': float(row.get('time_s', 0)),
                        'ax': float(row.get('ax', 0)),
                        'ay': float(row.get('ay', 0)),
                        'az': float(row.get('az', 0)),
                        'us': float(row.get('us', 0)),
                        'bumps': int(float(row.get('bumps', 0))),
                    }
                    self.data.append(sample)
                except (ValueError, KeyError):
                    continue

        # Prepare visualization data
        self.times = [s['time_s'] for s in self.data]
        self.us_data = [s['us'] for s in self.data]
        self.magnitudes = [
            math.sqrt(s['ax']**2 + s['ay']**2 + s['az']**2) - 1.0
            for s in self.data
        ]

        # Initialize with full data as one unlabeled region
        self.boundaries = [0, len(self.data)]
        self.labels = ['UNLABELED']

        print(f"Loaded {len(self.data)} samples from {filename}")
        return len(self.data)

    def run_interactive(self):
        """Run interactive partition tool."""
        if not MATPLOTLIB_AVAILABLE:
            print("matplotlib required for interactive mode")
            return

        # Create figure
        self.fig, self.axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
        self.fig.suptitle('Interactive Partition Tool - Click to add boundaries, 1-5 to label',
                         fontsize=12, fontweight='bold')

        # Plot 1: Ultrasonic distance
        self.ax_us = self.axes[0]
        self.ax_us.plot(self.times, self.us_data, 'b-', linewidth=1)
        self.ax_us.set_ylabel('Distance (cm)')
        self.ax_us.set_title('Ultrasonic Distance')
        self.ax_us.grid(True, alpha=0.3)

        # Plot 2: Acceleration magnitude
        self.ax_mag = self.axes[1]
        self.ax_mag.plot(self.times, self.magnitudes, 'g-', linewidth=0.5)
        self.ax_mag.set_ylabel('Accel Mag (g)')
        self.ax_mag.set_title('Acceleration Magnitude')
        self.ax_mag.grid(True, alpha=0.3)

        # Plot 3: Partition visualization
        self.ax_part = self.axes[2]
        self.ax_part.set_ylabel('Partition')
        self.ax_part.set_xlabel('Time (s)')
        self.ax_part.set_title('Partitions (click to add boundary, 1-5 to label region)')
        self.ax_part.set_ylim(-0.5, 1.5)
        self.ax_part.set_yticks([])
        self.ax_part.grid(True, alpha=0.3, axis='x')

        # Draw initial partitions
        self._draw_partitions()

        # Status text
        self.status_text = self.fig.text(
            0.02, 0.02,
            'Click: add boundary | 1=FWD 2=BWD 3=STOP 4=LEFT 5=RIGHT | Backspace: undo | Enter: analyze | S: save | Q: quit',
            fontsize=9, family='monospace'
        )

        # Connect events
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.08)
        plt.show()

    def _draw_partitions(self):
        """Redraw partition visualization."""
        self.ax_part.clear()
        self.ax_part.set_ylabel('Partition')
        self.ax_part.set_xlabel('Time (s)')
        self.ax_part.set_ylim(-0.5, 1.5)
        self.ax_part.set_yticks([])
        self.ax_part.grid(True, alpha=0.3, axis='x')

        # Draw each partition
        for i in range(len(self.labels)):
            if i >= len(self.boundaries) - 1:
                break

            start_idx = self.boundaries[i]
            end_idx = self.boundaries[i + 1]

            if start_idx >= len(self.times) or end_idx > len(self.times):
                continue

            t_start = self.times[start_idx]
            t_end = self.times[min(end_idx - 1, len(self.times) - 1)]

            label = self.labels[i] if i < len(self.labels) else 'UNLABELED'
            color = self.COLORS.get(label, '#cccccc')

            # Draw colored region
            self.ax_part.axvspan(t_start, t_end, alpha=0.5, color=color)

            # Add label text
            t_mid = (t_start + t_end) / 2
            self.ax_part.text(t_mid, 0.5, f"{i+1}:{label[:3]}",
                            ha='center', va='center', fontsize=8,
                            fontweight='bold')

        # Draw boundary lines on all axes
        for ax in self.axes:
            for b in self.boundaries[1:-1]:  # Skip first and last
                if b < len(self.times):
                    ax.axvline(x=self.times[b], color='red', linestyle='--',
                              linewidth=1.5, alpha=0.7)

        # Highlight current region
        if self.current_region < len(self.labels):
            start_idx = self.boundaries[self.current_region]
            end_idx = self.boundaries[self.current_region + 1]
            if start_idx < len(self.times) and end_idx <= len(self.times):
                t_start = self.times[start_idx]
                t_end = self.times[min(end_idx - 1, len(self.times) - 1)]
                for ax in self.axes[:2]:
                    ax.axvspan(t_start, t_end, alpha=0.1, color='yellow')

        self.fig.canvas.draw_idle()

    def _on_click(self, event):
        """Handle mouse click - add partition boundary."""
        if event.inaxes not in self.axes:
            return

        # Find nearest time index
        click_time = event.xdata
        if click_time is None:
            return

        # Find index closest to click time
        idx = min(range(len(self.times)),
                 key=lambda i: abs(self.times[i] - click_time))

        # Don't add if too close to existing boundary
        for b in self.boundaries:
            if abs(b - idx) < 10:
                return

        # Insert boundary in sorted order
        insert_pos = 0
        for i, b in enumerate(self.boundaries):
            if idx > b:
                insert_pos = i + 1

        self.boundaries.insert(insert_pos, idx)

        # Add label for new region
        if insert_pos < len(self.labels):
            # Splitting existing region - copy its label
            self.labels.insert(insert_pos, self.labels[insert_pos - 1] if insert_pos > 0 else 'UNLABELED')
        else:
            self.labels.append('UNLABELED')

        self.current_region = insert_pos
        self._draw_partitions()
        self._update_status()

    def _on_key(self, event):
        """Handle key press."""
        key = event.key

        if key in self.LABELS:
            # Label current region
            if self.current_region < len(self.labels):
                self.labels[self.current_region] = self.LABELS[key]
                # Move to next region
                if self.current_region < len(self.labels) - 1:
                    self.current_region += 1
                self._draw_partitions()
                self._update_status()

        elif key == 'backspace':
            # Remove last boundary
            if len(self.boundaries) > 2:
                self.boundaries.pop(-2)  # Remove second-to-last (keep start and end)
                if self.labels:
                    self.labels.pop()
                self.current_region = max(0, len(self.labels) - 1)
                self._draw_partitions()
                self._update_status()

        elif key == 'left':
            # Previous region
            self.current_region = max(0, self.current_region - 1)
            self._draw_partitions()

        elif key == 'right':
            # Next region
            self.current_region = min(len(self.labels) - 1, self.current_region + 1)
            self._draw_partitions()

        elif key == 'enter':
            # Analyze and show results
            self._analyze_partitions()
            self._show_results()

        elif key == 's':
            # Save partitions
            self._save_partitions()

        elif key in ('q', 'escape'):
            plt.close(self.fig)

    def _update_status(self):
        """Update status text."""
        n_parts = len(self.labels)
        n_labeled = sum(1 for l in self.labels if l != 'UNLABELED')
        current_label = self.labels[self.current_region] if self.current_region < len(self.labels) else '-'

        self.status_text.set_text(
            f'Partitions: {n_parts} ({n_labeled} labeled) | Current: {self.current_region + 1} [{current_label}] | '
            f'1=FWD 2=BWD 3=STOP 4=LEFT 5=RIGHT | Enter: analyze'
        )
        self.fig.canvas.draw_idle()

    def _analyze_partitions(self):
        """Analyze each partition and compute statistics."""
        self.partitions = []

        for i in range(len(self.labels)):
            if i >= len(self.boundaries) - 1:
                break

            start_idx = self.boundaries[i]
            end_idx = self.boundaries[i + 1]
            label = self.labels[i]

            if end_idx <= start_idx:
                continue

            seg_data = self.data[start_idx:end_idx]
            seg_mags = self.magnitudes[start_idx:end_idx]

            # Basic metrics
            duration = seg_data[-1]['time_s'] - seg_data[0]['time_s']
            us_start = seg_data[0]['us']
            us_end = seg_data[-1]['us']
            us_delta = us_end - us_start
            us_distance = abs(us_delta)

            # Bump count
            bumps_start = seg_data[0].get('bumps', 0)
            bumps_end = seg_data[-1].get('bumps', 0)
            bumps = bumps_end - bumps_start

            # B/cm
            bumps_per_cm = bumps / us_distance if us_distance > 1 else 0

            # Speed
            speed = us_distance / duration if duration > 0.1 else 0

            # Bump rate (Hz)
            bump_rate = bumps / duration if duration > 0.1 else 0

            self.partitions.append({
                'index': i + 1,
                'label': label,
                'start_idx': start_idx,
                'end_idx': end_idx,
                'duration': duration,
                'us_start': us_start,
                'us_end': us_end,
                'us_distance': us_distance,
                'bumps': bumps,
                'bumps_per_cm': bumps_per_cm,
                'speed_cms': speed,
                'bump_rate_hz': bump_rate,
            })

        # Compute statistics by label
        self._compute_stats()

    def _compute_stats(self):
        """Compute statistics across partitions of same type."""
        self.stats = {}

        # Group by label
        by_label = {}
        for p in self.partitions:
            label = p['label']
            if label not in by_label:
                by_label[label] = []
            by_label[label].append(p)

        for label, parts in by_label.items():
            if not parts:
                continue

            # Collect values
            b_per_cm = [p['bumps_per_cm'] for p in parts if p['bumps_per_cm'] > 0]
            speeds = [p['speed_cms'] for p in parts if p['speed_cms'] > 0]
            bump_rates = [p['bump_rate_hz'] for p in parts if p['bump_rate_hz'] > 0]

            def calc_stats(values):
                if not values:
                    return {'mean': 0, 'std': 0, 'min': 0, 'max': 0, 'n': 0}
                n = len(values)
                mean = sum(values) / n
                variance = sum((x - mean) ** 2 for x in values) / n if n > 1 else 0
                std = math.sqrt(variance)
                return {
                    'mean': mean,
                    'std': std,
                    'min': min(values),
                    'max': max(values),
                    'n': n
                }

            self.stats[label] = {
                'count': len(parts),
                'total_distance': sum(p['us_distance'] for p in parts),
                'total_time': sum(p['duration'] for p in parts),
                'total_bumps': sum(p['bumps'] for p in parts),
                'bumps_per_cm': calc_stats(b_per_cm),
                'speed_cms': calc_stats(speeds),
                'bump_rate_hz': calc_stats(bump_rates),
            }

    def _show_results(self):
        """Display analysis results."""
        print("\n" + "=" * 70)
        print("PARTITION ANALYSIS RESULTS")
        print("=" * 70)

        print(f"\nFile: {self.filename}")
        print(f"Total partitions: {len(self.partitions)}")

        # Per-partition results
        print(f"\n{'#':>3} {'Label':>10} {'Time':>6} {'Dist':>6} {'Bumps':>6} {'B/cm':>6} {'Speed':>6} {'Rate':>6}")
        print("-" * 70)

        for p in self.partitions:
            print(f"{p['index']:3} {p['label']:>10} "
                  f"{p['duration']:5.2f}s {p['us_distance']:5.1f}cm "
                  f"{p['bumps']:5} {p['bumps_per_cm']:5.2f} "
                  f"{p['speed_cms']:5.1f} {p['bump_rate_hz']:5.1f}")

        # Statistics by label
        print(f"\n" + "=" * 70)
        print("STATISTICS BY DIRECTION")
        print("=" * 70)

        for label in ['FORWARD', 'BACKWARD', 'STOPPED', 'TURN_LEFT', 'TURN_RIGHT']:
            if label not in self.stats:
                continue

            s = self.stats[label]
            bpc = s['bumps_per_cm']
            spd = s['speed_cms']

            print(f"\n{label}:")
            print(f"  Samples:    {s['count']}")
            print(f"  Total dist: {s['total_distance']:.1f} cm")
            print(f"  Total time: {s['total_time']:.2f} s")
            print(f"  Total bumps: {s['total_bumps']}")

            if bpc['n'] > 0:
                print(f"\n  B/cm:  {bpc['mean']:.3f} ± {bpc['std']:.3f} (n={bpc['n']})")
                print(f"         range: [{bpc['min']:.3f}, {bpc['max']:.3f}]")

            if spd['n'] > 0:
                print(f"\n  Speed: {spd['mean']:.1f} ± {spd['std']:.1f} cm/s (n={spd['n']})")
                print(f"         range: [{spd['min']:.1f}, {spd['max']:.1f}]")

        # Calibration recommendation
        if 'FORWARD' in self.stats and self.stats['FORWARD']['bumps_per_cm']['n'] > 0:
            bpc = self.stats['FORWARD']['bumps_per_cm']
            print(f"\n" + "=" * 70)
            print(f"RECOMMENDED CALIBRATION (FORWARD)")
            print(f"  B/cm = {bpc['mean']:.3f}")
            print(f"  Uncertainty: ±{bpc['std']:.3f} ({bpc['std']/bpc['mean']*100:.1f}%)" if bpc['mean'] > 0 else "")
            print("=" * 70)

    def _save_partitions(self):
        """Save partitions to JSON file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.splitext(os.path.basename(self.filename))[0]
        out_file = f"data/partitions_{base}_{timestamp}.json"

        os.makedirs("data", exist_ok=True)

        output = {
            'source_file': self.filename,
            'timestamp': timestamp,
            'boundaries': self.boundaries,
            'labels': self.labels,
            'partitions': self.partitions,
            'stats': self.stats,
        }

        with open(out_file, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"\nSaved partitions to: {out_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Interactive tool for manually partitioning sensor data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Controls:
  Click         Add partition boundary
  1-5           Label region (1=FWD, 2=BWD, 3=STOP, 4=LEFT, 5=RIGHT)
  Left/Right    Navigate regions
  Backspace     Remove last boundary
  Enter         Analyze and show results
  S             Save partitions to JSON
  Q/Escape      Quit

Examples:
  python partition_tool.py recording.csv
  python partition_tool.py data/calibration_run.csv
        """
    )

    parser.add_argument('csv_file', help='CSV file with recorded data')

    args = parser.parse_args()

    if not MATPLOTLIB_AVAILABLE:
        print("Error: matplotlib required. Install with: pip install matplotlib")
        return

    tool = PartitionTool()
    tool.load_csv(args.csv_file)
    tool.run_interactive()


if __name__ == '__main__':
    main()
