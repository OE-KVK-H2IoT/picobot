"""
Utility classes - DataLogger, Timer, etc.

These help students with data collection and timing.
"""

import time


class DataLogger:
    """
    Simple CSV data logger for experiment data.

    Usage:
        logger = DataLogger("experiment.csv")
        logger.start("time", "error", "speed")

        for i in range(100):
            logger.log(i * 20, get_error(), get_speed())

        logger.stop()
        # Then download: mpremote cp :experiment.csv .
    """

    def __init__(self, filename="data.csv"):
        """
        Initialize logger.

        Args:
            filename: Output file name (on Pico filesystem)
        """
        self._filename = filename
        self._file = None
        self._fields = []
        self._start_time = 0

    def start(self, *fields):
        """
        Start logging with given field names.

        Args:
            *fields: Column names for CSV header
        """
        self._fields = fields
        self._file = open(self._filename, 'w')

        # Write header
        header = "timestamp_ms," + ",".join(fields)
        self._file.write(header + "\n")

        self._start_time = time.ticks_ms()
        print(f"Logging to {self._filename}: {fields}")

    def log(self, *values):
        """
        Log a row of values.

        Args:
            *values: Values matching the fields from start()
        """
        if self._file is None:
            return

        timestamp = time.ticks_diff(time.ticks_ms(), self._start_time)
        row = f"{timestamp}," + ",".join(str(v) for v in values)
        self._file.write(row + "\n")

    def stop(self):
        """Close the log file."""
        if self._file:
            self._file.close()
            self._file = None
            print(f"Saved to {self._filename}")
            print(f"Download: mpremote cp :{self._filename} .")


class Timer:
    """
    Non-blocking timer for periodic tasks.

    Usage:
        fast_timer = Timer(10)   # 10ms = 100Hz
        slow_timer = Timer(100)  # 100ms = 10Hz

        while True:
            if fast_timer.ready():
                do_fast_thing()

            if slow_timer.ready():
                do_slow_thing()
    """

    def __init__(self, period_ms):
        """
        Create a timer.

        Args:
            period_ms: Period in milliseconds
        """
        self._period = period_ms
        self._last = time.ticks_ms()

    def ready(self):
        """
        Check if timer period has elapsed.

        Returns:
            True if period elapsed (and resets timer)
            False if still waiting
        """
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last) >= self._period:
            self._last = now
            return True
        return False

    def reset(self):
        """Reset timer to now."""
        self._last = time.ticks_ms()

    @property
    def elapsed(self):
        """Milliseconds since last trigger."""
        return time.ticks_diff(time.ticks_ms(), self._last)


class Stopwatch:
    """
    Simple stopwatch for timing code.

    Usage:
        sw = Stopwatch()
        sw.start()
        do_something()
        print(f"Took {sw.elapsed_ms}ms")
    """

    def __init__(self):
        self._start = 0

    def start(self):
        """Start the stopwatch."""
        self._start = time.ticks_us()

    @property
    def elapsed_us(self):
        """Microseconds since start."""
        return time.ticks_diff(time.ticks_us(), self._start)

    @property
    def elapsed_ms(self):
        """Milliseconds since start."""
        return self.elapsed_us / 1000

    def lap(self):
        """Get elapsed time and restart."""
        elapsed = self.elapsed_us
        self._start = time.ticks_us()
        return elapsed


class LowPassFilter:
    """
    Simple low-pass filter for noisy sensor data.

    Usage:
        lpf = LowPassFilter(alpha=0.3)
        while True:
            raw = read_sensor()
            smooth = lpf.update(raw)
    """

    def __init__(self, alpha=0.3):
        """
        Create filter.

        Args:
            alpha: Smoothing factor (0-1)
                   Low (0.1) = very smooth, laggy
                   High (0.9) = responsive, less smoothing
        """
        self._alpha = alpha
        self._value = None

    def update(self, new_value):
        """
        Update filter with new value.

        Args:
            new_value: New sensor reading

        Returns:
            Filtered value
        """
        if self._value is None:
            self._value = new_value
        else:
            self._value = self._alpha * new_value + (1 - self._alpha) * self._value
        return self._value

    @property
    def value(self):
        """Current filtered value."""
        return self._value

    def reset(self, value=None):
        """Reset filter."""
        self._value = value
