"""
==============================================================================
DataLogger - Unified Data Logging for Pico Robot
==============================================================================

A simple, unified interface for logging data from the Pico robot that supports:
  1. CSV output to serial (copy-paste or redirect to file)
  2. UDP streaming to host PC for real-time visualization

This class is designed for educational use - keeping things simple while
teaching good practices for data collection and analysis.

USAGE MODES:
------------
Mode 1: CSV-only (USB connected, no WiFi)
    logger = DataLogger()
    logger.log(time_ms=100, led_state=1, error_ms=2)

Mode 2: UDP + CSV (WiFi connected)
    logger = DataLogger(host_ip="192.168.1.100")
    logger.log(time_ms=100, led_state=1, error_ms=2)

Mode 3: UDP-only (minimize serial output)
    logger = DataLogger(host_ip="192.168.1.100", csv_output=False)
    logger.log(time_ms=100, led_state=1, error_ms=2)

FLEXIBLE COLUMNS:
-----------------
The logger automatically handles any variables you pass to log():
    logger.log(pwm=100, speed=15.2, distance=42.5)
    logger.log(ax=0.1, ay=0.0, az=1.0, gx=0.5, gy=0.1, gz=0.0)

First call to log() establishes the columns and prints the CSV header.

CSV OUTPUT:
-----------
Output is formatted for easy copy-paste into spreadsheets:
    timestamp_ms,pwm,speed,distance
    0,100,15.2,42.5
    50,100,15.3,41.8
    ...

To save: redirect serial output to file, or copy-paste from terminal.

UDP STREAMING:
--------------
When host_ip is provided, data is also sent via UDP for real-time
visualization. The host PC can run the viewer.py tool to display charts.

Packet format: CSV line as UTF-8 text (simple, debuggable)
Default port: 5005

==============================================================================
"""

import time

# Try to import network modules (only available on Pico W)
try:
    import socket
    SOCKET_AVAILABLE = True
except ImportError:
    SOCKET_AVAILABLE = False


class DataLogger:
    """
    Unified data logger with CSV and UDP output.

    Example:
        logger = DataLogger()  # CSV only
        # or
        logger = DataLogger(host_ip="192.168.1.100")  # CSV + UDP

        # Log data (any keyword arguments become columns)
        for i in range(10):
            value = read_sensor()
            logger.log(reading=value, iteration=i)

        # Print statistics
        logger.summary()
    """

    DEFAULT_PORT = 5005

    def __init__(self, host_ip=None, port=None, csv_output=True,
                 include_timestamp=True, quiet=False):
        """
        Initialize the DataLogger.

        Args:
            host_ip: IP address of host PC for UDP streaming (None = CSV only)
            port: UDP port for streaming (default: 5005)
            csv_output: Print CSV to serial (default: True)
            include_timestamp: Auto-add timestamp column (default: True)
            quiet: Suppress status messages (default: False)
        """
        self.host_ip = host_ip
        self.port = port or self.DEFAULT_PORT
        self.csv_output = csv_output
        self.include_timestamp = include_timestamp
        self.quiet = quiet

        # State
        self.columns = None  # Set on first log() call
        self.start_time = time.ticks_ms()
        self.row_count = 0
        self.header_printed = False

        # UDP socket (created on demand)
        self.sock = None

        # Statistics tracking
        self.stats = {}  # column_name -> {min, max, sum, count}

        if not quiet:
            if host_ip:
                print(f"DataLogger: CSV + UDP -> {host_ip}:{self.port}")
            else:
                print("DataLogger: CSV output (copy from terminal)")

    def _ensure_socket(self):
        """Create UDP socket if needed."""
        if self.sock is None and self.host_ip and SOCKET_AVAILABLE:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception as e:
                if not self.quiet:
                    print(f"DataLogger: UDP socket failed: {e}")
                self.host_ip = None  # Disable UDP

    def _print_header(self):
        """Print CSV header line."""
        if self.csv_output and self.columns:
            header = ",".join(self.columns)
            print(header)
            self._send_udp(header)
        self.header_printed = True

    def _send_udp(self, line):
        """Send line via UDP if configured."""
        if self.host_ip and self.sock:
            try:
                self.sock.sendto(line.encode(), (self.host_ip, self.port))
            except Exception:
                pass  # Ignore send errors (host might not be listening)

    def _update_stats(self, data):
        """Update running statistics for numeric columns."""
        for key, value in data.items():
            if isinstance(value, (int, float)):
                if key not in self.stats:
                    self.stats[key] = {
                        'min': value, 'max': value,
                        'sum': value, 'count': 1
                    }
                else:
                    s = self.stats[key]
                    s['min'] = min(s['min'], value)
                    s['max'] = max(s['max'], value)
                    s['sum'] += value
                    s['count'] += 1

    def log(self, **kwargs):
        """
        Log a row of data.

        First call establishes column names from the keyword arguments.
        Subsequent calls must use the same columns.

        Args:
            **kwargs: Column name/value pairs to log

        Example:
            logger.log(pwm=100, speed=15.2, error=0.5)
        """
        # Auto-add timestamp if enabled
        if self.include_timestamp:
            timestamp = time.ticks_diff(time.ticks_ms(), self.start_time)
            kwargs = {'timestamp_ms': timestamp, **kwargs}

        # First call: establish columns
        if self.columns is None:
            self.columns = list(kwargs.keys())
            self._ensure_socket()
            self._print_header()

        # Format values
        values = []
        for col in self.columns:
            val = kwargs.get(col, '')
            if isinstance(val, float):
                values.append(f"{val:.3f}")
            else:
                values.append(str(val))

        # Output CSV line
        line = ",".join(values)
        if self.csv_output:
            print(line)
        self._send_udp(line)

        # Track statistics
        self._update_stats(kwargs)
        self.row_count += 1

    def log_list(self, column_names, values):
        """
        Log using separate column names and values lists.

        Useful when column names are dynamic or come from configuration.

        Args:
            column_names: List of column names
            values: List of values (same length as column_names)

        Example:
            logger.log_list(['pwm', 'speed'], [100, 15.2])
        """
        if len(column_names) != len(values):
            raise ValueError("column_names and values must have same length")
        kwargs = dict(zip(column_names, values))
        self.log(**kwargs)

    def header(self, *columns):
        """
        Pre-define column names before logging.

        Optional - if not called, columns are inferred from first log() call.

        Args:
            *columns: Column names in order

        Example:
            logger.header('pwm', 'speed', 'distance')
        """
        self.columns = list(columns)
        if self.include_timestamp:
            self.columns = ['timestamp_ms'] + self.columns
        self._ensure_socket()
        self._print_header()

    def summary(self):
        """
        Print summary statistics for all numeric columns.

        Call at end of data collection to see min/max/avg.
        """
        if not self.stats:
            print("No data logged yet")
            return

        print("\n--- DataLogger Summary ---")
        print(f"Rows logged: {self.row_count}")
        print(f"Duration: {time.ticks_diff(time.ticks_ms(), self.start_time)} ms")
        print()

        for col, s in self.stats.items():
            if col == 'timestamp_ms':
                continue
            avg = s['sum'] / s['count'] if s['count'] > 0 else 0
            print(f"{col}:")
            print(f"  min={s['min']:.3f}, max={s['max']:.3f}, avg={avg:.3f}")

    def close(self):
        """Clean up resources."""
        if self.sock:
            self.sock.close()
            self.sock = None


# ==============================================================================
# Convenience functions for quick logging
# ==============================================================================

def quick_log(data_dict, header=True):
    """
    One-shot logging without creating a DataLogger instance.

    Useful for simple scripts that just need to print one dataset.

    Args:
        data_dict: Dictionary of column -> list of values
        header: Print CSV header (default: True)

    Example:
        quick_log({
            'pwm': [100, 120, 140, 160],
            'speed': [12.5, 15.2, 18.1, 20.8]
        })
    """
    if not data_dict:
        return

    # Get column names and ensure all columns have same length
    columns = list(data_dict.keys())
    num_rows = len(data_dict[columns[0]])

    if header:
        print(",".join(columns))

    for i in range(num_rows):
        values = []
        for col in columns:
            val = data_dict[col][i]
            if isinstance(val, float):
                values.append(f"{val:.3f}")
            else:
                values.append(str(val))
        print(",".join(values))


# ==============================================================================
# Example usage (runs when file is executed directly)
# ==============================================================================

if __name__ == '__main__':
    print("DataLogger Demo")
    print("="*40)

    # Mode 1: Simple CSV logging
    print("\n1. CSV-only mode:")
    logger = DataLogger(quiet=True)

    for i in range(5):
        logger.log(count=i, squared=i*i, cubed=i*i*i)
        time.sleep_ms(100)

    logger.summary()

    # Mode 2: Using quick_log for simple datasets
    print("\n2. Quick log mode:")
    quick_log({
        'pwm': [60, 80, 100, 120],
        'speed_cms': [8.5, 12.3, 16.1, 19.8]
    })
