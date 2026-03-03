# vl53l1x.py — MicroPython driver for VL53L1X Time-of-Flight sensor
# Works with generic breakout boards over I2C.
# Register values and lookup tables verified against official ST VL53L1X ULD
# API v3.5.5 (STSW-IMG009).

import time
import struct

_DEFAULT_ADDR = 0x29

# Default configuration blob — exact copy from ST VL53L1X ULD v3.5.5
# VL51L1X_DEFAULT_CONFIGURATION[] in VL53L1X_api.c
# 91 bytes, written starting at register 0x2D.
_DEFAULT_CONFIG = bytes([
    0x00,  # 0x2D
    0x00,  # 0x2E
    0x00,  # 0x2F
    0x01,  # 0x30
    0x02,  # 0x31
    0x00,  # 0x32
    0x02,  # 0x33
    0x08,  # 0x34
    0x00,  # 0x35
    0x08,  # 0x36
    0x10,  # 0x37
    0x01,  # 0x38
    0x01,  # 0x39
    0x00,  # 0x3A
    0x00,  # 0x3B
    0x00,  # 0x3C
    0x00,  # 0x3D
    0xFF,  # 0x3E
    0x00,  # 0x3F
    0x0F,  # 0x40
    0x00,  # 0x41
    0x00,  # 0x42
    0x00,  # 0x43
    0x00,  # 0x44
    0x00,  # 0x45
    0x20,  # 0x46 : interrupt config — 0x20 = new sample ready
    0x0B,  # 0x47
    0x00,  # 0x48
    0x00,  # 0x49
    0x02,  # 0x4A
    0x0A,  # 0x4B : PHASECAL_CONFIG__TIMEOUT_MACROP
    0x21,  # 0x4C
    0x00,  # 0x4D
    0x00,  # 0x4E
    0x05,  # 0x4F
    0x00,  # 0x50
    0x00,  # 0x51
    0x00,  # 0x52
    0x00,  # 0x53
    0xC8,  # 0x54
    0x00,  # 0x55
    0x00,  # 0x56
    0x38,  # 0x57
    0xFF,  # 0x58
    0x01,  # 0x59
    0x00,  # 0x5A
    0x08,  # 0x5B
    0x00,  # 0x5C
    0x00,  # 0x5D
    0x01,  # 0x5E : RANGE_CONFIG__TIMEOUT_MACROP_A_HI
    0xCC,  # 0x5F : RANGE_CONFIG__TIMEOUT_MACROP_A_LO
    0x0F,  # 0x60 : RANGE_CONFIG__VCSEL_PERIOD_A
    0x01,  # 0x61 : RANGE_CONFIG__TIMEOUT_MACROP_B_HI
    0xF1,  # 0x62 : RANGE_CONFIG__TIMEOUT_MACROP_B_LO
    0x0D,  # 0x63 : RANGE_CONFIG__VCSEL_PERIOD_B
    0x01,  # 0x64 : Sigma threshold MSB
    0x68,  # 0x65 : Sigma threshold LSB
    0x00,  # 0x66 : Min count rate MSB
    0x80,  # 0x67 : Min count rate LSB
    0x08,  # 0x68
    0xB8,  # 0x69 : RANGE_CONFIG__VALID_PHASE_HIGH
    0x00,  # 0x6A
    0x00,  # 0x6B
    0x00,  # 0x6C : Inter-measurement period MSB
    0x00,  # 0x6D
    0x0F,  # 0x6E
    0x89,  # 0x6F : Inter-measurement period LSB
    0x00,  # 0x70
    0x00,  # 0x71
    0x00,  # 0x72
    0x00,  # 0x73
    0x00,  # 0x74
    0x00,  # 0x75
    0x00,  # 0x76
    0x01,  # 0x77
    0x0F,  # 0x78 : SD_CONFIG__WOI_SD0
    0x0D,  # 0x79 : SD_CONFIG__WOI_SD1
    0x0E,  # 0x7A : SD_CONFIG__INITIAL_PHASE_SD0
    0x0E,  # 0x7B : SD_CONFIG__INITIAL_PHASE_SD1
    0x00,  # 0x7C
    0x00,  # 0x7D
    0x02,  # 0x7E
    0xC7,  # 0x7F : ROI center
    0xFF,  # 0x80 : ROI XY size
    0x9B,  # 0x81
    0x00,  # 0x82
    0x00,  # 0x83
    0x00,  # 0x84
    0x01,  # 0x85
    0x00,  # 0x86 : SYSTEM__INTERRUPT_CLEAR
    0x00,  # 0x87 : SYSTEM__MODE_START (0x00=stopped)
])

# Range status mapping from ST ULD (raw 5-bit -> final status code)
_STATUS_RTN = bytes([
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
    255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
    255, 255, 11, 12,
])

# --- Register addresses (16-bit) ---
_SOFT_RESET                      = 0x0000
_OSC_CALIBRATE_VAL               = 0x0006
_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008
_VHV_CONFIG_INIT                 = 0x000B
_GPIO_HV_MUX_CTRL               = 0x0030
_GPIO_TIO_HV_STATUS              = 0x0031
_PHASECAL_CONFIG_TIMEOUT_MACROP  = 0x004B
_RANGE_CONFIG_TIMEOUT_MACROP_A   = 0x005E  # 16-bit
_RANGE_CONFIG_VCSEL_PERIOD_A     = 0x0060
_RANGE_CONFIG_TIMEOUT_MACROP_B   = 0x0061  # 16-bit
_RANGE_CONFIG_VCSEL_PERIOD_B     = 0x0063
_RANGE_CONFIG_VALID_PHASE_HIGH   = 0x0069
_SYSTEM_INTERMEASUREMENT_PERIOD  = 0x006C  # 32-bit
_SD_CONFIG_WOI_SD0               = 0x0078  # 16-bit (SD0 + SD1)
_SD_CONFIG_INITIAL_PHASE_SD0     = 0x007A  # 16-bit (SD0 + SD1)
_ROI_CONFIG_USER_ROI_CENTRE_SPAD = 0x007F
_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080
_SYSTEM_INTERRUPT_CLEAR          = 0x0086
_SYSTEM_MODE_START               = 0x0087
_RESULT_RANGE_STATUS             = 0x0089
_RESULT_DISTANCE                 = 0x0096  # 16-bit (FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
_FIRMWARE_SYSTEM_STATUS          = 0x00E5
_IDENTIFICATION_MODEL_ID         = 0x010F

# --- Timing budget lookup tables (exact values from ST ULD v3.5.5) ---
_TB_SHORT = {
    15:  (0x001D, 0x0027),
    20:  (0x0051, 0x006E),
    33:  (0x00D6, 0x006E),
    50:  (0x01AE, 0x01E8),
    100: (0x02E1, 0x0388),
    200: (0x03E1, 0x0496),
    500: (0x0591, 0x05C1),
}

_TB_LONG = {
    20:  (0x001E, 0x0022),
    33:  (0x0060, 0x006E),
    50:  (0x00AD, 0x00C6),
    100: (0x01CC, 0x01EA),
    200: (0x02D9, 0x02F8),
    500: (0x048F, 0x04A4),
}


class VL53L1X:
    """MicroPython driver for VL53L1X ToF sensor."""

    SHORT = 1  # up to 1.3m, better ambient immunity
    LONG = 2   # up to 4m

    def __init__(self, i2c, addr=_DEFAULT_ADDR):
        self.i2c = i2c
        self.addr = addr
        self._mode = self.LONG
        self._timing_budget_ms = 50

    # --- low-level I2C (16-bit register addresses) ---

    def _write_reg(self, reg, data):
        buf = struct.pack(">H", reg) + data
        self.i2c.writeto(self.addr, buf)

    def _write8(self, reg, val):
        self._write_reg(reg, bytes([val]))

    def _write16(self, reg, val):
        self._write_reg(reg, struct.pack(">H", val))

    def _write32(self, reg, val):
        self._write_reg(reg, struct.pack(">I", val))

    def _read_reg(self, reg, n):
        self.i2c.writeto(self.addr, struct.pack(">H", reg), False)
        return self.i2c.readfrom(self.addr, n)

    def _read8(self, reg):
        return self._read_reg(reg, 1)[0]

    def _read16(self, reg):
        return struct.unpack(">H", self._read_reg(reg, 2))[0]

    # --- initialization (matches ST VL53L1X_SensorInit) ---

    def init(self):
        """Full sensor initialization. Call once after power-up."""
        # Check sensor identity
        model = self._read16(_IDENTIFICATION_MODEL_ID)
        if model != 0xEACC:
            raise RuntimeError(
                "VL53L1X not found (id=0x{:04X}, expected 0xEACC)".format(model))

        # Software reset (2ms sleeps match working diagnostic)
        self._write8(_SOFT_RESET, 0x00)
        time.sleep_ms(2)
        self._write8(_SOFT_RESET, 0x01)
        time.sleep_ms(2)

        # Wait for firmware boot (VL53L1X_BootState)
        deadline = time.ticks_add(time.ticks_ms(), 1000)
        while True:
            if self._read8(_FIRMWARE_SYSTEM_STATUS) & 0x01:
                break
            if time.ticks_diff(time.ticks_ms(), deadline) > 0:
                raise RuntimeError("VL53L1X boot timeout")
            time.sleep_ms(2)

        # Load default configuration (byte-by-byte like ST does)
        self._write_reg(0x002D, _DEFAULT_CONFIG)

        # Cache interrupt polarity BEFORE first ranging (needed by _data_ready)
        # ST: IntPol = !(reg 0x0030 bit4 >> 4)
        raw_pol = (self._read8(_GPIO_HV_MUX_CTRL) & 0x10) >> 4
        self._int_polarity = 0 if raw_pol else 1

        # First ranging (VHV calibration) — must clear interrupt before start
        self._clear_interrupt()
        self._start_ranging()
        if not self._wait_data_ready(1000):
            raise RuntimeError("VHV calibration timeout")
        self._clear_interrupt()
        self._stop_ranging()

        # Configure VHV for subsequent ranging (matches ST SensorInit exactly)
        self._write8(_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
        self._write8(_VHV_CONFIG_INIT, 0)

        # Apply defaults
        self.set_distance_mode(self.LONG)
        self.set_timing_budget_ms(50)

    # --- configuration (exact match to ST VL53L1X ULD v3.5.5) ---

    def set_distance_mode(self, mode):
        """Set SHORT (1) or LONG (2) distance mode."""
        budget = self._timing_budget_ms

        if mode == self.SHORT:
            self._write8(_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14)
            self._write8(_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07)
            self._write8(_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05)
            self._write8(_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38)
            self._write16(_SD_CONFIG_WOI_SD0, 0x0705)
            self._write16(_SD_CONFIG_INITIAL_PHASE_SD0, 0x0606)
        elif mode == self.LONG:
            self._write8(_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A)
            self._write8(_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F)
            self._write8(_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D)
            self._write8(_RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8)
            self._write16(_SD_CONFIG_WOI_SD0, 0x0F0D)
            self._write16(_SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E)
        else:
            raise ValueError("mode must be SHORT (1) or LONG (2)")

        self._mode = mode
        self.set_timing_budget_ms(budget)

    def set_timing_budget_ms(self, budget_ms):
        """Set timing budget. Valid values: 15(short only), 20, 33, 50, 100, 200, 500."""
        table = _TB_SHORT if self._mode == self.SHORT else _TB_LONG

        if budget_ms not in table:
            valid = sorted(table.keys())
            budget_ms = min(valid, key=lambda x: abs(x - budget_ms))

        timeout_a, timeout_b = table[budget_ms]
        self._write16(_RANGE_CONFIG_TIMEOUT_MACROP_A, timeout_a)
        self._write16(_RANGE_CONFIG_TIMEOUT_MACROP_B, timeout_b)
        self._timing_budget_ms = budget_ms

    def get_timing_budget_ms(self):
        return self._timing_budget_ms

    def set_inter_measurement_ms(self, period_ms):
        """Set inter-measurement period. 0 = single-shot."""
        # ST: ClockPLL = reg 0x0006 & 0x3FF; val = ClockPLL * ms * 1.075
        clock_pll = self._read16(_OSC_CALIBRATE_VAL) & 0x3FF
        if clock_pll == 0:
            clock_pll = 1
        val = int(clock_pll * period_ms * 1.075)
        self._write32(_SYSTEM_INTERMEASUREMENT_PERIOD, val)

    # --- ranging ---

    def _start_ranging(self):
        self._write8(_SYSTEM_MODE_START, 0x40)

    def _stop_ranging(self):
        self._write8(_SYSTEM_MODE_START, 0x00)

    def _clear_interrupt(self):
        self._write8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

    def _data_ready(self):
        """Check if measurement data is available (matches ST CheckForDataReady)."""
        intr = self._read8(_GPIO_TIO_HV_STATUS) & 0x01
        return intr == self._int_polarity

    def _wait_data_ready(self, timeout_ms=200):
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while True:
            if self._data_ready():
                return True
            if time.ticks_diff(time.ticks_ms(), deadline) > 0:
                return False
            time.sleep_ms(1)

    def read_mm(self, timeout_ms=200):
        """Single-shot measurement. Returns distance in mm, or -1 on timeout/error."""
        self._clear_interrupt()
        self._start_ranging()
        # Quick poll: ensure TIO flips to "not ready" before waiting for "ready".
        # Typically takes 0-1 iterations (~0.1ms) instead of fixed 4ms sleep.
        for _ in range(10):
            if not self._data_ready():
                break
        if not self._wait_data_ready(timeout_ms):
            self._stop_ranging()
            return -1

        dist = self._read16(_RESULT_DISTANCE)

        # Range status check (matches ST GetRangeStatus + status_rtn table)
        raw_status = self._read8(_RESULT_RANGE_STATUS) & 0x1F
        if raw_status < 24:
            final_status = _STATUS_RTN[raw_status]
        else:
            final_status = 255

        self._clear_interrupt()
        self._stop_ranging()

        # Status 0 = valid, 3 = valid min range, 7 = wrap target
        # Also accept 2 = sigma fail and 1 = signal fail (degraded but usable)
        if final_status == 255:
            return -1

        return dist

    # --- continuous ranging (efficient for multizone scanning) ---

    def start_continuous(self):
        """Start continuous ranging. Use with read_continuous_mm()."""
        self._clear_interrupt()
        self._start_ranging()

    def read_continuous_mm(self, next_roi_center=None, timeout_ms=200):
        """Read one measurement in continuous mode.

        Pass next_roi_center to set the ROI for the NEXT measurement.
        The center is applied BEFORE clearing the interrupt, so the sensor
        starts its next measurement with the correct ROI.
        """
        if not self._wait_data_ready(timeout_ms):
            if next_roi_center is not None:
                self.set_roi_center(next_roi_center)
            self._clear_interrupt()
            return -1
        dist = self._read16(_RESULT_DISTANCE)
        raw_status = self._read8(_RESULT_RANGE_STATUS) & 0x1F
        # Set next ROI center BEFORE clearing interrupt
        if next_roi_center is not None:
            self.set_roi_center(next_roi_center)
        self._clear_interrupt()  # sensor starts next measurement with new center
        if raw_status < 24:
            final_status = _STATUS_RTN[raw_status]
        else:
            final_status = 255
        if final_status == 255:
            return -1
        return dist

    def stop_continuous(self):
        """Stop continuous ranging."""
        self._stop_ranging()
        self._clear_interrupt()

    # --- ROI (Region of Interest) for multizone scanning ---

    def set_roi(self, width, height):
        """Set ROI size. Min 4, max 16 for each dimension.
        Matches ST SetROI: register encodes as (Y-1)<<4 | (X-1).
        """
        width = max(4, min(16, width))
        height = max(4, min(16, height))
        self._write8(_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                     ((height - 1) << 4) | (width - 1))

    def set_roi_center(self, spad):
        """Set ROI center SPAD number (0-255). Matches ST SetROICenter."""
        self._write8(_ROI_CONFIG_USER_ROI_CENTRE_SPAD, spad)

    @staticmethod
    def spad_number(row, col):
        """Convert (row, col) in 16x16 SPAD array to SPAD number.

        Row 0 = top, Col 0 = left (sensor front view).
        Used for set_roi_center().
        """
        if row < 8:
            return 128 + col * 8 + row
        else:
            return 127 - col * 8 - (row - 8)
