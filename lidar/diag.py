# diag.py — VL53L1X diagnostic script
# Run with: mpremote connect /dev/ttyACM0 run pico/diag.py

import time
import struct
from machine import Pin, I2C

# I2C setup (match your wiring)
I2C_ID = 1
I2C_SDA = 14
I2C_SCL = 15

ADDR = 0x29

print("=" * 50)
print("VL53L1X Diagnostic")
print("=" * 50)

# --- Step 1: I2C bus ---
print("\n[1] I2C bus scan...")
i2c = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
devices = i2c.scan()
print("    Devices found:", [hex(d) for d in devices])
if ADDR not in devices:
    print("    ERROR: VL53L1X (0x29) not found on I2C bus!")
    raise SystemExit

# --- I2C helpers ---
def w8(reg, val):
    buf = struct.pack(">H", reg) + bytes([val])
    i2c.writeto(ADDR, buf)

def w_reg(reg, data):
    buf = struct.pack(">H", reg) + data
    i2c.writeto(ADDR, buf)

def r8(reg):
    i2c.writeto(ADDR, struct.pack(">H", reg), False)
    return i2c.readfrom(ADDR, 1)[0]

def r16(reg):
    i2c.writeto(ADDR, struct.pack(">H", reg), False)
    d = i2c.readfrom(ADDR, 2)
    return (d[0] << 8) | d[1]

# --- Step 2: Sensor ID ---
print("\n[2] Reading sensor ID...")
model_id = r16(0x010F)
print("    Model ID: 0x{:04X} (expected 0xEACC)".format(model_id))
if model_id != 0xEACC:
    print("    ERROR: Wrong model ID!")
    raise SystemExit
print("    OK")

# --- Step 3: Soft reset ---
print("\n[3] Soft reset...")
w8(0x0000, 0x00)
time.sleep_ms(2)
w8(0x0000, 0x01)
time.sleep_ms(2)
print("    Done")

# --- Step 4: Wait for boot ---
print("\n[4] Waiting for firmware boot...")
for attempt in range(100):
    val = r8(0x00E5)
    if val & 0x01:
        print("    Booted after {} polls (reg 0x00E5 = 0x{:02X})".format(attempt, val))
        break
    time.sleep_ms(10)
else:
    print("    ERROR: Boot timeout! Last reg 0x00E5 = 0x{:02X}".format(val))
    # Try alternate boot register
    val2 = r8(0x0010)
    print("    Alt boot reg 0x0010 = 0x{:02X}".format(val2))

# --- Step 5: Load default config ---
print("\n[5] Loading default configuration...")
# Exact config from ST VL53L1X ULD v3.5.5
config = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00,
])
w_reg(0x002D, config)
print("    Wrote {} bytes at 0x002D".format(len(config)))

# Verify a few key registers
phasecal = r8(0x004B)
vcsel_a = r8(0x0060)
vcsel_b = r8(0x0063)
valid_phase = r8(0x0069)
print("    PHASECAL(0x4B)=0x{:02X} VCSEL_A(0x60)=0x{:02X} VCSEL_B(0x63)=0x{:02X} VALID_PH(0x69)=0x{:02X}".format(
    phasecal, vcsel_a, vcsel_b, valid_phase))

# --- Step 6: Read interrupt polarity ---
print("\n[6] Interrupt polarity...")
gpio_mux = r8(0x0030)
raw_pol = (gpio_mux & 0x10) >> 4
int_pol = 0 if raw_pol else 1
print("    GPIO_HV_MUX(0x30) = 0x{:02X}, bit4={}, IntPol={}".format(gpio_mux, raw_pol, int_pol))

# --- Step 7: VHV calibration ranging ---
print("\n[7] VHV calibration (first ranging)...")
# Clear interrupt first
w8(0x0086, 0x01)
# Start ranging
w8(0x0087, 0x40)
print("    Ranging started, waiting for data ready...")

for attempt in range(200):
    tio = r8(0x0031)
    intr = tio & 0x01
    ready = (intr == int_pol)
    if ready:
        print("    Data ready after {} polls (TIO_HV=0x{:02X}, bit0={})".format(attempt, tio, intr))
        break
    time.sleep_ms(5)
else:
    tio = r8(0x0031)
    print("    TIMEOUT! Last TIO_HV(0x31)=0x{:02X} bit0={} IntPol={}".format(tio, tio & 1, int_pol))

# Read result regardless
dist_vhv = r16(0x0096)
status_vhv = r8(0x0089)
print("    VHV result: dist={} mm, raw_status=0x{:02X} (bits4:0 = {})".format(
    dist_vhv, status_vhv, status_vhv & 0x1F))

# Clear and stop
w8(0x0086, 0x01)
w8(0x0087, 0x00)

# VHV config
w8(0x0008, 0x09)
w8(0x000B, 0x00)
print("    VHV config done")

# --- Step 8: Set distance mode LONG ---
print("\n[8] Setting distance mode LONG...")
w8(0x004B, 0x0A)
w8(0x0060, 0x0F)
w8(0x0063, 0x0D)
w8(0x0069, 0xB8)
# WOI SD0/SD1 as 16-bit write
w_reg(0x0078, struct.pack(">H", 0x0F0D))
w_reg(0x007A, struct.pack(">H", 0x0E0E))
print("    Done")

# --- Step 9: Set timing budget 50ms LONG ---
print("\n[9] Setting timing budget 50ms (LONG)...")
w_reg(0x005E, struct.pack(">H", 0x00AD))
w_reg(0x0061, struct.pack(">H", 0x00C6))
# Verify
ta = r16(0x005E)
tb = r16(0x0061)
print("    Timeout A=0x{:04X} B=0x{:04X}".format(ta, tb))

# --- Step 10: Single-shot reading (full ROI) ---
print("\n[10] Single-shot measurement (full 16x16 ROI)...")
w8(0x0086, 0x01)  # clear interrupt
w8(0x0087, 0x40)  # start

for attempt in range(200):
    tio = r8(0x0031)
    if (tio & 0x01) == int_pol:
        print("    Ready after {} polls".format(attempt))
        break
    time.sleep_ms(5)
else:
    print("    TIMEOUT!")

dist = r16(0x0096)
raw_st = r8(0x0089) & 0x1F
print("    Distance: {} mm, raw_status: {}".format(dist, raw_st))

# Map through status table
status_rtn = [255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
              255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
              255, 255, 11, 12]
if raw_st < 24:
    final = status_rtn[raw_st]
else:
    final = 255
print("    Final status: {} (0=valid, 255=invalid)".format(final))

w8(0x0086, 0x01)
w8(0x0087, 0x00)

# --- Step 11: Test with 4x4 ROI ---
print("\n[11] Testing 4x4 ROI...")
# Set ROI size to 4x4: (4-1)<<4 | (4-1) = 0x33
w8(0x0080, 0x33)
roi_size = r8(0x0080)
print("    ROI size reg (0x80) = 0x{:02X} (expect 0x33)".format(roi_size))

# Test center 199 (default optical center)
w8(0x007F, 199)
w8(0x0086, 0x01)
w8(0x0087, 0x40)

for attempt in range(200):
    if (r8(0x0031) & 0x01) == int_pol:
        print("    Center 199: ready after {} polls".format(attempt))
        break
    time.sleep_ms(5)
else:
    print("    Center 199: TIMEOUT!")

dist = r16(0x0096)
raw_st = r8(0x0089) & 0x1F
print("    Center 199: dist={} mm, raw_status={}".format(dist, raw_st))
w8(0x0086, 0x01)
w8(0x0087, 0x00)

# --- Step 12: Scan a few zone centers ---
print("\n[12] Scanning zone centers with 4x4 ROI...")
zone_spads = [146, 178, 210, 234, 150, 182, 214, 238]
zone_names = ["(0,0)", "(0,1)", "(0,2)", "(0,3)", "(1,0)", "(1,1)", "(1,2)", "(1,3)"]

for idx, spad in enumerate(zone_spads):
    w8(0x007F, spad)  # set ROI center
    w8(0x0086, 0x01)  # clear interrupt
    w8(0x0087, 0x40)  # start ranging

    ready = False
    for attempt in range(100):
        if (r8(0x0031) & 0x01) == int_pol:
            ready = True
            break
        time.sleep_ms(5)

    if ready:
        dist = r16(0x0096)
        raw_st = r8(0x0089) & 0x1F
        final = status_rtn[raw_st] if raw_st < 24 else 255
        print("    Zone {} spad={:3d}: {:5d} mm  raw_st={:2d} final={}  polls={}".format(
            zone_names[idx], spad, dist, raw_st, final, attempt))
    else:
        tio = r8(0x0031)
        print("    Zone {} spad={:3d}: TIMEOUT  TIO=0x{:02X}".format(
            zone_names[idx], spad, tio))

    w8(0x0086, 0x01)
    w8(0x0087, 0x00)

print("\n" + "=" * 50)
print("Diagnostic complete")
print("=" * 50)
