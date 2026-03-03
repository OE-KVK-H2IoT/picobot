"""
==============================================================================
AUDIO DEMO - Microphone ADC Streaming
==============================================================================

Demonstrates streaming audio from a microphone connected to ADC pin.

HARDWARE:
---------
  - Electret microphone module on GP27 (ADC1)
  - Microphone output should be biased to ~1.65V (mid-scale)

USAGE:
------
  1. Deploy to Pico:
     ampy --port /dev/ttyACM0 put lib/picobot.py picobot.py
     ampy --port /dev/ttyACM0 put examples/audio_demo.py main.py

  2. Run host collector:
     python host/collector.py

  3. Audio waveform will appear in the host UI

==============================================================================
"""

import network
import time
from machine import Pin

# Import DataLogger from picobot library
from picobot import DataLogger

# ==============================================================================
# CONFIGURATION
# ==============================================================================

# WiFi credentials - CHANGE THESE
WIFI_SSID = "NeoNexus"
WIFI_PASSWORD = "userC013$"

# Host IP (use broadcast for auto-discovery)
HOST_IP = "192.168.28.104"  # Change to your host IP, or use .255 for broadcast
HOST_PORT = 5005

# Audio settings
AUDIO_PIN = 27           # GP27 = ADC1
AUDIO_SAMPLE_RATE = 16000 # 16 kHz
AUDIO_BATCH_SIZE = 256    # ~16ms per batch
AUDIO_OVERSAMPLE = 2      # 2x oversample (32kHz ISR) - optimized for speed

# ==============================================================================
# WIFI CONNECTION
# ==============================================================================

def connect_wifi():
    """Connect to WiFi and return IP address."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print(f"Connecting to {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(".", end="")
        print()

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print(f"Connected! IP: {ip}")
        return ip
    else:
        raise RuntimeError("WiFi connection failed!")

# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("\n" + "=" * 50)
    print("AUDIO STREAMING DEMO")
    print(f"ADC Pin: GP{AUDIO_PIN}, Sample Rate: {AUDIO_SAMPLE_RATE} Hz")
    print("=" * 50 + "\n")

    # Connect to WiFi
    ip = connect_wifi()

    # Create DataLogger with audio enabled
    # Disable other sensors for this demo
    logger = DataLogger(
        robot=None,
        host_ip=HOST_IP,
        port=HOST_PORT,
        enable_motors=False,
        enable_ultrasonic=False,
        enable_opto=False,
        enable_imu=False,
        enable_audio=True,
        audio_pin=AUDIO_PIN,
        audio_sample_rate=AUDIO_SAMPLE_RATE,
        audio_batch_size=AUDIO_BATCH_SIZE,
        audio_oversample=AUDIO_OVERSAMPLE,
    )

    # Set log rate to match audio batch rate
    # At 16kHz with 256 samples, buffer fills every 16ms (62.5 Hz)
    # Log interval must be shorter than buffer fill time to not miss data
    batch_time_ms = (AUDIO_BATCH_SIZE / AUDIO_SAMPLE_RATE) * 1000
    logger._min_interval_us = int(batch_time_ms * 1000 * 0.8)  # 80% of batch time
    print(f"Log interval: {logger._min_interval_us/1000:.1f}ms (batch time: {batch_time_ms:.1f}ms)")

    print("\nStreaming audio to host...")
    print("Press Ctrl+C to stop\n")

    packet_count = 0
    try:
        while True:
            # Log sends packet if audio buffer is ready
            if logger.log():
                packet_count += 1
                if packet_count % 100 == 0:
                    print(f"Sent {packet_count} packets")

            # Small sleep to prevent busy-wait
            time.sleep_ms(5)

    except KeyboardInterrupt:
        print(f"\nStopped. Total packets: {packet_count}")

    finally:
        # Cleanup timer
        if logger._audio_timer:
            logger._audio_timer.deinit()

if __name__ == '__main__':
    main()
