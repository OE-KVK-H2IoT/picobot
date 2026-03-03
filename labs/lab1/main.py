"""
==============================================================================
LAB 1: RGB LEDs and PWM Basics
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Understand how WS2812B (NeoPixel) LEDs work
  2. Learn RGB color mixing
  3. Create LED patterns and animations

HARDWARE:
---------
  - 4 x WS2812B RGB LEDs on GPIO 6

THEORY:
-------
  WS2812B LEDs contain 3 individual LEDs (Red, Green, Blue) in one package.
  By mixing these colors at different intensities (0-255), you can create
  any color:
    - Red   = (255, 0, 0)
    - Green = (0, 255, 0)
    - Blue  = (0, 0, 255)
    - Yellow = Red + Green = (255, 255, 0)
    - Cyan   = Green + Blue = (0, 255, 255)
    - Magenta = Red + Blue = (255, 0, 255)
    - White  = (255, 255, 255)

==============================================================================
"""

import time
from picobot import LEDStrip


# ==============================================================================
# CONFIGURATION
# ==============================================================================

NUM_LEDS = 4          # Number of LEDs on the robot
BRIGHTNESS = 50       # Global brightness (0-255), keep low to save power


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 1: RGB LEDs")
    print("="*50)

    # Initialize LED strip
    leds = LEDStrip(num_leds=NUM_LEDS, brightness=BRIGHTNESS)
    print(f"Initialized {NUM_LEDS} LEDs at brightness {BRIGHTNESS}")

    # -------------------------------------------------------------------------
    # DEMO 1: Basic colors
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Basic Colors ---")

    colors = [
        ("Red",     255, 0, 0),
        ("Green",   0, 255, 0),
        ("Blue",    0, 0, 255),
        ("Yellow",  255, 255, 0),
        ("Cyan",    0, 255, 255),
        ("Magenta", 255, 0, 255),
        ("White",   255, 255, 255),
    ]

    for name, r, g, b in colors:
        print(f"  {name}")
        leds.fill(r, g, b)
        leds.show()
        time.sleep(0.5)

    leds.clear()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 2: Individual LED control
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Individual LEDs ---")

    for i in range(NUM_LEDS):
        print(f"  LED {i}")
        leds.clear()
        leds.set_pixel(i, 0, 255, 0)  # Green
        leds.show()
        time.sleep(0.3)

    leds.clear()
    time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 3: Running light
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: Running Light ---")

    for _ in range(3):  # 3 cycles
        for i in range(NUM_LEDS):
            leds.clear()
            leds.set_pixel(i, 255, 0, 0)  # Red
            leds.show()
            time.sleep(0.1)

    leds.clear()
    time.sleep(0.5)

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Color gradient
    # --------------------------
    # Set each LED to a different color to create a rainbow effect.
    # Hint: Use leds.set_pixel(index, r, g, b) for each LED
    #
    # Your code here:
    # leds.set_pixel(0, 255, 0, 0)    # Red
    # leds.set_pixel(1, 255, 255, 0)  # Yellow
    # leds.set_pixel(2, 0, 255, 0)    # Green
    # leds.set_pixel(3, 0, 0, 255)    # Blue
    # leds.show()

    # EXERCISE 2: Breathing effect
    # ----------------------------
    # Gradually increase and decrease brightness to create a "breathing" effect.
    # Hint: Use a for loop and leds.set_brightness()
    #
    # Your code here:
    # for brightness in range(0, 100, 5):
    #     leds.set_brightness(brightness)
    #     leds.fill(0, 0, 255)
    #     leds.show()
    #     time.sleep(0.05)
    # for brightness in range(100, 0, -5):
    #     leds.set_brightness(brightness)
    #     leds.fill(0, 0, 255)
    #     leds.show()
    #     time.sleep(0.05)

    # EXERCISE 3: Traffic light
    # -------------------------
    # Simulate a traffic light: Red (2s) -> Yellow (0.5s) -> Green (2s) -> repeat
    # Use LED index 0 for red, 1 for yellow, 2 for green
    #
    # Your code here:

    # EXERCISE 4: Police lights
    # -------------------------
    # Alternate between red and blue flashing (like police car lights)
    #
    # Your code here:

    # =========================================================================

    print("\n--- Lab 1 Complete ---")
    leds.clear()


# ==============================================================================
# RUN
# ==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
