"""
==============================================================================
LAB 3: Ultrasonic Distance Sensor and Obstacle Avoidance
==============================================================================

LEARNING OBJECTIVES:
--------------------
  1. Understand how ultrasonic distance sensors work
  2. Read and filter distance measurements
  3. Implement obstacle avoidance behavior

HARDWARE:
---------
  - HC-SR04 ultrasonic sensor
  - Trigger pin: GPIO 0
  - Echo pin: GPIO 1

THEORY - HOW ULTRASONIC SENSORS WORK:
-------------------------------------
  1. The sensor sends out an ultrasonic pulse (40kHz, inaudible to humans)
  2. The pulse bounces off obstacles and returns
  3. The sensor measures the time for the round trip
  4. Distance = (Time × Speed of Sound) / 2

  Speed of sound ≈ 343 m/s = 0.0343 cm/µs
  Distance (cm) = time (µs) / 58.2

  LIMITATIONS:
  - Minimum range: ~2 cm (too close = no reading)
  - Maximum range: ~400 cm
  - Beam angle: ~15° (cone shape, may miss thin objects)
  - Soft/angled surfaces may not reflect well

==============================================================================
"""

import time
from picobot import Ultrasonic, Motors, LEDStrip


# ==============================================================================
# CONFIGURATION
# ==============================================================================

# Distance thresholds (in cm)
DANGER_DISTANCE = 15      # Too close! Stop or reverse
WARNING_DISTANCE = 30     # Getting close, slow down
SAFE_DISTANCE = 50        # All clear

# Motor speeds
NORMAL_SPEED = 120
SLOW_SPEED = 80
TURN_SPEED = 100


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================

def distance_to_color(distance):
    """
    Convert distance to LED color for visual feedback.
    Returns (R, G, B) tuple.
    """
    if distance < 0:
        return (255, 255, 255)  # White = error/no reading
    elif distance < DANGER_DISTANCE:
        return (255, 0, 0)      # Red = danger
    elif distance < WARNING_DISTANCE:
        return (255, 165, 0)    # Orange = warning
    elif distance < SAFE_DISTANCE:
        return (255, 255, 0)    # Yellow = caution
    else:
        return (0, 255, 0)      # Green = safe


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    print("="*50)
    print("LAB 3: Ultrasonic Sensor")
    print("="*50)

    # Initialize hardware
    ultrasonic = Ultrasonic()
    motors = Motors()
    leds = LEDStrip(num_leds=4, brightness=30)

    print("Hardware initialized")
    print(f"Danger: < {DANGER_DISTANCE} cm")
    print(f"Warning: < {WARNING_DISTANCE} cm")
    print(f"Safe: >= {SAFE_DISTANCE} cm")

    # -------------------------------------------------------------------------
    # DEMO 1: Basic distance reading
    # -------------------------------------------------------------------------
    print("\n--- Demo 1: Distance Reading ---")
    print("Move your hand in front of the sensor...")
    print("(Press Ctrl+C to continue to next demo)\n")

    try:
        for _ in range(50):  # 50 readings
            distance = ultrasonic.distance_cm()

            # Visual feedback
            r, g, b = distance_to_color(distance)
            leds.fill(r, g, b)
            leds.show()

            if distance < 0:
                print(f"  Distance: ERROR (no echo)")
            else:
                bar = "█" * int(min(distance, 100) / 5)
                print(f"  Distance: {distance:5.1f} cm  {bar}")

            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    leds.clear()

    # -------------------------------------------------------------------------
    # DEMO 2: Filtered reading (reduces noise)
    # -------------------------------------------------------------------------
    print("\n--- Demo 2: Filtered Reading ---")
    print("Filtered uses median of 5 samples for accuracy\n")

    for _ in range(10):
        raw = ultrasonic.distance_cm()
        filtered = ultrasonic.distance_cm_filtered(samples=5)
        print(f"  Raw: {raw:5.1f} cm | Filtered: {filtered:5.1f} cm")
        time.sleep(0.5)

    # -------------------------------------------------------------------------
    # DEMO 3: Simple obstacle avoidance
    # -------------------------------------------------------------------------
    print("\n--- Demo 3: Obstacle Avoidance ---")
    print("Robot will avoid obstacles autonomously")
    print("Place robot on ground, press Enter to start...")
    input()

    print("Running! (Ctrl+C to stop)\n")
    leds.fill(0, 255, 0)
    leds.show()

    try:
        while True:
            distance = ultrasonic.distance_cm_filtered(samples=3)

            # Update LED color
            r, g, b = distance_to_color(distance)
            leds.fill(r, g, b)
            leds.show()

            if distance < 0:
                # No reading - stop for safety
                print("  No reading - stopping")
                motors.stop()

            elif distance < DANGER_DISTANCE:
                # Too close! Back up and turn
                print(f"  {distance:.0f} cm - DANGER! Reversing...")
                motors.backward(NORMAL_SPEED)
                time.sleep(0.4)
                motors.turn_right(TURN_SPEED)
                time.sleep(0.3)

            elif distance < WARNING_DISTANCE:
                # Getting close - turn away
                print(f"  {distance:.0f} cm - Warning, turning...")
                motors.turn_right(TURN_SPEED)
                time.sleep(0.2)

            else:
                # Safe - move forward
                print(f"  {distance:.0f} cm - Safe, forward")
                motors.forward(NORMAL_SPEED)

            time.sleep(0.1)

    except KeyboardInterrupt:
        motors.stop()
        leds.clear()
        print("\nStopped")

    # =========================================================================
    # YOUR EXERCISES:
    # =========================================================================
    #
    # EXERCISE 1: Distance-based speed control
    # ----------------------------------------
    # Instead of fixed speeds, make speed proportional to distance.
    # Closer = slower, farther = faster
    #
    # Hint:
    # speed = min(200, int(distance * 3))  # Scale distance to speed
    # motors.forward(speed)

    # EXERCISE 2: Wall following
    # --------------------------
    # Mount the sensor sideways and make the robot follow a wall
    # at a constant distance (~20cm)
    #
    # Hint: If too close, curve away. If too far, curve toward wall.

    # EXERCISE 3: Parking assist
    # --------------------------
    # Make the robot drive forward until it's exactly 10cm from an obstacle,
    # then stop. Use the LEDs to show distance feedback.

    # EXERCISE 4: Explore and map
    # ---------------------------
    # Make the robot explore, recording minimum distance found in each
    # direction. Report back the "clearest" direction.

    # =========================================================================

    print("\n--- Lab 3 Complete ---")
    motors.stop()
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
    finally:
        # Safety: always stop motors
        Motors().stop()
