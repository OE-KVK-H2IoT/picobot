"""
==============================================================================
LAB 1 BONUS: DJ Mode - Live Beat-Reactive Robot
==============================================================================

Play music from your phone near the robot's microphone.
The robot detects beats and reacts with LEDs, buzzer, and dance moves.

CHANGE THE SETTINGS BELOW TO MAKE IT YOUR OWN!

==============================================================================
"""

import time
from picobot import Robot


# ==============================================================================
# SETTINGS - CHANGE THESE!
# ==============================================================================

SENSITIVITY = 40          # Beat detection: lower = more sensitive (10-80)
DANCE_EVERY = 3           # Dance move every N beats

# Your color palette
COLORS = [
    (255, 0, 0),      # Red
    (255, 127, 0),    # Orange
    (255, 255, 0),    # Yellow
    (0, 255, 0),      # Green
    (0, 255, 255),    # Cyan
    (0, 0, 255),      # Blue
    (255, 0, 255),    # Magenta
]

# Dance moves: (left_speed, right_speed, duration_ms)
DANCE_MOVES = [
    (-70, 70, 80),    # Wiggle left
    (70, -70, 80),    # Wiggle right
    (80, 80, 100),    # Forward burst
    (-80, -80, 100),  # Backward burst
]


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 50)
    print("LIVE DJ MODE")
    print("=" * 50)

    robot = Robot()

    color_idx = 0
    beat_count = 0
    move_idx = 0

    print()
    print("Play music near the robot!")
    print(f"Sensitivity: {SENSITIVITY}")
    print("Press Ctrl+C to stop")
    print()

    try:
        while True:
            if robot.mic.beat(threshold=SENSITIVITY):
                beat_count += 1

                # Flash a color
                robot.set_leds(COLORS[color_idx % len(COLORS)])
                color_idx += 1

                # Beep (different pitch each beat for variety)
                robot.beep(400 + (color_idx % 4) * 100, 25)

                # Dance move every N beats
                if beat_count % DANCE_EVERY == 0:
                    left, right, dur = DANCE_MOVES[move_idx % len(DANCE_MOVES)]
                    robot.set_motors(left, right)
                    time.sleep(dur / 1000)
                    robot.stop()
                    move_idx += 1

                print(f"  BEAT #{beat_count}")
            else:
                robot.leds_off()

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    robot.stop()
    robot.leds_off()

    print()
    print(f"Total beats detected: {beat_count}")
    robot.play_melody("success")
    print("=" * 50)


# ==============================================================================
# RUN
# ==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by DJ")
