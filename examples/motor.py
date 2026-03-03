from picobot import Robot
import time

robot = Robot(encoders=True)

def test_encoder(enc, label, fwd_speed, bwd_speed):
    """Test one encoder: ticks, direction, speed."""
    print(f"--- {label} ---")

    # Tick test
    enc.reset()
    robot.set_motors(*fwd_speed)
    time.sleep(1)
    robot.stop()
    fwd = enc.ticks
    print(f"  Forward ticks:  {fwd}")

    time.sleep(0.3)

    # Direction test
    enc.reset()
    robot.set_motors(*bwd_speed)
    time.sleep(1)
    robot.stop()
    bwd = enc.ticks
    print(f"  Backward ticks: {bwd}")

    if fwd == 0 and bwd == 0:
        print(f"  FAIL: no ticks — check wiring/pins")
        return False

    if (fwd > 0 and bwd < 0):
        print(f"  OK: direction correct")
    elif (fwd < 0 and bwd > 0):
        print(f"  OK: direction inverted (swap A/B pins in config to fix)")
    else:
        print(f"  WARN: unexpected signs ({fwd}, {bwd})")

    time.sleep(0.3)

    # Speed test
    enc.reset()
    robot.set_motors(*fwd_speed)
    time.sleep(0.3)
    for _ in range(10):
        enc.update()
        time.sleep(0.05)
    robot.stop()
    print(f"  Speed: {enc.speed_tps:.1f} ticks/sec")
    print()
    return True

# Test left motor+encoder
ok_left = test_encoder(
    robot.encoders.left, "LEFT ENCODER",
    fwd_speed=(160, 0), bwd_speed=(-160, 0)
)

# Test right motor+encoder
if robot.encoders.right:
    ok_right = test_encoder(
        robot.encoders.right, "RIGHT ENCODER",
        fwd_speed=(0, 160), bwd_speed=(0, -160)
    )
else:
    print("--- RIGHT ENCODER ---")
    print("  SKIPPED: ENCODER_RIGHT_A/B not set in config.py\n")
    ok_right = True

robot.stop()
if ok_left and ok_right:
    print("=== ALL TESTS PASSED ===")
else:
    print("=== SOME TESTS FAILED ===")
