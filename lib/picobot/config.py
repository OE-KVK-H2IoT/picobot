"""
Robot configuration - All hardware pins and default parameters.

Students can look here to understand the hardware mapping.
Later (Lab 09) they'll create their own config.py.
"""


class PINS:
    """Hardware pin assignments."""

    # Motors (DRV8833 H-bridge, 2 PWM pins per motor)
    MOTOR_LEFT_A = 13     # Forward
    MOTOR_LEFT_B = 12     # Backward

    MOTOR_RIGHT_A = 10    # Forward
    MOTOR_RIGHT_B = 11    # Backward

    # Encoders (quadrature hall sensors)
    ENCODER_LEFT_A = 16
    ENCODER_LEFT_B = 17
    ENCODER_RIGHT_A = 8
    ENCODER_RIGHT_B = 9

    # Line sensors (directly configure)
    LINE_SENSORS = [2, 3, 4, 5]  # Left to right

    # Ultrasonic (directly configure)
    ULTRASONIC_TRIG = 0
    ULTRASONIC_ECHO = 1

    # IMU (I2C) (directly configure)
    I2C_SDA = 14
    I2C_SCL = 15
    IMU_I2C_BUS = 1

    # LEDs (directly configure)
    NEOPIXEL = 6
    NEOPIXEL_COUNT = 8

    # Buzzer (directly configure)
    BUZZER = 22

    # Microphone (ADC, shared with light sensor on GP27)
    MIC_PIN = 27


class CONTROL:
    """Default control parameters."""

    # Line following
    DEFAULT_SPEED = 80
    DEFAULT_KP = 30
    SENSOR_WEIGHTS = [-1.5, -0.5, 0.5, 1.5]

    # Turning
    TURN_SPEED = 80
    TURN_TOLERANCE_DEG = 5

    # Safety
    OBSTACLE_DISTANCE_CM = 15

    # Timing
    LOOP_PERIOD_MS = 20
    GYRO_CALIBRATION_SAMPLES = 100


class HARDWARE:
    """Hardware-specific constants."""

    # Motor PWM
    MOTOR_PWM_FREQ = 1000
    MOTOR_MAX_DUTY = 65535

    # Ultrasonic
    ULTRASONIC_TIMEOUT_US = 30000  # ~5m max range
    SPEED_OF_SOUND_CM_US = 0.0343

    # IMU
    IMU_I2C_FREQ = 400000

    # Encoders
    ENCODER_PPR = None            # Pulses per revolution (set after calibration)
    WHEEL_CIRCUMFERENCE_MM = None # Set after measurement
