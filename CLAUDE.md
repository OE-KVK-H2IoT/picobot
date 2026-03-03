# CLAUDE.md

This file provides guidance to Claude Code when working with this repository's source code.

## Project Overview

PicoBot is an educational robotics platform for BSc students at Obuda University. It uses a Raspberry Pi Pico 2 W with a BMI160 IMU sensor to teach embedded systems concepts through an 11-week hands-on course.

## Directory Structure

```
picobot/
├── lib/
│   ├── picobot/            # Hardware abstraction package
│   │   ├── __init__.py     # Package exports
│   │   ├── robot.py        # Main Robot class (all layers)
│   │   ├── imu.py          # BMI160 IMU driver (I2C)
│   │   ├── motors.py       # Differential drive motor control
│   │   ├── leds.py         # WS2812B NeoPixel LED control
│   │   ├── sensors.py      # Line sensors + ultrasonic
│   │   ├── buzzer.py       # Piezo buzzer control
│   │   ├── config.py       # Pin mapping + default parameters
│   │   ├── state_machine.py # State machine framework
│   │   ├── encoders.py     # Motor encoder reading
│   │   └── utils.py        # Timing, data logging, helpers
│   ├── bmi160.py           # Low-level BMI160 register driver
│   ├── mpu6050.py          # Alternative IMU driver
│   ├── vl53l1x.py          # ToF distance sensor driver
│   ├── oled.py             # OLED display driver
│   ├── odometry.py         # Odometry calculations
│   ├── motor_model.py      # Motor modeling
│   ├── motion_ai.py        # AI-based motion control
│   ├── speed_estimator.py  # Speed estimation
│   ├── datalogger.py       # Data logging utilities
│   ├── ota.py              # Over-the-air updates
│   ├── control_receiver.py # Remote control receiver
│   └── robot_state.py      # Robot state management
├── labs/                   # Progressive lab exercises (lab1–lab7+)
├── examples/               # Standalone example programs
├── tests/                  # Unit tests
├── calibration/            # Motor and sensor calibration
├── lidar/                  # LIDAR scanning
├── collector.py            # Data collection for host
└── host/                   # PC-side tools
    ├── analyzers/          # Data analysis tools
    ├── lidar/              # LIDAR visualization
    ├── collector.py        # Data collection receiver
    ├── viewer.py           # Real-time visualizer
    ├── motor_sysid.py      # Motor system identification
    └── requirements.txt    # Host dependencies
```

## Course Structure (11 Weeks)

| Week | Phase | Theory (2×45 min) | Lab (3×45 min) |
|------|-------|--------------------|-----------------|
| 1 | Foundation | What is Embedded? | Robot Unboxing |
| 2 | Foundation | Motors & Motion | Motor Control |
| 3 | Control | Sensors & Feedback | Line Following |
| 4 | Control | Measurement & Integration | IMU Turns |
| 5 | Control | Timing & Concurrency | Ultrasonic Sensor |
| 6 | Software | Experimental Method | Data Logging |
| 7 | Software | State Machines | State Machines |
| 8 | Software | Abstraction Layers | Hardware Abstraction |
| 9 | Software | Software Engineering | Software Architecture |
| 10 | Project | System Integration | Project Integration |
| 11 | Project | Course Synthesis | Demo Day |

## Deployment

### Deploy libraries to Pico via mpremote
```bash
mpremote connect /dev/ttyACM0 cp -r lib/picobot/ :lib/picobot/
mpremote connect /dev/ttyACM0 cp lib/bmi160.py :lib/bmi160.py
```

### Per-lab: Deploy lab main.py
```bash
mpremote connect /dev/ttyACM0 cp labs/lab3/main.py :main.py
```

### Host visualization
```bash
cd host
pip install -r requirements.txt
python viewer.py
```

## Hardware Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| Left motor A/B | 13/12 | DRV8833, PWM 1kHz |
| Right motor A/B | 10/11 | DRV8833, PWM 1kHz |
| Left encoder A/B | 16/17 | Quadrature hall |
| Right encoder A/B | 8/9 | Quadrature hall |
| Line sensors (L→R) | 2/3/4/5 | Analog (ADC) |
| Ultrasonic Trig/Echo | 0/1 | HC-SR04 |
| I2C SDA/SCL | 14/15 | 400kHz, for BMI160 |
| NeoPixel LEDs | 22 | 8× WS2812B, PIO |
| Buzzer | 15 | PWM |
| Microphone | 28 | ADC |

## Code Style

- Module docstrings explain concepts (what, why, how)
- Configuration constants in `config.py`
- Safety: always stop motors in `finally` blocks
- picobot uses layered API: high-level (demos) → control (robot.forward) → component (robot.motors) → hardware (Pin)

## Key Package: picobot

### robot.py — Main entry point
- `Robot`: Bundles all hardware. High-level methods (forward, backward, turn, stop, demo_*) plus access to component objects (robot.motors, robot.imu, robot.leds, robot.sensors, robot.buzzer)

### imu.py — BMI160 IMU driver
- `IMU`: I2C driver for 6-axis IMU (accelerometer + gyroscope)
  - Gyro calibration, heading tracking, angle-based turns

### motors.py — Differential drive
- `Motors`: PWM motor control (left/right speed, forward, backward, stop)

### sensors.py — Line + ultrasonic
- `LineSensors`: 4-channel analog line sensor array
- `Ultrasonic`: HC-SR04 distance measurement

### state_machine.py — State machine framework
- `StateMachine`: Event-driven state machine with named states and transitions

### utils.py — Helpers
- Timing utilities, data logging, non-blocking patterns
