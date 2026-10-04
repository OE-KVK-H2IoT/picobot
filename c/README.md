# picobot-c — PicoBot hardware helpers for the C / Pico SDK firmware

The C translation of `lib/picobot/config.py` (the MicroPython pin map), for the
**Pico SDK + FreeRTOS** labs. Board: **RP2350 (Pico 2 W)**.

Status: **motors, buzzer, I²C, failsafe implemented**; NeoPixel (PIO) and the sensor drivers
(BMI160 IMU, VL53L1X ToF) are next. Verify the map against your unit before first use.

## Wiring (from `config.py`)

| Function | Pins | Peripheral |
|---|---|---|
| Left motor | GP13 fwd / GP12 back | PWM 1 kHz, H-bridge |
| Right motor | GP10 fwd / GP11 back | PWM 1 kHz, H-bridge |
| Encoders L/R | GP20·21 / GP18·19 | GPIO IRQ (quadrature) |
| IMU BMI160 | I²C1 SDA GP14 / SCL GP15 | I²C 400 kHz |
| NeoPixel ×8 | GP6 | PIO (not implemented here yet) |
| Buzzer | GP22 | PWM |

## Use it

Add `picobot.c` to your project and link `pico_stdlib hardware_pwm hardware_i2c`:

```cmake
add_executable(my_robot main.c picobot.c)
target_include_directories(my_robot PRIVATE .)
target_link_libraries(my_robot pico_stdlib hardware_pwm hardware_i2c)
```

```c
#include "picobot.h"

picobot_motors_init();
picobot_buzzer_init();
picobot_i2c_init();

picobot_motors_set(80, 80);          /* forward, ~31% duty */
picobot_buzzer_tone(523);            /* C5 */
```

## Safety — the failsafe is not optional

On a moving platform, a lost command must stop the motors. Arm a timeout, feed it on every
valid command, poll it often (e.g. in your control task):

```c
picobot_failsafe_arm(500);           /* same 500 ms idea as ControlReceiver */
for (;;) {
    if (command_arrived()) { picobot_motors_set(l, r); picobot_failsafe_feed(); }
    if (picobot_failsafe_poll()) logln("failsafe: motors stopped");
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

## Notes

- Motors follow `motors.py`: **one PWM pin per motor at a time**, the other driven low; the
  two pins of a motor share one PWM slice.
- The buzzer clock matches the course's `tone()` (2.34375 MHz PWM clock).
- The failsafe trips once and disarms; re-arm (or just feed) on the next command.
