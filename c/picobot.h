/*
 * picobot.h — PicoBot hardware map and helpers for the C/Pico SDK firmware.
 *
 * This is the C translation of lib/picobot/config.py (the MicroPython pin map).
 * Board: RP2350 (Pico 2 W). One AM1016A H-bridge per motor, two pins per motor.
 *
 * NOTE: verify against your unit's silkscreen/wiring before first use; the
 * map is taken from the MicroPython config and the motors follow motors.py's
 * two-PWM-pin scheme (including its RP2350 duty=0 behaviour).
 */
#ifndef PICOBOT_H
#define PICOBOT_H

#include <stdbool.h>
#include <stdint.h>

/* ---- motors (H-bridge, two pins each) ---------------------------------- */
#define PICOBOT_MOTOR_LEFT_A   13u   /* forward  */
#define PICOBOT_MOTOR_LEFT_B   12u   /* backward */
#define PICOBOT_MOTOR_RIGHT_A  10u   /* forward  */
#define PICOBOT_MOTOR_RIGHT_B  11u   /* backward */
#define PICOBOT_MOTOR_PWM_HZ   1000u

/* ---- buzzer (PWM) ------------------------------------------------------- */
#define PICOBOT_BUZZER_PIN     22u

/* ---- I2C bus (BMI160 IMU; OLED / VL53L1X ToF share it) ------------------ */
#define PICOBOT_I2C_SDA        14u
#define PICOBOT_I2C_SCL        15u
#define PICOBOT_I2C_FREQ_HZ    400000u

/* ---- NeoPixel panel (PIO-driven; helper not implemented yet) ------------ */
#define PICOBOT_NEOPIXEL_PIN   6u
#define PICOBOT_NEOPIXEL_COUNT 8u

/* ---- motors ------------------------------------------------------------- */
void picobot_motors_init(void);
void picobot_motors_set(int16_t left, int16_t right);   /* -255..255 */
void picobot_motors_stop(void);

/* ---- buzzer ------------------------------------------------------------- */
void picobot_buzzer_init(void);
void picobot_buzzer_tone(uint32_t freq_hz);              /* 0 = silent */
void picobot_buzzer_off(void);

/* ---- I2C ---------------------------------------------------------------- */
void picobot_i2c_init(void);

/* ---- failsafe -----------------------------------------------------------
 * Arm with a timeout, feed on every valid command, poll often. If a poll
 * finds the last feed older than the timeout, the motors stop. This is the
 * C equivalent of ControlReceiver's 500 ms timeout. */
void     picobot_failsafe_arm(uint32_t timeout_ms);
void     picobot_failsafe_feed(void);
bool     picobot_failsafe_poll(void);                    /* true if it just tripped */
uint32_t picobot_failsafe_age_ms(void);

#endif /* PICOBOT_H */
