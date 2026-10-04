/*
 * picobot.c — PicoBot hardware helpers (Pico SDK, RP2350).
 *
 * Motors use the two-PWM-pin H-bridge scheme from lib/picobot/motors.py: only
 * one pin of a motor is in PWM mode at a time, the other is driven low. The
 * two pins of each motor share one PWM slice (left GP13/GP12 -> slice 6,
 * right GP10/GP11 -> slice 5), so both channels are configured together.
 */
#include "picobot.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

/* ================= motors ================= */

static void motor_pin_low(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static void motor_drive(uint pin_fwd, uint pin_rev, int16_t speed) {
    const uint slice = pwm_gpio_to_slice_num(pin_fwd);
    const uint ch_fwd = pwm_gpio_to_channel(pin_fwd);
    const uint ch_rev = pwm_gpio_to_channel(pin_rev);

    if (speed == 0) {
        motor_pin_low(pin_fwd);
        motor_pin_low(pin_rev);
        pwm_set_chan_level(slice, ch_fwd, 0);
        pwm_set_chan_level(slice, ch_rev, 0);
        return;
    }

    const uint16_t duty =
        (uint16_t)((uint32_t)(speed < 0 ? -speed : speed) * 65535u / 255u);

    if (speed > 0) {
        gpio_set_function(pin_fwd, GPIO_FUNC_PWM);
        motor_pin_low(pin_rev);
        pwm_set_chan_level(slice, ch_fwd, duty);
        pwm_set_chan_level(slice, ch_rev, 0);
    } else {
        gpio_set_function(pin_rev, GPIO_FUNC_PWM);
        motor_pin_low(pin_fwd);
        pwm_set_chan_level(slice, ch_rev, duty);
        pwm_set_chan_level(slice, ch_fwd, 0);
    }
    pwm_set_enabled(slice, true);
}

void picobot_motors_init(void) {
    const uint slices[2] = {
        pwm_gpio_to_slice_num(PICOBOT_MOTOR_LEFT_A),
        pwm_gpio_to_slice_num(PICOBOT_MOTOR_RIGHT_A),
    };
    for (int i = 0; i < 2; i++) {
        pwm_set_wrap(slices[i], 65535u);
        /* wrap 65535 -> freq = clk / (clkdiv * 65536) */
        pwm_set_clkdiv(slices[i],
                       (float)clock_get_hz(clk_sys) /
                           ((float)PICOBOT_MOTOR_PWM_HZ * 65536.0f));
        pwm_set_enabled(slices[i], true);
    }
    picobot_motors_stop();
}

void picobot_motors_set(int16_t left, int16_t right) {
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    motor_drive(PICOBOT_MOTOR_LEFT_A, PICOBOT_MOTOR_LEFT_B, left);
    motor_drive(PICOBOT_MOTOR_RIGHT_A, PICOBOT_MOTOR_RIGHT_B, right);
}

void picobot_motors_stop(void) {
    picobot_motors_set(0, 0);
}

/* ================= buzzer ================= */

void picobot_buzzer_init(void) {
    const uint slice = pwm_gpio_to_slice_num(PICOBOT_BUZZER_PIN);
    gpio_set_function(PICOBOT_BUZZER_PIN, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 64.0f);          /* 150 MHz / 64 = 2.34375 MHz */
    pwm_set_enabled(slice, false);
}

void picobot_buzzer_tone(uint32_t freq_hz) {
    if (freq_hz == 0) {
        picobot_buzzer_off();
        return;
    }
    const uint slice = pwm_gpio_to_slice_num(PICOBOT_BUZZER_PIN);
    const uint32_t wrap = 2343750u / freq_hz - 1u;   /* matches the course tone() */
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(PICOBOT_BUZZER_PIN, wrap / 2u);
    pwm_set_enabled(slice, true);
}

void picobot_buzzer_off(void) {
    pwm_set_enabled(pwm_gpio_to_slice_num(PICOBOT_BUZZER_PIN), false);
}

/* ================= I2C ================= */

void picobot_i2c_init(void) {
    i2c_init(i2c1, PICOBOT_I2C_FREQ_HZ);
    gpio_set_function(PICOBOT_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PICOBOT_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PICOBOT_I2C_SDA);
    gpio_pull_up(PICOBOT_I2C_SCL);
}

/* ================= failsafe ================= */

static uint32_t fs_timeout_ms = 0;
static uint32_t fs_last_feed_ms = 0;
static bool fs_armed = false;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void picobot_failsafe_arm(uint32_t timeout_ms) {
    fs_timeout_ms = timeout_ms;
    fs_last_feed_ms = now_ms();
    fs_armed = true;
}

void picobot_failsafe_feed(void) {
    fs_last_feed_ms = now_ms();
}

uint32_t picobot_failsafe_age_ms(void) {
    return now_ms() - fs_last_feed_ms;
}

bool picobot_failsafe_poll(void) {
    if (!fs_armed) return false;
    if (picobot_failsafe_age_ms() <= fs_timeout_ms) return false;
    picobot_motors_stop();
    fs_armed = false;                     /* trip once; re-arm on the next command */
    return true;
}
