#include "app_io.h"

void app_delay_ms(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

void app_gpio_write(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

bool app_gpio_is_high(GPIO_Regs *port, uint32_t pin)
{
    return DL_GPIO_readPins(port, pin) != 0U;
}

bool app_start_key_pressed(void)
{
    return !app_gpio_is_high(GPIO_START_KEY_PORT, GPIO_START_KEY_KEY_PIN);
}

void app_debug_led_set(bool on)
{
    app_gpio_write(GPIO_DEBUG_LED_PORT, GPIO_DEBUG_LED_LED_PIN, on);
}

void app_gyro_led_set(bool on)
{
    app_gpio_write(GPIO_GYRO_LED_PORT, GPIO_GYRO_LED_STATUS_LED_PIN, on);
}

void app_gyro_led_startup_test(void)
{
    app_gyro_led_set(true);
    app_delay_ms(1000U);
    app_gyro_led_set(false);
}

void app_gyro_led_blink_error(uint8_t error)
{
    uint8_t pulses = (error == 0U) ? 1U : error;

    for (uint8_t repeat = 0U; repeat < 2U; repeat++) {
        for (uint8_t pulse = 0U; pulse < pulses; pulse++) {
            app_gyro_led_set(true);
            app_delay_ms(400U);
            app_gyro_led_set(false);
            app_delay_ms(400U);
        }
        app_delay_ms(1500U);
    }
}

void app_gyro_led_report_imu_error(uint8_t error)
{
    for (uint8_t pulse = 0U; pulse < 3U; pulse++) {
        app_gyro_led_set(true);
        app_delay_ms(800U);
        app_gyro_led_set(false);
        app_delay_ms(400U);
    }
    app_delay_ms(1000U);
    app_gyro_led_blink_error(error);
}

void app_gyro_led_report_motion_error(uint8_t error)
{
    app_gyro_led_set(true);
    app_delay_ms(4000U);
    app_gyro_led_set(false);
    app_delay_ms(1000U);
    app_gyro_led_blink_error(error);
}
