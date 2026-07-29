#ifndef APP_IO_H
#define APP_IO_H

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

void app_delay_ms(uint32_t ms);
void app_gpio_write(GPIO_Regs *port, uint32_t pin, bool high);
bool app_gpio_is_high(GPIO_Regs *port, uint32_t pin);
bool app_start_key_pressed(void);
void app_debug_led_set(bool on);
void app_gyro_led_set(bool on);
void app_gyro_led_startup_test(void);
void app_gyro_led_blink_error(uint8_t error);
void app_gyro_led_report_imu_error(uint8_t error);
void app_gyro_led_report_motion_error(uint8_t error);

#endif
