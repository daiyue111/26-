#ifndef POWER_SWITCH_H
#define POWER_SWITCH_H

#include <stdbool.h>
#include <stdint.h>

void power_switch_init(void);
void power_switch_set(bool on);
void power_switch_set_duty(uint8_t percent);
void power_switch_pulse_ms(uint32_t durationMs);
void power_switch_off(void);
void power_switch_update_1ms(void);
uint8_t power_switch_get_duty(void);
bool power_switch_is_on(void);
bool power_switch_has_timed_out(void);

#endif
