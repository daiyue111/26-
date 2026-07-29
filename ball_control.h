#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void ball_control_init(void);
void ball_control_set_enabled(bool enabled);
void ball_control_set_target_tenth_mm(int16_t targetTenthMm);
void ball_control_update_1ms(uint32_t nowMs, int32_t chassisAccelMmps2);
bool ball_control_is_enabled(void);
bool ball_control_is_vision_healthy(void);
int16_t ball_control_get_target_tenth_mm(void);
int16_t ball_control_get_position_tenth_mm(void);
int16_t ball_control_get_velocity_tenth_mmps(void);
int16_t ball_control_get_error_tenth_mm(void);
int16_t ball_control_get_angle_command_mdeg(void);
uint32_t ball_control_get_vision_age_ms(uint32_t nowMs);

#endif
