#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_LEFT_NO_FEEDBACK,
    MOTION_FAULT_RIGHT_NO_FEEDBACK,
    MOTION_FAULT_LEFT_DIRECTION,
    MOTION_FAULT_RIGHT_DIRECTION,
    MOTION_FAULT_LEFT_OVERSPEED,
    MOTION_FAULT_RIGHT_OVERSPEED
} MotionControlFault;

void motion_control_init(void);
void motion_control_enable(bool enable);
void motion_control_reset(void);
void motion_control_set_speed_targets(int16_t left, int16_t right);
void motion_control_update_1ms(void);
int32_t motion_control_get_left_count(void);
int32_t motion_control_get_right_count(void);
int32_t motion_control_get_average_count(void);
int16_t motion_control_get_left_speed(void);
int16_t motion_control_get_right_speed(void);
int16_t motion_control_get_left_pwm(void);
int16_t motion_control_get_right_pwm(void);
MotionControlFault motion_control_get_fault(void);

#endif
