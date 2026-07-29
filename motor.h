#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_safe_stop(void);
void motor_active_brake(void);
void motor_forward(void);
void motor_pwm_run_1ms(uint8_t leftDuty, uint8_t rightDuty);
void motor_set_signed(int16_t leftDuty, int16_t rightDuty);
void motor_test_set_direction(uint8_t step);
void motor_test_pwm_run_1ms(uint8_t step);

#endif
