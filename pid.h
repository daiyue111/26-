#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t scale;
    int32_t integralLimit;
    int32_t outputLimit;
    int32_t integral;
    int32_t previousError;
    uint8_t initialized;
} PidController;

void pid_init(PidController *pid, int32_t kp, int32_t ki, int32_t kd,
    int32_t scale, int32_t integralLimit, int32_t outputLimit);
void pid_reset(PidController *pid);
int32_t pid_step(PidController *pid, int32_t setpoint,
    int32_t measurement);
int32_t pid_step_error(PidController *pid, int32_t error);

#endif
