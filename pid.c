#include "pid.h"

#include <stddef.h>

static int32_t clamp_i32(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void pid_init(PidController *pid, int32_t kp, int32_t ki, int32_t kd,
    int32_t scale, int32_t integralLimit, int32_t outputLimit)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->scale = (scale == 0) ? 1 : scale;
    pid->integralLimit = integralLimit;
    pid->outputLimit = outputLimit;
    pid_reset(pid);
}

void pid_reset(PidController *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0;
    pid->previousError = 0;
    pid->initialized = 0U;
}

int32_t pid_step_error(PidController *pid, int32_t error)
{
    int32_t candidateIntegral;
    int32_t derivative;
    int32_t output;

    if (pid == NULL) {
        return 0;
    }

    derivative = pid->initialized ? (error - pid->previousError) : 0;
    pid->initialized = 1U;
    pid->previousError = error;
    candidateIntegral = clamp_i32(pid->integral + error,
        pid->integralLimit);

    output = ((pid->kp * error) + (pid->ki * candidateIntegral) +
        (pid->kd * derivative)) / pid->scale;
    if (((output > pid->outputLimit) && (error > 0)) ||
        ((output < -pid->outputLimit) && (error < 0))) {
        output = ((pid->kp * error) + (pid->ki * pid->integral) +
            (pid->kd * derivative)) / pid->scale;
    } else {
        pid->integral = candidateIntegral;
    }
    return clamp_i32(output, pid->outputLimit);
}

int32_t pid_step(PidController *pid, int32_t setpoint,
    int32_t measurement)
{
    return pid_step_error(pid, setpoint - measurement);
}
