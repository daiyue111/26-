#include "line_follow.h"

#include "app_config.h"
#include "app_io.h"
#include "imu.h"
#include "motor.h"
#include "track.h"

#include <stddef.h>

static uint8_t clamp_duty(int32_t duty)
{
    if (duty < 0) {
        return 0U;
    }
    if (duty > (int32_t)PWM_PERIOD_TICKS) {
        return PWM_PERIOD_TICKS;
    }
    return (uint8_t)duty;
}

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

void line_follow_reset(LineFollowState *state, bool imuReady)
{
    if (state == NULL) {
        return;
    }

    state->lastError = 0;
    state->lineWasLost = false;
    state->imuReady = imuReady;
    state->imuReadOk = imuReady;
    state->lostHeading = 0;
    state->imuFailMs = 0U;
    state->error = 0;
    state->derivative = 0;
    state->leftDuty = 0U;
    state->rightDuty = 0U;
    state->gyroZDelta = 0;
    state->gyroLostHeading = 0;
    state->gyroCorrectionTicks = 0;
}

void line_follow_run_1ms(LineFollowState *state, uint8_t blackMask)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((blackMask & LINE_CENTER_MASK) == LINE_CENTER_MASK) {
        state->lastError = 0;
        state->lineWasLost = false;
        state->lostHeading = 0;
        state->imuFailMs = 0U;
        state->error = 0;
        state->derivative = 0;
        state->gyroLostHeading = 0;
        state->gyroCorrectionTicks = 0;
        leftDuty = LINE_BASE_LEFT_DUTY;
        rightDuty = LINE_BASE_RIGHT_DUTY;
    } else if (blackMask != 0U) {
        int16_t error = track_position_error(blackMask);
        int16_t derivative = state->lineWasLost ? 0 :
            (error - state->lastError);
        int32_t correction = ((int32_t)error * LINE_KP) +
            ((int32_t)derivative * LINE_KD);

        state->lastError = error;
        state->lineWasLost = false;
        state->lostHeading = 0;
        state->imuFailMs = 0U;
        state->error = error;
        state->derivative = derivative;
        state->gyroLostHeading = 0;
        state->gyroCorrectionTicks = 0;
        leftDuty = clamp_duty((int32_t)LINE_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)LINE_BASE_RIGHT_DUTY + correction);
    } else {
        int32_t correctionPercent = 0;
        int32_t correctionTicks = 0;

        if (!state->lineWasLost) {
            state->lostHeading = 0;
            state->imuFailMs = 0U;
        }
        state->lineWasLost = true;
        state->error = 0;
        state->derivative = 0;

        if (state->imuReady &&
            (state->imuFailMs < GYRO_FAIL_LIMIT_MS)) {
            int32_t gz = 0;

            if (imu_read_gyro_z_delta(&gz)) {
                state->imuFailMs = 0U;
                state->imuReadOk = true;
                app_gyro_led_set(true);
                state->gyroZDelta = gz;
                if ((gz > GYRO_DEAD_RAW) || (gz < -GYRO_DEAD_RAW)) {
                    state->lostHeading += gz;
                }
                correctionPercent = -((state->lostHeading *
                    GYRO_HEADING_KP) / GYRO_HEADING_CORR_DIV);
                correctionPercent = clamp_i32(correctionPercent,
                    GYRO_HEADING_CORR_MAX);
                correctionTicks = correctionPercent *
                    (int32_t)GYRO_PWM_TICKS_PER_PERCENT;
            } else {
                state->imuFailMs++;
                state->imuReadOk = false;
                app_gyro_led_set(false);
            }
        }

        state->gyroLostHeading = state->lostHeading;
        state->gyroCorrectionTicks = (int16_t)correctionTicks;
        leftDuty = clamp_duty((int32_t)LINE_LOST_LEFT_DUTY +
            correctionTicks);
        rightDuty = clamp_duty((int32_t)LINE_LOST_RIGHT_DUTY -
            correctionTicks);
    }

    state->leftDuty = leftDuty;
    state->rightDuty = rightDuty;
    motor_pwm_run_1ms(leftDuty, rightDuty);
}
