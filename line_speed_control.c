#include "line_speed_control.h"

#include "app_config.h"
#include "motion_control.h"
#include "track.h"

#include <stddef.h>

void line_speed_control_init(LineSpeedController *controller)
{
    if (controller == NULL) {
        return;
    }

    pid_init(&controller->pid, SQUARE_LINE_PID_KP, SQUARE_LINE_PID_KI,
        SQUARE_LINE_PID_KD, SQUARE_LINE_PID_SCALE,
        SQUARE_LINE_CORR_LIMIT * 20, SQUARE_LINE_CORR_LIMIT);
    line_speed_control_reset(controller);
}

void line_speed_control_reset(LineSpeedController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->periodMs = 0U;
    controller->centerLostCycles = 0U;
    controller->error = 0;
    controller->filteredErrorX4 = 0;
    controller->correction = 0;
    pid_reset(&controller->pid);
}

void line_speed_control_update_1ms(LineSpeedController *controller,
    uint8_t blackMask)
{
    int16_t rawError;
    int16_t targetCorrection;

    if (controller == NULL) {
        return;
    }

    controller->periodMs++;
    if (controller->periodMs < LINE_CONTROL_PERIOD_MS) {
        return;
    }
    controller->periodMs = 0U;

    if ((blackMask & LINE_CENTER_MASK) != 0U) {
        controller->centerLostCycles = 0U;
        controller->error = 0;
        controller->filteredErrorX4 = 0;
        controller->correction = 0;
        pid_reset(&controller->pid);
        return;
    }

    if ((blackMask != 0U) &&
        (controller->centerLostCycles <
            SQUARE_LINE_CENTER_RELEASE_CYCLES)) {
        controller->centerLostCycles++;
        controller->error = 0;
        controller->filteredErrorX4 = 0;
        controller->correction = 0;
        pid_reset(&controller->pid);
        return;
    }

    rawError = (blackMask == 0U) ? 0 :
        track_position_error(blackMask);
    if ((controller->filteredErrorX4 == 0) ||
        (rawError >= SQUARE_LINE_EDGE_ERROR) ||
        (rawError <= -SQUARE_LINE_EDGE_ERROR)) {
        controller->filteredErrorX4 = (int16_t)(rawError * 4);
    } else {
        controller->filteredErrorX4 = (int16_t)(
            (controller->filteredErrorX4 + (rawError * 4)) / 2);
    }
    controller->error = (int16_t)(controller->filteredErrorX4 / 4);
    targetCorrection = (int16_t)pid_step_error(&controller->pid,
        controller->error);
    if ((controller->error >= SQUARE_LINE_EDGE_ERROR) &&
        (targetCorrection < SQUARE_LINE_CORR_LIMIT)) {
        targetCorrection++;
    } else if ((controller->error <= -SQUARE_LINE_EDGE_ERROR) &&
        (targetCorrection > -SQUARE_LINE_CORR_LIMIT)) {
        targetCorrection--;
    }
    if ((targetCorrection == 0) ||
        ((targetCorrection > 0) && (controller->correction < 0)) ||
        ((targetCorrection < 0) && (controller->correction > 0))) {
        controller->correction = 0;
    } else if (targetCorrection > controller->correction) {
        controller->correction++;
    } else if (targetCorrection < controller->correction) {
        controller->correction--;
    }
}

void line_speed_control_command(const LineSpeedController *controller,
    int16_t forwardSpeed)
{
    int16_t effectiveForward;
    int16_t correction;
    int16_t correctionLimit;
    int16_t errorMagnitude;

    if (controller == NULL) {
        motion_control_set_speed_targets(0, 0);
        return;
    }

    effectiveForward = forwardSpeed;
    errorMagnitude = (controller->error < 0) ?
        (int16_t)-controller->error : controller->error;
    if (forwardSpeed >= SQUARE_CRUISE_SPEED_TICKS) {
        if (errorMagnitude >= SQUARE_LINE_LARGE_ERROR) {
            effectiveForward = SQUARE_LINE_LARGE_SPEED_TICKS;
        } else if (errorMagnitude >= SQUARE_LINE_MEDIUM_ERROR) {
            effectiveForward = SQUARE_LINE_MEDIUM_SPEED_TICKS;
        }
    }

    correction = controller->correction;
    correctionLimit = (effectiveForward < 0) ?
        (int16_t)-effectiveForward : effectiveForward;
    if (correction > correctionLimit) {
        correction = correctionLimit;
    } else if (correction < -correctionLimit) {
        correction = (int16_t)-correctionLimit;
    }

    motion_control_set_speed_targets(
        (int16_t)(effectiveForward - correction),
        (int16_t)(effectiveForward + correction));
}
