#include "h_line_control.h"

#include "app_config.h"
#include "motion_control.h"
#include "track.h"

#include <stddef.h>

static int16_t clamp_i16(int32_t value, int16_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

void h_line_control_reset(HLineControl *control)
{
    if (control == NULL) {
        return;
    }
    control->periodMs = 0U;
    control->lostMs = 0U;
    control->lineVisible = false;
    control->error = 0;
    control->lastVisibleError = 0;
    control->filteredErrorX4 = 0;
    control->previousError = 0;
    control->derivative = 0;
    control->correction = 0;
}

void h_line_control_init(HLineControl *control)
{
    h_line_control_reset(control);
}

void h_line_control_update_1ms(HLineControl *control, uint8_t blackMask,
    bool curveMode)
{
    int16_t rawError;
    int16_t requested;
    int16_t kp;
    int16_t kd;

    if (control == NULL) {
        return;
    }
    control->periodMs++;
    if (control->periodMs < LINE_CONTROL_PERIOD_MS) {
        return;
    }
    control->periodMs = 0U;

    if (blackMask != 0U) {
        rawError = track_position_error(blackMask);
        if ((control->filteredErrorX4 == 0) ||
            (rawError >= H_LINE_LARGE_ERROR) ||
            (rawError <= -H_LINE_LARGE_ERROR)) {
            control->filteredErrorX4 = (int16_t)(rawError * 4);
        } else {
            control->filteredErrorX4 = (int16_t)(
                (control->filteredErrorX4 + rawError * 4) / 2);
        }
        control->error = (int16_t)(control->filteredErrorX4 / 4);
        control->derivative = control->error - control->previousError;
        control->derivative = clamp_i16(control->derivative,
            H_LINE_DERIVATIVE_LIMIT);
        control->previousError = control->error;
        control->lastVisibleError = control->error;
        control->lineVisible = true;
        control->lostMs = 0U;

        kp = curveMode ? H_LINE_CURVE_KP : H_LINE_STRAIGHT_KP;
        kd = curveMode ? H_LINE_CURVE_KD : H_LINE_STRAIGHT_KD;
        requested = clamp_i16(((int32_t)kp * control->error +
            (int32_t)kd * control->derivative) / H_LINE_SCALE,
            H_LINE_CORRECTION_LIMIT_TICKS);
    } else {
        control->lineVisible = false;
        if (control->lostMs <= (uint16_t)(UINT16_MAX -
                LINE_CONTROL_PERIOD_MS)) {
            control->lostMs = (uint16_t)(control->lostMs +
                LINE_CONTROL_PERIOD_MS);
        }
        control->error = control->lastVisibleError;
        control->derivative = 0;
        if (control->lostMs < H_LINE_RECOVERY_START_MS) {
            requested = control->correction;
        } else if (control->lastVisibleError > 0) {
            requested = H_LINE_RECOVERY_CORRECTION_TICKS;
        } else if (control->lastVisibleError < 0) {
            requested = -H_LINE_RECOVERY_CORRECTION_TICKS;
        } else {
            requested = control->correction;
        }
    }
    if (requested > control->correction) {
        control->correction += H_LINE_CORRECTION_SLEW_TICKS;
        if (control->correction > requested) {
            control->correction = requested;
        }
    } else if (requested < control->correction) {
        control->correction -= H_LINE_CORRECTION_SLEW_TICKS;
        if (control->correction < requested) {
            control->correction = requested;
        }
    }
}

void h_line_control_command(const HLineControl *control,
    int16_t forwardSpeedTicks, int16_t steeringFeedforwardTicks)
{
    int16_t correction;
    int16_t limit;

    if (control == NULL) {
        motion_control_set_speed_targets(0, 0);
        return;
    }
    if (!control->lineVisible &&
        (control->lostMs >= H_LINE_SPEED_REDUCTION_START_MS)) {
        if (forwardSpeedTicks > H_LINE_LOST_SPEED_TICKS) {
            forwardSpeedTicks = H_LINE_LOST_SPEED_TICKS;
        }
    }
    limit = (forwardSpeedTicks < 0) ? (int16_t)-forwardSpeedTicks :
        forwardSpeedTicks;
    if (control->lineVisible) {
        if (limit > H_LINE_MIN_VISIBLE_WHEEL_SPEED_TICKS) {
            limit = (int16_t)(limit -
                H_LINE_MIN_VISIBLE_WHEEL_SPEED_TICKS);
        } else {
            limit = 0;
        }
    }
    if (!control->lineVisible &&
        (control->lostMs >= H_LINE_RECOVERY_START_MS) &&
        (steeringFeedforwardTicks != 0)) {
        steeringFeedforwardTicks *= 2;
    }
    correction = (int16_t)(control->correction +
        steeringFeedforwardTicks);
    if (correction > limit) {
        correction = limit;
    } else if (correction < -limit) {
        correction = (int16_t)-limit;
    }
    motion_control_set_speed_targets(
        (int16_t)(forwardSpeedTicks - correction),
        (int16_t)(forwardSpeedTicks + correction));
}
