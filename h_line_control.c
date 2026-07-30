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

static int16_t divide_round_i16(int16_t value, int16_t divisor)
{
    if (value >= 0) {
        return (int16_t)((value + (divisor / 2)) / divisor);
    }
    return (int16_t)-((-value + (divisor / 2)) / divisor);
}

static int16_t filter_step_x4(int16_t filtered, int16_t target)
{
    int16_t delta = (int16_t)(target - filtered);

    if (delta > 0) {
        return (int16_t)(filtered + ((delta + 3) / 4));
    }
    if (delta < 0) {
        return (int16_t)(filtered - (((-delta) + 3) / 4));
    }
    return filtered;
}

void h_line_control_reset(HLineControl *control)
{
    if (control == NULL) {
        return;
    }
    control->periodMs = 0U;
    control->lostMs = 0U;
    control->lineVisible = false;
    control->filterReady = false;
    control->error = 0;
    control->lastVisibleError = 0;
    control->filteredErrorX4 = 0;
    control->previousErrorX4 = 0;
    control->derivativeX4 = 0;
    control->derivative = 0;
    control->correctionX4 = 0;
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
    int16_t requestedX4;
    int16_t kp;
    int16_t kd;
    int16_t correctionLimit;
    int16_t correctionSlewX4;

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
        if (!control->filterReady ||
            (rawError >= H_LINE_LARGE_ERROR) ||
            (rawError <= -H_LINE_LARGE_ERROR)) {
            control->filteredErrorX4 = (int16_t)(rawError * 4);
            control->filterReady = true;
        } else {
            control->filteredErrorX4 = filter_step_x4(
                control->filteredErrorX4, (int16_t)(rawError * 4));
        }
        control->error = divide_round_i16(control->filteredErrorX4, 4);
        control->derivativeX4 = (int16_t)(control->filteredErrorX4 -
            control->previousErrorX4);
        control->derivativeX4 = clamp_i16(control->derivativeX4,
            (int16_t)(H_LINE_DERIVATIVE_LIMIT * 4));
        control->derivative = divide_round_i16(control->derivativeX4, 4);
        control->previousErrorX4 = control->filteredErrorX4;
        control->lastVisibleError = rawError;
        control->lineVisible = true;
        control->lostMs = 0U;

        kp = curveMode ? H_LINE_CURVE_KP : H_LINE_STRAIGHT_KP;
        kd = curveMode ? H_LINE_CURVE_KD : H_LINE_STRAIGHT_KD;
        correctionLimit = curveMode ?
            H_LINE_CURVE_CORRECTION_LIMIT_TICKS :
            H_LINE_STRAIGHT_CORRECTION_LIMIT_TICKS;
        requestedX4 = clamp_i16(((int32_t)kp *
            control->filteredErrorX4 + (int32_t)kd *
            control->derivativeX4) / H_LINE_SCALE,
            (int16_t)(correctionLimit * 4));
    } else {
        control->lineVisible = false;
        if (control->lostMs <= (uint16_t)(UINT16_MAX -
                LINE_CONTROL_PERIOD_MS)) {
            control->lostMs = (uint16_t)(control->lostMs +
                LINE_CONTROL_PERIOD_MS);
        }
        control->error = control->lastVisibleError;
        control->derivativeX4 = 0;
        control->derivative = 0;
        if (control->lostMs < H_LINE_RECOVERY_START_MS) {
            requestedX4 = control->correctionX4;
        } else if (control->lastVisibleError > 0) {
            requestedX4 = H_LINE_RECOVERY_CORRECTION_TICKS * 4;
        } else if (control->lastVisibleError < 0) {
            requestedX4 = -H_LINE_RECOVERY_CORRECTION_TICKS * 4;
        } else {
            requestedX4 = control->correctionX4;
        }
    }
    correctionSlewX4 = curveMode ? H_LINE_CURVE_CORRECTION_SLEW_X4 :
        H_LINE_STRAIGHT_CORRECTION_SLEW_X4;
    if (requestedX4 > control->correctionX4) {
        control->correctionX4 += correctionSlewX4;
        if (control->correctionX4 > requestedX4) {
            control->correctionX4 = requestedX4;
        }
    } else if (requestedX4 < control->correctionX4) {
        control->correctionX4 -= correctionSlewX4;
        if (control->correctionX4 < requestedX4) {
            control->correctionX4 = requestedX4;
        }
    }
    control->correction = divide_round_i16(control->correctionX4, 4);
}

void h_line_control_command(const HLineControl *control,
    int16_t forwardSpeedTicks, int16_t steeringFeedforwardX4)
{
    int16_t correctionX4;
    int16_t limitX4;

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
    limitX4 = (forwardSpeedTicks < 0) ? (int16_t)-forwardSpeedTicks :
        forwardSpeedTicks;
    if (control->lineVisible) {
        if (limitX4 > H_LINE_MIN_VISIBLE_WHEEL_SPEED_TICKS) {
            limitX4 = (int16_t)(limitX4 -
                H_LINE_MIN_VISIBLE_WHEEL_SPEED_TICKS);
        } else {
            limitX4 = 0;
        }
    }
    if (!control->lineVisible &&
        (control->lostMs >= H_LINE_RECOVERY_START_MS) &&
        (steeringFeedforwardX4 != 0)) {
        steeringFeedforwardX4 *= 2;
    }
    correctionX4 = (int16_t)(control->correctionX4 +
        steeringFeedforwardX4);
    limitX4 = (int16_t)(limitX4 * 4);
    if (correctionX4 > limitX4) {
        correctionX4 = limitX4;
    } else if (correctionX4 < -limitX4) {
        correctionX4 = (int16_t)-limitX4;
    }
    motion_control_set_forward_steering_x4(forwardSpeedTicks,
        correctionX4);
}

void h_line_control_command_pwm(const HLineControl *control,
    int16_t forwardSpeedTicks)
{
    int16_t basePwm;
    int16_t correctionPwm;

    if ((control == NULL) || (forwardSpeedTicks <= 0)) {
        motion_control_set_pwm_targets(0, 0);
        return;
    }
    if (!control->lineVisible &&
        (control->lostMs >= H_LINE_SPEED_REDUCTION_START_MS) &&
        (forwardSpeedTicks > H_LINE_LOST_SPEED_TICKS)) {
        forwardSpeedTicks = H_LINE_LOST_SPEED_TICKS;
    }
    basePwm = (int16_t)(H_LINE_PWM_STATIC +
        forwardSpeedTicks * H_LINE_PWM_PER_SPEED_TICK);
    correctionPwm = (int16_t)(((int32_t)control->filteredErrorX4 *
        H_LINE_PWM_ERROR_GAIN) / 4);
    correctionPwm = clamp_i16(correctionPwm,
        H_LINE_PWM_CORRECTION_LIMIT);

    /* motor.c logical sides are opposite to the proven 24H side names. */
    motion_control_set_pwm_targets((int16_t)(basePwm - correctionPwm),
        (int16_t)(basePwm + correctionPwm));
}
