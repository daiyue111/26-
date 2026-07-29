#include "motion_control.h"

#include "app_config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

typedef struct {
    bool enabled;
    uint8_t periodMs;
    int16_t leftTarget;
    int16_t rightTarget;
    int16_t leftSpeed;
    int16_t rightSpeed;
    int16_t leftPwm;
    int16_t rightPwm;
    int32_t previousLeftCount;
    int32_t previousRightCount;
    uint16_t leftNoFeedbackMs;
    uint16_t rightNoFeedbackMs;
    uint16_t leftDirectionFaultMs;
    uint16_t rightDirectionFaultMs;
    uint16_t leftOverspeedMs;
    uint16_t rightOverspeedMs;
    MotionControlFault fault;
    PidController leftPid;
    PidController rightPid;
} MotionControlState;

static MotionControlState gMotion;

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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t speed_feedforward(int16_t target)
{
    if (target > 0) {
        return SPEED_STATIC_FRICTION_PWM +
            (int32_t)target * SPEED_FEEDFORWARD_PER_TICK;
    }
    if (target < 0) {
        return -SPEED_STATIC_FRICTION_PWM +
            (int32_t)target * SPEED_FEEDFORWARD_PER_TICK;
    }
    return 0;
}

static int16_t slew_pwm(int16_t current, int32_t requested)
{
    if (requested > current + SPEED_PWM_SLEW_PER_UPDATE) {
        return (int16_t)(current + SPEED_PWM_SLEW_PER_UPDATE);
    }
    if (requested < current - SPEED_PWM_SLEW_PER_UPDATE) {
        return (int16_t)(current - SPEED_PWM_SLEW_PER_UPDATE);
    }
    return (int16_t)requested;
}

static void update_fault_timer(bool condition, uint16_t *timerMs)
{
    if (condition) {
        if (*timerMs <= (uint16_t)(UINT16_MAX - SPEED_CONTROL_PERIOD_MS)) {
            *timerMs = (uint16_t)(*timerMs + SPEED_CONTROL_PERIOD_MS);
        }
    } else {
        *timerMs = 0U;
    }
}

static void motion_control_check_feedback(void)
{
    bool leftCommanded = (gMotion.leftTarget != 0) &&
        (abs_i32(gMotion.leftPwm) >= ENCODER_WATCHDOG_MIN_PWM);
    bool rightCommanded = (gMotion.rightTarget != 0) &&
        (abs_i32(gMotion.rightPwm) >= ENCODER_WATCHDOG_MIN_PWM);
    bool leftWrongDirection = ((gMotion.leftTarget > 0) &&
        (gMotion.leftSpeed < 0)) || ((gMotion.leftTarget < 0) &&
        (gMotion.leftSpeed > 0));
    bool rightWrongDirection = ((gMotion.rightTarget > 0) &&
        (gMotion.rightSpeed < 0)) || ((gMotion.rightTarget < 0) &&
        (gMotion.rightSpeed > 0));

    update_fault_timer(leftCommanded && (gMotion.leftSpeed == 0),
        &gMotion.leftNoFeedbackMs);
    update_fault_timer(rightCommanded && (gMotion.rightSpeed == 0),
        &gMotion.rightNoFeedbackMs);
    update_fault_timer(leftCommanded && leftWrongDirection,
        &gMotion.leftDirectionFaultMs);
    update_fault_timer(rightCommanded && rightWrongDirection,
        &gMotion.rightDirectionFaultMs);
    update_fault_timer(
        abs_i32(gMotion.leftSpeed) > ENCODER_OVERSPEED_TICKS,
        &gMotion.leftOverspeedMs);
    update_fault_timer(
        abs_i32(gMotion.rightSpeed) > ENCODER_OVERSPEED_TICKS,
        &gMotion.rightOverspeedMs);

    if (gMotion.leftNoFeedbackMs >= ENCODER_NO_FEEDBACK_TIMEOUT_MS) {
        gMotion.fault = MOTION_FAULT_LEFT_NO_FEEDBACK;
    } else if (gMotion.rightNoFeedbackMs >=
        ENCODER_NO_FEEDBACK_TIMEOUT_MS) {
        gMotion.fault = MOTION_FAULT_RIGHT_NO_FEEDBACK;
    } else if (gMotion.leftDirectionFaultMs >=
        ENCODER_DIRECTION_FAULT_MS) {
        gMotion.fault = MOTION_FAULT_LEFT_DIRECTION;
    } else if (gMotion.rightDirectionFaultMs >=
        ENCODER_DIRECTION_FAULT_MS) {
        gMotion.fault = MOTION_FAULT_RIGHT_DIRECTION;
    } else if (gMotion.leftOverspeedMs >= ENCODER_OVERSPEED_FAULT_MS) {
        gMotion.fault = MOTION_FAULT_LEFT_OVERSPEED;
    } else if (gMotion.rightOverspeedMs >= ENCODER_OVERSPEED_FAULT_MS) {
        gMotion.fault = MOTION_FAULT_RIGHT_OVERSPEED;
    }
}

void motion_control_init(void)
{
    encoder_init();
    pid_init(&gMotion.leftPid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
        SPEED_PID_SCALE, SPEED_PID_INTEGRAL_LIMIT,
        SPEED_PID_OUTPUT_LIMIT);
    pid_init(&gMotion.rightPid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
        SPEED_PID_SCALE, SPEED_PID_INTEGRAL_LIMIT,
        SPEED_PID_OUTPUT_LIMIT);
    motion_control_reset();
}

void motion_control_enable(bool enable)
{
    gMotion.enabled = enable;
    if (!enable) {
        gMotion.leftTarget = 0;
        gMotion.rightTarget = 0;
        gMotion.leftPwm = 0;
        gMotion.rightPwm = 0;
        pid_reset(&gMotion.leftPid);
        pid_reset(&gMotion.rightPid);
        motor_safe_stop();
    }
}

void motion_control_reset(void)
{
    EncoderCounts counts;

    encoder_reset();
    counts = encoder_get_counts();
    gMotion.enabled = false;
    gMotion.periodMs = 0U;
    gMotion.leftTarget = 0;
    gMotion.rightTarget = 0;
    gMotion.leftSpeed = 0;
    gMotion.rightSpeed = 0;
    gMotion.leftPwm = 0;
    gMotion.rightPwm = 0;
    gMotion.previousLeftCount = counts.left;
    gMotion.previousRightCount = counts.right;
    gMotion.leftNoFeedbackMs = 0U;
    gMotion.rightNoFeedbackMs = 0U;
    gMotion.leftDirectionFaultMs = 0U;
    gMotion.rightDirectionFaultMs = 0U;
    gMotion.leftOverspeedMs = 0U;
    gMotion.rightOverspeedMs = 0U;
    gMotion.fault = MOTION_FAULT_NONE;
    pid_reset(&gMotion.leftPid);
    pid_reset(&gMotion.rightPid);
    motor_safe_stop();
}

void motion_control_set_speed_targets(int16_t left, int16_t right)
{
    int16_t newLeft = (int16_t)clamp_i32(left,
        WHEEL_SPEED_TARGET_LIMIT);
    int16_t newRight = (int16_t)clamp_i32(right,
        WHEEL_SPEED_TARGET_LIMIT);

    if ((newLeft == 0) || ((newLeft > 0) != (gMotion.leftTarget > 0))) {
        pid_reset(&gMotion.leftPid);
    }
    if ((newRight == 0) || ((newRight > 0) !=
            (gMotion.rightTarget > 0))) {
        pid_reset(&gMotion.rightPid);
    }
    gMotion.leftTarget = newLeft;
    gMotion.rightTarget = newRight;
}

void motion_control_update_1ms(void)
{
    EncoderCounts counts;
    int32_t leftOutput;
    int32_t rightOutput;

    if (!gMotion.enabled) {
        return;
    }

    gMotion.periodMs++;
    if (gMotion.periodMs < SPEED_CONTROL_PERIOD_MS) {
        return;
    }
    gMotion.periodMs = 0U;

    counts = encoder_get_counts();
    gMotion.leftSpeed = (int16_t)(counts.left - gMotion.previousLeftCount);
    gMotion.rightSpeed = (int16_t)(counts.right -
        gMotion.previousRightCount);
    gMotion.previousLeftCount = counts.left;
    gMotion.previousRightCount = counts.right;

    motion_control_check_feedback();
    if (gMotion.fault != MOTION_FAULT_NONE) {
        motion_control_enable(false);
        return;
    }

    leftOutput = (gMotion.leftTarget == 0) ? 0 :
        pid_step(&gMotion.leftPid, gMotion.leftTarget,
            gMotion.leftSpeed) + speed_feedforward(gMotion.leftTarget);
    rightOutput = (gMotion.rightTarget == 0) ? 0 :
        pid_step(&gMotion.rightPid, gMotion.rightTarget,
            gMotion.rightSpeed) + speed_feedforward(gMotion.rightTarget);
    leftOutput = clamp_i32(leftOutput, SPEED_PWM_HARD_LIMIT);
    rightOutput = clamp_i32(rightOutput, SPEED_PWM_HARD_LIMIT);
    gMotion.leftPwm = (gMotion.leftTarget == 0) ? 0 :
        slew_pwm(gMotion.leftPwm, leftOutput);
    gMotion.rightPwm = (gMotion.rightTarget == 0) ? 0 :
        slew_pwm(gMotion.rightPwm, rightOutput);
    motor_set_signed(gMotion.leftPwm, gMotion.rightPwm);
}

int32_t motion_control_get_left_count(void)
{
    return encoder_get_counts().left;
}

int32_t motion_control_get_right_count(void)
{
    return encoder_get_counts().right;
}

int32_t motion_control_get_average_count(void)
{
    EncoderCounts counts = encoder_get_counts();

    return (counts.left + counts.right) / 2;
}

int16_t motion_control_get_left_speed(void)
{
    return gMotion.leftSpeed;
}

int16_t motion_control_get_right_speed(void)
{
    return gMotion.rightSpeed;
}

int16_t motion_control_get_left_pwm(void)
{
    return gMotion.leftPwm;
}

int16_t motion_control_get_right_pwm(void)
{
    return gMotion.rightPwm;
}

MotionControlFault motion_control_get_fault(void)
{
    return gMotion.fault;
}
