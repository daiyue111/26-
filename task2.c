#include "task2.h"

#include "chassis_model.h"
#include "motion_control.h"
#include "motor.h"
#include "task2_config.h"
#include "track.h"

#include <limits.h>

typedef enum {
    TASK2_PHASE_STOPPED = 0,
    TASK2_PHASE_RUNNING,
    TASK2_PHASE_PASS_FINISH,
    TASK2_PHASE_BRAKING,
    TASK2_PHASE_COMPLETE,
    TASK2_PHASE_FAULT
} Task2Phase;

typedef struct {
    uint8_t periodMs;
    uint16_t lostMs;
    bool visible;
    bool filterReady;
    int16_t error;
    int16_t filteredErrorX4;
    int16_t correctionPwm;
} Task2Line;

typedef struct {
    Task2Phase phase;
    CompetitionTaskFault fault;
    bool markerArmed;
    uint16_t phaseMs;
    uint16_t lineLostMs;
    uint16_t markerClearMs;
    uint16_t markerDebounceMs;
    uint16_t speedRampMs;
    uint8_t lineMaskHistory1;
    uint8_t lineMaskHistory2;
    uint32_t startMs;
    uint32_t resultMs;
    int32_t startCount;
    int32_t passStartCount;
    uint32_t finishPassCounts;
    int32_t distanceMm;
    int16_t currentSpeedTicks;
    Task2Line line;
} Task2Context;

static Task2Context gTask2;

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

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
        return (int16_t)((value + divisor / 2) / divisor);
    }
    return (int16_t)-((-value + divisor / 2) / divisor);
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

static void line_reset(void)
{
    gTask2.line.periodMs = 0U;
    gTask2.line.lostMs = 0U;
    gTask2.line.visible = false;
    gTask2.line.filterReady = false;
    gTask2.line.error = 0;
    gTask2.line.filteredErrorX4 = 0;
    gTask2.line.correctionPwm = 0;
}

static void line_update_1ms(uint8_t lineMask)
{
    int16_t rawError;

    gTask2.line.periodMs++;
    if (gTask2.line.periodMs < TASK2_CONTROL_PERIOD_MS) {
        return;
    }
    gTask2.line.periodMs = 0U;
    if (lineMask == 0U) {
        gTask2.line.visible = false;
        if (gTask2.line.lostMs <= (uint16_t)(UINT16_MAX -
                TASK2_CONTROL_PERIOD_MS)) {
            gTask2.line.lostMs = (uint16_t)(gTask2.line.lostMs +
                TASK2_CONTROL_PERIOD_MS);
        }
        return;
    }

    rawError = track_position_error(lineMask);
    if (!gTask2.line.filterReady ||
        (rawError >= TASK2_LINE_LARGE_ERROR) ||
        (rawError <= -TASK2_LINE_LARGE_ERROR)) {
        gTask2.line.filteredErrorX4 = (int16_t)(rawError * 4);
        gTask2.line.filterReady = true;
    } else {
        gTask2.line.filteredErrorX4 = filter_step_x4(
            gTask2.line.filteredErrorX4, (int16_t)(rawError * 4));
    }
    gTask2.line.error = divide_round_i16(
        gTask2.line.filteredErrorX4, 4);
    gTask2.line.visible = true;
    gTask2.line.lostMs = 0U;
}

static void line_command_pwm(void)
{
    int16_t speed = gTask2.currentSpeedTicks;
    int16_t basePwm;
    int16_t correctionPwm;

    if (!gTask2.line.visible &&
        (gTask2.line.lostMs >= TASK2_LINE_SPEED_REDUCTION_MS) &&
        (speed > TASK2_LINE_LOST_SPEED_TICKS)) {
        speed = TASK2_LINE_LOST_SPEED_TICKS;
    }
    basePwm = (int16_t)(TASK2_PWM_STATIC +
        speed * TASK2_PWM_PER_SPEED_TICK);
    correctionPwm = (int16_t)(((int32_t)gTask2.line.filteredErrorX4 *
        TASK2_PWM_ERROR_GAIN) / 4);
    correctionPwm = clamp_i16(correctionPwm,
        TASK2_PWM_CORRECTION_LIMIT);
    gTask2.line.correctionPwm = correctionPwm;
    motion_control_set_pwm_targets((int16_t)(basePwm - correctionPwm),
        (int16_t)(basePwm + correctionPwm));
}

static uint8_t filter_line_mask(uint8_t sample)
{
    uint8_t filtered = (uint8_t)((sample & gTask2.lineMaskHistory1) |
        (sample & gTask2.lineMaskHistory2) |
        (gTask2.lineMaskHistory1 & gTask2.lineMaskHistory2));

    gTask2.lineMaskHistory2 = gTask2.lineMaskHistory1;
    gTask2.lineMaskHistory1 = sample;
    return filtered;
}

static void fail(CompetitionTaskFault fault)
{
    motion_control_enable(false);
    motor_safe_stop();
    gTask2.currentSpeedTicks = 0;
    gTask2.fault = fault;
    gTask2.phase = TASK2_PHASE_FAULT;
    gTask2.phaseMs = 0U;
}

static void begin_braking(void)
{
    motion_control_enable(false);
    motor_active_brake();
    gTask2.currentSpeedTicks = 0;
    gTask2.phase = TASK2_PHASE_BRAKING;
    gTask2.phaseMs = 0U;
}

static void update_speed(void)
{
    int16_t target = (gTask2.phase == TASK2_PHASE_PASS_FINISH) ?
        TASK2_FINISH_SPEED_TICKS : TASK2_CRUISE_SPEED_TICKS;
    uint16_t stepMs = (gTask2.currentSpeedTicks > target) ?
        TASK2_DECEL_STEP_MS : TASK2_ACCEL_STEP_MS;

    if (gTask2.currentSpeedTicks == target) {
        gTask2.speedRampMs = 0U;
        return;
    }
    if (gTask2.speedRampMs < stepMs) {
        gTask2.speedRampMs++;
        return;
    }
    gTask2.speedRampMs = 0U;
    gTask2.currentSpeedTicks +=
        (gTask2.currentSpeedTicks < target) ? 1 : -1;
}

static bool update_finish_marker(uint8_t lineMask)
{
    bool marker = (track_active_count(lineMask) >=
        TASK2_MARKER_MIN_ACTIVE) &&
        ((lineMask & TASK2_LINE_CENTER_MASK) != 0U) &&
        ((lineMask & TASK2_LINE_OUTER_MASK) != 0U);

    if (!gTask2.markerArmed) {
        if (!marker) {
            if (gTask2.markerClearMs < TASK2_MARKER_CLEAR_MS) {
                gTask2.markerClearMs++;
            }
            if ((gTask2.markerClearMs >= TASK2_MARKER_CLEAR_MS) &&
                (gTask2.phaseMs >= TASK2_MARKER_ARM_MS)) {
                gTask2.markerArmed = true;
            }
        } else {
            gTask2.markerClearMs = 0U;
        }
        return false;
    }
    if (marker && (gTask2.phaseMs >= TASK2_MARKER_MIN_TIME_MS)) {
        if (gTask2.markerDebounceMs < TASK2_MARKER_DEBOUNCE_MS) {
            gTask2.markerDebounceMs++;
        }
    } else {
        gTask2.markerDebounceMs = 0U;
    }
    if (gTask2.markerDebounceMs >= TASK2_MARKER_DEBOUNCE_MS) {
        gTask2.markerArmed = false;
        return true;
    }
    return false;
}

static void task2_init(void)
{
    gTask2.phase = TASK2_PHASE_STOPPED;
    gTask2.fault = COMPETITION_FAULT_NONE;
    gTask2.resultMs = 0U;
    gTask2.distanceMm = 0;
    gTask2.currentSpeedTicks = 0;
    line_reset();
}

static bool task2_start(uint32_t nowMs, uint8_t lineMask)
{
    motor_safe_stop();
    if (lineMask == 0U) {
        gTask2.fault = COMPETITION_FAULT_START_CONDITION;
        gTask2.phase = TASK2_PHASE_FAULT;
        return false;
    }
    gTask2.fault = COMPETITION_FAULT_NONE;
    gTask2.markerArmed = false;
    gTask2.phaseMs = 0U;
    gTask2.lineLostMs = 0U;
    gTask2.markerClearMs = 0U;
    gTask2.markerDebounceMs = 0U;
    gTask2.speedRampMs = TASK2_ACCEL_STEP_MS;
    gTask2.lineMaskHistory1 = lineMask;
    gTask2.lineMaskHistory2 = lineMask;
    gTask2.startMs = nowMs;
    gTask2.resultMs = 0U;
    gTask2.distanceMm = 0;
    gTask2.finishPassCounts = 0U;
    gTask2.currentSpeedTicks = 0;
    line_reset();
    motion_control_reset();
    motion_control_enable(true);
    gTask2.startCount = motion_control_get_average_count();
    gTask2.passStartCount = gTask2.startCount;
    gTask2.phase = TASK2_PHASE_RUNNING;
    return true;
}

static void task2_stop(void)
{
    motion_control_enable(false);
    motor_safe_stop();
    gTask2.currentSpeedTicks = 0;
    gTask2.phase = TASK2_PHASE_STOPPED;
}

static void task2_update_1ms(uint32_t nowMs, uint8_t lineMask)
{
    int32_t averageCount;

    if ((gTask2.phase == TASK2_PHASE_STOPPED) ||
        (gTask2.phase == TASK2_PHASE_COMPLETE) ||
        (gTask2.phase == TASK2_PHASE_FAULT)) {
        return;
    }
    gTask2.phaseMs++;
    if (gTask2.phase == TASK2_PHASE_BRAKING) {
        if (gTask2.phaseMs >= TASK2_ACTIVE_BRAKE_MS) {
            motor_safe_stop();
            gTask2.resultMs = nowMs - gTask2.startMs;
            gTask2.phase = TASK2_PHASE_COMPLETE;
        }
        return;
    }

    averageCount = motion_control_get_average_count();
    gTask2.distanceMm = chassis_counts_to_um(abs_i32(averageCount -
        gTask2.startCount)) / 1000;
    lineMask = filter_line_mask(lineMask);
    line_update_1ms(lineMask);
    if (lineMask == 0U) {
        if (gTask2.lineLostMs < TASK2_LINE_LOST_TIMEOUT_MS) {
            gTask2.lineLostMs++;
        }
        if (gTask2.lineLostMs >= TASK2_LINE_LOST_TIMEOUT_MS) {
            fail(COMPETITION_FAULT_LINE_LOST);
            return;
        }
    } else {
        gTask2.lineLostMs = 0U;
    }
    if ((nowMs - gTask2.startMs) >= TASK2_TIMEOUT_MS) {
        fail(COMPETITION_FAULT_TIMEOUT);
        return;
    }

    update_speed();
    line_command_pwm();
    motion_control_update_1ms();
    if (motion_control_get_fault() != MOTION_FAULT_NONE) {
        fail(COMPETITION_FAULT_MOTION);
        return;
    }

    if (gTask2.phase == TASK2_PHASE_PASS_FINISH) {
        uint32_t passedCounts = (uint32_t)abs_i32(averageCount -
            gTask2.passStartCount);

        if (passedCounts >= gTask2.finishPassCounts) {
            begin_braking();
        }
        return;
    }
    if (update_finish_marker(lineMask)) {
        uint32_t lapCounts = (uint32_t)abs_i32(averageCount -
            gTask2.startCount);

        gTask2.finishPassCounts = (uint32_t)(((uint64_t)lapCounts *
            TASK2_FINISH_PASS_MM + TASK2_ROUTE_LAP_MM / 2) /
            TASK2_ROUTE_LAP_MM);
        gTask2.passStartCount = averageCount;
        gTask2.phase = TASK2_PHASE_PASS_FINISH;
        gTask2.phaseMs = 0U;
    } else if (gTask2.phaseMs > TASK2_MARKER_MAX_TIME_MS) {
        fail(COMPETITION_FAULT_FINISH_MARKER);
    }
}

static CompetitionTaskState task2_get_state(void)
{
    if (gTask2.phase == TASK2_PHASE_STOPPED) {
        return COMPETITION_TASK_STOPPED;
    }
    if (gTask2.phase == TASK2_PHASE_COMPLETE) {
        return COMPETITION_TASK_COMPLETE;
    }
    if (gTask2.phase == TASK2_PHASE_FAULT) {
        return COMPETITION_TASK_FAULT;
    }
    return COMPETITION_TASK_RUNNING;
}

static CompetitionTaskFault task2_get_fault(void)
{
    return gTask2.fault;
}

static uint32_t task2_get_result_ms(void)
{
    return gTask2.resultMs;
}

static uint32_t task2_get_elapsed_ms(uint32_t nowMs)
{
    if (gTask2.resultMs != 0U) {
        return gTask2.resultMs;
    }
    return (gTask2.phase == TASK2_PHASE_STOPPED) ? 0U :
        nowMs - gTask2.startMs;
}

static int32_t task2_get_distance_mm(void)
{
    return gTask2.distanceMm;
}

static int16_t task2_get_speed_command(void)
{
    return gTask2.currentSpeedTicks;
}

static int16_t task2_get_line_error(void)
{
    return gTask2.line.error;
}

static int16_t task2_get_line_correction(void)
{
    return gTask2.line.correctionPwm;
}

static const CompetitionTaskOps gTask2Ops = {
    2U,
    task2_init,
    task2_start,
    task2_update_1ms,
    task2_stop,
    task2_get_state,
    task2_get_fault,
    task2_get_result_ms,
    task2_get_elapsed_ms,
    task2_get_distance_mm,
    task2_get_speed_command,
    task2_get_line_error,
    task2_get_line_correction
};

const CompetitionTaskOps *task2_get_ops(void)
{
    return &gTask2Ops;
}
