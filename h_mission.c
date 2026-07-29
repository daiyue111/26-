#include "h_mission.h"

#include "app_config.h"
#include "ball_control.h"
#include "ball_vision.h"
#include "chassis_model.h"
#include "h_line_control.h"
#include "motion_control.h"
#include "motor.h"
#include "track.h"

typedef struct {
    HMissionState state;
    HMissionFault fault;
    uint8_t task;
    bool markerArmed;
    uint16_t stateMs;
    uint16_t lineLostMs;
    uint16_t markerClearMs;
    uint16_t markerDebounceMs;
    uint16_t ballFaultMs;
    uint16_t settleMs;
    uint16_t speedRampMs;
    uint8_t lineMaskHistory1;
    uint8_t lineMaskHistory2;
    uint32_t startMs;
    uint32_t resultMs;
    int32_t startCount;
    int32_t passStartCount;
    int32_t distanceMm;
    int16_t arbitraryTargetTenthMm;
    int16_t currentSpeedTicks;
    int16_t targetSpeedTicks;
    int32_t accelerationMmps2;
    HLineControl line;
} HMission;

static HMission gH;

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool task_requires_ball(uint8_t task)
{
    return task >= H_TASK_STATIC_BALL;
}

static bool task_is_valid(uint8_t task)
{
    return (task >= H_TASK_CAR_LAP_STOP) &&
        (task <= H_TASK_LAP_ARBITRARY_BALL);
}

static uint32_t task_timeout_ms(void)
{
    if (gH.task == H_TASK_CAR_LAP_STOP) {
#if H_TEMP_TRACK_TUNING_MODE
        return H_TUNING_TIMEOUT_MS;
#else
        return H_TASK2_TIMEOUT_MS;
#endif
    }
    if (gH.task == H_TASK_STATIC_BALL) {
        return H_TASK3_TIMEOUT_MS;
    }
    if (gH.task == H_TASK_AB_CENTER_BALL) {
        return H_TASK4_TIMEOUT_MS;
    }
    return H_TASK56_TIMEOUT_MS;
}

static void set_state(HMissionState state)
{
    gH.state = state;
    gH.stateMs = 0U;
    gH.settleMs = 0U;
}

static void fail_mission(HMissionFault fault)
{
    motion_control_enable(false);
    motor_safe_stop();
    gH.fault = fault;
    gH.accelerationMmps2 = 0;
    set_state(H_STATE_FAULT);
}

static void begin_braking(void)
{
    motion_control_enable(false);
    motor_active_brake();
    gH.currentSpeedTicks = 0;
    gH.targetSpeedTicks = 0;
    gH.accelerationMmps2 = 0;
    set_state(H_STATE_BRAKING);
}

static uint16_t speed_ramp_step_ms(void)
{
    return (gH.task == H_TASK_CAR_LAP_STOP) ? H_FAST_RAMP_STEP_MS :
        H_STABLE_RAMP_STEP_MS;
}

static uint8_t filter_line_mask(uint8_t sample)
{
    uint8_t filtered = (uint8_t)((sample & gH.lineMaskHistory1) |
        (sample & gH.lineMaskHistory2) |
        (gH.lineMaskHistory1 & gH.lineMaskHistory2));

    gH.lineMaskHistory2 = gH.lineMaskHistory1;
    gH.lineMaskHistory1 = sample;
    return filtered;
}

static int16_t route_speed_target(void)
{
#if H_TEMP_TRACK_TUNING_MODE
    int16_t lineError = gH.line.error;
    int32_t firstCurveEnd = H_ROUTE_AB_MM + H_ROUTE_HALF_CIRCLE_MM;
    bool routeCurve =
        ((gH.distanceMm >= (H_ROUTE_AB_MM -
                H_TUNING_CURVE_APPROACH_MM)) &&
         (gH.distanceMm < (firstCurveEnd +
                H_TUNING_CURVE_EXIT_MARGIN_MM))) ||
        (gH.distanceMm >= (H_ROUTE_CD_END_MM -
                H_TUNING_CURVE_APPROACH_MM));

    if (gH.state == H_STATE_PASS_FINISH) {
        return H_FINISH_APPROACH_SPEED_TICKS;
    }
    if (!gH.line.lineVisible) {
        return H_LINE_LOST_SPEED_TICKS;
    }
    if (routeCurve) {
        return H_TUNING_CURVE_SPEED_TICKS;
    }
    if (lineError < 0) {
        lineError = (int16_t)-lineError;
    }
    if (lineError >= H_TUNING_LARGE_ERROR) {
        return H_TUNING_CURVE_SPEED_TICKS;
    }
    if (lineError >= H_TUNING_MEDIUM_ERROR) {
        return H_TUNING_MEDIUM_SPEED_TICKS;
    }
    return H_TUNING_STRAIGHT_SPEED_TICKS;
#else
    bool fast = gH.task == H_TASK_CAR_LAP_STOP;
    int16_t straight = fast ? H_FAST_STRAIGHT_SPEED_TICKS :
        H_STABLE_STRAIGHT_SPEED_TICKS;
    int16_t curve = fast ? H_FAST_CURVE_SPEED_TICKS :
        H_STABLE_CURVE_SPEED_TICKS;
    int32_t firstCurveStart = H_ROUTE_AB_MM;
    int32_t firstCurveEnd = H_ROUTE_AB_MM + H_ROUTE_HALF_CIRCLE_MM;
    int32_t secondCurveStart = H_ROUTE_CD_END_MM;

    if (gH.state == H_STATE_PASS_FINISH) {
        return H_FINISH_APPROACH_SPEED_TICKS;
    }
    if (gH.task == H_TASK_AB_CENTER_BALL) {
        return straight;
    }
    if (gH.distanceMm >= H_FINISH_DECEL_START_MM) {
        return H_FINISH_APPROACH_SPEED_TICKS;
    }
    if (((gH.distanceMm >= (firstCurveStart - H_CURVE_APPROACH_MM)) &&
            (gH.distanceMm < firstCurveEnd)) ||
        (gH.distanceMm >= (secondCurveStart - H_CURVE_APPROACH_MM))) {
        return curve;
    }
    return straight;
#endif
}

static int16_t route_steering_feedforward(void)
{
#if H_TEMP_TRACK_TUNING_MODE
    return 0;
#else
    int32_t firstCurveEnd = H_ROUTE_AB_MM + H_ROUTE_HALF_CIRCLE_MM;
    int16_t feedforward;

    if ((gH.task == H_TASK_AB_CENTER_BALL) ||
        (gH.state == H_STATE_PASS_FINISH)) {
        return 0;
    }
    if (((gH.distanceMm >= (H_ROUTE_AB_MM - H_CURVE_STEER_LEAD_MM)) &&
            (gH.distanceMm < firstCurveEnd)) ||
        ((gH.distanceMm >= (H_ROUTE_CD_END_MM -
                H_CURVE_STEER_LEAD_MM)) &&
            (gH.distanceMm < H_ROUTE_LAP_MM))) {
        feedforward = H_CURVE_STEERING_FEEDFORWARD_TICKS;
        if (((gH.stateMs / SPEED_CONTROL_PERIOD_MS) & 1U) != 0U) {
            feedforward += H_CURVE_STEERING_EXTRA_TICKS;
        }
        return feedforward;
    }
    return 0;
#endif
}

static void update_speed_ramp(void)
{
    uint16_t rampMs;
    int16_t oldSpeed;
    int32_t oldMmps;
    int32_t newMmps;

    gH.targetSpeedTicks = route_speed_target();
    rampMs = speed_ramp_step_ms();
#if H_TEMP_TRACK_TUNING_MODE
    if (gH.currentSpeedTicks > gH.targetSpeedTicks) {
        rampMs = H_TUNING_DECEL_STEP_MS;
    }
#endif
    if (gH.currentSpeedTicks == gH.targetSpeedTicks) {
        gH.speedRampMs = 0U;
        gH.accelerationMmps2 = 0;
        return;
    }
    if (gH.speedRampMs < rampMs) {
        gH.speedRampMs++;
        return;
    }
    gH.speedRampMs = 0U;
    oldSpeed = gH.currentSpeedTicks;
    if (gH.currentSpeedTicks < gH.targetSpeedTicks) {
        gH.currentSpeedTicks++;
    } else {
        gH.currentSpeedTicks--;
    }
    oldMmps = chassis_speed_ticks_to_mmps(oldSpeed);
    newMmps = chassis_speed_ticks_to_mmps(gH.currentSpeedTicks);
    gH.accelerationMmps2 = ((newMmps - oldMmps) * 1000) / rampMs;
}

static bool update_finish_marker(uint8_t blackMask)
{
    bool marker = track_active_count(blackMask) >=
        H_MARKER_MIN_ACTIVE_SENSORS;
#if H_TEMP_TRACK_TUNING_MODE
    bool markerArmEligible = true;
    bool finishEligible = gH.stateMs >= H_TUNING_MARKER_MIN_TIME_MS;
#else
    bool markerArmEligible = gH.distanceMm >= H_MARKER_ARM_DISTANCE_MM;
    bool finishEligible = gH.distanceMm >= H_MARKER_MIN_LAP_DISTANCE_MM;
#endif

    if (!gH.markerArmed) {
        if (!marker) {
            if (gH.markerClearMs < H_MARKER_CLEAR_MS) {
                gH.markerClearMs++;
            }
            if ((gH.markerClearMs >= H_MARKER_CLEAR_MS) &&
                markerArmEligible) {
                gH.markerArmed = true;
            }
        } else {
            gH.markerClearMs = 0U;
        }
        return false;
    }

    if (finishEligible && marker) {
        if (gH.markerDebounceMs < H_MARKER_DEBOUNCE_MS) {
            gH.markerDebounceMs++;
        }
    } else {
        gH.markerDebounceMs = 0U;
    }
    if (gH.markerDebounceMs >= H_MARKER_DEBOUNCE_MS) {
        gH.markerArmed = false;
        return true;
    }
    return false;
}

static bool ball_is_settled(int16_t toleranceTenthMm)
{
    return (abs_i32(ball_control_get_error_tenth_mm()) <=
        toleranceTenthMm) &&
        (abs_i32(ball_control_get_velocity_tenth_mmps()) <=
        H_BALL_FINAL_SPEED_TENTH_MMPS);
}

static void update_task3(uint32_t nowMs)
{
    ball_control_update_1ms(nowMs, 0);
    if (!ball_control_is_vision_healthy()) {
        if (++gH.ballFaultMs >= H_BALL_FAULT_GRACE_MS) {
            fail_mission(H_FAULT_VISION_LOST);
        }
        return;
    }
    gH.ballFaultMs = 0U;

    if (gH.state == H_STATE_TASK3_TO_POSITIVE) {
        if (ball_is_settled(H_BALL_WAYPOINT_TOL_TENTH_MM)) {
            if (++gH.settleMs >= H_BALL_WAYPOINT_SETTLE_MS) {
                ball_control_set_target_tenth_mm(
                    -H_BALL_WAYPOINT_TENTH_MM);
                set_state(H_STATE_TASK3_TO_NEGATIVE);
            }
        } else {
            gH.settleMs = 0U;
        }
    } else if (gH.state == H_STATE_TASK3_TO_NEGATIVE) {
        if (ball_is_settled(H_BALL_FINAL_TOL_TENTH_MM)) {
            set_state(H_STATE_TASK3_SETTLING);
        }
    } else if (gH.state == H_STATE_TASK3_SETTLING) {
        if (!ball_is_settled(H_BALL_FINAL_TOL_TENTH_MM)) {
            set_state(H_STATE_TASK3_TO_NEGATIVE);
        } else if (++gH.settleMs >= H_BALL_FINAL_SETTLE_MS) {
            gH.resultMs = nowMs - gH.startMs;
            set_state(H_STATE_COMPLETE);
        }
    }
}

static void update_running_safety(uint32_t nowMs, uint8_t blackMask)
{
    if (blackMask == 0U) {
        if (gH.lineLostMs < H_LINE_LOST_TIMEOUT_MS) {
            gH.lineLostMs++;
        }
        if (gH.lineLostMs >= H_LINE_LOST_TIMEOUT_MS) {
            fail_mission(H_FAULT_LINE_LOST);
            return;
        }
    } else {
        gH.lineLostMs = 0U;
    }

    if (task_requires_ball(gH.task)) {
        if (!ball_control_is_vision_healthy()) {
            if (gH.ballFaultMs < H_BALL_FAULT_GRACE_MS) {
                gH.ballFaultMs++;
            }
            if (gH.ballFaultMs >= H_BALL_FAULT_GRACE_MS) {
                fail_mission(H_FAULT_VISION_LOST);
                return;
            }
        } else {
            gH.ballFaultMs = 0U;
        }
        if (abs_i32(ball_control_get_position_tenth_mm()) >
            H_BALL_POSITION_LIMIT_TENTH_MM) {
            fail_mission(H_FAULT_BALL_LIMIT);
            return;
        }
    }

    if ((nowMs - gH.startMs) >= task_timeout_ms()) {
        fail_mission(H_FAULT_TIMEOUT);
    }
}

void h_mission_init(void)
{
    gH.state = H_STATE_STOPPED;
    gH.fault = H_FAULT_NONE;
    gH.task = H_DEFAULT_TASK;
    gH.arbitraryTargetTenthMm = 0;
    h_line_control_init(&gH.line);
    ball_vision_init();
    ball_control_init();
    ball_control_set_enabled(false);
}

bool h_mission_set_task(uint8_t task)
{
    if (!task_is_valid(task) || ((gH.state != H_STATE_STOPPED) &&
        (gH.state != H_STATE_COMPLETE) && (gH.state != H_STATE_FAULT))) {
        return false;
    }
    gH.task = task;
    if ((task == H_TASK_STATIC_BALL) ||
        (task == H_TASK_AB_CENTER_BALL) ||
        (task == H_TASK_LAP_CENTER_BALL)) {
        ball_control_set_target_tenth_mm(0);
        ball_control_set_enabled(true);
    } else if (task == H_TASK_LAP_ARBITRARY_BALL) {
        ball_control_set_target_tenth_mm(gH.arbitraryTargetTenthMm);
        ball_control_set_enabled(true);
    } else {
        ball_control_set_enabled(false);
    }
    return true;
}

bool h_mission_set_arbitrary_target_tenth_mm(int16_t targetTenthMm)
{
    if ((targetTenthMm < -H_BALL_TARGET_LIMIT_TENTH_MM) ||
        (targetTenthMm > H_BALL_TARGET_LIMIT_TENTH_MM)) {
        return false;
    }
    gH.arbitraryTargetTenthMm = targetTenthMm;
    if (gH.task == H_TASK_LAP_ARBITRARY_BALL) {
        ball_control_set_target_tenth_mm(targetTenthMm);
    }
    return true;
}

bool h_mission_start(uint32_t nowMs, uint8_t blackMask)
{
    if (!task_is_valid(gH.task)) {
        gH.fault = H_FAULT_BAD_TASK;
        set_state(H_STATE_FAULT);
        return false;
    }

    gH.fault = H_FAULT_NONE;
    gH.startMs = nowMs;
    gH.resultMs = 0U;
    gH.stateMs = 0U;
    gH.lineLostMs = 0U;
    gH.markerClearMs = 0U;
    gH.markerDebounceMs = 0U;
    gH.ballFaultMs = 0U;
    gH.settleMs = 0U;
    gH.markerArmed = false;
    gH.distanceMm = 0;
    gH.currentSpeedTicks = 0;
    gH.targetSpeedTicks = 0;
    gH.speedRampMs = speed_ramp_step_ms();
    gH.lineMaskHistory1 = blackMask;
    gH.lineMaskHistory2 = blackMask;
    gH.accelerationMmps2 = 0;
    h_line_control_reset(&gH.line);

    if (gH.task == H_TASK_STATIC_BALL) {
        if (!ball_control_is_vision_healthy()) {
            gH.fault = H_FAULT_VISION_NOT_READY;
            set_state(H_STATE_FAULT);
            return false;
        }
        ball_control_set_enabled(true);
        ball_control_set_target_tenth_mm(H_BALL_WAYPOINT_TENTH_MM);
        set_state(H_STATE_TASK3_TO_POSITIVE);
        return true;
    }

    if (blackMask == 0U) {
        gH.fault = H_FAULT_START_OFF_LINE;
        set_state(H_STATE_FAULT);
        return false;
    }
    if (task_requires_ball(gH.task) &&
        !ball_control_is_vision_healthy()) {
        gH.fault = H_FAULT_VISION_NOT_READY;
        set_state(H_STATE_FAULT);
        return false;
    }

    if ((gH.task == H_TASK_AB_CENTER_BALL) ||
        (gH.task == H_TASK_LAP_CENTER_BALL)) {
        ball_control_set_target_tenth_mm(0);
        ball_control_set_enabled(true);
    } else if (gH.task == H_TASK_LAP_ARBITRARY_BALL) {
        ball_control_set_target_tenth_mm(gH.arbitraryTargetTenthMm);
        ball_control_set_enabled(true);
    } else {
        ball_control_set_target_tenth_mm(0);
        ball_control_set_enabled(false);
    }

    motion_control_reset();
    motion_control_enable(true);
    gH.startCount = motion_control_get_average_count();
    gH.passStartCount = gH.startCount;
    set_state(H_STATE_RUNNING);
    return true;
}

void h_mission_stop(void)
{
    motion_control_enable(false);
    motor_safe_stop();
    gH.currentSpeedTicks = 0;
    gH.targetSpeedTicks = 0;
    gH.accelerationMmps2 = 0;
    set_state(H_STATE_STOPPED);
}

void h_mission_update_1ms(uint32_t nowMs, uint8_t blackMask)
{
    int32_t averageCount;
    bool finishMarker;

    if ((gH.state == H_STATE_STOPPED) ||
        (gH.state == H_STATE_COMPLETE) ||
        (gH.state == H_STATE_FAULT)) {
        ball_control_update_1ms(nowMs, 0);
        return;
    }

    gH.stateMs++;
    if ((gH.state == H_STATE_TASK3_TO_POSITIVE) ||
        (gH.state == H_STATE_TASK3_TO_NEGATIVE) ||
        (gH.state == H_STATE_TASK3_SETTLING)) {
        update_task3(nowMs);
        if (((nowMs - gH.startMs) >= H_TASK3_TIMEOUT_MS) &&
            (gH.state != H_STATE_COMPLETE)) {
            fail_mission(H_FAULT_TIMEOUT);
        }
        return;
    }

    if (gH.state == H_STATE_BRAKING) {
        ball_control_update_1ms(nowMs, 0);
        if (gH.stateMs >= H_ACTIVE_BRAKE_MS) {
            motor_safe_stop();
            if (gH.resultMs == 0U) {
                gH.resultMs = nowMs - gH.startMs;
            }
            set_state(H_STATE_COMPLETE);
        }
        return;
    }

    averageCount = motion_control_get_average_count();
    gH.distanceMm = chassis_counts_to_um(abs_i32(averageCount -
        gH.startCount)) / 1000;
    blackMask = filter_line_mask(blackMask);
    h_line_control_update_1ms(&gH.line, blackMask);
    update_running_safety(nowMs, blackMask);
    if (gH.state == H_STATE_FAULT) {
        ball_control_update_1ms(nowMs, 0);
        return;
    }

    update_speed_ramp();
    ball_control_update_1ms(nowMs, gH.accelerationMmps2);
    h_line_control_command(&gH.line, gH.currentSpeedTicks,
        route_steering_feedforward());
    motion_control_update_1ms();
    if (motion_control_get_fault() != MOTION_FAULT_NONE) {
        fail_mission(H_FAULT_MOTION);
        return;
    }

    if (gH.state == H_STATE_PASS_FINISH) {
        int32_t passedMm = chassis_counts_to_um(abs_i32(averageCount -
            gH.passStartCount)) / 1000;

        if (passedMm >= ((gH.task == H_TASK_AB_CENTER_BALL) ?
            H_TASK4_PASS_B_MM : H_FINISH_PASS_MM)) {
            begin_braking();
        }
        return;
    }

    if ((gH.task == H_TASK_AB_CENTER_BALL) &&
        (gH.distanceMm >= H_ROUTE_AB_MM)) {
        gH.resultMs = nowMs - gH.startMs;
        gH.passStartCount = averageCount;
        set_state(H_STATE_PASS_FINISH);
        return;
    }

#if H_TEMP_TRACK_TUNING_MODE
    finishMarker = H_TUNING_AUTO_FINISH_ENABLE ?
        update_finish_marker(blackMask) : false;
#else
    finishMarker = update_finish_marker(blackMask);
#endif
    if (finishMarker) {
        if (gH.task == H_TASK_CAR_LAP_STOP) {
            begin_braking();
        } else {
            gH.resultMs = nowMs - gH.startMs;
            gH.passStartCount = averageCount;
            set_state(H_STATE_PASS_FINISH);
        }
    }
#if !H_TEMP_TRACK_TUNING_MODE
    else if (gH.distanceMm > (H_ROUTE_LAP_MM + 600)) {
        fail_mission(H_FAULT_FINISH_MARKER);
    }
#endif
}

HMissionState h_mission_get_state(void)
{
    return gH.state;
}

HMissionFault h_mission_get_fault(void)
{
    return gH.fault;
}

uint8_t h_mission_get_task(void)
{
    return gH.task;
}

uint32_t h_mission_get_elapsed_ms(uint32_t nowMs)
{
    if (gH.resultMs != 0U) {
        return gH.resultMs;
    }
    if (gH.state == H_STATE_STOPPED) {
        return 0U;
    }
    return nowMs - gH.startMs;
}

uint32_t h_mission_get_result_ms(void)
{
    return gH.resultMs;
}

int32_t h_mission_get_distance_mm(void)
{
    return gH.distanceMm;
}

int16_t h_mission_get_speed_command_ticks(void)
{
    return gH.currentSpeedTicks;
}

int32_t h_mission_get_acceleration_mmps2(void)
{
    return gH.accelerationMmps2;
}

int16_t h_mission_get_line_error(void)
{
    return gH.line.error;
}

int16_t h_mission_get_line_correction(void)
{
    return gH.line.correction;
}
