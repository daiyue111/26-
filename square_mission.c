#include "square_mission.h"

#include "app_config.h"
#include "chassis_model.h"
#include "imu.h"
#include "line_speed_control.h"
#include "motion_control.h"
#include "pid.h"
#include "track.h"

typedef struct {
    SquareMissionState state;
    uint8_t cornerCount;
    uint8_t faultCode;
    uint16_t stateMs;
    uint16_t cornerClearMs;
    uint16_t cornerDebounceMs;
    uint16_t cornerLostDebounceMs;
    uint16_t turnLineClearMs;
    uint16_t turnLineDebounceMs;
    uint16_t reacquireDebounceMs;
    uint16_t lineLostMs;
    uint16_t settleMs;
    uint8_t imuPeriodMs;
    uint8_t distancePeriodMs;
    int16_t forwardSpeedCommand;
    int16_t turnSpeedCommand;
    int32_t approachStartCount;
    int32_t straightStartCount;
    int32_t cornerLostStartCount;
    int32_t distanceError;
    int32_t headingTargetMdeg;
    int32_t headingErrorMdeg;
    bool cornerArmed;
    bool turnLineArmed;
    LineSpeedController lineControl;
    PidController distancePid;
    PidController headingPid;
} SquareMission;

static SquareMission gSquare;

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
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

static bool is_corner_pattern(uint8_t blackMask)
{
    return ((blackMask & SQUARE_CORNER_SIDE_X1_MASK) != 0U) &&
        ((blackMask & SQUARE_CORNER_SIDE_X8_MASK) != 0U);
}

static int32_t turn_heading_mdeg(void)
{
    int32_t magnitude = abs_i32(imu_heading_get_mdeg());

    return (SQUARE_TURN_DIRECTION < 0) ? -magnitude : magnitude;
}

static void set_state(SquareMissionState state)
{
    gSquare.state = state;
    gSquare.stateMs = 0U;
    gSquare.settleMs = 0U;
}

static bool update_heading(void)
{
    gSquare.imuPeriodMs++;
    if (gSquare.imuPeriodMs < IMU_HEADING_PERIOD_MS) {
        return true;
    }
    gSquare.imuPeriodMs = 0U;
    return imu_heading_update(IMU_HEADING_PERIOD_MS);
}

static void begin_corner_approach(int32_t startCount)
{
    gSquare.approachStartCount = startCount;
    gSquare.cornerDebounceMs = 0U;
    gSquare.cornerLostDebounceMs = 0U;
    gSquare.distancePeriodMs = 0U;
    gSquare.forwardSpeedCommand = SQUARE_APPROACH_MAX_SPEED_TICKS;
    pid_reset(&gSquare.distancePid);
    set_state(SQUARE_STATE_APPROACH_STOP);
}

static void run_straight(uint8_t blackMask)
{
    bool cornerPattern = is_corner_pattern(blackMask);
    int32_t averageCount = motion_control_get_average_count();
    bool cornerDistanceReady = abs_i32(averageCount -
        gSquare.straightStartCount) >= chassis_mm_to_counts(
            SQUARE_CORNER_MIN_TRAVEL_MM);

    if (blackMask == 0U) {
        if (gSquare.lineLostMs < SQUARE_LINE_LOST_TIMEOUT_MS) {
            gSquare.lineLostMs++;
        }
        if (gSquare.lineLostMs >= SQUARE_LINE_LOST_TIMEOUT_MS) {
            gSquare.faultCode = 5U;
            motion_control_enable(false);
            set_state(SQUARE_STATE_FAULT);
            return;
        }
    } else {
        gSquare.lineLostMs = 0U;
    }

    line_speed_control_update_1ms(&gSquare.lineControl, blackMask);
    if ((gSquare.forwardSpeedCommand < SQUARE_CRUISE_SPEED_TICKS) &&
        ((gSquare.stateMs % SQUARE_SPEED_RAMP_STEP_MS) == 0U)) {
        gSquare.forwardSpeedCommand++;
    }
    line_speed_control_command(&gSquare.lineControl,
        gSquare.forwardSpeedCommand);

    if (!gSquare.cornerArmed) {
        if (!cornerPattern) {
            if (gSquare.cornerClearMs < SQUARE_CORNER_CLEAR_MS) {
                gSquare.cornerClearMs++;
            }
            if (gSquare.cornerClearMs >= SQUARE_CORNER_CLEAR_MS) {
                gSquare.cornerArmed = true;
            }
        } else {
            gSquare.cornerClearMs = 0U;
        }
        gSquare.cornerDebounceMs = 0U;
    } else if (cornerDistanceReady &&
        cornerPattern) {
        if (gSquare.cornerDebounceMs < SQUARE_CORNER_DEBOUNCE_MS) {
            gSquare.cornerDebounceMs++;
        }
    } else {
        gSquare.cornerDebounceMs = 0U;
    }

    if (gSquare.cornerArmed && cornerDistanceReady &&
        (blackMask == 0U)) {
        if (gSquare.cornerLostDebounceMs == 0U) {
            gSquare.cornerLostStartCount = averageCount;
        }
        if (gSquare.cornerLostDebounceMs <
            SQUARE_CORNER_LOST_DEBOUNCE_MS) {
            gSquare.cornerLostDebounceMs++;
        }
    } else {
        gSquare.cornerLostDebounceMs = 0U;
    }

    if (gSquare.cornerLostDebounceMs >=
        SQUARE_CORNER_LOST_DEBOUNCE_MS) {
        begin_corner_approach(gSquare.cornerLostStartCount);
        return;
    }
}

static void run_approach_stop(uint8_t blackMask)
{
    bool positionReached;

    line_speed_control_update_1ms(&gSquare.lineControl, blackMask);
    gSquare.distancePeriodMs++;
    if (gSquare.distancePeriodMs >= SPEED_CONTROL_PERIOD_MS) {
        int32_t distance = motion_control_get_average_count() -
            gSquare.approachStartCount;
        int32_t forwardSpeed;

        gSquare.distancePeriodMs = 0U;
        gSquare.distanceError = chassis_mm_to_counts(
            (int32_t)chassis_model_get_corner_center_offset_mm()) -
            distance;
        forwardSpeed = pid_step_error(&gSquare.distancePid,
            gSquare.distanceError);
        gSquare.forwardSpeedCommand = (int16_t)clamp_i32(forwardSpeed,
            SQUARE_APPROACH_MAX_SPEED_TICKS);
    }

    positionReached = abs_i32(gSquare.distanceError) <=
        SQUARE_DISTANCE_TOLERANCE_TICKS;
    if (positionReached) {
        motion_control_set_speed_targets(0, 0);
    } else {
        line_speed_control_command(&gSquare.lineControl,
            gSquare.forwardSpeedCommand);
    }

    if (positionReached &&
        (abs_i32(motion_control_get_left_speed()) <=
            SQUARE_STOP_SPEED_TOLERANCE) &&
        (abs_i32(motion_control_get_right_speed()) <=
            SQUARE_STOP_SPEED_TOLERANCE)) {
        gSquare.settleMs++;
    } else {
        gSquare.settleMs = 0U;
    }

    if (gSquare.settleMs >= SQUARE_STOP_SETTLE_MS) {
        motion_control_set_speed_targets(0, 0);
        set_state(SQUARE_STATE_PRE_TURN);
    }
}

static void run_pre_turn(void)
{
    motion_control_set_speed_targets(0, 0);
    if (gSquare.stateMs >= SQUARE_PRE_TURN_PAUSE_MS) {
        imu_heading_reset();
        pid_reset(&gSquare.headingPid);
        gSquare.headingTargetMdeg =
            SQUARE_TURN_DIRECTION * SQUARE_TURN_ANGLE_MDEG;
        gSquare.headingErrorMdeg = gSquare.headingTargetMdeg;
        gSquare.imuPeriodMs = 0U;
        gSquare.turnSpeedCommand = 0;
        gSquare.turnLineClearMs = 0U;
        gSquare.turnLineDebounceMs = 0U;
        gSquare.turnLineArmed = false;
        set_state(SQUARE_STATE_TURN);
    }
}

static void complete_corner(void)
{
    gSquare.cornerCount++;
    if (!SQUARE_CONTINUOUS_RUN &&
        (gSquare.cornerCount >= SQUARE_TOTAL_CORNERS)) {
        motion_control_enable(false);
        set_state(SQUARE_STATE_COMPLETE);
    } else {
        gSquare.forwardSpeedCommand = SQUARE_CRUISE_SPEED_TICKS;
        gSquare.cornerClearMs = 0U;
        gSquare.cornerDebounceMs = 0U;
        gSquare.cornerLostDebounceMs = 0U;
        gSquare.lineLostMs = 0U;
        gSquare.cornerArmed = false;
        gSquare.straightStartCount = motion_control_get_average_count();
        gSquare.cornerLostStartCount = gSquare.straightStartCount;
        line_speed_control_reset(&gSquare.lineControl);
        set_state(SQUARE_STATE_STRAIGHT);
    }
}

static void run_turn(uint8_t blackMask)
{
    int32_t headingMagnitude = abs_i32(turn_heading_mdeg());

    if (!gSquare.turnLineArmed) {
        if (blackMask == 0U) {
            if (gSquare.turnLineClearMs < SQUARE_TURN_LINE_CLEAR_MS) {
                gSquare.turnLineClearMs++;
            }
        } else {
            gSquare.turnLineClearMs = 0U;
        }
        if (gSquare.turnLineClearMs >= SQUARE_TURN_LINE_CLEAR_MS) {
            gSquare.turnLineArmed = true;
        }
    } else if ((headingMagnitude >= SQUARE_TURN_LINE_MIN_ANGLE_MDEG) &&
        ((blackMask & LINE_CENTER_MASK) != 0U)) {
        if (gSquare.turnLineDebounceMs <
            SQUARE_TURN_LINE_DEBOUNCE_MS) {
            gSquare.turnLineDebounceMs++;
        }
    } else {
        gSquare.turnLineDebounceMs = 0U;
    }

    if (gSquare.turnLineDebounceMs >=
        SQUARE_TURN_LINE_DEBOUNCE_MS) {
        line_speed_control_reset(&gSquare.lineControl);
        complete_corner();
        return;
    }

    if (gSquare.imuPeriodMs == 0U) {
        int32_t turnSpeed;

        gSquare.headingErrorMdeg = gSquare.headingTargetMdeg -
            turn_heading_mdeg();
        turnSpeed = pid_step_error(&gSquare.headingPid,
            gSquare.headingErrorMdeg);
        turnSpeed = clamp_i32(turnSpeed, SQUARE_TURN_MAX_SPEED_TICKS);

        if ((abs_i32(gSquare.headingErrorMdeg) >
                SQUARE_HEADING_TOLERANCE_MDEG) &&
            (abs_i32(turnSpeed) < SQUARE_TURN_MIN_SPEED_TICKS)) {
            turnSpeed = (gSquare.headingErrorMdeg < 0) ?
                -SQUARE_TURN_MIN_SPEED_TICKS : SQUARE_TURN_MIN_SPEED_TICKS;
        }
        gSquare.turnSpeedCommand = (int16_t)turnSpeed;
    }

    if (abs_i32(gSquare.headingErrorMdeg) <=
        SQUARE_HEADING_TOLERANCE_MDEG) {
        motion_control_set_speed_targets(0, 0);
        if ((abs_i32(motion_control_get_left_speed()) <=
                SQUARE_STOP_SPEED_TOLERANCE) &&
            (abs_i32(motion_control_get_right_speed()) <=
                SQUARE_STOP_SPEED_TOLERANCE)) {
            gSquare.settleMs++;
        } else {
            gSquare.settleMs = 0U;
        }
    } else {
        motion_control_set_speed_targets((int16_t)-gSquare.turnSpeedCommand,
            gSquare.turnSpeedCommand);
        gSquare.settleMs = 0U;
    }

    if (gSquare.settleMs >= SQUARE_TURN_SETTLE_MS) {
        motion_control_set_speed_targets(0, 0);
        gSquare.reacquireDebounceMs = 0U;
        gSquare.turnSpeedCommand = 0;
        pid_reset(&gSquare.headingPid);
        line_speed_control_reset(&gSquare.lineControl);
        set_state(SQUARE_STATE_REACQUIRE_LINE);
    } else if (gSquare.stateMs >= SQUARE_TURN_TIMEOUT_MS) {
        gSquare.faultCode = 6U;
        motion_control_enable(false);
        set_state(SQUARE_STATE_FAULT);
    }
}

static void run_reacquire_line(uint8_t blackMask)
{
    if ((blackMask & LINE_CENTER_MASK) != 0U) {
        if (gSquare.reacquireDebounceMs <
            SQUARE_REACQUIRE_DEBOUNCE_MS) {
            gSquare.reacquireDebounceMs++;
        }
    } else {
        gSquare.reacquireDebounceMs = 0U;
    }

    if (gSquare.reacquireDebounceMs >=
        SQUARE_REACQUIRE_DEBOUNCE_MS) {
        complete_corner();
        return;
    }

    if (gSquare.stateMs >= SQUARE_REACQUIRE_TIMEOUT_MS) {
        gSquare.faultCode = 7U;
        motion_control_enable(false);
        set_state(SQUARE_STATE_FAULT);
        return;
    }

    /* Once any sensor reaches the outgoing line, let the line PID pull
     * it to the center instead of continuing the open-loop scan. */
    if (blackMask != 0U) {
        line_speed_control_update_1ms(&gSquare.lineControl, blackMask);
        line_speed_control_command(&gSquare.lineControl,
            SQUARE_REACQUIRE_FORWARD_SPEED_TICKS);
        return;
    }

    if (gSquare.imuPeriodMs == 0U) {
        int32_t scanTarget = gSquare.headingTargetMdeg;
        int32_t turnSpeed;

        if (gSquare.stateMs >= SQUARE_REACQUIRE_SCAN_DELAY_MS) {
            uint16_t scanMs = (uint16_t)(gSquare.stateMs -
                SQUARE_REACQUIRE_SCAN_DELAY_MS);
            int32_t offset = SQUARE_REACQUIRE_SCAN_ANGLE_MDEG;

            if (((scanMs / SQUARE_REACQUIRE_SCAN_HALF_MS) & 1U) !=
                0U) {
                offset = -offset;
            }
            scanTarget += SQUARE_TURN_DIRECTION * offset;
        }

        gSquare.headingErrorMdeg = scanTarget - turn_heading_mdeg();
        turnSpeed = pid_step_error(&gSquare.headingPid,
            gSquare.headingErrorMdeg);
        turnSpeed = clamp_i32(turnSpeed,
            SQUARE_REACQUIRE_MAX_SPEED_TICKS);
        if ((abs_i32(gSquare.headingErrorMdeg) >
                SQUARE_HEADING_TOLERANCE_MDEG) &&
            (abs_i32(turnSpeed) < SQUARE_REACQUIRE_MIN_SPEED_TICKS)) {
            turnSpeed = (gSquare.headingErrorMdeg < 0) ?
                -SQUARE_REACQUIRE_MIN_SPEED_TICKS :
                SQUARE_REACQUIRE_MIN_SPEED_TICKS;
        }
        gSquare.turnSpeedCommand = (int16_t)turnSpeed;
    }

    motion_control_set_speed_targets(
        (int16_t)(SQUARE_REACQUIRE_FORWARD_SPEED_TICKS -
            gSquare.turnSpeedCommand),
        (int16_t)(SQUARE_REACQUIRE_FORWARD_SPEED_TICKS +
            gSquare.turnSpeedCommand));
}

void square_mission_init(void)
{
    line_speed_control_init(&gSquare.lineControl);
    pid_init(&gSquare.distancePid, SQUARE_DISTANCE_PID_KP,
        SQUARE_DISTANCE_PID_KI, SQUARE_DISTANCE_PID_KD,
        SQUARE_DISTANCE_PID_SCALE, 2000, SQUARE_CRUISE_SPEED_TICKS);
    pid_init(&gSquare.headingPid, SQUARE_HEADING_PID_KP,
        SQUARE_HEADING_PID_KI, SQUARE_HEADING_PID_KD,
        SQUARE_HEADING_PID_SCALE, 200000,
        SQUARE_TURN_MAX_SPEED_TICKS);
    gSquare.state = SQUARE_STATE_STOPPED;
}

bool square_mission_start(bool imuReady)
{
    if (!imuReady) {
        set_state(SQUARE_STATE_FAULT);
        return false;
    }

    motion_control_reset();
    motion_control_enable(true);
    imu_heading_reset();
    line_speed_control_reset(&gSquare.lineControl);
    pid_reset(&gSquare.distancePid);
    pid_reset(&gSquare.headingPid);
    gSquare.cornerCount = 0U;
    gSquare.faultCode = 0U;
    gSquare.cornerClearMs = 0U;
    gSquare.cornerDebounceMs = 0U;
    gSquare.cornerLostDebounceMs = 0U;
    gSquare.turnLineClearMs = 0U;
    gSquare.turnLineDebounceMs = 0U;
    gSquare.reacquireDebounceMs = 0U;
    gSquare.lineLostMs = 0U;
    gSquare.imuPeriodMs = 0U;
    gSquare.distancePeriodMs = 0U;
    gSquare.forwardSpeedCommand = SQUARE_CRUISE_SPEED_TICKS;
    gSquare.turnSpeedCommand = 0;
    gSquare.distanceError = 0;
    gSquare.headingErrorMdeg = 0;
    gSquare.straightStartCount = motion_control_get_average_count();
    gSquare.cornerLostStartCount = gSquare.straightStartCount;
    gSquare.cornerArmed = false;
    gSquare.turnLineArmed = false;
    set_state(SQUARE_STATE_STRAIGHT);
    return true;
}

void square_mission_stop(void)
{
    motion_control_enable(false);
    set_state(SQUARE_STATE_STOPPED);
}

void square_mission_update_1ms(uint8_t blackMask)
{
    if ((gSquare.state == SQUARE_STATE_STOPPED) ||
        (gSquare.state == SQUARE_STATE_COMPLETE) ||
        (gSquare.state == SQUARE_STATE_FAULT)) {
        return;
    }

    gSquare.stateMs++;
    if (!update_heading()) {
        gSquare.faultCode = 8U;
        motion_control_enable(false);
        set_state(SQUARE_STATE_FAULT);
        return;
    }

    switch (gSquare.state) {
        case SQUARE_STATE_STRAIGHT:
            run_straight(blackMask);
            break;
        case SQUARE_STATE_APPROACH_STOP:
            run_approach_stop(blackMask);
            break;
        case SQUARE_STATE_PRE_TURN:
            run_pre_turn();
            break;
        case SQUARE_STATE_TURN:
            run_turn(blackMask);
            break;
        case SQUARE_STATE_REACQUIRE_LINE:
            run_reacquire_line(blackMask);
            break;
        default:
            break;
    }

    motion_control_update_1ms();
    if (motion_control_get_fault() != MOTION_FAULT_NONE) {
        motion_control_enable(false);
        set_state(SQUARE_STATE_FAULT);
    }
}

SquareMissionState square_mission_get_state(void)
{
    return gSquare.state;
}

uint8_t square_mission_get_corner_count(void)
{
    return gSquare.cornerCount;
}

int16_t square_mission_get_line_error(void)
{
    return gSquare.lineControl.error;
}

int16_t square_mission_get_line_correction(void)
{
    return gSquare.lineControl.correction;
}

int32_t square_mission_get_distance_error(void)
{
    return gSquare.distanceError;
}

int32_t square_mission_get_heading_error(void)
{
    return gSquare.headingErrorMdeg;
}

uint8_t square_mission_get_fault_code(void)
{
    return gSquare.faultCode;
}
