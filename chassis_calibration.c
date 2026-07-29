#include "chassis_calibration.h"

#include "app_config.h"
#include "chassis_config.h"
#include "chassis_model.h"
#include "imu.h"
#include "line_speed_control.h"
#include "motion_control.h"
#include "track.h"

#include <stddef.h>

#define PI_SCALED_1E6 3141593LL

typedef struct {
    ChassisCalibrationState state;
    ChassisCalibrationFault fault;
    uint32_t stateMs;
    uint16_t cornerClearMs;
    uint16_t cornerDebounceMs;
    uint16_t turnSettleMs;
    uint8_t imuPeriodMs;
    bool startCornerCleared;
    LineSpeedController lineControl;
    ChassisCalibrationRecord record;
} ChassisCalibration;

static ChassisCalibration gCalibration;

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t divide_round_u64(uint64_t numerator, uint64_t denominator)
{
    if (denominator == 0U) {
        return 0U;
    }
    return (uint32_t)((numerator + denominator / 2U) / denominator);
}

static void stop_motion(void)
{
    motion_control_enable(false);
}

static void set_state(ChassisCalibrationState state)
{
    gCalibration.state = state;
    gCalibration.stateMs = 0U;
}

static void set_fault(ChassisCalibrationFault fault)
{
    stop_motion();
    gCalibration.fault = fault;
    set_state(CAL_STATE_FAULT);
}

static void start_straight(void)
{
    gCalibration.fault = CAL_FAULT_NONE;
    motion_control_reset();
    motion_control_enable(true);
    line_speed_control_reset(&gCalibration.lineControl);
    gCalibration.cornerClearMs = 0U;
    gCalibration.cornerDebounceMs = 0U;
    gCalibration.startCornerCleared = false;
    set_state(CAL_STATE_STRAIGHT_RUNNING);
}

static void finish_straight(void)
{
    int32_t left = motion_control_get_left_count();
    int32_t right = motion_control_get_right_count();
    uint32_t averageCounts = (uint32_t)((abs_i32(left) +
        abs_i32(right)) / 2);
    uint32_t knownDistanceMm = CHASSIS_TRACK_OUTER_SIZE_MM -
        CHASSIS_LINE_WIDTH_MM;

    stop_motion();
    gCalibration.record.straightLeftCounts = left;
    gCalibration.record.straightRightCounts = right;
    gCalibration.record.straightDistanceMm = knownDistanceMm;
    gCalibration.record.countsPerMeter = divide_round_u64(
        (uint64_t)averageCounts * 1000U, knownDistanceMm);
    if (gCalibration.record.countsPerMeter == 0U) {
        set_fault(CAL_FAULT_INVALID_RESULT);
        return;
    }
    gCalibration.record.flags |= CALIBRATION_FLAG_STRAIGHT_VALID;
    chassis_model_set_counts_per_meter(
        gCalibration.record.countsPerMeter);
    set_state(CAL_STATE_READY_TURN);
}

static void start_turn(bool imuReady)
{
    if (!imuReady) {
        set_fault(CAL_FAULT_IMU_INIT);
        return;
    }

    motion_control_reset();
    imu_heading_reset();
    motion_control_enable(true);
    gCalibration.imuPeriodMs = 0U;
    gCalibration.turnSettleMs = 0U;
    set_state(CAL_STATE_TURN_RUNNING);
}

static void finish_turn(void)
{
    int32_t left = motion_control_get_left_count();
    int32_t right = motion_control_get_right_count();
    int32_t heading = imu_heading_get_mdeg();
    uint64_t sumCounts = (uint64_t)abs_i32(left) +
        (uint64_t)abs_i32(right);
    uint64_t numerator = sumCounts * 1000ULL * 180000ULL * 1000000ULL;
    uint64_t denominator =
        (uint64_t)gCalibration.record.countsPerMeter *
        (uint64_t)abs_i32(heading) * PI_SCALED_1E6;

    stop_motion();
    gCalibration.record.turnLeftCounts = left;
    gCalibration.record.turnRightCounts = right;
    gCalibration.record.turnHeadingMdeg = heading;
    gCalibration.record.effectiveTrackMm = divide_round_u64(numerator,
        denominator);
    if (gCalibration.record.effectiveTrackMm == 0U) {
        set_fault(CAL_FAULT_INVALID_RESULT);
        return;
    }
    gCalibration.record.flags |= CALIBRATION_FLAG_TURN_VALID;
    set_state(CAL_STATE_READY_OFFSET);
}

static void start_offset(void)
{
    motion_control_reset();
    motion_control_enable(true);
    line_speed_control_reset(&gCalibration.lineControl);
    set_state(CAL_STATE_OFFSET_RUNNING);
}

static void finish_offset(void)
{
    int32_t left = motion_control_get_left_count();
    int32_t right = motion_control_get_right_count();
    uint32_t averageCounts = (uint32_t)((abs_i32(left) +
        abs_i32(right)) / 2);

    stop_motion();
    gCalibration.record.cornerOffsetLeftCounts = left;
    gCalibration.record.cornerOffsetRightCounts = right;
    gCalibration.record.cornerOffsetMm = divide_round_u64(
        (uint64_t)averageCounts * 1000U,
        gCalibration.record.countsPerMeter);
    gCalibration.record.flags |= CALIBRATION_FLAG_OFFSET_VALID;
    chassis_model_set_corner_center_offset_mm(
        gCalibration.record.cornerOffsetMm);

    if (!calibration_store_save(&gCalibration.record)) {
        set_fault(CAL_FAULT_FLASH_STORE);
        return;
    }
    set_state(CAL_STATE_COMPLETE);
}

void chassis_calibration_init(uint32_t previousSequence)
{
    uint32_t *words = (uint32_t *)&gCalibration.record;

    for (uint32_t i = 0U;
         i < (sizeof(gCalibration.record) / sizeof(uint32_t)); i++) {
        words[i] = 0U;
    }
    gCalibration.record.sequence = previousSequence + 1U;
    line_speed_control_init(&gCalibration.lineControl);
    gCalibration.fault = CAL_FAULT_NONE;
    set_state(CAL_STATE_READY_STRAIGHT);
}

void chassis_calibration_button(bool imuReady)
{
    switch (gCalibration.state) {
        case CAL_STATE_READY_STRAIGHT:
            start_straight();
            break;
        case CAL_STATE_STRAIGHT_RUNNING:
            stop_motion();
            set_state(CAL_STATE_READY_STRAIGHT);
            break;
        case CAL_STATE_READY_TURN:
            start_turn(imuReady);
            break;
        case CAL_STATE_TURN_RUNNING:
        case CAL_STATE_TURN_SETTLING:
            stop_motion();
            set_state(CAL_STATE_READY_TURN);
            break;
        case CAL_STATE_READY_OFFSET:
            start_offset();
            break;
        case CAL_STATE_OFFSET_RUNNING:
            finish_offset();
            break;
        default:
            break;
    }
}

static void update_straight(uint8_t blackMask)
{
    uint8_t activeCount = track_active_count(blackMask);
    int32_t averageCount = motion_control_get_average_count();

    line_speed_control_update_1ms(&gCalibration.lineControl, blackMask);
    line_speed_control_command(&gCalibration.lineControl,
        CAL_STRAIGHT_SPEED_TICKS);

    if (activeCount < CAL_CORNER_MIN_ACTIVE) {
        if (gCalibration.cornerClearMs < CAL_CORNER_CLEAR_MS) {
            gCalibration.cornerClearMs++;
        }
        if (gCalibration.cornerClearMs >= CAL_CORNER_CLEAR_MS) {
            gCalibration.startCornerCleared = true;
        }
    } else {
        gCalibration.cornerClearMs = 0U;
    }

    if (gCalibration.startCornerCleared &&
        (averageCount >= chassis_mm_to_counts(
            CAL_STRAIGHT_MIN_DISTANCE_MM)) &&
        (activeCount >= CAL_CORNER_MIN_ACTIVE)) {
        if (gCalibration.cornerDebounceMs < CAL_CORNER_DEBOUNCE_MS) {
            gCalibration.cornerDebounceMs++;
        }
    } else {
        gCalibration.cornerDebounceMs = 0U;
    }

    if (gCalibration.cornerDebounceMs >= CAL_CORNER_DEBOUNCE_MS) {
        finish_straight();
    } else if ((gCalibration.stateMs >= CAL_STRAIGHT_TIMEOUT_MS) ||
        (averageCount >= chassis_mm_to_counts(
            CAL_STRAIGHT_MAX_DISTANCE_MM))) {
        set_fault(CAL_FAULT_STRAIGHT_TIMEOUT);
    }
}

static bool update_imu_heading(void)
{
    gCalibration.imuPeriodMs++;
    if (gCalibration.imuPeriodMs < IMU_HEADING_PERIOD_MS) {
        return true;
    }
    gCalibration.imuPeriodMs = 0U;
    return imu_heading_update(IMU_HEADING_PERIOD_MS);
}

static void update_turn(void)
{
    int16_t direction = (SQUARE_TURN_DIRECTION < 0) ? -1 : 1;
    int32_t heading = imu_heading_get_mdeg();

    if (!update_imu_heading()) {
        set_fault(CAL_FAULT_IMU_READ);
        return;
    }
    heading = imu_heading_get_mdeg();

    if (((direction < 0) &&
            (heading >= CAL_TURN_WRONG_DIRECTION_MDEG)) ||
        ((direction > 0) &&
            (heading <= -CAL_TURN_WRONG_DIRECTION_MDEG))) {
        set_fault(CAL_FAULT_TURN_DIRECTION);
        return;
    }
    if ((abs_i32(motion_control_get_left_count()) >=
            chassis_mm_to_counts(CAL_TURN_MAX_WHEEL_DISTANCE_MM)) ||
        (abs_i32(motion_control_get_right_count()) >=
            chassis_mm_to_counts(CAL_TURN_MAX_WHEEL_DISTANCE_MM))) {
        set_fault(CAL_FAULT_TURN_DISTANCE);
        return;
    }
    motion_control_set_speed_targets(
        (int16_t)(-direction * CAL_TURN_SPEED_TICKS),
        (int16_t)(direction * CAL_TURN_SPEED_TICKS));

    if (((direction < 0) && (heading <= -CAL_TURN_TARGET_MDEG)) ||
        ((direction > 0) && (heading >= CAL_TURN_TARGET_MDEG))) {
        motion_control_set_speed_targets(0, 0);
        gCalibration.turnSettleMs = 0U;
        set_state(CAL_STATE_TURN_SETTLING);
    } else if (gCalibration.stateMs >= CAL_TURN_TIMEOUT_MS) {
        set_fault(CAL_FAULT_TURN_TIMEOUT);
    }
}

static void update_turn_settle(void)
{
    motion_control_set_speed_targets(0, 0);
    if (!update_imu_heading()) {
        set_fault(CAL_FAULT_IMU_READ);
        return;
    }

    if ((abs_i32(motion_control_get_left_speed()) <=
            SQUARE_STOP_SPEED_TOLERANCE) &&
        (abs_i32(motion_control_get_right_speed()) <=
            SQUARE_STOP_SPEED_TOLERANCE)) {
        gCalibration.turnSettleMs++;
    } else {
        gCalibration.turnSettleMs = 0U;
    }
    if (gCalibration.turnSettleMs >= CAL_TURN_SETTLE_MS) {
        finish_turn();
    }
}

static void update_offset(uint8_t blackMask)
{
    line_speed_control_update_1ms(&gCalibration.lineControl, blackMask);
    line_speed_control_command(&gCalibration.lineControl,
        CAL_OFFSET_SPEED_TICKS);
    if ((gCalibration.stateMs >= CAL_OFFSET_TIMEOUT_MS) ||
        (abs_i32(motion_control_get_average_count()) >=
            chassis_mm_to_counts(CAL_OFFSET_MAX_DISTANCE_MM))) {
        set_fault(CAL_FAULT_OFFSET_TIMEOUT);
    }
}

void chassis_calibration_update_1ms(uint8_t blackMask)
{
    if (!chassis_calibration_is_running()) {
        return;
    }

    gCalibration.stateMs++;
    switch (gCalibration.state) {
        case CAL_STATE_STRAIGHT_RUNNING:
            update_straight(blackMask);
            break;
        case CAL_STATE_TURN_RUNNING:
            update_turn();
            break;
        case CAL_STATE_TURN_SETTLING:
            update_turn_settle();
            break;
        case CAL_STATE_OFFSET_RUNNING:
            update_offset(blackMask);
            break;
        default:
            break;
    }

    motion_control_update_1ms();
    if (motion_control_get_fault() != MOTION_FAULT_NONE) {
        set_fault(CAL_FAULT_MOTION_CONTROL);
    }
}

ChassisCalibrationState chassis_calibration_get_state(void)
{
    return gCalibration.state;
}

ChassisCalibrationFault chassis_calibration_get_fault(void)
{
    return gCalibration.fault;
}

const ChassisCalibrationRecord *chassis_calibration_get_record(void)
{
    return &gCalibration.record;
}

int16_t chassis_calibration_get_line_error(void)
{
    return gCalibration.lineControl.error;
}

int16_t chassis_calibration_get_line_correction(void)
{
    return gCalibration.lineControl.correction;
}

bool chassis_calibration_is_running(void)
{
    return (gCalibration.state == CAL_STATE_STRAIGHT_RUNNING) ||
        (gCalibration.state == CAL_STATE_TURN_RUNNING) ||
        (gCalibration.state == CAL_STATE_TURN_SETTLING) ||
        (gCalibration.state == CAL_STATE_OFFSET_RUNNING);
}
