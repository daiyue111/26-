#include "car_app.h"

#include "app_config.h"
#include "app_io.h"
#include "ball_control.h"
#include "ball_vision.h"
#include "calibration_store.h"
#include "chassis_calibration.h"
#include "chassis_model.h"
#include "control_scheduler.h"
#include "encoder.h"
#include "h_mission.h"
#include "imu.h"
#include "line_follow.h"
#include "motion_control.h"
#include "motor.h"
#include "power_switch.h"
#include "rod_actuator.h"
#include "square_mission.h"
#include "track.h"

#include <stdbool.h>

#define FAULT_LED_CYCLE_MS 20000U
#define MOTION_FAULT_HEADER_ON_MS 4000U
#define FAULT_HEADER_GAP_MS 1000U
#define FAULT_PULSE_PERIOD_MS 1200U
#define FAULT_PULSE_ON_MS 600U
#define READY_LED_CYCLE_MS 4000U
#define READY_LED_PULSE_PERIOD_MS 1000U
#define READY_LED_PULSE_ON_MS 500U
#define MISSION_FAULT_BLINK_CODE 7U
#define H_FAULT_LED_CYCLE_MS 3000U
#define H_FAULT_LED_SLOT_MS 300U
#define H_FAULT_LED_ON_MS 150U

volatile uint8_t gTrackRawMask;
volatile uint8_t gTrackBlackMask;
volatile uint8_t gTrackActiveCount;
volatile int16_t gTrackError;
volatile int16_t gTrackDerivative;
volatile uint8_t gLeftDuty;
volatile uint8_t gRightDuty;
volatile uint8_t gStartButtonPressed;
volatile uint8_t gLineRunning;
volatile uint8_t gMotorTestChannel;
volatile uint8_t gImuReady;
volatile uint8_t gImuReadOk;
volatile uint8_t gImuError;
volatile int32_t gGyroZDelta;
volatile int32_t gGyroLostHeading;
volatile int16_t gGyroCorrectionTicks;
volatile int32_t gEncoderLeftCount;
volatile int32_t gEncoderRightCount;
volatile int16_t gEncoderLeftSpeed;
volatile int16_t gEncoderRightSpeed;
volatile int16_t gLeftPwmCommand;
volatile int16_t gRightPwmCommand;
volatile int32_t gHeadingMdeg;
volatile uint8_t gMissionState;
volatile uint8_t gMissionCornerCount;
volatile int32_t gMissionDistanceError;
volatile int32_t gMissionHeadingError;
volatile uint8_t gPowerSwitchDutyPercent;
volatile uint8_t gPowerSwitchOutputOn;
volatile uint8_t gPowerSwitchTimedOut;
volatile uint8_t gMotionFault;
volatile int32_t gEncoderLeftSpeedMmps;
volatile int32_t gEncoderRightSpeedMmps;
volatile uint8_t gCalibrationStoredValid;
volatile uint8_t gCalibrationState;
volatile uint8_t gCalibrationFault;
volatile uint32_t gCalibrationSequence;
volatile uint32_t gCalibrationFlags;
volatile int32_t gCalibrationStraightLeftCounts;
volatile int32_t gCalibrationStraightRightCounts;
volatile uint32_t gCalibrationStraightDistanceMm;
volatile uint32_t gCalibrationCountsPerMeter;
volatile int32_t gCalibrationTurnLeftCounts;
volatile int32_t gCalibrationTurnRightCounts;
volatile int32_t gCalibrationTurnHeadingMdeg;
volatile uint32_t gCalibrationEffectiveTrackMm;
volatile int32_t gCalibrationOffsetLeftCounts;
volatile int32_t gCalibrationOffsetRightCounts;
volatile uint32_t gCalibrationCornerOffsetMm;
volatile uint8_t gHRequestedTask;
volatile int16_t gHRequestedTargetTenthMm;
volatile uint8_t gHTask;
volatile uint8_t gHState;
volatile uint8_t gHFault;
volatile uint32_t gHElapsedMs;
volatile uint32_t gHResultMs;
volatile int32_t gHDistanceMm;
volatile int16_t gHSpeedCommandTicks;
volatile int32_t gHAccelerationMmps2;
volatile int16_t gHBallTargetTenthMm;
volatile int16_t gHBallPositionTenthMm;
volatile int16_t gHBallVelocityTenthMmps;
volatile int16_t gHBallErrorTenthMm;
volatile int16_t gHRodAngleMdeg;
volatile uint8_t gHVisionHealthy;
volatile uint32_t gHVisionAgeMs;
volatile uint32_t gHVisionGoodFrames;
volatile uint32_t gHVisionCrcErrors;
volatile uint32_t gSchedulerOverruns;

typedef struct {
    uint16_t pressedMs;
    uint16_t releasedMs;
    bool keyLatched;
    bool keyArmed;
    bool running;
    uint16_t startBoostMs;
    uint8_t testStep;
    uint32_t testStepMs;
    LineFollowState line;
} CarAppState;

static CarAppState gApp;
static ChassisCalibrationRecord gStoredCalibration;
static bool gStoredCalibrationValid;

static uint8_t h_fault_display_code(HMissionFault fault)
{
    switch (fault) {
        case H_FAULT_LINE_LOST:
            return 1U;
        case H_FAULT_MOTION:
            return 2U;
        case H_FAULT_TIMEOUT:
            return 3U;
        case H_FAULT_FINISH_MARKER:
            return 4U;
        case H_FAULT_NONE:
            return 0U;
        default:
            return 5U;
    }
}

static bool h_fault_led_on(uint32_t nowMs, HMissionFault fault)
{
    uint8_t pulses = h_fault_display_code(fault);
    uint16_t phase = (uint16_t)(nowMs % H_FAULT_LED_CYCLE_MS);

    return (pulses != 0U) &&
        ((phase / H_FAULT_LED_SLOT_MS) < pulses) &&
        ((phase % H_FAULT_LED_SLOT_MS) < H_FAULT_LED_ON_MS);
}

static void sync_power_switch_telemetry(void)
{
    gPowerSwitchDutyPercent = power_switch_get_duty();
    gPowerSwitchOutputOn = power_switch_is_on() ? 1U : 0U;
    gPowerSwitchTimedOut = power_switch_has_timed_out() ? 1U : 0U;
}

static uint8_t abs_pwm(int16_t value)
{
    int32_t result = value;

    if (result < 0) {
        result = -result;
    }
    if (result > PWM_PERIOD_TICKS) {
        result = PWM_PERIOD_TICKS;
    }
    return (uint8_t)result;
}

static void sync_line_telemetry(void)
{
    gTrackError = gApp.line.error;
    gTrackDerivative = gApp.line.derivative;
    gLeftDuty = gApp.line.leftDuty;
    gRightDuty = gApp.line.rightDuty;
    gImuReady = gApp.line.imuReady ? 1U : 0U;
    gImuReadOk = gApp.line.imuReadOk ? 1U : 0U;
    gGyroZDelta = gApp.line.gyroZDelta;
    gGyroLostHeading = gApp.line.gyroLostHeading;
    gGyroCorrectionTicks = gApp.line.gyroCorrectionTicks;
}

static void sync_square_telemetry(void)
{
    gTrackError = square_mission_get_line_error();
    gTrackDerivative = 0;
    gEncoderLeftCount = motion_control_get_left_count();
    gEncoderRightCount = motion_control_get_right_count();
    gEncoderLeftSpeed = motion_control_get_left_speed();
    gEncoderRightSpeed = motion_control_get_right_speed();
    gEncoderLeftSpeedMmps = chassis_speed_ticks_to_mmps(
        gEncoderLeftSpeed);
    gEncoderRightSpeedMmps = chassis_speed_ticks_to_mmps(
        gEncoderRightSpeed);
    gLeftPwmCommand = motion_control_get_left_pwm();
    gRightPwmCommand = motion_control_get_right_pwm();
    gMotionFault = (uint8_t)motion_control_get_fault();
    gLeftDuty = abs_pwm(gLeftPwmCommand);
    gRightDuty = abs_pwm(gRightPwmCommand);
    gHeadingMdeg = imu_heading_get_mdeg();
    gGyroZDelta = imu_get_last_gyro_delta();
    gGyroLostHeading = gHeadingMdeg;
    gGyroCorrectionTicks = square_mission_get_line_correction();
    gMissionState = (uint8_t)square_mission_get_state();
    gMissionCornerCount = square_mission_get_corner_count();
    gMissionDistanceError = square_mission_get_distance_error();
    gMissionHeadingError = square_mission_get_heading_error();
    gImuReadOk = (gMissionState == SQUARE_STATE_FAULT) ? 0U : gImuReady;
}

static void sync_h_telemetry(uint32_t nowMs)
{
    gHTask = h_mission_get_task();
    gHState = (uint8_t)h_mission_get_state();
    gHFault = (uint8_t)h_mission_get_fault();
    gHElapsedMs = h_mission_get_elapsed_ms(nowMs);
    gHResultMs = h_mission_get_result_ms();
    gHDistanceMm = h_mission_get_distance_mm();
    gHSpeedCommandTicks = h_mission_get_speed_command_ticks();
    gHAccelerationMmps2 = h_mission_get_acceleration_mmps2();
    gHBallTargetTenthMm = ball_control_get_target_tenth_mm();
    gHBallPositionTenthMm = ball_control_get_position_tenth_mm();
    gHBallVelocityTenthMmps = ball_control_get_velocity_tenth_mmps();
    gHBallErrorTenthMm = ball_control_get_error_tenth_mm();
    gHRodAngleMdeg = rod_actuator_get_angle_mdeg();
    gHVisionHealthy = ball_control_is_vision_healthy() ? 1U : 0U;
    gHVisionAgeMs = ball_control_get_vision_age_ms(nowMs);
    gHVisionGoodFrames = ball_vision_get_good_frame_count();
    gHVisionCrcErrors = ball_vision_get_crc_error_count();
    gSchedulerOverruns = control_scheduler_get_overrun_count();
    gTrackError = h_mission_get_line_error();
    gTrackDerivative = 0;
    gGyroCorrectionTicks = h_mission_get_line_correction();
    gEncoderLeftCount = motion_control_get_left_count();
    gEncoderRightCount = motion_control_get_right_count();
    gEncoderLeftSpeed = motion_control_get_left_speed();
    gEncoderRightSpeed = motion_control_get_right_speed();
    gEncoderLeftSpeedMmps = chassis_speed_ticks_to_mmps(gEncoderLeftSpeed);
    gEncoderRightSpeedMmps = chassis_speed_ticks_to_mmps(gEncoderRightSpeed);
    gLeftPwmCommand = motion_control_get_left_pwm();
    gRightPwmCommand = motion_control_get_right_pwm();
    gMotionFault = (uint8_t)motion_control_get_fault();
    gLeftDuty = abs_pwm(gLeftPwmCommand);
    gRightDuty = abs_pwm(gRightPwmCommand);
}

static void sync_calibration_telemetry(void)
{
    const ChassisCalibrationRecord *record =
        chassis_calibration_get_record();

    gTrackError = chassis_calibration_get_line_error();
    gTrackDerivative = 0;
    gGyroCorrectionTicks = chassis_calibration_get_line_correction();
    gEncoderLeftCount = motion_control_get_left_count();
    gEncoderRightCount = motion_control_get_right_count();
    gEncoderLeftSpeed = motion_control_get_left_speed();
    gEncoderRightSpeed = motion_control_get_right_speed();
    gEncoderLeftSpeedMmps = chassis_speed_ticks_to_mmps(
        gEncoderLeftSpeed);
    gEncoderRightSpeedMmps = chassis_speed_ticks_to_mmps(
        gEncoderRightSpeed);
    gLeftPwmCommand = motion_control_get_left_pwm();
    gRightPwmCommand = motion_control_get_right_pwm();
    gMotionFault = (uint8_t)motion_control_get_fault();
    gLeftDuty = abs_pwm(gLeftPwmCommand);
    gRightDuty = abs_pwm(gRightPwmCommand);
    gHeadingMdeg = imu_heading_get_mdeg();
    gGyroZDelta = imu_get_last_gyro_delta();
    gCalibrationState = (uint8_t)chassis_calibration_get_state();
    gCalibrationFault = (uint8_t)chassis_calibration_get_fault();
    gCalibrationSequence = record->sequence;
    gCalibrationFlags = record->flags;
    gCalibrationStraightLeftCounts = record->straightLeftCounts;
    gCalibrationStraightRightCounts = record->straightRightCounts;
    gCalibrationStraightDistanceMm = record->straightDistanceMm;
    gCalibrationCountsPerMeter = record->countsPerMeter;
    gCalibrationTurnLeftCounts = record->turnLeftCounts;
    gCalibrationTurnRightCounts = record->turnRightCounts;
    gCalibrationTurnHeadingMdeg = record->turnHeadingMdeg;
    gCalibrationEffectiveTrackMm = record->effectiveTrackMm;
    gCalibrationOffsetLeftCounts = record->cornerOffsetLeftCounts;
    gCalibrationOffsetRightCounts = record->cornerOffsetRightCounts;
    gCalibrationCornerOffsetMm = record->cornerOffsetMm;
    if ((record->flags == CALIBRATION_FLAGS_ALL) &&
        (chassis_calibration_get_state() == CAL_STATE_COMPLETE)) {
        gCalibrationStoredValid = 1U;
    }
}

static void start_line_following(void)
{
#if APP_RUN_MODE == APP_MODE_H2026
    uint8_t blackMask = track_black_mask(track_read_raw_mask());

    motor_safe_stop();
    gApp.running = h_mission_start(control_scheduler_now_ms(), blackMask);
#else
    bool imuReady;

    motor_safe_stop();
    app_gyro_led_set(false);
    imuReady = imu_init_gyro_z();
    gImuError = imu_get_error();
    gImuReady = imuReady ? 1U : 0U;
    gImuReadOk = gImuReady;
    line_follow_reset(&gApp.line, imuReady);
    app_gyro_led_set(imuReady);

    if (!imuReady) {
        app_gyro_led_report_imu_error(gImuError);
    }

#if APP_RUN_MODE == APP_MODE_SQUARE_3LOOP
    if (!imuReady) {
        gApp.running = false;
        return;
    }
    if ((track_black_mask(track_read_raw_mask()) & LINE_CENTER_MASK) == 0U) {
        gApp.running = false;
        app_gyro_led_blink_error(MISSION_FAULT_BLINK_CODE);
        return;
    }
    gApp.running = square_mission_start(imuReady);
    gApp.startBoostMs = 0U;
#else
    gApp.running = true;
    gApp.startBoostMs = START_BOOST_MS;
#endif
#endif
}

static void handle_start_key(bool pressed)
{
    if (pressed) {
        if (gApp.pressedMs < KEY_DEBOUNCE_MS) {
            gApp.pressedMs++;
        }
        if ((gApp.pressedMs >= KEY_DEBOUNCE_MS) && !gApp.keyLatched) {
            gApp.keyLatched = true;

            if (gApp.running) {
                gApp.running = false;
                gApp.startBoostMs = 0U;
#if APP_RUN_MODE == APP_MODE_SQUARE_3LOOP
                square_mission_stop();
#elif APP_RUN_MODE == APP_MODE_H2026
                h_mission_stop();
#else
                motor_safe_stop();
#endif
            } else {
                start_line_following();
            }
        }
    } else {
        gApp.pressedMs = 0U;
        gApp.keyLatched = false;
    }
}

#if POWER_SWITCH_TEST
static void run_power_switch_test_step(void)
{
    bool pressed = app_start_key_pressed();

    motor_safe_stop();
    gStartButtonPressed = pressed ? 1U : 0U;
    gLineRunning = 0U;
    app_debug_led_set(pressed);
    if (pressed) {
        power_switch_set_duty(POWER_SWITCH_TEST_DUTY_PERCENT);
    } else {
        power_switch_off();
    }
    power_switch_update_1ms();
    sync_power_switch_telemetry();
}
#elif MOTOR_CHANNEL_TEST
static void run_motor_test_step(void)
{
    bool activeStep = (gApp.testStep & 1U) != 0U;
    uint16_t stepDurationMs = (gApp.testStep == 0U) ? 2000U :
        (activeStep ? 2000U : 1000U);

    gStartButtonPressed = 0U;
    gMotorTestChannel = gApp.testStep;
    gLineRunning = activeStep ? 1U : 0U;
    app_debug_led_set(activeStep);

    if (activeStep) {
        motor_test_set_direction(gApp.testStep);
        motor_test_pwm_run_1ms(gApp.testStep);
    } else {
        motor_safe_stop();
    }

    gApp.testStepMs++;
    if (gApp.testStepMs >= stepDurationMs) {
        motor_safe_stop();
        gApp.testStep = (uint8_t)((gApp.testStep + 1U) % 8U);
        gApp.testStepMs = 0U;
    }
}
#elif APP_RUN_MODE == APP_MODE_ENCODER_DIAGNOSTIC
static bool encoder_test_led(int32_t count)
{
    if (count > 0) {
        return true;
    }
    if (count < 0) {
        return ((gApp.testStepMs / 200U) & 1U) != 0U;
    }
    return false;
}

static void run_encoder_input_test_step(void)
{
    bool pressed = app_start_key_pressed();
    int32_t selectedCount;

    motor_safe_stop();
    power_switch_off();
    if (pressed) {
        if (gApp.pressedMs < KEY_DEBOUNCE_MS) {
            gApp.pressedMs++;
        }
        if ((gApp.pressedMs >= KEY_DEBOUNCE_MS) && !gApp.keyLatched) {
            gApp.keyLatched = true;
            gApp.testStep ^= 1U;
            encoder_reset();
        }
    } else {
        gApp.pressedMs = 0U;
        gApp.keyLatched = false;
    }
    gApp.testStepMs++;
    gStartButtonPressed = pressed ? 1U : 0U;
    gLineRunning = 0U;
    sync_square_telemetry();
    sync_power_switch_telemetry();
    gMotorTestChannel = gApp.testStep;
    selectedCount = (gApp.testStep == 0U) ? gEncoderLeftCount :
        gEncoderRightCount;
    app_debug_led_set(gApp.testStep != 0U);
    app_gyro_led_set(encoder_test_led(selectedCount));
}
#elif APP_RUN_MODE == APP_MODE_SAFE_IDLE
static void run_safe_idle_step(void)
{
    motor_safe_stop();
    power_switch_off();
    gLineRunning = 0U;
    gStartButtonPressed = app_start_key_pressed() ? 1U : 0U;
    sync_square_telemetry();
    sync_power_switch_telemetry();
    app_debug_led_set(false);
    app_gyro_led_set(false);
}
#elif APP_RUN_MODE == APP_MODE_SPEED_TUNE
static void run_speed_tune_step(void)
{
    bool pressed = app_start_key_pressed();

    if (pressed) {
        if (gApp.pressedMs < KEY_DEBOUNCE_MS) {
            gApp.pressedMs++;
        }
        if ((gApp.pressedMs >= KEY_DEBOUNCE_MS) && !gApp.keyLatched) {
            gApp.keyLatched = true;
            if (gApp.running) {
                gApp.running = false;
                motion_control_enable(false);
            } else {
                motion_control_reset();
                motion_control_enable(true);
                motion_control_set_speed_targets(SPEED_TUNE_TARGET_TICKS,
                    SPEED_TUNE_TARGET_TICKS);
                gApp.running = true;
            }
        }
    } else {
        gApp.pressedMs = 0U;
        gApp.keyLatched = false;
    }

    if (gApp.running) {
        motion_control_update_1ms();
        if (motion_control_get_fault() != MOTION_FAULT_NONE) {
            gApp.running = false;
        }
    } else {
        motor_safe_stop();
    }

    gStartButtonPressed = pressed ? 1U : 0U;
    gLineRunning = gApp.running ? 1U : 0U;
    sync_square_telemetry();
    app_debug_led_set(gApp.running);
    app_gyro_led_set(motion_control_get_fault() != MOTION_FAULT_NONE);
}
#elif APP_RUN_MODE == APP_MODE_CHASSIS_CALIBRATION
static bool calibration_status_led(ChassisCalibrationState state)
{
    uint16_t phase =
        (uint16_t)(gApp.testStepMs % READY_LED_CYCLE_MS);
    uint8_t readyPulses;

    switch (state) {
        case CAL_STATE_READY_STRAIGHT:
            readyPulses = 1U;
            break;
        case CAL_STATE_READY_TURN:
            readyPulses = 2U;
            break;
        case CAL_STATE_READY_OFFSET:
            readyPulses = 3U;
            break;
        case CAL_STATE_STRAIGHT_RUNNING:
        case CAL_STATE_TURN_RUNNING:
        case CAL_STATE_TURN_SETTLING:
        case CAL_STATE_OFFSET_RUNNING:
            return true;
        case CAL_STATE_COMPLETE:
            return phase < 2000U;
        case CAL_STATE_FAULT: {
            uint16_t faultPhase =
                (uint16_t)(gApp.testStepMs % FAULT_LED_CYCLE_MS);
            uint8_t motionFault = (uint8_t)motion_control_get_fault();
            uint8_t pulses;

            if (motionFault != MOTION_FAULT_NONE) {
                if (faultPhase < MOTION_FAULT_HEADER_ON_MS) {
                    return true;
                }
                if (faultPhase < (MOTION_FAULT_HEADER_ON_MS +
                        FAULT_HEADER_GAP_MS)) {
                    return false;
                }
                faultPhase = (uint16_t)(faultPhase -
                    (MOTION_FAULT_HEADER_ON_MS + FAULT_HEADER_GAP_MS));
                pulses = motionFault;
            } else {
                if (faultPhase < FAULT_HEADER_GAP_MS) {
                    return false;
                }
                faultPhase = (uint16_t)(faultPhase -
                    FAULT_HEADER_GAP_MS);
                pulses = (uint8_t)chassis_calibration_get_fault();
            }
            return ((faultPhase / FAULT_PULSE_PERIOD_MS) < pulses) &&
                ((faultPhase % FAULT_PULSE_PERIOD_MS) <
                    FAULT_PULSE_ON_MS);
        }
        default:
            return false;
    }

    return ((phase / READY_LED_PULSE_PERIOD_MS) < readyPulses) &&
        ((phase % READY_LED_PULSE_PERIOD_MS) < READY_LED_PULSE_ON_MS);
}

static void run_chassis_calibration_step(void)
{
    bool pressed = app_start_key_pressed();
    uint8_t rawMask = track_read_raw_mask();
    uint8_t blackMask = track_black_mask(rawMask);

    gTrackRawMask = rawMask;
    gTrackBlackMask = blackMask;
    gTrackActiveCount = track_active_count(blackMask);
    gStartButtonPressed = pressed ? 1U : 0U;

    if (!gApp.keyArmed) {
        gApp.pressedMs = 0U;
        gApp.keyLatched = false;
        if (pressed) {
            gApp.releasedMs = 0U;
        } else {
            if (gApp.releasedMs < KEY_RELEASE_ARM_MS) {
                gApp.releasedMs++;
            }
            if (gApp.releasedMs >= KEY_RELEASE_ARM_MS) {
                gApp.keyArmed = true;
            }
        }
    } else if (pressed) {
        if (gApp.pressedMs < KEY_DEBOUNCE_MS) {
            gApp.pressedMs++;
        }
        if ((gApp.pressedMs >= KEY_DEBOUNCE_MS) && !gApp.keyLatched) {
            bool imuReady = false;

            gApp.keyLatched = true;
            if (chassis_calibration_get_state() == CAL_STATE_READY_TURN) {
                motor_safe_stop();
                app_gyro_led_set(false);
                imuReady = imu_init_gyro_z();
                gImuError = imu_get_error();
                gImuReady = imuReady ? 1U : 0U;
                gImuReadOk = gImuReady;
                if (!imuReady) {
                    app_gyro_led_blink_error(gImuError);
                }
            }
            chassis_calibration_button(imuReady);
        }
    } else {
        gApp.pressedMs = 0U;
        gApp.keyLatched = false;
    }

    chassis_calibration_update_1ms(blackMask);
    gApp.running = chassis_calibration_is_running();
    gLineRunning = gApp.running ? 1U : 0U;
    gApp.testStepMs++;
    sync_calibration_telemetry();
    app_debug_led_set(false);
    app_gyro_led_set(calibration_status_led(
        chassis_calibration_get_state()));
    power_switch_off();
    power_switch_update_1ms();
    sync_power_switch_telemetry();
}
#elif APP_RUN_MODE == APP_MODE_H2026
static void run_h2026_step(void)
{
    bool pressed = app_start_key_pressed();
    uint8_t rawMask = track_read_raw_mask();
    uint8_t blackMask = track_black_mask(rawMask);
    uint32_t nowMs = control_scheduler_now_ms();
    HMissionState state;

    gStartButtonPressed = pressed ? 1U : 0U;
    gTrackRawMask = rawMask;
    gTrackBlackMask = blackMask;
    gTrackActiveCount = track_active_count(blackMask);

    if (!gApp.running) {
        if (gHRequestedTask != h_mission_get_task()) {
            (void)h_mission_set_task(gHRequestedTask);
        }
        if ((h_mission_get_task() == H_TASK_LAP_ARBITRARY_BALL) &&
            (gHRequestedTargetTenthMm !=
                ball_control_get_target_tenth_mm())) {
            (void)h_mission_set_arbitrary_target_tenth_mm(
                gHRequestedTargetTenthMm);
        }
    }
    handle_start_key(pressed);
    h_mission_update_1ms(nowMs, blackMask);
    state = h_mission_get_state();
    if ((state == H_STATE_COMPLETE) || (state == H_STATE_FAULT)) {
        gApp.running = false;
    }

    gLineRunning = gApp.running ? 1U : 0U;
    app_debug_led_set(gApp.running);
    if (state == H_STATE_FAULT) {
        app_gyro_led_set(h_fault_led_on(nowMs, h_mission_get_fault()));
    } else {
        app_gyro_led_set((state == H_STATE_COMPLETE) ||
            ball_control_is_vision_healthy());
    }
    sync_h_telemetry(nowMs);
}
#elif (APP_RUN_MODE == APP_MODE_LINE_GYRO) || \
    (APP_RUN_MODE == APP_MODE_SQUARE_3LOOP)
static void run_line_follow_step(void)
{
    bool pressed = app_start_key_pressed();
    uint8_t rawMask = track_read_raw_mask();
    uint8_t blackMask = track_black_mask(rawMask);

    gStartButtonPressed = pressed ? 1U : 0U;
    gTrackRawMask = rawMask;
    gTrackBlackMask = blackMask;
    gTrackActiveCount = track_active_count(blackMask);

    handle_start_key(pressed);

    gLineRunning = gApp.running ? 1U : 0U;
    app_debug_led_set(gApp.running);

    if (gApp.running) {
#if APP_RUN_MODE == APP_MODE_SQUARE_3LOOP
        SquareMissionState missionState;

        square_mission_update_1ms(blackMask);
        missionState = square_mission_get_state();
        if (missionState == SQUARE_STATE_COMPLETE) {
            gApp.running = false;
            app_gyro_led_set(false);
        } else if (missionState == SQUARE_STATE_FAULT) {
            uint8_t faultCode = (uint8_t)motion_control_get_fault();

            gApp.running = false;
            app_gyro_led_set(false);
            if (faultCode == 0U) {
                uint8_t missionFault = square_mission_get_fault_code();

                app_gyro_led_blink_error((missionFault == 0U) ?
                    MISSION_FAULT_BLINK_CODE : missionFault);
            } else {
                app_gyro_led_report_motion_error(faultCode);
            }
        }
#else
        motor_forward();
        if (gApp.startBoostMs > 0U) {
            gApp.line.leftDuty = START_BOOST_LEFT_DUTY;
            gApp.line.rightDuty = START_BOOST_RIGHT_DUTY;
            motor_pwm_run_1ms(START_BOOST_LEFT_DUTY,
                START_BOOST_RIGHT_DUTY);
            gApp.startBoostMs--;
        } else {
            line_follow_run_1ms(&gApp.line, blackMask);
        }
#endif
    } else {
        motor_safe_stop();
        gApp.line.leftDuty = 0U;
        gApp.line.rightDuty = 0U;
    }

#if APP_RUN_MODE == APP_MODE_SQUARE_3LOOP
    sync_square_telemetry();
#else
    sync_line_telemetry();
#endif
    gLineRunning = gApp.running ? 1U : 0U;
    app_debug_led_set(gApp.running);
}
#endif

void car_app_init(void)
{
    gStoredCalibrationValid = calibration_store_load(&gStoredCalibration);
    gCalibrationStoredValid = gStoredCalibrationValid ? 1U : 0U;
    if (gStoredCalibrationValid && USE_STORED_CHASSIS_CALIBRATION) {
        chassis_model_set_counts_per_meter(
            gStoredCalibration.countsPerMeter);
        chassis_model_set_corner_center_offset_mm(
            gStoredCalibration.cornerOffsetMm);
    }
    line_follow_reset(&gApp.line, false);
    motor_init();
    power_switch_init();
    motion_control_init();
    square_mission_init();
    h_mission_init();
    gHRequestedTask = H_DEFAULT_TASK;
    gHRequestedTargetTenthMm = 0;
    chassis_calibration_init(gStoredCalibrationValid ?
        gStoredCalibration.sequence : 0U);
    motor_safe_stop();
    app_debug_led_set(false);
#if APP_RUN_MODE == APP_MODE_CHASSIS_CALIBRATION
    for (uint8_t pulse = 0U; pulse < 3U; pulse++) {
        app_gyro_led_set(true);
        app_delay_ms(150U);
        app_gyro_led_set(false);
        app_delay_ms(150U);
    }
#else
    app_gyro_led_startup_test();
#endif
    sync_line_telemetry();
    sync_power_switch_telemetry();
}

void car_app_step(void)
{
#if POWER_SWITCH_TEST
    run_power_switch_test_step();
#elif MOTOR_CHANNEL_TEST
    run_motor_test_step();
#elif APP_RUN_MODE == APP_MODE_ENCODER_DIAGNOSTIC
    run_encoder_input_test_step();
#elif APP_RUN_MODE == APP_MODE_SAFE_IDLE
    run_safe_idle_step();
#elif APP_RUN_MODE == APP_MODE_SPEED_TUNE
    run_speed_tune_step();
#elif APP_RUN_MODE == APP_MODE_CHASSIS_CALIBRATION
    run_chassis_calibration_step();
#elif APP_RUN_MODE == APP_MODE_H2026
    run_h2026_step();
    power_switch_off();
    power_switch_update_1ms();
    sync_power_switch_telemetry();
#elif (APP_RUN_MODE == APP_MODE_LINE_GYRO) || \
    (APP_RUN_MODE == APP_MODE_SQUARE_3LOOP)
    run_line_follow_step();
    power_switch_update_1ms();
    sync_power_switch_telemetry();
#else
#error "APP_RUN_MODE has no application handler"
#endif
}
