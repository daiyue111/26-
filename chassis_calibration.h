#ifndef CHASSIS_CALIBRATION_H
#define CHASSIS_CALIBRATION_H

#include "calibration_store.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAL_STATE_READY_STRAIGHT = 0,
    CAL_STATE_STRAIGHT_RUNNING,
    CAL_STATE_READY_TURN,
    CAL_STATE_TURN_RUNNING,
    CAL_STATE_TURN_SETTLING,
    CAL_STATE_READY_OFFSET,
    CAL_STATE_OFFSET_RUNNING,
    CAL_STATE_COMPLETE,
    CAL_STATE_FAULT
} ChassisCalibrationState;

typedef enum {
    CAL_FAULT_NONE = 0,
    CAL_FAULT_INVALID_RESULT,
    CAL_FAULT_IMU_INIT,
    CAL_FAULT_IMU_READ,
    CAL_FAULT_MOTION_CONTROL,
    CAL_FAULT_STRAIGHT_TIMEOUT,
    CAL_FAULT_TURN_DIRECTION,
    CAL_FAULT_TURN_TIMEOUT,
    CAL_FAULT_TURN_DISTANCE,
    CAL_FAULT_OFFSET_TIMEOUT,
    CAL_FAULT_FLASH_STORE
} ChassisCalibrationFault;

void chassis_calibration_init(uint32_t previousSequence);
void chassis_calibration_button(bool imuReady);
void chassis_calibration_update_1ms(uint8_t blackMask);
ChassisCalibrationState chassis_calibration_get_state(void);
ChassisCalibrationFault chassis_calibration_get_fault(void);
const ChassisCalibrationRecord *chassis_calibration_get_record(void);
int16_t chassis_calibration_get_line_error(void);
int16_t chassis_calibration_get_line_correction(void);
bool chassis_calibration_is_running(void);

#endif
