#ifndef H_MISSION_H
#define H_MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H_STATE_STOPPED = 0,
    H_STATE_RUNNING,
    H_STATE_TASK3_TO_POSITIVE,
    H_STATE_TASK3_TO_NEGATIVE,
    H_STATE_TASK3_SETTLING,
    H_STATE_PASS_FINISH,
    H_STATE_BRAKING,
    H_STATE_COMPLETE,
    H_STATE_FAULT
} HMissionState;

typedef enum {
    H_FAULT_NONE = 0,
    H_FAULT_BAD_TASK,
    H_FAULT_START_OFF_LINE,
    H_FAULT_VISION_NOT_READY,
    H_FAULT_VISION_LOST,
    H_FAULT_BALL_LIMIT,
    H_FAULT_LINE_LOST,
    H_FAULT_MOTION,
    H_FAULT_TIMEOUT,
    H_FAULT_FINISH_MARKER
} HMissionFault;

void h_mission_init(void);
bool h_mission_set_task(uint8_t task);
bool h_mission_set_arbitrary_target_tenth_mm(int16_t targetTenthMm);
bool h_mission_start(uint32_t nowMs, uint8_t blackMask);
void h_mission_stop(void);
void h_mission_update_1ms(uint32_t nowMs, uint8_t blackMask);
HMissionState h_mission_get_state(void);
HMissionFault h_mission_get_fault(void);
uint8_t h_mission_get_task(void);
uint32_t h_mission_get_elapsed_ms(uint32_t nowMs);
uint32_t h_mission_get_result_ms(void);
int32_t h_mission_get_distance_mm(void);
int16_t h_mission_get_speed_command_ticks(void);
int32_t h_mission_get_acceleration_mmps2(void);
int16_t h_mission_get_line_error(void);
int16_t h_mission_get_line_correction(void);

#endif
