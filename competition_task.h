#ifndef COMPETITION_TASK_H
#define COMPETITION_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMPETITION_TASK_STOPPED = 0,
    COMPETITION_TASK_RUNNING,
    COMPETITION_TASK_COMPLETE,
    COMPETITION_TASK_FAULT
} CompetitionTaskState;

typedef enum {
    COMPETITION_FAULT_NONE = 0,
    COMPETITION_FAULT_START_CONDITION,
    COMPETITION_FAULT_LINE_LOST,
    COMPETITION_FAULT_MOTION,
    COMPETITION_FAULT_TIMEOUT,
    COMPETITION_FAULT_FINISH_MARKER,
    COMPETITION_FAULT_NOT_READY
} CompetitionTaskFault;

typedef struct {
    uint8_t taskNumber;
    void (*init)(void);
    bool (*start)(uint32_t nowMs, uint8_t lineMask);
    void (*update_1ms)(uint32_t nowMs, uint8_t lineMask);
    void (*stop)(void);
    CompetitionTaskState (*get_state)(void);
    CompetitionTaskFault (*get_fault)(void);
    uint32_t (*get_result_ms)(void);
    uint32_t (*get_elapsed_ms)(uint32_t nowMs);
    int32_t (*get_distance_mm)(void);
    int16_t (*get_speed_command)(void);
    int16_t (*get_line_error)(void);
    int16_t (*get_line_correction)(void);
} CompetitionTaskOps;

#endif
