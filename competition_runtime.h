#ifndef COMPETITION_RUNTIME_H
#define COMPETITION_RUNTIME_H

#include "competition_task.h"

#include <stdint.h>

typedef enum {
    COMPETITION_RUNTIME_READY = 0,
    COMPETITION_RUNTIME_RUNNING,
    COMPETITION_RUNTIME_COMPLETE,
    COMPETITION_RUNTIME_FAULT
} CompetitionRuntimeState;

typedef struct {
    CompetitionRuntimeState state;
    CompetitionTaskState taskState;
    CompetitionTaskFault fault;
    uint8_t taskNumber;
    uint8_t lineRawMask;
    uint8_t lineBlackMask;
    uint8_t lineActiveCount;
    uint32_t elapsedMs;
    uint32_t resultMs;
} CompetitionRuntimeStatus;

void competition_runtime_init(void);
void competition_runtime_update_1ms(void);
const CompetitionRuntimeStatus *competition_runtime_get_status(void);

#endif
