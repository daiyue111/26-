#include "competition_app.h"

#include "task2.h"

#include <stddef.h>

static const CompetitionTaskOps *gSelectedTask;

void competition_app_init(void)
{
    gSelectedTask = task2_get_ops();
    gSelectedTask->init();
}

bool competition_app_select(uint8_t taskNumber)
{
    if ((gSelectedTask != NULL) &&
        (gSelectedTask->get_state() == COMPETITION_TASK_RUNNING)) {
        return false;
    }
    if (taskNumber != 2U) {
        return false;
    }
    gSelectedTask = task2_get_ops();
    gSelectedTask->init();
    return true;
}

bool competition_app_start(uint32_t nowMs, uint8_t lineMask)
{
    return (gSelectedTask != NULL) &&
        gSelectedTask->start(nowMs, lineMask);
}

void competition_app_update_1ms(uint32_t nowMs, uint8_t lineMask)
{
    if (gSelectedTask != NULL) {
        gSelectedTask->update_1ms(nowMs, lineMask);
    }
}

void competition_app_stop(void)
{
    if (gSelectedTask != NULL) {
        gSelectedTask->stop();
    }
}

uint8_t competition_app_get_task(void)
{
    return (gSelectedTask == NULL) ? 0U : gSelectedTask->taskNumber;
}

CompetitionTaskState competition_app_get_state(void)
{
    return (gSelectedTask == NULL) ? COMPETITION_TASK_FAULT :
        gSelectedTask->get_state();
}

CompetitionTaskFault competition_app_get_fault(void)
{
    return (gSelectedTask == NULL) ? COMPETITION_FAULT_NOT_READY :
        gSelectedTask->get_fault();
}

uint32_t competition_app_get_result_ms(void)
{
    return (gSelectedTask == NULL) ? 0U :
        gSelectedTask->get_result_ms();
}

uint32_t competition_app_get_elapsed_ms(uint32_t nowMs)
{
    return (gSelectedTask == NULL) ? 0U :
        gSelectedTask->get_elapsed_ms(nowMs);
}

int32_t competition_app_get_distance_mm(void)
{
    return (gSelectedTask == NULL) ? 0 :
        gSelectedTask->get_distance_mm();
}

int16_t competition_app_get_speed_command(void)
{
    return (gSelectedTask == NULL) ? 0 :
        gSelectedTask->get_speed_command();
}

int16_t competition_app_get_line_error(void)
{
    return (gSelectedTask == NULL) ? 0 :
        gSelectedTask->get_line_error();
}

int16_t competition_app_get_line_correction(void)
{
    return (gSelectedTask == NULL) ? 0 :
        gSelectedTask->get_line_correction();
}
