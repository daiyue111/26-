#include "competition_runtime.h"

#include "ball_control.h"
#include "ball_vision.h"
#include "competition_registry.h"
#include "competition_safety.h"
#include "competition_selector.h"
#include "competition_ui.h"
#include "control_scheduler.h"
#include "motion_control.h"
#include "motor.h"
#include "power_switch.h"
#include "track.h"

#include <stddef.h>

typedef struct {
    const CompetitionTaskOps *task;
    CompetitionRuntimeStatus status;
    bool terminalStopApplied;
} CompetitionRuntimeContext;

static CompetitionRuntimeContext gRuntime;

static bool select_task(uint8_t taskNumber)
{
    const CompetitionTaskOps *task = competition_registry_find(taskNumber);

    if (task == NULL) {
        return false;
    }
    gRuntime.task = task;
    gRuntime.status.taskNumber = taskNumber;
    task->init();
    return true;
}

static void enter_ready(void)
{
    competition_safety_stop_all();
    gRuntime.status.state = COMPETITION_RUNTIME_READY;
    gRuntime.status.taskState = COMPETITION_TASK_STOPPED;
    gRuntime.status.fault = COMPETITION_FAULT_NONE;
    gRuntime.status.elapsedMs = 0U;
    gRuntime.status.resultMs = 0U;
    gRuntime.terminalStopApplied = true;
}

static void start_selected_task(uint32_t nowMs)
{
    uint8_t selected = competition_selector_read();

    if ((gRuntime.task == NULL) ||
        (selected != gRuntime.status.taskNumber)) {
        if (!select_task(selected)) {
            gRuntime.status.state = COMPETITION_RUNTIME_FAULT;
            gRuntime.status.fault = COMPETITION_FAULT_NOT_READY;
            return;
        }
    }
    if (!gRuntime.task->start(nowMs, gRuntime.status.lineBlackMask)) {
        gRuntime.status.taskState = gRuntime.task->get_state();
        gRuntime.status.fault = gRuntime.task->get_fault();
        gRuntime.status.state = COMPETITION_RUNTIME_FAULT;
        gRuntime.terminalStopApplied = false;
        return;
    }
    gRuntime.status.state = COMPETITION_RUNTIME_RUNNING;
    gRuntime.status.taskState = COMPETITION_TASK_RUNNING;
    gRuntime.status.fault = COMPETITION_FAULT_NONE;
    gRuntime.status.elapsedMs = 0U;
    gRuntime.status.resultMs = 0U;
    gRuntime.terminalStopApplied = false;
}

static void update_task_state(uint32_t nowMs)
{
    CompetitionTaskState taskState;

    if (gRuntime.task == NULL) {
        gRuntime.status.state = COMPETITION_RUNTIME_FAULT;
        gRuntime.status.fault = COMPETITION_FAULT_NOT_READY;
        return;
    }
    taskState = gRuntime.task->get_state();
    gRuntime.status.taskState = taskState;
    gRuntime.status.fault = gRuntime.task->get_fault();
    gRuntime.status.elapsedMs = gRuntime.task->get_elapsed_ms(nowMs);
    gRuntime.status.resultMs = gRuntime.task->get_result_ms();
    if (taskState == COMPETITION_TASK_COMPLETE) {
        gRuntime.status.state = COMPETITION_RUNTIME_COMPLETE;
    } else if (taskState == COMPETITION_TASK_FAULT) {
        gRuntime.status.state = COMPETITION_RUNTIME_FAULT;
    }
}

void competition_runtime_init(void)
{
    motor_init();
    power_switch_init();
    motion_control_init();
    ball_vision_init();
    ball_control_init();
    ball_control_set_enabled(false);
    competition_selector_init();
    gRuntime.task = NULL;
    gRuntime.status.taskNumber = 0U;
    gRuntime.status.lineRawMask = 0U;
    gRuntime.status.lineBlackMask = 0U;
    gRuntime.status.lineActiveCount = 0U;
    if (!select_task(competition_selector_read())) {
        gRuntime.status.state = COMPETITION_RUNTIME_FAULT;
        gRuntime.status.taskState = COMPETITION_TASK_FAULT;
        gRuntime.status.fault = COMPETITION_FAULT_NOT_READY;
        gRuntime.status.elapsedMs = 0U;
        gRuntime.status.resultMs = 0U;
        gRuntime.terminalStopApplied = false;
    } else {
        enter_ready();
    }
    competition_ui_init();
}

void competition_runtime_update_1ms(void)
{
    uint32_t nowMs = control_scheduler_now_ms();

    gRuntime.status.lineRawMask = track_read_raw_mask();
    gRuntime.status.lineBlackMask = track_black_mask(
        gRuntime.status.lineRawMask);
    gRuntime.status.lineActiveCount = track_active_count(
        gRuntime.status.lineBlackMask);

    if (competition_ui_take_start_event_1ms()) {
        if (gRuntime.status.state == COMPETITION_RUNTIME_RUNNING) {
            gRuntime.task->stop();
            enter_ready();
        } else {
            start_selected_task(nowMs);
        }
    }
    if (gRuntime.status.state == COMPETITION_RUNTIME_RUNNING) {
        gRuntime.task->update_1ms(nowMs,
            gRuntime.status.lineBlackMask);
        update_task_state(nowMs);
    }
    if (((gRuntime.status.state == COMPETITION_RUNTIME_COMPLETE) ||
            (gRuntime.status.state == COMPETITION_RUNTIME_FAULT)) &&
        !gRuntime.terminalStopApplied) {
        competition_safety_stop_all();
        gRuntime.terminalStopApplied = true;
    }
    competition_safety_update_1ms();
    competition_ui_present_1ms(&gRuntime.status);
}

const CompetitionRuntimeStatus *competition_runtime_get_status(void)
{
    return &gRuntime.status;
}
