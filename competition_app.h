#ifndef COMPETITION_APP_H
#define COMPETITION_APP_H

#include "competition_task.h"

void competition_app_init(void);
bool competition_app_select(uint8_t taskNumber);
bool competition_app_start(uint32_t nowMs, uint8_t lineMask);
void competition_app_update_1ms(uint32_t nowMs, uint8_t lineMask);
void competition_app_stop(void);
uint8_t competition_app_get_task(void);
CompetitionTaskState competition_app_get_state(void);
CompetitionTaskFault competition_app_get_fault(void);
uint32_t competition_app_get_result_ms(void);
uint32_t competition_app_get_elapsed_ms(uint32_t nowMs);
int32_t competition_app_get_distance_mm(void);
int16_t competition_app_get_speed_command(void);
int16_t competition_app_get_line_error(void);
int16_t competition_app_get_line_correction(void);

#endif
