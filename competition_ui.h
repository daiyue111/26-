#ifndef COMPETITION_UI_H
#define COMPETITION_UI_H

#include "competition_runtime.h"

#include <stdbool.h>

void competition_ui_init(void);
bool competition_ui_take_start_event_1ms(void);
void competition_ui_present_1ms(const CompetitionRuntimeStatus *status);

#endif
