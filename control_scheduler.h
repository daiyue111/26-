#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

void control_scheduler_init(void);
bool control_scheduler_take_1ms(void);
uint32_t control_scheduler_now_ms(void);
uint32_t control_scheduler_get_overrun_count(void);

#endif
