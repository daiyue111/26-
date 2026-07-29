#ifndef LINE_SPEED_CONTROL_H
#define LINE_SPEED_CONTROL_H

#include "pid.h"

#include <stdint.h>

typedef struct {
    uint8_t periodMs;
    uint8_t centerLostCycles;
    int16_t error;
    int16_t filteredErrorX4;
    int16_t correction;
    PidController pid;
} LineSpeedController;

void line_speed_control_init(LineSpeedController *controller);
void line_speed_control_reset(LineSpeedController *controller);
void line_speed_control_update_1ms(LineSpeedController *controller,
    uint8_t blackMask);
void line_speed_control_command(const LineSpeedController *controller,
    int16_t forwardSpeed);

#endif
