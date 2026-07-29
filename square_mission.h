#ifndef SQUARE_MISSION_H
#define SQUARE_MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SQUARE_STATE_STOPPED = 0,
    SQUARE_STATE_STRAIGHT,
    SQUARE_STATE_APPROACH_STOP,
    SQUARE_STATE_PRE_TURN,
    SQUARE_STATE_TURN,
    SQUARE_STATE_REACQUIRE_LINE,
    SQUARE_STATE_COMPLETE,
    SQUARE_STATE_FAULT
} SquareMissionState;

void square_mission_init(void);
bool square_mission_start(bool imuReady);
void square_mission_stop(void);
void square_mission_update_1ms(uint8_t blackMask);
SquareMissionState square_mission_get_state(void);
uint8_t square_mission_get_corner_count(void);
int16_t square_mission_get_line_error(void);
int16_t square_mission_get_line_correction(void);
int32_t square_mission_get_distance_error(void);
int32_t square_mission_get_heading_error(void);
uint8_t square_mission_get_fault_code(void);

#endif
