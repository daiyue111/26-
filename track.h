#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>

uint8_t track_read_raw_mask(void);
uint8_t track_black_mask(uint8_t rawMask);
uint8_t track_active_count(uint8_t mask);
int16_t track_position_error(uint8_t blackMask);

#endif
