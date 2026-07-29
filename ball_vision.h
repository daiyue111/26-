#ifndef BALL_VISION_H
#define BALL_VISION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t positionTenthMm;
    uint8_t confidence;
    uint8_t frameId;
    uint32_t timestampMs;
    bool valid;
} BallVisionSample;

void ball_vision_init(void);
void ball_vision_feed_byte(uint8_t byte, uint32_t nowMs);
bool ball_vision_get_sample(BallVisionSample *sample);
void ball_vision_inject_sample(int16_t positionTenthMm,
    uint8_t confidence, uint8_t frameId, uint32_t nowMs);
uint32_t ball_vision_get_good_frame_count(void);
uint32_t ball_vision_get_crc_error_count(void);

#endif
