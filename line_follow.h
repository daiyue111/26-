#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t lastError;
    bool lineWasLost;
    bool imuReady;
    bool imuReadOk;
    int32_t lostHeading;
    uint16_t imuFailMs;
    int16_t error;
    int16_t derivative;
    uint8_t leftDuty;
    uint8_t rightDuty;
    int32_t gyroZDelta;
    int32_t gyroLostHeading;
    int16_t gyroCorrectionTicks;
} LineFollowState;

void line_follow_reset(LineFollowState *state, bool imuReady);
void line_follow_run_1ms(LineFollowState *state, uint8_t blackMask);

#endif
