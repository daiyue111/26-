#ifndef H_LINE_CONTROL_H
#define H_LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t periodMs;
    uint16_t lostMs;
    bool lineVisible;
    int16_t error;
    int16_t lastVisibleError;
    int16_t filteredErrorX4;
    int16_t previousError;
    int16_t derivative;
    int16_t correction;
    int16_t pendingCorrection;
    uint8_t correctionConfirmMs;
} HLineControl;

void h_line_control_init(HLineControl *control);
void h_line_control_reset(HLineControl *control);
void h_line_control_update_1ms(HLineControl *control, uint8_t blackMask,
    bool curveMode);
void h_line_control_command(const HLineControl *control,
    int16_t forwardSpeedTicks, int16_t steeringFeedforwardTicks);

#endif
