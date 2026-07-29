#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t left;
    int32_t right;
} EncoderCounts;

void encoder_init(void);
void encoder_reset(void);
EncoderCounts encoder_get_counts(void);

#endif
