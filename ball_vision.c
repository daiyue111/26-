#include "ball_vision.h"

#include <stddef.h>

#define BALL_PACKET_SYNC_0 0xAAU
#define BALL_PACKET_SYNC_1 0x55U
#define BALL_PACKET_LENGTH 7U

typedef struct {
    uint8_t data[BALL_PACKET_LENGTH];
    uint8_t index;
    volatile uint8_t sequence;
    volatile int16_t positionTenthMm;
    volatile uint8_t confidence;
    volatile uint8_t frameId;
    volatile uint32_t timestampMs;
    volatile bool valid;
    volatile uint32_t goodFrames;
    volatile uint32_t crcErrors;
} BallVisionState;

static BallVisionState gVision;

static uint8_t crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    for (uint8_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U) :
                (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

void ball_vision_init(void)
{
    gVision.index = 0U;
    gVision.sequence = 0U;
    gVision.positionTenthMm = 0;
    gVision.confidence = 0U;
    gVision.frameId = 0U;
    gVision.timestampMs = 0U;
    gVision.valid = false;
    gVision.goodFrames = 0U;
    gVision.crcErrors = 0U;
}

void ball_vision_inject_sample(int16_t positionTenthMm,
    uint8_t confidence, uint8_t frameId, uint32_t nowMs)
{
    gVision.sequence++;
    gVision.positionTenthMm = positionTenthMm;
    gVision.confidence = confidence;
    gVision.frameId = frameId;
    gVision.timestampMs = nowMs;
    gVision.valid = true;
    gVision.goodFrames++;
    gVision.sequence++;
}

void ball_vision_feed_byte(uint8_t byte, uint32_t nowMs)
{
    if ((gVision.index == 0U) && (byte != BALL_PACKET_SYNC_0)) {
        return;
    }
    if ((gVision.index == 1U) && (byte != BALL_PACKET_SYNC_1)) {
        gVision.index = (byte == BALL_PACKET_SYNC_0) ? 1U : 0U;
        return;
    }

    gVision.data[gVision.index++] = byte;
    if (gVision.index < BALL_PACKET_LENGTH) {
        return;
    }
    gVision.index = 0U;

    if (crc8(&gVision.data[2], 4U) != gVision.data[6]) {
        gVision.crcErrors++;
        return;
    }

    ball_vision_inject_sample((int16_t)((uint16_t)gVision.data[2] |
        ((uint16_t)gVision.data[3] << 8U)), gVision.data[4],
        gVision.data[5], nowMs);
}

bool ball_vision_get_sample(BallVisionSample *sample)
{
    uint8_t before;
    uint8_t after;

    if (sample == NULL) {
        return false;
    }

    while (true) {
        before = gVision.sequence;
        if ((before & 1U) != 0U) {
            continue;
        }
        sample->positionTenthMm = gVision.positionTenthMm;
        sample->confidence = gVision.confidence;
        sample->frameId = gVision.frameId;
        sample->timestampMs = gVision.timestampMs;
        sample->valid = gVision.valid;
        after = gVision.sequence;
        if ((before == after) && ((after & 1U) == 0U)) {
            break;
        }
    }

    return sample->valid;
}

uint32_t ball_vision_get_good_frame_count(void)
{
    return gVision.goodFrames;
}

uint32_t ball_vision_get_crc_error_count(void)
{
    return gVision.crcErrors;
}
