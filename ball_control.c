#include "ball_control.h"

#include "app_config.h"
#include "ball_vision.h"
#include "rod_actuator.h"

#include <limits.h>

typedef struct {
    bool enabled;
    bool visionHealthy;
    bool filterReady;
    uint8_t periodMs;
    uint8_t lastFrameId;
    uint32_t lastFrameTimestampMs;
    int16_t targetTenthMm;
    int32_t positionTenthMm;
    int32_t velocityTenthMmps;
    int32_t integralTenthMmMs;
    int16_t errorTenthMm;
    int16_t angleCommandMdeg;
} BallControlState;

static BallControlState gBall;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void update_filter(const BallVisionSample *sample)
{
    uint32_t dtMs;
    int32_t predicted;
    int32_t residual;

    if (!gBall.filterReady) {
        gBall.positionTenthMm = sample->positionTenthMm;
        gBall.velocityTenthMmps = 0;
        gBall.filterReady = true;
        gBall.lastFrameId = sample->frameId;
        gBall.lastFrameTimestampMs = sample->timestampMs;
        return;
    }
    if ((sample->frameId == gBall.lastFrameId) &&
        (sample->timestampMs == gBall.lastFrameTimestampMs)) {
        return;
    }

    dtMs = sample->timestampMs - gBall.lastFrameTimestampMs;
    if ((dtMs == 0U) || (dtMs > 100U)) {
        dtMs = 20U;
    }
    predicted = gBall.positionTenthMm +
        ((gBall.velocityTenthMmps * (int32_t)dtMs) / 1000);
    residual = (int32_t)sample->positionTenthMm - predicted;
    gBall.positionTenthMm = predicted + ((residual * 3) / 4);
    gBall.velocityTenthMmps += (residual * 1000) /
        ((int32_t)dtMs * 8);
    gBall.velocityTenthMmps = clamp_i32(gBall.velocityTenthMmps,
        -5000, 5000);
    gBall.lastFrameId = sample->frameId;
    gBall.lastFrameTimestampMs = sample->timestampMs;
}

void ball_control_init(void)
{
    gBall.enabled = false;
    gBall.visionHealthy = false;
    gBall.filterReady = false;
    gBall.periodMs = 0U;
    gBall.lastFrameId = 0U;
    gBall.lastFrameTimestampMs = 0U;
    gBall.targetTenthMm = 0;
    gBall.positionTenthMm = 0;
    gBall.velocityTenthMmps = 0;
    gBall.integralTenthMmMs = 0;
    gBall.errorTenthMm = 0;
    gBall.angleCommandMdeg = H_ROD_NEUTRAL_MDEG;
    rod_actuator_init();
}

void ball_control_set_enabled(bool enabled)
{
    gBall.enabled = enabled;
    gBall.integralTenthMmMs = 0;
    if (!enabled) {
        gBall.angleCommandMdeg = H_ROD_NEUTRAL_MDEG;
        rod_actuator_set_angle_mdeg(gBall.angleCommandMdeg);
    }
}

void ball_control_set_target_tenth_mm(int16_t targetTenthMm)
{
    gBall.targetTenthMm = (int16_t)clamp_i32(targetTenthMm,
        -H_BALL_TARGET_LIMIT_TENTH_MM, H_BALL_TARGET_LIMIT_TENTH_MM);
    gBall.errorTenthMm = (int16_t)clamp_i32(
        (int32_t)gBall.targetTenthMm - gBall.positionTenthMm,
        INT16_MIN, INT16_MAX);
    gBall.integralTenthMmMs = 0;
}

void ball_control_update_1ms(uint32_t nowMs, int32_t chassisAccelMmps2)
{
    BallVisionSample sample;
    uint32_t ageMs = UINT32_MAX;
    int32_t outputMdeg;
    int32_t pTerm;
    int32_t dTerm;
    int32_t iTerm;
    int32_t ffTerm;

    if (ball_vision_get_sample(&sample)) {
        ageMs = nowMs - sample.timestampMs;
        gBall.visionHealthy = (sample.confidence >= H_BALL_MIN_CONFIDENCE) &&
            (ageMs <= H_BALL_VISION_TIMEOUT_MS);
        if (gBall.visionHealthy) {
            update_filter(&sample);
        }
    } else {
        gBall.visionHealthy = false;
    }

    gBall.periodMs++;
    if (gBall.periodMs < H_BALL_CONTROL_PERIOD_MS) {
        return;
    }
    gBall.periodMs = 0U;

    if (!gBall.enabled || !gBall.visionHealthy || !gBall.filterReady) {
        gBall.integralTenthMmMs = 0;
        gBall.angleCommandMdeg = H_ROD_NEUTRAL_MDEG;
        rod_actuator_set_angle_mdeg(gBall.angleCommandMdeg);
        return;
    }

    gBall.errorTenthMm = (int16_t)clamp_i32(
        (int32_t)gBall.targetTenthMm - gBall.positionTenthMm,
        INT16_MIN, INT16_MAX);
    gBall.integralTenthMmMs = clamp_i32(gBall.integralTenthMmMs +
        ((int32_t)gBall.errorTenthMm * H_BALL_CONTROL_PERIOD_MS),
        -H_BALL_INTEGRAL_LIMIT_TENTH_MM_MS,
        H_BALL_INTEGRAL_LIMIT_TENTH_MM_MS);

    pTerm = ((int32_t)gBall.errorTenthMm * H_BALL_KP_MDEG_PER_MM) / 10;
    dTerm = (gBall.velocityTenthMmps * H_BALL_KD_MDEG_PER_MMPS) / 10;
    iTerm = (gBall.integralTenthMmMs * H_BALL_KI_MDEG_PER_MM_S) /
        10000;
    ffTerm = chassisAccelMmps2 * H_BALL_ACCEL_FF_MDEG_PER_MMPS2;
    outputMdeg = H_ROD_NEUTRAL_MDEG + (H_BALL_CONTROL_SIGN *
        (pTerm - dTerm + iTerm + ffTerm));
    outputMdeg = clamp_i32(outputMdeg, H_ROD_MIN_MDEG,
        H_ROD_MAX_MDEG);
    gBall.angleCommandMdeg = (int16_t)outputMdeg;
    rod_actuator_set_angle_mdeg(gBall.angleCommandMdeg);
}

bool ball_control_is_enabled(void)
{
    return gBall.enabled;
}

bool ball_control_is_vision_healthy(void)
{
    return gBall.visionHealthy;
}

int16_t ball_control_get_target_tenth_mm(void)
{
    return gBall.targetTenthMm;
}

int16_t ball_control_get_position_tenth_mm(void)
{
    return (int16_t)clamp_i32(gBall.positionTenthMm, INT16_MIN, INT16_MAX);
}

int16_t ball_control_get_velocity_tenth_mmps(void)
{
    return (int16_t)clamp_i32(gBall.velocityTenthMmps,
        INT16_MIN, INT16_MAX);
}

int16_t ball_control_get_error_tenth_mm(void)
{
    return gBall.errorTenthMm;
}

int16_t ball_control_get_angle_command_mdeg(void)
{
    return gBall.angleCommandMdeg;
}

uint32_t ball_control_get_vision_age_ms(uint32_t nowMs)
{
    BallVisionSample sample;

    return ball_vision_get_sample(&sample) ? (nowMs - sample.timestampMs) :
        UINT32_MAX;
}
