#include "chassis_model.h"

#include "app_config.h"
#include "chassis_config.h"

static uint32_t gCountsPerMeter = CHASSIS_ENCODER_COUNTS_PER_METER;
static uint32_t gCornerCenterOffsetMm = CHASSIS_CORNER_CENTER_OFFSET_MM;

static int32_t divide_round_i64(int64_t numerator, int32_t denominator)
{
    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }
    return (int32_t)(numerator / denominator);
}

int32_t chassis_counts_to_um(int32_t counts)
{
    return divide_round_i64((int64_t)counts * 1000000,
        (int32_t)gCountsPerMeter);
}

int32_t chassis_mm_to_counts(int32_t distanceMm)
{
    return divide_round_i64((int64_t)distanceMm * gCountsPerMeter, 1000);
}

int32_t chassis_speed_ticks_to_mmps(int16_t ticksPerControlPeriod)
{
    return divide_round_i64((int64_t)ticksPerControlPeriod * 1000000,
        (int32_t)(gCountsPerMeter * SPEED_CONTROL_PERIOD_MS));
}

int16_t chassis_mmps_to_speed_ticks(int32_t speedMmps)
{
    int32_t ticks = divide_round_i64((int64_t)speedMmps *
        gCountsPerMeter * SPEED_CONTROL_PERIOD_MS, 1000000);

    if (ticks > INT16_MAX) {
        ticks = INT16_MAX;
    } else if (ticks < INT16_MIN) {
        ticks = INT16_MIN;
    }
    return (int16_t)ticks;
}

void chassis_model_set_counts_per_meter(uint32_t countsPerMeter)
{
    if (countsPerMeter != 0U) {
        gCountsPerMeter = countsPerMeter;
    }
}

uint32_t chassis_model_get_counts_per_meter(void)
{
    return gCountsPerMeter;
}

void chassis_model_set_corner_center_offset_mm(uint32_t offsetMm)
{
    gCornerCenterOffsetMm = offsetMm;
}

uint32_t chassis_model_get_corner_center_offset_mm(void)
{
    return gCornerCenterOffsetMm;
}
