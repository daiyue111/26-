#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#include <stdbool.h>
#include <stdint.h>

#define CALIBRATION_FLAG_STRAIGHT_VALID (1UL << 0)
#define CALIBRATION_FLAG_TURN_VALID (1UL << 1)
#define CALIBRATION_FLAG_OFFSET_VALID (1UL << 2)
#define CALIBRATION_FLAGS_ALL (CALIBRATION_FLAG_STRAIGHT_VALID | \
    CALIBRATION_FLAG_TURN_VALID | CALIBRATION_FLAG_OFFSET_VALID)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    int32_t straightLeftCounts;
    int32_t straightRightCounts;
    uint32_t straightDistanceMm;
    uint32_t countsPerMeter;
    int32_t turnLeftCounts;
    int32_t turnRightCounts;
    int32_t turnHeadingMdeg;
    uint32_t effectiveTrackMm;
    int32_t cornerOffsetLeftCounts;
    int32_t cornerOffsetRightCounts;
    uint32_t cornerOffsetMm;
    uint32_t flags;
    uint32_t checksum;
} ChassisCalibrationRecord;

bool calibration_store_load(ChassisCalibrationRecord *record);
bool calibration_store_save(ChassisCalibrationRecord *record);

#endif
