#ifndef CHASSIS_MODEL_H
#define CHASSIS_MODEL_H

#include <stdint.h>

int32_t chassis_counts_to_um(int32_t counts);
int32_t chassis_mm_to_counts(int32_t distanceMm);
int32_t chassis_speed_ticks_to_mmps(int16_t ticksPerControlPeriod);
int16_t chassis_mmps_to_speed_ticks(int32_t speedMmps);
void chassis_model_set_counts_per_meter(uint32_t countsPerMeter);
uint32_t chassis_model_get_counts_per_meter(void);
void chassis_model_set_corner_center_offset_mm(uint32_t offsetMm);
uint32_t chassis_model_get_corner_center_offset_mm(void);

#endif
