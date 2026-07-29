#ifndef IMU_H
#define IMU_H

#include "ti_msp_dl_config.h"

void imu_bus_prepare(void);
bool imu_init_gyro_z(void);
bool imu_read_gyro_z_delta(int32_t *delta);
uint8_t imu_get_error(void);
void imu_heading_reset(void);
bool imu_heading_update(uint32_t sampleMs);
int32_t imu_heading_get_mdeg(void);
int32_t imu_get_last_gyro_delta(void);

#endif
