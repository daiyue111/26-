#ifndef ROD_ACTUATOR_H
#define ROD_ACTUATOR_H

#include <stdint.h>

void rod_actuator_init(void);
void rod_actuator_set_angle_mdeg(int16_t angleMdeg);
int16_t rod_actuator_get_angle_mdeg(void);

/* Override these weak hooks after the final timer and pin are selected. */
void rod_actuator_hw_init(void);
void rod_actuator_hw_write_mdeg(int16_t angleMdeg);

#endif
