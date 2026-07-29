#include "rod_actuator.h"

#include "app_config.h"

static int16_t gCommandMdeg;

__attribute__((weak)) void rod_actuator_hw_init(void)
{
}

__attribute__((weak)) void rod_actuator_hw_write_mdeg(int16_t angleMdeg)
{
    (void)angleMdeg;
}

void rod_actuator_init(void)
{
    gCommandMdeg = H_ROD_NEUTRAL_MDEG;
    rod_actuator_hw_init();
    rod_actuator_hw_write_mdeg(gCommandMdeg);
}

void rod_actuator_set_angle_mdeg(int16_t angleMdeg)
{
    if (angleMdeg > H_ROD_MAX_MDEG) {
        angleMdeg = H_ROD_MAX_MDEG;
    } else if (angleMdeg < H_ROD_MIN_MDEG) {
        angleMdeg = H_ROD_MIN_MDEG;
    }
    gCommandMdeg = angleMdeg;
    rod_actuator_hw_write_mdeg(angleMdeg);
}

int16_t rod_actuator_get_angle_mdeg(void)
{
    return gCommandMdeg;
}
