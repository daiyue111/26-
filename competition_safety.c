#include "competition_safety.h"

#include "ball_control.h"
#include "motion_control.h"
#include "motor.h"
#include "power_switch.h"

void competition_safety_stop_all(void)
{
    motion_control_enable(false);
    motor_safe_stop();
    ball_control_set_enabled(false);
    power_switch_off();
}

void competition_safety_init(void)
{
    competition_safety_stop_all();
}

void competition_safety_update_1ms(void)
{
    power_switch_off();
    power_switch_update_1ms();
}
