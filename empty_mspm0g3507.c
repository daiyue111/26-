#include "ti_msp_dl_config.h"
#include "car_app.h"
#include "control_scheduler.h"
#include "imu.h"

int main(void)
{
    SYSCFG_DL_init();
    imu_bus_prepare();
    car_app_init();
    control_scheduler_init();

    while (1) {
        if (control_scheduler_take_1ms()) {
            car_app_step();
        } else {
            __WFI();
        }
    }
}
