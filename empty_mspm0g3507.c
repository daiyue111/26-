#include "ti_msp_dl_config.h"
#include "competition_runtime.h"
#include "control_scheduler.h"
#include "imu.h"

int main(void)
{
    SYSCFG_DL_init();
    imu_bus_prepare();
    competition_runtime_init();
    control_scheduler_init();

    while (1) {
        if (control_scheduler_take_1ms()) {
            competition_runtime_update_1ms();
        } else {
            __WFI();
        }
    }
}
