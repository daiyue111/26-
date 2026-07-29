#include "control_scheduler.h"

#include "ti_msp_dl_config.h"

static volatile uint8_t gPendingTicks;
static volatile uint32_t gSystemMs;
static volatile uint32_t gOverrunCount;

#define CONTROL_SCHEDULER_MAX_PENDING 8U

void control_scheduler_init(void)
{
    gPendingTicks = 0U;
    gSystemMs = 0U;
    gOverrunCount = 0U;
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

bool control_scheduler_take_1ms(void)
{
    bool ready;

    __disable_irq();
    ready = gPendingTicks != 0U;
    if (ready) {
        gPendingTicks--;
    }
    __enable_irq();
    return ready;
}

uint32_t control_scheduler_now_ms(void)
{
    return gSystemMs;
}

uint32_t control_scheduler_get_overrun_count(void)
{
    return gOverrunCount;
}

void SysTick_Handler(void)
{
    gSystemMs++;
    if (gPendingTicks < CONTROL_SCHEDULER_MAX_PENDING) {
        gPendingTicks++;
    } else {
        gOverrunCount++;
    }
}
