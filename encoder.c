#include "encoder.h"

#include "app_config.h"
#include "app_io.h"
#include "ti_msp_dl_config.h"

static volatile int32_t gLeftCount;
static volatile int32_t gRightCount;

static void encoder_update_left(void)
{
    bool phaseA = app_gpio_is_high(GPIO_ENCODER_LEFT_A_PORT,
        GPIO_ENCODER_LEFT_A_PIN);
    bool phaseB = app_gpio_is_high(GPIO_ENCODER_LEFT_B_PORT,
        GPIO_ENCODER_LEFT_B_PIN);
    int32_t delta = (phaseA == phaseB) ? -1 : 1;

    gLeftCount += delta * ENCODER_LEFT_SIGN;
}

static void encoder_update_right(void)
{
    bool phaseA = app_gpio_is_high(GPIO_ENCODER_RIGHT_A_PORT,
        GPIO_ENCODER_RIGHT_A_PIN);
    bool phaseB = app_gpio_is_high(GPIO_ENCODER_RIGHT_B_PORT,
        GPIO_ENCODER_RIGHT_B_PIN);
    int32_t delta = (phaseA == phaseB) ? -1 : 1;

    gRightCount += delta * ENCODER_RIGHT_SIGN;
}

void encoder_init(void)
{
    encoder_reset();
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_LEFT_A_PORT,
        GPIO_ENCODER_LEFT_A_PIN);
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_A_PORT,
        GPIO_ENCODER_RIGHT_A_PIN);
    NVIC_EnableIRQ(GPIO_ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_GPIOB_INT_IRQN);
}

void encoder_reset(void)
{
    __disable_irq();
    gLeftCount = 0;
    gRightCount = 0;
    __enable_irq();
}

EncoderCounts encoder_get_counts(void)
{
    EncoderCounts counts;

    __disable_irq();
    counts.left = gLeftCount;
    counts.right = gRightCount;
    __enable_irq();
    return counts;
}

void GROUP1_IRQHandler(void)
{
    uint32_t leftStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_LEFT_A_PORT, GPIO_ENCODER_LEFT_A_PIN);
    uint32_t rightStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_RIGHT_A_PORT, GPIO_ENCODER_RIGHT_A_PIN);

    if ((leftStatus & GPIO_ENCODER_LEFT_A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_LEFT_A_PORT,
            GPIO_ENCODER_LEFT_A_PIN);
        encoder_update_left();
    }
    if ((rightStatus & GPIO_ENCODER_RIGHT_A_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_RIGHT_A_PORT,
            GPIO_ENCODER_RIGHT_A_PIN);
        encoder_update_right();
    }
}
