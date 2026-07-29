#include "track.h"

#include "app_config.h"
#include "app_io.h"

uint8_t track_read_raw_mask(void)
{
    uint8_t mask = 0U;

    if (app_gpio_is_high(GPIO_TRACK_A_PORT, GPIO_TRACK_A_X1_PIN)) {
        mask |= 0x01U;
    }
    if (app_gpio_is_high(GPIO_TRACK_A_PORT, GPIO_TRACK_A_X2_PIN)) {
        mask |= 0x02U;
    }
    if (app_gpio_is_high(GPIO_TRACK_A_PORT, GPIO_TRACK_A_X3_PIN)) {
        mask |= 0x04U;
    }
    if (app_gpio_is_high(GPIO_TRACK_A_PORT, GPIO_TRACK_A_X4_PIN)) {
        mask |= 0x08U;
    }
    if (app_gpio_is_high(GPIO_TRACK_B_PORT, GPIO_TRACK_B_X5_PIN)) {
        mask |= 0x10U;
    }
    if (app_gpio_is_high(GPIO_TRACK_B_PORT, GPIO_TRACK_B_X6_PIN)) {
        mask |= 0x20U;
    }
    if (app_gpio_is_high(GPIO_TRACK_A_PORT, GPIO_TRACK_A_X7_PIN)) {
        mask |= 0x40U;
    }
    if (app_gpio_is_high(GPIO_TRACK_B_PORT, GPIO_TRACK_B_X8_PIN)) {
        mask |= 0x80U;
    }

    return mask;
}

uint8_t track_black_mask(uint8_t rawMask)
{
#if TRACK_BLACK_IS_HIGH
    return rawMask;
#else
    return (uint8_t)~rawMask;
#endif
}

uint8_t track_active_count(uint8_t mask)
{
    uint8_t count = 0U;

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((mask & (uint8_t)(1U << bit)) != 0U) {
            count++;
        }
    }

    return count;
}

int16_t track_position_error(uint8_t blackMask)
{
#if TRACK_X1_IS_LEFT
    static const int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
#else
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
#endif
    int16_t sum = 0;
    uint8_t count = 0U;

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((blackMask & (uint8_t)(1U << bit)) != 0U) {
            sum += weights[bit];
            count++;
        }
    }

    return (count == 0U) ? 0 : (int16_t)(sum / count);
}
