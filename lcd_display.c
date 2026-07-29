#include "lcd_display.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define LCD_WIDTH 280U
#define LCD_HEIGHT 240U
#define LCD_X_OFFSET 20U

#define LCD_COLOR_BLACK 0x0000U
#define LCD_COLOR_GREEN 0x07E0U
#define LCD_COLOR_RED 0xF800U
#define LCD_COLOR_GRAY 0x8410U

#define DIGIT_WIDTH 34U
#define DIGIT_HEIGHT 72U
#define DIGIT_THICKNESS 6U
#define DIGIT_Y 84U

enum {
    SEGMENT_A = 1U << 0,
    SEGMENT_B = 1U << 1,
    SEGMENT_C = 1U << 2,
    SEGMENT_D = 1U << 3,
    SEGMENT_E = 1U << 4,
    SEGMENT_F = 1U << 5,
    SEGMENT_G = 1U << 6
};

static const uint8_t gDigitSegments[10] = {
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E |
        SEGMENT_F,
    SEGMENT_B | SEGMENT_C,
    SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,
    SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F |
        SEGMENT_G,
    SEGMENT_A | SEGMENT_B | SEGMENT_C,
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E |
        SEGMENT_F | SEGMENT_G,
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F |
        SEGMENT_G
};

static const uint8_t gPositiveGamma[14] = {
    0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x05U, 0x31U,
    0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
};

static const uint8_t gNegativeGamma[14] = {
    0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x15U, 0x31U,
    0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
};

static void lcd_delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static void lcd_write_byte(uint8_t value)
{
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        DL_GPIO_clearPins(GPIO_LCD_SCLK_PORT, GPIO_LCD_SCLK_PIN);
        delay_cycles(8U);
        if ((value & 0x80U) != 0U) {
            DL_GPIO_setPins(GPIO_LCD_MOSI_PORT, GPIO_LCD_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_LCD_MOSI_PORT, GPIO_LCD_MOSI_PIN);
        }
        delay_cycles(8U);
        DL_GPIO_setPins(GPIO_LCD_SCLK_PORT, GPIO_LCD_SCLK_PIN);
        delay_cycles(8U);
        value <<= 1;
    }
}

static void lcd_write_command(uint8_t command)
{
    DL_GPIO_clearPins(GPIO_LCD_DC_PORT, GPIO_LCD_DC_PIN);
    lcd_write_byte(command);
    DL_GPIO_setPins(GPIO_LCD_DC_PORT, GPIO_LCD_DC_PIN);
}

static void lcd_write_data8(uint8_t data)
{
    lcd_write_byte(data);
}

static void lcd_write_data16(uint16_t data)
{
    lcd_write_byte((uint8_t)(data >> 8));
    lcd_write_byte((uint8_t)data);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1)
{
    lcd_write_command(0x2AU);
    lcd_write_data16((uint16_t)(x0 + LCD_X_OFFSET));
    lcd_write_data16((uint16_t)(x1 + LCD_X_OFFSET));
    lcd_write_command(0x2BU);
    lcd_write_data16(y0);
    lcd_write_data16(y1);
    lcd_write_command(0x2CU);
}

static void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1,
    uint16_t y1, uint16_t color)
{
    uint32_t pixelCount;

    if ((x0 > x1) || (y0 > y1) || (x1 >= LCD_WIDTH) ||
        (y1 >= LCD_HEIGHT)) {
        return;
    }
    lcd_set_window(x0, y0, x1, y1);
    pixelCount = (uint32_t)(x1 - x0 + 1U) * (y1 - y0 + 1U);
    while (pixelCount-- > 0U) {
        lcd_write_data16(color);
    }
}

static void lcd_draw_segment(uint16_t x, uint16_t y, uint8_t segment,
    uint16_t color)
{
    uint16_t half = DIGIT_HEIGHT / 2U;
    uint16_t t = DIGIT_THICKNESS;

    switch (segment) {
        case SEGMENT_A:
            lcd_fill_rect((uint16_t)(x + t), y,
                (uint16_t)(x + DIGIT_WIDTH - t - 1U),
                (uint16_t)(y + t - 1U), color);
            break;
        case SEGMENT_B:
            lcd_fill_rect((uint16_t)(x + DIGIT_WIDTH - t),
                (uint16_t)(y + t),
                (uint16_t)(x + DIGIT_WIDTH - 1U),
                (uint16_t)(y + half - 1U), color);
            break;
        case SEGMENT_C:
            lcd_fill_rect((uint16_t)(x + DIGIT_WIDTH - t),
                (uint16_t)(y + half),
                (uint16_t)(x + DIGIT_WIDTH - 1U),
                (uint16_t)(y + DIGIT_HEIGHT - t - 1U), color);
            break;
        case SEGMENT_D:
            lcd_fill_rect((uint16_t)(x + t),
                (uint16_t)(y + DIGIT_HEIGHT - t),
                (uint16_t)(x + DIGIT_WIDTH - t - 1U),
                (uint16_t)(y + DIGIT_HEIGHT - 1U), color);
            break;
        case SEGMENT_E:
            lcd_fill_rect(x, (uint16_t)(y + half),
                (uint16_t)(x + t - 1U),
                (uint16_t)(y + DIGIT_HEIGHT - t - 1U), color);
            break;
        case SEGMENT_F:
            lcd_fill_rect(x, (uint16_t)(y + t),
                (uint16_t)(x + t - 1U),
                (uint16_t)(y + half - 1U), color);
            break;
        case SEGMENT_G:
            lcd_fill_rect((uint16_t)(x + t),
                (uint16_t)(y + half - t / 2U),
                (uint16_t)(x + DIGIT_WIDTH - t - 1U),
                (uint16_t)(y + half + (t - 1U) / 2U), color);
            break;
        default:
            break;
    }
}

static void lcd_draw_digit(uint16_t x, uint8_t digit, uint16_t color)
{
    uint8_t mask = gDigitSegments[digit % 10U];

    for (uint8_t segment = SEGMENT_A; segment <= SEGMENT_G;
        segment <<= 1) {
        if ((mask & segment) != 0U) {
            lcd_draw_segment(x, DIGIT_Y, segment, color);
        }
    }
}

void lcd_display_init(void)
{
    DL_GPIO_clearPins(GPIO_LCD_SCLK_PORT, GPIO_LCD_SCLK_PIN);
    DL_GPIO_setPins(GPIO_LCD_MOSI_PORT, GPIO_LCD_MOSI_PIN);
    DL_GPIO_setPins(GPIO_LCD_DC_PORT, GPIO_LCD_DC_PIN);
    lcd_delay_ms(200U);

    lcd_write_command(0x01U);
    lcd_delay_ms(150U);
    lcd_write_command(0x11U);
    lcd_delay_ms(120U);
    lcd_write_command(0x36U);
    lcd_write_data8(0xA0U);
    lcd_write_command(0x3AU);
    lcd_write_data8(0x05U);
    lcd_write_command(0xB2U);
    lcd_write_data8(0x0CU);
    lcd_write_data8(0x0CU);
    lcd_write_data8(0x00U);
    lcd_write_data8(0x33U);
    lcd_write_data8(0x33U);
    lcd_write_command(0xB7U);
    lcd_write_data8(0x35U);
    lcd_write_command(0xBBU);
    lcd_write_data8(0x32U);
    lcd_write_command(0xC2U);
    lcd_write_data8(0x01U);
    lcd_write_command(0xC3U);
    lcd_write_data8(0x15U);
    lcd_write_command(0xC4U);
    lcd_write_data8(0x20U);
    lcd_write_command(0xC6U);
    lcd_write_data8(0x0FU);
    lcd_write_command(0xD0U);
    lcd_write_data8(0xA4U);
    lcd_write_data8(0xA1U);
    lcd_write_command(0xE0U);
    for (uint8_t index = 0U; index < sizeof(gPositiveGamma); index++) {
        lcd_write_data8(gPositiveGamma[index]);
    }
    lcd_write_command(0xE1U);
    for (uint8_t index = 0U; index < sizeof(gNegativeGamma); index++) {
        lcd_write_data8(gNegativeGamma[index]);
    }
    lcd_write_command(0x21U);
    lcd_write_command(0x29U);
    lcd_delay_ms(20U);
    lcd_fill_rect(0U, 0U, LCD_WIDTH - 1U, LCD_HEIGHT - 1U,
        LCD_COLOR_BLACK);
}

void lcd_display_show_time_ms(uint32_t elapsedMs)
{
    static const uint16_t digitX[5] = {18U, 60U, 112U, 154U, 196U};
    uint8_t digits[5];
    uint32_t seconds;
    uint32_t milliseconds;
    uint16_t color;

    if (elapsedMs > 99999U) {
        elapsedMs = 99999U;
    }
    seconds = elapsedMs / 1000U;
    milliseconds = elapsedMs % 1000U;
    digits[0] = (uint8_t)(seconds / 10U);
    digits[1] = (uint8_t)(seconds % 10U);
    digits[2] = (uint8_t)(milliseconds / 100U);
    digits[3] = (uint8_t)((milliseconds / 10U) % 10U);
    digits[4] = (uint8_t)(milliseconds % 10U);
    color = (elapsedMs == 0U) ? LCD_COLOR_GRAY :
        ((elapsedMs <= 20000U) ? LCD_COLOR_GREEN : LCD_COLOR_RED);

    lcd_fill_rect(8U, 72U, 237U, 163U, LCD_COLOR_BLACK);
    for (uint8_t index = 0U; index < 5U; index++) {
        lcd_draw_digit(digitX[index], digits[index], color);
    }
    lcd_fill_rect(99U, 146U, 106U, 153U, color);
}
