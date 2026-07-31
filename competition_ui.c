#include "competition_ui.h"

#include "app_config.h"
#include "app_io.h"
#include "lcd_display.h"

#include <stddef.h>

#define UI_FAULT_LED_CYCLE_MS 3000U
#define UI_FAULT_LED_SLOT_MS 300U
#define UI_FAULT_LED_ON_MS 150U

typedef struct {
    uint16_t pressedMs;
    bool keyLatched;
    bool resultDisplayed;
    uint32_t displayTickMs;
} CompetitionUiState;

static CompetitionUiState gUi;

static uint8_t fault_display_code(CompetitionTaskFault fault)
{
    switch (fault) {
        case COMPETITION_FAULT_LINE_LOST:
            return 1U;
        case COMPETITION_FAULT_MOTION:
            return 2U;
        case COMPETITION_FAULT_TIMEOUT:
            return 3U;
        case COMPETITION_FAULT_FINISH_MARKER:
            return 4U;
        case COMPETITION_FAULT_NONE:
            return 0U;
        default:
            return 5U;
    }
}

static bool fault_led_on(uint32_t nowMs, CompetitionTaskFault fault)
{
    uint8_t pulses = fault_display_code(fault);
    uint16_t phase = (uint16_t)(nowMs % UI_FAULT_LED_CYCLE_MS);

    return (pulses != 0U) &&
        ((phase / UI_FAULT_LED_SLOT_MS) < pulses) &&
        ((phase % UI_FAULT_LED_SLOT_MS) < UI_FAULT_LED_ON_MS);
}

void competition_ui_init(void)
{
    gUi.pressedMs = 0U;
    gUi.keyLatched = false;
    gUi.resultDisplayed = true;
    gUi.displayTickMs = 0U;
#if APP_ENABLE_LCD
    lcd_display_init();
    lcd_display_show_time_ms(0U);
#endif
    app_debug_led_set(false);
    app_gyro_led_startup_test();
}

bool competition_ui_take_start_event_1ms(void)
{
    bool pressed = app_start_key_pressed();

    if (!pressed) {
        gUi.pressedMs = 0U;
        gUi.keyLatched = false;
        return false;
    }
    if (gUi.pressedMs < KEY_DEBOUNCE_MS) {
        gUi.pressedMs++;
    }
    if ((gUi.pressedMs >= KEY_DEBOUNCE_MS) && !gUi.keyLatched) {
        gUi.keyLatched = true;
        gUi.resultDisplayed = false;
        return true;
    }
    return false;
}

void competition_ui_present_1ms(const CompetitionRuntimeStatus *status)
{
    if (status == NULL) {
        return;
    }
    gUi.displayTickMs++;
    app_debug_led_set(status->state == COMPETITION_RUNTIME_RUNNING);
    if (status->state == COMPETITION_RUNTIME_FAULT) {
        app_gyro_led_set(fault_led_on(gUi.displayTickMs, status->fault));
    } else {
        app_gyro_led_set(status->state == COMPETITION_RUNTIME_COMPLETE);
    }
#if APP_ENABLE_LCD
    if ((status->state == COMPETITION_RUNTIME_COMPLETE) &&
        !gUi.resultDisplayed) {
        lcd_display_show_time_ms(status->resultMs);
        gUi.resultDisplayed = true;
    }
#endif
}
