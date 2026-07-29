#include "power_switch.h"

#include "app_config.h"
#include "app_io.h"

#if ((POWER_SWITCH_PWM_PERIOD_MS == 0U) || \
    (POWER_SWITCH_PWM_PERIOD_MS > 255U))
#error "POWER_SWITCH_PWM_PERIOD_MS must be in the range 1..255"
#endif

#if (POWER_SWITCH_MAX_DUTY_PERCENT > 100U)
#error "POWER_SWITCH_MAX_DUTY_PERCENT must not exceed 100"
#endif

#if (POWER_SWITCH_ACTIVE_HIGH > 1U)
#error "POWER_SWITCH_ACTIVE_HIGH must be 0 or 1"
#endif

typedef struct {
    uint32_t activeMs;
    uint32_t pulseDurationMs;
    uint8_t dutyPercent;
    uint8_t pwmPhaseMs;
    bool outputOn;
    bool timedOut;
} PowerSwitchState;

static PowerSwitchState gPowerSwitch;

static void write_output(bool on)
{
#if POWER_SWITCH_ACTIVE_HIGH
    app_gpio_write(GPIO_POWER_SWITCH_PORT,
        GPIO_POWER_SWITCH_CONTROL_PIN, on);
#else
    app_gpio_write(GPIO_POWER_SWITCH_PORT,
        GPIO_POWER_SWITCH_CONTROL_PIN, !on);
#endif
    gPowerSwitch.outputOn = on;
}

void power_switch_init(void)
{
    gPowerSwitch.activeMs = 0U;
    gPowerSwitch.pulseDurationMs = 0U;
    gPowerSwitch.dutyPercent = 0U;
    gPowerSwitch.pwmPhaseMs = 0U;
    gPowerSwitch.outputOn = false;
    gPowerSwitch.timedOut = false;
    write_output(false);
}

void power_switch_set(bool on)
{
    power_switch_set_duty(on ? 100U : 0U);
}

void power_switch_set_duty(uint8_t percent)
{
    if (percent > POWER_SWITCH_MAX_DUTY_PERCENT) {
        percent = POWER_SWITCH_MAX_DUTY_PERCENT;
    }

    if (percent == 0U) {
        power_switch_off();
        return;
    }

    if (gPowerSwitch.timedOut) {
        return;
    }

    if (gPowerSwitch.dutyPercent == 0U) {
        gPowerSwitch.activeMs = 0U;
        gPowerSwitch.pwmPhaseMs = 0U;
    }
    gPowerSwitch.pulseDurationMs = 0U;
    gPowerSwitch.dutyPercent = percent;
}

void power_switch_pulse_ms(uint32_t durationMs)
{
    power_switch_off();
    if (durationMs == 0U) {
        return;
    }

#if POWER_SWITCH_MAX_ON_MS > 0U
    if (durationMs > POWER_SWITCH_MAX_ON_MS) {
        durationMs = POWER_SWITCH_MAX_ON_MS;
    }
#endif

    power_switch_set_duty(100U);
    gPowerSwitch.pulseDurationMs = durationMs;
}

void power_switch_off(void)
{
    gPowerSwitch.activeMs = 0U;
    gPowerSwitch.pulseDurationMs = 0U;
    gPowerSwitch.dutyPercent = 0U;
    gPowerSwitch.pwmPhaseMs = 0U;
    gPowerSwitch.timedOut = false;
    write_output(false);
}

void power_switch_update_1ms(void)
{
    uint16_t onTimeMs;

    if (gPowerSwitch.dutyPercent == 0U) {
        write_output(false);
        return;
    }

    if ((gPowerSwitch.pulseDurationMs > 0U) &&
        (gPowerSwitch.activeMs >= gPowerSwitch.pulseDurationMs)) {
        power_switch_off();
        return;
    }

#if POWER_SWITCH_MAX_ON_MS > 0U
    if (gPowerSwitch.activeMs >= POWER_SWITCH_MAX_ON_MS) {
        gPowerSwitch.dutyPercent = 0U;
        gPowerSwitch.timedOut = true;
        write_output(false);
        return;
    }
#endif

    gPowerSwitch.activeMs++;
    onTimeMs = (POWER_SWITCH_PWM_PERIOD_MS *
        (uint16_t)gPowerSwitch.dutyPercent) / 100U;
    write_output((gPowerSwitch.dutyPercent >= 100U) ||
        (gPowerSwitch.pwmPhaseMs < onTimeMs));

    gPowerSwitch.pwmPhaseMs++;
    if (gPowerSwitch.pwmPhaseMs >= POWER_SWITCH_PWM_PERIOD_MS) {
        gPowerSwitch.pwmPhaseMs = 0U;
    }
}

uint8_t power_switch_get_duty(void)
{
    return gPowerSwitch.dutyPercent;
}

bool power_switch_is_on(void)
{
    return gPowerSwitch.outputOn;
}

bool power_switch_has_timed_out(void)
{
    return gPowerSwitch.timedOut;
}
