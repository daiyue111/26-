#include "motor.h"

#include "app_config.h"
#include "app_io.h"

#include <stdbool.h>

#define MOTOR_PWM_A_INST TIMA0
#define MOTOR_PWM_B_INST TIMG6

static bool gMotorInitialized;

static uint32_t duty_to_compare(uint16_t duty)
{
    if (duty > PWM_PERIOD_TICKS) {
        duty = PWM_PERIOD_TICKS;
    }
    return MOTOR_PWM_TIMER_PERIOD -
        ((MOTOR_PWM_TIMER_PERIOD * (uint32_t)duty) / PWM_PERIOD_TICKS);
}

static uint16_t abs_duty(int16_t duty)
{
    int32_t value = duty;

    if (value < 0) {
        value = -value;
    }
    if (value > PWM_PERIOD_TICKS) {
        value = PWM_PERIOD_TICKS;
    }
    return (uint16_t)value;
}

static void motor_set_pwm(uint16_t leftDuty, uint16_t rightDuty)
{
    uint32_t leftCompare = duty_to_compare(leftDuty);
    uint32_t rightCompare = duty_to_compare(rightDuty);

    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST, rightCompare,
        DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST, leftCompare,
        DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST, rightCompare,
        DL_TIMER_CC_3_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_B_INST, leftCompare,
        DL_TIMER_CC_0_INDEX);
}

static void motor_set_left_direction(bool forward)
{
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_BIN1_PIN, forward);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_BIN2_PIN, !forward);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_CIN1_PIN, !forward);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_CIN2_PIN, forward);
}

static void motor_set_right_direction(bool forward)
{
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_AIN1_PIN, forward);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_AIN2_PIN, !forward);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_DIN1_PIN, !forward);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_DIN2_PIN, forward);
}

static void motor_pwm_a_init(void)
{
    DL_TimerA_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerA_PWMConfig pwmConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = MOTOR_PWM_TIMER_PERIOD,
        .isTimerWithFourCC = true,
        .startTimer = false,
    };

    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_B_PWMA_IOMUX,
        IOMUX_PINCM25_PF_TIMA0_CCP0);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_B_PWMC_IOMUX,
        IOMUX_PINCM29_PF_TIMA0_CCP2);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_B_PWMD_IOMUX,
        IOMUX_PINCM15_PF_TIMA0_CCP3);

    DL_TimerA_reset(MOTOR_PWM_A_INST);
    DL_TimerA_enablePower(MOTOR_PWM_A_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerA_setClockConfig(MOTOR_PWM_A_INST, &clockConfig);
    DL_TimerA_initPWMMode(MOTOR_PWM_A_INST, &pwmConfig);
    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_A_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_A_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_A_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_A_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_A_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_A_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST,
        MOTOR_PWM_TIMER_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST,
        MOTOR_PWM_TIMER_PERIOD, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_A_INST,
        MOTOR_PWM_TIMER_PERIOD, DL_TIMER_CC_3_INDEX);
    DL_TimerA_enableClock(MOTOR_PWM_A_INST);
    DL_TimerA_setCCPDirection(MOTOR_PWM_A_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT);
    DL_TimerA_startCounter(MOTOR_PWM_A_INST);
}

static void motor_pwm_b_init(void)
{
    DL_TimerG_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerG_PWMConfig pwmConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = MOTOR_PWM_TIMER_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer = false,
    };

    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_B_PWMB_IOMUX,
        IOMUX_PINCM57_PF_TIMG6_CCP0);
    DL_TimerG_reset(MOTOR_PWM_B_INST);
    DL_TimerG_enablePower(MOTOR_PWM_B_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_setClockConfig(MOTOR_PWM_B_INST, &clockConfig);
    DL_TimerG_initPWMMode(MOTOR_PWM_B_INST, &pwmConfig);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_B_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_B_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_B_INST,
        MOTOR_PWM_TIMER_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_enableClock(MOTOR_PWM_B_INST);
    DL_TimerG_setCCPDirection(MOTOR_PWM_B_INST, DL_TIMER_CC0_OUTPUT);
    DL_TimerG_startCounter(MOTOR_PWM_B_INST);
}

void motor_init(void)
{
    motor_pwm_a_init();
    motor_pwm_b_init();
    gMotorInitialized = true;
    motor_safe_stop();
}

void motor_safe_stop(void)
{
    if (gMotorInitialized) {
        motor_set_pwm(0U, 0U);
    }
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_STBY_PIN, false);
}

void motor_active_brake(void)
{
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_BIN1_PIN, true);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_BIN2_PIN, true);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_CIN1_PIN, true);
    app_gpio_write(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_CIN2_PIN, true);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_AIN1_PIN, true);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_AIN2_PIN, true);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_DIN1_PIN, true);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_DIN2_PIN, true);
    motor_set_pwm(PWM_PERIOD_TICKS, PWM_PERIOD_TICKS);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_STBY_PIN, true);
}

void motor_forward(void)
{
    motor_set_left_direction(true);
    motor_set_right_direction(true);
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_STBY_PIN, true);
}

void motor_pwm_run_1ms(uint8_t leftDuty, uint8_t rightDuty)
{
    motor_set_pwm(leftDuty, rightDuty);
}

void motor_set_signed(int16_t leftDuty, int16_t rightDuty)
{
    motor_set_left_direction(leftDuty >= 0);
    motor_set_right_direction(rightDuty >= 0);
    motor_set_pwm(abs_duty(leftDuty), abs_duty(rightDuty));
    app_gpio_write(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_STBY_PIN,
        (leftDuty != 0) || (rightDuty != 0));
}

void motor_test_set_direction(uint8_t step)
{
    motor_forward();
    if (step == 3U) {
        motor_set_right_direction(false);
    } else if (step == 7U) {
        motor_set_left_direction(true);
    }
}

void motor_test_pwm_run_1ms(uint8_t step)
{
    uint8_t channel = (step <= 3U) ? 1U : 2U;

    motor_set_pwm((channel == 2U) ? MOTOR_TEST_DUTY : 0U,
        (channel == 1U) ? MOTOR_TEST_DUTY : 0U);
}
