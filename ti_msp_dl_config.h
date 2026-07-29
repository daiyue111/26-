/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000




/* Port definition for Pin Group GPIO_DEBUG_LED */
#define GPIO_DEBUG_LED_PORT                                              (GPIOB)

/* Defines for LED: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_DEBUG_LED_LED_PIN                                  (DL_GPIO_PIN_18)
#define GPIO_DEBUG_LED_LED_IOMUX                                 (IOMUX_PINCM44)
/* Port definition for Pin Group GPIO_GYRO_LED */
#define GPIO_GYRO_LED_PORT                                               (GPIOA)

/* Defines for STATUS_LED: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_GYRO_LED_STATUS_LED_PIN                             (DL_GPIO_PIN_0)
#define GPIO_GYRO_LED_STATUS_LED_IOMUX                            (IOMUX_PINCM1)
/* Port definition for Pin Group GPIO_START_KEY */
#define GPIO_START_KEY_PORT                                              (GPIOA)

/* Defines for KEY: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_START_KEY_KEY_PIN                                   (DL_GPIO_PIN_9)
#define GPIO_START_KEY_KEY_IOMUX                                 (IOMUX_PINCM20)
/* Port definition for Pin Group GPIO_POWER_SWITCH */
#define GPIO_POWER_SWITCH_PORT                                           (GPIOB)

/* Defines for CONTROL: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_POWER_SWITCH_CONTROL_PIN                           (DL_GPIO_PIN_16)
#define GPIO_POWER_SWITCH_CONTROL_IOMUX                          (IOMUX_PINCM33)
/* Port definition for Pin Group GPIO_TRACK_A */
#define GPIO_TRACK_A_PORT                                                (GPIOA)

/* Defines for X1: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_TRACK_A_X1_PIN                                     (DL_GPIO_PIN_26)
#define GPIO_TRACK_A_X1_IOMUX                                    (IOMUX_PINCM59)
/* Defines for X2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_TRACK_A_X2_PIN                                     (DL_GPIO_PIN_27)
#define GPIO_TRACK_A_X2_IOMUX                                    (IOMUX_PINCM60)
/* Defines for X3: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_TRACK_A_X3_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_TRACK_A_X3_IOMUX                                    (IOMUX_PINCM54)
/* Defines for X4: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_TRACK_A_X4_PIN                                     (DL_GPIO_PIN_25)
#define GPIO_TRACK_A_X4_IOMUX                                    (IOMUX_PINCM55)
/* Defines for X7: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_TRACK_A_X7_PIN                                     (DL_GPIO_PIN_22)
#define GPIO_TRACK_A_X7_IOMUX                                    (IOMUX_PINCM47)
/* Port definition for Pin Group GPIO_TRACK_B */
#define GPIO_TRACK_B_PORT                                                (GPIOB)

/* Defines for X5: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_TRACK_B_X5_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_TRACK_B_X5_IOMUX                                    (IOMUX_PINCM52)
/* Defines for X6: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_TRACK_B_X6_PIN                                     (DL_GPIO_PIN_25)
#define GPIO_TRACK_B_X6_IOMUX                                    (IOMUX_PINCM56)
/* Defines for X8: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_TRACK_B_X8_PIN                                     (DL_GPIO_PIN_20)
#define GPIO_TRACK_B_X8_IOMUX                                    (IOMUX_PINCM48)
/* Defines for LEFT_A: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_ENCODER_LEFT_A_PORT                                         (GPIOB)
// pins affected by this interrupt request:["LEFT_A"]
#define GPIO_ENCODER_GPIOB_INT_IRQN                             (GPIOB_INT_IRQn)
#define GPIO_ENCODER_GPIOB_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_LEFT_A_IIDX                            (DL_GPIO_IIDX_DIO23)
#define GPIO_ENCODER_LEFT_A_PIN                                 (DL_GPIO_PIN_23)
#define GPIO_ENCODER_LEFT_A_IOMUX                                (IOMUX_PINCM51)
/* Defines for LEFT_B: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_ENCODER_LEFT_B_PORT                                         (GPIOB)
#define GPIO_ENCODER_LEFT_B_PIN                                 (DL_GPIO_PIN_27)
#define GPIO_ENCODER_LEFT_B_IOMUX                                (IOMUX_PINCM58)
/* Defines for RIGHT_A: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_ENCODER_RIGHT_A_PORT                                        (GPIOA)
// pins affected by this interrupt request:["RIGHT_A"]
#define GPIO_ENCODER_GPIOA_INT_IRQN                             (GPIOA_INT_IRQn)
#define GPIO_ENCODER_GPIOA_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_RIGHT_A_IIDX                           (DL_GPIO_IIDX_DIO21)
#define GPIO_ENCODER_RIGHT_A_PIN                                (DL_GPIO_PIN_21)
#define GPIO_ENCODER_RIGHT_A_IOMUX                               (IOMUX_PINCM46)
/* Defines for RIGHT_B: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_ENCODER_RIGHT_B_PORT                                        (GPIOB)
#define GPIO_ENCODER_RIGHT_B_PIN                                (DL_GPIO_PIN_22)
#define GPIO_ENCODER_RIGHT_B_IOMUX                               (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_MOTOR_A */
#define GPIO_MOTOR_A_PORT                                                (GPIOA)

/* Defines for BIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_MOTOR_A_BIN1_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_MOTOR_A_BIN1_IOMUX                                  (IOMUX_PINCM35)
/* Defines for BIN2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_MOTOR_A_BIN2_PIN                                   (DL_GPIO_PIN_12)
#define GPIO_MOTOR_A_BIN2_IOMUX                                  (IOMUX_PINCM34)
/* Defines for CIN1: GPIOA.10 with pinCMx 21 on package pin 56 */
#define GPIO_MOTOR_A_CIN1_PIN                                   (DL_GPIO_PIN_10)
#define GPIO_MOTOR_A_CIN1_IOMUX                                  (IOMUX_PINCM21)
/* Defines for CIN2: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_MOTOR_A_CIN2_PIN                                   (DL_GPIO_PIN_11)
#define GPIO_MOTOR_A_CIN2_IOMUX                                  (IOMUX_PINCM22)
/* Port definition for Pin Group GPIO_MOTOR_B */
#define GPIO_MOTOR_B_PORT                                                (GPIOB)

/* Defines for STBY: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_MOTOR_B_STBY_PIN                                    (DL_GPIO_PIN_7)
#define GPIO_MOTOR_B_STBY_IOMUX                                  (IOMUX_PINCM24)
/* Defines for AIN1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_B_AIN1_PIN                                    (DL_GPIO_PIN_6)
#define GPIO_MOTOR_B_AIN1_IOMUX                                  (IOMUX_PINCM23)
/* Defines for AIN2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_MOTOR_B_AIN2_PIN                                    (DL_GPIO_PIN_9)
#define GPIO_MOTOR_B_AIN2_IOMUX                                  (IOMUX_PINCM26)
/* Defines for PWMA: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_MOTOR_B_PWMA_PIN                                    (DL_GPIO_PIN_8)
#define GPIO_MOTOR_B_PWMA_IOMUX                                  (IOMUX_PINCM25)
/* Defines for PWMB: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_MOTOR_B_PWMB_PIN                                   (DL_GPIO_PIN_26)
#define GPIO_MOTOR_B_PWMB_IOMUX                                  (IOMUX_PINCM57)
/* Defines for PWMC: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_MOTOR_B_PWMC_PIN                                   (DL_GPIO_PIN_12)
#define GPIO_MOTOR_B_PWMC_IOMUX                                  (IOMUX_PINCM29)
/* Defines for DIN1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_MOTOR_B_DIN1_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_MOTOR_B_DIN1_IOMUX                                  (IOMUX_PINCM30)
/* Defines for DIN2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_MOTOR_B_DIN2_PIN                                    (DL_GPIO_PIN_3)
#define GPIO_MOTOR_B_DIN2_IOMUX                                  (IOMUX_PINCM16)
/* Defines for PWMD: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_MOTOR_B_PWMD_PIN                                    (DL_GPIO_PIN_2)
#define GPIO_MOTOR_B_PWMD_IOMUX                                  (IOMUX_PINCM15)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);



#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
