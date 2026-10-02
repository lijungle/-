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



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_1
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM13)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM13_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMG7
#define PWM_1_INST_IRQHandler                                   TIMG7_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMG7_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOA
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_23
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM53)
#define GPIO_PWM_1_C0_IOMUX_FUNC                     IOMUX_PINCM53_PF_TIMG7_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOA
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_18
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM40)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM40_PF_TIMG7_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for CONTROL_TIMER */
#define CONTROL_TIMER_INST                                               (TIMG0)
#define CONTROL_TIMER_INST_IRQHandler                           TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN                             (TIMG0_INT_IRQn)
#define CONTROL_TIMER_INST_LOAD_VALUE                                    (1249U)




/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C0
#define OLED_I2C_INST_IRQHandler                                 I2C0_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C0_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             100000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOA
#define GPIO_OLED_I2C_SDA_PIN                                     DL_GPIO_PIN_28
#define GPIO_OLED_I2C_IOMUX_SDA                                   (IOMUX_PINCM3)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                    IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOA
#define GPIO_OLED_I2C_SCL_PIN                                     DL_GPIO_PIN_31
#define GPIO_OLED_I2C_IOMUX_SCL                                   (IOMUX_PINCM6)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                    IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for JY61P_UART */
#define JY61P_UART_INST                                                    UART3
#define JY61P_UART_INST_FREQUENCY                                       32000000
#define JY61P_UART_INST_IRQHandler                              UART3_IRQHandler
#define JY61P_UART_INST_INT_IRQN                                  UART3_INT_IRQn
#define GPIO_JY61P_UART_RX_PORT                                            GPIOB
#define GPIO_JY61P_UART_TX_PORT                                            GPIOB
#define GPIO_JY61P_UART_RX_PIN                                     DL_GPIO_PIN_3
#define GPIO_JY61P_UART_TX_PIN                                     DL_GPIO_PIN_2
#define GPIO_JY61P_UART_IOMUX_RX                                 (IOMUX_PINCM16)
#define GPIO_JY61P_UART_IOMUX_TX                                 (IOMUX_PINCM15)
#define GPIO_JY61P_UART_IOMUX_RX_FUNC                  IOMUX_PINCM16_PF_UART3_RX
#define GPIO_JY61P_UART_IOMUX_TX_FUNC                  IOMUX_PINCM15_PF_UART3_TX
#define JY61P_UART_BAUD_RATE                                            (115200)
#define JY61P_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define JY61P_UART_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group KEY1 */
#define KEY1_PORT                                                        (GPIOA)

/* Defines for KEY1_PIN: GPIOA.26 with pinCMx 59 on package pin 30 */
#define KEY1_KEY1_PIN_PIN                                       (DL_GPIO_PIN_26)
#define KEY1_KEY1_PIN_IOMUX                                      (IOMUX_PINCM59)
/* Port definition for Pin Group KEY2 */
#define KEY2_PORT                                                        (GPIOA)

/* Defines for KEY2_PIN: GPIOA.25 with pinCMx 55 on package pin 26 */
#define KEY2_KEY2_PIN_PIN                                       (DL_GPIO_PIN_25)
#define KEY2_KEY2_PIN_IOMUX                                      (IOMUX_PINCM55)
/* Port definition for Pin Group KEY3 */
#define KEY3_PORT                                                        (GPIOA)

/* Defines for KEY3_PIN: GPIOA.12 with pinCMx 34 on package pin 5 */
#define KEY3_KEY3_PIN_PIN                                       (DL_GPIO_PIN_12)
#define KEY3_KEY3_PIN_IOMUX                                      (IOMUX_PINCM34)
/* Port definition for Pin Group ENC_LEFT_A */
#define ENC_LEFT_A_PORT                                                  (GPIOA)

/* Defines for ENC_LEFT_A_PIN: GPIOA.17 with pinCMx 39 on package pin 10 */
// groups represented: ["ENC_RIGHT_A","ENC_LEFT_A"]
// pins affected: ["ENC_RIGHT_A_PIN","ENC_LEFT_A_PIN"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENC_LEFT_A_ENC_LEFT_A_PIN_IIDX                      (DL_GPIO_IIDX_DIO17)
#define ENC_LEFT_A_ENC_LEFT_A_PIN_PIN                           (DL_GPIO_PIN_17)
#define ENC_LEFT_A_ENC_LEFT_A_PIN_IOMUX                          (IOMUX_PINCM39)
/* Port definition for Pin Group ENC_LEFT_B */
#define ENC_LEFT_B_PORT                                                  (GPIOA)

/* Defines for ENC_LEFT_B_PIN: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENC_LEFT_B_ENC_LEFT_B_PIN_PIN                           (DL_GPIO_PIN_24)
#define ENC_LEFT_B_ENC_LEFT_B_PIN_IOMUX                          (IOMUX_PINCM54)
/* Port definition for Pin Group ENC_RIGHT_A */
#define ENC_RIGHT_A_PORT                                                 (GPIOA)

/* Defines for ENC_RIGHT_A_PIN: GPIOA.15 with pinCMx 37 on package pin 8 */
#define ENC_RIGHT_A_ENC_RIGHT_A_PIN_IIDX                    (DL_GPIO_IIDX_DIO15)
#define ENC_RIGHT_A_ENC_RIGHT_A_PIN_PIN                         (DL_GPIO_PIN_15)
#define ENC_RIGHT_A_ENC_RIGHT_A_PIN_IOMUX                        (IOMUX_PINCM37)
/* Port definition for Pin Group ENC_RIGHT_B */
#define ENC_RIGHT_B_PORT                                                 (GPIOA)

/* Defines for ENC_RIGHT_B_PIN: GPIOA.16 with pinCMx 38 on package pin 9 */
#define ENC_RIGHT_B_ENC_RIGHT_B_PIN_PIN                         (DL_GPIO_PIN_16)
#define ENC_RIGHT_B_ENC_RIGHT_B_PIN_IOMUX                        (IOMUX_PINCM38)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_CONTROL_TIMER_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_JY61P_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
