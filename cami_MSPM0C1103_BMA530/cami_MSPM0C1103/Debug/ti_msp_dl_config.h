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
 *  DO NOT EDIT - This file is generated for the MSPM0C110X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0C110X
#define CONFIG_MSPM0C1103

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



#define CPUCLK_FREQ                                                     24000000



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                    (TIMG14)
#define TIMER_0_INST_IRQHandler                                TIMG14_IRQHandler
#define TIMER_0_INST_INT_IRQN                                  (TIMG14_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (495U)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMG8)
#define TIMER_1_INST_IRQHandler                                 TIMG8_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMG8_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                         (23437U)




/* Defines for I2C */
#define I2C_INST                                                            I2C0
#define I2C_INST_IRQHandler                                      I2C0_IRQHandler
#define I2C_INST_INT_IRQN                                          I2C0_INT_IRQn
#define I2C_BUS_SPEED_HZ                                                  400000
#define GPIO_I2C_SDA_PORT                                                  GPIOA
#define GPIO_I2C_SDA_PIN                                           DL_GPIO_PIN_0
#define GPIO_I2C_IOMUX_SDA                                        (IOMUX_PINCM1)
#define GPIO_I2C_IOMUX_SDA_FUNC                         IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_SCL_PORT                                                  GPIOA
#define GPIO_I2C_SCL_PIN                                           DL_GPIO_PIN_1
#define GPIO_I2C_IOMUX_SCL                                        (IOMUX_PINCM2)
#define GPIO_I2C_IOMUX_SCL_FUNC                         IOMUX_PINCM2_PF_I2C0_SCL



/* Port definition for Pin Group LED_RED */
#define LED_RED_PORT                                                     (GPIOA)

/* Defines for PIN_0: GPIOA.27 with pinCMx 28 on package pin 1 */
#define LED_RED_PIN_0_PIN                                       (DL_GPIO_PIN_27)
#define LED_RED_PIN_0_IOMUX                                      (IOMUX_PINCM28)
/* Port definition for Pin Group LED_GREEN */
#define LED_GREEN_PORT                                                   (GPIOA)

/* Defines for PIN_1: GPIOA.24 with pinCMx 25 on package pin 8 */
#define LED_GREEN_PIN_1_PIN                                     (DL_GPIO_PIN_24)
#define LED_GREEN_PIN_1_IOMUX                                    (IOMUX_PINCM25)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_I2C_init(void);



#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
