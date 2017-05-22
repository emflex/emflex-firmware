/*****************************************************************************
* File:        bsp.h
*
* Created on:  Dec 27, 2015
*
* Description: platform specific routines for gtrack PCB
*
* Author:      rostokus
******************************************************************************

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
******************************************************************************
 */

#ifndef BSP
#define BSP

#include "common.h"

extern RV_t bspInit(void);
extern RV_t bspInitComplete(void);
extern void bspGsmPowerOnOff(void);
extern void bspSystemPowerOff(void);
extern RV_t bspIndicateError(uint32_t blinkTime);
extern RV_t bspNormalActivity(void);
extern RV_t bspGsmReset(void);

/**
 * @brief   CLI serial port.
 * @note    Allowed values are SD1, SD2 or SD3.
 * @note    SD3 (PB10 - TX, PB11 - RX)
 */
#define CLI_SERIAL_PORT     SD3

/**
 * @brief   GSM serial port.
 * @note    SD2 (PA2 - TX, PA3 - RX)
 */
#undef  GSM_SERIAL_PORT
#define GSM_SERIAL_PORT     SD2

#define CTRL_TASK_STACK_SIZE 1024

#define LOGGING_THREAD_STACK_SIZE 768

#define CLI_THREAD_STACK_SIZE 1024

#define ACCEL_THREAD_STACK_SIZE 1024

#define BSP_SYS_PWR_PORT GPIOA
#define BSP_SYS_PWR_PIN  GPIOA_PIN7

#define BSP_GSM_PWR_PIN  GPIOC_PIN2

#define BSP_PWR_OF_BUT   GPIOB_PIN12

#endif /* BSP */

