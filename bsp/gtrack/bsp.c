/******************************************************************************
* File:        bsp.c
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

#include "common.h"
#include "bsp.h"

#define BSP_PWR_OFF_CHANNEL GPIOB_PIN12

void bspGsmPowerOnOff(void)
{
  /* pull down PWRKEY pin in GSM module */
  palSetPad(GPIOC, BSP_GSM_PWR_PIN);

  /* wait at least 1 sec */
  chThdSleepMilliseconds(1200);

  /* release PWRKEY (automatically raises HIGH) */
  palClearPad(GPIOC, BSP_GSM_PWR_PIN);

  /* give GSM more time to ensure it is UP */
  chThdSleepMilliseconds(100);
}

void bspSystemPowerOn(void)
{
  /* PWR IO is initialized in board.h file */
  palSetPad(BSP_SYS_PWR_PORT, BSP_SYS_PWR_PIN);

  return;
}

void bspSystemPowerOff(void)
{
  /* PWR IO is initialized in board.h file */
  palClearPad(BSP_SYS_PWR_PORT, BSP_SYS_PWR_PIN);

  return;
}

static void bspPwrOff(void *arg) {

  (void)arg;

  if (palReadPad(GPIOB, BSP_PWR_OF_BUT))
  {
      /* switch off GSM module */
      /* pull down PWRKEY pin in GSM module */
      palSetPad(GPIOC, BSP_GSM_PWR_PIN);

      /* wait at least 1 sec */
      for (uint32_t i = 0; i < 0x8FFFFF; i++)
        ;

      /* release PWRKEY (automatically raises HIGH) */
      palClearPad(GPIOC, BSP_GSM_PWR_PIN);

      /* switch off device */
      bspSystemPowerOff();
  }
}

static void bspExtcb1(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  (void)channel;

  static virtual_timer_t vt4;

  chSysLockFromISR();

  chVTSetI(&vt4, S2ST(5), bspPwrOff, NULL);

  chSysUnlockFromISR();

  return;
}

/* External interrupt/event controller (EXTI) config.
   Each line corresponds to separate channel of EXTI */
static const EXTConfig bspPwrCfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, bspExtcb1}, /* power on/off button is connected here */
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

RV_t bspInit(void)
{
  /* Device initialization has started */
  palSetPad(GPIOC, 6);

  /* Activates the UART driver for debugging,
   * PB10 and PB11 are routed to USART3. */
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));

  /* Activates GSM driver pins,
   * PA2 and PA3 are routed to USART2. */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /* start EXTI driver that handles power off event */
  extStart(&EXTD1, &bspPwrCfg);

  /* set IO pin responsible for switching DC-DC converter */
  bspSystemPowerOn();

  return RV_SUCCESS;
}

/* GPT4 error callback. */
static void bspGpt4ErrorCb(GPTDriver *gptp)
{
  (void) gptp;

  palTogglePad(GPIOC, 6);
}

/* GPT4 error configuration. */
static const GPTConfig bspGpt4ErrorCfg = {
  1000,         /* 1kHz timer clock.*/
  bspGpt4ErrorCb,    /* Timer callback.*/
  0,
  0
};

/* GPT4 initial callback. */
static void bspGpt4InitialCb(GPTDriver *gptp)
{
  (void) gptp;

  palSetPad(GPIOC, 6);
  for (uint32_t i = 0; i < 100; i++)
      ;
  palClearPad(GPIOC, 6);
}

/* GPT4 initial configuration. */
static const GPTConfig bspGpt4InitialCfg = {
  1000,         /* 1kHz timer clock.*/
  bspGpt4InitialCb,    /* Timer callback.*/
  0,
  0
};

static RV_t bspStartTimer3Sec(void)
{
  gptStart(&GPTD4, &bspGpt4InitialCfg);
  gptStartContinuous(&GPTD4, 3000);

  return RV_SUCCESS;
}

RV_t bspInitComplete(void)
{
    /* Device initialization completed successfully */
    palClearPad(GPIOC, 6);

    /* enable processing power on/off button event */
    extChannelEnable(&EXTD1, BSP_PWR_OFF_CHANNEL);

    /* Display normal device activity.
     * Blink status LED each 10 sec */
    bspStartTimer3Sec();

    return RV_SUCCESS;
}

RV_t bspIndicateError(uint32_t blinkTime)
{
  gptStopTimer(&GPTD4);

  gptStart(&GPTD4, &bspGpt4ErrorCfg);
  gptStartContinuous(&GPTD4, blinkTime);

  return RV_SUCCESS;
}

RV_t bspNormalActivity()
{
  gptStopTimer(&GPTD4);

  bspStartTimer3Sec();

  return RV_SUCCESS;
}

RV_t bspGsmReset(void)
{
  palSetPad(GPIOA, GPIOA_PIN1);
  for (int i = 0; i < 1000; i++)
    ;
  palClearPad(GPIOA, GPIOA_PIN1);

  return RV_SUCCESS;
}
