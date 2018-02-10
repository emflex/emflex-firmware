/**
******************************************************************************
* File:         main.c
* Description:  Entry point of GSM motion detector
*
* Created on:   December 24, 2016
* Author:       Denys Haryachyy
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

#include <string.h>
#include "common.h"
#include "bsp.h"
#include "cnfgr.h"
#include "cnfgr_api.h"
#include "cli.h"
#include "serial_port.h"
#include "logging.h"

/*
 * Watchdog deadline is set to 1.77sec (LSI=37000 / (16 * 0xFFF)).
 */
static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_16,
  STM32_IWDG_RL(0xFFF)
};

int main(void)
{
   uint32_t resetReason = RCC->CSR;

  /* System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active. */
  halInit();
  chSysInit();

  /* Init console UART*/
  serialInit(CLI_SERIAL_SPEED, &CLI_SERIAL_PORT);

  /* platform specific initialization code */
  bspInit();

  LOG_TRACE(0, "Reset register value: %u", resetReason);

  /* component initialization */
  cnfgrInit();
  cnfgrRegister("CLI", cliInit);
  cnfgrRegister("Logging", loggingAppInit);

  /* init persistent logging */
  persist_init();

  profileInit();

  if (RV_SUCCESS != cnfgrInvoke())
  {
    /* let each profile decide how to handle error case */
    profileCnfgrErrorHandle();
  }

  wdgStart(&WDGD1, &wdgcfg);

  while (1)
  {
    wdgReset(&WDGD1);
  }

  return 0;
}
