/**
******************************************************************************
* File:         main.c
* Description:  Entry point of GPS tracker.
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

int main(void)
{
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

  /* component initialization */
  cnfgrInit();
  cnfgrRegister("CLI", cliInit);
  cnfgrRegister("Logging", loggingAppInit);

  /* init persistent logging */
  persist_init();

  profileInit();

  if (RV_SUCCESS != cnfgrInvoke())
  {
    /* need to handle this correctly */
  }

  while (1)
   ;

  return 0;
}
