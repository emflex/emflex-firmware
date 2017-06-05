/**
******************************************************************************
* File:         main.c
* Description:  Entry point of GSM tracker. Creates tasks and
*               goes to infinite loop
*
* Created on:   May 09, 2015
* Author:       rostokus
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
#include "logging.h"
#include "bsp.h"
#include "cnfgr_api.h"
#include "button.h"
#include "stepper.h"
#include "bl_api.h"
#include "ext_api.h"

static RV_t bleResponseParse(const char *blCmdBuf, int32_t len)
{
  int resp = 0;

  if (!blCmdBuf)
  {
    return RV_FAILURE;
  }

  if (0 == strncmp(blCmdBuf, "up", sizeof("up")-1))
  {
    LOG_TRACE(BLT_CMP, "UP button");

    resp = chMBPost(&stepMsg, STEP_UP, TIME_IMMEDIATE);
    if (resp < Q_OK)
    {
      LOG_TRACE(BLT_CMP, "rv = %i", resp);
    }
  }

  if (0 == strncmp(blCmdBuf, "down", sizeof("down")-1))
  {
    LOG_TRACE(BLT_CMP, "DOWN button");

    resp = chMBPost(&stepMsg, STEP_DOWN, TIME_IMMEDIATE);
    if (resp < Q_OK)
    {
      LOG_TRACE(BLT_CMP, "rv = %i", resp);
    }
  }

  if (0 == strncmp(blCmdBuf, "open", sizeof("open")-1))
  {
    LOG_TRACE(BLT_CMP, "OPEN button");

    resp = chMBPost(&stepMsg, FULLOPEN, TIME_IMMEDIATE);
    if (resp < Q_OK)
    {
      LOG_TRACE(BLT_CMP, "rv = %i", resp);
    }
  }

  if (0 == strncmp(blCmdBuf, "close", sizeof("close")-1))
  {
    LOG_TRACE(BLT_CMP, "CLOSE button");

    resp = chMBPost(&stepMsg, FULLCLOSED, TIME_IMMEDIATE);
    if (resp < Q_OK)
    {
      LOG_TRACE(BLT_CMP, "rv = %i", resp);
    }
  }

  return RV_SUCCESS;
}

void profileInit(void)
{
    extAppCbRegister(0,
                EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA,
                extcb1);

    extAppCbRegister(1,
                EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA,
                extcb2);

    blRegisterEventCb(bleResponseParse);

    cnfgrRegister("BLT", blInit);
    cnfgrRegister("Button", buttonAppInit);
    cnfgrRegister("Stepper", stepperAppInit);
    cnfgrRegister("Ext", extAppInit);
}
