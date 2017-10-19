/*****************************************************************************
* File:         cnfgr_api.c
* Description:  Configurator main code
*
* Created on:   Apr 08, 2016
*
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

#include "common.h"
#include "cnfgr.h"
#include "logging.h"

static uint32_t emptyElement = 0;
static cid_t cidList[CNFGR_COMPONENT_MAX_NUMBER];

RV_t cnfgrInit(void)
{
  uint32_t i = 0;

  for (i = 0; i < CNFGR_COMPONENT_MAX_NUMBER; i++)
  {
    cidList[i].cb = NULL;
  }

  return RV_SUCCESS;
}

RV_t cnfgrRegister(const char* name, cnfgrFunc_t cb)
{
  if (emptyElement == CNFGR_COMPONENT_MAX_NUMBER)
  {
    return RV_FAILURE;
  }

  cidList[emptyElement].name = name;
  cidList[emptyElement].cb = cb;

  emptyElement++;

  return RV_SUCCESS;
}

RV_t cnfgrInvoke(void)
{
  uint32_t i = 0;

  for (i = 0; i < CNFGR_COMPONENT_MAX_NUMBER; i++)
  {
    if (cidList[i].cb == NULL)
    {
      continue;
    }

    if (RV_SUCCESS != cidList[i].cb())
    {
      LOG_ERROR(CNFGR_CMP, "Failed to initialize %s!", cidList[i].name);

      return RV_FAILURE;
    }
  }

  return RV_SUCCESS;
}

