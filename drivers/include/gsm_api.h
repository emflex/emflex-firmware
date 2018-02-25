/*****************************************************************************
* File:       gsm_api.h
*
* Created on: May 01, 2015
* Author:     rostokus
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

#ifndef GSM_API
#define GSM_API

#include "common.h"
#include "gsm_common.h"

extern RV_t gsmInit(void);
extern RV_t gsmSmsSend(const char *smsStr);
extern RV_t gsmCall(void);
extern RV_t gsmRegisterEventCb(gsmEvent_t event, gsmCbFunc_t cb);
extern RV_t gsmStateReqSend(void);
extern RV_t gsmStateReqGet(uint32_t *sign, uint32_t *batt, char *bal, uint32_t len);
extern void gsmModulePhoneNumberAdd(char* number, char* name);
extern RV_t gsmSend(const char *gsm_command);

#endif


