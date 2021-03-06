/*****************************************************************************
* File:       gsm_api.c
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

#include "common.h"
#include "logging.h"
#include "gsm_common.h"
#include "gsm_ll_api.h"

/* Initialize gsm module */
RV_t gsmInit(void)
{
  /* create task, msg queue, etc. */
  if (RV_SUCCESS != gsmTaskInit())
  {
    LOG_ERROR(GSM_CMP, "Failed to create GSM task");
    return RV_FAILURE;
  }

  /* initialize GSM module */
  if (RV_SUCCESS != gsmModuleInit())
  {
    LOG_ERROR(GSM_CMP, "Failed to initialize GSM module");
    return RV_FAILURE;
  }

  LOG_TRACE(GSM_CMP, "GSM driver initialized successfully");

  return RV_SUCCESS;
}

/* Send sms to gsm module */
RV_t gsmSmsSend(const char* smsStr)
{
  if (smsStr == 0)
  {
    return RV_FAILURE;
  }

  if (phoneBook_g.resp_is_set == FALSE)
  {
    LOG_TRACE(GSM_CMP,"No number to respond to");
    return RV_FAILURE;
  }

  return gsmSendSmsToNumber(phoneBook_g.resp_number, smsStr);
}

/* Call a number */
RV_t gsmCall(const char *telNum)
{
  (void) telNum;

  return RV_SUCCESS;
}

/* Register callback to receive asynchronous events from gsm module */
RV_t gsmRegisterEventCb(gsmEvent_t event, gsmCbFunc_t cb)
{
  if ((event == GSM_EVENT_UNKNOWN) || (event == GSM_EVENT_LAST))
  {
      return RV_FAILURE;
  }

  gsmCbArray_g[event] = cb;

  return RV_SUCCESS;
}

RV_t gsmVoiceCallHandle(void)
{
  static BOOL startCmd = RV_FALSE;

  if (startCmd == RV_FALSE)
  {
    gsmCallEventCb(GSM_EVENT_SMS_START);
    startCmd = RV_TRUE;
  }
  else
  {
    gsmCallEventCb(GSM_EVENT_SMS_STOP);
    startCmd = RV_FALSE;
  }

  return RV_SUCCESS;
}

RV_t gsmStateReqSend(void)
{
  return gsmLlDeviceStateGet();
}
