/*****************************************************************************
* File:       gsm_common.h
*
* Created on: May 01, 2015
*
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

#ifndef GSM_COMMON
#define GSM_COMMON

#include "common.h"

typedef enum
{
  WAIT_FOR_SOS_CR_STATE,
  WAIT_FOR_SOS_LF_STATE,
  STORE_DATA_STATE,
  WAIT_FOR_SPACE_STATE,
  WAIT_FOR_LF_STATE,
  FINISH_STATE
} gsm_parse_state;

typedef enum
{
  GSM_EVENT_UNKNOWN,
  GSM_EVENT_UP,
  GSM_EVENT_DOWN,
  GSM_EVENT_SMS_START,
  GSM_EVENT_SMS_STOP,
  GSM_EVENT_SMS_STATE,
  GSM_EVENT_BALANCE_SIGN_BATT,
  GSM_EVENT_POWER_LOW,
  GSM_EVENT_POWER_HIGH,
  GSM_EVENT_VOICE_CALL,
  GSM_EVENT_LAST
} gsmEvent_t;

typedef RV_t (*gsmCbFunc_t)(gsmEvent_t);

#endif /* GSM_COMMON */


