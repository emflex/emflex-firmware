/*****************************************************************************
* File:       gsm_ll_api.h
*
* Created on: Apr 04, 2016
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

#ifndef GSM_LL_API
#define GSM_LL_API

#include "gsm_common.h"

#define CR '\r'
#define LF '\n'
#define EOF_SMS '\x1A'
#define NULL_SYM '\0'

typedef struct command_s
{
  int id;
  bool ack;
} command_t;

typedef RV_t (*gsmCmdAnalyze_t)(char *buf, uint32_t len, uint32_t *val);

RV_t gsmTaskInit(void);
RV_t gsmModuleInit(void);
void gsmModuleCfg(void);
RV_t gsmSendSmsToNumber(const char *telNum, const char *text);
RV_t gsmModuleBalanceGet(float *bal);
void gsmModuleCheckState(void);
void gsmModuleSpeed(void);
RV_t gsmCallEventCb(gsmEvent_t event);
RV_t gsmCmpCommand(const char *inBuf, const char *cmpBuf);
extern RV_t gsmModuleCmdSend(const char *buf);
extern RV_t gsmCmdSend(const char *gsm_command);
extern RV_t gsmModuleSend(const char *val);
extern RV_t gsmLlDeviceStateGet(void);

#define GSM_PHONE_BOOK_SIZE 10
#define GSM_PHONE_NUMBER_LEN 17
#define GSM_PHONE_NAME_LEN 40

typedef struct phoneBookEntry_s
{
  char number[GSM_PHONE_NUMBER_LEN];
  char name[GSM_PHONE_NAME_LEN];
} phoneBookEntry_t;

typedef struct phoneBook_s
{
  phoneBookEntry_t data[GSM_PHONE_BOOK_SIZE];
  uint8_t size;

  char resp_number[GSM_PHONE_NUMBER_LEN];
  bool resp_is_set;
} phoneBook_t;

typedef enum
{
  GSM_CMD_OK = 1,
  GSM_CMD_ERROR,
  GSM_CMS_ERROR,
  GSM_CME_ERROR,
  GSM_SMS_PROMPT,
  GSM_PS_BUSY,
  GSM_OP_NOT
} gsm_cmd_resp_t;

#define GSM_SIGNAL_LEVEL                "AT+CSQ\r"
#define GSM_BATTERY_DISCHARGE           "AT+CBC\r"
#define GSM_BALANCE_CHECK "AT+CUSD=1,\"*111#\"\r"
#define GSM_POWER_DOWN          "AT+CPOWD=1\r"

#endif
