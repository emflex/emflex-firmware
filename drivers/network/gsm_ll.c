/*****************************************************************************
* File:         gsm_ll.c
* Description:
*               Low level functionality that communicates with GSM module:
*               transmit new commands to GSM, parse response to sent commands
*               (i.e. OK, FAIL) and handle incoming asynchronous messages
*               (like events about new SMS.
*               All functions in this file must be prepended with 'gsmLl'
*               to distinguish GSM internal functionality from user API.
*
* Created on:   Mar 29, 2015
* Authors:      Rostyslav Spolyak, Denys Haryachyy
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
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "common.h"
#include "logging.h"
#include "gsm_ll.h"
#include "gsm_ll_api.h"
#include "gsm_common.h"
#include "utils.h"
#include "bsp.h"
#include "serial_port.h"

gsmCbFunc_t gsmCbArray_g[GSM_EVENT_LAST];
phoneBook_t phoneBook_g;

static mailbox_t gsm_tx_mb_s;
static msg_t gsm_tx_msg_queue_s[MAILBOX_QUEUE_TX_SIZE];
static uint32_t signal = 0;
static uint32_t battery = 0;
static bool gsmReady = false;
static command_t cur_command = {0, true};
static virtual_timer_t vt;

static RV_t gsmLlStateAnalyze(const char *str, uint32_t len);
static void gsmLlCmdDump(uint32_t bufLen, const char *buf);
static uint8_t gGsmLastCmdResend = 0;


static struct
{
  bool state;
  float balance;
} balance_s = {.state = false, .balance = 0.0};

static THD_WORKING_AREA(gsmThread, GSM_THREAD_STACK_SIZE);

static bool gsm_is_ready = FALSE;
static MUTEX_DECL(gsm_gSDMutex);
static CONDVAR_DECL(gsm_ready_cond_var);

extern mutex_t gSDMutex;

static bool gsmReadyGet(void)
{
  return gsmReady;
}

static void gsmReadySet(void)
{
  gsmReady = true;
}

RV_t gsmIsOnSet(void)
{
  chMtxLock(&gsm_gSDMutex);

  gsm_is_ready = TRUE;

  chCondSignal(&gsm_ready_cond_var);

  chMtxUnlock(&gsm_gSDMutex);

  return RV_SUCCESS;
}

RV_t gsmIsOnGet(systime_t time)
{
  chMtxLock(&gsm_gSDMutex);

  while (gsm_is_ready == FALSE)
  {
    if (MSG_TIMEOUT == chCondWaitTimeout(&gsm_ready_cond_var, time))
    {
        return RV_TIMEOUT;
    }
  }

  chMtxUnlock(&gsm_gSDMutex);

  return RV_SUCCESS;
}

/* check if received event from GSM is sms content */
RV_t gsmCmpCommand(const char *inBuf, const char *cmpBuf)
{
  if ((char *) 0 != strstr(inBuf, cmpBuf))
  {
    return RV_SUCCESS;
  }
  else
  {
    return RV_FAILURE;
  }
}

RV_t gsmStateReqGet(uint32_t *sign, uint32_t *batt, char *bal, uint32_t len)
{
  if (!sign || !batt || !bal)
  {
    LOG_ERROR(GSM_CMP, "Null pointer received");
    return RV_FAILURE;
  }

  *sign = signal;
  *batt = battery;
   if (balance_s.state == false)
   {
     strncpy(bal, "not ready", len);
   }
   else
   {
     if (RV_SUCCESS != floatToStr(balance_s.balance, bal, len))
     {
       LOG_ERROR(GSM_CMP, "Failed to convert balance");
       return RV_FAILURE;
     }
   }

  return RV_SUCCESS;
}

/* parse received event about new sms and form SMS request command */ 
static RV_t gsmFormSmsGetRqst(char *inBuf, char *outBuf)
{
  uint8_t i = 0;
  uint8_t len = 0;

  strncpy(outBuf, "AT+CMGR=", sizeof("AT+CMGR="));
  i = strlen(outBuf);

  len = strlen(inBuf);
  if (isdigit((uint8_t) inBuf[len-2]))
  {
    outBuf[i++] = inBuf[len-2];
  }
  outBuf[i++] = inBuf[len-1];

  outBuf[i++] = ',';
  outBuf[i++] = '0';
  outBuf[i++] = '\r';
  outBuf[i] = '\0';

  return RV_SUCCESS;
}

RV_t gsmLlSdWrite(const char *val)
{
  msg_t resp = Q_OK;

  if (!val)
  {
    LOG_ERROR(GSM_CMP, "Null pointer received");
    return RV_FAILURE;
  }

  if ((resp = sdWriteTimeout(&GSM_SERIAL_PORT, (uint8_t *) val, strlen(val),
                             GSM_WRITE_TIMEOUT)) < Q_OK)
  {
    LOG_ERROR(GSM_CMP, "Failed to send GSM command to UART buffer. Error %i", resp);
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}

static RV_t gsmLlCmdBufAllocate(const char *buf, char **out)
{
  uint32_t len = 0;

  if ((!buf) || (!out))
  {
    LOG_ERROR(GSM_CMP, "Null pointer received");
    return RV_FAILURE;
  }

  len = strnlen(buf, MAX_BUF_LEN) + 1;

  *out = (char *) chHeapAlloc(0, len);

  if (*out)
  {
    strncpy(*out, buf, len);
  }
  else
  {
    chHeapFree((void *) *out);
    LOG_ERROR(GSM_CMP, "GSM cmd pool is empty");
    return RV_NOT_READY;
  }

  return RV_SUCCESS;
}

static RV_t gsmLlCmdBufFree(char *buf)
{
  if (!buf)
  {
    LOG_ERROR(GSM_CMP, "Null pointer received");
    return RV_FAILURE;
  }

  chHeapFree((void *) buf);

  return RV_SUCCESS;
}

static RV_t gsmLlCmdSend(const char *buf)
{
  RV_t  rv     = RV_FAILURE;
  msg_t rdymsg = MSG_OK;
  char  *out   = 0;

  rv = gsmLlCmdBufAllocate(buf, &out);
  if (RV_SUCCESS != rv)
  {
    LOG_ERROR(GSM_CMP, "Failed to allocate GSM command memory");
    return rv;
  }

  rdymsg = chMBPost(&gsm_tx_mb_s, (msg_t) out, TIME_IMMEDIATE);
  if (rdymsg != MSG_OK)
  {
    gsmLlCmdBufFree(out);
    LOG_ERROR(GSM_CMP, "Failed to queue TX msg, RV=%u", rdymsg);
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}

static RV_t gsmLlCmdSendFirst(const char *buf)
{
  RV_t  rv     = RV_FAILURE;
  msg_t rdymsg = MSG_OK;
  char  *out   = 0;

  rv = gsmLlCmdBufAllocate(buf, &out);
  if (RV_SUCCESS != rv)
  {
    return rv;
  }

  rdymsg = chMBPostAhead(&gsm_tx_mb_s, (msg_t) out, TIME_IMMEDIATE);
  if (rdymsg != MSG_OK)
  {
    gsmLlCmdBufFree(out);
    LOG_ERROR(GSM_CMP, "Failed to queue TX msg, RV=%u", rdymsg);
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}

void gsmModuleConnectGprs(void)
{
    LOG_TRACE(GSM_CMP, "Configure GPRS ...\r\n");

    gsmCmdSend(GSM_CHECK_SYGNAL_GPRS);
    gsmCmdSend(GSM_CHECK_GPRS_NETWORK);
    gsmCmdSend(GSM_ATTACH_GPRS_NETWORK);
    gsmCmdSend(GSM_CONTYPE_GPRS);
    gsmCmdSend(GSM_CONTYPE_APN);
    gsmCmdSend(GSM_GPRS_CONNECT);
    gsmCmdSend(GSM_GPRS_CHECK);
    gsmCmdSend(GSM_ENABLE_HTTP_SERVICE);
}

void gsmModuleDisconnectGprs(void)
{
    LOG_TRACE(GSM_CMP, "Disconnect from GPRS...\r\n");

    gsmCmdSend(GSM_DISABLE_HTTP_SERVICE);
    gsmCmdSend(GSM_GPRS_DISCONNECT);
}

void gsmModuleSendGetHttpRequest(uint8_t signal, uint8_t battery)
{
    char buf[96] = GSM_HTTP_SET_URL;
    char temp[32] = {0};

    STRCAT_SAFE(buf, temp);

    osapiItoa(battery, temp, sizeof(temp));
    STRCAT_SAFE(buf, "bat=");
    STRCAT_SAFE(buf, temp);

    osapiItoa(signal, temp, sizeof(temp));
    STRCAT_SAFE(buf, "&sig=");
    STRCAT_SAFE(buf, temp);

    STRCAT_SAFE(buf, "\"\r");

    gsmCmdSend(GSM_HTTP_SET_BEARER_PROFILE_ID);
    gsmCmdSend(GSM_HTTP_SET_SSL);
    gsmCmdSend(buf);
    gsmCmdSend(GSM_HTTP_SET_GET_METHOD);
    gsmCmdSend(GSM_HTTP_READ_DATA);
}

void gsmModuleCfg(void)
{
  gsmLlCmdSend(GSM_FIXED_BAUDRATE);
  gsmLlCmdSend(GSM_ECHO_DISABLE);
  gsmLlCmdSend(GSM_TA_RESPONSE_FORMAT_ENABLE);
  gsmLlCmdSend(GSM_SMS_TEXT_MESSAGE_FORMAT);
  gsmLlCmdSend(GSM_NEW_SMS_MESSAGE_INDICATION);
  gsmLlCmdSend(GSM_REPORT_ERROR_CODE_VERBOSE_ENABLE);
  gsmLlCmdSend(GSM_LEGACY_SMS_CLEAR);

  //gsmCmdSend(GSM_PHONEBOOK_READ_ALL);

  gsmLlCmdSend(GSM_SLEEP_MODE_DTR);
}

void gsmModulePhoneNumberAdd(char* number, char* name)
{
  char buf[100] = {0};

  sprintf(buf, GSM_PHONEBOOK_WRITE_ENTRY, number, name);

  gsmCmdSend(buf);
}

/* handle asynchronous event about new SMS from GSM module */
static RV_t gsmModuleAsyncEventHandle(char *str)
{
  char gsmCommGetSms[GSM_CTRL_CMD_LEN] = {0};

  /* notification about new message received.
     create command to retrieve the message itself */
  if (RV_SUCCESS == gsmCmpCommand(str, GSM_MSG_CMTISM))
  {
    if (RV_SUCCESS == gsmFormSmsGetRqst(str, gsmCommGetSms))
    {
      gsmCmdSend(gsmCommGetSms);
    }
  }
  /* message content received*/
  else if (RV_SUCCESS == gsmCmpCommand(str, GSM_MSG_CMT))
  {
    LOG_TRACE(GSM_CMP, "Rec sms: %s", str);
    gsmTaskCb(str);
  }
  else
  {
    LOG_TRACE(GSM_CMP, "Rec event:%s", str);
  }

  return RV_SUCCESS;
}

/* Analyze data from GSM module */
static RV_t gsmModuleCmdAnalyze(char *buf, uint32_t len)
{
  if (buf == 0)
  {
    return RV_FAILURE;
  }

  /* gsm module is switched on */
  if (RV_SUCCESS == gsmCmpCommand(buf, GSM_CALL_RDY_EVENT) ||      
      RV_SUCCESS == gsmCmpCommand(buf, GSM_MODULE_VENDOR_NAME))
  {
    /* allow commands to be dispatched to GSM */
    cur_command.ack = true;

    gsmIsOnSet();

    LOG_TRACE(GSM_CMP, "GSM is ready");

    return RV_SUCCESS;
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_NETWORK_STATUS_STR))
  {
    return RV_SUCCESS;
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_POWER_DOWN_EVENT))
  {    
    return RV_SUCCESS;
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_RDY_EVENT))
  {
    return RV_SUCCESS;
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_CPIN_RDY_EVENT))
  {
    return RV_SUCCESS;
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_CFUN_EVENT))
  {
    return RV_SUCCESS;
  }
#if 0
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_POWER_WARN_EVENT))
  {
    /* change supply voltage range from normal: 3.4 ... 4.5V to extreme: 3.1…4.7V
     * this will increase our chances to successfully send last notification msg */
    gsmCmdSend("AT+VR=1\r\r\n");

    /* wait until GSM module process the command and send response */
    chThdSleepMilliseconds(300);

    if (RV_SUCCESS != gsmCallEventCb(GSM_EVENT_POWER_LOW))
    {
      LOG_TRACE(GSM_CMP, "Failed send under-voltage event");
      return RV_FAILURE;
    }

    LOG_TRACE(GSM_CMP, "Received under-voltage event");

    return RV_SUCCESS;
  }
#endif
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_BALANCE_RESPONSE) ||
           RV_SUCCESS == gsmCmpCommand(buf, GSM_BATTERY_CMD_RESPONCE) ||
           RV_SUCCESS == gsmCmpCommand(buf, GSM_SIGNAL_CMD_RESPONCE))
  {
    if (RV_SUCCESS != gsmLlStateAnalyze(buf, len))
    {
      LOG_TRACE(GSM_CMP, "Failed analyze GSM state");
      return RV_FAILURE;
    }
  }
  /* New voice call */
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_NO_CARRIER))
  {
    return gsmCallEventCb(GSM_EVENT_VOICE_CALL);
  }
#if 0
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_PHONE_BOOK_READ_MATCH_STR))
  {
    char number[MAX_BUF_LEN] = {};

    gsmPhoneNumberParse(buf, number);
    gsmPhoneNumberAdd(number);
  }
#endif
  else if (0 == strncmp(buf, GSM_NEW_MSG_EVENT, sizeof(GSM_NEW_MSG_EVENT)-1))
  {
    if (RV_SUCCESS != gsmModuleAsyncEventHandle(buf))
    {
      LOG_TRACE(GSM_CMP, "Failed to sent parsed cmd");
      return RV_FAILURE;
    }
    return RV_SUCCESS;
  }
  else
  {
    /* '>' can be considered as SMS send acknowledge */
    if (!strcmp(buf, "OK") || !strcmp(buf, ">"))
    {
        /* allow next command to be dispatched to GSM module */
        cur_command.ack = true;

        /* Clear GSM response timeout timer */
        chVTReset(&vt);
    }
    else if (!strcmp(buf, "ERROR"))
    {
      /* Blink LED once per 3 seconds. Specifies any error */
      bspIndicateError(3000);

      /* allow next command to be dispatched to GSM module */
      cur_command.ack = true;

      LOG_ERROR(GSM_CMP, "GSM replied with error:%s", buf);
    }
    else if (!strncmp(buf, "+CMS ERROR:", strlen("+CMS ERROR:")) ||
             !strncmp(buf, "+CME ERROR:", strlen("+CME ERROR:")))
    {
      char* sMatch = 0;
      sMatch = strchr(buf, ':');
      sMatch++;

      /* normally 'PS busy' is received when SIM balance is zero.
         Start blinking LED 4 times per second to notify user */
      if (strncmp(sMatch, "PS busy", sizeof("PS busy")))
      {         
         bspIndicateError(250);
      }
      /* Start blinking once per second */
      else if (strncmp(sMatch, "operation not allowed", sizeof("operation not allowed")))
      {
         bspIndicateError(1000);
      }
      /* Blink LED once per 3 seconds. Specifies any other error */
      else
      {
         bspIndicateError(3000);
      }

      /* allow next command to be dispatched to GSM module */
      cur_command.ack = true;

      LOG_ERROR(GSM_CMP, "GSM replied with error:%s", buf);
    }
    else
    {
      LOG_TRACE(GSM_CMP, "Unknown GSM command received:%s", buf);
    }
  }
  return RV_SUCCESS;
}

/* Parse input value and form GSM reply command.
 * State machine handles following cases:
 * "\r\nRESPONSE\r\n"
 * "\r\n> "
 * "\r\n+CMT*RESPONSE*\r\n*RESPONSE*\r\n" -> skip \r\n at the middle of response.
 *  */
static RV_t gsmModuleCmdParse(const char *inGsmStr, int32_t inGsmStrLen)
{
  static gsm_parse_state state = WAIT_FOR_SOS_CR_STATE;
  static char gsmCmdBuf[MAX_GSM_CMD_LEN * 2];
  static char *pGsmCmdBuf = gsmCmdBuf;
  static uint8_t cmtPresence = 0;

  const char *gsmCmdBufEndAddr = gsmCmdBuf + (sizeof(gsmCmdBuf) - 1);

  if (!inGsmStr)
  {
    return RV_FAILURE;
  }

  while (inGsmStrLen > 0)
  {
    switch (state)
    {
      case WAIT_FOR_SOS_CR_STATE:
        /* unlike other states, this state handle the case when junk bytes
         * are received before \r\n (like at gsm module startup) */
        if (*inGsmStr == CR)
        {
          state = WAIT_FOR_SOS_LF_STATE;
        }
        inGsmStr++;
        inGsmStrLen--;
        if (inGsmStrLen < 0)
        {
          return RV_NOT_COMPLETED;
        }
        break;

      case WAIT_FOR_SOS_LF_STATE:
        if (*inGsmStr == LF)
        {
          inGsmStr++;
          inGsmStrLen--;
          state = STORE_DATA_STATE;
          if (inGsmStrLen < 0)
          {
            return RV_NOT_COMPLETED;
          }
        }
        else
        {
          LOG_TRACE(GSM_CMP, "Unexpected symbol received:%x. GSM state: WAIT_FOR_SOS_LF_STATE",
                    *inGsmStr);
          state = WAIT_FOR_SOS_CR_STATE;
        }
        break;

      case STORE_DATA_STATE:
        while (inGsmStrLen > 0)
        {
          if (*inGsmStr == '>')
          {
            if (pGsmCmdBuf <= gsmCmdBufEndAddr)
            {
              *pGsmCmdBuf = *inGsmStr;
              pGsmCmdBuf++;
            }
            else
            {
              LOG_TRACE(GSM_CMP, "Failed to add symbol %c. GSM cmd buffer is full", *inGsmStr);
            }
            inGsmStr++;
            inGsmStrLen--;
            state = WAIT_FOR_SPACE_STATE;
            break;
          }
          else if (*inGsmStr == CR)
          {
            inGsmStr++;
            inGsmStrLen--;
            state = WAIT_FOR_LF_STATE;
            break;
          }

          if (pGsmCmdBuf < gsmCmdBufEndAddr)
          {
            *pGsmCmdBuf = *inGsmStr;
            pGsmCmdBuf++;
          }
          else
          {
            LOG_TRACE(GSM_CMP, "Failed to add symbol %c. GSM cmd buffer is full", *inGsmStr);
          }

          inGsmStr++;
          inGsmStrLen--;
          if (inGsmStrLen == 0)
          {
            return RV_NOT_COMPLETED;
          }
        }
        break;

      case WAIT_FOR_SPACE_STATE:
        if (*inGsmStr == ' ')
        {
          inGsmStr++;
          state = FINISH_STATE;
        }
        else
        {
          LOG_TRACE(GSM_CMP, "Unexpected symbol received:%x. GSM state: WAIT_FOR_SOS_CR_STATE",
                    *inGsmStr);
          state = WAIT_FOR_SOS_CR_STATE;
        }
        break;

      case WAIT_FOR_LF_STATE:
        if (*inGsmStr == LF)
        {
          inGsmStr++;
          if ((0 == memcmp(gsmCmdBuf, GSM_MSG_CMT, strlen(GSM_MSG_CMT))) &&
              (cmtPresence == 0))
          {
            state = STORE_DATA_STATE;
            cmtPresence = 1;
            inGsmStrLen--;
          }
          else
          {
            state = FINISH_STATE;
            cmtPresence = 0;
          }

          if (inGsmStrLen < 0)
          {
            return RV_NOT_COMPLETED;
          }
        }
        else
        {
          LOG_TRACE(GSM_CMP, "Unexpected symbol received:%x. GSM state: WAIT_FOR_SOS_CR_STATE",
                    *inGsmStr);
          state = WAIT_FOR_SOS_CR_STATE;
        }
        break;

      case FINISH_STATE:
        *pGsmCmdBuf = '\0';
        state = WAIT_FOR_SOS_CR_STATE;
        inGsmStrLen--;

        LOG_TRACE(GSM_CMP, "New GSM command received:%s", gsmCmdBuf);

        if (RV_SUCCESS != gsmModuleCmdAnalyze(gsmCmdBuf, (pGsmCmdBuf-gsmCmdBuf)+1))
        {
          LOG_ERROR(GSM_CMP, "Failed to analyze received GSM command");
        }

        pGsmCmdBuf = gsmCmdBuf;
        break;
    }
  }
  return RV_SUCCESS;
}

void gsmLlTimeoutCb(void *param)
{
  (void) param;

  /* re-send previous command */
  gGsmLastCmdResend = 1;
}

static THD_FUNCTION(gsmTask, arg)
{
  (void) arg;
  uint32_t gsmInByteNum = 0;
  char buf[MAX_GSM_CMD_LEN] = {0};
  char* pBuf = 0;
  msg_t res = 0;
  RV_t rv = RV_FAILURE;
  uint32_t currTime = 0;
  int32_t time = -15;
  char gsmCurrCmd[64] = {0};

  while (1)
  {
    /*Decrease the read speed from UART*/
    chThdSleepMilliseconds(200);

    /* TODO: add support to re-send complex commands, like sending SMS */
    if (gGsmLastCmdResend)
    {
      LOG_ERROR(GSM_CMP, "GSM response timeout. Re-sending command:%s", gsmCurrCmd);
     
      if (RV_SUCCESS != gsmLlCmdSendFirst(gsmCurrCmd))
      {
        LOG_ERROR(GSM_CMP, "Failed to re-send command");
      }

      /* Reset GSM module */
      bspGsmReset();
    }

    /* dispatch next cmd to GSM if response to previous command was received */
    if (cur_command.ack == true)
    {
      res = chMBFetch(&gsm_tx_mb_s, (msg_t*) &pBuf, 100);
      if (res == MSG_OK)
      {
        cur_command.id++;

        /* exit sleep mode. serial port will be active after about 50ms */
        palClearPad(GPIOA, GPIOA_PIN6);
        chThdSleepMilliseconds(100);

        LOG_TRACE(GSM_CMP, "Sending a command #%d:%s", cur_command.id, pBuf);
        rv = gsmLlSdWrite(pBuf);
        if (RV_SUCCESS != rv)
        {
          LOG_ERROR(GSM_CMP, "GSM task failed to send command with %error %u", rv);
        }

        /* enter sleep mode */
        palSetPad(GPIOA, GPIOA_PIN6);

        cur_command.ack = false;

        /* store current command to be re-send in case of failure */
        strncpy(gsmCurrCmd, pBuf, sizeof(gsmCurrCmd));

        /* return memory block to heap pool */
        if (pBuf)
        {
          chHeapFree((void *) pBuf);
        }

        /* start timer to track GSM response */
        chVTSet(&vt, S2ST(120), gsmLlTimeoutCb, 0);
      }
    }

    /* read data from serial port */
    gsmInByteNum = sdAsynchronousRead(&GSM_SERIAL_PORT, (uint8_t *) buf, sizeof(buf));
    if (gsmInByteNum)
    {
      LOG_TRACE(GSM_CMP, "GSM returned %u bytes", gsmInByteNum);

      gsmLlCmdDump(gsmInByteNum, buf);

      rv = gsmModuleCmdParse(buf, gsmInByteNum);
      if (rv == RV_FAILURE)
      {
        LOG_ERROR(GSM_CMP, "Failed to parse GSM command");
      }
    }

    /* query GSM module status (balance, battery, signal level) each 15 minutes */
    if (gsmReadyGet() == true)
    {
      currTime = logTimeStampGet();
      if ((currTime - time) >= 15)
      {
        time = currTime;
        gsmLlDeviceStateGet();

      }
    }
  }
}

RV_t gsmTaskInit(void)
{
  char number[] = "+380982297151"; // Rostyk
//  char number[] = "+380676708491"; // Denys

  /* create message queue to send asynchronous requests */
  chMBObjectInit(&gsm_tx_mb_s, gsm_tx_msg_queue_s, MAILBOX_QUEUE_TX_SIZE);

  if (serialInit(GSM_SERIAL_SPEED, &GSM_SERIAL_PORT) < 0)
  {
    LOG_ERROR(GSM_CMP, "Serial is already occupied");
    return RV_FAILURE;
  }

  /* Create thread */
  chThdCreateStatic(gsmThread, sizeof(gsmThread), NORMALPRIO+1, gsmTask, 0);

  memset(&phoneBook_g, 0, sizeof(phoneBook_g));

  phoneBook_g.resp_is_set = TRUE;
  strncpy(phoneBook_g.resp_number, number, sizeof(phoneBook_g.resp_number));

  /* add predefined phone number to allow out-of-box configuration */
  gsmPhoneNumberAdd(number);

  return RV_SUCCESS;
}

/*
 * Create message and sent to GSM
 */
RV_t gsmSendSmsToNumber(const char *telNum, const char *text)
{
  char buf[MAX_BUF_LEN] = {0};
  uint32_t len = 0;

  strcpy(buf, SEND_SMS_STR);
  strcat(buf, telNum);
  strcat(buf, "\"\r");

  gsmCmdSend(buf);

  strncpy(buf, text, sizeof(buf));
  len = strlen(buf);

  /* transmit EOF character */
  if ((len+1) >= sizeof(buf))
  {
    buf[len-1] = EOF_SMS;
    buf[len] = NULL_SYM;
  }
  else
  {
    buf[len] = EOF_SMS;
    buf[len+1] = NULL_SYM;
  }

  gsmCmdSend(buf);

  return RV_SUCCESS;
}

/* Call a callback */
RV_t gsmCallEventCb(gsmEvent_t event)
{
  if ((event > GSM_EVENT_UNKNOWN) && (event < GSM_EVENT_LAST))
  {
      return gsmCbArray_g[event]();
  }

  LOG_ERROR(GSM_CMP, "Failed to execute GSM callback");

  return RV_FAILURE;
}

RV_t gsmModuleInit()
{
  /* Check whether GSM module is already running.
   * Send test command and wait for GSM reply. */
  if (RV_SUCCESS != gsmLlCmdSend(GSM_MODULE_MODEL_GET))
  {
    LOG_ERROR(GSM_CMP, "Failed to send initial GSM command");
    return RV_FAILURE;
  }

  /* If GSM module does not respond within 1 sec timeout,
   * proceed to switch it on */
  if (RV_TIMEOUT == gsmIsOnGet(S2ST(1)))
  {
      bspGsmPowerOnOff();

      /* block until GSM is ready */
      gsmIsOnGet(TIME_INFINITE);

      /* GSM is ready to be configured. Send initialization commands */
      gsmModuleCfg();
  }

  gsmReadySet();
\
  LOG_TRACE(GSM_CMP, "GSM module configured successfully");

  return RV_SUCCESS;
}

RV_t gsmPhoneNumberParse(const char* buf, char* number)
{
  char* pMatchStart = NULL;
  char* pMatchEnd   = NULL;
  int   length      = 0;

  pMatchStart = strchr(buf, '"');
  pMatchStart++;
  pMatchEnd = strchr(pMatchStart, '"');
  length = pMatchEnd - pMatchStart;

  memcpy(number, pMatchStart, length);

  return RV_SUCCESS;
}

RV_t gsmPhoneNumberAdd(const char* number)
{
  strncpy(phoneBook_g.data[phoneBook_g.size].number, number, GSM_PHONE_NUMBER_LEN);

  phoneBook_g.size++;

  return RV_SUCCESS;
}

RV_t gsmPhoneNumberFind(const char* number)
{
  uint32_t i = 0;

  for (i = 0; i < GSM_PHONE_BOOK_SIZE; i++)
  {
    if (0 == strncmp(phoneBook_g.data[i].number, number, GSM_PHONE_NUMBER_LEN))
    {
      return RV_SUCCESS;
    }
  }

  LOG_ERROR(GSM_CMP, "Phone number is not found");

  return RV_FAILURE;
}

/******************************************************
* @purpose Receive ctrl message from user and
*          parse cmds
*
* @notes   1) ctrl message format: 1 - start, 2 - stop
*          2) get tel number and compare it with predefined
*             numbers from internal database
*          3) input data example:
*             +CMT: "+12223334444","","14/05/29,01:04:18-32"
*             The text message body goes here
*
* @end
******************************************************/
RV_t gsmTaskCb(const char *in)
{
  char senderTelNum[GSM_PHONE_NUMBER_LEN] = {0};

  /* First of all extract tel number and ensure that we trust this sender */
  gsmPhoneNumberParse(in, senderTelNum);

  if (gsmPhoneNumberFind(senderTelNum) != RV_SUCCESS)
  {
    LOG_ERROR(GSM_CMP, "Unauthorized user access!");
    return RV_FAILURE;
  }

  //phoneBook_g.resp_is_set = TRUE;

  //strncpy(phoneBook_g.resp_number, senderTelNum, sizeof(senderTelNum));

  if (RV_SUCCESS == gsmCmpCommand(in, START_CMD))
  {
    return gsmCallEventCb(GSM_EVENT_SMS_START);
  }

  if (RV_SUCCESS == gsmCmpCommand(in, STOP_CMD))
  {
    return gsmCallEventCb(GSM_EVENT_SMS_STOP);
  }

  if (RV_SUCCESS == gsmCmpCommand(in, STATE_CMD))
  {
    return gsmCallEventCb(GSM_EVENT_SMS_STATE);
  }

  LOG_ERROR(GSM_CMP, "Unsupported SMS command");

  return RV_FAILURE;
}

RV_t gsmCmdSend(const char *gsm_command)
{
  RV_t rv = RV_FAILURE;

  if (gsmReadyGet() == true)
  { 
    if (RV_SUCCESS == (rv = gsmLlCmdSend(gsm_command)))
    {
      return RV_SUCCESS;
    }
    else if (RV_NOT_READY == rv)
    {
      chThdSleepMilliseconds(300);

      LOG_ERROR(GSM_CMP, "Timeout sending GSM command. Sending again...");

      rv = gsmLlCmdSend(gsm_command);
    }

    if (RV_FAILURE == rv)
    {
      LOG_ERROR(GSM_CMP, "Failed to send GSM command with error %u", rv);
      return RV_FAILURE;
    }
  }
  else
  {
    LOG_ERROR(GSM_CMP, "Failed to send GSM command. GSM is not ready");
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}

RV_t gsmLlDeviceStateGet(void)
{
  if (RV_SUCCESS == gsmCmdSend(GSM_SIGNAL_LEVEL) &&
      RV_SUCCESS == gsmCmdSend(GSM_BATTERY_DISCHARGE) &&
      RV_SUCCESS == gsmCmdSend(GSM_BALANCE_CHECK))
  {
      return RV_SUCCESS;
  }

  LOG_ERROR(GSM_CMP, "Failed to query GSM state");

  return RV_FAILURE;
}

static RV_t gsmLlStateAnalyze(const char *buf, uint32_t len)
{
  (void) len;

  static uint8_t isGsmOk         = 0;
  static BOOL    isBatNotifSend  = RV_FALSE;
  BOOL           isGsmError      = RV_FALSE;

  if (buf == 0)
  {
    LOG_ERROR(GSM_CMP, "Null parameter was passed to %s", __FUNCTION__);
    return RV_FAILURE;
  }

  if (RV_SUCCESS == gsmCmpCommand(buf, GSM_BALANCE_RESPONSE))
  {
      /* possible return types:
       * +CUSD: 4  or  +CUSD: 2
       * +CUSD: 1 or 0,"Na rahunku 13.05 grn.
       */
      char* pMatch = strstr(buf, GSM_BALANCE_MATCH_TYPE);
      uint32_t ret_type = atoi(pMatch + strlen(GSM_BALANCE_MATCH_TYPE));

      if ((ret_type == 1) || (ret_type == 0))
      {
          pMatch = strstr(buf, GSM_BALANCE_MATCH_STR);
          balance_s.balance = atof(pMatch + strlen(GSM_BALANCE_MATCH_STR));
          balance_s.state = true;

          if (balance_s.balance < 1.0)
          {
            isGsmError = RV_TRUE;
          }
          else
          {
            isGsmOk++;
          }
      }
      else if ((ret_type == 2) || (ret_type == 4))
      {
          balance_s.balance = 0.0;
          balance_s.state = false;

          isGsmOk++;
      }
      else
      {
          LOG_ERROR(GSM_CMP, "GSM returned unexpected balance type - %u", ret_type);
      }

      LOG_TRACE(GSM_CMP, "Balance = %f", balance_s.balance);

      /* if balance, signal and battery are OK, enable normal LED activity  */
      if (isGsmOk == 3)
      {
        bspNormalActivity();
      }

      /* flush status counter */
      isGsmOk = 0;

      gsmCallEventCb(GSM_EVENT_BALANCE_SIGN_BATT);
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_BATTERY_CMD_RESPONCE))
  {
      /* +CBC: 0,80,3916 */
      char* sMatch = 0;
      char *eMatch = 0;
      sMatch = strchr(buf, ',');
      eMatch = strrchr(buf, ',');
      *eMatch = '\0';
      battery = atoi(sMatch+1);

      LOG_TRACE(GSM_CMP, "Battery discharge: %u", battery);

      /* send notification about low battery */
      if (battery < 10)
      {
        isGsmError = RV_TRUE;

        if (isBatNotifSend == RV_FALSE)
        {
          if (RV_SUCCESS != gsmCallEventCb(GSM_EVENT_POWER_LOW))
          {
            LOG_ERROR(GSM_CMP, "Failed to send under-voltage event");
            return RV_FAILURE;
          }

          isBatNotifSend = RV_TRUE;
        }
      }
      else
      {
         isGsmOk++;
         isBatNotifSend = RV_FALSE;
      }
  }
  else if (RV_SUCCESS == gsmCmpCommand(buf, GSM_SIGNAL_CMD_RESPONCE))
  {
      /* +CSQ: 9,0 */
      char* sMatch = 0;
      char *eMatch = 0;
      sMatch = strchr(buf, ' ');
      eMatch = strrchr(buf, ',');
      *eMatch = '\0';
      signal = atoi(sMatch+1);

      LOG_TRACE(GSM_CMP, "Signal level: %u", signal);

      /* Notify about low signal level.
         Blink LED once per second */
      if (signal <= 1)
      {
        isGsmError = RV_TRUE;
      }
      else
      {
        isGsmOk++;
      }
  }

  if (isGsmError == RV_TRUE)
  {
    bspIndicateError(1000);
  }

  return RV_SUCCESS;
}

static void gsmLlCmdDump(uint32_t bufLen, const char *buf)
{
    if (!buf)
    {
        return;
    }

    chMtxLock(&gSDMutex);
    chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), "___________\r\n");

    for (uint32_t i = 0; i < bufLen; i++)
    {
       chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), "%02x ", buf[i]);
    }

    chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), "\r\n___________\r\n");
    chMtxUnlock(&gSDMutex);
}
