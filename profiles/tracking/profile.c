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
#include "bsp.h"
#include "cnfgr_api.h"
#include "gsm_api.h"
#include "control_api.h"
#include "cli.h"
#include "accelGyro.h"
#include "utils.h"
#include "logging.h"

const uint8_t gImuThresholdInDegrees = 1.0;

typedef enum
{
  START_EVENT = 0,
  STOP_EVENT,
  STATE_EVENT,
  ALARM_EVENT,
  BALANCE_EVENT,
  LOW_VOLTAGE_EVENT
} track_sm_event_t;

typedef enum
{
  IDLE_STATE = 0,
  ACTIVE_STATE,
  ALARM_STATE,
  TEST_STATE,
  LAST_STATE,
} track_sm_state_t;

static RV_t ctrlGsmEventUpProcess(void);
static RV_t ctrlGsmEventDownProcess(void);
static RV_t ctrlGsmEventBalanceProcess(void);
static RV_t ctrlGsmEventSmsStartProcess(void);
static RV_t ctrlGsmEventSmsStopProcess(void);
static RV_t ctrlGsmEventSmsStateProcess(void);
static RV_t ctrlImuEventAlarmProcess(void);
static RV_t ctrlVoltageEventAlarmProcess(void);
static RV_t ctrlGsmStateSend(void);
static RV_t underVoltageProcess(void);

RV_t doStateIdle(ctrl_sm_event_t ev, ctrl_sm_state_t* state)
{
  static bool ctrlWaitForBalance = false;
  dof_t dof;

  switch (ev)
  {
    case START_EVENT:
      LOG_TRACE(CONTROL_CMP, "START event!");

      *state = ACTIVE_STATE;

      LOG_TRACE(CONTROL_CMP, "ACTIVE state!");

      /* enable collection of statistics from gyro and accel */
      imuSumAngleGet(&dof);

      LOG_TRACE(CONTROL_CMP, "x=%f y=%f", dof.x, dof.y);

      imuThresholdSet(dof, gImuThresholdInDegrees);
      imuEnable();

      if (RV_SUCCESS != gsmSmsSend("ARM\r\n"))
      {
        LOG_TRACE(CONTROL_CMP, "Sms msg send failed!");
      }
      break;

    case STOP_EVENT:
      LOG_TRACE(CONTROL_CMP, "STOP event!");
      break;

    case ALARM_EVENT:
      LOG_TRACE(CONTROL_CMP, "ALARM event!");
      break;

    case STATE_EVENT:
      /* request GSM battery discharge, signal level, SIM card balance */
      if (RV_SUCCESS != gsmStateReqSend())
      {
        LOG_TRACE(CONTROL_CMP, "Could not send state request commands");
      }

      /* wait for balance event */
      ctrlWaitForBalance = true;

      break;

    case BALANCE_EVENT:
      if (ctrlWaitForBalance == true)
      {
        if (RV_SUCCESS != ctrlGsmStateSend())
        {
         LOG_TRACE(CONTROL_CMP, "Could not retrieve device state");
        }

        ctrlWaitForBalance = false;
      }
      break;

    case LOW_VOLTAGE_EVENT:
      if (RV_SUCCESS != underVoltageProcess())
      {
        LOG_TRACE(CONTROL_CMP, "Ctrl task: under voltage event failed");
      }
      break;

    default:
      LOG_TRACE(CONTROL_CMP, "UNSUPPORTED event!");
      break;
  }

  return RV_SUCCESS;
}

RV_t doStateActive(ctrl_sm_event_t ev, ctrl_sm_state_t* state)
{
  static bool ctrlWaitForBalance = false;

  switch (ev)
  {
    case START_EVENT:
      LOG_TRACE(CONTROL_CMP, "START event!");
      break;

    case STOP_EVENT:
      LOG_TRACE(CONTROL_CMP, "STOP event!");

      *state = IDLE_STATE;

      LOG_TRACE(CONTROL_CMP, "IDLE state!");

      /* disable collection of statistics from gyro and accel */
      imuDisable();

      if (RV_SUCCESS != gsmSmsSend("DISARM\r\n"))
      {
        LOG_TRACE(CONTROL_CMP, "Sms msg send failed!");
      }

      break;

    case ALARM_EVENT:
      LOG_TRACE(CONTROL_CMP, "ALARM event!");

      *state = ALARM_STATE;

      if (RV_SUCCESS != gsmSmsSend("ALARM\r\n"))
      {
        LOG_TRACE(CONTROL_CMP,"Sms msg send failed!");
      }

      break;

    case STATE_EVENT:
      /* request GSM battery discharge, signal level, SIM card balance */
      if (RV_SUCCESS != gsmStateReqSend())
      {
        LOG_TRACE(CONTROL_CMP,"Could not send state request commands");
      }

      /* wait for balance event */
      ctrlWaitForBalance = true;

      break;

    case BALANCE_EVENT:
      if (ctrlWaitForBalance == true)
      {
        if (RV_SUCCESS != ctrlGsmStateSend())
        {
         LOG_TRACE(CONTROL_CMP,"Could not get gTrack state");
        }

        ctrlWaitForBalance = false;
      }
      break;

    case LOW_VOLTAGE_EVENT:
      if (RV_SUCCESS != underVoltageProcess())
      {
        LOG_TRACE(CONTROL_CMP,"Ctrl task: under voltage event failed");
      }
      break;

    default:
      LOG_TRACE(CONTROL_CMP,"UNSUPPORTED event!");
      break;
  }

  return RV_SUCCESS;
}

RV_t doStateAlarm(ctrl_sm_event_t ev, ctrl_sm_state_t* state)
{
  static bool ctrlWaitForBalance = false;

  switch (ev)
  {
    case START_EVENT:
      LOG_TRACE(CONTROL_CMP, "START event!");
      break;

    case STOP_EVENT:
      LOG_TRACE(CONTROL_CMP, "STOP event!");

      *state = IDLE_STATE;

      LOG_TRACE(CONTROL_CMP, "IDLE state!");

      /* disable collection of statistics from gyro and accel */
      imuDisable();

      if (RV_SUCCESS != gsmSmsSend("DISARM\r\n"))
      {
        LOG_TRACE(CONTROL_CMP,"Sms msg send failed!");
      }

      break;

    case ALARM_EVENT:
      LOG_TRACE(CONTROL_CMP, "ALARM event!");

      /*if (RV_SUCCESS != gsmSmsSend("ALARM"))
      {
        LOG_TRACE(CONTROL_CMP,"Sms msg send failed!");
      }*/

      break;

    case STATE_EVENT:
      /* request GSM battery discharge, signal level, SIM card balance */
      if (RV_SUCCESS != gsmStateReqSend())
      {
        LOG_TRACE(CONTROL_CMP,"Could not send state request commands");
      }

      /* wait for balance event */
      ctrlWaitForBalance = true;

      break;

    case BALANCE_EVENT:
      if (ctrlWaitForBalance == true)
      {
        if (RV_SUCCESS != ctrlGsmStateSend())
        {
         LOG_TRACE(CONTROL_CMP,"Could not get gTrack state");
        }

        ctrlWaitForBalance = false;
      }
      break;

    case LOW_VOLTAGE_EVENT:
      if (RV_SUCCESS != underVoltageProcess())
      {
        LOG_TRACE(CONTROL_CMP,"Under voltage event send failed");
      }
      break;

    default:
      LOG_TRACE(CONTROL_CMP,"UNSUPPORTED event!");
      break;
  }

  return RV_SUCCESS;
}

RV_t doStateTest(ctrl_sm_event_t ev, ctrl_sm_state_t* state)
{
  (void) ev;
  (void) state;
  /* this state does not have an handler. Test state is used only to trigger
     get functions for each sensor only once and then return to previous state */

  return RV_SUCCESS;
}

/* make MT safe */
uint32_t logTimeStampGet(void)
{
  RTCDateTime timp;
  static uint32_t timestamp_g = 0;

  rtcGetTime(&RTCD1, &timp);

  if (timestamp_g == 0)
  {
    timestamp_g = timp.millisecond;
  }

  return (timp.millisecond - timestamp_g) / 60000;
}

static RV_t ctrlGsmEventUpProcess(void)
{
  return ctrlEventPost(START_EVENT);
}

static RV_t ctrlGsmEventDownProcess(void)
{
  return ctrlEventPost(STOP_EVENT);
}

static RV_t ctrlGsmEventBalanceProcess(void)
{
  return ctrlEventPost(BALANCE_EVENT);
}

static RV_t ctrlGsmEventSmsStartProcess(void)
{
  return ctrlEventPost(START_EVENT);
}

static RV_t ctrlGsmEventSmsStopProcess(void)
{
  return ctrlEventPost(STOP_EVENT);
}

static RV_t ctrlGsmEventSmsStateProcess(void)
{
  return ctrlEventPost(STATE_EVENT);
}

static RV_t ctrlImuEventAlarmProcess(void)
{
  return ctrlEventPost(ALARM_EVENT);
}

static RV_t ctrlVoltageEventAlarmProcess(void)
{
  return ctrlEventPost(LOW_VOLTAGE_EVENT);
}

/* read GSM battery discharge, signal level, SIM card balance */
static RV_t ctrlGsmStateSend()
{
  uint32_t signal = 0;
  uint32_t battery = 0;
  char buf[96] = "Balance (UAH): ";
  char temp[32] = {0};

  if (RV_SUCCESS == gsmStateReqGet(&signal, &battery, temp, sizeof(temp)))
  {
    STRCAT_SAFE(buf, temp);

    osapiItoa(signal, temp, sizeof(temp));
    STRCAT_SAFE(buf, "\r\nSIGNAL: ");
    STRCAT_SAFE(buf, temp);

    osapiItoa(battery, temp, sizeof(temp));
    STRCAT_SAFE(buf, "\r\nBATTERY (%): ");
    STRCAT_SAFE(buf, temp);

    STRCAT_SAFE(buf, "\r\nSTATE: ");
    strncpy(temp, ctrlCurStateToStr(), sizeof(temp));
    STRCAT_SAFE(buf, temp);

    if (RV_SUCCESS != gsmSmsSend(buf))
    {
      LOG_TRACE(CONTROL_CMP,"Failed to send GSM state!");
      return RV_FAILURE;
    }

    //gsmModuleConnectGprs();
    //gsmModuleSendGetHttpRequest(signal, battery);
  }
  else
  {
    LOG_TRACE(CONTROL_CMP,"Failed to get GSM state");
    return RV_FAILURE;
  }
  return RV_SUCCESS;
}

static RV_t underVoltageProcess(void)
{
  LOG_TRACE(CONTROL_CMP, "Low battery voltage!");

  if (RV_SUCCESS != gsmSmsSend("Low battery voltage!"))
  {
    LOG_TRACE(CONTROL_CMP,"Low voltage msg send failed!");
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}

void profileInit(void)
{
  ctrlStateInitStateSet(IDLE_STATE);
  ctrlStateAdd(IDLE_STATE, doStateIdle, "IDLE");
  ctrlStateAdd(ACTIVE_STATE, doStateActive, "ACTIVE");
  ctrlStateAdd(ALARM_STATE, doStateAlarm, "ALARM");
  ctrlStateAdd(TEST_STATE, doStateTest, "TEST");

  cliCmdRegister("start", ctrlGsmEventSmsStartProcess);
  cliCmdRegister("stop", ctrlGsmEventSmsStopProcess);
  cliCmdRegister("state", ctrlGsmEventSmsStateProcess);

  gsmRegisterEventCb(GSM_EVENT_UP, ctrlGsmEventUpProcess);
  gsmRegisterEventCb(GSM_EVENT_DOWN, ctrlGsmEventDownProcess);
  gsmRegisterEventCb(GSM_EVENT_SMS_START, ctrlGsmEventSmsStartProcess);
  gsmRegisterEventCb(GSM_EVENT_SMS_STOP, ctrlGsmEventSmsStopProcess);
  gsmRegisterEventCb(GSM_EVENT_SMS_STATE, ctrlGsmEventSmsStateProcess);
  gsmRegisterEventCb(GSM_EVENT_BALANCE_SIGN_BATT, ctrlGsmEventBalanceProcess);
  gsmRegisterEventCb(GSM_EVENT_POWER_LOW, ctrlVoltageEventAlarmProcess);
  gsmRegisterEventCb(GSM_EVENT_VOICE_CALL, gsmVoiceCallHandle);

  imuRegisterEventCb(IMU_EVENT_ALARM, ctrlImuEventAlarmProcess);

  cnfgrRegister("Control",      ctrlAppInit);
  cnfgrRegister("GSM",          gsmInit);  
  cnfgrRegister("IMU",          accelGyroInit);
  cnfgrRegister("BSP_INIT_FIN", bspInitComplete);
}
