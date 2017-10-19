/*****************************************************************************
* File:       control.c
* Created on: May 09, 2015
* Author:     Rostyslav Spolyak
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
#include <math.h>
#include "common.h"
#include "logging.h"
#include "control_api.h"
#include "accelGyro.h"
#include "gsm_api.h"
#include "bsp.h"
#include "cli.h"
#include "utils.h"

#define ALERT_MSG_QUEUE_SIZE 5
#define CTRL_STATE_NUMBER 15

static mailbox_t alertMsg;
static msg_t alertMsgQueue[ALERT_MSG_QUEUE_SIZE];
static ctrl_sm_state_t c_state = 0;
static ctrl_state_info_t stateArray[CTRL_STATE_NUMBER];

RV_t ctrlStateInitStateSet(ctrl_sm_state_t state)
{
  c_state = state;
  return RV_SUCCESS;
}

RV_t ctrlStateAdd(ctrl_sm_state_t state, runState cb, const char* name)
{
  if (state < 0 || state >= CTRL_STATE_NUMBER)
  {
    LOG_TRACE(CONTROL_CMP, "Incorrect state %d", state);
    return RV_FAILURE;
  }

  stateArray[state].cb = cb;
  stateArray[state].name = name;

  return RV_SUCCESS;
}

const char* ctrlCurStateToStr()
{
  return stateArray[c_state].name;
}

void changeState(ctrl_sm_event_t ev)
{
  stateArray[c_state].cb(ev, &c_state);
}

static THD_WORKING_AREA(ctrlAppThread, CTRL_TASK_STACK_SIZE);

static THD_FUNCTION(ctrlAppTask, arg)
{
  (void)arg;

  msg_t resp = Q_OK;
  msg_t val = 0;

  while (1)
  {
    /* wait for event */
    if ((resp = chMBFetch(&alertMsg, &val, TIME_INFINITE)) >= Q_OK)
    {
      changeState(val);

#ifdef DEBUG
      debugStackDepth(CNTR_CMP, (uint8_t *) &ctrlAppThread, sizeof(ctrlAppThread));
#endif
    }
  }
}

RV_t ctrlAppInit(void)
{
  /* create message queue to retrieve alert messages from other tasks */
  chMBObjectInit(&alertMsg, alertMsgQueue, ALERT_MSG_QUEUE_SIZE);

  /* Create thread */
  chThdCreateStatic(ctrlAppThread, sizeof(ctrlAppThread), NORMALPRIO+1, ctrlAppTask, 0);

  return RV_SUCCESS;
}

RV_t ctrlEventPost(ctrl_sm_event_t event)
{
  msg_t resp = Q_OK;

  LOG_TRACE(CONTROL_CMP, "Posting event %d", event);

  resp = chMBPost(&alertMsg, event, TIME_IMMEDIATE);
  if (resp < Q_OK)
  {
    LOG_TRACE(CONTROL_CMP,"rv = %i", resp);
    return RV_FAILURE;
  }

  return RV_SUCCESS;
}
