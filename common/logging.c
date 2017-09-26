/*****************************************************************************
* File:       logging.c
*
* Created on: Jul 26, 2016
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
#include "common.h"
#include "logging.h"

#if defined(DEBUG) && !defined(DEBUG_INLINE)

#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "memstreams.h"
#include "chprintf.h"
#include "bsp.h"

#define LOG_MSG_QUEUE_SIZE 15
#define LOG_BUF_SIZE       64

mailbox_t logMsg;
static msg_t logMsgQueue[LOG_MSG_QUEUE_SIZE];
static THD_WORKING_AREA(logAppThread, LOGGING_THREAD_STACK_SIZE);

static THD_FUNCTION(logAppTask, arg)
{
  (void) arg;
  msg_t data = 0;
  msg_t resp = 0;
  char buf[32] = {0};

  while (1)
  {
    /* wait for event */
    if ((resp = chMBFetch(&logMsg, &data, TIME_INFINITE)) >= Q_OK)
    {
      (void) logTimeStampGet(buf, sizeof(buf));

      chMtxLock(&gSDMutex);

      chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), (char *) "<");
      chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), buf);
      chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), (char *) "> ");

      chprintf(((BaseSequentialStream *) &CLI_SERIAL_PORT), (char *) data);

      chMtxUnlock(&gSDMutex);
      if (data != 0)
      {
        chHeapFree((void *) data);
      }
    }
  }
}

void logEvent(const char *msg, ...)
{
  char                  logBuf[LOG_BUF_SIZE] = {0};
  char                  *pBuf                = 0;
  uint32_t              bytesWritten         = 0;
  MemoryStream          ms;
  va_list               ap;
  BaseSequentialStream *chp;

  /* Memory stream object to be used as a string writer */
  msObjectInit(&ms, (uint8_t *) logBuf, LOG_BUF_SIZE, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *) &ms;

  va_start(ap, msg);
  bytesWritten = chvprintf(chp, msg, ap);
  va_end(ap);

  logBuf[bytesWritten] = '\0';

  bytesWritten++;

  pBuf = (char *) chHeapAlloc(0, bytesWritten);
  if (pBuf == NULL)
  {
    chMtxLock(&gSDMutex);
    chprintf((BaseSequentialStream *) &CLI_SERIAL_PORT, "Error: %s (%u): Failed to allocate from heap!\r\n",
             __FUNCTION__, __LINE__);
    chMtxUnlock(&gSDMutex);

    return;
  }

  memset(pBuf, 0x00, bytesWritten);

  strncpy(pBuf, logBuf, bytesWritten);

  if (chMBPost(&logMsg, (msg_t) pBuf, TIME_IMMEDIATE) != MSG_OK)
  {
    chHeapFree((void *) pBuf);
    return;
  }
}

RV_t loggingAppInit(void)
{
  chMBObjectInit(&logMsg, logMsgQueue, LOG_MSG_QUEUE_SIZE);

  /* Create thread */
  chThdCreateStatic(logAppThread, sizeof(logAppThread), NORMALPRIO+1, logAppTask, 0);

  return RV_SUCCESS;
}

#else
RV_t loggingAppInit(void)
{
  /* do nothing */
  return RV_SUCCESS;
}
#endif /* #ifndef DEBUG_INLINE */
