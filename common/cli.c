/*****************************************************************************
* File:       control.c
*
* Created on: Mar 19, 2016
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
#include <string.h>
#include "common.h"
#include "logging.h"
#include "bsp.h"
#include "cli.h"
#include "utils.h"

static uint32_t emptyElement = 0;
static cliEntry_t cliCmdsList[CLI_CMD_MAX_NUMBER];

static THD_WORKING_AREA(cliThread, CLI_THREAD_STACK_SIZE);

extern mutex_t gSDMutex;

static RV_t cliEchoReturn(char *buf, uint32_t len)
{
  uint32_t i = 0;

  for (i = 0; i < len; i++)
  {
    chMtxLock(&gSDMutex);
    if (buf[i] != '\r')
    {      
      sdPut(&CLI_SERIAL_PORT, buf[i]);
    }
    else
    {
      sdAsynchronousWrite(&CLI_SERIAL_PORT, (uint8_t *) "\r\n", sizeof("\r\n")-1);      
    }
    chMtxUnlock(&gSDMutex);
  }
  return RV_SUCCESS;
}

static RV_t cliCmdDispatch(const char *cmdBuf, uint32_t len)
{
  uint32_t i = 0;

  if (!cmdBuf)
  {
    return RV_FAILURE;
  }

  //LOG_TRACE(CLI_CMP, "%s %u", cmdBuf, len);

  /* check if only CR was pressed */
  if (!len)
  {
    return RV_SUCCESS;
  }

  for (i = 0; i < CLI_CMD_MAX_NUMBER; i++)
  {
    if (0 == strncmp(cmdBuf, cliCmdsList[i].cmd, len+1))
    {
      return cliCmdsList[i].cb();
    }
  }

  return RV_FAILURE;
}

static RV_t cliCmdParse(char *data, uint32_t len)
{
  (void) len;

  static char leftOverBuf[64];
  char operData[MAX_BUF_LEN] = {0};
  char *pp = 0, *p = operData;
  uint32_t size = 0;

  if (!data)
  {
    return RV_FAILURE;
  }

  if (leftOverBuf[0] != 0)
  {
    strncpy(operData, leftOverBuf, sizeof(operData));
  }
  strncat(operData, data, sizeof(operData)-strlen(operData));

  while (*p != '\0')
  {
    pp = strrchr(p, '\r');
    if (pp)
    {
      size = pp-p;
      memcpy(leftOverBuf, p, size);
      leftOverBuf[size] = '\0';

      cliCmdDispatch(leftOverBuf, size);
    }
    else
    {
      size = strlen(p);
      memcpy(leftOverBuf, p, size);
      leftOverBuf[size] = '\0';
      return RV_NOT_COMPLETED;
    }
    p = pp;
    p++;
  }

  leftOverBuf[0] = 0;

  return RV_SUCCESS;
}

static THD_FUNCTION(cliTask, arg)
{
  (void) arg;

  char buf[MAX_BUF_LEN] = {0};
  uint32_t cliInByteNum = 0;

  /* display user prompt */
  chMtxLock(&gSDMutex);
  sdPut(&CLI_SERIAL_PORT, '>');
  chMtxUnlock(&gSDMutex);

  while (1)
  {
#ifdef DEBUG
    debugStackDepth(CLI_CMP, (uint8_t *) &cliThread, sizeof(cliThread));
#endif

    chThdSleepMilliseconds(300);

    chMtxLock(&gSDMutex);
    cliInByteNum = sdAsynchronousRead(&CLI_SERIAL_PORT, (uint8_t *) buf, sizeof(buf));
    chMtxUnlock(&gSDMutex);

    if (cliInByteNum)
    {
      /* echo received data */
      buf[cliInByteNum] = '\0';
      cliEchoReturn(buf, cliInByteNum);

      if (RV_SUCCESS == cliCmdParse(buf, cliInByteNum))
      {
        chMtxLock(&gSDMutex);
        sdPut(&CLI_SERIAL_PORT, '>');
        chMtxUnlock(&gSDMutex);
      }
    }

    memset(buf, 0x00, sizeof(buf));
    cliInByteNum = 0;

#ifdef DEBUG
    debugStackDepth(CLI_CMP, (uint8_t *) &cliThread, sizeof(cliThread));
#endif
  }
}

RV_t cliInit(void)
{
  /* Create thread */
  chThdCreateStatic(cliThread, sizeof(cliThread), NORMALPRIO+1, cliTask, 0);

  return RV_SUCCESS;
}

RV_t cliCmdRegister(const char* cmd, cli_cb_t cb)
{
  if (emptyElement >= CLI_CMD_MAX_NUMBER)
  {
    return RV_FAILURE;
  }

  cliCmdsList[emptyElement].cmd = cmd;
  cliCmdsList[emptyElement].cb = cb;

  emptyElement++;

  return RV_SUCCESS;
}
