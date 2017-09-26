/*****************************************************************************
* File:       logging.h
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

#ifndef LOG_API
#define LOG_API

#include <string.h>
#include <stdio.h>
#include "common.h"
#include "chprintf.h"
#include "bsp.h"

extern void logEvent(const char *msg, ...);
extern mutex_t gSDMutex;
extern RV_t logTimeStampGet(char *buf, uint32_t len);
extern RV_t loggingAppInit(void);

/* print logs directly to serial port */
#define DEBUG_INLINE

#ifdef DEBUG

#ifdef DEBUG_INLINE
#define LOG_TRACE(cmp, msg, ...)                                                                \
   do                                                                                           \
   {                                                                                            \
     char timeBuf[32] = {0};                                                                    \
     logTimeStampGet(timeBuf, sizeof(timeBuf));                                                 \
     chMtxLock(&gSDMutex);                                                                      \
     chprintf((BaseSequentialStream *) &CLI_SERIAL_PORT, "<%s> DEBUG: %s(%u): " msg "\r\n",     \
               timeBuf, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                 \
     chMtxUnlock(&gSDMutex);                                                                    \
   }                                                                                            \
   while(0);
#else
#define LOG_TRACE(cmp, msg, ...)            \
   logEvent("DEBUG: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#endif /*DEBUG_INLINE*/

#else

#define LOG_TRACE(cmp, msg, ...)

#endif /*DEBUG*/

#ifdef DEBUG_INLINE
#define LOG_ERROR(cmp, msg, ...)                                                       \
    do                                                                                           \
    {                                                                                            \
      char timeBuf[32] = {0};                                                                    \
      logTimeStampGet(timeBuf, sizeof(timeBuf));                                                 \
      chMtxLock(&gSDMutex);                                                                      \
      chprintf((BaseSequentialStream *) &CLI_SERIAL_PORT, "<%s> ERROR: %s(%u): " msg "\r\n",     \
                timeBuf, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                 \
      chMtxUnlock(&gSDMutex);                                                                    \
    }                                                                                            \
    while(0);
#else
#define LOG_ERROR(cmp, msg, ...)                                                       \
    logEvent("ERROR: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#endif /*DEBUG_INLINE*/

#endif /*LOG_API*/
