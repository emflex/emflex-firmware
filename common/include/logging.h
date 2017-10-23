/*****************************************************************************
* File:       logging.h
* Created on: Jul 26, 2016
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

#ifndef LOG_API
#define LOG_API

#include <string.h>
#include <stdio.h>
#include "common.h"
#include "chprintf.h"
#include "bsp.h"
#include "persist_log.h"

extern void logEvent(const char *msg, ...);
extern void logInline(const char *fmt, ...);
extern void logError(const char *fmt, ...);
extern RV_t loggingAppInit(void);
extern RV_t persistentLogProcess(void);

extern mutex_t gSDMutex;

#ifdef DEBUG

#ifdef DEBUG_INLINE
#define LOG_TRACE(cmp, msg, ...)                                                       \
    logInline("DEBUG: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define LOG_ERROR(cmp, msg, ...)                                                       \
    logInline("ERROR: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);    \
    logError("ERROR: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#else
#define LOG_TRACE(cmp, msg, ...)                                                       \
   logEvent("DEBUG: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define LOG_ERROR(cmp, msg, ...)                                                       \
   logEvent("ERROR: %s(%u): " msg "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#endif /*DEBUG_INLINE*/

#else /*DEBUG*/

#define LOG_TRACE(cmp, msg, ...)
#define LOG_ERROR(cmp, msg, ...)

#endif /*DEBUG*/

#endif /*LOG_API*/
