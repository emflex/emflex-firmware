/*****************************************************************************
* File:       utils.h
*
* Created on: Apr 14, 2016
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

#ifndef UTILS_H
#define UTILS_H

#include <string.h>
#include <stdint.h>
#include "common.h"

extern RV_t floatToStr(float f, char *rez, int len);

extern char *osapiItoa(int val, char *rez, uint32_t rezLen);

extern void debugStackDepth(uint8_t thread_id, const uint8_t *wa_addr, const uint32_t total_stack_size);

/* Assumes (d) is an array and not a pointer */
#define STRCAT_SAFE(d, s) strncat((d), (s), sizeof(d) - strlen(d))

#endif
