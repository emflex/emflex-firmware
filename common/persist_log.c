/*****************************************************************************
* File:        persist_log.c
* Created:     Sept 30, 2017
* Description: Persistent storage is implemented as a circular buffer.
*
* Author:      Rostyslav Spolyak
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
*******************************************************************************/

/*
 * EEPROM (or other persistent memory) has finite size. So it is concidered as
 * a circular buffer where last memory sell is connected with first memory cell.
 *
 * First eight bytes are allocated for buffer head and tail correspondingly
 * (4 bytes each). They are retrieved at device start up to retain configuration
 * as it was before reboot.
 *
 * STM32L152's EEPROM contains 4096 bytes.
 * Devide it into 96 bytes chunks which gives nearly 42 log entries
 *
 *   8    96      96      96        96
 *  _____________________________________
 * |RES|  0  |        |          |  41   |
 * |___|_____|________|__________|_______|
 *        ^                ^
 *        |                |
 *       head             tail
 *
 * Add operation increase p_tail, get operation increase p_head variable
 *
 */

#include <stdint.h>
#include "eeprom.h"

#define EEPROM_OFFSET      8
#define LOG_ENTRY_SIZE     96
#define BUFFER_LENGTH      (EEPROM_SIZE / LOG_ENTRY_SIZE)

static uint32_t p_head;
static uint32_t p_tail;

/*
 *
 */
void persist_init(void)
{
   if (0 != eeprom_read(0, (void *) &p_head, sizeof(p_head)) ||
       0 != eeprom_read(4, (void *) &p_tail, sizeof(p_tail)))
   {
     /* log message */
     return;
   }
}

/*
 * Store log message in persistent storage device (EEPROM as of now)
 * Up to LOG_ENTRY_SIZE bytes are written. Remaining part will be truncated.
 */
int persist_add(const char *data, uint32_t data_len)
{
  uint32_t addr = 0;
  uint32_t length = 0;

  if (!data)
  {
    /* log message */
    return -1;
  }

  addr = EEPROM_OFFSET + (p_tail * LOG_ENTRY_SIZE);
  length = (data_len > LOG_ENTRY_SIZE) ? LOG_ENTRY_SIZE : data_len;

  if (0 != eeprom_write(addr, (uint8_t *) data, length))
  {
    /* log message */
    return -1;
  }

  p_tail = (p_tail+1) % BUFFER_LENGTH;

  if (p_tail == p_head)
  {
    p_head = (p_head+1) % BUFFER_LENGTH;
  }

  if (0 != eeprom_write(0, (void *) &p_head, sizeof(p_head)) ||
      0 != eeprom_write(4, (void *) &p_tail, sizeof(p_tail)))
  {
    /* log message */
    return -1;
  }

  return 0;
}

/*
 * Retrieve next log message from persistent storage device.
 */
int persist_get_next(char *data, uint32_t data_len)
{
  uint32_t addr = 0;

  if (!data)
  {
    /* log message */
    return 0;
  }

  if (data_len > LOG_ENTRY_SIZE)
  {
    /* log message */
    return 0;
  }

  if (p_tail != p_head)
  {
    addr = EEPROM_OFFSET + (p_head * LOG_ENTRY_SIZE);

    eeprom_read(addr, (void *) data, data_len);

    p_head = (p_head+1) % BUFFER_LENGTH;

    if (0 != eeprom_write(0, (void *) &p_head, sizeof(p_head)) ||
        0 != eeprom_write(4, (void *) &p_tail, sizeof(p_tail)))
    {
      /* log message */
      return -1;
    }

    return 1;
  }
  else
  {
    /* buffer empty log message */
    return 0;
  }
}
