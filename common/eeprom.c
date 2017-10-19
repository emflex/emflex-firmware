/*****************************************************************************
* File:       eeprom.c
* Created:    Sept 30, 2017
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
*******************************************************************************/

#include "board.h"
#include "stm32l1xx_flash.h"
#include "eeprom.h"

#define EEPROM_START 0x08080000
#define EEPROM_END   0x08080FFF

/*
 * Read 'num_bytes' from 'addr' and store result in 'data'
 * Return 0 if success or -1 in case of failure
 */
int eeprom_read(uint32_t addr, uint8_t *data, uint32_t num_bytes)
{
  if (!data)
  {
    return -1;
  }

  for (uint32_t i = 0; i < num_bytes; i++)
  {
    *data = *((volatile uint8_t *) (EEPROM_START+addr+i));
    data++;
  }

  return 0;
}

/*
 * Write 'num_bytes' from 'data' buffer to 'addr' address
 * Return 0 in case of success and -1 in case of failure
 */
int eeprom_write(uint32_t addr, uint8_t *data, uint32_t num_bytes)
{
  if (!data)
  {
    return -1;
  }

  DATA_EEPROM_Unlock();

  for (uint32_t i = 0; i < num_bytes; i++)
  {
    if (FLASH_COMPLETE != DATA_EEPROM_ProgramByte((EEPROM_START+addr+i), *(data+i)))
    {
      /* do something */
    }
  }

  DATA_EEPROM_Lock();

  return 0;
}
