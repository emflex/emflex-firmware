/*****************************************************************************
* File:       eeprom.h
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
******************************************************************************/

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

#define EEPROM_SIZE     4096
#define LOG_ENTRY_SIZE  96

/*
 * Read 'num_bytes' from 'addr' and store result in 'buf'
 * Return 0 if success or -1 in case of failure
 */
int eeprom_read(uint32_t addr, uint8_t *buf, uint32_t num_bytes);

/*
 * Write 'num_bytes' from 'buf' buffer to 'addr'address
 * Return 0 in case of success and -1 in case of failure
 */
int eeprom_write(uint32_t addr, uint8_t *buf, uint32_t num_bytes);


#endif // EEPROM_H

