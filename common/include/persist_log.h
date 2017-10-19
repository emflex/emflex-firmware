/*****************************************************************************
* File:       persist_log.h
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
******************************************************************************
*/

#ifndef PERSIST_LOG_H
#define PERSIST_LOG_H

void persist_init(void);
int persist_add(const char *data, uint32_t data_len);
int persist_get_next(char *data, uint32_t data_len);

#endif // PERSIST_LOG_H

