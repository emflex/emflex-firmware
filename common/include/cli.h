/*****************************************************************************
* File:       cli.h
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
#ifndef CLI_H
#define CLI_H

#include "common.h"

#define CLI_CMD_MAX_NUMBER 20

#define CLI_SERIAL_SPEED 38400

typedef RV_t (*cli_cb_t)(void);
typedef RV_t (*cli_var_cb_t)(const char *);

typedef struct cliEntry_s
{
  const char* cmd;
  cli_cb_t cb;
} cliEntry_t;

typedef struct
{
  const char* cmd;
  cli_var_cb_t cb;
} cliVarEntry_t;

RV_t cliInit(void);
RV_t cliCmdRegister(const char* cmd, cli_cb_t cb);
RV_t cliVarCmdRegister(const char* cmd, cli_var_cb_t cb);

#endif
