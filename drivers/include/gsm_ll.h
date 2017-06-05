/*****************************************************************************
* File:       gsm_ll.h
*
* Created on: May 01, 2015
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

#ifndef GSM_LL
#define GSM_LL

/**
 * @brief   CLI serial speed.
 */
#define GSM_SERIAL_SPEED    19200

#define GSM_THREAD_STACK_SIZE   1200

#define MED_BUF_LEN 64
#define GSM_SMS_INTERNAL_BLOCKS 3

#define MAILBOX_QUEUE_TX_SIZE   20
#define MAX_GSM_CMD_LEN         128
#define GSM_CTRL_CMD_LEN        32

#define GSM_READ_TIMEOUT  1000
#define GSM_WRITE_TIMEOUT 1000

#define SEND_SMS_STR            "AT+CMGS=\""

#define GSM_NEW_MSG_EVENT       "+CMT"
#define GSM_CPIN_RDY_EVENT      "+CPIN: READY"
#define GSM_RDY_EVENT           "RDY"
#define GSM_CFUN_EVENT          "+CFUN: 1"
#define GSM_CALL_RDY_EVENT      "Call Ready"
#define GSM_POWER_DOWN_EVENT    "NORMAL POWER DOWN"
#define GSM_POWER_WARN_EVENT    "UNDER-VOLTAGE WARNNING"
#define GSM_REPLY_OK            "OK"
#define GSM_REPLY_DOWNLOAD      "DOWNLOAD"
#define GSM_REPLY_ERROR         "ERROR"
#define GSM_MSG_CMT             "+CMT:"
#define GSM_MSG_CMTISM          "+CMTISM"
#define GSM_BALANCE_RESPONSE    "+CUSD"
#define GSM_BALANCE_MATCH_TYPE  "+CUSD: "
#define GSM_BALANCE_MATCH_STR   "rahunku "
#define GSM_PHONE_BOOK_READ_MATCH_STR   "+CPBR"
#define GSM_MODULE_MODEL_GET    "AT+GSV\r"
#define GSM_MODULE_VENDOR_NAME  "SIMCOM"
#define GSM_NO_CARRIER          "NO CARRIER"

#define GSM_CARRET_RETURN       "\r"
#define GSM_ECHO_ENABLE         "ATE1\r"
#define GSM_ECHO_DISABLE        "ATE0\r"
#define GSM_TA_RESPONSE_FORMAT_ENABLE        "ATV1\r"
#define GSM_TA_RESPONSE_FORMAT_DISABLE       "ATV0\r"
#define GSM_SMS_TEXT_MESSAGE_FORMAT "AT+CMGF=1\r"
#define GSM_SMS_PDU_MESSAGE_FORMAT  "AT+CMGF=0\r"
#define GSM_REPORT_ERROR_CODE_DISABLE "AT+CMEE=0\r"
#define GSM_REPORT_ERROR_CODE_ENABLE "AT+CMEE=1\r"
#define GSM_REPORT_ERROR_CODE_VERBOSE_ENABLE "AT+CMEE=2\r"

#define GSM_NEW_SMS_MESSAGE_INDICATION "AT+CNMI=1,2,0,0,0\r"

#define GSM_PHONEBOOK_CHECK "AT+CPBF=?\r"
#define GSM_PHONEBOOK_RANGE_CHECK "AT+CPBR=?\r"
#define GSM_PHONEBOOK_READ_ALL "AT+CPBR=1,250\r"
#define GSM_PHONEBOOK_STORAGE_CHECK "AT+CPBS=?\r"
#define GSM_PHONEBOOK_STORAGE_TEST "AT+CPBS?\r"
#define GSM_PHONEBOOK_WRITE_TEST "AT+CPBW=?\r"
#define GSM_PHONEBOOK_WRITE_ENTRY "AT+CPBW=,\"%s\",129,\"%s\"\r"
#define GSM_PHONEBOOK_READ_ENTRY "AT+CPBR=1\r"

#define GSM_SYNCHR_BAUD_RATE "AT\r"
#define GSM_CHECK_GSM_STATE "AT+CPIN?\r"
#define GSM_FIXED_BAUDRATE "AT+IPR=19200\r"
#define GSM_CHECK_SYGNAL_GPRS "AT+CSQ\r"
#define GSM_CHECK_GPRS_NETWORK "AT+CGATT?\r"
#define GSM_ATTACH_GPRS_NETWORK "AT+CGATT=1\r"
#define GSM_CONTYPE_GPRS "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r"
#define GSM_CONTYPE_APN "AT+SAPBR=3,1,\"APN\",\"internet\"\r"
#define GSM_GPRS_CONNECT "AT+SAPBR=1,1\r"
#define GSM_GPRS_CHECK "AT+SAPBR=2,1\r"
#define GSM_ENABLE_HTTP_SERVICE "AT+HTTPINIT\r"

#define GSM_HTTP_SET_BEARER_PROFILE_ID "AT+HTTPPARA=\"CID\",1\r"
#define GSM_HTTP_SET_SSL "AT+HTTPSSL=1\r"
//#define GSM_HTTP_SET_URL "AT+HTTPPARA=\"URL\",\"heroku-cartracker.herokuapp.com/?id=552&name=Test2\"\r"
#define GSM_HTTP_SET_URL "AT+HTTPPARA=\"URL\",\"https://dweet.io/dweet/for/emflex?"
#define GSM_HTTP_SET_GET_METHOD "AT+HTTPACTION=0\r"
#define GSM_HTTP_READ_DATA "AT+HTTPREAD=0,1000\r"

#define GSM_HTTP_SET_POST_DATA_LENGTH "AT+HTTPDATA=50,10000\r"
#define GSM_HTTP_CHECK_POST_DATA "AT+HTTPDATA=?\r"

#define GSM_HTTP_SET_POST_METHOD "AT+HTTPACTION=1\r"
#define GSM_HTTP_SET_HEAD_METHOD "AT+HTTPACTION=2\r"
#define GSM_HTTP_CHECK_DATA "AT+HTTPREAD=?\r"

#define GSM_DISABLE_HTTP_SERVICE "AT+HTTPTERM\r"
#define GSM_GPRS_DISCONNECT "AT+SAPBR=0,1\r"

#define GSM_SLEEP_MODE "AT+CSCLK=2\r"
/*enter sleep mode 1 (controlled via DTR line) */
#define GSM_SLEEP_MODE_DTR "AT+CSCLK=1\r"

#define GSM_BATTERY_CMD_RESPONCE        "+CBC"
#define GSM_SIGNAL_CMD_RESPONCE         "+CSQ"

#define GSM_LEGACY_SMS_CLEAR            "AT+CMGD=1,4\r"

#define GSM_NETWORK_STATUS               "AT+CREG?\r"
#define GSM_NETWORK_STATUS_STR           "+CREG"
#define GSM_NETWORK_MATCH_TYPE           "+CREG: "

RV_t gsmTaskCb(const char *in);
RV_t gsmPhoneNumberParse(const char* buf, char* number);
RV_t gsmPhoneNumberAdd(const char* number);
void gsmModuleConnectGprs(void);
void gsmModuleSendGetHttpRequest(uint8_t signal, uint8_t battery);

#define START_CMD "start"
#define STOP_CMD  "stop"
#define STATE_CMD "state"
#define REBOOT_CMD "reboot"

#endif
