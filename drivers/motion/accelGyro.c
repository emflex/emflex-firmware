/*****************************************************************************
* File:       accelGyro.c
*
* Created on: Aug 21, 2015
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
#include <stdint.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "logging.h"
#include "common.h"
#include "mpu6050.h"
#include "accelGyro.h"
#include "utils.h"
#include "i2c_api.h"
#include "bsp.h"

imuCbFunc_t imuCbArray_g[IMU_EVENT_LAST] = {0};

#define GYRO_SENSIV             (131.0)
#define ACCEL_SENS              (16384.0)
#define RAD2DEG                 (57.29)
#define IMU_GET_RATE_MSEC       (100)
#define MSG_SENT_INTERVAL_MSEC  (60000)

#define maxValSet(cur, max)     if ((fabs(cur)) > (fabs(max)))      \
                                {                                   \
                                  (max) = (cur);                    \
                                }

#define GYRO_I2C_TIMEOUT 1000

typedef struct imuThreshold_s
{
  float min;
  float max;
} imuThreshold_t;

static imuThreshold_t xThreshold;
static imuThreshold_t yThreshold;
const uint8_t numSamples = 100;

static bool accel_is_inited_g = FALSE;

static bool accel_is_enabled = FALSE;
static MUTEX_DECL(gyro_alarm_mutex);
static CONDVAR_DECL(gyro_alarm_cond_var);

static THD_WORKING_AREA(accelGyroThread, ACCEL_THREAD_STACK_SIZE);

static RV_t imuCallEventCb(imuEvent_t event);

static bool isAccelInited(void)
{
  return accel_is_inited_g;
}

static void accelInitedSet(bool value)
{
  accel_is_inited_g = value;
}

static RV_t accelGyroRead(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t *err)
{
  msg_t i2cRv = MSG_RESET;
  uint8_t regBuf = reg;

  if (out == 0 || err == 0)
  {
    return RV_FAILURE;
  }

  *err = 0;

  i2cRv = i2cMasterTransmitTimeout(&I2CD1, addr, &regBuf, 1, out, 1,
                                   GYRO_I2C_TIMEOUT);
  if (MSG_RESET == i2cRv)
  {
    *err = i2cGetErrors(&I2CD1);
    return RV_ERROR;
  }
  else if (MSG_RESET == i2cRv)
  {
     return RV_TIMEOUT;
  }
  else
  {
    return RV_SUCCESS;
  }
}

static RV_t accelGyroWrite(uint8_t addr, uint8_t reg, uint8_t val, uint8_t *err)
{
  RV_t rv = RV_SUCCESS;
  uint8_t *out = 0;
  uint8_t regBuf[2] = {reg, val};

  /* set number of bytes to be read as 0 in order to just write data to sensor
   * without further read */
  if (MSG_OK != i2cMasterTransmitTimeout(&I2CD1, addr, regBuf, 2, out, 0,
                                         GYRO_I2C_TIMEOUT))
  {
    *err = i2cGetErrors(&I2CD1);
    rv = RV_FAILURE;
  }
  return rv;
}
/* Get raw gyroscope value */
static RV_t gyroGet(rawImu_t *dof)
{
  RV_t rv = RV_SUCCESS;
  uint8_t GYRO_XOUT_H = 0;
  uint8_t GYRO_XOUT_L = 0;
  uint8_t GYRO_YOUT_H = 0;
  uint8_t GYRO_YOUT_L = 0;
  uint8_t GYRO_ZOUT_H = 0;
  uint8_t GYRO_ZOUT_L = 0;
  uint8_t err = 0;
  uint8_t line = 0;

  do
  {
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H,
                                    &GYRO_XOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L,
                                    &GYRO_XOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H,
                                    &GYRO_YOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L,
                                    &GYRO_YOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H,
                                    &GYRO_ZOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L,
                                    &GYRO_ZOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
  } while (0);

  if (RV_FAILURE == rv)
  {
    LOG_TRACE(GYRO_CMP, "Error! %s, %u: err=%u", __FUNCTION__, line, err);
    return RV_FAILURE;
  }

  dof->x = (int16_t)((GYRO_XOUT_H << 8) | GYRO_XOUT_L);
  dof->y = (int16_t)((GYRO_YOUT_H << 8) | GYRO_YOUT_L);
  dof->z = (int16_t)((GYRO_ZOUT_H << 8) | GYRO_ZOUT_L);

  return RV_SUCCESS;
}

static RV_t accel2angle(rawImu_t dof, float *p, float *fi)
{
  *p = RAD2DEG * atan(dof.x / sqrt((dof.y*dof.y) + (dof.z*dof.z)));
  *fi = RAD2DEG * atan(dof.y / sqrt((dof.x*dof.x) + (dof.z*dof.z)));

  return RV_SUCCESS;
}

/* Get raw accelerometer data */
static RV_t accelGet(rawImu_t *dof)
{
  RV_t rv = RV_SUCCESS;
  uint8_t ACCEL_XOUT_H = 0;
  uint8_t ACCEL_XOUT_L = 0;
  uint8_t ACCEL_YOUT_H = 0;
  uint8_t ACCEL_YOUT_L = 0;
  uint8_t ACCEL_ZOUT_H = 0;
  uint8_t ACCEL_ZOUT_L = 0;
  uint8_t err = 0;
  uint32_t line = 0;

  do
  {
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
                                    &ACCEL_XOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }

    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L,
                                    &ACCEL_XOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H,
                                    &ACCEL_YOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L,
                                    &ACCEL_YOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H,
                                    &ACCEL_ZOUT_H, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
    if (RV_SUCCESS != accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L,
                                    &ACCEL_ZOUT_L, &err))
    {
      line = __LINE__;
      rv = RV_FAILURE;
      break;
    }
  } while (0);

  if (RV_FAILURE == rv)
  {
    LOG_TRACE(GYRO_CMP, "Error! line=%u err=%u", line, err);
    return RV_FAILURE;
  }

  dof->x = (((int16_t)ACCEL_XOUT_H << 8) | ACCEL_XOUT_L);
  dof->y = (((int16_t)ACCEL_YOUT_H << 8) | ACCEL_YOUT_L);
  dof->z = (((int16_t)ACCEL_ZOUT_H << 8) | ACCEL_ZOUT_L);

  return RV_SUCCESS;
}

/* Get angle value from sensor */
static RV_t imuAngleGet(dof_t *dof)
{
  rawImu_t dof_local;
  float a_x = 0;
  float a_y = 0;
  float g_x = 0;
  float g_y = 0;
//  uint8_t tau = 1;
  const float sRate = 0.1;
  const float alfa = 0.98;//tau / (tau + sRate);

  if (!isAccelInited())
  {
    LOG_ERROR(GYRO_CMP, "Accelerometer is not inited!");
    return RV_NOT_READY;
  }

  if (RV_SUCCESS != accelGet(&dof_local))
  {
    return RV_FAILURE;
  }

  if (RV_SUCCESS != accel2angle(dof_local, &a_x, &a_y))
  {
    return RV_FAILURE;
  }

  if (RV_SUCCESS != gyroGet(&dof_local))
  {
    return RV_FAILURE;
  }

  g_x = a_x + (sRate * ((float)dof_local.x / GYRO_SENSIV));
  g_y = a_y + (sRate * ((float)dof_local.y / GYRO_SENSIV));

  dof->x = (alfa * g_x) + (1.0 - alfa) * a_x;
  dof->y = (alfa * g_y) + (1.0 - alfa) * a_y;

  return RV_SUCCESS;
}

/* Get average angle from sensor */
RV_t imuSumAngleGet(dof_t *dof)
{
  RV_t rv = RV_FAILURE;
  dof_t cur;
  dof_t average;
  uint32_t i = 0;

  if (!isAccelInited())
  {
    LOG_ERROR(GYRO_CMP, "Accelerometer is not inited!");
    return RV_NOT_READY;
  }

  cur.x = cur.y = cur.z = average.x = average.y = average.z = 0.0;

  for (i = 0; i < numSamples; i++)
  {
    if (RV_SUCCESS == (rv = imuAngleGet(&cur)))
    {
      average.x += cur.x;
      average.y += cur.y;
      average.z += cur.z;
    }
    else
    {
      LOG_TRACE(GYRO_CMP, "Error!");
    }
    chThdSleepMilliseconds(30);
  }
  dof->x = average.x / (float)numSamples;
  dof->y = average.y / (float)numSamples;
  dof->z = average.z / (float)numSamples;

  return RV_SUCCESS;
}

static THD_FUNCTION(accelGyroTask, arg)
{
  (void)arg;
  dof_t dof;
  dof_t sumDof;
  uint32_t i = 0;

  while (1)
  {
    imuWait();

#ifdef DEBUG
    debugStackDepth(IMU_CMP, (uint8_t *) &accelGyroThread, sizeof(accelGyroThread));
#endif

    memset(&sumDof, 0x00, sizeof(sumDof));
    for (i = 0; i < numSamples; i++)
    {
      if (RV_SUCCESS != imuAngleGet(&dof))
      {
        LOG_TRACE(GYRO_CMP,"Error!");
        continue;
      }

      sumDof.x += dof.x;
      sumDof.y += dof.y;
      sumDof.z += dof.z;
    }

    sumDof.x /= numSamples;
    sumDof.y /= numSamples;
    sumDof.z /= numSamples;

    /* if either x or y angle is out of allowed range - notify user */
    if (!((sumDof.x >= xThreshold.min) && (sumDof.x <= xThreshold.max))
        ||
        !((sumDof.y >= yThreshold.min) && (sumDof.y <= yThreshold.max)))
    {
      LOG_TRACE(GYRO_CMP, "ANGLE x=%f y=%f", sumDof.x, sumDof.y);

      imuCallEventCb(IMU_EVENT_ALARM);
    }

    chThdSleepMilliseconds(IMU_GET_RATE_MSEC);
  }
}

RV_t accelGyroInit(void)
{
  RV_t rv = RV_FAILURE;
  uint8_t err = 0;
  uint8_t rcvBuf[64] = {0};

  /* communication interface */
  i2cDrvInit();

  LOG_TRACE(GYRO_CMP, "Initializing Gyroscope");

  do
  {
    /* Check WHO_AM_I register */
    rv = accelGyroRead(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, rcvBuf, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "WHO_AM_I error.");
      if (rv == RV_ERROR)
      {
        LOG_TRACE(GYRO_CMP, "I2C error code %d", err);
      }
      break;
    }

    /* When DLPF is enabled, gyro sample rate is 1kHz. Used below equation
     * to get average sample rate: Gyroscope Output Rate / (1 + SMPLRT_DIV)
     * 1000/1+0 = 1000Hz
     */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "Could not write MPU6050_RA_SMPLRT_DIV");
      break;
    }

    /* Disable FSync - 5,4,3 bits. DLPF enable at 5Hz*/
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "Could not disable FSync");
      break;
    }

    /* Disable gyro self tests, scale of 250 degrees/s */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "Could not disable gyro self tests");
      break;
    }

    /* Disable accel self tests, scale of +-2g, no DHPF */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "error");
      break;
    }

    /* Setup INT pin and AUX I2C pass through */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "error");
      break;
    }

    /* Enable data ready interrupt */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "error");
      break;
    }

    /* Sets clock source to gyro reference w/ PLL */
    rv = accelGyroWrite(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x02, &err);
    if (rv != RV_SUCCESS)
    {
      LOG_TRACE(GYRO_CMP, "error");
      break;
    }

    chThdSleepMilliseconds(100);

    /* Create thread for processing data from accelerometer and gyroscope */
    chThdCreateStatic(accelGyroThread, sizeof(accelGyroThread), NORMALPRIO+1,
                      accelGyroTask, 0);

    accelInitedSet(TRUE);

    return RV_SUCCESS;

  } while (0);

  return RV_FAILURE;
}

RV_t imuEnable(void)
{
  if (!isAccelInited())
  {
    LOG_ERROR(GYRO_CMP, "Accelerometer is not inited!");
    return RV_NOT_READY;
  }

  chMtxLock(&gyro_alarm_mutex);

  accel_is_enabled = TRUE;

  chCondSignal(&gyro_alarm_cond_var);

  chMtxUnlock(&gyro_alarm_mutex);

  return RV_SUCCESS;
}

RV_t imuDisable(void)
{
  if (!isAccelInited())
  {
    LOG_ERROR(GYRO_CMP, "Accelerometer is not inited!");
    return RV_NOT_READY;
  }

  chMtxLock(&gyro_alarm_mutex);

  accel_is_enabled = FALSE;

  chCondSignal(&gyro_alarm_cond_var);

  chMtxUnlock(&gyro_alarm_mutex);

  return RV_SUCCESS;
}

RV_t imuWait(void)
{
  chMtxLock(&gyro_alarm_mutex);

  while (accel_is_enabled == FALSE)
  {
    chCondWait(&gyro_alarm_cond_var);
  }

  chMtxUnlock(&gyro_alarm_mutex);

  return RV_SUCCESS;
}

RV_t imuThresholdSet(dof_t dof, float k)
{
  if (!isAccelInited())
  {
    LOG_ERROR(GYRO_CMP, "Accelerometer is not inited!");
    return RV_NOT_READY;
  }

  xThreshold.min = (dof.x - k);
  xThreshold.max = (dof.x + k);
  yThreshold.min = (dof.y - k);
  yThreshold.max = (dof.y + k);

  return RV_SUCCESS;
}

/* Register callback to receive asynchronous events from accel module */
RV_t imuRegisterEventCb(imuEvent_t event, imuCbFunc_t cb)
{
  if ((event == IMU_EVENT_UNKNOWN) || (event == IMU_EVENT_LAST))
  {
      return RV_FAILURE;
  }

  imuCbArray_g[event] = cb;

  return RV_SUCCESS;
}

/* Call a callback */
static RV_t imuCallEventCb(imuEvent_t event)
{
  if ((event == IMU_EVENT_UNKNOWN) || (event == IMU_EVENT_LAST))
  {
    return RV_FAILURE;
  }

  return imuCbArray_g[event]();
}
