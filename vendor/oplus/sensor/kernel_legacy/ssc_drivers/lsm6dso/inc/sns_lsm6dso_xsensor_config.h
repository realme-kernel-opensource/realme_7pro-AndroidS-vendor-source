/**
 * @file sns_lsm6dso_oem.h
 *
 * OEM to add MLC related info
 *
 * Copyright (c) 2020, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#include "sns_lsm6dso_sensor_instance.h"
#ifndef LSM6DSO_XSENSOR_CONFIG_H
#define LSM6DSO_XSENSOR_CONFIG_H 1
#if (LSM6DSO_MLC_ENABLED | LSM6DSO_FSM_ENABLED)

/* Fixed code - Not expected to change */
#define LSM6DSO_DATA_MAX_SIZE				2048

struct lsm6dso_sensor_data {
  uint8_t data[LSM6DSO_DATA_MAX_SIZE];
  uint16_t len;
};

struct lsm6dso_xsensor {
  uint16_t id;
};


#define MLC_SENSOR_CNT ARR_SIZE(lsm6dso_mlc_sensor_list)
#define FSM_SENSOR_CNT ARR_SIZE(lsm6dso_fsm_sensor_list)
/* Variable code to be updated by OEM */

/* XSENSOR_1 related flags */
#define LSM6DSO_XSENSOR_1_NAME  "stm_activity_fsm"
#define LSM6DSO_XSENSOR_1_STREAM_TYPE  SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT
#define LSM6DSO_XSENSOR_1_INT   XSENSOR_INT_1
#define LSM6DSO_XSENSOR_1_TYPE  XSENSOR_TYPE_FSM /* XSensor 1  is FSM */
#define LSM6DSO_XSENSOR_1_GYRO_REQ 0

/* XSENSOR_2 related flags */
#define LSM6DSO_XSENSOR_2_NAME  "stm_inactivity_fsm"
#define LSM6DSO_XSENSOR_2_STREAM_TYPE  SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT
#define LSM6DSO_XSENSOR_2_INT   XSENSOR_INT_1
#define LSM6DSO_XSENSOR_2_TYPE  XSENSOR_TYPE_FSM /* XSensor 2  is FSM */
#define LSM6DSO_XSENSOR_2_GYRO_REQ 0

/* ODR to be used by FSM/MLC - based on ODR that all MLC/FSM were designed for */
#define LSM6DSO_FSM_MLC_ODR            26.0

/* USE COMBINED FSM/MLC */
#define LSM6DSO_MLC_FSM_COMBINE     0

static const struct lsm6dso_xsensor lsm6dso_mlc_sensor_list[] = {
};

static const struct lsm6dso_xsensor lsm6dso_fsm_sensor_list[] = {
  //Activity
  {
    .id = LSM6DSO_XSENSOR_1,
  },
  //Inactivity
  {
    .id = LSM6DSO_XSENSOR_2,
  },
};

static const struct lsm6dso_sensor_data lsm6dso_xsensor_data = {
  /* Activity FSM, Inactivity FSM*/
  .data = {
    0x01,0x80,0x04,0x00,0x05,0x00,0x5F,0x4B,
    0x46,0x00,0x47,0x00,0x0A,0x00,0x0B,0x01,
    0x0C,0x00,0x0E,0x00,0x0F,0x02,0x10,0x00,
    0x02,0x01,0x17,0x40,0x09,0x00,0x02,0x11,
    0x08,0x7A,0x09,0x00,0x09,0x00,0x09,0x02,
    0x09,0x02,0x09,0x00,0x09,0x04,0x02,0x41,
    0x08,0x00,0x09,0x50,0x09,0x00,0x09,0x0E,
    0x09,0x00,0x09,0x0A,0x09,0x00,0x09,0x66,
    0x09,0x3C,0x09,0x02,0x09,0x00,0x09,0x05,
    0x09,0x99,0x09,0x00,0x09,0x00,0x09,0x54,
    0x09,0x00,0x09,0x12,0x09,0x00,0x09,0x0E,
    0x09,0x00,0x09,0x66,0x09,0x3C,0x09,0x02,
    0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x82,
    0x09,0x00,0x09,0x51,0x09,0x99,0x09,0x00,
    0x09,0x00,0x04,0x00,0x05,0x01,0x17,0x80,
    0x02,0x01,0x01,0x00,
  },
  .len = 124,
};
#endif //(LSM6DSO_MLC_ENABLED | LSM6DSO_FSM_ENABLED)
#endif //LSM6DSO_XSENSOR_H
