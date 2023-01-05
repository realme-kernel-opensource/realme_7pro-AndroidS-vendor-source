#pragma once
/**
 * @file sns_lsm6dso_xsensor.h
 *
 * Common implementation for LSM6DSO XSensor - header
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

#include "sns_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"

/** Enable FSM/MLC based user defined sensors */
#if(LSM6DSO_ESP_XSENSOR_1 || \
    LSM6DSO_ESP_XSENSOR_2 || \
    LSM6DSO_ESP_XSENSOR_3 || \
    LSM6DSO_ESP_XSENSOR_4 || \
    LSM6DSO_ESP_XSENSOR_5 || \
    LSM6DSO_ESP_XSENSOR_6 )
#define LSM6DSO_ESP_XSENSOR         1
#else
#define LSM6DSO_ESP_XSENSOR         0
#endif

#if LSM6DSO_ESP_XSENSOR
#include "sns_xsensor.pb.h"
#define XSENSOR_1_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x29, 0x44, 0x1b, 0x3c, 0xbf, 0xe7, 0x4b, 0x2d,  \
        0xb5, 0x66, 0xf2, 0xaa, 0xfc, 0x30, 0x5d, 0x25  \
      }  \
  }

#define XSENSOR_2_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x96, 0x7f, 0xcf, 0x34, 0x75, 0x1c, 0x4f, 0xab,  \
        0x96, 0x9e, 0xc3, 0xec, 0x56, 0x90, 0x4f, 0x4d  \
      }  \
  }

#define XSENSOR_3_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0xad, 0xc5, 0x73, 0x8d, 0xc5, 0x1a, 0x4b, 0x41,  \
        0x9a, 0xa6, 0x60, 0x3f, 0x97, 0x9e, 0x46, 0xdf  \
      }  \
  }

#define XSENSOR_4_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x73, 0xe7, 0x59, 0xed, 0x42, 0x78, 0x48, 0x19,  \
        0xa4, 0x51, 0x38, 0xfd, 0xd4, 0x9a, 0x61, 0x57  \
      }  \
  }

#define XSENSOR_5_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x6c, 0x90, 0x2d, 0x1c, 0x10, 0x0b, 0x45, 0x9c,  \
        0x86, 0x68, 0xdb, 0xe2, 0x85, 0x86, 0x32, 0x4b  \
      }  \
  }

#define XSENSOR_6_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x95, 0x7f, 0x8f, 0x5a, 0xcf, 0x6b, 0x4d, 0x67,  \
        0xa4, 0x0d, 0x6b, 0x9b, 0xce, 0x28, 0x23, 0x37  \
      }  \
  }
#if LSM6DSO_DUAL_SENSOR_ENABLED
#define XSENSOR_1_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x29, 0x44, 0x1b, 0x3c, 0xbf, 0xe7, 0x4b, 0x2d,  \
        0xb5, 0x66, 0xf2, 0xaa, 0xfc, 0x30, 0x5d, 0x26  \
      }  \
  }

#define XSENSOR_2_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x96, 0x7f, 0xcf, 0x34, 0x75, 0x1c, 0x4f, 0xab,  \
        0x96, 0x9e, 0xc3, 0xec, 0x56, 0x90, 0x4f, 0x4e  \
      }  \
  }
#endif
typedef enum
{
#if LSM6DSO_ESP_XSENSOR_1
    XSENSOR_1_IDX,
#endif
#if LSM6DSO_ESP_XSENSOR_2
    XSENSOR_2_IDX,
#endif
#if LSM6DSO_ESP_XSENSOR_3
    XSENSOR_3_IDX,
#endif
#if LSM6DSO_ESP_XSENSOR_4
    XSENSOR_4_IDX,
#endif
#if LSM6DSO_ESP_XSENSOR_5
    XSENSOR_5_IDX,
#endif
#if LSM6DSO_ESP_XSENSOR_6
    XSENSOR_6_IDX,
#endif
    MAX_XSENSORS,
} xsensor_index;

typedef enum
{
  XSENSOR_INT_1,
  XSENSOR_INT_2,
} xsensor_int;

typedef enum
{
  XSENSOR_TYPE_FSM,
  XSENSOR_TYPE_MLC,
} xsensor_type;

typedef struct lsm6dso_xsensor_info
{
  bool                     self_test_is_successful;
  bool                     enable_int;
  bool                     client_present;
  size_t                   encoded_event_len;
} lsm6dso_xsensor_info;

typedef struct lsm6dso_xsensor_group_info {
  sns_sensor_uid          suid[MAX_XSENSORS];
  uint16_t                enabled_sensors;
  uint16_t                desired_sensors;
  uint16_t                is_gyro_req;
  sns_data_stream         *timer_data_stream;
#if LSM6DSO_ESP_XSENSOR
  lsm6dso_xsensor_info    xsensor_info[MAX_XSENSORS];
#endif
#if LSM6DSO_FSM_ENABLED
  uint8_t                 fsm_status[2];
  uint8_t                 fsm_outs[8];
  uint8_t                 fsm_lc[2];
#endif
#if LSM6DSO_MLC_ENABLED
  uint8_t                 mlc_status;
  uint8_t                 mlc_src[8];
#endif
} lsm6dso_xsensor_group_info;

extern lsm6dso_esp_sensors lsm6dso_supported_xsensors[MAX_XSENSORS];

#define LSM6DSO_IS_XSENSOR(s) ((s == LSM6DSO_XSENSOR_1)|| \
                                  (s == LSM6DSO_XSENSOR_2)|| \
                                  (s == LSM6DSO_XSENSOR_3)|| \
                                  (s == LSM6DSO_XSENSOR_4)|| \
                                  (s == LSM6DSO_XSENSOR_5)) ? (true) : (false)

#define LSM6DSO_XSENSOR_ENABLED_MASK ( 0 | \
                                    LSM6DSO_XSENSOR_1 | \
                                    LSM6DSO_XSENSOR_2 | \
                                    LSM6DSO_XSENSOR_3 | \
                                    LSM6DSO_XSENSOR_4 | \
                                    LSM6DSO_XSENSOR_5 | \
                                    0                     )

#define LSM6DSO_IS_XSENSOR_ENABLED(s) (s->xgroup_info.enabled_sensors)
#define LSM6DSO_IS_XSENSOR_DESIRED(s) (s->xgroup_info.desired_sensors)
#define LSM6DSO_IS_XSENSOR_BIT_SET(s) (s & LSM6DSO_XSENSOR_ENABLED_MASK) ? (true) : (false)
#define LSM6DSO_IS_XSENSOR_TIMER_ON(s)(s->xgroup_info.timer_data_stream ? (true) : (false))
sns_sensor_instance* lsm6dso_set_xsensor_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove,
                                                uint8_t idx,
                                                sns_sensor_uid* suid);
void lsm6dso_send_xsensor_event(sns_sensor_instance *const instance,
                                   xsensor_type type,
                                   uint8_t xsensor_idx,
                                   sns_xsensor_event_type event,
                                   sns_time ts,
                                   float *data);

void lsm6dso_handle_xsensor_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src,
                                 uint8_t const *emb_src);
void lsm6dso_init_xsensor_instance(sns_sensor_instance *instance);
sns_rc lsm6dso_xsensor_enable(sns_sensor_instance *instance, xsensor_type type, uint16_t sensor,  uint16_t xsensor_idx, xsensor_int int_line, bool enable);
bool lsm6dso_get_xsensor_interrupt_status(sns_sensor_instance *instance, xsensor_type type, uint16_t sensor);
void lsm6dso_reconfig_xsensor(sns_sensor_instance *const instance);
sns_rc lsm6dso_handle_xsensor_timer_events(sns_sensor_instance *const instance);
float lsm6dso_get_xsensor_rate(sns_sensor_instance *instance);
void lsm6dso_send_xsensor_oem_event(sns_sensor_instance *const instance,
                                   xsensor_type type,
                                   uint8_t xsensor_idx,
                                   sns_xsensor_event_type event,
                                   sns_time ts,
                                   float *data);


#else
void lsm6dso_reconfig_xsensor(sns_sensor_instance *const instance);
sns_rc lsm6dso_handle_xsensor_timer_events(sns_sensor_instance *const instance);
#define LSM6DSO_IS_XSENSOR_ENABLED(s) 0
#define LSM6DSO_IS_XSENSOR_DESIRED(s) 0
#define LSM6DSO_IS_XSENSOR(s) (false)
#define LSM6DSO_IS_XSENSOR_TIMER_ON(s) false
typedef struct lsm6dso_xsensor_group_info {
} lsm6dso_xsensor_group_info;

#endif //LSM6DSO_ESP_XSENSOR
