#pragma once
/**
 * @file sns_lsm6dso_ois.h
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
#include "sns_register.h"
#include "sns_lsm6dso_sensor_instance.h"

#define LSM6DSO_IS_OIS_SENSOR(s) (s == LSM6DSO_OIS) ? (true) : (false)
#define OIS_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x97, 0x19, 0x9a, 0x0e, 0x77, 0x14, 0x40, 0x30,  \
        0x9a, 0x4b, 0x76, 0x94, 0xa7, 0x92, 0x9b, 0xcb  \
      }  \
  }
#if LSM6DSO_DUAL_SENSOR_ENABLED
#define OIS_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x8c, 0x45, 0x07, 0x60, 0x42, 0x2e, 0x4e, 0xb1,  \
        0x91, 0x18, 0xd8, 0x35, 0xa9, 0x13, 0x42, 0x5d  \
      }  \
  }
#endif

#define STM_LSM6DSO_REG_UI_STATUS_OIS           (0x49)
#define STM_LSM6DSO_REG_UI_OUTX_L_G_OIS         (0x4A)
#define STM_LSM6DSO_REG_UI_INT_OIS              (0x6F)
#define STM_LSM6DSO_REG_UI_CTRL1_OIS            (0x70)

#if LSM6DSO_OIS_POLLING
#define LSM6DSO_OIS_RATE (LSM6DSO_ODR_416) //default polling rate Hz. Over ride from registry
#else
#define LSM6DSO_OIS_RATE (LSM6DSO_ODR_6656) //6656 hz
#endif //#if LSM6DSO_OIS_POLLING

#define LSM6DSO_OIS_DATA_SIZE (6)

extern const float lsm6dso_ois_resolutions[];
typedef struct
{
  uint8_t                       resolution_idx;
  //sns_lsm6dso_registry_cfg      cal;
} lsm6dso_ois_registry_cfg;

typedef struct lsm6dso_ois_info
{
  bool                       desired;
  bool                       enabled;
  bool                       mode_polling;
  sns_sensor_uid             suid;
  uint8_t                    resolution_idx;
  sns_time                   sampling_intvl_ticks;
  float                      sampling_rate_hz;
  //sns_lsm6dso_registry_cfg   registry_cfg;
} lsm6dso_ois_info;


#define LSM6DSO_IS_OIS_ENABLED(s) (s->ois_info.enabled)
#define LSM6DSO_IS_OIS_DESIRED(s) (s->ois_info.desired)


void lsm6dso_handle_ois_interrupt(sns_sensor_instance *const instance,
    sns_time irq_timestamp);

void lsm6dso_ois_register(sns_register_cb const *register_api);
sns_rc lsm6dso_ois_init(sns_sensor *const this);
sns_rc lsm6dso_ois_deinit(sns_sensor *const this);

void lsm6dso_init_ois_instance(sns_sensor_instance *instance,
    struct lsm6dso_instance_config const *inst_cfg);

void lsm6dso_store_ois_registry_data(sns_sensor_instance *instance, sns_sensor_state const *this);
bool lsm6dso_is_ois_request_present(sns_sensor_instance *instance);
void lsm6dso_update_ois_sensor_config(sns_sensor *this,
                              sns_sensor_instance *instance);
void lsm6dso_config_ois(sns_sensor_instance *const instance);
void lsm6dso_send_ois_registry_requests(sns_sensor *const this, uint8_t hw_id);
void lsm6dso_process_ois_registry_event(sns_sensor *const this, sns_sensor_event *event);
void lsm6dso_update_ois_resolution_idx(sns_sensor *const this, uint8_t res_idx);
sns_rc lsm6dso_handle_ois_polling_timer_events(sns_sensor_instance *const this,
             sns_time timestamp, sns_timer_sensor_event* latest_timer_event);
