#pragma once
/**
 * @file sns_lsm6dsm_esp.h
 *
 * Copyright (c) 2018, STMicroelectronics.
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
#include "sns_motion_detect.pb.h"
#if LSM6DSM_ESP_ENABLED
#include "sns_free_fall.pb.h"
#include "sns_high_shock.pb.h"
#include "sns_activity.pb.h"
#endif
#define MAX_SUPPORTED_ESP_SENSORS 5

#define STM_LSM6DSM_STEP_COUNTER_EN_MASK    (0x14)
#define STM_LSM6DSM_STEP_DETCTOR_INT_MASK   (0x80)

#define FREE_FALL_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x1d, 0x2f, 0x96, 0x8f, 0xbc, 0xf8, 0x4f, 0x89,  \
        0xba, 0x32, 0xde, 0x34, 0x99, 0xea, 0x22, 0x5a  \
      }  \
  }

#define HIGH_SHOCK_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x5c, 0x79, 0x91, 0x5c, 0x0a, 0x1f, 0x4f, 0x08,  \
        0xb5, 0x94, 0x1a, 0x34, 0x20, 0x80, 0xfc, 0xcf  \
      }  \
  }

#define ACTIVITY_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0xb7, 0x3c, 0x3d, 0xb2, 0xf0, 0xc4, 0x48, 0xd2,  \
        0x90, 0xe2, 0x78, 0xd8, 0xd2, 0x49, 0xd9, 0xc1  \
      }  \
  }

#define INACTIVITY_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x78, 0xce, 0xd6, 0xc6, 0x04, 0x74, 0x4a, 0xcc,  \
        0xbf, 0xa9, 0xb9, 0x16, 0x48, 0x15, 0xcd, 0x43  \
      }  \
  }

#define STEP_COUNTER_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0xf7, 0x4c, 0x99, 0xa1, 0xdd, 0xc3, 0x46, 0x23, \
        0xb5, 0x82, 0x15, 0x51, 0x76, 0x43, 0xc4, 0x19  \
      }  \
  }

#if LSM6DSM_DUAL_SENSOR_ENABLED
#define FREE_FALL_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x03, 0x19, 0x66, 0x88, 0xe7, 0x04, 0x4e, 0x60, \
        0x83, 0x3e, 0xbd, 0xe7, 0xc6, 0x94, 0x0d, 0x41  \
      }  \
  }

#define HIGH_SHOCK_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x03, 0x19, 0x66, 0x88, 0xe7, 0x04, 0x4e, 0x60, \
        0x83, 0x3e, 0xbd, 0xe7, 0xc6, 0x94, 0x0d, 0x51  \
      }  \
  }
#define ACTIVITY_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0xf9, 0x5b, 0x2c, 0xf4, 0x8a, 0x7c, 0x4a, 0x43,  \
        0xbc, 0xb0, 0x1f, 0xbe, 0x1b, 0xda, 0xad, 0x05  \
      }  \
  }

#define INACTIVITY_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x54, 0x1e, 0x25, 0x26, 0x76, 0x80, 0x46, 0x72,  \
        0xbf, 0x14, 0x18, 0x27, 0x19, 0x0f, 0x3c, 0x87  \
      }  \
  }

#define STEP_COUNTER_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x16, 0xb1, 0xf0, 0xf4, 0xd4, 0x3d, 0x4d, 0xd7, \
        0xac, 0x05, 0x2d, 0xee, 0xac, 0x90, 0x1e, 0xf5  \
      }  \
  }

#endif
typedef struct {
  char name[40];
  char proto[40];
  uint16_t sensor;
  uint16_t min_odr;
  uint16_t stream_type;
  sns_sensor_api* sensor_api;
} lsm6dsm_esp_sensors;

/**Registry items supported as part of FF
 *  * configuration registry group
 *   */
typedef struct
{
  // threshold in m/s2
  float thresh;
  // window in sec
  float dur;
} lsm6dsm_ff_registry_cfg;

/**Registry items supported as part of act and inact
 *  * configuration registry group
 *   */
typedef struct
{
  // threshold in m/s2
  float thresh;
  // window in sec
  float dur;
  // timer in sec
  float inact_timeout;
} lsm6dsm_act_registry_cfg;

typedef struct
{
  lsm6dsm_ff_registry_cfg ff_reg_conf;
  sns_registry_md_cfg hs_reg_conf;
  lsm6dsm_act_registry_cfg act_reg_conf;
} lsm6dsm_esp_registry_cfg;

typedef struct lsm6dsm_free_fall_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  sns_motion_detect_event cur_ff_state;
  sns_motion_detect_event prev_ff_state;
  lsm6dsm_ff_registry_cfg config;
  size_t                  encoded_event_len;
} lsm6dsm_free_fall_info;

typedef struct lsm6dsm_high_shock_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  sns_motion_detect_event cur_hs_state;
  sns_motion_detect_event prev_hs_state;
  sns_registry_md_cfg     config;
  sns_registry_md_cfg     config_md;
  sns_data_stream         *timer_hs_data_stream;
  size_t                  encoded_event_len;
} lsm6dsm_high_shock_info;

typedef struct lsm6dsm_activity_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  lsm6dsm_act_registry_cfg config;
  sns_data_stream         *timer_inact_data_stream;
  size_t                  encoded_event_len;
} lsm6dsm_activity_info;

typedef struct lsm6dsm_step_counter_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  uint32_t                stepCount_base;
  uint16_t                stepCount_prev;
  size_t                  encoded_event_len;
  size_t                  encoded_raw_log_len;
} lsm6dsm_step_counter_info;

typedef struct lsm6dsm_esp_info
{
  lsm6dsm_free_fall_info    ff_info;
  lsm6dsm_high_shock_info   hs_info;
  lsm6dsm_activity_info     act_info;
  lsm6dsm_step_counter_info sc_info;
  sns_sensor_uid            suid[MAX_SUPPORTED_ESP_SENSORS];
  uint16_t                  desired_sensors;
  uint16_t                  enabled_sensors;
  uint16_t                  update_int;
} lsm6dsm_esp_info;


extern const lsm6dsm_esp_sensors lsm6dsm_supported_esp_sensors[MAX_SUPPORTED_ESP_SENSORS];
typedef enum {
  FF_INDX,
  HS_INDX,
  ACT_INDX,
  INACT_INDX,
  SC_INDX,
} esp_sensor_index;

#if LSM6DSM_ESP_ENABLED

#define LSM6DSM_IS_ESP_ENABLED(s) (s->esp_info.enabled_sensors)
#define LSM6DSM_IS_ESP_DESIRED(s) (s->esp_info.desired_sensors)
#define LSM6DSM_IS_ESP_SENSOR(s) ((s == LSM6DSM_FREE_FALL) || \
                                  (s == LSM6DSM_HIGH_SHOCK) || \
                                  (s == LSM6DSM_STEP_COUNTER) || \
                                  (s == LSM6DSM_ACTIVITY) || \
                                  (s == LSM6DSM_INACTIVITY)) ? (true) : (false)
#define LSM6DSM_IS_ESP_BIT_SET(s) (s & (LSM6DSM_FREE_FALL | \
                                  LSM6DSM_HIGH_SHOCK | \
                                  LSM6DSM_STEP_COUNTER | \
                                  LSM6DSM_ACTIVITY | \
                                  LSM6DSM_INACTIVITY)) ? (true) : (false)
#define LSM6DSM_ESP_SENSORS_MASK  (LSM6DSM_FREE_FALL | \
                                   LSM6DSM_HIGH_SHOCK | \
                                   LSM6DSM_ACTIVITY | \
                                   LSM6DSM_INACTIVITY | \
                                   LSM6DSM_STEP_COUNTER)

#define LSM6DSM_IS_ESP_CONF_CHANGED(state) (state->esp_info.desired_sensors ^ state->esp_info.enabled_sensors)
#define LSM6DSM_IS_ESP_PRESENCE_CHANGED(state) ((state->esp_info.desired_sensors & ~state->esp_info.enabled_sensors) || \
                                                (~state->esp_info.desired_sensors & state->esp_info.enabled_sensors))
#define LSM6DSM_UPDATE_ESP_SENSOR_BIT(state, sensor, enable) ((enable) ? (state->esp_info.desired_sensors |= sensor) : \
                                                                      (state->esp_info.desired_sensors &= ~sensor))
#else
#define LSM6DSM_IS_ESP_SENSOR(s) (false)
#define LSM6DSM_IS_ESP_ENABLED(s) (false)
#define LSM6DSM_IS_ESP_DESIRED(s) (false)
#define LSM6DSM_IS_ESP_CONF_CHANGED(state) (false)
#endif


#define LSM6DSM_FF_THRESHOLD             (8)             //free fall threshold
#define LSM6DSM_FF_PERIOD                (8)             //free fall duration
#define LSM6DSM_FF_ODR_MIN               (832)           //Default sample rate for DBT
#define LSM6DSM_FF_REPORT_PERIOD         (1000 *1000)    //Default report  period for DBT in secs conv to msec
/**
 * Handles DBT interrupt:
 *   Sends DBT fired event.
 *
 * @param[i] instance        Instance reference
 * @param[i] irq_timestamp   interrupt timestamp
 * @param[i] wake_src        TAP_SRC register contents
 *
 * @return none
 */
  void lsm6dsm_handle_esp_interrupt(sns_sensor_instance *const instance,
                                   sns_time irq_timestamp,
                                   uint8_t const *src_regs);
sns_rc lsm6dsm_handle_esp_timer_events(sns_sensor_instance *const instance);

void lsm6dsm_esp_register(sns_register_cb const *register_api);
void lsm6dsm_send_esp_registry_requests(sns_sensor *const this, uint8_t hw_id);
void lsm6dsm_process_esp_registry_event(sns_sensor *const this, sns_sensor_event *event);
sns_rc lsm6dsm_free_fall_init(sns_sensor *const this);
sns_rc lsm6dsm_high_shock_init(sns_sensor *const this);
sns_rc lsm6dsm_activity_init(sns_sensor *const this);
sns_rc lsm6dsm_inactivity_init(sns_sensor *const this);
sns_rc lsm6dsm_free_fall_deinit(sns_sensor *const this);
sns_rc lsm6dsm_high_shock_deinit(sns_sensor *const this);
sns_rc lsm6dsm_activity_deinit(sns_sensor *const this);
sns_rc lsm6dsm_inactivity_deinit(sns_sensor *const this);
sns_rc lsm6dsm_step_counter_init(sns_sensor *const this);
sns_rc lsm6dsm_step_counter_deinit(sns_sensor *const this);

void lsm6dsm_init_esp_instance(sns_sensor_instance *instance, sns_sensor_state const *this);

void lsm6dsm_store_esp_registry_data(sns_sensor_instance *instance, sns_sensor_state const *this);
bool lsm6dsm_is_esp_request_present(sns_sensor_instance *instance);
void lsm6dsm_update_esp_sensor_config(sns_sensor *this,
                              sns_sensor_instance *instance,
                              struct lsm6dsm_instance_config *inst_cfg);
void lsm6dsm_poweron_esp(sns_sensor_instance *const instance);
void lsm6dsm_powerdown_esp(sns_sensor_instance *const instance);
void lsm6dsm_update_pending_esp_intr(sns_sensor_instance *const instance);
float lsm6dsm_get_esp_rate(sns_sensor_instance *instance);
uint16_t lsm6dsm_get_esp_rate_idx(sns_sensor_instance *instance);
void lsm6dsm_update_esp_request_q(
    sns_sensor               *const this,
    sns_sensor_instance      *instance,
    struct sns_request const *exist,
    bool remove);
void lsm6dsm_sc_report_steps(sns_sensor_instance *const instance);
uint16_t read_step_count(sns_sensor_instance *const this);
void lsm6dsm_esp_handle_reset_device(sns_sensor_instance *const instance, sns_sensor_state const *sstate);
void init_esp_shared_state(sns_sensor *const this);

