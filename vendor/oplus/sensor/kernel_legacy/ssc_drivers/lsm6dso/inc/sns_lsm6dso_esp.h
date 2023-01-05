#pragma once
/**
 * @file sns_lsm6dso_esp.h
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
#include "sns_lsm6dso_build_config.h"

/** Enable FSM/MLC based user defined sensors */
#if(LSM6DSO_ESP_ACTIVITY      || \
    LSM6DSO_ESP_FREE_FALL     || \
    LSM6DSO_ESP_HIGH_SHOCK    || \
    LSM6DSO_ESP_STEP_COUNTER  || \
    LSM6DSO_ESP_DOUBLE_TAP    || \
    LSM6DSO_ESP_XSENSOR_1     || \
    LSM6DSO_ESP_XSENSOR_2     || \
    LSM6DSO_ESP_XSENSOR_3     || \
    LSM6DSO_ESP_XSENSOR_4     || \
    LSM6DSO_ESP_XSENSOR_5     )
#define LSM6DSO_ESP_ENABLED   1
#else
#define LSM6DSO_ESP_ENABLED   0
#endif


#define STM_LSM6DSO_REG_FUNC_CFG_EN               (0x80)
#define STM_LSM6DSO_REG_FUNC_CFG_EN_MASK          (0x80)
#define STM_LSM6DSO_REG_INT1_EMB_FUNC             (0x02)
#define STM_LSM6DSO_REG_INT1_EMB_FUNC_MASK        (0x02)
#define STM_LSM6DSO_REG_PEDO_EN                   (0x08)
#define STM_LSM6DSO_REG_PEDO_EN_MASK              (0x08)
#define STM_LSM6DSO_REG_INT1_STEP_DETECTOR        (0x08)
#define STM_LSM6DSO_REG_INT1_STEP_DETECTOR_MASK   (0x08)
#define STM_LSM6DSO_REG_STEP_DETECTED             (0x20)
#define STM_LSM6DSO_REG_STEPCOUNTER_BIT_SET       (0x04)
#define STM_LSM6DSO_REG_STEP_DET_INIT             (0x08)
#define STM_LSM6DSO_REG_STEP_DET_INIT_MASK        (0x08)
#define STM_LSM6DSO_REG_STATUS_MAINPAGE           (0x35)

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
        0xf9, 0xbe, 0xcc, 0xc7, 0x4c, 0x82, 0x4d, 0xd4,  \
        0x96, 0x2b, 0xcb, 0x1a, 0xb8, 0x22, 0xe8, 0x81  \
      }  \
  }

#define DOUBLE_TAP_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0xfe, 0x04, 0xd4, 0x95, 0xcc, 0x97, 0x4d, 0xcb,  \
        0xaa, 0xc9, 0xca, 0x4b, 0x44, 0xe4, 0x03, 0x60  \
      }  \
  }

#if LSM6DSO_DUAL_SENSOR_ENABLED
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
        0xe7, 0xd8, 0x7e, 0xc9, 0xd9, 0x7d, 0x4b, 0x75,  \
        0x8f, 0x2d, 0x6c, 0x84, 0xcb, 0x4f, 0x39, 0xc0  \
      }  \
  }

#define DOUBLE_TAP_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x83, 0x99, 0xbf, 0xf7, 0x43, 0xfd, 0x4e, 0xc6,  \
        0x8e, 0x80, 0xf3, 0x8c, 0x41, 0x01, 0x23, 0xab  \
      }  \
  }

#endif


typedef struct {
  char name[40];
  char proto[40];
  uint16_t sensor;
  uint16_t min_odr;
  uint16_t stream_type;
  void (*register_sensor)(sns_register_cb const *register_api); 
  void (*reconfig)(sns_sensor_instance *const instance, bool enable); 
  void (*handle_intr)(sns_sensor_instance *const instance, 
                                 sns_time ts,
                                 uint8_t reg,
                                 uint8_t data);
  void (*handle_timer_events)(sns_sensor_instance *const instance); 
} lsm6dso_esp_sensors;

#if LSM6DSO_ESP_ENABLED
#define LSM6DSO_GEN_GROUP(x,group) NAME "_"#x group
/**Registry items supported as part of FF
 *  * configuration registry group
 *   */
typedef struct
{
  // threshold in m/s2
  float thresh;
  // window in sec
  float dur;
} lsm6dso_ff_registry_cfg;

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
} lsm6dso_act_registry_cfg;

typedef struct
{
  float thresh_x;
  float thresh_y;
  float thresh_z;
  uint8_t priority;
  uint8_t dur;
  uint8_t quiet;
  uint8_t shock;
  uint8_t dur_sw;
  uint8_t quiet_sw;
} lsm6dso_dtp_registry_cfg;

typedef struct
{
  uint32_t    stepCount_base;
  uint16_t    stepCount_prev;
#if LSM6DSO_OEM_FACTORY_CONFIG
  bool        oem_config_on;
  bool        oem_config_applied;
#endif
} lsm6dso_sc_cfg;

typedef struct lsm6dso_free_fall_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  sns_motion_detect_event cur_ff_state;
  sns_motion_detect_event prev_ff_state;
  lsm6dso_ff_registry_cfg config;
  size_t                  encoded_event_len;
} lsm6dso_free_fall_info;

typedef struct lsm6dso_high_shock_info
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
} lsm6dso_high_shock_info;

typedef struct lsm6dso_activity_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  uint16_t                client_present;
  lsm6dso_act_registry_cfg config;
  sns_data_stream         *timer_inact_data_stream;
  size_t                  encoded_event_len;
} lsm6dso_activity_info;

typedef struct lsm6dso_step_counter_info
{
  bool                    self_test_is_successful;
  bool                    enable_int;
  bool                    client_present;
  lsm6dso_sc_cfg          config;
  size_t                  encoded_event_len;
  size_t                  encoded_raw_log_len;
} lsm6dso_step_counter_info;

typedef struct lsm6dso_double_tap_info
{
  bool                    client_present;
  lsm6dso_dtp_registry_cfg config;
  uint8_t                 who_am_i_val;
#if LSM6DSO_LOG_VERBOSE_EX
  uint16_t                dtp_cnt;
#endif
  size_t                  encoded_event_len;
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  uint32_t                single_tap_peak;
  sns_time                pre_dtp_ts;
  sns_time                dur;
  sns_time                quiet;
#endif
} lsm6dso_double_tap_info;

typedef enum {
#if LSM6DSO_ESP_FREE_FALL
  FF_INDX,
#endif
#if LSM6DSO_ESP_HIGH_SHOCK
  HS_INDX,
#endif
#if LSM6DSO_ESP_ACTIVITY
  ACT_INDX,
  INACT_INDX,
#endif
#if LSM6DSO_ESP_STEP_COUNTER
  SC_INDX,
#endif
#if LSM6DSO_ESP_DOUBLE_TAP
  DTP_INDX,
#endif
  MAX_ESP_SENSORS,
} esp_sensor_index;

typedef struct
{
#if LSM6DSO_ESP_FREE_FALL
  lsm6dso_ff_registry_cfg ff_reg_conf;
#endif
#if LSM6DSO_ESP_HIGH_SHOCK
  sns_registry_md_cfg hs_reg_conf;
#endif
#if LSM6DSO_ESP_ACTIVITY
  lsm6dso_act_registry_cfg act_reg_conf;
#endif
#if LSM6DSO_ESP_STEP_COUNTER
  lsm6dso_sc_cfg sc_conf;
#endif
#if LSM6DSO_ESP_DOUBLE_TAP
  lsm6dso_dtp_registry_cfg dtp_reg_conf;
#endif
} lsm6dso_esp_registry_cfg;

typedef struct lsm6dso_esp_info
{
#if LSM6DSO_ESP_FREE_FALL
  lsm6dso_free_fall_info  ff_info;
#endif
#if LSM6DSO_ESP_HIGH_SHOCK
  lsm6dso_high_shock_info hs_info;
#endif
#if LSM6DSO_ESP_ACTIVITY
  lsm6dso_activity_info   act_info;
#endif
#if LSM6DSO_ESP_STEP_COUNTER
  lsm6dso_step_counter_info sc_info;
#endif
#if LSM6DSO_ESP_DOUBLE_TAP
  lsm6dso_double_tap_info dtp_info;
#endif
  sns_sensor_uid          suid[MAX_ESP_SENSORS];
  uint16_t                desired_sensors;
  uint16_t                enabled_sensors;
  uint16_t                update_int;
} lsm6dso_esp_info;

extern lsm6dso_esp_sensors lsm6dso_supported_esp_sensors[MAX_ESP_SENSORS];


#define LSM6DSO_IS_ESP_ENABLED(s) (s->esp_info.enabled_sensors | LSM6DSO_IS_XSENSOR_ENABLED(s))
#define LSM6DSO_IS_ESP_DESIRED(s) (s->esp_info.desired_sensors | LSM6DSO_IS_XSENSOR_DESIRED(s))

#define LSM6DSO_IS_ESP_SENSOR(s) ((s == LSM6DSO_FREE_FALL) || \
                                  (s == LSM6DSO_HIGH_SHOCK) || \
                                  (s == LSM6DSO_ACTIVITY) || \
                                  (s == LSM6DSO_INACTIVITY) || \
                                  (s == LSM6DSO_STEP_COUNTER) || \
                                  (s == LSM6DSO_DOUBLE_TAP)   || \
                                  LSM6DSO_IS_XSENSOR(s)) ? (true) : (false)

#define LSM6DSO_ESP_ENABLED_MASK ( 0 | \
                                    LSM6DSO_FREE_FALL | \
                                    LSM6DSO_HIGH_SHOCK | \
                                    LSM6DSO_ACTIVITY | \
                                    LSM6DSO_INACTIVITY | \
                                    LSM6DSO_STEP_COUNTER | \
                                    LSM6DSO_DOUBLE_TAP    | \
                                    0                     )

#define LSM6DSO_IS_ESP_BIT_SET(s) (s & LSM6DSO_ESP_ENABLED_MASK) ? (true) : (false)

#define LSM6DSO_IS_ESP_CONF_CHANGED(state) (state->esp_info.desired_sensors ^ state->esp_info.enabled_sensors)
#define LSM6DSO_IS_ESP_PRESENCE_CHANGED(state) ((state->esp_info.desired_sensors & ~state->esp_info.enabled_sensors) || \
                                                (~state->esp_info.desired_sensors & state->esp_info.enabled_sensors))
#define LSM6DSO_UPDATE_ESP_SENSOR_BIT(state, sensor, enable) ((enable) ? (state->esp_info.desired_sensors |= sensor) : \
                                                                      (state->esp_info.desired_sensors &= ~sensor))
#else
#define LSM6DSO_IS_ESP_SENSOR(s) (false)
#define LSM6DSO_IS_ESP_ENABLED(s) (false)
#define LSM6DSO_IS_ESP_DESIRED(s) (false)
#define LSM6DSO_IS_ESP_CONF_CHANGED(state) (false)
#define LSM6DSO_ESP_ENABLED_MASK 0
#endif

void lsm6dso_esp_register(sns_register_cb const *register_api);
void lsm6dso_send_esp_registry_requests(sns_sensor *const this, uint8_t hw_id);
void lsm6dso_process_esp_registry_event(sns_sensor *const this, sns_sensor_event *event);

void lsm6dso_init_esp_instance(sns_sensor_instance *instance, sns_sensor_state const *sstate_ptr);

bool lsm6dso_is_esp_request_present(sns_sensor_instance *instance);
void lsm6dso_recover_esp(sns_sensor_instance *const instance);
void lsm6dso_reconfig_esp(sns_sensor_instance *const instance);
float lsm6dso_get_esp_rate(sns_sensor_instance *instance);
uint16_t lsm6dso_get_esp_rate_idx(sns_sensor_instance *instance);
void lsm6dso_handle_esp_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src,
                                 uint8_t const *emb_src);
sns_rc lsm6dso_handle_esp_timer_events(sns_sensor_instance *const instance);

sns_rc
lsm6dso_publish_esp_attributes(sns_sensor *const this, lsm6dso_esp_sensors* esp_sensor_ptr);
sns_rc lsm6dso_esp_init(sns_sensor *const this, sns_sensor_uid* suid, lsm6dso_esp_sensors* esp_sensor_ptr, uint16_t sensor);
void lsm6dso_esp_deinit(sns_sensor_instance *instance);
void lsm6dso_clear_request_q(sns_sensor *const this, sns_sensor_instance *instance, struct sns_request const *exist, sns_sensor_uid* suid, bool remove);

sns_rc lsm6dso_emb_cfg_access(sns_sensor_instance *instance, bool enable);

