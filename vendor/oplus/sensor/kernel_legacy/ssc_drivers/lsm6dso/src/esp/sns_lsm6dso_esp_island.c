/**
 * @file sns_lsm6dso_esp_island.c
 *
 * Common implementation for LSM6DSO esp Sensors.
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
#include <math.h>
#include <string.h>

#include "sns_lsm6dso_sensor.h"
#include "sns_types.h"

#if LSM6DSO_ESP_ENABLED
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_printf.h"
#include "pb_encode.h"

#include "sns_diag.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"

extern void lsm6dso_register_inactivity(sns_register_cb const *register_api);
extern void lsm6dso_register_activity(sns_register_cb const *register_api);
extern void lsm6dso_register_free_fall(sns_register_cb const *register_api);
extern void lsm6dso_register_high_shock(sns_register_cb const *register_api);
extern void lsm6dso_register_step_counter(sns_register_cb const *register_api);
extern void lsm6dso_register_double_tap(sns_register_cb const *register_api);
extern sns_rc lsm6dso_sensor_notify_event(sns_sensor *const this);
extern const odr_reg_map lsm6dso_odr_map[];
extern const uint32_t lsm6dso_odr_map_len;


extern void lsm6dso_handle_xsensor_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src,
                                 uint8_t const *emb_src);
extern void lsm6dso_init_xsensor_instance(sns_sensor_instance *instance);

//const lsm6dso_esp_sensors lsm6dso_supported_esp_sensors [ MAX_ESP_SENSORS ];
lsm6dso_esp_sensors lsm6dso_supported_esp_sensors [MAX_ESP_SENSORS] = {
#if LSM6DSO_ESP_FREE_FALL
  { .name = "free_fall",  .register_sensor = &lsm6dso_register_free_fall},
#endif
#if LSM6DSO_ESP_HIGH_SHOCK
  { .name = "high_shock",  .register_sensor = &lsm6dso_register_high_shock},
#endif
#if LSM6DSO_ESP_ACTIVITY
  { .name = "st_activity",  .register_sensor = &lsm6dso_register_activity},
  { .name = "st_inactivity",  .register_sensor = &lsm6dso_register_inactivity},
#endif
#if LSM6DSO_ESP_STEP_COUNTER
  { .name = "step_counter",  .register_sensor = &lsm6dso_register_step_counter},
#endif
#if LSM6DSO_ESP_DOUBLE_TAP
  { .name = "st_double_tap",  .register_sensor = &lsm6dso_register_double_tap},
#endif
};

float lsm6dso_get_esp_rate(sns_sensor_instance *instance)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_esp_sensors);
  float max_odr = 0.0f;
  for(int i=0; i< sensor_count ; i++) {
    if(inst_state->esp_info.desired_sensors & lsm6dso_supported_esp_sensors[i].sensor)
      max_odr = SNS_MAX(max_odr, lsm6dso_supported_esp_sensors[i].min_odr);
  }
#if LSM6DSO_ESP_XSENSOR
  max_odr = SNS_MAX(max_odr, lsm6dso_get_xsensor_rate(instance));
#endif
  return max_odr;
}

uint16_t lsm6dso_get_esp_rate_idx(sns_sensor_instance *instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  float esp_sample_rate = lsm6dso_get_esp_rate(instance);
  uint8_t idx = lsm6dso_get_odr_rate_idx(esp_sample_rate);
  idx = SNS_MAX(idx, state->min_odr_idx);
  return lsm6dso_odr_map[idx].accel_odr_reg_value;
}

void lsm6dso_recover_esp(sns_sensor_instance *const instance)
{
#if LSM6DSO_ESP_XSENSOR
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint16_t desired_xsesnor = state->xgroup_info.desired_sensors;
  lsm6dso_init_xsensor_instance(instance);
  state->xgroup_info.desired_sensors = desired_xsesnor;
  state->xgroup_info.enabled_sensors = 0;
#endif
  lsm6dso_reconfig_esp(instance);
}
void lsm6dso_reconfig_esp(sns_sensor_instance *const instance)
{
  uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_esp_sensors);
  uint8_t i = 0;
  for(; i < sensor_count; i++) {
    if(lsm6dso_supported_esp_sensors[i].reconfig)
      lsm6dso_supported_esp_sensors[i].reconfig(instance, false);
  }
  DBG_INST_PRINTF_EX(LOW, instance, "reconfig esp  %d",sensor_count, i);
  lsm6dso_reconfig_xsensor(instance);
}
//remove all the requests from queue, except the current request
void lsm6dso_clear_request_q(
    sns_sensor               *const this,
    sns_sensor_instance      *instance,
    struct sns_request const *exist,
    sns_sensor_uid* suid,
    bool remove)
{
  UNUSED_VAR(remove);
  sns_request const *request;
  lsm6dso_state *sstate = (lsm6dso_state*)this->state;
  UNUSED_VAR(sstate);
  bool reset = false;
  //clear the queue, if not present in state
  request = instance->cb->get_client_request(instance, suid, true);
  while(NULL != request) {
    if(request != exist) {
      DBG_PRINTF(ERROR, this, "removing req from Q sensor=%d req=%p msg=%d", sstate->sensor, request, request->message_id);
      instance->cb->remove_client_request(instance, request);
      reset = true;
    } else {
      DBG_PRINTF(ERROR, this, "skip removing req from Q sensor=%d req=%p msg=%d", sstate->sensor, request, request->message_id);
    }
    request = instance->cb->get_client_request(instance, suid, reset);
    reset = false;
  }
}

sns_rc lsm6dso_handle_esp_timer_events(sns_sensor_instance *const instance)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_esp_sensors);
  uint8_t i = 0;
  for(; i < sensor_count; i++) {
    if(lsm6dso_supported_esp_sensors[i].handle_timer_events) {
      lsm6dso_supported_esp_sensors[i].handle_timer_events(instance);
    }
  }
  lsm6dso_handle_xsensor_timer_events(instance);
  return rv;
}

void lsm6dso_handle_esp_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src,
                                 uint8_t const *emb_src)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  uint8_t src_reg = STM_LSM6DSO_REG_WAKE_SRC;
  uint8_t wake_data = 0;
  UNUSED_VAR(emb_src);
  if(state->esp_info.enabled_sensors &
      LSM6DSO_ESP_ENABLED_MASK) {
    DBG_INST_PRINTF_EX(HIGH, instance, "ESP HW INT:sensors_e=0x%x ", state->esp_info.enabled_sensors);
    if(wake_src)
    {
      DBG_INST_PRINTF_EX(HIGH, instance, "ESP HW INT:src/data=0x%x/0x%x", *wake_src, *emb_src);
      if(*wake_src == STM_LSM6DSO_REG_WAKE_SRC ||
         *wake_src == STM_LSM6DSO_REG_TAP_SRC)
      {
        src_reg = *wake_src;
        wake_data = *emb_src;
      }
      else
      {
        wake_data = 0x00;
      }
    }
    else
    {
      uint8_t rw_buffer[2];
      lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_WAKE_SRC, 2, rw_buffer); 
    }
    uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_esp_sensors);
    uint8_t i = 0;
    for(; i < sensor_count; i++) {
      if(lsm6dso_supported_esp_sensors[i].handle_intr) {
        lsm6dso_supported_esp_sensors[i].handle_intr(instance, irq_timestamp, src_reg, wake_data);
      }
    }
  }
  lsm6dso_handle_xsensor_interrupt(instance, irq_timestamp, wake_src, emb_src);
}

bool lsm6dso_is_esp_request_present(sns_sensor_instance *instance)
{
  bool ret = false;
  uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_esp_sensors);
  if(instance != NULL) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    int i = 0;
    for(i=0; i< sensor_count ; i++) {
      if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[i], true)) {
        ret = true;
        break;
      }
    }
#if LSM6DSO_ESP_XSENSOR
    sensor_count = ARR_SIZE(lsm6dso_supported_xsensors);
    for(i=0; i< sensor_count ; i++) {
      if(NULL != instance->cb->get_client_request(instance, &inst_state->xgroup_info.suid[i], true)) {
        ret = true;
        break;
      }
    }
#endif
  }
  return ret;
}

void lsm6dso_esp_deinit(sns_sensor_instance *instance)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_memset(&inst_state->esp_info, 0, sizeof(inst_state->esp_info));
}

void lsm6dso_init_esp_instance(sns_sensor_instance *instance, sns_sensor_state const *sstate_ptr)
{
  lsm6dso_state *sstate = (lsm6dso_state*)sstate_ptr->state;
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

#if LSM6DSO_ESP_DOUBLE_TAP
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state_from_state(sstate_ptr);
  inst_state->esp_info.dtp_info.who_am_i_val = shared_state->who_am_i;
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  inst_state->esp_info.dtp_info.single_tap_peak = 0;
#endif //LSM6DSO_EX_TAP_TUNING_ENABLED
#endif //LSM6DSO_ESP_DOUBLE_TAP

  inst_state->esp_info.desired_sensors = 0;
  inst_state->esp_info.update_int = 0;
  SNS_INST_PRINTF(HIGH, instance, "esp_inst_init:: sensor:%d",sstate->sensor);

#if LSM6DSO_SUB_XSENSOR_DISABLED
  if(sstate->hardware_id != 1)
#endif
    lsm6dso_init_xsensor_instance(instance);
}

#endif

