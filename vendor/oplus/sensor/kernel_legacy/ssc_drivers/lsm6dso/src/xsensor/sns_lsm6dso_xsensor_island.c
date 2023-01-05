/**
 * @file sns_lsm6dso_xsensor_island.c
 *
 * Common implementation for LSM6DSO XSensor
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
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#if LSM6DSO_ESP_XSENSOR
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_printf.h"

#include "sns_diag.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_xsensor.pb.h"
#include "sns_lsm6dso_fsm.h"
#include "sns_lsm6dso_mlc.h"
#include "sns_lsm6dso_xsensor_config.h"

extern const odr_reg_map lsm6dso_odr_map[];
extern void lsm6dso_register_xsensor_1(sns_register_cb const * register_api);
extern void lsm6dso_register_xsensor_2(sns_register_cb const * register_api);
extern void lsm6dso_register_xsensor_3(sns_register_cb const * register_api);
extern void lsm6dso_register_xsensor_4(sns_register_cb const * register_api);
extern void lsm6dso_register_xsensor_5(sns_register_cb const * register_api);
extern void lsm6dso_register_xsensor_6(sns_register_cb const * register_api);

extern void lsm6dso_process_xsensor_registry_event(sns_sensor *const this, sns_sensor_event *event);

lsm6dso_esp_sensors lsm6dso_supported_xsensors [MAX_XSENSORS] = {
#if LSM6DSO_ESP_XSENSOR_1
  { .name = LSM6DSO_XSENSOR_1_NAME,  .register_sensor = &lsm6dso_register_xsensor_1},
#endif
#if LSM6DSO_ESP_XSENSOR_2
  { .name = LSM6DSO_XSENSOR_2_NAME,  .register_sensor = &lsm6dso_register_xsensor_2},
#endif
#if LSM6DSO_ESP_XSENSOR_3
  { .name = LSM6DSO_XSENSOR_3_NAME,  .register_sensor = &lsm6dso_register_xsensor_3},
#endif
#if LSM6DSO_ESP_XSENSOR_4
  { .name = LSM6DSO_XSENSOR_4_NAME,  .register_sensor = &lsm6dso_register_xsensor_4},
#endif
#if LSM6DSO_ESP_XSENSOR_5
  { .name = LSM6DSO_XSENSOR_5_NAME,  .register_sensor = &lsm6dso_register_xsensor_5},
#endif
#if LSM6DSO_ESP_XSENSOR_6
  { .name = LSM6DSO_XSENSOR_6_NAME,  .register_sensor = &lsm6dso_register_xsensor_6},
#endif
};

void lsm6dso_send_xsensor_event(sns_sensor_instance *const instance,
                                   xsensor_type type,
                                   uint8_t xsensor_idx,
                                   sns_xsensor_event_type event,
                                   sns_time ts,
                                   float *data)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  sns_xsensor_event xsensor_state;
  xsensor_state.xsensor_event_type = event;
  float evt_data = (data)?*data:0;
#if LSM6DSO_MLC_ENABLED
  //If no data is specified, default send mlc_src(classifier) value
  if((NULL == data) && (type == XSENSOR_TYPE_MLC))
  {
    int i=0;
    for (i = 0; i < MLC_SENSOR_CNT; i++)
    {
      if (lsm6dso_mlc_sensor_list[i].id == lsm6dso_supported_xsensors[xsensor_idx].sensor)
        break;
    }
    uint8_t status_mask = (1 << i);
    evt_data = (float)state->xgroup_info.mlc_src[status_mask];
  }
#else
  UNUSED_VAR(type);
#endif

  if(evt_data == 0) {
    DBG_INST_PRINTF_EX(LOW, instance, "For XSENSOR(%0x), sending event: %d",xsensor_idx,event);
    pb_send_event(instance,
        sns_xsensor_event_fields,
        &xsensor_state,
        ts,
        SNS_XSENSOR_MSGID_SNS_XSENSOR_EVENT,
        &state->xgroup_info.suid[xsensor_idx]);
  } else {
    DBG_INST_PRINTF_EX(LOW, instance, "For XSENSOR(%0x), sending data: %d",xsensor_idx,(int32_t)evt_data);
    pb_send_sensor_stream_event(
        instance,
        &state->xgroup_info.suid[xsensor_idx],
        ts,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
        &evt_data,
        1,
        state->xgroup_info.xsensor_info[xsensor_idx].encoded_event_len);
  }

}
bool lsm6dso_is_acc_settled(sns_sensor_instance *instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_time cur_time, odr_time_elapsed, settling_time_ticks;
  uint32_t settling_time_ms = 0;
  int64_t req_timeout = 0;
  uint8_t odr_idx = (state->desired_conf.odr_idx) ? state->desired_conf.odr_idx : state->min_odr_idx;
  if(state->common_info.accel_curr_odr == LSM6DSO_ACCEL_ODR_OFF) {
    DBG_INST_PRINTF_EX(LOW, instance, "acc off");
    return false;
  }

  settling_time_ms = (uint32_t)((lsm6dso_odr_map[odr_idx].accel_discard_samples * 1000.0f) / lsm6dso_odr_map[odr_idx].odr);

  settling_time_ticks = sns_convert_ns_to_ticks(settling_time_ms * 1000000); // in ticks
  cur_time = sns_get_system_time();
  odr_time_elapsed = cur_time - state->cur_odr_change_info.accel_odr_settime;
  req_timeout = (int64_t) (settling_time_ticks - odr_time_elapsed);

  DBG_INST_PRINTF(HIGH, instance,
      "odr_idx %d cur_t %u set_t %u/%u (ns/ticks)",
      odr_idx, (uint32_t)cur_time, settling_time_ms, (uint32_t)settling_time_ticks);
  DBG_INST_PRINTF(HIGH, instance,
      "a_odr_set_t %u elapsed %u req_t/o %u",
      (uint32_t)state->cur_odr_change_info.accel_odr_settime, (uint32_t)odr_time_elapsed,
      (uint32_t)req_timeout);

  if(req_timeout > 0) {
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = settling_time_ticks;
    lsm6dso_inst_create_timer(instance, &state->xgroup_info.timer_data_stream, &req_payload);
    return false;
  }
  return true;
}

void lsm6dso_reconfig_xsensor(sns_sensor_instance *const instance)
{
   uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_xsensors);
    for(int i = 0; i < sensor_count; i++) {
      DBG_INST_PRINTF_EX(LOW, instance, "reconfig count = %d cur_idx =  %d",sensor_count, i);
      if(lsm6dso_supported_xsensors[i].reconfig)
        lsm6dso_supported_xsensors[i].reconfig(instance, false);
    }
}

static sns_rc lsm6dso_xsensor_timer_cb(sns_sensor_instance *const instance,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(latest_timer_event);
  UNUSED_VAR(timestamp);
  sns_rc rv = SNS_RC_SUCCESS;
  DBG_INST_PRINTF_EX(LOW, instance, "handle xsensor timer events ts %u", (uint32_t)sns_get_system_time());
  sns_sensor_util_remove_sensor_instance_stream(instance, &state->xgroup_info.timer_data_stream);
  lsm6dso_reconfig_xsensor(instance);
  return rv;
}

sns_rc lsm6dso_handle_xsensor_timer_events(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  lsm6dso_handle_timer(instance, state->xgroup_info.timer_data_stream,
      &lsm6dso_xsensor_timer_cb);
  return SNS_RC_SUCCESS;
}

void lsm6dso_handle_xsensor_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src,
                                 uint8_t const *emb_src)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  if(state->xgroup_info.enabled_sensors &
      LSM6DSO_XSENSOR_ENABLED_MASK) {
#if LSM6DSO_FSM_ENABLED
    lsm6dso_fsm_get_interrupt_status(instance, wake_src, emb_src);
#endif
#if LSM6DSO_MLC_ENABLED
    lsm6dso_mlc_get_interrupt_status(instance, wake_src, emb_src);
#endif
    uint8_t sensor_count = ARR_SIZE(lsm6dso_supported_xsensors);
    uint8_t i = 0;
    for(; i < sensor_count; i++) {
      if(lsm6dso_supported_xsensors[i].handle_intr) {
        lsm6dso_supported_xsensors[i].handle_intr(instance, irq_timestamp, *wake_src, *emb_src);
      }
    }
  }
}
sns_rc lsm6dso_xsensor_enable(sns_sensor_instance *instance, xsensor_type type, uint16_t sensor,  uint16_t xsensor_idx, xsensor_int int_line, bool enable)
{
  sns_rc rc = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  DBG_INST_PRINTF_EX(LOW, instance, "type %d int %d enable %d",type,int_line, enable);
  //do not enable if acc settling timer is still on
  if(enable) {
    //timer is running, enable when timer fires
    if(state->xgroup_info.timer_data_stream) {
      DBG_INST_PRINTF_EX(LOW, instance, "timer is running");
      return SNS_RC_FAILED;
    } else if((!state->irq_ready  && !lsm6dso_dae_if_available(instance)) || !lsm6dso_is_acc_settled(instance)) {
      DBG_INST_PRINTF_EX(LOW, instance, "timer is started or irq not ready: %d", state->irq_ready);
      return SNS_RC_FAILED;
    }
  } else if(state->xgroup_info.timer_data_stream)  {
    DBG_INST_PRINTF_EX(LOW, instance, "deleting timer data stream");
    sns_sensor_util_remove_sensor_instance_stream(instance, &state->xgroup_info.timer_data_stream);
  }

  if(type == XSENSOR_TYPE_FSM) {
#if LSM6DSO_FSM_ENABLED
    rc = lsm6dso_fsm_set_enable(instance, sensor, int_line, enable);
#endif
  } else {
#if LSM6DSO_MLC_ENABLED
    rc = lsm6dso_mlc_set_enable(instance, sensor, int_line, enable);
#endif
  }
  if(enable && (rc == SNS_RC_SUCCESS))
    lsm6dso_send_xsensor_event(instance, type, xsensor_idx, SNS_XSENSOR_EVENT_TYPE_ENABLED, sns_get_system_time(), NULL);
  else if(!enable && (rc == SNS_RC_SUCCESS))
    lsm6dso_send_xsensor_event(instance, type, xsensor_idx, SNS_XSENSOR_EVENT_TYPE_DISABLED, sns_get_system_time(), NULL);

  return rc;
}
bool lsm6dso_get_xsensor_interrupt_status(sns_sensor_instance *instance, xsensor_type type, uint16_t sensor)
{
  bool fired = false;
  if(type == XSENSOR_TYPE_FSM) {
#if LSM6DSO_FSM_ENABLED
    fired = lsm6dso_check_fsm_sensor_interrupt(instance, sensor);
#endif
  } else {
#if LSM6DSO_MLC_ENABLED
    fired = lsm6dso_mlc_check_sensor_interrupt(instance, sensor);
#endif
  }
  return fired;

}
void lsm6dso_init_xsensor_instance(sns_sensor_instance *instance)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

  inst_state->xgroup_info.desired_sensors = 0;
  SNS_INST_PRINTF(HIGH, instance, "xsensor_inst_init");

#if LSM6DSO_FSM_ENABLED
  //Load FSM
  lsm6dso_init_fsm_instance(instance);
#endif
#if LSM6DSO_MLC_ENABLED
  //Load MLC
  lsm6dso_init_mlc_instance(instance);
#endif
}

static void lsm6dso_fill_xsensor_inst_info(sns_sensor *const this, sns_sensor_instance *instance,
                                                uint8_t xsensor_idx,
                                                sns_sensor_uid* suid)
{
  UNUSED_VAR(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

  float data[1];
  //to make sure not to overwrite
  if(inst_state->xgroup_info.xsensor_info[xsensor_idx].encoded_event_len)
    return;
  inst_state->xgroup_info.xsensor_info[xsensor_idx].encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);

  sns_memscpy(&inst_state->xgroup_info.suid[xsensor_idx],
      sizeof(inst_state->xgroup_info.suid[xsensor_idx]),
      suid,
      sizeof(inst_state->xgroup_info.suid[xsensor_idx]));

  DBG_INST_PRINTF(HIGH, instance,
      "copyng xsensor Sensor esp config");
}

/** See sns_lsm6dso_sensor.h */
sns_sensor_instance* lsm6dso_set_xsensor_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove,
                                                uint8_t idx,
                                                sns_sensor_uid* suid)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;

  if(!remove && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id)) {
    SNS_PRINTF(ERROR, this, "xsensor config req rejec, msg=%d", new_request->message_id);
    return instance;
  }
  SNS_PRINTF(HIGH, this, "client_req: sensor=%u req=%d/%d remove=%u hw_id=[%u]",
             state->sensor, exist_request != NULL ? exist_request->message_id : -1,
             new_request != NULL ? new_request->message_id : -1, remove,
             state->hardware_id);

  if(instance &&
      (lsm6dso_supported_xsensors[idx].stream_type == SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT)) {

    lsm6dso_instance_state *inst_state =
      (lsm6dso_instance_state*)instance->state->state;
    if(!(inst_state->xgroup_info.desired_sensors & lsm6dso_supported_xsensors[idx].sensor))
      lsm6dso_clear_request_q(this, instance, exist_request, suid, remove);
  }
  
  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);
  if(instance) {
    lsm6dso_instance_state *inst_state =
      (lsm6dso_instance_state*)instance->state->state;

    lsm6dso_fill_xsensor_inst_info(this, instance, idx, suid);

    inst_state->xgroup_info.desired_sensors &= ~state->sensor;
    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, suid, true))
      inst_state->xgroup_info.desired_sensors |= state->sensor;
    DBG_PRINTF(HIGH, this, "en: de = 0x%x : 0x%x",inst_state->xgroup_info.enabled_sensors, inst_state->xgroup_info.desired_sensors);
    inst_state->xgroup_info.xsensor_info[idx].client_present = (inst_state->xgroup_info.desired_sensors & state->sensor);
    if(remove || (inst_state->xgroup_info.enabled_sensors & state->sensor) != (inst_state->xgroup_info.desired_sensors & state->sensor))
     instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}

float lsm6dso_get_xsensor_rate(sns_sensor_instance *instance)
{
  UNUSED_VAR(instance);
  return LSM6DSO_FSM_MLC_ODR;
}
#endif //LSM6DSO_ESP_XSENSOR

