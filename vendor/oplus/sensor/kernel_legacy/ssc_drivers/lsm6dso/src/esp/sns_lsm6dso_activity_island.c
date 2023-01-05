/**
 * @file sns_lsm6dso_activity_island.c
 *
 * Common implementation for LSM6DSO activity Sensor
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

#if LSM6DSO_ESP_ACTIVITY
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_printf.h"

#include "sns_diag.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_activity.pb.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"

#define LSM6DSO_IS_ACTIVITY_INT(data) (data & 0x08)

extern sns_rc lsm6dso_activity_init(sns_sensor *const this);
extern sns_rc lsm6dso_inactivity_init(sns_sensor *const this);
extern sns_rc lsm6dso_activity_deinit(sns_sensor *const this);
extern sns_rc lsm6dso_inactivity_deinit(sns_sensor *const this);
extern void lsm6dso_process_act_registry_event(sns_sensor *const this, sns_sensor_event *event);

void lsm6dso_start_inact_timer(
    sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time();
  req_payload.start_config.early_start_delta =
        sns_convert_ns_to_ticks(1000000000.0f / state->current_conf.odr);
  req_payload.start_config.late_start_delta =
        sns_convert_ns_to_ticks(1000000000.0f / state->current_conf.odr);

  // QC - if the period does not need to be exact (say it can be +/- 20%) then it would be
  // good to add values for the "timeout_config" fields.  This will allow the timer sensor 
  // to synchronize this with other periodic timers which have a similar period.

  //timeout in sec, convert to ticks
  req_payload.timeout_period = sns_convert_ns_to_ticks(1000000000 * state->esp_info.act_info.config.inact_timeout);
  lsm6dso_inst_create_timer(instance, &state->esp_info.act_info.timer_inact_data_stream, &req_payload);
}

void lsm6dso_send_esp_event(sns_sensor_instance *const instance, lsm6dso_sensor_type sensor, void* event)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  pb_field_t const *event_fields;
  struct sns_sensor_uid* uid;
  uint32_t msg_id;
  if(sensor == LSM6DSO_ACTIVITY) {
    event_fields = &sns_activity_event_fields[0];
    uid = &state->esp_info.suid[ACT_INDX];
    msg_id = SNS_ACTIVITY_MSGID_SNS_ACTIVITY_EVENT;
    DBG_INST_PRINTF(LOW, instance, "[ESP Event] sensor :0x%x event :%d",
      sensor, ((sns_activity_event *)(event))->activity_event_type);
  } else {
    event_fields = &sns_inactivity_event_fields[0];
    uid = &state->esp_info.suid[INACT_INDX];
    msg_id = SNS_INACTIVITY_MSGID_SNS_INACTIVITY_EVENT;
    DBG_INST_PRINTF(LOW, instance, "[ESP Event] sensor :0x%x event :%d",
      sensor, ((sns_inactivity_event *)(event))->inactivity_event_type);
  }
  pb_send_event(instance,
      event_fields,
      event,
      sns_get_system_time(),
      msg_id,
      uid);
}


void lsm6dso_disable_activity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  if(send_event && (state->config_sensors & LSM6DSO_ACTIVITY)) {
    sns_activity_event event;
    event.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_DISABLED;
    lsm6dso_send_esp_event(instance, LSM6DSO_ACTIVITY, &event);
  }
  state->esp_info.enabled_sensors &= ~LSM6DSO_ACTIVITY;
  state->md_info.internal_client_present &= ~LSM6DSO_ACTIVITY;
  if(!lsm6dso_is_md_int_required(instance) && state->current_conf.md_enabled)
    lsm6dso_disable_md(instance, false);
}
void lsm6dso_enable_activity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  if(send_event && state->config_sensors & LSM6DSO_ACTIVITY) {
    sns_activity_event event;
    event.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_ENABLED;
    lsm6dso_send_esp_event(instance, LSM6DSO_ACTIVITY, &event);
  }
  state->esp_info.enabled_sensors |= LSM6DSO_ACTIVITY;
  //md is handled outside esp, keep special handling here
  //if(!state->current_conf.md_enabled && !state->timer_md_data_stream) {
  //    lsm6dso_enable_md(instance, false);

}

void lsm6dso_disable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  //md is handled outside esp, keep special handling here
  //if(!lsm6dso_is_md_int_required(instance) && state->current_conf.md_enabled)
  //  lsm6dso_disable_md(instance, false);
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_sensor_util_remove_sensor_instance_stream(instance, &state->esp_info.act_info.timer_inact_data_stream);
  if(send_event && (state->config_sensors & LSM6DSO_INACTIVITY)) {
    sns_inactivity_event event;
    event.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_DISABLED;
    lsm6dso_send_esp_event(instance, LSM6DSO_INACTIVITY, &event);
  }
  state->esp_info.enabled_sensors &= ~LSM6DSO_INACTIVITY;
  state->md_info.internal_client_present &= ~LSM6DSO_INACTIVITY;
  if(!lsm6dso_is_md_int_required(instance) && state->current_conf.md_enabled)
    lsm6dso_disable_md(instance, false);
}
void lsm6dso_enable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  if(!(state->esp_info.enabled_sensors & LSM6DSO_INACTIVITY)) {
    lsm6dso_start_inact_timer(instance);
    state->esp_info.enabled_sensors |= LSM6DSO_INACTIVITY;
  }
  if(send_event && state->config_sensors & LSM6DSO_INACTIVITY) {
    sns_inactivity_event event;
    event.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_ENABLED;
    lsm6dso_send_esp_event(instance, LSM6DSO_INACTIVITY, &event);
  }

  //md is handled outside esp, keep special handling here
  //if(!state->current_conf.md_enabled && !state->timer_md_data_stream) {
  //    lsm6dso_enable_md(instance, false);
}


void lsm6dso_handle_activity_intr(
    sns_sensor_instance *const instance,
    sns_time irq_timestamp,
    uint8_t src_reg,
    uint8_t wake_data)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  UNUSED_VAR(irq_timestamp);

  if(src_reg != STM_LSM6DSO_REG_WAKE_SRC)
    return;

  if(!state->current_conf.md_enabled || !state->md_info.is_filter_settled) {
    // ignore this interrupt, either md not enabled or filter is not settled
    return;
  }

  if((state->esp_info.enabled_sensors & (LSM6DSO_ACTIVITY | LSM6DSO_INACTIVITY)) &&
      LSM6DSO_IS_ACTIVITY_INT(wake_data)) {

    if(state->esp_info.enabled_sensors & LSM6DSO_INACTIVITY) {
      //restart timer
      lsm6dso_start_inact_timer(instance);
    }
    if(state->esp_info.enabled_sensors & LSM6DSO_ACTIVITY) {
      sns_activity_event act_state;
      act_state.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_FIRED;

      lsm6dso_send_esp_event(instance, LSM6DSO_ACTIVITY, &act_state);
      state->esp_info.desired_sensors &= ~LSM6DSO_ACTIVITY;
      lsm6dso_disable_activity(instance, false);
    }
  }
}

static sns_rc lsm6dso_handle_inact_timer(sns_sensor_instance *const instance , sns_time ts, sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(ts);
  UNUSED_VAR(latest_timer_event);
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_inactivity_event inact_state;
  inact_state.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_FIRED;

  //send inact_event
  lsm6dso_send_esp_event(instance, LSM6DSO_INACTIVITY, &inact_state);
  state->esp_info.desired_sensors &= ~LSM6DSO_INACTIVITY;
  lsm6dso_disable_inactivity(instance, false);
  return rv;
}

void lsm6dso_handle_inactivity_timer_events(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  if(NULL != state->esp_info.act_info.timer_inact_data_stream) {
    lsm6dso_handle_timer(instance, state->esp_info.act_info.timer_inact_data_stream,
        &lsm6dso_handle_inact_timer);
  }
}

void lsm6dso_reconfig_act(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(enable); 
  DBG_INST_PRINTF(LOW, instance, "reconfig_act d/e:%d/%d", state->esp_info.desired_sensors, state->esp_info.enabled_sensors);
  if(state->esp_info.desired_sensors & LSM6DSO_ACTIVITY)
  {
    lsm6dso_enable_activity(instance, false);
  } else if(state->esp_info.enabled_sensors & LSM6DSO_ACTIVITY) {
    lsm6dso_disable_activity(instance, false);
  }

}

void lsm6dso_reconfig_inact(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "reconfig_inact d/e:%d/%d", state->esp_info.desired_sensors, state->esp_info.enabled_sensors);
  UNUSED_VAR(enable); 
  if(state->esp_info.desired_sensors & LSM6DSO_INACTIVITY)
  {
    lsm6dso_enable_inactivity(instance, false);
  } else if(state->esp_info.enabled_sensors & LSM6DSO_INACTIVITY) {
    lsm6dso_disable_inactivity(instance, false);
  }
}

void lsm6dso_fill_act_inst_info(sns_sensor *const this, sns_sensor_instance *instance)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  float data[1];
  //to make sure not to overwrite
  if(inst_state->esp_info.act_info.encoded_event_len)
    return;

  inst_state->esp_info.act_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);
  inst_state->esp_info.act_info.timer_inact_data_stream = NULL;

#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[ACT_INDX],
        sizeof(inst_state->esp_info.suid[ACT_INDX]),
        &((sns_sensor_uid)ACTIVITY_SUID_1),
        sizeof(inst_state->esp_info.suid[ACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[INACT_INDX],
        sizeof(inst_state->esp_info.suid[INACT_INDX]),
        &((sns_sensor_uid)INACTIVITY_SUID_1),
        sizeof(inst_state->esp_info.suid[INACT_INDX]));
  } else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[ACT_INDX],
        sizeof(inst_state->esp_info.suid[ACT_INDX]),
        &((sns_sensor_uid)ACTIVITY_SUID_0),
        sizeof(inst_state->esp_info.suid[ACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[INACT_INDX],
        sizeof(inst_state->esp_info.suid[INACT_INDX]),
        &((sns_sensor_uid)INACTIVITY_SUID_0),
        sizeof(inst_state->esp_info.suid[INACT_INDX]));

  }
  sns_memscpy(&inst_state->esp_info.act_info.config, sizeof(inst_state->esp_info.act_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.act_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.act_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng ACT esp config");

}
/** See sns_lsm6dso_sensor.h */
sns_sensor_instance* lsm6dso_set_activity_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  uint8_t idx = 0;

  if(new_request == NULL || new_request->message_id != SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    SNS_PRINTF(HIGH, this, "client_req: sensor=%u req=%d/%d remove=%u hw_id=[%u]",
        state->sensor, exist_request != NULL ? exist_request->message_id : -1,
        new_request != NULL ? new_request->message_id : -1, remove,
        state->hardware_id);
  }

  if(!remove && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id) ) {
    SNS_PRINTF(ERROR, this, "activity req rejec, msg=%d", new_request->message_id);
    return instance;
  }

  if(state->sensor == LSM6DSO_ACTIVITY)
    idx = ACT_INDX;
  else
    idx = INACT_INDX;

  //if instance is already present, init corresponding sensor
  if(instance) {
    lsm6dso_instance_state *inst_state =
      (lsm6dso_instance_state*)instance->state->state;
    SNS_PRINTF(ERROR, this, "instance avail d_s = 0x%x sensor = 0x%x", inst_state->esp_info.desired_sensors, state->sensor);
    if(!(inst_state->esp_info.desired_sensors & state->sensor))
      //only incase of one-shot sensors: no need to use in other sensor types
      lsm6dso_clear_request_q(this, instance, exist_request, &inst_state->esp_info.suid[idx], remove);
  }
  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);
  if(instance) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    lsm6dso_fill_act_inst_info(this, instance);
    
    inst_state->esp_info.desired_sensors &= ~state->sensor;
  
    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[idx], true))
      inst_state->esp_info.desired_sensors |= state->sensor;

    //clear first internal client, if this sensor depends on md 
    inst_state->md_info.internal_client_present &= ~state->sensor;

    //if needed, update md internal client info
    inst_state->md_info.internal_client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    inst_state->esp_info.act_info.client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    if(remove || ((inst_state->esp_info.enabled_sensors & state->sensor) != (inst_state->esp_info.desired_sensors & state->sensor)))
      instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}

sns_rc lsm6dso_sensor_notify_activity_event(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_data_stream *stream = state->reg_data_stream;

  if(NULL != stream && 0 != stream->api->get_input_cnt(stream))
  {
    DBG_PRINTF_EX(HIGH, this, "registry_event: sensor=%u stream=%x", state->sensor, stream);
    for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
    {
      sns_sensor_event *event = stream->api->peek_input(stream);
      lsm6dso_process_act_registry_event(this, event);
    }
  }
  return lsm6dso_sensor_notify_event(this);
}


sns_sensor_api lsm6dso_activity_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_activity_init,
  .deinit             = &lsm6dso_activity_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_activity_request,
  .notify_event       = &lsm6dso_sensor_notify_activity_event,
};
sns_sensor_api lsm6dso_inactivity_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_inactivity_init,
  .deinit             = &lsm6dso_inactivity_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_activity_request,
  .notify_event       = &lsm6dso_sensor_notify_activity_event,
};

#endif //!LSM6DSO_ESP_ACTIVITY


