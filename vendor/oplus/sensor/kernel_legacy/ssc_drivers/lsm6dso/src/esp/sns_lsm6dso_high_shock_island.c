/**
 * @file sns_lsm6dso_high_shock_island.c
 *
 * Common implementation for LSM6DSO high shock Sensor
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

#if LSM6DSO_ESP_HIGH_SHOCK
#include "sns_high_shock.pb.h"
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

extern sns_rc lsm6dso_high_shock_init(sns_sensor *const this);
extern sns_rc lsm6dso_high_shock_deinit(sns_sensor *const this);
extern void lsm6dso_process_hs_registry_event(sns_sensor *const this, sns_sensor_event *event);

static void lsm6dso_hs_enable(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  DBG_INST_PRINTF_EX(LOW, instance,
      "Enabling High-Shock");

  if(accel_odr == LSM6DSO_ACCEL_ODR_OFF)
    accel_odr = lsm6dso_get_esp_rate(instance);

  //backup md threshold/win values
  state->esp_info.hs_info.config_md.thresh = state->md_info.md_config.thresh;
  state->esp_info.hs_info.config_md.win = state->md_info.md_config.win;

  //update the md_info with high_shock values as now we want to enable high_shock
  state->md_info.md_config.thresh = state->esp_info.hs_info.config.thresh;
  state->md_info.md_config.win = state->esp_info.hs_info.config.win;

  //Route MD/HIGH-SHOCK to INT2
  if(state->irq2_inst_enabled)
    state->route_md_to_irq2 = true;

  state->esp_info.enabled_sensors |= LSM6DSO_HIGH_SHOCK;

  //Turn ON High_Shock
  lsm6dso_set_md_config(instance, true);
  if(state->irq_ready)
    lsm6dso_update_md_intr(instance, true);
  else
    state->esp_info.update_int |= LSM6DSO_HIGH_SHOCK;

  if(accel_odr != state->common_info.accel_curr_odr)
  {
    lsm6dso_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
  }

}

static void lsm6dso_hs_disable(sns_sensor_instance *const instance,
                                 sns_time timestamp)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  DBG_INST_PRINTF_EX(LOW, instance,
               "Disabling HS");
  //update the md_info with MD values as now we want to switch to MD until HS is enabled again
  state->md_info.md_config.thresh = state->esp_info.hs_info.config_md.thresh;
  state->md_info.md_config.win = state->esp_info.hs_info.config_md.win;

  lsm6dso_handle_md_fired(instance,timestamp);
  //Route MD back to INT1
  state->route_md_to_irq2 = false;
  state->esp_info.enabled_sensors &= ~LSM6DSO_HIGH_SHOCK;

  //MD internal client present
  state->md_info.internal_client_present &= ~LSM6DSO_HIGH_SHOCK;

  if((accel_odr != state->common_info.accel_curr_odr) || (!accel_odr))
  {
    lsm6dso_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
  }
}

static sns_rc lsm6dso_handle_hs_timer(sns_sensor_instance *const instance , sns_time ts, sns_timer_sensor_event* latest_timer_event)
{
  sns_rc rv = SNS_RC_SUCCESS;
  UNUSED_VAR(latest_timer_event);
  lsm6dso_hs_disable(instance, ts);
  return rv;
}

void lsm6dso_handle_hs_timer_events(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  if(NULL != state->esp_info.hs_info.timer_hs_data_stream) {
    lsm6dso_handle_timer(instance, state->esp_info.hs_info.timer_hs_data_stream,
        &lsm6dso_handle_hs_timer);
  }
}

static bool lsm6dso_check_hs_interrupt(sns_sensor_instance *const instance,
                                       uint8_t wake_data)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  bool is_hs = false;
  rw_buffer = wake_data;

#if LSM6DSO_HS_MULTI_AXIS_DETECTION
  rw_buffer &= 0xf;
  SNS_INST_PRINTF(LOW, instance,
      "!!!! wake_src 0x%x", rw_buffer);
  if((rw_buffer == 0x0b) ||
     (rw_buffer == 0x0d) ||
     (rw_buffer == 0x0e) ||
     (rw_buffer == 0x0f))
    is_hs = true;
  else if((rw_buffer == 0x09) ||
          (rw_buffer == 0x0a) ||
          (rw_buffer == 0x0c))
    state->wake_src |= (rw_buffer & 0x07);
  if((state->wake_src == 0x03) ||
     (state->wake_src == 0x05) ||
     (state->wake_src == 0x06) ||
     (state->wake_src == 0x07)) {
    is_hs = true;
    DBG_INST_PRINTF(LOW, instance,
        "extended HS detected");
  }
#else
  is_hs = (rw_buffer & 0x08) ? true : false;
#endif
  return is_hs;
}

void lsm6dso_handle_high_shock_intr(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t src_reg,
                                 uint8_t wake_data)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  if(src_reg != STM_LSM6DSO_REG_WAKE_SRC)
    return;

  /**
   * 1. Handle HS interrupt: Send HS fired event to client.
   */
  if((state->esp_info.enabled_sensors & LSM6DSO_HIGH_SHOCK) &&
      lsm6dso_check_hs_interrupt(instance, wake_data))
  {
    sns_high_shock_event hs_state;
    hs_state.high_shock_event_type = SNS_HIGH_SHOCK_EVENT_TYPE_FIRED;
    pb_send_event(instance,
                  sns_motion_detect_event_fields,
                  &hs_state,
                  irq_timestamp,
                  SNS_HIGH_SHOCK_MSGID_SNS_HIGH_SHOCK_EVENT,
                  &state->esp_info.suid[HS_INDX]);

    DBG_INST_PRINTF(LOW, instance,
                 "HS fired");
    state->wake_src = 0;

    state->num_md_ints++;

#if !LSM6DSO_LOGGING_DISABLED
    sns_diag_service* diag = state->diag_service;
    // Sensor State HW Interrupt Log
    sns_diag_sensor_state_interrupt *log =
      (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
        diag,
        instance,
        &state->esp_info.suid[HS_INDX],
        sizeof(sns_diag_sensor_state_interrupt),
        SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT);

    if(NULL != log)
    {
      log->interrupt = SNS_DIAG_INTERRUPT_MOTION;
      log->timestamp = irq_timestamp;

      diag->api->submit_log(diag,
                            instance,
                            &state->esp_info.suid[HS_INDX],
                            sizeof(sns_diag_sensor_state_interrupt),
                            log,
                            SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT,
                            state->log_interrupt_encoded_size,
                            lsm6dso_encode_sensor_state_log_interrupt);
    }
#endif
  }
}

void lsm6dso_reconfig_hs(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(enable);
  if(state->esp_info.desired_sensors & LSM6DSO_HIGH_SHOCK)
  {
    if(!(state->esp_info.enabled_sensors & LSM6DSO_HIGH_SHOCK)) {
      //Disable MD if already enabled
      if(state->current_conf.md_enabled)
      {
        lsm6dso_handle_md_fired(instance, sns_get_system_time());
      }
      //Enable High_Shock sensor
      lsm6dso_hs_enable(instance);
    } else if(state->esp_info.update_int & LSM6DSO_HIGH_SHOCK) {
      lsm6dso_update_md_intr(instance, true);
    }
  } else if(state->esp_info.enabled_sensors & LSM6DSO_HIGH_SHOCK) {
    lsm6dso_hs_disable(instance,sns_get_system_time());
  }
}

void lsm6dso_fill_hs_inst_info(sns_sensor *const this, sns_sensor_instance *instance)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  float data[1];
  //to make sure not to overwrite
  if(inst_state->esp_info.hs_info.encoded_event_len)
    return;
  inst_state->esp_info.hs_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);

#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[HS_INDX],
        sizeof(inst_state->esp_info.suid[HS_INDX]),
        &((sns_sensor_uid)HIGH_SHOCK_SUID_1),
        sizeof(inst_state->esp_info.suid[HS_INDX]));
  } else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[HS_INDX],
        sizeof(inst_state->esp_info.suid[HS_INDX]),
        &((sns_sensor_uid)HIGH_SHOCK_SUID_0),
        sizeof(inst_state->esp_info.suid[HS_INDX]));
  }

  sns_memscpy(&inst_state->esp_info.hs_info.config, sizeof(inst_state->esp_info.hs_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng HS esp config");
}

/** See sns_lsm6dso_sensor.h */
sns_sensor_instance* lsm6dso_set_high_shock_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  
  if(!remove && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id)) {
    SNS_PRINTF(ERROR, this, "high shock req rejec, msg=%d", new_request->message_id);
    return instance;
  }

  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);

  if(instance) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    lsm6dso_fill_hs_inst_info(this, instance);

    inst_state->esp_info.desired_sensors &= ~state->sensor;

    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[HS_INDX], true))
      inst_state->esp_info.desired_sensors |= state->sensor;

    //clear first internal client, if this sensor depends on md
    inst_state->md_info.internal_client_present &= ~state->sensor;

    //if needed, update md internal client info
    inst_state->md_info.internal_client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    inst_state->esp_info.hs_info.client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    if(remove || ((inst_state->esp_info.enabled_sensors & state->sensor) != (inst_state->esp_info.desired_sensors & state->sensor)))
      instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}

sns_rc lsm6dso_sensor_notify_high_shock_event(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_data_stream *stream = state->reg_data_stream;

  if(NULL != stream && 0 != stream->api->get_input_cnt(stream))
  {
    DBG_PRINTF_EX(HIGH, this, "registry_event: sensor=%u stream=%x", state->sensor, stream);
    for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
    {
      sns_sensor_event *event = stream->api->peek_input(stream);
      lsm6dso_process_hs_registry_event(this, event);
    }
  }
  return lsm6dso_sensor_notify_event(this);
}

sns_sensor_api lsm6dso_high_shock_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_high_shock_init,
  .deinit             = &lsm6dso_high_shock_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_high_shock_request,
  .notify_event       = &lsm6dso_sensor_notify_high_shock_event,
};


#endif //LSM6DSO_ESP_HIGH_SHOCK

