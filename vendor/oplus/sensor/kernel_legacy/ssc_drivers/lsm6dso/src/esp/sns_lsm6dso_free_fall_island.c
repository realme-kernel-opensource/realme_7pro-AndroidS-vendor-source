/**
 * @file sns_lsm6dso_free_fall_island.c
 *
 * Common implementation for LSM6DSO free fall Sensor
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

#if LSM6DSO_ESP_FREE_FALL
#include "sns_free_fall.pb.h"
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

const float ff_thresh[] = {1.53, 2.15, 2.45, 3.05, 3.37, 3.97, 4.6, 4.9}; //unit m/s2
#define LSM6DSO_FF_THRESH_MAX (ff_thresh[ARR_SIZE(ff_thresh)-1]) //unit m/s2
#define LSM6DSO_FF_THRESHOLD             (8)             //free fall threshold
#define LSM6DSO_FF_PERIOD                (8)             //free fall duration
#define LSM6DSO_FF_ODR_MIN               (832)           //Default sample rate for DBT
#define LSM6DSO_FF_REPORT_PERIOD         (1000 *1000)    //Default report  period for DBT in secs conv to msec

extern sns_rc lsm6dso_free_fall_init(sns_sensor *const this);
extern sns_rc lsm6dso_free_fall_deinit(sns_sensor *const this);
extern void lsm6dso_process_ff_registry_event(sns_sensor *const this, sns_sensor_event *event);

void lsm6dso_set_free_fall_config(sns_sensor_instance *const instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint8_t dur = LSM6DSO_FF_PERIOD; // bits 3 to 7 and bit7 of WAKE_UP_DUR
  uint8_t thresh = LSM6DSO_FF_THRESHOLD;    //thresh bits 0 to 2;
  uint32_t xfer_bytes;
  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  if(enable && state->esp_info.enabled_sensors & LSM6DSO_FREE_FALL) {
    //already configured
    return;
  }
  if(!enable && !(state->esp_info.enabled_sensors & LSM6DSO_FREE_FALL)) {
    //already disabled
    return;
  }

  if(enable && (accel_odr == LSM6DSO_ACCEL_ODR_OFF))
    accel_odr = lsm6dso_get_esp_rate_idx(instance);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "set_free_fall_config: en=%d des_odr(a)=0x%x",
      enable, state->accel_info.desired_odr);
  DBG_INST_PRINTF_EX(HIGH, instance,
      "ths(*1000)=%d dur(*1000)=%d",
      (int)(state->esp_info.ff_info.config.thresh *1000) ,
      (int)(state->esp_info.ff_info.config.dur*1000));
  if(enable && state->esp_info.ff_info.config.thresh <= LSM6DSO_FF_THRESH_MAX &&
      state->esp_info.ff_info.config.thresh >= 0.0)
  {
    int i = 0;
    // Threshold - bits 0 to 2 of STM_LSM6DSO_REG_FF_THS
    for(i = 0; i < ARR_SIZE(ff_thresh); i++) {
      if(state->esp_info.ff_info.config.thresh < ff_thresh[i])
        break;
    }
    thresh = i;
  }
  if(state->esp_info.ff_info.config.dur <= (float)0x1F / LSM6DSO_FF_ODR_MIN
      && state->esp_info.ff_info.config.dur >= 0.0)
  {
    dur = ((uint8_t)((roundf)(state->esp_info.ff_info.config.dur *
                                    LSM6DSO_FF_ODR_MIN)) & 0x1F);
  }
  if(enable)
    lsm6dso_set_interrupts(instance);

  rw_buffer = !enable ? 0xFF : ((dur << 3) & 0xF8) | (thresh & 0x07);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "th_set=0x%x dur_set=0x%x rw_buffer=0x%x",
      thresh, dur, rw_buffer);

  lsm6dso_com_write_wrapper(instance,
      STM_LSM6DSO_REG_FREE_FALL,
      &rw_buffer,
      1,
      &xfer_bytes,
      false);

  if((accel_odr != state->common_info.accel_curr_odr) || LSM6DSO_IS_ESP_PRESENCE_CHANGED(state)) {
    lsm6dso_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
    //Handle case when High-SHock is not enabled, but MD client is present and enabled.
    //We need to give time for MD filter to settle
    if(!(state->esp_info.desired_sensors & LSM6DSO_HIGH_SHOCK) &&
      (state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED) && !accel_odr) {
      DBG_INST_PRINTF(MED, instance,
                     "lsm6dso_set_free_fall_config:: re-enable MD interrupt to allow for settling time");
      lsm6dso_set_md_intr(instance, false);
      lsm6dso_set_md_config(instance, true);
      lsm6dso_update_md_intr(instance, true);
    }

  }

  DBG_INST_PRINTF_EX(HIGH, instance,
      "a_odr=0x%x cur_odr(a/g)=0x%x/0x%x",
      accel_odr, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);
}


void lsm6dso_update_ff_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  bool update_reg = false;
  DBG_INST_PRINTF_EX(LOW, instance,
                 "update_ff_intr: %d",
                 enable);
  if(enable &&  state->irq_info.irq_ready)
  {
    rw_buffer = 0x10;
    update_reg = true;
    state->esp_info.enabled_sensors |= LSM6DSO_FREE_FALL;
  }
  else if(enable && !state->irq_info.irq_ready)
  {
    state->esp_info.update_int |= LSM6DSO_FREE_FALL;
    DBG_INST_PRINTF(LOW, instance,
                   "update_ff_intr: IRQ not ready");
  }
  else if(!enable)
  {
    rw_buffer = 0;
    update_reg = true;
    state->esp_info.enabled_sensors &= ~LSM6DSO_FREE_FALL;
  }
  if(update_reg) {
    state->esp_info.update_int &= ~LSM6DSO_FREE_FALL;
    lsm6dso_read_modify_write(instance,
        STM_LSM6DSO_REG_MD1_CFG,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x10);
  }
}

static bool lsm6dso_check_ff_interrupt(lsm6dso_instance_state const *state,
                                                  uint8_t wake_data)
{
  UNUSED_VAR(state);
  return (wake_data & 0x20);
}

void lsm6dso_handle_free_fall_intr(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t src_reg,
                                 uint8_t wake_data)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  bool ret = false;

  if(src_reg != STM_LSM6DSO_REG_WAKE_SRC)
    return;


  if((state->esp_info.enabled_sensors & LSM6DSO_FREE_FALL) &&
      lsm6dso_check_ff_interrupt(state, wake_data))
  {
    sns_free_fall_event ff_state;
    ff_state.free_fall_event_type = SNS_FREE_FALL_EVENT_TYPE_FIRED;
    ret = pb_send_event(instance,
        sns_free_fall_event_fields,
        &ff_state,
        irq_timestamp,
        SNS_FREE_FALL_MSGID_SNS_FREE_FALL_EVENT,
        &state->esp_info.suid[FF_INDX]);

    DBG_INST_PRINTF_EX(LOW, instance,
        "Free Fall fired");

    state->wake_src = 0;

    if(ret) {
      DBG_INST_PRINTF_EX(LOW, instance,
          "event sent to framework successfully");
    } else {
      DBG_INST_PRINTF_EX(ERROR, instance,
          "event failed to sent to framework");
    }

#if !LSM6DSO_LOGGING_DISABLED
    // Sensor State HW Interrupt Log
    sns_diag_service* diag = state->diag_service;
    sns_diag_sensor_state_interrupt *log =
      (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
        diag,
        instance,
        &state->esp_info.suid[FF_INDX],
        sizeof(sns_diag_sensor_state_interrupt),
        SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT);
    if(NULL != log)
    {
      // QC - can be replaced with SNS_DIAG_INTERRUPT_FREEFALL once it's made available
      log->interrupt = SNS_DIAG_INTERRUPT_MOTION;
      log->timestamp = irq_timestamp;

      diag->api->submit_log(diag,
                            instance,
                            &state->esp_info.suid[FF_INDX],
                            sizeof(sns_diag_sensor_state_interrupt),
                            log,
                            SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT,
                            state->log_interrupt_encoded_size,
                            lsm6dso_encode_sensor_state_log_interrupt);
    } else {
      DBG_INST_PRINTF_EX(LOW, instance,
          "log is NULL");
    }
#endif
  }
}

void lsm6dso_reconfig_ff(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(enable);
  if(state->esp_info.desired_sensors & LSM6DSO_FREE_FALL) {
    if(!(state->esp_info.enabled_sensors & LSM6DSO_FREE_FALL)) {
      lsm6dso_set_free_fall_config(instance, true);
      lsm6dso_update_ff_intr(instance, true);
    } else if(state->esp_info.update_int & LSM6DSO_FREE_FALL) {
      lsm6dso_update_ff_intr(instance, true);
    }
  } else if(state->esp_info.enabled_sensors & LSM6DSO_FREE_FALL) {
    lsm6dso_set_free_fall_config(instance, false);
    lsm6dso_update_ff_intr(instance, false);
  }
}

void lsm6dso_fill_ff_inst_info(sns_sensor *const this, sns_sensor_instance *instance)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  float data[1];
  //to make sure not to overwrite
  if(inst_state->esp_info.ff_info.encoded_event_len)
    return;
  inst_state->esp_info.ff_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);

#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[FF_INDX],
        sizeof(inst_state->esp_info.suid[FF_INDX]),
        &((sns_sensor_uid)FREE_FALL_SUID_1),
        sizeof(inst_state->esp_info.suid[FF_INDX]));
  } else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[FF_INDX],
        sizeof(inst_state->esp_info.suid[FF_INDX]),
        &((sns_sensor_uid)FREE_FALL_SUID_0),
        sizeof(inst_state->esp_info.suid[FF_INDX]));
  }

  sns_memscpy(&inst_state->esp_info.ff_info.config, sizeof(inst_state->esp_info.ff_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng FF esp config");
}

/** See sns_lsm6dso_sensor.h */
sns_sensor_instance* lsm6dso_set_free_fall_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  
  if(!remove && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id)) {
    SNS_PRINTF(ERROR, this, "free fall req rejec, msg=%d", new_request->message_id);
    return instance;
  }

  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);

  if(instance) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    lsm6dso_fill_ff_inst_info(this, instance);

    inst_state->esp_info.desired_sensors &= ~state->sensor;

    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[FF_INDX], true))
      inst_state->esp_info.desired_sensors |= state->sensor;

    inst_state->esp_info.ff_info.client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    if(remove || ((inst_state->esp_info.enabled_sensors & state->sensor) != (inst_state->esp_info.desired_sensors & state->sensor)))
      instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}


sns_rc lsm6dso_sensor_notify_free_fall_event(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_data_stream *stream = state->reg_data_stream;

  if(NULL != stream && 0 != stream->api->get_input_cnt(stream))
  {
    DBG_PRINTF_EX(HIGH, this, "registry_event: sensor=%u stream=%x", state->sensor, stream);
    for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
    {
      sns_sensor_event *event = stream->api->peek_input(stream);
      lsm6dso_process_ff_registry_event(this, event);
    }
  }
  return lsm6dso_sensor_notify_event(this);
}

sns_sensor_api lsm6dso_free_fall_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_free_fall_init,
  .deinit             = &lsm6dso_free_fall_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_free_fall_request,
  .notify_event       = &lsm6dso_sensor_notify_free_fall_event,
};

#endif //LSM6DSO_ESP_FREE_FALL

