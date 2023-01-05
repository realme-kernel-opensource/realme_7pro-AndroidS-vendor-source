/**
 * @file sns_lsm6ds3c_esp_island.c
 *
 * Common implementation for LSM6DS3C Sensors.
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
#include <math.h>
#include <string.h>

#include "sns_lsm6ds3c_sensor.h"
#include "sns_types.h"

#if LSM6DS3C_ESP_ENABLED
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
#include "sns_event_service.h"

sns_sensor_api lsm6ds3c_free_fall_sensor_api;
sns_sensor_api lsm6ds3c_high_shock_sensor_api;
sns_sensor_api lsm6ds3c_activity_sensor_api;
sns_sensor_api lsm6ds3c_inactivity_sensor_api;
sns_sensor_api lsm6ds3c_step_counter_sensor_api;
extern sns_rc lsm6ds3c_sensor_notify_event(sns_sensor *const this);
extern const odr_reg_map lsm6ds3c_odr_map[];
extern const uint32_t lsm6ds3c_odr_map_len;

const lsm6ds3c_esp_sensors lsm6ds3c_supported_esp_sensors [ ] = {
  { "free_fall",  "sns_free_fall.proto",  LSM6DS3C_FREE_FALL, LSM6DS3C_ODR_416,  SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE, &lsm6ds3c_free_fall_sensor_api },
  { "high_shock", "sns_high_shock.proto", LSM6DS3C_HIGH_SHOCK, LSM6DS3C_ODR_416, SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE, &lsm6ds3c_high_shock_sensor_api },
  { "st_activity", "sns_activity.proto",   LSM6DS3C_ACTIVITY,   LSM6DS3C_ODR_26, SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT, &lsm6ds3c_activity_sensor_api },
  { "st_inactivity", "sns_activity.proto", LSM6DS3C_INACTIVITY, LSM6DS3C_ODR_26, SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT, &lsm6ds3c_inactivity_sensor_api },
  { "step_counter", "sns_step_counter.proto", LSM6DS3C_STEP_COUNTER, LSM6DS3C_ODR_26, SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT, &lsm6ds3c_step_counter_sensor_api },
};

const float ff_thresh[] = {1.53, 2.15, 2.45, 3.05, 3.37, 3.97, 4.6, 4.9}; //unit m/s2
#define LSM6DS3C_FF_THRESH_MAX (ff_thresh[ARR_SIZE(ff_thresh)-1]) //unit m/s2
#define LSM6DS3C_IS_ACTIVITY_INT(data) (data & 0x08)

float lsm6ds3c_get_esp_rate(sns_sensor_instance *instance)
{
  lsm6ds3c_instance_state *inst_state =
    (lsm6ds3c_instance_state*)instance->state->state;
  uint8_t sensor_count = ARR_SIZE(lsm6ds3c_supported_esp_sensors);
  float max_odr = 0.0f;
  for(int i=0; i< sensor_count ; i++) {
    if(inst_state->esp_info.desired_sensors & lsm6ds3c_supported_esp_sensors[i].sensor)
      max_odr = SNS_MAX(max_odr, lsm6ds3c_supported_esp_sensors[i].min_odr);
  }
  return max_odr;
}

uint16_t lsm6ds3c_get_esp_rate_idx(sns_sensor_instance *instance)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  float esp_sample_rate = lsm6ds3c_get_esp_rate(instance);
  uint8_t idx = lsm6ds3c_get_odr_rate_idx(esp_sample_rate);
  idx = SNS_MAX(idx, state->min_odr_idx);
  return lsm6ds3c_odr_map[idx].accel_odr_reg_value;
}

#if LSM6DS3C_ESP_FREE_FALL
void lsm6ds3c_set_free_fall_config(sns_sensor_instance *const instance, bool enable)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint8_t dur = LSM6DS3C_FF_PERIOD; // bits 3 to 7 and bit7 of WAKE_UP_DUR
  uint8_t thresh = LSM6DS3C_FF_THRESHOLD;    //thresh bits 0 to 2;
  uint32_t xfer_bytes;
  lsm6ds3c_accel_odr accel_odr = state->accel_info.desired_odr;
  if(enable && state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL) {
    //already configured
    return;
  }
  if(!enable && !(state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL)) {
    //already disabled
    return;
  }

  if(enable && (accel_odr == LSM6DS3C_ACCEL_ODR_OFF))
    accel_odr = lsm6ds3c_get_esp_rate_idx(instance);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "set_free_fall_config: en=%d des_odr(a)=0x%x",
      enable, state->accel_info.desired_odr);
  DBG_INST_PRINTF_EX(HIGH, instance,
      "ths(*1000)=%d dur(*1000)=%d",
      (int)(state->esp_info.ff_info.config.thresh *1000) ,
      (int)(state->esp_info.ff_info.config.dur*1000));
  if(enable && state->esp_info.ff_info.config.thresh <= LSM6DS3C_FF_THRESH_MAX &&
      state->esp_info.ff_info.config.thresh >= 0.0)
  {
    int i = 0;
    // Threshold - bits 0 to 2 of STM_LSM6DS3C_REG_FF_THS
    for(i = 0; i < ARR_SIZE(ff_thresh); i++) {
      if(state->esp_info.ff_info.config.thresh < ff_thresh[i])
        break;
    }
    thresh = i;
  }
  if(state->esp_info.ff_info.config.dur <= (float)0x1F / LSM6DS3C_FF_ODR_MIN
      && state->esp_info.ff_info.config.dur >= 0.0)
  {
    dur = ((uint8_t)((roundf)(state->esp_info.ff_info.config.dur *
                                    LSM6DS3C_FF_ODR_MIN)) & 0x1F);
  }
  if(enable)
    lsm6ds3c_set_interrupts(instance);

  rw_buffer = !enable ? 0xFF : ((dur << 3) & 0xF8) | (thresh & 0x07);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "th_set=0x%x dur_set=0x%x rw_buffer=0x%x",
      thresh, dur, rw_buffer);

  lsm6ds3c_com_write_wrapper(instance,
      STM_LSM6DS3C_REG_FREE_FALL,
      &rw_buffer,
      1,
      &xfer_bytes,
      false);

  if((accel_odr != state->common_info.accel_curr_odr) || LSM6DS3C_IS_ESP_PRESENCE_CHANGED(state)) {
    lsm6ds3c_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DS3C_ACCEL_BW50);
    //Handle case when High-SHock is not enabled, but MD client is present and enabled.
    //We need to give time for MD filter to settle
    if(!(state->esp_info.desired_sensors & LSM6DS3C_HIGH_SHOCK) &&
      (state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED) && !accel_odr) {
      DBG_INST_PRINTF(MED, instance,
                     "lsm6ds3c_set_free_fall_config:: re-enable MD interrupt to allow for settling time");
      lsm6ds3c_set_md_intr(instance, false);
      lsm6ds3c_set_md_config(instance, true);
      lsm6ds3c_update_md_intr(instance, true);
    }

  }

  DBG_INST_PRINTF_EX(HIGH, instance,
      "a_odr=0x%x cur_odr(a/g)=0x%x/0x%x",
      accel_odr, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);
}


void lsm6ds3c_update_ff_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)instance->state->state;

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
    state->esp_info.enabled_sensors |= LSM6DS3C_FREE_FALL;
  }
  else if(enable && !state->irq_info.irq_ready)
  {
    state->esp_info.update_int |= LSM6DS3C_FREE_FALL;
    DBG_INST_PRINTF(LOW, instance,
                   "update_ff_intr: IRQ not ready");
  }
  else if(!enable)
  {
    rw_buffer = 0;
    update_reg = true;
    state->esp_info.enabled_sensors &= ~LSM6DS3C_FREE_FALL;
  }
  if(update_reg) {
    state->esp_info.update_int &= ~LSM6DS3C_FREE_FALL;
    lsm6ds3c_read_modify_write(instance,
        STM_LSM6DS3C_REG_MD1_CFG,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x10);
  }
}
#else
void lsm6ds3c_set_free_fall_config(sns_sensor_instance *const instance, bool enable)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(enable);
}

void lsm6ds3c_update_ff_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(enable);
}
#endif

#if LSM6DS3C_ESP_HIGH_SHOCK
static void lsm6ds3c_hs_enable(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  lsm6ds3c_accel_odr accel_odr = state->accel_info.desired_odr;
  DBG_INST_PRINTF_EX(LOW, instance,
      "Enabling High-Shock");

  if(accel_odr == LSM6DS3C_ACCEL_ODR_OFF)
    accel_odr = lsm6ds3c_get_esp_rate(instance);

  //backup md threshold/win values
  state->esp_info.hs_info.config_md.thresh = state->md_info.md_config.thresh;
  state->esp_info.hs_info.config_md.win = state->md_info.md_config.win;

  //update the md_info with high_shock values as now we want to enable high_shock
  state->md_info.md_config.thresh = state->esp_info.hs_info.config.thresh;
  state->md_info.md_config.win = state->esp_info.hs_info.config.win;

  //Route MD/HIGH-SHOCK to INT2
  state->route_md_to_irq2 = true;

  state->esp_info.enabled_sensors |= LSM6DS3C_HIGH_SHOCK;

  //Turn ON High_Shock
  lsm6ds3c_set_md_config(instance, true);
  if(state->irq_ready)
    lsm6ds3c_update_md_intr(instance, true);
  else
    state->esp_info.update_int |= LSM6DS3C_HIGH_SHOCK;

  if(accel_odr != state->common_info.accel_curr_odr)
  {
    lsm6ds3c_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DS3C_ACCEL_BW50);
  }

}

static void lsm6ds3c_hs_disable(sns_sensor_instance *const instance,
                                 sns_time timestamp)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  lsm6ds3c_accel_odr accel_odr = state->accel_info.desired_odr;
  DBG_INST_PRINTF_EX(LOW, instance,
               "Disabling HS");
  //update the md_info with MD values as now we want to switch to MD until HS is enabled again
  state->md_info.md_config.thresh = state->esp_info.hs_info.config_md.thresh;
  state->md_info.md_config.win = state->esp_info.hs_info.config_md.win;

  lsm6ds3c_handle_md_fired(instance,timestamp);
  //Route MD back to INT1
  state->route_md_to_irq2 = false;
  state->esp_info.enabled_sensors &= ~LSM6DS3C_HIGH_SHOCK;

  //MD internal client present
  state->md_info.internal_client_present &= ~LSM6DS3C_HIGH_SHOCK;

  if((accel_odr != state->common_info.accel_curr_odr) || (!accel_odr))
  {
    lsm6ds3c_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DS3C_ACCEL_BW50);
  }
}
#else
static void lsm6ds3c_hs_enable(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

static void lsm6ds3c_hs_disable(sns_sensor_instance *const instance,
                                 sns_time timestamp)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(timestamp);
}
#endif

#if LSM6DS3C_ESP_ACTIVITY
void lsm6ds3c_start_inact_timer(
    sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
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
  lsm6ds3c_inst_create_timer(instance, &state->esp_info.act_info.timer_inact_data_stream, &req_payload);
}

void lsm6ds3c_send_esp_event(sns_sensor_instance *const instance, lsm6ds3c_sensor_type sensor, void* event)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)instance->state->state;
  pb_field_t const *event_fields;
  struct sns_sensor_uid* uid;
  uint32_t msg_id;
  if(sensor == LSM6DS3C_ACTIVITY) {
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

void lsm6ds3c_disable_activity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  if(send_event && (state->config_sensors & LSM6DS3C_ACTIVITY)) {
    sns_activity_event event;
    event.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_DISABLED;
    lsm6ds3c_send_esp_event(instance, LSM6DS3C_ACTIVITY, &event);
  }
  state->esp_info.enabled_sensors &= ~LSM6DS3C_ACTIVITY;
  state->md_info.internal_client_present &= ~LSM6DS3C_ACTIVITY;
  if(!lsm6ds3c_is_md_int_required(instance) && state->current_conf.md_enabled)
    lsm6ds3c_disable_md(instance, false);
}
void lsm6ds3c_enable_activity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  if(send_event && state->config_sensors & LSM6DS3C_ACTIVITY) {
    sns_activity_event event;
    event.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_ENABLED;
    lsm6ds3c_send_esp_event(instance, LSM6DS3C_ACTIVITY, &event);
  }
  state->esp_info.enabled_sensors |= LSM6DS3C_ACTIVITY;
  //md is handled outside esp, keep special handling here
  //if(!state->current_conf.md_enabled && !state->timer_md_data_stream) {
  //    lsm6ds3c_enable_md(instance, false);

}

void lsm6ds3c_disable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  //md is handled outside esp, keep special handling here
  //if(!lsm6ds3c_is_md_int_required(instance) && state->current_conf.md_enabled)
  //  lsm6ds3c_disable_md(instance, false);
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_sensor_util_remove_sensor_instance_stream(instance, &state->esp_info.act_info.timer_inact_data_stream);
  if(send_event && (state->config_sensors & LSM6DS3C_INACTIVITY)) {
    sns_inactivity_event event;
    event.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_DISABLED;
    lsm6ds3c_send_esp_event(instance, LSM6DS3C_INACTIVITY, &event);
  }
  state->esp_info.enabled_sensors &= ~LSM6DS3C_INACTIVITY;
  state->md_info.internal_client_present &= ~LSM6DS3C_INACTIVITY;
  if(!lsm6ds3c_is_md_int_required(instance) && state->current_conf.md_enabled)
    lsm6ds3c_disable_md(instance, false);
}
void lsm6ds3c_enable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;

  if(!(state->esp_info.enabled_sensors & LSM6DS3C_INACTIVITY)) {
    lsm6ds3c_start_inact_timer(instance);
    state->esp_info.enabled_sensors |= LSM6DS3C_INACTIVITY;
  }
  if(send_event && state->config_sensors & LSM6DS3C_INACTIVITY) {
    sns_inactivity_event event;
    event.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_ENABLED;
    lsm6ds3c_send_esp_event(instance, LSM6DS3C_INACTIVITY, &event);
  }

  //md is handled outside esp, keep special handling here
  //if(!state->current_conf.md_enabled && !state->timer_md_data_stream) {
  //    lsm6ds3c_enable_md(instance, false);
}
#else
void lsm6ds3c_send_esp_event(sns_sensor_instance *const instance, lsm6ds3c_sensor_type sensor, void* event)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(sensor);
  UNUSED_VAR(event);
}
void lsm6ds3c_disable_activity(sns_sensor_instance *const instance, bool send_event)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(send_event);
}
void lsm6ds3c_enable_activity(sns_sensor_instance *const instance, bool send_event)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(send_event);
}
void lsm6ds3c_disable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(send_event);
}
void lsm6ds3c_enable_inactivity(sns_sensor_instance *const instance, bool send_event)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(send_event);
}
#endif

#if LSM6DS3C_ESP_STEP_COUNTER
static void lsm6ds3c_sc_enable(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  if(state->esp_info.enabled_sensors & LSM6DS3C_STEP_COUNTER) {
    return;
  }

  if(!state->irq_info.irq_ready)
  {
    state->esp_info.update_int |= LSM6DS3C_STEP_COUNTER;
    DBG_INST_PRINTF(LOW, instance,
                   "sc_enable: IRQ not ready");
    return;
  }
  DBG_INST_PRINTF_EX(HIGH, instance,
      "enable step counter");
  //Enable embedded functions and pedometer algorithm
  rw_buffer = 0x0
       | (0<<7)           // WRIST_TILT_EN
       | (0<<6)           //
       | (0<<5)           // TIMER_EN
       | (1<<4)           // PEDO_EN
       | (0<<3)           // TILT_EN
       | (1<<2)           // FUNC_EN
       | (0<<1)           // PEDO_RST_STEP
       | 0;               // SIGN_MOTION_EN
  rv = lsm6ds3c_read_modify_write(instance,
           STM_LSM6DS3C_REG_CTRL10,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DS3C_STEP_COUNTER_EN_MASK);

  //Step detector interrupt driven to INT1 pin
  rw_buffer = 0x0
       | (1<<7)           // INT1_STEP_DETECTOR
       | (0<<6)           // INT1_SIGN_MOT
       | (0<<5)           // INT1_FULL_FLAG
       | (0<<4)           // INT1_FIFO_OVR
       | (0<<3)           // INT1_FTH
       | (0<<2)           // INT1_BOOT
       | (0<<1)           // INT1_DRDY_G
       | 0;               // INT1_DRDY_XL
  rv = lsm6ds3c_read_modify_write(instance,
           STM_LSM6DS3C_REG_INT1_CTRL,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DS3C_STEP_DETCTOR_INT_MASK);

  state->esp_info.enabled_sensors |= LSM6DS3C_STEP_COUNTER;

#if LSM6DS3C_OEM_FACTORY_CONFIG
  lsm6ds3c_sc_report_steps(instance);
#endif

  DBG_INST_PRINTF(HIGH, instance,
      "a_odr=0x%x cur_odr(a/g)=0x%x/0x%x",
      state->accel_info.desired_odr, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);
}

static void lsm6ds3c_sc_disable(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  //vODO: FN_EN is disabling here, might be useful for other sensors
  rv = lsm6ds3c_read_modify_write(instance,
           STM_LSM6DS3C_REG_CTRL10,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DS3C_STEP_COUNTER_EN_MASK);
  rv = lsm6ds3c_read_modify_write(instance,
           STM_LSM6DS3C_REG_INT1_CTRL,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DS3C_STEP_DETCTOR_INT_MASK);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "step counter disabled");
  state->esp_info.enabled_sensors &= ~LSM6DS3C_STEP_COUNTER;
}
#else
static void lsm6ds3c_sc_enable(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}
static void lsm6ds3c_sc_disable(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}
#endif

void lsm6ds3c_update_pending_esp_intr(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  if((state->esp_info.update_int & LSM6DS3C_FREE_FALL) && (state->esp_info.desired_sensors & LSM6DS3C_FREE_FALL))
  {
    lsm6ds3c_update_ff_intr(instance, true);
  }
  if((state->esp_info.update_int & LSM6DS3C_HIGH_SHOCK) && (state->esp_info.desired_sensors & LSM6DS3C_HIGH_SHOCK))
  {
    lsm6ds3c_update_md_intr(instance, true);
  }
  if((state->esp_info.update_int & LSM6DS3C_STEP_COUNTER) && (state->esp_info.desired_sensors & LSM6DS3C_STEP_COUNTER))
  {
    lsm6ds3c_sc_enable(instance);
  }
}

void lsm6ds3c_powerdown_esp(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  if(state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL) {
    lsm6ds3c_set_free_fall_config(instance, false);
    lsm6ds3c_update_ff_intr(instance, false);
  }
  if(state->esp_info.enabled_sensors & LSM6DS3C_HIGH_SHOCK) {
    lsm6ds3c_hs_disable(instance,sns_get_system_time());
  }
  if(state->esp_info.enabled_sensors & LSM6DS3C_ACTIVITY) {
    lsm6ds3c_disable_activity(instance, false);
  }
  if(state->esp_info.enabled_sensors & LSM6DS3C_INACTIVITY) {
    lsm6ds3c_disable_inactivity(instance, false);
  }
  if(state->esp_info.enabled_sensors & LSM6DS3C_STEP_COUNTER) {
    lsm6ds3c_sc_disable(instance);
  }
}

void lsm6ds3c_poweron_esp(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  if(state->esp_info.desired_sensors & LSM6DS3C_FREE_FALL) {
    if(!(state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL)) {
      lsm6ds3c_set_free_fall_config(instance, true);
      lsm6ds3c_update_ff_intr(instance, true);
    }
  } else if(state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL) {
    lsm6ds3c_set_free_fall_config(instance, false);
    lsm6ds3c_update_ff_intr(instance, false);
  }
  if(state->esp_info.desired_sensors & LSM6DS3C_HIGH_SHOCK)
  {
    //Disable MD if already enabled
    if(state->current_conf.md_enabled)
    {
      lsm6ds3c_handle_md_fired(instance, sns_get_system_time());
    }
    //Enable High_Shock sensor
    lsm6ds3c_hs_enable(instance);
  } else if(state->esp_info.enabled_sensors & LSM6DS3C_HIGH_SHOCK) {
    lsm6ds3c_hs_disable(instance,sns_get_system_time());
  }

  if(state->esp_info.desired_sensors & LSM6DS3C_ACTIVITY)
  {
    lsm6ds3c_enable_activity(instance, false);
  } else if(state->esp_info.enabled_sensors & LSM6DS3C_ACTIVITY) {
    lsm6ds3c_disable_activity(instance, false);
  }
  if(state->esp_info.desired_sensors & LSM6DS3C_INACTIVITY)
  {
    lsm6ds3c_enable_inactivity(instance, false);
  } else if(state->esp_info.enabled_sensors & LSM6DS3C_INACTIVITY) {
    lsm6ds3c_disable_inactivity(instance, false);
  }
  if(state->esp_info.desired_sensors & LSM6DS3C_STEP_COUNTER)
  {
    lsm6ds3c_sc_enable(instance);
  } else if(state->esp_info.enabled_sensors & LSM6DS3C_STEP_COUNTER) {
    lsm6ds3c_sc_disable(instance);
  }
}

#if LSM6DS3C_ESP_FREE_FALL
static bool lsm6ds3c_check_ff_interrupt(lsm6ds3c_instance_state const *state,
                                                  uint8_t wake_data)
{
  UNUSED_VAR(state);
  return (wake_data & 0x20);
}

static void lsm6ds3c_handle_ff_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t wake_data)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  bool ret = false;
  if(lsm6ds3c_check_ff_interrupt(state, wake_data))
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

#if !LSM6DS3C_LOGGING_DISABLED
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
                            lsm6ds3c_encode_sensor_state_log_interrupt);
    } else {
      DBG_INST_PRINTF_EX(LOW, instance,
          "log is NULL");
    }
#endif
  }
}
#else
static void lsm6ds3c_handle_ff_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t wake_data)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(irq_timestamp);
  UNUSED_VAR(wake_data);
}
#endif

#if LSM6DS3C_ESP_ACTIVITY
static void lsm6ds3c_handle_activity_interrupt(
    sns_sensor_instance *const instance,
    sns_time irq_timestamp,
    uint8_t wake_data)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;

  UNUSED_VAR(irq_timestamp);
  if(LSM6DS3C_IS_ACTIVITY_INT(wake_data)) {

    if(state->esp_info.enabled_sensors & LSM6DS3C_INACTIVITY) {
      //restart timer
      lsm6ds3c_start_inact_timer(instance);
    }
    if(state->esp_info.enabled_sensors & LSM6DS3C_ACTIVITY) {
      sns_activity_event act_state;
      act_state.activity_event_type = SNS_ACTIVITY_EVENT_TYPE_FIRED;

      lsm6ds3c_send_esp_event(instance, LSM6DS3C_ACTIVITY, &act_state);
      state->esp_info.desired_sensors &= ~LSM6DS3C_ACTIVITY;
      lsm6ds3c_disable_activity(instance, false);
    }
  }
}

void lsm6ds3c_handle_inact_timer(sns_sensor_instance *const instance , sns_time ts)
{
  UNUSED_VAR(ts);
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_inactivity_event inact_state;
  inact_state.inactivity_event_type = SNS_INACTIVITY_EVENT_TYPE_FIRED;

  //send inact_event
  lsm6ds3c_send_esp_event(instance, LSM6DS3C_INACTIVITY, &inact_state);
  state->esp_info.desired_sensors &= ~LSM6DS3C_INACTIVITY;
  lsm6ds3c_disable_inactivity(instance, false);
}

#else
static void lsm6ds3c_handle_activity_interrupt(
    sns_sensor_instance *const instance,
    sns_time irq_timestamp,
    uint8_t wake_data)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(irq_timestamp);
  UNUSED_VAR(wake_data);
}
void lsm6ds3c_handle_inact_timer(sns_sensor_instance *const instance , sns_time ts)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(ts);
}
#endif

#if LSM6DS3C_ESP_HIGH_SHOCK
static bool lsm6ds3c_check_hs_interrupt(sns_sensor_instance *const instance,
                                       uint8_t wake_data)
{
  uint8_t rw_buffer = 0;
  bool is_hs = false;
  rw_buffer = wake_data;

#if LSM6DS3C_HS_MULTI_AXIS_DETECTION
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;

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
  UNUSED_VAR(instance);
  is_hs = (rw_buffer & 0x08) ? true : false;
#endif
  return is_hs;
}

static void lsm6ds3c_handle_hs_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t wake_data)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;

  /**
   * 1. Handle HS interrupt: Send HS fired event to client.
   */
  if(lsm6ds3c_check_hs_interrupt(instance, wake_data))
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

#if !LSM6DS3C_LOGGING_DISABLED
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
                            lsm6ds3c_encode_sensor_state_log_interrupt);
    }
#endif
  }
}
#else
static void lsm6ds3c_handle_hs_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t wake_data)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(irq_timestamp);
  UNUSED_VAR(wake_data);
}
#endif

#if LSM6DS3C_ESP_STEP_COUNTER
static void lsm6ds3c_send_esp_stream_event(sns_sensor_instance *const instance,
                lsm6ds3c_sensor_type sensor, sns_time timestamp, float *data)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)instance->state->state;
  struct sns_sensor_uid* uid;
  size_t encoded_event_len;

  if(sensor == LSM6DS3C_STEP_COUNTER)
  {
    uid = &state->esp_info.suid[SC_INDX];
    encoded_event_len = state->esp_info.sc_info.encoded_event_len;
    pb_send_sensor_stream_event(instance,
                                uid,
                                timestamp,
                                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                data,
                                1,
                                encoded_event_len);
  }
}

static void lsm6ds3c_convert_and_send_sc_sample(sns_sensor_instance *const instance,
                                   sns_time timestamp,
                                   const uint8_t sc_data[2])
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)instance->state->state;

  if(state->esp_info.enabled_sensors & LSM6DS3C_STEP_COUNTER)
  {
    uint16_t stepCount_cur = (uint16_t)(sc_data[1] << 8 | sc_data[0]);
    if(state->esp_info.sc_info.stepCount_prev == stepCount_cur)
    {
      return;
    }
    else if(state->esp_info.sc_info.stepCount_prev > stepCount_cur)
    {
      state->esp_info.sc_info.stepCount_base += state->esp_info.sc_info.stepCount_prev;
    }
    state->esp_info.sc_info.stepCount_prev = stepCount_cur;
    float float_sc_val = (float)(state->esp_info.sc_info.stepCount_base + state->esp_info.sc_info.stepCount_prev);

    if(state->self_test_info.test_alive)
    {
      DBG_INST_PRINTF_EX(HIGH, instance,
                     "test_alive");
    }
    else
    {
      lsm6ds3c_send_esp_stream_event(instance, LSM6DS3C_STEP_COUNTER, timestamp, &float_sc_val);
    }
    DBG_INST_PRINTF_EX(HIGH, instance,
        "step_count=%d ts=%u", (int)float_sc_val, (uint32_t)timestamp);

#if !LSM6DS3C_LOGGING_DISABLED
    log_sensor_state_raw_info log_sc_state_raw_info;
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    sns_diag_service* diag = state->diag_service;

    sns_memzero(&log_sc_state_raw_info, sizeof(log_sc_state_raw_info));
    log_sc_state_raw_info.encoded_sample_size = state->esp_info.sc_info.encoded_raw_log_len;
    log_sc_state_raw_info.diag = diag;
    log_sc_state_raw_info.instance = instance;
    log_sc_state_raw_info.sensor_uid = &state->esp_info.suid[SC_INDX];
    lsm6ds3c_log_sensor_state_raw_alloc(&log_sc_state_raw_info, 0);

    lsm6ds3c_log_sensor_state_raw_add(
        &log_sc_state_raw_info,
        &float_sc_val,
        1,
        timestamp,
        status);
    lsm6ds3c_log_sensor_state_raw_submit(&log_sc_state_raw_info, 1, true);
#endif
  }
}

static void lsm6ds3c_handle_sc_interrupt(sns_sensor_instance *const instance,
                                 sns_time timestamp, uint8_t func_src)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)instance->state->state;
  uint8_t sc_data[2] = {0};
  uint32_t xfer_bytes;

  if((func_src & 0x10) == 0)
  {
    return;
  }

  lsm6ds3c_com_read_wrapper(state->scp_service,
           state->com_port_info.port_handle,
           STM_LSM6DS3C_REG_STEP_COUNTER_L,
           &sc_data[0],
           2,
           &xfer_bytes);
  if(xfer_bytes != 2)
  {
    return;
  }

  lsm6ds3c_convert_and_send_sc_sample(instance, timestamp, sc_data);
}

uint16_t read_step_count(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state =
     (lsm6ds3c_instance_state*)this->state->state;
  uint8_t sc_data[2] = {0};
  uint32_t xfer_bytes;

  lsm6ds3c_com_read_wrapper(state->scp_service,
           state->com_port_info.port_handle,
           STM_LSM6DS3C_REG_STEP_COUNTER_L,
           &sc_data[0],
           2,
           &xfer_bytes);
  if(xfer_bytes != 2)
  {
    return 0;
  }

  return (uint16_t)(sc_data[1] << 8 | sc_data[0]);
}
#else
static void lsm6ds3c_handle_sc_interrupt(sns_sensor_instance *const instance,
                                 sns_time timestamp, uint8_t func_src)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(timestamp);
  UNUSED_VAR(func_src);
}
uint16_t read_step_count(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
}
#endif

sns_rc lsm6ds3c_handle_timer(
    sns_sensor_instance *const instance, sns_data_stream* timer_data_stream,
    void (*timer_handler)(sns_sensor_instance *const , sns_time))
{
  sns_sensor_event *event;
  uint16_t num_events = 0;
  event = timer_data_stream->api->peek_input(timer_data_stream);
  while(NULL != event)
  {
    pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
        event->event_len);
    sns_timer_sensor_event timer_event;
    if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        num_events++;
        DBG_INST_PRINTF(LOW, instance,
            "esp_timer evernts:%d",num_events);
      }
    }
    else
    {
    }
    event = timer_data_stream->api->get_next_input(timer_data_stream);
  }
  if(num_events) {
    timer_handler(instance,sns_get_system_time());
  }
  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3c_handle_esp_timer_events(sns_sensor_instance *const instance)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if(NULL != state->esp_info.hs_info.timer_hs_data_stream)
  {
    lsm6ds3c_handle_timer(instance, state->esp_info.hs_info.timer_hs_data_stream,
        &lsm6ds3c_hs_disable);
  }
  if(NULL != state->esp_info.act_info.timer_inact_data_stream)
  {
    lsm6ds3c_handle_timer(instance, state->esp_info.act_info.timer_inact_data_stream,
        &lsm6ds3c_handle_inact_timer);
  }
  return rv;
}

void lsm6ds3c_handle_esp_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *src_regs)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)instance->state->state;
  uint8_t wake_src = 0;
  uint8_t func_src = 0;
  DBG_INST_PRINTF_EX(HIGH, instance, "ESP HW INT:sensors_e=0x%x ", state->esp_info.enabled_sensors);
  if(state->esp_info.enabled_sensors & LSM6DS3C_ESP_SENSORS_MASK) {
    wake_src = src_regs[0];
    func_src = src_regs[1];

    DBG_INST_PRINTF_EX(HIGH, instance, "wake_up_src=0x%x func_src=0x%x ", wake_src, func_src);

    if(state->esp_info.enabled_sensors & LSM6DS3C_FREE_FALL)
      lsm6ds3c_handle_ff_interrupt(instance, irq_timestamp, wake_src);
    if(state->esp_info.enabled_sensors & LSM6DS3C_HIGH_SHOCK)
      lsm6ds3c_handle_hs_interrupt(instance, irq_timestamp, wake_src);
    if(state->esp_info.enabled_sensors & (LSM6DS3C_ACTIVITY | LSM6DS3C_INACTIVITY))
      lsm6ds3c_handle_activity_interrupt(instance, irq_timestamp, wake_src);
    if(state->esp_info.enabled_sensors & LSM6DS3C_STEP_COUNTER)
      lsm6ds3c_handle_sc_interrupt(instance, irq_timestamp, func_src);
  }

}
void lsm6ds3c_update_esp_sensor_config(sns_sensor *this,
                              sns_sensor_instance *instance,
                              struct lsm6ds3c_instance_config *inst_cfg)
{
  lsm6ds3c_instance_state *inst_state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_request const *request;
  //if the request is not for ESP sensor, do not handle
  if(!LSM6DS3C_IS_ESP_BIT_SET(inst_cfg->config_sensors))
    return;

  uint8_t sensor_count = ARR_SIZE(lsm6ds3c_supported_esp_sensors);
  uint8_t i = 0;
  //reset desired_sensors
  //inst_state->esp_info.desired_sensors  = 0;
  //inst_state->md_info.internal_client_present = 0;
  //process queues whose config_sensors bit is set
  for(; i < sensor_count; i++) {
    if(inst_cfg->config_sensors & lsm6ds3c_supported_esp_sensors[i].sensor) {
      for(request = instance->cb->get_client_request(instance, &inst_state->esp_info.suid[i], true);
          NULL != request;
          request = instance->cb->get_client_request(instance, &inst_state->esp_info.suid[i], false))
      {
        DBG_PRINTF(HIGH, this,
            "req_id = %d",request->message_id );
        if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG) {
          LSM6DS3C_UPDATE_ESP_SENSOR_BIT(inst_state, lsm6ds3c_supported_esp_sensors[i].sensor, true);

          if((lsm6ds3c_supported_esp_sensors[i].sensor == LSM6DS3C_HIGH_SHOCK) ||
              (lsm6ds3c_supported_esp_sensors[i].sensor == LSM6DS3C_ACTIVITY) ||
              (lsm6ds3c_supported_esp_sensors[i].sensor == LSM6DS3C_INACTIVITY))
          {
            //MD internal client present
            inst_state->md_info.internal_client_present |= lsm6ds3c_supported_esp_sensors[i].sensor;
          }
          else if(lsm6ds3c_supported_esp_sensors[i].sensor == LSM6DS3C_STEP_COUNTER)
          {
            inst_state->esp_info.sc_info.client_present |= lsm6ds3c_supported_esp_sensors[i].sensor;
          }
          DBG_PRINTF(HIGH, this,
              "de_s = 0x%x sensor = 0x%x",inst_state->esp_info.desired_sensors, lsm6ds3c_supported_esp_sensors[i].sensor);
        }
      }
    }
  }

  lsm6ds3c_shared_state *shared_state = lsm6ds3c_get_shared_state(this);
  if(inst_state->esp_info.sc_info.client_present)
  {
    shared_state->esp_shared_info.stepCount_base = inst_state->esp_info.sc_info.stepCount_base + inst_state->esp_info.sc_info.stepCount_prev;
  }

  float esp_odr = lsm6ds3c_get_esp_rate(instance);
  DBG_PRINTF_EX(HIGH, this,
    "esp:(input SR/selcted SR) = %u, %u",(uint16_t)inst_cfg->sample_rate, (uint16_t)esp_odr);
  inst_cfg->sample_rate = SNS_MAX(inst_cfg->sample_rate, esp_odr);

  //keep if anything to update for special esp sensors
}

void lsm6ds3c_esp_send_fifo_flush_done(sns_sensor_instance *instance,
                                  lsm6ds3c_sensor_type flushing_sensors,
                                  lsm6ds3c_flush_done_reason reason)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)instance->state->state;
  flushing_sensors &= LSM6DS3C_ESP_SENSORS_MASK;
  while(flushing_sensors != 0)
  {
    sns_sensor_uid const *suid = NULL;
    lsm6ds3c_sensor_type sensor_type = LSM6DS3C_STEP_COUNTER;

    if(flushing_sensors & LSM6DS3C_FREE_FALL)
    {
      suid = &state->esp_info.suid[FF_INDX];
      sensor_type = LSM6DS3C_FREE_FALL;
    }
    else if(flushing_sensors & LSM6DS3C_HIGH_SHOCK)
    {
      suid = &state->esp_info.suid[HS_INDX];
      sensor_type = LSM6DS3C_HIGH_SHOCK;
    }
    else if(flushing_sensors & LSM6DS3C_STEP_COUNTER)
    {
      suid = &state->esp_info.suid[SC_INDX];
      sensor_type = LSM6DS3C_STEP_COUNTER;
    }

    if(NULL != suid)
    {
      sns_service_manager *mgr = instance->cb->get_service_manager(instance);
      sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
      sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
      event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
      event->event_len = 0;
      event->timestamp = sns_get_system_time();
      flushing_sensors &= ~sensor_type;
      e_service->api->publish_event(e_service, instance, event, suid);
      SNS_INST_PRINTF(HIGH, instance,
                      "FLUSH_EVENT esp sensor=%u (%u)", sensor_type, reason);
    }
  }
}

void lsm6ds3c_update_esp_request_q(
    sns_sensor               *const this,
    sns_sensor_instance      *instance,
    struct sns_request const *exist,
    bool remove)
{
  lsm6ds3c_state *state = (lsm6ds3c_state*)this->state->state;
  lsm6ds3c_instance_state *inst_state =
    (lsm6ds3c_instance_state*)instance->state->state;
  sns_request const *request;
  uint8_t idx = 0;

  if(state->sensor == LSM6DS3C_ACTIVITY)
    idx = ACT_INDX;
  else
    idx = INACT_INDX;
  if(!(inst_state->esp_info.desired_sensors & state->sensor)) {
    //clear the queue, if not present in state
    for(request = instance->cb->get_client_request(instance, &inst_state->esp_info.suid[idx], true);
        NULL != request;
        request = instance->cb->get_client_request(instance, &inst_state->esp_info.suid[idx], false))
    {
      if(request != exist)
        instance->cb->remove_client_request(instance, request);
    }
  } else if(remove) {
    inst_state->esp_info.desired_sensors &= ~state->sensor;
  }
}

bool lsm6ds3c_is_esp_request_present(sns_sensor_instance *instance)
{
  if(instance != NULL) {
    lsm6ds3c_instance_state *inst_state =
      (lsm6ds3c_instance_state*)instance->state->state;
    uint8_t sensor_count = ARR_SIZE(lsm6ds3c_supported_esp_sensors);
    int i = 0;
    for(i=0; i< sensor_count ; i++) {
      if(NULL != instance->cb->get_client_request(instance,
            &inst_state->esp_info.suid[i], true))
        break;
    }
    if(i >= sensor_count)
      return false;
  }
  return true;
}

void lsm6ds3c_store_esp_registry_data(sns_sensor_instance *instance, sns_sensor_state const *this)
{
  lsm6ds3c_instance_state *inst_state =
    (lsm6ds3c_instance_state*)instance->state->state;
  lsm6ds3c_shared_state *shared_state = lsm6ds3c_get_shared_state_from_state(this);
  sns_memscpy(&inst_state->esp_info.ff_info.config, sizeof(inst_state->esp_info.ff_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copying FF esp config");
  sns_memscpy(&inst_state->esp_info.hs_info.config, sizeof(inst_state->esp_info.hs_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng HS esp config");
  sns_memscpy(&inst_state->esp_info.act_info.config, sizeof(inst_state->esp_info.act_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.act_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.act_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng ACT esp config");
  //Add more based on the sensor type
}

sns_sensor_api lsm6ds3c_free_fall_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3c_free_fall_init,
  .deinit             = &lsm6ds3c_free_fall_deinit,
  .get_sensor_uid     = &lsm6ds3c_get_sensor_uid,
  .set_client_request = &lsm6ds3c_set_client_request,
  .notify_event       = &lsm6ds3c_sensor_notify_event,
};
sns_sensor_api lsm6ds3c_high_shock_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3c_high_shock_init,
  .deinit             = &lsm6ds3c_high_shock_deinit,
  .get_sensor_uid     = &lsm6ds3c_get_sensor_uid,
  .set_client_request = &lsm6ds3c_set_client_request,
  .notify_event       = &lsm6ds3c_sensor_notify_event,
};

sns_sensor_api lsm6ds3c_activity_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3c_activity_init,
  .deinit             = &lsm6ds3c_activity_deinit,
  .get_sensor_uid     = &lsm6ds3c_get_sensor_uid,
  .set_client_request = &lsm6ds3c_set_client_request,
  .notify_event       = &lsm6ds3c_sensor_notify_event,
};
sns_sensor_api lsm6ds3c_inactivity_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3c_inactivity_init,
  .deinit             = &lsm6ds3c_inactivity_deinit,
  .get_sensor_uid     = &lsm6ds3c_get_sensor_uid,
  .set_client_request = &lsm6ds3c_set_client_request,
  .notify_event       = &lsm6ds3c_sensor_notify_event,
};
sns_sensor_api lsm6ds3c_step_counter_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3c_step_counter_init,
  .deinit             = &lsm6ds3c_step_counter_deinit,
  .get_sensor_uid     = &lsm6ds3c_get_sensor_uid,
  .set_client_request = &lsm6ds3c_set_client_request,
  .notify_event       = &lsm6ds3c_sensor_notify_event,
};


#else

void lsm6ds3c_init_esp_instance(sns_sensor_instance *instance, sns_sensor_state const *this)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(this);
}

void lsm6ds3c_store_esp_registry_data(sns_sensor_instance *instance, sns_sensor_state const *this)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(this);
}

void lsm6ds3c_poweron_esp(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

void lsm6ds3c_powerdown_esp(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

uint16_t lsm6ds3c_get_esp_rate_idx(sns_sensor_instance *instance)
{
  UNUSED_VAR(instance);
  return LSM6DS3C_ACCEL_ODR_OFF;
}
sns_rc lsm6ds3c_handle_esp_timer_events(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
  return SNS_RC_SUCCESS;
}

void lsm6ds3c_handle_esp_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(irq_timestamp);
  UNUSED_VAR(wake_src);
}

void lsm6ds3c_update_pending_esp_intr(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

void lsm6ds3c_update_esp_sensor_config(sns_sensor *this,
                              sns_sensor_instance *instance,
                              struct lsm6ds3c_instance_config *inst_cfg)
{
  UNUSED_VAR(this);
  UNUSED_VAR(instance);
  UNUSED_VAR(inst_cfg);
}

bool lsm6ds3c_is_esp_request_present(sns_sensor_instance *instance)
{
  UNUSED_VAR(instance);
  return false;
}

#endif

