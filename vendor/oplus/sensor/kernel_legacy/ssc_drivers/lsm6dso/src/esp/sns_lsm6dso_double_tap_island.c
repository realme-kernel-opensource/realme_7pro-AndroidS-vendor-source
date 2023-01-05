/**
 * @file sns_lsm6dso_double_tap_island.c
 *
 * Common implementation for LSM6DSO Double Tap Sensor
 *
 * Copyright (c) 2019, STMicroelectronics.
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

#if LSM6DSO_ESP_DOUBLE_TAP
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_printf.h"

#include "sns_diag.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_double_tap.pb.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"

/** double tap configuration */
#define LSM6DSO_DTP_THRESH_MAX      (lsm6dso_accel_range_max[state->accel_info.range_idx] * 1000) //mg
#define LSM6DSO_DTP_THRESH_RES      (32.0f) //2^5

//bit[3,2.1] for tap x/y/z enable
#define STM_LSM6DSO_REG_TAP_EN_MASK       (0x02) //tap_z_en only
#if LSM6DSO_EX_TAP_TUNING_ENABLED
#define LSM6DSO_IS_DOUBLE_TAP_INT(data) (data & 0x20)
#else
#define LSM6DSO_IS_DOUBLE_TAP_INT(data) (data & 0x10)
#endif
extern sns_rc lsm6dso_double_tap_init(sns_sensor *const this);
extern sns_rc lsm6dso_double_tap_deinit(sns_sensor *const this);
extern void lsm6dso_process_dtp_registry_event(sns_sensor *const this, sns_sensor_event *event);

void lsm6dso_send_double_tap_event(sns_sensor_instance *const instance, lsm6dso_sensor_type sensor, void* event)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  pb_field_t const *event_fields;
  struct sns_sensor_uid* uid;
  uint32_t msg_id;
  if(sensor == LSM6DSO_DOUBLE_TAP) {
    event_fields = &sns_double_tap_event_fields[0];
    uid = &state->esp_info.suid[DTP_INDX];
    msg_id = SNS_DOUBLE_TAP_MSGID_SNS_DOUBLE_TAP_EVENT;
    DBG_INST_PRINTF(LOW, instance, "[ESP Event] sensor :0x%x event :%d",
      sensor, ((sns_double_tap_event *)(event))->double_tap_event_type);

    pb_send_event(instance,
        event_fields,
        event,
        sns_get_system_time(),
        msg_id,
        uid);
  }
}

void lsm6dso_enable_double_tap(sns_sensor_instance *const instance, bool send_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  float thresh_lsb;
  uint8_t thresh_x, thresh_y, thresh_z;

  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  lsm6dso_accel_odr esp_desired_odr = (lsm6dso_accel_odr)lsm6dso_get_esp_rate_idx(instance);
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  float accel_odr_value = lsm6dso_get_accel_odr(accel_odr);
  float esp_desired_odr_value = lsm6dso_get_accel_odr(esp_desired_odr);
#endif
  if(state->esp_info.enabled_sensors & LSM6DSO_DOUBLE_TAP) {
    return;
  }
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  if(accel_odr_value < esp_desired_odr_value) {
    accel_odr = esp_desired_odr;
    accel_odr_value = esp_desired_odr_value;
  }
  if(state->accel_info.desired_odr == LSM6DSO_ACCEL_ODR832)
    accel_odr_value = esp_desired_odr_value;
#else
  if(accel_odr < esp_desired_odr) {
    accel_odr = esp_desired_odr;
  }
#endif
  DBG_INST_PRINTF_EX(HIGH, instance, "enable double_tap: accel_ds=0x%x/0x%x",
    state->accel_info.desired_odr, accel_odr);

  if((accel_odr != state->common_info.accel_curr_odr) &&
     (!state->fifo_info.reconfig_req)){
    lsm6dso_set_accel_config(instance,
        accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
  }

  //enable double tap
  rw_buffer = STM_LSM6DSO_REG_TAP_EN_MASK | 0x40;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG0,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           (STM_LSM6DSO_REG_TAP_EN_MASK | 0x40));

  thresh_lsb = LSM6DSO_DTP_THRESH_MAX/LSM6DSO_DTP_THRESH_RES;
  thresh_x = (uint8_t)(state->esp_info.dtp_info.config.thresh_x/thresh_lsb);
  thresh_y = (uint8_t)(state->esp_info.dtp_info.config.thresh_y/thresh_lsb);
  thresh_z = (uint8_t)(state->esp_info.dtp_info.config.thresh_z/thresh_lsb);
  if(thresh_x == 0) thresh_x = 1;
  if(thresh_y == 0) thresh_y = 1;
  if(thresh_z == 0) thresh_z = 1;
  DBG_INST_PRINTF_EX(HIGH, instance, "FS_idx=%u thresh_max*1000=%d thresh_lsb*1000=%d",
    state->accel_info.range_idx, (int)(LSM6DSO_DTP_THRESH_MAX*1000), (int)(thresh_lsb*1000));
  DBG_INST_PRINTF_EX(HIGH, instance, "thresh_x/y/z=0x%x/0x%x/0x%x",
    thresh_x, thresh_y, thresh_z);
  rw_buffer = thresh_x | (state->esp_info.dtp_info.config.priority << 5);
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG1,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0xFF);

  rw_buffer = 0x80 | thresh_y;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG2,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x9F);

  rw_buffer = thresh_z;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_THS_6D,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x1F);
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  uint32_t intvl = sns_convert_ns_to_ticks(1000000000.0f / accel_odr_value);
  uint8_t dur = (state->esp_info.dtp_info.config.dur_sw == 0) ? 16 : (state->esp_info.dtp_info.config.dur_sw * 32);
  uint8_t quiet = (state->esp_info.dtp_info.config.quiet_sw == 0) ? 2 : (state->esp_info.dtp_info.config.quiet_sw * 4);
  state->esp_info.dtp_info.dur = dur * intvl;
  state->esp_info.dtp_info.quiet = quiet * intvl;

  DBG_INST_PRINTF_EX(HIGH, instance, "+++dur=%u", (uint32_t)state->esp_info.dtp_info.dur);
#endif
  rw_buffer = (state->esp_info.dtp_info.config.dur << 4) | 
              (state->esp_info.dtp_info.config.quiet << 2) |
              state->esp_info.dtp_info.config.shock;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_DUR,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0xFF);

  rw_buffer = 0x80;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_WAKE_THS,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x80);
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  rw_buffer = 0x40;
#else
  rw_buffer = 0x08;
#endif
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           rw_buffer);

  DBG_INST_PRINTF_EX(HIGH, instance, "+++send_event=%d config_sensor=0x%x",
    send_event, state->config_sensors);
  //Enable concurency with streaming
  uint8_t reg_addr = 0;
  if(state->esp_info.dtp_info.who_am_i_val == LSM6DSO_WHOAMI_VALUE)
  {
    reg_addr = STM_LSM6DSO_REG_I3C_BUS_AVB;
    rw_buffer = 0x40;
  }
  else if(state->esp_info.dtp_info.who_am_i_val == LSM6DST_WHOAMI_VALUE)
  {
    reg_addr = STM_LSM6DSO_REG_CNT_BDR1;
    rw_buffer = 0x08;
  }
  lsm6dso_read_modify_write(instance,
           reg_addr,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           rw_buffer);

  if(send_event && state->config_sensors & LSM6DSO_DOUBLE_TAP) {
    sns_double_tap_event event;
    event.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_ENABLED;
    lsm6dso_send_double_tap_event(instance, LSM6DSO_DOUBLE_TAP, &event);
  }
  state->esp_info.enabled_sensors |= LSM6DSO_DOUBLE_TAP;
  lsm6dso_dump_reg(instance, state->fifo_info.fifo_enabled);
}

void lsm6dso_disable_double_tap(sns_sensor_instance *const instance, bool send_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  rw_buffer = 0x0;
#if LSM6DSO_EX_TAP_TUNING_ENABLED
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x40);
#else
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x08);
#endif
  rw_buffer = 0x0;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG0,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_TAP_EN_MASK);

  rw_buffer = 0x0;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG1,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0xFF);

  rw_buffer = 0x0;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_CFG2,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x1F);

  rw_buffer = 0x0;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_THS_6D,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x1F);

  rw_buffer = 0x6;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_TAP_DUR,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0xFF);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "double_tap disabled");
  if(!state->fifo_info.reconfig_req) {
    lsm6dso_set_accel_config(instance,
          state->accel_info.desired_odr,
          state->accel_info.sstvt,
          state->accel_info.range,
          LSM6DSO_ACCEL_BW50);
  }

  if(send_event && (state->config_sensors & LSM6DSO_DOUBLE_TAP)) {
    sns_double_tap_event event;
    event.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_DISABLED;
    lsm6dso_send_double_tap_event(instance, LSM6DSO_DOUBLE_TAP, &event);
  }
  state->esp_info.enabled_sensors &= ~LSM6DSO_DOUBLE_TAP;
}

void lsm6dso_handle_double_tap_intr(
    sns_sensor_instance *const instance,
    sns_time irq_timestamp,
    uint8_t src_reg,
    uint8_t tap_data)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
#if !LSM6DSO_EX_TAP_TUNING_ENABLED
  UNUSED_VAR(irq_timestamp);
#endif
  DBG_INST_PRINTF_EX(HIGH, instance, "double_tap_intr: 0x%x/0x%x 0x%x",
    src_reg, tap_data, state->esp_info.enabled_sensors);
  if(src_reg != STM_LSM6DSO_REG_TAP_SRC)
    return;

  if(state->esp_info.enabled_sensors & LSM6DSO_DOUBLE_TAP)
  {
    if(LSM6DSO_IS_DOUBLE_TAP_INT(tap_data))
    {
#if LSM6DSO_EX_TAP_TUNING_ENABLED
      sns_time delta = irq_timestamp - state->esp_info.dtp_info.pre_dtp_ts;
      DBG_INST_PRINTF_EX(HIGH, instance, "+++irq_ts/pre_ts/dur=%u/%u/%u/%u quiet=%u peak=%d",
        (uint32_t)irq_timestamp, (uint32_t)state->esp_info.dtp_info.pre_dtp_ts,
        (uint32_t)delta, (uint32_t)state->esp_info.dtp_info.dur, (uint32_t)state->esp_info.dtp_info.quiet, state->esp_info.dtp_info.single_tap_peak);
      if(state->esp_info.dtp_info.quiet < delta &&
         (state->esp_info.dtp_info.single_tap_peak < 2) &&
         delta < state->esp_info.dtp_info.dur)
      {
        sns_double_tap_event dtp_state;
        dtp_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_FIRED;

        lsm6dso_send_double_tap_event(instance, LSM6DSO_DOUBLE_TAP, &dtp_state);
        DBG_INST_PRINTF_EX(HIGH, instance, "double_tap fired! %u/0x%x",
          ++state->esp_info.dtp_info.dtp_cnt, tap_data);
        state->esp_info.dtp_info.pre_dtp_ts = 0;
        state->esp_info.dtp_info.single_tap_peak = 0;
      }
      else if(delta > state->esp_info.dtp_info.dur)
      {
          DBG_INST_PRINTF_EX(HIGH, instance, "+++long dur, single tap detected!");
          state->esp_info.dtp_info.pre_dtp_ts = irq_timestamp;
          state->esp_info.dtp_info.single_tap_peak = 1;
      }
      else
      {
        state->esp_info.dtp_info.single_tap_peak++;
        DBG_INST_PRINTF_EX(HIGH, instance, "+++short dur, in quiet period!");
      }
#else
      sns_double_tap_event dtp_state;
      dtp_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_FIRED;

      lsm6dso_send_double_tap_event(instance, LSM6DSO_DOUBLE_TAP, &dtp_state);
      DBG_INST_PRINTF_EX(HIGH, instance, "double_tap fired! %u/0x%x",
        ++state->esp_info.dtp_info.dtp_cnt, tap_data);
#endif
    }
  }
}

void lsm6dso_update_double_tap_request_q(
    sns_sensor               *const this,
    sns_sensor_instance      *instance,
    struct sns_request const *exist,
    bool remove)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

  if(remove)
  {
    inst_state->esp_info.desired_sensors &= ~state->sensor;
    if(NULL != exist)
    {
      instance->cb->remove_client_request(instance, exist);
    }
  }
}

void lsm6dso_reconfig_double_tap(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(enable); 

  DBG_INST_PRINTF(LOW, instance, "reconfig_st d/e:0x%x/0x%x",
    state->esp_info.desired_sensors, state->esp_info.enabled_sensors);

  if(state->esp_info.desired_sensors & LSM6DSO_DOUBLE_TAP)
  {
    lsm6dso_enable_double_tap(instance, true);
  } else if(state->esp_info.enabled_sensors & LSM6DSO_DOUBLE_TAP) {
    lsm6dso_disable_double_tap(instance, true);
  }

}

void lsm6dso_fill_double_tap_inst_info(sns_sensor *const this, sns_sensor_instance *instance)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  float data[1];
  //to make sure not to overwrite
  if(inst_state->esp_info.dtp_info.encoded_event_len)
    return;
#if LSM6DSO_LOG_VERBOSE_EX
  inst_state->esp_info.dtp_info.dtp_cnt = 0;
#endif
  inst_state->esp_info.dtp_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);

#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[DTP_INDX],
        sizeof(inst_state->esp_info.suid[DTP_INDX]),
        &((sns_sensor_uid)DOUBLE_TAP_SUID_1),
        sizeof(inst_state->esp_info.suid[DTP_INDX]));
  } else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[DTP_INDX],
        sizeof(inst_state->esp_info.suid[DTP_INDX]),
        &((sns_sensor_uid)DOUBLE_TAP_SUID_0),
        sizeof(inst_state->esp_info.suid[DTP_INDX]));

  }
  sns_memscpy(&inst_state->esp_info.dtp_info.config, sizeof(inst_state->esp_info.dtp_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng ST esp config");
          DBG_PRINTF_EX(HIGH, this,
              "double_tap thresh x/y/z(*1000):%d/%d/%d m/s2 priority=%u dur=%u quiet=%u dur_sw=%u quiet_sw=%u shock=%u",
              (int)(shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.thresh_x*1000),
              (int)(shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.thresh_y*1000),
              (int)(shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.thresh_z*1000),
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.priority,
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.dur,
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.quiet,
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.dur_sw,
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.quiet_sw,
              shared_state->inst_cfg.esp_reg_cfg.dtp_reg_conf.shock);
}

sns_sensor_instance* lsm6dso_set_double_tap_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;

  SNS_PRINTF(ERROR, this, "set_double_tap_req: instance=%p sensor=0x%x remove=%d",
    instance, state->sensor, remove);
  if(!remove && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id)) {
    SNS_PRINTF(ERROR, this, "DTP req rejec, msg=%d", new_request->message_id);
    return instance;
  }

  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);

  if(instance) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    lsm6dso_fill_double_tap_inst_info(this, instance);

    inst_state->esp_info.desired_sensors &= ~state->sensor;

    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[DTP_INDX], true))
      inst_state->esp_info.desired_sensors |= state->sensor;

    inst_state->esp_info.dtp_info.client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    if(remove || ((inst_state->esp_info.enabled_sensors & state->sensor) != (inst_state->esp_info.desired_sensors & state->sensor)))
      instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}

sns_rc lsm6dso_sensor_notify_double_tap_event(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_data_stream *stream = state->reg_data_stream;

  if(NULL != stream && 0 != stream->api->get_input_cnt(stream))
  {
    DBG_PRINTF_EX(HIGH, this, "registry_event: sensor=%u stream=%x", state->sensor, stream);
    for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
    {
      sns_sensor_event *event = stream->api->peek_input(stream);
      lsm6dso_process_dtp_registry_event(this, event);
    }
  }
  return lsm6dso_sensor_notify_event(this);
}


sns_sensor_api lsm6dso_double_tap_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_double_tap_init,
  .deinit             = &lsm6dso_double_tap_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_double_tap_request,
  .notify_event       = &lsm6dso_sensor_notify_double_tap_event,
};

#endif //!LSM6DSO_ESP_DOUBLE_TAP
