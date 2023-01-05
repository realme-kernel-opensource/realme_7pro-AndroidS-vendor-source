/**
 * @file sns_lsm6dso_ois_island.c
 *
 * Implementation for OIS sensor island code.
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
#include "sns_cal_util.h"
#include <string.h>
#include "sns_math_util.h"
#include "sns_mem_util.h"

#include "sns_event_service.h"
#include "sns_diag_service.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_ois.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_stream_service.h"
#include "sns_types.h"

#include "sns_motion_detect.pb.h"
#include "sns_pb_util.h"
#include "sns_printf.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_diag.pb.h"
#include "sns_cal.pb.h"

#if LSM6DSO_OIS_ENABLED
#include "sns_timer.pb.h"
//forwared declaration

extern sns_rc lsm6dso_sensor_notify_event(sns_sensor *const this);
/**
 * LSM6DSO Full Scales in register setting for OIS
 */
typedef enum
{
  STM_LSM6DSO_OIS_RANGE_125DPS   = 0x02,
  STM_LSM6DSO_OIS_RANGE_245DPS   = 0x00,  /*corresponding value in register setting*/
  STM_LSM6DSO_OIS_RANGE_500DPS   = 0x04,
  STM_LSM6DSO_OIS_RANGE_1000DPS  = 0x08,
  STM_LSM6DSO_OIS_RANGE_2000DPS  = 0x0C,
} lsm6dso_ois_range;


const lsm6dso_ois_range lsm6dso_ois_ranges[] =
{
  STM_LSM6DSO_OIS_RANGE_125DPS,
  STM_LSM6DSO_OIS_RANGE_245DPS,  /*corresponding value in register setting*/
  STM_LSM6DSO_OIS_RANGE_500DPS,
  STM_LSM6DSO_OIS_RANGE_1000DPS,
  STM_LSM6DSO_OIS_RANGE_2000DPS
};

const float lsm6dso_ois_resolutions[] =
{
  LSM6DSO_GYRO_SSTVT_125DPS,
  LSM6DSO_GYRO_SSTVT_245DPS,
  LSM6DSO_GYRO_SSTVT_500DPS,
  LSM6DSO_GYRO_SSTVT_1000DPS,
  LSM6DSO_GYRO_SSTVT_2000DPS
};  //mdps/LSB

#if 0
uint16_t lsm6dso_get_ois_rate_idx(sns_sensor_instance *instance)
{
  float ois_sample_rate = 0;
  lsm6dso_accel_odr accel_chosen_sample_rate_reg_value = LSM6DSO_ACCEL_ODR_OFF;

  ois_sample_rate = lsm6dso_get_ois_rate(instance);
  accel_chosen_sample_rate_reg_value = lsm6dso_get_odr_rate_idx(ois_sample_rate);
  return accel_chosen_sample_rate_reg_value;
}
#endif
void lsm6dso_set_ois_polling_config(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  uint8_t buffer[50];
  sns_request timer_req = {
    .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
    .request    = buffer
  };

  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = state->ois_info.sampling_intvl_ticks;

  timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                                            sns_timer_sensor_config_fields, NULL);
  if(timer_req.request_len > 0)
  {
    if(state->timer_sensor_ois_data_stream == NULL)
    {
      sns_service_manager *service_mgr = this->cb->get_service_manager(this);
      sns_stream_service *stream_mgr = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
          this,
          state->timer_suid,
          &state->timer_sensor_ois_data_stream);
      if(state->timer_sensor_ois_data_stream == NULL)
      {
        SNS_INST_PRINTF(ERROR, this, "timer_sensor_ois_data_stream NULL");
        return;
      }
    }
    DBG_INST_PRINTF(HIGH, this, "setting ois timer: rate(*1000)=%d period=%u",
                    (int)(state->ois_info.sampling_rate_hz*1000),
                    (uint32_t)req_payload.timeout_period);
    state->timer_sensor_ois_data_stream->api->
      send_request(state->timer_sensor_ois_data_stream, &timer_req);
  }
  else
  {
    //SNS_INST_PRINTF(LOW, this,
    //                         "encode err");
  }
}
void lsm6dso_set_ois_config(sns_sensor_instance *const instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  DBG_INST_PRINTF_EX(LOW, instance, "lsm6dso_set_ois_config called enable =%d", enable );
  if(enable) {
    rw_buffer = 0x01;
    lsm6dso_read_modify_write(instance,
        STM_LSM6DSO_REG_FUNC_CFG,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x01);
  }
  //set FS range and set OIS_EN_SPI2
  rw_buffer = lsm6dso_ois_ranges[state->ois_info.resolution_idx] |
              (enable) ? 0x01 : 0;
  lsm6dso_read_modify_write(instance,
      STM_LSM6DSO_REG_UI_CTRL1_OIS,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      0x0F);

  if(!state->ois_info.mode_polling){
      //enable ois interrupt
	  rw_buffer = (enable) ? 0x80 : 0;
      lsm6dso_read_modify_write(instance,
          STM_LSM6DSO_REG_UI_INT_OIS,
          &rw_buffer,
          1,
          &xfer_bytes,
          false,
          0x80);
  }
  else{
    //set polling timer
    lsm6dso_set_ois_polling_config(instance);
  }
  state->ois_info.enabled = enable;
}

void lsm6dso_config_ois(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  LSM6DSO_INST_DEBUG_TS(LOW, instance, "config_ois %d %d",state->ois_info.desired, state->ois_info.enabled );
  state->ois_info.sampling_intvl_ticks = sns_convert_ns_to_ticks(1000000000.0 / (state->ois_info.mode_polling ? state->ois_info.sampling_rate_hz : LSM6DSO_OIS_RATE));

  if(state->ois_info.desired && !state->ois_info.enabled) {
    lsm6dso_set_ois_config(instance, true);
  }
  else if(!state->ois_info.desired && state->ois_info.enabled) {
    lsm6dso_set_ois_config(instance, false);
  }
}

sns_rc lsm6dso_handle_ois_polling_timer_events(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(latest_timer_event);
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  DBG_INST_PRINTF_EX(LOW, this, "lsm6dso_handle_ois_polling_timer_events called");
  if(state->ois_info.enabled)
  {
    lsm6dso_handle_ois_interrupt(this, timestamp);
  }
  else
  {
    sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_ois_data_stream);
  }
  return rv;
}

static bool lsm6dso_check_ois_interrupt(
    lsm6dso_instance_state const *state,
    sns_std_sensor_sample_status* status)
{
  uint8_t rw_buffer = 0;

  uint32_t xfer_bytes;
  lsm6dso_com_read_wrapper(state->scp_service,
      state->com_port_info.port_handle,
      STM_LSM6DSO_REG_UI_STATUS_OIS,
      &rw_buffer,
      1,
      &xfer_bytes);
  *status = (rw_buffer & 0x04) ?
    SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE :
    SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  return (rw_buffer & 0x02);
}

static void lsm6dso_handle_ois_sample(sns_sensor_instance *const instance,
                                      const uint8_t fifo_sample[6],
                                      sns_time timestamp,
                                      sns_time filter_delay,
                                      sns_std_sensor_sample_status status)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_diag_service* diag = state->diag_service;
  log_sensor_state_raw_info log_ois_state_raw_info;

  sns_memzero(&log_ois_state_raw_info, sizeof(log_ois_state_raw_info));
  log_ois_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  log_ois_state_raw_info.diag = diag;
  log_ois_state_raw_info.instance = instance;
  log_ois_state_raw_info.sensor_uid = &state->ois_info.suid;
  log_ois_state_raw_info.log = NULL;

  lsm6dso_log_sensor_state_raw_alloc(&log_ois_state_raw_info, 0);
  const uint8_t* ip_raw = &fifo_sample[0];

  //state->gyro_info.opdata_status = status;

  vector3 opdata_cal;

  float ipdata[TRIAXIS_NUM] = {0};
  float opdata_raw[TRIAXIS_NUM] = {0};
  float sstvt = lsm6dso_ois_resolutions[state->ois_info.resolution_idx];
  uint8_t i;

  for(i = 0; i < ARR_SIZE(ipdata); i++)
  {
    uint8_t j = i << 1;
    ipdata[i] =
      (int16_t)(((ip_raw[j+1] << 8) & 0xFF00) | ip_raw[j]) *
      ((sstvt *  PI) / 180.0f);
  }

  for(i = 0; i < ARR_SIZE(opdata_raw); i++)
  {
    opdata_raw[state->axis_map[i].opaxis] =
      (state->axis_map[i].invert ? -1.0 : 1.0) * ipdata[state->axis_map[i].ipaxis];
  }

  // factory calibration
  opdata_cal = sns_apply_calibration_correction_3(
      make_vector3_from_array(opdata_raw),
      make_vector3_from_array(state->gyro_registry_cfg.fac_cal_bias),
      state->gyro_registry_cfg.fac_cal_corr_mat);

  pb_send_sensor_stream_event(instance,
      &state->ois_info.suid,
      timestamp - filter_delay,
      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
      status,
      opdata_cal.data,
      ARR_SIZE(opdata_cal.data),
      state->encoded_imu_event_len);
  LSM6DSO_INST_DEBUG_TS(LOW, instance, "OIS  timestamp = %u cur_time = %u", (uint32_t)timestamp, (uint32_t)sns_get_system_time());

  // Log raw uncalibrated sensor data
  lsm6dso_log_sensor_state_raw_add(
      &log_ois_state_raw_info,
      opdata_raw,
      LSM6DSO_NUM_AXES,
      timestamp - filter_delay,
      status);
  lsm6dso_log_sensor_state_raw_submit(&log_ois_state_raw_info, LSM6DSO_NUM_AXES, true);
}


void lsm6dso_handle_ois_interrupt(sns_sensor_instance *const instance,
    sns_time irq_timestamp)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t fifo_data[6];

  sns_std_sensor_sample_status status;
  if(lsm6dso_check_ois_interrupt(state, &status))
  {
    sns_rc rc = lsm6dso_read_regs_scp(instance,  STM_LSM6DSO_REG_UI_OUTX_L_G_OIS, LSM6DSO_OIS_DATA_SIZE, fifo_data);
    if(rc == SNS_RC_SUCCESS) {
      lsm6dso_handle_ois_sample(instance,
          fifo_data,
          irq_timestamp,
          state->ois_info.sampling_intvl_ticks,
          status);

    }
  }
}
#if 0
//implement heart attack for ois
sns_rc lsm6dso_handle_ois_timer_events(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_sensor_event *event;
  sns_rc rv = SNS_RC_SUCCESS;


  if(NULL != state->ois_info.timer_ois_data_stream)
  {
    event = state->ois_info.timer_ois_data_stream->api->peek_input(state->ois_info.timer_ois_data_stream);
    while(NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
          event->event_len);
      sns_timer_sensor_event timer_event;
      if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
        {
          DBG_INST_PRINTF(LOW, instance,
                       "OIS Heart attack  detected .. handle it");
          lsm6dso_handle_ois_heart_attack(instance,sns_get_system_time());
        }
      }
      else
      {
      }
      if(NULL != state->ois_info.timer_ois_data_stream)
      {
        event = state->ois_info.timer_ois_data_stream->api->get_next_input(state->ois_info.timer_ois_data_stream);
      }
      else
      {
        event = NULL;
        break;
      }
    }
  }
  return rv;
}
#endif
void lsm6dso_update_ois_sensor_config(sns_sensor *this,
                              sns_sensor_instance *instance)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_request const *request;

  //reset desired_sensors
  inst_state->ois_info.desired  = false;
  for(request = instance->cb->get_client_request(instance, &inst_state->ois_info.suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &inst_state->ois_info.suid, false))
  {
    DBG_PRINTF(HIGH, this,
        "req_id = %d",request->message_id );
    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
      inst_state->ois_info.desired  = true;
      DBG_PRINTF(HIGH, this,
          "de_s = 0x%x ",inst_state->ois_info.desired);
      break;
    }
  }
}

bool lsm6dso_is_ois_request_present(sns_sensor_instance *instance)
{
  if(instance != NULL) {
    lsm6dso_instance_state *inst_state =
      (lsm6dso_instance_state*)instance->state->state;
    return instance->cb->get_client_request(instance,
        &inst_state->ois_info.suid, true);
  }
  return false;
}


void lsm6dso_store_ois_registry_data(sns_sensor_instance *instance, sns_sensor_state const *this)
{
  UNUSED_VAR(this);
  UNUSED_VAR(instance);
#if 0
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state_from_state(this);
  //store ois calibration data
  sns_memscpy(&inst_state->ois_info.cal, sizeof(inst_state->ois_info.cal),
      &shared_state->inst_cfg.ois_reg_cfg.ois_cal, sizeof(shared_state->inst_cfg.ois_reg_cfg.ois_cal));
#endif
  DBG_INST_PRINTF(HIGH, instance,
      "copying ois calibration data");
}
void lsm6dso_init_ois_instance(sns_sensor_instance *instance,
    lsm6dso_instance_config const *inst_cfg)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

  inst_state->ois_info.enabled = false;
  inst_state->ois_info.desired = false;
  inst_state->ois_info.resolution_idx = inst_cfg->ois_reg_cfg.resolution_idx;
  DBG_INST_PRINTF(HIGH, instance,
      "ois_resolution_idx = %d", inst_state->ois_info.resolution_idx);
  //TODO:: can use only gyro calibration data ?
  //lsm6dso_copy_calibration_info(&inst_state->ois_info.registry_cfg, &inst_cfg->ois_reg_cfg.ois_cal);

  //Override default mode and rate from registry read
  inst_state->ois_info.sampling_rate_hz = LSM6DSO_OIS_RATE;
  inst_state->ois_info.mode_polling = (bool)LSM6DSO_OIS_POLLING;
  DBG_INST_PRINTF(HIGH, instance,
      "ois sampling rate = %d, polling mode = %d", (int)(inst_state->ois_info.sampling_rate_hz*1000), inst_state->ois_info.mode_polling);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->ois_info.suid,
        sizeof(inst_state->ois_info.suid),
        &((sns_sensor_uid)OIS_SUID_1),
        sizeof(inst_state->ois_info.suid));
  }
  else
#endif
  {
    sns_memscpy(&inst_state->ois_info.suid,
        sizeof(inst_state->ois_info.suid),
        &((sns_sensor_uid)OIS_SUID_0),
        sizeof(inst_state->ois_info.suid));
  }
}

sns_sensor_api lsm6dso_ois_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_ois_init,
  .deinit             = &lsm6dso_ois_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_client_request,
  .notify_event       = &lsm6dso_sensor_notify_event,
};

#endif
