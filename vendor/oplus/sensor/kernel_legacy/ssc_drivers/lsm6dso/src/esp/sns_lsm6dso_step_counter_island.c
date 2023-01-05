/**
 * @file sns_lsm6dso_step_counter_island.c
 *
 * Common implementation for LSM6DSO Step Counter Sensors.
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

#if LSM6DSO_ESP_STEP_COUNTER
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
#include "sns_lsm6dso_log_pckts.h"

extern const odr_reg_map lsm6dso_odr_map[];
extern const uint32_t lsm6dso_odr_map_len;

extern sns_rc lsm6dso_step_counter_init(sns_sensor *const this);
extern sns_rc lsm6dso_step_counter_deinit(sns_sensor *const this);
extern sns_rc lsm6dso_sensor_notify_event(sns_sensor *const this);

#define LSM6DSO_IS_STEP_COUNTER_INT(data) (data & 0x10)

sns_rc read_step_count(sns_sensor_instance *const this, uint16_t *step_count)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)this->state->state;
  uint8_t sc_data[2] = {0};
  uint32_t xfer_bytes;

  lsm6dso_emb_cfg_access(this, true);
  lsm6dso_com_read_wrapper(state->scp_service,
           state->com_port_info.port_handle,
           STM_LSM6DSO_STEP_COUNTER_L,
           &sc_data[0],
           2,
           &xfer_bytes);
  if(xfer_bytes != 2)
  {
    lsm6dso_emb_cfg_access(this, false);
    return SNS_RC_FAILED;
  }

  *step_count = (uint16_t)(sc_data[1] << 8 | sc_data[0]);
  lsm6dso_emb_cfg_access(this, false);
  return SNS_RC_SUCCESS;
}

void lsm6dso_sc_report_steps(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint16_t stepCount_cur;
  float float_sc_val;

  if(read_step_count(instance, &stepCount_cur) != SNS_RC_SUCCESS)
    return;

  float_sc_val = (float)stepCount_cur;
  DBG_INST_PRINTF(HIGH, instance,
      "sc_enabled: sc_val %u", stepCount_cur);

  pb_send_sensor_stream_event(instance,
                              &state->esp_info.suid[SC_INDX],
                              sns_get_system_time(),
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                              &float_sc_val,
                              1,
                              state->esp_info.sc_info.encoded_event_len);
}

static void lsm6dso_sc_enable(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  if(state->esp_info.enabled_sensors & LSM6DSO_STEP_COUNTER) {
    return;
  }

  if(!state->irq_info.irq_ready)
  {
    state->esp_info.update_int |= LSM6DSO_STEP_COUNTER;
    DBG_INST_PRINTF(LOW, instance,
                   "sc_enable: IRQ not ready");
    return;
  }

  if(accel_odr == LSM6DSO_ACCEL_ODR_OFF) {
    accel_odr = (lsm6dso_accel_odr)lsm6dso_get_esp_rate_idx(instance);
  }

  DBG_INST_PRINTF_EX(HIGH, instance,
      "step_counter enabled");
  if((accel_odr != state->common_info.accel_curr_odr) &&
     (!state->fifo_info.reconfig_req)){
    lsm6dso_set_accel_config(instance,
        (state->accel_info.desired_odr) ? state->accel_info.desired_odr : accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
  }

  //routing of embedded functions event on INT1
  rw_buffer = STM_LSM6DSO_REG_INT1_EMB_FUNC;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_INT1_EMB_FUNC_MASK);

  //embedded functions cfg en
  lsm6dso_emb_cfg_access(instance, true);
  //enable pedometer algorithm
  rw_buffer = STM_LSM6DSO_REG_PEDO_EN;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_EN_A,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_PEDO_EN_MASK);
  //routing of pedometer step recognition event on INT1
  rw_buffer = STM_LSM6DSO_REG_INT1_STEP_DETECTOR;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_INT1,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_INT1_STEP_DETECTOR_MASK);
  //pedometer step counter/detector algorithm initialization request
  rw_buffer = STM_LSM6DSO_REG_STEP_DET_INIT;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_INIT_A,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_STEP_DET_INIT_MASK);
  //embedded functions cfg off
  lsm6dso_emb_cfg_access(instance, false);

  state->esp_info.enabled_sensors |= LSM6DSO_STEP_COUNTER;

#if LSM6DSO_OEM_FACTORY_CONFIG
  lsm6dso_sc_report_steps(instance);
#endif

  DBG_INST_PRINTF(HIGH, instance,
      "a_odr=0x%x cur_odr(a/g)=0x%x/0x%x enabled_sensors 0x%x",
      accel_odr, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr, state->esp_info.enabled_sensors);
}

void lsm6dso_sc_disable(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  //embedded functions cfg en
  lsm6dso_emb_cfg_access(instance, true);
  //turn off routing of pedometer step recognition event on INT1
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_INT1,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_INT1_STEP_DETECTOR_MASK);
  //disable pedometer algorithm
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_EN_A,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_PEDO_EN_MASK);
  //pedometer step counter/detector algorithm initialization request
  rw_buffer = STM_LSM6DSO_REG_STEP_DET_INIT;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_EMB_FUNC_INIT_A,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_STEP_DET_INIT_MASK);
  //embedded functions cfg off
  lsm6dso_emb_cfg_access(instance, false);

  //disable routing of embedded functions event on INT1
  rw_buffer = 0;
  lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_INT1_EMB_FUNC_MASK);

  DBG_INST_PRINTF_EX(HIGH, instance,
      "step_counter disabled");
  if(!state->fifo_info.reconfig_req) {
    lsm6dso_set_accel_config(instance,
          state->accel_info.desired_odr,
          state->accel_info.sstvt,
          state->accel_info.range,
          LSM6DSO_ACCEL_BW50);
  }
  state->esp_info.enabled_sensors &= ~LSM6DSO_STEP_COUNTER;
}

static void lsm6dso_convert_and_send_sc_sample(sns_sensor_instance *const instance,
                                   sns_time timestamp,
                                   const uint16_t stepCount_cur)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  if(state->esp_info.enabled_sensors & LSM6DSO_STEP_COUNTER)
  {
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    if(state->esp_info.sc_info.config.stepCount_prev == stepCount_cur)
    {
      return;
    }
    else if(state->esp_info.sc_info.config.stepCount_prev > stepCount_cur)
    {
      state->esp_info.sc_info.config.stepCount_base += state->esp_info.sc_info.config.stepCount_prev;
    }
    state->esp_info.sc_info.config.stepCount_prev = stepCount_cur;
    float float_sc_val = (float)(state->esp_info.sc_info.config.stepCount_base + state->esp_info.sc_info.config.stepCount_prev);

    log_sensor_state_raw_info log_sc_state_raw_info;

    sns_memzero(&log_sc_state_raw_info, sizeof(log_sc_state_raw_info));
    log_sc_state_raw_info.encoded_sample_size = state->esp_info.sc_info.encoded_raw_log_len;
    log_sc_state_raw_info.diag = state->diag_service;
    log_sc_state_raw_info.instance = instance;
    log_sc_state_raw_info.sensor_uid = &state->esp_info.suid[SC_INDX];
    lsm6dso_log_sensor_state_raw_alloc(&log_sc_state_raw_info, 0);

    if(state->self_test_info.test_alive)
    {
      DBG_INST_PRINTF_EX(HIGH, instance,
                     "test_alive");
    }
    else
    {

      pb_send_sensor_stream_event(instance,
                                  &state->esp_info.suid[SC_INDX],
                                  timestamp,
                                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                  status,
                                  &float_sc_val,
                                  1,
                                  state->esp_info.sc_info.encoded_event_len);

    }
    DBG_INST_PRINTF_EX(HIGH, instance,
        "step_count=%d ts=%u", (int)float_sc_val, (uint32_t)timestamp);

    lsm6dso_log_sensor_state_raw_add(
        &log_sc_state_raw_info,
        &float_sc_val,
        1,
        timestamp,
        status);
    lsm6dso_log_sensor_state_raw_submit(&log_sc_state_raw_info, 1, true);
  }
}

void lsm6dso_handle_sc_interrupt(
    sns_sensor_instance *const instance,
    sns_time irq_timestamp,
    uint8_t src_reg,
    uint8_t wake_data)
{
  UNUSED_VAR(wake_data);
  UNuSED_VAR(src_reg);
  uint16_t stepCount_cur;
  if(read_step_count(instance, &stepCount_cur) == SNS_RC_SUCCESS)
    lsm6dso_convert_and_send_sc_sample(instance, irq_timestamp, stepCount_cur);
}

void lsm6dso_reconfig_sc(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(enable);
  DBG_INST_PRINTF(LOW, instance, "reconfig_sc d/e:0x%x/0x%x", state->esp_info.desired_sensors, state->esp_info.enabled_sensors);
  if(state->esp_info.desired_sensors & LSM6DSO_STEP_COUNTER)
  {
#if LSM6DSO_OEM_FACTORY_CONFIG
    if(state->esp_info.sc_info.config.oem_config_on &&
       !state->esp_info.sc_info.config.oem_config_applied)
    {
      lsm6dso_oem_factory_test_config(instance, false);
      state->esp_info.sc_info.config.oem_config_applied = true;
    }
#endif
    lsm6dso_sc_enable(instance);
  } else if(state->esp_info.enabled_sensors & LSM6DSO_STEP_COUNTER) {
    lsm6dso_sc_disable(instance);
  }
}

size_t lsm6dso_get_encoded_log_len(uint16_t sensor)
{
  size_t len= 0;
  if(sensor == LSM6DSO_STEP_COUNTER)
  {
    uint64_t buffer[10];
    pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
    sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
    uint8_t arr_index = 0;
    float sc_value[1];
    pb_float_arr_arg arg = {.arr = (float*)sc_value, .arr_len = 1,
      .arr_index = &arr_index};
    batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;

    if(pb_encode_tag(&stream, PB_WT_STRING,
          sns_diag_sensor_state_raw_sample_tag))
    {
      if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
            &batch_sample))
      {
        len = stream.bytes_written;
      }
    }
  }
  return len;
}

void lsm6dso_fill_sc_inst_info(sns_sensor *const this, sns_sensor_instance *instance)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  float data[1];
  //to make sure not to overwrite
  if(inst_state->esp_info.sc_info.encoded_event_len)
    return;
  inst_state->esp_info.sc_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);
  inst_state->esp_info.sc_info.encoded_raw_log_len = lsm6dso_get_encoded_log_len(LSM6DSO_STEP_COUNTER);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[SC_INDX],
        sizeof(inst_state->esp_info.suid[SC_INDX]),
        &((sns_sensor_uid)STEP_COUNTER_SUID_1),
        sizeof(inst_state->esp_info.suid[SC_INDX]));
  } else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[SC_INDX],
        sizeof(inst_state->esp_info.suid[SC_INDX]),
       &((sns_sensor_uid)STEP_COUNTER_SUID_0),
        sizeof(inst_state->esp_info.suid[SC_INDX]));

  }
  sns_memscpy(&inst_state->esp_info.sc_info.config, sizeof(inst_state->esp_info.sc_info.config),
      &shared_state->inst_cfg.esp_reg_cfg.sc_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.sc_conf));
  DBG_INST_PRINTF(HIGH, instance,
      "copyng SC esp config");
#if LSM6DSO_OEM_FACTORY_CONFIG
  if(inst_state->esp_info.sc_info.config.oem_config_on)
  {
    inst_state->esp_info.sc_info.config.oem_config_applied = false;
  }
#endif
}

sns_sensor_instance* lsm6dso_set_step_counter_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);


  if(!remove &&
     (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id) &&
     (SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG != new_request->message_id) ) {
    SNS_PRINTF(ERROR, this, "step counter req rejec, msg=%d", new_request->message_id);
    return instance;
  }

  instance = lsm6dso_update_request_q(this, exist_request, new_request, remove);

  if(instance) {
    lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
    if(new_request && (SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG == new_request->message_id))
    {
      sns_std_request decoded_request;
      sns_physical_sensor_test_config decoded_payload = sns_physical_sensor_test_config_init_default;
      if (lsm6dso_get_decoded_self_test_request(this, new_request, &decoded_request, &decoded_payload))
      {
        shared_state->inst_cfg.config_sensors |= state->sensor;
#if LSM6DSO_OEM_FACTORY_CONFIG
        if(decoded_payload.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
        {
          shared_state->inst_cfg.esp_reg_cfg.sc_conf.oem_config_on = true;
        }
#endif
      }
    }
    lsm6dso_fill_sc_inst_info(this, instance);

    inst_state->esp_info.desired_sensors &= ~state->sensor;

    //update the config of specific sensor: this updates desired_sensors variable
    if(NULL != instance->cb->get_client_request(instance, &inst_state->esp_info.suid[SC_INDX], true))
      inst_state->esp_info.desired_sensors |= state->sensor;

    inst_state->esp_info.sc_info.client_present |= (inst_state->esp_info.desired_sensors & state->sensor);
    //enabling stc
    if(!(inst_state->esp_info.enabled_sensors & state->sensor) && (inst_state->esp_info.desired_sensors & state->sensor)) {
  
      inst_state->esp_info.sc_info.config.stepCount_base += inst_state->esp_info.sc_info.config.stepCount_prev;
      inst_state->esp_info.sc_info.config.stepCount_prev = 0;
      sns_memscpy(&shared_state->inst_cfg.esp_reg_cfg.sc_conf, sizeof(shared_state->inst_cfg.esp_reg_cfg.sc_conf),
          &inst_state->esp_info.sc_info.config, sizeof(inst_state->esp_info.sc_info.config));
      DBG_PRINTF(HIGH, this,
      "saving SC esp config: sc_base %u", inst_state->esp_info.sc_info.config.stepCount_base);
    }
    if(remove || ((inst_state->esp_info.enabled_sensors & state->sensor) != (inst_state->esp_info.desired_sensors & state->sensor)))
      instance = lsm6dso_handle_client_request(this, exist_request, new_request, remove);
  }
  return instance;
}

sns_sensor_api lsm6dso_step_counter_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_step_counter_init,
  .deinit             = &lsm6dso_step_counter_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_step_counter_request,
  .notify_event       = &lsm6dso_sensor_notify_event,
};

#endif
