/**
 * @file sns_lsm6dso_sensor_instance.c
 *
 * LSM6DSO Accel virtual Sensor Instance implementation.
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

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_types.h"
#include "sns_sensor_util.h"

#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"
#include "sns_lsm6dso_log_pckts.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_cal.pb.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "float.h"
/**
 * Accelerometer LSM6DSO hardware self test settings
 */
#define LSM6DSO_ST_2G_MIN       50*G/1000      //Unit: m/s2
#define LSM6DSO_ST_2G_MAX       1700*G/1000    //Unit: m/s2

/**
 * Gyroscope LSM6DSO self test settings
 */
#define LSM6DSO_2000DPS_ST_MIN      150  //unit dps.
#define LSM6DSO_2000DPS_ST_MAX      700
#define ACC_CONV          ((LSM6DSO_ACCEL_SSTVT_2G * G)/1000000)
#define GYRO_CONV         LSM6DSO_GYRO_SSTVT_125DPS
extern const odr_reg_map lsm6dso_odr_map[];
extern const uint32_t lsm6dso_odr_map_len;

#if LSM6DSO_DAE_ENABLED
static void lsm6dso_flush_dae_data(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint64_t max_requested_flush_ticks = state->desired_conf.max_requested_flush_ticks;

  //Flush old data from DAE if already streaming
  state->desired_conf.max_requested_flush_ticks = 0;
  lsm6dso_dae_if_start_streaming(this);
  lsm6dso_dae_if_flush_hw(this);

  state->desired_conf.max_requested_flush_ticks = max_requested_flush_ticks;
}
#endif

static sns_rc lsm6dso_set_accel_config_fifo(
  sns_sensor_instance *const instance,
  lsm6dso_accel_odr    curr_odr,
  lsm6dso_accel_sstvt  sstvt,
  lsm6dso_accel_range  range,
  lsm6dso_accel_bw     bw)
{

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  bool is_md_force_disabled = false;
  //disable md if enabled and is required
  //Avoiding spurious interrupts
  if(state->current_conf.md_enabled && lsm6dso_is_md_int_required(instance)) {
    lsm6dso_disable_md(instance, false);
    is_md_force_disabled = true;
  }

  // set fifo bypass mode
  lsm6dso_set_fifo_bypass_mode(instance);

#if LSM6DSO_DAE_ENABLED
  if(lsm6dso_dae_if_available(instance)) {
    lsm6dso_flush_dae_data(instance);
    lsm6dso_dae_if_start_streaming(instance);
  }
#endif

  //Set accel config
  rv = lsm6dso_set_accel_config(instance,curr_odr,sstvt,range,bw);
  if(rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->accel_info.num_samples_to_discard =
    lsm6dso_odr_map[lsm6dso_get_odr_rate_idx(curr_odr)].accel_discard_samples;
  state->cur_odr_change_info.odr_idx = state->desired_conf.odr_idx;
  state->cur_odr_change_info.accel_odr_settime = sns_get_system_time();
  state->cur_odr_change_info.change_req |= LSM6DSO_ACCEL;
  state->cur_odr_change_info.changed |= LSM6DSO_ACCEL;

  if(!state->self_test_info.reconfig_postpone)
    lsm6dso_set_fifo_wmk(instance);
  //start streaming,stream mode
  lsm6dso_set_fifo_stream_mode(instance);

  //re-enable if force disabled before
  if(is_md_force_disabled) {
    lsm6dso_enable_md(instance, false);
  }

  state->fifo_info.last_timestamp = sns_get_system_time();
  return rv;
}

sns_rc lsm6dso_set_gyro_config_fifo(
  sns_sensor_instance *const instance,
  lsm6dso_gyro_odr     curr_odr,
  lsm6dso_gyro_sstvt   sstvt,
  lsm6dso_gyro_range   range)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  // set fifo bypass mode
  lsm6dso_set_fifo_bypass_mode(instance);

#if LSM6DSO_DAE_ENABLED
  if(lsm6dso_dae_if_available(instance)) {
    lsm6dso_flush_dae_data(instance);
    lsm6dso_dae_if_start_streaming(instance);
  }
#endif

  //Set gyro config
  rv = lsm6dso_set_gyro_config(instance,curr_odr,sstvt,range);
  if(rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->gyro_info.num_samples_to_discard  =
    lsm6dso_odr_map[lsm6dso_get_odr_rate_idx(curr_odr)].gyro_discard_samples;
  state->cur_odr_change_info.odr_idx = state->desired_conf.odr_idx;
  state->cur_odr_change_info.gyro_odr_settime = sns_get_system_time();
  state->cur_odr_change_info.change_req |= LSM6DSO_GYRO;
  state->cur_odr_change_info.changed |= LSM6DSO_GYRO;

  if(!state->self_test_info.reconfig_postpone)
    lsm6dso_set_fifo_wmk(instance);

  //start streaming,stream mode
  lsm6dso_set_fifo_stream_mode(instance);

  state->fifo_info.last_timestamp = sns_get_system_time();
  return rv;
}

static void enable_hw_self_test(
  sns_sensor_instance *const this,
  lsm6dso_sensor_type sensor,
  bool enable)
{
  uint8_t buffer;
  uint32_t xfer_bytes;

  if(enable == false)
  {
    buffer = 0x00;
  }
  else if(sensor == LSM6DSO_ACCEL)
  {
    buffer = 0x01;
  }
  else //enable bit for gyro
  {
    buffer = 0x04;
  }
  lsm6dso_read_modify_write(this,
                            STM_LSM6DSO_REG_CTRL5,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x00);

}

static void lsm6dso_inst_collect_data(sns_sensor_instance *const this, int64_t *data)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_sensor_type sensor[1];
  int16_t raw_data[3] = {0,0,0};
  int8_t idx;
  lsm6dso_accel_odr curr_odr = LSM6DSO_ACCEL_ODR52;
  uint16_t self_test_odr = LSM6DSO_ODR_52;

  //handle case were new streaming ODR is less than self test ODR
  if((state->self_test_info.curr_odr >  state->common_info.accel_curr_odr) &&
     (state->common_info.accel_curr_odr != LSM6DSO_ACCEL_ODR_OFF))
  {
    state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
  }

  for(idx = 0; idx < lsm6dso_odr_map_len; idx++)
  {
    if(curr_odr == lsm6dso_odr_map[idx].accel_odr_reg_value
       &&
       curr_odr != LSM6DSO_ACCEL_ODR_OFF)
    {
      self_test_odr = lsm6dso_odr_map[idx].odr;
      break;
    }
  }
  sensor[0] = state->self_test_info.sensor;

  //Check if any samples need to be dropped
  if(state->self_test_info.skip_count)
  {
    lsm6dso_get_data(this,sensor,1,raw_data);
    DBG_INST_PRINTF_EX(LOW, this, "Discarding the first %u samples", state->self_test_info.skip_count);
    sns_time timeout = sns_convert_ns_to_ticks(((1000 * 1000)/self_test_odr) * 1000 * state->self_test_info.skip_count);
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout;
    lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); // Skip this sample as it may be invalid
    state->self_test_info.skip_count=0;
    return;
  }

  //Poll data for 25 samples, save total
  lsm6dso_get_data(this,sensor,1,raw_data);
  state->self_test_info.polling_count++;
  data[0] += raw_data[0];
  data[1] += raw_data[1];
  data[2] += raw_data[2];

  if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
  {
  //Collect sample square sum for Variance evaluation
    state->self_test_info.cumulative_data_post[0] += raw_data[0]*raw_data[0];
    state->self_test_info.cumulative_data_post[1] += raw_data[1]*raw_data[1];
    state->self_test_info.cumulative_data_post[2] += raw_data[2]*raw_data[2];
  }

  if(state->self_test_info.polling_count == SELF_TEST_DATA_COUNT_MAX)
  {
    data[0] /= SELF_TEST_DATA_COUNT_MAX;
    data[1] /= SELF_TEST_DATA_COUNT_MAX;
    data[2] /= SELF_TEST_DATA_COUNT_MAX;
    state->self_test_info.polling_count = 0;
    state->self_test_info.self_test_stage++;
  }
  else
  {
    sns_time timeout = sns_convert_ns_to_ticks(((1000 * 1000)/self_test_odr) * 1000);
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout;
    lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); // wait for next odr to read the next sample
  }
}


static void send_hw_self_test_result(sns_sensor_instance *const instance)
{

  bool test_passed = false;
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;
  sns_sensor_uid *suid_current;
  float data_conv;
  int32_t mean_diff_i[3]={0,0,0};
  float abs_mean_diff_f[3]={0,0,0};

  //Compare the two averages - copy from ddf
  //Check self test limits
  if(state->self_test_info.sensor == LSM6DSO_ACCEL)
  {
    suid_current = &state->accel_info.suid;
    data_conv = LSM6DSO_ACCEL_SSTVT_HW_SELFTEST * G/1000000;
    mean_diff_i[0]=abs((int32_t)state->self_test_info.cumulative_data_post[0]-(int32_t)state->self_test_info.cumulative_data_pre[0]);
    mean_diff_i[1]=abs((int32_t)state->self_test_info.cumulative_data_post[1]-(int32_t)state->self_test_info.cumulative_data_pre[1]);
    mean_diff_i[2]=abs((int32_t)state->self_test_info.cumulative_data_post[2]-(int32_t)state->self_test_info.cumulative_data_pre[2]);
    abs_mean_diff_f[0]=mean_diff_i[0]*data_conv;
    abs_mean_diff_f[1]=mean_diff_i[1]*data_conv;
    abs_mean_diff_f[2]=mean_diff_i[2]*data_conv;
    if( (LSM6DSO_ST_2G_MIN <= abs_mean_diff_f[0]) && (abs_mean_diff_f[0] <= LSM6DSO_ST_2G_MAX)
      &&(LSM6DSO_ST_2G_MIN <= abs_mean_diff_f[1]) && (abs_mean_diff_f[1] <= LSM6DSO_ST_2G_MAX)
      &&(LSM6DSO_ST_2G_MIN <= abs_mean_diff_f[2]) && (abs_mean_diff_f[2] <= LSM6DSO_ST_2G_MAX))
      test_passed = true;
    else
      test_passed = false;
  }
  else //gyro
  {
    suid_current = &state->gyro_info.suid;
    data_conv = LSM6DSO_GYRO_SSTVT_2000DPS;
    mean_diff_i[0]=abs((int32_t)state->self_test_info.cumulative_data_post[0]-(int32_t)state->self_test_info.cumulative_data_pre[0]);
    mean_diff_i[1]=abs((int32_t)state->self_test_info.cumulative_data_post[1]-(int32_t)state->self_test_info.cumulative_data_pre[1]);
    mean_diff_i[2]=abs((int32_t)state->self_test_info.cumulative_data_post[2]-(int32_t)state->self_test_info.cumulative_data_pre[2]);
    abs_mean_diff_f[0]=mean_diff_i[0]*data_conv;
    abs_mean_diff_f[1]=mean_diff_i[1]*data_conv;
    abs_mean_diff_f[2]=mean_diff_i[2]*data_conv;
    if( (LSM6DSO_2000DPS_ST_MIN <= abs_mean_diff_f[0]) && (abs_mean_diff_f[0] <= LSM6DSO_2000DPS_ST_MAX)
      &&(LSM6DSO_2000DPS_ST_MIN <= abs_mean_diff_f[1]) && (abs_mean_diff_f[1] <= LSM6DSO_2000DPS_ST_MAX)
      &&(LSM6DSO_2000DPS_ST_MIN <= abs_mean_diff_f[2]) && (abs_mean_diff_f[2] <= LSM6DSO_2000DPS_ST_MAX))
      test_passed = true;
    else
      test_passed = false;
  }
  DBG_INST_PRINTF(MED, instance, "Self Test Result=%d", test_passed);


  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };

  DBG_INST_PRINTF_EX(HIGH, instance, "Sending Self Test event");

  physical_sensor_test_event.test_passed = test_passed;
  physical_sensor_test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
  physical_sensor_test_event.test_data.funcs.encode = &pb_encode_string_cb;
  physical_sensor_test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &physical_sensor_test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                suid_current);

  state->self_test_info.test_alive = false;
}

static void send_factory_self_test_result(sns_sensor_instance *const instance, bool test_passed)
{

  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  DBG_INST_PRINTF(MED, instance, "[%u] Self Test(factory) Result=%d", state->hw_idx, test_passed);

  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_sensor_uid *suid_current;

  //update suid
  if(state->self_test_info.sensor == LSM6DSO_ACCEL)
  {
    suid_current = &state->accel_info.suid;
  }
  else
  {
    suid_current = &state->gyro_info.suid;
  }

  if(test_passed)
    lsm6dso_send_cal_event(instance, state->self_test_info.sensor);

  DBG_INST_PRINTF_EX(HIGH, instance, "Sending Self Test event");

  physical_sensor_test_event.test_passed = test_passed;
  physical_sensor_test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
  physical_sensor_test_event.test_data.funcs.encode = &pb_encode_string_cb;
  physical_sensor_test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &physical_sensor_test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                suid_current);

  state->self_test_info.test_alive = false;
}

static void send_com_self_test_result(sns_sensor_instance *const instance, bool test_passed)
{

  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  DBG_INST_PRINTF_EX(MED, instance, "Self Test(com) Result=%d", test_passed);

  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_sensor_uid *suid_current;

  //update suid
  if(state->self_test_info.sensor == LSM6DSO_GYRO)
  {
    suid_current = &state->gyro_info.suid;
  }
  else if(state->self_test_info.sensor == LSM6DSO_MOTION_DETECT)
  {
    suid_current = &state->md_info.suid;
  }
  else if(state->self_test_info.sensor == LSM6DSO_SENSOR_TEMP)
  {
    suid_current = &state->sensor_temp_info.suid;
  }
  else
  {
    suid_current = &state->accel_info.suid;
  }

  DBG_INST_PRINTF_EX(HIGH, instance, "Sending Self Test event");

  physical_sensor_test_event.test_passed = test_passed;
  physical_sensor_test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
  physical_sensor_test_event.test_data.funcs.encode = &pb_encode_string_cb;
  physical_sensor_test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &physical_sensor_test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                suid_current);

  state->self_test_info.test_alive = false;
  state->health.heart_attack = false;
}

void lsm6dso_inst_hw_self_test(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  uint32_t timeout = 0;
  lsm6dso_accel_odr curr_odr = LSM6DSO_ACCEL_ODR52;

  switch (state->self_test_info.self_test_stage)
  {
    case LSM6DSO_SELF_TEST_STAGE_1:
    {
      //Check if sensor is already streaming. If not, turn it ON
      if((state->common_info.accel_curr_odr) && (state->common_info.accel_curr_odr < LSM6DSO_ACCEL_ODR832))
      {
        state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
        curr_odr = state->self_test_info.curr_odr;
      }
      else
      {
        state->self_test_info.curr_odr = LSM6DSO_ACCEL_ODR52;
      }

      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        if((state->gyro_info.desired_odr) && !(state->accel_info.desired_odr))
        {
          lsm6dso_set_fifo_wmk(this);
        }
        if(!state->accel_info.desired_odr)
        {
          lsm6dso_set_accel_config(this,curr_odr,LSM6DSO_ACCEL_SSTVT_HW_SELFTEST,LSM6DSO_ACCEL_RANGE_HW_SELFTEST,state->accel_info.bw);
        }
        else
        {
          lsm6dso_set_accel_config_fifo(this,curr_odr,LSM6DSO_ACCEL_SSTVT_HW_SELFTEST,LSM6DSO_ACCEL_RANGE_HW_SELFTEST,state->accel_info.bw);
        }
        timeout = 200;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dso_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == lsm6dso_odr_map[state->self_test_info.odr_idx].accel_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
            break;
          }
        }
      }
      else
      {
        if((state->accel_info.desired_odr) && !(state->gyro_info.desired_odr))
        {
          lsm6dso_set_fifo_wmk(this);
        }
        if(!state->gyro_info.desired_odr)
        {
          lsm6dso_set_gyro_config(this,(lsm6dso_gyro_odr)curr_odr,LSM6DSO_GYRO_SSTVT_2000DPS,STM_LSM6DSO_GYRO_RANGE_2000DPS);
        }
        else
        {
          lsm6dso_set_gyro_config_fifo(this,(lsm6dso_gyro_odr)curr_odr,LSM6DSO_GYRO_SSTVT_2000DPS,STM_LSM6DSO_GYRO_RANGE_2000DPS);
        }
        timeout = 300;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dso_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == (lsm6dso_accel_odr)lsm6dso_odr_map[state->self_test_info.odr_idx].gyro_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
            break;
          }
        }
      }
      state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_2;

      lsm6dso_dump_reg(this, state->fifo_info.fifo_enabled);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); //Settling time for ODR
    }
    break;

    case LSM6DSO_SELF_TEST_STAGE_2:
    {
      lsm6dso_inst_collect_data(this, state->self_test_info.cumulative_data_pre);
    }
    if(state->self_test_info.self_test_stage == LSM6DSO_SELF_TEST_STAGE_2)
      break;

    case LSM6DSO_SELF_TEST_STAGE_3:
    {
      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        timeout = 100;
        state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
      }
      else //gyro
      {
        timeout = 50;
        state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
      }
      //Set bit for self test
      enable_hw_self_test(this, state->self_test_info.sensor, true);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload);
      state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_4;
    }
    break;

    case LSM6DSO_SELF_TEST_STAGE_4:
    {
      lsm6dso_inst_collect_data(this, state->self_test_info.cumulative_data_post);
    }
    if(state->self_test_info.self_test_stage == LSM6DSO_SELF_TEST_STAGE_4)
      break;
    if(state->self_test_info.self_test_stage == LSM6DSO_SELF_TEST_STAGE_5)
    {
      //Disable Self test bit
      enable_hw_self_test(this, state->self_test_info.sensor, false);
      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        timeout = 100;
      }
      else
      {
        timeout = 50;
      }
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload);
      break;
    }

    case LSM6DSO_SELF_TEST_STAGE_5:
    {
      send_hw_self_test_result(this);
      //Disable streaming if started for self test else set the original ODR is more than 832Hz
      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        if(!state->accel_info.desired_odr)
        {
          lsm6dso_set_accel_config(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        }
        else
        {
          lsm6dso_set_accel_config_fifo(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        }
        if(state->gyro_info.desired_odr)
        {
          lsm6dso_set_fifo_wmk(this);
        }
        if((state->gyro_info.desired_odr) && (!state->accel_info.desired_odr))
        {
          state->gyro_info.num_samples_to_discard +=
            lsm6dso_odr_map[state->desired_conf.odr_idx].gyro_discard_samples;
        }
      }
      else if(state->self_test_info.sensor == LSM6DSO_GYRO)
      {
        if(!state->gyro_info.desired_odr)
        {
          lsm6dso_set_gyro_config(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
        }
        else
        {
          lsm6dso_set_gyro_config_fifo(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
        }
        if(state->accel_info.desired_odr)
        {
          lsm6dso_set_fifo_wmk(this);
        }
        if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr))
        {
          state->accel_info.num_samples_to_discard += lsm6dso_odr_map[state->desired_conf.odr_idx].accel_discard_samples;
        }
      }
      sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_self_test_data_stream);
      state->common_info.mode &= ~LSM6DSO_MODE_SELF_TEST;
      //Disable all self test flags
      state->self_test_info.polling_count = 0;
      state->self_test_info.curr_odr = LSM6DSO_ACCEL_ODR52;
      state->health.heart_attack = false;
      sns_memset(state->self_test_info.cumulative_data_pre, 0, sizeof(state->self_test_info.cumulative_data_pre));
      sns_memset(state->self_test_info.cumulative_data_post, 0, sizeof(state->self_test_info.cumulative_data_post));
    }
    default:
    {
    }
    break;
  }

}

void lsm6dso_inst_factory_self_test(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint32_t timeout = 0;
  lsm6dso_accel_odr curr_odr = LSM6DSO_ACCEL_ODR52;
  float fac_cal_bias[3] = {0};
  float bias_thresholds[3] = {0};
  int64_t variance[3] = {0};
  int64_t variance_threshold = 0;

  switch (state->self_test_info.self_test_stage)
  {
    case LSM6DSO_SELF_TEST_STAGE_1:
    {
#if LSM6DSO_OEM_FACTORY_CONFIG
    //Enable factory test specific OEM configuration
    lsm6dso_oem_factory_test_config(this, true);
    if(state->self_test_info.return_now)
    {
      state->self_test_info.return_now = false;
      send_factory_self_test_result(this, true);
      return;
    }
#endif
      //Check if sensor is already streaming. If not, turn it ON
      if((state->common_info.accel_curr_odr) && (state->common_info.accel_curr_odr < LSM6DSO_ACCEL_ODR832))
      {
        state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
        curr_odr = state->self_test_info.curr_odr;
      }
      else
      {
        state->self_test_info.curr_odr = LSM6DSO_ACCEL_ODR52;
      }

      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        if((state->gyro_info.desired_odr) && !(state->accel_info.desired_odr))
        {
          lsm6dso_set_fifo_wmk(this);
        }
        lsm6dso_set_accel_config(this,curr_odr,LSM6DSO_ACCEL_SSTVT_2G,LSM6DSO_ACCEL_RANGE_2G,state->accel_info.bw);
        timeout = 200;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dso_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == lsm6dso_odr_map[state->self_test_info.odr_idx].accel_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
            break;
          }
        }
      }
      else
      {
        if((state->accel_info.desired_odr) && !(state->gyro_info.desired_odr))
        {
          lsm6dso_set_fifo_wmk(this);
        }
        lsm6dso_set_gyro_config(this,(lsm6dso_gyro_odr)curr_odr,LSM6DSO_GYRO_SSTVT_125DPS,STM_LSM6DSO_GYRO_RANGE_125DPS);
        timeout = 300;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dso_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == (lsm6dso_accel_odr)lsm6dso_odr_map[state->self_test_info.odr_idx].gyro_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dso_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
            break;
          }
        }
      }
      state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_2;

      lsm6dso_dump_reg(this, state->fifo_info.fifo_enabled);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dso_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); //Settling time for ODR
    }
    break;

    case LSM6DSO_SELF_TEST_STAGE_2:
    {
      lsm6dso_inst_collect_data(this, state->self_test_info.cumulative_data_pre);
    }
    if(state->self_test_info.self_test_stage == LSM6DSO_SELF_TEST_STAGE_2)
      break;

    case LSM6DSO_SELF_TEST_STAGE_3:
    {
      int i;
      bool test_pass=true;
      SNS_SPRINTF(ERROR, sns_fw_printf,"Sample Sums: %ld/%ld/%ld",
                      state->self_test_info.cumulative_data_pre[0],
                      state->self_test_info.cumulative_data_pre[1],
                      state->self_test_info.cumulative_data_pre[2]);

      SNS_SPRINTF(ERROR, sns_fw_printf, "Sample square sums: %ld/%ld/%ld",
                      state->self_test_info.cumulative_data_post[0],
                      state->self_test_info.cumulative_data_post[1],
                      state->self_test_info.cumulative_data_post[2]);

/*
      SNS_INST_PRINTF(HIGH, this, "Sample Sums: %d/%d/%d",
                      state->self_test_info.cumulative_data_pre[0],
                      state->self_test_info.cumulative_data_pre[1],
                      state->self_test_info.cumulative_data_pre[2]);

      SNS_INST_PRINTF(HIGH, this, "Sample square sums: %d/%d/%d",
                      state->self_test_info.cumulative_data_post[0],
                      state->self_test_info.cumulative_data_post[1],
                      state->self_test_info.cumulative_data_post[2]);
*/
#if LSM6DSO_USE_FIXED_ORIENTATION
      if(state->self_test_info.sensor == LSM6DSO_ACCEL)
      {
        uint64_t cumulative_data_pre = (state->axis_map[2].invert ? -1.0 : 1.0) * state->self_test_info.cumulative_data_pre[2];
        bias_thresholds[0] = 180 * G / 1000; // 210 m/s2
        bias_thresholds[1] = 180 * G / 1000; // 210 m/s2
        bias_thresholds[2] = 210 * G / 1000; // 210 m/s2
        if (cumulative_data_pre > 0) {
          if (state->axis_map[2].invert)
            state->self_test_info.cumulative_data_pre[2] += (float)(G/ACC_CONV);
          else
            state->self_test_info.cumulative_data_pre[2] -= (float)(G/ACC_CONV);
        }
      }
#endif

      for(i = 0; i < TRIAXIS_NUM; i++)
      {
        uint64_t varT = (state->self_test_info.cumulative_data_pre[i]) * (state->self_test_info.cumulative_data_pre[i]);

        variance[i] = (state->self_test_info.cumulative_data_post[i]/SELF_TEST_DATA_COUNT_MAX
                                            - (varT));

        DBG_INST_PRINTF_EX(HIGH, this, "varT = %d/%d var[%u]=%d/%d",
                        (int32_t)varT, (int32_t)(varT>>32), i, (int32_t)variance[i], (int32_t)(variance[i]>>32));

        //variance_threshold = 1073741823; //2^15^2-1

        if(state->self_test_info.sensor == LSM6DSO_ACCEL)
        {
#if !LSM6DSO_USE_FIXED_ORIENTATION
          //Calculate bias based on orientation
          if((float)(state->self_test_info.cumulative_data_pre[i] * ACC_CONV) > 2.0f)
          {
            state->self_test_info.cumulative_data_pre[i] = state->self_test_info.cumulative_data_pre[i]  - (G / ACC_CONV);
            bias_thresholds[i] = 210 * G / 1000; // 210 m/s2;
          }
          else  if((state->self_test_info.cumulative_data_pre[i] * ACC_CONV) < -2.0f)
          {
            state->self_test_info.cumulative_data_pre[i] = state->self_test_info.cumulative_data_pre[i]  + (G / ACC_CONV);
            bias_thresholds[i] =  210 * G / 1000; // 210 m/s2;
          }
          else
          {
            bias_thresholds[i] = 180 * G / 1000; // 180 m/s2
          }
#endif
          fac_cal_bias[i] = (float)(state->self_test_info.cumulative_data_pre[i] * ACC_CONV);
          variance_threshold = 1073741823;
        }
        else if(state->self_test_info.sensor == LSM6DSO_GYRO)
        {
          fac_cal_bias[i] = (float)(state->self_test_info.cumulative_data_pre[i] * GYRO_CONV * PI) / (180.0f);
          bias_thresholds[i] = 40 * PI / 180; //40 rad/sec
          variance_threshold = 3343673;
        }

        // Check variance to determine whether device is stationary
        if(variance[i] > variance_threshold)
        {
          // device is not stationary
          test_pass = false;
          SNS_INST_PRINTF(ERROR, this, "FAILED device not stationary var[%u]=%d/%d %d/%d",
                          i, (int32_t)variance[i], (int32_t)(variance[i]>>32),
                          (int32_t)variance_threshold, (int32_t)(variance_threshold>>32));
          break;
        }

        // Check biases are in defined limits
        if((uint32_t)(fabsf(fac_cal_bias[i])*1000) > (uint32_t)((bias_thresholds[i])*1000))
        {
          test_pass = false;
          SNS_INST_PRINTF(ERROR, this, "FAILED bias very large.");
          break;
        }

        // Check for zero variance
        if(variance[i] == 0)
        {
           test_pass = false;
           SNS_INST_PRINTF(ERROR, this, "FAILED zero variance");
           break;
        }
      }

      if(!test_pass)
      {
        send_factory_self_test_result(this, false);
      }
      else
      {
        sns_time now = sns_get_system_time();

        //Calculate bias values
        if(state->self_test_info.sensor == LSM6DSO_ACCEL)
        {
          //Calculate bias based on orientation
          for(i=0;i<TRIAXIS_NUM;i++)
          {
            state->accel_registry_cfg.fac_cal_bias[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) * fac_cal_bias[state->axis_map[i].ipaxis];
          }

          state->accel_registry_cfg.ts = now;
          state->accel_registry_cfg.registry_instance_version++;
          DBG_INST_PRINTF(LOW, this, "Debug: Accel Bias values*1000: X=%d Y=%d Z=%d",
                         (int32_t)(state->accel_registry_cfg.fac_cal_bias[0]*1000),
                         (int32_t)(state->accel_registry_cfg.fac_cal_bias[1]*1000),
                         (int32_t)(state->accel_registry_cfg.fac_cal_bias[2]*1000));
        }
        else if(state->self_test_info.sensor == LSM6DSO_GYRO)
        {
          for(i=0;i<TRIAXIS_NUM;i++)
          {
            state->gyro_registry_cfg.fac_cal_bias[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) * fac_cal_bias[state->axis_map[i].ipaxis];
          }
          state->gyro_registry_cfg.ts = now;
          state->gyro_registry_cfg.registry_instance_version++;
          DBG_INST_PRINTF(LOW, this, "Debug: Gyro Bias values*1000: X=%d Y=%d Z=%d",
                         (int32_t)(state->gyro_registry_cfg.fac_cal_bias[0]*1000),
                         (int32_t)(state->gyro_registry_cfg.fac_cal_bias[1]*1000),
                         (int32_t)(state->gyro_registry_cfg.fac_cal_bias[2]*1000));
        }

        //Set flag to enable write registory with bias values from sensor set_client_request
        state->self_test_info.update_registry = true;

        //write_output_to_registry(this);
        //Send result
        send_factory_self_test_result(this, true);
      }

     //This is needed for a change where in reconfig_postpone no HW reconfiguration is needed.
      if(state->self_test_info.reconfig_postpone)
      {
        if(state->common_info.accel_curr_odr)
          lsm6dso_set_accel_config_fifo(this,state->self_test_info.curr_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        if(state->common_info.gyro_curr_odr)
          lsm6dso_set_gyro_config_fifo(this,(lsm6dso_gyro_odr)state->self_test_info.curr_odr,state->gyro_info.sstvt,state->gyro_info.range);
      }
      //Disable streaming if started for self test else set the original ODR if more than 832Hz
      if(state->self_test_info.sensor == LSM6DSO_ACCEL &&
         !state->self_test_info.reconfig_postpone)
      {
        if(!(state->accel_info.desired_odr) || (state->accel_info.desired_odr > LSM6DSO_ACCEL_ODR832) || (state->accel_info.range != LSM6DSO_ACCEL_RANGE_2G))
        {
          if(!state->accel_info.desired_odr)
          {
            lsm6dso_set_accel_config(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
            if(state->gyro_info.desired_odr)
            {
              lsm6dso_set_fifo_wmk(this);
            }
          }
          else
          {
            lsm6dso_set_accel_config_fifo(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
          }
        }
        if((state->gyro_info.desired_odr) && (!state->accel_info.desired_odr))
        {
          state->gyro_info.num_samples_to_discard += lsm6dso_odr_map[state->desired_conf.odr_idx].gyro_discard_samples;
        }
      }
      else if(state->self_test_info.sensor == LSM6DSO_GYRO &&
              !state->self_test_info.reconfig_postpone)
      {
        if(!(state->gyro_info.desired_odr) || (state->gyro_info.desired_odr > LSM6DSO_ACCEL_ODR832) || (state->gyro_info.range != STM_LSM6DSO_GYRO_RANGE_125DPS))
        {
          if(!state->gyro_info.desired_odr)
          {
            lsm6dso_set_gyro_config(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
            if(state->accel_info.desired_odr)
            {
              lsm6dso_set_fifo_wmk(this);
            }
          }
          else
          {
            lsm6dso_set_gyro_config_fifo(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
          }
        }
        if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr))
        {
          state->accel_info.num_samples_to_discard += lsm6dso_odr_map[state->desired_conf.odr_idx].accel_discard_samples;
        }
      }
      sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_self_test_data_stream);

      state->common_info.mode &= ~LSM6DSO_MODE_SELF_TEST;
      //Disable all self test flags
      state->self_test_info.polling_count = 0;
      state->self_test_info.curr_odr = LSM6DSO_ACCEL_ODR52;
      state->health.heart_attack = false;
      sns_memset(state->self_test_info.cumulative_data_pre, 0, sizeof(state->self_test_info.cumulative_data_pre));
      sns_memset(state->self_test_info.cumulative_data_post, 0, sizeof(state->self_test_info.cumulative_data_post));

      //lsm6dso_dae_if_start_streaming(this, state->self_test_info.sensor);
    }
    default:
    {
    }
    break;
  }

}

void lsm6dso_inst_com_self_test(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t buffer = 0;
  uint32_t rw_bytes = 0;
  bool who_am_i_success = false;

  rv = lsm6dso_instance_com_read_wrapper(state,
      STM_LSM6DSO_REG_WHO_AM_I,
      &buffer,
      1,
      &rw_bytes);

  if(rv == SNS_RC_SUCCESS
     &&
     buffer == LSM6DSO_WHOAMI_VALUE)
  {
    who_am_i_success = true;
  }

  //Send result
  send_com_self_test_result(this, who_am_i_success);
  state->common_info.mode &= ~LSM6DSO_MODE_SELF_TEST;
  //Disable all self test flags
  state->self_test_info.polling_count = 0;
  state->self_test_info.curr_odr = LSM6DSO_ACCEL_ODR52;
}

static void inst_cleanup(sns_sensor_instance *const this,
                         lsm6dso_instance_state *state)
{
  SNS_INST_PRINTF(HIGH, this, "[%u] inst_cleanup: #samples=%u/%u/%u/%u",
                  state->hw_idx, state->accel_sample_counter, state->gyro_sample_counter,
                  state->num_temp_samples, state->num_md_ints);

  lsm6dso_set_fifo_config(this, 0, 0, 0, 0 );
  lsm6dso_reconfig_hw(this);
  lsm6dso_turn_on_bus_power(state, false);

  sns_sensor_util_remove_sensor_instance_stream(this, &state->interrupt2_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->interrupt_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->async_com_port_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_temp_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_ois_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_md_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_self_test_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_heart_beat_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_polling_data_stream);

  if(NULL != state->scp_service)
  {
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
    state->scp_service = NULL;
  }
  lsm6dso_dae_if_deinit(this);
  lsm6dso_esp_deinit(this);
}

/** See sns_sensor_instance_api::init */
sns_rc lsm6dso_inst_init(sns_sensor_instance *const this, sns_sensor_state const *sstate)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state_from_state(sstate);
  lsm6dso_instance_config const *inst_cfg = &shared_state->inst_cfg;
  float data[3];
  float temp_data[1];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  //memset instance state
  sns_memset(state, 0, sizeof(lsm6dso_instance_state));

  //this would be stored when the state is initialized
  state->hw_idx = shared_state->hw_idx;
  state->rigid_body_type = shared_state->rigid_body_type;
  state->clock_trim_factor = shared_state->clock_trim_factor;
  state->odr_percent_var_accel = shared_state->odr_percent_var_accel;
  state->odr_percent_var_gyro = shared_state->odr_percent_var_gyro;
  state->min_odr_idx = inst_cfg->min_odr_idx;
  state->bus_pwr_on = false;

  state->scp_service = (sns_sync_com_port_service*)
              service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  /**---------Setup stream connections with dependent Sensors---------*/
  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 inst_cfg->irq_suid,
                                                 &state->interrupt_data_stream);

  if(shared_state->irq2_enabled)
  {
    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                   this,
                                                   inst_cfg->irq_suid,
                                                   &state->interrupt2_data_stream);
  }

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 inst_cfg->acp_suid,
                                                 &state->async_com_port_data_stream);
  state->ag_stream_mode = inst_cfg->ag_stream_mode;
  SNS_INST_PRINTF(HIGH, this, "[%u] inst_init:: ag_stream_mode:%d",
                  state->hw_idx, state->ag_stream_mode);
  /** Initialize COM port to be used by the Instance */
  sns_memscpy(&state->com_port_info,
              sizeof(state->com_port_info),
              &inst_cfg->com_port_info,
              sizeof(inst_cfg->com_port_info));

  state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                              &state->com_port_info.port_handle);

  if(NULL == state->interrupt_data_stream ||
     (NULL == state->interrupt2_data_stream && inst_cfg->irq2_enabled) ||
     NULL == state->async_com_port_data_stream ||
     NULL == state->com_port_info.port_handle)
  {
    SNS_INST_PRINTF(ERROR, this, "inst_init: something failed");
    inst_cleanup(this, state);
    return SNS_RC_FAILED;
  }


  /**------------------------- Init Hw Reg -------------------------*/
  /** customer request to set OIS_PU_DIS on PIN_CTRL */
#if LSM6DSO_SET_OIS_PU_DIS
  uint8_t reg_buffer = 0x80;
  uint32_t xfer_bytes;
  // QC - won't this get reset after lsm6dso_reset_device() is called?
  lsm6dso_read_modify_write(this,
                            STM_LSM6DSO_REG_PIN_CTRL,
                            &reg_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x80);
#endif

  /**----------- Copy all Sensor UIDs in instance state -------------*/
  if(state->hw_idx == 0) {
    sns_memscpy(&state->accel_info.suid,
        sizeof(state->accel_info.suid),
        &((sns_sensor_uid)ACCEL_SUID_0),
        sizeof(state->accel_info.suid));
    sns_memscpy(&state->gyro_info.suid,
        sizeof(state->gyro_info.suid),
        &((sns_sensor_uid)GYRO_SUID_0),
        sizeof(state->gyro_info.suid));
    sns_memscpy(&state->md_info.suid,
        sizeof(state->md_info.suid),
        &((sns_sensor_uid)MOTION_DETECT_SUID_0),
        sizeof(state->md_info.suid));
    sns_memscpy(&state->sensor_temp_info.suid,
        sizeof(state->sensor_temp_info.suid),
        &((sns_sensor_uid)SENSOR_TEMPERATURE_SUID_0),
        sizeof(state->sensor_temp_info.suid));
#if LSM6DSO_DUAL_SENSOR_ENABLED
  } else {
    sns_memscpy(&state->accel_info.suid,
        sizeof(state->accel_info.suid),
        &((sns_sensor_uid)ACCEL_SUID_1),
        sizeof(state->accel_info.suid));
    sns_memscpy(&state->gyro_info.suid,
        sizeof(state->gyro_info.suid),
        &((sns_sensor_uid)GYRO_SUID_1),
        sizeof(state->gyro_info.suid));
    sns_memscpy(&state->md_info.suid,
        sizeof(state->md_info.suid),
        &((sns_sensor_uid)MOTION_DETECT_SUID_1),
        sizeof(state->md_info.suid));
    sns_memscpy(&state->sensor_temp_info.suid,
        sizeof(state->sensor_temp_info.suid),
        &((sns_sensor_uid)SENSOR_TEMPERATURE_SUID_1),
        sizeof(state->sensor_temp_info.suid));
#endif
  }
  sns_memscpy(&state->timer_suid,
      sizeof(state->timer_suid),
      &(inst_cfg->timer_suid),
      sizeof(inst_cfg->timer_suid));
  /* Create MD and heart-beat timer data streams to save power */
  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
      this,
      state->timer_suid,
      &state->timer_md_data_stream);

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
      this,
      state->timer_suid,
      &state->timer_heart_beat_data_stream);

  state->int_enabled = false;
 
  /**-------------------------Init FIFO State-------------------------*/
  state->fifo_info.fifo_rate = LSM6DSO_ACCEL_ODR_OFF;
  state->fifo_info.th_info.is_dae_ts_reliable = true;
  state->fifo_info.bh_info.is_dae_ts_reliable = true;
  state->fifo_info.gyro_extra_sample = false;

  /**-------------------------Init Accel State-------------------------*/
  state->common_info.accel_curr_odr = LSM6DSO_ACCEL_ODR_OFF;
  state->accel_info.sstvt = lsm6dso_accel_resolutions[inst_cfg->accel_resolution_idx]*1000;  //convert to micro-g/LSB
  state->accel_info.range = lsm6dso_accel_ranges[inst_cfg->accel_resolution_idx];
  state->accel_info.range_idx = inst_cfg->accel_resolution_idx;
  state->accel_info.bw = LSM6DSO_ACCEL_BW50;
  state->accel_info.lp_mode = false;
  state->accel_info.sample.opdata_status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  lsm6dso_copy_calibration_info(&state->accel_registry_cfg, &inst_cfg->accel_cal);

  /**-------------------------Init Gyro State-------------------------*/
  state->common_info.gyro_curr_odr = LSM6DSO_GYRO_ODR_OFF;
  state->cur_odr_change_info.changed = 0;
  state->prev_odr_change_info.changed = 0;
  state->cur_odr_change_info.gyro_startup = false;
  state->prev_odr_change_info.gyro_startup = false;
  state->cur_odr_change_info.odr_idx = 0;
  state->cur_odr_change_info.change_req = 0;
  state->cur_odr_change_info.accel_odr_settime = 0;
  state->cur_odr_change_info.gyro_odr_settime = 0;

  state->gyro_info.sstvt = lsm6dso_gyro_resolutions[inst_cfg->gyro_resolution_idx];
  state->gyro_info.range = lsm6dso_gyro_ranges[inst_cfg->gyro_resolution_idx];
  state->gyro_info.range_idx = inst_cfg->gyro_resolution_idx;
  state->gyro_info.sample.opdata_status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  state->gyro_info.reset_filter = false;
  state->gyro_info.sw_lpf_data_settled = false;
  lsm6dso_copy_calibration_info(&state->gyro_registry_cfg, &inst_cfg->gyro_cal);

  /**-------------------------Init MD State-------------------------*/
  state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
  state->md_info.client_present = false;
  state->md_info.is_timer_running = false;
  state->md_info.add_request = false;
  state->md_info.event_ts = sns_get_system_time();
  state->fifo_info.new_config.md_timestamp = sns_get_system_time();
  sns_memscpy(&state->md_info.md_config, sizeof(state->md_info.md_config),
              &inst_cfg->md_config, sizeof(inst_cfg->md_config));

  /**-------------------------Init Temperature State-------------------------*/
  lsm6dso_copy_calibration_info(&state->sensor_temp_registry_cfg, &inst_cfg->temper_cal);

  /**-------------------------Init Current Configure State-------------------------*/
  state->current_conf.md_enabled = false;

  /**-------------------------Init Desired Configure State-------------------------*/
  state->desired_conf.md_enabled = false;

  /**-------------------------Init Self Test State-------------------------*/
  state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_1;
  state->self_test_info.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
  state->self_test_info.update_registry = false;
  state->self_test_info.test_alive = false;
  state->self_test_info.reconfig_postpone = false;

  state->health.heart_attack = false;
  state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
  state->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, 1);

  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  state->scp_service =  (sns_sync_com_port_service*)
      service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  sns_busy_wait(sns_convert_ns_to_ticks(1000*1000));
  lsm6dso_reset_device( this, LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_MOTION_DETECT | LSM6DSO_SENSOR_TEMP);
  /** Initialize IRQ info to be used by the Instance */
  state->irq_info.irq_config         = inst_cfg->irq_config;
  state->irq_info.irq_registered     = false;
  state->irq_info.irq_ready          = false;
  state->route_md_to_irq2            = false;
  //state->enable_md_with_ngated_accel = false;
  if(shared_state->irq2_enabled)
  {
    state->irq2_inst_enabled         = true;
    state->irq2_info.irq_config      = inst_cfg->irq2_config;
    state->irq2_info.irq_registered  = false;
    state->irq2_info.irq_ready       = false;
  }
  else
  {
    state->irq2_inst_enabled         = false;
  }

  /** Initialize com config to be used by the Instance */
  {
    sns_com_port_config const *com_config = &inst_cfg->com_port_info.com_config;
    state->ascp_config.bus_type          = (sns_async_com_port_bus_type)com_config->bus_type;
    state->ascp_config.slave_control     = com_config->slave_control;
    state->ascp_config.reg_addr_type     = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
    state->ascp_config.min_bus_speed_kHz = com_config->min_bus_speed_KHz;
    state->ascp_config.max_bus_speed_kHz = com_config->max_bus_speed_KHz;
    state->ascp_config.bus_instance      = com_config->bus_instance;
  }

  /** Configure the Async Com Port */
  {
    sns_data_stream* data_stream = state->async_com_port_data_stream;
    uint8_t pb_encode_buffer[100];
    sns_request async_com_port_request =
    {
      .message_id  = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
      .request     = &pb_encode_buffer
    };

    async_com_port_request.request_len =
      pb_encode_request(pb_encode_buffer,
                        sizeof(pb_encode_buffer),
                        &state->ascp_config,
                        sns_async_com_port_config_fields,
                        NULL);
    data_stream->api->send_request(data_stream, &async_com_port_request);
  }

  /** Copy down axis conversion settings */
  sns_memscpy(state->axis_map,  sizeof(state->axis_map),
              inst_cfg->axis_map, sizeof(inst_cfg->axis_map));

  /** init s4s structure */
  state->s4s_info.sync_state = LSM6DSO_S4S_OFF;

#if LSM6DSO_ESP_XSENSOR
  state->xgroup_info.is_gyro_req = 0;
#endif
  lsm6dso_init_esp_instance(this, sstate);
  lsm6dso_turn_on_bus_power(state, false);

  lsm6dso_init_ois_instance(this, inst_cfg);
  //store gyro calibration data into ois info
  lsm6dso_store_ois_registry_data(this,sstate);

  lsm6dso_dae_if_init(this, stream_mgr, inst_cfg);

  if(!lsm6dso_dae_if_available(this))
  {
    lsm6dso_register_interrupt(this, &state->irq_info, state->interrupt_data_stream);
    if(state->irq2_inst_enabled)
      lsm6dso_register_interrupt(this, &state->irq2_info, state->interrupt2_data_stream);
  }
  lsm6dso_init_raw_log_info(this);
#if LSM6DSO_USE_OEM_TS_OFFSET
  state->oem_ts_offset = lsm6dso_get_oem_ts_offset();
#endif
  return SNS_RC_SUCCESS;
}

sns_rc lsm6dso_inst_deinit(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  DBG_INST_PRINTF_EX(HIGH, this, "inst_deinit");
  inst_cleanup(this, state);

  return SNS_RC_SUCCESS;
}

void lsm6dso_set_client_test_config(
  sns_sensor_instance *this,
  sns_request const *client_request)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_instance_config const *inst_cfg =
    (lsm6dso_instance_config const*)client_request->request;

  // 1. Extract test type from client_request.
  // 2. Configure sensor HW for test type.
  // 3. send_request() for Timer Sensor in case test needs polling/waits.
  // 4. Factory test is TBD.
  if(inst_cfg->selftest.test_type != SNS_PHYSICAL_SENSOR_TEST_TYPE_SW)
  {
    state->common_info.mode |= LSM6DSO_MODE_SELF_TEST;
    state->self_test_info.sensor = inst_cfg->selftest.sensor;
    state->self_test_info.test_type = inst_cfg->selftest.test_type;
    state->self_test_info.test_alive = true;
    DBG_INST_PRINTF_EX(HIGH, this, "Self test start, type=%u sensor=%u",
                    state->self_test_info.test_type, state->self_test_info.sensor);

    //if md was running before and disable event was sent before, send disable event
    if(inst_cfg->client_present & LSM6DSO_MOTION_DETECT)
    {
      state->md_info.client_present = (inst_cfg->client_present & LSM6DSO_MOTION_DETECT);
      lsm6dso_send_config_event(this, true);
      state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
      state->md_info.event_ts = sns_get_system_time();
      lsm6dso_send_md_event(this, &state->md_info.cur_state, state->md_info.event_ts);
    }
    //start HW self test
    if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
    {
      state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_1;
      lsm6dso_inst_hw_self_test(this);
    }
    else if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      state->self_test_info.self_test_stage = LSM6DSO_SELF_TEST_STAGE_1;
      lsm6dso_inst_factory_self_test(this);
    }
    else // must be SNS_PHYSICAL_SENSOR_TEST_TYPE_COM
    {
      lsm6dso_inst_com_self_test(this);
    }
  }
  else // this should never happen as the parents would not have forwarded bad request
  {
    DBG_INST_PRINTF_EX(HIGH, this, "Unsupported test type = %d", state->self_test_info.test_type);
    state->self_test_info.test_alive = false;
  }
}
