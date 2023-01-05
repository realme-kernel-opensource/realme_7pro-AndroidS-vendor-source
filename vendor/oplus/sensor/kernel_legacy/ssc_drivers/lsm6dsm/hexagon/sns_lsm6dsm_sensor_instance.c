/**
 * @file sns_lsm6dsm_sensor_instance.c
 *
 * LSM6DSM Accel virtual Sensor Instance implementation.
 *
 * Copyright (c) 2018-2019, STMicroelectronics.
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

#include "sns_lsm6dsm_hal.h"
#include "sns_lsm6dsm_sensor.h"
#include "sns_lsm6dsm_sensor_instance.h"

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
#include "float.h"
/**
 * Accelerometer LSM6DSM hardware self test settings
 */
#define LSM6DSM_ST_2G_MIN       90*G/1000.0f      //Unit: m/s2
#define LSM6DSM_ST_2G_MAX       1700*G/1000.0f    //Unit: m/s2

/**
 * Gyroscope LSM6DSM self test settings
 */
#define LSM6DSM_2000DPS_ST_MIN      150  //unit dps.
#define LSM6DSM_2000DPS_ST_MAX      700
#define ACC_CONV          ((LSM6DSM_ACCEL_SSTVT_2G * G)/1000000)
#define GYRO_CONV         LSM6DSM_GYRO_SSTVT_125DPS
#define LSM6DSM_ACCEL_FS_MASK       0x0C
#define LSM6DSM_GYRO_FS_MASK        0x0E
extern const odr_reg_map lsm6dsm_odr_map[];
extern const uint32_t lsm6dsm_odr_map_len;

#if LSM6DSM_DAE_ENABLED
static void lsm6dsm_flush_dae_data(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  uint64_t max_requested_flush_ticks = state->fifo_info.max_requested_flush_ticks;

  //Flush old data from DAE if already streaming
  state->fifo_info.max_requested_flush_ticks = 0;
  lsm6dsm_dae_if_start_streaming(this);
  lsm6dsm_dae_if_flush_hw(this);

  state->fifo_info.max_requested_flush_ticks = max_requested_flush_ticks;
}
#endif

static sns_rc lsm6dsm_set_accel_config_fifo(
  sns_sensor_instance *const instance,
  lsm6dsm_accel_odr    curr_odr,
  lsm6dsm_accel_sstvt  sstvt,
  lsm6dsm_accel_range  range,
  lsm6dsm_accel_bw     bw)
{

  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  bool is_md_force_disabled = false;
  //disable md if enabled and is required
  //Avoiding spurious interrupts
  if(state->current_conf.md_enabled && lsm6dsm_is_md_int_required(instance)) {
    lsm6dsm_disable_md(instance, false);
    is_md_force_disabled = true;
  }

  // set fifo bypass mode
  lsm6dsm_set_fifo_bypass_mode(instance);

#if LSM6DSM_DAE_ENABLED
  if(lsm6dsm_dae_if_available(instance)) {
    lsm6dsm_flush_dae_data(instance);
    lsm6dsm_dae_if_start_streaming(instance);
  }
#endif

  //Set accel config
  rv = lsm6dsm_set_accel_config(instance,curr_odr,sstvt,range,bw);
  if(rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->accel_info.num_samples_to_discard =
    lsm6dsm_odr_map[lsm6dsm_get_odr_rate_idx(curr_odr)].accel_discard_samples;
  state->common_info.odr_changed |= LSM6DSM_ACCEL;

  if(!state->self_test_info.reconfig_postpone)
    lsm6dsm_set_fifo_wmk(instance);

  //start streaming,stream mode
  lsm6dsm_set_fifo_stream_mode(instance);

  //re-enable if force disabled before
  if(is_md_force_disabled) {
    lsm6dsm_enable_md(instance, false);
  }

  state->fifo_info.last_timestamp = sns_get_system_time();
  return rv;
}

sns_rc lsm6dsm_set_gyro_config_fifo(
  sns_sensor_instance *const instance,
  lsm6dsm_gyro_odr     curr_odr,
  lsm6dsm_gyro_sstvt   sstvt,
  lsm6dsm_gyro_range   range)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  // set fifo bypass mode
  lsm6dsm_set_fifo_bypass_mode(instance);

#if LSM6DSM_DAE_ENABLED
  if(lsm6dsm_dae_if_available(instance)) {
    lsm6dsm_flush_dae_data(instance);
    lsm6dsm_dae_if_start_streaming(instance);
  }
#endif

  //Set gyro config
  rv = lsm6dsm_set_gyro_config(instance,curr_odr,sstvt,range);
  if(rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->gyro_info.num_samples_to_discard  =
    lsm6dsm_odr_map[lsm6dsm_get_odr_rate_idx(curr_odr)].gyro_discard_samples;
  state->common_info.odr_changed |= LSM6DSM_GYRO;

  if(!state->self_test_info.reconfig_postpone)
    lsm6dsm_set_fifo_wmk(instance);

  //start streaming,stream mode
  lsm6dsm_set_fifo_stream_mode(instance);

  state->fifo_info.last_timestamp = sns_get_system_time();
  return rv;
}

static void enable_hw_self_test(
  sns_sensor_instance *const this,
  lsm6dsm_sensor_type sensor,
  bool enable)
{
  uint8_t buffer;
  uint32_t xfer_bytes;

  if(enable == false)
  {
    buffer = 0x00;
  }
  else if(sensor == LSM6DSM_ACCEL)
  {
    buffer = 0x01;
  }
  else //enable bit for gyro
  {
    buffer = 0x04;
  }
  lsm6dsm_read_modify_write(this,
                            STM_LSM6DSM_REG_CTRL5,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x00);

}

static void lsm6dsm_inst_collect_data(sns_sensor_instance *const this, int64_t *data)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  lsm6dsm_sensor_type sensor[1];
  int16_t raw_data[3] = {0,0,0};
  int8_t idx;
  lsm6dsm_accel_odr curr_odr = LSM6DSM_ACCEL_ODR52;
  uint16_t self_test_odr = LSM6DSM_ODR_52;

  //handle case were new streaming ODR is less than self test ODR
  if((state->self_test_info.curr_odr >  state->common_info.accel_curr_odr) &&
     (state->common_info.accel_curr_odr != LSM6DSM_ACCEL_ODR_OFF))
  {
    state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
  }

  for(idx = 0; idx < lsm6dsm_odr_map_len; idx++)
  {
    if(curr_odr == lsm6dsm_odr_map[idx].accel_odr_reg_value
       &&
       curr_odr != LSM6DSM_ACCEL_ODR_OFF)
    {
      self_test_odr = lsm6dsm_odr_map[idx].odr;
      break;
    }
  }
  sensor[0] = state->self_test_info.sensor;

  //Check if any samples need to be dropped
  if(state->self_test_info.skip_count)
  {
    lsm6dsm_get_data(this,sensor,1,raw_data);
    DBG_INST_PRINTF_EX(LOW, this, "Discarding the first %u samples", state->self_test_info.skip_count);
    sns_time timeout = sns_convert_ns_to_ticks(((1000 * 1000)/self_test_odr) * 1000 * state->self_test_info.skip_count);
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout;
    lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); // Skip this sample as it may be invalid
    state->self_test_info.skip_count=0;
    return;
  }

  //Poll data for 25 samples, save total
  lsm6dsm_get_data(this,sensor,1,raw_data);
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
    lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); // wait for next odr to read the next sample
  }
}


static void send_hw_self_test_result(sns_sensor_instance *const instance)
{

  bool test_passed = false;
  lsm6dsm_instance_state *state =
     (lsm6dsm_instance_state*)instance->state->state;
  sns_sensor_uid *suid_current;
  float data_conv;
  int32_t mean_diff_i[3]={0,0,0};
  float abs_mean_diff_f[3]={0,0,0};

  //Compare the two averages - copy from ddf
  //Check self test limits
  if(state->self_test_info.sensor == LSM6DSM_ACCEL)
  {
    suid_current = &state->accel_info.suid;
    data_conv = LSM6DSM_ACCEL_SSTVT_HW_SELFTEST * G/1000000;
    mean_diff_i[0]=abs((int32_t)state->self_test_info.cumulative_data_post[0]-(int32_t)state->self_test_info.cumulative_data_pre[0]);
    mean_diff_i[1]=abs((int32_t)state->self_test_info.cumulative_data_post[1]-(int32_t)state->self_test_info.cumulative_data_pre[1]);
    mean_diff_i[2]=abs((int32_t)state->self_test_info.cumulative_data_post[2]-(int32_t)state->self_test_info.cumulative_data_pre[2]);
    abs_mean_diff_f[0]=mean_diff_i[0]*data_conv;
    abs_mean_diff_f[1]=mean_diff_i[1]*data_conv;
    abs_mean_diff_f[2]=mean_diff_i[2]*data_conv;
    if( (LSM6DSM_ST_2G_MIN <= abs_mean_diff_f[0]) && (abs_mean_diff_f[0] <= LSM6DSM_ST_2G_MAX)
      &&(LSM6DSM_ST_2G_MIN <= abs_mean_diff_f[1]) && (abs_mean_diff_f[1] <= LSM6DSM_ST_2G_MAX)
      &&(LSM6DSM_ST_2G_MIN <= abs_mean_diff_f[2]) && (abs_mean_diff_f[2] <= LSM6DSM_ST_2G_MAX))
      test_passed = true;
    else
      test_passed = false;
  }
  else //gyro
  {
    suid_current = &state->gyro_info.suid;
    data_conv = LSM6DSM_GYRO_SSTVT_2000DPS;
    mean_diff_i[0]=abs((int32_t)state->self_test_info.cumulative_data_post[0]-(int32_t)state->self_test_info.cumulative_data_pre[0]);
    mean_diff_i[1]=abs((int32_t)state->self_test_info.cumulative_data_post[1]-(int32_t)state->self_test_info.cumulative_data_pre[1]);
    mean_diff_i[2]=abs((int32_t)state->self_test_info.cumulative_data_post[2]-(int32_t)state->self_test_info.cumulative_data_pre[2]);
    abs_mean_diff_f[0]=mean_diff_i[0]*data_conv;
    abs_mean_diff_f[1]=mean_diff_i[1]*data_conv;
    abs_mean_diff_f[2]=mean_diff_i[2]*data_conv;
    if( (LSM6DSM_2000DPS_ST_MIN <= abs_mean_diff_f[0]) && (abs_mean_diff_f[0] <= LSM6DSM_2000DPS_ST_MAX)
      &&(LSM6DSM_2000DPS_ST_MIN <= abs_mean_diff_f[1]) && (abs_mean_diff_f[1] <= LSM6DSM_2000DPS_ST_MAX)
      &&(LSM6DSM_2000DPS_ST_MIN <= abs_mean_diff_f[2]) && (abs_mean_diff_f[2] <= LSM6DSM_2000DPS_ST_MAX))
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

  lsm6dsm_instance_state *state =
     (lsm6dsm_instance_state*)instance->state->state;

  DBG_INST_PRINTF(MED, instance, "Self Test(factory) Result=%d", test_passed);

  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_sensor_uid *suid_current;

  //update suid
  if(state->self_test_info.sensor == LSM6DSM_ACCEL)
  {
    suid_current = &state->accel_info.suid;
  }
#if LSM6DSM_ESP_ENABLED
  else if(state->self_test_info.sensor == LSM6DSM_STEP_COUNTER)
  {
    suid_current = &state->esp_info.suid[SC_INDX];
  }
#endif
  else
  {
    suid_current = &state->gyro_info.suid;
  }

  if(test_passed)
    lsm6dsm_send_cal_event(instance, state->self_test_info.sensor);

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

  lsm6dsm_instance_state *state =
     (lsm6dsm_instance_state*)instance->state->state;

  DBG_INST_PRINTF(MED, instance, "Self Test(com) Result=%d", test_passed);

  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_sensor_uid *suid_current;

  //update suid
  if(state->self_test_info.sensor == LSM6DSM_GYRO)
  {
    suid_current = &state->gyro_info.suid;
  }
  else if(state->self_test_info.sensor == LSM6DSM_MOTION_DETECT)
  {
    suid_current = &state->md_info.suid;
  }
  else if(state->self_test_info.sensor == LSM6DSM_SENSOR_TEMP)
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

void lsm6dsm_inst_hw_self_test(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)this->state->state;
  uint32_t timeout = 0;
  lsm6dsm_accel_odr curr_odr = LSM6DSM_ACCEL_ODR52;

  switch (state->self_test_info.self_test_stage)
  {
    case LSM6DSM_SELF_TEST_STAGE_1:
    {
      //Check if sensor is already streaming. If not, turn it ON
      if((state->common_info.accel_curr_odr) && (state->common_info.accel_curr_odr < LSM6DSM_ACCEL_ODR832))
      {
        state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
        curr_odr = state->self_test_info.curr_odr;
      }
      else
      {
        state->self_test_info.curr_odr = LSM6DSM_ACCEL_ODR52;
      }

      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
      {
        if((state->gyro_info.desired_odr) && !(state->accel_info.desired_odr))
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        if(!state->accel_info.desired_odr)
        {
          lsm6dsm_set_accel_config(this,curr_odr,LSM6DSM_ACCEL_SSTVT_HW_SELFTEST,LSM6DSM_ACCEL_RANGE_HW_SELFTEST,state->accel_info.bw);
        }
        else
        {
          lsm6dsm_set_accel_config_fifo(this,curr_odr,LSM6DSM_ACCEL_SSTVT_HW_SELFTEST,LSM6DSM_ACCEL_RANGE_HW_SELFTEST,state->accel_info.bw);
        }
        timeout = 200;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dsm_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == lsm6dsm_odr_map[state->self_test_info.odr_idx].accel_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
            break;
          }
        }
      }
      else
      {
        if((state->accel_info.desired_odr) && !(state->gyro_info.desired_odr))
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        if(!state->gyro_info.desired_odr)
        {
          lsm6dsm_set_gyro_config(this,(lsm6dsm_gyro_odr)curr_odr,LSM6DSM_GYRO_SSTVT_2000DPS,STM_LSM6DSM_GYRO_RANGE_2000DPS);
        }
        else
        {
          lsm6dsm_set_gyro_config_fifo(this,(lsm6dsm_gyro_odr)curr_odr,LSM6DSM_GYRO_SSTVT_2000DPS,STM_LSM6DSM_GYRO_RANGE_2000DPS);
        }
        timeout = 300;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dsm_odr_map_len; state->self_test_info.odr_idx++)
        {
          if((lsm6dsm_gyro_odr)state->self_test_info.curr_odr == lsm6dsm_odr_map[state->self_test_info.odr_idx].gyro_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
            break;
          }
        }
      }
      state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_2;

      lsm6dsm_dump_reg(this, state->fifo_info.fifo_enabled);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); //Settling time for ODR
    }
    break;

    case LSM6DSM_SELF_TEST_STAGE_2:
    {
      lsm6dsm_inst_collect_data(this, state->self_test_info.cumulative_data_pre);
    }
    if(state->self_test_info.self_test_stage == LSM6DSM_SELF_TEST_STAGE_2)
      break;

    case LSM6DSM_SELF_TEST_STAGE_3:
    {
      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
      {
        timeout = 100;
        state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
      }
      else //gyro
      {
        timeout = 50;
        state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
      }
      //Set bit for self test
      enable_hw_self_test(this, state->self_test_info.sensor, true);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload);
      state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_4;
    }
    break;

    case LSM6DSM_SELF_TEST_STAGE_4:
    {
      lsm6dsm_inst_collect_data(this, state->self_test_info.cumulative_data_post);
    }
    if(state->self_test_info.self_test_stage == LSM6DSM_SELF_TEST_STAGE_4)
      break;
    if(state->self_test_info.self_test_stage == LSM6DSM_SELF_TEST_STAGE_5)
    {
      //Disable Self test bit
      enable_hw_self_test(this, state->self_test_info.sensor, false);
      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
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
      lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload);
      break;
    }

    case LSM6DSM_SELF_TEST_STAGE_5:
    {
      send_hw_self_test_result(this);
      //Disable streaming if started for self test else set the original ODR is more than 832Hz
      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
      {
        if(!state->accel_info.desired_odr)
        {
          lsm6dsm_set_accel_config(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        }
        else
        {
          lsm6dsm_set_accel_config_fifo(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        }
        if(state->gyro_info.desired_odr)
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        if((state->gyro_info.desired_odr) && (!state->accel_info.desired_odr))
        {
          state->gyro_info.num_samples_to_discard +=
            lsm6dsm_odr_map[state->desired_conf.odr_idx].gyro_discard_samples;
        }
      }
      else if(state->self_test_info.sensor == LSM6DSM_GYRO)
      {
        if(!state->gyro_info.desired_odr)
        {
          lsm6dsm_set_gyro_config(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
        }
        else
        {
          lsm6dsm_set_gyro_config_fifo(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
        }
        if(state->accel_info.desired_odr)
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr))
        {
          state->accel_info.num_samples_to_discard += lsm6dsm_odr_map[state->desired_conf.odr_idx].accel_discard_samples;
        }
      }
      sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_self_test_data_stream);
      state->common_info.mode &= ~LSM6DSM_MODE_SELF_TEST;
      //Disable all self test flags
      state->self_test_info.polling_count = 0;
      state->self_test_info.curr_odr = LSM6DSM_ACCEL_ODR52;
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

void lsm6dsm_inst_factory_self_test(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  uint32_t timeout = 0;
  lsm6dsm_accel_odr curr_odr = LSM6DSM_ACCEL_ODR52;
  float fac_cal_bias[3] = {0};
  float bias_thresholds[3] = {0};
  int64_t variance[3] = {0};
  int64_t variance_threshold = 0;

  switch (state->self_test_info.self_test_stage)
  {
    case LSM6DSM_SELF_TEST_STAGE_1:
    {
#if LSM6DSM_OEM_FACTORY_CONFIG
    //Enable factory test specific OEM configuration
    lsm6dsm_oem_factory_test_config(this, true);
    if(state->self_test_info.return_now)
    {
      state->self_test_info.return_now = false;
      send_factory_self_test_result(this, true);
      return;
    }
#endif
      //Check if sensor is already streaming. If not, turn it ON
      if((state->common_info.accel_curr_odr) && (state->common_info.accel_curr_odr < LSM6DSM_ACCEL_ODR832))
      {
        state->self_test_info.curr_odr =  state->common_info.accel_curr_odr;
        curr_odr = state->self_test_info.curr_odr;
      }
      else
      {
        state->self_test_info.curr_odr = LSM6DSM_ACCEL_ODR52;
      }

      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
      {
        if((state->gyro_info.desired_odr) && !(state->accel_info.desired_odr))
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        lsm6dsm_set_accel_config(this,curr_odr,LSM6DSM_ACCEL_SSTVT_2G,LSM6DSM_ACCEL_RANGE_2G,state->accel_info.bw);
        timeout = 200;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dsm_odr_map_len; state->self_test_info.odr_idx++)
        {
          if(state->self_test_info.curr_odr == lsm6dsm_odr_map[state->self_test_info.odr_idx].accel_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].accel_discard_samples;
            break;
          }
        }
      }
      else
      {
        if((state->accel_info.desired_odr) && !(state->gyro_info.desired_odr))
        {
          lsm6dsm_set_fifo_wmk(this);
        }
        lsm6dsm_set_gyro_config(this,(lsm6dsm_gyro_odr)curr_odr,LSM6DSM_GYRO_SSTVT_125DPS,STM_LSM6DSM_GYRO_RANGE_125DPS);
        timeout = 300;
        for(state->self_test_info.odr_idx = 0; state->self_test_info.odr_idx < lsm6dsm_odr_map_len; state->self_test_info.odr_idx++)
        {
          if((lsm6dsm_gyro_odr)state->self_test_info.curr_odr == lsm6dsm_odr_map[state->self_test_info.odr_idx].gyro_odr_reg_value)
          {
            state->self_test_info.skip_count = lsm6dsm_odr_map[state->self_test_info.odr_idx].gyro_discard_samples;
            break;
          }
        }
      }
      state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_2;

      lsm6dsm_dump_reg(this, state->fifo_info.fifo_enabled);
      sns_time timeout_ticks = sns_convert_ns_to_ticks(timeout * 1000 * 1000);
      sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
      req_payload.is_periodic = false;
      req_payload.start_time = sns_get_system_time();
      req_payload.timeout_period = timeout_ticks;
      lsm6dsm_inst_create_timer(this, &state->timer_self_test_data_stream, &req_payload); //Settling time for ODR
    }
    break;

    case LSM6DSM_SELF_TEST_STAGE_2:
    {
      lsm6dsm_inst_collect_data(this, state->self_test_info.cumulative_data_pre);
    }
    if(state->self_test_info.self_test_stage == LSM6DSM_SELF_TEST_STAGE_2)
      break;

    case LSM6DSM_SELF_TEST_STAGE_3:
    {
      int i;
      bool test_pass=true;

      DBG_INST_PRINTF(HIGH, this, "Sample Sums: %d/%d/%d",
                      state->self_test_info.cumulative_data_pre[0],
                      state->self_test_info.cumulative_data_pre[1],
                      state->self_test_info.cumulative_data_pre[2]);
      DBG_INST_PRINTF(HIGH, this, "Sample square sums: %d/%d/%d",
                      state->self_test_info.cumulative_data_post[0],
                      state->self_test_info.cumulative_data_post[1],
                      state->self_test_info.cumulative_data_post[2]);

#if LSM6DSM_USE_FIXED_ORIENTATION
      if(state->self_test_info.sensor == LSM6DSM_ACCEL)
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

        variance_threshold = 1073741823; //2^15^2-1

        if(state->self_test_info.sensor == LSM6DSM_ACCEL)
        {
#if !LSM6DSM_USE_FIXED_ORIENTATION
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
        }
        else if(state->self_test_info.sensor == LSM6DSM_GYRO)
        {
          fac_cal_bias[i] = (float)(state->self_test_info.cumulative_data_pre[i] * GYRO_CONV * PI) / (180.0f);
          bias_thresholds[i] = 40 * PI / 180; //40 rad/sec
        }

        // Check variance to determine whether device is stationary
        if(variance[i] == 0)
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
        if(variance[i] == FLT_MIN)
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
        if(state->self_test_info.sensor == LSM6DSM_ACCEL)
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
        else if(state->self_test_info.sensor == LSM6DSM_GYRO)
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
      //Revert FS if reconfig_postpone.
      //This is needed for a change where in reconfig_postpone no HW reconfiguration is needed.
      if(state->self_test_info.reconfig_postpone)
      {
        if(state->common_info.accel_curr_odr)
          lsm6dsm_set_accel_config_fifo(this,state->self_test_info.curr_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
        if(state->common_info.gyro_curr_odr)
          lsm6dsm_set_gyro_config_fifo(this,(lsm6dsm_gyro_odr)state->self_test_info.curr_odr,state->gyro_info.sstvt,state->gyro_info.range);
      }
      //Disable streaming if started for self test else set the original ODR if more than 833Hz
      if(state->self_test_info.sensor == LSM6DSM_ACCEL &&
         !state->self_test_info.reconfig_postpone)
      {
        if(!(state->accel_info.desired_odr) || (state->accel_info.desired_odr > LSM6DSM_ACCEL_ODR832) || (state->accel_info.range != LSM6DSM_ACCEL_RANGE_2G))
        {
          if(!state->accel_info.desired_odr)
          {
            lsm6dsm_set_accel_config(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
            if(state->gyro_info.desired_odr)
            {
              lsm6dsm_set_fifo_wmk(this);
            }
          }
          else
          {
            lsm6dsm_set_accel_config_fifo(this,state->accel_info.desired_odr,state->accel_info.sstvt,state->accel_info.range,state->accel_info.bw);
          }
        }
        if((state->gyro_info.desired_odr) && (!state->accel_info.desired_odr))
        {
          state->gyro_info.num_samples_to_discard += lsm6dsm_odr_map[state->desired_conf.odr_idx].gyro_discard_samples;
        }
      }
      else if(state->self_test_info.sensor == LSM6DSM_GYRO &&
              !state->self_test_info.reconfig_postpone)
      {
        if(!(state->gyro_info.desired_odr) || (state->gyro_info.desired_odr > LSM6DSM_ACCEL_ODR832) || (state->gyro_info.range != STM_LSM6DSM_GYRO_RANGE_125DPS))
        {
          if(!state->gyro_info.desired_odr)
          {
            lsm6dsm_set_gyro_config(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
            if(state->accel_info.desired_odr)
            {
              lsm6dsm_set_fifo_wmk(this);
            }
          }
          else
          {
            lsm6dsm_set_gyro_config_fifo(this,state->gyro_info.desired_odr,state->gyro_info.sstvt,state->gyro_info.range);
          }
        }
        if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr))
        {
          state->accel_info.num_samples_to_discard += lsm6dsm_odr_map[state->desired_conf.odr_idx].accel_discard_samples;
        }
      }
      sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_self_test_data_stream);

      state->common_info.mode &= ~LSM6DSM_MODE_SELF_TEST;
      //Disable all self test flags
      state->self_test_info.polling_count = 0;
      state->self_test_info.curr_odr = LSM6DSM_ACCEL_ODR52;
      state->health.heart_attack = false;
      sns_memset(state->self_test_info.cumulative_data_pre, 0, sizeof(state->self_test_info.cumulative_data_pre));
      sns_memset(state->self_test_info.cumulative_data_post, 0, sizeof(state->self_test_info.cumulative_data_post));

      //lsm6dsm_dae_if_start_streaming(this, state->self_test_info.sensor);
    }
    default:
    {
    }
    break;
  }

}

void lsm6dsm_inst_com_self_test(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t buffer = 0;
  bool who_am_i_success = false;

  rv = lsm6dsm_get_who_am_i(state->scp_service,
                            state->com_port_info.port_handle,
                            &buffer);

  if(rv == SNS_RC_SUCCESS
     &&
     buffer == LSM6DSM_WHOAMI_VALUE)
  {
    who_am_i_success = true;
  }

  //Send result
  send_com_self_test_result(this, who_am_i_success);
  state->common_info.mode &= ~LSM6DSM_MODE_SELF_TEST;
  //Disable all self test flags
  state->self_test_info.polling_count = 0;
  state->self_test_info.curr_odr = LSM6DSM_ACCEL_ODR52;
}

static void inst_cleanup(sns_sensor_instance *const this,
                         lsm6dsm_instance_state *state)
{
  SNS_INST_PRINTF(HIGH, this, "[%u] inst_cleanup: #samples=%u/%u/%u/%u",
                  state->hw_idx, state->accel_sample_counter, state->gyro_sample_counter,
                  state->num_temp_samples, state->num_md_ints);

  if(NULL != state->com_port_info.port_handle)
  {
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);
  }
  lsm6dsm_set_fifo_config(this, 0, 0, 0, 0 );
  lsm6dsm_reconfig_hw(this);
  if(NULL != state->com_port_info.port_handle)
  {
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
  }

  sns_sensor_util_remove_sensor_instance_stream(this, &state->interrupt2_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->interrupt_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->async_com_port_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_temp_data_stream);
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
  lsm6dsm_dae_if_deinit(this);
}

/** See sns_sensor_instance_api::init */
sns_rc lsm6dsm_inst_init(sns_sensor_instance *const this, sns_sensor_state const *sstate)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  lsm6dsm_shared_state *shared_state = lsm6dsm_get_shared_state_from_state(sstate);
  lsm6dsm_instance_config const *inst_cfg = &shared_state->inst_cfg;
  float data[3];
  float temp_data[1];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
#if !LSM6DSM_LOGGING_DISABLED
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
#endif
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[LSM6DSM_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = LSM6DSM_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;
  //this would be stored when the state is initialized
  state->hw_idx = shared_state->hw_idx;
  state->rigid_body_type = shared_state->rigid_body_type;
  state->min_odr_idx = inst_cfg->min_odr_idx;

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

  state->timer_self_test_data_stream = NULL;
  state->timer_md_data_stream = NULL;
  state->timer_sensor_temp_data_stream = NULL;
  state->timer_heart_beat_data_stream = NULL;
  state->timer_polling_data_stream = NULL;
  state->ag_stream_mode = inst_cfg->ag_stream_mode;
  SNS_INST_PRINTF(HIGH, this, "[%u] inst_init:: ag_stream_mode:%d",
                  state->hw_idx, state->ag_stream_mode);
  /** Initialize COM port to be used by the Instance */
  sns_memscpy(&state->com_port_info,
              sizeof(state->com_port_info),
              &inst_cfg->com_port_info,
              sizeof(inst_cfg->com_port_info));
  state->com_port_info.port_handle = NULL;

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
  /** Some customer request to set OIS_PU_DIS on PIN_CTRL */
  //  The procedure to disable the pull-up on pins 10-11 is as follows:
  //1. AP side: write 80h in register at address 00h
  //2. AP side: write 01h in register at address 05h (disable the pull-up on pins 10 and 11 of LSM6DSM)
  //3. AP side: write 00h in register at address 00h
#if LSM6DSM_SET_OEM_OIS_PU_DIS
  uint8_t reg_buffer = 0x80;
  uint32_t xfer_bytes;
  // QC - won't this get reset after lsm6dsm_reset_device() is called?
  lsm6dsm_read_modify_write(this,
                            STM_LSM6DSM_SLAVE_ENABLE_REG,
                            &reg_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x80);

  reg_buffer = 0x01;
  lsm6dsm_read_modify_write(this,
                            STM_LSM6DSM_OIS_PU_DIS_REG,
                            &reg_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x01);

  reg_buffer = 0x00;
  lsm6dsm_read_modify_write(this,
                            STM_LSM6DSM_SLAVE_ENABLE_REG,
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
#if LSM6DSM_DUAL_SENSOR_ENABLED
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
  state->fifo_info.fifo_enabled = 0;
  state->fifo_info.inst_publish_sensors = 0;
  state->fifo_info.fifo_rate = LSM6DSM_ACCEL_ODR_OFF;
  state->fifo_info.cur_wmk = 0;
  state->fifo_info.max_requested_wmk = 0;
  state->fifo_info.interrupt_cnt = 0;
  state->fifo_info.timer_active = false;
  state->fifo_info.last_ts_valid = false;
  state->fifo_info.is_streaming = false;
  state->fifo_info.avg_to_nominal_ratio_cnt = -1;
  state->fifo_info.avg_to_nominal_ratio_cnt_g = -1;
  state->fifo_info.accel_ratio =
  state->fifo_info.avg_to_nominal_ratio = inst_cfg->avg_to_nominal_ratio;
  state->fifo_info.gyro_ratio =
  state->fifo_info.avg_to_nominal_ratio_g = inst_cfg->avg_to_nominal_ratio_g;
  state->accel_sample_counter = 0;
  state->gyro_sample_counter = 0;
  #if LSM6DSM_AUTO_DEBUG
    state->missingSamples = false;
  #endif

  /**-------------------------Init Accel State-------------------------*/
  state->common_info.accel_curr_odr = LSM6DSM_ACCEL_ODR_OFF;
  state->accel_info.sstvt = lsm6dsm_accel_resolutions[inst_cfg->accel_resolution_idx]*1000;  //convert to micro-g/LSB
  state->accel_info.range = lsm6dsm_accel_ranges[inst_cfg->accel_resolution_idx];
  state->accel_info.range_idx = inst_cfg->accel_resolution_idx;
  state->accel_info.bw = LSM6DSM_ACCEL_BW50;
  state->accel_info.lp_mode = false;
  sns_memset(state->accel_info.opdata_raw, 0, sizeof(state->accel_info.opdata_raw));
  state->accel_info.opdata_status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  lsm6dsm_copy_calibration_info(&state->accel_registry_cfg, &inst_cfg->accel_cal);

  /**-------------------------Init Gyro State-------------------------*/
  state->common_info.gyro_curr_odr = LSM6DSM_GYRO_ODR_OFF;
  state->gyro_info.sstvt = lsm6dsm_gyro_resolutions[inst_cfg->gyro_resolution_idx];
  sns_memscpy(state->gyro_info.sstvt_adj,  sizeof(state->gyro_info.sstvt_adj),
              inst_cfg->sstvt_adj, sizeof(inst_cfg->sstvt_adj));

  state->gyro_info.range = lsm6dsm_gyro_ranges[inst_cfg->gyro_resolution_idx];
  state->gyro_info.range_idx = inst_cfg->gyro_resolution_idx;
  sns_memset(state->gyro_info.opdata_raw, 0, sizeof(state->gyro_info.opdata_raw));
  state->gyro_info.opdata_status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  lsm6dsm_copy_calibration_info(&state->gyro_registry_cfg, &inst_cfg->gyro_cal);

  /**-------------------------Init MD State-------------------------*/
  state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
  state->md_info.internal_client_present = 0;
  state->md_info.client_present = false;
  state->md_info.is_timer_running = false;
  state->md_info.event_ts = sns_get_system_time();
  state->fifo_info.new_config.md_timestamp = sns_get_system_time();
  sns_memscpy(&state->md_info.md_config, sizeof(state->md_info.md_config),
              &inst_cfg->md_config, sizeof(inst_cfg->md_config));

  /**-------------------------Init Temperature State-------------------------*/
  lsm6dsm_copy_calibration_info(&state->sensor_temp_registry_cfg, &inst_cfg->temper_cal);

  /**-------------------------Init Current Configure State-------------------------*/
  state->current_conf.odr = 0;
  state->current_conf.fifo_odr = 0;
  state->current_conf.odr_idx = 0;
  state->current_conf.last_odr_idx = 0;
  state->current_conf.wmk = 0;
  state->current_conf.enabled_sensors = 0;
  state->current_conf.md_enabled = false;

  /**-------------------------Init Desired Configure State-------------------------*/
  state->desired_conf.odr = 0;
  state->desired_conf.fifo_odr = 0;
  state->desired_conf.odr_idx = 0;
  state->desired_conf.wmk = 0;
  state->desired_conf.enabled_sensors = 0;
  state->desired_conf.md_enabled = false;

  /**-------------------------Init Self Test State-------------------------*/
  sns_memset(state->self_test_info.cumulative_data_pre, 0, sizeof(state->self_test_info.cumulative_data_pre));
  sns_memset(state->self_test_info.cumulative_data_post, 0, sizeof(state->self_test_info.cumulative_data_post));
  state->self_test_info.polling_count = 0;
  state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_1;
  state->self_test_info.skip_count = 0;
  state->self_test_info.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
  state->self_test_info.update_registry = false;
  state->self_test_info.test_alive = false;
  state->self_test_info.reconfig_postpone = false;

  state->health.heart_attack = false;
  state->health.heart_attack_cnt = 0;
  state->health.heart_attack_flush = false;
  state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
  state->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, 1);

  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  state->scp_service =  (sns_sync_com_port_service*)
      service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  sns_busy_wait(sns_convert_ns_to_ticks(1000*1000));
  lsm6dsm_reset_device( this, LSM6DSM_ACCEL | LSM6DSM_GYRO | LSM6DSM_MOTION_DETECT | LSM6DSM_SENSOR_TEMP);

#if LSM6DSM_ESP_ENABLED
  lsm6dsm_esp_handle_reset_device(this, sstate);
#endif

  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);
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

  state->ascp_req_count = 0;
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

#if !LSM6DSM_LOGGING_DISABLED
  /** Determine sizes of encoded logs */
  sns_diag_sensor_state_interrupt sensor_state_interrupt =
        sns_diag_sensor_state_interrupt_init_default;
  pb_get_encoded_size(&state->log_interrupt_encoded_size,
                      sns_diag_sensor_state_interrupt_fields,
                      &sensor_state_interrupt);
#endif
  lsm6dsm_init_esp_instance(this, sstate);
  lsm6dsm_store_esp_registry_data(this, sstate);
#if !LSM6DSM_LOGGING_DISABLED
  /** Determine size of sns_diag_sensor_state_raw as defined in
   *  sns_diag.proto
   *  sns_diag_sensor_state_raw is a repeated array of samples of
   *  type sns_diag_batch sample. The following determines the
   *  size of sns_diag_sensor_state_raw with a single batch
   *  sample */
  if(pb_encode_tag(&stream, PB_WT_STRING,
                    sns_diag_sensor_state_raw_sample_tag))
  {
    if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                               &batch_sample))
    {
      state->log_raw_encoded_size = stream.bytes_written;
    }
  }
  //get the temp sensor log encoded size
  arg.arr_len = 1;
  if(pb_encode_tag(&stream, PB_WT_STRING, sns_diag_sensor_state_raw_sample_tag))
  {
    if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields, &batch_sample))
    {
      state->log_temp_raw_encoded_size = stream.bytes_written;
    }
  }
#endif
  lsm6dsm_dae_if_init(this, stream_mgr, inst_cfg);

  if(!lsm6dsm_dae_if_available(this))
  {
    lsm6dsm_register_interrupt(this, &state->irq_info, state->interrupt_data_stream);
    if(state->irq2_inst_enabled)
      lsm6dsm_register_interrupt(this, &state->irq2_info, state->interrupt2_data_stream);
  }
  state->oem_ts_offset = 0;
#if LSM6DSM_USE_OEM_TS_OFFSET
  state->oem_ts_offset = lsm6dsm_get_oem_ts_offset();
#endif
  return SNS_RC_SUCCESS;
}

sns_rc lsm6dsm_inst_deinit(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  DBG_INST_PRINTF_EX(HIGH, this, "inst_deinit");
  inst_cleanup(this, state);

  return SNS_RC_SUCCESS;
}

void lsm6dsm_set_client_test_config(
  sns_sensor_instance *this,
  sns_request const *client_request)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  lsm6dsm_instance_config const *inst_cfg =
    (lsm6dsm_instance_config const*)client_request->request;

  // 1. Extract test type from client_request.
  // 2. Configure sensor HW for test type.
  // 3. send_request() for Timer Sensor in case test needs polling/waits.
  // 4. Factory test is TBD.
  if(inst_cfg->selftest.test_type != SNS_PHYSICAL_SENSOR_TEST_TYPE_SW)
  {
    state->common_info.mode |= LSM6DSM_MODE_SELF_TEST;
    state->self_test_info.sensor = inst_cfg->selftest.sensor;
    state->self_test_info.test_type = inst_cfg->selftest.test_type;
    state->self_test_info.test_alive = true;
    DBG_INST_PRINTF_EX(HIGH, this, "Self test start, type=%u sensor=%u",
                    state->self_test_info.test_type, state->self_test_info.sensor);

    //start HW self test
    if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
    {
      state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_1;
      lsm6dsm_inst_hw_self_test(this);
    }
    else if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      state->self_test_info.self_test_stage = LSM6DSM_SELF_TEST_STAGE_1;
      lsm6dsm_inst_factory_self_test(this);
    }
    else // must be SNS_PHYSICAL_SENSOR_TEST_TYPE_COM
    {
      lsm6dsm_inst_com_self_test(this);
    }
  }
  else // this should never happen as the parents would not have forwarded bad request
  {
    DBG_INST_PRINTF_EX(HIGH, this, "Unsupported test type = %d", state->self_test_info.test_type);
    state->self_test_info.test_alive = false;
  }
}
