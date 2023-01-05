/**
 * @file sns_lsm6dso_hal_island.c
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
#include "sns_com_port_types.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_sensor_event.h"
#include "sns_service_manager.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include "sns_types.h"
#include "sns_sensor_util.h"
#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"
#include "sns_lsm6dso_log_pckts.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_printf.h"
#include "sns_async_com_port.pb.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_timer.pb.h"
#include "sns_cal.pb.h"

#define ASYNC_MIN_SAMPLES (5) //use ascp only if wmk > 5
#define LSM6DSO_MIN_HEART_BEAT_TIMEOUT_NS (20*1000*1000ULL)
#define LSM6DSO_HEART_BEAT_ODR_COUNT 5
// QC - please put usage of t and b in parenthesis
#define IS_ODR_CHANGE_TIME_EXPIRED(t, b) ((t > 0) && ((t) > (b))) ? (true) : (false)
/** Need to use ODR table. */
extern const odr_reg_map lsm6dso_odr_map[];
extern const uint32_t lsm6dso_odr_map_len;

/** LSM6DSO Accel Resolution */
const float lsm6dso_accel_resolutions[] =
{
  LSM6DSO_ACCEL_RESOLUTION_2G,
  LSM6DSO_ACCEL_RESOLUTION_4G,
  LSM6DSO_ACCEL_RESOLUTION_8G,
  LSM6DSO_ACCEL_RESOLUTION_16G
}; //mg/LSB

/** LSM6DSO Gyro Resolution */
const float lsm6dso_gyro_resolutions[] =
{
  LSM6DSO_GYRO_SSTVT_125DPS,
  LSM6DSO_GYRO_SSTVT_245DPS,
  LSM6DSO_GYRO_SSTVT_500DPS,
  LSM6DSO_GYRO_SSTVT_1000DPS,
  LSM6DSO_GYRO_SSTVT_2000DPS
};  //mdps/LSB

const lsm6dso_gyro_range lsm6dso_gyro_ranges[] =
{
  STM_LSM6DSO_GYRO_RANGE_125DPS,
  STM_LSM6DSO_GYRO_RANGE_245DPS,  /*corresponding value in register setting*/
  STM_LSM6DSO_GYRO_RANGE_500DPS,
  STM_LSM6DSO_GYRO_RANGE_1000DPS,
  STM_LSM6DSO_GYRO_RANGE_2000DPS
};

const lsm6dso_accel_range lsm6dso_accel_ranges[] =
{
  LSM6DSO_ACCEL_RANGE_2G,
  LSM6DSO_ACCEL_RANGE_4G,
  LSM6DSO_ACCEL_RANGE_8G,
  LSM6DSO_ACCEL_RANGE_16G
};

/** LSM6DSO Accel Range min */
const float lsm6dso_accel_range_min[] =
{
  LSM6DSO_ACCEL_RANGE_2G_MIN,
  LSM6DSO_ACCEL_RANGE_4G_MIN,
  LSM6DSO_ACCEL_RANGE_8G_MIN,
  LSM6DSO_ACCEL_RANGE_16G_MIN
};  //mg/LSB

/** LSM6DSO Accel Range max */
const float lsm6dso_accel_range_max[] =
{
  LSM6DSO_ACCEL_RANGE_2G_MAX,
  LSM6DSO_ACCEL_RANGE_4G_MAX,
  LSM6DSO_ACCEL_RANGE_8G_MAX,
  LSM6DSO_ACCEL_RANGE_16G_MAX
};  //mg/LSB

/** LSM6DSO Gyro Range min */
const float lsm6dso_gyro_range_min[] =
{
  LSM6DSO_GYRO_RANGE_125_MIN,
  LSM6DSO_GYRO_RANGE_245_MIN,
  LSM6DSO_GYRO_RANGE_500_MIN,
  LSM6DSO_GYRO_RANGE_1000_MIN,
  LSM6DSO_GYRO_RANGE_2000_MIN
};  //mdps/LSB

/** LSM6DSO Gyro Range max */
const float lsm6dso_gyro_range_max[] =
{
  LSM6DSO_GYRO_RANGE_125_MAX,
  LSM6DSO_GYRO_RANGE_245_MAX,
  LSM6DSO_GYRO_RANGE_500_MAX,
  LSM6DSO_GYRO_RANGE_1000_MAX,
  LSM6DSO_GYRO_RANGE_2000_MAX
};  //mdps/LSB

#define SLOPE_SETTLING_TIME_NS(min_odr) ((5*1000000000ULL)/(min_odr))
#define HPF_SETTLING_TIME_NS(min_odr) ((50*1000000000ULL)/(min_odr))
#define LSM6DSO_ODR_TOLERANCE (2) //% tolerance
#define LSM6DSO_IS_INBOUNDS(data, ref, var) \
  (((data) > ((ref) * (100+(var)))/100) ||  \
   ((data) < ((ref) * (100-(var)))/100)) ? (false) : (true)
#define WINDOW_SIZE (20)
#define MAX_MISSING_SAMPLES (32) //max_odr / min_odr

void lsm6dso_log_hw_conf(sns_sensor_instance * this)
{

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_hw_config* conf = &state->desired_conf;
  DBG_INST_PRINTF_EX(
      HIGH, this, "prev_cofig: %d %d %d %d 0x%x 0x%x %d %d",
      (uint32_t)conf->max_requested_flush_ticks, conf->odr, conf->fifo_odr, conf->wmk,
      conf->enabled_sensors, conf->publish_sensors, conf->odr_idx, conf->md_enabled);
  conf = &state->current_conf;
  DBG_INST_PRINTF_EX(
      HIGH, this, "current_cofig: %d %d %d %d 0x%x 0x%x %d %d",
      (uint32_t)conf->max_requested_flush_ticks, conf->odr, conf->fifo_odr, conf->wmk,
      conf->enabled_sensors, conf->publish_sensors, conf->odr_idx, conf->md_enabled);
}
/**
 * Turn on/off Com Port Service if not yet.
 *
 * @param[i] state            sensor instance state
 * @param[i] turn_on          true to turn on, false to turn off
 * @return sns_rc
 */
sns_rc lsm6dso_turn_on_bus_power( lsm6dso_instance_state * state, bool turn_on)
{
   sns_rc com_rv = SNS_RC_SUCCESS;
   if (turn_on !=state->bus_pwr_on)
   {
     com_rv = state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, turn_on);
     if (SNS_RC_SUCCESS == com_rv)
     {
       state->bus_pwr_on = turn_on;
     }
   }
   return com_rv;
}

 /**
 * Read wrapper for Sensor instance Synch Com Port Service.
 *
 * @param[i] state            sensor instance state
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
sns_rc lsm6dso_instance_com_read_wrapper(
   lsm6dso_instance_state * state,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes)
{
  sns_rc com_rv = SNS_RC_SUCCESS;

  com_rv = lsm6dso_turn_on_bus_power(state, true);
  if( SNS_RC_SUCCESS == com_rv )
  {
     com_rv = lsm6dso_com_read_wrapper(state->scp_service,
                            state->com_port_info.port_handle,
                            reg_addr,
                            buffer,
                            bytes,
                            xfer_bytes);
  }
  return com_rv;
}

/**
 * Read wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
sns_rc lsm6dso_com_read_wrapper(
  sns_sync_com_port_service* scp_service,
  sns_sync_com_port_handle*  port_handle,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes)
{
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = false;
  port_vec.reg_addr = reg_addr;

    return scp_service->api->sns_scp_register_rw(port_handle,
                                                        &port_vec,
                                                        1,
                                                        false,
                                                        xfer_bytes);
}

/**
 * Write wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           write buffer
 * @param[i] bytes            bytes to write
 * @param[o] xfer_bytes       bytes written
 * @param[i] save_write_time  true to save write transfer time.
 *
 * @return sns_rc
 */
sns_rc lsm6dso_com_write_wrapper(
   sns_sensor_instance * instance,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes,
   bool save_write_time)
{
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = true;
  port_vec.reg_addr = reg_addr;
  sns_rc rc;
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;

  rc = lsm6dso_turn_on_bus_power(inst_state, true);
  if (SNS_RC_SUCCESS != rc)
  {
    SNS_INST_PRINTF(ERROR, instance, "lsm6dso_com_write_wrapper: update_bus_power ON failed %d", rc);
  }
  else
  {
    rc = inst_state->scp_service->api->sns_scp_register_rw(inst_state->com_port_info.port_handle,
                                                        &port_vec,
                                                        1,
                                                        save_write_time,
                                                        xfer_bytes);
  }
  if(rc != SNS_RC_SUCCESS || *xfer_bytes != bytes)
  {
    SNS_INST_PRINTF(ERROR, instance, "write_wrapper: reg=0x%x rc=%d #bytes=%u",
                    reg_addr, rc, *xfer_bytes);
  }
#if LSM6DSO_DUMP_REG
  DBG_INST_PRINTF(HIGH, instance, "reg write 0x%x=%x", reg_addr, buffer[0]);
#endif
  return rc;
}

/**
 * If mask = 0x0 or 0xFF, or if size > 1, write reg_value
 * directly to reg_addr. Else, read value at reg_addr and only
 * modify bits defined by mask.
 *
 * @param[i] port_handle      handle to synch COM port
 * @param[i] reg_addr         reg addr to modify
 * @param[i] reg_value        value to write to register
 * @param[i] size             number of bytes to write
 * @param[o]  xfer_bytes      number of bytes transfered
 * @param[i] save_write_time  save write time input
 * @param[i] mask             bit mask to update
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dso_read_modify_write(
    sns_sensor_instance * instance,
    uint32_t reg_addr,
    uint8_t *reg_value,
    uint32_t size,
    uint32_t *xfer_bytes,
    bool save_write_time,
    uint8_t mask)
{
  uint8_t rw_buffer = 0;
  uint32_t rw_bytes = 0;

  sns_rc rc = SNS_RC_FAILED;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  
  if((size > 1) || (mask == 0xFF) || (mask == 0x00))
  {
    rc = lsm6dso_com_write_wrapper(instance,
                          reg_addr,
                          &reg_value[0],
                          size,
                          xfer_bytes,
                          save_write_time);

  }
  else
  {
    // read current value from this register
    rc = lsm6dso_instance_com_read_wrapper (state,
                                            reg_addr,
                                            &rw_buffer,
                                            1,
                                            &rw_bytes);
    if (SNS_RC_SUCCESS == rc)
    {
      // generate new value
      rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

      // write new value to this register
      rc = lsm6dso_com_write_wrapper(instance,
                            reg_addr,
                            &rw_buffer,
                            1,
                            xfer_bytes,
                            save_write_time);
    }

  }

  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance, "read_modify_write %d", rc);
  }

#if LSM6DSO_DUMP_REG
  DBG_INST_PRINTF(HIGH, instance, "reg write 0x%x=0x%x mask=0x%x",
                  reg_addr, *reg_value, mask);
#endif
  return rc;
}

/**
 * see sns_lsm6dso_hal.h
 */
/**
 * Read regs using sync com port
 *
 * @param[i] state              Instance state
 * @param[i] addr              address to read
 * @param[i] num_of_bytes       num of bytes to read
 * @param[o] buffer             status registers
 *
 * @return SNS_RC_SUCCESS if successful else SNS_RC_FAILED
 */
sns_rc lsm6dso_read_regs_scp(sns_sensor_instance * instance,
                             uint8_t addr, uint16_t num_of_bytes, uint8_t *buffer)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  rc = lsm6dso_instance_com_read_wrapper(state,
                                         addr,
                                         buffer,
                                         num_of_bytes,
                                         &xfer_bytes);

  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance, "read_regs_scp FAILED reg=0x%x", addr);
  }

  return rc;
}

sns_rc lsm6dso_write_regs_scp(sns_sensor_instance *const instance,
                              uint8_t addr, uint16_t num_of_bytes, uint8_t *buffer)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rc = lsm6dso_com_write_wrapper(instance,
                                 addr,
                                 buffer,
                                 num_of_bytes,
                                 &xfer_bytes,
                                 false);

  if(rc != SNS_RC_SUCCESS)
  {
    DBG_INST_PRINTF(ERROR, instance, "write_regs_scp FAILED");
  }

  return rc;
}

void lsm6dso_dump_odr_change_info(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type       sensor,
    bool       is_change)
{

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
   lsm6dso_odr_change_info* odr_info = &state->prev_odr_change_info; 
    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info(p): s=%d chg=%d set_t(a:g:h)=%u:%u:%u odr_chge_t=%u s_intv=%u",
        state->hw_idx, sensor, is_change,
        (uint32_t)odr_info->accel_odr_settime, (uint32_t)odr_info-> gyro_odr_settime, (uint32_t)odr_info->hw_timer_start_time,
        (uint32_t)odr_info->odr_change_timestamp, (uint32_t)odr_info->nominal_sampling_intvl);

    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info(p): s=%d chg=%d cg:cg_re:odr_idx:g_startup=0x%x:0x%x:%d:%d",
        state->hw_idx, sensor, is_change,
        odr_info->changed, odr_info->change_req, odr_info->odr_idx, odr_info->gyro_startup);

   odr_info = &state->cur_odr_change_info;
    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info(c): s=%d chg=%d set_t(a:g:h)=%u:%u:%u odr_chge_t=%u s_intv=%u",
        state->hw_idx, sensor, is_change,
        (uint32_t)odr_info->accel_odr_settime, (uint32_t)odr_info-> gyro_odr_settime, (uint32_t)odr_info->hw_timer_start_time,
        (uint32_t)odr_info->odr_change_timestamp, (uint32_t)odr_info->nominal_sampling_intvl);

    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info(c): s=%d chg=%d cg:cg_re:odr_idx:g_startup=0x%x:0x%x:%d:%d",
        state->hw_idx, sensor, is_change,
        odr_info->changed, odr_info->change_req, odr_info->odr_idx, odr_info->gyro_startup);
}

/**
* lsm6dso_update_odr_change_config
*/
void lsm6dso_update_odr_change_config(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type       sensor,
    bool       is_change)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  state->prev_odr_change_info.changed |= (is_change) ? (state->cur_odr_change_info.changed & sensor) : 0;
  state->cur_odr_change_info.changed |= (is_change) ? sensor : 0;
  DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info: s=%d chg=%d",
        state->hw_idx, sensor, is_change);
  if(sensor == LSM6DSO_GYRO)
  {
    if(is_change)
    {
      state->prev_odr_change_info.gyro_odr_settime = state->cur_odr_change_info.gyro_odr_settime;
      state->cur_odr_change_info.gyro_odr_settime = sns_get_system_time();
      //state->fifo_info.new_config.timestamp = sns_get_system_time();
    }
#if 0
    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info: s=%d chg=%d prev_chg=0x%x:%u cur_chg=0x%x:%u",
        state->hw_idx, sensor, is_change, state->prev_odr_change_info.changed, (uint32_t)state->prev_odr_change_info.gyro_odr_settime,
        state->cur_odr_change_info.changed, (uint32_t)state->cur_odr_change_info.gyro_odr_settime);
#endif
  } else if(sensor == LSM6DSO_ACCEL) {
    if(is_change)
    {
      state->prev_odr_change_info.accel_odr_settime = state->cur_odr_change_info.accel_odr_settime;
      state->cur_odr_change_info.accel_odr_settime = sns_get_system_time();
      //state->fifo_info.new_config.timestamp = sns_get_system_time();
    }
    if(state->cur_odr_change_info.odr_idx != state->desired_conf.odr_idx) {
      state->prev_odr_change_info.nominal_sampling_intvl = state->fifo_info.avg_sampling_intvl;
      state->prev_odr_change_info.odr_idx = state->cur_odr_change_info.odr_idx;
      state->cur_odr_change_info.odr_idx = state->desired_conf.odr_idx;
      state->prev_odr_change_info.change_req = state->cur_odr_change_info.change_req;
    }
    state->cur_odr_change_info.change_req |= state->config_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO);
#if 0
    DBG_INST_PRINTF(LOW, instance, "[%d] odr_chg_info: s=%d chg=%d prev_chg=0x%x:%u cur_chg=0x%x:%u",
        state->hw_idx, sensor, is_change, state->prev_odr_change_info.changed, (uint32_t)state->prev_odr_change_info.accel_odr_settime,
        state->cur_odr_change_info.changed, (uint32_t)state->cur_odr_change_info.accel_odr_settime,
        state->prev_odr_change_info.odr_idx, state->cur_odr_change_info.odr_idx);
#endif
  }
}
/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_set_gyro_config(
    sns_sensor_instance *const instance,
    lsm6dso_gyro_odr         curr_odr,
    lsm6dso_gyro_sstvt       sstvt,
    lsm6dso_gyro_range       range)
{
  UNUSED_VAR(sstvt);
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint8_t buffer;

  if(LSM6DSO_IS_ESP_ENABLED(state) && ((curr_odr != LSM6DSO_GYRO_ODR_OFF)
#if LSM6DSO_ESP_XSENSOR
    || (state->xgroup_info.is_gyro_req)
#endif
    ))
  {
    lsm6dso_gyro_odr esp_odr = (lsm6dso_gyro_odr)lsm6dso_get_esp_rate_idx(instance);
    curr_odr = (curr_odr < esp_odr) ? esp_odr : curr_odr;
  }

  buffer = (uint8_t)curr_odr | (uint8_t)range;

  if(!state->common_info.gyro_curr_odr && curr_odr)
    state->gyro_introduced = true;
  else
    state->gyro_introduced = false;

  if(!curr_odr)
  {
    state->gyro_info.reset_filter = false;
    state->gyro_info.sw_lpf_data_settled = false;
  }
  bool is_change = state->common_info.gyro_curr_odr ^ curr_odr;
  if(curr_odr)
    lsm6dso_update_odr_change_config(instance, LSM6DSO_GYRO,
                                    (state->common_info.gyro_curr_odr ^ curr_odr));
  if(state->common_info.gyro_curr_odr ^ curr_odr)
    state->cur_odr_change_info.changed |= LSM6DSO_GYRO;
 
  state->common_info.gyro_curr_odr = curr_odr;
  if(!state->self_test_info.test_alive)
  {
    state->gyro_info.sstvt = sstvt;
  }

  DBG_INST_PRINTF(LOW, instance, "gyro_config: odr=0x%x, CTRL2_G=0x%x, settime=%u",
                  curr_odr, buffer, (uint32_t)state->cur_odr_change_info.gyro_odr_settime);

  lsm6dso_dump_odr_change_info(instance, LSM6DSO_GYRO, is_change);
  return lsm6dso_write_regs_scp(instance, STM_LSM6DSO_REG_CTRL2_G, 1, &buffer);
}

/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_set_accel_config(
   sns_sensor_instance *const instance,
                              lsm6dso_accel_odr       curr_odr,
                              lsm6dso_accel_sstvt     sstvt,
                              lsm6dso_accel_range     range,
                              lsm6dso_accel_bw        bw)
{
  UNUSED_VAR(sstvt);
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint8_t buffer;

  if(LSM6DSO_IS_ESP_ENABLED(state))
  {
    lsm6dso_accel_odr esp_odr = (lsm6dso_accel_odr)lsm6dso_get_esp_rate_idx(instance);
    curr_odr = (curr_odr < esp_odr) ? esp_odr : curr_odr;
  }

  buffer = (uint8_t)curr_odr | (uint8_t)range | (uint8_t)bw;

  state->current_conf.odr_idx = state->desired_conf.odr_idx;
  state->current_conf.odr = lsm6dso_odr_map[state->desired_conf.odr_idx].odr;

  if(!state->self_test_info.test_alive)
  {
    state->accel_info.sstvt = sstvt;
  }

  bool is_change = (state->common_info.accel_curr_odr ^ curr_odr);
  lsm6dso_update_odr_change_config(instance, LSM6DSO_ACCEL,
                                  (state->common_info.accel_curr_odr ^ curr_odr));

  state->common_info.accel_curr_odr = curr_odr;
  lsm6dso_dump_odr_change_info(instance, LSM6DSO_ACCEL, is_change);
  DBG_INST_PRINTF(LOW, instance, "accel_config: odr=0x%x, CTRL1_XL=0x%x, settime=%u",
                  curr_odr, buffer, (uint32_t)state->cur_odr_change_info.accel_odr_settime);

  return lsm6dso_write_regs_scp(instance, STM_LSM6DSO_REG_CTRL1_XL, 1, &buffer);
}

/**
 * Updates GYRO_SLEEP bit.
 *
 * @param[i] port_handle   synch COM port handle
 * @param[i] set_sleep     true to enable Gyro sleep else false
 *
 * @return sns_rc
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dso_set_gyro_sleep(
    sns_sensor_instance *const instance,
    bool set_sleep)
{
  uint8_t buffer;
  uint32_t xfer_bytes;

  DBG_INST_PRINTF_EX(HIGH, instance, "set_gyro_sleep");
  if(set_sleep)
  {
    buffer = 0x40;
  }
  else
  {
    buffer = 0x0;
  }
  // Update Gyro Sleep state
  lsm6dso_read_modify_write(instance,
                            STM_LSM6DSO_REG_CTRL4,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x40);
  return SNS_RC_SUCCESS;
}

/**
 * Updates ACC Power mode  bit.
 *
 * @param[i] port_handle   synch COM port handle
 * @param[i] set_lp     true to enable acc low power else false high performance
 *
 * @return sns_rc
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dso_set_acc_lpmode(
    sns_sensor_instance *const instance,
    bool set_lp)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  //QC requested that even when only gyro is streaming, accel lp mode should be disabled.
  //The reason being some FW algos read accel data directly from DAE even when no non-gated client is present.
  //When gated accel is requested along with gyro, both accel/gyro are
  //in HW fifo, accel at DAE level has more noise if in lpm (SEE driver does filter it when gated).
  //Another option is only put gyro in fifo in this case.
  if(state->desired_conf.publish_sensors & LSM6DSO_GYRO)
    set_lp = false;
  uint8_t buffer = (set_lp) ? 0x10 : 0x0;
  uint32_t xfer_bytes;

  DBG_INST_PRINTF(HIGH, instance, "set_lp=%d", set_lp);
  // Update Gyro Sleep state
  lsm6dso_read_modify_write(instance,
                            STM_LSM6DSO_REG_CTRL6_G,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x10);
  return SNS_RC_SUCCESS;
}

sns_time lsm6dso_enable_hw_timestamp(sns_sensor_instance *this, bool enable)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint8_t rw_buffer =(enable) ?  0x20 : 0x00;
  sns_time cur_time;
  uint32_t xfer_bytes;
  lsm6dso_com_write_wrapper(this,
        STM_LSM6DSO_REG_CTRL10,
        &rw_buffer,
        1,
        &xfer_bytes,
        true);
  state->scp_service->api->sns_scp_get_write_time(state->com_port_info.port_handle,
                                                  &cur_time);
  DBG_INST_PRINTF_EX(HIGH, this, "[%d] hw_timestamps e:%d cur_time:%u", state->hw_idx, enable, (uint32_t)cur_time);
  if(!enable) {
    rw_buffer = 0xAA;
    lsm6dso_write_regs_scp(this, STM_LSM6DSO_REG_TIMESTAMP2, 1, &rw_buffer);
  }
  //returns time when hw_timer started
  return cur_time;
}

void lsm6dso_powerdown_gyro(sns_sensor_instance *this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  DBG_INST_PRINTF_EX(HIGH, this, "power down gyro");
  if(state->common_info.gyro_curr_odr) {
    lsm6dso_set_gyro_config(this,
        state->gyro_info.desired_odr,
        state->gyro_info.sstvt,
        state->gyro_info.range);
  }

  if(state->gyro_info.is_in_sleep) {
    lsm6dso_set_gyro_sleep(this, false);
    state->gyro_info.is_in_sleep = false;
  }
}

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_stop_fifo_streaming(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  if(state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF) {
    lsm6dso_set_fifo_bypass_mode(instance);
    state->fifo_info.fifo_rate = LSM6DSO_ACCEL_ODR_OFF;
    state->current_conf.fifo_odr = 0;
  }

  if(state->common_info.accel_curr_odr > LSM6DSO_ACCEL_ODR_OFF)
  {
    lsm6dso_set_accel_config(instance,
        LSM6DSO_ACCEL_ODR_OFF,
        state->accel_info.sstvt,
        state->accel_info.range,
        state->accel_info.bw);
  }

  lsm6dso_powerdown_gyro(instance);
  if(state->common_info.gyro_curr_odr > LSM6DSO_GYRO_ODR_OFF)
  {
    lsm6dso_set_gyro_sleep(instance, true);
    state->gyro_info.is_in_sleep = true;
  }
  lsm6dso_set_acc_lpmode(instance, true);

  state->fifo_info.is_streaming = false;
  DBG_INST_PRINTF(MED, instance, "stop_fifo_streaming: odr a/g 0x%x/0x%x",
                  state->common_info.accel_curr_odr,state->common_info.gyro_curr_odr);
}

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_set_fifo_config(sns_sensor_instance *const instance,
                             uint16_t desired_wmk,
                             lsm6dso_accel_odr a_chosen_sample_rate,
                             lsm6dso_gyro_odr g_chosen_sample_rate,
                             lsm6dso_sensor_type sensor)
{
  UNUSED_VAR(g_chosen_sample_rate);
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  DBG_INST_PRINTF(MED, instance, "set_fifo_config: sensor=0x%x sr=0x%x wm=%u",
                  sensor, a_chosen_sample_rate, desired_wmk);

  state->fifo_info.desired_wmk = desired_wmk;
  state->fifo_info.desired_fifo_rate = a_chosen_sample_rate;
  state->current_conf.fifo_odr = lsm6dso_odr_map[state->desired_conf.odr_idx].odr;

  if(sensor & LSM6DSO_ACCEL || sensor & LSM6DSO_MOTION_DETECT)
  {
    state->accel_info.desired_odr = a_chosen_sample_rate;
    if(state->accel_info.desired_odr < LSM6DSO_ACCEL_ODR1664)
    {
      // BW configuration bit to be unset for all ODRs till 832Hz
      state->accel_info.bw = LSM6DSO_ACCEL_BW50;
    }
    else
    {
      //BW configuration bit to be set for 1.6Khz and greater ODRs
      state->accel_info.bw = LSM6DSO_ACCEL_BW1600;
    }
  } else
    state->accel_info.desired_odr = LSM6DSO_ACCEL_ODR_OFF;

  if(sensor & LSM6DSO_GYRO)
  {
    //if gyro is enabled set acc desired rate as gyro
    state->accel_info.desired_odr = a_chosen_sample_rate;
    state->gyro_info.desired_odr = g_chosen_sample_rate;
  } else
    state->gyro_info.desired_odr = LSM6DSO_GYRO_ODR_OFF;
}


void lsm6dso_update_heartbeat_monitor(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  if((state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) ||
     (state->accel_info.gated_client_present && !state->current_conf.md_enabled))
  {
    uint64_t sampling_intvl = sns_convert_ns_to_ticks(1000000000.0f / state->desired_conf.odr);
    uint32_t wm = SNS_MAX(1, state->current_conf.wmk);
    if(lsm6dso_dae_if_available(instance))
    {
      state->fifo_info.nominal_dae_intvl =
        state->fifo_info.max_requested_wmk * sampling_intvl * 1.1f;
      wm = (state->desired_conf.max_requested_flush_ticks == 0) ?
        UINT32_MAX : state->fifo_info.max_requested_wmk;
      DBG_INST_PRINTF(LOW, instance, "update_hb_mon:: dae_intvl=%u",
                      state->fifo_info.nominal_dae_intvl);
    }
    if(wm < LSM6DSO_MAX_FIFO)
    {
      uint64_t min_hb_to = sns_convert_ns_to_ticks(LSM6DSO_MIN_HEART_BEAT_TIMEOUT_NS);
      uint64_t max_hb_to = sampling_intvl * LSM6DSO_HW_MAX_FIFO;
      state->health.heart_beat_timeout = sampling_intvl * wm * LSM6DSO_HEART_BEAT_ODR_COUNT;

      // avoid sending timer request too frequently
      state->health.heart_beat_timeout = SNS_MAX(state->health.heart_beat_timeout, min_hb_to);
      // limit to one full FIFO to avoid large data gap
      state->health.heart_beat_timeout = SNS_MIN(state->health.heart_beat_timeout, max_hb_to);
    }
    else
    {
      state->health.heart_beat_timeout = sampling_intvl * LSM6DSO_HEART_BEAT_ODR_COUNT * wm;
    }
    DBG_INST_PRINTF(MED, instance, "update_hb_mon:: wm=%u heart_beat_to=%X%08X",
                    wm, (uint32_t)(state->health.heart_beat_timeout >> 32),
                    (uint32_t)state->health.heart_beat_timeout);
  }
  else
  {
    DBG_INST_PRINTF(MED, instance, "update_hb_mon:: removing timer");
    state->health.expected_expiration = UINT64_MAX;
    state->health.heart_beat_timeout = UINT64_MAX/2;
  }
  lsm6dso_restart_hb_timer(instance, false);
}

static void lsm6dso_interrupt_interval_init_nominal(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint32_t interrupt_intvl_threshold;

  state->fifo_info.avg_interrupt_intvl =
    state->fifo_info.avg_sampling_intvl * state->desired_conf.wmk;

  // threshold is +- LSM6DSO_ODR_TOLERANCE% of nominal sampling intvl
  interrupt_intvl_threshold =
    state->fifo_info.avg_interrupt_intvl * LSM6DSO_ODR_TOLERANCE / 100;
  state->fifo_info.interrupt_intvl_upper_bound =
    state->fifo_info.avg_interrupt_intvl + interrupt_intvl_threshold;
  state->fifo_info.interrupt_intvl_lower_bound =
    state->fifo_info.avg_interrupt_intvl - interrupt_intvl_threshold;

  DBG_INST_PRINTF(
      LOW, instance, "i_intvl=%u i_bounds=%u/%u",
      state->fifo_info.avg_interrupt_intvl,
      state->fifo_info.interrupt_intvl_lower_bound,
      state->fifo_info.interrupt_intvl_upper_bound);
}
 
/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_set_fifo_wmk(sns_sensor_instance *const instance)
{
  uint16_t wmk = 0;
  uint8_t wmkL = 0;
  uint8_t wmkH = 0;
  uint8_t mask;
  uint8_t bdr = 0;
  uint16_t set_wmk = 0;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint8_t *regs = state->fifo_info.ctrl_regs;

  DBG_INST_PRINTF(HIGH, instance, "set_fifo_wmk: cur=%u des=%u",
                  state->fifo_info.cur_wmk, state->fifo_info.desired_wmk);
  lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_FIFO_CTRL1, 6, regs);
  DBG_INST_PRINTF(LOW, instance, "%02X %02X %02X %02X %02X %02X",
                  regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);

  state->fifo_info.cur_wmk = state->fifo_info.desired_wmk;

  // QC: How many things are current at the same time?
  //There's a fifo_info current watermark, and a current config watermark. Which one is more current?
  state->current_conf.wmk = state->fifo_info.desired_wmk;

  if(state->accel_info.gated_client_present 
       && !(state->desired_conf.publish_sensors & LSM6DSO_ACCEL)
       && (state->current_conf.wmk != state->fifo_info.last_sent_config.fifo_watermark))
    state->fifo_info.new_config.gated_timestamp = sns_get_system_time();

  set_wmk = state->current_conf.wmk;
  if(state->ag_stream_mode == S4S_SYNC) {
    set_wmk = LSM6DSO_MAX_FIFO;
  }

  //convert samples to words
  if(((state->accel_info.desired_odr > LSM6DSO_ACCEL_ODR_OFF) &&
      (state->fifo_info.fifo_enabled & LSM6DSO_ACCEL)) ||
     ((state->self_test_info.sensor == LSM6DSO_ACCEL)&&(state->self_test_info.test_alive)))
  {
    wmk = set_wmk;
    bdr |= state->accel_info.desired_odr >> 4;
  }

  if(((state->gyro_info.desired_odr > LSM6DSO_GYRO_ODR_OFF) &&
      (state->fifo_info.fifo_enabled & LSM6DSO_GYRO)) ||
     ((state->self_test_info.sensor == LSM6DSO_GYRO)&&(state->self_test_info.test_alive)))
  {
    if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr) &&
       (state->self_test_info.self_test_stage == LSM6DSO_SELF_TEST_STAGE_1))
    {
      wmk += state->fifo_info.cur_wmk * 2;
    }
    else
    {
      wmk += set_wmk;
    }
    bdr |= state->gyro_info.desired_odr;
  }

  // Set Accel decimation to no decimation
  mask = STM_LSM6DSO_FIFO_WTM_CTRL1_MASK;
  // Configure FIFO WM
  wmkL = wmk & STM_LSM6DSO_FIFO_WTM_CTRL1_MASK;
  regs[0] = ((regs[0] & (~mask)) | (wmkL & mask));

  // Enables ODR CHANGE virtual sensor to be batched in FIFO
  mask = STM_LSM6DSO_FIFO_WTM_CTRL2_MASK|STM_LSM6DSO_FIFO_ODR_CHG_EN_MASK;
  wmkH = ((wmk >> 8) & STM_LSM6DSO_FIFO_WTM_CTRL2_MASK) | STM_LSM6DSO_FIFO_ODR_CHG_EN_MASK;
  regs[1] = ((regs[1] & (~mask)) | (wmkH & mask));
  regs[2] = bdr;

  //Use counter_BDR for getting accurate number of sensor samples(without tag sensor overhead)
  if((state->gyro_info.desired_odr > LSM6DSO_GYRO_ODR_OFF) &&
     (state->fifo_info.fifo_enabled & LSM6DSO_GYRO)) {
    wmk -= set_wmk;
  }

  mask = STM_LSM6DSO_FIFO_CNT_BDR_REG1_MASK;
  wmkH = (wmk >> 8) & mask;
  //clear BDR_TRIG if gyro is turned off
  mask |= STM_LSM6DSO_FIFO_TRIG_CNT_BDR_MASK;
  //If gyro is streaming, count gyro samples
  if(state->gyro_info.desired_odr > LSM6DSO_GYRO_ODR_OFF) {
    wmkH |= STM_LSM6DSO_FIFO_TRIG_CNT_BDR_MASK;
  }
  regs[4] = ((regs[4] & (~mask)) | (wmkH & mask));

  mask = STM_LSM6DSO_FIFO_CNT_BDR_REG2_MASK;
  wmkL = wmk & mask;
  regs[5] = ((regs[5] & (~mask)) | (wmkL & mask));

  lsm6dso_write_regs_scp(instance, STM_LSM6DSO_REG_FIFO_CTRL1, 6, regs);
  DBG_INST_PRINTF(LOW, instance, "%02X %02X %02X %02X %02X %02X",
                  regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);

  //update interrupt interval and bounds since wmk is changing
  lsm6dso_interrupt_interval_init_nominal(instance);
  lsm6dso_update_heartbeat_monitor(instance);
}

/**
 * see sns_lsm6dso_hal.h
 */
sns_time lsm6dso_get_sample_interval(sns_sensor_instance *this, float odr)
{
  sns_time  sample_interval = 0;

  if(odr > 0.0f)
  {
    lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
    uint16_t odr_coeff = lsm6dso_odr_map[lsm6dso_get_odr_rate_idx(odr)].odr_coeff;
    float odr_drift_max = state->clock_trim_factor * LSM6DSO_HW_MAX_ODR; //6667
    float actual_odr = odr_drift_max/odr_coeff;
    sample_interval = sns_convert_ns_to_ticks(1000000000.0f / actual_odr);

    LSM6DSO_INST_DEBUG_TS(
      LOW, this, "get_sample_interval: coeff=%u odr_drift_max=%d odr*1000=%d intvl=%u percent_var xl:g=%d:%d",
      odr_coeff, (int)(odr_drift_max*1000),
      (int)(actual_odr*1000), (uint32_t)sample_interval, state->odr_percent_var_accel, state->odr_percent_var_gyro);
  }

  return sample_interval;
}

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_start_fifo_streaming(sns_sensor_instance *const instance)
{
  // Enable FIFO Streaming
  // Enable Accel Streaming
  // Enable GYRO Streaming

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  bool is_md_req;
  bool is_md_force_disabled;
  bool init_nominal = false;
  int32_t odr_percent_var;


  DBG_INST_PRINTF(HIGH, instance, "start_fifo_streaming: rate=0x%x last_ts=%u",
                  state->fifo_info.desired_fifo_rate,
                  (uint32_t)state->fifo_info.last_timestamp);

  DBG_INST_PRINTF(
    LOW, instance, "cur=%u des=%u fifo=%x common=%u",
    state->current_conf.odr, state->desired_conf.odr,
    state->fifo_info.fifo_rate, state->common_info.accel_curr_odr);

  bool hw_timer_reset = false;
  //if odr is changing or tag for prev odr change is already received
  if((state->current_conf.odr != state->desired_conf.odr) ||
      !state->cur_odr_change_info.changed ||
      (state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF)) {
    lsm6dso_enable_hw_timestamp(instance, false);
    hw_timer_reset = true;
  }

  if((state->current_conf.odr != state->desired_conf.odr ||
    state->current_conf.enabled_sensors != state->desired_conf.enabled_sensors ||
     state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF)) {
    //reset only if odr is changed
    //if odr is same as before, no need to recalculate sampling interval
    state->fifo_info.interrupt_cnt = 0;
    init_nominal = true;

    if((state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF)
        && (state->common_info.accel_curr_odr != LSM6DSO_ACCEL_ODR_OFF)) {
      DBG_INST_PRINTF(MED, instance, "start_fifo_streaming: turning off accel");
      //accel is on but fifo is off, turn off accel
      lsm6dso_set_accel_config(instance,
          LSM6DSO_ACCEL_ODR_OFF,
          state->accel_info.sstvt,
          state->accel_info.range,
          LSM6DSO_ACCEL_BW50);
    }
  }

  is_md_req = lsm6dso_is_md_int_required(instance);
  is_md_force_disabled = false;
  DBG_INST_PRINTF(HIGH, instance, "start_fifo_streaming: md_enabled/md_req = %d, %d",
      state->current_conf.md_enabled, is_md_req);
  //disable md if enabled and is required
  //Avoiding spurious interrupts
  if(state->current_conf.md_enabled && is_md_req) {
    lsm6dso_disable_md(instance, false);
    is_md_force_disabled = true;
  }

  if(state->ag_stream_mode == S4S_SYNC)  {
    if(state->fifo_info.reconfig_req)
      lsm6dso_update_s4s_sync(instance, state->desired_conf.odr);
    lsm6dso_start_s4s_sync(instance, state->desired_conf.odr, sns_get_system_time());
  }

  //starting new h/w timer for cfg tag timestamp
  sns_time hw_timer_start;
  if(hw_timer_reset) {
    hw_timer_start = lsm6dso_enable_hw_timestamp(instance, true);
  }
  else
    hw_timer_start = sns_get_system_time();
  bool hw_timer_time_set = false;
  sns_time cur_odr_set_time = state->cur_odr_change_info.accel_odr_settime;
  lsm6dso_set_accel_config(instance,
      state->accel_info.desired_odr,
      state->accel_info.sstvt,
      state->accel_info.range,
      state->accel_info.bw);
  //check if accel odr is changed
  if(state->cur_odr_change_info.accel_odr_settime != cur_odr_set_time) {
    state->prev_odr_change_info.hw_timer_start_time = state->cur_odr_change_info.hw_timer_start_time;
    state->cur_odr_change_info.hw_timer_start_time = state->cur_odr_change_info.accel_odr_settime;
    hw_timer_time_set = true;
  } else if (state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF) {
    // streaming just started
    state->prev_odr_change_info.hw_timer_start_time = 0; 
    state->cur_odr_change_info.hw_timer_start_time = hw_timer_start;
    hw_timer_time_set = true;
  }
  lsm6dso_gyro_odr cur_odr = state->common_info.gyro_curr_odr;
  lsm6dso_set_gyro_config(instance,
      state->gyro_info.desired_odr,
      state->gyro_info.sstvt,
      state->gyro_info.range);
  //if accel odr not changed
  if(!hw_timer_time_set && hw_timer_reset) {
    //gyro odr is changed
    if(state->common_info.gyro_curr_odr != cur_odr) {
      //if odr changed or
      //gyro started with same odr
      if(state->current_conf.odr != state->desired_conf.odr) {
        state->prev_odr_change_info.hw_timer_start_time = state->cur_odr_change_info.hw_timer_start_time;
        state->cur_odr_change_info.hw_timer_start_time = state->cur_odr_change_info.gyro_odr_settime;
      } else if(state->common_info.gyro_curr_odr) {
        //odr is changed
        state->cur_odr_change_info.hw_timer_start_time = state->cur_odr_change_info.gyro_odr_settime;
      } else {
        state->cur_odr_change_info.hw_timer_start_time = hw_timer_start;
      }
      hw_timer_time_set = true;
    }
  }
  //accel and gyro not changed
  if(!hw_timer_time_set && hw_timer_reset)
    lsm6dso_enable_hw_timestamp(instance, false);

  if((state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF) || (!state->fifo_info.is_streaming)){
    sns_time now = sns_get_system_time();
    DBG_INST_PRINTF(LOW, instance, "start_fifo_streaming: resetting last_timestamp to now %u", (uint32_t)now);
    state->fifo_info.last_timestamp = now;
    state->fifo_info.last_ts_valid = false; //set last ts as inaccurate
    state->fifo_info.gyro_extra_sample = false;
    //reset prev odr configuration as streaming is starting now
    sns_memset(&state->prev_odr_change_info, 0, sizeof(lsm6dso_odr_change_info));
  }

  if(init_nominal)
  {
    //reset only if odr is changed
    //if odr is same as before, no need to recalculate sampling interval
    state->fifo_info.interrupt_cnt = 0;

    //reset only if odr is changed or gyro was added
    //if odr is same as before with same sensor, no need to recalculate sampling interval
    state->cur_odr_change_info.nominal_sampling_intvl = (uint32_t)lsm6dso_get_sample_interval(instance, (float)state->desired_conf.odr);

    if(state->common_info.gyro_curr_odr && !state->gyro_introduced)
      odr_percent_var = state->odr_percent_var_gyro;
    else
      odr_percent_var = state->odr_percent_var_accel;

    state->fifo_info.avg_sampling_intvl = state->cur_odr_change_info.nominal_sampling_intvl - (odr_percent_var / state->desired_conf.odr);
    lsm6dso_interrupt_interval_init_nominal(instance);

  }
  //start streaming,stream mode
  lsm6dso_set_fifo_stream_mode(instance);

  if(state->gyro_info.is_in_sleep)
  {
    lsm6dso_set_gyro_sleep(instance, false);
    state->gyro_info.is_in_sleep = false;
  }
  if(state->ag_stream_mode != DRI) {
    lsm6dso_run_polling_timer(instance);
  }

  //re-enable if force disabled before
  if(is_md_force_disabled) {
    lsm6dso_enable_md(instance, false);
  }

//  QC requested that even when only gyro is streaming, accel lp mode should be disabled.
//  if(state->desired_conf.publish_sensors & LSM6DSO_ACCEL) {
    lsm6dso_set_acc_lpmode(instance, false);
//  }
  state->fifo_info.is_streaming =
    ((state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) &&
     state->desired_conf.max_requested_flush_ticks);

  state->config_sensors &= ~(LSM6DSO_ACCEL | LSM6DSO_GYRO) ;
  state->fifo_info.fifo_rate = state->fifo_info.desired_fifo_rate;
  state->current_conf.enabled_sensors =
    state->fifo_info.fifo_enabled & (LSM6DSO_ACCEL | LSM6DSO_GYRO);

  // Handle case when ODR is not changeing and accel is introduced
  // while gyro was already streaming.
  if(!(state->current_conf.publish_sensors & LSM6DSO_ACCEL) &&
      (state->desired_conf.publish_sensors & LSM6DSO_ACCEL) &&
      !(state->cur_odr_change_info.changed & LSM6DSO_ACCEL))
  {
    state->cur_odr_change_info.changed |= LSM6DSO_ACCEL;
    state->cur_odr_change_info.accel_odr_settime = sns_get_system_time();
  }

  DBG_INST_PRINTF(MED, instance,
                  "start_fifo_streaming EX: a_odr=0x%x g_odr=0x%x",
                  state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);

}

void lsm6dso_run_polling_timer(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  sns_time report_period; //in ticks
  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time(); // QC - should be system_time() - report_period.
  req_payload.timeout_period =  report_period =
    sns_convert_ns_to_ticks(1000000000.0 / state->current_conf.odr) * state->current_conf.wmk;
  req_payload.priority = (state->ag_stream_mode) ? SNS_TIMER_PRIORITY_POLLING : SNS_TIMER_PRIORITY_S4S;
  req_payload.is_dry_run = lsm6dso_dae_if_available(instance); // if dae available, set dry run
  if(state->ag_stream_mode == S4S_SYNC)  {
    if(state->s4s_info.sync_state == LSM6DSO_S4S_SYNCED) {
      // QC - Instead of doing this, we recommend changing req_payload.start_time above
      // and switching early_start_delta and late_start_delta
      // This will lead to a shorter time to the first sample
      req_payload.start_config.early_start_delta =
        sns_convert_ns_to_ticks(1000000000.0f / state->current_conf.odr);
      req_payload.start_config.late_start_delta = 0;
    }

    req_payload.timeout_period = report_period =
      sns_convert_ns_to_ticks(1000000000.0 / state->current_conf.odr) * state->s4s_info.poll_time;
    DBG_INST_PRINTF(HIGH, instance,
        "run_polling_timer:: sync_poll_count = %d sync_time = %d poll_time = %d",
        state->s4s_info.sync_poll_count, state->s4s_info.sync_time, state->s4s_info.poll_time);
  }
  DBG_INST_PRINTF(HIGH, instance, "run_polling_timer:: timeout_period =%u report_period = %u",
      (uint32_t)req_payload.timeout_period, (uint32_t)report_period);

  state->poll_timeout = req_payload.timeout_period;
  lsm6dso_inst_create_timer(instance, &state->timer_polling_data_stream, &req_payload);
}

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_enable_fifo_intr(sns_sensor_instance *const instance,
                              lsm6dso_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint8_t *int1_ctrl_reg = 
    &state->fifo_info.ctrl_regs[STM_LSM6DSO_REG_INT1_CTRL - STM_LSM6DSO_REG_FIFO_CTRL1];

  DBG_INST_PRINTF(HIGH, instance,
      "enable_fifo_intr:: sensor=0x%x INT1_CTRL=0x%x ag_stream_mode %d",
      sensor, *int1_ctrl_reg, state->ag_stream_mode);

  if(*int1_ctrl_reg == 0 &&
     state->ag_stream_mode == DRI)
  {
    DBG_INST_PRINTF(HIGH, instance,
        "enable_fifo_intr:: fifo_enabled=0x%x irq_ready=%d cur_wmk=%d",
        state->fifo_info.fifo_enabled, state->irq_ready, state->fifo_info.cur_wmk);

    if(state->fifo_info.fifo_enabled & (LSM6DSO_ACCEL | LSM6DSO_GYRO) &&
        (state->fifo_info.cur_wmk > 0) &&
        (state->irq_ready || lsm6dso_dae_if_available(instance))) {

      lsm6dso_clear_interrupt_q(instance, state->interrupt_data_stream);
      // Configure lsm6dso FIFO control registers
      rw_buffer = 0x0
        | (0<<7)       // DEN_DRDY
        | (1<<6)       // INT1 CNT_BDR
        | (0<<5)       // INT1 FSS5
        | (1<<4)       // INT1 FIFO_OVR flag
        | (0<<3)       // INT1 FIFO_FTH flag
        | (0<<2)       // INT1 BOOT flag
        | (0<<1)       // INT1 DRDY_G flag
        | 0;           // INT1 DRDY_XL flag

      lsm6dso_read_modify_write(instance,
          STM_LSM6DSO_REG_INT1_CTRL,
          &rw_buffer,
          1,
          &xfer_bytes,
          false,
          // QC - Does this want to turn FIFO_FTH off? If so, it still needs to be part of the mask.
          STM_LSM6DSO_FIFO_CNT_BDR_MASK|STM_LSM6DSO_FIFO_OVR_INT_MASK);
      lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_INT1_CTRL, 1,
          int1_ctrl_reg);
      DBG_INST_PRINTF_EX(LOW, instance, "INT1_CTRL=0x%x", *int1_ctrl_reg);
    }
  } else {
    DBG_INST_PRINTF(LOW, instance, "Not enabling FIFO interrupt stream_mode = %d",
                    state->ag_stream_mode);
  }
}

void lsm6dso_disable_fifo_intr(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  uint32_t xfer_bytes;
  uint8_t rw_buffer = 0;
  uint8_t *int1_ctrl_reg = 
    &state->fifo_info.ctrl_regs[STM_LSM6DSO_REG_INT1_CTRL - STM_LSM6DSO_REG_FIFO_CTRL1];

  DBG_INST_PRINTF_EX(HIGH, instance, "disable_fifo_intr");
  *int1_ctrl_reg = 0;
  lsm6dso_read_modify_write(instance,
                            STM_LSM6DSO_REG_INT1_CTRL,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            STM_LSM6DSO_FIFO_CNT_BDR_MASK|STM_LSM6DSO_FIFO_OVR_INT_MASK);
}

//return no of bytes in fifo
sns_rc lsm6dso_get_fifo_status(
    sns_sensor_instance *const instance,
    uint16_t* bytes_in_fifo,
    uint8_t* status_reg)
{
  sns_rc rc = SNS_RC_SUCCESS;
#if !LSM6DSO_DAE_ENABLED
  //read fifo regs
  rc = lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_FIFO_STATUS1, 2, status_reg);

  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance,
        "read_fifo_status fail %d", rc);
    return rc;
  }
  // Calculate the number of bytes to be read
  uint16_t countH = status_reg[1] & STM_LSM6DSO_FIFO_DIFF2_MASK;
  uint16_t countL = status_reg[0] & STM_LSM6DSO_FIFO_DIFF1_MASK;
  *bytes_in_fifo =  (((countH << 8) & 0xFF00) | countL) * (STM_LSM6DSO_FIFO_SAMPLE_SIZE);
  if((!*bytes_in_fifo) && (status_reg[1] & 0x40)) {
    *bytes_in_fifo = 3072;
  }
  LSM6DSO_INST_DEBUG_TS(LOW, instance,
    "status_reg 0x%x 0x%x",status_reg[0], status_reg[1]);
  LSM6DSO_INST_DEBUG_TS(LOW, instance,
    "count 0x%x num_of_bytes %d",(countH << 8) | countL, *bytes_in_fifo);
#else
  UNUSED_VAR(instance);
  UNUSED_VAR(bytes_in_fifo);
  UNUSED_VAR(status_reg);
  rc = SNS_RC_NOT_SUPPORTED;
#endif
  return rc;
}

// QC - should keep a state variable for TAP_CFG2 and TAP_CFG and do 1 bus operation instead of 4
sns_rc lsm6dso_set_interrupts(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  sns_rc rv = SNS_RC_SUCCESS;
  if(!state->int_enabled) {
    rw_buffer = 0x80;
    rv = lsm6dso_read_modify_write(instance,
        STM_LSM6DSO_REG_TAP_CFG2,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x80);
    //Enable Latched mode
    rw_buffer = 0x01;
    rv = lsm6dso_read_modify_write(instance,
        STM_LSM6DSO_REG_TAP_CFG0,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x01);
    state->int_enabled = true;
  }
  return rv;
}

#if !LSM6DSO_DAE_ENABLED
static void lsm6dso_reset_bdr_cnt(sns_sensor_instance *const this)
{
  uint32_t xfer_bytes;
  uint8_t rw_buffer = 0x40;
  lsm6dso_read_modify_write(this,
      STM_LSM6DSO_REG_CNT_BDR1,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      0x40);
}
#endif

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_flush_fifo(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) {
    LSM6DSO_INST_DEBUG_TS(HIGH, this, "resetting int_cnt");
    state->fifo_info.interrupt_cnt = 0;
  }
  state->fifo_info.th_info.flush_req = true;
  if (!lsm6dso_dae_if_flush_hw(this))
  {
    lsm6dso_read_fifo_data(this, sns_get_system_time(), true);
  }
}

/**
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
void lsm6dso_set_polling_config(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  DBG_INST_PRINTF(
    MED, this, "set_polling_config: s_intvl=%u sr(*1000)=%d",
    (uint32_t)state->sensor_temp_info.sampling_intvl,
    (int)(state->sensor_temp_info.desired_sampling_rate_hz*1000));

  if(state->sensor_temp_info.sampling_intvl > 0)
  {
    lsm6dso_start_sensor_temp_polling_timer(this);
  }
  else
  {
    state->sensor_temp_info.timer_is_active = false;
    state->sensor_temp_info.cur_sampling_rate_hz = 0;
  }
}

/**
 * Gets current Accel ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 */
float lsm6dso_get_accel_odr(lsm6dso_accel_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for(idx = 0; idx < lsm6dso_odr_map_len; idx++)
  {
    if(curr_odr == lsm6dso_odr_map[idx].accel_odr_reg_value
       &&
       curr_odr != LSM6DSO_ACCEL_ODR_OFF)
    {
      odr = lsm6dso_odr_map[idx].odr;
      break;
    }
  }

  return odr;
}

static inline vector3 vector3_mul_scalar(const vector3 v, float f)
{
  return make_vector3(v.x * f, v.y * f, v.z * f);
}

/**
 * LSM6DSO SW Filter.
 *
 * @param[i] instance              sensor instance
 * @param[i] inData                Current sensor data
 * @param[i] prevData              Previous sensor data
 * @param[i] prevFilteredData      Previous filtered data
 * @param[i] outData               Current filtered output data
 *
 */

void lsm6dso_sw_lpf_filter(sns_sensor_instance *const instance,
               float* restrict inData, float* restrict prevData, 
               float* restrict prevFilteredData, float* restrict outData)

{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  int n;

  if(!state->gyro_info.reset_filter)
  {
    for(n = 0; n < TRIAXIS_NUM; n++)
    {
      prevData[n] = prevFilteredData[n]  = outData[n] = inData[n];
    }
    DBG_INST_PRINTF_EX(HIGH, instance, "reset done for the filter");
    state->gyro_info.reset_filter = true;
    return;
  }

  for(n = 0; n < TRIAXIS_NUM; n++)
  {
    outData[n] = prevFilteredData[n] = LSM6DSO_SW_FILTER_COEFF_B * (inData[n] + prevData[n]) - LSM6DSO_SW_FILTER_COEFF_A *prevFilteredData[n];
    prevData[n] = inData[n];
  }
  return;
}
vector3 lsm6dso_accel_data_conversion(sns_sensor_instance *const instance,
                                     const uint8_t* ip_raw,
                                     float* opdata_raw)
{

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  vector3 opdata_cal;
  float ipdata[TRIAXIS_NUM] = {0};
  uint_fast8_t i;
  for(i = 0; i < ARR_SIZE(ipdata); i++)
  {
    uint_fast8_t j = i << 1;
    ipdata[i] =
      (int16_t)(((ip_raw[j+1] << 8) & 0xFF00) | ip_raw[j]) *
      state->accel_info.sstvt * G / 1000000;
    state->accel_info.sample.opdata_raw[j] = ip_raw[j];
    state->accel_info.sample.opdata_raw[j+1] = ip_raw[j+1];
  }

  for(i = 0; i < TRIAXIS_NUM; i++)
  {
    opdata_raw[state->axis_map[i].opaxis] =
      (state->axis_map[i].invert ? -1.0f : 1.0f) * ipdata[state->axis_map[i].ipaxis];
  }


 /* thermal_bias = thermal_scale * (T - T_6pt) */
  vector3 thermal_bias = vector3_mul_scalar(
      state->accel_registry_cfg.thermal_scale,
      state->most_recent_temperature_reading == 0.f ? 0.f :
      state->most_recent_temperature_reading - state->accel_registry_cfg.mean_temp);
#if LSM6DSO_DEBUG_SENSOR_DATA
  DBG_INST_PRINTF(HIGH, instance, "accel thermal_bias: %d/10000, %d/10000, %d/10000",
      (int)(thermal_bias.x * 10000),
      (int)(thermal_bias.y * 10000),
      (int)(thermal_bias.z * 10000));
#endif
  // factory calibration
  opdata_cal = sns_apply_calibration_correction_3(
      make_vector3_from_array(opdata_raw),
      vector3_add(make_vector3_from_array(state->accel_registry_cfg.fac_cal_bias), thermal_bias),
      state->accel_registry_cfg.fac_cal_corr_mat);
  return opdata_cal;
}


vector3 lsm6dso_gyro_data_conversion(sns_sensor_instance *const instance,
                                     const uint8_t* ip_raw,
                                     float* opdata_raw)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  vector3 opdata_cal;

  float ipdata[TRIAXIS_NUM] = {0};
  uint_fast8_t i;
  float* temp_opdata = NULL;
  for(i = 0; i < ARR_SIZE(ipdata); i++)
  {
    uint_fast8_t j = i << 1;
    ipdata[i] =
      (int16_t)(((ip_raw[j+1] << 8) & 0xFF00) | ip_raw[j]) *
      ((state->gyro_info.sstvt *  PI) / 180.0f);
    state->gyro_info.sample.opdata_raw[j] = ip_raw[j];
    state->gyro_info.sample.opdata_raw[j+1] = ip_raw[j+1];
  }
  for(i = 0; i < TRIAXIS_NUM; i++)
  {
    opdata_raw[state->axis_map[i].opaxis] =
      (state->axis_map[i].invert ? -1.0f : 1.0f) * ipdata[state->axis_map[i].ipaxis];
  }

  temp_opdata = opdata_raw;

  lsm6dso_odr_change_info* config_ptr = &state->cur_odr_change_info;
  if(state->fifo_info.orphan_batch)
    config_ptr = &state->prev_odr_change_info;

  if((lsm6dso_odr_map[config_ptr->odr_idx].gyro_odr_reg_value == LSM6DSO_GYRO_ODR832) &&
     !state->gyro_info.sw_lpf_data_settled)
  {
    DBG_INST_PRINTF_EX(HIGH, instance, "Using gyro sw filtering");
    lsm6dso_sw_lpf_filter(instance, temp_opdata,
                          state->gyro_info.prev_data,
                          state->gyro_info.prev_filtered,
                          opdata_raw);
  }
  /* thermal_bias = thermal_scale * (T - T_6pt) */
  vector3 thermal_bias = vector3_mul_scalar(
      state->gyro_registry_cfg.thermal_scale,
      state->most_recent_temperature_reading == 0.f ? 0.f :
      state->most_recent_temperature_reading - state->gyro_registry_cfg.mean_temp);
#if LSM6DSO_DEBUG_SENSOR_DATA
  DBG_INST_PRINTF(HIGH, instance, "gyro thermal_bias: %d/10000, %d/10000, %d/10000",
      (int)(thermal_bias.x * 10000),
      (int)(thermal_bias.y * 10000),
      (int)(thermal_bias.z * 10000));
#endif
  // factory calibration
  opdata_cal = sns_apply_calibration_correction_3(
      make_vector3_from_array(opdata_raw),
      vector3_add(make_vector3_from_array(state->gyro_registry_cfg.fac_cal_bias), thermal_bias),
      state->gyro_registry_cfg.fac_cal_corr_mat);
  return opdata_cal;

}

/*
 * cur_odr_changed : true  - CFG_TAG not received -> check if tag is dropped at DAE
 * cur_odr_changed : false - CFG_TAG is received, data may not stable yet 
 * invalid count > 0: DATA not stable yet -> some data may have dropped at DAE, thats OKAY, mark few samples invalid
 * invalid count <= 0: DATA stable
 *
 * orphan batch: prev odr data: 
 * prev_odr_changed : true  - CFG_TAG not received -> check if tag is dropped at DAE
 * prev_odr_changed : false - CFG_TAG is received, data may not stable yet 
 * invalid count > 0: DATA not stable yet -> some data may have dropped at DAE, thats OKAY, mark few samples invalid
 * invalid count <= 0: DATA stable
 * 
 * while handling orphan batch, if invalid count is set, it means previous ODR data was not stable. 
 */

sns_std_sensor_sample_status lsm6dso_mark_sample_status(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type sensor,
    uint16_t sample_idx,
    lsm6dso_odr_change_info* config,
    sns_time ts)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  int64_t elapsed_time;
  uint8_t odr_idx = config->odr_idx;
  sns_time  min_time;
  uint16_t max_invalid_cnt;
  uint16_t* cur_invalid_cnt;
  bool cfg_changed = (config->changed & sensor);
  sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

  if(sensor == LSM6DSO_ACCEL) {
    cur_invalid_cnt = &state->accel_info.num_samples_to_discard;
    max_invalid_cnt = lsm6dso_odr_map[odr_idx].accel_discard_samples;
    elapsed_time =  ts - config->accel_odr_settime;
    min_time = 0;
  } else {
    cur_invalid_cnt = &state->gyro_info.num_samples_to_discard;
    max_invalid_cnt = lsm6dso_odr_map[odr_idx].gyro_discard_samples;
    min_time = sns_convert_ns_to_ticks(LSM6DSO_GYRO_ON_TIME_MS * 1000 * 1000) ;
    elapsed_time =  ts - config->gyro_odr_settime;
  }
  if(!sample_idx && (cfg_changed || *cur_invalid_cnt)) {
    sns_time sampling_intvl = lsm6dso_get_sample_interval(instance, lsm6dso_odr_map[odr_idx].odr);
    sns_time max_time = min_time + max_invalid_cnt * sampling_intvl;

    DBG_INST_PRINTF(LOW, instance, "mark samples: %d %d %u %u", sensor, state->fifo_info.orphan_batch, config->changed, *cur_invalid_cnt);
    if(IS_ODR_CHANGE_TIME_EXPIRED(elapsed_time, max_time)) {
      status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

      *cur_invalid_cnt = 0;
      DBG_INST_PRINTF(LOW, instance, "odr change tag not received: s=0x%x elapsed_time=%d",
          sensor, (int32_t)elapsed_time);
      state->gyro_introduced = false;
      if(!state->fifo_info.orphan_batch)
        state->current_conf.publish_sensors = state->desired_conf.publish_sensors;
    } else if(*cur_invalid_cnt) {
      (*cur_invalid_cnt)--;
      status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    } else {
      //comes here only if cfg_change = true and first sample is not TAG, it means TAG is missing
      //so set invalid count
      *cur_invalid_cnt = (max_time - elapsed_time ) / sampling_intvl + 1;
      status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
      if(!state->fifo_info.orphan_batch)
        state->current_conf.publish_sensors = state->desired_conf.publish_sensors;
    }
    config->changed &= ~sensor;
  } else if((sensor == LSM6DSO_GYRO) && config->gyro_startup) {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    DBG_INST_PRINTF(LOW, instance, "Gyro Starting: discarding GYRO ts=%u (#%u)",
        (uint32_t)ts, state->gyro_sample_counter);
    //DAE dropped TAG data after first TAG is received for gyro
    if(!sample_idx && IS_ODR_CHANGE_TIME_EXPIRED(elapsed_time, min_time)) {
      *cur_invalid_cnt = max_invalid_cnt;
      config->gyro_startup = false;
      state->gyro_introduced = false;
    }
  } else if(*cur_invalid_cnt) {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    (*cur_invalid_cnt)--;
    if(*cur_invalid_cnt == 0)
    {
      DBG_INST_PRINTF(MED, instance, "discarding s=%d ts=%u",
          sensor, (uint32_t)ts);
    }
  }
  return status;
}

//ts = calculated timestamp
//ts_adjust = group delay + any oem adjustments
//data_convert = data conversion function

void lsm6dso_handle_imu_sample(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type sensor,
    const uint8_t fifo_sample[6],
    sns_time ts,
    uint16_t sample_idx,
    log_sensor_state_raw_info* raw_log_ptr)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  const uint8_t* ip_raw = &fifo_sample[0];

  lsm6dso_std_sensor_event* prev_sample;
  sns_sensor_uid* suid;
  uint32_t *sample_count;
  sns_time ts_adjust;

  vector3 (*convert_raw_data)(sns_sensor_instance *const instance,
      const uint8_t* ip_raw, float* opdata_raw);

  lsm6dso_odr_change_info* config_ptr = &state->cur_odr_change_info;
  if(state->fifo_info.orphan_batch)
    config_ptr = &state->prev_odr_change_info;

  if(sensor == LSM6DSO_ACCEL) {
    prev_sample = &state->accel_info.sample;
    suid = &state->accel_info.suid;
    sample_count =  &state->accel_sample_counter;
    ts_adjust = sns_convert_ns_to_ticks(lsm6dso_odr_map[config_ptr->odr_idx].accel_group_delay * 1000000) + state->oem_ts_offset;
    convert_raw_data = &lsm6dso_accel_data_conversion;
  } else {
    prev_sample = &state->gyro_info.sample;
    suid = &state->gyro_info.suid;
    sample_count =  &state->gyro_sample_counter;
    ts_adjust = sns_convert_ns_to_ticks(lsm6dso_odr_map[config_ptr->odr_idx].gyro_group_delay * 1000000);
    convert_raw_data = &lsm6dso_gyro_data_conversion;
  }

  sns_std_sensor_sample_status status = lsm6dso_mark_sample_status(instance, sensor, sample_idx, config_ptr, ts);

  if((lsm6dso_odr_map[config_ptr->odr_idx].gyro_odr_reg_value == LSM6DSO_GYRO_ODR832) && !state->gyro_info.sw_lpf_data_settled)
  {
    sns_time elapsed_time = (ts - config_ptr->gyro_odr_settime);
    sns_time max_time =  sns_convert_ns_to_ticks(LSM6DSO_SW_FILTER_SETTLING_TIME * 1000 * 1000);
    if(IS_ODR_CHANGE_TIME_EXPIRED(elapsed_time, max_time))
      state->gyro_info.sw_lpf_data_settled = true;
  }

  if(status == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH) 
  {
    //data stabilized reset config sensors bit
    config_ptr->change_req &= ~sensor;
  }

  if((status == SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE) &&
      !(config_ptr->change_req & sensor)) {
    ip_raw = prev_sample->opdata_raw;
    status = prev_sample->opdata_status;
  }

  if(!(state->current_conf.publish_sensors & sensor) ||
      !(state->desired_conf.publish_sensors & sensor)) {
    DBG_INST_PRINTF_EX(HIGH, instance, "[%d]Skipping to publish s=0x%x dp:cp=0x%x:0x%x", state->hw_idx, sensor, state->desired_conf.publish_sensors, state->current_conf.publish_sensors);
    return;
  }
  LSM6DSO_INST_DEBUG_TS(LOW, instance, "s,ts,status  %d, %u, %u %d", sensor, (uint32_t)ts, (uint32_t)ts_adjust, status);
  LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, "s=%d sample time=%u", sensor, (uint32_t)ts);

  (*sample_count)++;

  if((*sample_count & 0x3FF) == 0)
  {
    DBG_INST_PRINTF(LOW, instance, "sensor=%d ts=%08X/%u #s=%u",
        sensor, (uint32_t)ts, (uint32_t)ts, *sample_count);
  }

  prev_sample->opdata_status = status;
  //update status only if sending data
  float opdata_raw[TRIAXIS_NUM] = {0};
  vector3 opdata_cal = convert_raw_data(instance, ip_raw, opdata_raw);

  if(status != SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE) {
    LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH,instance, "s: %d sample(*1000): %d, %d, %d",
        sensor,
        (int32_t)(opdata_cal.data[0]*1000),
        (int32_t)(opdata_cal.data[1]*1000),
        (int32_t)(opdata_cal.data[2]*1000));
    LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, " s: %d last_ts=%u count=%u",
        sensor, (uint32_t)ts, *sample_count);
  }

  float data[] = {
    [0] = opdata_cal.x,
    [1] = opdata_cal.y,
    [2] = opdata_cal.z,
  };

  pb_send_sensor_stream_event(instance,
      suid,
      ts - ts_adjust,
      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
      status,
      data,
      LSM6DSO_IMU_EVENT_AXIS,
      state->encoded_imu_event_len);

  lsm6dso_log_sample(instance, raw_log_ptr, suid, 
      state->log_raw_encoded_size,
      opdata_raw, LSM6DSO_NUM_AXES,
      ts - ts_adjust, status, 
      sample_idx ? SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE : SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST);
}

int64_t lsm6dso_timestamps_correction(sns_sensor_instance *instance,
    uint16_t sample_sets,
    sns_time irq_time,
    sns_time first_ts,
    sns_time last_ts,
    sns_time sample_interval_ticks)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  // QC - Please remove, if first_ts is unused varibale
  UNUSED_VAR(first_ts);
  int64_t ts_drift = irq_time - (last_ts + sample_sets * sample_interval_ticks);
  //int64_t first_ts_gap = first_ts - last_ts;
  //if last timestamp is not valid do not try to correct
  if((ts_drift == 0) || !state->fifo_info.last_ts_valid)
  {
    return 0;
  }
  //take 2% of sample_interval_ticks for all samples ts_drift
  //0.02 * sample_interval_ticks * sample_sets
  sns_time ts_catchup_min = (0.02f * sample_interval_ticks * sample_sets);
  int64_t ts_correction_drift;
  int64_t ts_correction = ts_catchup_min;

  if(ts_drift > 0) {
    ts_correction_drift = ts_drift - ts_catchup_min;
    //if > 0 use ts_catchup_min or use ts_drift as correction value
    if(ts_correction_drift < 0)
      ts_correction = ts_drift;
  }
  else {
    ts_correction = -ts_catchup_min;
    ts_correction_drift = ts_drift + ts_catchup_min;
    // if < 0 use ts_catchup_min or use ts_drift as correction value
    if(ts_correction_drift > 0)
      ts_correction = ts_drift;
  }
  //Floor function seems to have some issue in QCOM framework
  // QC - please elaborate what the issue is with Floor function
  // QC - Was the "issue" with the floor function fixed with "ts_correction = -ts_catchup_min;"?
   return (ts_correction < 0) ? ((ts_correction/sample_sets) - 1) : (ts_correction/sample_sets);
}
void lsm6dso_process_cfg_tag(
  sns_sensor_instance *instance,
  lsm6dso_odr_change_info* config,
  uint8_t gyro_startup)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  lsm6dso_sensor_type odr_changed = config->changed;
  UNUSED_VAR(odr_changed); // only used when debug print is enabled
  odr_reg_map const *odr_map = &lsm6dso_odr_map[config->odr_idx];
  if(config->changed & LSM6DSO_ACCEL) {
    state->accel_info.num_samples_to_discard = odr_map->accel_discard_samples;
    config->changed &= ~LSM6DSO_ACCEL;
  }
  if((state->current_conf.publish_sensors & LSM6DSO_GYRO) &&
    ((config->changed & LSM6DSO_GYRO) || config->gyro_startup)) {
    if(lsm6dso_odr_map[config->odr_idx].odr != LSM6DSO_ODR_832)
    config->gyro_startup = gyro_startup;

    if(config->gyro_startup) {
      DBG_INST_PRINTF_EX(HIGH, instance, "[%d]Gyro starting!", state->hw_idx);
    } else {
      state->gyro_info.num_samples_to_discard = odr_map->gyro_discard_samples;
      DBG_INST_PRINTF_EX(HIGH, instance, "[%d]Gyro started!", state->hw_idx);
    }
    config->changed &= ~LSM6DSO_GYRO;
    state->gyro_introduced = false;
  }

  if(!(state->current_conf.publish_sensors & LSM6DSO_GYRO))
    config->changed &= ~LSM6DSO_GYRO;

  DBG_INST_PRINTF(HIGH, instance, "changed=%x #discard(A/G)=%u/%u counter=%u/%u",
      odr_changed, state->accel_info.num_samples_to_discard,
      state->gyro_info.num_samples_to_discard,
      state->accel_sample_counter, state->gyro_sample_counter);
}

/* Define gyro startup byte position in the data - Low byte of Y-axis */
#define GYRO_STARTUP_DATA_POS (4)

void lsm6dso_process_fifo_data_buffer(
  sns_sensor_instance *instance,
  sns_time            first_timestamp,
  sns_time            use_time,
  sns_time            sample_interval_ticks,
  const uint8_t       *fifo_start,
  size_t              num_bytes,
  uint16_t            total_sample_sets,
  bool                is_first_batch
)
{
  UNUSED_VAR(is_first_batch);
  uint16_t accel_sample_sets = 0, gyro_sample_sets = 0, ref_sample_sets = 0;
  uint32_t i;
  uint8_t tag = 0;
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  sns_time timestamp = 0, ref_ts = 0;
  log_sensor_state_raw_info log_accel_state_raw_info, log_gyro_state_raw_info;

  sns_memset(&log_accel_state_raw_info, 0, sizeof(log_sensor_state_raw_info));
  sns_memset(&log_gyro_state_raw_info, 0, sizeof(log_sensor_state_raw_info));

  if(fifo_start == NULL)
  {
    SNS_INST_PRINTF(ERROR,instance,
        "ASCP buffer has NULL address!");
    state->num_ascp_null_events++;
    return;
  }

  //Temporary debug variable. To be removed
  state->fifo_start_address = (uint8_t *)fifo_start;

  LSM6DSO_INST_DEBUG_TS(HIGH, instance,
      "self_test alive %d ascp_req_count %d first_ts %u sample_interval_ticks %u",
      state->self_test_info.test_alive, state->ascp_req_count,
      (uint32_t)first_timestamp, (uint32_t)sample_interval_ticks);

  //if orphan batch or interrupt fired
  if((state->fifo_info.interrupt_cnt >= MAX_INTERRUPT_CNT) &&
      (state->fifo_info.orphan_batch ||
       state->fifo_info.bh_info.interrupt_fired)) {

    int64_t sample_time_correction = lsm6dso_timestamps_correction(
        instance, total_sample_sets,  use_time, first_timestamp,
        state->fifo_info.last_timestamp, sample_interval_ticks);

    LSM6DSO_INST_DEBUG_TS(HIGH, instance,
        "correction:  a/c= %u/%u correction = %d",
        (uint32_t)sample_interval_ticks,
        (uint32_t)(sample_interval_ticks+sample_time_correction),
        (int32_t)sample_time_correction);

    sample_interval_ticks += sample_time_correction;
    first_timestamp += sample_time_correction;

  }
  if(state->ag_stream_mode == S4S_SYNC) {
    lsm6dso_get_s4s_sample_interval(instance, total_sample_sets, &sample_interval_ticks,
        &first_timestamp, &use_time);
  }
  //reference timestamp
  timestamp = first_timestamp;
  lsm6dso_sensor_type sensor = 0;

  uint16_t* sample_sets;
  log_sensor_state_raw_info* raw_log_ptr;

  for(i = 0; i + STM_LSM6DSO_FIFO_SAMPLE_SIZE - 1 < num_bytes; i += STM_LSM6DSO_FIFO_SAMPLE_SIZE)
  {
    //initialize
    sensor = 0;
    tag = fifo_start[i] >> 3;
    uint8_t tag_cnt = (fifo_start[i] >> 1) & 0x03;
    if(STM_LSM6DSO_FIFO_TAG_GYRO == tag) {
      sensor = LSM6DSO_GYRO;
      raw_log_ptr = &log_gyro_state_raw_info;
      sample_sets = &gyro_sample_sets;
      ref_sample_sets = (accel_sample_sets > gyro_sample_sets) ? accel_sample_sets : gyro_sample_sets;
      state->fifo_info.gyro_tag_cnt = tag_cnt;
    }
    else if(STM_LSM6DSO_FIFO_TAG_ACCEL == tag) {
      sensor = LSM6DSO_ACCEL;
      raw_log_ptr = &log_accel_state_raw_info;
      sample_sets = &accel_sample_sets;
      ref_sample_sets = accel_sample_sets;
      state->fifo_info.accel_tag_cnt = tag_cnt;
    }
    if(sensor) {
      ref_ts = first_timestamp + (ref_sample_sets * sample_interval_ticks);
      timestamp = (ref_ts < timestamp) ? timestamp : ref_ts;
      if((state->fifo_info.gyro_extra_sample) &&
        (state->fifo_info.accel_tag_cnt == state->fifo_info.gyro_tag_cnt) &&
        (STM_LSM6DSO_FIFO_TAG_ACCEL == tag))
        timestamp = state->fifo_info.last_timestamp;

      if(timestamp <= use_time) {
        if(!state->self_test_info.test_alive) {
          lsm6dso_handle_imu_sample(
              instance, sensor, &fifo_start[i+1],
              timestamp,
              *sample_sets,
              raw_log_ptr);
        } else {
          //skip sending sample, but make sure all timestamps are updated
          DBG_INST_PRINTF_EX(HIGH, instance, "Skipping handle fifo sample.");
        }
        if((!state->fifo_info.gyro_extra_sample) || (STM_LSM6DSO_FIFO_TAG_GYRO == tag))
          (*sample_sets)++;

        state->fifo_info.gyro_extra_sample = false;
        state->fifo_info.last_timestamp = timestamp;

      } else {
        LSM6DSO_INST_DEBUG_TS(HIGH, instance,
            "Drop: sensor=%d last_ts=%u ts=%u use_time=%u ",
            sensor, (uint32_t)state->fifo_info.last_timestamp, (uint32_t)timestamp,
            (int32_t)use_time);
        //last_ts is used for PCE
        state->fifo_info.last_timestamp +=1;
      }
    }
    else if(STM_LSM6DSO_FIFO_TAG_CFG_CHANGE == tag) {
      uint8_t gyro_startup = fifo_start[i + GYRO_STARTUP_DATA_POS] & STM_LSM6DSO_FIFO_TAG_GYRO_START_MASK;
      if(!state->fifo_info.orphan_batch && state->cur_odr_change_info.changed)
        state->current_conf.publish_sensors = state->desired_conf.publish_sensors;
      lsm6dso_process_cfg_tag(instance,
          (state->fifo_info.orphan_batch) ? &state->prev_odr_change_info : &state->cur_odr_change_info,
          gyro_startup);
    }
#if LSM6DSO_LOG_VERBOSE_DEFAULT
    else
    {
      uint32_t ts = (uint32_t)(((fifo_start[i+4] << 24) & 0xFF000000) | ((fifo_start[i+3] << 16) & 0xFF0000) | ((fifo_start[i+2] << 8) & 0xFF00) | fifo_start[i+1]);
      DBG_INST_PRINTF(HIGH, instance, "[%d]Unused TAG=0x%x: 0x%x:0x%x:0x%x:0x%x ::: %d/1000",
        state->hw_idx, tag, fifo_start[i+1], fifo_start[i+2], fifo_start[i+3], fifo_start[i+4], ts*25);
    }
#endif
  }
  if(gyro_sample_sets > accel_sample_sets)
    state->fifo_info.gyro_extra_sample=true;

  //makesure to submit the packets at the end
  lsm6dso_log_sample(instance, &log_accel_state_raw_info, NULL, 0, NULL, 0, 0, 0, SNS_DIAG_BATCH_SAMPLE_TYPE_LAST);
  lsm6dso_log_sample(instance, &log_gyro_state_raw_info, NULL, 0, NULL, 0, 0, 0, SNS_DIAG_BATCH_SAMPLE_TYPE_LAST);

  LSM6DSO_INST_DEBUG_TS(HIGH, instance,
      "[use_time_drift]last_ts=%u use_time=%u drift=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)use_time,
      (int32_t)(use_time - state->fifo_info.last_timestamp));

  LSM6DSO_INST_DEBUG_TS(HIGH, instance,
      "[cur_time_drift]last_ts=%u cur_time=%u drift=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)sns_get_system_time(),
      (int32_t)(sns_get_system_time() - state->fifo_info.last_timestamp));
}

sns_time lsm6dso_get_cfg_tag_ts(
    sns_sensor_instance *instance,
    const uint8_t* buffer,
    uint16_t tag_pos)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  lsm6dso_odr_change_info* odr_change_info = (state->fifo_info.orphan_batch) ? &state->prev_odr_change_info : &state->cur_odr_change_info;
  sns_time odr_settime = 0;
  uint16_t ts_pos = tag_pos + STM_LSM6DSO_FIFO_SAMPLE_SIZE;
  uint32_t ts = (uint32_t)(((buffer[ts_pos + 4] << 24) & 0xFF000000) | ((buffer[ts_pos + 3] << 16) & 0xFF0000) |
      ((buffer[ts_pos + 2] << 8) & 0xFF00) | buffer[ts_pos + 1]);
  float ts_ns = (ts * 1000000000.0f)/(LSM6DSO_HW_TS_CLOCK_RATE * state->clock_trim_factor);
  sns_time ts_ticks = sns_convert_ns_to_ticks(ts_ns);
  odr_settime = odr_change_info->hw_timer_start_time;


  LSM6DSO_INST_DEBUG_TS(HIGH, instance, "[%d] ts=%u ts_ticks=%u hw_start_t=%u first_ts=%u",
      state->hw_idx, ts, (uint32_t)ts_ticks, (uint32_t)(odr_settime), (uint32_t)(odr_settime + ts_ticks));
  return (!odr_settime || !ts_ticks) ? 0 : (odr_settime + ts_ticks);
}

bool lsm6dso_get_cfg_tag_pos(
    sns_sensor_instance *instance,
    const uint8_t* buffer,
    uint32_t bytes,
    uint16_t* tag_idx,
    sns_time*  cfg_tag_ts)
{
  UNUSED_VAR(instance);
  uint8_t tag = 0;
  for(uint_fast16_t i = 0; i < bytes; i += STM_LSM6DSO_FIFO_SAMPLE_SIZE)
  {
    tag = buffer[i] >> 3;
    if(STM_LSM6DSO_FIFO_TAG_CFG_CHANGE == tag) {
      *tag_idx = i;
      *cfg_tag_ts = lsm6dso_get_cfg_tag_ts(instance, buffer, i);
      LSM6DSO_INST_DEBUG_TS(LOW, instance, "cfg_tag-idx=%d tag_ts=%u", *tag_idx/STM_LSM6DSO_FIFO_SAMPLE_SIZE, (uint32_t)*cfg_tag_ts);
      return (!(*cfg_tag_ts)) ? false : true;
    }
  }
  return false;
}


bool lsm6dso_count_sample_sets(
    sns_sensor_instance *instance,
    const uint8_t* buffer,
    uint32_t bytes,
    uint16_t* restrict accel_count,
    uint16_t* restrict gyro_count)
{
  UNUSED_VAR(instance);
  uint8_t tag = 0;
  *accel_count = 0;
  *gyro_count = 0;
  bool cfg_tag_found = false;
  uint16_t tag_idx = 0;
  for(uint_fast16_t i = 0; i < bytes; i += STM_LSM6DSO_FIFO_SAMPLE_SIZE)
  {
    tag = buffer[i] >> 3;
    if(STM_LSM6DSO_FIFO_TAG_GYRO == tag) {
      (*gyro_count)++;
    } else if(STM_LSM6DSO_FIFO_TAG_ACCEL == tag) {
      (*accel_count)++;
    } else if(STM_LSM6DSO_FIFO_TAG_CFG_CHANGE == tag) {
      if(!cfg_tag_found)
        tag_idx = i;
      cfg_tag_found = true;
    }
  }
  LSM6DSO_INST_DEBUG_TS(LOW, instance, "cfg_tag/idx=%d/%d a/g=%d/%d ", cfg_tag_found, tag_idx/STM_LSM6DSO_FIFO_SAMPLE_SIZE, *accel_count, *gyro_count);
  return cfg_tag_found;
}

/*estimated end timestamp based on sampling intvl, number of sample set and interrupt ts*/
sns_time lsm6dso_get_use_time(sns_sensor_instance *this,
                              uint16_t num_sample_sets,
                              sns_time sampling_intvl,
                              sns_time cfg_tag_ts,
                              bool     cfg_tag_ts_available)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)this->state->state;
  uint16_t cur_wmk = state->fifo_info.bh_info.wmk;
  sns_time cur_time = state->fifo_info.bh_info.cur_time;
  // use_time is the boarder ts, calculated timestamps should not go beyond this
  sns_time use_time = state->fifo_info.bh_info.interrupt_ts;

  
  if(state->fifo_info.bh_info.interrupt_fired) {
    use_time = state->fifo_info.bh_info.interrupt_ts + (sampling_intvl * ((num_sample_sets >= cur_wmk) ? (num_sample_sets - cur_wmk) : 0 ));
  } else if(cfg_tag_ts_available) {
    use_time = cfg_tag_ts + sampling_intvl * (num_sample_sets - 1); 
  } else if((!state->fifo_info.interrupt_cnt) && (!state->fifo_info.last_ts_valid)) {
    //For flush only use-case, use flush time to mark use-time so that samples can be inserted to avoid drift.
    use_time = state->fifo_info.bh_info.interrupt_ts;
  } else {
    //defines time when the data request sent, does not represent actual cur time
    use_time = state->fifo_info.last_timestamp + sampling_intvl * num_sample_sets;
  }

  LSM6DSO_INST_DEBUG_TS(HIGH, this,
      "use_ts: use_time=%u cur_time=%u last_ts=%u ",
      (uint32_t)use_time, (uint32_t)cur_time,(uint32_t)state->fifo_info.last_timestamp);
  //if use time is greater than cur_time
  if((use_time > cur_time) && (state->fifo_info.bh_info.is_dae_ts_reliable)) {
    if(state->fifo_info.bh_info.interrupt_fired && (num_sample_sets == cur_wmk))
      use_time = state->fifo_info.bh_info.interrupt_ts;
    else if(!state->fifo_info.bh_info.interrupt_fired)
      use_time = cur_time;
  }

  return use_time;
}

sns_time lsm6dso_get_first_ts(sns_sensor_instance *this,
                              uint16_t num_sample_sets,
                              sns_time sampling_intvl,
                              sns_time use_time,
                              bool cfg_tag_present)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)this->state->state;
  //one of the timestamp should be accurate, either last_ts valid or irq ts
  sns_time first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;

  LSM6DSO_INST_DEBUG_TS(HIGH, this,
      "get_first_ts: first_ts=%u last_ts_valid=%d ", (uint32_t)first_timestamp, state->fifo_info.last_ts_valid);
  if(!state->fifo_info.last_ts_valid &&
      state->fifo_info.bh_info.interrupt_fired) {
    first_timestamp = use_time - sampling_intvl * (num_sample_sets - 1);
    int64_t first_ts_gap = first_timestamp - state->fifo_info.last_timestamp;

    LSM6DSO_INST_DEBUG_TS(HIGH, this,
      "get_first_ts: first_ts_gap=%d sam_int=%u use_time=%u", first_ts_gap, (uint32_t)sampling_intvl, (uint32_t)use_time);
    //if streaming just started, use irq timestamp to calculate first ts
    if(cfg_tag_present && state->cur_odr_change_info.changed && !state->prev_odr_change_info.odr_idx)
      return first_timestamp;

    //either we cross last ts or
    //DAE did not drop any samples(last_ts_valid is used to check if DAE dropped some data or not)
    if(first_ts_gap < 0 || (first_ts_gap < (sampling_intvl + state->prev_odr_change_info.nominal_sampling_intvl)) ||
      (lsm6dso_dae_if_available(this) && (first_ts_gap < sampling_intvl * state->fifo_info.cur_wmk))) {
      first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;
      state->fifo_info.last_ts_valid = true;
    }
  }
  return first_timestamp;
}

void lsm6dso_calculate_sampling_intvl(sns_sensor_instance *this,
    uint16_t num_sample_sets,
    sns_time* sampling_intvl)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)this->state->state;
  uint16_t cur_wmk = state->fifo_info.cur_wmk;
  sns_time cal_st  = 0;
  LSM6DSO_INST_DEBUG_TS(HIGH, this,
      "calculate_ts: last_ts=%u irq_ts=%u num_sample_sets=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)state->fifo_info.bh_info.interrupt_ts,
      num_sample_sets);
  if(state->fifo_info.bh_info.accel_odr == state->fifo_info.fifo_rate)
  {
    if((state->fifo_info.bh_info.interrupt_fired) && (num_sample_sets == cur_wmk)) {
      *sampling_intvl = lsm6dso_estimate_avg_st(this, state->fifo_info.bh_info.interrupt_ts, num_sample_sets);
    } else if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) {
      state->fifo_info.interrupt_cnt = 0;
      cal_st = state->fifo_info.avg_sampling_intvl;
      if(state->fifo_info.bh_info.interrupt_fired) {
        cal_st = (state->fifo_info.bh_info.interrupt_ts -
            state->fifo_info.last_timestamp) / cur_wmk;
      }
      state->fifo_info.avg_sampling_intvl = cal_st;
      if(!LSM6DSO_IS_INBOUNDS(cal_st, state->cur_odr_change_info.nominal_sampling_intvl,
            LSM6DSO_ODR_TOLERANCE )) {
        state->fifo_info.avg_sampling_intvl = state->cur_odr_change_info.nominal_sampling_intvl;
      }
      *sampling_intvl = state->fifo_info.avg_sampling_intvl;
    }
  }
  else if(state->fifo_info.bh_info.accel_odr ==  lsm6dso_odr_map[state->prev_odr_change_info.odr_idx].accel_odr_reg_value)  {
    /* must be last batch of samples of previous ODR */
    *sampling_intvl = state->prev_odr_change_info.nominal_sampling_intvl;
    DBG_INST_PRINTF(MED, this,
        "calculate_ts: orphan batch odr=0x%x sampling_intv=%u #set=%u",
        state->fifo_info.bh_info.accel_odr, (uint32_t)*sampling_intvl, num_sample_sets);
  }
  else {
    float odr = lsm6dso_get_accel_odr(state->fifo_info.bh_info.accel_odr);
    *sampling_intvl = lsm6dso_get_sample_interval(this, odr);
    DBG_INST_PRINTF(MED, this,
        "calculate_ts: older to orphan batch odr=0x%x sampling_intv=%u #set=%u",
        state->fifo_info.bh_info.accel_odr, (uint32_t)*sampling_intvl, num_sample_sets);
  }
}

void lsm6dso_send_missing_samples(
    sns_sensor_instance *instance,
    bool gyro_enabled,
    sns_time sampling_intvl,
    sns_time use_time,
    uint16_t missing_samples)
{

  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;

  lsm6dso_log_hw_conf(instance);
  DBG_INST_PRINTF_EX(HIGH, instance,
      "missing_samples=%u intv=%u use_time=%u last_ts=%u orphan=%d",
      missing_samples, (uint32_t)sampling_intvl, (uint32_t)use_time,
      (uint32_t)state->fifo_info.last_timestamp, state->fifo_info.orphan_batch);

  if(missing_samples > 0 && missing_samples <= MAX_MISSING_SAMPLES) {
    sns_time first_timestamp = state->fifo_info.last_timestamp;
    uint16_t idx = 0, i = 0;
    uint8_t buffer[STM_LSM6DSO_FIFO_SAMPLE_SIZE << 1];
    uint8_t sample_size = STM_LSM6DSO_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);

    if(gyro_enabled) {
      buffer[idx] = STM_LSM6DSO_FIFO_TAG_GYRO << 3;
      sns_memscpy(&buffer[idx + STM_LSM6DSO_TAG_SIZE],
          STM_LSM6DSO_SAMPLE_SIZE,
          state->gyro_info.sample.opdata_raw,
          STM_LSM6DSO_SAMPLE_SIZE);
      idx += STM_LSM6DSO_FIFO_SAMPLE_SIZE;
    }
    buffer[idx] = STM_LSM6DSO_FIFO_TAG_ACCEL << 3;
    sns_memscpy(&buffer[idx + STM_LSM6DSO_TAG_SIZE],
        STM_LSM6DSO_SAMPLE_SIZE,
        state->accel_info.sample.opdata_raw,
        STM_LSM6DSO_SAMPLE_SIZE);
    if((state->gyro_info.num_samples_to_discard != 0) ||
      (state->gyro_info.sample.opdata_status == SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE)) {
      state->gyro_info.num_samples_to_discard += missing_samples;
    }

    if((state->accel_info.num_samples_to_discard != 0) ||
      (state->accel_info.sample.opdata_status == SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE)) {
      state->accel_info.num_samples_to_discard += missing_samples;
    }

    sns_time end_ts = use_time;
    sns_time cal_st = (use_time - state->fifo_info.last_timestamp)/missing_samples; // QC - what guarantees that use_time is greater than last_timestamp?

    if(!LSM6DSO_IS_INBOUNDS(cal_st, sampling_intvl, LSM6DSO_ODR_TOLERANCE))
      cal_st = sampling_intvl;

    for(i = 0; i < missing_samples; i++) {
      first_timestamp = state->fifo_info.last_timestamp + cal_st;
      end_ts = (i == (missing_samples - 1)) ? use_time : (state->fifo_info.last_timestamp + cal_st);
      lsm6dso_process_fifo_data_buffer(instance,
          first_timestamp,
          end_ts,
          cal_st,
          buffer,
          sample_size,
          1,
          false);
    }
  }
  else if(missing_samples > 0)
    state->fifo_info.last_ts_valid = false;
}

bool lsm6dso_adjust_ts_drift(
    sns_sensor_instance *instance,
    bool gyro_enabled,
    sns_time sampling_intvl,
    sns_time last_ts,
    sns_time use_time)
{

  //check the drift w.r.t interrupt ts
  //in flush case fifo_info.ts stores the time when flush started
  //in interrupt case fifo_info.ts stores the time when int received
  //try end timestamp to be closer to this
  int64_t ts_drift;
  float low_bound_odr_tolerance = (100 - LSM6DSO_ODR_TOLERANCE)/100.0f;
  sns_time low_bound_sampling_intvl = low_bound_odr_tolerance  * sampling_intvl;
  uint32_t missing_samples = 0;

  ts_drift = use_time - last_ts; // QC - what guarantees that use_time is greater than last_ts?
  //in case of flush- cur time is flush time
  if(ts_drift > 0) {
    missing_samples = ts_drift / low_bound_sampling_intvl;
  }

  if(missing_samples > 0) {
    DBG_INST_PRINTF(HIGH, instance,
        "#missing=%u intv=%u use_time=%u last_ts=%u",
        missing_samples, (uint32_t)sampling_intvl, (uint32_t)use_time,
        (uint32_t)last_ts);
    lsm6dso_send_missing_samples(
        instance,
        gyro_enabled,
        sampling_intvl,
        use_time,
        missing_samples);

  }
  return (missing_samples ? true : false);

}

void lsm6dso_update_ag_config_event(
    sns_sensor_instance *instance,
    bool new_client)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  float sample_rate = (float)(13 << ((state->fifo_info.bh_info.accel_odr >> 4) - 1));
  bool is_config_event_required = false, is_wmk_only_change = false;
  //if the received data is orphan and streaming is stopped; no need to send config event
  if(state->fifo_info.orphan_batch && 
     !(state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)))
    return;

  //If its orphan samples then send the config event with the odr reported by orphan sample odr
  if(!state->fifo_info.orphan_batch){
    if((state->fifo_info.bh_info.wmk  != state->fifo_info.last_sent_config.fifo_watermark)
      && (sample_rate == state->fifo_info.last_sent_config.sample_rate))
      is_wmk_only_change = true;
#if LSM6DSO_DAE_ENABLED
    if(state->fifo_info.bh_info.wmk  != state->fifo_info.last_sent_config.fifo_watermark ||
       sample_rate != state->fifo_info.last_sent_config.sample_rate  ||
       ((state->fifo_info.max_requested_wmk != state->fifo_info.last_sent_config.dae_watermark) 
        && state->fifo_info.bh_info.interrupt_fired))
#else
    if(state->fifo_info.bh_info.wmk  != state->fifo_info.last_sent_config.fifo_watermark ||
       sample_rate != state->fifo_info.last_sent_config.sample_rate)
#endif
         is_config_event_required = true;
  }else if((sample_rate = lsm6dso_odr_map[state->prev_odr_change_info.odr_idx].odr) != state->fifo_info.last_sent_config.sample_rate){
          is_config_event_required = true;
  }
  LSM6DSO_INST_DEBUG_TS(HIGH, instance, "ag_config_event orphan %d pre %d sr %d last %d",state->fifo_info.orphan_batch, state->prev_odr_change_info.odr_idx, sample_rate,state->fifo_info.last_sent_config.sample_rate);
  if(is_config_event_required)
  {
    state->fifo_info.new_config.fifo_watermark = state->fifo_info.bh_info.wmk;
    state->fifo_info.new_config.sample_rate    = sample_rate;
    if(state->fifo_info.last_ts_valid)
      state->cur_odr_change_info.odr_change_timestamp = state->fifo_info.last_timestamp+1;
    if(state->fifo_info.orphan_batch)
      state->fifo_info.new_config.timestamp = state->prev_odr_change_info.odr_change_timestamp;
    else
      state->fifo_info.new_config.timestamp = state->cur_odr_change_info.odr_change_timestamp;
    //Update gated config ts since wmk is changing
#if LSM6DSO_DAE_ENABLED
    if(state->accel_info.gated_client_present
         && !(state->desired_conf.publish_sensors & LSM6DSO_ACCEL)
         && ((state->current_conf.wmk != state->fifo_info.last_sent_config.fifo_watermark) ||
            (state->fifo_info.max_requested_wmk != state->fifo_info.last_sent_config.dae_watermark)))
#else
    if(state->accel_info.gated_client_present
         && !(state->desired_conf.publish_sensors & LSM6DSO_ACCEL)
         && (state->current_conf.wmk != state->fifo_info.last_sent_config.fifo_watermark))
#endif
      state->fifo_info.new_config.gated_timestamp = state->cur_odr_change_info.odr_change_timestamp;

    if(is_wmk_only_change)
      state->fifo_info.new_config.timestamp = state->cur_odr_change_info.odr_change_timestamp = state->fifo_info.last_timestamp;

#if LSM6DSO_DAE_ENABLED
    state->fifo_info.new_config.dae_watermark  = state->fifo_info.max_requested_wmk;
#endif
    lsm6dso_send_config_event(instance, new_client);
  }
}

/* used only for the samples following odr changes */
/* samples just after ODR change usecase *
 * first sample might have generated with prev sample rate *
 * check the gap: if it matches 95% of cur sample rate, if not check with prev sample rate and send
 * send tag and samples if any left */
bool lsm6dso_send_orphan_samples(
                            sns_sensor_instance *instance,
                            const uint8_t* buffer,
                            uint32_t bytes,
                            uint16_t sample_sets,
                            bool gyro_enabled,
                            sns_time first_ts,
                            bool cfg_tag_available)
{
  UNUSED_VAR(buffer);
  UNUSED_VAR(bytes);
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  sns_time use_time = lsm6dso_get_use_time(instance, sample_sets, state->fifo_info.avg_sampling_intvl, first_ts, cfg_tag_available);

  float low_bound_odr_tolerance = (100 - LSM6DSO_ODR_TOLERANCE)/100.0f;
  sns_time low_bound_sampling_intvl_cur = low_bound_odr_tolerance  * state->fifo_info.avg_sampling_intvl;
  sns_time low_bound_sampling_intvl_prev = low_bound_odr_tolerance  * state->prev_odr_change_info.nominal_sampling_intvl;

  int32_t ts_gap = (int64_t)first_ts - (int64_t)state->fifo_info.last_timestamp;
  //represents if any orphan sample in the current batch
  int8_t sent_samples = 0;
  sns_time sampling_intvl;
  sns_time est_use_time;
  uint16_t orphan_samples = 0;

  LSM6DSO_INST_DEBUG_TS(HIGH, instance, "orphan_samples: odr_idx cur:prev=%d:%d orphan=%d last_ts_valid=%d",
      state->cur_odr_change_info.odr_idx, state->prev_odr_change_info.odr_idx,
      state->cur_odr_change_info.odr_idx, state->prev_odr_change_info.odr_idx,
      state->fifo_info.orphan_batch, state->fifo_info.last_ts_valid);

  if((ts_gap <= 0) ||
    (state->prev_odr_change_info.odr_idx == 0) ||
    state->fifo_info.orphan_batch ||
      (state->prev_odr_change_info.odr_idx == state->cur_odr_change_info.odr_idx) ||
      !state->fifo_info.last_ts_valid) {
    return sent_samples;
  }

  // from high --> low only
  if(state->prev_odr_change_info.odr_idx > state->cur_odr_change_info.odr_idx) {
    if(ts_gap > low_bound_sampling_intvl_cur) {
      ts_gap -= (ts_gap/low_bound_sampling_intvl_cur)*low_bound_sampling_intvl_cur;
      sent_samples--;
    }
    if(ts_gap > 0)
      orphan_samples = ts_gap / low_bound_sampling_intvl_prev;
  } else {
    //low --> high
    orphan_samples = ts_gap / low_bound_sampling_intvl_prev;
    if((ts_gap - orphan_samples * low_bound_sampling_intvl_prev) >= low_bound_sampling_intvl_cur)
      sent_samples--;
  }
  est_use_time = state->fifo_info.last_timestamp + (orphan_samples *  state->prev_odr_change_info.nominal_sampling_intvl);
  sampling_intvl = state->prev_odr_change_info.nominal_sampling_intvl;

  LSM6DSO_INST_DEBUG_TS(HIGH, instance, "#orphan missing: use_time=%u first_ts=%u ts_gap=%d missing=%d",
      (uint32_t)use_time, (uint32_t)first_ts,
      (int32_t)ts_gap, orphan_samples);

  if(orphan_samples) {
    if(est_use_time > use_time)
      est_use_time = use_time;

    bool is_orphan_batch = state->fifo_info.orphan_batch;
    state->fifo_info.orphan_batch = true;
    //Send config event if there is any change in odr/wm
    lsm6dso_update_ag_config_event(instance, false);
    lsm6dso_send_missing_samples(
        instance,
        gyro_enabled,
        sampling_intvl,
        est_use_time,
        orphan_samples);
    state->fifo_info.orphan_batch = is_orphan_batch;
    sent_samples++;
  }
  return (sent_samples >= 0 ? sent_samples : 0);
}



void lsm6dso_process_imu_data(
    sns_sensor_instance *instance,
    const uint8_t* fifo_head,
    uint32_t bytes,
    bool gyro_enabled,
    bool cfg_tag_present,
    uint16_t tag_pos,
    sns_time cfg_tag_ts)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  uint16_t num_sample_sets;
  uint16_t accel_count, gyro_count;
  sns_time use_time;
  sns_time sampling_intvl = state->fifo_info.avg_sampling_intvl;
  sns_time first_timestamp;
  lsm6dso_odr_change_info* odr_changed = (state->fifo_info.orphan_batch) ? &state->prev_odr_change_info : &state->cur_odr_change_info;
  lsm6dso_odr_change_info temp_odr_changed;
  sns_memscpy(&temp_odr_changed, sizeof(lsm6dso_odr_change_info), odr_changed, sizeof(lsm6dso_odr_change_info));

  bool cfg_tag_ts_available = false; 

  lsm6dso_count_sample_sets(instance, fifo_head, bytes, &accel_count, &gyro_count);

  num_sample_sets = SNS_MAX(accel_count, gyro_count);
  const uint8_t* buffer = &fifo_head[0];

  LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, "c_wmk=%u bytes=%u",
      state->fifo_info.cur_wmk, bytes);
  LSM6DSO_INST_DEBUG_TS(LOW, instance,
      "cur_wmk=%u bytes=%u num_sample_sets=%d",
      state->fifo_info.cur_wmk, bytes, num_sample_sets);

  if(num_sample_sets >= 1)
  {
    sns_time est_first_ts = 0;
    //TAG for gyro may come multiple times, avoid to enter multiple times
    if(cfg_tag_present && (temp_odr_changed.changed & LSM6DSO_ACCEL) && (tag_pos == 0)) {
      if((state->fifo_info.bh_info.wmk == 1) && state->fifo_info.bh_info.interrupt_fired) {
        est_first_ts = state->fifo_info.bh_info.interrupt_ts;
      } else {
        est_first_ts = cfg_tag_ts;
        cfg_tag_ts_available = true;
      }

      //check to see if first sample to go with prev sample rate
      bool orphan_sample_sent = lsm6dso_send_orphan_samples(instance, buffer, bytes, num_sample_sets, gyro_enabled, est_first_ts, cfg_tag_ts_available);
      if(orphan_sample_sent) {
        num_sample_sets--;
        state->fifo_info.new_config.timestamp  = state->fifo_info.last_timestamp;
      }
      //update last ts, if last_ts is invalid and interrupt is not fired (only flush case)
      else if(!state->fifo_info.last_ts_valid && !state->fifo_info.bh_info.interrupt_fired) {
        state->fifo_info.last_timestamp = est_first_ts -  
          (state->fifo_info.orphan_batch ? state->prev_odr_change_info.nominal_sampling_intvl : state->fifo_info.avg_sampling_intvl);
      }
    }
    //Send config event if there is any change in odr/wm
    lsm6dso_update_ag_config_event(instance, false);

    if(num_sample_sets)
      lsm6dso_calculate_sampling_intvl(instance, num_sample_sets, &sampling_intvl);

    //use_time is the border ts, calculated timestamps should not go beyond this
    use_time = lsm6dso_get_use_time(instance, num_sample_sets, sampling_intvl, est_first_ts, cfg_tag_ts_available);
    //calculate first timestamp
    first_timestamp = lsm6dso_get_first_ts(instance, num_sample_sets, sampling_intvl, use_time, cfg_tag_present);

    if((state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) ||
        (cfg_tag_present && temp_odr_changed.changed))
    {
      float low_bound_odr_tolerance = (100 - LSM6DSO_ODR_TOLERANCE)/100.0f;
      sns_time low_bound_sampling_intvl = low_bound_odr_tolerance  * sampling_intvl;
      sns_time expected_end_ts = first_timestamp + (num_sample_sets - 1) * sampling_intvl;

      //adjust drift only when expected end ts is less than use_time
      int64_t drift = (use_time >= expected_end_ts) ? (use_time - expected_end_ts) : 0 ;
      int32_t missing = drift / low_bound_sampling_intvl;

      if(missing) {

        DBG_INST_PRINTF(HIGH, instance,
            "#missing=%u intv=%u use_time=%u first_ts=%u",
            missing, (uint32_t)sampling_intvl, (uint32_t)use_time, (uint32_t)first_timestamp);
        // Adjust sampling intvl if (missing + sample sets) time is greater than use time
        if(first_timestamp + (sampling_intvl * (missing + num_sample_sets - 1)) > use_time)
          sampling_intvl = (use_time - first_timestamp) / (missing +  num_sample_sets - 1);
        //is sampling_intvl need to be validated here?
        lsm6dso_send_missing_samples(instance, state->gyro_introduced ? false : gyro_enabled, 
            sampling_intvl,
            first_timestamp + (missing - 1) * sampling_intvl,
            missing);
        //update first_ts and use_time
        first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;
        use_time = lsm6dso_get_use_time(instance, num_sample_sets, sampling_intvl, first_timestamp, false);
      }
    }

    sns_memscpy(odr_changed, sizeof(lsm6dso_odr_change_info), &temp_odr_changed, sizeof(lsm6dso_odr_change_info));

    LSM6DSO_INST_DEBUG_TS(LOW, instance,
        "sampling_intv=%u last_ts=%u first_ts=%u use_time=%u",
        (uint32_t)sampling_intvl, (uint32_t)state->fifo_info.last_timestamp,
        (uint32_t)first_timestamp, (uint32_t)use_time);

    LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, "sampling_intv=%u last_ts=%u first_ts=%u",
        (uint32_t)sampling_intvl, (uint32_t)state->fifo_info.last_timestamp,
        (uint32_t)first_timestamp);

    lsm6dso_process_fifo_data_buffer(instance,
        first_timestamp,
        use_time,
        sampling_intvl,
        buffer,
        bytes,
        num_sample_sets,
        (cfg_tag_present && odr_changed->changed));

    state->fifo_info.last_ts_valid = true;

  }
}
void lsm6dso_send_fifo_data(
    sns_sensor_instance *instance,
    const uint8_t* fifo_head,
    uint32_t bytes,
    bool gyro_enabled)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
  lsm6dso_odr_change_info* odr_change_info = (state->fifo_info.orphan_batch) ? &state->prev_odr_change_info : &state->cur_odr_change_info;
  uint16_t tag_pos = 0;
  sns_time cfg_tag_ts = 0;
  bool cfg_tag_present = false;

  //update current time based on number of samples of accel and gyro
  if(state->fifo_info.bh_info.interrupt_fired) {
    uint16_t xl_cnt = 0, g_cnt=0;
    lsm6dso_count_sample_sets(instance, fifo_head, bytes, &xl_cnt, &g_cnt);
    uint16_t max_cnt = SNS_MAX(xl_cnt, g_cnt);
    if(max_cnt > state->fifo_info.bh_info.wmk) {
      state->fifo_info.bh_info.cur_time = state->fifo_info.bh_info.interrupt_ts + (max_cnt -  state->fifo_info.bh_info.wmk) * odr_change_info->nominal_sampling_intvl;
      sns_time cur_time = sns_get_system_time();
      if(state->fifo_info.bh_info.cur_time > cur_time) {
        state->fifo_info.bh_info.cur_time = cur_time;
      }
      LSM6DSO_INST_DEBUG_TS(LOW, instance, "[%d] orphan = %d total_samples:wmk = %d:%d cur_time(bh_c:c)=%u:%u", 
          state->hw_idx, state->fifo_info.orphan_batch, max_cnt, state->fifo_info.bh_info.wmk, (uint32_t)state->fifo_info.bh_info.cur_time, (uint32_t)cur_time); 
    }
  }

  LSM6DSO_INST_DEBUG_TS(LOW, instance, "[%d] orphan = %d odr_changed = %d hw_timer_start(p:c)=%u:%u", 
      state->hw_idx, state->fifo_info.orphan_batch, odr_change_info->changed,
      (uint32_t)state->prev_odr_change_info.hw_timer_start_time, (uint32_t)state->cur_odr_change_info.hw_timer_start_time);

  if(odr_change_info->changed) {
    bool irq_fired = state->fifo_info.bh_info.interrupt_fired;
    sns_time irq_ts = state->fifo_info.bh_info.interrupt_ts;
    cfg_tag_present = lsm6dso_get_cfg_tag_pos(instance, fifo_head, bytes, &tag_pos, &cfg_tag_ts);
    if(cfg_tag_present && tag_pos) {
      state->fifo_info.bh_info.interrupt_fired = false;
      state->fifo_info.bh_info.interrupt_ts = cfg_tag_ts - 
        (state->fifo_info.orphan_batch ? state->prev_odr_change_info.nominal_sampling_intvl : state->fifo_info.avg_sampling_intvl);
      lsm6dso_process_imu_data(instance, fifo_head, tag_pos, gyro_enabled, cfg_tag_present, tag_pos, cfg_tag_ts);
      bytes -= tag_pos;
      state->fifo_info.bh_info.interrupt_fired = irq_fired;
      state->fifo_info.bh_info.interrupt_ts = irq_ts;
    }
    if(state->gyro_introduced && !state->fifo_info.orphan_batch && cfg_tag_present)
       state->fifo_info.bh_info.interrupt_fired = false;
  }
  lsm6dso_process_imu_data(instance, &fifo_head[tag_pos], bytes, gyro_enabled, cfg_tag_present, 0, cfg_tag_ts);
  if(cfg_tag_present && (odr_change_info->changed & LSM6DSO_GYRO))
  {
    if(state->gyro_introduced) {
      state->fifo_info.avg_sampling_intvl = state->cur_odr_change_info.nominal_sampling_intvl - 
        (state->odr_percent_var_gyro / state->desired_conf.odr);
      lsm6dso_interrupt_interval_init_nominal(instance);
    }
      state->gyro_introduced = false;
  }
}

void lsm6dso_process_com_port_vector(sns_port_vector *vector, void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  if(STM_LSM6DSO_REG_FIFO_OUT_TAG == vector->reg_addr)
  {
    lsm6dso_send_fifo_data(instance, vector->buffer, vector->bytes, gyro_enabled);
  }
}


/** See lsm6dso_hal.h */
void lsm6dso_send_fifo_flush_done(sns_sensor_instance *instance,
                                  lsm6dso_sensor_type flushing_sensors,
                                  lsm6dso_flush_done_reason reason)
{
  UNUSED_VAR(reason);

  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  SNS_INST_PRINTF(HIGH, instance, "[%u] FLUSH_EVENT sensor=%x (%u) %u/%u/%u",
                  state->hw_idx, flushing_sensors, reason, state->accel_sample_counter,
                  state->gyro_sample_counter, state->num_temp_samples);

  while(flushing_sensors != 0)
  {
    sns_sensor_uid const *suid = NULL;
    lsm6dso_sensor_type sensor_type = LSM6DSO_ACCEL;
    if(flushing_sensors & LSM6DSO_ACCEL)
    {
      suid = &state->accel_info.suid;
      sensor_type = LSM6DSO_ACCEL;
    }
    else if(flushing_sensors & LSM6DSO_GYRO)
    {
      suid = &state->gyro_info.suid;
      sensor_type = LSM6DSO_GYRO;
    }
    else if(flushing_sensors & LSM6DSO_MOTION_DETECT)
    {
      suid = &state->md_info.suid;
      sensor_type = LSM6DSO_MOTION_DETECT;
    }
    else if(flushing_sensors & LSM6DSO_SENSOR_TEMP)
    {
      suid = &state->sensor_temp_info.suid;
      sensor_type = LSM6DSO_SENSOR_TEMP;
    }
    else
    {
      flushing_sensors = 0;
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
    }
  }
}


//estimate average sample time
sns_time lsm6dso_estimate_avg_st(
  sns_sensor_instance *const instance,
  sns_time irq_timestamp,
  uint16_t num_samples)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  uint32_t cur_int_delta = (uint32_t)(irq_timestamp - state->fifo_info.interrupt_timestamp);
  sns_time sampling_intvl = state->fifo_info.avg_sampling_intvl;

  if(state->fifo_info.interrupt_cnt == 0) {
    if(state->fifo_info.last_ts_valid)
      cur_int_delta = (uint32_t)(irq_timestamp - state->fifo_info.last_timestamp);
    else
      cur_int_delta = state->fifo_info.avg_interrupt_intvl;
  }
  float odr_tolerance = LSM6DSO_ODR_TOLERANCE;
  if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) {
    odr_tolerance += 1;
  }

  state->fifo_info.interrupt_cnt++;
  if(LSM6DSO_IS_INBOUNDS(cur_int_delta/num_samples, state->cur_odr_change_info.nominal_sampling_intvl,
        odr_tolerance)) {

    if(state->fifo_info.interrupt_cnt > MAX_INTERRUPT_CNT) {

      uint32_t avg_int;
      uint16_t int_cnt;
      int32_t odr_percent_var;
      if(state->fifo_info.interrupt_cnt == UINT16_MAX)
        state->fifo_info.interrupt_cnt = WINDOW_SIZE + MAX_INTERRUPT_CNT;

      // QC - Is it guaranteed that fifo_info.interurpt_cnt + 1 is greater than MAX_INTERRUPT_CNT?
      int_cnt = state->fifo_info.interrupt_cnt - MAX_INTERRUPT_CNT + 1;
      if(state->fifo_info.interrupt_cnt >= WINDOW_SIZE+MAX_INTERRUPT_CNT)
        int_cnt = WINDOW_SIZE;

      avg_int = state->fifo_info.avg_interrupt_intvl;

      state->fifo_info.avg_interrupt_intvl += (int32_t)(cur_int_delta - avg_int)/int_cnt ;
      state->fifo_info.avg_sampling_intvl = state->fifo_info.avg_interrupt_intvl/num_samples;
      odr_percent_var = ((int)state->cur_odr_change_info.nominal_sampling_intvl - 
                         (int)state->fifo_info.avg_sampling_intvl) * state->current_conf.odr;
      
      if(state->common_info.gyro_curr_odr)
        state->odr_percent_var_gyro = odr_percent_var;
      else
        state->odr_percent_var_accel = odr_percent_var;
      sampling_intvl = cur_int_delta/num_samples;


      LSM6DSO_INST_DEBUG_TS(LOW, instance, "avg_st: cnt=%d int_delta=%u prev=%u cur int:samp=%u:%u:%d",
          int_cnt, (uint32_t)cur_int_delta, avg_int, (uint32_t)state->fifo_info.avg_interrupt_intvl,
          (uint32_t)state->fifo_info.avg_sampling_intvl, odr_percent_var);

    } else {

      DEBUG_TS_EST(HIGH, instance,
          "avg_st: irq_ts=%u prev_irq=%u delta=%d #s=%u #int=%u",
          (uint32_t)irq_timestamp, (uint32_t)state->fifo_info.interrupt_timestamp,
          cur_int_delta, num_samples, state->fifo_info.interrupt_cnt);

      if(state->fifo_info.interrupt_cnt == 1) {
        sampling_intvl = cur_int_delta / num_samples;
      } else {
        state->fifo_info.avg_interrupt_intvl = cur_int_delta;
        state->fifo_info.avg_sampling_intvl = state->fifo_info.avg_interrupt_intvl/num_samples;
        sampling_intvl = state->fifo_info.avg_sampling_intvl; 
      }

    }
  }
  state->fifo_info.interrupt_timestamp = irq_timestamp;
  return sampling_intvl;

}

#define LSM6DSO_IS_FIFO_INTERUPT_SET(x) ((x & STM_LSM6DSO_FIFO_STATUS2_CNT_BDR_INT) ? true : false)

// is_data_read --> atleast one sample is read from fifo
void lsm6dso_read_fifo_data_cleanup(sns_sensor_instance *const instance, bool is_fifo_read)
{
#if !LSM6DSO_DAE_ENABLED
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  if(state->ascp_req_count <= 0) {

    if(state->flushing_sensors != 0) {
      lsm6dso_send_fifo_flush_done(instance, state->flushing_sensors, FLUSH_DONE_AFTER_DATA);
      state->flushing_sensors = 0;
      if(is_fifo_read) {
        lsm6dso_restart_hb_timer(instance, true);
      }
    }
  }

  //reset the top half params
  state->fifo_info.th_info.interrupt_fired = false;
  state->fifo_info.th_info.recheck_int = false;
  state->fifo_info.th_info.flush_req = false;

   //if async read not in progress
  if(state->ascp_req_count <= 0) {
    //if cur wmk = 1, we try to reconfig at interrupt context, so ignoe if wmk=1
    if(state->fifo_info.reconfig_req) {
      DBG_INST_PRINTF(
          LOW, instance, "read_fifo_data: reconfig_req wmk(old/new)=%d/%d",
          state->current_conf.wmk, state->desired_conf.wmk);
      //if only wmk is changing, do not follow normal sequence
      if((state->current_conf.odr == state->desired_conf.odr) &&
          (state->current_conf.enabled_sensors == state->desired_conf.enabled_sensors) &&
          (state->current_conf.wmk != state->desired_conf.wmk) &&
          !state->fifo_info.full_reconf_req) {
        lsm6dso_set_fifo_wmk(instance);
        //reset avg interrupt interval
        state->fifo_info.avg_interrupt_intvl =
          state->fifo_info.avg_sampling_intvl * state->current_conf.wmk;
        //enable interrupt
        lsm6dso_enable_fifo_intr(instance, state->fifo_info.fifo_enabled);
        state->current_conf.publish_sensors = state->desired_conf.publish_sensors;
        state->fifo_info.reconfig_req = false;
        lsm6dso_update_ag_config_event(instance, false);
      } else if((state->current_conf.wmk != 1) || (state->fifo_info.bh_info.interrupt_fired)) {
        lsm6dso_reconfig_fifo(instance, false);
      }
    }
  }
#else
  UNUSED_VAR(instance);
  UNUSED_VAR(is_fifo_read);
#endif
}

void lsm6dso_fill_bh_info(sns_sensor_instance *const instance)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_memscpy(&state->fifo_info.bh_info,
      sizeof(lsm6dso_fifo_req),
      &state->fifo_info.th_info,
      sizeof(lsm6dso_fifo_req));
  //reset th parameters
  uint8_t buffer;
  lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_FIFO_CTRL3, 1, &buffer);
  if(LSM6DSO_IS_ESP_ENABLED(state))
    state->fifo_info.bh_info.accel_odr = (lsm6dso_accel_odr)((buffer & 0x0F)<<4);

  state->fifo_info.th_info.interrupt_fired = false;
  state->fifo_info.th_info.flush_req = false;
  state->fifo_info.th_info.recheck_int = false;
  LSM6DSO_INST_DEBUG_TS(LOW, instance,
      "bh_info int_fired=%d flush_req=%d s_intvl=%u",
      state->fifo_info.bh_info.interrupt_fired, state->fifo_info.bh_info.flush_req,
      (uint32_t)(state->fifo_info.avg_sampling_intvl));
  LSM6DSO_INST_DEBUG_TS(LOW, instance,
      "irq_ts=%u cur_time=%u ",
      (uint32_t)state->fifo_info.bh_info.interrupt_ts,
      (uint32_t)state->fifo_info.bh_info.cur_time);
}

void lsm6dso_send_fw_data_read_msg(
    sns_sensor_instance *const instance,
    uint16_t num_of_bytes,
    bool gyro_enabled)
{
#if !LSM6DSO_DAE_ENABLED
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t buffer[100];
  uint32_t enc_len;
  uint8_t sample_size = STM_LSM6DSO_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);
  sns_port_vector async_read_msg;
  // Compose the async com port message
  async_read_msg.bytes = num_of_bytes;
  async_read_msg.reg_addr = STM_LSM6DSO_REG_FIFO_OUT_TAG;
  async_read_msg.is_write = false;
  async_read_msg.buffer = NULL;

  //if samples > ASYNC_MIN_SAMPLES use async com
  //else use sync com

  // QC: One set is 2 samples when Gyro is active, so we're OK using Sync com port for 60 bytes?
  if((num_of_bytes/sample_size) <= ASYNC_MIN_SAMPLES) {
    uint8_t fifo_data[async_read_msg.bytes];
    // QC: is memset necessary?
    sns_memset(fifo_data, 0, sizeof(fifo_data));
    rc = lsm6dso_read_regs_scp(instance, async_read_msg.reg_addr, async_read_msg.bytes, fifo_data);
    if(rc == SNS_RC_SUCCESS) {
      sns_port_vector vector;
      vector.reg_addr = async_read_msg.reg_addr;
      vector.bytes = async_read_msg.bytes;
      vector.buffer = fifo_data;
      lsm6dso_process_com_port_vector(&vector, instance);
    }
  }
  else if(sns_ascp_create_encoded_vectors_buffer(&async_read_msg, 1, true, buffer,
                                                 sizeof(buffer), &enc_len)) {
    // Send message to Async Com Port
    sns_request async_com_port_request =
      (sns_request)
      {
        .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW,
        .request_len = enc_len,
        .request = buffer
      };
    rc = state->async_com_port_data_stream->api->send_request(
        state->async_com_port_data_stream, &async_com_port_request);
    if(rc != SNS_RC_SUCCESS) {
      SNS_INST_PRINTF(ERROR, instance, "async com port send request failed status %d",rc);
    }
    LSM6DSO_INST_DEBUG_TS(HIGH, instance,
        "send ascp req ascp_req_count %d", state->ascp_req_count);
    state->ascp_req_count++;
  } else {
    SNS_INST_PRINTF(ERROR, instance, "sns_ascp_create_encoded_vectors_buffer failed");
  }
#else
    UNUSED_VAR(instance);
    UNUSED_VAR(num_of_bytes);
    UNUSED_VAR(gyro_enabled);
#endif
}

/** read fifo data after checking fifo int and use sync com port or async com port */
void lsm6dso_read_fifo_data(sns_sensor_instance *const instance, sns_time irq_timestamp, bool flush)
{
#if !LSM6DSO_DAE_ENABLED
  uint8_t fifo_status[4] = {0, 0, 0, 0};
  sns_rc rc = SNS_RC_SUCCESS;
  uint16_t num_of_bytes = 0 , num_sets;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(irq_timestamp);
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  uint8_t sample_size = STM_LSM6DSO_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);

  if(flush)
    state->fifo_info.th_info.flush_req = true;

  // Read the FIFO Status register
  rc = lsm6dso_get_fifo_status(instance, &num_of_bytes, fifo_status);

  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance, "lsm6dso_read_fifo_status FAILED");
    return;
  }
  /*when gyro is running the tag will come in following order
  CFG,TIMESTAMP,GYRO,ACCEL
  We would be reading even samples at one time so we donot get extra gyro sample
  */
  if(flush)
  {
    num_sets = num_of_bytes/sample_size;
    num_of_bytes = num_sets * sample_size;
  }

  if(num_of_bytes < sample_size) {
    LSM6DSO_INST_DEBUG_TS(LOW, instance,
        "#bytes %u < one pattern %u", num_of_bytes, sample_size);
    lsm6dso_read_fifo_data_cleanup(instance, false);
  } else {

    //Debug print
    if(state->fifo_info.reconfig_req) {
      DBG_INST_PRINTF(MED, instance,
          "recheck %d int_fired %d flush_req %d reconfig_req %d",
          state->fifo_info.th_info.recheck_int,state->fifo_info.th_info.interrupt_fired,
          state->fifo_info.th_info.flush_req, state->fifo_info.reconfig_req);
    }

    //**special case**:
    //if flush request received just before interrupt
    //and data reading from async com port
    //case 1: data not read yet, async req pending at fw
    //case 2: data read, and notification to driver pending at fw
    if(state->fifo_info.th_info.interrupt_fired && state->ascp_req_count) {

      //making sure this interrupt belong to the request in-progress
      if(state->fifo_info.bh_info.interrupt_ts > state->fifo_info.th_info.interrupt_ts) {
        LSM6DSO_INST_DEBUG_TS(HIGH, instance,
            "current int is skipping as the same is in-progress: bh_ts=%u cur_int_ts=%u",
            (uint32_t)state->fifo_info.bh_info.interrupt_ts,
            (uint32_t)state->fifo_info.th_info.interrupt_ts);
        state->fifo_info.th_info.interrupt_fired = false;

        //update bottom half info
        state->fifo_info.bh_info.interrupt_fired = true;
        state->fifo_info.bh_info.interrupt_ts = state->fifo_info.th_info.interrupt_ts;
      }
    }

    //At Top half
    //interrupt_fired = true --> use_time =  irq_ts
    //else
    //recheck_int or flush = true --> use_time = cur_time  last_ts + sample_sets * sample_interval

    //At Bottom half
    //update use_time based on new information
    //interrupt_fired = true --> use_time =  irq_ts
    //else
    //recheck_int or flush = true --> use_time = last_ts + sample_sets * sample_interval


    if(!LSM6DSO_IS_FIFO_INTERUPT_SET(fifo_status[1])) {
      state->fifo_info.th_info.recheck_int = false;
      state->fifo_info.th_info.interrupt_fired = false;
      if(state->fifo_info.th_info.flush_req && (state->current_conf.wmk != 1)) {
        SNS_INST_PRINTF(HIGH, instance, "lsm6dso_reset_bdr_cnt!!!!cur_time = %u",
                        sns_get_system_time());
        lsm6dso_reset_bdr_cnt(instance);
      }
    }


    if((!state->fifo_info.th_info.recheck_int) && (!state->fifo_info.th_info.interrupt_fired) &&
        (!state->fifo_info.th_info.flush_req)) {
      //return nothing to be done here
      //useful for active_high/active_low interrupt handling
      return;
    }

    if(state->ascp_req_count) {
      DBG_INST_PRINTF_EX(HIGH, instance,
          "ascp req is pending .. returning without reading data req count %d",
          state->ascp_req_count);
      return;
    }
    state->fifo_info.th_info.accel_odr = state->common_info.accel_curr_odr;
    state->fifo_info.th_info.wmk = state->fifo_info.cur_wmk;
    state->fifo_info.th_info.cur_time = sns_get_system_time();
    if(!state->fifo_info.th_info.interrupt_fired)
      state->fifo_info.th_info.interrupt_ts = state->fifo_info.th_info.cur_time;
    lsm6dso_fill_bh_info(instance);

  LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, "bh_info int_fired = %d flush_req = %d",
                  state->fifo_info.bh_info.interrupt_fired, state->fifo_info.bh_info.flush_req);

    lsm6dso_send_fw_data_read_msg(instance, num_of_bytes, gyro_enabled);
    lsm6dso_read_fifo_data_cleanup(instance, true);
  }
#else
  UNUSED_VAR(instance);
  UNUSED_VAR(irq_timestamp);
  UNUSED_VAR(flush);
#endif
}

/**
 * Extract a accel sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] instance           The current lsm6dso sensor instance
 * @param[i] sensors[]          Array of sensors for which data is requested
 * @param[i] num_sensors        Number of sensor for which data is requested
 * @param[i] raw_data           Uncalibrated sensor data to be logged
 */
void lsm6dso_get_data(sns_sensor_instance *const instance,
                                lsm6dso_sensor_type sensors[],
                                uint8_t num_sensors,
                                int16_t *raw_data)
{

// Timestap is not needed for this implementation as we are not sending anything ot framework
  uint8_t read_addr;

  if((num_sensors == 2)||(sensors[0] == LSM6DSO_GYRO))
    read_addr = STM_LSM6DSO_REG_OUT_X_L_G;
  else
    read_addr = STM_LSM6DSO_REG_OUT_X_L_XL;

  lsm6dso_read_regs_scp(instance, read_addr, 6*num_sensors, (uint8_t*)raw_data);
  SNS_INST_PRINTF(LOW, instance, "DATA sensor=%u ts=%u [%d, %d, %d]",
                  sensors[0], (uint32_t)sns_get_system_time(),
                  raw_data[0], raw_data[1], raw_data[2]);


  if(num_sensors == 2) { //both accel and gyro data requested
    SNS_INST_PRINTF(LOW, instance, "DATA sensor=%u ts=%u [%d, %d, %d]",
                    sensors[1], (uint32_t)sns_get_system_time(),
                    raw_data[3], raw_data[4], raw_data[5]);
  }

}

/**
 * see sns_lsm6dso_hal.h
 */
void lsm6dso_dump_reg(sns_sensor_instance *const instance,
                      lsm6dso_sensor_type sensor)
{  UNUSED_VAR(sensor);
#if LSM6DSO_DUMP_REG
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  struct group_read {
    uint32_t first_reg;
    uint8_t  num_regs;
  } groups[] = { /* must fit within state->reg_status[] */
    { STM_LSM6DSO_REG_FIFO_CTRL1, 4 },
    { STM_LSM6DSO_REG_CNT_BDR1, 17 },
    { STM_LSM6DSO_REG_TAP_CFG0,    9 }
  };
  uint8_t *reg_val = state->reg_status;

  for(uint32_t i=0; i<ARR_SIZE(groups); i++)
  {
    lsm6dso_read_regs_scp(instance, groups[i].first_reg, groups[i].num_regs, reg_val);
    for(uint32_t j=0; j<groups[i].num_regs; j++)
    {
      DBG_INST_PRINTF(LOW, instance, "dump: 0x%02x=0x%02x",
                      groups[i].first_reg+j, reg_val[j]);
    }
    reg_val += groups[i].num_regs;
  }
#else
  UNUSED_VAR(instance);
#endif
}

void lsm6dso_set_gated_accel_config(lsm6dso_instance_state *state,
                      uint16_t desired_wmk,
                      lsm6dso_accel_odr a_chosen_sample_rate,
                      lsm6dso_sensor_type sensor)
{
  if(!state->accel_info.gated_client_present)
  {
    desired_wmk = 0;
    a_chosen_sample_rate = LSM6DSO_ACCEL_ODR_OFF;
  }

  if(sensor & LSM6DSO_ACCEL)
  {
    state->md_info.desired_wmk = desired_wmk;
    state->md_info.desired_odr = a_chosen_sample_rate;
    state->md_info.sstvt = state->accel_info.sstvt;
    state->md_info.range = state->accel_info.range;
    state->md_info.bw = LSM6DSO_ACCEL_BW50;
  }
}

void lsm6dso_set_md_filter(sns_sensor_instance *const instance,
                            lsm6dso_md_filter_type filter)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)instance->state->state;
  uint32_t xfer_bytes;
  lsm6dso_md_filter_type  md_filter = (filter ==  SLOPE_FILTER) ? 0x0 : 0x10;
  if (inst_state->md_info.filter != filter) {
    lsm6dso_read_modify_write(instance,
        STM_LSM6DSO_REG_TAP_CFG0,
        &md_filter,
        1,
        &xfer_bytes,
        false,
        0x10);

    inst_state->md_info.filter = filter;
  }
}

void lsm6dso_set_md_intr(sns_sensor_instance *const instance, bool enable)
{
  //update the reg only if necessary
  //save some cycles
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  bool update_reg = false;
  uint32_t reg_addr = 0;
  DBG_INST_PRINTF(LOW, instance, "set_md_intr: %u %u %u %u",
                  enable, state->irq_ready, state->md_info.is_filter_settled,
                  state->md_info.cur_state.motion_detect_event_type);
  if(!state->irq_ready) {
    DBG_INST_PRINTF(LOW, instance, "set_md_intr: irq not ready");
    return;
  }
  if((enable) &&
      (state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED))
  {
    rw_buffer = 0
      | (0<<7)            // INT1_SLEEP_CHANGE
      | (0<<6)            // INT1_SINGLE_TAP
      | (1<<5)            // INT1_WU
      | (0<<4)            // INT1_FF
      | (0<<3)            // INT1_TAP
      | (0<<2)            // INT1_6D
      | (0<<1)            // 0
      | 0;                // 0
    update_reg = true;
  } else if((!enable) &&
      (state->md_info.is_filter_settled)) {
    // Workaround for Qualcomm MTP issue where interrupt pin is not configured(pull type) properly.
    // If Int pin properly configured, the update_reg value below should be true.
    update_reg = true;
  }

  if(update_reg) {
    state->current_conf.md_enabled = enable;
    if(state->route_md_to_irq2)
      reg_addr = STM_LSM6DSO_REG_MD2_CFG;
    else
      reg_addr = STM_LSM6DSO_REG_MD1_CFG;
    DBG_INST_PRINTF_EX(LOW, instance, "set_md_intr: reg_addr = 0x%0x, MD_CFG=0x%x", reg_addr, rw_buffer);
    lsm6dso_read_modify_write(instance,
        reg_addr,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x20);
  }
}

static bool is_lsm6dso_mdf_settled(sns_sensor_instance *this, lsm6dso_md_filter_type filter)
{
  lsm6dso_instance_state *inst_state =
    (lsm6dso_instance_state*)this->state->state;
  sns_time cur_time, odr_time_elapsed, filter_settling_time;
  uint64_t settling_time_ns = 0;
  int64_t req_timeout = 0;
  uint8_t odr_idx = (inst_state->desired_conf.odr_idx) ? inst_state->desired_conf.odr_idx : inst_state->min_odr_idx;
  if(inst_state->common_info.accel_curr_odr == LSM6DSO_ACCEL_ODR_OFF) {
    DBG_INST_PRINTF_EX(LOW, this, "acc on");
  }

  if(filter == SLOPE_FILTER) {
    settling_time_ns = SLOPE_SETTLING_TIME_NS((uint64_t)lsm6dso_odr_map[odr_idx].odr);
  } else if(filter == HP_FILTER) {
    settling_time_ns = HPF_SETTLING_TIME_NS((uint64_t)lsm6dso_odr_map[odr_idx].odr);
  }

  //get sample interval -- not possible at sensor level, use minimum 13Hz
  filter_settling_time = sns_convert_ns_to_ticks(settling_time_ns); // in ticks
  cur_time = sns_get_system_time();
  odr_time_elapsed = cur_time - inst_state->cur_odr_change_info.accel_odr_settime;
  req_timeout = (int64_t) (filter_settling_time - odr_time_elapsed);

  DBG_INST_PRINTF(HIGH, this,
                  "cur_t %u set_t %u/%u (ns/ticks)",
                  (uint32_t)cur_time, (uint32_t)settling_time_ns, (uint32_t)filter_settling_time);
  DBG_INST_PRINTF(HIGH, this,
                  "a_odr_set_t %u elapsed %u req_t/o %u",
                  (uint32_t)inst_state->cur_odr_change_info.accel_odr_settime, (uint32_t)odr_time_elapsed,
                  (uint32_t)req_timeout);

  //dump registers
  //lsm6dso_dump_reg(this, inst_state->fifo_info.fifo_enabled);
  if(req_timeout > 0) {
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = filter_settling_time;
    lsm6dso_inst_create_timer(this, &inst_state->timer_md_data_stream, &req_payload);
    inst_state->md_info.is_timer_running = true;
    return false;
  } else {
    inst_state->md_info.is_timer_running = false;
    return true;
  }
}
// to set the correct md filter and enable md interrupt
void lsm6dso_update_md_filter(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)this->state->state;

  lsm6dso_md_filter_type filter;
  DBG_INST_PRINTF_EX(HIGH, this, "mdf request: %u %u",
                  lsm6dso_is_md_int_required(this),
                  inst_state->md_info.cur_state.motion_detect_event_type);
  if(inst_state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED) {
    filter = SLOPE_FILTER;
    //default is slope filter
    inst_state->md_info.is_filter_settled = false;
    if(is_lsm6dso_mdf_settled(this, SLOPE_FILTER)) {
      inst_state->md_info.is_filter_settled = true;
      if(is_lsm6dso_mdf_settled(this, HP_FILTER)) {
        filter = HP_FILTER;
      }
    }
    if(inst_state->md_info.is_filter_settled) {
      lsm6dso_set_md_intr(this, true);
    }
    lsm6dso_set_md_filter(this, filter);

  } else {
    // ignore the request

  }
}

void lsm6dso_set_md_config(sns_sensor_instance *const instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint8_t r_buffer[2], w_buffer[2];
  uint8_t dur_set = 0x60; // wake up duration bits 5,6 set to 1
  uint8_t thresh_set = 0x3F; // thresh bits 0 to 5 set to 1
  lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
  uint8_t md_odr_idx =(state->desired_conf.odr_idx) ? state->desired_conf.odr_idx : state->min_odr_idx;
  uint8_t md_odr = (md_odr_idx << 4);
  float md_odr_val = lsm6dso_get_accel_odr(md_odr);
  float md_thresh;
  float thresh_lsb;

  if(enable && (accel_odr == LSM6DSO_ACCEL_ODR_OFF)) {
    accel_odr = md_odr;
  }

  DBG_INST_PRINTF_EX(HIGH, instance,
                  "md_config: en=%d des_odr(md/a) 0x%x/0x%x",
                  enable, state->md_info.desired_odr, state->accel_info.desired_odr);
  DBG_INST_PRINTF_EX(HIGH, instance, "md_thd(*1000) =%d  win(*1000)=%d",
                  (int)(state->md_info.md_config.thresh *1000),
                  (int)(state->md_info.md_config.win*1000));

  md_thresh = state->md_info.md_config.thresh;
  //one lsb value for MD threshold
  //all values are in m/s2
  thresh_lsb = LSM6DSO_MD_THRESH_MAX/LSM6DSO_MD_COARSE_RES; //2^6
  if(enable && md_thresh <= LSM6DSO_MD_THRESH_MAX && md_thresh >= 0.0) {
    //with FINE_RES, max can reach FS/4
    if(md_thresh < LSM6DSO_MD_THRESH_MAX/4) {
      thresh_lsb = LSM6DSO_MD_THRESH_MAX/LSM6DSO_MD_FINE_RES; //2^8
      dur_set |= 0x10; //WAKE_THS_W bit 4 in WAKE_UP_DUR register
    }
    thresh_set &= ((uint8_t)((lroundf)(md_thresh / thresh_lsb)) | 0xC0);
  }

  if(state->md_info.md_config.win <= 0x3 / md_odr_val
     && state->md_info.md_config.win >= 0.0)
  {
    // Wake Up Duration - bits 5,6 of STM_LSM6DSO_REG_WAKE_DUR
    dur_set &= (((uint8_t)((lroundf)(state->md_info.md_config.win *
                                     md_odr_val)) << 5) | 0x9F);
  }

  DBG_INST_PRINTF(LOW, instance, "th_set=0x%x  dur_set=0x%x",
                  thresh_set, dur_set);

  if(SNS_RC_SUCCESS == lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_WAKE_THS, 2, r_buffer)) {
    w_buffer[0] = (r_buffer[0] & (~0x3F)) | (!enable ? 0x3F : thresh_set);
    w_buffer[1] = (r_buffer[1] & (~0x60)) | dur_set;

    if(*(uint16_t*)r_buffer != *(uint16_t*)w_buffer)
    {
      DBG_INST_PRINTF(LOW, instance, "THS/DUR %02x%02x --> %02x%02x",
                      r_buffer[0], r_buffer[1], w_buffer[0], w_buffer[1]);
      lsm6dso_write_regs_scp(instance, STM_LSM6DSO_REG_WAKE_THS, 2, w_buffer);
    }
  }

  if(enable && (state->common_info.accel_curr_odr == LSM6DSO_ACCEL_ODR_OFF)) {
    lsm6dso_set_accel_config(instance,
        accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);
  }

  DBG_INST_PRINTF(LOW, instance, "en=%d, cur_odr(a/g)=%x/%x",
                  enable, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);
}

void lsm6dso_send_md_event(sns_sensor_instance *const instance, sns_motion_detect_event* event, sns_time event_timestamp)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "[MD Event] = %d",
      event->motion_detect_event_type);
  if(state->md_info.add_request)
  {
    state->md_info.add_request = false;
  }
  pb_send_event(instance,
      sns_motion_detect_event_fields,
      event,
      event_timestamp,
      SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
      &state->md_info.suid);
}

void lsm6dso_update_md_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  if(enable)
  {
    state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_ENABLED;
    lsm6dso_update_md_filter(instance);
  }
  else
  {
    state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
    lsm6dso_set_md_intr(instance, enable);
    lsm6dso_set_md_filter(instance, SLOPE_FILTER);
  }
}

/**
 * Changes all gated accel requests to non-gated accel requests.
 *
 * @param instance   Reference to the instance
 *
 * @return None
 */
void lsm6dso_convert_accel_gated_req_to_non_gated(
   sns_sensor_instance *const instance)
{
  sns_request *request;
  bool req_converted_to_non_gated = false;
  sns_sensor_uid* suid = &((sns_sensor_uid)ACCEL_SUID_0);
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
#if LSM6DSO_DUAL_SENSOR_ENABLED
  if(state->hw_idx)
    suid = &((sns_sensor_uid)ACCEL_SUID_1);
#endif

  /** Parse through existing requests and change gated accel
   *  requests to non-gated accel requests. */
  for(request = (sns_request *)instance->cb->get_client_request(instance, suid, true);
      NULL != request;
      request = (sns_request *)instance->cb->get_client_request(instance, suid, false))
  {
    if(request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      request->message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
      state->desired_conf.publish_sensors |= LSM6DSO_ACCEL;
      //updating inst_publish too,as this is used to send data
      state->current_conf.publish_sensors |= LSM6DSO_ACCEL;
      req_converted_to_non_gated = true;
    }
  }

  /** Send an event to gated stream clients that the request is
   *  now treated as non-gated */
  if(req_converted_to_non_gated)
  {
    sns_service_manager *mgr = instance->cb->get_service_manager(instance);
    sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
    sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
    lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;

    if(NULL != event)
    {
      event->message_id = SNS_STD_EVENT_GATED_SENSOR_MSGID_GATED_REQ_CONVERTED_TO_NON_GATED;
      event->event_len = 0;
      event->timestamp = sns_get_system_time();
      e_service->api->publish_event(e_service, instance, event, &state->accel_info.suid);
    }
  }
}

#if LSM6DSO_REMOVE_ON_CHANGE_REQUEST 
static void lsm6dso_remove_on_change_requests(sns_sensor_instance *const instance)
{
  sns_request const *md_req;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_sensor_uid* suid = &state->md_info.suid;
  int num_md_req = 0;
  bool from_head = true;

  while(NULL != (md_req = instance->cb->get_client_request(instance, suid, from_head)))
  {
    if(md_req->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
    {
      num_md_req++;
      instance->cb->remove_client_request(instance, md_req);
      from_head = true;
    }
    else
    {
      from_head = false;
    }
  }
  if(num_md_req > 0)
  {
    DBG_INST_PRINTF(MED, instance, "[%u] MD requests removed = %d", state->hw_idx, num_md_req);
  }
}
#endif

void lsm6dso_disable_md(sns_sensor_instance *const instance, bool send_md_status_event)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "Disable_MD cur state = %d",
      state->md_info.cur_state.motion_detect_event_type);
  lsm6dso_update_md_intr(instance, false);
  lsm6dso_set_md_config(instance, false);
  state->md_info.is_timer_running = false;
  if(state->desired_conf.publish_sensors & LSM6DSO_ACCEL)
  {
    lsm6dso_set_acc_lpmode(instance, false);
  }
  if(send_md_status_event)
  {
    /*TODO: is config event needed for disable*/
    if(state->md_info.client_present || (state->config_sensors & LSM6DSO_MOTION_DETECT))
    {
      state->md_info.event_ts = sns_get_system_time();
      lsm6dso_send_md_event(instance, &state->md_info.cur_state, state->md_info.event_ts);
    }
  }
}
void lsm6dso_enable_md(sns_sensor_instance *const instance, bool send_md_status_event)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "Enable_MD cur state = %d",
      state->md_info.cur_state.motion_detect_event_type);
  if(!(state->desired_conf.publish_sensors & LSM6DSO_ACCEL))
  {
    lsm6dso_set_acc_lpmode(instance, true);
  }
  lsm6dso_set_md_config(instance, true);
  lsm6dso_update_md_intr(instance, true);
  if(send_md_status_event)
  {
    lsm6dso_send_config_event(instance, false);
    if(state->md_info.client_present || (state->config_sensors & LSM6DSO_MOTION_DETECT))
    {
      state->md_info.event_ts = sns_get_system_time();
      lsm6dso_send_md_event(instance, &state->md_info.cur_state, state->md_info.event_ts);
    }
  }
}

void lsm6dso_turn_off_md(sns_sensor_instance *const instance,
                         lsm6dso_instance_state *state)
{
  DBG_INST_PRINTF(HIGH, instance, "turn_off_md: gated_client=%u",
                  state->accel_info.gated_client_present);
  bool is_md_req = lsm6dso_is_md_int_required(instance);

  if(!is_md_req) {
    lsm6dso_disable_md(instance, false);
  }
  //set reconfiguration only if no non-gated clients
  if((state->accel_info.gated_client_present) &&
    (state->fifo_info.fifo_rate == LSM6DSO_ACCEL_ODR_OFF)) {

    lsm6dso_enable_hw_timestamp(instance, false);

    //turn off accel before configuring for streaming
    lsm6dso_set_accel_config(instance,
        LSM6DSO_ACCEL_ODR_OFF,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSO_ACCEL_BW50);

    lsm6dso_set_fifo_config(
      instance,
      state->md_info.desired_wmk,
      state->md_info.desired_odr,
      state->common_info.gyro_curr_odr,
      state->fifo_info.fifo_enabled);
    lsm6dso_set_fifo_wmk(instance);
    lsm6dso_start_fifo_streaming(instance);
    lsm6dso_enable_fifo_intr(instance, state->fifo_info.fifo_enabled);
    lsm6dso_send_config_event(instance, false);
    state->fifo_info.interrupt_cnt = 0;
  }
  lsm6dso_dump_reg(instance, state->fifo_info.fifo_enabled);
}

uint8_t lsm6dso_read_wake_src(lsm6dso_instance_state *state)
{
  uint8_t buffer = 0;
  uint32_t xfer_bytes;
  lsm6dso_instance_com_read_wrapper(state,
                           STM_LSM6DSO_REG_WAKE_SRC,
                           &buffer,
                           1,
                           &xfer_bytes);
  return buffer;
}

bool lsm6dso_check_md_interrupt(lsm6dso_instance_state *state,
                                uint8_t const *wake_src)
{
  uint8_t rw_buffer = 0;
  if(wake_src == NULL)
  {
    rw_buffer = lsm6dso_read_wake_src(state);
  }
  else
  {
    rw_buffer = *wake_src;
  }

  return (rw_buffer & 0x08) ? true : false;
}

void lsm6dso_handle_md_fired(sns_sensor_instance *const instance,
                             sns_time irq_timestamp)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  /**
   * 1. Handle MD interrupt: Send MD fired event to client.
   * 2. Disable MD.
   * 3. Start Motion Accel FIFO stream with desired config.
   */
  sns_motion_detect_event md_state;
  md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_FIRED;
  DBG_INST_PRINTF(HIGH, instance, "[%u] MD_fired", state->hw_idx);
  lsm6dso_send_md_event(instance, &md_state, irq_timestamp);
  lsm6dso_log_interrupt_event(instance, &state->md_info.suid, irq_timestamp);

  //since MD fired- clear client_present variable ()
  state->md_info.client_present = false;
  lsm6dso_convert_accel_gated_req_to_non_gated(instance);

#if LSM6DSO_REMOVE_ON_CHANGE_REQUEST 
  // remove all on-change requests
  lsm6dso_remove_on_change_requests(instance);
#endif

  lsm6dso_turn_off_md(instance, state);
  state->num_md_ints++;
}

void lsm6dso_handle_md_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;

  if(state->md_info.client_present && lsm6dso_check_md_interrupt(state, wake_src))
  {
    lsm6dso_handle_md_fired(instance, irq_timestamp);
  }
}


/** See sns_lsm6dso_hal.h */
void lsm6dso_send_config_event(sns_sensor_instance *const instance, bool new_client)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  char operating_mode_normal[] = {LSM6DSO_NORMAL};
  char operating_mode_lpm[] = {LSM6DSO_LPM};
  char operating_mode_hpf[] = {LSM6DSO_HIGH_PERF};
  char operating_mode_off[] = {LSM6DSO_OFF};
  uint32_t active_current = 0;
  sns_time event_ts = sns_get_system_time();
  bool is_passive_needed_a_g = (!state->current_conf.odr
                                && (state->passive_client_present & (LSM6DSO_ACCEL | LSM6DSO_GYRO)));
  bool is_passive_needed_t = (!state->sensor_temp_info.cur_sampling_rate_hz
                                && (state->passive_client_present & LSM6DSO_SENSOR_TEMP));
  sns_std_sensor_physical_config_event phy_sensor_config =
     sns_std_sensor_physical_config_event_init_default;

  pb_buffer_arg op_mode_args;

  if(state->desired_conf.publish_sensors == 0 && !state->md_info.client_present && 
     !state->passive_client_present)
  {
    return; // no clients present
  }

  if(state->desired_conf.publish_sensors != 0 || state->md_info.client_present)
  {
    if(!new_client)
    {
      state->fifo_info.last_sent_config = state->fifo_info.new_config;
    }
    /*if already passive client is present and new active client comes
      then update the timestamp with current time*/
    if((state->passive_client_present && !state->current_conf.odr && state->desired_conf.publish_sensors != 0)
      || (!state->fifo_info.last_sent_config.timestamp))
      state->fifo_info.last_sent_config.timestamp = event_ts;

    phy_sensor_config.sample_rate   = state->fifo_info.last_sent_config.sample_rate;
    phy_sensor_config.water_mark    = state->fifo_info.last_sent_config.fifo_watermark;
    event_ts                        = state->fifo_info.last_sent_config.timestamp;
#if LSM6DSO_DAE_ENABLED
    phy_sensor_config.DAE_watermark = state->fifo_info.last_sent_config.dae_watermark;
#endif
  }
  if(!phy_sensor_config.sample_rate)
  {
    //OFF mode
    op_mode_args.buf = &operating_mode_off[0];
    op_mode_args.buf_len = sizeof(operating_mode_off);
    active_current = LSM6DSO_ACCEL_SLEEP_CURRENT;
  }
  else if(!state->accel_info.lp_mode)
  {
    //High Performance mode
    op_mode_args.buf = &operating_mode_hpf[0];
    op_mode_args.buf_len = sizeof(operating_mode_hpf);
    active_current = LSM6DSO_ACCEL_ACTIVE_CURRENT_HIGH;
  }
  else if(state->common_info.accel_curr_odr > LSM6DSO_ACCEL_ODR52)
  {
    //Normal mode
    op_mode_args.buf = &operating_mode_normal[0];
    op_mode_args.buf_len = sizeof(operating_mode_normal);
  }
  else
  {
    //Low power mode
    op_mode_args.buf = &operating_mode_lpm[0];
    op_mode_args.buf_len = sizeof(operating_mode_lpm);
  }
  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.has_water_mark = true;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_active_current = true;
  phy_sensor_config.active_current = active_current;
  phy_sensor_config.has_resolution = true;
  phy_sensor_config.resolution = (float)(state->accel_info.sstvt)/1000 * ACC_RES_CONVERSION;
  phy_sensor_config.range_count = 2;
  // Convert to m/s2
  phy_sensor_config.range[0] = lsm6dso_accel_range_min[state->accel_info.range_idx] * ACC_CONVERSION;
  phy_sensor_config.range[1] = lsm6dso_accel_range_max[state->accel_info.range_idx] * ACC_CONVERSION;
  phy_sensor_config.has_stream_is_synchronous = (state->s4s_info.sync_state == LSM6DSO_S4S_SYNCED);
  phy_sensor_config.stream_is_synchronous = (state->s4s_info.sync_state == LSM6DSO_S4S_SYNCED);
  phy_sensor_config.has_DAE_watermark = lsm6dso_dae_if_available(instance);
  phy_sensor_config.has_dri_enabled = (state->ag_stream_mode == DRI);
  phy_sensor_config.dri_enabled = (state->ag_stream_mode == DRI);
  DBG_INST_PRINTF(
    MED, instance, "[%u] config_event:: new=%u sens=%x/%x SR=%u WM=%u/%u tempSR=%u ts=%u",
    state->hw_idx, new_client, (state->desired_conf.publish_sensors | state->md_info.client_present), 
    state->passive_client_present, (uint32_t)phy_sensor_config.sample_rate, 
    phy_sensor_config.water_mark, phy_sensor_config.DAE_watermark, 
    (uint32_t)state->sensor_temp_info.desired_sampling_rate_hz, (uint32_t)event_ts);

  LSM6DSO_INST_AUTO_DEBUG_PRINTF(
     HIGH, instance, "send_config_event: pub=0x%x SR*1000=%u WM=%u",
     state->desired_conf.publish_sensors, (uint32_t)(phy_sensor_config.sample_rate*1000),
     phy_sensor_config.water_mark);

  if((((state->desired_conf.publish_sensors & LSM6DSO_ACCEL) && (phy_sensor_config.sample_rate)) ||
      state->accel_info.gated_client_present))
  {
    float temp_sample_rate = phy_sensor_config.sample_rate;
    sns_time temp_ts       = event_ts;
    pb_buffer_arg curr_op_mode_args = op_mode_args;
    uint32_t cur_active_current = phy_sensor_config.active_current;
    if(state->accel_info.gated_client_present 
       && !(state->desired_conf.publish_sensors & LSM6DSO_ACCEL))
    {
      phy_sensor_config.sample_rate = 0.0f;
      event_ts = state->fifo_info.new_config.gated_timestamp;
      op_mode_args.buf = &operating_mode_off[0];
      op_mode_args.buf_len = sizeof(operating_mode_off);
      phy_sensor_config.operation_mode.arg = &op_mode_args;
      phy_sensor_config.active_current = LSM6DSO_ACCEL_SLEEP_CURRENT;
    }
    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  event_ts,
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->accel_info.suid);

    //Restore the values back
    phy_sensor_config.sample_rate = temp_sample_rate;
    event_ts = temp_ts;
    op_mode_args = curr_op_mode_args;
    phy_sensor_config.operation_mode.arg = &curr_op_mode_args;
    phy_sensor_config.active_current = cur_active_current;
    lsm6dso_send_cal_event(instance, LSM6DSO_ACCEL);
  }

  if((state->desired_conf.publish_sensors & LSM6DSO_GYRO) &&
      (phy_sensor_config.sample_rate))
  {
    // Override above values with gyro info
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = (!state->common_info.accel_curr_odr) ?
                                       LSM6DSO_GYRO_SLEEP_CURRENT : LSM6DSO_GYRO_ACTIVE_CURRENT_HIGH;
    // Convert range and resolutions to radians/sec
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = state->gyro_info.sstvt * GYRO_CONVERSION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = lsm6dso_gyro_range_min[state->gyro_info.range_idx] * GYRO_CONVERSION;
    phy_sensor_config.range[1] = lsm6dso_gyro_range_max[state->gyro_info.range_idx] * GYRO_CONVERSION;

    pb_send_event(instance,
        sns_std_sensor_physical_config_event_fields,
        &phy_sensor_config,
        event_ts,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
        &state->gyro_info.suid);

    lsm6dso_send_cal_event(instance, LSM6DSO_GYRO);
  }

  if(state->desired_conf.publish_sensors & LSM6DSO_SENSOR_TEMP)
  {

    if(!new_client)
    {
      state->sensor_temp_info.last_sent_config = state->sensor_temp_info.new_config;
    }
//    event_ts                        = state->sensor_temp_info.last_sent_config.timestamp;
    phy_sensor_config.sample_rate   = state->sensor_temp_info.last_sent_config.sample_rate;
#if LSM6DSO_DAE_ENABLED
    phy_sensor_config.DAE_watermark = state->sensor_temp_info.last_sent_config.dae_watermark;
#endif
    // Override above values with sensor temperature info
    bool current_dri_enabled = phy_sensor_config.dri_enabled;

    pb_buffer_arg curr_op_mode_args = op_mode_args;
    op_mode_args.buf = &operating_mode_hpf[0];
    op_mode_args.buf_len = sizeof(operating_mode_hpf);

    phy_sensor_config.dri_enabled = false;
    //phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = 1;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = LSM6DSO_TEMP_ACTIVE_CURRENT_HIGH;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = LSM6DSO_SENSOR_TEMPERATURE_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = LSM6DSO_SENSOR_TEMPERATURE_RANGE_MIN;
    phy_sensor_config.range[1] = LSM6DSO_SENSOR_TEMPERATURE_RANGE_MAX;

    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  state->sensor_temp_info.last_sent_config.timestamp,
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->sensor_temp_info.suid);
    //Restore the value back
    phy_sensor_config.dri_enabled = current_dri_enabled;
    phy_sensor_config.operation_mode.arg = &curr_op_mode_args;
  }

   if (!(state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_SENSOR_TEMP))
       && (is_passive_needed_a_g | is_passive_needed_t))
   {
    lsm6dso_sensor_type passive_clients = 
      state->passive_client_present & (LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_SENSOR_TEMP);
    if(passive_clients & (LSM6DSO_ACCEL | LSM6DSO_GYRO))
    {
      if(state->fifo_info.new_config.timestamp)
        event_ts = state->fifo_info.last_sent_config.timestamp;
      else
        state->fifo_info.new_config.timestamp = state->fifo_info.last_sent_config.timestamp = event_ts;
    }
    phy_sensor_config.sample_rate = 0;
    while(passive_clients != 0)
    {
      sns_sensor_uid *suid;
      if(passive_clients & LSM6DSO_ACCEL)
      {
        phy_sensor_config.active_current = LSM6DSO_ACCEL_SLEEP_CURRENT;
        suid = &state->accel_info.suid;
        passive_clients &= ~LSM6DSO_ACCEL;
      }
      else if(passive_clients & LSM6DSO_GYRO)
      {
        phy_sensor_config.active_current = LSM6DSO_GYRO_SLEEP_CURRENT;
        // Convert to radians/sec
        phy_sensor_config.range[0] = lsm6dso_gyro_range_min[state->gyro_info.range_idx] * GYRO_CONVERSION;
        phy_sensor_config.range[1] = lsm6dso_gyro_range_max[state->gyro_info.range_idx] * GYRO_CONVERSION;
        phy_sensor_config.resolution = state->gyro_info.sstvt * GYRO_CONVERSION;
        suid = &state->gyro_info.suid;
        passive_clients &= ~LSM6DSO_GYRO;
      }
      else
      {
        phy_sensor_config.active_current = LSM6DSO_TEMP_SLEEP_CURRENT;
        phy_sensor_config.range[0] = LSM6DSO_SENSOR_TEMPERATURE_RANGE_MIN;
        phy_sensor_config.range[1] = LSM6DSO_SENSOR_TEMPERATURE_RANGE_MAX;
        phy_sensor_config.resolution = LSM6DSO_SENSOR_TEMPERATURE_RESOLUTION;
        phy_sensor_config.dri_enabled = false;
        suid = &state->sensor_temp_info.suid;
        passive_clients &= ~LSM6DSO_SENSOR_TEMP;
 
        if(state->sensor_temp_info.new_config.timestamp)
          event_ts = state->sensor_temp_info.last_sent_config.timestamp;
        else
          state->sensor_temp_info.new_config.timestamp = state->sensor_temp_info.last_sent_config.timestamp = event_ts = sns_get_system_time(); 
      }
      pb_send_event(instance,
                    sns_std_sensor_physical_config_event_fields,
                    &phy_sensor_config,
                    event_ts,
                    SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                    suid);

      if(suid == &state->accel_info.suid)
        lsm6dso_send_cal_event(instance, LSM6DSO_ACCEL);
      if(suid == &state->gyro_info.suid)
        lsm6dso_send_cal_event(instance, LSM6DSO_GYRO);
    }
  }
  if(state->md_info.client_present)
  {
    op_mode_args.buf = &operating_mode_hpf[0];
    op_mode_args.buf_len = sizeof(operating_mode_hpf);
    // Override above values with Motion Detect info
    phy_sensor_config.has_sample_rate = false;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = LSM6DSO_ACCEL_ACTIVE_CURRENT_LOW;
    phy_sensor_config.has_resolution = false;
    phy_sensor_config.range_count = 0;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = (state->ag_stream_mode == DRI);
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    phy_sensor_config.has_DAE_watermark = false;

    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  state->fifo_info.new_config.md_timestamp,
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->md_info.suid);
  }
}

void lsm6dso_convert_and_send_temp_sample(
  sns_sensor_instance *const instance,
  sns_time            timestamp,
  const uint8_t       temp_data[2])
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  log_sensor_state_raw_info log_temp_state_raw_info;
  sns_memset(&log_temp_state_raw_info, 0, sizeof(log_sensor_state_raw_info));
  if(state->desired_conf.publish_sensors & LSM6DSO_SENSOR_TEMP)
  {
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    int16_t temp_val = (int16_t)(temp_data[1] << 8 | temp_data[0]);
    float float_temp_val = (temp_val / 256.0) + 25.0;
    // factory calibration
    // Sc = C * (Sr - B)
    // Where,
    // *Sc = Calibrated sample
    // *Sr = Raw sample
    // *C = Scale
    // *B = Bias
    float_temp_val = state->sensor_temp_registry_cfg.fac_cal_corr_mat.e00 * (float_temp_val -
      state->sensor_temp_registry_cfg.fac_cal_bias[0]);
    if(!state->self_test_info.test_alive)
    {
      state->most_recent_temperature_reading = float_temp_val;
      pb_send_sensor_stream_event(instance,
                                  &state->sensor_temp_info.suid,
                                  timestamp,
                                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                  status,
                                  &float_temp_val,
                                  1,
                                  state->encoded_sensor_temp_event_len);
    }
    state->num_temp_samples++;

    if((state->num_temp_samples % 100) == 0)
    {
      DBG_INST_PRINTF(LOW, instance, "TEMP  ts=%08X/%u val=%d",
                      (uint32_t)timestamp, (uint32_t)timestamp, (int)float_temp_val);
    }

    LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, instance, "Temp data:  = %d timestamp = %u",
                    (int)float_temp_val, (uint32_t)timestamp);

    lsm6dso_log_sample(instance, &log_temp_state_raw_info, &state->sensor_temp_info.suid, 
        state->log_temp_raw_encoded_size,
        &float_temp_val, 1,
        timestamp, status, SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY);
  }
}

/** See sns_lsm6dso_hal.h */
void lsm6dso_handle_sensor_temp_sample(sns_sensor_instance *const instance)
{
  uint8_t temp_data[2] = {0};
  uint8_t buffer;

  lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_STATUS, 1, &buffer);
  lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_OUT_TEMP_L, 2, temp_data);

  lsm6dso_convert_and_send_temp_sample(instance, sns_get_system_time(), temp_data);
}

/** See sns_lsm6dso_hal.h */
void lsm6dso_send_cal_event(sns_sensor_instance *const instance, lsm6dso_sensor_type sensor_type)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)instance->state->state;
  sns_cal_event new_calibration_event = sns_cal_event_init_default;
  sns_lsm6dso_registry_cfg *src_cfg = (sensor_type == LSM6DSO_ACCEL) ?
    &state->accel_registry_cfg : &state->gyro_registry_cfg;
  sns_sensor_uid *suid_current = (sensor_type == LSM6DSO_ACCEL) ?
    &state->accel_info.suid : &state->gyro_info.suid;
  float bias_data[] = {0,0,0};
  float comp_matrix_data[] = {0,0,0,0,0,0,0,0,0};

  bias_data[0] = src_cfg->fac_cal_bias[0];
  bias_data[1] = src_cfg->fac_cal_bias[1];
  bias_data[2] = src_cfg->fac_cal_bias[2];
  comp_matrix_data[0] = src_cfg->fac_cal_corr_mat.data[0];
  comp_matrix_data[1] = src_cfg->fac_cal_corr_mat.data[1];
  comp_matrix_data[2] = src_cfg->fac_cal_corr_mat.data[2];
  comp_matrix_data[3] = src_cfg->fac_cal_corr_mat.data[3];
  comp_matrix_data[4] = src_cfg->fac_cal_corr_mat.data[4];
  comp_matrix_data[5] = src_cfg->fac_cal_corr_mat.data[5];
  comp_matrix_data[6] = src_cfg->fac_cal_corr_mat.data[6];
  comp_matrix_data[7] = src_cfg->fac_cal_corr_mat.data[7];
  comp_matrix_data[8] = src_cfg->fac_cal_corr_mat.data[8];

  pb_buffer_arg buff_arg_bias = (pb_buffer_arg)
      { .buf = &bias_data, .buf_len = ARR_SIZE(bias_data) };
  pb_buffer_arg buff_arg_comp_matrix = (pb_buffer_arg)
      { .buf = &comp_matrix_data, .buf_len = ARR_SIZE(comp_matrix_data) };

  new_calibration_event.bias.funcs.encode = &pb_encode_float_arr_cb;
  new_calibration_event.bias.arg = &buff_arg_bias;
  new_calibration_event.comp_matrix.funcs.encode = &pb_encode_float_arr_cb;
  new_calibration_event.comp_matrix.arg = &buff_arg_comp_matrix;
  new_calibration_event.status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

  DBG_INST_PRINTF_EX(MED, instance, "CAL_EVENT sensor=%u", sensor_type);
  pb_send_event(instance,
                sns_cal_event_fields,
                &new_calibration_event,
                src_cfg->ts,
                SNS_CAL_MSGID_SNS_CAL_EVENT,
                suid_current);
}

void lsm6dso_start_sensor_temp_polling_timer(sns_sensor_instance *this)
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
  req_payload.timeout_period = state->sensor_temp_info.sampling_intvl;

  timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                                            sns_timer_sensor_config_fields, NULL);
  if(timer_req.request_len > 0)
  {
    if(state->timer_sensor_temp_data_stream == NULL)
    {
      sns_service_manager *service_mgr = this->cb->get_service_manager(this);
      sns_stream_service *stream_mgr = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
          this,
          state->timer_suid,
          &state->timer_sensor_temp_data_stream);
      if(state->timer_sensor_temp_data_stream == NULL)
      {
        SNS_INST_PRINTF(ERROR, this, "timer_sensor_temp NULL");
        return;
      }
    }
    if(state->sensor_temp_info.desired_sampling_rate_hz !=
       state->sensor_temp_info.cur_sampling_rate_hz) {
      DBG_INST_PRINTF(HIGH, this, "setting timer: rate(*1000)=%d period=%u",
                      (int)(state->sensor_temp_info.desired_sampling_rate_hz*1000),
                      (uint32_t)req_payload.timeout_period);
      state->timer_sensor_temp_data_stream->api->
        send_request(state->timer_sensor_temp_data_stream, &timer_req);
      state->sensor_temp_info.cur_sampling_rate_hz =
        state->sensor_temp_info.desired_sampling_rate_hz;
    }
    state->sensor_temp_info.timer_is_active = true;
  }
  else
  {
    //SNS_INST_PRINTF(LOW, this,
    //                         "encode err");
  }
}

void lsm6dso_turn_off_fifo(sns_sensor_instance *this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  DBG_INST_PRINTF(HIGH, this, "turn_off_fifo: rate=0x%x", state->fifo_info.fifo_rate);
  if(state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF) {
    lsm6dso_set_fifo_config(this,
        0,
        LSM6DSO_ACCEL_ODR_OFF,
        LSM6DSO_GYRO_ODR_OFF,
        state->fifo_info.fifo_enabled);
    lsm6dso_stop_fifo_streaming(this);
    lsm6dso_set_fifo_wmk(this);
    lsm6dso_disable_fifo_intr(this);
    // Disable timer
    state->health.expected_expiration = UINT64_MAX;
    state->health.heart_beat_timeout = UINT64_MAX/2;
    lsm6dso_restart_hb_timer(this, true);
    state->health.heart_attack = false;
    state->fifo_info.last_sent_config.sample_rate = LSM6DSO_ACCEL_ODR_OFF;
    state->fifo_info.last_sent_config.fifo_watermark = 0;
    state->fifo_info.last_sent_config.timestamp = sns_get_system_time();
   }
}
void lsm6dso_powerdown_hw(sns_sensor_instance *this)
{
  DBG_INST_PRINTF(HIGH, this, "power down sensor");
  lsm6dso_enable_hw_timestamp(this, false);
  lsm6dso_turn_off_fifo(this);
  lsm6dso_disable_md(this, false);
  lsm6dso_set_polling_config(this);
  lsm6dso_reconfig_esp(this);
}
/**sets fifo odr = 0
 * this retains old data and
 * new data doesn't store */
#if 0
void lsm6dso_disable_fifo(sns_sensor_instance *this)
{
  uint8_t rw_buffer = 0x06 ;
  uint32_t xfer_bytes = 0;
  lsm6dso_read_modify_write(this,
      STM_LSM6DSO_REG_FIFO_CTRL5,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      0xFF);
}
#endif


void lsm6dso_set_fifo_bypass_mode(sns_sensor_instance *this)
{
  uint8_t rw_buffer = 0x0;
  uint32_t xfer_bytes;
  lsm6dso_read_modify_write(this,
      STM_LSM6DSO_REG_FIFO_CTRL4,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      STM_LSM6DSO_FIFO_MODE_MASK);
}

void lsm6dso_set_fifo_stream_mode(sns_sensor_instance *this)
{
  uint8_t rw_buffer = LSM6DSO_FIFO_STREAM_MODE;
  uint32_t xfer_bytes;
  lsm6dso_read_modify_write(this,
      STM_LSM6DSO_REG_FIFO_CTRL4,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      STM_LSM6DSO_FIFO_MODE_MASK);
}

// reconfig fifo after flush or without flush
void lsm6dso_reconfig_fifo(sns_sensor_instance *this, bool flush)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  bool set_bypass_mode = false;
  bool was_gyro_enabled = state->common_info.gyro_curr_odr;

  DBG_INST_PRINTF_EX(HIGH, this, "reconfig_fifo En: flush=%u cur0x%x des=0x%x",
                  flush, state->fifo_info.fifo_rate, state->fifo_info.desired_fifo_rate);

  lsm6dso_disable_fifo_intr(this);

  if(state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF || LSM6DSO_IS_ESP_ENABLED(state)) {
    if(flush && (state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF))
      lsm6dso_flush_fifo(this);
    if((state->ascp_req_count <= 0) && state->fifo_info.reconfig_req && 
        (state->desired_conf.odr != state->current_conf.odr)) {
      lsm6dso_set_fifo_bypass_mode(this);
      set_bypass_mode = true;
    }
  }

  if((state->ascp_req_count <= 0) && state->fifo_info.reconfig_req) {
    state->prev_odr_change_info.odr_change_timestamp = state->cur_odr_change_info.odr_change_timestamp;
    lsm6dso_set_fifo_wmk(this);
    lsm6dso_start_fifo_streaming(this);

    sns_time config_change_ts = (state->cur_odr_change_info.gyro_odr_settime > state->cur_odr_change_info.accel_odr_settime) ?
               state->cur_odr_change_info.gyro_odr_settime : state->cur_odr_change_info.accel_odr_settime;
    state->cur_odr_change_info.odr_change_timestamp = 
      (config_change_ts > state->cur_odr_change_info.odr_change_timestamp) ? 
      config_change_ts : state->cur_odr_change_info.odr_change_timestamp;

    if(state->fifo_info.last_sent_config.fifo_watermark == 0) {
      state->fifo_info.new_config.fifo_watermark = state->fifo_info.desired_wmk;
      state->fifo_info.new_config.sample_rate =
        (float)(13 << ((state->fifo_info.desired_fifo_rate >> 4) - 1));
      state->fifo_info.new_config.timestamp = state->cur_odr_change_info.odr_change_timestamp;
#if LSM6DSO_DAE_ENABLED
      state->fifo_info.new_config.dae_watermark  = state->fifo_info.max_requested_wmk;
#endif
      lsm6dso_send_config_event(this, false);
    }
#if LSM6DSO_DAE_ENABLED
    state->config_step = LSM6DSO_CONFIG_IDLE; /* done with reconfig */
#endif
    state->fifo_info.reconfig_req = false;
    state->fifo_info.full_reconf_req = false;
    if(set_bypass_mode && !lsm6dso_dae_if_available(this)) {
      state->fifo_info.orphan_batch = true;
      sns_time sampling_intvl = state->prev_odr_change_info.nominal_sampling_intvl;
      if(sns_get_system_time() > state->fifo_info.last_timestamp +  sampling_intvl) {
        //check if there is a gap of 1 sample time
        lsm6dso_adjust_ts_drift(this, was_gyro_enabled, sampling_intvl,
            state->fifo_info.last_timestamp,
            state->fifo_info.last_timestamp + sampling_intvl);
      }
      state->fifo_info.orphan_batch = false;
    }

    lsm6dso_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
  }

  DBG_INST_PRINTF(HIGH, this, "reconfig_fifo EX: flush=%u cur=0x%x",
                  flush, state->fifo_info.fifo_rate);
}

void lsm6dso_reconfig_hw(sns_sensor_instance *this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  //tells whether fifo int is req or not
  bool is_fifo_int_req =
    (state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO));
  bool is_md_int_req = lsm6dso_is_md_int_required(this);    //tells whether md int is req or not
  bool is_sensor_active; //tells whether sensor is req or not
  bool md_force_disable = false;
  is_sensor_active = (is_md_int_req ||
                      state->desired_conf.publish_sensors ||
                      LSM6DSO_IS_ESP_DESIRED(state) ||
                      state->accel_info.gated_client_present);

  bool active_goes_passive_remains_a_g = 
                   (!state->desired_conf.enabled_sensors && state->current_conf.enabled_sensors
                     && (state->passive_client_present & (LSM6DSO_ACCEL | LSM6DSO_GYRO)));
  bool active_goes_passive_remains_temp = 
                   (!state->sensor_temp_info.desired_sampling_rate_hz && state->sensor_temp_info.cur_sampling_rate_hz
                     && (state->passive_client_present & LSM6DSO_SENSOR_TEMP));
  bool active_goes_passive_remains = active_goes_passive_remains_a_g | active_goes_passive_remains_temp;
#if LSM6DSO_DAE_ENABLED
  DBG_INST_PRINTF_EX(
    HIGH, this, "reconfig_hw: %u %u %u %u %u",
    is_fifo_int_req, is_md_int_req, is_sensor_active, state->fifo_info.reconfig_req,
    state->config_step);
#else
  DBG_INST_PRINTF_EX(
    HIGH, this, "reconfig_hw: %u %u %u %u",
    is_fifo_int_req, is_md_int_req, is_sensor_active, state->fifo_info.reconfig_req);
#endif
  bool reconfig_req = state->fifo_info.reconfig_req;
  if(!is_sensor_active) {
    lsm6dso_powerdown_hw(this);
  } else {
    if(state->fifo_info.reconfig_req && is_fifo_int_req) {
      //flush before configuring in non-dae mode
      lsm6dso_reconfig_fifo(this, lsm6dso_dae_if_available(this) ? false : true);
    } else if(!is_fifo_int_req && (state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF)) {
      lsm6dso_turn_off_fifo(this);
      //if accel is off, need settling time for md
      if(is_md_int_req && state->current_conf.md_enabled) {
        md_force_disable = true;
        lsm6dso_disable_md(this, false);
      }
    }
    //avoid trying to configure md if md timer is running
    if(is_md_int_req) {
      // case 1: first MD request, enable
      if(!state->current_conf.md_enabled && !state->md_info.is_timer_running) {
        lsm6dso_enable_md(this, !md_force_disable);
      } else {
        // case 2: previous MD req is processing or MD enabled, send event
        // only if MD request is configuring
        if((state->config_sensors & LSM6DSO_MOTION_DETECT) && state->md_info.client_present) {
          lsm6dso_send_config_event(this, false);
          if(state->md_info.add_request)
          {
            lsm6dso_send_md_event(this, &state->md_info.cur_state, state->md_info.event_ts);
          }
        }
      }
    } else {
      if(state->current_conf.md_enabled || state->md_info.is_timer_running) {
        lsm6dso_disable_md(this, true);
      } else {
        //case 1: MD client present, non-gated accel req present
        //send disable event
        if((state->config_sensors & LSM6DSO_MOTION_DETECT)
            && state->md_info.client_present) {
          lsm6dso_send_md_event(this, &state->md_info.cur_state, state->md_info.event_ts);
        }
      }
      state->md_info.is_timer_running = false;
    }

    if (!lsm6dso_dae_if_available(this))
      state->config_sensors &= ~LSM6DSO_MOTION_DETECT;

    //if accel is still off, turn on some one need this
    if(state->common_info.accel_curr_odr == LSM6DSO_ACCEL_ODR_OFF) {
      lsm6dso_accel_odr accel_odr = state->accel_info.desired_odr;
      //Set minimum ODR
      if(state->accel_info.desired_odr == LSM6DSO_ACCEL_ODR_OFF)
      {
        if(LSM6DSO_IS_ESP_ENABLED(state))
        {
          accel_odr = (lsm6dso_accel_odr)lsm6dso_get_esp_rate_idx(this);
        }
        else
        {
          accel_odr = lsm6dso_odr_map[state->min_odr_idx].accel_odr_reg_value;
        }
      }
      lsm6dso_set_accel_config(this,
          accel_odr,
          state->accel_info.sstvt,
          state->accel_info.range,
          state->accel_info.bw);
    }

    if(state->ascp_req_count <= 0)
      lsm6dso_reconfig_esp(this);

    if(state->fifo_info.reconfig_req && !is_fifo_int_req) {
      state->fifo_info.reconfig_req = false;
      reconfig_req = false;
    }

    DBG_INST_PRINTF(MED, this, "pub=0x%x md=%u",
                    state->desired_conf.publish_sensors, state->md_info.client_present);

    if(state->fifo_info.fifo_enabled != 0 ||
       state->desired_conf.publish_sensors != 0 ||
       lsm6dso_is_md_int_required(this) ||
       LSM6DSO_IS_ESP_ENABLED(state) ||
       LSM6DSO_IS_XSENSOR_TIMER_ON(state)
      ) {
#if LSM6DSO_DAE_ENABLED
      if(lsm6dso_dae_if_available(this)) {
        lsm6dso_dae_if_start_streaming(this);
      }
      else
#endif
      if((state->desired_conf.publish_sensors & LSM6DSO_SENSOR_TEMP) ||
         (state->sensor_temp_info.timer_is_active)) {
        lsm6dso_set_polling_config(this);
      }
      lsm6dso_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
      //overwrite publish sensors only if reconfigure is not required
      if(!reconfig_req)
        state->current_conf.publish_sensors = state->desired_conf.publish_sensors;
    }
  }
   /*send config event
  when active sensor goes out and passive sensor is present
  with config event timestamp of when reconfiguration odr->0)
  */
  if(active_goes_passive_remains)
  {
    if(active_goes_passive_remains_a_g)
    {
      if(state->cur_odr_change_info.accel_odr_settime)
        state->fifo_info.last_sent_config.timestamp = state->cur_odr_change_info.accel_odr_settime;
    }
    if(active_goes_passive_remains_temp)
    {
      state->sensor_temp_info.last_sent_config.timestamp = sns_get_system_time();
    }
    lsm6dso_send_config_event(this, true);
  }

#if LSM6DSO_DAE_ENABLED
  DBG_INST_PRINTF(MED, this, "step=%u", state->config_step);
#endif
}

void lsm6dso_register_interrupt(
    sns_sensor_instance *this,
    lsm6dso_irq_info* irq_info,
    sns_data_stream* data_stream)
{
  UNUSED_VAR(this);
  uint8_t buffer[20];
  sns_request irq_req =
  {
    .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
    .request    = buffer
  };
  if(!irq_info->irq_registered)
  {
    irq_req.request_len = pb_encode_request(buffer,
                                            sizeof(buffer),
                                            &irq_info->irq_config,
                                            sns_interrupt_req_fields,
                                            NULL);
    if(irq_req.request_len > 0)
    {
      data_stream->api->send_request(data_stream, &irq_req);
      irq_info->irq_registered = true;
    }
  }
}

