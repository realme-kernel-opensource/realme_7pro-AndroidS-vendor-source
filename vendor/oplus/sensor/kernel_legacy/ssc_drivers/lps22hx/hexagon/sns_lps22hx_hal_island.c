/**
 * @file sns_lps22hx_hal.c
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
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "sns_lps22hx_hal.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"

#include "sns_timer.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"

/** Need to use ODR table. */
extern const odr_reg_map lps22hx_odr_map[LPS22HX_REG_MAP_TABLE_SIZE];

typedef struct log_sensor_state_raw_info
{
  /* Pointer to diag service */
  sns_diag_service *diag;
  /* Pointer to sensor instance */
  sns_sensor_instance *instance;
  /* Pointer to sensor UID*/
  struct sns_sensor_uid *sensor_uid;
  /* Size of a single encoded sample */
  size_t encoded_sample_size;
  /* Pointer to log*/
  void *log;
  /* Size of allocated space for log*/
  uint32_t log_size;
  /* Number of actual bytes written*/
  uint32_t bytes_written;
  /* Number of batch samples written*/
  /* A batch may be composed of several logs*/
  uint32_t batch_sample_cnt;
  /* Number of log samples written*/
  uint32_t log_sample_cnt;
} log_sensor_state_raw_info;

// Unencoded batch sample
typedef struct
{
  /* Batch Sample type as defined in sns_diag.pb.h */
  sns_diag_batch_sample_type sample_type;
  /* Timestamp of the sensor state data sample */
  sns_time timestamp;
  /*Raw sensor state data sample*/
  float sample[1];
  /* Data status.*/
  sns_std_sensor_sample_status status;
} lps22hx_batch_sample_oneaxis;

/**
 * Encode Sensor State Log.Interrupt
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lps22hx_encode_sensor_state_log_interrupt(
    void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
    size_t *bytes_written)
{
  UNUSED_VAR(log_size);
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == encoded_log || NULL == log || NULL == bytes_written)
  {
    return SNS_RC_FAILED;
  }

  sns_diag_sensor_state_interrupt *sensor_state_interrupt =
    (sns_diag_sensor_state_interrupt *)log;
  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);

  if(!pb_encode(&stream, sns_diag_sensor_state_interrupt_fields,
        sensor_state_interrupt))
  {
    rc = SNS_RC_FAILED;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }

  return rc;
}

/**
 * Encode log sensor state raw packet for sensor with 1 axis data
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lps22hx_encode_log_sensor_state_raw_oneaxis(
    void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
    size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[1];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = 1,
    .arr_index = &arr_index};

  if(NULL == encoded_log || NULL == log || NULL == bytes_written)
  {
    return SNS_RC_FAILED;
  }

  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
        &batch_sample))
  {
    return SNS_RC_FAILED;
  }

  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
  lps22hx_batch_sample_oneaxis *raw_sample = (lps22hx_batch_sample_oneaxis *)log;

  while(parsed_log_size < log_size &&
      (stream.bytes_written + encoded_sample_size)<= encoded_log_size &&
      i < (uint32_t)(-1))
  {
    arr_index = 0;
    arg.arr = (float *)raw_sample[i].sample;

    batch_sample.sample_type = raw_sample[i].sample_type;
    batch_sample.status = raw_sample[i].status;
    batch_sample.timestamp = raw_sample[i].timestamp;

    if(!pb_encode_tag(&stream, PB_WT_STRING,
          sns_diag_sensor_state_raw_sample_tag))
    {
      rc = SNS_RC_FAILED;
      break;
    }
    else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
          &batch_sample))
    {
      rc = SNS_RC_FAILED;
      break;
    }

    parsed_log_size += sizeof(lps22hx_batch_sample_oneaxis);
    i++;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }

  return rc;
}


/**
 * Encode log sensor state raw packet
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lps22hx_encode_log_sensor_state_raw(
    void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
    size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample sample = sns_diag_batch_sample_init_default;

  if(NULL == encoded_log || NULL == log || NULL == bytes_written)
  {
    return SNS_RC_FAILED;
  }

  if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
        &sample))
  {
    return SNS_RC_FAILED;
  }

  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
  sns_diag_batch_sample *batch_sample = (sns_diag_batch_sample *)log;

  while(parsed_log_size < log_size &&
      (stream.bytes_written + encoded_sample_size)<= encoded_log_size &&
      i < (uint32_t)(-1))
  {
    if(!pb_encode_tag(&stream, PB_WT_STRING,
          sns_diag_sensor_state_raw_sample_tag))
    {
      rc = SNS_RC_FAILED;
      break;
    }
    else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
          &batch_sample[i]))
    {
      rc = SNS_RC_FAILED;
      break;
    }

    parsed_log_size += sizeof(sns_diag_batch_sample);
    i++;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }

  return rc;
}

/**
 * Allocate Sensor State Raw Log Packet
 *
 + * @param[i] diag       Pointer to diag service
 + * @param[i] log_size   Optional size of log packet to
 + *    be allocated. If not provided by setting to 0, will
 + *    default to using maximum supported log packet size
 */
void lps22hx_log_sensor_state_raw_alloc(
    log_sensor_state_raw_info *log_raw_info,
    uint32_t log_size)
{
  uint32_t max_log_size = 0;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
      NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid)
  {
    return;
  }

  // allocate memory for sensor state - raw sensor log packet
  max_log_size = log_raw_info->diag->api->get_max_log_size(
      log_raw_info->diag);

  if(0 == log_size)
  {
    // log size not specified.
    // Use max supported log packet size
    log_raw_info->log_size = max_log_size;
  }
  else if(log_size <= max_log_size)
  {
    log_raw_info->log_size = log_size;
  }
  else
  {
    return;
  }

  log_raw_info->log = log_raw_info->diag->api->alloc_log(
      log_raw_info->diag,
      log_raw_info->instance,
      log_raw_info->sensor_uid,
      log_raw_info->log_size,
      SNS_DIAG_SENSOR_STATE_LOG_RAW);

  log_raw_info->log_sample_cnt = 0;
  log_raw_info->bytes_written = 0;
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void lps22hx_log_sensor_state_raw_submit(
    log_sensor_state_raw_info *log_raw_info,
    uint8_t sample_size,
    bool batch_complete)
{
  lps22hx_batch_sample_oneaxis *sample = NULL;

  if(sample_size > 1)
    return;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
      NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
      NULL == log_raw_info->log)
  {
    return;
  }

  sample = (lps22hx_batch_sample_oneaxis *)log_raw_info->log;

  if(batch_complete)
  {
    // overwriting previously sample_type for last sample
    if(1 == log_raw_info->batch_sample_cnt)
    {
      sample[0].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
    }
    else if(1 < log_raw_info->batch_sample_cnt)
    {
      sample[log_raw_info->log_sample_cnt - 1].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
    }
  }
  log_raw_info->diag->api->submit_log(
      log_raw_info->diag,
      log_raw_info->instance,
      log_raw_info->sensor_uid,
      log_raw_info->bytes_written,
      log_raw_info->log,
      SNS_DIAG_SENSOR_STATE_LOG_RAW,
      log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
      lps22hx_encode_log_sensor_state_raw_oneaxis);

  log_raw_info->log = NULL;
}



/**
 *
 * Add raw uncalibrated sensor data to Sensor State Raw log
 * packet
 *
 * @param[i] log_raw_info Pointer to logging information
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 *
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lps22hx_log_sensor_state_raw_add(
    log_sensor_state_raw_info *log_raw_info,
    float *raw_data,
    uint8_t  sample_size,
    sns_time timestamp,
    sns_std_sensor_sample_status status)
{
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
      NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
      NULL == raw_data || NULL == log_raw_info->log)
  {
    return SNS_RC_FAILED;
  }

  uint8_t batch_sample_size = sizeof(lps22hx_batch_sample_oneaxis);

  if( (log_raw_info->bytes_written + batch_sample_size) >
      log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    lps22hx_log_sensor_state_raw_submit(log_raw_info, sample_size, false);
    lps22hx_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    lps22hx_batch_sample_oneaxis *sample_oneaxis =
      (lps22hx_batch_sample_oneaxis *)log_raw_info->log;
    if(0 == log_raw_info->batch_sample_cnt)
    {
      sample_oneaxis[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
    }
    else
    {
      sample_oneaxis[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
    }

    sample_oneaxis[log_raw_info->log_sample_cnt].timestamp = timestamp;

    sns_memscpy(sample_oneaxis[log_raw_info->log_sample_cnt].sample,
        sizeof(sample_oneaxis[log_raw_info->log_sample_cnt].sample),
        raw_data,
        sizeof(sample_oneaxis[log_raw_info->log_sample_cnt].sample));

    sample_oneaxis[log_raw_info->log_sample_cnt].status = status;

    log_raw_info->bytes_written += batch_sample_size;

    log_raw_info->log_sample_cnt++;
    log_raw_info->batch_sample_cnt++;
  }

  return rc;
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
static sns_rc lps22hx_com_read_wrapper(
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
static sns_rc lps22hx_com_write_wrapper(
    sns_sensor_instance *const instance,
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

  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;

  return inst_state->scp_service->api->sns_scp_register_rw(inst_state->com_port_info.port_handle,
      &port_vec,
      1,
      save_write_time,
      xfer_bytes);
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
sns_rc lps22hx_read_modify_write(
    sns_sensor_instance *const instance,
    uint32_t reg_addr,
    uint8_t *reg_value,
    uint32_t size,
    uint32_t *xfer_bytes,
    bool save_write_time,
    uint8_t mask)
{
  uint8_t rw_buffer = 0;
  uint32_t rw_bytes = 0;

  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;
  sns_sync_com_port_handle* port_handle = state->com_port_info.port_handle;

  if((size > 1) || (mask == 0xFF) || (mask == 0x00))
  {
    lps22hx_com_write_wrapper(instance,
        reg_addr,
        &reg_value[0],
        size,
        xfer_bytes,
        save_write_time);

  }
  else
  {
    // read current value from this register
    lps22hx_com_read_wrapper(state->scp_service,
        port_handle,
        reg_addr,
        &rw_buffer,
        1,
        &rw_bytes);

    // generate new value
    rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

    // write new value to this register
    lps22hx_com_write_wrapper(instance,
        reg_addr,
        &rw_buffer,
        1,
        xfer_bytes,
        save_write_time);

  }

  return SNS_RC_SUCCESS;;
}

/**
 * see sns_lps22hx_hal.h
 */
sns_rc lps22hx_reset_device(
    sns_sensor_instance *const instance,
    lps22hx_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t reg;
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;
  DBG_INST_PRINT_EX(instance, HIGH, __FILENAME__, __LINE__, "reset_device");
  // Init CTRL_REG1, put LPS22HX into power down.
  reg = 0
    | (0<<4)        // ODR[2:0]: 0-1S/1S;1-1/1Hz;2-10/10;3-25/25;4-50/50;5-75/75;
    | (LPS22HX_ENABLE_LPF<<3) // EN_LPFP: 0-low pass filter disabled; 1-enabled.
    | (1<<2)        // LPF_CFG: 0-ODR/9; 1-ODR/20.
#if LPS22HX_DEVICE_LPS22HB
    | (0<<1)        // BDU
#else
    | (1<<1)        // BDU
#endif
    | 0;            // SIM: 0-4 wire SPI; 1-3wire SPI.
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_CTRL_REG1,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    return rv;
  }
  /* create mirror reg. Avoids reading HW reg before every write */
  state->ctrl_reg1 = reg;

  /* wait for 1ms for sensor to power down */
  sns_busy_wait(sns_convert_ns_to_ticks(1*1000*1000));

  // Init INTERRUPT_CFG
  reg = 0;
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_INTERRUPT_CFG,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    return rv;
  }

  // Init CTRL_REG2
  reg = 0
    | (0<<7)        // BOOT
    | (0<<6)        // FIFO_EN
    | (0<<5)        // STOP_ON_FIFO:
    | (1<<4)        // IF_ADD_INC:
    | (0<<3)        // I2C_DIS: 0-I2c enabled; 1-disable. / Not Used
    | (0<<2)        // SWRESET: 0-Normal mode; 1-Software reset.
#if LPS22HX_DEVICE_LPS22HB
    | (0<<1)
#else
    | (1<<1)        // LOW_NOISE_EN. 0-low current mode;1-no noise.
#endif
    | 0;            // OneShotEnable when ODR=0: 0-wait for start of conversion; 1-start for a new data set.
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_CTRL_REG2,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    return rv;
  }

  // Init FIFO_CTRL
  reg=0;
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_FIFO_CTRL,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    return rv;
  }

#if LPS22HX_DEVICE_LPS22HB
  // Init RES_CONF
  reg = 0
    | 0;            // LC_EN: 0-Normal mode; 1-Low Current Mode.
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_RES_CONF,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    return rv;
  }
#endif
  return SNS_RC_SUCCESS;
}


/** See sns_lps22hx_hal.h */
sns_rc  lps22hx_set_odr(sns_sensor_instance *const instance, uint8_t idx)
{
  uint32_t xfer_bytes;
  sns_rc rv = SNS_RC_SUCCESS;

  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;

  /* Samples to discard on ODR change */
  state->press_info.num_samples_to_discard = lps22hx_odr_map[idx].press_discard_samples;

  state->ctrl_reg1=(state->ctrl_reg1&0x0F) | (lps22hx_odr_map[idx].odr_reg_value<<4);
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_CTRL_REG1,
      &state->ctrl_reg1,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS	|| xfer_bytes != 1)
  {
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "ODR update err");
    return rv;
  }

#if LPS22HX_DUMP_REG
      lps22hx_dump_reg(instance, LPS22HX_PRESS);
#endif

#if (ENABLE_LPS22HB && LPS22HX_ENABLE_LPF)
  if (idx>0)
  {
    uint8_t rw_buffer = 0;
    // AN4833: reset LPF immediately after setting ODR.
    rv = lps22hx_com_read_wrapper(state->scp_service,
        state->com_port_info.port_handle,
        0x33,
        &rw_buffer,
        1,
        &xfer_bytes);

    if(rv != SNS_RC_SUCCESS
        ||
        xfer_bytes != 1)
    {
      DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
          "reset LPF err");
      return rv;
    }
  }
#endif
#if 0
  if(idx == 0)
    lps22hx_enable_intr(instance, false);
  else
    lps22hx_enable_intr(instance, true);
#endif

  return rv;
}

sns_rc lps22hx_enable_intr(sns_sensor_instance *const instance)
{
  uint8_t reg;
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  /* enable DRDY interrupt */
  // Init CTRL_REG3
  reg = 0
    | (0<<7)        // INT_H_L: 0-interrupt active high; 1-active low. / Not used
    | (0<<6)        // PP_OD: 0-push pull; 1-open drain. / Not used
    | (0<<5)        // F_FSS5: 0-disable; 1-FIFO full on INT_DRDY pin.
    | (0<<4)        // F_FTH:
    | (0<<3)        // F_OVR:
    | (1<<2)        // DRDY:
    | 0;            // INT1_S[1:0]: 0-Data Signal; 1-P High; 2-P Low; 3-P_low or P_high;
  rv = lps22hx_com_write_wrapper(instance,
      STM_LPS22HX_CTRL_REG3,
      &reg,
      1,
      &xfer_bytes,
      false);
  if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
  {
    return rv;
  }
  return rv;
}

static sns_rc lps22hx_get_data(sns_sensor_instance *const instance, float *pressure_data, float *temp_data)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t reg[LPS22HX_NUM_BYTES_PRESS+LPS22HX_NUM_BYTES_TEMP];
  uint32_t xfer_bytes;
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;

  rv = lps22hx_com_read_wrapper(state->scp_service,
      state->com_port_info.port_handle,
      STM_LPS22HX_PRESS_OUT_XL,
      &reg[0],
      (LPS22HX_NUM_BYTES_PRESS+LPS22HX_NUM_BYTES_TEMP),
      &xfer_bytes);
  if(rv != SNS_RC_SUCCESS || xfer_bytes != (LPS22HX_NUM_BYTES_PRESS+LPS22HX_NUM_BYTES_TEMP))
  {
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "data read err");
    return rv;
  }

  *pressure_data= (float)((reg[2]<<16)|(reg[1]<<8)|reg[0]) / 4096.0;
  *temp_data= (float)((reg[4]<<8)|reg[3]) / 100.0;

  return rv;

}

static void lps22hx_read_and_send_data(sns_sensor_instance * const instance, sns_time timestamp)
{
  sns_rc rv = SNS_RC_SUCCESS;
  float pressure_data[1] = {0};
  float temp_data[1] = {0};
  sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;
#if LPS22HX_ENABLE_LOGGING
  sns_diag_service* diag = state->diag_service;
  log_sensor_state_raw_info log_pressure_state_raw_info;
  log_sensor_state_raw_info log_temp_state_raw_info;
#endif


  /* Read data from device */
  rv=lps22hx_get_data(instance, pressure_data, temp_data);
  if (SNS_RC_SUCCESS!=rv)
    return;

  if(state->press_info.num_samples_to_discard != 0)
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    DBG_INST_PRINT_EX(instance, HIGH, __FILENAME__, __LINE__,
      "status %d #_discard %d",
      status, state->press_info.num_samples_to_discard);
    state->press_info.num_samples_to_discard--;
  }

  /* Send data to framework */
  if(state->publish_sensors & LPS22HX_PRESS)
  {
    pb_send_sensor_stream_event(instance,
        &state->press_info.suid,
        timestamp,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        status,
        pressure_data,
        ARR_SIZE(pressure_data),
        state->encoded_press_event_len);
#if LPS22HX_LOG_DATA
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "Pressure Sensor Data(*1000) = %d timestamp = %u",
        (int32_t)(pressure_data[0]*1000), (uint32_t)timestamp);
#endif
#if LPS22HX_ENABLE_LOGGING //Logging
    // Allocate Sensor State Raw log packets for pressure
    sns_memzero(&log_pressure_state_raw_info, sizeof(log_pressure_state_raw_info));
    log_pressure_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
    log_pressure_state_raw_info.diag = diag;
    log_pressure_state_raw_info.instance = instance;
    log_pressure_state_raw_info.sensor_uid = &state->press_info.suid;
    lps22hx_log_sensor_state_raw_alloc(&log_pressure_state_raw_info, 0);
    // Log raw uncalibrated sensor data
    lps22hx_log_sensor_state_raw_add(
        &log_pressure_state_raw_info,
        pressure_data,
        LPS22HX_PRESS_NUM_AXES,
        timestamp,
        status);
    lps22hx_log_sensor_state_raw_submit(&log_pressure_state_raw_info, 1, true);
#endif
  }
  if(state->publish_sensors & LPS22HX_SENSOR_TEMP)
  {
    pb_send_sensor_stream_event(instance,
        &state->temp_info.suid,
        timestamp,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        status,
        temp_data,
        ARR_SIZE(temp_data),
        state->encoded_sensor_temp_event_len);
#if LPS22HX_LOG_DATA
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "Temprature Sensor Data(*1000) = %d timestamp = %u",
        (int32_t)(temp_data[0]*1000), (uint32_t)timestamp);
#endif
#if LPS22HX_ENABLE_LOGGING //Logging
    // Allocate Sensor State Raw log packets for temperature
    sns_memzero(&log_temp_state_raw_info, sizeof(log_temp_state_raw_info));
    log_temp_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
    log_temp_state_raw_info.diag = diag;
    log_temp_state_raw_info.instance = instance;
    log_temp_state_raw_info.sensor_uid = &state->temp_info.suid;
    lps22hx_log_sensor_state_raw_alloc(&log_temp_state_raw_info, 0);
    // Log raw uncalibrated sensor data
    lps22hx_log_sensor_state_raw_add(
        &log_temp_state_raw_info,
        temp_data,
        LPS22HX_TEMP_NUM_AXES,
        timestamp,
        status);
    lps22hx_log_sensor_state_raw_submit(&log_temp_state_raw_info, 1, true);
#endif
  }
}

void lps22hx_handle_sensor_sample(sns_sensor_instance * const instance, sns_time timestamp)
{
  lps22hx_read_and_send_data(instance,timestamp);
}

/**
 * Provides sample interval based on current ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 * @return sampling interval time in ticks
 */
static sns_time lps22hx_get_sample_interval(lps22hx_sensor_odr curr_odr)
{
  sns_time  sample_interval = 0;

  if(curr_odr > 0.0)
  {
    sample_interval = sns_convert_ns_to_ticks(1000000000 / curr_odr);
  }

  return sample_interval;
}

void lps22hx_start_sensor_polling_timer(sns_sensor_instance *const instance)
{
  sns_time  sample_interval = 0;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  uint8_t buffer[50];
  sns_request timer_req = {
    .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
    .request    = buffer
  };
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;

  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time();
  sample_interval = lps22hx_get_sample_interval(SNS_MAX(state->press_info.current_odr, state->temp_info.current_odr));
  state->press_info.sample_period = sample_interval;
  req_payload.timeout_period = sample_interval;

  timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
      sns_timer_sensor_config_fields, NULL);
  if(timer_req.request_len > 0)
  {
    if(state->timer_data_stream == NULL)
    {
      sns_service_manager *service_mgr = instance->cb->get_service_manager(instance);
      sns_stream_service *stream_mgr = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
          instance,
          state->timer_suid,
          &state->timer_data_stream);
    }
    if(state->timer_data_stream != NULL)
    {
      state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
      state->press_info.timer_is_active = true;
      state->temp_info.timer_is_active = true;
    }
    else
    {
      DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
          "Error: timer stream NULL");
    }
  }
  else
  {
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "timer req encode err");
  }
}

/**
 * Updates temp & press sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
void lps22hx_set_polling_config(sns_sensor_instance *const this)
{
  lps22hx_instance_state *state = (lps22hx_instance_state*)this->state->state;

  if((state->press_info.current_odr > 0)||(state->temp_info.current_odr > 0))
  {
    lps22hx_start_sensor_polling_timer(this);
  }
  else if(state->press_info.timer_is_active)
  {
    state->press_info.timer_is_active = false;
    state->temp_info.timer_is_active = false;
  }
}

void lps22hx_handle_interrupt_event(sns_sensor_instance *const instance)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;

  sns_time timestamp = state->interrupt_timestamp;

  if ((state->interrupt_timestamp - state->last_sample_ts) < sns_convert_ns_to_ticks(2000000000/LPS22HX_ODR_25)) //TBD - need to clarify this with Wei */
  {
    // If IRQ interval is less than 1/2 Maximum ODR, treat this IRQ as false IRQ.
    DBG_INST_PRINT(instance, ERROR, __FILENAME__, __LINE__, "False IRQ");
    return;
  }
  lps22hx_read_and_send_data(instance,timestamp);
}

/**
 * see sns_lps22hx_hal.h
 */
sns_rc lps22hx_get_who_am_i(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = lps22hx_com_read_wrapper(scp_service,
      port_handle,
      STM_LPS22HX_WHO_AM_I,
      buffer,
      1,
      &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
      ||
      xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * see sns_lps22hx_hal.h
 */
#if LPS22HX_DUMP_REG
void lps22hx_dump_reg(sns_sensor_instance *const instance,
    lps22hx_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  lps22hx_instance_state *state = (lps22hx_instance_state*)instance->state->state;
  uint32_t xfer_bytes;
  uint8_t odr_map[] = {
  STM_LPS22HX_INTERRUPT_CFG,
  STM_LPS22HX_THS_P_L,
  STM_LPS22HX_THS_P_H,
  STM_LPS22HX_WHO_AM_I,
  STM_LPS22HX_CTRL_REG1,
  STM_LPS22HX_CTRL_REG2,
  STM_LPS22HX_CTRL_REG3,
  STM_LPS22HX_FIFO_CTRL,
#if LPS22HX_DEVICE_LPS22HB
  STM_LPS22HX_FIFO_STATUS,
#else
  STM_LPS22HX_FIFO_STATUS1,
  STM_LPS22HX_FIFO_STATUS2,
#endif
  };

  uint8_t i = 0;
  uint16_t n = sizeof(odr_map)/sizeof(odr_map[0]);

  for(i=0; i<n;i++)
  {
    lps22hx_com_read_wrapper(state->scp_service,
        state->com_port_info.port_handle,
        odr_map[i],
        &state->reg_status[i],
        1,
        &xfer_bytes);
    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "reg 0x%x = 0x%x",odr_map[i], state->reg_status[i]);

  }
}
#endif

/** See sns_lps22hx_hal.h */
void lps22hx_send_config_event(sns_sensor_instance *const instance)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
    sns_std_sensor_physical_config_event_init_default;

  // TODO: Use appropriate op_mode selected by driver.
  char operating_mode[] = "NORMAL";

  pb_buffer_arg op_mode_args;
  pb_buffer_arg payload_args;

  op_mode_args.buf = &operating_mode[0];
  op_mode_args.buf_len = sizeof(operating_mode);

  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.sample_rate = state->press_info.current_odr;
  phy_sensor_config.has_water_mark = false;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_active_current = true;
  phy_sensor_config.active_current = STM_LPS22HX_ACTIVE_CURRENT;
  phy_sensor_config.has_resolution = true;
  phy_sensor_config.resolution = LPS22HX_PRESS_RESOLUTION;
  phy_sensor_config.range_count = 2;
  phy_sensor_config.range[0] = LPS22HX_PRESS_RANGE_MIN;
  phy_sensor_config.range[1] = LPS22HX_PRESS_RANGE_MAX;
  phy_sensor_config.has_stream_is_synchronous = true;
  phy_sensor_config.stream_is_synchronous = false;

  payload_args.buf = &phy_sensor_config;
  payload_args.buf_len = sizeof(phy_sensor_config);

  if(state->publish_sensors & LPS22HX_PRESS)
  {
    pb_send_event(instance,
        sns_std_sensor_physical_config_event_fields,
        &phy_sensor_config,
        sns_get_system_time(),
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
        &state->press_info.suid);

    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "send_config_event pressure published sr=%d", state->press_info.current_odr);
  }

  if(state->publish_sensors & LPS22HX_SENSOR_TEMP)
  {
    // Override above values with sensor temperature info
    phy_sensor_config.sample_rate = state->temp_info.current_odr;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = STM_LPS22HX_ACTIVE_CURRENT;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = LPS22HX_TEMP_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = LPS22HX_SENSOR_TEMPERATURE_RANGE_MIN;
    phy_sensor_config.range[1] = LPS22HX_SENSOR_TEMPERATURE_RANGE_MAX;

    payload_args.buf = &phy_sensor_config;
    payload_args.buf_len = sizeof(phy_sensor_config);

    pb_send_event(instance,
        sns_std_sensor_physical_config_event_fields,
        &phy_sensor_config,
        sns_get_system_time(),
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
        &state->temp_info.suid);

    DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__,
        "send_config_event temp sr=%d", state->temp_info.current_odr);
  }
}
