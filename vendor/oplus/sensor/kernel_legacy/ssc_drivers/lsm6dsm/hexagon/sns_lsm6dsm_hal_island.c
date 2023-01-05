/**
 * @file sns_lsm6dsm_hal.c
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
#include "sns_lsm6dsm_hal.h"
#include "sns_lsm6dsm_sensor.h"
#include "sns_lsm6dsm_sensor_instance.h"

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
#define DAE_SYNC_MIN_SAMPLES (16) //use sync com port only if samples is 16
#define LSM6DSM_MIN_HEART_BEAT_TIMEOUT_NS (20*1000*1000ULL)
#define LSM6DSM_HEART_BEAT_ODR_COUNT 5
#define AVG_NOMINAL_RATIO_SAMPLE_CNT 10
#define AVG_NOMINAL_RATIO_START_SAMPLE 6
/** Need to use ODR table. */
extern const odr_reg_map lsm6dsm_odr_map[];
extern const uint32_t lsm6dsm_odr_map_len;

/** LSM6DSM Accel Resolution */
const float lsm6dsm_accel_resolutions[] =
{
  LSM6DSM_ACCEL_RESOLUTION_2G,
  LSM6DSM_ACCEL_RESOLUTION_4G,
  LSM6DSM_ACCEL_RESOLUTION_8G,
  LSM6DSM_ACCEL_RESOLUTION_16G
}; //mg/LSB

/** LSM6DSM Gyro Resolution */
const float lsm6dsm_gyro_resolutions[] =
{
  LSM6DSM_GYRO_SSTVT_125DPS,
  LSM6DSM_GYRO_SSTVT_245DPS,
  LSM6DSM_GYRO_SSTVT_500DPS,
  LSM6DSM_GYRO_SSTVT_1000DPS,
  LSM6DSM_GYRO_SSTVT_2000DPS
};  //mdps/LSB

const lsm6dsm_gyro_range lsm6dsm_gyro_ranges[] =
{
  STM_LSM6DSM_GYRO_RANGE_125DPS,
  STM_LSM6DSM_GYRO_RANGE_245DPS,  /*corresponding value in register setting*/
  STM_LSM6DSM_GYRO_RANGE_500DPS,
  STM_LSM6DSM_GYRO_RANGE_1000DPS,
  STM_LSM6DSM_GYRO_RANGE_2000DPS
};

const lsm6dsm_accel_range lsm6dsm_accel_ranges[] =
{
  LSM6DSM_ACCEL_RANGE_2G,
  LSM6DSM_ACCEL_RANGE_4G,
  LSM6DSM_ACCEL_RANGE_8G,
  LSM6DSM_ACCEL_RANGE_16G
};

/** LSM6DSM Accel Range min */
const float lsm6dsm_accel_range_min[] =
{
  LSM6DSM_ACCEL_RANGE_2G_MIN,
  LSM6DSM_ACCEL_RANGE_4G_MIN,
  LSM6DSM_ACCEL_RANGE_8G_MIN,
  LSM6DSM_ACCEL_RANGE_16G_MIN
};  //mg/LSB

/** LSM6DSM Accel Range max */
const float lsm6dsm_accel_range_max[] =
{
  LSM6DSM_ACCEL_RANGE_2G_MAX,
  LSM6DSM_ACCEL_RANGE_4G_MAX,
  LSM6DSM_ACCEL_RANGE_8G_MAX,
  LSM6DSM_ACCEL_RANGE_16G_MAX
};  //mg/LSB

/** LSM6DSM Gyro Range min */
const float lsm6dsm_gyro_range_min[] =
{
  LSM6DSM_GYRO_RANGE_125_MIN,
  LSM6DSM_GYRO_RANGE_245_MIN,
  LSM6DSM_GYRO_RANGE_500_MIN,
  LSM6DSM_GYRO_RANGE_1000_MIN,
  LSM6DSM_GYRO_RANGE_2000_MIN
};  //mdps/LSB

/** LSM6DSM Gyro Range max */
const float lsm6dsm_gyro_range_max[] =
{
  LSM6DSM_GYRO_RANGE_125_MAX,
  LSM6DSM_GYRO_RANGE_245_MAX,
  LSM6DSM_GYRO_RANGE_500_MAX,
  LSM6DSM_GYRO_RANGE_1000_MAX,
  LSM6DSM_GYRO_RANGE_2000_MAX
};  //mdps/LSB

#define SLOPE_SETTLING_TIME_NS(min_odr) ((3*1000000000ULL)/min_odr)
#define HPF_SETTLING_TIME_NS(min_odr) ((15*1000000000ULL)/min_odr)
#define LSM6DSM_ODR_TOLERANCE (2.0f) //% tolerance
#define LSM6DSM_ODR_TOLERANCE_10 (10.0f) //% tolerance
#define LSM6DSM_IS_INBOUNDS(data, ref, var) \
  (((data) > ((ref) * (100.0f+(var)))/100.0f) || \
   ((data) < ((ref) * (100.0f-(var)))/100.0f)) ? (false) : (true)
#define WINDOW_SIZE (20)
#define LSM6DSM_MAX_MISSING_SAMPLES (32) //max_odr / min_odr


#if !LSM6DSM_LOGGING_DISABLED
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
sns_rc lsm6dsm_encode_sensor_state_log_interrupt(
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
sns_rc lsm6dsm_encode_log_sensor_state_raw_oneaxis(
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
  lsm6dsm_batch_sample_oneaxis *raw_sample = (lsm6dsm_batch_sample_oneaxis *)log;

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

    parsed_log_size += sizeof(lsm6dsm_batch_sample_oneaxis);
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
sns_rc lsm6dsm_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[LSM6DSM_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = LSM6DSM_NUM_AXES,
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
  lsm6dsm_batch_sample *raw_sample = (lsm6dsm_batch_sample *)log;

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

    parsed_log_size += sizeof(lsm6dsm_batch_sample);
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
 * @param[i] diag       Pointer to diag service
 * @param[i] log_size   Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void lsm6dsm_log_sensor_state_raw_alloc(
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
void lsm6dsm_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  uint8_t sample_size,
  bool batch_complete)
{
  lsm6dsm_batch_sample *sample = NULL;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == log_raw_info->log)
  {
    return;
  }

  sample = (lsm6dsm_batch_sample *)log_raw_info->log;

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
  if(sample_size == 1)
  {
  log_raw_info->diag->api->submit_log(
        log_raw_info->diag,
        log_raw_info->instance,
        log_raw_info->sensor_uid,
        log_raw_info->bytes_written,
        log_raw_info->log,
        SNS_DIAG_SENSOR_STATE_LOG_RAW,
        log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
        lsm6dsm_encode_log_sensor_state_raw_oneaxis);
  }
  else
  {
    log_raw_info->diag->api->submit_log(
          log_raw_info->diag,
          log_raw_info->instance,
          log_raw_info->sensor_uid,
          log_raw_info->bytes_written,
          log_raw_info->log,
          SNS_DIAG_SENSOR_STATE_LOG_RAW,
          log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
        lsm6dsm_encode_log_sensor_state_raw);
  }

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
sns_rc lsm6dsm_log_sensor_state_raw_add(
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

  uint8_t batch_sample_size = sizeof(lsm6dsm_batch_sample);
  if(sample_size == 1)
    batch_sample_size = sizeof(lsm6dsm_batch_sample_oneaxis);

  if( (log_raw_info->bytes_written + batch_sample_size) >
     log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    lsm6dsm_log_sensor_state_raw_submit(log_raw_info, sample_size, false);
    lsm6dsm_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    if(sample_size == 1) {
      lsm6dsm_batch_sample_oneaxis *sample_oneaxis =
        (lsm6dsm_batch_sample_oneaxis *)log_raw_info->log;
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
    }
    else {
    lsm6dsm_batch_sample *sample =
        (lsm6dsm_batch_sample *)log_raw_info->log;
    if(0 == log_raw_info->batch_sample_cnt)
    {
      sample[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
    }
    else
    {
      sample[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
    }

    sample[log_raw_info->log_sample_cnt].timestamp = timestamp;

    sns_memscpy(sample[log_raw_info->log_sample_cnt].sample,
                sizeof(sample[log_raw_info->log_sample_cnt].sample),
                raw_data,
                sizeof(sample[log_raw_info->log_sample_cnt].sample));

    sample[log_raw_info->log_sample_cnt].status = status;
    }

    log_raw_info->bytes_written += batch_sample_size;

    log_raw_info->log_sample_cnt++;
    log_raw_info->batch_sample_cnt++;
  }

  return rc;
}
#endif

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
sns_rc lsm6dsm_com_read_wrapper(
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
sns_rc lsm6dsm_com_write_wrapper(
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
  sns_rc rc;
  lsm6dsm_instance_state *inst_state =
    (lsm6dsm_instance_state*)instance->state->state;
  rc = inst_state->scp_service->api->sns_scp_register_rw(inst_state->com_port_info.port_handle,
                                                        &port_vec,
                                                        1,
                                                        save_write_time,
                                                        xfer_bytes);
  if(rc != SNS_RC_SUCCESS || *xfer_bytes != bytes)
  {
    SNS_INST_PRINTF(ERROR, instance, "write_wrapper: reg=0x%x rc=%d #bytes=%u",
                    reg_addr, rc, *xfer_bytes);
  }
#if LSM6DSM_DUMP_REG
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
sns_rc lsm6dsm_read_modify_write(
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

  sns_rc rc = SNS_RC_FAILED;
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  sns_sync_com_port_handle* port_handle = state->com_port_info.port_handle;

  if((size > 1) || (mask == 0xFF) || (mask == 0x00))
  {
    rc = lsm6dsm_com_write_wrapper(instance,
                          reg_addr,
                          &reg_value[0],
                          size,
                          xfer_bytes,
                          save_write_time);

  }
  else
  {
    // read current value from this register
    rc = lsm6dsm_com_read_wrapper(state->scp_service,
                                  port_handle,
                                  reg_addr,
                                  &rw_buffer,
                                  1,
                                  &rw_bytes);
    if (SNS_RC_SUCCESS == rc)
    {
      // generate new value
      rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

      // write new value to this register
      rc = lsm6dsm_com_write_wrapper(instance,
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

#if LSM6DSM_DUMP_REG
  DBG_INST_PRINTF(HIGH, instance, "reg write 0x%x=0x%x mask=0x%x",
                  reg_addr, *reg_value, mask);
#endif
  return rc;
}

/**
 * see sns_lsm6dsm_hal.h
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
sns_rc lsm6dsm_read_regs_scp(sns_sensor_instance *const instance,
                             uint8_t addr, uint16_t num_of_bytes, uint8_t *buffer)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  rc = lsm6dsm_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
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

sns_rc lsm6dsm_write_regs_scp(sns_sensor_instance *const instance,
                              uint8_t addr, uint16_t num_of_bytes, uint8_t *buffer)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rc = lsm6dsm_com_write_wrapper(instance,
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


/**
 * see sns_lsm6dsm_hal.h
 */
sns_rc lsm6dsm_device_sw_reset(
    sns_sensor_instance *const instance,
    lsm6dsm_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t buffer = 0x01;
  int8_t num_attempts = 5;
  sns_rc rc = SNS_RC_FAILED;

  DBG_INST_PRINTF_EX(LOW, instance, "sw_rst");

  while(num_attempts-- > 0 && SNS_RC_SUCCESS != rc)
  {
    rc = lsm6dsm_write_regs_scp(instance, STM_LSM6DSM_REG_CTRL3, 1, &buffer);
    if(SNS_RC_SUCCESS != rc)
    {
      DBG_INST_PRINTF(ERROR, instance, "sw_rst: failed; wait and try again");
      sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
    }
  }

  if(num_attempts <= 0)
  {
    DBG_INST_PRINTF(ERROR, instance, "sw_rst: failed all attempts");
  }

  num_attempts = 10;
  do
  {
    if(num_attempts-- <= 0)
    {
      DBG_INST_PRINTF(LOW, instance, "sw_rst: failed due to timeout- attempts:%d", 10-num_attempts);
      // Sensor HW has not recovered from SW reset.
      return SNS_RC_FAILED;
    }
    else
    {
      //0.1ms wait
      sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
      lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_CTRL3, 1, &buffer);
    }

  } while((buffer & 0x01));

  DBG_INST_PRINTF_EX(HIGH, instance, "sw_rst success! attempts %d", 10-num_attempts);
  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6dsm_hal.h
 */
static sns_rc lsm6dsm_device_set_default_state(
    sns_sensor_instance *const instance,
    lsm6dsm_sensor_type sensor)
{
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  UNUSED_VAR(state);
  DBG_INST_PRINTF_EX(LOW, instance, "set_default_state: sensor=0x%x", sensor);

  if(sensor == LSM6DSM_ACCEL)
  {
    // reset Accel state only
  }
  else if(sensor == LSM6DSM_GYRO)
  {
    // reset Gyro state only
  }
  else if(sensor == (LSM6DSM_ACCEL | LSM6DSM_GYRO | LSM6DSM_MOTION_DETECT | LSM6DSM_SENSOR_TEMP))
  {

     // Configure control register 3
     buffer[0] = 0x0
       | (0<<7)           // BOOT bit
       | (1<<6)           // BDU bit
       | (0<<5)           // H_LACTIVE bit
       | (0<<4)           // PP_OD bit
       | (0<<3)           // SIM bit
       | (1<<2)           // IF_INC
       | (0<<1)           // BLE
       | 0;               // SW_RESET

     rv = lsm6dsm_read_modify_write(instance,
                            STM_LSM6DSM_REG_CTRL3,
                            &buffer[0],
                            1,
                            &xfer_bytes,
                            false,
                            0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     //workaround enable HPF for XL here
     //initialize with high threshold
     buffer[0] = 0x3F;
     rv = lsm6dsm_read_modify_write(instance,
                         STM_LSM6DSM_REG_WAKE_THS,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0x3F);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     buffer[0] = 0
       | (0<<7)            // LPF2_XL_EN: refer to figure 5.
       | (0<<5)            // HPCF_XL[1:0]: 0-SlopeFilter=ODR/50; 1-HP Filter=ODR/100; 2-HP Filter=ODR/9; 3-HP Filter=ODR/400.
       | (0<<2)            // HP_SLOPE_XL_EN:
       | 0;                // LOW_PASS_ON_6D:
     rv = lsm6dsm_read_modify_write(instance,
                         STM_LSM6DSM_REG_CTRL8_XL,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

      buffer[0] = 0
        | (0<<7)            // DEN_X
        | (0<<6)            // DEN_Y
        | (0<<5)            // DEN_Z
        | (0<<4)            // DEN_XL_G
        | (0<<3)            // DEN_XL_GEN
        | (0<<2)            // DEN_LH
        | (0<<1)            // I3C_disable
        | 0;                // Force enable INT when I3C is active
     rv = lsm6dsm_read_modify_write(instance,
                         STM_LSM6DSM_REG_CTRL9_A,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     //Enable interrupt for MD
     rv = lsm6dsm_set_interrupts(instance);
     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }
#if LSM6DSM_SENSOR_OEM_CONFIG
     lsm6dsm_oem_set_default_config(instance);
#endif

     lsm6dsm_set_accel_config(instance,
                            LSM6DSM_ACCEL_ODR_OFF,
                            state->accel_info.sstvt,
                            state->accel_info.range,
                            LSM6DSM_ACCEL_BW50);

     lsm6dsm_set_gyro_config(instance,
                             LSM6DSM_GYRO_ODR_OFF,
                             state->gyro_info.sstvt,
                             state->gyro_info.range);
  }

  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6dsm_hal.h
 */
sns_rc lsm6dsm_reset_device(
    sns_sensor_instance *const instance,
    lsm6dsm_sensor_type sensor)
{
  sns_rc rv = SNS_RC_SUCCESS;

  DBG_INST_PRINTF_EX(HIGH, instance, "reset_device");
  /** HW reset only when both Accel and Gyro are requested for
   *  reset. */
  if( sensor == (LSM6DSM_ACCEL | LSM6DSM_GYRO | LSM6DSM_MOTION_DETECT | LSM6DSM_SENSOR_TEMP))
  {
     rv = lsm6dsm_device_sw_reset(instance, sensor);
  }

  if(rv == SNS_RC_SUCCESS)
  {
    rv = lsm6dsm_device_set_default_state(instance, sensor);
  }

  if(rv != SNS_RC_SUCCESS)
  {
    DBG_INST_PRINTF(ERROR, instance, "reset_device failed!");
  }
  return rv;
}


/**
 * see sns_lsm6dsm_hal.h
 */
sns_rc lsm6dsm_set_gyro_config(
    sns_sensor_instance *const instance,
    lsm6dsm_gyro_odr         curr_odr,
    lsm6dsm_gyro_sstvt       sstvt,
    lsm6dsm_gyro_range       range)
{
  UNUSED_VAR(sstvt);
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  uint8_t buffer;

  if(LSM6DSM_IS_ESP_ENABLED(state) && curr_odr != LSM6DSM_GYRO_ODR_OFF)
  {
    lsm6dsm_gyro_odr esp_odr = (lsm6dsm_gyro_odr)lsm6dsm_get_esp_rate_idx(instance);
    curr_odr = (curr_odr < esp_odr) ? esp_odr : curr_odr;
  }

  buffer = (uint8_t)curr_odr | (uint8_t)range;

  if(state->common_info.gyro_curr_odr != curr_odr)
  {
    state->common_info.odr_changed |= (state->config_sensors & LSM6DSM_GYRO);
    state->gyro_info.num_samples_to_discard = lsm6dsm_odr_map[state->desired_conf.odr_idx].gyro_discard_samples;
    if(!state->common_info.gyro_curr_odr)
      state->gyro_info.num_samples_to_discard += (lsm6dsm_odr_map[state->desired_conf.odr_idx].odr * LSM6DSM_GYRO_ON_TIME_MS )/1000;
    state->common_info.gyro_odr_settime = sns_get_system_time();
  }
  state->common_info.gyro_curr_odr = curr_odr;
  if(!state->self_test_info.test_alive)
  {
    state->gyro_info.sstvt = sstvt;
  }

  DBG_INST_PRINTF(LOW, instance, "gyro_config: odr=0x%x, CTRL2_G=0x%x, settime=%u",
                  curr_odr, buffer, (uint32_t)state->common_info.gyro_odr_settime);

  return lsm6dsm_write_regs_scp(instance, STM_LSM6DSM_REG_CTRL2_G, 1, &buffer);
}

/**
 * see sns_lsm6dsm_hal.h
 */
sns_rc lsm6dsm_set_accel_config(
   sns_sensor_instance *const instance,
                              lsm6dsm_accel_odr       curr_odr,
                              lsm6dsm_accel_sstvt     sstvt,
                              lsm6dsm_accel_range     range,
                              lsm6dsm_accel_bw        bw)
{
  UNUSED_VAR(sstvt);
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  uint8_t buffer;

  if(LSM6DSM_IS_ESP_ENABLED(state))
  {
    lsm6dsm_accel_odr esp_odr = (lsm6dsm_accel_odr)lsm6dsm_get_esp_rate_idx(instance);
    curr_odr = (curr_odr < esp_odr) ? esp_odr : curr_odr;
  }

  buffer = (uint8_t)curr_odr | (uint8_t)range | (uint8_t)bw;

  if(state->common_info.accel_curr_odr != curr_odr)
  {
    state->common_info.odr_changed |= (state->config_sensors & LSM6DSM_ACCEL);
    state->accel_info.num_samples_to_discard = lsm6dsm_odr_map[state->desired_conf.odr_idx].accel_discard_samples;
    state->fifo_info.new_config.timestamp = sns_get_system_time();
    state->common_info.accel_odr_settime = sns_get_system_time();
  }
  state->common_info.accel_curr_odr = curr_odr;
  state->current_conf.odr_idx = state->desired_conf.odr_idx;
  state->current_conf.odr = lsm6dsm_odr_map[state->desired_conf.odr_idx].odr;
  if(!state->self_test_info.test_alive)
  {
    state->accel_info.sstvt = sstvt;
  }

  DBG_INST_PRINTF(LOW, instance, "accel_config: odr=0x%x, CTRL1_XL=0x%x, settime=%u",
                  curr_odr, buffer, (uint32_t)state->common_info.accel_odr_settime);

  return lsm6dsm_write_regs_scp(instance, STM_LSM6DSM_REG_CTRL1_A, 1, &buffer);
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
sns_rc lsm6dsm_set_gyro_sleep(
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
  lsm6dsm_read_modify_write(instance,
                            STM_LSM6DSM_REG_CTRL4,
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
sns_rc lsm6dsm_set_acc_lpmode(
    sns_sensor_instance *const instance,
    bool set_lp)
{
  uint8_t buffer = (set_lp) ? 0x10 : 0x0;
  uint32_t xfer_bytes;

  DBG_INST_PRINTF(HIGH, instance, "set_lp=%d", set_lp);
  // Update Gyro Sleep state
  lsm6dsm_read_modify_write(instance,
                            STM_LSM6DSM_REG_CTRL6_G,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x10);
  return SNS_RC_SUCCESS;
}

void lsm6dsm_powerdown_gyro(sns_sensor_instance *this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;

  DBG_INST_PRINTF_EX(HIGH, this, "power down gyro");
  if(state->common_info.gyro_curr_odr) {
    lsm6dsm_set_gyro_config(this,
        state->gyro_info.desired_odr,
        state->gyro_info.sstvt,
        state->gyro_info.range);
  }

  if(state->gyro_info.is_in_sleep) {
    lsm6dsm_set_gyro_sleep(this, false);
    state->gyro_info.is_in_sleep = false;
  }
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_stop_fifo_streaming(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if(state->fifo_info.fifo_rate > LSM6DSM_ACCEL_ODR_OFF) {
    lsm6dsm_set_fifo_bypass_mode(instance);
    state->fifo_info.fifo_rate = LSM6DSM_ACCEL_ODR_OFF;
    state->current_conf.fifo_odr = 0;
  }

  if(state->common_info.accel_curr_odr > LSM6DSM_ACCEL_ODR_OFF)
  {
    lsm6dsm_set_accel_config(instance,
        LSM6DSM_ACCEL_ODR_OFF,
        state->accel_info.sstvt,
        state->accel_info.range,
        state->accel_info.bw);
  }

  lsm6dsm_powerdown_gyro(instance);
  if(state->common_info.gyro_curr_odr > LSM6DSM_GYRO_ODR_OFF)
  {
    lsm6dsm_set_gyro_sleep(instance, true);
    state->gyro_info.is_in_sleep = true;
  }
  lsm6dsm_set_acc_lpmode(instance, true);

  state->fifo_info.is_streaming = false;
  DBG_INST_PRINTF(MED, instance, "stop_fifo_streaming: odr a/g 0x%x/0x%x",
                  state->common_info.accel_curr_odr,state->common_info.gyro_curr_odr);
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_set_fifo_config(sns_sensor_instance *const instance,
                             uint16_t desired_wmk,
                             lsm6dsm_accel_odr a_chosen_sample_rate,
                             lsm6dsm_gyro_odr g_chosen_sample_rate,
                             lsm6dsm_sensor_type sensor)
{
  UNUSED_VAR(g_chosen_sample_rate);
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  DBG_INST_PRINTF(MED, instance, "set_fifo_config: sensor=0x%x sr=0x%x wm=%u",
                  sensor, a_chosen_sample_rate, desired_wmk);

  state->fifo_info.desired_wmk = desired_wmk;
  state->fifo_info.desired_fifo_rate = a_chosen_sample_rate;
  state->current_conf.fifo_odr = lsm6dsm_odr_map[state->desired_conf.odr_idx].odr;

  if(sensor & LSM6DSM_ACCEL || sensor & LSM6DSM_MOTION_DETECT)
  {
    state->accel_info.desired_odr = a_chosen_sample_rate;
    if(state->accel_info.desired_odr < LSM6DSM_ACCEL_ODR1664)
    {
      // BW configuration bit to be unset for all ODRs till 832Hz
      state->accel_info.bw = LSM6DSM_ACCEL_BW50;
    }
    else
    {
      //BW configuration bit to be set for 1.6Khz and greater ODRs
      state->accel_info.bw = LSM6DSM_ACCEL_BW1600;
    }
  } else
    state->accel_info.desired_odr = LSM6DSM_ACCEL_ODR_OFF;

  if(sensor & LSM6DSM_GYRO)
  {
    //if gyro is enabled set acc desired rate as gyro
    state->accel_info.desired_odr = a_chosen_sample_rate;
    state->gyro_info.desired_odr = g_chosen_sample_rate;
  } else
    state->gyro_info.desired_odr = LSM6DSM_GYRO_ODR_OFF;
}

void lsm6dsm_update_heartbeat_monitor(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if((state->fifo_info.publish_sensors & (LSM6DSM_ACCEL | LSM6DSM_GYRO)) ||
     (state->accel_info.gated_client_present && !state->current_conf.md_enabled))
  {
    uint64_t sampling_intvl = sns_convert_ns_to_ticks(1000000000.0f / state->desired_conf.odr);
    uint32_t wm = SNS_MAX(1, state->current_conf.wmk);
    if(lsm6dsm_dae_if_available(instance))
    {
      state->fifo_info.nominal_dae_intvl =
        state->fifo_info.max_requested_wmk * sampling_intvl * 1.1f;
      wm = (state->fifo_info.max_requested_flush_ticks == 0) ?
        UINT32_MAX : state->fifo_info.max_requested_wmk;
      DBG_INST_PRINTF(LOW, instance, "update_hb_mon:: dae_intvl=%u",
                      state->fifo_info.nominal_dae_intvl);
    }


    if(wm <= LSM6DSM_MAX_FIFO)
    {
      uint64_t min_hb_to = sns_convert_ns_to_ticks(LSM6DSM_MIN_HEART_BEAT_TIMEOUT_NS);
      uint64_t max_hb_to = sampling_intvl * LSM6DSM_HW_MAX_FIFO;
      state->health.heart_beat_timeout = sampling_intvl * wm * LSM6DSM_HEART_BEAT_ODR_COUNT;

      // avoid sending timer request too frequently
      state->health.heart_beat_timeout = SNS_MAX(state->health.heart_beat_timeout, min_hb_to);
      // limit to one full FIFO to avoid large data gap
      state->health.heart_beat_timeout = SNS_MIN(state->health.heart_beat_timeout, max_hb_to);
    }
    else
    {
      state->health.heart_beat_timeout = sampling_intvl * LSM6DSM_HEART_BEAT_ODR_COUNT * wm;
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
  lsm6dsm_restart_hb_timer(instance, false);
}

static void lsm6dsm_get_nominal_sampling_interval(sns_sensor_instance *const instance, float odr)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  if(state->common_info.gyro_curr_odr == LSM6DSM_GYRO_ODR_OFF)
  {
    state->fifo_info.nominal_sampling_intvl = (odr != 0) ?
      sns_convert_ns_to_ticks(1000000000.0f / odr) * state->fifo_info.avg_to_nominal_ratio:
      0;
  }
  else
  {
    state->fifo_info.nominal_sampling_intvl = (odr != 0) ?
      sns_convert_ns_to_ticks(1000000000.0f / odr) * state->fifo_info.avg_to_nominal_ratio_g:
      0;
  }

}
static void lsm6dsm_interrupt_interval_init_nominal(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  uint32_t interrupt_intvl_threshold;
  //reset only if odr is changed or gyro was added
  //if odr is same as before with same sensor, no need to recalculate sampling interval
  lsm6dsm_get_nominal_sampling_interval(instance, state->desired_conf.odr);

  state->fifo_info.avg_interrupt_intvl =
    state->fifo_info.nominal_sampling_intvl * state->desired_conf.wmk;

  // threshold is +- LSM6DSM_ODR_TOLERANCE% of nominal sampling intvl
  interrupt_intvl_threshold =
    state->fifo_info.avg_interrupt_intvl * LSM6DSM_ODR_TOLERANCE / 100;
  state->fifo_info.interrupt_intvl_upper_bound =
    state->fifo_info.avg_interrupt_intvl + interrupt_intvl_threshold;
  state->fifo_info.interrupt_intvl_lower_bound =
    state->fifo_info.avg_interrupt_intvl - interrupt_intvl_threshold;
  DBG_INST_PRINTF(
    LOW, instance, "s_intvl=%u s_bounds=%u/%u",
    state->fifo_info.avg_interrupt_intvl,
    state->fifo_info.interrupt_intvl_lower_bound,
    state->fifo_info.interrupt_intvl_upper_bound);
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_set_fifo_wmk(sns_sensor_instance *const instance)
{
  uint16_t wmk_words = 0;
  uint8_t wmkL = 0;
  uint8_t wmkH = 0;
  uint32_t xfer_bytes;
  uint8_t buffer;
  uint8_t decimation = 0;
  uint16_t set_wmk = 0;
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;

  DBG_INST_PRINTF(HIGH, instance,
                 "set_fifo_wmk: cur=%u des=%u",
                 state->fifo_info.cur_wmk, state->fifo_info.desired_wmk);

  state->fifo_info.cur_wmk = state->fifo_info.desired_wmk;
  state->fifo_info.new_config.fifo_watermark = state->fifo_info.desired_wmk;
  if(state->fifo_info.cur_wmk != state->fifo_info.last_sent_config.fifo_watermark)
  {
    state->fifo_info.new_config.timestamp = sns_get_system_time();
  }

#if LSM6DSM_DAE_ENABLED
  state->fifo_info.new_config.dae_watermark  = state->fifo_info.max_requested_wmk;
#endif

  // QC: How many things are current at the same time?
  //There's a fifo_info current watermark, and a current config watermark. Which one is more current?
  state->current_conf.wmk = state->fifo_info.desired_wmk;

  set_wmk = state->current_conf.wmk;

  //convert samples to words
  if(((state->accel_info.desired_odr > LSM6DSM_ACCEL_ODR_OFF) &&
      (state->fifo_info.fifo_enabled & LSM6DSM_ACCEL)) ||
     ((state->self_test_info.sensor == LSM6DSM_ACCEL)&&(state->self_test_info.test_alive)))
  {
    wmk_words = state->fifo_info.cur_wmk * 3;
    decimation |= 1;
  }

  if(((state->gyro_info.desired_odr > LSM6DSM_GYRO_ODR_OFF) &&
      (state->fifo_info.fifo_enabled & LSM6DSM_GYRO)) ||
     ((state->self_test_info.sensor == LSM6DSM_GYRO)&&(state->self_test_info.test_alive)))
  {
    wmk_words += state->fifo_info.cur_wmk * 3;
    if((state->accel_info.desired_odr) && (!state->gyro_info.desired_odr) && (state->self_test_info.self_test_stage == LSM6DSM_SELF_TEST_STAGE_1))
    {
      wmk_words += state->fifo_info.cur_wmk * 2 * 3;
    }
    decimation |= 0x8;
  }

  // Set Accel decimation to no decimation
  buffer = decimation;
  lsm6dsm_read_modify_write(instance,
                        STM_LSM6DSM_REG_FIFO_CTRL3,
                        &buffer,
                        1,
                        &xfer_bytes,
                        false,
                        0xFF);
  // Configure FIFO WM
  wmkL = wmk_words & STM_LSM6DSM_FIFO_WTM_CTRL1_MASK;
  lsm6dsm_read_modify_write(instance,
                        STM_LSM6DSM_REG_FIFO_CTRL1,
                        &wmkL,
                        1,
                        &xfer_bytes,
                        false,
                        STM_LSM6DSM_FIFO_WTM_CTRL1_MASK);

  wmkH = (wmk_words >> 8) & STM_LSM6DSM_FIFO_WTM_CTRL2_MASK;
  lsm6dsm_read_modify_write(instance,
                        STM_LSM6DSM_REG_FIFO_CTRL2,
                        &wmkH,
                        1,
                        &xfer_bytes,
                        false,
                        STM_LSM6DSM_FIFO_WTM_CTRL2_MASK);

  //update interrupt interval and bounds since wmk is changing
  lsm6dsm_interrupt_interval_init_nominal(instance);

  lsm6dsm_update_heartbeat_monitor(instance);
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_start_fifo_streaming(sns_sensor_instance *const instance)
{
  // Enable FIFO Streaming
  // Enable Accel Streaming
  // Enable GYRO Streaming

  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  bool is_md_req;
  bool is_md_force_disabled;
  bool init_nominal = false;
  bool accel_boot = false;
  DBG_INST_PRINTF(HIGH, instance, "start_fifo_streaming: rate=0x%x last_ts=%u",
                  state->fifo_info.desired_fifo_rate,
                  (uint32_t)state->fifo_info.last_timestamp);

  if((state->fifo_info.fifo_rate == LSM6DSM_ACCEL_ODR_OFF) || (!state->fifo_info.is_streaming)){
    DBG_INST_PRINTF(LOW, instance, "start_fifo_streaming: resetting last_timestamp");
    accel_boot = true;
    state->fifo_info.last_ts_valid = false; //set last ts as inaccurate
  }

  if(state->current_conf.odr != state->desired_conf.odr ||
     state->fifo_info.fifo_rate == LSM6DSM_ACCEL_ODR_OFF ||
     (state->desired_conf.enabled_sensors != state->current_conf.enabled_sensors)) {
    //reset only if odr is changed or if fifo is starting or if gyro is starting/stopping
    //if odr is same as before, no need to recalculate sampling interval
    state->fifo_info.interrupt_cnt = 0;
    init_nominal = true;

    if((state->fifo_info.fifo_rate == LSM6DSM_ACCEL_ODR_OFF)
       && (state->common_info.accel_curr_odr != LSM6DSM_ACCEL_ODR_OFF)) {
      DBG_INST_PRINTF(MED, instance, "start_fifo_streaming: turning off accel");
      //accel is on but fifo is off, turn off accel
      lsm6dsm_set_accel_config(instance,
          LSM6DSM_ACCEL_ODR_OFF,
          state->accel_info.sstvt,
          state->accel_info.range,
          LSM6DSM_ACCEL_BW50);
    }
  }
  state->fifo_info.fifo_rate = state->fifo_info.desired_fifo_rate;

  DBG_INST_PRINTF(
    LOW, instance, "cur=%u des=%u fifo=%x common=%u",
    state->current_conf.odr, state->desired_conf.odr,
    state->fifo_info.fifo_rate, state->common_info.accel_curr_odr);

  is_md_req = lsm6dsm_is_md_int_required(instance);
  is_md_force_disabled = false;
  DBG_INST_PRINTF(HIGH, instance, "start_fifo_streaming: md_enabled/md_req = %d, %d",
      state->current_conf.md_enabled, is_md_req);
  //disable md if enabled
  //Avoiding spurious interrupts
  if(state->current_conf.md_enabled) {
    lsm6dsm_disable_md(instance, !is_md_req);
    is_md_force_disabled = true;
  }

  lsm6dsm_set_accel_config(instance,
      state->accel_info.desired_odr,
      state->accel_info.sstvt,
      state->accel_info.range,
      state->accel_info.bw);

  lsm6dsm_set_gyro_config(instance,
      state->gyro_info.desired_odr,
      state->gyro_info.sstvt,
      state->gyro_info.range);

  if(init_nominal)
  {
    //update interrupt interval and bounds
    lsm6dsm_interrupt_interval_init_nominal(instance);

    // Update sampling interval
    state->fifo_info.avg_sampling_intvl = state->fifo_info.nominal_sampling_intvl;
  }

  //start streaming,stream mode
  lsm6dsm_set_fifo_stream_mode(instance);

  if(state->gyro_info.is_in_sleep)
  {
    lsm6dsm_set_gyro_sleep(instance, false);
    state->gyro_info.is_in_sleep = false;
  }
  if(state->ag_stream_mode != DRI) {
    lsm6dsm_run_polling_timer(instance);
  }

  //re-enable if force disabled before and required
  if(is_md_force_disabled && is_md_req) {
    lsm6dsm_enable_md(instance, false);
  }

  if(state->fifo_info.publish_sensors & LSM6DSM_ACCEL) {
    lsm6dsm_set_acc_lpmode(instance, false);
  }
  state->fifo_info.is_streaming =
    ((state->fifo_info.publish_sensors & (LSM6DSM_ACCEL | LSM6DSM_GYRO)) &&
     state->fifo_info.max_requested_flush_ticks);
  state->config_sensors &= ~(LSM6DSM_ACCEL | LSM6DSM_GYRO);

  state->current_conf.enabled_sensors =
    state->fifo_info.fifo_enabled & (LSM6DSM_ACCEL | LSM6DSM_GYRO);

  if(accel_boot)
    state->fifo_info.last_timestamp = sns_get_system_time();
  DBG_INST_PRINTF(MED, instance,
                  "start_fifo_streaming EX: a_odr=0x%x g_odr=0x%x last_ts=%u",
                  state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr,(uint32_t)state->fifo_info.last_timestamp);

}

void lsm6dsm_run_polling_timer(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  sns_time report_period; //in ticks
  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time(); // QC - should be system_time() - report_period.
  req_payload.timeout_period =  report_period =
    sns_convert_ns_to_ticks(1000000000.0 / state->current_conf.odr) * state->current_conf.wmk;
  req_payload.priority = SNS_TIMER_PRIORITY_POLLING;
  req_payload.is_dry_run = lsm6dsm_dae_if_available(instance); // if dae available, set dry run
  DBG_INST_PRINTF(HIGH, instance, "run_polling_timer:: timeout_period =%u report_period = %u",
      (uint32_t)req_payload.timeout_period, (uint32_t)report_period);

  state->poll_timeout = req_payload.timeout_period;
  lsm6dsm_inst_create_timer(instance, &state->timer_polling_data_stream, &req_payload);
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_enable_fifo_intr(sns_sensor_instance *const instance,
                              lsm6dsm_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  DBG_INST_PRINTF(HIGH, instance,
      "enable_fifo_intr:: sensor=0x%x INT1_CTRL=0x%x ag_stream_mode %d",
      sensor, state->fifo_info.int1_ctrl_reg, state->ag_stream_mode);
  if(state->fifo_info.fifo_enabled && state->fifo_info.int1_ctrl_reg == 0 &&
     state->ag_stream_mode == DRI)
  {
    if(state->fifo_info.fifo_enabled & (LSM6DSM_ACCEL | LSM6DSM_GYRO) &&
        (state->fifo_info.cur_wmk > 0) &&
        (state->irq_ready || lsm6dsm_dae_if_available(instance))) {
          lsm6dsm_clear_interrupt_q(instance, state->interrupt_data_stream);
          // Configure lsm6dsm FIFO control registers
          rw_buffer = 0x0
            | (0<<7)       // DEN_DRDY
            | (0<<6)       // INT1 CNT_BDR
            | (0<<5)       // INT1 FSS5
            | (1<<4)       // INT1 FIFO_OVR flag
            | (1<<3)       // INT1 FIFO_FTH flag
            | (0<<2)       // INT1 BOOT flag
            | (0<<1)       // INT1 DRDY_G flag
            | 0;           // INT1 DRDY_XL flag
           rv = lsm6dsm_read_modify_write(instance,
                                    STM_LSM6DSM_REG_INT1_CTRL,
                                    &rw_buffer,
                                    1,
                                    &xfer_bytes,
                                    false,
                                    STM_LSM6DSM_FIFO_INT_MASK); 
           if(rv == SNS_RC_SUCCESS)
             state->fifo_info.int1_ctrl_reg = 1;
          DBG_INST_PRINTF_EX(LOW, instance, "INT1_CTRL=0x%x",rw_buffer);
    }
  } else {
    DBG_INST_PRINTF(LOW, instance, "Not enabling FIFO interrupt stream_mode = %d",
                    state->ag_stream_mode);
    }
}

void lsm6dsm_disable_fifo_intr(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  uint32_t xfer_bytes;
  uint8_t rw_buffer = 0;

  DBG_INST_PRINTF_EX(HIGH, instance, "disable_fifo_intr");
  state->fifo_info.int1_ctrl_reg = 0;
  lsm6dsm_read_modify_write(instance,
                            STM_LSM6DSM_REG_INT1_CTRL,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            STM_LSM6DSM_FIFO_INT_MASK);
}

//return no of bytes in fifo
sns_rc lsm6dsm_get_fifo_status(
    sns_sensor_instance *const instance,
    uint16_t* bytes_in_fifo,
    uint8_t* status_reg)
{
  sns_rc rc = SNS_RC_SUCCESS;
  //read fifo regs
  rc = lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_FIFO_STATUS1, 4, status_reg);

#if LSM6DSM_DEBUG_CRASH
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;

  state->debug_status_reg[0] = status_reg[0];
  state->debug_status_reg[1] = status_reg[1];
  state->debug_status_reg[2] = status_reg[2];
  state->debug_status_reg[3] = status_reg[3];
  SNS_INST_PRINTF(HIGH, instance,
    "LSM6DSM_DEBUG_CRASH: status_reg 0x%x 0x%x 0x%x 0x%x",state->debug_status_reg[0], state->debug_status_reg[1], state->debug_status_reg[2], state->debug_status_reg[3]);
#endif
  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance,
        "read_fifo_status fail %d", rc);
    return rc;
  }
  // Calculate the number of bytes to be read
  uint16_t countH = status_reg[1] & 0x07;
  uint16_t countL = status_reg[0] & 0xFF;
  *bytes_in_fifo =  (((countH << 8) & 0xFF00) | countL) * 2;
  if(status_reg[1] & 0x10) { //Check empty
      *bytes_in_fifo = 0;
    }
  if((!*bytes_in_fifo) && (status_reg[1] & 0x40)) {
    *bytes_in_fifo = 4096;
  }
  LSM6DSM_INST_DEBUG_TS(LOW, instance,
    "status_reg 0x%x 0x%x 0x%x 0x%x",status_reg[0], status_reg[1], status_reg[2], status_reg[3]);
  LSM6DSM_INST_DEBUG_TS(LOW, instance,
    "count 0x%x num_of_bytes %d pattern %d",(countH << 8) | countL, *bytes_in_fifo, ((status_reg[3] & 0x03) << 8) | (status_reg[2] & 0xFF));
  return rc;
}

// QC - should keep a state variable for TAP_CFG2 and TAP_CFG and do 1 bus operation instead of 4
sns_rc lsm6dsm_set_interrupts(sns_sensor_instance *const instance)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  if(!state->int_enabled) {
    rw_buffer = 0x81;
    rv = lsm6dsm_read_modify_write(instance,
        STM_LSM6DSM_REG_TAP_CFG,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x81);
    state->int_enabled = true;
  }
  return rv;
}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_flush_fifo(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) {
    LSM6DSM_INST_DEBUG_TS(HIGH, this, "resetting int_cnt");
    state->fifo_info.interrupt_cnt = 0;
  }
  //Ignore any new flush commands in DAE mode if flush already in progress
  if((state->fifo_info.th_info.flush_req) && (lsm6dsm_dae_if_available(this)))
    return;

  state->fifo_info.th_info.flush_req = true;
  if (!lsm6dsm_dae_if_flush_hw(this))
  {
    lsm6dsm_read_fifo_data(this, sns_get_system_time(), true);
  }
}

/**
 * see sns_lsm6dsm_hal.h
 */
sns_rc lsm6dsm_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = lsm6dsm_com_read_wrapper(scp_service,
                                port_handle,
                                STM_LSM6DSM_REG_WHO_AM_I,
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
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
void lsm6dsm_set_polling_config(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;

  DBG_INST_PRINTF(
    MED, this, "set_polling_config: s_intvl=%u sr(*1000)=%d",
    (uint32_t)state->sensor_temp_info.sampling_intvl,
    (int)(state->sensor_temp_info.desired_sampling_rate_hz*1000));

  if(state->sensor_temp_info.sampling_intvl > 0)
  {
    lsm6dsm_start_sensor_temp_polling_timer(this);
  }
  else
  {
    state->sensor_temp_info.timer_is_active = false;
    state->sensor_temp_info.cur_sampling_rate_hz = 0;
  }
}

sns_rc lsm6dsm_recover_device(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  struct group_read {
    uint32_t first_reg;
    uint8_t  num_regs;
  } groups[] = { /* must fit within state->reg_status[] */
    { STM_LSM6DSM_REG_FIFO_CTRL1, 5 },
    { STM_LSM6DSM_REG_INT1_CTRL, 13 },
    { STM_LSM6DSM_REG_TAP_CFG, 8 }
  };

  //Save Context
  {
    uint8_t *dest = state->reg_status;

    for(uint32_t i=0; i<ARR_SIZE(groups); i++)
    {
      lsm6dsm_read_regs_scp(this, groups[i].first_reg, groups[i].num_regs, dest);
#if 1 //LSM6DSM_DUMP_REG
      for(uint32_t j=0; j<groups[i].num_regs; j++)
      {
        DBG_INST_PRINTF(LOW, this, "dump: 0x%02x=0x%02x",
                        groups[i].first_reg+j, dest[j]);
      }
#endif
      dest += groups[i].num_regs;
    }
    DBG_INST_PRINTF(MED, this, "Context saved");
  }

  //Gyro set power down mode
  lsm6dsm_set_gyro_config(this,
                          LSM6DSM_GYRO_ODR_OFF,
                          state->gyro_info.sstvt,
                          state->gyro_info.range);

  //Set Accel in High Performance mode
  lsm6dsm_set_acc_lpmode(this, false);

  // Reset Sensor
  rv = lsm6dsm_reset_device(this,
      LSM6DSM_ACCEL | LSM6DSM_GYRO | LSM6DSM_MOTION_DETECT | LSM6DSM_SENSOR_TEMP);
#if LSM6DSM_ESP_ENABLED
  lsm6dsm_esp_handle_reset_device(this, NULL);
#endif

  //Power up Accel if needed. It was powered down during reset_device
  lsm6dsm_set_accel_config(this,
      state->accel_info.desired_odr,
      state->accel_info.sstvt,
      state->accel_info.range,
      state->accel_info.bw);

  //Power up gyro if needed
  lsm6dsm_set_gyro_config(this,
      state->gyro_info.desired_odr,
      state->gyro_info.sstvt,
      state->gyro_info.range);

  //Restore context
  {
    uint8_t *src = state->reg_status;

    for(uint32_t i=0; i<ARR_SIZE(groups); i++)
    {
      lsm6dsm_write_regs_scp(this, groups[i].first_reg, groups[i].num_regs, src);
      src += groups[i].num_regs;
    }
    DBG_INST_PRINTF(MED, this, "Context restored");
  }

  //Resrt flags
  state->ascp_req_count = 0;
  state->fifo_info.reconfig_req = true;

  if(state->fifo_info.reconfig_req)
    lsm6dsm_reconfig_fifo(this, false);

  state->fifo_info.last_timestamp = sns_get_system_time();

  return rv;
}


/**
 * Gets current Accel ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 */
float lsm6dsm_get_accel_odr(lsm6dsm_accel_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for(idx = 0; idx < lsm6dsm_odr_map_len; idx++)
  {
    if(curr_odr == lsm6dsm_odr_map[idx].accel_odr_reg_value
       &&
       curr_odr != LSM6DSM_ACCEL_ODR_OFF)
    {
      odr = lsm6dsm_odr_map[idx].odr;
      break;
    }
  }

  return odr;
}

/**
 * Provides sample interval based on current ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time lsm6dsm_get_sample_interval(lsm6dsm_accel_odr curr_odr)
{
  sns_time  sample_interval = 0;
  float odr = lsm6dsm_get_accel_odr(curr_odr);

  if(odr > 0.0)
  {
    sample_interval = sns_convert_ns_to_ticks(1000000000.0f / odr);
  }

  return sample_interval;
}

/**
 * Extract a gyro sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] fifo_sample        The segment of fifo buffer that has the gyro sample
 * @param[i] timestamp          Timestamp to be used for this sample
 * @param[i] filter_delay       Sample filter delay in ticks.
 * @param[i] instance           The current lsm6dsm sensor instance
 * @param[i] event_service      Event service for sending out the gyro sample
 * @param[i] state              The state of the lsm6dsm sensor instance
 */
static void lsm6dsm_handle_gyro_sample(const uint8_t fifo_sample[6],
                                sns_time timestamp,
                                sns_sensor_instance *const instance,
                                sns_event_service *event_service,
                                lsm6dsm_instance_state *state,
                                void *log_gyro_state_raw_info)
{
  UNUSED_VAR(event_service);
#if LSM6DSM_LOGGING_DISABLED
  UNUSED_VAR(log_gyro_state_raw_info);
#endif
  sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  const uint8_t* ip_raw = &fifo_sample[0];

  if(state->gyro_info.num_samples_to_discard != 0)
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    state->gyro_info.num_samples_to_discard--;
    if(state->gyro_info.num_samples_to_discard == 0)
    {
      DBG_INST_PRINTF(MED, instance, "discarding GYRO ts=%u (#%u)",
          (uint32_t)timestamp, state->gyro_sample_counter);
    }
  }
  /* protect config_sensors from being overwritten by HW INT
     during reconfig */
  else if(!state->fifo_info.reconfig_req)
  {
    //data stabilized reset config sensors bit
    state->common_info.odr_changed &= ~LSM6DSM_GYRO;
  }

  if((status == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH) ||
     (state->common_info.odr_changed  & LSM6DSM_GYRO)) {
    ip_raw = &fifo_sample[0];
    state->gyro_info.opdata_status = status;
  } else {
    ip_raw = &state->gyro_info.opdata_raw[0];
    status = state->gyro_info.opdata_status;
  }

  sns_time filter_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.odr_idx].gyro_group_delay * 1000000);
  if(state->fifo_info.publish_sensors & LSM6DSM_GYRO)
  {
    vector3 opdata_cal;

    float ipdata[TRIAXIS_NUM] = {0};
    float opdata_raw[TRIAXIS_NUM] = {0};
    uint_fast8_t i;

    for(i = 0; i < ARR_SIZE(ipdata); i++)
    {
      uint_fast8_t j = i << 1;
      ipdata[i] =
        (int16_t)(((ip_raw[j+1] << 8) & 0xFF00) | ip_raw[j]) *
        ((state->gyro_info.sstvt * (1.0f + state->gyro_info.sstvt_adj[i] / 100.0f) *  PI) / 180.0f);
      state->gyro_info.opdata_raw[j] = ip_raw[j];
      state->gyro_info.opdata_raw[j+1] = ip_raw[j+1];
    }

    for(i = 0; i < ARR_SIZE(opdata_raw); i++)
    {
      opdata_raw[state->axis_map[i].opaxis] =
        (state->axis_map[i].invert ? -1.0f : 1.0f) * ipdata[state->axis_map[i].ipaxis];
    }

    // factory calibration
    opdata_cal = sns_apply_calibration_correction_3(
        make_vector3_from_array(opdata_raw),
        make_vector3_from_array(state->gyro_registry_cfg.fac_cal_bias),
        state->gyro_registry_cfg.fac_cal_corr_mat);
#if LSM6DSM_AUTO_DEBUG
  if(status != SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE){
    SNS_INST_PRINTF(HIGH, instance, "ori gyro sample(*1000): %d, %d, %d",
                   (int32_t)(opdata_cal.data[0]*1000),
                   (int32_t)(opdata_cal.data[1]*1000),
                   (int32_t)(opdata_cal.data[2]*1000));
    SNS_INST_PRINTF(HIGH, instance, "last_ts=%u count=%u",
                    (uint32_t)timestamp, state->gyro_sample_counter);
  }
#endif

#if LSM6DSM_AUTO_DEBUG
  SNS_INST_PRINTF(HIGH, instance, "g_sample time=%u new_gyro=%u",(uint32_t)timestamp,(uint32_t)(timestamp - filter_delay + state->oem_ts_offset));
#endif

    pb_send_sensor_stream_event(instance,
        &state->gyro_info.suid,
        timestamp - filter_delay + state->oem_ts_offset,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        status,
        opdata_cal.data,
        ARR_SIZE(opdata_cal.data),
        state->encoded_imu_event_len);
    LSM6DSM_INST_DEBUG_TS(LOW, instance, "GYRO  ts=%u", (uint32_t)timestamp);
#if LSM6DSM_AUTO_DEBUG
  SNS_INST_PRINTF(HIGH, instance, "g_sample time=%u",(uint32_t)timestamp);
#endif
    state->gyro_sample_counter++;
    if((state->gyro_sample_counter & 0x3FF) == 0)
    {
      DBG_INST_PRINTF(LOW, instance, "GYRO  ts=%08X/%u #s=%u",
                      (uint32_t)timestamp, (uint32_t)timestamp, state->gyro_sample_counter);
    }

#if !LSM6DSM_LOGGING_DISABLED
    // Log raw uncalibrated sensor data
    lsm6dsm_log_sensor_state_raw_add(
        (log_sensor_state_raw_info *)log_gyro_state_raw_info,
        opdata_raw,
        LSM6DSM_NUM_AXES,
        timestamp - filter_delay + state->oem_ts_offset,
        status);
#endif
  }
}

/**
 * Extract a accel sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] fifo_sample        The segment of fifo buffer that has the accel sample
 * @param[i] timestamp          Timestamp to be used for this sample
 * @param[i] filter_delay       Sample filter delay in ticks
 * @param[i] instance           The current lsm6dsm sensor instance
 * @param[i] event_service      Event service for sending out the gyro sample
 * @param[i] state              The state of the lsm6dsm sensor instance
 * @param[i] log_accel_state_raw_info The log_sensor_state_raw_info for accel
 */
static void lsm6dsm_handle_accel_sample(const uint8_t fifo_sample[6],
                                sns_time timestamp,
                                sns_sensor_instance *const instance,
                                sns_event_service *event_service,
                                lsm6dsm_instance_state *state,
                                void *log_accel_state_raw_info)
{
  UNUSED_VAR(event_service);
#if LSM6DSM_LOGGING_DISABLED
  UNUSED_VAR(log_accel_state_raw_info);
#endif
  const uint8_t* ip_raw = &fifo_sample[0];
  sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

  if(state->accel_info.num_samples_to_discard != 0)
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    state->accel_info.num_samples_to_discard--;
    if(state->accel_info.num_samples_to_discard == 0)
    {
      DBG_INST_PRINTF(MED, instance, "discarding ACCEL ts=%u (#%u)",
          (uint32_t)timestamp, state->accel_sample_counter);
    }
  }
  /* protect config_sensors from being overwrited by HW INT
     during reconfig */
  else if(!state->fifo_info.reconfig_req)
  {
    //data stabilized reset config sensors bit
    state->common_info.odr_changed &= ~LSM6DSM_ACCEL;
  }

  sns_time filter_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.odr_idx].accel_group_delay * 1000000);
  if((status == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH) ||
     (state->common_info.odr_changed & LSM6DSM_ACCEL)) {
    ip_raw = &fifo_sample[0];
    state->accel_info.opdata_status = status;
  } else {
    ip_raw = &state->accel_info.opdata_raw[0];
    status = state->accel_info.opdata_status;
  }

  LSM6DSM_INST_DEBUG_TS(LOW, instance, "reconfig_req=%d publish_s=0x%x inst_publish_s=0x%x",
      state->fifo_info.reconfig_req, state->fifo_info.publish_sensors,
      state->fifo_info.inst_publish_sensors);
  if(state->fifo_info.inst_publish_sensors & LSM6DSM_ACCEL)
  {
    vector3 opdata_cal;
    float ipdata[TRIAXIS_NUM] = {0};
    float opdata_raw[TRIAXIS_NUM] = {0};
    uint_fast8_t i;

    for(i = 0; i < ARR_SIZE(ipdata); i++)
    {
      uint_fast8_t j = i << 1;
      ipdata[i] =
        (int16_t)(((ip_raw[j+1] << 8) & 0xFF00) | ip_raw[j]) *
        state->accel_info.sstvt * G / 1000000;
      state->accel_info.opdata_raw[j] = ip_raw[j];
      state->accel_info.opdata_raw[j+1] = ip_raw[j+1];
    }

    for(i = 0; i < ARR_SIZE(opdata_raw); i++)
    {
      opdata_raw[state->axis_map[i].opaxis] =
        (state->axis_map[i].invert ? -1.0f : 1.0f) * ipdata[state->axis_map[i].ipaxis];
    }

    // factory calibration
    opdata_cal = sns_apply_calibration_correction_3(
        make_vector3_from_array(opdata_raw),
        make_vector3_from_array(state->accel_registry_cfg.fac_cal_bias),
        state->accel_registry_cfg.fac_cal_corr_mat);

    LSM6DSM_INST_DEBUG_TS(LOW, instance, "ACCEL ts=%u", (uint32_t)timestamp);
#if LSM6DSM_AUTO_DEBUG
    SNS_INST_PRINTF(HIGH, instance, "a_sample time=%u",(uint32_t)timestamp);
#endif
    state->accel_sample_counter++;

    if((state->accel_sample_counter & 0x3FF) == 0)
    {
      DBG_INST_PRINTF(LOW, instance, "ACCEL ts=%08X/%u #s=%u",
                      (uint32_t)timestamp, (uint32_t)timestamp, state->accel_sample_counter);
    }

#if LSM6DSM_AUTO_DEBUG
  if(status != SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE)  {
    SNS_INST_PRINTF(HIGH, instance, "ori accel sample(*1000): %d, %d, %d",
                   (int32_t)(opdata_cal.data[0]*1000),
                   (int32_t)(opdata_cal.data[1]*1000),
                   (int32_t)(opdata_cal.data[2]*1000));
    SNS_INST_PRINTF(HIGH, instance, "last_ts=%u ",
                    (uint32_t)timestamp, state->accel_sample_counter);
  }
#endif
#if LSM6DSM_DEBUG_SENSOR_DATA
    DBG_INST_PRINTF(HIGH, instance, "ori accel sample(*1000): %d, %d, %d",
                                   (int32_t)(opdata_cal.data[0]*1000),
                                   (int32_t)(opdata_cal.data[1]*1000),
                                   (int32_t)(opdata_cal.data[2]*1000));
#endif

#if LSM6DSM_AUTO_DEBUG
    SNS_INST_PRINTF(HIGH, instance, "a_sample time=%u new_accel=%u",(uint32_t)timestamp,(uint32_t)(timestamp - filter_delay + state->oem_ts_offset));
#endif

    pb_send_sensor_stream_event(instance,
        &state->accel_info.suid,
        timestamp - filter_delay + state->oem_ts_offset,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        status,
        opdata_cal.data,
        ARR_SIZE(opdata_cal.data),
        state->encoded_imu_event_len);

#if !LSM6DSM_LOGGING_DISABLED
    // Log raw uncalibrated sensor data
    lsm6dsm_log_sensor_state_raw_add(
        (log_sensor_state_raw_info *)log_accel_state_raw_info,
        opdata_raw,
        LSM6DSM_NUM_AXES,
        timestamp - filter_delay + state->oem_ts_offset,
        status);
#endif
  }
  else {
    state->accel_info.opdata_status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  }
}

int64_t lsm6dsm_timestamps_correction(sns_sensor_instance *instance,
    uint16_t sample_sets,
    sns_time irq_time,
    sns_time last_ts,
    sns_time sample_interval_ticks)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
  int64_t ts_drift = irq_time - (last_ts + sample_sets * sample_interval_ticks);
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
#if !LSM6DSM_LOGGING_DISABLED
void lsm6dsm_init_raw_log(
    sns_sensor_instance* instance,
    log_sensor_state_raw_info* log_state_raw_info,
    sns_sensor_uid* suid)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
// Allocate Sensor State Raw log packets for accel and gyro
  sns_memzero(log_state_raw_info, sizeof(log_sensor_state_raw_info));
  log_state_raw_info->encoded_sample_size = state->log_raw_encoded_size;
  log_state_raw_info->diag = state->diag_service;
  log_state_raw_info->instance = instance;
  log_state_raw_info->sensor_uid = suid;
  log_state_raw_info->log = NULL;
}
#endif

void lsm6dsm_process_fifo_data_buffer(
  sns_sensor_instance *instance,
  bool                gyro_enabled,
  sns_time            first_timestamp,
  sns_time            use_time,
  sns_time            sample_interval_ticks,
  const uint8_t       *fifo_start,
  size_t              num_bytes,
  uint16_t            total_sample_sets,
  bool                is_first_batch
)
{
  uint16_t num_sample_sets = 0;
  uint32_t i;
  uint8_t offset = 0;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
  sns_service_manager *service_manager = instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
#if !LSM6DSM_LOGGING_DISABLED
  log_sensor_state_raw_info log_accel_state_raw_info, log_gyro_state_raw_info;
#endif
  sns_time timestamp=0;
  //uint8_t sample_size = (gyro_enabled) ?
  //  STM_LSM6DSM_FIFO_SAMPLE_SIZE*2 : STM_LSM6DSM_FIFO_SAMPLE_SIZE;
#if !LSM6DSM_LOGGING_DISABLED
  bool log_accel =
    ((!state->fifo_info.reconfig_req && (state->fifo_info.publish_sensors & LSM6DSM_ACCEL)) ||
     (state->fifo_info.reconfig_req && (state->fifo_info.inst_publish_sensors & LSM6DSM_ACCEL)) ||
     (state->accel_info.gated_client_present && !state->current_conf.md_enabled));
  bool log_gyro = gyro_enabled;
#endif
  if(fifo_start == NULL)
  {
    SNS_INST_PRINTF(ERROR,instance,
        "ASCP buffer has NULL address!");
    state->num_ascp_null_events++;
    return;
  }

  //Temporary debug variable. To be removed
  state->fifo_start_address = (uint8_t *)fifo_start;

  bool is_streaming = (!state->self_test_info.test_alive);

  LSM6DSM_INST_DEBUG_TS(HIGH, instance,
      "self_test alive %d ascp_req_count %d is_streaming %d",
      state->self_test_info.test_alive, state->ascp_req_count, is_streaming);

  LSM6DSM_INST_DEBUG_TS(HIGH, instance,
      "first_timestamp %u sample_interval_ticks %u",
      (uint32_t)first_timestamp, (uint32_t)sample_interval_ticks);
#if !LSM6DSM_LOGGING_DISABLED
  lsm6dsm_init_raw_log(instance, &log_accel_state_raw_info, &state->accel_info.suid);
#endif
  offset = 5;

  if(gyro_enabled)
  {
#if !LSM6DSM_LOGGING_DISABLED
    lsm6dsm_init_raw_log(instance, &log_gyro_state_raw_info, &state->gyro_info.suid);
#endif
    offset = 11;
  }

  if(0 == state->fifo_info.last_timestamp)
  {
    state->fifo_info.last_timestamp = first_timestamp - sample_interval_ticks;
  }

  LSM6DSM_INST_DEBUG_TS(HIGH, instance,
      "validate timestamps start = %u sample_ticks = %u",
      (uint32_t)first_timestamp, (uint32_t)sample_interval_ticks);
  if(!is_first_batch &&
     (state->fifo_info.bh_info.interrupt_fired && state->fifo_info.interrupt_cnt > 1)) {
    int64_t sample_time_correction = lsm6dsm_timestamps_correction(
        instance, total_sample_sets, use_time,
        state->fifo_info.last_timestamp, sample_interval_ticks);
    LSM6DSM_INST_DEBUG_TS(HIGH, instance,
        "correction:  a/c= %u/%u correction = %d",
        (uint32_t)sample_interval_ticks,
        (uint32_t)(sample_interval_ticks+sample_time_correction),
        (int32_t)sample_time_correction);

    sample_interval_ticks += sample_time_correction;
    first_timestamp += sample_time_correction;
  }
  sns_time max_use_time = ((use_time + (state->fifo_info.nominal_sampling_intvl/10)) < state->fifo_info.bh_info.cur_time) ?
                            (use_time + (state->fifo_info.nominal_sampling_intvl/10)) : use_time;
  for(i = 0; i+offset < num_bytes; i += 6)
  {
    timestamp = first_timestamp + (num_sample_sets * sample_interval_ticks);

    bool gyro_drop = false;
    bool accel_drop = false;
    if(is_first_batch && state->current_conf.last_odr_idx)
    {
      sns_time last_gyro_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.last_odr_idx].gyro_group_delay * 1000000);
      sns_time last_accel_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.last_odr_idx].accel_group_delay * 1000000);
      sns_time cur_gyro_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.odr_idx].gyro_group_delay * 1000000);
      sns_time cur_accel_delay = sns_convert_ns_to_ticks(lsm6dsm_odr_map[state->current_conf.odr_idx].accel_group_delay * 1000000);
      if((timestamp - cur_gyro_delay) < (state->fifo_info.last_timestamp - last_gyro_delay))
        gyro_drop = true;
      if((timestamp - cur_accel_delay) < (state->fifo_info.last_timestamp - last_accel_delay))
        accel_drop = true;
      if(gyro_drop || accel_drop)
        LSM6DSM_INST_DEBUG_TS(HIGH, instance,
          "Drop g/a=%d/%d", gyro_drop, accel_drop);
    }
    if(timestamp <= max_use_time) {

      if(gyro_enabled && !gyro_drop) {
        if(is_streaming) {
#if !LSM6DSM_LOGGING_DISABLED
          if(!num_sample_sets)
            lsm6dsm_log_sensor_state_raw_alloc(&log_gyro_state_raw_info, 0);
#endif
          // First sample belongs to Gyro
          lsm6dsm_handle_gyro_sample(&fifo_start[i],
              timestamp,
              instance,
              event_service,
              state,
#if LSM6DSM_LOGGING_DISABLED
              NULL);
#else
              &log_gyro_state_raw_info);
#endif

#if LSM6DSM_AUTO_DEBUG
          if(state->missingSamples){
            SNS_INST_PRINTF(HIGH, instance,"num_sample_sets=1");
            state->missingSamples = false;
          }
          if(num_sample_sets == 0){
             sns_time curr_time= sns_get_system_time();
             SNS_INST_PRINTF(HIGH, instance, "auto_debug_drift_gyro=%u total_sample_sets=%u",curr_time,total_sample_sets);
          }
#endif
        } else {
          //skip sending sample, but make sure all timestamps are updated
          DBG_INST_PRINTF_EX(HIGH, instance, "Skipping handle fifo sample gyro.");
        }
        i += 6;
      }
      if(!accel_drop)
      {
        if(is_streaming) {
#if !LSM6DSM_LOGGING_DISABLED
          if(!num_sample_sets && log_accel)
            lsm6dsm_log_sensor_state_raw_alloc(&log_accel_state_raw_info, 0);
#endif
          lsm6dsm_handle_accel_sample(&fifo_start[i],
              timestamp,
              instance,
              event_service,
              state,
#if LSM6DSM_LOGGING_DISABLED
              NULL);
#else
              &log_accel_state_raw_info);
#endif
#if LSM6DSM_AUTO_DEBUG
        if(state->missingSamples){
          SNS_INST_PRINTF(HIGH, instance,"num_sample_sets=1");
          state->missingSamples = false;
          }

        if(num_sample_sets == 0){
           sns_time curr_time= sns_get_system_time();
           SNS_INST_PRINTF(HIGH, instance, "auto_debug_drift_accel=%u",curr_time);
          }
#endif
        }
      } else {
        //skip sending sample, but make sure all timestamps are updated
        DBG_INST_PRINTF_EX(HIGH, instance, "Skipping handle fifo sample accel.");
      }
      state->fifo_info.last_timestamp = timestamp;
    } else {
      if(state->accel_info.num_samples_to_discard)
        state->accel_info.num_samples_to_discard--;
      if(state->gyro_info.num_samples_to_discard)
        state->gyro_info.num_samples_to_discard--;
      LSM6DSM_INST_DEBUG_TS(HIGH, instance,
          "Drop : last_ts=%u ts=%u use_time=%u discard=a/g %u/%u",
          (uint32_t)state->fifo_info.last_timestamp, (uint32_t)timestamp,
          (int32_t)use_time,
          state->accel_info.num_samples_to_discard,
          state->gyro_info.num_samples_to_discard);
    }

    num_sample_sets++;
  }
  state->current_conf.last_odr_idx = state->current_conf.odr_idx;
#if !LSM6DSM_LOGGING_DISABLED
  // Skip logging data when self test is running or driver has not initiated any async read
  if(is_streaming)
  {
    if(log_accel)
    {
      lsm6dsm_log_sensor_state_raw_submit(&log_accel_state_raw_info, LSM6DSM_NUM_AXES, true);
    }
    if(log_gyro)
    {
      lsm6dsm_log_sensor_state_raw_submit(&log_gyro_state_raw_info, LSM6DSM_NUM_AXES, true);
    }
  }
#endif

  LSM6DSM_INST_DEBUG_TS(HIGH, instance,
      "[use_time_drift]last_ts=%u use_time=%u drift=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)use_time,
      (int32_t)(use_time - state->fifo_info.last_timestamp));

  LSM6DSM_INST_DEBUG_TS(HIGH, instance,
      "[cur_time_drift]last_ts=%u cur_time=%u drift=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)sns_get_system_time(),
      (int32_t)(sns_get_system_time() - state->fifo_info.last_timestamp));
}

void lsm6dsm_count_sample_sets(
    sns_sensor_instance *instance,
    const uint8_t* buffer,
    uint32_t bytes,
    uint16_t* restrict accel_count,
    uint16_t* restrict gyro_count)
{
  UNUSED_VAR(buffer);
  *accel_count = 0;
  *gyro_count = 0;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  uint8_t sample_size = (gyro_enabled) ?
    STM_LSM6DSM_FIFO_SAMPLE_SIZE*2 : STM_LSM6DSM_FIFO_SAMPLE_SIZE;

  *accel_count = *gyro_count = bytes/sample_size;
  LSM6DSM_INST_DEBUG_TS(LOW, instance, "a/g=%d/%d ", *accel_count, *gyro_count);
}

/*estimated end timestamp based on sampling intvl, number of sample set and interrupt ts*/
sns_time lsm6dsm_get_use_time(sns_sensor_instance *this,
                              uint16_t num_sample_sets,
                              sns_time sampling_intvl)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)this->state->state;
  uint16_t cur_wmk = state->fifo_info.bh_info.wmk;
  sns_time cur_time = state->fifo_info.bh_info.cur_time;
  // use_time is the boarder ts, calculated timestamps should not go beyond this
  sns_time use_time = state->fifo_info.bh_info.interrupt_ts;

  if(state->fifo_info.bh_info.interrupt_fired) {
    use_time = state->fifo_info.bh_info.interrupt_ts + (sampling_intvl * ((num_sample_sets >= cur_wmk) ? (num_sample_sets - cur_wmk) : 0 ));
  } else {
    //defines time when the data request sent, does not represent actual cur time
    use_time = state->fifo_info.last_timestamp + sampling_intvl * num_sample_sets;
  }

  //if use time is greater than cur_time
  if(use_time > cur_time)
    use_time = (state->fifo_info.bh_info.interrupt_fired && (num_sample_sets == cur_wmk)) ? state->fifo_info.bh_info.interrupt_ts : cur_time;
  return use_time;
}

sns_time lsm6dsm_get_first_ts(sns_sensor_instance *this,
                              uint16_t num_sample_sets,
                              sns_time sampling_intvl,
                              sns_time use_time)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)this->state->state;
  //one of the timestamp should be accurate, either last_ts valid or irq ts
  sns_time first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;
  //for the first sample after the odr change  take first timestamp as (irq - SI*samples)
  if((state->common_info.odr_changed & LSM6DSM_ACCEL) &&
      (state->fifo_info.interrupt_cnt ==1) &&
       state->fifo_info.bh_info.interrupt_fired &&
       (num_sample_sets <= state->fifo_info.bh_info.wmk) &&
       (state->accel_sample_counter || state->gyro_sample_counter)){
      first_timestamp = state->fifo_info.bh_info.interrupt_ts - sampling_intvl * (num_sample_sets - 1);
  }
  if(!state->fifo_info.last_ts_valid &&
      state->fifo_info.bh_info.interrupt_fired) {
    first_timestamp = use_time - sampling_intvl * (num_sample_sets - 1);
    int64_t first_ts_gap = first_timestamp - state->fifo_info.last_timestamp;
    if((state->accel_sample_counter == 0) && (state->gyro_sample_counter == 0))
      return first_timestamp;

    LSM6DSM_INST_DEBUG_TS(HIGH, this,
      "get_first_ts: first_ts_gap=%d last_ts_valid=%d", first_ts_gap, state->fifo_info.last_ts_valid);
    //either we cross last ts or
    //DAE did not drop any samples(last_ts_valid is used to check if DAE dropped some data or not)
    if(first_ts_gap < 0 || (first_ts_gap < (sampling_intvl + state->fifo_info.nominal_sampling_intvl)) ||
      (lsm6dsm_dae_if_available(this) && (first_ts_gap < sampling_intvl * state->fifo_info.cur_wmk))) {
      first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;
      state->fifo_info.last_ts_valid = true;
    }
  }
  return first_timestamp;
}

void lsm6dsm_calculate_sampling_intvl(sns_sensor_instance *this,
    uint16_t num_sample_sets,
    sns_time* sampling_intvl)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)this->state->state;
  uint16_t cur_wmk = state->fifo_info.cur_wmk;
  sns_time cal_st  = 0;
  uint8_t odr_tolerance;
  LSM6DSM_INST_DEBUG_TS(HIGH, this,
      "calculate_ts: last_ts=%u irq_ts=%u num_sample_sets=%d",
      (uint32_t)state->fifo_info.last_timestamp, (uint32_t)state->fifo_info.bh_info.interrupt_ts,
      num_sample_sets);

#if LSM6DSM_AUTO_DEBUG
   SNS_INST_PRINTF(HIGH, this, "num_sample_sets=%u", num_sample_sets);
#endif

  if(state->fifo_info.bh_info.accel_odr == state->fifo_info.fifo_rate)
  {
    if((state->fifo_info.bh_info.interrupt_fired) && (num_sample_sets == cur_wmk)) {
      *sampling_intvl = lsm6dsm_estimate_avg_st(this, state->fifo_info.bh_info.interrupt_ts, num_sample_sets);
    } else if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT) {
      state->fifo_info.interrupt_cnt = 0;
      if(state->fifo_info.bh_info.interrupt_fired) {
        cal_st = (state->fifo_info.bh_info.interrupt_ts -
            state->fifo_info.last_timestamp) / num_sample_sets;
      } else {
        cal_st = state->fifo_info.nominal_sampling_intvl;
      }
      state->fifo_info.avg_sampling_intvl = cal_st;
      if((state->common_info.gyro_curr_odr == LSM6DSM_GYRO_ODR_OFF &&
          state->fifo_info.avg_to_nominal_ratio == 1.0f) ||
         (state->common_info.gyro_curr_odr != LSM6DSM_GYRO_ODR_OFF &&
          state->fifo_info.avg_to_nominal_ratio_g == 1.0f))
      {
        odr_tolerance = LSM6DSM_ODR_TOLERANCE_10;
      }
      else
      {
        odr_tolerance = LSM6DSM_ODR_TOLERANCE;
      }
      if(!LSM6DSM_IS_INBOUNDS(cal_st, state->fifo_info.nominal_sampling_intvl,
                odr_tolerance)) {
        state->fifo_info.avg_sampling_intvl = state->fifo_info.nominal_sampling_intvl;
      }
      *sampling_intvl = state->fifo_info.avg_sampling_intvl;
    }
  }
}

void lsm6dsm_send_missing_samples(
    sns_sensor_instance *instance,
    bool gyro_enabled,
    sns_time sampling_intvl,
    sns_time use_time,
    uint16_t missing_samples)
{

  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;

  DBG_INST_PRINTF_EX(HIGH, instance,
      "missing_samples=%u intv=%u use_time=%u last_ts=%u",
      missing_samples, (uint32_t)sampling_intvl, (uint32_t)use_time,
      (uint32_t)state->fifo_info.last_timestamp);

  if(missing_samples > 0 && missing_samples <= LSM6DSM_MAX_MISSING_SAMPLES) {
    sns_time first_timestamp = state->fifo_info.last_timestamp;
    uint16_t idx = 0, i = 0;
    uint8_t buffer[STM_LSM6DSM_FIFO_SAMPLE_SIZE << 1];
    uint8_t sample_size = STM_LSM6DSM_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);

    if(gyro_enabled) {
      sns_memscpy(&buffer[idx],
          STM_LSM6DSM_SAMPLE_SIZE,
          state->gyro_info.opdata_raw,
          STM_LSM6DSM_SAMPLE_SIZE);
      idx += STM_LSM6DSM_FIFO_SAMPLE_SIZE;
    }
    sns_memscpy(&buffer[idx],
        STM_LSM6DSM_SAMPLE_SIZE,
        state->accel_info.opdata_raw,
        STM_LSM6DSM_SAMPLE_SIZE);
    if((state->gyro_info.num_samples_to_discard != 0) ||
      (state->gyro_info.opdata_status == SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE)) {
      state->gyro_info.num_samples_to_discard += missing_samples;
    }

    if((state->accel_info.num_samples_to_discard != 0) ||
      (state->accel_info.opdata_status == SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE)) {
      state->accel_info.num_samples_to_discard += missing_samples;
    }

    sns_time end_ts = use_time;
    for(i = 0; i < missing_samples; i++) {
    #if LSM6DSM_AUTO_DEBUG
      state->missingSamples = true;
    #endif

      first_timestamp = state->fifo_info.last_timestamp + sampling_intvl;
      end_ts = (i == (missing_samples - 1)) ? use_time : (state->fifo_info.last_timestamp + ((100 + LSM6DSM_ODR_TOLERANCE)/100.0f * sampling_intvl));
      lsm6dsm_process_fifo_data_buffer(instance,
          gyro_enabled,
          first_timestamp,
          end_ts,
          sampling_intvl,
          buffer,
          sample_size,
          1,
          false);
    }
  }
  else if(missing_samples > 0)
    state->fifo_info.last_ts_valid = false;
}

void lsm6dsm_adjust_ts_drift(
    sns_sensor_instance *instance,
    bool gyro_enabled,
    sns_time cur_time,
    sns_time sampling_intvl,
    sns_time use_time,
    uint16_t odr_changed)
{

  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
  UNUSED_VAR(use_time);
  //check the drift w.r.t interrupt ts
  //in flush case fifo_info.ts stores the time when flush started
  //in interrupt case fifo_info.ts stores the time when int received
  //try end timestamp to be closer to this
  int64_t ts_drift;
  float low_bound_odr_tolerance = (odr_changed)? ((100 - LSM6DSM_ODR_TOLERANCE)/100.0f) : 1.5f;
  sns_time low_bound_sampling_intvl = low_bound_odr_tolerance  * sampling_intvl;
  sampling_intvl = (odr_changed)? low_bound_sampling_intvl : sampling_intvl;
  uint32_t missing_samples = 0;

  if(!state->fifo_info.bh_info.interrupt_fired) {
    use_time = cur_time;
  }
  ts_drift = use_time - state->fifo_info.last_timestamp;
  //in case of flush- cur time is flush time
  if(ts_drift > 0) {
    missing_samples = ts_drift / low_bound_sampling_intvl;
  }

  if(missing_samples > 0) {
    DBG_INST_PRINTF(HIGH, instance,
        "#missing=%u intv=%u use_time=%u last_ts=%u",
        missing_samples, (uint32_t)sampling_intvl, (uint32_t)use_time,
        (uint32_t)state->fifo_info.last_timestamp);
  }

  lsm6dsm_send_missing_samples(
        instance,
        gyro_enabled,
        sampling_intvl,
        use_time,
        missing_samples);

}


void lsm6dsm_send_fifo_data(
    sns_sensor_instance *instance,
    const uint8_t* buffer,
    uint32_t bytes,
    bool gyro_enabled)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;
  uint16_t num_sample_sets;
  uint16_t accel_count, gyro_count;
  sns_time use_time;
  sns_time sampling_intvl = state->fifo_info.avg_sampling_intvl;
  sns_time first_timestamp;
  uint16_t odr_changed = state->common_info.odr_changed;

  lsm6dsm_count_sample_sets(instance, buffer, bytes, &accel_count, &gyro_count);
  num_sample_sets = SNS_MAX(accel_count, gyro_count);

#if LSM6DSM_AUTO_DEBUG
  SNS_INST_PRINTF(HIGH, instance, "c_wmk=%u bytes=%u",
      state->fifo_info.cur_wmk, bytes);
#endif
  LSM6DSM_INST_DEBUG_TS(LOW, instance,
      "cur_wmk=%u bytes=%u num_sample_sets=%d",
      state->fifo_info.cur_wmk, bytes, num_sample_sets);

  if(num_sample_sets >= 1)
  {
    lsm6dsm_calculate_sampling_intvl(instance, num_sample_sets, &sampling_intvl);
    // use_time is the boarder ts, calculated timestamps should not go beyond this
    use_time = lsm6dsm_get_use_time(instance, num_sample_sets, sampling_intvl);
    //calculate first timestamp
    first_timestamp = lsm6dsm_get_first_ts(instance, num_sample_sets, sampling_intvl, use_time);

    LSM6DSM_INST_DEBUG_TS(LOW, instance,
        "sampling_intv=%u last_ts=%u first_ts=%u use_time=%u",
        (uint32_t)sampling_intvl, (uint32_t)state->fifo_info.last_timestamp,
        (uint32_t)first_timestamp, (uint32_t)use_time);
#if LSM6DSM_AUTO_DEBUG
    SNS_INST_PRINTF(HIGH, instance, "sampling_intv=%u last_ts=%u first_ts=%u",
        (uint32_t)sampling_intvl, (uint32_t)state->fifo_info.last_timestamp,
        (uint32_t)first_timestamp);
#endif

    // QC: In addition to taking the 1st timestamp, this should probably take an "n" and the "nth sample timestamp".
    // In normal cases (including late reads) N would be the watermark, and the nth sample timestamp would be the interrupt timestamp.
    // This will also prevent having to caluclate drifts and invent completely incorrect samples below.
    lsm6dsm_process_fifo_data_buffer(instance,
        gyro_enabled,
        first_timestamp,
        use_time,
        sampling_intvl,
        buffer,
        bytes,
        num_sample_sets,
        (state->fifo_info.interrupt_cnt == 1));

    //check the drift w.r.t interrupt ts
    //in flush case fifo_info.ts stores the time when flush started
    //in interrupt case fifo_info.ts stores the time when int received
    //try end timestamp to be closer to this

    state->fifo_info.last_ts_valid = true;
    if(state->fifo_info.interrupt_cnt < MAX_INTERRUPT_CNT ||
       odr_changed) 
    {
      lsm6dsm_adjust_ts_drift(instance, gyro_enabled, state->fifo_info.bh_info.cur_time,
                              sampling_intvl, use_time, odr_changed);
    }

    if((state->fifo_info.max_requested_wmk * sampling_intvl >
      state->fifo_info.max_requested_flush_ticks) &&
      lsm6dsm_dae_if_available(instance)) {
      state->fifo_info.last_ts_valid = false;
    }
  }
}
void lsm6dsm_process_com_port_vector(sns_port_vector *vector, void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  if(STM_LSM6DSM_REG_FIFO_DATA_OUT_L == vector->reg_addr)
  {
    lsm6dsm_send_fifo_data(instance, vector->buffer, vector->bytes, gyro_enabled);
  }
}


/** See lsm6dsm_hal.h */
void lsm6dsm_send_fifo_flush_done(sns_sensor_instance *instance,
                                  lsm6dsm_sensor_type flushing_sensors,
                                  lsm6dsm_flush_done_reason reason)
{
  UNUSED_VAR(reason);
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

#if LSM6DSM_ESP_ENABLED
  if(flushing_sensors & LSM6DSM_ESP_SENSORS_MASK)
  { lsm6dsm_sensor_type esp_flushing_sensors = flushing_sensors & LSM6DSM_ESP_SENSORS_MASK;
    lsm6dsm_esp_send_fifo_flush_done(instance, esp_flushing_sensors, reason);
  }
  flushing_sensors &= ~LSM6DSM_ESP_SENSORS_MASK;
#endif
  while(flushing_sensors != 0)
  {
    sns_sensor_uid const *suid = NULL;
    lsm6dsm_sensor_type sensor_type = LSM6DSM_ACCEL;
    if(flushing_sensors & LSM6DSM_ACCEL)
    {
      suid = &state->accel_info.suid;
      sensor_type = LSM6DSM_ACCEL;
    }
    else if(flushing_sensors & LSM6DSM_GYRO)
    {
      suid = &state->gyro_info.suid;
      sensor_type = LSM6DSM_GYRO;
    }
    else if(flushing_sensors & LSM6DSM_MOTION_DETECT)
    {
      suid = &state->md_info.suid;
      sensor_type = LSM6DSM_MOTION_DETECT;
    }
    else if(flushing_sensors & LSM6DSM_SENSOR_TEMP)
    {
      suid = &state->sensor_temp_info.suid;
      sensor_type = LSM6DSM_SENSOR_TEMP;
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
      SNS_INST_PRINTF(HIGH, instance, "[%u] FLUSH_EVENT sensor=%u (%u) %u/%u/%u",
                      state->hw_idx, sensor_type, reason, state->accel_sample_counter,
                      state->gyro_sample_counter, state->num_temp_samples);
    }
  }
}

void lsm6dsm_align_fifo_data(sns_sensor_instance *const instance, uint16_t pattern_pos, uint16_t* num_of_bytes, bool is_over_run)
{
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  UNUSED_VAR(is_over_run);
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  uint8_t sample_size = (gyro_enabled) ? 12 : 6;
  //Limit pattern to between 0-5
  uint8_t pattern_pos_check = pattern_pos % (sample_size /2);
  uint8_t bytes_to_discard = sample_size - pattern_pos_check*2;
  uint8_t buffer_size = SNS_MAX(bytes_to_discard, 2);
  uint8_t buffer[buffer_size];
  sns_memset(buffer, 0, sizeof(buffer));

  if(*num_of_bytes > bytes_to_discard) {
    // Read the the  samples to discard
    lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_FIFO_DATA_OUT_L, bytes_to_discard, buffer);
    *num_of_bytes -= bytes_to_discard;
#if LSM6DSM_PATTERN_DEBUG
    //check pattern again after reading data
    if(*num_of_bytes >= sample_size) {
      lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_FIFO_STATUS3, 2, buffer);
      uint16_t pattern_pos = (((buffer[1] & 0x03) << 8) | (buffer[0] & 0xFF));
      DBG_INST_PRINTF(HIGH, instance,
                   "updated pattern:: %d num_of_bytes %d",pattern_pos, *num_of_bytes);
    }
#endif
  }
}

void lsm6dsm_update_nominal_interval(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  float * avg_to_nominal_ratio;
  float cur_nominal_ratio = 1.0f;
  int8_t *avg_cnt;
  lsm6dsm_sensor_type sensor;

  if(state->common_info.gyro_curr_odr == LSM6DSM_GYRO_ODR_OFF)
  {
    avg_to_nominal_ratio = &state->fifo_info.accel_ratio;
    avg_cnt = &state->fifo_info.avg_to_nominal_ratio_cnt;
    sensor = LSM6DSM_ACCEL;
  }
  else
  {
    avg_to_nominal_ratio = &state->fifo_info.gyro_ratio;
    avg_cnt = &state->fifo_info.avg_to_nominal_ratio_cnt_g;
    sensor = LSM6DSM_GYRO;
  }
  if(((*avg_to_nominal_ratio == 1.0f) || (*avg_cnt >= 0)) &&
     (state->fifo_info.interrupt_cnt > AVG_NOMINAL_RATIO_START_SAMPLE))
  {
    cur_nominal_ratio =
      (float)state->fifo_info.cur_sampling_intvl/(float)state->fifo_info.nominal_sampling_intvl;
    if(*avg_to_nominal_ratio == 1.0f)
    {
      *avg_cnt = 0;
      *avg_to_nominal_ratio = cur_nominal_ratio;
    }
    else
    {
      if(LSM6DSM_IS_INBOUNDS(state->fifo_info.cur_sampling_intvl,
                             lsm6dsm_get_sample_interval(state->fifo_info.bh_info.accel_odr),
                             LSM6DSM_ODR_TOLERANCE_10))
      {
        *avg_to_nominal_ratio = *avg_to_nominal_ratio+
          (cur_nominal_ratio-*avg_to_nominal_ratio)/(float)((*avg_cnt)+1);
      }
      else
      {
        --(*avg_cnt);
      }
    }
    if(++(*avg_cnt) >= AVG_NOMINAL_RATIO_SAMPLE_CNT)
    {
      if(sensor == LSM6DSM_ACCEL)
      {
        state->fifo_info.avg_to_nominal_ratio = *avg_to_nominal_ratio;
      }
      else
      {
        state->fifo_info.avg_to_nominal_ratio_g = *avg_to_nominal_ratio;
      }
      state->fifo_info.update_registry = true ;
      state->fifo_info.nominal_sampling_intvl = lsm6dsm_get_sample_interval(state->fifo_info.bh_info.accel_odr) *
                        (*avg_to_nominal_ratio);
      *avg_cnt = -1.0f;
    }
    else
    {
      DEBUG_TS_EST(HIGH, instance, "sensor %d avg_cnt=%d avg_to_nominal_ratio*1000000=%d",
      sensor, *avg_cnt, (int)(*avg_to_nominal_ratio*1000000.0));
    }
    if(LSM6DSM_IS_INBOUNDS(*avg_to_nominal_ratio,
                            1.0f,
                            LSM6DSM_ODR_TOLERANCE_10 ))
    {
      DEBUG_TS_EST(HIGH, instance,
      "Calculated avg_to_nominal_ratio*1000=%d, Gyro OFF = %d",
      (int)(*avg_to_nominal_ratio*1000.0), (state->common_info.gyro_curr_odr == LSM6DSM_GYRO_ODR_OFF));
    }
    else
    {
      *avg_to_nominal_ratio = 1.0f;
    }
  }

}
//estimate average sample time
sns_time lsm6dsm_estimate_avg_st(
  sns_sensor_instance *const instance,
  sns_time irq_timestamp,
  uint16_t num_samples)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  uint32_t cur_int_delta = (uint32_t)(irq_timestamp - state->fifo_info.interrupt_timestamp);
  sns_time sampling_intvl = state->fifo_info.avg_sampling_intvl;
  int8_t odr_tolerance;

  if(state->fifo_info.interrupt_cnt == 0) {
    cur_int_delta = (uint32_t)(irq_timestamp - state->fifo_info.last_timestamp);
  }

  state->fifo_info.cur_sampling_intvl = cur_int_delta/num_samples;

  if((state->common_info.gyro_curr_odr == LSM6DSM_GYRO_ODR_OFF &&
      state->fifo_info.avg_to_nominal_ratio == 1.0f) ||
     (state->common_info.gyro_curr_odr != LSM6DSM_GYRO_ODR_OFF &&
      state->fifo_info.avg_to_nominal_ratio_g == 1.0f))
  {
    odr_tolerance = LSM6DSM_ODR_TOLERANCE_10;
  }
  else
  {
    odr_tolerance = LSM6DSM_ODR_TOLERANCE;
  }
  state->fifo_info.interrupt_cnt++;
  if(LSM6DSM_IS_INBOUNDS(state->fifo_info.cur_sampling_intvl, state->fifo_info.nominal_sampling_intvl,
        odr_tolerance)) {

    if(state->fifo_info.interrupt_cnt > MAX_INTERRUPT_CNT) {

      uint32_t avg_int;
      uint16_t int_cnt;
      if(state->fifo_info.interrupt_cnt == UINT16_MAX)
        state->fifo_info.interrupt_cnt = WINDOW_SIZE + MAX_INTERRUPT_CNT;

      // QC - Is it guaranteed that fifo_info.interurpt_cnt + 1 is greater than MAX_INTERRUPT_CNT?
      int_cnt = state->fifo_info.interrupt_cnt - MAX_INTERRUPT_CNT + 1;
      if(state->fifo_info.interrupt_cnt >= WINDOW_SIZE+MAX_INTERRUPT_CNT)
        int_cnt = WINDOW_SIZE;

      avg_int = state->fifo_info.avg_interrupt_intvl;

      state->fifo_info.avg_interrupt_intvl += (int32_t)(cur_int_delta - avg_int)/int_cnt ;
      state->fifo_info.avg_sampling_intvl = state->fifo_info.avg_interrupt_intvl/num_samples;
      sampling_intvl = state->fifo_info.avg_sampling_intvl;

      LSM6DSM_INST_DEBUG_TS(LOW, instance, "avg_st: cnt=%d int_delta=%u prev=%u cur int:samp=%u:%u",
          int_cnt, (uint32_t)cur_int_delta, avg_int, (uint32_t)state->fifo_info.avg_interrupt_intvl,
          (uint32_t)state->fifo_info.avg_sampling_intvl);

    } else {

      DEBUG_TS_EST(HIGH, instance,
          "avg_st: irq_ts=%u prev_irq=%u delta=%d #s=%u #int=%u",
          (uint32_t)irq_timestamp, (uint32_t)state->fifo_info.interrupt_timestamp,
          cur_int_delta, num_samples, state->fifo_info.interrupt_cnt);

      if(state->fifo_info.interrupt_cnt == 1) {
        state->fifo_info.avg_sampling_intvl = state->fifo_info.nominal_sampling_intvl;
        sampling_intvl = cur_int_delta / num_samples;

      } else {

        state->fifo_info.avg_interrupt_intvl = cur_int_delta;
        state->fifo_info.avg_sampling_intvl = state->fifo_info.avg_interrupt_intvl/num_samples;
        sampling_intvl = state->fifo_info.avg_sampling_intvl;
      }

    }
  } else if(state->fifo_info.interrupt_cnt == 0) {
      //without this, we are stuck until last_ts is closer to interrupt_ts
      //increase count as interrupt is generated
      state->fifo_info.interrupt_cnt++;
  }
  state->fifo_info.interrupt_timestamp = irq_timestamp;

  lsm6dsm_update_nominal_interval(instance);

  DEBUG_TS_EST(HIGH, instance,
      "avg_st: avg_intvl(int/smp)=%u/%u ratio*1000=%d, ratio_g*1000=%d",
      state->fifo_info.avg_interrupt_intvl,
      state->fifo_info.avg_sampling_intvl,
      (int)(state->fifo_info.avg_to_nominal_ratio*1000.0),
      (int)(state->fifo_info.avg_to_nominal_ratio_g*1000.0));

  return sampling_intvl;
}

#define LSM6DSM_IS_FIFO_INTERUPT_SET(x) ((x & STM_LSM6DSM_FIFO_WTM_STATUS_MASK) ? true : false)

// is_data_read --> atleast one sample is read from fifo
void lsm6dsm_read_fifo_data_cleanup(sns_sensor_instance *const instance, bool is_fifo_read, bool is_interrupt_fired)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if(lsm6dsm_dae_if_available(instance))
    return;

  if(state->ascp_req_count <= 0) {
    //clear interrupt
    if(!is_fifo_read && is_interrupt_fired) {
      lsm6dsm_send_interrupt_is_cleared_msg(instance);
    }

    if(state->flushing_sensors != 0) {
      lsm6dsm_send_fifo_flush_done(instance, state->flushing_sensors, FLUSH_DONE_AFTER_DATA);
      state->flushing_sensors = 0;
      if(is_fifo_read) {
        lsm6dsm_restart_hb_timer(instance, true);
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
        lsm6dsm_set_fifo_wmk(instance);
        //reset avg interrupt interval
        state->fifo_info.avg_interrupt_intvl =
          state->fifo_info.avg_sampling_intvl * state->current_conf.wmk;
        //enable interrupt
        lsm6dsm_enable_fifo_intr(instance, state->fifo_info.fifo_enabled);
        state->fifo_info.inst_publish_sensors = state->fifo_info.publish_sensors;
        state->fifo_info.reconfig_req = false;
        lsm6dsm_send_config_event(instance, false);
      } else if((state->current_conf.wmk != 1) || (state->fifo_info.bh_info.interrupt_fired)) {
        lsm6dsm_reconfig_fifo(instance, false);
      }
    }
  }
}

void lsm6dsm_fill_bh_info(sns_sensor_instance *const instance)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  memcpy(&state->fifo_info.bh_info, &state->fifo_info.th_info, sizeof(lsm6dsm_fifo_req));
  //reset th parameters
  state->fifo_info.th_info.interrupt_fired = false;
  state->fifo_info.th_info.flush_req = false;
  state->fifo_info.th_info.recheck_int = false;
  LSM6DSM_INST_DEBUG_TS(LOW, instance,
      "bh_info int_fired=%d flush_req=%d s_intvl=%u",
      state->fifo_info.bh_info.interrupt_fired, state->fifo_info.bh_info.flush_req,
      (uint32_t)(state->fifo_info.avg_sampling_intvl));
  LSM6DSM_INST_DEBUG_TS(LOW, instance,
      "irq_ts=%u cur_time=%u ",
      (uint32_t)state->fifo_info.bh_info.interrupt_ts,
      (uint32_t)state->fifo_info.bh_info.cur_time);
}

void lsm6dsm_send_fw_data_read_msg(
    sns_sensor_instance *const instance,
    uint16_t num_of_bytes,
    bool gyro_enabled)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t buffer[100];
  uint32_t enc_len;
  uint8_t sample_size = STM_LSM6DSM_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);
  sns_port_vector async_read_msg;
  // Compose the async com port message
  async_read_msg.bytes = num_of_bytes;
  async_read_msg.reg_addr = STM_LSM6DSM_REG_FIFO_DATA_OUT_L;
  async_read_msg.is_write = false;
  async_read_msg.buffer = NULL;

  //if samples > ASYNC_MIN_SAMPLES use async com
  //else use sync com

  // QC: One set is 2 samples when Gyro is active, so we're OK using Sync com port for 60 bytes?
  //In case of DAE use SYNC com port only as these are samples reamining after flush and should not be more than a couple
  if(((num_of_bytes/sample_size) <= ASYNC_MIN_SAMPLES) || (lsm6dsm_dae_if_available(instance) && ((num_of_bytes/sample_size) <= DAE_SYNC_MIN_SAMPLES))) {
    uint8_t fifo_data[async_read_msg.bytes];
    // QC: is memset necessary?
    sns_memset(fifo_data, 0, sizeof(fifo_data));
    rc = lsm6dsm_read_regs_scp(instance, async_read_msg.reg_addr, async_read_msg.bytes, fifo_data);
    lsm6dsm_send_interrupt_is_cleared_msg(instance);
    if(rc == SNS_RC_SUCCESS) {
      sns_port_vector vector;
      vector.reg_addr = async_read_msg.reg_addr;
      vector.bytes = async_read_msg.bytes;
      vector.buffer = fifo_data;
      lsm6dsm_process_com_port_vector(&vector, instance);
    }
  }
  else if(lsm6dsm_dae_if_available(instance)) {
    state->fifo_info.last_ts_valid = false;
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
    LSM6DSM_INST_DEBUG_TS(HIGH, instance,
        "send ascp req ascp_req_count %d", state->ascp_req_count);
    state->ascp_req_count++;
  } else {
    SNS_INST_PRINTF(ERROR, instance, "sns_ascp_create_encoded_vectors_buffer failed");
  }
}

/** read fifo data after checking fifo int and use sync com port or async com port */
void lsm6dsm_read_fifo_data(sns_sensor_instance *const instance, sns_time irq_timestamp, bool flush)
{

  uint8_t fifo_status[4] = {0, 0, 0, 0};
  sns_rc rc = SNS_RC_SUCCESS;
  uint16_t num_of_bytes = 0;
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  UNUSED_VAR(irq_timestamp);
  bool gyro_enabled = (state->common_info.gyro_curr_odr > 0);
  uint8_t sample_size = STM_LSM6DSM_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);

  if(flush)
    state->fifo_info.th_info.flush_req = true;

  // Read the FIFO Status register
  rc = lsm6dsm_get_fifo_status(instance, &num_of_bytes, fifo_status);

  if(rc != SNS_RC_SUCCESS)
  {
    SNS_INST_PRINTF(ERROR, instance, "lsm6dsm_read_fifo_status FAILED");
    return;
  }

  uint16_t pattern_pos = (((fifo_status[3] & 0x03) << 8) | (fifo_status[2] & 0xFF));
  pattern_pos  %= (sample_size /2);

  if(pattern_pos && num_of_bytes) {
    lsm6dsm_align_fifo_data(instance, pattern_pos, &num_of_bytes, (fifo_status[1] & 0x40)) ;
  }

  if(num_of_bytes < sample_size) {
    LSM6DSM_INST_DEBUG_TS(LOW, instance,
        "#bytes %u < one pattern %u", num_of_bytes, sample_size);
    lsm6dsm_read_fifo_data_cleanup(instance, false, state->fifo_info.th_info.interrupt_fired);
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
        LSM6DSM_INST_DEBUG_TS(HIGH, instance,
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


    if(!LSM6DSM_IS_FIFO_INTERUPT_SET(fifo_status[1])) {
      state->fifo_info.th_info.recheck_int = false;
      state->fifo_info.th_info.interrupt_fired = false;
     }


    if((!state->fifo_info.th_info.recheck_int) && (!state->fifo_info.th_info.interrupt_fired) &&
        (!state->fifo_info.th_info.flush_req)) {
      //return nothing to be done here
      //useful for active_high/active_low interrupt handling
      lsm6dsm_send_interrupt_is_cleared_msg(instance);
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
    lsm6dsm_fill_bh_info(instance);
#if LSM6DSM_AUTO_DEBUG
  SNS_INST_PRINTF(HIGH, instance, "bh_info int_fired = %d flush_req = %d",
                  state->fifo_info.bh_info.interrupt_fired, state->fifo_info.bh_info.flush_req);
#endif

    lsm6dsm_send_fw_data_read_msg(instance, num_of_bytes, gyro_enabled);
    lsm6dsm_read_fifo_data_cleanup(instance, true, state->fifo_info.bh_info.interrupt_fired);
  }
}

/**
 * Extract a accel sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] instance           The current lsm6dsm sensor instance
 * @param[i] sensors[]          Array of sensors for which data is requested
 * @param[i] num_sensors        Number of sensor for which data is requested
 * @param[i] raw_data           Uncalibrated sensor data to be logged
 */
void lsm6dsm_get_data(sns_sensor_instance *const instance,
                                lsm6dsm_sensor_type sensors[],
                                uint8_t num_sensors,
                                int16_t *raw_data)
{

// Timestap is not needed for this implementation as we are not sending anything ot framework
  uint8_t read_addr;

  if((num_sensors == 2)||(sensors[0] == LSM6DSM_GYRO))
    read_addr = STM_LSM6DSM_REG_OUT_X_L_G;
  else
    read_addr = STM_LSM6DSM_REG_OUT_X_L_XL;

  lsm6dsm_read_regs_scp(instance, read_addr, 6*num_sensors, (uint8_t*)raw_data);
  DBG_INST_PRINTF_EX(LOW, instance, "DATA sensor=%u ts=%u [%d, %d, %d]",
                  sensors[0], (uint32_t)sns_get_system_time(),
                  raw_data[0], raw_data[1], raw_data[2]);


  if(num_sensors == 2) { //both accel and gyro data requested
    DBG_INST_PRINTF_EX(LOW, instance, "DATA sensor=%u ts=%u [%d, %d, %d]",
                    sensors[1], (uint32_t)sns_get_system_time(),
                    raw_data[3], raw_data[4], raw_data[5]);
  }

}

/**
 * see sns_lsm6dsm_hal.h
 */
void lsm6dsm_dump_reg(sns_sensor_instance *const instance,
                      lsm6dsm_sensor_type sensor)
{  UNUSED_VAR(sensor);
#if LSM6DSM_DUMP_REG
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  struct group_read {
    uint32_t first_reg;
    uint8_t  num_regs;
  } groups[] = { /* must fit within state->reg_status[] */
    { STM_LSM6DSM_REG_FIFO_CTRL1, 5 },
    { STM_LSM6DSM_REG_INT1_CTRL, 15 },
    { STM_LSM6DSM_REG_TAP_CFG,    9 }
  };
  uint8_t *reg_val = state->reg_status;

  for(uint32_t i=0; i<ARR_SIZE(groups); i++)
  {
    lsm6dsm_read_regs_scp(instance, groups[i].first_reg, groups[i].num_regs, reg_val);
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

void lsm6dsm_set_gated_accel_config(lsm6dsm_instance_state *state,
                      uint16_t desired_wmk,
                      lsm6dsm_accel_odr a_chosen_sample_rate,
                      lsm6dsm_sensor_type sensor)
{
  if(!state->accel_info.gated_client_present)
  {
    desired_wmk = 0;
    a_chosen_sample_rate = LSM6DSM_ACCEL_ODR_OFF;
  }

  if(sensor & LSM6DSM_ACCEL)
  {
    state->md_info.desired_wmk = desired_wmk;
    state->md_info.desired_odr = a_chosen_sample_rate;
    state->md_info.sstvt = state->accel_info.sstvt;
    state->md_info.range = state->accel_info.range;
    state->md_info.bw = LSM6DSM_ACCEL_BW50;
  }
}
void lsm6dsm_set_md_filter(sns_sensor_instance *const instance,
                            lsm6dsm_md_filter_type filter)
{
  lsm6dsm_instance_state *inst_state =
    (lsm6dsm_instance_state*)instance->state->state;
  if (inst_state->md_info.filter != filter) {
    uint32_t xfer_bytes;
    lsm6dsm_md_filter_type  md_filter = (filter ==  SLOPE_FILTER) ? 0x0 : 0x10;
    lsm6dsm_read_modify_write(instance,
        STM_LSM6DSM_REG_TAP_CFG,
        &md_filter,
        1,
        &xfer_bytes,
        false,
        0x10);
    inst_state->md_info.filter = filter;
  }
}

void lsm6dsm_set_md_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  //update the reg only if necessary
  //save some cycles
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;

  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  bool update_reg = false;
  uint32_t reg_addr = 0;
  DBG_INST_PRINTF(LOW, instance, "set_md_intr: %u %u %u %u",
                  enable, state->irq_ready, state->md_info.is_filter_settled,
                  state->md_info.cur_state.motion_detect_event_type);
  if(!((state->irq2_inst_enabled && state->irq2_info.irq_ready) ||
     state->irq_info.irq_ready)) {
    DBG_INST_PRINTF(LOW, instance, "set_md_intr: irq not ready");
    return;
  }
  if((enable) &&
      (state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED))
  {
    rw_buffer = 0
      | (0<<7)            // INT1_INACT_STATE
      | (0<<6)            // INT1_SINGLE_TAP
      | (1<<5)            // INT1_WU
      | (0<<4)            // INT1_FF
      | (0<<3)            // INT1_DOUBLE_TAP
      | (0<<2)            // INT1_6D
      | (0<<1)            // INT1_TILT
      | 0;                // INT1_TIMER
    update_reg = true;
  } else if((!enable) &&
      (state->md_info.is_filter_settled)) {
    update_reg = true;
  }

  if(update_reg) {
    state->current_conf.md_enabled = enable;
    if(state->route_md_to_irq2)
      reg_addr = STM_LSM6DSM_REG_MD2_CFG;
    else
      reg_addr = STM_LSM6DSM_REG_MD1_CFG;
    DBG_INST_PRINTF_EX(LOW, instance, "set_md_intr: reg_addr = 0x%0x, MD_CFG=0x%x", reg_addr, rw_buffer);
    lsm6dsm_read_modify_write(instance,
        reg_addr,
        &rw_buffer,
        1,
        &xfer_bytes,
        false,
        0x20);
  }
}

bool is_lsm6dsm_mdf_settled(sns_sensor_instance *this, lsm6dsm_md_filter_type filter)
{
  lsm6dsm_instance_state *inst_state =
    (lsm6dsm_instance_state*)this->state->state;
  sns_time cur_time, odr_time_elapsed, filter_settling_time;
  uint64_t settling_time_ns = 0;
  int64_t req_timeout = 0;
  uint8_t odr_idx = (inst_state->desired_conf.odr_idx) ? inst_state->desired_conf.odr_idx : inst_state->min_odr_idx;

  if(inst_state->common_info.accel_curr_odr == LSM6DSM_ACCEL_ODR_OFF) {
    DBG_INST_PRINTF_EX(LOW, this, "acc on");
  }
  if(filter == SLOPE_FILTER) {
    settling_time_ns = SLOPE_SETTLING_TIME_NS(lsm6dsm_odr_map[odr_idx].odr);
  } else if(filter == HP_FILTER) {
    settling_time_ns = HPF_SETTLING_TIME_NS((uint64_t)(lsm6dsm_odr_map[odr_idx].odr));
  }
  //filter_settling_time = (samples_to_settle*1000000)/inst_state->common_info.accel_curr_odr;
  //get sample interval -- not possible at sensor level, use minimum 13Hz
  filter_settling_time = sns_convert_ns_to_ticks(settling_time_ns); // in ticks
  cur_time = sns_get_system_time();
  odr_time_elapsed = cur_time - inst_state->common_info.accel_odr_settime;
  req_timeout = (int64_t) (filter_settling_time - odr_time_elapsed);

  DBG_INST_PRINTF(HIGH, this,
                 "filter %d, cur_t %u, set_t %u/%u (ns/ticks)",
                 filter, (uint32_t)cur_time, (uint32_t)settling_time_ns, (uint32_t)filter_settling_time);
  DBG_INST_PRINTF(HIGH, this,
                 "a_odr_set_t %u elapsed %u req_t/o %u",
                 (uint32_t)inst_state->common_info.accel_odr_settime, (uint32_t)odr_time_elapsed,
                 (uint32_t)req_timeout);

  //dump registers
  //lsm6dsm_dump_reg(this, inst_state->fifo_info.fifo_enabled);
  if(req_timeout > 0) {
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = filter_settling_time;
    lsm6dsm_inst_create_timer(this, &inst_state->timer_md_data_stream, &req_payload);
    inst_state->md_info.is_timer_running = true;
    return false;
  } else {
    inst_state->md_info.is_timer_running = false;
    return true;
  }
}

void lsm6dsm_send_md_event(sns_sensor_instance *const instance, sns_motion_detect_event* event, sns_time event_timestamp)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "[MD Event] = %d",
      event->motion_detect_event_type);
  pb_send_event(instance,
      sns_motion_detect_event_fields,
      event,
      event_timestamp,
      SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
      &state->md_info.suid);
}

// to set the correct md filter and enable md interrupt
void lsm6dsm_update_md_filter(sns_sensor_instance *const this)
{
  lsm6dsm_instance_state *inst_state = (lsm6dsm_instance_state*)this->state->state;

  lsm6dsm_md_filter_type filter;
  DBG_INST_PRINTF_EX(HIGH, this, "mdf request: %u %u",
                  lsm6dsm_is_md_int_required(this),
                  inst_state->md_info.cur_state.motion_detect_event_type);
  if(inst_state->md_info.cur_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED) {
    //default is slope filter
    filter = SLOPE_FILTER;
    inst_state->md_info.is_filter_settled = false;
    if(is_lsm6dsm_mdf_settled(this, SLOPE_FILTER)) {
      inst_state->md_info.is_filter_settled = true;
      if(is_lsm6dsm_mdf_settled(this, HP_FILTER)) {
        filter = HP_FILTER;
      }
    }
    if(inst_state->md_info.is_filter_settled) {
      lsm6dsm_read_wake_src(inst_state);
      if((inst_state->md_info.client_present || (inst_state->config_sensors & LSM6DSM_MOTION_DETECT)) && (!inst_state->current_conf.md_enabled))
      {
        lsm6dsm_send_md_event(this, &inst_state->md_info.cur_state, sns_get_system_time());
      }
      lsm6dsm_set_md_intr(this, true);
    }
    lsm6dsm_set_md_filter(this, filter);
  } else {
    // ignore the request

  }
}

void lsm6dsm_set_md_config(sns_sensor_instance *const instance, bool enable)
{
  lsm6dsm_instance_state *state =
    (lsm6dsm_instance_state*)instance->state->state;
  uint8_t r_buffer[2], w_buffer[2];
  uint8_t dur_set = 0x60; // wake up duration bits 5,6 set to 1
  uint8_t thresh_set = 0x3F; // thresh bits 0 to 5 set to 1
  lsm6dsm_accel_odr accel_odr = state->accel_info.desired_odr;
  uint8_t md_odr = (state->desired_conf.odr_idx) ? state->desired_conf.odr_idx : state->min_odr_idx;
  md_odr = (md_odr << 4);
  float md_odr_val = lsm6dsm_get_accel_odr(md_odr);
  float md_thresh;
  float thresh_lsb;

  if(enable && (accel_odr == LSM6DSM_ACCEL_ODR_OFF)) {
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
  thresh_lsb = LSM6DSM_MD_THRESH_MAX/LSM6DSM_MD_COARSE_RES; //2^6
  if(enable && md_thresh <= LSM6DSM_MD_THRESH_MAX && md_thresh >= 0.0f) {
    //with FINE_RES, max can reach FS/4
#if 0
    if(md_thresh < LSM6DSM_MD_THRESH_MAX/4) {
      thresh_lsb = LSM6DSM_MD_THRESH_MAX/LSM6DSM_MD_FINE_RES; //2^8
      dur_set |= 0x10; //WAKE_THS_W bit 4 in WAKE_UP_DUR register
    }
#endif
    if(md_thresh < thresh_lsb)
      md_thresh = thresh_lsb; //make sure min 1 LSB threshold is set
    thresh_set &= ((uint8_t)((lroundf)(md_thresh / thresh_lsb)) | 0xC0);
  }

  if(state->md_info.md_config.win <= 0x3 / md_odr_val
     && state->md_info.md_config.win >= 0.0)
  {
    // Wake Up Duration - bits 5,6 of STM_LSM6DSM_REG_WAKE_DUR
    dur_set &= (((uint8_t)((lroundf)(state->md_info.md_config.win *
                                     md_odr_val)) << 5) | 0x9F);
  }

  DBG_INST_PRINTF(LOW, instance, "th_set=0x%x  dur_set=0x%x",
                  thresh_set, dur_set);

  if(SNS_RC_SUCCESS == lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_WAKE_THS, 2, r_buffer)) {
    w_buffer[0] = (r_buffer[0] & (~0x3F)) | (!enable ? 0x3F : thresh_set);
    w_buffer[1] = (r_buffer[1] & (~0x60)) | dur_set;

    if(*(uint16_t*)r_buffer != *(uint16_t*)w_buffer)
    {
      DBG_INST_PRINTF(LOW, instance, "THS/DUR %02x%02x --> %02x%02x",
                      r_buffer[0], r_buffer[1], w_buffer[0], w_buffer[1]);
      lsm6dsm_write_regs_scp(instance, STM_LSM6DSM_REG_WAKE_THS, 2, w_buffer);
    }
  }

  if(enable && (state->common_info.accel_curr_odr == LSM6DSM_ACCEL_ODR_OFF)) {
    lsm6dsm_set_accel_config(instance,
        accel_odr,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSM_ACCEL_BW50);
  }

  DBG_INST_PRINTF(LOW, instance, "en=%d, cur_odr(a/g)=%x/%x",
                  enable, state->common_info.accel_curr_odr, state->common_info.gyro_curr_odr);
}

void lsm6dsm_update_md_intr(sns_sensor_instance *const instance,
                            bool enable)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if(enable)
  {
    state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_ENABLED;
    lsm6dsm_update_md_filter(instance);
  }
  else
  {
    state->md_info.cur_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
    lsm6dsm_set_md_intr(instance, enable);
    lsm6dsm_set_md_filter(instance, SLOPE_FILTER);
  }

}

/**
 * Changes all gated accel requests to non-gated accel requests.
 *
 * @param instance   Reference to the instance
 *
 * @return None
 */
void lsm6dsm_convert_accel_gated_req_to_non_gated(
   sns_sensor_instance *const instance)
{
  sns_request *request;
  bool req_converted_to_non_gated = false;
  sns_sensor_uid* suid = &((sns_sensor_uid)ACCEL_SUID_0);
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
#if LSM6DSM_DUAL_SENSOR_ENABLED
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
      state->fifo_info.publish_sensors |= LSM6DSM_ACCEL;
      //updating inst_publish too,as this is used to send data
      state->fifo_info.inst_publish_sensors |= LSM6DSM_ACCEL;
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
    lsm6dsm_instance_state *state = (lsm6dsm_instance_state *)instance->state->state;

    if(NULL != event)
    {
      event->message_id = SNS_STD_EVENT_GATED_SENSOR_MSGID_GATED_REQ_CONVERTED_TO_NON_GATED;
      event->event_len = 0;
      event->timestamp = sns_get_system_time();
      e_service->api->publish_event(e_service, instance, event, &state->accel_info.suid);
    }
  }
}

static void lsm6dsm_remove_on_change_requests(sns_sensor_instance *const instance)
{
  sns_request const *md_req;
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
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

void lsm6dsm_disable_md(sns_sensor_instance *const instance, bool send_md_status_event)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "Disable_MD cur state = %d",
      state->md_info.cur_state.motion_detect_event_type);
  lsm6dsm_update_md_intr(instance, false);
  lsm6dsm_set_md_config(instance, false);
  state->md_info.is_timer_running = false;
  if(state->fifo_info.publish_sensors & LSM6DSM_ACCEL)
  {
    lsm6dsm_set_acc_lpmode(instance, false);
  }
  if(send_md_status_event)
  {
    /*TODO: is config event needed for disable*/
    //lsm6dsm_send_config_event(instance, false);
    if(state->md_info.client_present || (state->config_sensors & LSM6DSM_MOTION_DETECT))
    {
      state->md_info.event_ts = sns_get_system_time();
      lsm6dsm_send_md_event(instance, &state->md_info.cur_state, state->md_info.event_ts);
    }
  }
}
void lsm6dsm_enable_md(sns_sensor_instance *const instance, bool send_md_status_event)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  DBG_INST_PRINTF(LOW, instance, "Enable_MD cur state = %d",
      state->md_info.cur_state.motion_detect_event_type);
  if(!(state->fifo_info.publish_sensors & LSM6DSM_ACCEL))
  {
    lsm6dsm_set_acc_lpmode(instance, true);
  }
  lsm6dsm_set_md_config(instance, true);
  lsm6dsm_update_md_intr(instance, true);
  if(send_md_status_event)
  {
    lsm6dsm_send_config_event(instance, false);
  }
}

void lsm6dsm_turn_off_md(sns_sensor_instance *const instance,
                         lsm6dsm_instance_state *state)
{
  DBG_INST_PRINTF(HIGH, instance, "turn_off_md: gated_client=%u",
                  state->accel_info.gated_client_present);
  bool is_md_req = lsm6dsm_is_md_int_required(instance);

  if(!is_md_req) {
    lsm6dsm_disable_md(instance, false);
  }
  //set reconfiguration only if no non-gated clients
  if((state->accel_info.gated_client_present) &&
    (state->fifo_info.fifo_rate == LSM6DSM_ACCEL_ODR_OFF)) {

    //turn off accel before configuring for streaming
    lsm6dsm_set_accel_config(instance,
        LSM6DSM_ACCEL_ODR_OFF,
        state->accel_info.sstvt,
        state->accel_info.range,
        LSM6DSM_ACCEL_BW50);

    lsm6dsm_set_fifo_config(
      instance,
      state->md_info.desired_wmk,
      state->md_info.desired_odr,
      state->common_info.gyro_curr_odr,
      state->fifo_info.fifo_enabled);
    lsm6dsm_set_fifo_wmk(instance);
    lsm6dsm_start_fifo_streaming(instance);
    lsm6dsm_enable_fifo_intr(instance, state->fifo_info.fifo_enabled);
    lsm6dsm_send_config_event(instance, false);
    state->fifo_info.interrupt_cnt = 0;
  }
  lsm6dsm_dump_reg(instance, state->fifo_info.fifo_enabled);
}

uint8_t lsm6dsm_read_wake_src(lsm6dsm_instance_state const *state)
{
  uint8_t buffer = 0;
  uint32_t xfer_bytes;
  lsm6dsm_com_read_wrapper(state->scp_service,
                           state->com_port_info.port_handle,
                           STM_LSM6DSM_REG_WAKE_SRC,
                           &buffer,
                           1,
                           &xfer_bytes);
  return buffer;
}

void lsm6dsm_read_src_regs(lsm6dsm_instance_state const *state, uint8_t src_regs[2])
{
  uint32_t xfer_bytes;
  lsm6dsm_com_read_wrapper(state->scp_service,
                           state->com_port_info.port_handle,
                           STM_LSM6DSM_REG_WAKE_SRC,
                           &src_regs[0],
                           1,
                           &xfer_bytes);
  lsm6dsm_com_read_wrapper(state->scp_service,
                           state->com_port_info.port_handle,
                           STM_LSM6DSM_REG_FUNC_SRC,
                           &src_regs[1],
                           1,
                           &xfer_bytes);
}

bool lsm6dsm_check_md_interrupt(lsm6dsm_instance_state const *state,
                                uint8_t const *wake_src)
{
  uint8_t rw_buffer = 0;
  if(wake_src == NULL)
  {
    rw_buffer = lsm6dsm_read_wake_src(state);
  }
  else
  {
    rw_buffer = *wake_src;
  }

  return (rw_buffer & 0x08) ? true : false;
}

void lsm6dsm_log_interrupt_event(sns_sensor_instance *const instance,
                                 sns_time ts)
{
#if LSM6DSM_LOGGING_DISABLED
  UNUSED_VAR(instance);
  UNUSED_VAR(ts);
#else
  lsm6dsm_instance_state *state =
     (lsm6dsm_instance_state*)instance->state->state;

  sns_diag_service* diag = state->diag_service;
  // Sensor State HW Interrupt Log
  sns_diag_sensor_state_interrupt *log =
    (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
        diag,
        instance,
        &state->md_info.suid,
        sizeof(sns_diag_sensor_state_interrupt),
        SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT);

  if(NULL != log)
  {
    log->interrupt = SNS_DIAG_INTERRUPT_MOTION;
    log->timestamp = ts;

    diag->api->submit_log(diag,
        instance,
        &state->md_info.suid,
        sizeof(sns_diag_sensor_state_interrupt),
        log,
        SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT,
        state->log_interrupt_encoded_size,
        lsm6dsm_encode_sensor_state_log_interrupt);
  }
#endif
}

void lsm6dsm_handle_md_fired(sns_sensor_instance *const instance,
                             sns_time irq_timestamp)
{
  lsm6dsm_instance_state *state =
     (lsm6dsm_instance_state*)instance->state->state;

  /**
   * 1. Handle MD interrupt: Send MD fired event to client.
   * 2. Disable MD.
   * 3. Start Motion Accel FIFO stream with desired config.
   */
  sns_motion_detect_event md_state;
  md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_FIRED;
  DBG_INST_PRINTF(HIGH, instance, "[%u] MD_fired", state->hw_idx);
  lsm6dsm_send_md_event(instance, &md_state, irq_timestamp);
  lsm6dsm_log_interrupt_event(instance, irq_timestamp);

  //since MD fired- clear client_present variable ()
  state->md_info.client_present = false;
  lsm6dsm_convert_accel_gated_req_to_non_gated(instance);

  // remove all on-change requests
  lsm6dsm_remove_on_change_requests(instance);

  lsm6dsm_turn_off_md(instance, state);
  state->num_md_ints++;
}

void lsm6dsm_handle_md_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if(state->md_info.client_present && lsm6dsm_check_md_interrupt(state, wake_src))
  {
    lsm6dsm_handle_md_fired(instance, irq_timestamp);
  }
}


/** See sns_lsm6dsm_hal.h */
void lsm6dsm_send_config_event(sns_sensor_instance *const instance, bool new_client)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  char operating_mode_normal[] = {LSM6DSM_NORMAL};
  char operating_mode_lpm[] = {LSM6DSM_LPM};
  char operating_mode_hpf[] = {LSM6DSM_HIGH_PERF};
  sns_time event_ts = sns_get_system_time();

  sns_std_sensor_physical_config_event phy_sensor_config =
     sns_std_sensor_physical_config_event_init_default;

  pb_buffer_arg op_mode_args;

  if(state->fifo_info.publish_sensors == 0 && !state->md_info.client_present)
  {
    return; // no clients present
  }

  if(!state->accel_info.lp_mode)
  {
    //High Performance mode
    op_mode_args.buf = &operating_mode_hpf[0];
    op_mode_args.buf_len = sizeof(operating_mode_hpf);
  }
  else if(state->common_info.accel_curr_odr > LSM6DSM_ACCEL_ODR52)
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

  if(state->fifo_info.publish_sensors != 0 || state->md_info.client_present)
  {
    if(!new_client)
    {
      state->fifo_info.last_sent_config = state->fifo_info.new_config;
    }
    phy_sensor_config.sample_rate   = state->fifo_info.last_sent_config.sample_rate;
    phy_sensor_config.water_mark    = state->fifo_info.last_sent_config.fifo_watermark;
    event_ts                        = state->fifo_info.last_sent_config.timestamp;
#if LSM6DSM_DAE_ENABLED
    phy_sensor_config.DAE_watermark = state->fifo_info.last_sent_config.dae_watermark;
#endif
  }

  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.has_water_mark = true;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_active_current = true;
  phy_sensor_config.active_current = 240;
  phy_sensor_config.has_resolution = true;
  phy_sensor_config.resolution = (float)(state->accel_info.sstvt)/1000;
  phy_sensor_config.range_count = 2;
  phy_sensor_config.range[0] = lsm6dsm_accel_range_min[state->accel_info.range_idx];
  phy_sensor_config.range[1] = lsm6dsm_accel_range_max[state->accel_info.range_idx];
  phy_sensor_config.has_stream_is_synchronous = true;
  phy_sensor_config.has_DAE_watermark = lsm6dsm_dae_if_available(instance);

  DBG_INST_PRINTF(
    MED, instance, "[%u] config_event:: new=%u sens=0x%x SR=%u WM=%u/%u tempSR=%u ts=%u",
    state->hw_idx, new_client, (state->fifo_info.publish_sensors | state->md_info.client_present),
    (uint32_t)phy_sensor_config.sample_rate,
    phy_sensor_config.water_mark, phy_sensor_config.DAE_watermark,
    (uint32_t)state->sensor_temp_info.desired_sampling_rate_hz, (uint32_t)event_ts);
#if LSM6DSM_AUTO_DEBUG
  SNS_INST_PRINTF(
     HIGH, instance, "send_config_event: pub=0x%x SR*1000=%u WM=%u",
     state->fifo_info.publish_sensors, (uint32_t)(phy_sensor_config.sample_rate*1000),
     phy_sensor_config.water_mark);
#endif
  if(((state->fifo_info.publish_sensors & LSM6DSM_ACCEL) ||
      state->accel_info.gated_client_present))
  {
    float temp_sample_rate = phy_sensor_config.sample_rate;
    sns_time temp_ts       = event_ts;

    if(state->accel_info.gated_client_present 
       && !(state->fifo_info.publish_sensors & LSM6DSM_ACCEL))
    {
      phy_sensor_config.sample_rate = 0.0f;
      event_ts = state->fifo_info.new_config.gated_timestamp;
      phy_sensor_config.has_water_mark = false;
      phy_sensor_config.has_DAE_watermark = false;
    }
    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  event_ts,
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->accel_info.suid);

    phy_sensor_config.sample_rate = temp_sample_rate;
    event_ts = temp_ts;
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.has_DAE_watermark = lsm6dsm_dae_if_available(instance);

    lsm6dsm_send_cal_event(instance, LSM6DSM_ACCEL);
  }

  if(state->fifo_info.publish_sensors & LSM6DSM_GYRO)
  {
    // Override above values with gyro info
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = 1250;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = state->gyro_info.sstvt;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = lsm6dsm_gyro_range_min[state->gyro_info.range_idx];
    phy_sensor_config.range[1] = lsm6dsm_gyro_range_max[state->gyro_info.range_idx];

    pb_send_event(instance,
        sns_std_sensor_physical_config_event_fields,
        &phy_sensor_config,
        event_ts,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
        &state->gyro_info.suid);

    lsm6dsm_send_cal_event(instance, LSM6DSM_GYRO);
  }

  if(state->fifo_info.publish_sensors & LSM6DSM_SENSOR_TEMP)
  {

    if(!new_client)
    {
      state->sensor_temp_info.last_sent_config = state->sensor_temp_info.new_config;
    }
//    event_ts                        = state->sensor_temp_info.last_sent_config.timestamp;
    phy_sensor_config.sample_rate   = state->sensor_temp_info.last_sent_config.sample_rate;
#if LSM6DSM_DAE_ENABLED
    phy_sensor_config.DAE_watermark = state->sensor_temp_info.last_sent_config.dae_watermark;
#endif
    // Override above values with sensor temperature info
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = 240;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = LSM6DSM_SENSOR_TEMPERATURE_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = LSM6DSM_SENSOR_TEMPERATURE_RANGE_MIN;
    phy_sensor_config.range[1] = LSM6DSM_SENSOR_TEMPERATURE_RANGE_MAX;

    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  state->sensor_temp_info.last_sent_config.timestamp,
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->sensor_temp_info.suid);
  }

  if(state->md_info.client_present)
  {
    // Override above values with Motion Detect info
    phy_sensor_config.has_sample_rate = false;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = 24;
    phy_sensor_config.has_resolution = false;
    phy_sensor_config.range_count = 0;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = true;
    phy_sensor_config.has_stream_is_synchronous = true;
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

void lsm6dsm_convert_and_send_temp_sample(
  sns_sensor_instance *const instance,
  sns_time            timestamp,
  const uint8_t       temp_data[2])
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;

  if(state->fifo_info.publish_sensors & LSM6DSM_SENSOR_TEMP)
  {
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    int16_t temp_val = (int16_t)(temp_data[1] << 8 | temp_data[0]);
    float float_temp_val = (temp_val / TEMPERATURE_RESOLUTION) + TEMPERATURE_BASE;
#if !LSM6DSM_LOGGING_DISABLED
    sns_diag_service* diag = state->diag_service;
    log_sensor_state_raw_info log_temp_state_raw_info;

    // Allocate Sensor State Raw log packets for accel and gyro
    sns_memzero(&log_temp_state_raw_info, sizeof(log_temp_state_raw_info));
    log_temp_state_raw_info.encoded_sample_size = state->log_temp_raw_encoded_size;
    log_temp_state_raw_info.diag = diag;
    log_temp_state_raw_info.instance = instance;
    log_temp_state_raw_info.sensor_uid = &state->sensor_temp_info.suid;
    lsm6dsm_log_sensor_state_raw_alloc(&log_temp_state_raw_info, 0);
#endif
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
#if LSM6DSM_AUTO_DEBUG
    SNS_INST_PRINTF(HIGH, instance, "Temp data:  = %d timestamp = %u",
                    (int)float_temp_val, (uint32_t)timestamp);
#endif
#if !LSM6DSM_LOGGING_DISABLED
    // Log raw uncalibrated sensor data
    lsm6dsm_log_sensor_state_raw_add(
        &log_temp_state_raw_info,
        &float_temp_val,
        1,
        timestamp,
        status);
    lsm6dsm_log_sensor_state_raw_submit(&log_temp_state_raw_info, 1, true);
#endif
  }
}

/** See sns_lsm6dsm_hal.h */
void lsm6dsm_handle_sensor_temp_sample(sns_sensor_instance *const instance)
{
  uint8_t temp_data[2] = {0};
  uint8_t buffer;

  lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_STATUS, 1, &buffer);
  lsm6dsm_read_regs_scp(instance, STM_LSM6DSM_REG_OUT_TEMP_L, 2, temp_data);

  lsm6dsm_convert_and_send_temp_sample(instance, sns_get_system_time(), temp_data);
}

/** See sns_lsm6dsm_hal.h */
void lsm6dsm_send_cal_event(sns_sensor_instance *const instance, lsm6dsm_sensor_type sensor_type)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)instance->state->state;
  sns_cal_event new_calibration_event = sns_cal_event_init_default;
  sns_lsm6dsm_registry_cfg *src_cfg = (sensor_type == LSM6DSM_ACCEL) ?
    &state->accel_registry_cfg : &state->gyro_registry_cfg;
  sns_sensor_uid *suid_current = (sensor_type == LSM6DSM_ACCEL) ?
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

void lsm6dsm_start_sensor_temp_polling_timer(sns_sensor_instance *this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
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

void lsm6dsm_turn_off_fifo(sns_sensor_instance *this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  DBG_INST_PRINTF(HIGH, this, "turn_off_fifo: rate=0x%x", state->fifo_info.fifo_rate);
  if(state->fifo_info.fifo_rate > LSM6DSM_ACCEL_ODR_OFF) {
    lsm6dsm_set_fifo_config(this,
        0,
        LSM6DSM_ACCEL_ODR_OFF,
        LSM6DSM_GYRO_ODR_OFF,
        state->fifo_info.fifo_enabled);
    lsm6dsm_stop_fifo_streaming(this);
    lsm6dsm_set_fifo_wmk(this);
    lsm6dsm_disable_fifo_intr(this);
    state->accel_sample_counter = 0;
    state->gyro_sample_counter = 0;
    // Disable timer
    state->health.expected_expiration = UINT64_MAX;
    state->health.heart_beat_timeout = UINT64_MAX/2;
    lsm6dsm_restart_hb_timer(this, true);
    state->health.heart_attack = false;
   }
}
void lsm6dsm_powerdown_hw(sns_sensor_instance *this)
{
  DBG_INST_PRINTF(HIGH, this, "power down sensor");
  lsm6dsm_turn_off_fifo(this);
  lsm6dsm_disable_md(this, false);
  lsm6dsm_set_polling_config(this);
  lsm6dsm_powerdown_esp(this);
}
/**sets fifo odr = 0
 * this retains old data and
 * new data doesn't store */
#if 0
void lsm6dsm_disable_fifo(sns_sensor_instance *this)
{
  uint8_t rw_buffer = 0x06 ;
  uint32_t xfer_bytes = 0;
  lsm6dsm_read_modify_write(this,
      STM_LSM6DSM_REG_FIFO_CTRL5,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      0xFF);
}
#endif

void lsm6dsm_set_fifo_bypass_mode(sns_sensor_instance *this)
{
  uint8_t rw_buffer = 0x0;
  uint32_t xfer_bytes;
  lsm6dsm_read_modify_write(this,
      STM_LSM6DSM_REG_FIFO_CTRL5,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      STM_LSM6DSM_FIFO_MODE_MASK);
}

void lsm6dsm_set_fifo_stream_mode(sns_sensor_instance *this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  uint8_t rw_buffer = LSM6DSM_FIFO_STREAM_MODE | (uint8_t)(state->fifo_info.fifo_rate >> 1);
  uint32_t xfer_bytes;
  lsm6dsm_read_modify_write(this,
      STM_LSM6DSM_REG_FIFO_CTRL5,
      &rw_buffer,
      1,
      &xfer_bytes,
      false,
      STM_LSM6DSM_FIFO_MODE_MASK | STM_LSM6DSM_FIFO_ODR_MASK);
}

// reconfig fifo after flush or without flush
void lsm6dsm_reconfig_fifo(sns_sensor_instance *this, bool flush)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;

  DBG_INST_PRINTF_EX(HIGH, this, "reconfig_fifo En: flush=%u cur0x%x des=0x%x",
                  flush, state->fifo_info.fifo_rate, state->fifo_info.desired_fifo_rate);

  lsm6dsm_disable_fifo_intr(this);

  if(state->fifo_info.fifo_rate > LSM6DSM_ACCEL_ODR_OFF || LSM6DSM_IS_ESP_ENABLED(state)) {
    if(flush)
      lsm6dsm_flush_fifo(this);
    if((state->ascp_req_count <= 0) && state->fifo_info.reconfig_req &&
      (state->desired_conf.odr != state->current_conf.odr || 
      (state->desired_conf.enabled_sensors != state->current_conf.enabled_sensors))) {
      lsm6dsm_set_fifo_bypass_mode(this);
    }
  }

  if((state->ascp_req_count <= 0) && state->fifo_info.reconfig_req) {
#if LSM6DSM_ESP_ENABLED
    if(state->fifo_info.full_reconf_req)
      lsm6dsm_poweron_esp(this);
#endif
    lsm6dsm_set_fifo_wmk(this);
    lsm6dsm_start_fifo_streaming(this);
    if(!lsm6dsm_dae_if_available(this) ||
       state->fifo_info.last_sent_config.fifo_watermark == 0)
    {
      state->fifo_info.new_config.fifo_watermark = state->fifo_info.desired_wmk;
      state->fifo_info.new_config.sample_rate =
        (float)(13 << ((state->fifo_info.desired_fifo_rate >> 4) - 1));
#if LSM6DSM_DAE_ENABLED
      state->fifo_info.new_config.dae_watermark  = state->fifo_info.max_requested_wmk;
#endif
      lsm6dsm_send_config_event(this, false);
    }

#if LSM6DSM_DAE_ENABLED
    state->config_step = LSM6DSM_CONFIG_IDLE; /* done with reconfig */
#endif
    state->fifo_info.inst_publish_sensors = state->fifo_info.publish_sensors;
    state->fifo_info.reconfig_req = false;
    state->fifo_info.full_reconf_req = false;
    lsm6dsm_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
  }

  DBG_INST_PRINTF(HIGH, this, "reconfig_fifo EX: flush=%u cur=0x%x",
                  flush, state->fifo_info.fifo_rate);
}

void lsm6dsm_reconfig_hw(sns_sensor_instance *this)
{
  lsm6dsm_instance_state *state = (lsm6dsm_instance_state*)this->state->state;
  //tells whether fifo int is req or not
  bool is_fifo_int_req =
    (state->fifo_info.publish_sensors & (LSM6DSM_ACCEL | LSM6DSM_GYRO));
  bool is_md_int_req = lsm6dsm_is_md_int_required(this);    //tells whether md int is req or not
  bool is_sensor_active; //tells whether sensor is req or not
  bool md_force_disable = false;
  is_sensor_active = (is_md_int_req ||
                      state->fifo_info.publish_sensors ||
                      LSM6DSM_IS_ESP_DESIRED(state) ||
                      state->accel_info.gated_client_present);

#if LSM6DSM_DAE_ENABLED
  DBG_INST_PRINTF_EX(
    HIGH, this, "reconfig_hw: %u %u %u %u %u",
    is_fifo_int_req, is_md_int_req, is_sensor_active, state->fifo_info.reconfig_req,
    state->config_step);
#else
  DBG_INST_PRINTF_EX(
    HIGH, this, "reconfig_hw: %u %u %u %u",
    is_fifo_int_req, is_md_int_req, is_sensor_active, state->fifo_info.reconfig_req);
#endif

  if(!is_sensor_active) {
    lsm6dsm_powerdown_hw(this);
  } else {
    if(state->fifo_info.reconfig_req && is_fifo_int_req) {
      //need to flush fifo
      //set fifo rate to zero
      //retains old data
      lsm6dsm_reconfig_fifo(this, lsm6dsm_dae_if_available(this) ? false : true);
    } else if(!is_fifo_int_req && (state->fifo_info.fifo_rate > LSM6DSM_ACCEL_ODR_OFF)) {
      lsm6dsm_turn_off_fifo(this);
      //if accel is off, need settling time for md
      if(is_md_int_req && state->current_conf.md_enabled) {
        md_force_disable = true;
        lsm6dsm_disable_md(this, false);
      }

    } else if (is_fifo_int_req && !lsm6dsm_dae_if_available(this)) {
      // Case when reconfig is not required
      lsm6dsm_send_config_event(this, false);
    }
    //avoid trying to configure md if md timer is running
    if(is_md_int_req) {
      // case 1: first MD request, enable
      if(!state->current_conf.md_enabled && !state->md_info.is_timer_running) {
        lsm6dsm_enable_md(this, !md_force_disable);
      } else {
        // case 2: previous MD req is processing or MD enabled, send event
        // only if MD request is configuring
        if((state->config_sensors & LSM6DSM_MOTION_DETECT) && state->md_info.client_present) {
          lsm6dsm_send_config_event(this, false);
          if(state->current_conf.md_enabled)
            lsm6dsm_send_md_event(this, &state->md_info.cur_state, state->md_info.event_ts);
        }
      }
    } else {
      if(state->current_conf.md_enabled || state->md_info.is_timer_running) {
        lsm6dsm_disable_md(this, true);
      } else {
        //case 1: MD client present, non-gated accel req present
        //send disable event
        if((state->config_sensors & LSM6DSM_MOTION_DETECT)
            && state->md_info.client_present) {
          lsm6dsm_send_md_event(this, &state->md_info.cur_state, state->md_info.event_ts);
        }
      }
      state->md_info.is_timer_running = false;
    }

    if (!lsm6dsm_dae_if_available(this))
      state->config_sensors &= ~LSM6DSM_MOTION_DETECT;

    lsm6dsm_poweron_esp(this);

    //if accel is still off, turn on some one need this
    if(state->common_info.accel_curr_odr == LSM6DSM_ACCEL_ODR_OFF) {
      lsm6dsm_accel_odr accel_odr = state->accel_info.desired_odr;
      //Set minimum ODR
      if(state->accel_info.desired_odr == LSM6DSM_ACCEL_ODR_OFF)
      {
        if(LSM6DSM_IS_ESP_ENABLED(state))
        {
          accel_odr = (lsm6dsm_accel_odr)lsm6dsm_get_esp_rate_idx(this);
        }
        else
        {
          accel_odr = lsm6dsm_odr_map[state->min_odr_idx].accel_odr_reg_value;
        }
      }
      lsm6dsm_set_accel_config(this,
          accel_odr,
          state->accel_info.sstvt,
          state->accel_info.range,
          state->accel_info.bw);
    }
    if(state->fifo_info.reconfig_req && !is_fifo_int_req) {
      state->fifo_info.reconfig_req = false;
    }

    DBG_INST_PRINTF(MED, this, "pub=0x%x md=%u",
                    state->fifo_info.publish_sensors, state->md_info.client_present);

    if(state->fifo_info.fifo_enabled != 0 ||
       state->fifo_info.publish_sensors != 0 ||
       lsm6dsm_is_md_int_required(this)) {
#if LSM6DSM_DAE_ENABLED
      if(lsm6dsm_dae_if_available(this)) {
        if(state->fifo_info.publish_sensors || lsm6dsm_is_md_int_required(this)) {
          lsm6dsm_dae_if_start_streaming(this);
        }
      }
      else
#endif
      if((state->fifo_info.publish_sensors & LSM6DSM_SENSOR_TEMP) ||
         (state->sensor_temp_info.timer_is_active)) {
        lsm6dsm_set_polling_config(this);
      }
      lsm6dsm_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
      //overwrite publish sensors only if reconfigure is not required
      if(!state->fifo_info.reconfig_req)
        state->fifo_info.inst_publish_sensors = state->fifo_info.publish_sensors;
    }
  }
#if LSM6DSM_DAE_ENABLED
  DBG_INST_PRINTF(MED, this, "step=%u", state->config_step);
#endif
}

void lsm6dsm_register_interrupt(
    sns_sensor_instance *this,
    lsm6dsm_irq_info* irq_info,
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

