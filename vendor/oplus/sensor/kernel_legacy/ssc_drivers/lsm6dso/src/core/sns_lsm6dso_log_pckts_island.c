/**
 * @file sns_lsm6dso_log_pckts_island.c
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
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_sensor_event.h"
#include "sns_service_manager.h"
#include "sns_types.h"
#include "sns_sensor_util.h"
#include "sns_lsm6dso_sensor_instance.h"
#include "sns_lsm6dso_log_pckts.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_printf.h"
#include "sns_diag.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"

#if !LSM6DSO_LOGGING_DISABLED
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
sns_rc lsm6dso_encode_sensor_state_log_interrupt(
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
sns_rc lsm6dso_encode_log_sensor_state_raw_oneaxis(
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
  lsm6dso_batch_sample_oneaxis *raw_sample = (lsm6dso_batch_sample_oneaxis *)log;

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

    parsed_log_size += sizeof(lsm6dso_batch_sample_oneaxis);
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
sns_rc lsm6dso_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[LSM6DSO_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = LSM6DSO_NUM_AXES,
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
  lsm6dso_batch_sample *raw_sample = (lsm6dso_batch_sample *)log;

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

    parsed_log_size += sizeof(lsm6dso_batch_sample);
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
void lsm6dso_log_sensor_state_raw_alloc(
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
void lsm6dso_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  uint8_t sample_size,
  bool batch_complete)
{
  lsm6dso_batch_sample *sample = NULL;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == log_raw_info->log)
  {
    return;
  }

  sample = (lsm6dso_batch_sample *)log_raw_info->log;

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
        lsm6dso_encode_log_sensor_state_raw_oneaxis);
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
        lsm6dso_encode_log_sensor_state_raw);
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
sns_rc lsm6dso_log_sensor_state_raw_add(
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

  uint8_t batch_sample_size = sizeof(lsm6dso_batch_sample);
  if(sample_size == 1)
    batch_sample_size = sizeof(lsm6dso_batch_sample_oneaxis);

  if( (log_raw_info->bytes_written + batch_sample_size) >
     log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    lsm6dso_log_sensor_state_raw_submit(log_raw_info, sample_size, false);
    lsm6dso_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    if(sample_size == 1) {
      lsm6dso_batch_sample_oneaxis *sample_oneaxis =
        (lsm6dso_batch_sample_oneaxis *)log_raw_info->log;
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
    lsm6dso_batch_sample *sample =
        (lsm6dso_batch_sample *)log_raw_info->log;
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
void lsm6dso_init_raw_log(
    sns_sensor_instance* instance,
    log_sensor_state_raw_info* log_state_raw_info,
    size_t encoded_size,
    sns_sensor_uid* suid)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state *)instance->state->state;
// Allocate Sensor State Raw log packets for accel and gyro
  sns_memzero(log_state_raw_info, sizeof(log_sensor_state_raw_info));
  log_state_raw_info->encoded_sample_size = encoded_size;
  log_state_raw_info->diag = state->diag_service;
  log_state_raw_info->instance = instance;
  log_state_raw_info->sensor_uid = suid;
  log_state_raw_info->log = NULL;
}

void lsm6dso_log_sample(sns_sensor_instance *const instance,
    log_sensor_state_raw_info* raw_log_ptr,
    sns_sensor_uid* suid,
    size_t encoded_size,
    float* raw_data,
    uint8_t num_axis,
    sns_time ts,
    sns_std_sensor_sample_status status,
    sns_diag_batch_sample_type packet_type)
{
  if(!raw_log_ptr)
    return;
  if(!raw_log_ptr->log && raw_data) {
    lsm6dso_init_raw_log(instance, raw_log_ptr, encoded_size, suid);
    lsm6dso_log_sensor_state_raw_alloc(raw_log_ptr, 0);
  }
  if(raw_data) {
    // Log raw uncalibrated sensor data
    lsm6dso_log_sensor_state_raw_add(
        (log_sensor_state_raw_info *)raw_log_ptr,
        raw_data,
        num_axis,
        ts,
        status);
  }
  
  if((packet_type == SNS_DIAG_BATCH_SAMPLE_TYPE_LAST) || 
    (packet_type == SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY)) {
    if(!raw_log_ptr->log)
      return;

    lsm6dso_log_sensor_state_raw_submit(raw_log_ptr, num_axis, true);
    return;
  }
}

void lsm6dso_log_interrupt_event(sns_sensor_instance *const instance,
    sns_sensor_uid* suid,
    sns_time ts)
{
  lsm6dso_instance_state *state =
     (lsm6dso_instance_state*)instance->state->state;

  sns_diag_service* diag = state->diag_service;
  // Sensor State HW Interrupt Log
  sns_diag_sensor_state_interrupt *log =
    (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
        diag,
        instance,
        suid,
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
        lsm6dso_encode_sensor_state_log_interrupt);
  }
}

void lsm6dso_init_raw_log_info(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[LSM6DSO_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = LSM6DSO_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));

  /** Determine sizes of encoded logs */
  sns_diag_sensor_state_interrupt sensor_state_interrupt =
    sns_diag_sensor_state_interrupt_init_default;
  pb_get_encoded_size(&state->log_interrupt_encoded_size,
      sns_diag_sensor_state_interrupt_fields,
      &sensor_state_interrupt);

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
}

#else

void lsm6dso_log_interrupt_event(sns_sensor_instance *const instance,
    sns_sensor_uid* suid,
    sns_time ts)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(suid);
  UNUSED_VAR(ts);
}
void lsm6dso_log_sample(sns_sensor_instance *const instance,
    log_sensor_state_raw_info* raw_log_ptr,
    sns_sensor_uid* suid,
    size_t encoded_size,
    float* raw_data,
    uint8_t num_axis,
    sns_time ts,
    sns_std_sensor_sample_status status,
    sns_diag_batch_sample_type packet_type)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(raw_log_ptr);
  UNUSED_VAR(suid);
  UNUSED_VAR(encoded_size);
  UNUSED_VAR(num_axis);
  UNUSED_VAR(raw_data);
  UNUSED_VAR(ts);
  UNUSED_VAR(status);
  UNUSED_VAR(packet_type);
}

void lsm6dso_init_raw_log_info(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
}
#endif
