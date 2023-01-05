/**
 * @file sns_ak0991x_hal_island.c
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"
#include "sns_island_service.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"

#include "sns_cal_util.h"
#include "sns_cal.pb.h"
#include "sns_sensor_util.h"

#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_fb_utils.h"
#endif


//#define AK0991X_VERBOSE_DEBUG             // Define to enable extra debugging

/** Need to use ODR table. */
extern const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE];

log_sensor_state_raw_info log_mag_state_raw_info;

/**
 * Encode log sensor state raw packet
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the logz
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc ak0991x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[AK0991X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = AK0991X_NUM_AXES,
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
  ak0991x_batch_sample *raw_sample = (ak0991x_batch_sample *)log;

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

    parsed_log_size += sizeof(ak0991x_batch_sample);
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
void ak0991x_log_sensor_state_raw_alloc(
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
void ak0991x_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  bool batch_complete)
{
  ak0991x_batch_sample *sample = NULL;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == log_raw_info->log)
  {
    return;
  }

  sample = (ak0991x_batch_sample *)log_raw_info->log;

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
        ak0991x_encode_log_sensor_state_raw);

  log_raw_info->log = NULL;
}

/**
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
sns_rc ak0991x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
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

  if( (log_raw_info->bytes_written + sizeof(ak0991x_batch_sample)) >
     log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    ak0991x_log_sensor_state_raw_submit(log_raw_info, false);
    ak0991x_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    ak0991x_batch_sample *sample =
        (ak0991x_batch_sample *)log_raw_info->log;

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

    log_raw_info->bytes_written += sizeof(ak0991x_batch_sample);

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
static sns_rc ak0991x_com_read_wrapper(sns_sync_com_port_service * scp_service,
                                       sns_sync_com_port_handle *port_handle,
                                       uint32_t reg_addr,
                                       uint8_t *buffer,
                                       uint32_t bytes,
                                       uint32_t *xfer_bytes)
{
  sns_rc rc;
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = false;
  port_vec.reg_addr = reg_addr;

  rc = scp_service->api->sns_scp_register_rw(port_handle,
                                               &port_vec,
                                               1,
                                               false,
                                               xfer_bytes);
  #ifdef OPLUS_FEATURE_SENSOR_FB
  struct fb_event fb_event;
  memset(&fb_event, 0, sizeof(struct fb_event));
  if (rc != SNS_RC_SUCCESS || *xfer_bytes != bytes) {
	  fb_event.event_id = MAG_I2C_ERR_ID;
	  fb_event.buff[0] = 0;//read
	  fb_event.buff[1] = (int)reg_addr;
	  oplus_add_fd_event(&fb_event);
  }
  #endif
  return rc;
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
sns_rc ak0991x_com_write_wrapper(sns_sensor_instance *const this,
                                 sns_sync_com_port_service * scp_service,
                                 sns_sync_com_port_handle *port_handle,
                                 uint32_t reg_addr,
                                 uint8_t *buffer,
                                 uint32_t bytes,
                                 uint32_t *xfer_bytes,
                                 bool save_write_time )
{
  sns_rc rc;
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = true;
  port_vec.reg_addr = reg_addr;

#ifdef AK0991X_VERBOSE_DEBUG
  if( this )
  {
    ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
    sns_diag_service *diag = state->diag_service;
    if( diag )
    {
      AK0991X_INST_PRINT(LOW, this,
                         "ak0991x write reg:0x%x val[0]:0x%x, bytes %d",
                      reg_addr, (uint32_t) buffer[0], bytes);
    }
  }
#else
  UNUSED_VAR( this );
#endif
  rc = scp_service->api->sns_scp_register_rw(port_handle,
                                               &port_vec,
                                               1,
                                               save_write_time,
                                               xfer_bytes);
  #ifdef OPLUS_FEATURE_SENSOR_FB
  struct fb_event fb_event;
  memset(&fb_event, 0, sizeof(struct fb_event));
  if (rc != SNS_RC_SUCCESS || *xfer_bytes != bytes) {
	  fb_event.event_id = MAG_I2C_ERR_ID;
	  fb_event.buff[0] = 1;//write
	  fb_event.buff[1] = (int)reg_addr;
	  oplus_add_fd_event(&fb_event);
  }
  #endif
  return rc;
}

void ak0991x_clear_old_events(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)instance->state->state;
  sns_sensor_event    *event;

  // Handle interrupts
  if (NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);

    while (NULL != event && SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
    {
      AK0991X_INST_PRINT(LOW, instance, "Old interrupt event detected. Cleared.");
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
  }

  // Handle Async Com Port events
  if (NULL != state->async_com_port_data_stream)
  {
    event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);

    while (NULL != event && SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
    {
      AK0991X_INST_PRINT(LOW, instance, "Old ASCP event detected. Cleared.");
      event = state->async_com_port_data_stream->api->get_next_input(
          state->async_com_port_data_stream);
    }
  }

  // Handle timer event
  if (NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);

    while (NULL != event && SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
    {
      AK0991X_INST_PRINT(LOW, instance, "Old timer event detected. Cleared.");
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_enter_i3c_mode(sns_sensor_instance *const instance,
                              ak0991x_com_port_info *com_port,
                              sns_sync_com_port_service * scp_service)
{
  sns_rc                       rv = SNS_RC_FAILED;
#ifdef AK0991X_ENABLE_I3C_SUPPORT
  sns_sync_com_port_handle    *i2c_port_handle = NULL;
  sns_com_port_config          i2c_com_config = com_port->com_config;
  uint32_t                     xfer_bytes;
  uint8_t                      buffer[6];
  bool                         enable_ibi = false;

  if(com_port->com_config.bus_type != SNS_BUS_I3C_SDR &&
     com_port->com_config.bus_type != SNS_BUS_I3C_HDR_DDR )
  {
    return SNS_RC_SUCCESS;
  }

  if(NULL != instance)
  {
    AK0991X_INST_PRINT(LOW, instance, "enter i3c mode");
  }
  if(com_port->in_i3c_mode)
  {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(MED, instance, "already in i3c mode");
    }
    return SNS_RC_SUCCESS;
  }

  i2c_com_config.slave_control = com_port->i2c_address;
  rv = scp_service->api->sns_scp_register_com_port(&i2c_com_config, &i2c_port_handle);

  if( rv != SNS_RC_SUCCESS )
  {
    if (NULL != instance)
    {
      SNS_INST_PRINTF(ERROR, instance, "i3c_mode: register_com_port() rv=%d", rv);
    }
    return SNS_RC_FAILED;
  }
  rv = scp_service->api->sns_scp_open(i2c_port_handle);

  if( rv != SNS_RC_SUCCESS )
  {
    if (NULL != instance)
    {
      SNS_INST_PRINTF(ERROR, instance, "i3c_mode: open() rv=%d", rv);
    }
    return SNS_RC_FAILED;
  }
  /**-------------------Assign I3C dynamic address------------------------*/
  buffer[0] = (com_port->i3c_address & 0xFF)<<1;
  rv = scp_service->api->
    sns_scp_issue_ccc( i2c_port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETDASA,
                       buffer, 1, &xfer_bytes );
  scp_service->api->sns_scp_close(i2c_port_handle);
  scp_service->api->sns_scp_deregister_com_port(&i2c_port_handle);
  com_port->in_i3c_mode = true;

  if(NULL != instance)
  {
    if(rv == SNS_RC_SUCCESS)
    {
      SNS_INST_PRINTF(HIGH, instance, "I3C address assigned: 0x%x",((uint32_t)buffer[0])>>1);
    }
    else
    {
      SNS_INST_PRINTF(HIGH, instance, "Assign I3C address failed; rv=%d", rv);
    }
  }

  /**-------------Set max read size to the size of the FIFO------------------*/
  if(com_port->port_handle != NULL)
  {
    buffer[0] = (uint8_t)((AK0991X_MAX_FIFO_SIZE >> 8) & 0xFF);
    buffer[1] = (uint8_t)(AK0991X_MAX_FIFO_SIZE & 0xFF);
    buffer[2] = 0;
    rv = scp_service->api->
      sns_scp_issue_ccc( com_port->port_handle,
                         SNS_SYNC_COM_PORT_CCC_SETMRL,
                         buffer, 3, &xfer_bytes );
    if( rv != SNS_RC_SUCCESS ) {
      if(NULL != instance)
      {
        SNS_INST_PRINTF(ERROR, instance, "Set max read length failed! rv=%d hndl=0x%x", 
                        rv, com_port->port_handle);
      }
      com_port->in_i3c_mode = false;
    }

    /**-------------------Enable/Disable IBI------------------------*/
    if(NULL != instance)
    {
      ak0991x_instance_state *state = (ak0991x_instance_state *)(instance->state->state);
      if (state->mag_info.int_mode == AK0991X_INT_OP_MODE_IBI)
      {
        enable_ibi = true;
      }
    }

    if( enable_ibi )
    {
      buffer[0] = 0x1;
      rv = scp_service->api->
        sns_scp_issue_ccc( com_port->port_handle,
                           SNS_SYNC_COM_PORT_CCC_ENEC,
                           buffer, 1, &xfer_bytes );
      if( rv == SNS_RC_SUCCESS ) {
        if(NULL != instance)
        {
          SNS_INST_PRINTF(HIGH, instance, "IBI enabled");
        }
      } else {
        if(NULL != instance)
        {
          SNS_INST_PRINTF(ERROR, instance, "Enable IBI  FAILED! rv=%d hndl=0x%x",
              rv, com_port->port_handle);
        }
        com_port->in_i3c_mode = false;
      }
    }
    else
    {
      buffer[0] = 0x1;
      rv = scp_service->api->
        sns_scp_issue_ccc( com_port->port_handle,
                           SNS_SYNC_COM_PORT_CCC_DISEC,
                           buffer, 1, &xfer_bytes );
      if( rv == SNS_RC_SUCCESS ) {
        if(NULL != instance)
        {
          AK0991X_INST_PRINT(HIGH, instance, "IBI disabled");
        }
      } else {
        if(NULL != instance)
        {
            SNS_INST_PRINTF(ERROR, instance, "Disable IBI FAILED! rv=%d hndl=0x%x",
                        rv, com_port->port_handle);
        }
        com_port->in_i3c_mode = false;
      }
    }
  }

#ifdef AK0991X_ENABLE_I3C_DEBUG
  /**-------------------Debug -- read all CCC info------------------------*/
  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMWL,
                       buffer, 2, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "max write length:0x%02x%02x", buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get max write length failed!");
    }
  }
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETMWL,
                       buffer, 2, &xfer_bytes );

  if( rv != SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Set max write length failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMRL,
                       buffer, 2, &xfer_bytes );

  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "max read length:0x%02x%02x",
                      buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get max read length failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETPID,
                       buffer, 6, &xfer_bytes );

  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "PID:0x%02x%02x:%02x%02x:%02x%02x",
                      buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get PID failed!");
    }
  }
    
  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETBCR,
                       buffer, 1, &xfer_bytes );

  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "bus charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get BCR failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETDCR,
                       buffer, 1, &xfer_bytes );

  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "device charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get DCR failed!");
    }
  }
    
  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETSTATUS,
                       buffer, 2, &xfer_bytes );

  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(LOW, instance, "status register:0x%x", (buffer[0] | buffer[1] << 8));
    }
  } else {
    if(NULL != instance)
    {
      AK0991X_INST_PRINT(ERROR, instance, "Get status failed!");
    }
  }
    
#  endif /*  AK0991X_ENABLE_DEBUG_MSG */
#else /* AK0991X_ENABLE_I3C_SUPPORT */
  UNUSED_VAR(instance);
  UNUSED_VAR(com_port);
  UNUSED_VAR(scp_service);
#endif /* AK0991X_ENABLE_I3C_SUPPORT */
  return rv;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_device_sw_reset(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               ak0991x_com_port_info *com_port)
{
  uint8_t  buffer[1];
  int8_t   num_attempts = 5;
  sns_rc   rv = SNS_RC_FAILED;
#ifdef AK0991X_ENABLE_I3C_SUPPORT
  sns_rc   rv_enter_i3c = SNS_RC_FAILED;
#endif
  uint32_t xfer_bytes;

  if(this != NULL)
  {
    AK0991X_INST_PRINT(LOW, this, "device_sw_reset called");
  }

  // clear old events
  if(this != NULL)
  {
    ak0991x_clear_old_events(this);
  }

  while(num_attempts-- > 0 && SNS_RC_SUCCESS != rv)
  {
    buffer[0] = AK0991X_SOFT_RESET;
    rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
                                 com_port->port_handle,
                                 AKM_AK0991X_REG_CNTL3,
                                 &buffer[0],
                                 1,
                                 &xfer_bytes,
                                 false);
    if( (SNS_RC_SUCCESS != rv) || (xfer_bytes != 1))
    {
      if (xfer_bytes != 1)
      {
        rv = SNS_RC_FAILED;
      }
      if(this != NULL)
      {
        SNS_INST_PRINTF(ERROR, this, "device_sw_reset failed rc=%d, xfer_bytes=%d", rv, xfer_bytes);
      }
      sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
    }
    else
    {
      if(this!= NULL)
      {
         AK0991X_INST_PRINT(LOW, this, "device_sw_reset sucessful");
      }
    }
  }
  
  com_port->in_i3c_mode = false;

#ifdef AK0991X_ENABLE_I3C_SUPPORT
  if(com_port->com_config.bus_type != SNS_BUS_I3C_SDR &&
     com_port->com_config.bus_type != SNS_BUS_I3C_HDR_DDR )
  {
    return rv;
  }

  if(num_attempts <= 0)
  {
    if(this != NULL)
    {
      SNS_INST_PRINTF(ERROR, this, "sw_rst: failed all attempts");
      rv = SNS_RC_FAILED;
    }
  }
  else
  {
    num_attempts = 5;
    while(num_attempts-- > 0 && SNS_RC_SUCCESS != rv_enter_i3c)
    {
      rv_enter_i3c = ak0991x_enter_i3c_mode(this, com_port, scp_service);
      if(SNS_RC_SUCCESS != rv_enter_i3c)
      {
        sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
      }
    }
    if(num_attempts <= 0)
    {
      if(this != NULL)
      {
        SNS_INST_PRINTF(HIGH, this, "sw_rst: enter i3c failed all attempts");
      }
    }
  } 
#endif
  return rv;
}

/**
 * Sets Mag ODR, range and sensitivity.
 *
 * @param[i] this        Current instance
 * @param[i] force_off   Turn device off rather than use instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static
sns_rc ak0991x_set_mag_config(sns_sensor_instance *const this,
                              bool force_off )
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_sync_com_port_service * scp_service = state->scp_service;
  sns_sync_com_port_handle *port_handle = state->com_port_info.port_handle;
  uint32_t xfer_bytes;
  uint8_t  buffer[2] = {0};
  ak0991x_mag_odr desired_odr = state->mag_info.cur_cfg.odr;
  sns_rc ret;

  if( force_off )
  {
    desired_odr = AK0991X_MAG_ODR_OFF;
  }

#ifdef AK0991X_ENABLE_S4S_TEST
  desired_odr = AK0991X_MAG_ODR10;
#endif

  AK0991X_INST_PRINT(LOW, this, "set_mag_config: ODR: %d dev:%d wmk:%d",
                                desired_odr, state->mag_info.device_select,
                                state->mag_info.cur_cfg.fifo_wmk );

  sns_rc rv = SNS_RC_SUCCESS;
  // Set mag config for S4S
  rv = ak0991x_s4s_set_mag_config(this);
  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  akm_device_type device_select = state->mag_info.device_select;
  uint16_t reg_wmk = state->mag_info.cur_cfg.fifo_wmk - 1; // register FIFO value 0 means WM=1

  if(((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917)) &&
     state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && !state->mag_info.use_sync_stream)
  {
    reg_wmk = 0x1F; // signals DAE driver
  }
  // Configure control register 1
  if ((device_select == AK09912) || (device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    if (device_select == AK09912)
    {
      buffer[0] = 0x0
        | (state->mag_info.nsf << 5); // NSF bit
    }
    else
    {
      buffer[0] = 0x0
        | (state->mag_info.nsf << 5) // NSF bit
        | reg_wmk;                   // WM[4:0] bits
    }
  }

  // Configure control register 2
  if ((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    uint8_t enable_fifo = (force_off)? 0 : (uint8_t)state->mag_info.use_fifo;
    if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && !state->mag_info.use_sync_stream)
    {
      enable_fifo = 1; // FIFO always ON when Polling to prevent duplicate samples
    }
    buffer[1] = 0x0
      | (enable_fifo << 7) // FIFO bit
      | (state->mag_info.sdr << 6)               // SDR bit
      | (uint8_t)desired_odr;                    // MODE[4:0] bits
  }
  else
  {
    buffer[1] = 0x0
      | (uint8_t)desired_odr; // MODE[4:0] bits
  }

  // Force 100Hz DRI measurement starts to get the clock error.
  if(!force_off)
  {
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING &&
       (state->mag_info.clock_error_meas_count == 0) &&
       !state->in_clock_error_procedure)
    {
      buffer[0] = 0x0;
      buffer[1] = 0x0
          | (state->mag_info.sdr << 6)               // SDR bit
          | (uint8_t)AK0991X_MAG_ODR100;             // MODE[4:0] bits

      /* Enable FIFO for AK09917D RevA/B bug */
      if(device_select == AK09917)
      {
        buffer[1] |= AK0991X_FIFO_BIT;
      }

      AK0991X_INST_PRINT(LOW, this, "100Hz dummy measurement start. Now in_clk_err_prcdr is TRUE");

      state->in_clock_error_procedure = true;
    }
    else
    {
      AK0991X_INST_PRINT(LOW, this, "Real measurement start at %u"
        ,(uint32_t)sns_get_system_time());
    }
  }

  AK0991X_INST_PRINT(LOW, this, "CNTL1=0x%02X CNTL2=0x%02X", buffer[0], buffer[1]);

  ret = ak0991x_com_write_wrapper(this,
                                   scp_service,
                                   port_handle,
                                   AKM_AK0991X_REG_CNTL1,
                                   buffer,
                                   2,
                                   &xfer_bytes,
                                   false);

  if(!force_off)
  {
    ak0991x_reset_mag_parameters(this);
  }

  return ret;
}

static sns_rc ak0991x_set_timer_request_payload(sns_sensor_instance *const this);



static sns_time ak0991x_set_heart_beat_timeout_period_for_polling(
    sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_time sample_period;
  sns_time timeout_period;

  sample_period = ak0991x_get_sample_interval(state->mag_info.cur_cfg.odr);

  if (state->mag_info.use_fifo)
  {
    timeout_period = sample_period * (state->mag_info.cur_cfg.fifo_wmk);
  }
  else
  {
    timeout_period = sample_period;
  }

  // Set heart_beat_timeout_period for heart beat in Polling/FIFO+Polling
  // as 5 samples time for Polling plus 1 sample time for jitter
  // or 2 FIFO buffers time for FIFO+Polling plus 2 sample time for jitter

  // Set heart_beat_timeout_period for heart beat in S4S/FIFO+S4S
  // as 5 samples time for S4S plus 1 sample time for jitter
  // or 2 FIFO buffers time for FIFO+S4S plus 2 sample time for jitter

  state->heart_beat_timeout_period =
    (state->mag_info.use_fifo)? timeout_period * 2 + sample_period * 2
    : sample_period * 5 + sample_period;

  AK0991X_INST_PRINT(LOW, this, "calculated heart_beat_timeout_period = %u %u %u",
      (uint32_t)state->heart_beat_timeout_period,
      (uint32_t)timeout_period,
      (uint32_t)sample_period);

  return timeout_period;
}


/**
 * see sns_ak0991x_hal.h
 */
void ak0991x_reset_averaged_interval(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_time meas_usec;
  ak0991x_get_meas_time(state->mag_info.device_select, state->mag_info.sdr, &meas_usec);
  state->half_measurement_time = ((sns_convert_ns_to_ticks(meas_usec * 1000) * state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION)>>1;
  if( state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF &&
      state->internal_clock_error != 0)
  {
    state->nominal_intvl = ak0991x_get_sample_interval(state->mag_info.cur_cfg.odr);
    state->averaged_interval = (state->nominal_intvl * state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION;
  }
}

void ak0991x_reset_mag_parameters(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);

  state->this_is_first_data = true;
  state->mag_info.data_count_for_dri = 0;
  state->s4s_reg_event_done = false;
  state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
  state->heart_beat_sample_count = 0;
  state->system_time = sns_get_system_time();
  state->heart_beat_timestamp = state->system_time;

  if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
  {
    state->pre_timestamp = state->system_time + (state->half_measurement_time<<1) - state->averaged_interval;
  }
  else
  {
    state->pre_timestamp = state->system_time;
  }
  state->irq_event_time = state->pre_timestamp;
  state->previous_irq_time = state->pre_timestamp;

  AK0991X_INST_PRINT(LOW, this, "reset pre_timestamp %u pre_orphan %u",
      (uint32_t)state->pre_timestamp,
      (uint32_t)state->pre_timestamp_for_orphan);
}

sns_rc ak0991x_start_mag_streaming(sns_sensor_instance *const this )
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_rc rv;

  AK0991X_INST_PRINT(LOW, this, "ak0991x_start_mag_streaming.");

  // Enable Mag Streaming

  //Transit to Power-down mode first and then transit to other modes.
  rv = ak0991x_set_mag_config(this, true);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // Wait at least 100usec after power down-mode before setting another mode.
  sns_busy_wait(sns_convert_ns_to_ticks(AK0991X_TWAIT_USEC * 1000 * 2));

  rv = ak0991x_set_mag_config(this, false);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  AK0991X_INST_PRINT(HIGH, this, "start_mag_streaming at %u pre_ts %u avg %u half %u", 
                     (uint32_t)sns_get_system_time(), (uint32_t)state->pre_timestamp,
                     (uint32_t)state->averaged_interval, (uint32_t)state->half_measurement_time);

  // QC - is it not possible to miss any of the interrupts during dummy measurement?
  if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
  {
    ak0991x_register_heart_beat_timer(this);
  }
  else
  {
    if( ak0991x_dae_if_available(this) && !state->mag_info.flush_only && !state->mag_info.max_batch)
    {
      ak0991x_set_heart_beat_timeout_period_for_polling(this);
    }
  }
  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_stop_mag_streaming(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_rc rv;

  // Disable Mag Streaming
  AK0991X_INST_PRINT(LOW, this, "ak0991x_stop_mag_streaming");

  if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
  {
    ak0991x_unregister_heart_beat_timer(this);
  }

  rv = ak0991x_set_mag_config(this, true);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(scp_service,
                                port_handle,
                                AKM_AK0991X_REG_WIA1,
                                buffer,
                                AK0991X_NUM_READ_DEV_ID,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_READ_DEV_ID)
  {
    #ifdef OPLUS_FEATURE_SENSOR_FB
    struct fb_event fb_event;
    memset(&fb_event, 0, sizeof(struct fb_event));
    fb_event.event_id = MAG_INIT_FAIL_ID;
    fb_event.buff[0] = (int) xfer_bytes;
    fb_event.buff[1] = (int) AK0991X_NUM_READ_DEV_ID;
    oplus_add_fd_event(&fb_event);
    #endif
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Read asa value.
 *
 * @param[i] port_handle              handle to synch COM port
 * @param[o] buffer                   fifo data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_asa(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle,
                               uint8_t *asa)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint8_t  buffer[1];
  uint32_t xfer_bytes;

  buffer[0] = AK0991X_MAG_FUSEROM;
  // Set Fuse ROM access mode
  rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    // transit to power-down mode from Fuse ROM access mode if possible
    buffer[0] = AK0991X_MAG_ODR_OFF;
    ak0991x_com_write_wrapper(this,
                              scp_service,
                              port_handle,
                              AKM_AK0991X_REG_CNTL2,
                              buffer,
                              1,
                              &xfer_bytes,
                              false);
    return rv;
  }


  // Read Fuse ROM
  rv = ak0991x_com_read_wrapper(scp_service,
                                port_handle,
                                AKM_AK0991X_FUSE_ASAX,
                                &asa[0],
                                AK0991X_NUM_SENSITIVITY,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_SENSITIVITY)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  buffer[0] = AK0991X_MAG_ODR_OFF;
  // Set power-down mode
  rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * check threshold.
 *
 * @param[i] testno                   test number
 * @param[i] testdata                 test data
 * @param[i] lolimit                  low limit
 * @param[i] hilimit                  high limit
 * @param[o] err                      error code
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_test_threshold(uint16_t testno,
                              int16_t testdata,
                              int16_t lolimit,
                              int16_t hilimit,
                              uint32_t *err)
{
  if ((lolimit <= testdata) && (testdata <= hilimit))
  {
    return SNS_RC_SUCCESS;
  }
  else
  {
    *err = (uint32_t)((((uint32_t)testno) << 16) | ((uint16_t)testdata));
    return SNS_RC_FAILED;
  }
}

#define AKM_FST(no, data, lo, hi, err) \
  if (ak0991x_test_threshold((no), (data), (lo), (hi), (err)) \
      != SNS_RC_SUCCESS) { goto TEST_SEQUENCE_FAILED; }

/**
 * Convert buffered data to mag data.
 *
 * @param[i] this              reference to the instance
 * @param[i] buffer            mag register data
 * @param[o] out               output 16bit mag data
 *
 */
static void ak0991x_get_adjusted_mag_data(sns_sensor_instance *const this, uint8_t *const buffer, int16_t *out)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  for(int i=0; i<AK0991X_NUM_AXES; i++)
  {
    if (state->mag_info.device_select == AK09917)
    {
      out[i] = (int16_t)((((int16_t)buffer[i*2] << 8) & 0xFF00) | (int16_t)buffer[i*2 + 1]);
    }
    else
    {
      out[i] = (int16_t)((((int16_t)buffer[i*2 + 1] << 8) & 0xFF00) | (int16_t)buffer[i*2]);
    }
    // sensitivity adjustment
    out[i] = (int16_t)(out[i] * state->mag_info.sstvt_adj[i]);
  }
}

/**
 * Read ST1(10h) register data.
 *
 * @param[i] state                    Instance state
 * @param[o] buffer                   st1 register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_st1(ak0991x_instance_state *state,
                               uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST1,
                                buffer,
                                1,
                                &xfer_bytes);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}


/**
 * Read ST2(18h) register data.
 *
 * @param[i] state                    Instance state
 * @param[o] buffer                   st2 register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_st2(ak0991x_instance_state *state,
                               uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST2,
                                buffer,
                                1,
                                &xfer_bytes);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Read ST1(10h) to ST2(18h) register data.
 *
 * @param[i] state                    Instance state
 * @param[o] buffer                   register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_st1_st2(ak0991x_instance_state *state,
                                   uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  // ST1 read
  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST1,
                                buffer,
                                1,
                                &xfer_bytes);
  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  // HXL to ST2
  rv |= ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_HXL,
                                &buffer[1],
                                (uint32_t)(AK0991X_NUM_DATA_HXL_TO_ST2),
                                &xfer_bytes);

  if (xfer_bytes != (uint32_t)(AK0991X_NUM_DATA_HXL_TO_ST2))
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Read HXL(11h) to ST2(18h) register data.
 *
 * @param[i] state                    Instance state
 * @param[i] num_samples              number of samples
 * @param[o] buffer                   register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_hxl_st2(ak0991x_instance_state *state,
                                   uint16_t num_samples,
                                   uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_HXL,
                                buffer,
                                (uint32_t)(num_samples * AK0991X_NUM_DATA_HXL_TO_ST2),
                                &xfer_bytes);

  if (xfer_bytes != (uint32_t)(num_samples * AK0991X_NUM_DATA_HXL_TO_ST2))
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Wait DRDY bit is ready
 * @param[i] this              reference to the instance
 * @param[i] timeout_ms        Waiting time for drdy
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_wait_drdy_poll(sns_sensor_instance *const this,
                                     int32_t timeout_ms)
{
  sns_rc  rv;
  int16_t i;
  uint8_t st1_buf;
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  for (i = 0; i < timeout_ms; i++)
  {
    /* check DRDY */
    rv = ak0991x_read_st1(state, &st1_buf);  // read ST1
    if (rv != SNS_RC_SUCCESS)
    {
      return rv;
    }

    state->data_is_ready = (st1_buf & AK0991X_DRDY_BIT) ? true : false; // check DRDY bit
    if (state->data_is_ready)
    {
      /* OK, DRDY bit is high */
      return SNS_RC_SUCCESS;
    }

    /* DRDY bit is still LOW, wait for a while and read again */
    sns_busy_wait(sns_convert_ns_to_ticks(1 * 1000 * 1000)); // Wait 1ms
  }

  return SNS_RC_FAILED;
}

/**
 * Run a hardware self-test
 * @param[i]            reference to the instance
 * @param[o]            error code
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_hw_self_test(sns_sensor_instance *const this,
                            uint32_t *err)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  sns_time usec_time_for_measure;
  uint8_t  asa[AK0991X_NUM_SENSITIVITY];
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int16_t  data[3];
  uint8_t  sdr = 0;
  int      i;
  
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  // Initialize error code
  *err = 0;

  // Reset device
  rv = ak0991x_device_sw_reset(this,
                               state->scp_service,
                               &state->com_port_info);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  /** Step 1
   *   If the device has FUSE ROM, test the sensitivity value
   **/
  if ((state->mag_info.device_select == AK09911) || (state->mag_info.device_select == AK09912))
  {
    rv = ak0991x_read_asa(this,
                          state->scp_service,
                          state->com_port_info.port_handle,
                          asa);


    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_READ_ASA) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    AKM_FST(TLIMIT_NO_ASAX, asa[0], TLIMIT_LO_ASAX, TLIMIT_HI_ASAX, err);
    AKM_FST(TLIMIT_NO_ASAY, asa[1], TLIMIT_LO_ASAY, TLIMIT_HI_ASAY, err);
    AKM_FST(TLIMIT_NO_ASAZ, asa[2], TLIMIT_LO_ASAZ, TLIMIT_HI_ASAZ, err);
  }

  /** Step 2
   *   Continuous mode check
   **/

  /* Set to CNT measurement mode3 (50Hz) */
  buffer[0] = 0x00;
  buffer[1] = AK0991X_MAG_ODR50;
  /* Enable FIFO for AK09917D RevA/B bug */
  if (state->mag_info.device_select == AK09917)
  {
    buffer[1] |= AK0991X_FIFO_BIT;
  }
  rv = ak0991x_com_write_wrapper(this,
                   state->scp_service,
                   state->com_port_info.port_handle,
                   AKM_AK0991X_REG_CNTL1,
                   buffer,
                   2,
                   &xfer_bytes,
                   false);

  if (rv != SNS_RC_SUCCESS
    ||
    xfer_bytes != 2)
  {
    *err = ((TLIMIT_NO_CNT_CNTL2) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  for (i = 0; i < TLIMIT_NO_CNT_ITR; i++)
  {
    /* current test program sets to MODE3, i.e. 50Hz=20msec interval
     * so, wait for double length of measurement time.
     */
    rv = ak0991x_wait_drdy_poll(this, 40);
    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_WAIT) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Get measurement from device (1st). */
    rv = ak0991x_read_st1_st2(state, buffer);

    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_READ) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Check DRDY bit (1st) */
    state->data_is_ready = (buffer[0] & AK0991X_DRDY_BIT) ? true : false;
    AKM_FST(TLIMIT_NO_CNT_1ST, state->data_is_ready, TLIMIT_LO_CNT_1ST, TLIMIT_HI_CNT_1ST, err);

    /* Get measurement from device (2nd). */
    rv = ak0991x_read_st1_st2(state, buffer);

    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_READ) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Check DRDY bit (2nd) */
    state->data_is_ready = (buffer[0] & AK0991X_DRDY_BIT) ? true : false;
    AKM_FST(TLIMIT_NO_CNT_2ND, state->data_is_ready, TLIMIT_LO_CNT_2ND, TLIMIT_HI_CNT_2ND, err);
  }

  // Reset device
  rv = ak0991x_device_sw_reset(this,
                               state->scp_service,
                               &state->com_port_info);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  /* Wait over 100us */
  sns_busy_wait(sns_convert_ns_to_ticks(100 * 1000));

  /** Step 3
   *   Start self test
   **/
  buffer[0] = AK0991X_MAG_SELFTEST;
  rv = ak0991x_com_write_wrapper(this,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (rv != SNS_RC_SUCCESS
      ||
      xfer_bytes != 1)
  {
    *err = ((TLIMIT_NO_SET_SELFTEST) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  rv = ak0991x_get_meas_time(state->mag_info.device_select, sdr, &usec_time_for_measure);

  if(rv != SNS_RC_SUCCESS)
  {
    *err = (TLIMIT_NO_INVALID_ID << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  // To ensure that measurement is finished, wait for double as typical
  sns_busy_wait(sns_convert_ns_to_ticks(usec_time_for_measure * 1000 * 2));

  /** Step 4
   *   Read and check data
   **/
  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST1,
                                buffer,
                                AK0991X_NUM_DATA_ST1_TO_ST2,
                                &xfer_bytes);

  if (rv != SNS_RC_SUCCESS
      ||
      xfer_bytes != AK0991X_NUM_DATA_ST1_TO_ST2)
  {
    *err = ((TLIMIT_NO_READ_DATA) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  ak0991x_get_adjusted_mag_data(this, &buffer[1], &data[0]);

  // check read value
  if (state->mag_info.device_select == AK09918)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09918, TLIMIT_HI_SLF_RVHX_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09918, TLIMIT_HI_SLF_RVHY_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09918, TLIMIT_HI_SLF_RVHZ_AK09918,
            err);
  }
  else if (state->mag_info.device_select == AK09917)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09917, TLIMIT_HI_SLF_RVHX_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09917, TLIMIT_HI_SLF_RVHY_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09917, TLIMIT_HI_SLF_RVHZ_AK09917,
            err);
  }
  else if ((state->mag_info.device_select == AK09916C) || (state->mag_info.device_select == AK09916D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09916, TLIMIT_HI_SLF_RVHX_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09916, TLIMIT_HI_SLF_RVHY_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09916, TLIMIT_HI_SLF_RVHZ_AK09916,
            err);
  }
  else if ((state->mag_info.device_select == AK09915C) || (state->mag_info.device_select == AK09915D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09915, TLIMIT_HI_SLF_RVHX_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09915, TLIMIT_HI_SLF_RVHY_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09915, TLIMIT_HI_SLF_RVHZ_AK09915,
            err);
  }
  else if (state->mag_info.device_select == AK09913)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09913, TLIMIT_HI_SLF_RVHX_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09913, TLIMIT_HI_SLF_RVHY_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09913, TLIMIT_HI_SLF_RVHZ_AK09913,
            err);
  }
  else if (state->mag_info.device_select == AK09912)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09912, TLIMIT_HI_SLF_RVHX_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09912, TLIMIT_HI_SLF_RVHY_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09912, TLIMIT_HI_SLF_RVHZ_AK09912,
            err);
  }
  else if (state->mag_info.device_select == AK09911)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09911, TLIMIT_HI_SLF_RVHX_AK09911,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09911, TLIMIT_HI_SLF_RVHY_AK09911,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09911, TLIMIT_HI_SLF_RVHZ_AK09911,
            err);
  }
  else
  {
    *err = (TLIMIT_NO_INVALID_ID << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  AKM_FST(TLIMIT_NO_SLF_ST2, (buffer[8] & TLIMIT_ST2_MASK),
          TLIMIT_LO_SLF_ST2, TLIMIT_HI_SLF_ST2, err);

TEST_SEQUENCE_FAILED:

  if (*err == 0)
  {
    return SNS_RC_SUCCESS;
  }
  else
  {
    AK0991X_INST_PRINT(HIGH, this, "hw self-test failed!! err code = %x",*err);
    // Reset device
    ak0991x_device_sw_reset(this,
                            state->scp_service,
                            &state->com_port_info);

    return SNS_RC_FAILED;
  }
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_sstvt_adj(
                             sns_sync_com_port_service* scp_service,
                             sns_sync_com_port_handle *port_handle,
                             akm_device_type device_select,
                             float *sstvt_adj)
{
  sns_rc  rv = SNS_RC_SUCCESS;
  uint8_t i;

  // If the device does not have FUSE ROM, we don't need to access it.
  if ((device_select != AK09911) && (device_select != AK09912))
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = 1.0f;
    }
    return rv;
  }

  uint8_t buffer[AK0991X_NUM_SENSITIVITY] = {0};
  rv = ak0991x_read_asa(NULL, scp_service,port_handle, buffer);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  if (device_select == AK09911)
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = ((buffer[i] / 128.0f) + 1.0f);
    }
  }
  else if (device_select == AK09912)
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = ((buffer[i] / 256.0f) + 0.5f);
    }
  }
  else
  {
    // no device
    rv = SNS_RC_FAILED;
  }
  return rv;
}

/**
 * Gets current ODR.
 *
 * @param[i] curr_odr              Current ODR.
 *
 * @return current ODR
 */
float ak0991x_get_mag_odr(ak0991x_mag_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for (idx = 0; idx < ARR_SIZE(reg_map_ak0991x); idx++)
  {
    if (curr_odr == reg_map_ak0991x[idx].mag_odr_reg_value
        &&
        curr_odr != AK0991X_MAG_ODR_OFF)
    {
      odr = reg_map_ak0991x[idx].odr;
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
sns_time ak0991x_get_sample_interval(ak0991x_mag_odr curr_odr)
{
  if(curr_odr == AK0991X_MAG_ODR_OFF)
  {
    return 0;
  }
  return sns_convert_ns_to_ticks(1000000000 / ak0991x_get_mag_odr(curr_odr));
}

/**
 * Extract a mag sample from a segment of the mag buffer and generate an
 * event.
 *
 * @param[i] fifo_sample       The segment of fifo buffer that has the mag sample
 * @param[i] timestamp         Timestamp to be used for this sample
 * @param[i] instance          The current ak0991x sensor instance
 * @param[i] state             The state of the ak0991x sensor instance
 */
static sns_std_sensor_sample_status ak0991x_handle_mag_sample(uint8_t mag_sample[8],
                                      sns_time timestamp,
                                      sns_sensor_instance *const instance,
                                      ak0991x_instance_state *state,
                                      log_sensor_state_raw_info *log_mag_state_raw_info
                                      )
{
  float ipdata[TRIAXIS_NUM] = {0}, opdata_raw[TRIAXIS_NUM] = {0};
  int16_t lsbdata[TRIAXIS_NUM] = {0};
  uint8_t inv_fifo_bit = 0x00;
  uint8_t i = 0;
  sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  vector3 opdata_cal;
  uint32_t cal_id;

  ak0991x_get_adjusted_mag_data(instance, mag_sample, lsbdata);

  // Check magnetic sensor overflow (and invalid data for FIFO)
  if(state->mag_info.use_fifo ||
     // Since FIFO is forced to enable on Polling mode for preventing duplicate samples
     ((state->mag_info.device_select == AK09917) &&
     (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING) &&
     !state->mag_info.use_sync_stream))
  {
    inv_fifo_bit = AK0991X_INV_FIFO_DATA;
  }

  if ((mag_sample[7] & (AK0991X_HOFL_BIT)) != 0 ||
      (mag_sample[7] & (inv_fifo_bit)) != 0)
  {
    // data is unreliable. use previous measured data instead.
    lsbdata[TRIAXIS_X] = state->mag_info.previous_lsbdata[TRIAXIS_X];
    lsbdata[TRIAXIS_Y] = state->mag_info.previous_lsbdata[TRIAXIS_Y];
    lsbdata[TRIAXIS_Z] = state->mag_info.previous_lsbdata[TRIAXIS_Z];
    AK0991X_INST_PRINT(MED, instance, "UNRELIABLE: HOFL_BIT=1 or INV=1 ST2=0x%X", mag_sample[7]);
  }

  if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING &&
      (lsbdata[TRIAXIS_X] == 0) &&
      (lsbdata[TRIAXIS_Y] == 0) &&
      (lsbdata[TRIAXIS_Z] == 0) )
  {
    lsbdata[TRIAXIS_X] = state->mag_info.previous_lsbdata[TRIAXIS_X];
    lsbdata[TRIAXIS_Y] = state->mag_info.previous_lsbdata[TRIAXIS_Y];
    lsbdata[TRIAXIS_Z] = state->mag_info.previous_lsbdata[TRIAXIS_Z];

    // no measurement done yet, set UNRELIABLE
    if( (lsbdata[TRIAXIS_X] == 0) &&
        (lsbdata[TRIAXIS_Y] == 0) &&
        (lsbdata[TRIAXIS_Z] == 0) )
    {
      AK0991X_INST_PRINT(MED, instance, "UNRELIABLE: raw(0,0,0)");
      status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    }
  }

  ipdata[TRIAXIS_X] = lsbdata[TRIAXIS_X] * state->mag_info.resolution;
  ipdata[TRIAXIS_Y] = lsbdata[TRIAXIS_Y] * state->mag_info.resolution;
  ipdata[TRIAXIS_Z] = lsbdata[TRIAXIS_Z] * state->mag_info.resolution;

  // axis conversion
  for (i = 0; i < TRIAXIS_NUM; i++)
  {
    opdata_raw[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) *
      ipdata[state->axis_map[i].ipaxis];
  }

  cal_id = (state->cal.id == AK0991X_UNKNOWN_DEVICE_MODE) ? state->prev_cal_id : state->cal.id;

  // factory calibration
  opdata_cal = sns_apply_calibration_correction_3(
    make_vector3_from_array(opdata_raw),
    make_vector3_from_array(state->cal.params[cal_id].bias),
    state->cal.params[cal_id].corr_mat);

  state->mag_info.previous_lsbdata[TRIAXIS_X] = lsbdata[TRIAXIS_X];
  state->mag_info.previous_lsbdata[TRIAXIS_Y] = lsbdata[TRIAXIS_Y];
  state->mag_info.previous_lsbdata[TRIAXIS_Z] = lsbdata[TRIAXIS_Z];

  pb_send_sensor_stream_event(instance,
                              &state->mag_info.suid,
                              timestamp,
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              status,
                              opdata_cal.data,
                              ARR_SIZE(opdata_cal.data),
                              state->encoded_mag_event_len);

  // Log raw uncalibrated sensor data
  ak0991x_log_sensor_state_raw_add(
    log_mag_state_raw_info,
    opdata_raw,
    timestamp,
    status);
  state->total_samples++;

  return status;
}

void ak0991x_process_mag_data_buffer(sns_sensor_instance *instance,
                                     sns_time            first_timestamp,
                                     sns_time            sample_interval_ticks,
                                     uint8_t             *fifo_start,
                                     size_t              num_bytes
)
{
  uint16_t num_samples_sets = 0;
  uint32_t i;
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_time timestamp = state->pre_timestamp;
  sns_time report_time;

  if(!state->this_is_the_last_flush && state->mag_info.cur_cfg.odr == AK0991X_MAG_ODR_OFF)
  {
    AK0991X_INST_PRINT(MED, instance, "Discarding %u data bytes", num_bytes);
    return;
  }

  sns_diag_service          *diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;
  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  log_mag_state_raw_info.diag = diag;
  log_mag_state_raw_info.instance = instance;
  log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

  size_t num_bytes_to_report;
  num_bytes_to_report = num_bytes;

  int8_t over_sample;

  //skip the data to adjust timing for Polling+FIFO
  if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && state->mag_info.use_fifo)
  {
    if(state->fifo_flush_in_progress)
    {
      over_sample = state->flush_sample_count - state->mag_info.cur_cfg.fifo_wmk;
    }
    else
    {
      over_sample = state->num_samples - state->mag_info.cur_cfg.fifo_wmk;
    }

    if(over_sample > 0)
    {
      num_bytes_to_report -= over_sample * 8;
    }
  }

  for(i = 0; i < num_bytes_to_report; i += AK0991X_NUM_DATA_HXL_TO_ST2)
  {
    timestamp = first_timestamp + (num_samples_sets++ * sample_interval_ticks);
    report_time = timestamp - state->half_measurement_time;
    state->accuracy = ak0991x_handle_mag_sample(
        &fifo_start[i],
        report_time,
        instance,
        state,
        &log_mag_state_raw_info);

#ifdef AK0991X_ENABLE_TS_DEBUG
    if(num_samples_sets == 1 || num_samples_sets == (num_bytes>>3) )
    {
      if(ak0991x_dae_if_available(instance))
      {
        AK0991X_INST_PRINT(LOW, instance, "TS %X%08X dae %X%08X ave %u # %u of %u flush %u # %u",
            (uint32_t)(report_time>>32),
            (uint32_t)(report_time & 0xFFFFFFFF),
//            (uint32_t)(state->pre_timestamp>>32),
//            (uint32_t)(state->pre_timestamp & 0xFFFFFFFF),
//            (uint32_t)(state->irq_event_time>>32),
//            (uint32_t)(state->irq_event_time & 0xFFFFFFFF),
            (uint32_t)(state->dae_event_time>>32),
            (uint32_t)(state->dae_event_time & 0xFFFFFFFF),
            (uint32_t)state->averaged_interval,
            num_samples_sets,
            state->num_samples,
            state->fifo_flush_in_progress,
            state->total_samples);
      }
      else
      {
        AK0991X_INST_PRINT(LOW, instance, "TS %X%08X sys %X%08X ave %u # %u of %u flush %u # %u",
            (uint32_t)(report_time>>32),
            (uint32_t)(report_time & 0xFFFFFFFF),
//            (uint32_t)(state->pre_timestamp>>32),
//            (uint32_t)(state->pre_timestamp & 0xFFFFFFFF),
//            (uint32_t)(state->irq_event_time>>32),
//            (uint32_t)(state->irq_event_time & 0xFFFFFFFF),
            (uint32_t)(state->system_time>>32),
            (uint32_t)(state->system_time & 0xFFFFFFFF),
            (uint32_t)state->averaged_interval,
            num_samples_sets,
            state->num_samples,
            state->fifo_flush_in_progress,
            state->total_samples);
      }
    }
#endif

    // Since FIFO is forced to enable on Polling mode for preventing duplicate samples. Break.
    if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && !state->mag_info.use_sync_stream )
    {
      break;
    }
  }

  // store previous is irq status
  state->is_previous_irq = state->irq_info.detect_irq_event;

  // store previous timestamp
  state->pre_timestamp_for_orphan = timestamp;

  if(!state->is_orphan)
  {
    state->pre_timestamp = timestamp;
    state->this_is_first_data = false;
  }

  // reset flags
  state->irq_info.detect_irq_event = false;

  if(state->accuracy == SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH)
  {
    if( !ak0991x_dae_if_available(instance) && state->fifo_flush_in_progress) 
    {
      ak0991x_send_fifo_flush_done(instance);
    }
  }

  ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
}

/** See ak0991x_hal.h */
void ak0991x_send_fifo_flush_done(sns_sensor_instance *const instance)
{
  sns_service_manager *mgr = instance->cb->get_service_manager(instance);
  sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr,SNS_EVENT_SERVICE);
  sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  state->system_time = sns_get_system_time();

  if(NULL != event)
  {
    event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
    event->event_len = 0;
    event->timestamp = state->system_time;

    e_service->api->publish_event(e_service, instance, event, &state->mag_info.suid);

    AK0991X_INST_PRINT(HIGH, instance, "FLUSH DONE.");
  }
  else
  {
    AK0991X_INST_PRINT(HIGH, instance, "FLUSH DONE. event = null");
  }

  state->fifo_flush_in_progress = false;
  state->flush_requested_in_dae = false;
}

static void ak0991x_calc_clock_error(ak0991x_instance_state *state, sns_time intvl)
{
  state->internal_clock_error =
      ((state->interrupt_timestamp - state->previous_irq_time) << AK0991X_CALC_BIT_RESOLUTION) / intvl;
}

static sns_rc ak0991x_calc_average_interval_for_dri(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_rc rc = SNS_RC_SUCCESS;
  sns_time calculated_average_interval;

  // set interrupt_timestamp
  if( state->irq_info.detect_irq_event && !state->is_orphan )  // DRI
  {
    state->interrupt_timestamp = state->irq_event_time;
    if( !state->this_is_first_data )
    {
      if(!(state->ascp_xfer_in_progress>0 && state->this_is_the_last_flush) &&
          (state->interrupt_timestamp > state->previous_irq_time))
      {
        if( state->num_samples == state->mag_info.cur_cfg.fifo_wmk )
        {
          calculated_average_interval = ((state->interrupt_timestamp - state->previous_irq_time) /
                                      state->mag_info.data_count_for_dri);
          if( (calculated_average_interval > state->averaged_interval - state->averaged_interval/50) &&
              (calculated_average_interval < state->averaged_interval + state->averaged_interval/50) )
          {
            // keep re-calculating for clock frequency drifting.
            ak0991x_calc_clock_error(state, state->nominal_intvl * state->mag_info.data_count_for_dri);

            //update
            state->averaged_interval = calculated_average_interval;
          }
          else
          {
            AK0991X_INST_PRINT(HIGH, instance, "Calculated interval %u is too different from the previous %u",
                (uint32_t)calculated_average_interval,
                (uint32_t)state->averaged_interval);
          }
        }
        else
        {
          AK0991X_INST_PRINT(LOW, instance, "Unreliable irq. (#samples != WM) (%d != %d)",
              (int)state->num_samples, state->mag_info.cur_cfg.fifo_wmk);
          rc = SNS_RC_FAILED;
        }
      }
      else
      {
        AK0991X_INST_PRINT(HIGH, instance, "Skip calc clock error. ascp=%d, last_flush=%d, int=%u pre_irq=%u",
            state->ascp_xfer_in_progress,
            state->this_is_the_last_flush,
            (uint32_t)state->interrupt_timestamp,
            (uint32_t)state->previous_irq_time);
        rc = SNS_RC_FAILED;
      }
    }
  }

  if(state->this_is_first_data) // come into DRI and flush request
  {
    AK0991X_INST_PRINT(LOW, instance, "this is the first data. avg_intvl %u clk_err %u",
                       (uint32_t)state->averaged_interval,
                       (uint32_t)state->internal_clock_error);
  }

  return rc;
}

void ak0991x_validate_timestamp_for_dri(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  int32_t delta_ts_now;

  sns_rc rc = ak0991x_calc_average_interval_for_dri(instance);

  if(state->irq_info.detect_irq_event && (rc == SNS_RC_SUCCESS) )  // DRI
  {
    state->first_data_ts_of_batch = state->interrupt_timestamp -
      (state->averaged_interval * (state->mag_info.cur_cfg.fifo_wmk-1));
    state->mag_info.data_count_for_dri = 0; // reset only for DRI mode
    state->previous_irq_time = state->interrupt_timestamp;
    state->delta_ts_time = state->system_time - state->interrupt_timestamp; // update delta_ts_time
  }
  else    // flush request during DRI, use previous calculated average_interval
  {
    state->interrupt_timestamp = state->pre_timestamp + state->averaged_interval * state->num_samples;
    state->first_data_ts_of_batch = state->pre_timestamp + state->averaged_interval;

    if( ak0991x_dae_if_available(instance) )
    {
      if( state->dae_event_time < state->interrupt_timestamp )  // check negative timestamp
      {
        state->interrupt_timestamp = state->dae_event_time;
      }
      else if( state->dae_event_time > state->interrupt_timestamp + state->averaged_interval )    // check drifting for flush only
      {
        // reset using dae_event_time
        state->interrupt_timestamp = state->dae_event_time - state->averaged_interval;
      }
      state->first_data_ts_of_batch = state->interrupt_timestamp - (state->num_samples-1) * state->averaged_interval;
    }
    else
    {
      // check TS time drifting part when flush is continued and adjust it
      {
        // in case the first data is flush
        if(state->this_is_first_data)
        {
          state->delta_ts_time = state->averaged_interval + state->averaged_interval/10; // 10% average interval
        }
        delta_ts_now = state->system_time - state->interrupt_timestamp;
        if( !state->this_is_the_last_flush &&
            state->mag_info.data_count_for_dri > (state->mag_info.cur_cfg.fifo_wmk * 10) &&
            delta_ts_now > state->delta_ts_time)
        {
            AK0991X_INST_PRINT(LOW, instance, "detect delay: delta %u",
                               (uint32_t)(delta_ts_now - state->delta_ts_time));
            state->interrupt_timestamp = state->interrupt_timestamp + state->averaged_interval/5;          // added 20% average time
            state->first_data_ts_of_batch = state->first_data_ts_of_batch + state->averaged_interval/5;    // added 20% average time
        }
      }
      if( rc != SNS_RC_SUCCESS )
      {
        state->previous_irq_time = state->interrupt_timestamp;
      }
    }
  }
}

void ak0991x_validate_timestamp_for_polling(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_time calculated_timestamp_from_previous;

  calculated_timestamp_from_previous = state->pre_timestamp + state->averaged_interval * state->num_samples;

  // check negative timestamp
  if( state->system_time < calculated_timestamp_from_previous )
  {
    calculated_timestamp_from_previous = state->system_time;
  }

  if( state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
  {
    if(ak0991x_dae_if_available(instance))
    {
      if(calculated_timestamp_from_previous > state->dae_event_time)
      {
        state->interrupt_timestamp = state->dae_event_time;
      }
      else
      {
        state->interrupt_timestamp = calculated_timestamp_from_previous;
      }
    }
    else
    {
      // from flush request
      state->interrupt_timestamp = state->system_time;
    }
  }
  else
  {
    if(ak0991x_dae_if_available(instance))
    {
#ifdef AK0991X_ENABLE_TIMER_FILTER
      // check delayed timer timestamp for preventing jitter
      if( !state->this_is_first_data &&
          state->is_previous_irq &&
          state->dae_event_time > (calculated_timestamp_from_previous + state->averaged_interval/50) )
      {
        state->interrupt_timestamp = calculated_timestamp_from_previous;
        AK0991X_INST_PRINT(LOW, instance, "delayed timer detected. recalculate timestamp %u->%u",
                                   (uint32_t)state->dae_event_time,
                                   (uint32_t)state->interrupt_timestamp);
      }
      else
      {
        state->interrupt_timestamp = state->dae_event_time;
      }
#else
      state->interrupt_timestamp = state->dae_event_time;
#endif
    }
    else
    {
      // from timer event
      state->interrupt_timestamp = state->system_time;
    }
  }

  state->first_data_ts_of_batch = state->interrupt_timestamp - state->averaged_interval * (state->num_samples - 1);
}
void ak0991x_get_st1_status(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  uint8_t st1_buf = 0, st2_buf = 0;

  ak0991x_read_st1(state, &st1_buf);  // read ST1
  /* Read ST2 for AK09917D RevA/B bug */
  if( state->mag_info.device_select == AK09917 && 
      !state->mag_info.use_fifo)
  {
    if( !(!state->mag_info.use_sync_stream &&
        state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING) )
    {
      ak0991x_read_st2(state, &st2_buf);
    }
  }

  state->data_over_run = (st1_buf & AK0991X_DOR_BIT) ? true : false;  // check data over run
  state->data_is_ready = (state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) ? ((st1_buf & AK0991X_DRDY_BIT) ? true : false) : true; // check DRDY when DRI mode

  if( state->mag_info.use_fifo )
  {
    if(state->mag_info.device_select == AK09917)
    {
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
      {
        state->num_samples = st1_buf >> 2;
      }
      else
      {
        // current event is requested flush or flush to change ODR
        if(state->fifo_flush_in_progress || state->this_is_the_last_flush)
        {
          AK0991X_INST_PRINT(LOW, instance, "requested flush or flush to change ODR");

          state->num_samples = st1_buf >> 2;
          if(state->fifo_flush_in_progress)
          {
            state->flush_sample_count += state->num_samples;
          }
          else
          {
            state->flush_sample_count = 0;
          }
        }
        else
        {
          // check previous event
          if(state->flush_sample_count == 0) //both previous and current event are Polling
          {
//            AK0991X_INST_PRINT(LOW, instance, "both pre and cur event is polling");
            state->num_samples = state->mag_info.cur_cfg.fifo_wmk;
          }
          else //previous event is requested FLUSH and current event is Polling
          {
            int16_t calculated_samples;
            calculated_samples = state->mag_info.cur_cfg.fifo_wmk - state->flush_sample_count;
            state->num_samples = st1_buf >> 2;
            AK0991X_INST_PRINT(LOW, instance, "calculated_samples %d num_samples %d", 
                               calculated_samples,state->num_samples);

            if(calculated_samples > 0)
            {
              state->num_samples = (uint8_t)calculated_samples;

              // check timestamp
              if( ( state->system_time + (state->averaged_interval>>1) ) <= state->pre_timestamp + (state->averaged_interval * state->num_samples) )
              {
                state->num_samples--;
                AK0991X_INST_PRINT(LOW, instance, "interval is less than averaged_interval/2. num_sample is now %d", state->num_samples);
              }
            }
            else
            {
              state->num_samples = 0;
            }
          }
          state->flush_sample_count = 0;
        }
      }

      AK0991X_INST_PRINT(LOW, instance, "FNUM %d num_samples %d flush_sample_count %d wm %d", (st1_buf >> 2), state->num_samples, state->flush_sample_count, state->mag_info.cur_cfg.fifo_wmk);

      if(state->num_samples == 0)
      {
        AK0991X_INST_PRINT(LOW, instance, "num_samples==0!!!");
      }
    }
    else
    {
      state->num_samples = state->mag_info.cur_cfg.fifo_wmk;
    }

    if((state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2) > AK0991X_MAX_FIFO_SIZE)
    {
      SNS_INST_PRINTF(ERROR, instance,
          "FIFO size should not be greater than AK0991X_MAX_FIFO_SIZE."
          "So, num_samples to read limiting to max value");
      state->num_samples = (AK0991X_MAX_FIFO_SIZE / AK0991X_NUM_DATA_HXL_TO_ST2);
    }
  }
  else
  {
    //Since FIFO is forced to enable on Polling mode for preventing duplicate samples
    if((state->mag_info.device_select == AK09917) &&
       (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING) &&
       !state->mag_info.use_sync_stream)
    {
      state->fifo_num_samples = st1_buf >> 2;
    }

    state->num_samples = state->data_is_ready ? 1 : 0;
  }

  if(state->data_over_run && state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
  {
    AK0991X_INST_PRINT(LOW, instance, "DOR detected");
  }
}

static void ak0991x_ascp_request(sns_sensor_instance *const instance)
{
  uint8_t  buffer[AK0991X_MAX_FIFO_SIZE];
  uint32_t enc_len;
  uint16_t num_of_bytes;
  sns_request async_com_port_request;
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;
  sns_rc rc;
  sns_port_vector async_read_msg;

  num_of_bytes = AK0991X_NUM_DATA_HXL_TO_ST2 * state->num_samples;

  // Compose the async com port message
  async_read_msg.bytes = num_of_bytes;
  async_read_msg.reg_addr = AKM_AK0991X_REG_HXL;
  async_read_msg.is_write = false;
  async_read_msg.buffer = NULL;

  sns_ascp_create_encoded_vectors_buffer(&async_read_msg, 1, true, buffer, sizeof(buffer),
                                         &enc_len);

  // Send message to Async Com Port
  async_com_port_request =
    (sns_request)
  {
    .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW,
    .request_len = enc_len,
    .request = buffer
  };
  rc = state->async_com_port_data_stream->api->send_request(
    state->async_com_port_data_stream, &async_com_port_request);
  if(rc == SNS_RC_SUCCESS)
  {
    state->ascp_xfer_in_progress++;
    AK0991X_INST_PRINT(LOW, instance, "Succeeded sending request to ASCP");
  }
  else
  {
    SNS_INST_PRINTF(ERROR, instance, "Failed sending request to ASCP");
  }
}

static sns_rc ak0991x_check_ascp(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_rc rc = SNS_RC_SUCCESS;

  // is ASCP is still during in the process, skip flush
  if(state->ascp_xfer_in_progress > 0)
  {
    AK0991X_INST_PRINT(LOW, instance, "#ascp_xfer=%u", state->ascp_xfer_in_progress);
    rc |= SNS_RC_FAILED;
  }

  if(state->fifo_flush_in_progress && state->in_clock_error_procedure)
  {
    // whether IRQ detected or not, no flushing can be done during clock error procedure
    AK0991X_INST_PRINT(LOW, instance, "irq for osc error is not received yet. wait...");
    rc |= SNS_RC_FAILED;
  }

  if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING &&
     !state->irq_info.detect_irq_event &&
     state->heart_beat_attempt_count==0 )
  {
    if(state->system_time < state->pre_timestamp + state->averaged_interval)
    {
      AK0991X_INST_PRINT(LOW, instance, "FIFO empty");
      rc |= SNS_RC_FAILED;
    }
    else
    {
      const sns_time few_ms = sns_convert_ns_to_ticks(5 * 1000 * 1000);
      sns_time estimated_irq_time = 
        state->pre_timestamp + state->averaged_interval * state->mag_info.cur_cfg.fifo_wmk;

      // irq expected to fire within a few ms?
      if(estimated_irq_time > state->system_time &&
         estimated_irq_time < state->system_time + few_ms)
      {
        AK0991X_INST_PRINT(LOW, instance, "estimated irq %u. wait...", (uint32_t)estimated_irq_time);
        rc |= SNS_RC_FAILED;
      }
    }
  }
  return rc;
}

void ak0991x_clock_error_calc_procedure(
  sns_sensor_instance *const instance,
  uint8_t const *st1_buf)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  uint8_t buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int64_t previous_error;

  if(st1_buf == NULL)
  {
    ak0991x_read_st1_st2(state, &buffer[0]);
  }
  else
  {
    buffer[0] = *st1_buf;
  }
  AK0991X_INST_PRINT(LOW, instance, "ST1=%x", buffer[0]);

  state->data_over_run = (buffer[0] & AK0991X_DOR_BIT) ? true : false;  // check data over run
  state->data_is_ready = (buffer[0] & AK0991X_DRDY_BIT) ? true : false; // check DRDY bit

  if(state->data_over_run)
  {
    // measurement failed. Try again.
    state->mag_info.clock_error_meas_count = 0;
    AK0991X_INST_PRINT(LOW, instance, "DOR. restart clock error measurement");
  }
  else if(!state->data_is_ready)
  {
    // measurement failed. Try again.
    state->mag_info.clock_error_meas_count = 0;
    AK0991X_INST_PRINT(LOW, instance, "DRDY is not ready. restart clock error measurement");
  }
  else
  {
    state->mag_info.clock_error_meas_count++;
    state->previous_irq_time = state->interrupt_timestamp;
    state->interrupt_timestamp = state->irq_event_time;

    previous_error = state->internal_clock_error;
    ak0991x_calc_clock_error(state, ak0991x_get_sample_interval(AK0991X_MAG_ODR100));
  }

  if(state->mag_info.clock_error_meas_count == AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC)
  {
    // check clock stability
    int64_t diff = state->internal_clock_error - previous_error;
    AK0991X_INST_PRINT(LOW, instance, "clock error calc: %u and %u.",
        (uint32_t)previous_error,
        (uint32_t)state->internal_clock_error);
    if(diff < -(AK0991X_CALC_BIT_ERROR) || diff > AK0991X_CALC_BIT_ERROR) // 0.5% with 2^13
    {
      AK0991X_INST_PRINT(LOW, instance, "clock error is too big. restart clock error measurement");
      state->mag_info.clock_error_meas_count = 0; // measurement failed. Try again.
    }
    else
    {
      // got clock error rate.
      state->in_clock_error_procedure = false;
      state->internal_clock_error = (state->internal_clock_error >> 1) + (previous_error >> 1);

      AK0991X_INST_PRINT(LOW, instance, "INT %u PRE %u diff %u clk_err %d Re-Start with actual ODR.",
          (uint32_t)state->interrupt_timestamp,
          (uint32_t)state->previous_irq_time,
          (uint32_t)(state->interrupt_timestamp - state->previous_irq_time),
          (uint32_t)state->internal_clock_error);
    }
  }

  // when failed restart clock error procedure.
  if(state->mag_info.clock_error_meas_count == 0)
  {
    state->in_clock_error_procedure = false;

    // restart clock error procedure.
    ak0991x_reconfig_hw(instance, true);
  }

  state->irq_info.detect_irq_event = false;
}

static void ak0991x_read_fifo_buffer(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  uint8_t buffer[AK0991X_MAX_PHYSICAL_FIFO_SIZE*AK0991X_NUM_DATA_HXL_TO_ST2] = {0};// Sometimes, mag is taking scp path with number of samples 32

  // FIFO mode
  if(state->mag_info.use_fifo)
  {
    if(state->mag_info.device_select == AK09917)    // AK09917
    {
      if( state->num_samples > 2 &&
          !state->this_is_the_last_flush &&
          state->heart_beat_attempt_count == 0)
      {
        ak0991x_ascp_request(instance);  // ASCP request
      }
      else
      {
        if(state->num_samples > 0)
        {
          // Read fifo buffer(HXL to ST2 register)
          if (SNS_RC_SUCCESS != ak0991x_read_hxl_st2(state, state->num_samples, &buffer[0]))  // SYNC read
          {
            state->num_samples = 0;
            SNS_INST_PRINTF(ERROR, instance, "Error in reading the FIFO buffer");
          }
        }
      }
    }
    else // AK09915C / AK09915D
    {
      // sync flush
      //Continue reading until fifo buffer is clear
      //because there is no way to check FIFO samples for AK09915C/D.
      // QC - we recommend reading WM+1 samples then verifying that the last sample is
      // INV_FIFO_DATA; if the last sample is valid THEN start reading one sample at a time
      uint32_t i;
      state->num_samples = state->mag_info.max_fifo_size;
      for (i = 0; i < state->mag_info.max_fifo_size; i++)
      {
        //Read fifo buffer(HXL to ST2 register)
        if (SNS_RC_SUCCESS != ak0991x_read_hxl_st2(state,
                                                   1,
                                                   &buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2]))
        {
          SNS_INST_PRINTF(ERROR, instance, "Error in reading the FIFO buffer");
        }

        if ((buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2 + 7] & AK0991X_INV_FIFO_DATA) != 0)
        {
          //fifo buffer is clear
          state->num_samples = i;
          break;
        }
      }
    }
  }
  else  // Non FIFO mode
  {
    uint16_t num_samples = 1;

    //Since FIFO is forced to enable on Polling mode for preventing duplicate samples
    if((state->mag_info.device_select == AK09917) &&
       (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING) &&
       !state->mag_info.use_sync_stream)
    {
      num_samples = (state->fifo_num_samples > num_samples)?
                     state->fifo_num_samples : num_samples;
      if(num_samples > AK0991X_MAX_PHYSICAL_FIFO_SIZE)
      {
        SNS_INST_PRINTF(ERROR, instance,"Number of samples should not be greater than max physical fifo available");
        num_samples = AK0991X_MAX_PHYSICAL_FIFO_SIZE;
      }
    }

    ak0991x_read_hxl_st2(state,
                         num_samples,
                         &buffer[0]);
  }

  if(state->num_samples > 0)
  {
    // adjust timestamp and interval if needed.
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
    {
      state->mag_info.data_count_for_dri+=state->num_samples;
      ak0991x_validate_timestamp_for_dri(instance);
    }
    else
    {
      ak0991x_validate_timestamp_for_polling(instance);
    }

    // sync flush
    if( state->ascp_xfer_in_progress == 0 )
    {
      ak0991x_process_mag_data_buffer(instance,
                                      state->first_data_ts_of_batch,
                                      state->averaged_interval,
                                      buffer,
                                      AK0991X_NUM_DATA_HXL_TO_ST2 * state->num_samples);
    }

    state->heart_beat_attempt_count = 0;
  }
  if( state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING &&
      !state->this_is_the_last_flush &&
      state->system_time + (state->averaged_interval * state->mag_info.cur_cfg.fifo_wmk) > state->hb_timer_fire_time )
  {
    ak0991x_register_heart_beat_timer(instance);
  }
}

void ak0991x_read_mag_samples(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  uint8_t num_count = 0;

  if(state->this_is_the_last_flush || SNS_RC_SUCCESS == ak0991x_check_ascp(instance))
  {
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) // DRI mode
    {
      if(!state->irq_info.detect_irq_event) // flush request received.
      {
        ak0991x_get_st1_status(instance);

        // add dummy when last flush. For MAG-048 on DRI+FIFO caused by calculated average_intarval mismatch on flush only measurement
        if( (state->this_is_the_last_flush) &&
            (state->averaged_interval > 0) )
        {
          while( state->system_time + sns_convert_ns_to_ticks(4 * 1000 * 1000) >
                 state->pre_timestamp + state->averaged_interval * (state->num_samples + 1) ) // margin 4msec
          {
            state->num_samples++;
            AK0991X_INST_PRINT(LOW, instance,"add dummy");
          }
          state->num_samples = SNS_MIN(state->num_samples, AK0991X_MAX_FIFO_SIZE);
        }
        else
        {
          if( state->num_samples == 0 && 
              state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && 
              !state->mag_info.use_sync_stream )
          {
            AK0991X_INST_PRINT(LOW, instance, "num_samples=0 But forced to set 1.");
            state->num_samples = 1;            
          }
        }
      }
      // else, use state->num_samples when check DRDY status by INTERRUPT_EVENT
    }
    else  // Polling mode
    {
      if(state->mag_info.use_fifo || state->mag_info.use_sync_stream)
      {
        ak0991x_get_st1_status(instance);

        // check num_samples when the last fifo flush to prevent negative timestamp
        if( (state->this_is_the_last_flush) &&
            (state->averaged_interval > 0) )
        {
          while( (state->pre_timestamp + state->averaged_interval * (num_count+1) < state->system_time) && (state->num_samples > num_count) )
          {
            num_count++;
          }
          state->num_samples = num_count;
        }
      }
      else // no FIFO
      {
        ak0991x_get_st1_status(instance);

        if( state->this_is_the_last_flush )
        {
          state->num_samples = ( state->pre_timestamp + state->averaged_interval * 8/10 < state->system_time ) ? 1 : 0;
        }
        else
        {
          state->num_samples = 1;
        }
      }
    }

    if( state->this_is_the_last_flush )
    {
      AK0991X_INST_PRINT(LOW, instance,"last_flush num=%d sys=%u pre=%u ave=%u",
          state->num_samples,
          (uint32_t)state->system_time,
          (uint32_t)state->pre_timestamp,
          (uint32_t)state->averaged_interval);
    }

    // read data. when AK09917 && FIFO mode && WM>2, use ASCP
    ak0991x_read_fifo_buffer(instance);
  }
}

void ak0991x_send_cal_event(sns_sensor_instance *const instance, bool is_new_cal)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_cal_event cal_event = sns_cal_event_init_default;

  if(is_new_cal)
  {
    state->last_cal_event_sent_time = sns_get_system_time();
  }


#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR

  // use prev_cal_id when AK0991X_UNKNOWN_DEVICE_MODE
  cal_event.cal_id = (state->cal.id == AK0991X_UNKNOWN_DEVICE_MODE) ? state->prev_cal_id : state->cal.id;

  if(state->cal.id == AK0991X_UNKNOWN_DEVICE_MODE)
  {
    AK0991X_INST_PRINT(HIGH, instance, "send Unknown cal id");
  }
  pb_buffer_arg buff_arg_bias = { 
    .buf     = &state->cal.params[cal_event.cal_id].bias,
    .buf_len = ARR_SIZE(state->cal.params[cal_event.cal_id].bias) };
  pb_buffer_arg buff_arg_comp_matrix = { 
    .buf     = &state->cal.params[cal_event.cal_id].corr_mat.data,
    .buf_len = ARR_SIZE(state->cal.params[cal_event.cal_id].corr_mat.data) };

  cal_event.bias.funcs.encode        = &pb_encode_float_arr_cb;
  cal_event.bias.arg                 = &buff_arg_bias;
  cal_event.comp_matrix.funcs.encode = &pb_encode_float_arr_cb;
  cal_event.comp_matrix.arg          = &buff_arg_comp_matrix;
  cal_event.status = (state->cal.id == AK0991X_UNKNOWN_DEVICE_MODE) ?
      SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE : SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  cal_event.has_cal_id               = true;

  AK0991X_INST_PRINT(HIGH, instance,
                     "tx CAL_EVENT Time %u",
                     (uint32_t)state->last_cal_event_sent_time);

  AK0991X_INST_PRINT(HIGH, instance,
                     "tx CAL_EVENT: cm %X/%X/%X bias %X/%X/%X",
                     state->cal.params[cal_event.cal_id].corr_mat.e00,
                     state->cal.params[cal_event.cal_id].corr_mat.e11,
                     state->cal.params[cal_event.cal_id].corr_mat.e22,
                     state->cal.params[cal_event.cal_id].bias[0],
                     state->cal.params[cal_event.cal_id].bias[1],
                     state->cal.params[cal_event.cal_id].bias[2]);
  AK0991X_INST_PRINT(HIGH, instance, "tx CAL_EVENT, cal_id:%u", cal_event.cal_id);
#else //AK0991X_ENABLE_DEVICE_MODE_SENSOR
  AK0991X_INST_PRINT(HIGH, instance, "tx CAL_EVENT");
#endif
  pb_send_event(instance,
                sns_cal_event_fields,
                &cal_event,
                state->last_cal_event_sent_time,
                SNS_CAL_MSGID_SNS_CAL_EVENT,
                &state->mag_info.suid);
}

void ak0991x_reset_cal_data(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  float bias_data[] = {0,0,0};
  float comp_matrix_data[] = {1,0,0,0,1,0,0,0,1};

  if(state->cal.id == AK0991X_UNKNOWN_DEVICE_MODE)
  {
    AK0991X_INST_PRINT(HIGH, instance, "unknown cal id, do not reset cal data");
    return;
  }
  for (int i = 0; i < ARR_SIZE(bias_data); i++)
  {
    state->cal.params[state->cal.id].bias[i] = bias_data[i];
  }
  for (int i = 0; i < ARR_SIZE(comp_matrix_data); i++)
  {
    state->cal.params[state->cal.id].corr_mat.data[i] = comp_matrix_data[i];
  }
  state->cal.params[state->cal.id].version++;
}

/** See sns_ak0991x_hal.h */
sns_rc ak0991x_send_config_event(sns_sensor_instance *const instance, bool is_new_config)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
    sns_std_sensor_physical_config_event_init_default;
  char *operating_mode;
  uint8_t op_mode_str_len;
  pb_buffer_arg op_mode_args;

  ak0991x_config_event_info cfg = (is_new_config) ? state->mag_info.cur_cfg : state->mag_info.last_sent_cfg;

  switch (state->mag_info.device_select)
  {
  case AK09911:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09911_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09911_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09911_MIN_RANGE;
    phy_sensor_config.range[1] = AK09911_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = false;
    break;
  case AK09912:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09912_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09912_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09912_MIN_RANGE;
    phy_sensor_config.range[1] = AK09912_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = true;
    break;
  case AK09913:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09913_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09913_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09913_MIN_RANGE;
    phy_sensor_config.range[1] = AK09913_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = false;
    break;
  case AK09915C:

    if (state->mag_info.sdr == 1)
    {
      operating_mode = AK0991X_LOW_NOISE;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_NOISE);
    }
    else
    {
      operating_mode = AK0991X_LOW_POWER;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_POWER);
    }

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = cfg.fifo_wmk;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09915_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09915_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09915_MIN_RANGE;
    phy_sensor_config.range[1] = AK09915_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = true;
    break;
  case AK09915D:

    if (state->mag_info.sdr == 1)
    {
      operating_mode = AK0991X_LOW_NOISE;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_NOISE);
    }
    else
    {
      operating_mode = AK0991X_LOW_POWER;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_POWER);
    }

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = cfg.fifo_wmk;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09915_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09915_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09915_MIN_RANGE;
    phy_sensor_config.range[1] = AK09915_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = true;
    break;
  case AK09916C:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09916_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09916_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09916_MIN_RANGE;
    phy_sensor_config.range[1] = AK09916_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = false;
    break;
  case AK09916D:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09916_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09916_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09916_MIN_RANGE;
    phy_sensor_config.range[1] = AK09916_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = true;
    break;
  case AK09917:

    if (state->mag_info.sdr == 0)
    {
      operating_mode = AK0991X_LOW_NOISE;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_NOISE);
    }
    else
    {
      operating_mode = AK0991X_LOW_POWER;
      op_mode_str_len = ARR_SIZE(AK0991X_LOW_POWER);
    }

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = cfg.fifo_wmk;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09917_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09917_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09917_MIN_RANGE;
    phy_sensor_config.range[1] = AK09917_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = true;
    break;
  case AK09918:
    operating_mode = AK0991X_NORMAL;
    op_mode_str_len = ARR_SIZE(AK0991X_NORMAL);
    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = 1;//1 if FIFO not in use.
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09918_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09918_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09918_MIN_RANGE;
    phy_sensor_config.range[1] = AK09918_MAX_RANGE;
    phy_sensor_config.has_dri_enabled = false;
    break;
  default:
    return SNS_RC_FAILED;
  }

  //Set config event for s4s
  ak0991x_s4s_send_config_event(instance, &phy_sensor_config);

  op_mode_args.buf = operating_mode;
  op_mode_args.buf_len = op_mode_str_len;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_sample_rate    = true;
  phy_sensor_config.sample_rate        = ak0991x_get_mag_odr(cfg.odr);
  phy_sensor_config.has_DAE_watermark  = ak0991x_dae_if_available(instance);
  phy_sensor_config.DAE_watermark      = SNS_MAX(cfg.dae_wmk, 1);
  phy_sensor_config.dri_enabled        = (state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING);
  phy_sensor_config.has_sync_ts_anchor = state->has_sync_ts_anchor;
  phy_sensor_config.sync_ts_anchor     = state->sync_ts_anchor;

  if( is_new_config )
  {
    state->config_set_time = sns_get_system_time();
  }

  AK0991X_INST_PRINT(HIGH, instance,
                     "tx PHYSICAL_CONFIG_EVENT Time %u : rate %u wm %u dae_wm %u is_new_config %d",
                     (uint32_t)state->config_set_time,
                     (uint32_t)(phy_sensor_config.sample_rate),
                     phy_sensor_config.water_mark,
                     phy_sensor_config.DAE_watermark,
                     is_new_config);

  pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                state->config_set_time,
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &state->mag_info.suid);

  state->mag_info.last_sent_cfg = cfg;

  if( is_new_config )
  {
    // done new config process
    state->processing_new_config = false;
  }

  return SNS_RC_SUCCESS;
}

void ak0991x_register_interrupt(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_data_stream* data_stream;

  if(!state->irq_info.is_registered)
  {
    uint8_t buffer[20];
    sns_request irq_req;
    const pb_field_t *fields;
    const void *payload = NULL;
    sns_ibi_req ibi_config =
    {
      .dynamic_slave_addr = state->com_port_info.com_config.slave_control,
      .bus_instance = state->com_port_info.com_config.bus_instance,
      .ibi_data_bytes = 0,
    };

    data_stream = state->interrupt_data_stream;
    irq_req.request = buffer;
    if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_IBI)
    {
      irq_req.message_id = SNS_INTERRUPT_MSGID_SNS_IBI_REQ;
      fields = sns_ibi_req_fields;

      payload = &ibi_config;
      SNS_INST_PRINTF(HIGH, this, "Registering IBI...");
    }
    else
    {
      irq_req.message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ;
      fields = sns_interrupt_req_fields;
      payload = &state->irq_info.irq_config;
      SNS_INST_PRINTF(HIGH, this, "Registering IRQ...");
    }

    irq_req.request_len = pb_encode_request(buffer,
                                            sizeof(buffer),
                                            payload,
                                            fields,
                                            NULL);
    if(irq_req.request_len > 0)
    {
      data_stream->api->send_request(data_stream, &irq_req);
      state->irq_info.is_registered = true;
    }
  }
  if(!state->irq_info.is_registered)
  {
    SNS_INST_PRINTF(ERROR, this, "Interrupt reg failed stream=%x", 
                    state->interrupt_data_stream);
  }
}

static sns_rc ak0991x_send_timer_request(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  sns_request             timer_req;
  size_t                  req_len = 0;
  uint8_t                 buffer[100];
  sns_rc rv = SNS_RC_SUCCESS;

  sns_timer_sensor_config payload = sns_timer_sensor_config_init_default;

  if (state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
  {
    payload = state->req_payload;
  }

  if (NULL == state->timer_data_stream)
  {
    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                   this,
                                                   state->timer_suid,
                                                   &state->timer_data_stream
                                                   );
    AK0991X_INST_PRINT(LOW, this, "create timer data stream");
  }

  if (NULL != state->timer_data_stream)
  {
    req_len = pb_encode_request(buffer,
                                sizeof(buffer),
                                &payload,
                                sns_timer_sensor_config_fields,
                                NULL);
    if (req_len > 0)
    {
      timer_req.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG;
      timer_req.request_len = req_len;
      timer_req.request = buffer;
      /** Send encoded request to Timer Sensor */
      rv = state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
    }
  }
  if (req_len == 0)
  {
    rv = SNS_RC_FAILED;
  }
  return rv;
}


static sns_rc ak0991x_set_timer_request_payload(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  sns_rc rv = SNS_RC_SUCCESS;

  if(state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
  {
    sns_time sample_period = ak0991x_get_sample_interval(state->mag_info.cur_cfg.odr);
    state->system_time = sns_get_system_time();

    // for heart beat timer
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
    {
      req_payload.has_priority = true;
      req_payload.priority = SNS_TIMER_PRIORITY_OTHER;
      req_payload.is_periodic = false;
      req_payload.start_time = state->system_time;

      // Set timeout_period for heart beat in DRI/FIFO+DRI
      // as 5 samples time for DRI
      // or 5 FIFO buffers time for FIFO+DRI
      if (state->mag_info.use_fifo)
      {
        if(state->in_clock_error_procedure)
        {
          // 100Hz dummy measurement fixed.
          req_payload.timeout_period = sns_convert_ns_to_ticks(10 * 1000 * 1000) * 5 * 11 / 10;
        }
        else
        {
          if(!ak0991x_dae_if_available(this))
          {
            sns_time max_timeout = (state->mag_info.max_fifo_size * sample_period) * 11 / 10;
            req_payload.timeout_period = sample_period * 5 * state->mag_info.cur_cfg.fifo_wmk;
            if(req_payload.timeout_period > max_timeout)
            {
              req_payload.timeout_period = max_timeout; // to avoid large data gap
            }
          }
          else  // DAE enabled
          {
            req_payload.timeout_period = (sample_period* 5 * state->mag_info.cur_cfg.dae_wmk) * 11 / 10;
          }
        }
      }
      else
      {
        req_payload.timeout_period = sample_period * 5;
      }
      state->hb_timer_fire_time = req_payload.start_time + req_payload.timeout_period;
      AK0991X_INST_PRINT(LOW, this, "Register HB timer. Fire time= %u", (uint32_t)state->hb_timer_fire_time);
    }
    // for polling timer
    else
    {
      // Wait measurement time + margin(2msec)
      sns_time delay = (2 * state->half_measurement_time) + sns_convert_ns_to_ticks(2 * 1000 * 1000);

      // DAE streaming is already started. But sensor is not started yet on DAE. 
      // Required additional delay for the first sample to ignore UNRELIABLE data received.
      if( ak0991x_dae_if_available(this) &&
          (state->mag_info.previous_lsbdata[TRIAXIS_X] == 0) &&
          (state->mag_info.previous_lsbdata[TRIAXIS_Y] == 0) &&
          (state->mag_info.previous_lsbdata[TRIAXIS_Z] == 0))
      {
        delay += (2 * state->half_measurement_time);
      }
      req_payload.has_priority = true;
      req_payload.priority = SNS_TIMER_PRIORITY_POLLING;
      req_payload.is_periodic = true;
      req_payload.start_time = state->system_time - sample_period + delay;  // add delay for RELIABLE data sample
      req_payload.start_config.early_start_delta = 0;
      req_payload.start_config.late_start_delta = sample_period;
      req_payload.timeout_period = ak0991x_set_heart_beat_timeout_period_for_polling(this);
      req_payload.has_is_dry_run = true;
      req_payload.is_dry_run = ak0991x_dae_if_available(this);
      AK0991X_INST_PRINT(LOW, this, "polling timer now= %u start= %u, delta= %u, pre_timestamp= %u",
          (uint32_t)state->system_time,
          (uint32_t)req_payload.start_time,
          (uint32_t)req_payload.start_config.late_start_delta,
          (uint32_t)state->pre_timestamp);
    }

    // reset request payload
    state->req_payload = req_payload;
  }
  else
  {
    AK0991X_INST_PRINT(LOW, this, "Timer request payload failed.");
    rv = SNS_RC_FAILED;
  }
  return rv;
}

void ak0991x_unregister_heart_beat_timer(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  if(state->timer_data_stream != NULL && state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING )
  {
    sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
    AK0991X_INST_PRINT(LOW, this, "Unregister HB timer");
  }
}

// Only perform for DRI modes
void ak0991x_register_heart_beat_timer(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  state->heart_beat_attempt_count = 0;

  // clear remained HB events
  if (NULL != state->timer_data_stream)
  {
    sns_sensor_event *event =
      state->timer_data_stream->api->peek_input(state->timer_data_stream);

    while (NULL != event)
    {
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  if( !state->in_clock_error_procedure &&
      (state->mag_info.flush_only || state->mag_info.max_batch) )
  {
    ak0991x_unregister_heart_beat_timer(this);
  }
  else
  {
    ak0991x_register_timer(this); // register HB timer for DRI mode
  }
}

void ak0991x_register_timer(sns_sensor_instance *const this)
{
  if( SNS_RC_SUCCESS == ak0991x_set_timer_request_payload(this) )
  {
    if( SNS_RC_SUCCESS != ak0991x_send_timer_request(this) )
    {
      SNS_INST_PRINTF(ERROR, this, "Failed send timer request");
    }
  }
}

sns_rc ak0991x_get_meas_time( akm_device_type device_select,
                              uint8_t sdr,
                              sns_time* meas_ts )
{
  sns_time usec_time_for_measure;
  switch(device_select)
  {
  case AK09918:
    usec_time_for_measure = AK09918_TIME_FOR_MEASURE_US;
    break;
  case AK09917:
    if (sdr == 0)
    {
      usec_time_for_measure = AK09917_TIME_FOR_LOW_NOISE_MODE_MEASURE_US;
    }
    else
    {
      usec_time_for_measure = AK09917_TIME_FOR_LOW_POWER_MODE_MEASURE_US;
    }
    break;
  case AK09916C:
    usec_time_for_measure = AK09916_TIME_FOR_MEASURE_US;
    break;
  case AK09916D:
    usec_time_for_measure = AK09916_TIME_FOR_MEASURE_US;
    break;
  case AK09915C:
    if (sdr == 1)
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_NOISE_MODE_MEASURE_US;
    }
    else
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_POWER_MODE_MEASURE_US;
    }
    break;
  case AK09915D:
    if (sdr == 1)
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_NOISE_MODE_MEASURE_US;
    }
    else
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_POWER_MODE_MEASURE_US;
    }
    break;
  case AK09913:
    usec_time_for_measure = AK09913_TIME_FOR_MEASURE_US;
    break;
  case AK09912:
    usec_time_for_measure = AK09912_TIME_FOR_MEASURE_US;
    break;
  case AK09911:
    usec_time_for_measure = AK09911_TIME_FOR_MEASURE_US;
    break;
  default:
    return SNS_RC_FAILED;
  }
  UNUSED_VAR(sdr);
  *meas_ts = usec_time_for_measure;
  return SNS_RC_SUCCESS;
}

sns_rc ak0991x_reconfig_hw(sns_sensor_instance *this, bool reset_device)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  // ignore reconfig. same setting.
  if( state->only_dae_wmk_is_changed )
  {
    SNS_INST_PRINTF(HIGH, this, "reconfig_hw ignored because same ODR and WM.");
    return SNS_RC_SUCCESS;
  }
  else
  {
    SNS_INST_PRINTF(HIGH, this, "reconfig_hw: reset=%u", reset_device);
  }
  
  if(reset_device)
  {
    rv = ak0991x_device_sw_reset(this, state->scp_service, &state->com_port_info);
    AK0991X_INST_PRINT(HIGH, this, "ak0991x_device_sw_reset.");
  }
  else
  {
    ak0991x_enter_i3c_mode(this, &state->com_port_info, state->scp_service);
  }

  if (state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
  {
    // update averaged_interval
    ak0991x_reset_averaged_interval(this);

    AK0991X_INST_PRINT(HIGH, this, "reconfig_hw: irq_ready=%u", state->irq_info.is_ready);
    if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING ||
        state->irq_info.is_ready ||
        ak0991x_dae_if_is_streaming(this))
    {
      rv = ak0991x_start_mag_streaming(this);
      if(rv != SNS_RC_SUCCESS)
      {
        SNS_INST_PRINTF(ERROR, this, "reconfig_hw: failed to start");
      }
      else if(!state->in_clock_error_procedure && state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) // on DRI mode
      {
        // if config was updated, send correct config.
        if( ( !ak0991x_dae_if_is_streaming(this) && state->mag_info.cur_cfg.num > state->mag_info.last_sent_cfg.num ) ||
            ( ak0991x_dae_if_is_streaming(this) && state->mag_info.cur_cfg.num - state->mag_info.last_sent_cfg.num > 1 ) )  // wait for in order to send config in DAE
        {
          // config changed. send config event if non DAE mode
          AK0991X_INST_PRINT(MED, this, "Send new config: odr=0x%02X fifo_wmk=%d",
              (uint32_t)state->mag_info.cur_cfg.odr,
              (uint32_t)state->mag_info.cur_cfg.fifo_wmk);
          ak0991x_send_config_event(this, true);  // send new config event
          ak0991x_send_cal_event(this, false);    // send previous cal event
        }
      }
    }
  }
  else
  {
    rv = ak0991x_stop_mag_streaming(this);
  }

  if( state->in_clock_error_procedure )
  {
    AK0991X_INST_PRINT(HIGH, this, "reconfig_hw: in clock error procedure.");
  }
  else
  {
    AK0991X_INST_PRINT(HIGH, this, "reconfig_hw: reset=%u, ODR=%d result=%d",
        reset_device, state->mag_info.cur_cfg.odr, rv);
  }
  return rv;
}

static void ak0991x_inst_exit_island(sns_sensor_instance *this)
{
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
}

sns_rc ak0991x_heart_beat_timer_event(sns_sensor_instance *const this)
{
 ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
 sns_rc rv = SNS_RC_SUCCESS;

 if(state->mag_info.cur_cfg.odr == AK0991X_MAG_ODR_OFF)
 {
   AK0991X_INST_PRINT(HIGH, this, "heart beat timer event is skipped since ODR=0.");
   return rv;
 }

 if (state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
 {
   SNS_INST_PRINTF(ERROR, this, "Detect streaming has stopped #HB= %u start_time= %u period = %u fire_time %u now= %u",
                        state->heart_beat_attempt_count,
                        (uint32_t)state->req_payload.start_time,
                        (uint32_t)state->req_payload.timeout_period,
                        (uint32_t)state->hb_timer_fire_time,
                        (uint32_t)state->system_time);
   // Streaming is unable to resume after 4 attempts
   if (state->heart_beat_attempt_count >= 4)
   {
     ak0991x_inst_exit_island(this);
     SNS_INST_PRINTF(ERROR, this, "Streaming is unable to resume after 3 attempts");
     rv = SNS_RC_INVALID_STATE;
   }
   // Perform a reset operation in an attempt to revive the sensor
   else
   {
     state->heart_beat_attempt_count++;
     if(ak0991x_dae_if_available(this))
     {
       ak0991x_dae_if_flush_hw(this);
     }
     else
     {
       ak0991x_read_mag_samples(this);
       if(state->heart_beat_attempt_count >= 3)
       {
         ak0991x_inst_exit_island(this);
         ak0991x_reconfig_hw(this, true);
         // Indicate streaming error
         rv = SNS_RC_NOT_AVAILABLE;
       }
     }
   }
 }
 else // polling
 {
   uint8_t heart_beat_thresthold =
     ( state->mag_info.use_fifo )? 1 : 4;
   if (state->heart_beat_sample_count < heart_beat_thresthold)
   {
     state->heart_beat_sample_count++;
   }
   else
   {
     AK0991X_INST_PRINT(LOW, this, "heart_beat_gap=%u, heart_beat_timeout=%u",
       (uint32_t)(state->interrupt_timestamp - state->heart_beat_timestamp),
       (uint32_t)state->heart_beat_timeout_period);
     // Detect streaming has stopped
     if (state->interrupt_timestamp > state->heart_beat_timestamp + state->heart_beat_timeout_period)
     {
       AK0991X_INST_PRINT(HIGH, this, "Detect streaming has stopped. int_timestamp:%u hb_timestamp:%u hb_period:%u",
           (uint32_t)state->interrupt_timestamp,
           (uint32_t)state->heart_beat_timestamp,
           (uint32_t)state->heart_beat_timeout_period);
       // Streaming is unable to resume after 3 attempts
       if (state->heart_beat_attempt_count >= 3)
       {
         ak0991x_inst_exit_island(this);
         SNS_INST_PRINTF(ERROR, this, "Streaming is unable to resume after 3 attempts");
         rv = SNS_RC_INVALID_STATE;
       }
       // Perform a reset operation in an attempt to revive the sensor
       else
       {
         ak0991x_inst_exit_island(this);
         ak0991x_reconfig_hw(this, true);
         // Indicate streaming error
         rv = SNS_RC_NOT_AVAILABLE;
         state->heart_beat_attempt_count++;
       }
     }
     else
     {
       state->heart_beat_timestamp = state->interrupt_timestamp;
       state->heart_beat_sample_count = 0;
       state->heart_beat_attempt_count = 0;
     }
   }
 }

 return rv;
}


/**
 * Runs a communication test - verifies WHO_AM_I, publishes self
 * test event.
 *
 * @param[i] instance    Instance reference
 * @param[i] uid         Sensor UID
 *
 * @return none
 */
static void ak0991x_send_com_test_event(sns_sensor_instance *instance,
                                        sns_sensor_uid *uid, bool test_result,
                                        sns_physical_sensor_test_type test_type)
{
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_physical_sensor_test_event test_event =
    sns_physical_sensor_test_event_init_default;

  test_event.test_passed = test_result;
  test_event.test_type = test_type;
  test_event.test_data.funcs.encode = &pb_encode_string_cb;
  test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                uid);
}

/** See sns_ak0991x_hal.h */
void ak0991x_run_self_test(sns_sensor_instance *instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  AK0991X_INST_PRINT(HIGH, instance, "run_self_test:: present=%u type=%u",
                     state->mag_info.test_info.test_client_present,
                     state->mag_info.test_info.test_type);

  if(state->mag_info.test_info.test_client_present)
  {
    if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      uint8_t buffer[AK0991X_NUM_READ_DEV_ID] = {0};
      bool who_am_i_success = false;

      ak0991x_enter_i3c_mode(instance, &state->com_port_info, state->scp_service);

      rv = ak0991x_get_who_am_i(state->scp_service,
                                state->com_port_info.port_handle,
                                &buffer[0]);

      if(rv == SNS_RC_SUCCESS
         &&
         buffer[0] == AK0991X_WHOAMI_COMPANY_ID)
      {
        who_am_i_success = true;
        AK0991X_INST_PRINT(LOW, instance, "COM self-test success!!");
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, instance, "COM self-test failed!!");
      }
      ak0991x_send_com_test_event(instance, &state->mag_info.suid, who_am_i_success,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test in progress.
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
    {
      bool hw_success = false;
      uint32_t err;

      ak0991x_enter_i3c_mode(instance, &state->com_port_info, state->scp_service);

      AK0991X_INST_PRINT(LOW, instance, "hw self-test start!");
      rv = ak0991x_hw_self_test(instance, &err);

      if(rv == SNS_RC_SUCCESS
         &&
         err == 0)
      {
        hw_success = true;
        AK0991X_INST_PRINT(LOW, instance, "hw self-test success!!");
      }
      ak0991x_send_com_test_event(instance, &state->mag_info.suid, hw_success,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_HW);
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_SW)
    {
      AK0991X_INST_PRINT(LOW, instance, "Not supported. type=%d",(uint8_t)state->mag_info.test_info.test_type);
      rv = SNS_RC_NOT_SUPPORTED;
    }
    state->mag_info.test_info.test_client_present = false;
  }
}

