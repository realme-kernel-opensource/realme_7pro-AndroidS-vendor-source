/**
 * @file sns_mmc5603x_hal_island.c
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
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

#include "sns_mmc5603x_hal.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_s4s.h"

#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#ifdef MMC5603X_ENABLE_DIAG_LOGGING
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#endif

#include "sns_cal_util.h"
#include "sns_cal.pb.h"
#include "sns_sensor_util.h"

#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_fb_utils.h"
#endif


//#define MMC5603X_VERBOSE_DEBUG             // Define to enable extra debugging

#define SELFTEST_DURATION_TIME  (15)   //15ms


/** Need to use ODR table. */
extern const odr_reg_map reg_map_mmc5603x[MMC5603X_REG_MAP_TABLE_SIZE];

log_sensor_state_raw_info log_mag_state_raw_info;
void mmc5603x_enable_measure(sns_sensor_instance *const this);
static void mmc5603x_set_timer_request_payload(sns_sensor_instance *const this );

void mmc5603x_disable_measure(sns_sensor_instance *const this);
void mmc5603x_enable_measure(sns_sensor_instance *const this);


uint8_t st_thd_reg[3];// save strim data from reg 0x27-0x29
enum Read_Mode {
  CONTINUOUS_MODE_AUTO_SR=1,
  SINGLE_MODE=2
};
enum Read_Mode MMC5603_Read_mode = CONTINUOUS_MODE_AUTO_SR;
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
sns_rc mmc5603x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[MMC5603X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = MMC5603X_NUM_AXES,
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
  mmc5603x_batch_sample *raw_sample = (mmc5603x_batch_sample *)log;

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

    parsed_log_size += sizeof(mmc5603x_batch_sample);
    i++;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }
#else
  UNUSED_VAR(log);
  UNUSED_VAR(log_size);
  UNUSED_VAR(encoded_log_size);
  UNUSED_VAR(encoded_log);
  UNUSED_VAR(bytes_written);
#endif
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
void mmc5603x_log_sensor_state_raw_alloc(
  log_sensor_state_raw_info *log_raw_info,
  uint32_t log_size)
{
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
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
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(log_size);
#endif
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void mmc5603x_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  bool batch_complete)
{
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
  mmc5603x_batch_sample *sample = NULL;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == log_raw_info->log)
  {
    return;
  }

  sample = (mmc5603x_batch_sample *)log_raw_info->log;

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
        mmc5603x_encode_log_sensor_state_raw);

  log_raw_info->log = NULL;
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(batch_complete);
#endif
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
sns_rc mmc5603x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  sns_time timestamp,
  sns_std_sensor_sample_status status)
{
  sns_rc rc = SNS_RC_SUCCESS;
#ifdef MMC5603X_ENABLE_DIAG_LOGGING

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == raw_data || NULL == log_raw_info->log)
  {
    return SNS_RC_FAILED;
  }

  if( (log_raw_info->bytes_written + sizeof(mmc5603x_batch_sample)) >
     log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    mmc5603x_log_sensor_state_raw_submit(log_raw_info, false);
    mmc5603x_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    mmc5603x_batch_sample *sample =
        (mmc5603x_batch_sample *)log_raw_info->log;

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

    log_raw_info->bytes_written += sizeof(mmc5603x_batch_sample);

    log_raw_info->log_sample_cnt++;
    log_raw_info->batch_sample_cnt++;
  }
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(raw_data);
  UNUSED_VAR(timestamp);
  UNUSED_VAR(status);
#endif
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
static sns_rc mmc5603x_com_read_wrapper(sns_sync_com_port_service * scp_service,
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
	  fb_event.buff[0] = 0;
	  fb_event.buff[1] = (int)reg_addr;
	  fb_event.buff[2] = (int)rc;
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
sns_rc mmc5603x_com_write_wrapper(sns_sync_com_port_service * scp_service,
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
	  fb_event.buff[0] = 1;
	  fb_event.buff[1] = (int)reg_addr;
	  fb_event.buff[2] = (int)rc;
	  oplus_add_fd_event(&fb_event);
  }
  #endif
 return rc;
}
sns_rc mmc5603x_read_modify_write(
    sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle,
    uint32_t reg_addr,
    uint8_t *reg_value,
    uint32_t size,
    uint32_t *xfer_bytes,
    bool save_write_time,
    uint8_t mask)
{
    uint8_t rw_buffer = 0;
    uint32_t rw_bytes = 0;

    if((size > 1) || (mask == 0xFF) || (mask == 0x00))
    {
        mmc5603x_com_write_wrapper(scp_service,
                                    port_handle,
                                    reg_addr,
                                    &reg_value[0],
                                    size,
                                    xfer_bytes,
                                    save_write_time);
    }
    else
    {
        // read current value from this register
        mmc5603x_com_read_wrapper(scp_service,
                                   port_handle,
                                   reg_addr,
                                   &rw_buffer,
                                   1,
                                   &rw_bytes);

        // generate new value
        rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

        // write new value to this register
        mmc5603x_com_write_wrapper(scp_service,
                                    port_handle,
                                    reg_addr,
                                    &rw_buffer,
                                    1,
                                    xfer_bytes,
                                    save_write_time);

    }

    return SNS_RC_SUCCESS;;
}

static void mmc5603x_clear_old_events(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)instance->state->state;
  sns_sensor_event    *event;

  // Handle timer event
  if (NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);

    while (NULL != event && SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Old timer event detected. Cleared.");
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }
}



sns_rc mmc5603x_enter_i3c_mode(sns_sensor *const this,
	                           sns_sensor_instance *const instance,
                              mmc5603x_com_port_info *com_port,
                              sns_sync_com_port_service * scp_service)
{
  sns_rc                       rv = SNS_RC_FAILED;
#ifdef MMC5603X_ENABLE_I3C_SUPPORT

  sns_sync_com_port_handle    *i2c_port_handle = NULL;
  sns_com_port_config          i2c_com_config = com_port->com_config;
  uint32_t                     xfer_bytes;
  uint8_t                      buffer[6];

  if(com_port->com_config.bus_type != SNS_BUS_I3C_SDR &&
     com_port->com_config.bus_type != SNS_BUS_I3C_HDR_DDR )
  {
    return SNS_RC_FAILED;
  }

 /// mmc5603nj_prepare_i3c_mode(instance, com_port, scp_service);//memsic add 

  if(NULL != instance)
  {
    MMC5603X_INST_PRINT(ERROR, instance, "enter i3c mode");
  }
  if(com_port->in_i3c_mode)
  {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(MED, instance, "already in i3c mode");
    }
    return SNS_RC_SUCCESS;
  }

  i2c_com_config.slave_control = com_port->i2c_address;
 // MMC5603X_INST_PRINT(ERROR, instance, "enter_i3c_mode: i2c_com_config.slave_control = %d i2c_com_config.bus_type = %d", i2c_com_config.slave_control, i2c_com_config.bus_type);
  SNS_PRINTF(ERROR, this, "memsic 444  i2c_com_config.slave_control %d", i2c_com_config.slave_control);

  rv = scp_service->api->sns_scp_register_com_port(&i2c_com_config, &i2c_port_handle);
  if( rv != SNS_RC_SUCCESS )
  {
    if (NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "i3c_mode: register_com_port() rv=%d", rv);
    }
    return SNS_RC_FAILED;
  }
  rv = scp_service->api->sns_scp_open(i2c_port_handle);
  if( rv != SNS_RC_SUCCESS )
  {
    if (NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "i3c_mode: open() rv=%d", rv);
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
  //com_port->in_i3c_mode = true;
  if(NULL != instance)
  {
    if(rv == SNS_RC_SUCCESS)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "I3C address assigned: 0x%x",((uint32_t)buffer[0])>>1);
    }
    else
    {
      MMC5603X_INST_PRINT(ERROR, instance, "assign i3c address failed; rv=%d", rv);
    }
  }  

  /**-------------Set max read size to the size of the FIFO------------------*/
#if 0 
  if(com_port->port_handle != NULL)
  {
    buffer[0] = (uint8_t)((MMC5603NJ_MAX_FIFO_SIZE >> 8) & 0xFF);
    buffer[1] = (uint8_t)(MMC5603NJ_MAX_FIFO_SIZE & 0xFF);
    buffer[2] = 0;
    rv = scp_service->api->
      sns_scp_issue_ccc( com_port->port_handle,
                         SNS_SYNC_COM_PORT_CCC_SETMRL,
                         buffer, 3, &xfer_bytes );
    if( rv != SNS_RC_SUCCESS ) {
      if(NULL != instance)
      {
        MMC5603X_INST_PRINT(ERROR, instance, "Set max read length failed! rv=%d hndl=0x%x", 
                        rv, com_port->port_handle);
      }
    }
  }
#endif
  /**-------------------Disable IBI------------------------*/
  if(com_port->port_handle != NULL)
  {
    buffer[0] = 0x1;
    rv = scp_service->api->
      sns_scp_issue_ccc( com_port->port_handle,
                         SNS_SYNC_COM_PORT_CCC_DISEC,
                         buffer, 1, &xfer_bytes );
	SNS_PRINTF(ERROR, this, "memsic 99999 ");
    if(NULL != instance)
    {
      if( rv == SNS_RC_SUCCESS ) {
        MMC5603X_INST_PRINT(ERROR, instance, "IBI disabled");
      } else {
        MMC5603X_INST_PRINT(ERROR, instance, "IBI disable FAILED! rv=%d hndl=0x%x", 
                        rv, com_port->port_handle);
      }
    }
  }

#ifdef MMC5603X_ENABLE_I3C_DEBUG
  /**-------------------Debug -- read all CCC info------------------------*/
  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMWL,
                       buffer, 2, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "max write length:0x%02x%02x", buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get max write length failed!");
    }
  }
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETMWL,
                       buffer, 2, &xfer_bytes );
  if( rv != SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Set max write length failed!");
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
      MMC5603X_INST_PRINT(ERROR, instance, "max read length:0x%02x%02x",
                      buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get max read length failed!");
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
      MMC5603X_INST_PRINT(ERROR, instance, "PID:0x%02x%02x:%02x%02x:%02x%02x",
                      buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get PID failed!");
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
      MMC5603X_INST_PRINT(ERROR, instance, "bus charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get BCR failed!");
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
      MMC5603X_INST_PRINT(LOW, instance, "device charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get DCR failed!");
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
      MMC5603X_INST_PRINT(LOW, instance, "status register:0x%x", (buffer[0] | buffer[1] << 8));
    }
  } else {
    if(NULL != instance)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "Get status failed!");
    }
  }
    
#  endif /*  MMC5603NJ_ENABLE_DEBUG_MSG */
#else /* MMC5603NJ_ENABLE_I3C_SUPPORT */
  UNUSED_VAR(instance);
  UNUSED_VAR(this);
  UNUSED_VAR(com_port);
  UNUSED_VAR(scp_service);
#endif /* MMC5603NJ_ENABLE_I3C_SUPPORT */
  return rv;
}

/**
 * see sns_mmc5603x_hal.h
 */
sns_rc mmc5603x_device_sw_reset(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle)
{
  uint8_t  buffer[1];
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  // clear old events
  if(this != NULL)
  {
    mmc5603x_clear_old_events(this);
  }
  buffer[0] = MMC5603X_SOFT_RESET;
  rv = mmc5603x_com_write_wrapper(scp_service,
                                 port_handle,
                                 MEMSIC_MMC5603X_REG_CNTL3,
                                 &buffer[0],
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }
  return SNS_RC_SUCCESS;
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
sns_rc mmc5603x_set_mag_config(sns_sensor_instance *const this,
                              bool force_off )
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)(this->state->state);
  mmc5603x_mag_odr desired_odr = state->mag_info.desired_odr;

  if( force_off )
  {
    desired_odr = MMC5603X_MAG_ODR_OFF;
  }

  MMC5603X_INST_PRINT(ERROR, this, "set_mag_config: ODR: %d  wmk:%d",
                                desired_odr,
                                state->mag_info.cur_wmk );

  sns_rc rv = SNS_RC_SUCCESS;
  // Set mag config for S4S
  rv = mmc5603x_s4s_set_mag_config(this);
  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  return SNS_RC_SUCCESS;
}



/**
 * see sns_mmc5603x_hal.h
 */
sns_rc mmc5603x_start_mag_streaming(sns_sensor_instance *const this )
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)(this->state->state);
  sns_rc rv;
  sns_time meas_usec;

  MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_start_mag_streaming.");

  if(state->ascp_xfer_in_progress)
  {
    MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_start_mag_streaming skipped. wait for the ASCP done.");
    state->config_mag_after_ascp_xfer = true;
    return SNS_RC_SUCCESS;
  }

  // Enable Mag Streaming

  //Transit to Power-down mode first and then transit to other modes.
  rv = mmc5603x_set_mag_config(this, true);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // Wait at least 100usec after power down-mode before setting another mode.
  sns_busy_wait(sns_convert_ns_to_ticks(MMC5603X_TWAIT_USEC * 1000 * 2));

  rv = mmc5603x_set_mag_config(this, false);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->system_time = sns_get_system_time();
  mmc5603x_get_meas_time(state->mag_info.device_select, state->mag_info.sdr, &meas_usec);
  state->this_is_first_data = true;
  state->mag_info.data_count = 0;
  state->mag_info.curr_odr = state->mag_info.desired_odr;
  state->heart_beat_sample_count = 0;
  state->heart_beat_timestamp = state->system_time;
  state->irq_event_time = state->system_time;
  state->reg_event_done = false;
  state->is_temp_average = false;
  state->irq_info.detect_irq_event = false;
  state->previous_meas_is_irq = false;
  state->previous_meas_is_correct_wm = true;
#ifdef MMC5603X_ENABLE_S4S
  state->s4s_reg_event_done = false;
  state->mag_info.s4s_sync_state = MMC5603X_S4S_NOT_SYNCED;
#endif

  state->half_measurement_time = ((sns_convert_ns_to_ticks(meas_usec * 1000) * state->internal_clock_error) >> MMC5603X_CALC_BIT_RESOLUTION)>>1;
  state->nominal_intvl = mmc5603x_get_sample_interval(state->mag_info.curr_odr);
  state->averaged_interval = (state->nominal_intvl * state->internal_clock_error) >> MMC5603X_CALC_BIT_RESOLUTION;

  if(state->mag_info.use_dri)
  {
    state->pre_timestamp = state->system_time + (state->half_measurement_time<<1) - state->averaged_interval;
  }
  else
  {
    state->pre_timestamp = state->system_time;
  }
  state->previous_irq_time = state->pre_timestamp;

  MMC5603X_INST_PRINT(ERROR, this, "start_mag_streaming at %u pre_ts %u avg %u   state->mag_info.curr_odr =%d",
                     (uint32_t)state->system_time, (uint32_t)state->pre_timestamp,
                     (uint32_t)state->averaged_interval, state->mag_info.curr_odr);


  return SNS_RC_SUCCESS;
}

/**
 * see sns_mmc5603x_hal.h
 */
sns_rc mmc5603x_stop_mag_streaming(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)(this->state->state);
  sns_rc rv;

  // Disable Mag Streaming
  MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_stop_mag_streaming");

  // remove timer instance stream
  if (NULL != state->timer_data_stream)
  {
    sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
  }

  rv = mmc5603x_set_mag_config(this, true );

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->mag_info.curr_odr = state->mag_info.desired_odr;

  return SNS_RC_SUCCESS;
}

/**
 * see sns_mmc5603x_hal.h
 */
sns_rc mmc5603x_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

   rv = mmc5603x_com_read_wrapper(scp_service,
                                    port_handle,
                                    MMC5603NJ_REG_WHO_AM_I,
                                    buffer,
                                    1,
                                    &xfer_bytes);
  if (rv != SNS_RC_SUCCESS)
  {
    #ifdef OPLUS_FEATURE_SENSOR_FB
	struct fb_event fb_event;
	memset(&fb_event, 0, sizeof(struct fb_event));
	fb_event.event_id = MAG_INIT_FAIL_ID;
	fb_event.buff[0] = (int)rv;
	fb_event.buff[1] = (int)buffer[0];
	fb_event.buff[2] = (int)MMC5603NJ_REG_WHO_AM_I;
	oplus_add_fd_event(&fb_event);
    #endif
    rv = SNS_RC_FAILED;
  }

  return rv;
}

sns_rc mmc5603x_get_trim_data(sns_sync_com_port_service *scp_service,
                              sns_sync_com_port_handle *port_handle
                              )
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t reg_value[3];
  int16_t st_thr_data[3] = {0};
  int16_t st_thr_new[3] = {0};

  int16_t st_thd[3] = {0};
  int i=0;
  //uint8_t st_thd_reg[3];

  rv = mmc5603x_com_read_wrapper(scp_service,
                                 port_handle,
                                 MMC5603NJ_REG_ST_X_VAL,
                                 reg_value,
                                 3,
                                 &xfer_bytes);

  for (i = 0; i < 3; i++)
  {
    st_thr_data[i] = (int16_t)(reg_value[i] - 128) * 32;
    if (st_thr_data[i] < 0)
      st_thr_data[i] = -st_thr_data[i];
    st_thr_new[i] = st_thr_data[i] - st_thr_data[i] / 5;

    st_thd[i] = st_thr_new[i] / 8;
    if (st_thd[i] > 255)
      st_thd_reg[i] = 0xFF;
    else
      st_thd_reg[i] = (uint8_t)st_thd[i];
  }

  if (rv != SNS_RC_SUCCESS)
  {
    rv = SNS_RC_FAILED;
  }
  return rv;
}

/**
 * Start the saturation checking
 * @param[i]            reference to the instance
 * @param[o]            error code
 *
 * @return bool
 */

/*********************************************************************************
* decription: Auto self-test
*********************************************************************************/
static int mmc5603x_saturation_check_start (sns_sensor_instance * const this)
{
  uint8_t rw_buffer = 0x00;
  uint32_t xfer_bytes;
  mmc5603x_instance_state * state = (mmc5603x_instance_state *)this->state->state;

  /* Write 0x40 to register 0x08, set Auto_st_en bit high */
  rw_buffer = MMC5603NJ_CMD_AUTO_ST_EN;
  mmc5603x_read_modify_write (state->scp_service,
    state->com_port_info.port_handle,
    MMC5603NJ_REG_CTRL0,
    &rw_buffer,
    1,
    &xfer_bytes,
    false,
    0xFF);

  return 1;
}


/**
 * Stop the saturation checking
 * @param[i]            reference to the instance
 * @param[o]            error code
 *
 * @return bool
 */

/*********************************************************************************
* decription: Auto self-test
*********************************************************************************/
static int mmc5603x_saturation_check_stop (sns_sensor_instance * const this)
{
  uint8_t reg_status = 0;
  uint32_t xfer_bytes;
  mmc5603x_instance_state * state = (mmc5603x_instance_state *)this->state->state;

  /* Read Sat_sensor bit */
  mmc5603x_com_read_wrapper (state->scp_service,
    state->com_port_info.port_handle,
    MMC5603NJ_REG_STATUS1,
    &reg_status,
    1,
    &xfer_bytes);

  if ((reg_status & MMC5603NJ_SAT_SENSOR))
  {
    return - 1;
  }

  return 2;
}
	/*********************************************************************************
	* decription: SET operation 
	*********************************************************************************/
	static void mmc5603x_set(sns_sensor_instance *const this)
	{
	  uint8_t rw_buffer = 0x00;
	  uint32_t xfer_bytes;
	  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
	
	  /* Write 0x08 to register 0x1B, set SET bit high */
	  rw_buffer = MMC5603NJ_CMD_SET;
	  mmc5603x_read_modify_write(state->scp_service,
								 state->com_port_info.port_handle,
								 MMC5603NJ_REG_CTRL0,
								 &rw_buffer,
								 1,
								 &xfer_bytes,
								 false,
								 0xFF);
	  /* Delay at least 1ms to finish the SET operation */
	  sns_busy_wait(sns_convert_ns_to_ticks(3 * 1000 * 1000));
	  return;
	}
	/*********************************************************************************
	* decription: RESET operation 
	*********************************************************************************/
	static void mmc5603x_reset(sns_sensor_instance *const this)
	{
	  uint8_t rw_buffer = 0x00;
	  uint32_t xfer_bytes;
	  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
	
	  /* Write 0x08 to register 0x1B, set SET bit high */
	  rw_buffer = MMC5603NJ_CMD_RESET;
	  mmc5603x_read_modify_write(state->scp_service,
								 state->com_port_info.port_handle,
								 MMC5603NJ_REG_CTRL0,
								 &rw_buffer,
								 1,
								 &xfer_bytes,
								 false,
								 0xFF);
	  /* Delay at least 1ms to finish the SET operation */
	  sns_busy_wait(sns_convert_ns_to_ticks(3 * 1000 * 1000));
	  return;
	}
	
	
/*********************************************************************************
* decription: Do selftest operation periodically
*********************************************************************************/
static int mmc5603x_saturation_checking (sns_sensor_instance * const this)
{
  mmc5603x_instance_state * state = (mmc5603x_instance_state *)this->state->state;
  static sns_time start_ticks = 0;
  static sns_time stop_ticks = 0;
  static uint64_t selftest_period = 500;   //500ms
  if((sns_get_system_time() - stop_ticks) >=
    sns_convert_ns_to_ticks (selftest_period * 1000 * 1000)) {
        if(state->satu_checking_flag == 0) {
            state->satu_checking_flag = mmc5603x_saturation_check_start(this);
            start_ticks = sns_get_system_time();
        } else if(state->satu_checking_flag == 1 && (sns_get_system_time() - start_ticks) >=
            sns_convert_ns_to_ticks (SELFTEST_DURATION_TIME * 1000 * 1000)) {
            start_ticks = 0;
            /* Start count again */
            stop_ticks = sns_get_system_time();
            /* Stop the saturation checking, return 2 or -1 */
            state->satu_checking_flag = mmc5603x_saturation_check_stop(this);
            if(state->satu_checking_flag == -1) {
            /* Sensor is saturated, need to do SET operation */
            mmc5603x_set (this);
            selftest_period = 100;
            //MMC56x3X_INST_PRINT (LOW, this, "MEMSIC MMC56x3xx is saturated");
            } else {
                selftest_period = 500;
            }
        }
    }
  return state->satu_checking_flag;
}

/**
 * mmc5603_read_mode_switch.
 *
 * @param  raw_data_ut    Triaxial raw data the unit is ut
 *
 * @return    none 
 */

static void mmc5603x_read_mode_switch(float raw_data_ut[], sns_sensor_instance *this)
{
    uint8_t rw_buffer = 0x00;
    uint32_t xfer_bytes;
    mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
    static int cmm_mode_disable_flag = 0;
    static sns_time last_asr_ticks = 0;
    if (MMC5603_Read_mode == CONTINUOUS_MODE_AUTO_SR) {
        /* If X or Y axis output exceed 10 Gauss, then switch to single mode */
        if ((fabs(raw_data_ut[0]) > 1000.0f) || (fabs(raw_data_ut[1]) > 1000.0f)) {
            MMC5603_Read_mode = SINGLE_MODE;
            /* Disable continuous mode */
            rw_buffer = 0x00;
            mmc5603x_read_modify_write( state->scp_service,
                                        state->com_port_info.port_handle,
                                        MMC5603NJ_REG_CTRL2,
                                        &rw_buffer,
                                        1,
                                        &xfer_bytes,
                                        false,
                                        0xFF);
            last_asr_ticks = sns_get_system_time();
            cmm_mode_disable_flag = 1;
        }
    } else if (MMC5603_Read_mode== SINGLE_MODE) {
        if(cmm_mode_disable_flag == 1) {
            if((sns_get_system_time() - last_asr_ticks) >= sns_convert_ns_to_ticks (5 * 1000 * 1000)) {
                /* If BW=01, then needs 5ms to finish the last autoSR operation before SET operation */
                cmm_mode_disable_flag = 0;
                /* Do SET operation */
                mmc5603x_set(this);
                /* Do TM_M before next data reading */
                rw_buffer = MMC5603NJ_CMD_TMM;
                mmc5603x_read_modify_write (state->scp_service,
                                            state->com_port_info.port_handle,
                                            MMC5603NJ_REG_CTRL0,
                                            &rw_buffer,
                                            1,
                                            &xfer_bytes,
                                            false,
                                            0xFF);
        //sns_busy_wait (sns_convert_ns_to_ticks (8 * 1000 * 1000)); //Delay 8ms to finish the TM_M operation
                }
        }
        /* If both of X and Y axis output less than 8 Gauss, then switch to continuous mode with Auto_SR */
        else if ((fabs (raw_data_ut[0]) < 800.0f) && (fabs (raw_data_ut[1]) < 800.0f)) {
            MMC5603_Read_mode = CONTINUOUS_MODE_AUTO_SR;
            /* Enable continuous mode with Auto_SR */
            rw_buffer = MMC5603NJ_CMD_CMM_FREQ_EN | MMC5603NJ_CMD_AUTO_SR_EN;
            mmc5603x_read_modify_write (state->scp_service,
                                        state->com_port_info.port_handle,
                                        MMC5603NJ_REG_CTRL0,
                                        &rw_buffer,
                                        1,
                                        &xfer_bytes,
                                        false,
                                        0xFF);
            rw_buffer = MMC5603NJ_CMD_CMM_EN;
            mmc5603x_read_modify_write (state->scp_service,
                                        state->com_port_info.port_handle,
                                        MMC5603NJ_REG_CTRL2,
                                        &rw_buffer,
                                        1,
                                        &xfer_bytes,
                                        false,
                                        0xFF);
        } else {
      /* Sensor checking */
            mmc5603x_saturation_checking (this);
            if(state->satu_checking_flag != 1) {/* Do TM_M before next data reading */
                rw_buffer = MMC5603NJ_CMD_TMM;
                mmc5603x_read_modify_write (state->scp_service,
                state->com_port_info.port_handle,
                MMC5603NJ_REG_CTRL0,
                &rw_buffer,
                1,
                &xfer_bytes,
                false,
                0xFF);
            }
        }
    }
    if(state->satu_checking_flag != 1)
    {
        state->satu_checking_flag = 0;
    }
}

int mmc5603x_set_reset_test(sns_sensor_instance *const this)
{
	int i;
	uint8_t  magnetic_write_buffer;
	uint8_t data_reg[6] ={0};
	uint16_t data_set[3] = {0};
	uint16_t data_reset[3] = {0};
	uint32_t delta_data[3] = {0};
	int32_t sum_data[3] = {0};
	uint16_t thr_srst_low = 50;			//50lsb = 50mG
	uint16_t thr_srst_high = 30720;		//30720lsb = 30Gauss 
	uint16_t thr_srst_off = 5120;		//5120lsb = 5Gauss 
	uint32_t xfer_bytes = 0;
	mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
	/* Write reg 0x1D */
	/* Set Cmm_en bit '0', Disable continuous mode */	
	magnetic_write_buffer = 0x00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL2, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	sns_busy_wait(sns_convert_ns_to_ticks(20*1000*1000));
	/* Write reg 0x1B */
	/* Set Auto_SR_en bit '0', Disable the function of automatic set/reset */
	magnetic_write_buffer = 0x00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	
	/* Write reg 0x1C, Set BW<1:0> = 00 */
	magnetic_write_buffer = 0x00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL1, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
				
	/* Do RESET operation */
	mmc5603x_reset(this);
	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' */	
	magnetic_write_buffer = MMC5603NJ_CMD_TMM;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	/* Delay 10 ms to finish the TM operation */
	for(i = 0; i < 5; i++) {
		sns_busy_wait(sns_convert_ns_to_ticks(2*1000*1000));
	}
	/* Read register data */
	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA, 
                               data_reg,
                               6,
                               &xfer_bytes);
	/* Get high 16bits data */
	data_reset[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	//X axis
	data_reset[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);	//Y axis		
	data_reset[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]);	//Z axis 
	 MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_set_reset_test data_reset %d %d %d ", data_reset[0],  data_reset[1],data_reset[2]);
	
	/* Do SET operation */
	mmc5603x_set(this);	
	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' */	
	magnetic_write_buffer = MMC5603NJ_CMD_TMM;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	/* Delay 10 ms to finish the TM operation */
	for(i = 0; i < 5; i++) {
		sns_busy_wait(sns_convert_ns_to_ticks(2*1000*1000));
	}
	/* Read register data */
	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA, 
                               data_reg,
                               6,
                               &xfer_bytes);
	/* Get high 16bits data */
	data_set[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	//X axis
	data_set[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);	//Y axis		
	data_set[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]);	//Z axis 	
	
	 MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_set_reset_test data_set %d %d %d ", data_set[0],  data_set[1],data_set[2]);
	for(i=0;i<3;i++)
	{
		if(data_set[i] >= data_reset[i])
			delta_data[i] = data_set[i] - data_reset[i];
		else
			delta_data[i] = data_reset[i] - data_set[i];
		
		delta_data[i] = delta_data[i]>>1;
		
		sum_data[i] = ((int32_t)data_set[i] + (int32_t)data_reset[i] - 65536)>>1;
		if(sum_data[i] < 0)
			sum_data[i] = -sum_data[i];		
	}
	
	 MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_set_reset_test delta_data %d %d %d ", delta_data[0],  delta_data[1],delta_data[2]);
	/* If output < 50mG, fail*/
	if(delta_data[0]<thr_srst_low && delta_data[1]<thr_srst_low && delta_data[2]<thr_srst_low)
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_set_reset_test  fail delta_data  <thr_srst_low ");
		return -1;	// fail
	}
	
	/* If output > 30Gauss, fail*/
	if(delta_data[0]>thr_srst_high || delta_data[1]>thr_srst_high || delta_data[2]>thr_srst_high)
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_set_reset_test  fail delta_data >thr_srst_high ");
		return -1;	// fail	
	}
	
	if(sum_data[0]>thr_srst_off || sum_data[1]>thr_srst_off || sum_data[2]>thr_srst_off)
	{
         MMC5603X_INST_PRINT(MED, this, "mmc5603x_set_reset_test  fail sum_data >thr_srst_off ");
		return -1;	// fail
	}
	
	return 1;	//pass
}

int mmc5603x_continuous_mode_autoSR_test(sns_sensor_instance *const this)
{
	uint8_t  magnetic_write_buffer;

	uint8_t i;
	uint8_t data_reg[6] ={0};
	uint16_t data_ref[3] = {0};	
	
	int16_t data_lsb_mgauss[3] = {0};
	uint16_t thr_lsb_mgauss = 50;
	uint32_t xfer_bytes = 0;

	mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
	/* Write reg 0x1C, Set BW<1:0> = bandwith */
	magnetic_write_buffer = MMC5603NJ_CMD_BW00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL1, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	
	/* Write reg 0x1A, set ODR<7:0> = sampling_rate */ 
	magnetic_write_buffer = MMC5603NJ_CMD_ODR_50HZ;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_ODR, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	
	/* Write reg 0x1B */
	/* Set Auto_SR_en bit '1', Enable the function of automatic set/reset */
	/* Set Cmm_freq_en bit '1', Start the calculation of the measurement period 
according to the ODR*/	
	magnetic_write_buffer = MMC5603NJ_CMD_CMM_FREQ_EN|MMC5603NJ_CMD_AUTO_SR_EN;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);

	/* Write reg 0x1D */
	/* Set Cmm_en bit '1', Enter continuous mode */		
	
	magnetic_write_buffer = MMC5603NJ_CMD_CMM_EN;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL2, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);

	sns_busy_wait(sns_convert_ns_to_ticks(20*1000*1000)); 

	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA, 
                               data_reg,
                               6,
                               &xfer_bytes);
	data_ref[0] = (uint16_t)(data_reg[0] << 8 | data_reg[1]);	
	data_ref[1] = (uint16_t)(data_reg[2] << 8 | data_reg[3]);			
	data_ref[2] = (uint16_t)(data_reg[4] << 8 | data_reg[5]); 
	
   MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_continuous_mode_autoSR_test  data_ref  %d %d %d ", data_ref[0],  data_ref[1],data_ref[2]);
	/* Write reg 0x1D */
	/* Set Cmm_en bit '0', Disable continuous mode */	
	magnetic_write_buffer = 0x00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL2, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	sns_busy_wait(sns_convert_ns_to_ticks(20*1000*1000)); 

		
	for(i=0;i<3;i++)
	{
		if((data_ref[i]==0) || (data_ref[i] == 0xFFFF))
		{
			data_lsb_mgauss[i] = 0;
			return -1;	// fail
		}
		else
		{
			data_lsb_mgauss[i] = (int16_t)(data_ref[i]-32768);
			if(data_lsb_mgauss[i]<0)
				data_lsb_mgauss[i] = -data_lsb_mgauss[i];
		}
	}
	
	if(data_lsb_mgauss[0]<thr_lsb_mgauss && data_lsb_mgauss[1]<thr_lsb_mgauss &&data_lsb_mgauss[2]<thr_lsb_mgauss)
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_continuous_mode_autoSR_test  fail data_lsb_mgauss <thr_lsb_mgauss ");
		return -1;	// fail
	}

	return 1;	// pass	
}

int mmc5603x_manual_selftest(sns_sensor_instance *const this)
{
	int i;
	uint8_t  magnetic_write_buffer;
	uint8_t reg_value[3]={0};
	int16_t st_thr_data[3]={0};
	int16_t st_thr_low[3]={0};
	int16_t st_thr_high[3]={0};
	
	uint8_t reg_buf[6]={0};
	uint16_t data_temp[3]={0};
	
	int16_t norm_data[3]={0};
	int16_t st_data[3]={0};
	int16_t delta_data[3]={0};
	uint32_t xfer_bytes = 0;

	mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
	/* Do SET operation before seld-test */
	mmc5603x_set(this);
	
	/* Read trim data from reg 0x27-0x29 */	
	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_ST_X_VAL, 
                               reg_value,
                               3,
                               &xfer_bytes);
	for(i=0;i<3;i++)
	{
		st_thr_data[i] = (int16_t)(reg_value[i]-128)*32; 
		if(st_thr_data[i]<0)
			st_thr_data[i] = -st_thr_data[i];
		st_thr_low[i] = st_thr_data[i]-st_thr_data[i]/2;	//80%
		st_thr_high[i] = st_thr_data[i]+5*st_thr_data[i];	//500%
	}
		
	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' */ 
	magnetic_write_buffer = MMC5603NJ_CMD_TMM;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	sns_busy_wait(sns_convert_ns_to_ticks(10*1000*1000)); 

	/* Read register 0x00-0x05 */
	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA, 
                               reg_buf,
                               6,
                               &xfer_bytes);
	data_temp[0] = reg_buf[0]<<8 | reg_buf[1];
	data_temp[1] = reg_buf[2]<<8 | reg_buf[3];
	data_temp[2] = reg_buf[4]<<8 | reg_buf[5];	
	for(i=0;i<3;i++)
	{
		norm_data[i] =(int16_t)(((float)data_temp[i]-32768)/1024*1000);
	}

	 MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_manual_selftest  norm_data %d %d %d ", norm_data[0],  norm_data[1],norm_data[2]);
	/* Write 0x20 to register 0x1C, set St_enp bit '1' */

	magnetic_write_buffer = MMC5603NJ_CMD_ST_ENP;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL1, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	sns_busy_wait(sns_convert_ns_to_ticks(2*1000*1000));

	/* Write 0x01 to register 0x1B, set Take_meas_M bit '1' to bring a 
	 * DC current through the self-test coil of the sensor */	 
	magnetic_write_buffer = MMC5603NJ_CMD_TMM;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL0, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	sns_busy_wait(sns_convert_ns_to_ticks(10*1000*1000));

	/* Write 0x00 to register 0x1C, set St_enp bit '0' */
	magnetic_write_buffer = 0x00;
	mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle, 
                                MMC5603NJ_REG_CTRL1, 
								&magnetic_write_buffer,
								1,
								&xfer_bytes,
                                false,
                                0xFF);
	/* Read register 0x00-0x05 */
	mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA, 
                               reg_buf,
                               6,
                               &xfer_bytes);
	data_temp[0] = reg_buf[0]<<8 | reg_buf[1];
	data_temp[1] = reg_buf[2]<<8 | reg_buf[3];
	data_temp[2] = reg_buf[4]<<8 | reg_buf[5];		
	
	for(i=0;i<3;i++)
	{
		st_data[i] =(int16_t)(((float)data_temp[i]-32768)/1024*1000);		
		delta_data[i]= st_data[i]-norm_data[i];
		if(delta_data[i]<0)
			delta_data[i] = -delta_data[i];
	}
	MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_manual_selftest  st_data  %d %d %d  norm_data %d %d %d delta_data %d %d %d",
	norm_data[0],  norm_data[1],norm_data[2],st_data[0],st_data[1],st_data[2],delta_data[0],delta_data[1],delta_data[2]);

	MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_manual_selftest  st_thr_low %d %d %d ", st_thr_low[0],  st_thr_low[1],st_thr_low[2]);
	MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_manual_selftest  st_thr_high %d %d %d ", st_thr_high[0],  st_thr_high[1],st_thr_high[2]);
	
	/* Check if the sensor is OK or not */
	if((delta_data[0]<st_thr_low[0])||(delta_data[1]<st_thr_low[1])||(delta_data[2]<st_thr_low[2]))
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_manual_selftest  delta_data < st_thr_low fail");
		return -1;//self test fail
	}	
	else if((delta_data[0]>st_thr_high[0])||(delta_data[1]>st_thr_high[1])||(delta_data[2]>st_thr_high[2]))
	{
			MMC5603X_INST_PRINT(MED, this, "mmc5603x_manual_selftest  delta_data >delta_data fail");
		return -1;//self test fail
	}
	else
	{
		return 1;//self test pass
	}
}


bool mmc5603x_hw_self_test(sns_sensor_instance *const this)
{
	/*if(mmc5603x_software_reset(this) == -1)
		return false;	//fail
	*/
    int ret = 0;
    int count = 0;
    do {
        count++;
        ret = mmc5603x_set_reset_test(this);
    } while(ret == -1 && count < 3);
    if (count == 3 && ret == -1) {
        MMC5603X_INST_PRINT(MED, this, "mmc5603x_set_reset_test fail");
        return false;
    }

	if(mmc5603x_continuous_mode_autoSR_test(this) == -1)
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_continuous_mode_autoSR_test fail");
		//return false;	//fail
	}

	if(mmc5603x_manual_selftest(this) == -1)
	{
		MMC5603X_INST_PRINT(MED, this, "mmc5603x_manual_selftest fail");
		//return false;	//fail  avoid misjudje
	}
	
	return true;	//pass
}


/**
 * Gets current ODR.
 *
 * @param[i] curr_odr              Current ODR.
 *
 * @return current ODR
 */
float mmc5603x_get_mag_odr(mmc5603x_mag_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for (idx = 0; idx < ARR_SIZE(reg_map_mmc5603x); idx++)
  {
    if (curr_odr == reg_map_mmc5603x[idx].mag_odr_reg_value
        &&
        curr_odr != MMC5603X_MAG_ODR_OFF)
    {
      odr = reg_map_mmc5603x[idx].odr;
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
sns_time mmc5603x_get_sample_interval(mmc5603x_mag_odr curr_odr)
{
  int8_t   idx;
  sns_time sample_interval = 0;

  for (idx = 0; idx < ARR_SIZE(reg_map_mmc5603x); idx++)
  {
    if (curr_odr == reg_map_mmc5603x[idx].mag_odr_reg_value
        &&
        curr_odr != MMC5603X_MAG_ODR_OFF)
    {
      sample_interval = sns_convert_ns_to_ticks(1000000000 / reg_map_mmc5603x[idx].odr);
      break;
    }
  }

  return sample_interval;
}


/**
 * Extract a mag sample from a segment of the mag buffer and generate an
 * event.
 *
 * @param[i] fifo_sample       The segment of fifo buffer that has the mag sample
 * @param[i] timestamp         Timestamp to be used for this sample
 * @param[i] instance          The current mmc5603x sensor instance
 * @param[i] event_service     Event service for sending out the mag sample
 * @param[i] state             The state of the mmc5603x sensor instance
 */
static void mmc5603x_handle_mag_sample(uint8_t data[6],
                                      sns_time timestamp,
                                      sns_sensor_instance *const instance,
                                      sns_event_service *event_service,
                                      mmc5603x_instance_state *state,
                                      log_sensor_state_raw_info *log_mag_state_raw_info
                                      )
{
    int32_t raw_data_count[3] = {0};
    float raw_data_ut[3]  = {0}, convert_data_ut[3]={0};
    uint8_t i = 0;
	event_service = NULL;
	//int zero_check = 0;

    sns_std_sensor_sample_status status =  SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    sns_diag_service* diag = state->diag_service;

    // Allocate Sensor State Raw log packets for mag 
    sns_memzero(log_mag_state_raw_info, sizeof(log_sensor_state_raw_info));
    log_mag_state_raw_info->encoded_sample_size = state->log_raw_encoded_size;
    log_mag_state_raw_info->diag = diag;
    log_mag_state_raw_info->instance = instance;
    log_mag_state_raw_info->sensor_uid = &state->mag_info.suid;
    mmc5603x_log_sensor_state_raw_alloc(log_mag_state_raw_info, 0);


    /* lsb=magnetic_read_data[1], msb=magnetic_read_data[0] */
    raw_data_count[0] = (((int32_t)data[0] << 8) | (int32_t)data[1]);  
    raw_data_count[1] = (((int32_t)data[2] << 8) | (int32_t)data[3]);
    raw_data_count[2] = (((int32_t)data[4] << 8) | (int32_t)data[5]); 
    MMC5603X_INST_PRINT(ERROR, instance, "MEMSIC MMC5603x decode raw_data_count %d %d %d ", raw_data_count[0],  
    	                                                               raw_data_count[1], raw_data_count[2] );
    raw_data_ut[0] = ((float)raw_data_count[0] - 32768.0)*100.0 / 1024.0;
    raw_data_ut[1] = ((float)raw_data_count[1] - 32768.0)*100.0 / 1024.0;
    raw_data_ut[2] = ((float)raw_data_count[2] - 32768.0)*100.0 / 1024.0;
    MMC5603X_INST_PRINT(ERROR, instance, "MEMSIC MMC5603xx raw_data_ut*1000 %d  %d %d", (int32_t)(raw_data_ut[0]*1000),
    	                                                               (int32_t)(raw_data_ut[1]*1000),(int32_t)(raw_data_ut[2]*1000)) ;
#ifdef MMC5603X_MODE_SWITCH
    mmc5603x_read_mode_switch(raw_data_ut,instance);
#endif
#if  0// remove the auto_selftest because the mag  saturaion is use mmc5603x_saturation_checking
    if ((fabsf(raw_data_ut[0]) < 5.0) && (fabsf(raw_data_ut[1]) < 5.0) && (fabsf(raw_data_ut[2]) < 5.0))
    {
        mmc5603x_disable_measure(instance);
        zero_check = mmc5603x_auto_selftest(instance);
		mmc5603x_enable_measure(instance);
        if (zero_check == -1)
        {
            raw_data_ut[0] = 3200.0;
			raw_data_ut[1] = 3200.0;
			raw_data_ut[2] = 3200.0;
        }
    }
#endif

    for(i = 0; i < TRIAXIS_NUM; i++)
    {
        convert_data_ut[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) *raw_data_ut[state->axis_map[i].ipaxis];
    }
    MMC5603X_INST_PRINT(ERROR, instance, "MEMSIC MMC5603xxx convert_data_ut*1000 %d   %d  %d", (int32_t)(convert_data_ut[0]*1000),
     	                                                               (int32_t)(convert_data_ut[1]*1000),(int32_t)(convert_data_ut[2]*1000)) ;

	// factory calibration
    vector3 data_cal_ut = sns_apply_calibration_correction_3(
        make_vector3_from_array(convert_data_ut),
        make_vector3_from_array(state->mag_registry_cfg.fac_cal_bias),
        state->mag_registry_cfg.fac_cal_corr_mat);

    MMC5603X_INST_PRINT(ERROR, instance, "MEMSIC MMC5603Nxxxxx data_cal_ut*1000 %d   %d  %d", (int32_t)(data_cal_ut.data[0]*1000),
                    (int32_t)(data_cal_ut.data[1]*1000),(int32_t)(data_cal_ut.data[2]*1000)) ;

    //if(state->mag_info.num_samples_to_discard != 0) 
	//if (((raw_data_count[0] == 0) && (raw_data_count[1] == 0) && (raw_data_count[2] == 0))
	//	|| (state->mag_info.num_samples_to_discard != 0))
	if ((raw_data_count[0] == 0) && (raw_data_count[1] == 0) && (raw_data_count[2] == 0))
	{
	    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE; 
		MMC5603X_INST_PRINT(ERROR, instance, "MEMSIC MMC5603N discard the first data") ;
		state->mag_info.num_samples_to_discard--; 
	}

  pb_send_sensor_stream_event(instance,
                              &state->mag_info.suid,
                              timestamp,
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              status,
                              data_cal_ut.data,
                              ARR_SIZE(data_cal_ut.data),
                              state->encoded_mag_event_len);

  // Log raw uncalibrated sensor data
  mmc5603x_log_sensor_state_raw_add(
    log_mag_state_raw_info,
    convert_data_ut,         //memsic add 
    timestamp,
    status);
  state->total_samples++;
}

void mmc5603x_process_mag_data_buffer(sns_sensor_instance *instance,
                                     sns_time            first_timestamp,
                                     sns_time            sample_interval_ticks,
                                     uint8_t             *fifo_start,
                                     size_t              num_bytes
)
{

  uint8_t  raw_data[6];
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;
  sns_service_manager *service_manager = instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
  MMC5603X_INST_PRINT(ERROR, instance, "mmc5603x_process_mag_data_buffer");


#ifdef MMC5603X_ENABLE_DIAG_LOGGING
  sns_diag_service          *diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;
  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  log_mag_state_raw_info.diag = diag;
  log_mag_state_raw_info.instance = instance;
  log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
  mmc5603x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);
#endif

  size_t num_bytes_to_report;
  num_bytes_to_report = num_bytes;
  uint32_t xfer_bytes;

  #ifndef MMC5603X_ENABLE_DAE 
    mmc5603x_com_read_wrapper(state->scp_service,
                               state->com_port_info.port_handle,
                               MMC5603NJ_REG_DATA,
                               &raw_data[0],
                               6,
                               &xfer_bytes);


	mmc5603x_handle_mag_sample(raw_data,
		                       first_timestamp,
                               instance,
                               event_service,
                               state,
                               &log_mag_state_raw_info);
	fifo_start = NULL;  //gumu add
	#else
	mmc5603x_handle_mag_sample(fifo_start,
		                       first_timestamp,
                               instance,
                               event_service,
                               state,
                               &log_mag_state_raw_info);
	UNUSED_VAR(raw_data);
	UNUSED_VAR(xfer_bytes);
	#endif
	MMC5603X_INST_PRINT(ERROR, instance, "TS %u pre %u ",
			  (uint32_t)first_timestamp,
			  (uint32_t)sample_interval_ticks
		       );

  // store previous measurement is irq and also right WM
  if(state->mag_info.use_dri && state->irq_info.detect_irq_event)
  {
    state->previous_meas_is_irq = true;
  }
  else
  {
    state->previous_meas_is_irq = false;
  }

  if(state->mag_info.cur_wmk + 1 == state->num_samples)
  {
    state->previous_meas_is_correct_wm = true;
  }
  else
  {
    state->previous_meas_is_correct_wm = false;
  }

  // store previous timestamp
 // state->pre_timestamp = timestamp;
  state->pre_timestamp = first_timestamp;

  // reset flags
  state->irq_info.detect_irq_event = false;
  state->this_is_first_data = false;

  state->this_is_the_last_flush = false;
  if(!mmc5603x_dae_if_available(instance) && state->fifo_flush_in_progress)
  {
   MMC5603X_INST_PRINT(ERROR, instance, "TS %u pre %u ");
    mmc5603x_send_fifo_flush_done(instance);
  }

#ifdef MMC5603X_ENABLE_DIAG_LOGGING
    mmc5603x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
#endif
}

/** See mmc5603x_hal.h */
void mmc5603x_send_fifo_flush_done(sns_sensor_instance *const instance)
{
  sns_service_manager *mgr = instance->cb->get_service_manager(instance);
  sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr,SNS_EVENT_SERVICE);
  sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;
  state->system_time = sns_get_system_time();

  if(NULL != event)
  {
    event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
    event->event_len = 0;
    event->timestamp = state->system_time;

    e_service->api->publish_event(e_service, instance, event, &state->mag_info.suid);

    MMC5603X_INST_PRINT(ERROR, instance, "FLUSH_EVENT sent");
  }

  state->fifo_flush_in_progress = false;
}


void mmc5603x_validate_timestamp_for_dri(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

void mmc5603x_validate_timestamp_for_polling(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;

  if(state->fifo_flush_in_progress || state->this_is_the_last_flush)
  {
    // from flush request
    state->interrupt_timestamp = state->pre_timestamp + state->averaged_interval * state->num_samples;
  }
  else
  {
    // from timer event
    state->interrupt_timestamp = state->system_time;
  }

  state->first_data_ts_of_batch = state->interrupt_timestamp - state->averaged_interval * (state->num_samples - 1);
}


static void mmc5603x_read_fifo_buffer(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;
  uint8_t buffer[MMC5603X_MAX_PHYSICAL_FIFO_SIZE*MMC5603X_NUM_DATA_HXL_TO_ST2];// Sometimes, mag is taking scp path with number of samples 32
  MMC5603X_INST_PRINT(ERROR, instance, "mmc5603x_read_fifo_buffer  state->ascp_xfer_in_progress = %d", state->ascp_xfer_in_progress);

  state->num_samples = 1;
  state->mag_info.data_count+=state->num_samples;
  
  mmc5603x_validate_timestamp_for_polling(instance);
  mmc5603x_process_mag_data_buffer(instance,
                                      state->first_data_ts_of_batch,
                                      state->averaged_interval,
                                      buffer,
                                      MMC5603X_NUM_DATA_HXL_TO_ST2 * state->num_samples);


  state->heart_beat_attempt_count = 0;
}

void mmc5603x_read_mag_samples(sns_sensor_instance *const instance)
{
  MMC5603X_INST_PRINT(ERROR, instance, "mmc5603x_read_mag_samples");
  mmc5603x_read_fifo_buffer(instance);
}

void mmc5603x_send_cal_event(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;
  sns_cal_event cal_event = sns_cal_event_init_default;
  pb_buffer_arg buff_arg_bias = {
    .buf     = &state->mag_registry_cfg.fac_cal_bias,
    .buf_len = ARR_SIZE(state->mag_registry_cfg.fac_cal_bias) };
  pb_buffer_arg buff_arg_comp_matrix = {
    .buf     = &state->mag_registry_cfg.fac_cal_corr_mat.data,
    .buf_len = ARR_SIZE(state->mag_registry_cfg.fac_cal_corr_mat.data) };

  cal_event.bias.funcs.encode        = &pb_encode_float_arr_cb;
  cal_event.bias.arg                 = &buff_arg_bias;
  cal_event.comp_matrix.funcs.encode = &pb_encode_float_arr_cb;
  cal_event.comp_matrix.arg          = &buff_arg_comp_matrix;
  cal_event.status                   = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_LOW;

  MMC5603X_INST_PRINT(ERROR, instance, "tx CAL_EVENT");


  pb_send_event(instance,
                sns_cal_event_fields,
                &cal_event,
                sns_get_system_time(),
                SNS_CAL_MSGID_SNS_CAL_EVENT,
                &state->mag_info.suid);
}

void mmc5603x_reset_cal_data(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)instance->state->state;
  float bias_data[] = {0,0,0};
  float comp_matrix_data[] = {1,0,0,0,1,0,0,0,1};

  for (int i = 0; i < ARR_SIZE(bias_data); i++)
  {
    state->mag_registry_cfg.fac_cal_bias[i] = bias_data[i];
  }
  for (int i = 0; i < ARR_SIZE(comp_matrix_data); i++)
  {
    state->mag_registry_cfg.fac_cal_corr_mat.data[i] = comp_matrix_data[i];
  }
  state->mag_registry_cfg.version++;
}

/** See sns_mmc5603x_hal.h */
sns_rc mmc5603x_send_config_event(sns_sensor_instance *const instance)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
    sns_std_sensor_physical_config_event_init_default;
  char *operating_mode;
  pb_buffer_arg op_mode_args;

  if(MMC5603X_ODR_0 == state->mag_req.sample_rate)
  {
    return SNS_RC_SUCCESS;
  }


#if defined(MMC5603X_ENABLE_ALL_DEVICES)
    operating_mode = MMC5603X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = MMC5603X_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = MMC5603X_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = MMC5603X_MIN_RANGE;
    phy_sensor_config.range[1] = MMC5603X_MAX_RANGE;
#endif


  //Set config event for s4s
  mmc5603x_s4s_send_config_event(instance, &phy_sensor_config);

  op_mode_args.buf = operating_mode;
  op_mode_args.buf_len = sizeof(operating_mode);
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.sample_rate = state->mag_req.sample_rate;
  phy_sensor_config.has_DAE_watermark  = mmc5603x_dae_if_available(instance);
  phy_sensor_config.DAE_watermark      = SNS_MAX(state->mag_info.req_wmk, 1);

  state->last_sent_cfg.odr             = state->new_cfg.odr;
  state->last_sent_cfg.fifo_wmk        = state->new_cfg.fifo_wmk;
  MMC5603X_INST_PRINT(ERROR, instance,
                     "tx PHYSICAL_CONFIG_EVENT: rate %u/100 wm %u sync %u",
                     (uint32_t)(state->mag_req.sample_rate * 100),
                     phy_sensor_config.has_water_mark ? phy_sensor_config.water_mark : 0,
                     phy_sensor_config.stream_is_synchronous);

  state->system_time = sns_get_system_time();
  pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                state->system_time,
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &state->mag_info.suid);

  mmc5603x_send_cal_event(instance);

  return SNS_RC_SUCCESS;
}

void mmc5603x_register_interrupt(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}

static sns_rc mmc5603x_send_timer_request(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  sns_request             timer_req;
  size_t                  req_len = 0;
  uint8_t                 buffer[20];
  sns_rc rv = SNS_RC_SUCCESS;

  if (state->mag_req.sample_rate != MMC5603X_ODR_0)
  {
    if (NULL == state->timer_data_stream)
    {
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                     this,
                                                     state->timer_suid,
                                                     &state->timer_data_stream
                                                     );
    }

    if (NULL != state->timer_data_stream)
    {
      req_len = pb_encode_request(buffer,
                                  sizeof(buffer),
                                  &state->req_payload,
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
  }
  return rv;
}

void mmc5603x_set_timer_request_payload(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  sns_time                sample_period;
  state->system_time = sns_get_system_time();


  // for S4S timer
  if (state->mag_info.use_sync_stream)
  {
#ifdef MMC5603X_ENABLE_S4S
    req_payload.has_priority = true;
    req_payload.priority = SNS_TIMER_PRIORITY_POLLING;
    req_payload.is_periodic = true;
    sample_period = sns_convert_ns_to_ticks(
        1 / state->mag_req.sample_rate * 1000 * 1000 * 1000);
    req_payload.start_time = state->system_time - sample_period;
    req_payload.start_config.early_start_delta = 0;
    req_payload.start_config.late_start_delta = sample_period * 2;


    req_payload.timeout_period = sample_period;

    // Set heart_beat_timeout_period for heart beat in S4S/FIFO+S4S
    // as 5 samples time for S4S plus 1 sample time for jitter
    // or 2 FIFO buffers time for FIFO+S4S plus 2 sample time for jitter
    state->heart_beat_timeout_period = req_payload.timeout_period * 5 * sample_period;
 
#endif // MMC5603X_ENABLE_S4S
  }
  // for polling timer
  else
  {
    req_payload.has_priority = true;
    req_payload.priority = SNS_TIMER_PRIORITY_POLLING;
    req_payload.is_periodic = true;
    req_payload.start_time = state->system_time;
    sample_period = sns_convert_ns_to_ticks(
        1 / state->mag_req.sample_rate * 1000 * 1000 * 1000);
    //start time calculation should be similar to above use_sync_stream case
    //If this sensor is doing polling, it would be good to synchronize the Mag polling timer,
    //with other polling timers on the system.
    //For example, that the pressure sensor is polling at 20Hz.
    //It would be good to make sure the mag polling timer and the pressure polling timer are synchronized if possible.
    req_payload.start_config.early_start_delta = 0;
    req_payload.start_config.late_start_delta = sample_period * 2;


    req_payload.timeout_period = sample_period;


    // Set heart_beat_timeout_period for heart beat in Polling/FIFO+Polling
    // as 5 samples time for Polling plus 1 sample time for jitter
    // or 2 FIFO buffers time for FIFO+Polling plus 2 sample time for jitter
    state->heart_beat_timeout_period = req_payload.timeout_period * 5 * sample_period;
  }
  // reset request payload
  state->req_payload = req_payload;
}

void mmc5603x_register_heart_beat_timer(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  state->req_payload.start_time = state->system_time;
  state->hb_timer_fire_time = state->req_payload.start_time + state->req_payload.timeout_period;

  if (SNS_RC_SUCCESS != mmc5603x_send_timer_request(this))
  {
    MMC5603X_INST_PRINT(ERROR, this, "Failed send timer request");
  }

  MMC5603X_INST_PRINT(ERROR, this, "hb_timer start %u fire %u period %u",
      (uint32_t)state->req_payload.start_time, (uint32_t)state->hb_timer_fire_time, (uint32_t)state->req_payload.timeout_period);
   
}

void mmc5603x_register_timer(sns_sensor_instance *const this)
{
  mmc5603x_set_timer_request_payload(this);
  mmc5603x_send_timer_request(this);
}

sns_rc mmc5603x_get_meas_time( memsic_device_type device_select,
                              uint8_t sdr,
                              sns_time* meas_ts )
{
  *meas_ts = MMC5603X_TIME_FOR_MEASURE_US;
  UNUSED_VAR(device_select);
  UNUSED_VAR(sdr);
  return SNS_RC_SUCCESS;
}


void mmc5603x_disable_measure(sns_sensor_instance *const this)
{
    mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
    MMC5603X_INST_PRINT(ERROR, this,"MEMSIC MMC5603NJ mmc5603x_disable_measure");


    uint8_t rw_buffer = 0x00;
    uint32_t xfer_bytes;

    rw_buffer = 0x00;	
    mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle,
                                MMC5603NJ_REG_CTRL2,
                                &rw_buffer,
                                1,
                                &xfer_bytes,
                                false,
                                0xFF);

    state->is_enabled = MMC5603NJ_DISABLE_MEAS;
    //sns_busy_wait(sns_convert_ns_to_ticks(MMC5603NJ_STABLE_DELAY*1000*1000));
}

void mmc5603x_enable_measure(sns_sensor_instance *const this)
{
    mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;

    uint8_t rw_buffer = 0x00;
    uint32_t xfer_bytes;
    MMC5603_Read_mode=CONTINUOUS_MODE_AUTO_SR;
	state->satu_checking_flag = 0;


   // if(state->is_enabled == MMC5603NJ_DISABLE_MEAS)
    {

        rw_buffer = MMC5603NJ_BW_300HZ;
        mmc5603x_read_modify_write(state->scp_service,
                                    state->com_port_info.port_handle,
                                    MMC5603NJ_REG_CTRL1,
                                    &rw_buffer,
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);
    }
    rw_buffer = (uint8_t)(state->mag_info.curr_odr*1.2);
	MMC5603X_INST_PRINT(ERROR, this,"mmc5603x_enable_measure odr = %d ",state->mag_info.curr_odr);

    mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle,
                                MMC5603NJ_REG_ODR,
                                &rw_buffer,
                                1,
                                &xfer_bytes,
                                false,
                                0xFF);
    rw_buffer = MMC5603NJ_S4S_CONF;
    mmc5603x_read_modify_write(state->scp_service,
                                state->com_port_info.port_handle,
                                MMC5603NJ_REG_CTRL0,
                                &rw_buffer,
                                1,
                                &xfer_bytes,
                                false,
                                0xFF);

  //  if(state->is_enabled == MMC5603NJ_DISABLE_MEAS)
    {
        rw_buffer = MMC5603NJ_AUTO_SET;
        mmc5603x_read_modify_write(state->scp_service,
                                    state->com_port_info.port_handle,
                                    MMC5603NJ_REG_CTRL2,
                                    &rw_buffer,
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);
        state->is_enabled = MMC5603NJ_ENABLE_MEAS;


    }
	//sns_busy_wait(sns_convert_ns_to_ticks(MMC5603NJ_STABLE_DELAY*1000*1000));

}


sns_rc mmc5603x_reconfig_hw(sns_sensor_instance *this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;


  MMC5603X_INST_PRINT(ERROR, this, "reconfig_hw: irq_ready=%u", state->irq_info.is_ready);

  if (state->mag_info.desired_odr != MMC5603X_MAG_ODR_OFF)
  {

	if(!state->mag_info.use_dri || state->irq_info.is_ready || 
       mmc5603x_dae_if_is_streaming(this))
    {
      MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_enable_measure!");
      //mmc5603x_enable_measure(this);
      rv = mmc5603x_start_mag_streaming(this);
	  if(rv != SNS_RC_SUCCESS)
      {
        MMC5603X_INST_PRINT(ERROR, this, "reconfig_hw: failed to start");
      }
      else if(!state->in_clock_error_procedure && !mmc5603x_dae_if_available(this))
      {
        mmc5603x_send_config_event(this);
      }
    }
	mmc5603x_enable_measure(this);
  }
  else
  {
    MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_disable_measure!");
    mmc5603x_disable_measure(this);
    rv = mmc5603x_stop_mag_streaming(this);

    if (rv != SNS_RC_SUCCESS)
    {
      //state->mag_info.cur_wmk = pre_wmk;
      return rv;
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
static void mmc5603x_send_com_test_event(sns_sensor_instance *instance,
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

/** See sns_mmc5603x_hal.h */
void mmc5603x_run_self_test(sns_sensor_instance *instance)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)instance->state->state;
  bool hw_selftest_result = false;
  
  hw_selftest_result = mmc5603x_hw_self_test(instance);
  MMC5603X_INST_PRINT(ERROR, instance, "memsic mmc5603x_run_self_test");

  if(hw_selftest_result)
  {
    MMC5603X_INST_PRINT(ERROR, instance, "self-test hw_selftest_result success");
  }
  else
  {
    MMC5603X_INST_PRINT(ERROR, instance, "self-test hw_selftest_result failed");
  }

  if(state->mag_info.test_info.test_client_present)
  {
    if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "self-test SNS_PHYSICAL_SENSOR_TEST_TYPE_COM!!");
      mmc5603x_send_com_test_event(instance, &state->mag_info.suid, hw_selftest_result,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test in progress.
      MMC5603X_INST_PRINT(ERROR, instance, "self-test SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY!!");
      mmc5603x_send_com_test_event(instance, &state->mag_info.suid, hw_selftest_result,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY);
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
    {
      MMC5603X_INST_PRINT(ERROR, instance, "self-test SNS_PHYSICAL_SENSOR_TEST_TYPE_HW!!");
      mmc5603x_send_com_test_event(instance, &state->mag_info.suid, hw_selftest_result,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_HW);
    }
    state->mag_info.test_info.test_client_present = false;
  }
}

