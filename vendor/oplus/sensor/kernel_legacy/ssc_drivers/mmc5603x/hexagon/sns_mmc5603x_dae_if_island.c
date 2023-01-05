/**
 * @file sns_mmc5603x_dae_if_island.c
 *
 * MMC5603X - DAE sensor interface
 *
 *
 * Copyright (c) 2017-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_mmc5603x_dae_if.h"

#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_types.h"

#include "sns_mmc5603x_hal.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_s4s.h"

#include "sns_dae.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

//#define MMC5603X_DAE_FORCE_NOT_AVAILABLE

#ifndef SNS_MAX
#define SNS_MAX(a,b) ({ __auto_type _a = (a);    \
                        __auto_type _b = (b);    \
                        _a > _b ? _a : _b; })
#endif /* SNS_MAX */

/*======================================================================================
  Helper Functions
  ======================================================================================*/
#ifdef MMC5603X_ENABLE_DAE
static void build_static_config_request(
  mmc5603x_state             *sensor_state,
  sns_dae_set_static_config *config_req,
  int64_t hardware_id)
{
  if(hardware_id == 0)
  {
    sns_strlcpy(config_req->func_table_name, "mmc5603x_hal_table",
               sizeof(config_req->func_table_name));
  }
  else
  {
    sns_strlcpy(config_req->func_table_name, "mmc5603x_hal_table2",
               sizeof(config_req->func_table_name));
  }


#if  1 ///gumu add
  config_req->interrupt              = 0;
  config_req->has_irq_config         = false;
  config_req->has_ibi_config         = false;
#else
  config_req->interrupt              = sensor_state->is_dri;
  config_req->has_irq_config         = sensor_state->is_dri == 1;
  config_req->has_ibi_config         = sensor_state->is_dri == 2;
  
#endif /* MMC5603X_DAE_FORCE_POLLING */
  if(sensor_state->is_dri == 1)
  {
    config_req->irq_config           = sensor_state->irq_config;
  }
  else 
  if(sensor_state->is_dri == 2)
  {
    config_req->ibi_config           =
    (sns_ibi_req){ .dynamic_slave_addr = sensor_state->com_port_info.com_config.slave_control,
                   .bus_instance = sensor_state->com_port_info.com_config.bus_instance,
                   .ibi_data_bytes = 0, };
  }

#ifdef MMC5603X_ENABLE_REGISTRY_ACCESS
    config_req->has_s4s_config       = sensor_state->registry_cfg.sync_stream;
#else
    config_req->has_s4s_config       = false;
#endif // MMC5603X_ENABLE_REGISTRY_ACCESS

  config_req->s4s_config.st_reg_addr = MEMSIC_MMC5603X_REG_SYT;
  config_req->s4s_config.st_reg_data = 0;
  config_req->s4s_config.dt_reg_addr = MEMSIC_MMC5603X_REG_DT;

  sns_com_port_config const *com_config  = &sensor_state->com_port_info.com_config;
  sns_async_com_port_config *ascp_config = &config_req->ascp_config;

  ascp_config->bus_type             = (sns_async_com_port_bus_type)com_config->bus_type;
  ascp_config->slave_control        = com_config->slave_control;
  ascp_config->reg_addr_type        = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  ascp_config->min_bus_speed_kHz    = com_config->min_bus_speed_KHz;
  ascp_config->max_bus_speed_kHz    = com_config->max_bus_speed_KHz;
  ascp_config->bus_instance         = com_config->bus_instance;

  config_req->has_accel_info         = false;
}

static sns_rc send_static_config_request(
  sns_data_stream           *stream,
  sns_dae_set_static_config *config_req)
{
  sns_rc rc = SNS_RC_FAILED;
  uint8_t encoded_msg[sns_dae_set_static_config_size];
  sns_request req = {
    .message_id  = SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG,
    .request     = encoded_msg,
    .request_len = 0
  };

  req.request_len = pb_encode_request(encoded_msg, sizeof(encoded_msg), config_req,
                                      sns_dae_set_static_config_fields, NULL);
  if(0 < req.request_len)
  {
    rc = stream->api->send_request(stream, &req);
  }
  return rc;
}

static bool stream_usable(mmc5603x_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

/* ------------------------------------------------------------------------------------ */
static bool send_mag_config(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  mmc5603x_instance_state *state      = (mmc5603x_instance_state*)this->state->state;
  mmc5603x_dae_stream     *dae_stream = &state->dae_if.mag;
  mmc5603x_mag_info       *mag_info   = &state->mag_info;
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };
  uint16_t wm;
  sns_time meas_usec;
  mmc5603x_get_meas_time(mag_info->device_select, mag_info->sdr, &meas_usec);

  MMC5603X_INST_PRINT(HIGH, this, "send_mag_config:: stream=0x%x #clk_err=%u  max_batch=%d ", dae_stream->stream, state->mag_info.clock_error_meas_count, mag_info->max_batch);

  
  config_req.dae_watermark = wm = 1;


  config_req.has_data_age_limit_ticks = true;

  if (mag_info->max_batch )
  {
     config_req.data_age_limit_ticks = UINT64_MAX;
  }
  else
  {
    config_req.data_age_limit_ticks =
      sns_convert_ns_to_ticks((uint64_t)mag_info->flush_period*1000ULL);
    config_req.data_age_limit_ticks += config_req.data_age_limit_ticks / 10;
  }

  config_req.has_polling_config  = !mag_info->use_dri;
  if( config_req.has_polling_config )
  {
#ifdef MMC5603X_ENABLE_S4S
    if (mag_info->use_sync_stream)
    {
      config_req.polling_config.polling_interval_ticks =
        sns_convert_ns_to_ticks( 1000000ULL * (uint64_t)(mag_info->cur_wmk + 1)
                                 * MMC5603X_S4S_INTERVAL_MS / (uint64_t) mag_info->s4s_t_ph );
    }
    else
    {
#endif // MMC5603X_ENABLE_S4S
      config_req.polling_config.polling_interval_ticks = 
        mmc5603x_get_sample_interval(state->mag_info.desired_odr);
#ifdef MMC5603X_ENABLE_S4S
    }
#endif // MMC5603X_ENABLE_S4S
    //TODO: it looks like the polling offset will not be adjusted for S4S.
    //So it won't be synced with any other sensors
    config_req.polling_config.polling_offset =
      (state->system_time + state->averaged_interval) / state->averaged_interval *
      state->averaged_interval;
  }
  config_req.has_accel_info      = false;

  config_req.has_expected_get_data_bytes = true;
  config_req.expected_get_data_bytes = 6;



    MMC5603X_INST_PRINT(HIGH, this, " memsic dae_watermark=%u, data_age_limit_ticks=0x%x%x, wm=%u,expected_get_data_bytes=%d dae_stream->status_bytes_per_fifo ",
                       config_req.dae_watermark, (uint32_t)(config_req.data_age_limit_ticks>>32),(uint32_t)(config_req.data_age_limit_ticks &0xFFFFFFFF),
                        wm, config_req.expected_get_data_bytes, dae_stream->status_bytes_per_fifo);

  if((req.request_len =
      pb_encode_request(encoded_msg,
                        sizeof(encoded_msg),
                        &config_req,
                        sns_dae_set_streaming_config_fields,
                        NULL)) > 0)
  {
    if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
    {
      cmd_sent = true;
      dae_stream->state = STREAM_STARTING;
    }
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_hw(mmc5603x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_HW,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->flushing_hw = true;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_samples(mmc5603x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->flushing_data = true;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool stop_streaming(mmc5603x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->state = STREAM_STOPPING;
  }
  return cmd_sent;
}

static void process_data_event(
  sns_sensor_instance *this,
  pb_istream_t        *pbstream)
{
  pb_buffer_arg decode_arg;
  sns_dae_data_event data_event = sns_dae_data_event_init_default;
  data_event.sensor_data.funcs.decode = &pb_decode_string_cb;
  data_event.sensor_data.arg = &decode_arg;

  if(pb_decode(pbstream, sns_dae_data_event_fields, &data_event))
  {
    mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
	
    state->system_time = sns_get_system_time();
    state->previous_irq_time = state->irq_event_time;
    state->irq_event_time = data_event.timestamp;
	state->nominal_intvl = mmc5603x_get_sample_interval(state->mag_info.curr_odr);

    mmc5603x_process_mag_data_buffer(this, state->irq_event_time, state->nominal_intvl, (uint8_t*)decode_arg.buf, decode_arg.buf_len);
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_response(
  sns_sensor_instance *this,
  mmc5603x_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;

  sns_dae_resp resp = sns_dae_resp_init_default;
  if(pb_decode(pbstream, sns_dae_resp_fields, &resp))
  {
    switch(resp.msg_id)
    {
    case SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG:
      MMC5603X_INST_PRINT(ERROR, this,"DAE_SET_STATIC_CONFIG");
      if(SNS_STD_ERROR_NO_ERROR == resp.err)
      {
        dae_stream->state = IDLE;
        if(mmc5603x_dae_if_start_streaming(this))
        {
          
		  MMC5603X_INST_PRINT(ERROR, this,"memsic 20190110 call mmc5603x_dae_if_start_streaming 6666 ");
		  // Start - Comment for CTS testing 20180111
		  // state->config_step = MMC5603X_CONFIG_UPDATING_HW;
		  // End - Commented for CTS testing 20180111
        }
      }
      else
      {
        /* DAE sensor does not have support for this driver */
        dae_stream->stream_usable = false;
        dae_stream->state = PRE_INIT;
        if(state->mag_info.desired_odr > MMC5603X_MAG_ODR_OFF)
        {
          mmc5603x_continue_client_config(this);
        }
      }
      break;
#ifdef MMC5603X_ENABLE_S4S
    case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
      //The DAE environment will send the ST/DT messages to the HW automatically
      //without involvement of either the normal mag sensor driver,
      //or the mag sensor driver in DAE environment.
      //The ST/DT messages will continue to be sent automatically until the PAUSE_S4S
      //message is sent by the mag driver to the DAE.
      //
      //Send ST/DT command
     // mmc5603x_s4s_handle_timer_event(this);
      break;
#endif // MMC5603X_ENABLE_S4S
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
      MMC5603X_INST_PRINT(ERROR, this,"DAE_SET_STREAMING_CONFIG");
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        if(SNS_STD_ERROR_NO_ERROR == resp.err)
        {
          dae_stream->state = STREAMING;
          mmc5603x_reconfig_hw(this);
          state->config_step = MMC5603X_CONFIG_IDLE;
        }
        else
        {
          dae_stream->state = IDLE;
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      MMC5603X_INST_PRINT(ERROR, this,"DAE_FLUSH_HW");
      dae_stream->flushing_hw = false;
      if(state->config_step != MMC5603X_CONFIG_IDLE)
      {
        mmc5603x_dae_if_start_streaming(this);
        mmc5603x_dae_if_flush_samples(this);
        state->config_step = MMC5603X_CONFIG_UPDATING_HW;
      }
      else if(state->fifo_flush_in_progress)
      {
        mmc5603x_dae_if_flush_samples(this);
      }
      else if(state->heart_beat_attempt_count >= 3)
      {
        // Perform a reset operation in an attempt to revive the sensor
        mmc5603x_reconfig_hw(this);
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING: //no this case gumu
      MMC5603X_INST_PRINT(ERROR, this,
                         "DAE_PAUSE_SAMPLING dae_stream->state:%u,mag.state=%u,config_step=%u",
                         dae_stream->state, state->dae_if.mag.state, state->config_step);
      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }

      if(dae_stream->state != STREAM_STOPPING)
      {
        if(state->config_step == MMC5603X_CONFIG_STOPPING_STREAM && 
           mmc5603x_dae_if_flush_hw(this))
        {
          state->config_step = MMC5603X_CONFIG_FLUSHING_HW;
        }
        else if(state->config_step == MMC5603X_CONFIG_UPDATING_HW)
        {
          mmc5603x_dae_if_start_streaming(this);
        }
      }
      break;
#ifdef MMC5603X_ENABLE_S4S
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
      if(state->mag_info.use_sync_stream)
      {
      state->mag_info.s4s_dt_abort = true;
      mmc5603x_s4s_handle_timer_event(this);
      state->mag_info.s4s_dt_abort = false;
      }
      break;
#endif // MMC5603X_ENABLE_S4S

    case SNS_DAE_MSGID_SNS_DAE_RESP:
    case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
      break; /* unexpected */
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_events(sns_sensor_instance *this, mmc5603x_dae_stream *dae_stream)
{
  sns_sensor_event *event;

  while(NULL != dae_stream->stream &&
        NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream)))
  {
    if (dae_stream->stream_usable)
    {
      pb_istream_t pbstream =
        pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

      if (SNS_DAE_MSGID_SNS_DAE_DATA_EVENT == event->message_id)
      {
        process_data_event(this, &pbstream);
      }
      else if(SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id)
      {
      }
      else if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
      {
        MMC5603X_INST_PRINT(ERROR, this,"SNS_DAE_RESP");
        process_response(this, dae_stream, &pbstream);
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
        dae_stream->stream_usable = false;
        MMC5603X_INST_PRINT(ERROR, this,"SNS_STD_ERROR_EVENT");
        if(dae_stream->state == INIT_PENDING && 
           state->mag_info.desired_odr > MMC5603X_MAG_ODR_OFF)
        {
#ifdef MMC5603X_ENABLE_DRI
          mmc5603x_continue_client_config(this);
#endif
        }
      }
      else
      {
        MMC5603X_INST_PRINT(ERROR, this,"Unexpected message id %u", event->message_id);
      }
    }
    event = dae_stream->stream->api->get_next_input(dae_stream->stream);
  }

  if(!dae_stream->stream_usable)
  {
#if 0
  /* TODO - restore this once framework can properly deal with events published between
     the start and end of stream remove process */

    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, dae_stream->stream);
    dae_stream->stream = NULL;
#endif
  }
}

/*======================================================================================
  Public Functions
  ======================================================================================*/

void mmc5603x_dae_if_check_support(sns_sensor *this)
{
  mmc5603x_state *state = (mmc5603x_state*)this->state->state;
  MMC5603X_PRINT(ERROR, this, "mmc5603x: mmc5603x_dae_if_check_support");
  if(NULL == state->dae_if.mag.stream)
  {
    sns_service_manager *svc_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
      svc_mgr->get_service(svc_mgr, SNS_STREAM_SERVICE);

    sns_sensor_uid dae_suid;
    if(sns_suid_lookup_get(&state->suid_lookup_data, "data_acquisition_engine", &dae_suid))
    {
      stream_svc->api->create_sensor_stream(stream_svc, this, dae_suid, &state->dae_if.mag.stream);
    }
  }

  if(NULL != state->dae_if.mag.stream)
  {
    sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
    if(state->dae_if.mag.state == PRE_INIT)
    {
      state->dae_if.mag.state = INIT_PENDING;
      build_static_config_request(state, &config_req, state->hardware_id);
    }

    if(SNS_RC_SUCCESS != send_static_config_request(state->dae_if.mag.stream, &config_req))
    {
      MMC5603X_PRINT(ERROR, this, "check_support:: static config fail");
      state->dae_if.mag.state = UNAVAILABLE;
      sns_sensor_util_remove_sensor_stream(this, &state->dae_if.mag.stream);
    }
  }
}

bool mmc5603x_dae_if_available(sns_sensor_instance *this)
{
  mmc5603x_dae_if_info *dae_if = &((mmc5603x_instance_state*)this->state->state)->dae_if;
  MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_dae_if_available");
  return stream_usable(&dae_if->mag);
}

void mmc5603x_dae_if_process_sensor_events(sns_sensor *this)
{
  mmc5603x_state *state = (mmc5603x_state*)this->state->state;
  sns_data_stream *stream = state->dae_if.mag.stream;
  sns_sensor_event *event;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return;
  }

  while(NULL != (event = stream->api->peek_input(stream)))
  {
    pb_istream_t pbstream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

    if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
    {
      sns_dae_resp resp = sns_dae_resp_init_default;
      if(pb_decode(&pbstream, sns_dae_resp_fields, &resp))
      {
        if(SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG == resp.msg_id)
        {
          if(state->dae_if.mag.state == INIT_PENDING)
          {
            state->dae_if.mag.state =
              (SNS_STD_ERROR_NO_ERROR != resp.err) ? UNAVAILABLE : IDLE;
            if(UNAVAILABLE == state->dae_if.mag.state)
            {
              SNS_PRINTF(HIGH, this, "process_sensor_events:: dae unavailable");
            }
          }
        }
      }
    }
    else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
    {
      state->dae_if.mag.state = UNAVAILABLE;
    }

    event = stream->api->get_next_input(stream);
  }

  if(UNAVAILABLE == state->dae_if.mag.state || IDLE == state->dae_if.mag.state)
  {
    sns_sensor_util_remove_sensor_stream(this, &state->dae_if.mag.stream);
  }
}

/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_is_initializing(sns_sensor_instance *this)
{
  mmc5603x_dae_if_info *dae_if = &((mmc5603x_instance_state*)this->state->state)->dae_if;
  MMC5603X_INST_PRINT(ERROR, this,"mmc5603x_dae_if_is_initializing");
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == INIT_PENDING);
}

/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_is_streaming(sns_sensor_instance *this)
{
  mmc5603x_dae_if_info *dae_if = &((mmc5603x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == STREAMING);
}

/* ------------------------------------------------------------------------------------ */
sns_rc mmc5603x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  mmc5603x_state           *sensor_state)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;

  MMC5603X_INST_PRINT(ERROR, this, "dae_if_init");
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  mmc5603x_dae_if_info* dae_if = &state->dae_if;

  dae_if->mag.state = sensor_state->dae_if.mag.state;

#ifdef MMC5603X_DAE_FORCE_NOT_AVAILABLE
  sns_sensor_util_remove_sensor_instance_stream(this, &dae_if->mag.stream);
  dae_if->mag.stream_usable = false;
  return rc;
#endif

  if(IDLE == dae_if->mag.state)
  {
    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                   this,
                                                   *dae_suid,
                                                   &dae_if->mag.stream);

    if(NULL != dae_if->mag.stream)
    {
      sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
      build_static_config_request(sensor_state, &config_req, sensor_state->hardware_id);
      rc = send_static_config_request(dae_if->mag.stream, &config_req);
    }
  }

  if(SNS_RC_SUCCESS != rc)
  {
    mmc5603x_dae_if_deinit(this);
  }
  else
  {
    MMC5603X_INST_PRINT(ERROR, this, "dae ready");
    dae_if->mag.stream_usable   = true;
    dae_if->mag.status_bytes_per_fifo = 3; /* CNTL1, CNTL2, ST1 */
  }

  return rc;
}

/* ------------------------------------------------------------------------------------ */
void mmc5603x_dae_if_deinit(sns_sensor_instance *this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.mag.stream);
  state->dae_if.mag.state = PRE_INIT;
}

/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  mmc5603x_dae_if_info    *dae_if = &state->dae_if;

  if(stream_usable(&state->dae_if.mag) &&
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    MMC5603X_INST_PRINT(ERROR, this,"stopping mag stream=0x%x", &dae_if->mag.stream);
    cmd_sent |= stop_streaming(&dae_if->mag);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_start_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  MMC5603X_INST_PRINT(ERROR, this,"mmc5603x_dae_if_start_streaming");
  if(stream_usable(&state->dae_if.mag) && state->dae_if.mag.state > PRE_INIT &&
     (0 < state->mag_info.desired_odr))
  {
    cmd_sent |= send_mag_config(this);
  }
  return cmd_sent;
}
/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_flush_hw(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  mmc5603x_dae_if_info *dae_if = &((mmc5603x_instance_state*)this->state->state)->dae_if;
  MMC5603X_INST_PRINT(ERROR, this,"mmc5603x_dae_if_flush_hw");
  if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
  {
    if(!dae_if->mag.flushing_hw)
    {
      flush_hw(&dae_if->mag);
    }
    cmd_sent |= dae_if->mag.flushing_hw;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool mmc5603x_dae_if_flush_samples(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  mmc5603x_dae_if_info *dae_if = &((mmc5603x_instance_state*)this->state->state)->dae_if;
  MMC5603X_INST_PRINT(ERROR, this,"mmc5603x_dae_if_flush_samples");
  if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
  {
    if(!dae_if->mag.flushing_data)
    {
      flush_samples(&dae_if->mag);
    }
    cmd_sent |= dae_if->mag.flushing_data;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
void mmc5603x_dae_if_process_events(sns_sensor_instance *this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;

  process_events(this, &state->dae_if.mag);

  if(!state->dae_if.mag.stream_usable)
  {
    mmc5603x_dae_if_deinit(this);
  }
}

#else //defined(MMC5603X_ENABLE_DAE)

bool mmc5603x_dae_if_available(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void mmc5603x_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}
void mmc5603x_dae_if_process_sensor_events(sns_sensor *this)
{
  UNUSED_VAR(this);
}
bool mmc5603x_dae_if_is_initializing(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool mmc5603x_dae_if_is_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
sns_rc mmc5603x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  mmc5603x_state           *sensor_state)
{
  UNUSED_VAR(this);
  UNUSED_VAR(stream_mgr);
  UNUSED_VAR(dae_suid);
  UNUSED_VAR(sensor_state);
  return SNS_RC_NOT_AVAILABLE;
}
void mmc5603x_dae_if_deinit(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
bool mmc5603x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool mmc5603x_dae_if_start_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool mmc5603x_dae_if_flush_hw(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool mmc5603x_dae_if_flush_samples(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void mmc5603x_dae_if_process_events(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
#endif //defined(MMC5603X_ENABLE_DAE)

