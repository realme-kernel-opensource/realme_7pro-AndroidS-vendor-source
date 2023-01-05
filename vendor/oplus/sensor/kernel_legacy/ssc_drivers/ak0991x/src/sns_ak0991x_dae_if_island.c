/**
 * @file sns_ak0991x_dae_if_island.c
 *
 * AK0991X - DAE sensor interface
 *
 * Copyright (c) 2017-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2019 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_ak0991x_dae_if.h"

#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_dae.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

//#define AK0991X_DAE_FORCE_NOT_AVAILABLE

#ifndef SNS_MAX
#define SNS_MAX(a,b) ({ __auto_type _a = (a);    \
                        __auto_type _b = (b);    \
                        _a > _b ? _a : _b; })
#endif /* SNS_MAX */

#ifdef AK0991X_PATCH_FOR_DAE_S4S_DT_EVENT_FIELDS_ON_704
const pb_field_t sns_dae_s4s_dt_event_fields[3] = {
    PB_FIELD(  1, FIXED64 , REQUIRED, STATIC  , FIRST, sns_dae_s4s_dt_event, timestamp, timestamp, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, sns_dae_s4s_dt_event, dt_value, timestamp, 0),
    PB_LAST_FIELD
};
#endif

/*======================================================================================
  Helper Functions
  ======================================================================================*/
#if defined(AK0991X_ENABLE_DAE)
static void build_static_config_request(
  ak0991x_state             *sensor_state,
  sns_dae_set_static_config *config_req,
  int64_t hardware_id)
{
  if(hardware_id == 0)
  {
    sns_strlcpy(config_req->func_table_name, "ak0991x_hal_table",
               sizeof(config_req->func_table_name));
  }
  else
  {
    sns_strlcpy(config_req->func_table_name, "ak0991x_hal_table2",
               sizeof(config_req->func_table_name));
  }


#ifdef AK0991X_DAE_FORCE_POLLING
  config_req->interrupt      = SNS_DAE_INT_OP_MODE_POLLING;
  config_req->has_irq_config = false;
  config_req->has_ibi_config = false;
#else
  config_req->interrupt =
      (sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI) ?
          SNS_DAE_INT_OP_MODE_IBI :
      (sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ) ?
          SNS_DAE_INT_OP_MODE_IRQ : SNS_DAE_INT_OP_MODE_POLLING;
  config_req->has_irq_config = sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ;
  config_req->has_ibi_config = sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI;
#endif /* AK0991X_DAE_FORCE_POLLING */
  if(sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ)
  {
    config_req->irq_config = sensor_state->irq_config;
  }
  else if(sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI)
  {
    config_req->ibi_config             =
    (sns_ibi_req){ .dynamic_slave_addr = sensor_state->com_port_info.com_config.slave_control,
                   .bus_instance = sensor_state->com_port_info.com_config.bus_instance,
                   .ibi_data_bytes = 0, };
  }
  switch (sensor_state->device_select)
  {
  case AK09911:
  case AK09912:
  case AK09913:
  case AK09916C:
  case AK09916D:
  case AK09918:
    config_req->has_s4s_config       = false;
    break;
  case AK09915C:
  case AK09915D:
  case AK09917:
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
    config_req->has_s4s_config       = sensor_state->registry_cfg.sync_stream;
#else
    config_req->has_s4s_config       = false;
#endif // AK0991X_ENABLE_REGISTRY_ACCESS
    break;
  default:
    config_req->has_s4s_config       = false;
    break;
  }

  config_req->s4s_config.st_reg_addr = AKM_AK0991X_REG_SYT;
  config_req->s4s_config.st_reg_data = 0;
  config_req->s4s_config.dt_reg_addr = AKM_AK0991X_REG_DT;

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

static bool stream_usable(ak0991x_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

/* ------------------------------------------------------------------------------------ */
void ak0991x_send_mag_s4s_config(sns_sensor_instance *this, bool send_dt_event)
{
  ak0991x_instance_state *state      = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_stream     *dae_stream = &state->dae_if.mag;

  sns_dae_s4s_dynamic_config s4s_config_req = sns_dae_s4s_dynamic_config_init_default;
  uint8_t s4s_encoded_msg[sns_dae_s4s_dynamic_config_size];
  sns_request s4s_req = {
    .message_id  = SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG,
    .request     = s4s_encoded_msg,
    .request_len = 0
  };

  //This is T_Ph start moment at the first time
  s4s_config_req.ideal_sync_offset = state->polling_timer_start_time;
  s4s_config_req.sync_interval = sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000);
  s4s_config_req.resolution_ratio = AK0991X_S4S_RR;
  //st_delay is defined in the sns_dae.proto file
  //This is a hardware and sampling-rate dependent value which needs to be filled in by the vendor

  s4s_config_req.st_delay = s4s_config_req.sync_interval * 0.001f;

  // Ask DAE to send DT events until S4S is synchronized
  s4s_config_req.has_send_dt_event = true;
  s4s_config_req.send_dt_event = send_dt_event;

  
  if((s4s_req.request_len =
      pb_encode_request(s4s_encoded_msg,
                        sizeof(s4s_encoded_msg),
                        &s4s_config_req,
                        sns_dae_s4s_dynamic_config_fields,
                        NULL)) > 0)
  {
    // The mag driver on Q6 never receives this message. It only sends this message. It can be sent at any time
    if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &s4s_req))
    {
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static bool send_mag_config(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state      = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_stream     *dae_stream = &state->dae_if.mag;
  ak0991x_mag_info       *mag_info   = &state->mag_info;
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  uint16_t batch_num = 0;
  uint16_t wm;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };

  AK0991X_INST_PRINT(HIGH, this, "send_mag_config:: stream=0x%x, #clk_err_meas_count=%u/%u, in_clk_err_proc=%u, use_dri=%u",
                     dae_stream->stream, 
                     state->mag_info.clock_error_meas_count,
                     AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC,
                     state->in_clock_error_procedure,
                     state->mag_info.int_mode);

  if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING ||
      state->mag_info.clock_error_meas_count >= AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC)
  {
    wm = !mag_info->use_fifo ? 1 : ((mag_info->device_select == AK09917) ? 
                                    mag_info->cur_cfg.fifo_wmk : mag_info->max_fifo_size);

    if(state->mag_info.flush_only || state->mag_info.max_batch)
    {
      config_req.dae_watermark = UINT32_MAX;
    }
    else
    {
      batch_num = SNS_MAX(mag_info->cur_cfg.dae_wmk, 1) / wm;
      config_req.dae_watermark = batch_num * wm;
    }

    AK0991X_INST_PRINT(LOW, this, "cur_wmk=%d wm=%d batch_num=%u dae_watermark=%u",
        mag_info->cur_cfg.fifo_wmk,
        wm,
        batch_num,
        config_req.dae_watermark);
  }
  else
  {
    config_req.dae_watermark = wm = 1;  // go into clock error procedure
  }

  config_req.has_data_age_limit_ticks = true;
  if(state->mag_info.max_batch)
  {
    config_req.data_age_limit_ticks = UINT64_MAX;
  }
  else
  {
    config_req.data_age_limit_ticks = mag_info->flush_period * 1.1f;
  }

  config_req.has_polling_config  = (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING);
  if( config_req.has_polling_config )
  {
    config_req.polling_config.polling_interval_ticks =
      wm * ak0991x_get_sample_interval(state->mag_info.cur_cfg.odr);

    state->system_time = sns_get_system_time();

    //TODO: it looks like the polling offset will not be adjusted for S4S.
    //So it won't be synced with any other sensors
    config_req.polling_config.polling_offset = state->polling_timer_start_time;
  }
  config_req.has_accel_info = false;
  config_req.has_expected_get_data_bytes = true;
  config_req.expected_get_data_bytes = 
      wm * AK0991X_NUM_DATA_HXL_TO_ST2 + dae_stream->status_bytes_per_fifo;

  if(mag_info->use_sync_stream)
  {
    ak0991x_send_mag_s4s_config( this, true );
    config_req.polling_config.polling_offset +=
      sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000 * 0.001f);
  }

  AK0991X_INST_PRINT(HIGH, this, "send_mag_config:: sys= %u, pre_orphan= %u, polling_offset= %u interval= %u",
      (uint32_t)state->system_time,
      (uint32_t)state->pre_timestamp_for_orphan,
      (uint32_t)config_req.polling_config.polling_offset,
      (uint32_t)config_req.polling_config.polling_interval_ticks);

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: polling_offset=%X%08X sys=%X%08X inteval_tick=%u",
      (uint32_t)(config_req.polling_config.polling_offset>>32),
      (uint32_t)(config_req.polling_config.polling_offset & 0xFFFFFFFF),
      (uint32_t)(state->system_time>>32),
      (uint32_t)(state->system_time & 0xFFFFFFFF),
      (uint32_t)config_req.polling_config.polling_interval_ticks);

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: dae_watermark=%u, data_age_limit_ticks=0x%X%08X, wm=%u,expected_get_data_bytes=%d ",
                       config_req.dae_watermark,
                       (uint32_t)(config_req.data_age_limit_ticks>>32),
                       (uint32_t)(config_req.data_age_limit_ticks & 0xFFFFFFFF),
                        wm,
                        config_req.expected_get_data_bytes);

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

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: stream=0x%x, dae_stream=%d request_len=%d cmd_sent=%d",
      dae_stream->stream,
      (uint8_t)dae_stream->state,
      (uint8_t)req.request_len,
      (uint8_t)cmd_sent);

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_hw(ak0991x_dae_stream *dae_stream)
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
static bool flush_samples(ak0991x_dae_stream *dae_stream)
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
static bool stop_streaming(ak0991x_dae_stream *dae_stream)
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

/* ------------------------------------------------------------------------------------ */
static bool pause_s4s_streaming(ak0991x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
//    dae_stream->state = PAUSE_S4S_SCHEDULE;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static void process_fifo_samples(
  sns_sensor_instance *this,
  uint8_t             *buf,
  size_t              buf_len)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  uint16_t fifo_len = buf_len - state->dae_if.mag.status_bytes_per_fifo;
  uint32_t sampling_intvl;
  uint8_t dummy_buf[] = {0U,0U,0U,0U,0U,0U,0U,AK0991X_INV_FIFO_DATA}; // set INV bit 
  uint8_t *mag_data_buf = buf + state->dae_if.mag.status_bytes_per_fifo;

  //////////////////////////////
  // data buffer formed in sns_ak0991x_dae.c for non-fifo mode
  // buf[0] : CNTL1
  // buf[1] : CNTL2
  // buf[2] : ST1
  // buf[3] : HXL (HXH AK09917)
  // buf[4] : HXH (HXL AK09917)
  // buf[5] : HYL (HYH AK09917)
  // buf[6] : HYH (HYL AK09917)
  // buf[7] : HZL (HZH AK09917)
  // buf[8] : HZH (HZL AK09917)
  // buf[9] : TMPS
  // buf[10]: ST2
  //////////////////////////////

  ak0991x_mag_odr odr = (ak0991x_mag_odr)(buf[1] & 0x1F);
  uint16_t fifo_wmk = (state->mag_info.use_fifo) ? (uint8_t)(buf[0] & 0x1F) + 1 : 1;  // read value from WM[4:0]

  state->is_orphan = false;
  sampling_intvl = (ak0991x_get_sample_interval(odr) *
                    state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION;

  state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;

  // calculate num_samples
  if(!state->in_clock_error_procedure)
  {
    state->is_orphan = 
//        (state->dae_event_time < state->config_set_time); // use time
        ( odr != state->mag_info.cur_cfg.odr ) ||
        ( fifo_wmk != state->mag_info.cur_cfg.fifo_wmk );

    // calc num_samples
    {
      if(state->mag_info.use_fifo)
      {
        // num_samples update when FIFO enabled.
        if(state->mag_info.device_select == AK09917)
        {
          state->num_samples = buf[2] >> 2;
        }
        else if(state->mag_info.device_select == AK09915C || state->mag_info.device_select == AK09915D)
        {
          state->num_samples = fifo_wmk; // read value from WM[4:0]
        }
      }
      else
      {
        // num_samples update when Polling mode.
        if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)  // polling mode: *** Doesn't care FIFO+Polling ***
        {
          if( !state->this_is_the_last_flush )
          {
            // only timer event. skip all flush requests.
            state->num_samples = (state->irq_info.detect_irq_event) ? 1 : 0;
          }
          else
          {
            if( !state->fifo_flush_in_progress ) // last batch but not last flush
            {
              // only timer event. skip all flush requests.
              state->num_samples = (state->irq_info.detect_irq_event) ? 1 : 0;
            }
            else  // last flush data
            {
              state->num_samples = (state->system_time > state->pre_timestamp_for_orphan + sampling_intvl/2) ? 1 : 0;
            }
          }

          if( state->num_samples > 0 && state->fifo_flush_in_progress )
          {
            state->flush_sample_count++;
          }
          else
          {
            state->flush_sample_count = 0;
          }
        }
      }
    }

    if( state->is_orphan )  // orphan
    {
      AK0991X_INST_PRINT(LOW, this, "orphan num_samples=%d, pre_orphan=%u, event=%u, intvl=%u sys=%u offset=%u",
          state->num_samples,
          (uint32_t)state->pre_timestamp_for_orphan,
          (uint32_t)state->dae_event_time,
          (uint32_t)sampling_intvl,
          (uint32_t)state->system_time,
          (uint32_t)state->polling_timer_start_time);

    }
  }

  if((state->num_samples*AK0991X_NUM_DATA_HXL_TO_ST2) > fifo_len)
  {
    // for polling mode, always FIFO enabled. No sample data from DAE
    if( fifo_len == 0 &&
        state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING && 
        !state->mag_info.use_sync_stream )
    {
      AK0991X_INST_PRINT(HIGH, this, "num_samples=0 But forced to set 1, fifo_len=8.");
      state->num_samples = 1;
      fifo_len = AK0991X_NUM_DATA_HXL_TO_ST2;
      mag_data_buf = dummy_buf; // switch pointer to the dummy buf data.
    }
    else
    {
      SNS_INST_PRINTF(
        ERROR, this, "fifo_samples:: #samples %u disagrees with fifo len %u",
        state->num_samples, fifo_len);
        state->num_samples = fifo_len/AK0991X_NUM_DATA_HXL_TO_ST2;
    }
  }

  if( !state->is_orphan )
  {
    state->mag_info.data_count_for_dri += state->num_samples;
  }

  if(state->num_samples >= 1)
  {
    if(!state->in_clock_error_procedure)
    {
      state->data_over_run = (buf[2] & AK0991X_DOR_BIT) ? true : false;
      state->data_is_ready = (buf[2] & AK0991X_DRDY_BIT) ? true : false;

      if( !state->is_orphan ) // regular sequence
      {
        if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
        {
          ak0991x_validate_timestamp_for_dri(this);
          sampling_intvl = state->averaged_interval; // update sampling_intvl
        }
        else
        {
          state->interrupt_timestamp = state->dae_event_time;

#ifdef AK0991X_OPEN_SSC_711_PATCH_FOR_JITTER
          // use ideal interval for more than 50Hz ODR because of the timer jitter
          if( !state->this_is_first_data && (sampling_intvl < 384000 ) && 
              !(state->this_is_the_last_flush && state->fifo_flush_in_progress) ) 
          {
            state->interrupt_timestamp = state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples;
          }
#endif

          if( state->this_is_the_last_flush && state->fifo_flush_in_progress )
          {
            AK0991X_INST_PRINT(LOW, this, "last flush total= %d pre= %u ts= %u sys= %u p_off=%u",
                state->total_samples,
                (uint32_t)state->pre_timestamp_for_orphan,
                (uint32_t)state->interrupt_timestamp,
                (uint32_t)state->system_time,
                (uint32_t)state->polling_timer_start_time);
          }

          state->first_data_ts_of_batch = state->interrupt_timestamp;
          state->averaged_interval = sampling_intvl;
        }

#ifdef AK0991X_ENABLE_TS_DEBUG
        AK0991X_INST_PRINT(
          MED, this, "fifo_samples:: odr=0x%X intvl=%u #samples=%u ts=%X-%X first_data=%d",
          odr, (uint32_t)sampling_intvl, state->num_samples,
          (uint32_t)state->first_data_ts_of_batch,
          (uint32_t)state->irq_event_time,
          state->this_is_first_data);
#endif
      }
      else  // orphan
      {
        // when the pre_timestamp_for_orphan is too old or newer than the dae_event_time, use dae_event_time instead.
        if(state->irq_info.detect_irq_event && // dri or timer event
           (state->dae_event_time > state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples + sampling_intvl/5 ||
            state->dae_event_time < state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples) )
        {
          state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
        }
        else if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)  // dri
        {
          if(state->irq_info.detect_irq_event ||  // interrupt event
             state->dae_event_time < state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples)
          {
            state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
          }
          else
          {
            state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
          }
        }
        else  // polling
        {
          state->first_data_ts_of_batch = state->dae_event_time;
#ifdef AK0991X_OPEN_SSC_711_PATCH_FOR_JITTER
          // use ideal interval for more than 50Hz ODR because of the timer jitter
          if( !state->this_is_first_data && (sampling_intvl < 384000 ) && 
              !(state->this_is_the_last_flush && state->fifo_flush_in_progress) ) 
          {
            state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
          }
#endif


        }
        AK0991X_INST_PRINT(MED, this, "fifo_samples:: orphan batch odr=(%d->%d) num_samples=%d event_time=%u",
            odr,
            state->mag_info.cur_cfg.odr,
            state->num_samples,
            (uint32_t)state->dae_event_time);
      }

      ak0991x_process_mag_data_buffer(this,
                                      state->first_data_ts_of_batch,
                                      sampling_intvl,
                                      mag_data_buf,
                                      fifo_len);
    }
    else  // in clock error procedure
    {
      AK0991X_INST_PRINT(LOW, this, "ak0991x_clock_error_calc_procedure call in DAE");
      ak0991x_clock_error_calc_procedure(this, &buf[2]);
      if (!state->in_clock_error_procedure && ak0991x_dae_if_stop_streaming(this))
      {
        AK0991X_INST_PRINT(LOW, this, "DONE clock error procedure");
        state->config_step = AK0991X_CONFIG_UPDATING_HW;
      }
      else
      {
        AK0991X_INST_PRINT(HIGH, this, "Discarding %u stale samples.", state->num_samples);
      }
    }

    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) // for DRI mode
    {
      ak0991x_register_heart_beat_timer(this);
    }
    else  // for Polling mode
    {
      // No heart_beat_timer_event when orphan.
      // Just update heart_beat_timestamp in order to ignore performing unnecessary heart beat detection
      if( state->is_orphan )
      {
        state->heart_beat_timestamp = state->system_time;
        state->heart_beat_sample_count = 0;
        state->heart_beat_attempt_count = 0;
//        AK0991X_INST_PRINT(HIGH, this, "Reset HB timestamp %u", (uint32_t)state->heart_beat_timestamp);
      }
      else
      {
        // check heart beat fire time
//        ak0991x_heart_beat_timer_event(this);
      }
    }
  }
}

static void estimate_event_type(
    sns_sensor_instance *this,
    uint8_t* buf)
{
  //////////////////////////////
  // data buffer formed in sns_ak0991x_dae.c for non-fifo mode
  // buf[0] : CNTL1
  // buf[1] : CNTL2
  // buf[2] : ST1
  // buf[3] : HXL (HXH AK09917)
  // buf[4] : HXH (HXL AK09917)
  // buf[5] : HYL (HYH AK09917)
  // buf[6] : HYH (HYL AK09917)
  // buf[7] : HZL (HZH AK09917)
  // buf[8] : HZH (HZL AK09917)
  // buf[9] : TMPS
  // buf[10]: ST2
  //////////////////////////////

  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  uint8_t wm = (buf[1] & AK0991X_FIFO_BIT) ? (buf[0] & 0x1F) + 1 : 1;
  sns_time polling_timestamp;

  if( state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING ) // DRI
  {
    if( buf[2] & AK0991X_DRDY_BIT )
    {
      state->irq_info.detect_irq_event = true;  // regular DRI
    }
    else
    {
      state->fifo_flush_in_progress = true;  // Flush request
    }
  }
  else  // Polling
  {
    polling_timestamp = state->pre_timestamp + state->averaged_interval * wm;

    if( state->this_is_first_data )
    {
      state->irq_info.detect_irq_event = true;  // polling timer event
    }
    // there is a chance to get wrong result
    if( state->dae_event_time > polling_timestamp - state->averaged_interval/200 &&
        state->dae_event_time < polling_timestamp + state->averaged_interval/200 )
    {
      state->irq_info.detect_irq_event = true;  // polling timer event
    }
    else
    {
      state->fifo_flush_in_progress = true;  // Flush request
    }
  }
  AK0991X_INST_PRINT(LOW, this, "estimated result: detect_irq=%d flush=%d",
      (uint8_t)state->irq_info.detect_irq_event,
      (uint8_t)state->fifo_flush_in_progress);
}

/* ------------------------------------------------------------------------------------ */
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
    ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
    state->system_time = sns_get_system_time();
    state->dae_event_time = data_event.timestamp;
    state->irq_info.detect_irq_event = false;
    state->fifo_flush_in_progress = false;

    if(data_event.has_timestamp_type)
    {
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) // DRI
      {
        if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_HW_IRQ )
        {
          state->irq_info.detect_irq_event = true;  // DRI interrupt
        }
      }
      else  // Polling
      {
        if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_TIMER )
        {
          state->irq_info.detect_irq_event = true;  // timer event in polling mode
        }
      }

      if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_SYSTEM_TIME )
      {
        state->fifo_flush_in_progress = true;  // Flush request
      }
    }
    else
    {
      estimate_event_type(this, (uint8_t*)decode_arg.buf);
    }

    AK0991X_INST_PRINT(HIGH, this, "process_data_event:%u. flush=(%d,%d) count=(%d,%d) ts_type=%d",
        (uint32_t)data_event.timestamp,
        (uint8_t)state->fifo_flush_in_progress,
        state->dae_if.mag.flushing_data,
        state->mag_info.data_count_for_dri,
        state->total_samples,
        data_event.timestamp_type);

    if(state->irq_info.detect_irq_event)
    {
      state->irq_event_time = state->dae_event_time;
    }

    process_fifo_samples(
      this, (uint8_t*)decode_arg.buf, decode_arg.buf_len);

    if(state->irq_info.detect_irq_event)
    {
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
      {
        // When DAE enable, validate timestamp can return false.
        if(state->mag_info.data_count_for_dri == 0)
        {
          state->previous_irq_time = state->interrupt_timestamp;
        }
      }
      else
      {
        state->previous_irq_time = state->interrupt_timestamp;
      }
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_response(
  sns_sensor_instance *this,
  ak0991x_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_dae_resp resp = sns_dae_resp_init_default;
  if(pb_decode(pbstream, sns_dae_resp_fields, &resp))
  {
    switch(resp.msg_id)
    {
    case SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG:
      AK0991X_INST_PRINT(LOW, this, "STATIC_CONFIG - err=%u state=%u",
                         resp.err, dae_stream->state);
      if(SNS_STD_ERROR_NO_ERROR == resp.err)
      {
        dae_stream->state = IDLE;
        if(ak0991x_dae_if_start_streaming(this))
        {
          state->config_step = AK0991X_CONFIG_UPDATING_HW;
        }
        else
        {
          AK0991X_INST_PRINT(LOW, this, "ak0991x_dae_if_start_streaming return false - err=%u state=%u odr=%d dae_mag_state=%d",
                             resp.err, dae_stream->state, state->mag_info.cur_cfg.odr, state->dae_if.mag.state);
        }
      }
      else
      {
        /* DAE sensor does not have support for this driver */
        dae_stream->stream_usable = false;
        dae_stream->state = PRE_INIT;
        if(state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_continue_client_config(this, true);
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
      //The DAE environment will send the ST/DT messages to the HW automatically
      //without involvement of either the normal mag sensor driver,
      //or the mag sensor driver in DAE environment.
      //The ST/DT messages will continue to be sent automatically until the PAUSE_S4S
      //message is sent by the mag driver to the DAE.
      //
      //Send ST/DT command
      //ak0991x_s4s_handle_timer_event(this);
      break;
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
      AK0991X_INST_PRINT(LOW, this,"DAE_SET_STREAMING_CONFIG - err=%u state=%u",
                         resp.err, dae_stream->state);
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        if(SNS_STD_ERROR_NO_ERROR == resp.err)
        {
          dae_stream->state = STREAMING;
          // No reset call.
          ak0991x_reconfig_hw(this, false);
          state->config_step = AK0991X_CONFIG_IDLE;
          if(state->do_flush_after_change_config)
          {
            state->do_flush_after_change_config = false;
            AK0991X_INST_PRINT(LOW, this,"Flush after change config.");
            if(!state->flush_requested_in_dae)
            {
              state->flush_requested_in_dae = true;
              if(state->mag_info.use_fifo && state->mag_info.cur_cfg.fifo_wmk > 1)
              {
                ak0991x_dae_if_flush_hw(this);
              }
              else if(state->mag_info.cur_cfg.dae_wmk > 1)
              {
                ak0991x_dae_if_flush_samples(this);
              }
              else
              {
                ak0991x_send_fifo_flush_done(this);
              }
            }
          }
        }
        else
        {
          if(SNS_STD_ERROR_INVALID_STATE == resp.err &&
              state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)
          {
            AK0991X_INST_PRINT(LOW, this,"stop and restart dae streaming");
            ak0991x_dae_if_stop_streaming(this);
            state->config_step = AK0991X_CONFIG_UPDATING_HW;
          }
          else
          {
             dae_stream->state = IDLE;
          }
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_HW - err=%u state=%u config_step=%d",
                         resp.err, dae_stream->state, state->config_step);
      dae_stream->flushing_hw = false;

      if(state->config_step != AK0991X_CONFIG_IDLE)
      {
        ak0991x_dae_if_start_streaming(this);
        state->config_step = AK0991X_CONFIG_UPDATING_HW;
        ak0991x_dae_if_flush_samples(this);
      }
      else if(state->heart_beat_attempt_count >= 3)
      {
        // Perform a reset operation in an attempt to revive the sensor
        ak0991x_reconfig_hw(this, true);
      }
      else
      {
        ak0991x_dae_if_flush_samples(this); // Flush client request
      }

      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS:
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_DATA - err=%u state=%u config_step=%d num_samples=%d",
                         resp.err, dae_stream->state, state->config_step, state->num_samples);
      if(state->flush_requested_in_dae)
      {
        ak0991x_send_fifo_flush_done(this);
      }
      dae_stream->flushing_data = false;
      state->this_is_the_last_flush = false;
      state->wait_for_last_flush = false;

      if( !state->in_self_test && 
          state->mag_info.cur_cfg.num > state->mag_info.last_sent_cfg.num && 
          state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
      {
        AK0991X_INST_PRINT(HIGH, this, "Send new config #%d in DAE: odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
            state->mag_info.cur_cfg.num,
            (uint32_t)state->mag_info.cur_cfg.odr,
            (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
            (uint32_t)state->mag_info.cur_cfg.dae_wmk);

        ak0991x_send_config_event(this, true);  // send new config event
        ak0991x_send_cal_event(this, false);    // send previous cal event
      }

      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
      AK0991X_INST_PRINT(LOW, this,
                         "DAE_PAUSE_SAMPLING stream_state=%u if_state=%u config_step=%u",
                         dae_stream->state,
                         state->dae_if.mag.state,
                         state->config_step);

      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }

      if(dae_stream->state != STREAM_STOPPING)
      {
        if(state->config_step == AK0991X_CONFIG_STOPPING_STREAM)
        {
          state->this_is_the_last_flush = true;
          if(ak0991x_dae_if_flush_hw(this))
          {
            state->config_step = AK0991X_CONFIG_FLUSHING_HW;
            AK0991X_INST_PRINT(LOW, this,"Last flush before changing ODR.");
          }
        }
        else if(state->config_step == AK0991X_CONFIG_UPDATING_HW)
        {
          if(state->do_flush_after_clock_error_procedure)
          {
            state->do_flush_after_clock_error_procedure = false;
            AK0991X_INST_PRINT(LOW, this,"Flush after clock error procedure.");
            if(!state->flush_requested_in_dae)
            {
              state->flush_requested_in_dae = true;
              if(state->mag_info.use_fifo && state->mag_info.cur_cfg.fifo_wmk > 1)
              {
                ak0991x_dae_if_flush_hw(this);
              }
              else if(state->mag_info.cur_cfg.dae_wmk > 1)
              {
                ak0991x_dae_if_flush_samples(this);
              }
              else
              {
                ak0991x_send_fifo_flush_done(this);
              }
            }
          }
          else
          {
            if( !state->in_self_test && 
                state->mag_info.cur_cfg.num > state->mag_info.last_sent_cfg.num && 
                state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
            {
              AK0991X_INST_PRINT(HIGH, this, "Send new config #%d in DAE: odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
                  state->mag_info.cur_cfg.num,
                  (uint32_t)state->mag_info.cur_cfg.odr,
                  (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
                  (uint32_t)state->mag_info.cur_cfg.dae_wmk);

              ak0991x_send_config_event(this, true);  // send new config event
              ak0991x_send_cal_event(this, false);    // send previous cal event
            }
            ak0991x_dae_if_start_streaming(this);
          }
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
      // After DAE receivce PAUSE_S4S_SCHEDULE message,
      // Stop sending the ST/DT message to the HW automatically
      AK0991X_INST_PRINT(LOW, this,
                         "DAE_PAUSE_S4S_SCHED stream_state=%u if_state=%u config_step=%u",
                         dae_stream->state,
                         state->dae_if.mag.state,
                         state->config_step);
      //if(state->mag_info.use_sync_stream)
      //{
      //  state->mag_info.s4s_dt_abort = true;
      //  ak0991x_s4s_handle_timer_event(this);
      //  state->mag_info.s4s_dt_abort = false;
      //}
      break;

    case SNS_DAE_MSGID_SNS_DAE_RESP:
    case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
      break; /* unexpected */
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_dt_event(
  sns_sensor_instance *this,
  pb_istream_t        *pbstream)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_dae_s4s_dt_event dt_event = sns_dae_s4s_dt_event_init_default;
  if(pb_decode(pbstream, sns_dae_s4s_dt_event_fields, &dt_event))
  {
    if( dt_event.dt_value < 0x80 )
    {
      if( state->mag_info.s4s_sync_state < AK0991X_S4S_SYNCED )
      {
        state->mag_info.s4s_sync_state++;
      }
      if( state->mag_info.s4s_sync_state == AK0991X_S4S_1ST_SYNCED )
      {
        // Update config_set_time, since we need to send out phy config
        // event with new timestamp.
        state->config_set_time = state->pre_timestamp_for_orphan+1;

        // Send config event with stream_is_synchronous=true:
        ak0991x_send_config_event( this, true );

        // Turn off DT messages from DAE, since they're no longer needed:
        ak0991x_send_mag_s4s_config( this, false );

        // send previous cal event
        ak0991x_send_cal_event( this, false );
      }
    }
    else
    {
      if( state->mag_info.s4s_sync_state < AK0991X_S4S_1ST_SYNCED )
      {
        state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
      }
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_events(sns_sensor_instance *this, ak0991x_dae_stream *dae_stream)
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
        AK0991X_INST_PRINT(LOW, this,"DAE_DATA_EVENT");
        process_data_event(this, &pbstream);
      }
      else if(SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this,"DAE_INTERRUPT_EVENT");
      }
      else if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this,"SNS_DAE_RESP");
        process_response(this, dae_stream, &pbstream);
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
        dae_stream->stream_usable = false;
        AK0991X_INST_PRINT(LOW, this,"SNS_STD_ERROR_EVENT");
        if(dae_stream->state == INIT_PENDING && 
           state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_continue_client_config(this, true);
        }
      }
      else if(SNS_DAE_MSGID_SNS_DAE_S4S_DT_EVENT == event->message_id)
      {
        process_dt_event(this, &pbstream);
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this,"Unexpected message id %u", event->message_id);
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

void ak0991x_dae_if_check_support(sns_sensor *this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;

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
      AK0991X_PRINT(LOW, this, "check_support:: static config fail");
      state->dae_if.mag.state = UNAVAILABLE;
      sns_sensor_util_remove_sensor_stream(this, &state->dae_if.mag.stream);
    }
  }
}

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return stream_usable(&dae_if->mag);
}

void ak0991x_dae_if_process_sensor_events(sns_sensor *this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;
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
bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == INIT_PENDING);
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == STREAMING);
}

/* ------------------------------------------------------------------------------------ */
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  ak0991x_state           *sensor_state)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info* dae_if = &state->dae_if;

  AK0991X_INST_PRINT(LOW, this, "dae_if_init set inst mag.state %d <= %d", (int)dae_if->mag.state, (int)sensor_state->dae_if.mag.state);
  dae_if->mag.state = sensor_state->dae_if.mag.state;

#ifdef AK0991X_DAE_FORCE_NOT_AVAILABLE
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
    ak0991x_dae_if_deinit(this);
  }
  else
  {
    AK0991X_INST_PRINT(LOW, this, "dae ready");
    dae_if->mag.stream_usable   = true;
    dae_if->mag.status_bytes_per_fifo = 3; /* CNTL1, CNTL2, ST1 */
  }

  return rc;
}

/* ------------------------------------------------------------------------------------ */
void ak0991x_dae_if_deinit(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.mag.stream);
  state->dae_if.mag.state = PRE_INIT;
}

/* ------------------------------------------------------------------------------------ */
/*
bool ak0991x_dae_if_pause_s4s_schedule(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info    *dae_if = &state->dae_if;

  if(stream_usable(&state->dae_if.mag) && 
     state->mag_info.use_sync_stream &&
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    AK0991X_INST_PRINT(LOW, this,"pausing s4s schedule stream=0x%x", &dae_if->mag.stream);
    cmd_sent |= pause_s4s_streaming(&dae_if->mag);
  }
  return cmd_sent;
}
*/

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info    *dae_if = &state->dae_if;

  if(stream_usable(&state->dae_if.mag) &&
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    AK0991X_INST_PRINT(LOW, this,"stopping mag stream=0x%x", &dae_if->mag.stream);
    if( state->mag_info.use_sync_stream )
    {
      cmd_sent |= pause_s4s_streaming(&dae_if->mag);
    }
    cmd_sent |= stop_streaming(&dae_if->mag);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  if(stream_usable(&state->dae_if.mag) && state->dae_if.mag.state > PRE_INIT &&
     (AK0991X_MAG_ODR_OFF != state->mag_info.cur_cfg.odr))
  {
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
    {
      cmd_sent |= send_mag_config(this);
    }
    else
    {
      AK0991X_INST_PRINT(HIGH, this, "cur_cfg.num=%d, cur_cfg.odr=0x%02X, last_sent_cfg.num=%d, last_sent_cfg.odr=0x%02X",
          state->mag_info.cur_cfg.num,
          (uint32_t)state->mag_info.cur_cfg.odr,
          state->mag_info.last_sent_cfg.num,
          (uint32_t)state->mag_info.last_sent_cfg.odr);
      if(state->reg_event_for_dae_poll_sync)
      {
        cmd_sent |= send_mag_config(this);
        state->reg_event_for_dae_poll_sync = false;
      }
      else
      {
        // Register timer to synchronize the DAE polling timing with other sensors.
        ak0991x_register_timer(this);
        cmd_sent = true;
        AK0991X_INST_PRINT(LOW, this,"register timer with is_dry_run=ture");
      }
    }
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  if( state->mag_info.use_fifo || state->this_is_the_last_flush )
  {
    if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
    {
      if(!dae_if->mag.flushing_hw)
      {
        flush_hw(&dae_if->mag);
      }
      else
      {
        AK0991X_INST_PRINT( HIGH, this,"Already flushing_hw=%d", (uint8_t)dae_if->mag.flushing_hw );
      }
      cmd_sent |= dae_if->mag.flushing_hw;
    }

    if(!cmd_sent)
    {
      AK0991X_INST_PRINT( HIGH, this,"Failed to flush_hw state=%d", (uint8_t)dae_if->mag.state );
    }
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;

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
void ak0991x_dae_if_process_events(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  process_events(this, &state->dae_if.mag);

  if(!state->dae_if.mag.stream_usable)
  {
    ak0991x_dae_if_deinit(this);
  }
}

#else //defined(AK0991X_ENABLE_DAE)

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void ak0991x_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}
void ak0991x_dae_if_process_sensor_events(sns_sensor *this)
{
  UNUSED_VAR(this);
}
bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  ak0991x_state           *sensor_state)
{
  UNUSED_VAR(this);
  UNUSED_VAR(stream_mgr);
  UNUSED_VAR(dae_suid);
  UNUSED_VAR(sensor_state);
  return SNS_RC_NOT_AVAILABLE;
}
void ak0991x_dae_if_deinit(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void ak0991x_dae_if_process_events(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
#endif //defined(AK0991X_ENABLE_DAE)

