/**
 * @file sns_ak0991x_s4s.c
 *
 * AK0991X - S4S functions
 *
 * Copyright (c) 2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"
#include "sns_ak0991x_dae_if.h"

#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_sensor_util.h"

sns_rc ak0991x_s4s_set_mag_config(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
  sns_rc  rv = SNS_RC_SUCCESS;

  // Configure TPH and RR for S4S
  if((state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF) && state->mag_info.use_sync_stream)
  {
    uint8_t buf_s4s[3];
    uint32_t xfer_bytes;
    uint16_t t_ph_cnt;
#ifdef AK0991X_ENABLE_S4S_TEST
    t_ph_cnt = 80;//TEST
//    t_ph_cnt = (uint16_t)ak0991x_get_mag_odr(state->mag_info.cur_cfg.odr) * (AK0991X_S4S_INTERVAL_MS / 1000);
#else
    t_ph_cnt = (uint16_t)ak0991x_get_mag_odr(state->mag_info.cur_cfg.odr) * (AK0991X_S4S_INTERVAL_MS / 1000);
#endif

    AK0991X_INST_PRINT(LOW, this, "t_ph_cnt= %d odr = %d/1000",t_ph_cnt, (int)(1000*ak0991x_get_mag_odr(state->mag_info.cur_cfg.odr)));

    buf_s4s[0] = 0x0
      | (1 << 7)                                   // TPH
      | ((t_ph_cnt >> 1) & 0x7F);                  // TPH1
    buf_s4s[1] = 0x0
      | ((t_ph_cnt >> 9) & 0xFF);                  // TPH2
    if(state->mag_info.device_select == AK09917)
    {
      buf_s4s[2] = 0x0
        | (state->mag_info.s4s_rr & 0x07);         // RR
    }
    else
    {
      buf_s4s[2] = 0x0
        | (state->mag_info.s4s_rr & 0x03);         // RR
    }

    AK0991X_INST_PRINT(LOW, this, "bf[0]=%d bf[1]=%d",buf_s4s[0], buf_s4s[1]);
    rv = ak0991x_com_write_wrapper(this,
                                   state->scp_service,
                                   state->com_port_info.port_handle,
                                   AKM_AK0991X_REG_TPH1,
                                   &buf_s4s[0],
                                   3,
                                   &xfer_bytes,
                                   false);

    if (xfer_bytes != 3)
    {
      rv = SNS_RC_FAILED;
    }
  }

  return rv;
}

void ak0991x_s4s_send_config_event(sns_sensor_instance *const this,
                                   sns_std_sensor_physical_config_event *phy_sensor_config)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;

  switch (state->mag_info.device_select)
  {
  case AK09911:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  case AK09912:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  case AK09913:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  case AK09915C:
    phy_sensor_config->has_stream_is_synchronous = state->mag_info.use_sync_stream;
    phy_sensor_config->stream_is_synchronous =
       (state->mag_info.s4s_sync_state >= AK0991X_S4S_1ST_SYNCED)? true : false;
    break;
  case AK09915D:
    phy_sensor_config->has_stream_is_synchronous = state->mag_info.use_sync_stream;
    phy_sensor_config->stream_is_synchronous =
        (state->mag_info.s4s_sync_state >= AK0991X_S4S_1ST_SYNCED)? true : false;
    break;
  case AK09916C:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  case AK09916D:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  case AK09917:
    phy_sensor_config->has_stream_is_synchronous = state->mag_info.use_sync_stream;
    phy_sensor_config->stream_is_synchronous =
        (state->mag_info.s4s_sync_state >= AK0991X_S4S_1ST_SYNCED)? true : false;
    break;
  case AK09918:
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
    break;
  default:
    break;
  }
}

void ak0991x_s4s_inst_init(sns_sensor_instance *const this,
                           sns_sensor_state const *sstate)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  ak0991x_state *sensor_state = (ak0991x_state *)sstate->state;
#else
  UNUSED_VAR(sstate);
#endif // AK0991X_ENABLE_REGISTRY_ACCESS

  /** Init Mag State */
  state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
  state->mag_info.s4s_rr         = AK0991X_S4S_RR;
  state->mag_info.s4s_dt_abort   = false;

  switch (state->mag_info.device_select)
  {
  case AK09911:
  case AK09912:
  case AK09913:
  case AK09916C:
  case AK09916D:
  case AK09918:
    state->mag_info.use_sync_stream = false;
    break;
  case AK09915C:
  case AK09915D:
  case AK09917:
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
    state->mag_info.use_sync_stream = sensor_state->registry_cfg.sync_stream;
#else
    state->mag_info.use_sync_stream = false;
#endif // AK0991X_ENABLE_REGISTRY_ACCESS
    break;
  default:
    state->mag_info.use_sync_stream = false;
    break;
  }

  /** Initialize Timer info to be used by the Instance */
  state->s4s_timer_data_stream = NULL;
}

void ak0991x_s4s_inst_deinit(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this,&state->s4s_timer_data_stream);
}

void ak0991x_s4s_register_timer(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  sns_request             timer_req;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[100] = {0};
  req_payload.is_periodic = true;
  req_payload.has_priority = true;
  sns_time                t_ph_period = sns_convert_ns_to_ticks(
      AK0991X_S4S_INTERVAL_MS * 1000 * 1000);
  req_payload.start_time = sns_get_system_time() - t_ph_period;
  req_payload.start_config.early_start_delta = 0;
  req_payload.start_config.late_start_delta = t_ph_period;
  req_payload.priority = SNS_TIMER_PRIORITY_S4S;
  req_payload.timeout_period = t_ph_period;
  req_payload.has_is_dry_run = true;
  req_payload.is_dry_run = ak0991x_dae_if_available(this);

  if (state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
  {
    AK0991X_INST_PRINT(LOW, this, "timeout_period=   %u", (uint32_t)req_payload.timeout_period);
    AK0991X_INST_PRINT(LOW, this, "start_time=       %u", (uint32_t)req_payload.start_time);
    AK0991X_INST_PRINT(LOW, this, "late_start_delta= %u", (uint32_t)req_payload.start_config.late_start_delta);

    if (NULL == state->s4s_timer_data_stream)
    {
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                     this,
                                                     state->timer_suid,
                                                     &state->s4s_timer_data_stream
                                                     );
    }

    req_len = pb_encode_request(buffer,
                                sizeof(buffer),
                                &req_payload,
                                sns_timer_sensor_config_fields,
                                NULL);

    if (req_len > 0)
    {
      timer_req.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG;
      timer_req.request_len = req_len;
      timer_req.request = buffer;

      /** Send encoded request to Timer Sensor */
      state->s4s_timer_data_stream->api->send_request(state->s4s_timer_data_stream, &timer_req);
    }
    else
    {
      AK0991X_INST_PRINT(ERROR, this, "Fail to send request to Timer Sensor");
    }
  }
  else
  {
    state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
    if (state->s4s_timer_data_stream != NULL)
    {
      stream_mgr->api->remove_stream(stream_mgr, state->s4s_timer_data_stream);
      state->s4s_timer_data_stream = NULL;
    }
  }
}

#ifdef AK0991X_OPEN_SSC_704_PATCH
// patch for OpenSSC7.0.4
static sns_time convert_bus_ts( sns_time bus_ts )
{
  sns_time ts;
  sns_time now = sns_get_system_time();
  bus_ts = 0xFFFFFFFFULL & bus_ts;
  ts = (now & (UINT64_MAX << 32 << 7)) | (bus_ts<<7);
  
  if(ts > now)
  {
    // rollover -- subtract 1<<32<<7;
    ts -= 1ULL<<32<<7;
  }
  return ts;
}
#endif


sns_rc ak0991x_s4s_handle_timer_event(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

//  AK0991X_INST_PRINT(LOW, instance, "handle s4s_timer event");

  sns_time i2c_start_time;
  uint8_t  buffer;
  uint16_t dt_count;
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  buffer = 0;
  // Send a ST command
  rv = ak0991x_com_write_wrapper(instance,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
                                 AKM_AK0991X_REG_SYT,
                                 &buffer,
                                 1,
                                 &xfer_bytes,
                                 true);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // Get the start time for s4s
  rv = state->scp_service->api->sns_scp_get_write_time(state->com_port_info.port_handle,
                                                       &i2c_start_time);

#ifdef AK0991X_OPEN_SSC_704_PATCH
  i2c_start_time = convert_bus_ts(i2c_start_time);  // patch for OpenSSC7.0.4
#endif

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  dt_count = (i2c_start_time - state->s4s_tph_start_time) * (1 << state->mag_info.s4s_rr) * 2048ULL
             / sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000ULL);

  AK0991X_INST_PRINT(LOW, instance, "i2c_start_time=%u", (uint32_t)i2c_start_time);
  AK0991X_INST_PRINT(LOW, instance, "t_ph_time=     %u", (uint32_t)state->s4s_tph_start_time);
  AK0991X_INST_PRINT(LOW, instance, "t_ph_interval= %u", (uint32_t)sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000ULL));
  AK0991X_INST_PRINT(LOW, instance, "rr=            %u", (uint32_t)state->mag_info.s4s_rr);
  AK0991X_INST_PRINT(LOW, instance, "dt_count=      %u", (uint32_t)dt_count);

  if ( dt_count > 127 || state->mag_info.s4s_dt_abort)
  {
    buffer = 0x80;
  }
  else
  {
    buffer = (uint8_t)dt_count;
  }

  // Send a DT command and DT Data
  rv = ak0991x_com_write_wrapper(instance,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
                                 AKM_AK0991X_REG_DT,
                                 &buffer,
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

  /* Processes DT abort */
  if (dt_count > 127 || state->mag_info.s4s_dt_abort)
  {
    //Even after sending a DT Abort command, the hardware will still stay synchronized using the previous ST/DT pairs.
    //If there are many DT Aborts in a row, the synchronization will slowly drift until it is no longer good.
    //However, just sending one DT Abort due to missing the timeline will probably not impact the timing significantly
    AK0991X_INST_PRINT(LOW, instance, "DT abort");
    return rv;
  }

  /* Checks the S4S synchronization state */
  if (state->mag_info.s4s_sync_state == AK0991X_S4S_NOT_SYNCED)
  {
    state->mag_info.s4s_sync_state = AK0991X_S4S_SYNCING;
    AK0991X_INST_PRINT(LOW, instance, "S4S syncing...");
  }
  else if (state->mag_info.s4s_sync_state == AK0991X_S4S_SYNCING)
  {
    state->mag_info.s4s_sync_state = AK0991X_S4S_1ST_SYNCED;
    AK0991X_INST_PRINT(LOW, instance, "S4S 1st synced");
    ak0991x_send_config_event(instance, true);  // send new config event
    ak0991x_send_cal_event(instance, false);    // send previous cal event
  }
  else if (state->mag_info.s4s_sync_state == AK0991X_S4S_1ST_SYNCED)
  {
    state->mag_info.s4s_sync_state = AK0991X_S4S_SYNCED;
    AK0991X_INST_PRINT(LOW, instance, "S4S synced");
  }

  return rv;
}

void ak0991x_s4s_handle_timer_data_stream(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
  sns_sensor_event    *event;

  // Handle timer event for s4s
  if (NULL != state->s4s_timer_data_stream)
  {
    event = state->s4s_timer_data_stream->api->peek_input(state->s4s_timer_data_stream);

    while (NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);

      if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this, "Received config id=%d",
                                      event->message_id);
      }
      else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
      {
        sns_timer_sensor_event timer_event;
        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if(state->s4s_reg_event_done)
          {
            state->s4s_tph_start_time = timer_event.timeout_time;
            AK0991X_INST_PRINT(LOW, this, "Execute handle s4s timer event tph= %u",
              (uint32_t)state->s4s_tph_start_time);
            ak0991x_s4s_handle_timer_event(this); // send ST/DT
          }
        }
      }
      else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
      {
        sns_timer_sensor_reg_event timer_reg_event;
        if (pb_decode(&stream, sns_timer_sensor_reg_event_fields, &timer_reg_event))
        {
          state->s4s_tph_start_time = timer_reg_event.start_time;
          AK0991X_INST_PRINT(LOW, this, "Execute handle tiemr s4s reg event: tph start time= %u",
            (uint32_t)state->s4s_tph_start_time);
          state->s4s_reg_event_done = true;
        }
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "handle timer ERROR s4s");
      }

      event = state->s4s_timer_data_stream->api->get_next_input(state->s4s_timer_data_stream);
    }
  }
}


