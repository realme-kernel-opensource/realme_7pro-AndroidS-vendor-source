/**
 * @file sns_mmc5603x_s4s.c
 *
 * MMC5603X - S4S functions
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_types.h"

#include "sns_mmc5603x_hal.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_s4s.h"

#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_sensor_util.h"

sns_rc mmc5603x_s4s_set_mag_config(sns_sensor_instance *const this)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
  sns_rc  rv = SNS_RC_SUCCESS;

  // Configure TPH and RR for S4S
  if((state->mag_info.desired_odr != MMC5603X_MAG_ODR_OFF) && state->mag_info.use_sync_stream)
  {
    uint8_t buf_s4s[3];
    uint32_t xfer_bytes;
    uint16_t t_ph_cnt;
    t_ph_cnt = (uint16_t)state->mag_req.sample_rate * (MMC5603X_S4S_INTERVAL_MS / 1000);

    buf_s4s[0] = 0x0
      | (1 << 7)                                   // TPH
      | ((t_ph_cnt >> 1) & 0x7F);                  // TPH1
    buf_s4s[1] = 0x0
      | ((t_ph_cnt >> 9) & 0xFF);                  // TPH2
    if(state->mag_info.device_select == 0)  //MEMSIC
    {
      buf_s4s[2] = 0x0
        | (state->mag_info.s4s_rr & 0x07);         // RR
    }
    else
    {
      buf_s4s[2] = 0x0
        | (state->mag_info.s4s_rr & 0x03);         // RR
    }

    MMC5603X_INST_PRINT(ERROR, this, "bf[0]=%d bf[1]=%d",buf_s4s[0], buf_s4s[1]);
    rv = mmc5603x_com_write_wrapper(this,
                                   state->scp_service,
                                   state->com_port_info.port_handle,
                                   MEMSIC_MMC5603X_REG_TPH1,
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
#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
#endif  // MMC5603X_ENABLE_S4S
}

void mmc5603x_s4s_send_config_event(sns_sensor_instance *const this,
                                   sns_std_sensor_physical_config_event *phy_sensor_config)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;

#if defined(MMC5603X_ENABLE_ALL_DEVICES)
    phy_sensor_config->has_stream_is_synchronous = false;
    phy_sensor_config->stream_is_synchronous = false;
#endif

#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
  phy_sensor_config->has_stream_is_synchronous = false;
  phy_sensor_config->stream_is_synchronous = false;
#endif // MMC5603X_ENABLE_S4S
}

void mmc5603x_s4s_inst_init(sns_sensor_instance *const this,
                           sns_sensor_state const *sstate)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;
  //mmc5603x_state *sensor_state = (mmc5603x_state *)sstate->state;

  /** Init Mag State */
  state->mag_info.s4s_sync_state = MMC5603X_S4S_NOT_SYNCED;
  state->mag_info.s4s_rr         = MMC5603X_S4S_RR;
  state->mag_info.s4s_dt_abort   = false;

#if defined(MMC5603X_ENABLE_ALL_DEVICES)
    state->mag_info.use_sync_stream = false;
#endif

  /** Initialize Timer info to be used by the Instance */
  state->s4s_timer_data_stream = NULL;

#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
  UNUSED_VAR(sstate);
#endif // MMC5603X_ENABLE_S4S
}

void mmc5603x_s4s_inst_deinit(sns_sensor_instance *const this)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;

  sns_sensor_util_remove_sensor_instance_stream(this,&state->s4s_timer_data_stream);
#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
#endif // MMC5603X_ENABLE_S4S
}

void mmc5603x_s4s_register_timer(sns_sensor_instance *const this)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state*)this->state->state;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  sns_request             timer_req;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[20] = {0};
  req_payload.is_periodic = true;
  req_payload.has_priority = true;
  sns_time                t_ph_period = sns_convert_ns_to_ticks(
      MMC5603X_S4S_INTERVAL_MS * 1000 * 1000);
  req_payload.start_time = sns_get_system_time() - t_ph_period;
  req_payload.start_config.early_start_delta = 0;
  req_payload.start_config.late_start_delta = t_ph_period;
  req_payload.priority = SNS_TIMER_PRIORITY_S4S;
  req_payload.timeout_period = t_ph_period;

  if (state->mag_req.sample_rate != MMC5603X_ODR_0)
  {
    MMC5603X_INST_PRINT(ERROR, this, "timeout_period=%u", (uint32_t)req_payload.timeout_period);
    MMC5603X_INST_PRINT(ERROR, this, "start_time=%u", (uint32_t)req_payload.start_time);
    MMC5603X_INST_PRINT(ERROR, this, "late_start_delta=%u", (uint32_t)req_payload.start_config.late_start_delta);

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
      MMC5603X_INST_PRINT(ERROR, this, "Fail to send request to Timer Sensor");
    }
  }
  else
  {
    state->mag_info.s4s_sync_state = MMC5603X_S4S_NOT_SYNCED;
    if (state->s4s_timer_data_stream != NULL)
    {
      stream_mgr->api->remove_stream(stream_mgr, state->s4s_timer_data_stream);
      state->s4s_timer_data_stream = NULL;
    }
  }
#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
#endif // MMC5603X_ENABLE_S4S
}

sns_rc mmc5603x_s4s_handle_timer_event(sns_sensor_instance *const instance)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)instance->state->state;

  MMC5603X_INST_PRINT(ERROR, instance, "handle s4s_timer event");

  sns_time t_ph_time = sns_get_system_time();
  sns_time i2c_start_time;
  uint8_t  buffer;
  uint16_t dt_count;
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  buffer = 0;
  // Send a ST command
  rv = mmc5603x_com_write_wrapper(instance,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
                                 MEMSIC_MMC5603X_REG_SYT,
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

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  dt_count = (i2c_start_time - t_ph_time) * (1 << state->mag_info.s4s_rr) * 2048
             / (float)sns_convert_ns_to_ticks(MMC5603X_S4S_INTERVAL_MS * 1000 * 1000);

  MMC5603X_INST_PRINT(ERROR, instance, "i2c_start_time=%u", (uint32_t)i2c_start_time);
  MMC5603X_INST_PRINT(ERROR, instance, "t_ph_time=%u", (uint32_t)t_ph_time);
  MMC5603X_INST_PRINT(ERROR, instance, "dt_count=%u", (uint32_t)dt_count);
  MMC5603X_INST_PRINT(ERROR, instance, "t_ph_interval=%u",(uint32_t)sns_convert_ns_to_ticks(MMC5603X_S4S_INTERVAL_MS * 1000 * 1000));


  if ( dt_count > 127 || state->mag_info.s4s_dt_abort)
  {
    buffer = 0x80;
  }
  else
  {
    buffer = (uint8_t)dt_count;
  }

  // Send a DT command and DT Data
  rv = mmc5603x_com_write_wrapper(instance,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
                                 MEMSIC_MMC5603X_REG_DT,
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
  if (dt_count >= 0x80)
  {
    //Even after sending a DT Abort command, the hardware will still stay synchronized using the previous ST/DT pairs.
    //If there are many DT Aborts in a row, the synchronization will slowly drift until it is no longer good.
    //However, just sending one DT Abort due to missing the timeline will probably not impact the timing significantly
    MMC5603X_INST_PRINT(ERROR, instance, "DT abort");
    return rv;
  }

  /* Checks the S4S synchronization state */
  if (state->mag_info.s4s_sync_state == MMC5603X_S4S_NOT_SYNCED)
  {
    state->mag_info.s4s_sync_state = MMC5603X_S4S_SYNCING;
    MMC5603X_INST_PRINT(ERROR, instance, "S4S syncing...");
  }
  else if (state->mag_info.s4s_sync_state == MMC5603X_S4S_SYNCING)
  {
    state->mag_info.s4s_sync_state = MMC5603X_S4S_1ST_SYNCED;
    MMC5603X_INST_PRINT(ERROR, instance, "S4S 1st synced");
  }
  else if (state->mag_info.s4s_sync_state == MMC5603X_S4S_1ST_SYNCED)
  {
    state->mag_info.s4s_sync_state = MMC5603X_S4S_SYNCED;
    mmc5603x_send_config_event(instance);
    MMC5603X_INST_PRINT(ERROR, instance, "S4S synced");
  }

  return rv;
#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(instance);
  return SNS_RC_SUCCESS;
#endif // MMC5603X_ENABLE_S4S
}

void mmc5603x_s4s_handle_timer_data_stream(sns_sensor_instance *const this)
{
#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;
  sns_sensor_event    *event;

  // Handle timer event for s4s
  if (NULL != state->s4s_timer_data_stream)
  {
    MMC5603X_INST_PRINT(ERROR, this, "s4s_timer_data_stream");
    event = state->s4s_timer_data_stream->api->peek_input(state->s4s_timer_data_stream);

    while (NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;

      if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
        {
          MMC5603X_INST_PRINT(ERROR, this, "Received config id=%d",
                                        event->message_id);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
          if(state->s4s_reg_event_done)
          {
            mmc5603x_s4s_handle_timer_event(this);
            MMC5603X_INST_PRINT(ERROR, this, "Execute handle s4s timer event");
          }
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
        {
          state->s4s_reg_event_done = true;
          MMC5603X_INST_PRINT(ERROR, this, "Execute handle tiemr s4s reg event");
        }
        else
        {
          MMC5603X_INST_PRINT(ERROR, this, "handle timer ERROR s4s");
        }
      }
      else
      {
        MMC5603X_INST_PRINT(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
      }

      event = state->s4s_timer_data_stream->api->get_next_input(state->s4s_timer_data_stream);
    }
  }
#else // MMC5603X_ENABLE_S4S
  UNUSED_VAR(this);
#endif // MMC5603X_ENABLE_S4S
}


