/**
 * @file sns_mmc5603x_sensor_instance_island.c
 *
 * MMC5603X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_island_service.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_mmc5603x_hal.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_s4s.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_sync_com_port_service.h"
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#endif


const odr_reg_map reg_map_mmc5603x[MMC5603X_REG_MAP_TABLE_SIZE] = {
  {
    .odr = MMC5603X_ODR_0,
    .mag_odr_reg_value = MMC5603X_MAG_ODR_OFF,
  },
  {
    .odr = MMC5603X_ODR_5,
    .mag_odr_reg_value = MMC5603X_MAG_ODR5,
  },
  {
    .odr = MMC5603X_ODR_10,
    .mag_odr_reg_value = MMC5603X_MAG_ODR10,
  },
  {
	  .odr = MMC5603X_ODR_15,
	  .mag_odr_reg_value = MMC5603X_MAG_ODR15,
   },

  {
    .odr = MMC5603X_ODR_25,
    .mag_odr_reg_value = MMC5603X_MAG_ODR25,
  },
  {
    .odr = MMC5603X_ODR_50,
    .mag_odr_reg_value = MMC5603X_MAG_ODR50,
  },
  {
    .odr = MMC5603X_ODR_100,
    .mag_odr_reg_value = MMC5603X_MAG_ODR100,
  },

};

static void mmc5603x_inst_exit_island(sns_sensor_instance *this)
{
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
}


static sns_rc mmc5603x_heart_beat_timer_event(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if(state->mag_info.desired_odr != MMC5603X_MAG_ODR_OFF)
  {  
      uint8_t heart_beat_thresthold = 4;
      if (state->heart_beat_sample_count < heart_beat_thresthold)
      {
        state->heart_beat_sample_count++;
      }
      else
      {
        MMC5603X_INST_PRINT(ERROR, this, "heart_beat_gap=%u, heart_beat_timeout=%u",
          (uint32_t)((state->interrupt_timestamp-state->heart_beat_timestamp)/19200),
          (uint32_t)(state->heart_beat_timeout_period/19200));
        // Detect streaming has stopped
        if (state->interrupt_timestamp > state->heart_beat_timestamp + state->heart_beat_timeout_period)
        {
          MMC5603X_INST_PRINT(ERROR, this, "Detect streaming has stopped");
          // Streaming is unable to resume after 3 attempts
          if (state->heart_beat_attempt_count >= 3)
          {
            mmc5603x_inst_exit_island(this);
            SNS_INST_PRINTF(ERROR, this, "Streaming is unable to resume after 3 attempts");
            rv = SNS_RC_INVALID_STATE;
          }
          // Perform a reset operation in an attempt to revive the sensor
          else
          {
            mmc5603x_inst_exit_island(this);
            rv = mmc5603x_device_sw_reset(this,
                                         state->scp_service,
                                         state->com_port_info.port_handle);
            if (rv == SNS_RC_SUCCESS) {
              MMC5603X_INST_PRINT(ERROR, this, "soft reset called");
            } else {
              SNS_INST_PRINTF(ERROR, this, "soft reset failed");
            }
            // Indicate streaming error
            rv = SNS_RC_NOT_AVAILABLE;
            mmc5603x_reconfig_hw(this);
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
  else
  {
    SNS_INST_PRINTF(ERROR, this, "heart beat timer event is skipped since ODR=0.");
  }

  return rv;
}



sns_rc mmc5603x_handle_device_mode_stream(sns_sensor_instance *const this)
{

  sns_rc rv = SNS_RC_FAILED;
  UNUSED_VAR(this);
  return rv;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc mmc5603x_inst_notify_event(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)this->state->state;
  sns_sensor_event    *event;
  sns_rc rv = SNS_RC_SUCCESS;

  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);
  MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_inst_notify_event...");

  mmc5603x_dae_if_process_events(this);

  // Handle timer event
  if (NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);
    MMC5603X_INST_PRINT(ERROR, this, "mmc5603x_inst_notify_event.. for handle handle timer memsic .");
    while (NULL != event)
    {
      if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);
        sns_timer_sensor_event timer_event;

        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          sns_time now = sns_get_system_time();
          state->system_time = timer_event.requested_timeout_time;
          if(state->system_time + state->nominal_intvl < now )
          {
            SNS_INST_PRINTF(ERROR, this, "req_to=%u now=%u", 
                            (uint32_t)timer_event.requested_timeout_time, (uint32_t)now);
          }
		  
          // for regular polling mode
          if (!state->mag_info.use_dri && state->reg_event_done)
          {
            MMC5603X_INST_PRINT(ERROR, this, "Execute handle timer event. now %u req_timeout_time %u",
                       (uint32_t)now,
                       (uint32_t)state->system_time);

            // mag data read
            mmc5603x_read_mag_samples(this);
          }
          else
          {
            // reset system time for heart beat timer on the DRI mode
            state->system_time = now;
          }

          // check heart beat fire time
          if(now > state->hb_timer_fire_time)
          {
            rv = mmc5603x_heart_beat_timer_event(this);
          }
          else
          {
            SNS_INST_PRINTF(ERROR, this, "Wrong HB timer fired. fire_time %u now %u",(uint32_t)state->hb_timer_fire_time, (uint32_t)now );
          }
        }
        else
        {
          SNS_INST_PRINTF(ERROR, this, "Failed decoding timer event");
        }
      }
      else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
      {
        //TODO:add support for handling SNS_TIMER_SENSOR_REG_EVENT timer event to successfully support S4S
        // When a range of start times is provided to the timer sensor, the timer sensor will pick a specific time.
        // That specific time will be returned in the SNS_TIMER_SENSOR_REG_EVENT event --
        // and will be needed by the mag sensor to populate the fields sent to the DAE sensor(so that timers remain synchronized in the DAE environment),
        // and the field in the Physical Sensor Config event (which needs absolute timing for the future events).
        if(!state->mag_info.use_dri)
        {
          state->reg_event_done = true;
        }
      }
      else
      {
        MMC5603X_INST_PRINT(ERROR, this, "Received invalid event id=%d", event->message_id);
      }

      if(NULL != state->timer_data_stream)
      {
        event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
      }
    }
  }

  // Handle timer data stream for S4S
  mmc5603x_s4s_handle_timer_data_stream(this);

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);
  return rv;
}

/** Public Data Definitions. */

sns_sensor_instance_api mmc5603x_sensor_instance_api = {
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &mmc5603x_inst_init,
  .deinit            = &mmc5603x_inst_deinit,
  .set_client_config = &mmc5603x_inst_set_client_config,
  .notify_event      = &mmc5603x_inst_notify_event
};
