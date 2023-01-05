/**
 * @file sns_ak0991x_sensor_instance_island.c
 *
 * AK0991X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
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

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_sync_com_port_service.h"

const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE] = {
  {
    .odr = AK0991X_ODR_0,
    .mag_odr_reg_value = AK0991X_MAG_ODR_OFF,
  },
  {
    .odr = AK0991X_ODR_1,
    .mag_odr_reg_value = AK0991X_MAG_ODR1,
  },
  {
    .odr = AK0991X_ODR_10,
    .mag_odr_reg_value = AK0991X_MAG_ODR10,
  },
  {
    .odr = AK0991X_ODR_20,
    .mag_odr_reg_value = AK0991X_MAG_ODR20,
  },
  {
    .odr = AK0991X_ODR_50,
    .mag_odr_reg_value = AK0991X_MAG_ODR50,
  },
  {
    .odr = AK0991X_ODR_100,
    .mag_odr_reg_value = AK0991X_MAG_ODR100,
  },
  {
    .odr = AK0991X_ODR_200,
    .mag_odr_reg_value = AK0991X_MAG_ODR200,
  }
};

static void ak0991x_process_com_port_vector(sns_port_vector *vector,
                                     void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  if (AKM_AK0991X_REG_HXL == vector->reg_addr)
  {
    if(vector->bytes != 0){
      ak0991x_process_mag_data_buffer(instance,
                                      state->first_data_ts_of_batch,
                                      state->averaged_interval,
                                      vector->buffer,
                                      vector->bytes);
    }
    else{
      AK0991X_INST_PRINT(LOW, instance, "skip ak0991x_process_mag_data_buffer because vector->bytes=%d detected.", vector->bytes);
    }
  }
}

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
static bool sns_device_mode_event_decode_cb(pb_istream_t *stream, const pb_field_t *field,
    void **arg)
{
  UNUSED_VAR(field);
  ak0991x_instance_state *state = (ak0991x_instance_state *)*arg;
  if(!pb_decode(stream, sns_device_mode_event_mode_spec_fields, &state->device_mode[state->device_mode_cnt]))
  {
    return false;
  }
  else
  {
    state->device_mode_cnt++;
  }
  return true;
}

static void ak0991x_device_mode2cal_id(sns_sensor_instance *const instance)
{
  uint32_t cal_id = 0;
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  uint32_t cnt = SNS_MIN(state->device_mode_cnt, MAX_DEVICE_MODE_SUPPORTED);

  for(int i = 0; i < cnt; ++i)
  {
    if(state->device_mode[i].mode == SNS_DEVICE_MODE_UNKNOWN && 
       state->device_mode[i].state == SNS_DEVICE_STATE_ACTIVE)
    {
      // -1 denotes unknown device mode 
      cal_id = AK0991X_UNKNOWN_DEVICE_MODE;
      break;
    }
    uint8_t state_set = state->device_mode[i].state == SNS_DEVICE_STATE_ACTIVE ? 0 : 1;
    if(state_set)
    {
      cal_id |= state_set << (state->device_mode[i].mode - SNS_DEVICE_MODE_FLIP_OPEN);
      state->prev_cal_id = cal_id;  // store previous cal id for AK0991X_UNKNOWN_DEVICE_MODE
    }
  }
  state->cal.id = cal_id;
}
#endif // AK0991X_ENABLE_DEVICE_MODE_SENSOR

static sns_rc ak0991x_handle_device_mode_stream(sns_sensor_instance *const this)
{
#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
  sns_sensor_event    *event;
  sns_rc rv = SNS_RC_FAILED;

  if (NULL != state->device_mode_stream)
  {
    event = state->device_mode_stream->api->peek_input(state->device_mode_stream);

    while (NULL != event)
    {
      if(event->message_id == SNS_DEVICE_MODE_MSGID_SNS_DEVICE_MODE_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);
        sns_device_mode_event device_mode_event = sns_device_mode_event_init_default;
        state->device_mode_cnt = 0;
        device_mode_event.device_mode.arg = (void*)state;
        device_mode_event.device_mode.funcs.decode = sns_device_mode_event_decode_cb;
        if(!pb_decode(&stream, sns_device_mode_event_fields, &device_mode_event))
        {
          SNS_INST_PRINTF(ERROR, this, "Error in decoding device mode event!");
        }
        else
        {
          AK0991X_INST_PRINT(LOW, this, "Received %u device mode specs", state->device_mode_cnt);
          for(int i = 0; i < state->device_mode_cnt; ++i)
          {
            AK0991X_INST_PRINT(LOW, this, "mode: %u, state: %u", state->device_mode[i].mode,
                                                                 state->device_mode[i].state);
          }
          ak0991x_device_mode2cal_id(this);
          rv = SNS_RC_SUCCESS;
        }
      }
      event = state->device_mode_stream->api->get_next_input(state->device_mode_stream);
    }
  }
#else // AK0991X_ENABLE_DEVICE_MODE_SENSOR
  sns_rc rv = SNS_RC_FAILED;
  UNUSED_VAR(this);
#endif // AK0991X_ENABLE_DEVICE_MODE_SENSOR

  return rv;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc ak0991x_inst_notify_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_sensor_event    *event;
  sns_rc rv = SNS_RC_SUCCESS;

  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);

  ak0991x_dae_if_process_events(this);

  // Handle interrupts
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  if (NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);

    while (NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);

        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
          if(!state->in_clock_error_procedure)
          {
            if(state->ascp_xfer_in_progress == 0)
            {
              AK0991X_INST_PRINT(LOW, this, "   %u Detect interrupt.",(uint32_t)irq_event.timestamp);

              // check DRDY status.
              ak0991x_get_st1_status(this);

              if(state->data_is_ready)
              {
                state->irq_event_time = irq_event.timestamp;
                state->irq_info.detect_irq_event = true; // detect interrupt
                state->system_time = sns_get_system_time();
                ak0991x_read_mag_samples(this);
              }
            }
            else
            {
              AK0991X_INST_PRINT(LOW, this, "   %u Detect interrupt. But ascp_xfer_in_progress=%d.",
                  (uint32_t)irq_event.timestamp,
                  state->ascp_xfer_in_progress);
              state->re_read_data_after_ascp = true;
            }
          }
          else
          {
            state->irq_event_time = irq_event.timestamp;
            ak0991x_clock_error_calc_procedure(this, NULL);
            if (!state->in_clock_error_procedure)
            {
              // actual ODR measurement start.
              ak0991x_reconfig_hw(this, false);
            }
          }
        }
        else
        {
          SNS_INST_PRINTF(ERROR, this, "Failed decoding interrupt event");
        }
      }
      else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        state->irq_info.is_ready = true;
        ak0991x_reconfig_hw(this, false);
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid interrupt event id=%d",
                                      event->message_id);
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);

      if(NULL != event)
      {
        AK0991X_INST_PRINT(ERROR, this, "Still have int event in the queue... %u DRDY= %d", 
                           (uint32_t)sns_get_system_time(), state->data_is_ready);
      }
    }
  }

  // Handle Async Com Port events
  if (NULL != state->async_com_port_data_stream)
  {
    event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);

    while (NULL != event)
    {
      if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);

        sns_ascp_for_each_vector_do(&stream, ak0991x_process_com_port_vector, (void *)this);

        if(state->ascp_xfer_in_progress>0)
        {
          state->ascp_xfer_in_progress--;
        }
        else
        {
          AK0991X_INST_PRINT(ERROR, this, "ascp_xfer_in_progress is already 0. Can't decriment.");
        }

        if(state->ascp_xfer_in_progress>0)
        {
          AK0991X_INST_PRINT(LOW, this, "ascp_xfer_in_progress = %d", state->ascp_xfer_in_progress);
        }

        if(state->re_read_data_after_ascp && (state->ascp_xfer_in_progress == 0))
        {
          ak0991x_read_mag_samples(this);
          state->re_read_data_after_ascp = false;
          state->this_is_the_last_flush = false;
          state->wait_for_last_flush = false;
        }

        if(state->config_mag_after_ascp_xfer)
        {
          state->config_mag_after_ascp_xfer = false;
          ak0991x_continue_client_config(this, true);
        }
      }
      else if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
      {
        SNS_INST_PRINTF(ERROR, this, "Received ASYNC_COM_PORT_ERROR");
      }

      event = state->async_com_port_data_stream->api->get_next_input(
          state->async_com_port_data_stream);
    }
  }

  // Handle timer event
  if (NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);

    while (NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                    event->event_len);

      if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
      {
        sns_timer_sensor_event timer_event;
        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          sns_time now = sns_get_system_time();

          // for regular polling mode
          if (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING &&
              state->reg_event_done &&
              timer_event.requested_timeout_time != 0)
          {
            state->system_time = timer_event.requested_timeout_time;
//            state->system_time = timer_event.timeout_time;  // for S4S test
            AK0991X_INST_PRINT(LOW, this, "Polling: now %u event %u req %u",
                       (uint32_t)now,
                       (uint32_t)state->system_time,
                       (uint32_t)timer_event.requested_timeout_time);

            // mag data read
            ak0991x_read_mag_samples(this);
          }
          else
          {
            // reset system time for heart beat timer on the DRI mode
            state->system_time = now;
          }

          // check heart beat fire time
          if(now > state->hb_timer_fire_time)
          {
            rv = ak0991x_heart_beat_timer_event(this);
          }
          else
          {
//            AK0991X_INST_PRINT(ERROR, this, "Wrong HB timer fired. fire_time %u now %u",(uint32_t)state->hb_timer_fire_time, (uint32_t)now );
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
        // and the field in the Physical Sensor Config event (which needs absolute stiming for the future events).
        if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)
        {
          sns_timer_sensor_reg_event timer_reg_event;
          if (pb_decode(&stream, sns_timer_sensor_reg_event_fields, &timer_reg_event))
          {
            //if(state->polling_timer_start_time == 0)
            //{
              state->polling_timer_start_time = timer_reg_event.start_time + timer_reg_event.timeout_period; // set actual polling timer start time
              state->pre_timestamp = state->polling_timer_start_time;
              if(state->mag_info.use_sync_stream)
              {
                state->pre_timestamp +=
                  sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000 * 0.001f); //This is a hardware and sampling-rate dependent value
              }
              state->reg_event_done = true;
              AK0991X_INST_PRINT(LOW, this, "Received TIMER_SENSOR_REG_EVENT odr=%d wm=%d dae_wm=%d last_cfg.num= %d cur_cfg.num= %d",
                state->mag_info.cur_cfg.odr,
                state->mag_info.cur_cfg.fifo_wmk,
                state->mag_info.cur_cfg.dae_wmk,
                state->mag_info.last_sent_cfg.num,
                state->mag_info.cur_cfg.num);

              if( state->mag_info.use_sync_stream )
              {
                state->has_sync_ts_anchor = true;
              }
              //This value should be a timestamp(ideally in the nearby future) of a valid synchronized sample
              //Example: if running at exactly 100Hz, samples will be 10ms apart.
              //If the stream is synchronized to time 1234ms -- ,
              //then valid values to put into here would be "1234ms + (10ms * N)" for any (reasonable) value of N.
              state->sync_ts_anchor = state->pre_timestamp + state->req_payload.timeout_period - state->half_measurement_time;

              // very first time after inst init, and when new request received before sending config event(cur-last>1) to prevent WaitForEvents
              if( !state->in_self_test && 
                  (!ak0991x_dae_if_available(this) || 
                   state->mag_info.last_sent_cfg.num == 0 || 
                   state->mag_info.cur_cfg.num - state->mag_info.last_sent_cfg.num > 1 ))  // wait for in order to send config in DAE
              {
                if(state->this_is_the_last_flush)
                {
                  AK0991X_INST_PRINT(LOW, this, "Wait for the send_config_event until a flush is done.");
                }
                else
                {
                  AK0991X_INST_PRINT(HIGH, this, "Send new config #%d in REG_EVENT: odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
                    state->mag_info.cur_cfg.num,
                    (uint32_t)state->mag_info.cur_cfg.odr,
                    (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
                    (uint32_t)state->mag_info.cur_cfg.dae_wmk);

                  ak0991x_send_config_event(this, true); // send new config event
                  ak0991x_send_cal_event(this, (state->mag_info.cur_cfg.num == 1));    // send new cal event
                }
              }
              
              state->has_sync_ts_anchor = false;
 
              if(ak0991x_dae_if_available(this) && (state->config_step != AK0991X_CONFIG_IDLE))
              {
                state->reg_event_for_dae_poll_sync = true;
                ak0991x_dae_if_start_streaming(this);
                state->config_step = AK0991X_CONFIG_UPDATING_HW;
              }
            //}
          }
        }
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        // ignore
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid timer event id=%d", event->message_id);
      }
      if(NULL != state->timer_data_stream)
      {
        event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
      }
    }
  }


  //Handle device_mode stream events
  if( ak0991x_handle_device_mode_stream(this) == SNS_RC_SUCCESS)
  {
    // report
    ak0991x_send_cal_event(this, true);    // send new cal event
  }

  // Handle timer data stream for S4S
  ak0991x_s4s_handle_timer_data_stream(this);

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);
  return rv;
}

/** Public Data Definitions. */

sns_sensor_instance_api ak0991x_sensor_instance_api = {
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &ak0991x_inst_init,
  .deinit            = &ak0991x_inst_deinit,
  .set_client_config = &ak0991x_inst_set_client_config,
  .notify_event      = &ak0991x_inst_notify_event
};
