/**
 * @file sns_lps22hx_sensor_instance_island.c
 *
 * LPS22HX sensor instance implementation.
 *
 * Copyright (c) 2019, STMicroelectronics.
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
 **/

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_lps22hx_hal.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"
#include "sns_sensor_util.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_island_service.h"
#include "sns_event_service.h"
#include "sns_std_type.pb.h"

const odr_reg_map lps22hx_odr_map[] =
{
  {
    .odr = LPS22HX_ODR_0,
    .odr_reg_value = LPS22HX_SENSOR_ODR_OFF,
    .press_discard_samples = 0,
  },
#if !LPS22HX_ENABLE_LPF
  {
    .odr = LPS22HX_ODR_1,
    .odr_reg_value = LPS22HX_SENSOR_ODR1,
    .press_discard_samples = NUM_SAMPLE_TO_DISCARD,
  },
#endif
  {
    .odr = LPS22HX_ODR_10,
    .odr_reg_value = LPS22HX_SENSOR_ODR10,
    .press_discard_samples = NUM_SAMPLE_TO_DISCARD,
  },
  {
    .odr = LPS22HX_ODR_25,
    .odr_reg_value = LPS22HX_SENSOR_ODR25,
    .press_discard_samples = NUM_SAMPLE_TO_DISCARD,
  },
};

/**
Function to send error event
*/
static void lps22hx_report_error(sns_sensor_instance *this, sns_sensor_uid const *suid)
{
  sns_service_manager *mgr = this->cb->get_service_manager(this);
  sns_event_service *ev_svc = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
  sns_sensor_event *event = ev_svc->api->alloc_event(ev_svc, this, 0);
  if(NULL != event)
  {
    event->message_id = SNS_STD_MSGID_SNS_STD_ERROR_EVENT;
    event->event_len = 0;
    event->timestamp = sns_get_system_time();
    ev_svc->api->publish_event(ev_svc, this, event, suid);
  }
}

static sns_rc lps22hx_match_odr(float desired_sample_rate, float *chosen_sample_rate, uint8_t *index)
{
  uint8_t idx;
  if(LPS22HX_ODR_MAX < desired_sample_rate)
  {
    return SNS_RC_INVALID_VALUE;
  }

  for(idx = 0; idx < ARR_SIZE(lps22hx_odr_map); idx++)
  {
    if(desired_sample_rate <= lps22hx_odr_map[idx].odr)
    {
      break;
    }
  }

  if (idx >= ARR_SIZE(lps22hx_odr_map))
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  *chosen_sample_rate = lps22hx_odr_map[idx].odr;
  *index = idx;

  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc lps22hx_inst_notify_event(sns_sensor_instance *const this)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)this->state->state;
#if !(LPS22HX_POLLING)
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
#endif
  sns_sensor_event *event;

#if LPS22HX_LOG_DATA //Do not print log for every sample
  DBG_INST_PRINT(this, LOW, __FILENAME__, __LINE__,
      "lps22hx_inst_notify_event");
#endif
  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
      true);

#if !(LPS22HX_POLLING)
  // Handle interrupts
  if(NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
    while(NULL != event)
    {
      if (event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT)
      {
        state->irq_info.irq_ready = true;
      }
      else if(event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
          state->interrupt_timestamp = irq_event.timestamp;
          lps22hx_handle_interrupt_event(this);
        }
      }
      else
      {
        DBG_INST_PRINT(this, HIGH, __FILENAME__, __LINE__,
            "invalid event=%d", event->message_id);
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
  }
#endif

  /* TBD - Remove Async Com port code
  // Handle Async Com Port events
  if(NULL != state->async_com_port_data_stream)
  {
  event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);
  while(NULL != event)
  {
  if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
  {
  //TODO: Warning;
  DBG_INST_PRINT(this, ERROR, __FILENAME__, __LINE__,
  "Received ASCP error event id=%d",
  event->message_id);
  }
  else if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
  {
  pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
  sns_ascp_for_each_vector_do(&stream, lps22hx_process_fifo_data_buffer, (void *)this);
  }
  event = state->async_com_port_data_stream->api->get_next_input(state->async_com_port_data_stream);
  }
  }
  */
  if(NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);
    while(NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
          event->event_len);
      sns_timer_sensor_event timer_event;
      if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
        {
#if LPS22HX_LOG_DATA //Do not print log for every sample
          DBG_INST_PRINT(this, LOW, __FILENAME__, __LINE__,
              "Received timer event id=%d", event->message_id);
#endif
          if(((state->publish_sensors & LPS22HX_SENSOR_TEMP)
                &&
                state->temp_info.timer_is_active
                &&
                (state->temp_info.current_odr> 0))
              ||
              ((state->publish_sensors & LPS22HX_PRESS)
               &&
               state->press_info.timer_is_active
               &&
               (state->press_info.current_odr> 0)))
          {
            lps22hx_handle_sensor_sample(this, timer_event.timeout_time);
          }
          else
          {
            sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
            state->press_info.timer_is_active = false;
            state->temp_info.timer_is_active = false;
            break;
          }
        }
      }
      else
      {
      }
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
      false);

  return SNS_RC_SUCCESS;
}
static void lps22hx_inst_exit_island(sns_sensor_instance *this)
{
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
}
/** See sns_sensor_instance_api::set_client_config */
sns_rc lps22hx_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)this->state->state;
  state->client_req_id = client_request->message_id;
  float desired_sample_rate = 0;
  float chosen_sample_rate = 0;
  lps22hx_sensor_type sensor;
  bool reconfig = false;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t idx;

  DBG_INST_PRINT(this, LOW, __FILENAME__, __LINE__,
      "set client config =%d", client_request->message_id);
  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,true);

  // Reset the device if not streaming.
  if(state->press_info.current_odr == LPS22HX_ODR_0  && ((state->enabled_sensors & LPS22HX_SENSOR_TEMP) || (state->enabled_sensors & LPS22HX_PRESS)) )
  {
    lps22hx_reset_device(this, LPS22HX_PRESS | LPS22HX_SENSOR_TEMP);
  }
  if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
    // 1. Extract sample, report rates from client_request.
    // 2. Configure sensor HW.
    // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
    // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
    // 5. Save the current config information like type, sample_rate etc.
    if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      sns_lps22hx_config_req *payload =
        (sns_lps22hx_config_req*)client_request->request;
      desired_sample_rate = payload->sample_rate;
      sensor = payload->sensor_type;
    }
    else // TODO self-test
    {
    }

    DBG_INST_PRINT(this, LOW, __FILENAME__, __LINE__,
        "sr(*1000)=%d", (int)(desired_sample_rate*1000));
    rv = lps22hx_match_odr(desired_sample_rate, &chosen_sample_rate, &idx);
    if(rv != SNS_RC_SUCCESS)
    {
      DBG_INST_PRINT(this, ERROR, __FILENAME__, __LINE__, "Invalid ODR");
      lps22hx_report_error(this, &state->press_info.suid);
      return rv;
    }
    if(state->enabled_sensors) {
      if(state->press_info.current_odr != chosen_sample_rate) {
        rv = lps22hx_set_odr(this, idx);
        state->press_info.current_odr =  chosen_sample_rate;
        if(state->enabled_sensors & LPS22HX_SENSOR_TEMP)
          state->temp_info.current_odr =  chosen_sample_rate;
        else
          state->temp_info.current_odr =  LPS22HX_ODR_0;
        reconfig = true;
      }
    } else {
      if(state->press_info.current_odr != LPS22HX_ODR_0) {
        rv= lps22hx_set_odr(this, LPS22HX_ODR_0);
        state->press_info.current_odr =  LPS22HX_ODR_0;
        state->temp_info.current_odr =  LPS22HX_ODR_0;
        reconfig = true;
      }
    }
    lps22hx_send_config_event(this);
    if(reconfig)
    {

#if !(LPS22HX_POLLING)  // TODO: Temporarily enable polling mode by default as requested by customer. With registry support this hardcode will be gone.
      // Enable interrupt
      if(state->publish_sensors & (LPS22HX_PRESS | LPS22HX_SENSOR_TEMP))
      {
        rv = lps22hx_enable_intr(this);
      }
#else
      // Enable timer
      if(state->publish_sensors & (LPS22HX_PRESS | LPS22HX_SENSOR_TEMP))
      {
        lps22hx_set_polling_config(this);
      }
#endif
    }
  }
  else if(state->client_req_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    /** All self-tests are handled in normal mode. */
    lps22hx_inst_exit_island(this);
    lps22hx_set_client_test_config(this);
  }
  else
    lps22hx_inst_set_oem_client_config(this);

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,false);

  DBG_INST_PRINT(this, LOW, __FILENAME__, __LINE__,
      "ODR(P/T)=0x%x/0x%x", state->press_info.current_odr,state->temp_info.current_odr);
  return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */
sns_sensor_instance_api lps22hx_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &lps22hx_inst_init,
  .deinit            = &lps22hx_inst_deinit,
  .set_client_config = &lps22hx_inst_set_client_config,
  .notify_event      = &lps22hx_inst_notify_event
};
