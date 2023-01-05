/**
 * @file sns_lps22hx_sensor_island.c
 *
 * Common implementation for LPS22HX Sensors.
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

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_island_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"

#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_hal.h"
#include "sns_lps22hx_sensor_instance.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"
#include "sns_attribute_util.h"

/** Forward Declaration of lps22hx pressure sensor API */
sns_sensor_api lps22hx_press_sensor_api;

/** Forward Declaration of lps22hx temprature sensor API */
sns_sensor_api lps22hx_sensor_temp_sensor_api;

const lps22hx_sensors lps22hx_supported_sensors[ MAX_SUPPORTED_SENSORS ] = {
  {LPS22HX_PRESS, &lps22hx_press_sensor_api, &lps22hx_sensor_instance_api},
  {LPS22HX_SENSOR_TEMP, &lps22hx_sensor_temp_sensor_api, &lps22hx_sensor_instance_api},
};

static void lps22hx_exit_island(sns_sensor *const this)
{
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_island_exit(island_svc, this);
}

/**
 * Returns decoded request message for type
 * sns_physical_sensor_test_config_fields.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool lps22hx_get_decoded_self_test_request(sns_sensor const *this, sns_request const *in_request,
                                            sns_std_request *decoded_request,
                                            sns_physical_sensor_test_config *decoded_payload)
{

  pb_istream_t stream;
  lps22hx_state *state = (lps22hx_state *) this->state->state;
  sns_diag_service* diag = state->diag_service;

  pb_simple_cb_arg arg =
      { .decoded_struct = decoded_payload,
        .fields = sns_physical_sensor_test_config_fields };
  decoded_request->payload = (struct pb_callback_s)
      { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };

  stream = pb_istream_from_buffer(in_request->request,
                                  in_request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
        " self test decode error");
    return false;
  }
  return true;
}

static void lps22hx_get_sensor_test_config(sns_sensor *const this, sns_sensor_instance *instance)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  sns_diag_service* diag = inst_state->diag_service;
  sns_sensor_uid suid;
  sns_request const *request;
  DBG_PRINT_EX(diag, this, LOW, __FILENAME__, __LINE__,
      "Enter lps22hx_get_sensor_test_config");

  if(state->sensor == LPS22HX_PRESS) {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)PRESS_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->press_info.suid,
        sizeof(inst_state->press_info.suid),
        &suid,
        sizeof(suid));
  } else if (state->sensor == LPS22HX_SENSOR_TEMP) {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)PRESS_SENSOR_TEMP_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->temp_info.suid,
        sizeof(inst_state->temp_info.suid),
        &suid,
        sizeof(suid));
  }
  else {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
        "config:Unknown %d", state->sensor);
  }
  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_physical_sensor_test_config decoded_payload_phy = sns_physical_sensor_test_config_init_default;
    if(request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
	{
      if(lps22hx_get_decoded_self_test_request(this, request, &decoded_request, &decoded_payload_phy)) //check for self test
      {
        DBG_PRINT(diag,this, HIGH,__FILENAME__, __LINE__,
          "lps22hx_get_sensor_config: Physical sensor self test: %d", decoded_payload_phy.test_type);

        // Only COM test is supported.
        if(decoded_payload_phy.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
        {
          inst_state->self_test_info.test_type = decoded_payload_phy.test_type;
          if(state->sensor == LPS22HX_PRESS)
          {
            inst_state->self_test_enabled = true;
            inst_state->press_info.self_test_is_successful = true;
          }
          else if(state->sensor == LPS22HX_SENSOR_TEMP)
          {
            inst_state->self_test_enabled = true;
            inst_state->temp_info.self_test_is_successful = true;
          }
        }
      }
    }
  }
}
static sns_rc lps22hx_process_timer_events(sns_sensor *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_data_stream *stream = state->timer_stream;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return rv;
  }

  for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
  {
    sns_sensor_event *event = stream->api->peek_input(stream);
    sns_diag_service *diag = state->diag_service;
    sns_timer_sensor_event timer_event;
    pb_istream_t pbstream;

    if(SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT != event->message_id)
    {
      continue; /* not interested in other events */
    }

    pbstream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
    if(!pb_decode(&pbstream, sns_timer_sensor_event_fields, &timer_event))
    {
      DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__, "pb_decode err");
      continue;
    }

    DBG_PRINT_EX(diag, this, HIGH, __FILENAME__, __LINE__,
              "Timer fired: sensor=%d req_timeout=%u timeout=%u power_rail=%d",
              state->sensor, timer_event.requested_timeout_time, timer_event.timeout_time, state->power_rail_pend_state);

    if(state->power_rail_pend_state == LPS22HX_POWER_RAIL_PENDING_INIT)
    {
      /** Initial HW discovery is OK to run in normal mode. */
      lps22hx_exit_island(this);
      lps22hx_discover_hw(this);
      if(!state->hw_is_present)
      {
        rv = SNS_RC_INVALID_LIBRARY_STATE;
        DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__, "HW absent");
      }
      state->power_rail_pend_state = LPS22HX_POWER_RAIL_PENDING_NONE;
    }
    else if(state->power_rail_pend_state == LPS22HX_POWER_RAIL_PENDING_SET_CLIENT_REQ)
    {
      sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
      if(NULL != instance)
      {
        lps22hx_instance_state *inst_state =  (lps22hx_instance_state*) instance->state->state;
        DBG_PRINT_EX(diag, this, LOW, __FILENAME__, __LINE__,
            "inst available: reval config");
        inst_state->instance_is_ready_to_configure = true;
        lps22hx_get_sensor_test_config(this, instance);
        if(inst_state->self_test_info.test_type !=SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
          lps22hx_reval_instance_config(this, instance, state->sensor);
      } else {
        DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
            "inst not available");
      }
      state->power_rail_pend_state = LPS22HX_POWER_RAIL_PENDING_NONE;
    }
  }

  if(state->power_rail_pend_state == LPS22HX_POWER_RAIL_PENDING_NONE)
  {
    sns_sensor_util_remove_sensor_stream(this, &state->timer_stream);
  }
  return rv;
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const* lps22hx_sensor_temp_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = PRESS_SENSOR_TEMP_SUID;
  return &sensor_uid;
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const* lps22hx_press_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = PRESS_SUID;
  return &sensor_uid;
}

/* See sns_sensor::get_sensor_uid */
sns_sensor_uid const* lps22hx_get_sensor_uid(sns_sensor const *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  return &state->my_suid;
}

static void lps22hx_start_power_rail_timer(sns_sensor *const this,
    sns_time timeout_ticks,
    lps22hx_power_rail_pending_state pwr_rail_pend_state)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;

  DBG_PRINT_EX(diag, this, LOW, __FILENAME__, __LINE__,
      "start_power_rail_timer IN");

  if(NULL == state->timer_stream)
  {
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid,
                                          &state->timer_stream);
  }

  if(NULL != state->timer_stream)
  {
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t req_len;
    uint8_t buffer[20];
    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout_ticks;

    req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                                sns_timer_sensor_config_fields, NULL);
    if(req_len > 0)
    {
      sns_request timer_req =
        {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
           .request = buffer, .request_len = req_len};
      state->timer_stream->api->send_request(state->timer_stream, &timer_req);
      state->power_rail_pend_state = pwr_rail_pend_state;
    }
    else
    {
      DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
                "timer req encode err");
    }
  }
  else
  {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
              "create timer stream err");
  }
}

/** See sns_lps22hx_sensor.h*/
sns_rc lps22hx_sensor_notify_event(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  sns_diag_service* diag = state->diag_service;
  DBG_PRINT_EX(diag, this, HIGH, __FILENAME__, __LINE__,
      "Enter lps22hx_sensor_notify_event");

  if((NULL != state->fw_stream &&
      0 != state->fw_stream->api->get_input_cnt(state->fw_stream)) ||
     (NULL != state->reg_data_stream &&
      0 != state->reg_data_stream->api->get_input_cnt(state->reg_data_stream)))
  {
    lps22hx_exit_island(this);
    lps22hx_process_suid_events(this);
    rv = lps22hx_process_registry_events(this);
  }

  if(rv == SNS_RC_SUCCESS)
  {
    rv = lps22hx_process_timer_events(this);
  }

  if((rv == SNS_RC_SUCCESS) && (LPS22HX_PRESS == state->sensor))
  {
    if(!state->hw_is_present &&
       NULL != state->pwr_rail_service &&
       NULL != state->timer_stream &&
       state->power_rail_pend_state == LPS22HX_POWER_RAIL_PENDING_NONE)
    {
      sns_time timeticks;

      state->rail_config.rail_vote = SNS_RAIL_ON_LPM;
      state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                               this,
                                                               &state->rail_config,
                                                               &timeticks); /* ignored */
      timeticks = sns_convert_ns_to_ticks(LPS22HX_OFF_TO_IDLE_MS * 1000 * 1000);
      lps22hx_start_power_rail_timer(this,
                                     timeticks,
                                     LPS22HX_POWER_RAIL_PENDING_INIT);
    }

    if(!state->available && state->hw_is_present && state->outstanding_reg_requests == 0)
    {
      lps22hx_exit_island(this);
      lps22hx_update_siblings(this);
    }

  }

  return rv;
}

/**
 * Returns decoded request message for type
 * sns_std_sensor_config.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool lps22hx_get_decoded_request(sns_sensor const *this, sns_request const *in_request,
    sns_std_request *decoded_request,
    sns_std_sensor_config *decoded_payload)
{

  lps22hx_state *state = (lps22hx_state *) this->state->state;
  sns_diag_service* diag = state->diag_service;
  pb_istream_t stream;
  pb_simple_cb_arg arg =
  { .decoded_struct = decoded_payload,
    .fields = sns_std_sensor_config_fields };
  decoded_request->payload = (struct pb_callback_s)
  { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(in_request->request,
      in_request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
        "decode error");
    return false;
  }
  return true;
}

static void lps22hx_get_sensor_config(sns_sensor *this,
    sns_sensor_instance *instance,
    lps22hx_sensor_type sensor_type,
    float *chosen_sample_rate,
    bool *sensor_client_present)
{
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  sns_diag_service* diag = inst_state->diag_service;
  sns_sensor_uid suid;
  sns_request const *request;
  DBG_PRINT_EX(diag, this, LOW, __FILENAME__, __LINE__,
      "Enter lps22hx_get_sensor_config");
  if(sensor_type == LPS22HX_PRESS) {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)PRESS_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->press_info.suid,
        sizeof(inst_state->press_info.suid),
        &suid,
        sizeof(suid));
  } else if (sensor_type == LPS22HX_SENSOR_TEMP) {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)PRESS_SENSOR_TEMP_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->temp_info.suid,
        sizeof(inst_state->temp_info.suid),
        &suid,
        sizeof(suid));
  }
  else {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
        "config:Unknown %d", sensor_type);
  }

  *chosen_sample_rate = 0;
  *sensor_client_present = false;

  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload = {0};
    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      if(lps22hx_get_decoded_request(this, request, &decoded_request, &decoded_payload))
      {
         *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                       decoded_payload.sample_rate);
         /* FIFO not supported, so ignoring has_batch_period, batch_period and report rate*/
         *sensor_client_present = true;
       }
    }
  }

}

static void lps22hx_set_inst_config(sns_sensor *this,
    sns_sensor_instance *instance,
    lps22hx_sensor_type sensor,
    float chosen_sample_rate)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_request req_config;
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  sns_diag_service* diag = state->diag_service;
  DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
      "setting inst configuration");
  if(((sensor == LPS22HX_PRESS) ||
      (sensor == LPS22HX_SENSOR_TEMP)) &&
     (inst_state->self_test_enabled))
  {
    sns_lps22hx_self_test_req client_config;
    client_config.test_type = inst_state->self_test_info.test_type;
    req_config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
    req_config.request_len = sizeof(sns_lps22hx_self_test_req);
    req_config.request = &client_config;
    DBG_PRINT(diag,this, HIGH, __FILENAME__, __LINE__,
                             "setting self test configuration::  ");
    this->instance_api->set_client_config(instance, &req_config);
  }
  else if((sensor == LPS22HX_PRESS) ||
          (sensor == LPS22HX_SENSOR_TEMP))
  {
    sns_lps22hx_config_req client_config;
    client_config.sample_rate = chosen_sample_rate;
    client_config.sensor_type = sensor;
    req_config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
    req_config.request_len = sizeof(sns_lps22hx_config_req);
    req_config.request = &client_config;

    DBG_PRINT(diag, this, MED, __FILENAME__, __LINE__,
      "setting stream configuration::(*1000) %d , sensor type = %d",
      (int)(client_config.sample_rate*1000), client_config.sensor_type);

    this->instance_api->set_client_config(instance, &req_config);
  }
  else
    return;
}

static void lps22hx_turn_rails_off(sns_sensor *this)
{
  sns_sensor *sensor;

  for(sensor = this->cb->get_library_sensor(this, true);
      NULL != sensor;
      sensor = this->cb->get_library_sensor(this, false))
  {
    lps22hx_state *sensor_state = (lps22hx_state*)sensor->state->state;
    if(sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
    {
      DBG_PRINT(sensor_state->diag_service, this, MED, __FILENAME__, __LINE__, 
                "turning rails off");
      sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
      sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
                                                                      sensor,
                                                                      &sensor_state->rail_config,
                                                                      NULL);
    }
  }
}
void lps22hx_update_sensor_config(sns_sensor *this,
    sns_sensor_instance *instance,
    lps22hx_sensor_type sensor,
    bool client_present)
{
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  UNUSED_VAR(this);

  if(client_present) {
    inst_state->enabled_sensors |= sensor;
    inst_state->publish_sensors |= sensor;

  } else {
    inst_state->enabled_sensors &= ~sensor;
    inst_state->publish_sensors &= ~sensor;
  }


}
void lps22hx_reval_instance_config(sns_sensor *this,
    sns_sensor_instance *instance,
    lps22hx_sensor_type sensor_type)
{
  /**
   * Get best pressure Config.
   * Get best sensor temp Config.
   * Decide best Instance Config based on above outputs.
   */

  float chosen_sample_rate = 0.0f;
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;
  uint8_t sensor_count = ARR_SIZE(lps22hx_supported_sensors);
  struct {
    lps22hx_sensor_type sensor;
    float sample_rate;
    bool client_present;
    bool xtra_flags; //depends on sensor type
  } sensor_info[sensor_count];
  int i = 0;
  for(; i< sensor_count ; i++) {
    sensor_info[i].client_present = false;
    sensor_info[i].xtra_flags = false;
    sensor_info[i].sensor = lps22hx_supported_sensors[i].sensor;
    lps22hx_get_sensor_config(this, instance, sensor_info[i].sensor,
        &sensor_info[i].sample_rate,
        &sensor_info[i].client_present);

    chosen_sample_rate = SNS_MAX(chosen_sample_rate, sensor_info[i].sample_rate);
    lps22hx_update_sensor_config(this, instance, sensor_info[i].sensor, sensor_info[i].client_present);
    DBG_PRINT(diag, this, LOW, __FILENAME__, __LINE__,
        "config: %d %d(*1000)", sensor_info[i].sensor, (int)(sensor_info[i].sample_rate*1000));
  }
    //pressure is default enabled for temp
  if(inst_state->enabled_sensors & LPS22HX_SENSOR_TEMP)
    inst_state->enabled_sensors |= LPS22HX_PRESS;

  lps22hx_set_inst_config(this,
      instance,
      sensor_type,
      chosen_sample_rate);

  if(!inst_state->enabled_sensors)
  {
    lps22hx_turn_rails_off(this);
    inst_state->instance_is_ready_to_configure = false;
  }
}
/**
sns_lps22hx_handle_self_test handle self test request
*/
void sns_lps22hx_handle_self_test(sns_sensor *const this,
     sns_sensor_instance *instance,
     struct sns_request const *new_request)
{
  sns_std_request decoded_request;
  sns_physical_sensor_test_config decoded_payload_phy = sns_physical_sensor_test_config_init_default;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  lps22hx_instance_state *inst_state =
    (lps22hx_instance_state*)instance->state->state;
  sns_diag_service* diag = state->diag_service;
  
  if(lps22hx_get_decoded_self_test_request(this, new_request, &decoded_request, &decoded_payload_phy)) //check for self test
  {
    DBG_PRINT(diag,this, HIGH,__FILENAME__, __LINE__,
            "lps22hx_get_sensor_config: Physical sensor self test: %d", decoded_payload_phy.test_type);

    // Only COM test is supported.
    if(decoded_payload_phy.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      inst_state->self_test_info.test_type = decoded_payload_phy.test_type;
      if(state->sensor == LPS22HX_PRESS)
      {
        inst_state->self_test_enabled = true;
        inst_state->press_info.self_test_is_successful = true;
      }
      else if(state->sensor == LPS22HX_SENSOR_TEMP)
      {
        inst_state->self_test_enabled = true;
        inst_state->temp_info.self_test_is_successful = true;
      }
    }
  }
}

static void lps22hx_check_instance(sns_sensor *const this)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  if(NULL != instance &&
     NULL == instance->cb->get_client_request(instance, 
                                              &(sns_sensor_uid)PRESS_SUID, true) &&
     NULL == instance->cb->get_client_request(instance, 
                                              &(sns_sensor_uid)PRESS_SENSOR_TEMP_SUID, true))
  {
    lps22hx_state *state = (lps22hx_state*)this->state->state;
    state->power_rail_pend_state = LPS22HX_POWER_RAIL_PENDING_NONE;
    sns_sensor_util_remove_sensor_stream(this, &state->timer_stream);
    lps22hx_turn_rails_off(this);
    this->cb->remove_instance(instance);
  }
}


/** See sns_lps22hx_sensor.h */
sns_sensor_instance* lps22hx_set_client_request(sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  sns_time on_timestamp;
  sns_time delta;
  bool reval_config = false;
  sns_diag_service* diag = state->diag_service;

  DBG_PRINT(diag, this, HIGH, __FILENAME__, __LINE__,
            "set_client_request: sensor=%u msg=%d/%d remove=%u inst=%x", 
            state->sensor, exist_request ? exist_request->message_id : -1,
            new_request ? new_request->message_id : -1, remove, instance);

  if(remove)
  {
    if(NULL == instance) {
      DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
          "Inst NULL!");
      return instance;
    }
    lps22hx_instance_state *inst_state =
        (lps22hx_instance_state*)instance->state->state;
    instance->cb->remove_client_request(instance, exist_request);

    if(lps22hx_set_oem_client_request(this, exist_request))
    {
      /* Assumption: The FW will call deinit() on the instance before destroying it.
         Putting all HW resources (sensor HW, COM port, power rail)in
         low power state happens in Instance deinit().*/
      if(NULL != exist_request &&
         exist_request->message_id != SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
      {
        lps22hx_reval_instance_config(this, instance, state->sensor);
      }
      else
      {
        lps22hx_exit_island(this);
      }
      if(!inst_state->enabled_sensors)
      {
        lps22hx_turn_rails_off(this);
        inst_state->instance_is_ready_to_configure = false;
      }
    }
  }
  else
  {
    // 1. If new request then:
    //     a. Power ON rails.
    //     b. Power ON COM port - Instance must handle COM port power.
    //     c. Create new instance.
    //     d. Re-evaluate existing requests and choose appropriate instance config.
    //     e. set_client_config for this instance.
    //     f. Add new_request to list of requests handled by the Instance.
    //     g. Power OFF COM port if not needed- Instance must handle COM port power.
    //     h. Return the Instance.
    // 2. If there is an Instance already present:
    //     a. Add new_request to list of requests handled by the Instance.
    //     b. Remove exist_request from list of requests handled by the Instance.
    //     c. Re-evaluate existing requests and choose appropriate Instance config.
    //     d. set_client_config for the Instance if not the same as current config.
    //     e. publish the updated config.
    //     f. Return the Instance.
    // 3.  If "flush" request:
    //     a. Perform flush on the instance.
    //     b. Return NULL.

    if(NULL == instance)
    {
      sns_time off2idle = sns_convert_ns_to_ticks(LPS22HX_OFF_TO_IDLE_MS*1000*1000);
      state->rail_config.rail_vote = SNS_RAIL_ON_LPM;
      state->pwr_rail_service->api->sns_vote_power_rail_update(
          state->pwr_rail_service,
          this,
          &state->rail_config,
          &on_timestamp);

      delta = sns_get_system_time() - on_timestamp;

      DBG_PRINT(diag, this, LOW, __FILENAME__, __LINE__,
          "set Power rails delta=%u o2i=%u", (uint32_t)delta, (uint32_t)off2idle);

      // Use on_timestamp to determine correct Timer value.
      if(delta < off2idle)
      {
        lps22hx_start_power_rail_timer(this,
            off2idle - delta,
            LPS22HX_POWER_RAIL_PENDING_SET_CLIENT_REQ);
      } else {
        // rail is already ON
        reval_config = true;

      }
      DBG_PRINT_EX(diag, this, LOW, __FILENAME__, __LINE__,
          "Call create_inst()");
      /** create_instance() calls init() for the Sensor Instance */
      instance = this->cb->create_instance(this,
          sizeof(lps22hx_instance_state));

      /* If rail is already ON then flag instance OK to configure */
      if(reval_config)
      {
        lps22hx_instance_state *inst_state =
          (lps22hx_instance_state*)instance->state->state;

        inst_state->instance_is_ready_to_configure = true;
      }

    }
    else
    {
      if(NULL != exist_request
        && NULL != new_request
        && new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
      {
        sns_sensor_util_send_flush_event(&state->my_suid, instance);
        return instance;
      }
      else if(NULL != new_request &&
              new_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
      {
        sns_lps22hx_handle_self_test(this, instance, new_request);
      }
      else
      {
        reval_config = true;

        /** An existing client is changing request*/
        if((NULL != exist_request) && (NULL != new_request))
        {
          instance->cb->remove_client_request(instance, exist_request);
        }

        /** A new client sent new_request*/
        else if(NULL != new_request)
        {
          // No-op. new_request will be added to requests list below.
        }
      }
    }
    /** Add the new request to list of client_requests.*/
    if(NULL != instance)
    {
      lps22hx_instance_state *inst_state =
        (lps22hx_instance_state*)instance->state->state;
      if(NULL != new_request)
      {
        instance->cb->add_client_request(instance, new_request);
      }
      if(new_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
      {
        sns_lps22hx_handle_self_test(this, instance, new_request);
        lps22hx_reval_instance_config(this, instance, state->sensor);
      }
      if(lps22hx_set_oem_client_request_msgid(this, new_request))
      {
        if(reval_config && inst_state->instance_is_ready_to_configure)
        {
          lps22hx_reval_instance_config(this, instance, state->sensor);
        }
      }
    }
  }

  lps22hx_check_instance(this);

  DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
      "Exit lps22hx_set_client_request");
  return instance;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/
sns_sensor_api lps22hx_press_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lps22hx_press_init,
  .deinit             = &lps22hx_press_deinit,
  .get_sensor_uid     = &lps22hx_press_get_sensor_uid,
  .set_client_request = &lps22hx_set_client_request,
  .notify_event       = &lps22hx_sensor_notify_event,
};
sns_sensor_api lps22hx_sensor_temp_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lps22hx_sensor_temp_init,
  .deinit             = &lps22hx_sensor_temp_deinit,
  .get_sensor_uid     = &lps22hx_sensor_temp_get_sensor_uid,
  .set_client_request = &lps22hx_set_client_request,
  .notify_event       = &lps22hx_sensor_notify_event,
};
