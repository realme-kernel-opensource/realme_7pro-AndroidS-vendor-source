/**
 * @file sns_lps22hx_sensor_instance.c
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
#include "sns_types.h"

#include "sns_lps22hx_hal.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"

static void send_com_self_test_result(sns_sensor_instance *const instance, bool test_passed)
{

  lps22hx_instance_state *state =
     (lps22hx_instance_state*)instance->state->state;

  sns_physical_sensor_test_event physical_sensor_test_event;
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_sensor_uid *suid_current = NULL;

  //update suid
  if(state->self_test_info.sensor == LPS22HX_PRESS)
  {
    suid_current = &state->press_info.suid;
  }
  else if(state->self_test_info.sensor == LPS22HX_SENSOR_TEMP)
  {
    suid_current = &state->temp_info.suid;
  }

  DBG_INST_PRINT(instance, MED, __FILENAME__, __LINE__,
        "Self Test(com) Result=%d", test_passed);
  DBG_INST_PRINT(instance, HIGH, __FILENAME__, __LINE__, "Sending Self Test event");

  physical_sensor_test_event.test_passed = test_passed;
  physical_sensor_test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
  physical_sensor_test_event.test_data.funcs.encode = &pb_encode_string_cb;
  physical_sensor_test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &physical_sensor_test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                suid_current);
}

void lps22hx_inst_com_self_test(sns_sensor_instance *const this)
{
  lps22hx_instance_state *state = (lps22hx_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t buffer = 0;
  bool who_am_i_success = false;

  rv = lps22hx_get_who_am_i(state->scp_service,
                            state->com_port_info.port_handle,
                            &buffer);

  if(rv == SNS_RC_SUCCESS
     &&
     buffer == LPS22HX_WHOAMI_VALUE)
  {
    who_am_i_success = true;
  }

  //Send result
  send_com_self_test_result(this, who_am_i_success);
  state->self_test_enabled = false;
}

static void inst_cleanup(lps22hx_instance_state *state, sns_stream_service *stream_mgr)
{
#if !(LPS22HX_POLLING)
  if(NULL != state->interrupt_data_stream)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);
    state->interrupt_data_stream = NULL;
  }
#endif
  if(NULL != state->async_com_port_data_stream)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->async_com_port_data_stream);
    state->async_com_port_data_stream = NULL;
  }
  if(NULL != state->timer_data_stream)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
    state->timer_data_stream = NULL;
  }
  if(NULL != state->scp_service)
  {
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
    state->scp_service = NULL;
  }
}

/** See sns_sensor_instance_api::init */
sns_rc lps22hx_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)this->state->state;
  lps22hx_state *sensor_state =
    (lps22hx_state*)sstate->state;
  float data[1];
  float temp_data[1];

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  state->diag_service =  (sns_diag_service*)
    service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
  DBG_INST_PRINT(this, MED, __FILENAME__, __LINE__,"inst_init");
#if !(LPS22HX_POLLING)
  // Setup stream connections with dependent Sensors
  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
      this,
      sensor_state->irq_suid,
      &state->interrupt_data_stream);
#endif
  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
      this,
      sensor_state->acp_suid,
      &state->async_com_port_data_stream);

  state->timer_suid = sensor_state->timer_suid;
  state->press_info.current_odr = LPS22HX_ODR_0;
  state->temp_info.current_odr = LPS22HX_ODR_0;
  state->press_info.sample_period = 0;
  state->press_info.offset = sensor_state->fac_cal_bias[0];

  state->encoded_press_event_len = pb_get_encoded_size_sensor_stream_event(data, LPS22HX_PRESS_NUM_AXES);
  state->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, LPS22HX_TEMP_NUM_AXES);

  /**----------- Copy all Sensor UIDs in instance state -------------*/
  sns_memscpy(&state->press_info.suid,
              sizeof(state->press_info.suid),
              &((sns_sensor_uid)PRESS_SUID),
              sizeof(state->press_info.suid));
  sns_memscpy(&state->temp_info.suid,
              sizeof(state->temp_info.suid),
              &((sns_sensor_uid)PRESS_SENSOR_TEMP_SUID),
              sizeof(state->temp_info.suid));
  /** Initialize COM port to be used by the Instance */
  sns_memscpy(&state->com_port_info.com_config,
      sizeof(state->com_port_info.com_config),
      &sensor_state->com_port_info.com_config,
      sizeof(sensor_state->com_port_info.com_config));

  state->scp_service = (sns_sync_com_port_service*)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
      &state->com_port_info.port_handle);

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  if(NULL == state->async_com_port_data_stream ||
#if !(LPS22HX_POLLING)
      NULL == state->interrupt_data_stream ||
#endif
      NULL == state->com_port_info.port_handle)
  {
    inst_cleanup(state, stream_mgr);
    return SNS_RC_FAILED;
  }

#if !(LPS22HX_POLLING)
  /** Initialize IRQ info to be used by the Instance */
  state->irq_info.irq_config = sensor_state->irq_config;
  {
    sns_data_stream* data_stream = state->interrupt_data_stream;
    uint8_t buffer[20];
    sns_request irq_req =
    {
      .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
      .request    = buffer
    };

    sns_memset(buffer, 0, sizeof(buffer));
    irq_req.request_len = pb_encode_request(buffer,
        sizeof(buffer),
        &state->irq_info.irq_config,
        sns_interrupt_req_fields,
        NULL);
    if(irq_req.request_len > 0)
    {
      data_stream->api->send_request(data_stream, &irq_req);
    }
  }
#endif
  /** Configure the Async Com Port */
  /* TBD Async com port should not be needed for LPS22HX (unless we start using FIFO?)
     uint8_t pb_encode_buffer[100];
     uint32_t enc_len;
     sns_async_com_port_config async_com_port_config;
     async_com_port_config.bus_instance = sensor_state->com_port_info.com_config.bus_instance;
#if LPS22HX_USE_I2C
async_com_port_config.bus_type = SNS_ASYNC_COM_PORT_BUS_TYPE_I2C;
#else
async_com_port_config.bus_type = SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
#endif
async_com_port_config.max_bus_speed_kHz = sensor_state->com_port_info.com_config.max_bus_speed_KHz;
async_com_port_config.min_bus_speed_kHz = sensor_state->com_port_info.com_config.min_bus_speed_KHz;
async_com_port_config.reg_addr_type = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
async_com_port_config.slave_control = sensor_state->com_port_info.com_config.slave_control;
enc_len = pb_encode_request(pb_encode_buffer, 100, &async_com_port_config,
sns_async_com_port_config_fields, NULL);

sns_request async_com_port_request =
(sns_request)
{
.message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
.request_len = enc_len,
.request = &pb_encode_buffer
};
state->async_com_port_data_stream->api->send_request(
state->async_com_port_data_stream, &async_com_port_request);
*/



  /** Determine size of sns_diag_sensor_state_raw as defined in
   **  sns_diag.proto
   **  sns_diag_sensor_state_raw is a repeated array of samples of
   **  type sns_diag_batch sample. The following determines the
   **  size of sns_diag_sensor_state_raw with a single batch
   **  sample */
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[LPS22HX_PRESS_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = LPS22HX_PRESS_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  if(pb_encode_tag(&stream, PB_WT_STRING,
        sns_diag_sensor_state_raw_sample_tag))
  {
    if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
          &batch_sample))
    {
      state->log_raw_encoded_size = stream.bytes_written;
    }
  }

  DBG_INST_PRINT(this, MED, __FILENAME__, __LINE__, "inst_init exit");
  return SNS_RC_SUCCESS;
}

sns_rc lps22hx_inst_deinit(sns_sensor_instance *const this)
{
  lps22hx_instance_state *state =
    (lps22hx_instance_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr =
    (sns_stream_service*)service_mgr->get_service(service_mgr,
        SNS_STREAM_SERVICE);

  DBG_INST_PRINT_EX(this, MED, __FILENAME__, __LINE__,"inst_deinit");

  inst_cleanup(state, stream_mgr);

  return SNS_RC_SUCCESS;
}

void lps22hx_set_client_test_config(sns_sensor_instance *this)
{
  lps22hx_instance_state *state = (lps22hx_instance_state*)this->state->state;

  // 1. Extract test type from client_request.
  // 2. Configure sensor HW for test type.
  // 3. send_request() for Timer Sensor in case test needs polling/waits.
  // 4. Factory test is TBD.
  if(state->press_info.self_test_is_successful)
  {
    state->self_test_info.sensor = LPS22HX_PRESS;
    state->press_info.self_test_is_successful = false;
  }
  else if(state->temp_info.self_test_is_successful)
  {
    state->self_test_info.sensor = LPS22HX_SENSOR_TEMP;
    state->temp_info.self_test_is_successful = false;
  }

  DBG_INST_PRINT(this, HIGH, __FILENAME__, __LINE__,
      "Self test start, test_type=%d sensor=%d",
      state->self_test_info.test_type, state->self_test_info.sensor);
  //start HW self test
  if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
  {
    DBG_INST_PRINT(this, HIGH, __FILENAME__, __LINE__,
                 "SNS_PHYSICAL_SENSOR_TEST_TYPE_COM test requested, test_ype=%d", state->self_test_info.test_type);
    lps22hx_inst_com_self_test(this);
  }
  else
  {
    DBG_INST_PRINT(this, HIGH, __FILENAME__, __LINE__,
                 "Unsupported test, test_ype=%d", state->self_test_info.test_type);
  }
}
