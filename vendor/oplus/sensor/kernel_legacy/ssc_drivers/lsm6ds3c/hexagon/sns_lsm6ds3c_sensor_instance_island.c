/**
 * @file sns_lsm6ds3c_sensor_instance.c
 *
 * LSM6DS3C Accel virtual Sensor Instance implementation.
 *
 * Copyright (c) 2018-2019, STMicroelectronics.
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
 *
 **/

#include <math.h>
#include <float.h>

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_event_service.h"
#include "sns_island_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_lsm6ds3c_hal.h"
#include "sns_lsm6ds3c_sensor.h"
#include "sns_lsm6ds3c_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_sensor_util.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_cal.pb.h"
#include "sns_printf.h"
#ifdef SNS_AVOID_HANDLING_INTERRUPT_DURING_PC
#include "sns_psm.h"
#endif


/* accel_discard_samples = AN4987 value + 2
   gyro_discard_samples =  AN4987 value + 2 + LSM6DS3C_GYRO_ON_TIME_MS equivalent samples
*/
#define MAX_HEART_ATTACKS 6
const odr_reg_map lsm6ds3c_odr_map[] =
{
  {
    .odr = LSM6DS3C_ODR_0,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR_OFF,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR_OFF,
    .odr_coeff = 0,
    .accel_group_delay = 0,
    .gyro_group_delay = 0,
    .accel_discard_samples = 0,
    .gyro_discard_samples = 0
  },
  {
    .odr = LSM6DS3C_ODR_13,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR13,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR13,
    .odr_coeff = 512,
    .accel_group_delay = 40.00f,
    .gyro_group_delay = 80.00f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 3
  },
  {
    .odr = LSM6DS3C_ODR_26,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR26,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR26,
    .odr_coeff = 256,
    .accel_group_delay = 19.23f,
    .gyro_group_delay = 38.46f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DS3C_ODR_52,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR52,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR52,
    .odr_coeff = 128,
    .accel_group_delay = 9.62f,
    .gyro_group_delay = 19.23f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DS3C_ODR_104,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR104,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR104,
    .odr_coeff = 64,
    .accel_group_delay = 4.81f,
    .gyro_group_delay = 9.62f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DS3C_ODR_208,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR208,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR208,
    .odr_coeff = 32,
    .accel_group_delay = 2.40f,
    .gyro_group_delay = 5.00f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DS3C_ODR_416,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR416,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR416,
    .odr_coeff = 16,
    .accel_group_delay = 1.20f,
    .gyro_group_delay = 2.50f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DS3C_ODR_832,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR832,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR832,
    .odr_coeff = 8,
    .accel_group_delay = 0.60f,
    .gyro_group_delay = 1.25f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 40
  },
  {
    .odr = LSM6DS3C_ODR_1664,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR1664,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR1664,
    .odr_coeff = 4,
    .accel_group_delay = 0.30f,
    .gyro_group_delay = 0.67f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 136
  },
  {
    .odr = LSM6DS3C_ODR_3328,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR3328,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR1664,
    .odr_coeff = 2,
    .accel_group_delay = 0.15f,
    .gyro_group_delay = 0.38f,
    .accel_discard_samples = 6,
    .gyro_discard_samples = 271
  },
  {
    .odr = LSM6DS3C_ODR_6656,
    .accel_odr_reg_value = LSM6DS3C_ACCEL_ODR6656,
    .gyro_odr_reg_value = LSM6DS3C_GYRO_ODR1664,
    .odr_coeff = 1,
    .accel_group_delay = 0.08f,
    .gyro_group_delay = 0.22f,
    .accel_discard_samples = 15,
    .gyro_discard_samples = 541
  },
};

const uint32_t lsm6ds3c_odr_map_len = ARR_SIZE(lsm6ds3c_odr_map);

static void inst_exit_island(sns_sensor_instance *this)
{
#if !LSM6DS3C_ISLAND_DISABLED
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
#else
  UNUSED_VAR(this);
#endif
}
void lsm6ds3c_inst_create_timer(sns_sensor_instance *this,
    sns_data_stream** timer_data_stream,
    sns_timer_sensor_config* req_payload)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;

  if(!timer_data_stream || !req_payload) {
    DBG_INST_PRINTF(ERROR, this, "Invalid timer data stream or req_payload pointer passed");
    return;
  }
  size_t req_len;
  uint8_t buffer[50];
  req_len = pb_encode_request(buffer, sizeof(buffer), req_payload,
      sns_timer_sensor_config_fields, NULL);
  if(req_len > 0)
  {
    if(*timer_data_stream == NULL)
    {
      sns_service_manager *service_mgr = this->cb->get_service_manager(this);
      sns_stream_service *stream_mgr = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
          this,
          state->timer_suid,
          timer_data_stream);
      DBG_INST_PRINTF_EX(MED, this, "creating timer stream!");
    }
    if(*timer_data_stream == NULL)
    {
      SNS_INST_PRINTF(HIGH, this,
          "Error creating timer stream!");
      return;
    }
    sns_request timer_req =
    {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
      .request = buffer, .request_len = req_len  };
    (*timer_data_stream)->api->send_request(*timer_data_stream, &timer_req);
    LSM6DS3C_INST_DEBUG_TS(
      MED, this, "set timer start_time=%u timeout_period=%u",
      (uint32_t)req_payload->start_time, (uint32_t)req_payload->timeout_period);
  }
  else
  {
    SNS_INST_PRINTF(ERROR, this, "LSM timer req encode error");
  }
}

uint8_t lsm6ds3c_get_odr_rate_idx(float desired_sample_rate)
{
  uint8_t rate_idx = lsm6ds3c_odr_map_len;
  if(LSM6DS3C_ODR_832 >= desired_sample_rate)
  {
    for(uint8_t i = 0; i < lsm6ds3c_odr_map_len; i++)
    {
      if(desired_sample_rate <= lsm6ds3c_odr_map[i].odr)
      {
        rate_idx = i;
        break;
      }
    }
  }
  return rate_idx;
}

static sns_rc lsm6ds3c_validate_sensor_temp_odr(
  lsm6ds3c_instance_state *state,
  lsm6ds3c_instance_config* inst_cfg)
{
  sns_rc rc = SNS_RC_SUCCESS;
  lsm6ds3c_sensor_temp_info *sti = &state->sensor_temp_info;

  sti->desired_sampling_rate_hz = 0.0f;
  if(inst_cfg->temper_sample_rate <= LSM6DS3C_SENSOR_TEMP_ODR_1)
  {
    sti->desired_sampling_rate_hz = LSM6DS3C_SENSOR_TEMP_ODR_1;
  }
  else if(inst_cfg->temper_sample_rate > LSM6DS3C_SENSOR_TEMP_ODR_1 &&
          inst_cfg->temper_sample_rate <= LSM6DS3C_SENSOR_TEMP_ODR_5)
  {
    sti->desired_sampling_rate_hz = LSM6DS3C_SENSOR_TEMP_ODR_5;
  }
  else
  {
    sti->sampling_intvl = 0;
    rc = SNS_RC_NOT_SUPPORTED;
  }

  if (rc == SNS_RC_SUCCESS)
  {
    sti->report_rate_hz = SNS_MIN(inst_cfg->temper_report_rate,
                                  inst_cfg->temper_sample_rate);
    sti->sampling_intvl = sns_convert_ns_to_ticks(1000000000.0f /
                                                  sti->desired_sampling_rate_hz);
    if(sti->cur_sampling_rate_hz != sti->desired_sampling_rate_hz)
    {
      sti->new_config.timestamp     = sns_get_system_time();
      sti->new_config.sample_rate   = sti->desired_sampling_rate_hz;
    }
#if LSM6DS3C_DAE_ENABLED
    sti->new_config.dae_watermark = state->sensor_temp_info.desired_sampling_rate_hz /
                                    state->sensor_temp_info.report_rate_hz;
#endif
  }

  return rc;
}
void lsm6ds3c_restart_hb_timer(sns_sensor_instance *const this, bool reset)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_health *health = &state->health;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;

  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = health->heart_beat_timeout;
  lsm6ds3c_inst_create_timer(this, &state->timer_heart_beat_data_stream, &req_payload);

  health->expected_expiration = req_payload.start_time + health->heart_beat_timeout;
  if(reset)
  {
    health->heart_attack     = false;
    health->heart_attack_cnt = 0;
  }
  DBG_INST_PRINTF_EX(HIGH, this, "config: (i,w,r,s,md) %u, %u, %u, 0x%x, %u",
      state->current_conf.odr_idx,
      state->current_conf.wmk, state->current_conf.odr,
      state->current_conf.enabled_sensors, state->current_conf.md_enabled);

  DBG_INST_PRINTF_EX(HIGH, this,
      "last_irq, last_ts: %u, %u", (uint32_t)(state->fifo_info.interrupt_timestamp), (uint32_t)(state->fifo_info.last_timestamp));
}

static void lsm6ds3c_handle_fifo_interrupt(sns_sensor_instance *const this, sns_time irq_ts)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)this->state->state;
  //set this once entered in irq handle
  bool is_clear_needed = true;
  state->fifo_info.th_info.recheck_int = false;
  if(state->fifo_info.cur_wmk > 0
      && state->fifo_info.fifo_rate > LSM6DS3C_ACCEL_ODR_OFF)
  {
    LSM6DS3C_INST_DEBUG_TS(MED, this,
        "[INT] last_ts = %u irq_time = %u",(uint32_t)(state->fifo_info.last_timestamp), (uint32_t)(irq_ts));

    //check if this irq is valid, may belong to the recent flush request
    if(irq_ts > state->fifo_info.last_timestamp) {
      state->fifo_info.th_info.interrupt_fired = true;
      state->fifo_info.th_info.interrupt_ts = irq_ts;
      is_clear_needed = false;
      lsm6ds3c_read_fifo_data(this, irq_ts, false);
    } else {
      LSM6DS3C_INST_DEBUG_TS(MED, this, "[INT]: irq_time less than last_ts");
    }
  }
  if(is_clear_needed)
  {
    lsm6ds3c_send_interrupt_is_cleared_msg(this);
  }
}

static void lsm6ds3c_handle_hw_interrupts(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)this->state->state;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  sns_sensor_event *event;
  pb_istream_t stream;

  // Handle interrupts
  if(NULL != state->interrupt_data_stream)
  {
    uint16_t int1_event_count = 0;
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
    while(NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        DBG_INST_PRINTF_EX(MED, this, "INT reg");

        state->irq_info.irq_ready = true;
        if(!state->irq2_inst_enabled || state->irq2_info.irq_ready)
        {
          state->irq_ready = true;
        }
        if(lsm6ds3c_is_md_int_required(this) && !state->route_md_to_irq2 && state->md_info.is_filter_settled)
        {
          lsm6ds3c_set_md_intr(this, true);
        }
        if((state->fifo_info.publish_sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO)) ||
           (state->accel_info.gated_client_present && !state->current_conf.md_enabled)) {
          lsm6ds3c_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
        }
        lsm6ds3c_update_pending_esp_intr(this);
      }
      else if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        //set this once entered in irq handle
        state->fifo_info.th_info.recheck_int = false;

        int1_event_count++;
        stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
    if(int1_event_count) {

      if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
      {
        if(state->fifo_info.reconfig_req)
          DBG_INST_PRINTF_EX(HIGH, this, "HW INT");


        DBG_INST_PRINTF_EX(HIGH, this, "HW INT: handle esp interrupt: %d/%d/%d/%d/%d",
            state->current_conf.md_enabled,
            state->md_info.is_filter_settled,
            state->self_test_info.test_alive,
            state->route_md_to_irq2,
            LSM6DS3C_IS_ESP_ENABLED(state));
        if((!state->self_test_info.test_alive) &&
           (LSM6DS3C_IS_ESP_ENABLED(state) ||
            (state->current_conf.md_enabled &&
             state->md_info.is_filter_settled &&
             !state->route_md_to_irq2)))
        {
          uint8_t src_regs[2];
          lsm6ds3c_read_src_regs(state, src_regs);
          if(src_regs[0] & 0x08)
            lsm6ds3c_handle_md_interrupt(this, irq_event.timestamp, &src_regs[0]);
          lsm6ds3c_handle_esp_interrupt(this, irq_event.timestamp, src_regs);
        }
        lsm6ds3c_handle_fifo_interrupt(this, irq_event.timestamp);
        if(state->health.expected_expiration <
           sns_get_system_time() + (state->fifo_info.avg_interrupt_intvl<<1))
        {
          lsm6ds3c_restart_hb_timer(this, true);
        }
        else
        {
          state->health.heart_attack = false;
          state->health.heart_attack_cnt = 0;
        }
      }
    }
  }
  // Handle interrupts on INT2
  if(NULL != state->interrupt2_data_stream)
  {
    uint16_t int2_event_count = 0;
    event = state->interrupt2_data_stream->api->peek_input(state->interrupt2_data_stream);
    while(NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        SNS_INST_PRINTF(HIGH, this, "INT2 reg");

        state->irq2_info.irq_ready = true;
        if(state->irq_info.irq_ready)
        {
          state->irq_ready = true;
        }
        if(lsm6ds3c_is_md_int_required(this) && state->route_md_to_irq2 && state->md_info.is_filter_settled)
        {
          lsm6ds3c_set_md_intr(this, true);
        }
        lsm6ds3c_update_pending_esp_intr(this);
      }
      else if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        //set this once entered in irq handle
        SNS_INST_PRINTF(HIGH, this, "IRQ msg on INT2 %d", event->message_id);
        int2_event_count++;
        stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
      }
      event = state->interrupt2_data_stream->api->get_next_input(state->interrupt2_data_stream);

    }
    if(int2_event_count) {

      if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
      {
        if(LSM6DS3C_IS_ESP_ENABLED(state) ||
           state->route_md_to_irq2)
        {
          uint8_t src_regs[2];
          lsm6ds3c_read_src_regs(state, src_regs);
          if((src_regs[0] & 0x08) && state->current_conf.md_enabled)
            lsm6ds3c_handle_md_interrupt(this, irq_event.timestamp, &src_regs[0]);
          lsm6ds3c_handle_esp_interrupt(this, irq_event.timestamp, src_regs);
          lsm6ds3c_send_interrupt_is_cleared_msg(this);
        }
      }
    }
  }
#if 0
  if((NULL != state->interrupt_data_stream) ||
      (NULL != state->interrupt2_data_stream)) {

    //re-enable MD if disabled to handle Wakeup interrupt
    //check if MD required & timer not in use & not - enabled yet
    if(lsm6ds3c_is_md_int_required(this) && !state->timer_md_data_stream && !state->current_conf.md_enabled)
      lsm6ds3c_enable_md(this, false);
  }
#endif
}

#define LSM6DS3C_IS_INT_LEVEL_TRIG(x) (((x) == SNS_INTERRUPT_TRIGGER_TYPE_HIGH) || \
                                      ((x) == SNS_INTERRUPT_TRIGGER_TYPE_LOW) ? true : false)

void lsm6ds3c_send_interrupt_is_cleared_msg(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)this->state->state;
  // clear msg is not needed for edge triggered and dae
  if(!LSM6DS3C_IS_INT_LEVEL_TRIG(state->irq_info.irq_config.interrupt_trigger_type) ||
      lsm6ds3c_dae_if_available(this))
    return;

  sns_data_stream* data_stream = state->interrupt_data_stream;
  sns_request irq_msg =
  {
    .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_IS_CLEARED,
    .request    = NULL
  };

  data_stream->api->send_request(data_stream, &irq_msg);
}

void lsm6ds3c_clear_interrupt_q(sns_sensor_instance *const instance,
    sns_data_stream* interrupt_data_stream)
{
  UNUSED_VAR(instance);
  if((interrupt_data_stream == NULL) || lsm6ds3c_dae_if_available(instance))
    return;
  sns_sensor_event *event;
  uint16_t num_events = 0;
  event = interrupt_data_stream->api->peek_input(interrupt_data_stream);
  while(NULL != event)
  {
    event = interrupt_data_stream->api->get_next_input(interrupt_data_stream);
    num_events++;
  }
  DBG_INST_PRINTF(HIGH, instance, "clean interrupt q events = %d", num_events);
  lsm6ds3c_send_interrupt_is_cleared_msg(instance);
}



/** this function executes for handling fifo read data only
 * driver make sure to have only one ascp com port req at a time*/
static void lsm6ds3c_handle_ascp_events(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  sns_data_stream *data_stream = state->async_com_port_data_stream;
  sns_sensor_event *event = NULL;
  uint32_t port_rw_events_rcvd = 0;

  if(NULL == data_stream || NULL == (event = data_stream->api->peek_input(data_stream))) {
    return;
  }

  // Handle Async Com Port events
  while(NULL != event)
  {
    if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
    {
      SNS_INST_PRINTF(ERROR, this, "ASCP error=%d", event->message_id);
      state->ascp_req_count = 0;
    }
    else if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id &&
            state->ascp_req_count)
    {
      if(state->fifo_info.reconfig_req)
        DBG_INST_PRINTF(HIGH, this, "ASCP VEC_RW req_cnt=%d", state->ascp_req_count);
      state->ascp_req_count--;

      //is this timestamp accurate to use?
      state->fifo_info.ascp_event_timestamp = event->timestamp;
      pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
      sns_ascp_for_each_vector_do(&stream, lsm6ds3c_process_com_port_vector, (void *)this);

      if(state->flushing_sensors != 0)
      {
        lsm6ds3c_send_fifo_flush_done(this, state->flushing_sensors, FLUSH_DONE_AFTER_DATA);
        state->flushing_sensors = 0;
        lsm6ds3c_restart_hb_timer(this, false);
      }

      lsm6ds3c_send_interrupt_is_cleared_msg(this);
      port_rw_events_rcvd++;
    }
    event = data_stream->api->get_next_input(data_stream);
  }

  if(port_rw_events_rcvd > 0)
  {
    if(port_rw_events_rcvd > 1)
    {
      LSM6DS3C_INST_DEBUG_TS(MED, this, "#rw_evs=%u", port_rw_events_rcvd);
    }
    //if heart beat had happened, just reset the flag
    //if events are coming through, no need to worry about heart_attack
    state->health.heart_attack = false;
    //TODO::how about the counter
    if(state->health.heart_attack_cnt < MAX_HEART_ATTACKS/2)
    {
      state->health.heart_attack_cnt = 0;
      // how does framework reacts receiving data after heartattack
    }

    //check if fifo int is still high, if high read fifo data
    //item:1 May be not req for ACCTIVE HIGH/LOW  Interrupts
    //recheck int status
    sns_time cur_time = sns_get_system_time();
    uint16_t min_samples_to_check = state->fifo_info.cur_wmk;
    if(state->fifo_info.reconfig_req || state->fifo_info.th_info.flush_req)
      min_samples_to_check = 1;

    LSM6DS3C_INST_DEBUG_TS(MED, this,
        "recheck fifo int cur_t=%u last=%u elapsed=%d",
        (uint32_t)cur_time,(uint32_t)(state->fifo_info.last_timestamp), (int32_t)(cur_time - state->fifo_info.last_timestamp));

    state->fifo_info.th_info.recheck_int = true;
    if(state->fifo_info.th_info.flush_req || state->fifo_info.th_info.interrupt_fired)
      state->fifo_info.th_info.recheck_int = false;

    if(state->fifo_info.reconfig_req)
      state->fifo_info.th_info.flush_req = true;
    lsm6ds3c_read_fifo_data(this, cur_time, false);

    //still req reconfig, do it here, hard reconfig
    if(state->fifo_info.reconfig_req && state->ascp_req_count == 0)
      lsm6ds3c_reconfig_fifo(this, false);
  }
}

static sns_rc lsm6ds3c_handle_heart_attack_timer_events(
  sns_sensor_instance *const this,
  sns_timer_sensor_event *timer_event)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_health *health = &state->health;
  sns_rc rv = SNS_RC_SUCCESS;

  if(((state->fifo_info.publish_sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO)) ||
      (state->accel_info.gated_client_present && !state->current_conf.md_enabled)) &&
     !state->self_test_info.test_alive)
  {
    DBG_INST_PRINTF(MED, this,
        "heart_attack: count=%u to=%u exp=%u last=%u",
        state->health.heart_attack_cnt, (uint32_t)timer_event->timeout_time,
        (uint32_t)health->expected_expiration, (uint32_t)state->fifo_info.last_timestamp);
  }
  else
  {
    health->expected_expiration = UINT64_MAX;
    health->heart_beat_timeout = UINT64_MAX/2;
    lsm6ds3c_restart_hb_timer(this, true);
    health->heart_attack = false;
    return SNS_RC_SUCCESS;
  }
  //check this event is closer to last timestamp
  //case: interrupt and timer would have fired at the same time
  //no need to handle this, check current time with previous interrupt time
  // QC: It's probably sufficient to look at the timer event timestamp.
  int64_t diff = timer_event->timeout_time - state->fifo_info.th_info.interrupt_ts;

  if(timer_event->timeout_time < state->fifo_info.last_timestamp ||
     diff < (health->heart_beat_timeout >> 1)) {
    //checkng most recent event, if the diff is less than one report interval
    DBG_INST_PRINTF(LOW, this, "heart_attack: diff=%u last_ts=%u int_ts=%u", 
        (int32_t)diff, (uint32_t)state->fifo_info.last_timestamp, (uint32_t)state->fifo_info.th_info.interrupt_ts);
    health->expected_expiration += health->heart_beat_timeout;
    return SNS_RC_SUCCESS;
  }

  health->heart_attack = true;
  health->heart_attack_cnt++;
  rv = SNS_RC_SUCCESS;
  if(health->heart_attack_cnt >= MAX_HEART_ATTACKS) {
    inst_exit_island(this);
    SNS_INST_PRINTF(ERROR, this, "heart_attack: Max reset tried.");
    health->expected_expiration = UINT64_MAX;
    health->heart_beat_timeout = UINT64_MAX/2;
    lsm6ds3c_restart_hb_timer(this, true);
    health->heart_attack = false;
    lsm6ds3c_send_interrupt_is_cleared_msg(this);
    rv = SNS_RC_INVALID_STATE;
    health->heart_attack_cnt = 0;

  } else if(health->heart_attack_cnt >= MAX_HEART_ATTACKS/2) {
    // reset sensor Something is wrong. Reset device.
    // Reset Sensor
    inst_exit_island(this);
    SNS_INST_PRINTF(ERROR, this, "heart_attack: Something wrong. Reseting Device!");
    lsm6ds3c_send_interrupt_is_cleared_msg(this);
    rv = lsm6ds3c_recover_device(this);
    //restart timer here even if this is periodic
    lsm6ds3c_restart_hb_timer(this, false);
    // increment till it reaches MAX_HEART_ATTACKS
    health->heart_attack_cnt++;
    health->heart_attack = false;
    rv = SNS_RC_NOT_AVAILABLE;
  }
  if(rv == SNS_RC_SUCCESS) {
    int64_t ts_diff;
    DBG_INST_PRINTF(LOW, this, "heart_attack: flushing FIFO");
    //heart attack timer fires, flush fifo and check last_ts
    //read wake_src/func_src register and handle related interrupt
    {
      uint8_t src_regs[2];
      lsm6ds3c_read_src_regs(state, src_regs);
      if(src_regs[0] & 0x08)
      {
        lsm6ds3c_handle_md_interrupt(this, sns_get_system_time(), &src_regs[0]);
      }
#if LSM6DS3C_ESP_ENABLED
      lsm6ds3c_handle_esp_interrupt(this, sns_get_system_time(), &src_regs[1]);
#endif
    }

    state->health.heart_attack_flush = true;

    lsm6ds3c_flush_fifo(this);
    sns_time cur_time = sns_get_system_time();
    ts_diff = cur_time - state->fifo_info.last_timestamp;
    if((!state->ascp_req_count) && (ts_diff < health->heart_beat_timeout)) {
      DBG_INST_PRINTF(LOW, this, "heart_attck: ts_diff=%d cur=%u last_ts=%u",ts_diff, (uint32_t)cur_time, (uint32_t)state->fifo_info.last_timestamp);
      state->health.heart_attack_cnt = 0;
      state->health.heart_attack = false;
      lsm6ds3c_send_interrupt_is_cleared_msg(this);
    }
    health->expected_expiration += health->heart_beat_timeout;
  }
  return rv;
}
static sns_rc lsm6ds3c_handle_polling_timer_events(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)this->state->state;
  sns_sensor_event *event;
  uint8_t event_cnt = 0;
  sns_rc rv = SNS_RC_SUCCESS;
  if(NULL != state->timer_polling_data_stream) {
    event = state->timer_polling_data_stream->api->peek_input(state->timer_polling_data_stream);
    while(NULL != event)
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        event_cnt++;
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
            event->event_len);
        sns_timer_sensor_event timer_event;
        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {

          if(state->fifo_info.publish_sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO))
          {
            //state->fifo_info.th_info.interrupt_fired = true;
            //state->fifo_info.th_info.interrupt_ts = timer_event.timeout_time;
            lsm6ds3c_read_fifo_data(this, timer_event.timeout_time, true);
          }
          else
          {
            sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_polling_data_stream);
            return rv;
          }
        }
      }
      event = state->timer_polling_data_stream->api->get_next_input(state->timer_polling_data_stream);
    }
  }
  //restart heart beat timer
  if(event_cnt) {
    lsm6ds3c_restart_hb_timer(this, false);

  }

  return rv;
}
sns_rc lsm6ds3c_handle_timer_events(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state =
    (lsm6ds3c_instance_state*)this->state->state;
  sns_sensor_event *event;
  sns_rc rv = SNS_RC_SUCCESS;


  if(NULL != state->timer_sensor_temp_data_stream)
  {
    event = state->timer_sensor_temp_data_stream->api->peek_input(state->timer_sensor_temp_data_stream);
    while(NULL != event)
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
            event->event_len);
        sns_timer_sensor_event timer_event;
        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if(state->fifo_info.publish_sensors & LSM6DS3C_SENSOR_TEMP
              &&
              state->sensor_temp_info.timer_is_active
              &&
              state->sensor_temp_info.sampling_intvl > 0)
          {
            lsm6ds3c_handle_sensor_temp_sample(this);
          }
          else
          {
            sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_temp_data_stream);
            state->sensor_temp_info.timer_is_active = false;
            return rv;
          }
        }
        else
        {
          DBG_INST_PRINTF_EX(ERROR, this, "Failed to decode sensor temp timer event");
        }
      }
      event = state->timer_sensor_temp_data_stream->api->get_next_input(state->timer_sensor_temp_data_stream);
    }
  }

  if(NULL != state->timer_self_test_data_stream)
  {
    event = state->timer_self_test_data_stream->api->peek_input(state->timer_self_test_data_stream);
    while(NULL != event)
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
            event->event_len);
        sns_timer_sensor_event timer_event;
        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if(state->common_info.mode & LSM6DS3C_MODE_SELF_TEST)
          {
            inst_exit_island(this);
            if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
              lsm6ds3c_inst_hw_self_test(this);
            else if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
              lsm6ds3c_inst_factory_self_test(this);
          }
        }
        else
        {
          DBG_INST_PRINTF_EX(ERROR, this, "Failed to decode selftest timer event");
        }
      }
      if(NULL != state->timer_self_test_data_stream)
      {
        event = state->timer_self_test_data_stream->api->get_next_input(state->timer_self_test_data_stream);
      }
      else
      {
        event = NULL;
        break;
      }
    }
  }

  if(NULL != state->timer_md_data_stream)
  {
    event = state->timer_md_data_stream->api->peek_input(state->timer_md_data_stream);
    while(NULL != event)
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
            event->event_len);
        sns_timer_sensor_event timer_event;

        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if(state->desired_conf.md_enabled && state->md_info.is_timer_running)
          {
            DBG_INST_PRINTF_EX(HIGH, this, "Enable MD filter_settled");
            state->md_info.is_timer_running = false;
            lsm6ds3c_update_md_filter(this);
          }
        }
        else
        {
          DBG_INST_PRINTF_EX(ERROR, this, "Failed to decode MD timer event");
        }
      }
      event = state->timer_md_data_stream->api->get_next_input(state->timer_md_data_stream);
    }
    if(state->md_info.filter == HP_FILTER || !lsm6ds3c_is_md_int_required(this))
    {
      state->md_info.is_timer_running = false;
    }
  }
  // QC: Put in another function?
  if(NULL != state->timer_heart_beat_data_stream)
  {
    sns_timer_sensor_event latest_timer_event;
    event = state->timer_heart_beat_data_stream->api->peek_input(state->timer_heart_beat_data_stream);
    uint8_t event_cnt = 0;

    while(NULL != event)
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
            event->event_len);
        sns_timer_sensor_event timer_event;
        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          latest_timer_event = timer_event;
          if(!state->self_test_info.test_alive) //Skip heart attack for  self-test case
            event_cnt++;
        }
        else
        {
          DBG_INST_PRINTF_EX(ERROR, this, "Failed to decode heart beat timer event");
        }
      }
      event = state->timer_heart_beat_data_stream->api->get_next_input(state->timer_heart_beat_data_stream);
    }
    if(event_cnt) {
      rv = lsm6ds3c_handle_heart_attack_timer_events(this, &latest_timer_event);
    }
  }
  lsm6ds3c_handle_polling_timer_events(this);
  return rv;
}

static void lsm6ds3c_report_error(sns_sensor_instance *this, sns_sensor_uid const *suid)
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

static void lsm6ds3c_process_flush_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_sensor_type flushing_sensor = *((lsm6ds3c_sensor_type*)client_request->request);
  sns_time now = sns_get_system_time();
  if(state->flushing_sensors != 0)
  {
    // Flush already started
    state->flushing_sensors |= flushing_sensor;
  }
  else if((state->fifo_info.last_timestamp + state->fifo_info.avg_sampling_intvl) > now)
  {
    // there're no samples to flush
    lsm6ds3c_send_fifo_flush_done(this, flushing_sensor, FLUSH_DONE_FIFO_EMPTY);
  }
  else if(state->fifo_info.cur_wmk == 1 &&
          (!lsm6ds3c_dae_if_available(this) || state->fifo_info.max_requested_wmk <= 1))
  {
    // ignore flush req if wmk=1
    lsm6ds3c_send_fifo_flush_done(this, flushing_sensor, FLUSH_DONE_FIFO_EMPTY);
  }
  else
  {
    sns_time next_int;
    state->flushing_sensors |= flushing_sensor;
    next_int = (state->fifo_info.bh_info.interrupt_ts +
                state->fifo_info.avg_interrupt_intvl);
    if(state->fifo_info.max_requested_wmk > state->fifo_info.cur_wmk  || // DAE WM bigger than FIFO WM.
       next_int < now || // flush-only request or missed interrupt
       next_int > now + state->fifo_info.avg_sampling_intvl )
    {
      lsm6ds3c_flush_fifo(this);
    }
    // else be patient, interrupt will fire soon
  }
}

static void lsm6ds3c_process_cal_reset_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  sns_lsm6ds3c_registry_cfg const *src_cal =
    (sns_lsm6ds3c_registry_cfg const *)client_request->request;
  sns_lsm6ds3c_registry_cfg *dest_cal = (src_cal->sensor_type == LSM6DS3C_ACCEL) ?
    &state->accel_registry_cfg : &state->gyro_registry_cfg;
  lsm6ds3c_copy_calibration_info(dest_cal, src_cal);
  lsm6ds3c_send_cal_event(this, src_cal->sensor_type);
}

bool lsm6ds3c_is_md_int_required(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  bool md_int_req = false;
  //md client is present and non-gated request is not present
  if(state->md_info.client_present &&
      !(state->fifo_info.publish_sensors & LSM6DS3C_ACCEL))
    md_int_req = true;

  //md is required if any internal client is requested
  md_int_req = (md_int_req || state->md_info.internal_client_present);

  return md_int_req;
}


static sns_rc lsm6ds3c_parse_config_info(
  sns_sensor_instance     *const this,
  lsm6ds3c_instance_config *inst_cfg)
{
  sns_rc rc = SNS_RC_SUCCESS;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  uint8_t desired_rate_idx = 0;
  uint32_t curr_max_requested_wmk = state->fifo_info.max_requested_wmk;

  state->config_sensors |= inst_cfg->config_sensors;

  if(inst_cfg->fifo_enable & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO))
  {
    desired_rate_idx = lsm6ds3c_get_odr_rate_idx(inst_cfg->sample_rate);
    desired_rate_idx = SNS_MAX(desired_rate_idx, state->min_odr_idx);
  }

  if(desired_rate_idx < lsm6ds3c_odr_map_len)
  {
    float desired_report_rate = inst_cfg->report_rate;
    float dae_report_rate     = inst_cfg->dae_report_rate;
    state->fifo_info.fifo_enabled       = inst_cfg->fifo_enable;
    state->fifo_info.publish_sensors    = inst_cfg->client_present & ~LSM6DS3C_MOTION_DETECT;
    //if gated client present and no MD request present
    //gated request becomes non-gated
    if(inst_cfg->gated_client_present && !(inst_cfg->client_present & LSM6DS3C_MOTION_DETECT))
    {
      state->fifo_info.publish_sensors |= inst_cfg->gated_client_present;
    }
    state->accel_info.gated_client_present = (inst_cfg->gated_client_present != 0);
    state->desired_conf.odr_idx         = desired_rate_idx;
    state->desired_conf.odr             = lsm6ds3c_odr_map[desired_rate_idx].odr;
    state->desired_conf.enabled_sensors = (state->fifo_info.fifo_enabled &
                                           (LSM6DS3C_ACCEL | LSM6DS3C_GYRO));

    state->md_info.client_present      = (inst_cfg->client_present & LSM6DS3C_MOTION_DETECT);
    state->desired_conf.md_enabled     = lsm6ds3c_is_md_int_required(this);

    desired_report_rate = SNS_MIN(desired_report_rate, state->desired_conf.odr);
    dae_report_rate     = SNS_MIN(dae_report_rate, state->desired_conf.odr);
    if(inst_cfg->gated_client_present)
    {
      if(!state->fifo_info.new_config.gated_timestamp)//Note the time when first gated request comes
        state->fifo_info.new_config.gated_timestamp =  sns_get_system_time();
      else if(!(state->desired_conf.enabled_sensors & LSM6DS3C_ACCEL)//note the time when non gated request become gated
            && (state->current_conf.enabled_sensors & LSM6DS3C_ACCEL))
        state->fifo_info.new_config.gated_timestamp =  sns_get_system_time();
    }
    else
    {
      //when gated request goes away
      state->fifo_info.new_config.gated_timestamp = 0;
    }
    // calculate FIFO WM
    if(LSM6DS3C_ODR_0 < state->desired_conf.odr && 0.0f < desired_report_rate)
    {
      uint16_t max_fifo = LSM6DS3C_MAX_FIFO;
      if(state->desired_conf.enabled_sensors & LSM6DS3C_GYRO &&
         (max_fifo > (LSM6DS3C_HW_MAX_FIFO >> 1)))
      {
        max_fifo >>= 1;
      }
      if(((float)state->desired_conf.odr / UINT16_MAX) >= desired_report_rate)
      {
        state->desired_conf.wmk = max_fifo;
      }
      else
      {
        state->desired_conf.wmk =
          (uint16_t)lroundf(state->desired_conf.odr/desired_report_rate);
        if( state->desired_conf.wmk > max_fifo )
        {
          uint32_t divider = (max_fifo - 1 + state->desired_conf.wmk) / max_fifo;
          state->desired_conf.wmk = state->desired_conf.wmk / SNS_MAX(divider,1);
        }
        else if(state->desired_conf.wmk * desired_report_rate * 0.9f > state->desired_conf.odr)
        {
          // must adjust WM or the actual report rate would be too low
          state->desired_conf.wmk--;
        }
      }
    }
    else if(LSM6DS3C_ODR_0 == state->desired_conf.odr)
    {
      state->desired_conf.wmk = 0;
    }
    else
    {
      state->desired_conf.wmk = 1;
    }

    // calculate DAE WM
    if(FLT_MIN < dae_report_rate &&
       ((float)state->desired_conf.odr / UINT32_MAX) < dae_report_rate)
    {
      state->fifo_info.max_requested_wmk =
       (uint32_t)(lroundf(state->desired_conf.odr / dae_report_rate));
      if(state->fifo_info.max_requested_wmk > state->desired_conf.wmk)
      {
        state->fifo_info.max_requested_wmk =
          (state->fifo_info.max_requested_wmk / state->desired_conf.wmk) *
          state->desired_conf.wmk;
      }
      if(state->fifo_info.max_requested_wmk * dae_report_rate * 0.9f > state->desired_conf.odr)
      {
        // must adjust WM or the actual report rate would be too low
        state->fifo_info.max_requested_wmk--;
      }
    }
    else
    {
      state->fifo_info.max_requested_wmk = UINT32_MAX;
    }

    // Flush old DAE data before reconfiguration as SEE driver does not need the old data.
    if(state->fifo_info.publish_sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO) &&
       state->fifo_info.max_requested_flush_ticks == 0)
    {
      DBG_INST_PRINTF(LOW, this, "config: resetting last_timestamp");
      state->fifo_info.last_timestamp = sns_get_system_time();
      lsm6ds3c_dae_if_flush_hw(this);
    }

    state->fifo_info.max_requested_flush_ticks = inst_cfg->flush_period_ticks * 1.1f;
    DBG_INST_PRINTF(LOW, this, "config: publish=0x%x dae_wm=%u flush_per=%u",
                    state->fifo_info.publish_sensors, state->fifo_info.max_requested_wmk,
                    (uint32_t)state->fifo_info.max_requested_flush_ticks);

    state->fifo_info.reconfig_req = false;
    state->fifo_info.full_reconf_req = false;

    if((state->desired_conf.wmk != state->current_conf.wmk) ||
       (state->desired_conf.odr != state->current_conf.odr) ||
       (state->desired_conf.enabled_sensors != state->current_conf.enabled_sensors))
    {
      state->fifo_info.reconfig_req = true;
    }
    else if(LSM6DS3C_IS_ESP_CONF_CHANGED(state))
    {
      state->fifo_info.reconfig_req = true;
    }

    if(state->desired_conf.enabled_sensors == 0 && state->current_conf.enabled_sensors == 0)
    {
      state->fifo_info.reconfig_req = false;
    }
    else if(LSM6DS3C_IS_ESP_CONF_CHANGED(state) ||
            (state->current_conf.md_enabled != state->desired_conf.md_enabled))
    {
      state->fifo_info.full_reconf_req = true;
      state->fifo_info.reconfig_req = true;
    }

    if(!state->fifo_info.reconfig_req &&
       curr_max_requested_wmk != state->fifo_info.max_requested_wmk)
    {
      lsm6ds3c_update_heartbeat_monitor(this);
    }

    //If odr, wmk, sensors all same as before (reconfig_req = false)
    //but flush_period is changed
    //case: if first request is with flush_period = 0 (DAE discards data instead of pushing to SEE)
    //and new request has non-zero flush_period
    bool will_stream = ((state->fifo_info.publish_sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO)) &&
        (!lsm6ds3c_dae_if_available(this) || inst_cfg->flush_period_ticks));

    if(!state->fifo_info.reconfig_req &&
        !state->fifo_info.is_streaming &&
        will_stream) {
      state->fifo_info.last_ts_valid = false; //set last ts as inaccurate
      state->fifo_info.is_streaming = will_stream;
      //move last ts, useful when flush comes right after this
      state->fifo_info.last_timestamp = sns_get_system_time();
    }

    DBG_INST_PRINTF(MED, this, "[%u] config: have(i,w,r,s,md) %u,%u,%u,0x%x,%u",
                    state->hw_idx, state->current_conf.odr_idx,
                    state->current_conf.wmk, state->current_conf.odr,
                    state->current_conf.enabled_sensors, state->current_conf.md_enabled);
    DBG_INST_PRINTF(MED, this, "[%u] config: want(i,w,r,s,md) %u,%u,%u,0x%x,%u",
                    state->hw_idx, state->desired_conf.odr_idx,
                    state->desired_conf.wmk, state->desired_conf.odr,
                    state->desired_conf.enabled_sensors, state->desired_conf.md_enabled);
  }
  else
  {
    if(inst_cfg->fifo_enable & LSM6DS3C_ACCEL)
    {
      rc = SNS_RC_FAILED;
      SNS_INST_PRINTF(ERROR, this, "Sending error report for ACCEL");
      lsm6ds3c_report_error(this, &state->accel_info.suid);
    }
    if(inst_cfg->fifo_enable & LSM6DS3C_GYRO)
    {
      rc = SNS_RC_FAILED;
      SNS_INST_PRINTF(ERROR, this, "Sending error report for GYRO");
      lsm6ds3c_report_error(this, &state->gyro_info.suid);
    }
  }

  if(inst_cfg->config_sensors & LSM6DS3C_SENSOR_TEMP)
  {
    if(inst_cfg->client_present & LSM6DS3C_SENSOR_TEMP)
    {
      rc = lsm6ds3c_validate_sensor_temp_odr(state, inst_cfg);
      if(rc == SNS_RC_SUCCESS)
      {
        DBG_INST_PRINTF(MED, this, "config: ST(RR*1000/SR)=%u/%u",
                        (uint32_t)(state->sensor_temp_info.report_rate_hz*1000),
                        (uint32_t)state->sensor_temp_info.desired_sampling_rate_hz);
        lsm6ds3c_send_config_event(this, false);
      }
      else
      {
        SNS_INST_PRINTF(ERROR, this, "Sending error report for TEMP");
        lsm6ds3c_report_error(this, &state->sensor_temp_info.suid);
      }
    }
    else
    {
      state->sensor_temp_info.desired_sampling_rate_hz = 0.0f;
      state->sensor_temp_info.sampling_intvl = 0;
    }
  }

  return rc;
}

static void lsm6ds3c_process_sensor_config_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  // 1. Extract sample, report rates from client_request.
  // 2. Configure sensor HW.
  // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
  // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
  // 5. Save the current config information like type, sample_rate, report_rate, etc.
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_instance_config *payload = (lsm6ds3c_instance_config*)client_request->request;
#if LSM6DS3C_DAE_ENABLED
  uint32_t current_dae_wm = state->fifo_info.max_requested_wmk;
#endif

  if(state->self_test_info.reconfig_postpone) {
    payload->config_sensors |= state->config_sensors;
  }

  DBG_INST_PRINTF_EX(HIGH, this, "config_sensors = 0x%x", payload->config_sensors);
  //check if any other sensors are configuring
  if(!(payload->config_sensors)) {
    return;
  }

  sns_rc rc = lsm6ds3c_parse_config_info(this, payload);
  if(SNS_RC_SUCCESS == rc)
  {
    const odr_reg_map *map_entry = &lsm6ds3c_odr_map[state->desired_conf.odr_idx];

    DBG_INST_PRINTF(HIGH, this, "client_config: reconfig=%u", state->fifo_info.reconfig_req);
    // Reset ACCEL/GYRO bits of config sensors, If there is no change in configuration
    if(!state->fifo_info.reconfig_req && !lsm6ds3c_dae_if_available(this))
    {
      state->config_sensors &= ~(LSM6DS3C_ACCEL | LSM6DS3C_GYRO);
    }
    if(state->accel_info.gated_client_present)
    {
      DBG_INST_PRINTF_EX(HIGH, this, "gated accel config");

      lsm6ds3c_set_gated_accel_config(state,
          state->desired_conf.wmk,
          map_entry->accel_odr_reg_value,
          state->fifo_info.fifo_enabled);
    }

    if(state->current_conf.odr > 0.0f || state->accel_info.gated_client_present ||
        state->sensor_temp_info.cur_sampling_rate_hz > 0.0f)
    {
      bool send_old_config = true;
      if(state->fifo_info.last_sent_config.sample_rate    != state->current_conf.odr ||
         state->fifo_info.last_sent_config.fifo_watermark != state->current_conf.wmk) 
      {
        state->fifo_info.new_config.sample_rate    = state->current_conf.odr;
        state->fifo_info.new_config.fifo_watermark = state->current_conf.wmk;
#if LSM6DS3C_DAE_ENABLED
        state->fifo_info.new_config.dae_watermark  = current_dae_wm;
#endif
        send_old_config = false;
      }

      // new clients need to know ODR/WM for samples received between now and
      // when the next config takes effect
      lsm6ds3c_send_config_event(this, send_old_config);
    }
    else
    {
      sns_memzero(&state->fifo_info.last_sent_config, sizeof(state->fifo_info.last_sent_config));
    }

    if(state->fifo_info.reconfig_req)
    {
      lsm6ds3c_set_fifo_config(this,
          state->desired_conf.wmk,
          map_entry->accel_odr_reg_value,
          map_entry->gyro_odr_reg_value,
          state->fifo_info.fifo_enabled);
    }

    if(!state->self_test_info.test_alive) {
      if(!lsm6ds3c_dae_if_available(this)) {
        if((state->sensor_temp_info.desired_sampling_rate_hz !=
            state->sensor_temp_info.cur_sampling_rate_hz) &&
           ((state->fifo_info.publish_sensors & LSM6DS3C_SENSOR_TEMP) ||
            state->sensor_temp_info.timer_is_active)) {
          lsm6ds3c_set_polling_config(this);
        }
      }
#if LSM6DS3C_DAE_ENABLED
      else if(state->config_step == LSM6DS3C_CONFIG_IDLE)
      {
        bool stopping = false;
        if(state->fifo_info.reconfig_req)
        {
          lsm6ds3c_sensor_type sensors =
            (state->config_sensors | state->current_conf.enabled_sensors) & ~LSM6DS3C_SENSOR_TEMP;
          stopping |= lsm6ds3c_dae_if_stop_streaming(this, sensors);
        }
        if(state->sensor_temp_info.desired_sampling_rate_hz !=
           state->sensor_temp_info.cur_sampling_rate_hz)
        {
          stopping |= lsm6ds3c_dae_if_stop_streaming(this, LSM6DS3C_SENSOR_TEMP);
        }
        if(stopping)
        {
          state->config_step = LSM6DS3C_CONFIG_STOPPING_STREAM;
        }
      }
      if(state->config_step == LSM6DS3C_CONFIG_IDLE)
#endif
      {
        lsm6ds3c_reconfig_hw(this);
        lsm6ds3c_dump_reg(this, state->fifo_info.fifo_enabled);
      }
      state->self_test_info.reconfig_postpone = false;
    }
    else
    {
      //Postpone the reconfig till Self-test is over
      state->fifo_info.reconfig_req = false;
      state->self_test_info.reconfig_postpone = true;
    }
  }
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc lsm6ds3c_inst_notify_event(sns_sensor_instance *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  sns_rc com_rv = SNS_RC_SUCCESS;

  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
#ifdef SNS_AVOID_HANDLING_INTERRUPT_DURING_PC
  if ((true == pc_in_progress()) && state->current_conf.md_enabled)
  {
    SNS_INST_PRINTF(ERROR, this, "notify_event: pc_in_progress ");
    return rv;
  }
#endif

  // Turn COM port ON
  com_rv = state->scp_service->api->sns_scp_update_bus_power(
     state->com_port_info.port_handle, true);
  if (SNS_RC_SUCCESS != com_rv)
  {
    DBG_INST_PRINTF_EX(ERROR, this, "notify_event: update_bus_power ON %d", com_rv);
  }
  lsm6ds3c_dae_if_process_events(this);
  lsm6ds3c_handle_ascp_events(this);
  lsm6ds3c_handle_hw_interrupts(this);
  rv = lsm6ds3c_handle_esp_timer_events(this);
  rv = lsm6ds3c_handle_timer_events(this);


  // Turn COM port OFF
  com_rv = state->scp_service->api->sns_scp_update_bus_power(
     state->com_port_info.port_handle, false);
  if (SNS_RC_SUCCESS != com_rv)
  {
    SNS_INST_PRINTF(ERROR, this, "notify_event: update_bus_power OFF %d", com_rv);
  }

  return rv;
}

/** See sns_sensor_instance_api::set_client_config */
static sns_rc lsm6ds3c_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;

  if(client_request->message_id != SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    SNS_INST_PRINTF(MED, this, "[%u] client_config: msg=%u #samples=%u/%u/%u/%u",
                    state->hw_idx, client_request->message_id, state->accel_sample_counter,
                    state->gyro_sample_counter, state->num_temp_samples, state->num_md_ints);
  }
#if LSM6DS3C_AUTO_DEBUG
  DBG_INST_PRINTF(HIGH, this, "set_client_config: msg=%u", client_request->message_id);
#endif
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
      true);

  if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
    lsm6ds3c_process_sensor_config_request(this, client_request);
  }
  else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    lsm6ds3c_process_flush_request(this, client_request);
  }
  else if(client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    /** All self-tests are handled in normal mode. */
    inst_exit_island(this);
    lsm6ds3c_set_client_test_config(this, client_request);
  }
  else if(client_request->message_id == SNS_CAL_MSGID_SNS_CAL_RESET)
  {
    lsm6ds3c_process_cal_reset_request(this, client_request);
  }

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
      false);
  return SNS_RC_SUCCESS;
}

void lsm6ds3c_copy_calibration_info(
  sns_lsm6ds3c_registry_cfg *dest_cal,
  sns_lsm6ds3c_registry_cfg const *src_cal)
{
  sns_memscpy(&dest_cal->fac_cal_bias, sizeof(dest_cal->fac_cal_bias),
              src_cal->fac_cal_bias, sizeof(src_cal->fac_cal_bias));

  sns_memscpy(&dest_cal->fac_cal_corr_mat, sizeof(dest_cal->fac_cal_corr_mat),
              &src_cal->fac_cal_corr_mat, sizeof(src_cal->fac_cal_corr_mat));

  dest_cal->registry_persist_version = src_cal->registry_persist_version;

  dest_cal->ts = src_cal->ts;
}


/** Public Data Definitions. */

sns_sensor_instance_api lsm6ds3c_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &lsm6ds3c_inst_init,
  .deinit            = &lsm6ds3c_inst_deinit,
  .set_client_config = &lsm6ds3c_inst_set_client_config,
  .notify_event      = &lsm6ds3c_inst_notify_event
};

