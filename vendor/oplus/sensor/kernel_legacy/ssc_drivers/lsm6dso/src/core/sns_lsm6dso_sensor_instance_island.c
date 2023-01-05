/**
 * @file sns_lsm6dso_sensor_instance.c
 *
 * LSM6DSO Accel virtual Sensor Instance implementation.
 *
 * Copyright (c) 2020, STMicroelectronics.
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

#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"

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

#define IBI_CLOCK_FREQ_RES (0.5f) //ibi clock freq resolution in MHz
/* accel_discard_samples = AN4987 value + 2
   gyro_discard_samples =  AN4987 value + 2 + LSM6DSO_GYRO_ON_TIME_MS equivalent samples
*/
#define MAX_HEART_ATTACKS 6

#define ODR_832_DISCARD_SAMPLE 50 //required with lpf1 enable for gyro
const odr_reg_map lsm6dso_odr_map[] =
{
  {
    .odr = LSM6DSO_ODR_0,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR_OFF,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR_OFF,
    .odr_coeff = 0,
    .accel_group_delay = 0,
    .gyro_group_delay = 0,
    .accel_discard_samples = 0,
    .gyro_discard_samples = 0
  },
  {
    .odr = LSM6DSO_ODR_13,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR13,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR13,
    .odr_coeff = 512,
    .accel_group_delay = 57.69f,
    .gyro_group_delay = 74.79f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 3
  },
  {
    .odr = LSM6DSO_ODR_26,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR26,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR26,
    .odr_coeff = 256,
    .accel_group_delay = 28.89f,
    .gyro_group_delay = 38.89f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DSO_ODR_52,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR52,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR52,
    .odr_coeff = 128,
    .accel_group_delay = 14.44f,
    .gyro_group_delay = 19.44f,
    .accel_discard_samples = 3,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DSO_ODR_104,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR104,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR104,
    .odr_coeff = 64,
    .accel_group_delay = 7.22f,
    .gyro_group_delay = 9.72f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DSO_ODR_208,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR208,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR208,
    .odr_coeff = 32,
    .accel_group_delay = 3.74f,
    .gyro_group_delay = 4.86f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DSO_ODR_416,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR416,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR416,
    .odr_coeff = 16,
    .accel_group_delay = 1.93f,
    .gyro_group_delay = 2.46f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = 4
  },
  {
    .odr = LSM6DSO_ODR_832,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR832,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR832,
    .odr_coeff = 8,
    .accel_group_delay = 1.03f,
    .gyro_group_delay = 1.26f,
    .accel_discard_samples = 4,
    .gyro_discard_samples = ODR_832_DISCARD_SAMPLE
  },
  {
    .odr = LSM6DSO_ODR_1664,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR1664,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR1664,
    .odr_coeff = 4,
    .accel_group_delay = 0.58f,
    .gyro_group_delay = 0.69f,
    .accel_discard_samples = 7,
    .gyro_discard_samples = 136
  },
  {
    .odr = LSM6DSO_ODR_3328,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR3328,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR3328,
    .odr_coeff = 2,
    .accel_group_delay = 0.36f,
    .gyro_group_delay = 0.42f,
    .accel_discard_samples = 13,
    .gyro_discard_samples = 271
  },
  {
    .odr = LSM6DSO_ODR_6656,
    .accel_odr_reg_value = LSM6DSO_ACCEL_ODR6656,
    .gyro_odr_reg_value = LSM6DSO_GYRO_ODR6656,
    .odr_coeff = 1,
    .accel_group_delay = 0.25f,
    .gyro_group_delay = 0.28f,
    .accel_discard_samples = 35,
    .gyro_discard_samples = 541
  },
};

const uint32_t lsm6dso_odr_map_len = ARR_SIZE(lsm6dso_odr_map);

void lsm6dso_inst_exit_island(sns_sensor_instance *this)
{
#if !LSM6DSO_ISLAND_DISABLED
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
#else
  UNUSED_VAR(this);
#endif
}
void lsm6dso_inst_create_timer(sns_sensor_instance *this,
    sns_data_stream** timer_data_stream,
    sns_timer_sensor_config* req_payload)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  if(!timer_data_stream || !req_payload) {
    SNS_INST_PRINTF(ERROR, this, "Invalid timer data stream or req_payload pointer passed");
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
    LSM6DSO_INST_DEBUG_TS(
      MED, this, "set timer start_time=%u timeout_period=%d",
      (uint32_t)req_payload->start_time, (int32_t)req_payload->timeout_period);
  }
  else
  {
    SNS_INST_PRINTF(ERROR, this, "LSM timer req encode error");
  }
}

uint8_t lsm6dso_get_odr_rate_idx(float desired_sample_rate)
{
  uint8_t rate_idx = lsm6dso_odr_map_len;
  if(MAX_LOW_LATENCY_RATE >= desired_sample_rate)
  {
    for(uint8_t i = 0; i < lsm6dso_odr_map_len; i++)
    {
      if(desired_sample_rate <= lsm6dso_odr_map[i].odr)
      {
        rate_idx = i;
        break;
      }
    }
  }
  return rate_idx;
}

static sns_rc lsm6dso_validate_sensor_temp_odr(
  lsm6dso_instance_state *state,
  lsm6dso_instance_config* inst_cfg)
{
  sns_rc rc = SNS_RC_SUCCESS;
  lsm6dso_sensor_temp_info *sti = &state->sensor_temp_info;

  sti->desired_sampling_rate_hz = 0.0f;
  if(inst_cfg->temper_sample_rate <= LSM6DSO_SENSOR_TEMP_ODR_1)
  {
    sti->desired_sampling_rate_hz = LSM6DSO_SENSOR_TEMP_ODR_1;
  }
  else if(inst_cfg->temper_sample_rate > LSM6DSO_SENSOR_TEMP_ODR_1 &&
          inst_cfg->temper_sample_rate <= LSM6DSO_SENSOR_TEMP_ODR_5)
  {
    sti->desired_sampling_rate_hz = LSM6DSO_SENSOR_TEMP_ODR_5;
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
#if LSM6DSO_DAE_ENABLED
    sti->new_config.dae_watermark = state->sensor_temp_info.desired_sampling_rate_hz /
                                    state->sensor_temp_info.report_rate_hz;
#endif
  }

  return rc;
}
void lsm6dso_restart_hb_timer(sns_sensor_instance *const this, bool reset)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_health *health = &state->health;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;

  req_payload.is_periodic = true;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = health->heart_beat_timeout;
  lsm6dso_inst_create_timer(this, &state->timer_heart_beat_data_stream, &req_payload);

  health->expected_expiration = req_payload.start_time + health->heart_beat_timeout;
  if(reset)
  {
    health->heart_attack     = false;
    health->heart_attack_cnt = 0;
  }
  DBG_INST_PRINTF_EX(HIGH, this, "config: (i,w,r,s,md) %u, %u, %u, 0x%x, 0x%x, %u",
      state->current_conf.odr_idx,
      state->current_conf.wmk, state->current_conf.odr,
      state->current_conf.enabled_sensors,
      state->current_conf.publish_sensors,
      state->current_conf.md_enabled);


  DBG_INST_PRINTF_EX(HIGH, this,
      "last_irq, last_ts: %u, %u", (uint32_t)(state->fifo_info.interrupt_timestamp), (uint32_t)(state->fifo_info.last_timestamp));
}


#if !LSM6DSO_DAE_ENABLED
static void lsm6dso_handle_fifo_interrupt(sns_sensor_instance *const this, sns_time irq_ts)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  //set this once entered in irq handle
  state->fifo_info.th_info.recheck_int = false;
  if(state->fifo_info.cur_wmk > 0
      && state->fifo_info.fifo_rate > LSM6DSO_ACCEL_ODR_OFF)
  {
    LSM6DSO_INST_DEBUG_TS(MED, this,
        "[INT] last_ts = %u irq_time = %u",(uint32_t)(state->fifo_info.last_timestamp), (uint32_t)(irq_ts));

    //check if this irq is valid, may belong to the recent flush request
    if(irq_ts > state->fifo_info.last_timestamp) {
      state->fifo_info.th_info.interrupt_fired = true;
      state->fifo_info.th_info.interrupt_ts = irq_ts;
      lsm6dso_read_fifo_data(this, irq_ts, false);
    } else {
      LSM6DSO_INST_DEBUG_TS(MED, this, "[INT]: irq_time less than last_ts");
    }
  }
}

static sns_rc lsm6dso_handle_int1_event(sns_sensor_instance *const this,
    sns_time cur_time,
    uint16_t event_msg,
    sns_interrupt_event* irq_event)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  UNUSED_VAR(cur_time);
  sns_time irq_ts = irq_event->timestamp;
  // Handle interrupts
  if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event_msg)
  {
    DBG_INST_PRINTF_EX(MED, this, "INT reg");

    state->irq_info.irq_ready = true;

    if(!state->irq2_inst_enabled || state->irq2_info.irq_ready)
    {
      state->irq_ready = true;
    }

    /* This is to make sure to check if client req reaches to instance(state->FW->instance) or not
    instance config_sensors variable would be set only when client req reaches to instance
    usecase:
    1.state receives req and sents to fw to call instance client_set_req function
    2. Meanwhile receives interrupt registered event
    3. At this stage, driver should wait for client req to reach instance, instead of configuring */


    if(lsm6dso_is_md_int_required(this) && !state->route_md_to_irq2 && state->md_info.is_filter_settled)
    {
      lsm6dso_set_md_intr(this, true);
    }
    if((state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) ||
        (state->accel_info.gated_client_present && !state->current_conf.md_enabled)) {
      lsm6dso_enable_fifo_intr(this, state->fifo_info.fifo_enabled);
    }
    if(LSM6DSO_IS_ESP_DESIRED(state))
      lsm6dso_reconfig_esp(this);
  }
  else if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event_msg)
  {
    //set this once entered in irq handle
    state->fifo_info.th_info.recheck_int = false;

    DBG_INST_PRINTF_EX(HIGH, this, "[%d]HW INT: handle interrupt: %d/%d/%d/%d/%d",
        state->hw_idx,
        state->current_conf.md_enabled,
        state->md_info.is_filter_settled,
        state->self_test_info.test_alive,
        state->route_md_to_irq2,
        LSM6DSO_IS_ESP_ENABLED(state));

    uint8_t rw_buffer[2];
    if(!state->route_md_to_irq2) 
    {
      if(LSM6DSO_IS_ESP_ENABLED(state) || (state->current_conf.md_enabled))
      {
        uint8_t wake_src = STM_LSM6DSO_REG_WAKE_SRC;
        uint8_t tap_src = STM_LSM6DSO_REG_TAP_SRC;
        lsm6dso_read_regs_scp(this, STM_LSM6DSO_REG_WAKE_SRC, 2, rw_buffer); 

        if(!state->self_test_info.test_alive) {
          //read md register only if md is settled and enabled
          if(state->current_conf.md_enabled && state->md_info.is_filter_settled)
            lsm6dso_handle_md_interrupt(this, irq_ts, &rw_buffer[0]);
        }
        if(LSM6DSO_IS_ESP_ENABLED(state))
        {
          lsm6dso_handle_esp_interrupt(this, irq_ts, &wake_src, &rw_buffer[0]);
          if(rw_buffer[1] != 0)
            lsm6dso_handle_esp_interrupt(this, irq_event->timestamp, &tap_src, &rw_buffer[1]);
        }
      }
    }
    lsm6dso_handle_fifo_interrupt(this, irq_ts);
    if(state->health.expected_expiration <
        sns_get_system_time() + (state->fifo_info.avg_interrupt_intvl<<1))
    {
      lsm6dso_restart_hb_timer(this, true);
    }
    else
    {
      state->health.heart_attack = false;
      state->health.heart_attack_cnt = 0;
    }
  }
  return SNS_RC_SUCCESS;
}

static sns_rc lsm6dso_handle_int2_event(sns_sensor_instance *const this,
    sns_time cur_time,
    uint16_t event_msg,
    sns_interrupt_event* irq_event)
{
  UNUSED_VAR(cur_time);
#if INT2_SUPPORT
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event_msg)
  {
    SNS_INST_PRINTF(HIGH, this, "INT2 reg");

    state->irq2_info.irq_ready = true;
    if(state->irq_info.irq_ready)
    {
      state->irq_ready = true;
    }
    if(lsm6dso_is_md_int_required(this) && state->route_md_to_irq2)
    {
      lsm6dso_set_md_intr(this, true);
    }
  }
  else if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event_msg)
  {
    //set this once entered in irq handle
    SNS_INST_PRINTF(HIGH, this, "IRQ msg on INT2 %d", event_msg);

    if((state->current_conf.md_enabled && state->md_info.is_filter_settled &&
          !state->self_test_info.test_alive && state->route_md_to_irq2) ||
        LSM6DSO_IS_ESP_ENABLED(state))
    {
      uint8_t rw_buffer = lsm6dso_read_wake_src(state);
      uint8_t wake_src = STM_LSM6DSO_REG_WAKE_SRC;
      if((rw_buffer & 0x08) &&
          state->current_conf.md_enabled)
        lsm6dso_handle_md_interrupt(this, irq_event->timestamp, &rw_buffer);
      lsm6dso_handle_esp_interrupt(this, irq_event->timestamp, &wake_src, &rw_buffer);
    }
    lsm6dso_handle_ois_interrupt(this, irq_event->timestamp);
  }
#else
  UNUSED_VAR(this);
  UNUSED_VAR(event_msg);
  UNUSED_VAR(irq_event);
#endif
  return SNS_RC_SUCCESS;
}
#endif

#define LSM6DSO_IS_INT_LEVEL_TRIG(x) (((x) == SNS_INTERRUPT_TRIGGER_TYPE_HIGH) || \
                                      ((x) == SNS_INTERRUPT_TRIGGER_TYPE_LOW) ? true : false)

void lsm6dso_send_interrupt_is_cleared_msg(sns_sensor_instance *const this, sns_data_stream* data_stream)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  // clear msg is not needed for edge triggered and dae
  if(!LSM6DSO_IS_INT_LEVEL_TRIG(state->irq_info.irq_config.interrupt_trigger_type) ||
      lsm6dso_dae_if_available(this) || !data_stream)
    return;

  sns_request irq_msg =
  {
    .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_IS_CLEARED,
    .request    = NULL
  };

  data_stream->api->send_request(data_stream, &irq_msg);
  DBG_INST_PRINTF_EX(HIGH, this, "interrupt clear msg sent");
}

void lsm6dso_clear_interrupt_q(sns_sensor_instance *const instance,
    sns_data_stream* interrupt_data_stream)
{
  UNUSED_VAR(instance);
  if((interrupt_data_stream == NULL) || lsm6dso_dae_if_available(instance))
    return;
  sns_sensor_event *event;
  uint16_t num_events = 0;
  event = interrupt_data_stream->api->peek_input(interrupt_data_stream);
  while((NULL != event) && ((SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT != event->message_id)))
  {
    event = interrupt_data_stream->api->get_next_input(interrupt_data_stream);
    num_events++;
  }
  DBG_INST_PRINTF(HIGH, instance, "clean interrupt q events = %d", num_events);
  if(num_events)
    lsm6dso_send_interrupt_is_cleared_msg(instance, interrupt_data_stream);
}


#if !LSM6DSO_DAE_ENABLED
uint16_t lsm6dso_handle_interrupt(sns_sensor_instance *const instance,
    sns_data_stream* interrupt_data_stream,
    sns_rc (*interrupt_handler)(sns_sensor_instance *const , sns_time cur_time, uint16_t event_msg, sns_interrupt_event* latest_interrupt_event))
{
  sns_sensor_event *event;
  uint16_t num_events = 0;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  pb_istream_t stream;
  sns_rc rv = SNS_RC_SUCCESS;
  if(interrupt_data_stream == NULL)
    return num_events;
  event = interrupt_data_stream->api->peek_input(interrupt_data_stream);
  while(NULL != event)
  {
    if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id) {
      interrupt_handler(instance, sns_get_system_time(), SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT, &irq_event);

    }
    else if(SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id) {
      stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
      num_events++;
    }
    else
    {
    }
    event = interrupt_data_stream->api->get_next_input(interrupt_data_stream);
  }
  if(num_events) {
    if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
      rv = interrupt_handler(instance, sns_get_system_time(), SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT, &irq_event);
  }
  return num_events;
}

static void lsm6dso_handle_hw_interrupts(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  uint16_t event_cnt = 0;
  event_cnt = lsm6dso_handle_interrupt(this, state->interrupt_data_stream,
      &lsm6dso_handle_int1_event);

  if(event_cnt && !state->ascp_req_count)
    lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt_data_stream);

  event_cnt = lsm6dso_handle_interrupt(this, state->interrupt2_data_stream,
      &lsm6dso_handle_int2_event);
  if(event_cnt)
    lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt2_data_stream);
}

/** this function executes for handling fifo read data only
 * driver make sure to have only one ascp com port req at a time*/
static void lsm6dso_handle_ascp_events(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
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
      sns_ascp_for_each_vector_do(&stream, lsm6dso_process_com_port_vector, (void *)this);

      if(state->flushing_sensors != 0)
      {
        lsm6dso_send_fifo_flush_done(this, state->flushing_sensors, FLUSH_DONE_AFTER_DATA);
        state->flushing_sensors = 0;
        lsm6dso_restart_hb_timer(this, false);
      }
      lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt_data_stream);
      port_rw_events_rcvd++;
    }
    event = data_stream->api->get_next_input(data_stream);
  }

  if(port_rw_events_rcvd > 0)
  {
    if(port_rw_events_rcvd > 1)
    {
      LSM6DSO_INST_DEBUG_TS(MED, this, "#rw_evs=%u", port_rw_events_rcvd);
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

    LSM6DSO_INST_DEBUG_TS(MED, this,
        "recheck fifo int cur_t=%u last=%u elapsed=%d",
        (uint32_t)cur_time,(uint32_t)(state->fifo_info.last_timestamp), (int32_t)(cur_time - state->fifo_info.last_timestamp));

    state->fifo_info.th_info.recheck_int = true;
    if(state->fifo_info.th_info.flush_req || state->fifo_info.th_info.interrupt_fired)
      state->fifo_info.th_info.recheck_int = false;

    if(state->fifo_info.reconfig_req)
      state->fifo_info.th_info.flush_req = true;
    lsm6dso_read_fifo_data(this, cur_time, false);

    //still req reconfig, do it here, hard reconfig
    if(state->fifo_info.reconfig_req && state->ascp_req_count == 0)
    {
      lsm6dso_reconfig_fifo(this, false);
      lsm6dso_reconfig_esp(this);
    }
  }
}
#endif
static sns_rc lsm6dso_handle_heart_attack_timer_events(
  sns_sensor_instance *const this,
  sns_timer_sensor_event *timer_event)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_health *health = &state->health;
  sns_rc rv = SNS_RC_SUCCESS;

  if(((state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) ||
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
    lsm6dso_restart_hb_timer(this, true);
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
    health->expected_expiration = sns_get_system_time()  + health->heart_beat_timeout;
    return SNS_RC_SUCCESS;
  }

  health->heart_attack = true;
  health->heart_attack_cnt++;
  rv = SNS_RC_SUCCESS;
  if(health->heart_attack_cnt >= MAX_HEART_ATTACKS) {
    lsm6dso_inst_exit_island(this);
    SNS_INST_PRINTF(ERROR, this, "heart_attack: Max reset tried.");
    health->expected_expiration = UINT64_MAX;
    health->heart_beat_timeout = UINT64_MAX/2;
    lsm6dso_restart_hb_timer(this, true);
    health->heart_attack = false;
    lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt_data_stream);
    rv = SNS_RC_INVALID_STATE;
    health->heart_attack_cnt = 0;

  } else if(health->heart_attack_cnt >= MAX_HEART_ATTACKS/2) {
    // reset sensor Something is wrong. Reset device.
    // Reset Sensor
    lsm6dso_inst_exit_island(this);
    SNS_INST_PRINTF(ERROR, this, "heart_attack: Something wrong. Reseting Device!");
    lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt_data_stream);
    rv = lsm6dso_recover_device(this);
    //restart timer here even if this is periodic
    lsm6dso_restart_hb_timer(this, false);
    // increment till it reaches MAX_HEART_ATTACKS
    health->heart_attack_cnt++;
    health->heart_attack = false;
    rv = SNS_RC_NOT_AVAILABLE;
  }
  if(rv == SNS_RC_SUCCESS) {
    int64_t ts_diff;
    DBG_INST_PRINTF(LOW, this, "heart_attack: flushing FIFO");

    if(!lsm6dso_dae_if_available(this) && state->current_conf.md_enabled)
      lsm6dso_read_wake_src(state);

    lsm6dso_flush_fifo(this);
    sns_time cur_time = sns_get_system_time();
    ts_diff = cur_time - state->fifo_info.last_timestamp;
    if((!state->ascp_req_count) && (ts_diff < health->heart_beat_timeout)) {
      DBG_INST_PRINTF(LOW, this, "heart_attck: ts_diff=%d cur=%u last_ts=%u",ts_diff, (uint32_t)cur_time, (uint32_t)state->fifo_info.last_timestamp);
      state->health.heart_attack_cnt = 0;
      state->health.heart_attack = false;
      lsm6dso_send_interrupt_is_cleared_msg(this, state->interrupt_data_stream);
    }
    health->expected_expiration += health->heart_beat_timeout;
  }
  return rv;
}
sns_rc lsm6dso_handle_timer(
    sns_sensor_instance *const instance, sns_data_stream* timer_data_stream,
    sns_rc (*timer_handler)(sns_sensor_instance *const , sns_time, sns_timer_sensor_event* latest_timer_event))
{
  sns_sensor_event *event;
  uint16_t num_events = 0;
  sns_timer_sensor_event timer_event;
  sns_rc rv = SNS_RC_SUCCESS;
  if(timer_data_stream == NULL)
    return rv;
  event = timer_data_stream->api->peek_input(timer_data_stream);
  while(NULL != event)
  {
    pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
        event->event_len);
    if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
    {
      if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
      {
        num_events++;
      }
    }
    else
    {
    }
    event = timer_data_stream->api->get_next_input(timer_data_stream);
  }
  if(num_events) {
    rv = timer_handler(instance,sns_get_system_time(), &timer_event);
  }
  return rv;
}

static sns_rc lsm6dso_handle_temp_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(latest_timer_event);
  UNUSED_VAR(timestamp);
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  if(state->desired_conf.publish_sensors & LSM6DSO_SENSOR_TEMP
      &&
      state->sensor_temp_info.timer_is_active
      &&
      state->sensor_temp_info.sampling_intvl > 0)
  {
    lsm6dso_handle_sensor_temp_sample(this);
  }
  else
  {
    sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_sensor_temp_data_stream);
    state->sensor_temp_info.timer_is_active = false;
  }
  return rv;
}

static sns_rc lsm6dso_handle_selftest_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(latest_timer_event);
  UNUSED_VAR(timestamp);
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  if(state->common_info.mode & LSM6DSO_MODE_SELF_TEST)
  {
    lsm6dso_inst_exit_island(this);
    if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
      lsm6dso_inst_hw_self_test(this);
    else if(state->self_test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
      lsm6dso_inst_factory_self_test(this);
  }
  return rv;
}

static sns_rc lsm6dso_handle_md_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(latest_timer_event);
  UNUSED_VAR(timestamp);
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  if(state->desired_conf.md_enabled && state->md_info.is_timer_running)
  {
    DBG_INST_PRINTF_EX(HIGH, this, "Enable MD filter_settled");
    state->md_info.is_timer_running = false;
    lsm6dso_update_md_filter(this);
  }
  if(state->md_info.filter == HP_FILTER || !lsm6dso_is_md_int_required(this))  
  {
    state->md_info.is_timer_running = false;
  }
  return rv;
}

static sns_rc lsm6dso_handle_heart_beat_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  sns_rc rv = SNS_RC_SUCCESS;
  UNUSED_VAR(timestamp);
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  if(!state->self_test_info.test_alive) //Skip heart attack for  self-test case
    rv = lsm6dso_handle_heart_attack_timer_events(this, latest_timer_event);
  return rv;
}
sns_rc lsm6dso_handle_timer_events(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  lsm6dso_handle_timer(this, state->timer_sensor_temp_data_stream,
      &lsm6dso_handle_temp_timer_event);

  lsm6dso_handle_timer(this, state->timer_self_test_data_stream,
      &lsm6dso_handle_selftest_timer_event);

  lsm6dso_handle_timer(this, state->timer_md_data_stream,
      &lsm6dso_handle_md_timer_event);

  rv = lsm6dso_handle_timer(this, state->timer_heart_beat_data_stream,
      &lsm6dso_handle_heart_beat_timer_event);

  lsm6dso_handle_timer(this, state->timer_polling_data_stream,
      &lsm6dso_handle_polling_timer_event);

  return rv;
}

static void lsm6dso_report_error(sns_sensor_instance *this, sns_sensor_uid const *suid)
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

static void lsm6dso_process_flush_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_sensor_type flushing_sensor = *((lsm6dso_sensor_type*)client_request->request);
  sns_time now = sns_get_system_time();
  if(state->flushing_sensors != 0)
  {
    // Flush already started
    state->flushing_sensors |= flushing_sensor;
  }
  else if((state->fifo_info.last_timestamp + state->fifo_info.avg_sampling_intvl) > now)
  {
    // there're no samples to flush
    lsm6dso_send_fifo_flush_done(this, flushing_sensor, FLUSH_DONE_FIFO_EMPTY);
  }
  else if(state->fifo_info.cur_wmk == 1 &&
          (!lsm6dso_dae_if_available(this) || state->fifo_info.max_requested_wmk <= 1))
  {
    // ignore flush req if wmk=1
    lsm6dso_send_fifo_flush_done(this, flushing_sensor, FLUSH_DONE_FIFO_EMPTY);
  }
  /* DAE usecase: flush DAE data only if hw_wmk=1 and dae_wmk > 1 */
  else if(state->fifo_info.cur_wmk == 1 &&
      state->fifo_info.max_requested_wmk > state->fifo_info.cur_wmk && 
      lsm6dso_dae_if_available(this)) {
      state->flushing_sensors |= flushing_sensor;
      DBG_INST_PRINTF_EX(MED, this, "flush samples: s=0x%x wmk:%d dae_wmk=%d",
          flushing_sensor, state->fifo_info.cur_wmk, state->fifo_info.max_requested_wmk);
      lsm6dso_dae_if_flush_samples(this);
  } else {
    sns_time next_int;
    state->flushing_sensors |= flushing_sensor;
    next_int = (state->fifo_info.bh_info.interrupt_ts + 
                state->fifo_info.avg_interrupt_intvl);
    if(state->fifo_info.max_requested_wmk > state->fifo_info.cur_wmk  || // DAE WM bigger than FIFO WM.
       next_int < now || // flush-only request or missed interrupt
       next_int > now + state->fifo_info.avg_sampling_intvl )
    {
      lsm6dso_flush_fifo(this);
    }
    // else be patient, interrupt will fire soon
  }
}

static void lsm6dso_process_cal_reset_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  sns_lsm6dso_registry_cfg const *src_cal =
    (sns_lsm6dso_registry_cfg const *)client_request->request;
  sns_lsm6dso_registry_cfg *dest_cal = (src_cal->sensor_type == LSM6DSO_ACCEL) ?
    &state->accel_registry_cfg : &state->gyro_registry_cfg;
  lsm6dso_copy_calibration_info(dest_cal, src_cal);
  lsm6dso_send_cal_event(this, src_cal->sensor_type);
}

bool lsm6dso_is_md_int_required(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  bool md_int_req = false;
  //md client is present and non-gated request is not present
  if(state->md_info.client_present &&
      !(state->desired_conf.publish_sensors & LSM6DSO_ACCEL))
    md_int_req = true;

  //md is required if any internal client is requested
  md_int_req = (md_int_req || state->md_info.internal_client_present);

  return md_int_req;
}


static sns_rc lsm6dso_parse_config_info(
  sns_sensor_instance     *const this,
  lsm6dso_instance_config *inst_cfg)
{
  sns_rc rc = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint8_t desired_rate_idx = 0;
  uint32_t curr_max_requested_wmk = state->fifo_info.max_requested_wmk;

  state->config_sensors |= inst_cfg->config_sensors;

  if(inst_cfg->fifo_enable & (LSM6DSO_ACCEL | LSM6DSO_GYRO))
  {
    desired_rate_idx = lsm6dso_get_odr_rate_idx(inst_cfg->sample_rate);
    desired_rate_idx = SNS_MAX(desired_rate_idx, state->min_odr_idx);
  }

  if(desired_rate_idx < lsm6dso_odr_map_len)
  {
    float desired_report_rate = inst_cfg->report_rate;
    float dae_report_rate     = inst_cfg->dae_report_rate;
    state->fifo_info.fifo_enabled       = inst_cfg->fifo_enable;
    state->desired_conf.publish_sensors    = inst_cfg->client_present & ~LSM6DSO_MOTION_DETECT;
    //if gated client present and no MD request present
    //gated request becomes non-gated
    if(inst_cfg->gated_client_present && !(inst_cfg->client_present & LSM6DSO_MOTION_DETECT))
    {
      state->desired_conf.publish_sensors |= inst_cfg->gated_client_present;
    }
    state->accel_info.gated_client_present = (inst_cfg->gated_client_present != 0);
    state->desired_conf.odr_idx         = desired_rate_idx;
    state->desired_conf.odr             = lsm6dso_odr_map[desired_rate_idx].odr;
    state->desired_conf.enabled_sensors = (state->fifo_info.fifo_enabled &
                                           (LSM6DSO_ACCEL | LSM6DSO_GYRO));

    state->passive_client_present      = inst_cfg->passive_client_present;
    state->md_info.client_present      = (inst_cfg->client_present & LSM6DSO_MOTION_DETECT);
    state->desired_conf.md_enabled     = lsm6dso_is_md_int_required(this);

    desired_report_rate = SNS_MIN(desired_report_rate, state->desired_conf.odr);
    dae_report_rate     = SNS_MIN(dae_report_rate, state->desired_conf.odr);

    if(inst_cfg->gated_client_present)
    {
      if(!state->fifo_info.new_config.gated_timestamp)//Note the time when first gated request comes
        state->fifo_info.new_config.gated_timestamp =  sns_get_system_time();
      else if(!(state->desired_conf.enabled_sensors & LSM6DSO_ACCEL)//note the time when non gated request become gated
            && (state->current_conf.enabled_sensors & LSM6DSO_ACCEL))
        state->fifo_info.new_config.gated_timestamp =  sns_get_system_time();
    }
    else
    {
      //when gated request goes away
      state->fifo_info.new_config.gated_timestamp = 0;
    }
    // calculate FIFO WM
    if(LSM6DSO_ODR_0 < state->desired_conf.odr && 0.0f < desired_report_rate)
    {
      uint16_t max_fifo = LSM6DSO_MAX_FIFO;
      if(state->desired_conf.enabled_sensors & LSM6DSO_GYRO &&
         (max_fifo > (LSM6DSO_HW_MAX_FIFO >> 1)))
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
          (uint16_t)(state->desired_conf.odr/desired_report_rate);
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
    else if(LSM6DSO_ODR_0 == state->desired_conf.odr)
    {
      state->desired_conf.wmk = 0;
    }
    else
    {
      state->desired_conf.wmk = 1;
    }

    // calculate DAE WM
    if(FLT_MIN < dae_report_rate &&
       ((float)state->desired_conf.odr / (float)UINT32_MAX) < dae_report_rate)
    {
      state->fifo_info.max_requested_wmk =
       (uint32_t)(state->desired_conf.odr / dae_report_rate);
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
    if(state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO) &&
       !state->desired_conf.max_requested_flush_ticks &&
       state->current_conf.odr > LSM6DSO_ODR_0)
    {
      DBG_INST_PRINTF(LOW, this, "config: resetting last_timestamp");
      state->fifo_info.last_timestamp = sns_get_system_time();
      lsm6dso_dae_if_flush_hw(this);
    }

    state->desired_conf.max_requested_flush_ticks = inst_cfg->flush_period_ticks * 1.1f;
    DBG_INST_PRINTF(LOW, this, "config: publish=0x%x dae_wm=%u flush_per=%u",
                    state->desired_conf.publish_sensors, state->fifo_info.max_requested_wmk,
                    (uint32_t)state->desired_conf.max_requested_flush_ticks);

    state->fifo_info.reconfig_req = false;
    state->fifo_info.full_reconf_req = false;

    if((state->desired_conf.wmk != state->current_conf.wmk) ||
       (state->desired_conf.odr != state->current_conf.odr) ||
       (state->desired_conf.enabled_sensors != state->current_conf.enabled_sensors))
    {
      state->fifo_info.reconfig_req = true;
    }
    else if(LSM6DSO_IS_ESP_CONF_CHANGED(state))
    {
      state->fifo_info.reconfig_req = true;
    }

    if(state->desired_conf.enabled_sensors == 0 && state->current_conf.enabled_sensors == 0)
    {
      state->fifo_info.reconfig_req = false;
    }
    else if(LSM6DSO_IS_ESP_CONF_CHANGED(state))
    {
      state->fifo_info.full_reconf_req = true;
      state->fifo_info.reconfig_req = true;
    }

    if(!state->fifo_info.reconfig_req &&
       curr_max_requested_wmk != state->fifo_info.max_requested_wmk)
    {
      lsm6dso_update_heartbeat_monitor(this);
    }

    //If odr, wmk, sensors all same as before (reconfig_req = false)
    //but flush_period is changed
    //case: if first request is with flush_period = 0 (DAE discards data instead of pushing to SEE)
    //and new request has non-zero flush_period
    bool will_stream = ((state->desired_conf.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) &&
        (!lsm6dso_dae_if_available(this) || inst_cfg->flush_period_ticks));

    if(!state->fifo_info.reconfig_req &&
        !state->fifo_info.is_streaming &&
        will_stream) {
      state->fifo_info.last_ts_valid = false; //set last ts as inaccurate
      state->fifo_info.is_streaming = will_stream;
      //move last ts, useful when flush comes right after this
      state->fifo_info.last_timestamp = sns_get_system_time();
    }

    DBG_INST_PRINTF(MED, this, "[%u] config: have(i,w,r,es,ps,md) %u,%u,%u,0x%x,0x%x,%u",
                    state->hw_idx, state->current_conf.odr_idx,
                    state->current_conf.wmk, state->current_conf.odr,
                    state->current_conf.enabled_sensors,
                    state->current_conf.publish_sensors,
                    state->current_conf.md_enabled);
    DBG_INST_PRINTF(MED, this, "[%u] config: want(i,w,r,es,ps,md) %u,%u,%u,0x%x,0x%x,%u",
                    state->hw_idx, state->desired_conf.odr_idx,
                    state->desired_conf.wmk, state->desired_conf.odr,
                    state->desired_conf.enabled_sensors,
                    state->desired_conf.publish_sensors,
                    state->desired_conf.md_enabled);
  }
  else
  {
    if(inst_cfg->fifo_enable & LSM6DSO_ACCEL)
    {
      rc = SNS_RC_FAILED;
      SNS_INST_PRINTF(ERROR, this, "Sending error report for ACCEL");
      lsm6dso_report_error(this, &state->accel_info.suid);
    }
    if(inst_cfg->fifo_enable & LSM6DSO_GYRO)
    {
      rc = SNS_RC_FAILED;
      SNS_INST_PRINTF(ERROR, this, "Sending error report for GYRO");
      lsm6dso_report_error(this, &state->gyro_info.suid);
    }
  }

  if(inst_cfg->config_sensors & LSM6DSO_SENSOR_TEMP)
  {
    if(inst_cfg->client_present & LSM6DSO_SENSOR_TEMP)
    {
      rc = lsm6dso_validate_sensor_temp_odr(state, inst_cfg);
      if(rc == SNS_RC_SUCCESS)
      {
        DBG_INST_PRINTF(MED, this, "config: ST(RR*1000/SR)=%u/%u",
                        (uint32_t)(state->sensor_temp_info.report_rate_hz*1000),
                        (uint32_t)state->sensor_temp_info.desired_sampling_rate_hz);
        lsm6dso_send_config_event(this, false);
      }
      else
      {
        SNS_INST_PRINTF(ERROR, this, "Sending error report for TEMP");
        lsm6dso_report_error(this, &state->sensor_temp_info.suid);
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

static void lsm6dso_process_sensor_config_request(
  sns_sensor_instance *const this,
  sns_request const *client_request)
{
  // 1. Extract sample, report rates from client_request.
  // 2. Configure sensor HW.
  // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
  // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
  // 5. Save the current config information like type, sample_rate, report_rate, etc.
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_instance_config *payload = (lsm6dso_instance_config*)client_request->request;

  if(state->self_test_info.reconfig_postpone) {
    payload->config_sensors |= state->config_sensors;
  }

  DBG_INST_PRINTF_EX(HIGH, this, "config_sensors = 0x%x", payload->config_sensors);
  //check if any bits set in config_sensors
  if(payload->config_sensors & LSM6DSO_OIS) {
    //configure ois not needed to follow normal path
    lsm6dso_config_ois(this);
  }
  //check if any other sensors are configuring
  if(!(payload->config_sensors & ~LSM6DSO_OIS)) {
    return;
  }

  sns_rc rc = lsm6dso_parse_config_info(this, payload);
  if(SNS_RC_SUCCESS == rc)
  {
    const odr_reg_map *map_entry = &lsm6dso_odr_map[state->desired_conf.odr_idx];

    DBG_INST_PRINTF(HIGH, this, "client_config: reconfig=%u", state->fifo_info.reconfig_req);
    // Reset ACCEL/GYRO bits of config sensors, If there is no change in configuration
    if(!state->fifo_info.reconfig_req && !lsm6dso_dae_if_available(this))
    {
      state->config_sensors &= ~(LSM6DSO_ACCEL | LSM6DSO_GYRO);
    }
    if(state->accel_info.gated_client_present)
    {
      DBG_INST_PRINTF_EX(HIGH, this, "gated accel config");

      lsm6dso_set_gated_accel_config(state,
          state->desired_conf.wmk,
          map_entry->accel_odr_reg_value,
          state->fifo_info.fifo_enabled);
    }

    if(state->current_conf.odr > 0.0f || 
       state->accel_info.gated_client_present ||
       state->sensor_temp_info.cur_sampling_rate_hz > 0.0f ||
       ((state->passive_client_present & (LSM6DSO_ACCEL | LSM6DSO_GYRO)) && !state->current_conf.odr) ||
       ((state->passive_client_present & LSM6DSO_SENSOR_TEMP) && !state->sensor_temp_info.cur_sampling_rate_hz)
      ) 
    {
      // new clients need to know ODR/WM for samples received between now and
      // when the next config takes effect
      lsm6dso_send_config_event(this, true);
    }
    else
    {
      sns_memzero(&state->fifo_info.last_sent_config, sizeof(state->fifo_info.last_sent_config));
    }

    if(state->fifo_info.reconfig_req)
    {
      lsm6dso_set_fifo_config(this,
          state->desired_conf.wmk,
          map_entry->accel_odr_reg_value,
          map_entry->gyro_odr_reg_value,
          state->fifo_info.fifo_enabled);
    }

    if(!state->self_test_info.test_alive) {
#if LSM6DSO_DAE_ENABLED
      if(state->config_step == LSM6DSO_CONFIG_IDLE)
      {
        bool stopping = false;
        if(state->fifo_info.reconfig_req)
        {
          lsm6dso_disable_fifo_intr(this);
          lsm6dso_sensor_type sensors = 
            (state->config_sensors | state->current_conf.enabled_sensors) & ~LSM6DSO_SENSOR_TEMP;
          stopping |= lsm6dso_dae_if_stop_streaming(this, sensors);
        }
        if(state->sensor_temp_info.desired_sampling_rate_hz !=
           state->sensor_temp_info.cur_sampling_rate_hz)
        {
          stopping |= lsm6dso_dae_if_stop_streaming(this, LSM6DSO_SENSOR_TEMP);
        }
        if(stopping)
        {
          state->config_step = LSM6DSO_CONFIG_STOPPING_STREAM;
        }
      }
      if(state->config_step == LSM6DSO_CONFIG_IDLE)
#endif
      {
        lsm6dso_reconfig_hw(this);
        lsm6dso_dump_reg(this, state->fifo_info.fifo_enabled);
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
static sns_rc lsm6dso_inst_notify_event(sns_sensor_instance *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  lsm6dso_dae_if_process_events(this);
#if !LSM6DSO_DAE_ENABLED
  lsm6dso_handle_ascp_events(this);
  lsm6dso_handle_hw_interrupts(this);
#endif
  rv = lsm6dso_handle_esp_timer_events(this);
  rv = lsm6dso_handle_timer_events(this);

  lsm6dso_turn_on_bus_power(state, false);
  return rv;
}

/** See sns_sensor_instance_api::set_client_config */
static sns_rc lsm6dso_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  if(client_request->message_id != SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    SNS_INST_PRINTF(MED, this, "[%u] client_config: msg=%u #samples=%u/%u/%u/%u",
                    state->hw_idx, client_request->message_id, state->accel_sample_counter,
                    state->gyro_sample_counter, state->num_temp_samples, state->num_md_ints);
  }

  LSM6DSO_INST_AUTO_DEBUG_PRINTF(HIGH, this, "set_client_config: msg=%u", client_request->message_id);

  if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
    lsm6dso_process_sensor_config_request(this, client_request);
  }
  else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    lsm6dso_process_flush_request(this, client_request);
  }
  else if(client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    /** All self-tests are handled in normal mode. */
    lsm6dso_inst_exit_island(this);
    lsm6dso_set_client_test_config(this, client_request);
  }
  else if(client_request->message_id == SNS_CAL_MSGID_SNS_CAL_RESET)
  {
    lsm6dso_process_cal_reset_request(this, client_request);
  }

  lsm6dso_turn_on_bus_power(state, false);
  return SNS_RC_SUCCESS;
}

void lsm6dso_copy_calibration_info(
  sns_lsm6dso_registry_cfg *dest_cal,
  sns_lsm6dso_registry_cfg const *src_cal)
{
  sns_memscpy(&dest_cal->fac_cal_bias, sizeof(dest_cal->fac_cal_bias),
              src_cal->fac_cal_bias, sizeof(src_cal->fac_cal_bias));

  sns_memscpy(&dest_cal->fac_cal_corr_mat, sizeof(dest_cal->fac_cal_corr_mat),
              &src_cal->fac_cal_corr_mat, sizeof(src_cal->fac_cal_corr_mat));

  sns_memscpy(&dest_cal->thermal_scale, sizeof(dest_cal->thermal_scale),
              &src_cal->thermal_scale, sizeof(src_cal->thermal_scale));

  dest_cal->mean_temp = src_cal->mean_temp;

  dest_cal->registry_persist_version = src_cal->registry_persist_version;

  dest_cal->ts = src_cal->ts;
}


/** Public Data Definitions. */

sns_sensor_instance_api lsm6dso_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &lsm6dso_inst_init,
  .deinit            = &lsm6dso_inst_deinit,
  .set_client_config = &lsm6dso_inst_set_client_config,
  .notify_event      = &lsm6dso_inst_notify_event
};

