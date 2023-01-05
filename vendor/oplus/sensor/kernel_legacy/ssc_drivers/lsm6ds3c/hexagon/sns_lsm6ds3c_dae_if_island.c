/**
 * @file sns_lsm6ds3c_dae_if_island.c
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

#include "sns_types.h"
#include "sns_lsm6ds3c_sensor.h"
#include "sns_lsm6ds3c_sensor_instance.h"

#if LSM6DS3C_DAE_ENABLED

#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_service_manager.h"
#include "sns_sensor_util.h"
#include "sns_stream_service.h"
#include "sns_time.h"

#include "sns_lsm6ds3c_hal.h"

#include "sns_dae.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"

#define META_DATA_SIZE 8
#if !LSM6DS3C_DEBUG_TS
#define PRINTABLE_BATCH_SIZE 100
#else
#define PRINTABLE_BATCH_SIZE 1
#endif
/*======================================================================================
  Extern
  ======================================================================================*/
extern const odr_reg_map lsm6ds3c_odr_map[];
static bool flush_samples(sns_sensor_instance *this, lsm6ds3c_dae_stream *dae_stream);

/*======================================================================================
  Helper Functions
  ======================================================================================*/
static bool stream_usable(lsm6ds3c_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

/* ------------------------------------------------------------------------------------ */
static void build_static_config_request(
  lsm6ds3c_instance_config const *inst_cfg,
  sns_dae_set_static_config     *config_req,
  uint8_t                       hw_idx,
  uint8_t                       rigid_body_type,
  bool                          for_ag)
{
  sns_com_port_config const *com_config  = &inst_cfg->com_port_info.com_config;
  sns_async_com_port_config *ascp_config = &config_req->ascp_config;

  ascp_config->bus_type             = (sns_async_com_port_bus_type)com_config->bus_type;
  ascp_config->slave_control        = com_config->slave_control;
  ascp_config->reg_addr_type        = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  ascp_config->min_bus_speed_kHz    = com_config->min_bus_speed_KHz;
  ascp_config->max_bus_speed_kHz    = com_config->max_bus_speed_KHz;
  ascp_config->bus_instance         = com_config->bus_instance;

  if(for_ag)
  {
    triaxis_conversion const *src_axis_map = inst_cfg->axis_map;
    sensor_to_phone_conversion *dest_axis_map = config_req->accel_info.axis_map;

    sns_strlcpy(
      config_req->func_table_name,
      (hw_idx == 0) ? "lsm6ds3c_fifo_hal_table" : "lsm6ds3c_fifo_hal_table2",
      sizeof(config_req->func_table_name));
    config_req->interrupt              = 1;
    config_req->has_irq_config         = true;
    config_req->irq_config             = inst_cfg->irq_config;

    config_req->has_accel_info         = true;
    config_req->accel_info.accel_range =
      lsm6ds3c_accel_range_max[inst_cfg->accel_resolution_idx];
    config_req->accel_info.axis_map_count = ARR_SIZE(config_req->accel_info.axis_map);
    for(uint32_t i = 0; i < config_req->accel_info.axis_map_count; i++)
    {
      dest_axis_map[i].ipaxis = src_axis_map[i].ipaxis;
      dest_axis_map[i].opaxis = src_axis_map[i].opaxis;
      dest_axis_map[i].invert = src_axis_map[i].invert;
    }
#if LSM6DS3C_DUAL_SENSOR_ENABLED
    // Populate Additional Attributes
    config_req->accel_info.accel_attr[0].value.has_sint = true;
    config_req->accel_info.accel_attr[0].value.sint = rigid_body_type;
    config_req->accel_info.accel_attr[0].attr_id = SNS_STD_SENSOR_ATTRID_RIGID_BODY;
    config_req->accel_info.accel_attr_count = 1;
#else
    UNUSED_VAR(rigid_body_type);
#endif
  }
  else // for temperature
  {
    sns_strlcpy(
      config_req->func_table_name,
      (hw_idx == 0) ? "lsm6ds3c_temperature_hal_table" : "lsm6ds3c_temperature_hal_table2",
      sizeof(config_req->func_table_name));
    config_req->interrupt             = 0;
    config_req->has_irq_config        = false;
    config_req->has_accel_info        = false;
  }

}

/* ------------------------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------------------------ */
static bool send_ag_config(lsm6ds3c_dae_stream *dae_stream, sns_sensor_instance *this)
{
  bool cmd_sent = false;
  bool flush_data = false;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  sns_accel_dynamic_info *accel_info;
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };
  uint16_t cur_wmk = state->fifo_info.cur_wmk;

  if(cur_wmk > 0)
  {
    if(dae_stream->dae_wm > 1 && dae_stream->dae_wm < UINT32_MAX &&
       state->fifo_info.max_requested_wmk > dae_stream->dae_wm)
    {
      flush_data = true;
    }
    dae_stream->dae_wm = state->fifo_info.max_requested_wmk;

    config_req.dae_watermark            = state->fifo_info.max_requested_wmk;
    if((state->accel_info.desired_odr > LSM6DS3C_ACCEL_ODR_OFF)
       &&
       (state->fifo_info.fifo_enabled & LSM6DS3C_ACCEL)
       &&
       (state->gyro_info.desired_odr > LSM6DS3C_GYRO_ODR_OFF)
       &&
       (state->fifo_info.fifo_enabled & LSM6DS3C_GYRO))
    {
      if(config_req.dae_watermark <= (UINT32_MAX >> 1))
      {
        // Double DAE watermark if both accel & gyro are enabled.
        config_req.dae_watermark <<= 1;
      }
      cur_wmk <<= 1;
    }
    config_req.data_age_limit_ticks   = state->fifo_info.max_requested_flush_ticks;
  }
  else // MD only
  {
    if(dae_stream->dae_wm > 0)
    {
      flush_data = true; // discard old samples
    }
    dae_stream->dae_wm = config_req.dae_watermark = 0;
    config_req.data_age_limit_ticks   = 0;
  }

  config_req.has_data_age_limit_ticks = true;
  config_req.has_polling_config       = false;

  config_req.has_accel_info    = true;
  accel_info                   = &config_req.accel_info;
  accel_info->odr              = lsm6ds3c_get_accel_odr(state->common_info.accel_curr_odr);
  accel_info->offset_cal_count = 3;
  accel_info->offset_cal[0]    = state->accel_registry_cfg.fac_cal_bias[0];
  accel_info->offset_cal[1]    = state->accel_registry_cfg.fac_cal_bias[1];
  accel_info->offset_cal[2]    = state->accel_registry_cfg.fac_cal_bias[2];
  accel_info->scale_cal_count  = 3;
  accel_info->scale_cal[0]     = state->accel_registry_cfg.fac_cal_corr_mat.e00;
  accel_info->scale_cal[1]     = state->accel_registry_cfg.fac_cal_corr_mat.e11;
  accel_info->scale_cal[2]     = state->accel_registry_cfg.fac_cal_corr_mat.e22;

  if(state->common_info.odr_changed & LSM6DS3C_ACCEL)
  {
    accel_info->num_initial_invalid_samples =
      lsm6ds3c_odr_map[state->current_conf.odr_idx].accel_discard_samples;
  }
  config_req.has_expected_get_data_bytes = true;
  /* DAE driver reads 8 bytes for meta data, plus the FIFO */
  if(cur_wmk > 0)
  {
    config_req.expected_get_data_bytes   =
      cur_wmk * STM_LSM6DS3C_FIFO_SAMPLE_SIZE + META_DATA_SIZE;
  }

  DBG_INST_PRINTF(MED, this, "send_ag_config: age_limit=%u wm=%x #bytes=%u #discard=%u",
                  (uint32_t)config_req.data_age_limit_ticks, config_req.dae_watermark,
                  config_req.expected_get_data_bytes,
                  accel_info->num_initial_invalid_samples);

  if(state->fifo_info.max_requested_wmk * state->fifo_info.nominal_sampling_intvl >
     config_req.data_age_limit_ticks)
  {
    state->fifo_info.last_ts_valid = false;
  }

  if((req.request_len =
      pb_encode_request(encoded_msg,
                        sizeof(encoded_msg),
                        &config_req,
                        sns_dae_set_streaming_config_fields,
                        NULL)) > 0)
  {
    if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
    {
      dae_stream->state = STREAM_STARTING;
      cmd_sent = true;
    }
  }
  if(flush_data)
  {
    flush_samples(this, dae_stream);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool send_temp_config(
  lsm6ds3c_dae_stream       *dae_stream,
  sns_sensor_instance      *this)
{
  bool cmd_sent = false;
  sns_time time_now = sns_get_system_time();
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_sensor_temp_info *temp_info = &state->sensor_temp_info;
  uint32_t dae_wm = (uint32_t)(temp_info->desired_sampling_rate_hz / temp_info->report_rate_hz);
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_zero;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };

  config_req.dae_watermark = dae_wm;
  config_req.has_data_age_limit_ticks = true;
  config_req.data_age_limit_ticks     = state->fifo_info.max_requested_flush_ticks;
  config_req.has_polling_config       = true;
  config_req.polling_config.polling_interval_ticks = temp_info->sampling_intvl;
  config_req.polling_config.polling_offset =
    (time_now + temp_info->sampling_intvl) / temp_info->sampling_intvl *
    temp_info->sampling_intvl;

  config_req.has_expected_get_data_bytes = true;
  /* DAE temp driver reads status register, and 2 bytes for temp */
  config_req.expected_get_data_bytes     = 3;

  DBG_INST_PRINTF_EX(MED, this,
                 "send_temp_config: SR=%u RR=%u WM=%u now=%x %08x",
                 (uint32_t)temp_info->desired_sampling_rate_hz,
                 (uint32_t)temp_info->report_rate_hz, dae_wm,
                 (uint32_t)(time_now >> 32), (uint32_t)time_now);
  DBG_INST_PRINTF_EX(MED, this,
                 "send_temp_config: wm=%u age=%x invtl=%u offset=%x %08x",
                 config_req.dae_watermark, (uint32_t)config_req.data_age_limit_ticks,
                 (uint32_t)config_req.polling_config.polling_interval_ticks,
                 (uint32_t)(config_req.polling_config.polling_offset >> 32),
                 (uint32_t)config_req.polling_config.polling_offset);

  if((req.request_len =
      pb_encode_request(encoded_msg,
                        sizeof(encoded_msg),
                        &config_req,
                        sns_dae_set_streaming_config_fields,
                        NULL)) > 0)
  {
    if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
    {
      dae_stream->state = STREAM_STARTING;
      cmd_sent = true;
      temp_info->cur_sampling_rate_hz = temp_info->desired_sampling_rate_hz;
    }
    else
    {
      SNS_INST_PRINTF(ERROR, this,
                     "send_temp_config: req fail on stream %x",
                     dae_stream->stream);
    }
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_hw(lsm6ds3c_dae_stream *dae_stream)
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
static bool flush_samples(sns_sensor_instance *this, lsm6ds3c_dae_stream *dae_stream)
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
  DBG_INST_PRINTF(MED, this, "flush_samples: sent=%u", cmd_sent);
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool stop_streaming(lsm6ds3c_dae_stream *dae_stream)
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
static void process_fifo_samples(
  sns_sensor_instance *this,
  sns_time            timestamp,
#if LSM6DS3C_DAE_TIMESTAMP_TYPE
  sns_dae_timestamp_type ts_type,
#endif
  const uint8_t       *buf,
  size_t              buf_len)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  uint16_t meta_data_size = META_DATA_SIZE;
  bool gyro_enabled = ((buf[2] & STM_LSM6DS3C_FIFO_DEC_G_MASK) != 0);
  uint8_t sample_size = STM_LSM6DS3C_FIFO_SAMPLE_SIZE * (gyro_enabled ? 2 : 1);

  //Check for pattern position
  uint16_t pattern_pos = (((buf[5] & 0x03) << 8) | (buf[4] & 0xFF));
  pattern_pos  %= (sample_size /2);
  
  /*In nano driver, if pattern pos is non-zero, we may read partial samples to align pattern pos.
    so those partial samples ultimately will be dropped here*/
  if(pattern_pos)
  {
    uint8_t bytes_to_discard = sample_size - pattern_pos*2;
    meta_data_size += bytes_to_discard;
  }
  const uint8_t *fifo_start = buf + meta_data_size;
  uint16_t num_sample_sets = (buf_len - meta_data_size) / sample_size;
  lsm6ds3c_accel_odr accel_odr = (lsm6ds3c_accel_odr)(buf[6] & 0xF0);

  if(num_sample_sets >= 1 && accel_odr != LSM6DS3C_ACCEL_ODR_OFF)
  {
    /* fifo starts after FIFO_CTRL1, FIFO_CTRL2, FIFO_CTRL3, FIFO_STATUS2, FIFO_STATUS3, FIFO_STATUS4, CTRL1_XL, REG_CTRL2_G */
    uint16_t wm_l = buf[0] & STM_LSM6DS3C_FIFO_WTM_CTRL1_MASK;
    uint16_t wm_h = buf[1] & STM_LSM6DS3C_FIFO_WTM_CTRL2_MASK;
    uint16_t wm = ((wm_h << 8) & 0xFF00) | wm_l;
    wm /= STM_LSM6DS3C_SAMPLE_SIZE/2; //Convert word(2bytes) to sample count
    float sample_rate = (float)(13 << ((accel_odr >> 4) - 1));

    wm >>= gyro_enabled ? 1 : 0;
    if(wm          != state->fifo_info.last_sent_config.fifo_watermark ||
       sample_rate != state->fifo_info.last_sent_config.sample_rate)
    {
      state->fifo_info.new_config.fifo_watermark = wm;
      state->fifo_info.new_config.sample_rate    = sample_rate;
      lsm6ds3c_send_config_event(this, false);
    }
    state->fifo_info.th_info.interrupt_ts = timestamp;
#if LSM6DS3C_DAE_TIMESTAMP_TYPE
    state->fifo_info.bh_info.interrupt_fired = (ts_type == SNS_DAE_TIMESTAMP_TYPE_HW_IRQ);
#else
    state->fifo_info.bh_info.interrupt_fired = buf[3] & STM_LSM6DS3C_FIFO_WTM_STATUS_MASK;
    if(state->health.heart_attack_flush)
      state->fifo_info.bh_info.interrupt_fired = 0;
#endif
    state->fifo_info.bh_info.interrupt_ts = timestamp;
    state->fifo_info.bh_info.cur_time = timestamp;
    state->fifo_info.bh_info.flush_req = !state->fifo_info.bh_info.interrupt_fired;
    state->fifo_info.bh_info.wmk = wm;
    state->fifo_info.bh_info.accel_odr = accel_odr;

    if(wm >= PRINTABLE_BATCH_SIZE || /* let's not print every sample */
       (state->fifo_info.interrupt_cnt <= MAX_INTERRUPT_CNT &&
        ts_type == SNS_DAE_TIMESTAMP_TYPE_HW_IRQ))
    {
      DBG_INST_PRINTF(
        LOW, this, "[%u] [DAE]: int=%u #int=%u #samples=%u/%u/%u irq=%u last_irq=%u last=%u",
        state->hw_idx, state->fifo_info.bh_info.interrupt_fired, state->fifo_info.interrupt_cnt,
        num_sample_sets, state->accel_sample_counter, state->gyro_sample_counter,
        (uint32_t)timestamp, (uint32_t)state->fifo_info.interrupt_timestamp,
        (uint32_t)state->fifo_info.last_timestamp);
    }
    lsm6ds3c_send_fifo_data(this, fifo_start, buf_len-meta_data_size, gyro_enabled);
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_temp_samples(
  sns_sensor_instance *this,
  sns_time            timestamp,
  const uint8_t       *buf,
  size_t              buf_len)
{
  if(buf_len == 3)
  {
    lsm6ds3c_convert_and_send_temp_sample(this, timestamp, buf + 1);
  }
  else
  {
    SNS_INST_PRINTF(ERROR, this, "temp_samples: Unexpected data len %u", buf_len);
  }
}


/* ------------------------------------------------------------------------------------ */
static void process_data_event(
  sns_sensor_instance *this,
  lsm6ds3c_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  pb_buffer_arg decode_arg;
  sns_dae_data_event data_event = sns_dae_data_event_init_default;
  data_event.sensor_data.funcs.decode = &pb_decode_string_cb;
  data_event.sensor_data.arg = &decode_arg;
  sns_time now = sns_get_system_time();
  if(pb_decode(pbstream, sns_dae_data_event_fields, &data_event))
  {
    lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
    lsm6ds3c_dae_if_info* dae_if = &state->dae_if;
#if LSM6DS3C_DAE_TIMESTAMP_TYPE
    sns_dae_timestamp_type ts_type = SNS_DAE_TIMESTAMP_TYPE_UNKNOWN;
#endif
    if (dae_stream == &dae_if->ag)
    {
#if LSM6DS3C_DAE_TIMESTAMP_TYPE
      if(data_event.has_timestamp_type)
      {
        ts_type = data_event.timestamp_type;
        if(ts_type == SNS_DAE_TIMESTAMP_TYPE_TIMER )
        {
          /* this driver does not support polling mode for A/G via DAE */
          SNS_INST_PRINTF(ERROR, this, "[%u] data_event: unexpected ts type", state->hw_idx);
        }
      }
      process_fifo_samples(
        this, data_event.timestamp, ts_type, (uint8_t*)decode_arg.buf, decode_arg.buf_len);
#else
      process_fifo_samples(
        this, data_event.timestamp, (uint8_t*)decode_arg.buf, decode_arg.buf_len);
#endif
      if(state->health.expected_expiration < now + (state->fifo_info.nominal_dae_intvl<<1))
      {
        lsm6ds3c_restart_hb_timer(this, true);
      }
      else
      {
        state->health.heart_attack = false;
        state->health.heart_attack_cnt = 0;
      }
    }
    else
    {
      process_temp_samples(
        this, data_event.timestamp, (uint8_t*)decode_arg.buf, decode_arg.buf_len);
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_interrupt_event(
  sns_sensor_instance *this,
  lsm6ds3c_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;

  if(dae_stream == &state->dae_if.ag)
  {
    sns_dae_interrupt_event interrupt_event = sns_dae_interrupt_event_init_default;
    if(pb_decode(pbstream, sns_dae_interrupt_event_fields, &interrupt_event))
    {
      DBG_INST_PRINTF(
        HIGH, this, "interrupt_event: byte=%x settled=%u test=%u",
        interrupt_event.registers.bytes[0], state->md_info.is_filter_settled,
        state->self_test_info.test_alive);
      if(state->current_conf.md_enabled &&
         state->md_info.is_filter_settled &&
         !state->self_test_info.test_alive)
      {
        lsm6ds3c_handle_md_interrupt(this, interrupt_event.timestamp,
                                    &interrupt_event.registers.bytes[0]);
      }
      lsm6ds3c_handle_esp_interrupt(this,interrupt_event.timestamp,
                                   &interrupt_event.registers.bytes[0]);
      if(state->dae_if.ag.state == STREAMING || state->dae_if.ag.state == STREAM_STARTING)
      {
        // tells DAE to resume
        send_ag_config(&state->dae_if.ag, this);
      }
      if(state->dae_if.ag.state == STREAMING || state->dae_if.ag.state == STREAM_STARTING)
      {
        // tells DAE to resume
        send_ag_config(&state->dae_if.ag, this);
      }
    }
    else
    {
      SNS_INST_PRINTF(ERROR, this, "interrupt_event: decode fail");
    }
  }
  else
  {
    DBG_INST_PRINTF(ERROR, this, "interrupt_event: Unexpected INT");
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_response(
  sns_sensor_instance *this,
  lsm6ds3c_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;

  sns_dae_resp resp = sns_dae_resp_init_default;
  if(pb_decode(pbstream, sns_dae_resp_fields, &resp))
  {
    DBG_INST_PRINTF_EX(MED, this, "process_response: msg=%u err=%u step=%u",
                   resp.msg_id, resp.err, state->config_step);
    switch(resp.msg_id)
    {
    case SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG:
      if(SNS_STD_ERROR_NO_ERROR == resp.err)
      {
        state->irq_info.irq_ready = true;
        state->irq2_info.irq_ready = true;
        state->irq_ready = true;
      }
      else
      {
        /* DAE sensor does not have support for this driver */
        dae_stream->stream_usable = false;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR == resp.err) ? STREAMING : IDLE;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      dae_stream->flushing_hw = false;
      if((state->flushing_sensors != 0) || state->health.heart_attack_flush || (LSM6DS3C_CONFIG_IDLE != state->config_step))
      {
        flush_samples(this, dae_stream);
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS:
      state->fifo_info.th_info.flush_req = false;
      if(LSM6DS3C_CONFIG_IDLE != state->config_step)
      {
        state->config_step = LSM6DS3C_CONFIG_IDLE;
        lsm6ds3c_read_fifo_data(this,sns_get_system_time(),true);
        lsm6ds3c_reconfig_hw(this);
      }
      else
      if(state->flushing_sensors != 0)
      {
        lsm6ds3c_send_fifo_flush_done(this, state->flushing_sensors,
                                     FLUSH_DONE_AFTER_DATA);
        state->flushing_sensors = 0;
      }
      state->health.heart_attack_flush = false;
      dae_stream->flushing_data = false;
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }
      if(NULL != state->dae_if.ag.stream &&
         state->dae_if.ag.state != STREAM_STOPPING &&
         NULL != state->dae_if.temp.stream &&
         state->dae_if.temp.state != STREAM_STOPPING)
      {
        if(state->config_step == LSM6DS3C_CONFIG_STOPPING_STREAM)
        {
          if(lsm6ds3c_dae_if_flush_hw(this))
            state->config_step = LSM6DS3C_CONFIG_FLUSHING_HW;
          else
          {
            state->config_step = LSM6DS3C_CONFIG_IDLE;
            lsm6ds3c_reconfig_hw(this);
          }
        }
      }
      break;

    case SNS_DAE_MSGID_SNS_DAE_RESP:
    case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
      break; /* unexpected */
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_events(sns_sensor_instance *this, lsm6ds3c_dae_stream *dae_stream)
{
  sns_sensor_event *event;
  bool data_received = false;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;

  while(NULL != dae_stream->stream &&
        NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream)))
  {
    if (dae_stream->stream_usable)
    {
      pb_istream_t pbstream =
        pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

      if (SNS_DAE_MSGID_SNS_DAE_DATA_EVENT == event->message_id)
      {
        process_data_event(this, dae_stream, &pbstream);
        data_received = true;
      }
      else if(SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id)
      {
        process_interrupt_event(this, dae_stream, &pbstream);
      }
      else if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
      {
        process_response(this, dae_stream, &pbstream);
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
        DBG_INST_PRINTF(MED, this, "process_events: ERROR_EVENT stream=%x state=%u",
                        dae_stream->stream, dae_stream->state);
        dae_stream->stream_usable = false;
        lsm6ds3c_register_interrupt(this, &state->irq_info, state->interrupt_data_stream);
        if(state->irq2_inst_enabled)
        {
          lsm6ds3c_register_interrupt(this, &state->irq2_info, state->interrupt2_data_stream);
        }
      }
      else
      {
        SNS_INST_PRINTF(ERROR, this, "process_events: Unexpected msg id %u", event->message_id);
      }
    }
    event = dae_stream->stream->api->get_next_input(dae_stream->stream);
  }

  if(data_received &&
     state->flushing_sensors != 0 && // Flush request was received, but
     !dae_stream->flushing_data)     // FLUSH_DATA_EVENTS was not sent to DAE sensor
  {
    // This is the case of Flush request received close to next expected interrupt
    uint8_t flushing_sensors = (dae_stream == &state->dae_if.ag) ?
      (state->flushing_sensors & (LSM6DS3C_ACCEL|LSM6DS3C_GYRO)) :
      (state->flushing_sensors & LSM6DS3C_SENSOR_TEMP);
    if(flushing_sensors != 0)
    {
      lsm6ds3c_send_fifo_flush_done(this, flushing_sensors, FLUSH_DONE_AFTER_DATA);
      state->flushing_sensors &= ~flushing_sensors;
    }
  }
}

/*======================================================================================
  Public Functions
  ======================================================================================*/
void lsm6ds3c_dae_if_check_support(sns_sensor *this)
{
  lsm6ds3c_shared_state *shared_state = lsm6ds3c_get_shared_state(this);
  if(NULL == shared_state->dae_stream)
  {
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    DBG_PRINTF(LOW, this, "check_support: creating stream");
    stream_svc->api->create_sensor_stream(stream_svc, this,
                                          shared_state->inst_cfg.dae_suid,
                                          &shared_state->dae_stream);
  }
  if(NULL != shared_state->dae_stream)
  {
    sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
    if(shared_state->inst_cfg.dae_ag_state == PRE_INIT)
    {
      shared_state->inst_cfg.dae_ag_state = INIT_PENDING;
      build_static_config_request(&shared_state->inst_cfg, &config_req,
                                  shared_state->hw_idx, shared_state->rigid_body_type, true);
    }
    else
    {
      shared_state->inst_cfg.dae_temper_state = INIT_PENDING;
      build_static_config_request(&shared_state->inst_cfg, &config_req,
                                  shared_state->hw_idx, shared_state->rigid_body_type, false);
    }

    if(SNS_RC_SUCCESS != send_static_config_request(shared_state->dae_stream, &config_req))
    {
      shared_state->inst_cfg.dae_ag_state     = UNAVAILABLE;
      shared_state->inst_cfg.dae_temper_state = UNAVAILABLE;
      sns_sensor_util_remove_sensor_stream(this, &shared_state->dae_stream);
    }
  }
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_process_sensor_events(sns_sensor *this)
{
  lsm6ds3c_shared_state *shared_state = lsm6ds3c_get_shared_state(this);
  sns_data_stream *stream = shared_state->dae_stream;
  sns_sensor_event *event;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return;
  }

  while(NULL != (event = stream->api->peek_input(stream)))
  {
    pb_istream_t pbstream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

    DBG_PRINTF(MED, this, "dae_sensor_events: msg=%u state(ag/t)=%u/%u",
               event->message_id, shared_state->inst_cfg.dae_ag_state,
               shared_state->inst_cfg.dae_temper_state);

    if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
    {
      sns_dae_resp resp = sns_dae_resp_init_default;
      if(pb_decode(&pbstream, sns_dae_resp_fields, &resp))
      {
        if(SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG == resp.msg_id)
        {
          if(shared_state->inst_cfg.dae_ag_state == INIT_PENDING)
          {
            shared_state->inst_cfg.dae_ag_state =
              (SNS_STD_ERROR_NO_ERROR != resp.err) ? UNAVAILABLE : IDLE;
            if(shared_state->inst_cfg.dae_ag_state == IDLE)
            {
              lsm6ds3c_dae_if_check_support(this); // for temperature
            }
          }
          else // must be for temperature
          {
            shared_state->inst_cfg.dae_temper_state =
              (SNS_STD_ERROR_NO_ERROR != resp.err) ? UNAVAILABLE : IDLE;
          }
        }
      }
    }
    else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
    {
      shared_state->inst_cfg.dae_ag_state = UNAVAILABLE;
    }

    event = stream->api->get_next_input(stream);
  }

  if(UNAVAILABLE == shared_state->inst_cfg.dae_ag_state |
     IDLE == shared_state->inst_cfg.dae_temper_state)
  {
    sns_sensor_util_remove_sensor_stream(this, &shared_state->dae_stream);
  }
  DBG_PRINTF(HIGH, this, "dae_sensor_events: state(ag/t)=%u/%u",
             shared_state->inst_cfg.dae_ag_state, shared_state->inst_cfg.dae_temper_state);
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_available(sns_sensor_instance *this)
{
  /* both streams must be availble */
  lsm6ds3c_dae_if_info *dae_if = &((lsm6ds3c_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->ag) && stream_usable(&dae_if->temp));
}

/* ------------------------------------------------------------------------------------ */
sns_rc lsm6ds3c_dae_if_init(
  sns_sensor_instance           *const this,
  sns_stream_service            *stream_mgr,
  lsm6ds3c_instance_config const *inst_cfg)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_dae_if_info *dae_if = &state->dae_if;

  dae_if->ag.state = inst_cfg->dae_ag_state;
  dae_if->temp.state = inst_cfg->dae_temper_state;

  if(IDLE == dae_if->ag.state && IDLE == dae_if->temp.state)
  {
    bool dae_avail = false;
    stream_mgr->api->create_sensor_instance_stream(
                       stream_mgr, this, inst_cfg->dae_suid, &dae_if->ag.stream);
    stream_mgr->api->create_sensor_instance_stream(
                       stream_mgr, this, inst_cfg->dae_suid, &dae_if->temp.stream);
    if(NULL != dae_if->ag.stream && NULL != dae_if->temp.stream)
    {
      sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
      dae_if->ag.stream_usable = dae_if->temp.stream_usable = true;
      build_static_config_request(inst_cfg, &config_req, state->hw_idx, state->rigid_body_type, true);
      if(SNS_RC_SUCCESS == send_static_config_request(dae_if->ag.stream, &config_req))
      {
        build_static_config_request(inst_cfg, &config_req, state->hw_idx, state->rigid_body_type, false);
        if(SNS_RC_SUCCESS == send_static_config_request(dae_if->temp.stream, &config_req))
        {
          dae_avail = true;
        }
      }
    }
    if(!dae_avail)
    {
      lsm6ds3c_dae_if_deinit(this);
    }
  }
  DBG_INST_PRINTF_EX(HIGH, this, "dae_if_init: state(ag/t)=%u/%u usable(ag/t)=%u/%u",
                  dae_if->ag.state, dae_if->temp.state,
                  dae_if->ag.stream_usable, dae_if->temp.stream_usable);
  return rc;
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_deinit(sns_sensor_instance *const this)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.ag.stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.temp.stream);
  state->dae_if.ag.flushing_hw = false;
  state->dae_if.ag.flushing_data = false;
  state->dae_if.ag.state = PRE_INIT;
  state->dae_if.temp.flushing_hw = false;
  state->dae_if.temp.flushing_data = false;
  state->dae_if.temp.state = PRE_INIT;
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_stop_streaming(sns_sensor_instance *this, uint8_t sensors)
{
  bool cmd_sent = false;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_dae_if_info    *dae_if = &state->dae_if;

  DBG_INST_PRINTF(MED, this, "stop_streaming: sensors=%x step=%u",
                  sensors, state->config_step);

  if((sensors & (LSM6DS3C_ACCEL | LSM6DS3C_GYRO | LSM6DS3C_MOTION_DETECT)) &&
     stream_usable(&state->dae_if.ag) &&
     (dae_if->ag.state == STREAMING || dae_if->ag.state == STREAM_STARTING))
  {
    DBG_INST_PRINTF(MED, this, "stop_streaming: AG stream=%x", &dae_if->ag.stream);
    cmd_sent |= stop_streaming(&dae_if->ag);
  }

  if((sensors & LSM6DS3C_SENSOR_TEMP) && stream_usable(&state->dae_if.temp) &&
     (dae_if->temp.state == STREAMING || dae_if->temp.state == STREAM_STARTING))
  {
    DBG_INST_PRINTF(MED, this, "stop_streaming: Temp stream=%x", &dae_if->temp.stream);
    cmd_sent |= stop_streaming(&dae_if->temp);
    state->sensor_temp_info.cur_sampling_rate_hz = 0.0f;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_start_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  lsm6ds3c_dae_if_info    *dae_if = &state->dae_if;
  bool interrrupt_sensors_enabled =
    (state->config_sensors | state->desired_conf.enabled_sensors) ||
    LSM6DS3C_IS_ESP_ENABLED(state);

  DBG_INST_PRINTF(HIGH, this,
                 "start_streaming: sensors=%x ag_state=%u temp_state=%u",
                 state->config_sensors, dae_if->ag.state, dae_if->temp.state);

  if(interrrupt_sensors_enabled &&
     stream_usable(&dae_if->ag) && (dae_if->ag.state >= IDLE) &&
     (0 < state->common_info.accel_curr_odr || 0 < state->common_info.gyro_curr_odr))
  {
    DBG_INST_PRINTF(MED, this, "start_streaming: AG stream=%x", &dae_if->ag.stream);
    cmd_sent |= send_ag_config(&dae_if->ag, this);
  }

  if(((state->config_sensors & LSM6DS3C_SENSOR_TEMP) ||
      (state->desired_conf.enabled_sensors & LSM6DS3C_SENSOR_TEMP)) &&
     stream_usable(&dae_if->temp) &&
     (dae_if->temp.state == IDLE || dae_if->temp.state == STREAM_STOPPING))
  {
    if(dae_if->temp.state != STREAMING && 0 < state->sensor_temp_info.sampling_intvl)
    {
      DBG_INST_PRINTF(MED, this, "start_streaming: Temp stream=%x", &dae_if->temp.stream);
      cmd_sent |= send_temp_config(&dae_if->temp, this);
    }
    else if((dae_if->temp.state == STREAMING  || dae_if->temp.state == STREAM_STARTING) &&
            (state->sensor_temp_info.sampling_intvl == 0))
    {
      /* In the case Temp request is removed while A/G/MD requests remain and reconfig
         is not needed, this is the only chance to stop Temp streaming on DAE */
      DBG_INST_PRINTF(MED, this,
                     "start_streaming: stop Temp stream=%x", &dae_if->temp.stream);
      stop_streaming(&dae_if->temp);
    }
  }

  state->config_sensors = 0;
  state->config_step = LSM6DS3C_CONFIG_IDLE;
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_flush_hw(sns_sensor_instance *this)
{
  lsm6ds3c_dae_if_info *dae_if = &((lsm6ds3c_instance_state*)this->state->state)->dae_if;
  bool cmd_sent = dae_if->ag.flushing_hw;

  if(stream_usable(&dae_if->ag) && dae_if->ag.state >= IDLE && !dae_if->ag.flushing_hw)
  {
    cmd_sent |= flush_hw(&dae_if->ag);
    DBG_INST_PRINTF_EX(MED, this, "flush_hw: AG stream=%x flushing=%u",
                    &dae_if->ag.stream, dae_if->ag.flushing_hw);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_process_events(sns_sensor_instance *this)
{
  lsm6ds3c_instance_state *state = (lsm6ds3c_instance_state*)this->state->state;
  process_events(this, &state->dae_if.ag);
  process_events(this, &state->dae_if.temp);

  if(!stream_usable(&state->dae_if.ag) || !stream_usable(&state->dae_if.temp))
  {
    /* both streams are needed; if one was removed, remove the other one too */
    lsm6ds3c_dae_if_deinit(this);
  }
}

#else

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_available(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}

/* ------------------------------------------------------------------------------------ */
bool lsm6ds3c_dae_if_flush_hw(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}

/* ------------------------------------------------------------------------------------ */
sns_rc lsm6ds3c_dae_if_init(
  sns_sensor_instance           *const this,
  sns_stream_service            *stream_mgr,
  lsm6ds3c_instance_config const *inst_cfg)
{
  UNUSED_VAR(this);
  UNUSED_VAR(stream_mgr);
  UNUSED_VAR(inst_cfg);
  return false;
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_deinit(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_process_events(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
void lsm6ds3c_dae_if_process_sensor_events(sns_sensor *this)
{
  UNUSED_VAR(this);
}
/* ------------------------------------------------------------------------------------ */
#endif
