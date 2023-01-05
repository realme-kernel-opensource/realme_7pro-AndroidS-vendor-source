/**
 * @file sns_lsm6dso_dae_if.c
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

#include "sns_types.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"

#if LSM6DSO_DAE_ENABLED

#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_service_manager.h"
#include "sns_sensor_util.h"
#include "sns_stream_service.h"
#include "sns_time.h"

#include "sns_lsm6dso_hal.h"

#include "sns_dae.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"

/* ------------------------------------------------------------------------------------ */
void lsm6dso_dae_if_build_static_config_request(
  lsm6dso_instance_config const *inst_cfg,
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
      (hw_idx == 0) ? "lsm6dso_fifo_hal_table" : "lsm6dso_fifo_hal_table2",
      sizeof(config_req->func_table_name));

    config_req->interrupt              = 1;
    config_req->has_irq_config         = true;
    config_req->irq_config             = inst_cfg->irq_config;

    config_req->has_accel_info         = true;
    config_req->accel_info.accel_range =
      lsm6dso_accel_range_max[inst_cfg->accel_resolution_idx];
    config_req->accel_info.axis_map_count = ARR_SIZE(config_req->accel_info.axis_map);
    for(uint32_t i = 0; i < config_req->accel_info.axis_map_count; i++)
    {
      dest_axis_map[i].ipaxis = src_axis_map[i].ipaxis;
      dest_axis_map[i].opaxis = src_axis_map[i].opaxis;
      dest_axis_map[i].invert = src_axis_map[i].invert;
    }
    // Populate Additional Attributes
    config_req->accel_info.accel_attr[0].value.has_sint = true;
    config_req->accel_info.accel_attr[0].value.sint = rigid_body_type;
    config_req->accel_info.accel_attr[0].attr_id = SNS_STD_SENSOR_ATTRID_RIGID_BODY;
    config_req->accel_info.accel_attr_count = 1;
  }
  else // for temperature
  {
    sns_strlcpy(
      config_req->func_table_name,
      (hw_idx == 0) ? "lsm6dso_temperature_hal_table" : "lsm6dso_temperature_hal_table2",
      sizeof(config_req->func_table_name));
    config_req->interrupt             = 0;
    config_req->has_irq_config        = false;
    config_req->has_accel_info        = false;
  }

  // Neither AG nor temp use IBI or S4S
#if LSM6DSO_USE_I3C
  config_req->has_ibi_config         = false;
#endif
  config_req->has_s4s_config         = false;
}

/* ------------------------------------------------------------------------------------ */
sns_rc lsm6dso_dae_if_send_static_config_request(
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
sns_rc lsm6dso_dae_if_init(
  sns_sensor_instance           *const this,
  sns_stream_service            *stream_mgr,
  lsm6dso_instance_config const *inst_cfg)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_dae_if_info *dae_if = &state->dae_if;

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
      lsm6dso_dae_if_build_static_config_request(inst_cfg, &config_req, state->hw_idx, state->rigid_body_type, true);
      if(SNS_RC_SUCCESS == lsm6dso_dae_if_send_static_config_request(dae_if->ag.stream, &config_req))
      {
        lsm6dso_dae_if_build_static_config_request(inst_cfg, &config_req, state->hw_idx, state->rigid_body_type, false);
        if(SNS_RC_SUCCESS == lsm6dso_dae_if_send_static_config_request(dae_if->temp.stream, &config_req))
        {
          dae_avail = true;
        }
      }
    }
    if(!dae_avail)
    {
      lsm6dso_dae_if_deinit(this);
    }
  }
  DBG_INST_PRINTF_EX(HIGH, this, "dae_if_init: state(ag/t)=%u/%u usable(ag/t)=%u/%u",
                  dae_if->ag.state, dae_if->temp.state,
                  dae_if->ag.stream_usable, dae_if->temp.stream_usable);
  return rc;
}

void lsm6dso_dae_if_check_support(sns_sensor *this)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
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
    lsm6dso_exit_island(this);
    if(shared_state->inst_cfg.dae_ag_state == PRE_INIT)
    {
      shared_state->inst_cfg.dae_ag_state = INIT_PENDING;
      lsm6dso_dae_if_build_static_config_request(&shared_state->inst_cfg, &config_req,
                                  shared_state->hw_idx, shared_state->rigid_body_type, true);
    }
    else
    {
      shared_state->inst_cfg.dae_temper_state = INIT_PENDING;
      lsm6dso_dae_if_build_static_config_request(&shared_state->inst_cfg, &config_req,
                                  shared_state->hw_idx, shared_state->rigid_body_type, false);
    }

    if(SNS_RC_SUCCESS != lsm6dso_dae_if_send_static_config_request(shared_state->dae_stream, &config_req))
    {
      shared_state->inst_cfg.dae_ag_state     = UNAVAILABLE;
      shared_state->inst_cfg.dae_temper_state = UNAVAILABLE;
      sns_sensor_util_remove_sensor_stream(this, &shared_state->dae_stream);
    }
  }
}

/* ------------------------------------------------------------------------------------ */
void lsm6dso_dae_if_deinit(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.ag.stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.temp.stream);
  state->dae_if.ag.flushing_hw = false;
  state->dae_if.ag.flushing_data = false;
  state->dae_if.ag.state = PRE_INIT;
  state->dae_if.temp.flushing_hw = false;
  state->dae_if.temp.flushing_data = false;
  state->dae_if.temp.state = PRE_INIT;
}

#else
/* ------------------------------------------------------------------------------------ */
void lsm6dso_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
sns_rc lsm6dso_dae_if_init(
  sns_sensor_instance           *const this,
  sns_stream_service            *stream_mgr,
  lsm6dso_instance_config const *inst_cfg)
{
  UNUSED_VAR(this);
  UNUSED_VAR(stream_mgr);
  UNUSED_VAR(inst_cfg);
  return false;
}

/* ------------------------------------------------------------------------------------ */
void lsm6dso_dae_if_deinit(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
}

#endif
