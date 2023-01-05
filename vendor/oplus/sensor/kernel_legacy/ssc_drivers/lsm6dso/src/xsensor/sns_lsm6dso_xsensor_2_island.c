/**
 * @file sns_lsm6dso_xsensor_2_island.c
 *
 * Specific implementation for LSM6DSO  XSensor
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

#include <string.h>
#include "sns_types.h"
#include "sns_lsm6dso_sensor.h"
#if LSM6DSO_ESP_XSENSOR_2
#include "sns_mem_util.h"
#include "sns_lsm6dso_xsensor.h"
#include "sns_lsm6dso_xsensor_config.h"
#include "sns_lsm6dso_fsm.h"
#include "sns_lsm6dso_mlc.h"

#define XSENSOR             LSM6DSO_XSENSOR_2
#define XSENSOR_SUID_0      XSENSOR_2_SUID_0
#if LSM6DSO_DUAL_SENSOR_ENABLED
#define XSENSOR_SUID_1      XSENSOR_2_SUID_1
#endif
#define XSENSOR_IDX         XSENSOR_2_IDX
#define XSENSOR_TYPE        LSM6DSO_XSENSOR_2_TYPE
#define XSENSOR_INT         LSM6DSO_XSENSOR_2_INT
#define XSENSOR_STREAM_TYPE LSM6DSO_XSENSOR_2_STREAM_TYPE

extern sns_sensor_api lsm6dso_xsensor_sensor_api;

void lsm6dso_reconfig_xsensor_2(sns_sensor_instance *instance, bool enable)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  bool update_intr = false;
  sns_rc rc = SNS_RC_SUCCESS;
  bool is_remove = ((!state->xgroup_info.desired_sensors) && (state->xgroup_info.enabled_sensors & XSENSOR));
  if(state->xgroup_info.desired_sensors & XSENSOR) {
    if(!(state->xgroup_info.enabled_sensors & XSENSOR)) {
      update_intr = true;
      enable = true;
    }
  } else if(state->xgroup_info.enabled_sensors & XSENSOR) {
    update_intr = true;
    enable = false;
  }
  if(update_intr) {
    rc = lsm6dso_xsensor_enable(instance, XSENSOR_TYPE, XSENSOR, XSENSOR_IDX, XSENSOR_INT, enable);
    if(rc == SNS_RC_SUCCESS) {
      if(enable)
      {
#if LSM6DSO_XSENSOR_2_GYRO_REQ
        state->xgroup_info.is_gyro_req |= XSENSOR;
#endif
        state->xgroup_info.enabled_sensors |= XSENSOR;
      }
      else
      {
#if LSM6DSO_XSENSOR_2_GYRO_REQ
        state->xgroup_info.is_gyro_req &= ~XSENSOR;
#endif
        state->xgroup_info.enabled_sensors &= ~XSENSOR;
      }
    }
  }

  if((state->xgroup_info.enabled_sensors & XSENSOR) || is_remove)
  {
    lsm6dso_set_accel_config(instance,
      state->accel_info.desired_odr,
      state->accel_info.sstvt,
      state->accel_info.range,
      state->accel_info.bw);
#if LSM6DSO_XSENSOR_2_GYRO_REQ
    {
      lsm6dso_set_gyro_config(instance,
        state->gyro_info.desired_odr,
        state->gyro_info.sstvt,
        state->gyro_info.range);
    }
#endif
  }
}

void lsm6dso_handle_xsensor_2_intr(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t src_reg,
                                 uint8_t wake_data)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  UNUSED_VAR(src_reg);
  UNUSED_VAR(wake_data);
  bool fired = false;

  if(state->xgroup_info.enabled_sensors & XSENSOR) {
    fired = lsm6dso_get_xsensor_interrupt_status(instance, XSENSOR_TYPE, XSENSOR);
    if(fired) {
#if LSM6DSO_XSENSOR_OEM_EVENT_HANDLING
      lsm6dso_send_xsensor_oem_event(instance, XSENSOR_TYPE, XSENSOR_IDX, SNS_XSENSOR_EVENT_TYPE_FIRED, irq_timestamp, NULL);
#else
      lsm6dso_send_xsensor_event(instance, XSENSOR_TYPE, XSENSOR_IDX, SNS_XSENSOR_EVENT_TYPE_FIRED, irq_timestamp, NULL);
#endif

      DBG_INST_PRINTF_EX(LOW, instance,
          "XSENSOR_2 Interrupt fired!");

      if(XSENSOR_STREAM_TYPE == SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT)
      {
        state->xgroup_info.desired_sensors &= ~lsm6dso_supported_xsensors[XSENSOR_IDX].sensor;
        lsm6dso_reconfig_xsensor_2(instance, false);
      }
    }
  }
}

sns_rc lsm6dso_xsensor_2_init(sns_sensor *const this)
{
  sns_rc rc;
  sns_sensor_uid* suid = &((sns_sensor_uid)XSENSOR_SUID_0);
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  suid = (shared_state->hw_idx) ? &((sns_sensor_uid)XSENSOR_SUID_1) : suid;
#endif
  state->hardware_id = shared_state->hw_idx;
  rc = lsm6dso_esp_init(this, suid, &lsm6dso_supported_xsensors[XSENSOR_IDX], XSENSOR);
  return rc;
}

sns_rc lsm6dso_xsensor_2_deinit(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  UNUSED_VAR(state);
  DBG_PRINTF_EX(HIGH, this, "Deinit  xsensor: %d", state->sensor);
  return SNS_RC_SUCCESS;
}

sns_sensor_instance* lsm6dso_set_xsensor_2_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{

  sns_sensor_uid* suid = &((sns_sensor_uid)XSENSOR_SUID_0);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  suid = (shared_state->hw_idx) ? &((sns_sensor_uid)XSENSOR_SUID_1) : suid;
#endif
  return lsm6dso_set_xsensor_request(this, exist_request, new_request, remove, XSENSOR_IDX, suid);
}

sns_sensor_api lsm6dso_xsensor_2_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6dso_xsensor_2_init,
  .deinit             = &lsm6dso_xsensor_2_deinit,
  .get_sensor_uid     = &lsm6dso_get_sensor_uid,
  .set_client_request = &lsm6dso_set_xsensor_2_request,
  .notify_event       = &lsm6dso_sensor_notify_event,
};

void lsm6dso_register_xsensor_2(sns_register_cb const *register_api)
{
   uint16_t xsensor_idx = XSENSOR_IDX;
   sns_strlcpy(lsm6dso_supported_xsensors[xsensor_idx].proto, "sns_xsensor.proto", sizeof(lsm6dso_supported_xsensors[xsensor_idx].proto));
   lsm6dso_supported_xsensors[xsensor_idx].sensor = XSENSOR;
   lsm6dso_supported_xsensors[xsensor_idx].min_odr = LSM6DSO_FSM_MLC_ODR;
   lsm6dso_supported_xsensors[xsensor_idx].stream_type = XSENSOR_STREAM_TYPE;
   lsm6dso_supported_xsensors[xsensor_idx].reconfig = &lsm6dso_reconfig_xsensor_2;
   lsm6dso_supported_xsensors[xsensor_idx].handle_intr = &lsm6dso_handle_xsensor_2_intr;
   lsm6dso_supported_xsensors[xsensor_idx].handle_timer_events = NULL;
   register_api->init_sensor(sizeof(lsm6dso_state),
                              &lsm6dso_xsensor_2_sensor_api,
                              &lsm6dso_sensor_instance_api);
}
#endif //LSM6DSO_ESP_XSENSOR_2

