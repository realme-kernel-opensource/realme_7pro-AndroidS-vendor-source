/**
 * @file sns_lsm6dso_step_counter.c
 *
 * LSM6DSO Step Counter Sensor implementation.
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

#if LSM6DSO_ESP_STEP_COUNTER
#include "pb_encode.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"

extern sns_sensor_api lsm6dso_step_counter_sensor_api;

extern void lsm6dso_reconfig_sc(sns_sensor_instance *instance, bool enable);
extern void lsm6dso_handle_sc_interrupt(sns_sensor_instance *const instance, sns_time irq_timestamp,
                                        uint8_t src_reg, uint8_t wake_data);

void lsm6dso_publish_step_counter_attributes(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  {
    float data[3] = {0};
    state->encoded_event_len =
        pb_get_encoded_size_sensor_stream_event(data, 1);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
  }
}

sns_rc lsm6dso_step_counter_init(sns_sensor *const this)
{
  sns_rc rc = SNS_RC_SUCCESS;

  sns_sensor_uid* suid = &((sns_sensor_uid)STEP_COUNTER_SUID_0);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  suid = (shared_state->hw_idx) ? &((sns_sensor_uid)STEP_COUNTER_SUID_1) : suid;
#endif
  lsm6dso_publish_step_counter_attributes(this);
  lsm6dso_publish_esp_attributes(this, &lsm6dso_supported_esp_sensors[SC_INDX]);
  lsm6dso_init_sensor_info(this, suid, LSM6DSO_STEP_COUNTER);
  return rc;
}

sns_rc lsm6dso_step_counter_deinit(sns_sensor *const this, lsm6dso_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  DBG_PRINTF_EX(HIGH, this, "Deinit sensor: %d", sensor);
  return SNS_RC_SUCCESS;
}

void lsm6dso_register_step_counter(sns_register_cb const *register_api)
{
   sns_strlcpy(lsm6dso_supported_esp_sensors[SC_INDX].proto, "sns_step_counter.proto", sizeof(lsm6dso_supported_esp_sensors[SC_INDX].proto));
   lsm6dso_supported_esp_sensors[SC_INDX].sensor = LSM6DSO_STEP_COUNTER;
   lsm6dso_supported_esp_sensors[SC_INDX].min_odr = LSM6DSO_ODR_26;
   lsm6dso_supported_esp_sensors[SC_INDX].stream_type = SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT;
   lsm6dso_supported_esp_sensors[SC_INDX].reconfig = &lsm6dso_reconfig_sc;
   lsm6dso_supported_esp_sensors[SC_INDX].handle_intr = &lsm6dso_handle_sc_interrupt;
   lsm6dso_supported_esp_sensors[SC_INDX].handle_timer_events = NULL;

   register_api->init_sensor(sizeof(lsm6dso_state),
                              &lsm6dso_step_counter_sensor_api,
                              &lsm6dso_sensor_instance_api);
}

#endif
