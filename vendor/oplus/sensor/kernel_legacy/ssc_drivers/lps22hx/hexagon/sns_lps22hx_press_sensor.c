/**
 * @file sns_lps22hx_press_sensor.c
 *
 * LPS22HX Press virtual Sensor implementation.
 *
 * Copyright (c) 2018, STMicroelectronics.
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
#include "sns_types.h"
#include "sns_service_manager.h"
#include "pb_encode.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"
#include "sns_lps22hx_sensor.h"

/**
 * Initialize attributes to their default state. They may/will
 * be updated within notify_event.
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void lps22hx_press_publish_attributes(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;

  static const char type[] = "pressure";
  const float resolutions[] = {LPS22HX_PRESS_RESOLUTION };  // 1/4096 lsb/hPa

  static const uint32_t active_current = STM_LPS22HX_ACTIVE_CURRENT; //uA
  static const uint32_t sleep_current = 1; //uA

  DBG_PRINT_EX(state->diag_service, this, ERROR, __FILENAME__, __LINE__,"lps22hx_press_publish_attributes ");

  lps22hx_publish_def_attributes(this);

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }

  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = resolutions[0];
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
        values, ARR_SIZE(values), false);
  }

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = active_current; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT, &value, 1, false);
  }

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = sleep_current; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};

    sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
    range1[0].has_flt = true;
    range1[0].flt = LPS22HX_PRESS_RANGE_MIN;
    range1[1].has_flt = true;
    range1[1].flt = LPS22HX_PRESS_RANGE_MAX;
    values[0].has_subtype = true;
    values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[0].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
        values, ARR_SIZE(values), false);
  }
  {
#if LPS22HX_ENABLE_LPF
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};

    values[0].has_flt = true;
    values[0].flt = LPS22HX_ODR_10;
    values[1].has_flt = true;
    values[1].flt = LPS22HX_ODR_25;
#else
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};

    values[0].has_flt = true;
    values[0].flt = LPS22HX_ODR_1;
    values[1].has_flt = true;
    values[1].flt = LPS22HX_ODR_10;
    values[2].has_flt = true;
    values[2].flt = LPS22HX_ODR_25;
#endif

    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
        values, ARR_SIZE(values), true);
  }
}

/* See sns_sensor::init */
sns_rc lps22hx_press_init(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;

  sns_memset(state, 0, sizeof(state));
  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  DBG_PRINT_EX(state->diag_service, this, ERROR, __FILENAME__, __LINE__,"Entering ");

  state->sensor = LPS22HX_PRESS;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
      sizeof(state->my_suid),
      &((sns_sensor_uid)PRESS_SUID),
      sizeof(sns_sensor_uid));
  lps22hx_press_publish_attributes(this);

  //initialize default dependencies - interrupt, async port, timer
  lps22hx_init_dependencies(this);

#ifdef LPS22HX_ENABLE_REGISTRY_ACCESS
  lps22hx_send_suid_req(this, "registry", 9);
#else
  sns_registry_phy_sensor_pf_cfg cfg;
  sns_lps22hx_registry_def_config(this, &cfg);
  sns_lps22hx_save_registry_pf_cfg(this, &cfg);
#endif //LPS22HX_ENABLE_REGISTRY_ACCESS

  DBG_PRINT_EX(state->diag_service, this, ERROR, __FILENAME__, __LINE__,"Exit ");

  return SNS_RC_SUCCESS;
}

sns_rc lps22hx_press_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  // Turn Sensor OFF.
  // Close COM port.
  // Turn Power Rails OFF.
  // No need to clear lps22hx_state because it will get freed anyway.

  return SNS_RC_SUCCESS;
}
