/**
 * @file sns_lps22hx_sensor_temperature.c
 *
 * LPS22HX Sensor Temperature virtual Sensor implementation.
 *
 * Copyright (c) 2018 Qualcomm Technologies, Inc. All Rights
 * Reserved. Confidential and Proprietary - Qualcomm
 * Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_lps22hx_sensor.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "sns_attribute_util.h"
/**
 * Initialize attributes to their default state. They may/will
 * be updated within notify_event.
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void lps22hx_sensor_temp_publish_attributes(sns_sensor *const this)
{

  static const char type[] = "sensor_temperature";

  const float resolutions[] = {LPS22HX_PRESS_RESOLUTION };  // 1/4096 lsb/hPa

  static const uint32_t active_current = STM_LPS22HX_ACTIVE_CURRENT; //uA
  static const uint32_t sleep_current = 1; //uA

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
        values, 0, false);
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
    range1[0].flt = LPS22HX_SENSOR_TEMPERATURE_RANGE_MIN;
    range1[1].has_flt = true;
    range1[1].flt = LPS22HX_SENSOR_TEMPERATURE_RANGE_MAX;
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
sns_rc lps22hx_sensor_temp_init(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_memset(state, 0, sizeof(state));
  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  state->sensor = LPS22HX_SENSOR_TEMP;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
      sizeof(state->my_suid),
      &((sns_sensor_uid)PRESS_SENSOR_TEMP_SUID),
      sizeof(sns_sensor_uid));
  lps22hx_sensor_temp_publish_attributes(this);

  return SNS_RC_SUCCESS;
}

sns_rc lps22hx_sensor_temp_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  // Turn Sensor OFF.
  // Close COM port.
  // Turn Power Rails OFF.
  // No need to clear lps22hx_state because it will get freed anyway.

  return SNS_RC_SUCCESS;
}
