/**
 * @file sns_ak0991x_magnetic_sensor.c
 *
 * AK0991X Magnetic virtual Sensor implementation.
 *
 * Copyright (c) 2016-2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_ver.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#include "sns_printf.h"
/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
static void ak0991x_publish_default_attributes(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    static char const proto1[] = "sns_mag.proto";
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }
  {
    static char const name[] = "ak0991x";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    static char const type[] = "mag";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    static char const vendor[] = "akm";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = vendor, .buf_len = sizeof(vendor) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = AK0991X_DRIVER_VERSION;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
  }
  {
    float data[3] = {0};
    state->encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
    values[1].has_sint = true;
    values[1].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
        values, ARR_SIZE(values), true);
  }
}


/* See sns_sensor::init */
sns_rc ak0991x_mag_init(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  struct sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->scp_service =
     (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  ak0991x_publish_default_attributes(this);

  AK0991X_PRINT(LOW, this, "ak0991x: init");

  state->hw_is_present = false;
  state->sensor_client_present = false;
  state->debug_log_count = 0;

#ifdef AK0991X_ENABLE_DUAL_SENSOR
  state->registration_idx = this->cb->get_registration_index(this);
  AK0991X_PRINT(LOW, this, "registration_idx=%d",state->registration_idx);
#endif // AK0991X_ENABLE_DUAL_SENSOR

  uint8_t i = 0;

  // initialize axis conversion settings
  for(i = 0; i < TRIAXIS_NUM; i++)
  {
    state->axis_map[i].opaxis = i;
    state->axis_map[i].ipaxis = i;
    state->axis_map[i].invert = false;
  }

  // initialize fac cal correction matrix to identity
  uint8_t j;
  for(j = 0; j < MAX_DEVICE_MODE_SUPPORTED; j++)
  {
    state->cal_params[j].corr_mat.e00 = 1.0f;
    state->cal_params[j].corr_mat.e11 = 1.0f;
    state->cal_params[j].corr_mat.e22 = 1.0f;
  }

  SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
#ifdef AK0991X_ENABLE_DAE
  sns_suid_lookup_add(this, &state->suid_lookup_data, "data_acquisition_engine");
#endif // AK0991X_ENABLE_DAE
  sns_suid_lookup_add(this, &state->suid_lookup_data, "interrupt");
  sns_suid_lookup_add(this, &state->suid_lookup_data, "async_com_port");
  sns_suid_lookup_add(this, &state->suid_lookup_data, "timer");
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  sns_suid_lookup_add(this, &state->suid_lookup_data, "registry");
#endif // AK0991X_ENABLE_REGISTRY_ACCESS
#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
  sns_suid_lookup_add(this, &state->suid_lookup_data, "device_mode");
#endif

  return SNS_RC_SUCCESS;
}

/** See sns_sensor.h */
sns_rc ak0991x_mag_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

