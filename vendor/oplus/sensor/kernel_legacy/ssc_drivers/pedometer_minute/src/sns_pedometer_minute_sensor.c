/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for free fall detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/ 

#include "sns_pedometer_minute_sensor.h"
#include "sns_pedometer_minute_sensor_instance.h"

static void pedometer_minute_publish_attributes(sns_sensor *const this)
{
    {
        char const name[] = "pedometer_minute";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg){ .buf = name, .buf_len = sizeof(name)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
    {
        char const type[] = "pedometer_minute";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) { .buf = type, .buf_len = sizeof(type)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }
    {
        char const vendor[] = "oppo";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg){ .buf = vendor, .buf_len = sizeof(vendor)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = 10.0;
        values[1].has_flt = true;
        values[1].flt = 50.0;
        values[2].has_flt = true;
        values[2].flt = 100.0;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES, values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        char const proto1[] = "sns_std_sensor.proto";

        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {.buf = proto1, .buf_len = sizeof(proto1)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API, values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.has_sint = true;
        value.sint = 1;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, true);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.has_sint = true;
        value.sint  = SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE, &value, 1, true);
    }
}

static void pedometer_minute_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

    PEDEMETER_LOG_0("pedometer_minute_publish_available");

    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

sns_rc sns_pedometer_minute_init(sns_sensor *const this)
{
    sns_pedometer_minute_sensor_state *state = (sns_pedometer_minute_sensor_state*)this->state->state;
    struct sns_service_manager *smgr = this->cb->get_service_manager(this);
    float data[6] = {0};

    state->diag_service = (sns_diag_service*)smgr->get_service(smgr, SNS_DIAG_SERVICE);

    state->config.encoded_data_event_len = pb_get_encoded_size_sensor_stream_event(data, 6);

    SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
    sns_suid_lookup_add(this, &state->suid_lookup_data, "resampler");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "timer");
#ifdef SW_STEPCNT
    sns_suid_lookup_add(this, &state->suid_lookup_data, "pedometer");
#else
    sns_suid_lookup_add(this, &state->suid_lookup_data, "step_count");
#endif

#ifdef REGISTRY_CMC
    sns_suid_lookup_add(this, &state->suid_lookup_data, "motion_recognition");
#endif

    pedometer_minute_publish_attributes(this);
    PEDEMETER_LOG_0("pedometer_minute init success and attributes published");

    return SNS_RC_SUCCESS;
}

sns_rc sns_pedometer_minute_deinit(sns_sensor *const this)
{
    UNUSED_VAR(this);
    PEDEMETER_LOG_0("sns_pedometer_minute_deinit");
    return SNS_RC_SUCCESS;
}

sns_rc sns_pedometer_minute_notify_event(sns_sensor *const this)
{
    sns_pedometer_minute_sensor_state *state = (sns_pedometer_minute_sensor_state*)this->state->state;

    PEDEMETER_LOG_0("sns_pedometer_minute_notify_event");

    sns_suid_lookup_handle(this, &state->suid_lookup_data);

    if (sns_suid_lookup_complete(&state->suid_lookup_data)) {
        pedometer_minute_publish_available(this);

        sns_suid_lookup_deinit(this, &state->suid_lookup_data);
    }

    return SNS_RC_SUCCESS;
}
