/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_xx.x
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "sns_oplus_monitor_sensor.h"
#include "sns_oplus_monitor_sensor_instance.h"
#include "sns_physical_sensor_test.pb.h"

static void oplus_monitor_publish_attributes(sns_sensor *const this)
{
    {
        char const name[] = "oplus_monitor";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) { .buf = name, .buf_len = sizeof(name)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
    {
        char const type[] = "oplus_monitor";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) { .buf = type, .buf_len = sizeof(type)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }
    {
        char const vendor[] = "oplus";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg){ .buf = vendor, .buf_len = sizeof(vendor)});
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
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

static void oplus_monitor_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

sns_rc sns_oplus_monitor_init(sns_sensor *const this)
{
    sns_oplus_monitor_sensor_state *state = (sns_oplus_monitor_sensor_state*)this->state->state;
    float data[8] = {0};

    SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
    sns_suid_lookup_add(this, &state->suid_lookup_data, "timer");

    // determine encoded output event size
    state->self_suid = *(this->sensor_api->get_sensor_uid(this));
    state->config.encoded_data_event_len = pb_get_encoded_size_sensor_stream_event(data, 8);

    oplus_monitor_publish_attributes(this);

    //init feedback smem
    oplus_feedback_init();

    SNS_PRINTF(HIGH, this, "oplus_monitor init success and attributes published");

    return SNS_RC_SUCCESS;
}

sns_rc sns_oplus_monitor_deinit(sns_sensor *const this)
{
    SNS_PRINTF(HIGH, this, "sns_oplus_monitor_deinit");
    return SNS_RC_SUCCESS;
}

sns_rc sns_oplus_monitor_notify_event(sns_sensor *const this)
{
    sns_oplus_monitor_sensor_state *state = (sns_oplus_monitor_sensor_state*)this->state->state;
    SNS_PRINTF(LOW, this, "sns_oplus_monitor_notify_event");

    sns_suid_lookup_handle(this, &state->suid_lookup_data);

    if (sns_suid_lookup_complete(&state->suid_lookup_data)) {
            oplus_monitor_publish_available(this);
            sns_suid_lookup_deinit(this, &state->suid_lookup_data);
    }
    return SNS_RC_SUCCESS;
}

sns_sensor_instance* sns_oplus_monitor_set_client_request(sns_sensor *const this,
        sns_request const *exist_request,
        sns_request const *new_request, bool remove)
{
    sns_sensor_instance *ret_inst = sns_sensor_util_get_shared_instance(this);

    SNS_PRINTF(HIGH, this, "oplus_monitor: exist_request = 0x%x, new_request = 0x%x, remove = %d\n",
        exist_request, new_request, remove);

    if (remove) {
        if ((NULL != ret_inst) && (exist_request != NULL)) {
            ret_inst->cb->remove_client_request(ret_inst, exist_request);
        }
    } else if ((NULL != new_request) && ((SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == new_request->message_id) ||
            (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == new_request->message_id))) {
        SNS_PRINTF(HIGH, this, "oplus_monitor_set_client_request: message_id = %d", new_request->message_id);
        if (NULL == ret_inst) {
            // If this is a request from a new client
            SNS_PRINTF(HIGH, this, "oplus_monitor_set_client_request: create instance");

            ret_inst = this->cb->create_instance(this, (sizeof(sns_oplus_monitor_inst_state)));
        }

        if (NULL != ret_inst) {
            if (exist_request != NULL) {
                ret_inst->cb->remove_client_request(ret_inst, exist_request);
            }
            SNS_PRINTF(HIGH, this, "oplus_monitor_set_client_request: set client config");

            ret_inst->cb->add_client_request(ret_inst, new_request);
            this->instance_api->set_client_config(ret_inst, new_request);
        }
    }

    if (NULL != ret_inst &&
        NULL == ret_inst->cb->get_client_request(ret_inst, this->sensor_api->get_sensor_uid(this), true)) {
        SNS_PRINTF(HIGH, this, "oplus_monitor_set_client_request: remove instance");
        this->cb->remove_instance(ret_inst);
    }

    return ret_inst;
}
