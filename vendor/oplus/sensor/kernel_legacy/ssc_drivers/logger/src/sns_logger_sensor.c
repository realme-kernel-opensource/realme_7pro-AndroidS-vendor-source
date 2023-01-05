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

#include "sns_logger_sensor.h"
#include "sns_logger_sensor_instance.h"

static void logger_publish_attributes(sns_sensor *const this)
{
    {
        char const name[] = "sensor_logger";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = name, .buf_len = sizeof(name)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
    {
        char const type[] = "sensor_logger";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = type, .buf_len = sizeof(type)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }
    {
        char const vendor[] = "oppo";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = vendor, .buf_len = sizeof(vendor)
        });
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
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = proto1, .buf_len = sizeof(proto1)
        });
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

static void logger_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;

    SNS_PRINTF(ERROR, this, "logger_publish_available");

    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

sns_rc sns_logger_init(sns_sensor *const this)
{
    sns_logger_sensor_state *state = (sns_logger_sensor_state *)this->state->state;
    struct sns_service_manager *smgr = this->cb->get_service_manager(this);

    float logdata[sizeof(log_data_info) / sizeof(int)] = {0};

    state->diag_service = (sns_diag_service *)smgr->get_service(smgr, SNS_DIAG_SERVICE);

    // determine encoded output event size
    state->config.encoded_data_event_len = pb_get_encoded_size_sensor_stream_event(logdata,
            ARR_SIZE(logdata));

    SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
    sns_suid_lookup_add(this, &state->suid_lookup_data, "timer");

    logger_publish_attributes(this);

    SNS_PRINTF(ERROR, this, "logger init success and attributes published");

    return SNS_RC_SUCCESS;
}

sns_rc sns_logger_deinit(sns_sensor *const this)
{
    UNUSED_VAR(this);
    SNS_PRINTF(ERROR, this, "sns_logger_deinit");

    return SNS_RC_SUCCESS;
}

sns_rc sns_logger_notify_event(sns_sensor *const this)
{
    sns_logger_sensor_state *state = (sns_logger_sensor_state *)this->state->state;

    SNS_PRINTF(ERROR, this, "sns_logger_notify_event");

    sns_suid_lookup_handle(this, &state->suid_lookup_data);

    if (sns_suid_lookup_complete(&state->suid_lookup_data)) {
        logger_publish_available(this);

        sns_suid_lookup_deinit(this, &state->suid_lookup_data);
    }

    return SNS_RC_SUCCESS;
}

static void sns_logger_power_set(sns_sensor *const this, bool on)
{
    static bool power_has_on = false;
    sns_rc rv = SNS_RC_SUCCESS;
    sns_rail_config logger_power_confg;
    struct sns_service_manager *smgr = this->cb->get_service_manager(this);

    sns_pwr_rail_service *stream_mgr = (sns_pwr_rail_service *)smgr->get_service(smgr,
            SNS_POWER_RAIL_SERVICE);

    logger_power_confg.num_of_rails = 1;
    sns_strlcpy(logger_power_confg.rails[0].name, "/pmic/client/dummy_vdd",
        sizeof(logger_power_confg.rails[0].name));

    if (on) {
        if (power_has_on) {
            SNS_PRINTF(ERROR, this, "logger power has been on, return;");

            return;
        }

        logger_power_confg.rail_vote = SNS_RAIL_ON_LPM;

        stream_mgr->api->sns_register_power_rails(stream_mgr, &logger_power_confg);

        rv = stream_mgr->api->sns_vote_power_rail_update(stream_mgr, this, &logger_power_confg, NULL);
        if (SNS_RC_SUCCESS != rv) {
            SNS_PRINTF(ERROR, this, "logger power on failed.");
        } else {
            SNS_PRINTF(ERROR, this, "logger power on.");

            power_has_on = true;
        }
    } else {
        if (!power_has_on) {
            SNS_PRINTF(ERROR, this, "logger power has been down, return;");

            return;
        }

        logger_power_confg.rail_vote = SNS_RAIL_OFF;

        rv = stream_mgr->api->sns_vote_power_rail_update(stream_mgr, this, &logger_power_confg, NULL);
        if (SNS_RC_SUCCESS != rv) {
            SNS_PRINTF(ERROR, this, "logger power down failed.");
        } else {
            SNS_PRINTF(ERROR, this, "logger power down.");

            power_has_on = false;
        }
    }
}

sns_sensor_instance *sns_logger_set_client_request(sns_sensor *const this,
    sns_request const *exist_request, sns_request const *new_request, bool remove)
{
    sns_sensor_instance *ret_inst = sns_sensor_util_get_shared_instance(this);

    SNS_PRINTF(ERROR, this, "logger_set_client_request: exist_request = 0x%x, new_request = 0x%x",
        exist_request, new_request);

    if (remove) {
        if (NULL != ret_inst) {
            ret_inst->cb->remove_client_request(ret_inst, exist_request);
        }
    } else if ((NULL != new_request) && (SNS_STD_MSGID_SNS_STD_FLUSH_REQ == new_request->message_id)
        && (NULL != ret_inst)) {
        SNS_PRINTF(ERROR, this, "Flush client req");

        this->instance_api->set_client_config(ret_inst, new_request);
    } else if ((NULL != new_request)
        && (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == new_request->message_id)) {
        if (NULL == ret_inst) {
            // If this is a request from a new client
            SNS_PRINTF(ERROR, this, "create_instance");

            // must operate power, or if just open logger, island will be blocked.
            sns_logger_power_set(this, true);

            ret_inst = this->cb->create_instance(this, (sizeof(sns_logger_inst_state)));
        }

        if ((NULL != ret_inst) && (NULL == exist_request)) {
            SNS_PRINTF(ERROR, this, "set_client_config");

            ret_inst->cb->add_client_request(ret_inst, new_request);
            this->instance_api->set_client_config(ret_inst, new_request);
        }
    }

    if (NULL != ret_inst
        && NULL == ret_inst->cb->get_client_request(ret_inst, this->sensor_api->get_sensor_uid(this),
            true)) {
        SNS_PRINTF(ERROR, this, "remove_instance");

        this->cb->remove_instance(ret_inst);

        sns_logger_power_set(this, false);
    }

    return ret_inst;
}

