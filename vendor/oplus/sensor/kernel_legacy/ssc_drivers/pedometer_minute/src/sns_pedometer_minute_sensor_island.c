/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "sns_pedometer_minute_sensor_instance.h"
#include "sns_pedometer_minute_sensor.h"

static sns_sensor_uid const* sns_pedometer_minute_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = PEDEMETER_SUID;
    return &suid;
}

static bool decode_request(sns_request const *in_request,
        sns_std_request *decoded_request, sns_std_sensor_config *decoded_payload )
{
    pb_istream_t stream;
        int err = 0;
    pb_simple_cb_arg arg =
    {
        .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields
    };
    decoded_request->payload = (struct pb_callback_s)
    {
        .funcs.decode = &pb_decode_simple_cb, .arg = &arg
    };
    stream = pb_istream_from_buffer(in_request->request,
                in_request->request_len);

        err = pb_decode(&stream, sns_std_request_fields, decoded_request);
    if ( !err ) {
        PEDEMETER_LOG_1("decode error err= %d", err);
        return false;
    }

    return true;
}

sns_sensor_instance* sns_pedometer_minute_set_client_request(sns_sensor *const this,
                                                             sns_request const *exist_request, sns_request const *new_request, bool remove)
{
    sns_sensor_instance *ret_inst = sns_sensor_util_get_shared_instance(this);
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload = {0};
    sns_request config;
    sns_pedometer_minute_req  request_config;

    PEDEMETER_LOG_2("pedometer_minute_set_client_request: exist_request = 0x%x, new_request = 0x%x", exist_request, new_request);

    if (remove) {
        if (NULL != ret_inst) {
            ret_inst->cb->remove_client_request(ret_inst, exist_request);
        }
    }
    else if ((NULL != new_request) && (SNS_STD_MSGID_SNS_STD_FLUSH_REQ == new_request->message_id) && (NULL != ret_inst)) {
        PEDEMETER_LOG_0("Flush client req");
        this->instance_api->set_client_config(ret_inst, new_request);
    }
    else if ((NULL != new_request) && ((SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == new_request->message_id)
                ||(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == new_request->message_id))) {
        if (NULL == ret_inst) {
            // If this is a request from a new client
            PEDEMETER_LOG_0("create_instance");
            ret_inst = this->cb->create_instance(this, (sizeof(sns_pedometer_minute_inst_state)));
        }

        if ((NULL != ret_inst)) {
                if (NULL != exist_request) {
                        ret_inst->cb->remove_client_request(ret_inst, exist_request);
                }
                        ret_inst->cb->add_client_request(ret_inst, new_request);
            decode_request(new_request, &decoded_request, &decoded_payload);
                        if (decoded_request.has_batching && decoded_request.batching.batch_period > 0) {
                                request_config.desired_sample_rate = (1000000.0f / (float)decoded_request.batching.batch_period);
                        }
                        else {
                            request_config.desired_sample_rate = decoded_payload.sample_rate;
                        }
            PEDEMETER_LOG_3("set_client_config message_id  = %d sampletate = %d desired_sample_rate =%d",
                            new_request->message_id,
                            (int)decoded_payload.sample_rate,
                            (int)request_config.desired_sample_rate);

            config.request_len = sizeof(sns_pedometer_minute_req);
            config.request = &request_config;
            config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
            this->instance_api->set_client_config(ret_inst, &config);
        }
    }

    if (NULL != ret_inst && NULL == ret_inst->cb->get_client_request(ret_inst, this->sensor_api->get_sensor_uid(this), true)) {
        PEDEMETER_LOG_0("remove_instance");

        this->cb->remove_instance(ret_inst);
    }

    return ret_inst;
}

sns_sensor_api sns_pedometer_minute_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_pedometer_minute_init,
    .deinit = &sns_pedometer_minute_deinit,
    .get_sensor_uid = &sns_pedometer_minute_get_sensor_uid,
    .set_client_request = &sns_pedometer_minute_set_client_request,
    .notify_event = &sns_pedometer_minute_notify_event,
};

