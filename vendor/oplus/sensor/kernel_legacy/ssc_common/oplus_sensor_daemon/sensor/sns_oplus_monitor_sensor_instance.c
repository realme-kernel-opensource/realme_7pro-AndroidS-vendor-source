/******************************************************************
** Copyright (C), 2004-2020, OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_xx.x
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "sns_oplus_monitor_sensor_instance.h"
#include "sns_mem_util.h"
#include "sns_physical_sensor_test.pb.h"

sns_rc sns_oplus_monitor_inst_init(sns_sensor_instance *this,
    sns_sensor_state const *state)
{
    sns_oplus_monitor_sensor_state *sensor_state = (sns_oplus_monitor_sensor_state*)state->state;
    sns_oplus_monitor_inst_state *inst_state = (sns_oplus_monitor_inst_state*)this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_rc rc = SNS_RC_SUCCESS;

    SNS_INST_PRINTF(HIGH, this, "oplus_monitor_inst_init enter");

    // read platform specific configuration
    inst_state->self_suid = sensor_state->self_suid;
    sns_memscpy(&inst_state->config, sizeof(inst_state->config),
        &sensor_state->config, sizeof(sensor_state->config));

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "timer", &inst_state->timer_suid);
    if (NULL == inst_state->timer_stream) {
        sns_stream_service  *strm_svc = (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this, inst_state->timer_suid, &inst_state->timer_stream);
        if ((rc != SNS_RC_SUCCESS) || (NULL == inst_state->timer_stream)) {
            SNS_INST_PRINTF(ERROR, this, "error creating stream %d", rc);
        }
    }
    inst_state->timer_count = 0;
    inst_state->fw_is_inited = false;
    return rc;
}

sns_rc sns_oplus_monitor_inst_deinit(sns_sensor_instance *const this)
{
    sns_oplus_monitor_inst_state *state = (sns_oplus_monitor_inst_state*)this->state->state;

    sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_stream);

    return SNS_RC_SUCCESS;
}

static bool oplus_monitor_get_decoded_request(sns_sensor_instance const *this, sns_request const *in_request,
                                        sns_std_request *decoded_request,
                                        sns_std_sensor_config *decoded_payload)
{
    pb_istream_t stream;

    pb_simple_cb_arg arg = {
        .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields
    };

    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb, .arg = &arg
    };

    stream = pb_istream_from_buffer(in_request->request,
        in_request->request_len);

    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        SNS_INST_PRINTF(ERROR, this, "decode error");
        return false;
    }

    SNS_INST_PRINTF(HIGH, this, "oplus monitor decoded request success");
    return true;
}

sns_rc sns_oplus_monitor_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_oplus_monitor_inst_state *state = (sns_oplus_monitor_inst_state*)this->state->state;

    if ((client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) ||
        (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)) {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (oplus_monitor_get_decoded_request(this, client_request, &decoded_request, &decoded_payload)) {
            SNS_INST_PRINTF(HIGH, this,
                "oplus monitor sample_rate = %d, batch_period = %d, flush_period = %d",
                (int)(decoded_payload.sample_rate),
                (int)decoded_request.batching.batch_period,
                (int)decoded_request.batching.flush_period);
        }
        if (!state->fw_is_inited) {
            oplus_monitor_start_timer(this, FIRST_TIMER_PERIOD);//5s
        } else {
            oplus_monitor_start_timer(this, TIMER_PERIOD);//1min
        }

    } else {
        SNS_INST_PRINTF(HIGH, this, "Unsupported request message id %u",
            client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }
    return rc;
}

