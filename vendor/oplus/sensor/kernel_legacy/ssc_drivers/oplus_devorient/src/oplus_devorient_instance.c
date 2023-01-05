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

#include "oplus_devorient_instance.h"

extern struct devorient_sensor_operation  devorient_ops;
extern sns_sensor_instance *devorient_mTask;

sns_rc devorient_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    devorient_inst_state *inst_state = (devorient_inst_state*)this->state->state;
    devorient_sensor_state *sensor_state = (devorient_sensor_state*)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    DEVORIENT_LOG_INFO("devorient_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->acc_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "amd", &inst_state->amd_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);

    inst_state->diag_service = (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config, sizeof(sensor_state->config));

    devorient_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &devorient_ops;

    devorient_mTask = this;

    return rc;
}

sns_rc devorient_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);

    devorient_algo_close();

    return SNS_RC_SUCCESS;
}

sns_rc devorient_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        DEVORIENT_LOG_INFO("Sensor config");

        devorient_algo_reset();
    } else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        DEVORIENT_LOG_DEBUG("Flush req");
        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        DEVORIENT_LOG_DEBUG("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}