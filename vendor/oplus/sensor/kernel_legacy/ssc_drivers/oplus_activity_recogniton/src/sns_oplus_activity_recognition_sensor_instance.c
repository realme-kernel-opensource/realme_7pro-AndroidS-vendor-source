/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - sns_oplus_activity_recognition_sensor_instance.c
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

#include "sns_oplus_activity_recognition_sensor_instance.h"

extern struct oplus_activity_recognition_sensor_operation oplus_activity_recognition_ops;
extern sns_sensor_instance *oplus_activity_recognition_mTask;

sns_rc sns_oplus_activity_recognition_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_oplus_activity_recognition_inst_state *inst_state = (sns_oplus_activity_recognition_inst_state *) this->state->state;
    sns_oplus_activity_recognition_sensor_state *sensor_state = (sns_oplus_activity_recognition_sensor_state *) state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    OPLUS_ACTIVITY_RECOGNITION_LOG_0("sns_oplus_activity_recognition_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "remote_proc_state", &inst_state->remote_proc_sensor_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "motion_recognition", &inst_state->motion_recognition_sensor_suid);

    inst_state->diag_service = (sns_diag_service *) service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config, sizeof(sensor_state->config));

    oplus_activity_recognition_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &oplus_activity_recognition_ops;

    oplus_activity_recognition_mTask = this;

    return rc;
}

sns_rc sns_oplus_activity_recognition_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);

    oplus_activity_recognition_close();

    return SNS_RC_SUCCESS;
}

sns_rc sns_oplus_activity_recognition_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Sensor config");

        oplus_activity_recognition_Reset();
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        OPLUS_ACTIVITY_RECOGNITION_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}
