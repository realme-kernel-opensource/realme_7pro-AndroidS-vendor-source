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

#include "sns_near_ear_detect_sensor_instance.h"

extern struct near_ear_detect_sensor_operation near_ear_detect_ops;
extern sns_sensor_instance *near_ear_detect_mTask;

sns_rc sns_near_ear_detect_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_near_ear_detect_inst_state *inst_state = (sns_near_ear_detect_inst_state *)this->state->state;
    sns_near_ear_detect_sensor_state *sensor_state = (sns_near_ear_detect_sensor_state *)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    NEAR_EAR_DETECT_LOG_0("sns_near_ear_detect_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->accel_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gyro", &inst_state->gyro_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gravity", &inst_state->gravity_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "proximity", &inst_state->prox_suid);


    inst_state->diag_service = (sns_diag_service *)service_mgr->get_service(service_mgr,
            SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config,
        sizeof(sensor_state->config));

    near_ear_detect_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &near_ear_detect_ops;

    near_ear_detect_mTask = this;

    return rc;
}

sns_rc sns_near_ear_detect_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);
    near_ear_detect_close();

    return SNS_RC_SUCCESS;
}

sns_rc sns_near_ear_detect_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        NEAR_EAR_DETECT_LOG_0("Sensor config");

        near_ear_detect_Reset();
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        NEAR_EAR_DETECT_LOG_0("Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        NEAR_EAR_DETECT_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

