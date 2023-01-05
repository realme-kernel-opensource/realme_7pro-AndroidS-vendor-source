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

#include "sns_fp_display_sensor_instance.h"

extern struct fp_display_sensor_operation fp_display_ops;
extern sns_sensor_instance *fp_display_mTask;

sns_rc sns_fp_display_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_fp_display_inst_state *inst_state = (sns_fp_display_inst_state *)this->state->state;
    sns_fp_display_sensor_state *sensor_state = (sns_fp_display_sensor_state *)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    FP_DISPLAY_LOG_0("sns_fp_display_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->accel_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gyro", &inst_state->gyro_suid);
    if (0 == sensor_state->config.smem->parameter[0]) {
        sns_suid_lookup_get(&sensor_state->suid_lookup_data, "proximity", &inst_state->prox_suid);
    } else {
        sns_suid_lookup_get(&sensor_state->suid_lookup_data, "proximity_fake", &inst_state->prox_suid);
    }

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "game_rv", &inst_state->game_rv_suid);

    inst_state->diag_service = (sns_diag_service *)service_mgr->get_service(service_mgr,
            SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config,
        sizeof(sensor_state->config));

    fp_display_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &fp_display_ops;

    fp_display_mTask = this;

    return rc;
}

sns_rc sns_fp_display_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);

    fp_display_close();

    return SNS_RC_SUCCESS;
}

sns_rc sns_fp_display_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        FP_DISPLAY_LOG_0("Sensor config");

        fp_display_Reset();
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        FP_DISPLAY_LOG_0("Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        FP_DISPLAY_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

