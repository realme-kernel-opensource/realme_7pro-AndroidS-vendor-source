/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: sns_elevator_detect_sensor_instance.c
**
** Description:
**      Normal funcitons for the elevator detect algorithm.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_elevator_detect_sensor_instance.h"

extern struct elevator_detect_sensor_operation elevator_detect_ops;
extern sns_sensor_instance *elevator_detect_mTask;

sns_rc sns_elevator_detect_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_elevator_detect_inst_state *inst_state = (sns_elevator_detect_inst_state*)this->state->state;
    sns_elevator_detect_sensor_state *sensor_state = (sns_elevator_detect_sensor_state*)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    ELEVATOR_DETECT_LOG_0("sns_elevator_detect_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->accel_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "game_rv", &inst_state->game_rv_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "timer", &inst_state->timer_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "amd", &inst_state->amd_suid);

    inst_state->diag_service = (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config, sizeof(sensor_state->config));

    elevator_detect_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &elevator_detect_ops;

    elevator_detect_mTask = this;

    return rc;
}

sns_rc sns_elevator_detect_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);

    elevator_detect_close();

    return SNS_RC_SUCCESS;
}

sns_rc sns_elevator_detect_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        ELEVATOR_DETECT_LOG_0("Sensor config");

        elevator_detect_Reset();
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        ELEVATOR_DETECT_LOG_0("Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        ELEVATOR_DETECT_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

