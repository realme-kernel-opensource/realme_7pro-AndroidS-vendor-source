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

#include "sns_logger_sensor_instance.h"

extern sns_logger_inst_state *g_logger_state;;
extern int data_count;
extern int timer_count;

sns_rc sns_logger_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_logger_inst_state *inst_state = (sns_logger_inst_state *)this->state->state;
    sns_logger_sensor_state *sensor_state = (sns_logger_sensor_state *)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    SNS_INST_PRINTF(ERROR, this, "sns_logger_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "timer", &inst_state->timer_suid);

    inst_state->diag_service = (sns_diag_service *)service_mgr->get_service(service_mgr,
            SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config),
        &sensor_state->config, sizeof(sensor_state->config));

    sns_memscpy(&inst_state->logger_suid, sizeof(inst_state->logger_suid),
        &((sns_sensor_uid)LOGGER_SUID), sizeof(inst_state->logger_suid));

    inst_state->owner = this;

    g_logger_state = inst_state;

    return rc;
}

sns_rc sns_logger_inst_deinit(sns_sensor_instance *const this)
{
    sns_logger_inst_state *inst_state = (sns_logger_inst_state *)this->state->state;

    SNS_INST_PRINTF(ERROR, this, "sns_logger_inst_deinit");

    inst_state->enabled = false;
    inst_state->timer_active = false;
    sns_busy_wait(sns_convert_ns_to_ticks(2 * 1000 *
            1000));//wait for 2ms, to wait previous sensors_log_report running completely
    sns_sensor_util_remove_sensor_instance_stream(this, &inst_state->timer_data_stream);
    g_logger_state = NULL;
    return SNS_RC_SUCCESS;
}

sns_rc sns_logger_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_logger_inst_state *inst_state = (sns_logger_inst_state *)this->state->state;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        SNS_INST_PRINTF(ERROR, this, "Sensor config");
        //make sure the add to and report from data_buffer at the same place, and memset previous data.
        data_count = 0;
        timer_count = 0;
        memset(inst_state->data_buffer, 0, sizeof(inst_state->data_buffer));

        inst_state->enabled = true;
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        SNS_INST_PRINTF(ERROR, this, "Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        SNS_INST_PRINTF(ERROR, this, "Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

