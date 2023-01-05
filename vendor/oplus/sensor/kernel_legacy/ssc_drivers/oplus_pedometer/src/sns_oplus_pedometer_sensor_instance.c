/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: sns_xxx.c
**
** Description:
**      Definitions for  detect algorithem .
**
** Version: 1.0
** Date created: 2020/05/09,11:50
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "sns_oplus_pedometer_sensor_instance.h"
#include "sns_pedometer.pb.h"

extern struct pedometer_detect_sensor_operation oplus_pedometer_ops;
extern sns_sensor_instance *oplus_pedometer_mTask;

sns_rc sns_oplus_pedometer_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)this->state->state;
    sns_oplus_pedometer_sensor_state *sensor_state = (sns_oplus_pedometer_sensor_state*)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    inst_state->self_suid = sensor_state->self_suid;

    OPLUS_PEDOMETER_DETECT_LOG_0("sns_oplus_pedometer_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->accel_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gravity", &inst_state->gravity_suid);

    inst_state->diag_service = (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config, sizeof(sensor_state->config));

    oplus_pedometer_detect_algo_init(&inst_state->state);

    inst_state->state->sensor_ops = &oplus_pedometer_ops;

    oplus_pedometer_mTask = this;

    return rc;
}

sns_rc sns_oplus_pedometer_inst_deinit(sns_sensor_instance *const this)
{
    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)this->state->state;

    oplus_pedometer_detect_close();
    sns_step_event_config oplus_pedometer_config = sns_step_event_config_init_default;
    oplus_pedometer_config.step_count = oplus_pedometer_detect_get_total_data();
    pb_send_event(oplus_pedometer_mTask,
                  sns_step_event_config_fields,
                  &oplus_pedometer_config,
                  sns_get_system_time(),
                  SNS_PEDOMETER_MSGID_SNS_STEP_EVENT_CONFIG,
                  &inst_state->self_suid);

    return SNS_RC_SUCCESS;
}

sns_rc sns_oplus_pedometer_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)this->state->state;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        OPLUS_PEDOMETER_DETECT_LOG_0("Sensor config");

        oplus_pedometer_detect_reset();

        sns_step_event_config oplus_pedometer_config = sns_step_event_config_init_default;
        oplus_pedometer_config.step_count = oplus_pedometer_detect_get_total_data();
        pb_send_event(oplus_pedometer_mTask,
                      sns_step_event_config_fields,
                      &oplus_pedometer_config,
                      sns_get_system_time(),
                      SNS_PEDOMETER_MSGID_SNS_STEP_EVENT_CONFIG,
                      &inst_state->self_suid);

        OPLUS_PEDOMETER_DETECT_LOG_0("send first data");
        sns_step_event oplus_pedometer_event = sns_step_event_init_default;
        oplus_pedometer_event.step_count = oplus_pedometer_detect_get_total_data();
        pb_send_event(oplus_pedometer_mTask,
                      sns_step_event_fields,
                      &oplus_pedometer_event,
                      sns_get_system_time(),
                      SNS_PEDOMETER_MSGID_SNS_STEP_EVENT,
                      &inst_state->self_suid);
    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        OPLUS_PEDOMETER_DETECT_LOG_0("Flush req");

        sns_sensor_util_send_flush_event(NULL, this);
    } else {
        OPLUS_PEDOMETER_DETECT_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

