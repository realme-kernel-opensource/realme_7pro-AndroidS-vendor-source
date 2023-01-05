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

#include "sns_pedometer_minute_sensor_instance.h"

extern struct pedometer_minute_sensor_operation  pedometer_minute_ops;
extern sns_sensor_instance *pedometer_minute_mTask;

sns_rc sns_pedometer_minute_inst_init(sns_sensor_instance *this, sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_pedometer_minute_inst_state *inst_state = (sns_pedometer_minute_inst_state*)this->state->state;
    sns_pedometer_minute_sensor_state *sensor_state = (sns_pedometer_minute_sensor_state*)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);

    PEDEMETER_LOG_0("sns_pedometer_minute_inst_init enter");

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler", &inst_state->resampler_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "timer", &inst_state->timer_suid);
#ifdef SW_STEPCNT
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "pedometer", &inst_state->step_counter_suid);
#else
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "step_count", &inst_state->step_counter_suid);
#endif
#ifdef REGISTRY_CMC
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "motion_recognition", &inst_state->cmc_sensor_uid);
#endif

    inst_state->diag_service = (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    // read platform specific configuration
    sns_memscpy(&inst_state->config, sizeof(inst_state->config), &sensor_state->config, sizeof(sensor_state->config));

    pedometer_minute_algo_register(&inst_state->state);

    inst_state->state->sensor_ops = &pedometer_minute_ops;

    pedometer_minute_mTask = this;

    return rc;
}

sns_rc sns_pedometer_minute_inst_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);

    pedometer_minute_close();

    return SNS_RC_SUCCESS;
}

