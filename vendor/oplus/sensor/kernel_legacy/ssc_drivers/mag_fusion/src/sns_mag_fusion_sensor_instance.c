/*=============================================================================
  @file sns_mag_fusion_sensor_instance.c

  The mag_fusion virtual Sensor Instance implementation

  Copyright (c) 2017 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_mag_fusion_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_diag_service.h"
#include "sns_printf.h"

/* See sns_sensor_instance_api::init */
sns_rc sns_mag_fusion_inst_init(sns_sensor_instance *this,
    sns_sensor_state const *state)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_mag_fusion_inst_state *inst_state =
        (sns_mag_fusion_inst_state*)this->state->state;
    sns_mag_fusion_sensor_state *sensor_state =
        (sns_mag_fusion_sensor_state*)state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_service = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "accel", &inst_state->accel_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "mag",   &inst_state->mag_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gyro",  &inst_state->gyro_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "gyro_cal",  &inst_state->gyro_cal_suid);
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "resampler",  &inst_state->resampler_suid);

    inst_state->diag_service = (sns_diag_service*)
        service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

    rc = stream_service->api->create_sensor_instance_stream(
            stream_service,
            this,
            inst_state->resampler_suid,
            &inst_state->accel_stream);
    rc = stream_service->api->create_sensor_instance_stream(
            stream_service,
            this,
            inst_state->resampler_suid,
            &inst_state->mag_stream);
    rc = stream_service->api->create_sensor_instance_stream(
            stream_service,
            this,
            inst_state->resampler_suid,
            &inst_state->gyro_stream);
    rc = stream_service->api->create_sensor_instance_stream(
            stream_service,
            this,
            inst_state->gyro_cal_suid,
            &inst_state->gyro_cal_stream);

    // restrict report rate to fit CTS test
    inst_state->max_report_interval = sns_convert_ns_to_ticks(200000);

    // read platform specific configuration
    inst_state->fusion_type = sensor_state->fusion_type;
    inst_state->fusion_api = sensor_state->fusion_api;

    // init vendor algo
    if(inst_state->fusion_api->algo_start)
        inst_state->fusion_api->algo_start();

    //reset sample
    memset(&inst_state->last_sample, 0, sizeof(mag_fusion_sample));
    return rc;
}

/* See sns_sensor_instance_api::deinit */
sns_rc sns_mag_fusion_inst_deinit(sns_sensor_instance *const this)
{
    sns_mag_fusion_inst_state *state =
        (sns_mag_fusion_inst_state*)this->state->state;

    sns_sensor_util_remove_sensor_instance_stream(this, &state->accel_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &state->mag_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &state->gyro_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &state->gyro_cal_stream);

    // deinit vendor algo
    if(state->fusion_api->algo_stop)
        state->fusion_api->algo_stop();

    MAG_FUSION_LOGI("inst deinit");

    return SNS_RC_SUCCESS;
}
