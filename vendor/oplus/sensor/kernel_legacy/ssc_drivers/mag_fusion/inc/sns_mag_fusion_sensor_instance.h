#pragma once
/*=============================================================================
  @file mag_fusion_sensor_instance.h

  The mag_fusion virtual Sensor Instance

  Copyright (c) 2017 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/
#include "sns_sensor_instance.h"
#include "sns_mag_fusion_sensor.h"
#include "sns_std_sensor.pb.h"
#include "sns_diag_service.h"
#include "sns_printf.h"

/*============================================================================
  Preprocessor Definitions and Constants
  ===========================================================================*/
/* Number of elements in the computed mag_fusion vector */
#define MAG_FUSION_NUM_ELEMS      (3)

/*============================================================================
  Type Declarations
  ===========================================================================*/

sns_rc sns_mag_fusion_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);
sns_rc sns_mag_fusion_inst_deinit(sns_sensor_instance *const this);
sns_rc sns_mag_fusion_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request);

typedef struct sns_mag_fusion_inst_state {
    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;
    mag_fusion_api *fusion_api;

    sns_sensor_uid accel_suid;
    sns_data_stream *accel_stream;

    sns_sensor_uid mag_suid;
    sns_data_stream *mag_stream;

    sns_sensor_uid gyro_suid;
    sns_data_stream *gyro_stream;

    sns_sensor_uid gyro_cal_suid;
    sns_data_stream * gyro_cal_stream;
    sns_sensor_uid resampler_suid;

    float gyro_bias[3];

    mag_fusion_type fusion_type;
    sns_time max_report_interval;
    mag_fusion_sample last_sample;
} sns_mag_fusion_inst_state;

bool encode_send_sns_cal_event(
    sns_sensor_instance *const this,
    mag_fusion_sample *const output_sample,
    bool new_request_report);

mag_fusion_api * mag_fusion_get_fusion_list(mag_fusion_type fusion_type);

