#pragma once
/*=============================================================================
  @file mag_fusion_sensor.h

  The MAG_FUSION virtual Sensor

  Copyright (c) 2017 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/

#include "sns_sensor_uid.h"
#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_resampler.pb.h"
#include "sns_sensor_util.h"
#include "sns_diag_service.h"
#include "sns_suid_util.h"

#include "fusion_api.h"

/*============================================================================
  Preprocessor Definitions and Constants
  ===========================================================================*/


/*=============================================================================
  Type Definitions
  ===========================================================================*/
#define MAG_FUSION_SUID \
{  \
  .sensor_uid =  \
  {  \
    0x07, 0x1e, 0x34, 0xc2, 0x2b, 0x6c, 0x4f, 0x40, \
    0xa7, 0xda, 0x18, 0x2f, 0xb0, 0x25, 0xb5, 0x05  \
  }  \
}

#define MAG_FUSION_GROUP_NAME "sns_mag_fusion.bias"

typedef enum mag_fusion_type {
    AKM_FUSION,
    OPLUS_FUSION,
    FUSION_TYPE_MAX
} mag_fusion_type;

typedef struct sns_mag_fusion_config {
    int accuracy;
    float bias[3];
} sns_mag_fusion_config;

typedef struct sns_mag_fusion_sensor_state {
    // Requests to the registry sensor
    sns_data_stream *registry_stream;
    bool registry_requested;

    // registry, accel, mag, gyro, gyro_cal
    SNS_SUID_LOOKUP_DATA(6) suid_lookup_data;
    sns_diag_service *diag_service;

    bool has_gyro;
    mag_fusion_type fusion_type;
    mag_fusion_api *fusion_api;
    sns_mag_fusion_config config;
} sns_mag_fusion_sensor_state;

//These are defined in sns_mag_fusion_sensor.c
// and referred in vtable in sns_mag_fusion_sensor_island.c
sns_rc sns_mag_fusion_init(sns_sensor *const this);
sns_rc sns_mag_fusion_deinit(sns_sensor *const this);
sns_sensor_instance* sns_mag_fusion_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_mag_fusion_notify_event(sns_sensor *const this);

