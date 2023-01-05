/*=============================================================================
  @file sns_mag_fusion.c

  MagFusion implementation

  Copyright (c) 2018 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/

#include "sns_sensor.h"
#include "sns_register.h"
#include "sns_mag_fusion_sensor.h"
#include "sns_sensor_instance.h"

/*=============================================================================
External Variable Declarations
===========================================================================*/

extern sns_sensor_instance_api sns_mag_fusion_sensor_instance_api;
extern sns_sensor_api sns_mag_fusion_api;

/*=============================================================================
Public Function Definitions
===========================================================================*/

sns_rc sns_mag_fusion_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_mag_fusion_sensor_state),
        &sns_mag_fusion_api,
        &sns_mag_fusion_sensor_instance_api);

    return SNS_RC_SUCCESS;
}

