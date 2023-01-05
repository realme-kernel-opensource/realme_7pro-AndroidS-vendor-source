/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: sns_motion_recognition.c
**
** Description:
**      The specific algorithm for how to detect motion state.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_motion_recognition_sensor.h"

extern sns_sensor_instance_api sns_motion_recognition_sensor_instance_api;
extern sns_sensor_api sns_motion_recognition_api;

sns_rc sns_motion_recognition_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_motion_recognition_sensor_state),
                              &sns_motion_recognition_api,
                              &sns_motion_recognition_sensor_instance_api);

    return SNS_RC_SUCCESS;
}

