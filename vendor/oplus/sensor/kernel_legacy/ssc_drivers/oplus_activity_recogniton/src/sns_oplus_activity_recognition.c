/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - sns_oplus_activity_recognition.c
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

#include "sns_oplus_activity_recognition_sensor.h"

extern sns_sensor_instance_api sns_oplus_activity_recognition_sensor_instance_api;
extern sns_sensor_api sns_oplus_activity_recognition_api;

sns_rc sns_oplus_activity_recognition_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_oplus_activity_recognition_sensor_state),
                              &sns_oplus_activity_recognition_api,
                              &sns_oplus_activity_recognition_sensor_instance_api);

    return SNS_RC_SUCCESS;
}

