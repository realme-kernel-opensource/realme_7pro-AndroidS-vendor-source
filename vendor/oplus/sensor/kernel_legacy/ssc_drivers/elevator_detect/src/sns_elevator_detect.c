/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: sns_elevator_detect.c
**
** Description:
**      Functions to register algorithm library.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_elevator_detect_sensor.h"

extern sns_sensor_instance_api sns_elevator_detect_sensor_instance_api;
extern sns_sensor_api sns_elevator_detect_api;

sns_rc sns_elevator_detect_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_elevator_detect_sensor_state),
                            &sns_elevator_detect_api,
                            &sns_elevator_detect_sensor_instance_api);

    return SNS_RC_SUCCESS;
}

