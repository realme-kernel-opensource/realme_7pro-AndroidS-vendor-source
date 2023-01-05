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

#include "oplus_devorient_sensor.h"

extern sns_sensor_instance_api devorient_sensor_instance_api;
extern sns_sensor_api devorient_sensor_api;

sns_rc devorient_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(devorient_sensor_state),
        &devorient_sensor_api,
        &devorient_sensor_instance_api);

    return SNS_RC_SUCCESS;
}
