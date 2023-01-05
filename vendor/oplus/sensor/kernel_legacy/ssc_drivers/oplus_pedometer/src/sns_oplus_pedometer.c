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

#include "sns_oplus_pedometer_sensor.h"

extern sns_sensor_instance_api sns_oplus_pedometer_sensor_instance_api;
extern sns_sensor_api sns_oplus_pedometer_api;

sns_rc sns_oplus_pedometer_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_oplus_pedometer_sensor_state),
                            &sns_oplus_pedometer_api,
                            &sns_oplus_pedometer_sensor_instance_api);

    return SNS_RC_SUCCESS;
}

