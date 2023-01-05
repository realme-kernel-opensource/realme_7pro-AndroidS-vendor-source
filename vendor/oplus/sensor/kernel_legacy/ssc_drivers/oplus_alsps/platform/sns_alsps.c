/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#include "sns_rc.h"
#include "sns_register.h"
#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"

extern sns_sensor_api als_sensor_api;
extern sns_sensor_api ps_sensor_api;
extern sns_sensor_instance_api alsps_sensor_instance_api;

sns_rc sns_register_alsps(sns_register_cb const *register_api)
{
    /* Register Ambient Light Sensor. */
    register_api->init_sensor(sizeof(alsps_state), &als_sensor_api,
        &alsps_sensor_instance_api);

    /* Register ps Sensor. */
    register_api->init_sensor(sizeof(alsps_state), &ps_sensor_api,
        &alsps_sensor_instance_api);

    return SNS_RC_SUCCESS;
}
