/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_xx.x
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "sns_oplus_monitor_sensor.h"

extern sns_sensor_instance_api sns_oplus_monitor_sensor_instance_api;
extern sns_sensor_api sns_oplus_monitor_api;

sns_rc sns_oplus_monitor_register(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_oplus_monitor_sensor_state),
                            &sns_oplus_monitor_api,
                            &sns_oplus_monitor_sensor_instance_api);

    return SNS_RC_SUCCESS;
}
