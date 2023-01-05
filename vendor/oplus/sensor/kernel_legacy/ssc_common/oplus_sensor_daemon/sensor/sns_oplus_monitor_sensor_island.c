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
#include "sns_oplus_monitor_sensor_instance.h"
#include "sns_oplus_monitor_sensor.h"

static sns_sensor_uid const* sns_oplus_monitor_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = OPLUS_MONITOR_SUID;
    return &suid;
}

sns_sensor_api sns_oplus_monitor_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_oplus_monitor_init,
    .deinit = &sns_oplus_monitor_deinit,
    .get_sensor_uid = &sns_oplus_monitor_get_sensor_uid,
    .set_client_request = &sns_oplus_monitor_set_client_request,
    .notify_event = &sns_oplus_monitor_notify_event,
};
