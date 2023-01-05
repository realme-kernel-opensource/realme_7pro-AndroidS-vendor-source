/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - sns_oplus_activity_recognition_sensor_island.c
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

#include "sns_oplus_activity_recognition_sensor_instance.h"
#include "sns_oplus_activity_recognition_sensor.h"

static sns_sensor_uid const *sns_oplus_activity_recognition_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = OPLUS_ACTIVITY_RECOGNITION_SUID;
    return &suid;
}

sns_sensor_api sns_oplus_activity_recognition_api =
        {
            .struct_len = sizeof(sns_sensor_api),
            .init = &sns_oplus_activity_recognition_init,
            .deinit = &sns_oplus_activity_recognition_deinit,
            .get_sensor_uid = &sns_oplus_activity_recognition_get_sensor_uid,
            .set_client_request = &sns_oplus_activity_recognition_set_client_request,
            .notify_event = &sns_oplus_activity_recognition_notify_event,
        };

