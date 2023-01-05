/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: sns_motion_recognition_sensor_island.c
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

#include "sns_motion_recognition_sensor_instance.h"
#include "sns_motion_recognition_sensor.h"

static sns_sensor_uid const *sns_motion_recognition_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = MOTION_RECOGNITION_SUID;
    return &suid;
}

sns_sensor_api sns_motion_recognition_api =
        {
            .struct_len = sizeof(sns_sensor_api),
            .init = &sns_motion_recognition_init,
            .deinit = &sns_motion_recognition_deinit,
            .get_sensor_uid = &sns_motion_recognition_get_sensor_uid,
            .set_client_request = &sns_motion_recognition_set_client_request,
            .notify_event = &sns_motion_recognition_notify_event,
        };

