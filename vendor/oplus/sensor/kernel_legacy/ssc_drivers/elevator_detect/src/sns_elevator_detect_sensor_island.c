/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: sns_elevator_detect_sensor_island.c
**
** Description:
**      Island mode fucntions for elevator detect.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_elevator_detect_sensor_instance.h"
#include "sns_elevator_detect_sensor.h"

static sns_sensor_uid const* sns_elevator_detect_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = ELEVATOR_DETECT_SUID;
    return &suid;
}

sns_sensor_api sns_elevator_detect_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_elevator_detect_init,
    .deinit = &sns_elevator_detect_deinit,
    .get_sensor_uid = &sns_elevator_detect_get_sensor_uid,
    .set_client_request = &sns_elevator_detect_set_client_request,
    .notify_event = &sns_elevator_detect_notify_event,
};

