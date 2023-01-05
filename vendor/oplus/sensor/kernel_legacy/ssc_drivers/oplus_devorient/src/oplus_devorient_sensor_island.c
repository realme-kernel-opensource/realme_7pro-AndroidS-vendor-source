/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "oplus_devorient_instance.h"
#include "oplus_devorient_sensor.h"

static sns_sensor_uid const* devorient_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = OPLUS_DEVORIENT_SUID;
    return &suid;
}

sns_sensor_api devorient_sensor_api = {
    .struct_len = sizeof(sns_sensor_api),
    .init = &devorient_init,
    .deinit = &devorient_deinit,
    .get_sensor_uid = &devorient_get_sensor_uid,
    .set_client_request = &devorient_set_client_request,
    .notify_event = &devorient_notify_event,
};