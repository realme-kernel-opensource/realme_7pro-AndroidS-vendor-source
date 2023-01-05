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

#include "sns_lux_aod_sensor_instance.h"
#include "sns_lux_aod_sensor.h"

static sns_sensor_uid const *sns_lux_aod_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = LUX_AOD_SUID;
    return &suid;
}

sns_sensor_api sns_lux_aod_api = {
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_lux_aod_init,
    .deinit = &sns_lux_aod_deinit,
    .get_sensor_uid = &sns_lux_aod_get_sensor_uid,
    .set_client_request = &sns_lux_aod_set_client_request,
    .notify_event = &sns_lux_aod_notify_event,
};

