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

#include "sns_oplus_pedometer_sensor_instance.h"
#include "sns_oplus_pedometer_sensor.h"

static sns_sensor_uid const* sns_oplus_pedometer_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = OPLUS_PEDOMETER_SUID;
    return &suid;
}

sns_sensor_api sns_oplus_pedometer_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_oplus_pedometer_init,
    .deinit = &sns_oplus_pedometer_deinit,
    .get_sensor_uid = &sns_oplus_pedometer_get_sensor_uid,
    .set_client_request = &sns_oplus_pedometer_set_client_request,
    .notify_event = &sns_oplus_pedometer_notify_event,
};

