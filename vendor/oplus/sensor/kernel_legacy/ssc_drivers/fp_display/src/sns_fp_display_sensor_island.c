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

#include "sns_fp_display_sensor_instance.h"
#include "sns_fp_display_sensor.h"

static sns_sensor_uid const *sns_fp_display_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid suid = FP_DISPLAY_SUID;
    return &suid;
}

sns_sensor_api sns_fp_display_api = {
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_fp_display_init,
    .deinit = &sns_fp_display_deinit,
    .get_sensor_uid = &sns_fp_display_get_sensor_uid,
    .set_client_request = &sns_fp_display_set_client_request,
    .notify_event = &sns_fp_display_notify_event,
};

