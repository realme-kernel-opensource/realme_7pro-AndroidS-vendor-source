/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for free fall detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "sns_tp_gesture_sensor.h"
#include "oppo_sensor.h"

extern sns_sensor_instance_api sns_tp_gesture_sensor_instance_api;
extern sns_sensor_api sns_tp_gesture_api;

sns_rc sns_tp_gesture_register(sns_register_cb const *register_api)
{
    struct sensor_algorithm *smem = NULL;
    if (oppo_get_virtual_sensor(OPPO_TP_GESTURE, &smem)) {
        register_api->init_sensor(sizeof(sns_tp_gesture_sensor_state),
            &sns_tp_gesture_api,
            &sns_tp_gesture_sensor_instance_api);
    }
    return SNS_RC_SUCCESS;
}

