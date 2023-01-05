/************************************************************************************
# Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
# OPLUS_FEATURE_SENSOR
# File: step_counter.cpp
#
# Description:
#      Definitions for devinfo sensor.
#
# Version: 1.0
# Date created: 2018/03/09,20:27
#
# --------------------------- Revision History: ------------------------------------
# <version>     <date>      <author>        <desc>
**************************************************************************************/
#include "sns_devinfo_sensor.h"
#include "sns_register.h"
#include "sns_sensor.h"
#include "sns_sensor_instance.h"

extern sns_sensor_api sns_devinfo_sensor_api;

sns_rc sns_register_devinfo_sensor(sns_register_cb const *register_api)
{
    register_api->init_sensor(sizeof(sns_devinfo_sensor_state),
        &sns_devinfo_sensor_api, NULL);

    return SNS_RC_SUCCESS;
}
