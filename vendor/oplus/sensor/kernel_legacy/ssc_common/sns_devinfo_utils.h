#pragma once
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

#include <string.h>
#include <stdio.h>

//sensor_id list
enum {
    UNKNOW = -1,
    OPLUS_ALS,
    OPLUS_PS,
    OPLUS_STRUCTURE_PS,
    OPLUS_GSENSOR,
    OPLUS_MAG,
    OPLUS_GYRO,
    OPLUS_RGB,
    OPLUS_RGB_REAR,
    OPLUS_PRESS,
    MAX
};

struct devinfo {
    const char *cal_path;
};

extern void register_sensor_devinfo(uint8_t sensor_id, struct devinfo *info);

