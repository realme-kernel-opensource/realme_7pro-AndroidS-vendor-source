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

#include "sns_data_stream.h"
#include "sns_sensor_uid.h"
#include "sns_suid_util.h"

/** Sensor devinfo state structure */
typedef struct sns_devinfo_sensor_state {
    /* Registry SUID Lookup */
    SNS_SUID_LOOKUP_DATA(1) suid_lookup_data;
    /* Registry data stream */
    sns_data_stream         *reg_stream;
} sns_devinfo_sensor_state;

