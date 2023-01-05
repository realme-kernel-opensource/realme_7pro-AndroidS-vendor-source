/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: sns_motion_recognition_sensor_instance.h
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

#pragma once

#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_printf.h"
#include "sns_mem_util.h"
#include "sns_oem1.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_proximity.pb.h"
#include "sns_request.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_time.h"
#include "sns_timer.pb.h"
#include "sns_sensor_instance.h"
#include "sns_motion_recognition_sensor.h"

#include "../motion_recognition_algo/motion_recognition_algo.h"

sns_rc sns_motion_recognition_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);

sns_rc sns_motion_recognition_inst_deinit(sns_sensor_instance *const this);

sns_rc sns_motion_recognition_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request);

typedef struct
{
    sns_sensor_uid resampler_suid;

    sns_sensor_uid accel_suid;
    sns_data_stream *resampler_accel_20_stream;

    sns_sensor_uid amd_suid;
    sns_data_stream *amd_data_stream;

    sns_sensor_uid timer_suid;
    sns_data_stream *timer_data_stream;

    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;

    sns_motion_recognition_config config;
    motion_recognition_state *state;
} sns_motion_recognition_inst_state;
