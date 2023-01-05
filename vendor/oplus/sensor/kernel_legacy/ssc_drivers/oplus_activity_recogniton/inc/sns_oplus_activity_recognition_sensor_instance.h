/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - sns_oplus_activity_recognition_sensor_instance.h
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

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
#include "sns_oplus_activity_recognition_sensor.h"

#include "../oplus_activity_recognition_algo/oplus_activity_recognition_algo.h"

sns_rc sns_oplus_activity_recognition_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);

sns_rc sns_oplus_activity_recognition_inst_deinit(sns_sensor_instance *const this);

sns_rc sns_oplus_activity_recognition_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request);

typedef struct
{
    sns_sensor_uid remote_proc_sensor_suid;
    sns_data_stream *remote_proc_sensor_stream;

    sns_sensor_uid motion_recognition_sensor_suid;
    sns_data_stream *motion_recognition_sensor_stream;

    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;

    sns_oplus_activity_recognition_config config;
    oplus_activity_recognition_state *state;
} sns_oplus_activity_recognition_inst_state;
