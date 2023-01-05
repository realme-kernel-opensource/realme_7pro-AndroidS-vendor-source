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
#include "sns_request.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_time.h"

#include "sns_sensor_instance.h"
#include "sns_free_fall_sensor.h"

#include "../free_fall_algo/free_fall_algo.h"

sns_rc sns_free_fall_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);
sns_rc sns_free_fall_inst_deinit(sns_sensor_instance *const this);
sns_rc sns_free_fall_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request);

typedef struct {
    float accel_sample[3];
    sns_time accel_sample_ts;
} sns_free_fall_input;

typedef struct {
    sns_sensor_uid resampler_suid;

    sns_sensor_uid accel_suid;
    sns_data_stream *resampler_accel_stream;

    sns_sensor_uid gravity_suid;
    sns_data_stream *resampler_gravity_stream;

    sns_sensor_uid game_rv_suid;
    sns_data_stream *resampler_game_rv_stream;

    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;

    sns_free_fall_config config;
    free_fall_state *state;
} sns_free_fall_inst_state;
