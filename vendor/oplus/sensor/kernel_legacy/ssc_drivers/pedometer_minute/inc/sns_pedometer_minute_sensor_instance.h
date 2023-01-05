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
#include "sns_timer.pb.h"
#include "sns_sensor_instance.h"
#include "sns_pedometer_minute_sensor.h"

#include "../pedometer_minute_algo/pedometer_minute_algo.h"
#include "sns_cmc.pb.h"

sns_rc sns_pedometer_minute_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);
sns_rc sns_pedometer_minute_inst_deinit(sns_sensor_instance *const this);
sns_rc sns_pedometer_minute_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request);

typedef struct
{
    float step_counter_count;
    sns_time step_counter_sample_ts;
} sns_pedometer_minute_input;


typedef struct
{
    float    desired_sample_rate;
}sns_pedometer_minute_req;

typedef struct
{
    sns_sensor_uid resampler_suid;

    sns_sensor_uid step_counter_suid;
    sns_data_stream *resampler_step_counter_stream;
    sns_sensor_uid timer_suid;
    sns_data_stream *timer_data_stream;
#ifdef REGISTRY_CMC
    sns_sensor_uid cmc_sensor_uid;
    sns_data_stream *cmc_sensor_stream;
#endif
    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;

    sns_pedometer_minute_config config;
    pedometer_minute_state *state;
} sns_pedometer_minute_inst_state;
