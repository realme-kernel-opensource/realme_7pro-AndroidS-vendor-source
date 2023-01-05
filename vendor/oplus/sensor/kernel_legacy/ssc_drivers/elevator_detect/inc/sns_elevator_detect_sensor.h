/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: sns_elevator_detect_sensor.h
**
** Description:
**      Sensor data types for the algorithm.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                    <desc>
**************************************************************************************/
#pragma once

#include "sns_sensor_uid.h"
#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_resampler.pb.h"
#include "sns_sensor_util.h"
#include "sns_suid_util.h"
#include "sns_oem1.pb.h"
#include "sns_service_manager.h"
#include "sns_types.h"
#include "sns_suid.pb.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_stream_service.h"
#include "sns_registry.pb.h"
#include "sns_mem_util.h"
#include "sns_event_service.h"
#include "sns_attribute_service.h"
#include "sns_service.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_register.h"
#include "sns_sensor_instance.h"

#define ELEVATOR_DETECT_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xf8, 0x1f, 0x3f, 0xff, 0x2f, 0x72, 0x19, 0xa9, \
        0xa8, 0x90, 0x42, 0x53, 0x73, 0xf3, 0xe8, 0xe6  \
    }  \
}

typedef struct
{
    size_t encoded_data_event_len;
    float sample_rate;
} sns_elevator_detect_config;

typedef struct sns_elevator_detect_sensor_state
{
    sns_data_stream *registry_stream;
    sns_diag_service *diag_service;
    SNS_SUID_LOOKUP_DATA(5) suid_lookup_data; // resampler, accel, gamerotationvector, amd, timer
    sns_elevator_detect_config config;
} sns_elevator_detect_sensor_state;

sns_rc sns_elevator_detect_init(sns_sensor *const this);
sns_rc sns_elevator_detect_deinit(sns_sensor *const this);
sns_sensor_instance* sns_elevator_detect_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_elevator_detect_notify_event(sns_sensor *const this);

