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
#include "oppo_sensor.h"

#define CAMERA_PROTECT_SUID \
    {  \
        .sensor_uid =  \
            {  \
                0xff, 0x1f, 0x3f, 0xff, 0x2f, 0x7f, 0x1f, 0xaf, \
                0xaf, 0x9f, 0x4f, 0x5f, 0x7f, 0xff, 0xef, 0xef  \
            }  \
    }

typedef struct {
    size_t encoded_data_event_len;
    float sample_rate;
    struct sensor_algorithm *smem;
} sns_camera_protect_config;

typedef struct sns_camera_protect_sensor_state {
    sns_data_stream *registry_stream;
    sns_diag_service *diag_service;
    SNS_SUID_LOOKUP_DATA(3) suid_lookup_data; // resampler, accel gyro
    sns_camera_protect_config config;
} sns_camera_protect_sensor_state;

sns_rc sns_camera_protect_init(sns_sensor *const this);
sns_rc sns_camera_protect_deinit(sns_sensor *const this);
sns_sensor_instance *sns_camera_protect_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_camera_protect_notify_event(sns_sensor *const this);

