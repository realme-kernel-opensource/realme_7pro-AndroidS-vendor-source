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

#define FREE_FALL_SUID \
    {  \
        .sensor_uid =  \
            {  \
                0xf1, 0x11, 0x31, 0xf1, 0x21, 0x71, 0x11, 0xa1, \
                0xa1, 0x91, 0x41, 0x51, 0x71, 0xf1, 0xe1, 0xe1  \
            }  \
    }

typedef struct {
    size_t encoded_data_event_len;
    float sample_rate;
} sns_free_fall_config;

typedef struct sns_free_fall_sensor_state {
    sns_data_stream *registry_stream;
    sns_diag_service *diag_service;
    SNS_SUID_LOOKUP_DATA(4) suid_lookup_data; // resampler, accel, gavity, gamerotationvector
    sns_free_fall_config config;
} sns_free_fall_sensor_state;

sns_rc sns_free_fall_init(sns_sensor *const this);
sns_rc sns_free_fall_deinit(sns_sensor *const this);
sns_sensor_instance *sns_free_fall_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_free_fall_notify_event(sns_sensor *const this);

