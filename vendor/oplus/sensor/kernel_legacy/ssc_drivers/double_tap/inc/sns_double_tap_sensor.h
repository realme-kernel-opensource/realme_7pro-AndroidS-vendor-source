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

#define DOUBLE_TAP_SUID \
    {  \
        .sensor_uid =  \
            {  \
                0xf2, 0x15, 0x35, 0xf5, 0x25, 0x75, 0x15, 0xa5, \
                0xa6, 0x96, 0x45, 0x56, 0x76, 0xf0, 0xe6, 0xe1  \
            }  \
    }

typedef struct {
    size_t encoded_data_event_len;
    float sample_rate;
} sns_double_tap_config;

typedef struct sns_double_tap_sensor_state {
    sns_data_stream *registry_stream;
    sns_diag_service *diag_service;
    SNS_SUID_LOOKUP_DATA(5) suid_lookup_data; // resampler, proximity, accel, double_tap
    sns_double_tap_config config;
} sns_double_tap_sensor_state;

sns_rc sns_double_tap_init(sns_sensor *const this);
sns_rc sns_double_tap_deinit(sns_sensor *const this);
sns_sensor_instance *sns_double_tap_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_double_tap_notify_event(sns_sensor *const this);

