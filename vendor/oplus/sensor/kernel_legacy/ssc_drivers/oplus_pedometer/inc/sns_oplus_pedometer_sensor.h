/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: sns_xxx.c
**
** Description:
**      Definitions for  detect algorithem .
**
** Version: 1.0
** Date created: 2020/05/09,11:50
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

#define OPLUS_PEDOMETER_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xf1, 0x22, 0x31, 0xf1, 0x21, 0x71, 0x11, 0xa1, \
        0xa1, 0x91, 0x41, 0x51, 0x71, 0xf1, 0x22, 0xe1  \
    }  \
}

typedef struct
{
    size_t encoded_data_event_len;
    float sample_rate;
} sns_oplus_pedometer_config;

typedef struct sns_oplus_pedometer_sensor_state
{
    sns_sensor_uid self_suid;
    sns_data_stream *registry_stream;
    sns_diag_service *diag_service;
    SNS_SUID_LOOKUP_DATA(4) suid_lookup_data; // resampler, accel, gavity, gamerotationvector
    sns_oplus_pedometer_config config;
} sns_oplus_pedometer_sensor_state;

sns_rc sns_oplus_pedometer_init(sns_sensor *const this);
sns_rc sns_oplus_pedometer_deinit(sns_sensor *const this);
sns_sensor_instance* sns_oplus_pedometer_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_oplus_pedometer_notify_event(sns_sensor *const this);

