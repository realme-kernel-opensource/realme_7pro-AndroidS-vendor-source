/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_xx.x
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#pragma once
#include "sns_sensor_uid.h"
#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_sensor_util.h"
#include "sns_suid_util.h"
#include "sns_service_manager.h"
#include "sns_types.h"
#include "sns_suid.pb.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_stream_service.h"
#include "sns_registry.pb.h"
#include "sns_timer.pb.h"
#include "sns_mem_util.h"
#include "sns_event_service.h"
#include "sns_attribute_service.h"
#include "sns_service.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_register.h"
#include "sns_sensor_instance.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "oplus_sensor_fb.h"
#include "oplus_fb_utils.h"

#define OPLUS_MONITOR_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xf2, 0x11, 0x31, 0x41, 0x21, 0x73, 0x11, 0xa5, \
        0x21, 0x91, 0x82, 0x51, 0x71, 0xf1, 0xe2, 0xe1  \
    }  \
}


typedef struct {
    size_t encoded_data_event_len;
    float sample_rate;
} sns_oplus_monitor_config;

typedef struct sns_oplus_monitor_sensor_state {
    sns_sensor_uid self_suid;
    // timer
    SNS_SUID_LOOKUP_DATA(1) suid_lookup_data;
    sns_oplus_monitor_config config;
} sns_oplus_monitor_sensor_state;

sns_rc sns_oplus_monitor_init(sns_sensor *const this);
sns_rc sns_oplus_monitor_deinit(sns_sensor *const this);
sns_sensor_instance* sns_oplus_monitor_set_client_request(
    sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);

sns_rc sns_oplus_monitor_notify_event(sns_sensor *const this);

