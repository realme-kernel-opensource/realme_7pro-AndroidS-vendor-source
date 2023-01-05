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

#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_printf.h"
#include "sns_mem_util.h"
#include "sns_std_sensor.pb.h"
#include "sns_request.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_time.h"
#include "sns_timer.pb.h"

#include "sns_sensor_instance.h"
#include "sns_oplus_monitor_sensor.h"
#include "oplus_sensor_fb.h"
#include "oplus_fb_utils.h"

sns_rc sns_oplus_monitor_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);
sns_rc sns_oplus_monitor_inst_deinit(sns_sensor_instance *const this);
sns_rc sns_oplus_monitor_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request);
sns_rc oplus_monitor_start_timer(sns_sensor_instance const *this, int period);

#define TIMER_PERIOD 60000 //60000ms = 1min
#define FIRST_TIMER_PERIOD 5000 //5000ms = 5S
#define REPORT_EVENT_TIMEOUT 120 //120 min

typedef struct {
    sns_sensor_uid self_suid;

    sns_data_stream *timer_stream;
    sns_sensor_uid timer_suid;

    sns_oplus_monitor_config config;
    unsigned int timer_count;
    bool fw_is_inited;
} sns_oplus_monitor_inst_state;
