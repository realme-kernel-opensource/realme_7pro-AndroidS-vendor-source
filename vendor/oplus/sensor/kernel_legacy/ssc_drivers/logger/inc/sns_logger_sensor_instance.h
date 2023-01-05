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
#include "sns_timer.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_proximity.pb.h"
#include "sns_request.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_time.h"

#include "sns_sensor_instance.h"
#include "sns_logger_sensor.h"

#define LOG_BUFFER_SIZE 30
#define TIMER_EXECUTE_PERIOD 40 // ms

typedef enum {
    SENSOR_TYPE_ACCELEROMETER = 1,
    SENSOR_TYPE_GYROSCOPE = 4,
    SENSOR_TYPE_LIGHT = 5,
    SENSOR_TYPE_RGB,
    SENSOR_TYPE_FLICKER,
    SENSOR_TYPE_PROXIMITY = 8,
    SENSOR_TYPE_STEP_COUNTER = 19,

} sensor_type_android;

typedef enum {
    ALSPS_APDS9922,     // 0
    ALSPS_STK3210,      // 1
    ALSPS_CM36286,      // 2
    ALSPS_TMD2725,      // 3
    ALSPS_STK3335,      // 4
    ALSPS_VCNL36658,    // 5
    ALSPS_STK3331,      // 6

} alsps_type;

typedef enum {
    SNS_LOGGER_AUTO_DEBUG0,
    SNS_LOGGER_AUTO_DEBUG1,
    SNS_LOGGER_AUTO_DEBUG2,
    SNS_LOGGER_AUTO_DEBUG3,
    SNS_LOGGER_AUTO_DEBUG4,
} sns_logger_common_str;

typedef enum {
    ALSPS_LOG_ENABLE_SCHED_DATA = 1,            // 1
    ALSPS_LOG_ALGO_GET = 5,                     // 5
    ALSPS_LOG_PROX_CONDITION = 18,               // 18
    ALSPS_LOG_LIGHT_PARAMETER = 19,               // 19
    ALSPS_LOG_PROX_PARAMETER = 20,               // 20
} alsps_log_string;

typedef enum {
    ACC_LOG_ENABLE_SCHED_DATA = 5,            // 5
    ACC_LOG_STEP_COUNT,                   // 6
    ACC_LOG_STEP_COUNT_AFTER_SLEEP,       // 7

} acc_log_string;

typedef struct {
    int sensor_id;
    int string_id;
    int argu2;
    int argu3;
    int argu4;
    int argu5;
    int argu6;

} log_data_info;

typedef struct {
    bool to_report;
    log_data_info log_data;
} logger_data;

typedef struct {
    sns_sensor_instance *owner;

    sns_sensor_uid  timer_suid;
    sns_data_stream *timer_data_stream;

    sns_sensor_uid  logger_suid;

    sns_diag_service *diag_service;
    sns_std_sensor_config client_config;

    sns_logger_config config;

    bool enabled;
    bool timer_active;
    logger_data data_buffer[LOG_BUFFER_SIZE];

} sns_logger_inst_state;


sns_rc sns_logger_inst_init(sns_sensor_instance *this, sns_sensor_state const *state);
sns_rc sns_logger_inst_deinit(sns_sensor_instance *const this);
sns_rc sns_logger_inst_set_client_config(sns_sensor_instance *const this,
    sns_request const *client_request);


extern void sensors_log_report(log_data_info log_data);


