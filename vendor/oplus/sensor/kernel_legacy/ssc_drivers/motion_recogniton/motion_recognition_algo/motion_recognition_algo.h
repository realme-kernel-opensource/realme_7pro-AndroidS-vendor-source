/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: motion_recognition_algo.h
**
** Description:
**      The specific algorithm for how to detect motion state.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#ifndef MOTION_RECOGNITION_ALGO_H
#define MOTION_RECOGNITION_ALGO_H

#include "model_parameter.h"

#define STAG "MOTION_RECOGNITION - "

#ifdef CFG_MTK_SCP_MOTION_RECOGNITION

#include "algoConfig.h"
#include <performance.h>
#include "contexthub_fw.h"
#define MOTION_RECOGNITION_LOG

#ifdef MOTION_RECOGNITION_LOG
#define MOTION_RECOGNITION_LOG_0(x...) osLog(LOG_ERROR, STAG x)
#define MOTION_RECOGNITION_LOG_1(x...) osLog(LOG_ERROR, STAG x)
#define MOTION_RECOGNITION_LOG_2(x...) osLog(LOG_ERROR, STAG x)
#define MOTION_RECOGNITION_LOG_3(x...) osLog(LOG_ERROR, STAG x)
#define MOTION_RECOGNITION_LOG_4(x...) osLog(LOG_ERROR, STAG x)
#else /*MOTION_RECOGNITION_LOG*/
#define MOTION_RECOGNITION_LOG_0(x...)
#define MOTION_RECOGNITION_LOG_1(x...)
#define MOTION_RECOGNITION_LOG_2(x...)
#define MOTION_RECOGNITION_LOG_3(x...)
#define MOTION_RECOGNITION_LOG_4(x...)
#endif /*MOTION_RECOGNITION_LOG*/

#endif  /*CFG_MTK_SCP_MOTION_RECOGNITION*/

#ifdef CFG_MSM_845_MOTION_RECOGNITION

#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_memmgr.h"
#include "sns_printf_int.h"
#include "sns_island_util.h"
#include "sns_island.h"
#include "sns_assert.h"

#define MOTION_RECOGNITION_LOG

#ifdef MOTION_RECOGNITION_LOG
#define MOTION_RECOGNITION_LOG_0(msg)                       SNS_SPRINTF(MED, sns_fw_printf, STAG msg)
#define MOTION_RECOGNITION_LOG_1(msg, p1)                   SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1)
#define MOTION_RECOGNITION_LOG_2(msg, p1, p2)               SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2)
#define MOTION_RECOGNITION_LOG_3(msg, p1, p2, p3)           SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2, p3)
#define MOTION_RECOGNITION_LOG_4(msg, p1, p2, p3, p4)       SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2, p3, p4)
#define MOTION_RECOGNITION_LOG_5(msg, p1, p2, p3, p4, p5)   SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2, p3, p4, p5)
#else /*MOTION_RECOGNITION_LOG*/
#define MOTION_RECOGNITION_LOG_0(msg)
#define MOTION_RECOGNITION_LOG_1(msg, p1)
#define MOTION_RECOGNITION_LOG_2(msg, p1, p2)
#define MOTION_RECOGNITION_LOG_3(msg, p1, p2, p3)
#define MOTION_RECOGNITION_LOG_4(msg, p1, p2, p3, p4)
#endif /*MOTION_RECOGNITION_LOG*/

#endif  /*CFG_MSM_845_MOTION_RECOGNITION*/

#ifdef CFG_MSM_660_MOTION_RECOGNITION

#include <math.h>
#include "sns_memmgr.h"
#include "sns_em.h"
#include "fixed_point.h"

//#define MOTION_RECOGNITION_LOG

#ifdef MOTION_RECOGNITION_LOG
#define MOTION_RECOGNITION_LOG_0(msg)                    UMSG(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg)
#define MOTION_RECOGNITION_LOG_1(msg, p1)                UMSG_1(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1)
#define MOTION_RECOGNITION_LOG_2(msg, p1, p2)            UMSG_2(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2)
#define MOTION_RECOGNITION_LOG_3(msg, p1, p2, p3)        UMSG_3(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2, p3)
#define MOTION_RECOGNITION_LOG_4(msg, p1, p2, p3, p4)    UMSG_4(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2, p3, p4)
#else /*MOTION_RECOGNITION_LOG*/
#define MOTION_RECOGNITION_LOG_0(msg)
#define MOTION_RECOGNITION_LOG_1(msg, p1)
#define MOTION_RECOGNITION_LOG_2(msg, p1, p2)
#define MOTION_RECOGNITION_LOG_3(msg, p1, p2, p3)
#define MOTION_RECOGNITION_LOG_4(msg, p1, p2, p3, p4)
#endif /*MOTION_RECOGNITION_LOG*/

#endif  /*CFG_MSM_660_MOTION_RECOGNITION*/

#define MOTION_RECOGNITION_ACC_CHECK_HZ 20

#define BUFFER_SIZE 20
#define MOTION_RECOGNITION_NORMAL_LATENCY 0ull

#define FLOAT_EPSILON 0.000001f
#define END_DRIVE_COUNTER 10
#define PIE 3.1416F
#define MAX_MOTION_STATES   6
#define MAX_MOTION_BUFF     20

typedef enum
{
    AP_SUSPEND,
    AP_AWAKE,
} AP_STATUS;

typedef enum
{
    AMD_STATIONARY,
    AMD_MOVE,
    AMD_UNKONOW,
}AMD_MOTION_TYPE;

typedef enum
{
    ACC_20_TYPE,
    REMOTE_PROC_TYPE,
    AMD_TYPE,
    TIMER_TYPE,
    MAX_TYPE,
} SENSOR_TYPE;

typedef enum
{
    INCAR_OUT,
    INCAR_IN,
    INCAR_END = 5,
    INCAR_TEMP_STOP = 9,
    INCAR_DRIVE = 10,
} IN_CAR_STATE;


typedef enum
{
    ALGO_REGISTER,
    ALGO_RUNNING,
    ALGO_PAUSE,
    ALGO_CLOSE,
} ALGO_STATUS;

typedef struct
{
    float x_value;
    float y_value;
    float z_value;

#ifdef CFG_MSM_845_MOTION_RECOGNITION
    uint64_t timestamp;
#else
    uint32_t timestamp;
#endif

    SENSOR_TYPE type;
} motion_recognition_sensor_data;

struct motion_recognition_sensor_operation
{
    void (*sensor_change_state)(int sensor_type, bool on);
    void (*report)(int report_count, int incar, int activity, uint64_t report_time);
    uint64_t (*get_delta_time_ms)(uint64_t timestamp);

    uint64_t (*get_current_time_tick)(void);

    void* (*malloc)(size_t size);   //malloc heap
    void (*free)(void*);            //free heap
};

typedef struct
{
    ALGO_STATUS algo_state;

    IN_CAR_STATE in_car_state;
    IN_CAR_STATE in_car_temp;
    IN_CAR_STATE in_car_pre_state;
    MOTION_STATE motion_state;
    ACTIVITY_MODE activity_mode;
    int acc_sample_rate;
    bool end_drive_detected;
    int end_drive_counter;
    int acc_count;
    motion_recognition_sensor_data acc_buffer[BUFFER_SIZE];
    struct motion_recognition_persistent_state _persistent;
    struct motion_recognition_sensor_operation *sensor_ops;
} motion_recognition_state;

void motion_recognition_algo_register(motion_recognition_state **state);

void motion_recognition_Reset(void);

void motion_recognition_close(void);

void motion_recognition_check_sensor_data(motion_recognition_sensor_data *input);

#endif //MOTION_RECOGNITION_ALGO_H

