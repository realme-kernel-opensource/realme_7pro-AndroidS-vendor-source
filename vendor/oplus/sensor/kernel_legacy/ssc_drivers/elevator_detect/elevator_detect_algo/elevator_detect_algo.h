/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: elevator_detect_algo.h
**
** Description:
**      Data types for detect elevator operator.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/
#ifndef ELEVATOR_DETECT_ALGO_H
#define ELEVATOR_DETECT_ALGO_H

#define STAG "ELEVATOR_DETECT - "

#ifdef CFG_MTK_SCP_ELEVATOR_DETECT

#include "algoConfig.h"
#include <performance.h>
#include "contexthub_fw.h"

//#define ELEVATOR_DETECT_LOG

#ifdef ELEVATOR_DETECT_LOG
#define ELEVATOR_DETECT_LOG_0(x...) osLog(LOG_ERROR, STAG x)
#define ELEVATOR_DETECT_LOG_1(x...) osLog(LOG_ERROR, STAG x)
#define ELEVATOR_DETECT_LOG_2(x...) osLog(LOG_ERROR, STAG x)
#define ELEVATOR_DETECT_LOG_3(x...) osLog(LOG_ERROR, STAG x)
#define ELEVATOR_DETECT_LOG_4(x...) osLog(LOG_ERROR, STAG x)
#else /*ELEVATOR_DETECT_LOG*/
#define ELEVATOR_DETECT_LOG_0(x...)
#define ELEVATOR_DETECT_LOG_1(x...)
#define ELEVATOR_DETECT_LOG_2(x...)
#define ELEVATOR_DETECT_LOG_3(x...)
#define ELEVATOR_DETECT_LOG_4(x...)
#endif /*ELEVATOR_DETECT_LOG*/

#endif  /*CFG_MTK_SCP_ELEVATOR_DETECT*/

#ifdef CFG_MSM_845_ELEVATOR_DETECT

#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
//#include "sns_memmgr.h"
//#include "sns_printf_int.h"

#define ELEVATOR_DETECT_LOG

#ifdef ELEVATOR_DETECT_LOG
#define ELEVATOR_DETECT_LOG_0(msg)                      SNS_SPRINTF(MED, sns_fw_printf, STAG msg)
#define ELEVATOR_DETECT_LOG_1(msg, p1)                  SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1)
#define ELEVATOR_DETECT_LOG_2(msg, p1, p2)              SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2)
#define ELEVATOR_DETECT_LOG_3(msg, p1, p2, p3)          SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2, p3)
#define ELEVATOR_DETECT_LOG_4(msg, p1, p2, p3, p4)      SNS_SPRINTF(MED, sns_fw_printf, STAG msg, p1, p2, p3, p4)
#else /*ELEVATOR_DETECT_LOG*/
#define ELEVATOR_DETECT_LOG_0(msg)
#define ELEVATOR_DETECT_LOG_1(msg, p1)
#define ELEVATOR_DETECT_LOG_2(msg, p1, p2)
#define ELEVATOR_DETECT_LOG_3(msg, p1, p2, p3)
#define ELEVATOR_DETECT_LOG_4(msg, p1, p2, p3, p4)
#endif /*ELEVATOR_DETECT_LOG*/

#endif  /*CFG_MSM_845_ELEVATOR_DETECT*/

#ifdef CFG_MSM_660_ELEVATOR_DETECT

#include <math.h>
#include "sns_memmgr.h"
#include "sns_em.h"
#include "fixed_point.h"

//#define ELEVATOR_DETECT_LOG

#ifdef ELEVATOR_DETECT_LOG
#define ELEVATOR_DETECT_LOG_0(msg)                    UMSG(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg)
#define ELEVATOR_DETECT_LOG_1(msg, p1)                UMSG_1(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1)
#define ELEVATOR_DETECT_LOG_2(msg, p1, p2)            UMSG_2(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2)
#define ELEVATOR_DETECT_LOG_3(msg, p1, p2, p3)        UMSG_3(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2, p3)
#define ELEVATOR_DETECT_LOG_4(msg, p1, p2, p3, p4)    UMSG_4(MSG_SSID_SNS, DBG_ERROR_PRIO, STAG msg, p1, p2, p3, p4)
#else /*ELEVATOR_DETECT_LOG*/
#define ELEVATOR_DETECT_LOG_0(msg)
#define ELEVATOR_DETECT_LOG_1(msg, p1)
#define ELEVATOR_DETECT_LOG_2(msg, p1, p2)
#define ELEVATOR_DETECT_LOG_3(msg, p1, p2, p3)
#define ELEVATOR_DETECT_LOG_4(msg, p1, p2, p3, p4)
#endif /*ELEVATOR_DETECT_LOG*/

#endif  /*CFG_MSM_660_ELEVATOR_DETECT*/

#define ELEVATOR_DETECT_SENSOR_COLLECT_HZ 20
#define DELTA_TIME 1.0 / ELEVATOR_DETECT_SENSOR_COLLECT_HZ
#define PI 3.1416F
#define INTERVAL_MAKE_PAIR_WAVEFORM 1000 * 60 * 2
#define THRESHOLD_BOTTOM 0.2
#define THRESHOLD_TOP 2
#define MINIMUM_STATIONARY_RATIO 0.6
#define MINIMUM_CHANGED_DISTANCE 1.5
#define CUT_OFF_FRENQUENCY 3
#define LOW_PASS_COEFFICIENT 1.0 / (1 + (ELEVATOR_DETECT_SENSOR_COLLECT_HZ / (2 * PI * CUT_OFF_FRENQUENCY)))
#define STEP_DETECT_THRESHOLD 4
#define STEP_COUNTER_SYMBOL_THRESHOLD 0.7
#define STEP_EFFECTIVE_TIME 2000
#define MAKE_STEP_TIME_BOTTOM 200
#define MAKE_STEP_TIME_TOP 1000

#define THRESHOLD_WAVE_STATE_NUM (1000 / (1000 / ELEVATOR_DETECT_SENSOR_COLLECT_HZ)) // 20
#define GRAVITY_EARTH 9.80665f

typedef enum
{
    ALGO_REGISTER,
    ALGO_RUNNING,
    ALGO_CLOSE,
} ALGO_STATUS;

typedef enum
{
    AMD_STATIONARY,
    AMD_MOVE,
    AMD_UNKONOW,
}AMD_MOTION_TYPE;

typedef enum
{
    ACC_TYPE,
    GAMEROTATION_VECTOR_TYPE,
    AMD_TYPE,
    TIMER_TYPE,
    MAX_TYPE,
}SENSOR_TYPE;

typedef enum
{
    ELEVATOR_UNKNOWN,
    ELEVATOR_UP,
    ELEVATOR_DOWN,
}ELEVATOR_STATUS;

typedef enum
{
    EVENT_STATE_UNKNOWN,
    EVENT_STATE_ENTER,
    EVENT_STATE_EXIT,
}EVENT_STATE;

typedef enum
{
    SIGN_UNKNOWN,
    SIGN_POSITIVE,
    SIGN_NEGTIVE,
}SIGN_STATUS;

typedef enum
{
    WAVE_UNKNOWN,
    WAVE_HILL,
    WAVE_VALLEY,
}WAVE_STATUS;

typedef struct
{
    float x_value;
    float y_value;
    float z_value;

#ifdef CFG_MSM_845_ELEVATOR_DETECT
    uint64_t timestamp;
#else
    uint32_t timestamp;
#endif

    SENSOR_TYPE type;
}elevator_detect_sensor_data;

struct elevator_detect_sensor_operation
{
    void        (*sensor_change_state)     (int sensor_type, bool on);
    void        (*report)                  (int value, uint16_t report_count);
    uint64_t    (*get_delta_time_ms)       (int64_t timestamp);
};

typedef struct
{
    ALGO_STATUS algo_state;
    WAVE_STATUS last_wave_state;
    SIGN_STATUS last_sign_state;
    SIGN_STATUS walk_detect_last_sign;
    ELEVATOR_STATUS current_elevator_state;
    EVENT_STATE current_event_state;

    int acc_sampleRate;
    int game_rv_sampleRate;
    int same_sign_count;
    int step_count;
    int stationary_count;
    int moving_count;

    float speed;
    float distance;
    float distance_peak;
    float distance_valley;

    uint64_t last_wave_end_ts;
    uint64_t walk_detect_enter_hill_ts;
    uint64_t walk_detect_last_step_ts;

    elevator_detect_sensor_data last_sensor_data;
    elevator_detect_sensor_data filtered_sensor_data;
    elevator_detect_sensor_data current_acc;
    struct elevator_detect_sensor_operation *sensor_ops;
} elevator_detect_state;

void elevator_detect_algo_register(elevator_detect_state **state);
void elevator_detect_Reset(void);
void elevator_detect_close(void);
void elevator_detect_check_sensor_data(elevator_detect_sensor_data *input);

#endif //ELEVATOR_DETECT_ALGO_H

