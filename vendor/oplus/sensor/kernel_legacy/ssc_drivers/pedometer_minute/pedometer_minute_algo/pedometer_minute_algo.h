#ifndef PEDEMETER_ALGO_H
#define PEDEMETER_ALGO_H

#define STAG "PEDEMETER - "

#ifdef CFG_MTK_SCP_PEDEMETER

#include "algoConfig.h"
#include <performance.h>
#include "contexthub_fw.h"

#define PEDEMETER_LOG

#ifdef PEDEMETER_LOG
#define PEDEMETER_LOG_0(x...) osLog(LOG_ERROR, STAG x)
#define PEDEMETER_LOG_1(x...) osLog(LOG_ERROR, STAG x)
#define PEDEMETER_LOG_2(x...) osLog(LOG_ERROR, STAG x)
#define PEDEMETER_LOG_3(x...) osLog(LOG_ERROR, STAG x)
#define PEDEMETER_LOG_4(x...) osLog(LOG_ERROR, STAG x)
#else /*TP_GESTURE_LOG*/
#define PEDEMETER_LOG_0(x...)
#define PEDEMETER_LOG_1(x...)
#define PEDEMETER_LOG_2(x...)
#define PEDEMETER_LOG_3(x...)
#define PEDEMETER_LOG_4(x...)
#endif /*TP_GESTURE_LOG*/

#endif  /*CFG_MTK_SCP_TP_GESTURE*/

#ifdef CFG_MSM_845_PEDEMETER

#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"

#define PEDEMETER_LOG

#ifdef PEDEMETER_LOG
#define PEDEMETER_LOG_0(msg)                SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg)
#define PEDEMETER_LOG_1(msg,p1)             SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1)
#define PEDEMETER_LOG_2(msg,p1,p2)          SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2)
#define PEDEMETER_LOG_3(msg,p1,p2,p3)       SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2,p3)
#define PEDEMETER_LOG_4(msg,p1,p2,p3,p4)    SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2,p3,p4)
#else /*TP_GESTURE_LOG*/
#define PEDEMETER_LOG_0(msg)
#define PEDEMETER_LOG_1(msg,p1)
#define PEDEMETER_LOG_2(msg,p1,p2)
#define PEDEMETER_LOG_3(msg,p1,p2,p3)
#define PEDEMETER_LOG_4(msg,p1,p2,p3,p4)
#endif /*TP_GESTURE_LOG*/

#endif  /*CFG_MSM_845_TP_GESTURE*/

#ifdef CFG_MSM_660_PEDEMETER

#include <math.h>
#include "sns_memmgr.h"
#include "sns_em.h"
#include "fixed_point.h"

#define PEDEMETER_LOG

#ifdef PEDEMETER_LOG
#define PEDEMETER_LOG_0(msg)                UMSG(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg)
#define PEDEMETER_LOG_1(msg,p1)             UMSG_1(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1)
#define PEDEMETER_LOG_2(msg,p1,p2)          UMSG_2(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2)
#define PEDEMETER_LOG_3(msg,p1,p2,p3)       UMSG_3(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2,p3)
#define PEDEMETER_LOG_4(msg,p1,p2,p3,p4)    UMSG_4(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2,p3,p4)
#else /*TP_GESTURE_LOG*/
#define PEDEMETER_LOG_0(msg)
#define PEDEMETER_LOG_1(msg,p1)
#define PEDEMETER_LOG_2(msg,p1,p2)
#define PEDEMETER_LOG_3(msg,p1,p2,p3)
#define PEDEMETER_LOG_4(msg,p1,p2,p3,p4)
#endif /*TP_GESTURE_LOG*/

#endif  /*CFG_MSM_660_TP_GESTURE*/
#define TIMESTAMP_RATIO 0.7

typedef enum
{
    AP_SUSPEND,
    AP_AWAKE,
}AP_STATUS;

typedef enum
{
    TIMER_TYPE,
    STEP_COUNTER_TYPE,
#ifdef REGISTRY_CMC
    CMC_TYPE,
#endif
    MAX_TYPE,
}SENSOR_TYPE;

typedef enum
{
    ALGO_REGISTER,
    ALGO_RESET,
    ALGO_CLOSE,
}ALGO_STATUS;



typedef enum
{
    MOTION_UNKNOWN,
    MOTION_WALK,
    MOTION_RUN,
    MOTION_STATIONARY,
    MOTION_VEHICLE,
    MOTION_BIKE_TRANSIT,
    MOTION_MOTOR_TRANSIT = 7,
    MOTION_BIKE,
} MOTION_MODE;

typedef struct
{
    float x;

#ifdef CFG_MSM_845_PEDEMETER
    uint64_t timestamp;
#else
    uint32_t timestamp;
#endif

    SENSOR_TYPE type;
}pedometer_minute_sensor_data;

struct pedometer_minute_sensor_operation
{
    void        (*sensor_change_state)     (int sensor_type, bool on);
    void        (*report)                  (int step_count, int step_run_step, int step_walk_step,
	  					uint16_t report_count,uint8_t move_state,int time_gap,uint64_t timestamp);
    uint64_t    (*get_delta_time_ms)       (int64_t timestamp);
};

struct pre_move_info_st
{
    uint32_t pre_step;
    uint32_t pre_run_step;
    uint32_t pre_walk_step;
    uint8_t  move_status;
};
typedef struct
{
    uint8_t timer_count ;
    uint64_t last_data_ts[MAX_TYPE];
    ALGO_STATUS algo_state;
    uint32_t last_step_counter;
    uint32_t current_step_counter;
    uint32_t current_run_step_counter;
    uint32_t current_walk_step_counter;
    uint8_t  current_move_state;
    uint8_t  current_ap_state;
    struct pre_move_info_st pre_move_info[120];
    struct pedometer_minute_sensor_operation *sensor_ops;
    uint64_t current_timer_count_stamp;
} pedometer_minute_state;

void pedometer_minute_algo_register(pedometer_minute_state **state);
void pedometer_minute_Reset(float samplerate);
void pedometer_minute_close();
void pedometer_minute_check_sensor_data(pedometer_minute_sensor_data *input);

#endif

