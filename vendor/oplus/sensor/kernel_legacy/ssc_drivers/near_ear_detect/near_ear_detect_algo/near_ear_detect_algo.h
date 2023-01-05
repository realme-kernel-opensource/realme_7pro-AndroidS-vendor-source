#ifndef NEAR_EAR_DETECT_ALGO_H
#define NEAR_EAR_DETECT_ALGO_H
#define STAG "NEAR_EAR_DETEDT - "

#ifdef CFG_MTK_SCP_NEAR_EAR_DETECT

#include "algoConfig.h"
#include "oplus_performance.h"
#include "contexthub_fw.h"

//#define NEAR_EAR_DETECT_LOG

#ifdef NEAR_EAR_DETECT_LOG
#define NEAR_EAR_DETECT_LOG_0(x...) osLog(LOG_ERROR, STAG x)
#define NEAR_EAR_DETECT_LOG_1(x...) osLog(LOG_ERROR, STAG x)
#define NEAR_EAR_DETECT_LOG_2(x...) osLog(LOG_ERROR, STAG x)
#define NEAR_EAR_DETECT_LOG_3(x...) osLog(LOG_ERROR, STAG x)
#define NEAR_EAR_DETECT_LOG_4(x...) osLog(LOG_ERROR, STAG x)
#else /*NEAR_EAR_DETECT_LOG*/
#define NEAR_EAR_DETECT_LOG_0(x...)
#define NEAR_EAR_DETECT_LOG_1(x...)
#define NEAR_EAR_DETECT_LOG_2(x...)
#define NEAR_EAR_DETECT_LOG_3(x...)
#define NEAR_EAR_DETECT_LOG_4(x...)
#endif /*NEAR_EAR_DETECT_LOG*/

#endif  /*CFG_MTK_SCP_NEAR_EAR_DETECT*/

#ifdef CFG_MSM_845_NEAR_EAR_DETECT

#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"

//#define NEAR_EAR_DETECT_LOG

#ifdef NEAR_EAR_DETECT_LOG
#define NEAR_EAR_DETECT_LOG_0(msg)                SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg)
#define NEAR_EAR_DETECT_LOG_1(msg,p1)             SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1)
#define NEAR_EAR_DETECT_LOG_2(msg,p1,p2)          SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2)
#define NEAR_EAR_DETECT_LOG_3(msg,p1,p2,p3)       SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2,p3)
#define NEAR_EAR_DETECT_LOG_4(msg,p1,p2,p3,p4)    SNS_SPRINTF(ERROR, sns_fw_printf, STAG msg,p1,p2,p3,p4)
#else /*NEAR_EAR_DETECT_LOG*/
#define NEAR_EAR_DETECT_LOG_0(msg)
#define NEAR_EAR_DETECT_LOG_1(msg,p1)
#define NEAR_EAR_DETECT_LOG_2(msg,p1,p2)
#define NEAR_EAR_DETECT_LOG_3(msg,p1,p2,p3)
#define NEAR_EAR_DETECT_LOG_4(msg,p1,p2,p3,p4)
#endif /*NEAR_EAR_DETECT_LOG*/

#endif  /*CFG_MSM_845_NEAR_EAR_DETECT*/

#ifdef CFG_MSM_660_NEAR_EAR_DETECT

#include <math.h>
#include "sns_memmgr.h"
#include "sns_em.h"
#include "fixed_point.h"

//#define NEAR_EAR_DETECT_LOG

#ifdef NEAR_EAR_DETECT_LOG
#define NEAR_EAR_DETECT_LOG_0(msg)                UMSG(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg)
#define NEAR_EAR_DETECT_LOG_1(msg,p1)             UMSG_1(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1)
#define NEAR_EAR_DETECT_LOG_2(msg,p1,p2)          UMSG_2(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2)
#define NEAR_EAR_DETECT_LOG_3(msg,p1,p2,p3)       UMSG_3(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2,p3)
#define NEAR_EAR_DETECT_LOG_4(msg,p1,p2,p3,p4)    UMSG_4(MSG_SSID_SNS,DBG_ERROR_PRIO, STAG msg,p1,p2,p3,p4)
#else /*NEAR_EAR_DETECT_LOG*/
#define NEAR_EAR_DETECT_LOG_0(msg)
#define NEAR_EAR_DETECT_LOG_1(msg,p1)
#define NEAR_EAR_DETECT_LOG_2(msg,p1,p2)
#define NEAR_EAR_DETECT_LOG_3(msg,p1,p2,p3)
#define NEAR_EAR_DETECT_LOG_4(msg,p1,p2,p3,p4)
#endif /*NEAR_EAR_DETECT_LOG*/

#endif  /*CFG_MSM_660_NEAR_EAR_DETECT*/

#define PIE 3.1416F
#define NEAR_EAR_DETECT_STABLE_CHECK_HZ (50)
#define BUFFER_SIZE (30)
#define NEAR_EAR_DETECT_NORMAL_LATENCY 0ull

#define NEAR_EAR_CHECK_NUM 3
#define EG (20.0f) //EARTH_GRAVITY
#define SS_MASK(pos) (1 << pos)
#define WAKEUP_ANGLE_BIAS (3)
#define STATIC_BIAS 0.2
#define G_FILTER_NUM 6

//NOT USED
#define SAM_NEAR_EAR_DETECT_ELEMS 2
#define NEAR_EAR_DETECT_VARIANCE_CHECK_IDX 4
#define NEAR_EAR_DETECT_VARIANCE_CHECK_NUM (40)
#define NEAR_EAR_DETECT_STABLE_CHECK_NUM (3)
#define NEAR_EAR_DETECT_SAMPLES_PER_REPORT (3)

typedef enum {
    ACC_TYPE,
    GYRO_TYPE,
    GRAVITY_TYPE,
    PROX_TYPE,
    MAX_TYPE,
} SENSOR_TYPE;

typedef enum {
    DEVICE_WAKEUP,
    DEVICE_SLEEP_FROM_WAKE,
    DEVICE_POSTURE_RESET
} device_status;

typedef enum {
    SLEEP_DROP,
    SLEEP_BACK,
    SLEEP_POCKET
} sleep_status;

typedef enum {
    AMD_UNKNOWN,
    AMD_STATIC,
    AMD_MOVE,
} AMD_STATUS;

typedef enum {
    ALGO_REGISTER,
    ALGO_RESET,
    ALGO_CLOSE,
} ALGO_STATUS;

typedef struct {
    float x;
    float y;
    float z;

#ifdef CFG_MSM_845_NEAR_EAR_DETECT
    uint64_t timestamp;
#else
    uint32_t timestamp;
#endif

    SENSOR_TYPE type;
} near_ear_detect_sensor_data;

enum {
    SS_BACK_SLEEP,
    SS_NEAR_EAR_WAKEUP,
    SS_ROTATE_FLAT_WAKEUP,
    SS_STATIC_NEAR_EAR_WAKEUP,
    SS_POCKET_WAKEUP,
    SS_NUM,
    SS_MOVING = 30
} SS_STATUS;

struct near_ear_detect_sensor_operation {
    void        (*sensor_change_state)     (int sensor_type, bool on);
    void        (*report)                  (int value, uint16_t report_count);
    uint64_t    (*get_delta_time_ms)       (uint64_t timestamp);
    bool        (*need_check_prox)         (void);
};

typedef struct {
    ALGO_STATUS algo_state;
    AMD_STATUS last_Amdstate;
    AMD_STATUS current_Amdstate;
    device_status dev_status;
    device_status last_dev_status;

    uint64_t sampleRate;
    int latency;

    int prx_state;
    int32_t prx_sample_rate;

    uint64_t last_wakeup_timestamp;
    uint64_t last_sleep_timestamp;
    uint64_t md_ts;

    int gravity_count;
    int gyro_count;
    int acc_count;
    uint64_t last_data_ts[MAX_TYPE];

    int static_flag_count;
    float gyro_last_rad_x;

    near_ear_detect_sensor_data gyro_rad;
    near_ear_detect_sensor_data gyro_buffer[BUFFER_SIZE];
    near_ear_detect_sensor_data acc_buffer[BUFFER_SIZE];
    near_ear_detect_sensor_data gravity_buffer[BUFFER_SIZE];
    near_ear_detect_sensor_data static_gdata;
    near_ear_detect_sensor_data md_data;

    struct near_ear_detect_sensor_operation *sensor_ops;
} near_ear_detect_state;

void near_ear_detect_algo_register(near_ear_detect_state **state);
void near_ear_detect_Reset(void);
void near_ear_detect_close(void);
void near_ear_detect_process_sensor_data(near_ear_detect_sensor_data *input);

#endif

