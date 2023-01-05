#ifndef OPLUS_DEVORIENT_ALGO_H
#define OPLUS_DEVORIENT_ALGO_H

#define STAG "DEVORIENT-- "

/********************
*****MTK log config*****
*********************/

#ifdef CFG_MTK_SCP_DEVORIENT 
#include "algoConfig.h"
#include <performance.h>
#include "contexthub_fw.h"


#define DEVORIENT_LOG_INFO(str, ...) osLog(LOG_ERROR, STAG str, ##__VA_ARGS__)
#ifdef OPLUS_DEVORIENT_ENABLE_LOG_DEBUG
#define DEVORIENT_LOG_DEBUG(str, ...) osLog(LOG_ERROR, STAG str, ##__VA_ARGS__)
#else
#define DEVORIENT_LOG_DEBUG(str, ...)
#endif
#endif /*CFG_MTK_SCP_DEVORIENT*/

/********************
*****MTK log config*****
*********************/


/********************
*****QCOM log config*****
*********************/
#ifdef CFG_QCOM_SEE_DEVORIENT
#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"

#define DEVORIENT_LOG_INFO(str, ...) do { \
  SNS_PRINTF(ERROR, sns_fw_printf, STAG str, ##__VA_ARGS__); \
} while (0)
#ifdef OPLUS_DEVORIENT_ENABLE_LOG_DEBUG
#define DEVORIENT_LOG_DEBUG(str, ...) do { \
  SNS_PRINTF(ERROR, sns_fw_printf, STAG str, ##__VA_ARGS__); \
} while (0)
#else
#define DEVORIENT_LOG_DEBUG(str, ...)
#endif

#endif /*CFG_QCOM_SEE_DEVORIENT*/

/********************
*****QCOM log config*****
*********************/

#define TILT_HISTORY_SIZE                   200
#define RADIANS_TO_DEGREES                  (180.0f / M_PI)
#define M_PI                                3.1415926535897932384

//#define OPLUS_FEATURE_DEVORIENT_SWING

#ifdef OPLUS_FEATURE_DEVORIENT_SWING
#define STEADY_ADJUST_NUM  25

#define X_SWINGING_VARIANCE_THRESHOLD (3.08)

#define Y_SWINGING_VARIANCE_THRESHOLD (3.08)

#define Z_SWINGING_VARIANCE_THRESHOLD (1.82)
#endif
typedef enum {
    ACC_TYPE,
    AMD_TYPE,
    MAX_TYPE,
} SENSOR_TYPE;

typedef enum {
    AMD_STATIONARY,
    AMD_MOVE,
    AMD_UNKONOW,
} AMD_MOTION_TYPE;

typedef enum {
    ALGO_REGISTER,
    ALGO_RESET,
    ALGO_CLOSE,
} ALGO_STATUS;

typedef struct {
    float x;
    float y;
    float z;

#ifdef CFG_QCOM_SEE_DEVORIENT
    uint64_t timestamp;
#else
    uint32_t timestamp;
#endif

    SENSOR_TYPE type;
} devorient_sensor_data;

struct devorient_sensor_operation {
    void        (*sensor_change_state)  (int sensor_type, bool on);
    void        (*report)               (int move, uint16_t report_count);
    uint64_t    (*get_time_ms)          (uint64_t timestamp);
};

typedef struct {
    int acc_sampleRate;

    uint64_t last_filtered_time;
    devorient_sensor_data last_filtered_data;

    uint64_t tilt_reference_time;
    uint64_t accelerating_time;
    uint64_t predicted_rotation_time;
    uint64_t flat_time;
    uint64_t swinging_time;

    uint32_t tilt_history_time[TILT_HISTORY_SIZE];
    int tilt_history_index;
    int8_t tilt_history[TILT_HISTORY_SIZE];

    int8_t current_rotation;
    int8_t prev_valid_rotation;
    int8_t proposed_rotation;
    int8_t predicted_rotation;
    #ifdef OPLUS_FEATURE_DEVORIENT_SWING
    float  history_x[STEADY_ADJUST_NUM];
    float  history_y[STEADY_ADJUST_NUM];
    float  history_z[STEADY_ADJUST_NUM];
    int32_t history_index;
    #endif

    bool flat;
    bool swinging;
    bool accelerating;
    bool overhead;
    bool first_init;
    bool need_refilter;
    ALGO_STATUS algo_state;
    struct devorient_sensor_operation *sensor_ops;
} devorient_algo_state;

void devorient_algo_register(devorient_algo_state **state);
void devorient_algo_reset(void);
void devorient_algo_close(void);
void devorient_algo_check_sensor_data(devorient_sensor_data *input);
#endif