#pragma once

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_island.h"
#include "sns_printf_int.h"
#include "sns_std_sensor.pb.h"

#define MAG_FUSION_ENABLE_LOG_INFO
//#define MAG_FUSION_ENABLE_LOG_DEBUG
//#define MAG_FUSION_ENABLE_LOG_DATA
#define MAG_FUSION_LOG_STR "mag_fusion -- "
#ifdef MAG_FUSION_ENABLE_LOG_INFO
#define MAG_FUSION_LOGI(str, ...) do { \
  SNS_SPRINTF(HIGH, sns_fw_printf, MAG_FUSION_LOG_STR "" str, ##__VA_ARGS__); \
} while (0)
#else
#define MAG_FUSION_LOGI(str, ...)
#endif

#ifdef MAG_FUSION_ENABLE_LOG_DEBUG
#define MAG_FUSION_LOGDB(str, ...) do { \
  SNS_SPRINTF(HIGH, sns_fw_printf, MAG_FUSION_LOG_STR "" str, ##__VA_ARGS__); \
} while (0)
#else
#define MAG_FUSION_LOGDB(str, ...)
#endif

#ifdef MAG_FUSION_ENABLE_LOG_DATA
#define MAG_FUSION_LOGDT(str, ...) do { \
  SNS_SPRINTF(HIGH, sns_fw_printf, MAG_FUSION_LOG_STR "" str, ##__VA_ARGS__); \
} while (0)
#else
#define MAG_FUSION_LOGDT(str, ...)
#endif

#ifdef MAG_FUSION_USE_ISLAND
#define mag_fusion_exit_island() SNS_ISLAND_EXIT()
#else
#define mag_fusion_exit_island()
#endif

#define MAG_FUSION_ALLOC(size) sns_malloc(SNS_HEAP_ISLAND, size)
#define MAG_FUSION_FREE(ptr)   sns_free(ptr)
#define MAG_FUSION_MEMSET(ptr,val,size) sns_memset(ptr,val,size)

/*============================================================================
  Type Declarations
  ===========================================================================*/

typedef enum {
    MF_INPUT_TYPE_ACC = 0,
    MF_INPUT_TYPE_MAG = 1,
    MF_INPUT_TYPE_GYR = 2,
    MF_INPUT_TYPE_MAX_NUM,
} mag_fusion_input_type;

typedef struct {
    mag_fusion_input_type type;
    float data[3];
    sns_time timestamp;
} mag_fusion_data_input;

typedef struct {
    // unit: uT, bias = raw - out
    float bias[3];
    sns_std_sensor_sample_status accuracy;
    int result_changed;
} mag_fusion_output;

typedef struct {
    sns_time ts;
    mag_fusion_output output;
} mag_fusion_sample;

typedef struct {
    // initialize algorithm, used for first time enable algorithm
    int (*algo_init)(bool has_gyro, mag_fusion_output initial_result);

    // de-initialize algorithm
    int (*algo_deinit)();

    // start algorithm
    int (*algo_start)();

    // stop algorithm
    int (*algo_stop)();

    // input: input sensor data including sensor type
    // output: mag_cal data
    int (*algo_update)(mag_fusion_data_input *input,
        mag_fusion_output *output);

    // size of algorithm instance.
    // if return 0, won't do memory allocation for algorithm
    //size_t (*algo_req_size)();

    // output: debug info data
    int (*algo_get_debug_info)(float * out);
} mag_fusion_api;

#define SNS_TIME_CONVERT_TO_PS(tick) ((uint64_t)((tick)*sns_get_time_tick_resolution_in_ps()))
#define SNS_TIME_CONVERT_TO_NS(tick) ((uint64_t)((tick)*sns_get_time_tick_resolution()))
#define SNS_TIME_CONVERT_TO_US(tick) ((uint64_t)(SNS_TIME_CONVERT_TO_NS(tick)/1000))
#define SNS_TIME_CONVERT_TO_MS(tick) ((uint64_t)(SNS_TIME_CONVERT_TO_NS(tick)/1000000))

