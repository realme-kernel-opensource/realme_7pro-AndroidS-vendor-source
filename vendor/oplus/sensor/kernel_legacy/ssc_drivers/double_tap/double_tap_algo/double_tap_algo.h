#ifndef DOUBLE_TAP_ALGO_H
#define DOUBLE_TAP_ALGO_H

#define STAG "DOUBLE_TAP - "

#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"

#define DOUBLE_TAP_ENABLE_LOGI


#ifdef DOUBLE_TAP_ENABLE_LOGI
#define DOUBLE_TAP_LOGI(str, ...) do { \
    SNS_SPRINTF(MED, sns_fw_printf, STAG "%s: " str, __func__, ##__VA_ARGS__); \
} while (0)
#else
#define DOUBLE_TAP_LOGI(str, ...)
#endif

#define ACC_SAMPLE_RATE 20
#define ACC_CHECK_LEN 20
#define GRAV_CHECK_LEN 20
#define HISTORY_DATA_LEN 15
#define STATIC_BIAS 0.2
#define G_FILTER_NUM 3
#define AMD_BUFFER_SIZE 12 // 20hz 600ms
typedef enum {
    ACCEL_TYPE,
    PROX_TYPE,
    IC_DOUBLE_TAP_TYPE,
    GRAVITY_TYPE,
    MAX_TYPE
} SENSOR_TYPE;

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
    uint64_t timestamp;
    SENSOR_TYPE type;
} double_tap_sensor_data;

struct double_tap_sensor_operation {
    void        (*sensor_change_state)     (int sensor_type, bool on);
    void        (*report)                  (uint16_t report_count);
    uint64_t    (*get_delta_time_ms)       (int64_t timestamp);
};

typedef struct {
    int sampleRate;
    int latency;
    ALGO_STATUS algo_state;

    uint8_t prx_state;
    uint8_t dbt_state;
    uint8_t dbt_fired;
    bool is_check_history;
    bool first_acc;
    int acc_point;
    int gravity_count;
    int count;
    int static_flag_count;
    AMD_STATUS current_status;
    AMD_STATUS last_status;
    double_tap_sensor_data static_gdata;
    double_tap_sensor_data acc_buf[ACC_CHECK_LEN];
    double_tap_sensor_data gravity_buf[GRAV_CHECK_LEN];
    uint64_t last_data_ts[MAX_TYPE];
    struct double_tap_sensor_operation *sensor_ops;
} double_tap_state;

void double_tap_algo_register(double_tap_state **state);
void double_tap_Reset(void);
void double_tap_close(void);
void double_tap_process_sensor_data(double_tap_sensor_data *input);
#endif

