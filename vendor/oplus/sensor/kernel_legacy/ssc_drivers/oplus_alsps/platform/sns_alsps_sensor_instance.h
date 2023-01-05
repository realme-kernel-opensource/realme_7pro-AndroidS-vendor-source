/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/
#pragma once

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_time.h"
#include "sns_com_port_types.h"
#include "sns_sensor_uid.h"
#include "sns_async_com_port.pb.h"
#include "sns_interrupt.pb.h"
#include "sns_timer.pb.h"
#include "sns_island_service.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_math_util.h"
#include "sns_registry_util.h"
#include "sns_proximity.pb.h"
#include "sns_osa_lock.h"
#include "oplus_alsps.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "../slit_prox_algo/light_algo.h"
#define OPLUS_ALSPS_ENABLE_LOG
#ifdef OPLUS_ALSPS_ENABLE_LOG

#define OPLUS_ALS_PS_LOG(...) do { \
    SNS_PRINTF(ERROR, sns_fw_printf, __VA_ARGS__); \
} while (0)

#define OPLUS_ALS_PS_SLOG(...) do { \
    SNS_SPRINTF(ERROR, sns_fw_printf, __VA_ARGS__); \
} while (0)
#else
#define OPLUS_ALS_PS_LOG(prio, ...)
#define OPLUS_ALS_PS_SLOG(prio, ...)
#endif

#define HB_ALS_BUF_LEN_FACTORY          5
#define HB_ALS_BUF_LEN                  10

typedef struct alsps_registry_cfg {
    alsps_sensor_type     sensor_type;
    float                   als_fac_cal_bias[2];
    float                   ps_fac_cal_bias[6];
    uint32_t                als_cal_version[2];
    uint32_t                ps_cal_version[6];
} alsps_registry_cfg;

typedef struct alsps_req {
    float               desired_sample_rate;
    float               desired_report_rate;
    alsps_sensor_type   sensor_type;
} sns_alsps_req;

typedef struct alsps_test_info {
    sns_physical_sensor_test_type test_type;
    bool test_client_present;
} alsps_test_info;

typedef struct boot_cali_info {
    int count;
    int max;
    int min;
    int delta;
} boot_cali_info;

typedef struct alsps_ps_info {
    sns_sensor_uid  prox_suid;
    alsps_test_info  test_info;
    uint8_t     cali_type;
    uint8_t     cali_status;
    uint8_t     enable_count;
    uint16_t   cali_count;
    uint32_t   cali_data;
    bool     is_ps_enable;
    bool     enable;
    bool     first_init;
    bool     first_data;
    bool     is_new_request;
    bool     int_registered;
    bool     hw_cali_started;
    bool                need_check_vertical;
    bool     is_alsps_enable_error;
    float     report_rate_hz;
    float     sampling_rate_hz;
    int   offset;
    uint16_t   thd_near;
    uint16_t   thd_far;
    uint16_t   ps_cali_goal;
    uint16_t   ps_cali_up_thrd;
    uint16_t   last_ps;
    uint8_t             lost_count;
    float               last_acc_x;
    float               last_acc_y;
    float               last_acc_z;
    alsps_ps_state  status;
    sns_proximity_event         data_stream_event;
    sns_proximity_event         delay_data_stream_event;
    struct alsps_ps_operations *ops;
    struct ps_algo *ps_parms;
    struct boot_cali_info boot_cali_parms;
    sns_time   last_irq_time;
    sns_time   last_irq_check_time;
    uint8_t             irq_error_count;
    bool                is_phone_mode;
    bool                in_hard_cali;
    sns_time            hard_cali_time;
} alsps_ps_info;

typedef struct alsps_als_info {
    sns_sensor_uid  als_suid;
    alsps_test_info  test_info;
    uint8_t     cali_status;
    uint16_t   cali_count;
    uint32_t   cali_data;
    uint8_t     enable_count;
    bool     is_als_enable;
    bool     enable;
    bool     first_init;
    bool                first_data;
    bool     is_new_request;
    float     report_rate_hz;
    float     sampling_rate_hz;
    float               scale;
    float               bias;
    bool     int_registered;
    bool                is_dri;
    bool                is_factory_mode;
    uint32_t            als_factor;
    uint32_t            als_sum;
    uint8_t             als_count;
    sns_time            last_irq_timestamp;
    struct alsps_als_operations *ops;
} alsps_als_info;

typedef struct alsps_under_lcd_algo_info {
    struct     alsps_data als_data[LIGHT_DATA_COUNT];
    uint8_t    data_count;
    uint8_t    buffer_lenth;
    uint8_t    persist_length;
    bool       data_full;
} alsps_under_lcd_algo_info;

struct acc_mode_flag {
    bool is_delay_report;
    bool first_acc_check;
    bool first_near_by;
    bool need_report_far;
};

typedef struct alsps_instance_state {
    sns_interrupt_req  irq_config;
    sns_com_port_config      com_config;
    sns_sync_com_port_handle *port_handle;
    sns_sync_com_port_service   *scp_service;
    sns_island_service          *island_service;
    bool  update_cal_registry;
    bool  update_dynamic_cal_registry;
    bool  instance_is_ready_to_configure;
    bool   is_first_init;

    /* Ambient light HW config details*/
    alsps_als_info        als_info;

    /* Proximity HW config details*/
    alsps_ps_info       ps_info;

    alsps_under_lcd_algo_info als_under_info;

    /* Data streams from dependentcies. */
    sns_sensor_uid          irq_suid;
    sns_sensor_uid          timer_suid;
    sns_sensor_uid          accel_suid;
    sns_sensor_uid          resampler_suid;

    /**----------Sensor specific registry configuration----------*/
    sns_registry_phy_sensor_cfg als_registry_cfg;
    sns_registry_phy_sensor_cfg prox_registry_cfg;

    size_t                      encoded_als_event_len;
    size_t                      encoded_ps_event_len;

    float                   als_fac_cal_data[ALS_FAC_CAL_NUM];
    float                   ps_fac_cal_data[PS_FAC_CAL_NUM];
    uint32_t                als_cal_version[ALS_FAC_CAL_NUM];
    uint32_t                ps_cal_version[PS_FAC_CAL_NUM];

    uint8_t    als_type;
    uint8_t    ps_type;
    alsps_sensor_type rgt_sensor_type;//just user to find instance when als ps only
    uint8_t      is_unit_device;

    bool        use_lb_algo;
    uint16_t    real_brightness;
    sns_time    last_report_time;
    uint8_t dc_mode;
    struct acc_mode_flag amode_flag;
} alsps_instance_state;

sns_rc alsps_inst_init(sns_sensor_instance* const this,
    sns_sensor_state const *sstate);
sns_rc alsps_inst_deinit(sns_sensor_instance* const this);
void alsps_run_sensor_test(sns_sensor_instance *instance);
void alsps_set_brightness(alsps_instance_state *state, uint16_t brightness);

