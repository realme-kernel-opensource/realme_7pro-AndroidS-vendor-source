/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - xxx
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/
#pragma once

#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"
#include "../../logger/inc/sns_logger_sensor_instance.h"
#include "sns_mem_util.h"
#include "oplus_list.h"
#include "../slit_prox_algo/psensor_algo.h"
#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_fb_utils.h"
#endif


#define ALSPS_MALLOC(x)  sns_malloc(SNS_HEAP_MAIN, x)
#define ALSPS_ISLAND_MALLOC(x)  sns_malloc(SNS_HEAP_ISLAND, x)
#define ALS_CALI_NUM 5
#define PS_CALI_NUM 3
#define ALS_CALI_TARGET_LUX 1024
#define DEFAULT_SAMPLING_RATE_HZ 15


enum stream_type {
    UNKNOWN = 0,
    TIMER_STREAM,
    IRQ_STREAM,
    REASMPLER_STREAM,
};

enum stream_id {
    S_HEAD = 0,
    PS_IRQ,
    PS_FACTORY_CALI,
    ALS_REPORT_DATA,
    ALS_FACTORY_CALI,
    PS_MONITOR_TIMRER,
    ACCEL_DATA,
    ALS_HEARTBEAT_TIMER
};

enum PS_CAIL_PARAMS_TYPE {
    PS_CAIL_DYNAMIC_PARAMS = 0,
    PS_CAIL_FACTORY_3CM,
    PS_CAIL_FACTORY_UNCOVERED,
    PS_CAIL_DYNAMIC_PARAMS_LOW,
    PS_CAIL_FACTORY_3CM_LOW,
    PS_CAIL_FACTORY_UNCOVERED_LOW
};

enum CALI_STATE {
    ALSPS_CALI_IDLE,
    ALSPS_CALI_RUNNING,
    ALSPS_CALI_FAILED,
    ALSPS_CALI_DONE
};

enum PS_CALI_TYPE {
    PS_CALI_CROSSTALK = 0,
    PS_CALI_3CM
};

// bit[0] = 1 means stk3a5x
// bit[1] = 1 means tcs3701
// bit[2] = 1 means TCS3408
// bit[3] = 1 means stk326x

enum VENDOR_ID {
    STK3A5X = 0x01,
    TCS3701 = 0x02,
//  TCS3408 = 0x04,
    STK3A6X = 0x08,
    MN78911 = 0x10,
};

struct vendor_node {
    uint8_t id;
    uint8_t slave_addr;
    uint8_t pid;
    struct alsps_als_operations *als_ops;
    struct alsps_ps_operations *ps_ops;
    struct ps_algo *ps_cali_parm;
    char name[15];
};

struct stream_node {
    struct list_head list;
    const char *name;
    enum stream_type type;
    int stream_id;
    bool is_vilid;
    sns_data_stream* data_stream;
    sns_sensor_instance *instance;
};

struct alsps_als_operations {
    sns_rc (*get_who_am_i) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*init_driver) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*deinit_driver) (void);
    sns_rc (*init_irq) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*als_enable) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, bool enable);
    sns_rc (*clear_als_int) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*get_als_data) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, float *raw_data, int len,
        uint8_t als_type, bool is_als_dri);
    sns_rc (*get_als_device_irq_mask) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, uint8_t * mask);
    void (*dump_reg)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    void (*reconfig_reg_table)(uint8_t reg_num, uint8_t* reg_table);

    void (*enable_fifo) (bool use_fifo);
    sns_rc (*get_als_fifo_data) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, float *raw_data,int len);
    void (*set_brightness) (uint16_t brightness);
};

struct alsps_ps_operations {
    sns_rc (*get_who_am_i) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*init_driver) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*deinit_driver) (void);
    sns_rc (*init_irq) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*ps_enable) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, bool enable);
    sns_rc (*clear_ps_int) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*get_ps_device_irq_mask) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, uint8_t * mask);
    sns_rc (*read_irq_flag) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, uint8_t* status);
    sns_rc (*get_ps_data) (sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle, uint16_t *raw_data);
    sns_rc (*ps_set_thd)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        uint16_t ps_thd_near,
        uint16_t ps_thd_far,
        alsps_ps_state status);
    sns_rc (*set_offset)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        int val);
    sns_rc (*get_offset)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        int *val);
    sns_rc (*get_ps_original_data)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        int *val);
    sns_rc (*get_ir_data)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        int *val);
    sns_rc (*hardware_cali)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    sns_rc (*get_hwcali_result)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle,
        bool *result);
    void (*dump_reg)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    bool (*prox_need_ir_info)(void);
    sns_rc (*recover_device)(sns_sync_com_port_service *scp_service,
        sns_sync_com_port_handle *port_handle);
    void (*reconfig_reg_table)(uint8_t reg_num, uint8_t* reg_table);
    void (*special_process_before_avaliable)(alsps_state *state);
    void (*ps_offset_cali)(float data_0,float data_5,int *offset,float *temp_offset);
};

struct alsps_als_operations stk3a5x_als_ops;
struct alsps_ps_operations stk3a5x_ps_ops;
struct alsps_als_operations tcs3701_als_ops;
struct alsps_ps_operations tcs3701_ps_ops;
struct alsps_als_operations stk3a6x_als_ops;
struct alsps_ps_operations stk3a6x_ps_ops;
struct alsps_als_operations mn78911_als_ops;
struct alsps_ps_operations mn78911_ps_ops;

void alsps_remove_stream_safe(int stream_id);

void alsps_als_enable(sns_sensor_instance *const this, bool enable);
void alsps_ps_enable(sns_sensor_instance *const this, bool enable);
void alsps_send_config_event(sns_sensor_instance *const instance, sns_time timestamp, alsps_sensor_type sensor);
int alsps_register_als(sns_sensor_instance *const this, struct alsps_als_operations *ops);
int alsps_register_ps(sns_sensor_instance *const this, struct alsps_ps_operations *ops);
int alsps_register_timer(sns_sensor_instance *this, int stream_id, sns_time period, bool periodic);
int alsps_register_interrupt(sns_sensor_instance *this, int stream_id);
void alsps_process_events(sns_sensor_instance *this);
sns_rc alsps_get_who_am_i(sns_sensor *this);
sns_rc alsps_dump_reg(sns_sensor_instance *this, alsps_sensor_type sensor);
sns_rc alsps_init_irq(sns_sensor_instance *this, alsps_sensor_type sensor);
sns_rc alsps_ps_set_offset(sns_sensor_instance *const this, int val);
int alsps_register_stream(sns_sensor_instance *this, int stream_id, enum stream_type type);
int alsps_unregister_stream(int stream_id);
struct vendor_node * alsps_get_s_list(void);
bool get_phone_status(sns_sensor_instance *this);

void alsps_ps_logger_report_raw_data(sns_sensor_instance *this);
void alsps_ps_monitor_thread(sns_sensor_instance *this);
void alsps_ps_hardware_cali_handle(sns_sensor_instance *this);
int alsps_register_accel_by_resampler(sns_sensor_instance *this, int stream_id, uint64_t batch_period, uint64_t resample_rate);
bool get_als_fac_mode(sns_sensor_instance *this);
sns_rc alsps_deinit_driver (sns_sensor_instance *const this, alsps_sensor_type sensor);
sns_rc alsps_init_driver(sns_sensor_instance *const this, alsps_sensor_type sensor);
sns_rc alsps_reconifg_reg_table(sns_sensor * this);
bool ps_need_gesture();
bool select_ps_offset_cal(void);
