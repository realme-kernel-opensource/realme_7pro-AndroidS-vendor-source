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

#include "sns_mem_util.h"
#include "sns_memmgr.h"
#include "sns_rc.h"
#include "sns_types.h"
#include "sns_timer.pb.h"
#include "sns_sensor_uid.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_sensor_util.h"
#include "sns_interrupt.pb.h"
#include "sns_printf_int.h"
#include "sns_sync_com_port_service.h"
#include "sns_com_port_types.h"
#include "sns_pwr_rail_service.h"
#include "sns_registry_util.h"
#include "sns_island_service.h"
#include "sns_oplus_iface.pb.h"

#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_fb_utils.h"
#endif

#ifndef EINVAL
#define EINVAL 13
#endif

#define ALSPS_DRIVER_VERSION         0x001

/* ALSPS ODR (Hz) definitions */
#define ALSPS_ODR_2                  2.0f
#define ALSPS_ODR_5                  5.0f
#define ALSPS_ODR_10                 10.0f
#define ALSPS_ODR_15                 15.0f
#define ALSPS_ODR_20                 20.0f
#define ALSPS_ODR_50                 50.0f
#define ALSPS_ODR_100                100.0f
/* ALSPS ALS ranges */
#define ALSPS_ALS_RANGE_MIN          0.0f
#define ALSPS_ALS_RANGE_MAX          65535.0f
/* ALSPS ALS resolutions */
#define ALSPS_ALS_RESOLUTION         0.1f
/* ALSPS ALS power */
#define ALSPS_ALS_ACTIVE_CURRENT     80
#define ALSPS_ALS_SLEEP_CURRENT      1
/* ALSPS PROX ranges */
#define ALSPS_PROX_RANGE_MIN         0.0f
#define ALSPS_PROX_RANGE_MAX         5.0f
/* ALSPS PROX resolutions */
#define ALSPS_PROX_RESOLUTION        1.0f
/* ALSPS PROX power */
#define ALSPS_PROX_ACTIVE_CURRENT    1000
#define ALSPS_PROX_SLEEP_CURRENT     1
/* Supported operating modes */#define ALSPS_LPM                    "LPM"
#define ALSPS_NORMAL                 "NORMAL"

#define ALSPS_CONFIG_ALS              "alsps.als.config"
#define ALSPS_CONFIG_PROX             "alsps.prox.config"
#define ALSPS_PLATFORM_ALS            "alsps_platform.als"
#define ALSPS_PLATFORM_PROX           "alsps_platform.prox"
#define ALSPS_PLATFORM_FAC_CAL_ALS    "alsps_platform.als.fac_cal"
#define ALSPS_PLATFORM_FAC_CAL_PROX   "alsps_platform.prox.fac_cal"
#define ALSPS_PLATFORM_CONFIG         "alsps_platform.config"
#define ALSPS_OFF_TO_IDLE_MS 100 //ms
#define ALS_PS_RAIL_OFF_TIMEOUT_NS 1000000000ULL /* 1 second */

#define PS_FAC_CAL_NUM 6
#define ALS_FAC_CAL_NUM 2

#define ALS_EVENT_SIZE 3
#define PROX_EVENT_SIZE 2

#define MAX_REG_TABLE_LEN 16

#define PS_DEFAULT_NEAR_THRESHOLD     500
#define PS_DEFAULT_FAR_THRESHOLD      400

#define ALSPS_ALS_SUID \
    {  \
        .sensor_uid =  \
        {  \
            0x62, 0x61, 0x43, 0x54, 0x52, 0x4C, 0x22, 0x35,  \
            0x39, 0x31, 0x91, 0x2C, 0x43, 0x5D, 0x52, 0x5F  \
        }  \
    }

#define ALSPS_PS_SUID \
    {  \
        .sensor_uid =  \
        {  \
            0x22, 0x31, 0x44, 0x54, 0x62, 0x8C, 0x72, 0x95,  \
            0x99, 0x61, 0x31, 0x8C, 0x13, 0x1D, 0x22, 0x3F  \
        }  \
    }

typedef enum {
    ALS = 0x01,
    PS = 0x02,
} alsps_sensor_type;

typedef enum {
    ALS_INT = 0x01,
    PS_INT = 0x02,
    PS_CALI = 0x04,
} alsps_sensor_inr_type;

typedef enum {
    NORMAL = 0x01,
    UNDER_LCD = 0x02,
    VIRTUAL_UNDER_LCD = 0x03,
} als_position_type;

typedef enum {
    PS_NORMAL = 0x01,
    PS_TYPE_Y = 0x02,
    PS_UNDER_LCD = 0x03,
} ps_position_type;

typedef enum {
    PRX_FAR_AWAY,
    RX_NEAR_BY,
    PRX_NEAR_BY_UNKNOWN
} alsps_ps_state;

typedef enum {
    SOFTWARE_CAIL = 0x01,
    HARDWARE_CAIL = 0x02,
} ps_calibration_type;

typedef struct alsps_pb_parse_arg {
    float    *data_array;
    uint32_t *version_array;
    uint8_t  *index;
    uint8_t  size;
} alsps_pb_parse_arg;

typedef struct pb_arg_reg_group_arg {
    sns_sensor_instance* instance;
    const char*          name;
    alsps_sensor_type sensor;
} pb_arg_reg_group_arg;

typedef enum {
    ALSPS_POWER_RAIL_PENDING_NONE,
    ALSPS_POWER_RAIL_PENDING_INIT,
    ALSPS_POWER_RAIL_PENDING_SET_CLIENT_REQ,
    ALSPS_POWER_RAIL_PENDING_OFF,
} alsps_power_rail_pending_state;

typedef struct alsps_com_port_info {
    sns_com_port_config      com_config;
    sns_sync_com_port_handle *port_handle;
} alsps_com_port_info;

typedef enum {
    ALSPS_NONE_PRESENT = 0x0,
    ALSPS_ALS_PRESENST = 0x1,
    ALSPS_PS_PRESENT = 0x2,
    ALSPS_PRESENT = 0x3,
} alsps_present_state;

typedef struct alsps_state {
    /* Sensor Type */
    alsps_sensor_type     sensor;

    sns_sensor_uid          my_suid;
    sns_sensor_uid          reg_suid;
    sns_sensor_uid          irq_suid;
    sns_sensor_uid          timer_suid;
    sns_sensor_uid          accel_suid;
    sns_sensor_uid          resampler_suid;

    alsps_com_port_info   com_port_info;
    sns_interrupt_req       irq_config;

    sns_rail_config         rail_config;
    sns_power_rail_state    registry_rail_on_state;
    alsps_power_rail_pending_state    power_rail_pend_state;

    sns_diag_service        *diag_service;
    sns_sync_com_port_service *scp_service;
    sns_pwr_rail_service    *pwr_rail_service;
    sns_island_service    *island_service;

    sns_data_stream         *fw_stream;
    sns_data_stream         *reg_data_stream;
    sns_data_stream         *timer_stream;
    bool    hw_is_present;
    bool    start_hw_detect;
    int64_t    hardware_id;
    uint8_t    resolution_idx;
    bool    supports_sync_stream;

    /* Registry sensor platform config */
    bool                    registry_pf_cfg_received;
    sns_registry_phy_sensor_pf_cfg registry_pf_cfg;


    float                   als_fac_cal_data[ALS_FAC_CAL_NUM];
    float                   ps_fac_cal_data[PS_FAC_CAL_NUM];
    uint32_t                als_cal_version[ALS_FAC_CAL_NUM];
    uint32_t                ps_cal_version[PS_FAC_CAL_NUM];
    bool    registry_cfg_received;
    bool    registry_fac_cal_received;
    bool    is_dri;

    uint32_t    als_encoded_event_len;
    uint32_t    ps_encoded_event_len;
    sns_registry_phy_sensor_cfg als_registry_cfg;
    sns_registry_phy_sensor_cfg prox_registry_cfg;

    /* decided by dynamic probe */
    int8_t    vendor_id;

    /* decided by different project config,match_id == vendor_id, it real probe*/
    uint32_t ps_match_id;
    uint32_t als_match_id;
    int als_type;
    int ps_type;
    uint8_t      is_als_initialed;
    uint8_t      is_unit_device;
    uint8_t      is_als_dri;
    uint8_t   irq_number;
    uint8_t   bus_number;
    uint32_t  als_factor;
    uint32_t  als_buffer_length;
    uint8_t   reg_num;
    uint8_t reg_table[MAX_REG_TABLE_LEN];
    bool is_need_check_pd;
    bool use_lb_algo; // low-brightness algo
    uint16_t ps_saturation;
    uint32_t ps_factory_cali_max;
} alsps_state;


sns_rc alsps_sensor_notify_event(sns_sensor *const this);

void alsps_reval_instance_config(sns_sensor *this,
    sns_sensor_instance *instance,
    alsps_sensor_type sensor_type);
sns_rc alsps_sensor_notify_event(sns_sensor *const this);

sns_sensor_instance* alsps_set_client_request(sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove);
sns_rc alsps_als_init(sns_sensor *const this);
sns_rc alsps_ps_init(sns_sensor *const this);
sns_rc alsps_als_deinit(sns_sensor *const this);
sns_rc alsps_ps_deinit(sns_sensor *const this);
sns_sensor_uid const* alsps_als_get_sensor_uid(sns_sensor const *const this);
sns_sensor_uid const* alsps_ps_get_sensor_uid(sns_sensor const *const this);
void alsps_send_suid_req(sns_sensor *this,
    char *const data_type, uint32_t data_type_len);
void alsps_common_init(sns_sensor *const this);
void alsps_publish_registry_attributes(sns_sensor *const this);
sns_sensor_instance * alsps_get_instance (struct sns_sensor *this,
    struct sns_request const *request);
void alsps_process_suid_events(sns_sensor *const this);
void alsps_request_registry(sns_sensor *const this);
void alsps_publish_available(sns_sensor *const this);
bool alsps_discover_hw(sns_sensor * const this);
void alsps_start_power_rail_timer(sns_sensor *const this,
    sns_time timeout_ticks,
    alsps_power_rail_pending_state pwr_rail_pend_state);
void alsps_start_hw_detect_sequence(sns_sensor *const this);
void alsps_sensor_process_registry_event(sns_sensor *const this,
    sns_sensor_event *event);
sns_rc alsps_als_factory_cali(sns_sensor_instance *const this);
sns_rc alsps_ps_factory_cali(sns_sensor_instance *const this, uint8_t type);
void alsps_update_sensor_state(sns_sensor *const this,
    sns_sensor_instance *const instance);
void alsps_update_registry(sns_sensor *const this,
    sns_sensor_instance *const instance, alsps_sensor_type sensor);
void alsps_publish_name(sns_sensor *const this, char *name);
int get_als_normal_value(void);
