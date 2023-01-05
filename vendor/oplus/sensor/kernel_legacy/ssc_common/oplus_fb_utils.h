/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_sensor_fb.cpp
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#pragma once

#define EVNET_DATA_LEN 3

enum sensor_fb_event_id {
    FD_HEAD_EVENT_ID = 0,
    //1~100
    PS_INIT_FAIL_ID = 1,
    PS_I2C_ERR_ID = 2,
    PS_ALLOC_FAIL_ID = 3,
    PS_ESD_REST_ID = 4,
    PS_NO_INTERRUPT_ID = 5,
    PS_FIRST_REPORT_DELAY_COUNT_ID = 6,
    PS_ORIGIN_DATA_TO_ZERO_ID = 7,

    //100~200
    ALS_INIT_FAIL_ID = 100,
    ALS_I2C_ERR_ID = 101,
    ALS_ALLOC_FAIL_ID = 102,
    ALS_ESD_REST_ID = 103,
    ALS_NO_INTERRUPT_ID = 104,
    ALS_FIRST_REPORT_DELAY_COUNT_ID = 105,
    ALS_ORIGIN_DATA_TO_ZERO_ID = 106,


    //200~300
    ACCEL_INIT_FAIL_ID = 200,
    ACCEL_I2C_ERR_ID = 201,
    ACCEL_ALLOC_FAIL_ID = 202,
    ACCEL_ESD_REST_ID = 203,
    ACCEL_NO_INTERRUPT_ID = 204,
    ACCEL_FIRST_REPORT_DELAY_COUNT_ID = 205,
    ACCEL_ORIGIN_DATA_TO_ZERO_ID = 206,

    //300~400
    GYRO_INIT_FAIL_ID = 300,
    GYRO_I2C_ERR_ID = 301,
    GYRO_ALLOC_FAIL_ID = 302,
    GYRO_ESD_REST_ID = 303,
    GYRO_NO_INTERRUPT_ID = 304,
    GYRO_FIRST_REPORT_DELAY_COUNT_ID = 305,
    GYRO_ORIGIN_DATA_TO_ZERO_ID = 306,

    //300~400
    MAG_INIT_FAIL_ID = 400,
    MAG_I2C_ERR_ID = 401,
    MAG_ALLOC_FAIL_ID = 402,
    MAG_ESD_REST_ID = 403,
    MAG_NO_INTERRUPT_ID = 404,
    MAG_FIRST_REPORT_DELAY_COUNT_ID = 405,
    MAG_ORIGIN_DATA_TO_ZERO_ID = 406,

    //600~700
    POWER_SENSOR_INFO_ID = 600,
    POWER_ACCEL_INFO_ID = 601,
    POWER_GYRO_INFO_ID = 602,
    POWER_MAG_INFO_ID = 603,
    POWER_PROXIMITY_INFO_ID = 604,
    POWER_LIGHT_INFO_ID = 605,
    POWER_WISE_LIGHT_INFO_ID = 606,
    POWER_WAKE_UP_RATE_ID = 607,

    //1000
    ALAILABLE_SENSOR_LIST_ID = 1000,
};

struct fd_data {
    int data_x;
    int data_y;
    int data_z;
};

struct fb_event {
    unsigned short event_id;
    unsigned int count;
    union {
        int buff[EVNET_DATA_LEN];
        struct fd_data data;
    };
};

extern int oplus_add_fd_event(struct fb_event *event);
extern int oplus_remove_fd_event(struct fb_event *event);
void oplus_init_cm_fw_sensor(sns_sensor_instance *this);
void oplus_collect_cm_info(sns_sensor_instance* this, const sns_sensor_uid *suid, int proc_type, int delivery_type);
void oplus_clear_cm_info(sns_sensor_instance* this, const sns_sensor_uid *suid);
void oplus_collect_wakeup_rate(float normal_rate, float island_rate);
float oplus_get_normal_wake_up_rate(void);
float oplus_get_island_wake_up_rate(void);
void oplus_set_sensor_wakeup_rate(const sns_sensor_uid *suid, float wakeup_rate);
void oplus_set_proximity_enable(bool enable);
bool oplus_is_resampler_request(const sns_sensor_uid *suid);
void oplus_parse_resampler_request(sns_request* request);
void oplus_save_power_info(void);
void oplus_save_available_sensor_info(void);