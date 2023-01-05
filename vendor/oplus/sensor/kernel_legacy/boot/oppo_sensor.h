/******************************************************************
** Copyright (C), 2004-2020, OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_SMEM
** File: - oppo_sensor.h
** Description: Header file for opp_sensor.
** Version: 1.0
** Date : 2020/04/04
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/

#ifndef _OPPO_SENSOR_H_
#define _OPPO_SENSOR_H_

enum sensor_id {
    OPPO_ACCEL,
    OPPO_GYRO,
    OPPO_MAG,
    OPPO_LIGHT,
    OPPO_PROXIMITY,
    OPPO_SAR,
    OPPO_CCT,
    OPPO_CCT_REAR,
    OPPO_BAROMETER,
    SENSORS_NUM
};

enum sensor_algo_id {
    OPPO_PICKUP_DETECT,
    OPPO_LUX_AOD,
    OPPO_TP_GESTURE,
    OPPO_FP_DISPLAY,
    OPPO_FREE_FALL,
    OPPO_CAMERA_PROTECT,
    SENSOR_ALGO_NUM
};

#define REG_NUM 10
#define PARAMETER_NUM 25
#define FEATURE_NUM 10
#define SOURCE_NUM 2
#define ALGO_PARAMETER_NUM 15
#define ALGO_FEATURE_NUM  5

typedef struct {
    int reg[REG_NUM];
    int parameter[PARAMETER_NUM];
    int feature[FEATURE_NUM];
} sensor_feature;

typedef struct {
    char sensor_name;
    char bus_number;
    char direction;
    char irq_number;
    sensor_feature feature;
} sensor_hw;

typedef struct {
    int sensor_id;
    sensor_hw hw[SOURCE_NUM];
} sensor_vector;


typedef struct {
    int sensor_id;
    int parameter[ALGO_PARAMETER_NUM];
    int feature[ALGO_FEATURE_NUM];
} sensor_algorithm;

typedef struct {
    int magic_num;
    sensor_vector s_vector[SENSORS_NUM];
    sensor_algorithm a_vector[SENSOR_ALGO_NUM];
} sensor_info;

#define EVNET_NUM_MAX 109
#define EVNET_DATA_LEN 3

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

struct list_head {
    struct list_head *next, *prev;
};

struct fb_event_node {
    struct list_head list;
    struct fb_event event;
};

struct fb_event_smem {
    struct fb_event event[EVNET_NUM_MAX];
};

#endif//_OPPO_SENSOR_H_

