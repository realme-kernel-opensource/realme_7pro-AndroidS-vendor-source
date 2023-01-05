/******************************************************************
** Copyright (C), 2004-2020, OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_SMEM
** File: - oppo_sensor.h
** Description: Header file for oppo sensor.
** Version: 1.0
** Date : 2019/03/15
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#ifndef _OPPO_SENSOR_H_
#define _OPPO_SENSOR_H_

#include "com_dtypes.h"
#include <stdint.h>
#include <stdbool.h>
#define FEATURE1_FOREIGN_MASK   (1 << 0)

#define MAX_OCP 6
#define MAX_LEN 8
#define FEATURE_COUNT 10
#define OPPO_SENSOR_MAGIC  0X37847857
#define REG_NUM 10
#define PARAMETER_NUM 25
#define FEATURE_NUM 10
#define SOURCE_NUM 2
#define ALGO_PARAMETER_NUM 15
#define ALGO_FEATURE_NUM  5
#define DEFAULT_CONFIG 0xff //read from registry

typedef enum OPPO_PROJECT OPPO_PROJECT;

enum OPPO_PROJECT {
    OPPO_UNKNOWN = 0,
    OPPO_19101 = 19101,
    OPPO_19125 = 19125,
    OPPO_19127 = 19127,
};

enum f_index {
    IDX_1 = 1,
    IDX_2,
    IDX_3,
    IDX_4,
    IDX_5,
    IDX_6,
    IDX_7,
    IDX_8,
    IDX_9,
    IDX_10,
};



enum sensor_id {
    OPPO_ACCEL,
    OPPO_GYROSCOPE,//avoid confilct OPPO_GYRO
    OPPO_MAGNETIC,//OPPO_MAG
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

/*
enum  {
        STK3A5X = 0x01,
        TCS3701 = 0x02,
        TCS3408 = 0x04,
};
*/
enum {
    LSM6DSM = 0x01,
    BMI160 = 0x02,
    LSM6DS3_C = 0x04,
    BMI260 = 0x08,
    LSM6DSO = 0x10,
    LIS2HH12 = 0x20,
    BMA420 = 0x40,
    BMI3X0 = 0x80,
};

enum {
    AKM0991X = 0x01,
    MMC5603 = 0x02,
    MXG4300 = 0X04,
};

enum {
    LPS22HH = 0x01,
    BMP380 = 0x02,
};

enum {
    SX9324 = 0x01,
};

struct sensor_direction {
    int8  sign[3];
    uint8 map[3];
};
typedef struct {
    uint32  nProject;
    uint32  nDtsi;
    uint32  nAudio;
    uint32  nRF;
    uint32  nFeature[FEATURE_COUNT];
    uint32  nOppoBootMode;
    uint32  nPCB;
    uint8   nPmicOcp[MAX_OCP];
    uint8   reserved[16];
} ProjectInfoCDTType;


struct sensor_feature {
    int reg[REG_NUM];
    int parameter[PARAMETER_NUM];
    int feature[FEATURE_NUM];
};

struct sensor_hw {
    uint8_t sensor_name;
    uint8_t bus_number;
    uint8_t direction;
    uint8_t irq_number;
    struct sensor_feature feature;
};

struct sensor_vector {
    int sensor_id;
    struct sensor_hw hw[SOURCE_NUM];
};


struct sensor_algorithm {
    int sensor_id;
    int parameter[ALGO_PARAMETER_NUM];
    int feature[ALGO_FEATURE_NUM];
};

struct sensor_data {
    int magic_num;
    struct sensor_vector s_vector[SENSORS_NUM];
    struct sensor_algorithm a_vector[SENSOR_ALGO_NUM];
};

bool oppo_get_sensor_hw(int sensor_id, int sensor_name, struct sensor_hw **hw);
bool oppo_get_sensor_feature(int sensor_id, int sensor_name, struct sensor_feature **feature);
bool oppo_get_virtual_sensor(int sensor_id, struct sensor_algorithm **algo);
struct sensor_data *get_sensor_info(void);
void init_sensor_info(void);
//unsigned int get_project(void);
//unsigned int is_project(OPPO_PROJECT project);
//unsigned char get_Oppo_Boot_Mode(void);
int get_direction(uint8 num, struct sensor_direction *s_dir);
uint32 get_oppo_feature(enum f_index index);
#endif  /*_OPPO_SENSOR_H_*/
