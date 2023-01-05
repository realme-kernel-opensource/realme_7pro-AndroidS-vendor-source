/*******************************************************************************
 * Copyright (c) 2017-2020, Bosch Sensortec GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file sns_bmi26x_sensor.h
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#pragma once

#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_sensor_uid.h"
#include "sns_pwr_rail_service.h"
#include "sns_island_service.h"

#include "sns_math_util.h"
#include "sns_diag_service.h"
#include "sns_registry_util.h"
#include "sns_suid_util.h"

#include "sns_bmi26x_config.h"
#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor_instance.h"

#define ACCEL_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xce, 0x37, 0x4e, 0xb8, 0x10, 0x85, 0x11, 0xe8, 0xb6, 0x42, 0x0e, 0xd5, 0xf8, 0x9f, 0x4d, 0x42\
    }  \
}

#define GYRO_SUID \
{  \
    .sensor_uid =  \
    {  \
        0x76, 0x3a, 0x4e, 0xb8, 0x10, 0x85, 0x11, 0xe8, 0xb6, 0x42, 0x0e, 0xd5, 0xf8, 0x9f, 0x4d, 0x42\
    }  \
}

#define MOTION_DETECT_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xc0, 0x3b, 0x4e, 0xb8, 0x10, 0x85, 0x11, 0xe8, 0xb6, 0x42, 0x0e, 0xd5, 0xf8, 0x9f, 0x4d, 0x42\
    }  \
}


#define SENSOR_TEMPERATURE_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xad, 0x58, 0xa1, 0xf3, 0xa8, 0x9c, 0x48, 0x6f, 0xa7, 0xde, 0x33, 0xd6, 0x91, 0xe7, 0x49, 0x46\
    }  \
}

#if BMI26X_CONFIG_ENABLE_PEDO
#define PEDO_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xcc, 0x84, 0x81, 0xbd, 0x60, 0xcf, 0x44, 0x88, 0xad, 0xf9, 0xcd, 0x3c, 0x80, 0x49, 0x43, 0x71\
    }  \
}
#endif

#if BMI26X_CONFIG_ENABLE_OIS
#define OIS_SENSOR_SUID \
{  \
    .sensor_uid =  \
    {  \
        0x38, 0x22, 0x92, 0x0f, 0xca, 0x68, 0x4f, 0xc1, 0xaa, 0xc9, 0x74, 0x4d, 0x3f, 0x40, 0x4a, 0x35\
    }  \
}
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
#define LOWG_SENSOR_SUID \
{  \
    .sensor_uid =  \
    {  \
        0x39, 0x22, 0x92, 0x0f, 0xca, 0x68, 0x4f, 0xc1, 0xaa, 0xc9, 0x74, 0x4d, 0x3f, 0x40, 0x4a, 0x35\
    }  \
}
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
#define DOUBLE_TAP_SENSOR_SUID \
{  \
    .sensor_uid =  \
    {  \
        0xff, 0xbb, 0xf4, 0x67, 0xc6, 0xdd, 0x4a, 0x50, 0xad, 0x7d, 0x61, 0x83, 0xbc, 0x40, 0x35, 0xc5\
    }  \
}
#endif


/** Forward Declaration of Accel Sensor API */
extern sns_sensor_api bmi26x_accel_sensor_api;

/** Forward Declaration of Gyro Sensor API */
extern sns_sensor_api bmi26x_gyro_sensor_api;

/** Forward Declaration of Motion Accel Sensor API */
extern sns_sensor_api bmi26x_motion_detect_sensor_api;

/** Forward Declaration of Sensor Temperature Sensor API */
extern sns_sensor_api bmi26x_sensor_temp_sensor_api;

#if BMI26X_CONFIG_ENABLE_PEDO
extern sns_sensor_api bmi26x_pedo_api;
#endif

#if BMI26X_CONFIG_ENABLE_OIS
extern sns_sensor_api bmi26x_ois_api;
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
extern sns_sensor_api bmi26x_free_fall_api;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
extern sns_sensor_api bmi26x_double_tap_api;
#endif


/**
 * BMI26X Power consumption definition
 */
#define BMI26X_ACC_ACTIVE_CURRENT     180
#define BMI26X_ACC_LOWPOWER_CURRENT   6

#define BMI26X_GYRO_ACTIVE_CURRENT    900
#define BMI26X_GYRO_LOWPOWER_CURRENT  6

#define BMI26X_MOTION_DETECTION_ACTIVE_CURRENT  12
#define BMI26X_MOTION_DETECTION_LOWPOER_CURRENT 6

#define BMI26X_TEMP_ACTIVE_CURRENT    180
#define BMI26X_TEMP_LOWPOWER_CURRENT  6

#if BMI26X_CONFIG_ENABLE_LOWG
#define BMI26X_FREE_FALL_ACTIVE_CURRENT   12
#define BMI26X_FREE_FALL_LOWPOER_CURRENT  6
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
#define BMI26X_DOUBLE_TAP_ACTIVE_CURRENT   12
#define BMI26X_DOUBLE_TAP_LOWPOER_CURRENT  6
#endif


/**
 * BMI26X ODR (Hz) definitions
 */
#define BMI26X_ODR_0                 0.0
#define BMI26X_ODR_12P5              12.5
#define BMI26X_ODR_25                25.0
#define BMI26X_ODR_50                50
#define BMI26X_ODR_100               100
#define BMI26X_ODR_200               200
#define BMI26X_ODR_400               400
#define BMI26X_ODR_800               800
#define BMI26X_ODR_1600              1600
#define BMI26X_ODR_3200              3200.0
#define BMI26X_ODR_6400              6400.0

/**
 * Accelerometer ranges
 */
#define BMI26X_ACCEL_RANGE_2G_MIN    (-2)
#define BMI26X_ACCEL_RANGE_2G_MAX    (2)
#define BMI26X_ACCEL_RANGE_4G_MIN    (-4)
#define BMI26X_ACCEL_RANGE_4G_MAX    (4)
#define BMI26X_ACCEL_RANGE_8G_MIN    (-8)
#define BMI26X_ACCEL_RANGE_8G_MAX    (8)
#define BMI26X_ACCEL_RANGE_16G_MIN   (-16)
#define BMI26X_ACCEL_RANGE_16G_MAX   (16)

/**
 * Accelerometer resolutions
 */
#define BMI26X_ACCEL_RESOLUTION_2G    (0.061)
#define BMI26X_ACCEL_RESOLUTION_4G    (0.122)
#define BMI26X_ACCEL_RESOLUTION_8G    (0.244)
#define BMI26X_ACCEL_RESOLUTION_16G   (0.488)


/* in the unit of micro-g/lsb */
#define BMI26X_ACCEL_SSTVT_2G    (1e6 * 4 / 65536)
#define BMI26X_ACCEL_SSTVT_4G    (1e6 * 8 / 65536)
#define BMI26X_ACCEL_SSTVT_8G    (1e6 * 16 / 65536)
#define BMI26X_ACCEL_SSTVT_16G    (1e6 * 32 / 65536)


#define BMI26X_SCALE_FACTOR_DATA_ACCEL     1e7  //can be 1e6 if we use +/- 8g or +/-16g range is used
#define BMI26X_SCALE_FACTOR_DATA_DFT       1e6
#define BMI26X_SCALE_FACTOR_DATA_GYRO      BMI26X_SCALE_FACTOR_DATA_DFT
#define BMI26X_SCALE_FACTOR_DATA_TEMP      BMI26X_SCALE_FACTOR_DATA_DFT
/**
 * Gyroscope ranges
 */
#define BMI26X_GYRO_RANGE_125_MIN    (-125)
#define BMI26X_GYRO_RANGE_125_MAX    (125)
#define BMI26X_GYRO_RANGE_250_MIN    (-250)
#define BMI26X_GYRO_RANGE_250_MAX    (250)
#define BMI26X_GYRO_RANGE_500_MIN    (-500)
#define BMI26X_GYRO_RANGE_500_MAX    (500)
#define BMI26X_GYRO_RANGE_1000_MIN   (-1000)
#define BMI26X_GYRO_RANGE_1000_MAX   (1000)
#define BMI26X_GYRO_RANGE_2000_MIN   (-2000)
#define BMI26X_GYRO_RANGE_2000_MAX   (2000)



#define BMI26X_GYRO_RESOLUTION_125DPS    (250 * 1.0 / 65536)
#define BMI26X_GYRO_RESOLUTION_250DPS    (500 * 1.0 / 65536)
#define BMI26X_GYRO_RESOLUTION_500DPS    (1000 * 1.0 / 65536)
#define BMI26X_GYRO_RESOLUTION_1000DPS    (2000 * 1.0 / 65536)
#define BMI26X_GYRO_RESOLUTION_2000DPS    (4000 * 1.0 / 65536)

/**
 * BMI26X sensitivity for gyro in (mrad/sec)/LSB
 */
#define BMI26X_GYRO_SSTVT_125DPS    ((250 * PI / 180 * 1e6) / 65536)
#define BMI26X_GYRO_SSTVT_250DPS    ((500 * PI / 180 * 1e6) / 65536)
#define BMI26X_GYRO_SSTVT_500DPS    ((1000 * PI / 180 * 1e6) / 65536)
#define BMI26X_GYRO_SSTVT_1000DPS   ((2000 * PI / 180 * 1e6) / 65536)
#define BMI26X_GYRO_SSTVT_2000DPS   ((4000 * PI / 180 * 1e6) / 65536)

/**
 * BMI26X Sensor Temperature ODR (Hz) definitions
 */
#define BMI26X_SENSOR_TEMP_ODR_1        1.0
#define BMI26X_SENSOR_TEMP_ODR_5        5.0

/**
 * Sensor Temperature resolution in deg Celsius/LSB
 * 1/16 deg Celsius per LSB
 */
#define BMI26X_SENSOR_TEMPERATURE_RESOLUTION  (0.002)

/**
 * Sensor Temperature range in deg Celsius
 */
#define BMI26X_SENSOR_TEMPERATURE_RANGE_MIN  (-40.0)
#define BMI26X_SENSOR_TEMPERATURE_RANGE_MAX  (85.0)


#define SENSOR_NAME    "bmi26x"
#define SENSOR_TEMP_NAME    "bmi26x_temp"
#define VENDOR_NAME    "BOSCH"

/** Supported operating modes */
#define BMI26X_LPM          "LPM"
#define BMI26X_HIGH_PERF    "HIGH_PERF"
#define BMI26X_NORMAL       "NORMAL"

/** Power rail timeout States for the BMI26X Sensors.*/
typedef enum {
    BMI26X_POWER_RAIL_PENDING_NONE,          //!< BMI26X_POWER_RAIL_PENDING_NONE
    BMI26X_POWER_RAIL_PENDING_INIT,          //!< BMI26X_POWER_RAIL_PENDING_INIT
    BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ,//!< BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ
    BMI26X_POWER_RAIL_PENDING_DEINIT,
    BMI26X_POWER_RAIL_INVALID                //!< BMI26X_POWER_RAIL_INVALID
} bmi26x_power_rail_pending_state;


/**Registry items supported as part of free-fall
 * configuration registry group
 */
typedef struct
{
  uint8_t enable;
  uint8_t mode;
  uint8_t debug;
} sns_registry_free_fall_cfg_t;



/**Registry items supported as part of double-tap
 * configuration registry group
 */
typedef struct
{
  uint32_t enable                  : 1;
  uint32_t tap_sens_thres          : 4;
  uint32_t quite_time_after_gest   : 10;
  uint32_t max_gest_dur            : 16;
} sns_registry_db_tap_cfg_t;

/** Interrupt Sensor State. */


#if BMI26X_CONFIG_ENABLE_OIS || BMI26X_CONFIG_ENABLE_CRT
/*!
 * Definition for self-define state
 */
typedef struct bmi26x_self_define_state {
    void *priv;
    int crt_buffer[3];
} bmi26x_self_define_state_t;
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
/*!
 * Definition for self-define state
 */
typedef struct bmi26x_self_define_cfg_int_type {
    void *priv;
    int  cfg_buffer[8];
} bmi26x_self_define_cfg_int_type_t;
#endif



/**
 * Sensor common state structure
 */
typedef struct bmi26x_common_state {
#if BMI26X_CONFIG_ENABLE_DAE
    sns_data_stream                   *dae_stream;
    sns_sensor_uid                    dae_suid;
    bmi26x_dae_if_state_t             dae_ag_state;
    bmi26x_dae_if_state_t             dae_temper_state;
#endif
    bmi26x_com_port_info_t  com_port_info;
    sns_interrupt_req       irq_config;

    sns_rail_config         rail_config;
    sns_power_rail_state    registry_rail_on_state;


    SNS_SUID_LOOKUP_DATA(5) suid_lookup_data;

    uint32_t                who_am_i                     : 16;
    uint32_t                hw_is_present                : 1;
    uint32_t                hw_is_finish_detection       : 1;
    // registry sensor config
    uint32_t                registry_pf_cfg_received     : 1;
    uint32_t                registry_placement_received  : 1;
    uint32_t                registry_orient_received     : 1;

    // test
    uint32_t                init_flags : 8;
    uint32_t                crt_ver    : 4;

    sns_registry_phy_sensor_cfg     registry_cfg;

    // registry sensor platform config
    sns_registry_phy_sensor_pf_cfg  registry_pf_cfg;

    // axis conversion
    triaxis_conversion axis_map[TRIAXIS_NUM];

    // placement
    float                   placement[12];
} bmi26x_common_state_t;


/**
 * Sensor state structure
 */
typedef struct bmi26x_state {
    bmi26x_common_state_t   common;
#if BMI26X_CONFIG_ENABLE_REGISTRY
    sns_data_stream         *reg_data_stream;
#endif
    sns_data_stream         *fw_stream;
    sns_data_stream         *timer_stream;
    sns_pwr_rail_service    *pwr_rail_service;
    sns_diag_service        *diag_service;
    sns_sync_com_port_service *scp_service;

    sns_sensor_uid          my_suid;

    uint32_t scale_factor               : 24;   //up to 10^7
    uint32_t sensor                     : 12; //of bmi260_sensor_type
    //bm2160_power_rail_pending_state power_rail_pend_state;
    uint32_t power_rail_pend_state      : 3;

    // sensor enable flag
    uint32_t sensor_client_present : 1;
    // sensor configuration
    uint32_t is_dri : 1;
    uint32_t supports_sync_stream : 1;
    uint32_t registry_md_config_received : 1;

    uint32_t registry_cfg_received      : 1;
    uint32_t registry_fac_cal_received  : 1;
    uint32_t registry_attr_published    : 1;
#if BMI26X_CONFIG_ENABLE_CRT
    uint32_t registry_cfg_crt_received  : 1;
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    uint32_t registry_cfg_ois_received  : 1;
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    uint32_t registry_cfg_lowg_received : 1;
#endif


    uint32_t resolution_idx             : 4;
    uint32_t ss_flush_req               : 1;

    int64_t hardware_id;

    //TOPMZ: use union of fac_cal and md_config to save space
    // factory calibration
    matrix3 fac_cal_corr_mat;
    int32_t fac_cal_bias[TRIAXIS_NUM];
    float fac_cal_scale[TRIAXIS_NUM];

#if BMI26X_CONFIG_ENABLE_CRT
    bmi26x_gyr_crt_gain_t          crt_gain;
    uint32_t crt_version;
    bmi26x_gyr_multiple_crt_cfg_t  crt_cfg;
    bmi26x_gyr_multiple_crt_state_t crt_4_multiple_state;
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    bmi26x_gyr_ois_t               ois_cfg;
#endif


    uint32_t fac_cal_version;

    // md config
    sns_registry_md_cfg md_config;

#if BMI26X_CONFIG_ENABLE_LOWG
    sns_registry_free_fall_cfg_t lowg_config;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    sns_registry_db_tap_cfg_t   dbtap_cfg;
#endif

    size_t encoded_event_len;

    sns_sensor *owner;

} bmi26x_state;


/**
 * notify_event() Sensor API common between all BMI26X Sensors.
 *
 * @param[in] this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc bmi26x_sensor_notify_event(sns_sensor *const this);

/**
 * set_client_request() Sensor API common between all BMI26X
 * Sensors.
 *
 * @param[in] this            Sensor reference
 * @param[in] exist_request   existing request
 * @param[in] new_request     new request
 * @param[in] remove          true to remove request
 *
 * @return a sns_sensor_instance pointer
 */
sns_sensor_instance* bmi26x_set_client_request(sns_sensor *const            this,
        struct sns_request const    *exist_request,
        struct sns_request const    *new_request,
        bool                        remove);

/*!
 * The sensor init Sensor API for Accelemotor Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_accel_init(sns_sensor *const this);

/*!
 * The sensor init Sensor API for Gyroscope Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_gyro_init(sns_sensor *const this);

/*!
 * The sensor init Sensor API for Motion Detection Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_motion_detect_init(sns_sensor *const this);

/*!
 * The sensor init Sensor API for Sensor Temperature
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_sensor_temp_init(sns_sensor *const this);

/*!
 * The sensor deinit Sensor API for Accelemotor Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_accel_deinit(sns_sensor *const this);

/*!
 * The sensor deinit Sensor API for Accelemotor Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_gyro_deinit(sns_sensor *const this);

/*!
 * The osensor deinit Sensor API for Motion Detection Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_motion_detect_deinit(sns_sensor *const this);


/*!
 * The sensor deinit Sensor API for Sensor Temperature
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_sensor_temp_deinit(sns_sensor *const this);

#if BMI26X_CONFIG_ENABLE_PEDO
/*!
 * The sensor init Sensor API for Pedometer Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_pedo_init(sns_sensor * const this);

/*!
 * The sensor deinit Sensor API for Gyroscope Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_pedo_deinit(sns_sensor * const this);
#endif

#if BMI26X_CONFIG_ENABLE_OIS
/*!
 * The sensor init Sensor API for OIS Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_ois_init(sns_sensor * const this);

/*!
 * The sensor deinit Sensor API for OIS Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_ois_deinit(sns_sensor * const this);
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
/*!
 * The sensor init Sensor API for Free Fall Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_free_fall_init(sns_sensor * const this);

/*!
 * The sensor deinit Sensor API for Free Fall Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_free_fall_deinit(sns_sensor * const this);
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
/*!
 * The sensor init Sensor API for double-tap Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_double_tap_init(sns_sensor * const this);

/*!
 * The sensor deinit Sensor API for double-tap Sensor
 *
 * @param[in] this    The sensor handler
 */
sns_rc bmi26x_double_tap_deinit(sns_sensor * const this);
#endif


/*!
 * The Sensors Common initialization function
 *
 * @param[in] this    The sensor handler
 */
void bmi26x_common_init(sns_sensor *const this);


/*!
 * Start the power rail timer to wait the power rail on
 *
 * @param[in] this   The sensor handler
 * @param[in] timeout_ticks  The timer timeout ticks from start time
 * @param[in] pwr_rail_pend_state   The Power Rail Pending State
 *
 * @return[out] SNS_RC_SUCCESS    success if success
 *              others value on failure
 */
sns_rc bmi26x_start_power_rail_timer(sns_sensor *const this,
                                   sns_time timeout_ticks,
                                   bmi26x_power_rail_pending_state pwr_rail_pend_state);

/**
 * Publish the sensor availability
 *
 * @param[in] this   sensor handler
 * @return none
 */
void bmi26x_publish_available(sns_sensor *const this);


/**
 * Update IMU the sibling sensors
 *
 * @param[in] this   sensor handler
 */
void bmi26x_update_sibling_sensors(sns_sensor *const this);

/**
 * BMI26X HW detection function
 *
 * @param[in] this   sensor handler
 */
void bmi26x_start_hw_detect_sequence(sns_sensor *this);

/*!
 * Function to discover the HW after the platform is ready
 *
 * @param[in] this  The sensor state
 * @return      TRUE  when find the rigt sensor
 *              FALSE on failure
 */
bool bmi26x_discover_hw(sns_sensor *const this);

// <registry>
#if BMI26X_CONFIG_ENABLE_REGISTRY

/*!
 * Function to process reigistery event for the registry get request from the sensor
 *
 * @param[in] this  The sensor handler
 * @param[in] event Then registery event as the registry get request
 */
void bmi26x_sensor_process_registry_event(sns_sensor *const this,
        sns_sensor_event *event);

/*!
 * Sensor publish registry attributes to the framework
 *
 * @param[in]   this  The sensor handler
 */
void bmi26x_publish_registry_attributes(sns_sensor *const this);

/*!
 * Sensor publish default registry attributes with default value
 * @param this   Sensor Handler
 * @param ss_type Sensor Type
 */
void bmi26x_sensor_publish_default_registry_attributes(sns_sensor *const this, bmi26x_sensor_type ss_type);

/*!
 * Check the registry collection progresss
 *
 * @param[in]  this  The sensor handler
 */
void bmi26x_sensor_check_registry_col_progress(sns_sensor *const this);

/*!
 * Update registry interface
 *
 * @param[in]  this         The sensor handler
 * @param[in]  instance     The instance used now
 * @param[in]  sensor       The master sensor to call this function
 */
void bmi26x_update_registry(sns_sensor *const this,
                            sns_sensor_instance *const instance, bmi26x_sensor_type sensor);


/*!
 * Sensor request the registries
 *
 * @param[in]   this  The sensor handler
 */
void bmi26x_request_registry(sns_sensor *const this);

#if  BMI26X_CONFIG_ENABLE_CRT
/*!
 * Update the CRT parameters to the registry
 *
 * @param[in]  this  The sensor handler
 * @param[in]  instance  The instance used currently
 * @param[in]  sensor    The master sensor to update the crt registry
 */
void bmi26x_crt_update_registry(sns_sensor *const this,
                                sns_sensor_instance *const instance,
                                bmi26x_sensor_type sensor);
#endif

#else

/*!
 * Apply the configuration from the default registry
 *
 * @param[in]  this  The sensor handler
 */
void sns_bmi26x_registry_def_config(sns_sensor *const this);
#endif

#if BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
/*!
 * Apply the configuration from the default registry
 *
 * @param[in]  this  The sensor handler
 */
void sns_bmi26x_registry_def_config(sns_sensor *const this);
#endif

// </registry>
