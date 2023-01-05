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
 * @file sns_bmi26x_instance.h
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#pragma once

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_time.h"
#include "sns_com_port_types.h"

#include "sns_sensor_uid.h"

#include "sns_async_com_port.pb.h"
#include "sns_interrupt.pb.h"
#include "sns_island_service.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"
#include "sns_math_util.h"
#include "sns_registry_util.h"

#include "sns_bmi26x_dae_if.h"
#include "sns_bmi26x_config.h"
#include "sns_bmi26x_hal_comm.h"
#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_ver.h"


#if  BMI26X_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#endif


#if BMI26X_CONFIG_ENABLE_LOWG
#include "sns_free_fall.pb.h"
#endif

#define BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER             1

#define BMI26X_CONFIG_ENABLE_QUICK_REPORT_FAKE_FRAME      0
#define BMI26X_CONFIG_ENABLE_RESET_BEFORE_LOAD_CONFIG     0
#define BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES  1

#define BMI26X_CONFIG_ENABLE_DUMP_ALL_DEBUG_MSG           0
#define BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE             0
#define BMI26X_CONFIG_ENABLE_FAST_CONFIG_FIFO_IN_DAE_MODE 1

#define BMI26X_CONFIG_ENABLE_GROUP_DELAY              1

#define BMI26X_DELAY_DEINIT_IN_MS                         1000 //1.0s

//#define BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION 1

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
#include "sns_physical_sensor_oem_config.pb.h"
#endif

/** Forward Declaration of Instance API */
extern sns_sensor_instance_api bmi26x_sensor_instance_api;

/* improve power performance */
#define BMI26X_CONFIG_ENABLE_POWER_PERFORMANCE_IMPROVE      1

#define BMI26X_CONFIG_ENABLE_GYRO_SELF_TEST_ONESHOT         0
#define BMI26X_CONFIG_ENABLE_POWER_KEEP_ALIVE               0
#define BMI26X_EXPLICIT_CLEAR_FLUSH_PENDING_AFTER_FLUSHING  1

// imu performance
#define BMI26X_CONFIG_ENABLE_ENABLE_GYRO_HIGH_PERFORMANCE   1

/* debug message */
#define BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG                1

#define BMI26X_CONFIG_ENABLE_AVOID_DUPLIVATED_EVENT         1

// <Registry Sensors>

#ifndef BMI26X_CONFIG_ENABLE_FLUSH_PERIOD
#define BMI26X_CONFIG_ENABLE_FLUSH_PERIOD 1
#endif

#define BMI26X_CNT_HW_RES_START_EST_4_ALGO     16
#define BMI26X_THRESHOLD_FOR_CONTINUS_FIFO_BIG_JITTER   3

// Notice: normally, H/W detection should try again if the previous detection failure.
#ifndef BMI260_CONFIG_ENABLE_REDUCE_HW_DETECTION
#define BMI260_CONFIG_ENABLE_REDUCE_HW_DETECTION  0
#endif


#ifndef BMI26X_CONFIG_ENABLE_LOG_TS_BATCH
#define BMI26X_CONFIG_ENABLE_LOG_TS_BATCH 1
#endif

#ifndef BMI26X_CONFIG_ENABLE_LOG_TS_INDIVIDUAL_FRAME
#define BMI26X_CONFIG_ENABLE_LOG_TS_INDIVIDUAL_FRAME 1
#endif

#define BMI26X_CONFIG_ENABLE_LOG_CURRENT_CAL_BIAS      0

//@pedo
#ifndef BMI26X_CONFIG_ENABLE_STEP_DETECTOR
#define BMI26X_CONFIG_ENABLE_STEP_DETECTOR       0
#endif

#ifndef BMI26X_CONFIG_ENABLE_MD_TEST
#define BMI26X_CONFIG_ENABLE_MD_TEST    0
#endif

#ifndef BMI26X_CONFIG_ENABLE_DEBUG_TEST
#define BMI26X_CONFIG_ENABLE_DEBUG_TEST 0
#endif

#define BMI26X_CONFIG_ENABLE_WORKAROUND_4_SPI_SYNC_READ_LONG_DELAY   0
#define BMI26X_CONFIG_ENABLE_TS_IMPROVE                              1
#define BMI26X_CONFIG_IGNORE_TS_JITTER                               0

#define BMI26X_CONFIG_TS_IMPROVE_FRAME_COUNT_OVER_REFERENCE_SAMPLE   64
#define BMI26X_CONFIG_TS_IMPROVE_EST_ACURACY                         98

#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
#define BMI16X_CONFIG_TS_ALLOW_BIG_TOLERANCE                         1
#endif
#define BMI26X_TS_ALGO_UPDATE_FREQUENCY_OVER_TIME                    500


#define BMI26X_CONFIG_DFT_LOG_LEVEL SNS_MED



#define BMI26X_CONFIG_MAG_LOWEST_ODR            1
#define BMI26X_CONFIG_MAG_FASTEST_ODR           100


#define BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW_FILT  1
#define BMI26X_CONFIG_ENABLE_ERROR_ON_HIGHODR           1


#define BMI26X_CONFIG_ENABLE_WAIT4CMD_IMPROVED       1
#define BMI26X_CONFIG_ENABLE_PROFILING_SYS_DELAY     0
#define BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION      0

#define BMI26X_CONFIG_FIFO_WMI_MISSING_ITVL_US_THRES    5000


#define BMI26X_CONFIG_COM_FLOW_CTRL_THRESH      2


#define BMI26X_CONFIG_FIFO_HEADROOM             10 * 13

#define BMI26X_CONFIG_ACC_RANGE_DFT             BMI26X_REGV_RANGE_ACC_PM8G
#define BMI26X_CONFIG_GYR_RANGE_DFT             BMI26X_REGV_RANGE_GYR_PM2000DPS

#define BMI26X_CONFIG_ACC_BWP                   BMI26X_REGV_ACC_BWP_NORM_AVG4
#define BMI26X_CONFIG_GYR_BWP                   BMI26X_REGV_GYR_BWP_NORM   //normal filter


#define BMI26X_CONFIG_SEE_ASYNC_COM_DELAY_US    3500    //w/ some buffer, 4500 - 1000
#define BMI26X_CONFIG_SEE_SPI_BYTE_XFER_WAIT_CYCLES 2       //based on DB820

#define BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY  1
#define BMI26X_CONFIG_SPI_BURST_READ_LEN_MAX    16


#define BMI26X_CONFIG_TS_ITVL_EST_COEF          0.8f
#define BMI26X_CONFIG_TS_ITVL_EST_HIGH_RES_COEF  0.92f
#define BMI26X_CONFIG_TS_ITVL_EST_1ST_COEF      0.8f


#define BMI26X_CONFIG_TS_DELAY_ADJ_I2C          1

#define BMI26X_DROP_FIRST_FRAMES_NUM_DURING_CALIB        5


#if  BMI26X_CONFIG_ENABLE_DEBUG

#if BMI26X_CONFIG_ENABLE_DUMP_ALL_DEBUG_MSG
#define BMI26X_SENSOR_LOG(LOG_LEVEL, sensor, arg...) { \
    if (NULL != sensor) { \
        if (SNS_##LOG_LEVEL >= BMI26X_CONFIG_DFT_LOG_LEVEL) { \
            SNS_PRINTF(ERROR, sensor, ##arg); \
        } \
    } \
}


#define BMI26X_INST_LOG(LOG_LEVEL, inst, arg...) { \
    if (NULL != inst) { \
        if (SNS_##LOG_LEVEL >= BMI26X_CONFIG_DFT_LOG_LEVEL) { \
            SNS_INST_PRINTF(ERROR, inst, ##arg); \
        } \
    } \
}


#else   //BMI26X_CONFIG_ENABLE_DUMP_ALL_DEBUG_MSG


#define BMI26X_SENSOR_LOG(LOG_LEVEL, sensor, arg...) { \
    if (NULL != sensor) { \
        if (SNS_##LOG_LEVEL >= BMI26X_CONFIG_DFT_LOG_LEVEL) { \
            SNS_PRINTF(LOG_LEVEL, sensor, ##arg); \
        } \
    } \
}


#define BMI26X_INST_LOG(LOG_LEVEL, inst, arg...) { \
    if (NULL != inst) { \
        if (SNS_##LOG_LEVEL >= BMI26X_CONFIG_DFT_LOG_LEVEL) { \
            SNS_INST_PRINTF(LOG_LEVEL, inst, ##arg); \
        } \
    } \
}

// @BMI26X_CONFIG_ENABLE_DUMP_ALL_DEBUG_MSG
#endif

#else
#define BMI26X_SENSOR_LOG(LOG_LEVEL, sensor, arg...)  UNUSED_VAR(sensor)
#define BMI26X_INST_LOG(LOG_LEVEL, inst, arg...)  UNUSED_VAR(inst)

#endif    //BMI26X_CONFIG_ENABLE_DEBUG

#define BMI26X_FTI

#define BMI26X_NUM_AXES                         3



#define BMI26X_INST_CHECK_RC(inst, rc) { \
            if (rc != SNS_RC_SUCCESS) {         \
                BMI26X_INST_LOG(ERROR, inst, "ERROR!!! on ret:%d", rc); \
            }  \
}


typedef enum {
    BMI26X_SENSOR_NONE   = 0x0,
    BMI26X_ACCEL         = 0x1,
    BMI26X_GYRO          = 0x2,
    BMI26X_MAG           = 0x4, //this is also compatible with the FIFO header bit position
    BMI26X_MOTION_DETECT = 0x8,
    BMI26X_SENSOR_TEMP   = 0x10,
    BMI26X_PEDO          = 0x20,
    BMI26X_OIS           = 0x40,
    BMI26X_FREE_FALL     = 0x80,
    BMI26X_DOUBLE_TAP    = 0x100
} bmi26x_sensor_type;


typedef enum {
    BMI26X_CONFIG_IDLE,            /** not configuring */
    BMI26X_CONFIG_POWERING_DOWN,   /** cleaning up when no clients left */
    BMI26X_CONFIG_PENDING_ON_SENSOR_GOTO_NORMAL_MODE,   /** waiting for power up */
    BMI26X_CONFIG_STOPPING_STREAM, /** stream stop initiated, waiting for completion */
    BMI26X_CONFIG_FLUSHING_HW,     /** FIFO flush initiated, waiting for completion */
    BMI26X_CONFIG_UPDATING_HW,      /** updating sensor HW, when done goes back to IDLE */
    BMI26X_CONFIG_PENDING_DEINIT,   /** pending deinit instance and related resource */
    BMI26X_CONFIG_READY_TO_DEINIT   /** ready to deinit instance and related resource */
} bmi26x_config_step_t;


typedef enum {
    BMI26X_FLUSH_TO_BE_DONE,           // 0
    BMI26X_FLUSH_DONE_CONFIGURING,     // 1
    BMI26X_FLUSH_DONE_ACC_GYRO_DISABLED,  // 2
    BMI26X_FLUSH_DONE_NOT_ACCEL_GYRO,  // 2
    BMI26X_FLUSH_DONE_NOT_FIFO,        // 3
    BMI26X_FLUSH_DONE_FIFO_EMPTY,      // 4
    BMI26X_FLUSH_DONE_AFTER_DATA,      // 5
} bmi26x_flush_done_reason_t;


/*!
 * Definition for self test information
 */
typedef struct bmi26x_self_test_info {
    uint32_t                    test_type       : 4; //of type sns_physical_sensor_test_type ;

    uint32_t                    sensor          : 8; //of type bmi26x_sensor_type
    uint32_t                    test_client_present: 1; //of type bool
} bmi26x_self_test_info_t;

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
typedef struct bmi26x_self_test_info_bias {
    sns_physical_sensor_oem_config_type config_type;
    bool            test_client_present;
    float           offset_x;
    float           offset_y;
    float           offset_z;
} bmi26x_self_test_info_bias_t;
#endif


typedef enum {
    BMI26X_FAC_TEST_NO_ERROR,
    BMI26X_FAC_TEST_HIGH_BIAS,
    BMI26X_FAC_TEST_DEV_NOT_STATIONARY,
    BMI26X_FAC_TEST_ZERO_VARIANCE,
    BMI26X_FAC_TEST_DEV_BUSY,
    BMI26X_FAC_TEST_DEV_FATAL_ERROR
} bmi26x_test_err_code_t;

/*!
 * Definition for factory test information
 */
typedef struct bmi26x_factory_test_info {
    float variance_threshold;        /** stationary detect variance threshold */
    float variance[TRIAXIS_NUM];          /** variance */
    float sample_square_sum[TRIAXIS_NUM]; /** sum of square of sample data */
    float sample_sum[TRIAXIS_NUM];        /** sum of sample data */
    float bias_thresholds[TRIAXIS_NUM];

    uint32_t  num_samples       : 20;            /** number of samples acquired */
    uint32_t  at_rest           : 1;
    uint32_t  fac_test_sensor   : 6; //type bmi26x_sensor_type
} bmi26x_factory_test_info_t;


typedef float bmi26x_sstvt_t;
typedef bmi26x_sstvt_t bmi26x_gyro_sstvt;
typedef float bmi26x_real_t;

/*!
 *  Definition for COM port information
 */
typedef struct bmi26x_com_port_info {
    sns_com_port_config           com_config;
    sns_sync_com_port_handle      *port_handle;
} bmi26x_com_port_info_t;

/**
 * Definition for Range attribute
 */
typedef struct range_attr {
    float min;
    float max;
} range_attr_t;


typedef enum BMI26X_FIFO_READ_CTX_TYPE {
    BMI26X_FIFO_READ_CTX_TYPE_WMI,
    BMI26X_FIFO_READ_CTX_TYPE_FLUSH,
    BMI26X_FIFO_READ_CTX_TYPE_DAE
#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    ,
    BMI26X_FIFO_READ_CTX_TYPE_HB
#endif
} bmi26x_fifo_read_ctx_type_t;

typedef enum BMI26X_INT_CTX_TYPE {
    BMI26X_INT_TRIGGER_SOURCE_DAE,
    BMI26X_INT_TRIGGER_SOURCE_SEE,
    BMI26X_INT_TRIGGER_SOURCE_UNKNOWN,
} bmi26x_int_trigger_source_t;


typedef enum BMI26X_FIFO_FLUSH_TRIGGER {
    BMI26X_FIFO_FLUSH_TRIGGER_CLIENTS = 1,
    BMI26X_FIFO_FLUSH_TRIGGER_HW_CHANGE = (1 << 1),
} bmi26x_fifo_flush_trigger_t;


typedef enum BMI26X_FIFO_FLUSH_DONE_CTX {
    BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_REQ = 1,
    BMI26X_FIFO_FLUSH_DONE_CTX_HW_CHANGE = (1 << 1),
    BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW = (1 << 2),
    BMI26X_FIFO_FLUSH_DONE_CTX_DRDY = (1 << 3),
    BMI26X_FIFO_FLUSH_DONE_DAE      = (1 << 4),
} bmi26x_fifo_flush_ctx_t;

typedef enum BMI26X_FIFO_FLUSH_REASON {
    BMI26X_FIFO_FLUSH_SENSOR_DISABLE      = 1,
    BMI26X_FIFO_FLUSH_ODR_CHANGE_ON_FLY_GO_LOWER   = (1 << 1),
    BMI26X_FIFO_FLUSH_ODR_CHANGE_ON_FLY_GO_HIGHER  = (1 << 2),
    BMI26X_FIFO_FLUSH_RANGE_CHANGE        = (1 << 3),
} bmi26x_fifo_flush_reason_t;


/*!
 * Definition for FIFO read context
 */
typedef struct bmi26x_fifo_read_ctx {
    uint8_t             ctx_type                : 4;
    uint8_t             sync_read               : 2;
} bmi26x_fifo_read_ctx_t;




/*!
 * contains fifo reading req information
 * either interrupt or flush
*/
typedef struct bmi26x_fifo_req {
    sns_time          interrupt_ts;
    sns_time          cur_time;
    //s4s info
    sns_time          last_sync_ts;
    sns_time          ideal_sync_interval;
    uint16_t          t_ph;
    uint32_t          interrupt_fired  : 1;
    uint32_t          recheck_int      : 1;
    uint32_t          flush_req        : 1;

} bmi26x_fifo_req_t;


/*!
 * contains fifo time stamp context info
*/
typedef struct bmi26x_fifo_ts_context {
    bmi26x_regv_odr_t  regv_last_odr_acc;
    bmi26x_regv_odr_t  regv_last_odr_gyr;

    sns_time           ts_last_ff_batch_acc;
    sns_time           ts_last_ff_batch_gyr;

    sns_time           ts_ff_new_1st_batch_acc;
    sns_time           ts_ff_old_end_batch_acc;

    sns_time           ts_ff_new_1st_batch_gyr;
    sns_time           ts_ff_old_end_batch_gyr;

    uint32_t           ff_cnt_compensate_acc;
    uint32_t           ff_cnt_compensate_gyr;

} bmi26x_fifo_ts_tx_t;
typedef struct bmi26x_fifo_confg_chx_ctx {
    uint32_t             cfg_range_changed    :1;
    uint32_t             cfg_odr_changed_on_fly  :1;

    uint32_t             cfg_odr_changed_on_req :16;
    uint32_t             cfg_odr_changed_on_resp:16;
} bmi26x_fifo_confg_chx_ctx_t;

/*!
 * FIFO time information
 */
typedef struct bmi26x_fifo_time_info {
    uint32_t             fc_accum_curr_acc;

    uint32_t             fc_accum_curr_gyr;

    sns_time            ts_end_frame_last_batch_acc;
    sns_time            ts_end_frame_last_batch_gyr;

    sns_time            ts_end_frame_this_batch_acc;
    sns_time            ts_end_frame_this_batch_gyr;

    sns_time            ts_1st_frame_this_batch_acc;
    sns_time            ts_1st_frame_this_batch_gyr;

    bmi26x_real_t       itvl_this_batch_acc;
    bmi26x_real_t       itvl_this_batch_gyr;
    //TODOMAG

    uint32_t            avail_ts_last_batch     : 3;
    uint32_t            avail_ts_this_batch     : 3;

    //uint32_t            frm_header_at_irq       :3;
    //this will be the frm_cnt for one of the sensors in frm_header_at_irq;
    //int32_t             frm_cnt_accum_at_irq;
    //
    uint32_t            ff_ts_dev_delay         : 2;

    //uint32_t            avail_ts_ref_masters    :1;

    uint32_t            boost_read_size         : 1;

    uint32_t            ts_dev_ff_last          : 24;

#if BMI26X_CONFIG_ENABLE_DAE
    uint32_t            interrupt_fired         : 1;
    uint32_t            t_ph                    : 16;
#endif

    int32_t            fc_accum_masters;
    int32_t            fc_masters_last_dev_ff;


    sns_time            ts_ref_masters;

    //TODOMAG
} bmi26x_fifo_time_info_t;

/*!
 *  HW FIFO information
 */
typedef struct bmi26x_fifo_info {
    /** fifo cur rate index */
    bmi26x_regv_odr_t   ff_master_odr_req;
    uint16_t            ff_wml_bytes_req;

    bmi26x_regv_odr_t   ff_master_odr_curr;
    uint16_t            ff_wml_bytes_curr;

    bmi26x_real_t       ff_itvl_ticks_est_masters;
    bmi26x_real_t       ff_itvl_ticks_est_acc;
    bmi26x_real_t       ff_itvl_ticks_est_gyr;


    /** Determines which Sensor data to publish. Uses
     *  bmi26x_sensor_type as bit mask. */
    uint32_t            publish_sensors         : 10;

    uint32_t            ff_sensors_to_invalidate: 3;
    uint32_t            ff_sensors_flush_done_before_invalid: 3;
    uint32_t            ff_sensors_en_req       : 3;
    /** bits for sensors enabled in FIFO. Uses bmi26x_sensor_type as bit mask
     *  to determine which FIFO Sensors are enabled */
    uint32_t            ff_sensors_en_curr      : 3;


    uint32_t            ff_master_sensors_curr   : 3;


    uint32_t            ff_suggested_bytes2read : 10;

    uint32_t            ff_read_ctx_type        : 3;

    uint32_t            ff_master_sensors_ready : 3;
    uint32_t            ff_all_synced           : 1;

    uint32_t            ff_flush_trigger        : 3;

    uint32_t            ff_flush_in_proc        : 1;
    uint32_t            ff_flush_pending_on_sensors: 2;

    uint32_t            ff_wmi_missing_possible : 1;
    uint32_t            ff_int_masked           : 1;
    uint32_t            ff_wml_int_handled      : 1;
    uint32_t            ff_need_stab_cnt_due_to_odr_acc  : 6;
    uint32_t            ff_need_stab_cnt_due_to_odr_gyr  : 6;


#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
    int32_t             devi_est_irq;
#endif

    bmi26x_fifo_time_info_t ff_tm_info;

    /*contains info before sending ascp req*/
    bmi26x_fifo_req_t   th_info;

    /*contains info to use after ascp returns */
    bmi26x_fifo_req_t   bh_info;
} bmi26x_fifo_info_t;

struct bmi26x_state;

/*!
 * Structure definition for Accelemetor sensor information
 */
typedef struct bmi26x_accel_info {
    struct bmi26x_state     *sstate;

    float                   sample_rate_req;
    float                   report_rate_req;

    float                   backup_before_fac_sample_rate_req;
    float                   backup_before_fac_report_rate_req;


    bmi26x_fifo_confg_chx_ctx_t  ff_cfg_ctx;

    // flush
    sns_time                flush_period_ticks; //NU

    bmi26x_regv_odr_t       odr_req;
    uint16_t                ff_wml_req;

    bmi26x_regv_acc_range_t range_req;

    bmi26x_regv_odr_t       odr_curr;


    int32_t                 sstvt_curr;
    bmi26x_regv_acc_range_t range_curr;

    uint32_t                filter_delay_in_tick;

    uint32_t                ff_wml_curr             : 16;
    uint32_t                gated_client_present    : 1;
    uint32_t                client_present          : 1;
    uint32_t                max_batching_available  : 1;

    uint32_t                in_lpm                  : 1;

    uint32_t                batch_flush_flag        : 2;
    //uint32_t                resolution_idx          :4;


    bmi26x_self_test_info_t   test_info;
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    bmi26x_self_test_info_bias_t	test_info_bias;
#endif

} bmi26x_accel_info_t;


/*!
 * Structure definition for Gyro sensor information
 */
typedef struct bmi26x_gyro_info {
    struct bmi26x_state     *sstate;
    float                   sample_rate_req;
    float                   report_rate_req;

    float                   backup_before_fac_sample_rate_req;
    float                   backup_before_fac_report_rate_req;

    bmi26x_fifo_confg_chx_ctx_t  ff_cfg_ctx;

    // flush
    sns_time                flush_period_ticks; //NU
    bmi26x_regv_odr_t       odr_req;
    uint16_t                ff_wml_req;

    bmi26x_regv_gyr_range_t range_req;
    //CHECK for redundency

    bmi26x_regv_odr_t       odr_curr;

    int32_t                 sstvt_curr;
    bmi26x_regv_gyr_range_t range_curr;

    uint32_t                filter_delay_in_tick;

    uint32_t                ff_wml_curr             : 16;
    uint32_t                client_present          : 1;

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
    uint32_t                downsample_sw_factor    : 4;
    uint32_t                downsample_sw_cnt       : 4;

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW_FILT
    int16_t                 downsample_sw_dcache[3];
#endif

#endif
    uint32_t               pwr_cfg_pending       : 1;
    uint32_t               batch_flush_flag      : 2;

    /* cross sense */
    int32_t                signed_gyr_cross_sense_coef;

    bmi26x_self_test_info_t   test_info;
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    bmi26x_self_test_info_bias_t   test_info_bias;
#endif
} bmi26x_gyro_info_t;

/*!
 * Structure definition for MAG sensor information
 */
typedef struct bmi26x_mag_info {
    struct bmi26x_state     *sstate;

    float                   sample_rate_req;
    float                   report_rate_req;

    bmi26x_regv_odr_t       odr_curr;
    uint16_t                ff_wml_req;



    uint32_t                client_present          : 1;
    bmi26x_self_test_info_t   test_info;
} bmi26x_mag_info_t;

#if BMI26X_CONFIG_ENABLE_PEDO

typedef enum BMI26X_PEDO_EVENT_CTX {
    BMI26X_PEDO_EVT_CTX_INT     = 1,
    BMI26X_PEDO_EVT_CTX_TIMEOUT = 1 << 1,
} bmi26x_pedo_event_ctx_t;

/*!
 * Structure definition for Pedometer sensor information
 */
typedef struct bmi26x_pedo_info {
    struct bmi26x_state     *sstate;

    sns_data_stream         *pedo_timer_data_stream;
    float                   sample_rate_req;
    float                   report_rate_req;

    sns_time                sampling_intvl;

    //bmi26x_regv_acc_range_t range;
    bmi26x_acc_range_t      range_idx_req;
    uint32_t                step_count_hw_last;
    uint32_t                step_count;

    uint32_t                enable_pedo_int         : 1;
    uint32_t                client_present          : 1;
    uint32_t                pedo_new_req            : 1;
#if BMI26X_CONFIG_ENABLE_PEDO_TIMER
    uint32_t                timer_is_active         : 1;
#else
    uint32_t                  pedo_int_is_active      : 1;
#endif
    uint32_t                 is_first                : 1;

    bmi26x_self_test_info_t   test_info;
    sns_registry_md_cfg     pedo_config;
} bmi26x_pedo_info_t;
#endif

/*!
 * Structure definition for Motion Detection sensor information
 */
typedef struct bmi26x_motion_detect_info {
    struct bmi26x_state     *sstate;
    bmi26x_acc_range_t      range_idx_req;

    uint32_t                enable_md_int           : 1;
    uint32_t                client_present          : 1;

    uint32_t                md_new_req              : 1;
    uint32_t                md_event_num          : 24;
    sns_motion_detect_event md_state;

    bmi26x_self_test_info_t test_info;
    sns_registry_md_cfg     md_config;
    sns_time                ts_latest_event_fired;
} bmi26x_motion_detect_info_t;

/*!
 * Structure definition for temperature sensor information
 */
typedef struct bmi26x_sensor_temp_info {
    struct bmi26x_state     *sstate;

    float                   sample_rate_req;
    float                   report_rate_req;

    sns_time                sampling_intvl_req;
    sns_time                sampling_intvl_curr;
    sns_time                ts_last_frame;

    uint64_t                max_requested_flush_ticks;

    uint32_t                timer_is_active         : 1;
    uint32_t                timer_itvl_changed      : 1;
    uint32_t                client_present          : 1;

    bmi26x_self_test_info_t   test_info;
} bmi26x_sensor_temp_info_t;

#if BMI26X_CONFIG_ENABLE_OIS
/*!
 * Structure definition for OIS sensor information
 */
typedef struct bmi26x_ois_sensor_info {
    struct bmi26x_state     *sstate;

    uint32_t                client_present          : 1;
    uint32_t                en                      : 1;
    uint32_t                ois_range               : 2;

    bmi26x_self_test_info_t   test_info;
} bmi26x_ois_sensor_info_t;
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
/*!
 * Structure definition for Lowg
 */
typedef struct bmi26x_free_fall_info {
    struct bmi26x_state     *sstate;

    uint32_t                enable_lowg_int         : 1;
    uint32_t                client_present          : 1;
    uint32_t                lowg_new_req            : 1;
    uint32_t                lowg_wait_for_fired_event : 1;
    uint32_t                lowg_event_num          : 24;

    bmi26x_self_test_info_t test_info;

    sns_free_fall_event     lowg_state;

    sns_time                ts_latest_event_fired;
} bmi26x_lowg_info_t;
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
/*!
 * Structure definition for Lowg
 */
typedef struct bmi26x_double_tap_info {
    struct bmi26x_state     *sstate;

    uint32_t                enable_dbtap_int            : 1;
    uint32_t                client_present              : 1;
    uint32_t                dbtap_new_req               : 1;
    uint32_t                dbtap_wait_for_fired_event  : 1;
    uint32_t                dbtap_event_num             : 24;

    bmi26x_self_test_info_t test_info;

    sns_double_tap_event    dbtap_state;

    sns_time                ts_latest_event_fired;
} bmi26x_double_tap_info_t;
#endif


/*!
 * Structure information for IRQ information
 */
typedef struct bmi26x_irq_info {
    sns_interrupt_req       irq_config;
    bool                    irq_ready;
    bool                    irq_registered;
} bmi26x_irq_info_t;

/*!
 * Structure information for COM port information
 */
typedef struct bmi26x_async_com_port_info {
    uint32_t              port_handle;
} bmi26x_async_com_port_info;


typedef union bmi26x_pmu_stat_reg {
    uint8_t reg;
} bmi26x_pmu_stat_reg_t;

typedef union bmi26x_int_en_fifo_drdy_flag {
    uint8_t flag;
} bmi26x_int_en_fifo_drdy_flag_t;

typedef union bmi26x_int_en_flag {
    struct {
        bmi26x_int_en_fifo_drdy_flag_t drdy;
        bmi26x_int_en_fifo_drdy_flag_t fifo;

        uint8_t  pedo           : 1;
        uint8_t  sigmot         : 1;
        uint8_t  md             : 1;
        uint8_t  pmu_trigger    : 1;
        uint8_t  lowg           : 1;
        uint8_t  dbtap          : 1;
    } bits;

    uint32_t flag;
} bmi26x_int_en_flag_t;


/*!
 * Register Interrupt Context Structure
 */
typedef struct bmi26x_reg_int_context {
    uint32_t        ts_dev;
    sns_time        ts_sys;

    uint8_t         event;

    uint8_t         int_status_0;
    uint8_t         int_status_1;

    uint16_t        temperature;
    uint16_t        fifo_len;

    union {
        struct {
            uint8_t ts              : 1;
            uint8_t event           : 1;
            uint8_t int_status_0    : 1;
            uint8_t int_status_1    : 1;
            uint8_t temperature     : 1;
            uint8_t fifo_len        : 1;
        } bits;

        uint8_t flags;
    } avail;
} bmi26x_reg_int_ctx_t;

/*!
 * Interrupt Status structure for Data Ready
 */
typedef union bmi26x_int_stat_drdy {
    struct {
        uint8_t acc : 1;
        uint8_t gyr : 1;
        uint8_t mag : 1;
    } bits;

    uint8_t flag;
} bmi26x_int_stat_drdy_t;

/*!
 * Interrupt Status structure mapped from sensor INT definition
 */
typedef union bmi26x_int_stat_flag {
    struct {
        bmi26x_int_stat_drdy_t          drdy_detail;
        uint8_t  drdy                   : 1;

        uint8_t  ff_wml                 : 1;
        uint8_t  ff_full                : 1;
        uint8_t  err                    : 1;
        uint8_t  md                     : 1;
        uint8_t  dbtap                  : 1;
        uint8_t  sigmot                 : 1;
        uint8_t  step                   : 1;
        uint8_t  pmu_trigger            : 1;
#if BMI26X_CONFIG_ENABLE_LOWG
        uint8_t  lowg                   : 1;
#endif
    } bits;

    uint32_t flag;
} bmi26x_int_stat_flag_t;


/*!
 * Structure definition for Any Motion Configuration
 */
typedef struct bmi26x_int_cfg_any_motion {
    uint32_t int_anym_th                : 11;
    uint32_t int_anym_dur               : 13;
} bmi26x_int_cfg_any_motion_t;

/*!
 * Command request structure definition
 */
struct bmi26x_cmd_req {
    uint32_t    regv               : 8;
    uint32_t    type               : 3;
    uint32_t    pending            : 1;
    uint32_t    exec_time_us       : 20;

    sns_time    ts_req;
};


/*!
 * Command Handler structure definition
 */
typedef struct {
    bool                    cmd_in_proc;
    sns_data_stream         *timer_cmd_stream;
    sns_time                ts_timeout_tk_expectation;
} bmi26x_cmd_handler_t;


typedef enum {
    BMI26X_CRT_NONE,                 // 0
    BMI26X_CRT_PREPARE_TO_START,     // 1
    BMI26X_CRT_TIMER_START,          // 2
    BMI26X_CRT_RUN_IN_PROGRESS,      // 2
    BMI26X_CRT_DONE,                 // 3
    BMI26X_CRT_ABORT,                // 4
} bmi26x_crt_state_t;

/*!
 * CRT timer structure definition
 */
typedef struct {
    sns_data_stream         *timer_crt_stream;
    bmi26x_crt_state_t      crt_state;
    sns_time                ts_timeout_tk_expectation;
    sns_time                ts_timeout_tk_itvl;
} bmi26x_crt_handler_t;




/*!
 * Strucure definition for System time and Sensor time pair
 */
typedef struct bmi26x_sys_dev_time_pair {
    sns_time                    ts_sys;
    uint32_t                    ts_dev      : 24;

    uint32_t                    avail_1st   : 1;
} bmi26x_sys_dev_time_pair_t;



/*!
 * Strucure definition for Sensor State Raw Information Log message
 */
typedef struct log_sensor_state_raw_info {
    sns_diag_service *diag;
    /* Pointer to sensor instance */
    sns_sensor_instance *instance;
    /* Pointer to sensor UID*/
    struct sns_sensor_uid *sensor_uid;

    /* Size of a single encoded sample */
    size_t encoded_sample_size;
    /* Pointer to log*/
    void *log;
    /* Size of allocated space for log*/
    uint32_t log_size;
    /* Number of actual bytes written*/
    uint32_t bytes_written;
    /* Number of batch samples written*/
    uint32_t log_sample_cnt;

    uint32_t batch_sample_cnt;
} log_sensor_state_raw_info_t;


/*!
 * Structure for Unencoded Batching Sample
 */
typedef struct {
    /* Batch Sample type as defined in sns_diag.pb.h */
#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    sns_diag_batch_sample_type sample_type;
#endif
    /* Timestamp of the sensor state data sample */
    sns_time timestamp;
    /*Raw sensor state data sample*/
    float sample[BMI26X_NUM_AXES];
    /* Data status.*/
    sns_std_sensor_sample_status status;
} bmi26x_batch_sample_t;


/*!
 * Structure definition for FIFO Sample Report Context
 */
typedef struct bmi26x_fifo_sample_report_ctx {
    log_sensor_state_raw_info_t   *log_state_info_acc;
    log_sensor_state_raw_info_t   *log_state_info_gyr;
} bmi26x_fifo_sample_report_ctx_t;


/*!
 * Structure definition for Registry Configuration
 */
typedef struct sns_bmi26x_registry_cfg {
    bmi26x_sensor_type    sensor_type;
    matrix3               fac_cal_corr_mat;
    float                 fac_cal_bias[3];
    uint32_t              version;
    uint8_t               res_idx;     /* resolution index */
} sns_bmi26x_registry_cfg_t;


enum BMI26X_INT_CHECK_TRIGGER {
    BMI26X_INT_CHECK_TRIGGER_IRQ,
    BMI26X_INT_CHECK_TRIGGER_POST_FIFO_READ,
};

/*!
 * Struture type defnition for Interrupt Context Checking
 */
typedef struct bmi26x_int_check_ctx {
    uint32_t                    int_check_trigger   : 3;
    uint32_t                    req_ts_hw           : 1;

    sns_time                    timestamp;
} bmi26x_int_check_ctx_t;

/*!
 * Struture type defnition for Interrupt Context Checking in dae mode
 */
typedef struct bmi26x_int_check_ctx_from_dae {
    uint8_t const *data_ptr;
    uint32_t                    int_check_trigger   : 3;
    uint32_t                    acc_gyro_flag       : 3;

    sns_time                    timestamp;
} bmi26x_int_check_ctx_from_dae_t;

#if BMI26X_CONFIG_ENABLE_PEDO
typedef union bmi26x_config_4_pedo {
    struct  {
        // mini_buffer
        uint16_t  step_buffer_size;
        /*! Water-mark level */
        uint16_t pedo_wml         : 10;
        /*! Reset counter */
        uint16_t en_reset_cnt     : 1;
        /*! Enable bits for enabling output into the register status bits
        for step-detector */
        uint16_t en_step_detector : 1;
        /*! Enable bits for enabling output into the register status bits
        for step-counter */
        uint16_t en_step_counter  : 1;
        uint16_t en_activity      : 1;
        uint16_t rsv_1            : 2;

        /*! word 2*/
        uint16_t out_conf_step_detector : 4;
        uint16_t out_conf_step_activity : 4;
        /*! 1 word 3*/
        uint16_t _barrier_1_;
        uint16_t page_reserved[4];
    } bmi26x_step_counter_config;
    uint16_t settings[8];
} bmi26x_config_4_pedo_t;

#endif

/*! configuration for motion detection */
typedef union bmi26x_cfg_md {
    struct {
        uint16_t duration      : 13;
        uint16_t select_x      : 1;
        uint16_t select_y      : 1;
        uint16_t select_z      : 1;

        uint16_t threshold      : 11;
        uint16_t out_conf       : 4;
        uint16_t en_md          : 1;
    } bmi26x_motion_detection_config;
    uint16_t settings[2];
} bmi26x_cfg_md_t;

/*!
 * BMI26X Register cache Structure
 */
typedef struct bmi26x_regv_cache {
    uint32_t     regv_fifo_config_0   :8;  // 0x46
    uint32_t     regv_fifo_config_1   :8;  // 0x47

    uint32_t     regv_acc_cfg         :8;
    uint32_t     regv_gyr_cfg         :8;

    uint32_t     cache_regv_int_map_1 :8;
    uint32_t     regv_pwr_ctrl        :8; // 0x7d
    uint32_t     regv_pwr_conf        :8; // 0x7c
    uint32_t     regv_reserved        :8; // XXX
} bmi26x_regv_cache_t;

#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER

#define BMI26X_TS_DEFAULT_ATTACK_MS               5000     //5s
#define BMI26X_TS_DEFAULT_LONG_ATTACK_MS          BMI26X_TS_DEFAULT_ATTACK_MS //5s
#define BMI26X_TS_DEFAULT_TIMEOUT_MS              1000     //1s
#define BMI26X_TS_DEFAULT_ATTACK_CNT_BEFORE_RESET 6        //6 times
#define BMI26X_MIN_FIFO_WML_FOR_HB_TIMEOUT        6

enum BMI160_HB_CHECK_TRIGGER {
    BMI160_HB_TIMER_NONE,
    BMI160_HB_TIMER_FLUSH,
    BMI160_HB_TIMER_NEED_RECONFIG_HW,
    BMI160_HB_TIMER_NEED_RESET_DEVICE,
};

typedef struct bmi26x_heart_beat_timer {
    sns_data_stream         *timer_heart_beat_data_stream;
    sns_time                ts_data_event;
    sns_time                heart_beat_timeout;
    uint32_t                timer_enable        :1;
    uint32_t                heart_attack_cnt    :8;
    uint32_t                odr_master_on_hb    :5;
} bmi26x_heart_beat_timer_t;

#endif




/*!
 * CRT state structure definition for present state
 */
typedef struct bmi26x_crt_gain_state{
    uint32_t regv_crt_params_x   :8; // 4 rega 0x78
    uint32_t regv_crt_params_y   :8; // 4 rega 0x79
    uint32_t regv_crt_params_z   :8; // 4 rega 0x7a
    uint32_t regv_gain_en         :1; // gain enable
} bmi26x_crt_gain_state_t;

/** Private state. */
typedef struct bmi26x_instance_state {
    sns_sensor_instance     *owner;

    //the creator
    struct bmi26x_state     *sstate_creator;

    /** accel HW config details*/
    bmi26x_accel_info_t     accel_info;

    /** gyro HW config details*/
    bmi26x_gyro_info_t      gyro_info;

#if BMI26X_CONFIG_ENABLE_PEDO
    /** pedometer info */
    bmi26x_pedo_info_t   pedo_info;
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    bmi26x_ois_sensor_info_t  ois_info;
    bmi26x_com_port_info_t    ois_com_port_info;
    sns_data_stream           *ois_async_com_port_data_stream;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    bmi26x_double_tap_info_t    dbtap_info;
#endif

#if BMI26X_CONFIG_ENABLE_MAG_IF
    bmi26x_mag_info_t       mag_info;
#endif

    /** motion detect info */
    bmi26x_motion_detect_info_t md_info;

    /** Sensor Temperature config details. */
    bmi26x_sensor_temp_info_t sensor_temp_info;


    //OPTIM3 /** COM port info */
    bmi26x_com_port_info_t    com_port_info;
#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
    sns_time                ts_irq_last;
#endif
    sns_time                ts_irq;       // irq timestamp

    /*preserve the timer suid to handle the timer for temperature*/
    //OPTIM3
    sns_sensor_uid          timer_suid;
    /** Data streams from dependencies. */
    sns_stream_service      *stream_mgr;
    sns_data_stream         *interrupt_data_stream;
    sns_data_stream         *timer_data_stream;
    sns_data_stream         *async_com_port_data_stream;

    bmi26x_cmd_handler_t    cmd_handler;

#if BMI26X_CONFIG_ENABLE_CRT
    bmi26x_crt_handler_t    crt_handler;
    bmi26x_crt_gain_state_t crt_gain_state;
#endif

#if  BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    bmi26x_heart_beat_timer_t   hb_cfg_info;    // heart beat configure info
#endif


    size_t                  encoded_imu_event_len;
    size_t                  encoded_sensor_temp_event_len;


    sns_sync_com_port_service *scp_service;
    sns_gpio_service        *gpio_service;

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    sns_diag_service        *diag_service;
    size_t                  log_interrupt_encoded_size;
    size_t                  log_raw_encoded_size;
#endif

    sns_time                ts_last_sbus_read_pre;
    sns_time                ts_last_sbus_write;

    int32_t                 async_com_read_request;
    int32_t                 async_com_read_response;

    bmi26x_pmu_stat_reg_t   pmu_stat;

    //resolution of HW timestamps
    bmi26x_real_t           ts_hw_res_ticks_per_bit;
    bmi26x_real_t           ts_hw_res_ticks_per_bit_ideal;
    bmi26x_real_t           xfer_time_per_byte_ticks;

    uint32_t                ticks_in_1ms;
#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
    uint32_t                ticks_delta_i;
    uint32_t                ticks_delta_p;
#endif

    /** fifo details*/
    bmi26x_fifo_info_t        fifo_info;

    bmi26x_fifo_ts_tx_t     last_ff_ts_ctx;

    /**--------DAE interface---------*/
#if BMI26X_CONFIG_ENABLE_DAE
    bmi26x_dae_if_info_t      dae_if;
#endif
    bmi26x_config_step_t      config_step;


    bmi26x_sys_dev_time_pair_t ts_pair_sys_dev;
    bmi26x_sys_dev_time_pair_t ts_pair_sys_dev_4_ts_res_est;

    bmi26x_int_en_flag_t    int_en_flags_req;
    bmi26x_int_en_flag_t    int_en_flags_curr;

    bmi26x_reg_int_ctx_t    reg_int_ctx;

    bmi26x_factory_test_info_t fac_test_info;

    /* pedo */
#if BMI26X_CONFIG_ENABLE_PEDO
    size_t                   encoded_pedo_count_len;
    bmi26x_config_4_pedo_t   pedo_cfg;
    uint8_t                  pedo_cfg_page_sync;
#endif

    bmi26x_regv_cache_t      regv_cache;

    bmi26x_cfg_md_t          md_cfg;

#if BMI26X_CONFIG_ENABLE_LOWG
    bmi26x_lowg_info_t      lowg_info;
#endif

    uint32_t                sbus_mon_single_byte_rw_off         : 1;
    uint32_t                sbus_in_normal_mode                 : 1;

    uint32_t                hw_mod_needed                       : 3;
    uint32_t                hw_config_pending                   : 1;

    //uint32_t                avail_1st_est_ts_hw_res             :1;
    uint32_t                new_self_test_request               : 1;

    uint32_t                fac_test_in_progress                : 1;

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    bool                    new_self_test_request_bias          : 1;
#endif

    uint32_t                cnt_hw_res_est                      : 10;
#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
#endif
    uint32_t                update_fac_cal_in_registry          : 1;

    uint32_t                sensors_col                         : 12;

    uint32_t                irq_ready                           : 1;
    uint32_t                irq_registered                      : 1;

    uint32_t                ff_flush_client_req                 : 1;

    uint32_t                bus_is_spi                          : 1;

    uint32_t                bus_cycle_time_ns                   : 16;


    uint32_t                feature_config_loaded               : 1;
    uint32_t                keep_pedo_in_fac_mode               : 1;

#if BMI26X_CONFIG_ENABLE_CRT
    uint32_t                pending_update_crt_in_registry      : 1;
#endif

    uint32_t                 cfg_load_success                    : 1;
    uint32_t                 cfg_current_pg_num                  : 3;
    uint32_t                 need_recfg_hw                       : 1;
    uint32_t                 pwr_state_present                   : 3;
    uint32_t                 pwr_state_4_bus                     : 1;
    uint32_t                 inst_inted                          : 1;
    uint32_t                 cfg_ver_minor                       : 8;
    uint32_t                 cfg_ver_major                       : 8;

    /**----------debug----------*/
} bmi26x_instance_state;

/*!
 * Request message payload defintion
 */
typedef struct bmi26x_req_payload {
    sns_sensor *req_ssensor;
} bmi26x_req_payload_t;


/*!
 * Assess overall requests on the instance
 *
 * @param[in] istate    bmi26x instance handler
 */
void bmi26x_inst_assess_overall_req(bmi26x_instance_state *istate);

/*!
 * Instance intialization interface
 *
 * @param[in] this      The instance handler created by framework
 * @param[in] sstate    sensor state the framework used to create the instance
 * @return  SNS_RC_SUCESS    on success
 *          others value when failure
 */
sns_rc bmi26x_inst_init(sns_sensor_instance *const this,
                        sns_sensor_state const *sstate);

/*!
  * Instance intialization interface
  *
  * @param[in] this      The instance handler created by framework
  * @return  SNS_RC_SUCESS    on success
  *          others value when failure`
 */
sns_rc bmi26x_inst_deinit(sns_sensor_instance *const this);

/*!
 * Set client request interface
 *
 * @param[in] this              The instance handler created by framework
 * @param[in] client request    The client request from the framework
 * @return  SNS_RC_SUCESS    on success
 *          others value when failure
 */
sns_rc bmi26x_inst_set_client_config(
    sns_sensor_instance     *const this,
    sns_request const       *client_request);

/*!
 * Run self test fcuntion
 *
 * @param[in] instance   The instance handler
 */
void bmi26x_run_self_test(sns_sensor_instance *instance);

/**
 * Processes results for factory test for accel and gyro.
 *
 * @param[in] instance   reference to instance
 */
void bmi26x_process_fac_test(sns_sensor_instance *instance);

/*!
 * Send factory calibration event
 *
 * @param[in] instance   The instance handler
 * @param[in] sstate     BMI26x sensor state
 */
void bmi26x_send_fac_cal_event(
    sns_sensor_instance *const  instance,
    const struct bmi26x_state   *sstate);

/*!
 * Reset the factory calibration data
 *
 * @param[in] instance                 The instance handler
 * @param[in] sstate                   BMI26x sensor state
 * @param[in] update_version           Update version flag
 */
void bmi26x_reset_fac_cal_data(
    sns_sensor_instance *const  instance,
    struct bmi26x_state         *sstate,
    bool                        update_version);

/*!
 * The Function point definition for the GPIO read function
 *
 * @param[in] gpio            The GPIO number
 * @param[in] is_chip_pin     Is the CHIP pin
 * @param[in] state           The GPIO state
 * @return   SNS_RC_SUCCESS  on success
 *           others value when failure
 */
typedef sns_rc (*fp_read_gpio)(uint32_t gpio, bool is_chip_pin, sns_gpio_state *state);

/*!
 * The Function point definition for read and write comport
 *
 * @param[in] port_handle        The port handler
 * @param[in] vectors            Read/Write vector
 * @param[in] num_vectors        Vector Numbers
 * @param[in] save_write_time    Save write time or not
 * @param[out] xfer_bytes         The pratical transfer bytes
 * @return   SNS_RC_SUCCESS on success
 *           the others value if failure
 */
typedef sns_rc (*fp_sns_scp_register_rw)(sns_sync_com_port_handle *port_handle,
        sns_port_vector *vectors,
        int32_t num_vectors,
        bool save_write_time,
        uint32_t *xfer_bytes);

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
/*!
 * Run self test bias function
 *
 * @param[in]  instance           The instance handler
 * @return
 */
void bmi26x_run_self_test_bias(sns_sensor_instance *instance);
#endif

/*!
 * Get the master sensor state via the input sensor state
 *
 * @param[in] caller_sensor_state  The sensor state who call this function
 * @return       SNS_RC_SUCCESS   on success
 *               The others value when failure
 */
struct bmi26x_state* bmi26x_inst_get_master_sensor_state(
    sns_sensor_state const      *caller_sensor_state);
