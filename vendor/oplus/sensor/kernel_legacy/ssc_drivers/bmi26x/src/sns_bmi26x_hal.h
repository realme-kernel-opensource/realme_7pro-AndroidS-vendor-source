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
 * @file sns_bmi26x_hal.h
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#pragma once

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_gpio_service.h"
#include "float.h"

#include "sns_bmi26x_sensor_instance.h"
#include "sns_bmi26x_trace.h"
#include "sns_bmi26x_config.h"
#include "sns_bmi26x_hal_comm.h"

#if BMI26X_CONFIG_ENABLE_LOWG
#include "sns_free_fall.pb.h"
#endif


extern bmi26x_instance_state *bmi26x_inst_singleton;

#define BMI26X_RETRY_TIMES_ON_BUS_ERROR              3

//crt
#define BMI26X_CRT_PERFORM_ROUTINE_CRT_WIN_IN_MINUTES   600 //10 hour
//@crt


#define BMI26X_SPEC_SPI_DUMMY_READ_LEN              1

#define BMI26X_SPEC_STARTUP_TIME_SBUS_US            3800

#define BMI26X_CONFIG_ENABLE_POWER_RAIL_DELAY_OFF   1
#define BMI26X_US_LOWER_POWER_DELAY                 1000    // 1ms

#define BMI26X_CRT_TIMEOUT_TIMES_BY_MD_DURATION     3

#define BMI26X_DEFAULT_PAGE_NUM                      0

// @ois
#define BMI26X_OIS_RANGE_250                         0
#define BMI26X_OIS_RANGE_2000                        1
#define BMI26X_OIS_IF_DISABLE                        0
#define BMI26X_OIS_IF_EN                                  1

#define BMI26X_REGA_OIS_ACC_0                        0x0C
#define BMI26X_REGA_OIS_GYR_0                        0x12

// <4 ois>
#define BMI26X_CONFIG_ENABLE_OIS_LP_FILTER           1

#define BMI26X_ODR_1000              (1000.0)   /* just for test */
#define BMI26X_ODR_2000              (2000.0)   /* just for test */
// </ 4 ois>


// @pedo
#if BMI26X_CONFIG_ENABLE_PEDO
#define  BMI26X_PEDO_RANGE_MIN   0.0
#define  BMI26X_PEDO_RANGE_MAX   65535.0

#define BMI26X_PEDO_RESOLUTION                  1
#define BMI26X_CONFIG_PEDO_LOWEST_ODR           25
#define BMI26X_CONFIG_PEDO_FASTEST_ODR          25
#define BMI26X_CONFIG_PEDO_MAX_FREQ             BMI26X_CONFIG_PEDO_FASTEST_ODR

#endif

#define BMI26X_SPEC_STARTUP_TIME_MS             10
#define BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US      4    //2us base on spec
#define BMI26X_SPEC_IF_IDLE_TIME_SUSPEND_US     1000
#define BMI26X_SPEC_IF_SPI_SWITCH_TIME_US       200


#define BMI26X_MAX_TS_DLTA_MULTIPLE             2   // default is 1; change it when ts overlay issue happened

#define BMI26X_SPEC_SOFT_RESET_DELAY_TIME_MS    10

#define BMI26X_SPEC_ACC_STARTUP_TIME_US         3800  //2.5ms in datasheet
#define BMI26X_SPEC_GYR_STARTUP_TIME_US         80000
#define BMI26X_SPEC_MAG_STARTUP_TIME_US         500

#define BMI26X_SPEC_ACC_SELF_TEST_WAIT_TIME_MS  50
#define BMI26X_SPEC_GYR_SELF_TEST_WAIT_TIME_MS  50

#define BMI26X_SPEC_SENSORTIME_RES_US           39.0
#define BMI26X_SPEC_TS_LSB_OVERFLOW_VAL         0x1000000

#define BMI26X_SPEC_CONFIG_STREAM_PAGE_LEN      32
#define BMI26X_FEAT_SIZE_IN_BYTES               16


#define BMI26X_DATA_BUFFER_OFFSET_ACCEL         0
#define BMI26X_DATA_BUFFER_OFFSET_GYRO          6


/*!
 * register map
 */

#define BMI26X_REGA_USR_CHIP_ID         0x00
#define BMI26X_REGA_USR_REV_ID          0x01
#define BMI26X_REGA_USR_ERR_REG         0x02
#define BMI26X_REGA_USR_STATUS          0x03
#define BMI26X_REGA_USR_DATA_0          0x04
#define BMI26X_REGA_USR_DATA_7          0x0b
// data
#define BMI26X_REGA_USR_DATA_ACC_X_LSB  0x0c
#define BMI26X_REGA_USR_DATA_GYR_X_LSB  0x12
#define BMI26X_REGA_USR_DATA_19         0x17

#define BMI26X_REGA_USR_SENSORTIME_0    0x18
#define BMI26X_REGA_USR_SENSORTIME_2    0x1a

#define BMI26X_REGA_USR_EVENT           0x1b

#define BMI26X_REGA_USR_INT_STATUS_0    0x1c
#define BMI26X_REGA_USR_INT_STATUS_1    0x1d

#define BMI26X_REGA_STEP_COUNT_OUT_0    0x1e
#define BMI26X_REGA_STEP_COUNT_OUT_1    0x1f
#define BMI26X_REGA_USR_UC_GP_2         0x20
#define BMI26X_REGA_USR_UC_GP_3         0x21

#define BMI26X_REGA_USR_TEMPERATURE_0   0x22
#define BMI26X_REGA_USR_TEMPERATURE_1   0x23

#define BMI26X_REGA_USR_FIFO_LENGTH_0   0x24
#define BMI26X_REGA_USR_FIFO_LENGTH_1   0x25

#define BMI26X_REGA_USR_FIFO_DATA       0x26

#define BMI26X_REGA_USR_FEA_CONF_PAGE   0x2f
#define BMI26X_REGA_USR_FEA_CONF_REGS   0x30

#define BMI26X_REGA_USR_ACC_CONF        0x40
#define BMI26X_REGA_USR_ACC_RANGE       0x41
#define BMI26X_REGA_USR_GYR_CONF        0x42
#define BMI26X_REGA_USR_GYR_RANGE       0x43
#define BMI26X_REGA_USR_AUX_CONF        0x44

#define BMI26X_REGA_USR_FIFO_DOWNS      0x45

#define BMI26X_REGA_USR_FIFO_WTM_0      0x46
#define BMI26X_REGA_USR_FIFO_WTM_1      0x47

#define BMI26X_REGA_USR_FIFO_CONFIG_0   0x48
#define BMI26X_REGA_USR_FIFO_CONFIG_1   0x49


#define BMI26X_REGA_USR_SATURATION      0x4a

#define BMI26X_REGA_USR_AUX_DEV_ID      0x4b
#define BMI26X_REGA_USR_AUX_IF_CONF     0x4c
#define BMI26X_REGA_USR_AUX_RD_ADDR     0x4d
#define BMI26X_REGA_USR_AUX_WR_ADDR     0x4e
#define BMI26X_REGA_USR_AUX_WR_DATA     0x4f

#define BMI26X_REGA_USR_ERR_REG_MASK    0x52

#define BMI26X_REGA_USR_INT1_IO_CTRL    0x53
#define BMI26X_REGA_USR_INT2_IO_CTRL    0x54
#define BMI26X_REGA_USR_INT_LATCH       0x55
#define BMI26X_REGA_USR_INT1_MAP        0x56
#define BMI26X_REGA_USR_INT2_MAP        0x57
#define BMI26X_REGA_USR_INT_MAP_HW      0x58

#define BMI26X_REGA_USR_TITAN_CTRL      0x59

#define BMI26X_REGA_USR_CONF_STREAM_IDX_LSB 0x5b

#define BMI26X_REGA_USR_CONF_STREAM_IN  0x5e

#define BMI26X_REGA_USR_INTERNAL_ERROR  0x5f

#define BMI26X_REGA_USR_AUX_IF_TRIM     0x68
#define BMI26X_REGA_GYR_CRT_CONF_ADDR   0x69

#define BMI26X_REGA_USR_NVM_CTRL        0x6a
#define BMI26X_REGA_USR_IF_CONF         0x6b

#define BMI26X_REGA_USR_DRV             0x6c

#define BMI26X_REGA_USR_ACC_SELF_TEST   0x6d
#define BMI26X_REGA_GYR_SELF_TEST_AXIS  0x6e
#define BMI26X_REGA_USR_GYR_SELF_TEST   0x6f
#define BMI26X_REGA_USR_NV_CONF         0x70

#define BMI26X_REGA_GYR_GAIN_CONFIG     0x77
//#define BMI26X_REGA_CRT_PARAM_1         0x78

#define BMI26X_REGA_USR_PWR_CONF        0x7c
#define BMI26X_REGA_USR_PWR_CTRL        0x7d
#define BMI26X_REGA_USR_CMD             0x7e

#define BMI26X_REGA_EXT_MODE            0x7f

#define BMI26X_INT_STATUS_0_ANYM_INT    (1<<2)
#define BMI26X_INT_STATUS_0_D_TAP_INT   (1<<4)

#define BMI26X_REGV_CHIP_ID_MAJOR       0x20
#define BMI16X_REGV_CHIP_ID_MAJOR       0xd0

#define BMI26X_REGV_CMD_RUN_CRT         0x02
#define BMI26X_REGV_CMD_FIFO_FLUSH      0xb0
#define BMI26X_REGV_CMD_INT_RESET       0xb1
#define BMI26X_REGV_CMD_SOFT_RESET      0xb6

#define BMI26X_REGV_CMD_TGT_PAGE_USR   0
#define BMI26X_REGV_CMD_TGT_PAGE_COM   1
#define BMI26X_REGV_CMD_TGT_PAGE_ACC   2
#define BMI26X_REGV_CMD_TGT_PAGE_GYR   3


#define BMI26X_REGV_CMD_EXT_MODE_EN_B0  0x37
#define BMI26X_REGV_CMD_EXT_MODE_EN_B1  0x9a
#define BMI26X_REGV_CMD_EXT_MODE_EN_B2  0xc0


//FIFO
// @ph_param = 0x02
#define BMI26X_FIFO_FRAMES_CFG_INPUT_CONFIG_MASK 0x3F
#define BMI26X_FIFO_FRAMES_CFG_INPUT_CONFIG_ACC_MASK 0x03
#define BMI26X_FIFO_FRAMES_CFG_INPUT_CONFIG_GYR_MASK 0x0C
#define BMI26X_DATA_STREAMING_CFG_BW_MASK            0xA0
// </FIFO>



#define BMI26X_REGV_GYR_RANGE_PM2000DPS         0
#define BMI26X_REGV_GYR_RANGE_PM1000DPS         1
#define BMI26X_REGV_GYR_RANGE_PM500DPS          2
#define BMI26X_REGV_GYR_RANGE_PM250DPS          3
#define BMI26X_REGV_GYR_RANGE_PM125DPS          4

#define BMI26X_REGV_PMU_STAT_ACC_SUSPEND        0
#define BMI26X_REGV_PMU_STAT_ACC_NORMAL         1
#define BMI26X_REGV_PMU_STAT_ACC_LP1            2
#define BMI26X_REGV_PMU_STAT_ACC_LP2            3

#define BMI26X_REGV_PMU_STAT_GYR_SUSPEND        0
#define BMI26X_REGV_PMU_STAT_GYR_NORMAL         1
#define BMI26X_REGV_PMU_STAT_GYR_RESERVED       2
#define BMI26X_REGV_PMU_STAT_GYR_FAST_STARTUP   3

#define BMI26X_REGV_PMU_STAT_MAG_SUSPEND        0
#define BMI26X_REGV_PMU_STAT_MAG_NORMAL         1
#define BMI26X_REGV_PMU_STAT_MAG_LP1            2
#define BMI26X_REGV_PMU_STAT_MAG_LP2            3

#define BMI26X_REGV_STEP_CONF_0_NORMAL          0x15
#define BMI26X_REGV_STEP_CONF_1_NORMAL          0x03

#define BMI26X_REGV_STEP_CONF_0_SENSITIVE       0x2d
#define BMI26X_REGV_STEP_CONF_1_SENSITIVE       0x00

#define BMI26X_REGV_STEP_CONF_0_ROBUST          0x1d
#define BMI26X_REGV_STEP_CONF_1_ROBUST          0x07


#define BMI26X_REGV_INT_LATCH_NONE              0
#define BMI26X_REGV_INT_LATCH_313US             1
#define BMI26X_REGV_INT_LATCH_625US             2
#define BMI26X_REGV_INT_LATCH_1P25MS            3
#define BMI26X_REGV_INT_LATCH_2P5MS             4
#define BMI26X_REGV_INT_LATCH_5MS               5
#define BMI26X_REGV_INT_LATCH_10MS              6
#define BMI26X_REGV_INT_LATCH_20MS              7
#define BMI26X_REGV_INT_LATCH_40MS              8
#define BMI26X_REGV_INT_LATCH_80MS              9
#define BMI26X_REGV_INT_LATCH_160MS             10
#define BMI26X_REGV_INT_LATCH_320MS             11
#define BMI26X_REGV_INT_LATCH_640MS             12
#define BMI26X_REGV_INT_LATCH_1280S             13
#define BMI26X_REGV_INT_LATCH_2560S             14
#define BMI26X_REGV_INT_LATCH_PERM              15


// int bit in INT0
#define BMI26X_DOUBLE_TAP_BIT_POS_IN_INT_STATUS  3
// </int bit in INT0>


#define B0_SET  1
#define B1_SET  (1 << 1)
#define B2_SET  (1 << 2)
#define B3_SET  (1 << 3)
#define B4_SET  (1 << 4)
#define B5_SET  (1 << 5)
#define B6_SET  (1 << 6)
#define B7_SET  (1 << 7)


#define BST_GET_VAL_BIT(val, bit) (((val) >> (bit)) & 0x01)
#define BST_GET_VAL_BITBLOCK(val, start, end) (((val) >> (start)) & ((1 << (end - start + 1)) - 1))

#define BST_SET_VAL_BIT(val, bit)      (val | (1 << bit))
#define BST_CLR_VAL_BIT(val, bit)      (val & (~(1 << bit)))

#define BST_SET_VAL_BITBLOCK(val, start, end, val_bb) \
    ((val & \
       (~ ((uint32_t)(((1 << (end - start + 1)) - 1) << start))) \
      )\
      |\
      ((val_bb &\
        ((1 << (end - start + 1)) - 1)) << start)\
    )

#define BST_CEIL_P(f) ((f + 0.5f))
#define BST_CEIL_P_BUF(f, buf) ((f + buf))
//#define BST_IS_FLOAT_ZERO(f)    (((f) >= 0) && ((f) <= 0))
//#define BST_IS_FLOAT_ZERO(f)    (0 == (int)(1e3 * (f)))
#define BST_IS_FLOAT_ZERO(f)      (((f) >= -FLT_MIN) && ((f) <= FLT_MIN))

#define BST_ASSERT_POINT(inst) (!inst)

#define BMS_SCALE_S2US          1000000

#define BST_ARRAY_SIZE(array)   (sizeof(array) / sizeof(array[0]))
#define BST_MIN(a, b)           (((a) < (b)) ? (a) : (b))
#define BST_MAX(a, b)           (((a) > (b)) ? (a) : (b))
#define BST_MAX3(a, b, c)       (BST_MAX(BST_MAX((a), (b)), (c)))

#define BST_ABS(x)              (((x) > 0) ? (x) : (0 - (x)))

#define BST_IGNORE_WARNING_UNUSED_VAR(v)    ((void)v)

#define BST_ASSUME_TS_IS_64BIT

#ifndef likely
#define likely(x) x
#endif


// 4 page switch
#define BMI26X_PAGE_SWITCH_DELAY_IN_US                           6

// @motion
#define BMI26X_MOTION_DETECTION_DURATION_MIN_MS     (20.0)      //20ms
#define BMI26X_MOTION_DETECTION_DURATION_MAX_MS     (163000.0)  //163s
#define BMI26X_MOTION_DETECTION_THRESHOLD_MIN_G     (0.083)     //83mg
#define BMI26X_MOTION_DETECTION_THRESHOLD_MAX_G     (1.0)       //1.0g
#define BMI26X_MOTION_DETECTION_DURATION_BASE_MS    (20.0)
#define BMI26X_MOTION_DETECTION_REGV_DURATION       (0x1FFF)    //13 bits
#define BMI26X_MOTION_DETECTION_REGV_THRESHOLD      (0x7FF)     //11 bits

#if BMI26X_CONFIG_ENABLE_LOWG
// !@low-g
#define BMI26X_LOWG_THRESOHOD_DEFAULT_VAL_G         0.25f       //0.25g
#define BMI26X_LOWG_THRESOHOD_DEFAULT_VAL_LSB       512        //0.25g
#define BMI26X_LOWG_THRESHOLD_RESOLUTION            (0.00048828125) // 16(g)/2^15

#define BMI26X_LOWG_HYSTERESIS_DEFAULT_VAL_G        0.125      //0.125g
#define BMI26X_LOWG_HYSTERESIS_DEFAULT_VAL_LSB      256        //256 = 0.125g
#define BMI26X_LOWG_HYSTERESIS_RESOLUTION           (0.00048828125) // 2(g)/2^12 = 0.00048828125

// duration
#define BMI26X_LOWG_DURATION_DEFAULT_VAL_ELP        0.0f        //ms
#define BMI26X_LOWG_DURATION_DEFAULT_VAL_LSB        0          //lsb
#define BMI26X_LOWG_DURATION_RESOLUTION             20.0       //ms

// </@low-g>
#endif


#define BMI26X_CONFIG_MD_ACC_BWP                BMI26X_REGV_ACC_BWP_OSR4_AVG1
#define BMI26X_CONFIG_MD_ACC_ODR                BMI26X_REGV_ODR_50HZ
#define BMI26X_REGV_INT_LATCH_EN                1
#define BMI26X_REGV_INT_LATCH_DISABLE           0
#define BMI26X_CONFIG_INT_LATCH_REGV            BMI26X_REGV_INT_LATCH_EN


#ifndef BMI26X_DD_CHECK_RETVAL
#define BMI26X_DD_CHECK_RETVAL(val, val_success)\
    if (likely(val_success == val)) {       \
    } else {                                \
        val = val;                      \
        return val;                     \
    }
#endif


struct bst_sbus_spec {
    uint32_t clk_rate;

    uint32_t type       : 2;
    uint32_t start_delay_us : 30;
};


#define BMI26X_FF_DEPTH_BYTES      1024


#define BMI26X_FF_FRAME_LEN_TS  4
#define BMI26X_FF_DATA_LEN_ACC  6
#define BMI26X_FF_DATA_LEN_GYR  6
#define BMI26X_FF_DATA_LEN_IMU  6
#define BMI26X_FF_DATA_LEN_MAG  8

#define BMI26X_FF_FH_MODE_CTRL                  0x01
#define BMI26X_FF_FH_MODE_REGULAR               0x02
#define BMI26X_FF_FH_EMPTY                      0x80


#define BMI26X_FF_FH_MODE(frame_header_byte)    BST_GET_VAL_BITBLOCK(frame_header_byte, 6, 7)
#define BMI26X_FF_FH_PARAM(frame_header_byte)   BST_GET_VAL_BITBLOCK(frame_header_byte, 2, 5)
#define BMI26X_FF_FH_EXT(frame_header_byte)     BST_GET_VAL_BITBLOCK(frame_header_byte, 0, 1)

/** pmu */
#define BMI26X_PWR_CTRL_AUX_EN_BIT_POS          0
#define BMI26X_PWR_CTRL_GYR_EN_BIT_POS          1
#define BMI26X_PWR_CTRL_ACC_EN_BIT_POS          2
#define BMI26X_PWR_CTRL_TEMP_EN_BIT_POS         3

#if BMI26X_CONFIG_ENABLE_CRT
#define BMI26X_REGV_GYR_CRT_GAIN_EN_BIT_POS     2
#define BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS     2
#define BMI26X_REGV_GYR_CRT_GAIN_READY_BIT_POS  3
#define BMI26X_REGV_GYR_CRT_DOWNLOAD_READY_BIT_POS 3

#define BMI26X_MAX_TRY_NUM_4_CRT_PROCESS        3
#define BMI26X_MAX_TRY_NUM_4_CONFIGURE_DOWNLOAD 200

#define BMI26X_CRT_MAX_BURST_READ_WRITE_LEN     0x80/* 256 bytes */
#define BMI26X_CRT_OFFSET_IN_CONFIG_FILE        0x1800

#define BMI26X_CRT_GAIN_NUM                     3


/*!
 * Definition for feature input of user gain update
 */
typedef struct bmi26x_feature_in_user_gain_update {
    uint16_t    ratio_x  : 11;

    uint16_t    ratio_y  : 11;

    uint16_t    ratio_z  : 11;
    uint16_t    en_user_gain_update : 1;
} bmi26x_feature_in_user_gain_update_t;


/*! @name Structure to define gyroscope saturation status of user gain
*/
typedef struct bmi26x_gyr_user_gain_status {
    /*! Status in x-axis */
    uint8_t sat_x     : 1;
    /*! Status in y-axis */
    uint8_t sat_y     : 1;
    /*! Status in z-axis */
    uint8_t sat_z     : 1;

    /*! Status of CRT */
    uint8_t crt_status : 3;
    uint8_t reserved   : 2;
} bmi26x_gyr_user_gain_status_t;


enum {
    BMI26X_GYRO_TRIGGER_STATUS_NO_ERROR,
    BMI26X_GYRO_TRIGGER_STATUS_PRECON_ERROR,
    BMI26X_GYRO_TRIGGER_STATUS_DOWNLOAD_ERROR,
    BMI26X_GYRO_TRIGGER_STATUS_ABORT_ERROR,
};

#endif


typedef enum {
    HW_CONFIG_CTX_CLIENT,
    HW_CONFIG_CTX_HW_CHANGE,
    HW_CONFIG_CTX_ON_DEF_TIMEROUT,
    HW_CONFIG_CTX_ON_DEF_FLUSH_PENDING,
    HW_CONFIG_CTX_ON_MOTION_DETECTION,
    HW_CONFIG_CTX_BEFORE_FAC_START,      /* 5 */
    HW_CONFIG_CTX_AFTER_FAC,
    HW_CONFIG_CTX_ON_DAE_FLUSH_HW,
    HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS,
    HW_CONFIG_CTX_ON_DAE_PAUSE_SAMPLING,
    HW_CONFIG_CTX_ON_HEART_BEART_ATTACK,
} bmi26x_hw_cfg_ctx_t;


/*!
@brief Returns the current island mode state

@return
SNS_ISLAND_STATE_IN_ISLAND When system is in island mode
SNS_ISLAND_STATE_NOT_IN_ISLAND When system is not in island mode
SNS_ISLAND_STATE_ISLAND_DISABLED When island mode is disabled
*/
typedef enum {
    /* Island mode is disabled. It might be enabled later */
    SNS_ISLAND_STATE_ISLAND_DISABLED = 0,
    /* Island mode is available. */
    SNS_ISLAND_STATE_IN_ISLAND,
    /* Island mode is not available and will not be available in the future. */
    SNS_ISLAND_STATE_NOT_IN_ISLAND
} sns_island_state;


struct bmi26x_odr_regv_map {
    float       odr;
    uint8_t     regv;
};


#if BMI26X_CONFIG_ENABLE_CRT
/*!
 * crt gain value
 */
typedef struct bmi26x_gyr_crt_gain {
    uint8_t  gain_x;
    uint8_t  gain_y;
    uint8_t  gain_z;
} bmi26x_gyr_crt_gain_t;


typedef struct bmi26x_gyr_multiple_crt_cfg{
    uint32_t  crt_execution_win         : 16;    // multiple execution windows
    uint32_t  crt_repeate_on_error      : 1;     // repeat CRT when error happen in previous process
    uint32_t  version;
} bmi26x_gyr_multiple_crt_cfg_t;

typedef struct bmi26x_gyr_multiple_crt_state{
    uint32_t  need_repeat_crt          : 1;     // repeat CRT when error happen in previous process
    uint32_t  latest_repeat_crt_reason : 4;
    uint32_t  crt_repeat_times         : 16;
    sns_time  ts_crt_latest_passed;
} bmi26x_gyr_multiple_crt_state_t;

typedef enum {
    REPEAT_MULTIPLE_NONE,
    REPEAT_MULTIPLE_CRT_DUE_TO_CRT_FIFO_ERROR,
    REPEAT_MULTIPLE_CRT_DUE_TO_CRT_DOWNLOAD_ERROR,
    REPEAT_MULTIPLE_CRT_DUE_TO_CRT_ABORT_ERROR,
    REPEAT_MULTIPLE_CRT_UNKNOWN_ERROR
} bmi26x_reason_4_rep_multiple_crt_t;

typedef enum {
    CRT_TRIGGER_SOURCE_NONE,
    CRT_TRIGGER_SOURCE_FAC_TEST,
    CRT_TRIGGER_SOURCE_FIRST_TIME,
    CRT_TRIGGER_SOURCE_ROUTINE,
    CRT_TRIGGER_SOURCE_REPEATE_ON_ERROR,
} bmi26x_crt_trigger_source_t;


#endif

#if BMI26X_CONFIG_ENABLE_OIS
/*!
 * Ois function definition
 */
typedef struct bmi26x_gyr_ois {
    uint8_t  enable;
    uint8_t  range_idx;
    uint8_t  spi4;
} bmi26x_gyr_ois_t;
#endif

#define BMI26X_ACC_GYRO_DIMENSION        3
#define BMI26X_ACC_GYRO_ONE_FRAME_SIZE_IN_BYTES  6

/** fifo paramters */
#define BMI26X_FF_MAX_FRAMES_IMU            (BMI26X_FF_DEPTH_BYTES / (BMI26X_FF_DATA_LEN_IMU + 1))
#define BMI26X_FF_MAX_FRAMES_MAG            (BMI26X_FF_DEPTH_BYTES / (BMI26X_FF_DATA_LEN_MAG + 1))

/** Off to idle time */
#define BMI26X_OFF_TO_IDLE_MS      60  //ms

#if BMI26X_CONFIG_ENABLE_SELF_TEST_HW_SW

/*! @name For enable and disable */
#define BMI2XY_ENABLE             1
#define BMI2XY_DISABLE            0

#define BMI26X_CMD_RUN_GYRO_SELF_TEST            1

#define BMI26X_SENSOR_HW_TEST_ACC_RANGE_INDEX    3    //16g
#define BMI26X_SENSOR_HW_TEST_ACC_ODR_REGV       0x0C //16g


#define BMI26X_ST_ACC_X_SIG_MIN_DIFF  (16000)     //16g = 16000mg

#define BMI26X_ST_ACC_Y_SIG_MIN_DIFF  (-15000)   // -15g = -15000mg

#define BMI26X_ST_ACC_Z_SIG_MIN_DIFF  (10000)    //10g = 10 000mg

#endif

/*!
 * Status Register mapping definition for bmi26x
 */
typedef union bmi26x_status_reg {
    struct {
        uint8_t por_detected            : 1;
        uint8_t gyr_self_test_ok        : 1;
        uint8_t mag_man_op              : 1;
        uint8_t foc_rdy                 : 1;
        uint8_t nvm_rdy                 : 1;
        uint8_t drdy_mag                : 1;
        uint8_t drdy_gyr                : 1;
        uint8_t drdy_acc                : 1;
    } bits;
    uint8_t reg;
} bmi26x_status_reg_t;


/*!
 * Sensor Raw Data buffer definition
 */
struct bmi26x_sensor_data_raw_buf {
    uint8_t buf_a[6];

    uint8_t buf_g[6];

    uint8_t buf_m[8];

    uint8_t buf_ts[3];

    uint8_t buf_temp[2];

    uint8_t _reserved[2];


    uint8_t avail_ts   : 1;
};


/*!
 * Sensoe Data Request Definition
 */
typedef union bmi26x_sensor_data_req {
    struct {
        uint32_t m:  1;
        uint32_t g:  1;
        uint32_t a:  1;
        uint32_t ts: 1;
        uint32_t t:  1;
        uint32_t p:  1;

    } bits;

    uint32_t req;
} bmi26x_sensor_data_req_t;


enum {
    BMI26X_FF_FRAME_ERR_NONE = 0,
    BMI26X_FF_FRAME_ERR_UNKNOWN,
    BMI26X_FF_FRAME_ERR_PARTIAL_READ,
};

typedef enum BMI26X_CMD_TYPE {
    BMI26X_CMD_TYPE_ACC,
    BMI26X_CMD_TYPE_GYR,
    BMI26X_CMD_TYPE_MAG,
    BMI26X_CMD_TYPE_MISC,
    BMI26X_CMD_TYPE_UNKNOWN,
} bmi26x_cmd_type_t;

/*!
 * The definition for FIFO parse Result
 */
typedef union bmi26x_fifo_parse_result {
    struct {
        //NU
        uint32_t ff_err                 : 1;
        //NU
        uint32_t ff_err_code            : 3;

        uint32_t ff_overflow            : 1;

        uint32_t ff_cache_full_agm      : 3;


        uint32_t ff_avail_ts            : 1;
        uint32_t ff_avail_ts_header     : 1;
        uint32_t ff_avail_end_frame     : 1;

        uint32_t offset_ts_header       : 13;   //bmi26x
        uint32_t ff_batch_empty         : 1; //empty fifo batch

    } bits;

    uint32_t rslt;
} bmi26x_fifo_parse_result_t;


/*!
 *
 */
typedef struct bmi26x_fifo_parse_out_desc {
    bmi26x_fifo_parse_result_t  ff_parse_rslt;

    uint32_t                    ts_dev_ff               : 24;

    uint32_t                    ff_frm_header_at_irq    : 3;
    uint32_t                    ff_frm_header_before_ts : 3;

    //in this batch only
    uint32_t                    fc_masters_this_batch   : 8;

    uint32_t                    fc_at_irq_acc           : 8;
    uint32_t                    fc_at_irq_gyr           : 8;
    uint32_t                    fc_at_irq_mag           : 8;
    uint32_t                    fc_at_irq_masters       : 8;

    uint32_t                    fc_this_batch_acc       : 8;
    uint32_t                    fc_this_batch_gyr       : 8;
    uint32_t                    fc_this_batch_mag       : 8;

    uint32_t                    fc_masters_mark_acc     : 8;
    uint32_t                    fc_masters_mark_gyr     : 8;
    uint32_t                    fc_masters_mark_mag     : 8;
} bmi26x_fifo_parse_out_desc_t;

typedef struct bmi26x_fifo_parse_ctx {
    const uint8_t               *ff_buf;
    uint16_t                    ff_buf_len;

    //offset of member @ff_buf relative to the whole buffer read out in this batch
    //uint16_t                    offset_buffer;

    uint32_t                    ff_parse_flag;

    uint32_t                    ff_proc_len_left;


    void                        *priv_data;
} bmi26x_fifo_parse_ctx_t;

#define BMI26X_FIFO_PARSE_FLAG_DRYRUN   (1)

typedef union bmi26x_fifo_pkg_stat {
    struct {
        //NU
        uint32_t ff_err         : 1;
        //NU
        uint32_t ff_err_code    : 3;

        uint32_t ff_overflow    : 1;

        uint32_t ff_rdy_acc     : 1;
        uint32_t ff_rdy_gyr     : 1;
        uint32_t ff_rdy_mag     : 1;
    } bits;

    uint32_t stat;
} bmi26x_fifo_pkg_stat_t;

typedef union bmi26x_hw_err_stat {
    /*! bmi26x sensor error status */
    struct {
        uint8_t fatal_err:      1;
        uint8_t err_code:       4;
        uint8_t i2c_fail_err:   1;
        uint8_t drop_cmd_err:   1;
        uint8_t mag_drdy_err:   1;
    } bits;

    uint8_t         regv;
} bmi26x_hw_err_stat_t;




/******************* Function Declarations ***********************************/

/*!
 * Exit from island mode
 * @param this   instance handler
 */
void bmi26x_hal_inst_exit_island(sns_sensor_instance *this);

/**
 * Resets the Sensor HW. Also calls
 * bmi26x_device_set_default_state()
 *
 * @param[in] this          Instance state
 * @param[in] sensor        bit mask for sensors to reset
 *
 * @return sns_rc           SNS_RC_SUCCESS on success
 *                          The others value when failure
 */
sns_rc bmi26x_hal_reset_device(
    sns_sensor_instance     *this,
    uint8_t sensor);

/**
 * Prepare sensor state
 * bmi26x_hal_prepare_sensor_state()
 *
 * @param[in] this          Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc bmi26x_hal_reset_sensor_state(
    sns_sensor_instance     *this);

/*!
 * Reset device wrapper Operation
 * reset the device state
 * prepare the interface
 * reset the internal status
 *
 * @param[in] int   the instance handler
 * @param[out] SNS_RC_SUCCESS      on success
 *                  other values                  failure
 */
sns_rc bmi26x_hal_reset_device_wrapper(sns_sensor_instance *inst);

/**
 * Prepare sensor state
 * bmi26x_hal_load_sensor_cfg()
 *
 * @param[in] this          Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc bmi26x_hal_load_sensor_cfg(
    sns_sensor_instance     *this);

/**
 * Loads default values in config registers.
 *
 * @param[in] scp_service   handle to synch COM port service
 * @param[in] port_handle   handle to synch COM port
 * @param[in] sensor        bit mask for sensors to handle
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc bmi26x_device_set_default_state(sns_sync_com_port_service *scp_service,
                                       sns_sync_com_port_handle *port_handle,
                                       bmi26x_sensor_type sensor);

/**
 * Enables interrupt for FIFO sensors.
 *
 * @param[in] state         Instance state
 * @param[in] sensors       sensor bit mask to enable
 *
 * @return none
 */
void bmi26x_hal_enable_fifo_intr(bmi26x_instance_state *state);

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[in] state         Instance state
 * @param[out] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc bmi26x_hal_get_who_am_i(
    sns_sync_com_port_service   *scp_service,
    bmi26x_com_port_info_t      *port_info,
    uint8_t                     *chip_id,
    uint8_t                     *dummy_byte);


/**
 * Reads status registers in Instance State.
 * This function is for debug only.
 *
 * @param[in] state                 Instance state
 * @param[in] sensor                bit mask of Sensors to enabl
 *
 * @return none
 */
void bmi26x_hal_dump_reg(sns_sensor_instance *this);

/**
 * Sets Motion Detect config.
 *
 * @param[in] state        Instance state
 * @param[in] enable       true to enable Motion Accel else false
 *
 * @return none
 */
sns_rc bmi26x_hal_set_md_config(sns_sensor_instance *const instance, bool enable);

/**
 * Updates Motion detect interrupt state.
 *
 * @param[in] state               Instance state
 * @param[in] enable              true to enable Motion Accel
 *                                      else false
 * @return none
 */
sns_rc bmi26x_hal_config_int_output(
    sns_sensor_instance     *const  instance,
    bool                    enable);

/*!
 * Config INT for MD
 * @param[in] instance   instance handler
 * @param[in] enable     enable or not
 * @param[in] md_not_armed_event   Is a Motion Detection Not armed event
 * @param[in] trigger              trigger souce
 * @return                     SNS_RC_SUCCESS on susscess
 *                             the others value when failure
 */
sns_rc bmi26x_hal_config_int_md(sns_sensor_instance *const instance,
                                bool enable,
                                bool md_not_armed_event,
                                uint8_t trigger);



/*!
 * Configure the FIFO interrupt
 * @param[in] instance
 * @param[in] enable
 * @return
 */
sns_rc bmi26x_hal_config_int_fifo(
    sns_sensor_instance     *const  instance,
    bool                    enable);



/**
 * Handles MD interrupt:
 *   1. Sends MD fired event.
 *   2. Starts Motion Accel Stream.
 *
 * @param[in] instance        Instance reference
 * @param[in] irq_timestamp   MD interrupt timestamp
 *
 * @return none
 */
void bmi26x_hal_handle_interrupt_md(
    bmi26x_instance_state   *istate,
    sns_time                 irq_timestamp);



/**
 * Processes a fifo buffer and extracts accel and gyro samples from the buffer
 * and generates events for each sample.
 *
 * @param[in] instance     Sensor instance
 * @param[in] fifo_buf     Buffer containing samples read from HW FIFO
 * @param[in] fifo_len     Number of bytes in fifo buffer
 * #param[in] trigger      trigger source
 */
void bmi26x_hal_process_fifo_data_buffer(
    sns_sensor_instance     *instance,
    const uint8_t           *fifo_buf,
    uint32_t                fifo_len,
    bmi26x_int_trigger_source_t  trigger
    );

/**
 * Handle an interrupt by reading the Fifo status register and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param[in] instance       Sensor Instance
 */
void bmi26x_hal_handle_interrupt_fifo(sns_sensor_instance *const instance);


/*!
 * Handle the interrupt
 * @param[in] instance  The sensor instance handler
 * @param[in] ctx       The interrupt context
 * @return   SNS_RC_SUCCESS on success
 *           The others value when failure
 */
sns_rc bmi26x_hal_handle_interrupt(
    sns_sensor_instance     *const instance,
    bmi26x_int_check_ctx_t    *ctx);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @param[in] instance    reference to this Instance
 */
void bmi26x_hal_send_config_event(sns_sensor_instance *const instance);

/**
 * Sends sensor temperature event.
 *
 * @param[in] instance   Sensor Instance
 */
void bmi26x_dev_convert_and_send_temp_sample(
    sns_sensor_instance *const instance,
    sns_time            timestamp,
    const uint8_t       temp_data[2]);

/**
 * Sends sensor temperature event.
 *
 * @param[in] instance   Sensor Instance
 */
void bmi26x_hal_handle_sensor_temp_sample(sns_sensor_instance *const instance);


/**
 * Starts/restarts polling timer
 *
 * @param[in] instance   Instance reference
 */
void bmi26x_hal_start_sensor_temp_polling_timer(sns_sensor_instance *this);

/**
 * Configures sensor with new/recomputed settings
 *
 * @param[in] instance   Instance reference
 * @param[in] hw_cfg_ctx hardware cofniguration context
 */
sns_rc bmi26x_hal_reconfig_hw(sns_sensor_instance  *this, bmi26x_hw_cfg_ctx_t hw_cfg_ctx);


/*!
 * Get the Sensor Request ODR and the Current ODR match or NOT
 * @param[in] istate  the BMI26X sensor state
 * @return        Invalidate Sensor list flag
 */
uint8_t bmi26x_hal_fifo_get_sensors_to_invalidate(bmi26x_instance_state   *istate);

/**
 * Reset the sensor FIFO information base on the invalidate sensor information
 * @param[in] istate                    The bmi26x sensor state handler
 * @param[in] sensors_to_invalidate      Invalidate Sensor list flag
 */
void bmi26x_hal_fifo_invalidate_sensors(
    bmi26x_instance_state   *istate,
    uint32_t                sensors_to_invalidate);

/*!
 * Update the sensor ODR request to find the master ODR sensor
 * @param[in] state   The bmi26x sensor instance used present
 */
void bmi26x_hal_fifo_update_master_odr_req(bmi26x_instance_state *state);

/*!
 * Update FIFO water mark level base on the requst
 * @param[in] istate The BMI26X sensor instance
 */
void bmi26x_hal_fifo_calc_wml_req_ldt(bmi26x_instance_state *istate);

/**
 * Sends a FIFO complete event.
 *
 * @param[in] instance   Instance reference
 * @param[in] uid        uid of sensor to send event to
 */
void bmi26x_hal_send_fifo_flush_done(
    sns_sensor_instance     *const instance,
    uint8_t                 sensor,
    sns_time                ts,
    uint8_t                 context);


/*!
 * Read bunch of data to the register wrapper function over the BUS
 * @param[in] sbus_obj   sbus object
 * @param[in] rega       The register address
 * @param[in] buf        The read buffer for the register value
 * @param[in] len        The read data len in bytes
 * @return  SNS_RC_SUCCESS  on success
 *          The others value when failure
 */
sns_rc bmi26x_sbus_read_wrapper(
    void        *sbus_obj,
    uint32_t    rega,
    uint8_t     *buf,
    uint32_t    len);

/*!
 * Write bunch of data to the register wrapper function over the BUS
 * @param[in] sbus_obj   sbus object which is the BMI26X instance state
 * @param[in] rega       The register address
 * @param[in] buf        The read buffer for the register value
 * @param[in] len        The read data len in bytes
 * @return  SNS_RC_SUCCESS  on success
 *          The others value when failure
 */
sns_rc bmi26x_sbus_write_wrapper(
    void        *sbus_obj,
    uint32_t    rega,
    uint8_t     *buf,
    uint32_t    len);

/*!
 * Write a byte data to the register over the bus
 * @param[in] sbus_obj   The sbus object which is the BMI26X instance state
 * @param[in] reg_addr   Register address
 * @param[in] regv       The target register address
 * @return           SNS_RC_SUCCESS on success
 *                   The others value when failure
 */
sns_rc bmi26x_sbus_write_byte(
    void        *sbus_obj,
    uint32_t    reg_addr,
    uint8_t     regv);

/*!
 * The Register operation for read then wirte back on the bit level
 *
 * @param[in] sbus_obj       The sbus object which is the BMI26X instance state
 * @param[in] rega           The register address
 * @param[in] bit_start      The start bit
 * @param[in] bit_end        The end bit
 * @param[in] bit_block_val  The bit value within the start and end bit
 * @return               SNS_RC_SUCCESS on success
 *                       The others value when failure
 */
sns_rc bmi26x_dev_reg_read_modify_write(
    void        *sbus_obj,
    uint8_t     rega,
    uint8_t     bit_start,
    uint8_t     bit_end,
    uint8_t     bit_block_val);


/*!
 * Get the sensor hardware error register value
 *
 * @param[in]  sbus_obj        The sbus object which is the BMI26X instance state
 * @param[out] hw_err_st       The Hardware error register value buffer
 * @return                     SNS_RC_SUCCESS on success
 *                             The others value when failure
 */
sns_rc bmi26x_dev_get_reg_hw_err_stat(
    void        *sbus_obj,
    union bmi26x_hw_err_stat        *hw_err_st);


/*!
 * Start a timer
 * @param[in]   this          The sensor instance handler
 * @param[in]   timer_stream  The timer stream
 * @param[in]   periodic      Whether the timer is a periodic timer
 * @param[in]   time_out      The timeout value from the start time in tick
 * @return                     true if success false if failure
 */
bool bmi26x_hal_start_timer(
    sns_sensor_instance     *this,
    sns_data_stream         *timer_stream,
    bool                    periodic,
    sns_time                time_out);


/*!
 * Send the COMMAND to the command register, 0x7E
 *
 * @param[in] istate            The BMI26X sensor instance state
 * @param[in] regv_cmd          The register value which want to send
 * @return                  SNS_RC_SUCCESS on success
 *                          The others value when failure
 */
sns_rc bmi26x_hal_send_cmd(
    bmi26x_instance_state   *istate,
    uint8_t                 regv_cmd);

/*!
 * Handle the timer time-out event
 *
 * @param[in] istate        The BMI26X sensor instance state
 * @return              SNS_RC_SUCCESS on success
 *                      The others value when failure
 */
sns_rc bmi26x_hal_handle_timer_cmd(bmi26x_instance_state   *istate);

/*!
 * Update the PMU state from read the PMU related register(s)
 *
 * @param[in] istate           The BMI26X sensor instante state
 * @param[in] check_err        Check the error status register along with the pmu reading
 * @return                 SNS_RC_SUCCESS on success
 *                         The others value when failure
 */
sns_rc bmi26x_hal_update_pmu_stat(bmi26x_instance_state   *istate, bool check_err);

/*!
 * Read out FIFO data
 *
 * @param[in] istate        The BMI26X sensor instance state
 * @param[in] sync_read     SYNC read or not
 * @param[in] trigger       The trigger source for FIFO drain
 */
void bmi26x_hal_fifo_drain(
    bmi26x_instance_state   *istate,
    bool                    sync_read,
    bmi26x_fifo_flush_trigger_t trigger);

/*!
 * Match the HW supported ODR from the request ODR
 *
 * @param[in] sensor_type     The sensor type to match
 * @param[in] odr_req         The request odr value from client request
 * @param[out] odr_matched    The mateched odr HW
 * @param[out] regv           The mateched ODR register value
 */
void bmi26x_hal_match_odr(
    bmi26x_sensor_type  sensor_type,
    float               odr_req,
    float               *odr_matched,
    bmi26x_regv_odr_t   *regv);

/**
 * Reads value of a GPIO pin.
 * Function has been written to demonstrate use of the GPIO
 * Service to read gpio.
 *
 * @param[in] instance     instance reference
 * @param[in] gpio_cfg     gpio config
 * @param[in] level        gpio logic level
 *
 * @return none
 */

sns_rc bmi26x_read_gpio(
    bmi26x_instance_state   *istate,
    sns_interrupt_req       *gpio_cfg,
    sns_gpio_state          *level);

/*!
 * Power mode assert due to the sensor state
 * @param istate         Sensor instance state
 * @return               SNS_RC_SUCCESS on success
 *                       the others value when failure
 */
sns_rc bmi26x_hal_power_mode_assert(bmi26x_instance_state *istate);

/**
 * Writes to a GPIO pin.
 * Function has been written to demonstrate use of the GPIO
 * Service to write to gpio.
 *
 * @param[in] instance        instance reference
 * @param[in] gpio            gpio pin to write to
 * @param[in] is_chip_pin     true if this is a chip level TLMM pin
 * @param[in] drive_strength  gpio pin drive strength
 * @param[in] pull            pull type config
 * @param[in] state           output state to write
 *
 * @return none
 */
#ifndef SSC_TARGET_HEXAGON_CORE_QDSP6_2_0
void bmi26x_write_gpio(
    sns_sensor_instance     *instance,
    uint32_t                gpio,
    bool                    is_chip_pin,
    sns_gpio_drive_strength drive_strength,
    sns_gpio_pull_type      pull,
    sns_gpio_state          state);
#endif

/*!
 * Check the INT pins status
 *
 * @param[in] istate    The BMI26X sensor instance state
 * @param[in/out] regv   regv buffer
 * @return          TRUE if the PIN is high
 *                  FALSE when the pin is on low level
 */
bool bmi26x_hal_int_pin_is_high(
    bmi26x_instance_state       *istate,
    uint8_t   *regv
    );

/*!
 * check the configuration status
 * @param[in] inst   the instance handler
 * @return          SNS_RC_SUCCESS   on success
 *                  others value when failure
 */
bool bmi26x_hal_check_cfg_available(sns_sensor_instance *inst);


/*!
 * Send Motion Detection Event
 * @param istate                  sensor instance state
 * @param event                   MD event
 * @param event_timestamp         event time stamp
 */
void bmi26x_hal_send_md_event(bmi26x_instance_state *istate,
                sns_motion_detect_event* event, sns_time event_timestamp);

/*!
 * Get system tick
 *
 * @return  the system tick
 */
static inline sns_time bmi26x_get_sys_tick()
{
    return sns_get_system_time();
}

/*!
 * Convert the tick to us
 *
 * @param[in] ticks     The ticks
 * @return          The converted us from the tick
 */
static inline uint32_t bmi26x_convert_tick_2_us(sns_time ticks)
{
    uint32_t us;

    uint64_t ticks_in_1ms = sns_convert_ns_to_ticks(1000 * 1000);

    us = ticks * 1e3 / ticks_in_1ms;

    return us;
}

/*!
 * Convert the us to ticks on the platform
 *
 * @param[in]    us   The us to covert
 * @return   The ticks to us valued
 */
static inline float bmi26x_convert_us2ticks(float us)
{
    float ticks;

    uint32_t ticks_in_1ms = sns_convert_ns_to_ticks(1000 * 1000);

    ticks = (us * ticks_in_1ms) * 1e-3;

    return ticks;
}


/*!
 * Delay for us
 *
 * @param[in] us   the delay times in us
 */
static inline void bmi26x_delay_us(uint32_t us)
{
    sns_time    ticks;
    sns_time    temp_us = us;

    ticks = sns_convert_ns_to_ticks(temp_us * 1000);

    sns_busy_wait(ticks);
}




//ASSUME_TS_IS_64BIT: sizeof(sns_time) == sizeof(uint64_t)
/*!
 * Get the time elapse between two system time
 *
 * @param[in] before   The first time
 * @param[in] after    The sencod time
 * @return         The difference time between first and second time
 */
BST_ASSUME_TS_IS_64BIT
static inline sns_time bmi26x_get_time_elapse_sys(sns_time before, sns_time after)
{

    return (after - before);
}


/*!
 * Get the time interval
 *
 * @param t1    time 1
 * @param t2    time 2
 * @return      The difference t1 and t2
 */
BST_ASSUME_TS_IS_64BIT
static inline
//we don't know which of t1 and t2 is eariler
sns_time bmi26x_get_time_interval(sns_time t1, sns_time t2)
{
    if (t2 >= t1) {
        return t2 - t1;
    } else {
        return t1 - t2;
    }
}

/*!
 * Get the average of two time
 *
 * @param[in] a    time a
 * @param[in] b    time b
 * @return     The average value of two time
 */
BST_ASSUME_TS_IS_64BIT
static inline
sns_time bmi26x_get_time_mid_sys(sns_time a, sns_time b)
{
    sns_time mid;

    mid = (a + b) >> 1;

    return mid;
}

/*!
 * Calculate the delay time shoule be performed depend on the last sbus write
 * @param ts_current
 * @param ts_last_sbus_write
 * @param ts_expect_delay
 * @return
 */
static inline uint32_t bmi26x_validatation_delay_time(sns_time ts_curr,
                sns_time ts_last_sbus_write,
                uint32_t ts_us_expect_delay)
{
    uint32_t delay_us = 0;
    if (ts_curr > ts_last_sbus_write) {
        delay_us = bmi26x_convert_tick_2_us(ts_curr - ts_last_sbus_write);

        if (delay_us > ts_us_expect_delay) {
            delay_us = 0;
        } else {
            delay_us = ts_us_expect_delay - delay_us;
        }
    } else {
        delay_us = ts_us_expect_delay;
    }

    return delay_us;
}


/*
static
inline
uint32_t bmi26x_util_get_com_div(int32_t a, int32_t b)
{
    uint32_t mcd = 1;
    int32_t  r;
    int32_t  t;

    if (a < 0) {
        a = -a;
    }

    if (b < 0) {
        b = -b;
    }

    if (a < b) {
        t = a;
        a = b;
        b = t;
    }

    if (0 == a) {
        return (uint32_t)b;
    }

    if (0 == b) {
        return (uint32_t)a;
    }

    while (1) {
        mcd = b;

        if (0 == b) {
            return (uint32_t)a;
        }

        r = a % b;

        if (0 != r) {
            a = b;
            b = r;
        } else {
            break;
        }
    }

    return mcd;
}

static
inline
uint32_t bmi26x_util_get_max_div(uint32_t a, uint32_t cap)
{
    uint32_t i;

    cap = cap < a ? cap : a;

    if (cap == a) {
        cap --;
    }

    for (i = cap; i > 1; i --) {
        if (i > 0) {
            if (0 == (a % i)) {
                return i;
            }
        } else {
            break;
        }
    }

    return 1;
}
*/



//time a is later than b
BST_ASSUME_TS_IS_64BIT
#define BMI26X_TIME_LT(a, b) ((a) > (b))

//time b is earier than a
BST_ASSUME_TS_IS_64BIT
#define BMI26X_TIME_ET(b, a) ((a) > (b))



BST_ASSUME_TS_IS_64BIT
#define BMI26X_SYS_TIME_LH(t)  ((uint32_t)(t & 0xffffffff))
#define BMI26X_SYS_TIME_HH(t)  ((uint32_t)((t >> 32) & 0xffffffff))

/*!
 * Stop the temperatue timer
 *
 * @param[in]  The sensor instance handler
 */
void bmi26x_hal_stop_tempetature_timer(sns_sensor_instance  *const this);

/*!
 * Register the interrupt in framework
 *
 * @param[in]   The sensor instance handler
 */
void bmi26x_hal_register_interrupt(sns_sensor_instance *this);

/*!
 * Enable the sensor interrupt
 *
 * @param[in] istate  The BMI26X instance state
 * @return  SNS_RC_SUCCESS on success
 *          The others value when failure
 */
sns_rc bmi26x_inst_enable_sensor_intrrupt(bmi26x_instance_state *istate);

/**
 * Quick prepare SPI if without write operation validation
 *
 * @param[in] this   Sensor frame handler
 * @return           SNS_RC_SUCCESS     on success
 *                   The others value when failure
 */
sns_rc bmi26x_hal_sensor_prepare_spi_if(sns_sensor * const this);

/*!
 * Configure for power configuration register
 *
 * @param[in] istate     The BMI26X sensor instance
 * @param[in] start_bit  The start bit to operate
 * @param[in] end_bit    The end bit to operate
 * @param[in] val        The value within the start and end bit
 * @return           SNS_RC_SUCCESS on success
 *                   failure when failure
 */
sns_rc bmi26x_dev_pwr_conf(bmi26x_instance_state       *istate,
                           uint8_t start_bit,
                           uint8_t end_bit,
                           uint8_t val);

/*!
 * Configure for power control register
 *
 * @param[in] istate     The BMI26X sensor instance
 * @param[in] start_bit  The start bit to operate
 * @param[in] end_bit    The end bit to operate
 * @param[in] val        The value within the start and end bit
 * @return           SNS_RC_SUCCESS on success
 *                   failure when failure
 */
sns_rc bmi26x_dev_pwr_ctrl(bmi26x_instance_state       *istate,
                           uint8_t start_bit,
                           uint8_t end_bit,
                           uint8_t val);

/*!
 * Configure for power control register
 *
 * @param[in] istate     The BMI26X sensor instance
 * @param[in] start_bit  The start bit to operate
 * @param[in] end_bit    The end bit to operate
 * @param[in] val        The value within the start and end bit
 * @return           SNS_RC_SUCCESS on success
 *                   failure when failure
 */
sns_rc bmi26x_hal_config_power_mode(bmi26x_instance_state       *istate,
                                    uint8_t expect_power_mode);

/*!
 * Set the sensor state to the default state
 *
 * @param[in] inst                The instance handler
 * @param[in] sensor_collection   The sensor collection which want to set to default state
 */
void bmi26x_hal_set_sensor_state_2_default_state(
    sns_sensor_instance         *inst,
    uint8_t sensor_collection);

/*!
 * Handle the temperature data
 * 1. Convert LSB value to the physical value
 * 2. Send the physical value to the framework/hal
 *
 * @param[in] instance      The sensor instance handler
 * @param[in] timestamp     The timestamp when the sensor
 *                          temperature event(timer normally) happended
 * @param[in] buf           The LSB data buffer
 */
void bmi26x_hal_convert_and_send_temp_sample(
    sns_sensor_instance     *const instance,
    sns_time                timestamp,
    const uint8_t           *buf);


#if BMI26X_CONFIG_ENABLE_PEDO
/*!
 * Handle PEDO event
 *
 * @param[in] inst            The sensor instance handler
 * @param[in] ss_type         The sensor type
 * @param[in] event_time      The time when the event happened
 * @return  SNS_RC_SUCCESS on success
 *          The others value when failure
 */
sns_rc bmi26x_hal_handle_pedo(sns_sensor_instance * const inst,
                              sns_time event_time, uint8_t evt_type);
#endif

#if BMI26X_CONFIG_ENABLE_OIS
/*!
 * Handle OIS event
 *
 * @param[in] inst            The sensor instance handler
 * @param[in] ss_type         The sensor type
 * @param[in] event_time      The time when the event happened
 * @return  SNS_RC_SUCCESS on success
 *          The others value when failure
 */
sns_rc bmi26x_hal_handle_ois(sns_sensor_instance * const inst,
                             bmi26x_sensor_type ss_type,
                             sns_time event_time);
#endif


/*!
 * Get the page number and the feature offset via the parameter param_id within the feature_scope
 * @param[in] param_id                parameter id
 * @param[out] page_offset            the feature offset within the page
 * @param[in] feature_scope           the feature scope in/out
 * @return                            page number of the input feature
 */
uint8_t bmi26x_hal_cfg_get_page_num(uint8_t param_id, uint8_t *page_offset,
                uint8_t feature_scope);

/*!
 * Get the configure data of the page located at the offset
 * @param istate             instance state
 * @param page_num           page number
 * @param pg_offset          offset in the page
 * @param rw_buffer          read buffer
 * @param rw_data_size       read data size
 * @return                   SNS_RC_SUCCESS on success
 *                           others value if failure
 */
sns_rc bmi26x_get_cfg_data(bmi26x_instance_state *istate,
                                  uint8_t page_num,
                                  uint8_t pg_offset,
                                  uint8_t *rw_buffer,
                                  uint32_t rw_data_size);

/*!
 * set the  configuration to the data
 * @param istate          instance state
 * @param page_num        page number
 * @param pg_offset       offset within the page
 * @param rw_buffer       r/w data buffer
 * @param rw_data_size    r/w data size
 * @return      SNS_RC_SUCCESS  success
 *              others value    failure
 */
sns_rc bmi26x_set_cfg_data(bmi26x_instance_state *istate,
                                  uint8_t page_num,
                                  uint8_t pg_offset,
                                  uint8_t *rw_buffer,
                                  uint32_t rw_data_size);

/*!
 * Load the configuration bit stream
 * @param inst                 instance handler
 * @param cfg_data_buffer      configuration data buffer
 * @param cfg_size             configuration buffer size
 * @param burst_write_size     burst write data size
 * @return                     SNS_RC_SUCCESS on success
 *                             the others value if failure
 */
sns_rc bmi26x_do_hal_load_cfg(sns_sensor_instance *inst, uint8_t* cfg_data_buffer,
                              uint32_t cfg_size, uint32_t burst_write_size);

#if BMI26X_CONFIG_ENABLE_CRT
/*!
 * Reconfigure the CRT parameters
 * @param inst              the instance handler
 * @param crt_params        the crt parameter buffer
 * @return                  SNS_RC_SUCCESS   on success
 *                          the others value if failure
 */
sns_rc bmi26x_hal_reconfig_crt_param(
    sns_sensor_instance     *inst,
    bmi26x_gyr_crt_gain_t   *crt_params);

/*!
 * Run CRT process to do the CRT
 *
 * @param[in] inst   The sensor instance handler
 * @param[in] trigger_source  The process trigger source
 * @param[out] need_repeat_crt Need to repeat do CRT flag
 * @return    SNS_RC_SUCCESS    on success
 *            The others value when failure
 */
sns_rc bmi26x_hal_run_crt_process(sns_sensor_instance *const inst,
                bmi26x_crt_trigger_source_t trigger_source,
                bmi26x_reason_4_rep_multiple_crt_t *need_repeat_do_crt);

/*!
 * CRT process api. In this function it will check precondition to
 * do the calibration.
 * @param inst             instance handler
 *
 * @return                            SNS_RC_SUCCESS on success
 *                                    others value if failure
 */
sns_rc bmi26x_hal_crt_evaluate_crt(sns_sensor_instance * const inst);

/*!
 * Start the timer to handle crt request
 * @param istate                      state handler
 */
void bmi26x_hal_handle_timer_crt(bmi26x_instance_state   *istate);


/*!
 * Abort the CRT process
 * @param inst   instance handler
 * @return                            SNS_RC_SUCCESS on success
 *                                    others value if failure
 */
sns_rc bmi26x_crt_abrot_process(sns_sensor_instance *const inst);

/*!
 *
 *  Get advanced feature wrapper function
 *  Get the feature data from the specified feature
 *
 * @param istate                      instance state
 * @param param_id                    parameter ID
 * @param feature_scope               feature scope
 * @param read_buffer                 read buffer
 * @param rw_data_size                read data size
 * @return                            SNS_RC_SUCCESS on success
 *                                    others value if failure
 */
sns_rc bmi26x_hal_get_cfg_data_wrapper(
                bmi26x_instance_state *istate,
                uint8_t param_id,
                uint8_t feature_scope,
                uint8_t *read_buffer,
                uint32_t rw_data_size);


/*!
 * Set configure data wrapper function via the parameter
 *
 * @param istate                      instance state
 * @param param_id                    parameter ID
 * @param feature_scope               feature scope
 * @param write_buffer                 read buffer
 * @param rw_data_size                read data size
 * @return                            SNS_RC_SUCCESS on success
 *                                    others value if failure
 */
sns_rc bmi26x_hal_set_cfg_data_wrapper(
                bmi26x_instance_state *istate,
                uint8_t param_id,
                uint8_t feature_scope,
                uint8_t *write_buffer,
                uint32_t rw_data_size);

#endif


/*!
 * Clean flush request
 *
 * @param[in] istate   The BMI26X sensor instance state
 * @param context      The flush context, who call the flush function
 */
void bmi26x_hal_flush_cleanup(bmi26x_instance_state *istate, uint8_t context);

/*!
 * Check the Interrupt pin is still high
 *
 * @param[in] istate  The BMI26X sensor instance state
 * @param[in] regv_ff_int_st int status including fifo wml interrupt
 * @return    TRUE    If still high
 *            FALSE   if already low
 */
bool bmi26x_hal_is_fifo_int_still_hangup(
    bmi26x_instance_state       *istate,
    uint8_t  regv_ff_int_st);

/*!
 * Handle the virtual sensor event
 * @param[in] istate   bmi26x instance handler
 * @param[in] regv     the regv value will be checked
 *
 * @return    true     if the INT be latched
 *            false    if the INT has not latched
 */
bool bmi26x_hal_handle_int_latch(bmi26x_instance_state *istate, uint8_t regv);

/*!
 * Handle interrupt from dae
 *
 * @param[in] this   The sensor instance handler
 * @param[in] ctx    The interrupt status checking context
 */
void bmi26x_hal_handle_drdy_from_dae(
    sns_sensor_instance     *const this,
    bmi26x_int_check_ctx_from_dae_t *ctx);

/*!
 * Reset then load configuration
 * @param inst  instance handler
 * @return   SNS_RC_SUCCESS oters value on failure
 */

sns_rc bmi26x_hal_reset_load_cfg_wrapper(sns_sensor_instance *const inst);

/*!
 * Parse the INT status
 * @param ctx               interrupt context
 * @param int_stat       INT status buffer
 */
void bmi26x_dev_parse_int_stat_flags(
    bmi26x_reg_int_ctx_t    *ctx,
    bmi26x_int_stat_flag_t  *int_stat);

/*!
 * Handle FIFO Full Interrupt
 * @param istate     instance state
 */
void bmi26x_hal_handle_interrupt_fifo_full(bmi26x_instance_state *istate);


/*!
 * Prepare the H/W
 * @param istate   instance handler
 * @return
 */
sns_rc bmi26x_hal_prepare_hw(bmi26x_instance_state  *istate);

/*!
 * clean the processing fifo flush
 * @param istate                                      instance state
 * @param ff_send_flush_done            flush done context
 */
void bmi26x_hal_cleanup_processing_fifo_flush(bmi26x_instance_state  *istate,
                                              uint32_t    ff_send_flush_done);

/*!
 * Prepare SPI interfance
 * @param istate     instance  state
 * @return  SNS_RC_SUCCESS
 */
sns_rc bmi26x_hal_prepare_spi_if(bmi26x_instance_state *istate);

#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
/*!
 *       Handle the Heart Beat Timer
 * @param istate      istate handler
 * @return NONE
 */
void bmi26x_hal_handle_hb_timer_event(bmi26x_instance_state *istate);
#endif

#if BMI26X_CONFIG_ENABLE_LOWG

#define BMI26X_FREE_FALL_MODE_NUM                 4

typedef struct bmi26x_free_fall_mode_cfg {
    uint16_t accel_setting_1;
    uint16_t accel_setting_2;
    uint16_t accel_setting_3;
    uint16_t accel_setting_4;
} bmi26x_free_fall_mode_cfg_t;

sns_rc bmi26x_hal_inst_set_lowg_config(bmi26x_instance_state *istate, bool en);

/*sns_rc bmi26x_hal_inst_en_acc_lowg(bmi26x_instance_state *istate, bool en,
                                    bool  lowg_not_armed_event);*/

/**
 * Handles LOWG interrupt:
 *   1. Sends LOWG fired event.
 *
 * @param[i] instance        Instance reference
 * @param[i] irq_timestamp   LOWg interrupt timestamp
 *
 * @return none
 */
void bmi26x_hal_handle_interrupt_lowg(
        bmi26x_instance_state   *istate,
        sns_time                 irq_timestamp);

/*!
 * Configure the LOWG interrupt
 * @param[i] inst  Instance reference
 * @param[i] enable enable lowg
 * @param[i] lowg_armed_event  lowg armed
 * @param[i] trigger the trigger source
 */
sns_rc bmi26x_hal_config_int_lowg(sns_sensor_instance *const inst,
                                     bool enable,
                                     bool lowg_armed_event,
                                     uint8_t trigger);
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP

#define BMI26X_FREE_FALL_MODE_NUM                 4

sns_rc bmi26x_hal_inst_set_dbtap_config(bmi26x_instance_state *istate, bool en);


/**
 * Handles double tap interrupt:
 *   1. Sends double tap fired event.
 *
 * @param[i] instance        Instance reference
 * @param[i] irq_timestamp   double tap interrupt timestamp
 *
 * @return none
 */
void bmi26x_hal_handle_interrupt_dbtap(
        bmi26x_instance_state   *istate,
        sns_time                 irq_timestamp);

/*!
 * Configure the double tap interrupt
 * @param[i] inst  Instance reference
 * @param[i] enable enable double tap
 * @param[i] dbtap_armed_event  double tap armed
 * @param[i] trigger the trigger source
 */
sns_rc bmi26x_hal_config_int_dbtap(sns_sensor_instance *const inst,
                                     bool enable,
                                     bool dbtap_armed_event,
                                     uint8_t trigger);
#endif
