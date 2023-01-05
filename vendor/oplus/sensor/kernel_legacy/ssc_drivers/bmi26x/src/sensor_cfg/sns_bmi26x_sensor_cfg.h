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
#pragma once

#include "sns_sensor.h"
#include "sns_bmi26x_config.h"

#define BMI260_VER                     1
#define BMI261_VER                     2

#define BMI26X_VER                     BMI260_VER

#define BMI26X_MAX_CONFIG_PAGE_NUM          7
#define BMI26X_PAGE_BLOCK_DEFAULT_SIZE      16

/*  name Macro to define axis-re-mapping feature for BMI26X */
#define BMI26X_CONFIG_INDEX_GYRO                       1
#define BMI26X_CONFIG_INDEX_AUX                        2
#define BMI26X_CONFIG_INDEX_TEMP                       3
#define BMI26X_CONFIG_INDEX_ANY_MOTION                 4
#define BMI26X_CONFIG_INDEX_NO_MOTION                  5
#define BMI26X_CONFIG_INDEX_TILT                       6
#define BMI26X_CONFIG_INDEX_ORIENTATION                7
#define BMI26X_CONFIG_INDEX_SIG_MOTION                 8
#define BMI26X_CONFIG_INDEX_STEP_DETECTOR              9
#define BMI26X_CONFIG_INDEX_STEP_COUNTER               10
#define BMI26X_CONFIG_INDEX_STEP_ACTIVITY              11
#define BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE           12
#define BMI26X_CONFIG_INDEX_PICK_UP                    13
#define BMI26X_CONFIG_INDEX_GLANCE_DETECTOR            14
#define BMI26X_CONFIG_INDEX_WAKE_UP                    15
#define BMI26X_CONFIG_INDEX_HIGH_G                     16
#define BMI26X_CONFIG_INDEX_LOW_G                      17
#define BMI26X_CONFIG_INDEX_FLAT                       18
#define BMI26X_CONFIG_INDEX_EXT_SENS_SYNC              19
#define BMI26X_CONFIG_INDEX_SELF_OFFSET_CORR           20
#define BMI26X_CONFIG_INDEX_WRIST_GESTURE              21
#define BMI26X_CONFIG_INDEX_WRIST_WEAR_WAKE_UP         22
#define BMI26X_CONFIG_INDEX_LP_FILTER                  31


#define BMI26X_CONFIG_INDEX_AXIS_MAP                   23
#define BMI26X_CONFIG_INDEX_GYR_SELF_TEST_OFF          24
#define BMI26X_CONFIG_INDEX_NVM_STATUS                 26
#define BMI26X_CONFIG_INDEX_VFRM_STATUS                27
#define BMI26X_CONFIG_INDEX_SELF_TEST                  29

#define BMI26X_CONFIG_INDEX_OUT_CROSS_SENSE      28
#define BMI26X_CONFIG_INDEX_GYR_TRIGGER_SELECT   32
#define BMI26X_CONFIG_INDEX_GYR_ABORT_CRT        33
#define BMI26X_CONFIG_INDEX_NVM_PROG_PREP        34
#define BMI26X_CONFIG_INDEX_MAX_BURST_LEN        30   /* max burst length */
#define BMI26X_CONFIG_INDEX_CONFIG_ID            (41)


// double tap
#define BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_1   (38)
#define BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_2   (39)



/* To define BMI2 pages */
#define BMI26X_PAGE_0                     (0)
#define BMI26X_PAGE_1                     (1)
#define BMI26X_PAGE_2                     (2)
#define BMI26X_PAGE_3                     (3)
#define BMI26X_PAGE_4                     (4)
#define BMI26X_PAGE_5                     (5)
#define BMI26X_PAGE_6                     (6)
#define BMI26X_PAGE_7                     (7)

/* BMI26X feature input start addresses */
#define BMI26X_CONFIG_ID_STRT_ADDR           (0x00)
/** page 1*/
#define BMI26X_MAX_BURST_LEN_STRT_ADDR       (0x02)   // "max_burst_len", the second byte, the fist WORD 0x008:{bit 0...7} in page

#define BMI26X_SELF_OFF_CORR_STRT_ADDR       (0x02)   // "select", the second byte, the second WORD 0x009:{bit8} in page
#define BMI26X_GYR_ABORT_CRT                 (0x02)   // "block", the third byte, the second WORD 0x009:{bit 9}

#define BMI26X_AXIS_REMAP_STRT_ADDR          (0x04)   // "map_xx_axis, map_xx_sign", the fourth byte, the third WORD 0x00A:{bit 0...8}

#define BMI26X_AXIS_GYR_SELF_OFF_START_ADSDR (0x05)   // "gyr_self_offset_correction", the fifth byte, the third WORD 0x00A:{bit 9}
#define BMI26X_NVM_PROG_PREP_START_ADSDR     (0x05)   // "nvm_prog_prep", the fifth byte, the third WORD 0x00A:{bit 10}

#define BMI26X_ANY_MOT_STRT_ADDR             (0x06)   // the sixth byte, the fourth WORD 0x00B(bit 0...15)
#define BMI26X_GYR_USER_GAIN_STRT_ADDR       (0x0A)



#define BMI26X_STEP_COUNTER_MEAN_STEP_DURATOPM_OFFSET  (0x00)   // the first byte{0}, the first WORD 0x011{bit 0...15}  in page
#define BMI26X_STEP_COUNTER_OFFSET                     (0x02)   // the second byte{2}, the second WORD 0x012{bit 0...9, 10, 11, 12, 13}
#define BMI26X_STEP_DETECTOR_OFFSET                    (0x02)   // the second byte{2}, the second WORD 0x012{bit 0...9, 10, 11, 12, 13}

// low filter
#define BMI26X_LP_FILTER_OFFSET                        (0x00)
#define BMI26X_FEATURE_LP_PARAM_SIZE                   2

// # HIGH-G
// 0x12-0x14, range:3 WORD/6 bytes, @page2
#define BMI26X_FEATURE_HIGHG_START_ADDR                (0x04)

// # LOW-G
// 0x15-0x17, range:3 WORD/6 bytes, @page2
#define BMI26X_FEATURE_LOWG_PARAM_SIZE                 10       // 16 bytes, 10 r/w for current
#define BMI26X_FEATURE_LOWG_START_ADDR                 (0x00)

typedef struct bmi26x_feature_lowg_t {
    // map to 0x18
    uint16_t   feature_en     :1;
    uint16_t   out_conf       :4;

    // map to 0x19 - 0x1F
    uint16_t   free_fall_acc_setting[7];   //1-7
} bmi26x_feature_lowg_t;


/* BMI26X feature output start addresses */
#define BMI26X_STEP_CNT_OUT_STRT_ADDR       (0x00)
#define BMI26X_STEP_ACT_OUT_STRT_ADDR       (0x04)
#define BMI26X_GYR_USER_GAIN_OUT_STRT_ADDR  (0x06)
#define BMI26X_CROSS_SENSE_STRT_ADDR        (0x0C)
#define BMI26X_NVM_VFRM_OUT_STRT_ADDR       (0x0E)


// CRT
#define BMI26X_CRT_PARAM_NUM                 3
#define BMI26X_REGA_CRT_CAPACITY             0x1E
#define BMI26X_REGA_CRT_CONFIG               0x77
#define BMI26X_REGA_CRT_PARAM_0              0x78
#define BMI26X_REGA_CRT_PARAM_1              0x79
#define BMI26X_REGA_CRT_PARAM_2              0x7A

#define BMI26X_REGA_CRT_CONFIG_BIT_4_CRT_GAIN_EN  7

#define BMI26X_CONFIG_BURST_WRITE_SIZE       256 //should cut down in some case due to platform limitation
#define BMI26X_CHECK_CONFIGURE_STATUS_TIMES  15

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
#define BMI26X_FEATURE_DOUBLE_TAP_START_ADDR                (0x00)
#define BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE                 16
#define BMI26X_DOUBLE_TAP_SENSITIVITY_THRESHOLD_MAX   15
#define BMI26X_DOUBLE_TAP_DEFAULT_SENSITIVITY_THRESHOLD_MAX   3
#define BMI26X_DOUBLE_TAP_DEFAULT_MAX_GEST_DUR                400    //ms
#define BMI26X_DOUBLE_TAP_DEFAULT_REGV_MAX_GEST_DUR                80 // 400 ms
#define BMI26X_DOUBLE_TAP_DEFAULT_REGV_QUITE_TIME_AFTER_GEST       80 //400 ms

typedef struct bmi26x_feature_dbtap {
    // map to 0x28
    uint16_t   single_tap_en   :1;
    uint16_t   double_tap_en   :1;
    uint16_t   triple_tap_en   :1;
    uint16_t   data_reg_en     :1;
    uint16_t   out_conf        :4;
    uint16_t                   :8;

    // map to 0x29 NOT used
    uint16_t   dbtap_setting_1; //reserved
    // map to 0x2a
    uint16_t   tap_sens_thres;
    // map to 0x2b
    uint16_t   max_gest_dur;
    // map to 0x2c - 0x2e
    uint16_t   dbtap_setting_2[3];
    // map to 0x2f
    uint16_t   quite_time_after_gest;
} bmi26x_feature_dbtap_t;
#endif



/*!
 *  Feature configuration definition
*/
typedef struct bmi26x_feature_config {
    uint32_t type       :8;
    uint32_t page       :4;
    uint32_t start_addr :4;
    uint32_t bit_start_in_word_adress :8;
    uint32_t bit_end_in_word_adress   :8;
    uint32_t bit_size_of_feature      :8;
    uint32_t block_size  :8;
} bmi26x_feature_config_t;

/*!
 *  Define the advance feature enumer value for input and output
 */
enum {
    BMI26X_ADVANCED_FEATURE_INPUT, //!< BMI26X_ADVANCED_FEATURE_INPUT
    BMI26X_ADVANCED_FEATURE_OUTPUT,//!< BMI26X_ADVANCED_FEATURE_OUTPUT
};

#if (BMI26X_VER == BMI261_VER)
#define  bmi26x_sensor_cfg_size     bmi261_sensor_cfg_size
#define  bmi26x_sensor_cfg          bmi261_sensor_cfg
#define  feat_in                    bmi261_feat_in

extern unsigned int bmi261_sensor_cfg_size;
extern unsigned char bmi261_sensor_cfg[];
extern bmi26x_feature_config_t bmi261_feat_in[];
extern int bmi261_sensor_feature_confure_in_size(void);

#define  bmi26x_sensor_feature_configure_in_size bmi261_sensor_feature_confure_in_size

#elif (BMI26X_VER == BMI260_VER)
#define BMI26X_VER_260_C2          1
#define BMI26X_CONFIG_INDEX_STEP_CNT        BMI26X_CONFIG_INDEX_STEP_COUNTER_PARAMS

/* in */
#define  bmi26x_sensor_cfg_size bmi260_sensor_cfg_size
#define  bmi26x_sensor_cfg      bmi260_sensor_cfg
#define  feat_in                bmi260_feat_in

/* out */
#define feat_out                bmi260_feat_out

extern unsigned int bmi260_sensor_cfg_size;
extern uint8_t bmi260_sensor_cfg[];
extern bmi26x_feature_config_t bmi260_feat_in[];
extern bmi26x_feature_config_t bmi260_feat_out[];

#if BMI26X_CONFIG_ENABLE_CRT
#define  bmi26x_crt_cfg         bmi260_sensor_cfg
#define  bmi26x_crt_cfg_size    bmi260_sensor_crt_cfg_size
extern unsigned int bmi260_sensor_crt_cfg_size;
extern unsigned char bmi260_crt_cfg[];
#endif

/*!
 * Get the advanced feature size
 *
 * @return  The sensor configure for input on size
 */
extern int bmi260_sensor_feature_confure_in_size(void);

/*!
 * Get the advanced feature size
 *
 * @return  The sensor configure for output on size
 */
extern int bmi260_sensor_feature_confure_out_size(void);

#define  bmi26x_sensor_feature_configure_in_size bmi260_sensor_feature_confure_in_size
#define  bmi26x_sensor_feature_configure_out_size bmi260_sensor_feature_confure_out_size

#endif
