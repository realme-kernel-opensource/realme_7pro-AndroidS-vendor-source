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
/**!
 * sensor configuration definition
 */


enum bmi26x_pwr_mode {
    BMI26X_PWR_ACC_NONE,
    BMI26X_PWR_ACC_SUSPED,
    BMI26X_PWR_ACC_LOW_POWER,
    BMI26X_PWR_ACC_NORMAL,


    BMI26X_PWR_GYR_SUSPED,
    BMI26X_PWR_GYR_LOW_POWER,
    BMI26X_PWR_GYR_NORMAL,

    BMI26X_PWR_TEMP_SUSPEND,
    BMI26X_PWR_TEMP_NORMAL
} ;


//@BMI26x sensor config

enum bmi26x_regv_acc_bwp {
    BMI26X_REGV_ACC_BWP_OSR4_AVG1 = 0,//!< BMI26X_REGV_ACC_BWP_OSR4_AVG1
    BMI26X_REGV_ACC_BWP_OSR2_AVG2,    //!< BMI26X_REGV_ACC_BWP_OSR2_AVG2
    BMI26X_REGV_ACC_BWP_NORM_AVG4,    //!< BMI26X_REGV_ACC_BWP_NORM_AVG4
    BMI26X_REGV_ACC_BWP_CIC_AVG8,     //!< BMI26X_REGV_ACC_BWP_CIC_AVG8
    BMI26X_REGV_ACC_BWP_RES_AVG16,    //!< BMI26X_REGV_ACC_BWP_RES_AVG16
    BMI26X_REGV_ACC_BWP_RES_AVG32,    //!< BMI26X_REGV_ACC_BWP_RES_AVG32
    BMI26X_REGV_ACC_BWP_RES_AVG64,    //!< BMI26X_REGV_ACC_BWP_RES_AVG64
    BMI26X_REGV_ACC_BWP_RES_AVG128,   //!< BMI26X_REGV_ACC_BWP_RES_AVG128
};

enum bmi26x_regv_gyr_bwp {
    BMI26X_REGV_GYR_BWP_OSR4 = 0,
    BMI26X_REGV_GYR_BWP_OSR2,
    BMI26X_REGV_GYR_BWP_NORM,
    BMI26X_REGV_GYR_BWP_CIC,
};

typedef enum {
    BMI26X_REGV_RANGE_ACC_PM2G = 0,
    BMI26X_REGV_RANGE_ACC_PM4G = 1,
    BMI26X_REGV_RANGE_ACC_PM8G = 2,
    BMI26X_REGV_RANGE_ACC_PM16G = 3,
} bmi26x_regv_acc_range_t;

typedef enum {
    BMI26X_RANGE_ACC_PM2G = 0,
    BMI26X_RANGE_ACC_PM4G,
    BMI26X_RANGE_ACC_PM8G,
    BMI26X_RANGE_ACC_PM16G,
} bmi26x_acc_range_t;



typedef enum {
    BMI26X_REGV_RANGE_GYR_PM2000DPS = 0,
    BMI26X_REGV_RANGE_GYR_PM1000DPS,
    BMI26X_REGV_RANGE_GYR_PM500DPS,
    BMI26X_REGV_RANGE_GYR_PM250DPS,
    BMI26X_REGV_RANGE_GYR_PM125DPS,
} bmi26x_regv_gyr_range_t;

typedef enum {
    BMI26X_RANGE_GYR_PM125DPS = 0,
    BMI26X_RANGE_GYR_PM250DPS,
    BMI26X_RANGE_GYR_PM500DPS,
    BMI26X_RANGE_GYR_PM1000DPS,
    BMI26X_RANGE_GYR_PM2000DPS,
} bmi26x_gyr_range_t;


typedef enum BMI26X_REGV_ODR {
    BMI26X_REGV_ODR_OFF = 0,
    BMI26X_REGV_ODR_0_78HZ,
    BMI26X_REGV_ODR_1_56HZ,
    BMI26X_REGV_ODR_3_12HZ,
    BMI26X_REGV_ODR_6_25HZ,
    BMI26X_REGV_ODR_12_5HZ,
    BMI26X_REGV_ODR_25HZ,
    BMI26X_REGV_ODR_50HZ,
    BMI26X_REGV_ODR_100HZ,
    BMI26X_REGV_ODR_200HZ,
    BMI26X_REGV_ODR_400HZ,
    BMI26X_REGV_ODR_800HZ,
    BMI26X_REGV_ODR_1600HZ,
} bmi26x_regv_odr_t;

enum bmi26x_power_mode {
    BMI26X_POWER_MODE_NONE  = 0,
    BMI26X_POWER_MODE_SUSPEND,
    BMI26X_POWER_MODE_LOW,
    BMI26X_POWER_MODE_NORMAL,
    BMI26X_POWER_MODE_HIGH_PERFORMANCE,
};

enum bmi26x_md_config_source {
    BMI26X_MD_CONFIG_HW_CFG,
    BMI26X_MD_CONFIG_MD_CLEARUP,
    BMI26X_MD_CONFIG_ENABLE_INT,
    BMI26X_MD_CONFIG_HANDLE_INT,
} ;


enum bmi26x_lowg_config_source {
    BMI26X_LOWG_CONFIG_HW_CFG,
    BMI26X_LOWG_CONFIG_LOWG_CLEARUP,
    BMI26X_LOWG_CONFIG_ENABLE_INT,
    BMI26X_LOWG_CONFIG_HANDLE_INT,
} ;

enum bmi26x_dbtap_config_source {
    BMI26X_DBTAP_CONFIG_HW_CFG,
    BMI26X_DBTAP_CONFIG_DBTP_CLEARUP,
    BMI26X_DBTAP_CONFIG_ENABLE_INT,
    BMI26X_DBTAP_CONFIG_HANDLE_INT,
} ;



