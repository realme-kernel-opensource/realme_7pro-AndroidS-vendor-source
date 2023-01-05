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
 * @file sns_bmi26x_lowg.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#pragma once

// <Registry Sensors.SEE-LITE sensor>
#ifndef BMI26X_CONFIG_ENABLE_SEE_LITE
#define BMI26X_CONFIG_ENABLE_SEE_LITE   0
#endif

#ifndef BMI26X_CONFIG_ENABLE_REGISTRY
#define BMI26X_CONFIG_ENABLE_REGISTRY   1
#endif


#define BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT    0

#if !BMI26X_CONFIG_ENABLE_REGISTRY
#ifndef BMI26X_CONFIG_DFT_BUS_SPI
#define BMI26X_CONFIG_DFT_BUS_SPI    0
#endif
#else
#if BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
#ifndef BMI26X_CONFIG_DFT_BUS_SPI
#define BMI26X_CONFIG_DFT_BUS_SPI    0
#endif
#endif
#endif


#if !BMI26X_CONFIG_ENABLE_REGISTRY
#ifndef BMI26X_CONFIG_DFT_BUS_SPI
#define BMI26X_CONFIG_DFT_BUS_SPI    1
#endif
#endif

// LOWG
#ifndef BMI26X_CONFIG_ENABLE_LOWG
#define BMI26X_CONFIG_ENABLE_LOWG          0

#define BMI26X_CONFIG_ENABLE_LOWG_ON_INT2  0

#endif

// <Debug Print Messages>
#ifndef BMI26X_CONFIG_ENABLE_DEBUG
#define BMI26X_CONFIG_ENABLE_DEBUG   1
#endif

// <DAE Usage>
#ifdef SNS_ENABLE_DAE
#define BMI26X_CONFIG_ENABLE_DAE     1
#define BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE   1
#endif


// <Diagnostic Logging>
#ifndef BMI26X_CONFIG_ENABLE_DIAG_LOG
#define BMI26X_CONFIG_ENABLE_DIAG_LOG           0
#endif


// <power rail reference>
#ifndef BMI26X_CONFIG_POWER_RAIL
#define BMI26X_CONFIG_POWER_RAIL                1
#endif

#ifndef BMI26X_CONFIG_ENABLE_ISLAND_MODE
#define BMI26X_CONFIG_ENABLE_ISLAND_MODE        1
#endif

#ifndef BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
#define BMI26X_CONFIG_ENABLE_SELF_TEST_FAC      1
#endif


#ifndef BMI26X_CONFIG_ENABLE_DUMP_REG
#define BMI26X_CONFIG_ENABLE_DUMP_REG           0
#endif


#ifndef BMI26X_CONFIG_ENABLE_MAG_IF
#define BMI26X_CONFIG_ENABLE_MAG_IF             0
#endif

#ifndef BMI26X_CONFIG_ENABLE_CONF_STREAM
#define BMI26X_CONFIG_ENABLE_CONF_STREAM        0
#endif

// normally we use 12.5, change it to 25 when 1hz cts failure
#define BMI26X_CONFIG_ACC_LOWEST_ODR            25      //sometimes using 12.5 will hurt power consumption when gyro is also used
#define BMI26X_CONFIG_ACC_FASTEST_ODR           400

// normally we use 12.5, change it to 25 when 1hz cts failure
#define BMI26X_CONFIG_GYR_LOWEST_ODR            25
#define BMI26X_CONFIG_GYR_FASTEST_ODR           400

#ifndef BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
#define BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW 0    //enable this if desired lowest sample_rate of gyro is 12.5hz, (BMI26X_CONFIG_GYR_LOWEST_ODR has to be 12.5hz to be effective)
#endif

#define BMI26X_CONFIG_ENABLE_LOW_LATENCY_RATE   0

#define BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION 0
#define BMI26X_CONFIG_ENABLE_SELF_TEST_HW_SW    1

#define BMI26X_CONFIG_ENABLE_OPTIM_LOAD_FP      0
#define BMI26X_CONFIG_ENABLE_SIMPLE_CAL         1

// NOTICE, this should be disable when BMI26X_CONFIG_ENABLE_DAE opened
#define BMI26X_CONFIG_ENABLE_DRI_MODE           0


// <step counter>
#ifndef BMI26X_CONFIG_ENABLE_PEDO
#define BMI26X_CONFIG_ENABLE_PEDO               0
#define BMI26X_CONFIG_ENABLE_PEDO_TIMER         0
#endif

// <OIS>
#ifndef BMI26X_CONFIG_ENABLE_OIS
#define BMI26X_CONFIG_ENABLE_OIS               1
#endif

// <CRT>
#ifndef BMI26X_CONFIG_ENABLE_CRT
#define BMI26X_CONFIG_ENABLE_CRT               1
#define BMI26X_CONFIG_ENABLE_CRT_ON_FAC_TEST   1
#endif

#if BMI26X_CONFIG_ENABLE_CRT_ON_FAC_TEST
#if BMI26X_CONFIG_ENABLE_CRT
#else
#error "!!!error configuration!!!"
#endif
#endif
// </CRT>

// <Double tap>

#ifndef BMI26X_CONFIG_ENABLE_DOUBLE_TAP
#define BMI26X_CONFIG_ENABLE_DOUBLE_TAP   1
#include "sns_double_tap.pb.h"
#endif

// </Double tap>



//if a clock of 32768hz is used
#define BMI26X_CONFIG_SEE_LITE_SLOW_CLOCK      0
#define BMI26X_CONFIG_ENABLE_LOG_DATA_L3       0
