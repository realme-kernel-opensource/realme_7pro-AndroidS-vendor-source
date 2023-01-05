#pragma once
/**
 * @file sns_lsm6ds3c_build_config.h
 *
 * Copyright (c) 2018-2019, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
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
 *
 **/



/** Default build flags for debugging */

/** Enables DBG_PRINTF macro for default debug logs */
#define LSM6DS3C_LOG_VERBOSE_DEFAULT    1

/** Enables DBG_PRINTF_EX macro for extensive debug logs */
#define LSM6DS3C_LOG_VERBOSE_EX         0

/** Enables LSM6DS3C_INST_DEBUG_TS macro for instance and timestamp logs */
#define LSM6DS3C_DEBUG_TS               0

/** Enables DEBUG_TS_EST macro for logs in Timestamp estimation function */
#define LSM6DS3C_DEBUG_TS_EST           0


/** Enables logs of sensor data, very costly, do not enable unless required*/
#define LSM6DS3C_DEBUG_SENSOR_DATA      0

/** Enables logs of register read/write operation */
#define LSM6DS3C_DUMP_REG               0

/** Enables the logs for sanity tests */
#define LSM6DS3C_AUTO_DEBUG             0

#define LSM6DS3C_PATTERN_DEBUG          0
/** Enables DAE support */
#ifdef SNS_ENABLE_DAE
#define LSM6DS3C_DAE_ENABLED            1
#else
#define LSM6DS3C_DAE_ENABLED            0
#endif

/** Build flags for advanced features */

/** Enables embedded sensor features like free fall, step counter, etc.. */
#define LSM6DS3C_ESP_ENABLED            0
#if LSM6DS3C_ESP_ENABLED
#define LSM6DS3C_ESP_ACTIVITY           0
#define LSM6DS3C_ESP_FREE_FALL          0
#define LSM6DS3C_ESP_HIGH_SHOCK         0
#define LSM6DS3C_ESP_STEP_COUNTER       1
#endif

/** Enables LITE Driver */
#define LSM6DS3C_LITE_DRIVER_ENABLED    0

/** Enables Dual sensor fetaure */
#ifdef LSM6DS3C_ENABLE_DUAL_SENSOR
#define LSM6DS3C_DUAL_SENSOR_ENABLED    1
#else
#define LSM6DS3C_DUAL_SENSOR_ENABLED    0
#endif

#define LSM6DS3C_ODR_REGISTRY_FEATURE_ENABLE 1
#define LSM6DS3C_DAE_TIMESTAMP_TYPE 1
#define LSM6DS3C_REGISTRY_WRITE_EVENT   1

/** If fixed orientaiton is to be used for Self-Test */
#define LSM6DS3C_USE_FIXED_ORIENTATION 1

#define LSM6DS3C_HS_MULTI_AXIS_DETECTION 0

/** Handle OEM specific requests */
#define LSM6DS3C_SENSOR_OEM_CONFIG      0
#define LSM6DS3C_USE_OEM_TS_OFFSET      0
#define LSM6DS3C_SET_OEM_OIS_PU_DIS     0
#define LSM6DS3C_OEM_FACTORY_CONFIG     0

#define LSM6DS3C_DEBUG_CRASH            0
/** Default Build flags for Dragon board
 * This replaces old BUILD_DB flag needs */
#ifdef SENSORS_DD_DEV_FLAG

#define LSM6DS3C_ENABLE_REGISTRY        1
#define SDM_845_BUILDS                 1

#endif

/** set sensor instacne count for Dual sensors */
#if LSM6DS3C_DUAL_SENSOR_ENABLED
#define SENSOR_INST_CNT                2
#else
#define SENSOR_INST_CNT                1
#endif

#if LSM6DS3C_LOG_VERBOSE_EX
#define LSM6DS3C_LOG_VERBOSE_DEFAULT    1
#endif


#if LSM6DS3C_DEBUG_TS

#define LSM6DS3C_INST_DEBUG_TS(prio, inst, ...) do { \
SNS_INST_PRINTF(prio, inst, __VA_ARGS__); \
} while (0)

#else
#define LSM6DS3C_INST_DEBUG_TS(prio, sensor, ...)
#endif

#if LSM6DS3C_DEBUG_TS_EST

#define DEBUG_TS_EST(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst, __VA_ARGS__); \
} while (0)

#else
#define DEBUG_TS_EST(prio, sensor, ...)
#endif

#if LSM6DS3C_LOG_VERBOSE_EX

#define DBG_PRINTF_EX(prio, sensor, ...) do { \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)

#define DBG_INST_PRINTF_EX(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst , __VA_ARGS__); \
} while (0)

#else
#define DBG_PRINTF_EX(prio, sensor,...) UNUSED_VAR(sensor);
#define DBG_INST_PRINTF_EX(prio, inst,...) UNUSED_VAR(inst);
#endif

#if LSM6DS3C_LOG_VERBOSE_DEFAULT

#define DBG_PRINTF(prio, sensor, ...) do { \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)

#define DBG_INST_PRINTF(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst , __VA_ARGS__); \
} while (0)

#else
#define DBG_PRINTF(prio, sensor,...) UNUSED_VAR(sensor);
#define DBG_INST_PRINTF(prio, inst,...) UNUSED_VAR(inst);
#endif

#define LSM6DS3C_LOGGING_DISABLED   0

#if LSM6DS3C_LITE_DRIVER_ENABLED
#define LSM6DS3C_REGISTRY_DISABLED  1
#define LSM6DS3C_ATTRIBUTE_DISABLED 1
#define LSM6DS3C_ISLAND_DISABLED    1
#define LSM6DS3C_LOGGING_DISABLED   1
#define LSM6DS3C_POWERRAIL_DISABLED 1
#endif
