#pragma once
/**
 * @file sns_lsm6dso_build_config.h
 *
 * Copyright (c) 2020, STMicroelectronics.
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
#define LSM6DSO_LOG_VERBOSE_DEFAULT    1

/** Enables DBG_PRINTF_EX macro for extensive debug logs */
#define LSM6DSO_LOG_VERBOSE_EX         0

/** Enables LSM6DSO_INST_DEBUG_TS macro for instance and timestamp logs */
#define LSM6DSO_DEBUG_TS               0

/** Enables DEBUG_TS_EST macro for logs in Timestamp estimation function */
#define LSM6DSO_DEBUG_TS_EST           0


/** Enables logs of sensor data, very costly, do not enable unless required*/
#define LSM6DSO_DEBUG_SENSOR_DATA      0

/** Enables logs of register read/write operation */
#define LSM6DSO_DUMP_REG               0

/** Enables the logs for sanity tests */
#define LSM6DSO_AUTO_DEBUG             0

/** Enables debug code for i3c, reads all CCC information */
#define LSM6DSO_DEBUG_I3C              0

/** Disables raw log packets of samples and interrupts */
#define LSM6DSO_LOGGING_DISABLED       0

/** Build flags for advanced features */

/** Enables embedded sensor features like free fall, step counter, etc.. */
#define LSM6DSO_ESP_ACTIVITY           0

#define LSM6DSO_ESP_FREE_FALL          0

#define LSM6DSO_ESP_HIGH_SHOCK         0

#define LSM6DSO_ESP_STEP_COUNTER       0

#define LSM6DSO_ESP_DOUBLE_TAP         0
#define LSM6DSO_EX_TAP_TUNING_ENABLED  0

/** Enables FSM/MLC features */
#define LSM6DSO_ESP_XSENSOR_1          0

#define LSM6DSO_ESP_XSENSOR_2          0

/** Enabled FSM support for ESP sensors */
#define LSM6DSO_FSM_ENABLED            0

/** Enabled FSM support for ESP sensors */
#define LSM6DSO_MLC_ENABLED            0

/** ENABLE OEM SPECIFIC XSENSOR EVENT HANDLING */
#define LSM6DSO_XSENSOR_OEM_EVENT_HANDLING 0

/** Enables S4S fetaure */
#define LSM6DSO_S4S_ENABLED            0

/** Enables OIS feature */
#define LSM6DSO_OIS_ENABLED            0

/** Temporary method - use registry eventually */
#define LSM6DSO_OIS_POLLING            0

/** Enables LITE Driver */
#define LSM6DSO_LITE_DRIVER_ENABLED    0

/** Force enable temperature if accel or gyro is enabled */
#define LSM6DSO_FORCE_SENSOR_TEMP_ENABLED 0

#define LSM6DSO_PASSIVE_SENSOR_SUPPORT    1

/** return special instance pointer if flush req is handled*/
#define LSM6DSO_FLUSH_SPECIAL_HANDLING   1

#define MAX_LOW_LATENCY_RATE LSM6DSO_ODR_832


/** Enables Dual sensor fetaure */
/** set sensor instacne count for Dual sensors */
#ifdef LSM6DSO_ENABLE_DUAL_SENSOR
#define LSM6DSO_DUAL_SENSOR_ENABLED    1
#define SENSOR_INST_CNT                2
#else
#define LSM6DSO_DUAL_SENSOR_ENABLED    0
#define SENSOR_INST_CNT                1
#endif

/** Enables DAE support */
#ifdef SNS_ENABLE_DAE
#define LSM6DSO_DAE_ENABLED             1
#else
#define LSM6DSO_DAE_ENABLED             0
#endif

#ifdef SSC_TARGET_NO_I3C_SUPPORT
#define LSM6DSO_USE_I3C                 0
#ifdef BUILD_DB
#define SDM_845_BUILDS                  0
#else
#define SDM_845_BUILDS                  1
#endif
#else
#define LSM6DSO_USE_I3C                 1
#define SDM_845_BUILDS                  0
#endif

#define LSM6DSO_ODR_REGISTRY_FEATURE_ENABLE  1
#define LSM6DSO_DAE_TIMESTAMP_TYPE           1
#define LSM6DSO_REGISTRY_WRITE_EVENT         1
#define LSM6DSO_REMOVE_ON_CHANGE_REQUEST     1
#define LSM6DSO_UPDATED_RANGE_RESOLUTION     1
#define LSM6DSO_TS_IN_META_DATA              0
#define LSM6DSO_FSM_LC_ENABLED               0
#define LSM6DSO_FSM_OUTS_ENABLED             0

//By default disable Xsensor for hw_id=1
#define LSM6DSO_SUB_XSENSOR_DISABLED         1

/** If fixed orientaiton is to be used for Self-Test */
#define LSM6DSO_USE_FIXED_ORIENTATION   1

#define LSM6DSO_HS_MULTI_AXIS_DETECTION 1

/** Handle OEM specific requests */
#define LSM6DSO_SENSOR_OEM_CONFIG      0
#define LSM6DSO_USE_OEM_TS_OFFSET      0
#define LSM6DSO_SET_OIS_PU_DIS         0
#define LSM6DSO_OEM_FACTORY_CONFIG     0

#if LSM6DSO_LOG_VERBOSE_EX
#define LSM6DSO_LOG_VERBOSE_DEFAULT    1
#endif

#if LSM6DSO_AUTO_DEBUG

#define LSM6DSO_INST_AUTO_DEBUG_PRINTF(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst, __VA_ARGS__); \
} while (0)
#define LSM6DSO_AUTO_DEBUG_PRINTF(prio, sensor, ...) do { \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)

#else
#define LSM6DSO_AUTO_DEBUG_PRINTF(prio, sensor,...) UNUSED_VAR(sensor);
#define LSM6DSO_INST_AUTO_DEBUG_PRINTF(prio, inst,...) UNUSED_VAR(inst);
#endif

#if LSM6DSO_DEBUG_TS

#define LSM6DSO_INST_DEBUG_TS(prio, inst, ...) do { \
SNS_INST_PRINTF(prio, inst, __VA_ARGS__); \
} while (0)

#else
#define LSM6DSO_INST_DEBUG_TS(prio, sensor, ...)
#endif

#if LSM6DSO_DEBUG_TS_EST

#define DEBUG_TS_EST(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst, __VA_ARGS__); \
} while (0)

#else
#define DEBUG_TS_EST(prio, sensor, ...)
#endif

#if LSM6DSO_LOG_VERBOSE_EX

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

#if LSM6DSO_LOG_VERBOSE_DEFAULT

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


#if LSM6DSO_LITE_DRIVER_ENABLED
#define LSM6DSO_REGISTRY_DISABLED  1
#define LSM6DSO_ATTRIBUTE_DISABLED 1
#define LSM6DSO_ISLAND_DISABLED    1
#define LSM6DSO_POWERRAIL_DISABLED 1
#endif
