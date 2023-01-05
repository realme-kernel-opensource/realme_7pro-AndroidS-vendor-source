#pragma once
/**
 * @file sns_lsm6ds3c_sensor.h
 *
 * Copyright (c) 2018, STMicroelectronics.
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

#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_lsm6ds3c_hal.h"
#include "sns_math_util.h"
#include "sns_pwr_rail_service.h"
#include "sns_registry_util.h"
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_sync_com_port_service.h"
#include "sns_lsm6ds3c_ver.h"

#define ACCEL_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0xdc, 0x64, 0x08, 0x29, 0x84, 0x03, 0x43, 0xb9,  \
        0xb4, 0x21, 0x6f, 0x39, 0xf7, 0x40, 0xa7, 0xd2  \
      }  \
  }

#define GYRO_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x00, 0x8b, 0xb9, 0xd6, 0x03, 0x45, 0x47, 0x12,  \
        0x88, 0x2f, 0x81, 0x64, 0x7d, 0xdd, 0x85, 0x44  \
      }  \
  }

#define MOTION_DETECT_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x46, 0xee, 0xc6, 0x06, 0x78, 0x5e, 0x46, 0x2b,  \
        0x8f, 0xec, 0xe0, 0xa5, 0x10, 0x0e, 0x78, 0x3a  \
      }  \
  }

#define SENSOR_TEMPERATURE_SUID_0 \
  {  \
    .sensor_uid =  \
      {  \
        0x88, 0x08, 0x1b, 0xf8, 0x1e, 0x00, 0x4f, 0xd0,  \
        0x8d, 0x6b, 0x07, 0x98, 0x2e, 0xa0, 0x41, 0x7c   \
      }  \
  }


#if LSM6DS3C_DUAL_SENSOR_ENABLED
#define ACCEL_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x70, 0x30, 0x96, 0xcc, 0xce, 0x71, 0x49, 0xfa,  \
        0x83, 0x8b, 0xf8, 0xd5, 0x98, 0x02, 0xaf, 0x88  \
      }  \
  }

#define GYRO_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0xd4, 0x5a, 0x44, 0x98, 0x77, 0xbf, 0x43, 0x6d,  \
        0xad, 0xb0, 0xb5, 0xeb, 0x62, 0xf9, 0xf5, 0x20  \
      }  \
  }

#define MOTION_DETECT_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x03, 0x55, 0x1b, 0xc1, 0x2c, 0x04, 0x47, 0xec, \
        0x89, 0xbc, 0x0f, 0x7c, 0x4a, 0xb0, 0x46, 0x5a  \
      }  \
  }
#define SENSOR_TEMPERATURE_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x1b, 0x81, 0x48, 0x2e, 0x05, 0x1f, 0x4d, 0xf9, \
        0x9d, 0xcb, 0x66, 0x0d, 0x6a, 0x25, 0x5b, 0x92  \
      }  \
  }


#endif

#ifndef SUID_IS_NULL
#define SUID_IS_NULL(suid_ptr) ( sns_memcmp( (suid_ptr),                \
                                             &(sns_sensor_uid){{0}},    \
                                             sizeof(sns_sensor_uid) ) == 0 )
#endif

#define NAME   "lsm6ds3c"
#if BUILD_DB
#define VENDOR "template"
#else
#define VENDOR "STMicro"
#endif

#define LSM6DS3C_WHOAMI_VALUE            (0x6A) /** Who Am I default value */
#define ACCEL_LP_MODE_CURRENT           25     //uA
#define ACCEL_NORMAL_MODE_CURRENT       85     //uA
#define ACCEL_HIPRF_MODE_CURRENT        150    //uA
#define ACCEL_SLEEP_CURRENT             3      //uA
#define ESP_ACTIVE_CURRENT              24     //uA
#define GYRO_LP_MODE_CURRENT            270    //uA
#define GYRO_NORMAL_MODE_CURRENT        430    //uA
#define GYRO_HIPRF_MODE_CURRENT         555    //uA
#define GYRO_SLEEP_CURRENT              3      //uA
#define TEMPERATURE_LP_MODE_CURRENT     24     //uA
#define TEMPERATURE_NORMAL_MODE_CURRENT 70     //uA
#define TEMPERATURE_HIPRF_MODE_CURRENT  240    //uA
#define TEMPERATURE_SLEEP_CURRENT       6      //uA
#define TEMPERATURE_BASE                25.0f
#define TEMPERATURE_RESOLUTION          256.0f

/**
 * LSM6DS3C ODR definitions
 */
#define LSM6DS3C_ODR_0                 0.0f
#define LSM6DS3C_ODR_13                13.0f
#define LSM6DS3C_ODR_26                26.0f
#define LSM6DS3C_ODR_52                52.0f
#define LSM6DS3C_ODR_104               104.0f
#define LSM6DS3C_ODR_208               208.0f
#define LSM6DS3C_ODR_416               416.0f
#define LSM6DS3C_ODR_832               832.0f
#define LSM6DS3C_ODR_1664              1664.0f
#define LSM6DS3C_ODR_3328              3328.0f
#define LSM6DS3C_ODR_6656              6656.0f

/**
 * Accelerometer ranges in +/-g unit
 */
#define LSM6DS3C_ACCEL_RANGE_2G_MIN    (-2)
#define LSM6DS3C_ACCEL_RANGE_2G_MAX    (2)
#define LSM6DS3C_ACCEL_RANGE_4G_MIN    (-4)
#define LSM6DS3C_ACCEL_RANGE_4G_MAX    (4)
#define LSM6DS3C_ACCEL_RANGE_8G_MIN    (-8)
#define LSM6DS3C_ACCEL_RANGE_8G_MAX    (8)
#define LSM6DS3C_ACCEL_RANGE_16G_MIN   (-16)
#define LSM6DS3C_ACCEL_RANGE_16G_MAX   (16)

/**
 * Accelerometer resolutions in mg/LSB
 */
#define LSM6DS3C_ACCEL_RESOLUTION_2G    (0.061f)
#define LSM6DS3C_ACCEL_RESOLUTION_4G    (0.122f)
#define LSM6DS3C_ACCEL_RESOLUTION_8G    (0.244f)
#define LSM6DS3C_ACCEL_RESOLUTION_16G   (0.488f)

/**
 * Gyroscope ranges in dps
 */
#define LSM6DS3C_GYRO_RANGE_125_MIN    (-125)
#define LSM6DS3C_GYRO_RANGE_125_MAX    (125)
#define LSM6DS3C_GYRO_RANGE_245_MIN    (-245)
#define LSM6DS3C_GYRO_RANGE_245_MAX    (245)
#define LSM6DS3C_GYRO_RANGE_500_MIN    (-500)
#define LSM6DS3C_GYRO_RANGE_500_MAX    (500)
#define LSM6DS3C_GYRO_RANGE_1000_MIN   (-1000)
#define LSM6DS3C_GYRO_RANGE_1000_MAX   (1000)
#define LSM6DS3C_GYRO_RANGE_2000_MIN   (-2000)
#define LSM6DS3C_GYRO_RANGE_2000_MAX   (2000)

/**
 * LSM6DS3C sensitivity for gyro in dps/LSB
 */
#define LSM6DS3C_GYRO_SSTVT_125DPS   (0.004375f)
#define LSM6DS3C_GYRO_SSTVT_245DPS   (0.008750f)
#define LSM6DS3C_GYRO_SSTVT_500DPS   (0.017500f)
#define LSM6DS3C_GYRO_SSTVT_1000DPS  (0.035000f)
#define LSM6DS3C_GYRO_SSTVT_2000DPS  (0.070000f)

/**
 * LSM6DS3C Sensor Temperature ODR (Hz) definitions
 */
#define LSM6DS3C_SENSOR_TEMP_ODR_1        1.0f
#define LSM6DS3C_SENSOR_TEMP_ODR_5        5.0f

/**
 * Sensor Temprature resolution in deg Celsius/LSB
 * 1/16 deg Celsius per LSB
 */
#define LSM6DS3C_SENSOR_TEMPERATURE_RESOLUTION  (0.0039f)

/**
 * Sensor Temperature range in deg Celsius
 */
#define LSM6DS3C_SENSOR_TEMPERATURE_RANGE_MIN  (-40.0f)
#define LSM6DS3C_SENSOR_TEMPERATURE_RANGE_MAX  (85.0f)


/** Supported opertating modes */
#define LSM6DS3C_LPM          "LPM"
#define LSM6DS3C_HIGH_PERF    "HIGH_PERF"
#define LSM6DS3C_NORMAL       "NORMAL"

typedef enum
{
  ATTR_AVAILABLE = 0,
  ATTR_NAME,
  ATTR_DATA_TYPE,
  ATTR_VENDOR,
  ATTR_VERSION,
  ATTR_RATES,
  ATTR_RESOLUTION,
  ATTR_FIFO_SIZE,
  ATTR_ACTIVE_CURRENT,
  ATTR_SLEEP_CURRENT,
  ATTR_RANGES,
  ATTR_OP_MODES,
  ATTR_API,
  ATTR_EVENT_SIZE,
  ATTR_STREAM_TYPE,
  ATTR_IS_DYNAMIC,
  ATTR_RIGID_BODY,
  ATTR_PLACEMENT,
  ATTR_HW_ID,
  ATTR_DRI_SUPPORT,
  ATTR_SYNC_STREAM_SUPPORT,
  ATTR__MAX,
}lsm6ds3c_attrib;

#if !LSM6DS3C_POWERRAIL_DISABLED
/** Power rail timeout States for the LSM6DS3C Sensors.*/
#define LSM6DS3C_POWER_RAIL_OFF_TIMEOUT_NS 1000000000ULL /* 1 second */
typedef enum
{
  LSM6DS3C_POWER_RAIL_PENDING_NONE,
  LSM6DS3C_POWER_RAIL_PENDING_INIT,
  LSM6DS3C_POWER_RAIL_PENDING_SET_CLIENT_REQ,
  LSM6DS3C_POWER_RAIL_PENDING_OFF,
} lsm6ds3c_power_rail_pending_state;
#endif

#if LSM6DS3C_ESP_ENABLED
typedef struct lsm6ds3c_esp_shared_info
{
  uint32_t    stepCount_base;
#if LSM6DS3C_OEM_FACTORY_CONFIG
  bool        oem_config_on:1;
#endif
} lsm6ds3c_esp_shared_info;
#endif

/** State shared by all LSM6DS3C sensors. */
typedef struct lsm6ds3c_shared_state
{
#if LSM6DS3C_DAE_ENABLED
  sns_data_stream                   *dae_stream;
#endif
  sns_data_stream                   *suid_stream;
  sns_data_stream                   *timer_stream;
  sns_sync_com_port_service         *scp_service;
  sns_pwr_rail_service              *pwr_rail_service;
#if !LSM6DS3C_POWERRAIL_DISABLED
  sns_rail_config                   rail_config;
  sns_power_rail_state              registry_rail_on_state;
  lsm6ds3c_power_rail_pending_state  power_rail_pend_state;
#endif
  float                             placement[12];
  uint16_t                          who_am_i;
  uint8_t                           rigid_body_type;
  bool                              hw_is_present:1;
  bool                              irq2_enabled:1;
  uint8_t                           hw_idx;
  lsm6ds3c_instance_config           inst_cfg;
#if LSM6DS3C_ESP_ENABLED
  lsm6ds3c_esp_shared_info           esp_shared_info;
#endif
} lsm6ds3c_shared_state;

typedef struct lsm6ds3c_combined_request
{
  float                   sample_rate;
  float                   report_rate;
  float                   dae_report_rate;
  uint64_t                flush_period_ticks;
  uint32_t                report_per_us;
  uint32_t                dae_report_per_us;
  bool                    max_batch:1;
  bool                    flush_only:1;
  bool                    ngated_client_present:1;
  bool                    gated_client_present:1;
} lsm6ds3c_combined_request;

typedef struct lsm6ds3c_state
{
  int64_t                 hardware_id;
  sns_data_stream         *reg_data_stream;

  size_t                  encoded_event_len;
  sns_sensor_uid          my_suid;
  lsm6ds3c_sensor_type     sensor;
  int8_t                  outstanding_reg_requests;
  bool                    available:1;
  bool                    is_dri:1;
  bool                    supports_sync_stream:1;

  // consolidated from all IMU requests
  lsm6ds3c_combined_request combined_imu;

} lsm6ds3c_state;


#define MAX_SUPPORTED_SENSORS 4
typedef struct {
  lsm6ds3c_sensor_type     sensor;
  size_t                  state_size;
  sns_sensor_uid const    *suid;
  sns_sensor_api          *sensor_api;
  sns_sensor_instance_api *instance_api;
} lsm6ds3c_sensors;


/** Global const tables */
extern const lsm6ds3c_gyro_range lsm6ds3c_gyro_ranges[];
extern const lsm6ds3c_accel_range lsm6ds3c_accel_ranges[];
extern const float lsm6ds3c_accel_resolutions[];
extern const float lsm6ds3c_gyro_resolutions[];
extern const float lsm6ds3c_accel_range_min[];
extern const float lsm6ds3c_accel_range_max[];
extern const float lsm6ds3c_gyro_range_min[];
extern const float lsm6ds3c_gyro_range_max[];
extern const lsm6ds3c_sensors lsm6ds3c_supported_sensors[MAX_SUPPORTED_SENSORS];


/** Functions shared by all LSM6DS3C Sensors */
/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 * @param[i] sensor_type   Sensor type
 *
 * @return none
 */
void lsm6ds3c_set_client_config(sns_sensor *this,
                               sns_sensor_instance *instance,
                               lsm6ds3c_shared_state *shared_state);

/**
 * set_client_request() Sensor API common between all LSM6DS3C
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
sns_sensor_instance* lsm6ds3c_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove);

/**
 * Publishes default Sensor attributes.
 *
 * @param this   Sensor reference
 *
 * @return none
 */
void lsm6ds3c_publish_def_attributes(sns_sensor *const this);
void lsm6ds3c_init_sensor_info(sns_sensor *const this,
                              sns_sensor_uid const *suid,
                              lsm6ds3c_sensor_type sensor_type);
void lsm6ds3c_process_suid_events(sns_sensor *const this);
void lsm6ds3c_handle_selftest_request_removal(sns_sensor *const this,
                                             sns_sensor_instance *const instance,
                                             lsm6ds3c_shared_state *shared_state);
void lsm6ds3c_process_registry_events(sns_sensor *const this);
sns_rc lsm6ds3c_discover_hw(sns_sensor *const this);
void lsm6ds3c_update_siblings(sns_sensor *const this, lsm6ds3c_shared_state *shared_state);
sns_sensor_uid const* lsm6ds3c_get_sensor_uid(sns_sensor const *const this);
bool lsm6ds3c_sensor_write_output_to_registry(sns_sensor*);

sns_rc lsm6ds3c_accel_init(sns_sensor *const this);
sns_rc lsm6ds3c_gyro_init(sns_sensor *const this);
sns_rc lsm6ds3c_motion_detect_init(sns_sensor *const this);
sns_rc lsm6ds3c_sensor_temp_init(sns_sensor *const this);
sns_rc lsm6ds3c_accel_deinit(sns_sensor *const this);
sns_rc lsm6ds3c_gyro_deinit(sns_sensor *const this);
sns_rc lsm6ds3c_motion_detect_deinit(sns_sensor *const this);
sns_rc lsm6ds3c_sensor_temp_deinit(sns_sensor *const this);
sns_sensor* lsm6ds3c_get_sensor_by_type(sns_sensor *const this, lsm6ds3c_sensor_type sensor);
sns_sensor* lsm6ds3c_get_master_sensor(sns_sensor *const this);
lsm6ds3c_shared_state* lsm6ds3c_get_shared_state(sns_sensor *const this);
lsm6ds3c_shared_state* lsm6ds3c_get_shared_state_from_state(sns_sensor_state const *state);
lsm6ds3c_instance_config const* lsm6ds3c_get_instance_config(sns_sensor_state const *state);
void lsm6ds3c_update_rail_vote(sns_sensor *this,
                              lsm6ds3c_shared_state* shared_state,
                              sns_power_rail_state vote,
                              sns_time *on_timestamp);
void sns_lsm6ds3c_registry_def_config(sns_sensor *const this,
                                     sns_registry_phy_sensor_pf_cfg *cfg);
void lsm6ds3c_send_registry_request(sns_sensor *const this, char *reg_group_name);

