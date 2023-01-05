#pragma once
/**
 * @file sns_lps22hx_sensor_instance.h
 *
 * Copyright (c) 2019, STMicroelectronics.
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
 **/

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_time.h"
#include "sns_com_port_types.h"
#include "sns_sensor_uid.h"
#include "sns_async_com_port.pb.h"
#include "sns_interrupt.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_pressure.pb.h"
#include "sns_sensor_temperature.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_printf.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"

/** Forward Declaration of Instance API */
sns_sensor_instance_api lps22hx_sensor_instance_api;

/** Number of registers to read for debug */
#define LPS22HX_DEBUG_REGISTERS          (33)

/** Number of entries in reg_map table. */
#define LPS22HX_REG_MAP_TABLE_SIZE       (5)
#define BUILD_DB         0
#define LPS22HX_DUMP_REG 0
#define LPS22HX_LOG_VERBOSE 1
#define LPS22HX_LOG_DATA 0
#define LPS22HX_POLLING 1
#define LPS22HX_ENABLE_LOGGING 1

#define LPS22HX_ENABLE_LPF 0

/* Select sensor to build */
//#define ENABLE_LPS22HB 1
#define ENABLE_LPS22HH 1

/**Set 1 if want to read through registry */
#define LPS22HX_ENABLE_REGISTRY_ACCESS  1

#if LPS22HX_ENABLE_LPF
#define NUM_SAMPLE_TO_DISCARD 2
#else
#define NUM_SAMPLE_TO_DISCARD 0
#endif

#if defined(ENABLE_LPS22HB) && defined(ENABLE_LPS22HH)
#error "Select only one sensor(LPS22HB or LPS22HH)"
#endif
#if defined(ENABLE_LPS22HB)
#define LPS22HX_DEVICE_LPS22HB 1
#elif defined(ENABLE_LPS22HH)
#define LPS22HX_DEVICE_LPS22HB 0
#elif !defined(ENABLE_LPS22HB) && !defined(ENABLE_LPS22HH)
#error "Please select one sensor(LPS22HB or LPS22HH)"
#endif

#if LPS22HX_LOG_VERBOSE
#define LPS22HX_LOG_VERBOSE_DEFAULT 1
#define DBG_PRINT_EX(diag, sensor, prio, file, line, ...) do { \
  (void)diag; \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)
#define DBG_INST_PRINT_EX(inst, prio, file, line, ...) do { \
  SNS_INST_PRINTF(prio, inst , __VA_ARGS__); \
} while (0)
#else
#define DBG_PRINT_EX(sensor,...) UNUSED_VAR(sensor);
#define DBG_INST_PRINT_EX(inst,...) UNUSED_VAR(inst);
#endif
#if LPS22HX_LOG_VERBOSE_DEFAULT
#define DBG_PRINT(diag, sensor, prio, file, line, ...) do { \
  (void)diag; \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)
#define DBG_INST_PRINT(inst, prio, file, line, ...) do { \
  SNS_INST_PRINTF(prio, inst , __VA_ARGS__); \
} while (0)
#else
#define DBG_PRINT(sensor,...) UNUSED_VAR(sensor);
#define DBG_INST_PRINT(inst,...) UNUSED_VAR(inst);
#endif

/**
 * Pressure sensor output data rate in register setting
 */
typedef enum
{
  LPS22HX_SENSOR_ODR_OFF   = 0x00,  /* power down output data rate */
  LPS22HX_SENSOR_ODR1      = 0x01,  /* 1  Hz output data rate */
  LPS22HX_SENSOR_ODR10     = 0x02,  /* 10 Hz output data rate */
  LPS22HX_SENSOR_ODR25     = 0x03,  /* 25 Hz output data rate */
  //LPS22HX_SENSOR_ODR50   = 0x04,  /* 50 Hz output data rate */
  //LPS22HX_SENSOR_ODR75   = 0x05,  /* 75 Hz output data rate */
} lps22hx_sensor_odr;

typedef struct lps22hx_com_port_info
{
  sns_com_port_config          com_config;
  sns_sync_com_port_handle     *port_handle;

} lps22hx_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr {
  float min;
  float max;
} range_attr;

typedef enum
{
  LPS22HX_PRESS         = 0x1,
  LPS22HX_SENSOR_TEMP   = 0x2
} lps22hx_sensor_type;

typedef struct lps22hx_press_info
{
  uint8_t                 num_samples_to_discard;
  bool                    lp_mode;
  bool                    self_test_is_successful;
  bool                    timer_is_active;
  lps22hx_sensor_odr      current_odr;
  sns_time                sample_period;
  sns_sensor_uid          suid;

  float                   offset;
} lps22hx_press_info;

typedef struct lps22hx_temp_info
{
  lps22hx_sensor_odr      current_odr;
  bool                    timer_is_active;
  bool                    lp_mode;
  sns_sensor_uid          suid;
  uint8_t                 num_samples_to_discard;
  bool                    self_test_is_successful;
} lps22hx_temp_info;

typedef struct lps22hx_self_test_info
{
  lps22hx_sensor_type sensor;
  sns_physical_sensor_test_type test_type;
  bool test_alive;
} lps22hx_self_test_info;

#if !(LPS22HX_POLLING)
typedef struct lps22hx_irq_info
{
  sns_interrupt_req       irq_config;
  bool                    irq_ready;
} lps22hx_irq_info;
#endif

typedef struct lps22hx_async_com_port_info
{
  uint32_t                port_handle;
}lps22hx_async_com_port_info;

/** Private state. */
typedef struct lps22hx_instance_state
{
  /** pressure HW config details*/
  lps22hx_press_info      press_info;

  /** temp HW config details*/
  lps22hx_temp_info       temp_info;

#if !(LPS22HX_POLLING)
  /** Interrupt dependency info. */
  lps22hx_irq_info        irq_info;
#endif

  /** COM port info */
  lps22hx_com_port_info   com_port_info;

  /**--------Async Com Port--------*/
  sns_async_com_port_config  ascp_config;

  /** detail about self test params*/
  lps22hx_self_test_info  self_test_info;
  bool                 self_test_enabled;

  sns_time             interrupt_timestamp;
  sns_time 	           last_sample_ts;
  sns_time             new_odr_irq_ts;      // first valid irq_timestamp after setting new odr.

  /** Data streams from dependentcies. */
  sns_data_stream      *interrupt_data_stream;
  sns_data_stream      *timer_data_stream;
  sns_data_stream      *async_com_port_data_stream;

  uint32_t                     client_req_id;
  sns_std_sensor_config        std_sensor_req;

  /** Determines which Sensor data to publish. Uses
   *  lps22hx_sensor_type as bit mask. */
  uint8_t              publish_sensors;

  /** Determines which Sensors are enabled */
  uint8_t              enabled_sensors;

  uint8_t              ctrl_reg1; // mirror register for LPS22HX CTRL_REG1
  size_t               encoded_press_event_len;
  size_t               encoded_sensor_temp_event_len;

  /**----------debug----------*/
  float     p_stream_event;
  float     t_stream_event;
  uint8_t   reg_status[LPS22HX_DEBUG_REGISTERS];

  sns_diag_service *diag_service;
  sns_sync_com_port_service * scp_service;
  size_t           log_interrupt_encoded_size;
  bool instance_is_ready_to_configure;
  size_t           log_raw_encoded_size;
  lps22hx_sensor_type curr_sensor;
  sns_sensor_uid timer_suid;
} lps22hx_instance_state;

typedef struct sns_lps22hx_config_req
{
  float sample_rate;
  lps22hx_sensor_type sensor_type;
} sns_lps22hx_config_req;

typedef struct sns_lps22hx_self_test_req
{
  sns_physical_sensor_test_type test_type;
} sns_lps22hx_self_test_req;

typedef struct odr_reg_map
{
  float              odr;
  lps22hx_sensor_odr odr_reg_value;
  uint16_t           press_discard_samples;
} odr_reg_map;

sns_rc lps22hx_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate);

sns_rc lps22hx_inst_deinit(sns_sensor_instance *const this);
void lps22hx_inst_set_oem_client_config(sns_sensor_instance *const this);
bool lps22hx_set_oem_client_request(sns_sensor *const this,
    struct sns_request const *exist_request);
bool lps22hx_set_oem_client_request_msgid(sns_sensor *const this,
    struct sns_request const *new_request);
void lps22hx_set_client_test_config(sns_sensor_instance *this);
