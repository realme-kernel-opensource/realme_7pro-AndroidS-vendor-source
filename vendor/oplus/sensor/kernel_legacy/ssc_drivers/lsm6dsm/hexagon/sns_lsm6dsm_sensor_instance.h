#pragma once
/**
 * @file sns_lsm6dsm_sensor_instance.h
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

#include <stdint.h>

#include "sns_lsm6dsm_build_config.h"
#include "sns_async_com_port.pb.h"
#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#include "sns_lsm6dsm_dae_if.h"
#include "sns_math_util.h"
#include "sns_motion_detect.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_printf.h"
#include "sns_registry_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_uid.h"
#include "sns_std_sensor.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include "sns_timer.pb.h"
#include "sns_lsm6dsm_esp.h"

/** Forward Declaration of Instance API */
extern sns_sensor_instance_api lsm6dsm_sensor_instance_api;

/** Number of registers to read for debug */
#define LSM6DSM_DEBUG_REGISTERS          (32)

#define SELF_TEST_DATA_COUNT_MAX         (25.0f)

#define LSM6DSM_ACCEL_RANGE_HW_SELFTEST LSM6DSM_ACCEL_RANGE_2G
#define LSM6DSM_ACCEL_SSTVT_HW_SELFTEST LSM6DSM_ACCEL_SSTVT_2G
//QC change to enable MD only
#define LSM6DSM_MD_SAMPLE_RATE    LSM6DSM_ODR_13            //Default sample rate for MD
#define LSM6DSM_MD_REPORT_PERIOD  (1000 *1000) //Default report  period for MD in secs conv to msec
#define MAX_INTERRUPT_CNT  2

/**
 * Accelerometer LSM6DSM_ACC Full Scales in register setting.
 */
typedef enum
{
  LSM6DSM_ACCEL_RANGE_2G   = 0x00,  /*corresponding value in register setting*/
  LSM6DSM_ACCEL_RANGE_4G   = 0x08,
  LSM6DSM_ACCEL_RANGE_8G   = 0x0C,
  LSM6DSM_ACCEL_RANGE_16G   = 0x04,
} lsm6dsm_accel_range;

/**
 * Accelerometer LSM6DSM_ACC sensitivity for each range.
 */
typedef enum
{
  LSM6DSM_ACCEL_SSTVT_2G  = 61,   /* in the unit of micro-g/digit */
  LSM6DSM_ACCEL_SSTVT_4G  = 122,
  LSM6DSM_ACCEL_SSTVT_8G  = 244,
  LSM6DSM_ACCEL_SSTVT_16G = 488,
} lsm6dsm_accel_sstvt;

/**
 * Accelerometer LSM6DSM ACC filter bandwidth in register setting
 */
typedef enum
{
  LSM6DSM_ACCEL_BW0       = 0x00,  /* 00 Hz bandwidth */
  LSM6DSM_ACCEL_BW13      = 0x00,  /* 13 Hz bandwidth */
  LSM6DSM_ACCEL_BW26      = 0x00,  /* 26 Hz bandwidth */
  LSM6DSM_ACCEL_BW50      = 0x00,  /* 50 Hz bandwidth */
  LSM6DSM_ACCEL_BW100     = 0x00,  /* 100 Hz bandwidth */
  LSM6DSM_ACCEL_BW200     = 0x00,  /* 200 Hz bandwidth */
  LSM6DSM_ACCEL_BW400     = 0x00,  /* 400 Hz bandwidth */
  LSM6DSM_ACCEL_BW800     = 0x00,  /* 800 Hz bandwidth */
  LSM6DSM_ACCEL_BW1600    = 0x01,  /* 1600 Hz bandwidth */
  LSM6DSM_ACCEL_BW3300    = 0x01,  /* 3300 Hz bandwidth */
  LSM6DSM_ACCEL_BW6600    = 0x01   /* 6600 Hz bandwidth */
} lsm6dsm_accel_bw;

/**
 * Accelerometer LSM6DSM_ACC output data rate in register setting
 */
typedef enum
{
  LSM6DSM_ACCEL_ODR_OFF   = 0x00,  /* power down output data rate */
  LSM6DSM_ACCEL_ODR13     = 0x10,  /* 13 Hz output data rate */
  LSM6DSM_ACCEL_ODR26     = 0x20,  /* 26 Hz output data rate */
  LSM6DSM_ACCEL_ODR52     = 0x30,  /* 52 Hz output data rate */
  LSM6DSM_ACCEL_ODR104    = 0x40,  /* 104 Hz output data rate */
  LSM6DSM_ACCEL_ODR208    = 0x50,  /* 208 Hz output data rate */
  LSM6DSM_ACCEL_ODR416    = 0x60,  /* 416 Hz output data rate */
  LSM6DSM_ACCEL_ODR832    = 0x70,  /* 832 Hz output data rate */
  LSM6DSM_ACCEL_ODR1664   = 0x80,  /* 1.66 kHz output data rate */
  LSM6DSM_ACCEL_ODR3328   = 0x90,  /* 3.33 kHz output data rate */
  LSM6DSM_ACCEL_ODR6656   = 0xA0,  /* 6.66 kHz output data rate */
} lsm6dsm_accel_odr;

/**
 * LSM6DSM output data rate for gyro, Disabling LPF2,
 * so BW setting is not required
 */
typedef enum
{
  LSM6DSM_GYRO_ODR_OFF = 0x00,       /* power down output data rate */
  LSM6DSM_GYRO_ODR13   = 0x10,       /* 13 Hz output data rate */
  LSM6DSM_GYRO_ODR26   = 0x20,       /* 26 Hz output data rate */
  LSM6DSM_GYRO_ODR52   = 0x30,       /* 52 Hz output data rate */
  LSM6DSM_GYRO_ODR104  = 0x40,       /* 104 Hz output data rate */
  LSM6DSM_GYRO_ODR208  = 0x50,       /* 208 Hz output data rate */
  LSM6DSM_GYRO_ODR416  = 0x60,       /* 416 Hz output data rate */
  LSM6DSM_GYRO_ODR832  = 0x70,       /* 832 Hz output data rate */
  LSM6DSM_GYRO_ODR1664 = 0x80,       /* 1.66 kHz output data rate */
} lsm6dsm_gyro_odr;

/**
 * LSM6DSM Full Scales in register setting for gyro
 */
typedef enum
{
  STM_LSM6DSM_GYRO_RANGE_125DPS   = 0x02,
  STM_LSM6DSM_GYRO_RANGE_245DPS   = 0x00,  /*corresponding value in register setting*/
  STM_LSM6DSM_GYRO_RANGE_500DPS   = 0x04,
  STM_LSM6DSM_GYRO_RANGE_1000DPS  = 0x08,
  STM_LSM6DSM_GYRO_RANGE_2000DPS  = 0x0C,
} lsm6dsm_gyro_range;

typedef float lsm6dsm_gyro_sstvt;

typedef struct lsm6dsm_com_port_info
{
  sns_com_port_config          com_config;
  sns_sync_com_port_handle     *port_handle;
  uint8_t                      i2c_address;
} lsm6dsm_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr {
  float min;
  float max;
} range_attr;

typedef enum {
  FILTER_DEFAULT ,
  SLOPE_FILTER ,
  HP_FILTER,
}lsm6dsm_md_filter_type;
typedef enum {
  DISABLED, //md disabled
  ENABLED_INT, //md enabled with interrupt
  ENABLED, //md enabled, no interrupt
}lsm6dsm_md_state;
typedef enum
{
  LSM6DSM_ACCEL         = 0x01,
  LSM6DSM_GYRO          = 0x02,
  LSM6DSM_MOTION_DETECT = 0x04,
  LSM6DSM_SENSOR_TEMP   = 0x08,
  LSM6DSM_STEP_COUNTER  = 0x10,
  LSM6DSM_FREE_FALL     = 0x20,
  LSM6DSM_HIGH_SHOCK    = 0x40,
  LSM6DSM_ACTIVITY      = 0x100,
  LSM6DSM_INACTIVITY    = 0x200,
} lsm6dsm_sensor_type;

//only for accel and gyro
typedef enum
{
  POLLING,
  DRI,
} lsm6dsm_stream_mode;

#if LSM6DSM_DAE_ENABLED
typedef enum
{
  LSM6DSM_CONFIG_IDLE,            /** not configuring */
  LSM6DSM_CONFIG_POWERING_DOWN,   /** cleaning up when no clients left */
  LSM6DSM_CONFIG_STOPPING_STREAM, /** stream stop initiated, waiting for completion */
  LSM6DSM_CONFIG_FLUSHING_HW,     /** FIFO flush initiated, waiting for completion */
  LSM6DSM_CONFIG_FLUSHING_DATA,   /** FIFO flush initiated, waiting for completion */
  LSM6DSM_CONFIG_UPDATING_HW      /** updating sensor HW, when done goes back to IDLE */
} lsm6dsm_config_step;
#endif

// QC - Fields in structures should be sorted by size to optimize code space

/*contains fifo reading req information
 * either interrupt or flush */

 typedef struct lsm6dsm_fifo_req
 {
  bool              interrupt_fired;
  bool              recheck_int;
  bool              flush_req;
  //sns_time  sampling_intvl;
  uint8_t        freq_drift;
  sns_time          cur_time;
  uint16_t          wmk;
  sns_time          interrupt_ts;
  lsm6dsm_accel_odr accel_odr;
 }lsm6dsm_fifo_req;

typedef struct lsm6dsm_config_event_info
{
  float     sample_rate;
  uint32_t  fifo_watermark;
  sns_time  timestamp;
  sns_time  md_timestamp;
  sns_time  gated_timestamp;
#if LSM6DSM_DAE_ENABLED
  uint32_t  dae_watermark;
#endif
} lsm6dsm_config_event_info;

/** HW FIFO information */
typedef struct lsm6dsm_fifo_info
{
  /** is fifo reconfiguration is req for the new config req*/
  bool reconfig_req;

  /** is full fifo reconfiguration is req for the new config req*/
  bool full_reconf_req;

  /** reconfig time active */
  bool timer_active;

  bool                       update_registry;
  /** is fifo data pushing to clients */
  bool                       is_streaming;
  /** is last timestamp valid for calculating sample time*/
  bool                       last_ts_valid;

  /** FIFO enabled or not. Uses lsm6dsm_sensor_type as bit mask
   *  to determine which FIFO Sensors are enabled */
  uint8_t fifo_enabled;

  uint8_t int1_ctrl_reg;

  /** Determines which Sensor data to publish. Uses
   *  lsm6dsm_sensor_type as bit mask. */
  uint8_t publish_sensors;

  /** instance publish sensors */
  uint8_t inst_publish_sensors;

  int8_t                     avg_to_nominal_ratio_cnt;
  int8_t                     avg_to_nominal_ratio_cnt_g;

  /** desired FIFO watermark levels for accel and gyro*/
  uint16_t desired_wmk;

  /** FIFO watermark levels for accel and gyro*/
  uint16_t cur_wmk;

  /** number of interrupts fired without reconfig*/
  uint16_t                   interrupt_cnt;

  /** fifo cur rate index */
  lsm6dsm_accel_odr fifo_rate;

  /** fifo desired rate index */
  lsm6dsm_accel_odr desired_fifo_rate;

  /** max requested FIFO watermark levels; possibly larger than max HW FIFO */
  uint32_t max_requested_wmk;

  /** avg interrupt interval without reconfiguring*/
  uint32_t                   avg_interrupt_intvl;
  /** interrupt thresholds - average interrupt interval must be recalculated if outside this window*/
  uint32_t                   interrupt_intvl_upper_bound;
  uint32_t                   interrupt_intvl_lower_bound;
  /** avg sampling interval without reconfiguring*/
  uint32_t                   avg_sampling_intvl;
  /** sampling interval calculated from ODR */
  uint32_t                   nominal_sampling_intvl;
  uint32_t                   nominal_dae_intvl;
  uint32_t                   cur_sampling_intvl;
  float                      avg_to_nominal_ratio;
  float                      avg_to_nominal_ratio_g;
  float                      accel_ratio;
  float                      gyro_ratio;
  /** max flush ticks*/
  uint64_t max_requested_flush_ticks;

  /** timestamp of last sample sent to framework*/
  sns_time                   last_timestamp;
  /** ascp event timestamp
   * ascp fills timestamp once reading completes */
  sns_time                   ascp_event_timestamp;
  /** timestamp when fifo interrupt fired*/
  sns_time                   interrupt_timestamp;
  /*contains info before sending ascp req*/
  lsm6dsm_fifo_req th_info;
  /*contains info to use after ascp returns */
  lsm6dsm_fifo_req bh_info;

  lsm6dsm_config_event_info last_sent_config;
  lsm6dsm_config_event_info new_config;

} lsm6dsm_fifo_info;

typedef enum
{
  LSM6DSM_MODE_POLLING    = 0x1,
  LSM6DSM_MODE_FIFO       = 0x2,
  LSM6DSM_MODE_SELF_TEST  = 0x4
} lsm6dsm_streaming_mode;

typedef enum
{
  LSM6DSM_SELF_TEST_STAGE_1    = 0x1,
  LSM6DSM_SELF_TEST_STAGE_2    = 0x2,
  LSM6DSM_SELF_TEST_STAGE_3    = 0x3,
  LSM6DSM_SELF_TEST_STAGE_4    = 0x4,
  LSM6DSM_SELF_TEST_STAGE_5    = 0x5,
} lsm6dsm_self_test_stage;

typedef enum
{
  FLUSH_TO_BE_DONE,           // 0
  FLUSH_DONE_CONFIGURING,     // 1
  FLUSH_DONE_NOT_ACCEL_GYRO,  // 2
  FLUSH_DONE_NOT_FIFO,        // 3
  FLUSH_DONE_FIFO_EMPTY,      // 4
  FLUSH_DONE_AFTER_DATA,      // 5
} lsm6dsm_flush_done_reason;

typedef struct lsm6dsm_common_info
{
  uint8_t mode;
  //add few more hre
  lsm6dsm_sensor_type odr_changed;
  lsm6dsm_accel_odr accel_curr_odr;
  lsm6dsm_gyro_odr gyro_curr_odr;
  sns_time  accel_odr_settime;
  sns_time  gyro_odr_settime;
} lsm6dsm_common_info;

typedef struct lsm6dsm_self_test_info
{
  bool test_alive;
  bool reconfig_postpone;
  bool update_registry;
#if LSM6DSM_OEM_FACTORY_CONFIG
  bool return_now;
#endif
  uint8_t odr_idx;
  uint8_t skip_count;
  uint16_t  polling_count;
  lsm6dsm_self_test_stage  self_test_stage;
  int64_t cumulative_data_pre[TRIAXIS_NUM];
  int64_t cumulative_data_post[TRIAXIS_NUM];
  lsm6dsm_sensor_type sensor;
  lsm6dsm_accel_odr curr_odr;
  sns_physical_sensor_test_type test_type;
  //add few more hre
} lsm6dsm_self_test_info;

typedef struct lsm6dsm_accel_info
{
  bool                    lp_mode;
  bool                    gated_client_present;
  uint8_t                 range_idx;
  uint8_t                 opdata_raw[6];
  uint16_t                num_samples_to_discard;
  lsm6dsm_accel_odr desired_odr;
  lsm6dsm_accel_sstvt     sstvt;
  lsm6dsm_accel_range     range;
  lsm6dsm_accel_bw        bw;
  sns_std_sensor_sample_status opdata_status;
  sns_sensor_uid          suid;
} lsm6dsm_accel_info;

typedef struct lsm6dsm_gyro_info
{
  bool                    is_in_sleep;
  uint8_t                 range_idx;
  uint8_t                 opdata_raw[6];
  uint16_t                num_samples_to_discard;
  lsm6dsm_gyro_odr desired_odr;
  lsm6dsm_gyro_sstvt      sstvt;
  float                   sstvt_adj[3];
  lsm6dsm_gyro_range      range;
  sns_std_sensor_sample_status opdata_status;
  sns_sensor_uid          suid;
} lsm6dsm_gyro_info;

typedef struct lsm6dsm_motion_detect_info
{
  bool                    is_filter_settled;
  bool                    is_timer_running;
  uint16_t                desired_wmk;
  /*denotes if external client is present or not */
  uint16_t                client_present;
  lsm6dsm_accel_odr       desired_odr;
  lsm6dsm_accel_sstvt     sstvt;
  lsm6dsm_accel_range     range;
  lsm6dsm_accel_bw        bw;
  /* some features depends on md, so represents
   * useful while disabling or re-enabling md */
  lsm6dsm_sensor_type     internal_client_present;
  sns_motion_detect_event cur_state;
  lsm6dsm_md_filter_type  filter;
  sns_registry_md_cfg     md_config;
  sns_time                event_ts;
  sns_sensor_uid          suid;
} lsm6dsm_motion_detect_info;

typedef struct lsm6dsm_sensor_temp_info
{
  bool                    timer_is_active;
  bool                    self_test_is_successful;
  float                   report_rate_hz;
  float                   desired_sampling_rate_hz;
  float                   cur_sampling_rate_hz;
  sns_time                sampling_intvl;
  uint64_t                max_requested_flush_ticks;
  lsm6dsm_config_event_info last_sent_config;
  lsm6dsm_config_event_info new_config;
  sns_sensor_uid          suid;
} lsm6dsm_sensor_temp_info;

typedef struct lsm6dsm_irq_info
{
  bool                    irq_registered:1;
  bool                    irq_ready:1;
  sns_interrupt_req       irq_config;
} lsm6dsm_irq_info;

typedef struct lsm6dsm_async_com_port_info
{
  uint32_t                port_handle;
}lsm6dsm_async_com_port_info;

typedef struct sns_lsm6dsm_registry_cfg
{
  sns_time            ts;
  lsm6dsm_sensor_type sensor_type;
  uint32_t            registry_persist_version;
  uint32_t            registry_instance_version;
  float               fac_cal_bias[3];
  float               registry_nom_val;
  matrix3             fac_cal_corr_mat;
}sns_lsm6dsm_registry_cfg;

typedef struct lsm6dsm_hw_config
{
  //defines whether HW MD feature needed or not
  /* set based on gated/non-gated requests/
   * client present and internal client present */
  bool md_enabled;
  //decides which sensors are enabled
  uint8_t enabled_sensors;
  // index into odr map table
  uint8_t odr_idx;
  uint8_t last_odr_idx;
  //value of a/g odr in HZ
  uint16_t odr;
  //fifo odr - if set fifo mode, polling
  uint16_t fifo_odr;
  //set wmk
  uint16_t wmk;

} lsm6dsm_hw_config;

typedef struct sns_lsm6dsm_registry_reset
{
  bool            request;
  lsm6dsm_sensor_type sensor_type;
}sns_lsm6dsm_registry_reset;

typedef struct lsm6dsm_health
{
  bool heart_attack;
  bool heart_attack_flush;
  uint8_t heart_attack_cnt;
  sns_time    expected_expiration;
  uint64_t    heart_beat_timeout;
} lsm6dsm_health;

/** Private state. */
typedef struct lsm6dsm_instance_state
{
  bool                    irq_ready;
  bool                    irq2_inst_enabled;
  bool                    int_enabled; //enable H/W interrupts bit

  /** ESP info */
  bool                    route_md_to_irq2;

#if LSM6DSM_AUTO_DEBUG
  bool missingSamples;
#endif

  /** hw index */
  uint8_t hw_idx;

  /** rigid body type */
  uint8_t rigid_body_type;

  /** which entry in lsm6dsm_odr_map[] to use for min ODR */
  uint8_t min_odr_idx;

  uint8_t *fifo_start_address;

  uint8_t              wake_src;
#if LSM6DSM_DEBUG_CRASH
  uint8_t    debug_status_reg[4];
#endif
  uint8_t   reg_status[LSM6DSM_DEBUG_REGISTERS];
  lsm6dsm_stream_mode   ag_stream_mode;

  /** detail about self test params*/
  lsm6dsm_self_test_info self_test_info;

  /** Debug counter for sample count */
  uint32_t accel_sample_counter;

  /** Debug counter for sample count */
  uint32_t gyro_sample_counter;

  /** which sensors are being flushed */
  lsm6dsm_sensor_type flushing_sensors;

  /** which sensors are being (re)configured */
  lsm6dsm_sensor_type config_sensors;

  /** detail about common params*/
  lsm6dsm_common_info common_info;

  /** fifo details*/
  lsm6dsm_fifo_info       fifo_info;

  /** accel HW config details*/
  lsm6dsm_accel_info      accel_info;

  /** gyro HW config details*/
  lsm6dsm_gyro_info       gyro_info;

  /** motion detect info */
  lsm6dsm_motion_detect_info md_info;

  /** Sensor Temperature config details. */
  lsm6dsm_sensor_temp_info sensor_temp_info;

#if LSM6DSM_ESP_ENABLED
  /** ESP information */
  lsm6dsm_esp_info esp_info;
#endif


  /** current instance configuration */
  lsm6dsm_hw_config current_conf;

  /** desired instance configuration */
  lsm6dsm_hw_config desired_conf;

  /** Interrupt dependency info. */
  lsm6dsm_irq_info        irq_info;
  lsm6dsm_irq_info        irq2_info;
  /** COM port info */
  lsm6dsm_com_port_info   com_port_info;

  /**--------Async Com Port--------*/
  sns_async_com_port_config  ascp_config;
  int16_t ascp_req_count;

#if LSM6DSM_DAE_ENABLED
  /**--------DAE interface---------*/
  lsm6dsm_dae_if_info       dae_if;
  lsm6dsm_config_step       config_step;
#endif

  /** Data streams from dependentcies. */
  sns_data_stream      *interrupt_data_stream;
  sns_data_stream      *interrupt2_data_stream;
  sns_data_stream      *timer_sensor_temp_data_stream;
  sns_data_stream      *timer_self_test_data_stream;
  sns_data_stream      *timer_md_data_stream;
  sns_data_stream      *async_com_port_data_stream;
  sns_data_stream      *timer_heart_beat_data_stream;
  sns_data_stream      *timer_polling_data_stream;
  sns_time              poll_timeout;

  sns_sensor_uid          timer_suid;


  lsm6dsm_health  health;

  size_t               encoded_imu_event_len;
  size_t               encoded_sensor_temp_event_len;

  /**----------Axis Conversion----------*/
  triaxis_conversion axis_map[TRIAXIS_NUM];

  /**----------Sensor specific registry configuration----------*/
  sns_lsm6dsm_registry_cfg accel_registry_cfg;
  sns_lsm6dsm_registry_cfg gyro_registry_cfg;
  sns_lsm6dsm_registry_cfg sensor_temp_registry_cfg;
  sns_lsm6dsm_registry_reset registry_reset;

  /**----------debug----------*/
  uint32_t  num_temp_samples;
  uint32_t  num_md_ints;
  uint32_t  num_ascp_null_events;

  sns_diag_service *diag_service;
  sns_sync_com_port_service * scp_service;
#if !LSM6DSM_LOGGING_DISABLED
  size_t           log_interrupt_encoded_size;

  size_t           log_raw_encoded_size;
  size_t           log_temp_raw_encoded_size;
#endif
  sns_time         oem_ts_offset;
} lsm6dsm_instance_state;

typedef struct odr_reg_map
{
  uint16_t           odr_coeff;
  uint16_t           accel_discard_samples;
  uint16_t           gyro_discard_samples;
  float              odr;
  float              accel_group_delay;  //ms
  float              gyro_group_delay;   //ms
  lsm6dsm_accel_odr  accel_odr_reg_value;
  lsm6dsm_gyro_odr   gyro_odr_reg_value;
} odr_reg_map;

typedef struct sns_lsm6dsm_self_test_req
{
  sns_physical_sensor_test_type test_type;
} sns_lsm6dsm_self_test_req;

typedef struct lsm6dsm_selftest_state
{
  bool                            requested:1;
  lsm6dsm_sensor_type             sensor;
  sns_physical_sensor_test_type   test_type;
} lsm6dsm_selftest_state;

/** Instance read-only state */
typedef struct lsm6dsm_instance_config
{
  bool                              irq2_enabled:1;
  bool                              update_registry;
  uint8_t                           accel_resolution_idx;
  uint8_t                           gyro_resolution_idx;
  uint8_t                           min_odr_idx;
  uint8_t                           max_odr_idx;
  lsm6dsm_selftest_state            selftest;
  sns_sensor_uid                    irq_suid;
  sns_sensor_uid                    timer_suid;
  sns_sensor_uid                    acp_suid;
  sns_sensor_uid                    reg_suid;
#if LSM6DSM_DAE_ENABLED
  sns_sensor_uid                    dae_suid;
  lsm6dsm_dae_if_state              dae_ag_state;
  lsm6dsm_dae_if_state              dae_temper_state;
#endif
  triaxis_conversion                axis_map[TRIAXIS_NUM];
  lsm6dsm_com_port_info             com_port_info;
  sns_interrupt_req                 irq_config;
  sns_interrupt_req                 irq2_config;
  sns_registry_md_cfg               md_config;
#if LSM6DSM_ESP_ENABLED
  //esp reg config
  lsm6dsm_esp_registry_cfg          esp_reg_cfg;
#endif

  lsm6dsm_stream_mode               ag_stream_mode;

  sns_lsm6dsm_registry_cfg          accel_cal;
  sns_lsm6dsm_registry_cfg          gyro_cal;
  sns_lsm6dsm_registry_cfg          temper_cal;

  float                             temper_sample_rate;
  float                             temper_report_rate;
  float                             sample_rate;
  float                             report_rate;
  float                             dae_report_rate;
  float                             avg_to_nominal_ratio;
  float                             avg_to_nominal_ratio_g;
  float                             sstvt_adj[3];
  uint64_t                          flush_period_ticks;

  lsm6dsm_sensor_type               flushing_sensors;
  lsm6dsm_sensor_type               config_sensors;
  lsm6dsm_sensor_type               fifo_enable;
  lsm6dsm_sensor_type               client_present;
  lsm6dsm_sensor_type               selftest_client_present;
  lsm6dsm_sensor_type               gated_client_present;
} lsm6dsm_instance_config;


sns_rc lsm6dsm_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate);

void lsm6dsm_copy_calibration_info(sns_lsm6dsm_registry_cfg*, sns_lsm6dsm_registry_cfg const*);

sns_rc lsm6dsm_inst_deinit(sns_sensor_instance *const this);
void lsm6dsm_set_fifo_config_timer(sns_sensor_instance *this);
void lsm6dsm_set_client_test_config(sns_sensor_instance *this,
                                    sns_request const *client_request);
void lsm6dsm_inst_hw_self_test(sns_sensor_instance *const this);
void lsm6dsm_inst_factory_self_test(sns_sensor_instance *const this);
void lsm6dsm_context_save(sns_sensor_instance * const this,uint8_t context_buffer [ ],uint8_t reg_map [ ],uint8_t reg_num);
void lsm6dsm_context_restore(sns_sensor_instance * const this,uint8_t context_buffer [ ],uint8_t reg_map [ ],uint8_t reg_num);
/**
 * Sends a FIFO complete event.
 *
 * @param instance   Instance reference
 */
void lsm6dsm_send_fifo_flush_done(sns_sensor_instance*,
                                  lsm6dsm_sensor_type,
                                  lsm6dsm_flush_done_reason);
sns_time lsm6dsm_estimate_avg_st(
  sns_sensor_instance *const instance,
  sns_time irq_timestamp,
  uint16_t num_samples);

void lsm6dsm_send_interrupt_is_cleared_msg(sns_sensor_instance *const this);

void lsm6dsm_run_polling_timer(sns_sensor_instance *const instance);
uint8_t lsm6dsm_get_odr_rate_idx(float desired_sample_rate);
void lsm6dsm_restart_hb_timer(sns_sensor_instance *const this, bool reset);

void lsm6dsm_clear_interrupt_q(sns_sensor_instance *const instance,
    sns_data_stream* interrupt_data_stream);

bool lsm6dsm_is_valid_oem_request(uint32_t message_id);
void lsm6dsm_handle_oem_request(sns_sensor *const this, sns_sensor_instance *instance, struct sns_request const *request);
void lsm6dsm_oem_set_default_config(sns_sensor_instance * const instance);
sns_time lsm6dsm_get_oem_ts_offset(void);
void lsm6dsm_oem_factory_test_config(sns_sensor_instance * const this, bool enable);
void lsm6dsm_esp_send_fifo_flush_done(sns_sensor_instance *instance,
                                      lsm6dsm_sensor_type flushing_sensors,
                                      lsm6dsm_flush_done_reason reason);
void lsm6dsm_update_oem_factory_config(sns_sensor *const this);

