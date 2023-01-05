#pragma once
/**
 * @file sns_lsm6dso_sensor_instance.h
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

#include <stdint.h>

#include "sns_lsm6dso_build_config.h"
#include "sns_async_com_port.pb.h"
#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#include "sns_lsm6dso_dae_if.h"
#include "sns_lsm6dso_s4s.h"
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
#include "sns_lsm6dso_esp.h"
#include "sns_lsm6dso_xsensor.h"
#include "sns_lsm6dso_ois.h"

/** Forward Declaration of Instance API */
extern sns_sensor_instance_api lsm6dso_sensor_instance_api;

/** Number of registers to read for debug */
#define LSM6DSO_DEBUG_REGISTERS          (32)

#define SELF_TEST_DATA_COUNT_MAX         (25)

#define LSM6DSO_ACCEL_RANGE_HW_SELFTEST LSM6DSO_ACCEL_RANGE_2G
#define LSM6DSO_ACCEL_SSTVT_HW_SELFTEST LSM6DSO_ACCEL_SSTVT_2G
//QC change to enable MD only
#define LSM6DSO_MD_SAMPLE_RATE    LSM6DSO_ODR_13            //Default sample rate for MD
#define LSM6DSO_MD_REPORT_PERIOD  (1000 *1000) //Default report  period for MD in secs conv to msec
#define MAX_INTERRUPT_CNT  2

/**
 * Accelerometer LSM6DSO_ACC Full Scales in register setting.
 */
typedef enum
{
  LSM6DSO_ACCEL_RANGE_2G   = 0x00,  /*corresponding value in register setting*/
  LSM6DSO_ACCEL_RANGE_4G   = 0x08,
  LSM6DSO_ACCEL_RANGE_8G   = 0x0C,
  LSM6DSO_ACCEL_RANGE_16G   = 0x04,
} lsm6dso_accel_range;

/**
 * Accelerometer LSM6DSO_ACC sensitivity for each range.
 */
typedef enum
{
  LSM6DSO_ACCEL_SSTVT_2G  = 61,   /* in the unit of micro-g/digit */
  LSM6DSO_ACCEL_SSTVT_4G  = 122,
  LSM6DSO_ACCEL_SSTVT_8G  = 244,
  LSM6DSO_ACCEL_SSTVT_16G = 488,
} lsm6dso_accel_sstvt;

/**
 * Accelerometer LSM6DSO ACC filter bandwidth in register setting
 */
typedef enum
{
  LSM6DSO_ACCEL_BW0       = 0x00,  /* 00 Hz bandwidth */
  LSM6DSO_ACCEL_BW13      = 0x00,  /* 13 Hz bandwidth */
  LSM6DSO_ACCEL_BW26      = 0x00,  /* 26 Hz bandwidth */
  LSM6DSO_ACCEL_BW50      = 0x00,  /* 50 Hz bandwidth */
  LSM6DSO_ACCEL_BW100     = 0x00,  /* 100 Hz bandwidth */
  LSM6DSO_ACCEL_BW200     = 0x00,  /* 200 Hz bandwidth */
  LSM6DSO_ACCEL_BW400     = 0x00,  /* 400 Hz bandwidth */
  LSM6DSO_ACCEL_BW800     = 0x00,  /* 800 Hz bandwidth */
  LSM6DSO_ACCEL_BW1600    = 0x01,  /* 1600 Hz bandwidth */
  LSM6DSO_ACCEL_BW3300    = 0x01,  /* 3300 Hz bandwidth */
  LSM6DSO_ACCEL_BW6600    = 0x01   /* 6600 Hz bandwidth */
} lsm6dso_accel_bw;

/**
 * Accelerometer LSM6DSO_ACC output data rate in register setting
 */
typedef enum
{
  LSM6DSO_ACCEL_ODR_OFF   = 0x00,  /* power down output data rate */
  LSM6DSO_ACCEL_ODR13     = 0x10,  /* 13 Hz output data rate */
  LSM6DSO_ACCEL_ODR26     = 0x20,  /* 26 Hz output data rate */
  LSM6DSO_ACCEL_ODR52     = 0x30,  /* 52 Hz output data rate */
  LSM6DSO_ACCEL_ODR104    = 0x40,  /* 104 Hz output data rate */
  LSM6DSO_ACCEL_ODR208    = 0x50,  /* 208 Hz output data rate */
  LSM6DSO_ACCEL_ODR416    = 0x60,  /* 416 Hz output data rate */
  LSM6DSO_ACCEL_ODR832    = 0x70,  /* 832 Hz output data rate */
  LSM6DSO_ACCEL_ODR1664   = 0x80,  /* 1.66 kHz output data rate */
  LSM6DSO_ACCEL_ODR3328   = 0x90,  /* 3.33 kHz output data rate */
  LSM6DSO_ACCEL_ODR6656   = 0xA0,  /* 6.66 kHz output data rate */
} lsm6dso_accel_odr;

/**
 * LSM6DSO output data rate for gyro, Disabling LPF2,
 * so BW setting is not required
 */
typedef enum
{
  LSM6DSO_GYRO_ODR_OFF = 0x00,       /* power down output data rate */
  LSM6DSO_GYRO_ODR13   = 0x10,       /* 13 Hz output data rate */
  LSM6DSO_GYRO_ODR26   = 0x20,       /* 26 Hz output data rate */
  LSM6DSO_GYRO_ODR52   = 0x30,       /* 52 Hz output data rate */
  LSM6DSO_GYRO_ODR104  = 0x40,       /* 104 Hz output data rate */
  LSM6DSO_GYRO_ODR208  = 0x50,       /* 208 Hz output data rate */
  LSM6DSO_GYRO_ODR416  = 0x60,       /* 416 Hz output data rate */
  LSM6DSO_GYRO_ODR832  = 0x70,       /* 832 Hz output data rate */
  LSM6DSO_GYRO_ODR1664 = 0x80,       /* 1.66 kHz output data rate */
  LSM6DSO_GYRO_ODR3328 = 0x90,       /* 3.33 kHz output data rate */
  LSM6DSO_GYRO_ODR6656 = 0xA0,       /* 6.66 kHz output data rate */
} lsm6dso_gyro_odr;

/**
 * LSM6DSO Full Scales in register setting for gyro
 */
typedef enum
{
  STM_LSM6DSO_GYRO_RANGE_125DPS   = 0x02,
  STM_LSM6DSO_GYRO_RANGE_245DPS   = 0x00,  /*corresponding value in register setting*/
  STM_LSM6DSO_GYRO_RANGE_500DPS   = 0x04,
  STM_LSM6DSO_GYRO_RANGE_1000DPS  = 0x08,
  STM_LSM6DSO_GYRO_RANGE_2000DPS  = 0x0C,
} lsm6dso_gyro_range;

typedef float lsm6dso_gyro_sstvt;

typedef struct lsm6dso_com_port_info
{
  sns_com_port_config          com_config;
  sns_sync_com_port_handle     *port_handle;
  uint8_t                      i2c_address;
  uint8_t                      i3c_address;
} lsm6dso_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr {
  float min;
  float max;
} range_attr;

typedef enum {
  FILTER_DEFAULT,
  SLOPE_FILTER,
  HP_FILTER,
}lsm6dso_md_filter_type;
typedef enum {
  DISABLED, //md disabled
  ENABLED_INT, //md enabled with interrupt
  ENABLED, //md enabled, no interrupt
}lsm6dso_md_state;
typedef enum
{
  LSM6DSO_ACCEL         = 0x01,
  LSM6DSO_GYRO          = 0x02,
  LSM6DSO_MOTION_DETECT = 0x04,
  LSM6DSO_SENSOR_TEMP   = 0x08,
  LSM6DSO_STEP_COUNTER  = 0x10,
  LSM6DSO_FREE_FALL     = 0x20,
  LSM6DSO_HIGH_SHOCK    = 0x40,
  LSM6DSO_OIS           = 0x80,
  LSM6DSO_ACTIVITY      = 0x100,
  LSM6DSO_INACTIVITY    = 0x200,
  LSM6DSO_XSENSOR_1     = 0x400,
  LSM6DSO_XSENSOR_2     = 0x800,
  LSM6DSO_XSENSOR_3     = 0x1000,
  LSM6DSO_XSENSOR_4     = 0x2000,
  LSM6DSO_XSENSOR_5     = 0x4000,
  LSM6DSO_DOUBLE_TAP    = 0x8000,
} lsm6dso_sensor_type;

//only for accel and gyro
typedef enum
{
  POLLING,
  DRI,
  S4S_SYNC
} lsm6dso_stream_mode;

#if LSM6DSO_DAE_ENABLED
typedef enum
{
  LSM6DSO_CONFIG_IDLE,            /** not configuring */
  LSM6DSO_CONFIG_POWERING_DOWN,   /** cleaning up when no clients left */
  LSM6DSO_CONFIG_STOPPING_STREAM, /** stream stop initiated, waiting for completion */
  LSM6DSO_CONFIG_FLUSHING_HW,     /** FIFO flush initiated, waiting for completion */
  LSM6DSO_CONFIG_FLUSHING_DATA,   /** FIFO flush initiated, waiting for completion */
  LSM6DSO_CONFIG_UPDATING_HW      /** updating sensor HW, when done goes back to IDLE */
} lsm6dso_config_step;
#endif

// QC - Fields in structures should be sorted by size to optimize code space

/*contains fifo reading req information
 * either interrupt or flush */

typedef struct lsm6dso_fifo_req
{
  bool              interrupt_fired;
  bool              recheck_int;
  bool              flush_req;
  bool              is_dae_ts_reliable;
  sns_time          interrupt_ts;
  sns_time          cur_time;
  uint16_t          wmk;
  lsm6dso_accel_odr accel_odr;
  //sns_time  sampling_intvl;
  //s4s info
  sns_time                 last_sync_ts;
  sns_time                 ideal_sync_interval;
  uint16_t                 t_ph;
  lsm6dso_s4s_sync_state_t sync_state;
}lsm6dso_fifo_req;

typedef struct lsm6dso_config_event_info
{
  float     sample_rate;
  uint32_t  fifo_watermark;
  sns_time  timestamp;
  sns_time  md_timestamp;
  sns_time  gated_timestamp;
#if LSM6DSO_DAE_ENABLED
  uint32_t  dae_watermark;
#endif
} lsm6dso_config_event_info;

/** HW FIFO information */
typedef struct lsm6dso_fifo_info
{
  /** is fifo reconfiguration is req for the new config req*/
  bool reconfig_req;

  /** is full fifo reconfiguration is req for the new config req*/
  bool full_reconf_req;

  /** reconfig time active */
  bool timer_active;

  bool gyro_extra_sample;

  /** is fifo data pushing to clients */
  bool                       is_streaming;
  /** is last timestamp valid for calculating sample time*/
  bool                       last_ts_valid;
  /** to know current batch is orphan or not*/
  bool                       orphan_batch;

  /** FIFO enabled or not. Uses lsm6dso_sensor_type as bit mask
   *  to determine which FIFO Sensors are enabled */
  uint8_t fifo_enabled;

  uint8_t ctrl_regs[8]; /** holds the current values of FIFO_CTRL1 to INT2_CTRL */

  uint8_t gyro_tag_cnt;
  uint8_t accel_tag_cnt;

  /** fifo cur rate index */
  lsm6dso_accel_odr fifo_rate;

  /** fifo desired rate index */
  lsm6dso_accel_odr desired_fifo_rate;

  /** desired FIFO watermark levels for accel and gyro*/
  uint16_t desired_wmk;

  /** FIFO watermark levels for accel and gyro*/
  uint16_t cur_wmk;

  /** number of interrupts fired without reconfig*/
  uint16_t                   interrupt_cnt;

  /** max requested FIFO watermark levels; possibly larger than max HW FIFO */
  uint32_t max_requested_wmk;
  /** avg interrupt interval without reconfiguring*/
  uint32_t                   avg_interrupt_intvl;
  /** interrupt thresholds - average interrupt interval must be recalculated if outside this window*/
  uint32_t                   interrupt_intvl_upper_bound;
  uint32_t                   interrupt_intvl_lower_bound;
  /** avg sampling interval without reconfiguring*/
  uint32_t                   avg_sampling_intvl;
  uint32_t                   nominal_dae_intvl;

  /** timestamp of last sample sent to framework*/
  sns_time                   last_timestamp;
  /** ascp event timestamp
   * ascp fills timestamp once reading completes */
  sns_time                   ascp_event_timestamp;
  /** timestamp when fifo interrupt fired*/
  sns_time                   interrupt_timestamp;

  /*contains info before sending ascp req*/
  lsm6dso_fifo_req th_info;
  /*contains info to use after ascp returns */
  lsm6dso_fifo_req bh_info;

  lsm6dso_config_event_info last_sent_config;
  lsm6dso_config_event_info new_config;

} lsm6dso_fifo_info;

typedef enum
{
  LSM6DSO_MODE_POLLING    = 0x1,
  LSM6DSO_MODE_FIFO       = 0x2,
  LSM6DSO_MODE_SELF_TEST  = 0x4
} lsm6dso_streaming_mode;

typedef enum
{
  LSM6DSO_SELF_TEST_STAGE_1    = 0x1,
  LSM6DSO_SELF_TEST_STAGE_2    = 0x2,
  LSM6DSO_SELF_TEST_STAGE_3    = 0x3,
  LSM6DSO_SELF_TEST_STAGE_4    = 0x4,
  LSM6DSO_SELF_TEST_STAGE_5    = 0x5,
} lsm6dso_self_test_stage;

typedef enum
{
  FLUSH_TO_BE_DONE,           // 0
  FLUSH_DONE_CONFIGURING,     // 1
  FLUSH_DONE_NOT_ACCEL_GYRO,  // 2
  FLUSH_DONE_NOT_FIFO,        // 3
  FLUSH_DONE_FIFO_EMPTY,      // 4
  FLUSH_DONE_AFTER_DATA,      // 5
} lsm6dso_flush_done_reason;

typedef struct lsm6dso_odr_change_info {
  sns_time            accel_odr_settime;
  sns_time            gyro_odr_settime;
  sns_time            hw_timer_start_time;
  sns_time            odr_change_timestamp;
  /** sampling interval calculated from ODR */
  uint32_t            nominal_sampling_intvl;
  /*sensor whose odr is actually changed */
  lsm6dso_sensor_type changed;
  /*sensor which requested for change */
  lsm6dso_sensor_type change_req;
  uint8_t             odr_idx;
  bool                gyro_startup;
} lsm6dso_odr_change_info;

typedef struct lsm6dso_common_info
{
  lsm6dso_accel_odr accel_curr_odr;
  lsm6dso_gyro_odr gyro_curr_odr;
  uint8_t mode;
  //add few more hre
} lsm6dso_common_info;

typedef struct lsm6dso_self_test_info
{
  bool test_alive;
  bool reconfig_postpone;
  lsm6dso_self_test_stage  self_test_stage;
  uint16_t  polling_count;
  int64_t cumulative_data_pre[TRIAXIS_NUM];
  int64_t cumulative_data_post[TRIAXIS_NUM];
  lsm6dso_sensor_type sensor;
  lsm6dso_accel_odr curr_odr;
  uint8_t odr_idx;
  uint8_t skip_count;
  sns_physical_sensor_test_type test_type;
  bool update_registry;
#if LSM6DSO_OEM_FACTORY_CONFIG
  bool return_now;
#endif
  //add few more hre
} lsm6dso_self_test_info;

typedef struct lsm6dso_std_sensor_event {
  uint8_t                 opdata_raw[6];
  sns_std_sensor_sample_status opdata_status;
} lsm6dso_std_sensor_event;

typedef struct lsm6dso_accel_info
{
  lsm6dso_accel_odr desired_odr;
  lsm6dso_accel_sstvt     sstvt;
  lsm6dso_accel_range     range;
  uint8_t                 range_idx;
  lsm6dso_accel_bw        bw;
  bool                    lp_mode;
  sns_sensor_uid          suid;
  uint16_t                num_samples_to_discard;
  bool                    gated_client_present;
  lsm6dso_std_sensor_event sample;
} lsm6dso_accel_info;

typedef struct lsm6dso_gyro_info
{
  lsm6dso_gyro_odr desired_odr;
  lsm6dso_gyro_sstvt      sstvt;
  lsm6dso_gyro_range      range;
  uint8_t                 range_idx;
  bool                    is_in_sleep;
  sns_sensor_uid          suid;
  uint16_t                num_samples_to_discard;
  lsm6dso_std_sensor_event sample;
  float                   prev_data[3];
  float                   prev_filtered[3];
  bool                    reset_filter;
  bool                    sw_lpf_data_settled;
} lsm6dso_gyro_info;

typedef struct lsm6dso_motion_detect_info
{
  uint16_t                desired_wmk;
  lsm6dso_accel_odr       desired_odr;
  lsm6dso_accel_sstvt     sstvt;
  lsm6dso_accel_range     range;
  lsm6dso_accel_bw        bw;
  sns_sensor_uid          suid;
  /* some features depends on md, so represents
   * useful while disabling or re-enabling md */
  lsm6dso_sensor_type     internal_client_present;

  /*denotes if external client is present or not */
  uint16_t                client_present;

  bool                    is_filter_settled;
  bool                    is_timer_running;
  bool                    add_request;
  sns_motion_detect_event cur_state;
  lsm6dso_md_filter_type  filter;
  sns_registry_md_cfg     md_config;
  sns_time                event_ts;
} lsm6dso_motion_detect_info;

typedef struct lsm6dso_sensor_temp_info
{
  sns_sensor_uid          suid;
  bool                    timer_is_active;
  bool                    self_test_is_successful;
  float                   report_rate_hz;
  float                   desired_sampling_rate_hz;
  float                   cur_sampling_rate_hz;
  sns_time                sampling_intvl;
  uint64_t                max_requested_flush_ticks;
  lsm6dso_config_event_info last_sent_config;
  lsm6dso_config_event_info new_config;
} lsm6dso_sensor_temp_info;

typedef struct lsm6dso_irq_info
{
  sns_interrupt_req       irq_config;
  bool                    irq_registered:1;
  bool                    irq_ready:1;
} lsm6dso_irq_info;

typedef struct lsm6dso_async_com_port_info
{
  uint32_t                port_handle;
}lsm6dso_async_com_port_info;

typedef struct sns_lsm6dso_registry_cfg
{
  sns_time            ts;
  lsm6dso_sensor_type sensor_type;
  matrix3             fac_cal_corr_mat;
  float               fac_cal_bias[3];
  uint32_t            registry_persist_version;
  uint32_t            registry_instance_version;
  float               mean_temp;
  vector3             thermal_scale;
}sns_lsm6dso_registry_cfg;

typedef struct lsm6dso_hw_config
{
  /** max flush ticks*/
  uint64_t max_requested_flush_ticks;
  //value of a/g odr in HZ
  uint16_t odr;
  //fifo odr - if set fifo mode, polling
  uint16_t fifo_odr;
  //set wmk
  uint16_t wmk;
  //decides which sensors are enabled
  uint8_t enabled_sensors;
  /** Determines which Sensor data to publish. Uses
   *  lsm6dso_sensor_type as bit mask. */
  uint8_t publish_sensors;
  // index into odr map table
  uint8_t odr_idx;
  //defines whether HW MD feature needed or not
  /* set based on gated/non-gated requests/
   * client present and internal client present */
  bool md_enabled;

} lsm6dso_hw_config;

typedef struct sns_lsm6dso_registry_reset
{
  lsm6dso_sensor_type sensor_type;
  bool            request;
}sns_lsm6dso_registry_reset;

typedef struct lsm6dso_health
{
  sns_time    expected_expiration;
  uint64_t    heart_beat_timeout;
  bool heart_attack;
  uint8_t heart_attack_cnt;
} lsm6dso_health;

/** Private state. */
typedef struct lsm6dso_instance_state
{
  uint8_t *fifo_start_address;

  lsm6dso_stream_mode   ag_stream_mode;

  /** detail about self test params*/
  lsm6dso_self_test_info self_test_info;

  /** Debug counter for sample count */
  uint32_t accel_sample_counter;

  /** Debug counter for sample count */
  uint32_t gyro_sample_counter;

  /** which sensors are being flushed */
  lsm6dso_sensor_type flushing_sensors;

  /** which sensors are being (re)configured */
  lsm6dso_sensor_type config_sensors;

  /** detail about common params*/
  lsm6dso_common_info common_info;
  
  lsm6dso_odr_change_info prev_odr_change_info;

  lsm6dso_odr_change_info cur_odr_change_info;

  /** details about s4s parameters */
  lsm6dso_s4s_info       s4s_info;

  /** fifo details*/
  lsm6dso_fifo_info       fifo_info;

  /** accel HW config details*/
  lsm6dso_accel_info      accel_info;

  /** gyro HW config details*/
  lsm6dso_gyro_info       gyro_info;

  /** motion detect info */
  lsm6dso_motion_detect_info md_info;

  /** Sensor Temperature config details. */
  lsm6dso_sensor_temp_info sensor_temp_info;

#if LSM6DSO_ESP_ENABLED
  /** ESP information */
  lsm6dso_esp_info esp_info;
  lsm6dso_xsensor_group_info xgroup_info;
#endif

#if LSM6DSO_OIS_ENABLED
  /** ESP information */
  lsm6dso_ois_info ois_info;
#endif

  /** current instance configuration */
  lsm6dso_hw_config current_conf;

  /** desired instance configuration */
  lsm6dso_hw_config desired_conf;

  /** Interrupt dependency info. */
  lsm6dso_irq_info        irq_info;
  lsm6dso_irq_info        irq2_info;
  bool                    irq_ready;
  bool                    irq2_inst_enabled;
  bool                    int_enabled; //enable H/W interrupts bit

  /** ESP info */
  bool                    route_md_to_irq2;

  /** hw index */
  uint8_t hw_idx;

  /** rigid body type */
  uint8_t rigid_body_type;
  uint16_t ibi_clock_freq; //multiples of 0.5Mhz

  /** clock trim factor for the device */
  float clock_trim_factor;

  /** precentage of ODR variation w.r.t nominal value calculated
   * from fine freq register: odr percent variation (nominal - actual) * ODR
   * nominal: calc from fine freq register
   * actual: averaged from interrupt ts difference */
  int32_t odr_percent_var_accel;
  int32_t odr_percent_var_gyro;

  /** which entry in lsm6dso_odr_map[] to use for min ODR */
  uint8_t min_odr_idx;

  /** COM port info */
  lsm6dso_com_port_info   com_port_info;

  /**--------Async Com Port--------*/
  sns_async_com_port_config  ascp_config;
  int16_t ascp_req_count;
  bool  bus_pwr_on;

#if LSM6DSO_DAE_ENABLED
  /**--------DAE interface---------*/
  lsm6dso_dae_if_info       dae_if;
  lsm6dso_config_step       config_step;
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
  sns_data_stream      *timer_sensor_ois_data_stream;
  sns_time              poll_timeout;

  sns_sensor_uid          timer_suid;

  uint8_t              wake_src;

  lsm6dso_health  health;

  size_t               encoded_imu_event_len;
  size_t               encoded_sensor_temp_event_len;

  /**----------Axis Conversion----------*/
  triaxis_conversion axis_map[TRIAXIS_NUM];

  /**----------Sensor specific registry configuration----------*/
  sns_lsm6dso_registry_cfg accel_registry_cfg;
  sns_lsm6dso_registry_cfg gyro_registry_cfg;
  sns_lsm6dso_registry_cfg sensor_temp_registry_cfg;
  sns_lsm6dso_registry_reset registry_reset;

  /**----------debug----------*/
  uint32_t  num_temp_samples;
  uint32_t  num_md_ints;
  uint32_t  num_ascp_null_events;
  uint8_t   reg_status[LSM6DSO_DEBUG_REGISTERS];

  sns_diag_service *diag_service;
  sns_sync_com_port_service * scp_service;
  size_t           log_interrupt_encoded_size;
  size_t           log_raw_encoded_size;
  size_t           log_temp_raw_encoded_size;
  sns_time         oem_ts_offset;
  /* TODO For better accuracy, consider recording the last temperature sensor timestamp, if it's too
   * far in the past, perform a read on the temperature regsiter and overwrite the current value.
   * A drawback of this might be the delay in delivering the motion sample. */
  float            most_recent_temperature_reading;
  bool             gyro_introduced;
  lsm6dso_sensor_type  passive_client_present;
} lsm6dso_instance_state;

typedef struct odr_reg_map
{
  float              odr;
  float              accel_group_delay;  //ms
  float              gyro_group_delay;   //ms
  lsm6dso_accel_odr  accel_odr_reg_value;
  lsm6dso_gyro_odr   gyro_odr_reg_value;
  uint16_t           odr_coeff;
  uint16_t           accel_discard_samples;
  uint16_t           gyro_discard_samples;
} odr_reg_map;

typedef struct sns_lsm6dso_self_test_req
{
  sns_physical_sensor_test_type test_type;
} sns_lsm6dso_self_test_req;

typedef struct lsm6dso_selftest_state
{
  lsm6dso_sensor_type             sensor;
  sns_physical_sensor_test_type   test_type;
  bool                            requested:1;
} lsm6dso_selftest_state;

/** Instance read-only state */
typedef struct lsm6dso_instance_config
{
  sns_sensor_uid                    irq_suid;
  sns_sensor_uid                    timer_suid;
  sns_sensor_uid                    acp_suid;
  sns_sensor_uid                    reg_suid;
#if LSM6DSO_DAE_ENABLED
  sns_sensor_uid                    dae_suid;
  lsm6dso_dae_if_state              dae_ag_state;
  lsm6dso_dae_if_state              dae_temper_state;
#endif
  triaxis_conversion                axis_map[TRIAXIS_NUM];
  lsm6dso_com_port_info             com_port_info;
  sns_interrupt_req                 irq_config;
  sns_interrupt_req                 irq2_config;
  sns_registry_md_cfg               md_config;
#if LSM6DSO_ESP_ENABLED
  //esp reg config
  lsm6dso_esp_registry_cfg          esp_reg_cfg;
#endif
#if LSM6DSO_OIS_ENABLED
  //OIS reg config
  lsm6dso_ois_registry_cfg          ois_reg_cfg;
#endif

  uint8_t                           accel_resolution_idx;
  uint8_t                           gyro_resolution_idx;
  uint8_t                           min_odr_idx;
  uint8_t                           max_odr_idx;
  lsm6dso_selftest_state            selftest;
  bool                              irq2_enabled:1;
  lsm6dso_stream_mode               ag_stream_mode;

  sns_lsm6dso_registry_cfg          accel_cal;
  sns_lsm6dso_registry_cfg          gyro_cal;
  sns_lsm6dso_registry_cfg          temper_cal;

  float                             temper_sample_rate;
  float                             temper_report_rate;
  float                             sample_rate;
  float                             report_rate;
  float                             dae_report_rate;
  uint64_t                          flush_period_ticks;

  lsm6dso_sensor_type               flushing_sensors;
  lsm6dso_sensor_type               config_sensors;
  lsm6dso_sensor_type               fifo_enable;
  lsm6dso_sensor_type               client_present;
  lsm6dso_sensor_type               selftest_client_present;
  lsm6dso_sensor_type               gated_client_present;
  lsm6dso_sensor_type               passive_client_present;
} lsm6dso_instance_config;


sns_rc lsm6dso_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate);

void lsm6dso_copy_calibration_info(sns_lsm6dso_registry_cfg*, sns_lsm6dso_registry_cfg const*);

sns_rc lsm6dso_inst_deinit(sns_sensor_instance *const this);
void lsm6dso_set_fifo_config_timer(sns_sensor_instance *this);
void lsm6dso_set_client_test_config(sns_sensor_instance *this,
                                    sns_request const *client_request);
void lsm6dso_inst_hw_self_test(sns_sensor_instance *const this);
void lsm6dso_inst_factory_self_test(sns_sensor_instance *const this);
void lsm6dso_context_save(sns_sensor_instance * const this,uint8_t context_buffer [ ],uint8_t reg_map [ ],uint8_t reg_num);
void lsm6dso_context_restore(sns_sensor_instance * const this,uint8_t context_buffer [ ],uint8_t reg_map [ ],uint8_t reg_num);
/**
 * Sends a FIFO complete event.
 *
 * @param instance   Instance reference
 */
void lsm6dso_send_fifo_flush_done(sns_sensor_instance*,
                                  lsm6dso_sensor_type,
                                  lsm6dso_flush_done_reason);
sns_time lsm6dso_estimate_avg_st(
  sns_sensor_instance *const instance,
  sns_time irq_timestamp,
  uint16_t num_samples);


void lsm6dso_run_polling_timer(sns_sensor_instance *const instance);
uint8_t lsm6dso_get_odr_rate_idx(float desired_sample_rate);
void lsm6dso_restart_hb_timer(sns_sensor_instance *const this, bool reset);

sns_rc lsm6dso_handle_timer(
    sns_sensor_instance *const instance, sns_data_stream* timer_data_stream,
    sns_rc (*timer_handler)(sns_sensor_instance *const , sns_time, sns_timer_sensor_event* latest_timer_event));
void lsm6dso_clear_interrupt_q(sns_sensor_instance *const instance,
    sns_data_stream* interrupt_data_stream);

sns_time lsm6dso_recover_ibi_irq_ts(sns_sensor_instance *const this, sns_time master_ref, const uint8_t* ibi_data);
//OEM specific headers
bool lsm6dso_is_valid_oem_request(uint32_t message_id);
void lsm6dso_handle_oem_request(sns_sensor *const this, sns_sensor_instance *instance, struct sns_request const *request);
sns_time lsm6dso_get_oem_ts_offset(void);
void lsm6dso_oem_factory_test_config(sns_sensor_instance * const this, bool enable);

