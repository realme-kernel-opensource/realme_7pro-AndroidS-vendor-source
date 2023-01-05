#pragma once
/**
 * @file sns_mmc5603x_sensor_instance.h
 *
 * MMC5603X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_uid.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include <stdint.h>
#include "sns_printf.h"

#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#include "sns_timer.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_mmc5603x_lite.h"

#include "sns_mmc5603x_dae_if.h"

#include "sns_async_com_port.pb.h"
#include "sns_physical_sensor_test.pb.h"

#include "sns_math_util.h"
#include "sns_registry_util.h"



/** Forward Declaration of Instance API */
extern sns_sensor_instance_api mmc5603x_sensor_instance_api;

/** Number of entries in reg_map table. */
#define MMC5603X_REG_MAP_TABLE_SIZE  (7)

/** MMC5603X max number of settings */
#define MMC5603X_MAX_NUM_REP_MODE    3
#define MMC5603X_MAX_NUM_OPE_MODE    3
#define MMC5603X_MAX_NUM_ODR         6



#ifdef MMC5603X_ENABLE_DEBUG_MSG
#define MMC5603X_PRINT(prio, sensor, ...) do { \
  SNS_PRINTF(prio, sensor, __VA_ARGS__); \
} while (0)

#define MMC5603X_INST_PRINT(prio, inst, ...) do { \
  SNS_INST_PRINTF(prio, inst , __VA_ARGS__); \
} while (0)
#else // MMC5603X_ENABLE_DEBUG_MSG
#define MMC5603X_PRINT(prio,...)
#define MMC5603X_INST_PRINT(prio,...)
#endif // MMC5603X_ENABLE_DEBUG_MSG

/** Supported MEMSIC Devices */
typedef enum
{
  MMC5603X,
  SUPPORTED_DEVICES
} memsic_device_type;

/**
 * MMC5603X output data rate for mag
 */
typedef enum
{
  MMC5603X_MAG_ODR_OFF = 0x00,      /* power down output data rate */
  MMC5603X_MAG_ODR_SNG_MEAS = 0x01, /* single measurement mode */
  MMC5603X_MAG_ODR5=5,
  MMC5603X_MAG_ODR10 = 10,        /* 10 Hz output data rate */
  MMC5603X_MAG_ODR15 = 15,        /* 15Hz output data rate */
  MMC5603X_MAG_ODR25 = 25,        /* 25 Hz output data rate */
  MMC5603X_MAG_ODR50 = 50,        /* 50 Hz output data rate */
  MMC5603X_MAG_ODR100 = 100,       /* 100 Hz output data rate */
 // MMC5603X_MAG_ODR200 = 200,       /* 200 Hz output data rate */
  MMC5603X_MAG_ODR1 = 1,         /* 1 Hz output data rate */
  MMC5603X_MAG_SELFTEST = 0x10,     /* selftest */
  MMC5603X_MAG_FUSEROM = 0x1F,      /* FUSE ROM access mode */
} mmc5603x_mag_odr;

typedef enum{ 
	MMC5603NJ_DISABLE_MEAS, 
	MMC5603NJ_ENABLE_MEAS
	} 
mmc5603nj_enable_state;

typedef float mmc5603x_mag_sstvt;

typedef struct mmc5603x_com_port_info
{
  sns_com_port_config      com_config;
  sns_sync_com_port_handle *port_handle;
  uint8_t                  i2c_address;
  uint8_t                  i3c_address;
  bool                     in_i3c_mode;
} mmc5603x_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr
{
  float min;
  float max;
} range_attr;

typedef enum
{
  MMC5603X_CONFIG_IDLE,              /** not configuring */
  MMC5603X_CONFIG_POWERING_DOWN,     /** cleaning up when no clients left */
  MMC5603X_CONFIG_STOPPING_STREAM,   /** stream stop initiated, waiting for completion */
  MMC5603X_CONFIG_FLUSHING_HW,       /** FIFO flush initiated, waiting for completion */
  MMC5603X_CONFIG_UPDATING_HW        /** updating sensor HW, when done goes back to IDLE */
} mmc5603x_config_step;

#ifdef MMC5603X_ENABLE_S4S
typedef enum
{
  MMC5603X_S4S_NOT_SYNCED,
  MMC5603X_S4S_SYNCING,
  MMC5603X_S4S_1ST_SYNCED,
  MMC5603X_S4S_SYNCED
} mmc5603x_s4s_state;
#endif // MMC5603X_ENABLE_S4S

typedef struct mmc5603x_self_test_info
{
  sns_physical_sensor_test_type test_type;
  bool test_client_present;
} mmc5603x_self_test_info;

typedef struct mmc5603x_config_event_info
{
  mmc5603x_mag_odr odr;
  uint8_t         fifo_wmk;
} mmc5603x_config_event_info;

typedef struct mmc5603x_mag_info
{
  mmc5603x_mag_odr   desired_odr;
  mmc5603x_mag_odr   curr_odr;
  uint32_t          flush_period;
  mmc5603x_mag_sstvt sstvt_adj[3];
  mmc5603x_mag_sstvt resolution;
  memsic_device_type   device_select;
  uint8_t              num_samples_to_discard;
  uint32_t       req_wmk;
  uint16_t       cur_wmk;
  bool           use_dri;
  bool           use_fifo;
  bool           flush_only;
  bool           max_batch;
  bool           use_sync_stream;
  uint8_t        nsf;
  uint8_t        sdr;
  sns_sensor_uid suid;
  mmc5603x_self_test_info test_info;

  uint32_t      data_count;
  uint32_t      clock_error_meas_count;

#ifdef MMC5603X_ENABLE_S4S
  mmc5603x_s4s_state      s4s_sync_state;
  uint8_t                s4s_rr;
  bool                   s4s_dt_abort;
#endif // MMC5603X_ENABLE_S4S
} mmc5603x_mag_info;

typedef struct mmc5603x_irq_info
{
  sns_interrupt_req irq_config;
  bool is_registered;
  bool is_ready;
  bool detect_irq_event;
} mmc5603x_irq_info;


typedef struct mmc5603x_cal_param
{
  bool registry_fac_cal_received;
  matrix3 fac_cal_corr_mat;
  float fac_cal_bias[TRIAXIS_NUM];
  float fac_cal_scale[TRIAXIS_NUM];
   uint32_t version;
}mmc5603x_cal_param;


typedef struct mmc5603x_cal
{
  uint32_t id;
  mmc5603x_cal_param params[MAX_DEVICE_MODE_SUPPORTED];
}mmc5603x_cal;



typedef struct mmc5603x_async_com_port_info
{
  uint32_t port_handle;
} mmc5603x_async_com_port_info;

typedef struct sns_mmc5603x_registry_cfg
{
  matrix3             fac_cal_corr_mat;
  float               fac_cal_bias[3];

  uint32_t            version;
}sns_mmc5603x_registry_cfg;

/** Private state. */
typedef struct mmc5603x_instance_state
{
  /** mag HW config details*/
  mmc5603x_mag_info mag_info;
  mmc5603x_config_event_info last_sent_cfg;
  mmc5603x_config_event_info new_cfg;
  uint32_t total_samples; /* throughout the life of this instance */

  /** sampling info. */
  uint8_t num_samples;
  uint8_t ascp_xfer_in_progress;
  uint8_t heart_beat_sample_count;
  uint8_t heart_beat_attempt_count;
  uint8_t flush_sample_count;
  bool this_is_first_data;
  bool data_over_run;
  bool data_is_ready;
  bool re_read_data_after_ascp;
  bool fifo_flush_in_progress;
  bool new_self_test_request;
  bool config_mag_after_ascp_xfer;
  bool this_is_the_last_flush;
  bool reg_event_done;
#ifdef MMC5603X_ENABLE_S4S
  bool s4s_reg_event_done;
#endif
  bool is_temp_average;
  bool in_clock_error_procedure;
  bool previous_meas_is_irq;
  bool previous_meas_is_correct_wm;
  sns_time interrupt_timestamp;
  sns_time irq_event_time;
  sns_time pre_timestamp;
  sns_time first_data_ts_of_batch;
  sns_time averaged_interval;
  sns_time temp_averaged_interval;
  sns_time system_time;
  sns_time previous_irq_time;
  sns_time heart_beat_timestamp;
  sns_time odr_change_timestamp;
  sns_time heart_beat_timeout_period;
  sns_time nominal_intvl;
  sns_time half_measurement_time;
  sns_time hb_timer_fire_time;
  sns_timer_sensor_config req_payload;
  int64_t internal_clock_error;

  /** Timer info */
  sns_sensor_uid timer_suid;

  /** Interrupt dependency info. */
  mmc5603x_irq_info irq_info;

  /** COM port info */
  mmc5603x_com_port_info com_port_info;


  /**--------DAE interface---------*/
  mmc5603x_dae_if_info       dae_if;
  mmc5603x_config_step       config_step;

  /** Data streams from dependencies. */
  sns_data_stream       *timer_data_stream;
#ifdef MMC5603X_ENABLE_S4S
  sns_data_stream       *s4s_timer_data_stream;
#endif // MMC5603X_ENABLE_S4S

  uint32_t              client_req_id;
  sns_std_sensor_config mag_req;
  int16_t               pre_lsbdata[TRIAXIS_NUM];

   int satu_checking_flag;

  size_t encoded_mag_event_len;

  /**----------Axis Conversion----------*/
  triaxis_conversion axis_map[TRIAXIS_NUM];

  /**----------Sensor specific registry configuration----------*/
  sns_mmc5603x_registry_cfg mag_registry_cfg;
  mmc5603x_cal           cal;


  sns_diag_service *diag_service;
  sns_sync_com_port_service *scp_service;
  size_t           log_raw_encoded_size;
  mmc5603nj_enable_state  is_enabled;

} mmc5603x_instance_state;

typedef struct odr_reg_map
{
  float           odr;
  mmc5603x_mag_odr mag_odr_reg_value;
  uint8_t         discard_samples;
} odr_reg_map;

typedef struct sns_mmc5603x_mag_req
{
  float sample_rate;
  float report_rate;
  uint32_t flush_period;
  bool is_flush_only;
  sns_mmc5603x_registry_cfg registry_cfg;
  bool is_max_batch;
  uint32_t cal_id;
  uint32_t cal_version;
} sns_mmc5603x_mag_req;


sns_rc mmc5603x_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate);

sns_rc mmc5603x_inst_deinit(sns_sensor_instance *const this);

sns_rc mmc5603x_inst_set_client_config(sns_sensor_instance *const this,
                                      sns_request const *client_request);
