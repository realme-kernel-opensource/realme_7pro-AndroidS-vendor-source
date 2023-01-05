#pragma once
/**
 * @file sns_ak0991x_sensor.h
 *
 * AK0991X Sensor implementation.
 *
 * Copyright (c) 2016-2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2019 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_ak0991x_lite.h"
#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_sensor_uid.h"
#include "sns_pwr_rail_service.h"
#include "sns_suid_util.h"

#include "sns_ak0991x_hal.h"

#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_dae_if.h"
#include "sns_math_util.h"
#include "sns_registry_util.h"

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
#include "sns_device_mode.pb.h"
#endif //AK0991X_ENABLE_DEVICE_MODE_SENSOR

#ifndef AK0991X_ENABLE_REGISTRY_ACCESS
#include "sns_interface_defs.h"
#endif //AK0991X_ENABLE_REGISTRY_ACCESS

#define MAG_SUID1 \
  {  \
    .sensor_uid =  \
    {  \
      0x0f, 0x1e, 0x04, 0xec, 0x30, 0xcf, 0x4d, 0xa5,  \
      0xbc, 0x79, 0xd3, 0x09, 0x4d, 0x60, 0xd5, 0xeb  \
    }  \
  }

#ifdef AK0991X_ENABLE_DUAL_SENSOR
#define MAG_SUID2 \
  {  \
    .sensor_uid =  \
    {  \
      0xaf, 0xf9, 0x3c, 0x5a, 0xc8, 0xc5, 0x45, 0x70, \
      0xb4, 0xbe, 0xc1, 0xa5, 0x7b, 0x8d, 0x5f, 0xec  \
    }  \
  }
#endif //AK0991X_ENABLE_DUAL_SENSOR

#define AK0991X_STR                     "ak0991x_"
#define AK0991X_PLATFORM_CONFIG_STR     "_platform.config"
#define AK0991X_PLATFORM_PLACEMENT_STR  "_platform.placement"
#define AK0991X_PLATFORM_ORIENT_STR     "_platform.orient"
#define AK0991X_PLATFORM_FACCAL_STR     "_platform.mag.fac_cal"
#define AK0991X_MAG_CONFIG_STR          ".mag.config"
#define AK0991X_REG_CONFIG_STR          ".mag.config_2"

#ifndef AK0991X_ENABLE_REGISTRY_ACCESS
#define BUS_TYPE          _AK0991X_BUS_TYPE
#define RAIL_1            _AK0991X_RAIL_1
#define RAIL_2            _AK0991X_RAIL_2
#define IRQ_NUM           _AK0991X_IRQ_NUM
#define NUM_OF_RAILS      _AK0991X_NUM_RAILS
#define BUS_FREQ_MIN      _AK0991X_BUS_FREQ_MIN
#define BUS_FREQ_MAX      _AK0991X_BUS_FREQ_MAX
#define SLAVE_ADDRESS     _AK0991X_I2C_ADDR
#define I3C_ADDR          _AK0991X_I3C_ADDR
#define I2C_BUS_INSTANCE  _AK0991X_BUS_INSTANCE
#endif //AK0991X_ENABLE_REGISTRY_ACCESS

/** Forward Declaration of Magnetic Sensor API */
extern sns_sensor_api ak0991x_mag_sensor_api;

#ifdef AK0991X_GET_PARAMETER_FROM_SMEM
#define AK0991X_PARAMETER_FACTOR 10000.0
#define AK0991X_PARAMETER_NUM  9
#endif

/**
 * AK0991X ODR definitions
 */
#define AK0991X_ODR_0              0.0
#define AK0991X_ODR_1              1.0
#define AK0991X_ODR_10             10.0
#define AK0991X_ODR_20             20.0
#define AK0991X_ODR_50             50.0
#define AK0991X_ODR_100            100.0
#define AK0991X_ODR_200            200.0

/**
 * Magnetometer ranges
 */
#define AK09918_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09918_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09917_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09917_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09916_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09916_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09915_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09915_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09913_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09913_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09912_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09912_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09911_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09911_MAX_RANGE          4912  /* Maximum  4912uT */

/**
 * Magnetometer resolution
 */
#define AK09918_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09917_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09916_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09915_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09913_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09912_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09911_RESOLUTION         (0.6f)  /* uT/LSB */

/* Power consumption limits */
#define AK09918_LO_PWR             1    /* unit of uA */
#define AK09918_HI_PWR             1100 /* unit of uA @ 100Hz */
#define AK09917_LO_PWR             1    /* unit of uA */
#define AK09917_HI_PWR             900  /* unit of uA @ 100Hz low power (TBD copy from AK09915)*/
#define AK09916_LO_PWR             1    /* unit of uA */
#define AK09916_HI_PWR             1100 /* unit of uA @ 100Hz */
#define AK09915_LO_PWR             3    /* unit of uA */
#define AK09915_HI_PWR             900  /* unit of uA @ 100Hz low power */
#define AK09913_LO_PWR             3    /* unit of uA */
#define AK09913_HI_PWR             1500 /* unit of uA @ 100Hz */
#define AK09912_LO_PWR             3    /* unit of uA */
#define AK09912_HI_PWR             1000 /* unit of uA @ 100Hz */
#define AK09911_LO_PWR             3    /* unit of uA */
#define AK09911_HI_PWR             2400 /* unit of uA @ 100Hz */

/** Supported opertating modes */
#define AK0991X_NORMAL             "NORMAL"
#define AK0991X_LOW_POWER          "LOW_POWER"
#define AK0991X_LOW_NOISE          "LOW_NOISE"

#define AK0991X_NUM_OF_ATTRIBUTES  (21)
#define AK0991X_MAX_LEN_OF_ATTRIBUTES_STR (15)

/** Power rail timeout States for the AK0991X Sensors.*/

#define AK0991X_POWER_RAIL_OFF_TIMEOUT_NS     1000000000ULL /* 1 second */
#define AK0991X_POWER_RAIL_POLLING_TIMEOUT_NS 10000000ULL /* 10 msec */

typedef enum
{
  AK0991X_POWER_RAIL_PENDING_NONE,
  AK0991X_POWER_RAIL_PENDING_INIT,
  AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ,
  AK0991X_POWER_RAIL_PENDING_OFF,
  AK0991X_POWER_RAIL_PENDING_WAIT_FOR_FLUSH,
} ak0991x_power_rail_pending_state;

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
/** Registry items supported as part of physical sensor
 *  configuraton registry group
 */
typedef struct ak0991x_registry_phy_sensor_cfg
{
  bool    use_fifo;
  uint8_t nsf;
  uint8_t sdr;
} ak0991x_registry_phy_sensor_cfg;
#endif //AK0991X_ENABLE_REGISTRY_ACCESS

/** Interrupt Sensor State. */

typedef struct ak0991x_state
{
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  sns_data_stream       *reg_data_stream;
#endif // AK0991X_ENABLE_REGISTRY_ACCESS
  sns_data_stream       *fw_stream;
  sns_data_stream       *timer_stream;

  // Registry, IRQ, Timer, ASCP, DAE, device_mode
  SNS_SUID_LOOKUP_DATA(SUID_NUM) suid_lookup_data;

  ak0991x_com_port_info com_port_info;
  sns_interrupt_req      irq_config;
  sns_pwr_rail_service  *pwr_rail_service;
  sns_rail_config       rail_config;
  sns_power_rail_state  registry_rail_on_state;

  bool hw_is_present;
  bool sensor_client_present;
  bool remove_timer_stream;

  uint32_t debug_log_count;

  ak0991x_power_rail_pending_state power_rail_pend_state;

  // parameters which are determined when the connected device is specified.
  akm_device_type device_select; // store the current connected device
  uint8_t reg_rsv1_value;
  float sstvt_adj[AK0991X_NUM_SENSITIVITY];

  // sensor configuration
  uint8_t nsf;
  uint8_t sdr;
  uint8_t min_odr;
  uint8_t max_odr;
  bool use_fifo;
  bool supports_sync_stream;
  uint8_t resolution_idx;
  int64_t hardware_id;
  ak0991x_int_op_mode int_mode;
#ifdef AK0991X_ENABLE_DUAL_SENSOR
  uint32_t registration_idx;
#endif //AK0991X_ENABLE_DUAL_SENSOR

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  // registry sensor config
  bool registry_cfg_received;
  sns_registry_phy_sensor_cfg registry_cfg;
  // registry sensor reg config
  bool registry_reg_cfg_received;
  ak0991x_registry_phy_sensor_cfg registry_reg_cfg;
  // registry sensor platform config
  bool registry_pf_cfg_received;
  sns_registry_phy_sensor_pf_cfg registry_pf_cfg;
  // axis conversion
  bool registry_orient_received;
  // placement
  bool registry_placement_received;
#endif //AK0991X_ENABLE_REGISTRY_ACCESS

  triaxis_conversion axis_map[TRIAXIS_NUM];

  // factory calibration
  ak0991x_cal_param cal_params[MAX_DEVICE_MODE_SUPPORTED];


  float                   placement[12];
  ak0991x_dae_if_info     dae_if;

  // debug
  uint16_t who_am_i;
  sns_sync_com_port_service *scp_service;
  size_t   encoded_event_len;
} ak0991x_state;

/** Functions shared by all AK0991X Sensors */
/**
 * notify_event() Sensor API common between all AK0991X Sensors.
 *
 * @param this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc ak0991x_sensor_notify_event(sns_sensor *const this
);

/**
 * set_client_request() Sensor API common between all AK0991X
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
sns_sensor_instance *ak0991x_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove
);

/**
 * Initializes Magnetic Sensor attributes.
 *
 * @param this   Sensor reference
 * @param[i]     device ID
 *
 * @return none
 */
void ak0991x_mag_init_attributes(sns_sensor *const this,
                                 akm_device_type device_select
);


sns_rc ak0991x_mag_init(sns_sensor *const this);
sns_rc ak0991x_mag_deinit(sns_sensor *const this);

sns_rc ak0991x_mag_match_odr(float desired_sample_rate,
                             float *chosen_sample_rate,
                             ak0991x_mag_odr *chosen_reg_value,
                             akm_device_type device_select,
                             float max_odr);

void ak0991x_update_registry(sns_sensor *const this,
        sns_sensor_instance *const instance);

void ak0991x_update_sensor_state(sns_sensor *const this,
        sns_sensor_instance *const instance);

