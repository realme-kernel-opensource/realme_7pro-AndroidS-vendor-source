#pragma once
/**
 * @file sns_mmc5603x_sensor.h
 *
 * MMC5603X Sensor implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_mmc5603x_lite.h"
#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_sensor_uid.h"
#include "sns_pwr_rail_service.h"
#include "sns_suid_util.h"

#include "sns_mmc5603x_hal.h"

#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_dae_if.h"
#include "sns_math_util.h"
#include "sns_diag_service.h"
#include "sns_registry_util.h"



#define MAG_SUID1 \
  {  \
    .sensor_uid =  \
    {  \
      0x5b, 0x79, 0x1d, 0xb7, 0x69, 0x08, 0x45, 0xac,  \
      0xb0, 0x48, 0x90, 0xc9, 0x17, 0xa5, 0xbb, 0x64  \
    }  \
  }

#ifdef MMC5603X_ENABLE_DUAL_SENSOR
#define MAG_SUID2 \
  {  \
    .sensor_uid =  \
    {  \
      0xaf, 0xf9, 0x3c, 0x5a, 0xc8, 0xc5, 0x45, 0x70, \
      0xb4, 0xbe, 0xc1, 0xa5, 0x7b, 0x8d, 0x5f, 0xec  \
    }  \
  }
#endif

//#ifdef MMC5603X_ENABLE_REGISTRY_ACCESS
#define MMC5603X_STR                     "mmc5603x_"
#define MMC5603X_PLATFORM_CONFIG_STR     "_platform.config"
#define MMC5603X_PLATFORM_PLACEMENT_STR  "_platform.placement"
#define MMC5603X_PLATFORM_ORIENT_STR     "_platform.orient"
#define MMC5603X_PLATFORM_FACCAL_STR     "_platform.mag.fac_cal"
#define MMC5603X_MAG_CONFIG_STR          ".mag.config"
#define MMC5603X_REG_CONFIG_STR          ".mag.config_2"
//#endif // MMC5603X_ENABLE_REGISTRY_ACCESS


/** TODO Using 8996 Platform config as defaults. This is for
 *  test purpose only. All platform specific information will
 *  be available to the Sensor driver via Registry. */
#define RAIL_1                     "/pmic/client/sensor_vddio"
//#define RAIL_2                     "/pmic/client/sensor_vdd"
#define NUM_OF_RAILS               1
#define I2C_BUS_FREQ               400
#define I2C_SLAVE_ADDRESS          0x30
#define BUS_TYPE                   SNS_BUS_I3C_SDR
//#define BUS_TYPE                   SNS_BUS_I2C
#define I3C_ADDR                   50    //Dynamic address
#define I2C_BUS_INSTANCE           0x03
#define BUS_FREQ_MIN               400
#define BUS_FREQ_MAX               12500



/** Forward Declaration of Magnetic Sensor API */
extern sns_sensor_api mmc5603x_mag_sensor_api;

/**
 * MMC5603X ODR definitions
 */
#define MMC5603X_ODR_0              0.0
#define MMC5603X_ODR_5              5.0
#define MMC5603X_ODR_10             10.0
#define MMC5603X_ODR_15             15.0
#define MMC5603X_ODR_25             25.0
#define MMC5603X_ODR_50             50.0
#define MMC5603X_ODR_100            100.0
//#define MMC5603X_ODR_200            200.0

/**
 * Magnetometer resolution
 */
#define MMC5603X_RESOLUTION         (0.0976f) /* uT/LSB */
#define MMC5603X_HI_PWR             1000 /* unit of uA @ 100Hz */
#define MMC5603X_LO_PWR             1    /* unit of uA */
#define MMC5603X_MIN_RANGE          -3000 /* Minimum -4912uT */
#define MMC5603X_MAX_RANGE          3000  /* Maximum  4912uT */


/** Supported opertating modes */
#define MMC5603X_NORMAL             "NORMAL"
#define MMC5603X_ERROR_POWER          "ERROR_POWER"
#define MMC5603X_ERROR_NOISE          "ERROR_NOISE"

#define MMC5603X_NUM_OF_ATTRIBUTES  (21)

/** Power rail timeout States for the MMC5603X Sensors.*/

#define MMC5603X_POWER_RAIL_OFF_TIMEOUT_NS 1000000000ULL /* 1 second */
typedef enum
{
  MMC5603X_POWER_RAIL_PENDING_NONE,
  MMC5603X_POWER_RAIL_PENDING_INIT,
  MMC5603X_POWER_RAIL_PENDING_SET_CLIENT_REQ,
  MMC5603X_POWER_RAIL_PENDING_OFF,
} mmc5603x_power_rail_pending_state;

#ifdef MMC5603X_ENABLE_REGISTRY_ACCESS
/** Registry items supported as part of physical sensor
 *  configuraton registry group
 */
typedef struct mmc5603x_registry_phy_sensor_cfg
{
  bool    use_fifo;
  uint8_t nsf;
  uint8_t sdr;
} mmc5603x_registry_phy_sensor_cfg;
#endif



/** Interrupt Sensor State. */

typedef struct mmc5603x_state
{
  sns_data_stream       *reg_data_stream;
  sns_data_stream       *fw_stream;
  sns_data_stream       *timer_stream;

  // Registry, IRQ, Timer, ASCP, DAE, device_mode
  SNS_SUID_LOOKUP_DATA(6) suid_lookup_data;

  mmc5603x_com_port_info com_port_info;
  sns_interrupt_req      irq_config;

  sns_pwr_rail_service  *pwr_rail_service;
  sns_rail_config       rail_config;
  sns_power_rail_state  registry_rail_on_state;

  bool hw_is_present;
  bool sensor_client_present;
  bool remove_timer_stream;

  mmc5603x_power_rail_pending_state power_rail_pend_state;

  // parameters which are determined when the connected device is specified.
  memsic_device_type device_select; // store the current connected device
  float sstvt_adj[MMC5603X_NUM_SENSITIVITY];

  // sensor configuration
  uint8_t is_dri;
  bool supports_sync_stream;
  bool use_fifo;
  uint8_t nsf;
  uint8_t sdr;
  uint8_t resolution_idx;
  int64_t hardware_id;
#ifdef MMC5603X_ENABLE_DUAL_SENSOR
  uint32_t registration_idx;
#endif

#ifdef MMC5603X_ENABLE_REGISTRY_ACCESS
  // registry sensor config
  bool registry_cfg_received;
  sns_registry_phy_sensor_cfg registry_cfg;
  // registry sensor reg config
  bool registry_reg_cfg_received;
  mmc5603x_registry_phy_sensor_cfg registry_reg_cfg;
  // registry sensor platform config
  bool registry_pf_cfg_received;
  sns_registry_phy_sensor_pf_cfg registry_pf_cfg;
  // axis conversion
  bool registry_orient_received;
  bool registry_placement_received;
#endif

  triaxis_conversion axis_map[TRIAXIS_NUM];

  // factory calibration
  mmc5603x_cal_param cal_parameter[MAX_DEVICE_MODE_SUPPORTED];
  uint32_t                fac_cal_version;

  // placement
  float                   placement[12];
  mmc5603x_dae_if_info     dae_if;

  // debug
  uint16_t who_am_i;
  sns_diag_service *diag_service;
  sns_sync_com_port_service *scp_service;
  size_t   encoded_event_len;
} mmc5603x_state;

/** Functions shared by all MMC5603X Sensors */
/**
 * notify_event() Sensor API common between all MMC5603X Sensors.
 *
 * @param this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc mmc5603x_sensor_notify_event(sns_sensor *const this
);

/**
 * set_client_request() Sensor API common between all MMC5603X
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
sns_sensor_instance *mmc5603x_set_client_request(sns_sensor *const this,
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
void mmc5603x_mag_init_attributes(sns_sensor *const this,
                                 memsic_device_type device_select
);


sns_rc mmc5603x_mag_init(sns_sensor *const this);
#ifdef MMC5603X_ENABLE_DEINIT
sns_rc mmc5603x_mag_deinit(sns_sensor *const this);
#endif

sns_rc mmc5603x_mag_match_odr(float desired_sample_rate,
                             float *chosen_sample_rate,
                             mmc5603x_mag_odr *chosen_reg_value,
                             memsic_device_type device_select);
void mmc5603x_update_registry(sns_sensor *const this,
        sns_sensor_instance *const instance);

void mmc5603x_update_sensor_state(sns_sensor *const this,
        sns_sensor_instance *const instance);
