/*============================================================================
@file sns_lsm6ds3c_registry_default_config.c

Set default registry configuration in the absence of registry support

Copyright (c) 2019 Qualcomm Technologies, Inc.  All Rights Reserved.
Qualcomm Technologies Proprietary and Confidential.
============================================================================*/

#include "sns_lsm6ds3c_sensor.h"
#include "sns_lsm6ds3c_sensor_instance.h"
#include "sns_com_port_types.h"
#include "sns_types.h"
#include "sns_mem_util.h"

#if LSM6DS3C_REGISTRY_DISABLED
/** TODO Using 8996 Platform config as defaults. This is for
 *  test purpose only. All platform specific information will
 *  be available to the Sensor driver via Registry.
 *  otherwise, please change accordingly
 *  */

#if BUILD_DB
#define IRQ_NUM                    117
#define BUS_INSTANCE               0x01
#define BUS                        SNS_BUS_SPI
#define BUS_MIN_FREQ_KHZ           0  // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           3300 // 3.3MHz
#define SLAVE_CONTROL              0
#elif SDM_845_BUILDS
#define BUS_INSTANCE               0x02
#define BUS                        SNS_BUS_SPI
#define BUS_MIN_FREQ_KHZ           33*100  // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz
#define IRQ_NUM                    117
#define SLAVE_CONTROL              107
#elif defined(SSC_TARGET_SM6150) || defined(SSC_TARGET_SM7150)
#define BUS_INSTANCE               0x02
#define BUS                        SNS_BUS_SPI
#define BUS_MIN_FREQ_KHZ           33*100  // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz
#define IRQ_NUM                    81
#define SLAVE_CONTROL              0
#elif SSC_TARGET_SM7150
#define IRQ_NUM                    86
#define BUS_INSTANCE               0x01
#define BUS                        SNS_BUS_I3C_SDR
#define BUS_MIN_FREQ_KHZ           400    // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           12500 // 12.5MHz
#define SLAVE_CONTROL              107
#elif SSC_TARGET_MDM9205
#define IRQ_NUM                    26
#define BUS_INSTANCE               0x04
#define BUS                        SNS_BUS_I2C
#define BUS_MIN_FREQ_KHZ           400    // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           12500 // 12.5MHz
#define SLAVE_CONTROL              106
#define LSM6DS3C_ACCEL_MIN_ODR_IDX  LSM6DS3C_ACCEL_ODR13
#define LSM6DS3C_ACCEL_MAX_ODR_IDX  LSM6DS3C_ACCEL_ODR52
#else
#define IRQ_NUM                    132
#define BUS_INSTANCE               0x01
#define BUS                        SNS_BUS_I3C_SDR
#define BUS_MIN_FREQ_KHZ           400  // using a value of 0 will introduce more errors in timing
#define BUS_MAX_FREQ_KHZ           12500 // 12.5MHz
#define SLAVE_CONTROL              107
#define LSM6DS3C_ACCEL_MIN_ODR_IDX  LSM6DS3C_ACCEL_ODR26
#define LSM6DS3C_ACCEL_MAX_ODR_IDX  LSM6DS3C_ACCEL_ODR416
#endif

#define RAIL_1                     "/pmic/client/dummy_vdd"
#define NUM_OF_RAILS               1

#define I3C_ADDR                   10


#define LSM6DS3C_DEFAULT_REG_CFG_RAIL_ON         SNS_RAIL_ON_LPM
#define LSM6DS3C_DEFAULT_REG_CFG_ISDRI           1
#define LSM6DS3C_DEFAULT_REG_CFG_HW_ID           0
#define LSM6DS3C_DEFAULT_REG_ACCEL_RESOLUTION_IDX 2
#define LSM6DS3C_DEFAULT_REG_GYRO_RESOLUTION_IDX 3
#define LSM6DS3C_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM 0

/**
 * Sensor platform resource configuration with hrad coded values
 * @param this -- pointer to sensor
 * @param cfg -- pointer to cfg structure which will be filled in
 *               Caller should pass this to sensor_save_registry_pf_cfg
 */
void sns_lsm6ds3c_registry_def_config(sns_sensor *const this,
                                     sns_registry_phy_sensor_pf_cfg *cfg)
{
  lsm6ds3c_state *state = (lsm6ds3c_state*)this->state->state;
  lsm6ds3c_shared_state *shared_state = lsm6ds3c_get_shared_state(this);

  sns_registry_phy_sensor_pf_cfg def_cfg = {
    .slave_config =       SLAVE_CONTROL,
    .min_bus_speed_khz =  BUS_MIN_FREQ_KHZ,
    .max_bus_speed_khz =  BUS_MAX_FREQ_KHZ,
    .dri_irq_num =        IRQ_NUM,
#if LSM6DS3C_ODR_REGISTRY_FEATURE_ENABLE
    .max_odr =            500,
    .min_odr =            0,
#endif
    .bus_type =           BUS,
    .bus_instance =       BUS_INSTANCE,
    .reg_addr_type =      SNS_REG_ADDR_8_BIT,
    .irq_pull_type =      2,
    .irq_drive_strength = 0,
    .irq_trigger_type =   3,
    .num_rail =           NUM_OF_RAILS,
    .rail_on_state =      LSM6DS3C_DEFAULT_REG_CFG_RAIL_ON,
    .rigid_body_type =    0,
#if LSM6DS3C_USE_I3C
    .i3c_address =        I3C_ADDR,
#endif
    .irq_is_chip_pin =    1,
    .vddio_rail = RAIL_1,
    .vdd_rail = "",
  };

  *cfg = def_cfg;

  state->is_dri               = LSM6DS3C_DEFAULT_REG_CFG_ISDRI;
  state->hardware_id          = LSM6DS3C_DEFAULT_REG_CFG_HW_ID;
  state->supports_sync_stream = LSM6DS3C_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM;
  shared_state->inst_cfg.accel_resolution_idx = LSM6DS3C_DEFAULT_REG_ACCEL_RESOLUTION_IDX;
  shared_state->inst_cfg.gyro_resolution_idx = LSM6DS3C_DEFAULT_REG_GYRO_RESOLUTION_IDX;
  shared_state->inst_cfg.ag_stream_mode = state->is_dri;  
  shared_state->inst_cfg.min_odr_idx = (LSM6DS3C_ACCEL_MIN_ODR_IDX >> 4);
  shared_state->inst_cfg.max_odr_idx = (LSM6DS3C_ACCEL_MAX_ODR_IDX >> 4);
  sns_memset(shared_state->placement, 0, sizeof(shared_state->placement));

  shared_state->inst_cfg.axis_map[0] = (triaxis_conversion)
    { .ipaxis = TRIAXIS_X,
      .opaxis = TRIAXIS_X,
      .invert = true, };
  shared_state->inst_cfg.axis_map[1] = (triaxis_conversion)
    { .ipaxis = TRIAXIS_Y,
      .opaxis = TRIAXIS_Y,
      .invert = true, };
  shared_state->inst_cfg.axis_map[2] = (triaxis_conversion)
    { .ipaxis = TRIAXIS_Z,
      .opaxis = TRIAXIS_Z,
      .invert = false, };


  shared_state->inst_cfg.accel_cal.fac_cal_corr_mat.e00 = 1;
  shared_state->inst_cfg.accel_cal.fac_cal_corr_mat.e11 = 1;
  shared_state->inst_cfg.accel_cal.fac_cal_corr_mat.e22 = 1;
  shared_state->inst_cfg.accel_cal.fac_cal_bias[0] =
  shared_state->inst_cfg.accel_cal.fac_cal_bias[1] =
  shared_state->inst_cfg.accel_cal.fac_cal_bias[2] = 0;
  shared_state->inst_cfg.accel_cal.ts = sns_get_system_time();
  
  shared_state->inst_cfg.gyro_cal.fac_cal_corr_mat.e00 = 1;
  shared_state->inst_cfg.gyro_cal.fac_cal_corr_mat.e11 = 1;
  shared_state->inst_cfg.gyro_cal.fac_cal_corr_mat.e22 = 1;
  shared_state->inst_cfg.gyro_cal.fac_cal_bias[0] =
  shared_state->inst_cfg.gyro_cal.fac_cal_bias[1] =
  shared_state->inst_cfg.gyro_cal.fac_cal_bias[2] = 0;
  shared_state->inst_cfg.gyro_cal.ts = sns_get_system_time();

  shared_state->inst_cfg.md_config.thresh = 0.6132f;
  shared_state->inst_cfg.md_config.win = 0.0f;
  shared_state->inst_cfg.md_config.disable = 0;
}
#endif //LSM6DS3C_REGISTRY_DISABLED
