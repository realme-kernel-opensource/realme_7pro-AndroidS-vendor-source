/*============================================================================
@file sns_lps22hx_registry_default_config.c

Set default registry configuration in the absence of registry support

Copyright (c) 2019 STMicroelectronics.

All Rights Reserved.

STMicroelectronics Proprietary and Confidential.
============================================================================*/
#include "sns_sensor.h"
#include "sns_registry_util.h"
#include "sns_mem_util.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"

#if !LPS22HX_ENABLE_REGISTRY_ACCESS
#define SDM_845_BUILDS    0
#ifdef SSC_TARGET_SM8150
/** Using SM8150 Platform config as defaults. This is for
 *  test purpose only. All platform specific information will
 *  be available to the Sensor driver via Registry. */
#define LPS22HX_USE_I2C                1

#if LPS22HX_USE_I2C
#define LPS22HX_BUS_TYPE               SNS_BUS_I2C
#define LPS22HX_SLAVE_ADDRESS          0x5D
#define LPS22HX_BUS_INSTANCE           0x05
#define LPS22HX_BUS_MIN_FREQ           400
#define LPS22HX_BUS_MAX_FREQ           400
#else
#define LPS22HX_BUS_TYPE               SNS_BUS_SPI
#define LPS22HX_SLAVE_ADDRESS          0x0
#define LPS22HX_BUS_INSTANCE           0x01
#define LPS22HX_BUS_MIN_FREQ           0
#define LPS22HX_BUS_MAX_FREQ           33*100
#endif

#define LPS22HH_REG_ADDRESS_TYPE       SNS_REG_ADDR_8_BIT
#define LPS22HX_IRQ_NUM                118

#define LPS22HX_NUM_OF_RAILS           2
#define LPS22HX_RAIL_1                 "/pmic/client/sensor_vddio"
#define LPS22HX_RAIL_2                 "/pmic/client/sensor_vdd"
#define LPS22HX_RAIL_ON_STATE          SNS_RAIL_ON_NPM

#elif SDM_845_BUILDS
#define LPS22HX_BUS_TYPE               SNS_BUS_I2C
#define LPS22HX_SLAVE_ADDRESS          0x5D
#define LPS22HX_BUS_INSTANCE           0x1
#define LPS22HX_BUS_MIN_FREQ           400
#define LPS22HX_BUS_MAX_FREQ           400
#define LPS22HX_NUM_OF_RAILS           1
#define LPS22HX_RAIL_1                 "/pmic/client/sensor_vddio"
#define LPS22HH_REG_ADDRESS_TYPE       SNS_REG_ADDR_8_BIT
#define LPS22HX_RAIL_ON_STATE          SNS_RAIL_ON_NPM
#define LPS22HX_IRQ_NUM                119

#else
/* Added for PreSil verification when registry is not available.
 */
#define LPS22HX_BUS_TYPE               SNS_BUS_I2C
#define LPS22HX_SLAVE_ADDRESS          0x5C
#define LPS22HX_BUS_INSTANCE           0x5
#define LPS22HX_BUS_MIN_FREQ           400
#define LPS22HX_BUS_MAX_FREQ           400
#define LPS22HX_NUM_OF_RAILS           1
#define LPS22HX_RAIL_1                 "/pmic/client/sensor_vddio"
#define LPS22HH_REG_ADDRESS_TYPE       SNS_REG_ADDR_8_BIT
#define LPS22HX_RAIL_ON_STATE          SNS_RAIL_ON_NPM
#define LPS22HX_IRQ_NUM                129

#endif  // SSC_TARGET_SM8250
/**
 * Sensor platform resource configuration with hard coded values
 * @param this -- pointer to sensor
 * @param cfg -- pointer to cfg structure which will be filled in
 *               Caller should pass this to sns_lps22hx_save_registry_pf_cfg
*/
void sns_lps22hx_registry_def_config(sns_sensor *const this,
                                     sns_registry_phy_sensor_pf_cfg *cfg)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;

  sns_registry_phy_sensor_pf_cfg def_cfg = {
    .rigid_body_type =    0,
    .bus_type =           LPS22HX_BUS_TYPE,
    .bus_instance =       LPS22HX_BUS_INSTANCE,
    .slave_config =       LPS22HX_SLAVE_ADDRESS,
    .min_bus_speed_khz =  LPS22HX_BUS_MIN_FREQ,
    .max_bus_speed_khz =  LPS22HX_BUS_MAX_FREQ,
    .reg_addr_type =      LPS22HH_REG_ADDRESS_TYPE,
    .num_rail =           LPS22HX_NUM_OF_RAILS,
    .rail_on_state =      LPS22HX_RAIL_ON_STATE,
    .vddio_rail    =      LPS22HX_RAIL_1,
  };
  *cfg = def_cfg;
  state->is_dri = false;
}
#endif  //LPS22HX_ENABLE_REGISTRY_ACCESS