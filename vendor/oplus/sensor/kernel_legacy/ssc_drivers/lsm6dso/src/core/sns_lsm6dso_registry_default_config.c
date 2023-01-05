/**
 * @file sns_lsm6dso_registry_default_config.c
 *
 * Set default registry configuration in the absence of registry support.
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
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"
#include "sns_com_port_types.h"
#include "sns_types.h"
#include "sns_mem_util.h"


#if LSM6DSO_REGISTRY_DISABLED
#include "sns_interface_defs.h"
#define IRQ_NUM                    _LSM6DSO_IRQ_NUM_0
#define IRQ_NUM_1                  _LSM6DSO_IRQ_NUM_1
#define BUS_INSTANCE               _LSM6DSO_BUS_INSTANCE
#define BUS                        _LSM6DSO_BUS_TYPE
#define BUS_MIN_FREQ_KHZ           _LSM6DSO_BUS_FREQ_MIN
#define BUS_MAX_FREQ_KHZ           _LSM6DSO_BUS_FREQ_MAX
#define SLAVE_CONTROL              _LSM6DSO_I2C_ADDR_0
#define SLAVE_CONTROL_1            _LSM6DSO_I2C_ADDR_1
#define RAIL_1                     _LSM6DSO_RAIL_1
#define NUM_OF_RAILS               _LSM6DSO_NUM_RAILS
#define I3C_ADDR                   _LSM6DSO_I3C_ADDR_0
#define I3C_ADDR_1                 _LSM6DSO_I3C_ADDR_1

#define LSM6DSO_DEFAULT_REG_CFG_RAIL_ON         SNS_RAIL_ON_LPM
#define LSM6DSO_DEFAULT_REG_CFG_ISDRI           _LSM6DSO_IRQ_MODE
#define LSM6DSO_DEFAULT_REG_CFG_HW_ID           0
#define LSM6DSO_DEFAULT_REG_ACCEL_RESOLUTION_IDX 2
#define LSM6DSO_DEFAULT_REG_GYRO_RESOLUTION_IDX 3
#define LSM6DSO_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM 0

/**
 * Sensor platform resource configuration with hrad coded values
 * @param this -- pointer to sensor
 * @param cfg -- pointer to cfg structure which will be filled in
 *               Caller should pass this to sensor_save_registry_pf_cfg
 */
void sns_lsm6dso_registry_def_config(sns_sensor *const this,
                                     sns_registry_phy_sensor_pf_cfg *cfg)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);

  sns_registry_phy_sensor_pf_cfg def_cfg = {
    .slave_config =       SLAVE_CONTROL,
    .min_bus_speed_khz =  BUS_MIN_FREQ_KHZ,
    .max_bus_speed_khz =  BUS_MAX_FREQ_KHZ,
    .dri_irq_num =        IRQ_NUM,
#if LSM6DSO_ODR_REGISTRY_FEATURE_ENABLE
    .max_odr =            500,
    .min_odr =            20,
#endif
    .bus_type =           BUS,
    .bus_instance =       BUS_INSTANCE,
    .reg_addr_type =      SNS_REG_ADDR_8_BIT,
    .irq_pull_type =      2,
    .irq_drive_strength = 0,
    .irq_trigger_type =   3,
    .num_rail =           NUM_OF_RAILS,
    .rail_on_state =      LSM6DSO_DEFAULT_REG_CFG_RAIL_ON,
    .rigid_body_type =    RIGID_BODY_TYPE,
,
#if LSM6DSO_USE_I3C
    .i3c_address =        I3C_ADDR,
#endif
    .irq_is_chip_pin =    1,
    .vddio_rail = RAIL_1,
    .vdd_rail = "",
  };

#if defined(LSM6DSO_ENABLE_DUAL_SENSOR)
  if( shared_state->hw_idx == 1 )
  {
    def_cfg.slave_config    = SLAVE_CONTROL_1;
    def_cfg.dri_irq_num     = IRQ_NUM_1;
    def_cfg.rigid_body_type = RIGID_BODY_TYPE_1;
    def_cfg.i3c_address     = I3C_ADDR_1;
  }
#endif

  *cfg = def_cfg;

  state->is_dri               = LSM6DSO_DEFAULT_REG_CFG_ISDRI;
  state->hardware_id          = LSM6DSO_DEFAULT_REG_CFG_HW_ID;
  state->supports_sync_stream = LSM6DSO_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM;
  shared_state->inst_cfg.accel_resolution_idx = LSM6DSO_DEFAULT_REG_ACCEL_RESOLUTION_IDX;
  shared_state->inst_cfg.gyro_resolution_idx = LSM6DSO_DEFAULT_REG_GYRO_RESOLUTION_IDX;

  sns_memset(shared_state->placement, 0, sizeof(shared_state->placement));

#if defined(SSC_TARGET_SM6150) || defined(SSC_TARGET_SM7150)
  shared_state->inst_cfg.ag_stream_mode = DRI;
#endif
  lsm6dso_init_inst_config(this, shared_state);
}
#endif //LSM6DSO_REGISTRY_DISABLED
