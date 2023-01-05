/*******************************************************************************
 * Copyright (c) 2017-2020, Bosch Sensortec GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
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
 *******************************************************************************/

#include "sns_bmi26x_sensor.h"
#include "sns_bmi26x_sensor_instance.h"
#include "sns_com_port_types.h"
#include "sns_types.h"
#include "sns_mem_util.h"

#include "sns_bmi26x_config.h"

#if !BMI26X_CONFIG_ENABLE_REGISTRY   || BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT

#if BUILD_DB
#define IRQ_NUM                        117
#define SPI_BUS_INSTANCE               0x01
#define SPI_BUS_MIN_FREQ_KHZ           0  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz

#elif SDM_845_BUILDS
#define IRQ_NUM                        117
#define SPI_BUS_INSTANCE               0x02
#define SPI_BUS_MIN_FREQ_KHZ           33*100  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz

#elif SSC_TARGET_SM6150
#define IRQ_NUM                        81
#define SPI_BUS_INSTANCE               0x02
#define SPI_BUS_MIN_FREQ_KHZ           33*100  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz

#elif SSC_TARGET_SM7150
#define IRQ_NUM                        86
#define SPI_BUS_INSTANCE               0x01
#define SPI_BUS_MIN_FREQ_KHZ           33*100  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz

#else
    //@@@845
#define IRQ_NUM                        117
#define SPI_BUS_INSTANCE               0x02
#define SPI_BUS_MIN_FREQ_KHZ           7*100  // using a value of 0 will introduce more errors in timing
#define SPI_BUS_MAX_FREQ_KHZ           96*100 // 9.6MHz
#define SPI_SLAVE_CONTROL              0x0
#endif

#define RAIL_1                         "/pmic/client/sensor_vddio"
#define RAIL_2                         "/pmic/client/sensor_vdd"
#define NUM_OF_RAILS                   2


#define SENSOR_I2C_SLAVE_ADDRESS       0x68
#define I2C_BUS_INSTANCE               0x03
#define I2C_BUS_MIN_FREQ_KHZ           400
#define I2C_BUS_MAX_FREQ_KHZ           400



#define BMI26X_DEFAULT_REG_CFG_RAIL_ON         SNS_RAIL_ON_NPM


// <motion detection>
#define BMI26X_DEFAULT_REG_CFG_MD_THRESH          (0.6132f)
#define BMI26X_DEFAULT_REG_CFG_MD_ENABLE          (true)
#define BMI26X_DEFAULT_REG_CFG_MD_WIN             (0.0f)

#define BMI160_DEFAULT_REG_CFG_RIGIT_BODY_TYPE    0

#define bmi26x_sensor_com_port_spi_config_init_default   { \
  .bus_type          = SNS_BUS_SPI,            \
  .slave_control     = SPI_SLAVE_CONTROL,      \
  .reg_addr_type     = SNS_REG_ADDR_8_BIT,     \
  .min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ,   \
  .max_bus_speed_KHz = SPI_BUS_MAX_FREQ_KHZ,   \
  .bus_instance      = SPI_BUS_INSTANCE        \
  }


#define bmi26x_sensor_com_port_i2c_config_init_default   { \
  .bus_type          = SNS_BUS_I2C,            \
  .slave_control     = SENSOR_I2C_SLAVE_ADDRESS,   \
  .reg_addr_type     = SNS_REG_ADDR_8_BIT,     \
  .min_bus_speed_KHz = I2C_BUS_MIN_FREQ_KHZ,   \
  .max_bus_speed_KHz = I2C_BUS_MAX_FREQ_KHZ,   \
  .bus_instance      = I2C_BUS_INSTANCE        \
  }

#define bmi26x_sensor_irq_config_init_default { \
  .interrupt_num            = IRQ_NUM,\
  .interrupt_trigger_type   = SNS_INTERRUPT_TRIGGER_TYPE_RISING,\
  .interrupt_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP, \
  .interrupt_pull_type      = SNS_INTERRUPT_PULL_TYPE_KEEPER,\
  .is_chip_pin              = true \
  }


#define BMI26X_DEFAULT_REG_CFG_ACCEL_RES_IDEX  BMI26X_RANGE_ACC_PM8G
#define BMI26X_DEFAULT_REG_CFG_GYRO_RES_IDEX   BMI26X_RANGE_GYR_PM2000DPS
#define BMI26X_DEFAULT_REG_CFG_ISDRI           1
#define BMI26X_DEFAULT_REG_CFG_HW_ID           1
#define BMI26X_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM 0



/**
 * Sensor platform resource configuration with Hard Code
 * @param this
 */
void sns_bmi26x_registry_def_config(sns_sensor *const this)
{
    bmi26x_state* sstate = (bmi26x_state *) this->state->state;
    sns_com_port_config *com_port_cfg = &sstate->common.com_port_info.com_config;

    BMI26X_SENSOR_LOG(LOW, this, "registry_cfg dft: %d", sstate->sensor);
    // <general config>
    {
        sstate->is_dri         = BMI26X_DEFAULT_REG_CFG_ISDRI;
        sstate->hardware_id    = BMI26X_DEFAULT_REG_CFG_HW_ID;

        if (sstate->sensor == BMI26X_ACCEL) {
            sstate->resolution_idx = BMI26X_DEFAULT_REG_CFG_ACCEL_RES_IDEX;
        } else if (sstate->sensor == BMI26X_GYRO) {
            sstate->resolution_idx = BMI26X_DEFAULT_REG_CFG_GYRO_RES_IDEX;
        } else {
        }

        sstate->supports_sync_stream = BMI26X_DEFAULT_REG_CFG_SUPPORT_SYN_STREAM;
    sstate->registry_cfg_received = true;

    sstate->common.registry_cfg.is_dri = sstate->is_dri;
    sstate->common.registry_cfg.hw_id = sstate->hardware_id;
    sstate->common.registry_cfg.res_idx = sstate->resolution_idx;
    sstate->common.registry_cfg.sync_stream = sstate->supports_sync_stream;
    }

    // <platform configure>
    {
#if BMI26X_CONFIG_DFT_BUS_SPI
        sns_memscpy(com_port_cfg,
                    sizeof(sstate->common.com_port_info.com_config),
                    &((sns_com_port_config) bmi26x_sensor_com_port_spi_config_init_default ),
                    sizeof(sns_com_port_config));

#else  // I2C
        sns_memscpy(com_port_cfg,
                    sizeof(sstate->common.com_port_info.com_config),
                    &((sns_com_port_config) bmi26x_sensor_com_port_i2c_config_init_default),
                    sizeof(sns_com_port_config));
#endif

        sns_memscpy(&sstate->common.irq_config,
                    sizeof(sstate->common.irq_config),
                    &((sns_interrupt_req) bmi26x_sensor_irq_config_init_default ),
                    sizeof(sns_interrupt_req));

        sstate->common.registry_rail_on_state  = BMI26X_DEFAULT_REG_CFG_RAIL_ON;
        strlcpy(sstate->common.rail_config.rails[0].name,
                RAIL_1,
                sizeof(sstate->common.rail_config.rails[0].name));
        strlcpy(sstate->common.rail_config.rails[1].name,
                RAIL_2,
                sizeof(sstate->common.rail_config.rails[1].name));
        sstate->common.rail_config.num_of_rails = 2;

        sstate->common.registry_pf_cfg_received = true;
    }


    // <placement>
    {
        uint8_t i;
        for (i = 0; i < 9; i ++) {
            sstate->common.placement[i] = 0;
        }
        sstate->common.registry_placement_received = true;
    }

#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
    // <orient>
    {
        uint8_t i;
        //done in bmi26x_common_init
        if (sstate->sensor == BMI26X_ACCEL
                || sstate->sensor == BMI26X_GYRO) {
            // initialize axis conversion settings
            for (i = 0; i < TRIAXIS_NUM; i++) {
                sstate->common.axis_map[i].opaxis = (triaxis)i;
                sstate->common.axis_map[i].ipaxis = (triaxis)i;
                sstate->common.axis_map[i].invert = false;
            }
        }
        sstate->common.registry_orient_received = true;
    }
    // <fac_cal>
    {
        // <fac_cal/bias>
        {
            float *sensor_fac_cal_bias = sstate->fac_cal_bias;
            if (sstate->sensor == BMI26X_ACCEL) {
                sensor_fac_cal_bias[0] = 0.01;
                sensor_fac_cal_bias[1] = 0.01;
                sensor_fac_cal_bias[2] = 0.01;
            } else if (sstate->sensor == BMI26X_GYRO) {
                sensor_fac_cal_bias[0] = 0.0;
                sensor_fac_cal_bias[1] = 0.0;
                sensor_fac_cal_bias[2] = 0.0;
            }
        }
        // <fac cal/corr_mat>
        {
            if ((sstate->sensor == BMI26X_ACCEL) ||
                    (sstate->sensor == BMI26X_GYRO)) {
                uint8_t i;
                for (i = 0; i < 9; i ++) {
                    sstate->fac_cal_corr_mat.data[i] = 0.0;
                }
                sstate->fac_cal_corr_mat.e00 = 1.0;
                sstate->fac_cal_corr_mat.e11 = 1.0;
                sstate->fac_cal_corr_mat.e22 = 1.0;
            }
        }
        // <fac/scale>
        {
            uint8_t i;
            for (i = 0; i < TRIAXIS_NUM; i++) {
                sstate->fac_cal_scale[i] = 1.0;
            }
        }
        sstate->registry_fac_cal_received = true;
    }
#endif

    // <mag/threshold>
    {
        if (sstate->sensor == BMI26X_MOTION_DETECT) {
            sstate->md_config.disable = !BMI26X_DEFAULT_REG_CFG_MD_ENABLE;
            sstate->md_config.thresh  = BMI26X_DEFAULT_REG_CFG_MD_THRESH;
            sstate->md_config.win     = BMI26X_DEFAULT_REG_CFG_MD_WIN;
            sstate->registry_md_config_received = true;
        }
    }
    sstate->common.registry_pf_cfg.rigid_body_type = BMI160_DEFAULT_REG_CFG_RIGIT_BODY_TYPE;
}
#endif
