#pragma once
/**
 * @file sns_lsm6dso_mlc.h
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

#include "sns_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"

#define LSM6DSO_REG_MLC_EN_MASK   0x10
#define LSM6DSO_REG_MLC_INT1      0x0d
#define LSM6DSO_REG_MLC_INT2      0x11
#define LSM6DSO_MLC_INIT_MASK     0x10
#define LSM6DSO_REG_MLC_INIT_B    0x67

/* Finite State Machine ODR configuration */
#define LSM6DSO_REG_EMB_FUNC_ODR_CFG_C_ADDR  0x60
#define LSM6DSO_REG_MLC_ODR_MASK  0x30

#define LSM6DSO_MLC_ODR_12_5         0
#define LSM6DSO_MLC_ODR_26        0x10
#define LSM6DSO_MLC_ODR_52        0x20
#define LSM6DSO_MLC_ODR_104       0x30
#define STM_LSM6DSO_REG_MLC_STATUS_MAINPAGE  0x38
#define STM_LSM6DSO_REG_MLC_STATUS_EMB       0x15
#define STM_LSM6DSO_REG_MLC_SRC   0x70

#if LSM6DSO_MLC_ENABLED

void lsm6dso_init_mlc_instance(sns_sensor_instance *instance);

void lsm6dso_mlc_deinit(sns_sensor_instance *instance);

sns_rc lsm6dso_mlc_set_enable(sns_sensor_instance *instance, uint16_t sensor,
                                        xsensor_int int_line, bool enable);

bool lsm6dso_mlc_get_interrupt_status(sns_sensor_instance *instance, uint8_t const *wake_src, uint8_t const *emb_src);

bool lsm6dso_mlc_check_sensor_interrupt(sns_sensor_instance *const instance, uint16_t idx);

bool lsm6dso_mlc_send_sensor_value(sns_sensor_instance *const instance, sns_sensor_uid const *sensor_uid, sns_time irq_timestamp, uint8_t oem_inst, lsm6dso_sensor_type sensor);

#endif
