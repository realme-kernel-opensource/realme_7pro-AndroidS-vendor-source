#pragma once
/**
 * @file sns_lsm6dso_fsm.h
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
#include "sns_sensor_instance.h"

#define LSM6DSO_REG_FSM_ENABLE_A_ADDR   0x46
#define LSM6DSO_REG_FSM_ENABLE_B_ADDR	  0x47

#define LSM6DSO_FSM_BASE_ADDRESS        0x400
#define LSM6DSO_FSM_NFC_WRITE_ADDRESS   0x17c
#define LSM6DSO_FSM_START_WRITE_ADDRESS 0x17e
#define LSM6DSO_REG_FSM_EN_MASK         0x01
#define LSM6DSO_REG_FSM_INT1_A    0x0b
#define LSM6DSO_REG_FSM_INT2_A    0x0f
#define LSM6DSO_FSM_INIT_MASK     0x01
#define LSM6DSO_REG_FSM_INIT_B    0x67
#define LSM6DSO_REG_FSM_STATUS_MAINPAGE  0x36
#define LSM6DSO_REG_FSM_OUTS1     0x4C
#define LSM6DSO_REG_FSM_LC        0x48

/* Finite State Machine ODR configuration */
#define LSM6DSO_REG_EMB_FUNC_ODR_CFG_B_ADDR  0x5f
#define LSM6DSO_REG_FSM_ODR_MASK  0x18

#define LSM6DSO_FSM_ODR_12_5         0
#define LSM6DSO_FSM_ODR_26        0x08
#define LSM6DSO_FSM_ODR_52        0x10
#define LSM6DSO_FSM_ODR_104       0x18

#if LSM6DSO_FSM_ENABLED

void lsm6dso_init_fsm_instance(sns_sensor_instance *instance);

void lsm6dso_fsm_deinit(sns_sensor_instance *instance);

sns_rc lsm6dso_fsm_set_enable(sns_sensor_instance *instance, uint16_t sensor,
                                        xsensor_int int_line, bool enable);

void lsm6dso_fsm_get_interrupt_status(sns_sensor_instance *instance, uint8_t const *wake_src, uint8_t const *emb_src);

bool lsm6dso_check_fsm_sensor_interrupt(sns_sensor_instance *const instance, uint16_t idx);
#endif
