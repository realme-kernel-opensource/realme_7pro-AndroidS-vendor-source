#pragma once
/**
 * @file sns_lps22hx_hal.h
 *
 * Hardware Access Layer functions.
 *
 * Copyright (c) 2019, STMicroelectronics.
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
 **/

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"

#include "sns_lps22hx_sensor_instance.h"

/**
 *  Address registers
 */
/**
 * Pressure sensor LPS22HX register addresses
 */
#define STM_LPS22HX_INTERRUPT_CFG   0x0B
#define STM_LPS22HX_THS_P_L         0x0C
#define STM_LPS22HX_THS_P_H         0x0D
#define STM_LPS22HX_WHO_AM_I        0x0F    //ro, =0xB1(LPS22HB), 0xB3(LPS22HH)
#define STM_LPS22HX_CTRL_REG1       0x10
#define STM_LPS22HX_CTRL_REG2       0x11
#define STM_LPS22HX_CTRL_REG3       0x12
#if LPS22HX_DEVICE_LPS22HB
#define STM_LPS22HX_FIFO_CTRL       0x14
#else
#define STM_LPS22HX_FIFO_CTRL       0x13
#endif
#define STM_LPS22HX_REF_P_XL        0x15
#define STM_LPS22HX_REF_P_L         0x16
#define STM_LPS22HX_REF_P_H         0x17
#define STM_LPS22HX_RPDS_L          0x18
#define STM_LPS22HX_RPDS_H          0x19
#define STM_LPS22HX_RES_CONF        0x1A
#if LPS22HX_DEVICE_LPS22HB
#define STM_LPS22HX_INT_SOURCE      0x25
#define STM_LPS22HX_FIFO_STATUS     0x26
#define STM_LPS22HX_ACTIVE_CURRENT  4
#else
#define STM_LPS22HX_INT_SOURCE      0x24
#define STM_LPS22HX_FIFO_STATUS1    0x25
#define STM_LPS22HX_ACTIVE_CURRENT  12
#endif
#define STM_LPS22HX_STATUS_REG      0x27
#define STM_LPS22HX_PRESS_OUT_XL    0x28
#define STM_LPS22HX_PRESS_OUT_L     0x29
#define STM_LPS22HX_PRESS_OUT_H     0x2A
#define STM_LPS22HX_TEMP_OUT_L      0x2B
#define STM_LPS22HX_TEMP_OUT_H      0x2C

/**
 * Pressure sensor LPS22HX WHO AM I register
 */
#if LPS22HX_DEVICE_LPS22HB
#define LPS22HX_WHOAMI_VALUE        0xB1
#else
#define LPS22HX_WHOAMI_VALUE        0xB3
#endif

#define LPS22HX_NUM_BYTES_PRESS 3
#define LPS22HX_NUM_BYTES_TEMP  2

#define LPS22HX_OFF_TO_IDLE_MS      5  //ms

#define LPS22HX_PRESS_NUM_AXES  1
#define LPS22HX_TEMP_NUM_AXES   1

/******************* Function Declarations ***********************************/

/**
 * Resets the Sensor HW. Also calls
 * lps22hx_device_set_default_state()
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] sensor        bit mask for sensors to reset
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lps22hx_reset_device(
    sns_sensor_instance *const instance,
    lps22hx_sensor_type sensor);

/**
 * Loads default values in config registers.
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] sensor        bit mask for sensors to handle
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lps22hx_device_set_default_state(
    sns_sensor_instance *const instance,
    lps22hx_sensor_type sensor);

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] scp_service   handle to synch COM port service
 * @param[i] state         Instance state
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lps22hx_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer);

/**
 * Reads status registers in Instance State.
 * This function is for debug only.
 *
 * @param[i] state                 Instance state
 * @param[i] sensor                bit mask of Sensors to enabl
 *
 * @return none
 */
void lps22hx_dump_reg(sns_sensor_instance *const instance, lps22hx_sensor_type sensor);

/**
 * Sends pressure &  temperature event.
 *
 * @param[i] instance   Sensor Instance
 */
void lps22hx_handle_sensor_sample(sns_sensor_instance *const instance, sns_time timestamp);

/**
 * Starts/restarts polling timer
 *
 * @param instance   Instance reference
 */
void lps22hx_start_sensor_polling_timer(sns_sensor_instance *this);

/**
 * Updates temp & press sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
void lps22hx_set_polling_config(sns_sensor_instance *const this);

/**
 * Handle an interrupt by reading the Fifo status register and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param instance                 Sensor Instance
 */
void lps22hx_handle_interrupt_event(sns_sensor_instance *const instance);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @param[i] instance    reference to this Instance
 */
void lps22hx_send_config_event(sns_sensor_instance *const instance);

sns_rc  lps22hx_set_odr(sns_sensor_instance *const instance, uint8_t idx);

sns_rc lps22hx_enable_intr(sns_sensor_instance *const instance);
