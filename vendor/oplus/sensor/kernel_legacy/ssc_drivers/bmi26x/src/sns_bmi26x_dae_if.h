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

#pragma once
/**
 * @file sns_bmi26x_dae_if.h
 *
 * DAE sensor interface
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <stdint.h>
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_stream_service.h"

#include "sns_dae.pb.h"

struct sns_stream_service;
struct sns_data_stream;
struct bmi26x_instance_state;

#define BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME        6
#define BMI26X_TRY_NUM_ON_CHECK_GPIO                     2

/*!
 *  DAE interface status
 */
typedef enum {
    BMI26X_DAE_IF_PRE_INIT,             //!< BMI26X_DAE_IF_PRE_INIT
    BMI26X_DAE_IF_INIT_PENDING,         //!< BMI26X_DAE_IF_INIT_PENDING
    BMI26X_DAE_IF_UNAVAILABLE,          //!< BMI26X_DAE_IF_UNAVAILABLE
    BMI26X_DAE_IF_STATE_IDLE,           //!< BMI26X_DAE_IF_STATE_IDLE
    BMI26X_DAE_IF_STATE_STREAM_STARTING,//!< BMI26X_DAE_IF_STATE_STREAM_STARTING
    BMI26X_DAE_IF_STATE_STREAMING,      //!< BMI26X_DAE_IF_STATE_STREAMING
    BMI26X_DAE_IF_STATE_STREAM_STOPPING,//!< BMI26X_DAE_IF_STATE_STREAM_STOPPING

} bmi26x_dae_if_state_t;

/*!
 * The structure definition for DAE stream
 */
typedef struct {
    struct sns_data_stream        *stream;
    const char                    *nano_hal_vtable_name;
    bmi26x_dae_if_state_t         state;
    bool                          stream_usable : 1;
    bool                          flushing_hw    : 1;
    bool                          flushing_data : 1;
    bool                          fifo_int_pending : 1;
} bmi26x_dae_stream_t;


/*!
 * The structure definition for DAE event context
 */
typedef struct bmi26x_dae_resp_context {
    bmi26x_dae_stream_t *ag_stream;
    bmi26x_dae_stream_t *temp_stream;
    sns_dae_resp ag_resp;
    sns_dae_resp temp_resp;
} bmi26x_dae_resp_context_t;


/*!
 * The structure definition for DAE interface information
 */
typedef struct bmi26x_dae_if_info {
    sns_sensor_uid                    dae_suid;
    bmi26x_dae_if_state_t             dae_ag_state;
    bmi26x_dae_if_state_t             dae_temp_state;
    bmi26x_dae_stream_t               ag;
    bmi26x_dae_stream_t               temp;
    sns_time                          ts_dae_read_fifo_dev_time;
    sns_time                          ts_dae_read_fifo_sys_time;
} bmi26x_dae_if_info_t;





// for use by master sensor

/*!
 * Intialization interface for DAE feature
 *
 * @param[in]   this  The sensor handler
 */
void bmi26x_dae_if_check_support(sns_sensor *this);
void bmi26x_dae_if_process_sensor_events(sns_sensor *this);

/*!
 * DAE initialization interface
 *
 * @param[in] ssensor        The sensor handler
 * @param[in] this           The instance used currently
 * @param[in] stream_mgr     The stream service handler
 * @param[in] dae_suid       The DAE suid
 */
sns_rc bmi26x_dae_if_init(
    sns_sensor *const ssensor,
    sns_sensor_instance           *const this,
    struct sns_stream_service     *stream_mgr,
    sns_sensor_uid                *dae_suid);

/*!
 * DAE deinitialization interface
 *
 * @param[in] this     The instance used currently
 */
void bmi26x_dae_if_deinit(sns_sensor_instance *const this);


/*!
 * DAE Stop Data streaming interface
 *
 * @param[in] this           The instance handler
 * @param[in] sensors        Sensor which should be stop
 * @return               TRUE on success
 *                       FALSE when failure
 */
bool bmi26x_dae_if_stop_streaming(sns_sensor_instance *const this, uint8_t sensors);

/*!
 * DAE Start Streaming interface3
 *
 * @param[in] this         Sensor instance handler
 * @return               TRUE on success
 *                       FALSE when failure
 */
bool bmi26x_dae_if_start_streaming(sns_sensor_instance *this);

/*!
 * DAE Process events interface
 *
 * @param[in] this  The sensor instance handler
 */
void bmi26x_dae_if_process_events(sns_sensor_instance *this);

// for instance

/*!
 * DAE interface for check it available
 *
 * @param[in] this     Sensor Instance Handler
 * @return             TRUE   On success
 *                     FALSE when failure
 */
bool bmi26x_dae_if_available(sns_sensor_instance *this);

/*!
 * DAE interface for flush HW
 *
 * @param[in] this   The sensor instance used present
 * @param[in] dae_stream The DAE stream
 * @return       TRUE  on success
 *               FALSE when failure
 */
bool bmi26x_dae_if_flush_hw(sns_sensor_instance *this, bmi26x_dae_stream_t  *dae_stream);

/*!
 *  Check GPIO pin is high or not
 * @param  inst   instance handler
 * @return  True     if gpio pin is high
 *          False    GPIO pin is low
 */
bool bmi26x_dae_check_gpio_is_high(sns_sensor_instance *inst);
