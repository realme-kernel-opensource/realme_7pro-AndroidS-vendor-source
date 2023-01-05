#pragma once
/**
 * @file sns_lsm6dsm_hal.h
 *
 * Hardware Access Layer functions.
 *
 * Copyright (c) 2018, STMicroelectronics.
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

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sensor_uid.h"

#include "sns_lsm6dsm_sensor_instance.h"

// Enable for test code
#ifndef LSM6DSM_ENABLE_TEST_CODE
#define LSM6DSM_ENABLE_TEST_CODE      1
#endif

/**
 *  Address registers
 */
#define STM_LSM6DSM_REG_FUNC_CFG_ACCESS   (0x01)
#define STM_LSM6DSM_REG_FIFO_CTRL1        (0x06)
#define STM_LSM6DSM_REG_FIFO_CTRL2        (0x07)
#define STM_LSM6DSM_REG_FIFO_CTRL3        (0x08)
#define STM_LSM6DSM_REG_FIFO_CTRL4        (0x09)
#define STM_LSM6DSM_REG_FIFO_CTRL5        (0x0A)
#define STM_LSM6DSM_REG_INT1_CTRL         (0x0D)
#define STM_LSM6DSM_REG_INT2_CTRL         (0x0E)
#define STM_LSM6DSM_REG_WHO_AM_I          (0x0F)
#define STM_LSM6DSM_REG_CTRL1_A           (0x10)
#define STM_LSM6DSM_REG_CTRL2_G           (0x11)
#define STM_LSM6DSM_REG_CTRL3             (0x12)
#define STM_LSM6DSM_REG_CTRL4             (0x13)
#define STM_LSM6DSM_REG_CTRL5             (0x14)
#define STM_LSM6DSM_REG_CTRL6_G           (0x15)
#define STM_LSM6DSM_REG_CTRL7_G           (0x16)
#define STM_LSM6DSM_REG_CTRL8_XL          (0x17)
#define STM_LSM6DSM_REG_CTRL9_A           (0x18)
#define STM_LSM6DSM_REG_CTRL10            (0x19)
#define STM_LSM6DSM_REG_WAKE_SRC          (0x1B)
#define STM_LSM6DSM_REG_TAP_SRC           (0x1C)
#define STM_LSM6DSM_REG_D6D_SRC           (0x1D)
#define STM_LSM6DSM_REG_STATUS            (0x1E)

#define STM_LSM6DSM_REG_OUT_TEMP_L        (0x20)
#define STM_LSM6DSM_REG_OUT_TEMP_H        (0x21)
#define STM_LSM6DSM_REG_OUT_X_L_G         (0x22)
#define STM_LSM6DSM_REG_OUT_X_H_G         (0x23)
#define STM_LSM6DSM_REG_OUT_Y_L_G         (0x24)
#define STM_LSM6DSM_REG_OUT_Y_H_G         (0x25)
#define STM_LSM6DSM_REG_OUT_Z_L_G         (0x26)
#define STM_LSM6DSM_REG_OUT_Z_H_G         (0x27)
#define STM_LSM6DSM_REG_OUT_X_L_XL        (0x28)
#define STM_LSM6DSM_REG_OUT_X_H_XL        (0x29)
#define STM_LSM6DSM_REG_OUT_Y_L_XL        (0x2A)
#define STM_LSM6DSM_REG_OUT_Y_H_XL        (0x2B)
#define STM_LSM6DSM_REG_OUT_Z_L_XL        (0x2C)
#define STM_LSM6DSM_REG_OUT_Z_H_XL        (0x2D)
#define STM_LSM6DSM_REG_SENSOR_HUB10      (0x37)
#define STM_LSM6DSM_REG_FIFO_STATUS1      (0x3A)
#define STM_LSM6DSM_REG_FIFO_STATUS2      (0x3B)
#define STM_LSM6DSM_REG_FIFO_STATUS3      (0x3C)
#define STM_LSM6DSM_REG_FIFO_STATUS4      (0x3D)
#define STM_LSM6DSM_REG_FIFO_DATA_OUT_L   (0x3E)
#define STM_LSM6DSM_REG_FIFO_DATA_OUT_H   (0x3F)
#define STM_LSM6DSM_REG_TIMESTAMP0_REG    (0x40)
#define STM_LSM6DSM_REG_TIMESTAMP1_REG    (0x41)
#define STM_LSM6DSM_REG_TIMESTAMP2_REG    (0x42)
#define STM_LSM6DSM_REG_STEP_COUNTER_L    (0x4B)
#define STM_LSM6DSM_REG_STEP_COUNTER_H    (0x4C)
#define STM_LSM6DSM_REG_FUNC_SRC          (0x53)
#define STM_LSM6DSM_REG_TAP_CFG           (0x58)
#define STM_LSM6DSM_REG_TAP_THS_6D        (0x59)
#define STM_LSM6DSM_REG_TAP_DUR           (0x5A)
#define STM_LSM6DSM_REG_WAKE_THS          (0x5B)
#define STM_LSM6DSM_REG_WAKE_DUR          (0x5C)
#define STM_LSM6DSM_REG_FREE_FALL         (0x5D)
#define STM_LSM6DSM_REG_MD1_CFG           (0x5E)
#define STM_LSM6DSM_REG_MD2_CFG           (0x5F)

#define STM_LSM6DSM_FIFO_WTM_STATUS_MASK   (0x80)
#define STM_LSM6DSM_FIFO_OVR_STATUS_MASK   (0x40)
#define STM_LSM6DSM_FIFO_FULL_STATUS_MASK  (0x20)
#define STM_LSM6DSM_FIFO_EMPTY_STATUS_MASK (0x10)
#define STM_LSM6DSM_FIFO_PATTERN_STATUS3   (0xFF)
#define STM_LSM6DSM_FIFO_PATTERN_STATUS4   (0x03)
#define STM_LSM6DSM_FIFO_WTM_CTRL1_MASK    (0xFF)
#define STM_LSM6DSM_FIFO_WTM_CTRL2_MASK    (0x07)
#define STM_LSM6DSM_FIFO_DEC_XL_MASK       (0x07)
#define STM_LSM6DSM_FIFO_DEC_G_MASK        (0x38)
#define STM_LSM6DSM_FIFO_MODE_MASK         (0x07)
#define STM_LSM6DSM_FIFO_ODR_MASK          (0x78)
#define STM_LSM6DSM_INT1_FTH_MASK          (0x08)
#define STM_LSM6DSM_INT1_OVR_MASK          (0x10)
#define STM_LSM6DSM_FIFO_INT_MASK          (0x38)
#define STM_LSM6DSM_FS_XL_MASK             (0x0C)
#define STM_LSM6DSM_ODR_XL_MASK            (0xF0)
#define STM_LSM6DSM_ODR_BW_XL_MASK         (0xF3)
#define STM_LSM6DSM_FS_125_MASK            (0x02)
#define STM_LSM6DSM_FS_G_MASK              (0x0C)
#define STM_LSM6DSM_ODR_G_MASK             (0xF0)

#define STM_LSM6DSM_EMBEDDED_PEDO_THS_MIN  (0x0F)
#define STM_LSM6DSM_EMBEDDED_PEDO_DEB_REG  (0x14)

#define STM_LSM6DSM_PEDO_STEP_MASK         (0x07)

#define STM_LSM6DSM_SLAVE_ENABLE_REG       (0x00)
#define STM_LSM6DSM_OIS_PU_DIS_REG         (0x05)

#define STM_LSM6DSM_SAMPLE_SIZE		         6
#define STM_LSM6DSM_FIFO_SAMPLE_SIZE	     6

/** Default values loaded in probe function */
#define LSM6DSM_FIFO_STREAM_MODE           0x06 // fifo stream continuous mode

// TODO: Set correct size
#define LSM6DSM_HW_MAX_FIFO         511  // 0x1FF, limited by the 9-bit wide value
#define LSM6DSM_MAX_FIFO            208  // smaller fifo fits better in DAE buffer

/** fifo paramters */
//#define LSM6DSM_MAX_FIFO            682 // Max fifo samples 8K bytes to samples

//QC Temp Fix till batching is fixed.
//#define LSM6DSM_MAX_FIFO            208


/** Off to idle time */
#define LSM6DSM_OFF_TO_IDLE_MS      100  //ms

/** Gyro turn on/off time */
#define LSM6DSM_GYRO_ON_TIME_MS      70  //ms

/** Motion detect configuration */
#define LSM6DSM_MD_THRESH          (0.6132f)             // m/s2
#define LSM6DSM_MD_DUR             (0.0)                 // sec
// resolution of MD Threshold Register
#define LSM6DSM_MD_THRESH_MAX      (lsm6dsm_accel_range_max[state->accel_info.range_idx] * G)
// using 2^6
#define LSM6DSM_MD_COARSE_RES      (64)
// using 2^6. This change is to keep code aligned to other drivers where 2^8 is available
#define LSM6DSM_MD_FINE_RES        (64)

#define LSM6DSM_NUM_AXES           3

#if !LSM6DSM_LOGGING_DISABLED
typedef struct log_sensor_state_raw_info
{
  /* Pointer to diag service */
  sns_diag_service *diag;
  /* Pointer to sensor instance */
  sns_sensor_instance *instance;
  /* Pointer to sensor UID*/
  struct sns_sensor_uid *sensor_uid;
  /* Size of a single encoded sample */
  size_t encoded_sample_size;
  /* Pointer to log*/
  void *log;
  /* Size of allocated space for log*/
  uint32_t log_size;
  /* Number of actual bytes written*/
  uint32_t bytes_written;
  /* Number of batch samples written*/
  /* A batch may be composed of several logs*/
  uint32_t batch_sample_cnt;
  /* Number of log samples written*/
  uint32_t log_sample_cnt;
} log_sensor_state_raw_info;

// Unencoded batch sample
typedef struct
{
  /* Batch Sample type as defined in sns_diag.pb.h */
  sns_diag_batch_sample_type sample_type;
  /* Timestamp of the sensor state data sample */
  sns_time timestamp;
  /*Raw sensor state data sample*/
  float sample[LSM6DSM_NUM_AXES];
  /* Data status.*/
  sns_std_sensor_sample_status status;
} lsm6dsm_batch_sample;

// Unencoded batch sample
typedef struct
{
  /* Batch Sample type as defined in sns_diag.pb.h */
  sns_diag_batch_sample_type sample_type;
  /* Timestamp of the sensor state data sample */
  sns_time timestamp;
  /*Raw sensor state data sample*/
  float sample[1];
  /* Data status.*/
  sns_std_sensor_sample_status status;
} lsm6dsm_batch_sample_oneaxis;
#endif

/******************* Function Declarations ***********************************/

/**
 * Resets the Sensor HW. Also calls
 * lsm6dsm_device_set_default_state()
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] sensor        bit mask for sensors to reset
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dsm_reset_device(
    sns_sensor_instance *const instance,
    lsm6dsm_sensor_type sensor);

/**
 * Puts FIFO in bypass mode.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_set_fifo_bypass_mode(sns_sensor_instance *this);

/**
 * Puts FIFO in stream mode.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_set_fifo_stream_mode(sns_sensor_instance *this);

/**
 * Disables FIFO ODR. Also disables ODR for sensors with
 * non-zero ODR.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_stop_fifo_streaming(sns_sensor_instance *const instance);

/**
 * Sets FIFO WM and decimation config registers.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_set_fifo_wmk(sns_sensor_instance *const instance);

/**
 * Enable FIFO streaming. Also enables FIFO sensors with
 * non-zero desired ODR.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_start_fifo_streaming(sns_sensor_instance *const instance);

/**
 * Enables interrupt for FIFO sensors.
 *
 * @param[i] state         Instance state
 * @param[i] sensors       sensor bit mask to enable
 *
 * @return none
 */
void lsm6dsm_enable_fifo_intr(sns_sensor_instance *const instance,
                              lsm6dsm_sensor_type sensor);
/**
 * Disables interrupt for FIFO sensors.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void lsm6dsm_disable_fifo_intr(sns_sensor_instance *const instance);


/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] state         Instance state
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dsm_get_who_am_i(sns_sync_com_port_service *scp_service,
		                    sns_sync_com_port_handle  *port_handle,
                            uint8_t *buffer);

/**
 * Sets Accel ODR, range, BW and sensitivity.
 *
 * @param[i] port_handle     handle to synch COM port
 * @param[i] curr_odr        Accel ODR
 * @param[i] sstvt           Accel sensitivity
 * @param[i] range           Accel range
 * @param[i] bw              Accel BW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dsm_set_accel_config(
   sns_sensor_instance *const instance,
                                lsm6dsm_accel_odr      curr_odr,
                                lsm6dsm_accel_sstvt    sstvt,
                                lsm6dsm_accel_range    range,
                                lsm6dsm_accel_bw       bw);

/**
 * Sets Gyro ODR, range and sensitivity.
 *
 * @param[i] port_handle     handle to synch COM port
 * @param[i] curr_odr        Gyro ODR
 * @param[i] sstvt           Gyro sensitivity
 * @param[i] range           Gyro range
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dsm_set_gyro_config(
    sns_sensor_instance *const instance,
                               lsm6dsm_gyro_odr      curr_odr,
                               lsm6dsm_gyro_sstvt    sstvt,
                               lsm6dsm_gyro_range    range);

/**
 * Populates Instance state with desired FIFO configuration.
 *
 * @param[i] state                 Instance state
 * @param[i] desired_wmk           desired FIFO WM
 * @param[i] a_chosen_sample_rate  desired Accel ODR
 * @param[i] g_chosen_sample_rate  desired GYRO ODR
 * @param[i] sensor                bit mask of Sensors to enable
 *
 * @return none
 */
void lsm6dsm_set_fifo_config(sns_sensor_instance *const instance,
                             uint16_t desired_wmk,
                             lsm6dsm_accel_odr a_chosen_sample_rate,
                             lsm6dsm_gyro_odr g_chosen_sample_rate,
                             lsm6dsm_sensor_type sensor);

/**
 * Reads status registers in Instance State.
 * This function is for debug only.
 *
 * @param[i] state                 Instance state
 * @param[i] sensor                bit mask of Sensors to enabl
 *
 * @return none
 */
void lsm6dsm_dump_reg(sns_sensor_instance *const instance, lsm6dsm_sensor_type sensor);

/**
 * Sets Motion Detect config.
 *
 * @param[i] state        Instance state
 * @param[i] enable       true to enable Motion Accel else false
 *
 * @return none
 */
void lsm6dsm_set_md_config(sns_sensor_instance *const instance, bool enable);

/**
 * Updates Motion detect interrupt state.
 *
 * @param[i] state               Instance state
 * @param[i] enable              true to enable Motion Accel
 *                               else false
 * @param[i] md_not_armed_event  true if MD not armed event
 *                               must be sent
 *
 * @return none
 */
void lsm6dsm_update_md_intr(sns_sensor_instance *const instance,
                            bool enable);

/**
 * Handles MD interrupt:
 *   1. Sends MD fired event.
 *   2. Starts Motion Accel Stream.
 *
 * @param[i] instance        Instance reference
 * @param[i] irq_timestamp   MD interrupt timestamp
 *
 * @return none
 */
void lsm6dsm_handle_md_fired(sns_sensor_instance *const instance,
                             sns_time irq_timestamp);

/**
 * Handles MD interrupt:
 *   1. Handles MD fired sequence.
 *
 * @param[i] instance        Instance reference
 * @param[i] irq_timestamp   MD interrupt timestamp
 * @param[i] wake_src        WAKE_SRC register contents
 *
 * @return none
 */
void lsm6dsm_handle_md_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp,
                                 uint8_t const *wake_src);

/**
 * Sets Gated Accel config.
 *
 * @param[i] desired_wmk           desired FIFO WM
 * @param[i] a_chosen_sample_rate  desired Accel ODR
 * @param[i] g_chosen_sample_rate  desired GYRO ODR
 * @param[i] sensor                bit mask of Sensors to enable
 *
 * @return none
 */
void lsm6dsm_set_gated_accel_config(lsm6dsm_instance_state *state,
                           uint16_t desired_wmk,
                           lsm6dsm_accel_odr a_chosen_sample_rate,
                           lsm6dsm_sensor_type sensor);

/**
 * Changes all gated accel requests to non-gated accel requests.
 *
 * @param instance   Reference to the instance
 *
 * @return None
 */
void lsm6dsm_convert_accel_gated_req_to_non_gated(
   sns_sensor_instance *const instance);

/**
 * Checks for MD interrupt.
 *
 * @param[i] state               Instance state
 * @param[i] wake_src        WAKE_SRC register contents
 *
 * @return bool to represent if the interrupt has occoured
 */
bool lsm6dsm_check_md_interrupt(lsm6dsm_instance_state const *state,
                                uint8_t const *wake_src);

/**
 * Checks for MD interrupt.
 *
 * @param[i] instance        Instance reference
 * @param[i] state           Instance state
 *
 * @return None
 */
void lsm6dsm_turn_off_md(sns_sensor_instance *const instance,
                         lsm6dsm_instance_state *state);

/**
 * Encode Sensor State Log.Interrupt
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lsm6dsm_encode_sensor_state_log_interrupt(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written);

/**
 * Gets current Accel ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 */
float lsm6dsm_get_accel_odr(lsm6dsm_accel_odr curr_odr);

/**
 * Provides sample interval based on current ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time lsm6dsm_get_sample_interval(lsm6dsm_accel_odr curr_odr);

/**
 * send fifo data , extracts accel and gyro samples from the buffer
 * and generates events for each sample.
 *
 * @param[i] instance               Sensor instance
 * @param[i] buffer                 Buffer containing samples read from HW FIFO
 * @param[i] bytes              Number of bytes in fifo buffer
 * @param[i] gyro_enabled           Whether gyro is enabled
 */
void lsm6dsm_send_fifo_data(
    sns_sensor_instance *instance,
    const uint8_t* buffer, uint32_t bytes,
    bool gyro_enabled);

/**
 * read fifo data by reading the Fifo status register and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param instance                 Sensor Instance
 */
void lsm6dsm_read_fifo_data(sns_sensor_instance *const instance, sns_time irq_time, bool flush);

/**
 * flush fifo by reading the fifo data and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param instance                 Sensor Instance
 * @param scp_read                 flush using sync/async read
 */

void lsm6dsm_flush_fifo(sns_sensor_instance *const instance);
/**
 * Sends config update event for the chosen sample_rate
 *
 * @param[i] instance    reference to this Instance
 * @param[i] new_client  if true, send config event even if config has not changed
 */
void lsm6dsm_send_config_event(sns_sensor_instance *const instance, bool new_client);

/**
 * Sends sensor temperature event.
 *
 * @param[i] instance   Sensor Instance
 */
void lsm6dsm_convert_and_send_temp_sample(
  sns_sensor_instance *const instance,
  sns_time            timestamp,
  const uint8_t       temp_data[2]);

/**
 * Sends sensor temperature event.
 *
 * @param[i] instance   Sensor Instance
 */
void lsm6dsm_handle_sensor_temp_sample(sns_sensor_instance *const instance);

/**
 * Starts/restarts polling timer
 *
 * @param instance   Instance reference
 */
void lsm6dsm_start_sensor_temp_polling_timer(sns_sensor_instance *this);

/**
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
void lsm6dsm_set_polling_config(sns_sensor_instance *const this);

/**
 * Starts/restarts polling timer
 *
 * @param instance   Instance reference
 */
void lsm6dsm_reconfig_hw(sns_sensor_instance *this);

/**
 * Configures sensor with new/recomputed fifo settings
 *
 * @param instance   Instance reference
 */

void lsm6dsm_reconfig_fifo(sns_sensor_instance *this, bool flush);

/**
 * If mask = 0x0 or 0xFF, or if size > 1, write reg_value
 * directly to reg_addr. Else, read value at reg_addr and only
 * modify bits defined by mask.
 *
 * @param[i] port_handle      handle to synch COM port
 * @param[i] reg_addr         reg addr to modify
 * @param[i] reg_value        value to write to register
 * @param[i] size             number of bytes to write
 * @param[o]  xfer_bytes      number of bytes transfered
 * @param[i] save_write_time  save write time input
 * @param[i] mask             bit mask to update
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc lsm6dsm_read_modify_write(
    sns_sensor_instance *const instance,
    uint32_t reg_addr,
    uint8_t *reg_value,
    uint32_t size,
    uint32_t *xfer_bytes,
    bool save_write_time,
    uint8_t mask);


/**
 * Extract a accel sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] instance           The current lsm6dsm sensor instance
 * @param[i] sensors[]          Array of sensors for which data is requested
 * @param[i] num_sensors        Number of sensor for which data is requested
 * @param[i] raw_data           Uncalibrated sensor data to be logged
 */
void lsm6dsm_get_data(sns_sensor_instance *const instance,
                                lsm6dsm_sensor_type sensors[],
                                uint8_t num_sensors,
                                int16_t *raw_data);

void lsm6dsm_process_com_port_vector(sns_port_vector *vector, void *user_arg);

void lsm6dsm_send_cal_event(sns_sensor_instance * const instance, lsm6dsm_sensor_type sensor_type);

void lsm6dsm_set_md_intr(sns_sensor_instance *const instance, bool enable);

void lsm6dsm_update_md_filter(sns_sensor_instance * const instance);

void lsm6dsm_enable_md(sns_sensor_instance *const instance, bool send_md_status_event);
void lsm6dsm_disable_md(sns_sensor_instance *const instance, bool send_md_status_event);
void lsm6dsm_register_interrupt(sns_sensor_instance *this,
    lsm6dsm_irq_info* irq_info,
    sns_data_stream* data_stream);

sns_rc lsm6dsm_recover_device(sns_sensor_instance *const this);

void lsm6dsm_inst_create_timer(sns_sensor_instance *this,
    sns_data_stream** timer_data_stream,
    sns_timer_sensor_config* req_payload);

sns_rc lsm6dsm_get_fifo_status(sns_sensor_instance *const instance,
                               uint16_t* bytes_in_fifo,
                               uint8_t* status_reg);

sns_rc lsm6dsm_set_interrupts(sns_sensor_instance *const instance);
bool lsm6dsm_is_md_int_required(sns_sensor_instance *const this);

sns_rc lsm6dsm_com_write_wrapper(
    sns_sensor_instance *const instance,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes,
    bool save_write_time);

sns_rc lsm6dsm_com_read_wrapper(
  sns_sync_com_port_service* scp_service,
  sns_sync_com_port_handle*  port_handle,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes);

uint8_t lsm6dsm_read_wake_src(lsm6dsm_instance_state const *state);
void lsm6dsm_read_src_regs(lsm6dsm_instance_state const *state, uint8_t src_regs[2]);

sns_rc lsm6dsm_read_regs_scp(sns_sensor_instance *const instance,
                             uint8_t addr, uint16_t num_of_bytes, uint8_t *buffer);

#if !LSM6DSM_LOGGING_DISABLED
//logging functions
void lsm6dsm_log_sensor_state_raw_alloc(
  log_sensor_state_raw_info *log_raw_info,
  uint32_t log_size);

sns_rc lsm6dsm_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  uint8_t  sample_size,
  sns_time timestamp,
  sns_std_sensor_sample_status status);

void lsm6dsm_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  uint8_t sample_size,
  bool batch_complete);
#endif
void lsm6dsm_update_heartbeat_monitor(sns_sensor_instance *const instance);

