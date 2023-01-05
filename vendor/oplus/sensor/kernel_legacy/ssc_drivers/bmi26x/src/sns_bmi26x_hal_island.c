/******************************************************************************
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

/**
 * @file sns_bmi26x_hal_island.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/


#include "sns_com_port_types.h"
#include "sns_event_service.h"
#include "sns_gpio_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include "sns_types.h"

#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor.h"
#include "sns_bmi26x_sensor_instance.h"

#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#endif

#include "sns_timer.pb.h"
#include "sns_std_event_gated_sensor.pb.h"

#include "sns_cal.pb.h"
#include "sns_cal_util.h"

// sensor configuration
#include "sensor_cfg/sns_bmi26x_sensor_cfg.h"

#if BMI26X_CONFIG_ENABLE_SEE_LITE
fp_read_gpio bmi26x_fp_read_gpio;
#endif
fp_sns_scp_register_rw  bmi26x_fp_scp_rw;


void bmi26x_log_sensor_state_raw_submit(
    log_sensor_state_raw_info_t *log_raw_info,
    bool batch_complete);


//NOTE: edit this list with caution, the sequence/order is relied upon
//the list is arranged in the way that the regv member has the same value as the index itself
//OPTIM3
struct bmi26x_odr_regv_map BMI26X_REGV_ODR_MAP[] = {
    {
        .odr  = 0,
        .regv = 0
    },

    {
        .odr  = 0.78f,
        .regv = BMI26X_REGV_ODR_0_78HZ
    },

    {
        .odr  = 1.56f,
        .regv = BMI26X_REGV_ODR_1_56HZ
    },

    {
        .odr  = 3.125f,
        .regv = BMI26X_REGV_ODR_3_12HZ
    },

    {
        .odr  = 6.25f,
        .regv = BMI26X_REGV_ODR_6_25HZ
    },

    {
        .odr  = 12.5f,
        .regv = BMI26X_REGV_ODR_12_5HZ
    },

    {
        .odr  = 25,
        .regv = BMI26X_REGV_ODR_25HZ
    },

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_50) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_50)
    {
        .odr  = 50,
        .regv = BMI26X_REGV_ODR_50HZ
    },
#endif

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_100) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_100)
    //idx = 7
    {
        .odr  = 100,
        .regv = BMI26X_REGV_ODR_100HZ
    },
#endif

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_200) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_200)
    {
        .odr  = 200,
        .regv = BMI26X_REGV_ODR_200HZ
    },
#endif

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_400) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_400)
    {
        .odr  = 400,
        .regv = BMI26X_REGV_ODR_400HZ
    },
#endif

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_800) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_800)
    {
        .odr  = 800,
        .regv = BMI26X_REGV_ODR_800HZ
    },
#endif

#if (BMI26X_CONFIG_ACC_FASTEST_ODR >= BMI26X_ODR_1600) || (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_1600)
    {
        .odr  = 1600,
        .regv = BMI26X_REGV_ODR_1600HZ
    },
#endif
};


const range_attr_t bmi26x_accel_ranges[] = {
    {BMI26X_ACCEL_RANGE_2G_MIN, BMI26X_ACCEL_RANGE_2G_MAX},
    {BMI26X_ACCEL_RANGE_4G_MIN, BMI26X_ACCEL_RANGE_4G_MAX},
    {BMI26X_ACCEL_RANGE_8G_MIN, BMI26X_ACCEL_RANGE_8G_MAX},
    {BMI26X_ACCEL_RANGE_16G_MIN, BMI26X_ACCEL_RANGE_16G_MAX}
};

const float bmi26x_accel_resolutions[] = {
    BMI26X_ACCEL_RESOLUTION_2G,
    BMI26X_ACCEL_RESOLUTION_4G,
    BMI26X_ACCEL_RESOLUTION_8G,
    BMI26X_ACCEL_RESOLUTION_16G
};

const range_attr_t bmi26x_gyro_ranges[] = {
    {BMI26X_GYRO_RANGE_125_MIN, BMI26X_GYRO_RANGE_125_MAX},
    {BMI26X_GYRO_RANGE_250_MIN, BMI26X_GYRO_RANGE_250_MAX},
    {BMI26X_GYRO_RANGE_500_MIN, BMI26X_GYRO_RANGE_500_MAX},
    {BMI26X_GYRO_RANGE_1000_MIN, BMI26X_GYRO_RANGE_1000_MAX},
    {BMI26X_GYRO_RANGE_2000_MIN, BMI26X_GYRO_RANGE_2000_MAX}
};

const float bmi26x_gyro_resolutions[] = {
    BMI26X_GYRO_RESOLUTION_125DPS,
    BMI26X_GYRO_RESOLUTION_250DPS,
    BMI26X_GYRO_RESOLUTION_500DPS,
    BMI26X_GYRO_RESOLUTION_1000DPS,
    BMI26X_GYRO_RESOLUTION_2000DPS
};




#if BMI26X_CONFIG_ENABLE_DEBUG_TEST || BMI26X_CONFIG_ENABLE_LOG_TS_BATCH
uint8_t    g_bmi26x_cnt_session_acc;
uint8_t    g_bmi26x_cnt_session_gyr;
#endif



void bmi26x_hal_inst_exit_island(sns_sensor_instance *this)
{
#if BMI26X_CONFIG_ENABLE_ISLAND_MODE
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_island_service  *island_svc  =
                    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
    island_svc->api->sensor_instance_island_exit(island_svc, this);

#else
    UNUSED_VAR(this);
#endif
}



static
uint32_t bmi26x_util_get_com_div(int32_t a, int32_t b)
{
    uint32_t mcd = 1;
    int32_t  r;
    int32_t  t;
    uint32_t a_4_mode, b_4_mode;

    if (a < 0) {
        a = -a;
    }

    if (b < 0) {
        b = -b;
    }

    if (a < b) {
        t = a;
        a = b;
        b = t;
    }

    if (0 == a) {
        return (uint32_t)b;
    }

    if (0 == b) {
        return (uint32_t)a;
    }

    a_4_mode = a;
    b_4_mode = b;

    while (1) {
        mcd = b_4_mode;

        if (0 == b_4_mode) {
            return a_4_mode;
        }

        r = a_4_mode % b_4_mode;

        if (0 != r) {
            a_4_mode = b_4_mode;
            b_4_mode = r;
        } else {
            break;
        }
    }

    return mcd;
}

static
uint32_t bmi26x_util_get_max_div(uint32_t a, uint32_t cap)
{
    uint32_t i;

    cap = cap < a ? cap : a;

    if (cap == a) {
        cap --;
    }

    for (i = cap; i > 1; i --) {
        if (i > 0) {
            if (0 == (a % i)) {
                return i;
            }
        } else {
            break;
        }
    }

    return 1;
}


#if BMI26X_CONFIG_ENABLE_DIAG_LOG
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
 * SNS_RC_SUCCESS if encoding was successful
 * SNS_RC_FAILED otherwise
 */
sns_rc bmi26x_encode_sensor_state_log_interrupt(
    void      *diag_log,
    size_t    log_size,
    size_t    encoded_log_size,
    void      *encoded_log,
    size_t    *bytes_written)
{
    UNUSED_VAR(log_size);
    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL == encoded_log || NULL == diag_log || NULL == bytes_written) {
        return SNS_RC_FAILED;
    }

    sns_diag_sensor_state_interrupt *sensor_state_interrupt =
        (sns_diag_sensor_state_interrupt *) diag_log;
    pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);

    if (!pb_encode(&stream, sns_diag_sensor_state_interrupt_fields,
                   sensor_state_interrupt)) {
        rc = SNS_RC_FAILED;
    }

    if (SNS_RC_SUCCESS == rc) {
        *bytes_written = stream.bytes_written;
    }


    return rc;
}

/**
 * Encode log sensor state raw packet
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
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was successful
 * SNS_RC_FAILED otherwise
 */
sns_rc bmi26x_encode_log_sensor_state_raw(
    void *diag_log, size_t log_size,
    size_t encoded_log_size, void *encoded_log,
    size_t *bytes_written)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t i = 0;
    size_t encoded_sample_size = 0;
    size_t parsed_log_size = 0;
    sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
    uint8_t arr_index = 0;
    float temp[BMI26X_NUM_AXES];
    pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = BMI26X_NUM_AXES,
                            .arr_index = &arr_index
                           };

    if (NULL == encoded_log || NULL == diag_log || NULL == bytes_written) {
        return SNS_RC_FAILED;
    }

    batch_sample.sample.funcs.encode = pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;

    if (!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
                             &batch_sample)) {
        return SNS_RC_FAILED;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
    bmi26x_batch_sample_t *raw_sample = (bmi26x_batch_sample_t *)diag_log;

    while (parsed_log_size < log_size &&
            (stream.bytes_written + encoded_sample_size) <= encoded_log_size &&
            i < (uint32_t)(-1)) {
        arr_index = 0;
        arg.arr = (float *)raw_sample[i].sample;

        batch_sample.sample_type = raw_sample[i].sample_type;
        batch_sample.status = raw_sample[i].status;
        batch_sample.timestamp = raw_sample[i].timestamp;

        if (!pb_encode_tag(&stream, PB_WT_STRING,
                           sns_diag_sensor_state_raw_sample_tag)) {
            rc = SNS_RC_FAILED;
            break;
        } else if (!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                                        &batch_sample)) {
            rc = SNS_RC_FAILED;
            break;
        }

        parsed_log_size += sizeof(bmi26x_batch_sample_t);
        i++;
    }

    if (SNS_RC_SUCCESS == rc) {
        *bytes_written = stream.bytes_written;
    }

    return rc;
}

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] log_size   Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void bmi26x_log_sensor_state_raw_alloc(
    log_sensor_state_raw_info_t *log_raw_info,
    uint32_t log_size)
{
    uint32_t max_log_size = 0;

    if (NULL == log_raw_info || NULL == log_raw_info->diag ||
            NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid) {
        return;
    }

    // allocate memory for sensor state - raw sensor log packet
    max_log_size = log_raw_info->diag->api->get_max_log_size(
                       log_raw_info->diag);

    if (0 == log_size) {
        // log size not specified.
        // Use max supported log packet size
        log_raw_info->log_size = max_log_size;
    } else if (log_size <= max_log_size) {
        log_raw_info->log_size = log_size;
    } else {
        return;
    }

    log_raw_info->log = log_raw_info->diag->api->alloc_log(
                            log_raw_info->diag,
                            log_raw_info->instance,
                            log_raw_info->sensor_uid,
                            log_raw_info->log_size,
                            SNS_DIAG_SENSOR_STATE_LOG_RAW);

    log_raw_info->log_sample_cnt = 0;
    log_raw_info->bytes_written = 0;
}

/**
 *
 * Add raw uncalibrated sensor data to Sensor State Raw log
 * packet
 *
 * @param[i] log_raw_info Pointer to logging information
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 *
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was successful
 * SNS_RC_FAILED otherwise
 */
sns_rc bmi26x_log_sensor_state_raw_add(
    log_sensor_state_raw_info_t *log_raw_info,
    float *raw_data,
    sns_time timestamp,
    sns_std_sensor_sample_status status)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL == log_raw_info || NULL == log_raw_info->diag ||
            NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
            NULL == raw_data || NULL == log_raw_info->log) {
        return SNS_RC_FAILED;
    }

    if ( (log_raw_info->bytes_written + sizeof(bmi26x_batch_sample_t)) >
            log_raw_info->log_size) {
        // no more space in log packet
        // submit and allocate a new one
        bmi26x_log_sensor_state_raw_submit(log_raw_info, false);
        bmi26x_log_sensor_state_raw_alloc(log_raw_info, 0);
    }

    if (NULL == log_raw_info->log) {
        rc = SNS_RC_FAILED;
    } else {
        bmi26x_batch_sample_t *sample =
            (bmi26x_batch_sample_t *)log_raw_info->log;

        if (0 == log_raw_info->batch_sample_cnt) {
            sample[log_raw_info->log_sample_cnt].sample_type =
                SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
        } else {
            sample[log_raw_info->log_sample_cnt].sample_type =
                SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
        }

        sample[log_raw_info->log_sample_cnt].timestamp = timestamp;

        sns_memscpy(sample[log_raw_info->log_sample_cnt].sample,
                    sizeof(sample[log_raw_info->log_sample_cnt].sample),
                    raw_data,
                    sizeof(sample[log_raw_info->log_sample_cnt].sample));

        sample[log_raw_info->log_sample_cnt].status = status;

        log_raw_info->bytes_written += sizeof(bmi26x_batch_sample_t);

        log_raw_info->log_sample_cnt++;
        log_raw_info->batch_sample_cnt++;
    }

    return rc;
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void bmi26x_log_sensor_state_raw_submit(
    log_sensor_state_raw_info_t *log_raw_info,
    bool batch_complete)
{
    bmi26x_batch_sample_t *sample = NULL;

    if (NULL == log_raw_info || NULL == log_raw_info->diag ||
            NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
            NULL == log_raw_info->log) {
        return;
    }

    sample = (bmi26x_batch_sample_t *)log_raw_info->log;

    if (batch_complete) {
        // overwriting previously sample_type for last sample
        if (1 == log_raw_info->batch_sample_cnt) {
            sample[0].sample_type =
                SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
        } else if (1 < log_raw_info->batch_sample_cnt) {
            sample[log_raw_info->log_sample_cnt - 1].sample_type =
                SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
        }
    }

    if (log_raw_info->log != NULL) {
        log_raw_info->diag->api->submit_log(
            log_raw_info->diag,
            log_raw_info->instance,
            log_raw_info->sensor_uid,
            log_raw_info->bytes_written,
            log_raw_info->log,
            SNS_DIAG_SENSOR_STATE_LOG_RAW,
            log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
            bmi26x_encode_log_sensor_state_raw);

        log_raw_info->log = NULL;
    }
}

#endif


#if BMI26X_CONFIG_ENABLE_SEE_LITE
sns_rc bmi26x_read_gpio(
    bmi26x_instance_state   *istate,
    sns_interrupt_req       *gpio_cfg,
    sns_gpio_state          *level)
{
    sns_rc rc;

    //rc = istate->gpio_service->api->read_gpio(gpio_cfg->interrupt_num, gpio_cfg->is_chip_pin, level);
    rc = bmi26x_fp_read_gpio(gpio_cfg->interrupt_num, gpio_cfg->is_chip_pin, level);

#if BMI26X_CONFIG_ENABLE_DEBUG
    if (*level) {
        BMI26X_INST_LOG(MED, istate->owner, "gpio HIGH  rc=%d", rc);
    }
#else
    UNUSED_VAR(istate);
#endif


    return rc;
}
#endif

#if !BMI26X_CONFIG_ENABLE_SEE_LITE
/** See sns_bmi26x_hal.h */


/** See sns_bmi26x_hal.h */
#ifndef SSC_TARGET_HEXAGON_CORE_QDSP6_2_0
void bmi26x_write_gpio(sns_sensor_instance *instance, uint32_t gpio,
                       bool is_chip_pin,
                       sns_gpio_drive_strength drive_strength,
                       sns_gpio_pull_type pull,
                       sns_gpio_state gpio_state)
{
    sns_service_manager *ismgr = instance->cb->get_service_manager(instance);
    sns_gpio_service *gpio_svc = (sns_gpio_service*)ismgr->get_service(ismgr, SNS_GPIO_SERVICE);
    sns_rc rc = SNS_RC_SUCCESS;

    rc = gpio_svc->api->write_gpio(gpio, is_chip_pin, drive_strength, pull, gpio_state);

    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, instance, "WARNING!!! write gpio rc = %d", rc);
    }
}
#endif
#endif

//FIXME need more check on virtual sensors
static bool bmi26x_hal_confirm_low_power(bmi26x_instance_state *istate)
{
    // check current working mode
    bool need_low_power = false;
    if (istate->int_en_flags_curr.bits.md &&
            !(istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO))) {
        need_low_power = true;
    } else if ((istate->fifo_info.publish_sensors & 0xFF) == BMI26X_SENSOR_TEMP) {
        need_low_power = true;
    } else if ((istate->fifo_info.publish_sensors & 0xFF) == BMI26X_SENSOR_NONE) {
        need_low_power = true;
    }

    return need_low_power;
}

bool bmi26x_sbus_is_in_normal_mode(
    bmi26x_instance_state   *istate,
    uint8_t regv_acc_cfg,
    uint8_t regv_gyr_cfg,
    uint8_t regv_pwr_cfg,   // regv@0x7c
    uint8_t regv_pwr_ctrl)  // regv@0x7d
{
    bool ret = false;
    uint8_t imu_perf = 0;

    UNUSED_VAR(istate);

    if (BST_GET_VAL_BIT(regv_pwr_cfg, 0)) {   // adv_power_save == 1
        ret = (BMI26X_REGV_PMU_STAT_ACC_NORMAL == BST_GET_VAL_BIT(regv_pwr_ctrl, 2));
        ret = ret || (BMI26X_REGV_PMU_STAT_GYR_NORMAL == BST_GET_VAL_BIT(regv_pwr_ctrl, 1));
        ret = ret || (BMI26X_REGV_PMU_STAT_MAG_NORMAL == BST_GET_VAL_BIT(regv_pwr_ctrl, 0));

        imu_perf = BST_GET_VAL_BIT(regv_acc_cfg, 7);
        imu_perf |= (BST_GET_VAL_BIT(regv_gyr_cfg, 7) << 1);

        if (ret && imu_perf) {
            ret = true;
        } else {
            ret = false;
        }

    } else {
        ret = true;
    }

    return ret;
}

static bool bmi26x_dev_is_reg_rw_vector_input_valid(uint8_t rega, uint8_t bit_start, uint8_t bit_end)
{
    if ((rega >= BMI26X_REGA_EXT_MODE) ||
            (bit_start > 7) ||
            (bit_end > 7))  {
        return false;
    }
    return true;
}

/**
 * Read wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
static
sns_rc bmi26x_com_read_wrapper(
    sns_sync_com_port_service *scp_service,
    bmi26x_com_port_info_t      *port_info,
    uint32_t reg_addr,
    uint8_t *buffer,
    uint32_t bytes,
    uint32_t *xfer_bytes)
{
    sns_port_vector port_vec;
    sns_sync_com_port_handle    *port_handle = port_info->port_handle;


    if (reg_addr > BMI26X_REGA_EXT_MODE) {
        return SNS_RC_INVALID_VALUE;
    }

    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;

    return scp_service->api->sns_scp_register_rw(port_handle, &port_vec, 1,
                                                 false, xfer_bytes);
}

//NU:TODO2
sns_rc bmi26x_com_read_byte(
    sns_sync_com_port_service *scp_service,
    bmi26x_com_port_info_t    *port_info,
    uint32_t                reg_addr,
    uint8_t                 *buffer)
{
    sns_rc                      rc;
    uint8_t                     si_buf[1 + BMI26X_SPEC_SPI_DUMMY_READ_LEN];
    uint32_t                    xfer_bytes;


    if (reg_addr > BMI26X_REGA_EXT_MODE) {
        return SNS_RC_INVALID_VALUE;
    }

    rc = bmi26x_com_read_wrapper(scp_service, port_info, reg_addr, si_buf, (1 + BMI26X_SPEC_SPI_DUMMY_READ_LEN),
                                 &xfer_bytes);

    if (SNS_BUS_SPI == port_info->com_config.bus_type) {
        buffer[0] = si_buf[BMI26X_SPEC_SPI_DUMMY_READ_LEN];
    } else {
        buffer[0] = si_buf[0];
    }

    return rc;
}

sns_rc bmi26x_com_write_byte(
    sns_sync_com_port_service *scp_service,
    bmi26x_com_port_info_t    *port_info,
    uint32_t                  reg_addr,
    uint8_t                   *buffer)
{
    uint32_t                    xfer_bytes;
    sns_port_vector port_vec;

    if (reg_addr > BMI26X_REGA_EXT_MODE) {
        return SNS_RC_INVALID_VALUE;
    }

    port_vec.buffer = buffer;
    port_vec.bytes = 1;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;

    return scp_service->api->sns_scp_register_rw(port_info->port_handle,
                                                 &port_vec,
                                                 1,
                                                 false,
                                                 &xfer_bytes);
}



bmi26x_instance_state *bmi26x_inst_singleton = NULL;


sns_rc bmi26x_sbus_read_wrapper(
    void                *sbus_obj,
    uint32_t            reg_addr,
    uint8_t             *buf,
    uint32_t            len)
{
    bmi26x_instance_state   *istate = (bmi26x_instance_state *)sbus_obj;
    sns_port_vector         port_vec;
    uint32_t                xfer_bytes = 0;

    uint8_t                 si_buf[BMI26X_CONFIG_SPI_BURST_READ_LEN_MAX + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY] = "";

    sns_rc                  rc;
    uint32_t                delay_us = 0;
    sns_time                ts_curr = bmi26x_get_sys_tick();


    if ((len < 1) || (reg_addr > BMI26X_REGA_EXT_MODE)) {
        return SNS_RC_INVALID_VALUE;
    }

    //<wait according to spec>
    if (!istate->sbus_in_normal_mode) {
        delay_us = bmi26x_validatation_delay_time(ts_curr,
                                                  istate->ts_last_sbus_write,
                                                  BMI26X_SPEC_IF_IDLE_TIME_SUSPEND_US);
    } else {
        delay_us = bmi26x_validatation_delay_time(ts_curr,
                                                  istate->ts_last_sbus_write,
                                                  BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);
    }

    if (delay_us > 0) {
        bmi26x_delay_us(delay_us);
#if BMI26X_CONFIG_ENABLE_WAIT4CMD_IMPROVED
            rc = bmi26x_hal_update_pmu_stat(istate, false);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
#endif
    }
    //</wait according to spec>

    if (istate->bus_is_spi) {
        if (len <= BMI26X_CONFIG_SPI_BURST_READ_LEN_MAX) {
            port_vec.buffer = si_buf;
            port_vec.bytes = len + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY;
        } else {
            BMI26X_INST_LOG(HIGH, istate->owner, "sbus_read @0x%02x len exceeds max", reg_addr);
            return SNS_RC_INVALID_VALUE;
        }
    } else {
        port_vec.buffer = buf;
        port_vec.bytes = len;
    }


    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;

    //rc = istate->scp_service->api->sns_scp_register_rw(
    istate->ts_last_sbus_read_pre = bmi26x_get_sys_tick();

    rc = bmi26x_fp_scp_rw(istate->com_port_info.port_handle,
                          &port_vec, 1, false, &xfer_bytes);

    UNUSED_VAR(xfer_bytes);

    if (istate->bus_is_spi) {
        sns_memscpy(buf, len, (si_buf + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY), len);
    }

    if ((!istate->sbus_mon_single_byte_rw_off) || (SNS_RC_SUCCESS != rc)) {
        if (delay_us > 0) {
            BMI26X_INST_LOG(HIGH, istate->owner, "sbrd:%d", delay_us);
        }

        if (1 == len) {
            BMI26X_INST_LOG(MED, istate->owner, "<sbus_read> <@0x%02x:0x%02x rc:%d>",
                            reg_addr, buf[0], rc);
        }
    }

    if (SNS_RC_SUCCESS == rc) {
    } else {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! sbus_read err: %d @0x%02x", rc, reg_addr);
    }

    return rc;
}

sns_rc bmi26x_sbus_write_wrapper(
    void        *sbus_obj,
    uint32_t    reg_addr,
    uint8_t     *buf,
    uint32_t    len)
{
    bmi26x_instance_state *istate;
    sns_rc          rc;
    uint32_t        delay_us = 0;

    bool            cmd_valid = 0;

    sns_port_vector port_vec;
    uint32_t        xfer_bytes = 0;
    union bmi26x_hw_err_stat hw_err_st;
    sns_time ts_curr = bmi26x_get_sys_tick();

    istate = (bmi26x_instance_state *)sbus_obj;


    if (len >= 1) {
    } else {
        return SNS_RC_INVALID_VALUE;
    }


    if (reg_addr > BMI26X_REGA_EXT_MODE) {
        return SNS_RC_INVALID_VALUE;
    }

    //<wait according to spec>
    if (!istate->sbus_in_normal_mode) {

        delay_us = bmi26x_validatation_delay_time(ts_curr,
                                                  istate->ts_last_sbus_write,
                                                  BMI26X_SPEC_IF_IDLE_TIME_SUSPEND_US);
    } else {

        delay_us = bmi26x_validatation_delay_time(ts_curr,
                                                   istate->ts_last_sbus_write,
                                                   BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);
    }

    // delay
    if (delay_us > 0) {
        bmi26x_delay_us(delay_us);
        if (!istate->sbus_in_normal_mode) {
#if BMI26X_CONFIG_ENABLE_WAIT4CMD_IMPROVED
            rc = bmi26x_hal_update_pmu_stat(istate, false);
#endif
        }
    }
    //</wait according to spec>

    if ((1 == len) && (BMI26X_REGA_USR_CMD == reg_addr)) {
        cmd_valid = 1;
        switch (buf[0]) {
            case BMI26X_REGV_CMD_SOFT_RESET:
                //istate->sbus_in_normal_mode = false;
                //istate->accel_info.in_lpm = 1;
                bmi26x_hal_set_sensor_state_2_default_state(istate->owner, (uint8_t)BMI26X_ACCEL);
                cmd_valid = 0;  //cannot read out valid data value
                break;
            case BMI26X_REGV_CMD_INT_RESET: //TODO
                break;
            case BMI26X_REGV_CMD_FIFO_FLUSH:
                break;
            default:
                cmd_valid = 0;
                break;
        }
    }

    port_vec.buffer = buf;
    port_vec.bytes = len;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;

    //rc = istate->scp_service->api->sns_scp_register_rw(
    rc = bmi26x_fp_scp_rw(istate->com_port_info.port_handle,
                          &port_vec, 1, true, &xfer_bytes);

    istate->ts_last_sbus_write = bmi26x_get_sys_tick();

    if (!istate->sbus_mon_single_byte_rw_off || (SNS_RC_SUCCESS != rc)) {
        if (delay_us > 0) {
            BMI26X_INST_LOG(MED, istate->owner, "sbwd:%d", delay_us);  //does not work for some reason TOCHECK
        }
        if (1 == len) {
            BMI26X_INST_LOG(MED, istate->owner, "<sbus_write> <@0x%02x:0x%02x rc:%d>",
                            reg_addr, buf[0], rc);
        }
    }

    if (cmd_valid) {
        rc = bmi26x_dev_get_reg_hw_err_stat(istate, &hw_err_st);
    } else {
        /*BMI26X_INST_LOG(IGNORE, istate->owner, "ignore hw err check on cmd: 0x%x@0x%x",
                        buf[0], reg_addr);*/
    }

    if (SNS_RC_SUCCESS == rc) {
    } else {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! sbus_write err: %d @0x%02x", rc, reg_addr);
    }

    return rc;
}


sns_rc bmi26x_sbus_write_byte(
    void        *sbus_obj,
    uint32_t    reg_addr,
    uint8_t     regv)
{
    return bmi26x_sbus_write_wrapper(sbus_obj, reg_addr, &regv, 1);
}

sns_rc bmi26x_dev_reg_read_modify_write(
    void        *sbus_obj,
    uint8_t     rega,
    uint8_t     bit_start,
    uint8_t     bit_end,
    uint8_t     bit_block_val)
{
    sns_rc          rc;
    uint8_t         regv = 0;

    if (bmi26x_dev_is_reg_rw_vector_input_valid(rega, bit_start, bit_end) == false) {
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_sbus_read_wrapper(sbus_obj,
                                  rega, &regv, 1);

    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (BST_GET_VAL_BITBLOCK(regv, bit_start, bit_end) != bit_block_val) {
        regv = BST_SET_VAL_BITBLOCK(regv, bit_start, bit_end, bit_block_val);
        rc = bmi26x_sbus_write_wrapper(sbus_obj, (uint32_t)rega, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }


    return SNS_RC_SUCCESS;
}

static sns_rc bmi26x_hal_configure_fifo_downsampling_rate(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
#if BMI26X_CONFIG_ENABLE_LOWG
    if (istate->lowg_info.sstate->lowg_config.debug) {
        uint8_t regv_down_factor = 0x03; //BMI26X_REGV_ODR_1600HZ - istate->accel_info.odr_curr;
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_FIFO_DOWNS, 4, 7, regv_down_factor);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    } else {
        // don't touch it
    }
#else
    //uint8_t regv_down_factor = 0x0; //BMI26X_REGV_ODR_1600HZ - istate->accel_info.odr_curr;
    //rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_FIFO_DOWNS, 7, 7, regv_down_factor);
    UNUSED_VAR(istate);
#endif

    return rc;
}

uint8_t bmi26x_hal_cfg_get_page_num(uint8_t param_id, uint8_t *page_offset, uint8_t feature_scope)
{
    int loop = 0;
    uint8_t page_num = 0xFF;
    struct bmi26x_feature_config *p_cfg = NULL;
    uint16_t sensor_cfg_feature_in_size =  bmi26x_sensor_feature_configure_in_size();
    struct bmi26x_feature_config *p_features;


    if (feature_scope == BMI26X_ADVANCED_FEATURE_INPUT) {
        sensor_cfg_feature_in_size =  bmi26x_sensor_feature_configure_in_size();
        p_features = feat_in;
    } else {
        sensor_cfg_feature_in_size =  bmi26x_sensor_feature_configure_out_size();
        p_features = feat_out;
    }

    while (loop < sensor_cfg_feature_in_size) {
        if (p_features[loop].type == param_id) {
            p_cfg = &p_features[loop];
            break;
        }

        loop ++;
    }
    if (p_cfg) {
        page_num = p_cfg->page;
        *page_offset = p_cfg->start_addr;
    }

    return page_num;
}

static sns_rc bmi26x_hal_switch_page(bmi26x_instance_state *istate, uint8_t page_num)
{
    sns_rc rc = SNS_RC_SUCCESS;
    if (istate->cfg_current_pg_num != page_num) {
        rc = bmi26x_sbus_write_wrapper (istate, BMI26X_REGA_USR_FEA_CONF_PAGE, &page_num, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(BMI26X_PAGE_SWITCH_DELAY_IN_US);
        istate->cfg_current_pg_num = page_num;
    }
    return rc;
}


sns_rc bmi26x_get_cfg_data(bmi26x_instance_state *istate,
                                  uint8_t page_num,
                                  uint8_t pg_offset,
                                  uint8_t *rw_buffer,
                                  uint32_t rw_data_size
                                 )
{
    sns_rc rc = SNS_RC_SUCCESS;
    //1. switch page
    rc = bmi26x_hal_switch_page(istate, page_num);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // 2. get configure data
    rc = bmi26x_sbus_read_wrapper (istate, BMI26X_REGA_USR_FEA_CONF_REGS + pg_offset,
                                   rw_buffer, rw_data_size);
    return rc;
}

sns_rc bmi26x_set_cfg_data(bmi26x_instance_state *istate,
                                  uint8_t page_num,
                                  uint8_t pg_offset,
                                  uint8_t *rw_buffer,
                                  uint32_t rw_data_size
                                 )
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL == rw_buffer) {
        rc = SNS_RC_INVALID_VALUE;
        return rc;
    }

    //1. switch page
    rc = bmi26x_hal_switch_page(istate, page_num);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // 2. write configure data
    BMI26X_INST_LOG(LOW, istate->owner, "cfg: pg_num:%d, offset:%d, cfg_size:%d",
                    page_num, pg_offset, rw_data_size);
    rc = bmi26x_sbus_write_wrapper (istate, BMI26X_REGA_USR_FEA_CONF_REGS + pg_offset,
                                    rw_buffer, rw_data_size);
    return rc;
}

sns_rc bmi26x_hal_get_cfg_data_wrapper(
                bmi26x_instance_state *istate,
                uint8_t param_id,
                uint8_t feature_scope,
                uint8_t *read_buffer,
                uint32_t rw_data_size)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset;
    uint8_t page_num = 0xff;

    page_num = bmi26x_hal_cfg_get_page_num(param_id, &page_offset, feature_scope);
    if (page_num == 0xFF) {
        return SNS_RC_INVALID_TYPE;
    }

    // switch page
    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, read_buffer, rw_data_size);

    return rc;
}


sns_rc bmi26x_hal_set_cfg_data_wrapper(
                bmi26x_instance_state *istate,
                uint8_t param_id,
                uint8_t feature_scope,
                uint8_t *write_buffer,
                uint32_t rw_data_size)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset;
    uint8_t page_num = 0xff;

    page_num = bmi26x_hal_cfg_get_page_num(param_id, &page_offset, feature_scope);
    if (page_num == 0xFF) {
        return SNS_RC_INVALID_TYPE;
    }

    // switch page
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, write_buffer, rw_data_size);

    return rc;
}


static sns_rc bmi26x_hal_en_ois(bmi26x_instance_state *istate, bool en)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0, regv_present = 0;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_IF_CONF, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    regv_present = regv;

    if (istate->gyro_info.sstate->ois_cfg.spi4) {
        // clear
        regv = BST_CLR_VAL_BIT(regv, 1);
    } else {
        // set
        regv = BST_SET_VAL_BIT(regv, 1);
    }

    // ois.en @0x6B
    if (en) {
        regv = BST_SET_VAL_BIT(regv, 4);
    } else {
        regv = BST_CLR_VAL_BIT(regv, 4);
    }

    if (regv != regv_present) {
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_IF_CONF, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    return rc;
}

static sns_rc bmi26x_hal_pwr_cmd_handler(
    bmi26x_instance_state *istate,
    uint8_t regv_cmd)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_w = 0;
    uint8_t bit_start = 0, bit_end = 0;

    if (regv_cmd == BMI26X_PWR_ACC_NORMAL ||
            regv_cmd == BMI26X_PWR_ACC_LOW_POWER) {
        regv_w = 1;
        bit_start = 2;
        bit_end = 2;
    } else if (regv_cmd == BMI26X_PWR_GYR_LOW_POWER ||
               regv_cmd == BMI26X_PWR_GYR_NORMAL) {
        regv_w = 1;
        bit_start = 1;
        bit_end = 1;
    } else if (regv_cmd == BMI26X_PWR_ACC_SUSPED) {
        regv_w = 0;
        bit_start = 2;
        bit_end = 2;
    } else if (regv_cmd == BMI26X_PWR_GYR_SUSPED) {
        regv_w = 0;
        bit_start = 1;
        bit_end = 1;
    } else if ((BMI26X_PWR_TEMP_SUSPEND == regv_cmd) ||
            (BMI26X_PWR_TEMP_NORMAL == regv_cmd)) {

        if (BMI26X_PWR_TEMP_NORMAL == regv_cmd) {
            regv_w = 1;
        } else {
            regv_w = 0;
        }

        bit_start = 3;
        bit_end = 3;
    } else {
        return SNS_RC_INVALID_TYPE;
    }

    rc |= bmi26x_dev_pwr_ctrl(istate, bit_start, bit_end, regv_w);

    return rc;
}


#if BMI26X_CONFIG_ENABLE_OIS

static sns_rc bmi26x_hal_set_lp(bmi26x_instance_state *istate, bool en_lp)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t lp_buffer[2] = {0};
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint16_t *lp_cfg = (uint16_t *) lp_buffer;
    bool gyr_ois_lp_en = false;
    bool gyr_ois_en = false;
    bool en_gyr = false;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_LP_FILTER,
            &page_offset, BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner,
                "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, lp_buffer,
                             BMI26X_FEATURE_LP_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner,
            "lp cfg:@page:%d, offset:%d, val:0x%04x", page_num, page_offset,
            *lp_cfg);

    gyr_ois_lp_en = ((*lp_cfg) & 0x01);
    gyr_ois_en = ((*lp_cfg) & 0x40);

    en_gyr = ((istate->fifo_info.publish_sensors & BMI26X_GYRO) > 0);

    if ((en_lp == gyr_ois_lp_en) && (en_gyr == gyr_ois_en)) {
        BMI26X_INST_LOG(LOW, istate->owner,
                "no need to update ois.lp.cfg: %d %d/%d %d", gyr_ois_lp_en, gyr_ois_en,
                en_lp, en_gyr);
        return SNS_RC_SUCCESS;
    }

    if (en_lp) {
        *lp_cfg |= 0x01;
    } else {
        *lp_cfg = BST_CLR_VAL_BIT(*lp_cfg, 0);
    }

    if (en_gyr) {
        *lp_cfg |= 0x40;
    } else {
        *lp_cfg = BST_CLR_VAL_BIT(*lp_cfg, 6);
    }

    // sync to configuration
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, lp_buffer,
                            BMI26X_FEATURE_LP_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // debug
    {
        sns_memset(lp_buffer, 0, sizeof(lp_buffer));
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, lp_buffer,
                                 BMI26X_FEATURE_LP_PARAM_SIZE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        BMI26X_INST_LOG(LOW, istate->owner, "lp cfg:// 0x%04x", *lp_cfg);
    }
    // @debug

    return rc;
}


void bmi26x_hal_config_ois(sns_sensor_instance * const this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;
    bool en_ois = false;

    BMI26X_INST_LOG(MED, istate->owner, "cfg ois <%d %d %d>",
                    istate->gyro_info.sstate->ois_cfg.enable,
                    istate->gyro_info.sstate->ois_cfg.range_idx,
                    istate->gyro_info.sstate->ois_cfg.spi4
                   );

    // ois.range @0x43
    // 250dps, by default, should be a configurable in the json configuration file

    if (istate->fifo_info.publish_sensors & (BMI26X_OIS | BMI26X_GYRO)) {
        en_ois = true;
    }

    if (en_ois != istate->ois_info.en) {

        if (istate->gyro_info.sstate->ois_cfg.range_idx == 1) {
            regv = BMI26X_OIS_RANGE_2000;
        } else {
            //default is 250
            regv = BMI26X_OIS_RANGE_250;
        }

        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_GYR_RANGE, 3, 3, regv);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, istate->owner, "WARNING!!! reg r/w error:%d", rc);
            return ;
        }

        rc = bmi26x_hal_en_ois(istate, en_ois);
        if (rc != SNS_RC_SUCCESS) {
            // TODO
        }

        istate->ois_info.en = en_ois;
    }

    // LP filter
#if    BMI26X_CONFIG_ENABLE_OIS_LP_FILTER
    rc = bmi26x_hal_set_lp(istate, true);
#else
    rc = bmi26x_hal_set_lp(istate, false);
#endif
    if (rc != SNS_RC_SUCCESS) {
    }
}

static void bmi26x_precheck_gyr_hw_req(bmi26x_instance_state *istate)
{
    bool gyr_pwr_cfg_pending = false;

    if (!(istate->fifo_info.publish_sensors & BMI26X_GYRO)) {
        // close gyro power after ois
        gyr_pwr_cfg_pending = true;
    }

    istate->gyro_info.pwr_cfg_pending = gyr_pwr_cfg_pending;
}

static void bmi26x_recheck_gyr_hw_req(bmi26x_instance_state *istate)
{
    if (istate->gyro_info.pwr_cfg_pending) {
        uint8_t regv = 0;
        sns_rc rc = SNS_RC_SUCCESS;

        if (istate->hw_mod_needed & BMI26X_GYRO) {
            if (istate->fifo_info.publish_sensors & BMI26X_GYRO) {
                regv = BMI26X_PWR_GYR_NORMAL;
            }
        } else {
            regv = BMI26X_PWR_GYR_SUSPED;
        }

        rc = bmi26x_hal_pwr_cmd_handler(istate, regv);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! gyr.pwr:%d", rc);
        }
        istate->gyro_info.pwr_cfg_pending = 0;
    }
}

#endif


#if BMI26X_CONFIG_ENABLE_PEDO
static sns_rc bmi26x_set_step_count_config(
    bmi26x_instance_state *istate,
    uint8_t      param_id,
    bmi26x_config_4_pedo_t *pedo_cfg_struct)
{
    /* Variable to define loop */
    uint8_t loop = 0;
    /* Variable to set flag */
    /* Initialize feature configuration for step counter 1 */
    struct bmi26x_feature_config *step_config = NULL;
    uint8_t  *pedo_cfg = NULL;
    uint32_t  rw_size = 0;
    sns_rc  rc = SNS_RC_SUCCESS;
    uint16_t sensor_cfg_feature_in_size =  bmi26x_sensor_feature_configure_in_size();
    uint8_t pg_offset = 0;

    /* Search for step counter 1 feature in the feature
    configuration array */
    while (loop < sensor_cfg_feature_in_size) {
        if (feat_in[loop].type == param_id) {
            step_config = &feat_in[loop];
            break;
        }

        loop++;
    }

    if (step_config != NULL) {
        /* Set parameter 1 - 8 */
        if (param_id == BMI26X_CONFIG_INDEX_STEP_CNT) {
            pedo_cfg = (uint8_t *) pedo_cfg_struct->settings;
        }

        if (pedo_cfg == NULL) {
            return SNS_RC_INVALID_VALUE;
        }

        rw_size = step_config->block_size;
        pg_offset = step_config->start_addr;
        /* Set the configuration back to the page */
        rc = bmi26x_set_cfg_data(istate, step_config->page, pg_offset, pedo_cfg, rw_size);
    } else {
        rc = SNS_RC_INVALID_TYPE;
    }

    return rc;
}

static sns_rc bmi26x_set_step_config(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bool en_step_cnt = istate->pedo_info.enable_pedo_int;
    static bool in_fac_mode_ever = false;
    uint8_t page_num = 0xFF;
    uint8_t page_offset = 0;
    uint8_t *p_cfg = NULL;

    BMI26X_INST_LOG(LOW, istate->owner, "pedo hw cfg keep_pedo_in_fac_mode: %d, fac mode ever:%d, pwr:%d",
                    istate->keep_pedo_in_fac_mode,
                    in_fac_mode_ever, istate->sbus_in_normal_mode);
    // read back the min_step configuration block
    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_STEP_CNT,
                                           &page_offset, BMI26X_ADVANCED_FEATURE_INPUT);
    if (page_num <= BMI26X_MAX_CONFIG_PAGE_NUM) {
        if (BST_GET_VAL_BIT(istate->pedo_cfg_page_sync, page_num) == 0) {
            p_cfg = (uint8_t *)(&istate->pedo_cfg.settings[0]);
            rc = bmi26x_get_cfg_data(istate, page_num, 0, p_cfg, 16);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            istate->pedo_cfg_page_sync = BST_SET_VAL_BIT(istate->pedo_cfg_page_sync, page_num);

            BMI26X_INST_LOG(LOW, istate->owner, "page :%d, 0x%x 0x%x", page_num, istate->pedo_cfg.settings[0],
                            istate->pedo_cfg.settings[1]);
        }
    } else {
        BMI26X_INST_LOG(ERROR, istate->owner, "WARNING! invalid configure page number:%d", page_num);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    if (istate->keep_pedo_in_fac_mode || in_fac_mode_ever) {
        in_fac_mode_ever = true;
        istate->pedo_cfg.bmi26x_step_counter_config.step_buffer_size = 1;
        BMI26X_INST_LOG(LOW, istate->owner, "pedo hw cfg fac mode, min buffer:%d",
                        istate->pedo_cfg.bmi26x_step_counter_config.step_buffer_size);
    } else {
        istate->pedo_cfg.bmi26x_step_counter_config.step_buffer_size = 7;
        BMI26X_INST_LOG(LOW, istate->owner, "pedo hw cfg normal mode, min buffer:%d",
                        istate->pedo_cfg.bmi26x_step_counter_config.step_buffer_size);
    }

    if (en_step_cnt) {
        istate->pedo_cfg.bmi26x_step_counter_config.en_step_counter = 1;
#if !BMI26X_CONFIG_ENABLE_PEDO_TIMER
        istate->pedo_cfg.bmi26x_step_counter_config.pedo_wml = 2;
#endif
    } else {
        istate->pedo_cfg.bmi26x_step_counter_config.en_step_counter = 0;
        istate->pedo_cfg.bmi26x_step_counter_config.pedo_wml = 0;
    }

#if BMI26X_CONFIG_ENABLE_STEP_DETECTOR
    // configure INT @here
    if (istate->pedo_cfg.bmi26x_step_counter_config.en_step_detector) {
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH, 0, 0, BMI26X_CONFIG_INT_LATCH_REGV);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);


        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT1_MAP, 1, 1, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        istate->int_en_flags_curr.bits.pedo = 1;
    } else {

        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT1_MAP, 1, 1, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        istate->int_en_flags_curr.bits.pedo = 0;
    }
#endif

    // sync the firmware
    // 260 c2 version only has one page to configure
    rc = bmi26x_set_step_count_config(istate, BMI26X_CONFIG_INDEX_STEP_CNT, &istate->pedo_cfg);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // read back to check
    {
        uint8_t cfg_buffer[16] = {0};
        page_offset = 0;
        page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_STEP_CNT, &page_offset,
                                               BMI26X_ADVANCED_FEATURE_INPUT);

        if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
            BMI26X_INST_LOG(ERROR, istate->owner, "WARNING!!! invalid page number:%d", page_num);
            return SNS_RC_INVALID_TYPE;
        }

        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, cfg_buffer, 16);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        BMI26X_INST_LOG(LOW, istate->owner, "page :%d, 0x%x 0x%x", page_num,
                        cfg_buffer[0], cfg_buffer[1]);
    }

    return rc;
}


static sns_rc bmi26x_hal_cfg_hw_pedo(sns_sensor_instance * const inst)
{
    sns_rc rc  = SNS_RC_SUCCESS;
    bmi26x_instance_state *istate = (bmi26x_instance_state*)inst->state->state;

#if BMI26X_CONFIG_ENABLE_STEP_DETECTOR
    // INT
    if (istate->int_en_flags_req.bits.pedo) {
        rc |= bmi26x_hal_config_int_output(inst, true);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }
#endif

    // configure parameter
    rc |= bmi26x_set_step_config(istate);

    return rc;
}

static void bmi26x_hal_start_pedo_polling_timer(sns_sensor_instance *this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    BMI26X_INST_LOG(LOW, this, "will start pedo polling timer");

    if (NULL == istate->pedo_info.pedo_timer_data_stream) {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service *strm_svc =
            (sns_stream_service*) smgr->get_service(smgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this,
                istate->timer_suid, &istate->pedo_info.pedo_timer_data_stream);
    }

    if (rc != SNS_RC_SUCCESS
            || NULL == istate->pedo_info.pedo_timer_data_stream) {
        BMI26X_INST_LOG(ERROR, this, "error creating stream %d", rc);
        return;
    }

    if (!bmi26x_hal_start_timer(this, istate->pedo_info.pedo_timer_data_stream,
                           true,
                           istate->pedo_info.sampling_intvl)) {
    }
    BMI26X_INST_LOG(LOW, this, "started pedo polling timer");
}

static void bmi26x_hal_reset_counter(sns_sensor_instance *  const inst)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;
    istate->pedo_info.step_count_hw_last = 0;
}

static
void bmi26x_hal_config_pedo(sns_sensor_instance *  const inst)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;
    bool need_hw_recfg = false;

    if (istate->cfg_load_success == 0) {
        BMI26X_INST_LOG(ERROR, inst, "WARNING!!! cfg is not present, the pedo cfg rejected");
        return ;
    }

#if BMI26X_CONFIG_ENABLE_PEDO_TIMER
    if (istate->pedo_info.sampling_intvl > 0) {
        if (!istate->pedo_info.timer_is_active) {
            bmi26x_hal_start_pedo_polling_timer(inst);
            istate->pedo_info.timer_is_active = true;
            need_hw_recfg = true;

            //#ifdef VENDOR_EDIT
            istate->pedo_info.is_first = true;
            BMI26X_INST_LOG(HIGH, inst, "pedo_info.is_first: %d", istate->pedo_info.is_first);
            //#endif
            bmi26x_hal_reset_counter(inst);
        } else {
            BMI26X_INST_LOG(LOW, inst, "pedo timer already run!");
        }
    } else {
        if (istate->pedo_info.timer_is_active) {
            istate->pedo_info.timer_is_active = false;
            istate->keep_pedo_in_fac_mode = 0;
            need_hw_recfg = true;
            sns_sensor_util_remove_sensor_instance_stream(inst,
                    &istate->pedo_info.pedo_timer_data_stream);
        }
    }
#else
    if (istate->pedo_info.enable_pedo_int) { //enable
        if (!istate->pedo_info.pedo_int_is_active) {
            istate->pedo_info.pedo_int_is_active = true;
            need_hw_recfg = true;
        }
    } else { //disable
        if (istate->pedo_info.pedo_int_is_active) {
            istate->pedo_info.pedo_int_is_active = false;
            need_hw_recfg = true;
        }
    }
#endif

    if (
#if BMI26X_CONFIG_ENABLE_STEP_DETECTOR
        ((istate->pedo_cfg.bmi26x_step_counter_config.en_step_detector == 0) &&
         istate->keep_pedo_in_fac_mode)                                            ||
        ((istate->pedo_cfg.bmi26x_step_counter_config.en_step_detector == 1) &&
         (istate->keep_pedo_in_fac_mode == 0))
#else
        istate->keep_pedo_in_fac_mode
#endif
    ) {
        // mode change, update
        need_hw_recfg = true;
    }

    // hw configure
    if (need_hw_recfg) {
        if (bmi26x_hal_cfg_hw_pedo(inst) != SNS_RC_SUCCESS) {
        }
    }

}

static sns_rc bmi26x_get_step_count_hw(
    sns_sensor_instance * const inst,
    uint16_t              *step_count)
{
    sns_rc rc;
    uint8_t buf[2] = {0};
    bmi26x_instance_state   *istate = (bmi26x_instance_state *) inst->state->state;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_STEP_COUNT_OUT_0, buf, 2);

    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    *step_count = (buf[1] << 8) | buf[0];

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_hal_handle_pedo(sns_sensor_instance * const inst, sns_time event_time, uint8_t evt_type)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint16_t hw_step_counter_now = 0;
    float step_data = 1.0;
    uint8_t  need_update_pedo_count = 0;

    rc = bmi26x_get_step_count_hw(inst, &hw_step_counter_now);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

#if BMI26X_CONFIG_ENABLE_STEP_DETECTOR
    if (evt_type == BMI26X_PEDO_EVT_CTX_INT) {
        istate->pedo_info.step_count ++;
        istate->pedo_info.step_count_hw_last ++;
        need_update_pedo_count = 1;
        BMI26X_INST_LOG(LOW, inst, "event pedo");
    } else {
        if (istate->pedo_cfg.bmi26x_step_counter_config.en_step_detector == 0) {
#else
    UNUSED_VAR(evt_type);
#endif
            if (hw_step_counter_now >= istate->pedo_info.step_count_hw_last) {
                istate->pedo_info.step_count +=
                    (hw_step_counter_now - istate->pedo_info.step_count_hw_last);
                if (hw_step_counter_now > istate->pedo_info.step_count_hw_last) {
                    need_update_pedo_count = 1;
                }
                istate->pedo_info.step_count_hw_last = hw_step_counter_now;
            } else {
                if ((istate->pedo_info.step_count_hw_last - hw_step_counter_now) > 0x1000) {
                    // overflow
                    istate->pedo_info.step_count += hw_step_counter_now +
                                                    (0xffff - istate->pedo_info.step_count_hw_last);
                    need_update_pedo_count = 1;
                    istate->pedo_info.step_count_hw_last = hw_step_counter_now;
                }
            }
#if BMI26X_CONFIG_ENABLE_STEP_DETECTOR
        }
    }
#endif

    need_update_pedo_count |= istate->pedo_info.is_first;

    BMI26X_INST_LOG(LOW, inst, "pedo.hw:%d, pedo.acc:%d, update:%d, first:%d",
                    hw_step_counter_now, istate->pedo_info.step_count,
                    need_update_pedo_count,
                    istate->pedo_info.is_first);

    if (need_update_pedo_count) {
        BMI26X_INST_LOG(LOW, inst, "!update! pedo:%d", istate->pedo_info.step_count);

        step_data = (float)(istate->pedo_info.step_count * 1.0);
        //step_data = (float)(hw_step_counter_now * 1.0);
        rc = pb_send_sensor_stream_event(inst,
                                    &istate->pedo_info.sstate->my_suid,
                                    event_time,
                                    SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                    SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                    &step_data,
                                    1,
                                    istate->encoded_pedo_count_len);
        if (rc != SNS_RC_SUCCESS) {
            //
            BMI26X_INST_LOG(ERROR, inst, "ERROR!!! on pb_send_sensor_stream_event():%d",
                            rc);
        }
    }

    if (istate->pedo_info.is_first) {
        istate->pedo_info.is_first = false;
    }

    return rc;
}

sns_rc bmi26x_hal_handle_pedo_interrupt(sns_sensor_instance * const inst, sns_time event_time, uint8_t evt_type)
{
    sns_rc rc = SNS_RC_SUCCESS;

    BMI26X_INST_LOG(LOW, inst, "pedo event triggered @%u", (uint32_t) event_time);

    rc = bmi26x_hal_handle_pedo(inst, event_time, evt_type);
    return rc;
}

#endif


sns_rc bmi26x_hal_handle_timer_cmd(bmi26x_instance_state   *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_pwr_cfg = 0;
    sns_time ts = bmi26x_get_sys_tick();

    BMI26X_INST_LOG(LOW, istate->owner, "cmd timer event @%u",
                    (uint32_t)(ts & 0xFFFFFFFF));

    //bmi26x_hal_dump_reg(istate->owner);

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CONF, &regv_pwr_cfg, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS)

    if (regv_pwr_cfg & 0x01) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! power still in lower power mode"
                        " power config regv:0x%x", regv_pwr_cfg);
        // restart the timer ?
    } else {
        istate->sbus_in_normal_mode = true;
        BMI26X_INST_LOG(HIGH, istate->owner, "go out from low power:0x%x", regv_pwr_cfg);
    }

    istate->cmd_handler.cmd_in_proc = false;
    return SNS_RC_SUCCESS;
}


sns_rc bmi26x_dev_get_reg_hw_err_stat(
    void        *sbus_obj,
    union bmi26x_hw_err_stat        *hw_err_st)
{
    sns_rc                  rc;
    uint8_t                 si_buf;
    bmi26x_instance_state   *istate = (bmi26x_instance_state *)sbus_obj;

    UNUSED_VAR(istate);

    rc = bmi26x_sbus_read_wrapper(
             sbus_obj,
             BMI26X_REGA_USR_ERR_REG,
             &si_buf,
             1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    hw_err_st->bits.mag_drdy_err    = BST_GET_VAL_BITBLOCK(si_buf, 7, 7);
    hw_err_st->bits.drop_cmd_err    = BST_GET_VAL_BITBLOCK(si_buf, 6, 6);
    hw_err_st->bits.i2c_fail_err    = BST_GET_VAL_BITBLOCK(si_buf, 5, 5);
    hw_err_st->bits.err_code        = BST_GET_VAL_BITBLOCK(si_buf, 1, 4);
    hw_err_st->bits.fatal_err       = BST_GET_VAL_BITBLOCK(si_buf, 0, 0);

    if (hw_err_st->regv && (0x80 != hw_err_st->regv)) {
        BMI26X_INST_LOG(ERROR, istate->owner, "CMD_WARN!!! hw_err_st: 0x%x %d %d",
                        hw_err_st->regv,
                        hw_err_st->bits.fatal_err,
                        hw_err_st->bits.err_code);

        BMI26X_INST_LOG(ERROR, istate->owner, "CMD_WARN!!! hw_err_st: 0x%x %d %d",
                        hw_err_st->bits.mag_drdy_err,
                        hw_err_st->bits.drop_cmd_err,
                        hw_err_st->bits.i2c_fail_err);
    } else if (0x80 == hw_err_st->regv) {
        BMI26X_INST_LOG(ERROR, istate->owner, "WARNING!!! hw_err_st: mag_drdy_err");
    }

    return SNS_RC_SUCCESS;
}

static
sns_rc bmi26x_dev_get_reg_fifo_count(
    void                            *sbus_obj,
    uint16_t                        *fifo_cnt)
{
    sns_rc                              rc;;
    uint8_t si_buf[2] = "";

    rc = bmi26x_sbus_read_wrapper(sbus_obj,
                                  BMI26X_REGA_USR_FIFO_LENGTH_0,
                                  si_buf, 2);

    *fifo_cnt = ((si_buf[1] & 0x07) << 8) | si_buf[0];

    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}

static
sns_rc bmi26x_dev_sw_reset(
    sns_sensor_instance     *instance,
    uint8_t sensor)
{
    UNUSED_VAR(instance);
    UNUSED_VAR(sensor);

    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;

    UNUSED_VAR(istate);
    if (istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ) {
        //delay 2ms
        bmi26x_delay_us(2000);
    }

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_dev_pwr_conf(bmi26x_instance_state       *istate,
                           uint8_t start_bit,
                           uint8_t end_bit,
                           uint8_t val)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_target = 0;
    uint8_t regv_present = 0;

    regv_target = BST_SET_VAL_BITBLOCK(regv_target, start_bit, end_bit, val);

    if (regv_target != BST_GET_VAL_BITBLOCK(istate->regv_cache.regv_pwr_conf, start_bit, end_bit)) {
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CONF, &regv_present, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv_target = BST_SET_VAL_BITBLOCK(regv_present, start_bit, end_bit, val);

        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_PWR_CONF, &regv_target, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        if ((regv_target & 0x01) == 0) {
            bmi26x_delay_us(1000);
            // already in normal mode after 1ms
            istate->sbus_in_normal_mode = 1;
        } else {
            // target.adv = 1;
            istate->sbus_in_normal_mode = 0;
        }

        // sync to reg map
        istate->regv_cache.regv_pwr_conf = regv_target;
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "no need to update pwr_cfg <0x%x 0x%x>",
                        regv_target, istate->regv_cache.regv_pwr_conf);
    }

    return rc;
}

sns_rc bmi26x_dev_pwr_ctrl(bmi26x_instance_state       *istate,
                           uint8_t start_bit,
                           uint8_t end_bit,
                           uint8_t val)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (BST_GET_VAL_BITBLOCK(istate->regv_cache.regv_pwr_ctrl, start_bit, end_bit) != val) {

        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_PWR_CTRL,
                                          start_bit, end_bit, val);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        istate->regv_cache.regv_pwr_ctrl = BST_SET_VAL_BITBLOCK(istate->regv_cache.regv_pwr_ctrl,
                                                              start_bit, end_bit, val);
    }

    return rc;
}


sns_rc bmi26x_hal_config_power_mode(bmi26x_instance_state       *istate,
                                    uint8_t expect_power_mode)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_pwr_ctrl = 0xFF;
    uint8_t regv_adv_power_save = 0;

    if (expect_power_mode == BMI26X_POWER_MODE_LOW) {
        regv_adv_power_save = 1;
    } else if (expect_power_mode == BMI26X_POWER_MODE_NORMAL) {
        regv_adv_power_save = 0;
    } else if (expect_power_mode == BMI26X_POWER_MODE_SUSPEND) {
        regv_adv_power_save = 1;
        regv_pwr_ctrl = 0;
    } else {
        regv_adv_power_save = 0;
    }

    if (regv_pwr_ctrl == 0) {
        rc = bmi26x_dev_pwr_ctrl(istate, 1, 2, regv_pwr_ctrl);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    rc = bmi26x_dev_pwr_conf(istate, 0, 0, regv_adv_power_save);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "exp pwr:%d, <%d %d>",
                    expect_power_mode, regv_pwr_ctrl, regv_adv_power_save);

    return rc ;
}


static
void bmi26x_dev_parse_data_ts(
    const uint8_t           *buf,
    uint32_t                *ts_dev)
{
    *ts_dev = ((buf[2] << 16)
               | (buf[1] << 8)
               | (buf[0]));
}


BST_ASSUME_TS_IS_64BIT
static inline
sns_time bmi26x_hal_get_ts_backward(
    sns_time now,
    sns_time elapse)
{
    return now - elapse;
}

BST_ASSUME_TS_IS_64BIT
static inline
sns_time bmi26x_hal_get_ts_forward(
    sns_time now,
    sns_time elapse)
{
    return now + elapse;
}



static
sns_rc bmi26x_hal_read_int_ctx(
    bmi26x_instance_state           *istate,
    bmi26x_int_check_ctx_t          *int_check_ctx)
{
    sns_rc                      rc;
    int8_t                      reg_addr_start = -1;
    int8_t                      reg_addr_end = -1;
    int8_t                      len = 0;
    uint8_t                     si_buf[BMI26X_REGA_USR_FIFO_LENGTH_1 - BMI26X_REGA_USR_SENSORTIME_0 + 1] = "";


    bmi26x_int_en_flag_t        *int_en_flags = &istate->int_en_flags_curr;
    struct bmi26x_reg_int_context   *ctx = &istate->reg_int_ctx;

    sns_time                    t1;
    sns_time                    t2;

    uint32_t max_ts_dlta = istate->ticks_in_1ms;

    //if (!istate->bus_is_spi) {
        max_ts_dlta = BMI26X_MAX_TS_DLTA_MULTIPLE * istate->ticks_in_1ms;
    //}


    ctx->avail.flags = 0;
    //always include ts
    if (int_check_ctx->req_ts_hw) {
        reg_addr_start = BMI26X_REGA_USR_SENSORTIME_0;
        ctx->avail.bits.ts = 1;
        ctx->avail.bits.event = 1;
    } else {
        reg_addr_start = BMI26X_REGA_USR_INT_STATUS_0;
    }

    ctx->avail.bits.int_status_0 = 1;
    ctx->avail.bits.int_status_1 = 1;

    reg_addr_end = BMI26X_REGA_USR_INT_STATUS_1;

    if (int_en_flags->bits.fifo.flag) {
        reg_addr_end = BMI26X_REGA_USR_FIFO_LENGTH_1;
        ctx->avail.bits.temperature = 1;
        ctx->avail.bits.fifo_len = 1;
    } else {
    }

    len = reg_addr_end - reg_addr_start + 1;
    if ((len > 0) && (len <= (int8_t)BST_ARRAY_SIZE(si_buf))) {
        //t1 = bmi26x_get_sys_tick();
        rc = bmi26x_sbus_read_wrapper(istate,
                                      reg_addr_start, int_check_ctx->req_ts_hw ? si_buf : (si_buf + 4), len);
        if (rc != SNS_RC_SUCCESS) {
            ctx->avail.flags = 0;
            return rc;
        }

        if (int_check_ctx->req_ts_hw) {
            t1 = istate->ts_last_sbus_read_pre;
            t2 = bmi26x_get_sys_tick();
            uint32_t    delta;
            bool        delta_rej = false;

            delta = bmi26x_get_time_elapse_sys(t1, t2);
            BMI26X_INST_LOG(LOW, istate->owner, "NOTICE delta_i:%u %d", (uint32_t)delta,
                            (istate->async_com_read_request - istate->async_com_read_response));

#if BMI26X_CONFIG_IGNORE_TS_JITTER
#else
            if (delta <= max_ts_dlta) {
            } else {
                delta_rej = true;
            }
#endif

#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
            if (!delta_rej) {
                if (istate->ticks_delta_i > 0) {
#if BMI26X_CONFIG_IGNORE_TS_JITTER
#else
                    uint32_t devi = BST_ABS((int32_t)delta - (int32_t)istate->ticks_delta_i);
                    if ((devi << 1) <= istate->ticks_delta_i) {
                        delta_rej = false;
                        istate->ticks_delta_i = ((istate->ticks_delta_i << 3) + (delta << 1)) / 10;
                    } else {
                        delta_rej = true;
                        BMI26X_INST_LOG(HIGH, istate->owner, "devi_i:%u %u", devi, istate->ticks_delta_i);
                    }
#endif
                } else {
                    istate->ticks_delta_i = delta;
                    BMI26X_INST_LOG(HIGH, istate->owner, "delta_i_init:%u %u", delta, istate->ticks_delta_i);
                }
            }
#endif

            if (!delta_rej) {
#if BMI26X_CONFIG_ENABLE_WORKAROUND_4_SPI_SYNC_READ_LONG_DELAY
                uint32_t delay_read = (uint32_t)(istate->xfer_time_per_byte_ticks * 4);
                ctx->ts_sys = bmi26x_hal_get_ts_forward(istate->ts_last_sbus_read_pre, delay_read);
#else
#if BMI26X_CONFIG_SEE_LITE_SLOW_CLOCK
                ctx->ts_sys = bmi26x_hal_get_ts_backward(t2, istate->bus_is_spi ? 1 : 10);
#else
                ctx->ts_sys = bmi26x_hal_get_ts_backward(t2, (len - 1) * istate->xfer_time_per_byte_ticks);
#endif
#endif

                bmi26x_dev_parse_data_ts(si_buf, &ctx->ts_dev);
            } else {
                BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE delta_i:%u %u",
                                (uint32_t)delta, istate->ticks_delta_i);
                ctx->avail.bits.ts = 0;
            }

            ctx->event = si_buf[3];

            BMI26X_INST_LOG(MED, istate->owner, "ts_irq_ctx<%u,%u,0x%x>", BMI26X_SYS_TIME_LH(t2), BMI26X_SYS_TIME_LH(ctx->ts_sys),
                            ctx->ts_dev);
        }

        ctx->int_status_0 = si_buf[4];
        ctx->int_status_1 = si_buf[5];

        if (ctx->avail.bits.temperature) {
            ctx->temperature = (si_buf[BMI26X_REGA_USR_TEMPERATURE_0 - BMI26X_REGA_USR_SENSORTIME_0]
                                | (si_buf[BMI26X_REGA_USR_TEMPERATURE_1 - BMI26X_REGA_USR_SENSORTIME_0] << 8));
        }

        if (ctx->avail.bits.fifo_len) {
            ctx->fifo_len =  (si_buf[BMI26X_REGA_USR_FIFO_LENGTH_0 - BMI26X_REGA_USR_SENSORTIME_0]
                              | (si_buf[BMI26X_REGA_USR_FIFO_LENGTH_1 - BMI26X_REGA_USR_SENSORTIME_0] << 8));
        }
    } else {
        return SNS_RC_INVALID_VALUE;
    }

    return rc;
}

void bmi26x_dev_parse_int_stat_flags(
    bmi26x_reg_int_ctx_t    *ctx,
    bmi26x_int_stat_flag_t  *int_stat)
{
    int_stat->bits.md       = BST_GET_VAL_BIT(ctx->int_status_0, 6);
    int_stat->bits.step     = BST_GET_VAL_BIT(ctx->int_status_0, 1);
#if BMI26X_CONFIG_ENABLE_LOWG
    int_stat->bits.lowg     = BST_GET_VAL_BIT(ctx->int_status_0, 2);
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    int_stat->bits.dbtap = BST_GET_VAL_BIT(ctx->int_status_0, BMI26X_DOUBLE_TAP_BIT_POS_IN_INT_STATUS);
#endif


    int_stat->bits.drdy_detail.flag     = BST_GET_VAL_BITBLOCK(ctx->int_status_1, 5, 7);
    int_stat->bits.ff_full  = BST_GET_VAL_BIT(ctx->int_status_1, 0);
    int_stat->bits.ff_wml   = BST_GET_VAL_BIT(ctx->int_status_1, 1);
    int_stat->bits.err = BST_GET_VAL_BIT(ctx->int_status_1, 2);

}

sns_rc bmi26x_hal_send_cmd(
    bmi26x_instance_state   *istate,
    uint8_t                 regv_cmd)
{
    sns_rc                      rc = SNS_RC_SUCCESS;
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv_cmd, 1);
    return rc;
}

sns_rc bmi26x_hal_update_pmu_stat(
    bmi26x_instance_state   *istate,
    bool                    check_err)
{
    sns_rc                      rc;
    uint8_t                     buf[2] = {0};
    uint8_t                     regv_ss_cfg[3 + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY] = {0};
    uint8_t                     regv_pwr_buffer[2 + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY] = {0};
    uint32_t                    xfer_bytes;



    if (check_err) {
        //rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_ERR_REG, &buf, 1);
        rc = bmi26x_com_read_wrapper(istate->scp_service, &istate->com_port_info,
                                     BMI26X_REGA_USR_ERR_REG, buf, 1, &xfer_bytes);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        if (istate->bus_is_spi) {
            buf[0] = buf[1];
        }

        BMI26X_INST_LOG(MED, istate->owner, "update_pmu_stat <sbus_read> <@0x%02x:0x%02x rc:%d>",
                        BMI26X_REGA_USR_ERR_REG, buf[0], rc);
    } else {
        //some platform the odd address access will make a crash, so we make a copy from buf[0] to buf[1]
    }

    // 0x40
    //rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_ACC_CONF, regv_ss_cfg, 3);
    //BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    rc = bmi26x_com_read_wrapper(istate->scp_service, &istate->com_port_info,
                                 BMI26X_REGA_USR_ACC_CONF, regv_ss_cfg, 3, &xfer_bytes);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (istate->bus_is_spi) {
        regv_ss_cfg[0] = regv_ss_cfg[1];
        regv_ss_cfg[1] = regv_ss_cfg[2];
        regv_ss_cfg[2] = regv_ss_cfg[3];
    }

    // 0x7c
    //rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CONF, regv_pwr_buffer, 2);
    //BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_com_read_wrapper(istate->scp_service, &istate->com_port_info,
                                 BMI26X_REGA_USR_PWR_CONF, regv_pwr_buffer, 2, &xfer_bytes);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (istate->bus_is_spi) {
        regv_pwr_buffer[0] = regv_pwr_buffer[1];
        regv_pwr_buffer[1] = regv_pwr_buffer[2];
    }

    istate->sbus_in_normal_mode = bmi26x_sbus_is_in_normal_mode(istate, regv_ss_cfg[0], regv_ss_cfg[2],
                                  regv_pwr_buffer[0], regv_pwr_buffer[1]);

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_hal_prepare_spi_if(bmi26x_instance_state *istate)
{
    sns_rc                      rc;
    uint8_t                     regv;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_CHIP_ID, &regv, 1);
    bmi26x_delay_us(BMI26X_SPEC_IF_SPI_SWITCH_TIME_US);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}


sns_rc bmi26x_hal_sensor_prepare_spi_if(sns_sensor * const this)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t regv;
    sns_port_vector         port_vec;
    uint32_t                xfer_bytes = 0;
    bmi26x_state *sstate = (bmi26x_state *) this->state->state;

    port_vec.buffer = &regv;
    port_vec.bytes  = 1;
    port_vec.is_write = false;
    port_vec.reg_addr = BMI26X_REGA_USR_CHIP_ID;

    rv = sstate->scp_service->api->sns_scp_register_rw(
             sstate->common.com_port_info.port_handle,
             &port_vec, 1,
             false,
             &xfer_bytes);

    UNUSED_VAR(xfer_bytes);
    bmi26x_delay_us(BMI26X_SPEC_IF_SPI_SWITCH_TIME_US);
    BMI26X_DD_CHECK_RETVAL(rv, SNS_RC_SUCCESS)
    return  rv;
}

sns_rc bmi26x_hal_reset_sensor_state(
    sns_sensor_instance     *inst)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;

    // 0x6b
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_IF_CONF, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    regv = BST_SET_VAL_BITBLOCK(regv, 4, 5, 0);
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_IF_CONF, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(800);

    // 0x68
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_AUX_IF_TRIM, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    regv = BST_SET_VAL_BITBLOCK(regv, 0, 1, 0);
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_AUX_IF_TRIM, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(800);


    return SNS_RC_SUCCESS;
}

// @load configuration
sns_rc bmi26x_do_hal_load_cfg(sns_sensor_instance *inst, uint8_t* cfg_data_buffer,
                                     uint32_t cfg_size, uint32_t burst_write_size)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;
    /* Variable to update the configuration file index */
    uint16_t index = 0;
    /* Array to store address */
    uint8_t addr_array[2] = {0};

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_TITAN_CTRL, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    for (index = 0; index < cfg_size; index += burst_write_size) {
        //flush
        //1. len
        addr_array[0] = (uint8_t)((index / 2) & 0x0F);
        addr_array[1] = (uint8_t)((index / 2) >> 4);
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CONF_STREAM_IDX_LSB, addr_array, 2);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        //2. data
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CONF_STREAM_IN, cfg_data_buffer + index,
                                       burst_write_size);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    return rc;
}


bool bmi26x_hal_check_cfg_available(sns_sensor_instance *inst)
{
    if (BST_ASSERT_POINT(inst)) {
        return false;
    }

    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0xFF;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_UC_GP_3, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(MED, inst, "internal status:0x%x", regv);

    if ((regv & 0x0F) == 0x01) {
        BMI26X_INST_LOG(MED, inst, "YES!!! cfg is ok");
        istate->cfg_load_success = 1;
    } else {
        BMI26X_INST_LOG(HIGH, inst, "WARNING!!! configuration missing");
        istate->cfg_load_success = 0;
    }

    if (istate->cfg_load_success) {
        return true;
    }
    return false;
}

sns_rc bmi26x_hal_load_sensor_cfg(
    sns_sensor_instance     *inst)
{
    if (BST_ASSERT_POINT(inst)) {
        return SNS_RC_INVALID_VALUE;
    }
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_NOT_AVAILABLE;
    uint8_t regv = 0;
    uint8_t try_num_to_check_cfg = 0;
    bool cfg_is_available = bmi26x_hal_check_cfg_available(inst);


    if (cfg_is_available) {
        // already loaded
        return SNS_RC_SUCCESS;
    }

#if BMI26X_CONFIG_ENABLE_RESET_BEFORE_LOAD_CONFIG
    regv = BMI26X_REGV_CMD_SOFT_RESET;
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(MED, inst, "reset failure:0x%x", rc);
    }
    bmi26x_delay_us(2000);   //2ms

    istate->inst_inted = 0;
    /* prepare sensor*/
    rc = bmi26x_hal_prepare_hw(istate);
#endif

    //BMI26X_REGA_USR_PWR_CONF
    rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    bmi26x_delay_us(500);

    //  exit from island to access non-island resource
    bmi26x_hal_inst_exit_island(inst);

    // load configuration now
    rc = bmi26x_do_hal_load_cfg(inst, bmi26x_sensor_cfg, bmi26x_sensor_cfg_size,
                                BMI26X_CONFIG_BURST_WRITE_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    regv = 0x01;
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_TITAN_CTRL, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    cfg_is_available = false;
    rc = SNS_RC_NOT_AVAILABLE;
    do {
        bmi26x_delay_us(10 * 1000);   //10ms
        cfg_is_available = bmi26x_hal_check_cfg_available(inst);
        try_num_to_check_cfg ++;
        if (cfg_is_available) {
            rc = SNS_RC_SUCCESS;
            break;
        }
    } while((try_num_to_check_cfg < BMI26X_CHECK_CONFIGURE_STATUS_TIMES) &&
            (!cfg_is_available));

    return rc;
}


void bmi26x_hal_set_sensor_state_2_default_state(
    sns_sensor_instance         *inst,
    uint8_t sensor)
{
    UNUSED_VAR(sensor);
    bmi26x_instance_state *istate = (bmi26x_instance_state *)inst->state->state;

    // acc/gyr
    istate->accel_info.odr_curr = BMI26X_REGV_ODR_OFF;
    istate->accel_info.range_curr = BMI26X_REGV_RANGE_ACC_PM8G;

    istate->gyro_info.odr_curr = BMI26X_REGV_ODR_OFF;
    istate->gyro_info.range_curr = BMI26X_REGV_RANGE_GYR_PM2000DPS;


    // cfg context
    istate->inst_inted = 0;
    //istate->fac_test_in_progress = 0;
    istate->hw_config_pending = 0;
    istate->config_step = BMI26X_CONFIG_IDLE;

    istate->sbus_in_normal_mode = false;

    istate->int_en_flags_curr.flag = 0;

    istate->cfg_current_pg_num = BMI26X_DEFAULT_PAGE_NUM;

    // regv map
    istate->ois_info.en = 0;
    istate->regv_cache.regv_pwr_ctrl = 0x00;
    istate->regv_cache.regv_pwr_conf = 0x03;

    // fifo info
    istate->fifo_info.ff_flush_in_proc = 0;
    istate->fifo_info.ff_sensors_en_curr = 0;
    istate->fifo_info.ff_wml_bytes_curr = 0;
    istate->fifo_info.ff_master_sensors_ready = 0;
    istate->fifo_info.ff_sensors_to_invalidate = 0;
    istate->fifo_info.ff_sensors_flush_done_before_invalid = 0;
    istate->fifo_info.ff_tm_info.avail_ts_last_batch = 0x00;
    istate->fifo_info.ff_tm_info.avail_ts_this_batch = 0x00;

    istate->fifo_info.ff_tm_info.fc_accum_curr_acc = 0;
    istate->fifo_info.ff_tm_info.fc_accum_curr_gyr = 0;

    istate->async_com_read_request = istate->async_com_read_response = 0;

    istate->cnt_hw_res_est = 0;
#if BMI26X_CONFIG_ENABLE_LOWG
    istate->lowg_info.enable_lowg_int = 0;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    istate->dbtap_info.enable_dbtap_int = 0;
#endif

#if BMI26X_CONFIG_ENABLE_CRT
    // crt
    sns_memset(&istate->crt_gain_state, 0, sizeof(istate->crt_gain_state));
#endif

    istate->regv_cache.regv_acc_cfg = 0xA8;  //@0x40
    istate->regv_cache.regv_gyr_cfg = 0xA9;  //@0x42
    istate->regv_cache.regv_fifo_config_0 = 0x02;  //@0x48
    istate->regv_cache.regv_fifo_config_1 = 0x10;  //@0x49
}


sns_rc bmi26x_hal_reset_device(
    sns_sensor_instance     *instance,
    uint8_t sensor)
{
    sns_rc                      rc = SNS_RC_SUCCESS;

    uint8_t                     regv;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    /** HW reset only when both Accel and Gyro are requested for
     *  reset. */
    if (sensor == (uint8_t)(BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MOTION_DETECT | BMI26X_SENSOR_TEMP)) {
        rc = bmi26x_dev_sw_reset(instance, sensor);

        if (SNS_BUS_SPI == istate->com_port_info.com_config.bus_type) {
            if (bmi26x_hal_prepare_spi_if(istate) != SNS_RC_SUCCESS) {
            }
        }

        //TO speed up the configuration process
        rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_CHIP_ID, &regv, 1);
        rc |= bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_EVENT, &regv, 1);
        rc |= bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CTRL, &regv, 1);

        // clear latch and configure int output
        regv = 0;
        rc |= bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_INT_LATCH, &regv, 1);
        rc |= bmi26x_hal_config_int_output(instance, true);
        if (rc == SNS_RC_SUCCESS) {
            bmi26x_hal_set_sensor_state_2_default_state(instance, (uint8_t)sensor);
            // because the advanced power already cleared before h/w operation
            istate->sbus_in_normal_mode = 1;
        }
    }

    return rc;
}


/**
 * see sns_bmi26x_hal.h
 */
sns_rc bmi26x_hal_reset_device_wrapper(sns_sensor_instance *inst)
{
    uint8_t sensors = (uint8_t) (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MOTION_DETECT | BMI26X_SENSOR_TEMP);
    sns_rc rc = bmi26x_hal_reset_device(inst, sensors);
    if (rc == SNS_RC_SUCCESS) {
        rc = bmi26x_hal_reset_sensor_state(inst);
    }
    return rc;
}

sns_rc bmi26x_hal_reset_load_cfg_wrapper(sns_sensor_instance *const inst)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = BMI26X_REGV_CMD_SOFT_RESET;

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
    bmi26x_delay_us(8 * 1000);

    // reset the sensor status
    rc = bmi26x_hal_reset_device_wrapper(inst);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_hal_load_sensor_cfg(inst);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}


/**
 * see sns_bmi26x_hal.h
 */
sns_rc bmi26x_hal_get_who_am_i(
    sns_sync_com_port_service   *scp_service,
    bmi26x_com_port_info_t      *port_info,
    uint8_t                     *chip_id,
    uint8_t                     *dummy_byte)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t                        xfer_bytes = 0;
    uint8_t                         buffer[2] = "";

    rv = bmi26x_com_read_wrapper(scp_service,
                                 port_info,
                                 BMI26X_REGA_USR_CHIP_ID,
                                 buffer, 2, &xfer_bytes);

    if (SNS_RC_SUCCESS == rv) {
        if (2 != xfer_bytes) {
            rv = SNS_RC_NOT_AVAILABLE;
        }
    }

    if (NULL != dummy_byte) {
        *dummy_byte = buffer[0];
    }

    *chip_id = buffer[1];

    return rv;
}


void bmi26x_hal_match_odr(
    bmi26x_sensor_type      sensor_type,
    float                   odr_req,
    float                   *odr_matched,
    bmi26x_regv_odr_t       *regv)
{
    uint8_t i;
    struct bmi26x_odr_regv_map *map;

    uint8_t                     regv_min = 0;

    *regv = BMI26X_REGV_ODR_OFF;
    *odr_matched = 0;

    if (BST_IS_FLOAT_ZERO(odr_req)) {
        return;
    }

    if (BMI26X_ACCEL == sensor_type) {
        regv_min = BMI26X_REGV_ODR_12_5HZ;
    } else if (BMI26X_GYRO == sensor_type) {
        regv_min = BMI26X_REGV_ODR_25HZ;
    }

    //TODOMAG

    for (i = 0; i < ARR_SIZE(BMI26X_REGV_ODR_MAP); i++) {
        map = BMI26X_REGV_ODR_MAP + i;

        if (odr_req <= map->odr) {
            if (map->regv >= regv_min)  {
                *regv = (bmi26x_regv_odr_t) (map->regv);
                *odr_matched = map->odr;
                break;
            }
        }
    }
}


sns_rc bmi26x_hal_config_int_output(
    sns_sensor_instance     *const  instance,
    bool                    enable)
{
    sns_rc                      rc;
    bmi26x_instance_state      *istate = NULL;
    uint8_t                     regv;

    if (BST_ASSERT_POINT(instance)) {
        return SNS_RC_INVALID_VALUE;
    }

    istate = (bmi26x_instance_state *)instance->state->state;

    if (enable) {
        regv = 0x05;
    } else {
        regv = 0x00;
    }

    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT1_IO_CTRL, 1, 4, regv);

    return rc;
}

/**
 * see sns_bmi26x_hal.h
 */
//OPTIM
void bmi26x_hal_dump_reg(sns_sensor_instance     *this)
{
#if BMI26X_CONFIG_ENABLE_DUMP_REG
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    uint8_t                     regv[48] = {0};
    uint8_t                     *regvp = regv;
    sns_rc rc = SNS_RC_SUCCESS;

    struct group_read {
        uint32_t first_reg;
        uint8_t  num_regs;
    } groups[] = { /* must fit within state->reg_status[] */
        { BMI26X_REGA_USR_CHIP_ID, 3 },
        //dump more
        { BMI26X_REGA_USR_DATA_ACC_X_LSB, 12 },
        { BMI26X_REGA_USR_SATURATION, 1 },
        // acc configure
        { BMI26X_REGA_USR_ACC_CONF, 6 },
        // fifo
        { BMI26X_REGA_USR_FIFO_WTM_0, 4 },
        // int control
        { BMI26X_REGA_USR_INT1_IO_CTRL, 3},
        // INT map
        { BMI26X_REGA_USR_INT1_MAP, 4},
        // internal status
        { BMI26X_REGA_USR_INTERNAL_ERROR, 1 },
        // user interface
        { BMI26X_REGA_USR_IF_CONF, 1 },
        // crt gain
        {BMI26X_REGA_CRT_CONFIG, 4},
        // power configure
        { BMI26X_REGA_USR_PWR_CONF, 2 },
    };

    for (uint32_t i = 0; i < ARR_SIZE(groups); i ++) {
        rc = bmi26x_sbus_read_wrapper(istate, groups[i].first_reg,
                                      regvp, groups[i].num_regs);
        for (uint32_t j = 0; j < groups[i].num_regs; j ++) {
            BMI26X_INST_LOG(HIGH, this, "<dump_core_reg> <@0x%02x:0x%02x rc:%d>",
                            groups[i].first_reg + j, regvp[j], rc);
        }
        regvp += groups[i].num_regs;
    }

    BMI26X_INST_LOG(MED, this, "<dump map reg> <0x%x 0x%x>",
                    istate->regv_cache.regv_pwr_conf,
                    istate->regv_cache.regv_pwr_ctrl);

#else
    UNUSED_VAR(this);
#endif
}


#define BMI26X_SPEC_MD_THRESH_RESOLUTION (1.22e-4)
static
void bmi26x_hal_determine_md_param(
    bmi26x_instance_state       *istate,
    bmi26x_int_cfg_any_motion_t *param)
{
    uint16_t param_thresh;
    uint16_t param_dur;

    float dur_ms;
    float threshold_g;

    // duration
    dur_ms = istate->md_info.sstate->md_config.win * 1000.0;  //s -> ms
    if (dur_ms < BMI26X_MOTION_DETECTION_DURATION_MIN_MS) {
        dur_ms = BMI26X_MOTION_DETECTION_DURATION_MIN_MS;
    } else if (dur_ms > BMI26X_MOTION_DETECTION_DURATION_MAX_MS) {
        dur_ms = BMI26X_MOTION_DETECTION_DURATION_MAX_MS;
    }
    param_dur = (uint16_t)((dur_ms / BMI26X_MOTION_DETECTION_DURATION_BASE_MS) + 0.5);
    if (param_dur > BMI26X_MOTION_DETECTION_REGV_DURATION) {
        param_dur = BMI26X_MOTION_DETECTION_REGV_DURATION;
    }

    // threshold
    threshold_g = istate->md_info.sstate->md_config.thresh ;
    if (threshold_g < BMI26X_MOTION_DETECTION_THRESHOLD_MIN_G) {
        threshold_g = BMI26X_MOTION_DETECTION_THRESHOLD_MIN_G;
    } else if (threshold_g > BMI26X_MOTION_DETECTION_THRESHOLD_MAX_G) {
        threshold_g = BMI26X_MOTION_DETECTION_THRESHOLD_MAX_G;
    }

    param_thresh = (uint16_t)((threshold_g * 2048.0) + 0.5);
    if (param_thresh > BMI26X_MOTION_DETECTION_REGV_THRESHOLD) {
        param_thresh = BMI26X_MOTION_DETECTION_REGV_THRESHOLD;
    }

    param->int_anym_dur = (uint16_t)param_dur;
    param->int_anym_th = (uint16_t)param_thresh;
}

static sns_rc bmi26x_hal_cfg_md(bmi26x_instance_state *istate, bmi26x_int_cfg_any_motion_t *param, bool en)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_num;
    uint8_t page_offset;
    uint8_t *md_cfg = NULL;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_ANY_MOTION, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    if (page_num <= BMI26X_MAX_CONFIG_PAGE_NUM) {
        md_cfg = (uint8_t *)(&istate->md_cfg);
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, md_cfg,
                                 sizeof(istate->md_cfg.settings));
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        BMI26X_INST_LOG(MED, istate->owner, "@r, page :%d, offset:%d, en:%d, dur:%d, thresold:%d",
                        page_num, page_offset,
                        istate->md_cfg.bmi26x_motion_detection_config.en_md,
                        istate->md_cfg.bmi26x_motion_detection_config.duration,
                        istate->md_cfg.bmi26x_motion_detection_config.threshold);
    } else {
        BMI26X_INST_LOG(ERROR, istate->owner, "WARNING! invalid configure page number:%d", page_num);
        return rc;
    }

    istate->md_cfg.bmi26x_motion_detection_config.en_md = en;
    if (en) {
        if (param == NULL) {
            BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! md param error");
            return SNS_RC_INVALID_VALUE;
        }
        if (param->int_anym_dur > 0) {
            istate->md_cfg.bmi26x_motion_detection_config.duration = param->int_anym_dur;
        }
        if (param->int_anym_th > 0) {
            istate->md_cfg.bmi26x_motion_detection_config.threshold = param->int_anym_th;
        }
    }

    istate->md_cfg.bmi26x_motion_detection_config.select_x = 1;
    istate->md_cfg.bmi26x_motion_detection_config.select_y = 1;
    istate->md_cfg.bmi26x_motion_detection_config.select_z = 1;

    // sync to configuration
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, md_cfg, sizeof(istate->md_cfg.settings));
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "@w, page :%d, offset:%d, en:%d, dur:%d, thresold:%d",
                    page_num, page_offset,
                    istate->md_cfg.bmi26x_motion_detection_config.en_md,
                    istate->md_cfg.bmi26x_motion_detection_config.duration,
                    istate->md_cfg.bmi26x_motion_detection_config.threshold);

    {
        uint16_t md_parm[2] = {0};
        bmi26x_cfg_md_t *p_md_cfg = (bmi26x_cfg_md_t *)md_parm;
        md_cfg = (uint8_t *)(md_parm);

        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, md_cfg,
                                 sizeof(istate->md_cfg.settings));
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        BMI26X_INST_LOG(MED, istate->owner, "@wr, page :%d, offset:%d, 0x%x, 0x%x,"
                        "en:%d, out_cfg:%d, dur:%d, thresold:%d, xyz:0x%x",
                        page_num, page_offset, md_parm[0], md_parm[1],
                        p_md_cfg->bmi26x_motion_detection_config.en_md,
                        p_md_cfg->bmi26x_motion_detection_config.out_conf,
                        p_md_cfg->bmi26x_motion_detection_config.duration,
                        p_md_cfg->bmi26x_motion_detection_config.threshold,
                        p_md_cfg->bmi26x_motion_detection_config.select_x |
                        p_md_cfg->bmi26x_motion_detection_config.select_y << 1 |
                        p_md_cfg->bmi26x_motion_detection_config.select_y << 2
                       );
    }

    return rc;
}

sns_rc bmi26x_hal_set_md_config(sns_sensor_instance *const instance, bool enable)
{
    sns_rc                      rc;
    uint8_t                     regv;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;

    if (enable) {
        bmi26x_int_cfg_any_motion_t param_md;

        bmi26x_hal_determine_md_param(istate, &param_md);

        rc = bmi26x_hal_cfg_md(istate, &param_md, true);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv = 0;
        regv = BST_CLR_VAL_BIT(regv, 7);
        // odr
        regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, BMI26X_CONFIG_MD_ACC_ODR);
        regv = BST_SET_VAL_BITBLOCK(regv, 4, 6, BMI26X_CONFIG_MD_ACC_BWP);
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_ACC_CONF, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    } else {
    }

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_hal_config_int_md(
    sns_sensor_instance     *const  instance,
    bool                    enable,
    bool                    md_not_armed_event,
    uint8_t               trigger)
{
    sns_rc                      rc;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;

    BMI26X_INST_LOG(MED, instance, "MD cfg:%d %d %d %d %d, ev.num:%d",
                    istate->md_info.client_present,
                    istate->int_en_flags_req.bits.md,
                    enable,
                    md_not_armed_event,
                    trigger,
                    (uint32_t)istate->md_info.md_event_num
                    );

    if (enable) {
        //rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH, 0, 0, BMI26X_CONFIG_INT_LATCH_REGV);
        //BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT1_MAP, 6, 6, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        istate->md_info.md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_ENABLED;
    } else {
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT1_MAP, 6, 6, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        /*
        if (istate->int_en_flags_curr.bits.lowg) {
        } else {
            rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH, 0, 0, 0);
        }
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        */

        istate->md_info.md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
    }

    if (enable || md_not_armed_event) {
        bool avoid_duplicate_md_ev;

        avoid_duplicate_md_ev = (istate->int_en_flags_req.bits.md && istate->int_en_flags_curr.bits.md)
                                && (!md_not_armed_event);

        BMI26X_INST_LOG(MED, istate->owner,
                        "bmi26x_update_md_intr md_ev_type:%d %d",
                        istate->md_info.md_state.motion_detect_event_type,
                        istate->int_en_flags_req.bits.md | (istate->int_en_flags_curr.bits.md << 1) | (istate->md_info.client_present << 2));

        if (!avoid_duplicate_md_ev) {
            BMI26X_INST_LOG(MED, istate->owner, "#MD# pb event on:%d", istate->md_info.md_state);
            pb_send_event(instance,
                          sns_motion_detect_event_fields,
                          &istate->md_info.md_state,
                          sns_get_system_time(),
                          SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                          &istate->md_info.sstate->my_suid);
        }
    }

    return SNS_RC_SUCCESS;
}


sns_rc bmi26x_hal_config_int_fifo(
    sns_sensor_instance     *const  instance,
    bool                    enable)
{
    sns_rc                      rc;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    uint8_t                     regv;

    if (enable) {
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_INT_MAP_HW, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        if (BST_GET_VAL_BITBLOCK(regv, 0, 1) != 3) {
            regv = BST_SET_VAL_BITBLOCK(regv, 0, 1, 3);
            //regv = BST_SET_VAL_BIT(regv, 2);  //forced data ready mode
            rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_INT_MAP_HW, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
            istate->regv_cache.cache_regv_int_map_1 = regv;
        }
    } else {
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_MAP_HW, 0, 1, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    return SNS_RC_SUCCESS;
}

#if BMI26X_CONFIG_ENABLE_DRI_MODE
sns_rc bmi26x_hal_config_int_drdy(
    sns_sensor_instance     *const  instance,
    bool                    enable)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;

    if (enable) {
        // DRDY INT enable @0x58
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_MAP_HW, 2, 2, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_INT_MAP_HW, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        istate->regv_cache.cache_regv_int_map_1 = regv;
    } else {
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_MAP_HW, 2, 2, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    return SNS_RC_SUCCESS;
}
#endif


/** See sns_bmi26x_hal.h */
void bmi26x_hal_send_config_event(sns_sensor_instance *const instance)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)instance->state->state;
    sns_std_sensor_physical_config_event phy_sensor_config =
        sns_std_sensor_physical_config_event_init_default;

    char operating_mode[] = "NORMAL";
    char operating_mode_lp[] = "LOW POWER";
    pb_buffer_arg op_mode_args;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    struct bmi26x_state         *sstate;

    /** If no sensors are enabled for streaming then don't send
     *  config event */
    if (!istate->hw_mod_needed) {
        return;
    }

    BMI26X_INST_LOG(MED, instance, "send cfg event for sensors:0x%x",
                    istate->fifo_info.publish_sensors);

    phy_sensor_config.has_sample_rate = true;
    phy_sensor_config.sample_rate = 0.0;

    phy_sensor_config.has_water_mark = true;
    //phy_sensor_config.water_mark = istate->accel_info.ff_wml_req | (istate->fac_test_in_progress << 16);
    phy_sensor_config.water_mark = istate->accel_info.ff_wml_req;
    phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = BMI26X_ACC_ACTIVE_CURRENT;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.has_stream_is_synchronous = true;
    phy_sensor_config.stream_is_synchronous = false;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = true;
    /* For sensors that route data through the SDC/DAE sensor, the DAE watermark
       should be set to the number of samples stored in SDC before waking up Q6. */
    phy_sensor_config.has_DAE_watermark = bmi26x_dae_if_available(instance);

#if BMI26X_CONFIG_ENABLE_DAE
    if (istate->accel_info.batch_flush_flag || istate->gyro_info.batch_flush_flag) {
        phy_sensor_config.DAE_watermark = UINT32_MAX;
    } else {
        phy_sensor_config.DAE_watermark  =
        (istate->accel_info.flush_period_ticks == 0) ?
        UINT32_MAX :
        istate->fifo_info.ff_wml_bytes_curr / 7;
    }
#else
    phy_sensor_config.DAE_watermark = 0;
#endif

    if (fifo_info->publish_sensors & BMI26X_ACCEL) {
        sstate = istate->accel_info.sstate;
        phy_sensor_config.resolution = bmi26x_accel_resolutions[sstate->resolution_idx];
        phy_sensor_config.range[0] = bmi26x_accel_ranges[sstate->resolution_idx].min;
        phy_sensor_config.range[1] = bmi26x_accel_ranges[sstate->resolution_idx].max;

        phy_sensor_config.sample_rate = BMI26X_REGV_ODR_MAP[istate->accel_info.odr_curr].odr ;

        phy_sensor_config.water_mark = SNS_MAX(phy_sensor_config.water_mark, (uint32_t)(1));    //per req alignment from vikram
#if BMI26X_CONFIG_ENABLE_SEE_LITE
        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
#else
        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);
#endif

        BMI26X_INST_LOG(MED, instance, "acc: sr(100):%u, wml:<%u %u> dae:<%u %u>",
                        (uint32_t) (phy_sensor_config.sample_rate * 100.f),
                        (uint32_t)phy_sensor_config.water_mark,
                        istate->fifo_info.ff_wml_bytes_curr,
                        (uint32_t)phy_sensor_config.has_DAE_watermark,
                        (uint32_t) (phy_sensor_config.DAE_watermark)
                       );

        if (!pb_send_event(instance,
                                     sns_std_sensor_physical_config_event_fields,
                                     &phy_sensor_config,
                                     sns_get_system_time(),
                                     SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                                     &sstate->my_suid)) {
            // TODO
        }
    }

    if (fifo_info->publish_sensors & BMI26X_GYRO) {
#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
        phy_sensor_config.sample_rate = (istate->gyro_info.downsample_sw_factor <= 1) ?
                                        //BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_req].odr : istate->gyro_info.sample_rate_req;
                                        BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr : (BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr /
                                                istate->gyro_info.downsample_sw_factor);

        phy_sensor_config.water_mark  = (istate->gyro_info.downsample_sw_factor <= 1) ?
                                        istate->gyro_info.ff_wml_curr : (istate->gyro_info.ff_wml_curr / istate->gyro_info.downsample_sw_factor);
        phy_sensor_config.water_mark = (uint32_t)SNS_MAX(phy_sensor_config.water_mark, (uint32_t)1);
#else
        phy_sensor_config.sample_rate = BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr;

        phy_sensor_config.water_mark = istate->gyro_info.ff_wml_curr;
        phy_sensor_config.water_mark = SNS_MAX(phy_sensor_config.water_mark, (uint32_t)(1));    //per req alignment from vikram
#endif
        // Override above values with gyro info
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = BMI26X_GYRO_ACTIVE_CURRENT;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = true;

        sstate = istate->gyro_info.sstate;
        phy_sensor_config.resolution = bmi26x_gyro_resolutions[sstate->resolution_idx];
        phy_sensor_config.range[0] = bmi26x_gyro_ranges[sstate->resolution_idx].min;
        phy_sensor_config.range[1] = bmi26x_gyro_ranges[sstate->resolution_idx].max;

        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);

        BMI26X_INST_LOG(MED, instance, "gyro: sr:%u, wml:<%u %u> dae:<%u %u>",
                        (uint32_t) (phy_sensor_config.sample_rate),
                        (uint32_t) (phy_sensor_config.water_mark),
                        istate->fifo_info.ff_wml_bytes_curr,
                        (uint32_t)phy_sensor_config.has_DAE_watermark,
                        (uint32_t) (phy_sensor_config.DAE_watermark)
                       );

        if (!pb_send_event(instance,
                      sns_std_sensor_physical_config_event_fields,
                      &phy_sensor_config,
                      sns_get_system_time(),
                      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                      &sstate->my_suid)) {
            // TODO
        }
    }

    if (fifo_info->publish_sensors & BMI26X_SENSOR_TEMP) {
        // Override above values with sensor temperature info
        phy_sensor_config.sample_rate = istate->sensor_temp_info.sample_rate_req;

        phy_sensor_config.has_water_mark = false;
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 180;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = BMI26X_SENSOR_TEMPERATURE_RESOLUTION;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = BMI26X_SENSOR_TEMPERATURE_RANGE_MIN;
        phy_sensor_config.range[1] = BMI26X_SENSOR_TEMPERATURE_RANGE_MAX;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = false;
        phy_sensor_config.has_DAE_watermark = false;
        phy_sensor_config.DAE_watermark = 1;

#if BMI26X_CONFIG_ENABLE_SEE_LITE
        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
#else
        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);
#endif
        if (!pb_send_event(instance,
                      sns_std_sensor_physical_config_event_fields,
                      &phy_sensor_config,
                      sns_get_system_time(),
                      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                      &istate->sensor_temp_info.sstate->my_suid)) {
            // TODO
        }
    }

    //this section has to be at the very end because of the sns_memset
    if (istate->int_en_flags_curr.bits.md) {
        sns_memset(&phy_sensor_config, 0, sizeof(phy_sensor_config));

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 6;  //per configuration: ODR=50,AVG=1
        phy_sensor_config.has_resolution = false;
        phy_sensor_config.range_count = 0;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = true;
        phy_sensor_config.has_stream_is_synchronous = true;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_DAE_watermark = false;

        if (!pb_send_event(instance,
                           sns_std_sensor_physical_config_event_fields,
                           &phy_sensor_config,
                           sns_get_system_time(),
                           SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                           &istate->md_info.sstate->my_suid)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! send cfg event:@ss:%d", BMI26X_MOTION_DETECT);
        }
    }

#if BMI26X_CONFIG_ENABLE_PEDO
    if (fifo_info->publish_sensors & BMI26X_PEDO) {
        BMI26X_INST_LOG(MED, instance, "ticks:%u ",
                        istate->pedo_info.sampling_intvl);

        if (istate->pedo_info.sampling_intvl > 0) {
            phy_sensor_config.sample_rate = (float)(sns_convert_ns_to_ticks(1e9) / istate->pedo_info.sampling_intvl);
        }
        phy_sensor_config.has_water_mark = false;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 180;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = 1.0;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = BMI26X_PEDO_RANGE_MIN;
        phy_sensor_config.range[1] = BMI26X_PEDO_RANGE_MAX;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = false;
        phy_sensor_config.has_DAE_watermark = false;
        phy_sensor_config.DAE_watermark = 1;

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;

        if (!pb_send_event(instance,
                      sns_std_sensor_physical_config_event_fields,
                      &phy_sensor_config,
                      sns_get_system_time(),
                      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                      &istate->pedo_info.sstate->my_suid)) {
            // TODO
        }
    }
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    if (fifo_info->publish_sensors & BMI26X_OIS) {
        phy_sensor_config.has_water_mark = false;
        phy_sensor_config.has_sample_rate = false;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 180;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = BMI26X_GYRO_RESOLUTION_250DPS;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = BMI26X_GYRO_RANGE_250_MIN;
        phy_sensor_config.range[1] = BMI26X_GYRO_RANGE_250_MAX;
        phy_sensor_config.has_dri_enabled = false;
        phy_sensor_config.dri_enabled = false;
        phy_sensor_config.has_DAE_watermark = false;
        phy_sensor_config.DAE_watermark = 1;

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;

        if (!pb_send_event(instance,
                      sns_std_sensor_physical_config_event_fields,
                      &phy_sensor_config,
                      sns_get_system_time(),
                      SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                      &istate->ois_info.sstate->my_suid)) {
            // TODO
        }
    }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    if (istate->int_en_flags_curr.bits.lowg) {
        sns_memset(&phy_sensor_config, 0, sizeof(phy_sensor_config));

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 6;
        phy_sensor_config.has_resolution = false;
        phy_sensor_config.range_count = 0;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = true;
        phy_sensor_config.has_stream_is_synchronous = true;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_DAE_watermark = false;

        if (!pb_send_event(instance,
                           sns_std_sensor_physical_config_event_fields,
                           &phy_sensor_config,
                           sns_get_system_time(),
                           SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                           &istate->lowg_info.sstate->my_suid)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! send cfg event:@ss:%d", BMI26X_FREE_FALL);
        }
    }
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    if (istate->int_en_flags_curr.bits.dbtap) {
        sns_memset(&phy_sensor_config, 0, sizeof(phy_sensor_config));

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 6;
        phy_sensor_config.has_resolution = false;
        phy_sensor_config.range_count = 0;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = true;
        phy_sensor_config.has_stream_is_synchronous = false;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_DAE_watermark = false;

        if (!pb_send_event(instance,
                           sns_std_sensor_physical_config_event_fields,
                           &phy_sensor_config,
                           sns_get_system_time(),
                           SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                           &istate->dbtap_info.sstate->my_suid)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! send cfg event:@ss:%d", BMI26X_DOUBLE_TAP);
        }
    }
#endif



    if (!istate->fac_test_in_progress) {
        if (fifo_info->publish_sensors & BMI26X_ACCEL) {
            bmi26x_send_fac_cal_event(instance, istate->accel_info.sstate);
        }

        if (fifo_info->publish_sensors & BMI26X_GYRO) {
            bmi26x_send_fac_cal_event(instance, istate->gyro_info.sstate);
        }
    }
}


void bmi26x_dev_format_data_temp(
    const uint8_t           buf[],
    float                   *temp)
{
    int16_t temp_lsb;
    static const float BMI26X_SCALE_FACTOR_TEMP = 1.0f / 512;

    temp_lsb = (int16_t)buf[0] | (((int16_t)buf[1]) << 8);

    *temp = (temp_lsb * BMI26X_SCALE_FACTOR_TEMP) + 23;
}

void bmi26x_hal_convert_and_send_temp_sample(
    sns_sensor_instance     *const instance,
    sns_time                timestamp,
    const uint8_t           buf[2])
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    float                       float_temp_val;
    sns_rc rc = SNS_RC_SUCCESS;

    bmi26x_dev_format_data_temp(buf, &float_temp_val);

    BMI26X_INST_LOG(MED, instance, "temp:lsb 0x%x 0x%x, %d", buf[0], buf[1],
            (int16_t) float_temp_val);

    // factory calibration
    // Sc = C * (Sr - B)
    // Where,
    // *Sc = Calibrated sample
    // *Sr = Raw sample
    // *C = Scale
    // *B = Bias
    //
    int32_t *fac_cal_bias;
    fac_cal_bias = istate->sensor_temp_info.sstate->fac_cal_bias;

    float_temp_val = istate->sensor_temp_info.sstate->fac_cal_corr_mat.e00 *
                     (float_temp_val - (fac_cal_bias[0] / BMI26X_SCALE_FACTOR_DATA_TEMP));

    rc = pb_send_sensor_stream_event(instance,
                                &istate->sensor_temp_info.sstate->my_suid,
                                timestamp,
                                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                &float_temp_val,
                                1,
                                istate->encoded_sensor_temp_event_len);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, instance, "ERROR!!! on pb_send_sensor_stream_event:%d",
                        rc);
    }
}


//OPTIM
void bmi26x_hal_start_sensor_temp_polling_timer(sns_sensor_instance *this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t buffer[50];
    sns_request timer_req = {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };
    sns_rc rc = SNS_RC_SUCCESS;

    BMI26X_INST_LOG(MED, this, "start temperature polling timer");

    if (NULL == istate->timer_data_stream) {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service *strm_svc =
            (sns_stream_service*) smgr->get_service(smgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this,
                istate->timer_suid, &istate->timer_data_stream);
    }

    if (rc != SNS_RC_SUCCESS
            || NULL == istate->timer_data_stream) {
        BMI26X_INST_LOG(ERROR, this, "ERROR!!! error creating stream %d", rc);
        return;
    }
    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = true;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = istate->sensor_temp_info.sampling_intvl_curr;

    timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                            sns_timer_sensor_config_fields, NULL);
    if (timer_req.request_len > 0) {
        istate->timer_data_stream->api->send_request(istate->timer_data_stream, &timer_req);
        istate->sensor_temp_info.timer_is_active = true;
    } else {
        BMI26X_INST_LOG(ERROR, this, "ERROR!!! pb.encode.error: %d", rc);
    }
}

static
sns_rc bmi26x_hal_update_couple_ts_host_and_dev_rt(
    bmi26x_instance_state   *istate)
{
    sns_rc                      rc;
    sns_time                    ts;
    sns_time                    te;
    uint8_t                     si_buf[3] = "";
    uint32_t                    delta;
    bool                        delta_rej = false;
    uint32_t max_ts_dlta        = istate->ticks_in_1ms;

    if (BMI26X_FIFO_READ_CTX_TYPE_DAE == istate->fifo_info.ff_read_ctx_type) {
#if BMI26X_CONFIG_ENABLE_DAE
        istate->ts_pair_sys_dev.ts_sys = istate->dae_if.ts_dae_read_fifo_sys_time;
        istate->ts_pair_sys_dev.ts_dev = istate->dae_if.ts_dae_read_fifo_dev_time;
#endif
    } else {
        //if (!istate->bus_is_spi) {
            max_ts_dlta = BMI26X_MAX_TS_DLTA_MULTIPLE * istate->ticks_in_1ms;
        //}

        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_SENSORTIME_0,
                                      si_buf, 3);
        ts = istate->ts_last_sbus_read_pre;
        te = bmi26x_get_sys_tick();
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        delta = bmi26x_get_time_elapse_sys(ts, te);

#if BMI26X_CONFIG_IGNORE_TS_JITTER
#else
        if (delta <= max_ts_dlta) {
        } else {
            delta_rej = true;
        }
#endif

#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
        BMI26X_INST_LOG(LOW, istate->owner, "NOTICE delta_p_rt:%u %u %d", (uint32_t)delta, istate->ticks_delta_p,
                    (istate->async_com_read_request - istate->async_com_read_response));

        if (!delta_rej) {
            if (istate->ticks_delta_p > 0) {
#if BMI26X_CONFIG_IGNORE_TS_JITTER
#else
                //uint32_t devi = (delta >= istate->ticks_delta_p) ? (delta - istate->ticks_delta_p) : (istate->ticks_delta_p - delta);
                uint32_t devi = BST_ABS((int32_t)delta - (int32_t)istate->ticks_delta_p);
                if ((devi << 1) <= istate->ticks_delta_p) {
                    delta_rej = false;
                    istate->ticks_delta_p = ((istate->ticks_delta_p << 3) + (delta << 1)) / 10;
                } else {
                    delta_rej = true;
                    BMI26X_INST_LOG(MED, istate->owner, "devi_p_rt:%u", devi);
                }
#endif
            } else {
                istate->ticks_delta_p = delta;
                BMI26X_INST_LOG(MED, istate->owner, "delta_p_init_rt:%u", delta);
            }
        } else {
            if (istate->ticks_delta_p > 0) {
            } else {
                istate->ticks_delta_p = delta;
                BMI26X_INST_LOG(MED, istate->owner, "delta_p_init_forced_rt:%u", delta);
                delta_rej = false;
            }
        }
#endif

        if (!delta_rej) {
#if BMI26X_CONFIG_ENABLE_WORKAROUND_4_SPI_SYNC_READ_LONG_DELAY
            uint32_t delay_read = (uint32_t)(istate->xfer_time_per_byte_ticks * 4);
            istate->ts_pair_sys_dev.ts_sys = bmi26x_hal_get_ts_forward(istate->ts_last_sbus_read_pre, delay_read);
#else
#if BMI26X_CONFIG_SEE_LITE_SLOW_CLOCK
            istate->ts_pair_sys_dev.ts_sys = istate->bus_is_spi ? te : bmi26x_hal_get_ts_backward(te, 2);
#else
            istate->ts_pair_sys_dev.ts_sys = bmi26x_hal_get_ts_backward(te, 2 * istate->xfer_time_per_byte_ticks);
#endif
#endif
            istate->ts_pair_sys_dev.ts_dev = si_buf[0] | (si_buf[1] << 8) | (si_buf[2] << 16);
        } else {
            BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE delta_p_rt:%u", delta);
        }
    }

    return SNS_RC_SUCCESS;
}

//uted
static
sns_rc bmi26x_hal_get_couple_ts_host_and_dev(
    bmi26x_instance_state   *istate)
{
    sns_rc                      rc = SNS_RC_SUCCESS;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    bool                        read_ts_dev = true;

    if (BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type) {
        if (istate->reg_int_ctx.avail.bits.ts) {
            istate->ts_pair_sys_dev.ts_sys = istate->reg_int_ctx.ts_sys;
            istate->ts_pair_sys_dev.ts_dev = istate->reg_int_ctx.ts_dev;
            read_ts_dev = false;
        } else {
            read_ts_dev = true;
        }
    } else {
        read_ts_dev = true;
    }

    if (read_ts_dev) {
        rc = bmi26x_hal_update_couple_ts_host_and_dev_rt(istate);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    //istate->ts_pair_sys_dev.avail_1st = 1;

    return rc;
}

static
void bmi26x_hal_fifo_cor_ff_len(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_read_ctx_t  *ctx,
    uint16_t                *ff_count)
{
    uint32_t                    time_sbus_xfer_est;
    uint32_t                    aug = 0;
    uint32_t                    tmp;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    uint16_t                    odr_ff_master = BMI26X_REGV_ODR_MAP[fifo_info->ff_master_odr_curr].odr;
    const uint16_t BMI26X_SPI_CLK_CNT_PER_BYTE = 8 + BMI26X_CONFIG_SEE_SPI_BYTE_XFER_WAIT_CYCLES;

    if (istate->bus_is_spi) {
        time_sbus_xfer_est = ((((*ff_count) * BMI26X_SPI_CLK_CNT_PER_BYTE) + 2) * istate->bus_cycle_time_ns) / 1000;
    } else {
        time_sbus_xfer_est = 26 * (*ff_count + 3);
    }

    time_sbus_xfer_est += (BMI26X_CONFIG_SEE_ASYNC_COM_DELAY_US);

    tmp = time_sbus_xfer_est * odr_ff_master;
    aug = (tmp / BMS_SCALE_S2US);
    if ((tmp - aug * BMS_SCALE_S2US) > (BMS_SCALE_S2US >> 1)) {
        aug ++;
    }

    if (aug > 0) {
        aug = aug * (1 +
                     ((istate->accel_info.ff_wml_req ? BMI26X_FF_DATA_LEN_ACC : 0) +
                      (istate->gyro_info.ff_wml_req ? BMI26X_FF_DATA_LEN_GYR : 0) +
                      (0 ? BMI26X_FF_DATA_LEN_MAG : 0)));    //TODOMAG

        BMI26X_INST_LOG(MED, istate->owner, "[cor_ff_count] odr: %d ff_count: %d aug: %d",
                        odr_ff_master, *ff_count, aug);
    }

    if (BMI26X_FIFO_READ_CTX_TYPE_WMI == ctx->ctx_type) {
        if (fifo_info->ff_tm_info.boost_read_size) {

            if (fifo_info->ff_master_odr_curr <= BMI26X_REGV_ODR_200HZ) {
                aug += 5 * (BMI26X_FF_DATA_LEN_ACC + BMI26X_FF_DATA_LEN_GYR + BMI26X_FF_DATA_LEN_MAG + 1);
            } else {
                aug += 10 * (BMI26X_FF_DATA_LEN_ACC + BMI26X_FF_DATA_LEN_GYR + BMI26X_FF_DATA_LEN_MAG + 1);
            }
        }
    } else {
        aug += 4 * (BMI26X_FF_DATA_LEN_ACC + BMI26X_FF_DATA_LEN_GYR + BMI26X_FF_DATA_LEN_MAG + 1);
    }

    *ff_count += aug;
}

static
sns_rc bmi26x_hal_fifo_determine_ff_len(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_read_ctx_t  *ctx,
    uint16_t                *ff_len)
{
    uint8_t                     si_buf[2] = "";
    sns_rc                      rc;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    if ((BMI26X_FIFO_READ_CTX_TYPE_WMI == ctx->ctx_type) && istate->reg_int_ctx.avail.bits.fifo_len) {
        *ff_len = istate->reg_int_ctx.fifo_len;
    } else {
        *ff_len = BMI26X_FF_DEPTH_BYTES;
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_FIFO_LENGTH_0, si_buf, 2);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        *ff_len = si_buf[0] | (si_buf[1] << 8);
    }

    if (*ff_len > 0) {
        bmi26x_hal_fifo_cor_ff_len(istate, ctx, ff_len);

        if (BMI26X_FIFO_READ_CTX_TYPE_WMI == ctx->ctx_type) {
            *ff_len += BMI26X_FF_FRAME_LEN_TS + 1;
        } else {
            *ff_len += BMI26X_FF_FRAME_LEN_TS + 1;
            if (fifo_info->ff_sensors_en_curr & BMI26X_ACCEL) {
                *ff_len += BMI26X_FF_DATA_LEN_ACC;
            }

            if (fifo_info->ff_sensors_en_curr & BMI26X_GYRO) {
                *ff_len += BMI26X_FF_DATA_LEN_GYR;
            }

            //TODOMAG
        }
    } else {
    }

    return SNS_RC_SUCCESS;
}

static
void bmi26x_hal_fifo_update_sync_info(
    bmi26x_instance_state   *istate,
    const uint8_t           fh_param,
    bmi26x_fifo_parse_ctx_t *ff_parse_ctx,
    bmi26x_fifo_parse_out_desc_t    *ff_parse_out_desc)
{
    bmi26x_fifo_info_t      *fifo_info = &istate->fifo_info;
    UNUSED_VAR(ff_parse_ctx);

    fifo_info->ff_master_sensors_ready |= (fh_param & fifo_info->ff_master_sensors_curr);

    if (fifo_info->ff_master_sensors_ready) {
        ff_parse_out_desc->fc_masters_this_batch ++;
        fifo_info->ff_tm_info.fc_accum_masters ++;

        if (fh_param & BMI26X_ACCEL) {
            ff_parse_out_desc->fc_masters_mark_acc = ff_parse_out_desc->fc_masters_this_batch;
        }

        if (fh_param & BMI26X_GYRO) {
            ff_parse_out_desc->fc_masters_mark_gyr = ff_parse_out_desc->fc_masters_this_batch;
        }
        //TODOMAG

    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE bmi26x_cp_ ff_master sync_excep");
    }

    if (fh_param == fifo_info->ff_sensors_en_curr) {
        fifo_info->ff_all_synced = 1;
    }
}


static
uint8_t bmi26x_hal_report_single_frame_acc(
    bmi26x_instance_state   *istate,
    const uint8_t           *ff_buf,
    sns_time                ts,
    bmi26x_fifo_sample_report_ctx_t *ff_sample_rpt_ctx)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    int16_t                     data_in[3];
    int32_t                     data_remapped_fxp[3] = {0};
    float                       data_remapped[3] = {0.0};
    bool                        data_remapped_needed = true;
    bool                        data_rpt_skip = false;
    uint8_t                     i;
    triaxis_conversion          *axis_map;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t rv = 0;

    data_in[TRIAXIS_X] = ((int16_t)((ff_buf[1] << 8) | ff_buf[0]));
    data_in[TRIAXIS_Y] = ((int16_t)((ff_buf[3] << 8) | ff_buf[2]));
    data_in[TRIAXIS_Z] = ((int16_t)((ff_buf[5] << 8) | ff_buf[4]));

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
    BMI26X_INST_LOG(HIGH, istate->owner, "4dbtap,acc.lsb 0x84,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",
                    ff_buf[0], ff_buf[1],
                    ff_buf[2], ff_buf[3],
                    ff_buf[4], ff_buf[5]);
#endif

#if BMI26X_CONFIG_ENABLE_SIMPLE_CAL
    data_remapped_needed = false;
#endif

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    data_remapped_needed = true;
#endif

#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    if (istate->fac_test_in_progress && BMI26X_ACCEL == istate->fac_test_info.fac_test_sensor) {
        data_remapped_needed = true;
    }
#endif

    axis_map = istate->accel_info.sstate->common.axis_map;
    for (i = 0; i < TRIAXIS_NUM; i++) {
        data_remapped_fxp[axis_map[i].opaxis] = (axis_map[i].invert ? -1 : 1) *
                                                data_in[axis_map[i].ipaxis] * istate->accel_info.sstvt_curr;

        if (data_remapped_needed) {
            data_remapped[axis_map[i].opaxis] = data_remapped_fxp[axis_map[i].opaxis] / BMI26X_SCALE_FACTOR_DATA_ACCEL;
        }
    }

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
    BMI26X_INST_LOG(MED, istate->owner, "acc frame:%d, ssvt:%d remapped:<%d %d %d>",
                    istate->fac_test_info.num_samples,
                    (int32_t) (istate->accel_info.sstvt_curr),
                    (int32_t)(data_remapped[0] * 1000),
                    (int32_t)(data_remapped[1] * 1000),
                    (int32_t)(data_remapped[2] * 1000));
#endif

#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    if (istate->fac_test_in_progress && BMI26X_ACCEL == istate->fac_test_info.fac_test_sensor) {
        istate->fac_test_info.num_samples ++;

        for (i = 0; i < TRIAXIS_NUM; i ++) {
            if (i == (TRIAXIS_NUM - 1)) {
                data_remapped[i] -= G;
            }
            istate->fac_test_info.sample_sum[i] += (data_remapped[i]);
            istate->fac_test_info.sample_square_sum[i] += (data_remapped[i] * data_remapped[i]);
        }

        data_rpt_skip = true;
#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
            BMI26X_INST_LOG(MED, istate->owner, "samples:%u, sum:<%d %d %d> squr<%u %u %u>",
                        (uint32_t)istate->fac_test_info.num_samples,
                        (int32_t)(istate->fac_test_info.sample_sum[0] * 1e6),
                        (int32_t)(istate->fac_test_info.sample_sum[1] * 1e6),
                        (int32_t)(istate->fac_test_info.sample_sum[2] * 1e6),
                        (uint32_t)(istate->fac_test_info.sample_square_sum[0] * 1e6),
                        (uint32_t)(istate->fac_test_info.sample_square_sum[1] * 1e6),
                        (uint32_t)(istate->fac_test_info.sample_square_sum[2] * 1e6));
#endif

    }
#endif

    int32_t *fac_cal_bias;
    fac_cal_bias = istate->accel_info.sstate->fac_cal_bias;
#if BMI26X_CONFIG_ENABLE_SIMPLE_CAL
    vector3 data_cal;

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
    BMI26X_INST_LOG(MED, istate->owner, "acc.remaped <%d %d %d>",
                    ((data_remapped_fxp[0] - fac_cal_bias[0])),
                    (data_remapped_fxp[1] - fac_cal_bias[1]),
                    (data_remapped_fxp[2] - fac_cal_bias[2]));
#endif

    data_cal.x = (data_remapped_fxp[0] - fac_cal_bias[0]) / BMI26X_SCALE_FACTOR_DATA_ACCEL;
    data_cal.y = (data_remapped_fxp[1] - fac_cal_bias[1]) / BMI26X_SCALE_FACTOR_DATA_ACCEL;
    data_cal.z = (data_remapped_fxp[2] - fac_cal_bias[2]) / BMI26X_SCALE_FACTOR_DATA_ACCEL;
#else
    vector3 fac_cal_bias_vec;

    fac_cal_bias_vec.x = fac_cal_bias[0] / BMI26X_SCALE_FACTOR_DATA_ACCEL;
    fac_cal_bias_vec.y = fac_cal_bias[1] / BMI26X_SCALE_FACTOR_DATA_ACCEL;
    fac_cal_bias_vec.z = fac_cal_bias[2] / BMI26X_SCALE_FACTOR_DATA_ACCEL;

    vector3 data_cal = sns_apply_calibration_correction_3(
                           make_vector3_from_array(data_remapped),
                           fac_cal_bias_vec,
                           istate->accel_info.sstate->fac_cal_corr_mat);
#endif

#if BMI26X_CONFIG_ENABLE_LOG_CURRENT_CAL_BIAS
    BMI26X_INST_LOG(LOW, istate->owner, "acc bias <%d %d %d> @%u",
                    (int)(fac_cal_bias[0]),
                    (int)(fac_cal_bias[1]),
                    (int)(fac_cal_bias[2]),
                    (uint32_t) (ts & 0xFFFFFFFF));
#endif

    if (
        //(fifo_info->publish_sensors & BMI26X_ACCEL //this is the logic of sample code
        (fifo_info->ff_sensors_en_curr & BMI26X_ACCEL)
#if BMI26X_CONFIG_ENABLE_DRI_MODE
        || (istate->int_en_flags_curr.bits.drdy.flag)
#endif
        || (istate->accel_info.gated_client_present && !istate->int_en_flags_curr.bits.md)
    ) {
        sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
        if (!data_rpt_skip) {
            rc = pb_send_sensor_stream_event(istate->owner,
                                             &istate->accel_info.sstate->my_suid,
                                             ts - istate->accel_info.filter_delay_in_tick,
                                             SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                             status,
                                             data_cal.data,
                                             ARR_SIZE(data_cal.data),
                                             istate->encoded_imu_event_len);
            rv = rc;
        }

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
        // Log raw uncalibrated sensor data_remapped
        if (ff_sample_rpt_ctx != NULL) {
            rc = bmi26x_log_sensor_state_raw_add(
                 ff_sample_rpt_ctx->log_state_info_acc,
                 data_remapped,
                 ts,
                 status);
            rv = rv | rc << 4;
        }
#else
        UNUSED_VAR(ff_sample_rpt_ctx);
        UNUSED_VAR(data_remapped);
#endif
    }

    return rv;
}



static
void bmi26x_hal_fifo_handle_frame_acc(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_ctx_t *ff_parse_ctx,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc,
    uint32_t                fc,
    const uint8_t           *ff_buf)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    bmi26x_fifo_time_info_t     *ff_tm_info = &fifo_info->ff_tm_info;
    sns_time                    ts;
    uint8_t rv = 0;

    bmi26x_fifo_sample_report_ctx_t *ff_sample_rpt_ctx =
        (bmi26x_fifo_sample_report_ctx_t *)ff_parse_ctx->priv_data;

    ff_tm_info->fc_accum_curr_acc ++;

    if (ff_parse_out_desc->fc_this_batch_acc > 1) {
        ts = bmi26x_hal_get_ts_forward(ff_tm_info->ts_1st_frame_this_batch_acc,
                                       BST_CEIL_P((ff_tm_info->itvl_this_batch_acc * (fc - 1))));
    } else {
        ts = ff_tm_info->ts_1st_frame_this_batch_acc;
    }

    rv = bmi26x_hal_report_single_frame_acc(istate, ff_buf, ts, ff_sample_rpt_ctx);

#if BMI26X_CONFIG_ENABLE_LOG_TS_INDIVIDUAL_FRAME
    BMI26X_INST_LOG(MED, istate->owner, "dl_meta_acc:<0x%x,0x%x,%u,%u> rv:%d",
                    (fifo_info->ff_flush_trigger << 27) | (fifo_info->ff_read_ctx_type << 24) | (g_bmi26x_cnt_session_acc << 16) |
                    (ff_parse_out_desc->fc_this_batch_acc << 8) | fc,
                    BMI26X_SYS_TIME_HH(ts),
                    BMI26X_SYS_TIME_LH(ts),
                    ff_tm_info->fc_accum_curr_acc,
                    rv);
#endif
}

static
void bmi26x_hal_report_single_frame_gyr(
    bmi26x_instance_state   *istate,
    const uint8_t           *ff_buf,
    sns_time                ts,
    bmi26x_fifo_sample_report_ctx_t *ff_sample_rpt_ctx)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
    bmi26x_gyro_info_t          *gyro_info = &istate->gyro_info;
#endif

    int16_t                   data_in[3];
    int32_t                   data_remapped_fxp[3] = {0};
    float                     data_remapped[3] = {0.0};
    bool                      data_remapped_needed = true;
    bool                      data_rpt_skip = false;
    uint8_t                   i;
    triaxis_conversion        *axis_map;

    //OPTIM
    data_in[TRIAXIS_X] = ((int16_t)((ff_buf[1] << 8) | ff_buf[0]));
    data_in[TRIAXIS_Y] = ((int16_t)((ff_buf[3] << 8) | ff_buf[2]));
    data_in[TRIAXIS_Z] = ((int16_t)((ff_buf[5] << 8) | ff_buf[4]));

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
    BMI26X_INST_LOG(MED, istate->owner, "gyr.lsb <%d %d %d>",
                    data_in[TRIAXIS_X], data_in[TRIAXIS_Y],
                    data_in[TRIAXIS_Z]);
#endif

    // <cross Sensitivity>
    {
        // handle the cross sensitivity
        int32_t lsb_data_x = data_in[TRIAXIS_X];
        int32_t lsb_data_z = data_in[TRIAXIS_Z];

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
        BMI26X_INST_LOG(MED, istate->owner, "b.lsb<%d %d %d>", data_in[TRIAXIS_X],
                        data_in[TRIAXIS_Y], data_in[TRIAXIS_Z]);
#endif

        data_in[TRIAXIS_X] = (int16_t)(lsb_data_x - (int32_t)((istate->gyro_info.signed_gyr_cross_sense_coef * lsb_data_z) /
                                       512));

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
        BMI26X_INST_LOG(MED, istate->owner, "a.lsb<%d %d %d>",
                        data_in[TRIAXIS_X], data_in[TRIAXIS_Y],
                        data_in[TRIAXIS_Z]);
#endif
    }
    // </cross Sensitivity>

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
    bool                        sample_sel = true;

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW_FILT
    if (gyro_info->downsample_sw_factor <= 1) {
    } else {
        if (0 != (gyro_info->downsample_sw_cnt % gyro_info->downsample_sw_factor)) {
            data_in[TRIAXIS_X] = (data_in[TRIAXIS_X] + gyro_info->downsample_sw_dcache[TRIAXIS_X]);
            data_in[TRIAXIS_Y] = (data_in[TRIAXIS_Y] + gyro_info->downsample_sw_dcache[TRIAXIS_Y]);
            data_in[TRIAXIS_Z] = (data_in[TRIAXIS_Z] + gyro_info->downsample_sw_dcache[TRIAXIS_Z]);

            data_in[TRIAXIS_X] = (data_in[TRIAXIS_X] + (data_in[TRIAXIS_X] > 0 ? 1 : -1)) >> 1;
            data_in[TRIAXIS_Y] = (data_in[TRIAXIS_Y] + (data_in[TRIAXIS_Y] > 0 ? 1 : -1)) >> 1;
            data_in[TRIAXIS_Z] = (data_in[TRIAXIS_Z] + (data_in[TRIAXIS_Z] > 0 ? 1 : -1)) >> 1;
        }

        gyro_info->downsample_sw_dcache[TRIAXIS_X] = data_in[TRIAXIS_X];
        gyro_info->downsample_sw_dcache[TRIAXIS_Y] = data_in[TRIAXIS_Y];
        gyro_info->downsample_sw_dcache[TRIAXIS_Z] = data_in[TRIAXIS_Z];
    }
#endif


    if (gyro_info->downsample_sw_factor <= 1) {
    } else {
        gyro_info->downsample_sw_cnt ++;
        if (0 != (gyro_info->downsample_sw_cnt % gyro_info->downsample_sw_factor)) {
            sample_sel = false;
        } else {
            gyro_info->downsample_sw_cnt = 0;
        }
    }

    if (sample_sel) {
    } else {
        return;
    }
#endif


#if BMI26X_CONFIG_ENABLE_SIMPLE_CAL
    data_remapped_needed = false;
#endif

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    data_remapped_needed = true;
#endif

#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    if (istate->fac_test_in_progress && BMI26X_GYRO == istate->fac_test_info.fac_test_sensor) {
        data_remapped_needed = true;
    }
#endif

    axis_map = istate->gyro_info.sstate->common.axis_map;
    for (i = 0; i < TRIAXIS_NUM; i ++) {
        data_remapped_fxp[axis_map[i].opaxis] = (axis_map[i].invert ? -1 : 1) *
                                                data_in[axis_map[i].ipaxis] * istate->gyro_info.sstvt_curr;

        if (data_remapped_needed) {
            data_remapped[axis_map[i].opaxis] = data_remapped_fxp[axis_map[i].opaxis] / BMI26X_SCALE_FACTOR_DATA_GYRO;
        }
    }

#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    if (istate->fac_test_in_progress && BMI26X_GYRO == istate->fac_test_info.fac_test_sensor) {
        istate->fac_test_info.num_samples ++;
        //data_remapped_needed = 1;

        for (i = 0; i < TRIAXIS_NUM; i ++) {
            istate->fac_test_info.sample_sum[i] += (data_remapped[i]);
            istate->fac_test_info.sample_square_sum[i] += (data_remapped[i] * data_remapped[i]);
        }

        data_rpt_skip = true;
#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
        BMI26X_INST_LOG(MED, istate->owner, "fac.samples:%d, x:%d/%d, y:%d/%d, z:%d/%d, ssvt: %d",
                         istate->fac_test_info.num_samples,
                         (int32_t)(istate->fac_test_info.sample_sum[0] * 1e6),
                         (uint32_t)(istate->fac_test_info.sample_square_sum[0] * 1e6),
                         (int32_t)(istate->fac_test_info.sample_sum[1] * 1e6),
                         (uint32_t)(istate->fac_test_info.sample_square_sum[1] * 1e6),
                         (int32_t)(istate->fac_test_info.sample_sum[2] * 1e6),
                         (uint32_t)(istate->fac_test_info.sample_square_sum[2] * 1e6),
                         (uint32_t)(istate->gyro_info.sstvt_curr));
#endif
    }
#endif

    int32_t *fac_cal_bias;
    fac_cal_bias = istate->gyro_info.sstate->fac_cal_bias;
#if BMI26X_CONFIG_ENABLE_SIMPLE_CAL
    vector3 data_cal;

#if BMI26X_CONFIG_ENABLE_SENSOR_DATA_LOG
    BMI26X_INST_LOG(MED, istate->owner, "gyr.remaped<%d %d %d>",
                    (data_remapped_fxp[0] - fac_cal_bias[0]),
                    (data_remapped_fxp[1] - fac_cal_bias[1]),
                    (data_remapped_fxp[2] - fac_cal_bias[2]));
#endif

    data_cal.x = (data_remapped_fxp[0] - fac_cal_bias[0]) / BMI26X_SCALE_FACTOR_DATA_GYRO;
    data_cal.y = (data_remapped_fxp[1] - fac_cal_bias[1]) / BMI26X_SCALE_FACTOR_DATA_GYRO;
    data_cal.z = (data_remapped_fxp[2] - fac_cal_bias[2]) / BMI26X_SCALE_FACTOR_DATA_GYRO;
#else
    vector3 fac_cal_bias_vec;

    fac_cal_bias_vec.x = fac_cal_bias[0] / BMI26X_SCALE_FACTOR_DATA_GYRO;
    fac_cal_bias_vec.y = fac_cal_bias[1] / BMI26X_SCALE_FACTOR_DATA_GYRO;
    fac_cal_bias_vec.z = fac_cal_bias[2] / BMI26X_SCALE_FACTOR_DATA_GYRO;

    vector3 data_cal = sns_apply_calibration_correction_3(
                           make_vector3_from_array(data_remapped),
                           fac_cal_bias_vec,
                           istate->gyro_info.sstate->fac_cal_corr_mat);
#endif

    //if (fifo_info->publish_sensors & BMI26X_GYRO)     //the logic from sample code
    if (
        (fifo_info->ff_sensors_en_curr & BMI26X_GYRO)
#if BMI26X_CONFIG_ENABLE_DRI_MODE
        || (istate->int_en_flags_curr.bits.drdy.flag)
#endif
    ) {
        sns_rc rc = SNS_RC_SUCCESS;
        sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

        if (!data_rpt_skip) {
            rc = pb_send_sensor_stream_event(istate->owner,
                                             &istate->gyro_info.sstate->my_suid,
                                             ts - istate->gyro_info.filter_delay_in_tick,
                                             SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                             status,
                                             data_cal.data,
                                             ARR_SIZE(data_cal.data),
                                             istate->encoded_imu_event_len);
            if (rc == SNS_RC_SUCCESS) {
                BMI26X_INST_LOG(LOW, istate->owner, "gyr num:%u, gd:%u /ticks",
                                istate->fifo_info.ff_tm_info.fc_accum_curr_gyr,
                                istate->gyro_info.filter_delay_in_tick);
            }
        }
#if BMI26X_CONFIG_ENABLE_DIAG_LOG
        // Log raw uncalibrated sensor data_remapped
        if (NULL != ff_sample_rpt_ctx) {
            if (bmi26x_log_sensor_state_raw_add(
                    ff_sample_rpt_ctx->log_state_info_gyr,
                    data_remapped,
                    ts,
                    status) != SNS_RC_SUCCESS) {
            }
        }
#else
        UNUSED_VAR(ff_sample_rpt_ctx);
#endif
    }
}

static
void bmi26x_hal_fifo_handle_frame_gyr(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_ctx_t *ff_parse_ctx,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc,
    uint32_t                fc,
    const uint8_t           *ff_buf)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    bmi26x_fifo_time_info_t     *ff_tm_info = &fifo_info->ff_tm_info;
    sns_time                    ts;
    bmi26x_fifo_sample_report_ctx_t *ff_sample_rpt_ctx = (bmi26x_fifo_sample_report_ctx_t *)ff_parse_ctx->priv_data;

    ff_tm_info->fc_accum_curr_gyr ++;

    if (ff_parse_out_desc->fc_this_batch_gyr > 1) {
        ts = bmi26x_hal_get_ts_forward(ff_tm_info->ts_1st_frame_this_batch_gyr,
                                       BST_CEIL_P((ff_tm_info->itvl_this_batch_gyr * (fc - 1))));
    } else {
        ts = ff_tm_info->ts_1st_frame_this_batch_gyr;
    }

    bmi26x_hal_report_single_frame_gyr(istate, ff_buf, ts, ff_sample_rpt_ctx);

#if BMI26X_CONFIG_ENABLE_LOG_TS_INDIVIDUAL_FRAME
    BMI26X_INST_LOG(LOW, istate->owner, "dl_meta_gyr:<0x%x,0x%x,%u,%u>",
                    (uint32_t)((fifo_info->ff_flush_trigger << 27) | (fifo_info->ff_read_ctx_type << 24) |
                               (g_bmi26x_cnt_session_gyr << 16) |
                               (ff_parse_out_desc->fc_this_batch_gyr << 8) | fc),
                    BMI26X_SYS_TIME_HH(ts), BMI26X_SYS_TIME_LH(ts), ff_tm_info->fc_accum_curr_gyr);
#endif
}


// TODOMAG
static
void bmi26x_hal_fifo_handle_frame_mag(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_ctx_t *ff_parse_ctx,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc,
    uint32_t                fc,
    const uint8_t           *ff_buf)
{
    UNUSED_VAR(istate);
    UNUSED_VAR(ff_buf);
    UNUSED_VAR(ff_parse_ctx);
    UNUSED_VAR(ff_parse_out_desc);
    UNUSED_VAR(fc);
}


//uted
static inline
uint32_t bmi26x_dev_get_time_elapse_dev_lsb(
    uint32_t                before,
    uint32_t                after)
{
    if (after >= before) {
        return (after - before);
    } else {
        return (BMI26X_SPEC_TS_LSB_OVERFLOW_VAL - before + after);
    }
}

static inline
uint32_t bmi26x_dev_get_ts_dev_age(
    uint32_t                ts_dev,
    uint8_t                 ts_dev_lsb_bits)
{
    uint32_t                    age;

    age = (ts_dev & ((1 << ts_dev_lsb_bits) - 1));

    return age;
}

uint32_t bmi26x_dev_get_ts_dev_update_msb(
    uint32_t                start,
    uint32_t                end,
    uint8_t                 ts_dev_lsb_bits)
{
    uint32_t                    mask = ~((uint32_t)((1 << ts_dev_lsb_bits) - 1));
    uint32_t                    ret = 0;

    start &= mask;
    end &= mask;

    if (end >= start) {
        ret = (end - start) >> ts_dev_lsb_bits;
    } else {
        ret = (BMI26X_SPEC_TS_LSB_OVERFLOW_VAL - start + end) >> ts_dev_lsb_bits;
    }

    return ret;
}

static
void bmi26x_hal_fifo_update_ts_hw_res(
    bmi26x_instance_state   *istate,
    sns_time                *ts_curr)
{
    float                       ts_hw_res_ticks_per_bit;
    float                       devi;
    float                       diff_coef;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
    uint8_t                     ts_dev_lsb_bits = BMI26X_REGV_ODR_1600HZ - fifo_info->ff_master_odr_curr + 4;
#endif

    uint32_t                    itvl;
    sns_rc                      rc;

    bmi26x_sys_dev_time_pair_t  *ts_pair_sys_dev_old = &istate->ts_pair_sys_dev_4_ts_res_est;

    itvl = bmi26x_get_time_elapse_sys(istate->ts_pair_sys_dev.ts_sys, *ts_curr);


    if (itvl >= (30 * istate->ticks_in_1ms)) {
    } else {
#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
        if (istate->cnt_hw_res_est > 1)
#else
        if (istate->cnt_hw_res_est > 0)
#endif
        {
            BMI26X_INST_LOG(MED, istate->owner, "bmi26x_cp_ short itvl: %u %u",
                            BMI26X_SYS_TIME_LH(istate->ts_pair_sys_dev.ts_sys),
                            BMI26X_SYS_TIME_LH(*ts_curr));
            return;
        } else {
        }
    }

    rc = bmi26x_hal_get_couple_ts_host_and_dev(istate);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! get h.ts and dev.ts error:%d",
                        rc);
    }

    itvl = bmi26x_get_time_elapse_sys(istate->ts_pair_sys_dev_4_ts_res_est.ts_sys, *ts_curr);
    if (itvl < (BMI26X_TS_ALGO_UPDATE_FREQUENCY_OVER_TIME * istate->ticks_in_1ms)) {
        if (istate->cnt_hw_res_est) {
            BMI26X_INST_LOG(MED, istate->owner, "bmi26x_cp_ short itvl: %u %u",
                            BMI26X_SYS_TIME_LH(istate->ts_pair_sys_dev.ts_sys),
                            BMI26X_SYS_TIME_LH(*ts_curr));
            return;
        }
    }

    ts_hw_res_ticks_per_bit = bmi26x_get_time_elapse_sys(ts_pair_sys_dev_old->ts_sys,
                              istate->ts_pair_sys_dev.ts_sys) * 1.0f /
                              bmi26x_dev_get_time_elapse_dev_lsb(ts_pair_sys_dev_old->ts_dev, istate->ts_pair_sys_dev.ts_dev);

    if ((BMI26X_CONFIG_ACC_LOWEST_ODR <= 12.5) ||
            (BMI26X_CONFIG_ACC_LOWEST_ODR <= 12.5)) {
        diff_coef = 69.0;
    } else {
        diff_coef = 75.0;
    }

    devi = BST_ABS(ts_hw_res_ticks_per_bit - istate->ts_hw_res_ticks_per_bit_ideal);
    if (BST_ABS(devi) * diff_coef <= istate->ts_hw_res_ticks_per_bit_ideal) {  //75->69 when 12.5hz is used
        if (istate->cnt_hw_res_est > BMI26X_CNT_HW_RES_START_EST_4_ALGO) {
            /*istate->ts_hw_res_ticks_per_bit = istate->ts_hw_res_ticks_per_bit * BMI26X_CONFIG_TS_ITVL_EST_COEF
                                              + (1.0f - BMI26X_CONFIG_TS_ITVL_EST_COEF) * ts_hw_res_ticks_per_bit;*/

            istate->ts_hw_res_ticks_per_bit = istate->ts_hw_res_ticks_per_bit * BMI26X_CONFIG_TS_ITVL_EST_COEF +
                            (1.0f - BMI26X_CONFIG_TS_ITVL_EST_COEF) * ts_hw_res_ticks_per_bit;
        } else {
            if (0 == istate->cnt_hw_res_est) {
                istate->ts_hw_res_ticks_per_bit = ts_hw_res_ticks_per_bit;
            } else {
                istate->ts_hw_res_ticks_per_bit =
                    istate->ts_hw_res_ticks_per_bit * (1.0f - BMI26X_CONFIG_TS_ITVL_EST_1ST_COEF) +
                        (BMI26X_CONFIG_TS_ITVL_EST_1ST_COEF) * ts_hw_res_ticks_per_bit;
            }
        }

        if (istate->cnt_hw_res_est < 100) {
            istate->cnt_hw_res_est ++;
        }

#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
        fifo_info->ff_itvl_ticks_est_masters = istate->ts_hw_res_ticks_per_bit * (1 << ts_dev_lsb_bits);
#endif
        fifo_info->ff_itvl_ticks_est_acc = istate->ts_hw_res_ticks_per_bit * (1 << (BMI26X_REGV_ODR_1600HZ -
                                           istate->accel_info.odr_curr + 4));
        if (istate->gyro_info.odr_curr == istate->accel_info.odr_curr) {
            fifo_info->ff_itvl_ticks_est_gyr = fifo_info->ff_itvl_ticks_est_acc;
        } else {
            fifo_info->ff_itvl_ticks_est_gyr = istate->ts_hw_res_ticks_per_bit * (1 << (BMI26X_REGV_ODR_1600HZ -
                                               istate->gyro_info.odr_curr + 4));
        }

        BMI26X_INST_LOG(HIGH, istate->owner, "ts_hw_res: <%d %d %d %d %d>",
                        (int)((istate->ts_hw_res_ticks_per_bit * 1000)),
                        (int)(ts_hw_res_ticks_per_bit * 1000),
                        (int)BST_CEIL_P(fifo_info->ff_itvl_ticks_est_masters),
                        (int)BST_CEIL_P(fifo_info->ff_itvl_ticks_est_acc),
                        (int)BST_CEIL_P(fifo_info->ff_itvl_ticks_est_gyr));

        istate->ts_pair_sys_dev_4_ts_res_est = istate->ts_pair_sys_dev;
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! ts_hw_res ignored: %d(1000) %d(1000), diff:%d",
                (int)(ts_hw_res_ticks_per_bit * 1000),
                (int)(istate->ts_hw_res_ticks_per_bit_ideal * 1000),
                (uint32_t)(devi * 1000));
    }
}

static
void bmi26x_hal_cor_ts_dev(
    bmi26x_instance_state   *istate,
    uint32_t                *ts_dev)
{
    uint32_t                    ts_dev_cor = *ts_dev;
    uint8_t                     ts_dev_update_msb;
    uint8_t                     ts_dev_lsb_bits;

    bmi26x_fifo_time_info_t *ff_tm_info = &istate->fifo_info.ff_tm_info;

    //ts_dev_cor -= BMI26X_CONFIG_TS_DELAY_ADJ_I2C;
    ts_dev_cor = (ts_dev_cor - 1) & (BMI26X_SPEC_TS_LSB_OVERFLOW_VAL - 1);

    ts_dev_lsb_bits = BMI26X_REGV_ODR_1600HZ - istate->fifo_info.ff_master_odr_curr + 4;
    ts_dev_update_msb = bmi26x_dev_get_ts_dev_update_msb(
                            ts_dev_cor,  *ts_dev, ts_dev_lsb_bits);
    if (!ts_dev_update_msb) {
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ ts_dev_update_msb: %d 0x%x", ts_dev_update_msb, *ts_dev);

        if (1 == ts_dev_update_msb) {
            uint8_t tmp;
            tmp = bmi26x_dev_get_ts_dev_update_msb(
                      ff_tm_info->ts_dev_ff_last, ts_dev_cor, ts_dev_lsb_bits);
            //if (tmp == (ff_tm_info->fc_accum_masters - ff_tm_info->fc_masters_last_dev_ff))
            if ((tmp + 1) != (ff_tm_info->fc_accum_masters - ff_tm_info->fc_masters_last_dev_ff)) {
                *ts_dev = ts_dev_cor;
            } else {
                BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ ts_dev_cor rej: %d,%d",
                                ff_tm_info->fc_accum_masters, ff_tm_info->fc_masters_last_dev_ff);
            }
        }
    }

    ff_tm_info->ts_dev_ff_last = *ts_dev;
    ff_tm_info->fc_masters_last_dev_ff = ff_tm_info->fc_accum_masters;
}


static
void bmi26x_hal_fifo_est_ts_dev_ff(
    bmi26x_instance_state           *istate,
    bmi26x_fifo_parse_out_desc_t    *ff_parse_out_desc)
{
    sns_rc                              rc;
    uint8_t                             buf[3] = "";
    uint32_t                            ts_dev = 0;
    uint8_t                             ts_dev_lsb_bits;
    bmi26x_fifo_info_t                  *fifo_info = &istate->fifo_info;

    bmi26x_fifo_parse_result_t          *ff_parse_rslt = &ff_parse_out_desc->ff_parse_rslt;
    ts_dev_lsb_bits = BMI26X_REGV_ODR_1600HZ - fifo_info->ff_master_odr_curr + 4;

    // FIXME
    if ((BMI26X_FIFO_READ_CTX_TYPE_DAE != istate->fifo_info.ff_read_ctx_type) || 1) {
        rc = bmi26x_sbus_read_wrapper(istate,
                                      BMI26X_REGA_USR_SENSORTIME_0,
                                      buf, 3);
        if (SNS_RC_SUCCESS == rc) {
        } else {
            return;
        }

        bmi26x_dev_parse_data_ts(buf, &ts_dev);
    } else {
#if BMI26X_CONFIG_ENABLE_DAE
        ts_dev = istate->dae_if.ts_dae_read_fifo_dev_time;
#endif
    }

    if (ff_parse_rslt->bits.ff_avail_ts_header) {
        uint16_t    ff_count = 0xffff;
        rc = bmi26x_dev_get_reg_fifo_count(istate, &ff_count);
        if ((SNS_RC_SUCCESS == rc) && (0 == ff_count)) {
            ff_parse_out_desc->ts_dev_ff = ts_dev;
            ff_parse_rslt->bits.ff_avail_ts = true;
            BMI26X_INST_LOG(HIGH, istate->owner, "!!! bmi26x_cp_ good luck 1: ts_dev_ff est: 0x%x", ts_dev);
        }
    } else {
#if 0
        //this is not reliable
        if (BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type) {
            uint32_t   ts_dev_update_msb;
            ts_dev_update_msb = bmi26x_dev_get_ts_dev_update_msb(
                                    istate->reg_int_ctx.ts_dev,
                                    ts_dev,
                                    ts_dev_lsb_bits);

            if (0 == ts_dev_update_msb) {
                ff_parse_out_desc->ts_dev_ff =  ts_dev;
                ff_parse_rslt->bits.ff_avail_ts = true;
                BMI26X_INST_LOG(HIGH, istate->owner, "!!! bmi26x_cp_ good luck 2 (na): ts_dev_ff est: 0x%x", ts_dev);
            }
        }
#else
        UNUSED_VAR(ts_dev_lsb_bits);
#endif
    }

    if (ff_parse_rslt->bits.ff_avail_ts) {
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE ts_dev_est not blessed");
    }
}

#if 0
static bmi26x_hal_fifo_validate_ts_this_batch(bmi26x_instance_state   *istate,
                                              bmi26x_sensor_type ss_type)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    bmi26x_fifo_time_info_t     *ff_tm_info = &fifo_info->ff_tm_info;
    sns_time                    ts_end;

    sns_time ts_end_est = ff_tm_info->ts_end_frame_this_batch_gyr;
    ts_end = ff_tm_info->ts_end_frame_this_batch_gyr;

    if ((istate->cnt_hw_res_est > 1) ||
                    (ff_tm_info->fc_accum_curr_gyr > 64)) {
        ts_end_est = bmi26x_hal_get_ts_forward(ff_tm_info->ts_end_frame_last_batch_gyr,
                                           fifo_info->ff_itvl_ticks_est_gyr * (fc_this_batch));
        if (ts_end_est < ts_end) {
            if ((BST_ABS(ts_end_est - ts_end) << 5) < fifo_info->ff_itvl_ticks_est_gyr) {
                // reliable ts_end
                ts_end_est = ts_end;
            }
        } else {
            ts_end_est = ts_end;
        }
        ts_end = ts_end_est;
    }
    ff_tm_info->ts_end_frame_this_batch_gyr = ts_end;
}
#endif

static
void bmi26x_hal_fifo_calc_ts_start_end_this_batch(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    bmi26x_fifo_time_info_t     *ff_tm_info = &fifo_info->ff_tm_info;
    uint32_t                    itvl_acc;
    uint32_t                    itvl_gyr;
    //TODOMAG

    uint32_t                    fc_this_batch;
    int32_t                     delta_fc;
    sns_time                    ts_start;
    sns_time                    ts_end;
    sns_time                    ts_curr;

    ts_curr = bmi26x_get_sys_tick();
    ts_start = 0;

    //acc
    fc_this_batch = ff_parse_out_desc->fc_this_batch_acc;

    if (fc_this_batch) {
        itvl_acc = BST_CEIL_P(fifo_info->ff_itvl_ticks_est_acc);

        if (fc_this_batch > 1) {
            if (ff_tm_info->avail_ts_last_batch & BMI26X_ACCEL) {
                ts_start = bmi26x_hal_get_ts_forward(ff_tm_info->ts_end_frame_last_batch_acc, itvl_acc);
            } else {
                if (ff_tm_info->avail_ts_this_batch & BMI26X_ACCEL) {
                    ts_start = bmi26x_hal_get_ts_backward(ff_tm_info->ts_end_frame_this_batch_acc,
                                                          fifo_info->ff_itvl_ticks_est_acc * (fc_this_batch - 1));
                } else {
                    sns_time ts_this_batch_itvl = (sns_time)((sns_time)itvl_acc * (sns_time)(fc_this_batch - 1));
                    ts_start = bmi26x_hal_get_ts_backward(ts_curr, ts_this_batch_itvl);
                    BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! bmi26x_cp_ NOTICE: acc ts_start no ref: %u",
                                    BMI26X_SYS_TIME_LH(ts_curr));
                }
            }
        }

        if (ff_tm_info->avail_ts_this_batch & BMI26X_ACCEL) {
            //end is already available
            ts_end = ff_tm_info->ts_end_frame_this_batch_acc;
        } else {
            if (ff_tm_info->avail_ts_last_batch & BMI26X_ACCEL) {
                ts_end = bmi26x_hal_get_ts_forward(ff_tm_info->ts_end_frame_last_batch_acc,
                                                   fifo_info->ff_itvl_ticks_est_acc * (fc_this_batch));
                if (BMI26X_TIME_LT(ts_end, ts_curr)) {
                    ts_end = bmi26x_hal_get_ts_backward(ts_curr, 1);
                    BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! bmi26x_cp_ NOTICE: acc ts_end lt curr");
                }
            } else {
                ts_end = bmi26x_hal_get_ts_backward(ts_curr, 1);
                BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! bmi26x_cp_ NOTICE: acc ts_end no ref: %u",
                        BMI26X_SYS_TIME_LH(ts_curr));
            }

            ff_tm_info->ts_end_frame_this_batch_acc = ts_end;
        }

        if (1 == fc_this_batch) {
            ts_start = ts_end;
        }

        ff_tm_info->ts_1st_frame_this_batch_acc = ts_start;


#if BMI26X_CONFIG_ENABLE_OPTIM_LOAD_FP
        ff_tm_info->itvl_this_batch_acc = fifo_info->ff_itvl_ticks_est_acc;
        UNUSED_VAR(delta_fc);
#else
        delta_fc = fc_this_batch - 1;
        if (delta_fc > 0) {
            if (ts_start < ts_end) {
                ff_tm_info->itvl_this_batch_acc = bmi26x_get_time_elapse_sys(ts_start, ts_end) * 1.0f / (delta_fc);
            } else {
                BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! ts est error,  ts.end:%u < ts_start:%u",
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_acc),
                        BMI26X_SYS_TIME_LH(ts_start));

                ff_tm_info->itvl_this_batch_acc = fifo_info->ff_itvl_ticks_est_acc;
                ff_tm_info->ts_1st_frame_this_batch_acc = ts_start;
                ff_tm_info->ts_end_frame_this_batch_acc = ts_start + delta_fc * ff_tm_info->itvl_this_batch_acc;
                ts_end = ff_tm_info->ts_end_frame_this_batch_acc;
            }
        } else {
            ff_tm_info->itvl_this_batch_acc = fifo_info->ff_itvl_ticks_est_acc;
        }
#endif

#if BMI26X_CONFIG_ENABLE_LOG_TS_BATCH

        BMI26X_INST_LOG(MED, istate->owner, "dl_meta<:0x%x,%u,%u,%u,%u,%u,%u,%u,%u:>",
                        (BMI26X_ACCEL << 24) | (ff_tm_info->avail_ts_last_batch << 20) | (fifo_info->ff_flush_trigger << 19) |
                        (fifo_info->ff_read_ctx_type << 16) | (g_bmi26x_cnt_session_acc << 8),
                        (uint32_t)fc_this_batch,
                        (uint32_t)ff_tm_info->fc_accum_curr_acc,
                        (uint32_t) (ff_tm_info->itvl_this_batch_acc),
                        (uint32_t)(fifo_info->ff_itvl_ticks_est_acc),
                        BMI26X_SYS_TIME_LH(ts_start),
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_acc),
                        BMI26X_SYS_TIME_LH(istate->ts_irq),
                        BMI26X_SYS_TIME_LH(ts_curr));
#endif
    }


    // gyro
    fc_this_batch = ff_parse_out_desc->fc_this_batch_gyr;

    if (fc_this_batch) {
        itvl_gyr = BST_CEIL_P(fifo_info->ff_itvl_ticks_est_gyr);

        if (fc_this_batch > 1) {
            if (ff_tm_info->avail_ts_last_batch & BMI26X_GYRO) {
                sns_time ts_itvl_gyr = itvl_gyr;
                ts_start = bmi26x_hal_get_ts_forward(ff_tm_info->ts_end_frame_last_batch_gyr, ts_itvl_gyr);
            } else {
                if (ff_tm_info->avail_ts_this_batch & BMI26X_GYRO) {
                    sns_time ts_this_batch_gyr = (sns_time) ((sns_time)fifo_info->ff_itvl_ticks_est_gyr * (sns_time)(fc_this_batch - 1));
                    ts_start = bmi26x_hal_get_ts_backward(ff_tm_info->ts_end_frame_this_batch_gyr, ts_this_batch_gyr);
                } else {
                    sns_time ts_this_batch_gyr = (sns_time) ((sns_time)itvl_gyr * (sns_time)(fc_this_batch - 1));
                    ts_start = bmi26x_hal_get_ts_backward(ts_curr, ts_this_batch_gyr);
                    BMI26X_INST_LOG(HIGH, istate->owner,
                                    "WARN!!! bmi26x_cp_ NOTICE: gyr ts_start no ref: %u", BMI26X_SYS_TIME_LH(ts_curr));
                }
            }
        }

        if (ff_tm_info->avail_ts_this_batch & BMI26X_GYRO) {
            //end is already available
            ts_end = ff_tm_info->ts_end_frame_this_batch_gyr;
        } else {
            if (ff_tm_info->avail_ts_last_batch & BMI26X_GYRO) {
                ts_end = bmi26x_hal_get_ts_forward(ff_tm_info->ts_end_frame_last_batch_gyr,
                                                   fifo_info->ff_itvl_ticks_est_gyr * (fc_this_batch));
                if (BMI26X_TIME_LT(ts_end, ts_curr)) {
                    ts_end = bmi26x_hal_get_ts_backward(ts_curr, 1);
                    BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ NOTICE: gyr ts_end lt curr");
                }
            } else {
                ts_end = bmi26x_hal_get_ts_backward(ts_curr, 1);
                BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ NOTICE: gyr ts_end no ref");
            }

            ff_tm_info->ts_end_frame_this_batch_gyr = ts_end;
        }

        if (1 == fc_this_batch) {
            ts_start = ts_end;
        }

        ff_tm_info->ts_1st_frame_this_batch_gyr = ts_start;

#if BMI26X_CONFIG_ENABLE_OPTIM_LOAD_FP
        ff_tm_info->itvl_this_batch_gyr = fifo_info->ff_itvl_ticks_est_gyr;
#else
        delta_fc = fc_this_batch - 1;
        if (delta_fc > 0) {
            if (ts_start < ts_end) {
                ff_tm_info->itvl_this_batch_gyr = bmi26x_get_time_elapse_sys(ts_start, ts_end) * 1.0f / (delta_fc);
            } else {
                BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! ts est error,  ts.end:%u < ts_start:%u",
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_gyr),
                        BMI26X_SYS_TIME_LH(ts_start));

                ff_tm_info->itvl_this_batch_gyr = fifo_info->ff_itvl_ticks_est_gyr;
                ff_tm_info->ts_1st_frame_this_batch_gyr = ts_start;
                ff_tm_info->ts_end_frame_this_batch_gyr = ts_start + delta_fc * ff_tm_info->itvl_this_batch_gyr;
                ts_end = ff_tm_info->ts_end_frame_this_batch_gyr;
            }
        } else {
            ff_tm_info->itvl_this_batch_gyr = fifo_info->ff_itvl_ticks_est_gyr;
        }
#endif

#if BMI26X_CONFIG_ENABLE_LOG_TS_BATCH

        UNUSED_VAR(ts_end);

        BMI26X_INST_LOG(MED, istate->owner, "dl_meta<:0x%x,%u,%u,%u,%u,%u,%u,%u,%u:>",
                        (BMI26X_GYRO << 24) | (ff_tm_info->avail_ts_last_batch << 20) |
                        (fifo_info->ff_flush_trigger << 19) | (fifo_info->ff_read_ctx_type << 16) |
                        (g_bmi26x_cnt_session_gyr << 8),
                        (uint32_t)fc_this_batch,
                        (uint32_t)ff_tm_info->fc_accum_curr_gyr,
                        (uint32_t)(ff_tm_info->itvl_this_batch_gyr),
                        (uint32_t)(fifo_info->ff_itvl_ticks_est_gyr),
                        BMI26X_SYS_TIME_LH(ts_start),
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_gyr),
                        BMI26X_SYS_TIME_LH(istate->ts_irq),
                        BMI26X_SYS_TIME_LH(ts_curr));
#endif
    }
}

#if BMI26X_CONFIG_ENABLE_PROFILING_SYS_DELAY
static
void bmi26x_hal_fifo_get_est_ts_fifo_read_start_est(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc,
    sns_time                ts_ref_dev_ff,
    sns_time                *ts_est_fifo_read_start)
{
    *ts_est_fifo_read_start = bmi26x_hal_get_ts_backward(ts_ref_dev_ff,
                              (ff_parse_out_desc->ff_parse_rslt.bits.offset_ts_header * istate->xfer_time_per_byte_ticks));
}
#endif

//This is strongly HW dependentant
static
bool bmi26x_hal_dev_time_a_lt_b_short_itvl(
    int32_t                a,
    int32_t                b)
{
    bool                        ret;
    int32_t                     delta;

    delta = (int32_t)((a << 8) - (b << 8));

    ret = (delta >= 0);

    return ret;
}



static
void bmi26x_hal_fifo_resolve_ts_ref(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_parse_out_desc_t *ff_parse_out_desc)
{
    sns_time ts_curr;
    sns_time ts_ref_dev_ff = 0;
    sns_time ts_age = 0;
    uint32_t ts_dev_update_lsb = 0;
    uint8_t ts_dev_lsb_bits;
    uint8_t ts_dev_lsb_bits_acc;
    uint8_t ts_dev_lsb_bits_gyr;
    uint32_t ts_dev_ff = 0;

    bmi26x_fifo_info_t *fifo_info = &istate->fifo_info;
    bmi26x_fifo_time_info_t *ff_tm_info = &fifo_info->ff_tm_info;

#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
    sns_time                    ts_irq_est = 0;
    int32_t                     devi = 0;
#endif

#if BMI26X_CONFIG_ENABLE_PROFILING_SYS_DELAY
    sns_time                    ts_est_fifo_read_start = 0;
#endif

    ts_curr = bmi26x_get_sys_tick();

    bmi26x_hal_fifo_update_ts_hw_res(istate, &ts_curr);

    ts_dev_lsb_bits = BMI26X_REGV_ODR_1600HZ - fifo_info->ff_master_odr_curr + 4;
    ts_dev_lsb_bits_acc = BMI26X_REGV_ODR_1600HZ - istate->accel_info.odr_curr + 4;
    ts_dev_lsb_bits_gyr = BMI26X_REGV_ODR_1600HZ - istate->gyro_info.odr_curr + 4;


    //TODOMAG

    if (ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_ts) {
    } else {
        bmi26x_hal_fifo_est_ts_dev_ff(istate, ff_parse_out_desc);
    }

    ff_tm_info->avail_ts_this_batch = 0;

    if (ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_ts) {
        /* ts available in the FIFO */
        ts_dev_ff = ff_parse_out_desc->ts_dev_ff;
        bmi26x_hal_cor_ts_dev (istate, &ts_dev_ff);

        if (bmi26x_hal_dev_time_a_lt_b_short_itvl (ts_dev_ff, istate->ts_pair_sys_dev.ts_dev)) {
            ts_dev_update_lsb = bmi26x_dev_get_time_elapse_dev_lsb(istate->ts_pair_sys_dev.ts_dev, ts_dev_ff);
            ts_ref_dev_ff = bmi26x_hal_get_ts_forward (istate->ts_pair_sys_dev.ts_sys,
                            ts_dev_update_lsb * istate->ts_hw_res_ticks_per_bit);
        } else {
            ts_dev_update_lsb = bmi26x_dev_get_time_elapse_dev_lsb(ts_dev_ff, istate->ts_pair_sys_dev.ts_dev);
            ts_ref_dev_ff = bmi26x_hal_get_ts_backward(istate->ts_pair_sys_dev.ts_sys,
                            ts_dev_update_lsb * istate->ts_hw_res_ticks_per_bit);
        }

#if BMI26X_CONFIG_ENABLE_PROFILING_SYS_DELAY
        bmi26x_hal_fifo_get_est_ts_fifo_read_start_est(istate, ff_parse_out_desc, ts_ref_dev_ff, &ts_est_fifo_read_start);
#endif

        BMI26X_INST_LOG(LOW, istate->owner, "ts ref: ts.dev <%u %u> ts.sys:%u, res.per.lsb:<%u %u>, lsb:%u",
                        (uint32_t)istate->ts_pair_sys_dev.ts_dev,
                        (uint32_t)ts_dev_ff,
                        (uint32_t)istate->ts_pair_sys_dev.ts_sys,
                        (uint32_t)istate->ts_hw_res_ticks_per_bit,
                        (uint32_t)istate->ts_hw_res_ticks_per_bit_ideal,
                        (uint32_t)(ts_dev_update_lsb % 16));

        if (ff_parse_out_desc->fc_this_batch_acc) {
            ts_age = bmi26x_dev_get_ts_dev_age(ts_dev_ff, ts_dev_lsb_bits_acc) * istate->ts_hw_res_ticks_per_bit;
            ff_tm_info->ts_end_frame_this_batch_acc = bmi26x_hal_get_ts_backward(ts_ref_dev_ff, ts_age);
            ff_tm_info->avail_ts_this_batch |= BMI26X_ACCEL;
            BMI26X_INST_LOG(MED, istate->owner, "rslvtr a: %u @age:%u",
                            BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_acc),
                            (uint32_t) ts_age);
        }

        if (ff_parse_out_desc->fc_this_batch_gyr) {
            ts_age = bmi26x_dev_get_ts_dev_age(ts_dev_ff, ts_dev_lsb_bits_gyr) * istate->ts_hw_res_ticks_per_bit;
            ff_tm_info->ts_end_frame_this_batch_gyr = bmi26x_hal_get_ts_backward(ts_ref_dev_ff, ts_age);
            ff_tm_info->avail_ts_this_batch |= BMI26X_GYRO;
            BMI26X_INST_LOG(MED, istate->owner, "rslvtr g: %u @age:%u",
                            BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_gyr),
                            (uint32_t) ts_age);
        }
        //TODOMAG
    } else  {
        ff_tm_info->boost_read_size = 1;

#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
        //if (BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type) {
        if (BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type ||
                        BMI26X_FIFO_READ_CTX_TYPE_DAE == fifo_info->ff_read_ctx_type) {
            sns_time itvl;
            if ((istate->reg_int_ctx.fifo_len == fifo_info->ff_wml_bytes_curr)
                    && (fifo_info->ff_master_odr_req <= BMI26X_REGV_ODR_100HZ)
                    && (fifo_info->ff_wml_bytes_curr > 13)) {
                BMI26X_INST_LOG(HIGH, istate->owner, "good luck 3");

                if (ff_parse_out_desc->fc_this_batch_acc) {
                    itvl = fifo_info->ff_itvl_ticks_est_acc;
                    ff_tm_info->ts_end_frame_this_batch_acc = bmi26x_hal_get_ts_forward(istate->ts_irq,
                            itvl * (ff_parse_out_desc->fc_this_batch_acc - ff_parse_out_desc->fc_at_irq_acc));

                    BST_ASSUME_TS_IS_64BIT
                    ff_tm_info->ts_end_frame_this_batch_acc = (int64_t)ff_tm_info->ts_end_frame_this_batch_acc + fifo_info->devi_est_irq;
                    ff_tm_info->avail_ts_this_batch |= BMI26X_ACCEL;
                    BMI26X_INST_LOG(MED, istate->owner, "rslvtr a (irq): %u %u %u",
                                    BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_acc),
                                    ff_parse_out_desc->fc_this_batch_acc,
                                    ff_parse_out_desc->fc_at_irq_acc);
                }

                if (ff_parse_out_desc->fc_this_batch_gyr) {
                    //itvl = istate->ts_hw_res_ticks_per_bit * (1 << ts_dev_lsb_bits_gyr);
                    itvl = fifo_info->ff_itvl_ticks_est_gyr;
                    ff_tm_info->ts_end_frame_this_batch_gyr = bmi26x_hal_get_ts_forward(istate->ts_irq,
                            itvl * (ff_parse_out_desc->fc_this_batch_gyr - ff_parse_out_desc->fc_at_irq_gyr));

                    BST_ASSUME_TS_IS_64BIT
                    ff_tm_info->ts_end_frame_this_batch_gyr = (int64_t)ff_tm_info->ts_end_frame_this_batch_gyr + fifo_info->devi_est_irq;
                    ff_tm_info->avail_ts_this_batch |= BMI26X_GYRO;
                    BMI26X_INST_LOG(MED, istate->owner, "rslvtr g (irq): %u %u %u",
                                    BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_gyr),
                                    ff_parse_out_desc->fc_this_batch_gyr,
                                    ff_parse_out_desc->fc_at_irq_gyr);
                }
            }
        } else {
            //TODO2
            BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE ts_ref end not resolved");
            //would help if we know the time when fifo_read by async_com_port_data_stream was finished
            ts_dev_ff = (istate->ts_pair_sys_dev.ts_dev - 1) & (BMI26X_SPEC_TS_LSB_OVERFLOW_VAL - 1);
        }
#else
        BMI26X_INST_LOG(HIGH, istate->owner, "NOTICE no luck getting ts_ref due to delays in ascp_read");
#endif
    }

#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
    if (ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_ts) {
        if ((ff_parse_out_desc->fc_at_irq_masters > 0) &&
                (BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type ||
                                BMI26X_FIFO_READ_CTX_TYPE_DAE == fifo_info->ff_read_ctx_type)) {
                //(BMI26X_FIFO_READ_CTX_TYPE_WMI == fifo_info->ff_read_ctx_type)) {
            if (istate->reg_int_ctx.fifo_len == fifo_info->ff_wml_bytes_curr) {
                ts_age = bmi26x_dev_get_ts_dev_age(ts_dev_ff, ts_dev_lsb_bits) * istate->ts_hw_res_ticks_per_bit;
                ts_irq_est = bmi26x_hal_get_ts_backward(ts_ref_dev_ff, ts_age);

                ts_irq_est = bmi26x_hal_get_ts_backward(ts_irq_est,
                                                        (ff_parse_out_desc->fc_masters_this_batch - ff_parse_out_desc->fc_at_irq_masters) *
                                                        fifo_info->ff_itvl_ticks_est_masters);

                devi = ts_irq_est - istate->ts_irq;

                if (fifo_info->devi_est_irq) {
                    fifo_info->devi_est_irq = ((fifo_info->devi_est_irq << 3) + (devi << 1)) * 0.1f;
                } else {
                    fifo_info->devi_est_irq = devi;
                }

                BMI26X_INST_LOG(MED, istate->owner, "devi: %d %d",
                                devi, fifo_info->devi_est_irq); //noise p2p: +/-80us

#if BMI26X_CONFIG_ENABLE_PROFILING_SYS_DELAY
                BMI26X_INST_LOG(MED, istate->owner, "sys_delay: %d",
                                (int32_t)(ts_est_fifo_read_start - ts_irq_est));
#endif
            }
        }
    }
#else
        UNUSED_VAR(ts_dev_lsb_bits);
#endif

#if 0
    bmi26x_hal_fifo_fix_ts_ref(istate, ff_parse_out_desc);
#endif

    BMI26X_INST_LOG(MED, istate->owner, "resolve_ts_ref: <0x%x,%u,0x%x,0x%x,%u>",
                    ((istate->reg_int_ctx.fifo_len == fifo_info->ff_wml_bytes_curr) << 26) | (fifo_info->ff_read_ctx_type << 25) |
                    (ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_ts << 24) |
                    (ff_tm_info->avail_ts_last_batch << 20) | (ff_tm_info->avail_ts_this_batch << 16),
                    BMI26X_SYS_TIME_LH(istate->ts_pair_sys_dev.ts_sys),
                    istate->ts_pair_sys_dev.ts_dev,
                    ts_dev_ff + BMI26X_CONFIG_TS_DELAY_ADJ_I2C,
                    BMI26X_SYS_TIME_LH(ts_ref_dev_ff));

    bmi26x_hal_fifo_calc_ts_start_end_this_batch(istate, ff_parse_out_desc);

    if (BMI26X_TIME_ET(ff_tm_info->ts_end_frame_this_batch_acc, ff_tm_info->ts_end_frame_last_batch_acc) ||
            BMI26X_TIME_LT(ff_tm_info->ts_end_frame_this_batch_acc, ts_curr)) {
        INSERT_TRACE_POINT3_T(bmi26x, 'E', 'R', 'R');
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING 0x%x %u %u %u %u",
                        istate->ts_pair_sys_dev.ts_dev,
                        ts_dev_update_lsb,
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_last_batch_acc),
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_1st_frame_this_batch_acc),
                        BMI26X_SYS_TIME_LH(ff_tm_info->ts_end_frame_this_batch_acc));
    }

    ff_tm_info->ts_end_frame_last_batch_acc = ff_tm_info->ts_end_frame_this_batch_acc;
    ff_tm_info->ts_end_frame_last_batch_gyr = ff_tm_info->ts_end_frame_this_batch_gyr;

#if BMI26X_CONFIG_ENABLE_TS_IMPROVE
    istate->ts_irq_last = istate->ts_irq;
#endif

    ff_tm_info->avail_ts_last_batch |= ff_tm_info->avail_ts_this_batch;
    if (ff_tm_info->avail_ts_last_batch == fifo_info->ff_sensors_en_curr) {
        fifo_info->ff_tm_info.boost_read_size = 0;
    }
}

static
void bmi26x_hal_fifo_parse_n_proc_frames(
    bmi26x_instance_state           *istate,
    bmi26x_fifo_parse_ctx_t         *ff_parse_ctx,
    bmi26x_fifo_parse_out_desc_t    *ff_parse_out_desc)
{
    uint16_t                err_code = 0;
    uint8_t                 frame_header;
    uint8_t                 fh_mode;
    uint8_t                 fh_param;
    uint8_t                 frames_skipped;
    uint8_t                 frame_length;
    uint32_t                ff_buf_idx = 0;
    //uint8_t                 ff_cache_full_agm = 0;
    const uint8_t           *ff_buf = ff_parse_ctx->ff_buf;
    uint32_t                ff_buf_len = ff_parse_ctx->ff_buf_len;
    uint32_t                *ff_proc_len_left = &ff_parse_ctx->ff_proc_len_left;
    uint32_t                ts_dev_ff;
    uint32_t                fc_acc = 0;
    uint32_t                fc_gyr = 0;
    uint32_t                fc_mag = 0;
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
    bool                    ff_acc_need_fix = false;
    bool                    ff_gyr_need_fix = false;
    bool                    ff_mag_need_fix = false;

    uint8_t                 ff_acc_cnt_need_to_abandon  = istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc;
    uint8_t                 ff_gyr_cnt_need_to_abandon  = istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr;
#endif
    bool                    ff_parse_flag_dryrun = !!(ff_parse_ctx->ff_parse_flag & BMI26X_FIFO_PARSE_FLAG_DRYRUN);
    bmi26x_fifo_parse_result_t      *ff_parse_rslt = &ff_parse_out_desc->ff_parse_rslt;
    bmi26x_fifo_info_t      *fifo_info = &istate->fifo_info;

#if BMI26X_CONFIG_ENABLE_LOG_DATA_L3
    uint16_t i;

    for (i = 0; i < ff_buf_len; i++) {
        BMI26X_INST_LOG(LOW, istate->owner, "[ffpb] @%03d: 0x%02x",
                        i, ff_buf[i]);

        if (0x80 == ff_buf[i]) {
            i++;
            BMI26X_INST_LOG(LOW, istate->owner, "[ffpb] %d bytes omitted next byte: 0x%02x",
                            ff_buf_len - i, ff_buf[i]);
            break;
        }
    }
#endif

    err_code = 0;
    ff_buf_idx = 0;

    if (!ff_parse_flag_dryrun) {
        BMI26X_INST_LOG(HIGH, istate->owner, "4dbtap,log start >>>>>>");
    }

    while ((ff_buf_idx < ff_buf_len) &&  (!err_code)) {
        frame_header = ff_buf[ff_buf_idx];
        ff_buf_idx += 1;    // to the next
        if (BMI26X_FF_FH_EMPTY == frame_header) {
            // fifo empty
            if (ff_parse_flag_dryrun) {
                BMI26X_INST_LOG(LOW, istate->owner, "@%d f_end", ff_buf_idx - 1);
            }

            if (0 == (ff_buf_idx - 1)) {
                ff_parse_rslt->bits.ff_batch_empty = 1;
            }

            ff_parse_rslt->bits.ff_avail_end_frame = 1;
            break;
        }

        fh_mode         = BMI26X_FF_FH_MODE(frame_header);
        fh_param        = BMI26X_FF_FH_PARAM(frame_header);

#if BMI26X_CONFIG_ENABLE_LOG_DATA_L3
        BMI26X_INST_LOG(MED, istate->owner, "[ffp]frame_header = 0x%02x fh_mode: 0x%02x fh_param: 0x%02x",
                        frame_header, fh_mode, fh_param);
#endif

        if (BMI26X_FF_FH_MODE_REGULAR == fh_mode) {
            frame_length = 0;
            frame_length += ((fh_param & BMI26X_MAG) ? BMI26X_FF_DATA_LEN_MAG : 0);
            frame_length += ((fh_param & BMI26X_GYRO) ? BMI26X_FF_DATA_LEN_GYR : 0);
            frame_length += ((fh_param & BMI26X_ACCEL) ? BMI26X_FF_DATA_LEN_ACC : 0);

            if (ff_buf_idx + frame_length > ff_buf_len) {
                // buf is over
                BMI26X_INST_LOG(MED, istate->owner,
                                "[ffp]INFO!!! partial read of frame, %d bytes yet to be read, total: %d fh: 0x%x",
                                ff_buf_idx + frame_length - ff_buf_len,
                                ff_buf_len,
                                frame_header);

                err_code = (BMI26X_FF_FRAME_ERR_PARTIAL_READ << 8) | frame_header;
                break;
            }
#if 0
            ff_cache_full_agm = 0;

            if (!ff_parse_flag_dryrun) {
                //bmi26x_hal_fifo_check_buf_cap(istate, fh_param, &ff_cache_full_agm);
                if (!ff_cache_full_agm) {
                } else {
                    BMI26X_INST_LOG(HIGH, istate->owner, "ff_cache_full: %d", ff_cache_full_agm);
                }
            }

            ff_parse_rslt->bits.ff_cache_full_agm = ff_cache_full_agm;
            if (!ff_cache_full_agm) { }
#endif
            {

                if (fh_param & BMI26X_MAG) {
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
                    // mag data
                    if (ff_mag_need_fix) {
                        //ff_mag_need_fix = false;
                        //ff_parse_rslt->bits.ff_batch_empty = 1;
                        BMI26X_INST_LOG(MED, istate->owner, "mag.droped");
                    } else
#endif
                    {
                        //ff_parse_rslt->bits.ff_batch_empty = 0;
                        if (!ff_parse_flag_dryrun) {
                            fc_mag ++;
                            bmi26x_hal_fifo_handle_frame_mag(istate, ff_parse_ctx, ff_parse_out_desc, fc_mag, ff_buf + ff_buf_idx);
                        } else {
                            ff_parse_out_desc->fc_this_batch_mag ++;
                        }
                    }

                    ff_buf_idx += BMI26X_FF_DATA_LEN_MAG;
                }

                if (fh_param & BMI26X_GYRO) {
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
                    bool need_check_abandon_st = false;
                    if (!istate->fifo_info.ff_flush_in_proc) {  //ignore under flush
                        need_check_abandon_st = true;
                    } else {
                        // in flushing st
                        if (fifo_info->ff_tm_info.fc_accum_curr_gyr) {
                            // under active odr
                        } else {
                            // unactive odr
                            need_check_abandon_st = true;
                        }
                    }

                    if (need_check_abandon_st) {
                        if (ff_gyr_cnt_need_to_abandon)  {
                            ff_gyr_cnt_need_to_abandon --;
                            ff_gyr_need_fix = true;
                        } else {
                            ff_gyr_need_fix = false;
                        }
                    }
                    // gyr data
                    if (ff_gyr_need_fix) {
                        BMI26X_INST_LOG(MED, istate->owner, "gyro.droped: %d %d",
                                        ff_gyr_cnt_need_to_abandon,
                                        istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr);
                    } else
#endif
                    {
                        //ff_parse_rslt->bits.ff_batch_empty = 0;
                        if (!ff_parse_flag_dryrun) {
                            fc_gyr ++;
                            bmi26x_hal_fifo_handle_frame_gyr(istate, ff_parse_ctx, ff_parse_out_desc,
                                                            fc_gyr, ff_buf + ff_buf_idx);
                        } else {
                            ff_parse_out_desc->fc_this_batch_gyr ++;
                        }
                    }

                    ff_buf_idx += BMI26X_FF_DATA_LEN_GYR;
                }

                if (fh_param & BMI26X_ACCEL) {
                    // acc data
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES

                    bool need_check_abandon_st = false;
                    if (!istate->fifo_info.ff_flush_in_proc) {  //ignore under flush
                        need_check_abandon_st = true;
                    } else {
                        // in flushing st
                        if (fifo_info->ff_tm_info.fc_accum_curr_acc) {
                            // under active odr
                        } else {
                            // unactive odr
                            need_check_abandon_st = true;
                        }
                    }

                    if (need_check_abandon_st) {  // ignore under flush
                        if (ff_acc_cnt_need_to_abandon) {
                            ff_acc_cnt_need_to_abandon --;
                            ff_acc_need_fix = true;
                        } else {
                            ff_acc_need_fix = false;
                        }
                    }

                    if (ff_acc_need_fix) {
                        BMI26X_INST_LOG(MED, istate->owner, "acc.droped: %d %d",
                                        ff_acc_cnt_need_to_abandon,
                                        istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc);
                    } else
#endif
                    {
                        //ff_parse_rslt->bits.ff_batch_empty = 0;
                        if (!ff_parse_flag_dryrun) {
                            fc_acc ++;
                            bmi26x_hal_fifo_handle_frame_acc(istate, ff_parse_ctx, ff_parse_out_desc,
                                                            fc_acc, ff_buf + ff_buf_idx);
                        } else {
                            ff_parse_out_desc->fc_this_batch_acc ++;
                        }
                    }

                    ff_buf_idx += BMI26X_FF_DATA_LEN_ACC;
                }

                if (ff_parse_flag_dryrun) {
                    bmi26x_hal_fifo_update_sync_info(istate, fh_param, ff_parse_ctx, ff_parse_out_desc);

                    if (ff_buf_idx >= fifo_info->ff_wml_bytes_curr) {
                        if (0 == ff_parse_out_desc->ff_frm_header_at_irq) {
                            ff_parse_out_desc->ff_frm_header_at_irq = fh_param;

                            ff_parse_out_desc->fc_at_irq_acc = ff_parse_out_desc->fc_this_batch_acc;
                            ff_parse_out_desc->fc_at_irq_gyr = ff_parse_out_desc->fc_this_batch_gyr;
                            ff_parse_out_desc->fc_at_irq_mag = ff_parse_out_desc->fc_this_batch_mag;
                            ff_parse_out_desc->fc_at_irq_masters = ff_parse_out_desc->fc_masters_this_batch;

                            BMI26X_INST_LOG(MED, istate->owner, "fc_at_irq: 0x%x@%d %d %d",
                                            fh_param, ff_buf_idx,
                                            ff_parse_out_desc->fc_this_batch_acc, ff_parse_out_desc->fc_this_batch_gyr);
                        }
                    }
                }
            }
        } else if (BMI26X_FF_FH_MODE_CTRL == fh_mode) {
            switch (fh_param) {
                case 0x00:
                    if (ff_buf_idx + 1 > ff_buf_len) {
                        err_code = (BMI26X_FF_FRAME_ERR_PARTIAL_READ << 8)
                                   | frame_header;
                        break;
                    }
                    //fifo overflowed
                    ff_parse_rslt->bits.ff_overflow = true;

                    frames_skipped = ff_buf[ff_buf_idx];
                    ff_buf_idx += 1;
                    BMI26X_INST_LOG(MED, istate->owner,
                                    "[ffp]@%d frame_ctrl WARNING!!! frames_skipped = 0x%02x some samples are missing due to delay of FIFO read",
                                    ff_buf_idx - 2,
                                    frames_skipped);

                    BST_IGNORE_WARNING_UNUSED_VAR(frames_skipped);

                    *ff_proc_len_left = 0;
                    goto exit_parse_ff_frames;
                case 0x01:
                    ff_parse_rslt->bits.ff_avail_ts_header = 1;

                    if (ff_buf_idx + 3 > ff_buf_len) {
                        BMI26X_INST_LOG(MED, istate->owner, "[ffp]@%d INFO!!! frame_ctrl sensortime partial read, %d more bytes needed",
                                        ff_buf_idx - 1, ff_buf_len + 3 - ff_buf_idx);

                        err_code = (BMI26X_FF_FRAME_ERR_PARTIAL_READ << 8)
                                   | frame_header;
                        break;
                    }

                    if (ff_parse_flag_dryrun) {
                        bmi26x_dev_parse_data_ts(ff_buf + ff_buf_idx, &ts_dev_ff);
                        ff_parse_out_desc->ts_dev_ff = ts_dev_ff;

                        ff_parse_rslt->bits.offset_ts_header = ff_buf_idx;

                        BMI26X_INST_LOG(MED, istate->owner,
                                        "@%d f_ctrl st:0x%x",
                                        ff_buf_idx - 1,
                                        ts_dev_ff);
                    } else {
                        const uint8_t           *buf = ff_buf + ff_buf_idx;
                        BMI26X_INST_LOG(HIGH, istate->owner,
                                       "4dbtap,f_ctrl st:0x44,0x%02x,0x%02x,0x%02x",
                                       buf[0], buf[1], buf[2]);
                    }

                    ff_buf_idx += 3;
                    ff_parse_rslt->bits.ff_avail_ts = 1;
                    break;
                case 0x02:
                    // Fifo_Input_Config Frame
                    if (ff_buf_idx + 4 > ff_buf_len) {
                        err_code = (BMI26X_FF_FRAME_ERR_PARTIAL_READ << 8)
                                   | frame_header;
                        break;
                    }

                    uint8_t                 frame_input_cfg;
                    uint32_t                ts_dev_ff_next;
                    frame_input_cfg = ff_buf[ff_buf_idx];   // 0x48 | frame_input_cfg@ff_buf_idx
                    bmi26x_dev_parse_data_ts(ff_buf + ff_buf_idx + 1, &ts_dev_ff_next);
                    ff_buf_idx += 4;
                    BMI26X_INST_LOG(MED, istate->owner, "[ffp]@%d f_ctrl Fifo_Input_Config=0x%02x 0x%x",
                                    ff_buf_idx - 5,
                                    frame_input_cfg, ts_dev_ff_next);
                    BST_IGNORE_WARNING_UNUSED_VAR(frame_input_cfg);

                    if (BST_GET_VAL_BITBLOCK(frame_input_cfg, 0, 1)) {
                        ff_acc_cnt_need_to_abandon = 0;
                        BMI26X_INST_LOG(MED, istate->owner, "[ffp] fast fix acc due to new chg active" );
                    }

                    if (BST_GET_VAL_BITBLOCK(frame_input_cfg, 2, 3)) {
                        ff_gyr_cnt_need_to_abandon = 0;
                        BMI26X_INST_LOG(MED, istate->owner, "[ffp] fast fix gyr due to new chg active" );
                    }


                    break;
                default:
                    BMI26X_INST_LOG(HIGH, istate->owner,
                                    "[ffp]@%d frame_unknown WARNING!!! frame format unknown frame_header: 0x%02x",
                                    ff_buf_idx,
                                    frame_header);
                    err_code = (BMI26X_FF_FRAME_ERR_UNKNOWN << 8)
                               | frame_header;

                    *ff_proc_len_left = 0;
                    goto exit_parse_ff_frames;
            }
        } else {
            BMI26X_INST_LOG(HIGH, istate->owner, "[ffp]@%d frame_unknown WARNING!!! unknown fifo frame_header: 0x%02x",
                            ff_buf_idx - 1,
                            frame_header);
            err_code = (BMI26X_FF_FRAME_ERR_UNKNOWN << 8)
                       | frame_header;
            break;
        }
    }
    //end of while

    *ff_proc_len_left = 0;
exit_parse_ff_frames:
    if (err_code &&
            (((err_code >> 8) & 0xff) != BMI26X_FF_FRAME_ERR_PARTIAL_READ)) {
        BMI26X_INST_LOG(HIGH, istate->owner, "[ffp] WARNING!!! error seen during parsing: <err: 0x%04x %d %d>",
                        err_code, ff_buf_idx, ff_buf_len);

        uint8_t i;
        for (i = 0; (i < 8) && ((i + ff_buf_idx) < ff_buf_len); i++) {
            BMI26X_INST_LOG(HIGH, istate->owner, "[ffpb] @%03d: 0x%02x",
                            i, ff_buf[i + ff_buf_idx]);
        }
    }

#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
    // update
    if (!ff_parse_flag_dryrun) {
        if (istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc > 0) {
            istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc = ff_acc_cnt_need_to_abandon;
        }

        if (istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr > 0) {
            istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr = ff_gyr_cnt_need_to_abandon;
        }
    }
#endif

    if (!ff_parse_flag_dryrun) {
        BMI26X_INST_LOG(HIGH, istate->owner, "4dbtap,log end <<<<<<<");
    }

}

void bmi26x_hal_flush_cleanup(bmi26x_instance_state *istate, uint8_t context)
{
    sns_time ts_curr = bmi26x_get_sys_tick();
    if (istate->fifo_info.ff_sensors_en_curr & BMI26X_ACCEL) {
        //TODO: does it matter if we do this after bmi26x_hal_reconfig_hw
        bmi26x_hal_send_fifo_flush_done(istate->owner, (uint8_t)BMI26X_ACCEL, ts_curr, context);
    }

    if (istate->fifo_info.ff_sensors_en_curr & BMI26X_GYRO) {
        bmi26x_hal_send_fifo_flush_done(istate->owner, (uint8_t)BMI26X_GYRO, ts_curr, context);
    }
    //TODOMAG
    BMI26X_INST_LOG(LOW, istate->owner, "flush_done event: %d,%d @%u",
                    context, istate->fifo_info.ff_sensors_en_curr, BMI26X_SYS_TIME_LH(ts_curr));
}


static
void bmi26x_hal_fifo_read_out(
    bmi26x_instance_state   *istate,
    bmi26x_fifo_read_ctx_t  *ctx)
{
    uint8_t                     buffer[100];
    uint32_t                    enc_len;
    sns_port_vector             async_read_msg;

    uint16_t                    ff_len = 0;

    UNUSED_VAR(ctx);

    if (bmi26x_hal_fifo_determine_ff_len(istate, ctx, &ff_len) != SNS_RC_SUCCESS) {
    }



    if (ff_len > 0) {
        bool    com_port_read_granted = ((istate->async_com_read_request - istate->async_com_read_response) == 0);

        if (((istate->accel_info.odr_curr >= BMI26X_REGV_ODR_800HZ) || (istate->gyro_info.odr_curr >= BMI26X_REGV_ODR_800HZ)) &&
                com_port_read_granted) {
            //ctx->sync_read = true;
        }

        if (!ctx->sync_read) {
            // Compose the async com port message
            async_read_msg.bytes = ff_len + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY;
            async_read_msg.reg_addr = BMI26X_REGA_USR_FIFO_DATA;
            async_read_msg.is_write = false;
            async_read_msg.buffer = NULL;

            sns_ascp_create_encoded_vectors_buffer(&async_read_msg, 1, true, buffer, sizeof(buffer), &enc_len);

            // Send message to Async Com Port
            sns_request async_com_port_request =
            (sns_request) {
                .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW,
                .request_len = enc_len,
                .request = buffer
            };

            sns_rc              rc = SNS_RC_POLICY;
            com_port_read_granted = (istate->async_com_read_request - istate->async_com_read_response) <
                                    BMI26X_CONFIG_COM_FLOW_CTRL_THRESH;

            if (com_port_read_granted) {
                rc = istate->async_com_port_data_stream->api->send_request(
                         istate->async_com_port_data_stream, &async_com_port_request);
                if (rc != SNS_RC_SUCCESS) {
                    BMI26X_INST_LOG(ERROR, istate->owner, "async_com_req send failure: %d", rc);
                }
            } else {
                BMI26X_INST_LOG(MED, istate->owner, "dropped async_com_req: %d",
                                (istate->async_com_read_request - istate->async_com_read_response));
            }

            if (SNS_RC_SUCCESS == rc) {
                istate->async_com_read_request ++;
            } else {
                if (com_port_read_granted) {
                    BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! ascp_req: %d", rc);
                }
            }
        } else {
            //FIXME, need to check
            sns_rc              rc = SNS_RC_SUCCESS;
            uint8_t  fifo_buffer[ff_len + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY];

            sns_memset(fifo_buffer, 0x00, sizeof(fifo_buffer));

            sns_port_vector port_vec;
            uint32_t xfer_bytes = 0;

            port_vec.buffer = fifo_buffer;
            port_vec.bytes = ff_len;

            if (istate->bus_is_spi) {
                port_vec.bytes = ff_len + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY;
            } else {
            }

            port_vec.is_write = false;
            port_vec.reg_addr = BMI26X_REGA_USR_FIFO_DATA;

            rc = bmi26x_fp_scp_rw(istate->com_port_info.port_handle, &port_vec,
                    1, false, &xfer_bytes);
            if (rc != SNS_RC_SUCCESS) {
            }

            // handle the fifo data right now
            bmi26x_hal_process_fifo_data_buffer(istate->owner, fifo_buffer,
                    ff_len, BMI26X_INT_TRIGGER_SOURCE_SEE);
        }
    } else {
        bmi26x_fifo_info_t   *fifo_info = &istate->fifo_info;
        // reset FIFO istate if there are no samples to flush
        if (fifo_info->ff_flush_in_proc) {
            if ((istate->async_com_read_request - istate->async_com_read_response) <= 0) {
                fifo_info->ff_flush_in_proc = false;

                if (istate->ff_flush_client_req) {
                    istate->ff_flush_client_req = 0;
                }

                bmi26x_hal_flush_cleanup(istate, BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_REQ);
                //TODOMAG
            } else {
                BMI26X_INST_LOG(HIGH, istate->owner, "#flush# not to be done %d",
                                (int32_t) (istate->async_com_read_request - istate->async_com_read_response));
            }
        }
    }
}


void bmi26x_hal_fifo_drain(
    bmi26x_instance_state   *istate,
    bool                    sync_read,
    bmi26x_fifo_flush_trigger_t trigger)
{
    bmi26x_fifo_read_ctx_t      ctx;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    ctx.ctx_type = BMI26X_FIFO_READ_CTX_TYPE_FLUSH;
    ctx.sync_read = sync_read;

    fifo_info->ff_read_ctx_type = BMI26X_FIFO_READ_CTX_TYPE_FLUSH;

    if (!sync_read) {
        fifo_info->ff_flush_in_proc = true;
    } else {
        fifo_info->ff_flush_in_proc = false;
    }
    fifo_info->ff_flush_trigger = trigger;

    BMI26X_INST_LOG(HIGH, istate->owner, "fifo_flush trigger: %d %d", trigger, sync_read);

    bmi26x_hal_fifo_read_out(istate, &ctx);
}


void bmi26x_hal_fifo_update_master_odr_req(bmi26x_instance_state *istate)
{
    uint16_t    odr_ff_master = BMI26X_REGV_ODR_OFF;
    bmi26x_fifo_info_t   *fifo_info = &istate->fifo_info;

    if (fifo_info->publish_sensors & BMI26X_ACCEL) {
        if (istate->accel_info.ff_wml_req) {
            odr_ff_master = SNS_MAX(odr_ff_master, istate->accel_info.odr_req);
        }
    }

    if (fifo_info->publish_sensors & BMI26X_GYRO) {
        if (istate->gyro_info.ff_wml_req) {
            odr_ff_master = SNS_MAX(odr_ff_master, istate->gyro_info.odr_req);
        }
    }
    //TODOMAG

    fifo_info->ff_master_odr_req = (bmi26x_regv_odr_t) odr_ff_master;
}


static
void bmi26x_hal_fifo_invalidate_master_info(
    bmi26x_instance_state *istate)
{
    bmi26x_fifo_info_t     *fifo_info = &istate->fifo_info;

    fifo_info->ff_tm_info.fc_masters_last_dev_ff = 0;
    fifo_info->ff_tm_info.fc_accum_masters = 0;
}

void bmi26x_hal_fifo_update_curr_masters(bmi26x_instance_state *istate)
{
    uint8_t                     ff_masters = 0;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    if (fifo_info->publish_sensors & BMI26X_ACCEL) {
        if (istate->accel_info.ff_wml_req) {
            if (istate->accel_info.odr_curr == fifo_info->ff_master_odr_req) {
                ff_masters |= BMI26X_ACCEL;
            }
        }
    }

    if (fifo_info->publish_sensors & BMI26X_GYRO) {
        if (istate->gyro_info.ff_wml_req) {
            if (istate->gyro_info.odr_curr == fifo_info->ff_master_odr_req) {
                ff_masters |= BMI26X_GYRO;
            }
        }
    }
    //TODOMAG

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        ff_masters = 0;
    }
#endif

    BMI26X_INST_LOG(MED, istate->owner, "bmi26x_cp_ fifo_update_master: <0x%x 0x%x %d %d>",
                    (int)ff_masters, (int)fifo_info->ff_master_sensors_curr,
                    (int)fifo_info->ff_master_odr_curr, (int)fifo_info->ff_master_odr_req);

    if (!(ff_masters & fifo_info->ff_master_sensors_curr) ||
            (fifo_info->ff_master_odr_curr != fifo_info->ff_master_odr_req)) {
        bmi26x_hal_fifo_invalidate_master_info(istate);
    }

    //update
    fifo_info->ff_master_sensors_curr = ff_masters;
    fifo_info->ff_master_odr_curr = fifo_info->ff_master_odr_req;

    fifo_info->ff_master_sensors_ready = 0;
    fifo_info->ff_all_synced = 0;
}


void bmi26x_hal_fifo_validate_n_adj_wml_req(
    bmi26x_instance_state   *istate,
    uint32_t                *ff_wml_req_acc,
    uint32_t                *ff_wml_req_gyr,
    uint32_t                *ff_wml_req_mag)
{
    uint32_t                    mcd;
    uint32_t                    wml_adj;
    uint32_t                    wml_adj_needed = 0;
    uint32_t                    ff_max_len = 0;

    UNUSED_VAR(istate);


    wml_adj_needed = ((*ff_wml_req_acc) >= BMI26X_FF_MAX_FRAMES_IMU);
    wml_adj_needed |= (((*ff_wml_req_gyr) >= BMI26X_FF_MAX_FRAMES_IMU) << 1);
    //TODOMAG

    if (istate->accel_info.max_batching_available &&
            (*ff_wml_req_gyr) >= BMI26X_FF_MAX_FRAMES_IMU) {
        // MAX batching
        wml_adj = (BMI26X_FF_MAX_FRAMES_IMU - 1);
    } else {
        if ((*ff_wml_req_acc >= 0xFFFF) || (*ff_wml_req_gyr >= 0xFFFF)) {
            mcd = (*ff_wml_req_acc >= 0xFFFF) ? (*ff_wml_req_gyr) : (*ff_wml_req_acc);
            if (mcd > 0) {
                uint32_t req_sr_max = SNS_MAX(istate->accel_info.sample_rate_req, istate->gyro_info.sample_rate_req);
                uint32_t req_rr_max = SNS_MAX(istate->accel_info.report_rate_req, istate->gyro_info.report_rate_req);
                uint32_t req_adj_wml = 0;
                if (req_rr_max > 0) {
                    req_adj_wml = req_sr_max / req_rr_max;
                }
                if (req_adj_wml > 0) {
                    if ((req_adj_wml % mcd) == 0) {
                        mcd = req_adj_wml;
                    }
                }
            } else {
                mcd = (BMI26X_FF_MAX_FRAMES_IMU - 1);
            }
        } else {
            mcd = bmi26x_util_get_com_div(*ff_wml_req_acc, *ff_wml_req_gyr);
            mcd = bmi26x_util_get_com_div(mcd, *ff_wml_req_mag);
        }

        wml_adj = mcd;

        while (true) {
            ff_max_len = (*ff_wml_req_acc) ? ((BMI26X_FF_DATA_LEN_ACC + 1) * wml_adj) : 0;
            ff_max_len += (*ff_wml_req_gyr) ? ((BMI26X_FF_DATA_LEN_GYR + 1) * wml_adj) : 0;
            //ff_max_len += *ff_wml_req_mag ? ((BMI26X_FF_DATA_LEN_MAG + 1) * wml_adj) : 0;   //TODOMAG

            if ((ff_max_len < (BMI26X_FF_DEPTH_BYTES - BMI26X_CONFIG_FIFO_HEADROOM)) && (wml_adj < BMI26X_FF_MAX_FRAMES_IMU)) {
                break;
            } else {
                wml_adj = bmi26x_util_get_max_div(mcd, BST_MIN((wml_adj - 1), BMI26X_FF_MAX_FRAMES_IMU));

                wml_adj_needed |= (ff_max_len >= (BMI26X_FF_DEPTH_BYTES - BMI26X_CONFIG_FIFO_HEADROOM)) ? (1 << 8) : 0;
            }
        }
    }

    BMI26X_INST_LOG(MED, istate->owner, "wml_adj <%d %d %d>",
                    *ff_wml_req_acc, *ff_wml_req_gyr, *ff_wml_req_mag);

    if (wml_adj_needed) {
        BMI26X_INST_LOG(MED, istate->owner, "wml_adj_needed <0x%x %d>",
                        wml_adj_needed, wml_adj);

        *ff_wml_req_acc = wml_adj;
        *ff_wml_req_gyr  = wml_adj;

        istate->accel_info.ff_wml_req = wml_adj;
        istate->gyro_info.ff_wml_req = wml_adj;
#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
        *ff_wml_req_mag  = wml_adj;
        istate->mag_info.ff_wml_req = wml_adj;
#endif
    }
}

void bmi26x_hal_fifo_calc_wml_req_ldt(bmi26x_instance_state *istate)
{
    uint16_t                    odr_acc = 0;
    uint16_t                    odr_gyr = 0;
    uint16_t                    odr_mag = 0;
    uint16_t                    odr_fifo_master  = 0;
    uint32_t                    fc_when_time_expires_acc = 0;
    uint32_t                    fc_when_time_expires_gyr = 0;
    uint32_t                    fc_when_time_expires_mag = 0;
    uint32_t                    fc_wml_master = 0;
    uint32_t                    fc_wml_acc_max = 0;
    uint32_t                    fc_wml_gyr_max = 0;
    uint32_t                    fc_wml_mag_max = 0;
    uint32_t                    ff_wml_req_acc = 0;
    uint32_t                    ff_wml_req_gyr = 0;
    uint32_t                    ff_wml_req_mag = 0;
    uint32_t                    bytes_when_time_expires_max = 0;
    uint32_t                    bytes_when_wml_fires = 0;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    ff_wml_req_acc = istate->accel_info.ff_wml_req;
    ff_wml_req_gyr = istate->gyro_info.ff_wml_req;

    bmi26x_hal_fifo_validate_n_adj_wml_req(istate, &ff_wml_req_acc, &ff_wml_req_gyr, &ff_wml_req_mag);

    odr_acc = istate->accel_info.odr_req;
    odr_gyr = istate->gyro_info.odr_req;
    //TODOMAG
    odr_fifo_master = fifo_info->ff_master_odr_req;

    if (odr_acc > 0) {
        fc_when_time_expires_acc = (1 << (odr_fifo_master - odr_acc)) * ff_wml_req_acc;
    }

    if (odr_gyr > 0) {
        fc_when_time_expires_gyr = (1 << (odr_fifo_master - odr_gyr)) * ff_wml_req_gyr;
    }

    if (odr_mag > 0) {
        //TODOMAG
    }

    fc_wml_master = bmi26x_util_get_com_div(fc_when_time_expires_acc, fc_when_time_expires_gyr);
    fc_wml_master = bmi26x_util_get_com_div(fc_wml_master, fc_when_time_expires_mag);

    if (odr_acc > 0) {
        fc_wml_acc_max = (odr_acc == odr_fifo_master) ? fc_wml_master : 0;
        fc_wml_acc_max = fc_wml_acc_max ? fc_wml_acc_max : 1;
    }

    if (odr_gyr > 0) {
        fc_wml_gyr_max = (odr_gyr == odr_fifo_master) ? fc_wml_master : 0;
        fc_wml_gyr_max = fc_wml_gyr_max ? fc_wml_gyr_max : 1;
    }

    if (odr_mag > 0) {
        fc_wml_mag_max = (odr_mag == odr_fifo_master) ? fc_wml_master : 0;
        fc_wml_mag_max = fc_wml_mag_max ? fc_wml_mag_max : 1;
    }

    bytes_when_time_expires_max =
        BMI26X_FF_DATA_LEN_ACC * fc_wml_acc_max +
        BMI26X_FF_DATA_LEN_GYR * fc_wml_gyr_max +
        BMI26X_FF_DATA_LEN_MAG * fc_wml_mag_max +
        + fc_wml_master;

    bytes_when_wml_fires =
        BMI26X_FF_DATA_LEN_ACC * (fc_wml_master >> (odr_fifo_master - odr_acc)) +
        BMI26X_FF_DATA_LEN_GYR * (fc_wml_master >> (odr_fifo_master - odr_gyr)) +
        BMI26X_FF_DATA_LEN_MAG * (fc_wml_master >> (odr_fifo_master - odr_mag)) +
        + fc_wml_master;

    if ((1000000 * fc_wml_master / BMI26X_REGV_ODR_MAP[odr_fifo_master].odr) <=
            BMI26X_CONFIG_FIFO_WMI_MISSING_ITVL_US_THRES) {
        fifo_info->ff_wmi_missing_possible = 1;
        BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ ff_wmi_missing_possible");
    } else {
        fifo_info->ff_wmi_missing_possible = 0;
    }

    //OUTPUT
    BMI26X_INST_LOG(MED, istate->owner, "odr <%d %d %d %d>",
                    odr_acc, odr_gyr, odr_mag, odr_fifo_master);

    BMI26X_INST_LOG(MED, istate->owner, "fc_when_time_expires<%d %d %d> fc_wml_master: %d",
                    fc_when_time_expires_acc, fc_when_time_expires_gyr, fc_when_time_expires_mag, fc_wml_master);

    fifo_info->ff_suggested_bytes2read = bytes_when_time_expires_max;

    BMI26X_INST_LOG(MED, istate->owner, "[calc_wml_ldt] ff_wml_bytes: %d ff_suggested_bytes2read: %d",
                    bytes_when_wml_fires, fifo_info->ff_suggested_bytes2read);

    if (bytes_when_wml_fires > (BMI26X_FF_DEPTH_BYTES - BMI26X_CONFIG_FIFO_HEADROOM)) {
        bytes_when_wml_fires = BMI26X_FF_DEPTH_BYTES - BMI26X_CONFIG_FIFO_HEADROOM;
        fifo_info->ff_suggested_bytes2read = bytes_when_wml_fires;
    }

    fifo_info->ff_wml_bytes_req = bytes_when_wml_fires;

}



//TODO: check more
void bmi26x_hal_fifo_invalidate_sensors(
    bmi26x_instance_state   *istate,
    uint32_t                sensors_to_invalidate)
{
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    uint32_t                    sensors_to_invalidate_all;

    sensors_to_invalidate_all = sensors_to_invalidate;

    if (sensors_to_invalidate_all) {
        BMI26X_INST_LOG(MED, istate->owner, "bmi26x_cp_ sensors_to_invalidate: %x %x %x %x",
                        (int)sensors_to_invalidate, (int)fifo_info->ff_sensors_en_curr, (int)fifo_info->ff_sensors_en_req,
                        (int)sensors_to_invalidate_all);
    }

    fifo_info->ff_tm_info.avail_ts_last_batch &= ~(sensors_to_invalidate_all);

    if (sensors_to_invalidate_all & BMI26X_ACCEL) {
        fifo_info->ff_tm_info.fc_accum_curr_acc = 0;
    }

    if (sensors_to_invalidate_all & BMI26X_GYRO) {
        fifo_info->ff_tm_info.fc_accum_curr_gyr = 0;
    }

    if ((fifo_info->ff_tm_info.fc_accum_curr_acc == 0) ||
            (fifo_info->ff_tm_info.fc_accum_curr_gyr == 0)) {
        istate->cnt_hw_res_est = 0;
    }

    //TODOMAG
}


uint8_t bmi26x_hal_fifo_get_sensors_to_invalidate(bmi26x_instance_state   *istate)
{
    uint8_t                     ff_sensors_to_invalidate = 0;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;


    //if range of existing sensors in FIFO changed
    if (fifo_info->ff_sensors_en_curr & BMI26X_ACCEL) {
        if (istate->accel_info.range_curr != istate->accel_info.range_req) {
            ff_sensors_to_invalidate |= BMI26X_ACCEL;
        }

        if (istate->accel_info.odr_curr != istate->accel_info.odr_req) {
            ff_sensors_to_invalidate |= BMI26X_ACCEL;
            bmi26x_regv_odr_t temp_current_odr = istate->accel_info.odr_curr;
            if (temp_current_odr == BMI26X_REGV_ODR_OFF) {
                temp_current_odr = BMI26X_REGV_ODR_100HZ; //0xA8
            }
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
            if (temp_current_odr > istate->accel_info.odr_req) {
                uint8_t ff_need_to_stab_after_odr_chg = (1 << (temp_current_odr - istate->accel_info.odr_req));
                istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc = SNS_MAX(ff_need_to_stab_after_odr_chg, 4);
            } else {
                uint8_t ff_need_to_stab_after_odr_chg = istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc;
                istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc = SNS_MAX(ff_need_to_stab_after_odr_chg, 2);
            }
#endif
        }

        if (!(fifo_info->ff_sensors_en_req & BMI26X_ACCEL)) {
            ff_sensors_to_invalidate |= BMI26X_ACCEL;
        }
    }

    // gyro
    if (fifo_info->ff_sensors_en_curr & BMI26X_GYRO) {
        if (istate->gyro_info.range_curr != istate->gyro_info.range_req) {
            ff_sensors_to_invalidate |= BMI26X_GYRO;
        }

        if (istate->gyro_info.odr_curr != istate->gyro_info.odr_req) {
            ff_sensors_to_invalidate |= BMI26X_GYRO;
            bmi26x_regv_odr_t temp_current_odr = istate->gyro_info.odr_curr;
            if (temp_current_odr == BMI26X_REGV_ODR_OFF) {
                temp_current_odr = BMI26X_REGV_ODR_200HZ; //0xA9
            }
#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
            if (temp_current_odr > istate->gyro_info.odr_req) {
                uint8_t ff_need_to_stab_after_odr_chg = (1 << (temp_current_odr - istate->gyro_info.odr_req));
                istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr = SNS_MAX(ff_need_to_stab_after_odr_chg, 4);
            } else {
                uint8_t ff_need_to_stab_after_odr_chg = istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr;
                istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr = SNS_MAX(ff_need_to_stab_after_odr_chg, 2);
            }
#endif
        }

        if (!(fifo_info->ff_sensors_en_req & BMI26X_GYRO)) {
            ff_sensors_to_invalidate |= BMI26X_GYRO;
        }
    }

    //TODOMAG

    return ff_sensors_to_invalidate;
}

static
sns_rc bmi26x_hal_prepare_cross_sense(sns_sensor_instance *this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint8_t gyr_cross_ss[2] = {0};

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_OUT_CROSS_SENSE, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_OUTPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "cross sense @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, gyr_cross_ss, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    istate->gyro_info.signed_gyr_cross_sense_coef = (int32_t)(gyr_cross_ss[0] & 0x7F);
    if (BST_GET_VAL_BIT(gyr_cross_ss[0], 6)) {
        // signed bit
        istate->gyro_info.signed_gyr_cross_sense_coef -= 128;
    }

    BMI26X_INST_LOG(MED, istate->owner, "gyro cross ss:%d %d", (uint8_t) (gyr_cross_ss[0] & 0x7F),
                    istate->gyro_info.signed_gyr_cross_sense_coef);

    return rc;
}

//TODO: check, it seems that framework takes care of flush? SNS_STD_MSGID_SNS_STD_FLUSH_REQ
//OPTIM
static
sns_rc bmi26x_hal_fifo_prepare_4_cos(sns_sensor_instance *this, bmi26x_hw_cfg_ctx_t hw_cfg_ctx)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t                     ff_sensors_need_to_invalidate = 0;
    uint8_t                     ff_sensors_en_curr_dur_transit = 0;
    bool                        ff_flush_needed = false;
    uint8_t                     regv;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

#if BMI26X_CONFIG_ENABLE_DAE
    bool ds_ag_need_stop = false;
    bool ds_temp_need_stop = false;
#endif

    UNUSED_VAR(hw_cfg_ctx);

    ff_sensors_need_to_invalidate = bmi26x_hal_fifo_get_sensors_to_invalidate(istate);

    //if (ff_sensors_to_invalidate)
    if (ff_sensors_need_to_invalidate & (~fifo_info->ff_sensors_flush_done_before_invalid)) {
        ff_flush_needed = true;

        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_1, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        ff_sensors_en_curr_dur_transit = BST_GET_VAL_BITBLOCK(regv, 5, 7);
        if (ff_sensors_need_to_invalidate & BMI26X_ACCEL) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 1);
        }

        if (ff_sensors_need_to_invalidate & BMI26X_GYRO) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 2);
        }

        if (ff_sensors_need_to_invalidate & BMI26X_MAG) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 0);
        }

        if (BST_GET_VAL_BITBLOCK(regv, 5, 7) != ff_sensors_en_curr_dur_transit) {
            regv = BST_SET_VAL_BITBLOCK(regv, 5, 7, ff_sensors_en_curr_dur_transit);
            rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_1, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            BMI26X_INST_LOG(HIGH, istate->owner, "sensors disabled during transition: %d",
                            ff_sensors_need_to_invalidate);
        }
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ sensors not disabled during transition: %d,%d",
                        ff_sensors_need_to_invalidate,
                        fifo_info->ff_sensors_flush_done_before_invalid);
    }

    fifo_info->ff_sensors_flush_done_before_invalid = 0;

    //if overall wml goes lower
    if ((fifo_info->ff_wml_bytes_req < fifo_info->ff_wml_bytes_curr)
#if BMI26X_EXPLICIT_CLEAR_FLUSH_PENDING_AFTER_FLUSHING
                    &&
                    (hw_cfg_ctx != HW_CONFIG_CTX_ON_DEF_FLUSH_PENDING)
#endif
                    ) {
        ff_flush_needed = true;
    }

    if (ff_flush_needed) {
        if (fifo_info->ff_flush_in_proc) {
            BMI26X_INST_LOG(HIGH, istate->owner, "ff_flush_in_proc trigger: %d", fifo_info->ff_flush_trigger);
        } else {
#if  BMI26X_CONFIG_ENABLE_DAE
            // dae mode, dae alrady handle this
            uint8_t sensor_stop = (uint8_t) (BMI26X_ACCEL | BMI26X_GYRO);

            if (bmi26x_dae_if_available(this)) {
                if (istate->config_step == BMI26X_CONFIG_IDLE &&
                    bmi26x_dae_if_stop_streaming(this, sensor_stop)) {
                    ds_ag_need_stop = true;
                }
            } else {
                // NON-DAE
                BMI26X_INST_LOG(MED, istate->owner, "#flush# start to flush witout dae");
                bmi26x_hal_fifo_drain(istate, false, BMI26X_FIFO_FLUSH_TRIGGER_HW_CHANGE);
            }
#else
            bmi26x_hal_fifo_drain(istate, false, BMI26X_FIFO_FLUSH_TRIGGER_HW_CHANGE);
#endif
        }
    }

#if  BMI26X_CONFIG_ENABLE_DAE
    if (istate->sensor_temp_info.timer_itvl_changed) {
        ds_temp_need_stop = bmi26x_dae_if_stop_streaming(this, BMI26X_SENSOR_TEMP);
    }

    if (ds_ag_need_stop || ds_temp_need_stop) {
        istate->fifo_info.ff_flush_pending_on_sensors = (ds_ag_need_stop | (ds_temp_need_stop << 1));
        istate->fifo_info.ff_flush_in_proc = true;
        istate->config_step = BMI26X_CONFIG_STOPPING_STREAM;
        BMI26X_INST_LOG(HIGH, istate->owner, "flush on sensors:0x%x",
                        istate->fifo_info.ff_flush_pending_on_sensors);
    }
#endif

    fifo_info->ff_sensors_to_invalidate = ff_sensors_need_to_invalidate;

    return SNS_RC_SUCCESS;
}



static
int32_t bmi26x_hal_fifo_get_wml_compensated(
    struct bst_sbus_spec    *bus,
    uint32_t                ff_wml_bytes,
    uint16_t                odr_acc,
    uint16_t                odr_gyr,
    uint16_t                odr_mag)
{
    int32_t                     ff_wml_bytes_compensated = 0;
    uint32_t                    bits;
    uint32_t                    traffic_delay_us;
    uint16_t                    fc_new_acc = 0;
    uint16_t                    fc_new_gyr = 0;
    uint16_t                    fc_new_mag = 0;
    uint16_t                    fc_new_max;

    if (bus->type) {
        //SPI
        bits = (((ff_wml_bytes + 1) * (8 + BMI26X_CONFIG_SEE_SPI_BYTE_XFER_WAIT_CYCLES)) + 2);
    } else {
        bits = (30 + (ff_wml_bytes * 9));
    }

    traffic_delay_us = bits * 1e6 / bus->clk_rate;

    traffic_delay_us += BMI26X_CONFIG_SEE_ASYNC_COM_DELAY_US;

    if (odr_acc > 0) {
        fc_new_acc = BST_CEIL_P_BUF(
                         (((traffic_delay_us) * odr_acc) * 1e-6), 0.2);
    }

    if (odr_gyr > 0) {
        fc_new_gyr = BST_CEIL_P_BUF(
                         (((traffic_delay_us) * odr_gyr) * 1e-6), 0.2);
    }

    if (odr_mag > 0) {
        fc_new_mag = BST_CEIL_P_BUF(
                         (((traffic_delay_us) * odr_mag) * 1e-6), 0.2);
    }

    fc_new_max = fc_new_acc;

    fc_new_max = (fc_new_max > fc_new_gyr) ? fc_new_max : fc_new_gyr;
    fc_new_max = (fc_new_max > fc_new_mag) ? fc_new_max : fc_new_mag;

    ff_wml_bytes_compensated = (int32_t)ff_wml_bytes
                               - (fc_new_acc * BMI26X_FF_DATA_LEN_ACC)
                               - (fc_new_gyr * BMI26X_FF_DATA_LEN_GYR)
                               - (fc_new_mag * BMI26X_FF_DATA_LEN_MAG)
                               - fc_new_max;

    if (ff_wml_bytes_compensated <= 0) {
        ff_wml_bytes_compensated = ff_wml_bytes;
    }

    return ff_wml_bytes_compensated;
}


sns_rc bmi26x_hal_config_power_ctrl(bmi26x_instance_state  *istate)
{
    uint8_t                     regv = 0;

    //TODOMAG
    if (SNS_RC_SUCCESS != bmi26x_dev_pwr_conf(istate, 0, 0, 0)) {
    }
    BMI26X_INST_LOG(LOW, istate->owner, "configure step:%d", istate->config_step);

    if (istate->hw_mod_needed & BMI26X_GYRO) {
        regv = BST_SET_VAL_BIT(regv, 1);
    }

    if (istate->hw_mod_needed & BMI26X_ACCEL) {
        regv = BST_SET_VAL_BIT(regv, 2);
    }

    if (istate->hw_mod_needed & BMI26X_SENSOR_TEMP) {
        regv = BST_SET_VAL_BIT(regv, 3);
    }

    BMI26X_INST_LOG(MED, istate->owner, "hwm: 0x%02x, regv:0x%02x",
                    istate->hw_mod_needed, regv);


    return bmi26x_dev_pwr_ctrl(istate, 0, 3, regv);
}

//OPTIM
sns_rc bmi26x_hal_config_acc(sns_sensor_instance *this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t                     regv;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    if (istate->hw_mod_needed & BMI26X_ACCEL) {
        //TODO: temperature doesn't need accel
        if ( (fifo_info->publish_sensors & BMI26X_ACCEL)
                || ((fifo_info->publish_sensors & BMI26X_SENSOR_TEMP)
                    && !(fifo_info->publish_sensors & BMI26X_GYRO))
                //TODOMAG
           ) {
            rc = bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_ACC_NORMAL);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

        regv = istate->accel_info.range_req;

        BMI26X_INST_LOG(MED, istate->owner, "config_acc range: %d,%d", istate->accel_info.range_curr,
                        istate->accel_info.range_req);

        if (istate->accel_info.range_curr != istate->accel_info.range_req) {
            rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_RANGE, 0, 7, regv);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            istate->accel_info.range_curr = istate->accel_info.range_req;
            // FIXME
            istate->accel_info.ff_cfg_ctx.cfg_range_changed = 1;
        }

        bmi26x_sstvt_t acc_ssvt_array[] = {
            /* in the unit of micro-g/lsb */
            BMI26X_ACCEL_SSTVT_2G,
            BMI26X_ACCEL_SSTVT_4G,
            BMI26X_ACCEL_SSTVT_8G,
            BMI26X_ACCEL_SSTVT_16G,
        };

        if (istate->fac_test_in_progress &
                (istate->fac_test_info.fac_test_sensor & BMI26X_ACCEL)) {
            uint8_t res_idx = 0;
            // get current range:
            rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_ACC_RANGE, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
            res_idx = regv & 0x03;
            istate->accel_info.sstvt_curr = roundf(acc_ssvt_array[res_idx] * G *
                                                   (istate->accel_info.sstate->scale_factor * 1e-6));
        } else {
            istate->accel_info.sstvt_curr = roundf(acc_ssvt_array[istate->accel_info.sstate->resolution_idx] * G *
                                                   (istate->accel_info.sstate->scale_factor * 1e-6));
        }

        if (fifo_info->publish_sensors & BMI26X_ACCEL) {
            if (istate->accel_info.odr_req != BMI26X_REGV_ODR_OFF) {
                regv = 0;
                regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, istate->accel_info.odr_req);
                regv = BST_SET_VAL_BITBLOCK(regv, 4, 6, BMI26X_CONFIG_ACC_BWP);
                regv = BST_SET_VAL_BIT(regv, 7);

                rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_CONF, 0, 7, regv);
                BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

                // save the last odr
                istate->last_ff_ts_ctx.regv_last_odr_acc = istate->accel_info.odr_curr;
                istate->accel_info.odr_curr = istate->accel_info.odr_req;
#if BMI26X_CONFIG_ENABLE_GROUP_DELAY
                const uint32_t us_filter_delay_us[] = {
                        80000,     //80ms@12.5hz
                        40000,    //40ms@25hz
                        20500,    //20.5ms@50hz
                        10500,    //10.5ms@100hz
                        5400,    //5.4ms@200hz
                        2000,    //2ms@400hz
                        1300,    //1.3ms@800hz
                        600,    //0.6ms@1600hz
                };
                float filter_delay_in_tick  = 0;
                if (istate->accel_info.odr_curr >= BMI26X_REGV_ODR_12_5HZ) {
                    filter_delay_in_tick = bmi26x_convert_us2ticks(us_filter_delay_us[istate->accel_info.odr_curr - BMI26X_REGV_ODR_12_5HZ]);
                    istate->accel_info.filter_delay_in_tick = (uint32_t) filter_delay_in_tick;
                }
#else
                istate->accel_info.filter_delay_in_tick = 0;
#endif

            }
        } else {
            istate->last_ff_ts_ctx.ts_last_ff_batch_acc = 0;
        }
    } else {
        //rc = bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_ACC_SUSPED);
        istate->accel_info.range_curr = BMI26X_REGV_RANGE_ACC_PM8G;
        istate->accel_info.odr_curr = BMI26X_REGV_ODR_OFF;
        istate->last_ff_ts_ctx.ts_last_ff_batch_acc = 0;
    }

    return SNS_RC_SUCCESS;
}

//OPTIM
static
sns_rc bmi26x_hal_config_gyro(sns_sensor_instance *this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t                     regv;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    uint8_t                     range_val_now = 0;

    if (istate->hw_mod_needed & BMI26X_GYRO) {
        if (fifo_info->publish_sensors & BMI26X_GYRO) {
            rc = bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_GYR_NORMAL);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
        regv = istate->gyro_info.range_req;
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_GYR_RANGE, &range_val_now, 1);
        BMI26X_INST_LOG(MED, istate->owner, "config_gyro range: %d,%d %d", istate->gyro_info.range_curr,
                        istate->gyro_info.range_req, range_val_now);
        if (range_val_now != regv) {
            regv = istate->gyro_info.range_req;

            rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_GYR_RANGE, 0, 2, regv);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            istate->gyro_info.range_curr = istate->gyro_info.range_req;
        }

        bmi26x_gyro_sstvt gyro_ssvt_array[] = {
            BMI26X_GYRO_SSTVT_125DPS,
            BMI26X_GYRO_SSTVT_250DPS,
            BMI26X_GYRO_SSTVT_500DPS,
            BMI26X_GYRO_SSTVT_1000DPS,
            BMI26X_GYRO_SSTVT_2000DPS,
        };

        /*if ((istate->fac_test_in_progress) &&
                (istate->fac_test_info.fac_test_sensor & BMI26X_GYRO)) {
            // get current range:
            regv = 0;
            uint8_t res_idx = 0;

            rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_GYR_RANGE, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            res_idx = regv & 0x07;
            istate->gyro_info.sstvt_curr = roundf(gyro_ssvt_array[4 - res_idx] *
                                                  (istate->gyro_info.sstate->scale_factor * 1e-6));
        } else */{
            istate->gyro_info.sstvt_curr = roundf(gyro_ssvt_array[istate->gyro_info.sstate->resolution_idx] *
                                                  (istate->gyro_info.sstate->scale_factor * 1e-6));
        }

        if (fifo_info->publish_sensors & BMI26X_GYRO) {
            if (istate->gyro_info.odr_curr != istate->gyro_info.odr_req) {
                if (istate->gyro_info.odr_req != BMI26X_REGV_ODR_OFF) {
                    regv = 0;
                    regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, istate->gyro_info.odr_req);
                    regv = BST_SET_VAL_BITBLOCK(regv, 4, 5, BMI26X_CONFIG_GYR_BWP);
#if  BMI26X_CONFIG_ENABLE_ENABLE_GYRO_HIGH_PERFORMANCE
                    regv = BST_SET_VAL_BITBLOCK(regv, 6, 7, 0x03);
#else
                    regv = BST_SET_VAL_BIT(regv, 7);
#endif

                    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_GYR_CONF, 0, 7, regv);
                    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

                    // fly change
                    // save the last odr
                    istate->last_ff_ts_ctx.regv_last_odr_gyr = istate->gyro_info.odr_curr;
                    istate->gyro_info.odr_curr = istate->gyro_info.odr_req;
#if BMI26X_CONFIG_ENABLE_GROUP_DELAY
                const uint32_t us_filter_delay_us[] = {
                        80000,     //80ms@12.5hz
                        40000,    //40ms@25hz
                        20500,    //20.5ms@50hz
                        10800,    //10.8ms@100hz
                        5970,    //5.97ms@200hz
                        3550,    //3.55ms@400hz
                        2340,    //2.34ms@800hz
                        970,     //0.97ms@1600hz
                };
                float filter_delay_in_tick  = 0;
                if (istate->gyro_info.odr_curr >= BMI26X_REGV_ODR_12_5HZ) {
                    filter_delay_in_tick = bmi26x_convert_us2ticks(us_filter_delay_us[istate->gyro_info.odr_curr - BMI26X_REGV_ODR_12_5HZ]);
                    istate->gyro_info.filter_delay_in_tick = (uint32_t) filter_delay_in_tick;
                }
#else
                istate->gyro_info.filter_delay_in_tick = 0;
#endif
                }
            }

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
            if (istate->gyro_info.odr_req != BMI26X_REGV_ODR_OFF) {
                uint8_t downsample_sw_factor_old = istate->gyro_info.downsample_sw_factor;
                if (BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_req].odr >= istate->gyro_info.sample_rate_req) {
                    istate->gyro_info.downsample_sw_factor = BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_req].odr
                            / istate->gyro_info.sample_rate_req;
                } else {
                    istate->gyro_info.downsample_sw_factor = 1;
                }

                BMI26X_INST_LOG(MED, istate->owner, "ds_sw: %d", istate->gyro_info.downsample_sw_factor);

                if (downsample_sw_factor_old != istate->gyro_info.downsample_sw_factor) {
                    istate->gyro_info.downsample_sw_cnt = 0;;
                }
            }
#endif
        } else {
            // reset, set to default
            regv = 0xA9;
            rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_GYR_CONF, 0, 7, regv);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
            istate->gyro_info.odr_curr = BMI26X_REGV_ODR_OFF;
            istate->last_ff_ts_ctx.ts_last_ff_batch_gyr = 0;
        }
    } else {
        //rc = bmi26x_hal_pwr_cmd_handler(istate, BMI26X_REGV_CMD_GYR_MODE_SUSP);
        istate->gyro_info.range_curr = BMI26X_REGV_RANGE_GYR_PM2000DPS;
        //istate->gyro_info.odr_curr = BMI26X_REGV_ODR_OFF;
        istate->last_ff_ts_ctx.ts_last_ff_batch_gyr = 0;
    }

    return SNS_RC_SUCCESS;
}

//OPTIM
static
sns_rc bmi26x_hal_config_shared_fifo(sns_sensor_instance *this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    struct bst_sbus_spec        bus_spec;
    uint8_t                     regv;
    uint8_t                     si_buf[2];
    uint16_t                    ff_wml_bytes;

    uint16_t                    odr_acc;
    uint16_t                    odr_gyr;
    uint16_t                    odr_mag;
    uint8_t                     ff_sensors_en_orig = fifo_info->ff_sensors_en_curr;

    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_FIFO_CONFIG_0, 0, 1, 2);

    regv = 0;

    if (fifo_info->publish_sensors & BMI26X_ACCEL) {
        regv = BST_SET_VAL_BIT(regv, 6);
    }

    if (fifo_info->publish_sensors & BMI26X_GYRO) {
        regv = BST_SET_VAL_BIT(regv, 7);
    }

    //TODOMAG

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        regv = 0;
    }
#endif

    if (fifo_info->publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG))  {
        regv = BST_SET_VAL_BIT(regv, 4);
    }

    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_FIFO_CONFIG_1, 0, 7, regv);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    istate->regv_cache.regv_fifo_config_1 = regv;

    //INSERT_TRACE_POINT4_T(bmi26x, 'F', 'F', fifo_info->ff_sensors_en_curr, fifo_info->ff_sensors_en_req);

#if BMI26X_CONFIG_ENABLE_DEBUG_TEST
    if (!(fifo_info->ff_sensors_en_curr & BMI26X_ACCEL)) {
        if (fifo_info->ff_sensors_en_req & BMI26X_ACCEL) {
            g_bmi26x_cnt_session_acc++;
        }
    }

    if (!(fifo_info->ff_sensors_en_curr & BMI26X_GYRO)) {
        if (fifo_info->ff_sensors_en_req & BMI26X_GYRO) {
            g_bmi26x_cnt_session_gyr++;
        }
    }
#endif

    fifo_info->ff_sensors_en_curr = fifo_info->publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        fifo_info->ff_sensors_en_curr = 0;
    }
#endif

    //we have calls of config_acc() and config_gyr() ahead of this
    bmi26x_hal_fifo_update_curr_masters(istate);

    //config wml in hw
    bus_spec.type = (SNS_BUS_SPI == istate->com_port_info.com_config.bus_type);
    bus_spec.clk_rate = (istate->com_port_info.com_config.min_bus_speed_KHz +
                         istate->com_port_info.com_config.max_bus_speed_KHz) * 500; //500: / 2 * 1000

    odr_acc = (uint16_t) (BMI26X_REGV_ODR_MAP[istate->accel_info.odr_req].odr);
    odr_gyr = (uint16_t) (BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_req].odr);
    odr_mag = 0;    //TODOMAG
    ff_wml_bytes = bmi26x_hal_fifo_get_wml_compensated(&bus_spec, fifo_info->ff_wml_bytes_req, odr_acc, odr_gyr, odr_mag);

    //INSERT_TRACE_POINT4_T(bmi26x, 'F', fifo_info->ff_wml_bytes_req & 0xff, (fifo_info->ff_wml_bytes_req >> 8)&0xff, ff_wml_bytes);
    if (ff_wml_bytes > 0) {
        si_buf[0] = ff_wml_bytes & 0xff;
        si_buf[1] = (ff_wml_bytes >> 8) & 0xff;
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_FIFO_WTM_0, si_buf, 2);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    fifo_info->ff_wml_bytes_curr = ff_wml_bytes;


    BMI26X_INST_LOG(MED, istate->owner, "ff_wml: %d", ff_wml_bytes);


    if (fifo_info->ff_sensors_en_curr) {
        if (bmi26x_hal_update_couple_ts_host_and_dev_rt(istate) != SNS_RC_SUCCESS) {
        }
        BMI26X_INST_LOG(MED, istate->owner, "ts_pair <%u,0x%x>",
                        BMI26X_SYS_TIME_LH(istate->ts_pair_sys_dev.ts_sys),
                        istate->ts_pair_sys_dev.ts_dev);

        istate->ts_pair_sys_dev_4_ts_res_est = istate->ts_pair_sys_dev;


        fifo_info->ff_tm_info.boost_read_size = 1;
#if BMI26X_CONFIG_ENABLE_TS_REF_SPECULATION
        fifo_info->devi_est_irq = 0;
#endif

        fifo_info->ff_itvl_ticks_est_masters = istate->ts_hw_res_ticks_per_bit * (1 << (BMI26X_REGV_ODR_1600HZ -
                                               fifo_info->ff_master_odr_curr + 4));
        fifo_info->ff_itvl_ticks_est_acc = istate->ts_hw_res_ticks_per_bit * (1 << (BMI26X_REGV_ODR_1600HZ -
                                           istate->accel_info.odr_curr + 4));
        fifo_info->ff_itvl_ticks_est_gyr = istate->ts_hw_res_ticks_per_bit * (1 << (BMI26X_REGV_ODR_1600HZ -
                                           istate->gyro_info.odr_curr + 4));

    } else {
        if (ff_sensors_en_orig) {
            if (bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_FIFO_FLUSH) != SNS_RC_SUCCESS) {
            }
        }

        istate->ts_pair_sys_dev.avail_1st = 0;
    }

    istate->accel_info.ff_wml_curr = istate->accel_info.ff_wml_req;
    istate->gyro_info.ff_wml_curr = istate->gyro_info.ff_wml_req;

    return SNS_RC_SUCCESS;
}

/**
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */

static
void bmi26x_hal_config_polling_timer_4_temp(sns_sensor_instance *const this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;

    BMI26X_INST_LOG(LOW, this, "sample interval:%u. timer is active:%d",
            (uint32_t) istate->sensor_temp_info.sampling_intvl_curr,
                    (istate->sensor_temp_info.timer_is_active | (istate->sensor_temp_info.timer_itvl_changed << 1)));

    if (istate->sensor_temp_info.sampling_intvl_req > 0) {
        if (!istate->sensor_temp_info.timer_is_active || istate->sensor_temp_info.timer_itvl_changed) {
            if (istate->timer_data_stream == NULL) {
                BMI26X_INST_LOG(MED, this, "timer data stream created");
                istate->stream_mgr->api->create_sensor_instance_stream(
                    istate->stream_mgr,
                    this,
                    istate->timer_suid,
                    &istate->timer_data_stream);
            }
            bmi26x_hal_start_sensor_temp_polling_timer(this);
        } else {
            BMI26X_INST_LOG(MED, this, "timer reconfig ignored");
        }
    } else if (istate->sensor_temp_info.timer_is_active) {
        istate->sensor_temp_info.timer_is_active = false;
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->timer_data_stream);
        BMI26X_INST_LOG(MED, this, "timer removed");
    }
}

void
bmi26x_hal_stop_tempetature_timer(sns_sensor_instance  *const this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *)this->state->state;
    if ((NULL != istate->timer_data_stream) &&
            (istate->sensor_temp_info.timer_is_active)) {
        BMI26X_INST_LOG(MED, this, "timer data stream removed");
        istate->stream_mgr->api->remove_stream(istate->stream_mgr, istate->timer_data_stream);
        istate->timer_data_stream = NULL;
        istate->sensor_temp_info.timer_is_active = false;
    }
}

void bmi26x_send_fac_cal_event(
    sns_sensor_instance *const  instance,
    const struct bmi26x_state   *sstate)
{
    bmi26x_instance_state *istate = NULL;
    const sns_sensor_uid            *suid = &(sstate->my_suid);
    sns_cal_event new_calibration_event = sns_cal_event_init_default;
    float bias_data[] = {0, 0, 0};
    float comp_matrix_data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (BST_ASSERT_POINT(instance)) {
        return ;
    }

    istate = (bmi26x_instance_state*)instance->state->state;
    UNUSED_VAR(istate);

    bias_data[0] = sstate->fac_cal_bias[0] * 1.0f /
                   sstate->scale_factor; // values in istate are usually from registry or from factory test routine
    bias_data[1] = sstate->fac_cal_bias[1] * 1.0f / sstate->scale_factor;
    bias_data[2] = sstate->fac_cal_bias[2] * 1.0f / sstate->scale_factor;
    comp_matrix_data[0] = sstate->fac_cal_corr_mat.data[0];
    comp_matrix_data[1] = sstate->fac_cal_corr_mat.data[1];
    comp_matrix_data[2] = sstate->fac_cal_corr_mat.data[2];
    comp_matrix_data[3] = sstate->fac_cal_corr_mat.data[3];
    comp_matrix_data[4] = sstate->fac_cal_corr_mat.data[4];
    comp_matrix_data[5] = sstate->fac_cal_corr_mat.data[5];
    comp_matrix_data[6] = sstate->fac_cal_corr_mat.data[6];
    comp_matrix_data[7] = sstate->fac_cal_corr_mat.data[7];
    comp_matrix_data[8] = sstate->fac_cal_corr_mat.data[8];

    pb_buffer_arg buff_arg_bias = (pb_buffer_arg) {
        .buf = &bias_data, .buf_len = ARR_SIZE(bias_data)
    };
    pb_buffer_arg buff_arg_comp_matrix = (pb_buffer_arg) {
        .buf = &comp_matrix_data, .buf_len = ARR_SIZE(comp_matrix_data)
    };

    new_calibration_event.bias.funcs.encode = pb_encode_float_arr_cb;
    new_calibration_event.bias.arg = &buff_arg_bias;
    new_calibration_event.comp_matrix.funcs.encode = pb_encode_float_arr_cb;
    new_calibration_event.comp_matrix.arg = &buff_arg_comp_matrix;
    new_calibration_event.status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    BMI26X_INST_LOG(MED, instance, "bmi cal evt: %d <%d %d %d> ver: %d", sstate->sensor,
                    (int)(bias_data[0] * 1000), (int)(bias_data[1] * 1000), (int)(bias_data[2] * 1000),
                    sstate->fac_cal_version);

    pb_send_event(instance,
                  sns_cal_event_fields,
                  &new_calibration_event,
                  sns_get_system_time(),
                  SNS_CAL_MSGID_SNS_CAL_EVENT,
                  suid);
}

sns_rc bmi26x_hal_power_mode_assert(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bool need_low_power = false;

    // check current working mode
    need_low_power = bmi26x_hal_confirm_low_power(istate);

    if (need_low_power) {
        rc = bmi26x_hal_config_power_mode(istate, BMI26X_POWER_MODE_LOW);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        istate->accel_info.in_lpm = 1;
        istate->sbus_in_normal_mode = 0;

        // double tap need acc hp in low power mode
        if (istate->int_en_flags_curr.bits.dbtap) {
            rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_CONF, 7, 7, 1);
        }
    } else {
    }

    // check latch, BMI26X_REGA_USR_INT_LATCH
    if (istate->int_en_flags_curr.bits.lowg ||
            istate->int_en_flags_curr.bits.md
            ||
            istate->int_en_flags_curr.bits.dbtap
            ) {
        /*rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH,
                                              0, 0, BMI26X_REGV_INT_LATCH_EN);*/
    } else {
        //rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH, 0, 0, BMI26X_REGV_INT_LATCH_DISABLE);
    }
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // always
    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_INT_LATCH,
                                          0, 0, BMI26X_REGV_INT_LATCH_EN);


    return rc;
}


void bmi26x_hal_fifo_flush(sns_sensor_instance *const inst)
{
    //TOCHECK
    bmi26x_instance_state *istate = (bmi26x_instance_state*) inst->state->state;
    bool dae_flush_done = false;

#if BMI26X_CONFIG_ENABLE_DAE
    dae_flush_done = bmi26x_dae_if_flush_hw(inst, &istate->dae_if.ag);
#endif

    if (!dae_flush_done) {
        BMI26X_INST_LOG(HIGH, inst, "fifo_flush status: %x",
                (istate->ff_flush_client_req) |
                (istate->fifo_info.ff_flush_in_proc << 1)|
                (istate->fifo_info.ff_flush_trigger << 2) |
                (istate->hw_config_pending << 3));

        bmi26x_hal_fifo_drain(istate, false, BMI26X_FIFO_FLUSH_TRIGGER_HW_CHANGE);
    }
}




#if BMI26X_CONFIG_ENABLE_CRT
static sns_rc bmi26x_hal_crt_assert(sns_sensor_instance  *inst)
{
    // exit from island
    bmi26x_hal_inst_exit_island(inst);
    return bmi26x_hal_crt_evaluate_crt(inst);
}
#endif

#if  BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER

void bmi26x_restart_hb_timer(sns_sensor_instance *const inst, bool en, sns_time ts_timeout)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*) inst->state->state;

    if (en) {
        if (istate->hb_cfg_info.timer_enable == false) {

            if (istate->hb_cfg_info.timer_heart_beat_data_stream == NULL) {
                sns_rc rc = istate->stream_mgr->api->create_sensor_instance_stream(
                    istate->stream_mgr,
                    inst,
                    istate->timer_suid,
                    &istate->hb_cfg_info.timer_heart_beat_data_stream);
                if (rc != SNS_RC_SUCCESS) {
                    return ;
                }
            }

            if (!bmi26x_hal_start_timer(inst, istate->hb_cfg_info.timer_heart_beat_data_stream,
                        true, ts_timeout)) {
            }
            // update timer
            istate->hb_cfg_info.ts_data_event = bmi26x_get_sys_tick();
            istate->hb_cfg_info.timer_enable = 1;
            istate->hb_cfg_info.heart_attack_cnt = 0;

            BMI26X_INST_LOG(MED, inst, "#HB# restart_hb_timer: now:%X exp:%X, over stream:%p",
                    (uint32_t) sns_get_system_time(),
                    (uint32_t) ts_timeout,
                    istate->hb_cfg_info.timer_heart_beat_data_stream);

        } else {
            BMI26X_INST_LOG(MED, inst, "time already start: the latest timeout @<%u %u>",
                    (uint32_t)(istate->hb_cfg_info.ts_data_event >> 32), (uint32_t)(istate->hb_cfg_info.ts_data_event));
        }
    } else {
        // REMOVE
        if (istate->hb_cfg_info.timer_heart_beat_data_stream != NULL) {
            sns_sensor_util_remove_sensor_instance_stream(inst, &istate->hb_cfg_info.timer_heart_beat_data_stream);
            istate->hb_cfg_info.timer_heart_beat_data_stream = NULL;
        }
        istate->hb_cfg_info.timer_enable = 0;
    }
}

void bmi26x_hal_prepare_hb_timer(bmi26x_instance_state *istate)
{
#if 0
    if (istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO)) {
        //XXX need to change the timeout interval in this version
        bmi26x_restart_hb_timer(istate->owner, true, sns_convert_ns_to_ticks(BMI26X_TS_DEFAULT_TIMEOUT_MS * 1000 * 1000));
    } else {
        // remove hb timer
        bmi26x_restart_hb_timer(istate->owner, false, 0);
        return ;
    }
#else
    if ((istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO)) &&
            !istate->fac_test_in_progress) {
        //XXX need to change the timeout interval in this version

        uint32_t wml_curr = istate->fifo_info.ff_wml_bytes_curr / 7;
        bmi26x_regv_odr_t odr_master = istate->fifo_info.ff_master_odr_curr;
        float sr_master = BMI26X_REGV_ODR_MAP[odr_master].odr;
        uint32_t hb_check_ff_cnt = 0;
        uint32_t hb_check_ff_cnt_adj = 0;
        sns_time hb_check_itvl_adj = 0;

        /*if (istate->fifo_info.flush_only)  {
            hb_check_itvl = BMI160_TS_DEFAULT_LONG_ATTACK_MS;
        } else*/ {
            // TODO need to add headroom time for the timer because the fifo frame stabilization before new odr take effect
            hb_check_ff_cnt = 2 * wml_curr;
        }

        if (wml_curr < 6) {
            if (odr_master >= BMI26X_REGV_ODR_400HZ) {
                // 10ms / 2.5 = 4
                hb_check_ff_cnt_adj = SNS_MAX(hb_check_ff_cnt, (wml_curr + 6 + 4));
            } else if (odr_master >= BMI26X_REGV_ODR_50HZ) {
                //hb_check_itvl_adj = SNS_MAX(hb_check_itvl, (wml_curr + 6) * (1000.0f/sr_master));
                hb_check_ff_cnt_adj = SNS_MAX(hb_check_ff_cnt, (wml_curr + 6));
            } else if (odr_master >= BMI26X_REGV_ODR_12_5HZ){
                hb_check_ff_cnt_adj = SNS_MAX(hb_check_ff_cnt, (wml_curr + 4));
            } else {
            }

            if ((istate->fifo_info.ff_tm_info.fc_accum_curr_acc == 0 &&
                    istate->fifo_info.ff_tm_info.fc_accum_curr_gyr == 0) &&
                    istate->hb_cfg_info.heart_attack_cnt) {
                // 150ms due to cfg download
                hb_check_ff_cnt_adj += (150 * sr_master / 1000);
            }

        } else if (wml_curr > 0) {
            hb_check_ff_cnt_adj = hb_check_ff_cnt;
        } else {
            hb_check_ff_cnt_adj = 0;
        }

        hb_check_itvl_adj = hb_check_ff_cnt_adj * (1000.0f / sr_master) * istate->ticks_in_1ms;

        if (((hb_check_itvl_adj > 0) &&
                (hb_check_itvl_adj != istate->hb_cfg_info.heart_beat_timeout)) ||
                (odr_master != istate->hb_cfg_info.odr_master_on_hb)) {
            // remove hb timer
            bmi26x_restart_hb_timer(istate->owner, false, 0);

            istate->hb_cfg_info.heart_beat_timeout = hb_check_itvl_adj;
        }

        BMI26X_INST_LOG(MED, istate->owner, "cfg hb: ts:%d, ts.tick:%u ticks / ff.cnt.adj:%d odr.master:%d, wml:%d",
                        (uint32_t) hb_check_itvl_adj,
                        (uint32_t) istate->hb_cfg_info.heart_beat_timeout,
                        hb_check_ff_cnt_adj,
                        odr_master,
                        wml_curr);


        bmi26x_restart_hb_timer(istate->owner, true, (istate->hb_cfg_info.heart_beat_timeout * 10 / 8));
        istate->hb_cfg_info.odr_master_on_hb = odr_master;
    } else {
        // remove hb timer
        bmi26x_restart_hb_timer(istate->owner, false, 0);
        return ;
    }
#endif
}

void bmi26x_hal_handle_hb_timer_event(bmi26x_instance_state *istate)
{
    sns_sensor_event        *event;
    //sns_time              ts_expect_time_out_ticks;
    sns_time                ts_assert_time_out_ticks;
    uint8_t                 ff_need_flush = 0;
    bool                    cfg_is_available = true; //bmi26x_hal_check_cfg_available(inst);

    if (istate->hb_cfg_info.timer_heart_beat_data_stream == NULL) {
        return ;
    }


    //ts_assert_time_out_ticks = BMI160_TS_DEFAULT_LONG_ATTACK_MS;
    //ts_assert_time_out_ticks = ts_assert_time_out_ticks * 1000 * 1000;
    //ts_assert_time_out_ticks = sns_convert_ns_to_ticks(ts_assert_time_out_ticks);

    ts_assert_time_out_ticks = istate->hb_cfg_info.heart_beat_timeout;

    if (istate->hb_cfg_info.timer_heart_beat_data_stream != NULL) {
        event = istate->hb_cfg_info.timer_heart_beat_data_stream->api->peek_input(
                istate->hb_cfg_info.timer_heart_beat_data_stream);

        while(NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
            sns_timer_sensor_event timer_event;
            if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    BMI26X_INST_LOG(MED, istate->owner, "#HB# expect timeout:%d %d",
                                istate->fifo_info.ff_master_odr_curr,
                                istate->fifo_info.ff_wml_bytes_curr);
                    // update
                    int64_t ts_delta = bmi26x_get_sys_tick() - istate->hb_cfg_info.ts_data_event;
                    // check
                    if (ts_delta <= 0) {
                    } else {
                        if ((uint32_t)ts_delta > ts_assert_time_out_ticks) {
                            if ((uint32_t)ts_delta < 2 * ts_assert_time_out_ticks) {
                                // ignore
                            } else {
                                ff_need_flush = 1;
                            }
                            // handle
                            BMI26X_INST_LOG(HIGH, istate->owner, "#HB# attack due to fifo event timeout:%u - %u = %u > %u",
                                (uint32_t)bmi26x_get_sys_tick(),
                                (uint32_t)istate->hb_cfg_info.ts_data_event,
                                (uint32_t)(ts_delta),
                                (uint32_t)ts_assert_time_out_ticks);

                        } else {
                            // check on the next time
                            BMI26X_INST_LOG(MED, istate->owner, "#HB# hb delta:%d", (uint32_t) (ts_delta));
                        }
                    }

                    // check cfg status
                    cfg_is_available = bmi26x_hal_check_cfg_available(istate->owner);
                    ff_need_flush |= ((!cfg_is_available) << 1);
                }
            } else {
            }

            event = istate->hb_cfg_info.timer_heart_beat_data_stream->api->get_next_input(
                    istate->hb_cfg_info.timer_heart_beat_data_stream);
        }
    }

    if (ff_need_flush) {
        sns_rc rc = SNS_RC_SUCCESS;
        if ((istate->hb_cfg_info.heart_attack_cnt >= BMI26X_TS_DEFAULT_ATTACK_CNT_BEFORE_RESET) ||
                (!cfg_is_available)) {
            uint8_t ff_pb_ss = istate->fifo_info.publish_sensors;

            if (!cfg_is_available) {
                BMI26X_INST_LOG(MED, istate->owner, "cfg missing");
            } else {
                if (istate->hb_cfg_info.heart_attack_cnt < BMI26X_TS_DEFAULT_ATTACK_CNT_BEFORE_RESET) {
                    istate->hb_cfg_info.heart_attack_cnt ++;
                }
            }

            bmi26x_hal_inst_exit_island(istate->owner);
            rc = bmi26x_hal_reset_device(istate->owner, true);
            if (rc != SNS_RC_SUCCESS) {
                BMI26X_INST_LOG(MED, istate->owner, "reset failure:0x%x", rc);
            }

            // this is lead to HB keep do soft-reset and flush data due to the attack counter reset
            //bmi26x_restart_hb_timer(istate->owner, false, 0);
            //@@@@

            istate->fifo_info.publish_sensors = ff_pb_ss;
            //bmi26x_hal_reveal_client_config_wrapper(istate->owner, HW_CONFIG_CTX_ON_HEART_BEART_ATTACK, true);
            bmi26x_inst_assess_overall_req(istate);
            rc |= bmi26x_hal_reconfig_hw(istate->owner, HW_CONFIG_CTX_ON_HEART_BEART_ATTACK);
        } else {
            // just flush data the data
            istate->hb_cfg_info.heart_attack_cnt ++;
            // clear pending flush data
            istate->fifo_info.ff_flush_in_proc = false;
            bmi26x_hal_fifo_flush(istate->owner);
        }

        BMI26X_INST_LOG(MED, istate->owner, "#HB# attack count:%d, rc:%d",
        istate->hb_cfg_info.heart_attack_cnt, rc);
    }
}

#endif

static sns_rc bmi26x_hal_get_config_version(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    //BMI26X_CONFIG_ID_STRT_ADDR
    uint8_t read_buffer[2] = {0};
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;

    if (istate->cfg_ver_major == 0x00) {
        uint16_t ver_msb_lsb = 0xff;

        page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_CONFIG_ID, &page_offset,
                                               BMI26X_ADVANCED_FEATURE_INPUT);

        if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
            BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
            return SNS_RC_INVALID_TYPE;
        }

        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, read_buffer, 2);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        ver_msb_lsb = (uint16_t)(read_buffer[0] | (read_buffer[1]  << 8));
        istate->cfg_ver_major = ((ver_msb_lsb & 0x3C0) >> 6);
        istate->cfg_ver_minor = read_buffer[0] & 0x3F;
    }

    BMI26X_INST_LOG(HIGH, istate->owner, "configuration ver:0x%02x.%02x",
            istate->cfg_ver_major, istate->cfg_ver_minor);

    return rc;
}

//OPTIM
static
sns_rc bmi26x_hal_do_config_hw_now(sns_sensor_instance  *this)
{
    sns_rc                      rc = SNS_RC_SUCCESS;
    uint8_t  enable_fifo_stream = true;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    BMI26X_INST_LOG(MED, istate->owner, "<dchw> hw_mod_needed: %d ps: 0x%x",
                    istate->hw_mod_needed, fifo_info->publish_sensors);

    // clear advanced power save, take the sensor leave the power save mode
    rc = bmi26x_hal_config_power_ctrl(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // get configuration version
    rc |= bmi26x_hal_get_config_version(istate);

    // prepare gyro cross sensitivity
    rc |= bmi26x_hal_prepare_cross_sense(this);

#if BMI26X_CONFIG_ENABLE_CRT
    // exit from island
    bmi26x_hal_inst_exit_island(this);

    rc = bmi26x_hal_reconfig_crt_param(istate->owner,
                                       &istate->gyro_info.sstate->crt_gain);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
#endif

    //configure power, range odr of accel
    rc |= bmi26x_hal_config_acc(this);


#if BMI26X_CONFIG_ENABLE_OIS
    bmi26x_precheck_gyr_hw_req(istate);
#endif

    //config power, range odr of gyro
    rc |= bmi26x_hal_config_gyro(this);

    //TODOMAG

    //<config_shared_fifo>
    rc |= bmi26x_hal_config_shared_fifo(this);
    //</config_shared_fifo>

    rc |= bmi26x_hal_configure_fifo_downsampling_rate(istate);

    //<config_interrupts>
    enable_fifo_stream = fifo_info->publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);
    enable_fifo_stream |= istate->fac_test_in_progress;

    if (istate->irq_ready) {
        if (enable_fifo_stream
                || istate->int_en_flags_req.bits.md
                || istate->int_en_flags_req.bits.dbtap) {
            rc |= bmi26x_hal_config_int_output(this, true);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
    }

    // pedometer
#if BMI26X_CONFIG_ENABLE_PEDO
    bmi26x_hal_config_pedo(this);
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    bmi26x_hal_config_ois(this);
    bmi26x_recheck_gyr_hw_req(istate);
#else
    BMI26X_INST_LOG(MED, this, "not support ois sensor");
    rc = bmi26x_hal_en_ois(istate, true);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
#endif

    if (istate->int_en_flags_req.bits.md) {
        rc |= bmi26x_hal_set_md_config(this, true);
        rc |= bmi26x_hal_config_int_md(this,  true, false, (uint8_t)BMI26X_MD_CONFIG_HW_CFG);
    } else {
        if (istate->md_info.client_present) {
            rc |= bmi26x_hal_config_int_md(this, false, true, (uint8_t)BMI26X_MD_CONFIG_HW_CFG);
        } else {
            rc |= bmi26x_hal_config_int_md(this, false, false, (uint8_t)BMI26X_MD_CONFIG_HW_CFG);
        }
    }

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    rc |= bmi26x_hal_config_int_drdy(this, istate->int_en_flags_req.bits.drdy.flag);
#endif

    BMI26X_INST_LOG(MED, this, "md int:0x%x, enable_fifo_stream:%d",
                    istate->int_en_flags_req.bits.md | (istate->md_info.client_present << 1), enable_fifo_stream);
    rc |= bmi26x_hal_config_int_fifo(this, enable_fifo_stream);


#if BMI26X_CONFIG_ENABLE_LOWG
    // istate->lowg_info.enable_lowg_int
    if (istate->int_en_flags_req.bits.lowg) {
        rc |= bmi26x_hal_inst_set_lowg_config(istate, true);
        rc |= bmi26x_hal_config_int_lowg(istate->owner, true, false, (uint8_t)BMI26X_LOWG_CONFIG_HW_CFG);
    } else {
        if (istate->lowg_info.client_present) {
            rc |= bmi26x_hal_inst_set_lowg_config(istate, false);
            rc |= bmi26x_hal_config_int_lowg(istate->owner, false, false, (uint8_t)BMI26X_LOWG_CONFIG_HW_CFG);
        }
    }

    BMI26X_INST_LOG(MED, istate->owner, "lowg.debug, int:%d, ps:%d",
            istate->int_en_flags_req.bits.lowg,
            istate->lowg_info.client_present);
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    // istate->lowg_info.enable_lowg_int
    if (istate->int_en_flags_req.bits.dbtap) {
        rc |= bmi26x_hal_inst_set_dbtap_config(istate, true);
        rc |= bmi26x_hal_config_int_dbtap(istate->owner, true, false, (uint8_t)BMI26X_LOWG_CONFIG_HW_CFG);
    } else {
        if (!istate->dbtap_info.client_present) {
            if (istate->dbtap_info.enable_dbtap_int) {
                rc |= bmi26x_hal_config_int_dbtap(istate->owner, false, true, (uint8_t)BMI26X_LOWG_CONFIG_HW_CFG);
            }
            rc |= bmi26x_hal_inst_set_dbtap_config(istate, false);
            rc |= bmi26x_hal_config_int_dbtap(istate->owner, false, false, (uint8_t)BMI26X_LOWG_CONFIG_HW_CFG);
        }
    }

    BMI26X_INST_LOG(MED, istate->owner, "dbtap.debug, int:%d, ps:%d",
            istate->int_en_flags_req.bits.dbtap,
            istate->dbtap_info.client_present);
#endif

    istate->int_en_flags_curr.flag = istate->int_en_flags_req.flag;

    //</config_interrupts>

    //<config_dae>
    // Enable timer in case of sensor temp clients
    BMI26X_INST_LOG(MED, this, "publish sensors:0x%x", fifo_info->publish_sensors);

    if (fifo_info->publish_sensors & BMI26X_SENSOR_TEMP) {
        bool avail_dae_if = false;

        if (fifo_info->publish_sensors & BMI26X_GYRO) {
            BMI26X_INST_LOG(MED, this, "sensor temp start with gyro already");
        } else {
            rc |= bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_TEMP_NORMAL);
        }
#if BMI26X_CONFIG_ENABLE_DAE
        avail_dae_if = bmi26x_dae_if_available(this);
#endif
        if (!avail_dae_if) {
            bmi26x_hal_config_polling_timer_4_temp(this);
        }
    } else {
        bmi26x_hal_stop_tempetature_timer(this);
        rc |= bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_TEMP_SUSPEND);
    }

    istate->sensor_temp_info.sampling_intvl_curr = istate->sensor_temp_info.sampling_intvl_req;
    istate->sensor_temp_info.timer_itvl_changed = 0;

#if BMI26X_CONFIG_ENABLE_DAE
    if ((0 != fifo_info->publish_sensors) || (istate->int_en_flags_req.bits.md)
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            || istate->int_en_flags_req.bits.dbtap
#endif
            ) {
        if (!bmi26x_dae_if_start_streaming(this)) {
            // TODO
        }
    }
#endif
    //</config_dae>

    istate->config_step = BMI26X_CONFIG_IDLE; /* done with reconfigure */

    bmi26x_hal_send_config_event(this);

#if BMI26X_CONFIG_ENABLE_CRT
    rc = bmi26x_hal_crt_assert(this);
    // exit from island
    bmi26x_hal_inst_exit_island(this);
    rc = bmi26x_hal_reconfig_crt_param(istate->owner,
                                       &istate->gyro_info.sstate->crt_gain);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
#endif

    /* power mode assert */
    rc = bmi26x_hal_power_mode_assert(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

#if   BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    bmi26x_hal_prepare_hb_timer(istate);
#endif
    //XXX Notice, this will lead to system delay due to bus read
    bmi26x_hal_dump_reg(this);

    return rc;
}

bool bmi26x_hal_is_fifo_int_still_hangup(
    bmi26x_instance_state       *istate,
    uint8_t  regv_ff_int_st)
{
    UNUSED_VAR(istate);

    if (BST_GET_VAL_BITBLOCK(regv_ff_int_st, 0, 1)) {
        return true;
    } else {
        return false;
    }
}


bool bmi26x_hal_handle_int_latch(bmi26x_instance_state *istate, uint8_t regv)
{
    bool int_fired = false;
    if (BST_GET_VAL_BIT(regv, 6)) {
        // handle a MD event
        if (istate->int_en_flags_curr.bits.md && !istate->fac_test_in_progress) {
            bmi26x_hal_handle_interrupt_md(istate, sns_get_system_time());
        }
        // TODO others
        int_fired = true;
    }

#if BMI26X_CONFIG_ENABLE_LOWG
    if (BST_GET_VAL_BIT(regv, 2)) {
        if (istate->int_en_flags_curr.bits.lowg && !istate->fac_test_in_progress) {
            bmi26x_hal_handle_interrupt_lowg(istate, sns_get_system_time ());
        }
        int_fired = true;
    }
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    if (BST_GET_VAL_BIT(regv, BMI26X_DOUBLE_TAP_BIT_POS_IN_INT_STATUS)) {
        // handle a MD event
        if (istate->int_en_flags_curr.bits.dbtap && !istate->fac_test_in_progress) {
            bmi26x_hal_handle_interrupt_dbtap(istate, sns_get_system_time());
        }
        // TODO others
        int_fired = true;
    }
#endif


    return int_fired;
}


bool bmi26x_hal_int_pin_is_high(
    bmi26x_instance_state       *istate,
    uint8_t *regv)
{
    bool                        is_high = false;
    sns_rc                      rc = SNS_RC_SUCCESS;
    uint8_t range_val = 0x00;
    uint8_t regv_set = 0x00;

#if BMI26X_CONFIG_ENABLE_SEE_LITE
    sns_gpio_state              gpio_level = SNS_GPIO_STATE_HIGH;

    rc = bmi26x_read_gpio(istate, &istate->sstate_creator->common.irq_config, &gpio_level);
    if ((gpio_level == SNS_GPIO_STATE_HIGH) || (SNS_RC_SUCCESS != rc)) {
        is_high = true;
    } else {
        is_high = false;
    }
#else
    //regv_set = istate->gyro_info.range_req;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_INT_STATUS_0, regv, 2);
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_GYR_RANGE, &range_val, 1);
    if (range_val != regv_set) {
      BMI26X_INST_LOG(MED, istate->owner, "range value 0x%x requeest 0x%x 0x%x",
        range_val, istate->gyro_info.range_req, regv_set);
      rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_GYR_RANGE, 0, 2, regv_set);
    }
    is_high = (regv[0] | regv[1]) || (SNS_RC_SUCCESS != rc);

    BMI26X_INST_LOG(MED, istate->owner, "check int pin, <0x%x 0x%x 0x%x>",
                    regv[0], regv[1], range_val);
#endif

    return is_high;
}

sns_rc bmi26x_hal_prepare_hw(bmi26x_instance_state  *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (istate->inst_inted == 0) {
        rc = bmi26x_hal_reset_device_wrapper(istate->owner);
        if (rc != SNS_RC_SUCCESS) {
        }
        istate->inst_inted = 1;
    }
    return rc;
}

static sns_rc bmi26x_hal_reconfig_hw_process(sns_sensor_instance *this, bmi26x_hw_cfg_ctx_t hw_cfg_ctx)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;

    sns_rc                      rc = SNS_RC_SUCCESS;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    BMI26X_INST_LOG(HIGH, this, "#HAL# reconfig_hw: %d %d step:%d, sbus.mode:%d on ctx:%d",
                    fifo_info->ff_sensors_en_req,
                    fifo_info->ff_sensors_en_curr, istate->config_step,
                    istate->sbus_in_normal_mode,
                    hw_cfg_ctx);

    /* prepare sensor*/
    rc |= bmi26x_hal_prepare_hw(istate);

    /* load configuration */
    rc = bmi26x_hal_load_sensor_cfg(this);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    //we cannot change the HW setting when FIFO async read is still pending
    if (fifo_info->ff_flush_in_proc) {
        BMI26X_INST_LOG(HIGH, this, "hw_config_pending due to fifo_flush 1: %d, <%u %u>",
                        fifo_info->ff_flush_trigger,
                        istate->async_com_read_request,
                        istate->async_com_read_response);
        istate->hw_config_pending = 1;
        return SNS_RC_SUCCESS;
    }

    rc |= bmi26x_hal_fifo_prepare_4_cos(this, hw_cfg_ctx);

    if (!fifo_info->ff_flush_in_proc) {
        istate->hw_config_pending = 0;

        bmi26x_hal_fifo_invalidate_sensors(istate, fifo_info->ff_sensors_to_invalidate);
        fifo_info->ff_sensors_to_invalidate = 0;
    } else {
        istate->hw_config_pending = 1;

        BMI26X_INST_LOG(HIGH, istate->owner, "hw_config_pending due to fifo_flush 2: 0x%x, now cfg step:%d",
                        (fifo_info->ff_sensors_to_invalidate << 8) | fifo_info->ff_flush_trigger,
                        istate->config_step);

        return SNS_RC_SUCCESS;
    }

    rc = bmi26x_hal_do_config_hw_now(this);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}

sns_rc bmi26x_hal_reconfig_hw(sns_sensor_instance *this, bmi26x_hw_cfg_ctx_t hw_cfg_ctx)
{
    sns_rc rc = SNS_RC_SUCCESS;

    rc = bmi26x_hal_reconfig_hw_process(this, hw_cfg_ctx);
    return rc;
}

/** See sns_bmi26x_hal.h */
void bmi26x_hal_handle_sensor_temp_sample(sns_sensor_instance *const instance)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    sns_rc                      rc;
    uint8_t                     buf[2];

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_TEMPERATURE_0, buf, 2);

    if (SNS_RC_SUCCESS == rc) {
        const uint8_t               *data_buf = buf;
        bmi26x_hal_convert_and_send_temp_sample(instance,
                                                sns_get_system_time(),
                                                data_buf);
    }
}
void bmi26x_hal_cleanup_processing_fifo_flush(bmi26x_instance_state  *istate,
                                              uint32_t    ff_send_flush_done)
{
    bmi26x_fifo_info_t              *fifo_info = &istate->fifo_info;
    sns_rc rc = SNS_RC_SUCCESS;

    fifo_info->ff_flush_in_proc = false;

    if (istate->hw_config_pending) {
        istate->hw_config_pending = 0;

        fifo_info->ff_sensors_flush_done_before_invalid = fifo_info->ff_sensors_to_invalidate;
        BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ pick up pending hw_config task: 0x%x",
                        fifo_info->ff_sensors_flush_done_before_invalid);

        bmi26x_hal_fifo_invalidate_sensors(istate, fifo_info->ff_sensors_to_invalidate);
        fifo_info->ff_sensors_to_invalidate = 0;

        if (ff_send_flush_done) {
            bmi26x_hal_flush_cleanup(istate, ff_send_flush_done);
        }

        //in extreme cases, the state may have changed during the fifo flush
        rc = bmi26x_hal_reconfig_hw(istate->owner, HW_CONFIG_CTX_ON_DEF_FLUSH_PENDING);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! rcfg hw error:%d", rc);
        }
    }
}

void bmi26x_hal_fifo_buffer_proc_cleanup(
    bmi26x_instance_state           *istate,
    bmi26x_fifo_parse_out_desc_t    *ff_parse_out_desc)
{
    bmi26x_fifo_info_t              *fifo_info = &istate->fifo_info;
    bmi26x_int_check_ctx_t          ctx;
    uint32_t                        ff_send_flush_done = 0;
    sns_rc rc = SNS_RC_SUCCESS;

    istate->reg_int_ctx.avail.flags = 0;

    if (istate->async_com_read_request) {
        istate->async_com_read_response ++;
    }

    BMI26X_INST_LOG(MED, istate->owner, "fifo clean up:<%d %d>, flush:<%d %d>",
            istate->async_com_read_request, istate->async_com_read_response,
            fifo_info->ff_flush_in_proc, istate->ff_flush_client_req);

    if (istate->async_com_read_response <= istate->async_com_read_request) {
        if (istate->async_com_read_request == istate->async_com_read_response) {
            istate->async_com_read_response = 0;
            istate->async_com_read_request = 0;
        }
    } else {
        istate->async_com_read_request = istate->async_com_read_response;
        BMI26X_INST_LOG(HIGH, istate->owner,
                "bmi26x_cp_ handle exception in ascp_req");
    }

    if (!istate->ff_flush_client_req) {
    } else {
        istate->ff_flush_client_req = 0;
        ff_send_flush_done |= BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_REQ;
    }

    if (!fifo_info->ff_flush_in_proc) {
    } else {
        if ((istate->async_com_read_request == istate->async_com_read_response)
                && ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_end_frame) {

            bmi26x_hal_cleanup_processing_fifo_flush(istate, 0);

            if (fifo_info->ff_flush_in_proc) {
                BMI26X_INST_LOG(HIGH, istate->owner, "bmi26x_cp_ another fifo_flush");
            } else {
                ff_send_flush_done |= BMI26X_FIFO_FLUSH_DONE_CTX_HW_CHANGE;
            }
        } else {
            BMI26X_INST_LOG(MED, istate->owner, "#flush# not clean up ff_flush_in_proc still 0x%x",
                            ((istate->async_com_read_request - istate->async_com_read_response) << 1) |
                            ff_parse_out_desc->ff_parse_rslt.bits.ff_avail_end_frame);
        }
    }

    if (ff_send_flush_done) {
        bmi26x_hal_flush_cleanup(istate, ff_send_flush_done);
    }

    if (fifo_info->ff_int_masked) {
        istate->sbus_mon_single_byte_rw_off = 0;
        fifo_info->ff_int_masked = 0;

        ctx.int_check_trigger = BMI26X_INT_CHECK_TRIGGER_POST_FIFO_READ;
        ctx.timestamp = bmi26x_get_sys_tick();
        bool com_port_read_granted;
        com_port_read_granted  = (istate->async_com_read_request - istate->async_com_read_response) <
                                 BMI26X_CONFIG_COM_FLOW_CTRL_THRESH;
        if (com_port_read_granted) {
            rc = bmi26x_hal_handle_interrupt(istate->owner, &ctx);
            if (rc != SNS_RC_SUCCESS) {
                BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! handle isr error:%d", rc);
            }
        } else {
            //FIXME
            //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_FIFO_FLUSH);
            BMI26X_INST_LOG(HIGH, istate->owner, "dropped int_check: %d",
                            (istate->async_com_read_request - istate->async_com_read_response));
        }
    }
}


#if BMI26X_CONFIG_ENABLE_DIAG_LOG
static
void bmi26x_hal_init_diag_sample_state(
    sns_sensor_instance             *instance,
    bmi26x_fifo_sample_report_ctx_t *ctx,
    uint8_t                         sensors)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state *)instance->state->state;
    sns_diag_service            *diag = istate->diag_service;

    if (sensors & BMI26X_ACCEL) {
        log_sensor_state_raw_info_t *tmp = ctx->log_state_info_acc;

        sns_memzero(tmp, sizeof(log_sensor_state_raw_info_t));
        tmp->encoded_sample_size = istate->log_raw_encoded_size;

        tmp->diag = diag;
        tmp->instance = instance;
        tmp->sensor_uid = &istate->accel_info.sstate->my_suid;
        bmi26x_log_sensor_state_raw_alloc(tmp, 0);
    }

    if (sensors & BMI26X_GYRO) {
        log_sensor_state_raw_info_t *tmp = ctx->log_state_info_gyr;

        sns_memzero(tmp, sizeof(log_sensor_state_raw_info_t));
        tmp->encoded_sample_size = istate->log_raw_encoded_size;

        tmp->diag = diag;
        tmp->instance = instance;
        tmp->sensor_uid = &istate->gyro_info.sstate->my_suid;
        bmi26x_log_sensor_state_raw_alloc(tmp, 0);
    }

    //TODOMAG
}
#endif

void bmi26x_hal_process_fifo_data_buffer(
    sns_sensor_instance     *instance,
    const uint8_t           *fifo_buf,
    uint32_t                fifo_len,
    bmi26x_int_trigger_source_t  trigger)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state *)instance->state->state;
    bmi26x_fifo_parse_ctx_t     ff_parse_ctx = {NULL, 0, 0, 0, NULL};
    bmi26x_fifo_parse_ctx_t     ff_parse_ctx_dryrun = {NULL, 0, 0, 0, NULL};
    bmi26x_fifo_parse_out_desc_t  ff_parse_out_desc;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;
    uint8_t                     regv;
    uint16_t dummy_bytes = 0;

    if (istate->bus_is_spi) {
        dummy_bytes = 1;
    } else {
    }

    if (fifo_len < (7 + dummy_bytes)) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! fifo len:%d", fifo_len);
        return ;
    }

    // ff_read_ctx_type
    BMI26X_INST_LOG(LOW, istate->owner, "process fifo data buffer len:%u content:<0x%x 0x%x 0x%x 0x%x>on ctx:%d, trigger:%d",
                    fifo_len,
                    fifo_buf[0], fifo_buf[1], fifo_buf[2],
                    istate->fifo_info.ff_read_ctx_type,
                    trigger);

    if (BMI26X_FIFO_READ_CTX_TYPE_DAE != istate->fifo_info.ff_read_ctx_type) {
        uint8_t regv_buff[2] = {0};
        //if (fifo_info->ff_wmi_missing_possible)
        if (bmi26x_hal_int_pin_is_high(istate, regv_buff)) {
            uint8_t regv_cache = istate->regv_cache.cache_regv_int_map_1;
            regv = BST_SET_VAL_BITBLOCK(regv_cache, 5, 6, 0);
            BMI26X_INST_LOG(LOW, istate->owner, "pin is high:%x", regv);
            istate->sbus_mon_single_byte_rw_off = 1;
            //bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_INT_MAP_HW, &regv, 1);
            fifo_info->ff_int_masked |= 1;
        }
    }

    if (istate->bus_is_spi) {
        ff_parse_ctx.ff_buf = fifo_buf + BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY;
        ff_parse_ctx.ff_buf_len = fifo_len - BMI26X_CONFIG_SPI_BURST_READ_LEN_DUMMY;
    } else {
        ff_parse_ctx.ff_buf = fifo_buf;
        ff_parse_ctx.ff_buf_len = fifo_len;
    }
    ff_parse_ctx_dryrun = ff_parse_ctx;
    ff_parse_ctx_dryrun.ff_parse_flag |= BMI26X_FIFO_PARSE_FLAG_DRYRUN;

    memset(&ff_parse_out_desc, 0, sizeof(ff_parse_out_desc));
    // drdyrun just to collect the fifo info
    bmi26x_hal_fifo_parse_n_proc_frames(istate, &ff_parse_ctx_dryrun, &ff_parse_out_desc);

    if (!ff_parse_out_desc.ff_parse_rslt.bits.ff_batch_empty) {
        bmi26x_hal_fifo_resolve_ts_ref(istate, &ff_parse_out_desc);

        BMI26X_INST_LOG(LOW, instance, "fifo sensor enable:0x%x fc:%d",
                        fifo_info->ff_sensors_en_curr,
                        ff_parse_out_desc.fc_this_batch_acc);

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
        log_sensor_state_raw_info_t   log_accel_state_raw_info;
        log_sensor_state_raw_info_t   log_gyro_state_raw_info;
        bmi26x_fifo_sample_report_ctx_t ff_sample_rpt_ctx;
        //TODOMAG

        ff_sample_rpt_ctx.log_state_info_acc = &log_accel_state_raw_info;
        ff_sample_rpt_ctx.log_state_info_gyr = &log_gyro_state_raw_info;

        bmi26x_hal_init_diag_sample_state(instance, &ff_sample_rpt_ctx, fifo_info->ff_sensors_en_curr);

        ff_parse_ctx.priv_data = &ff_sample_rpt_ctx;
#endif

        bmi26x_hal_fifo_parse_n_proc_frames(istate, &ff_parse_ctx, &ff_parse_out_desc);


#if BMI26X_CONFIG_ENABLE_DEBUG_TEST
        if (fifo_info->ff_sensors_en_curr & BMI26X_ACCEL) {
            INSERT_TRACE_POINT3_T(bmi26x, g_bmi26x_cnt_session_acc + 1, fifo_info->ff_tm_info.fc_accum_curr_acc & 0xff,
                                  (fifo_info->ff_tm_info.fc_accum_curr_acc >> 8) & 0xff);
        }

        if (fifo_info->ff_sensors_en_curr & BMI26X_GYRO) {
            INSERT_TRACE_POINT3_T(bmi26x, g_bmi26x_cnt_session_gyr + 1, fifo_info->ff_tm_info.fc_accum_curr_gyr & 0xff,
                                  (fifo_info->ff_tm_info.fc_accum_curr_gyr >> 8) & 0xff);
        }
#endif

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
        if (fifo_info->ff_sensors_en_curr & BMI26X_ACCEL) {
            bmi26x_log_sensor_state_raw_submit(&log_accel_state_raw_info, true);
        }

        if (fifo_info->ff_sensors_en_curr & BMI26X_GYRO) {
            bmi26x_log_sensor_state_raw_submit(&log_gyro_state_raw_info, true);
        }
#endif

#if BMI26X_CONFIG_ENABLE_ONLY_ACCEPT_ODR_EFFECTIVE_FIFO_FRAMES
        //FIXME
        if ((istate->fifo_info.ff_need_stab_cnt_due_to_odr_acc > 0)
                /*&& (!istate->fifo_info.ff_flush_in_proc)*/) {
            uint8_t acc_disable = BMI26X_ACCEL;
            fifo_info->ff_tm_info.avail_ts_last_batch &= ~(acc_disable);
            //bmi26x_hal_update_couple_ts_host_and_dev_rt(istate);
        } else {
        }

        if ((istate->fifo_info.ff_need_stab_cnt_due_to_odr_gyr > 0)
                /*&& (!istate->fifo_info.ff_flush_in_proc)*/) {
            uint8_t gyr_disable = BMI26X_GYRO;
            fifo_info->ff_tm_info.avail_ts_last_batch &= ~(gyr_disable);
            //bmi26x_hal_update_couple_ts_host_and_dev_rt(istate);
        } else {
        }

    } else { //NOTICE
        //bmi26x_hal_fifo_resolve_ts_ref(istate, &ff_parse_out_desc);
    }
#endif

    // FIXME
    if (BMI26X_FIFO_READ_CTX_TYPE_DAE != istate->fifo_info.ff_read_ctx_type) {
        bmi26x_hal_fifo_buffer_proc_cleanup(istate, &ff_parse_out_desc);
    }

#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    istate->hb_cfg_info.ts_data_event = bmi26x_get_sys_tick();
#endif

   BMI26X_INST_LOG(LOW, instance, "est itvl tick: %u %u",
                   (uint32_t)(istate->fifo_info.ff_itvl_ticks_est_acc),
                   (uint32_t)(istate->ts_hw_res_ticks_per_bit_ideal * (1 << (BMI26X_REGV_ODR_1600HZ -
                                   istate->accel_info.odr_curr + 4))));
    //TODOMAG
}


/**
 * Changes all gated accel requests to non-gated accel requests.
 *
 * @param instance   Reference to the instance
 *
 * @return None
 */
static void bmi26x_hal_convert_accel_gated_req_to_non_gated(
    sns_sensor_instance *const   instance)
{
    sns_request                 *request;
    bool                        req_converted_to_non_gated = false;

    /** Parse through existing requests and change gated accel
     *  requests to non-gated accel requests. */
    for (request = (sns_request *)instance->cb->get_client_request(instance, &((sns_sensor_uid)ACCEL_SUID), true);
            NULL != request;
            request = (sns_request *)instance->cb->get_client_request(instance, &((sns_sensor_uid)ACCEL_SUID), false)) {
        if (request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            request->message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
            req_converted_to_non_gated = true;
        }
    }

    /** Send an event to gated stream clients that the request is
     *  now treated as non-gated */
    if (req_converted_to_non_gated) {
        sns_service_manager *mgr = instance->cb->get_service_manager(instance);
        sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
        sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
        bmi26x_instance_state *istate = (bmi26x_instance_state *)instance->state->state;

        if (NULL != event) {
            event->message_id = SNS_STD_EVENT_GATED_SENSOR_MSGID_GATED_REQ_CONVERTED_TO_NON_GATED;
            event->event_len = 0;
            event->timestamp = sns_get_system_time();
            e_service->api->publish_event(e_service, instance, event, &istate->accel_info.sstate->my_suid);
        }
    }
}

static void bmi26x_hal_log_interrupt_event(bmi26x_instance_state *istate, sns_time irq_timestamp)
{
#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    sns_diag_service* diag = istate->diag_service;
    // Sensor State HW Interrupt Log
    sns_diag_sensor_state_interrupt *log_diag_4_ss_int =
        (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
            diag,
            istate->owner,
            &istate->md_info.sstate->my_suid,
            sizeof(sns_diag_sensor_state_interrupt),
            SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT);

    if (NULL != log_diag_4_ss_int) {
        log_diag_4_ss_int->interrupt = SNS_DIAG_INTERRUPT_MOTION;
        log_diag_4_ss_int->timestamp = irq_timestamp;

        diag->api->submit_log(diag,
                              istate->owner,
                              &istate->md_info.sstate->my_suid,
                              sizeof(sns_diag_sensor_state_interrupt),
                              log_diag_4_ss_int,
                              SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT,
                              istate->log_interrupt_encoded_size,
                              bmi26x_encode_sensor_state_log_interrupt);
    }

    pb_send_event(istate->owner,
                  sns_motion_detect_event_fields,
                  &istate->md_info.md_state,
                  irq_timestamp,
                  SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                  &istate->md_info.sstate->my_suid);
#else
    UNUSED_VAR(istate);
    UNUSED_VAR(irq_timestamp);
#endif
}

void bmi26x_hal_send_md_event(bmi26x_instance_state *istate,
                sns_motion_detect_event* event, sns_time event_timestamp)
{
    BMI26X_INST_LOG(HIGH, istate->owner, "MD event:%d", event->motion_detect_event_type);

    pb_send_event(istate->owner,
                  sns_motion_detect_event_fields,
                  event,
                  event_timestamp,
                  SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                  &istate->md_info.sstate->my_suid);
}

static void bmi26x_hal_interrupt_md_clearup(bmi26x_instance_state   *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;

    istate->int_en_flags_curr.bits.md = false;
    istate->int_en_flags_req.bits.md = false;

    rc = bmi26x_hal_config_int_md(istate->owner, false, false, (uint8_t)BMI26X_MD_CONFIG_MD_CLEARUP);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! cfg int md error");
        return ;
    }

    istate->md_info.md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_FIRED;
    if (istate->accel_info.gated_client_present) {
        //TODO: check if this is needed by driver or will SEE framework do set_client_request() again
        istate->fifo_info.publish_sensors |= BMI26X_ACCEL;
        bmi26x_inst_assess_overall_req(istate);

        rc = bmi26x_hal_reconfig_hw(istate->owner, HW_CONFIG_CTX_ON_MOTION_DETECTION);

        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! cfg int md error");
            return ;
        }

        if (istate->hw_config_pending) {
        }
    }
}

void bmi26x_hal_handle_interrupt_md(
    bmi26x_instance_state   *istate,
    sns_time                irq_timestamp)
{
    sns_sensor_instance         *instance = istate->owner;

    /**
     * 1. Handle MD interrupt: Send MD fired event to client.
     * 2. Disable MD.
     * 3. Start Motion Accel FIFO stream with desired config.
     */

    // 1.
    sns_motion_detect_event md_state;
    md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_FIRED;

    BMI26X_INST_LOG(HIGH, instance, "!!! MD fired! md.cp:%d md_evt_num:%u @ts:%u",
                    istate->md_info.client_present,
                    (uint32_t)istate->md_info.md_event_num,
                    (uint32_t) irq_timestamp);

    bmi26x_hal_send_md_event(istate, &md_state, irq_timestamp);

    istate->md_info.md_state.motion_detect_event_type = md_state.motion_detect_event_type;

    bmi26x_hal_log_interrupt_event(istate, irq_timestamp);

    bmi26x_hal_convert_accel_gated_req_to_non_gated(instance);
    istate->md_info.md_event_num ++;
    istate->md_info.ts_latest_event_fired = irq_timestamp;

    // 2,3
    bmi26x_hal_interrupt_md_clearup(istate);

    // dump
    //bmi26x_hal_dump_reg(istate->owner);
}

void bmi26x_hal_handle_interrupt_fifo_full(bmi26x_instance_state *istate)
{
    sns_rc  rc;
    uint8_t si_buf[2];
    uint16_t ff_wml_bytes;
    bmi26x_hal_fifo_invalidate_sensors(istate, (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG));

    istate->ts_pair_sys_dev.avail_1st = 0;
    rc = bmi26x_hal_update_couple_ts_host_and_dev_rt(istate);
    if (rc != SNS_RC_SUCCESS) {
    }
    istate->ts_pair_sys_dev_4_ts_res_est = istate->ts_pair_sys_dev;

    rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_FIFO_FLUSH);
    if (rc != SNS_RC_SUCCESS) {
    }

    ff_wml_bytes = (istate->fifo_info.ff_wml_bytes_curr >> 1);
    si_buf[0] = ff_wml_bytes & 0xff;
    si_buf[1] = (ff_wml_bytes >> 8) & 0xff;

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_FIFO_WTM_0, si_buf, 2);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! ss write error:%d",
                        rc);
    }

    BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! FIFO full");
    //TODO: how to send fifo full event
}


void bmi26x_hal_handle_interrupt_fifo_wml(
    bmi26x_instance_state   *istate)
{
    bmi26x_fifo_read_ctx_t      ctx;

    ctx.ctx_type = BMI26X_FIFO_READ_CTX_TYPE_WMI;
    ctx.sync_read = false;

    istate->fifo_info.ff_read_ctx_type = BMI26X_FIFO_READ_CTX_TYPE_WMI;

    bmi26x_hal_fifo_read_out(istate, &ctx);
    INSERT_TRACE_POINT2_T(bmi26x, 'f', 'f');
}

#if BMI26X_CONFIG_ENABLE_DRI_MODE
static
sns_rc bmi26x_hal_read_data_n_ctx_at_dri(
    bmi26x_instance_state   *istate,
    uint8_t                 *si_buf,
    uint8_t                 regv_status)
{
    int8_t      reg_addr_start      = BMI26X_REGA_USR_EVENT;
    int8_t      reg_addr_end        = BMI26X_REGA_USR_EVENT;
    uint8_t     len                 = 0;
    sns_rc      rc = SNS_RC_SUCCESS;
    uint8_t     *regvp = si_buf;

    if (regv_status & B7_SET) { //accel
        reg_addr_start = BMI26X_REGA_USR_DATA_ACC_X_LSB;
        regvp += BMI26X_DATA_BUFFER_OFFSET_ACCEL;
    } else {  // ONLY gyro
        if (regv_status & B6_SET) { //gyro
            reg_addr_start = BMI26X_REGA_USR_DATA_GYR_X_LSB;
            regvp += BMI26X_DATA_BUFFER_OFFSET_GYRO;
        }
    }
    //TODOMAG

    len = 1 + reg_addr_end - reg_addr_start;
    rc = bmi26x_sbus_read_wrapper(istate, reg_addr_start, regvp, len);

    return rc;
}


sns_rc bmi26x_hal_handle_interrupt_drdy_from_dae(
    bmi26x_instance_state   *istate,
    bmi26x_int_check_ctx_from_dae_t *ctx)
{
    uint8_t                     sensors_to_report = (uint8_t) (ctx->acc_gyro_flag);
    sns_time                    ts = ctx->timestamp;
    bmi26x_fifo_sample_report_ctx_t ff_sample_rpt_ctx;
    sns_rc                      rc = SNS_RC_SUCCESS;

    if (ctx->data_ptr == NULL) {
        return SNS_RC_FAILED;
    }

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    log_sensor_state_raw_info_t   log_accel_state_raw_info;
    log_sensor_state_raw_info_t   log_gyro_state_raw_info;

    ff_sample_rpt_ctx.log_state_info_acc = &log_accel_state_raw_info;
    ff_sample_rpt_ctx.log_state_info_gyr = &log_gyro_state_raw_info;

    bmi26x_hal_init_diag_sample_state(istate->owner, &ff_sample_rpt_ctx, sensors_to_report);
#endif

    if (sensors_to_report & BMI26X_ACCEL) {
        BMI26X_INST_LOG(LOW, istate->owner, "drdy mode from dae");
        bmi26x_hal_report_single_frame_acc(istate,
                                           ctx->data_ptr + BMI26X_DATA_BUFFER_OFFSET_ACCEL,
                                           ts, &ff_sample_rpt_ctx);
    }

    if (sensors_to_report & BMI26X_GYRO) {
        bmi26x_hal_report_single_frame_gyr(istate,
                                           ctx->data_ptr + BMI26X_DATA_BUFFER_OFFSET_GYRO,
                                           ts, &ff_sample_rpt_ctx);
    }

    //TODOMAG

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    if (sensors_to_report & BMI26X_ACCEL) {
        bmi26x_log_sensor_state_raw_submit(&log_accel_state_raw_info, true);
    }

    if (sensors_to_report & BMI26X_GYRO) {
        bmi26x_log_sensor_state_raw_submit(&log_gyro_state_raw_info, true);
    }
#endif

    return rc;
}


void bmi26x_hal_handle_drdy_from_dae(
    sns_sensor_instance     *const this,
    bmi26x_int_check_ctx_from_dae_t *ctx)
{
    sns_rc                      rc;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;

    uint8_t                     cnt_irq_check;

    cnt_irq_check = 0;

    if (istate->int_en_flags_curr.bits.drdy.flag || 1) {
        rc = bmi26x_hal_handle_interrupt_drdy_from_dae(istate, ctx);
        if (rc != SNS_RC_SUCCESS) {
            return ;
        }

        cnt_irq_check = 1;  //no need to read sensortime in the following steps

        bmi26x_int_en_flag_t int_en_flags_curr = istate->int_en_flags_curr;
        int_en_flags_curr.bits.drdy.flag = 0;
        int_en_flags_curr.bits.fifo.flag = 0;
        if (!int_en_flags_curr.flag) {
            return ;
        }
    }
}

sns_rc bmi26x_hal_handle_interrupt_drdy(
    bmi26x_instance_state   *istate,
    bmi26x_int_check_ctx_t    *ctx)
{
    uint8_t                     si_buf[32];
    uint8_t                     flag = istate->int_en_flags_curr.bits.drdy.flag;
    uint8_t                     sensors_to_report = 0;
    uint8_t                     regv_status;
    sns_time                    ts = ctx->timestamp;
    bmi26x_fifo_sample_report_ctx_t ff_sample_rpt_ctx;
    sns_rc                      rc = SNS_RC_SUCCESS;
    uint8_t                     ff_send_flush_done = 0;

    istate->sbus_mon_single_byte_rw_off = 1;
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_INT_STATUS_1, &regv_status, 1);
    istate->sbus_mon_single_byte_rw_off = 0;

    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_hal_read_data_n_ctx_at_dri(istate, si_buf, regv_status);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if ((regv_status & B7_SET) && (flag & BMI26X_ACCEL)) {
        // acc
        sensors_to_report |= BMI26X_ACCEL;
    }

    if ((regv_status & B6_SET) && (flag & BMI26X_GYRO)) {
        // gyro
        sensors_to_report |= BMI26X_GYRO;
    }

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    log_sensor_state_raw_info_t   log_accel_state_raw_info;
    log_sensor_state_raw_info_t   log_gyro_state_raw_info;

    ff_sample_rpt_ctx.log_state_info_acc = &log_accel_state_raw_info;
    ff_sample_rpt_ctx.log_state_info_gyr = &log_gyro_state_raw_info;

    bmi26x_hal_init_diag_sample_state(istate->owner, &ff_sample_rpt_ctx, sensors_to_report);
#endif

    if (sensors_to_report & BMI26X_ACCEL) {
        bmi26x_hal_report_single_frame_acc(istate,
                                           si_buf + BMI26X_DATA_BUFFER_OFFSET_ACCEL,
                                           ts, &ff_sample_rpt_ctx);
    }

    if (sensors_to_report & BMI26X_GYRO) {
        bmi26x_hal_report_single_frame_gyr(istate,
                                           si_buf + BMI26X_DATA_BUFFER_OFFSET_GYRO,
                                           ts, &ff_sample_rpt_ctx);
    }

    //TODOMAG

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    if (sensors_to_report & BMI26X_ACCEL) {
        bmi26x_log_sensor_state_raw_submit(&log_accel_state_raw_info, true);
    }

    if (sensors_to_report & BMI26X_GYRO) {
        bmi26x_log_sensor_state_raw_submit(&log_gyro_state_raw_info, true);
    }
#endif

    BMI26X_INST_LOG(MED, istate->owner, "dl_drdy:%x,%x @<%u %u>",
                    flag, regv_status,
                    (uint32_t)(ts >> 32), (uint32_t)(ts & 0xFFFFFFFF));

    // flush request in data ready mode
    if (!istate->ff_flush_client_req) {
    } else {
        istate->ff_flush_client_req = 0;
        ff_send_flush_done |= BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_REQ;
    }

    if (istate->fifo_info.ff_flush_in_proc) {
        istate->fifo_info.ff_flush_in_proc = false;
    }

    if (ff_send_flush_done) {
        ff_send_flush_done |= BMI26X_FIFO_FLUSH_DONE_CTX_DRDY;
        bmi26x_hal_flush_cleanup(istate, ff_send_flush_done);
    }

    return rc;
}
#endif


sns_rc bmi26x_hal_prepare_interrupt_handling(
    bmi26x_instance_state   *istate)
{
    sns_rc                      rc = SNS_RC_SUCCESS;
    UNUSED_VAR(rc);

    if (!istate->sbus_in_normal_mode) {
        rc = bmi26x_hal_update_pmu_stat(istate, true);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        if (!istate->sbus_in_normal_mode) {
            rc = bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_ACC_NORMAL);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
    }

    return rc;
}

sns_rc bmi26x_hal_handle_interrupt(
    sns_sensor_instance     *const this,
    bmi26x_int_check_ctx_t    *ctx)
{
    sns_rc                      rc;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    bmi26x_int_stat_flag_t      int_stat = {.flag = 0};
    bool                        ff_wmi_missing_possible;
    uint8_t                     cnt_irq_check;
    bmi26x_fifo_info_t          *fifo_info = &istate->fifo_info;

    rc = bmi26x_hal_prepare_interrupt_handling(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    cnt_irq_check = 0;

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_curr.bits.drdy.flag) {
        rc = bmi26x_hal_handle_interrupt_drdy(istate, ctx);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        cnt_irq_check = 1;  //no need to read sensortime in the following steps

        bmi26x_int_en_flag_t int_en_flags_curr = istate->int_en_flags_curr;
        int_en_flags_curr.bits.drdy.flag = 0;
        int_en_flags_curr.bits.fifo.flag = 0;
        if (!int_en_flags_curr.flag) {
            return SNS_RC_SUCCESS;
        }
    }
#endif

    do {
        ctx->req_ts_hw = (0 == cnt_irq_check);
        rc = bmi26x_hal_read_int_ctx(istate, ctx);
        BMI26X_INST_LOG(LOW, istate->owner, "int_status_1:0x%x", istate->reg_int_ctx.int_status_1
                        | (((uint32_t)BMI26X_SEE_DD_ATTRIB_VERSION << 8) & (~0xff)) );

        cnt_irq_check ++;
        if (SNS_RC_SUCCESS == rc) {
        } else {
            //try again in case of error
            ff_wmi_missing_possible = true;
            if (cnt_irq_check < BMI26X_RETRY_TIMES_ON_BUS_ERROR) {
                continue;
            }
        }
        bmi26x_dev_parse_int_stat_flags(&istate->reg_int_ctx, &int_stat);

        BMI26X_INST_LOG(MED,
                        istate->owner, "ist<0x%x,0x%x,0x%x>",
                        ((istate->int_en_flags_curr.bits.dbtap << 24) |
                         (istate->int_en_flags_curr.bits.md << 16) |
                         (istate->int_en_flags_curr.bits.fifo.flag << 8) |
                         (istate->int_en_flags_curr.bits.pedo << 4) |
                         (istate->int_en_flags_curr.bits.drdy.flag)),

                        (
                            (cnt_irq_check << 8) |
                            (int_stat.bits.dbtap << 7) |
                            (int_stat.bits.md << 6) |
                            (int_stat.bits.step << 4) |
                            (int_stat.bits.drdy_detail.flag << 3 ) |
                            (int_stat.bits.err << 2) |
                            (int_stat.bits.ff_full << 1) |
                            (int_stat.bits.ff_wml)),
                        (ctx->int_check_trigger | (((uint32_t)BMI26X_SEE_DD_ATTRIB_VERSION << 8) & (~0xff)) )
                       );

        ff_wmi_missing_possible = false;

        if (int_stat.bits.ff_wml || int_stat.bits.ff_full) {
            if (istate->int_en_flags_curr.bits.fifo.flag) {
                istate->ts_irq = ctx->timestamp;

                if (!int_stat.bits.ff_full) {
                    INSERT_TRACE_POINT2_T(bmi26x, 'f', '0');
                    if (istate->async_com_read_request - istate->async_com_read_response) {
                        fifo_info->ff_int_masked |= 1;
                        BMI26X_INST_LOG(HIGH, istate->owner, "WARN!! Redundent INT when a async request on going,"
                                        "this requese will be handled combined with it");
                    } else {
                        bmi26x_hal_handle_interrupt_fifo_wml(istate);
                    }
                } else {
                    bmi26x_hal_handle_interrupt_fifo_full(istate);
                }
            }
        } else {
            if (int_stat.bits.md | int_stat.bits.dbtap | int_stat.bits.step
#if  BMI26X_CONFIG_ENABLE_LOWG
                    | int_stat.bits.lowg
#endif
                    ) {
                if (fifo_info->ff_sensors_en_curr) {
                    ff_wmi_missing_possible = true;
                }
            }
        }

#if BMI26X_CONFIG_ENABLE_PEDO
        // pedo detector
        if (int_stat.bits.step) {
            rc = bmi26x_hal_handle_pedo_interrupt(this, ctx->timestamp, BMI26X_PEDO_EVT_CTX_INT);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
#endif

        if (int_stat.bits.md) {
            /**
             * 1. Handle MD interrupt: Send MD fired event to client.
             * 2. Disable MD.
             * 3. Start Gated Accel FIFO stream with desired config.
             */
            BMI26X_INST_LOG(MED, istate->owner, "MD ISR EVENT");

            if (istate->int_en_flags_curr.bits.md) {
                bmi26x_hal_handle_interrupt_md(istate, ctx->timestamp);
            }
        }
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
        if (int_stat.bits.dbtap) {
            BMI26X_INST_LOG(MED, istate->owner, "DOUBLE_TAP ISR EVENT");

            if (istate->int_en_flags_curr.bits.dbtap) {
                bmi26x_hal_handle_interrupt_dbtap(istate, ctx->timestamp);
            }
        }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
        if (int_stat.bits.lowg) {
            BMI26X_INST_LOG(MED, istate->owner, "LOWG ISR EVENT");
            bmi26x_hal_handle_interrupt_lowg(istate, ctx->timestamp);
        }
#endif

        if (int_stat.bits.md | int_stat.bits.dbtap | int_stat.bits.step) {
            //TODO4: what is the command for bmi26x
            //bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_INT_RESET);
        }

    } while (ff_wmi_missing_possible && (cnt_irq_check < 3));


    return SNS_RC_SUCCESS;
}

/** See bmi26x_hal.h */

void bmi26x_hal_send_fifo_flush_done(
    sns_sensor_instance     *const inst,
    uint8_t                 sensor,
    sns_time                ts,
    uint8_t                 context)
{
    bmi26x_instance_state *istate = NULL;
    sns_sensor_uid const *suid = NULL;

    if (BST_ASSERT_POINT(inst)) {
        return ;
    }

    istate = (bmi26x_instance_state *)inst->state->state;

    if (sensor & BMI26X_ACCEL) {
        suid = &istate->accel_info.sstate->my_suid;
    } else if (sensor & BMI26X_GYRO) {
        suid = &istate->gyro_info.sstate->my_suid;
    } else if (sensor & BMI26X_MOTION_DETECT) {
        suid = &istate->md_info.sstate->my_suid;
    } else if (sensor & BMI26X_SENSOR_TEMP) {
        suid = &istate->sensor_temp_info.sstate->my_suid;
#if BMI26X_CONFIG_ENABLE_PEDO
    } else if (sensor & BMI26X_PEDO) {
        suid = &istate->pedo_info.sstate->my_suid;
#endif
#if BMI26X_CONFIG_ENABLE_OIS
    } else if (sensor & BMI26X_OIS) {
        suid = &istate->ois_info.sstate->my_suid;
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    } else if (sensor & BMI26X_FREE_FALL) {
        suid = &istate->lowg_info.sstate->my_suid;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    } else if (sensor & BMI26X_DOUBLE_TAP) {
        suid = &istate->dbtap_info.sstate->my_suid;
#endif
    }

    BMI26X_INST_LOG(MED, inst, "flush done event on ss:0x%x", sensor);

    if (NULL != suid) {
        sns_service_manager *mgr = inst->cb->get_service_manager(inst);
        sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
        sns_sensor_event *event = NULL;

        event = e_service->api->alloc_event(e_service, inst, 0);

        if ((NULL != event)) {
            event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
            event->event_len = 0;
            event->timestamp = (ts & (~(sns_time)0x01f)) | (context & 0x01f) + 0x20;

            e_service->api->publish_event(e_service, inst, event, suid);
        }
    }
}


bool bmi26x_hal_start_timer(
    sns_sensor_instance     *this,
    sns_data_stream         *timer_stream,
    bool                    periodic,
    sns_time                time_out)
{
    bmi26x_instance_state   *istate = (bmi26x_instance_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t                 buffer[50] = "";
    sns_request             timer_req = {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };

    UNUSED_VAR(istate);

    if (NULL == timer_stream) {
        BMI26X_INST_LOG(HIGH, istate->owner, "timer_stream NULL");
        return false;
    }

    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = periodic;
    req_payload.start_time = bmi26x_get_sys_tick();
    req_payload.timeout_period = time_out;

    timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                            sns_timer_sensor_config_fields, NULL);
    if (timer_req.request_len > 0) {
        timer_stream->api->send_request(timer_stream, &timer_req);

        BMI26X_INST_LOG(MED, istate->owner, "timer_start expires %u ticks later", (uint32_t)time_out);
    } else {
        BMI26X_INST_LOG(ERROR, this, "ERROR!!! timer req encode error");
        return false;
    }
    return true;
}

void bmi26x_hal_register_interrupt(sns_sensor_instance *this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    if (!(istate->irq_registered)) {
        sns_data_stream* data_stream = istate->interrupt_data_stream;
        uint8_t buffer[20];
        sns_request irq_req = {
            .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
            .request    = buffer
        };

        irq_req.request_len = pb_encode_request(buffer,
                                                sizeof(buffer),
                                                &istate->sstate_creator->common.irq_config,
                                                sns_interrupt_req_fields,
                                                NULL);
        if (irq_req.request_len > 0) {
            data_stream->api->send_request(data_stream, &irq_req);
            istate->irq_registered = true;
        }
    }
}

sns_rc bmi26x_inst_enable_sensor_intrrupt(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    if (istate->irq_ready) {
        int enable_fifo_stream = istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);
        enable_fifo_stream |= istate->fac_test_in_progress;

        if (enable_fifo_stream
                || istate->int_en_flags_req.bits.md) {
            rc |= bmi26x_hal_config_int_output(istate->owner, true);
        }

        if (istate->md_info.enable_md_int) {
            rc = bmi26x_hal_config_int_md(istate->owner, true, false, (uint8_t)BMI26X_MD_CONFIG_ENABLE_INT);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
#if BMI26X_CONFIG_ENABLE_LOWG
        if (istate->lowg_info.enable_lowg_int) {
            rc = bmi26x_hal_config_int_lowg(istate->owner, true, false, (uint8_t)BMI26X_LOWG_CONFIG_ENABLE_INT);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
        if (istate->dbtap_info.enable_dbtap_int) {
            rc = bmi26x_hal_config_int_dbtap(istate->owner, true, false, (uint8_t)BMI26X_DBTAP_CONFIG_ENABLE_INT);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

#endif

        if (enable_fifo_stream) {
            rc = bmi26x_hal_config_int_fifo(istate->owner, true);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
    }
    return rc;
}

// low-g
#if BMI26X_CONFIG_ENABLE_LOWG

bmi26x_free_fall_mode_cfg_t BMI26X_FREE_FALL_MODE[] = {
    // Very Sensitive
    {
            .accel_setting_1 = 0x02EE,
            .accel_setting_2 = 0x03,
            .accel_setting_3 = 0x12C,
            .accel_setting_4 = 0x12
    },

    // Sensitive
    {
            .accel_setting_1 = 0x02EE,
            .accel_setting_2 = 0x03,
            .accel_setting_3 = 0x12C,
            .accel_setting_4 = 0x15
    },

    // Robust
    {
            .accel_setting_1 = 0x02EE,
            .accel_setting_2 = 0x03,
            .accel_setting_3 = 0x12C,
            .accel_setting_4 = 0x18
    },

    // Very Robust
    {
            .accel_setting_1 = 0x01F4,
            .accel_setting_2 = 0x06,
            .accel_setting_3 = 0x12C,
            .accel_setting_4 = 0x18
    },
};

static
void bmi26x_hal_send_lowg_event(bmi26x_instance_state *istate,
        sns_free_fall_event *event, sns_time ts_event)
{
    BMI26X_INST_LOG(HIGH, istate->owner, "LOWg event:%d", event->free_fall_event_type);

    pb_send_event(istate->owner,
                  sns_free_fall_event_fields,
                  event,
                  ts_event,
                  SNS_FREE_FALL_MSGID_SNS_FREE_FALL_EVENT,
                  &istate->lowg_info.sstate->my_suid);
}



static sns_rc bmi26x_hal_set_lowg_config(bmi26x_instance_state *istate, uint8_t mode_select, bool en)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t lowg_buffer[sizeof(bmi26x_feature_lowg_t)] = {0};
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    bmi26x_feature_lowg_t *lowg = (bmi26x_feature_lowg_t *) lowg_buffer;

    if (mode_select >= (sizeof(BMI26X_FREE_FALL_MODE) / sizeof(bmi26x_free_fall_mode_cfg_t))) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid mode:%d", mode_select);
        return SNS_RC_INVALID_TYPE;
    }

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_LOW_G, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, lowg_buffer,
                             BMI26X_FEATURE_LOWG_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "low-g cfg:%d %d <0x%04x 0x%04x 0x%04x 0x%04x>",
                    lowg->feature_en, lowg->out_conf,
                    lowg->free_fall_acc_setting[0],
                    lowg->free_fall_acc_setting[1],
                    lowg->free_fall_acc_setting[2],
                    lowg->free_fall_acc_setting[3]);

    if (en) {
        lowg->free_fall_acc_setting[0] = BMI26X_FREE_FALL_MODE[mode_select].accel_setting_1;
        lowg->free_fall_acc_setting[1] = BMI26X_FREE_FALL_MODE[mode_select].accel_setting_2;
        lowg->free_fall_acc_setting[2] = BMI26X_FREE_FALL_MODE[mode_select].accel_setting_3;
        lowg->free_fall_acc_setting[3] = BMI26X_FREE_FALL_MODE[mode_select].accel_setting_4;
    } else {
    }

    lowg->feature_en = en;

    BMI26X_INST_LOG(LOW, istate->owner, "low-g cfg/:%d %d <0x%04x 0x%04x 0x%04x 0x%04x>",
                    lowg->feature_en, lowg->out_conf,
                    lowg->free_fall_acc_setting[0],
                    lowg->free_fall_acc_setting[1],
                    lowg->free_fall_acc_setting[2],
                    lowg->free_fall_acc_setting[3]);
    // sync to configuration
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, lowg_buffer, BMI26X_FEATURE_LOWG_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // debug
    {
        sns_memset(lowg_buffer, 0, sizeof(lowg_buffer));
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, lowg_buffer,
                             BMI26X_FEATURE_LOWG_PARAM_SIZE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);


        BMI26X_INST_LOG(MED, istate->owner, "low-g cfg//:%d %d <0x%04x 0x%04x 0x%04x 0x%04x>",
                        lowg->feature_en, lowg->out_conf,
                        lowg->free_fall_acc_setting[0],
                        lowg->free_fall_acc_setting[1],
                        lowg->free_fall_acc_setting[2],
                        lowg->free_fall_acc_setting[3]);
    }
    // @debug

    return rc;
}

sns_rc bmi26x_hal_config_int_lowg(sns_sensor_instance *const inst,
                                     bool en,
                                     bool lowg_armed_event,
                                     uint8_t trigger)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;

    // enable low-g interrupt
    uint8_t regv_int_funciton, regv_int_out;
    uint8_t rega_int_map, rega_int_io_ctrl;

    BMI26X_INST_LOG(MED, inst, "lowg cfg:%d %d %d %d %d /%d",
                    istate->lowg_info.client_present,
                    istate->int_en_flags_req.bits.lowg,
                    en,
                    lowg_armed_event,
                    trigger,
                    istate->lowg_info.lowg_event_num);

#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
    rega_int_map = BMI26X_REGA_USR_INT2_MAP;
    rega_int_io_ctrl = BMI26X_REGA_USR_INT2_IO_CTRL;
#else
    rega_int_map = BMI26X_REGA_USR_INT1_MAP;
    rega_int_io_ctrl = BMI26X_REGA_USR_INT1_IO_CTRL;
#endif

    // BMI26X_REGA_USR_INT2_MAP
    // istate->scp_service, &istate->com_port_info
    rc = bmi26x_com_read_byte(istate->scp_service, &istate->com_port_info, rega_int_map, &regv_int_funciton);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // BMI26X_REGA_USR_INT2_IO_CTRL
    rc = bmi26x_com_read_byte(istate->scp_service, &istate->com_port_info, rega_int_io_ctrl, &regv_int_out);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (en) {
        regv_int_funciton = BST_SET_VAL_BIT(regv_int_funciton, 2);
#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
        regv_int_out = BST_SET_VAL_BITBLOCK(regv_int_out, 1, 4, 0x05);
#endif
        istate->lowg_info.lowg_state.free_fall_event_type = SNS_FREE_FALL_EVENT_TYPE_ENABLED;
    } else {
        regv_int_funciton = BST_CLR_VAL_BIT(regv_int_funciton, 2);
#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
        regv_int_out = BST_SET_VAL_BITBLOCK(regv_int_out, 1, 4, 0x00);
#endif
        istate->lowg_info.lowg_state.free_fall_event_type = SNS_FREE_FALL_EVENT_TYPE_DISABLED;
    }

    // write
#if 0
    rc = bmi26x_com_write_byte(istate->scp_service, &istate->com_port_info, rega_int_map, &regv_int_funciton);
#endif

    rc = bmi26x_sbus_write_wrapper(istate, rega_int_map, &regv_int_funciton, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);

#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
    rc = bmi26x_com_write_byte(istate->scp_service, istate->port_info, rega_int_io_ctrl, &regv_int_out);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);
#endif

#if BMI26X_CONFIG_ENABLE_AVOID_DUPLIVATED_EVENT
    if (en || lowg_armed_event) {
        bool avoid_duplicate_lowg_ev;

        avoid_duplicate_lowg_ev = (istate->int_en_flags_req.bits.lowg && istate->int_en_flags_curr.bits.lowg)
                                && (!lowg_armed_event);

        BMI26X_INST_LOG(MED, istate->owner,
                        "bmi26x_update_lowg_intr md_ev_type:%d %d",
                        istate->lowg_info.lowg_state.free_fall_event_type,
                        istate->int_en_flags_req.bits.lowg | (istate->int_en_flags_curr.bits.lowg << 1) | (istate->lowg_info.client_present << 2));

        if (!avoid_duplicate_lowg_ev) {
            bmi26x_hal_send_lowg_event(istate, &istate->lowg_info.lowg_state,
                                       sns_get_system_time());
        }
    }
#else
    BMI26X_INST_LOG(MED, istate->owner,
                    "bmi26x_update_lowg_intr md_ev_type:%d %d",
                    istate->lowg_info.lowg_state.free_fall_event_type,
                    istate->int_en_flags_req.bits.lowg | (istate->int_en_flags_curr.bits.lowg << 1) |
                    (istate->lowg_info.client_present << 2));

    bmi26x_hal_send_lowg_event(istate, &istate->lowg_info.lowg_state,
                               sns_get_system_time());
#endif

    return rc;
}

sns_rc bmi26x_hal_inst_set_lowg_config(bmi26x_instance_state *istate, bool en)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t mode, debug;
    bool lp_need = false;

    BMI26X_INST_LOG(MED, istate->owner, "set lowg cfg:0x%02x",
                    (en | istate->lowg_info.enable_lowg_int << 1));
#if BMI26X_CONFIG_ENABLE_REGISTRY
    mode = istate->lowg_info.sstate->lowg_config.mode;
    debug = istate->lowg_info.sstate->lowg_config.debug;
    //hyst = BMI26X_LOWG_HYSTERESIS_DEFAULT_VAL_G;      //0.125g
#else
#endif

    BMI26X_INST_LOG(MED, istate->owner, "lowg cfg:%d %d %d",
            mode, debug, istate->lowg_info.sstate->lowg_config.enable);

    if (en != istate->lowg_info.enable_lowg_int) {
        // check sensor power mode
        if (!istate->sbus_in_normal_mode) {
            if (istate->regv_cache.regv_pwr_conf & 0x01) {
                lp_need = 1;
            }
            rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
        }

        // sync to configuration file
        rc = bmi26x_hal_set_lowg_config(istate, mode, en);

        // enable acc
        if (en) {
            rc |= bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_ACC_NORMAL);
            // XXX put acc to high performance mode
            //uint8_t regv = 0x0A;
            //rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_CONF, 4, 7, regv);
        }

        if (lp_need) {
            rc |= bmi26x_dev_pwr_conf(istate, 0, 0, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

        istate->lowg_info.enable_lowg_int = en;
    }

    return rc;
}


static void bmi26x_hal_interrupt_lowg_clearup(bmi26x_instance_state   *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;

    istate->int_en_flags_req.bits.lowg  = false;

    rc = bmi26x_hal_config_int_lowg(istate->owner, false, true,
                                    (uint8_t)BMI26X_LOWG_CONFIG_LOWG_CLEARUP);

    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! cfg int md error");
        return ;
    }

    istate->int_en_flags_curr.bits.lowg = false;
    istate->lowg_info.lowg_wait_for_fired_event = 0;
    istate->lowg_info.enable_lowg_int = 0;

    istate->lowg_info.lowg_state.free_fall_event_type = SNS_FREE_FALL_EVENT_TYPE_FIRED;
}

void bmi26x_hal_handle_interrupt_lowg(
        bmi26x_instance_state   *istate,
        sns_time                irq_timestamp)
{
    sns_sensor_instance         *instance = istate->owner;

    // 1. send LOWg fired event
    sns_free_fall_event lowg_state;
    lowg_state.free_fall_event_type = SNS_FREE_FALL_EVENT_TYPE_FIRED;

    BMI26X_INST_LOG(HIGH, instance, "!!! LOWG fired! lowg.cp:%d evt:%d lowg_evt_num:%u @ts:%u",
                    istate->lowg_info.client_present,
                    lowg_state.free_fall_event_type,
                    (uint32_t)(istate->lowg_info.lowg_event_num + 1),
                    (uint32_t)irq_timestamp);

    bmi26x_hal_send_lowg_event(istate, &lowg_state, irq_timestamp);

    istate->lowg_info.lowg_state.free_fall_event_type = lowg_state.free_fall_event_type;

    //bmi26x_hal_log_interrupt_event(istate, irq_timestamp);

    // XXX lowg no need to do this
    //bmi26x_hal_convert_accel_gated_req_to_non_gated(instance);

    istate->lowg_info.lowg_event_num ++;
    istate->lowg_info.ts_latest_event_fired = irq_timestamp;

    // disable lowg int and send out lowg disabled event
    bmi26x_hal_interrupt_lowg_clearup(istate);
}


#endif

// low-g
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP

static
void bmi26x_hal_send_dbtap_event(bmi26x_instance_state *istate,
        sns_double_tap_event *event, sns_time ts_event)
{
    BMI26X_INST_LOG(HIGH, istate->owner, "dbtap event: %d", event->double_tap_event_type);

    pb_send_event(istate->owner,
                  sns_double_tap_event_fields,
                  event,
                  ts_event,
                  SNS_DOUBLE_TAP_MSGID_SNS_DOUBLE_TAP_EVENT,
                  &istate->dbtap_info.sstate->my_suid);
}



static sns_rc bmi26x_hal_set_dbtap_config(bmi26x_instance_state *istate, bool en,
                                          uint16_t tap_sens_thres,
                                          uint16_t max_gest_dur,
                                          uint16_t quite_time_after_gest)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t dbtap_buffer[sizeof(bmi26x_feature_dbtap_t)] = {0};
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    bmi26x_feature_dbtap_t *dbtap = (bmi26x_feature_dbtap_t *) dbtap_buffer;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_1, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "dbtap cfg r:%d %d %d %d %d %d",
                    dbtap->single_tap_en, dbtap->double_tap_en,
                    dbtap->data_reg_en, dbtap->tap_sens_thres,
                    dbtap->max_gest_dur, dbtap->quite_time_after_gest);

    /*if (istate->fifo_info.publish_sensors & BMI26X_ACCEL) {
        dbtap->data_reg_en = 1;
    } else {
        dbtap->data_reg_en = 0;
    }*/

    if (en) {
        dbtap->max_gest_dur = max_gest_dur;
        dbtap->quite_time_after_gest = quite_time_after_gest;
        dbtap->tap_sens_thres = tap_sens_thres;
    } else {
    }

    dbtap->double_tap_en = en;


    BMI26X_INST_LOG(LOW, istate->owner, "dbtap cfg:%d %d %d %d %d %d",
                    dbtap->single_tap_en, dbtap->double_tap_en,
                    dbtap->data_reg_en, dbtap->tap_sens_thres,
                    dbtap->max_gest_dur, dbtap->quite_time_after_gest);

    // sync to configuration
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, dbtap_buffer, BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // debug
    {
        sns_memset(dbtap_buffer, 0, sizeof(dbtap_buffer));
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);


        BMI26X_INST_LOG(LOW, istate->owner, "dbtap cfg w/r:%d %d %d %d %d %d",
                        dbtap->single_tap_en, dbtap->double_tap_en,
                        dbtap->data_reg_en, dbtap->tap_sens_thres,
                        dbtap->max_gest_dur, dbtap->quite_time_after_gest);
    }
    // @debug

    return rc;
}

sns_rc bmi26x_hal_config_int_dbtap(sns_sensor_instance *const inst,
                                     bool en,
                                     bool dbtap_armed_event,
                                     uint8_t trigger)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;

    // enable low-g interrupt
    uint8_t regv_int_funciton, regv_int_out, regv_int_latch;
    uint8_t rega_int_map, rega_int_io_ctrl, rega_latch;

    BMI26X_INST_LOG(MED, inst, "dbtap cfg:%d %d %d %d %d /%d",
                    istate->dbtap_info.client_present,
                    istate->int_en_flags_req.bits.dbtap,
                    en,
                    dbtap_armed_event,
                    trigger,
                    istate->dbtap_info.dbtap_event_num);

#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
    rega_int_map = BMI26X_REGA_USR_INT2_MAP;
    rega_int_io_ctrl = BMI26X_REGA_USR_INT2_IO_CTRL;
#else
    rega_int_map = BMI26X_REGA_USR_INT1_MAP;
    rega_int_io_ctrl = BMI26X_REGA_USR_INT1_IO_CTRL;
    rega_latch = BMI26X_REGA_USR_INT_LATCH;
    UNUSED_VAR(rega_latch);
#endif

    // BMI26X_REGA_USR_INT2_MAP
    // istate->scp_service, &istate->com_port_info
    rc = bmi26x_com_read_byte(istate->scp_service, &istate->com_port_info, rega_int_map, &regv_int_funciton);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // BMI26X_REGA_USR_INT2_IO_CTRL
    rc = bmi26x_com_read_byte(istate->scp_service, &istate->com_port_info, rega_int_io_ctrl, &regv_int_out);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (en) {
        regv_int_funciton = BST_SET_VAL_BIT(regv_int_funciton, 3);
        regv_int_latch = 1;
#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
        regv_int_out = BST_SET_VAL_BITBLOCK(regv_int_out, 1, 4, 0x05);
#endif
        istate->dbtap_info.dbtap_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_ENABLED;
    } else {
        regv_int_funciton = BST_CLR_VAL_BIT(regv_int_funciton, 3);
        regv_int_latch = 0;
#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
        regv_int_out = BST_SET_VAL_BITBLOCK(regv_int_out, 1, 4, 0x00);
#endif
        istate->dbtap_info.dbtap_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_DISABLED;
    }
    UNUSED_VAR(regv_int_latch);

    // write
#if 0
    rc = bmi26x_com_write_byte(istate->scp_service, &istate->com_port_info, rega_int_map, &regv_int_funciton);
#endif

    rc = bmi26x_sbus_write_wrapper(istate, rega_int_map, &regv_int_funciton, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);

#if BMI26X_CONFIG_ENABLE_LOWG_ON_INT2
    rc = bmi26x_com_write_byte(istate->scp_service, istate->port_info, rega_int_io_ctrl, &regv_int_out);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(BMI26X_SPEC_IF_IDLE_TIME_NORMAL_US);
#endif

#if BMI26X_CONFIG_ENABLE_AVOID_DUPLIVATED_EVENT
    if (en || dbtap_armed_event) {
        bool avoid_duplicate_dbtap_ev;

        avoid_duplicate_dbtap_ev = (istate->int_en_flags_req.bits.dbtap && istate->int_en_flags_curr.bits.dbtap)
                                && (!dbtap_armed_event);

        BMI26X_INST_LOG(MED, istate->owner,
                        "bmi26x_update_dbtap_intr type:%d %d",
                        istate->dbtap_info.dbtap_state.double_tap_event_type,
                        istate->int_en_flags_req.bits.dbtap | (istate->int_en_flags_curr.bits.dbtap << 1) |
                            (istate->dbtap_info.client_present << 2));

        if (!avoid_duplicate_dbtap_ev) {
            bmi26x_hal_send_dbtap_event(istate, &istate->dbtap_info.dbtap_state,
                                       sns_get_system_time());
        }
    }
#else
    BMI26X_INST_LOG(MED, istate->owner,
                    "bmi26x_update_lowg_intr md_ev_type:%d %d",
                    istate->lowg_info.lowg_state.free_fall_event_type,
                    istate->int_en_flags_req.bits.lowg | (istate->int_en_flags_curr.bits.lowg << 1) |
                    (istate->lowg_info.client_present << 2));

    bmi26x_hal_send_lowg_event(istate, &istate->lowg_info.lowg_state,
                               sns_get_system_time());
#endif

    return rc;
}

static void bmi26x_hal_cfg_dbtap_param(bmi26x_instance_state *istate)
{
    //
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t dbtap_buffer[16] = {0};
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint16_t *dbtap_cfg = NULL;
    uint16_t cfg_stream_1[] = {5, 0, 120, 2, 4, 8, 80};
    uint16_t cfg_stream_2[] = {1, 160, 2, 2, 2};
    uint32_t idx = 0;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_1, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return ;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    // [0x29
    dbtap_cfg = (uint16_t *) dbtap_buffer;
    for (idx = 0; idx < ARR_SIZE(cfg_stream_1); idx ++) {
        dbtap_cfg[idx + 1] = cfg_stream_1[idx];
    }

    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, dbtap_buffer, BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    if (rc != SNS_RC_SUCCESS) {
        return ;
    }

    // debug
    {
        sns_memset(dbtap_buffer, 0, sizeof(dbtap_buffer));
        dbtap_cfg = (uint16_t *) dbtap_buffer;
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);

        for (idx = 0; idx < (sizeof(dbtap_buffer) / sizeof(uint16_t)); idx ++) {
            BMI26X_INST_LOG(MED, istate->owner, "pg:%d, offset:%d, 0x%x ",
                    page_num, idx,
                    dbtap_cfg[idx]);
        }

    }
    // @debug

    // [0x30, 0x24]
    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_DOUBLE_TAP_CFG_2, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return ;
    }

    sns_memset(dbtap_buffer, 0, sizeof(dbtap_buffer));
    dbtap_cfg = (uint16_t *) dbtap_buffer;
    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    for (idx = 0; idx < ARR_SIZE(cfg_stream_2); idx ++) {
        dbtap_cfg[idx] = cfg_stream_2[idx];
    }

    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, dbtap_buffer, BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
    if (rc != SNS_RC_SUCCESS) {
        return ;
    }

    // debug
    {
        sns_memset(dbtap_buffer, 0, sizeof(dbtap_buffer));
        dbtap_cfg = (uint16_t *) dbtap_buffer;
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, dbtap_buffer,
                             BMI26X_FEATURE_DOUBLE_TAP_PARAM_SIZE);
        UNUSED_VAR(rc);

        for (idx = 0; idx < (sizeof(dbtap_buffer) / sizeof(uint16_t)); idx ++) {
            BMI26X_INST_LOG(MED, istate->owner, "pg:%d, offset:%d, 0x%x ",
                    page_num, idx,
                    dbtap_cfg[idx]);
        }

    }
    // @debug

}

sns_rc bmi26x_hal_inst_set_dbtap_config(bmi26x_instance_state *istate, bool en)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint16_t tap_sens_thres, max_gest_dur, quite_time_after_gest;
    bool lp_need = false;

    BMI26X_INST_LOG(MED, istate->owner, "set dbtap cfg:0x%02x",
                    (en | istate->dbtap_info.enable_dbtap_int << 1));
#if BMI26X_CONFIG_ENABLE_REGISTRY
    tap_sens_thres = istate->dbtap_info.sstate->dbtap_cfg.tap_sens_thres;
    max_gest_dur = istate->dbtap_info.sstate->dbtap_cfg.max_gest_dur;
    quite_time_after_gest = istate->dbtap_info.sstate->dbtap_cfg.quite_time_after_gest;
#else
    tap_sens_thres = BMI26X_DOUBLE_TAP_SENSITIVITY_THRESHOLD_MAX;
    max_gest_dur = BMI26X_DOUBLE_TAP_MAX_GEST_DUR;
    quite_time_after_gest = BMI26X_DOUBLE_TAP_QUITE_TIME_AFTER_GEST;

#endif

    // dump cfg input
    BMI26X_INST_LOG(MED, istate->owner, "dbtap cfg input:%d %d %d",
            tap_sens_thres, max_gest_dur, quite_time_after_gest);

    // prepare cfg
    if (tap_sens_thres >= BMI26X_DOUBLE_TAP_SENSITIVITY_THRESHOLD_MAX) {
        tap_sens_thres = BMI26X_DOUBLE_TAP_SENSITIVITY_THRESHOLD_MAX;
    }

    max_gest_dur = max_gest_dur / 5;
    quite_time_after_gest = quite_time_after_gest / 5;

    // dump cfg input
    BMI26X_INST_LOG(MED, istate->owner, "dbtap convert:%d %d %d",
            tap_sens_thres, max_gest_dur, quite_time_after_gest)

    // </prepare cfg>


    if (en != istate->dbtap_info.enable_dbtap_int) {
        // check sensor power mode
        if (!istate->sbus_in_normal_mode) {
            if (istate->regv_cache.regv_pwr_conf & 0x01) {
                lp_need = 1;
            }
            rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
        }

        // sync to configuration file
        rc = bmi26x_hal_set_dbtap_config(istate, en, tap_sens_thres, max_gest_dur, quite_time_after_gest);

        // enable acc
        if (en) {
            rc |= bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_ACC_NORMAL);
            // XXX put acc to high performance mode
            //uint8_t regv = 0x0A;
            //rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_CONF, 4, 7, regv);
        }

        if (lp_need) {
            rc |= bmi26x_dev_pwr_conf(istate, 0, 0, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

        istate->dbtap_info.enable_dbtap_int = en;

        /*if (en)*/ {
            bmi26x_hal_cfg_dbtap_param(istate);
        }
    }

    // advanced configuration

    return rc;
}


static void bmi26x_hal_interrupt_dbtap_cleanup(bmi26x_instance_state   *istate)
{
#if 0
    sns_rc rc = SNS_RC_SUCCESS;
    //uint8_t regv = 0x00;

    istate->int_en_flags_req.bits.dbtap = false;

    rc = bmi26x_hal_config_int_dbtap(istate->owner, false, true,
                                    (uint8_t)BMI26X_DBTAP_CONFIG_DBTP_CLEARUP);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! cfg int dbtap error");
        return ;
    }

    bmi26x_hal_set_dbtap_config(istate, false, BMI26X_DOUBLE_TAP_DEFAULT_SENSITIVITY_THRESHOLD_MAX,
                                BMI26X_DOUBLE_TAP_DEFAULT_REGV_MAX_GEST_DUR,
                                BMI26X_DOUBLE_TAP_DEFAULT_REGV_QUITE_TIME_AFTER_GEST);

    // clear the latch
    //regv = 0;
    //rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_INT_LATCH, &regv, 1);


    istate->int_en_flags_curr.bits.dbtap = false;
    istate->dbtap_info.dbtap_wait_for_fired_event = 0;
    istate->dbtap_info.enable_dbtap_int = 0;

    istate->dbtap_info.dbtap_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_FIRED;
#else
    // Do nothing as req
    UNUSED_VAR(istate);
#endif
}

void bmi26x_hal_handle_interrupt_dbtap(
        bmi26x_instance_state   *istate,
        sns_time                irq_timestamp)
{
    sns_sensor_instance         *instance = istate->owner;

    // 1. send double tap fired event
    sns_double_tap_event dbtap_state;
    dbtap_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_FIRED;

    BMI26X_INST_LOG(HIGH, instance, "!!! DOUBLE TAP fired! dbtap.cp:%d evt:%d dbtap_evt_num:%u @ts:%u",
                    istate->dbtap_info.client_present,
                    dbtap_state.double_tap_event_type,
                    (uint32_t)(istate->dbtap_info.dbtap_event_num + 1),
                    (uint32_t)irq_timestamp);

    bmi26x_hal_send_dbtap_event(istate, &dbtap_state, irq_timestamp);

    istate->dbtap_info.dbtap_state.double_tap_event_type = dbtap_state.double_tap_event_type;

    istate->dbtap_info.dbtap_event_num ++;
    istate->dbtap_info.ts_latest_event_fired = irq_timestamp;

    // disable dbtap int and send out dbtap disabled event
    bmi26x_hal_interrupt_dbtap_cleanup(istate);
}


#endif

