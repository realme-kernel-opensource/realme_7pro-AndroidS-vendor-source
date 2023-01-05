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

/**
 * @file sns_bmi26x_instance.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/
#include <stdint.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_types.h"

#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor.h"
#include "sns_bmi26x_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#endif

#include "sns_cal.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

#include "sns_sync_com_port_service.h"

/**
 * Instance cleanup
 * @param istate        instance handler
 * @param stream_mgr    stream manager
 */
static void bmi26x_inst_cleanup(sns_sensor_instance *const  this, sns_stream_service *stream_mgr)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;

    BMI26X_INST_LOG(MED, this, "<bmi26x_if inst_cleanup>");

    UNUSED_VAR(stream_mgr);

    // TODO: Is this OK to do here? Introduced by DAE sensor, since we could be
    // waiting for response from DAE before writing to HW -- but we don't have that chance.
    if (NULL != istate->com_port_info.port_handle) {
        istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle, true);
#if !BMI26X_CONFIG_ENABLE_POWER_KEEP_ALIVE
        sns_rc rc = SNS_RC_SUCCESS;
        uint8_t regv = BMI26X_REGV_CMD_SOFT_RESET;
        //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_SOFT_RESET);
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(MED, this, "reset failure:0x%x", rc);
        }
        bmi26x_delay_us(2000);   //2ms

        // debug
        //regv = 0;
       // bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CTRL, &regv, 1);
#else

        sns_rc rc = SNS_RC_SUCCESS;
        uint8_t regv = BMI26X_REGV_CMD_SOFT_RESET;
        //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_SOFT_RESET);
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(MED, this, "reset failure:0x%x", rc);
        }
        bmi26x_delay_us(2000);   //2ms

        istate->inst_inted = 0;
        /* prepare sensor*/
        bmi26x_hal_prepare_hw(istate);

        /* load configuration */
        bmi26x_hal_load_sensor_cfg(this);
        // set the sensor to suspend mode
        bmi26x_hal_config_power_mode(istate, (uint8_t) BMI26X_POWER_MODE_SUSPEND);
#endif
        istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle, false);
    }

    sns_sensor_util_remove_sensor_instance_stream(this, &istate->interrupt_data_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &istate->async_com_port_data_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &istate->timer_data_stream);
    sns_sensor_util_remove_sensor_instance_stream(this, &istate->cmd_handler.timer_cmd_stream);

#if BMI26X_CONFIG_ENABLE_CRT
    sns_sensor_util_remove_sensor_instance_stream(this, &istate->crt_handler.timer_crt_stream);
#endif

#if BMI26X_CONFIG_ENABLE_PEDO_TIMER
    if (istate->pedo_info.pedo_timer_data_stream != NULL) {
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->pedo_info.pedo_timer_data_stream);
    }
#endif

#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    if (istate->hb_cfg_info.timer_heart_beat_data_stream != NULL) {
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->hb_cfg_info.timer_heart_beat_data_stream);
        istate->hb_cfg_info.timer_heart_beat_data_stream = NULL;
    }
#endif

    if (NULL != istate->scp_service) {
        istate->scp_service->api->sns_scp_close(istate->com_port_info.port_handle);
        istate->scp_service->api->sns_scp_deregister_com_port(&istate->com_port_info.port_handle);
        istate->scp_service = NULL;
    }

#if BMI26X_CONFIG_ENABLE_DAE
    bmi26x_dae_if_deinit(this);
#endif
}

// <sensor test>
/**
 * Runs a communication test - verfies WHO_AM_I, publishes self
 * test event.
 *
 * @param[i] instance  Instance reference
 * @param[i] uid       Sensor UID
 *
 * @return none
 */
static void bmi26x_send_self_test_event(
    sns_sensor_instance     *instance,
    const sns_sensor_uid    *uid,
    bool                    test_result,
    sns_physical_sensor_test_type type,
    bmi26x_test_err_code_t    err_code)

{
    uint8_t data[1] = {(uint8_t)err_code};
    pb_buffer_arg buff_arg = (pb_buffer_arg) {
        .buf = &data, .buf_len = sizeof(data)
    };
    sns_physical_sensor_test_event test_event =
        sns_physical_sensor_test_event_init_default;

    test_event.test_passed = test_result;
    test_event.test_type   = type;
    test_event.test_data.funcs.encode = pb_encode_string_cb;
    test_event.test_data.arg = &buff_arg;

    BMI26X_INST_LOG(HIGH, instance, "bmi26x self_test_ev: %d %d %d",
                    test_result, err_code, type);
    pb_send_event(instance,
                  sns_physical_sensor_test_event_fields,
                  &test_event,
                  sns_get_system_time(),
                  SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                  uid);
}

#if BMI26X_CONFIG_ENABLE_SELF_TEST_HW_SW

/*! @name Structure to define accelerometer and gyroscope sensor axes and
 * sensor time for virtual frames
 */
struct bmi2xy_sens_axes_data {
    /*! Data in x-axis */
    int16_t x;

    /*! Data in y-axis */
    int16_t y;

    /*! Data in z-axis */
    int16_t z;

    /*! Sensor time for virtual frames */
    uint32_t virt_sens_time;
};

/*! @name Structure to define the difference in accelerometer values  */
struct selftest_delta_limit {
    /*! X  data */
    int32_t x;

    /*! Y  data */
    int32_t y;

    /*! Z  data */
    int32_t z;
};



static
void bmi26x_parse_sensor_data(
    uint8_t                 *buf,
    uint8_t                 len,
    struct bmi2xy_sens_axes_data  *val)
{
    UNUSED_VAR(len);
    val->x = (buf[1] << 8) | (buf[0]);
    val->y = (buf[3] << 8) | (buf[2]);
    val->z = (buf[5] << 8) | (buf[4]);
}


static sns_rc bmi26x_set_self_test_config(bmi26x_instance_state *istate,
        bmi26x_sensor_type sensor_type,
        bool is_positive)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;
    uint8_t rega = BMI26X_REGA_USR_ACC_SELF_TEST;
    uint8_t bit_start, bit_end;

    if (sensor_type == BMI26X_ACCEL) {
        rega = BMI26X_REGA_USR_ACC_SELF_TEST;
        bit_start = 0;
        bit_end   = 2;
        regv = BST_SET_VAL_BITBLOCK(regv, 0, 0, BMI2XY_ENABLE);
        regv = BST_SET_VAL_BITBLOCK(regv, 2, 2, is_positive);
    } else {
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_dev_reg_read_modify_write(istate, rega,
                                          bit_start, bit_end, regv);
    return rc;
}

static sns_rc bmi26x_enable_sensor_self_test(bmi26x_instance_state *istate,
        bmi26x_sensor_type sensor_type,
        bool enable)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t rega = 0;

    if (sensor_type == BMI26X_ACCEL) {
        rega = BMI26X_REGA_USR_ACC_SELF_TEST;
    } else {
        return SNS_RC_SUCCESS;
    }

    rc = bmi26x_dev_reg_read_modify_write(istate, rega, 0, 0, enable);
    return rc;
}


static sns_rc bmi26x_read_sensor_xyz_val(bmi26x_instance_state *istate, bmi26x_sensor_type sensor_type,
        struct bmi2xy_sens_axes_data *frame_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t rega = 0x00;
    uint8_t regv[BMI26X_ACC_GYRO_ONE_FRAME_SIZE_IN_BYTES] = {0};

    if (sensor_type == BMI26X_ACCEL) {
        rega = BMI26X_REGA_USR_DATA_ACC_X_LSB;
    } else if (sensor_type == BMI26X_GYRO) {
        rega = BMI26X_REGA_USR_DATA_GYR_X_LSB;
    } else {
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_sbus_read_wrapper(istate, rega, regv,
                                  BMI26X_ACC_GYRO_ONE_FRAME_SIZE_IN_BYTES);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    bmi26x_parse_sensor_data(regv, BMI26X_ACC_GYRO_ONE_FRAME_SIZE_IN_BYTES, frame_buffer);

    BMI26X_INST_LOG(MED, istate->owner, "frame val:<%d %d %d>",
                    frame_buffer->x, frame_buffer->y, frame_buffer->z)

    return rc;
}

static sns_rc bmi26x_pre_self_test_hw_config(bmi26x_instance_state *istate,
        bmi26x_sensor_type sensor_type)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv[2] = {0};

    if (sensor_type == BMI26X_ACCEL) {
        /* Set self test amplitude low */
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_ACC_SELF_TEST,
                                              3, 3, BMI2XY_ENABLE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        // prepare the sensor configuration
        // odr, bwp, fileter_perf
        // 1600hz,4,1
        regv[0] = 0xAC;
        // range
        // +/- 16G
        regv[1] = 0x03;

        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_ACC_CONF, regv, 2);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        bmi26x_delay_us(2 * 1000); //20ms
    }

    return rc;
}

/*!
 * @brief This internal API converts LSB value of accelerometer axes to form
 * 'g' to 'mg' for self-test.
 */
static void bmi26x_convert_lsb_g(bmi26x_instance_state *istate,
                                 const struct selftest_delta_limit *acc_data_diff,
                                 struct selftest_delta_limit *acc_data_diff_mg)
{
    bmi26x_sstvt_t acc_ssvt_array[] = {
        /* in the unit of micro-g/lsb */
        BMI26X_ACCEL_SSTVT_2G,
        BMI26X_ACCEL_SSTVT_4G,
        BMI26X_ACCEL_SSTVT_8G,
        BMI26X_ACCEL_SSTVT_16G,
    };

    /* Range considered for self-test is +/-16g */
    uint8_t range_idx = BMI26X_SENSOR_HW_TEST_ACC_RANGE_INDEX;
    int32_t                 sstvt_curr;

    sstvt_curr = roundf(acc_ssvt_array[range_idx] *
                        (istate->accel_info.sstate->scale_factor * 1e-6));

    // 1g * 10^7
    acc_data_diff_mg->x = (acc_data_diff->x * sstvt_curr) * 1e-4;
    acc_data_diff_mg->y = (acc_data_diff->y * sstvt_curr) * 1e-4;
    acc_data_diff_mg->z = (acc_data_diff->z * sstvt_curr) * 1e-4;

    BMI26X_INST_LOG(MED, istate->owner, "range:%d ssvt:%d <%d %d %d>",
                    range_idx, sstvt_curr,
                    acc_data_diff_mg->x, acc_data_diff_mg->y, acc_data_diff_mg->z)
}

/*!
 * @brief This internal API validates the accelerometer self-test data and
 * decides the result of self test operation.
 */
static int8_t bmi26x_validate_self_test(const struct selftest_delta_limit *accel_data_diff)
{
    /* Variable to define error */
    int8_t rslt;

    /* Validating accelerometer data by comparing with minimum and maximum
       difference signal value of the axes in */
    if ((accel_data_diff->x > BMI26X_ST_ACC_X_SIG_MIN_DIFF) &&
            (accel_data_diff->y < BMI26X_ST_ACC_Y_SIG_MIN_DIFF) &&
            (accel_data_diff->z > BMI26X_ST_ACC_Z_SIG_MIN_DIFF)) {
        /* Self-test pass */
        rslt = 0;
    } else {
        /* Self-test fail*/
        rslt = -1;
    }

    return rslt;
}

/*!
 * @brief This internal API enables and configures the accelerometer which is
 * needed for self test operation. It also sets the amplitude for the self-test.
 */
static sns_rc bmi26x_pre_self_test_config(bmi26x_instance_state *istate,
        bmi26x_sensor_type      sensor_type)
{
    sns_rc rc = SNS_RC_SUCCESS;

    // clear adv.power.save always
    rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (sensor_type == BMI26X_ACCEL) {
        /* Enable accelerometer */
        rc = bmi26x_dev_pwr_ctrl(istate, 2, 2, BMI2XY_ENABLE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(1 * 1000);
    } else if (sensor_type == BMI26X_GYRO) {
        /* Enable gyroscope*/
        // pwr_ctrl.gyr_en = 1 AND pwr_config.fup = 0
        rc = bmi26x_dev_pwr_conf(istate, 2, 2, 0);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi26x_dev_pwr_ctrl(istate, 1, 1, BMI2XY_ENABLE);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(50 * 1000);   // delay 50ms after gyro enable
    } else {
        return SNS_RC_INVALID_TYPE;
    }

    return rc;
}


static sns_rc bmi26x_run_sensor_hw_test(bmi26x_instance_state *istate,
                                        bmi26x_sensor_type      sensor_type)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bool polarity_positive = true;
    struct bmi2xy_sens_axes_data  sensor_data[2] = { {0, 0, 0, 0}, {0, 0, 0, 0} };

    /* Enable accelerometer */
    if (sensor_type == BMI26X_ACCEL) {
        /* Structure for difference of accelerometer values in g */
        struct selftest_delta_limit accel_data_diff = { 0, 0, 0 };
        /* Structure for difference of accelerometer values in mg */
        struct selftest_delta_limit accel_data_diff_mg = { 0, 0, 0 };

        // +++ positive
        rc = bmi26x_pre_self_test_hw_config(istate, sensor_type);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        // enable self test
        BMI26X_INST_LOG(MED, istate->owner, "start test on positive:%d", polarity_positive);
        rc = bmi26x_set_self_test_config(istate, BMI26X_ACCEL, polarity_positive);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        bmi26x_delay_us(50 * 1000); //100ms

        // read sensor data
        rc = bmi26x_read_sensor_xyz_val(istate, sensor_type, &sensor_data[polarity_positive]);
        if (rc != SNS_RC_SUCCESS) {
            // TODO
        }

        // --- negative
        polarity_positive = !polarity_positive;
        rc = bmi26x_pre_self_test_hw_config(istate, sensor_type);
        if (rc != SNS_RC_SUCCESS) {
            // TODO
        }

        // enable self test
        BMI26X_INST_LOG(MED, istate->owner, "start test on positive:%d", polarity_positive);
        rc = bmi26x_set_self_test_config(istate, BMI26X_ACCEL, polarity_positive);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(50 * 1000); //100ms

        // read sensor data
        rc = bmi26x_read_sensor_xyz_val(istate, sensor_type, &sensor_data[polarity_positive]);
        if (rc != SNS_RC_SUCCESS) {
            // TODO
        }

        /* Subtract -ve acceleration values from that of
         +ve values */
        accel_data_diff.x = (sensor_data[1].x) - (sensor_data[0].x);
        accel_data_diff.y = (sensor_data[1].y) - (sensor_data[0].y);
        accel_data_diff.z = (sensor_data[1].z) - (sensor_data[0].z);

        /* Convert differences of acceleration values
         * from 'g' to 'mg'
         */
        bmi26x_convert_lsb_g(istate, &accel_data_diff, &accel_data_diff_mg);

        BMI26X_INST_LOG(MED, istate->owner, "diff <%d %d %d> <%d %d %d>",
                        accel_data_diff.x, accel_data_diff.y, accel_data_diff.z,
                        (int32_t)(accel_data_diff_mg.x / 1000),
                        (int32_t)(accel_data_diff_mg.y / 1000),
                        (int32_t)(accel_data_diff_mg.z / 1000));

        /* Validate self test for acceleration values
         * in mg and get the self-test result
         */
        if ((sns_rc) (bmi26x_validate_self_test(&accel_data_diff_mg)) ==
                SNS_RC_SUCCESS) {
            rc = SNS_RC_SUCCESS;
        } else {
            rc = SNS_RC_INVALID_STATE;
        }

    } else if (sensor_type == BMI26X_GYRO) {
        uint8_t regv = 0;
#if BMI26X_CONFIG_ENABLE_GYRO_SELF_TEST_ONESHOT
        rc = bmi26x_hal_send_cmd(istate, BMI26X_CMD_RUN_GYRO_SELF_TEST);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        // wait the slef test finish
        bmi26x_delay_us(40 * 1000);

        do {
            regv = 0;
            rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_SELF_TEST_AXIS, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            BMI26X_INST_LOG(MED, istate->owner, "gyr selftest axis st:0x%x", regv);

            if (BST_GET_VAL_BITBLOCK(regv, 1, 3) == 0x07) {
                rc = SNS_RC_SUCCESS;
            }
            bmi26x_delay_us(10 * 1000);
        } while ((regv & 0x01) == 0);
#else
        // mems level self test
        regv = BMI2XY_ENABLE;
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_GYR_SELF_TEST, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(20 * 1000);

        do {
            regv = 0;
            rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_GYR_SELF_TEST, &regv, 1);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            BMI26X_INST_LOG(MED, istate->owner, "gyr selftest mems: 0x%x", regv);

            if (BST_GET_VAL_BIT(regv, 0) == 0) {
                if (BST_GET_VAL_BIT(regv, 1)) {
                    rc = SNS_RC_SUCCESS;
                }
            } else {
            }
            bmi26x_delay_us(5 * 1000);
        } while (BST_GET_VAL_BIT(regv, 0));
#endif

    } else {
        return SNS_RC_INVALID_TYPE;
    }

    // disable sensor self-test
    rc = bmi26x_enable_sensor_self_test(istate, sensor_type, BMI2XY_DISABLE);

    return rc;
}


static
sns_rc bmi26x_run_hw_sw_test(
    sns_sensor_instance     *instance,
    bmi26x_sensor_type      sensor_type,
    bmi26x_test_err_code_t *err_code)
{
    sns_rc                  rc = SNS_RC_SUCCESS;
    uint8_t regv;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) instance->state->state;

    BMI26X_INST_LOG(MED, instance, "h/w self test start");

    //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_SOFT_RESET);
    regv = BMI26X_REGV_CMD_SOFT_RESET;
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    bmi26x_delay_us(10 * 1000);

    // reset the sensor status
    rc = bmi26x_hal_reset_device_wrapper(instance);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_hal_load_sensor_cfg(instance);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_pre_self_test_config(istate, sensor_type);

    rc = bmi26x_run_sensor_hw_test(istate, sensor_type);
    BMI26X_INST_LOG(MED, instance, "!!! HW/SW test finised on sensor:%d, %d",
                    sensor_type, rc);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(MED, instance, "!!! HW/SW test PASSED");

    //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_SOFT_RESET);
    regv = BMI26X_REGV_CMD_SOFT_RESET;
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    bmi26x_delay_us(10 * 1000);

    *err_code = BMI26X_FAC_TEST_NO_ERROR;

    return rc;
}
#endif

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
static void bmi26x_send_self_test_event_bias(sns_sensor_instance *instance,
        sns_sensor_uid *uid, bool test_result,
        sns_physical_sensor_oem_config_type type)
{
    sns_physical_sensor_oem_config_event test_event =
        sns_physical_sensor_oem_config_event_init_default;

    test_event.config_result = test_result;
    test_event.config_type = type;

    pb_send_event(instance,
                  sns_physical_sensor_oem_config_event_fields,
                  &test_event,
                  sns_get_system_time(),
                  SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG_EVENT,
                  uid);
}
#endif

#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC

static void bmi26x_clear_curr_req(bmi26x_instance_state *istate)
{
    istate->int_en_flags_req.bits.md = 0;
    istate->int_en_flags_curr.bits.md = 0;
    istate->md_info.client_present = 0;

    istate->hw_mod_needed = 0;
    istate->fifo_info.publish_sensors = 0;
    //istate->fifo_info.ff_has_max_batch_req = 0;

    //istate->passive_client_present_on_sensors = 0;
    //istate->accel_info.sstate->gated_client_present = 0;
    //istate->accel_info.sstate->ngated_client_present = 1;

    //istate->gyro_info.sstate->gated_client_present = 0;
    //istate->gyro_info.sstate->ngated_client_present = 1;

    bmi26x_hal_set_md_config(istate->owner, false);
    bmi26x_hal_config_int_md(istate->owner,  false, false, (uint8_t)BMI26X_MD_CONFIG_HW_CFG);
}



static
void bmi26x_test_fac_req_hw(sns_sensor_instance     *instance)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)instance->state->state;
    BMI26X_INST_LOG(MED, instance, "fac_test req_hw");

    //istate->hw_mod_needed = 0;
    //istate->fifo_info.publish_sensors = 0;

    bmi26x_clear_curr_req(istate);

    if (BMI26X_ACCEL == istate->fac_test_info.fac_test_sensor) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors |= BMI26X_ACCEL;


        istate->accel_info.sample_rate_req = 100;
        istate->accel_info.report_rate_req = istate->accel_info.sample_rate_req;
    } else if (BMI26X_GYRO == istate->fac_test_info.fac_test_sensor) {
        istate->hw_mod_needed |= BMI26X_GYRO;
        istate->fifo_info.publish_sensors |= BMI26X_GYRO;


        istate->gyro_info.sample_rate_req = 100;
        istate->gyro_info.report_rate_req = istate->gyro_info.sample_rate_req;
    }
#if BMI26X_CONFIG_ENABLE_PEDO
    else if (BMI26X_PEDO == istate->fac_test_info.fac_test_sensor) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->pedo_info.enable_pedo_int = 1;
    }
#endif
    //TODOMAG

    if (
#if BMI26X_CONFIG_ENABLE_DAE
        !bmi26x_dae_if_available(instance)
#else
        1
#endif
    ) {
        bmi26x_hal_register_interrupt(instance);
        istate->irq_ready = true;
    }

    bmi26x_inst_assess_overall_req(istate);
    if (bmi26x_hal_reconfig_hw(instance, HW_CONFIG_CTX_BEFORE_FAC_START) != SNS_RC_SUCCESS) {
    }
}


static
void bmi26x_test_fac_rls_hw(sns_sensor_instance     *instance)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)instance->state->state;

    BMI26X_INST_LOG(MED, instance, "fac_test rls_hw");

    istate->hw_mod_needed = 0;
    istate->fifo_info.publish_sensors  = 0;

    if (bmi26x_hal_reconfig_hw(instance, HW_CONFIG_CTX_AFTER_FAC) != SNS_RC_SUCCESS) {
    }
}





static void bmi26x_inst_test_fac_recovery_process(sns_sensor_instance * inst)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;

    istate->accel_info.sample_rate_req = istate->accel_info.backup_before_fac_sample_rate_req;
    istate->accel_info.report_rate_req = istate->accel_info.backup_before_fac_report_rate_req;
    istate->gyro_info.sample_rate_req  = istate->gyro_info.backup_before_fac_sample_rate_req;
    istate->gyro_info.report_rate_req  = istate->gyro_info.backup_before_fac_report_rate_req;

    bmi26x_hal_set_sensor_state_2_default_state(inst, (uint8_t) (BMI26X_ACCEL | BMI26X_GYRO));

    if (!BST_IS_FLOAT_ZERO(istate->accel_info.sample_rate_req)) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors  |= BMI26X_ACCEL;
    }

    if (!BST_IS_FLOAT_ZERO(istate->gyro_info.sample_rate_req)) {
        istate->hw_mod_needed |= BMI26X_GYRO;
        istate->fifo_info.publish_sensors  |= BMI26X_GYRO;
    }


    // trigger a request assessing
    bmi26x_inst_assess_overall_req(istate);
    if (bmi26x_hal_reconfig_hw(inst, HW_CONFIG_CTX_AFTER_FAC) != SNS_RC_SUCCESS) {
        // TODO
    }
}

#endif

int bmi26x_start_factory_test(
    sns_sensor_instance     *instance,
    bmi26x_sensor_type      sensor,
    bmi26x_test_err_code_t    *err_code)
{
    int rc = SNS_RC_SUCCESS;

    UNUSED_VAR(instance);
    UNUSED_VAR(sensor);
    UNUSED_VAR(err_code);
#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    bmi26x_instance_state *istate = (bmi26x_instance_state*)instance->state->state;


    if (istate->fifo_info.ff_sensors_en_req) {
        BMI26X_INST_LOG(HIGH, instance,
                        "WARNING!!! normal streaming is in process, will be paused now and resumed after fac test finishes");
    }

    istate->fac_test_info.num_samples = 0;
    istate->fac_test_info.sample_square_sum[0] = 0;
    istate->fac_test_info.sample_square_sum[1] = 0;
    istate->fac_test_info.sample_square_sum[2] = 0;
    istate->fac_test_info.sample_sum[0] = 0;
    istate->fac_test_info.sample_sum[1] = 0;
    istate->fac_test_info.sample_sum[2] = 0;
    istate->fac_test_in_progress = true;
    istate->fac_test_info.fac_test_sensor = sensor;
    istate->fac_test_info.at_rest = true;

    if (BMI26X_ACCEL == sensor) {
        istate->fac_test_info.bias_thresholds[0] = 300 * G / 1000; // 300 m/s2
        istate->fac_test_info.bias_thresholds[1] = 300 * G / 1000; // 300 m/s2
        istate->fac_test_info.bias_thresholds[2] = 360 * G / 1000; // 360 m/s2
        istate->fac_test_info.variance_threshold = 360.0; // 360(m/s2)2
    } else if (BMI26X_GYRO == sensor) {
        istate->fac_test_info.bias_thresholds[0] = 40 * PI / 180; //40 rad/sec
        istate->fac_test_info.bias_thresholds[1] = 40 * PI / 180; //40 rad/sec
        istate->fac_test_info.bias_thresholds[2] = 40 * PI / 180; //40 rad/sec
        istate->fac_test_info.variance_threshold = 100.0; // 100(rad/sec)2
    }
#if BMI26X_CONFIG_ENABLE_PEDO
    else if (BMI26X_PEDO == sensor) {
        istate->keep_pedo_in_fac_mode = 1;
        BMI26X_INST_LOG(MED, instance, "fac self test for pedo: %d", istate->keep_pedo_in_fac_mode);
        bmi26x_send_self_test_event(instance, (&((sns_sensor_uid) PEDO_SUID)), true, SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY,
                                    BMI26X_FAC_TEST_NO_ERROR);
        istate->fac_test_in_progress = false;
    }
#endif
    else {
        BMI26X_INST_LOG(ERROR, instance, "fac self test not supported for sensor: %d", sensor);
        return SNS_RC_NOT_SUPPORTED;
    }

    BMI26X_INST_LOG(HIGH, instance, "start_factory_test() sensor %d", sensor);

    bmi26x_test_fac_req_hw(instance);
#endif

    return rc;
}


#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
void bmi26x_run_self_test_bias(sns_sensor_instance *instance)
{
    bmi26x_instance_state *state = (bmi26x_instance_state*)instance->state->state;

    BMI26X_INST_LOG(HIGH, instance, "bmi26x : bmi26x_run_self_test_bias");
    if (state->accel_info.test_info_bias.test_client_present) {
        if (state->accel_info.test_info_bias.config_type == SNS_PHYSICAL_SENSOR_CONFIG_BIAS) {
            // Handle factory test. The driver may choose to reject any new
            // streaming/self-test requests when factory test is in progress.
            /** update_fac_cal_in_registry is used to demonstrate a registry write operation.*/
            /** Using dummy data for registry write demonstration. */
            state->accel_info.sstate->fac_cal_bias[0] = roundf(state->accel_info.test_info_bias.offset_x *
                    BMI26X_SCALE_FACTOR_DATA_ACCEL);
            state->accel_info.sstate->fac_cal_bias[1] = roundf(state->accel_info.test_info_bias.offset_y *
                    BMI26X_SCALE_FACTOR_DATA_ACCEL);
            state->accel_info.sstate->fac_cal_bias[2] = roundf(state->accel_info.test_info_bias.offset_z *
                    BMI26X_SCALE_FACTOR_DATA_ACCEL);
            state->update_fac_cal_in_registry = true;
            state->accel_info.sstate->fac_cal_version++;
            BMI26X_INST_LOG(HIGH, instance, "bmi : Custom Accel Calibration [version:%d][x:%d][y:%d][z:%d]",
                            state->accel_info.sstate->fac_cal_version,
                            (int)(state->accel_info.sstate->fac_cal_bias[0]),
                            (int)(state->accel_info.sstate->fac_cal_bias[1]),
                            (int)(state->accel_info.sstate->fac_cal_bias[2]));

            bmi26x_send_self_test_event_bias(instance, &state->accel_info.sstate->my_suid,
                                             true, state->accel_info.test_info_bias.config_type);
        }
        state->accel_info.test_info_bias.test_client_present = false;
    }
    if (state->gyro_info.test_info_bias.test_client_present) {
        if (state->gyro_info.test_info_bias.config_type == SNS_PHYSICAL_SENSOR_CONFIG_BIAS) {
            // Handle factory test. The driver may choose to reject any new
            // streaming/self-test requests when factory test is in progress.
            /** update_fac_cal_in_registry is used to demonstrate a registry write operation.*/
            /** Using dummy data for registry write demonstration. */
            state->gyro_info.sstate->fac_cal_bias[0] = roundf(state->gyro_info.test_info_bias.offset_x *
                    BMI26X_SCALE_FACTOR_DATA_GYRO);
            state->gyro_info.sstate->fac_cal_bias[1] = roundf(state->gyro_info.test_info_bias.offset_y *
                    BMI26X_SCALE_FACTOR_DATA_GYRO);
            state->gyro_info.sstate->fac_cal_bias[2] = roundf(state->gyro_info.test_info_bias.offset_z *
                    BMI26X_SCALE_FACTOR_DATA_GYRO);
            state->update_fac_cal_in_registry = true;
            state->gyro_info.sstate->fac_cal_version++;
            BMI26X_INST_LOG(HIGH, instance, "bmi26x : Custom Gyro Calibration [version:%d][x:%d][y:%d][z:%d]",
                            state->gyro_info.sstate->fac_cal_version,
                            (int)(state->gyro_info.sstate->fac_cal_bias[0]),
                            (int)(state->gyro_info.sstate->fac_cal_bias[1]),
                            (int)(state->gyro_info.sstate->fac_cal_bias[2]));

            bmi26x_send_self_test_event_bias(instance, &state->gyro_info.sstate->my_suid,
                                             true, state->gyro_info.test_info_bias.config_type);
        }
        state->gyro_info.test_info_bias.test_client_present = false;
    }
}
#endif

void bmi26x_reset_fac_cal_data(
    sns_sensor_instance *const  instance,
    struct bmi26x_state         *sstate,
    bool                        update_version)
{
    UNUSED_VAR(instance);

    memset(sstate->fac_cal_bias, 0, sizeof(sstate->fac_cal_bias));
    memset(&sstate->fac_cal_corr_mat, 0, sizeof(sstate->fac_cal_corr_mat));

    sstate->fac_cal_corr_mat.e00 = 1.0;
    sstate->fac_cal_corr_mat.e11 = 1.0;
    sstate->fac_cal_corr_mat.e22 = 1.0;

    if (update_version) {
        sstate->fac_cal_version ++;
    }
}

void bmi26x_process_fac_test(sns_sensor_instance *instance)
{
    UNUSED_VAR(instance);
#if BMI26X_CONFIG_ENABLE_SELF_TEST_FAC
    int i;
    bool                        test_pass = true;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    sns_sensor_uid              *uid;
    bmi26x_state                *sstate;
    float                       fac_cal_bias[3];
    bmi26x_test_err_code_t      err_code = BMI26X_FAC_TEST_NO_ERROR;

    BMI26X_INST_LOG(LOW, instance, "fac_test process_result for sensor: %d %d ",
                    istate->fac_test_info.fac_test_sensor,
                    istate->fac_test_info.num_samples);

    if (BMI26X_ACCEL == istate->fac_test_info.fac_test_sensor) {
        sstate = istate->accel_info.sstate;
    } else if (BMI26X_GYRO == istate->fac_test_info.fac_test_sensor) {
        sstate = istate->gyro_info.sstate;
    } else {
        return;
    }

    uid = &sstate->my_suid;

    //if(state->fac_test_info.num_samples == 64)
    if (istate->fac_test_info.num_samples >= 64) {
        if (BMI26X_GYRO == istate->fac_test_info.fac_test_sensor) {
            BMI26X_INST_LOG(LOW, instance, "fac sample sums: %d %d %d",
                        (int32_t)(istate->fac_test_info.sample_sum[0] * 1000000),
                        (int32_t)(istate->fac_test_info.sample_sum[1] * 1000000),
                        (int32_t)(istate->fac_test_info.sample_sum[2] * 1000000));
            BMI26X_INST_LOG(LOW, instance, "fac sample square sums: %d %d %d",
                        (int32_t)(istate->fac_test_info.sample_square_sum[0] * 1000000),
                        (int32_t)(istate->fac_test_info.sample_square_sum[1] * 1000000),
                        (int32_t)(istate->fac_test_info.sample_square_sum[2] * 1000000));
        } else {
            BMI26X_INST_LOG(LOW, instance, "fac sample sums: %d %d %d",
                        (int32_t)(istate->fac_test_info.sample_sum[0] * 1000),
                        (int32_t)(istate->fac_test_info.sample_sum[1] * 1000),
                        (int32_t)(istate->fac_test_info.sample_sum[2] * 1000));
            BMI26X_INST_LOG(LOW, instance, "fac sample square sums: %d %d %d",
                        (int32_t)(istate->fac_test_info.sample_square_sum[0] * 1000),
                        (int32_t)(istate->fac_test_info.sample_square_sum[1] * 1000),
                        (int32_t)(istate->fac_test_info.sample_square_sum[2] * 1000));
        }

        istate->fac_test_in_progress = false;
        bmi26x_test_fac_rls_hw(instance);

        fac_cal_bias[0] =
            istate->fac_test_info.sample_sum[0] / (istate->fac_test_info.num_samples);
        fac_cal_bias[1] =
            istate->fac_test_info.sample_sum[1] / (istate->fac_test_info.num_samples);
        fac_cal_bias[2] =
            istate->fac_test_info.sample_sum[2] / (istate->fac_test_info.num_samples);

        for (i = 0; i < TRIAXIS_NUM; i ++) {
            float varT = (istate->fac_test_info.sample_sum[i]) * (istate->fac_test_info.sample_sum[i]);

            istate->fac_test_info.variance[i] = (istate->fac_test_info.sample_square_sum[i]
                                                - (varT / istate->fac_test_info.num_samples)) / istate->fac_test_info.num_samples;

            // Check variance to determine whether device is stationary
            if (istate->fac_test_info.variance[i] > istate->fac_test_info.variance_threshold) {
                // device is not stationary
                istate->fac_test_info.at_rest = false;
                test_pass = false;
                err_code = BMI26X_FAC_TEST_DEV_NOT_STATIONARY;
                BMI26X_INST_LOG(ERROR, instance, "ERROR!!! fac_test FAILED device not stationary var[%u]=%u  %u",
                                i,
                                (uint32_t)istate->fac_test_info.variance[i],
                                (uint32_t)istate->fac_test_info.variance_threshold);
                break;
            }

            // Check biases are in defined limits
            if (fabsf(fac_cal_bias[i]) > istate->fac_test_info.bias_thresholds[i]) {
                test_pass = false;
                err_code = BMI26X_FAC_TEST_HIGH_BIAS;
                BMI26X_INST_LOG(ERROR, instance, "ERROR!!! fac_test FAILED bias very large: %d <%d %d>", i,
                                (int)(1000 * fac_cal_bias[i]),
                                (int)(1000 * istate->fac_test_info.bias_thresholds[i]));
                break;
            }

            // Check for zero variance
            //if (istate->fac_test_info.variance[i] == 0.0) {
            if (BST_IS_FLOAT_ZERO(istate->fac_test_info.variance[i])) {
                test_pass = false;
                err_code = BMI26X_FAC_TEST_ZERO_VARIANCE;
                BMI26X_INST_LOG(ERROR, instance, "ERROR!!! fac_test FAILED zero variance at axis:%d", i);
                break;
            }
        }

        /** update_fac_cal_in_registry if test is successful.*/
        istate->update_fac_cal_in_registry = test_pass;

        bmi26x_send_self_test_event(instance, uid,
                                    test_pass, SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY, err_code);

        if (test_pass) {
            for (i = 0; i < TRIAXIS_NUM; i++) {
                sstate->fac_cal_bias[i] = roundf(fac_cal_bias[i] * sstate->scale_factor);
            }

            sstate->fac_cal_version ++;

            bmi26x_send_fac_cal_event(instance, sstate);
            BMI26X_INST_LOG(HIGH, instance, "CONGRATULATIONS!!! fac_test SUCCESS");
        }

        // restore configuration after fac
        bmi26x_inst_test_fac_recovery_process(instance);
    }
#endif
}


static
void bmi26x_run_self_test_per_sensor(
    sns_sensor_instance     *instance,
    bmi26x_self_test_info_t *test_info,
    const sns_sensor_uid    *suid,
    bool                    hw_detect_success)
{
    int rc = SNS_RC_SUCCESS;
    bmi26x_test_err_code_t err_code = BMI26X_FAC_TEST_NO_ERROR;
    UNUSED_VAR(instance);
    bmi26x_instance_state   *istate = (bmi26x_instance_state *)instance->state->state;

    if (test_info->test_client_present) {
        BMI26X_INST_LOG(MED, instance, "test_client_present sensor:%d test_type:%d",
                        test_info->sensor, test_info->test_type);

        if (SNS_PHYSICAL_SENSOR_TEST_TYPE_COM == test_info->test_type) {
            bmi26x_send_self_test_event(instance, suid,
                                        hw_detect_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM, BMI26X_FAC_TEST_NO_ERROR);
        } else if (SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY == test_info->test_type) {
            if (hw_detect_success) {
                if (!istate->fac_test_in_progress) {
                    if (istate->fifo_info.publish_sensors & BMI26X_ACCEL) {
                        istate->accel_info.backup_before_fac_sample_rate_req = istate->accel_info.sample_rate_req;
                        istate->accel_info.backup_before_fac_report_rate_req = istate->accel_info.report_rate_req;
                    } else {
                        istate->accel_info.backup_before_fac_sample_rate_req = 0.0;
                        istate->accel_info.backup_before_fac_report_rate_req = 0.0;
                    }

                    if (istate->fifo_info.publish_sensors & BMI26X_GYRO) {
                        istate->gyro_info.backup_before_fac_sample_rate_req = istate->gyro_info.sample_rate_req;
                        istate->gyro_info.backup_before_fac_report_rate_req = istate->gyro_info.report_rate_req;
                    } else {
                        istate->gyro_info.backup_before_fac_sample_rate_req = 0.0;
                        istate->gyro_info.backup_before_fac_report_rate_req = 0.0;
                    }

#if BMI26X_CONFIG_ENABLE_CRT_ON_FAC_TEST
                    if (test_info->sensor == BMI26X_GYRO) {
                        //uint8_t regv;

                        if (istate->crt_handler.crt_state > BMI26X_CRT_NONE) {
                            rc = bmi26x_crt_abrot_process(instance);
                            bmi26x_delay_us(1000);
                        }

                        if (istate->int_en_flags_curr.bits.md) {
                            sns_motion_detect_event md_state;
                            md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
                            bmi26x_hal_send_md_event(istate, &md_state, sns_get_system_time());
                        }

                        rc = bmi26x_hal_reset_load_cfg_wrapper(instance);
                        if (rc != SNS_RC_SUCCESS) {
                        }

                        if (bmi26x_hal_check_cfg_available(instance)) {
                            bmi26x_reason_4_rep_multiple_crt_t need_repeat_do_crt;

                            rc = bmi26x_hal_run_crt_process(instance, CRT_TRIGGER_SOURCE_FAC_TEST,
                                                            &need_repeat_do_crt);
                            BMI26X_INST_LOG(HIGH, istate->owner, "@CRT crt finish with rc:%d, need_repeat_crt:%d",
                                            rc, need_repeat_do_crt);
                            if (rc == SNS_RC_SUCCESS) {
                                istate->gyro_info.sstate->crt_version ++;
                                istate->pending_update_crt_in_registry = 1;
                            }
                        }

                        // reset again
                        uint8_t fac_test_in_progress = istate->fac_test_in_progress;
                        rc = bmi26x_hal_reset_load_cfg_wrapper(instance);
                        if (rc != SNS_RC_SUCCESS) {
                        }
                        istate->fac_test_in_progress = fac_test_in_progress;
                    }
#else
#endif
                    rc = bmi26x_start_factory_test(instance, (bmi26x_sensor_type)test_info->sensor, &err_code);
                } else {
                    BMI26X_INST_LOG(HIGH, instance, "NOTICE!!! fac only one sensor at a time, curr_sensor: %d",
                                    istate->fac_test_info.fac_test_sensor);
                    rc = (int)SNS_RC_NOT_AVAILABLE;
                    err_code = BMI26X_FAC_TEST_DEV_BUSY;
                }

                if ((!istate->fac_test_in_progress) || (SNS_RC_SUCCESS != rc)) {
                    bmi26x_send_self_test_event(instance, suid,
                                                false, SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY, err_code);
                }
            } else {
                err_code = BMI26X_FAC_TEST_DEV_FATAL_ERROR;   //because of hw_detect_success
                bmi26x_send_self_test_event(instance, suid,
                                            false, SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY, err_code);
            }
        }
#if BMI26X_CONFIG_ENABLE_SELF_TEST_HW_SW
        else if ((SNS_PHYSICAL_SENSOR_TEST_TYPE_HW == test_info->test_type) ||
                 (SNS_PHYSICAL_SENSOR_TEST_TYPE_SW == test_info->test_type)) {
            if (hw_detect_success && (!istate->fac_test_in_progress)) {
                bmi26x_sensor_type ss_type = (bmi26x_sensor_type) test_info->sensor;
                sns_physical_sensor_test_type phy_ss_test_type =
                        (sns_physical_sensor_test_type)test_info->test_type;

                if (istate->fifo_info.ff_sensors_en_curr & BMI26X_ACCEL) {
                    istate->accel_info.backup_before_fac_sample_rate_req = istate->accel_info.sample_rate_req;
                    istate->accel_info.backup_before_fac_report_rate_req = istate->accel_info.report_rate_req;
                } else {
                    istate->accel_info.backup_before_fac_sample_rate_req = 0.0f;
                }

                if (istate->fifo_info.ff_sensors_en_curr & BMI26X_GYRO) {
                    istate->gyro_info.backup_before_fac_sample_rate_req = istate->gyro_info.sample_rate_req;
                    istate->gyro_info.backup_before_fac_report_rate_req = istate->gyro_info.report_rate_req;
                } else {
                    istate->gyro_info.backup_before_fac_sample_rate_req = 0.0f;
                }

                rc = bmi26x_run_hw_sw_test(instance, ss_type, &err_code);
                // always send out the test result
                bmi26x_send_self_test_event(instance, suid,
                                            hw_detect_success, phy_ss_test_type, err_code);
                if (rc == SNS_RC_SUCCESS) {
                    // XXX do this after a soft reset
                    rc = bmi26x_hal_reset_device_wrapper(instance);
                    if (rc != SNS_RC_SUCCESS) {
                        // TODO
                    }

                    bmi26x_inst_test_fac_recovery_process(instance);
                }
            } else {
                sns_physical_sensor_test_type phy_ss_test_type = (sns_physical_sensor_test_type) test_info->test_type;
                bmi26x_test_err_code_t test_error_code = BMI26X_FAC_TEST_NO_ERROR;

                if (hw_detect_success) {
                } else {
                    test_error_code = BMI26X_FAC_TEST_DEV_FATAL_ERROR;
                }

                bmi26x_send_self_test_event(instance, suid,
                                            hw_detect_success, phy_ss_test_type,
                                            test_error_code);
            }
        }
#endif

        test_info->test_client_present = false;
    }
}
/**
 * Executes requested self-tests.
 *
 * @param instance   reference to the instace
 *
 * @return none
 */
void bmi26x_run_self_test(sns_sensor_instance *instance)
{
    bmi26x_instance_state   *istate = (bmi26x_instance_state*)instance->state->state;
    bool                    who_am_i_success = false;
    sns_rc rc = SNS_RC_SUCCESS;

    BMI26X_INST_LOG(HIGH, instance, "run_self_test()");

    uint8_t  chip_id;

    if (!istate->inst_inted) {
        if (SNS_BUS_SPI == istate->com_port_info.com_config.bus_type) {
            if (bmi26x_hal_prepare_spi_if(istate) != SNS_RC_SUCCESS) {
                rc = SNS_RC_NOT_AVAILABLE ;
            }
        }
    }

    rc |= bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_CHIP_ID, &chip_id, 1);

    if (rc == SNS_RC_SUCCESS
            &&
            (BMI26X_REGV_CHIP_ID_MAJOR == (chip_id & BMI26X_REGV_CHIP_ID_MAJOR))) {
        who_am_i_success = true;
    } else {
        BMI26X_INST_LOG(HIGH, instance, "chip id:0x%x", chip_id);
    }

    bmi26x_run_self_test_per_sensor(instance, &istate->accel_info.test_info, &istate->accel_info.sstate->my_suid,
                                    who_am_i_success);
    bmi26x_run_self_test_per_sensor(instance, &istate->gyro_info.test_info, &istate->gyro_info.sstate->my_suid,
                                    who_am_i_success);
    bmi26x_run_self_test_per_sensor(instance, &istate->md_info.test_info, &istate->md_info.sstate->my_suid,
                                    who_am_i_success);
    bmi26x_run_self_test_per_sensor(instance, &istate->sensor_temp_info.test_info,
                                    &istate->sensor_temp_info.sstate->my_suid, who_am_i_success);
#if BMI26X_CONFIG_ENABLE_PEDO
    bmi26x_run_self_test_per_sensor(instance, &istate->pedo_info.test_info, &istate->pedo_info.sstate->my_suid,
                                    who_am_i_success);
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    bmi26x_run_self_test_per_sensor(instance, &istate->lowg_info.test_info,
                                    &istate->lowg_info.sstate->my_suid, who_am_i_success);
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    bmi26x_run_self_test_per_sensor(instance, &istate->dbtap_info.test_info,
                                    &istate->dbtap_info.sstate->my_suid, who_am_i_success);
#endif

}
// </sensor test>


extern
fp_read_gpio bmi26x_fp_read_gpio;
extern
fp_sns_scp_register_rw  bmi26x_fp_scp_rw;



void bmi26x_inst_collect_sensors(
    bmi26x_instance_state   *istate,
    bmi26x_state            *sstate)
{
    sns_sensor                  *sensor_this = sstate->owner;
    sns_sensor                  *sensor;
    bmi26x_state                *sensor_state;

    for (sensor = sensor_this->cb->get_library_sensor(sensor_this, true);
            sensor != NULL;
            sensor = sensor_this->cb->get_library_sensor(sensor_this, false)) {
        sensor_state = (bmi26x_state*)sensor->state->state;
        if (BMI26X_ACCEL == sensor_state->sensor) {
            istate->accel_info.sstate = sensor_state;
        } else if (BMI26X_GYRO == sensor_state->sensor) {
            istate->gyro_info.sstate = sensor_state;
            //TODOMAG
        } else if (BMI26X_MOTION_DETECT == sensor_state->sensor) {
            istate->md_info.sstate = sensor_state;
        } else if (BMI26X_SENSOR_TEMP == sensor_state->sensor) {
            istate->sensor_temp_info.sstate = sensor_state;
#if BMI26X_CONFIG_ENABLE_PEDO
        } else if (BMI26X_PEDO == sensor_state->sensor) {
            istate->pedo_info.sstate = sensor_state;
#endif
        }
#if BMI26X_CONFIG_ENABLE_OIS
        else if (BMI26X_OIS == sensor_state->sensor) {
            istate->ois_info.sstate = sensor_state;
        }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
        else if (BMI26X_FREE_FALL == sensor_state->sensor) {
            istate->lowg_info.sstate = sensor_state;
        }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
        else if (BMI26X_DOUBLE_TAP == sensor_state->sensor) {
            istate->dbtap_info.sstate = sensor_state;
        }
#endif

        istate->sensors_col |= sensor_state->sensor;
    }

    BMI26X_INST_LOG(LOW, istate->owner, "sensors_col: 0x%x", istate->sensors_col);
}

static void bmi26x_inst_config_ascp(
    bmi26x_instance_state   *istate)
{
    sns_data_stream             *data_stream = istate->async_com_port_data_stream;
    sns_com_port_config         *com_config = &istate->sstate_creator->common.com_port_info.com_config;

    sns_async_com_port_config  ascp_config;

    uint8_t                     pb_encode_buffer[100];
    bmi26x_real_t               bus_cycle_time_us;

    sns_request                 async_com_port_request = {
        .message_id  = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
        .request     = &pb_encode_buffer
    };

    memset(&ascp_config, 0, sizeof(ascp_config));

    if (SNS_BUS_SPI == com_config->bus_type) {
        ascp_config.bus_type          = SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
    } else {
        ascp_config.bus_type          = SNS_ASYNC_COM_PORT_BUS_TYPE_I2C;
    }
    ascp_config.slave_control     = com_config->slave_control;
    ascp_config.reg_addr_type     = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
    ascp_config.min_bus_speed_kHz = com_config->min_bus_speed_KHz;
    ascp_config.max_bus_speed_kHz = com_config->max_bus_speed_KHz;
    ascp_config.bus_instance      = com_config->bus_instance;

    bus_cycle_time_us = BMS_SCALE_S2US * 1.0 / ((com_config->min_bus_speed_KHz + com_config->max_bus_speed_KHz) / 2 * 1000);
    istate->bus_cycle_time_ns = roundf(bus_cycle_time_us * 1000);

    if (SNS_BUS_SPI == com_config->bus_type) {
        istate->xfer_time_per_byte_ticks = bmi26x_convert_us2ticks(bus_cycle_time_us * (8 +
                                           BMI26X_CONFIG_SEE_SPI_BYTE_XFER_WAIT_CYCLES));
        istate->bus_is_spi = 1;
    } else {
        istate->xfer_time_per_byte_ticks = bmi26x_convert_us2ticks(bus_cycle_time_us * 9);
    }

    async_com_port_request.request_len =
        pb_encode_request(pb_encode_buffer,
                          sizeof(pb_encode_buffer),
                          &ascp_config,
                          sns_async_com_port_config_fields,
                          NULL);
    data_stream->api->send_request(data_stream, &async_com_port_request);
}

/** See sns_sensor_instance_api::init */
sns_rc bmi26x_inst_init(
    sns_sensor_instance *const  this,
    sns_sensor_state const      *sensor_state)
{
    bmi26x_instance_state     *istate = (bmi26x_instance_state*)this->state->state;
    bmi26x_state              *sstate = (bmi26x_state*)sensor_state->state;
    sns_service_manager       *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service        *stream_mgr = (sns_stream_service*)service_mgr->get_service(service_mgr,
            SNS_STREAM_SERVICE);
    float                     data[3] = {0.0};
    float                     temp_data[1] = {0.0};
    uint8_t                   si_buf[2] = {0};
    sns_sensor_uid            irq_suid;
    sns_sensor_uid            ascp_suid;

    sns_rc                    rc = SNS_RC_SUCCESS;
#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    uint64_t                        buffer[10];
    pb_ostream_t                    stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));


    uint8_t                         arr_index = 0;
    float                           diag_temp[BMI26X_NUM_AXES];
    pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = BMI26X_NUM_AXES,
                            .arr_index = &arr_index
                           };
    sns_diag_batch_sample           batch_sample = sns_diag_batch_sample_init_default;

    batch_sample.sample.funcs.encode = pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;
#endif

    UNUSED_VAR(si_buf);

    bmi26x_inst_singleton = istate;

    istate->owner = this;
    istate->sstate_creator = sstate;

    istate->stream_mgr = stream_mgr;
    istate->scp_service = (sns_sync_com_port_service*)service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
    bmi26x_fp_scp_rw = istate->scp_service->api->sns_scp_register_rw;

    istate->gpio_service = (sns_gpio_service*)service_mgr->get_service(service_mgr, SNS_GPIO_SERVICE);

#if BMI26X_CONFIG_ENABLE_SEE_LITE
    bmi26x_fp_read_gpio = istate->gpio_service->api->read_gpio;
#endif

    /**---------Setup stream connections with dependent Sensors---------*/
    sns_suid_lookup_get(&sstate->common.suid_lookup_data, "interrupt", &irq_suid);
    sns_suid_lookup_get(&sstate->common.suid_lookup_data, "async_com_port", &ascp_suid);
    sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", &istate->timer_suid);
    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this,
                                                        irq_suid,
                                                        &istate->interrupt_data_stream);
    BMI26X_INST_CHECK_RC(this, rc);

    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this,
                                                        ascp_suid,
                                                        &istate->async_com_port_data_stream);
    BMI26X_INST_CHECK_RC(this, rc);

    //CHECK4IMPROVE: this is where improvement when needed can be made, we don't need timer to send temperature data, we can take advantage of the  fifo interrupt for acc/gyr
    //this improvement will save some island memory size needed
    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this,
                                                        istate->timer_suid,
                                                        &istate->timer_data_stream);
    BMI26X_INST_CHECK_RC(this, rc);

    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this,
                                                        istate->timer_suid,
                                                        &istate->cmd_handler.timer_cmd_stream);
    BMI26X_INST_CHECK_RC(this, rc);

#if BMI26X_CONFIG_ENABLE_CRT
    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this,
                                                        istate->timer_suid,
                                                        &istate->crt_handler.timer_crt_stream);
    BMI26X_INST_CHECK_RC(this, rc);
#endif

#if BMI26X_CONFIG_ENABLE_PEDO_TIMER
    rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                        this,
                                                        istate->timer_suid,
                                                        &istate->pedo_info.pedo_timer_data_stream);
    BMI26X_INST_CHECK_RC(this, rc);
#endif

    /** Initialize COM port to be used by the Instance */
    sns_memscpy(&istate->com_port_info.com_config,
                sizeof(istate->com_port_info.com_config),
                &sstate->common.com_port_info.com_config,
                sizeof(sstate->common.com_port_info.com_config));

    rc = istate->scp_service->api->sns_scp_register_com_port(&istate->com_port_info.com_config,
                                                             &istate->com_port_info.port_handle);
    BMI26X_INST_CHECK_RC(this, rc);

    if (NULL == istate->interrupt_data_stream ||
            NULL == istate->async_com_port_data_stream ||
            NULL == istate->timer_data_stream ||
            NULL == istate->cmd_handler.timer_cmd_stream  ||
#if BMI26X_CONFIG_ENABLE_PEDO_TIMER
            NULL == istate->pedo_info.pedo_timer_data_stream ||
#endif
#if BMI26X_CONFIG_ENABLE_CRT
            NULL == istate->crt_handler.timer_crt_stream ||
#endif

            NULL == istate->com_port_info.port_handle) {
        bmi26x_inst_cleanup(this, stream_mgr);
        return SNS_RC_FAILED;
    }

    /**-------------------------Init FIFO State-------------------------*/
    istate->fifo_info.ff_wml_bytes_curr = 0;

#if  BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    istate->hb_cfg_info.timer_heart_beat_data_stream = NULL;
    istate->hb_cfg_info.timer_enable = 0;
#endif

    if (SNS_BUS_I2C == sstate->common.com_port_info.com_config.bus_type) {
        istate->fifo_info.ff_tm_info.ff_ts_dev_delay = BMI26X_CONFIG_TS_DELAY_ADJ_I2C;
    } else if (SNS_BUS_SPI == sstate->common.com_port_info.com_config.bus_type) {
        istate->bus_is_spi = 1;
    }

    // 1Ms = 10^6 Ns
    istate->ts_hw_res_ticks_per_bit_ideal = sns_convert_ns_to_ticks(BMS_SCALE_S2US) * BMI26X_SPEC_SENSORTIME_RES_US / 1000;
    istate->ts_hw_res_ticks_per_bit = istate->ts_hw_res_ticks_per_bit_ideal;

    istate->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
    istate->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, 1);

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    istate->diag_service =  (sns_diag_service*)
                            service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
#endif

    istate->scp_service =  (sns_sync_com_port_service*)
                           service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

    istate->scp_service->api->sns_scp_open(istate->com_port_info.port_handle);

    // device reset
    istate->pwr_state_present = sstate->power_rail_pend_state;

    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            false);

#if BMI26X_CONFIG_ENABLE_PEDO
    istate->encoded_pedo_count_len  = pb_get_encoded_size_sensor_stream_event(temp_data, 1);
#endif

    /** Configure the Async Com Port */
    bmi26x_inst_config_ascp(istate);

    bmi26x_inst_collect_sensors(istate, sstate);

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
    /** Determine sizes of encoded logs */
    sns_diag_sensor_state_interrupt sensor_state_interrupt =
        sns_diag_sensor_state_interrupt_init_default;
    pb_get_encoded_size(&istate->log_interrupt_encoded_size,
                        sns_diag_sensor_state_interrupt_fields,
                        &sensor_state_interrupt);

    /** Determine size of sns_diag_sensor_state_raw as defined in
     *  sns_diag.proto
     *  sns_diag_sensor_state_raw is a repeated array of samples of
     *  type sns_diag_batch sample. The following determines the
     *  size of sns_diag_sensor_state_raw with a single batch
     *  sample */
    if (pb_encode_tag(&stream, PB_WT_STRING,
                      sns_diag_sensor_state_raw_sample_tag)) {
        if (pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                                &batch_sample)) {
            istate->log_raw_encoded_size = stream.bytes_written;
        }
    }
#endif
    BMI26X_INST_LOG(MED, this, "<bmi26x_iif inst_init> 0x%x %d 0x%x pwr:%d",
                    this, sstate->sensor, istate, istate->pwr_state_present);
    BMI26X_INST_LOG(MED, this, "0x%x 0x%x 0x%x", bmi26x_inst_singleton, istate->scp_service,
                    istate->com_port_info.port_handle);

    INSERT_TRACE_POINT_P(bmi26x, this);

#if BMI26X_CONFIG_ENABLE_DAE
    sns_sensor_uid dae_suid;

    sns_suid_lookup_get(&sstate->common.suid_lookup_data, "data_acquisition_engine", &dae_suid);

    rc = bmi26x_dae_if_init(sstate->owner, this, stream_mgr, &dae_suid);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(HIGH, istate->owner, "dae_if_init error:%d", rc);
    }

    if (!bmi26x_dae_if_available(this)) {
        BMI26X_INST_LOG(MED, istate->owner, "NON-DAE, register isr on platform");
    } else {
        BMI26X_INST_LOG(MED, istate->owner, "DAE available, isr away from SEE");
    }
#endif

    istate->config_step = BMI26X_CONFIG_IDLE;
    istate->ticks_in_1ms = sns_convert_ns_to_ticks(1000 * 1000);

#if BMI26X_CONFIG_ENABLE_PEDO
    istate->pedo_info.step_count = 0;
    istate->pedo_info.step_count_hw_last = 0;
    istate->pedo_info.is_first = false;
#endif


    if ((istate->sensors_col & BMI26X_ACCEL) == 0) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! ACCEL info missing:0x%x",
                        istate->sensors_col);
        return SNS_RC_FAILED;
    }

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_inst_deinit(sns_sensor_instance *const this)
{
    bmi26x_instance_state   *istate = (bmi26x_instance_state*)this->state->state;
    sns_service_manager     *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service      *stream_mgr = (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    UNUSED_VAR(istate);
    BMI26X_INST_LOG(HIGH, istate->owner, "<bmi26x_iif inst_deinit> 0x%x", this);

    bmi26x_inst_cleanup(this, stream_mgr);

    bmi26x_inst_singleton = NULL;

    return SNS_RC_SUCCESS;
}
