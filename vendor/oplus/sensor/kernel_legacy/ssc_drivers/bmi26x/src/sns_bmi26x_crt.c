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
#include "sns_bmi26x_config.h"

#if BMI26X_CONFIG_ENABLE_CRT
#include "sns_types.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_timer.pb.h"

#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor_instance.h"
#include "sns_bmi26x_sensor.h"
#include "sensor_cfg/sns_bmi26x_sensor_cfg.h"

static sns_rc bmi26x_crt_abort(sns_sensor_instance *const inst, bool block_process)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint8_t cfg_buffer[2] = {0};
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_GYR_ABORT_CRT, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, cfg_buffer, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS)

    if (block_process) {
        cfg_buffer[1] = BST_SET_VAL_BIT(cfg_buffer[1], 1);
    } else {
        cfg_buffer[1] = BST_CLR_VAL_BIT(cfg_buffer[1], 1);
    }

    // TODO set the abortion bit

    /* set max burst length to default value */
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, cfg_buffer, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}


// NOTICE, the feature configuration should work under normal mode
static sns_rc bmi26x_hal_get_maxburst_len(bmi26x_instance_state *istate,
        uint8_t *max_burst_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;


    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_MAX_BURST_LEN, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset)

    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, max_burst_buffer, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "max burst len:0x%x, 0x%x", max_burst_buffer[0], max_burst_buffer[1]);

    return rc;
}

static sns_rc bmi26x_hal_set_maxburst_len(bmi26x_instance_state *istate,
        uint8_t *max_burst_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_MAX_BURST_LEN, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    /* set max burst length to default value */
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, max_burst_buffer, 2);

    return rc;
}

static sns_rc bmi26x_validate_crt_running_status(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t crt_run_status = 0;

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR, &crt_run_status, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (BST_GET_VAL_BIT(crt_run_status, BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS)) {
        /* CRT already running */
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! crt already running");
        return SNS_RC_NOT_AVAILABLE;
    } else {
    }

    crt_run_status = BST_SET_VAL_BIT(crt_run_status, BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS);
    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR, &crt_run_status, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}


static sns_rc bmi26x_hal_wait_crt_finish(bmi26x_instance_state *istate)
{
    uint8_t regv = 0;
    uint8_t try_num = BMI26X_MAX_TRY_NUM_4_CONFIGURE_DOWNLOAD;
    sns_rc rc = SNS_RC_SUCCESS;
    do {
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR,
                                      &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        bmi26x_delay_us(10000); //10ms
    } while ((BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS)) &&
             (try_num --));

    if (BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS)) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! crt still running");
        rc = SNS_RC_INVALID_STATE;
    }
    return rc;
}

sns_rc bmi26x_crt_abrot_process(sns_sensor_instance *const inst)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    uint8_t regv_crt_cmd = BMI26X_REGV_CMD_RUN_CRT;

    BMI26X_INST_LOG(HIGH, istate->owner, "@CRT abort crt process, crt.st:%d",
                    istate->crt_handler.crt_state);

    if (istate->crt_handler.crt_state == BMI26X_CRT_TIMER_START) {
        istate->crt_handler.crt_state = BMI26X_CRT_ABORT;
        return SNS_RC_SUCCESS;
    }

    rc = bmi26x_crt_abort(inst, true);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv_crt_cmd, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    rc = bmi26x_hal_wait_crt_finish(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    istate->crt_handler.crt_state = BMI26X_CRT_NONE;

    return rc;
}


// to wait till the ready for download bit toggles after every pack of bytes
static sns_rc bmi26x_hal_wait_crt_download_toggle(bmi26x_instance_state *istate,
        uint8_t download_ready)
{
    uint8_t regv = 0;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t try_num = BMI26X_MAX_TRY_NUM_4_CRT_PROCESS;
    do {
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR,
                                      &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(1000);
    } while ((download_ready == BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_DOWNLOAD_READY_BIT_POS)) &&
             (try_num --));

    if (download_ready == BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_DOWNLOAD_READY_BIT_POS)) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! try out, downlaod failure:<0x%x 0x%x>",
                        download_ready, regv);
        rc = SNS_RC_INVALID_STATE;
    }

    if (rc == SNS_RC_SUCCESS) {
        regv = 0;
        // get the status of st_status from gyr_crt_conf register
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR,
                                      &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        if (BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_RUNNING_BIT_POS) == 0) {
            rc = SNS_RC_NOT_AVAILABLE;
        }
    }

    return rc;
}

static sns_rc bmi26x_hal_audit_crt_download_process(bmi26x_instance_state *istate,
        bool is_last_write)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_ready_4_download;
    uint8_t regv_crt_cmd = BMI26X_REGV_CMD_RUN_CRT;


    // get ready for dwonload
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR,
                                  &regv_ready_4_download, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    regv_ready_4_download = BST_GET_VAL_BIT(regv_ready_4_download, BMI26X_REGV_GYR_CRT_DOWNLOAD_READY_BIT_POS);

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv_crt_cmd, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (!is_last_write) {
        rc = bmi26x_hal_wait_crt_download_toggle(istate, regv_ready_4_download);
    }

    return rc;
}



static sns_rc bmi26x_hal_write_crt_config_file(bmi26x_instance_state *istate,
        uint8_t *cfg_buffer,
        uint32_t cfg_start_position,
        uint32_t cfg_block_size,
        uint32_t burst_write_size,
        uint8_t *burst_trigger_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t remain = cfg_block_size % burst_write_size;
    uint32_t balance_byte = cfg_start_position + cfg_block_size - remain;
    bool is_last_byte = false;

    if (!remain) {
        /* Write the configuration file */
        for (uint32_t index = cfg_start_position; index < (cfg_start_position + cfg_block_size);
                index += burst_write_size) {
            rc = bmi26x_do_hal_load_cfg(istate->owner, cfg_buffer + index, burst_write_size, burst_write_size);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            if (index < ((cfg_start_position + cfg_block_size) - burst_write_size)) {
                is_last_byte = 1;
            }

            rc = bmi26x_hal_audit_crt_download_process(istate, is_last_byte);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
    } else {
        /* Write the configuration file for the balance
         bytes */
        uint32_t burst_write_size_for_remian = 0;

        for (uint32_t index = cfg_start_position; index < balance_byte ; index += burst_write_size) {
            rc = bmi26x_do_hal_load_cfg(istate->owner, cfg_buffer + index, burst_write_size,
                                        burst_write_size);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            rc = bmi26x_hal_audit_crt_download_process(istate, is_last_byte);
        }
        // the remain bytes
        burst_write_size_for_remian = 2;
        burst_trigger_buffer[0] = 2;
        rc = bmi26x_hal_set_maxburst_len(istate, burst_trigger_buffer);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! set maxburst len error:0x%x", rc);
        }

        /* Write the configuration file for the remaining bytes */
        for (uint32_t index = balance_byte; index < cfg_start_position + cfg_block_size;
                index += burst_write_size_for_remian) {
            rc = bmi26x_do_hal_load_cfg(istate->owner, cfg_buffer + index,
                                        burst_write_size_for_remian, burst_write_size_for_remian);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            if (index < ((cfg_start_position + cfg_block_size) - burst_write_size_for_remian)) {
                is_last_byte = true;
            }
            rc = bmi26x_hal_audit_crt_download_process(istate, is_last_byte);
        }
    }


    return rc;
}


static sns_rc bmi26x_hal_crt_update_result(bmi26x_instance_state *istate,
                uint8_t *crt_max_trigger_buffer,
                bmi26x_reason_4_rep_multiple_crt_t *need_repeat_do_crt)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_gyr_user_gain_status_t  gyr_gain_status ;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint8_t user_gain_buffer[2] = {0};
    bool update_maxburst_len = true;

    // get gain update status
    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_OUTPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }


    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, user_gain_buffer, sizeof(user_gain_buffer));
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    gyr_gain_status.sat_x = BST_GET_VAL_BIT(user_gain_buffer[0], 0);
    gyr_gain_status.sat_y = BST_GET_VAL_BIT(user_gain_buffer[0], 1);
    gyr_gain_status.sat_z = BST_GET_VAL_BIT(user_gain_buffer[0], 2);

    gyr_gain_status.crt_status = BST_GET_VAL_BITBLOCK(user_gain_buffer[0], 3, 5);

    if (gyr_gain_status.crt_status == BMI26X_GYRO_TRIGGER_STATUS_NO_ERROR) {
        crt_max_trigger_buffer[0] = 0;
        *need_repeat_do_crt = REPEAT_MULTIPLE_NONE;
        BMI26X_INST_LOG(LOW, istate->owner, "crt trigger status no error");
    } else if (gyr_gain_status.crt_status == BMI26X_GYRO_TRIGGER_STATUS_PRECON_ERROR) {
        crt_max_trigger_buffer[0] = 0;
        *need_repeat_do_crt = REPEAT_MULTIPLE_CRT_DUE_TO_CRT_FIFO_ERROR;
        rc = SNS_RC_FAILED;
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! crt trigger pre-condition error");
    } else if (gyr_gain_status.crt_status  == BMI26X_GYRO_TRIGGER_STATUS_DOWNLOAD_ERROR) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! crt trigger download error");
        *need_repeat_do_crt = REPEAT_MULTIPLE_CRT_DUE_TO_CRT_DOWNLOAD_ERROR;
        rc = SNS_RC_FAILED;
    } else if (gyr_gain_status.crt_status == BMI26X_GYRO_TRIGGER_STATUS_ABORT_ERROR) {
        crt_max_trigger_buffer[0] = BMI26X_CRT_MAX_BURST_READ_WRITE_LEN;
        *need_repeat_do_crt = REPEAT_MULTIPLE_CRT_DUE_TO_CRT_ABORT_ERROR;
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! crt trigger abort error");
        rc = SNS_RC_FAILED;
    } else {
        crt_max_trigger_buffer[0] = 0xff;
        *need_repeat_do_crt = REPEAT_MULTIPLE_CRT_UNKNOWN_ERROR;
        update_maxburst_len = false;
    }

    if (update_maxburst_len) {
        rc |= bmi26x_hal_set_maxburst_len(istate, crt_max_trigger_buffer);
    }

    return rc;
}

// G_TRIG selection configuration
static sns_rc bmi26x_hal_select_crt(bmi26x_instance_state *istate, bool gyr_crt, uint8_t *crt_max_trigger_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint8_t *g_trigger = crt_max_trigger_buffer;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_GYR_TRIGGER_SELECT, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    // select feature that should be executed
    if (gyr_crt) {
        g_trigger[1] = BST_SET_VAL_BIT(g_trigger[1], 0);   //crt wil be executed
    } else {
        g_trigger[1] = BST_CLR_VAL_BIT(g_trigger[1], 0);   //gyro built-in-self-test will executed
    }
    rc = bmi26x_set_cfg_data(istate, page_num, page_offset, g_trigger, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}

static sns_rc bmi26x_hal_trigger_crt(bmi26x_instance_state *istate,
                                     uint8_t *crt_max_trigger_buffer,
                                     bmi26x_reason_4_rep_multiple_crt_t *need_repeat_do_crt)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv;

    /* check if FIFO is unchanged by checking the max burst length */
    if (1) {
        // XXX step: send g_trigger command using register CMD
        /* trigger CRT */
        BMI26X_INST_LOG(MED, istate->owner, "@CRT fifo is empty, do crt soon");
        regv = BMI26X_REGV_CMD_RUN_CRT;
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD,
                                       &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }
    else
    {
        /* FIFO maybe used */
        uint8_t regv_ready_for_download = 0;

        BMI26X_INST_LOG(MED, istate->owner, "@CRT WARN!!! fifo used, need reload configure to run crt");

        /* set max trigger bytes */
        crt_max_trigger_buffer[0] = BMI26X_CRT_MAX_BURST_READ_WRITE_LEN;
        rc = bmi26x_hal_set_maxburst_len(istate, crt_max_trigger_buffer);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        /* get the status for download ? */
        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_GYR_CRT_CONF_ADDR, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv_ready_for_download = BST_GET_VAL_BIT(regv, BMI26X_REGV_GYR_CRT_DOWNLOAD_READY_BIT_POS);

        /* trigger CRT */
        regv = BMI26X_REGV_CMD_RUN_CRT;
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        /* wait till either read for download toggle or crt running = 0 */
        rc = bmi26x_hal_wait_crt_download_toggle(istate, regv_ready_for_download);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        /* download configure stream*/
        uint32_t crt_burst_write_size = 2 * BMI26X_CRT_MAX_BURST_READ_WRITE_LEN;
        rc = bmi26x_hal_write_crt_config_file(istate, bmi26x_crt_cfg, BMI26X_CRT_OFFSET_IN_CONFIG_FILE,
                                              bmi260_sensor_crt_cfg_size, crt_burst_write_size,
                                              crt_max_trigger_buffer);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }
    // XXX step: CRT complete
    /* wait for crt finish */
    rc = bmi26x_hal_wait_crt_finish(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // XXX step: update CRT final result
    rc = bmi26x_hal_crt_update_result(istate, crt_max_trigger_buffer, need_repeat_do_crt);

    return rc;
}

sns_rc bmi26x_hal_reconfig_crt_param(
    sns_sensor_instance     *inst,
    bmi26x_gyr_crt_gain_t   *gain_buffer)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state *)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    if (gain_buffer == NULL) {
        return SNS_RC_SUCCESS;
    }

    if (gain_buffer->gain_x == istate->crt_gain_state.regv_crt_params_x &&
        gain_buffer->gain_y == istate->crt_gain_state.regv_crt_params_y  &&
        gain_buffer->gain_z == istate->crt_gain_state.regv_crt_params_z) {
    } else {

        uint8_t gain_val[3] = {0};
        gain_val[0] = gain_buffer->gain_x;
        gain_val[1] = gain_buffer->gain_y;
        gain_val[2] = gain_buffer->gain_z;

        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_CRT_PARAM_0, gain_val, 3);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        bmi26x_delay_us(1000);

        istate->crt_gain_state.regv_crt_params_x = gain_buffer->gain_x;
        istate->crt_gain_state.regv_crt_params_y = gain_buffer->gain_y;
        istate->crt_gain_state.regv_crt_params_z = gain_buffer->gain_z;
    }

    // enable crt gain
    //if (istate->crt_gain_en == 0) {
    if (istate->crt_gain_state.regv_gain_en == 0) {
        rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_CRT_CONFIG,
                                          BMI26X_REGA_CRT_CONFIG_BIT_4_CRT_GAIN_EN,
                                          BMI26X_REGA_CRT_CONFIG_BIT_4_CRT_GAIN_EN, 1);
        istate->crt_gain_state.regv_gain_en = 1;
    }

    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}



static sns_rc bmi26x_hal_get_gain_update_status(bmi26x_instance_state *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    uint8_t gain_update_buffer[6] = {0};

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    // XXX step: wait CRT status complete
    do {
        rc = bmi26x_get_cfg_data(istate, page_num, page_offset, gain_update_buffer,
                                 sizeof(gain_update_buffer));
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi26x_delay_us(1000);
    } while (BST_GET_VAL_BIT(gain_update_buffer[5], 3));

    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_GYR_GAIN_CONFIG, 7, 7, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // enable gyro

    return rc;
}

static sns_rc bmi26x_hal_get_gyr_gain_ratio(bmi26x_instance_state *istate, uint8_t *param_ratio_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
#if 0
    uint8_t page_offset = 0;
    uint8_t page_num = 0xff;
    bmi26x_feature_in_user_gain_update_t *p_user_gain_update = (bmi26x_feature_in_user_gain_update_t *) param_ratio_buffer;

    page_num = bmi26x_hal_cfg_get_page_num(BMI26X_CONFIG_INDEX_GYRO_GAIN_UPDATE, &page_offset,
                                           BMI26X_ADVANCED_FEATURE_INPUT);
    BMI26X_INST_LOG(LOW, istate->owner, "max burst len @page:%d, page offset:%d",
                    page_num, page_offset);
    if (page_num > BMI26X_MAX_CONFIG_PAGE_NUM) {
        BMI26X_INST_LOG(HIGH, istate->owner, "WARNING!!! invalid page number:%d", page_num);
        return SNS_RC_INVALID_TYPE;
    }

    rc = bmi26x_get_cfg_data(istate, page_num, page_offset, param_ratio_buffer,
                             sizeof(bmi26x_feature_in_user_gain_update_t));
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, istate->owner, "get user gain ratio_x:0x%x, ratio_y:0x%x, ratio_z:0x%x, en_gain_update:%d",
                    p_user_gain_update->ratio_x, p_user_gain_update->ratio_y,
                    p_user_gain_update->ratio_z, p_user_gain_update->en_user_gain_update);
#else
    UNUSED_VAR(param_ratio_buffer);
    uint8_t regv_gain_buffer[3] = {0};
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_CRT_PARAM_0, regv_gain_buffer, 3);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if ((regv_gain_buffer[0] != istate->gyro_info.sstate->crt_gain.gain_x) ||
                    (regv_gain_buffer[1] != istate->gyro_info.sstate->crt_gain.gain_y) ||
                    (regv_gain_buffer[2] != istate->gyro_info.sstate->crt_gain.gain_z)) {
        // update when any change on x/y/z
        istate->gyro_info.sstate->crt_gain.gain_x = regv_gain_buffer[0];
        istate->gyro_info.sstate->crt_gain.gain_y = regv_gain_buffer[1];
        istate->gyro_info.sstate->crt_gain.gain_z = regv_gain_buffer[2];

        // update crt
        rc = bmi26x_hal_reconfig_crt_param(istate->owner, &istate->gyro_info.sstate->crt_gain);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    } else {
        BMI26X_INST_LOG(LOW, istate->owner, "crt gain value not change, no need update registers");
    }

    BMI26X_INST_LOG(LOW, istate->owner, "crt finish, gain val<0x%x 0x%x 0x%x>",
                    regv_gain_buffer[0], regv_gain_buffer[1], regv_gain_buffer[2]);
#endif

    return rc;
}


static sns_rc bmi26x_prepare_crt(bmi26x_instance_state *istate, uint8_t *crt_max_trigger_buffer)
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv_buffer[2] ;

    // XXX step: Disable GYR
    /* Disable gyroscope */
    rc = bmi26x_dev_pwr_ctrl(istate, BMI26X_PWR_CTRL_GYR_EN_BIT_POS,
                             BMI26X_PWR_CTRL_GYR_EN_BIT_POS, 0);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    bmi26x_delay_us(500);

    // XXX Disable OIS
    /* Disable OIS  */
    rc = bmi26x_dev_reg_read_modify_write(istate, BMI26X_REGA_USR_IF_CONF, 4, 4, 0);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // XXX step: Disable FIFO for all sensors
    /* Disable FIFO for all sensors */
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_0,
                                  regv_buffer, 2);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    //regv_buffer[0] = BST_SET_VAL_BITBLOCK(regv_buffer[0], 0, 1, 0);
    //regv_buffer[1] = BST_SET_VAL_BITBLOCK(regv_buffer[1], 5, 7, 0);
    regv_buffer[1] = 0x10; //BST_SET_VAL_BITBLOCK(regv_buffer[1], 5, 7, 0);

    rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_0,
                                   regv_buffer, 2);

    // XXX step: Enable accelerometer
    /* enable acc */
    rc = bmi26x_dev_pwr_ctrl(istate, BMI26X_PWR_CTRL_ACC_EN_BIT_POS,
                             BMI26X_PWR_CTRL_ACC_EN_BIT_POS, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // disable abort after 1msec
    bmi26x_delay_us(1000);
    rc = bmi26x_crt_abort(istate->owner, false);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // XXX step: set G_TRIG
    /*select crt */
    rc = bmi26x_hal_select_crt(istate, true, crt_max_trigger_buffer);

    return rc;
}


static sns_rc bmi26x_hal_run_crt_calibration(
    sns_sensor_instance     *inst,
    bmi26x_reason_4_rep_multiple_crt_t *need_repeat_do_crt)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state *)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t regv = 0;
    uint8_t crt_maxburst_buffer[2] = {0};

    /* power configuration */
    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_PWR_CONF, &regv, 1);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(LOW, inst, "pwm cfg:0x%x", regv);

    /* disable advanced power save */
    rc = bmi26x_dev_pwr_conf(istate, 0, 0, 0);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    /* Get max burst length */
    rc = bmi26x_hal_get_maxburst_len(istate, crt_maxburst_buffer);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    /* check CRT running status */
    rc = bmi26x_validate_crt_running_status(istate);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    /* prepare to run CRT */
    rc = bmi26x_prepare_crt(istate, crt_maxburst_buffer);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    // trigger crt
    rc = bmi26x_hal_trigger_crt(istate, crt_maxburst_buffer, need_repeat_do_crt);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return rc;
}

static void bmi26x_hal_prepare_next_crt_process_due_to_problem(bmi26x_instance_state *istate,
                bmi26x_reason_4_rep_multiple_crt_t reason)
{
    if (istate->gyro_info.sstate->crt_cfg.crt_repeate_on_error) {
        istate->gyro_info.sstate->crt_4_multiple_state.latest_repeat_crt_reason = reason;
        istate->gyro_info.sstate->crt_4_multiple_state.need_repeat_crt = 1;
    }
}

sns_rc bmi26x_hal_run_crt_process(sns_sensor_instance * const inst,
                                    bmi26x_crt_trigger_source_t trigger_source,
                                    bmi26x_reason_4_rep_multiple_crt_t *need_repeat_do_crt)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t gain_ratio[6] = {0xff};
    uint8_t regv_gain_buffer[3] = {0};

    BMI26X_INST_LOG(LOW, inst, "@CRT triggred by: %d", trigger_source);

    // double check the sensor status
    if ((istate->fifo_info.ff_sensors_en_curr & BMI26X_GYRO) ||
        (istate->fifo_info.ff_sensors_en_curr & BMI26X_ACCEL)) {
        // abort, ds in progress now
        BMI26X_INST_LOG(HIGH, istate->owner, "@CRT NOTICE!!! CRT can perform due to ss ds in progress ss:0x%x",
                        istate->fifo_info.ff_sensors_en_curr);
        //return SNS_RC_NOT_AVAILABLE;
    }

    if (bmi26x_hal_check_cfg_available(inst)) {
        BMI26X_INST_LOG(HIGH, istate->owner, "@CRT WARM NOTICE!!! ready to do CRT");
    } else {
        // soft reset before do CRT
        uint8_t regv;
        //rc = bmi26x_hal_send_cmd(istate, BMI26X_REGV_CMD_SOFT_RESET);
        regv = BMI26X_REGV_CMD_SOFT_RESET;
        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_CMD, &regv, 1);
        bmi26x_delay_us(8 * 1000);

        rc = bmi26x_hal_load_sensor_cfg(inst);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, inst, "@CRT load crt rc:%d", rc);
            return SNS_RC_FAILED;
        }
    }

    rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_CRT_PARAM_0, regv_gain_buffer, 3);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    BMI26X_INST_LOG(HIGH, istate->owner, "@CRT WARM NOTICE!!! start to do CRT");

    BMI26X_INST_LOG(LOW, istate->owner, "before crt, gain val<0x%x 0x%x 0x%x>",
                    regv_gain_buffer[0], regv_gain_buffer[1], regv_gain_buffer[2]);

    rc = bmi26x_hal_run_crt_calibration(inst, need_repeat_do_crt);
    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    if (rc == SNS_RC_SUCCESS) {
        rc = bmi26x_hal_get_gain_update_status(istate);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi26x_hal_get_gyr_gain_ratio(istate, gain_ratio);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    } else {
        BMI26X_INST_LOG(HIGH, istate->owner, "@CRT done with error:%d", rc);
    }

    BMI26X_INST_LOG(MED, istate->owner, "@CRT WARM NOTICE!!! CRT finished");

    rc = bmi26x_hal_power_mode_assert(istate);
    return rc;
}

static void bmi26x_hal_crt_cleanup(bmi26x_instance_state   *istate)
{
    sns_rc rc;
    if ((rc = bmi26x_hal_power_mode_assert(istate)) != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(MED, istate->owner, "@CRT power mode assert failure:%d", rc);
    }
}


void bmi26x_hal_handle_timer_crt(bmi26x_instance_state   *istate)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_sensor_event            *event;
    bool         crt_timer_event_handled = false;

    if (NULL != istate->crt_handler.timer_crt_stream) {
        // handle crt timer event
        event = istate->crt_handler.timer_crt_stream->api->peek_input(
                        istate->crt_handler.timer_crt_stream);
        while (NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                         event->event_len);
            sns_timer_sensor_event timer_event;
            if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    uint8_t crt_assert_in_timer_event = (istate->fifo_info.ff_sensors_en_curr & (BMI26X_ACCEL | BMI26X_GYRO));
                    sns_time ts = bmi26x_get_sys_tick();
                    BMI26X_INST_LOG(MED, istate->owner, "@CRT timer event @%u", (uint32_t)ts);

                    {
                        bmi26x_reason_4_rep_multiple_crt_t need_repeat_do_crt;
                        BMI26X_INST_LOG(MED, istate->owner, "@CRT ready to do crt now");

                        crt_timer_event_handled = true;

                        if (istate->fac_test_in_progress) {
                            BMI26X_INST_LOG(HIGH, istate->owner, "@CRT WARN!!! CRT fac in process, it's not right time to do crt");
                            event = istate->crt_handler.timer_crt_stream->api->get_next_input(
                                            istate->crt_handler.timer_crt_stream);
                            continue;
                        }

                        if ((istate->crt_handler.crt_state == BMI26X_CRT_ABORT) ||
                                        crt_assert_in_timer_event) {
                            BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! NO need to do due to:0x%x",
                                            (istate->crt_handler.crt_state << 3) | crt_assert_in_timer_event);
                        } else {

                            istate->crt_handler.crt_state = BMI26X_CRT_RUN_IN_PROGRESS;
                            rc = bmi26x_hal_run_crt_process(istate->owner, CRT_TRIGGER_SOURCE_ROUTINE,
                                                        &need_repeat_do_crt);
                            istate->crt_handler.crt_state = BMI26X_CRT_DONE;

                            if (rc == SNS_RC_SUCCESS) {
                                istate->gyro_info.sstate->crt_version ++;
                                istate->pending_update_crt_in_registry = 1;
                                istate->gyro_info.sstate->crt_4_multiple_state.ts_crt_latest_passed =
                                            bmi26x_get_sys_tick();
                                istate->gyro_info.sstate->crt_4_multiple_state.need_repeat_crt = 0;
                                istate->gyro_info.sstate->crt_4_multiple_state.crt_repeat_times ++;
                                BMI26X_INST_LOG(MED, istate->owner,
                                        "@CRT runs %u times",
                                            istate->gyro_info.sstate->crt_4_multiple_state.crt_repeat_times);
                            } else {
                                BMI26X_INST_LOG(HIGH, istate->owner, "@CRT finish with error:%d",
                                            need_repeat_do_crt);
                                bmi26x_hal_prepare_next_crt_process_due_to_problem(istate, need_repeat_do_crt);
                            }
                            // clean up CRT state
                            bmi26x_hal_crt_cleanup(istate);
                        }
                    }
                }
            } else {
            }
            event = istate->crt_handler.timer_crt_stream->api->get_next_input(
                            istate->crt_handler.timer_crt_stream);
        }
    }

    if (crt_timer_event_handled) {
        istate->crt_handler.crt_state = BMI26X_CRT_NONE;
    }

}


sns_rc bmi26x_hal_crt_evaluate_crt(sns_sensor_instance * const inst)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)inst->state->state;
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_crt_trigger_source_t crt_trigger_source = CRT_TRIGGER_SOURCE_NONE;

    if (istate->gyro_info.sstate->crt_cfg.crt_execution_win == 0) {
        istate->gyro_info.sstate->crt_cfg.crt_execution_win = BMI26X_CRT_PERFORM_ROUTINE_CRT_WIN_IN_MINUTES;
    }

    BMI26X_INST_LOG(MED, inst, "@CRT routine:%u/min, en.int:0x%x",
                    istate->gyro_info.sstate->crt_cfg.crt_execution_win,
                    istate->int_en_flags_curr.flag);

    if (istate->int_en_flags_curr.bits.md &&
                    ((istate->fifo_info.ff_sensors_en_curr & BMI26X_GYRO) == 0)) {
        int64_t ts_delta = bmi26x_get_sys_tick() - istate->gyro_info.sstate->crt_4_multiple_state.ts_crt_latest_passed;
        uint64_t ts_itvl_for_multiple_crt = (uint64_t)(istate->gyro_info.sstate->crt_cfg.crt_execution_win);

        ts_itvl_for_multiple_crt =  ts_itvl_for_multiple_crt * 60 * 1000 * istate->ticks_in_1ms;

        BMI26X_INST_LOG(MED, inst, "@CRT ts_delta:<0X[%x %x] 0X[%x %x] 0X[%x %x]>",
                        (uint32_t) (ts_delta >> 32),
                        (uint32_t) ts_delta,
                        (uint32_t) (ts_itvl_for_multiple_crt >> 32),
                        (uint32_t) (ts_itvl_for_multiple_crt),
                        (uint32_t) (istate->gyro_info.sstate->crt_4_multiple_state.ts_crt_latest_passed >> 32),
                        (uint32_t) istate->gyro_info.sstate->crt_4_multiple_state.ts_crt_latest_passed);

        if (istate->fac_test_in_progress) {
            BMI26X_INST_LOG(HIGH, inst, "@CRT fac in process, it's not right time to do crt");
            return SNS_RC_SUCCESS;
        }

        if (istate->gyro_info.sstate->crt_4_multiple_state.ts_crt_latest_passed == 0) {
            // first time, no need to check the time interval
            crt_trigger_source = CRT_TRIGGER_SOURCE_FIRST_TIME;
        } else {
            if (ts_delta > (int64_t) ts_itvl_for_multiple_crt) {
                crt_trigger_source = CRT_TRIGGER_SOURCE_ROUTINE;
            } else if (istate->gyro_info.sstate->crt_4_multiple_state.need_repeat_crt) {
                crt_trigger_source = CRT_TRIGGER_SOURCE_REPEATE_ON_ERROR;
            } else {
                crt_trigger_source = CRT_TRIGGER_SOURCE_NONE;
            }
        }

        BMI26X_INST_LOG(MED, inst, "@CRT trigger:%u, crt_state:%u",
                        crt_trigger_source, istate->crt_handler.crt_state);

        if (crt_trigger_source > CRT_TRIGGER_SOURCE_NONE &&
                        (istate->crt_handler.crt_state == BMI26X_CRT_NONE)) {
            // check the time delta
            sns_time                time_out_tick;
            float dur_us = istate->md_info.sstate->md_config.win * 1000.0 * 1000.0;  // sec --> us

            time_out_tick = bmi26x_convert_us2ticks(dur_us) * BMI26X_CRT_TIMEOUT_TIMES_BY_MD_DURATION;

            if (bmi26x_hal_start_timer(inst, istate->crt_handler.timer_crt_stream,
                        false, time_out_tick)) {
            }
            istate->crt_handler.ts_timeout_tk_expectation = time_out_tick +
                        bmi26x_get_sys_tick();
            istate->crt_handler.ts_timeout_tk_itvl = time_out_tick;

            istate->crt_handler.crt_state = BMI26X_CRT_TIMER_START;
            BMI26X_INST_LOG(MED, istate->owner, "@CRT It's time to do CRT result from:%d",
                    crt_trigger_source);
        } else {
            BMI26X_INST_LOG(MED, inst, "@CRT No need to do crt now (%u < %u)",
                            (uint32_t)ts_delta,
                            (uint32_t)ts_itvl_for_multiple_crt);
        }
    } else {
        BMI26X_INST_LOG(MED, inst, "@CRT Not ready for do crt now: %d 0x%x",
                        istate->int_en_flags_curr.bits.md,
                        istate->fifo_info.ff_sensors_en_curr);

        if (istate->crt_handler.crt_state > BMI26X_CRT_NONE) {
            BMI26X_INST_LOG(MED, inst, "@CRT WARN!!! CRT in progress:%d, try to abort it",
                            istate->crt_handler.crt_state);
            rc = bmi26x_crt_abrot_process(inst);
            BMI26X_INST_LOG(MED, inst, "@CRT aborted rc:%d, crt.st:%d",
                            rc, istate->crt_handler.crt_state);
            //istate->crt_handler.crt_state = BMI26X_CRT_NONE;
        }
    }

    return rc;
}

#endif

