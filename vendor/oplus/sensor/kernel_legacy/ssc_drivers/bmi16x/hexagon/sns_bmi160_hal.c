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
#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"
#include "sns_gpio_service.h"
#include "sns_cal_util.h"
#include "sns_sensor_util.h"

#include "sns_bmi160_hal.h"
#include "sns_bmi160_sensor.h"
#include "sns_bmi160_sensor_instance.h"

#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#if BMI160_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#endif

#include "sns_timer.pb.h"
#include "sns_std_event_gated_sensor.pb.h"

#include "sns_cal.pb.h"

extern struct bmi160_odr_regv_map BMI160_REGV_ODR_MAP[];
extern const range_attr bmi160_accel_ranges[];
extern const float bmi160_accel_resolutions[];
extern const range_attr bmi160_gyro_ranges[];
extern const float bmi160_gyro_resolutions[];

/**
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */

    static
void bmi160_hal_config_polling_timer_4_temp(sns_sensor_instance *const this)
{
    bmi160_instance_state *istate = (bmi160_instance_state*)this->state->state;

    BMI160_INST_LOG(MED, this, "sample interval:%u. timer is active:%d", (uint32_t) istate->sensor_temp_info.sampling_intvl,
            (istate->sensor_temp_info.timer_is_active | (istate->sensor_temp_info.timer_itvl_changed << 1)));
    if (istate->sensor_temp_info.sampling_intvl > 0)
    {
        if (!istate->sensor_temp_info.timer_is_active || istate->sensor_temp_info.timer_itvl_changed)
        {
            if (istate->timer_data_stream == NULL)
            {
                BMI160_INST_LOG(MED, this, "timer data stream created");
                istate->stream_mgr->api->create_sensor_instance_stream(
                        istate->stream_mgr,
                        this,
                        istate->timer_suid,
                        &istate->timer_data_stream);
            }
            bmi160_hal_start_sensor_temp_polling_timer(this);

            istate->sensor_temp_info.timer_itvl_changed = false;
        } else {
            BMI160_INST_LOG(MED, this, "timer reconfig ignored");
        }
    }
    else if (istate->sensor_temp_info.timer_is_active)
    {
        istate->sensor_temp_info.timer_is_active = false;
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->timer_data_stream);
        BMI160_INST_LOG(MED, this, "timer removed");
    }
}




#define BMI160_SPEC_MD_THRESH_RESOLUTION 3.9025f
    static
void bmi160_hal_determine_md_param(
        bmi160_instance_state       *istate,
        bmi160_int_cfg_any_motion_t *param)
{
    uint8_t param_thresh;   //bit 0:1
    uint8_t param_dur;  //bit 0:7

    float mg;
    float dur_ms;
    float itvl_ms;

    mg = istate->md_info.sstate->md_config.thresh * 1000 / G;
    dur_ms = istate->md_info.sstate->md_config.win * 1000;

    param_thresh = mg / BMI160_SPEC_MD_THRESH_RESOLUTION;
    param_thresh = param_thresh >> (istate->md_info.range_idx_req - BMI160_RANGE_ACC_PM2G);

    itvl_ms = (1e3 * (1 << (BMI160_REGV_ODR_1600HZ - BMI160_CONFIG_MD_ODR))) / 1600;
    param_dur = (dur_ms / itvl_ms);

    if (param_dur >= 1) {
        param_dur--;
    }

    param_dur = param_dur < 2 ? param_dur : 2;

    param->int_anym_dur = param_dur;
    param->int_anym_th = param_thresh;
}






static sns_rc bmi160_hal_set_md_config(sns_sensor_instance *const instance, bool enable)
{
    sns_rc                      rc;
    uint8_t                     regv;
    bmi160_instance_state       *istate = (bmi160_instance_state*)instance->state->state;

    if (enable) {
        bmi160_int_cfg_any_motion_t param_md;

        bmi160_hal_determine_md_param(istate, &param_md);

        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_MOTION_0, 0, 1, param_md.int_anym_dur);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_MOTION_1, 0, 7, param_md.int_anym_th);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv = 0;
        regv = BST_SET_VAL_BIT(regv, 7);
        regv = BST_SET_VAL_BITBLOCK(regv, 4, 6, BMI160_CONFIG_MD_ACC_BWP);
        regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, BMI160_CONFIG_MD_ODR);
        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_ACC_CONF, 0, 7, regv);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_ACC_MODE_LP);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        istate->accel_info.normal_mode_req_sent = 0;
        istate->accel_info.in_lpm = 1;
        //istate->accel_info.odr_curr = BMI160_REGV_ODR_OFF;
    } else {
    }


    return SNS_RC_SUCCESS;
}


static
void bmi160_hal_fifo_invalidate_master_info(
    bmi160_instance_state *istate)
{
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    fifo_info->ff_tm_info.fc_masters_last_dev_ff = 0;
    fifo_info->ff_tm_info.fc_accum_masters = 0;
}



static void bmi160_hal_fifo_update_curr_masters(bmi160_instance_state *istate)
{
    uint8_t                     ff_masters = 0;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    if (fifo_info->publish_sensors & BMI160_ACCEL) {
        if (istate->accel_info.ff_wml_req) {
            if (istate->accel_info.odr_curr == fifo_info->ff_master_odr_req) {
                ff_masters |= BMI160_ACCEL;
            }
        }
    }

    if (fifo_info->publish_sensors & BMI160_GYRO) {
        if (istate->gyro_info.ff_wml_req) {
            if (istate->gyro_info.odr_curr == fifo_info->ff_master_odr_req) {
                ff_masters |= BMI160_GYRO;
            }
        }
    }
    //TODOMAG

#if BMI160_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        ff_masters = 0;
    }
#endif

    BMI160_INST_LOG(MED, istate->owner, "bmi160_cp_ fifo_update_master: <0x%x 0x%x %d %d>",
            (int)ff_masters, (int)fifo_info->ff_master_sensors_curr,
            (int)fifo_info->ff_master_odr_curr, (int)fifo_info->ff_master_odr_req);

    if (!(ff_masters & fifo_info->ff_master_sensors_curr) ||
            (fifo_info->ff_master_odr_curr != fifo_info->ff_master_odr_req)) {
        bmi160_hal_fifo_invalidate_master_info(istate);
    }

    //update
    fifo_info->ff_master_sensors_curr = ff_masters;
    fifo_info->ff_master_odr_curr = fifo_info->ff_master_odr_req;

    fifo_info->ff_master_sensors_ready = 0;
    fifo_info->ff_all_synced = 0;
}


static
int32_t bmi160_hal_fifo_get_wml_compensated(
    struct bst_sbus_spec    *bus,
    uint32_t                ff_wml_bytes,
    uint16_t                odr_acc,
    uint16_t                odr_gyr,
    uint16_t                odr_mag)
{
    int32_t ff_wml_bytes_compensated = 0;
    uint32_t bits;
    uint32_t traffic_delay_us;
    uint16_t fc_new_acc = 0;
    uint16_t fc_new_gyr = 0;
    uint16_t fc_new_mag = 0;
    uint16_t fc_new_max;

    if (bus->type) {
        //SPI
        bits = (((ff_wml_bytes + 1)
                * (8 + BMI160_CONFIG_SEE_SPI_BYTE_XFER_WAIT_CYCLES)) + 2);
    } else {
        bits = (30 + (ff_wml_bytes * 9));
    }

    traffic_delay_us = bits * 1e6 / bus->clk_rate;

    traffic_delay_us += BMI160_CONFIG_SEE_ASYNC_COM_DELAY_US;

    if (odr_acc > 0) {
        fc_new_acc = BST_CEIL_P_BUF(
                ((traffic_delay_us) * odr_acc) * 1e-6, 0.2);
    }

    if (odr_gyr > 0) {
        fc_new_gyr = BST_CEIL_P_BUF(
                ((traffic_delay_us) * odr_gyr) * 1e-6, 0.2);
    }

    if (odr_mag > 0) {
        fc_new_mag = BST_CEIL_P_BUF(
                ((traffic_delay_us) * odr_mag) * 1e-6, 0.2);
    }

    fc_new_max = fc_new_acc;

    fc_new_max = (fc_new_max > fc_new_gyr) ? fc_new_max : fc_new_gyr;
    fc_new_max = (fc_new_max > fc_new_mag) ? fc_new_max : fc_new_mag;

    ff_wml_bytes_compensated = (int32_t) ff_wml_bytes
            - (fc_new_acc * BMI160_FF_DATA_LEN_ACC)
            - (fc_new_gyr * BMI160_FF_DATA_LEN_GYR)
            - (fc_new_mag * BMI160_FF_DATA_LEN_MAG) - fc_new_max;

    if (ff_wml_bytes_compensated <= 0) {
        ff_wml_bytes_compensated = ff_wml_bytes;
    }

    return ff_wml_bytes_compensated;
}



//OPTIM
    static
sns_rc bmi160_hal_config_shared_fifo(sns_sensor_instance *this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    sns_rc                      rc;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    struct bst_sbus_spec        bus_spec;

    uint8_t                     regv;
    uint16_t                    ff_wml_bytes;

    uint16_t                    odr_acc;
    uint16_t                    odr_gyr;
    uint16_t                    odr_mag;

    uint8_t                     ff_sensors_en_orig = fifo_info->ff_sensors_en_curr;


    regv = 0;
    if (fifo_info->publish_sensors & BMI160_ACCEL) {
        regv = BST_SET_VAL_BIT(regv, 6);
    }

    if (fifo_info->publish_sensors & BMI160_GYRO) {
        regv = BST_SET_VAL_BIT(regv, 7);
    }

    //TODOMAG

#if BMI160_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        regv = 0;
    }
#endif

    if (fifo_info->publish_sensors & (BMI160_ACCEL | BMI160_GYRO | BMI160_MAG))  {
        regv = BST_SET_VAL_BIT(regv, 1);
    }

    regv = BST_SET_VAL_BIT(regv, 4);

    rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_FIFO_CONFIG_1, 0, 7, regv);
    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    istate->cache_regv_fifo_config_1 = regv;

    //INSERT_TRACE_POINT4_T(bmi160, 'F', 'F', fifo_info->ff_sensors_en_curr, fifo_info->ff_sensors_en_req);

#if BMI160_CONFIG_ENABLE_DEBUG_TEST
    if (!(fifo_info->ff_sensors_en_curr & BMI160_ACCEL)) {
        if (fifo_info->ff_sensors_en_req & BMI160_ACCEL) {
            g_bmi160_cnt_session_acc++;
        }
    }

    if (!(fifo_info->ff_sensors_en_curr & BMI160_GYRO)) {
        if (fifo_info->ff_sensors_en_req & BMI160_GYRO) {
            g_bmi160_cnt_session_gyr++;
        }
    }
#endif

    fifo_info->ff_sensors_en_curr = fifo_info->publish_sensors & (BMI160_ACCEL | BMI160_GYRO | BMI160_MAG);

#if BMI160_CONFIG_ENABLE_DRI_MODE
    if (istate->int_en_flags_req.bits.drdy.flag) {
        fifo_info->ff_sensors_en_curr = 0;
    }
#endif

    //we have calls of config_acc() and config_gyr() ahead of this
    bmi160_hal_fifo_update_curr_masters(istate);

    //config wml in hw
    bus_spec.type = (SNS_BUS_SPI == istate->com_port_info.com_config.bus_type);
    bus_spec.clk_rate = (istate->com_port_info.com_config.min_bus_speed_KHz + istate->com_port_info.com_config.max_bus_speed_KHz) * 500; //500: / 2 * 1000

    odr_acc = BMI160_REGV_ODR_MAP[istate->accel_info.odr_req].odr;
    odr_gyr = BMI160_REGV_ODR_MAP[istate->gyro_info.odr_req].odr;
    odr_mag = 0;    //TODOMAG
    ff_wml_bytes = bmi160_hal_fifo_get_wml_compensated(&bus_spec, fifo_info->ff_wml_bytes_req, odr_acc, odr_gyr, odr_mag);

    //INSERT_TRACE_POINT4_T(bmi160, 'F', fifo_info->ff_wml_bytes_req & 0xff, (fifo_info->ff_wml_bytes_req >> 8)&0xff, ff_wml_bytes);
    regv = ff_wml_bytes >> 2;

    //if (regv > 0)
    if ((regv > 0) && (fifo_info->ff_sensors_en_curr))
    {
        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_FIFO_CONFIG_0, 0, 7, regv);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    fifo_info->ff_wml_bytes_curr = ff_wml_bytes;


    BMI160_INST_LOG(HIGH, istate->owner, "ff_wml: %d", ff_wml_bytes);


    if (fifo_info->ff_sensors_en_curr) {
        bmi160_hal_update_couple_ts_host_and_dev_rt(istate);
        BMI160_INST_LOG(HIGH, istate->owner, "ts_pair<%u,0x%x>",
                BMI160_SYS_TIME_LH(istate->ts_pair_sys_dev.ts_sys),
                istate->ts_pair_sys_dev.ts_dev);

        istate->ts_pair_sys_dev_4_ts_res_est = istate->ts_pair_sys_dev;


        fifo_info->ff_tm_info.boost_read_size = 1;
#if BMI160_CONFIG_ENABLE_TS_REF_SPECULATION
        fifo_info->devi_est_irq = 0;
#endif

        fifo_info->ff_itvl_ticks_est_masters = istate->ts_hw_res_ticks_per_bit * (1 << (BMI160_REGV_ODR_1600HZ - fifo_info->ff_master_odr_curr + 4));
        fifo_info->ff_itvl_ticks_est_acc = istate->ts_hw_res_ticks_per_bit * (1 << (BMI160_REGV_ODR_1600HZ - istate->accel_info.odr_curr + 4));
        fifo_info->ff_itvl_ticks_est_gyr = istate->ts_hw_res_ticks_per_bit * (1 << (BMI160_REGV_ODR_1600HZ - istate->gyro_info.odr_curr + 4));

    } else {
        if (ff_sensors_en_orig) {
            bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_FIFO_FLUSH);
        }

        istate->ts_pair_sys_dev.avail_1st = 0;
    }

    istate->accel_info.ff_wml_curr = istate->accel_info.ff_wml_req;
    istate->gyro_info.ff_wml_curr = istate->gyro_info.ff_wml_req;


    return SNS_RC_SUCCESS;
}


//OPTIM
    static
sns_rc bmi160_hal_config_gyro(sns_sensor_instance *this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t                     regv;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    if (istate->hw_mod_needed & BMI160_GYRO) {
        if (fifo_info->publish_sensors & BMI160_GYRO) {
            if (!istate->gyro_info.normal_mode_req_sent) {
                rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_GYR_MODE_NORMAL);
                BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                istate->gyro_info.normal_mode_req_sent = 1;
            }
        }

        if (istate->gyro_info.range_curr != istate->gyro_info.range_req) {
            regv = istate->gyro_info.range_req;

            rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_GYR_RANGE, 0, 7, regv);
            BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
            istate->gyro_info.range_curr = istate->gyro_info.range_req;
        }

        bmi160_gyro_sstvt gyro_ssvt_array[] = {
            BMI160_GYRO_SSTVT_125DPS,
            BMI160_GYRO_SSTVT_250DPS,
            BMI160_GYRO_SSTVT_500DPS,
            BMI160_GYRO_SSTVT_1000DPS,
            BMI160_GYRO_SSTVT_2000DPS,
        };
        istate->gyro_info.sstvt_curr = roundf(gyro_ssvt_array[istate->gyro_info.sstate->resolution_idx] * (istate->gyro_info.sstate->scale_factor * 1e-6));

        if (fifo_info->publish_sensors & BMI160_GYRO) {
            if (istate->gyro_info.odr_curr != istate->gyro_info.odr_req) {
                if (istate->gyro_info.odr_req != BMI160_REGV_ODR_OFF) {
                    regv = 0;
                    regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, istate->gyro_info.odr_req);
                    regv = BST_SET_VAL_BITBLOCK(regv, 4, 5, BMI160_CONFIG_GYR_BWP);
                    rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_GYR_CONF, 0, 7, regv);
                    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                    istate->gyro_info.odr_curr = istate->gyro_info.odr_req;
                }
            }

#if BMI160_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
            if (istate->gyro_info.odr_req != BMI160_REGV_ODR_OFF) {
                uint8_t downsample_sw_factor_old = istate->gyro_info.downsample_sw_factor;
                if (BMI160_REGV_ODR_MAP[istate->gyro_info.odr_req].odr >= istate->gyro_info.sample_rate_req) {
                    istate->gyro_info.downsample_sw_factor = BMI160_REGV_ODR_MAP[istate->gyro_info.odr_req].odr
                        / istate->gyro_info.sample_rate_req;
                } else {
                    istate->gyro_info.downsample_sw_factor = 1;
                }

                BMI160_INST_LOG(MED, istate->owner, "ds_sw: %d", istate->gyro_info.downsample_sw_factor);

                if (downsample_sw_factor_old != istate->gyro_info.downsample_sw_factor) {
                    istate->gyro_info.downsample_sw_cnt = 0;
                }
            }
#endif
        }


    } else {
        if (istate->gyro_info.normal_mode_req_sent) {
            rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_GYR_MODE_SUSP);
            istate->gyro_info.normal_mode_req_sent = 0;
            istate->gyro_info.range_curr = BMI160_REGV_RANGE_GYR_PM2000DPS;
            //istate->gyro_info.odr_curr = BMI160_REGV_ODR_OFF;
        }
    }

    return SNS_RC_SUCCESS;
}


//OPTIM
static sns_rc bmi160_hal_config_acc(sns_sensor_instance *this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    sns_rc                      rc;
    uint8_t                     regv;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    if (istate->hw_mod_needed & BMI160_ACCEL) {
        if ( (fifo_info->publish_sensors & BMI160_ACCEL)
                || ((fifo_info->publish_sensors & BMI160_SENSOR_TEMP)
                    && !(fifo_info->publish_sensors & BMI160_GYRO))
                //TODOMAG
           ) {
            if (!istate->accel_info.normal_mode_req_sent) {
                rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_ACC_MODE_NORMAL);
                BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                istate->accel_info.normal_mode_req_sent = 1;
            }
        }

        regv = istate->accel_info.range_req;


        BMI160_INST_LOG(HIGH, istate->owner, "config_acc range: %d,%d", istate->accel_info.range_curr, istate->accel_info.range_req);

        if (istate->accel_info.range_curr != istate->accel_info.range_req) {
            rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_ACC_RANGE, 0, 7, regv);
            BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
            istate->accel_info.range_curr = istate->accel_info.range_req;
        }

        bmi160_sstvt_t acc_ssvt_array[] = {
            /* in the unit of micro-g/lsb */
            BMI160_ACCEL_SSTVT_2G,
            BMI160_ACCEL_SSTVT_4G,
            BMI160_ACCEL_SSTVT_8G,
            BMI160_ACCEL_SSTVT_16G,
        };
        istate->accel_info.sstvt_curr = roundf(acc_ssvt_array[istate->accel_info.sstate->resolution_idx] * G * (istate->accel_info.sstate->scale_factor * 1e-6));
        //istate->accel_info.sstvt_curr = BMI160_ACCEL_SSTVT_8G * G;


        if (fifo_info->publish_sensors & BMI160_ACCEL) {
            //if (istate->accel_info.odr_curr != istate->accel_info.odr_req) {
            if (istate->accel_info.odr_req != BMI160_REGV_ODR_OFF) {
                regv = 0;
                regv = BST_SET_VAL_BITBLOCK(regv, 0, 3, istate->accel_info.odr_req);
                regv = BST_SET_VAL_BITBLOCK(regv, 4, 6, BMI160_CONFIG_ACC_BWP);
                rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_ACC_CONF, 0, 7, regv);
                BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                istate->accel_info.odr_curr = istate->accel_info.odr_req;

            }
            //}
        }
    } else {
        if (istate->accel_info.normal_mode_req_sent) {
            rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_ACC_MODE_SUSP);
            istate->accel_info.normal_mode_req_sent = 0;
            istate->accel_info.range_curr = BMI160_REGV_RANGE_ACC_PM2G;
            //istate->accel_info.odr_curr = BMI160_REGV_ODR_OFF;
        }
    }

    return SNS_RC_SUCCESS;
}

#if BMI160_CONFIG_ENABLE_PEDO

static sns_rc bmi160_hal_cfg_hw_pedo(sns_sensor_instance * const inst)
{
    sns_rc rc;
    uint8_t regv;
    bmi160_instance_state *istate = (bmi160_instance_state*) inst->state->state;
    bool en_step_cnt = istate->pedo_info.enable_pedo_int;
    static bool in_fac_mode_ever = false;

    BMI160_INST_LOG(MED,
                    inst,
                    "pedo hw cfg keep_pedo_in_fac_mode %d in_fac_mode_ever %d ",
                    istate->keep_pedo_in_fac_mode,
                    in_fac_mode_ever);
//change regv to BMI160_REGV_STEP_CONF_0_NORMAL for 0x15 in reg 0x7A (normal mode)
    if (istate->keep_pedo_in_fac_mode) {
        regv = BMI160_REGV_STEP_CONF_0_SENSITIVE;
    } else {
        regv = BMI160_REGV_STEP_CONF_0_NORMAL;
    }
    rc = bmi160_sbus_write_wrapper(istate,
    BMI160_REGA_USR_STEP_CONF_0,
                                   &regv, 1);
    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    //this register write will reset the step count
    if (istate->keep_pedo_in_fac_mode || in_fac_mode_ever) {
        regv = BMI160_CONFIG_PEDO_REGV_CONF_1_FAC;
        in_fac_mode_ever = true;
        BMI160_INST_LOG(HIGH,
                        inst,
                        "pedo hw cfg BMI160_CONFIG_PEDO_REGV_CONF_1_FAC");
    } else {
        regv = BMI160_CONFIG_PEDO_REGV_CONF_1;
        BMI160_INST_LOG(HIGH,
                        inst,
                        "pedo hw cfg BMI160_CONFIG_PEDO_REGV_CONF_1 ");
    }
    if (en_step_cnt) {
        regv = BST_SET_VAL_BIT(regv, 3);
    } else {
        regv = BST_CLR_VAL_BIT(regv, 3);
    }

    rc = bmi160_sbus_write_wrapper(istate,
                                   BMI160_REGA_USR_STEP_CONF_1,
                                   &regv, 1);
    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    istate->pedo_info.step_count_hw_last = 0;

    // power
    if (istate->accel_info.normal_mode_req_sent == 0) {
#if BMI160_FORCE_SENSOR_IN_NORMAL_MODE
#else
        rc = bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_ACC_MODE_LP);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
#endif
        istate->accel_info.normal_mode_req_sent = 0;
        istate->accel_info.in_lpm = 1;
    }

    return SNS_RC_SUCCESS;
}

static void bmi160_hal_start_pedo_polling_timer(sns_sensor_instance *this)
{
    bmi160_instance_state *istate = (bmi160_instance_state*)this->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    BMI160_INST_LOG(MED, this, "will start pedo polling timer");

    if (NULL == istate->pedo_info.pedo_timer_data_stream)
    {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service *strm_svc =
            (sns_stream_service*) smgr->get_service(smgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this,
                istate->timer_suid, &istate->pedo_info.pedo_timer_data_stream);
    }

    if(rc != SNS_RC_SUCCESS
            || NULL == istate->pedo_info.pedo_timer_data_stream) {
        BMI160_INST_LOG(ERROR, this, "error creating stream %d", rc);
        return;
    }

    bmi160_hal_start_timer(this, istate->pedo_info.pedo_timer_data_stream,
                           true,
                           istate->pedo_info.sampling_intvl);
    BMI160_INST_LOG(MED, this, "started pedo polling timer");
}


static
void bmi160_hal_config_pedo(sns_sensor_instance *  const inst)
{
    bmi160_instance_state *istate = (bmi160_instance_state *) inst->state->state;
    bool need_hw_recfg = false;

    if (istate->pedo_info.sampling_intvl > 0) {
        if (!istate->pedo_info.timer_is_active) {
            if (istate->pedo_info.pedo_timer_data_stream == NULL) {
                BMI160_INST_LOG(MED, inst, "pedo timer data stream created");
                istate->stream_mgr->api->create_sensor_instance_stream(istate->stream_mgr,
                                                                       inst,
                                                                       istate->timer_suid,
                                                                       &istate->pedo_info.pedo_timer_data_stream);
            }
	     bmi160_hal_start_pedo_polling_timer(inst);
            istate->pedo_info.timer_is_active = true;
            need_hw_recfg = true;
	    istate->pedo_info.is_first = true;
	    BMI160_INST_LOG(HIGH, inst, "pedo_info.is_first: %d",istate->pedo_info.is_first);
        } else {
            BMI160_INST_LOG(MED, inst, "pedo timer already run! ignored cfg");
        }
    } else {
        if (istate->pedo_info.timer_is_active) {
            istate->pedo_info.timer_is_active = false;
            need_hw_recfg = true;
            sns_sensor_util_remove_sensor_instance_stream(inst,
                                                          &istate->pedo_info.pedo_timer_data_stream);
            BMI160_INST_LOG(MED, inst, "pedo timer removed");
        }
    }

    // hw configure
    if (need_hw_recfg) {
        bmi160_hal_cfg_hw_pedo(inst);
    }
}
#endif


#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
static bool bmi160_hal_precheck_hb_need_reset(sns_sensor_instance * this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    if ((istate->accel_info.odr_req != istate->accel_info.odr_curr) ||
            (istate->accel_info.ff_wml_req != istate->accel_info.ff_wml_curr) ||
            (istate->gyro_info.odr_req != istate->gyro_info.odr_curr) ||
            (istate->gyro_info.ff_wml_req != istate->gyro_info.ff_wml_curr) ||
            (istate->int_en_flags_req.bits.fifo.flag != istate->int_en_flags_curr.bits.fifo.flag)) {
        return true;
    }
    return false;
}
#endif

static void bmi160_hal_latch_mode_assert(bmi160_instance_state *istate)
{
    // check latch, BMI160_REGA_USR_INT_LATCH
    sns_rc rc = SNS_RC_SUCCESS;

    if (istate->int_en_flags_curr.bits.md) {
        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_LATCH, 0, 3, BMI160_CONFIG_INT_LATCH_REGV);
    } else {
        //rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_LATCH, 0, 3, BMI160_CONFIG_INT_LATCH_REGV);
    }

    if (rc != SNS_RC_SUCCESS) {
    }
}


//OPTIM
    static
sns_rc bmi160_hal_do_config_hw_now(sns_sensor_instance  *this, bmi160_hw_cfg_ctx_t hw_cfg_ctx)
{
    sns_rc                      rc;
    bool                        enable_fifo_stream = true;
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    BMI160_INST_LOG(HIGH, istate->owner, "<dchw> hw_mod_needed: %d publish_sensors: %d",
            istate->hw_mod_needed, fifo_info->publish_sensors);

#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
    bool need_reset_hb = bmi160_hal_precheck_hb_need_reset(this);
#endif

    //config power, range odr of accel
    rc = bmi160_hal_config_acc(this);

    //config power, range odr of gyro
    rc |= bmi160_hal_config_gyro(this);

    //TODOMAG

    //<config_shared_fifo>
    rc |= bmi160_hal_config_shared_fifo(this);
    //</config_shared_fifo>


    //<config_interrupts>
    enable_fifo_stream = fifo_info->publish_sensors & (BMI160_ACCEL | BMI160_GYRO | BMI160_MAG);
    enable_fifo_stream |= istate->fac_test_in_progress;

    if (istate->irq_ready)
    {
        if (enable_fifo_stream
                || istate->int_en_flags_req.bits.md) {
            rc |= bmi160_hal_config_int_output(this, true, BMI160_INT_PIN1);
            BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }
    }

#if BMI160_CONFIG_ENABLE_PEDO
    // pedometer
    bmi160_hal_config_pedo(this);
#endif

    if (istate->int_en_flags_req.bits.md) {
        rc |= bmi160_hal_set_md_config(this, true);
        rc |= bmi160_hal_config_int_md(this,  true, false);
    } else {
        if (istate->md_info.client_present) {
            rc |= bmi160_hal_config_int_md(this, false, true);
        }
    }


#if BMI160_CONFIG_ENABLE_DRI_MODE
    rc |= bmi160_hal_config_int_drdy(this, istate->int_en_flags_req.bits.drdy.flag);
#endif

    BMI160_INST_LOG(MED, this, "md int:0x%x, enable_fifo_stream=%d",
                    istate->int_en_flags_req.bits.md | (istate->md_info.client_present << 1), enable_fifo_stream);
    rc |= bmi160_hal_config_int_fifo(this, enable_fifo_stream);


    istate->int_en_flags_curr.flag = istate->int_en_flags_req.flag;
    //</config_interrupts>



    //<config_dae>
    // Enable timer in case of sensor temp clients
    BMI160_INST_LOG(MED, this, "publish sensors:0x%x", fifo_info->publish_sensors);
    if (fifo_info->publish_sensors & BMI160_SENSOR_TEMP) {
        bool avail_dae_if = false;

#if BMI160_CONFIG_ENABLE_DAE
        avail_dae_if = bmi160_dae_if_available(this);
#endif
        if (!avail_dae_if) {
            bmi160_hal_config_polling_timer_4_temp(this);
        }
    }
    else
    {
        bmi160_hal_stop_tempetature_timer(this);
        //bmi26x_hal_pwr_cmd_handler(istate, BMI26X_PWR_TEMP_SUSPEND);//bryan add
    }

    istate->sensor_temp_info.sample_rate_curr = (uint16_t)(istate->sensor_temp_info.sample_rate_req);

#if BMI160_CONFIG_ENABLE_DAE
    //    if (0 != fifo_info->publish_sensors) {
    if ((0 != fifo_info->publish_sensors) || (istate->int_en_flags_req.bits.md)) {
        if (!bmi160_dae_if_start_streaming(this)) {
            //TODO
        }
    }
#endif
    //</config_dae>

#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
    if (enable_fifo_stream) {
        if (!istate->fac_test_in_progress) {
            if (need_reset_hb == false) {
                BMI160_INST_LOG(MED, this, "seems no need to reset timer @%u",
                                (uint32_t)bmi160_get_sys_tick());
            } else {
                BMI160_INST_LOG(HIGH, this, "NOTICE!!! restart HB @%u",
                        (uint32_t)bmi160_get_sys_tick());
                bmi160_hal_prepare_hb_timer(istate);
                //bmi160_remove_hb_timer(this);
                bmi160_restart_hb_timer(this, true);
                if (hw_cfg_ctx == BMI160_HW_CFG_CLIENT_HB_ATTACK_REQ) {
                } else {
                    istate->hb_cfg_info.hb_timeout_action_flag = BMI160_HB_TIMER_NONE;
                    istate->hb_cfg_info.heart_attack_cnt = 0;
                }
            }
        } else {
            BMI160_INST_LOG(MED, this, "fac in progress, remove HB");
            //bmi160_remove_hb_timer(this);
        }
    } else {
        // no-any-streaming req , remove the hb timer
        BMI160_INST_LOG(MED, this, "Notice!!! no any stream now, remove HB");
        //bmi160_remove_hb_timer(this);
    }
#else
    UNUSED_VAR(hw_cfg_ctx);
#endif

#if BMI160_CONFIG_ENABLE_LOWG
    rc = bmi160_hal_inst_en_acc_lowg(istate, true);
    bmi160_hal_dump_reg(this);
#endif


    istate->config_step = BMI160_CONFIG_IDLE; /* done with reconfig */

    bmi160_hal_send_config_event(this);

//bryan why? only in bmi160
    if (fifo_info->publish_sensors & BMI160_ACCEL) {
        bmi160_send_fac_cal_event(this, istate->accel_info.sstate);
    }

    if (fifo_info->publish_sensors & BMI160_GYRO) {
        bmi160_send_fac_cal_event(this, istate->gyro_info.sstate);
    }
//bryan end

    bmi160_hal_latch_mode_assert(istate);

    bmi160_hal_dump_reg(this);

    return SNS_RC_SUCCESS;
}


//TODO: check, it seems that framework takes care of flush? SNS_STD_MSGID_SNS_STD_FLUSH_REQ
//OPTIM
    static
sns_rc bmi160_hal_fifo_prepare_4_cos(sns_sensor_instance *this, bmi160_hw_cfg_ctx_t hw_cfg_ctx)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    sns_rc                      rc;

    uint8_t                     ff_sensors_to_invalidate = 0;
    uint8_t                     ff_sensors_en_curr_dur_transit = 0;

    bool                        ff_flush_needed = false;
    uint8_t                     regv;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;
//bryan add
#if BMI160_CONFIG_ENABLE_DAE
    bool ds_ag_need_stop = false;
    bool ds_temp_need_stop = false;
#endif
//bryan add end

    ff_sensors_to_invalidate = bmi160_hal_fifo_get_sensors_to_invalidate(istate);

    BMI160_INST_LOG(MED, istate->owner, "sensors disabled checking: 0x%x 0x%x on ctx:%d",
                    ff_sensors_to_invalidate, fifo_info->ff_sensors_flush_done_before_invalid,
                    hw_cfg_ctx);

    //if (ff_sensors_to_invalidate)
    if (ff_sensors_to_invalidate & (~fifo_info->ff_sensors_flush_done_before_invalid)) {
        ff_flush_needed = true;

        rc = bmi160_sbus_read_wrapper(istate, BMI160_REGA_USR_FIFO_CONFIG_1, &regv, 1);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        ff_sensors_en_curr_dur_transit = BST_GET_VAL_BITBLOCK(regv, 5, 7);
        if (ff_sensors_to_invalidate & BMI160_ACCEL) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 1);
        }

        if (ff_sensors_to_invalidate & BMI160_GYRO) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 2);
        }

        if (ff_sensors_to_invalidate & BMI160_MAG) {
            ff_sensors_en_curr_dur_transit = BST_CLR_VAL_BIT(ff_sensors_en_curr_dur_transit, 0);
        }

        if (BST_GET_VAL_BITBLOCK(regv, 5, 7) != ff_sensors_en_curr_dur_transit) {
            regv = BST_SET_VAL_BITBLOCK(regv, 5, 7, ff_sensors_en_curr_dur_transit);
            rc = bmi160_sbus_write_wrapper(istate, BMI160_REGA_USR_FIFO_CONFIG_1, &regv, 1);
            BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

            BMI160_INST_LOG(HIGH, istate->owner, "sensors disabled during transition: %d", ff_sensors_to_invalidate);
        }
    } else {
        BMI160_INST_LOG(HIGH, istate->owner, "bmi160_cp_ sensors not disabled during transition: %d,%d",
                ff_sensors_to_invalidate, fifo_info->ff_sensors_flush_done_before_invalid);
    }

    fifo_info->ff_sensors_flush_done_before_invalid = 0;


    //if overall wml goes lower
    if (fifo_info->ff_wml_bytes_req < fifo_info->ff_wml_bytes_curr) {
        if ((HW_CONFIG_CTX_ON_DEF_FLUSH_PENDING == hw_cfg_ctx) ||
                (HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS == hw_cfg_ctx)) {
            BMI160_INST_LOG(MED, istate->owner, "abandon this change due to fifo aleady flushed");
        } else {
            ff_flush_needed = true;
        }
    }


    if (ff_flush_needed) {
        if (fifo_info->ff_flush_in_proc) {
            BMI160_INST_LOG(HIGH, istate->owner, "ff_flush_in_proc trigger: %d", fifo_info->ff_flush_trigger);
        } else {
#if  BMI160_CONFIG_ENABLE_DAE
            // dae mode, dae already handle this
            uint8_t sensor_stop = (uint8_t) (BMI160_ACCEL | BMI160_GYRO);
            if (bmi160_dae_if_available(this)) {
                if (istate->config_step == BMI160_CONFIG_IDLE &&
                    bmi160_dae_if_stop_streaming(this, sensor_stop)) {
                    //istate->fifo_info.ff_flush_in_proc = true;
                    //istate->config_step = BMI160_CONFIG_STOPPING_STREAM;
                    ds_ag_need_stop = true;
                }
            } else {
                // NON-DAE
                BMI160_INST_LOG(MED, istate->owner, "#flush# start to flush witout dae");
                bmi160_hal_fifo_drain(istate, false, BMI160_FIFO_FLUSH_TRIGGER_HW_CHANGE);
            }
#else
            bmi160_hal_fifo_drain(istate, false, BMI160_FIFO_FLUSH_TRIGGER_HW_CHANGE);
#endif
        }
    }

//bryan add
#if  BMI160_CONFIG_ENABLE_DAE
    if (istate->sensor_temp_info.timer_itvl_changed) {
        ds_temp_need_stop = bmi160_dae_if_stop_streaming(this, BMI160_SENSOR_TEMP);
    }
    if (ds_ag_need_stop || ds_temp_need_stop) {
        istate->fifo_info.ff_flush_in_proc = true;
        istate->config_step = BMI160_CONFIG_STOPPING_STREAM;
    }
#endif
//bryan added

    fifo_info->ff_sensors_to_invalidate = ff_sensors_to_invalidate;

    return SNS_RC_SUCCESS;
}


sns_rc bmi160_hal_dev_reconfig_hw(sns_sensor_instance *this, bmi160_hw_cfg_ctx_t hw_cfg_ctx)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;

    sns_rc                      rc = SNS_RC_SUCCESS;
    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    //INSERT_TRACE_POINT3_T(bmi160, 'H', fifo_info->ff_sensors_en_req, fifo_info->ff_sensors_en_curr);
    BMI160_INST_LOG(MED, this, "reconfig_hw: %d %d on ctx:%d",
                    fifo_info->ff_sensors_en_req, fifo_info->ff_sensors_en_curr, hw_cfg_ctx);
    //we cannot change the HW setting when FIFO async read is still pending
    if (fifo_info->ff_flush_in_proc) {
        BMI160_INST_LOG(HIGH, this, "hw_config_pending due to fifo_flush 1: %d", fifo_info->ff_flush_trigger);
        istate->hw_config_pending = 1;
        return SNS_RC_SUCCESS;
    }

    rc = bmi160_hal_fifo_prepare_4_cos(this, hw_cfg_ctx);

    if (!fifo_info->ff_flush_in_proc) {
        istate->hw_config_pending = 0;

        bmi160_hal_fifo_invalidate_sensors(istate, fifo_info->ff_sensors_to_invalidate);
        fifo_info->ff_sensors_to_invalidate = 0;
    } else {
        istate->hw_config_pending = 1;

        BMI160_INST_LOG(HIGH, istate->owner, "hw_config_pending due to fifo_flush 2: 0x%x",
                        (fifo_info->ff_sensors_to_invalidate << 8) | fifo_info->ff_flush_trigger);

        return SNS_RC_SUCCESS;
    }

    rc = bmi160_hal_do_config_hw_now(this, hw_cfg_ctx);
    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return SNS_RC_SUCCESS;
}



static sns_rc bmi160_hal_prepare_spi_if(bmi160_instance_state *istate)
{
    sns_rc                      rc;
    uint8_t                     regv;

    rc = bmi160_sbus_read_wrapper(istate, BMI160_REGA_CMD_EXT_MODE, &regv, 1);
    bmi160_delay_us(BMI160_SPEC_IF_SPI_SWITCH_TIME_US);
    BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

    return SNS_RC_SUCCESS;
}



static
sns_rc bmi160_dev_sw_reset(
    sns_sensor_instance     *instance,
    uint8_t        sensor)
{
    UNUSED_VAR(instance);
    UNUSED_VAR(sensor);

    bmi160_instance_state       *istate = (bmi160_instance_state*)instance->state->state;
    bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_SOFT_RESET);
    bmi160_delay_us(BMI160_SPEC_SOFT_RESET_DELAY_TIME_MS * 1000);

    return SNS_RC_SUCCESS;
}

sns_rc bmi160_hal_reset_device(
        sns_sensor_instance     *instance,
        bool                    need_device_reset)
{
    sns_rc                      rc = SNS_RC_SUCCESS;
    uint8_t                     regv;
    bmi160_instance_state       *istate = (bmi160_instance_state*)instance->state->state;
    uint8_t sensor_to_reset = (uint8_t)(BMI160_ACCEL | BMI160_GYRO | BMI160_MAG | BMI160_MOTION_DETECT | BMI160_SENSOR_TEMP);

    /** HW reset only when both Accel and Gyro are requested for
     *  reset. */
    if (need_device_reset) {
        rc = bmi160_dev_sw_reset(instance,
                                 (uint8_t)(BMI160_ACCEL | BMI160_GYRO | BMI160_MAG | BMI160_MOTION_DETECT | BMI160_SENSOR_TEMP));

        if (SNS_BUS_SPI == istate->com_port_info.com_config.bus_type) {
            bmi160_hal_prepare_spi_if(istate);
        }

        rc |= bmi160_sbus_read_wrapper(istate, BMI160_REGA_USR_CHIP_ID, &regv, 1);
        rc |= bmi160_sbus_read_wrapper(istate, BMI160_REGA_USR_PMU_STATUS, &regv, 1);
        rc |= bmi160_sbus_read_wrapper(istate, BMI160_REGA_USR_ACC_RANGE, &regv, 1);
    } else {
        // reset only reset the sensor power/odr/int...
        // disable fifo: 0x47, int, 0x55 - 0x57, power
        regv = 0;
        rc = bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_FIFO_CONFIG_1, 1, 7, regv);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS)

        regv = 0;
        rc |= bmi160_sbus_write_wrapper(istate, BMI160_REGA_USR_INT_EN_0, &regv , 1);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv = 0;
        rc |= bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_EN_1, 0, 6, regv);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        regv = 0;
        rc |= bmi160_dev_reg_read_modify_write(istate, BMI160_REGA_USR_INT_EN_2, 0, 3, regv);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        bmi160_hal_set_default_state(instance, (uint8_t)sensor_to_reset);

        // disable power
        rc |= bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_GYR_MODE_SUSP);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi160_delay_us(500);

        rc |= bmi160_hal_send_cmd(istate, BMI160_REGV_CMD_ACC_MODE_SUSP);
        BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        bmi160_delay_us(500);
    }

    if (rc == SNS_RC_SUCCESS) {
        bmi160_hal_set_default_state(instance, (uint8_t)sensor_to_reset);
    }

    return rc;
}


/** See sns_bmi160_hal.h */
void bmi160_hal_send_config_event(sns_sensor_instance *const instance)
{
    bmi160_instance_state *istate = (bmi160_instance_state*)instance->state->state;

    sns_std_sensor_physical_config_event phy_sensor_config =
        sns_std_sensor_physical_config_event_init_default;

    char operating_mode[] = "NORMAL";
    char operating_mode_lp[] = "LOW POWER";

    pb_buffer_arg op_mode_args;

    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    struct bmi160_state         *sstate;

    /** If no sensors are enabled for streaming then don't send
     *  config event */
    if (!istate->hw_mod_needed)
    {
        return;
    }


    phy_sensor_config.has_sample_rate = true;
    phy_sensor_config.sample_rate = 0.0;

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = istate->accel_info.ff_wml_curr;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = BMI160_ACC_ACTIVE_CURRENT;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.has_stream_is_synchronous = true;
    phy_sensor_config.stream_is_synchronous = false;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = true;
    /* For sensors that route data through the SDC/DAE sensor, the DAE watermark
       should be set to the number of samples stored in SDC before waking up Q6. */
    phy_sensor_config.has_DAE_watermark = bmi160_dae_if_available(instance);
    phy_sensor_config.DAE_watermark =
#if BMI160_CONFIG_ENABLE_DAE
                    (istate->accel_info.flush_period_ticks == 0) ?
                    UINT32_MAX :
                    istate->fifo_info.ff_wml_bytes_curr / 7;
#else
    0
#endif
    ;

    if (fifo_info->publish_sensors & BMI160_ACCEL)
    {
        sstate = istate->accel_info.sstate;
        phy_sensor_config.resolution = bmi160_accel_resolutions[sstate->resolution_idx];
        phy_sensor_config.range[0] = bmi160_accel_ranges[sstate->resolution_idx].min;
        phy_sensor_config.range[1] = bmi160_accel_ranges[sstate->resolution_idx].max;

        phy_sensor_config.sample_rate = BMI160_REGV_ODR_MAP[istate->accel_info.odr_curr].odr;

        phy_sensor_config.water_mark = SNS_MAX(phy_sensor_config.water_mark, 1);    //per req alignment from vikram

#if BMI160_CONFIG_ENABLE_SEE_LITE
        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
#else
        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);
#endif
        pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &sstate->my_suid);
    }

    if (fifo_info->publish_sensors & BMI160_GYRO)
    {
#if BMI160_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
        phy_sensor_config.sample_rate = (istate->gyro_info.downsample_sw_factor <= 1) ?
            //BMI160_REGV_ODR_MAP[istate->gyro_info.odr_req].odr : istate->gyro_info.sample_rate_req;
            BMI160_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr : (BMI160_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr / istate->gyro_info.downsample_sw_factor);

        phy_sensor_config.water_mark = (istate->gyro_info.downsample_sw_factor <= 1) ?
            istate->gyro_info.ff_wml_curr : (istate->gyro_info.ff_wml_curr / istate->gyro_info.downsample_sw_factor);

        phy_sensor_config.water_mark = SNS_MAX(phy_sensor_config.water_mark, 1);
#else
        phy_sensor_config.sample_rate = BMI160_REGV_ODR_MAP[istate->gyro_info.odr_curr].odr;

        phy_sensor_config.water_mark = istate->gyro_info.ff_wml_curr;

        phy_sensor_config.water_mark = SNS_MAX(phy_sensor_config.water_mark, 1);    //per req alignment from vikram
#endif
        // Override above values with gyro info
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = BMI160_GYRO_ACTIVE_CURRENT;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = true;
        /* For sensors that route data through the SDC/DAE sensor, the DAE watermark
           should be set to the number of samples stored in SDC before waking up Q6. */

        sstate = istate->gyro_info.sstate;
        phy_sensor_config.resolution = bmi160_gyro_resolutions[sstate->resolution_idx];
        phy_sensor_config.range[0] = bmi160_gyro_ranges[sstate->resolution_idx].min;
        phy_sensor_config.range[1] = bmi160_gyro_ranges[sstate->resolution_idx].max;

        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);
        pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &sstate->my_suid);
    }


    if (fifo_info->publish_sensors & BMI160_SENSOR_TEMP)
    {
        // Override above values with sensor temperature info
        phy_sensor_config.sample_rate = (float)(istate->sensor_temp_info.sample_rate_curr);

        phy_sensor_config.has_water_mark = false;
        phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 180;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = BMI160_SENSOR_TEMPERATURE_RESOLUTION;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = BMI160_SENSOR_TEMPERATURE_RANGE_MIN;
        phy_sensor_config.range[1] = BMI160_SENSOR_TEMPERATURE_RANGE_MAX;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = false;
        phy_sensor_config.has_DAE_watermark = false;
        phy_sensor_config.DAE_watermark = 1;

#if BMI160_CONFIG_ENABLE_SEE_LITE
        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
#else
        op_mode_args.buf = &operating_mode[0];
        op_mode_args.buf_len = sizeof(operating_mode);
#endif
        pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &istate->sensor_temp_info.sstate->my_suid);
    }

    //this section has to be at the very end because of the sns_memset
    if (istate->int_en_flags_curr.bits.md)
    {
        sns_memset(&phy_sensor_config, 0, sizeof(phy_sensor_config));

        op_mode_args.buf = &operating_mode_lp[0];
        op_mode_args.buf_len = sizeof(operating_mode_lp);
        phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 46;  //per configuration: ODR=100,AVG=1
        phy_sensor_config.has_resolution = false;
        phy_sensor_config.range_count = 0;
        phy_sensor_config.has_stream_is_synchronous = true;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_DAE_watermark = false;

        pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &istate->md_info.sstate->my_suid);
    }

#if BMI160_CONFIG_ENABLE_PEDO
    if (fifo_info->publish_sensors & BMI160_PEDO)
    {
        if (istate->pedo_info.sampling_intvl > 0) {
              phy_sensor_config.sample_rate = (float)sns_convert_ns_to_ticks(1000000000) / istate->pedo_info.sampling_intvl;
        } else {
              phy_sensor_config.sample_rate = 0;
        }

        phy_sensor_config.has_water_mark = false;
        phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 180;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = 1;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = BMI160_PEDO_RANGE_MIN;
        phy_sensor_config.range[1] = BMI160_PEDO_RANGE_MAX;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = false;
        phy_sensor_config.has_DAE_watermark = false;

        pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &istate->pedo_info.sstate->my_suid);
    }
#endif


    if (fifo_info->publish_sensors & BMI160_ACCEL) {
        bmi160_send_fac_cal_event(instance, istate->accel_info.sstate);
    }

    if (fifo_info->publish_sensors & BMI160_GYRO) {
        bmi160_send_fac_cal_event(instance, istate->gyro_info.sstate);
    }

}


void bmi160_send_fac_cal_event(
        sns_sensor_instance *const  instance,
        const struct bmi160_state   *sstate)
{
    bmi160_instance_state *istate = (bmi160_instance_state*)instance->state->state;

    const sns_sensor_uid            *suid = &(sstate->my_suid);

    sns_cal_event new_calibration_event = sns_cal_event_init_default;
    float bias_data[] = {0,0,0};
    float comp_matrix_data[] = {0,0,0,0,0,0,0,0,0};

    UNUSED_VAR(istate);

    bias_data[0] = sstate->fac_cal_bias[0] * 1.0f / sstate->scale_factor; // values in istate are usually from registry or from factory test routine
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

    pb_buffer_arg buff_arg_bias = (pb_buffer_arg)
    { .buf = &bias_data, .buf_len = ARR_SIZE(bias_data) };
    pb_buffer_arg buff_arg_comp_matrix = (pb_buffer_arg)
    { .buf = &comp_matrix_data, .buf_len = ARR_SIZE(comp_matrix_data) };

    new_calibration_event.bias.funcs.encode = &pb_encode_float_arr_cb;
    new_calibration_event.bias.arg = &buff_arg_bias;
    new_calibration_event.comp_matrix.funcs.encode = &pb_encode_float_arr_cb;
    new_calibration_event.comp_matrix.arg = &buff_arg_comp_matrix;
    new_calibration_event.status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    BMI160_INST_LOG(HIGH, instance, "bmi cal evt: %d <%d %d %d> ver: %d", sstate->sensor,
            (int)(bias_data[0] * 1000), (int)(bias_data[1] * 1000), (int)(bias_data[2] * 1000),
            sstate->fac_cal_version);

    pb_send_event(instance,
            sns_cal_event_fields,
            &new_calibration_event,
            sns_get_system_time(),
            SNS_CAL_MSGID_SNS_CAL_EVENT,
            suid);
}


static void bmi160_hal_fifo_validate_n_adj_wml_req(
        bmi160_instance_state   *istate,
        uint32_t                *ff_wml_req_acc,
        uint32_t                *ff_wml_req_gyr,
        uint32_t                *ff_wml_req_mag)
{
    uint32_t                    mcd;
    uint32_t                    wml_adj;
    uint32_t                    wml_adj_needed= 0;
    uint32_t                    ff_max_len = 0;

    UNUSED_VAR(istate);

    mcd = bmi160_util_get_com_div(*ff_wml_req_acc, *ff_wml_req_gyr);
    mcd = bmi160_util_get_com_div(mcd, *ff_wml_req_mag);

    wml_adj_needed = ((*ff_wml_req_acc) >= BMI160_FF_MAX_FRAMES_IMU);
    wml_adj_needed |= (((*ff_wml_req_gyr) >= BMI160_FF_MAX_FRAMES_IMU) << 1);
    //TODOMAG

    wml_adj = mcd;
    while (true) {
        ff_max_len = (*ff_wml_req_acc) ? ((BMI160_FF_DATA_LEN_ACC + 1) * wml_adj) : 0;
        ff_max_len += (*ff_wml_req_gyr) ? ((BMI160_FF_DATA_LEN_GYR + 1) * wml_adj) : 0;
        //ff_max_len += *ff_wml_req_mag ? ((BMI160_FF_DATA_LEN_MAG + 1) * wml_adj) : 0;   //TODOMAG

        if ((ff_max_len < (BMI160_FF_DEPTH_BYTES - BMI160_CONFIG_FIFO_HEADROOM)) && (wml_adj < BMI160_FF_MAX_FRAMES_IMU)) {
            break;
        } else {
            wml_adj = bmi160_util_get_max_div(mcd, BST_MIN((wml_adj - 1), BMI160_FF_MAX_FRAMES_IMU));

            wml_adj_needed |= (ff_max_len >= (BMI160_FF_DEPTH_BYTES - BMI160_CONFIG_FIFO_HEADROOM)) ? (1 << 8) : 0;
        }
    }


    BMI160_INST_LOG(MED, istate->owner, "wml_adj <%d %d %d>",
            *ff_wml_req_acc, *ff_wml_req_gyr, *ff_wml_req_mag);

    if (wml_adj_needed) {
        BMI160_INST_LOG(MED, istate->owner, "wml_adj_needed <0x%x %d>",
                wml_adj_needed, wml_adj);

        *ff_wml_req_acc = wml_adj;
        *ff_wml_req_gyr  = wml_adj;

        istate->accel_info.ff_wml_req = wml_adj;
        istate->gyro_info.ff_wml_req = wml_adj;
#if BMI160_CONFIG_ENABLE_MAG_IF //TODOMAG
        *ff_wml_req_mag  = wml_adj;
        istate->mag_info.ff_wml_req = wml_adj;
#endif
    }
}

void bmi160_hal_fifo_calc_wml_req_ldt(bmi160_instance_state *istate)
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

    bmi160_fifo_info            *fifo_info = &istate->fifo_info;

    ff_wml_req_acc = istate->accel_info.ff_wml_req;
    ff_wml_req_gyr = istate->gyro_info.ff_wml_req;

    bmi160_hal_fifo_validate_n_adj_wml_req(istate, &ff_wml_req_acc, &ff_wml_req_gyr, &ff_wml_req_mag);

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

    fc_wml_master = bmi160_util_get_com_div(fc_when_time_expires_acc, fc_when_time_expires_gyr);
    fc_wml_master = bmi160_util_get_com_div(fc_wml_master, fc_when_time_expires_mag);

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
        BMI160_FF_DATA_LEN_ACC * fc_wml_acc_max +
        BMI160_FF_DATA_LEN_GYR * fc_wml_gyr_max +
        BMI160_FF_DATA_LEN_MAG * fc_wml_mag_max +
        + fc_wml_master;

    bytes_when_wml_fires =
        BMI160_FF_DATA_LEN_ACC * (fc_wml_master >> (odr_fifo_master - odr_acc)) +
        BMI160_FF_DATA_LEN_GYR * (fc_wml_master >> (odr_fifo_master - odr_gyr)) +
        BMI160_FF_DATA_LEN_MAG * (fc_wml_master >> (odr_fifo_master - odr_mag)) +
        + fc_wml_master;



    if ((1000000 * fc_wml_master / BMI160_REGV_ODR_MAP[odr_fifo_master].odr) <= BMI160_CONFIG_FIFO_WMI_MISSING_ITVL_US_THRES) {
        fifo_info->ff_wmi_missing_possible = 1;
        BMI160_INST_LOG(HIGH, istate->owner, "bmi160_cp_ ff_wmi_missing_possible");
    } else {
        fifo_info->ff_wmi_missing_possible = 0;
    }


    //OUTPUT
    BMI160_INST_LOG(HIGH, istate->owner, "odr <%d %d %d %d>",
            odr_acc, odr_gyr, odr_mag, odr_fifo_master);

    BMI160_INST_LOG(HIGH, istate->owner, "fc_when_time_expires<%d %d %d> fc_wml_master: %d",
            fc_when_time_expires_acc, fc_when_time_expires_gyr, fc_when_time_expires_mag, fc_wml_master);

    fifo_info->ff_suggested_bytes2read = bytes_when_time_expires_max;

    BMI160_INST_LOG(HIGH, istate->owner, "[calc_wml_ldt] ff_wml_bytes: %d ff_suggested_bytes2read: %d",
            bytes_when_wml_fires, fifo_info->ff_suggested_bytes2read);

    if (bytes_when_wml_fires > (BMI160_FF_DEPTH_BYTES - BMI160_CONFIG_FIFO_HEADROOM)) {
        bytes_when_wml_fires = BMI160_FF_DEPTH_BYTES - BMI160_CONFIG_FIFO_HEADROOM;
        fifo_info->ff_suggested_bytes2read = bytes_when_wml_fires;
    }

    fifo_info->ff_wml_bytes_req = bytes_when_wml_fires;

}

