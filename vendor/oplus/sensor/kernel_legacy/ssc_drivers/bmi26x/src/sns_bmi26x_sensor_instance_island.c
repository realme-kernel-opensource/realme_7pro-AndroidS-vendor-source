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
 * @file sns_bmi26x_sensor_instance_island.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_island_service.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor.h"
#include "sns_bmi26x_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_std_event_gated_sensor.pb.h"

#if BMI26X_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#endif

#include "sns_sync_com_port_service.h"
#include "sns_cal_util.h"

extern struct bmi26x_odr_regv_map BMI26X_REGV_ODR_MAP[];

bmi26x_state* bmi26x_inst_get_master_sensor_state(
    sns_sensor_state const      *caller_sensor_state)
{
    bmi26x_state           *sstate = (bmi26x_state *) caller_sensor_state->state;
    sns_sensor             *sensor_this = sstate->owner;
    sns_sensor             *sensor;
    bmi26x_state           *sensor_state = NULL;

    if (sstate->sensor == BMI26X_ACCEL) {
        return sstate;
    }

    for (sensor = sensor_this->cb->get_library_sensor(sensor_this, true);
            sensor != NULL;
            sensor = sensor_this->cb->get_library_sensor(sensor_this, false)) {
        sensor_state = (bmi26x_state*)sensor->state->state;
        if (BMI26X_ACCEL == sensor_state->sensor) {
            return sensor_state;
        }
    }

    return NULL;
}



#if BMI26X_CONFIG_ENABLE_PEDO_TIMER

static
sns_rc bmi26x_validate_pedo_odr(sns_sensor_instance * const inst)
{
    sns_rc rc = SNS_RC_SUCCESS;
#if BMI26X_CONFIG_ENABLE_PEDO
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;

    if (istate->pedo_info.enable_pedo_int) {
        // 1s is 1, 10s=0.1, 100s = 0.01:
        // pedo always use a predefine report rate to report data
        istate->pedo_info.sample_rate_req = BMI26X_CONFIG_PEDO_LOWEST_ODR;
        istate->pedo_info.report_rate_req = BMI26X_CONFIG_PEDO_LOWEST_ODR;
        istate->pedo_info.sampling_intvl = sns_convert_ns_to_ticks(1000000000.0 / BMI26X_SENSOR_TEMP_ODR_5);
    } else {
        istate->pedo_info.sample_rate_req = 0.0f;
        istate->pedo_info.report_rate_req = 0.0f;
        istate->pedo_info.sampling_intvl = 0;
    }

#else
    UNUSED_VAR(inst);
#endif

    return rc;
}


static sns_rc bmi26x_inst_handle_pedo_event(
    sns_sensor_instance     *const inst)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;
    sns_sensor_event *event;

    if ((istate->fifo_info.publish_sensors & BMI26X_PEDO) &&
            (NULL != istate->pedo_info.pedo_timer_data_stream)) {
        event = istate->pedo_info.pedo_timer_data_stream->api->peek_input(
                    istate->pedo_info.pedo_timer_data_stream);

        while (NULL != event) {
            BMI26X_INST_LOG(LOW, inst, "timer evt:%d", event->message_id);
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                  event->event_len);
            if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                sns_timer_sensor_event timer_event;
                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                    if ((istate->fifo_info.publish_sensors & BMI26X_PEDO)
                            &&
                            istate->pedo_info.timer_is_active) {
                        if (bmi26x_hal_handle_pedo(inst, timer_event.timeout_time, BMI26X_PEDO_EVT_CTX_TIMEOUT) != SNS_RC_SUCCESS) {
                        }
                    }
                }
            } else if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT) {
                /** TODO: decode and qse timer_reg_event*/
                BMI26X_INST_LOG(LOW, inst, "TIMER_SENSOR_REG_EVENT");
            } else {
                BMI26X_INST_LOG(MED, inst, "unknown message_id %d", event->message_id);
            }
            event = istate->pedo_info.pedo_timer_data_stream->api->get_next_input(
                        istate->pedo_info.pedo_timer_data_stream);
        }
    }

    return SNS_RC_SUCCESS;
}
#endif

/**
 * validate the sensor temperature odr
 * @param istate   instance state handler
 *
 * @return  SNS_RC_SUCCESS on success  otherwise value on failure
 */
static void
bmi26x_validate_sensor_temp_odr(bmi26x_instance_state *istate)
{
    if (!BST_IS_FLOAT_ZERO(istate->sensor_temp_info.sample_rate_req)) {
        if (istate->sensor_temp_info.sample_rate_req <= BMI26X_SENSOR_TEMP_ODR_1) {
            istate->sensor_temp_info.sample_rate_req = BMI26X_SENSOR_TEMP_ODR_1;
        } else {
            istate->sensor_temp_info.sample_rate_req = BMI26X_SENSOR_TEMP_ODR_5;
        }
    }

    if (!BST_IS_FLOAT_ZERO(istate->sensor_temp_info.sample_rate_req)) {
        istate->sensor_temp_info.sampling_intvl_req = sns_convert_ns_to_ticks(1000000000.0 /
                istate->sensor_temp_info.sample_rate_req);
    } else {
        istate->sensor_temp_info.sampling_intvl_req = 0;
    }

}

static
void bmi26x_inst_determine_max_wml_req(bmi26x_instance_state *istate)
{
    uint32_t wml_max = 0;

    uint8_t  odr_wml_max = BMI26X_ODR_0;
    float    scale_wml;

    if (istate->accel_info.ff_wml_req > 0) {
        wml_max = istate->accel_info.ff_wml_req;
        odr_wml_max = istate->accel_info.odr_req;
    }

    if (istate->gyro_info.ff_wml_req > istate->accel_info.ff_wml_req) {
        wml_max = istate->gyro_info.ff_wml_req;
        odr_wml_max = istate->gyro_info.odr_req;
    }
    //TODOMAG

    wml_max = (wml_max > 0) ? wml_max : BMI26X_FF_MAX_FRAMES_IMU;

    if (istate->fifo_info.publish_sensors & BMI26X_ACCEL) {
        if (0 == istate->accel_info.ff_wml_req) {   //this is true when is_max_batch is set
            if (odr_wml_max > BMI26X_ODR_0) {
                scale_wml = BMI26X_REGV_ODR_MAP[istate->accel_info.odr_req].odr / BMI26X_REGV_ODR_MAP[odr_wml_max].odr;
                istate->accel_info.ff_wml_req = wml_max * scale_wml;
            } else {
                istate->accel_info.ff_wml_req = wml_max;
            }
        }
    }

    if (istate->fifo_info.publish_sensors & BMI26X_GYRO) {
        if (0 == istate->gyro_info.ff_wml_req) {
            if (odr_wml_max > BMI26X_ODR_0) {
                scale_wml = BMI26X_REGV_ODR_MAP[istate->gyro_info.odr_req].odr / BMI26X_REGV_ODR_MAP[odr_wml_max].odr;
                istate->gyro_info.ff_wml_req = wml_max * scale_wml;
            } else {
                istate->gyro_info.ff_wml_req = wml_max;
            }
        }
    }
}

#if BMI26X_CONFIG_ENABLE_ERROR_ON_HIGHODR
void bmi26x_inst_report_error(
    bmi26x_instance_state   *istate,
    bmi26x_sensor_type      sensor_type,
    sns_sensor_uid          *uid,
    uint32_t                error)
{
    sns_sensor_instance         *this = istate->owner;
    sns_service_manager         *mgr = this->cb->get_service_manager(this);
    sns_event_service           *event_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
    sns_sensor_event            *event = event_service->api->alloc_event(event_service, this, 0);
    UNUSED_VAR(sensor_type);
    UNUSED_VAR(error);

    if (NULL != event) {
        event->message_id = SNS_STD_MSGID_SNS_STD_ERROR_EVENT;
        event->event_len = 0;
        event->timestamp = sns_get_system_time();

        //event->event_len = sizeof(event->event);
        event->event[0] = (sensor_type << 16) | error;
        //use helper publish_error()
        //OPTIM3
        event_service->api->publish_event(event_service, this, event, uid);
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! std_error_ev %d %d", sensor_type, error);
    }
}
#endif

/**
 * Access all the requests
 * @param istate    instance state handler
 *
 * @return  none
 */
void bmi26x_inst_assess_overall_req(bmi26x_instance_state *istate)
{
    bmi26x_regv_odr_t   regv_odr;
    float               odr_matched;
    bool need_recfg_hw = false;

    // @pedo
#if BMI26X_CONFIG_ENABLE_PEDO
    rc = bmi26x_validate_pedo_odr(istate->owner);
    if (istate->pedo_info.sample_rate_req > 0.0) {
        istate->accel_info.sample_rate_req = SNS_MAX(istate->pedo_info.sample_rate_req,
                                             istate->accel_info.sample_rate_req);
    }
    BMI26X_INST_LOG(LOW, istate->owner, "#pedo# en: %d acc:ss(10):%u pedo:sample itvl:%u",
                    istate->pedo_info.enable_pedo_int,
                    (uint32_t)(istate->accel_info.sample_rate_req * 10),
                    (uint32_t)istate->pedo_info.sampling_intvl);
#endif

    //accel_info
    if (istate->accel_info.sample_rate_req > 0) {
        istate->accel_info.sample_rate_req = SNS_MAX(istate->accel_info.sample_rate_req, BMI26X_CONFIG_ACC_LOWEST_ODR);
        istate->accel_info.sample_rate_req = SNS_MIN(istate->accel_info.sample_rate_req, BMI26X_CONFIG_ACC_FASTEST_ODR);
        if (istate->accel_info.sample_rate_req > BMI26X_CONFIG_ACC_FASTEST_ODR) {
            istate->accel_info.sample_rate_req = BMI26X_CONFIG_ACC_FASTEST_ODR;
#if BMI26X_CONFIG_ENABLE_ERROR_ON_HIGHODR
            bmi26x_inst_report_error(istate, BMI26X_ACCEL, &istate->accel_info.sstate->my_suid, 0);
#endif
        }
    }

    bmi26x_hal_match_odr(BMI26X_ACCEL, istate->accel_info.sample_rate_req, &odr_matched, &regv_odr);

#if BMI26X_CONFIG_ENABLE_LOWG
    if (istate->lowg_info.sstate->lowg_config.debug) {
        // XXX Notice!!! Free fall debug under 200hz
        regv_odr = BMI26X_REGV_ODR_200HZ;
        BMI26X_INST_LOG(HIGH, istate->owner,
                "WARN!!! NOTICE free fall debug mode enable, acc odr change to 200hz");
    }
#endif

    istate->accel_info.odr_req = regv_odr;

    if (istate->accel_info.report_rate_req > odr_matched) {
        istate->accel_info.report_rate_req = odr_matched;
        BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! NOTICE acc rr lt sr");
    }

    if (BST_IS_FLOAT_ZERO(istate->accel_info.report_rate_req)) {
        istate->accel_info.ff_wml_req = 0;
        BMI26X_INST_LOG(HIGH, istate->owner, "acc_rr:zero");
    } else {
        istate->accel_info.ff_wml_req = odr_matched / istate->accel_info.report_rate_req;
    }

    {
        bmi26x_regv_acc_range_t acc_range_regv_array[] = {
            BMI26X_REGV_RANGE_ACC_PM2G,
            BMI26X_REGV_RANGE_ACC_PM4G,
            BMI26X_REGV_RANGE_ACC_PM8G,
            BMI26X_REGV_RANGE_ACC_PM16G,
        };

        istate->accel_info.range_req   = acc_range_regv_array[istate->accel_info.sstate->resolution_idx];
        BMI26X_INST_LOG(LOW, istate->owner, "acc resolution idx=%d, range regv:%x",
                        istate->accel_info.sstate->resolution_idx,
                        istate->accel_info.range_req);
    }

    if (istate->gyro_info.sample_rate_req > 0) {
        istate->gyro_info.sample_rate_req = SNS_MAX(istate->gyro_info.sample_rate_req, BMI26X_CONFIG_GYR_LOWEST_ODR);

        istate->gyro_info.sample_rate_req = SNS_MIN(istate->gyro_info.sample_rate_req, BMI26X_CONFIG_GYR_FASTEST_ODR);
        if (istate->gyro_info.sample_rate_req > BMI26X_CONFIG_GYR_FASTEST_ODR) {
            istate->gyro_info.sample_rate_req = BMI26X_CONFIG_GYR_FASTEST_ODR;
#if BMI26X_CONFIG_ENABLE_ERROR_ON_HIGHODR
            bmi26x_inst_report_error(istate, BMI26X_GYRO, &istate->gyro_info.sstate->my_suid, 0);
#endif
        }
    }
    //gyro_info
    bmi26x_hal_match_odr(BMI26X_GYRO, istate->gyro_info.sample_rate_req, &odr_matched, &regv_odr);
    istate->gyro_info.odr_req = regv_odr;

    if (istate->gyro_info.report_rate_req > odr_matched) {
        istate->gyro_info.report_rate_req = odr_matched;
        BMI26X_INST_LOG(HIGH, istate->owner, "WARN!!! NOTICE gyr rr lt sr");
    }

    if (BST_IS_FLOAT_ZERO(istate->gyro_info.report_rate_req)) {
        istate->gyro_info.ff_wml_req = 0;
        BMI26X_INST_LOG(HIGH, istate->owner, "gyr_rr:zero");
    } else {
        istate->gyro_info.ff_wml_req = odr_matched / istate->gyro_info.report_rate_req;
    }

    {
        bmi26x_regv_gyr_range_t gyro_range_regv_array[] = {
            BMI26X_REGV_RANGE_GYR_PM125DPS,
            BMI26X_REGV_RANGE_GYR_PM250DPS,
            BMI26X_REGV_RANGE_GYR_PM500DPS,
            BMI26X_REGV_RANGE_GYR_PM1000DPS,
            BMI26X_REGV_RANGE_GYR_PM2000DPS
        };

        istate->gyro_info.range_req = gyro_range_regv_array[istate->gyro_info.sstate->resolution_idx];
        BMI26X_INST_LOG(LOW, istate->owner, "gyro resolution idx=%d, range regv:%x",
                        istate->gyro_info.sstate->resolution_idx,
                        istate->gyro_info.range_req);
    }

    istate->md_info.range_idx_req = (bmi26x_acc_range_t)istate->accel_info.sstate->resolution_idx;

    bmi26x_inst_determine_max_wml_req(istate);

    bmi26x_hal_fifo_update_master_odr_req(istate);
    bmi26x_hal_fifo_calc_wml_req_ldt(istate);

    istate->int_en_flags_req.bits.fifo.flag = istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);

#if BMI26X_CONFIG_ENABLE_DRI_MODE
    bool dri_mode_allowed;

    //TODOMAG
    //bmi26x_inst_determine_max_wml_req has already made sure that (ff_wml_req > 0) if publish_sensors bit is set
    dri_mode_allowed = (istate->accel_info.ff_wml_req <= 1) && (istate->gyro_info.ff_wml_req <= 1);
    if (dri_mode_allowed) {
        istate->int_en_flags_req.bits.drdy.flag = istate->int_en_flags_req.bits.fifo.flag;
    } else {
        istate->int_en_flags_req.bits.drdy.flag = 0;
    }

    if (istate->fac_test_in_progress) {
        istate->int_en_flags_req.bits.drdy.flag = 0;
    }

    if (istate->int_en_flags_req.bits.drdy.flag) {
        istate->int_en_flags_req.bits.fifo.flag = 0;
    }

    BMI26X_INST_LOG(HIGH, istate->owner, "drdy_flag:%d", istate->int_en_flags_req.bits.drdy.flag);
#endif


    //istate->fifo_info.ff_sensors_en_req = istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);
    istate->fifo_info.ff_sensors_en_req = istate->int_en_flags_req.bits.fifo.flag;

    bmi26x_validate_sensor_temp_odr(istate);
    if (istate->sensor_temp_info.sampling_intvl_req != istate->sensor_temp_info.sampling_intvl_curr) {
        istate->sensor_temp_info.timer_itvl_changed = 1;
    }



    // FIXME
    {
        bmi26x_int_en_flag_t int_req, int_curr;
        int_req.flag = istate->int_en_flags_req.flag;
        int_curr.flag = istate->int_en_flags_curr.flag;
#if BMI26X_CONFIG_ENABLE_LOWG
        int_curr.bits.lowg = int_req.bits.lowg;
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
        int_curr.bits.dbtap = int_req.bits.dbtap;
#endif

        if (int_req.flag != int_curr.flag ||
            istate->accel_info.odr_req != istate->accel_info.odr_curr
            || istate->gyro_info.odr_req != istate->gyro_info.odr_curr
            || istate->fifo_info.ff_sensors_en_req != istate->fifo_info.ff_sensors_en_curr
            || istate->fifo_info.ff_master_odr_req != istate->fifo_info.ff_master_odr_curr
            || istate->fifo_info.ff_wml_bytes_req != istate->fifo_info.ff_wml_bytes_curr
            /* need more info, example, istate->accel_info.ff_wml_req*/
       ) {
            need_recfg_hw = true;
        }
    }

    istate->need_recfg_hw = need_recfg_hw;
}

/**
 * Process COM PORT vector event
 * @param vector         the vector handler
 * @param user_arg       user defined arguments
 *
 * @return none
 */
static
void bmi26x_process_com_port_vector(sns_port_vector *vector, void *user_arg)
{
    sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

    if (BMI26X_REGA_USR_FIFO_DATA == vector->reg_addr) {
        //Vector contains a FIFO buffer read
        if (vector->bytes > 0) {
            bmi26x_hal_process_fifo_data_buffer(instance,
                                                vector->buffer,
                                                vector->bytes,
                                                BMI26X_INT_TRIGGER_SOURCE_SEE);
        }
    }
}




sns_rc bmi26x_inst_handle_event_int(
    sns_sensor_instance     *const this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state *) this->state->state;
    sns_sensor_event            *event;
    sns_rc                      rc = SNS_RC_SUCCESS;
    sns_interrupt_event         irq_event = sns_interrupt_event_init_zero;
    bmi26x_int_check_ctx_t      ctx;

    // Handle interrupts
    if (NULL != istate->interrupt_data_stream) {
        ctx.int_check_trigger = BMI26X_INT_CHECK_TRIGGER_IRQ;
        event = istate->interrupt_data_stream->api->peek_input(istate->interrupt_data_stream);

        while (NULL != event) {

            if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id) {
                int enable_fifo_stream = istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);

                enable_fifo_stream |= istate->fac_test_in_progress;
                BMI26X_INST_LOG(MED, this, "irq_ready event id=%d", event->message_id);
                istate->irq_ready = true;

                if (enable_fifo_stream
                        || istate->int_en_flags_req.bits.md
#if BMI26X_CONFIG_ENABLE_LOWG
                        || istate->int_en_flags_req.bits.lowg
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
                        || istate->int_en_flags_req.bits.dbtap
#endif

                        ) {
                    rc |= bmi26x_hal_config_int_output(this, true);
                }

                if (istate->md_info.enable_md_int) {
                    rc |= bmi26x_hal_config_int_md(this, true, false, (uint8_t)BMI26X_MD_CONFIG_HANDLE_INT);
                }

#if BMI26X_CONFIG_ENABLE_LOWG
                if (istate->lowg_info.enable_lowg_int) {
                    rc |= bmi26x_hal_config_int_lowg(this, true, false, (uint8_t)BMI26X_LOWG_CONFIG_HANDLE_INT);
                }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
                if (istate->dbtap_info.enable_dbtap_int) {
                    rc |= bmi26x_hal_config_int_dbtap(this, true, false, (uint8_t)BMI26X_DBTAP_CONFIG_HANDLE_INT);
                }
#endif

                if (enable_fifo_stream) {
                    rc |= bmi26x_hal_config_int_fifo(this, true);
                }
            } else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id) {
                pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                      event->event_len);

                if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event)) {
                    ctx.timestamp = irq_event.timestamp;
                    rc = bmi26x_hal_handle_interrupt(this, &ctx);
                    BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                }
            } else {
                BMI26X_INST_LOG(ERROR, this, "WARN! Received invalid event id:%d", event->message_id);
            }

            event = istate->interrupt_data_stream->api->get_next_input(istate->interrupt_data_stream);
        }
    } else {
        BMI26X_INST_LOG(ERROR, this, "WARNING!!! interrupt_data_stream is NULL");
    }

    return rc;
}

sns_rc bmi26x_inst_handle_event_async_com(
    bmi26x_instance_state   *istate)
{
    sns_sensor_instance         *const this = istate->owner;
    sns_sensor_event            *event;

    if (NULL != istate->async_com_port_data_stream) {
        bool                    ascp_vector_found = false;

        event = istate->async_com_port_data_stream->api->peek_input(istate->async_com_port_data_stream);
        while (NULL != event) {
            if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id) {
                BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! Received ASCP error event id: %d",
                                 event->message_id);
            } else if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id) {
                pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
                sns_ascp_for_each_vector_do(&stream, bmi26x_process_com_port_vector, (void *)this);

                ascp_vector_found = true;
            }
            event = istate->async_com_port_data_stream->api->get_next_input(istate->async_com_port_data_stream);
        }

        if (ascp_vector_found) {
            if (istate->fac_test_in_progress) {
                bmi26x_hal_inst_exit_island(this);

                bmi26x_process_fac_test(this);
            }
#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
            istate->hb_cfg_info.ts_data_event = sns_get_system_time();
#endif
        }
    }

    return SNS_RC_SUCCESS;
}

static void bmi26x_inst_handle_event_timer(
    bmi26x_instance_state   *istate)
{
    sns_sensor_instance         *const this = istate->owner;
    sns_sensor_event            *event;
    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL != istate->timer_data_stream) {
        event = istate->timer_data_stream->api->peek_input(istate->timer_data_stream);
        while (NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                  event->event_len);
            if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                sns_timer_sensor_event timer_event;
                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                    if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                        if ((istate->fifo_info.publish_sensors & BMI26X_SENSOR_TEMP)
                                &&
                                istate->sensor_temp_info.timer_is_active
                                &&
                                istate->sensor_temp_info.sampling_intvl_curr > 0) {
                            //istate->sensor_temp_info.timer_is_active = false;
                            bmi26x_hal_handle_sensor_temp_sample(this);
                            //bmi26x_hal_start_sensor_temp_polling_timer(this);
                        }
                    }
                }
            } else if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT) {
                /** TODO: decode and qse timer_reg_event*/
                BMI26X_INST_LOG(LOW, this, "TIMER_SENSOR_REG_EVENT");
            } else {
                BMI26X_INST_LOG(MED, this, "unknown message_id %d", event->message_id);
            }
            event = istate->timer_data_stream->api->get_next_input(istate->timer_data_stream);
        }

        if ((istate->fifo_info.publish_sensors & BMI26X_SENSOR_TEMP) == 0) {
            //remove timer stream	//FIXLK
            bmi26x_hal_stop_tempetature_timer(this);
        }
    }

    if (NULL != istate->cmd_handler.timer_cmd_stream) {
        event = istate->cmd_handler.timer_cmd_stream->api->peek_input(istate->cmd_handler.timer_cmd_stream);
        while (NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
            sns_timer_sensor_event timer_event;
            if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    rc = bmi26x_hal_handle_timer_cmd(istate);
                    if (rc != SNS_RC_SUCCESS) {
                        // TODO
                    }
                }
            } else {
            }
            event = istate->cmd_handler.timer_cmd_stream->api->get_next_input(istate->cmd_handler.timer_cmd_stream);
        }
    }

#if BMI26X_CONFIG_ENABLE_PEDO
    if (bmi26x_inst_handle_pedo_event(istate->owner) != SNS_RC_SUCCESS) {
    }
#endif

#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
    bmi26x_hal_handle_hb_timer_event(istate);
#endif

#if BMI26X_CONFIG_ENABLE_CRT
    if (NULL != istate->crt_handler.timer_crt_stream &&
                    istate->crt_handler.crt_state >= BMI26X_CRT_TIMER_START) {
        // handle crt timer event
        bmi26x_hal_inst_exit_island(this);

        bmi26x_hal_handle_timer_crt(istate);
    }
#endif
}

/** See sns_sensor_instance_api::set_client_config */
sns_rc bmi26x_inst_set_client_config(
    sns_sensor_instance     *const this,
    sns_request const       *client_request)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc;

#if BMI26X_CONFIG_ENABLE_DEBUG
    bmi26x_req_payload_t          *req_payload = (bmi26x_req_payload_t *) client_request->request;
    bmi26x_state *sstate = (bmi26x_state *)(req_payload->req_ssensor->state->state);

    BMI26X_INST_LOG(MED, this, "<bmi26x_iif set_client_config> 0x%x @sensor:%d over msg:%d on step:%u",
                    this,
                    (NULL != req_payload) ? sstate->sensor : -1,
                    client_request->message_id,
                    istate->config_step);
#endif

    // Turn COM port ON
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            true);

    if (istate->pwr_state_present != BMI26X_POWER_RAIL_PENDING_NONE) {
        istate->pwr_state_present = BMI26X_POWER_RAIL_PENDING_NONE;
    }

    if (
#if BMI26X_CONFIG_ENABLE_DAE
        !bmi26x_dae_if_available(this)
#else
        1
#endif
    ) {
        bmi26x_hal_register_interrupt(this);
    }

    if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
        // 1. Extract sample, report rates from client_request.
        // 2. Configure sensor HW.
        // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
        // 4. sendRequest() for Interrupt register/de-register in case of DRI using interrupt_data_stream.
        // 5. Save the current configure information like type, sample_rate, report_rate, etc.

        bmi26x_inst_assess_overall_req(istate);

        // NOTICE, these code from sample code
        // new clients need to know ODR/WM for samples received between now and
        // when the next configure takes effect
        if ((istate->accel_info.odr_curr > BMI26X_REGV_ODR_OFF ||
                istate->gyro_info.odr_curr > BMI26X_REGV_ODR_OFF ||
                istate->sensor_temp_info.sampling_intvl_curr > 0)) {
            bmi26x_hal_send_config_event(this);
        }

        if (BMI26X_CONFIG_IDLE == istate->config_step) {
            rc = bmi26x_hal_reconfig_hw(this, HW_CONFIG_CTX_CLIENT);
            BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

    } else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        //TOCHECK
        bool dae_flush_done = false;

#if BMI26X_CONFIG_ENABLE_DAE
        dae_flush_done = bmi26x_dae_if_flush_hw(this, &istate->dae_if.ag);
        if (dae_flush_done) {
            istate->ff_flush_client_req = 1;
        }
#endif

        if (!dae_flush_done) {
            // non-dae mode
            BMI26X_INST_LOG(MED, this, "fifo_flush status: %x",
                            (istate->ff_flush_client_req) |
                            (istate->fifo_info.ff_flush_in_proc << 1) |
                            (istate->fifo_info.ff_flush_trigger << 2) |
                            (istate->hw_config_pending << 3));

            istate->ff_flush_client_req = 1;
            bmi26x_hal_fifo_drain(istate, false, BMI26X_FIFO_FLUSH_TRIGGER_CLIENTS);
        }
#if BMI26X_CONFIG_ENABLE_PEDO
        sns_time ts_curr = bmi26x_get_sys_tick();
        if (istate->fifo_info.publish_sensors & BMI26X_PEDO) {
            bmi26x_hal_send_fifo_flush_done(istate->owner, (uint8_t)BMI26X_PEDO,
                                            ts_curr, 0);
        }
#endif
    } else if (client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
        // 1. Extract test type from client_request.
        // 2. Configure sensor HW for test type.
        // 3. send_request() for Timer Sensor in case test needs polling/waits.
        // 4. Factory test is TBD.

        bmi26x_hal_inst_exit_island(this);

        bmi26x_run_self_test(this);
        istate->new_self_test_request = false;
    }
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    else if (client_request->message_id == SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG) {
        BMI26X_INST_LOG(HIGH, this, "BMI26X : SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG is received");
        /** All self-tests can be handled in normal mode. */
        bmi26x_hal_inst_exit_island(this);

        bmi26x_run_self_test_bias(this);
        istate->new_self_test_request_bias = false;
    }
#endif

    // Turn COM port OFF
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            false);

    return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
BMI26X_FTI
static sns_rc bmi26x_inst_notify_event(
    sns_sensor_instance     *const this)
{
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)this->state->state;
    sns_rc                      rc = SNS_RC_SUCCESS;

    BMI26X_INST_LOG(MED, istate->owner, "<bmi26x_iif_ inst_notify_ev>");

    // Turn COM port ON
    rc = istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
                                                            true);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! bus power update error with true");
    }

#if BMI26X_CONFIG_ENABLE_DAE
    bmi26x_dae_if_process_events(this);
#endif

    // Handle Async Com Port events
    rc |= bmi26x_inst_handle_event_async_com(istate);

    // Handle event int
    rc = bmi26x_inst_handle_event_int(this);

    // Handle Timer events
    bmi26x_inst_handle_event_timer(istate);


    // Turn COM port OFF
    rc |= istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
                                                             false);
    if (rc != SNS_RC_SUCCESS) {
        BMI26X_INST_LOG(ERROR, istate->owner, "ERROR!!! error when update bus pwr:%d", rc);
    }

    BMI26X_INST_LOG(MED, istate->owner, "<bmi26x_if_ inst_notify_ev>>");

    return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */

sns_sensor_instance_api bmi26x_sensor_instance_api = {
    .struct_len             = sizeof(sns_sensor_instance_api),
    .init                   = bmi26x_inst_init,
    .deinit                 = bmi26x_inst_deinit,
    .set_client_config      = bmi26x_inst_set_client_config,
    .notify_event           = bmi26x_inst_notify_event
};
