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
 * @file sns_bmi26x_dae_if_island.c
 *
 * BMI26X - DAE sensor interface
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/


#include "sns_types.h"

#include "sns_bmi26x_hal.h"
#include "sns_bmi26x_sensor.h"
#include "sns_bmi26x_sensor_instance.h"


#if  BMI26X_CONFIG_ENABLE_DAE

#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_service_manager.h"
#include "sns_sensor_util.h"
#include "sns_stream_service.h"
#include "sns_time.h"

#include "sns_dae.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"

#include "sns_bmi26x_hal.h"


/*======================================================================================
  Macros
  ======================================================================================*/

/*======================================================================================
  Extern
  ======================================================================================*/

extern struct bmi26x_odr_regv_map BMI26X_REGV_ODR_MAP[];
/*======================================================================================
  Helper Functions
  ======================================================================================*/
static bool bmi26x_dae_stream_usable(bmi26x_dae_stream_t *dae_stream)
{
    return (NULL != dae_stream->stream && dae_stream->stream_usable);
}


/* ------------------------------------------------------------------------------------ */

static bool bmi26x_dae_pwr_available(bmi26x_instance_state *istate)
{
    bool pwr_available = false;

    if (istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_NONE ||
        istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_DEINIT) {
        pwr_available = true;
    } else {
        BMI26X_INST_LOG(MED, istate->owner, "@DAE,pwr not available st:%d",
                istate->pwr_state_present);
    }

    return pwr_available;
}

static void bmi26x_dae_build_static_config_request(
    bmi26x_state *sstate,
    bmi26x_instance_state *istate,     // NITICE:: this pointer will be NULL on sensoe stage
    sns_dae_set_static_config     *config_req,
    bool                          for_ag)
{
    sns_com_port_config const *com_config  = &sstate->common.com_port_info.com_config;
    sns_async_com_port_config *ascp_config = &config_req->ascp_config;
    uint8_t rigid_body_type;

    BMI26X_SENSOR_LOG(MED, sstate->owner, "@DAE, build static cfg req:<%d %d %d %d %d> @ss:%d",
                      sstate->common.com_port_info.com_config.bus_type,
                      sstate->common.com_port_info.com_config.bus_instance,
                      sstate->common.com_port_info.com_config.slave_control,
                      sstate->common.com_port_info.com_config.reg_addr_type,
                      sstate->common.com_port_info.com_config.max_bus_speed_KHz,
                      sstate->sensor);

    ascp_config->bus_type             = (sns_async_com_port_bus_type)com_config->bus_type;
    ascp_config->slave_control        = com_config->slave_control;
    ascp_config->reg_addr_type        = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
    ascp_config->min_bus_speed_kHz    = com_config->min_bus_speed_KHz;

    if (SNS_BUS_SPI == sstate->common.com_port_info.com_config.bus_type) {
        ascp_config->max_bus_speed_kHz = com_config->max_bus_speed_KHz;
    } else {
        ascp_config->max_bus_speed_kHz = sstate->common.com_port_info.com_config.max_bus_speed_KHz;
    }
    ascp_config->bus_instance         = com_config->bus_instance;

    BMI26X_SENSOR_LOG(MED, sstate->owner, "@DAE:: build static cfg req:<%d %d %d %d %d> @ss:%d",
                      ascp_config->bus_type, ascp_config->bus_instance,
                      ascp_config->slave_control, ascp_config->reg_addr_type,
                      ascp_config->max_bus_speed_kHz, sstate->sensor);

    if (for_ag) {
        triaxis_conversion const *src_axis_map = NULL;
        if (NULL != istate) {
            src_axis_map = istate->accel_info.sstate->common.axis_map;
            rigid_body_type = istate->accel_info.sstate->common.registry_pf_cfg.rigid_body_type;
        } else {
            src_axis_map = sstate->common.axis_map;
            rigid_body_type = sstate->common.registry_pf_cfg.rigid_body_type;
        }
        sensor_to_phone_conversion *dest_axis_map = config_req->accel_info.axis_map;

        sns_strlcpy(config_req->func_table_name, "bmi26x_fifo_hal_table",
                    sizeof(config_req->func_table_name));
        config_req->interrupt          = 1;
        config_req->has_irq_config     = true;
        config_req->irq_config         = sstate->common.irq_config;

        config_req->has_accel_info     = true;
        config_req->accel_info.accel_range = sstate->resolution_idx;
        config_req->accel_info.axis_map_count = ARR_SIZE(config_req->accel_info.axis_map);

        for (uint32_t i = 0; i < config_req->accel_info.axis_map_count; i ++) {
            dest_axis_map[i].ipaxis = src_axis_map[i].ipaxis;
            dest_axis_map[i].opaxis = src_axis_map[i].opaxis;
            dest_axis_map[i].invert = src_axis_map[i].invert;
        }

        // Populate Additional Attributes
        config_req->accel_info.accel_attr[0].value.has_sint = true;
        config_req->accel_info.accel_attr[0].value.sint = rigid_body_type;
        config_req->accel_info.accel_attr[0].attr_id = SNS_STD_SENSOR_ATTRID_RIGID_BODY;
        config_req->accel_info.accel_attr_count = 1;

        BMI26X_SENSOR_LOG(MED, sstate->owner, "@DAE, build static cfg req_2 :<%d %d %d> %d",
                          config_req->irq_config.interrupt_num,
                          config_req->irq_config.is_chip_pin,
                          config_req->irq_config.interrupt_trigger_type,
                          sstate->resolution_idx);
    } else {   // for temperature
        sns_strlcpy(config_req->func_table_name,
                    "bmi26x_temperature_hal_table",
                    sizeof(config_req->func_table_name));
        config_req->interrupt                = 0;
        config_req->has_irq_config        = false;
        config_req->has_accel_info        = false;
    }

    config_req->has_ibi_config         = false;
    config_req->has_s4s_config         = false;
}


/* ------------------------------------------------------------------------------------ */
static sns_rc bmi26x_dae_send_static_config_request(
    sns_data_stream           *stream,
    sns_dae_set_static_config *config_req)
{
    sns_rc rc = SNS_RC_FAILED;
    uint8_t encoded_msg[sns_dae_set_static_config_size];
    sns_request req = {
        .message_id  = SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG,
        .request     = encoded_msg,
        .request_len = 0
    };

    if (NULL == stream ||
            NULL == config_req) {
        return SNS_RC_INVALID_VALUE;
    }

    req.request_len = pb_encode_request(encoded_msg, sizeof(encoded_msg), config_req,
                                        sns_dae_set_static_config_fields, NULL);
    if (0 < req.request_len) {
        rc = stream->api->send_request(stream, &req);
    }
    return rc;
}

/* ------------------------------------------------------------------------------------ */
static bool bmi26x_dae_send_ag_config(bmi26x_dae_stream_t *dae_stream, bmi26x_instance_state* istate)
{
    bool cmd_sent = false;
    sns_accel_dynamic_info *accel_info;
    sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
    uint8_t encoded_msg[sns_dae_set_streaming_config_size];
    sns_request req = {
        .message_id = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
        .request = encoded_msg
    };

    config_req.dae_watermark = istate->fifo_info.ff_wml_bytes_curr  / 7;
    config_req.has_data_age_limit_ticks = true;
    config_req.data_age_limit_ticks = (uint64_t)((110 * SNS_MAX(istate->accel_info.flush_period_ticks,
                                      istate->gyro_info.flush_period_ticks)) / 100);
    config_req.has_polling_config = false;

    config_req.has_accel_info                   = true;
    accel_info                                  = &config_req.accel_info;
    accel_info->odr                             = BMI26X_REGV_ODR_MAP[istate->accel_info.odr_curr].odr;
    accel_info->num_initial_invalid_samples     = 0;
    accel_info->offset_cal_count                = 3;
    accel_info->offset_cal[0]   = istate->accel_info.sstate->fac_cal_bias[0];
    accel_info->offset_cal[1]   = istate->accel_info.sstate->fac_cal_bias[1];
    accel_info->offset_cal[2]   = istate->accel_info.sstate->fac_cal_bias[2];
    accel_info->scale_cal_count = 3;
    accel_info->scale_cal[0]    = istate->accel_info.sstate->fac_cal_corr_mat.e00;
    accel_info->scale_cal[1]    = istate->accel_info.sstate->fac_cal_corr_mat.e11;
    accel_info->scale_cal[2]    = istate->accel_info.sstate->fac_cal_corr_mat.e22;

    config_req.has_expected_get_data_bytes = true;
    if (istate->fifo_info.ff_wml_bytes_curr) {
        config_req.expected_get_data_bytes = istate->fifo_info.ff_wml_bytes_curr + BMI26X_FF_FRAME_LEN_TS + 2;
    }

    BMI26X_INST_LOG(LOW, istate->owner, "@DAE,send_ag_config: age_limit=%u wm=%u #bytes=%u #discard=%u",
                    (uint32_t)config_req.data_age_limit_ticks, config_req.dae_watermark,
                    config_req.expected_get_data_bytes,
                    accel_info->num_initial_invalid_samples);

    if ((req.request_len = pb_encode_request (encoded_msg, sizeof(encoded_msg), &config_req,
                           sns_dae_set_streaming_config_fields,
                           NULL)) > 0) {
        if (SNS_RC_SUCCESS == dae_stream->stream->api->send_request (dae_stream->stream, &req)) {
            dae_stream->state = BMI26X_DAE_IF_STATE_STREAM_STARTING;
            cmd_sent = true;
        }
    }

    return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool
bmi26x_dae_send_temp_config (bmi26x_dae_stream_t *dae_stream, sns_sensor_instance *this)
{
    bool cmd_sent = false;
    sns_time time_now = sns_get_system_time ();
    bmi26x_instance_state *istate = (bmi26x_instance_state*) this->state->state;
    bmi26x_sensor_temp_info_t *temp_info = &istate->sensor_temp_info;
    uint32_t dae_wm = 1; //(uint32_t)(temp_info->sampling_rate_hz / temp_info->report_rate_hz);
    sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_zero;
    uint8_t encoded_msg[sns_dae_set_streaming_config_size];
    sns_request req = {
        .message_id = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
        .request = encoded_msg
    };

    config_req.dae_watermark = dae_wm;
    config_req.has_data_age_limit_ticks = true;
    config_req.data_age_limit_ticks =  (uint64_t)((110 * SNS_MAX(istate->accel_info.flush_period_ticks,
                                       istate->gyro_info.flush_period_ticks)) / 100);
    config_req.has_polling_config = true;
    config_req.polling_config.polling_interval_ticks = temp_info->sampling_intvl_curr;
    config_req.polling_config.polling_offset = (time_now + temp_info->sampling_intvl_curr) / temp_info->sampling_intvl_curr
            * temp_info->sampling_intvl_curr;

    config_req.has_expected_get_data_bytes = true;
    /* DAE temp driver reads status register, and 2 bytes for temp */
    config_req.expected_get_data_bytes     = 2;
    BMI26X_INST_LOG (LOW, this, "@DAE, send_temp_config:: SR=%u RR=%u WM=%u now=%x %08x",
                     (uint32_t) istate->sensor_temp_info.sample_rate_req, (uint32_t) temp_info->report_rate_req,
                     dae_wm, (uint32_t) (time_now >> 32), (uint32_t) time_now);
    BMI26X_INST_LOG (LOW, this, "send_temp_config:: wm=%u age=%u invtl=%u offset=%x %08x", config_req.dae_watermark,
                     (uint32_t) config_req.data_age_limit_ticks,
                     (uint32_t) config_req.polling_config.polling_interval_ticks,
                     (uint32_t) (config_req.polling_config.polling_offset >> 32),
                     (uint32_t) config_req.polling_config.polling_offset);

    if ((req.request_len = pb_encode_request (encoded_msg, sizeof(encoded_msg), &config_req,
                           sns_dae_set_streaming_config_fields,
                           NULL)) > 0) {
        if (SNS_RC_SUCCESS == dae_stream->stream->api->send_request (dae_stream->stream, &req)) {
            dae_stream->state = BMI26X_DAE_IF_STATE_STREAM_STARTING;
            cmd_sent = true;
            //temp_info->cur_sampling_rate_hz = temp_info->desired_sampling_rate_hz;
        } else {
            BMI26X_INST_LOG(ERROR, this,
                            "@DAE ERROR!!! send_temp_config: req fail on stream %x",
                            dae_stream->stream);
        }
    }

    return cmd_sent;
}


/* ------------------------------------------------------------------------------------ */
static bool bmi26x_dae_flush_hw(bmi26x_dae_stream_t *dae_stream)
{
    bool cmd_sent = false;
    sns_request req = {
        .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_HW,
        .request      = NULL,
        .request_len  = 0
    };

    if (SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req)) {
        cmd_sent = true;
        dae_stream->flushing_hw = true;
    }

    return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool bmi26x_dae_flush_samples(sns_sensor_instance const * inst,
                                     bmi26x_dae_stream_t *dae_stream)
{
    bool cmd_sent = false;
    sns_request req = {
        .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS,
        .request      = NULL,
        .request_len  = 0
    };

    if (SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req)) {
        cmd_sent = true;
        dae_stream->flushing_data = true;
        BMI26X_INST_LOG(MED, inst, "@DAE,flush data event send/:%d", SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS);
    } else {
        BMI26X_INST_LOG(ERROR, inst, "@DAE ERROR!!! dae: flush data event send error");
    }

    return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool bmi26x_dae_stop_streaming(bmi26x_dae_stream_t *dae_stream)
{
    bool cmd_sent = false;
    sns_request req = {
        .message_id   = SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING,
        .request      = NULL,
        .request_len  = 0
    };

    if (SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req)) {
        cmd_sent = true;
        dae_stream->state = BMI26X_DAE_IF_STATE_STREAM_STOPPING;
    }

    return cmd_sent;
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


/* ------------------------------------------------------------------------------------ */
static void bmi26x_dae_process_fifo_data_event(
    sns_sensor_instance *inst,
    sns_time            timestamp,
#if BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE
    sns_dae_timestamp_type ts_type,
#endif
    const uint8_t       *buf,
    size_t              buf_len)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *)inst->state->state;
    bmi26x_fifo_parse_ctx_t     ff_parse_ctx = {NULL, 0, 0, 0, NULL};
    uint16_t dummy_byte = 1;
    uint8_t regv_int_status[2] = {0};
    bmi26x_reg_int_ctx_t int_ctx;
    bmi26x_int_stat_flag_t int_st;

    if (istate->bus_is_spi) {
    } else {
        dummy_byte = 0;
    }

    {
        const uint8_t *data_buf = buf + dummy_byte;
        uint32_t dev_ts = 0;

        bmi26x_dev_parse_data_ts(data_buf, &dev_ts);

        // extract the dev timestamp for use it for frame ts calculation
        istate->dae_if.ts_dae_read_fifo_dev_time = dev_ts;
        istate->dae_if.ts_dae_read_fifo_sys_time = timestamp;
        istate->ts_irq = timestamp;
#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
        BMI26X_INST_LOG(LOW, istate->owner, "@DAE,parse out dev ts from dae:dev:%u int:%u",
                        (uint32_t) istate->dae_if.ts_dae_read_fifo_dev_time,
                        (uint32_t) istate->dae_if.ts_dae_read_fifo_sys_time);
#endif
    }

    /* 3 bytes sensor time + 1 byte event + dummy byte */
    ff_parse_ctx.ff_buf = buf + dummy_byte +
                          BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME;             //keep dummy byte in fifo buffer
    ff_parse_ctx.ff_buf_len = buf_len - dummy_byte -
                              BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME;     //keep dummy byte in fifo buffer

    istate->fifo_info.bh_info.interrupt_fired = SNS_DAE_TIMESTAMP_TYPE_UNKNOWN;

    if (ff_parse_ctx.ff_buf_len > 0) {
#if   BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE
        istate->fifo_info.th_info.interrupt_ts = timestamp;
        istate->fifo_info.bh_info.interrupt_fired = (ts_type == SNS_DAE_TIMESTAMP_TYPE_HW_IRQ);
#else
        istate->fifo_info.th_info.interrupt_ts = timestamp;
#endif
        istate->fifo_info.ff_read_ctx_type = BMI26X_FIFO_READ_CTX_TYPE_DAE;

        regv_int_status[1] = buf[dummy_byte + BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME - 1];
        regv_int_status[0] = buf[dummy_byte + BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME - 2];

        int_ctx.int_status_0 = regv_int_status[0];
        int_ctx.int_status_1 = regv_int_status[1];
        bmi26x_dev_parse_int_stat_flags(&int_ctx, &int_st);


#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE || 1
        BMI26X_INST_LOG(LOW, istate->owner, "@DAE,int st:<0x%x 0x%x> fifo len:%u, dev.ts:%u, 0x%02x",
                        regv_int_status[0],
                        regv_int_status[1],
                        ff_parse_ctx.ff_buf_len,
                        (uint32_t) istate->dae_if.ts_dae_read_fifo_dev_time,
                        (int_st.bits.ff_wml | (int_st.bits.ff_full << 1) | (int_st.bits.md << 2)));
#endif

        if (istate->int_en_flags_curr.bits.drdy.flag) {
#if BMI26X_CONFIG_ENABLE_DRI_MODE
            bmi26x_int_check_ctx_from_dae_t   dae_drdy_int = {.acc_gyro_flag = 0, .data_ptr = NULL};
            uint8_t flag = istate->int_en_flags_curr.bits.drdy.flag | istate->int_en_flags_curr.bits.fifo.flag;
            uint8_t regv_status = buf[dummy_byte + BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME - 1];
            dae_drdy_int.data_ptr = buf + dummy_byte + BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME + dummy_byte;

            BMI26X_INST_LOG(LOW, istate->owner, "int st:0x%x", regv_status);

            dae_drdy_int.timestamp = timestamp;
            if ((regv_status & B7_SET) && (flag & BMI26X_ACCEL)) {
                // acc
                dae_drdy_int.acc_gyro_flag |= BMI26X_ACCEL;
            }

            if ((regv_status & B6_SET) && (flag & BMI26X_GYRO)) {
                // gyro
                dae_drdy_int.acc_gyro_flag |= BMI26X_GYRO;
            }

            bmi26x_hal_handle_drdy_from_dae(inst, &dae_drdy_int);
#endif
        } else {
            if (int_st.bits.ff_full || int_st.bits.ff_wml ||
                    (ff_parse_ctx.ff_buf_len >= (BMI26X_READ_EXTRA_BYTES_BEFORE_DATA_FRAME + 1))) {
                if (int_st.bits.ff_full) {
                    /*bmi26x_hal_process_fifo_data_buffer(inst, ff_parse_ctx.ff_buf, ff_parse_ctx.ff_buf_len,
                                                BMI26X_INT_TRIGGER_SOURCE_DAE);*/
                    bmi26x_hal_handle_interrupt_fifo_full(istate);
                } else {
                    //bmi26x_hal_handle_interrupt_fifo_full(istate);
                    bmi26x_hal_process_fifo_data_buffer(inst, ff_parse_ctx.ff_buf, ff_parse_ctx.ff_buf_len,
                                                                    BMI26X_INT_TRIGGER_SOURCE_DAE);
                }

                // HB
                istate->fifo_info.ff_wml_int_handled = 1;

                if (istate->fac_test_in_progress) {
                    bmi26x_hal_inst_exit_island(inst);

                    bmi26x_process_fac_test(inst);
                }
            } else {
                // flush on request without ff int fired??
                BMI26X_INST_LOG(MED, istate->owner, "flush req? %d %d",
                        istate->ff_flush_client_req, istate->fifo_info.ff_flush_in_proc);
                if (istate->fifo_info.ff_flush_in_proc || istate->ff_flush_client_req) {
                    bmi26x_hal_process_fifo_data_buffer(inst, ff_parse_ctx.ff_buf, ff_parse_ctx.ff_buf_len,
                                                BMI26X_INT_TRIGGER_SOURCE_DAE);
                }
            }
        }

        // check the MD event within the DS INT
        if (istate->int_en_flags_curr.bits.md && !istate->fac_test_in_progress) {
            //if (BST_GET_VAL_BIT(regv_int_status[0], 6)) {
            if (int_st.bits.md) {
                bmi26x_hal_handle_interrupt_md(istate, timestamp);
            }
        }
#if BMI26X_CONFIG_ENABLE_LOWG
        if (istate->int_en_flags_curr.bits.lowg && !istate->fac_test_in_progress) {
            //if (BST_GET_VAL_BIT(regv_int_status[0], 2)) {
            if (int_st.bits.lowg) {
                bmi26x_hal_handle_interrupt_lowg(istate, timestamp);
            }
        }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
        if (istate->int_en_flags_curr.bits.dbtap && !istate->fac_test_in_progress) {
            if (int_st.bits.dbtap) {
                bmi26x_hal_handle_interrupt_dbtap(istate, timestamp);
            }
        }
#endif


#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
        if (istate->fifo_info.bh_info.interrupt_fired) {
            BMI26X_INST_LOG(LOW, inst, "@DAE, irq_ts: %u,%u",
                            (uint32_t)timestamp, (uint32_t)sns_get_system_time ());
        } else {
            BMI26X_INST_LOG(LOW, inst, "@DAE, ts: %u,%u",
                            (uint32_t)timestamp, (uint32_t)sns_get_system_time ());
        }
#endif
    }
}


/* ------------------------------------------------------------------------------------ */
static void bmi26x_dae_process_temp_samples(
    sns_sensor_instance *this,
    sns_time            timestamp,
    uint8_t             *buf,
    size_t              buf_len)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;
    uint8_t dumy_byte = 0;
    sns_time ts_est = timestamp;

    if (istate->bus_is_spi) {
        dumy_byte = 1;
    }

#if 0
    if (istate->sensor_temp_info.ts_last_frame) {
        float ts_delta_percentage = 1.0f;
        uint32_t ts_delta;
        ts_delta = timestamp - istate->sensor_temp_info.ts_last_frame;
        ts_delta_percentage = ts_delta / istate->sensor_temp_info.sampling_intvl_curr;
        if (ts_delta_percentage > 2.0) {
            ts_est = istate->sensor_temp_info.ts_last_frame + istate->sensor_temp_info.sampling_intvl_curr;
        } else if (ts_delta_percentage < 0.5) {
            //ts_est = istate->sensor_temp_info.ts_last_frame + (istate->sensor_temp_info.sampling_intvl_curr >> 1);
            ts_est = sns_get_system_time () - 1; //istate->sensor_temp_info.ts_last_frame + (istate->sensor_temp_info.sampling_intvl_curr >> 1);
        }
    }
#endif


    {
        const uint8_t * data_buf = buf + dumy_byte;
        if (buf_len >= 2) {
            bmi26x_hal_convert_and_send_temp_sample(this, ts_est, (uint8_t *)data_buf);
            istate->sensor_temp_info.ts_last_frame = ts_est;
        } else {
            BMI26X_INST_LOG(ERROR, this,
                            "@DAE ERROR!!! Unexpected data len %u from DAE sensor", buf_len);
        }
    }
}

/* ------------------------------------------------------------------------------------ */
static void bmi26x_dae_process_data_event(
    sns_sensor_instance   *this,
    bmi26x_dae_stream_t   *dae_stream,
    pb_istream_t          *pbstream)
{
    pb_buffer_arg decode_arg;
    sns_dae_data_event data_event = sns_dae_data_event_init_default;
    data_event.sensor_data.funcs.decode = &pb_decode_string_cb;
    data_event.sensor_data.arg = &decode_arg;

    if (pb_decode(pbstream, sns_dae_data_event_fields, &data_event)) {
        bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
        bmi26x_dae_if_info_t* dae_if = &istate->dae_if;

#if BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE
        sns_dae_timestamp_type ts_type = SNS_DAE_TIMESTAMP_TYPE_UNKNOWN;
#endif

        if (dae_stream == &dae_if->ag) {  //IMU
#if  BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE
            if (data_event.has_timestamp_type) {
                ts_type = data_event.timestamp_type;
            }
#else
#endif
            BMI26X_INST_LOG(LOW, this,
                            "@DAE, ts data_event @%u", (uint32_t)(data_event.timestamp));
            bmi26x_dae_process_fifo_data_event(this, data_event.timestamp,
#if BMI26X_CONFIG_ENABLE_DAE_TIMESTAMP_TYPE
                                            ts_type,
#endif
                                            (uint8_t*)decode_arg.buf,
                                            decode_arg.buf_len);
        } else {
            if (istate->fifo_info.publish_sensors & BMI26X_SENSOR_TEMP) {
                bmi26x_dae_process_temp_samples(this, data_event.timestamp,
                                            (uint8_t*)decode_arg.buf, decode_arg.buf_len);
            }
        }
    }
}

/* ------------------------------------------------------------------------------------ */
static void
bmi26x_dae_process_interrupt_event (sns_sensor_instance *this, bmi26x_dae_stream_t *dae_stream,
                                    pb_istream_t *pbstream)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*) this->state->state;

    if (dae_stream == &istate->dae_if.ag) {
        sns_dae_interrupt_event interrupt_event = sns_dae_interrupt_event_init_default;
        if (pb_decode (pbstream, sns_dae_interrupt_event_fields, &interrupt_event)) {
#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
            BMI26X_INST_LOG(LOW, this, "@DAE,interrupt_event: int:0x%x fac_test:%u, int.md_enable:%d",
                            interrupt_event.registers.bytes[0],
                            istate->fac_test_in_progress,
                            istate->md_info.enable_md_int);
#endif

            if (istate->int_en_flags_curr.bits.md && !istate->fac_test_in_progress) {
                if (BST_GET_VAL_BIT(interrupt_event.registers.bytes[0], 6)) {
                    bmi26x_hal_handle_interrupt_md(istate, interrupt_event.timestamp);
                }
            }
#if BMI26X_CONFIG_ENABLE_LOWG
            if (istate->int_en_flags_curr.bits.lowg && !istate->fac_test_in_progress) {
                if (BST_GET_VAL_BIT(interrupt_event.registers.bytes[0], 2)) {
                    bmi26x_hal_handle_interrupt_lowg(istate, interrupt_event.timestamp);
                }
            }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            if (1/*istate->int_en_flags_curr.bits.dbtap && !istate->fac_test_in_progress*/) {
                if (BST_GET_VAL_BIT(interrupt_event.registers.bytes[0], BMI26X_DOUBLE_TAP_BIT_POS_IN_INT_STATUS)) {
                    bmi26x_hal_handle_interrupt_dbtap(istate, interrupt_event.timestamp);
                }
            }
#endif

        } else {
            BMI26X_INST_LOG(ERROR, this, "@DAE ERROR!!! interrupt_event: decode fail");
        }
    } else {
        BMI26X_INST_LOG(ERROR, this, "@DAE ERROR!!! interrupt_event: Unexpected INT");
    }
}



/* ------------------------------------------------------------------------------------ */

static sns_rc bmi26x_dae_reconfig_hw(sns_sensor_instance *this, bmi26x_hw_cfg_ctx_t hw_cfg_ctx)
{
    sns_rc rc = SNS_RC_SUCCESS;
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;

    if ((istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_NONE) ||
            (istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_DEINIT)) {
        rc = bmi26x_hal_reconfig_hw(this, hw_cfg_ctx);
        if (rc != SNS_RC_SUCCESS) {
            BMI26X_INST_LOG(ERROR, this, "@DAE ERROR!!! reconfig hw error:%d", rc);
        } else {
            BMI26X_INST_LOG(LOW, this, "@DAE, reconfig hw");
        }
    }

    return rc;
}


bool bmi26x_dae_check_gpio_is_high(sns_sensor_instance *this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    uint8_t try_num = 0;
    bool pin_is_high = false;
    uint8_t need_fifo_flush_flag = 0;

    if (!bmi26x_dae_stream_usable(&istate->dae_if.ag)) {
        return false;
    }

    if (bmi26x_dae_pwr_available(istate) == false) {
        return false;
    }


    do {
        // double check the INT pin
        bool ff_int_st_available = false;
        uint8_t regv[2] = {0};
        regv[0] = 0;
        regv[1] = 0;
        pin_is_high = bmi26x_hal_int_pin_is_high(istate, regv);

        if (pin_is_high) {
            BMI26X_INST_LOG(MED, istate->owner, "@DAE, double checking int st, 0x%x 0x%x",
                            regv[0], regv[1]);

            if (bmi26x_hal_is_fifo_int_still_hangup(istate, regv[1])) {
                ff_int_st_available = true;
                if (istate->fifo_info.ff_flush_in_proc) {
                    BMI26X_INST_LOG(MED, istate->owner, "@DAE, flush in already in progress");
                } else {
                    need_fifo_flush_flag |= (1 << try_num);
                }
            }

            if (regv[0]) {
                // it is virtual sensor interrupt
                pin_is_high = bmi26x_hal_handle_int_latch(istate, regv[0]);

                if (ff_int_st_available == false) {
                    // is a virtual sensor interrupt, already handled above
                    pin_is_high = false;
                }
            }
        } else {
            break;
        }

        try_num ++;
    } while ((try_num < BMI26X_TRY_NUM_ON_CHECK_GPIO) && pin_is_high);


    if (need_fifo_flush_flag >= BMI26X_TRY_NUM_ON_CHECK_GPIO) {
        bool flush_hw_done = bmi26x_dae_if_flush_hw(istate->owner, &istate->dae_if.ag);
        // XXX should not reset the pin st due to FIFO need more action from DAE to pull down the INT pin
        if (!flush_hw_done) {
            // TODO
        } else {
            istate->fifo_info.ff_flush_in_proc = 1;
        }
        istate->dae_if.ag.fifo_int_pending = 1;
        BMI26X_INST_LOG(LOW, istate->owner, "@DAE, fifo int status still high, %d",
                istate->dae_if.ag.fifo_int_pending);
    }

    return pin_is_high;
}


static void bmi26x_dae_process_flush_data_event(sns_sensor_instance  *this,  bmi26x_dae_stream_t  *dae_stream,
                                          sns_dae_resp resp)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;

    if (dae_stream->state == BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ?
                            BMI26X_DAE_IF_STATE_STREAM_STARTING : BMI26X_DAE_IF_STATE_IDLE;
        if (NULL != istate->dae_if.ag.stream &&
            istate->dae_if.ag.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING &&
            NULL != istate->dae_if.temp.stream &&
            istate->dae_if.temp.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
            if (istate->config_step == BMI26X_CONFIG_STOPPING_STREAM) {
                if (bmi26x_dae_if_flush_hw(this, dae_stream)) {
                    istate->config_step = BMI26X_CONFIG_FLUSHING_HW;
                } else {
                    if (bmi26x_dae_pwr_available(istate)) {

                        if (bmi26x_dae_check_gpio_is_high(this)) {
                        } else {
                            istate->config_step = BMI26X_CONFIG_IDLE;
                            //istate->fifo_info.ff_flush_in_proc = 0;
                            //bmi26x_dae_reconfig_hw(this, HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS);
                            bmi26x_hal_cleanup_processing_fifo_flush(istate,
                                                                     BMI26X_FIFO_FLUSH_DONE_CTX_HW_CHANGE);
                        }
                    }
                }
            }
        }
    } else {
        uint8_t stream_flush_st = istate->fifo_info.ff_flush_pending_on_sensors;

        if (istate->ff_flush_client_req) {
            uint8_t flush_done = BMI26X_FIFO_FLUSH_DONE_DAE;
            // FIXME, should flush the base-on-request-sensor
            bmi26x_hal_flush_cleanup(istate, flush_done);
            istate->ff_flush_client_req = false;
        }
        dae_stream->flushing_data = false;

        if (dae_stream == &istate->dae_if.ag) {
            stream_flush_st = BST_CLR_VAL_BIT(stream_flush_st, 0);
        }

        if (dae_stream == &istate->dae_if.temp) {
            stream_flush_st = BST_CLR_VAL_BIT(stream_flush_st, 1);
        }

        istate->fifo_info.ff_flush_pending_on_sensors = stream_flush_st;

        BMI26X_INST_LOG(MED, this, "flush on sensor st:0x%x", stream_flush_st);

        if (istate->fifo_info.ff_flush_pending_on_sensors) {
            BMI26X_INST_LOG(MED, this, "still wait for flush finish");
            if (bmi26x_dae_pwr_available(istate)) {
                if (bmi26x_dae_check_gpio_is_high(this)) {
                }
            }
        } else {
            if (istate->fifo_info.ff_flush_in_proc) {
                istate->fifo_info.ff_flush_in_proc = 0;
            }

            // suppose there is no interrupt happened at this point
            if (bmi26x_dae_pwr_available(istate)) {

                if (bmi26x_dae_check_gpio_is_high(this)) {
                } else {
                    if (istate->hw_config_pending ||
                            (istate->config_step != BMI26X_CONFIG_IDLE)) {
                        if (bmi26x_dae_reconfig_hw(this, HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS) != SNS_RC_SUCCESS) {
                        }
                    }
                }
            }
            if (!istate->fifo_info.ff_flush_in_proc) {
                istate->config_step = BMI26X_CONFIG_IDLE;
            }
        }
    }
}


static void bmi26x_dae_process_response(
    sns_sensor_instance  *this,
    bmi26x_dae_stream_t  *dae_stream,
    pb_istream_t         *pbstream,
    bmi26x_dae_resp_context_t   *dae_resp_ctx
    )
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    sns_dae_resp resp = sns_dae_resp_init_default;
    sns_rc rc = SNS_RC_SUCCESS;

    if (pb_decode(pbstream, sns_dae_resp_fields, &resp)) {
#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
        BMI26X_INST_LOG(LOW, this,
                        "@DAE,process_response:: msg=%u err=%u step=%u",
                        resp.msg_id, resp.err, istate->config_step);

        BMI26X_INST_LOG(LOW, this, "@DAE, resp <%u %u %u %u>, irq_ready:%u",
                        dae_stream->state,
                        istate->dae_if.ag.state,
                        istate->dae_if.temp.state,
                        istate->config_step,
                        istate->irq_ready
                       );
#else
        BMI26X_INST_LOG(MED, this, "resp msg.id:%d, err:%d, cfg.step:%d",
                        resp.msg_id, resp.err, istate->config_step);
#endif

        switch (resp.msg_id) {
            case SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG:
                if (SNS_STD_ERROR_NO_ERROR == resp.err) {
                    bool enable_fifo_stream = istate->fifo_info.publish_sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MAG);
                    enable_fifo_stream |= istate->fac_test_in_progress;

                    BMI26X_INST_LOG(MED, this, "@DAE,irq_ready resp id=%d", resp.msg_id);
                    istate->irq_ready = true;
                    // sometimes the response has some latency after the hw config, so need a check @here to handle the pending config, for example the
                    // INT output configuration
                    if (bmi26x_dae_pwr_available(istate)) {
                        if (enable_fifo_stream
                            || istate->int_en_flags_req.bits.md) {
                            rc |= bmi26x_hal_config_int_output(this, true);
                            if (rc != SNS_RC_SUCCESS) {
                            }
                        }
                    }
                } else {
                    // DAE sensor does not have support for this driver
                    dae_stream->stream_usable = false;
                }
                break;

            case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
                break;

            case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
                if (dae_stream->stream != NULL && dae_stream->state == BMI26X_DAE_IF_STATE_STREAM_STARTING) {
                    dae_stream->state = (SNS_STD_ERROR_NO_ERROR == resp.err) ? BMI26X_DAE_IF_STATE_STREAMING :
                                        BMI26X_DAE_IF_STATE_IDLE;
                }
                // XXX check the int status
                if (bmi26x_dae_check_gpio_is_high(this)) {
                }
                break;

            case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
                dae_stream->flushing_hw = false;
                istate->fifo_info.th_info.flush_req = false;

                BMI26X_INST_LOG(LOW, this, "@DAE, flush hw resp.msg:<%d %d %d>",
                                istate->fifo_info.ff_flush_in_proc,
                                istate->ff_flush_client_req,
                                istate->dae_if.ag.fifo_int_pending);

                bool flush_event_done = bmi26x_dae_flush_samples(this, dae_stream);
                if (!flush_event_done) {
                    // TODO
                }
                break;

            case SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS:
#if 0
                if (dae_stream->state == BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
                    dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ?
                                        BMI26X_DAE_IF_STATE_STREAM_STARTING : BMI26X_DAE_IF_STATE_IDLE;
                    if (NULL != istate->dae_if.ag.stream &&
                        istate->dae_if.ag.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING &&
                        NULL != istate->dae_if.temp.stream &&
                        istate->dae_if.temp.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
                        if (istate->config_step == BMI26X_CONFIG_STOPPING_STREAM) {
                            if (bmi26x_dae_if_flush_hw(this, dae_stream)) {
                                istate->config_step = BMI26X_CONFIG_FLUSHING_HW;
                            } else {
                                if (bmi26x_dae_pwr_available(istate)) {

                                    if (bmi26x_dae_check_gpio_is_high(istate)) {
                                    } else {
                                        istate->config_step = BMI26X_CONFIG_IDLE;
                                        //istate->fifo_info.ff_flush_in_proc = 0;
                                        //bmi26x_dae_reconfig_hw(this, HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS);
                                        bmi26x_hal_cleanup_processing_fifo_flush(istate,
                                                                                 BMI26X_FIFO_FLUSH_DONE_CTX_HW_CHANGE);
                                    }
                                }
                            }
                        }
                    }
                } else {
                    uint8_t stream_flush_st = istate->fifo_info.ff_flush_pending_on_sensors;

                    if (istate->ff_flush_client_req) {
                        uint8_t flush_done = BMI26X_FIFO_FLUSH_DONE_DAE;
                        // FIXME, should flush the base-on-request-sensor
                        bmi26x_hal_flush_cleanup(istate, flush_done);
                        istate->ff_flush_client_req = false;
                    }
                    dae_stream->flushing_data = false;

                    if (dae_stream == &istate->dae_if.ag) {
                        stream_flush_st = BST_CLR_VAL_BIT(stream_flush_st, 0);
                    }

                    if (dae_stream == &istate->dae_if.temp) {
                        stream_flush_st = BST_CLR_VAL_BIT(stream_flush_st, 1);
                    }

                    istate->fifo_info.ff_flush_pending_on_sensors = stream_flush_st;

                    BMI26X_INST_LOG(MED, this, "flush on sensor st:0x%x", stream_flush_st);

                    if (istate->fifo_info.ff_flush_pending_on_sensors) {
                        BMI26X_INST_LOG(MED, this, "still wait for flush finish");
                        if (bmi26x_dae_pwr_available(istate)) {
                            if (bmi26x_dae_check_gpio_is_high(istate)) {
                            }
                        }
                    } else {
                        if (istate->fifo_info.ff_flush_in_proc) {
                            istate->fifo_info.ff_flush_in_proc = 0;
                        }

                        // suppose there is no interrupt happened at this point
                        if (bmi26x_dae_pwr_available(istate)) {

                            if (bmi26x_dae_check_gpio_is_high(this)) {
                            } else {
                                if (istate->hw_config_pending ||
                                        (istate->config_step != BMI26X_CONFIG_IDLE)) {
                                    bmi26x_dae_reconfig_hw(this, HW_CONFIG_CTX_ON_DAE_FLUSH_DATA_EVENTS);
                                }
                            }
                        }
                        if (!istate->fifo_info.ff_flush_in_proc) {
                            istate->config_step = BMI26X_CONFIG_IDLE;
                        }
                    }
                }
#endif
                //bmi26x_dae_process_flush_data_event(this, dae_stream, resp);

                if (dae_stream == &istate->dae_if.ag) {
                    dae_resp_ctx->ag_stream = dae_stream;
                    dae_resp_ctx->ag_resp = resp;
                }

                if (dae_stream == &istate->dae_if.temp) {
                    dae_resp_ctx->temp_stream = dae_stream;
                    dae_resp_ctx->temp_resp = resp;
                }

                break;

            case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
                if (dae_stream->state == BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
                    dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ?
                                        BMI26X_DAE_IF_STATE_STREAM_STARTING : BMI26X_DAE_IF_STATE_IDLE;
                }

                if (NULL != istate->dae_if.ag.stream &&
                        istate->dae_if.ag.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING &&
                        NULL != istate->dae_if.temp.stream &&
                        istate->dae_if.temp.state != BMI26X_DAE_IF_STATE_STREAM_STOPPING) {
                    if (istate->config_step == BMI26X_CONFIG_STOPPING_STREAM) {
                        if (bmi26x_dae_if_flush_hw(this, dae_stream)) {
                            istate->config_step = BMI26X_CONFIG_FLUSHING_HW;
                        } else {
                            if (bmi26x_dae_pwr_available(istate)) {

                                if (bmi26x_dae_check_gpio_is_high(this)) {
                                } else {
                                    istate->config_step = BMI26X_CONFIG_IDLE;
                                    istate->fifo_info.ff_flush_in_proc = 0;
                                    rc = bmi26x_dae_reconfig_hw(this, HW_CONFIG_CTX_ON_DAE_PAUSE_SAMPLING);
                                    if (rc != SNS_RC_SUCCESS) {
                                    }
                                }
                            }
                        }
                    }
                }
                break;

            case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
                break;

            case SNS_DAE_MSGID_SNS_DAE_RESP:
            case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
                break; /* unexpected */
        }
    }

#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
    BMI26X_INST_LOG(LOW, this, "@DAE,<%u %u %u %u>, irq:%u rc:%d",
                    dae_stream->state,
                    istate->dae_if.ag.state,
                    istate->dae_if.temp.state,
                    istate->config_step,
                    istate->irq_ready,
                    rc);
#endif
}

/* ------------------------------------------------------------------------------------ */


static void bmi26x_dae_process_events(sns_sensor_instance *this, bmi26x_dae_stream_t *dae_stream)
{
    sns_sensor_event *event;
    bool data_received = false;
    bool event_received = false;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;
    bmi26x_dae_resp_context_t dae_resp_ctx;

#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
    BMI26X_INST_LOG(LOW, this, "@DAE, proess event, stream:%p, stream.usable:%d",
                    dae_stream->stream, dae_stream->stream_usable);
    BMI26X_INST_LOG(LOW, this, "@DAE, fifo flush state:%d", istate->fifo_info.ff_flush_in_proc);
#endif

    dae_resp_ctx.ag_stream = NULL;
    dae_resp_ctx.temp_stream = NULL;

    memset(&dae_resp_ctx.ag_resp, 0, sizeof(sns_dae_resp));
    memset(&dae_resp_ctx.temp_resp, 0, sizeof(sns_dae_resp));

    while (NULL != dae_stream->stream &&
            NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream))) {
        if (dae_stream->stream_usable) {
            pb_istream_t pbstream =
                pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE || 1
            BMI26X_INST_LOG(MED, this, "@DAE, p/evt msg_id:%d @%u",
                            event->message_id, (uint32_t)event->timestamp);
#endif

            if (SNS_DAE_MSGID_SNS_DAE_DATA_EVENT == event->message_id) {
                istate->fifo_info.ff_wml_int_handled = 0;
                bmi26x_dae_process_data_event(this, dae_stream, &pbstream);
                data_received = true;
            } else if (SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id) {
                bmi26x_dae_process_interrupt_event(this, dae_stream, &pbstream);
                event_received = true;
            } else if (SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id) {
                bmi26x_dae_process_response(this, dae_stream, &pbstream, &dae_resp_ctx);
            } else if (SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id) {
#if BMI26X_CONFIG_ENABLE_DEBUG_DETAIL_DAE
                BMI26X_INST_LOG(LOW, this,
                                "@DAE,process_events:: ERROR_EVENT stream=%x state=%u",
                                dae_stream->stream, dae_stream->state);
#endif
                dae_stream->stream_usable = false;
                bmi26x_hal_register_interrupt(this);
            } else {
                BMI26X_INST_LOG(ERROR, this,
                                "@DAE ERROR!!! Unexpected message id %u", event->message_id);
            }
        }
        event = dae_stream->stream->api->get_next_input(dae_stream->stream);
    }

    if (dae_resp_ctx.ag_stream != NULL) {
        data_received = true;
        bmi26x_dae_process_flush_data_event(this, dae_resp_ctx.ag_stream, dae_resp_ctx.ag_resp);
    }

    if (dae_resp_ctx.temp_stream != NULL) {
        bmi26x_dae_process_flush_data_event(this, dae_resp_ctx.temp_stream, dae_resp_ctx.temp_resp);
    }

    if (data_received &&
            istate->ff_flush_client_req &&
            !dae_stream->flushing_data) {
        uint8_t flush_done = BMI26X_FIFO_FLUSH_DONE_DAE;
        bmi26x_hal_flush_cleanup(istate, flush_done);
        istate->ff_flush_client_req = 0;
    }

    if ((data_received) || (event_received) || (istate->int_en_flags_curr.bits.lowg ||
            istate->int_en_flags_curr.bits.md ||
            istate->int_en_flags_curr.bits.dbtap)) {
        // INT missing possible
        if (bmi26x_dae_check_gpio_is_high(this)) {
        }
    }

    if (istate->fifo_info.ff_wml_int_handled) {
#if BMI26X_CONFIG_ENABLE_HEART_BEAT_TIMER
        istate->hb_cfg_info.ts_data_event = sns_get_system_time();
#endif
        istate->fifo_info.ff_wml_int_handled = 0;
    }
}


/*======================================================================================
  Public Functions
  ======================================================================================*/

void bmi26x_dae_if_check_support(sns_sensor *this)
{
    bmi26x_state *sstate = NULL;
    sns_rc rc = SNS_RC_SUCCESS;
    sns_sensor_uid  dae_suid;

    BMI26X_SENSOR_LOG(LOW, this, "<dae_if_ check support> from ss: %d", ((bmi26x_state *) this->state->state)->sensor);

    sstate = bmi26x_inst_get_master_sensor_state(this->state);

    if (NULL == sstate) {
        BMI26X_SENSOR_LOG(ERROR, this, "@DAE, WARNING!!! dae::can't find master sensor");
        return ;
    }

    if (!sns_suid_lookup_get(&sstate->common.suid_lookup_data, "data_acquisition_engine",
                             &dae_suid)) {
        BMI26X_SENSOR_LOG(ERROR, this, "@DAE, WARNING!!! dae::DAE not supported");
        return ;
    } else {
        BMI26X_SENSOR_LOG(MED, this, "@DAE,find DAE, suid <0x%x 0x%x 0x%x 0x%x>", dae_suid.sensor_uid[0],
                          dae_suid.sensor_uid[1],
                          dae_suid.sensor_uid[2],
                          dae_suid.sensor_uid[3]);
    }

    BMI26X_SENSOR_LOG(MED, this, "@DAE,check dae supported@ss:%d", sstate->sensor);

    if (NULL == sstate->common.dae_stream) {
        sns_service_manager *service_mgr = this->cb->get_service_manager(this);
        sns_stream_service *stream_svc = (sns_stream_service*)
                                         service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

        rc = stream_svc->api->create_sensor_stream(stream_svc, this,
                dae_suid,
                &sstate->common.dae_stream);
        BMI26X_SENSOR_LOG(MED, this, "@DAE,check_support: creating stream:%p, rc:%d",
                sstate->common.dae_stream, rc);
    }

    if (NULL != sstate->common.dae_stream) {
        sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
        if (sstate->common.dae_ag_state == BMI26X_DAE_IF_PRE_INIT) {
            sstate->common.dae_ag_state = BMI26X_DAE_IF_INIT_PENDING;
            bmi26x_dae_build_static_config_request(sstate, NULL, &config_req, true);
        } else {
            sstate->common.dae_temper_state =  BMI26X_DAE_IF_INIT_PENDING;
            bmi26x_dae_build_static_config_request(sstate, NULL, &config_req, false);
        }

        rc =  bmi26x_dae_send_static_config_request(sstate->common.dae_stream, &config_req);
        if (SNS_RC_SUCCESS != rc) {
            BMI26X_SENSOR_LOG(ERROR, this, "@DAE WARNING!!! send static config req failure:%d", rc);
            sstate->common.dae_ag_state     =  BMI26X_DAE_IF_UNAVAILABLE;
            sstate->common.dae_temper_state =  BMI26X_DAE_IF_UNAVAILABLE;
            sns_sensor_util_remove_sensor_stream(this, &sstate->common.dae_stream);
        }
    }

    BMI26X_SENSOR_LOG(MED, this, "@DAE,st.ag%d, st.temp",
                      sstate->common.dae_ag_state,
                      sstate->common.dae_temper_state);
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_process_sensor_events(sns_sensor *this)
{
    bmi26x_state *sstate = NULL; //(bmi26x_state *) this->state->state;
    sns_data_stream *stream = NULL ;//sstate->common.dae_stream;
    sns_sensor_event *event;

    sstate = bmi26x_inst_get_master_sensor_state(this->state);
    if (NULL == sstate) {
        BMI26X_SENSOR_LOG(ERROR, this, "@DAE ERROR!!! can't find master sensor");
        return ;
    }

    stream = sstate->common.dae_stream;

    if (NULL == stream || 0 == stream->api->get_input_cnt(stream)) {
        BMI26X_SENSOR_LOG(MED, this, "@DAE stream:%p", stream);
        return;
    }

    while (NULL != (event = stream->api->peek_input(stream))) {
        pb_istream_t pbstream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

        BMI26X_SENSOR_LOG(MED, this, "@DAE, dae_sensor_events: msg=%u state(ag/t): <%d %d>",
                          event->message_id, sstate->common.dae_ag_state,
                          sstate->common.dae_temper_state);

        if (SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id) {
            sns_dae_resp resp = sns_dae_resp_init_default;

            if (pb_decode(&pbstream, sns_dae_resp_fields, &resp)) {

                BMI26X_SENSOR_LOG(MED, this, "@DAE, reps.msg:%d", resp.msg_id);

                if (SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG == resp.msg_id) {
                    if (sstate->common.dae_ag_state == BMI26X_DAE_IF_INIT_PENDING) {
                        sstate->common.dae_ag_state = (SNS_STD_ERROR_NO_ERROR != resp.err) ?
                                                      BMI26X_DAE_IF_UNAVAILABLE : BMI26X_DAE_IF_STATE_IDLE;

                        if (sstate->common.dae_ag_state == BMI26X_DAE_IF_STATE_IDLE) {
                            bmi26x_dae_if_check_support(this); // for temperature
                        }
                    } else {     // must be for temperature
                        sstate->common.dae_temper_state =
                            (SNS_STD_ERROR_NO_ERROR != resp.err) ? BMI26X_DAE_IF_UNAVAILABLE :
                            BMI26X_DAE_IF_STATE_IDLE;
                    }
                }
            }
        } else if (SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id) {
            sstate->common.dae_ag_state =  BMI26X_DAE_IF_UNAVAILABLE;
        }

        event = stream->api->get_next_input(stream);
    }

    if (sstate->common.dae_ag_state ==  BMI26X_DAE_IF_UNAVAILABLE |
            BMI26X_DAE_IF_STATE_IDLE == sstate->common.dae_temper_state) {
        sns_sensor_util_remove_sensor_stream(this, &sstate->common.dae_stream);
    }

    BMI26X_SENSOR_LOG(HIGH, this, "@DAE, state(ag/t): %u/%u",
                      sstate->common.dae_ag_state, sstate->common.dae_temper_state);
}


bool bmi26x_dae_if_available(sns_sensor_instance *this)
{
    /* both streams must be availble */
    BMI26X_INST_LOG(LOW, this, "<dae_if_ available>");
    bmi26x_dae_if_info_t *dae_if = &((bmi26x_instance_state*)this->state->state)->dae_if;
    return (bmi26x_dae_stream_usable(&dae_if->ag) && bmi26x_dae_stream_usable(&dae_if->temp));
}

/* ------------------------------------------------------------------------------------ */
sns_rc bmi26x_dae_if_init(
    sns_sensor *const ssensor,
    sns_sensor_instance           *const this,
    struct sns_stream_service     *stream_mgr,
    sns_sensor_uid                *dae_suid)
{
    bmi26x_state *sstate = NULL;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) this->state->state;
    sns_rc rc = SNS_RC_NOT_AVAILABLE;
    bmi26x_dae_if_info_t* dae_if = &istate->dae_if;

    BMI26X_INST_LOG(LOW, this, "<dae_if_ init> from ss:%d", ((bmi26x_state *)this->state->state)->sensor);

    sstate = bmi26x_inst_get_master_sensor_state(ssensor->state);
    if (NULL == sstate) {
        BMI26X_INST_LOG(ERROR, this, "@DAE ERROR!!! can't find master sensor");
        return SNS_RC_INVALID_TYPE;
    } else {
        BMI26X_INST_LOG(LOW, this, "@DAE, find master sensor:0x%x @%d", sstate, sstate->sensor);
    }

    dae_if->ag.state = sstate->common.dae_ag_state;
    dae_if->temp.state = sstate->common.dae_temper_state;

    if (dae_if->ag.state == BMI26X_DAE_IF_STATE_IDLE &&
            dae_if->temp.state == BMI26X_DAE_IF_STATE_IDLE) {
        bool dae_avail = false;

        rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this, *dae_suid, &dae_if->ag.stream);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        rc = stream_mgr->api->create_sensor_instance_stream(stream_mgr, this, *dae_suid, &dae_if->temp.stream);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);

        if (NULL != dae_if->ag.stream && NULL != dae_if->temp.stream) {
            sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
            dae_if->ag.stream_usable = true;
            dae_if->temp.stream_usable = true;

            // build ag
            bmi26x_dae_build_static_config_request(sstate, istate, &config_req, true);
            rc = bmi26x_dae_send_static_config_request(dae_if->ag.stream, &config_req);

            if (SNS_RC_SUCCESS == rc) {
                //build temperature
                bmi26x_dae_build_static_config_request(sstate, istate, &config_req, false);
                BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                rc = bmi26x_dae_send_static_config_request(dae_if->temp.stream, &config_req);
                BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
                dae_avail = true;
            }
        }

        if (!dae_avail) {
            bmi26x_dae_if_deinit(this);
        }
    }

    BMI26X_INST_LOG(LOW, this, "@DAE, <dae_if_init st(ag/t): %d %d, usable(ag/t): %d %d, rc:%d>",
                    dae_if->ag.state, dae_if->temp.state,
                    dae_if->ag.stream_usable, dae_if->temp.stream_usable,
                    rc);

    return rc;
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_deinit(sns_sensor_instance *const this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;

    BMI26X_INST_LOG(LOW, this, "<dae_if_ deinit>");

    if (istate->dae_if.ag.stream) {
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->dae_if.ag.stream);
        istate->dae_if.ag.stream = NULL;
    }

    if (istate->dae_if.temp.stream) {
        sns_sensor_util_remove_sensor_instance_stream(this, &istate->dae_if.temp.stream);
        istate->dae_if.temp.stream = NULL;
    }
    istate->dae_if.ag.flushing_hw        = false;
    istate->dae_if.ag.flushing_data      = false;
    istate->dae_if.ag.state              = BMI26X_DAE_IF_PRE_INIT;
    istate->dae_if.temp.flushing_hw      = false;
    istate->dae_if.temp.flushing_data    = false;
    istate->dae_if.temp.state            = BMI26X_DAE_IF_PRE_INIT;
}

/* ------------------------------------------------------------------------------------ */
bool bmi26x_dae_if_stop_streaming(sns_sensor_instance *const this, uint8_t sensors)
{
    bool cmd_sent = false;
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    bmi26x_dae_if_info_t    *dae_if = &istate->dae_if;

    if (!bmi26x_dae_stream_usable(&istate->dae_if.ag)) {
        return false;
    }

#if BMI26X_CONFIG_ENABLE_FAST_CONFIG_FIFO_IN_DAE_MODE && 0
    uint8_t regv;
    if ((istate->fifo_info.ff_sensors_en_curr ^ istate->fifo_info.ff_sensors_en_req) &&
            (istate->fifo_info.ff_master_sensors_curr)) {
        sns_rc rc = SNS_RC_SUCCESS;

        rc = bmi26x_sbus_read_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_1, &regv, 1);
        if (rc != SNS_RC_SUCCESS) {
            return false;
        }
        if (((istate->fifo_info.ff_sensors_en_req & BMI26X_ACCEL) == 0) &&
                        (istate->fifo_info.ff_sensors_en_curr & BMI26X_ACCEL)) {
            // acc enabled at present but will disabled as request
            regv = BST_CLR_VAL_BIT(regv, 6);
        }

        if (((istate->fifo_info.ff_sensors_en_req & BMI26X_GYRO) == 0) &&
                        (istate->fifo_info.ff_sensors_en_curr & BMI26X_GYRO)) {
            // gyr enabled at present but will disabled as request
            regv = BST_CLR_VAL_BIT(regv, 7);
        }

        rc = bmi26x_sbus_write_wrapper(istate, BMI26X_REGA_USR_FIFO_CONFIG_1, &regv, 1);
        if (rc != SNS_RC_SUCCESS) {
            return false;
        }
    }
#endif

    if ((sensors & (BMI26X_ACCEL | BMI26X_GYRO | BMI26X_MOTION_DETECT)) &&
            bmi26x_dae_stream_usable(&istate->dae_if.ag) &&
            (dae_if->ag.state == BMI26X_DAE_IF_STATE_STREAMING ||
             dae_if->ag.state == BMI26X_DAE_IF_STATE_STREAM_STARTING)) {
        BMI26X_INST_LOG(MED, this,
                        "@DAE, stop_streaming AG stream req on stream: %p send/:%d",
                        &dae_if->ag.stream, SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING);
        cmd_sent |= bmi26x_dae_stop_streaming(&dae_if->ag);
    }

    if ((sensors & BMI26X_SENSOR_TEMP) &&
            bmi26x_dae_stream_usable(&istate->dae_if.temp) &&
            (dae_if->temp.state == BMI26X_DAE_IF_STATE_STREAMING ||
             dae_if->temp.state == BMI26X_DAE_IF_STATE_STREAM_STARTING)) {
        BMI26X_INST_LOG(MED, this,
                        "@DAE, stop_streaming Temp stream req on stream: %p send/:%d",
                        &dae_if->temp.stream,
                        SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING);
        cmd_sent |= bmi26x_dae_stop_streaming(&dae_if->temp);
    }

    return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool bmi26x_dae_if_start_streaming(sns_sensor_instance *this)
{
    bool cmd_sent = false;
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    bmi26x_dae_if_info_t  *dae_if = &istate->dae_if;

    BMI26X_INST_LOG(LOW, this,
                    "@DAE, start_streaming publish ss: 0x%x tmp_intvl: %u",
                    istate->fifo_info.publish_sensors,
                    (uint32_t)istate->sensor_temp_info.sampling_intvl_curr);

    if (bmi26x_dae_stream_usable(&dae_if->ag) && dae_if->ag.state >= BMI26X_DAE_IF_STATE_IDLE &&
            ((0 < istate->accel_info.odr_curr || 0 < istate->gyro_info.odr_curr || istate->int_en_flags_req.bits.md)
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
                    || (istate->int_en_flags_req.bits.dbtap)
#endif
                    )) {
        BMI26X_INST_LOG(LOW, this,
                        "@DAE, start_streaming AG req on stream: %p", &dae_if->ag.stream);
        cmd_sent |= bmi26x_dae_send_ag_config(&dae_if->ag, istate);
    }

    if (bmi26x_dae_stream_usable(&dae_if->temp) && dae_if->temp.state != BMI26X_DAE_IF_STATE_STREAMING &&
            0 < istate->sensor_temp_info.sampling_intvl_curr) {
        BMI26X_INST_LOG(LOW, this,
                        "@DAE start_streaming Temp req on stream: %p", &dae_if->temp.stream);
        cmd_sent |= bmi26x_dae_send_temp_config(&dae_if->temp, this);
    }

    istate->config_step = BMI26X_CONFIG_IDLE;

    return cmd_sent;
}


/* ------------------------------------------------------------------------------------ */
bool bmi26x_dae_if_flush_hw(sns_sensor_instance *this, bmi26x_dae_stream_t  *dae_stream)
{
    bmi26x_dae_if_info_t *dae_if = &((bmi26x_instance_state*)this->state->state)->dae_if;
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;
    bool cmd_sent = false; //dae_if->ag.flushing_hw;

    if (dae_stream == NULL) {
        BMI26X_INST_LOG(HIGH, this, "WARN!!! dae stream is NULL");
        return false;
    }

    BMI26X_INST_LOG(LOW, this,
                    "<dae_if_ flush hw> ag: <%u %u %u %u>",
                    dae_if->ag.stream_usable,
                    bmi26x_dae_stream_usable(&dae_if->ag),
                    dae_if->ag.state,
                    dae_if->ag.flushing_hw);

    if (dae_stream == &istate->dae_if.ag) {
       cmd_sent = dae_if->ag.flushing_hw;
        if (bmi26x_dae_stream_usable(&dae_if->ag)
            && dae_if->ag.state > BMI26X_DAE_IF_PRE_INIT
            && !dae_if->ag.flushing_hw) {
            cmd_sent |= bmi26x_dae_flush_hw(&dae_if->ag);
            BMI26X_INST_LOG(MED, this,
                        "@DAE,flush_hw AG stream: %p flushing st: %u, send/:%d",
                        &dae_if->ag.stream, dae_if->ag.flushing_hw, SNS_DAE_MSGID_SNS_DAE_FLUSH_HW);
        }
    } else {
        cmd_sent = dae_if->temp.flushing_hw;
        if (bmi26x_dae_stream_usable(&dae_if->temp)
                && dae_if->temp.state > BMI26X_DAE_IF_PRE_INIT
                && !dae_if->temp.flushing_hw) {
            cmd_sent |= bmi26x_dae_flush_hw(&dae_if->temp);
            BMI26X_INST_LOG(MED, this,
                            "@DAE,flush_hw AG stream: %p flushing st: %u, send/:%d",
                            &dae_if->temp.stream, dae_if->temp.flushing_hw, SNS_DAE_MSGID_SNS_DAE_FLUSH_HW);
        }
    }

    return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_process_events(sns_sensor_instance *const this)
{
    bmi26x_instance_state *istate = (bmi26x_instance_state*)this->state->state;

    BMI26X_INST_LOG(LOW, this, "<dae_if_ process events> ag/temp st:%d; %d>, cfg.step:%d",
                    istate->dae_if.ag.state, istate->dae_if.temp.state, istate->config_step);

    bmi26x_dae_process_events(this, &istate->dae_if.ag);
    bmi26x_dae_process_events(this, &istate->dae_if.temp);

    // CHECK the GPIO status
    //bmi26x_dae_check_gpio_is_high(this);

    if (!bmi26x_dae_stream_usable(&istate->dae_if.ag) || !bmi26x_dae_stream_usable(&istate->dae_if.temp)) {
        /* both streams are needed; if one was removed, remove the other one too */
        bmi26x_dae_if_deinit(this);
    }
}

#else

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_check_support(sns_sensor *this)
{
    UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
bool bmi26x_dae_if_available(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
    return false;
}

/* ------------------------------------------------------------------------------------ */
bool bmi26x_dae_if_flush_hw(sns_sensor_instance *this, bmi26x_dae_stream_t  *dae_stream)
{
    UNUSED_VAR(this);
    UNUSED_VAR(dae_stream);
    return false;
}

/* ------------------------------------------------------------------------------------ */
sns_rc bmi26x_dae_if_init(
                sns_sensor *const ssensor,
                sns_sensor_instance           *const this,
                struct sns_stream_service     *stream_mgr,
                sns_sensor_uid                *dae_suid)
{
    UNUSED_VAR(ssensor);
    UNUSED_VAR(this);
    UNUSED_VAR(stream_mgr);
    UNUSED_VAR(dae_suid);
    return false;
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_deinit(sns_sensor_instance *const this)
{
    UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_process_events(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
}

/* ------------------------------------------------------------------------------------ */
void bmi26x_dae_if_process_sensor_events(sns_sensor *this)
{
    UNUSED_VAR(this);
}

#endif
