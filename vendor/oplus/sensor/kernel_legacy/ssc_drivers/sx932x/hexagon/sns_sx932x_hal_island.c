/*******************************************************************************
 * Copyright (c) 2017, Semtech
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Semtech nor the
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
#include "sns_gpio_service.h"
#include "sns_types.h"
#include "sns_async_com_port.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_std_sensor.pb.h"
#include "sns_diag_service.h"
#include "sns_std.pb.h"
#include "sns_diag.pb.h"
#include "sns_sar.pb.h"

#include "sns_sx932x_hal.h"
#include "sns_sx932x_sensor.h"
#include "sns_sx932x_sensor_instance.h"
#include "oppo_sensor.h"
#include "sns_printf_int.h"

#ifdef SX932X_GET_PARAMETER_FROM_SMEM
struct sensor_hw *sar_hw = NULL;
struct sensor_feature *sar_feature = NULL;
#endif

const REGISTER_SETTING g_SX932xConfigurationSettings[] =
{
    //General control
    {SX932x_CTRL0_REG, 	(0x16)},

    //Analog front end Control
    {SX932x_AFE_CTRL0_REG, (0x80)},//0x80
    {SX932x_AFE_CTRL3_REG, (0x00)},
    {SX932x_AFE_CTRL4_REG, (0xAF)},
    {SX932x_AFE_CTRL6_REG, (0x00)},
    {SX932x_AFE_CTRL7_REG,	(0x2F)},
    {SX932x_AFE_PH0_REG,	(0x39)},
    {SX932x_AFE_PH1_REG,	(0x3F)},
    {SX932x_AFE_PH2_REG,	(0x1B)},
    {SX932x_AFE_PH3_REG,	(0x00)},
    {0x2c,                    0x12},
    {0x2d,                    0x06},

    //Main digital Prox control
    {SX932x_PROX_CTRL0_REG,(0x13)},
    {SX932x_PROX_CTRL1_REG,(0x12)},
    {SX932x_PROX_CTRL2_REG,(0x20)},
    {SX932x_PROX_CTRL3_REG,(0x20)},
    {SX932x_PROX_CTRL4_REG,(0x0C)},
    {SX932x_PROX_CTRL5_REG,(0x15)},
    {SX932x_PROX_CTRL6_REG,(0x19)},
    {SX932x_PROX_CTRL7_REG,(0x20)},

    //Advanced digital control
    { SX932x_ADV_CTRL0_REG,	(0x00) },
    { SX932x_ADV_CTRL1_REG,	(0x00) },
    { SX932x_ADV_CTRL2_REG,	(0x00) },
    { SX932x_ADV_CTRL3_REG,	(0x00) },
    { SX932x_ADV_CTRL4_REG,	(0x00) },
    { SX932x_ADV_CTRL5_REG,	(0x05) },
    { SX932x_ADV_CTRL6_REG,	(0x00) },
    { SX932x_ADV_CTRL7_REG,	(0x00) },
    { SX932x_ADV_CTRL8_REG,	(0x00) },
    { SX932x_ADV_CTRL9_REG,	(0x00) },
    { SX932x_ADV_CTRL10_REG,(0x44) },
    { SX932x_ADV_CTRL11_REG,(0x00) },
    { SX932x_ADV_CTRL12_REG,(0x00) },
    { SX932x_ADV_CTRL13_REG,(0x50) },
    { SX932x_ADV_CTRL14_REG,(0x9B) },
    { SX932x_ADV_CTRL15_REG,(0x0D) },
    { SX932x_ADV_CTRL16_REG,(0x10) },//14
    { SX932x_ADV_CTRL17_REG,(0x70) },
    { SX932x_ADV_CTRL18_REG,(0x00) },//20
    { SX932x_ADV_CTRL19_REG,(0x00) },
    { SX932x_ADV_CTRL20_REG,(0x00) },

    //Phase enable
    { SX932x_STAT1_REG,      0x00},
    { SX932x_STAT2_REG,      0x00},
    { SX932x_IRQ_ENABLE_REG, 0x60},
    { SX932x_IRQCFG0_REG,    0x00},
    { SX932x_IRQCFG1_REG,    0x80},
    { 0x08,                  0x01},
    { SX932x_CTRL1_REG,    (0x24)}
};

const uint32_t g_SX932xRegisterSize = (sizeof (g_SX932xConfigurationSettings) / sizeof(REGISTER_SETTING));

sns_rc sx932x_encode_log_sensor_state_raw(void *log,
                                                    size_t log_size,
                                                    size_t encoded_log_size,
                                                    void *encoded_log,
                                                    size_t *bytes_written
                                                   )
{
    sns_rc rc = SNS_RC_SUCCESS;
    uint32_t i = 0;
    size_t encoded_sample_size = 0;
    size_t parsed_log_size = 0;
    sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
    uint8_t arr_index = 0;
    float temp[SX932x_NUM_AXES];
    pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = SX932x_NUM_AXES,.arr_index = &arr_index};

    if(NULL == encoded_log || NULL == log || NULL == bytes_written)
    {
        return SNS_RC_FAILED;
    }

    batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;

    if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields, &batch_sample))
    {
        return SNS_RC_FAILED;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
    sx931x_batch_sample *raw_sample = (sx931x_batch_sample *)log;

    while(parsed_log_size < log_size
        && (stream.bytes_written + encoded_sample_size)<= encoded_log_size
        && i < (uint32_t)(-1))
    {
        arr_index = 0;
        arg.arr = (float *)raw_sample[i].sample;

        batch_sample.sample_type = raw_sample[i].sample_type;
        batch_sample.status = raw_sample[i].status;
        batch_sample.timestamp = raw_sample[i].timestamp;

        if(!pb_encode_tag(&stream, PB_WT_STRING,
                          sns_diag_sensor_state_raw_sample_tag))
        {
          rc = SNS_RC_FAILED;
          break;
        }
        else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                                     &batch_sample))
        {
          rc = SNS_RC_FAILED;
          break;
        }

        parsed_log_size += sizeof(sx931x_batch_sample);
        i++;
    }

    if (SNS_RC_SUCCESS == rc)
    {
        *bytes_written = stream.bytes_written;
    }

    return rc;
}

void sx932x_log_sensor_state_raw_alloc(log_sensor_state_raw_info *log_raw_info,uint32_t log_size)
{
    uint32_t max_log_size = 0;

    if(NULL == log_raw_info
        || NULL == log_raw_info->diag
        || NULL == log_raw_info->instance
        || NULL == log_raw_info->sensor_uid)
    {
        return;
    }

    // allocate memory for sensor state - raw sensor log packet
    max_log_size = log_raw_info->diag->api->get_max_log_size(log_raw_info->diag);

    if (0 == log_size)
    {
        // log size not specified.
        // Use max supported log packet size
        log_raw_info->log_size = max_log_size;
    }
    else if(log_size <= max_log_size)
    {
        log_raw_info->log_size = log_size;
    }
    else
    {
        return;
    }

    log_raw_info->log = log_raw_info->diag->api->alloc_log(log_raw_info->diag,
    														log_raw_info->instance,
    														log_raw_info->sensor_uid,
    														log_raw_info->log_size,
    														SNS_DIAG_SENSOR_STATE_LOG_RAW);

    log_raw_info->log_sample_cnt = 0;
    log_raw_info->bytes_written = 0;
}

void sx932x_log_sensor_state_raw_submit(log_sensor_state_raw_info *log_raw_info,bool batch_complete)
{
    sx931x_batch_sample *sample = NULL;

    if(NULL == log_raw_info || NULL == log_raw_info->diag ||  NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid || NULL == log_raw_info->log)
    {
        return;
    }

    sample = (sx931x_batch_sample *)log_raw_info->log;
    if(batch_complete)
    {
        if(1 == log_raw_info->batch_sample_cnt)
        {
            sample[0].sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
        }
        else if(1 < log_raw_info->batch_sample_cnt)
        {
            sample[log_raw_info->log_sample_cnt - 1].sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
        }
    }

    log_raw_info->diag->api->submit_log(log_raw_info->diag,
                                        log_raw_info->instance,
                                        log_raw_info->sensor_uid,
                                        log_raw_info->bytes_written,
                                        log_raw_info->log,
                                        SNS_DIAG_SENSOR_STATE_LOG_RAW,
                                        log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
                                        sx932x_encode_log_sensor_state_raw);

    log_raw_info->log = NULL;
}

sns_rc sx932x_log_sensor_state_raw_add(log_sensor_state_raw_info *log_raw_info,
                                                float *raw_data,
                                                sns_time timestamp,
                                                sns_std_sensor_sample_status status)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if(NULL == log_raw_info
        || NULL == log_raw_info->diag
        || NULL == log_raw_info->instance
        || NULL == log_raw_info->sensor_uid
        || NULL == raw_data
        || NULL == log_raw_info->log)
    {
        return SNS_RC_FAILED;
    }

    if( (log_raw_info->bytes_written + sizeof(sx931x_batch_sample)) > log_raw_info->log_size)
    {
        sx932x_log_sensor_state_raw_submit(log_raw_info, false);
        sx932x_log_sensor_state_raw_alloc(log_raw_info, 0);
    }

    if(NULL == log_raw_info->log)
    {
        rc = SNS_RC_FAILED;
    }
    else
    {
        sx931x_batch_sample *sample = (sx931x_batch_sample *)log_raw_info->log;

        if(0 == log_raw_info->batch_sample_cnt)
        {
            sample[log_raw_info->log_sample_cnt].sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
        }
        else
        {
            sample[log_raw_info->log_sample_cnt].sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
        }
        sample[log_raw_info->log_sample_cnt].timestamp = timestamp;
        sns_memscpy(sample[log_raw_info->log_sample_cnt].sample,
                    sizeof(sample[log_raw_info->log_sample_cnt].sample),
                    raw_data,
                    sizeof(sample[log_raw_info->log_sample_cnt].sample));
        sample[log_raw_info->log_sample_cnt].status = status;
        log_raw_info->bytes_written += sizeof(sx931x_batch_sample);
        log_raw_info->log_sample_cnt++;
        log_raw_info->batch_sample_cnt++;
    }

    return rc;
}

sns_rc sx932x_com_read_wrapper(sns_sync_com_port_service *scp_service,
                                        sns_sync_com_port_handle *port_handle,
                                        uint32_t reg_addr,
                                        uint8_t  *buffer,
                                        uint32_t bytes,
                                        uint32_t *xfer_bytes)
{
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = false;
    port_vec.reg_addr = reg_addr;

    return scp_service->api->sns_scp_register_rw(port_handle,
                                                &port_vec,
                                                1,
                                                false,
                                                xfer_bytes);
}

sns_rc sx932x_com_write_wrapper(sns_sync_com_port_service * scp_service,
                                        sns_sync_com_port_handle *port_handle,
                                        uint32_t reg_addr,
                                        uint8_t  *buffer,
                                        uint32_t bytes,
                                        uint32_t *xfer_bytes,
                                        bool     save_write_time)
{
    sns_port_vector port_vec;
    port_vec.buffer = buffer;
    port_vec.bytes = bytes;
    port_vec.is_write = true;
    port_vec.reg_addr = reg_addr;

    return  scp_service->api->sns_scp_register_rw(port_handle,
                                                &port_vec,
                                                1,
                                                save_write_time,
                                                xfer_bytes);
}

sns_rc sx932x_device_sw_reset(sns_sync_com_port_service *scp_service,
                                    sns_sync_com_port_handle *port_handle)
{
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;
    sns_time cur_time;
    uint32_t xfer_bytes;

    buffer[0] = 0x00;
    rv = sx932x_com_write_wrapper(scp_service,
                                port_handle,
                                SX932x_IRQ_ENABLE_REG,
                                &buffer[0],
                                1,
                                &xfer_bytes,
                                false);
    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        return rv;
    }


    buffer[0] = SX932x_SOFTRESET_VALUE;
    rv = sx932x_com_write_wrapper(scp_service,
                                port_handle,
                                SX932x_SOFTRESET_REG,
                                &buffer[0],
                                1,
                                &xfer_bytes,
                                false);
    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        return rv;
    }

    cur_time = sns_get_system_time();
    do
    {
        if(sns_get_system_time() > (cur_time + sns_convert_ns_to_ticks(10*1000*1000)))
        {
            // Sensor HW has not recovered from SW reset.
            return SNS_RC_FAILED;
        }
        else
        {
            //1ms wait
            sns_busy_wait(sns_convert_ns_to_ticks(10*1000*1000));

            rv = sx932x_com_read_wrapper(scp_service,
                                        port_handle,
                                        SX932x_IRQSTAT_REG,
                                        &buffer[0],
                                        1,
                                        &xfer_bytes);

            if(rv != SNS_RC_SUCCESS)
            {
                // HW not ready. Keep going.
            }
            if(xfer_bytes != 1)
            {
                return SNS_RC_FAILED;
            }
        }

    } while(!(buffer[0] & 0x80));

    buffer[0] = 0x00;
    rv = sx932x_com_write_wrapper(scp_service,
                                port_handle,
                                SX932x_IRQ_ENABLE_REG,
                                &buffer[0],
                                1,
                                &xfer_bytes,
                                false);
    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        return rv;
    }

    return SNS_RC_SUCCESS;

}

sns_rc sx932x_device_set_default_state(sns_sync_com_port_service * scp_service,
                                            sns_sync_com_port_handle *port_handle,
                                            sx932x_sensor_type sensor)
{
    UNUSED_VAR(sensor);
    sns_rc rv = SNS_RC_SUCCESS;
    REGISTER_SETTING register_set;
    uint32_t xfer_bytes;
    uint32_t i;
    #ifdef SX932X_GET_PARAMETER_FROM_SMEM
    uint32_t j;
    #endif
    sns_time cur_time;
    uint8_t buffer[1];

    #ifdef SX932X_GET_PARAMETER_FROM_SMEM
    if(oppo_get_sensor_hw(OPPO_SAR, SX9324, &sar_hw) && sar_hw != NULL) {
        oppo_get_sensor_feature(OPPO_SAR, SX9324, &sar_feature);
    }
    #endif

    for (i = 0; i < g_SX932xRegisterSize; i++)
    {
        register_set = g_SX932xConfigurationSettings[i];
        #ifdef SX932X_GET_PARAMETER_FROM_SMEM
        if(sar_feature != NULL && sar_feature->parameter[0] != NULL) {
            if(sar_feature->parameter[24] > SX932X_PARAMETER_NUM)
            {
                if(sar_feature->parameter[24] > SX932X_PARAMETER_NUM_MAX)
                {
                    for(j = 0; j < SX932X_PARAMETER_NUM_MAX; j++) {
                        if (sar_feature->parameter[j * 2] == register_set.Register) {
                            register_set.Value = sar_feature->parameter[j * 2 + 1];
                            SNS_PRINTF(ERROR, sns_fw_printf, "override reg[%d]: [%02X %02X]",j, register_set.Register, register_set.Value);
                        }
                    }
                    for(j = 0; j < sar_feature->parameter[24] - SX932X_PARAMETER_NUM_MAX; j++) {
                        if (sar_feature->feature[j * 2] == register_set.Register) {
                            register_set.Value = sar_feature->feature[j * 2 + 1];
                            SNS_PRINTF(ERROR, sns_fw_printf, "override reg[%d]: [%02X %02X]",j, register_set.Register, register_set.Value);
                        }
                    }
                }
                else{
                    for(j = 0; j < sar_feature->parameter[24]; j++) {
                        if (sar_feature->parameter[j * 2] == register_set.Register) {
                            register_set.Value = sar_feature->parameter[j * 2 + 1];
                            SNS_PRINTF(ERROR, sns_fw_printf, "override reg[%d]: [%02X %02X]",j, register_set.Register, register_set.Value);
                        }
                    }
                }
            }
            else{
                for(j = 0; j < SX932X_PARAMETER_NUM; j++) {
                    if (sar_feature->parameter[j * 2] == register_set.Register) {
                        register_set.Value = sar_feature->parameter[j * 2 + 1];
                        SNS_PRINTF(ERROR, sns_fw_printf, "override reg[%d]: [%02X %02X]",j, register_set.Register, register_set.Value);
                    }
                }
            }
        }
        #endif

        rv = sx932x_com_write_wrapper(scp_service,port_handle,
                                    register_set.Register,
                                    &register_set.Value,
                                    1,
                                    &xfer_bytes,
                                    false);

        if (rv != SNS_RC_SUCCESS || xfer_bytes != 1)
        {
            return rv;
        }
    }

    cur_time = sns_get_system_time();
    do
    {
        if(sns_get_system_time() > (cur_time + sns_convert_ns_to_ticks(2000*1000*1000)))
        {
            return SNS_RC_FAILED; 	// Sensor HW has not recovered from SW reset.
        }
        else
        {
            sns_busy_wait(sns_convert_ns_to_ticks(100*1000*1000)); 	//1ms wait
            rv = sx932x_com_read_wrapper(scp_service,port_handle,
                                        SX932x_STAT2_REG,
                                        &buffer[0],
                                        1,
                                        &xfer_bytes);

            if(rv != SNS_RC_SUCCESS)
            {
                // HW not ready. Keep going.
            }
            if(xfer_bytes != 1)
            {
                return SNS_RC_FAILED;
            }
        }
    }while((buffer[0] & 0x0F)); //compensation complete

    //rest IRQ STA
    rv = sx932x_com_read_wrapper(scp_service,
                                port_handle,
                                SX932x_IRQSTAT_REG,
                                &buffer[0],
                                1,
                                &xfer_bytes);

    return SNS_RC_SUCCESS;

}

sns_rc sx932x_reset_device(sns_sync_com_port_service *scp_service,
                                sns_sync_com_port_handle *port_handle,
                                sx932x_sensor_type sensor)
{
    sns_rc rv = SNS_RC_SUCCESS;

    if (sensor == (SX9XXX_SAR))
    {
        rv = sx932x_device_sw_reset(scp_service,port_handle);
    }
    if (rv == SNS_RC_SUCCESS)
    {
        rv = sx932x_device_set_default_state(scp_service, port_handle, sensor);
    }
    return rv;
}

sns_rc sx932x_get_who_am_i(sns_sync_com_port_service * scp_service,
                                sns_sync_com_port_handle *port_handle,
                                uint8_t *buffer)
{
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;

    rv = sx932x_com_read_wrapper(scp_service,
                                port_handle,
                                SX932x_WHOAMI_REG,
                                buffer,
                                1,
                                &xfer_bytes);

    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        rv = SNS_RC_FAILED;
    }

    return rv;
}

int8_t sx932x_set_power_mode(sx932x_instance_state *state, uint8_t v_power_mode_u8)
{
    int8_t com_rslt = -1;

    if (state == SX_NULL)
    {
        com_rslt = E_NULL_PTR;
    }
    else
    {
        if (v_power_mode_u8 <= NORMAL_MODE)
        {
        }
        else
        {
            com_rslt = E_OUT_OF_RANGE;
        }
    }

    return com_rslt;
}

void sx932x_update_intr(sns_sensor_instance *const instance)
{
    uint8_t rw_buffer = 0;
    uint32_t xfer_bytes;
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    SX932X_INST_LOG(LOW, instance, "<sx932x_update_intr>");
    sns_rc rv = SNS_RC_SUCCESS;
 
    rv = sx932x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                SX932x_IRQ_ENABLE_REG,
                                &rw_buffer,
                                1,
                                &xfer_bytes);

    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        SX932X_INST_LOG(ERROR, instance,"read reg SX932x_IRQ_ENABLE_REG failed = %d", rv );
    }

    SX932X_INST_LOG(LOW, instance,"read reg SX932x_IRQ_ENABLE_REG value = 0x%x", rw_buffer );

    if (rw_buffer != 0x68)
    {
        rw_buffer = 0x68;
        rv = sx932x_com_write_wrapper(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_IRQ_ENABLE_REG,
                        &rw_buffer,
                        1,
                        &xfer_bytes,
                        false);
        if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
        {
            SX932X_INST_LOG(ERROR, instance, "write reg SX932x_IRQ_ENABLE_REG failed = %d", rv );
        }
    }

    rv = sx932x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                SX932x_IRQSTAT_REG,
                                &rw_buffer,
                                1,
                                &xfer_bytes);

    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
        SX932X_INST_LOG(ERROR, instance,"read reg SX932x_IRQSTAT_REG failed = %d", rv );
        rv = SNS_RC_FAILED;
    }
}

void  sx932x_dump_reg(sns_sensor_instance *const instance)
{
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    int8_t com_rslt = -1;
    uint8_t a_data_u8r = 0;
    uint32_t xfer_bytes;
    uint8_t reg_map[] = {
        //Interrupt
        SX932x_IRQ_ENABLE_REG,
        //General control
        SX932x_CTRL0_REG,
        //Analog front end Control
        SX932x_AFE_CTRL3_REG,
        SX932x_AFE_CTRL4_REG,
        SX932x_AFE_CTRL6_REG,
        SX932x_AFE_CTRL7_REG,
        SX932x_AFE_PH0_REG,
        SX932x_AFE_PH1_REG,
        SX932x_AFE_PH2_REG,
        SX932x_AFE_PH3_REG,
        //Main digital Prox control
        SX932x_PROX_CTRL0_REG,
        SX932x_PROX_CTRL1_REG,
        SX932x_PROX_CTRL2_REG,
        SX932x_PROX_CTRL3_REG,
        SX932x_PROX_CTRL4_REG,
        SX932x_PROX_CTRL5_REG,
        SX932x_PROX_CTRL6_REG,
        SX932x_PROX_CTRL7_REG,
        //Advanced digital control
        SX932x_ADV_CTRL0_REG,
        SX932x_ADV_CTRL1_REG,
        SX932x_ADV_CTRL2_REG,
        SX932x_ADV_CTRL3_REG,
        SX932x_ADV_CTRL4_REG,
        SX932x_ADV_CTRL5_REG,
        SX932x_ADV_CTRL6_REG,
        SX932x_ADV_CTRL7_REG,
        SX932x_ADV_CTRL8_REG,
        SX932x_ADV_CTRL9_REG,
        SX932x_ADV_CTRL10_REG,
        SX932x_ADV_CTRL11_REG,
        SX932x_ADV_CTRL12_REG,
        SX932x_ADV_CTRL13_REG,
        SX932x_ADV_CTRL14_REG,
        SX932x_ADV_CTRL15_REG,
        SX932x_ADV_CTRL16_REG,
        SX932x_ADV_CTRL17_REG,
        SX932x_ADV_CTRL18_REG,
        SX932x_ADV_CTRL19_REG,
        SX932x_ADV_CTRL20_REG,
        //Phase enable
        SX932x_CTRL1_REG,
    };

    if (state == SX_NULL)
    {
        com_rslt = E_NULL_PTR;
    }
    else
    {
        uint8_t i = 0;
        uint16_t n = sizeof(reg_map)/sizeof(reg_map[0]);
        for(i = 0; i < n; i++)
        {
            com_rslt = state->com_read(state->scp_service,
                                    state->com_port_info.port_handle,
                                    reg_map[i], &a_data_u8r,
                                    1,
                                    &xfer_bytes);
            SX932X_INST_LOG(MED, instance, "reg[0x%X] = 0x%X",reg_map[i], a_data_u8r);
        }
    }
}

void sx932x_send_config_event(sns_sensor_instance *const instance)
{
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    sns_std_sensor_physical_config_event phy_sensor_config = sns_std_sensor_physical_config_event_init_default;
    char operating_mode[] = "NORMAL";
    pb_buffer_arg op_mode_args;

    op_mode_args.buf = &operating_mode[0];
    op_mode_args.buf_len = sizeof(operating_mode);
    phy_sensor_config.has_sample_rate = true;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = 1;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.has_stream_is_synchronous = true;
    phy_sensor_config.stream_is_synchronous = false;
    if (state->deploy_info.publish_sensors & SX9XXX_SAR)
    {
        phy_sensor_config.sample_rate = state->sar_info.sampling_rate_hz;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 720;
        phy_sensor_config.resolution = SX932x_SENSOR_SAR_RESOLUTION;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = SX932x_SENSOR_SAR_RANGE_MIN;
        phy_sensor_config.range[1] = SX932x_SENSOR_SAR_RANGE_MAX;

        pb_send_event(instance,
                    sns_std_sensor_physical_config_event_fields,
                    &phy_sensor_config,
                    sns_get_system_time(),
                    SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                    &state->sar_info.suid);
    }
}

static void sx932x_process_sar_report_stream_data_buffer(sns_sensor_instance *const instance)
{
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    sns_diag_service* diag       = state->diag_service;
    log_sensor_state_raw_info log_sensor_state_raw_info;
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    sns_time timestamp = sns_get_system_time();
    sns_sensor_uid suid;
    uint32_t xfer_bytes;
    int16_t useful_data_ph[3];
    int16_t diff_data_ph[3];
    uint16_t dc_offset_ph[3];
    uint8_t proximity_state;
    uint8_t tempData[2];
    float sample_data[7];
    uint8_t i = 0;

	uint8_t state1 = 0;
	uint8_t state3 = 0;

	int16_t sar_data;

    SX932X_INST_LOG(LOW, instance, "sx932x_process_sar_report_stream_data_buffer");

    //read state
    state->com_read(state->scp_service, state->com_port_info.port_handle,
                SX932x_STAT0_REG,
                tempData,
                1,
                &xfer_bytes);
    proximity_state = tempData[0]&0x0F;

	state->com_read(state->scp_service, state->com_port_info.port_handle,
                SX932x_STAT1_REG,
                tempData,
                1,
                &xfer_bytes);
    state1 = tempData[0];

	state->com_read(state->scp_service, state->com_port_info.port_handle,
                SX932x_STAT3_REG,
                tempData,
                1,
                &xfer_bytes);
    state3 = tempData[0];

    for(i = 0; i < 3; i++)
    {
        //here just read the PH0
        state->com_write(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_CPSRD,
                        &i,
                        1,
                        &xfer_bytes,
                        false);
        //read useful
        state->com_read(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_USEMSB,
                        tempData,
                        2,
                        &xfer_bytes);
        useful_data_ph[i] = (int16_t) tempData[0] << 8 | tempData[1];
        //read diff
        state->com_read(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_DIFFMSB,
                        tempData,
                        2,
                        &xfer_bytes);
        diff_data_ph[i] = (int16_t) tempData[0] << 8 | tempData[1];
        //read offset
        state->com_read(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_OFFSETMSB,
                        tempData,
                        2,
                        &xfer_bytes);
        dc_offset_ph[i] = (uint16_t) tempData[0] << 8 | tempData[1];

        SX932X_INST_LOG(ERROR, instance, "sx932x PH[%d] useful= %d; diff= %d; dc_offset= %d",
                        i,
                        useful_data_ph[i],
                        diff_data_ph[i],
                        dc_offset_ph[i]);
    }

	state->com_read(state->scp_service,
					state->com_port_info.port_handle,
					SX932x_SARMSB,
					tempData,
					2,
					&xfer_bytes);
	sar_data = (int16_t) tempData[0] << 8 | tempData[1];

    suid = state->sar_info.suid;

    SX932X_INST_LOG(ERROR, instance, "status = %d; diff= %d; dc_offset= %d",
      proximity_state, diff_data_ph[0], dc_offset_ph[0]);

#if 0
    sample_data[0] = (float)proximity_state;//(proximity_state != 0);

    sample_data[1] = (float)diff_data_ph[0];
    sample_data[2] = (float)dc_offset_ph[0];//(float)dc_offset_ph[0];

    sample_data[3] = (float)diff_data_ph[2];
    sample_data[4] = (float)dc_offset_ph[2];//state3;//(float)dc_offset_ph[2];
#endif

    sample_data[0] = (float)((proximity_state&0x04)>>2);//(proximity_state != 0);

    sample_data[1] = (float)diff_data_ph[2];
    sample_data[2] = (float)dc_offset_ph[2];//(float)dc_offset_ph[0];

    sample_data[3] = (float)diff_data_ph[0];
    sample_data[4] = (float)dc_offset_ph[0];//state3;//(float)dc_offset_ph[2];

    pb_send_sensor_stream_event(instance,
                                &suid,
                                timestamp,
                                SNS_SAR_MSGID_SNS_SAR_DATA,
                                SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                sample_data,
                                5,
                                state->encoded_imu_event_len);
    // log information
    log_sensor_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
    log_sensor_state_raw_info.diag = diag;
    log_sensor_state_raw_info.instance = instance;
    log_sensor_state_raw_info.sensor_uid = &suid;

    sx932x_log_sensor_state_raw_alloc(&log_sensor_state_raw_info, 0);
    sx932x_log_sensor_state_raw_add(&log_sensor_state_raw_info,
                                    sample_data,
                                    timestamp,
                                    status);
    sx932x_log_sensor_state_raw_submit(&log_sensor_state_raw_info, true);
}

void sx932x_handle_sar_data_stream_interrupt_event(sns_sensor_instance *const instance)
{
    sx932x_update_intr(instance);
    sx932x_process_sar_report_stream_data_buffer(instance);
}

void sx932x_handle_sar_data_stream_timer_event(sns_sensor_instance *const instance)
{
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    int err = 0;

    SX932X_INST_LOG(LOW, instance, "sar timer_event");

    sx932x_process_sar_report_stream_data_buffer(instance);
    err = sx932x_set_power_mode(state,NORMAL_MODE);
    if (err)
    {
        SX932X_INST_LOG(ERROR, instance,"config power mode failed err = %d", err);
    }
}

void sx932x_register_interrupt(sns_sensor_instance *this)
{
    sx932x_instance_state *state = (sx932x_instance_state*)this->state->state;

    if(!state->irq_info.irq_registered)
    {
        sns_data_stream* data_stream = state->interrupt_data_stream;
        uint8_t buffer[20];
        sns_request irq_req ={.message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ, .request = buffer};

        irq_req.request_len = pb_encode_request(buffer,
                                                sizeof(buffer),
                                                &state->irq_info.irq_config,
                                                sns_interrupt_req_fields,
                                                NULL);
        if(irq_req.request_len > 0)
        {
            data_stream->api->send_request(data_stream, &irq_req);
            state->irq_info.irq_registered = true;
        }
    }
}
