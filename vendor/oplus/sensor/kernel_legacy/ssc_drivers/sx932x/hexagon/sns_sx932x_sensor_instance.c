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
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"
#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sensor_util.h"
#include "sns_sync_com_port_service.h"

#include "sns_sx932x_hal.h"
#include "sns_sx932x_sensor.h"
#include "sns_sx932x_sensor_instance.h"

static void sx932x_send_com_test_event(sns_sensor_instance *instance,         sns_sensor_uid *uid, bool test_result)
{
    uint8_t data[1] = {0};
    pb_buffer_arg buff_arg = (pb_buffer_arg) { .buf = &data, .buf_len = sizeof(data) };
    sns_physical_sensor_test_event test_event = sns_physical_sensor_test_event_init_default;

    test_event.test_passed = test_result;
    test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
    test_event.test_data.funcs.encode = &pb_encode_string_cb;
    test_event.test_data.arg = &buff_arg;

    pb_send_event(instance,
                  sns_physical_sensor_test_event_fields,
                  &test_event,
                  sns_get_system_time(),
                  SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                  uid);
}

static void sx931x_send_factory_test_event(sns_sensor_instance *instance,
                                        sns_sensor_uid *uid, bool test_result)
{
    uint8_t data[1] = {0};
    pb_buffer_arg buff_arg = (pb_buffer_arg){ .buf = &data, .buf_len = sizeof(data)};
    sns_physical_sensor_test_event test_event = sns_physical_sensor_test_event_init_default;

    test_event.test_passed = test_result;
    test_event.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
    test_event.test_data.funcs.encode = &pb_encode_string_cb;
    test_event.test_data.arg = &buff_arg;

    pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                uid);
}

static bool sx932x_device_force_compensation(sns_sensor_instance *const instance)
{
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;
    uint32_t xfer_bytes;
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;

#if 0
    buffer[0] = 0x0F; //compensation
    rv = state->com_write(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_STAT2_REG,
                        &buffer[0],
                        1,
                        &xfer_bytes,
                        false);
    if(rv != SNS_RC_SUCCESS  ||  xfer_bytes != 1)
    {
        SX932X_INST_LOG(LOW, instance, "sx932x_device_force_compensation write error");
        return false;
    }
#endif
    buffer[0] = 0x20; //compensation
    rv = state->com_write(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_CTRL1_REG,
                        &buffer[0],
                        1,
                        &xfer_bytes,
                        false);
    if(rv != SNS_RC_SUCCESS  ||  xfer_bytes != 1)
    {
        SX932X_INST_LOG(LOW, instance, "sx932x_device_force_compensation write error");
        return false;
    }

	sns_time delay_ticksl;
	delay_ticksl = sns_convert_ns_to_ticks(100 * 1000 * 1000);
	sns_busy_wait(delay_ticksl);

    buffer[0] = 0x27; //compensation
    rv = state->com_write(state->scp_service,
                        state->com_port_info.port_handle,
                        SX932x_CTRL1_REG,
                        &buffer[0],
                        1,
                        &xfer_bytes,
                        false);
    if(rv != SNS_RC_SUCCESS  ||  xfer_bytes != 1)
    {
        SX932X_INST_LOG(LOW, instance, "sx932x_device_force_compensation write error");
        return false;
    }

	//sns_time delay_ticksl;
	delay_ticksl = sns_convert_ns_to_ticks(500 * 1000 * 1000);

	sns_busy_wait(delay_ticksl);

    return true;
}

void sx932x_run_self_test(sns_sensor_instance *instance)
{
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    sns_rc rv = SNS_RC_SUCCESS;
    uint8_t buffer = 0;
    bool who_am_i_success = false;

    rv = sx932x_get_who_am_i(state->scp_service, state->com_port_info.port_handle,  &buffer);
    if(rv == SNS_RC_SUCCESS && ((buffer == SX932x_WHOAMI_VALUE) || (buffer == SX932x_WHOAMI_VALUE2)))
    {
        who_am_i_success = true;
    }

    if(state->sar_info.test_info.test_client_present)
    {
        if(state->sar_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
        {
            sx932x_send_com_test_event(instance, &state->sar_info.suid, who_am_i_success);
        }
        else if(state->sar_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
        {
            bool test_result =  sx932x_device_force_compensation(instance);
            SX932X_INST_LOG(ERROR, instance, "sx931x_run_self_test result=%d", test_result);
            sx932x_force_intr(instance);
            sx931x_send_factory_test_event(instance, &state->sar_info.suid, test_result);
        }
        state->sar_info.test_info.test_client_present = false;
    }
}

static void inst_cleanup(sx932x_instance_state *state, sns_stream_service *stream_mgr)
{
    if(NULL != state->sar_timer_data_stream)
    {
        stream_mgr->api->remove_stream(stream_mgr, state->sar_timer_data_stream);
        state->sar_timer_data_stream = NULL;
    }

    if(NULL != state->scp_service)
    {
        state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
        state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
        state->scp_service = NULL;
    }
}

sns_rc sns_see_sx932x_inst_init(sns_sensor_instance * const this, sns_sensor_state const *sstate)
{
    sx932x_instance_state *state = (sx932x_instance_state*) this->state->state;
    sx932x_state *sensor_state = (sx932x_state*) sstate->state;
    float stream_data[6] = {0};
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr = (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    uint64_t buffer[10];
    pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
    sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
    uint8_t arr_index = 0;
    float diag_temp[SX932x_NUM_AXES];
    pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = SX932x_NUM_AXES,  	.arr_index = &arr_index};

    batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;
    state->scp_service = (sns_sync_com_port_service*)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

    SX932X_INST_LOG(LOW, this, "inst_init from sensor:0x%x", sensor_state->sensor);

    /************Setup stream connections with dependent Sensors*************************/
	//sns_suid_lookup_get(&sensor_state->suid_lookup_data, "interrupt", &sensor_state->irq_suid);
    stream_mgr->api->create_sensor_instance_stream(stream_mgr, this, sensor_state->irq_suid, &state->interrupt_data_stream);
	if (state->interrupt_data_stream == NULL)
		SX932X_INST_LOG(HIGH, this, "state->interrupt_data_stream == NULL");


    /**************Initialize COM port to be used by the Instance**************************/
    sns_memscpy(&state->com_port_info.com_config, sizeof(state->com_port_info.com_config), &sensor_state->com_port_info.com_config, sizeof(sensor_state->com_port_info.com_config));
    state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config, &state->com_port_info.port_handle);
	if (state->com_port_info.port_handle == NULL)
		SX932X_INST_LOG(HIGH, this, "state->com_port_info.port_handle == NULL");

    if ((NULL == state->interrupt_data_stream) || (NULL == state->com_port_info.port_handle))
    {
        inst_cleanup(state, stream_mgr);
        return SNS_RC_FAILED;
    }

    /**********************Copy all Sensor UIDs in instance state ************************/
    sns_memscpy(&state->sar_info.suid, sizeof(state->sar_info.suid),  &((sns_sensor_uid)SAR_SUID), sizeof(state->sar_info.suid));
    sns_memscpy(&state->timer_suid,  sizeof(state->timer_suid),  &sensor_state->timer_suid,  sizeof(sensor_state->timer_suid));

    /*************************Init sar sensor **************************************/
    state->interface = sensor_state->com_port_info.com_config.bus_instance;
    state->op_mode = NORMAL_MODE;/*default working mode*/
    state->com_read = sx932x_com_read_wrapper;/* com read function*/
    state->com_write = sx932x_com_write_wrapper;/*com write function*/
    state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(stream_data, 5);
    state->diag_service =  (sns_diag_service*)service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
    state->scp_service =  (sns_sync_com_port_service*)service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
    state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

    /*************************Initialize IRQ info to be used by the Instance****************/
    /*register interrupt*/
    state->is_dri = sensor_state->is_dri;
    state->irq_info.irq_config = sensor_state->irq_config;
    state->irq_info.irq_ready = false;
    /*****************************for test****************************************/
    sx932x_reset_device(state->scp_service,state->com_port_info.port_handle,  SX9XXX_SAR);
    /****************************end test****************************************/
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);

    /******************Determine sizes of encoded logs *******************************/
    sns_diag_sensor_state_interrupt sensor_state_interrupt=sns_diag_sensor_state_interrupt_init_default;
    pb_get_encoded_size(&state->log_interrupt_encoded_size, sns_diag_sensor_state_interrupt_fields, &sensor_state_interrupt);

    /***************************************************************************
    * Determine size of sns_diag_sensor_state_raw as defined in
    *  sns_diag.proto
    *  sns_diag_sensor_state_raw is a repeated array of samples of
    *  type sns_diag_batch sample. The following determines the
    *  size of sns_diag_sensor_state_raw with a single batch
    *  sample
    ***************************************************************************/
    if(pb_encode_tag(&stream, PB_WT_STRING, sns_diag_sensor_state_raw_sample_tag))
    {
        if (pb_encode_delimited(&stream, sns_diag_batch_sample_fields, &batch_sample))
        {
            state->log_raw_encoded_size = stream.bytes_written;
        }
    }

    SX932X_INST_LOG(LOW, this, "inst_init success");
    return SNS_RC_SUCCESS;
}


sns_rc sns_see_sx932x_inst_deinit(sns_sensor_instance *const this)
{
    sx932x_instance_state *state = (sx932x_instance_state*) this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr = (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    inst_cleanup(state, stream_mgr);
    return SNS_RC_SUCCESS;
}
