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
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_island_service.h"
#include "sns_sensor_util.h"

#include "sns_sx932x_hal.h"
#include "sns_sx932x_sensor.h"
#include "sns_sx932x_sensor_instance.h"

static void sx932x_inst_exit_island(sns_sensor_instance *this)
{
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_instance_island_exit(island_svc, this);
}

void sx932x_start_sensor_sar_polling_timer(sns_sensor_instance *this)
{
    sx932x_instance_state *state = (sx932x_instance_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t buffer[50] = {0};
    sns_request timer_req = { .message_id=SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG, .request= buffer};
    sns_rc rc = SNS_RC_SUCCESS;

    SX932X_INST_LOG(LOW, this, "start_sensor_sar_polling_timer");
    if (NULL == state->sar_timer_data_stream)
    {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service *srtm_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

        rc = srtm_svc->api->create_sensor_instance_stream(srtm_svc, this, state->timer_suid, &state->sar_timer_data_stream);
    }

    if(SNS_RC_SUCCESS != rc || NULL == state->sar_timer_data_stream)
    {
        SX932X_INST_LOG(ERROR, this, "failed timer stream create rc = %d", rc);
        return;
    }

    req_payload.is_periodic = true;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = state->sar_info.sampling_intvl;
    timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);
    if(timer_req.request_len > 0)
    {
        state->sar_timer_data_stream->api->send_request(state->sar_timer_data_stream, &timer_req);
        state->sar_info.timer_is_active = true;
    }
}

void sx932x_set_sar_polling_config(sns_sensor_instance *const this)
{
    sx932x_instance_state *state = (sx932x_instance_state*)this->state->state;

    SX932X_INST_LOG(LOW, this, "sar_info.timer_is_active:%d state->sar_info.sampling_intvl:%u",state->sar_info.timer_is_active,state->sar_info.sampling_intvl);

    if(state->sar_info.sampling_intvl > 0)
    {
        if(!state->sar_info.timer_is_active)
        {
            sx932x_start_sensor_sar_polling_timer(this);
        }
    }
    else if(state->sar_info.timer_is_active)
    {
        state->sar_info.timer_is_active = false;
        sns_sensor_util_remove_sensor_instance_stream(this, &state->sar_timer_data_stream);
    }
}

void sx932x_reconfig_hw(sns_sensor_instance *this, sx932x_sensor_type sensor_type)
{
    sx932x_instance_state *state = (sx932x_instance_state*)this->state->state;

    SX932X_INST_LOG(LOW, this, "sx932x_reconfig_hw state->config_step = %d", state->config_step);
    SX932X_INST_LOG(LOW, this, "enable sensor flag:0x%x publish sensor flag:0x%x", state->deploy_info.enable, state->deploy_info.publish_sensors);

    // Enable timer in case of sensor sar clients
    if (sensor_type == SX9XXX_SAR)
    {
        if(state->is_dri)
        {
            if(state->irq_info.irq_ready)
            {
                SX932X_INST_LOG(LOW, this, "sx932x_update_intr finished");
                //sx932x_update_intr(this);
            }
        }
        else
        {
            sx932x_set_sar_polling_config(this);
        }
    }

    state->config_step = CONFIG_IDLE; /* done with reconfig */

    sx932x_dump_reg(this);

    SX932X_INST_LOG(LOW, this, "sx932x_reconfig_hw finished");
}

void sx932x_force_intr(sns_sensor_instance *const instance)
{
    uint8_t rw_buffer = 0;
    uint32_t xfer_bytes;
    sx932x_instance_state *state = (sx932x_instance_state*)instance->state->state;
    SX932X_INST_LOG(LOW, instance, "<sx932x_force_intr>");
    sns_rc rv = SNS_RC_SUCCESS;
 
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

static sns_rc sx932x_validate_sensor_sar_odr(sns_sensor_instance *this)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sx932x_instance_state *state = (sx932x_instance_state*)this->state->state;

    SX932X_INST_LOG(LOW, this, "sar odr = %d", (int8_t)state->sar_info.sampling_rate_hz);
    if(state->sar_info.sampling_rate_hz > SMT_ODR_0 && state->sar_info.sampling_rate_hz <= SMT_ODR_1)
    {
        state->sar_info.sampling_rate_hz = SMT_ODR_1;
    }
    else if(state->sar_info.sampling_rate_hz > SMT_ODR_1 && state->sar_info.sampling_rate_hz <= SMT_ODR_5)
    {
        state->sar_info.sampling_rate_hz = SMT_ODR_5;
    }
    else if(state->sar_info.sampling_rate_hz > SMT_ODR_5 && state->sar_info.sampling_rate_hz <= SMT_ODR_10)
    {
        state->sar_info.sampling_rate_hz = SMT_ODR_10;
    }
    else if(state->sar_info.sampling_rate_hz > SMT_ODR_10)
    {
        state->sar_info.sampling_rate_hz = SMT_ODR_25;
    }
    else
    {
        state->sar_info.sampling_intvl = 0;
        state->sar_info.timer_is_active = 0;
        SX932X_INST_LOG(LOW, this, "close sar sensor = %d, timer_is_active =%d", (uint32_t)state->sar_info.sampling_rate_hz, state->sar_info.timer_is_active);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    if (rc == SNS_RC_SUCCESS)
    {
        state->sar_info.sampling_intvl = sns_convert_ns_to_ticks(1000000000.0 / state->sar_info.sampling_rate_hz);
        SX932X_INST_LOG(LOW, this, "sar timer_value = %u", (uint32_t)state->sar_info.sampling_intvl);
    }

    return rc;
}

static sns_rc sns_see_sx932x_inst_notify_event(sns_sensor_instance * const this)
{
    sx932x_instance_state *state = (sx932x_instance_state*) this->state->state;
    sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
    sns_sensor_event *event;

    SX932X_INST_LOG(LOW, this, "inst_notify_event called");

    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);

    /*************************Handle interrupt*******************************/
    if (NULL != state->interrupt_data_stream)
    {
        event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
        while(NULL != event)
        {
            if(event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT)
            {
                SX932X_INST_LOG(ERROR, this, "inst_notify_event INTERRUPT_REG_EVENT");

                state->irq_info.irq_ready = true;
                //sx932x_update_intr(this);
            }
            else if(event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT)
            {
                SX932X_INST_LOG(ERROR, this, "inst_notify_event INTERRUPT_EVENT");

                pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
                if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
                {
                    sx932x_handle_sar_data_stream_interrupt_event(this);
                }
            }
            else
            {
                SX932X_INST_LOG(ERROR, this, "Received invalid event id=%d", event->message_id);
            }
            event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
        }
    }


    /***** timer event, different odr should has different timer to handle it in a instance ****/
    if (state->sar_timer_data_stream != NULL)
    {
        event = state->sar_timer_data_stream->api->peek_input(state->sar_timer_data_stream);
        while (NULL != event)
        {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,event->event_len);
            sns_timer_sensor_event timer_event;
            if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
            {
                if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
                {
                    if(state->sar_info.timer_is_active && state->sar_info.sampling_intvl > 0)
                    {
                        sx932x_handle_sar_data_stream_timer_event(this);
                    }
                }
            }
            else
            {
                SX932X_INST_LOG(ERROR, this, "Received invalid event id=%d", event->message_id);
            }
            event = state->sar_timer_data_stream->api->get_next_input(state->sar_timer_data_stream);
        }
    }

    // Turn COM port OFF
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);

    return SNS_RC_SUCCESS;
}

static sns_rc sns_see_sx932x_inst_set_client_config(    sns_sensor_instance * const this, sns_request const *client_request)
{
    sx932x_instance_state *state = (sx932x_instance_state*) this->state->state;
    state->client_req_id = client_request->message_id;
    float desired_sample_rate = 0;
    float desired_report_rate = 0;
    sx932x_sensor_type sensor_type = SX9XXX_SENSOR_INVALID;
    sx932x_power_mode op_mode = INVALID_WORK_MODE;
    sns_sx932x_cfg_req *payload = (sns_sx932x_cfg_req*)client_request->request;
    sns_rc rv = SNS_RC_SUCCESS;

    SX932X_INST_LOG(LOW, this, "inst_set_client_config message_id = %d",client_request->message_id);

    /* Turn COM port ON, *physical* */
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);

    if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
        // 1. Extract sample, report rates from client_request.
        // 2. Configure sensor HW.
        // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
        // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
        // 5. Save the current config information like type, sample_rate, report_rate, etc.

        desired_sample_rate = payload->sample_rate;
        desired_report_rate = payload->report_rate;
        sensor_type = payload->sensor_type;
        op_mode = payload->op_mode;

        SX932X_INST_LOG(LOW, this, "inst_set_client_config desired_sample_rate = %d, desired_report_rate=%d, sensor_type=%d, op_mode=%d, state->is_dri=%d, state->config_step=%d"
                        ,desired_sample_rate
                        ,desired_report_rate
                        ,sensor_type
                        ,op_mode
                        ,state->is_dri
                        ,state->config_step);

        if (desired_report_rate > desired_sample_rate)
        {
            /* bad request. Return error or default report_rate to sample_rate */
            desired_report_rate = desired_sample_rate;
        }

        if (sensor_type == SX9XXX_SAR)
        {
            rv = sx932x_validate_sensor_sar_odr(this);
            if (rv != SNS_RC_SUCCESS && desired_sample_rate != 0)
            {
                // TODO Unsupported rate. Report error using sns_std_error_event.
                SX932X_INST_LOG(ERROR, this, "sensor_pressure ODR match error %d", rv);
                //return rv;
            }
        }

        if (state->deploy_info.publish_sensors == SX9XXX_SAR && state->deploy_info.enable == 1)
        {
            if (state->is_dri) //irq
            {
                sx932x_register_interrupt(this);
                //force interupt first time
                sx932x_force_intr(this);
            }
            else //timer
            {
                if (CONFIG_IDLE == state->config_step)
                {
                    state->config_step = CONFIG_STOPPING_STREAM;
                }

                if (state->config_step == CONFIG_IDLE)
                {
                    sx932x_reconfig_hw(this, sensor_type);
                }
            }
        }
        else
        {
            rv = sx932x_device_sw_reset(state->scp_service,state->com_port_info.port_handle);
            SX932X_INST_LOG(LOW, this, "sx932x_device_sw_reset result %d", rv);
        }

        sx932x_send_config_event(this);
    }
    else if (client_request->message_id ==  SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
    {
    	/* All self-tests can be handled in normal mode. */
		sx932x_inst_exit_island(this);
        sx932x_run_self_test(this);
        state->new_self_test_request = false;
    }

    // Turn COM port OFF
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);

    SX932X_INST_LOG(LOW, this, "inst_set_client_config exit");
    return SNS_RC_SUCCESS;
}

sns_sensor_instance_api sns_see_sx932x_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_see_sx932x_inst_init,
    .deinit = &sns_see_sx932x_inst_deinit,
    .set_client_config = &sns_see_sx932x_inst_set_client_config,
    .notify_event = &sns_see_sx932x_inst_notify_event
};
