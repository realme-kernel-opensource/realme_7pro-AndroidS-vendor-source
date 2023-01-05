/*
 * Copyright (c) 2018, ams AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "sns_island_service.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_time.h"
#include "sns_types.h"

#include "sns_async_com_port.pb.h"
#include "sns_interrupt.pb.h"
#include "sns_timer.pb.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#include "sns_pb_util.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"

#include "sns_tcs3408r_hal.h"
#include "sns_tcs3408r_sensor.h"
#include "sns_tcs3408r_sensor_instance.h"

void tcs3408r_register_interrupt(sns_sensor_instance *this)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)this->state->state;
	if (!state->irq_info.irq_registered)
	{
		sns_data_stream* data_stream = state->interrupt_data_stream;
		uint8_t buffer[20];
		sns_request irq_req =
		{
			.message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
			.request    = buffer
		};

		irq_req.request_len = pb_encode_request(buffer,
			sizeof(buffer),
			&state->irq_info.irq_config,
			sns_interrupt_req_fields,
			NULL);
		if (irq_req.request_len > 0)
		{
			data_stream->api->send_request(data_stream, &irq_req);
			state->irq_info.irq_registered = true;
		}
	}
}

void tcs3408r_start_polling_timer(sns_sensor_instance *const this,
									sns_time timeout_ticks)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)this->state->state;
	sns_service_manager *service_mgr = this->cb->get_service_manager(this);
	sns_stream_service *stream_mgr = (sns_stream_service*)
	service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

	sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
	size_t req_len;
	uint8_t buffer[20];
	sns_memset(buffer, 0, sizeof(buffer));
	req_payload.is_periodic = true;
	req_payload.start_time = sns_get_system_time();
	req_payload.timeout_period = timeout_ticks;

	if (NULL == state->timer_data_stream)
	{
		stream_mgr->api->create_sensor_instance_stream(stream_mgr,
			this,
			state->timer_suid,
			&state->timer_data_stream);
	}

	req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
		sns_timer_sensor_config_fields, NULL);

	if(req_len > 0 && NULL != state->timer_data_stream)
	{
		sns_request timer_req =
			{.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
			.request = buffer, .request_len = req_len};
		state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
	}
	else
	{
		SNS_INST_PRINTF(ERROR, this, "TCS3408 polling timer req encode error");
	}
}

static sns_rc tcs3408r_inst_notify_event(sns_sensor_instance *const this)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)this->state->state;
	sns_sensor_event *event;

	//SNS_INST_PRINTF(LOW, this, "TCS3408 tcs3408r_inst_notify_event\n");

	/* Turn COM port ON */
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);

	/* Handle interrupts */
	if (NULL != state->interrupt_data_stream)
	{
		event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
		while (NULL != event)
		{
			if (event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT)
			{
				state->irq_info.irq_ready = true;
				SNS_INST_PRINTF(LOW, this, "TCS3408 tcs3408r_inst_notify_event SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT\n");
			}
			else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
			{
				sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
				pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
				if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
				{
					tcs3408r_process_sensor_data(this);
					if (!state->als_info.test_info.test_client_present)
					{
						if ((state->publish_sensors&TCS3408_ALS) && state->chanl_data_ready)
						{
							tcs3408r_handle_sensor_als_sample(this, irq_event.timestamp);
						}
						if ((state->publish_sensors&TCS3408_RGB) && state->chanl_data_ready)
						{
						
							tcs3408r_handle_sensor_rgb_sample(this, irq_event.timestamp);
						}
					}
				}
				SNS_INST_PRINTF(LOW, this, "TCS3408 tcs3408r_inst_notify_event SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT\n");
			}
			else
			{
				SNS_INST_PRINTF(ERROR, this, "TCS3408 Received invalid event id=%d", event->message_id);
			}
			event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
		}
	}

	/* Handle Timer events */
	if (NULL != state->timer_data_stream)
	{
		event = state->timer_data_stream->api->peek_input(state->timer_data_stream);
		while (NULL != event)
		{
			pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

			if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
			{
				sns_timer_sensor_event timer_event;
				if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
				{
					if (!state->als_info.test_info.test_client_present &&
						!state->flicker_info.test_info.test_client_present &&
						!state->rgb_info.test_info.test_client_present)
					{
						tcs3408r_process_sensor_data(this);
						if (!state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS) && state->chanl_data_ready)
						{
							tcs3408r_handle_sensor_als_sample(this, timer_event.timeout_time);
						}
				        //SNS_INST_PRINTF(MED, this, "TCS3408 rgb parameters rgb_registry.is_dri %d, state->publish_sensors is %d\n", state->rgb_registry_cfg.is_dri,state->publish_sensors);
						if (!state->rgb_registry_cfg.is_dri && (state->publish_sensors&TCS3408_RGB) && state->chanl_data_ready)
						{
						    //SNS_INST_PRINTF(HIGH, this, "TCS3408 tcs3408r_inst_notify_event and enter TCS3408_RGB\n");
							tcs3408r_handle_sensor_rgb_sample(this, timer_event.timeout_time);
						}
						if (!state->flicker_registry_cfg.is_dri && (state->publish_sensors&TCS3408_FLICKER) && state->flicker_info.data_ready)
						{
						    //SNS_INST_PRINTF(HIGH, this, "TCS3408 tcs3408r_inst_notify_event and enter TCS3408_FLICKER\n");
							tcs3408r_handle_sensor_flicker_sample(this, timer_event.timeout_time);
						}	
					}

					SNS_INST_PRINTF(HIGH, this, "TCS3408 tcs3408r_inst_notify_event enter timer.\n");
				}
			}
			else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
			{
				/* TODO: decode and qse timer_reg_event*/
				SNS_INST_PRINTF(LOW, this, "TCS3408 TIMER_SENSOR_REG_EVENT");
			}
			else
			{
				SNS_INST_PRINTF(MED, this, "TCS3408 unknown message_id %d", event->message_id);
			}
			event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
		}
	}

	/*factory cali*/
	if (NULL != state->fac_cali_timer_data_stream)
	{
		event = state->fac_cali_timer_data_stream->api->peek_input(state->fac_cali_timer_data_stream);

		while (NULL != event)
		{
			if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT== event->message_id)
			{
				sns_timer_sensor_event timer_event;
				pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
				SNS_INST_PRINTF(LOW, this, "tcs3408r_inst_notify_event: cali timer");

				if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
				{
					if (state->fac_cali_st != TCS3408_CALI_IDLE)
					{
						tcs3408r_channel_cali(this);
					}
				}
			}
			else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
			{
				/** TODO: decode and qse timer_reg_event*/
				SNS_INST_PRINTF(LOW, this, "CALI TIMER_SENSOR_REG_EVENT");
			}
			else
			{
				SNS_INST_PRINTF(MED, this, "unknown message_id %d", event->message_id);
			}

			event = state->fac_cali_timer_data_stream->api->get_next_input(state->fac_cali_timer_data_stream);
		}

		if (TCS3408_CALI_IDLE == state->fac_cali_st)
		{
			SNS_INST_PRINTF(ERROR, this, "CALI DONE, REMOVE CALI TIMER");
			sns_sensor_util_remove_sensor_instance_stream(this, &state->fac_cali_timer_data_stream);
			state->fac_cali_timer_data_stream = NULL;
		}
	}

	/* Turn COM port OFF */
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
	return SNS_RC_SUCCESS;
}

static sns_rc tcs3408r_inst_set_client_config(sns_sensor_instance *const this,
									sns_request const *client_request)
{
	tcs3408_instance_state *state = (tcs3408_instance_state*)this->state->state;
	float desired_sample_rate = 0.0;
	float desired_report_rate = 0.0;
	uint8_t enable = 0;
	uint8_t intenab = 0;
	sns_tcs3408_req *payload = (sns_tcs3408_req*)client_request->request;

	SNS_INST_PRINTF(LOW, this, "TCS3408 set_client_cfg msg_id %d", client_request->message_id);

	/* Turn COM port ON */
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);

	if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id)
	{
		// 1. Extract sample, report rates from client_request.
		// 2. Configure sensor HW.
		// 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
		// 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
		// 5. Save the current config information like type, sample_rate, report_rate, etc.
		desired_sample_rate = payload->desired_sample_rate;
		desired_report_rate = payload->desired_report_rate;

		SNS_INST_PRINTF(LOW, this, "publish_sensors %d", state->publish_sensors);

		if (0 == state->publish_sensors)
		{
			tcs3408r_modify_enable(state, 0xFF, 0);
			if (state->als_registry_cfg.is_dri || state->rgb_registry_cfg.is_dri || state->flicker_registry_cfg.is_dri)
			{
				tcs3408r_modify_intenab(state, 0xFF, 0);
			}
			sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
			state->timer_data_stream = NULL;
			state->timer_is_active = false;
		}
		else
		{
			    enable = PON;
				if ((state->publish_sensors&TCS3408_ALS) || (state->publish_sensors&TCS3408_RGB))
				{
					enable |= AEN;
				//	enable |= FDEN;
				}
				if (state->publish_sensors&TCS3408_FLICKER)
				{
				//    enable |= AEN;
					enable |= FDEN;
				}

				if ((state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS)) ||
					(state->rgb_registry_cfg.is_dri && (state->publish_sensors&TCS3408_RGB)) ||
					(state->flicker_registry_cfg.is_dri && (state->publish_sensors&TCS3408_FLICKER)))
				{
					intenab |= AIEN;
				}


			/* Register interrupt if some sensor is working in DRI mode */
			if ((state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS)) ||
				(state->rgb_registry_cfg.is_dri && (state->publish_sensors&TCS3408_RGB)) ||
				(state->flicker_registry_cfg.is_dri && (state->publish_sensors&TCS3408_FLICKER)))
			{
				tcs3408r_register_interrupt(this);
				tcs3408r_modify_intenab(state, (AIEN|CIEN), intenab);
				if (state->new_request_sensor&TCS3408_ALS)
				{
					tcs3408r_set_als_pers(state, 0);
				}
				
				state->new_request_sensor = 0;
				SNS_INST_PRINTF(LOW, this, "TCS3408 set_client_cfg register interrupt\n");
			}

			/* Register timer if some sensor is working in polling mode */
			if ((!state->als_registry_cfg.is_dri && (state->publish_sensors&TCS3408_ALS)) ||
				(!state->rgb_registry_cfg.is_dri && (state->publish_sensors&TCS3408_RGB)) ||
				(!state->flicker_registry_cfg.is_dri && (state->publish_sensors&TCS3408_FLICKER)))
			{
				SNS_INST_PRINTF(LOW, this, "TCS3408 set_client_cfg timer_is_active:%d\n", state->timer_is_active);
				if (!state->timer_is_active)
				{
					tcs3408r_start_polling_timer(this, sns_convert_ns_to_ticks(TCS3408_POLLING_MS * 1000 * 1000));
					state->timer_is_active = true;
				}
			}
			else
			{
				sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
				state->timer_data_stream = NULL;
				state->timer_is_active = false;
			}

			/* Enable sensors */
			if (TCS3408_ALS == payload->sensor_type) {
				if (state->first_als && (0 == (state->publish_sensors&TCS3408_RGB))) {
					tcs3408r_modify_enable(state, (AEN|FDEN|PON), PON);
					tcs3408r_set_als_gain(state, 512);
					tcs3408r_set_als_time(state, 20000);
					tcs3408r_modify_control(state, ALS_MANUAL_AZ, ALS_MANUAL_AZ);// manual auto zero
				}
				if (!state->first_als && payload->config_is_new_request) {//report a event when  als new request (but not first als)
					tcs3408r_handle_sensor_als_sample(this, state->als_info.last_event_timestamp);
				}
			}

			if (TCS3408_RGB == payload->sensor_type) {
				if (state->first_als && (0 == (state->publish_sensors&TCS3408_ALS))) {
					tcs3408r_modify_enable(state, (AEN|FDEN|PON), PON);
					tcs3408r_set_als_gain(state, 512);
					tcs3408r_set_als_time(state, 20000);
					tcs3408r_modify_control(state, ALS_MANUAL_AZ, ALS_MANUAL_AZ);// manual auto zero
				}
				if (!state->first_als && payload->config_is_new_request) {//report a event when  als new request (but not first als)
					tcs3408r_handle_sensor_rgb_sample(this, state->als_info.last_event_timestamp);
				}
			}

			if ((TCS3408_FLICKER == payload->sensor_type) && state->first_fd) {
				if (0 == (state->publish_sensors&(TCS3408_ALS|TCS3408_RGB)))
					tcs3408r_modify_enable(state, (AEN|FDEN|PON), PON);
				tcs3408r_set_fd_gain(state, 256);//set init gain to 8x whenever open fd
				state->flicker_info.samples = 0;
			}

			tcs3408r_modify_enable(state, (AEN|FDEN|PON), enable);
			SNS_INST_PRINTF(HIGH, this, "tcs3408r_inst_set_client_config: enable, R_sca %d, G_sca %d, B_sca %d, C_sca %d, W_sca %d, F_sca %d",
				(int)(state->chanl_scale[0]), (int)(state->chanl_scale[1]), (int)(state->chanl_scale[2]),
				(int)(state->chanl_scale[3]), (int)(state->chanl_scale[4]), (int)(state->chanl_scale[5]));
		}

		state->last_publish_sensors = state->publish_sensors;
		tcs3408r_send_config_event(this);
	}
	else if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
	{
	}
	else if (client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
	{
		/* All self-tests can be handled in normal mode. */
		state->island_service->api->sensor_instance_island_exit(state->island_service, this);
		tcs3408r_run_self_test(this);
		state->new_self_test_request = false;
	}

	/* Turn COM port OFF */
	state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);

	return SNS_RC_SUCCESS;
}

/* Public Data Definitions. */
sns_sensor_instance_api tcs3408r_sensor_instance_api =
{
	.struct_len        = sizeof(sns_sensor_instance_api),
	.init              = &tcs3408r_inst_init,
	.deinit            = &tcs3408r_inst_deinit,
	.notify_event      = &tcs3408r_inst_notify_event,
	.set_client_config = &tcs3408r_inst_set_client_config,
};