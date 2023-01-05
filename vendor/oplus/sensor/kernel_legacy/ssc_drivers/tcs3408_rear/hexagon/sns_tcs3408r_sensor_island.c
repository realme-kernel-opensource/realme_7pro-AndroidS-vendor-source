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

#include <string.h>
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_motion_detect.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_printf.h"

#include "sns_tcs3408r_hal.h"
#include "sns_tcs3408r_sensor.h"

static sns_sensor_uid const* tcs3408r_als_get_sensor_uid(sns_sensor const *const this)
{
	UNUSED_VAR(this);
	static const sns_sensor_uid sensor_uid = TCS3408_ALS_SUID;

	return &sensor_uid;
}

static sns_sensor_uid const* tcs3408r_rgb_get_sensor_uid(sns_sensor const *const this)
{
	UNUSED_VAR(this);
	static const sns_sensor_uid sensor_uid = TCS3408_RGB_SUID;

	return &sensor_uid;
}

static sns_sensor_uid const* tcs3408r_flicker_get_sensor_uid(sns_sensor const *const this)
{
	UNUSED_VAR(this);
	static const sns_sensor_uid sensor_uid = TCS3408_FLICKER_SUID;

	return &sensor_uid;
}
/**
 * Returns decoded request message for type
 * sns_std_sensor_config.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool tcs3408r_get_decoded_std_request(sns_sensor const *this, sns_request const *in_request,
									sns_std_request *decoded_request,
									sns_std_sensor_config *decoded_payload)
{
	pb_istream_t stream;
	pb_simple_cb_arg arg =
		{.decoded_struct = decoded_payload, .fields = sns_std_sensor_config_fields};
	decoded_request->payload = (struct pb_callback_s)
		{.funcs.decode = &pb_decode_simple_cb, .arg = &arg};
	stream = pb_istream_from_buffer(in_request->request,
		in_request->request_len);
	if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
	{
		SNS_PRINTF(ERROR, this, "TCS3408 get_decoded_std_request decode error");
		return false;
	}
	return true;
}

/**
 * Parses through all starndard streaming requests and deduces
 * best HW config for the inertial sensor type.
 *
 * @param[i] this                 Sensor reference
 * @param[i] instance             Instance reference
 * @param[i] sensor_type          sensor type
 * @param[o] chosen_sample_rate   chosen sample rate in Hz
 * @param[o] chosen_report_rate   chosen report rate in Hz 
 * @param[o] client_present        
 * @param[o] num_clients          
 */
static void tcs3408r_get_device_config(sns_sensor *this,
									sns_sensor_instance *instance,
									tcs3408_sensor_type sensor_type,
									float *chosen_sample_rate,
									float *chosen_report_rate,
									bool *client_present,
									int32_t *num_clients)
{
	UNUSED_VAR(this);
	sns_sensor_uid suid;
	sns_request const *request;

	if (TCS3408_ALS == sensor_type)
	{
		sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)TCS3408_ALS_SUID), sizeof(sns_sensor_uid));
	}
	else if (TCS3408_RGB == sensor_type)
	{
		sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)TCS3408_RGB_SUID), sizeof(sns_sensor_uid));
	}
	else if (TCS3408_FLICKER== sensor_type)
	{
		sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)TCS3408_FLICKER_SUID), sizeof(sns_sensor_uid));
	}

	*chosen_report_rate = 0.0f;
	*chosen_sample_rate = 0.0f;
	*client_present = false;
	*num_clients = 0;

	/* Parse through existing requests and get fastest sample
	 *  rate and report rate requests. */
	for (request = instance->cb->get_client_request(instance, &suid, true);
		NULL != request;
		request = instance->cb->get_client_request(instance, &suid, false))
	{
		sns_std_request decoded_request;
		sns_std_sensor_config decoded_payload = {0};
		SNS_PRINTF(LOW, this, "TCS3408 tcs3408r_get_device_config message_id=%d\n", request->message_id);

		if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == request->message_id)
		{
			if (tcs3408r_get_decoded_std_request(this, request, &decoded_request, &decoded_payload))
			{
				*chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);
				*chosen_report_rate = *chosen_sample_rate;
				*client_present = true;
				(*num_clients)++;
			}
		}
	    else if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == request->message_id)
	    {
			SNS_PRINTF(MED, this, "TCS3408 get_device_config: sensor_type %d (ON_CHANGE)", sensor_type);
			*chosen_sample_rate = 10.0f;
			*chosen_report_rate = 10.0f;
			*client_present = true;
			(*num_clients)++;
	    }
	}
}

/**
 * Set standard streaming config for the instance.
 *
 * @param[i] this        Sensor reference
 * @param[i] instance    Sensor Instance reference
 * @param[i] chosen_report_rate   chosen report rate in Hz
 * @param[i] chosen_sample_rate   chosen sample rate in Hz 
 * @param[i] message_id           input message ID 
 *
 * @return none
 */
static void tcs3408r_set_inst_config(sns_sensor *this,
									sns_sensor_instance *instance,
									float chosen_report_rate,
									float chosen_sample_rate,
									tcs3408_sensor_type sensor_type,
									uint32_t message_id)
{
	tcs3408_state *state = (tcs3408_state *)this->state->state;
	sns_tcs3408_req new_client_config;
	sns_request config;

	new_client_config.desired_report_rate = chosen_report_rate;
	new_client_config.desired_sample_rate = chosen_sample_rate;
	new_client_config.sensor_type = sensor_type;
	new_client_config.config_is_new_request = state->is_new_request;
	state->is_new_request = false;

	config.message_id = message_id;
	config.request_len = sizeof(sns_tcs3408_req);
	config.request = &new_client_config;

	this->instance_api->set_client_config(instance, &config);
}

void tcs3408r_reval_instance_config(sns_sensor *this,
									sns_sensor_instance *instance,
									tcs3408_sensor_type sensor_type)
{
	/**
	* 1. Get best ALS Config.
	* 2. Decide best Instance Config based on above outputs.
	*/
	float chosen_sample_rate = 0.0f;
	float chosen_report_rate = 0.0f;
	float sample_rate = 0.0f;
	float report_rate = 0.0f;
	bool client_present = false;
	int32_t num_clients;
	tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;

	tcs3408r_get_device_config(this, instance, TCS3408_ALS, &sample_rate,
		&report_rate, &client_present, &num_clients);

	SNS_PRINTF(LOW, this, "TCS3408 ALS: rr %u sr %u num_clients %d",
		(uint32_t)report_rate, (uint32_t)sample_rate, num_clients);

	if (client_present)
	{
		chosen_sample_rate = sample_rate;
		chosen_report_rate = sample_rate;

		/* First ALS is special */
		if (0 == (inst_state->publish_sensors&(TCS3408_ALS|TCS3408_RGB)))
		{
			inst_state->first_als = true;
		}
		inst_state->publish_sensors |= TCS3408_ALS;
	}
	else
	{
		inst_state->publish_sensors &= ~TCS3408_ALS;
	}


	tcs3408r_get_device_config(this, instance, TCS3408_RGB, &sample_rate,
		&report_rate, &client_present, &num_clients);

	SNS_PRINTF(LOW, this, "TCS3408 RGB: rr %u sr %u num_clients %d",
		(uint32_t)report_rate, (uint32_t)sample_rate, num_clients);

	if (client_present)
	{
		chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
		chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

		/* First RGB is special */
		if (0 == (inst_state->publish_sensors&(TCS3408_ALS|TCS3408_RGB)))
		{
			inst_state->first_als = true;
		}
		inst_state->publish_sensors |= TCS3408_RGB;
	}
	else
	{
		inst_state->publish_sensors &= ~TCS3408_RGB;
	}

	if (0 == (inst_state->publish_sensors&(TCS3408_ALS|TCS3408_RGB)))
		inst_state->first_als = false;

	tcs3408r_get_device_config(this, instance, TCS3408_FLICKER, &sample_rate,
		&report_rate, &client_present, &num_clients);

	SNS_PRINTF(LOW, this, "TCS3408 FLICKER: rr %u sr %u num_clients %d",
		(uint32_t)report_rate, (uint32_t)sample_rate, num_clients);

	if (client_present)
	{
		chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
		chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

		/* First flicker is special */
		if (0 == (inst_state->publish_sensors&TCS3408_FLICKER))
		{
			inst_state->first_fd = true;
		}
		inst_state->publish_sensors |= TCS3408_FLICKER;
	}
	else
	{
		inst_state->publish_sensors &= ~TCS3408_FLICKER;
		inst_state->first_fd = false;
	}
	

	tcs3408r_set_inst_config(this,
		instance,
		chosen_report_rate,
		chosen_sample_rate,
		sensor_type,
		SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
}

/**
 * Sets instance config to run a self test.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 *
 * @return none
 */
void tcs3408r_set_self_test_inst_config(sns_sensor *this,
									sns_sensor_instance *instance)
{
	sns_request config;

	config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
	config.request_len = 0;
	config.request = NULL;

	this->instance_api->set_client_config(instance, &config);
}

void tcs3408r_start_power_rail_timer(sns_sensor *const this,
									sns_time timeout_ticks,
									tcs3408_power_rail_pending_state pwr_rail_pend_state)
{
	tcs3408_state *state = (tcs3408_state*)this->state->state;

	sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
	size_t req_len;
	uint8_t buffer[20];
	sns_memset(buffer, 0, sizeof(buffer));
	req_payload.is_periodic = false;
	req_payload.start_time = sns_get_system_time();
	req_payload.timeout_period = timeout_ticks;

	if (NULL == state->timer_stream)
	{
		sns_service_manager *smgr = this->cb->get_service_manager(this);
		sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
		stream_svc->api->create_sensor_stream(stream_svc, this, state->common.timer_suid, &state->timer_stream);
	}

	req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);
	if (req_len > 0 && NULL != state->timer_stream)
	{
		sns_request timer_req =
			{.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
			.request = buffer, .request_len = req_len};
		state->timer_stream->api->send_request(state->timer_stream, &timer_req);
		state->power_rail_pend_state = pwr_rail_pend_state;
	}
	else
	{
		SNS_PRINTF(ERROR, this, "TCS3408 timer req encode error");
	}
}

sns_rc tcs3408r_sensor_notify_event(sns_sensor *const this)
{
	tcs3408_state *state = (tcs3408_state*)this->state->state;
	sns_rc rv = SNS_RC_SUCCESS;
	sns_sensor_event *event;

	SNS_PRINTF(LOW, this, "TCS3408 tcs3408r_sensor_notify_event");

	if (state->fw_stream)
	{
		if (sns_sensor_uid_compare(&state->common.irq_suid, &((sns_sensor_uid){{0}}))
			|| sns_sensor_uid_compare(&state->common.timer_suid, &((sns_sensor_uid){{0}}))
			|| sns_sensor_uid_compare(&state->common.reg_suid, &((sns_sensor_uid){{0}})))
		{
			/* All SUID events can be handled in normal mode. */
			state->island_service->api->sensor_island_exit(state->island_service, this);

			tcs3408r_process_suid_events(this);

			if (!sns_sensor_uid_compare(&state->common.reg_suid, &((sns_sensor_uid){{0}})))
			{
				tcs3408r_request_registry(this);
			}
		}

		/* Check if the SUID sensor stream can be removed. */
		if (!sns_sensor_uid_compare(&state->common.reg_suid, &((sns_sensor_uid){{0}})))
		{
			/* Non-als sensors only request for registry SUID. */
			if (TCS3408_RGB != state->sensor)
			{
		  		sns_sensor_util_remove_sensor_stream(this, &state->fw_stream);
			}

			if (!sns_sensor_uid_compare(&state->common.irq_suid, &((sns_sensor_uid){{0}}))
				&& !sns_sensor_uid_compare(&state->common.timer_suid, &((sns_sensor_uid){{0}})))
			{
				sns_sensor_util_remove_sensor_stream(this, &state->fw_stream);
			}
		}
	}

	/* Handle a Timer Sensor event */
	if (NULL != state->timer_stream)
	{
		event = state->timer_stream->api->peek_input(state->timer_stream);
		while (NULL != event)
		{
			pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
			sns_timer_sensor_event timer_event;

			if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
			{
				if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
				{
					if (TCS3408_POWER_RAIL_PENDING_INIT == state->power_rail_pend_state)
					{
						/* Initial HW discovery is OK to run in normal mode. */
						state->island_service->api->sensor_island_exit(state->island_service, this);

						state->common.hw_is_present = tcs3408r_discover_hw(this);

						if (state->common.hw_is_present)
						{
							tcs3408r_publish_available(this);
							tcs3408r_update_sibling_sensors(this);
						}
						else
						{
							rv = SNS_RC_INVALID_LIBRARY_STATE;
							SNS_PRINTF(LOW, this, "TCS3408 HW absent");
						}
						state->power_rail_pend_state = TCS3408_POWER_RAIL_PENDING_NONE;
					}
					else if (TCS3408_POWER_RAIL_PENDING_SET_CLIENT_REQ == state->power_rail_pend_state)
					{
						sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
						if (NULL != instance)
						{
							tcs3408_instance_state *inst_state = (tcs3408_instance_state*) instance->state->state;
							inst_state->instance_is_ready_to_configure = true;
							inst_state->new_request_sensor = state->sensor;
							tcs3408r_reval_instance_config(this, instance, state->sensor);
							if (inst_state->new_self_test_request)
							{
								tcs3408r_set_self_test_inst_config(this, instance);
								SNS_PRINTF(LOW, this, "TCS3408 notify event tcs3408r_set_self_test_inst_config\n");
							}
						}
						state->power_rail_pend_state = TCS3408_POWER_RAIL_PENDING_NONE;
					}
				}
				else
				{
					SNS_PRINTF(ERROR, this, "TCS3408 pb_decode error");
				}
			}
			event = state->timer_stream->api->get_next_input(state->timer_stream);
		}
		/* Free up timer stream if not needed anymore */
		if (TCS3408_POWER_RAIL_PENDING_NONE == state->power_rail_pend_state)
		{
			sns_sensor_util_remove_sensor_stream(this, &state->timer_stream);
		}
	}

	if (NULL != state->reg_data_stream)
	{
		event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
		while (NULL != event)
		{
			/* All registry events can be handled in normal mode. */
			state->island_service->api->sensor_island_exit(state->island_service, this);
			tcs3408r_sensor_process_registry_event(this, event);

			event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
		}
	}

	if (!sns_sensor_uid_compare(&state->common.timer_suid, &((sns_sensor_uid){{0}})) &&
		state->common.registry_pf_cfg_received && state->common.registry_coefficient_received &&
		!state->common.start_hw_detect)
	{
		/* Initial HW detection sequence is OK to run in normal mode. */
		state->island_service->api->sensor_island_exit(state->island_service, this);

		tcs3408r_start_hw_detect_sequence(this);
	}
	return rv;
}

/**
 * Turns power rails off
 *
 * @paramp[i] this   Sensor reference
 *
 * @return none
 */
static void tcs3408r_turn_rails_off(sns_sensor *this)
{
	sns_sensor *sensor;

	for (sensor = this->cb->get_library_sensor(this, true);
		NULL != sensor;
		sensor = this->cb->get_library_sensor(this, false))
	{
		tcs3408_state *sensor_state = (tcs3408_state*)sensor->state->state;
		if (SNS_RAIL_OFF != sensor_state->common.rail_config.rail_vote)
		{
			sensor_state->common.rail_config.rail_vote = SNS_RAIL_OFF;
			sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
				sensor,
				&sensor_state->common.rail_config,
				NULL);
		}
	}
}

/**
 * Decodes self test requests.
 *
 * @param[i] this              Sensor reference
 * @param[i] request           Encoded input request
 * @param[o] decoded_request   Decoded standard request
 * @param[o] test_config       decoded self test request
 *
 * @return bool True if decoding is successfull else false.
 */
static bool tcs3408r_get_decoded_self_test_request(sns_sensor const *this,
									sns_request const *request,
									sns_std_request *decoded_request,
									sns_physical_sensor_test_config *test_config)
{
	pb_istream_t stream;
	pb_simple_cb_arg arg =
		{.decoded_struct = test_config,
		.fields = sns_physical_sensor_test_config_fields};
	decoded_request->payload = (struct pb_callback_s)
		{.funcs.decode = &pb_decode_simple_cb, .arg = &arg};
	stream = pb_istream_from_buffer(request->request, request->request_len);
	if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
	{
		SNS_PRINTF(ERROR, this, "TCS3408 get_decoded_self_test_request decode error");
		return false;
	}
	return true;
}

/**
 * Decodes a physical sensor self test request and updates
 * instance state with request info.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 * @param[i] new_request Encoded request
 *
 * @return True if request is valid else false
 */
static bool tcs3408r_extract_self_test_info(sns_sensor *this,
									sns_sensor_instance *instance,
									struct sns_request const *new_request)
{
	sns_std_request decoded_request;
	sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
	tcs3408_state *state = (tcs3408_state*)this->state->state;
	tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;
	tcs3408_self_test_info *self_test_info;

	if (TCS3408_ALS == state->sensor)
	{
		self_test_info = &inst_state->als_info.test_info;
	}
	else if (TCS3408_RGB == state->sensor)
	{
		self_test_info = &inst_state->rgb_info.test_info;
	}
	else if (TCS3408_FLICKER == state->sensor)
	{
		self_test_info = &inst_state->flicker_info.test_info;
	}
	else
	{
		return false;
	}

	if (tcs3408r_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config))
	{
		self_test_info->test_type = test_config.test_type;
		self_test_info->test_client_present = true;
		return true;
	}
	else
	{
		return false;
	}
}

sns_sensor_instance* tcs3408r_set_client_request(sns_sensor *const this,
									struct sns_request const *exist_request,
									struct sns_request const *new_request,
									bool remove)
{
	sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
	tcs3408_state *state = (tcs3408_state*)this->state->state;
	sns_time on_timestamp;
	sns_time delta;
	bool reval_config = false;

	SNS_PRINTF(LOW, this, "TCS3408 set_client_req 0x%X  0x%X  %d", exist_request, new_request, remove);

	if (remove)
	{
		if (NULL != instance)
		{
			instance->cb->remove_client_request(instance, exist_request);
			SNS_PRINTF(LOW, this, "TCS3408 set_client_req message_id %d\n", exist_request->message_id);
			if (SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG != exist_request->message_id)
			{
				tcs3408r_reval_instance_config(this, instance, state->sensor);
			}
		}
	}
	else
	{
		if ((NULL == instance) && (new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ))
		{
			// no op
			SNS_PRINTF(HIGH, this, "ams_set_client_request: SNS_STD_MSGID_SNS_STD_FLUSH_REQ, no-op for TCS3408_Rear");
		}
		else
		{
			// 1. If new request then:
			//     a. Power ON rails.
			//     b. Power ON COM port - Instance must handle COM port power.
			//     c. Create new instance.
			//     d. Re-evaluate existing requests and choose appropriate instance config.
			//     e. set_client_config for this instance.
			//     f. Add new_request to list of requests handled by the Instance.
			//     g. Power OFF COM port if not needed- Instance must handle COM port power.
			//     h. Return the Instance.
			// 2. If there is an Instance already present:
			//     a. Add new_request to list of requests handled by the Instance.
			//     b. Remove exist_request from list of requests handled by the Instance.
			//     c. Re-evaluate existing requests and choose appropriate Instance config.
			//     d. set_client_config for the Instance if not the same as current config.
			//     e. publish the updated config.
			//     f. Return the Instance.

			if (NULL == instance)
			{
				state->common.rail_config.rail_vote = SNS_RAIL_ON_NPM;

				state->pwr_rail_service->api->sns_vote_power_rail_update(
					state->pwr_rail_service,
					this,
					&state->common.rail_config,
					&on_timestamp);

				delta = sns_get_system_time() - on_timestamp;

				/* Use on_timestamp to determine correct Timer value. */
				if (delta < sns_convert_ns_to_ticks(TCS3408_OFF_TO_IDLE_MS*1000*1000))
				{
					tcs3408r_start_power_rail_timer(this,
						sns_convert_ns_to_ticks(TCS3408_OFF_TO_IDLE_MS*1000*1000) - delta,
						TCS3408_POWER_RAIL_PENDING_SET_CLIENT_REQ);
				}
				else
				{
					/* Rail is already ON */
					reval_config = true;
				}

				/* Create_instance() calls init() for the Sensor Instance */
				instance = this->cb->create_instance(this, sizeof(tcs3408_instance_state));

				/* If rail is already ON then flag instance OK to configure */
				if (reval_config)
				{
					tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;

					inst_state->instance_is_ready_to_configure = true;
				}
			}
			else
			{
				if (NULL != exist_request &&
					NULL != new_request &&
					SNS_STD_MSGID_SNS_STD_FLUSH_REQ == new_request->message_id)
				{
					sns_service_manager *mgr = instance->cb->get_service_manager(instance);
					sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
					sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);

					if (NULL != event)
					{
						event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
						event->event_len = 0;
						event->timestamp = sns_get_system_time();

						e_service->api->publish_event(e_service, instance, event, &state->my_suid);
					}
					return instance;
				}
				else
				{
					reval_config = true;

					/* An existing client is changing request*/
					if ((NULL != exist_request) && (NULL != new_request))
					{
						instance->cb->remove_client_request(instance, exist_request);
					}
					/* A new client sent new_request*/
					else if (NULL != new_request)
					{
						if (TCS3408_RGB == state->sensor)
							state->is_new_request = true;
						SNS_PRINTF(ERROR, this, "this client is new");
					}
				}
			}

			/* Add the new request to list of client_requests.*/
			if (NULL != instance)
			{
				tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;
				if (NULL != new_request)
				{
					instance->cb->add_client_request(instance, new_request);

					if (SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG == new_request->message_id)
					{
						if (tcs3408r_extract_self_test_info(this, instance, new_request))
						{
							inst_state->new_self_test_request = true;
						}
						SNS_PRINTF(LOW, this, "TCS3408 SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG %d\n",
							inst_state->new_self_test_request);
					}
				}
				if (reval_config && inst_state->instance_is_ready_to_configure)
				{
					inst_state->new_request_sensor = state->sensor;
					tcs3408r_reval_instance_config(this, instance, state->sensor);
					if (inst_state->new_self_test_request)
					{
						tcs3408r_set_self_test_inst_config(this, instance);
						SNS_PRINTF(LOW, this, "TCS3408 set_client_requet tcs3408r_set_self_test_inst_config\n");
					}
				}
			}
		}
	}

	if (NULL != instance)
	{
		tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;
		if (inst_state->update_fac_cal_in_registry)
		{
			/* Registry write can be handled in normal mode. */
			state->island_service->api->sensor_island_exit(state->island_service, this);
			sns_sensor *iter;
			tcs3408_instance_state *inst_state = (tcs3408_instance_state*)instance->state->state;
			for (iter = this->cb->get_library_sensor(this, true);
				NULL != iter;
				iter = this->cb->get_library_sensor(this, false))
			{
				tcs3408_state *sensor_state = (tcs3408_state*)iter->state->state;

				sensor_state->common.fac_cal_lux_scale       = inst_state->lux_scale;
				sensor_state->common.fac_cal_lux_bias        = inst_state->lux_bias;
				sensor_state->common.als_fac_cal_version[0]  = inst_state->als_fac_cal_version[0];
				sensor_state->common.als_fac_cal_version[1]  = inst_state->als_fac_cal_version[1];

				sensor_state->common.fac_cal_rgb_scale		 = inst_state->rgb_scale;
				sensor_state->common.fac_cal_rgb_bias		 = inst_state->rgb_bias;
				sensor_state->common.rgb_fac_cal_version[0]  = inst_state->rgb_fac_cal_version[0];
				sensor_state->common.rgb_fac_cal_version[1]  = inst_state->rgb_fac_cal_version[1];

				sensor_state->common.double_cali			  = inst_state->double_cali;
				sensor_state->common.channel_scale[0] 		  = inst_state->chanl_scale[0];
				sensor_state->common.channel_scale[1]		  = inst_state->chanl_scale[1];
				sensor_state->common.channel_scale[2] 		  = inst_state->chanl_scale[2];
				sensor_state->common.channel_scale[3] 		  = inst_state->chanl_scale[3];
				sensor_state->common.channel_scale[4] 		  = inst_state->chanl_scale[4];
				sensor_state->common.channel_scale[5] 		  = inst_state->chanl_scale[5];
				sensor_state->common.channel_cali_version[0]  = inst_state->chanl_cali_ver[0];
				sensor_state->common.channel_cali_version[1]  = inst_state->chanl_cali_ver[1];
				sensor_state->common.channel_cali_version[2]  = inst_state->chanl_cali_ver[2];
				sensor_state->common.channel_cali_version[3]  = inst_state->chanl_cali_ver[3];
				sensor_state->common.channel_cali_version[4]  = inst_state->chanl_cali_ver[4];
				sensor_state->common.channel_cali_version[5]  = inst_state->chanl_cali_ver[5];

				sensor_state->common.fac_cal_flicker_scale       = inst_state->flicker_scale;
				sensor_state->common.fac_cal_flicker_bias        = inst_state->flicker_bias;
				sensor_state->common.flicker_fac_cal_version[0]  = inst_state->flicker_fac_cal_version[0];
				sensor_state->common.flicker_fac_cal_version[1]  = inst_state->flicker_fac_cal_version[1];
			}

			tcs3408r_write_calibration_to_registry(this);
			inst_state->update_fac_cal_in_registry = false;
			SNS_PRINTF(LOW, this, "TCS3408 set_client_requet update_registry\n");
		}
		if (NULL == instance->cb->get_client_request(instance, &(sns_sensor_uid)TCS3408_RGB_SUID, true) &&
			NULL == instance->cb->get_client_request(instance, &(sns_sensor_uid)TCS3408_FLICKER_SUID, true))
		{
			this->cb->remove_instance(instance);
			tcs3408r_turn_rails_off(this);
		}
	}

	return instance;
}

sns_sensor_api tcs3408r_als_sensor_api =
{
	.struct_len         = sizeof(sns_sensor_api),
	.init               = &tcs3408r_als_init,
	.deinit             = &tcs3408r_als_deinit,
	.get_sensor_uid     = &tcs3408r_als_get_sensor_uid,
	.notify_event		= &tcs3408r_sensor_notify_event,
	.set_client_request = &tcs3408r_set_client_request,
};

sns_sensor_api tcs3408r_rgb_sensor_api =
{
	.struct_len         = sizeof(sns_sensor_api),
	.init               = &tcs3408r_rgb_init,
	.deinit             = &tcs3408r_rgb_deinit,
	.get_sensor_uid     = &tcs3408r_rgb_get_sensor_uid,
	.notify_event		= &tcs3408r_sensor_notify_event,
	.set_client_request = &tcs3408r_set_client_request,
};

sns_sensor_api tcs3408r_flicker_sensor_api =
{
	.struct_len         = sizeof(sns_sensor_api),
	.init               = &tcs3408r_flicker_init,
	.deinit             = &tcs3408r_flicker_deinit,
	.get_sensor_uid     = &tcs3408r_flicker_get_sensor_uid,
	.notify_event		= &tcs3408r_sensor_notify_event,
	.set_client_request = &tcs3408r_set_client_request,
};