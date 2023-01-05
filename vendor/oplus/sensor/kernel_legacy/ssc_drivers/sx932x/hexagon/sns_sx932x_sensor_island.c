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
#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_attribute_util.h"
#include "sns_sync_com_port_service.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_printf.h"
#include "sns_island_service.h"

#include "sns_sx932x_sensor.h"
#include "sns_sx932x_hal.h"


static bool sx932x_get_decoded_sensor_request(          sns_sensor const *this,
                                                      sns_request const *in_request,
                                                      sns_std_request *decoded_request,
                                                      sns_std_sensor_config *decoded_payload)
{
    pb_istream_t stream;
    pb_simple_cb_arg arg = {.decoded_struct = decoded_payload,.fields = sns_std_sensor_config_event_fields};
    SX932X_SENSOR_LOG(LOW, this, "decoded_sensor_request");

    /* decode functions.decode */
    decoded_request->payload = (struct pb_callback_s ) {.funcs.decode = &pb_decode_simple_cb, .arg = &arg };
    stream = pb_istream_from_buffer(in_request->request, in_request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
    {
        SX932X_SENSOR_LOG(ERROR, this, "LSM decode error");
    }

    return true;
}

static void sx932x_exit_island(sns_sensor *const this)
{
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
    island_svc->api->sensor_island_exit(island_svc, this);
}

static void sx932x_set_sensor_inst_config(sns_sensor *this,
												sns_sensor_instance *instance,
												float chosen_report_rate,
												float chosen_sample_rate,
												sx932x_sensor_type  sensor_type)
{
    sns_sx932x_cfg_req new_client_config;
    sns_request config;
    UNUSED_VAR(instance);

    SX932X_SENSOR_LOG(LOW, this, "inst_config sensor type:%d, sample rate %d", sensor_type, (uint8_t)chosen_sample_rate);

    new_client_config.report_rate = chosen_report_rate;
    new_client_config.sample_rate = chosen_sample_rate;
    new_client_config.sensor_type = sensor_type;
    new_client_config.op_mode = NORMAL_MODE;
    config.message_id  = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
    config.request_len = sizeof(sns_sx932x_cfg_req);
    config.request     = &new_client_config;
    this->instance_api->set_client_config(instance, &config);
}

static void sx932x_get_sensor_sar_config(sns_sensor *this,
    											sns_sensor_instance *instance,
    											sx932x_sensor_type sensor_type,
    											float *chosen_sample_rate,
    											float *chosen_report_rate,
    											bool *sensor_sar_client_present)
{
    sx932x_instance_state *inst_state = (sx932x_instance_state*)instance->state->state;
    sns_sensor_uid suid = SAR_SUID;
    sns_request const *request;
    UNUSED_VAR(this);
    UNUSED_VAR(sensor_type);

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *sensor_sar_client_present = false;

    /*
    * Parse through existing requests and get fastest sample
    * rate and report rate requests.
    */
    for (request = instance->cb->get_client_request(instance, &suid, true); NULL != request; request = instance->cb->get_client_request(instance, &suid, false))
    {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

		SX932X_SENSOR_LOG(LOW, this, "SX932X sx932x_get_sensor_sar_config: sensor_type %d, request->message_id=%d", sensor_type, request->message_id);

        if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
            if(sx932x_get_decoded_sensor_request(this, request, &decoded_request, &decoded_payload))
            {
                float report_rate;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);
                if(decoded_request.has_batching && decoded_request.batching.batch_period > 0)
                {
                    report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                }
                else
                {
                    report_rate = *chosen_sample_rate;
                }
                *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
                *sensor_sar_client_present = true;
            }
        }
        else if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
	    {
			SX932X_SENSOR_LOG(LOW, this, "SX932X sx932x_get_sensor_sar_config: sensor_type %d (ON_CHANGE)", sensor_type);
			*chosen_sample_rate = 0;
			*chosen_report_rate = 0;
			*sensor_sar_client_present = true;
	    }
    }
    inst_state->sar_info.report_timer_hz  = *chosen_report_rate;
    inst_state->sar_info.sampling_rate_hz= *chosen_sample_rate;

    SX932X_SENSOR_LOG(LOW, this, "sar sample rate: %d, report rate: %d,  sar present %d",  (uint8_t)inst_state->sar_info.sampling_rate_hz, inst_state->sar_info.report_timer_hz, *sensor_sar_client_present);
}

static void  sx932x_mark_sensor_enable_state (sns_sensor_instance *this, sx932x_sensor_type sensor_type, bool enable)
{
    sx932x_instance_state *inst_state = (sx932x_instance_state *) this->state->state;

    /* mark the corresponding sensor as fifo info field *now only the sw fifo* */
    switch (sensor_type)
    {
        case SX9XXX_SAR:
            if (enable)
            {
                inst_state->deploy_info.publish_sensors |= SX9XXX_SAR;
                inst_state->deploy_info.enable |= SX9XXX_SAR;
            }
            else
            {
                inst_state->deploy_info.publish_sensors &= ~SX9XXX_SAR;
                inst_state->deploy_info.enable &= ~SX9XXX_SAR;
            }
            break;

        default:
            break;
    }
}

static void sx932x_reval_instance_config(sns_sensor *this, sns_sensor_instance *instance, sx932x_sensor_type sensor_type)
{
    sx932x_instance_state *inst_state = (sx932x_instance_state*)instance->state->state;
    sx932x_state *sensor_state = (sx932x_state*)this->state->state;
    float p_sample_rate = 0;
    float p_report_rate = 0;
    float chosen_sample_rate;
    float chosen_report_rate;
    bool p_sensor_client_present = false;

    SX932X_SENSOR_LOG(LOW, this, "instance_config sensor type: %d %d", sensor_state->sensor, sensor_type);

    if (sensor_type == SX9XXX_SAR)
    {
        sx932x_get_sensor_sar_config(this, instance, sensor_state->sensor, &p_sample_rate, &p_report_rate, &p_sensor_client_present);
        chosen_sample_rate = p_sample_rate;
        chosen_report_rate = p_report_rate;

        sx932x_mark_sensor_enable_state(instance, SX9XXX_SAR, p_sensor_client_present);
        SX932X_SENSOR_LOG(LOW, this, "sensor type:%d, enable sensor flag:0x%x publish sensor flag:0x%x", sensor_type, inst_state->deploy_info.enable, inst_state->deploy_info.publish_sensors);

        /* set the sensor instance configuration*/
        sx932x_set_sensor_inst_config(this, instance, chosen_report_rate, chosen_sample_rate, sensor_type);
    }

    if (!inst_state->deploy_info.enable)
    {
        sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
        sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service, this, &sensor_state->rail_config, NULL);
    }
}

static void sx932x_sensor_start_power_rail_timer(sns_sensor * const this, sns_time timeout_ticks, sx932x_power_rail_pending_state pwr_rail_pend_state)
{
    sx932x_state *state = (sx932x_state*) this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t req_len;
    uint8_t buffer[20];

    SX932X_SENSOR_LOG(LOW, this, "start power rail timer");
    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout_ticks;
    req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);
    if(req_len > 0)
    {
        sns_request timer_req = {.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,.request = buffer,.request_len = req_len};
        state->timer_stream->api->send_request(state->timer_stream, &timer_req);
        state->power_rail_pend_state = pwr_rail_pend_state;
    }
    else
    {
        SX932X_SENSOR_LOG(ERROR, this, "LSM timer req encode error");
    }
    SX932X_SENSOR_LOG(LOW, this, "start power rail timer request sent");
}

void sx932x_set_self_test_inst_config(sns_sensor *this, sns_sensor_instance *instance)
{
    sns_request config;
    config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
    config.request_len = 0;
    config.request = NULL;

    SX932X_SENSOR_LOG(LOW, this, "sx932x_set_self_test_inst_config");
    this->instance_api->set_client_config(instance, &config);
}

static bool sx932x_get_decoded_self_test_request(sns_sensor const *this, sns_request const *request,
														sns_std_request *decoded_request,
														sns_physical_sensor_test_config *test_config)
{
    pb_istream_t stream;
    pb_simple_cb_arg arg = {.decoded_struct = test_config, .fields = sns_physical_sensor_test_config_fields};

    decoded_request->payload = (struct pb_callback_s){.funcs.decode = &pb_decode_simple_cb,.arg = &arg};

    stream = pb_istream_from_buffer(request->request, request->request_len);
    if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
    {
        SX932X_SENSOR_LOG(ERROR, this, "LSM decode error");
        return false;
    }

    return true;
}

static bool sx932x_extract_self_test_info(sns_sensor *this,sns_sensor_instance *instance,struct sns_request const *new_request)
{
    sns_std_request decoded_request;
    sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
    sx932x_state *state = (sx932x_state*)this->state->state;
    sx932x_instance_state *inst_state = (sx932x_instance_state*)instance->state->state;
    sx932x_self_test_info *self_test_info;

    if(state->sensor == SX9XXX_SAR)
    {
        self_test_info = &inst_state->sar_info.test_info;
    }
    else
    {
        return false;
    }

    if(sx932x_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config))
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

static void sx932x_turn_rails_off(sns_sensor *this)
{
    sns_sensor *sensor;

    for(sensor = this->cb->get_library_sensor(this, true);
    NULL != sensor;
    sensor = this->cb->get_library_sensor(this, false))
    {
        sx932x_state *sensor_state = (sx932x_state*)sensor->state->state;
        if(sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
        {
            sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
            sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
                                                              sensor,
                                                              &sensor_state->rail_config,
                                                              NULL);
        }
    }
}

static sns_sensor_uid const* sns_see_sx932x_sar_get_sensor_uid(
    sns_sensor const * const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = SAR_SUID;
    SX932X_SENSOR_LOG(LOW, this, "sns_see_sx932x_sar_get_sensor_uid");

    return &sensor_uid;
}

static sns_sensor_instance* sns_see_sx932x_sensor_set_client_request(sns_sensor *const this,
																		struct sns_request const *exist_request,
																		struct sns_request const *new_request,
																		bool remove)
{
    sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
    sx932x_state *state = (sx932x_state*)this->state->state;
    sns_time on_timestamp;
    sns_time delta;
    bool reval_config = false;

    SX932X_SENSOR_LOG(LOW, this, "sensor_set_client_request for %d, remove=%d, exist_request=0x%x, new_request=0x%x, instance=0x%x", state->sensor, remove, exist_request, new_request,instance);

    if (remove)
    {
        if(NULL != instance)
        {
            SX932X_SENSOR_LOG(LOW, this, "removing request", exist_request->message_id);
            instance->cb->remove_client_request(instance, exist_request);
            sx932x_reval_instance_config(this, instance, state->sensor);
        }
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
        // 3.  If "flush" request:
        //     a. Perform flush on the instance.
        //     b. Return NULL.

        if (NULL == instance)
        {
            if(state->sensor == SX9XXX_SAR)
            {
                state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
            }
            else
            {
                state->rail_config.rail_vote = SNS_RAIL_OFF;
            }
            state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
																	this,
																	&state->rail_config,
																	&on_timestamp);
            delta = sns_get_system_time() - on_timestamp;
            // Use on_timestamp to determine correct Timer value.
            if(delta < sns_convert_ns_to_ticks(SX93xx_OFF_TO_IDLE_MS * 1000 * 1000))
            {
                sx932x_sensor_start_power_rail_timer(this,
                                                    sns_convert_ns_to_ticks(SX93xx_OFF_TO_IDLE_MS * 1000 * 1000) - delta,
                                                    POWER_RAIL_PENDING_SET_CLIENT_REQ);
            }
            else
            {
                reval_config = true; /*rail is already ON*/
            }

            /** create_instance() calls init() for the Sensor Instance */
            instance = this->cb->create_instance(this, sizeof(sx932x_instance_state));
            SX932X_SENSOR_LOG(LOW, this, "instace is NULL, create a instance address: 0x%x", instance);

            /* If rail is already ON then flag instance OK to configure */
            if(reval_config)
            {
                sx932x_instance_state *inst_state = (sx932x_instance_state *)instance->state->state;
                inst_state->instance_is_ready_to_configure = true;
                SX932X_SENSOR_LOG(LOW, this, "reval configure with instance is NULL case ???");
            }
        }
        else
        {
            SX932X_SENSOR_LOG(LOW, this, "instace is not NULL new_request->message_id=%d", new_request->message_id);
            if ((NULL != exist_request) && (NULL != new_request) && (new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ))
            {
                sx932x_reval_instance_config(this, instance, state->sensor);
                sns_sensor_util_send_flush_event(&state->my_suid, instance);
                return instance;
            }
            else
            {
                reval_config = true;
                /** An existing client is changing request*/
                if ((NULL != exist_request) && (NULL != new_request))
                {
                    instance->cb->remove_client_request(instance, exist_request);
                }
                /** A new client sent new_request*/
                else if (NULL != new_request)
                {
                // No-op. new_request will be added to requests list below.
                }
            }
        }

        /** Add the new request to list of client_requests.*/
        if (NULL != instance)
        {
            sx932x_instance_state *inst_state = (sx932x_instance_state*)instance->state->state;
            if (new_request != NULL)
            {
                SX932X_SENSOR_LOG(LOW, this, "new request from sensor %d , message id:%d add to the client request list", state->sensor, new_request->message_id);
                SX932X_SENSOR_LOG(LOW, this, "instace2 address is 0x%x", instance);
                instance->cb->add_client_request(instance, new_request);

                if (new_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
                {
                    if (sx932x_extract_self_test_info(this, instance, new_request))
                    {
                        inst_state->new_self_test_request = true;
                    }
                }
            }

            if (reval_config 
                && inst_state->instance_is_ready_to_configure 
                && (NULL == exist_request || SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG != new_request->message_id)
                )
            {
                SX932X_SENSOR_LOG(LOW, this, "try to configure");
                sx932x_reval_instance_config(this, instance, state->sensor);
                if(inst_state->new_self_test_request)
                {
                    sx932x_set_self_test_inst_config(this, instance);
                }
            }
        }
    }

    if ((NULL != instance) && (NULL == instance->cb->get_client_request(instance,&(sns_sensor_uid)SAR_SUID, true)))
    {
        this->cb->remove_instance(instance);
        sx932x_turn_rails_off(this);
    }

    return instance;
}

static sns_rc sns_see_sx932x_sensor_notify_event(sns_sensor * const this)
{
    sx932x_state *state = (sx932x_state*) this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    sns_time on_timestamp;
    uint8_t buffer[1];
    sns_rc rv = SNS_RC_SUCCESS;
    sns_sensor_event *event;

    SX932X_SENSOR_LOG(LOW, this, "sns_see_sx932x_sensor_notify_event from sensor:%d> ", state->sensor);

    if (state->fw_stream)
    {
        SX932X_SENSOR_LOG(LOW, this, ">>fw_stream<<");
        if ((0 == sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
            || (0 == sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid)))
            || (0 == sns_memcmp(&state->irq_suid, &((sns_sensor_uid){{0}}), sizeof(state->irq_suid))))
        {
			/* All SUID events can be handled in normal mode. */
            sx932x_exit_island(this);

            sx932x_sensor_process_suid_events(this);
            /* request to registry sensor*/
            if (0 != sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid)))
            {
                if(state->reg_data_stream == NULL)
                {
                    stream_svc->api->create_sensor_stream(stream_svc,this, state->reg_suid, &state->reg_data_stream);
                    sx932x_sensor_send_registry_request(this, SX932x_PLATFORM_CONFIG);
                    sx932x_sensor_send_registry_request(this, SX932x_PLATFORM_PLACEMENT);
                    sx932x_sensor_send_registry_request(this, SX932x_CONFIG_SAR);
                }
            }
        }
    }

    /**----------------------Handle a Timer Sensor event.-------------------*/
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
                    if (state->power_rail_pend_state == POWER_RAIL_PENDING_INIT)
                    {
						/* Initial HW discovery is OK to run in normal mode. */
					    sx932x_exit_island(this);

                        /**-------------------Read and Confirm WHO-AM-I------------------------*/
                        buffer[0] = 0xFF;
                        rv = sx932x_get_who_am_i(state->scp_service,state->com_port_info.port_handle, &buffer[0]);
                        SX932X_SENSOR_LOG(ERROR, this, "sx932x who am i is 0x%x", buffer[0]);
                        if (rv == SNS_RC_SUCCESS && ((buffer[0] == SX932x_WHOAMI_VALUE) || (buffer[0] == SX932x_WHOAMI_VALUE2)))
                        {
                            /* Reset Sensor */
                            rv = sx932x_device_sw_reset(state->scp_service,state->com_port_info.port_handle);
                        	state->hw_is_present = true;
                        }
                        state->who_am_i = buffer[0];

                        /**------------------Power Down and Close COM Port--------------------*/
                        state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
                        state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
                        state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);

                        /**----------------------Turn Power Rail OFF--------------------------*/
                        state->rail_config.rail_vote = SNS_RAIL_OFF;
                        state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,this,&state->rail_config,NULL);
                        if (state->hw_is_present)
                        {
                            sx932x_publish_available(this);
                            SX932X_SENSOR_LOG(LOW, this, "sensor:%d initialize finished", state->sensor);
                        }
                        else
                        {
                            rv = SNS_RC_INVALID_STATE;
                            SX932X_SENSOR_LOG(ERROR, this, "sx932x HW absent");
                        }
                        state->power_rail_pend_state = POWER_RAIL_PENDING_NONE;
                    }
                    else if(state->power_rail_pend_state == POWER_RAIL_PENDING_SET_CLIENT_REQ)
                    {
                        sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
                        if (NULL != instance)
                        {
                            sx932x_instance_state *inst_state = (sx932x_instance_state *)instance->state->state;
                            inst_state->instance_is_ready_to_configure = true;
                            sx932x_reval_instance_config(this, instance, state->sensor);
                            if (inst_state->new_self_test_request)
                            {
                                sx932x_set_self_test_inst_config(this, instance);
                            }
                        }
                        state->power_rail_pend_state  = POWER_RAIL_PENDING_NONE;
                    }
                }
                else
                {
                    SX932X_SENSOR_LOG(ERROR, this, "pb_decode error");
                }
            }
            event = state->timer_stream->api->get_next_input(state->timer_stream);
        }
    }

    if(NULL != state->reg_data_stream)
    {
        SX932X_SENSOR_LOG(LOW, this, ">>reg_data_stream<<");
        event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
        while(NULL != event)
        {
			/* Initial HW discovery is OK to run in normal mode. */
			sx932x_exit_island(this);

            sx932x_sensor_process_registry_event(this, event);
            if(state->registry_cfg_received && state->registry_placement_received)
            {
                sx932x_publish_registry_attributes(this);
            }
            event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
        }
    }

    if((0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
       && state->registry_pf_cfg_received
       && state->registry_cfg_received
       && state->registry_placement_received)
    {
        state->registry_pf_cfg_received = false;

        /***************Register and Open COM Port************************************/
        if (NULL == state->com_port_info.port_handle)
        {
            rv = state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config, &state->com_port_info.port_handle);
            SX932X_SENSOR_LOG(LOW, this, "register com port success? rv=%d", rv);
            if(rv == SNS_RC_SUCCESS)
            {
                rv = state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
                SX932X_SENSOR_LOG(LOW, this, "open com port success> rv=%d, handle=%p", rv,
                state->com_port_info.port_handle);
            }
        }

        /****************************Register Power Rails*****************************/
        if ((0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
            && (NULL == state->pwr_rail_service)
            && (rv == SNS_RC_SUCCESS))
        {
            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service = (sns_pwr_rail_service*)service_mgr->get_service(service_mgr,SNS_POWER_RAIL_SERVICE);

            /* ************************register power rails ******************************/
            state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,&state->rail_config);
            SX932X_SENSOR_LOG(LOW, this, "register power rails: rail vote:%d, num of rails:%d [1]%s [2]%s rv=%d",
                        state->rail_config.rail_vote,
                        state->rail_config.num_of_rails,
                        state->rail_config.rails[0].name,
                        state->rail_config.rails[1].name,
                        rv);

            /*************************Turn Power Rails ON*******************************/
            if (state->sensor == SX9XXX_SAR)
            {
                state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
            } else
            {
                state->rail_config.rail_vote = SNS_RAIL_OFF;
            }
            rv = state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,this,&state->rail_config,&on_timestamp);
            SX932X_SENSOR_LOG(LOW, this, "vote power rails value:%d rv=%d",state->rail_config.rail_vote, rv);

            /************************Create a Timer stream for Power Rail ON timeout.*********/
            if (NULL == state->timer_stream)
            {
                SX932X_SENSOR_LOG(LOW, this, "here create the timer stream");
                stream_svc->api->create_sensor_stream(stream_svc,this,state->timer_suid,&state->timer_stream);
                if (NULL != state->timer_stream)
                {
                    sx932x_sensor_start_power_rail_timer(this,sns_convert_ns_to_ticks(SX93xx_OFF_TO_IDLE_MS * 1000 * 1000),	POWER_RAIL_PENDING_INIT);
                }
            }
        }
    }

    return rv;
}

sns_sensor_api sx932x_sar_sensor_api =
{
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_see_sx932x_sar_init,
    .deinit = &sns_see_sx932x_sar_deinit,
    .get_sensor_uid = &sns_see_sx932x_sar_get_sensor_uid,
    .set_client_request = &sns_see_sx932x_sensor_set_client_request,
    .notify_event = &sns_see_sx932x_sensor_notify_event,
};

