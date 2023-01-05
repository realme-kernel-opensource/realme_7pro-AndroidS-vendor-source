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
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"

#include "sns_sx932x_sensor.h"
#include "sns_sx932x_sensor_instance.h"

void sx932x_publish_registry_attributes(sns_sensor *const this)
{
  sx932x_state *state = (sx932x_state*)this->state->state;
  {
	sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
	value.has_boolean = true;
	value.boolean = state->registry_cfg.is_dri;
	sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_DRI,
		&value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = state->registry_cfg.is_dri ?
			SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE : SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
      values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->registry_cfg.is_dri;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->registry_cfg.hw_id;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    for(uint8_t i = 0; i < 12; i++)
    {
      values[i].has_flt = true;
      values[i].flt = state->placement[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->registry_pf_cfg.rigid_body_type;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
}

void sx932x_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;

    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

void sns_see_sx932x_send_suid_req(sns_sensor *this, char * const data_type, uint32_t data_type_len)
{
    uint8_t buffer[50];
    sns_memset(buffer, 0, sizeof(buffer));
    sx932x_state *state = (sx932x_state*)this->state->state;
    sns_service_manager *manager = this->cb->get_service_manager(this);
    sns_stream_service *stream_service =  (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);
    size_t encoded_len;
    pb_buffer_arg data = (pb_buffer_arg){.buf = data_type, .buf_len = data_type_len};
    sns_suid_req suid_req = sns_suid_req_init_default;

    suid_req.has_register_updates = true;
    suid_req.register_updates = true;
    suid_req.data_type.funcs.encode = &pb_encode_string_cb;
    suid_req.data_type.arg = &data;

    /* create the event/data stream for the platform resource */
    if (state->fw_stream == NULL)
    {
        stream_service->api->create_sensor_stream(stream_service, this,	sns_get_suid_lookup(),&state->fw_stream);
    }
    encoded_len = pb_encode_request(buffer,
									sizeof(buffer),
									&suid_req,
									sns_suid_req_fields,
									NULL);
    if (encoded_len > 0)
    {
        sns_request request = (sns_request){
                            .request_len = encoded_len,
                            .request = buffer,
                            .message_id = SNS_SUID_MSGID_SNS_SUID_REQ
                            };
        state->fw_stream->api->send_request(state->fw_stream, &request);
    }
}

void sx932x_sensor_process_suid_events(sns_sensor *const this)
{
    sx932x_state *state = (sx932x_state*)this->state->state;

    for (;0 != state->fw_stream->api->get_input_cnt(state->fw_stream); state->fw_stream->api->get_next_input(state->fw_stream))
    {
        sns_sensor_event *event = state->fw_stream->api->peek_input(state->fw_stream);

        if(SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
        {
            pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
            sns_suid_event suid_event = sns_suid_event_init_default;
            pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
            sns_sensor_uid uid_list;
            sns_suid_search suid_search;

            suid_search.suid = &uid_list;
            suid_search.num_of_suids = 0;
            suid_event.data_type.funcs.decode = &pb_decode_string_cb;
            suid_event.data_type.arg = &data_type_arg;
            suid_event.suid.funcs.decode = &pb_decode_suid_event;
            suid_event.suid.arg = &suid_search;

            if(!pb_decode(&stream, sns_suid_event_fields, &suid_event))
            {
                SX932X_SENSOR_LOG(ERROR, this, "SUID Decode failed");
                continue;
            }

            /* if no suids found, ignore the event */
            if(suid_search.num_of_suids == 0)
            {
                continue;
            }

            /* save suid based on incoming data type name */
            if(0 == strncmp(data_type_arg.buf, "interrupt", data_type_arg.buf_len))
            {
                state->irq_suid = uid_list;
            }
            else if(0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len))
            {
                state->timer_suid = uid_list;
            }
            else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len))
            {
                state->reg_suid = uid_list;
            }
            else
            {
                SX932X_SENSOR_LOG(ERROR, this, "invalid datatype_name");
            }
        }
    }
}

void sx932x_sensor_send_registry_request(sns_sensor *const this, char *reg_group_name)
{
    sx932x_state *state = (sx932x_state*)this->state->state;
    uint8_t buffer[100];
    int32_t encoded_len;
    sns_memset(buffer, 0, sizeof(buffer));
    sns_rc rc = SNS_RC_SUCCESS;
    sns_registry_read_req read_request;
    pb_buffer_arg data = (pb_buffer_arg){ .buf = reg_group_name, .buf_len = (strlen(reg_group_name) + 1) };

    read_request.name.arg = &data;
    read_request.name.funcs.encode = pb_encode_string_cb;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
    &read_request, sns_registry_read_req_fields, NULL);
    if(0 < encoded_len)
    {
        sns_request request = (sns_request){ .request_len = encoded_len, .request = buffer, .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ };
        rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
    }
}

void sx932x_sensor_process_registry_event(sns_sensor *const this,	sns_sensor_event *event)
{
    bool rv = true;
    sx932x_state *state = (sx932x_state*)this->state->state;
    pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);

    SX932X_SENSOR_LOG(LOW, this, "process_registry_event message_id=%d", event->message_id);

    if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
    {
        sns_registry_read_event read_event = sns_registry_read_event_init_default;
        pb_buffer_arg group_name = {0,0};
        read_event.name.arg = &group_name;
        read_event.name.funcs.decode = pb_decode_string_cb;

        if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
        {
            SX932X_SENSOR_LOG(ERROR, this, "Error decoding registry event");
        }
        else
        {
            stream = pb_istream_from_buffer((void*)event->event, event->event_len);
            if(0 == strncmp((char*)group_name.buf, SX932x_CONFIG_SAR, group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                                                    .item_group_name = &group_name,
                                                    .parse_info_len = 1,
                                                    .parse_info[0] =
                                                    {
                                                    .group_name = "config",
                                                    .parse_func = sns_registry_parse_phy_sensor_cfg,
                                                    .parsed_buffer = &state->registry_cfg
                                                    }
                                                 };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }
                if(rv)
                {
                    state->registry_cfg_received = true;
                    state->is_dri = state->registry_cfg.is_dri;
                    state->hardware_id = state->registry_cfg.hw_id;
                    state->resolution_idx = state->registry_cfg.res_idx;
                    state->supports_sync_stream = state->registry_cfg.sync_stream;

                    SX932X_SENSOR_LOG(LOW, this, "Registry read event for sensor %d received is_dri:%d, hardware_id:%lld",
                            state->sensor,
                            state->is_dri,
                            state->hardware_id);
                    SX932X_SENSOR_LOG(LOW, this, "resolution_idx:%d, supports_sync_stream:%d ",state->resolution_idx, state->supports_sync_stream);
                }
            }
            else if (0 == strncmp((char*)group_name.buf, SX932x_PLATFORM_CONFIG, group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                    .item_group_name = &group_name,
                    .parse_info_len = 1,
                    .parse_info[0] = {
                    .group_name = "config",
                    .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
                    .parsed_buffer = &state->registry_pf_cfg
                    }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }
                if(rv)
                {
                    state->registry_pf_cfg_received = true;

                    state->com_port_info.com_config.bus_type = state->registry_pf_cfg.bus_type;
                    state->com_port_info.com_config.bus_instance = state->registry_pf_cfg.bus_instance;
                    state->com_port_info.com_config.slave_control = state->registry_pf_cfg.slave_config;
                    state->com_port_info.com_config.min_bus_speed_KHz = state->registry_pf_cfg.min_bus_speed_khz;
                    state->com_port_info.com_config.max_bus_speed_KHz = state->registry_pf_cfg.max_bus_speed_khz;
                    state->com_port_info.com_config.reg_addr_type = state->registry_pf_cfg.reg_addr_type;
                    state->irq_config.interrupt_num = state->registry_pf_cfg.dri_irq_num;
                    state->irq_config.interrupt_pull_type = state->registry_pf_cfg.irq_pull_type;
                    state->irq_config.is_chip_pin = state->registry_pf_cfg.irq_is_chip_pin;
                    state->irq_config.interrupt_drive_strength = state->registry_pf_cfg.irq_drive_strength;
                    state->irq_config.interrupt_trigger_type = state->registry_pf_cfg.irq_trigger_type;
                    state->rail_config.num_of_rails = state->registry_pf_cfg.num_rail;
                    state->registry_rail_on_state = state->registry_pf_cfg.rail_on_state;
                    sns_strlcpy(state->rail_config.rails[0].name,
                    state->registry_pf_cfg.vddio_rail,
                    sizeof(state->rail_config.rails[0].name));
                    sns_strlcpy(state->rail_config.rails[1].name,
                    state->registry_pf_cfg.vdd_rail,
                    sizeof(state->rail_config.rails[1].name));

                    SX932X_SENSOR_LOG(LOW, this, "Registry read event for group %d received bus_type:%d bus_instance:%d slave_control:%d",
                                state->sensor,
                                state->com_port_info.com_config.bus_type,
                                state->com_port_info.com_config.bus_instance,
                                state->com_port_info.com_config.slave_control);

                    SX932X_SENSOR_LOG(LOW, this, "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                                state->com_port_info.com_config.min_bus_speed_KHz,
                                state->com_port_info.com_config.max_bus_speed_KHz,
                                state->com_port_info.com_config.reg_addr_type);
                    SX932X_SENSOR_LOG(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                                state->irq_config.interrupt_num,
                                state->irq_config.interrupt_pull_type,
                                state->irq_config.is_chip_pin);

                    SX932X_SENSOR_LOG(LOW, this, "interrupt_drive_strength:%d interrupt_trigger_type:%d rigid body type:%d",
            					state->irq_config.interrupt_drive_strength,
            					state->irq_config.interrupt_trigger_type,
            					state->registry_pf_cfg.rigid_body_type);

                    SX932X_SENSOR_LOG(LOW, this,"num_rail:%d, rail_on_state:%d, vddio_rail:%s, vdd_rail:%s", state->rail_config.num_of_rails,
                                 state->registry_rail_on_state,
                                 state->rail_config.rails[0].name,
                                 state->rail_config.rails[1].name);

                }
            }
            else if(0 == strncmp((char*)group_name.buf, SX932x_PLATFORM_PLACEMENT, group_name.buf_len))
            {
                {
                    uint8_t arr_index = 0;
                    pb_float_arr_arg arr_arg = {
                    .arr = state->placement,
                    .arr_index = &arr_index,
                    .arr_len = 12
                    };

                    sns_registry_decode_arg arg = {
                    .item_group_name = &group_name,
                    .parse_info_len = 1,
                    .parse_info[0] = {
                    .group_name = "placement",
                    .parse_func = sns_registry_parse_float_arr,
                    .parsed_buffer = &arr_arg
                    }
                    };
                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if(rv)
                {
                    state->registry_placement_received = true;
                    SX932X_SENSOR_LOG(LOW, this, "Registry read event for group %d received", state->sensor);
                }
            }
            else
            {
                SX932X_SENSOR_LOG(ERROR, this, "no find the group_name");
                rv = false;
            }

            if(!rv)
            {
                SX932X_SENSOR_LOG(ERROR,this, "Error decoding registry group %s due to %s", (char*)group_name.buf,
                PB_GET_ERROR(&stream));
            }
        }
    }
    else
    {
        SX932X_SENSOR_LOG(ERROR, this, "Received unsupported registry event msg id %u", event->message_id);
    }
}

