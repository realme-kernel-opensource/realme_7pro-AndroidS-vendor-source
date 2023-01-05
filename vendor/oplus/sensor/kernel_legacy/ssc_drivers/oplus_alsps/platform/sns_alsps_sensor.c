/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"
#include "sns_alsps_sensor.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"
#include "sns_printf.h"
#include "sns_devinfo_utils.h"

#include "oplus_alsps.h"
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

void alsps_process_suid_events(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;

    OPLUS_ALS_PS_LOG("start process suid events\n");

    for (; 0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
        state->fw_stream->api->get_next_input(state->fw_stream)) {
        sns_sensor_event *event = state->fw_stream->api->peek_input(state->fw_stream);

        if (SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id) {
            pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
            sns_suid_event suid_event = sns_suid_event_init_default;
            pb_buffer_arg data_type_arg = {.buf = NULL, .buf_len = 0};
            sns_sensor_uid uid_list;
            sns_suid_search suid_search;
            suid_search.suid = &uid_list;
            suid_search.num_of_suids = 0;

            suid_event.data_type.funcs.decode = &pb_decode_string_cb;
            suid_event.data_type.arg = &data_type_arg;
            suid_event.suid.funcs.decode = &pb_decode_suid_event;
            suid_event.suid.arg = &suid_search;

            if (!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
                OPLUS_ALS_PS_LOG("ALSPS SUID Decode failed");
                continue;
            }

            /* If no suids found, ignore the event */
            if (suid_search.num_of_suids == 0) {
                continue;
            }

            /* Save suid based on incoming data type name */
            if (0 == strncmp(data_type_arg.buf, "interrupt", data_type_arg.buf_len)) {
                state->irq_suid = uid_list;
            } else if (0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len)) {
                state->timer_suid = uid_list;
            } else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len)) {
                state->reg_suid = uid_list;
            } else if (0 == strncmp(data_type_arg.buf, "resampler", data_type_arg.buf_len)) {
                state->resampler_suid = uid_list;
            } else if (0 == strncmp(data_type_arg.buf, "accel", data_type_arg.buf_len)) {
                state->accel_suid = uid_list;
            } else {
                OPLUS_ALS_PS_LOG("ALSPS invalid datatype_name");
            }
        }
    }
}

void alsps_sensor_send_registry_request(sns_sensor *const this,
    char *reg_group_name)
{
    alsps_state *state = (alsps_state*)this->state->state;
    uint8_t buffer[100];
    int32_t encoded_len;
    sns_memset(buffer, 0, sizeof(buffer));
    sns_rc rc = SNS_RC_SUCCESS;

    sns_registry_read_req read_request;
    pb_buffer_arg data = (pb_buffer_arg) {
        .buf = reg_group_name,
        .buf_len = (strlen(reg_group_name) + 1)
    };

    read_request.name.arg = &data;
    read_request.name.funcs.encode = pb_encode_string_cb;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &read_request, sns_registry_read_req_fields, NULL);

    if (0 < encoded_len) {
        sns_request request = (sns_request) {
            .request_len = encoded_len, .request = buffer,
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ
        };
        rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
    }
}

void alsps_request_registry(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc =
        (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    /* Place a request to registry sensor */
    if (NULL == state->reg_data_stream) {
        stream_svc->api->create_sensor_stream(stream_svc,
            this, state->reg_suid, &state->reg_data_stream);

        alsps_sensor_send_registry_request(this, ALSPS_PLATFORM_CONFIG);
        alsps_sensor_send_registry_request(this, ALSPS_PLATFORM_FAC_CAL_ALS);
        alsps_sensor_send_registry_request(this, ALSPS_CONFIG_ALS);
        alsps_sensor_send_registry_request(this, ALSPS_PLATFORM_FAC_CAL_PROX);
        alsps_sensor_send_registry_request(this, ALSPS_CONFIG_PROX);
    }
}

void alsps_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;

    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE,
        &value, 1, true);
}

void alsps_publish_name(sns_sensor *const this, char *name)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg) {
        .buf = name, .buf_len = strlen(name) + 1
    });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
}

void alsps_publish_registry_attributes(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = state->is_dri;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = state->supports_sync_stream;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = state->hardware_id;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = state->registry_pf_cfg.rigid_body_type;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
    }

    if (state->sensor == ALS ||
        state->sensor == PS) {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_flt = true;
        value.flt = (state->sensor == ALS) ? ALSPS_ALS_RESOLUTION : ALSPS_PROX_RESOLUTION;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);
    }

    if (state->sensor == ALS ||
        state->sensor == PS) {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        sns_std_attr_value_data rangeMinMax[] = {SNS_ATTR, SNS_ATTR};
        rangeMinMax[0].has_flt = true;
        rangeMinMax[0].flt = (state->sensor == ALS) ? ALSPS_ALS_RANGE_MIN : ALSPS_PROX_RANGE_MIN;
        rangeMinMax[1].has_flt = true;
        rangeMinMax[1].flt = (state->sensor == ALS) ? ALSPS_ALS_RANGE_MAX : ALSPS_PROX_RANGE_MAX;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = rangeMinMax, .buf_len = ARR_SIZE(rangeMinMax)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0], ARR_SIZE(values), true);
    }

    {
        if (state->is_dri) {
            sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
            value.has_boolean = true;
            value.boolean = true;
            sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
        }
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
            values, ARR_SIZE(values), false);
    }
}

bool alsps_discover_hw(sns_sensor * const this)
{
    sns_rc rv = SNS_RC_SUCCESS;
    bool hw_is_present = false;

    alsps_state *state = (alsps_state*)this->state->state;

    rv = alsps_get_who_am_i(this);

    if (rv == SNS_RC_SUCCESS) {
        hw_is_present = true;
        static bool register_deinfo = false;
        struct devinfo info_ps = {
            .cal_path = ALSPS_PLATFORM_FAC_CAL_PROX,
        };
        struct devinfo info_als = {
            .cal_path = ALSPS_PLATFORM_FAC_CAL_ALS,
        };

        if (!register_deinfo) {
            register_deinfo = true;
            register_sensor_devinfo(OPLUS_PS, &info_ps);
            register_sensor_devinfo(OPLUS_ALS, &info_als);
            SNS_PRINTF(HIGH, this, "register_sensor_devinfo");
        }
    }

    ///Turn Power Rail OFF
    state->rail_config.rail_vote = SNS_RAIL_OFF;
    state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
        this,
        &state->rail_config,
        NULL);

    return hw_is_present;
}

void alsps_send_suid_req(sns_sensor *this, char *const data_type, uint32_t data_type_len)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_service = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
    pb_buffer_arg data = (pb_buffer_arg) {
        .buf = data_type, .buf_len = data_type_len
    };
    size_t encoded_len;
    uint8_t buffer[50];

    sns_memset(buffer, 0, sizeof(buffer));

    sns_suid_req suid_req = sns_suid_req_init_default;
    suid_req.has_register_updates = true;
    suid_req.register_updates = true;
    suid_req.data_type.funcs.encode = &pb_encode_string_cb;
    suid_req.data_type.arg = &data;

    if (NULL == state->fw_stream) {
        stream_service->api->create_sensor_stream(stream_service,
            this, sns_get_suid_lookup(),
            &state->fw_stream);
    }

    encoded_len = pb_encode_request(buffer, sizeof(buffer), &suid_req, sns_suid_req_fields, NULL);

    if (0 < encoded_len) {
        sns_request request = (sns_request) {
            .request_len = encoded_len,
            .request = buffer,
            .message_id = SNS_SUID_MSGID_SNS_SUID_REQ
        };

        state->fw_stream->api->send_request(state->fw_stream, &request);
    }
}

static sns_rc alsps_register_power_rail(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_rc rv = SNS_RC_SUCCESS;

    state->rail_config.rail_vote = SNS_RAIL_OFF;

    if (NULL == state->pwr_rail_service) {
        state->pwr_rail_service =
            (sns_pwr_rail_service*)smgr->get_service(smgr, SNS_POWER_RAIL_SERVICE);

        if (NULL == state->pwr_rail_service) {
            return SNS_RC_FAILED;
        }

        state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
            &state->rail_config);
    }

    return rv;
}

void alsps_start_power_rail_timer(sns_sensor *const this,
    sns_time timeout_ticks,
    alsps_power_rail_pending_state pwr_rail_pend_state)
{
    alsps_state *state = (alsps_state*)this->state->state;

    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t req_len;
    uint8_t buffer[20];
    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout_ticks;

    OPLUS_ALS_PS_LOG("start power rail timer\n");

    if (NULL == state->timer_stream) {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
        stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid, &state->timer_stream);
    }

    req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);

    if (req_len > 0 && NULL != state->timer_stream) {
        sns_request timer_req = {
            .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
            .request = buffer, .request_len = req_len
        };
        state->timer_stream->api->send_request(state->timer_stream, &timer_req);
        state->power_rail_pend_state = pwr_rail_pend_state;
    } else {
        OPLUS_ALS_PS_LOG("TCS3701 timer req encode error");
    }
}
void alsps_start_hw_detect_sequence(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    OPLUS_ALS_PS_LOG("start hw detect\n");
    /* Use thi flag to avoid doing hw detect too many times */
    state->start_hw_detect = true;

    /* Register Power Rails */
    if (!sns_sensor_uid_compare(&state->timer_suid, &((sns_sensor_uid){{0}}))
        && NULL == state->pwr_rail_service && SNS_RC_SUCCESS == rv) {
        rv = alsps_register_power_rail(this);

        /* Turn Power Rails ON */
        state->rail_config.rail_vote = state->registry_rail_on_state;

        if (SNS_RC_SUCCESS == rv) {
            rv = state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                    this,
                    &state->rail_config,
                    NULL);
        }

        /* Create a Timer stream for Power Rail ON timeout */
        if (SNS_RC_SUCCESS == rv) {
            alsps_start_power_rail_timer(this,
                sns_convert_ns_to_ticks(ALSPS_OFF_TO_IDLE_MS * 1000 * 1000),
                ALSPS_POWER_RAIL_PENDING_INIT);
        }
    }
}

bool alsps_parse_registry_float_array(sns_registry_data_item *reg_item,
    pb_buffer_arg  *item_name,
    pb_buffer_arg  *item_str_val,
    void *parsed_buffer)
{
    bool rv = true;
    alsps_pb_parse_arg *arg = (alsps_pb_parse_arg*) parsed_buffer;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
        NULL == parsed_buffer) {
        rv = false;
    } else if (reg_item->has_flt && arg->size > *arg->index) {
        arg->data_array[*arg->index] = reg_item->flt;

        if (reg_item->has_version) {
            arg->version_array[*arg->index] = reg_item->version;
        }

        (*arg->index)++;
    } else {
        rv = false;
    }

    return rv;
}

void alsps_sensor_process_registry_event(sns_sensor *const this,
    sns_sensor_event *event)
{
    bool rv = true;
    alsps_state *state = (alsps_state*)this->state->state;
    pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
            event->event_len);

    OPLUS_ALS_PS_LOG("start process registry event\n");

    if (SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id) {
        sns_registry_read_event read_event =
            sns_registry_read_event_init_default;
        pb_buffer_arg group_name = {0, 0};
        read_event.name.arg = &group_name;
        read_event.name.funcs.decode = pb_decode_string_cb;

        if (!pb_decode(&stream, sns_registry_read_event_fields, &read_event)) {
            OPLUS_ALS_PS_LOG("Error decoding registry event");
        } else {
            stream = pb_istream_from_buffer((void*)event->event,
                    event->event_len);

            if (0 == strncmp((char*)group_name.buf, ALSPS_CONFIG_ALS,
                    group_name.buf_len)) {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_cfg,
                            .parsed_buffer = &state->als_registry_cfg
                        }
                    };
                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    state->registry_cfg_received = true;
                    state->is_dri = state->als_registry_cfg.is_dri;
                    state->hardware_id = state->als_registry_cfg.hw_id;
                    state->resolution_idx = state->als_registry_cfg.res_idx;
                    state->supports_sync_stream = state->als_registry_cfg.sync_stream;

                    OPLUS_ALS_PS_LOG(
                        "Registry read event for group %s received,  %s senosr"
                        "is_dri:%d, hardware_id:%lld ",
                        (char*)group_name.buf,
                        state->sensor == ALS ? "als" : "ps",
                        state->is_dri,
                        state->hardware_id);
                    OPLUS_ALS_PS_LOG(
                        "resolution_idx:%d, supports_sync_stream:%d ",
                        state->resolution_idx,
                        state->supports_sync_stream);
                }
            } else if (0 == strncmp((char*)group_name.buf, ALSPS_CONFIG_PROX,
                    group_name.buf_len)) {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_cfg,
                            .parsed_buffer = &state->prox_registry_cfg
                        }
                    };
                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    state->registry_cfg_received = true;
                    state->is_dri = state->prox_registry_cfg.is_dri;
                    state->hardware_id = state->prox_registry_cfg.hw_id;
                    state->resolution_idx = state->prox_registry_cfg.res_idx;
                    state->supports_sync_stream = state->prox_registry_cfg.sync_stream;

                    OPLUS_ALS_PS_LOG(
                        "Registry read event for group %s received,  %s senosr"
                        "is_dri:%d, hardware_id:%lld ",
                        (char*)group_name.buf,
                        state->sensor == ALS ? "als" : "ps",
                        state->is_dri,
                        state->hardware_id);
                    OPLUS_ALS_PS_LOG(
                        "resolution_idx:%d, supports_sync_stream:%d ",
                        state->resolution_idx,
                        state->supports_sync_stream);
                }
            } else if (0 == strncmp((char*)group_name.buf, ALSPS_PLATFORM_CONFIG, group_name.buf_len)) {
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
                    read_event.data.items.funcs.decode =
                        &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields,
                            &read_event);
                }

                if (rv) {
                    state->registry_pf_cfg_received = true;
                    state->com_port_info.com_config.bus_type =
                        state->registry_pf_cfg.bus_type;
                    state->com_port_info.com_config.bus_instance =
                        state->registry_pf_cfg.bus_instance;
                    state->com_port_info.com_config.slave_control =
                        state->registry_pf_cfg.slave_config;
                    state->com_port_info.com_config.min_bus_speed_KHz =
                        state->registry_pf_cfg.min_bus_speed_khz;
                    state->com_port_info.com_config.max_bus_speed_KHz =
                        state->registry_pf_cfg.max_bus_speed_khz;
                    state->com_port_info.com_config.reg_addr_type =
                        state->registry_pf_cfg.reg_addr_type;
                    state->irq_config.interrupt_num =
                        state->registry_pf_cfg.dri_irq_num;
                    state->irq_config.interrupt_pull_type =
                        state->registry_pf_cfg.irq_pull_type;
                    state->irq_config.is_chip_pin =
                        state->registry_pf_cfg.irq_is_chip_pin;
                    state->irq_config.interrupt_drive_strength =
                        state->registry_pf_cfg.irq_drive_strength;
                    state->irq_config.interrupt_trigger_type =
                        state->registry_pf_cfg.irq_trigger_type;
                    state->rail_config.num_of_rails =
                        state->registry_pf_cfg.num_rail;
                    state->registry_rail_on_state =
                        state->registry_pf_cfg.rail_on_state;
                    sns_strlcpy(state->rail_config.rails[0].name,
                        state->registry_pf_cfg.vddio_rail,
                        sizeof(state->rail_config.rails[0].name));
                    sns_strlcpy(state->rail_config.rails[1].name,
                        state->registry_pf_cfg.vdd_rail,
                        sizeof(state->rail_config.rails[1].name));
                    OPLUS_ALS_PS_LOG(
                        "Registry read event for group %s received %s sensor"
                        "bus_type:%d bus_instance:%d slave_control:%d",
                        (char*)group_name.buf,
                        state->sensor == ALS ? "als" : "ps",
                        state->com_port_info.com_config.bus_type,
                        state->com_port_info.com_config.bus_instance,
                        state->com_port_info.com_config.slave_control);
                    OPLUS_ALS_PS_LOG(
                        "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                        state->com_port_info.com_config.min_bus_speed_KHz,
                        state->com_port_info.com_config.max_bus_speed_KHz,
                        state->com_port_info.com_config.reg_addr_type);
                    OPLUS_ALS_PS_LOG(
                        "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                        state->irq_config.interrupt_num,
                        state->irq_config.interrupt_pull_type,
                        state->irq_config.is_chip_pin);
                    OPLUS_ALS_PS_LOG(
                        "interrupt_drive_strength:%d interrupt_trigger_type:%d"
                        " rigid body type:%d",
                        state->irq_config.interrupt_drive_strength,
                        state->irq_config.interrupt_trigger_type,
                        state->registry_pf_cfg.rigid_body_type);
                }
            } else if (0 == strncmp((char*)group_name.buf, ALSPS_PLATFORM_FAC_CAL_PROX, group_name.buf_len)) {
                {
                    uint8_t ps_arr_index = 0;
                    alsps_pb_parse_arg ps_arr_arg = {
                        .data_array = state->ps_fac_cal_data,
                        .version_array = state->ps_cal_version,
                        .index = &ps_arr_index,
                        .size = PS_FAC_CAL_NUM
                    };
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "fac_cal",
                            .parse_func = alsps_parse_registry_float_array,
                            .parsed_buffer = &ps_arr_arg
                        },
                    };
                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    state->registry_fac_cal_received = true;
                    OPLUS_ALS_PS_LOG(
                        "pa data[0] = %d (ver: %d), data[1] = %d (ver: %d), data[2] = %d (ver: %d)"
                        "data[3] = %d (ver: %d), data[4] = %d (ver: %d)",
                        (uint32_t)(state->ps_fac_cal_data[0]),
                        state->ps_cal_version[0],
                        (uint32_t)(state->ps_fac_cal_data[1]),
                        state->ps_cal_version[1],
                        (uint32_t)(state->ps_fac_cal_data[2]),
                        state->ps_cal_version[2],
                        (uint32_t)(state->ps_fac_cal_data[3]),
                        state->ps_cal_version[3],
                        (uint32_t)(state->ps_fac_cal_data[4]),
                        state->ps_cal_version[4]);
                }
            } else if (0 == strncmp((char*)group_name.buf, ALSPS_PLATFORM_FAC_CAL_ALS, group_name.buf_len)) {
                {
                    uint8_t ps_arr_index = 0;
                    alsps_pb_parse_arg ps_arr_arg = {
                        .data_array = state->als_fac_cal_data,
                        .version_array = state->als_cal_version,
                        .index = &ps_arr_index,
                        .size = ALS_FAC_CAL_NUM
                    };
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "fac_cal",
                            .parse_func = alsps_parse_registry_float_array,
                            .parsed_buffer = &ps_arr_arg
                        },
                    };
                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;
                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    state->registry_fac_cal_received = true;
                    OPLUS_ALS_PS_LOG(
                        "ALS data[0] = %d (ver: %d), data[1] = %d (ver: %d)",
                        (uint32_t)(state->als_fac_cal_data[0]),
                        state->als_cal_version[0],
                        (uint32_t)(state->als_fac_cal_data[1]),
                        state->als_cal_version[1]);
                }
            } else {
                rv = false;
            }
        }
    } else {
        OPLUS_ALS_PS_LOG("Received unsupported registry event msg id %u",
            event->message_id);
    }
}

void alsps_update_sensor_state(sns_sensor *const this,
    sns_sensor_instance *const instance)
{
    alsps_state *sensor_state;
    alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;
    sns_sensor *sensor = NULL;

    for (sensor = this->cb->get_library_sensor(this, true);
        sensor != NULL;
        sensor = this->cb->get_library_sensor(this, false)) {
        sensor_state = (alsps_state*)sensor->state->state;

        sensor_state->als_fac_cal_data[0] = inst_state->als_fac_cal_data[0];
        sensor_state->als_fac_cal_data[1] = inst_state->als_fac_cal_data[1];
        sensor_state->als_cal_version[0]  = inst_state->als_cal_version[0];
        sensor_state->als_cal_version[1]  = inst_state->als_cal_version[1];
        sns_memscpy(&sensor_state->als_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg),
            &inst_state->als_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg));

        for (int i = 0; i < sizeof(sensor_state->ps_fac_cal_data) / sizeof(float); i++) {
            sensor_state->ps_fac_cal_data[i]  = inst_state->ps_fac_cal_data[i];
            sensor_state->ps_cal_version[i]   = inst_state->ps_cal_version[i];
        }

        sns_memscpy(&sensor_state->prox_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg),
            &inst_state->prox_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg));
    }
}

static bool alsps_encode_registry_group_cb_als(struct pb_ostream_s *stream,
    struct pb_field_s const *field,
    void *const *arg)
{
    pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
    alsps_instance_state *state = (alsps_instance_state*)pb_arg->instance->state->state;

    if (0 == strncmp(pb_arg->name, "fac_cal", strlen("fac_cal"))) {
        char const *names[] = {"scale", "bias"};

        for (int i = 0; i < ARR_SIZE(names); i++) {
            pb_buffer_arg name_data = (pb_buffer_arg) {
                .buf = names[i],
                .buf_len = strlen(names[i]) + 1
            };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;
            pb_item.name.funcs.encode = &pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;
            pb_item.flt = state->als_fac_cal_data[i];
            pb_item.version = state->als_cal_version[i];

            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
                return false;
            }
        }
    }

    return true;
}

static bool alsps_encode_registry_group_cb_ps(struct pb_ostream_s *stream,
    struct pb_field_s const *field,
    void *const *arg)
{
    pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
    alsps_instance_state *state = (alsps_instance_state*)pb_arg->instance->state->state;

    if (0 == strncmp(pb_arg->name, "fac_cal", strlen("fac_cal"))) {
        char const *names[] = {"offset", "3cm_threshold", "delta", "cali_goal", "cali_up_thrd", "dynamic_cali_offset"};

        for (int i = 0; i < ARR_SIZE(names); i++) {
            pb_buffer_arg name_data = (pb_buffer_arg) {
                .buf = names[i],
                .buf_len = strlen(names[i]) + 1
            };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;
            pb_item.name.funcs.encode = &pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;
            pb_item.flt = state->ps_fac_cal_data[i];
            pb_item.version = state->ps_cal_version[i];

            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
                SNS_INST_PRINTF(ERROR, pb_arg->instance, "Error encoding sns_registry_data_item_fields");
                return false;
            }
        }
    }

    return true;
}

static bool alsps_encode_registry_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
    void *const *arg)
{
    pb_arg_reg_group_arg *reg_arg = (pb_arg_reg_group_arg*)*arg;
    sns_sensor_instance *instance = reg_arg->instance;
    char const *names[] = {"fac_cal"};

    for (int i = 0; i < ARR_SIZE(names); i++) {
        pb_buffer_arg name_data = (pb_buffer_arg) {
            .buf = names[i],
            .buf_len = strlen(names[i]) + 1
        };
        sns_registry_data_item pb_item = sns_registry_data_item_init_default;
        pb_arg_reg_group_arg pb_arg = (pb_arg_reg_group_arg) {
            .name = NULL,
            .instance = instance,
            .sensor = reg_arg->sensor
        };
        pb_item.name.arg = &name_data;
        pb_item.name.funcs.encode = &pb_encode_string_cb;

        if (0 == strncmp(names[i], "fac_cal", name_data.buf_len)) {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;

            if (reg_arg->sensor == ALS) {
                pb_item.subgroup.items.funcs.encode = &alsps_encode_registry_group_cb_als;
            } else if (reg_arg->sensor == PS) {
                pb_item.subgroup.items.funcs.encode = &alsps_encode_registry_group_cb_ps;
            }

            pb_item.subgroup.items.arg = &pb_arg;
        }

        if (!pb_encode_tag_for_field(stream, field)) {
            SNS_INST_PRINTF(ERROR, instance, "Error encoding corr_mat");
            return false;
        }

        if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
            SNS_INST_PRINTF(ERROR, instance, "Error encoding sns_registry_data_item_fields");
            return false;
        }
    }

    return true;
}

void alsps_update_registry(sns_sensor *const this,
    sns_sensor_instance *const instance, alsps_sensor_type sensor)
{
    alsps_state *state = (alsps_state*)this->state->state;
    pb_arg_reg_group_arg arg = {.instance = instance };
    uint8_t buffer[1000];
    int32_t encoded_len;
    char als_name[] = ALSPS_PLATFORM_ALS;
    char ps_name[]  = ALSPS_PLATFORM_PROX;
    pb_buffer_arg name_data;
    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    if (sensor == PS) {
        name_data = (pb_buffer_arg) {
            .buf = ps_name,
            .buf_len = strlen(ps_name) + 1
        };
        arg.sensor = PS;
    } else if (sensor == ALS) {
        name_data = (pb_buffer_arg) {
            .buf = als_name, .buf_len = strlen(als_name) + 1
        };
        arg.sensor = ALS;
    } else {
        OPLUS_ALS_PS_LOG("Unsupported sensor %d", sensor);
    }

    write_req.name.funcs.encode = &pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = &alsps_encode_registry_cb;
    write_req.data.items.arg = &arg;
    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &write_req, sns_registry_write_req_fields, NULL);

    if (0 < encoded_len) {
        if (NULL == state->reg_data_stream) {
            sns_service_manager *smgr = this->cb->get_service_manager(this);
            sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
            stream_svc->api->create_sensor_stream(stream_svc, this, state->reg_suid, &state->reg_data_stream);
        }

        sns_request request = (sns_request) {
            .request_len = encoded_len,
            .request = buffer,
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ
        };
        state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
    }
}

void alsps_parse_smem_info(sns_sensor * this)
{
    alsps_state *state = (alsps_state*)this->state->state;
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
    /*one project has same parameter, don't distinguish*/
    struct sensor_feature* ps_feature = NULL;
    struct sensor_feature* als_feature = NULL;

    oppo_get_sensor_feature(OPPO_PROXIMITY, -1, &ps_feature);

    if (ps_feature != NULL) {
        state->ps_type = ps_feature->feature[0];
    }

    oppo_get_sensor_feature(OPPO_LIGHT, -1, &als_feature);

    if (als_feature != NULL) {
        state->als_type = als_feature->feature[0];
        state->is_unit_device = als_feature->feature[1];
        state->is_als_dri = als_feature->feature[2];
        state->use_lb_algo = als_feature->feature[7];

    }

#else
//here you can set default value
    state->ps_type = 1;
    state->als_type = 1;
    state->is_unit_device = 0;
    state->is_als_dri = 0;
    state->als_factor = 110;
    state->als_buffer_length = 10;
    state->is_als_initialed = 0;
    state->irq_number = 122;
    state->bus_number = 5;
    state->use_lb_algo = true;
#endif
    SNS_PRINTF(ERROR, this, "als_type %d ps_type %d"
        " is_unit_device %d is_als_dri %d\n",
        state->als_type, state->ps_type, state->is_unit_device, state->is_als_dri);
}

void alsps_common_init(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;

    struct sns_service_manager *smgr = this->cb->get_service_manager(this);
    state->diag_service = (sns_diag_service *)smgr->get_service(smgr, SNS_DIAG_SERVICE);
    state->scp_service = (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);
    state->island_service = (sns_island_service *) smgr->get_service(smgr, SNS_ISLAND_SERVICE);

    alsps_parse_smem_info(this);

    alsps_send_suid_req(this, "interrupt", sizeof("interrupt"));
    alsps_send_suid_req(this, "timer", sizeof("timer"));
    alsps_send_suid_req(this, "accel", sizeof("accel"));
    alsps_send_suid_req(this, "resampler", sizeof("resampler"));

    /* Send registry suid request at last */
    alsps_send_suid_req(this, "registry", sizeof("registry"));
}

