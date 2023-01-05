/************************************************************************************
# Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
# OPLUS_FEATURE_SENSOR
# File: step_counter.cpp
#
# Description:
#      Definitions for devinfo sensor.
#
# Version: 1.0
# Date created: 2018/03/09,20:27
#
# --------------------------- Revision History: ------------------------------------
# <version>     <date>      <author>        <desc>
**************************************************************************************/
#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_attribute_service.h"
#include "sns_attribute_util.h"
#include "sns_island_util.h"
#include "sns_mem_util.h"
#include "sns_memmgr.h"
#include "sns_pb_util.h"
#include "sns_devinfo_sensor.h"
#include "sns_printf_int.h"
#include "sns_registry.pb.h"
#include "sns_registry_util.h"
#include "sns_sensor.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_suid.pb.h"
#include "sns_types.h"
#include "sns_devinfo_utils.h"

#define DEVINFO_SENSOR_SUID 0xdd,0x58,0xbe,0xef,0x23,0xb8,0x47,0xd3,\
    0xad,0x4c,0xd3,0x2d,0x28,0x56,0xd8,0xe7

struct dev_node {
    int sensor_id;
    char *registry;
};

struct dev_node node[] = {
    {OPLUS_ALS, "devinfo.als"},
    {OPLUS_PS, "devinfo.ps"},
    {OPLUS_STRUCTURE_PS, "devinfo.structure_ps"},
    {OPLUS_GSENSOR, "devinfo.gsensor"},
    {OPLUS_GYRO, "devinfo.gyro"},
    {OPLUS_MAG, "devinfo.mag"},
    {OPLUS_RGB, "devinfo.rgb"},
    {OPLUS_RGB_REAR, "devinfo.rgb_rear"},
    {OPLUS_PRESS, "devinfo.pressure"}
};

static sns_sensor *devinfo_sensor = NULL;

/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 */
static void
devinfo_sensor_publish_attributes(sns_sensor *const this)
{
    {
        char const name[] = "devinfo_sensor";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = name, .buf_len = sizeof(name)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
    {
        char const type[] = "devinfo_sensor";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = type, .buf_len = sizeof(type)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }
    {
        char const vendor[] = "oppo";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = vendor, .buf_len = sizeof(vendor)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = 1;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, true);
    }
}

static void
sns_publish_available(sns_sensor *const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

/* See sns_sensor::init */
static sns_rc
devinfo_sensor_init(sns_sensor *const this)
{
    sns_devinfo_sensor_state *state = (sns_devinfo_sensor_state *)this->state->state;

    devinfo_sensor_publish_attributes(this);

    SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
    sns_suid_lookup_add(this, &state->suid_lookup_data, "registry");

    return SNS_RC_SUCCESS;
}

static bool
devinfo_sensor_encode_registry_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
    void *const *arg)
{
    struct devinfo *info = (struct devinfo *)*arg;
    char const *names[] = {"cal_path"};

    SNS_SPRINTF(ERROR, devinfo_sensor, "devinfo_sensor_encode_registry_cb start");

    for (int i = 0; i < ARR_SIZE(names); i++) {
        pb_buffer_arg name_data = (pb_buffer_arg) {
            .buf = names[i], .buf_len = strlen(names[i]) + 1
        };

        sns_registry_data_item pb_item = sns_registry_data_item_init_default;
        if (!strcmp(names[i], "cal_path")) {
            pb_buffer_arg str_data = (pb_buffer_arg) {
                .buf = info->cal_path, .buf_len = strlen(info->cal_path) + 1
            };
            pb_item.str.funcs.encode = &pb_encode_string_cb;
            pb_item.str.arg = &str_data;
            SNS_SPRINTF(ERROR, devinfo_sensor, "devinfo_sensor_encode_registry_cb names %s ", names[i]);
            SNS_SPRINTF(ERROR, devinfo_sensor, "devinfo_sensor_encode_registry_cb cal_path %s", info->cal_path);
            SNS_SPRINTF(ERROR, devinfo_sensor, "devinfo_sensor_encode_registry_cb cal_path_len %d",
                strlen(info->cal_path));
        }

        pb_item.name.funcs.encode = &pb_encode_string_cb;
        pb_item.name.arg = &name_data;

        if (!pb_encode_tag_for_field(stream, field)) {
            return false;
        }

        if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
            return false;
        }
    }

    return true;
}

/**
 * Send a request to the registry sensor to update provided devinfo to registry
 *
 * @param[i] sensor_id
 * @param[i] info   refer to definition struct devinf
 */
void register_sensor_devinfo(uint8_t sensor_id, struct devinfo *info)
{
    sns_devinfo_sensor_state *state = NULL;
    uint8_t i = 0;
    uint8_t buffer[256];
    int32_t encoded_len;
    char *reg_group_name = NULL;
    pb_buffer_arg name_data;
    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    sns_memset(buffer, 0, sizeof(buffer));

    if (!devinfo_sensor || !info) {
        return;
    } else {
        state = (sns_devinfo_sensor_state *)devinfo_sensor->state->state;
    }

    if (sensor_id >= MAX) {
        SNS_SPRINTF(ERROR, devinfo_sensor, "sensor_id %d >= MAX %d ,error", sensor_id, MAX);
        return;
    }

    for (i = 0; i < sizeof(node) / sizeof(struct dev_node); i++) {
        if (node[i].sensor_id == sensor_id) {
            reg_group_name = node[i].registry;
            SNS_SPRINTF(ERROR, devinfo_sensor, "update sensor %d registry reg_group_name '%s' cal_path '%s'",
                sensor_id, reg_group_name, info->cal_path);
            break;
        }
    }

    if (!reg_group_name) {
        SNS_SPRINTF(ERROR, devinfo_sensor, "reg_group_name NULL");
        return;
    }

    name_data = (pb_buffer_arg) {
        .buf = reg_group_name, .buf_len = strlen(reg_group_name) + 1
    };

    write_req.name.funcs.encode = &pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = &devinfo_sensor_encode_registry_cb;
    write_req.data.items.arg = info;
    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &write_req, sns_registry_write_req_fields, NULL);
    SNS_SPRINTF(ERROR, devinfo_sensor, "pb_encode_request end");
    if (0 < encoded_len) {
        SNS_SPRINTF(ERROR, devinfo_sensor, "send_request start encoded_len %d", encoded_len);
        if (NULL == state->reg_stream) {
            sns_service_manager *smgr = devinfo_sensor->cb->get_service_manager(devinfo_sensor);
            sns_stream_service *stream_svc = (sns_stream_service *)smgr->get_service(smgr, SNS_STREAM_SERVICE);
            sns_sensor_uid suid;
            sns_suid_lookup_get(&state->suid_lookup_data, "registry", &suid);
            stream_svc->api->create_sensor_stream(stream_svc, devinfo_sensor, suid, &state->reg_stream);
        }

        sns_request request = (sns_request) {
            .request_len = encoded_len,
            .request = buffer,
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ
        };
        state->reg_stream->api->send_request(state->reg_stream, &request);
        SNS_SPRINTF(ERROR, devinfo_sensor, "send_request end");
    }
}

/* See sns_sensor::notify_event */
sns_rc devinfo_sensor_notify_event(sns_sensor *const this)
{
    sns_devinfo_sensor_state *state = (sns_devinfo_sensor_state *)this->state->state;
    sns_service_manager *manager = this->cb->get_service_manager(this);
    sns_stream_service *stream_service =
        (sns_stream_service *)manager->get_service(manager, SNS_STREAM_SERVICE);
    bool completed = sns_suid_lookup_complete(&state->suid_lookup_data);

    devinfo_sensor = this;

    sns_suid_lookup_handle(this, &state->suid_lookup_data);
    SNS_SPRINTF(ERROR, this, "devinfo_sensor_notify_event");

    if (completed != sns_suid_lookup_complete(&state->suid_lookup_data)) {
        sns_sensor_uid suid;

        sns_suid_lookup_get(&state->suid_lookup_data, "registry", &suid);
        sns_suid_lookup_deinit(this, &state->suid_lookup_data);

        sns_publish_available(this);
        SNS_SPRINTF(ERROR, this, "sns_publish_available");
        if (NULL == state->reg_stream) {
            stream_service->api->create_sensor_stream(stream_service,
                this, suid, &state->reg_stream);
        }
    }

    if (NULL != state->reg_stream) {
        sns_sensor_event *event = state->reg_stream->api->peek_input(state->reg_stream);

        SNS_SPRINTF(ERROR, this, "devinfo_sensor_notify_event1");
        while (NULL != event) {
            //do nothing
            event = state->reg_stream->api->get_next_input(state->reg_stream);
        }
    }

    return SNS_RC_SUCCESS;
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const *
devinfo_sensor_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid devinfo_sensor_suid = {.sensor_uid = {DEVINFO_SENSOR_SUID}};
    return &devinfo_sensor_suid;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_api sns_devinfo_sensor_api = {
    .struct_len = sizeof(sns_sensor_api),
    .init = &devinfo_sensor_init,
    .deinit = NULL,
    .get_sensor_uid = &devinfo_sensor_get_sensor_uid,
    .set_client_request = NULL,
    .notify_event = &devinfo_sensor_notify_event,
};
