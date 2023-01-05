/*=============================================================================
  @file sns_mag_fusion_sensor.c

  The mag_fusion virtual Sensor implementation

  Copyright (c) 2017 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/
#include "sns_mem_util.h"
#include "sns_mag_fusion_sensor.h"
#include "sns_mag_fusion_sensor_instance.h"

#include "sns_mag_cal.pb.h"
#include "sns_suid.pb.h"
#include "sns_registry.pb.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "sns_stream_service.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_attribute_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"
#include "sns_printf.h"

#include "stdio.h"

/*=============================================================================
  Function Definitions
  ===========================================================================*/

static bool sns_send_to_registry_persist_cb(
    struct pb_ostream_s *stream,
    struct pb_field_s const *field,
    void *const *arg)
{
    sns_mag_fusion_config *config = (sns_mag_fusion_config*)*arg;

    if (config == NULL)
        return false;

    // char const *names[] = {"accuracy", "bias_0", "bias_1", "bias_2"};
    char const *names[] = {"accuracy", "bias_0", "bias_1", "bias_2"};

    for(int i = 0; i < ARR_SIZE(names); i++) {
        pb_buffer_arg name_data = (pb_buffer_arg) {
            .buf = names[i], .buf_len = strlen(names[i]) + 1
        };
        sns_registry_data_item pb_item = sns_registry_data_item_init_default;

        pb_item.name.funcs.encode = &pb_encode_string_cb;
        pb_item.name.arg = &name_data;
        pb_item.has_version = true;
        pb_item.version = 1;

        if (0 == strncmp(name_data.buf, "accuracy", name_data.buf_len)) {
            pb_item.has_sint = true;
            pb_item.sint = config->accuracy;
        } else if (0 == strncmp(name_data.buf, "bias_0", name_data.buf_len)) {
            pb_item.has_flt = true;
            pb_item.flt = config->bias[0];
        } else if (0 == strncmp(name_data.buf, "bias_1", name_data.buf_len)) {
            pb_item.has_flt = true;
            pb_item.flt = config->bias[1];
        } else if (0 == strncmp(name_data.buf, "bias_2", name_data.buf_len)) {
            pb_item.has_flt = true;
            pb_item.flt = config->bias[2];
        } else {
            continue;
        }

        if(!pb_encode_tag_for_field(stream, field))
            return false;

        if(!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
            return false;
    }

    return true;
}

static sns_rc sns_mag_fusion_write_bias_back_to_registry(sns_sensor *const this)
{
    sns_mag_fusion_sensor_state *state =
        (sns_mag_fusion_sensor_state*)this->state->state;

    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    pb_buffer_arg name_data = (pb_buffer_arg) {
        .buf = MAG_FUSION_GROUP_NAME, .buf_len = sizeof(MAG_FUSION_GROUP_NAME)
    };

    write_req.name.funcs.encode = &pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = &sns_send_to_registry_persist_cb;
    write_req.data.items.arg = &state->config;

    MAG_FUSION_LOGI("writing back bias=[%d, %d, %d]/1000, accuracy=%d",
        (int)(state->config.bias[0] * 1000),
        (int)(state->config.bias[1] * 1000),
        (int)(state->config.bias[2] * 1000),
        state->config.accuracy);
    pb_byte_t buffer[200];
    size_t encoded_len;
    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &write_req, sns_registry_write_req_fields, NULL);

    if(0 < encoded_len) {
        sns_request request = (sns_request) {
            .request_len = encoded_len, .request = buffer,
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ
        };
        sns_rc rc = state->registry_stream->api->send_request(state->registry_stream, &request);
        UNUSED_VAR(rc);
        MAG_FUSION_LOGI("send registry write request, rc %d", rc);
    } else {
        MAG_FUSION_LOGI("encode registry group failed. encoded_len=%d", encoded_len);
    }

    return true;
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const* sns_mag_fusion_get_sensor_uid(sns_sensor const *this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid mag_fusion_suid = MAG_FUSION_SUID;
    return &mag_fusion_suid;
}

// check if current bias is far from registry bias
static bool is_registry_write_necessary(float registry_bias_gauss[], float inst_bias_ut[], int inst_accuracy)
{
    if (inst_accuracy == 0)
        return false;

    float distance_gauss[3];
    float distance = 0;

    for (int i = 0; i < 3; i++) {
        distance_gauss[i] = registry_bias_gauss[i] - inst_bias_ut[i] * 0.01f;
        distance += distance_gauss[i] * distance_gauss[i];
    }

    //   if distance is less than 0.03 Gauss, the error of orientation angle will be less than atan(0.03/0.35)=4.899 degrees
    // and it is acceptable when compass is just enabled
    return distance > 0.03f * 0.03f;
}

/* See sns_sensor::set_client_request */
sns_sensor_instance* sns_mag_fusion_set_client_request(
    sns_sensor *const this,
    sns_request const *exist_req,
    sns_request const *new_req,
    bool remove)
{
    sns_sensor_instance *exist_inst = sns_sensor_util_get_shared_instance(this);

    MAG_FUSION_LOGI("%s: exist_req=%p, new_req=%p, exist_inst=%p",
        __func__, exist_req, new_req, exist_inst);

    if(remove) {
        if(NULL != exist_inst)
            exist_inst->cb->remove_client_request(exist_inst, exist_req);
    } else if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == new_req->message_id) {
        if(NULL == exist_inst) {
            // If this is a request from a new client
            exist_inst = this->cb->create_instance(this, sizeof(sns_mag_fusion_inst_state));
        } else {
            if (new_req != NULL) {
                if(NULL != exist_req)
                    exist_inst->cb->remove_client_request(exist_inst, exist_req);
                else
                    encode_send_sns_cal_event(exist_inst,
                        &((sns_mag_fusion_inst_state *)exist_inst->state->state)->last_sample, true);
            }
        }

        if(NULL != exist_inst && NULL != new_req) {
            exist_inst->cb->add_client_request(exist_inst, new_req);
            this->instance_api->set_client_config(exist_inst, new_req);
        }
    } else {
        MAG_FUSION_LOGI("%s: unhandled message id %d", __func__, new_req->message_id);
    }

    if (NULL != exist_inst &&
        NULL == exist_inst->cb->get_client_request(
            exist_inst, this->sensor_api->get_sensor_uid(this), true)) {
        sns_mag_fusion_sensor_state *sensor_state =
            (sns_mag_fusion_sensor_state*)this->state->state;
        sns_mag_fusion_inst_state *inst_state =
            (sns_mag_fusion_inst_state*)exist_inst->state->state;

        bool need_write_back_to_registry =
            is_registry_write_necessary(sensor_state->config.bias,
                inst_state->last_sample.output.bias,
                inst_state->last_sample.output.accuracy);
        // copy back as GAUSS unit
        sensor_state->config.bias[0] = inst_state->last_sample.output.bias[0] * 0.01f;
        sensor_state->config.bias[1] = inst_state->last_sample.output.bias[1] * 0.01f;
        sensor_state->config.bias[2] = inst_state->last_sample.output.bias[2] * 0.01f;
        sensor_state->config.accuracy = inst_state->last_sample.output.accuracy;

        if (need_write_back_to_registry)
            sns_mag_fusion_write_bias_back_to_registry(this);

        this->cb->remove_instance(exist_inst);
    }

    return exist_inst;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_api sns_mag_fusion_api = {
    .struct_len = sizeof(sns_sensor_api),
    .init = &sns_mag_fusion_init,
    .deinit = &sns_mag_fusion_deinit,
    .get_sensor_uid = &sns_mag_fusion_get_sensor_uid,
    .set_client_request = &sns_mag_fusion_set_client_request,
    .notify_event = &sns_mag_fusion_notify_event,
};
