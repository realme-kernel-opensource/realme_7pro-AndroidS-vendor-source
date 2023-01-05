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
#include "sns_mag_fusion_sensor.h"
#include "sns_mag_fusion_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_types.h"
#include "sns_suid.pb.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#include "sns_stream_service.h"
#include "sns_std_sensor.pb.h"

// API used by registry
#include "sns_registry.pb.h"

/*=============================================================================
  Function Definitions
  ===========================================================================*/
/**
 * Initialize attributes to their default state.  They may/will be updated
 * within mag_fusion_notify.
 */
static void publish_attributes(sns_sensor *const this)
{
    // Set Name
    {
        char const name[] = "mag_fusion";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = name, .buf_len = sizeof(name)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
    // Set Type
    {
        // INFO: see AP:sensors-see/sensors-hal/sensors/magnetometer.cpp:16
        //       -> static const char *SSC_DATATYPE_MAGCAL = "mag_cal";
        // if change sensor type here, SSC_DATATYPE_MAGCAL should be changed as also.
        char const type[] = "mag_cal";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = type, .buf_len = sizeof(type)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }
    // Set Vendor
    {
        char const vendor[] = "oplus";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = vendor, .buf_len = sizeof(vendor)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VENDOR,
            &value, 1, false);
    }
    // Set proto interface
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        char const proto1[] = "sns_mag_cal.proto";
        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = proto1, .buf_len = sizeof(proto1)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
            values, ARR_SIZE(values), false);
    }
    // Set Stream Type
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint     = SNS_STD_SENSOR_STREAM_TYPE_ON_CHANGE;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
            &value, 1, false);
    }
    // Set Data Rate
    {
        //TODO: make magic numbers macros
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = 5.0f;
        values[1].has_flt = true;
        values[1].flt = 200.0f;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
            values, ARR_SIZE(values), false);
    }
    // Set Version
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = 1;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VERSION,
            &value, 1, true);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = 0;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RIGID_BODY,
            &value, 1, false);
    }
}

static void sns_mag_fusion_publish_available(sns_sensor *const this)
{
    MAG_FUSION_LOGI("%s", __func__);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE,
        &value, 1, true);
}

static void sns_mag_fusion_parse_smem_info(sns_sensor *const this)
{
    sns_mag_fusion_sensor_state *state =
        (sns_mag_fusion_sensor_state*)this->state->state;

    state->has_gyro = true;
    state->fusion_type = OPLUS_FUSION;
}
/* See sns_sensor::init */
sns_rc sns_mag_fusion_init(sns_sensor *const this)
{
    sns_mag_fusion_sensor_state *state = (sns_mag_fusion_sensor_state*)this->state->state;
    struct sns_service_manager *smgr = this->cb->get_service_manager(this);
    //MAG_FUSION_LOGI("sns_mag_fusion_init");

    state->diag_service = (sns_diag_service*)
        smgr->get_service(smgr, SNS_DIAG_SERVICE);

    SNS_SUID_LOOKUP_INIT(state->suid_lookup_data, NULL);
    sns_suid_lookup_add(this, &state->suid_lookup_data, "registry");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "accel");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "mag");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "gyro");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "gyro_cal");
    sns_suid_lookup_add(this, &state->suid_lookup_data, "resampler");

    state->config.accuracy = 0;
    state->config.bias[0] = 0.0f;
    state->config.bias[1] = 0.0f;
    state->config.bias[2] = 0.0f;

    // select fusion_api
    sns_mag_fusion_parse_smem_info(this);
    state->fusion_api = mag_fusion_get_fusion_list(state->fusion_type);

    // init algo struct
    if(state->fusion_api->algo_init) {
        state->fusion_api->algo_init(state->has_gyro, (mag_fusion_output) {
            .bias = {0, 0, 0},
            .accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE,
            .result_changed = false
        });
    }

    publish_attributes(this);
    return SNS_RC_SUCCESS;
}

/* See sns_sensor::deinit */
sns_rc sns_mag_fusion_deinit(sns_sensor *const this)
{
    sns_mag_fusion_sensor_state *state = (sns_mag_fusion_sensor_state*)this->state->state;

    //MAG_FUSION_LOGI("sns_mag_fusion_deinit");
    if (state->registry_stream != NULL) {
        sns_sensor_util_remove_sensor_stream(this, &state->registry_stream);
        state->registry_stream = NULL;
    }

    // deinit algo struct
    if(state->fusion_api->algo_deinit)
        state->fusion_api->algo_deinit();

    return SNS_RC_SUCCESS;
}

static void sns_mag_fusion_registry_req(sns_sensor *const this)
{
    MAG_FUSION_LOGI("%s", __func__);

    sns_service_manager* service_mgr =
        this->cb->get_service_manager(this);
    sns_mag_fusion_sensor_state *state =
        (sns_mag_fusion_sensor_state*)this->state->state;
    sns_stream_service* stream_service =
        (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    uint8_t buffer[100];
    size_t encoded_len;
    sns_sensor_uid registry_suid;
    sns_suid_lookup_get(&state->suid_lookup_data, "registry", &registry_suid);

    //Create a data stream to Registry
    stream_service->api->create_sensor_stream(stream_service,
        this,
        registry_suid,
        &state->registry_stream);

    //Send a request to Registry
    sns_registry_read_req sns_mag_fusion_registry;
    pb_buffer_arg data = (pb_buffer_arg) {
        .buf = MAG_FUSION_GROUP_NAME,
        .buf_len = (strlen(MAG_FUSION_GROUP_NAME) + 1)
    };

    sns_mag_fusion_registry.name.arg = &data;
    sns_mag_fusion_registry.name.funcs.encode = pb_encode_string_cb;
    sns_std_request registry_std_req = sns_std_request_init_default;
    encoded_len = pb_encode_request(buffer,
            sizeof(buffer),
            &sns_mag_fusion_registry,
            sns_registry_read_req_fields,
            &registry_std_req);

    if(0 < encoded_len && NULL != state->registry_stream ) {
        sns_request request = (sns_request) {
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ,
            .request_len = encoded_len, .request = buffer
        };
        state->registry_stream->api->send_request(
            state->registry_stream, &request);
    }
}

static bool sns_mag_fusion_parse_registry(pb_istream_t *stream,
    const pb_field_t *field,
    void **arg)
{
    UNUSED_VAR(field);
    sns_registry_data_item reg_item;
    pb_buffer_arg item_name = {0, 0};
    reg_item.name.arg = &item_name;
    reg_item.name.funcs.decode = pb_decode_string_cb;

    bool rv = pb_decode(stream, sns_registry_data_item_fields, &reg_item);
    sns_mag_fusion_config *config = (sns_mag_fusion_config *)*arg;

#define PARSE(field, name, type) \
	else if (0 == strncmp((char*)item_name.buf, name, item_name.buf_len)) { \
		if (reg_item.has_##type) { \
			config->field = reg_item.type; \
		} else { \
			rv = false; \
		} \
	}
#define PARSE_SINT(field, name) PARSE(field, name, sint)
#define PARSE_FLT(field, name) PARSE(field, name, flt)

    if (false) {}

    PARSE_SINT(accuracy, "accuracy")
    PARSE_FLT(bias[0], "bias_0")
    PARSE_FLT(bias[1], "bias_1")
    PARSE_FLT(bias[2], "bias_2")

#undef PARSE_FLT
#undef PARSE_SINT
#undef PARSE
    // if(0 == strncmp((char*)item_name.buf, "accuracy", item_name.buf_len))
    // {
    //   if (reg_item.has_sint)
    //   {
    //     config->accuracy = reg_item.sint;
    //   }
    //   else
    //   {
    //     rv = false;
    //   }
    // }
    // else if(0 == strncmp((char*)item_name.buf, "bias_x", item_name.buf_len))
    // {
    //   if (reg_item.has_flt)
    //   {
    //     config->bias[0] = reg_item.flt;
    //   }
    //   else
    //   {
    //     rv = false;
    //   }
    // }
    // else if(0 == strncmp((char*)item_name.buf, "bias_y", item_name.buf_len))
    // {
    //   if (reg_item.has_flt)
    //   {
    //     config->bias[1] = reg_item.flt;
    //   }
    //   else
    //   {
    //     rv = false;
    //   }
    // }
    // else if(0 == strncmp((char*)item_name.buf, "bias_z", item_name.buf_len))
    // {
    //   if (reg_item.has_flt)
    //   {
    //     config->bias[2] = reg_item.flt;
    //   }
    //   else
    //   {
    //     rv = false;
    //   }
    // }

    return rv;
}

static sns_rc handle_mag_fusion_registry_event(sns_sensor *const this)
{
    sns_rc rv = SNS_RC_SUCCESS;
    sns_sensor_event *event;
    sns_mag_fusion_sensor_state *state =
        (sns_mag_fusion_sensor_state*)this->state->state;

    MAG_FUSION_LOGI("%s", __func__);

    for(event = state->registry_stream->api->peek_input(state->registry_stream);
        NULL != event;
        event = (NULL == state->registry_stream) ? NULL :
            state->registry_stream->api->get_next_input(state->registry_stream)) {

        if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id) {
            pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
                    event->event_len);
            sns_registry_read_event read_event = sns_registry_read_event_init_default;
            pb_buffer_arg group_name = {0, 0};

            read_event.name.arg = &group_name;
            read_event.name.funcs.decode = pb_decode_string_cb;
            read_event.data.items.arg = &state->config;
            read_event.data.items.funcs.decode = sns_mag_fusion_parse_registry;

            if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event)) {
                MAG_FUSION_LOGI("Error decoding registry event");
            } else if(0 == strncmp((char*)group_name.buf, MAG_FUSION_GROUP_NAME, group_name.buf_len)) {
                MAG_FUSION_LOGI("bias=[%d, %d, %d]/1000 accuracy=%d",
                    (int)(state->config.bias[0] * 1000),
                    (int)(state->config.bias[1] * 1000),
                    (int)(state->config.bias[2] * 1000),
                    state->config.accuracy);

                // re-init algo struct
                if(state->fusion_api->algo_init) {
                    state->fusion_api->algo_init(state->has_gyro, (mag_fusion_output) {
                        .bias = { // copy from GAUSS to uT
                            state->config.bias[0] * 100.0f,
                            state->config.bias[1] * 100.0f,
                            state->config.bias[2] * 100.0f
                        },
                        .accuracy = state->config.accuracy,
                        .result_changed = true
                    });
                }
            } else {
                MAG_FUSION_LOGI("config from registry failed");
            }
        } else {
            MAG_FUSION_LOGI("Unknown event received %d", event->message_id);
        }
    }

    return rv;
}

/* See sns_sensor::notify_event */
sns_rc sns_mag_fusion_notify_event(sns_sensor *const this)
{
    sns_mag_fusion_sensor_state *state = (sns_mag_fusion_sensor_state*)this->state->state;
    sns_suid_lookup_handle(this, &state->suid_lookup_data);

    if(NULL != state->registry_stream)
        handle_mag_fusion_registry_event(this);

    if(sns_suid_lookup_complete(&state->suid_lookup_data)) {
        if (state->fusion_api)
            sns_mag_fusion_publish_available(this);

        sns_suid_lookup_deinit(this, &state->suid_lookup_data);

        if (!state->registry_requested) {
            sns_mag_fusion_registry_req(this);
            state->registry_requested = true;
        }
    }

    return SNS_RC_SUCCESS;
}
