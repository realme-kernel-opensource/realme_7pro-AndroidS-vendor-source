/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "oplus_devorient_instance.h"
#include "sns_amd.pb.h"

sns_sensor_instance *devorient_mTask;

static void devorient_sensor_enable(int sensor_type, bool on)
{
    devorient_inst_state *inst_state = (devorient_inst_state*)devorient_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    DEVORIENT_LOG_DEBUG("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = devorient_mTask->cb->get_service_manager(devorient_mTask);
    sns_stream_service *stream_service = (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_std_sensor_config acc_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
            acc_config.sample_rate = inst_state->state->acc_sampleRate;

            if (NULL == inst_state->resampler_acc_stream) {
                DEVORIENT_LOG_DEBUG("create resampler amd stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, devorient_mTask,
                    inst_state->acc_suid, &inst_state->resampler_acc_stream);
            }

            if (NULL != inst_state->resampler_acc_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &acc_config, sns_std_sensor_config_fields, NULL);

                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_acc_stream->api->send_request(inst_state->resampler_acc_stream, &request);
                    DEVORIENT_LOG_DEBUG("Processed acc config request: sample_rate %d, result %u", inst_state->state->acc_sampleRate, rc);
                }
            } else {
                DEVORIENT_LOG_INFO("Error in creating accel_stream OR encoding failed");
            }

            break;

        case AMD_TYPE:
            if (NULL == inst_state->resampler_amd_stream) {
                DEVORIENT_LOG_DEBUG("create resampler amd stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, devorient_mTask,
                    inst_state->amd_suid, &inst_state->resampler_amd_stream);
            }

            if (NULL != inst_state->resampler_amd_stream) {
                DEVORIENT_LOG_DEBUG("Sending on-change reqest to AMD");
                sns_request sensor_req = (sns_request) {
                    .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                    .request_len = 0, .request = NULL
                };

                inst_state->resampler_amd_stream->api->send_request(inst_state->resampler_amd_stream, &sensor_req);
            }

            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(devorient_mTask, &inst_state->resampler_acc_stream);
            break;

        case AMD_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(devorient_mTask, &inst_state->resampler_amd_stream);
            break;
        }
    }
}

static void devorient_report(int move, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float devorient_data[2] = {0};
    devorient_inst_state *inst_state = (devorient_inst_state*)devorient_mTask->state->state;

    devorient_data[0] = move;
    devorient_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(devorient_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            devorient_data,
            2,
            inst_state->config.encoded_data_event_len);

    if(SNS_RC_SUCCESS != rc) {
        DEVORIENT_LOG_INFO("device oritation report failed, Error in sending event");
    }
}

static uint64_t devorient_get_time_ms(uint64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void devorient_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    devorient_sensor_data input;
    sns_amd_event amd_data = sns_amd_event_init_default;
    bool need_handle = false;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {
            .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
        };

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            DEVORIENT_LOG_INFO("Error in decoding event");
        } else {
            need_handle = true;

            DEVORIENT_LOG_INFO("data[%d %d %d], ts %llu",
                (int)(1000 * data[0]), (int)(1000 * data[1]),
                (int)(1000 * data[2]), event_in->timestamp);
        }
    }

    if(SNS_AMD_MSGID_SNS_AMD_EVENT == event_in->message_id) {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if(!pb_decode(&stream, sns_amd_event_fields, &amd_data)) {
            DEVORIENT_LOG_INFO("Error in decoding amd event");
        } else {
            need_handle = true;

            if(amd_data.state == SNS_AMD_EVENT_TYPE_STATIONARY) {
                data[0] = AMD_STATIONARY;
            } else {
                data[0] = AMD_MOVE;
            }
        }
    }

    if (true == need_handle) {
        input.x = data[0];
        input.y = data[1];
        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        devorient_algo_check_sensor_data(&input);
    }
}

static sns_rc devorient_process_event(sns_sensor_instance *const this)
{
    devorient_inst_state *inst_state = (devorient_inst_state*)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *amd_event = NULL;

    if (NULL != inst_state->resampler_acc_stream) {
        acc_event = inst_state->resampler_acc_stream->api->peek_input(inst_state->resampler_acc_stream);

        while (NULL != acc_event) {
            devorient_decode_and_checkdata(acc_event, ACC_TYPE);

            if (NULL != inst_state->resampler_acc_stream) {
                acc_event = inst_state->resampler_acc_stream->api->get_next_input(inst_state->resampler_acc_stream);
            } else {
                DEVORIENT_LOG_INFO("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_amd_stream) {
        amd_event = inst_state->resampler_amd_stream->api->peek_input(inst_state->resampler_amd_stream);

        while (NULL != amd_event) {
            devorient_decode_and_checkdata(amd_event, AMD_TYPE);

            if (NULL != inst_state->resampler_amd_stream) {
                amd_event = inst_state->resampler_amd_stream->api->get_next_input(inst_state->resampler_amd_stream);
            } else {
                DEVORIENT_LOG_INFO("accel stream has been released.");
                amd_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc devorient_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    devorient_process_event(this);

    return rc;
}

sns_sensor_instance_api devorient_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &devorient_inst_init,
    .deinit = &devorient_inst_deinit,
    .set_client_config = &devorient_inst_set_client_config,
    .notify_event = &devorient_inst_notify_event
};

struct devorient_sensor_operation devorient_ops = {
    .sensor_change_state = devorient_sensor_enable,
    .report = devorient_report,
    .get_time_ms = devorient_get_time_ms,
};