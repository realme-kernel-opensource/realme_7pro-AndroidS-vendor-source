/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: sns_motion_recognition_sensor_instance_island.c
**
** Description:
**      The specific algorithm for how to detect motion state.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_motion_recognition_sensor_instance.h"
#include "sns_remote_proc_state.pb.h"
#include "sns_amd.pb.h"

#define MOTION_RECOGNITION_BATCH_PERIOD 1000000  // 1000ms

sns_sensor_instance *motion_recognition_mTask;
extern sns_sensor_api sns_motion_recognition_api;

static void motion_recognition_sensor_enable(int sensor_type, bool on)
{
    sns_motion_recognition_inst_state *inst_state = (sns_motion_recognition_inst_state *) motion_recognition_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    MOTION_RECOGNITION_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = motion_recognition_mTask->cb->get_service_manager(motion_recognition_mTask);
    sns_stream_service *stream_service = (sns_stream_service *) manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = MOTION_RECOGNITION_BATCH_PERIOD;

    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};
    if (on) {
        switch (sensor_type) {
        case ACC_20_TYPE:
        {
            sns_resampler_config resampler_config = sns_resampler_config_init_default;
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid), &inst_state->accel_suid,
                        sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = inst_state->state->acc_sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_FIXED;
            resampler_config.filter = true;

            if (NULL == inst_state->resampler_accel_20_stream) {
                MOTION_RECOGNITION_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, motion_recognition_mTask,
                                        inst_state->resampler_suid, &inst_state->resampler_accel_20_stream);
            }

            if (NULL != inst_state->resampler_accel_20_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config, sns_resampler_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request){
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer };

                    rc = inst_state->resampler_accel_20_stream->api->send_request(inst_state->resampler_accel_20_stream, &request);
                    MOTION_RECOGNITION_LOG_2("Processed accel config request: sample_rate %f, result %u", resampler_config.resampled_rate, rc);
                }
            } else {
                MOTION_RECOGNITION_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;
        }
        case AMD_TYPE:
        {
            if (NULL == inst_state->amd_data_stream) {
                MOTION_RECOGNITION_LOG_0("create resampler amd stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, motion_recognition_mTask,
                                            inst_state->amd_suid, &inst_state->amd_data_stream);
            }

            if (NULL != inst_state->amd_data_stream) {
                MOTION_RECOGNITION_LOG_0("motion recognition sending on-change reqest to AMD");
                sns_request sensor_req = (sns_request){
                    .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                    .request_len = 0, .request = NULL };

                    inst_state->amd_data_stream->api->send_request(inst_state->amd_data_stream, &sensor_req);
            }
            break;
        }
        case TIMER_TYPE:
        {
            if (NULL == inst_state->timer_data_stream) {
                MOTION_RECOGNITION_LOG_0("create resampler timer stream.");

            // create connection with resampler sensor
            stream_service->api->create_sensor_instance_stream(stream_service, motion_recognition_mTask,
                                        inst_state->timer_suid, &inst_state->timer_data_stream);
            }
            float timeout = 10;
            if (NULL != inst_state->timer_data_stream) {
                sns_request timer_req =
                {
                    .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
                    .request    = buffer
                };

                req_payload.is_periodic    = true;
                req_payload.start_time     = sns_get_system_time();

                req_payload.timeout_period =  sns_convert_ns_to_ticks((sns_time)((sns_time)timeout* 1000 * 1000 * 1000));
                timer_req.request_len      = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);

                if (timer_req.request_len > 0) {
                    inst_state->timer_data_stream->api->send_request(inst_state->timer_data_stream, &timer_req);
                }
                MOTION_RECOGNITION_LOG_1("Processed timer config request: result %u", rc);
            } else {
                MOTION_RECOGNITION_LOG_0("Error in creating timer stream OR encoding failed");
            }
            break;
        }
        default:
            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_20_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(motion_recognition_mTask, &inst_state->resampler_accel_20_stream);
            break;
        case AMD_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(motion_recognition_mTask, &inst_state->amd_data_stream);
            break;
        case TIMER_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(motion_recognition_mTask, &inst_state->timer_data_stream);
            break;
        default:
            break;
        }
    }
}

static void motion_recognition_report(int count, int incar, int activity, uint64_t delta_time)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float motion_recognition_data[3] = {0}; // TODO: length depends on the event size which return to framework
    sns_motion_recognition_inst_state *inst_state = (sns_motion_recognition_inst_state *) motion_recognition_mTask->state->state;

    motion_recognition_data[0] = count;
    motion_recognition_data[1] = incar;
    motion_recognition_data[2] = activity;
    rc = pb_send_sensor_stream_event(motion_recognition_mTask,
                                     NULL,
                                     delta_time,
                                     SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                     SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                     motion_recognition_data,
                                     3,
                                     inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        MOTION_RECOGNITION_LOG_0("motion_recognition report failed, Error in sending event");
    } else {
        MOTION_RECOGNITION_LOG_0("report motion_recognition data.");
    }
}

static uint64_t motion_recognition_get_delta_time_ms(uint64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void motion_recognition_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    motion_recognition_sensor_data input;
    bool need_handle = false;

    if ((SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id)&&(ACC_20_TYPE == type_in)) {
        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {.funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg};

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *) event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            MOTION_RECOGNITION_LOG_1("Error in decoding event sensor_type= %d", type_in);
        } else {
            need_handle = true;
        }
    }

    else if (SNS_AMD_MSGID_SNS_AMD_EVENT == event_in->message_id) {
        sns_amd_event amd_data;
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);
        if (!pb_decode(&stream, sns_amd_event_fields, &amd_data)) {
            MOTION_RECOGNITION_LOG_0("Error in decoding amd event");
        } else {
            need_handle = true;
            if (amd_data.state == SNS_AMD_EVENT_TYPE_STATIONARY) {
                data[0] = AMD_STATIONARY;
            } else {
                data[0] = AMD_MOVE;
            }
        }
    }

    else if ((SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event_in->message_id) && (TIMER_TYPE == type_in)) {
        sns_timer_sensor_event timer_event;

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_timer_sensor_event_fields, &timer_event)) {
            MOTION_RECOGNITION_LOG_0("Error in decoding event");
        } else {
            need_handle = true;
        }
    }
    if (true == need_handle) {
        input.x_value = data[0];
        input.y_value = data[1];
        input.z_value = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;
        motion_recognition_check_sensor_data(&input);
    }
}

static sns_rc sns_motion_recognition_process_event(sns_sensor_instance *const this)
{
    sns_motion_recognition_inst_state *inst_state = (sns_motion_recognition_inst_state *) this->state->state;

    sns_sensor_event *acc_20_event = NULL;
    sns_sensor_event *amd_event = NULL;
    sns_sensor_event *timer_event = NULL;

    if (NULL != inst_state->resampler_accel_20_stream) {
        acc_20_event = inst_state->resampler_accel_20_stream->api->peek_input(inst_state->resampler_accel_20_stream);

        while (NULL != acc_20_event) {
            motion_recognition_decode_and_checkdata(acc_20_event, ACC_20_TYPE);

            if (NULL != inst_state->resampler_accel_20_stream) {
                acc_20_event = inst_state->resampler_accel_20_stream->api->get_next_input(
                        inst_state->resampler_accel_20_stream);
            } else {
                MOTION_RECOGNITION_LOG_0("accel_20 stream has been released.");
                acc_20_event = NULL;
            }
        }
    }

    if (NULL != inst_state->amd_data_stream) {
        amd_event = inst_state->amd_data_stream->api->peek_input(inst_state->amd_data_stream);
        while (NULL != amd_event) {
            motion_recognition_decode_and_checkdata(amd_event, AMD_TYPE);
            if (NULL != inst_state->amd_data_stream) {
                amd_event = inst_state->amd_data_stream->api->get_next_input(inst_state->amd_data_stream);
            }
            else {
                MOTION_RECOGNITION_LOG_0("amd stream has been released.");
                amd_event = NULL;
            }
        }
    }

    if (NULL != inst_state->timer_data_stream) {
        timer_event = inst_state->timer_data_stream->api->peek_input(inst_state->timer_data_stream);

        while (NULL != timer_event) {
            motion_recognition_decode_and_checkdata(timer_event, TIMER_TYPE);

            if (NULL != inst_state->timer_data_stream) {
                timer_event = inst_state->timer_data_stream->api->get_next_input(inst_state->timer_data_stream);
            } else {
                MOTION_RECOGNITION_LOG_0("timer stream has been released.");
                timer_event = NULL;
            }
        }
    }

    // TODO: read sensor input from stream

    return SNS_RC_SUCCESS;
}

static sns_rc sns_motion_recognition_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_motion_recognition_process_event(this);

    return rc;
}

static void* sns_motion_recognition_malloc(size_t size)
{
    void *result = NULL;
    result = sns_malloc(SNS_HEAP_ISLAND, size);
    if (result == NULL) {
        SNS_ISLAND_EXIT();
        result = sns_malloc(SNS_HEAP_MAIN, size);
        SNS_ASSERT(result != NULL);
    }
    memset(result, 0, size);
    return result;
}

static void sns_motion_recognition_free(void* ptr)
{
    if (!sns_island_is_island_ptr((intptr_t)ptr)) {
        SNS_ISLAND_EXIT();
        sns_free(ptr);
    } else {
        sns_free(ptr);
    }
}

sns_sensor_instance_api sns_motion_recognition_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_motion_recognition_inst_init,
    .deinit = &sns_motion_recognition_inst_deinit,
    .set_client_config = &sns_motion_recognition_inst_set_client_config,
    .notify_event = &sns_motion_recognition_inst_notify_event
};

struct motion_recognition_sensor_operation motion_recognition_ops =
{
    .sensor_change_state = motion_recognition_sensor_enable,
    .report = motion_recognition_report,
    .get_delta_time_ms = motion_recognition_get_delta_time_ms,

    .get_current_time_tick = sns_get_system_time,

    .malloc = sns_motion_recognition_malloc,
    .free = sns_motion_recognition_free,
};


