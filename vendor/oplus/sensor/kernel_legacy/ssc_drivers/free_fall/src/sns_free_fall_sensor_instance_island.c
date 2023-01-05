/************************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: sns_xxx.c
**
** Description:
**      Definitions for free fall detect algorithem .
**
** Version: 1.0
** Date created: 2018/03/09,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "sns_free_fall_sensor_instance.h"
#define FREE_FALL_BATCH_PERIOD 100000  // 100ms

sns_sensor_instance *free_fall_mTask;

static void free_fall_sensor_enable(int sensor_type, bool on)
{
    sns_free_fall_inst_state *inst_state = (sns_free_fall_inst_state *)free_fall_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    float sample_rate = inst_state->state->sampleRate;

    FREE_FALL_LOG_3("sensor enable operate, sensor_type = %d, on = %d, sample_rate = %d", sensor_type,
        on, inst_state->state->sampleRate);

    sns_service_manager *manager = free_fall_mTask->cb->get_service_manager(free_fall_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;
    sns_std_sensor_config game_rv_config = sns_std_sensor_config_init_default;
    sns_std_sensor_config gravity_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = FREE_FALL_BATCH_PERIOD;

    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (inst_state->resampler_accel_stream == NULL) {
                FREE_FALL_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, free_fall_mTask,
                    inst_state->resampler_suid, &inst_state->resampler_accel_stream);
            }

            FREE_FALL_LOG_1("resampler acc config, sample_rate = %d.", (int)sample_rate);

            if (NULL != inst_state->resampler_accel_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config,
                        sns_resampler_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_accel_stream->api->send_request(inst_state->resampler_accel_stream,
                            &request);
                    FREE_FALL_LOG_2("Processed accel config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                FREE_FALL_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;

        case GAMEROTATIONVECTOR_TYPE:
            game_rv_config.sample_rate = sample_rate;

            if (inst_state->resampler_game_rv_stream == NULL) {
                FREE_FALL_LOG_0("create game rv stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, free_fall_mTask,
                    inst_state->game_rv_suid, &inst_state->resampler_game_rv_stream);
            }

            if (NULL != inst_state->resampler_game_rv_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &game_rv_config,
                        sns_std_sensor_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_game_rv_stream->api->send_request(inst_state->resampler_game_rv_stream,
                            &request);
                    FREE_FALL_LOG_2("Processed game_rv config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                FREE_FALL_LOG_0("Error in creating game_rv_stream OR encoding failed");
            }

            break;

        case GRAVITY_TYPE:
            gravity_config.sample_rate = sample_rate;

            if (inst_state->resampler_gravity_stream == NULL) {
                FREE_FALL_LOG_0("create linacc gravity stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, free_fall_mTask,
                    inst_state->gravity_suid, &inst_state->resampler_gravity_stream);
            }

            if (NULL != inst_state->resampler_gravity_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &gravity_config,
                        sns_std_sensor_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_gravity_stream->api->send_request(inst_state->resampler_gravity_stream,
                            &request);
                    FREE_FALL_LOG_2("Processed gravity config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                FREE_FALL_LOG_0("Error in creating gravity_stream OR encoding failed");
            }

            break;

        case LINACC_TYPE:
            FREE_FALL_LOG_0("Has been enable in GRAVITY_TYPE");

            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(free_fall_mTask, &inst_state->resampler_accel_stream);
            break;
        case GRAVITY_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(free_fall_mTask,
                &inst_state->resampler_gravity_stream);
            break;
        case LINACC_TYPE:
            break;
        case GAMEROTATIONVECTOR_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(free_fall_mTask,
                &inst_state->resampler_game_rv_stream);
            break;
        }
    }
}

static void free_fall_report(uint32_t free_fall_time, uint32_t angle, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float free_fall_data[3];
    sns_free_fall_inst_state *inst_state = (sns_free_fall_inst_state *)free_fall_mTask->state->state;

    free_fall_data[0] = free_fall_time;
    free_fall_data[1] = angle;
    free_fall_data[2] = (float)report_count;

    rc = pb_send_sensor_stream_event(free_fall_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            free_fall_data,
            3,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        FREE_FALL_LOG_0("free fall report failed, Error in sending event");
    } else {
        FREE_FALL_LOG_0("report free_fall data.");
    }
}

static uint64_t free_fall_get_delta_time_ms(uint64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void free_fall_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    free_fall_sensor_data input;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
        //MOTION_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in, event_in->message_id, event_in->timestamp);

        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {
            .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
        };

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            FREE_FALL_LOG_0("Error in decoding event");
        } else {
            //FREE_FALL_LOG_4("data[%d %d %d]", type_in, (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
        }

        input.x = data[0];
        input.y = data[1];
        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        free_fall_process_sensor_data(&input);

        if (type_in == GRAVITY_TYPE) {
            input.x = data[3];
            input.y = data[4];
            input.z = data[5];
            input.timestamp = event_in->timestamp;
            input.type = LINACC_TYPE;

            free_fall_process_sensor_data(&input);
        }
    }
}

static sns_rc sns_free_fall_process_event(sns_sensor_instance *const this)
{
    sns_free_fall_inst_state *inst_state = (sns_free_fall_inst_state *)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *gravity_event = NULL;
    sns_sensor_event *game_rv_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            free_fall_decode_and_checkdata(acc_event, ACC_TYPE);

            // stream maybe released in free_fall_decode_and_checkdata(), so must judge this stream
            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(
                        inst_state->resampler_accel_stream);
            } else {
                // if stream has been released, event must be set to NULL to break the while()
                FREE_FALL_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_gravity_stream) {
        gravity_event = inst_state->resampler_gravity_stream->api->peek_input(
                inst_state->resampler_gravity_stream);

        while (NULL != gravity_event) {
            free_fall_decode_and_checkdata(gravity_event, GRAVITY_TYPE);

            if (NULL != inst_state->resampler_gravity_stream) {
                gravity_event = inst_state->resampler_gravity_stream->api->get_next_input(
                        inst_state->resampler_gravity_stream);
            } else {
                FREE_FALL_LOG_0("gravity stream has been released.");
                gravity_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_game_rv_stream) {
        game_rv_event = inst_state->resampler_game_rv_stream->api->peek_input(
                inst_state->resampler_game_rv_stream);

        while (NULL != game_rv_event) {
            free_fall_decode_and_checkdata(game_rv_event, GAMEROTATIONVECTOR_TYPE);

            if (NULL != inst_state->resampler_game_rv_stream) {
                game_rv_event = inst_state->resampler_game_rv_stream->api->get_next_input(
                        inst_state->resampler_game_rv_stream);
            } else {
                FREE_FALL_LOG_0("game rv stream has been released.");
                game_rv_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc sns_free_fall_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_free_fall_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_free_fall_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_free_fall_inst_init,
    .deinit = &sns_free_fall_inst_deinit,
    .set_client_config = &sns_free_fall_inst_set_client_config,
    .notify_event = &sns_free_fall_inst_notify_event
};

struct free_fall_sensor_operation free_fall_ops = {
    .sensor_change_state = free_fall_sensor_enable,
    .report = free_fall_report,
    .get_delta_time_ms = free_fall_get_delta_time_ms,
};

