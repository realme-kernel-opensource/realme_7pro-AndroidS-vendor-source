/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: sns_elevator_detect_sensor_instance_island.c
**
** Description:
**      Island mode functions for the elevator detect algorithm instance.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "sns_elevator_detect_sensor_instance.h"
#include "sns_amd.pb.h"

#define ELEVATOR_DETECT_BATCH_PERIOD 100000  // 100ms

sns_sensor_instance *elevator_detect_mTask;

static void elevator_detect_sensor_enable(int sensor_type, bool on)
{
    sns_elevator_detect_inst_state *inst_state = (sns_elevator_detect_inst_state*)elevator_detect_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    ELEVATOR_DETECT_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = elevator_detect_mTask->cb->get_service_manager(elevator_detect_mTask);
    sns_stream_service *stream_service = (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;
    sns_std_sensor_config game_rv_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = ELEVATOR_DETECT_BATCH_PERIOD;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;

    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
        {
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid), &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = inst_state->state->acc_sampleRate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_FIXED;
            resampler_config.filter = true;

            if (NULL == inst_state->resampler_accel_stream) {
                ELEVATOR_DETECT_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, elevator_detect_mTask,
                                        inst_state->resampler_suid, &inst_state->resampler_accel_stream);
            }

            if (NULL != inst_state->resampler_accel_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config, sns_resampler_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request){
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer };

                    rc = inst_state->resampler_accel_stream->api->send_request(inst_state->resampler_accel_stream, &request);
                    ELEVATOR_DETECT_LOG_2("Processed accel config request: sample_rate %f, result %u", resampler_config.resampled_rate, rc);
                }
            } else {
                ELEVATOR_DETECT_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;
        }
        case GAMEROTATION_VECTOR_TYPE:
        {
            game_rv_config.sample_rate = inst_state->state->game_rv_sampleRate;

            if (inst_state->resampler_game_rv_stream == NULL) {
                ELEVATOR_DETECT_LOG_0("create game rv stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, elevator_detect_mTask,
                                        inst_state->game_rv_suid, &inst_state->resampler_game_rv_stream);
            }

            if (NULL != inst_state->resampler_game_rv_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &game_rv_config, sns_std_sensor_config_fields, &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request){
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer };

                    rc = inst_state->resampler_game_rv_stream->api->send_request(inst_state->resampler_game_rv_stream, &request);
                    ELEVATOR_DETECT_LOG_2("Processed game_rv config request: sample_rate %f, result %u", game_rv_config.sample_rate, rc);
                }
            } else {
                ELEVATOR_DETECT_LOG_0("Error in creating game_rv_stream OR encoding failed");
            }
            break;
        }
        case AMD_TYPE:
        {
            if (NULL == inst_state->amd_data_stream) {
                ELEVATOR_DETECT_LOG_0("create resampler amd stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, elevator_detect_mTask,
                                            inst_state->amd_suid, &inst_state->amd_data_stream);
            }

            if (NULL != inst_state->amd_data_stream) {
                ELEVATOR_DETECT_LOG_0("motion recognition sending on-change reqest to AMD");
                sns_request sensor_req = (sns_request) {
                    .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                    .request_len = 0, .request = NULL };

                inst_state->amd_data_stream->api->send_request(inst_state->amd_data_stream, &sensor_req);
            }
            break;
        }
        case TIMER_TYPE:
        {
            if (NULL == inst_state->timer_data_stream) {
                ELEVATOR_DETECT_LOG_0("create resampler timer stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, elevator_detect_mTask,
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

                ELEVATOR_DETECT_LOG_1("Processed timer config request: result %u", rc);
            } else {
                ELEVATOR_DETECT_LOG_0("Error in creating timer stream OR encoding failed");
            }
            break;
        }

        default:
            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(elevator_detect_mTask, &inst_state->resampler_accel_stream);
            break;
        case GAMEROTATION_VECTOR_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(elevator_detect_mTask, &inst_state->resampler_game_rv_stream);
            break;
        case AMD_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(elevator_detect_mTask, &inst_state->amd_data_stream);
            break;
        case TIMER_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(elevator_detect_mTask, &inst_state->timer_data_stream);
            break;
        }
    }
}

static void elevator_detect_report(int value, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float elevator_detect_data[2] = {0};
    sns_elevator_detect_inst_state *inst_state = (sns_elevator_detect_inst_state*)elevator_detect_mTask->state->state;

    elevator_detect_data[0] = value;
    elevator_detect_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(elevator_detect_mTask,
                            NULL,
                            sns_get_system_time(),
                            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                            elevator_detect_data,
                            2,
                            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        ELEVATOR_DETECT_LOG_0("elevator detect report failed, Error in sending event");
    } else {
        ELEVATOR_DETECT_LOG_0("report elevator_detect data.");
    }
}

static uint64_t elevator_detect_get_delta_time_ms(int64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void elevator_detect_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    elevator_detect_sensor_data input;
    bool need_handle = false;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id &&
                    (type_in == ACC_TYPE || type_in == GAMEROTATION_VECTOR_TYPE)) {
        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t){.funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg};

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            ELEVATOR_DETECT_LOG_0("Error in decoding event");
        } else {
            need_handle = true;
        }
    }

    else if (SNS_AMD_MSGID_SNS_AMD_EVENT == event_in->message_id) {
        sns_amd_event amd_data;
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);
        if (!pb_decode(&stream, sns_amd_event_fields, &amd_data)) {
            ELEVATOR_DETECT_LOG_0("Error in decoding amd event");
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
            ELEVATOR_DETECT_LOG_0("Error in decoding event");
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
        elevator_detect_check_sensor_data(&input);
    }
}

static sns_rc sns_elevator_detect_process_event(sns_sensor_instance *const this)
{
    sns_elevator_detect_inst_state *inst_state = (sns_elevator_detect_inst_state*)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *game_rv_event = NULL;
    sns_sensor_event *amd_event = NULL;
    sns_sensor_event *timer_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            elevator_detect_decode_and_checkdata(acc_event, ACC_TYPE);

            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(inst_state->resampler_accel_stream);
            } else {
                ELEVATOR_DETECT_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_game_rv_stream) {
        game_rv_event = inst_state->resampler_game_rv_stream->api->peek_input(inst_state->resampler_game_rv_stream);

        while (NULL != game_rv_event) {
            elevator_detect_decode_and_checkdata(game_rv_event, GAMEROTATION_VECTOR_TYPE);

            if (NULL != inst_state->resampler_game_rv_stream) {
                game_rv_event = inst_state->resampler_game_rv_stream->api->get_next_input(inst_state->resampler_game_rv_stream);
            } else {
                ELEVATOR_DETECT_LOG_0("game_rv stream has been released.");
                game_rv_event = NULL;
            }
        }
    }

    if (NULL != inst_state->amd_data_stream) {
        amd_event = inst_state->amd_data_stream->api->peek_input(inst_state->amd_data_stream);
        while (NULL != amd_event) {
            elevator_detect_decode_and_checkdata(amd_event, AMD_TYPE);
            if (NULL != inst_state->amd_data_stream) {
                amd_event = inst_state->amd_data_stream->api->get_next_input(inst_state->amd_data_stream);
            }
            else {
                ELEVATOR_DETECT_LOG_0("amd stream has been released.");
                amd_event = NULL;
            }
        }
    }

    if (NULL != inst_state->timer_data_stream) {
        timer_event = inst_state->timer_data_stream->api->peek_input(inst_state->timer_data_stream);

        while (NULL != timer_event) {
            elevator_detect_decode_and_checkdata(timer_event, TIMER_TYPE);

            if (NULL != inst_state->timer_data_stream) {
                timer_event = inst_state->timer_data_stream->api->get_next_input(inst_state->timer_data_stream);
            } else {
                ELEVATOR_DETECT_LOG_0("timer stream has been released.");
                timer_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc sns_elevator_detect_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_elevator_detect_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_elevator_detect_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_elevator_detect_inst_init,
    .deinit = &sns_elevator_detect_inst_deinit,
    .set_client_config = &sns_elevator_detect_inst_set_client_config,
    .notify_event = &sns_elevator_detect_inst_notify_event
};

struct elevator_detect_sensor_operation elevator_detect_ops =
{
    .sensor_change_state = elevator_detect_sensor_enable,
    .report = elevator_detect_report,
    .get_delta_time_ms = elevator_detect_get_delta_time_ms,
};

