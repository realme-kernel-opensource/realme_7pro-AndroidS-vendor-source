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

#include "sns_pickup_detect_sensor_instance.h"
#define PICKUP_DETECT_BATCH_PERIOD 100000  // 100ms

sns_sensor_instance *pickup_detect_mTask;

static void pickup_detect_sensor_enable(int sensor_type, bool on)
{
    sns_pickup_detect_inst_state *inst_state = (sns_pickup_detect_inst_state *)
        pickup_detect_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    float sample_rate = inst_state->state->sampleRate;

    PICKUP_DETECT_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = pickup_detect_mTask->cb->get_service_manager(pickup_detect_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;

    sns_std_sensor_config gravity_config = sns_std_sensor_config_init_default;
    sns_std_sensor_config prox_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = PICKUP_DETECT_BATCH_PERIOD;

    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_accel_stream) {
                PICKUP_DETECT_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, pickup_detect_mTask,
                    inst_state->resampler_suid, &inst_state->resampler_accel_stream);
            }

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
                    PICKUP_DETECT_LOG_2("Processed accel config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                PICKUP_DETECT_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;

        case GYRO_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->gyro_suid, sizeof(inst_state->gyro_suid));

            resampler_config.resampled_rate = sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_gyro_stream) {
                PICKUP_DETECT_LOG_0("create resampler gyro stream.");
                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, pickup_detect_mTask,
                    inst_state->resampler_suid, &inst_state->resampler_gyro_stream);
            }

            if (NULL != inst_state->resampler_gyro_stream) {
#ifndef OPLUS_FEATURE_SENSOR_BASELINE
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config,
                        sns_resampler_config_fields, NULL);
#else
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config,
                        sns_resampler_config_fields, &std_req);
#endif
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_gyro_stream->api->send_request(inst_state->resampler_gyro_stream,
                            &request);
                    PICKUP_DETECT_LOG_2("Processed gyro config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                PICKUP_DETECT_LOG_0("Error in creating gyro_stream OR encoding failed");
            }

            break;

        case GRAVITY_TYPE:
            gravity_config.sample_rate = sample_rate;

            if (NULL == inst_state->resampler_gravity_stream) {
                PICKUP_DETECT_LOG_0("create gravity stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, pickup_detect_mTask,
                    inst_state->gravity_suid, &inst_state->resampler_gravity_stream);
            }

            if (NULL != inst_state->resampler_gravity_stream) {
#ifndef OPLUS_FEATURE_SENSOR_BASELINE
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &gravity_config,
                        sns_std_sensor_config_fields, NULL);
#else
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &gravity_config,
                        sns_std_sensor_config_fields, &std_req);
#endif
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_gravity_stream->api->send_request(inst_state->resampler_gravity_stream,
                            &request);
                    PICKUP_DETECT_LOG_2("Processed gravity config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                PICKUP_DETECT_LOG_0("Error in creating gravity_stream OR encoding failed");
            }

            break;

        case PROX_TYPE:
            prox_config.sample_rate = sample_rate;

            if (NULL == inst_state->resampler_prox_stream) {
                PICKUP_DETECT_LOG_0("create prox stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, pickup_detect_mTask,
                    inst_state->prox_suid, &inst_state->resampler_prox_stream);
            }

            if (NULL != inst_state->resampler_prox_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &prox_config, sns_std_sensor_config_fields,
                        NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_prox_stream->api->send_request(inst_state->resampler_prox_stream,
                            &request);
                    PICKUP_DETECT_LOG_2("Processed prox config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                PICKUP_DETECT_LOG_0("Error in creating prox_stream OR encoding failed");
            }

            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(pickup_detect_mTask,
                &inst_state->resampler_accel_stream);
            break;
        case GYRO_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(pickup_detect_mTask,
                &inst_state->resampler_gyro_stream);
            break;
        case GRAVITY_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(pickup_detect_mTask,
                &inst_state->resampler_gravity_stream);
            break;
        case PROX_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(pickup_detect_mTask,
                &inst_state->resampler_prox_stream);
            break;
        }
    }
}

static void pickup_detect_report(int value, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float pickup_detect_data[2] = {0};
    sns_pickup_detect_inst_state *inst_state = (sns_pickup_detect_inst_state *)
        pickup_detect_mTask->state->state;

    pickup_detect_data[0] = value;
    pickup_detect_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(pickup_detect_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            pickup_detect_data,
            2,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        PICKUP_DETECT_LOG_0("free fall report failed, Error in sending event");
    } else {
        PICKUP_DETECT_LOG_0("report pickup_detect data.");
    }
}

static uint64_t pickup_detect_get_delta_time_ms(uint64_t timestamp)
{
    return  sns_get_ms_time_from_tick(timestamp);
}

static bool pickup_detect_need_check_prox(void)
{
    sns_pickup_detect_inst_state *inst_state = (sns_pickup_detect_inst_state *)
        pickup_detect_mTask->state->state;
    return inst_state->need_prox;
}

static void pickup_detect_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    pickup_detect_sensor_data input;
    bool need_handle = false;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
        PICKUP_DETECT_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in,
            event_in->message_id, event_in->timestamp);

        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {
            .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
        };

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            PICKUP_DETECT_LOG_0("Error in decoding event");
        } else {
            need_handle = true;
            PICKUP_DETECT_LOG_3("data[%d %d %d]", (int)(1000 * data[0]), (int)(1000 * data[1]),
                (int)(1000 * data[2]));
        }
    } else if (event_in->message_id == SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT) {
        sns_proximity_event decoded_event = sns_proximity_event_init_default;

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_proximity_event_fields, &decoded_event)) {
            PICKUP_DETECT_LOG_0("Error in decoding prox event");
        } else {
            need_handle = true;

            data[0] = decoded_event.proximity_event_type;
            data[1] = decoded_event.raw_adc;
        }
    }
    if (true == need_handle) {
        input.x = data[0];
        input.y = data[1];
        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        pickup_detect_process_sensor_data(&input);
    }
}

static sns_rc sns_pickup_detect_process_event(sns_sensor_instance *const this)
{
    sns_pickup_detect_inst_state *inst_state = (sns_pickup_detect_inst_state *)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *gyro_event = NULL;
    sns_sensor_event *gravity_event = NULL;
    sns_sensor_event *prox_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            pickup_detect_decode_and_checkdata(acc_event, ACC_TYPE);

            // stream maybe released in pickup_detect_decode_and_checkdata(), so must judge this stream
            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(
                        inst_state->resampler_accel_stream);
            } else {
                // if stream has been released, event must be set to NULL to break the while()
                PICKUP_DETECT_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_gyro_stream) {
        gyro_event = inst_state->resampler_gyro_stream->api->peek_input(inst_state->resampler_gyro_stream);

        while (NULL != gyro_event) {
            pickup_detect_decode_and_checkdata(gyro_event, GYRO_TYPE);

            if (NULL != inst_state->resampler_gyro_stream) {
                gyro_event = inst_state->resampler_gyro_stream->api->get_next_input(
                        inst_state->resampler_gyro_stream);
            } else {
                PICKUP_DETECT_LOG_0("gyro stream has been released.");
                gyro_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_gravity_stream) {
        gravity_event = inst_state->resampler_gravity_stream->api->peek_input(
                inst_state->resampler_gravity_stream);

        while (NULL != gravity_event) {
            pickup_detect_decode_and_checkdata(gravity_event, GRAVITY_TYPE);

            if (NULL != inst_state->resampler_gravity_stream) {
                gravity_event = inst_state->resampler_gravity_stream->api->get_next_input(
                        inst_state->resampler_gravity_stream);
            } else {
                PICKUP_DETECT_LOG_0("gravity stream has been released.");
                gravity_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_prox_stream) {
        prox_event = inst_state->resampler_prox_stream->api->peek_input(inst_state->resampler_prox_stream);

        while (NULL != prox_event) {
            pickup_detect_decode_and_checkdata(prox_event, PROX_TYPE);

            if (NULL != inst_state->resampler_prox_stream) {
                prox_event = inst_state->resampler_prox_stream->api->get_next_input(
                        inst_state->resampler_prox_stream);
            } else {
                PICKUP_DETECT_LOG_0("prox stream has been released.");
                prox_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc sns_pickup_detect_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_pickup_detect_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_pickup_detect_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_pickup_detect_inst_init,
    .deinit = &sns_pickup_detect_inst_deinit,
    .set_client_config = &sns_pickup_detect_inst_set_client_config,
    .notify_event = &sns_pickup_detect_inst_notify_event
};

struct pickup_detect_sensor_operation pickup_detect_ops = {
    .sensor_change_state = pickup_detect_sensor_enable,
    .report = pickup_detect_report,
    .get_delta_time_ms = pickup_detect_get_delta_time_ms,
    .need_check_prox = pickup_detect_need_check_prox,
};

