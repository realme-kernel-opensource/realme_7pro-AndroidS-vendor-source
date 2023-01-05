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

#include "sns_tp_gesture_sensor_instance.h"
#define TP_GESTURE_BATCH_PERIOD 100000  // 100ms

sns_sensor_instance *tp_gesture_mTask;

static void tp_gesture_sensor_enable(int sensor_type, bool on)
{
    sns_tp_gesture_inst_state *inst_state = (sns_tp_gesture_inst_state *)tp_gesture_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    TP_GESTURE_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = tp_gesture_mTask->cb->get_service_manager(tp_gesture_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};


    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = inst_state->state->acc_sampleRate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_accel_stream) {
                TP_GESTURE_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, tp_gesture_mTask,
                    inst_state->resampler_suid, &inst_state->resampler_accel_stream);
            }

            if (NULL != inst_state->resampler_accel_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config,
                        sns_resampler_config_fields, NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_accel_stream->api->send_request(inst_state->resampler_accel_stream,
                            &request);
                    TP_GESTURE_LOG_2("Processed accel config request: sample_rate %f, result %u",
                        inst_state->state->acc_sampleRate, rc);
                }
            } else {
                TP_GESTURE_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;

        case GYRO_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->gyro_suid, sizeof(inst_state->gyro_suid));

            resampler_config.resampled_rate =  inst_state->state->gyro_sampleRate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_gyro_stream) {
                TP_GESTURE_LOG_0("create resampler gyro stream.");
                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, tp_gesture_mTask,
                    inst_state->resampler_suid, &inst_state->resampler_gyro_stream);
            }

            if (NULL != inst_state->resampler_gyro_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config,
                        sns_resampler_config_fields, NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_gyro_stream->api->send_request(inst_state->resampler_gyro_stream,
                            &request);
                    TP_GESTURE_LOG_2("Processed gyro config request: sample_rate %f, result %u",
                        inst_state->state->gyro_sampleRate, rc);
                }
            } else {
                TP_GESTURE_LOG_0("Error in creating gyro_stream OR encoding failed");
            }

            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(tp_gesture_mTask,
                &inst_state->resampler_accel_stream);
            break;
        case GYRO_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(tp_gesture_mTask, &inst_state->resampler_gyro_stream);
            break;
        }
    }
}

static void tp_gesture_report(int move, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float tp_gesture_data[2] = {0};
    sns_tp_gesture_inst_state *inst_state = (sns_tp_gesture_inst_state *)tp_gesture_mTask->state->state;

    tp_gesture_data[0] = move;
    tp_gesture_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(tp_gesture_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            tp_gesture_data,
            2,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        TP_GESTURE_LOG_0("free fall report failed, Error in sending event");
    } else {
        TP_GESTURE_LOG_0("report tp_gesture data.");
    }
}

static uint64_t tp_gesture_get_delta_time_ms(int64_t timestamp)
{
    if (timestamp < 0) {
        return 0;
    } else {
        return sns_get_ms_time_from_tick(timestamp);
    }
}

static void tp_gesture_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    tp_gesture_sensor_data input;

    if (
        (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id)
    ) {
        //TP_GESTURE_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in, event_in->message_id, event_in->timestamp);

        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {
            .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
        };

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            TP_GESTURE_LOG_0("Error in decoding event");
        } else {
            //TP_GESTURE_LOG_3("data[%d %d %d]", (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
        }

        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        tp_gesture_process_sensor_data(&input);
    }
}

static sns_rc sns_tp_gesture_process_event(sns_sensor_instance *const this)
{
    sns_tp_gesture_inst_state *inst_state = (sns_tp_gesture_inst_state *)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *gyro_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            tp_gesture_decode_and_checkdata(acc_event, ACC_TYPE);

            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(
                        inst_state->resampler_accel_stream);
            } else {
                TP_GESTURE_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_gyro_stream) {
        gyro_event = inst_state->resampler_gyro_stream->api->peek_input(inst_state->resampler_gyro_stream);

        while (NULL != gyro_event) {
            tp_gesture_decode_and_checkdata(gyro_event, GYRO_TYPE);

            if (NULL != inst_state->resampler_gyro_stream) {
                gyro_event = inst_state->resampler_gyro_stream->api->get_next_input(
                        inst_state->resampler_gyro_stream);
            } else {
                TP_GESTURE_LOG_0("gyro stream has been released.");
                gyro_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc sns_tp_gesture_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_tp_gesture_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_tp_gesture_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_tp_gesture_inst_init,
    .deinit = &sns_tp_gesture_inst_deinit,
    .set_client_config = &sns_tp_gesture_inst_set_client_config,
    .notify_event = &sns_tp_gesture_inst_notify_event
};

struct tp_gesture_sensor_operation tp_gesture_ops = {
    .sensor_change_state = tp_gesture_sensor_enable,
    .report = tp_gesture_report,
    .get_delta_time_ms = tp_gesture_get_delta_time_ms,
};

