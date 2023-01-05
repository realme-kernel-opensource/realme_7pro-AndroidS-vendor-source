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

#include "sns_double_tap_sensor_instance.h"

sns_sensor_instance *double_tap_mTask;

static void double_tap_sensor_enable(int sensor_type, bool on)
{
    sns_double_tap_inst_state *inst_state = (sns_double_tap_inst_state *)double_tap_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    DOUBLE_TAP_LOGI("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = double_tap_mTask->cb->get_service_manager(double_tap_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;
    sns_std_sensor_config ic_double_tap_config = sns_std_sensor_config_init_default;
    sns_std_sensor_config prox_config = sns_std_sensor_config_init_default;
    sns_std_sensor_config gravity_config  = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = false;

    if (on) {
        switch (sensor_type) {
        case ACCEL_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = inst_state->state->sampleRate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_accel_stream) {
                DOUBLE_TAP_LOGI("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, double_tap_mTask,
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
                    DOUBLE_TAP_LOGI("Processed accel config request: sample_rate %f, result %u",
                        resampler_config.resampled_rate, rc);
                }
            } else {
                DOUBLE_TAP_LOGI("Error in creating accel_stream OR encoding failed");
            }
            break;

        case PROX_TYPE:
            prox_config.sample_rate = inst_state->state->sampleRate;

            if (NULL == inst_state->prox_stream) {
                DOUBLE_TAP_LOGI("create prox stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, double_tap_mTask,
                    inst_state->prox_suid, &inst_state->prox_stream);
            }

            if (NULL != inst_state->prox_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &prox_config, sns_std_sensor_config_fields,
                        NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id =
                            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,   // the same as prox_xxx_set_client_config, enable prox message id
                            .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->prox_stream->api->send_request(inst_state->prox_stream,
                            &request);
                    DOUBLE_TAP_LOGI("Processed prox config request: sample_rate %f, result %u",
                        prox_config.sample_rate, rc);
                }
            } else {
                DOUBLE_TAP_LOGI("Error in creating prox_stream OR encoding failed");
            }

            break;

        case IC_DOUBLE_TAP_TYPE:
            ic_double_tap_config.sample_rate = inst_state->state->sampleRate;
	    if (NULL == inst_state->ic_double_tap_stream) {
                DOUBLE_TAP_LOGI("create double_tap stream.");
                stream_service->api->create_sensor_instance_stream(stream_service, double_tap_mTask,
                    inst_state->ic_double_tap_suid, &inst_state->ic_double_tap_stream);
            }
  	    if (NULL != inst_state->ic_double_tap_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &ic_double_tap_config, sns_std_sensor_config_fields,
                        NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id =
                            SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,   // the same as prox_xxx_set_client_config, enable prox message id
                            .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->ic_double_tap_stream->api->send_request(inst_state->ic_double_tap_stream,
                            &request);
                    DOUBLE_TAP_LOGI("Processed double_tap config request");
                }
            } else {
                DOUBLE_TAP_LOGI("Error in creating double_tap stream OR encoding failed");
            }


            break;
        case GRAVITY_TYPE:
            gravity_config.sample_rate = inst_state->state->sampleRate;
	    if (NULL == inst_state->resampler_gravity_stream) {
                DOUBLE_TAP_LOGI("create gravity stream.");
                stream_service->api->create_sensor_instance_stream(stream_service, double_tap_mTask,
                    inst_state->gravity_suid, &inst_state->resampler_gravity_stream);
            }
  	    if (NULL != inst_state->resampler_gravity_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &gravity_config, sns_std_sensor_config_fields,
                        &std_req);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id =
                            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,   // the same as prox_xxx_set_client_config, enable prox message id
                            .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_gravity_stream->api->send_request(inst_state->resampler_gravity_stream,
                            &request);
                    DOUBLE_TAP_LOGI("Processed gravity config request");
                }
            } else {
                DOUBLE_TAP_LOGI("Error in creating gravity stream OR encoding failed");
            }


            break;

        }
    } else {
        switch (sensor_type) {
        case ACCEL_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(double_tap_mTask,
                &inst_state->resampler_accel_stream);
            break;

        case PROX_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(double_tap_mTask, &inst_state->prox_stream);
            break;

        case IC_DOUBLE_TAP_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(double_tap_mTask, &inst_state->ic_double_tap_stream);
            break;
        case GRAVITY_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(double_tap_mTask, &inst_state->resampler_gravity_stream);
            break;

        default:
            break;
        }
    }
}


static void double_tap_report(uint16_t report_count)
{
    //sns_rc rc = SNS_RC_SUCCESS;
    //float double_tap_data[2] = {0};
    //sns_double_tap_inst_state *inst_state = (sns_double_tap_inst_state *)double_tap_mTask->state->state;

    //double_tap_data[0] = 2;
    //double_tap_data[1] = (float)report_count;
    UNUSED_VAR(report_count);
    sns_double_tap_event dbtap_state;
    dbtap_state.double_tap_event_type = SNS_DOUBLE_TAP_EVENT_TYPE_FIRED;
/*
    rc = pb_send_sensor_stream_event(double_tap_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            double_tap_data,
            2,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        DOUBLE_TAP_LOGI("double_tap report failed, Error in sending event");
    } else {
        DOUBLE_TAP_LOGI("report double_tap data.");
    }
*/
    pb_send_event(double_tap_mTask,
                  sns_double_tap_event_fields,
                  &dbtap_state,
                  sns_get_system_time(),
                  SNS_DOUBLE_TAP_MSGID_SNS_DOUBLE_TAP_EVENT,
                  NULL);
}

static uint64_t double_tap_get_delta_time_ms(int64_t timestamp)
{
    if (timestamp < 0) {
        return 0;
    } else {
        return sns_get_ms_time_from_tick(timestamp);
    }
}

static void double_tap_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    double_tap_sensor_data input;
    bool need_handle = false;

    switch (type_in) {
    case ACCEL_TYPE:
    case GRAVITY_TYPE:
        if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
            pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

            sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

            decoded_event.data = (pb_callback_t) {
                .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
            };

            pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

            if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
                DOUBLE_TAP_LOGI("Error in decoding event");
            } else {
                need_handle = true;
                //FP_DISPLAY_LOG_3("data[%d %d %d]", (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
            }
        }

        break;

    case PROX_TYPE:
        if (event_in->message_id == SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT) {
            sns_proximity_event decoded_event = sns_proximity_event_init_default;

            pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

            if (!pb_decode(&istream, sns_proximity_event_fields, &decoded_event)) {
                DOUBLE_TAP_LOGI("Error in decoding prox event");
            } else {
                need_handle = true;

                data[0] = decoded_event.proximity_event_type;
                data[1] = decoded_event.raw_adc;
            }
        }

        break;

    case IC_DOUBLE_TAP_TYPE:
	if (event_in->message_id == SNS_DOUBLE_TAP_MSGID_SNS_DOUBLE_TAP_EVENT) {
            sns_double_tap_event decoded_event = sns_double_tap_event_init_default;

            pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

            if (!pb_decode(&istream, sns_double_tap_event_fields, &decoded_event)) {
                DOUBLE_TAP_LOGI("Error in decoding double_tap event");
            } else {
                need_handle = true;

                data[0] = decoded_event.double_tap_event_type;
            }
        }


        break;

    default:

        break;
    }

    if (true == need_handle) {
        input.x = data[0];
        input.y = data[1];
        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        double_tap_process_sensor_data(&input);
    }
}


static sns_rc sns_double_tap_process_event(sns_sensor_instance *const this)
{
    sns_double_tap_inst_state *inst_state = (sns_double_tap_inst_state *)this->state->state;
    
    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *prox_event = NULL;
    sns_sensor_event *ic_double_tap_event = NULL;
    sns_sensor_event *gravity_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            double_tap_decode_and_checkdata(acc_event, ACCEL_TYPE);

            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(
                        inst_state->resampler_accel_stream);
            } else {
                DOUBLE_TAP_LOGI("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->prox_stream) {
        prox_event = inst_state->prox_stream->api->peek_input(inst_state->prox_stream);

        while (NULL != prox_event) {
            double_tap_decode_and_checkdata(prox_event, PROX_TYPE);

            if (NULL != inst_state->prox_stream) {
                prox_event = inst_state->prox_stream->api->get_next_input(
                        inst_state->prox_stream);
            } else {
                DOUBLE_TAP_LOGI("prox stream has been released.");
                prox_event = NULL;
            }
        }
    }

    if (NULL != inst_state->ic_double_tap_stream) {
        ic_double_tap_event = inst_state->ic_double_tap_stream->api->peek_input(inst_state->ic_double_tap_stream);

        while (NULL != ic_double_tap_event) {
            double_tap_decode_and_checkdata(ic_double_tap_event, IC_DOUBLE_TAP_TYPE);

            if (NULL != inst_state->ic_double_tap_stream) {
                ic_double_tap_event = inst_state->ic_double_tap_stream->api->get_next_input(
                        inst_state->ic_double_tap_stream);
            } else {
                DOUBLE_TAP_LOGI("gyro stream has been released.");
                ic_double_tap_event = NULL;
            }
        }
    }
    if (NULL != inst_state->resampler_gravity_stream) {
        gravity_event = inst_state->resampler_gravity_stream->api->peek_input(inst_state->resampler_gravity_stream);

        while (NULL != gravity_event) {
            double_tap_decode_and_checkdata(gravity_event, GRAVITY_TYPE);

            if (NULL != inst_state->resampler_gravity_stream) {
                gravity_event = inst_state->resampler_gravity_stream->api->get_next_input(
                        inst_state->resampler_gravity_stream);
            } else {
                DOUBLE_TAP_LOGI("gravity stream has been released.");
                gravity_event = NULL;
            }
        }
    }


    return SNS_RC_SUCCESS;
}

static sns_rc sns_double_tap_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_double_tap_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_double_tap_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_double_tap_inst_init,
    .deinit = &sns_double_tap_inst_deinit,
    .set_client_config = &sns_double_tap_inst_set_client_config,
    .notify_event = &sns_double_tap_inst_notify_event
};

struct double_tap_sensor_operation double_tap_ops = {
    .sensor_change_state = double_tap_sensor_enable,
    .report = double_tap_report,
    .get_delta_time_ms = double_tap_get_delta_time_ms
};

