/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: sns_xxx.c
**
** Description:
**      Definitions for  detect algorithem .
**
** Version: 1.0
** Date created: 2020/05/09,11:50
**
** --------------------------- Revision History: ------------------------------------
* <version>        <date>        <author>               <desc>
**************************************************************************************/

#include "sns_oplus_pedometer_sensor_instance.h"
#include "sns_pedometer.pb.h"

#define OPLUS_PEDOMETER_DETECT_BATCH_PERIOD 1000000  // 1s
#define OPLUS_PEDOMETER_ACC_BATCH_PERIOD 500000  // 500ms

sns_sensor_instance *oplus_pedometer_mTask;

static void oplus_pedometer_sensor_enable(int sensor_type, bool on)
{
    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)oplus_pedometer_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    float sample_rate = inst_state->state->sampleRate;

    OPLUS_PEDOMETER_DETECT_LOG_3("sensor enable operate, sensor_type = %d, on = %d, sample_rate = %d", sensor_type, on, inst_state->state->sampleRate);

    sns_service_manager *manager = oplus_pedometer_mTask->cb->get_service_manager(oplus_pedometer_mTask);
    sns_stream_service *stream_service = (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;
    sns_std_sensor_config gravity_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = OPLUS_PEDOMETER_DETECT_BATCH_PERIOD;

    if (on) {
        switch (sensor_type) {
            case ACC_TYPE:
                sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid), &inst_state->accel_suid, sizeof(inst_state->accel_suid));

                sns_std_request acc_std_req = sns_std_request_init_default;
                acc_std_req.has_batching = true;
                acc_std_req.batching.batch_period = OPLUS_PEDOMETER_ACC_BATCH_PERIOD;

                resampler_config.resampled_rate = 20;
                resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
                resampler_config.filter = false;

                if (inst_state->resampler_accel_stream == NULL) {
                    OPLUS_PEDOMETER_DETECT_LOG_0("create resampler acc stream.");

                    // create connection with resampler sensor
                    stream_service->api->create_sensor_instance_stream(stream_service, oplus_pedometer_mTask,
                                            inst_state->resampler_suid, &inst_state->resampler_accel_stream);
                }

                OPLUS_PEDOMETER_DETECT_LOG_1("resampler acc config, sample_rate = %d.", (int)resampler_config.resampled_rate);

                if (NULL != inst_state->resampler_accel_stream) {
                    encoded_len = pb_encode_request(buffer, sizeof(buffer), &resampler_config, sns_resampler_config_fields, &acc_std_req);
                    if (0 < encoded_len) {
                        sns_request request = (sns_request){
                            .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                            .request_len = encoded_len, .request = buffer };

                        rc = inst_state->resampler_accel_stream->api->send_request(inst_state->resampler_accel_stream, &request);
                        OPLUS_PEDOMETER_DETECT_LOG_2("Processed accel config request: sample_rate %f, result %u", sample_rate, rc);
                    }
                } else {
                    OPLUS_PEDOMETER_DETECT_LOG_0("Error in creating accel_stream OR encoding failed");
                }
                break;
            case GRAVITY_TYPE:
                gravity_config.sample_rate = sample_rate;

                if (inst_state->resampler_gravity_stream == NULL) {
                    OPLUS_PEDOMETER_DETECT_LOG_0("create linacc gravity stream.");

                    // create connection with resampler sensor
                    stream_service->api->create_sensor_instance_stream(stream_service, oplus_pedometer_mTask,
                                            inst_state->gravity_suid, &inst_state->resampler_gravity_stream);
                }

                if (NULL != inst_state->resampler_gravity_stream) {
                    encoded_len = pb_encode_request(buffer, sizeof(buffer), &gravity_config, sns_std_sensor_config_fields, &std_req);
                    if (0 < encoded_len) {
                        sns_request request = (sns_request){
                            .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                            .request_len = encoded_len, .request = buffer };

                        rc = inst_state->resampler_gravity_stream->api->send_request(inst_state->resampler_gravity_stream, &request);
                        OPLUS_PEDOMETER_DETECT_LOG_2("Processed gravity config request: sample_rate %f, result %u", sample_rate, rc);
                    }
                } else {
                    OPLUS_PEDOMETER_DETECT_LOG_0("Error in creating gravity_stream OR encoding failed");
                }
                break;
            case LINACC_TYPE:
                OPLUS_PEDOMETER_DETECT_LOG_0("Has been enable in GRAVITY_TYPE");
                break;
        }
    } else {
        switch (sensor_type) {
            case ACC_TYPE:
                sns_sensor_util_remove_sensor_instance_stream(oplus_pedometer_mTask, &inst_state->resampler_accel_stream);
                break;
            case GRAVITY_TYPE:
                sns_sensor_util_remove_sensor_instance_stream(oplus_pedometer_mTask, &inst_state->resampler_gravity_stream);
                break;
            case LINACC_TYPE:
                break;
        }
    }
}

static void oplus_pedometer_report(uint64_t oplus_pedometer_timestamp, uint32_t step_count_total)
{
    //float oplus_pedometer_data[3];
    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)oplus_pedometer_mTask->state->state;
/*
    oplus_pedometer_data[0] = 0.0f;
    oplus_pedometer_data[1] = (float)step_count_total;
    oplus_pedometer_data[2] = 0.0f;

    rc = pb_send_sensor_stream_event(oplus_pedometer_mTask,
                            NULL,
                            sns_get_system_time(),
                            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                            oplus_pedometer_data,
                            3,
                            inst_state->config.encoded_data_event_len);
    if(SNS_RC_SUCCESS != rc)
    {
        OPLUS_PEDOMETER_DETECT_LOG_0("oplus_pedometer report failed, Error in sending event");
    }
    else
    {
        OPLUS_PEDOMETER_DETECT_LOG_2("report oplus_pedometer data step_count = %d, oplus_pedometer_timestamp = %u",
            step_count_total, (uint32_t)oplus_pedometer_timestamp);
    }
    sns_step_event oplus_pedometer_event = sns_step_event_init_default;

    pb_buffer_arg buff_arg_step_count = {
        .buf     = step_count_total,
        .buf_len = sizeof(step_count_total)
    };
    oplus_pedometer_event.step_count.funcs.encode = &pb_encode_float_arr_cb;
    oplus_pedometer_event.step_count.arg          = &buff_arg_step_count;
*/
    sns_step_event oplus_pedometer_event = sns_step_event_init_default;
    oplus_pedometer_event.step_count = step_count_total;
    pb_send_event(oplus_pedometer_mTask,
                  sns_step_event_fields,
                  &oplus_pedometer_event,
                  sns_get_system_time(),
                  SNS_PEDOMETER_MSGID_SNS_STEP_EVENT,
                  &inst_state->self_suid);
    OPLUS_PEDOMETER_DETECT_LOG_2("report oplus_pedometer data step_count = %d, oplus_pedometer_timestamp = %u",
        step_count_total, (uint32_t)oplus_pedometer_timestamp);
}

static uint64_t oplus_pedometer_get_delta_time_ms(uint64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void oplus_pedometer_decode_and_checkdata(sns_sensor_event *event_in, MOTION_SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    pedometer_detect_sensor_data input;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
        //MOTION_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in, event_in->message_id, event_in->timestamp);

            pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

            sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

            decoded_event.data = (pb_callback_t){.funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg};

            pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

            if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
                OPLUS_PEDOMETER_DETECT_LOG_0("Error in decoding event");
            } else {
                //OPLUS_PEDOMETER_DETECT_LOG_4("data[%d %d %d]", type_in, (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
            }

        input.data_x = data[0];
        input.data_y = data[1];
        input.data_z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        if (type_in == GRAVITY_TYPE) {
            input.data_x = data[3];
            input.data_y = data[4];
            input.data_z = data[5];
            input.timestamp = event_in->timestamp;
            input.type = LINACC_TYPE;

            //oplus_pedometer_detect_process_gravity_linear_data(data, event_in->timestamp);
        }

        oplus_pedometer_detect_process_gravity_linear_data(data, event_in->timestamp, type_in);
    }
}

static sns_rc sns_oplus_pedometer_process_event(sns_sensor_instance *const this)
{
    sns_oplus_pedometer_inst_state *inst_state = (sns_oplus_pedometer_inst_state*)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *gravity_event = NULL;
    //sns_sensor_event *game_rv_event = NULL;

    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            oplus_pedometer_decode_and_checkdata(acc_event, ACC_TYPE);

            // stream maybe released in oplus_pedometer_decode_and_checkdata(), so must judge this stream
            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(inst_state->resampler_accel_stream);
            } else {
                // if stream has been released, event must be set to NULL to break the while()
                OPLUS_PEDOMETER_DETECT_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }

    if (NULL != inst_state->resampler_gravity_stream) {
        gravity_event = inst_state->resampler_gravity_stream->api->peek_input(inst_state->resampler_gravity_stream);

        while (NULL != gravity_event) {
            oplus_pedometer_decode_and_checkdata(gravity_event, GRAVITY_TYPE);

            if (NULL != inst_state->resampler_gravity_stream) {
                gravity_event = inst_state->resampler_gravity_stream->api->get_next_input(inst_state->resampler_gravity_stream);
            } else {
                OPLUS_PEDOMETER_DETECT_LOG_0("gravity stream has been released.");
                gravity_event = NULL;
            }
        }
    }
    /*
    if (NULL != inst_state->resampler_game_rv_stream)
    {
        game_rv_event = inst_state->resampler_game_rv_stream->api->peek_input(inst_state->resampler_game_rv_stream);

        while (NULL != game_rv_event)
        {
            oplus_pedometer_decode_and_checkdata(game_rv_event, GAMEROTATIONVECTOR_TYPE);

            if (NULL != inst_state->resampler_game_rv_stream)
            {
                game_rv_event = inst_state->resampler_game_rv_stream->api->get_next_input(inst_state->resampler_game_rv_stream);
            }
            else
            {
                OPLUS_PEDOMETER_DETECT_LOG_0("game rv stream has been released.");
                game_rv_event = NULL;
            }
        }
    }
    */
    return SNS_RC_SUCCESS;
}

static sns_rc sns_oplus_pedometer_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_oplus_pedometer_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_oplus_pedometer_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_oplus_pedometer_inst_init,
    .deinit = &sns_oplus_pedometer_inst_deinit,
    .set_client_config = &sns_oplus_pedometer_inst_set_client_config,
    .notify_event = &sns_oplus_pedometer_inst_notify_event
};

struct pedometer_detect_sensor_operation oplus_pedometer_ops =
{
    .sensor_change_state = oplus_pedometer_sensor_enable,
    .report = oplus_pedometer_report,
    .get_delta_time_ms = oplus_pedometer_get_delta_time_ms,
};

