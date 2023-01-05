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

#include "sns_pedometer_minute_sensor_instance.h"
#include "sns_pedometer.pb.h"

sns_sensor_instance *pedometer_minute_mTask;

static void pedometer_minute_sensor_enable(int sensor_type, bool on)
{
    sns_pedometer_minute_inst_state *inst_state = (sns_pedometer_minute_inst_state*)pedometer_minute_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    PEDEMETER_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = pedometer_minute_mTask->cb->get_service_manager(pedometer_minute_mTask);
    sns_stream_service *stream_service = (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_std_sensor_config step_counter_config = sns_std_sensor_config_init_default;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;


    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    if (on) {
        switch (sensor_type) {
            case STEP_COUNTER_TYPE:
                step_counter_config.sample_rate = 5;

                if (NULL == inst_state->resampler_step_counter_stream) {
                    PEDEMETER_LOG_0("create resampler step_counter stream.");

                    // create connection with resampler sensor
                    stream_service->api->create_sensor_instance_stream(stream_service, pedometer_minute_mTask,
                                            inst_state->step_counter_suid, &inst_state->resampler_step_counter_stream);
                }

                if (NULL != inst_state->resampler_step_counter_stream) {
                    encoded_len = pb_encode_request(buffer, sizeof(buffer), &step_counter_config, sns_std_sensor_config_fields, NULL);
                    if (0 < encoded_len) {
                        sns_request request = (sns_request){
                            .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                            .request_len = encoded_len, .request = buffer };

                        rc = inst_state->resampler_step_counter_stream->api->send_request(inst_state->resampler_step_counter_stream, &request);
                        PEDEMETER_LOG_1("Processed step_counter config request: sample_rate = 5, result %u", rc);
                    }
                }
                else {
                    PEDEMETER_LOG_0("Error in creating accel_stream OR encoding failed");
                }
                break;

                    case TIMER_TYPE:
                    if (NULL == inst_state->timer_data_stream) {
                        PEDEMETER_LOG_0("create resampler timer stream.");
                        // create connection with resampler sensor
                        stream_service->api->create_sensor_instance_stream(stream_service, pedometer_minute_mTask,
                                                inst_state->timer_suid, &inst_state->timer_data_stream);
                    }
                float timeout = 60000;
                    if (NULL != inst_state->timer_data_stream) {
                        sns_request timer_req =
                        {
                            .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
                            .request    = buffer
                        };

                        req_payload.is_periodic    = true;
                        req_payload.start_time     = sns_get_system_time();

                        req_payload.timeout_period =  sns_convert_ns_to_ticks((sns_time)((sns_time)timeout* 1000 * 1000));
                        timer_req.request_len      = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);

                        if (timer_req.request_len > 0) {
                            inst_state->timer_data_stream->api->send_request(inst_state->timer_data_stream, &timer_req);
                        }

                        PEDEMETER_LOG_1("Processed timer config request: result %u", rc);
                    } else {
                        PEDEMETER_LOG_0("Error in creating timer stream OR encoding failed");
                    }
                    break;

#ifdef REGISTRY_CMC
                        case CMC_TYPE:
                if (NULL == inst_state->cmc_sensor_stream) {
                    PEDEMETER_LOG_0("create resampler step_counter stream.");

                    // create connection with resampler sensor
                    stream_service->api->create_sensor_instance_stream(stream_service, pedometer_minute_mTask,
                                            inst_state->cmc_sensor_uid, &inst_state->cmc_sensor_stream);
                }

                if (NULL != inst_state->cmc_sensor_stream) {
                    encoded_len = pb_encode_request(buffer, sizeof(buffer), &step_counter_config, sns_std_sensor_config_fields, NULL);
                    if (0 < encoded_len) {
                        sns_request request = (sns_request) {
                            .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                            .request_len = encoded_len, .request = buffer };

                        rc = inst_state->cmc_sensor_stream->api->send_request(inst_state->cmc_sensor_stream, &request);
                        PEDEMETER_LOG_1("Processed cmc config request: sample_rate = 5, result %u", rc);
                    }
                }
                else {
                    PEDEMETER_LOG_0("Error in creating cmc OR encoding failed");
                }
                break;
#endif

                default:
                    break;
        }
    }
    else
    {
        switch (sensor_type) {
            case STEP_COUNTER_TYPE:
                sns_sensor_util_remove_sensor_instance_stream(pedometer_minute_mTask, &inst_state->resampler_step_counter_stream);
                break;
                        case TIMER_TYPE:
                sns_sensor_util_remove_sensor_instance_stream(pedometer_minute_mTask, &inst_state->timer_data_stream);
                        break;
#ifdef REGISTRY_CMC
                        case CMC_TYPE:
                sns_sensor_util_remove_sensor_instance_stream(pedometer_minute_mTask, &inst_state->cmc_sensor_stream);
                        break;
#endif

            default:
                break;
        }
    }
}

static void pedometer_minute_report(int step_count, int step_run_count, int step_walk_count,
        uint16_t report_count, uint8_t move_status, int time_gap, uint64_t timestamp)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float pedometer_minute_data[6] = {0};
    sns_pedometer_minute_inst_state *inst_state = (sns_pedometer_minute_inst_state*)pedometer_minute_mTask->state->state;

    pedometer_minute_data[0] = step_count;
    pedometer_minute_data[1] = (float)report_count;
    pedometer_minute_data[2] = (float)move_status;
    pedometer_minute_data[3] = (float)time_gap;
    pedometer_minute_data[4] = step_run_count;
    pedometer_minute_data[5] = step_walk_count;

    rc = pb_send_sensor_stream_event(pedometer_minute_mTask,
                            NULL,
                            timestamp,
                            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                            pedometer_minute_data,
                            6,
                            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        PEDEMETER_LOG_0("free fall report failed, Error in sending event");
    }
    else
    {
        PEDEMETER_LOG_0("report pedometer_minute data.");
    }
}

static uint64_t pedometer_minute_get_delta_time_ms(int64_t timestamp)
{
    if (timestamp < 0) {
        return 0;
    }
    else
        return sns_get_ms_time_from_tick(timestamp);
}


static void pedometer_minute_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    pedometer_minute_sensor_data input;

#ifdef SW_STEPCNT
        if (SNS_PEDOMETER_MSGID_SNS_STEP_EVENT == event_in->message_id && (STEP_COUNTER_TYPE == type_in)) {
                pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

                sns_step_event pedometer_data = sns_step_event_init_default;

                if (!pb_decode(&stream, sns_step_event_fields, &pedometer_data)) {
                        PEDEMETER_LOG_0("Error decoding pedometer sensor event");
                } else {
                        input.x = pedometer_data.step_count;
                        input.timestamp = event_in->timestamp;
                        input.type = type_in;

                        pedometer_minute_check_sensor_data(&input);
                }
        }
#else
    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id && (STEP_COUNTER_TYPE == type_in)) {
        //TP_GESTURE_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in, event_in->message_id, event_in->timestamp);

        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t){.funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg};

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            PEDEMETER_LOG_0("Error in decoding event");
        }
        else
        {
            //TP_GESTURE_LOG_3("data[%d %d %d]", (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
        }

        input.x = data[0];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        pedometer_minute_check_sensor_data(&input);
    }
#endif
    else if ((SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event_in->message_id) && (TIMER_TYPE == type_in)) {
        sns_timer_sensor_event timer_event;

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_timer_sensor_event_fields, &timer_event)) {
            PEDEMETER_LOG_0("Error in decoding event");
        } else {
            input.timestamp = event_in->timestamp;
            input.type = type_in;
            pedometer_minute_check_sensor_data(&input);
        }
    }
#ifdef REGISTRY_CMC
    else if ((SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) && (CMC_TYPE == type_in)) {
        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t){.funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg};

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            PEDEMETER_LOG_0("Error in decoding event");
        }
        else
        {
            PEDEMETER_LOG_3("data[%d %d %d]", (int)(data[0]), (int)(data[1]), (int)(data[2]));
            input.x = data[2];
            input.timestamp = event_in->timestamp;
            input.type = type_in;
            pedometer_minute_check_sensor_data(&input);
        }
    }
#endif
}

static sns_rc sns_pedometer_minute_process_event(sns_sensor_instance *const this)
{
    sns_pedometer_minute_inst_state *inst_state = (sns_pedometer_minute_inst_state*)this->state->state;

    sns_sensor_event *step_counter_event = NULL;
        sns_sensor_event *timer_event = NULL;
#ifdef REGISTRY_CMC
    sns_sensor_event *cmc_event = NULL;
#endif

    if (NULL != inst_state->resampler_step_counter_stream) {
        step_counter_event = inst_state->resampler_step_counter_stream->api->peek_input(inst_state->resampler_step_counter_stream);

        while (NULL != step_counter_event) {
            pedometer_minute_decode_and_checkdata(step_counter_event, STEP_COUNTER_TYPE);

            if (NULL != inst_state->resampler_step_counter_stream) {
                step_counter_event = inst_state->resampler_step_counter_stream->api->get_next_input(inst_state->resampler_step_counter_stream);
            }
            else
            {
                PEDEMETER_LOG_0("accel stream has been released.");
                step_counter_event = NULL;
            }
        }
    }

    if (NULL != inst_state->timer_data_stream) {
        timer_event = inst_state->timer_data_stream->api->peek_input(inst_state->timer_data_stream);

        while (NULL != timer_event) {
            pedometer_minute_decode_and_checkdata(timer_event, TIMER_TYPE);

            if (NULL != inst_state->timer_data_stream) {
                timer_event = inst_state->timer_data_stream->api->get_next_input(inst_state->timer_data_stream);
            } else {
                PEDEMETER_LOG_0("timer stream has been released.");
                timer_event = NULL;
            }
        }
    }
#ifdef REGISTRY_CMC
    if (NULL != inst_state->cmc_sensor_stream) {
        cmc_event = inst_state->cmc_sensor_stream->api->peek_input(inst_state->cmc_sensor_stream);

        while (NULL != cmc_event) {
            pedometer_minute_decode_and_checkdata(cmc_event, CMC_TYPE);

            if (NULL != inst_state->timer_data_stream) {
                cmc_event = inst_state->timer_data_stream->api->get_next_input(inst_state->cmc_sensor_stream);
            } else {
                PEDEMETER_LOG_0("timer stream has been released.");
                cmc_event = NULL;
            }
        }
    }
#endif

    return SNS_RC_SUCCESS;
}

static sns_rc sns_pedometer_minute_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_pedometer_minute_process_event(this);

    return rc;
}

sns_rc sns_pedometer_minute_inst_set_client_config(sns_sensor_instance *const this, sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;

    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == client_request->message_id ||
        SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
                sns_pedometer_minute_req* req = (sns_pedometer_minute_req*)client_request->request;
        PEDEMETER_LOG_1("Sensor config samplete = %d", (int)req->desired_sample_rate);

        pedometer_minute_Reset(req->desired_sample_rate);
    }
    else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
        PEDEMETER_LOG_0("Flush req");
        sns_sensor_util_send_flush_event(NULL, this);
    }
    else
    {
        PEDEMETER_LOG_1("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

sns_sensor_instance_api sns_pedometer_minute_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_pedometer_minute_inst_init,
    .deinit = &sns_pedometer_minute_inst_deinit,
    .set_client_config = &sns_pedometer_minute_inst_set_client_config,
    .notify_event = &sns_pedometer_minute_inst_notify_event
};

struct pedometer_minute_sensor_operation pedometer_minute_ops =
{
    .sensor_change_state = pedometer_minute_sensor_enable,
    .report = pedometer_minute_report,
    .get_delta_time_ms = pedometer_minute_get_delta_time_ms,
};

