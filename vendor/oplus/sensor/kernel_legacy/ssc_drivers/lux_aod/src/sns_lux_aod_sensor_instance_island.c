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

#include "sns_lux_aod_sensor_instance.h"

sns_sensor_instance *lux_aod_mTask;

static bool lux_aod_need_overload_thrd(void)
{
    return true;
}

static void lux_aod_get_new_thrd(int *thrd_low, int *thrd_high)
{
    sns_lux_aod_inst_state *inst_state = (sns_lux_aod_inst_state *)lux_aod_mTask->state->state;
    *thrd_low = inst_state->config.smem->parameter[0];
    *thrd_high = inst_state->config.smem->parameter[1];
    LUX_AOD_LOG_2("thrd_low: %d   thrd_high: %d", inst_state->config.smem->parameter[0],
        inst_state->config.smem->parameter[1]);
}

static void lux_aod_sensor_enable(int sensor_type, bool on)
{
    sns_lux_aod_inst_state *inst_state = (sns_lux_aod_inst_state *)lux_aod_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    LUX_AOD_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = lux_aod_mTask->cb->get_service_manager(lux_aod_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_std_sensor_config light_config = sns_std_sensor_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    if (on) {
        switch (sensor_type) {
        case LIGHT_TYPE:
            light_config.sample_rate = inst_state->state->light_sampleRate;

            if (NULL == inst_state->resampler_light_stream) {
                LUX_AOD_LOG_0("create resampler light stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, lux_aod_mTask,
                    inst_state->light_suid, &inst_state->resampler_light_stream);
            }

            if (NULL != inst_state->resampler_light_stream) {
                encoded_len = pb_encode_request(buffer, sizeof(buffer), &light_config, sns_std_sensor_config_fields,
                        NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG,
                        .request_len = encoded_len, .request = buffer
                    };

                    rc = inst_state->resampler_light_stream->api->send_request(inst_state->resampler_light_stream,
                            &request);
                    LUX_AOD_LOG_2("Processed light config request: sample_rate %d, result %u",
                        inst_state->state->light_sampleRate, rc);
                }
            } else {
                LUX_AOD_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;
        }
    } else {
        switch (sensor_type) {
        case LIGHT_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(lux_aod_mTask, &inst_state->resampler_light_stream);
            break;
        }
    }
}

static void lux_aod_report(int move, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float lux_aod_data[2] = {0};
    sns_lux_aod_inst_state *inst_state = (sns_lux_aod_inst_state *)lux_aod_mTask->state->state;

    lux_aod_data[0] = move;
    lux_aod_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(lux_aod_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            lux_aod_data,
            2,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        LUX_AOD_LOG_0("free fall report failed, Error in sending event");
    } else {
        LUX_AOD_LOG_0("report lux_aod data.");
    }
}

static uint64_t lux_aod_get_delta_time_ms(int64_t timestamp)
{
    if (timestamp < 0) {
        return 0;
    } else {
        return sns_get_ms_time_from_tick(timestamp);
    }
}

static void lux_aod_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    lux_aod_sensor_data input;

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
            LUX_AOD_LOG_0("Error in decoding event");
        } else {
            //TP_GESTURE_LOG_3("data[%d %d %d]", (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]));
        }

        input.x = data[0];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        lux_aod_process_sensor_data(&input);
    }
}

static sns_rc sns_lux_aod_process_event(sns_sensor_instance *const this)
{
    sns_lux_aod_inst_state *inst_state = (sns_lux_aod_inst_state *)this->state->state;

    sns_sensor_event *light_event = NULL;

    if (NULL != inst_state->resampler_light_stream) {
        light_event = inst_state->resampler_light_stream->api->peek_input(
                inst_state->resampler_light_stream);

        while (NULL != light_event) {
            lux_aod_decode_and_checkdata(light_event, LIGHT_TYPE);

            if (NULL != inst_state->resampler_light_stream) {
                light_event = inst_state->resampler_light_stream->api->get_next_input(
                        inst_state->resampler_light_stream);
            } else {
                LUX_AOD_LOG_0("accel stream has been released.");
                light_event = NULL;
            }
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc sns_lux_aod_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_lux_aod_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_lux_aod_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_lux_aod_inst_init,
    .deinit = &sns_lux_aod_inst_deinit,
    .set_client_config = &sns_lux_aod_inst_set_client_config,
    .notify_event = &sns_lux_aod_inst_notify_event
};

struct lux_aod_sensor_operation lux_aod_ops = {
    .sensor_change_state = lux_aod_sensor_enable,
    .report = lux_aod_report,
    .get_delta_time_ms = lux_aod_get_delta_time_ms,
    .need_overload_thrd = lux_aod_need_overload_thrd,
    .get_new_thrd = lux_aod_get_new_thrd,
};

