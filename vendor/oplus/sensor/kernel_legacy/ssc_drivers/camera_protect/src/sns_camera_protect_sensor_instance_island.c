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

#include "sns_camera_protect_sensor_instance.h"
#include "sns_gpio_service.h"
#define CAMERA_PROTECT_BATCH_PERIOD 100000  // 100ms
int INIT_PIN;
int IS_CHIP_PIN;

sns_sensor_instance *camera_protect_mTask;

static void sns_camera_protect_write_gpio(sns_sensor_instance *this, uint32_t gpio,
    bool is_chip_pin,
    sns_gpio_drive_strength drive_strength,
    sns_gpio_pull_type pull,
    sns_gpio_state gpio_state)
{
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_gpio_service *gpio_svc = (sns_gpio_service *)smgr->get_service(smgr, SNS_GPIO_SERVICE);
    sns_rc rc = SNS_RC_SUCCESS;

    rc = gpio_svc->api->write_gpio(gpio, is_chip_pin, drive_strength, pull, gpio_state);
    if (rc != SNS_RC_SUCCESS) {
        SNS_INST_PRINTF(ERROR, this, "rc = %d", rc);
    }
}

static void camera_protect_sensor_enable(int sensor_type, bool on)
{
    sns_camera_protect_inst_state *inst_state = (sns_camera_protect_inst_state *)
        camera_protect_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    INIT_PIN = inst_state->config.smem->parameter[2];
    IS_CHIP_PIN = inst_state->config.smem->parameter[3];

    float sample_rate = inst_state->state->sampleRate;

    CAMERA_PROTECT_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = camera_protect_mTask->cb->get_service_manager(camera_protect_mTask);
    sns_stream_service *stream_service = (sns_stream_service *)manager->get_service(manager,
            SNS_STREAM_SERVICE);

    sns_resampler_config resampler_config = sns_resampler_config_init_default;

    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = CAMERA_PROTECT_BATCH_PERIOD;

    if (on) {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->accel_suid, sizeof(inst_state->accel_suid));

            resampler_config.resampled_rate = sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_accel_stream) {
                CAMERA_PROTECT_LOG_0("create resampler acc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, camera_protect_mTask,
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
                    CAMERA_PROTECT_LOG_2("Processed accel config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                CAMERA_PROTECT_LOG_0("Error in creating accel_stream OR encoding failed");
            }
            break;

        case GYRO_TYPE:
            sns_memscpy(&resampler_config.sensor_uid, sizeof(resampler_config.sensor_uid),
                &inst_state->gyro_suid, sizeof(inst_state->gyro_suid));

            resampler_config.resampled_rate = sample_rate;
            resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
            resampler_config.filter = false;

            if (NULL == inst_state->resampler_gyro_stream) {
                CAMERA_PROTECT_LOG_0("create resampler gyro stream.");
                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, camera_protect_mTask,
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
                    CAMERA_PROTECT_LOG_2("Processed gyro config request: sample_rate %f, result %u", sample_rate, rc);
                }
            } else {
                CAMERA_PROTECT_LOG_0("Error in creating gyro_stream OR encoding failed");
            }

            break;
        }
    } else {
        switch (sensor_type) {
        case ACC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(camera_protect_mTask,
                &inst_state->resampler_accel_stream);
            break;
        case GYRO_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(camera_protect_mTask,
                &inst_state->resampler_gyro_stream);
            break;
        }
    }
}

static void camera_protect_report(int value, uint16_t report_count)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float camera_protect_data[2] = {0};
    sns_camera_protect_inst_state *inst_state = (sns_camera_protect_inst_state *)
        camera_protect_mTask->state->state;

    camera_protect_data[0] = value;
    camera_protect_data[1] = (float)report_count;

    rc = pb_send_sensor_stream_event(camera_protect_mTask,
            NULL,
            sns_get_system_time(),
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
            camera_protect_data,
            2,
            inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        CAMERA_PROTECT_LOG_0("free fall report failed, Error in sending event");
    } else {
        CAMERA_PROTECT_LOG_0("report camera_protect data.");
    }
}

static uint64_t camera_protect_get_delta_time_ms(uint64_t timestamp)
{
    return  sns_get_ms_time_from_tick(timestamp);
}

static void camera_protect_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    camera_protect_sensor_data input;

    if (
        (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id)
    ) {
        //CAMERA_PROTECT_LOG_3("earliest input_type = %d, message_id = %d, timestamp = %llu", type_in, event_in->message_id, event_in->timestamp);

        pb_float_arr_arg data_arg = {.arr = data, .arr_len = ARR_SIZE(data), .arr_index = &arr_index};

        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;

        decoded_event.data = (pb_callback_t) {
            .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
        };

        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t *)event_in->event, event_in->event_len);

        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            CAMERA_PROTECT_LOG_0("Error in decoding event");
        } else {
            CAMERA_PROTECT_LOG_3("data[%d %d %d]", (int)(1000 * data[0]), (int)(1000 * data[1]),
                (int)(1000 * data[2]));
        }

        input.x = data[0];
        input.y = data[1];
        input.z = data[2];
        input.timestamp = event_in->timestamp;
        input.type = type_in;

        camera_protect_process_sensor_data(&input);
    }
}

static sns_rc sns_camera_protect_process_event(sns_sensor_instance *const this)
{
    sns_camera_protect_inst_state *inst_state = (sns_camera_protect_inst_state *)this->state->state;

    sns_sensor_event *acc_event = NULL;
    sns_sensor_event *gyro_event = NULL;
    if (NULL != inst_state->resampler_accel_stream) {
        acc_event = inst_state->resampler_accel_stream->api->peek_input(inst_state->resampler_accel_stream);

        while (NULL != acc_event) {
            camera_protect_decode_and_checkdata(acc_event, ACC_TYPE);

            // stream maybe released in camera_protect_decode_and_checkdata(), so must judge this stream
            if (NULL != inst_state->resampler_accel_stream) {
                acc_event = inst_state->resampler_accel_stream->api->get_next_input(
                        inst_state->resampler_accel_stream);
            } else {
                // if stream has been released, event must be set to NULL to break the while()
                CAMERA_PROTECT_LOG_0("accel stream has been released.");
                acc_event = NULL;
            }
        }
    }
    if (NULL != inst_state->resampler_gyro_stream) {
        gyro_event = inst_state->resampler_gyro_stream->api->peek_input(inst_state->resampler_gyro_stream);

        while (NULL != gyro_event) {
            camera_protect_decode_and_checkdata(gyro_event, GYRO_TYPE);

            if (NULL != inst_state->resampler_gyro_stream) {
                gyro_event = inst_state->resampler_gyro_stream->api->get_next_input(
                        inst_state->resampler_gyro_stream);
            } else {
                CAMERA_PROTECT_LOG_0("gyro stream has been released.");
                gyro_event = NULL;
            }
        }
    }
    return SNS_RC_SUCCESS;
}

static sns_rc sns_camera_protect_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;
    static int first_check = 1;

    if (first_check) {
        first_check = 0;
        sns_camera_protect_write_gpio(this, INIT_PIN, IS_CHIP_PIN, SNS_GPIO_DRIVE_STRENGTH_2_MILLI_AMP,
            SNS_GPIO_PULL_TYPE_PULL_DOWN, SNS_GPIO_STATE_LOW);
    }

    sns_camera_protect_process_event(this);

    return rc;
}
static void camera_protect_set_int_gpio_high(void)
{
    CAMERA_PROTECT_LOG_0("camera_protect_set_int_gpio_high.");
    sns_camera_protect_write_gpio(camera_protect_mTask, INIT_PIN, IS_CHIP_PIN,
        SNS_GPIO_DRIVE_STRENGTH_2_MILLI_AMP,
        SNS_GPIO_PULL_TYPE_PULL_UP, SNS_GPIO_STATE_HIGH);
}
static void camera_protect_set_int_gpio_low(void)
{
    CAMERA_PROTECT_LOG_0("camera_protect_set_int_gpio_low.");
    sns_camera_protect_write_gpio(camera_protect_mTask, INIT_PIN, IS_CHIP_PIN,
        SNS_GPIO_DRIVE_STRENGTH_2_MILLI_AMP,
        SNS_GPIO_PULL_TYPE_PULL_DOWN, SNS_GPIO_STATE_LOW);
}
static bool camera_protect_need_gyro_compensation(void)
{
    return true;
}

sns_sensor_instance_api sns_camera_protect_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_camera_protect_inst_init,
    .deinit = &sns_camera_protect_inst_deinit,
    .set_client_config = &sns_camera_protect_inst_set_client_config,
    .notify_event = &sns_camera_protect_inst_notify_event
};

struct camera_protect_sensor_operation camera_protect_ops = {
    .sensor_change_state = camera_protect_sensor_enable,
    .report = camera_protect_report,
    .get_delta_time_ms = camera_protect_get_delta_time_ms,
    .set_int_gpio_high = camera_protect_set_int_gpio_high,
    .set_int_gpio_low = camera_protect_set_int_gpio_low,
    .need_gyro_compensation = camera_protect_need_gyro_compensation,
    .get_gyro_compensation_value = camera_protect_get_gyro_compensation_value,
};

