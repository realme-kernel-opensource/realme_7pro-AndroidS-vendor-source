/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - sns_oplus_activity_recognition_sensor_instance_island.c
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

#include "sns_oplus_activity_recognition_sensor_instance.h"
#include "sns_remote_proc_state.pb.h"
#include "sns_island.h"
#include "sns_memmgr.h"
#include "sns_assert.h"

#define OPLUS_ACTIVITY_RECOGNITION_BATCH_PERIOD 1000000  // 1000ms

sns_sensor_instance *oplus_activity_recognition_mTask;
extern sns_sensor_api sns_oplus_activity_recognition_api;

static void oplus_activity_recognition_sensor_enable(int sensor_type, bool on)
{
    sns_oplus_activity_recognition_inst_state *inst_state = (sns_oplus_activity_recognition_inst_state *) oplus_activity_recognition_mTask->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    OPLUS_ACTIVITY_RECOGNITION_LOG_2("sensor enable operate, sensor_type = %d, on = %d", sensor_type, on);

    sns_service_manager *manager = oplus_activity_recognition_mTask->cb->get_service_manager(oplus_activity_recognition_mTask);
    sns_stream_service *stream_service = (sns_stream_service *) manager->get_service(manager, SNS_STREAM_SERVICE);

    sns_std_request std_req = sns_std_request_init_default;
    std_req.has_batching = true;
    std_req.batching.batch_period = OPLUS_ACTIVITY_RECOGNITION_BATCH_PERIOD;
    size_t encoded_len = 0;
    uint8_t buffer[100] = {0};
    if (on) {
        switch (sensor_type) {
        case REMOTE_PROC_TYPE:
            if (NULL == inst_state->remote_proc_sensor_stream) {
                OPLUS_ACTIVITY_RECOGNITION_LOG_0("create resampler remote proc stream.");

                // create connection with resampler sensor
                stream_service->api->create_sensor_instance_stream(stream_service, oplus_activity_recognition_mTask,
                                        inst_state->remote_proc_sensor_suid, &inst_state->remote_proc_sensor_stream);
            }

            if (NULL != inst_state->remote_proc_sensor_stream) {
                const pb_field_t *fields = sns_remote_proc_state_config_fields;
                sns_std_request std_req = sns_std_request_init_default;
                sns_remote_proc_state_config remote_proc_state_config;
                remote_proc_state_config.proc_type = SNS_STD_CLIENT_PROCESSOR_APSS;
                encoded_len = pb_encode_request(
                                        buffer, sizeof(buffer),
                                        &remote_proc_state_config,
                                        fields, &std_req);
                if ((0 < encoded_len) && inst_state->remote_proc_sensor_stream) {
                    sns_request request = (sns_request) {
                        .message_id = SNS_REMOTE_PROC_STATE_MSGID_SNS_REMOTE_PROC_STATE_CONFIG,
                        .request_len = encoded_len,
                        .request = buffer,
                    };
                    rc = inst_state->remote_proc_sensor_stream->api->send_request(inst_state->remote_proc_sensor_stream, &request);
                    if (SNS_RC_SUCCESS == rc) {
                        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Successfully sent config request to remote proc sensor");
                    } else {
                        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Failed to send config request");
                    }
                } else {
                    OPLUS_ACTIVITY_RECOGNITION_LOG_1("Failed to send sensor request, stream=%p", inst_state->remote_proc_sensor_stream);
                }
            }
            break;
        case MOTION_RECOGNITION_TYPE:
            if (NULL == inst_state->motion_recognition_sensor_stream) {
                OPLUS_ACTIVITY_RECOGNITION_LOG_1("create motion_recognition_sensor stream, sensor_uid = %x", inst_state->motion_recognition_sensor_suid);
                stream_service->api->create_sensor_instance_stream(stream_service, oplus_activity_recognition_mTask,
                        inst_state->motion_recognition_sensor_suid, &inst_state->motion_recognition_sensor_stream);
            }
            if (NULL != inst_state->motion_recognition_sensor_stream) {
                sns_std_sensor_config motion_recognition_config = sns_std_sensor_config_init_default;
                encoded_len = pb_encode_request(buffer,
                                                sizeof(buffer),
                                                &motion_recognition_config,
                                                sns_std_sensor_config_fields,
                                                NULL);
                if (0 < encoded_len) {
                    sns_request request = (sns_request){
                        .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                        .request_len = encoded_len,
                        .request = buffer
                    };
                    rc = inst_state->motion_recognition_sensor_stream->api->send_request(inst_state->motion_recognition_sensor_stream, &request);
                    if (SNS_RC_SUCCESS == rc) {
                        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Successfully sent config request to motion_recognition sensor.");
                    } else {
                        OPLUS_ACTIVITY_RECOGNITION_LOG_0("Failed to send config request.");
                    }
                } else {
                    OPLUS_ACTIVITY_RECOGNITION_LOG_0("Failed to encode motion_recognition request.");
                }
            } else {
                OPLUS_ACTIVITY_RECOGNITION_LOG_0("motion_recognition_sensor_stream is NULL.");
            }
            break;
        default:
            break;
        }
    } else {
        switch (sensor_type) {
        case REMOTE_PROC_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(oplus_activity_recognition_mTask, &inst_state->remote_proc_sensor_stream);
            break;
        case MOTION_RECOGNITION_TYPE:
            sns_sensor_util_remove_sensor_instance_stream(oplus_activity_recognition_mTask, &inst_state->motion_recognition_sensor_stream);
            break;
        default:
            break;
        }
    }
}

static void oplus_activity_recognition_report(int count, int ith, int incar, int activity, uint64_t delta_time)
{
    sns_rc rc = SNS_RC_SUCCESS;
    float oplus_activity_recognition_data[5] = {0}; // TODO: length depends on the event size which return to framework
    sns_oplus_activity_recognition_inst_state *inst_state = (sns_oplus_activity_recognition_inst_state *) oplus_activity_recognition_mTask->state->state;

    oplus_activity_recognition_data[0] = count;
    oplus_activity_recognition_data[1] = ith;
    oplus_activity_recognition_data[2] = incar;
    oplus_activity_recognition_data[3] = activity;
    oplus_activity_recognition_data[4] = delta_time;
    rc = pb_send_sensor_stream_event(oplus_activity_recognition_mTask,
                                     NULL,
                                     sns_get_system_time(),
                                     SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                     SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                     oplus_activity_recognition_data,
                                     5,
                                     inst_state->config.encoded_data_event_len);
    if (SNS_RC_SUCCESS != rc) {
        OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition report failed, Error in sending event");
    } else {
        OPLUS_ACTIVITY_RECOGNITION_LOG_0("report oplus_activity_recognition data.");
    }
}

static uint64_t oplus_activity_recognition_get_delta_time_ms(uint64_t timestamp)
{
    return sns_get_ms_time_from_tick(timestamp);
}

static void oplus_activity_recognition_decode_and_checkdata(sns_sensor_event *event_in, SENSOR_TYPE type_in)
{
    uint8_t arr_index = 0;
    float data[6] = {0};
    oplus_activity_recognition_sensor_data input;

    if ((SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) && (MOTION_RECOGNITION_TYPE == type_in)) {
        pb_float_arr_arg data_arg = { .arr = data,
                                      .arr_len = ARR_SIZE(data),
                                      .arr_index = &arr_index};
        sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;
        decoded_event.data = (pb_callback_t){.funcs.decode = pb_decode_float_arr_cb,
                                             .arg = &data_arg};
        pb_istream_t istream = pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);
        if (!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
            OPLUS_ACTIVITY_RECOGNITION_LOG_0("Error in decoding event.");
        } else {
            OPLUS_ACTIVITY_RECOGNITION_LOG_3("motion_recognition data[%d, %d, %d]", (int)data[0], (int)data[1], (int)data[2]);
            input.x_value = data[0];
            input.y_value = data[1];
            input.z_value = data[2];
            input.timestamp = event_in->timestamp;
            input.type = type_in;
            oplus_activity_recognition_check_sensor_data(&input);
        }
    }
    // TODO decode the event if it's different from STD_SENSOR_EVENT
}

static void oplus_activity_recognition_handle_single_remote_proc_event(sns_sensor_event *event_in)
{
    sns_std_client_processor proc_type = SNS_STD_CLIENT_PROCESSOR_APSS;
    oplus_activity_recognition_sensor_data input;

    if (SNS_REMOTE_PROC_STATE_MSGID_SNS_REMOTE_PROC_STATE_EVENT == event_in->message_id) {
        pb_istream_t stream =
                pb_istream_from_buffer((pb_byte_t*)event_in->event, event_in->event_len);

        sns_remote_proc_state_event remote_proc_state_data =
                sns_remote_proc_state_event_init_default;

        if (!pb_decode(&stream, sns_remote_proc_state_event_fields,
                    &remote_proc_state_data)) {
            OPLUS_ACTIVITY_RECOGNITION_LOG_0("Error decoding Remote proc state sensor event");
        } else {
            proc_type = remote_proc_state_data.proc_type;
            sns_remote_proc_state_event_type event_type =
            remote_proc_state_data.event_type;

            //OPLUS_ACTIVITY_RECOGNITION_LOG_2("Got an event for proc %d event %d", proc_type,event_type);
            if (SNS_STD_CLIENT_PROCESSOR_APSS == proc_type) {
                input.x_value = event_type;
                input.timestamp = event_in->timestamp;
                input.type = REMOTE_PROC_TYPE;

                oplus_activity_recognition_check_sensor_data(&input);
            }
        }
    } else {
        OPLUS_ACTIVITY_RECOGNITION_LOG_0("CM Sensor received an event "
            "from remote processor which it does not understand");
    }
}

static void  oplus_activity_recognition_handle_remote_proc_event(
    sns_sensor_instance *const this)
{
    sns_oplus_activity_recognition_inst_state *inst_state = (sns_oplus_activity_recognition_inst_state*)this->state->state;
    sns_sensor_event *last_event_in = NULL;
    sns_sensor_event *event_in =
       inst_state->remote_proc_sensor_stream->api->peek_input(inst_state->remote_proc_sensor_stream);

    while (NULL != event_in) {
        last_event_in = event_in;
        event_in = inst_state->remote_proc_sensor_stream->api->get_next_input(inst_state->remote_proc_sensor_stream);
    }
    if (NULL != last_event_in) {
        oplus_activity_recognition_handle_single_remote_proc_event(last_event_in);
    }
}

static sns_rc sns_oplus_activity_recognition_process_event(sns_sensor_instance *const this)
{
    sns_oplus_activity_recognition_inst_state *inst_state = (sns_oplus_activity_recognition_inst_state *) this->state->state;
    sns_sensor_event *motion_recognition_event = NULL;

    if (NULL != inst_state->remote_proc_sensor_stream) {
        oplus_activity_recognition_handle_remote_proc_event(this);
    }

    if (NULL != inst_state->motion_recognition_sensor_stream) {
        motion_recognition_event = inst_state->motion_recognition_sensor_stream->api->peek_input(inst_state->motion_recognition_sensor_stream);
        while (NULL != motion_recognition_event) {
            oplus_activity_recognition_decode_and_checkdata(motion_recognition_event, MOTION_RECOGNITION_TYPE);
            if (NULL != inst_state->motion_recognition_sensor_stream) {
                motion_recognition_event = inst_state->motion_recognition_sensor_stream->api->get_next_input(inst_state->motion_recognition_sensor_stream);
            } else {
                OPLUS_ACTIVITY_RECOGNITION_LOG_0("motion_recognition_sensor stream has been released.");
                motion_recognition_event = NULL;
            }
        }
    }
    // TODO: read sensor input from stream

    return SNS_RC_SUCCESS;
}

static sns_rc sns_oplus_activity_recognition_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_oplus_activity_recognition_process_event(this);

    return rc;
}

sns_sensor_instance_api sns_oplus_activity_recognition_sensor_instance_api =
{
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_oplus_activity_recognition_inst_init,
    .deinit = &sns_oplus_activity_recognition_inst_deinit,
    .set_client_config = &sns_oplus_activity_recognition_inst_set_client_config,
    .notify_event = &sns_oplus_activity_recognition_inst_notify_event,
};

struct oplus_activity_recognition_sensor_operation oplus_activity_recognition_ops =
{
    .sensor_change_state = oplus_activity_recognition_sensor_enable,
    .report = oplus_activity_recognition_report,
    .get_delta_time_ms = oplus_activity_recognition_get_delta_time_ms,
    .get_current_time_tick = sns_get_system_time,
};
