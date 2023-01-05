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

#include "sns_logger_sensor_instance.h"
#include "sns_island_util.h"
#include "sns_island.h"

sns_logger_inst_state *g_logger_state = NULL;
int data_count = 0;
int timer_count = 0;

static void logger_register_timer(sns_sensor_instance *this, sns_time period)
{
    sns_logger_inst_state *state = (sns_logger_inst_state *)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t buffer[50];

    sns_request timer_req = {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };

    sns_rc rc = SNS_RC_SUCCESS;

    if (NULL == state->timer_data_stream) {
        sns_service_manager *smgr     = this->cb->get_service_manager(this);
        sns_stream_service  *strm_svc = (sns_stream_service *)smgr->get_service(smgr, SNS_STREAM_SERVICE);
        rc = strm_svc->api->create_sensor_instance_stream(strm_svc, this, state->timer_suid,
                &state->timer_data_stream);
    }

    if ((rc != SNS_RC_SUCCESS) || (NULL == state->timer_data_stream)) {
        SNS_INST_PRINTF(ERROR, this, "error creating stream %d", rc);
        return;
    }

    req_payload.is_periodic    = true;
    req_payload.start_time     = sns_get_system_time();
    req_payload.timeout_period = sns_convert_ns_to_ticks(period * 1000 * 1000);
    timer_req.request_len      = pb_encode_request(buffer, sizeof(buffer), &req_payload,
            sns_timer_sensor_config_fields, NULL);

    if (timer_req.request_len > 0) {
        state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
        state->timer_active = true;
    }
}

void sensors_log_report(log_data_info log_data)
{
    sns_logger_inst_state *state = NULL;
    sns_sensor_instance *this = NULL;

    static int data_count = 0;
    if (!g_logger_state) {
        return;
    }

    if (!sns_island_is_island_ptr((intptr_t)g_logger_state)) {
        SNS_ISLAND_EXIT();
    }

    state = g_logger_state;
    this = state->owner;
    if ((state == NULL) || (state->enabled == false)) {
        return;
    }

    SNS_INST_PRINTF(LOW, this, "data_count = %d.", data_count);

    if (state->data_buffer[data_count].to_report == false) {
        state->data_buffer[data_count].to_report = true;
        memset(&(state->data_buffer[data_count].log_data), 0, sizeof(log_data_info));
        memcpy(&(state->data_buffer[data_count].log_data), &log_data, sizeof(log_data_info));

        data_count ++;
        data_count %= LOG_BUFFER_SIZE;
    }
    //To prevent that during sns_logger_inst_deinit, it run into the following code
    //
    if (state->timer_active == false && state->enabled == true) {
        logger_register_timer(this, TIMER_EXECUTE_PERIOD);
    }
}

static sns_rc sns_logger_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_logger_inst_state *state = (sns_logger_inst_state *)this->state->state;
    sns_sensor_event *event;
    static int timer_count = 0;
    float logdata[sizeof(log_data_info) / sizeof(int)] = {0};
    //To prevent that during sns_logger_inst_deinit, it run into the following code
    if (NULL != state->timer_data_stream && state->enabled == true) {
        event = state->timer_data_stream->api->peek_input(state->timer_data_stream);

        while (NULL != event) {
            if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                sns_timer_sensor_event timer_event;
                pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event, event->event_len);

                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                    SNS_INST_PRINTF(LOW, this, "timer_count = %d.", timer_count);

                    while (state->data_buffer[timer_count].to_report == true) {
                        SNS_INST_PRINTF(LOW, this, "timer_count = %d.", timer_count);
                        state->data_buffer[timer_count].to_report = false;

                        logdata[0] = state->data_buffer[timer_count].log_data.sensor_id;
                        logdata[1] = state->data_buffer[timer_count].log_data.string_id;
                        logdata[2] = state->data_buffer[timer_count].log_data.argu2;
                        logdata[3] = state->data_buffer[timer_count].log_data.argu3;
                        logdata[4] = state->data_buffer[timer_count].log_data.argu4;
                        logdata[5] = state->data_buffer[timer_count].log_data.argu5;
                        logdata[6] = state->data_buffer[timer_count].log_data.argu6;

                        rc = pb_send_sensor_stream_event(this,
                                &state->logger_suid,
                                sns_get_system_time(),
                                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                                logdata,
                                ARR_SIZE(logdata),
                                state->config.encoded_data_event_len);
                        if (SNS_RC_SUCCESS != rc) {
                            SNS_INST_PRINTF(ERROR, this, "logger report failed, Error in sending event");
                        }

                        timer_count ++;
                        timer_count %= LOG_BUFFER_SIZE;

                    }
                }
            } else if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT) {
                SNS_INST_PRINTF(HIGH, this, "TIMER_SENSOR_REG_EVENT");
            } else {
                SNS_INST_PRINTF(HIGH, this, "unknown message_id %d", event->message_id);
            }

            event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
        }
    }

    return rc;
}

sns_sensor_instance_api sns_logger_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_logger_inst_init,
    .deinit = &sns_logger_inst_deinit,
    .set_client_config = &sns_logger_inst_set_client_config,
    .notify_event = &sns_logger_inst_notify_event
};

