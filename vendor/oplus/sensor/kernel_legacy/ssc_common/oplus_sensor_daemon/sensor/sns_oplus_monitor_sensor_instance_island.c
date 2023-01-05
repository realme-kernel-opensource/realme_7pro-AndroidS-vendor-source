/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_xx.x
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "sns_oplus_monitor_sensor_instance.h"

sns_rc oplus_monitor_start_timer(sns_sensor_instance const *this, int period)
{
    sns_oplus_monitor_inst_state *state = (sns_oplus_monitor_inst_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    sns_rc rc = SNS_RC_SUCCESS;
    uint8_t buffer[50] = {0};

    sns_request timer_req = {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };
    req_payload.is_periodic    = true;
    req_payload.start_time     = sns_get_system_time();
    req_payload.timeout_period = sns_convert_ns_to_ticks((sns_time)((sns_time)period * 1000 * 1000));//ms
    timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload, sns_timer_sensor_config_fields, NULL);

    if ((timer_req.request_len > 0) && (state->timer_stream)) {
        rc = state->timer_stream->api->send_request(state->timer_stream, &timer_req);
    }

    return rc;
}

static sns_rc sns_oplus_monitor_process_timer_event(sns_sensor_instance *const this,
    sns_sensor_event *event)
{
    float data = 0;
    int smem_fifo_len = 0;
    sns_rc rc = SNS_RC_SUCCESS;
    sns_oplus_monitor_inst_state *state = (sns_oplus_monitor_inst_state*)this->state->state;

    state->timer_count++;

#ifdef OPLUS_FEATURE_SENSOR_POWER_FB
    //oplus_show_fw_sensor();
    oplus_save_power_info();
#endif

    if (!state->fw_is_inited || state->timer_count >= REPORT_EVENT_TIMEOUT) {
        if (!state->fw_is_inited) {
            state->fw_is_inited = true;
            oplus_init_fw_sensor();
            oplus_save_available_sensor_info();
            oplus_monitor_start_timer(this, TIMER_PERIOD);//1min
        }
        oplus_copy_to_smem_fifo();
        state->timer_count = 0;
        smem_fifo_len = oplus_get_smem_fifo_len();
        data = 1.0 * smem_fifo_len;

        SNS_INST_PRINTF(ERROR, this, "try to update smem and report %d", smem_fifo_len);

        sns_timer_sensor_event timer_event;
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                              event->event_len);

        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
            if (smem_fifo_len > 0) {
                rc = pb_send_sensor_stream_event(this,
                        NULL,
                        event->timestamp,
                        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                        SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                        &data,
                        1,
                        state->config.encoded_data_event_len);
            }
        }
    }

    return rc;
}

static sns_rc sns_oplus_monitor_inst_notify_event(sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_oplus_monitor_inst_state *state = (sns_oplus_monitor_inst_state*)this->state->state;
    sns_sensor_event *event = NULL;

    SNS_INST_PRINTF(LOW, this, "sns_oplus_monitor_inst_notify_event enter");

    if (NULL != state->timer_stream) {
            event = state->timer_stream->api->peek_input(state->timer_stream);

            while (NULL != event) {
                    if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                            SNS_INST_PRINTF(ERROR, this, "sns_oplus_monitor_inst_notify_event");
                            sns_oplus_monitor_process_timer_event(this, event);
                    } else if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT) {
                            /** TODO: decode and qse timer_reg_event*/
                            SNS_INST_PRINTF(LOW, this, "LAPS_TIMER_SENSOR_REG_EVENT");
                    } else {
                            SNS_INST_PRINTF(MED, this, "unknown message_id %d", event->message_id);
                    }
                    event = state->timer_stream->api->get_next_input(state->timer_stream);
            }
    }
    return rc;
}

sns_sensor_instance_api sns_oplus_monitor_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_oplus_monitor_inst_init,
    .deinit = &sns_oplus_monitor_inst_deinit,
    .set_client_config = &sns_oplus_monitor_inst_set_client_config,
    .notify_event = &sns_oplus_monitor_inst_notify_event
};
