/******************************************************************
** Copyright (C), 2004-2020 OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: - oplus_activity_recognition_algo.c
** Description: Source file for oplus_activity_recognition sensor.
** Version: 1.0
** Date : 2020/07/01
**
** --------------------------- Revision History: ---------------------
* <version>            <date>             <author>                            <desc>
*******************************************************************/

#include "oplus_activity_recognition_algo.h"

static oplus_activity_recognition_state g_oplus_activity_recognition_state;
static void oplus_activity_recognition_handle_timer_event(oplus_activity_recognition_state *state)
{
    OPLUS_ACTIVITY_RECOGNITION_LOG_1("oplus_activity_recognition_handle_timer_event : motion_count= %d", state->motion_count);
    for (int i = 0; i < state->motion_count; ++i) {
        uint64_t delta_time = state->sensor_ops->get_delta_time_ms(state->sensor_ops->get_current_time_tick()
                    - state->pre_motion_info[i].timestamp);
        state->sensor_ops->report(state->motion_count, i+1,
                    state->pre_motion_info[i].incar_state,
                    state->pre_motion_info[i].activity_mode,
                    delta_time);
    }
    state->motion_count = 0;
}

static void report_motion_recognition_event_handle(oplus_activity_recognition_state *state, oplus_activity_recognition_sensor_data *data)
{
    static uint16_t report_count = 0;
    report_count++;
    report_count %= 0xFFFF;
    state->pre_motion_info[state->motion_count].incar_state     = data->y_value;
    state->pre_motion_info[state->motion_count].activity_mode   = data->z_value;
    state->pre_motion_info[state->motion_count].timestamp       = data->timestamp;
    state->motion_count++;
    if (MAX_MOTION_BUFF == state->motion_count || AP_AWAKE == state->current_ap_state) {
        oplus_activity_recognition_handle_timer_event(state);
    }
}

void oplus_activity_recognition_check_sensor_data(oplus_activity_recognition_sensor_data *input)
{
    oplus_activity_recognition_state *state = &g_oplus_activity_recognition_state;
    switch (input->type) {
    case REMOTE_PROC_TYPE:
        OPLUS_ACTIVITY_RECOGNITION_LOG_1("oplus_activity_recognition_check_sensor_data remote_proc event %d", (int)input->x_value);
        state->current_ap_state = (uint8_t)input->x_value;
        if (AP_AWAKE == state->current_ap_state) {
            oplus_activity_recognition_handle_timer_event(state);
            OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition_check_sensor_data remote_proc event, AP goto wake.");
        } else {
            OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition_check_sensor_data remote_proc event, AP goto sleep.");
        }
        break;
    case MOTION_RECOGNITION_TYPE:
        report_motion_recognition_event_handle(state, input);
        break;

    default:
        break;
    }
}

void oplus_activity_recognition_Reset(void)
{
    oplus_activity_recognition_state *state = &g_oplus_activity_recognition_state;
    state->current_ap_state = AP_AWAKE;
    memset(&state->pre_motion_info, 0, sizeof(state->pre_motion_info));
    OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition Reset.");

    state->sensor_ops->sensor_change_state(REMOTE_PROC_TYPE, true);
    state->sensor_ops->sensor_change_state(MOTION_RECOGNITION_TYPE, true);
}

void oplus_activity_recognition_close(void)
{
    oplus_activity_recognition_state *state = &g_oplus_activity_recognition_state;
    state->sensor_ops->sensor_change_state(REMOTE_PROC_TYPE, false);
    state->sensor_ops->sensor_change_state(MOTION_RECOGNITION_TYPE, false);
    memset(&state, 0, sizeof(state));
    OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition close.");
}

void oplus_activity_recognition_algo_register(oplus_activity_recognition_state **state)
{
    OPLUS_ACTIVITY_RECOGNITION_LOG_0("oplus_activity_recognition algo register.");
    memset(&g_oplus_activity_recognition_state, 0, sizeof(oplus_activity_recognition_state));
    *state = &g_oplus_activity_recognition_state;
}
