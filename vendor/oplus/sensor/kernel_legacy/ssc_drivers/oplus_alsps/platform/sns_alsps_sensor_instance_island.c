/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"
#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"

#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"
#include "oplus_alsps.h"

#ifdef OPLUS_FEATURE_SENSOR_FB
#include "oplus_fb_utils.h"
#endif

sns_rc alsps_inst_notify_event(sns_sensor_instance* const this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    // Turn COM port ON
    state->scp_service->api->sns_scp_update_bus_power(state->port_handle, true);

    alsps_process_events(this);

    // Turn COM port off
    state->scp_service->api->sns_scp_update_bus_power(state->port_handle, false);
    return SNS_RC_SUCCESS;
}

static void config_ps_phone_mode(sns_sensor_instance* const this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if ((fabs(state->ps_info.sampling_rate_hz - 20) < 0.001) && (state->ps_info.is_phone_mode == false)) {
        state->ps_info.is_phone_mode = true;
        state->amode_flag.need_report_far = false;
    } else if ((fabs(state->ps_info.sampling_rate_hz - 20) > 0.001)) {
        state->ps_info.is_phone_mode = false;
    }

    state->amode_flag.first_acc_check = true;
    state->amode_flag.is_delay_report = false;
    state->amode_flag.first_near_by = false;

    if (state->ps_info.is_ps_enable) {
        if (state->ps_info.is_phone_mode && ps_need_gesture()) {
            alsps_register_accel_by_resampler(this, ACCEL_DATA, 60, 50);
        } else {
            alsps_register_accel_by_resampler(this, ACCEL_DATA, 100, 10);
        }
    }
}
static sns_rc alsps_inst_set_client_config(sns_sensor_instance* const this,
    sns_request const* client_request)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_alsps_req *payload = (sns_alsps_req*)client_request->request;

    // Turn COM port ON
    state->scp_service->api->sns_scp_update_bus_power(state->port_handle, true);

    if (state->is_first_init) {
        state->is_first_init = false;
        alsps_init_driver(this, payload->sensor_type);
    }

    if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
        if (payload->sensor_type == ALS) {
            OPLUS_ALS_PS_LOG("ALS scale(%d) = %d, bias(%d) = %d",
                state->als_cal_version[0],
                (uint32_t)(state->als_fac_cal_data[0]),
                state->als_cal_version[1],
                (uint32_t)state->als_fac_cal_data[1]);

            state->als_info.scale               = state->als_fac_cal_data[0];
            state->als_info.bias                = state->als_fac_cal_data[1];
            state->als_info.report_rate_hz      = payload->desired_report_rate;
            state->als_info.sampling_rate_hz = payload->desired_sample_rate;
            sns_time curr_time = sns_get_system_time();
            alsps_send_config_event(this, curr_time, ALS);

            //if is under lcd type need to verify open mode
            if (get_als_fac_mode(this) == 1) {
                state->als_under_info.buffer_lenth = state->als_under_info.persist_length / 2;
            }

            OPLUS_ALS_PS_LOG("als_type = %d  state->als_under_info.buffer_lenth = %d\n",
                (int)state->als_type, state->als_under_info.buffer_lenth);

            if (state->als_info.enable != state->als_info.is_als_enable) {
                alsps_als_enable(this, state->als_info.is_als_enable);
            }

            OPLUS_ALS_PS_LOG("als enable_count %d is_factory_mode = %d\n", state->als_info.enable_count, get_als_fac_mode(this));
        } else if (payload->sensor_type == PS) {
            OPLUS_ALS_PS_LOG("PS offset(%d) = %d, 3cm(%d) = %d delta(%d) = %d goal(%d) = %d up_thrd(%d) = %d",
                state->ps_cal_version[0],
                (int)state->ps_fac_cal_data[0],
                state->ps_cal_version[1],
                (int)state->ps_fac_cal_data[1],
                state->ps_cal_version[2],
                (int)state->ps_fac_cal_data[2],
                state->ps_cal_version[3],
                (int)state->ps_fac_cal_data[3],
                state->ps_cal_version[4],
                (int)state->ps_fac_cal_data[4]);

            state->ps_info.report_rate_hz = payload->desired_report_rate;
            state->ps_info.sampling_rate_hz = payload->desired_sample_rate;
            state->ps_info.ps_cali_goal         = state->ps_fac_cal_data[3];

            config_ps_phone_mode(this);

            state->ps_info.ps_cali_up_thrd = state->ps_fac_cal_data[4];

            sns_time curr_time = sns_get_system_time();
            alsps_send_config_event(this, curr_time, PS);

            if (state->ps_info.enable != state->ps_info.is_ps_enable) {
                alsps_ps_enable(this, state->ps_info.is_ps_enable);
#ifdef OPLUS_FEATURE_SENSOR_FB
                oplus_set_proximity_enable(state->ps_info.is_ps_enable);
#endif
            }


            OPLUS_ALS_PS_LOG("ps enable_count %d phone_mode = %d samplerate= %d\n",
                state->ps_info.enable_count, state->ps_info.is_phone_mode, (int)state->ps_info.sampling_rate_hz);
        }
    } else if (client_request->message_id ==
        SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
        state->island_service->api->sensor_instance_island_exit(state->island_service, this);
        alsps_run_sensor_test(this);
    }

    // Turn COM port off
    state->scp_service->api->sns_scp_update_bus_power(state->port_handle, false);
    return 0;
}

sns_sensor_instance_api alsps_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &alsps_inst_init,
    .deinit = &alsps_inst_deinit,
    .set_client_config = &alsps_inst_set_client_config,
    .notify_event = &alsps_inst_notify_event
};

