/*=============================================================================
  @file sns_mag_fusion_sensor_instance.c

  The mag_fusion virtual Sensor Instance implementation

  Copyright (c) 2017 OnePlus Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - OnePlus Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/
#include "sns_mem_util.h"
#include "sns_mag_cal.pb.h"
#include "sns_gyro_cal.pb.h"
#include "sns_sensor_instance.h"
#include "sns_mag_fusion_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_island_service.h"
#include "sns_printf.h"

extern mag_fusion_api  akm_fusion_api;
extern mag_fusion_api  oplus_fusion_api;

mag_fusion_api * fusion_list[FUSION_TYPE_MAX] = {
    &akm_fusion_api,
    &oplus_fusion_api
};

/*============================================================================
  Preprocessor Definitions and Constants
  ===========================================================================*/

/*
#define EARLIER(earlier_event, later_event) ((earlier_event)->timestamp < (later_event)->timestamp)

bool get_earliest_input(
  sns_sensor_event *acc_event,
  sns_sensor_event *mag_event,
  sns_sensor_event *gyr_event,
  mag_fusion_input_type *ret_type,
  sns_sensor_event **ret_event)
{
  if (acc_event == NULL || mag_event == NULL || gyr_event == NULL){
    return false;
  }
  *ret_type = MF_INPUT_TYPE_ACC;
  *ret_event = acc_event;
  if(EARLIER(mag_event, *ret_event)){
    *ret_type = MF_INPUT_TYPE_MAG;
    *ret_event = mag_event;
  }
  if(EARLIER(gyr_event, *ret_event)){
    *ret_type = MF_INPUT_TYPE_GYR;
    *ret_event = gyr_event;
  }
  return true;
}
*/

mag_fusion_api * mag_fusion_get_fusion_list(mag_fusion_type fusion_type)
{
    if (fusion_type >=  FUSION_TYPE_MAX)
        fusion_type = AKM_FUSION;

    return fusion_list[fusion_type];
}

static bool get_any_input(
    sns_sensor_event *acc_event,
    sns_sensor_event *mag_event,
    sns_sensor_event *gyr_event,
    mag_fusion_input_type *ret_type,
    sns_sensor_event **ret_event)
{
    if (acc_event) {
        *ret_type = MF_INPUT_TYPE_ACC;
        *ret_event = acc_event;
        return true;
    }

    if (mag_event) {
        *ret_type = MF_INPUT_TYPE_MAG;
        *ret_event = mag_event;
        return true;
    }

    if (gyr_event) {
        *ret_type = MF_INPUT_TYPE_GYR;
        *ret_event = gyr_event;
        return true;
    }

    return false;
}

bool encode_send_sns_cal_event(
    sns_sensor_instance *const this,
    mag_fusion_sample *const output_sample,
    bool new_request_report)
{
    sns_mag_fusion_inst_state *state = (sns_mag_fusion_inst_state*)this->state->state;
    bool rv = true;

    if (!output_sample->ts)
        return rv;

    if (!new_request_report) {
        // restrict report rate to fit CTS test
        //if (output_sample->ts - state->last_sample.ts < state->max_report_interval) {
        //return rv;
        //}

        // avoid repeating same values
        if (state->last_sample.ts != 0 &&
            !memcmp(&state->last_sample.output,
                &output_sample->output,
                sizeof(mag_fusion_output)))

            return rv;

    }

    sns_cal_event event_out = sns_cal_event_init_default;
    pb_float_arr_arg arg_bias =
    {.arr = output_sample->output.bias, .arr_len =  ARR_SIZE(output_sample->output.bias), .arr_index = NULL };
    event_out.bias = (struct pb_callback_s) {
        .funcs.encode = &pb_encode_float_arr_cb, .arg = &arg_bias
    };
    event_out.status = output_sample->output.accuracy;

    rv = pb_send_event(this,
            sns_cal_event_fields,
            &event_out,
            output_sample->ts,
            SNS_CAL_MSGID_SNS_CAL_EVENT,
            NULL);

    MAG_FUSION_LOGI("send event, [ %d/1000, %d/1000, %d/1000 ] accu:%d ts=%llu(us)",
        (int)(1000 * output_sample->output.bias[0]),
        (int)(1000 * output_sample->output.bias[1]),
        (int)(1000 * output_sample->output.bias[2]),
        output_sample->output.accuracy,
        SNS_TIME_CONVERT_TO_US(output_sample->ts));

    memcpy(&state->last_sample, output_sample, sizeof(mag_fusion_sample));
    return rv;
}

#ifdef MAG_FUSION_SEND_DEBUG_INFO
static bool encode_send_debug_event(
    sns_sensor_instance *const this,
    sns_std_sensor_sample_status accuracy,
    sns_time timestamp,
    sns_sensor_uid *const sensor_uid)
{
    sns_mag_fusion_inst_state *state = (sns_mag_fusion_inst_state*)this->state->state;
    float debug_info[16];

    state->fusion_api->get_debug_info(state->vendor_algo, debug_info);

    sns_rc rc = pb_send_sensor_stream_event(this,
            sensor_uid,
            timestamp,
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
            accuracy,
            debug_info,
            16,
            pb_get_encoded_size_sensor_stream_event(debug_info, 16));

    if (rc != SNS_RC_SUCCESS) {
        MAG_FUSION_LOGI("sending debug event error %d", rc);
        return false;
    }

    return true;
}

#endif

/**
 * Process incoming events to the MAG_FUSION Sensor Instance.
 */
static sns_rc sns_mag_fusion_inst_notify_event(
    sns_sensor_instance *const this)
{
    sns_rc rc = SNS_RC_SUCCESS;

    sns_mag_fusion_inst_state *state = (sns_mag_fusion_inst_state*)this->state->state;
    mag_fusion_input_type input_type;
    mag_fusion_sample output_sample;
    sns_sensor_event *event_in;

    MAG_FUSION_LOGDB("%s: enter", __func__);
    memset(&output_sample, 0, sizeof(mag_fusion_sample));

    // process gyro_cal event
    sns_sensor_event *gyro_cal_event;

    for(gyro_cal_event = state->gyro_cal_stream->api->peek_input(state->gyro_cal_stream);
        gyro_cal_event != NULL;
        gyro_cal_event = state->gyro_cal_stream->api->get_next_input(state->gyro_cal_stream)) {

        MAG_FUSION_LOGDB("receiving gyro_cal_event. message_id=%d, timestamp=%llu",
            gyro_cal_event->message_id, gyro_cal_event->timestamp);

        if(SNS_CAL_MSGID_SNS_CAL_EVENT == gyro_cal_event->message_id) {

            uint8_t arr_index = 0;
            pb_float_arr_arg bias_arg =
            {.arr = state->gyro_bias, .arr_len = 3, .arr_index = &arr_index};

            pb_istream_t stream =
                pb_istream_from_buffer((pb_byte_t*)gyro_cal_event->event, gyro_cal_event->event_len);
            sns_cal_event cal_event = sns_cal_event_init_default;
            cal_event.bias = (pb_callback_t) {
                .funcs.decode = &pb_decode_float_arr_cb, .arg = &bias_arg
            };

            if(!pb_decode(&stream, sns_cal_event_fields, &cal_event)) {
                MAG_FUSION_LOGI("Error in decoding gyro_cal_event->message_id=%d", gyro_cal_event->message_id);
                continue;
            }

            MAG_FUSION_LOGDT("gyro_cal bias=[ %d/100000 %d/100000 %d/100000 ] accuracy=%d",
                (int)(state->gyro_bias[0] * 100000),
                (int)(state->gyro_bias[1] * 100000),
                (int)(state->gyro_bias[2] * 100000),
                cal_event.status);
        }
    }

    // process sample events
    sns_sensor_event *acc_event = state->accel_stream->api->peek_input(state->accel_stream);
    sns_sensor_event *mag_event = state->mag_stream->api->peek_input(state->mag_stream);
    sns_sensor_event *gyr_event = state->gyro_stream->api->peek_input(state->gyro_stream);

    while(get_any_input(acc_event, mag_event, gyr_event, &input_type, &event_in)) {
        MAG_FUSION_LOGDB("input_type=%d message_id=%d timestamp=%llu", input_type, event_in->message_id, event_in->timestamp);

        if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event_in->message_id) {
            mag_fusion_data_input vendor_input = {
                .type = input_type,
                .timestamp = event_in->timestamp
            };

            uint8_t arr_index = 0;
            pb_float_arr_arg data_arg =
            {.arr = vendor_input.data, .arr_len = ARR_SIZE(vendor_input.data), .arr_index = &arr_index};
            sns_std_sensor_event decoded_event = sns_std_sensor_event_init_default;
            decoded_event.data = (pb_callback_t) {
                .funcs.decode = pb_decode_float_arr_cb, .arg = &data_arg
            };

            pb_istream_t istream = pb_istream_from_buffer(
                    (pb_byte_t*)event_in->event, event_in->event_len);

            if(!pb_decode(&istream, sns_std_sensor_event_fields, &decoded_event)) {
                MAG_FUSION_LOGI("sns_mag_fusion_inst_notify_event : Error in decoding event");
            } else {//decode success
                mag_fusion_output vendor_output = {.result_changed = 0};

                if(vendor_input.type == MF_INPUT_TYPE_GYR) {
                    vendor_input.data[0] -= state->gyro_bias[0];
                    vendor_input.data[1] -= state->gyro_bias[1];
                    vendor_input.data[2] -= state->gyro_bias[2];
                }

                MAG_FUSION_LOGDT("input type %d, data [ %d %d %d ]/1000, ts %llu",
                    vendor_input.type,
                    (int)(vendor_input.data[0] * 1000),
                    (int)(vendor_input.data[1] * 1000),
                    (int)(vendor_input.data[2] * 1000),
                    SNS_TIME_CONVERT_TO_NS(vendor_input.timestamp));

                // deliver input data to vendor's algorithm
                vendor_output.result_changed = 0;
                int vendor_ret = state->fusion_api->algo_update(&vendor_input, &vendor_output);

                if(vendor_ret != 0)
                    MAG_FUSION_LOGI("Vendor algo failed with code %d", vendor_ret);

                // update output data
                if(vendor_output.result_changed)
                    memcpy(&output_sample.output,
                        &vendor_output,
                        sizeof(mag_fusion_output));

            }//end of decode success
        } else {
            MAG_FUSION_LOGI("unhandled message");
        }

        switch(input_type) {
        case MF_INPUT_TYPE_ACC:
            acc_event = state->accel_stream->api->get_next_input(state->accel_stream);
            break;

        case MF_INPUT_TYPE_MAG:
            mag_event = state->mag_stream->api->get_next_input(state->mag_stream);
            break;

        case MF_INPUT_TYPE_GYR:
            gyr_event = state->gyro_stream->api->get_next_input(state->gyro_stream);
            break;

        default:
            break;
        }
    }// end of "while(get_earliest_input(acc_event, mag_event, gyr_event, &vendor_input.type, &event_in))"

    MAG_FUSION_LOGI("%s, result_changed=%d", __func__, output_sample.output.result_changed);

    // encode and send calibration bias
    if(output_sample.output.result_changed) {
        output_sample.ts = sns_get_system_time();
        MAG_FUSION_LOGDB("MAG_FUSION output_valid [ %d/1000, %d/1000, %d/1000 ] accu:%d ts=%llu(us)",
            (int)(1000 * output_sample.output.bias[0]),
            (int)(1000 * output_sample.output.bias[1]),
            (int)(1000 * output_sample.output.bias[2]),
            output_sample.output.accuracy,
            SNS_TIME_CONVERT_TO_US(output_sample.ts));

        if(!encode_send_sns_cal_event(this, &output_sample, false))
            MAG_FUSION_LOGI("Error in sending event");

#ifdef MAG_FUSION_SEND_DEBUG_INFO
        encode_send_debug_event(this, output_sample.output.accuracy, curr_timestamp, NULL);
#endif

    }//end of "if(final_output.result_changed)"

    return rc;
}


/* See sns_sensor_instance_api::set_client_config */
sns_rc sns_mag_fusion_inst_set_client_config(
    sns_sensor_instance *const this,
    sns_request const *client_request)
{
    sns_rc rc = SNS_RC_SUCCESS;
    sns_mag_fusion_inst_state *state = (sns_mag_fusion_inst_state*)this->state->state;

    MAG_FUSION_LOGI("sns_mag_fusion_inst_set_client_config");

    if(SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == client_request->message_id) {
        // initialize state here
        sns_memscpy(&state->client_config,
            sizeof(state->client_config),
            client_request->request,
            client_request->request_len);

        // enable accel/mag/gyro here
        size_t encoded_len;
        uint8_t buffer[40];

        sns_resampler_config req_config[] = {
            {.resampled_rate = 50.0f},   // mag
            {.resampled_rate = 50.0f},  // gyro
            {.resampled_rate = 5.0f},   // accel
        };

        //MAG_FUSION_LOGDB("Enable mag   at rate %f", req_config[0].resampled_rate);
        sns_memscpy(&req_config[0].sensor_uid,
            sizeof(req_config[0].sensor_uid),
            &state->mag_suid,
            sizeof(state->mag_suid));
        req_config[0].rate_type = SNS_RESAMPLER_RATE_MINIMUM;
        req_config[0].filter = false;

        //MAG_FUSION_LOGDB("Enable gyro  at rate %f", req_config[1].resampled_rate);
        sns_memscpy(&req_config[1].sensor_uid,
            sizeof(req_config[1].sensor_uid),
            &state->gyro_suid,
            sizeof(state->gyro_suid));
        req_config[1].rate_type = SNS_RESAMPLER_RATE_MINIMUM;
        req_config[1].filter = false;

        //MAG_FUSION_LOGDB("Enable accel at rate %f", req_config[2].resampled_rate);
        sns_memscpy(&req_config[2].sensor_uid,
            sizeof(req_config[2].sensor_uid),
            &state->accel_suid,
            sizeof(state->accel_suid));
        req_config[2].rate_type = SNS_RESAMPLER_RATE_MINIMUM;
        req_config[2].filter = false;

        sns_data_stream *req_stream[] = {
            state->mag_stream,
            state->gyro_stream,
            state->accel_stream,
        };

        for (int i = 0; i < ARR_SIZE(req_stream); i++) {

            if (i == 2 && state->fusion_type == AKM_FUSION)//only akm fusion need accel data
                continue;

            encoded_len = pb_encode_request(buffer,
                    sizeof(buffer),
                    &req_config[i],
                    sns_resampler_config_fields,
                    NULL);

            if(0 < encoded_len && NULL != req_stream[i] ) {
                sns_request request = (sns_request) {
                    .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
                    .request_len = encoded_len, .request = buffer
                };
                rc = req_stream[i]->api->send_request(req_stream[i], &request);
                MAG_FUSION_LOGDB("Processed mag_fusion config request: index %d sample_rate %f, result %u",
                    i, (int)(req_config[i].resampled_rate), rc);
            } else {
                MAG_FUSION_LOGI("Error in creating stream[%d] OR encoding failed", i);
                rc = SNS_RC_NOT_SUPPORTED;
            }
        }

        // enable gyro_cal
        {
            sns_request request = (sns_request) {
                .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
                .request_len = 0, .request = NULL
            };
            rc = state->gyro_cal_stream->api->send_request(state->gyro_cal_stream, &request);
            MAG_FUSION_LOGI("Processed mag_fusion config request: gyro_cal on-change, result %u", rc);
        }
    } else {
        MAG_FUSION_LOGI("Unsupported request message id %u", client_request->message_id);
        rc = SNS_RC_NOT_SUPPORTED;
    }

    return rc;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_instance_api sns_mag_fusion_sensor_instance_api = {
    .struct_len = sizeof(sns_sensor_instance_api),
    .init = &sns_mag_fusion_inst_init,
    .deinit = &sns_mag_fusion_inst_deinit,
    .set_client_config = &sns_mag_fusion_inst_set_client_config,
    .notify_event = &sns_mag_fusion_inst_notify_event
};
