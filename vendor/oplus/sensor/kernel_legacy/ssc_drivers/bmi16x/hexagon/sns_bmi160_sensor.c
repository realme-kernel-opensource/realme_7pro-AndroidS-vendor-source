/*******************************************************************************
 * Copyright (c) 2017-2020, Bosch Sensortec GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file sns_bmi160.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"

#include "sns_bmi160_sensor.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_cal_util.h"
#include "sns_registry.pb.h"
#ifdef BMI160_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif
#include "sns_devinfo_utils.h"


#define BMI160_REG_NN_ACCEL                     "bmi160_0.accel.config"
#define BMI160_REG_NN_GYRO                      "bmi160_0.gyro.config"
#define BMI160_REG_NN_TEMP                      "bmi160_0.temp.config"
#define BMI160_REG_NN_MD                        "bmi160_0.md.config"
#define BMI160_REG_NN_PLATFORM_CONFIG           "bmi160_0_platform.config"
#define BMI160_REG_NN_PLATFORM_PLACEMENT        "bmi160_0_platform.placement"
#define BMI160_REG_NN_PLATFORM_ORIENT           "bmi160_0_platform.orient"
#define BMI160_REG_NN_PLATFORM_FAC_CAL_ACCEL    "bmi160_0_platform.accel.fac_cal"
#define BMI160_REG_NN_PLATFORM_FAC_CAL_GYRO     "bmi160_0_platform.gyro.fac_cal"
#define BMI160_REG_NN_PLATFORM_FAC_CAL_TEMP     "bmi160_0_platform.temp.fac_cal"
#define BMI160_REG_NN_PLATFORM_MD_CONFIG        "bmi160_0_platform.md.config"
#define BMI160_FAC_CAL_BIAS 	".bias"

typedef struct pb_arg_reg_group_arg
{
    sns_sensor_instance* instance;
    const char*         name;
    bmi160_sensor_type  sensor;
    uint32_t            version;
} pb_arg_reg_group_arg;


extern const range_attr bmi160_accel_ranges[];
extern const float bmi160_accel_resolutions[];
extern const range_attr bmi160_gyro_ranges[];
extern const float bmi160_gyro_resolutions[];



// <power rail>
#if BMI160_CONFIG_POWER_RAIL
static sns_rc bmi160_register_power_rail(sns_sensor *const this)
{
    bmi160_state *sstate = (bmi160_state*)this->state->state;
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_rc rv = SNS_RC_SUCCESS;

    sstate->common.rail_config.rail_vote = SNS_RAIL_OFF;

    if(NULL == sstate->pwr_rail_service)
    {
        sstate->pwr_rail_service =
            (sns_pwr_rail_service*)smgr->get_service(smgr, SNS_POWER_RAIL_SERVICE);

        sstate->pwr_rail_service->api->sns_register_power_rails(sstate->pwr_rail_service, &sstate->common.rail_config);
    }

    if(NULL == sstate->pwr_rail_service)
    {
        rv = SNS_RC_FAILED;
    }
    return rv;
}
#endif

/**
 * Returns decoded request message for type
 * sns_std_sensor_config.
 *
 * @param[in] in_request   Request as sorted in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool bmi160_get_decoded_imu_request(sns_sensor const *this, sns_request const *in_request,
        sns_std_request *decoded_request,
        sns_std_sensor_config *decoded_payload)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg =
    { .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields };
    decoded_request->payload = (struct pb_callback_s)
    { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
    stream = pb_istream_from_buffer(in_request->request,
            in_request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
    {
        BMI160_SENSOR_LOG(ERROR, this, "decode error");
        return false;
    }
    return true;
}

/**
 * Decodes self test requests.
 *
 * @param[i] this              Sensor reference
 * @param[i] request           Encoded input request
 * @param[o] decoded_request   Decoded standard request
 * @param[o] test_config       decoded self test request
 *
 * @return bool True if decoding is successful else false.
 */
static bool bmi160_get_decoded_self_test_request(
        sns_sensor const        *this,
        sns_request const       *request,
        sns_std_request         *decoded_request,
        sns_physical_sensor_test_config *test_config)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg =
    { .decoded_struct = test_config,
        .fields = sns_physical_sensor_test_config_fields };
    decoded_request->payload = (struct pb_callback_s)
    { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
    stream = pb_istream_from_buffer(request->request,
            request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
    {
        BMI160_SENSOR_LOG(ERROR, this, "BMI160 decode error");
        return false;
    }
    return true;
}


#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
static bool bmi160_get_decoded_self_test_request_bias(sns_sensor const *this, sns_request const *request,
        sns_std_request *decoded_request,
        sns_physical_sensor_oem_config *test_config)
{
    pb_istream_t stream;
    pb_simple_cb_arg arg =
    {
        .decoded_struct = test_config,
        .fields = sns_physical_sensor_oem_config_fields
    };
    decoded_request->payload = (struct pb_callback_s)
    {
        .funcs.decode = &pb_decode_simple_cb,
        .arg = &arg
    };
    stream = pb_istream_from_buffer(request->request, request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
    {
        BMI160_SENSOR_LOG(ERROR, this, "bmi160 :decode error");

        return false;
    }
    return true;
}
#endif


/**
 * get BMI160 ACC/Gyro configurations
 * @param this                                      sensor handler
 * @param instance                                  the valid instance handler
 * @param sensor_type                               sensor type from request
 * @param chosen_sample_rate                        chosen sample rate in hz
 * @param chosen_report_rate                        chosen report rate in hz
 * @param non_gated_sensor_client_present           non gated sensor client present flag
 * @param gated_sensor_client_present               gated sensor client present flag
 *
 * @return none
 */
static void bmi160_get_imu_config(sns_sensor *this,
        sns_sensor_instance *instance,
        bmi160_sensor_type  sensor_type,
        float               *chosen_sample_rate,
        float               *chosen_report_rate,
        sns_time            *chosen_flush_period_ticks,
        bool                *non_gated_sensor_client_present,
        bool                *gated_sensor_client_present)
{
#if BMI160_CONFIG_ENABLE_FLUSH_PERIOD
    UNUSED_VAR(this);
    sns_sensor_uid          suid;
    sns_request const       *request;
    bool is_max_batch = true;
    bool is_flush_only = true;

    if (BMI160_ACCEL == sensor_type)
    {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ACCEL_SUID), sizeof(sns_sensor_uid));
    }
    else if (BMI160_GYRO == sensor_type)
    {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)GYRO_SUID), sizeof(sns_sensor_uid));
    }
#if BMI160_CONFIG_ENABLE_MAG_IF //TODOMAG
    else if (BMI160_MAG == sensor_type)
    {
    }
#endif
    else
    {
    }

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *non_gated_sensor_client_present = false;
    *chosen_flush_period_ticks = 0;
    if (gated_sensor_client_present)
    {
        *gated_sensor_client_present = false;
    }

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for(request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false))
    {
        sns_std_request         decoded_request;
        sns_std_sensor_config   decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
                ||
                request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
            if (bmi160_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
            {
                float report_rate = 0.0f;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);
                if (decoded_request.has_batching)
                {
                    is_max_batch &= (decoded_request.batching.has_max_batch && decoded_request.batching.max_batch);


                    is_flush_only &= decoded_request.batching.flush_only;

                    if (decoded_request.batching.has_flush_period)
                    {
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks,
                                    sns_convert_ns_to_ticks(decoded_request.batching.flush_period*1000));
                    }
                    else
                    {
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks,
                                    sns_convert_ns_to_ticks(decoded_request.batching.batch_period*1000));
                    }
                }
                else
                {
                    report_rate = *chosen_sample_rate;
                    *chosen_flush_period_ticks = UINT64_MAX;
                    is_max_batch = false;
                    is_flush_only = false;
                }


                if(decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period > 0)
                {
                    report_rate = (1000000.0f / (float)decoded_request.batching.batch_period);
                }
                else
                {
                    report_rate = decoded_payload.sample_rate;
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                        report_rate);

                if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
                {
                    *non_gated_sensor_client_present = true;
                }
                else
                {
                    if (gated_sensor_client_present)
                    {
                        *gated_sensor_client_present = true;
                    }
                }
            }
        }
    }

    /** If there is a client for the sensor and
     *  if max_batch or flush_only are set in all requests then choose the largest WM. */
    if (is_max_batch)
    {
        if(*non_gated_sensor_client_present ||
                (gated_sensor_client_present && *gated_sensor_client_present))
        {
            *chosen_report_rate = (1.0f / (float)UINT32_MAX);
            BMI160_INST_LOG(LOW, instance, "max_batch: %d", sensor_type);
        }
    }
#else
    UNUSED_VAR(this);
    UNUSED_VAR(chosen_flush_period_ticks);
    sns_sensor_uid          suid;
    sns_request const       *request;

    if (BMI160_ACCEL == sensor_type)
    {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ACCEL_SUID), sizeof(sns_sensor_uid));
    }
    else if (BMI160_GYRO == sensor_type)
    {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)GYRO_SUID), sizeof(sns_sensor_uid));
    }
#if BMI160_CONFIG_ENABLE_MAG_IF //TODOMAG
    else if (BMI160_MAG == sensor_type)
    {
    }
#endif
    else
    {
    }

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *non_gated_sensor_client_present = false;
    if (gated_sensor_client_present)
    {
        *gated_sensor_client_present = false;
    }

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for(request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false))
    {
        sns_std_request         decoded_request;
        sns_std_sensor_config   decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
                ||
                request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
            if (bmi160_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
            {
                float report_rate;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                        decoded_payload.sample_rate);
                if (decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period > 0)
                {
                    report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                }
                else
                {
                    report_rate = *chosen_sample_rate;
                }
                *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                        report_rate);

                if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
                {
                    *non_gated_sensor_client_present = true;
                }
                else
                {
                    if (gated_sensor_client_present)
                    {
                        *gated_sensor_client_present = true;
                    }
                }
            }
        }
    }
#endif
}

/**
 * Get motion detection configuration
 *
 * @param this                  the sensor handler
 * @param instance              the valid instance handler
 * @param chosen_md_enable      MD enable flag
 * @param md_client_present     MD client present flag
 *
 * @return none
 */
    static
void bmi160_get_motion_detect_config(
        sns_sensor              *this,
        sns_sensor_instance     *instance,
        bool                    *chosen_md_enable,
        bool                    *md_client_present)
{
    UNUSED_VAR(this);
    sns_sensor_uid              suid = MOTION_DETECT_SUID;
    bmi160_instance_state       *istate = (bmi160_instance_state*)instance->state->state;
    bmi160_state                *sstate = (bmi160_state*)this->state->state;
    sns_request const           *request;

    UNUSED_VAR(sstate);
    *chosen_md_enable = false;
    *md_client_present = false; //TOCHECK


    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false))
    {
        if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == request->message_id)
        {
            // Enable MD interrupt when:
            // 1. There is a new request and MD is in MDF/MDD state OR
            // 2. There is an existing request and MD is in MDE/MDD state
            // Introduced for power measurement testing,
            // Disable md interrupt using registry setting and send Disable event to md client,
            // if disable flag is true and client is present
#if BMI160_CONFIG_ENABLE_MD_TEST
            istate->md_info.client_present = true;
#endif

            BMI160_SENSOR_LOG(LOW, this, "MD sstate p:%d n:%d t:%d",
                    (int)istate->md_info.client_present, (int)istate->md_info.md_new_req, (int)istate->md_info.md_state.motion_detect_event_type);

            if(!istate->md_info.md_config.disable)
            {
#if 0
                // seems this is always true
                *chosen_md_enable = ((istate->md_info.md_new_req &&
                            (istate->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_FIRED
                             || istate->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_DISABLED))
                        ||
                        (istate->md_info.client_present &&
                         (istate->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED
                          || istate->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_DISABLED)));
#endif
                //FOLLOWED_QC_EXAMPLE
                *chosen_md_enable = true;
            }

            //*chosen_md_enable = true;
            *md_client_present = true;
            // Consumed new request
            istate->md_info.md_new_req = false;

            return;
        }
    }
}

#if BMI160_CONFIG_ENABLE_PEDO
/**
 *
 * @param this                     sensor handler
 * @param instance                 instance handler
 * @param st_client_present        step counter client present
 * @return none
*/
static
void bmi160_get_pedo_config(sns_sensor *this, sns_sensor_instance *instance,
                bool *st_client_present)
{
    sns_sensor_uid suid = STEP_COUNTER_SUID;
    sns_request const *request;
    bmi160_instance_state *istate =
                    (bmi160_instance_state*) instance->state->state;

    UNUSED_VAR(this);
    *st_client_present = false; //TOCHECK

    for (request = instance->cb->get_client_request(instance, &suid, true);
                    NULL != request;
                    request = instance->cb->get_client_request(instance,
                                                               &suid,
                                                               false)) {

        BMI160_SENSOR_LOG(MED, this, "req msg:%d", request->message_id);

        if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == request->message_id
                        || SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
                                        == request->message_id) {
            *st_client_present = true;
        }
    }
    UNUSED_VAR(istate);
}

#endif


/**
 * get sensor temperature configuration
 * @param this                 sensor handler
 * @param instance             instance handler
 * @param chosen_sample_rate   chosen sample rate in hz
 * @param chosen_report_rate   chosen report rate in hz
 * @param sensor_temp_client_present  sensor temperature client present flag
 *
 * @return none
 */
static void bmi160_get_sensor_temp_config(sns_sensor *this,
        sns_sensor_instance *instance,
        float *chosen_sample_rate,
        float *chosen_report_rate,
        sns_time *chosen_flush_period_ticks,
        bool *sensor_temp_client_present)
{
#if  BMI160_CONFIG_ENABLE_FLUSH_PERIOD
    UNUSED_VAR(this);
    bmi160_instance_state *istate =
        (bmi160_instance_state*)instance->state->state;
    sns_sensor_uid suid = SENSOR_TEMPERATURE_SUID;
    sns_request const *request;
    bool is_max_batch = true;
    bool is_flush_only = true;

    *chosen_report_rate = 0.0f;
    *chosen_sample_rate = 0.0f;
    *chosen_flush_period_ticks = 0;
    *sensor_temp_client_present = false;

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false))
    {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
            if (bmi160_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
            {
                float report_rate = 0.0f;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                        decoded_payload.sample_rate);

                if (decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period >0)
                {
                    report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                }
                else
                {
                    report_rate = decoded_payload.sample_rate;
                }

                if(decoded_request.has_batching)
                {
                    is_max_batch &= (decoded_request.batching.has_max_batch && decoded_request.batching.max_batch);
                    is_flush_only &= (decoded_request.batching.flush_only);

                    if(decoded_request.batching.has_flush_period)
                    {
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks,
                                    sns_convert_ns_to_ticks(decoded_request.batching.flush_period * 1000));
                    }
                    else
                    {
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks,
                                    sns_convert_ns_to_ticks(decoded_request.batching.batch_period * 1000));
                    }
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
                *sensor_temp_client_present = true;
            }
        }
    }

    if(*sensor_temp_client_present &&
            (is_max_batch))
    {
        *chosen_report_rate = (1.0f / (float)UINT32_MAX);
    }

    istate->sensor_temp_info.report_rate_req  = *chosen_report_rate;
    istate->sensor_temp_info.sample_rate_req = *chosen_sample_rate;
#else
    UNUSED_VAR(this);
    bmi160_instance_state *istate =
        (bmi160_instance_state*)instance->state->state;
    sns_sensor_uid suid = SENSOR_TEMPERATURE_SUID;
    sns_request const *request;

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *sensor_temp_client_present = false;

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false))
    {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
            if (bmi160_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
            {
                float report_rate;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                        decoded_payload.sample_rate);

                bool rc = sns_sensor_util_decide_max_batch(instance, &suid);
                //There is request with max batch not set .
                // do normal calculation

                if (!rc) {
                    if (decoded_request.has_batching
                            &&
                            decoded_request.batching.batch_period> 0)
                    {
                        report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                    }
                    else
                    {
                        report_rate = *chosen_sample_rate;
                    }
                } else {
                    report_rate = (1000000.0 / (float)UINT32_MAX);
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
                *sensor_temp_client_present = true;
            }
        }
    }
    istate->sensor_temp_info.report_rate_req  = *chosen_report_rate;
    istate->sensor_temp_info.sample_rate_req = *chosen_sample_rate;
#endif
}




#if BMI160_CONFIG_POWER_RAIL
/**
 * Turn off power rails
 * @param this   sensor handler
 */
void bmi160_sensor_turn_rails_off(sns_sensor *this)
{
    sns_sensor *sensor;

    for(sensor = this->cb->get_library_sensor(this, true);
            NULL != sensor;
            sensor = this->cb->get_library_sensor(this, false))
    {
        bmi160_state *sensor_state = (bmi160_state*)sensor->state->state;
        if (sensor_state->common.rail_config.rail_vote != SNS_RAIL_OFF)
        {
            sensor_state->common.rail_config.rail_vote = SNS_RAIL_OFF;
            sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
                                                                            sensor,
                                                                            &sensor_state->common.rail_config,
                                                                            NULL);
        }
    }

}
#endif

bool bmi160_check_n_approve_md_req(
        bmi160_state            *sstate,
        bmi160_instance_state   *istate)
{
    bool approve_req = true;
    UNUSED_VAR(sstate);

    BMI160_SENSOR_LOG(LOW, sstate->owner, "get MD req");
    if (istate->fifo_info.publish_sensors & BMI160_ACCEL)
    {
        //send event as MD disabled since non-gated client is active
        //no need of this as we already set md_info state
        sns_motion_detect_event md_state;
        md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
        pb_send_event(istate->owner,
                sns_motion_detect_event_fields,
                &md_state,
                bmi160_get_sys_tick(),
                SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                &istate->md_info.sstate->my_suid);
        //as per requirement: reject the request directly
        BMI160_SENSOR_LOG(HIGH, sstate->owner, "MD request rejected");
        approve_req = false;
    }
    //FOLLOWED_QC_EXAMPLE
    else if (istate->int_en_flags_req.bits.md)
    {
        approve_req = false;
        //there is existing md client already present, just send event
        pb_send_event(istate->owner,
                sns_motion_detect_event_fields,
                &istate->md_info.md_state,
                bmi160_get_sys_tick(),
                SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                &istate->md_info.sstate->my_suid);
        BMI160_SENSOR_LOG(HIGH, sstate->owner, "there is already existing md client");
    }
    else
    {
        istate->md_info.md_new_req = true;
    }

    return approve_req;
}


/**
 * Decodes a physical sensor self test request and updates
 * instance state with request info.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 * @param[i] new_request Encoded request
 *
 * @return True if request is valid else false
 */
bool bmi160_extract_self_test_info(
        sns_sensor          *this,
        sns_sensor_instance *instance,
        struct sns_request const *new_request)
{
    sns_std_request         decoded_request;
    sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
    bmi160_state            *sstate = (bmi160_state*)this->state->state;
    bmi160_instance_state   *istate = (bmi160_instance_state*)instance->state->state;
    bmi160_self_test_info   *self_test_info;

    if (sstate->sensor == BMI160_ACCEL)
    {
        self_test_info = &istate->accel_info.test_info;
    }
    else if (sstate->sensor == BMI160_GYRO)
    {
        self_test_info = &istate->gyro_info.test_info;
    }
    else if (sstate->sensor == BMI160_MOTION_DETECT)
    {
        self_test_info = &istate->md_info.test_info;
    }
    else if (sstate->sensor == BMI160_SENSOR_TEMP)
    {
        self_test_info = &istate->sensor_temp_info.test_info;
    }
#if BMI160_CONFIG_ENABLE_PEDO
    else if (sstate->sensor == BMI160_PEDO)
    {
        self_test_info = &istate->pedo_info.test_info;
    }
#endif
    else
    {
        return false;
    }

    self_test_info->sensor = sstate->sensor;

    if (bmi160_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config))
    {
        self_test_info->test_type = test_config.test_type;
        self_test_info->test_client_present = true;

        BMI160_SENSOR_LOG(MED, this, "new client_request to do test: %d %d", sstate->sensor, self_test_info->test_type);
        return true;
    }
    else
    {
        return false;
    }
}

#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
bool bmi160_extract_self_test_info_bias(sns_sensor *this,
        sns_sensor_instance *instance,
        struct sns_request const *new_request)
{
    sns_std_request decoded_request;
    sns_physical_sensor_oem_config test_config = sns_physical_sensor_oem_config_init_default;
    bmi160_state *state = (bmi160_state*)this->state->state;
    bmi160_instance_state *inst_state = (bmi160_instance_state*)instance->state->state;
    bmi160_self_test_info_bias *self_test_info_bias;

    if(state->sensor == BMI160_ACCEL)
    {
        self_test_info_bias = &inst_state->accel_info.test_info_bias;
    }
    else if(state->sensor == BMI160_GYRO)
    {
        self_test_info_bias = &inst_state->gyro_info.test_info_bias;
    }
    else
    {
        return false;
    }

    if(bmi160_get_decoded_self_test_request_bias(this, new_request, &decoded_request, &test_config))
    {
        self_test_info_bias->config_type = test_config.config_type;
        self_test_info_bias->test_client_present = true;

        self_test_info_bias->offset_x = test_config.offset_x;
        self_test_info_bias->offset_y = test_config.offset_y;
        self_test_info_bias->offset_z = test_config.offset_z;

        BMI160_SENSOR_LOG(HIGH, this, "bmi160 : bmi16_extract_self_test_info_bias [config_type:%d][x:%d][y:%d][z:%d]",
                test_config.config_type,
                (int)(test_config.offset_x * 10),
                (int)(test_config.offset_y * 10),
                (int)(test_config.offset_z * 10));
        return true;
    }
    else
    {
        return false;
    }
}
#endif


/** Functions shared by all BMI160 Sensors */
/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 * @param[i] sensor_type   Sensor type
 *
 * @return none
 *
 */
void bmi160_reval_instance_config(
        sns_sensor              *this,
        sns_sensor_instance     *instance,
        bmi160_sensor_type      sensor_type)
{
    /**
     * 1. Get best non-gated and gated Accel Config.
     * 2. Get best Gyro Config.
     * 3. Get Motion Detect Config.
     * 4. Get best Sensor Temperature Config.
     * 5. Decide best Instance Config based on above outputs.
     */
    UNUSED_VAR(sensor_type);
    bmi160_state                *sstate = (bmi160_state*)this->state->state;
    bmi160_instance_state       *istate = (bmi160_instance_state*)instance->state->state;
    float                       sample_rate = 0;
    float                       report_rate = 0;
    bool                        a_sensor_client_present_non_gated = false;
    //only accel sensor is gated
    bool                        a_sensor_client_present_gated = false;
    bool                        g_sensor_client_present = false;
    bool                        md_sensor_client_present = false;
    bool                        sensor_temp_client_present = false;
    bool                        enable_md_int = false;
    sns_time                    acc_flush_period_ticks     = 0;
    sns_time                    gyro_flush_period_ticks    = 0;
    sns_time                    temp_flush_period_ticks    = 0;

    BMI160_SENSOR_LOG(LOW, this, "reval instance entrance from sensor:%d", sstate->sensor);

    bmi160_req_payload          req_payload = {.sensor_type = sensor_type};

    UNUSED_VAR(sstate);


    if (!istate->fac_test_in_progress) {
    } else {
        BMI160_SENSOR_LOG(MED, this, "NOTICE!!! inst_cfg postponed due to fac_test_in_progress: %d", istate->fac_test_info.fac_test_sensor);
        return;
    }

    bmi160_get_imu_config(this,
            instance,
            BMI160_ACCEL,
            &sample_rate,
            &report_rate,
            &acc_flush_period_ticks,
            &a_sensor_client_present_non_gated,
            &a_sensor_client_present_gated);

    istate->accel_info.sample_rate_req = sample_rate;
    istate->accel_info.report_rate_req = report_rate;
    istate->accel_info.gated_client_present = a_sensor_client_present_gated;
    istate->accel_info.client_present = a_sensor_client_present_non_gated;
    istate->accel_info.flush_period_ticks = acc_flush_period_ticks;

    BMI160_SENSOR_LOG(MED, this, "cfg_acc cp:%d, rates sr:%d, rr:%d fp:%u",
            ((a_sensor_client_present_gated<<1) | (a_sensor_client_present_non_gated&1)),
            (int)(sample_rate * 1000), (int)(report_rate * 1000),
            (uint32_t)acc_flush_period_ticks);

    if (a_sensor_client_present_gated && a_sensor_client_present_non_gated) {
        BMI160_SENSOR_LOG(MED, this, "NOTICE both gated and non-gated clients are present");
    }


    bmi160_get_imu_config(this,
            instance,
            BMI160_GYRO,
            &sample_rate,
            &report_rate,
            &gyro_flush_period_ticks,
            &g_sensor_client_present,
            NULL);

    istate->gyro_info.sample_rate_req = sample_rate;
    istate->gyro_info.report_rate_req = report_rate;
    istate->gyro_info.client_present = g_sensor_client_present;
    istate->gyro_info.flush_period_ticks = gyro_flush_period_ticks;

    BMI160_SENSOR_LOG(MED, this, "cfg_gyro cp:%d, rates sr:%d, rr:%d fp:%u",
            g_sensor_client_present,
            (int)(sample_rate * 1000), (int)(report_rate * 1000),
            (uint32_t)gyro_flush_period_ticks);


#if BMI160_CONFIG_ENABLE_MAG_IF //TODOMAG
#endif

    bmi160_get_motion_detect_config(this,
            instance,
            &enable_md_int,
            &md_sensor_client_present);

    istate->int_en_flags_req.bits.md = enable_md_int;
    istate->md_info.client_present = md_sensor_client_present;

    BMI160_SENSOR_LOG(MED, this, "cfg_md cp:%d, en:%d",
            istate->md_info.client_present,
            istate->int_en_flags_req.bits.md);

    if (a_sensor_client_present_non_gated)
    {
        istate->int_en_flags_req.bits.md = false;
    }

#if BMI160_CONFIG_ENABLE_PEDO
    bool                        pedo_client_present = false;

    bmi160_get_pedo_config (this, instance, &pedo_client_present);

    // pedo meter has a fixed macro definition sample rate
    istate->int_en_flags_req.bits.pedo = pedo_client_present;
    istate->pedo_info.client_present = pedo_client_present;
    istate->pedo_info.enable_pedo_int = pedo_client_present;

    BMI160_SENSOR_LOG(LOW, this, "cfg_pedo cp:%d, en:%d",
            istate->pedo_info.client_present,
            istate->int_en_flags_req.bits.pedo);
#endif

    bmi160_get_sensor_temp_config(this,
            instance,
            &sample_rate,
            &report_rate,
            &temp_flush_period_ticks,
            &sensor_temp_client_present);

    istate->sensor_temp_info.client_present = sensor_temp_client_present;
    //istate->sensor_temp_info.max_requested_flush_ticks = temp_flush_period_ticks;

    BMI160_SENSOR_LOG(MED, this, "cfg_temp cp:%d, rates sr:%d, rr:%d fp:%u",
            sensor_temp_client_present,
            (int)(sample_rate * 1000), (int)(report_rate * 1000),
            (uint32_t) temp_flush_period_ticks);

    /** Start with a clean sstate */
    istate->hw_mod_needed = 0;
    istate->fifo_info.publish_sensors = 0;

    if (a_sensor_client_present_non_gated)
    {
        istate->hw_mod_needed |= BMI160_ACCEL;
        istate->fifo_info.publish_sensors |= BMI160_ACCEL;
    }

    if (a_sensor_client_present_gated)
    {
        istate->hw_mod_needed |= BMI160_ACCEL;
        if (!istate->int_en_flags_req.bits.md)
        {
            istate->fifo_info.publish_sensors |= BMI160_ACCEL;
        }
    }

    if (g_sensor_client_present)
    {
        istate->hw_mod_needed |= (BMI160_ACCEL | BMI160_GYRO); //TOCHECK
        istate->fifo_info.publish_sensors |= BMI160_GYRO;
    }

#if BMI160_CONFIG_ENABLE_MAG_IF //TODOMAG
#endif

    if (md_sensor_client_present)
    {
        istate->hw_mod_needed |= BMI160_ACCEL;
    }

#if BMI160_CONFIG_ENABLE_PEDO
    if (pedo_client_present)
    {
      istate->hw_mod_needed |= BMI160_ACCEL;
      istate->fifo_info.publish_sensors |= BMI160_PEDO;
    }
#endif

    if (sensor_temp_client_present)
    {
        istate->hw_mod_needed |= BMI160_ACCEL;
        istate->fifo_info.publish_sensors |= BMI160_SENSOR_TEMP;
    }

    BMI160_SENSOR_LOG(MED, this, "inst_cfg summary ps:0x%x, hmn:0x%x",
            istate->fifo_info.publish_sensors,
            istate->hw_mod_needed);

    bmi160_set_inst_config(this,
            instance,
            req_payload,
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
}


// <registry>
#if BMI160_CONFIG_ENABLE_REGISTRY

static const char ss_name[] = SENSOR_NAME;
static const char ss_vendor[] = VENDOR_NAME;


/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void bmi160_publish_default_registry_attributes(sns_sensor *const this)
{
    bmi160_state *sstate = (bmi160_state*)this->state->state;
#if BMI160_CONFIG_ENABLE_SEE_LITE
    UNUSED_VAR(this);
#else

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg)
                { .buf = ss_name, .buf_len = sizeof(ss_name) });
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg)
                { .buf = ss_vendor, .buf_len = sizeof(ss_vendor) });
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI160_SEE_DD_ATTRIB_VERSION;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI160_FF_MAX_FRAMES_IMU; // samples
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        static char const op_mode1[] = BMI160_LPM;
        static char const op_mode2[] = BMI160_NORMAL;

        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg)
                { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
        values[1].str.funcs.encode = pb_encode_string_cb;
        values[1].str.arg = &((pb_buffer_arg)
                { .buf = op_mode2, .buf_len = sizeof(op_mode2) });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
                values, ARR_SIZE(values), false);
    }

    {
        float data[3] = {0};
        sstate->encoded_event_len =
            pb_get_encoded_size_sensor_stream_event(data, 3);
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->encoded_event_len;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
    }

    {
        const sns_std_sensor_rigid_body_type rigid_body = SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = rigid_body;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
            SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
        for(uint8_t i = 0; i < 12; i ++)
        {
            values[i].has_flt = true;
            values[i].flt = 0;
        }
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = 0;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
    }
    //@@@@@@
#endif
}



/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void bmi160_publish_registry_attributes(sns_sensor *const this)
{
#if BMI160_CONFIG_ENABLE_SEE_LITE
    UNUSED_VAR(this);
#else
    bmi160_state *sstate = (bmi160_state*)this->state->state;
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = sstate->is_dri;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = sstate->supports_sync_stream;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->hardware_id;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
            SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
        for(uint8_t i = 0; i < 12; i ++)
        {
            values[i].has_flt = true;
            values[i].flt = sstate->common.placement[i];
        }
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
                values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->common.registry_pf_cfg.rigid_body_type;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
    }

    if(sstate->sensor == BMI160_ACCEL ||
            sstate->sensor == BMI160_GYRO)
    {
        /** Only accel and gyro use registry information for selected resolution. */
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_flt = true;
        value.flt = (sstate->sensor == BMI160_ACCEL) ?
            bmi160_accel_resolutions[sstate->resolution_idx] :
            bmi160_gyro_resolutions[sstate->resolution_idx];
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);
    }

    /** Only accel and gyro use registry information for selected range. */
    if (sstate->sensor == BMI160_ACCEL ||
            sstate->sensor == BMI160_GYRO)
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        sns_std_attr_value_data rangeMinMax[] = {SNS_ATTR, SNS_ATTR};
        rangeMinMax[0].has_flt = true;
        rangeMinMax[0].flt = (sstate->sensor == BMI160_ACCEL) ?
            bmi160_accel_ranges[sstate->resolution_idx].min :
            bmi160_gyro_ranges[sstate->resolution_idx].min;
        rangeMinMax[1].has_flt = true;
        rangeMinMax[1].flt = (sstate->sensor == BMI160_ACCEL) ?
            bmi160_accel_ranges[sstate->resolution_idx].max :
            bmi160_gyro_ranges[sstate->resolution_idx].max;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg =
            &((pb_buffer_arg){ .buf = rangeMinMax, .buf_len = ARR_SIZE(rangeMinMax) });
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0], ARR_SIZE(values), true);
    }
#endif
}

void bmi160_sensor_check_registry_col_progress(sns_sensor *const this)

{
    bmi160_state *sstate = (bmi160_state*)this->state->state;

    if (sstate->common.hw_is_present) {
        bool registry_collection_done;

        if (sstate->registry_cfg_received && !sstate->registry_attr_published) {
            sstate->registry_attr_published = 1;
            bmi160_publish_registry_attributes(this);
            BMI160_SENSOR_LOG(LOW, this, "registry attr published 4 ss:%d", sstate->sensor);
        }

        registry_collection_done =
            sstate->registry_cfg_received &&
            sstate->common.registry_pf_cfg_received &&
            sstate->common.registry_orient_received && sstate->common.registry_placement_received &&
            sstate->registry_md_config_received;

        /** More clean up. */
        switch (sstate->sensor) {
            case BMI160_ACCEL:
            case BMI160_GYRO:
                //TODOMAG
                //case BMI160_SENSOR_TEMP:
                registry_collection_done = registry_collection_done && sstate->registry_fac_cal_received;
                break;
            default:
                break;
        }

        if (registry_collection_done) {
            //sns_sensor_util_remove_sensor_stream(this, &sstate->reg_data_stream);
            BMI160_SENSOR_LOG(LOW, this, "registry_collection_done: %d", sstate->sensor);
        }
    }
}

void bmi160_sensor_process_registry_event(sns_sensor *const this,
        sns_sensor_event *event)
{
    bool rv = true;
    bmi160_state *sstate = (bmi160_state*)this->state->state;

    pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
            event->event_len);


    BMI160_SENSOR_LOG(LOW, this, "process_registry_event snr: %d", sstate->sensor);

    if (SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
    {
        sns_registry_read_event read_event = sns_registry_read_event_init_default;
        pb_buffer_arg group_name = {0,0};
        read_event.name.arg = &group_name;
        read_event.name.funcs.decode = pb_decode_string_cb;

        if (!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
        {
            BMI160_SENSOR_LOG(ERROR, this, "Error decoding registry event");
        }
        else
        {
            stream = pb_istream_from_buffer((void*)event->event, event->event_len);

            if (0 == strncmp((char*)group_name.buf, BMI160_REG_NN_ACCEL,
                        group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI160_REG_NN_GYRO,
                        group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI160_REG_NN_TEMP,
                        group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI160_REG_NN_MD,
                        group_name.buf_len))
            {
#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_cfg,
                            .parsed_buffer = &sstate->common.registry_cfg
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv)
                {
                    sstate->registry_cfg_received = true;
                    sstate->is_dri = sstate->common.registry_cfg.is_dri;
                    sstate->hardware_id = sstate->common.registry_cfg.hw_id;
                    sstate->resolution_idx = sstate->common.registry_cfg.res_idx;
                    sstate->supports_sync_stream = sstate->common.registry_cfg.sync_stream;

                    if (BMI160_ACCEL == sstate->sensor) {
                        if (sstate->resolution_idx > BMI160_RANGE_ACC_PM16G) {
                            sstate->resolution_idx = BMI160_RANGE_ACC_PM16G;
                        }
                    } else if (BMI160_GYRO == sstate->sensor) {
                        if (sstate->resolution_idx > BMI160_RANGE_GYR_PM2000DPS) {
                            sstate->resolution_idx = BMI160_RANGE_GYR_PM2000DPS;
                        }
                    }

                    BMI160_SENSOR_LOG(LOW, this, "sensor: %d is_dri:%d, hardware_id:%d ",
                            sstate->sensor,
                            sstate->is_dri,
                            (int)sstate->hardware_id);
                    BMI160_SENSOR_LOG(LOW, this, "sensor: %d resolution_idx:%d, supports_sync_stream:%d ",
                            sstate->sensor,
                            sstate->common.registry_cfg.res_idx,
                            sstate->supports_sync_stream);
                }
#endif
            }

#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            else if (0 == strncmp((char*)group_name.buf, BMI160_REG_NN_PLATFORM_CONFIG,
                        group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
                            .parsed_buffer = &sstate->common.registry_pf_cfg
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv)
                {
                    sstate->common.registry_pf_cfg_received = true;

                    sstate->common.com_port_info.com_config.bus_type = (sns_bus_type)sstate->common.registry_pf_cfg.bus_type;
                    sstate->common.com_port_info.com_config.bus_instance = sstate->common.registry_pf_cfg.bus_instance;
                    sstate->common.com_port_info.com_config.slave_control = sstate->common.registry_pf_cfg.slave_config;
                    sstate->common.com_port_info.com_config.min_bus_speed_KHz = sstate->common.registry_pf_cfg.min_bus_speed_khz;
                    sstate->common.com_port_info.com_config.max_bus_speed_KHz = sstate->common.registry_pf_cfg.max_bus_speed_khz;
                    sstate->common.com_port_info.com_config.reg_addr_type = (sns_reg_addr_type)sstate->common.registry_pf_cfg.reg_addr_type;
                    sstate->common.irq_config.interrupt_num = sstate->common.registry_pf_cfg.dri_irq_num;
                    sstate->common.irq_config.interrupt_pull_type = (sns_interrupt_pull_type)sstate->common.registry_pf_cfg.irq_pull_type;
                    sstate->common.irq_config.is_chip_pin = sstate->common.registry_pf_cfg.irq_is_chip_pin;
                    sstate->common.irq_config.interrupt_drive_strength = (sns_interrupt_drive_strength)sstate->common.registry_pf_cfg.irq_drive_strength;
                    sstate->common.irq_config.interrupt_trigger_type = (sns_interrupt_trigger_type)sstate->common.registry_pf_cfg.irq_trigger_type;
                    sstate->common.rail_config.num_of_rails = sstate->common.registry_pf_cfg.num_rail;
                    sstate->common.registry_rail_on_state = (sns_power_rail_state)sstate->common.registry_pf_cfg.rail_on_state;
                    sns_strlcpy(sstate->common.rail_config.rails[0].name,
                            sstate->common.registry_pf_cfg.vddio_rail,
                            sizeof(sstate->common.rail_config.rails[0].name));
                    sns_strlcpy(sstate->common.rail_config.rails[1].name,
                            sstate->common.registry_pf_cfg.vdd_rail,
                            sizeof(sstate->common.rail_config.rails[1].name));

                    BMI160_SENSOR_LOG(LOW, this, "bus_type:%d bus_instance:%d slave_control:%d",
                            sstate->common.com_port_info.com_config.bus_type,
                            sstate->common.com_port_info.com_config.bus_instance,
                            sstate->common.com_port_info.com_config.slave_control);

                    BMI160_SENSOR_LOG(LOW, this, "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                            sstate->common.com_port_info.com_config.min_bus_speed_KHz,
                            sstate->common.com_port_info.com_config.max_bus_speed_KHz,
                            sstate->common.com_port_info.com_config.reg_addr_type);

                    BMI160_SENSOR_LOG(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                            sstate->common.irq_config.interrupt_num,
                            sstate->common.irq_config.interrupt_pull_type,
                            sstate->common.irq_config.is_chip_pin);

                    BMI160_SENSOR_LOG(LOW, this, "interrupt_drive_strength:%d interrupt_trigger_type:%d"
                            " rigid body type:%d",
                            sstate->common.irq_config.interrupt_drive_strength,
                            sstate->common.irq_config.interrupt_trigger_type,
                            sstate->common.registry_pf_cfg.rigid_body_type);

                }
            }

            //OPTIM3
            else if (0 == strncmp((char*)group_name.buf, BMI160_REG_NN_PLATFORM_PLACEMENT,
                        group_name.buf_len))
            {
                {
                    uint8_t arr_index = 0;
                    pb_float_arr_arg arr_arg = {
                        .arr = sstate->common.placement,
                        .arr_index = &arr_index,
                        .arr_len = 12
                    };

                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "placement",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &arr_arg
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv)
                {
                    sstate->common.registry_placement_received = true;
                }
            }

#endif
            //OPTIM3
            else if (0 == strncmp((char*)group_name.buf, BMI160_REG_NN_PLATFORM_ORIENT,
                        group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "orient",
                            .parse_func = sns_registry_parse_axis_orientation,
                            .parsed_buffer = sstate->common.axis_map
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }
                sstate->gyro_factor = 10000;
                if (rv)
                {
#ifdef BMI160_GET_PARAMETER_FROM_SMEM
                    uint8_t bs_gsensor_dir;
                    struct sensor_direction gsensor_dir;
                    struct sensor_hw *acc_hw;

                    oppo_get_sensor_hw(OPPO_ACCEL, BMI160, &acc_hw);
                    if (acc_hw != NULL && acc_hw->direction != DEFAULT_CONFIG)
                    {
                        bs_gsensor_dir = acc_hw->direction;
                        if(acc_hw->feature.feature[0] != 0) {
                            sstate->gyro_factor = acc_hw->feature.feature[0];
                        }else {
                            sstate->gyro_factor = 10000;
                        }
                        //sstate->gyro_factor = 9730;
                        SNS_PRINTF(HIGH, this, "gyro_factor: %d", sstate->gyro_factor);

                        get_direction(bs_gsensor_dir, &gsensor_dir);
                        sstate->common.registry_orient_received = true;
                        sstate->common.axis_map[0].opaxis = gsensor_dir.map[0];
                        sstate->common.axis_map[0].invert = gsensor_dir.sign[0];
                        sstate->common.axis_map[1].opaxis = gsensor_dir.map[1];
                        sstate->common.axis_map[1].invert = gsensor_dir.sign[1];
                        sstate->common.axis_map[2].opaxis = gsensor_dir.map[2];
                        sstate->common.axis_map[2].invert = gsensor_dir.sign[2];
                        SNS_PRINTF(HIGH, this, "bs_gsensor_dir: %d", bs_gsensor_dir);
                    }
#endif

                    //SENSOR_PRINTF_LOW_FULL(this, "Registry read event for group %s received ", (char*)group_name.buf);

                    BMI160_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                            sstate->common.axis_map[0].ipaxis,
                            sstate->common.axis_map[0].opaxis, sstate->common.axis_map[0].invert);

                    BMI160_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                            sstate->common.axis_map[1].ipaxis, sstate->common.axis_map[1].opaxis,
                            sstate->common.axis_map[1].invert);

                    BMI160_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                            sstate->common.axis_map[2].ipaxis, sstate->common.axis_map[2].opaxis,
                            sstate->common.axis_map[2].invert);
                }
            }
            else if (0 == strncmp((char*)group_name.buf,
                        BMI160_REG_NN_PLATFORM_FAC_CAL_ACCEL,
                        group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf,
                        BMI160_REG_NN_PLATFORM_FAC_CAL_GYRO,
                        group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf,
                        BMI160_REG_NN_PLATFORM_FAC_CAL_TEMP,
                        group_name.buf_len))
            {
                uint32_t fac_cal_version = 0;
                float fac_cal_bias[TRIAXIS_NUM];
                {
                    uint8_t bias_arr_index = 0, scale_arr_index = 0;
                    pb_float_arr_arg bias_arr_arg = {
                        .arr = fac_cal_bias,
                        .arr_index = &bias_arr_index,
                        .arr_len = TRIAXIS_NUM
                    };

                    pb_float_arr_arg scale_arr_arg = {
                        .arr = sstate->fac_cal_scale,
                        .arr_index = &scale_arr_index,
                        .arr_len = TRIAXIS_NUM
                    };

                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 3,
                        .parse_info[0] = {
                            .group_name = "bias",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &bias_arr_arg
                        },
                        .parse_info[1] = {
                            .group_name = "scale",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &scale_arr_arg
                        },
                        .parse_info[2] = {
                            .group_name = "corr_mat",
                            .parse_func = sns_registry_parse_corr_matrix_3,
                            .parsed_buffer = &sstate->fac_cal_corr_mat
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

                    fac_cal_version = arg.version;
                }

                if (rv)
                {
                    sstate->registry_fac_cal_received = true;
                    sstate->fac_cal_version = fac_cal_version;
                    BMI160_SENSOR_LOG(LOW, this, "fac_cal received: %d", sstate->sensor);

                    uint8_t i;
                    for (i = 0; i < TRIAXIS_NUM; i++) {
                        sstate->fac_cal_bias[i] = roundf(fac_cal_bias[i] * sstate->scale_factor);
                    }

                    if (sstate->fac_cal_scale[0] != 0.0)
                    {
                        sstate->fac_cal_corr_mat.e00 = sstate->fac_cal_scale[0];
                        sstate->fac_cal_corr_mat.e11 = sstate->fac_cal_scale[1];
                        sstate->fac_cal_corr_mat.e22 = sstate->fac_cal_scale[2];
                    }
                } else {
                    BMI160_SENSOR_LOG(ERROR, this, "fac_cal error: %d", sstate->sensor);
                }
            }
            else if (0 == strncmp((char*)group_name.buf,
                        BMI160_REG_NN_PLATFORM_MD_CONFIG,
                        group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = BMI160_REG_NN_PLATFORM_MD_CONFIG,
                            .parse_func = sns_registry_parse_md_cfg,
                            .parsed_buffer = &sstate->md_config
                        }
                    };

                    read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv)
                {
                    BMI160_SENSOR_LOG(LOW, this, "md_config received: %d %d %d %d",
                            sstate->sensor, (int)(sstate->md_config.thresh * 1e6), (int)(sstate->md_config.win * 1e6), sstate->md_config.disable);
                    sstate->registry_md_config_received = true;
                }
            }
            else
            {
                rv = false;
            }

            if (!rv)
            {
                BMI160_SENSOR_LOG(ERROR, this, "Error decoding registry");
            }
        }
    }
    else
    {
        BMI160_SENSOR_LOG(ERROR, this, "Received unsupported registry event msg id %u",
                event->message_id);
    }
}


/**
 * Registry request send function
 * sensor send the getting registry configuration request to the Framework
 *
 * @param this             the sensor handler
 * @param reg_group_name   the registry group name
 *
 * @return none
 */
static void bmi160_sensor_send_registry_request(sns_sensor *const this,
        char *reg_group_name)
{
    bmi160_state *sstate = (bmi160_state*)this->state->state;
    uint8_t buffer[100];
    int32_t encoded_len;
    sns_memset(buffer, 0, sizeof(buffer));
    sns_rc rc = SNS_RC_SUCCESS;

    sns_registry_read_req read_request;
    pb_buffer_arg data = (pb_buffer_arg){ .buf = reg_group_name,
        .buf_len = (strlen(reg_group_name) + 1) };

    read_request.name.arg = &data;
    read_request.name.funcs.encode = pb_encode_string_cb;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &read_request, sns_registry_read_req_fields, NULL);
    if (0 < encoded_len)
    {
        sns_request request = (sns_request) {
            .request_len = encoded_len,
                .request = buffer,
                .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ
        };
        rc = sstate->reg_data_stream->api->send_request(sstate->reg_data_stream, &request);

        BMI160_SENSOR_LOG(LOW, this, "req_registry sent %d %d", sstate->sensor, rc);
    }

    //SENSOR_PRINTF_LOW_FULL(this, "Sending registry request for group name:%s", reg_group_name);
}


void bmi160_request_registry(sns_sensor *const this)
{
    bmi160_state *sstate = (bmi160_state*)this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    BMI160_SENSOR_LOG(LOW, this, "req_registry: %d 0x%x %d", sstate->sensor, sstate->reg_data_stream, sstate->common.who_am_i);

    if ((NULL == sstate->reg_data_stream)
            && (0 == sstate->common.who_am_i))
    {
        sns_sensor_uid suid;

        sns_suid_lookup_get(&sstate->common.suid_lookup_data, "registry", &suid);


        stream_svc->api->create_sensor_stream(stream_svc,
                this, suid, &sstate->reg_data_stream);

#if BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        sns_bmi160_registry_def_config(this);
#endif

#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        bmi160_sensor_send_registry_request(this, BMI160_REG_NN_PLATFORM_CONFIG);
        bmi160_sensor_send_registry_request(this, BMI160_REG_NN_PLATFORM_PLACEMENT);
        bmi160_sensor_send_registry_request(this, BMI160_REG_NN_PLATFORM_MD_CONFIG);
#endif
        bmi160_sensor_send_registry_request(this, BMI160_REG_NN_PLATFORM_ORIENT);

        if (BMI160_ACCEL == sstate->sensor)
        {
#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            bmi160_sensor_send_registry_request(this, BMI160_REG_NN_ACCEL);
#endif
            bmi160_sensor_send_registry_request(
                    this, BMI160_REG_NN_PLATFORM_FAC_CAL_ACCEL);
        }
        else if (BMI160_GYRO == sstate->sensor)
        {
#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            bmi160_sensor_send_registry_request(this, BMI160_REG_NN_GYRO);
#endif
            bmi160_sensor_send_registry_request(
                    this, BMI160_REG_NN_PLATFORM_FAC_CAL_GYRO);
        }
#if !BMI160_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        else if (BMI160_SENSOR_TEMP == sstate->sensor)
        {
            bmi160_sensor_send_registry_request(
                    this, BMI160_REG_NN_TEMP);
            //todo: BMI160_REG_NN_PLATFORM_FAC_CAL_TEMP
        }
        else if (BMI160_MOTION_DETECT == sstate->sensor)
        {
            bmi160_sensor_send_registry_request(
                    this, BMI160_REG_NN_MD);
        }
#endif
    }
}


static bool
bmi160_encode_registry_group_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
        void *const *arg)
{
    pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
    bmi160_instance_state *state =
        (bmi160_instance_state*)pb_arg->instance->state->state;

    if (0 == strncmp(pb_arg->name,"bias",strlen("bias")))
    {
        char const *names[] = {"x", "y", "z"};

        for (int i = 0; i < ARR_SIZE(names); i++)
        {
            pb_buffer_arg name_data = (pb_buffer_arg)
            { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;

            pb_item.name.funcs.encode = &pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;

            if (pb_arg->sensor == BMI160_ACCEL)
            {
                pb_item.flt = state->accel_info.sstate->fac_cal_bias[i] / BMI160_SCALE_FACTOR_DATA_ACCEL;
                pb_item.version = state->accel_info.sstate->fac_cal_version;
            }
            else
            {
                pb_item.flt = state->gyro_info.sstate->fac_cal_bias[i] / BMI160_SCALE_FACTOR_DATA_DFT;
                pb_item.version = state->gyro_info.sstate->fac_cal_version;
            }

            if (!pb_encode_tag_for_field(stream, field))
                return false;

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
            {
                BMI160_INST_LOG(ERROR, pb_arg->instance, "Error encoding sns_registry_data_item_fields");
                return false;
            }
        }
    }
    else if (0 == strncmp(pb_arg->name,"corr_mat",strlen("corr_mat")))
    {
        char const *names[] = {"0_0", "0_1", "0_2",
            "1_0", "1_1", "1_2",
            "2_0", "2_1", "2_2",};

        for (int i = 0; i < ARR_SIZE(names); i++)
        {
            pb_buffer_arg name_data = (pb_buffer_arg)
            { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;

            pb_item.name.funcs.encode = &pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;
            if (pb_arg->sensor == BMI160_ACCEL)
            {
                pb_item.flt = state->accel_info.sstate->fac_cal_corr_mat.data[i];
                pb_item.version = state->accel_info.sstate->fac_cal_version;
            }
            else
            {
                pb_item.flt = state->gyro_info.sstate->fac_cal_corr_mat.data[i];
                pb_item.version = state->gyro_info.sstate->fac_cal_version;
            }

            if (!pb_encode_tag_for_field(stream, field))
                return false;

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
            {
                BMI160_INST_LOG(ERROR, pb_arg->instance, "Error encoding sns_registry_data_item_fields");
                return false;
            }
        }
    }
    return true;
}

static bool
bmi160_encode_registry_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
        void *const *arg)
{
    pb_arg_reg_group_arg *reg_arg = (pb_arg_reg_group_arg*)*arg;
    sns_sensor_instance *instance = reg_arg->instance;
    char const *names[] = {"bias", "corr_mat"};

    for(int i = 0; i < ARR_SIZE(names); i++)
    {
        pb_buffer_arg name_data = (pb_buffer_arg)
        { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
        sns_registry_data_item pb_item = sns_registry_data_item_init_default;
        pb_arg_reg_group_arg pb_arg= (pb_arg_reg_group_arg){
            .name = NULL,.instance = instance, .sensor = reg_arg->sensor
        };

        pb_item.has_version = true;
        pb_item.version = reg_arg->version;
        pb_item.name.arg = &name_data;
        pb_item.name.funcs.encode = &pb_encode_string_cb;

        if (0==strncmp(names[i],"bias",name_data.buf_len))
        {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;
            pb_item.subgroup.items.funcs.encode = &bmi160_encode_registry_group_cb;
            pb_item.subgroup.items.arg = &pb_arg;

        }
        else if (0==strncmp(names[i],"corr_mat",name_data.buf_len))
        {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;
            pb_item.subgroup.items.funcs.encode = &bmi160_encode_registry_group_cb;
            pb_item.subgroup.items.arg = &pb_arg;
        }
        if (!pb_encode_tag_for_field(stream, field))
        {
            BMI160_INST_LOG(ERROR, instance,"Error encoding corr_mat");

            return false;
        }

        if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
        {
            BMI160_INST_LOG(ERROR, instance,"Error encoding sns_registry_data_item_fields");
            return false;
        }
    }

    return true;
}



void bmi160_update_registry(sns_sensor *const this,
        sns_sensor_instance *const instance, bmi160_sensor_type sensor)
{
    bmi160_state *state = (bmi160_state*)this->state->state;
    pb_arg_reg_group_arg arg = {.instance = instance };

    uint8_t buffer[350];
    int32_t encoded_len;
    char accel_name[] = BMI160_REG_NN_PLATFORM_FAC_CAL_ACCEL;
    char gyro_name[] = BMI160_REG_NN_PLATFORM_FAC_CAL_GYRO;

    bmi160_instance_state *istate = (bmi160_instance_state *)instance->state->state;
    pb_buffer_arg name_data;
    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    if (sensor == BMI160_ACCEL)
    {
        name_data = (pb_buffer_arg)
        { .buf = accel_name, .buf_len = strlen(accel_name) + 1 };
        arg.sensor = BMI160_ACCEL;

        arg.version = istate->accel_info.sstate->fac_cal_version;
    }
    else if (sensor == BMI160_GYRO)
    {
        name_data = (pb_buffer_arg)
        { .buf = gyro_name, .buf_len = strlen(gyro_name) + 1 };
        arg.sensor = BMI160_GYRO;

        arg.version = istate->gyro_info.sstate->fac_cal_version;
    }
    else
    {
        BMI160_SENSOR_LOG(ERROR, this, "Unsupported sensor %d", sensor);
        return;
    }

    write_req.name.funcs.encode = &pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = &bmi160_encode_registry_cb;
    write_req.data.items.arg = &arg;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
            &write_req, sns_registry_write_req_fields, NULL);
    if (0 < encoded_len)
    {
        if (NULL == state->reg_data_stream)
        {
            sns_service_manager *smgr = this->cb->get_service_manager(this);
            sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

            sns_sensor_uid suid;
            sns_suid_lookup_get(&state->common.suid_lookup_data, "registry", &suid);


            stream_svc->api->create_sensor_stream(stream_svc, this, suid, &state->reg_data_stream);
        }


        if (NULL != state->reg_data_stream) {
            sns_request request = (sns_request){
                .request_len = encoded_len, .request = buffer,
                    .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ };
            state->reg_data_stream->api->send_request(state->reg_data_stream, &request);

            BMI160_SENSOR_LOG(MED, this, "req_update_registry for %d len:%d version: %d", sensor, encoded_len, arg.version);
        }
    }

}
#endif


static void bmi160_collect_common_info_from_sibling_sensors(sns_sensor * const this)
{
    sns_sensor *sensor = NULL;
    bmi160_state *sstate;


    bmi160_state *sstate_this = (bmi160_state*) this->state->state;

    for (sensor = this->cb->get_library_sensor(this, true);
            sensor != NULL;
            sensor = this->cb->get_library_sensor(this, false))
    {
        sstate = (bmi160_state*) sensor->state->state;


        if (sstate->sensor != sstate_this->sensor)
        {
            sns_memscpy(&sstate->common, sizeof(sstate->common),
                    &sstate_this->common, sizeof(sstate_this->common));
        }
        BMI160_SENSOR_LOG(LOW, this, "collect comm ss: %d, hw.present:%d",
                        sstate->sensor, sstate->common.hw_is_present);
    }
}



bool bmi160_discover_hw(sns_sensor *const this)
{
    uint8_t buffer[1] = {0};
    bool hw_is_present = false;
    bmi160_state *state = (bmi160_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    state->common.init_flags |= state->sensor;

	/**-----------------Register and Open COM Port-------------------------*/
    if (NULL == state->common.com_port_info.port_handle) {
        rv = state->scp_service->api->sns_scp_register_com_port(
             &state->common.com_port_info.com_config,
             &state->common.com_port_info.port_handle);

        if ((SNS_RC_SUCCESS == rv) && (NULL != state->common.com_port_info.port_handle)) {
            rv = state->scp_service->api->sns_scp_open(state->common.com_port_info.port_handle);
        }

        if (NULL != state->common.com_port_info.port_handle) {
            if (SNS_BUS_SPI == state->common.com_port_info.com_config.bus_type) {
                rv = bmi160_hal_switch_2_spi(state->scp_service, state->common.com_port_info.port_handle);
            }
        }
    }

    bmi160_collect_common_info_from_sibling_sensors(this);

    BMI160_SENSOR_LOG(MED, this, "dhw comport handle:%p, init flags:0x%x, def.hw.present:%d",
        state->common.com_port_info.port_handle,
        state->common.init_flags,
        state->common.hw_is_present
        );

    if (state->common.hw_is_present == 0) {
        // CHECK prepare for SPI
        if (state->common.com_port_info.com_config.bus_type == SNS_BUS_SPI) {
            bmi160_hal_sensor_prepare_spi_if(this);
        }
        /**-------------------Read and Confirm WHO-AM-I------------------------*/
        buffer[0] = 0x0;
        rv = bmi160_hal_get_who_am_i(state->scp_service, state->common.com_port_info.port_handle, &buffer[0]);
        BMI160_SENSOR_LOG(MED, this, "@sensor:%d get chip id:0x%x", state->sensor, buffer[0]);
        if (rv == SNS_RC_SUCCESS
            &&
            (BMI160_REGV_CHIP_ID_MAJOR == (buffer[0] & BMI160_REGV_CHIP_ID_MAJOR))) {
            // Reset Sensor only if an instance is not already running
            if (NULL == sns_sensor_util_get_shared_instance(this)) {
            /*            rv = bmi160_hal_reset_device(state->scp_service,state->common.com_port_info.port_handle,
                          BMI160_ACCEL | BMI160_GYRO | BMI160_MOTION_DETECT | BMI160_SENSOR_TEMP);*/
            }

            if (rv == SNS_RC_SUCCESS) {
                hw_is_present = true;
            }
        }
        state->common.who_am_i = buffer[0];
    } else {
        hw_is_present = true;
    }
#if BMI160_CONFIG_ENABLE_LOWG
    // DO this before the COM port closing
    rv = bmi160_hal_sensor_en_acc_lowg(this, true);
#endif

    /**------------------Power Down and Close COM Port--------------------*/
    state->scp_service->api->sns_scp_update_bus_power(
            state->common.com_port_info.port_handle,
            false);

    rv = state->scp_service->api->sns_scp_close(state->common.com_port_info.port_handle);
    rv = state->scp_service->api->sns_scp_deregister_com_port(&state->common.com_port_info.port_handle);

#if BMI160_CONFIG_POWER_RAIL
    /**----------------------Turn Power Rail OFF--------------------------*/
    state->common.rail_config.rail_vote = SNS_RAIL_OFF;
    state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                             this,
                                                             &state->common.rail_config,
                                                             NULL);
#endif

    // finish HW detection
    state->common.hw_is_finish_detection = 1;

    if (hw_is_present) {
	  static bool register_deinfo = false;
	  struct devinfo info_acc = {
		  .cal_path = BMI160_REG_NN_PLATFORM_FAC_CAL_ACCEL "" BMI160_FAC_CAL_BIAS,
	  };
	  struct devinfo info_gyro = {
		  .cal_path = BMI160_REG_NN_PLATFORM_FAC_CAL_GYRO "" BMI160_FAC_CAL_BIAS,
	  };
	  if (!register_deinfo) {
		  register_deinfo = true;
		  register_sensor_devinfo(OPLUS_GSENSOR,&info_acc);
		  register_sensor_devinfo(OPLUS_GYRO,&info_gyro);
		  SNS_PRINTF(HIGH, this, "bmi160 gsensor and gyro register_sensor_devinfo");
	  }
    }

    return hw_is_present;
}





void bmi160_publish_available(sns_sensor * const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}




void bmi160_start_hw_detect_sequence(sns_sensor *this)
{
    bmi160_state *sstate = (bmi160_state *) this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    //sstate->common.registry_pf_cfg_received = false; copied from QC example code

#if BMI160_CONFIG_POWER_RAIL
    /**---------------------Register Power Rails --------------------------*/
    if (sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", NULL)
            && NULL == sstate->pwr_rail_service
            && rv == SNS_RC_SUCCESS)
    {
        rv = bmi160_register_power_rail(this);

        /**---------------------Turn Power Rails ON----------------------------*/

        sstate->common.rail_config.rail_vote = sstate->common.registry_rail_on_state;

        if (rv == SNS_RC_SUCCESS) {
            sstate->pwr_rail_service->api->sns_vote_power_rail_update(sstate->pwr_rail_service,
                                                                      this,
                                                                      &sstate->common.rail_config,
                                                                      NULL);
        }

        /**-------------Create a Timer stream for Power Rail ON timeout.---------*/
        if (rv == SNS_RC_SUCCESS) {
            bmi160_start_power_rail_timer(this,
                    sns_convert_ns_to_ticks(BMI160_OFF_TO_IDLE_MS * 1000 * 1000),
                    BMI160_POWER_RAIL_PENDING_INIT);
        }
    }
#else
    sstate->common.hw_is_present = bmi160_discover_hw(this);

    if (sstate->common.hw_is_present)
    {
        BMI160_SENSOR_LOG(MED, this, "sensor:%d init finished", sstate->sensor);
        //bmi160_publish_available(this);
        bmi160_update_sibling_sensors(this);
    }
    else
    {
        #ifdef OPLUS_FEATURE_SENSOR_FB
        struct fb_event fb_event;
        memset(&fb_event, 0, sizeof(struct fb_event));
        fb_event.event_id = ACCEL_INIT_FAIL_ID;
        oplus_add_fd_event(&fb_event);
        #endif
        rv = SNS_RC_INVALID_LIBRARY_STATE;
        BMI160_SENSOR_LOG(ERROR, this, "BMI160 HW absent");
    }
#endif
}

void bmi160_update_sibling_sensors(sns_sensor * const this)
{
    sns_sensor *sensor = NULL;
    bmi160_state *sstate;


    bmi160_state *sstate_this = (bmi160_state*) this->state->state;

    for (sensor = this->cb->get_library_sensor(this, true);
            sensor != NULL;
            sensor = this->cb->get_library_sensor(this, false))
    {
        sstate = (bmi160_state*) sensor->state->state;


        if (sstate->sensor != sstate_this->sensor)
        {
            sns_memscpy(&sstate->common, sizeof(sstate->common),
                    &sstate_this->common, sizeof(sstate_this->common));
#if BMI160_CONFIG_POWER_RAIL
            bmi160_register_power_rail(sensor);
#endif
            //bmi160_publish_available(sensor);
        }

#if BMI160_CONFIG_ENABLE_REGISTRY
        bmi160_sensor_check_registry_col_progress(sensor);

        BMI160_SENSOR_LOG(LOW, this, "update_siblings: %d %d", sstate->sensor, sstate->registry_cfg_received);
#else
        if (sstate->sensor != sstate_this->sensor) {
            sns_bmi160_registry_def_config(sensor);
        }
#endif
        bmi160_publish_available(sensor);
    }
}

/**

 * Sensor common initialization functions
 * @param this   sensor handler
 */
void bmi160_common_init(sns_sensor * const this)
{
    bmi160_state *sstate = (bmi160_state*) this->state->state;
    uint8_t i;

    struct sns_service_manager *smgr = this->cb->get_service_manager(this);
    sstate->diag_service = (sns_diag_service *) smgr->get_service(smgr, SNS_DIAG_SERVICE);
    sstate->scp_service = (sns_sync_com_port_service *) smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

    sstate->sensor_client_present = false;
    sstate->owner = this;

    if ((sstate->sensor == BMI160_ACCEL) ||
            (sstate->sensor == BMI160_GYRO))
    {
        // initialize axis conversion settings
        for (i = 0; i < TRIAXIS_NUM; i ++)
        {
            sstate->common.axis_map[i].opaxis = (triaxis) i;
            sstate->common.axis_map[i].ipaxis = (triaxis) i;
            sstate->common.axis_map[i].invert = false;
        }
    }

    // initialize fac cal correction matrix to identity
    sstate->fac_cal_corr_mat.e00 = 1.0;
    sstate->fac_cal_corr_mat.e11 = 1.0;
    sstate->fac_cal_corr_mat.e22 = 1.0;

    SNS_SUID_LOOKUP_INIT(sstate->common.suid_lookup_data, NULL);
    if (BMI160_ACCEL == sstate->sensor) {
#if BMI160_CONFIG_ENABLE_DAE
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "data_acquisition_engine");
#endif
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "interrupt");
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "async_com_port");
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "timer");
    }

#if BMI160_CONFIG_ENABLE_REGISTRY
    sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "registry");
#endif
}

