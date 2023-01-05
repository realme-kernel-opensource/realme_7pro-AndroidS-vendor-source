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
 * @file sns_bmi26x_sensor_island.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "sns_bmi26x_sensor.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_motion_detect.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"

#include "sns_cal.pb.h"
#include "sns_bmi26x_config.h"

static sns_sensor_uid const* bmi26x_accel_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = ACCEL_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi26x_gyro_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = GYRO_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi26x_motion_detect_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = MOTION_DETECT_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi26x_sensor_temp_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = SENSOR_TEMPERATURE_SUID;

    return &sensor_uid;
}

#if BMI26X_CONFIG_ENABLE_LOWG
static sns_sensor_uid const* bmi26x_free_fall_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = LOWG_SENSOR_SUID;

    return &sensor_uid;
}
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
// bmi26x_double_tap_get_sensor_uid
static sns_sensor_uid const*  bmi26x_double_tap_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = DOUBLE_TAP_SENSOR_SUID;

    return &sensor_uid;
}
#endif

static void bmi26x_sensor_exit_island(sns_sensor *const this)
{
#if BMI26X_CONFIG_ENABLE_ISLAND_MODE
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_island_service  *island_svc  =
                    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
    island_svc->api->sensor_island_exit(island_svc, this);
#else
    UNUSED_VAR(this);
#endif
}


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
static bool bmi26x_get_decoded_imu_request(sns_sensor const *this, sns_request const *in_request,
        sns_std_request *decoded_request,
        sns_std_sensor_config *decoded_payload)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg = {
        .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields
    };
    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = pb_decode_simple_cb, .arg = &arg
    };

    stream = pb_istream_from_buffer(in_request->request,
                                    in_request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! decode error");
        return false;
    }
    return true;
}


#if BMI26X_CONFIG_ENABLE_PEDO
static sns_sensor_uid const* bmi26x_pedo_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = PEDO_SUID;

    return &sensor_uid;
}
#endif

/**
 * Get the advanced sensor configuration from the stream
 *
 * @param this                     sensor handler
 * @param instance                 instance handler
 * @param st_client_present        step counter client present
 * @return none
*/
static
void bmi26x_get_advanced_sensor_config(
    sns_sensor              *this,
    sns_sensor_instance     *instance,
    bmi26x_sensor_type      ss_type,
    bool                    *st_client_present)
{
    sns_sensor_uid *suid = NULL;
    sns_request const *request = NULL;


    if ((ss_type != BMI26X_PEDO) &&
            (ss_type != BMI26X_OIS)
#if  BMI26X_CONFIG_ENABLE_LOWG
            &&
            (ss_type != BMI26X_FREE_FALL)
#endif
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            && (ss_type != BMI26X_DOUBLE_TAP)
#endif
            ) {
        BMI26X_INST_LOG(ERROR, instance, "ERROR!!! unsupport ss:%d on get.ss.cfg",
                        ss_type);
        return ;
    }

#if BMI26X_CONFIG_ENABLE_PEDO
    if (ss_type == BMI26X_PEDO) {
        suid = &((sns_sensor_uid)PEDO_SUID);
    }
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    if (ss_type == BMI26X_OIS) {
        suid = &((sns_sensor_uid)OIS_SENSOR_SUID);
    }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    if (ss_type == BMI26X_FREE_FALL) {
        suid = &((sns_sensor_uid)LOWG_SENSOR_SUID);
    }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    if (ss_type == BMI26X_DOUBLE_TAP) {
        suid = &((sns_sensor_uid)DOUBLE_TAP_SENSOR_SUID);
    }
#endif


    UNUSED_VAR(this);
    *st_client_present = false; //TOCHECK

    if (BST_ASSERT_POINT(instance)) {
        return ;
    }

    for (request = instance->cb->get_client_request (instance, suid, true);
            NULL != request;
            request = instance->cb->get_client_request (instance, suid, false)) {
        BMI26X_SENSOR_LOG(LOW, this, "req msg:%d", request->message_id);

        if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == request->message_id ||
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == request->message_id) {
            *st_client_present = true;
#if BMI26X_CONFIG_ENABLE_LOWG
            bmi26x_instance_state  *istate = (bmi26x_instance_state*)instance->state->state;
            if ((ss_type == BMI26X_FREE_FALL) &&
                    istate->lowg_info.lowg_new_req) {
                istate->lowg_info.lowg_new_req = false;  //consume the req
                istate->lowg_info.lowg_wait_for_fired_event = 1;
            }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            bmi26x_instance_state  *istate = (bmi26x_instance_state*)instance->state->state;
            if ((ss_type == BMI26X_DOUBLE_TAP) &&
                    istate->dbtap_info.dbtap_new_req) {
                istate->dbtap_info.dbtap_new_req = false;  //consume the req
                istate->dbtap_info.dbtap_wait_for_fired_event = 1;
            }
#endif



        } //@if
    } //@for
}


#if BMI26X_CONFIG_ENABLE_OIS

static sns_sensor_uid const* bmi26x_ois_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = OIS_SENSOR_SUID;

    return &sensor_uid;
}

#endif


#if BMI26X_CONFIG_POWER_RAIL
sns_rc bmi26x_start_power_rail_timer(sns_sensor *const this,
                                   sns_time timeout_ticks,
                                   bmi26x_power_rail_pending_state pwr_rail_pend_state)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t req_len;
    uint8_t buffer[20];
    sns_rc rc = SNS_RC_SUCCESS;

    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout_ticks;

    if (NULL == sstate->timer_stream) {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service  *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
        sns_sensor_uid      suid;

        sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", &suid);


        rc = stream_svc->api->create_sensor_stream(stream_svc, this, suid,
                                              &sstate->timer_stream);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
    }

    req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                                sns_timer_sensor_config_fields, NULL);
    if (req_len > 0 && NULL != sstate->timer_stream) {
        sns_request timer_req = {
            .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
            .request = buffer, .request_len = req_len
        };
        rc = sstate->timer_stream->api->send_request(sstate->timer_stream, &timer_req);
        BMI26X_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        sstate->power_rail_pend_state = pwr_rail_pend_state;

        BMI26X_SENSOR_LOG(LOW, this, "power rail timer requested by: %d/%p @%u, timeout:%u",
                          sstate->sensor,
                          sstate->timer_stream,
                          (uint32_t)req_payload.start_time,
                          (uint32_t)(req_payload.timeout_period));
    } else {
        BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! timer req encode error");
    }

    return rc;
}
#endif



#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
void bmi26x_set_self_test_inst_config_bias(sns_sensor *this,
        sns_sensor_instance *instance)
{
    sns_request config;

    config.message_id = SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG;
    config.request_len = 0;
    config.request = NULL;

    this->instance_api->set_client_config(instance, &config);
}
#endif


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
static bool bmi26x_get_decoded_self_test_request(
    sns_sensor const        *this,
    sns_request const       *request,
    sns_std_request         *decoded_request,
    sns_physical_sensor_test_config *test_config)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg = {
        .decoded_struct = test_config,
        .fields = sns_physical_sensor_test_config_fields
    };
    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = pb_decode_simple_cb, .arg = &arg
    };
    stream = pb_istream_from_buffer(request->request,
                                    request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! BMI26X decode error");
        return false;
    }

    return true;
}

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
static bool bmi26x_get_decoded_self_test_request_bias(sns_sensor const *this, sns_request const *request,
        sns_std_request *decoded_request,
        sns_physical_sensor_oem_config *test_config)
{
    pb_istream_t stream;
    pb_simple_cb_arg arg = {
        .decoded_struct = test_config,
        .fields = sns_physical_sensor_oem_config_fields
    };
    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb,
        .arg = &arg
    };
    stream = pb_istream_from_buffer(request->request, request->request_len);
    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        BMI26X_SENSOR_LOG(ERROR, this, "bmi26x :decode error");

        return false;
    }
    return true;
}
#endif


/**
 * get BMI26X ACC/Gyro configurations
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
static void bmi26x_get_imu_config(sns_sensor *this,
                                  sns_sensor_instance *instance,
                                  bmi26x_sensor_type  sensor_type,
                                  float               *chosen_sample_rate,
                                  float               *chosen_report_rate,
                                  sns_time            *chosen_flush_period_ticks,
                                  bool                *non_gated_sensor_client_present,
                                  bool                *gated_sensor_client_present)
{
#if BMI26X_CONFIG_ENABLE_FLUSH_PERIOD
    UNUSED_VAR(this);
    sns_sensor_uid          *suid = NULL;
    sns_request const       *request = NULL;
    bool is_max_batch = true;

    if (BMI26X_ACCEL == sensor_type) {
        suid = &((sns_sensor_uid)ACCEL_SUID);
    } else if (BMI26X_GYRO == sensor_type) {
        suid = &((sns_sensor_uid)GYRO_SUID);
    }
#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
    else if (BMI26X_MAG == sensor_type) {
    }
#endif
    else {
    }

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *non_gated_sensor_client_present = false;
    *chosen_flush_period_ticks = 0;
    if (gated_sensor_client_present) {
        *gated_sensor_client_present = false;
    }

    if ((suid == NULL) || (BST_ASSERT_POINT(instance))) {
        return ;
    }

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, suid, false)) {
        sns_std_request         decoded_request;
        sns_std_sensor_config   decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
                ||
                request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            if (bmi26x_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload)) {
                float report_rate = 0.0f;
                bool is_flush_only = true;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate, decoded_payload.sample_rate);

                if (decoded_request.has_batching) {
                    is_max_batch = (decoded_request.batching.has_max_batch && decoded_request.batching.max_batch);
                    is_flush_only &= decoded_request.batching.flush_only;

                    if (decoded_request.batching.has_flush_period) {
                        sns_time batching_flush_period = (sns_time) ((sns_time)decoded_request.batching.flush_period * 1000);
                        batching_flush_period = sns_convert_ns_to_ticks(batching_flush_period);
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks, batching_flush_period);
                    } else {
                        sns_time batching_flush_period = (sns_time) ((sns_time)decoded_request.batching.batch_period * 1000);
                        batching_flush_period = sns_convert_ns_to_ticks(batching_flush_period);
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks, batching_flush_period);
                    }
                } else {
                    report_rate = *chosen_sample_rate;
                    *chosen_flush_period_ticks = UINT64_MAX;
                    is_max_batch = false;
                    is_flush_only = false;
                }
                BMI26X_INST_LOG(MED, instance, "is flush only:%d %d", is_flush_only);


                if (decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period > 0) {
                    report_rate = (1000000.0f / (float)decoded_request.batching.batch_period);
                } else {
                    if (is_max_batch) {
                        report_rate =  (1.0f / UINT32_MAX);
                        BMI26X_INST_LOG(LOW, instance, "max_batch: %d", sensor_type);
                    } else {
                        report_rate = decoded_payload.sample_rate;
                    }
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                              report_rate);

                if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
                    *non_gated_sensor_client_present = true;
                } else {
                    if (gated_sensor_client_present) {
                        *gated_sensor_client_present = true;
                    }
                }
            }

            BMI26X_INST_LOG(MED, instance, "decode.batch:<%d %d %d %d %d>  flush:%d %d %d",
                            decoded_request.has_batching,
                            decoded_request.batching.has_max_batch,
                            decoded_request.batching.max_batch,
                            (uint32_t) (decoded_payload.sample_rate),
                            decoded_request.batching.batch_period,
                            decoded_request.batching.flush_only,
                            decoded_request.batching.has_flush_period,
                            decoded_request.batching.flush_period);

        } else if (request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
            //find flush @here?
            BMI26X_INST_LOG(HIGH, instance, "flush in req @%d", sensor_type);
        } else if (request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
            // find physical test @here
            BMI26X_INST_LOG(HIGH, instance, "physical test in req @%d", sensor_type);
        }
    }

#if 0
    /** If there is a client for the sensor and
     *  if max_batch or flush_only are set in all requests then choose the largest WM. */
    if (is_max_batch) {
        if (*non_gated_sensor_client_present ||
                (gated_sensor_client_present && *gated_sensor_client_present)) {
            *chosen_report_rate = (1.0f / UINT32_MAX);
            BMI26X_INST_LOG(LOW, instance, "max_batch: %d", sensor_type);
        }
    }
#endif

#else
    UNUSED_VAR(this);
    UNUSED_VAR(chosen_flush_period_ticks);
    sns_sensor_uid          suid;
    sns_request const       *request;

    if (BMI26X_ACCEL == sensor_type) {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ACCEL_SUID), sizeof(sns_sensor_uid));
    } else if (BMI26X_GYRO == sensor_type) {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)GYRO_SUID), sizeof(sns_sensor_uid));
    }
#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
    else if (BMI26X_MAG == sensor_type) {
    }
#endif
    else {
    }

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *non_gated_sensor_client_present = false;
    if (gated_sensor_client_present) {
        *gated_sensor_client_present = false;
    }

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false)) {
        sns_std_request         decoded_request;
        sns_std_sensor_config   decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
                ||
                request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            if (bmi26x_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload)) {
                float report_rate;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                              decoded_payload.sample_rate);
                if (decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period > 0) {
                    report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                } else {
                    report_rate = *chosen_sample_rate;
                }
                *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                              report_rate);

                if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
                    *non_gated_sensor_client_present = true;
                } else {
                    if (gated_sensor_client_present) {
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
void bmi26x_get_motion_detect_config(
    sns_sensor              *this,
    sns_sensor_instance     *instance,
    bool                    *chosen_md_enable,
    bool                    *md_client_present)
{
    UNUSED_VAR(this);
    sns_sensor_uid              suid = MOTION_DETECT_SUID;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    bmi26x_state                *sstate = (bmi26x_state*)this->state->state;
    sns_request const           *request;

    UNUSED_VAR(sstate);
    *chosen_md_enable = false;
    *md_client_present = false; //TOCHECK


    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false)) {
        if (SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG == request->message_id) {
            // Enable MD interrupt when:
            // 1. There is a new request and MD is in MDF/MDD state OR
            // 2. There is an existing request and MD is in MDE/MDD state
            // Introduced for power measurement testing,
            // Disable md interrupt using registry setting and send Disable event to md client,
            // if disable flag is true and client is present
#if BMI26X_CONFIG_ENABLE_MD_TEST
            istate->md_info.client_present = true;
#endif

            BMI26X_SENSOR_LOG(LOW, this, "MD sstate p:%d n:%d t:%d",
                              (int)istate->md_info.client_present, (int)istate->md_info.md_new_req,
                              (int)istate->md_info.md_state.motion_detect_event_type);

            if (!istate->md_info.md_config.disable) {
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
                //FOLLOWED_QUALCOMM_EXAMPLE
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
static void bmi26x_get_sensor_temp_config(sns_sensor *this,
        sns_sensor_instance *instance,
        float *chosen_sample_rate,
        float *chosen_report_rate,
        sns_time *chosen_flush_period_ticks,
        bool *sensor_temp_client_present)
{
#if  BMI26X_CONFIG_ENABLE_FLUSH_PERIOD
    UNUSED_VAR(this);
    bmi26x_instance_state *istate =
        (bmi26x_instance_state*)instance->state->state;
    sns_sensor_uid suid = SENSOR_TEMPERATURE_SUID;
    sns_request const *request;
    bool is_max_batch = true;

    *chosen_report_rate = 0.0f;
    *chosen_sample_rate = 0.0f;
    *chosen_flush_period_ticks = 0;
    *sensor_temp_client_present = false;

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false)) {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            if (bmi26x_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload)) {
                float report_rate = 0.0f;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                              decoded_payload.sample_rate);

                if (decoded_request.has_batching
                        &&
                        decoded_request.batching.batch_period > 0) {
                    report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                } else {
                    report_rate = decoded_payload.sample_rate;
                }

                if (decoded_request.has_batching) {
                    bool is_flush_only = true;
                    is_max_batch &= (decoded_request.batching.has_max_batch && decoded_request.batching.max_batch);
                    is_flush_only &= (decoded_request.batching.flush_only);
                    BMI26X_SENSOR_LOG(MED, this, "is flush only:%d", is_flush_only);

                    if (decoded_request.batching.has_flush_period) {
                        sns_time batching_flush_period = (sns_time) ((sns_time)decoded_request.batching.flush_period * 1000);
                        batching_flush_period = sns_convert_ns_to_ticks(batching_flush_period);
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks, batching_flush_period);
                    } else {
                        sns_time batching_flush_period = (sns_time) ((sns_time)decoded_request.batching.batch_period * 1000);
                        batching_flush_period = sns_convert_ns_to_ticks(batching_flush_period);
                        *chosen_flush_period_ticks =
                            SNS_MAX(*chosen_flush_period_ticks, batching_flush_period);
                    }
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
                *sensor_temp_client_present = true;
            }
        }
    }

    if (*sensor_temp_client_present &&
            (is_max_batch)) {
        *chosen_report_rate = (1.0f / UINT32_MAX);
    }

    istate->sensor_temp_info.report_rate_req  = *chosen_report_rate;
    istate->sensor_temp_info.sample_rate_req = *chosen_sample_rate;
#else
    UNUSED_VAR(this);
    bmi26x_instance_state *istate =
        (bmi26x_instance_state*)instance->state->state;
    sns_sensor_uid suid = SENSOR_TEMPERATURE_SUID;
    sns_request const *request;

    *chosen_report_rate = 0;
    *chosen_sample_rate = 0;
    *sensor_temp_client_present = false;

    /** Parse through existing requests and get fastest sample
     *  rate and report rate requests. */
    for (request = instance->cb->get_client_request(instance, &suid, true);
            NULL != request;
            request = instance->cb->get_client_request(instance, &suid, false)) {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            if (bmi26x_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload)) {
                float report_rate;
                *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                              decoded_payload.sample_rate);

                bool rc = sns_sensor_util_decide_max_batch(instance, &suid);
                //There is request with max batch not set .
                // do normal calculation

                if (!rc) {
                    if (decoded_request.has_batching
                            &&
                            decoded_request.batching.batch_period > 0) {
                        report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
                    } else {
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


/**
 *
 * @param this
 * @param instance
 * @param registry_cfg
 * @param message_id
 */
static void bmi26x_set_inst_config(
    sns_sensor                  *this,
    sns_sensor_instance         *instance,
    uint32_t                    message_id)
{
    sns_request                     config;
    bmi26x_req_payload_t        req_payload = {.req_ssensor = this};

    config.message_id = message_id;
    config.request_len = sizeof(req_payload);
    config.request = &req_payload;

    this->instance_api->set_client_config(instance, &config);
}


/**
 * Turn off power rails
 * @param this   sensor handler
 */
static void bmi26x_turn_rails_off(sns_sensor *this)
{
    if (sns_sensor_util_get_shared_instance(this) == NULL) {
        sns_sensor *sensor;
        for (sensor = this->cb->get_library_sensor(this, true);
            NULL != sensor;
            sensor = this->cb->get_library_sensor(this, false)) {
            bmi26x_state *sensor_state = (bmi26x_state*)sensor->state->state;
            if (sensor_state->common.rail_config.rail_vote != SNS_RAIL_OFF) {
                sensor_state->common.rail_config.rail_vote = SNS_RAIL_OFF;
                sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
                                                                                sensor,
                                                                                &sensor_state->common.rail_config,
                                                                                NULL);
            }
        }
    }
}

/*!
 * Clean up power rail request
 * @param this  the sensor handler
 */
static void bmi26x_sensor_pwr_rail_cleanup(sns_sensor *this)
{
#if BMI26X_CONFIG_POWER_RAIL
#if BMI26X_CONFIG_ENABLE_POWER_KEEP_ALIVE
    UNUSED_VAR(this);
#else
#if BMI26X_CONFIG_ENABLE_POWER_RAIL_DELAY_OFF
    sns_time ts_pending_deinit = bmi26x_convert_us2ticks(BMI26X_DELAY_DEINIT_IN_MS * 1000);
    if (bmi26x_start_power_rail_timer(this, ts_pending_deinit,
                                      BMI26X_POWER_RAIL_PENDING_DEINIT) != SNS_RC_SUCCESS) {
        bmi26x_turn_rails_off(this);
    }
#else
    bmi26x_turn_rails_off(this);
#endif
#endif
#else
    UNUSED_VAR(this);
#endif
}


/*!
 * check and validate the motion detection request
 * @param sstate     sensor state
 * @param istate     instance state
 * @return           true if approve else false
 */
static
bool bmi26x_check_n_approve_md_req(
    bmi26x_state            *sstate,
    bmi26x_instance_state   *istate)
{
    bool approve_req = true;
    UNUSED_VAR(sstate);

    BMI26X_SENSOR_LOG(LOW, sstate->owner, "get MD req");
    if (istate->fifo_info.publish_sensors & BMI26X_ACCEL) {
        //send event as MD disabled since non-gated client is active
        //no need of this as we already set md_info state
        sns_motion_detect_event md_state;
        md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
        pb_send_event(istate->owner,
                      sns_motion_detect_event_fields,
                      &md_state,
                      bmi26x_get_sys_tick(),
                      SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                      &istate->md_info.sstate->my_suid);
        //as per requirement: reject the request directly
        BMI26X_SENSOR_LOG(HIGH, sstate->owner, "MD request rejected");
        approve_req = false;
    } else if (istate->int_en_flags_req.bits.md) {
        approve_req = false;
        //there is exsisting md client already present, just send event
        pb_send_event(istate->owner,
                      sns_motion_detect_event_fields,
                      &istate->md_info.md_state,
                      bmi26x_get_sys_tick(),
                      SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                      &istate->md_info.sstate->my_suid);
        BMI26X_SENSOR_LOG(HIGH, sstate->owner, "there is already existing md client");
    } else {
        istate->md_info.md_new_req = true;
        BMI26X_SENSOR_LOG(HIGH, sstate->owner, "MD accepted");
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
static bool bmi26x_extract_self_test_info(
    sns_sensor          *this,
    sns_sensor_instance *instance,
    struct sns_request const *new_request)
{
    sns_std_request         decoded_request;
    sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
    bmi26x_state            *sstate = (bmi26x_state*)this->state->state;
    bmi26x_instance_state   *istate = (bmi26x_instance_state*)instance->state->state;
    bmi26x_self_test_info_t   *self_test_info;

    if (sstate->sensor == BMI26X_ACCEL) {
        self_test_info = &istate->accel_info.test_info;
    } else if (sstate->sensor == BMI26X_GYRO) {
        self_test_info = &istate->gyro_info.test_info;
    } else if (sstate->sensor == BMI26X_MOTION_DETECT) {
        self_test_info = &istate->md_info.test_info;
    } else if (sstate->sensor == BMI26X_SENSOR_TEMP) {
        self_test_info = &istate->sensor_temp_info.test_info;
    }
#if BMI26X_CONFIG_ENABLE_PEDO
    else if (sstate->sensor == BMI26X_PEDO) {
        self_test_info = &istate->pedo_info.test_info;
    }
#endif
#if BMI26X_CONFIG_ENABLE_LOWG
    else if (sstate->sensor == BMI26X_FREE_FALL) {
        self_test_info = &istate->lowg_info.test_info;
    }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    else if (sstate->sensor == BMI26X_DOUBLE_TAP) {
        self_test_info = &istate->dbtap_info.test_info;
    }
#endif

    else {
        return false;
    }

    self_test_info->sensor = sstate->sensor;

    if (bmi26x_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config)) {
        self_test_info->test_type = test_config.test_type;
        self_test_info->test_client_present = true;

        BMI26X_SENSOR_LOG(MED, this, "new client_request to do test: %d %d", sstate->sensor, self_test_info->test_type);
        return true;
    } else {
        return false;
    }
}


#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
static bool bmi26x_extract_self_test_info_bias(sns_sensor *this,
        sns_sensor_instance *instance,
        struct sns_request const *new_request)
{
    sns_std_request decoded_request;
    sns_physical_sensor_oem_config test_config = sns_physical_sensor_oem_config_init_default;
    bmi26x_state *state = (bmi26x_state*)this->state->state;
    bmi26x_instance_state *inst_state = (bmi26x_instance_state*)instance->state->state;
    bmi26x_self_test_info_bias_t *self_test_info_bias;

    if (state->sensor == BMI26X_ACCEL) {
        self_test_info_bias = &inst_state->accel_info.test_info_bias;
    } else if (state->sensor == BMI26X_GYRO) {
        self_test_info_bias = &inst_state->gyro_info.test_info_bias;
    } else {
        return false;
    }

    if (bmi26x_get_decoded_self_test_request_bias(this, new_request, &decoded_request, &test_config)) {
        self_test_info_bias->config_type = test_config.config_type;
        self_test_info_bias->test_client_present = true;

        self_test_info_bias->offset_x = test_config.offset_x;
        self_test_info_bias->offset_y = test_config.offset_y;
        self_test_info_bias->offset_z = test_config.offset_z;

        BMI26X_SENSOR_LOG(HIGH, this, "bmi26x : bmi16_extract_self_test_info_bias [config_type:%d][x:%d][y:%d][z:%d]",
                          test_config.config_type,
                          (int)(test_config.offset_x * 10),
                          (int)(test_config.offset_y * 10),
                          (int)(test_config.offset_z * 10));
        return true;
    } else {
        return false;
    }
}
#endif

/** Functions shared by all BMI26X Sensors */
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

static
void bmi26x_reval_instance_config(
    sns_sensor              *this,
    sns_sensor_instance     *instance)
{
    /**
     * 1. Get best non-gated and gated Accel Config.
     * 2. Get best Gyro Config.
     * 3. Get Motion Detect Config.
     * 4. Get best Sensor Temperature Config.
     * 5. Decide best Instance Config based on above outputs.
     */
    bmi26x_state                *sstate = (bmi26x_state*)this->state->state;
    bmi26x_instance_state       *istate = (bmi26x_instance_state*)instance->state->state;
    float                       sample_rate = 0.0f;
    float                       report_rate = 0.0f;
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

    BMI26X_SENSOR_LOG(LOW, this, "reval instance entrance from sensor:%d", sstate->sensor);

    if (!istate->fac_test_in_progress) {
    } else {
        BMI26X_SENSOR_LOG(HIGH, this, "NOTICE!!! inst_cfg postponed due to fac_test_in_progress: %d",
                        istate->fac_test_info.fac_test_sensor);
        return;
    }

    bmi26x_get_imu_config(this,
                          instance,
                          BMI26X_ACCEL,
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

    if((a_sensor_client_present_non_gated == false) &&
            a_sensor_client_present_gated) {
        istate->accel_info.max_batching_available = 1;
    } else {
        istate->accel_info.max_batching_available = 0;
    }

    BMI26X_SENSOR_LOG(MED, this, "cfg_acc cp:%d, rates sr:%d, rr:%d fp:%u",
                    ((a_sensor_client_present_gated << 1) | (a_sensor_client_present_non_gated & 1)),
                    (int)(sample_rate * 1000), (int)(report_rate * 1000),
                    (uint32_t)acc_flush_period_ticks);

    if (a_sensor_client_present_gated && a_sensor_client_present_non_gated) {
        BMI26X_SENSOR_LOG(MED, this, "NOTICE both gated and non-gated clients are present");
    }

    bmi26x_get_imu_config(this,
                          instance,
                          BMI26X_GYRO,
                          &sample_rate,
                          &report_rate,
                          &gyro_flush_period_ticks,
                          &g_sensor_client_present,
                          NULL);

    istate->gyro_info.sample_rate_req = sample_rate;
    istate->gyro_info.report_rate_req = report_rate;
    istate->gyro_info.client_present = g_sensor_client_present;
    istate->gyro_info.flush_period_ticks = gyro_flush_period_ticks;

    BMI26X_SENSOR_LOG(MED, this, "cfg_gyro cp:%d, rates sr:%d, rr:%d fp:%u",
                    g_sensor_client_present,
                    (int)(sample_rate * 1000), (int)(report_rate * 1000),
                    (uint32_t)gyro_flush_period_ticks);


#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
#endif

    bmi26x_get_motion_detect_config(this,
                                    instance,
                                    &enable_md_int,
                                    &md_sensor_client_present);

    istate->int_en_flags_req.bits.md = enable_md_int;
    istate->md_info.client_present = md_sensor_client_present;

    BMI26X_SENSOR_LOG(MED, this, "cfg_md cp:%d, en:%d",
                      istate->md_info.client_present,
                      istate->int_en_flags_req.bits.md);

    if (a_sensor_client_present_non_gated) {
        istate->int_en_flags_req.bits.md = false;
    }

#if BMI26X_CONFIG_ENABLE_PEDO
    bool pedo_client_present = false;
    bmi26x_get_advanced_sensor_config(this, instance,
                                      BMI26X_PEDO,
                                      &pedo_client_present);

    // pedo meter has a fixed macro definition sample rate
    istate->int_en_flags_req.bits.pedo = pedo_client_present;
    istate->pedo_info.client_present = pedo_client_present;
    istate->pedo_info.enable_pedo_int = pedo_client_present;

    BMI26X_INST_LOG(MED, instance, "cfg_pedo cp:%d, en:%d",
                    istate->pedo_info.client_present,
                    istate->int_en_flags_req.bits.pedo);
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    bool lowg_client_present = false;
    bmi26x_get_advanced_sensor_config(this, instance, BMI26X_FREE_FALL,
                                      &lowg_client_present);

    istate->int_en_flags_req.bits.lowg = lowg_client_present;
    istate->lowg_info.client_present = lowg_client_present;

    if (istate->lowg_info.lowg_wait_for_fired_event && lowg_client_present) {
        istate->int_en_flags_req.bits.lowg = lowg_client_present;
    } else {
        istate->int_en_flags_req.bits.lowg = 0;
    }

    BMI26X_SENSOR_LOG(MED, this, "cfg_lowg cp:%d, en:%d",
                    istate->lowg_info.client_present,
                    istate->int_en_flags_req.bits.lowg);

    BMI26X_SENSOR_LOG(MED, this, "cfg_lowg en:%d, mode:%d, debug:%d",
                    (uint32_t)(istate->lowg_info.sstate->lowg_config.enable),
                    (uint32_t)(istate->lowg_info.sstate->lowg_config.mode),
                    istate->lowg_info.sstate->lowg_config.debug);
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    bool dbtap_client_present = false;
    bmi26x_get_advanced_sensor_config(this, instance, BMI26X_DOUBLE_TAP,
                                      &dbtap_client_present);

    istate->int_en_flags_req.bits.dbtap = dbtap_client_present;
    istate->dbtap_info.client_present = dbtap_client_present;

    if (istate->dbtap_info.dbtap_wait_for_fired_event && dbtap_client_present) {
        istate->int_en_flags_req.bits.dbtap = dbtap_client_present;
    } else {
        istate->int_en_flags_req.bits.dbtap = 0;
    }

    BMI26X_SENSOR_LOG(MED, this, "cfg_dbtap cp:%d, en:%d",
                    istate->dbtap_info.client_present,
                    istate->int_en_flags_req.bits.dbtap);

    BMI26X_SENSOR_LOG(MED, this, "cfg_dbtap <%d %d %d %d>",
                    (uint32_t)(istate->dbtap_info.sstate->dbtap_cfg.enable),
                    istate->dbtap_info.sstate->dbtap_cfg.tap_sens_thres,
                    (uint32_t)(istate->dbtap_info.sstate->dbtap_cfg.max_gest_dur),
                    istate->dbtap_info.sstate->dbtap_cfg.quite_time_after_gest);
#endif


#if BMI26X_CONFIG_ENABLE_OIS
    bool ois_client_present = false;
    bmi26x_get_advanced_sensor_config(this, instance,
                                      BMI26X_OIS,
                                      &ois_client_present);
    istate->ois_info.client_present = ois_client_present;

    BMI26X_SENSOR_LOG(MED, this, "cfg_ois acc.cp/en:%d",
                    istate->ois_info.client_present);
#endif

    bmi26x_get_sensor_temp_config(this,
                                  instance,
                                  &sample_rate,
                                  &report_rate,
                                  &temp_flush_period_ticks,
                                  &sensor_temp_client_present);

    istate->sensor_temp_info.client_present = sensor_temp_client_present;
    istate->sensor_temp_info.max_requested_flush_ticks = temp_flush_period_ticks;

    BMI26X_SENSOR_LOG(MED, this, "cfg_temp cp:%d, rates sr:%d, rr:%d fp:%u",
                    sensor_temp_client_present,
                    (int)(sample_rate * 1000), (int)(report_rate * 1000),
                    (uint32_t) temp_flush_period_ticks);

    /** Start with a clean sstate */
    istate->hw_mod_needed = 0;
    istate->fifo_info.publish_sensors = 0;

    if (a_sensor_client_present_non_gated) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors |= BMI26X_ACCEL;
    }

    if (a_sensor_client_present_gated) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        if (!istate->int_en_flags_req.bits.md) {
            istate->fifo_info.publish_sensors |= BMI26X_ACCEL;
        }
    }

    if (g_sensor_client_present) {
        istate->hw_mod_needed |= (BMI26X_GYRO); //TOCHECK
        istate->fifo_info.publish_sensors |= BMI26X_GYRO;
    }

#if BMI26X_CONFIG_ENABLE_MAG_IF //TODOMAG
#endif

    if (md_sensor_client_present) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
    }

#if BMI26X_CONFIG_ENABLE_PEDO
    if (pedo_client_present) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors |= BMI26X_PEDO;
        /* istate->pedo_info.is_first = true;
         BMI26X_SENSOR_LOG(HIGH, this, "pedo_info.is_first: %d, pb:0x%x",
                           istate->pedo_info.is_first,
                           istate->fifo_info.publish_sensors);*/
    }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
    if (lowg_client_present) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors |= BMI26X_FREE_FALL;
    }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    if (dbtap_client_present) {
        istate->hw_mod_needed |= BMI26X_ACCEL;
        istate->fifo_info.publish_sensors |= BMI26X_DOUBLE_TAP;
    }
#endif

#if BMI26X_CONFIG_ENABLE_OIS
    if (ois_client_present) {
        istate->hw_mod_needed |= BMI26X_GYRO;
        istate->fifo_info.publish_sensors |= BMI26X_OIS;
    }
#endif

    if (sensor_temp_client_present) {
        istate->hw_mod_needed |= BMI26X_SENSOR_TEMP;
        istate->fifo_info.publish_sensors |= BMI26X_SENSOR_TEMP;
    }

    BMI26X_SENSOR_LOG(MED, this, "inst_cfg summary ps:0x%x, hmn:0x%x",
                      istate->fifo_info.publish_sensors,
                      istate->hw_mod_needed);

    bmi26x_set_inst_config(this,
                           instance,
                           SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);

#if 0
    if (!istate->hw_mod_needed && !istate->new_self_test_request) {
#if BMI26X_CONFIG_POWER_RAIL
        bmi26x_turn_rails_off(this);
        BMI26X_SENSOR_LOG(MED, this, "sensor: %d req to turn off rails", sstate->sensor);
#endif
        //TODO2: check more
        istate->sbus_in_normal_mode = false;
        istate->pmu_stat.reg = 0;
    }
#endif
}


static void bmi26x_sensor_check_pending_flush_requests(
  sns_sensor           *this,
  sns_sensor_instance  *instance)
{
    sns_sensor *lib_sensor;
    for(lib_sensor = this->cb->get_library_sensor(this, true);
      NULL != lib_sensor;
      lib_sensor = this->cb->get_library_sensor(this, false)) {
        bmi26x_state *sstate = (bmi26x_state*)lib_sensor->state->state;
        if(sstate->ss_flush_req) {
            sstate->ss_flush_req = false;
            BMI26X_SENSOR_LOG(MED, this, "pending_flush_requests: sensor=%u", sstate->sensor);
            bmi26x_hal_send_fifo_flush_done(instance, sstate->sensor, sns_get_system_time(),
                                            BMI26X_FLUSH_DONE_CONFIGURING);
        }
    }
}


/*!
 * Check power state available
 * @param inst  instance handler
 * @return true if the power state is available
 *         false if the power state is not ready
 */
static bool bmi26x_sensor_pwr_rail_state_available(sns_sensor *const this, sns_sensor_instance *inst)
{
    bmi26x_instance_state   *istate = (bmi26x_instance_state*) inst->state->state;

   bool pwr_available = true;
   sns_sensor *lib_sensor;

   for(lib_sensor = this->cb->get_library_sensor(this, true);
       NULL != lib_sensor;
       lib_sensor = this->cb->get_library_sensor(this, false)) {
       bmi26x_state *lib_state = (bmi26x_state*)lib_sensor->state->state;
       pwr_available &= ((lib_state->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_NONE) ||
                        (lib_state->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_DEINIT));

       BMI26X_INST_LOG(MED, inst, "pwr.rail.st chk: ss %d@%d", lib_state->sensor,
                       lib_state->power_rail_pend_state);
   } //@

   if (istate->pwr_state_present == BMI26X_POWER_RAIL_PENDING_NONE) {
       pwr_available |= true;
   }

   return pwr_available;
}

/*!
 * Check power state available
 * @param inst  instance handler
 * @return true if the power state is available
 *         false if the power state is not ready
 */
static bool bmi26x_sensor_is_pwr_rail_pending_cfg(sns_sensor *const this)
{
   bool is_pwr_pending_cfg = false;
   sns_sensor *lib_sensor;

   for(lib_sensor = this->cb->get_library_sensor(this, true);
       NULL != lib_sensor;
       lib_sensor = this->cb->get_library_sensor(this, false)) {
       bmi26x_state *lib_state = (bmi26x_state*)lib_sensor->state->state;
       is_pwr_pending_cfg |= (lib_state->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ);
   } //@

   return is_pwr_pending_cfg;
}

/*!
 * Check power state available
 * @param inst  instance handler
 * @return true if the power state is available
 *         false if the power state is not ready
 */
static void bmi26x_sensor_clear_timer_stream(sns_sensor *const this)
{
   sns_sensor *lib_sensor;

   for(lib_sensor = this->cb->get_library_sensor(this, true);
       NULL != lib_sensor;
       lib_sensor = this->cb->get_library_sensor(this, false)) {
       bmi26x_state *lib_state = (bmi26x_state*)lib_sensor->state->state;
       if (lib_state->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ) {
           // rm
           BMI26X_SENSOR_LOG(MED, this, "rm ts.stream:%p", lib_state->timer_stream);
           sns_sensor_util_remove_sensor_stream(this, &lib_state->timer_stream);
           lib_state->timer_stream = NULL;
           lib_state->power_rail_pend_state = BMI26X_POWER_RAIL_PENDING_NONE;
       }
   } //@
}


/** See sns_bmi26x_sensor.h*/
sns_rc bmi26x_sensor_notify_event(sns_sensor *const this)
{
    bmi26x_state            *sstate = (bmi26x_state*)this->state->state;
    sns_rc                  rv = SNS_RC_SUCCESS;
    sns_sensor_event        *event;
    sns_sensor_instance     *instance = sns_sensor_util_get_shared_instance(this);
    bmi26x_instance_state   *istate = NULL;
    bool                    pwr_rail_state_pending = false;

    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_sif_ notify_event> @ss:%d this: 0x%x",
                      sstate->sensor, this);

#if BMI26X_CONFIG_ENABLE_DAE
    BMI26X_SENSOR_LOG(MED, this, "dae_sensor_events: state(ag)=%d state(t): %d",
                      sstate->common.dae_ag_state,
                      sstate->common.dae_temper_state)
#endif

    if (instance != NULL) {
        istate = (bmi26x_instance_state *) instance->state->state;
        pwr_rail_state_pending = bmi26x_sensor_is_pwr_rail_pending_cfg(this);
        BMI26X_SENSOR_LOG(MED, this, "cfg pending st:%d, tm stream:%p",
                        pwr_rail_state_pending, sstate->timer_stream);
    }

    if (!sns_suid_lookup_complete(&sstate->common.suid_lookup_data)) {
        bmi26x_sensor_exit_island(this);

        sns_suid_lookup_handle(this, &sstate->common.suid_lookup_data);

#if BMI26X_CONFIG_ENABLE_REGISTRY
        if (sns_suid_lookup_get(&sstate->common.suid_lookup_data, "registry", NULL)) {
            bmi26x_request_registry(this);
        }
#else
        sns_bmi26x_registry_def_config(this);
#endif
        BMI26X_SENSOR_LOG(LOW, this, "snr: %d r_idx=%d", sstate->sensor, sstate->resolution_idx);

        if (sns_suid_lookup_complete(&sstate->common.suid_lookup_data)) {
            sns_suid_lookup_deinit(this, &sstate->common.suid_lookup_data);
        }
    } else {
        BMI26X_SENSOR_LOG(LOW, this, "look up complete");
    }

    /**----------------------Handle a Timer Sensor event.-------------------*/
    if (NULL != sstate->timer_stream) {
        event = sstate->timer_stream->api->peek_input(sstate->timer_stream);
        while (NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                  event->event_len);
            sns_timer_sensor_event timer_event;
            if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id) {

                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
#if BMI26X_CONFIG_POWER_RAIL
                    if (sstate->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_INIT) {
                        bmi26x_sensor_exit_island(this);

                        sstate->common.hw_is_present = bmi26x_discover_hw(this);
                        if (sstate->common.hw_is_present) {
                            BMI26X_SENSOR_LOG(MED, this, "sensor:%d init finished", sstate->sensor);
                            bmi26x_publish_available(this);
                            bmi26x_update_sibling_sensors(this);
#if BMI26X_CONFIG_ENABLE_DAE
                            if (sstate->sensor == BMI26X_ACCEL) {
                                bmi26x_dae_if_check_support(this);
                            }
#endif
                        } else {
                            rv = SNS_RC_INVALID_LIBRARY_STATE;
                            BMI26X_SENSOR_LOG(HIGH, this, "WARN!!! BMI26X HW absent");
                        }
                        sstate->power_rail_pend_state = BMI26X_POWER_RAIL_PENDING_NONE;
                    } else if (pwr_rail_state_pending) {
                        BMI26X_SENSOR_LOG(HIGH, this, "timer event for instance: %p set_client_request snr: %d",
                                        instance, sstate->sensor);
                        if (instance == NULL) {
                            BMI26X_SENSOR_LOG(MED, this, "instance no longer available");
                            bmi26x_sensor_pwr_rail_cleanup(this);
                        } else {

                            bmi26x_reval_instance_config(this, instance);
                            if (BST_ASSERT_POINT(istate)) {
                                return SNS_RC_INVALID_VALUE;
                            }
                            if (istate->new_self_test_request) {
                                //bmi26x_set_self_test_inst_config(this, instance);
                                bmi26x_set_inst_config(this, instance,
                                                  SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG);
                            }
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
                            if (istate->new_self_test_request_bias) {
                                bmi26x_set_self_test_inst_config_bias(this, instance);
                            }
#endif

                            sstate->power_rail_pend_state = BMI26X_POWER_RAIL_PENDING_NONE;
                            bmi26x_sensor_check_pending_flush_requests(this, instance);
                        }
                    } else if (sstate->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_DEINIT) {
                        BMI26X_SENSOR_LOG(HIGH, this, "power rail delay off now:%u",
                                          (uint32_t) sns_get_system_time());
                        bmi26x_turn_rails_off(this);
                        sstate->power_rail_pend_state = BMI26X_POWER_RAIL_PENDING_NONE;
                    }
#endif  //BMI26X_CONFIG_POWER_RAIL
                } else {
                    BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! pb_decode error");
                }
            }

            event = sstate->timer_stream->api->get_next_input(sstate->timer_stream);
        }

        /** Free up timer stream if not needed anymore */
        if (sstate->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_NONE) {
            BMI26X_SENSOR_LOG(HIGH, this, "ts remove:%p on ss:%d",
                              sstate->timer_stream, sstate->sensor);
            sns_sensor_util_remove_sensor_stream(this, &sstate->timer_stream);
            sstate->timer_stream = NULL;
        }
    }

#if BMI26X_CONFIG_ENABLE_REGISTRY
    if (NULL != sstate->reg_data_stream) {
        event = sstate->reg_data_stream->api->peek_input(sstate->reg_data_stream);
        bmi26x_sensor_exit_island(this);

        while (NULL != event) {
            bmi26x_sensor_process_registry_event(this, event);
            event = sstate->reg_data_stream->api->get_next_input(sstate->reg_data_stream);
        }

        bmi26x_sensor_exit_island(this);
        bmi26x_sensor_check_registry_col_progress(this);
    }
#endif

    // DAE
    bmi26x_dae_if_process_sensor_events(this);

    //note: all the registry items must be provided
    if (sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", NULL) &&
            sstate->common.registry_pf_cfg_received && sstate->registry_cfg_received &&
            sstate->common.registry_orient_received && sstate->common.registry_placement_received
            ) {

        bmi26x_sensor_exit_island(this);

        if (!sstate->common.hw_is_present &&
                        !sstate->common.hw_is_finish_detection &&
                        sstate->power_rail_pend_state == BMI26X_POWER_RAIL_PENDING_NONE) {
            BMI26X_SENSOR_LOG(MED, this, "start_hw_detect_seq snr:%d", sstate->sensor);
            bmi26x_start_hw_detect_sequence(this);
        }
    }

    return rv;
}

static sns_sensor_instance *bmi26x_create_new_instance(sns_sensor *const this)
{
    sns_sensor_instance *instance = NULL;

    if (NULL != this) {
#if BMI26X_CONFIG_POWER_RAIL
        bmi26x_state *sstate = (bmi26x_state*)this->state->state;
        sns_time on_timestamp, delta;
        sns_time off2idle = sns_convert_ns_to_ticks(BMI26X_OFF_TO_IDLE_MS * 1000 * 1000);

        if (sstate->sensor == BMI26X_GYRO) {
            sstate->common.rail_config.rail_vote = SNS_RAIL_ON_NPM;
        } else {
            //TOCHECK
            sstate->common.rail_config.rail_vote = sstate->common.registry_rail_on_state;
        }

        sstate->pwr_rail_service->api->sns_vote_power_rail_update(sstate->pwr_rail_service,
                                                                  this,
                                                                  &sstate->common.rail_config,
                                                                  &on_timestamp);

        delta = sns_get_system_time() - on_timestamp;

        BMI26X_SENSOR_LOG(MED, this, "pwr rail update:%u %u %u",
                            BMI26X_SYS_TIME_LH(on_timestamp),
                            BMI26X_SYS_TIME_LH(sns_get_system_time()),
                            BMI26X_SYS_TIME_LH(delta));

        // Use on_timestamp to determine correct Timer value.
        if (delta < off2idle) {
            if (bmi26x_start_power_rail_timer(this,
                                          off2idle - delta,
                                          BMI26X_POWER_RAIL_PENDING_SET_CLIENT_REQ)) {
            }
        } else {
            // rail is already ON for time long enough
            sstate->power_rail_pend_state = BMI26X_POWER_RAIL_PENDING_NONE;
            BMI26X_SENSOR_LOG(MED, this, "rail already on: %u %u", BMI26X_SYS_TIME_LH(on_timestamp), BMI26X_SYS_TIME_LH(delta));
        }
#else
        UNUSED_VAR(on_timestamp);
        UNUSED_VAR(delta);
        reval_config = true;
#endif
    }

    if (NULL != this) {
        // must create instance from master sensor whose state includes the shared state
        BMI26X_SENSOR_LOG(LOW, this, "creating inst");
        instance = this->cb->create_instance(this, sizeof(bmi26x_instance_state));
    }

    return instance;
}


static void bmi26x_handle_flush_request(
    sns_sensor           *this,
    sns_sensor_instance  *inst)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    bmi26x_instance_state *istate = (bmi26x_instance_state *) inst->state->state;
    bmi26x_flush_done_reason_t reason = BMI26X_FLUSH_TO_BE_DONE;
    uint8_t ff_send_flush_done = BMI26X_FIFO_FLUSH_DONE_CTX_CLIENT_REQ;

#if BMI26X_CONFIG_POWER_RAIL
    bool pwr_state_available = bmi26x_sensor_pwr_rail_state_available(this, inst);

    if (pwr_state_available == false) {
        reason = BMI26X_FLUSH_DONE_CONFIGURING;
#if 1
        sstate->ss_flush_req = true;
        UNUSED_VAR(reason);
        return ;
#endif
    } else
#endif
        if (sstate->sensor & (BMI26X_MOTION_DETECT | BMI26X_SENSOR_TEMP
                | BMI26X_PEDO | BMI26X_OIS | BMI26X_FREE_FALL)) {
            reason = BMI26X_FLUSH_DONE_NOT_ACCEL_GYRO;
        } else if ((istate->fifo_info.ff_sensors_en_curr & (BMI26X_ACCEL | BMI26X_GYRO)) == 0) {
            reason = BMI26X_FLUSH_DONE_ACC_GYRO_DISABLED;
        } else if (istate->fifo_info.publish_sensors == 0
                   && !bmi26x_dae_if_available(inst)) {
            reason = BMI26X_FLUSH_DONE_NOT_FIFO;
        }

    UNUSED_VAR(reason);

    BMI26X_SENSOR_LOG(MED, this, "flush req assert reson:%d @ss:%d", reason, sstate->sensor);

    if (reason != BMI26X_FLUSH_TO_BE_DONE) {
        if ((sstate->sensor == BMI26X_ACCEL) ||
                        (sstate->sensor == BMI26X_GYRO)) {

            bmi26x_hal_send_fifo_flush_done(inst, (uint8_t)istate->accel_info.sstate->sensor,
                                            sns_get_system_time(),
                                            ff_send_flush_done);
            bmi26x_hal_send_fifo_flush_done(inst, (uint8_t)istate->gyro_info.sstate->sensor,
                                            sns_get_system_time(),
                                            ff_send_flush_done);
        } else {
            bmi26x_hal_send_fifo_flush_done(inst, (uint8_t)sstate->sensor, sns_get_system_time(),
                                        ff_send_flush_done);
        }
    } else {
        //bmi26x_send_flush_config(this, inst);
        bmi26x_set_inst_config(this, inst,
                               SNS_STD_MSGID_SNS_STD_FLUSH_REQ);
    }
}


static void bmi26x_handle_new_request(
    sns_sensor               *const this,
    sns_sensor_instance      *instance,
    struct sns_request const *exist_request,
    struct sns_request const *new_request)
{
    bmi26x_state            *sstate = (bmi26x_state*) this->state->state;
    bmi26x_instance_state   *istate = (bmi26x_instance_state*) instance->state->state;
    bool reval_config = false;
    bool pwr_state_available = bmi26x_sensor_pwr_rail_state_available(this, instance);

    // remove exist request
    if (NULL != exist_request) {
        BMI26X_SENSOR_LOG(LOW, this, "replace req");
        instance->cb->remove_client_request(instance, exist_request);
    }

    // add new req to the req list
    instance->cb->add_client_request(instance, new_request);


    if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == new_request->message_id ||
            SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG == new_request->message_id) {
        reval_config = true;
    } else if ((new_request->message_id ==
                SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
               &&
               (BMI26X_MOTION_DETECT == sstate->sensor)) {
        if (!bmi26x_check_n_approve_md_req(sstate, istate)) {
            reval_config = false;
        } else {
            reval_config = true;
        }
    }
#if BMI26X_CONFIG_ENABLE_LOWG
    else if ((new_request->message_id ==
                    SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
                   &&
                   (BMI26X_FREE_FALL == sstate->sensor)) {
        reval_config = true;
        istate->lowg_info.lowg_new_req = true; //mark the new req
    }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
    else if ((new_request->message_id ==
                    SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
                   &&
                   (BMI26X_DOUBLE_TAP == sstate->sensor)) {
        reval_config = true;
        istate->dbtap_info.dbtap_new_req = true; //mark the new req
    }
#endif

    else if (SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG ==
               new_request->message_id) {
        if (bmi26x_extract_self_test_info(this, instance, new_request)) {
            istate->new_self_test_request = true;
            // TODO need reval_instance_config ?
            //bmi26x_set_self_test_inst_config(this, instance);
#if 0  // FIXME
            bmi26x_set_inst_config(this, instance,
                                   SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG);
#endif
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
            if (istate->new_self_test_request_bias) {
                bmi26x_set_self_test_inst_config_bias(this, instance);
            }
#endif
        }
    }
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    else if (new_request->message_id ==
                    SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG) {
        BMI26X_SENSOR_LOG(MED, this, "SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG is called");
        if (bmi26x_extract_self_test_info_bias(this, instance, new_request)) {
            istate->new_self_test_request_bias = true;
        }
    }
#endif
    else {
        instance->cb->remove_client_request(instance, new_request);
        BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! unkonw req:%d", new_request->message_id);
    }

    BMI26X_SENSOR_LOG(MED, this, "reval:%d, pwr.avail:%d", reval_config,
                    pwr_state_available);

    if (reval_config && pwr_state_available) {
        bmi26x_reval_instance_config(this, instance);
    }

    if (pwr_state_available && istate->new_self_test_request) {
        bmi26x_set_inst_config(this, instance,
                               SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG);
#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
        if (istate->new_self_test_request_bias) {
            bmi26x_set_self_test_inst_config_bias(this, instance);
        }
#endif
    }

#if BMI26X_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    if (pwr_state_available && istate->new_self_test_request_bias) {
        bmi26x_set_inst_config(this, instance,
                               SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG);
    }
#endif

}

static void bmi26x_sensor_handle_cal_reset_req(sns_sensor  *this,
        sns_sensor_instance  *inst)
{
    bmi26x_state *sstate = (bmi26x_state *) this->state->state;
    bool    op_valid = false;

    //TODO2: how about temperature
    if ((BMI26X_ACCEL == sstate->sensor) || (BMI26X_GYRO == sstate->sensor)) {
        op_valid = true;
    }

    BMI26X_SENSOR_LOG(MED, this, "Request for resetting cal data: %d", sstate->sensor);
    if (op_valid) {
        bmi26x_sensor_exit_island(this);

        bmi26x_reset_fac_cal_data(inst, sstate, true);

#if BMI26X_CONFIG_ENABLE_REGISTRY
#if BMI26X_CONFIG_ENABLE_CRT
        bmi26x_crt_update_registry(this, inst, (bmi26x_sensor_type)sstate->sensor);
#endif
        bmi26x_update_registry(this, inst, (bmi26x_sensor_type)sstate->sensor);
#endif
        bmi26x_send_fac_cal_event(inst, sstate);
    }
}

static bool bmi26x_sensor_is_request_present(sns_sensor_instance *inst)
{
    if (NULL == inst->cb->get_client_request(inst,
            &(sns_sensor_uid)MOTION_DETECT_SUID, true) &&
            NULL == inst->cb->get_client_request(inst,
                    &(sns_sensor_uid)ACCEL_SUID, true) &&
            NULL == inst->cb->get_client_request(inst,
                    &(sns_sensor_uid)GYRO_SUID, true) &&
#if BMI26X_CONFIG_ENABLE_PEDO
            NULL == inst->cb->get_client_request(inst,
                    &(sns_sensor_uid)PEDO_SUID, true) &&
#endif
#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            NULL == inst->cb->get_client_request(inst,
                    &(sns_sensor_uid)DOUBLE_TAP_SENSOR_SUID, true) &&
#endif
            NULL == inst->cb->get_client_request(inst,
                    &(sns_sensor_uid)SENSOR_TEMPERATURE_SUID, true))  {
        return false;
    } else {
        return true;
    }
}


static void bmi26x_sensor_remove_request(
    sns_sensor               *const this,
    sns_sensor_instance      *inst,
    struct sns_request const *exist_request)
{
    if (NULL != inst) {
        bmi26x_state *sstate = (bmi26x_state*)this->state->state;
        inst->cb->remove_client_request(inst, exist_request);

        /* Assumption: The FW will call deinit() on the instance before destroying it.
           Putting all HW resources (sensor HW, COM port, power rail)in
           low power state happens in Instance deinit().*/
        if (exist_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
            BMI26X_SENSOR_LOG(MED, this, "remove request on physical test on present @ss:%d",
                              sstate->sensor);
        } else {
            BMI26X_SENSOR_LOG(MED, this, "cfg present:%d, pwr.st:%d",
                    bmi26x_sensor_is_request_present(inst),
                    bmi26x_sensor_pwr_rail_state_available(this, inst));

            if (!bmi26x_sensor_is_request_present(inst) &&
                    !bmi26x_sensor_pwr_rail_state_available(this, inst)) {
                bmi26x_sensor_clear_timer_stream(this);
                BMI26X_SENSOR_LOG(MED, this, "h/w not cfg yet, do nothing");
                // remove the time stream which pending init
            } else if (bmi26x_sensor_pwr_rail_state_available(this, inst)) {
                bmi26x_reval_instance_config(this, inst);
            }
        }
    }
}

/** See sns_bmi26x_sensor.h */
sns_sensor_instance* bmi26x_set_client_request(
    sns_sensor          *const this,
    struct sns_request  const *exist_request,
    struct sns_request  const *new_request,
    bool                remove)
{
    sns_sensor_instance     *instance = sns_sensor_util_get_shared_instance(this);
    bmi26x_state            *sstate = (bmi26x_state*)this->state->state;
    bmi26x_instance_state   *istate = NULL;

    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_sif_ set_client_request> snr:%d, ss:%p",
                      sstate->sensor, sstate);
    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_sif_ set_client_request> req(old/new):0x%x/0x%x, req.msgid(old/new):%d/%d rm:%d",
                      exist_request, new_request,
                      exist_request != NULL ? exist_request->message_id : -1,
                      new_request != NULL ? new_request->message_id : -1,
                      remove);

    if (remove && (NULL != instance)) {
        BMI26X_SENSOR_LOG(LOW, this, "remove exist_request");
        //if (bmi26x_sensor_pwr_rail_state_available(instance)) {
            bmi26x_sensor_remove_request(this, instance, exist_request);
        //}
    } else if (NULL != new_request) {
        // 1. If new request then:
        //     a. Power ON rails.
        //     b. Power ON COM port - Instance must handle COM port power.
        //     c. Create new instance.
        //     d. Re-evaluate existing requests and choose appropriate instance config.
        //     e. set_client_config for this instance.
        //     f. Add new_request to list of requests handled by the Instance.
        //     g. Power OFF COM port if not needed- Instance must handle COM port power.
        //     h. Return the Instance.
        // 2. If there is an Instance already present:
        //     a. Add new_request to list of requests handled by the Instance.
        //     b. Remove exist_request from list of requests handled by the Instance.
        //     c. Re-evaluate existing requests and choose appropriate Instance config.
        //     d. set_client_config for the Instance if not the same as current config.
        //     e. publish the updated config.
        //     f. Return the Instance.
        // 3.  If "flush" request:
        //     a. Perform flush on the instance.
        //     b. Return NULL.

        if ((NULL == instance) &&
                SNS_STD_MSGID_SNS_STD_FLUSH_REQ != new_request->message_id) {
            // first request cannot be a Flush request or Calibration reset request
            instance = bmi26x_create_new_instance(this);
        }

        /** Add the new request to list of client_requests.*/
        if (NULL != instance) {
            istate = (bmi26x_instance_state *)instance->state->state;

            if (!istate->fac_test_in_progress) {
                if (new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) { // most frequent request
                    BMI26X_SENSOR_LOG(MED, this, "flush req: ss.en.curr:0x%x@%u",
                                istate->fifo_info.ff_sensors_en_curr,
                                          BMI26X_SYS_TIME_LH(sns_get_system_time()));

                    if (NULL == exist_request) {
                        BMI26X_SENSOR_LOG(ERROR, this, "WARN!!! orphan flush req!");
                    }
                    bmi26x_handle_flush_request(this, instance);
                } else if (new_request->message_id != SNS_CAL_MSGID_SNS_CAL_RESET) {
                    bmi26x_handle_new_request(this, instance, exist_request, new_request);
                } else {// if (new_request->message_id == SNS_CAL_MSGID_SNS_CAL_RESET)
                    bmi26x_sensor_handle_cal_reset_req(this, instance);
                }
            } else {
                BMI26X_SENSOR_LOG(HIGH, this, "WARNING!!! selftest still running. Reject");
                instance = NULL;
            }
        } else {
            BMI26X_SENSOR_LOG(HIGH, this, "WARNING!!! inst is NULL");
        }
    } else { //bad request
        instance = NULL;
    }

#if BMI26X_CONFIG_ENABLE_REGISTRY
    if (NULL != instance) {
        istate = (bmi26x_instance_state*)instance->state->state;
        bool reg_updated = false;
        BMI26X_SENSOR_LOG(HIGH, this, "NORICE! should update registry:%d",
                istate->update_fac_cal_in_registry);
        if (istate->update_fac_cal_in_registry) {
            bmi26x_sensor_exit_island(this);

            // XXX update base on the current request sensor, it suppose the client on current sensor
            // removed after the sensor fact test finish
            if (sstate->sensor == BMI26X_ACCEL || istate->accel_info.test_info.test_client_present) {
                bmi26x_update_registry(this, instance, BMI26X_ACCEL);
                istate->accel_info.test_info.test_client_present = 0;
                reg_updated = true;
            }

            if (sstate->sensor == BMI26X_GYRO || istate->gyro_info.test_info.test_client_present) {
                bmi26x_update_registry(this, instance, BMI26X_GYRO);
                istate->gyro_info.test_info.test_client_present = 0;
                reg_updated = true;
            }

            if (reg_updated) {
                istate->update_fac_cal_in_registry = false;
            }
        }

#if BMI26X_CONFIG_ENABLE_CRT
        if (istate->pending_update_crt_in_registry) {
            bmi26x_sensor_exit_island(this);

            bmi26x_crt_update_registry(this, instance, BMI26X_GYRO);
            istate->pending_update_crt_in_registry = false;
        }
#endif
#endif
    }

    if (NULL != instance && !bmi26x_sensor_is_request_present(instance)) {
        BMI26X_SENSOR_LOG(HIGH, this, "instance has no clients, will be removed");
        // QC: must call remove_instance() when clientless
        this->cb->remove_instance(instance);
        instance = NULL;

        bmi26x_sensor_pwr_rail_cleanup(this);
    }

    if (NULL == instance && NULL != new_request &&
            SNS_CAL_MSGID_SNS_CAL_RESET == new_request->message_id) {
        instance = &sns_instance_no_error;
    }

    return instance;
}



sns_sensor_api bmi26x_accel_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_accel_init,
    .deinit             = bmi26x_accel_deinit,
    .get_sensor_uid     = bmi26x_accel_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

sns_sensor_api bmi26x_gyro_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_gyro_init,
    .deinit             = bmi26x_gyro_deinit,
    .get_sensor_uid     = bmi26x_gyro_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

sns_sensor_api bmi26x_motion_detect_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_motion_detect_init,
    .deinit             = bmi26x_motion_detect_deinit,
    .get_sensor_uid     = bmi26x_motion_detect_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

sns_sensor_api bmi26x_sensor_temp_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_sensor_temp_init,
    .deinit             = bmi26x_sensor_temp_deinit,
    .get_sensor_uid     = bmi26x_sensor_temp_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

#if BMI26X_CONFIG_ENABLE_PEDO
sns_sensor_api bmi26x_pedo_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_pedo_init,
    .deinit             = bmi26x_pedo_deinit,
    .get_sensor_uid     = bmi26x_pedo_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};
#endif

#if BMI26X_CONFIG_ENABLE_OIS
sns_sensor_api bmi26x_ois_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_ois_init,
    .deinit             = bmi26x_ois_deinit,
    .get_sensor_uid     = bmi26x_ois_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

#endif


#if BMI26X_CONFIG_ENABLE_LOWG
sns_sensor_api bmi26x_free_fall_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_free_fall_init,
    .deinit             = bmi26x_free_fall_deinit,
    .get_sensor_uid     = bmi26x_free_fall_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
sns_sensor_api bmi26x_double_tap_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = bmi26x_double_tap_init,
    .deinit             = bmi26x_double_tap_deinit,
    .get_sensor_uid     = bmi26x_double_tap_get_sensor_uid,
    .set_client_request = bmi26x_set_client_request,
    .notify_event       = bmi26x_sensor_notify_event,
};

#endif

