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

#include "sns_bmi160_sensor.h"

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

static sns_sensor_uid const* bmi160_accel_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = ACCEL_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi160_gyro_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = GYRO_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi160_motion_detect_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = MOTION_DETECT_SUID;

    return &sensor_uid;
}

static sns_sensor_uid const* bmi160_sensor_temp_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = SENSOR_TEMPERATURE_SUID;

    return &sensor_uid;
}


#if BMI160_CONFIG_ENABLE_PEDO
static sns_sensor_uid const* bmi160_pedo_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = STEP_COUNTER_SUID;

    return &sensor_uid;
}
#endif


static void bmi160_sensor_exit_island(sns_sensor *const this)
{
#if BMI160_CONFIG_ENABLE_ISLAND_MODE

  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_island_service  *island_svc  =
    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
  island_svc->api->sensor_island_exit(island_svc, this);
#else
    UNUSED_VAR(this);
#endif
}

/**
 *
 * @param this
 * @param instance
 * @param registry_cfg
 * @param message_id
 */
//called in sensor_island.c::set_client_request()/notify_event()
void bmi160_set_inst_config(
        sns_sensor                  *this,
        sns_sensor_instance         *instance,
        bmi160_req_payload          req_payload,
        uint32_t                    message_id)
{
    sns_request                     config;

    config.message_id = message_id;
    config.request_len = sizeof(req_payload);
    config.request = &req_payload;

    this->instance_api->set_client_config(instance, &config);
}


#if BMI160_CONFIG_POWER_RAIL
sns_rc bmi160_start_power_rail_timer(sns_sensor *const this,
        sns_time timeout_ticks,
        bmi160_power_rail_pending_state pwr_rail_pend_state)
{
    bmi160_state *sstate = (bmi160_state*)this->state->state;
    sns_rc rc = SNS_RC_SUCCESS;

    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t req_len;
    uint8_t buffer[20];
    sns_memset(buffer, 0, sizeof(buffer));
    req_payload.is_periodic = false;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = timeout_ticks;

    if(NULL == sstate->timer_stream)
    {
        sns_service_manager *smgr = this->cb->get_service_manager(this);
        sns_stream_service  *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
        sns_sensor_uid      suid;

        sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", &suid);


        rc = stream_svc->api->create_sensor_stream(stream_svc, this, suid, &sstate->timer_stream);
        if (rc == SNS_RC_SUCCESS) {
        } else {
            BMI160_SENSOR_LOG(HIGH, this, "timer stream create failure");
            return rc;
        }
    }

    req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
            sns_timer_sensor_config_fields, NULL);
    if(req_len > 0 && NULL != sstate->timer_stream)
    {
        sns_request timer_req =
        {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
            .request = buffer, .request_len = req_len  };
        sstate->timer_stream->api->send_request(sstate->timer_stream, &timer_req);
        sstate->power_rail_pend_state = pwr_rail_pend_state;

        BMI160_SENSOR_LOG(MED, this, "timer requested by: %d with %d",
                        sstate->sensor, pwr_rail_pend_state);
    }
    else
    {
        BMI160_SENSOR_LOG(ERROR, this, "timer req encode error");
    }

    return rc;
}
#endif

static void bmi160_sensor_pwr_rail_cleanup(sns_sensor *this)
{
#if BMI160_CONFIG_ENABLE_FORCE_DEINIT_DELAY
    sns_time ts_pending_deinit = bmi160_convert_us2ticks(BMI160_DELAY_DEINIT_IN_MS * 1000);
    if (bmi160_start_power_rail_timer(this, ts_pending_deinit, BMI160_POWER_RAIL_PENDING_DEINIT) != SNS_RC_SUCCESS) {
        bmi160_turn_rails_off(this);
    }
#else
    bmi160_turn_rails_off(this);
#endif
}

void bmi160_turn_rails_off(sns_sensor *this)
{
#if BMI160_CONFIG_POWER_RAIL
    sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
    if (NULL == instance) {
        bmi160_sensor_exit_island(this);
        bmi160_sensor_turn_rails_off(this);
    }
#endif
}

static void bmi160_sensor_check_pending_flush_requests(
  sns_sensor           *this,
  sns_sensor_instance  *instance)
{
    sns_sensor *lib_sensor;
    for(lib_sensor = this->cb->get_library_sensor(this, true);
      NULL != lib_sensor;
      lib_sensor = this->cb->get_library_sensor(this, false)) {
        bmi160_state *sstate = (bmi160_state*)lib_sensor->state->state;
        if(sstate->ss_flush_req) {
            sstate->ss_flush_req = false;
            BMI160_SENSOR_LOG(MED, this, "pending_flush_requests: sensor=%u", sstate->sensor);
            bmi160_send_fifo_flush_done(instance, sstate->sensor,
                                            BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_REQ);
        }
    }
}


/*!
 * Check power state available
 * @param inst  instance handler
 * @return true if the power state is available
 *         false if the power state is not ready
 */
static bool bmi160_sensor_pwr_rail_state_available(sns_sensor *const this, sns_sensor_instance *inst)
{
    bmi160_instance_state   *istate = (bmi160_instance_state*) inst->state->state;

   bool pwr_available = true;
   sns_sensor *lib_sensor;

   for(lib_sensor = this->cb->get_library_sensor(this, true);
       NULL != lib_sensor;
       lib_sensor = this->cb->get_library_sensor(this, false)) {
       bmi160_state *lib_state = (bmi160_state*)lib_sensor->state->state;
       pwr_available &= ((lib_state->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_NONE) ||
                        (lib_state->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_DEINIT));

       BMI160_INST_LOG(MED, inst, "pwr.rail.st chk: ss %d@%d %d", lib_state->sensor,
                       lib_state->power_rail_pend_state, pwr_available);
   } //@

   if (istate->pwr_state_present == BMI160_POWER_RAIL_PENDING_NONE) {
       //pwr_available |= true;
   }
   BMI160_INST_LOG(MED, inst, "pwr.rail.st chk: ss %d", pwr_available);

   return pwr_available;
}


/** See sns_bmi160_sensor.h*/
sns_rc bmi160_sensor_notify_event(sns_sensor *const this)
{
    bmi160_state            *sstate = (bmi160_state*)this->state->state;
    sns_rc                  rv = SNS_RC_SUCCESS;
    sns_sensor_event        *event;


    BMI160_SENSOR_LOG(MED, this, "<bmi160_if_ notify_event: %d this: 0x%x>", sstate->sensor, this);
#if BMI160_CONFIG_ENABLE_DAE
    BMI160_SENSOR_LOG(MED, this, "dae_sensor_events: state(ag)=%d state(t): %d",
                      sstate->common.dae_ag_state,
                      sstate->common.dae_temper_state)
#endif

    if (!sns_suid_lookup_complete(&sstate->common.suid_lookup_data))
    {
        bmi160_sensor_exit_island(this);
        sns_suid_lookup_handle(this, &sstate->common.suid_lookup_data);

#if BMI160_CONFIG_ENABLE_REGISTRY
        if(sns_suid_lookup_get(&sstate->common.suid_lookup_data, "registry", NULL))
        {
            bmi160_request_registry(this);
        }
#else
        sns_bmi160_registry_def_config(this);
#endif
        BMI160_SENSOR_LOG(LOW, this, "snr: %d r_idx=%d", sstate->sensor, sstate->resolution_idx);

        if (sns_suid_lookup_complete(&sstate->common.suid_lookup_data))
        {
            sns_suid_lookup_deinit(this, &sstate->common.suid_lookup_data);
        }
    }
    else
    {
        BMI160_SENSOR_LOG(LOW, this, "look up complete");
    }

#if BMI160_CONFIG_ENABLE_FORCE_DEINIT_DELAY
    BMI160_SENSOR_LOG(MED, this, "pwr.state:%d on %p",
                    sstate->power_rail_pend_state,
                    sstate->timer_stream);
#endif

    /**----------------------Handle a Timer Sensor event.-------------------*/
    if (NULL != sstate->timer_stream)
    {
        event = sstate->timer_stream->api->peek_input(sstate->timer_stream);
        while(NULL != event)
        {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                    event->event_len);
            sns_timer_sensor_event timer_event;
            if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
            {

                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
                {
#if BMI160_CONFIG_POWER_RAIL
                    if (sstate->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_INIT)
                    {
                        bmi160_sensor_exit_island(this);
                        sstate->common.hw_is_present = bmi160_discover_hw(this);
                        if (sstate->common.hw_is_present)
                        {
                            BMI160_SENSOR_LOG(MED, this, "sensor:%d init finished", sstate->sensor);
                            //bmi160_publish_available(this);
                            bmi160_update_sibling_sensors(this);
#if BMI160_CONFIG_ENABLE_DAE
                            if (sstate->sensor == BMI160_ACCEL) {
                                bmi160_dae_if_check_support(this);
                            }
#endif
                        }
                        else
                        {
                            rv = SNS_RC_INVALID_LIBRARY_STATE;
                            BMI160_SENSOR_LOG(ERROR, this, "BMI160 HW absent");
                        }
                        sstate->power_rail_pend_state = BMI160_POWER_RAIL_PENDING_NONE;
                    }
                    else if (sstate->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_SET_CLIENT_REQ)
                    {
                        sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);

                        BMI160_SENSOR_LOG(LOW, this, "timer event for instance: %p set_client_request snr: %d", instance, sstate->sensor);
                        if (NULL != instance)
                        {
                            bmi160_instance_state *istate =
                                (bmi160_instance_state*) instance->state->state;
                            istate->instance_is_ready_to_configure = true;
                            bmi160_req_payload          req_payload = {.sensor_type = sstate->sensor};

                            bmi160_sensor_exit_island(this);
                            bmi160_reval_instance_config(this, instance, sstate->sensor);

                            if (istate->new_self_test_request)
                            {
                                bmi160_set_inst_config(this, instance,
                                req_payload,
                                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG);
                            }
#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
                            if(istate->new_self_test_request_bias)
                            {
                                bmi160_set_inst_config(this, instance,
                                req_payload,
                                SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG);
                            }
#endif
                        }
                        sstate->power_rail_pend_state = BMI160_POWER_RAIL_PENDING_NONE;
                        bmi160_sensor_check_pending_flush_requests(this, instance);
                    }
                    else if (sstate->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_DEINIT)
                    {
                        BMI160_SENSOR_LOG(HIGH, this, "deinit power rail now:%u",
                                          (uint32_t)sns_get_system_time());
                        bmi160_turn_rails_off(this);
                        sstate->power_rail_pend_state = BMI160_POWER_RAIL_PENDING_NONE;
                    }
#endif  //BMI160_CONFIG_POWER_RAIL
                }
                else
                {
                    BMI160_SENSOR_LOG(ERROR, this, "pb_decode error");
                }
            }

            event = sstate->timer_stream->api->get_next_input(sstate->timer_stream);
        }

        /** Free up timer stream if not needed anymore */
        if (sstate->power_rail_pend_state == BMI160_POWER_RAIL_PENDING_NONE)
        {
            sns_sensor_util_remove_sensor_stream(this, &sstate->timer_stream);
            sstate->timer_stream = NULL;
        }
    }

#if BMI160_CONFIG_ENABLE_REGISTRY
    if (NULL != sstate->reg_data_stream)
    {
        event = sstate->reg_data_stream->api->peek_input(sstate->reg_data_stream);
        while (NULL != event)
        {
            bmi160_sensor_exit_island(this);
            bmi160_sensor_process_registry_event(this, event);

            event = sstate->reg_data_stream->api->get_next_input(sstate->reg_data_stream);
        }

        bmi160_sensor_exit_island(this);
        bmi160_sensor_check_registry_col_progress(this);
    }
#endif

    // DAE
    bmi160_dae_if_process_sensor_events(this);

    //note: all the registry items must be provided
    if (sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", NULL) &&
            sstate->common.registry_pf_cfg_received && sstate->registry_cfg_received &&
            sstate->common.registry_orient_received && sstate->common.registry_placement_received)
    {
        if ((!sstate->common.hw_is_present) &&
                        (!sstate->common.hw_is_finish_detection)) {
            bmi160_sensor_exit_island(this);
            BMI160_SENSOR_LOG(MED, this, "start_hw_detect_seq snr:%d", sstate->sensor);
            bmi160_start_hw_detect_sequence(this);
        } else {
            if (sstate->common.hw_is_finish_detection) {
                BMI160_SENSOR_LOG(MED, this, "hw detection was finished on sensor %d",
                                  sstate->sensor);
            }
        }
    }

    return rv;
}


/** See sns_bmi160_sensor.h */
sns_sensor_instance* bmi160_set_client_request(
        sns_sensor          *const this,
        struct sns_request  const *exist_request,
        struct sns_request  const *new_request,
        bool                remove)
{
    sns_sensor_instance     *instance = sns_sensor_util_get_shared_instance(this);
    bmi160_state            *sstate = (bmi160_state*)this->state->state;
    bmi160_instance_state   *istate = NULL;
    sns_time                on_timestamp;
    sns_time                delta;
    bool                    reval_config = false;



    BMI160_SENSOR_LOG(MED, this, "<bmi160_sif_ set_client_request> snr:%d",
            sstate->sensor);
    BMI160_SENSOR_LOG(MED, this, "<bmi160_sif_ set_client_request> req:0x%x/0x%x, req.msgid:%d/%d rm:%d",
                      exist_request,
                      new_request,
                      exist_request != NULL ? exist_request->message_id : -1,
                      new_request != NULL ? new_request->message_id : -1,
                      remove);

    if (remove)
    {
        if (NULL != instance)
        {
            BMI160_SENSOR_LOG(LOW, this, "remove exist_request");
            instance->cb->remove_client_request(instance, exist_request);
            /* Assumption: The FW will call deinit() on the instance before destroying it.
               Putting all HW resources (sensor HW, COM port, power rail)in
               low power sstate happens in Instance deinit().*/
            bmi160_sensor_exit_island(this);
            bmi160_reval_instance_config(this, instance, sstate->sensor);

        }
    }
    else
    {
        //false == remove
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

        if (NULL == instance)
        {
#if BMI160_CONFIG_POWER_RAIL
            if (sstate->sensor == BMI160_GYRO)
            {
                sstate->common.rail_config.rail_vote = SNS_RAIL_ON_NPM;
            }
            else
            {
                //TOCHECK
                sstate->common.rail_config.rail_vote = sstate->common.registry_rail_on_state;
            }
            sstate->pwr_rail_service->api->sns_vote_power_rail_update(sstate->pwr_rail_service,
                                                                      this,
                                                                      &sstate->common.rail_config,
                                                                      &on_timestamp);

            BST_ASSUME_TS_IS_64BIT
            delta = sns_get_system_time() - on_timestamp;

            // Use on_timestamp to determine correct Timer value.
            if (delta < sns_convert_ns_to_ticks(BMI160_OFF_TO_IDLE_MS * 1000 * 1000))
            {
                bmi160_start_power_rail_timer(this,
                        sns_convert_ns_to_ticks(BMI160_OFF_TO_IDLE_MS * 1000 * 1000) - delta,
                        BMI160_POWER_RAIL_PENDING_SET_CLIENT_REQ);
            }
            else
            {
                // rail is already ON for time long enough
                reval_config = true;
                BMI160_SENSOR_LOG(MED, this, "rail already on: %u %u", BMI160_SYS_TIME_LH(on_timestamp), BMI160_SYS_TIME_LH(delta));
            }
#else
            UNUSED_VAR(on_timestamp);
            UNUSED_VAR(delta);
            reval_config = true;
#endif

            /** create_instance() calls init() for the Sensor Instance */
            instance = this->cb->create_instance(this,
                    sizeof(bmi160_instance_state));

            istate = (bmi160_instance_state*)instance->state->state;
            BMI160_SENSOR_LOG(LOW, this, "instance created: 0x%x istate:0x%x, 0x%x", instance, istate, bmi160_inst_singleton);
            if (NULL == bmi160_inst_singleton) {
                bmi160_inst_singleton = istate;
            }
            /* If rail is already ON then flag instance OK to configure */
            if (reval_config)
            {
                istate->instance_is_ready_to_configure = true;
            }
        }
        else
        {
            if (NULL != new_request
                    &&
                    new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
            {
                istate = (bmi160_instance_state*)instance->state->state;
                bmi160_fifo_flush_ctx_t  flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_REQ;

                if (NULL == exist_request) {
                    BMI160_SENSOR_LOG(ERROR, this, "WARN!!! Orphan flush req");
                    instance = NULL;
                } else {

                BMI160_SENSOR_LOG(MED, this, "flush req: %d@%u 0x%x, req.num:<%d %d>",
                                sstate->sensor,
                                BMI160_SYS_TIME_LH(sns_get_system_time()),
                                istate->fifo_info.ff_sensors_en_curr,
                                istate->flush_req_on_acc,
                                istate->flush_req_on_gyr);
#if BMI160_CONFIG_POWER_RAIL
    bool pwr_state_available = bmi160_sensor_pwr_rail_state_available(this, instance);

    if (pwr_state_available == false) {
#if 1
        flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW;

        sstate->ss_flush_req = true;
        BMI160_SENSOR_LOG(MED, this, "flush req111: %d@%d %d",
                sstate->sensor,
                flush_reason, sstate->ss_flush_req);
#endif
    } else
#endif
#if BMI160_CONFIG_FLUSH_OPTIMIZATION
                if (sstate->sensor & (BMI160_MOTION_DETECT
                                      | BMI160_SENSOR_TEMP
#if BMI160_CONFIG_ENABLE_PEDO
                                      | BMI160_PEDO
#endif
                                      )) {
                    flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_NONE_FIFO_SENSOR_RES_NOW;
                } else {
                    if (istate->fifo_info.ff_sensors_en_curr) {
                    } else {
                        flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW;
                    }
                }
#else
                if (istate->fifo_info.ff_sensors_en_curr) {  // ONLY care about the current FIFO enable sensors
                    flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_REQ;
                    if (sstate->sensor == BMI160_ACCEL) {
                        istate->flush_req_on_acc ++;
                    } else if (sstate->sensor == BMI160_GYRO) {
                        istate->flush_req_on_gyr ++;
                    } else {
                        //TODO MAG, others
                        flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW;
                    }
                } else {
                    flush_reason = BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW;
                }
#endif

                if (flush_reason == BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_REQ) {
                    //TOCHECK
                    bmi160_req_payload          req_payload = {.sensor_type = sstate->sensor};

                    bmi160_set_inst_config(this, instance, req_payload, SNS_STD_MSGID_SNS_STD_FLUSH_REQ);
                    /** Do not update instance client request list at this point
                      because FIFO flush is a transitory request for an on-going
                      stream request. */
                    return instance;
                    } else if (sstate->ss_flush_req != true){
                            BMI160_SENSOR_LOG(MED, this, "flush req222: %d@%d %d",sstate->sensor,
                            flush_reason, sstate->ss_flush_req);
                    /** There aren't any FIFO sensors enabled to support flush */
                    /*  Send flush complete event anyway. */
                    bmi160_send_fifo_flush_done(instance, sstate->sensor, BMI160_FIFO_FLUSH_DONE_CTX_CLIENT_RES_NOW);
                    return NULL;
                }
                }
                // 
            }
            else
            {
                reval_config = true;

                /** An existing client is changing request*/
                if ((NULL != exist_request) && (NULL != new_request))
                {
                    BMI160_SENSOR_LOG(LOW, this, "remove exist_request due to change of req");
                    instance->cb->remove_client_request(instance, exist_request);
                }
                /** A new client sent new_request*/
                else if (NULL != new_request)
                {
                    // No-op. new_request will be added to requests list below.
                }
            }
        }

        /** Add the new request to list of client_requests.*/
        if (NULL != instance && (new_request->message_id != SNS_STD_MSGID_SNS_STD_FLUSH_REQ))
        {
            istate = (bmi160_instance_state*)instance->state->state;
            if (NULL != new_request)
            {
                instance->cb->add_client_request(instance, new_request);

                if ((new_request->message_id ==
                            SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
                        &&
                        (BMI160_MOTION_DETECT == sstate->sensor))
                {
                    bmi160_sensor_exit_island(this);
                    if (!bmi160_check_n_approve_md_req(sstate, istate)) {
                        reval_config = false;
                    }
                }

                if (new_request->message_id == SNS_CAL_MSGID_SNS_CAL_RESET) {
                    bool    op_valid = false;

                    //TODO2: how about temperature
                    if ((BMI160_ACCEL == sstate->sensor) || (BMI160_GYRO == sstate->sensor)) {
                        op_valid = true;
                    }

                    BMI160_SENSOR_LOG(MED, this, "Request for resetting cal data: %d");
                    if (op_valid) {
                        bmi160_sensor_exit_island(this);

                        bmi160_reset_fac_cal_data(instance, sstate, true);

#if BMI160_CONFIG_ENABLE_REGISTRY
                        bmi160_update_registry(this, instance, sstate->sensor);
#endif
                        bmi160_send_fac_cal_event(instance, sstate);
                    } else {
                    }

                    reval_config = false;
                }

                if (new_request->message_id ==
                        SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
                {
                    bmi160_sensor_exit_island(this);

                    if (bmi160_extract_self_test_info(this, instance, new_request))
                    {
                        istate->new_self_test_request = true;
                    }
                } else {
                }
#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
                if (new_request->message_id ==
                        SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG)
                {
                    BMI160_SENSOR_LOG(MED, this, "SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG is called");
                    bmi160_sensor_exit_island(this);

                    if (bmi160_extract_self_test_info_bias(this, instance, new_request))
                    {
                        istate->new_self_test_request_bias = true;
                    }
                }
#endif
            }

            if (reval_config && istate->instance_is_ready_to_configure)
            {
                bmi160_sensor_exit_island(this);

                bmi160_reval_instance_config(this, instance, sstate->sensor);

                if (istate->new_self_test_request)
                {
                    bmi160_req_payload          req_payload = {.sensor_type = sstate->sensor};
                    bmi160_set_inst_config(this, instance,
                    req_payload,
                    SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG
                    );
                }

#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
                if (istate->new_self_test_request_bias)
                {
                    bmi160_req_payload          req_payload = {.sensor_type = sensor_type};
                    bmi160_set_inst_config(this, instance,
                    SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG
                    );
                }
#endif
            }
        }
    }

#if BMI160_CONFIG_ENABLE_REGISTRY
    if ((NULL != instance) && remove) {
        bool reg_updated = false;
        istate = (bmi160_instance_state*)instance->state->state;
        if (istate->update_fac_cal_in_registry)
        {
            bmi160_sensor_exit_island(this);
            if(sstate->sensor == BMI160_ACCEL) {
                bmi160_update_registry(this, instance, BMI160_ACCEL);
                reg_updated = true;
                BMI160_SENSOR_LOG(MED, this, "bmi160_update_registry BMI160_ACCEL,remove ");
            }else if(sstate->sensor == BMI160_GYRO) {
                bmi160_update_registry(this, instance, BMI160_GYRO);
                reg_updated = true;
                BMI160_SENSOR_LOG(MED, this, "bmi160_update_registry BMI160_GYRO remove");
            }else {
                BMI160_SENSOR_LOG(MED, this, "bmi160_update_registry error %d", sstate->sensor);
            }
            if(reg_updated == true) {
               istate->update_fac_cal_in_registry = false;
            }
       }
    }
#endif

    // QC: Sensors are required to call remove_instance when clientless
    if (NULL != instance &&
            NULL == instance->cb->get_client_request(instance,
                &(sns_sensor_uid)MOTION_DETECT_SUID, true) &&
            NULL == instance->cb->get_client_request(instance,
                &(sns_sensor_uid)ACCEL_SUID, true) &&
            NULL == instance->cb->get_client_request(instance,
                &(sns_sensor_uid)GYRO_SUID, true) &&
#if BMI160_CONFIG_ENABLE_PEDO
            NULL == instance->cb->get_client_request(instance,
                &(sns_sensor_uid)STEP_COUNTER_SUID, true) &&
#endif
            NULL == instance->cb->get_client_request(instance,
                &(sns_sensor_uid)SENSOR_TEMPERATURE_SUID, true))
    {
        BMI160_SENSOR_LOG(MED, this, "instance has no clients, will be removed");

        this->cb->remove_instance(instance);
        instance = NULL;

        bmi160_sensor_pwr_rail_cleanup(this);
    }

    return instance;
}




sns_sensor_api bmi160_accel_sensor_api =
{
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &bmi160_accel_init,
    .deinit             = &bmi160_accel_deinit,
    .get_sensor_uid     = &bmi160_accel_get_sensor_uid,
    .set_client_request = &bmi160_set_client_request,
    .notify_event       = &bmi160_sensor_notify_event,
};

sns_sensor_api bmi160_gyro_sensor_api =
{
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &bmi160_gyro_init,
    .deinit             = &bmi160_gyro_deinit,
    .get_sensor_uid     = &bmi160_gyro_get_sensor_uid,
    .set_client_request = &bmi160_set_client_request,
    .notify_event       = &bmi160_sensor_notify_event,
};

sns_sensor_api bmi160_motion_detect_sensor_api =
{
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &bmi160_motion_detect_init,
    .deinit             = &bmi160_motion_detect_deinit,
    .get_sensor_uid     = &bmi160_motion_detect_get_sensor_uid,
    .set_client_request = &bmi160_set_client_request,
    .notify_event       = &bmi160_sensor_notify_event,
};

sns_sensor_api bmi160_sensor_temp_sensor_api =
{
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &bmi160_sensor_temp_init,
    .deinit             = &bmi160_sensor_temp_deinit,
    .get_sensor_uid     = &bmi160_sensor_temp_get_sensor_uid,
    .set_client_request = &bmi160_set_client_request,
    .notify_event       = &bmi160_sensor_notify_event,
};

#if BMI160_CONFIG_ENABLE_PEDO
sns_sensor_api bmi160_pedo_api =
{
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &bmi160_pedo_init,
    .deinit             = &bmi160_pedo_deinit,
    .get_sensor_uid     = &bmi160_pedo_get_sensor_uid,
    .set_client_request = &bmi160_set_client_request,
    .notify_event       = &bmi160_sensor_notify_event,
};
#endif

