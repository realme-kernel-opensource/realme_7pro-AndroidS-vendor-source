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

#include "sns_island_service.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_bmi160_hal.h"
#include "sns_bmi160_sensor.h"
#include "sns_bmi160_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_std_event_gated_sensor.pb.h"

#if BMI160_CONFIG_ENABLE_DIAG_LOG
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#endif

#include "sns_sync_com_port_service.h"
#include "sns_cal_util.h"
#include "sns_sensor_util.h"


static void bmi160_inst_exit_island(sns_sensor_instance *this)
{
#if BMI160_CONFIG_ENABLE_ISLAND_MODE
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_island_service  *island_svc  =
                    (sns_island_service *)smgr->get_service(smgr, SNS_ISLAND_SERVICE);
    island_svc->api->sensor_instance_island_exit(island_svc, this);
#else
    UNUSED_VAR(this);
#endif
}





/**
 * Process COM PORT vector event
 * @param vector         the vector handler
 * @param user_arg       user defined arguments
 *
 * @return none
 */
    static
void bmi160_process_com_port_vector(sns_port_vector *vector, void *user_arg)
{
    sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

    if(BMI160_REGA_USR_FIFO_DATA == vector->reg_addr)
    {
        //Vector contains a FIFO buffer read
        if (vector->bytes > 0)
        {
            bmi160_hal_process_fifo_data_buffer(instance,
                    vector->buffer,
                    vector->bytes);
        }
    }
}



sns_rc bmi160_inst_handle_event_int(
        sns_sensor_instance     *const this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state *) this->state->state;
    sns_sensor_event            *event;
    sns_rc                      rc = SNS_RC_SUCCESS;

    sns_interrupt_event         irq_event = sns_interrupt_event_init_zero;
    bmi160_int_check_ctx        ctx;

    // Handle interrupts
    if(NULL != istate->interrupt_data_stream)
    {
        ctx.int_check_trigger = BMI160_INT_CHECK_TRIGGER_IRQ;

        event = istate->interrupt_data_stream->api->peek_input(istate->interrupt_data_stream);
        while(NULL != event)
        {
            if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
            {
                int enable_fifo_stream = istate->fifo_info.publish_sensors & (BMI160_ACCEL | BMI160_GYRO | BMI160_MAG);

                enable_fifo_stream |= istate->fac_test_in_progress;

                BMI160_INST_LOG(MED, this, "irq_ready event id=%d", event->message_id);

                istate->irq_ready = 1;

                if (enable_fifo_stream
                        || istate->int_en_flags_req.bits.md) {
                    rc |= bmi160_hal_config_int_output(this, true, BMI160_INT_PIN1);
                }

                if (istate->md_info.enable_md_int)
                {
                    bmi160_hal_config_int_md(this, true, false);
                }

                if (enable_fifo_stream)
                {
                    bmi160_hal_config_int_fifo(this, true);
                }
            }
            else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
            {
                pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                        event->event_len);


                if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
                {
                    ctx.timestamp = irq_event.timestamp;
                    rc = bmi160_hal_handle_interrupt(this, &ctx);
                }
            }
            else
            {
                BMI160_INST_LOG(ERROR, this, "Received invalid event id=%d", event->message_id);
            }

            event = istate->interrupt_data_stream->api->get_next_input(istate->interrupt_data_stream);
        }
    } else {
        BMI160_INST_LOG(ERROR, this, "WARNING!!! interrupt_data_stream is NULL");
    }

    return SNS_RC_SUCCESS;
}

sns_rc bmi160_inst_handle_event_async_com(
        bmi160_instance_state   *istate)
{
    sns_sensor_instance         *const this = istate->owner;
    sns_sensor_event            *event;

    if(NULL != istate->async_com_port_data_stream)
    {
        bool                    ascp_vector_found = false;

        event = istate->async_com_port_data_stream->api->peek_input(istate->async_com_port_data_stream);
        while (NULL != event)
        {
            if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
            {
                //TODO: Warning;
                BMI160_INST_LOG(
                        ERROR, istate->owner,
                        "Received ASCP error event id=%d",
                        event->message_id);
            }
            else if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
            {
                pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
                sns_ascp_for_each_vector_do(&stream, bmi160_process_com_port_vector, (void *)this);

                ascp_vector_found = true;
            }
            event = istate->async_com_port_data_stream->api->get_next_input(istate->async_com_port_data_stream);
        }

        if (ascp_vector_found) {
            if (istate->fac_test_in_progress)
            {
                bmi160_inst_exit_island(this);
                bmi160_process_fac_test(this);
            } else {
            }
#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
            istate->hb_cfg_info.ts_data_event = sns_get_system_time();
#endif
        }
    }

    return SNS_RC_SUCCESS;
}

static sns_rc bmi160_inst_handle_pedo_timer(
        sns_sensor_instance     *const inst)
{
#if BMI160_CONFIG_ENABLE_PEDO
  bmi160_instance_state *istate = (bmi160_instance_state *) inst->state->state;
  sns_sensor_event *event;

  BMI160_INST_LOG(LOW, inst, "pedo timer, %p",
                  istate->pedo_info.pedo_timer_data_stream);


  if (NULL != istate->pedo_info.pedo_timer_data_stream)
  {
      event = istate->pedo_info.pedo_timer_data_stream->api->peek_input(
              istate->pedo_info.pedo_timer_data_stream);
      while(NULL != event)
      {
          BMI160_INST_LOG(LOW, inst, "timer evt:%d", event->message_id);
          pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                  event->event_len);
          if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
          {
              sns_timer_sensor_event timer_event;
              if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
              {
                  if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
                  {
                      if ((istate->fifo_info.publish_sensors & BMI160_PEDO)
                              &&
                              istate->pedo_info.timer_is_active
                              &&
                              istate->pedo_info.sampling_intvl > 0)
                      {
                        bmi160_hal_handle_pedo(inst, timer_event.timeout_time);
                      }
                  }
              }
          }
          else if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT)
          {
              /** TODO: decode and qse timer_reg_event*/
              BMI160_INST_LOG(LOW, inst, "TIMER_SENSOR_REG_EVENT");
          }
          else
          {
              BMI160_INST_LOG(MED, inst, "unknown message_id %d", event->message_id);
          }
          event = istate->pedo_info.pedo_timer_data_stream->api->get_next_input(
                  istate->pedo_info.pedo_timer_data_stream);
      }

      if ((istate->fifo_info.publish_sensors & BMI160_PEDO) == 0)
      {
        BMI160_INST_LOG(LOW, inst, "pedo disabled");
          //bmi160_hal_stop_tempetature_timer(this);
      }
  }


#else
  UNUSED_VAR(inst);
#endif

  return SNS_RC_SUCCESS;
}



sns_rc bmi160_inst_handle_event_timer(
        bmi160_instance_state   *istate)
{
    sns_sensor_instance         *const this = istate->owner;
    sns_sensor_event            *event;


    if (NULL != istate->timer_data_stream)
    {
        event = istate->timer_data_stream->api->peek_input(istate->timer_data_stream);
        while(NULL != event)
        {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                    event->event_len);
            if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
            {
                sns_timer_sensor_event timer_event;
                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
                {
                    if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
                    {
                        if ((istate->fifo_info.publish_sensors & BMI160_SENSOR_TEMP)
                                &&
                                istate->sensor_temp_info.timer_is_active
                                &&
                                istate->sensor_temp_info.sampling_intvl > 0)
                        {
                            //istate->sensor_temp_info.timer_is_active = false;
                            bmi160_hal_handle_sensor_temp_sample(this);
                            //bmi160_hal_start_sensor_temp_polling_timer(this);
                        }
                    }
                }
            }
            else if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT)
            {
                /** TODO: decode and qse timer_reg_event*/
                BMI160_INST_LOG(LOW, this, "TIMER_SENSOR_REG_EVENT");
            }
            else
            {
                BMI160_INST_LOG(MED, this, "unknown message_id %d", event->message_id);
            }
            event = istate->timer_data_stream->api->get_next_input(istate->timer_data_stream);
        }

        if ((istate->fifo_info.publish_sensors & BMI160_SENSOR_TEMP) == 0)
        {
            //remove timer stream	//FIXLK
            bmi160_hal_stop_tempetature_timer(this);
        }
    }

    if(NULL != istate->cmd_handler.timer_cmd_stream)
    {
        event = istate->cmd_handler.timer_cmd_stream->api->peek_input(istate->cmd_handler.timer_cmd_stream);
        while(NULL != event)
        {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
            sns_timer_sensor_event timer_event;
            if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
            {
                if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
                {
                    bmi160_hal_handle_timer_cmd(istate);
                }
            }
            else
            {
            }
            event = istate->cmd_handler.timer_cmd_stream->api->get_next_input(istate->cmd_handler.timer_cmd_stream);
        }
    }

#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
    if(NULL != istate->hb_cfg_info.timer_heart_beat_data_stream)
    {
        bool event_received = false;
        event = istate->hb_cfg_info.timer_heart_beat_data_stream->api->peek_input(istate->hb_cfg_info.timer_heart_beat_data_stream);
        bmi160_fifo_read_ctx_t      ctx;
        while(NULL != event)
        {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
            sns_timer_sensor_event timer_event;
            if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
                    ctx.ctx_type = BMI160_FIFO_READ_CTX_TYPE_HB;
                    ctx.sync_read = false;
                    ctx.ts_on_ev = timer_event.timeout_time;

                    bmi160_hal_handle_hb_timer_event(istate, &ctx);

                    // handled hb timer time out event
                    event_received = true;
                } else if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG ||
                        event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT){
                    BMI160_INST_LOG(MED, istate->owner, "hb event:%d, ignored", event->message_id);
                }
            } else {
            }
            if (NULL != istate->hb_cfg_info.timer_heart_beat_data_stream) {
                event = istate->hb_cfg_info.timer_heart_beat_data_stream->api->get_next_input(istate->hb_cfg_info.timer_heart_beat_data_stream);
            }
        }

        // postage process
        if (event_received) {
            // HB timer is one-shot timer, need to restart it when it expired
            if (istate->hb_cfg_info.hb_timeout_action_flag == BMI160_HB_TIMER_NEED_RESET_DEVICE) {
                // HB attack heavily
                // 1. reset device
                istate->hb_cfg_info.hb_timeout_cnt = 0;
                bmi160_inst_exit_island(istate->owner);
                if (bmi160_hal_reset_device(istate->owner, false) != SNS_RC_SUCCESS) {
                    BMI160_INST_LOG(HIGH, istate->owner, "error when reset device");
                }
            } else if (istate->hb_cfg_info.hb_timeout_action_flag == BMI160_HB_TIMER_NEED_RECONFIG_HW) {
                // HB attacked many timers
                // 1. reset device
                // 2. reconfigur hw   --> resetart HB timer
                bmi160_inst_exit_island(istate->owner);
                if (bmi160_hal_reset_device(istate->owner, false) == SNS_RC_SUCCESS) {
                    // XXX suppose clients will re-send all the requests
                    istate->config_step = BMI160_CONFIG_IDLE ;
                    if (bmi160_hal_reconfig_hw(istate->owner, BMI160_HW_CFG_CLIENT_HB_ATTACK_REQ) != SNS_RC_SUCCESS) {
                        BMI160_INST_LOG(HIGH, istate->owner, "error when recfg hw");
                    }
                }
            } else if (istate->hb_cfg_info.hb_timeout_action_flag == BMI160_HB_TIMER_FLUSH) {
                // HB attacked several times
                // 1. clear async read request/response flags
                // 2. flush fifo data
                // 3. restart HB timer
                bmi160_fifo_read_ctx_t ctx;
                ctx.ctx_type = BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER;
                ctx.sync_read = false;
                ctx.ts_on_ev = bmi160_get_sys_tick();
                istate->async_com_read_request = 0;
                istate->async_com_read_response = 0;
                bmi160_hal_fifo_frame_read_out(istate, &ctx);

                if (istate->int_en_flags_curr.bits.fifo.flag & (BMI160_ACCEL | BMI160_GYRO)) {
                    //bmi160_remove_hb_timer(istate->owner);
                    //bmi160_restart_hb_timer(istate->owner, false);
                }
            } else {
                // HB timeout or HB attack occasionally
                // 1. restart HB timer
                if (istate->int_en_flags_curr.bits.fifo.flag & (BMI160_ACCEL | BMI160_GYRO)) {
                    //bmi160_remove_hb_timer(istate->owner);
                    //bmi160_restart_hb_timer(istate->owner, false);
                }
            }
        }

        if (istate->fifo_info.publish_sensors & (BMI160_ACCEL | BMI160_GYRO)) {
        } else {
            // no ds processing
            bmi160_remove_hb_timer(istate->owner);
        }
    }
#endif

    bmi160_inst_handle_pedo_timer(this);

    return SNS_RC_SUCCESS;
}


void bmi160_hal_reveal_client_config_wrapper(sns_sensor_instance *inst,
                                             bmi160_hw_cfg_ctx_t hw_cfg_ctx,
                                             bool exit_from_island
                                             )
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)inst->state->state;
    if (exit_from_island) {
        bmi160_inst_exit_island(inst);
    }
    bmi160_inst_assess_overall_req(istate);
    bmi160_hal_reconfig_hw(inst, hw_cfg_ctx);
}

bmi160_state* bmi160_inst_get_master_sensor_state(
    sns_sensor_state const      *caller_sensor_state)
{
    bmi160_state           *sstate = (bmi160_state *) caller_sensor_state->state;
    sns_sensor             *sensor_this = sstate->owner;
    sns_sensor             *sensor;
    bmi160_state           *sensor_state = NULL;

    if (sstate->sensor == BMI160_ACCEL) {
        return sstate;
    }

    for (sensor = sensor_this->cb->get_library_sensor(sensor_this, true);
            sensor != NULL;
            sensor = sensor_this->cb->get_library_sensor(sensor_this, false)) {
        sensor_state = (bmi160_state*)sensor->state->state;
        if (BMI160_ACCEL == sensor_state->sensor) {
            return sensor_state;
        }
    }

    return NULL;
}

/** See sns_sensor_instance_api::set_client_config */
sns_rc bmi160_inst_set_client_config(
        sns_sensor_instance     *const this,
        sns_request const       *client_request)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    uint32_t                    time_info;
    sns_rc                      rc;

#if BMI160_CONFIG_ENABLE_DEBUG
    bmi160_req_payload          *req_payload = (bmi160_req_payload *) client_request->request;

    BMI160_INST_LOG(MED, this, "<bmi160_iif set_client_config 0x%x @sensor:%d", this,
            (NULL != req_payload) ? req_payload->sensor_type : -1);
#endif

    time_info = sns_convert_ns_to_ticks(1000 * 1000);

    if (istate->pwr_state_present == 0) {
        istate->pwr_state_present = 1;
    }

    // Turn COM port ON
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            true);

    if (
#if BMI160_CONFIG_ENABLE_DAE
            !bmi160_dae_if_available(this)
#else
            1
#endif
       ) {
        bmi160_hal_register_interrupt(this);
    }


    if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
        // 1. Extract sample, report rates from client_request.
        // 2. Configure sensor HW.
        // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
        // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
        // 5. Save the current config information like type, sample_rate, report_rate, etc.
        bmi160_inst_exit_island(this);
        bmi160_inst_assess_overall_req(istate);

#if 0
//bryan porting
        // NOTICE, these code from sample code
        // new clients need to know ODR/WM for samples received between now and
        // when the next configure takes effect
        if (istate->accel_info.odr_curr > BMI160_REGV_ODR_OFF ||
                istate->gyro_info.odr_curr > BMI160_REGV_ODR_OFF ||
                istate->sensor_temp_info.sampling_intvl_curr > 0) {
            bmi160_hal_send_config_event(this);
        }
//bryan porting end
#endif

        if (BMI160_CONFIG_IDLE == istate->config_step)
        {
            rc = bmi160_hal_reconfig_hw(this, HW_CONFIG_CTX_CLIENT);
            BMI160_DD_CHECK_RETVAL(rc, SNS_RC_SUCCESS);
        }

    }
    else if (client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
    {
        //TOCHECK
        bool dae_flush_done = false;

#if BMI160_CONFIG_ENABLE_DAE
        dae_flush_done = bmi160_dae_if_flush_hw(this);
        if (dae_flush_done) {
            istate->ff_flush_client_req = 1;
        }
#endif

        if (!dae_flush_done) {
            BMI160_INST_LOG(HIGH, this, "fifo_flush status: %x",
                    (istate->ff_flush_client_req) |
                    (istate->fifo_info.ff_flush_in_proc << 1)|
                    (istate->fifo_info.ff_flush_trigger << 2) |
                    (istate->hw_config_pending << 3));

            istate->ff_flush_client_req = 1;
#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER
            istate->hb_cfg_info.flush_req = 1;
#endif
            bmi160_hal_fifo_drain(istate, false, BMI160_FIFO_FLUSH_TRIGGER_CLIENTS);
        }
#if BMI160_CONFIG_ENABLE_PEDO
        if (istate->fifo_info.publish_sensors & BMI160_PEDO) {
            bmi160_send_fifo_flush_done(istate->owner, BMI160_PEDO,
                                        BMI160_FIFO_FLUSH_TRIGGER_CLIENTS);
        }
#endif
    }
    else if (client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
    {
        // 1. Extract test type from client_request.
        // 2. Configure sensor HW for test type.
        // 3. send_request() for Timer Sensor in case test needs polling/waits.
        // 4. Factory test is TBD.
        bmi160_inst_exit_island(this);
        bmi160_run_self_test(this);
        istate->new_self_test_request = false;
    }
#if BMI160_CONFIG_ENABLE_CUSTOM_FACTORY_CALIBRATION
    else if (client_request->message_id == SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG)
    {
        BMI160_INST_LOG(HIGH, this, "BMI160 : SNS_PHYSICAL_SENSOR_OEM_CONFIG_MSGID_SNS_PHYSICAL_SENSOR_OEM_CONFIG is received");
        /** All self-tests can be handled in normal mode. */
        bmi160_inst_exit_island(this);

        bmi160_run_self_test_bias(this);
        istate->new_self_test_request_bias = false;
    }
#endif


    // Turn COM port OFF
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            false);

    return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
    BMI160_FTI
static sns_rc bmi160_inst_notify_event(
        sns_sensor_instance     *const this)
{
    bmi160_instance_state       *istate = (bmi160_instance_state*)this->state->state;
    sns_rc                      rc = SNS_RC_SUCCESS;


    BMI160_INST_LOG(MED, istate->owner, "<bmi160_if_ inst_notify_ev>");

    // Turn COM port ON
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            true);

#if BMI160_CONFIG_ENABLE_DAE
    bmi160_dae_if_process_events(this);
#endif


    // Handle event int
    rc = bmi160_inst_handle_event_int(this);


    // Handle Async Com Port events
    rc |= bmi160_inst_handle_event_async_com(istate);

    // Handle Timer events
    rc |= bmi160_inst_handle_event_timer(istate);

    // Turn COM port OFF
    istate->scp_service->api->sns_scp_update_bus_power(istate->com_port_info.port_handle,
            false);

#if BMI160_CONFIG_ENABLE_HEART_BEAT_TIMER && 0
    if (istate->hb_cfg_info.heart_attack_cnt >= BMI160_HB_MAX_HEART_ATTACKS) {
        //remove timer when attack reach to the max
        bmi160_remove_hb_timer(this);
    }
#endif

    //BMI160_INST_LOG(LOW, istate->owner, "<bmi160_if_ inst_notify_ev> exit!!!")
    return SNS_RC_SUCCESS;
}

sns_rc bmi160_hal_reconfig_hw(sns_sensor_instance *this,
        bmi160_hw_cfg_ctx_t hw_cfg_ctx)
{
    bmi160_inst_exit_island(this);//bryan
    return bmi160_hal_dev_reconfig_hw(this, hw_cfg_ctx);
}


/** Public Data Definitions. */

sns_sensor_instance_api bmi160_sensor_instance_api =
{
    .struct_len        = sizeof(sns_sensor_instance_api),
    .init              = &bmi160_inst_init,
    .deinit            = &bmi160_inst_deinit,
    .set_client_config = &bmi160_inst_set_client_config,
    .notify_event      = &bmi160_inst_notify_event
};

