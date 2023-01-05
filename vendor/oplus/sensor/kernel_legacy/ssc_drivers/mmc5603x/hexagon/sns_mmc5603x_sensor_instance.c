/**
 * @file sns_mmc5603x_sensor_instance.c
 *
 * MMC5603X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_types.h"

#include "sns_mmc5603x_hal.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_s4s.h"


#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#ifdef MMC5603X_ENABLE_DIAG_LOGGING
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#endif
#include "sns_sync_com_port_service.h"
#include "sns_sensor_util.h"

/** See sns_sensor_instance_api::init */
sns_rc mmc5603x_inst_init(sns_sensor_instance *const this,
                                sns_sensor_state const *sstate)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)this->state->state;
  mmc5603x_state *sensor_state =
    (mmc5603x_state *)sstate->state;
  float               data[3];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
#ifdef MMC5603X_ENABLE_DAE

  sns_stream_service  *stream_mgr = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
#endif

#ifdef MMC5603X_ENABLE_DIAG_LOGGING
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[MMC5603X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = MMC5603X_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;
#endif
#ifdef MMC5603X_ENABLE_DUAL_SENSOR
  sns_sensor_uid mag_suid = (sensor_state->registration_idx == 0)? (sns_sensor_uid)MAG_SUID1 :
                                                              (sns_sensor_uid)MAG_SUID2;
  MMC5603X_INST_PRINT(ERROR, this, "hardware_id=%d registration_idx=%d", sensor_state->hardware_id, sensor_state->registration_idx);
#else
  sns_sensor_uid mag_suid = (sns_sensor_uid)MAG_SUID1;
#endif

  state->diag_service = (sns_diag_service *)
    service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  /**----------------Copy Sensor UID in instance state---------------*/
  sns_memscpy(&state->mag_info.suid,
              sizeof(state->mag_info.suid),
              &mag_suid,
              sizeof(state->mag_info.suid));

  SNS_INST_PRINTF(ERROR, this, "mmc5603x inst init" );

  /**-------------------------Init Mag State-------------------------*/
  state->mag_info.desired_odr = MMC5603X_MAG_ODR_OFF;
  state->mag_info.curr_odr = MMC5603X_MAG_ODR_OFF;
  sns_memscpy(&state->mag_info.sstvt_adj,
              sizeof(state->mag_info.sstvt_adj),
              &sensor_state->sstvt_adj,
              sizeof(sensor_state->sstvt_adj));
  sns_memscpy(&state->mag_info.device_select,
              sizeof(state->mag_info.device_select),
              &sensor_state->device_select,
              sizeof(sensor_state->device_select));
  state->mag_info.cur_wmk = 0;
  // Init for s4s
  mmc5603x_s4s_inst_init(this, sstate);


#if defined(MMC5603X_ENABLE_ALL_DEVICES)
    state->mag_info.resolution = MMC5603X_RESOLUTION;
    state->mag_info.use_dri = false;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
#endif


  state->pre_timestamp = sns_get_system_time();
  state->this_is_first_data = true;
  state->mag_info.data_count = 0;
  state->heart_beat_attempt_count = 0;
  state->flush_sample_count = 0;
  state->in_clock_error_procedure = false;

  state->mag_info.clock_error_meas_count = 0;
  state->internal_clock_error = 0x01 << MMC5603X_CALC_BIT_RESOLUTION;

  state->satu_checking_flag = 0;

  state->encoded_mag_event_len = pb_get_encoded_size_sensor_stream_event(data, MMC5603X_NUM_AXES);

  /** Initialize Timer info to be used by the Instance */
  sns_suid_lookup_get(&sensor_state->suid_lookup_data, "timer", &state->timer_suid);
  state->timer_data_stream = NULL;

  /** Initialize COM port to be used by the Instance */
  state->com_port_info = sensor_state->com_port_info;
  state->com_port_info.port_handle = NULL;

  state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                                     &state->com_port_info.port_handle );

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);

 /** Copy down axis conversion settings */
  sns_memscpy(state->axis_map,  sizeof(sensor_state->axis_map),
              sensor_state->axis_map, sizeof(sensor_state->axis_map));

  /** Initialize factory calibration */
  state->mag_registry_cfg.fac_cal_corr_mat.e00 = 1.0;
  state->mag_registry_cfg.fac_cal_corr_mat.e11 = 1.0;
  state->mag_registry_cfg.fac_cal_corr_mat.e22 = 1.0;
  
  //Modfify for Soft magnetic parameter start by memsic 20190114
  sns_memscpy( 
  &state->mag_registry_cfg.fac_cal_corr_mat, sizeof(state->mag_registry_cfg.fac_cal_corr_mat),
  &sensor_state->cal_parameter[0].fac_cal_corr_mat,
  sizeof(sensor_state->cal_parameter[0].fac_cal_corr_mat));
  //Modfify for Soft magnetic parameter end by memsic20190114

#ifdef MMC5603X_ENABLE_DIAG_LOGGING
  /** Determine size of sns_diag_sensor_state_raw as defined in
   *  sns_diag.proto
   *  sns_diag_sensor_state_raw is a repeated array of samples of
   *  type sns_diag_batch sample. The following determines the
   *  size of sns_diag_sensor_state_raw with a single batch
   *  sample */
  if(pb_encode_tag(&stream, PB_WT_STRING,
                    sns_diag_sensor_state_raw_sample_tag))
  {
    if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                               &batch_sample))
    {
      state->log_raw_encoded_size = stream.bytes_written;
    }
  }
#endif

#ifdef MMC5603X_ENABLE_DAE
  sns_sensor_uid dae_suid;
  sns_suid_lookup_get(&sensor_state->suid_lookup_data, "data_acquisition_engine", &dae_suid);
  mmc5603x_dae_if_init(this, stream_mgr, &dae_suid, sensor_state);
#endif // MMC5603X_ENABLE_DAE

return SNS_RC_SUCCESS;
}

sns_rc mmc5603x_inst_deinit(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)this->state->state;

  SNS_INST_PRINTF(ERROR, this, "deinit:: ", state->total_samples);

  if(NULL != state->com_port_info.port_handle)
  {
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);
    mmc5603x_reconfig_hw(this);
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
  }

  mmc5603x_dae_if_deinit(this);

  sns_sensor_util_remove_sensor_instance_stream(this,&state->timer_data_stream);
  //Deinit for S4S
  mmc5603x_s4s_inst_deinit(this);
  if(NULL != state->scp_service)
  {
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
  }
  return SNS_RC_SUCCESS;
}

static uint16_t mmc5603x_set_wmk(sns_sensor_instance *const this,
                                float desired_report_rate,
								float mag_chosen_sample_rate)
{
  uint16_t desired_wmk = 0;

  UNUSED_VAR(this);
  UNUSED_VAR(desired_report_rate);
  UNUSED_VAR(mag_chosen_sample_rate);
  return desired_wmk;
}

/*
static void mmc5603x_care_fifo_buffer(sns_sensor_instance *const this,
                                     float mag_chosen_sample_rate,
                                     mmc5603x_mag_odr mag_chosen_sample_rate_reg_value)
{
  UNUSED_VAR(this);
  UNUSED_VAR(mag_chosen_sample_rate);
  UNUSED_VAR(mag_chosen_sample_rate_reg_value);
}
*/

void mmc5603x_continue_client_config(sns_sensor_instance *const this)
{
  mmc5603x_instance_state *state = (mmc5603x_instance_state *)this->state->state;

  MMC5603X_INST_PRINT(ERROR, this, "continue_client_config::");

  mmc5603x_reconfig_hw(this);

  if(!state->mag_info.use_dri)
  {
    mmc5603x_register_timer(this);
    // Register for s4s timer
    if (state->mag_info.use_sync_stream)
    {
      mmc5603x_s4s_register_timer(this);
    }
  }
  else if(!mmc5603x_dae_if_available(this))
  {
    mmc5603x_register_interrupt(this);
  }
}

/** See sns_sensor_instance_api::set_client_config */
sns_rc mmc5603x_inst_set_client_config(sns_sensor_instance *const this,
                                      sns_request const *client_request)
{
  mmc5603x_instance_state *state =
    (mmc5603x_instance_state *)this->state->state;
  state->client_req_id = client_request->message_id;
  float           desired_sample_rate = 0;
  float           desired_report_rate = 0;
  float           mag_chosen_sample_rate = 0;
  mmc5603x_mag_odr mag_chosen_sample_rate_reg_value;
  uint16_t        desired_wmk;
  sns_rc          rv = SNS_RC_SUCCESS;

  MMC5603X_INST_PRINT(ERROR, this, "inst_set_client_config msg_id %d, state=0x%x", client_request->message_id, *state);
  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);

  if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
    // In case of DRI,
    //   1. Extract sample, report rates from client_request.
    //   2. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
    //   3. Configure sensor HW.
    //   4. Save the current config information like type, sample_rate, report_rate, etc.
    // In case of polling,
    //   1. Extract sample, report rates from client_request.
    //   2. Configure sensor HW.
    //   3. Save the current config information like type, sample_rate, report_rate, etc.
    //   4. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
    sns_mmc5603x_mag_req *payload =
      (sns_mmc5603x_mag_req *)client_request->request;
    desired_sample_rate = payload->sample_rate;
    desired_report_rate = payload->report_rate;
    state->mag_info.flush_only = payload->is_flush_only;
    state->mag_info.max_batch = payload->is_max_batch;

	///gumu addd 12-19 fix bug 2hz
	state->mag_req.sample_rate = desired_sample_rate;
	///gumu end 1219 

    
	if (desired_report_rate > desired_sample_rate)
    {
      // bad request. Return error or default report_rate to sample_rate
      desired_report_rate = desired_sample_rate;
    }

    MMC5603X_INST_PRINT(ERROR, this, "desired_SR=%d desired_RR=%d/100, req_wmk=%d",
        (int)desired_sample_rate, (int)(desired_report_rate*100), (int)state->mag_info.req_wmk);

    // If already a request, send out existing config before the next data events are sent
    if (desired_sample_rate > 0)
    {
      mmc5603x_send_config_event(this);
    }

    state->mag_info.flush_period = payload->flush_period;
    rv = mmc5603x_mag_match_odr(desired_sample_rate,
                               &mag_chosen_sample_rate,
                               &mag_chosen_sample_rate_reg_value,
                               state->mag_info.device_select);
    if (rv != SNS_RC_SUCCESS)
    {
      sns_service_manager *manager =
        this->cb->get_service_manager(this);
      sns_event_service *event_service =
      	(sns_event_service*)manager->get_service(manager, SNS_EVENT_SERVICE);
      SNS_INST_PRINTF(ERROR, this, "Cannot find match ODR: rv=%d, desired_SR=%d, mag device=%d",
        rv, (int)desired_sample_rate, state->mag_info.device_select);
      event_service->api->publish_error(event_service,this, SNS_RC_NOT_SUPPORTED);
      return rv;
    }

    if (state->mag_info.max_batch)
    {
      state->mag_info.req_wmk = UINT32_MAX;
    }
    else if (desired_report_rate != 0.0f)
    {
      if (state->mag_info.flush_only)
      {
        state->mag_info.req_wmk = UINT32_MAX;
      }
      else
      {
        // possibly larger than max HW FIFO for DAE
        state->mag_info.req_wmk = (uint16_t)(mag_chosen_sample_rate / desired_report_rate);
      }
    }
    else
    {
      state->mag_info.req_wmk = 0;
    }

    desired_wmk = mmc5603x_set_wmk(this, desired_report_rate, mag_chosen_sample_rate);
    MMC5603X_INST_PRINT(ERROR, this, "req_wmk=%u desired_wmk=%u,flush_period=%u, flushonly=%d, max_batch=%d",
                       state->mag_info.req_wmk, desired_wmk, state->mag_info.flush_period,
                       state->mag_info.flush_only?1:0, state->mag_info.max_batch?1:0 );

     
    if( state->mag_info.cur_wmk == desired_wmk &&
        state->mag_info.curr_odr == mag_chosen_sample_rate_reg_value )
    {
      // No change needed -- return success
      MMC5603X_INST_PRINT(ERROR, this, "Config not changed.");
      if(!state->in_clock_error_procedure && !mmc5603x_dae_if_available(this))
      {
        mmc5603x_send_config_event(this);
      }
      // Turn COM port OFF
      state->scp_service->api->sns_scp_update_bus_power(
                                                        state->com_port_info.port_handle,
                                                        false);

      return SNS_RC_SUCCESS;
    }

    state->mag_info.cur_wmk     = desired_wmk;
    state->mag_req.sample_rate  = mag_chosen_sample_rate;
    state->mag_info.desired_odr = state->new_cfg.odr = mag_chosen_sample_rate_reg_value;
    state->new_cfg.fifo_wmk     = state->mag_info.cur_wmk + 1;
	//memsic add 20190118 for droping the first data after enable sensor
	state->mag_info.num_samples_to_discard = 1; 
	//memsic add 20190118 for droping the first data after enable sensor

    if (0.0f == state->last_sent_cfg.odr)
    {
      mmc5603x_send_config_event(this);
    }

    MMC5603X_INST_PRINT(ERROR, this, "sample_rate=%d, reg_value=%d, config_step=%d",
                       (int)mag_chosen_sample_rate,
                       (int)mag_chosen_sample_rate_reg_value,
                       (int)state->config_step);


#ifdef MMC5603X_ENABLE_DAE
    if(mmc5603x_dae_if_is_initializing(this))
    {
      MMC5603X_INST_PRINT(ERROR, this, "Waiting for DAE init result");
      return SNS_RC_SUCCESS;
    }
#endif //MMC5603X_ENABLE_DAE

    state->system_time = sns_get_system_time();

	 // Register for timer  memsic add  18-11-11
      if (!state->mag_info.use_dri && !mmc5603x_dae_if_available(this))
      {
        MMC5603X_INST_PRINT(ERROR, this, "memsic call mmc5603x_reconfig_hw .");
        mmc5603x_reconfig_hw(this);
        mmc5603x_register_timer(this);
        // Register for s4s timer
        if (state->mag_info.use_sync_stream)
        {
          mmc5603x_s4s_register_timer(this);
        }
      }
	  // Register for timer  memsic add  18-11-11
#ifdef MMC5603X_ENABLE_DAE
    else
    {
      if (MMC5603X_CONFIG_IDLE == state->config_step && mmc5603x_dae_if_stop_streaming(this))
      {
        MMC5603X_INST_PRINT(ERROR, this, "done dae_if_stop_streaming");
        state->config_step = MMC5603X_CONFIG_STOPPING_STREAM;
      }
	   
	   MMC5603X_INST_PRINT(ERROR, this,"memsic 20190110 call mmc5603x_dae_if_start_streaming 444 ");

      if (MMC5603X_CONFIG_IDLE == state->config_step && mmc5603x_dae_if_start_streaming(this))
      {
        MMC5603X_INST_PRINT(ERROR, this, " mmc5603x_dae_if_start_streaming in  sensor instance");
        state->config_step = MMC5603X_CONFIG_UPDATING_HW;
      }
    }
#endif //MMC5603X_ENABLE_DAE

    if (state->config_step == MMC5603X_CONFIG_IDLE)  //TODO  MEMSIC AFTER DAE  11-11
    {
      mmc5603x_continue_client_config(this);
    }

	// Start - Added for CTS debugging 20180111 
	if (MMC5603X_CONFIG_UPDATING_HW == state->config_step)
	{
		state->config_step = MMC5603X_CONFIG_IDLE;
	}
	// End - Added for CTS debugging 20180111 
  }
  else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    state->system_time = sns_get_system_time();
    state->fifo_flush_in_progress = true;
    MMC5603X_INST_PRINT(ERROR, this, "Flush requested at %u", (uint32_t)state->system_time);

    if(!mmc5603x_dae_if_flush_hw(this))
    {
      mmc5603x_send_fifo_flush_done(this);
    }
  }
  else if (state->client_req_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    // If the sensor is streaming and there is a client_request to run self-test,
    // the existing stream can be temporarily paused.
    // When the self-test completes, the paused stream shall be restarted.
    if (state->mag_info.curr_odr != MMC5603X_MAG_ODR_OFF)
    {
      MMC5603X_INST_PRINT(ERROR, this, "pause the stream for self-test.");
      mag_chosen_sample_rate = state->mag_req.sample_rate;
      mag_chosen_sample_rate_reg_value = state->mag_info.desired_odr;
      state->mag_req.sample_rate = 0;
      state->mag_info.desired_odr = MMC5603X_MAG_ODR_OFF;

#ifdef MMC5603X_ENABLE_DAE
      if (MMC5603X_CONFIG_IDLE == state->config_step &&
          mmc5603x_dae_if_stop_streaming(this))
      {
        MMC5603X_INST_PRINT(ERROR, this, "done dae_if_stop_streaming");
        state->config_step = MMC5603X_CONFIG_STOPPING_STREAM;
      }
#endif //MMC5603X_ENABLE_DAE

      if (state->config_step == MMC5603X_CONFIG_IDLE)
      {
        // hardware setting for measurement mode
        
		MMC5603X_INST_PRINT(ERROR, this,"memsic 20190110 call mmc5603x_dae_if_start_streaming 555 ");
        if (state->mag_info.use_dri && !mmc5603x_dae_if_start_streaming(this))
       	{
          mmc5603x_reconfig_hw(this);
        }

        // Register for timer
        if (!state->mag_info.use_dri && !mmc5603x_dae_if_available(this))
        {
          mmc5603x_reconfig_hw(this);
          mmc5603x_register_timer(this);
        }
      }

      mmc5603x_dae_if_process_events(this);

      MMC5603X_INST_PRINT(ERROR, this, "Execute the self-test.");
      mmc5603x_run_self_test(this);
      state->new_self_test_request = false;

      MMC5603X_INST_PRINT(ERROR, this, "Resume the stream.");
      state->mag_req.sample_rate = mag_chosen_sample_rate;
      state->mag_info.desired_odr = mag_chosen_sample_rate_reg_value;

      // Register for timer
      if (!state->mag_info.use_dri && !mmc5603x_dae_if_available(this))
      {
        mmc5603x_reconfig_hw(this);
        mmc5603x_register_timer(this);
      }

      mmc5603x_send_config_event(this);

      // DAE handles SET_STREAMING_CONFIG request
      mmc5603x_dae_if_process_events(this);
    }
    else
    {
      MMC5603X_INST_PRINT(ERROR, this, "SENSOR_TEST_CONFIG for selftest" );
      mmc5603x_run_self_test(this);
      state->new_self_test_request = false;
    }
  }

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);

  return SNS_RC_SUCCESS;
}
