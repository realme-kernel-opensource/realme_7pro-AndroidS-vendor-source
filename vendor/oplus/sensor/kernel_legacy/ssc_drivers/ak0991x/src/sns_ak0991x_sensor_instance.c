/**
 * @file sns_ak0991x_sensor_instance.c
 *
 * AK0991X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
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

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"
#include "sns_ak0991x_ver.h"


#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_sensor_util.h"
#include "oppo_sensor.h"

/** See sns_sensor_instance_api::init */
sns_rc ak0991x_inst_init(sns_sensor_instance *const this,
                                sns_sensor_state const *sstate)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  ak0991x_state *sensor_state =
    (ak0991x_state *)sstate->state;
  float               data[3];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_mgr = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[AK0991X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = AK0991X_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;
#ifdef AK0991X_ENABLE_DUAL_SENSOR
  sns_sensor_uid mag_suid = (sensor_state->registration_idx == 0)? (sns_sensor_uid)MAG_SUID1 :
                                                              (sns_sensor_uid)MAG_SUID2;
  AK0991X_INST_PRINT(LOW, this, "hardware_id=%d registration_idx=%d", sensor_state->hardware_id, sensor_state->registration_idx);
#else
  sns_sensor_uid mag_suid = (sns_sensor_uid)MAG_SUID1;
#endif //AK0991X_ENABLE_DUAL_SENSOR

  state->diag_service = (sns_diag_service *)
    service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  /**----------------Copy Sensor UID in instance state---------------*/
  sns_memscpy(&state->mag_info.suid,
              sizeof(state->mag_info.suid),
              &mag_suid,
              sizeof(state->mag_info.suid));

  AK0991X_INST_PRINT(HIGH, this, "ak0991x inst init DRIVER_VER:%02d.%02d.%02d",
      (int)AK0991X_DRIVER_VERSION>>16,
      (int)(0xFF & (AK0991X_DRIVER_VERSION>>8)),
      (int)(0xFF & AK0991X_DRIVER_VERSION) );

  /**-------------------------Init Mag State-------------------------*/
  state->mag_info.cur_cfg.num = 0;
  state->mag_info.cur_cfg.odr = AK0991X_MAG_ODR_OFF;
  state->mag_info.last_sent_cfg.num = 0;
  state->mag_info.last_sent_cfg.odr = AK0991X_MAG_ODR_OFF;

  sns_memscpy(&state->mag_info.sstvt_adj,
              sizeof(state->mag_info.sstvt_adj),
              &sensor_state->sstvt_adj,
              sizeof(sensor_state->sstvt_adj));
  sns_memscpy(&state->mag_info.device_select,
              sizeof(state->mag_info.device_select),
              &sensor_state->device_select,
              sizeof(sensor_state->device_select));
  sns_memscpy(&state->mag_info.max_odr,
              sizeof(state->mag_info.max_odr),
              &sensor_state->max_odr,
              sizeof(sensor_state->max_odr));

  state->mag_info.cur_cfg.fifo_wmk = 1;
  // Init for s4s
  ak0991x_s4s_inst_init(this, sstate);

  switch (state->mag_info.device_select)
  {
  case AK09911:
    state->mag_info.resolution = AK09911_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09911_FIFO_SIZE;
    state->mag_info.int_mode = AK0991X_INT_OP_MODE_POLLING;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
    break;
  case AK09912:
    state->mag_info.resolution = AK09912_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09912_FIFO_SIZE;
    state->mag_info.int_mode = sensor_state->int_mode;
    state->mag_info.nsf = sensor_state->nsf;
    state->mag_info.sdr = 0;
    break;
  case AK09913:
    state->mag_info.resolution = AK09913_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09913_FIFO_SIZE;
    state->mag_info.int_mode = AK0991X_INT_OP_MODE_POLLING;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
    break;
  case AK09915C:
  case AK09915D:
    state->mag_info.resolution = AK09915_RESOLUTION;
    state->mag_info.use_fifo = sensor_state->use_fifo;
    state->mag_info.max_fifo_size = AK09915_FIFO_SIZE;
    state->mag_info.int_mode = sensor_state->int_mode;
    state->mag_info.nsf = sensor_state->nsf;
    state->mag_info.sdr = sensor_state->sdr;
    break;
  case AK09916C:
    state->mag_info.resolution = AK09916_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09916_FIFO_SIZE;
    state->mag_info.int_mode = AK0991X_INT_OP_MODE_POLLING;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
    break;
  case AK09916D:
    state->mag_info.resolution = AK09916_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09916_FIFO_SIZE;
    state->mag_info.int_mode = sensor_state->int_mode;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
    break;
  case AK09917:
    state->mag_info.resolution = AK09917_RESOLUTION;
    state->mag_info.use_fifo = sensor_state->use_fifo;
    state->mag_info.max_fifo_size = AK09917_FIFO_SIZE;
    state->mag_info.int_mode = sensor_state->int_mode;
    state->mag_info.nsf = sensor_state->nsf;
    state->mag_info.sdr = sensor_state->sdr;
    AK0991X_INST_PRINT(HIGH, this, "AK09917 RSV1=0x%02X", (int)sensor_state->reg_rsv1_value);
    break;
  case AK09918:
    state->mag_info.resolution = AK09918_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09918_FIFO_SIZE;
    state->mag_info.int_mode = AK0991X_INT_OP_MODE_POLLING;
    state->mag_info.nsf = 0;
    state->mag_info.sdr = 0;
    break;
  default:
    return SNS_RC_FAILED;
  }

  // reset timestamp and parameters
  state->system_time =
  state->pre_timestamp =
  state->pre_timestamp_for_orphan =
      sns_get_system_time();

  state->this_is_first_data = true;
  state->heart_beat_attempt_count = 0;
  state->flush_sample_count = 0;
  state->mag_info.data_count_for_dri = 0;
  state->in_clock_error_procedure = false;
  state->mag_info.clock_error_meas_count = 0;
  state->internal_clock_error = 0x01 << AK0991X_CALC_BIT_RESOLUTION;
  state->reg_event_done = false;
  state->has_sync_ts_anchor = false;
  state->reg_event_for_dae_poll_sync = false;
  state->is_previous_irq = false;
  state->total_samples = 0;
  state->flush_requested_in_dae = false;
  state->wait_for_last_flush = false;
  state->last_flush_poll_check_count = 0;
  state->do_flush_after_clock_error_procedure = false;
  state->do_flush_after_change_config = false;

  state->encoded_mag_event_len = pb_get_encoded_size_sensor_stream_event(data, AK0991X_NUM_AXES);

  {
    sns_rc rv;
    sns_sensor_uid irq_suid;
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "interrupt", &irq_suid);

    rv = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                        this,
                                                        irq_suid,
                                                        &state->interrupt_data_stream);
    if (rv != SNS_RC_SUCCESS)
    {
      return rv;
    }
  }

  {
    sns_rc rv;
    sns_sensor_uid acp_suid;
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "async_com_port", &acp_suid);
    rv = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                        this,
                                                        acp_suid,
                                                        &state->async_com_port_data_stream);
    if (rv != SNS_RC_SUCCESS)
    {
      stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);
      return rv;
    }
  }

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
  {
    sns_rc rv;
    sns_sensor_uid device_mode_suid;
    sns_suid_lookup_get(&sensor_state->suid_lookup_data, "device_mode", &device_mode_suid);
    rv = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                        this, 
                                                        device_mode_suid,
                                                        &state->device_mode_stream);
    if(rv != SNS_RC_SUCCESS)
    {
#ifndef AK0991X_BOARD_HDK845
      return rv;
#endif // AK0991X_BOARD_HDK845
    }
  }
#endif

  uint8_t i;
  for(i = 0; i < MAX_DEVICE_MODE_SUPPORTED; i++)
  {
    sns_memscpy(&state->cal.params[i],
                sizeof(state->cal.params[i]),
                &sensor_state->cal_params[i],
                sizeof(sensor_state->cal_params[i]));

    AK0991X_INST_PRINT(LOW, this, "| %4d %4d %4d |",
       (int)(state->cal.params[i].corr_mat.e00*100),
       (int)(state->cal.params[i].corr_mat.e01*100),
       (int)(state->cal.params[i].corr_mat.e02*100));
     AK0991X_INST_PRINT(LOW, this, "| %4d %4d %4d |",
       (int)(state->cal.params[i].corr_mat.e10*100),
       (int)(state->cal.params[i].corr_mat.e11*100),
       (int)(state->cal.params[i].corr_mat.e12*100));
     AK0991X_INST_PRINT(LOW, this, "| %4d %4d %4d |",
       (int)(state->cal.params[i].corr_mat.e20*100),
       (int)(state->cal.params[i].corr_mat.e21*100),
       (int)(state->cal.params[i].corr_mat.e22*100));
      AK0991X_INST_PRINT(LOW, this, "Fac Cal bias %d %d %d",
       (int)(state->cal.params[i].bias[0]),
       (int)(state->cal.params[i].bias[1]),
       (int)(state->cal.params[i].bias[2]));
  }
  state->cal.id = state->prev_cal_id = 0;

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

  /** Initialize IRQ info to be used by the Instance */
  sns_memscpy(&state->irq_info.irq_config,
              sizeof(state->irq_info.irq_config),
              &sensor_state->irq_config,
              sizeof(sensor_state->irq_config));

  state->irq_info.is_registered = false;
  state->irq_info.detect_irq_event = false;
  state->irq_info.is_ready = false;

  /** Configure the Async Com Port */
  uint8_t                   pb_encode_buffer[100];
  uint32_t                  enc_len;

  state->ascp_config.bus_instance = sensor_state->com_port_info.com_config.bus_instance;
  state->ascp_config.bus_type =
    (sns_async_com_port_bus_type)sensor_state->com_port_info.com_config.bus_type;
  state->ascp_config.max_bus_speed_kHz =
    sensor_state->com_port_info.com_config.max_bus_speed_KHz;
  state->ascp_config.min_bus_speed_kHz =
    sensor_state->com_port_info.com_config.min_bus_speed_KHz;
  state->ascp_config.reg_addr_type = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  state->ascp_config.slave_control = sensor_state->com_port_info.com_config.slave_control;
  enc_len = pb_encode_request(pb_encode_buffer, 100, &state->ascp_config,
                              sns_async_com_port_config_fields, NULL);

  sns_request async_com_port_request =
    (sns_request)
  {
    .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
    .request_len = enc_len,
    .request = &pb_encode_buffer
  };
  state->async_com_port_data_stream->api->send_request(
    state->async_com_port_data_stream, &async_com_port_request);

 /** Copy down axis conversion settings */
  sns_memscpy(state->axis_map,  sizeof(sensor_state->axis_map),
              sensor_state->axis_map, sizeof(sensor_state->axis_map));

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

#ifdef AK0991X_ENABLE_DAE
  sns_sensor_uid dae_suid;
  sns_suid_lookup_get(&sensor_state->suid_lookup_data, "data_acquisition_engine", &dae_suid);
  ak0991x_dae_if_init(this, stream_mgr, &dae_suid, sensor_state);
#endif // AK0991X_ENABLE_DAE

  return SNS_RC_SUCCESS;
}

sns_rc ak0991x_inst_deinit(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  if((!ak0991x_dae_if_available(this) && state->fifo_flush_in_progress) ||
     (ak0991x_dae_if_available(this) && state->flush_requested_in_dae))
  {
    AK0991X_INST_PRINT(HIGH, this, "flush_done in deinit");
    ak0991x_send_fifo_flush_done(this);
  }

  if(NULL != state->com_port_info.port_handle)
  {
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, true);
    if(SNS_RC_SUCCESS == ak0991x_enter_i3c_mode(this, &state->com_port_info, state->scp_service))
    {
      ak0991x_reconfig_hw(this, false);
    }
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
  }

  ak0991x_dae_if_deinit(this);

  sns_sensor_util_remove_sensor_instance_stream(this,&state->timer_data_stream);
  //Deinit for S4S
  ak0991x_s4s_inst_deinit(this);
  sns_sensor_util_remove_sensor_instance_stream(this,&state->interrupt_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this,&state->async_com_port_data_stream);
#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
  sns_sensor_util_remove_sensor_instance_stream(this, &state->device_mode_stream);
#endif
  if(NULL != state->scp_service)
  {
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
  }
  SNS_INST_PRINTF(HIGH, this, "deinit:: #samples=%u", state->total_samples);

  return SNS_RC_SUCCESS;
}

static uint16_t ak0991x_calc_fifo_wmk(
    sns_sensor_instance *const this,
    float desired_report_rate,
    float mag_chosen_sample_rate)
{
  uint16_t desired_wmk = 1;
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  if(state->mag_info.use_fifo && mag_chosen_sample_rate != 0.0f)
  {
    switch (state->mag_info.device_select)
    {
      case AK09915C:
      case AK09915D:
      case AK09917:
        if( desired_report_rate != 0.0f )
        {
          desired_wmk = (uint16_t) (mag_chosen_sample_rate + 0.01f * desired_report_rate) / desired_report_rate;
        }

        if (state->mag_info.max_batch)
        {
          desired_wmk = state->mag_info.max_fifo_size;
        }
        else if ( desired_wmk >= (UINT16_MAX - state->mag_info.max_fifo_size) )
        {
          desired_wmk = state->mag_info.max_fifo_size;
        }
        else if ( desired_wmk > state->mag_info.max_fifo_size )
        {
          uint32_t divider = 1;
          divider = (state->mag_info.max_fifo_size + desired_wmk) / state->mag_info.max_fifo_size;
          desired_wmk = desired_wmk / SNS_MAX(divider,1);
        }
        break;

      default:
        desired_wmk = 1;
        break;
    }
  }
  AK0991X_INST_PRINT(LOW, this, "calculated fifo_wmk=%u flush_only=%d max_batch=%d",
      desired_wmk,
      state->mag_info.flush_only,
      state->mag_info.max_batch);
  return desired_wmk;
}

static uint32_t ak0991x_calc_dae_wmk(
    sns_sensor_instance *const this,
    float desired_report_rate,
    float mag_chosen_sample_rate,
    uint16_t fifo_wmk)
{
  uint32_t dae_wmk = 1;

#ifdef AK0991X_ENABLE_DAE
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  if (desired_report_rate != 0.0f && mag_chosen_sample_rate != 0.0f)
  {
    dae_wmk = SNS_MAX(1, (uint32_t)((mag_chosen_sample_rate + 0.01f * desired_report_rate) / desired_report_rate)); // prevent dae_wmk=0
    if(state->mag_info.flush_only || state->mag_info.max_batch)
    {
      dae_wmk = UINT32_MAX;
    }
  }

  AK0991X_INST_PRINT(LOW, this, "calculated  dae_wmk=%u flush_only=%d max_batch=%d",
      dae_wmk,
      state->mag_info.flush_only,
      state->mag_info.max_batch);
  UNUSED_VAR(fifo_wmk);
#else
  dae_wmk = fifo_wmk; // dae_wmk = fifo_wmk when nonDAE? (MAG-042 on DRI+FIFO+nonDAE still fails)
  UNUSED_VAR(this);
  UNUSED_VAR(desired_report_rate);
  UNUSED_VAR(mag_chosen_sample_rate);
#endif
  return dae_wmk;
}

static void ak0991x_care_fifo_buffer(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  state->this_is_the_last_flush = true;
  AK0991X_INST_PRINT(LOW, this, "last flush before changing ODR");
  if(state->ascp_xfer_in_progress > 0)
  {
    state->config_mag_after_ascp_xfer = true;
    state->re_read_data_after_ascp = true;
    AK0991X_INST_PRINT(LOW, this, "last flush but ascp is in progress. Skip");
  }
  else
  {
    ak0991x_read_mag_samples(this);
    state->this_is_the_last_flush = false;
    state->wait_for_last_flush = false;
  }
}

void ak0991x_continue_client_config(sns_sensor_instance *const this, bool reset_device)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;

  if( state->config_mag_after_ascp_xfer )
  {
    AK0991X_INST_PRINT(LOW, this, "skip reconfig_hw because config_mag_after_ascp_xfer.");
    return;
  }

  AK0991X_INST_PRINT(LOW, this, "continue_client_config::");

  ak0991x_reconfig_hw(this, reset_device);

  if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)
  {
    // Register polling timer
    ak0991x_register_timer(this);

    // Register for s4s timer
    if (state->mag_info.use_sync_stream)
    {
      ak0991x_s4s_register_timer(this);
    }
  }
  else
  {
    if(!ak0991x_dae_if_available(this))
    {
        ak0991x_register_interrupt(this);
    }
    ak0991x_register_heart_beat_timer(this);
  }
}

/** See sns_sensor_instance_api::set_client_config */
sns_rc ak0991x_inst_set_client_config(sns_sensor_instance *const this,
                                      sns_request const *client_request)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  float           desired_sample_rate = 0.0f;
  float           desired_report_rate = 0.0f;
  float           mag_chosen_sample_rate = 0.0f;
  ak0991x_mag_odr mag_chosen_sample_rate_reg_value;
  ak0991x_config_event_info req_cfg;

  sns_rc          rv = SNS_RC_SUCCESS;

  AK0991X_INST_PRINT(MED, this, "@@@ inst_set_client_config msg_id %d first_data=%d pre=%u total=%d",
      client_request->message_id,
      state->this_is_first_data,
      (uint32_t)state->pre_timestamp,
      state->total_samples);

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

    sns_ak0991x_mag_req *payload =
      (sns_ak0991x_mag_req *)client_request->request;
    desired_sample_rate = payload->sample_rate;
    desired_report_rate = payload->report_rate;
    
#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
    if(NULL != state->device_mode_stream )
    {
      sns_request request = (sns_request){
       .message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
       .request_len = 0, .request = NULL };
      state->device_mode_stream->api->send_request(state->device_mode_stream, &request);
    }
#endif

    if (desired_report_rate > desired_sample_rate)
    {
      // bad request. Return error or default report_rate to sample_rate
      desired_report_rate = desired_sample_rate;
    }

    AK0991X_INST_PRINT(LOW, this, "desired_SR=%d desired_RR=%d/100",
        (int)desired_sample_rate, (int)(desired_report_rate*100));

    rv = ak0991x_mag_match_odr(desired_sample_rate,
                               &mag_chosen_sample_rate,
                               &mag_chosen_sample_rate_reg_value,
                               state->mag_info.device_select,
                               (float)state->mag_info.max_odr);

    // update requested config
    state->mag_info.flush_only = payload->is_flush_only;
    state->mag_info.max_batch = payload->is_max_batch;
    state->mag_info.flush_period = payload->flush_period;
    req_cfg.odr      = mag_chosen_sample_rate_reg_value;
    req_cfg.fifo_wmk = ak0991x_calc_fifo_wmk(this, desired_report_rate, mag_chosen_sample_rate);
    req_cfg.dae_wmk  = ak0991x_calc_dae_wmk(this, desired_report_rate, mag_chosen_sample_rate, req_cfg.fifo_wmk);

    AK0991X_INST_PRINT(LOW, this, "Calc odr=%d, req_wmk=%d, dae_wmk=%u, flush_period=%u, flushonly=%d, max_batch=%d",
        req_cfg.odr,
        req_cfg.fifo_wmk,
        req_cfg.dae_wmk,
        (uint32_t)state->mag_info.flush_period,
        state->mag_info.flush_only?1:0,
        state->mag_info.max_batch?1:0 );
    
    // Send out previous config
    if ( desired_sample_rate > 0 &&
         state->mag_info.last_sent_cfg.num > 0 &&
         !state->in_self_test &&
         !state->remove_request )
    {
      AK0991X_INST_PRINT(MED, this, "Send current config: odr=0x%02X fifo_wmk=%d",
          (uint32_t)state->mag_info.cur_cfg.odr,
          (uint32_t)state->mag_info.cur_cfg.fifo_wmk);
      ak0991x_send_config_event(this, false); // send previous config event
      ak0991x_send_cal_event(this, false);    // send previous cal event
    }

    state->only_dae_wmk_is_changed = false;

    // check if config not changed.
    if( !state->processing_new_config &&
        state->mag_info.last_sent_cfg.fifo_wmk == req_cfg.fifo_wmk &&
        state->mag_info.last_sent_cfg.odr == req_cfg.odr)
    {
      if((state->mag_info.last_sent_cfg.dae_wmk == req_cfg.dae_wmk) || !ak0991x_dae_if_available(this))
      {
        // No change needed -- return success
        AK0991X_INST_PRINT(LOW, this, "Config not changed. total=%d #%d odr=0x%02X fifo_wmk=%d, dae_wmk=%d", 
        state->total_samples,
        state->mag_info.cur_cfg.num,
        (uint32_t)state->mag_info.cur_cfg.odr,
        (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
        (uint32_t)state->mag_info.cur_cfg.dae_wmk);

        // self test done and resumed. No need to send config event.
        if( state->in_self_test )
        {
          AK0991X_INST_PRINT(LOW, this, "selftest done!" );
        }

        // Turn COM port OFF
        state->scp_service->api->sns_scp_update_bus_power(
                                                          state->com_port_info.port_handle,
                                                          false);
        return SNS_RC_SUCCESS;
      }
      AK0991X_INST_PRINT(LOW, this, "Same ODR and Same WM but DAE_WMK is different.");
        state->only_dae_wmk_is_changed = true;
      }
    
    // If dae_if.mag.state is STREAM_STARTING and config_step is UPDATING_HW, re-evaluate a configuration.
    if(state->dae_if.mag.state == STREAM_STARTING && state->config_step == AK0991X_CONFIG_UPDATING_HW)
    {
      AK0991X_INST_PRINT(LOW, this, "DAE state is STREAM_STARTING. Re-evaluate a configuration.");
      state->config_step = AK0991X_CONFIG_IDLE;
      state->only_dae_wmk_is_changed = false;
    }

    // new config received. start processing for new config.
    state->processing_new_config = true;
 
    // set current config values in state parameter
    state->mag_info.cur_cfg.num++;
    state->mag_info.cur_cfg.odr       = req_cfg.odr;
    state->mag_info.cur_cfg.fifo_wmk  = req_cfg.fifo_wmk;
    state->mag_info.cur_cfg.dae_wmk   = req_cfg.dae_wmk;

    AK0991X_INST_PRINT(HIGH, this, "Set new config #%d : odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
    state->mag_info.cur_cfg.num,
    (uint32_t)state->mag_info.cur_cfg.odr,
    (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
    (uint32_t)state->mag_info.cur_cfg.dae_wmk);

    // after inst init.
    if( state->mag_info.last_sent_cfg.num == 0 && 
        state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF )
    {
      // reset timestamp
      state->pre_timestamp =
      state->pre_timestamp_for_orphan =
      state->system_time;

      // update averaged_interval
      ak0991x_reset_averaged_interval(this);

      // since the physical config event needs to contain a proper "sync_ts_anchor",
      // this should be delayed until after receving the timer response for the polling timer.
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING )
      {
        AK0991X_INST_PRINT(LOW, this, "Config set time =%u", (uint32_t)state->config_set_time);

        ak0991x_send_config_event(this, true); // send new config event
        ak0991x_send_cal_event(this, true);    // send new cal event
      }
    }

    if(state->in_clock_error_procedure)
    {
      // Save request, but not set HW config -- return success
      AK0991X_INST_PRINT(LOW, this, "100Hz dummy measurement is still running. save request.");

      // Turn COM port OFF
      state->scp_service->api->sns_scp_update_bus_power(
                                                        state->com_port_info.port_handle,
                                                        false);
      return SNS_RC_SUCCESS;
    }

#ifdef AK0991X_ENABLE_DAE
    if(ak0991x_dae_if_is_initializing(this))
    {
      AK0991X_INST_PRINT(LOW, this, "Waiting for DAE init result");

      // Turn COM port OFF
      state->scp_service->api->sns_scp_update_bus_power(
                                                        state->com_port_info.port_handle,
                                                        false);
      return SNS_RC_SUCCESS;
    }
#endif //AK0991X_ENABLE_DAE

    if( !ak0991x_dae_if_available(this) && state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF )
    {
      ak0991x_care_fifo_buffer(this);
      if( state->timer_data_stream != NULL )
      {
        sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
      }
    }
#ifdef AK0991X_ENABLE_DAE
    else
    {
      if ((AK0991X_CONFIG_IDLE == state->config_step)
           && ak0991x_dae_if_stop_streaming(this))
      {
        AK0991X_INST_PRINT(LOW, this, "dae_if_stop_streaming");
        state->config_step = AK0991X_CONFIG_STOPPING_STREAM;
      }
      if ((AK0991X_CONFIG_IDLE == state->config_step || AK0991X_CONFIG_UPDATING_HW == state->config_step)
           && ak0991x_dae_if_start_streaming(this))
      {
        AK0991X_INST_PRINT(LOW, this, "ak0991x_dae_if_start_streaming");
        state->config_step = AK0991X_CONFIG_UPDATING_HW;
      }
    }
#endif //AK0991X_ENABLE_DAE

    if (state->config_step == AK0991X_CONFIG_IDLE)
    {
      ak0991x_continue_client_config(this, true);
    }
  }
  else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    state->system_time = sns_get_system_time();

    if(ak0991x_dae_if_available(this))
    {
      if(state->in_clock_error_procedure)
      {
        AK0991X_INST_PRINT(LOW, this, "Flush requested in DAE during Clock Error Procedure.");
        state->do_flush_after_clock_error_procedure = true;
      }
      else
      {
        AK0991X_INST_PRINT(LOW, this, "FLUSH requested in DAE at %u, requesed_in_dae=%d",
            (uint32_t)state->system_time, state->flush_requested_in_dae);

        // During configuration. Wait for send config...
        if(state->config_step != AK0991X_CONFIG_IDLE)
        {
          AK0991X_INST_PRINT(LOW, this, "Flush requested in DAE during change config. Wait for send config event...");
          state->do_flush_after_change_config = true;
        }
        else
        {
          if(!state->flush_requested_in_dae)
          {
            state->flush_requested_in_dae = true;
            if(state->mag_info.use_fifo && state->mag_info.cur_cfg.fifo_wmk > 1)
            {
              ak0991x_dae_if_flush_hw(this);
            }
            else if(state->mag_info.cur_cfg.dae_wmk > 1)
            {
              ak0991x_dae_if_flush_samples(this);
            }
            else
            {
              ak0991x_send_fifo_flush_done(this);
            }
          }
        }
      }
    }
    else
    {
      AK0991X_INST_PRINT(LOW, this, "Flush requested at %u", (uint32_t)state->system_time);
      state->fifo_flush_in_progress = true;
      if (NULL != state->interrupt_data_stream)
      {
        sns_sensor_event *event =
          state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
        if(NULL == event || SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT != event->message_id)
        {
          if(state->mag_info.use_fifo || (state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING))
          {
            ak0991x_read_mag_samples(this);
          }
        }
      }
      else
      {
        if(state->mag_info.use_fifo || (state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING))
        {
          ak0991x_read_mag_samples(this);
        }
      }
    }
  }
  else if (client_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    // If the sensor is streaming and there is a client_request to run self-test,
    // the existing stream can be temporarily paused.
    // When the self-test completes, the paused stream shall be restarted.
    if (state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
    {
      state->system_time = sns_get_system_time();

      //////////////////////////////////////////////////////////////////
      // 1. store current ODR.
      mag_chosen_sample_rate_reg_value = state->mag_info.cur_cfg.odr;

      //////////////////////////////////////////////////////////////////
      // 2. pause streaming.
      AK0991X_INST_PRINT(LOW, this, "Pause streaming before self-test.");

      if(!ak0991x_dae_if_available(this))
      {
        // care the FIFO buffer if enabled FIFO and already streaming
        ak0991x_care_fifo_buffer(this);
      }

      state->mag_info.cur_cfg.odr = AK0991X_MAG_ODR_OFF;

      if(!ak0991x_dae_if_available(this))
      {
        ak0991x_reconfig_hw(this, false);
        if(state->timer_data_stream != NULL )
        {
          sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
        }
      }
      else  // DAE enabled
      {
        ak0991x_unregister_heart_beat_timer(this);

        if (AK0991X_CONFIG_IDLE == state->config_step &&
            ak0991x_dae_if_stop_streaming(this))
        {
          AK0991X_INST_PRINT(LOW, this, "done dae_if_stop_streaming");
          state->config_step = AK0991X_CONFIG_STOPPING_STREAM;
        }
        ak0991x_dae_if_process_events(this);
      }

      //////////////////////////////////////////////////////////////////
      // 3. run self-test.
      AK0991X_INST_PRINT(LOW, this, "Execute the self-test.");
      ak0991x_run_self_test(this);
      //////////////////////////////////////////////////////////////////
      // 4. clear events during the test.
      ak0991x_clear_old_events(this);
      state->system_time = sns_get_system_time();

      //////////////////////////////////////////////////////////////////
      // 5. resume streaming using stored ODR
      AK0991X_INST_PRINT(LOW, this, "Resume streaming after self-test.");

      state->mag_info.cur_cfg.odr = mag_chosen_sample_rate_reg_value;

      if(!ak0991x_dae_if_available(this))
      {
        ak0991x_continue_client_config(this, false);  // no reset. keep
      }
      else  // DAE enabled
      {
        if (state->config_step == AK0991X_CONFIG_IDLE || state->config_step == AK0991X_CONFIG_UPDATING_HW)
        {
          ak0991x_dae_if_start_streaming(this);
          state->config_step = AK0991X_CONFIG_UPDATING_HW;
        }
        ak0991x_dae_if_process_events(this);
      }
      state->in_self_test = false;
    }
    else
    {
      AK0991X_INST_PRINT(LOW, this, "SENSOR_TEST_CONFIG for selftest" );
      ak0991x_run_self_test(this);
      AK0991X_INST_PRINT(LOW, this, "selftest done!" );
    }
  }

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);

  return SNS_RC_SUCCESS;
}

