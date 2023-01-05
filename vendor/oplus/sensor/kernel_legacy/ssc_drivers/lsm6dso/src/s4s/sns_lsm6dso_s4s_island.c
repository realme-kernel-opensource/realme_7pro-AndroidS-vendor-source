/**
 * @file sns_lsm6dso_s4s_island.c
 *
 * LSM6DSO Accel sync implementation.
 *
 * Copyright (c) 2020, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
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
 *
 **/

#include "sns_rc.h"
#include "sns_types.h"

#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"
#if LSM6DSO_S4S_ENABLED
#include "sns_lsm6dso_s4s.h"

#include "sns_printf.h"
extern const odr_reg_map lsm6dso_odr_map[];
const float lsm6dso_native_odr[]   = {0, 13, 26, 52, 104, 208, 416,  832, 1664, 3328, 6656};
const float lsm6dso_s4s_odr_min[]  = {0,  8, 17, 34,  67, 134, 267,  534, 1066, 2133, 4266};
const float lsm6dso_s4s_odr_norm[] = {0, 13, 25, 50, 100, 200, 400,  800, 1600, 3200, 6400};
const float lsm6dso_s4s_odr_max[]  = {0, 16, 33, 66, 133, 266, 533, 1065, 2132, 4265, 8532};
#define GET_S4S_INTVL( num_samples, ideal_sync_interval, t_ph ) ((num_samples)*(ideal_sync_interval) / (t_ph))


// compute s4s odr after tph stretching.
float lsm6dso_cal_sync_odr(sns_sensor_instance *const this, uint32_t ideal_sync_interval, uint16_t t_ph)
{
  UNUSED_VAR(this);
  float sync_odr;
  /*
  // sync_odr/norminal_odr=nominal_sync_interval/ideal_sync_interval
  int nominal_sync_interval = DD_ms2tick(lsm6dso_s4s_tph500ms[tph_index]);
  float nominal_odr = lsm6dso_s4s_odr_norm[odr_index];
  sync_odr = nominal_odr * nominal_sync_interval / s4s_config->ideal_sync_interval;
  */
  t_ph &= 0xFFFE; // t_ph must be even number.
  sync_odr = 19200 * 1000 * t_ph/ideal_sync_interval; //(19.2MHz clock)
  return sync_odr;
}

sns_time lsm6dso_cal_sync_period(sns_sensor_instance *const this, uint32_t ideal_sync_interval, uint16_t t_ph)
{
  UNUSED_VAR(this);
  sns_time sample_period;
  t_ph &= 0xFFFE; // t_ph must be even number.
  sample_period = ideal_sync_interval / t_ph;
  return sample_period;
}

sns_rc lsm6dso_get_s4s_sample_interval(sns_sensor_instance *const this,
    uint16_t total_samples,
    sns_time* sample_interval,
    sns_time* first_ts,
    sns_time* end_ts)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  UNUSED_VAR(total_samples);
  UNUSED_VAR(end_ts);
  DBG_INST_PRINTF(HIGH, this, "sample_int=%u total_samples=%d sync_state=%d",
      (uint32_t)*sample_interval, total_samples, state->fifo_info.bh_info.sync_state);
  if(state->fifo_info.bh_info.sync_state == LSM6DSO_S4S_1ST_SYNCED ||
      state->fifo_info.bh_info.sync_state == LSM6DSO_S4S_SYNCED) {
    *sample_interval = GET_S4S_INTVL( 1, state->fifo_info.bh_info.ideal_sync_interval, state->fifo_info.bh_info.t_ph );
    *first_ts = state->fifo_info.last_timestamp + *sample_interval;
  } else if(state->fifo_info.bh_info.sync_state == LSM6DSO_S4S_OFF ||
      state->fifo_info.bh_info.sync_state == LSM6DSO_S4S_NOT_SYNCED) {
      //recalculate sample interval
      sns_time cal_st = lsm6dso_get_sample_interval(this, (float)state->current_conf.odr);
      DBG_INST_PRINTF(HIGH, this, "cal_st=%u odr_drift_max=0x%x sample_interval=%u",
          (uint32_t)cal_st, (int32_t)(state->odr_drift_max*1000),(uint32_t)*sample_interval);
      *first_ts += cal_st - *sample_interval;
      *sample_interval = cal_st;
    //*first_ts = *end_ts - (total_samples-1) * *sample_interval;
  } else {
    return SNS_RC_FAILED;
  }
  DBG_INST_PRINTF(HIGH, this, "sample_int=%u first_ts=%u end_ts=%u",
      (uint32_t)*sample_interval, (uint32_t)*first_ts, (uint32_t)*end_ts);
  return SNS_RC_SUCCESS;
}

sns_rc lsm6dso_set_s4s_config(sns_sensor_instance *const this, lsm6dso_s4s_config* s4s_config_req)
{
  sns_rc status = SNS_RC_SUCCESS;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint32_t xfer_bytes;
  int rr_per_tph;
  if (s4s_config_req->t_ph == 0)  // disable S4S
  {
    state->s4s_info.ctrl_regs.S4sTphL=state->s4s_info.ctrl_regs.S4sTphH = 0;

    status |= lsm6dso_com_write_wrapper(this,
        LSM6DSO_S4S_TPH_L,
        &state->s4s_info.ctrl_regs.S4sTphL,
        2,
        &xfer_bytes,
        false);
    SNS_INST_PRINTF(HIGH, this, "s4s_config disable s4s(tph_l):0x%x",state->s4s_info.ctrl_regs.S4sTphL);
  }
  else
  {
    // set TPHL, TPHH, RR registers
    if (s4s_config_req->t_ph <= 254)
    {
      state->s4s_info.ctrl_regs.S4sTphL = s4s_config_req->t_ph >> 1;
      state->s4s_info.ctrl_regs.S4sTphH = 0;
    }
    else
    {
      state->s4s_info.ctrl_regs.S4sTphL = 0x80 | ((s4s_config_req->t_ph & 0x00FF) >> 1);
      state->s4s_info.ctrl_regs.S4sTphH=s4s_config_req->t_ph >> 8;
    }
    state->s4s_info.ctrl_regs.S4sRr=s4s_config_req->rr;
    //set tph
    status |= lsm6dso_com_write_wrapper(this,
        LSM6DSO_S4S_TPH_L,
        &state->s4s_info.ctrl_regs.S4sTphL,
        2,
        &xfer_bytes,
        false);
    //set rr
    status |= lsm6dso_com_write_wrapper(this,
        LSM6DSO_S4S_RR,
        &state->s4s_info.ctrl_regs.S4sRr,
        1,
        &xfer_bytes,
        false);

    SNS_INST_PRINTF(HIGH, this, "s4s_config(tph_l,tph_h,rr):0x%x,0x%x, 0x%x",state->s4s_info.ctrl_regs.S4sTphL, state->s4s_info.ctrl_regs.S4sTphH, state->s4s_info.ctrl_regs.S4sRr);
    rr_per_tph = 1 << (11 + s4s_config_req->rr);
    state->s4s_info.ticks_per_rr = (s4s_config_req->ideal_sync_interval + rr_per_tph / 2) / rr_per_tph;  // ticks_per_rr=ideal_sync_interval/2^(11+rr)
    SNS_INST_PRINTF(HIGH, this, "s4s_config(rr_per_tph, ticks_per_rr,ideal_sync_inte):%d,%d,%d",rr_per_tph, state->s4s_info.ticks_per_rr, s4s_config_req->ideal_sync_interval);
    if (state->s4s_info.ticks_per_rr == 0)
      state->s4s_info.ticks_per_rr = 1; //ticks_per_rr can't be 0.
  }
  state->s4s_info.current_conf = *s4s_config_req;
  return status;
}

sns_rc lsm6dso_send_st(sns_sensor_instance *const this, sns_time host_sync_ts)
{
  sns_rc status = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;

  uint8_t st_data = 0xCC;
  state->s4s_info.host_sync_ts = host_sync_ts;
  //TODO: is timer event timestamp accurate to use?
  //state->s4s_info.host_sync_ts = sns_get_system_time();
  status |= lsm6dso_com_write_wrapper(this,
      LSM6DSO_S4S_ST,
      &st_data,
      1,
      &xfer_bytes,
      true);
  status |= state->scp_service->api->sns_scp_get_write_time(state->com_port_info.port_handle,
      &state->s4s_info.st_ts);
  return status;
}

sns_rc lsm6dso_send_dt(sns_sensor_instance *const this, bool abort)
{
  sns_rc status;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  uint32_t xfer_bytes;
  uint32_t delay_time;
  uint8_t dt_data;
  if(!abort) {
    delay_time = (state->s4s_info.st_ts - state->s4s_info.host_sync_ts + state->s4s_info.ticks_per_rr / 2) / state->s4s_info.ticks_per_rr;
    dt_data = (delay_time <= 127) ? delay_time : 0x80;
    state->s4s_info.last_effect_sync_ts = state->s4s_info.host_sync_ts;
  } else {
    dt_data=0x80;
  }
  status = lsm6dso_com_write_wrapper(this,
      LSM6DSO_S4S_DT,
      &dt_data,
      1,
      &xfer_bytes,
      false);

  SNS_INST_PRINTF(HIGH, this, "StDt:%u,%u,0x%02x",(uint32_t)state->s4s_info.host_sync_ts, (uint32_t)state->s4s_info.st_ts, dt_data);
  return status;
}

sns_rc lsm6dso_update_s4s_sync(sns_sensor_instance *const this, uint16_t odr)
{
  sns_rc status=SNS_RC_SUCCESS;
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_s4s_info* s4s_info = &state->s4s_info;
  //if(s4s_info->odr == odr)
  //  return status;

  switch(s4s_info->sync_state) {
    case LSM6DSO_S4S_OFF:
    case LSM6DSO_S4S_NOT_SYNCED:
      s4s_info->sync_state = LSM6DSO_S4S_OFF;
      break;

    case LSM6DSO_S4S_SYNCING:
      lsm6dso_send_st(this, sns_get_system_time());
      lsm6dso_send_dt(this, true);
      s4s_info->sync_state = LSM6DSO_S4S_OFF;
      break;

    case  LSM6DSO_S4S_1ST_SYNCED:
    case  LSM6DSO_S4S_2ND_SYNCED:
    case  LSM6DSO_S4S_SYNCED:
      //s4s_info->sync_state = LSM6DSO_S4S_CONFIG_CHANGE;
      s4s_info->sync_state = LSM6DSO_S4S_1ST_SYNCED;
      break;

    default:
      break;
  }
  SNS_INST_PRINTF(HIGH, this, "update_sync:: sync_state %d odr %d",state->s4s_info.sync_state, odr);
  return status;
}

sns_rc lsm6dso_start_s4s_sync(sns_sensor_instance *const this, uint16_t odr, sns_time host_sync_ts)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  lsm6dso_s4s_info* s4s_info = &state->s4s_info;
  lsm6dso_s4s_config desired_conf;

  desired_conf.fifo_int_en = false;
  s4s_info->is_sync_updated = false;
  SNS_INST_PRINTF(HIGH, this, "start_sync:: sync_state %d odr %d",state->s4s_info.sync_state, odr);
  switch(s4s_info->sync_state) {
    case LSM6DSO_S4S_OFF:
      desired_conf.t_ph = (odr > LSM6DSO_ODR_208) ? odr/25 : 8;
      desired_conf.t_ph = (desired_conf.t_ph/4)*4 ;
      desired_conf.rr = 3;
      desired_conf.ideal_sync_interval = sns_convert_ns_to_ticks(1000000000.0/odr) * desired_conf.t_ph;
      lsm6dso_set_s4s_config(this, &desired_conf);
      s4s_info->sync_time = ceil((double)(200 * odr ) / 1000); //200ms convert to samples
      s4s_info->poll_time = s4s_info->sync_time;
      if(state->fifo_info.cur_wmk  < s4s_info->sync_time) {
        s4s_info->poll_time =  state->fifo_info.cur_wmk;
        while(s4s_info->sync_time % s4s_info->poll_time)
          s4s_info->poll_time = --s4s_info->poll_time & 0xFFFE;
      }

      //s4s_info->sync_time = sns_convert_ns_to_ticks(200000000.0); //200ms
      //s4s_info->sync_time = ceil((double)(200 * odr ) / 1000); //200ms
      s4s_info->is_sync_updated = true;
      s4s_info->sync_state = LSM6DSO_S4S_NOT_SYNCED;
      break;
    case LSM6DSO_S4S_NOT_SYNCED:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, false);
      //s4s_info->sync_time = sns_convert_ns_to_ticks(1000000000.0/odr) * s4s_info->current_conf.t_ph; //tph/2
      s4s_info->sync_time = s4s_info->current_conf.t_ph;
      s4s_info->poll_time = s4s_info->sync_time;
      if(state->fifo_info.cur_wmk  < s4s_info->sync_time) {
        s4s_info->poll_time =  state->fifo_info.cur_wmk;
        while(s4s_info->sync_time % s4s_info->poll_time)
          s4s_info->poll_time = --s4s_info->poll_time & 0xFFFE;
      }

      //s4s_info->sync_time = s4s_info->current_conf.t_ph; //tph/2
      s4s_info->is_sync_updated = true;
      s4s_info->sync_state = LSM6DSO_S4S_SYNCING;
      break;

    case LSM6DSO_S4S_SYNCING:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, false);
      //s4s_info->sync_time = sns_convert_ns_to_ticks(1000000000.0/odr); //1 odr
      s4s_info->sync_time = 1; //1 odr
      s4s_info->poll_time = 1; //1 odr
      s4s_info->is_sync_updated = true;
      s4s_info->sync_state = LSM6DSO_S4S_1ST_SYNCED;
      break;

    case LSM6DSO_S4S_1ST_SYNCED:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, true);

      desired_conf.t_ph = odr/4*4;
      if(state->fifo_info.cur_wmk >= desired_conf.t_ph) {
       s4s_info->poll_time = desired_conf.t_ph;
      } else {
        uint16_t mul =  desired_conf.t_ph / state->fifo_info.cur_wmk;
        if((desired_conf.t_ph %  state->fifo_info.cur_wmk) > 0.7 * state->fifo_info.cur_wmk)
          ++mul;
        desired_conf.t_ph = mul * state->fifo_info.cur_wmk;
        //t_ph should always be even number
        if(desired_conf.t_ph & 0x1)
          ++desired_conf.t_ph;
        s4s_info->poll_time = state->fifo_info.cur_wmk;
      }
      s4s_info->sync_time = desired_conf.t_ph;

      desired_conf.rr = 3;
      desired_conf.ideal_sync_interval = sns_convert_ns_to_ticks(1000000000.0/odr) * desired_conf.t_ph ;
      lsm6dso_set_s4s_config(this, &desired_conf);
      //s4s_info->sync_time = 1; //1 odr
      //s4s_info->poll_time = 1; //1 odr
      //s4s_info->is_sync_updated = true;

      s4s_info->sync_state = LSM6DSO_S4S_2ND_SYNCED;
      //break;
      // continue to 2nd sync
    case LSM6DSO_S4S_2ND_SYNCED:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, false);
      //s4s_info->sync_time = sns_convert_ns_to_ticks(1000000000.0); //tph
      //s4s_info->sync_time = s4s_info->current_conf.t_ph;//tph
      s4s_info->is_sync_updated = true;
      lsm6dso_send_config_event(this, false);
      s4s_info->sync_state = LSM6DSO_S4S_SYNCED;
      break;

    case LSM6DSO_S4S_SYNCED:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, false);
      //s4s_info->sync_time = s4s_info->current_conf.t_ph;//tph
      //s4s_info->sync_time = sns_convert_ns_to_ticks(1000000000.0); //tph
      s4s_info->is_sync_updated = false;
      s4s_info->sync_state = LSM6DSO_S4S_SYNCED;
      break;
    case LSM6DSO_S4S_CONFIG_CHANGE:
      lsm6dso_send_st(this, host_sync_ts);
      lsm6dso_send_dt(this, true);
      desired_conf.t_ph = odr/4*4;
      desired_conf.rr = 3;
      desired_conf.ideal_sync_interval = sns_convert_ns_to_ticks(1000000000.0);
      lsm6dso_set_s4s_config(this, &desired_conf);
      //calculate ODR boundary
      s4s_info->sync_time = s4s_info->current_conf.t_ph;//tph
      //s4s_info->sync_time = sns_convert_ns_to_ticks(1000000000.0); //tph
      s4s_info->is_sync_updated = true;
      s4s_info->sync_state = LSM6DSO_S4S_2ND_SYNCED;
    default:
      break;
  }
  s4s_info->odr = odr;
  s4s_info->sync_poll_count = s4s_info->sync_time/s4s_info->poll_time;
  return SNS_RC_SUCCESS;
}

void lsm6dso_handle_polling_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(timestamp);
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if(state->fifo_info.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO))
  {
    if(state->ag_stream_mode == S4S_SYNC)  {
      if(state->s4s_info.sync_poll_count)
        state->s4s_info.sync_poll_count--;

      DBG_INST_PRINTF(HIGH, this, "sync_poll_count %d req_timeout %u act_timeout %u",
          state->s4s_info.sync_poll_count, (uint32_t)latest_timer_event->requested_timeout_time, (uint32_t)latest_timer_event->timeout_time);

      state->fifo_info.th_info.sync_state = state->s4s_info.sync_state;
      state->fifo_info.th_info.last_sync_ts = state->s4s_info.last_effect_sync_ts;
      state->fifo_info.th_info.ideal_sync_interval = state->s4s_info.current_conf.ideal_sync_interval;
      state->fifo_info.th_info.t_ph = state->s4s_info.current_conf.t_ph;

      if(state->s4s_info.sync_poll_count == 0) {
        //state->s4s_info.sync(this, state->fifo_info.publish_sensors & (LSM6DSO_ACCEL | LSM6DSO_GYRO), state->current_conf.odr, sns_get_system_time());
        lsm6dso_start_s4s_sync(this, state->current_conf.odr, latest_timer_event->timeout_time);
        if(state->s4s_info.is_sync_updated)
          lsm6dso_run_polling_timer(this);
      }
    }
    //state->fifo_info.th_info.interrupt_fired = true;
    //state->fifo_info.th_info.interrupt_ts = timer_event.timeout_time;
    lsm6dso_read_fifo_data(this, latest_timer_event->timeout_time, true);
  }
  lsm6dso_restart_hb_timer(this, false);
  lsm6dso_handle_ois_polling_timer_events(this, timestamp, latest_timer_event);
  return rv;
}

#endif
