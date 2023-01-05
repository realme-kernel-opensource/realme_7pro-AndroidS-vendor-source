/**
 * @file sns_lsm6dso_s4s_island_stub.c
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
#if !LSM6DSO_S4S_ENABLED

float lsm6dso_cal_sync_odr(sns_sensor_instance *const this, uint32_t ideal_sync_interval, uint16_t t_ph)
{
  UNUSED_VAR(this);
  UNUSED_VAR(ideal_sync_interval);
  UNUSED_VAR(t_ph);
  return SNS_RC_NOT_SUPPORTED;
}

sns_time lsm6dso_cal_sync_period(sns_sensor_instance *const this, uint32_t ideal_sync_interval, uint16_t t_ph)
{
  UNUSED_VAR(this);
  UNUSED_VAR(ideal_sync_interval);
  UNUSED_VAR(t_ph);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_get_s4s_sample_interval(sns_sensor_instance *const this,
    uint16_t total_samples,
    sns_time* sample_interval,
    sns_time* first_ts,
    sns_time* end_ts)
{
  UNUSED_VAR(this);
  UNUSED_VAR(total_samples);
  UNUSED_VAR(sample_interval);
  UNUSED_VAR(first_ts);
  UNUSED_VAR(end_ts);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_set_s4s_config(sns_sensor_instance *const this, lsm6dso_s4s_config* s4s_config_req)
{
  UNUSED_VAR(this);
  UNUSED_VAR(s4s_config_req);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_send_st(sns_sensor_instance *const this, sns_time host_sync_ts)
{
  UNUSED_VAR(this);
  UNUSED_VAR(host_sync_ts);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_send_dt(sns_sensor_instance *const this, bool abort)
{
  UNUSED_VAR(this);
  UNUSED_VAR(abort);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_update_s4s_sync(sns_sensor_instance *const this, uint16_t odr)
{
  UNUSED_VAR(this);
  UNUSED_VAR(odr);
  return SNS_RC_NOT_SUPPORTED;
}
sns_rc lsm6dso_start_s4s_sync(sns_sensor_instance *const this, uint16_t odr, sns_time host_sync_ts)
{
  UNUSED_VAR(this);
  UNUSED_VAR(odr);
  UNUSED_VAR(host_sync_ts);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dso_handle_polling_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(this);
  UNUSED_VAR(timestamp);
  UNUSED_VAR(latest_timer_event);
  return SNS_RC_SUCCESS;
}

#endif
