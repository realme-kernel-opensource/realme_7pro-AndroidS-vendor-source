/**
 * @file sns_lsm6dso_ois_island_stub.c
 *
 * Implementation for OIS sensor island code.
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
#include <math.h>
#include "sns_cal_util.h"
#include <string.h>
#include "sns_math_util.h"
#include "sns_mem_util.h"

#include "sns_event_service.h"
#include "sns_diag_service.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_ois.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_stream_service.h"
#include "sns_types.h"

#include "sns_motion_detect.pb.h"
#include "sns_pb_util.h"
#include "sns_printf.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_diag.pb.h"
#include "sns_cal.pb.h"

#if !LSM6DSO_OIS_ENABLED

void lsm6dso_init_ois_instance(
  sns_sensor_instance *instance,
  lsm6dso_instance_config const *inst_cfg)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(inst_cfg);
}

void lsm6dso_store_ois_registry_data(
  sns_sensor_instance *instance,
  sns_sensor_state const *this)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(this);
}

void lsm6dso_config_ois(sns_sensor_instance *const instance)
{
  UNUSED_VAR(instance);
}

void lsm6dso_update_ois_sensor_config(
  sns_sensor *this,
  sns_sensor_instance *instance)
{
  UNUSED_VAR(this);
  UNUSED_VAR(instance);
}

void lsm6dso_handle_ois_interrupt(
  sns_sensor_instance *const instance,
  sns_time ts)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(ts);
}

bool lsm6dso_is_ois_request_present(sns_sensor_instance *instance)
{
  UNUSED_VAR(instance);
  return false;
}
sns_rc lsm6dso_handle_ois_polling_timer_events(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event)
{
  UNUSED_VAR(this);
  UNUSED_VAR(timestamp);
  UNUSED_VAR(latest_timer_event);
  return SNS_RC_NOT_SUPPORTED;
}

#endif