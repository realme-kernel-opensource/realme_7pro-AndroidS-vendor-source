/**
 * @file sns_lsm6dso_ois_stub.c
 *
 * LSM6DSO OIS Sensor implementation.
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

#include <string.h>
#include "sns_register.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_ois.h"
#include "pb_encode.h"
#include "sns_service_manager.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"

#if !LSM6DSO_OIS_ENABLED

void lsm6dso_send_ois_registry_requests(sns_sensor *const this, uint8_t hw_id)
{
  UNUSED_VAR(this);
  UNUSED_VAR(hw_id);
}

void lsm6dso_process_ois_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  UNUSED_VAR(this);
  UNUSED_VAR(event);
}

void lsm6dso_update_ois_resolution_idx(sns_sensor *const this, uint8_t res_idx)
{
  UNUSED_VAR(this);
  UNUSED_VAR(res_idx);
}

void lsm6dso_ois_register(sns_register_cb const *register_api)
{
  UNUSED_VAR(register_api);
}

#endif //LSM6DSO_OIS_ENABLED