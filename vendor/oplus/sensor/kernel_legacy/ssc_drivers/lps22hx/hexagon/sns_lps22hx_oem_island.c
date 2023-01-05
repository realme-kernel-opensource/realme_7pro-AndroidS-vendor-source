/**
 * @file sns_lps22hx_oem_island.c
 *
 * LPS22HX OEM implementation.
 *
 * Copyright (c) 2018, STMicroelectronics.
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
 **/

#include "sns_types.h"
#include "pb_encode.h"
#include "sns_pb_util.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"

void lps22hx_inst_set_oem_client_config(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
}

bool lps22hx_set_oem_client_request(sns_sensor *const this,
    struct sns_request const *exist_request)
{
  UNUSED_VAR(this);
  UNUSED_VAR(exist_request);
  return true;
}

bool lps22hx_set_oem_client_request_msgid(sns_sensor *const this,
    struct sns_request const *new_request)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;
  if(new_request != NULL &&
     (new_request->message_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG))
  {
    DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
      "msg_id=%d", new_request->message_id);
    return false;
  }
  return true;
}

