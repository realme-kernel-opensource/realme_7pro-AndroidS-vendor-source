/**
 * @file sns_lps22hx.c
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
#include "sns_rc.h"
#include "sns_register.h"
#include "sns_lps22hx_sensor.h"
#include "sns_lps22hx_sensor_instance.h"
#include "sns_types.h"
#ifdef LPS22HX_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

/** Public Function Definitions. */

sns_rc sns_register_lps22hx(sns_register_cb const *register_api)
{
#ifdef LPS22HX_GET_PARAMETER_FROM_SMEM
	struct sensor_hw *hw = NULL;
	if(oppo_get_sensor_hw(OPPO_BAROMETER, LPS22HH, &hw)) {
#endif
	  int i = 0;
	  /** Register Sensors */
	  for(i = 0; i< ARR_SIZE(lps22hx_supported_sensors) ; i++) {
		register_api->init_sensor(sizeof(lps22hx_state), lps22hx_supported_sensors[i].sensor_api,
			lps22hx_supported_sensors[i].instance_api);
	  }
#ifdef LPS22HX_GET_PARAMETER_FROM_SMEM
	}
#endif
  return SNS_RC_SUCCESS;
}
