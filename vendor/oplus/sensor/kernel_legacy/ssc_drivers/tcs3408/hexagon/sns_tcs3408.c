/*
 * Copyright (c) 2018, ams AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "sns_rc.h"
#include "sns_register.h"
#ifdef TCS3408_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

#include "sns_tcs3408_sensor.h"
#include "sns_tcs3408_sensor_instance.h"

/* Public Function Definitions. */
sns_rc sns_register_tcs3408(sns_register_cb const *register_api)
{
#ifdef TCS3408_GET_PARAMETER_FROM_SMEM
	struct sensor_hw *cct_hw = NULL;

	if (!oppo_get_sensor_hw(OPPO_CCT, 0x01, &cct_hw) || //0x01 - tcs3408
        (cct_hw && cct_hw->feature.feature[0])) {
		return SNS_RC_SUCCESS;
	}
#endif
	/* Register Ambient Light Sensor. */
	register_api->init_sensor(sizeof(tcs3408_state), &tcs3408_als_sensor_api,
							&tcs3408_sensor_instance_api);

	/* Register RGB Sensor. */
	register_api->init_sensor(sizeof(tcs3408_state), &tcs3408_rgb_sensor_api,
							&tcs3408_sensor_instance_api);

	/* Register flicker Sensor. */
	register_api->init_sensor(sizeof(tcs3408_state), &tcs3408_flicker_sensor_api,
							&tcs3408_sensor_instance_api);

	return SNS_RC_SUCCESS;
}
