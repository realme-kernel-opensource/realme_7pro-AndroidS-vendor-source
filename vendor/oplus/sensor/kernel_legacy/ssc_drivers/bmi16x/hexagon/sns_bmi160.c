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

#include "sns_rc.h"
#include "sns_register.h"
#include "sns_bmi160_sensor.h"
#include "sns_bmi160_sensor_instance.h"
#ifdef BMI160_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

/** Public Function Definitions. */

sns_rc sns_register_bmi160(sns_register_cb const *register_api)
{

#ifdef BMI160_GET_PARAMETER_FROM_SMEM
	struct sensor_hw *acc_hw = NULL;

	if (!oppo_get_sensor_hw(OPPO_ACCEL, BMI160, &acc_hw)) {
		return SNS_RC_SUCCESS;
	}
#endif
    /** Register Accel Sensor. */
    register_api->init_sensor(sizeof(bmi160_state), &bmi160_accel_sensor_api,
                                &bmi160_sensor_instance_api);

    /** Register Gyro Sensor. */
    register_api->init_sensor(sizeof(bmi160_state), &bmi160_gyro_sensor_api,
                                &bmi160_sensor_instance_api);

  /** Register Motion Accel Sensor. */
    register_api->init_sensor(sizeof(bmi160_state), &bmi160_motion_detect_sensor_api,
                                &bmi160_sensor_instance_api);

    /** Register Sensor Temperature Sensor. */
    register_api->init_sensor(sizeof(bmi160_state), &bmi160_sensor_temp_sensor_api,
                                &bmi160_sensor_instance_api);

#if BMI160_CONFIG_ENABLE_PEDO
  register_api->init_sensor(sizeof(bmi160_state), &bmi160_pedo_api,
      &bmi160_sensor_instance_api);
#endif

  return SNS_RC_SUCCESS;
}
