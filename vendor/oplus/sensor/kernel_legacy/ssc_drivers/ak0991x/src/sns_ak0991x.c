/**
 * @file sns_ak0991x.c
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * All Rights Reserved.
 *
 * Copyright (c) 2016-2018 Asahi Kasei Microdevices
 * Confidential and Proprietary - Asahi Kasei Microdevices
 **/

/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/

#include "sns_rc.h"
#include "sns_register.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_hal.h"
#ifdef AK0991X_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

/**
*****************************************************************************************
                            Public Function Definitions
*****************************************************************************************
*/

sns_rc sns_register_ak0991x(sns_register_cb const *register_api)
{
#ifdef AK0991X_GET_PARAMETER_FROM_SMEM
  struct sensor_hw *mag_hw = NULL;
  if (oppo_get_sensor_hw(OPPO_MAGNETIC, AKM0991X, &mag_hw)) {
#endif

  /** Register Magnetic Sensor. */
	register_api->init_sensor(sizeof(ak0991x_state), &ak0991x_mag_sensor_api,
                            &ak0991x_sensor_instance_api);
#ifdef AK0991X_GET_PARAMETER_FROM_SMEM
  }
#endif
  return SNS_RC_SUCCESS;
}
