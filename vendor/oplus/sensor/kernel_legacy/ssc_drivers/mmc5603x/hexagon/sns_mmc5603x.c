/**
 * @file sns_mmc5603x.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/

#include "sns_rc.h"
#include "sns_register.h"
#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_sensor_instance.h"
#include "sns_mmc5603x_hal.h"
#ifdef MMC5603X_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif
/**
*****************************************************************************************
                            Public Function Definitions
*****************************************************************************************
*/

sns_rc sns_register_mmc5603x(sns_register_cb const *register_api)
{
#ifdef MMC5603X_GET_PARAMETER_FROM_SMEM
	struct sensor_hw *mag_hw = NULL;
	if (oppo_get_sensor_hw(OPPO_MAGNETIC, MMC5603, &mag_hw)) {
#endif
  /** Register Magnetic Sensor. */
		 register_api->init_sensor(sizeof(mmc5603x_state), &mmc5603x_mag_sensor_api,
                            &mmc5603x_sensor_instance_api);
#ifdef MMC5603X_GET_PARAMETER_FROM_SMEM
	}
#endif
  return SNS_RC_SUCCESS;
}
