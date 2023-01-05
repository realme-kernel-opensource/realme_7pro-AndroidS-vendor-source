/**
 * @file sns_mmc5603x_sensor_island.c
 *
 * Common implementation for MMC5603X Sensors.
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_types.h"

#include "sns_mmc5603x_sensor.h"
#include "sns_mmc5603x_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_attribute_util.h"
#include "sns_mmc5603x_ver.h"

static sns_sensor_uid const* mmc5603x_mag_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  MMC5603X_PRINT(HIGH, this, "mmc5603x_mag_get_sensor_uid driver version %d", MMC5603X_DRIVER_VERSION);
  static const sns_sensor_uid sensor_uid = MAG_SUID1;
  return &sensor_uid;
}

sns_sensor_api mmc5603x_mag_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &mmc5603x_mag_init,
#ifdef MMC5603X_ENABLE_DEINIT
  .deinit             = &mmc5603x_mag_deinit,
#endif
  .get_sensor_uid     = &mmc5603x_mag_get_sensor_uid,
  .set_client_request = &mmc5603x_set_client_request,
  .notify_event       = &mmc5603x_sensor_notify_event,
};
