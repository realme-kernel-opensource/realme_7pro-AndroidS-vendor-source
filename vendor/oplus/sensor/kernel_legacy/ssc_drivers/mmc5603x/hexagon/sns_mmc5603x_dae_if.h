#pragma once
/**
 * @file sns_mmc5603x_dae_if.h
 *
 * DAE sensor interface
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_mmc5603x_lite.h"

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_stream_service.h"

struct sns_stream_service;
struct sns_data_stream;
struct mmc5603x_instance_state;
struct mmc5603x_state;
typedef enum
{
  PRE_INIT,
  INIT_PENDING,
  UNAVAILABLE,
  IDLE,
  STREAM_STARTING,
  STREAMING,
  STREAM_STOPPING,

} mmc5603x_dae_if_state;

typedef struct
{
#ifdef MMC5603X_ENABLE_DAE
  struct sns_data_stream *stream;
  const char             *nano_hal_vtable_name;
  uint8_t                status_bytes_per_fifo;
  bool                   stream_usable:1;
  bool                   flushing_hw:1;
  bool                   flushing_data:1;
#endif
  mmc5603x_dae_if_state   state;
} mmc5603x_dae_stream;

typedef struct mmc5603x_dae_if_info
{
  mmc5603x_dae_stream   mag;
} mmc5603x_dae_if_info;

void mmc5603x_dae_if_check_support(sns_sensor *this);

void mmc5603x_dae_if_process_sensor_events(sns_sensor *this);

bool mmc5603x_dae_if_available(sns_sensor_instance *this);

bool mmc5603x_dae_if_is_initializing(sns_sensor_instance *this);

bool mmc5603x_dae_if_is_streaming(sns_sensor_instance *this);

sns_rc mmc5603x_dae_if_init(
  sns_sensor_instance       *const this,
  sns_stream_service *stream_mgr,
  sns_sensor_uid            *dae_suid,
  struct mmc5603x_state      *sensor_state);

void mmc5603x_dae_if_deinit(sns_sensor_instance *this);

bool mmc5603x_dae_if_stop_streaming(sns_sensor_instance *this);

bool mmc5603x_dae_if_start_streaming(sns_sensor_instance *this);

bool mmc5603x_dae_if_flush_hw(sns_sensor_instance *this);

bool mmc5603x_dae_if_flush_samples(sns_sensor_instance *this);

void mmc5603x_dae_if_process_events(sns_sensor_instance *this);
