#pragma once
/**
 * @file sns_ak0991x_dae_if.h
 *
 * DAE sensor interface
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_ak0991x_lite.h"

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_stream_service.h"

struct sns_stream_service;
struct sns_data_stream;
struct ak0991x_state;

typedef enum
{
  PRE_INIT,
  INIT_PENDING,
  UNAVAILABLE,
  IDLE,
  STREAM_STARTING,
  STREAMING,
  STREAM_STOPPING,
//  PAUSE_S4S_SCHEDULE,
} ak0991x_dae_if_state;

typedef struct
{
#ifdef AK0991X_ENABLE_DAE
  struct sns_data_stream *stream;
  const char             *nano_hal_vtable_name;
  uint8_t                status_bytes_per_fifo;
  bool                   stream_usable:1;
  bool                   flushing_hw:1;
  bool                   flushing_data:1;
#endif
  ak0991x_dae_if_state   state;
} ak0991x_dae_stream;

typedef struct ak0991x_dae_if_info
{
  ak0991x_dae_stream   mag;
} ak0991x_dae_if_info;

void ak0991x_dae_if_check_support(sns_sensor *this);

void ak0991x_dae_if_process_sensor_events(sns_sensor *this);

bool ak0991x_dae_if_available(sns_sensor_instance *this);

bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this);

bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this);

sns_rc ak0991x_dae_if_init(
  sns_sensor_instance       *const this,
  struct sns_stream_service *stream_mgr,
  sns_sensor_uid            *dae_suid,
  struct ak0991x_state      *sensor_state);

void ak0991x_dae_if_deinit(sns_sensor_instance *this);

//bool ak0991x_dae_if_pause_s4s_schedule(sns_sensor_instance *this);

bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this);

bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this);

bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this);

bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this);

void ak0991x_dae_if_process_events(sns_sensor_instance *this);

