#pragma once
/**
 * @file sns_lsm6dso_dae_if.h
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

#include <stdint.h>
#include "sns_dae.pb.h"
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_stream_service.h"

struct sns_stream_service;
struct sns_data_stream;
struct lsm6dso_instance_state;
struct lsm6dso_instance_config;

typedef enum
{
  PRE_INIT,
  INIT_PENDING,
  UNAVAILABLE,
  IDLE,
  STREAM_STARTING,
  STREAMING,
  STREAM_STOPPING,

} lsm6dso_dae_if_state;

typedef struct
{
  struct sns_data_stream *stream;
  const char             *nano_hal_vtable_name;
  uint32_t               dae_wm;
  uint8_t                status_bytes_per_fifo;
  lsm6dso_dae_if_state   state;
  bool                   stream_usable:1;
  bool                   flushing_hw:1;
  bool                   flushing_data:1;
} lsm6dso_dae_stream;

typedef struct lsm6dso_dae_if_info
{
  lsm6dso_dae_stream   ag;
  lsm6dso_dae_stream   temp;
} lsm6dso_dae_if_info;

// for use by master sensor
void lsm6dso_dae_if_check_support(sns_sensor *this);
void lsm6dso_dae_if_process_sensor_events(sns_sensor *this);

sns_rc lsm6dso_dae_if_init(
  sns_sensor_instance                  *const this,
  struct sns_stream_service            *stream_mgr,
  struct lsm6dso_instance_config const *);

void lsm6dso_dae_if_deinit(sns_sensor_instance *const this);

bool lsm6dso_dae_if_stop_streaming(sns_sensor_instance *this, uint8_t sensors);

bool lsm6dso_dae_if_start_streaming(sns_sensor_instance *this);


void lsm6dso_dae_if_process_events(sns_sensor_instance *this);

// for use by instance
bool lsm6dso_dae_if_available(sns_sensor_instance *this);

bool lsm6dso_dae_if_flush_hw(sns_sensor_instance *this);


void lsm6dso_dae_if_build_static_config_request(
    struct lsm6dso_instance_config const *inst_cfg,
    sns_dae_set_static_config      *config_req,
    uint8_t                        hw_idx,
    uint8_t                        rigid_body_type,
    bool                           for_ag);

sns_rc lsm6dso_dae_if_send_static_config_request(
    sns_data_stream           *stream,
    sns_dae_set_static_config *config_req);
bool lsm6dso_dae_if_flush_samples(sns_sensor_instance *this);

