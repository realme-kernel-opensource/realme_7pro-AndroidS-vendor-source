#pragma once
/**
 * @file sns_lsm6dso_s4s.h
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
#include "sns_timer.pb.h"
#include "sns_sensor_instance.h"

#define LSM6DSO_S4S_TPH_L         0X04
#define LSM6DSO_S4S_TPH_H         0X05
#define LSM6DSO_S4S_RR            0X06
#define LSM6DSO_S4S_ST            0x60
#define LSM6DSO_S4S_DT            0x61
typedef enum
{
  LSM6DSO_S4S_OFF,          // tph=0 & odr=0. sensor is in power down mode.
  LSM6DSO_S4S_NOT_SYNCED,   // after set odr/tph/rr, before receive 1st ST-DT
  LSM6DSO_S4S_SYNCING,      // sensor received 1st ST-DT
  LSM6DSO_S4S_1ST_SYNCED,   // sensor received 2nd ST-DT
  LSM6DSO_S4S_2ND_SYNCED,   // sensor received updated tph, waiting 200ms before St/dt
  LSM6DSO_S4S_SYNCED,       // sensor received 3 or more ST-DT
  LSM6DSO_S4S_CONFIG_CHANGE, // temporary state while changing configuration - sent st/aborT
  LSM6DSO_S4S_NON_S4S       // sensor goes to nonS4S streaming. Not used for LSM6DSO
} lsm6dso_s4s_sync_state_t;

typedef struct
{
  uint8_t S4sTphL;
  uint8_t S4sTphH;
  uint8_t S4sRr;
  uint8_t S4sSt;
  uint8_t S4sDt;
} lsm6dso_s4s_ctrl_regs;
typedef struct lsm6dso_s4s_config
{
  uint16_t t_ph; /**< Number of samples in one synchronization period */
  uint8_t  rr; /**< 0-3, indicating the LSB resolution of the DT message.
                * rr=0 --> 2^-11 of the ideal sync interval.
                * rr=1 --> 2^-12 of the ideal sync interval.
                * rr=2 --> 2^-13 of the ideal sync interval.
                * rr=3 --> 2^-14 of the ideal sync interval.
                **/
  uint32_t ideal_sync_interval; /**< Number of ticks in one sync interval. */
  bool     fifo_int_en; /**< false: disable FIFO interrupt, or set watermark to maximum
                          true: enable FIFO interrupt as normal -- only used for debugging */
} lsm6dso_s4s_config;

typedef struct lsm6dso_s4s_info
{
  uint32_t ticks_per_rr;
  sns_time host_sync_ts;
  lsm6dso_s4s_sync_state_t sync_state;
  lsm6dso_s4s_ctrl_regs ctrl_regs;
  sns_time st_ts;
  uint16_t sync_time; //interms of number of samples
  uint16_t poll_time; //interms of number of samples
  uint16_t odr;
  bool is_sync_updated;
  bool is_new_tph;
  uint16_t sync_poll_count;
  sns_time last_effect_sync_ts;
  lsm6dso_s4s_config current_conf;
} lsm6dso_s4s_info;

sns_rc lsm6dso_start_s4s_sync(sns_sensor_instance *const this, uint16_t odr, sns_time host_sync_ts);
sns_rc lsm6dso_update_s4s_sync(sns_sensor_instance *const this, uint16_t odr);
sns_rc lsm6dso_get_s4s_sample_interval(sns_sensor_instance *const this,
    uint16_t total_samples,
    sns_time* sample_interval,
    sns_time* first_ts,
    sns_time* end_ts);

sns_rc lsm6dso_handle_polling_timer_event(sns_sensor_instance *const this,
    sns_time timestamp,
    sns_timer_sensor_event* latest_timer_event);
