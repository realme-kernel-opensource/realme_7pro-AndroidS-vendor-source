/**
 * @file sns_lsm6dso_mlc_island.c
 *
 * Common implementation for LSM6DSO mlc Sensors.
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
#include <math.h>
#include <string.h>

#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_mlc.h"
#include "sns_types.h"

#if LSM6DSO_MLC_ENABLED
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_printf.h"
#include "pb_encode.h"

#include "sns_diag.pb.h"
#include "sns_registry.pb.h"
#include "sns_std.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_lsm6dso_xsensor_config.h"

extern sns_rc lsm6dso_sensor_notify_event(sns_sensor *const this);
sns_rc lsm6dso_mlc_set_enable(sns_sensor_instance *instance, uint16_t sensor,
                                        xsensor_int int_line,
                                        bool enable)
{
  int8_t i;
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t enable_mask;
  uint32_t xfer_bytes;
  uint8_t rw_buffer = 0;
  uint8_t int_addr = LSM6DSO_REG_MLC_INT1;

  if(MLC_SENSOR_CNT == 0)
    return SNS_RC_INVALID_VALUE;

  for (i = 0; i < MLC_SENSOR_CNT; i++)
    if (lsm6dso_mlc_sensor_list[i].id == sensor)
      break;

  if (i == MLC_SENSOR_CNT)
    return SNS_RC_INVALID_VALUE;

  if(int_line == XSENSOR_INT_2)
    int_addr = LSM6DSO_REG_MLC_INT2;

  /* Enable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,true);
  if (rc != SNS_RC_SUCCESS)
    return rc;

    /* enable mlc interrupt */
  enable_mask = (1 << i);

  rw_buffer = enable_mask;
  if(!enable)
    rw_buffer = 0;
  DBG_INST_PRINTF_EX(HIGH, instance, "MLC Enable_mask=%d",enable_mask);
  rc = lsm6dso_read_modify_write(instance,
           int_addr,
           &enable_mask,
           1,
           &xfer_bytes,
           false,
           enable_mask);
  /* enable latched interrupts */
  rw_buffer = 0x80;
  rc = lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_PAGE_RW_ADDR,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_EMB_FUNC_LIR_MASK);
  if(!enable)
    rw_buffer = 0;
  else
    rw_buffer = LSM6DSO_REG_MLC_EN_MASK;

  rc = lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_EMB_FUNC_EN_B_ADDR,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           LSM6DSO_REG_MLC_EN_MASK);
  /* Disable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,false);

	return rc;
}

static void lsm6dso_load_mlc(sns_sensor_instance *instance)
{
  int reg_counter;
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  for(reg_counter = 0; reg_counter < lsm6dso_xsensor_data.len; )
  {
    rw_buffer = lsm6dso_xsensor_data.data[reg_counter+1];
#if LSM6DSO_DUMP_REG
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC reg:%0x value:%0x",
      lsm6dso_xsensor_data.data[reg_counter],lsm6dso_xsensor_data.data[reg_counter+1]);
#endif
    rc = lsm6dso_com_write_wrapper(instance,
                          lsm6dso_xsensor_data.data[reg_counter],
                          &rw_buffer,
                          1,
                          &xfer_bytes,
                          false);
    reg_counter+=2;
  }
}

void lsm6dso_init_mlc_instance(sns_sensor_instance *instance)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  rw_buffer = 0x02;
  rc = lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_REG_MD1_CFG,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x02);

  lsm6dso_load_mlc(instance);

  /* Disable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,false);
  return;
}

void lsm6dso_mlc_deinit(sns_sensor_instance *instance)
{
  UNUSED_VAR(instance);
}

bool lsm6dso_mlc_get_interrupt_status(sns_sensor_instance *instance, uint8_t const *wake_src, uint8_t const *emb_src)
{
  uint8_t i;
  uint8_t status_mask;
  uint8_t mlc_status = 0;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  state->xgroup_info.mlc_status = 0;
  sns_memset(&state->xgroup_info.mlc_src, 0, sizeof(state->xgroup_info.mlc_src));

  sns_rc rc = SNS_RC_SUCCESS;

  if((MLC_SENSOR_CNT == 0) || ((wake_src) && (*wake_src != STM_LSM6DSO_REG_MLC_STATUS_MAINPAGE) && (lsm6dso_dae_if_available(instance))))
  {
    DBG_INST_PRINTF_EX(HIGH, instance, "Not an MLC interrupt");
    return false;
  }

  if(lsm6dso_dae_if_available(instance)) {
    state->xgroup_info.mlc_status = emb_src[0];
    if(state->xgroup_info.mlc_status) {
      state->xgroup_info.mlc_src[0] = emb_src[1];
      state->xgroup_info.mlc_src[1] = emb_src[2];
      state->xgroup_info.mlc_src[2] = emb_src[3];
      state->xgroup_info.mlc_src[3] = emb_src[4];
      state->xgroup_info.mlc_src[4] = emb_src[5];
      state->xgroup_info.mlc_src[5] = emb_src[6];
    }
  }
  else { //Non DAE case
    rc = lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_MLC_STATUS_MAINPAGE, 1, &state->xgroup_info.mlc_status);
    if(state->xgroup_info.mlc_status)
    {
      rc = lsm6dso_emb_cfg_access(instance,true);
      for (i = 0; i < MLC_SENSOR_CNT; i++)
      {
        status_mask = (1<<i);
        if((state->xgroup_info.mlc_status) & (status_mask))
        rc = lsm6dso_read_regs_scp(instance, (STM_LSM6DSO_REG_MLC_SRC+i), 1, &state->xgroup_info.mlc_src[i]);
        rc = lsm6dso_read_regs_scp(instance, (STM_LSM6DSO_REG_MLC_STATUS_EMB), 1, &mlc_status);
        SNS_INST_PRINTF(HIGH, instance,  "mlc_src[%d] = %0x,mlc_status_emb = %0x",i, state->xgroup_info.mlc_src[i],mlc_status);
      }
      rc = lsm6dso_emb_cfg_access(instance,false);
    }
  }

  if(state->xgroup_info.mlc_status & 1)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 1 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[0]);
  if(state->xgroup_info.mlc_status & 2)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 2 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[1]);
  if(state->xgroup_info.mlc_status & 4)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 3 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[2]);
  if(state->xgroup_info.mlc_status & 8)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 4 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[3]);
  if(state->xgroup_info.mlc_status & 16)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 5 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[4]);
  if(state->xgroup_info.mlc_status & 32)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 6 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[5]);
  if(state->xgroup_info.mlc_status & 64)
    DBG_INST_PRINTF_EX(HIGH, instance, "MLC 7 interrupt, mlc_src = %0x",state->xgroup_info.mlc_src[6]);

  return true;
}

bool lsm6dso_mlc_check_sensor_interrupt(sns_sensor_instance *const instance, uint16_t sensor)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;

  int16_t i;
  for (i = 0; i < MLC_SENSOR_CNT; i++)
    if (lsm6dso_mlc_sensor_list[i].id == sensor)
      break;

  if (i == MLC_SENSOR_CNT)
  {
    DBG_INST_PRINTF_EX(HIGH, instance, "Not a MLC sensor = %0x %d", sensor,MLC_SENSOR_CNT);
    return 0;
  }
  uint16_t status_mask = (1 << i);

  DBG_INST_PRINTF_EX(HIGH, instance,  "Sensor_idx: %d, mlc_status: %d", i, state->xgroup_info.mlc_status);
  return (status_mask & state->xgroup_info.mlc_status);
}

#endif

