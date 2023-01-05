/**
 * @file sns_lsm6dso_fsm_island.c
 *
 * Common implementation for LSM6DSO fsm Sensors.
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
#include "sns_lsm6dso_fsm.h"
#include "sns_types.h"

#if LSM6DSO_FSM_ENABLED
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

sns_rc lsm6dso_fsm_set_enable(sns_sensor_instance *instance, uint16_t sensor,
                                        xsensor_int int_line, bool enable)
{
  int32_t i;
  sns_rc rc = SNS_RC_SUCCESS;
  uint16_t enable_mask = 0;
  uint8_t enable_mask_lsb = 0, enable_mask_msb = 0;
  uint8_t int_addr = LSM6DSO_REG_FSM_INT1_A;
  uint32_t xfer_bytes;
  uint8_t rw_buffer_lsb = 0, rw_buffer_msb = 0;
  uint8_t rw_buffer = 0;
  uint8_t rw_addr = 0;

  if(FSM_SENSOR_CNT == 0)
    return SNS_RC_INVALID_VALUE;

  for (i = 0; i < FSM_SENSOR_CNT; i++)
    if (lsm6dso_fsm_sensor_list[i].id == sensor)
      break;

  if (i == FSM_SENSOR_CNT)
    return SNS_RC_INVALID_VALUE;

  //Routing of embedded function on INT1/INT2
  rw_addr = (int_line) ? STM_LSM6DSO_REG_MD2_CFG : STM_LSM6DSO_REG_MD1_CFG;
  rw_buffer = 0x02;
  rc = lsm6dso_read_modify_write(instance,
           rw_addr,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           0x02);

  /* Enable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,true);
  if (rc != SNS_RC_SUCCESS)
    return rc;

  enable_mask = (1 << i);

  enable_mask_lsb = (uint8_t)(enable_mask & 0xFF);
  enable_mask_msb = (uint8_t)((uint8_t)(enable_mask >> 8) & 0xFF);
  DBG_INST_PRINTF(HIGH, instance,  "enable_mask_lsb=%d, enable_mask_msb = %d ",enable_mask_lsb, enable_mask_msb);

  /* enable fsm interrupt INT1 or INT2*/
  if(int_line == XSENSOR_INT_2)
    int_addr = LSM6DSO_REG_FSM_INT2_A;

  if(enable) {
    rw_buffer_lsb = enable_mask_lsb;
    rw_buffer_msb = enable_mask_msb;
  }

  uint8_t write_buf[4];

  lsm6dso_read_regs_scp(instance, int_addr, 2, &write_buf[0]);
  lsm6dso_read_regs_scp(instance, LSM6DSO_REG_FSM_ENABLE_A_ADDR, 2, &write_buf[2]);

  write_buf[0] = (write_buf[0] & ~enable_mask_lsb) |  (rw_buffer_lsb & enable_mask_lsb);
  write_buf[1] = (write_buf[1] & ~enable_mask_msb) |  (rw_buffer_msb & enable_mask_msb);
  write_buf[2] = (write_buf[2] & ~enable_mask_lsb) |  (rw_buffer_lsb & enable_mask_lsb);
  write_buf[3] = (write_buf[3] & ~enable_mask_msb) |  (rw_buffer_msb & enable_mask_msb);

  rc = lsm6dso_com_write_wrapper(instance,
                        int_addr,
                        &write_buf[0],
                        2,
                        &xfer_bytes,
                        false);

  rc |= lsm6dso_com_write_wrapper(instance,
                      LSM6DSO_REG_FSM_ENABLE_A_ADDR,
                      &write_buf[2],
                      2,
                      &xfer_bytes,
                      false);

  /* Disable embedded access*/
  rc |= lsm6dso_emb_cfg_access(instance,false);

  return rc;
}

#if !LSM6DSO_MLC_ENABLED
static void lsm6dso_load_fsm(sns_sensor_instance *instance)
{
  int reg_counter;
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  for(reg_counter=0; reg_counter<lsm6dso_xsensor_data.len; )
  {
    rw_buffer = lsm6dso_xsensor_data.data[reg_counter+1];
#if LSM6DSO_DUMP_REG
    DBG_INST_PRINTF_EX(HIGH, instance, "FSM reg:%0x value:%0x",
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
#endif
void lsm6dso_init_fsm_instance(sns_sensor_instance *instance)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  /* Enable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,true);

  rw_buffer = LSM6DSO_FSM_INIT_MASK;
  rc = lsm6dso_read_modify_write(instance,
           LSM6DSO_REG_FSM_INIT_B,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           LSM6DSO_FSM_INIT_MASK);

  /* enable latched interrupts */
  rw_buffer = 0x80;
  rc = lsm6dso_read_modify_write(instance,
           STM_LSM6DSO_PAGE_RW_ADDR,
           &rw_buffer,
           1,
           &xfer_bytes,
           false,
           STM_LSM6DSO_REG_EMB_FUNC_LIR_MASK);

  /* Disable embedded access*/
  rc = lsm6dso_emb_cfg_access(instance,false);

#if !LSM6DSO_MLC_ENABLED
  lsm6dso_load_fsm(instance);
#endif

  return;
}

void lsm6dso_fsm_get_interrupt_status(sns_sensor_instance *instance, uint8_t const *wake_src, uint8_t const *emb_src)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint16_t fsm_status = 0;
  sns_memset(&state->xgroup_info.fsm_status, 0, sizeof(state->xgroup_info.fsm_status));

  if(wake_src && (*wake_src != LSM6DSO_REG_FSM_STATUS_MAINPAGE) && lsm6dso_dae_if_available(instance))
    return;

  if(lsm6dso_dae_if_available(instance)) {
    state->xgroup_info.fsm_status[0] = emb_src[0];
    state->xgroup_info.fsm_status[1] = emb_src[1];
    state->xgroup_info.fsm_outs[0] = emb_src[2];
    state->xgroup_info.fsm_outs[1] = emb_src[3];
    state->xgroup_info.fsm_outs[2] = emb_src[4];
    state->xgroup_info.fsm_lc[0] = emb_src[5];
    state->xgroup_info.fsm_lc[1] = emb_src[6];
  }
  else {
    //read fsm_status main page, fsm_outs and fsm_lc - non-DAE mode
    lsm6dso_read_regs_scp(instance, LSM6DSO_REG_FSM_STATUS_MAINPAGE, 2, state->xgroup_info.fsm_status);

#if (LSM6DSO_FSM_LC_ENABLED || LSM6DSO_FSM_OUTS_ENABLED)
    /* Enable embedded access*/
    lsm6dso_emb_cfg_access(instance,true);

    lsm6dso_read_regs_scp(instance, LSM6DSO_REG_FSM_OUTS1, 8, state->xgroup_info.fsm_outs);
    lsm6dso_read_regs_scp(instance, LSM6DSO_REG_FSM_LC, 2, state->xgroup_info.fsm_lc);

    /* Disable embedded access*/
    lsm6dso_emb_cfg_access(instance,false);

#endif
  }
  fsm_status = (state->xgroup_info.fsm_status[0] | (state->xgroup_info.fsm_status[1] << 8));

  DBG_INST_PRINTF_EX(HIGH, instance, "FSM HW INT:sensors_e=0x%x, status = %d ", state->xgroup_info.enabled_sensors, (uint16_t)state->xgroup_info.fsm_status[0]);
  DBG_INST_PRINTF_EX(HIGH, instance, "FSM_OUTS = %0x %0x %0x FSM_LC = %0x", state->xgroup_info.fsm_outs[0], state->xgroup_info.fsm_outs[1], state->xgroup_info.fsm_outs[2], state->xgroup_info.fsm_lc[0]|(state->xgroup_info.fsm_lc[1]<<8));

  if(fsm_status & 1)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 1 detected");
  if(fsm_status & 2)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 2 detected");
  if(fsm_status & 4)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 3 detected");
  if(fsm_status & 8)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 4 detected");
  if(fsm_status & 16)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 5 detected");
  if(fsm_status & 32)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 6 detected");
  if(fsm_status & 64)
    DBG_INST_PRINTF_EX(HIGH, instance,  "FSM 7 detected");

}

bool lsm6dso_check_fsm_sensor_interrupt(sns_sensor_instance *const instance, uint16_t sensor)
{
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  uint16_t fsm_status = 0;
  fsm_status = (state->xgroup_info.fsm_status[0] | (state->xgroup_info.fsm_status[1] << 8));

  int16_t i;
  for (i = 0; i < FSM_SENSOR_CNT; i++)
    if (lsm6dso_fsm_sensor_list[i].id == sensor)
      break;

  if (i == FSM_SENSOR_CNT)
  {
    DBG_INST_PRINTF_EX(HIGH, instance, "Not an FSM sensor = %0x %d", sensor,FSM_SENSOR_CNT);
    return 0;
  }
  uint16_t status_mask = (1 << i);

  DBG_INST_PRINTF_EX(HIGH, instance,  "Status Mask: %d, FSM_STATUS: %d", status_mask, fsm_status);
  return (status_mask & fsm_status);
}

#endif

