/**
 * @file sns_lsm6dso_hal.c
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
#include "sns_com_port_types.h"
#include "sns_diag_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_service_manager.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include "sns_types.h"
#include "sns_sensor_util.h"
#include "sns_lsm6dso_hal.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_sensor_instance.h"

#include "sns_printf.h"
#include "sns_async_com_port.pb.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_enter_i3c_mode(sns_sensor_instance *const instance,
                              lsm6dso_com_port_info *com_port,
                              sns_sync_com_port_service * scp_service)
{
#if LSM6DSO_USE_I3C
  sns_sync_com_port_handle    *i2c_port_handle = NULL;
  sns_com_port_config          i2c_com_config = com_port->com_config;
  sns_rc                       rv = SNS_RC_SUCCESS;
  uint32_t                     xfer_bytes;
  uint8_t                      buffer[6];

  if(com_port->com_config.bus_type != SNS_BUS_I3C_SDR &&
     com_port->com_config.bus_type != SNS_BUS_I3C_HDR_DDR )
  {
    return SNS_RC_SUCCESS;
  }

  if(NULL != instance)
  {
    DBG_INST_PRINTF_EX(LOW, instance, "enter i3c mode");
  }

  i2c_com_config.slave_control = com_port->i2c_address;
  rv = scp_service->api->sns_scp_register_com_port(&i2c_com_config, &i2c_port_handle);
  if( rv != SNS_RC_SUCCESS )
  {
    return SNS_RC_FAILED;
  }
  rv = scp_service->api->sns_scp_open(i2c_port_handle);
  if( rv != SNS_RC_SUCCESS )
  {
    return SNS_RC_FAILED;
  }

  /**-------------------Assign I3C dynamic address------------------------*/
  buffer[0] = (com_port->i3c_address & 0xFF)<<1;
  rv = scp_service->api->
    sns_scp_issue_ccc( i2c_port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETDASA,
                       buffer, 1, &xfer_bytes );
  scp_service->api->sns_scp_close(i2c_port_handle);
  scp_service->api->sns_scp_deregister_com_port(&i2c_port_handle);

  if(NULL != instance && rv == SNS_RC_SUCCESS)
  {
    DBG_INST_PRINTF_EX(HIGH, instance, "I3C addr: 0x%x",((uint32_t)buffer[0])>>1);
  }

  /**-------------Set max read size to the size of the FIFO------------------*/
  buffer[0] = (uint8_t)((STM_LSM6DSO_MAX_FIFO_SIZE >> 8) & 0xFF);
  buffer[1] = (uint8_t)(STM_LSM6DSO_MAX_FIFO_SIZE & 0xFF);
  buffer[2] = 4;
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETMRL,
                       buffer, 3, &xfer_bytes );
  if( rv != SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      SNS_INST_PRINTF(ERROR, instance, "Set max read length failed!");
    }
  }

  /**-------------------Enable/Disable IBI------------------------*/
  buffer[0] = 0x1;
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
        SNS_SYNC_COM_PORT_CCC_DISEC,
        buffer, 1, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(HIGH, instance, "IBI disabled");
    }
  } else {
    if(NULL != instance)
    {
      SNS_INST_PRINTF(ERROR, instance, "IBI disable FAILED!");
    }
  }
  

#if LSM6DSO_DEBUG_I3C
  /**-------------------Debug -- read all CCC info------------------------*/
  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMWL,
                       buffer, 2, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "max write length:0x%02x%02x", buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get max write length failed!");
    }
  }
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_SETMWL,
                       buffer, 2, &xfer_bytes );
  if( rv != SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Set max write length failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMRL,
                       buffer, 3, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "max read length:0x%02x%02x%02x",
                         buffer[0], buffer[1], buffer[2]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get max read length failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETPID,
                       buffer, 6, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(LOW, instance, "PID:0x%02x%02x:%02x%02x:%02x%02x",
                      buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get PID failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETBCR,
                       buffer, 1, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "bus charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get BCR failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETDCR,
                       buffer, 1, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "device charactaristics register:0x%x", buffer[0]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get DCR failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETSTATUS,
                       buffer, 2, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS ) {
    uint32_t status_reg =  buffer[0] | buffer[1] << 8;
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "status register:0x%x", status_reg);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get status failed!");
    }
  }

  sns_memset(buffer, 0, sizeof(buffer));
  rv = scp_service->api->
    sns_scp_issue_ccc( com_port->port_handle,
                       SNS_SYNC_COM_PORT_CCC_GETMXDS,
                       buffer, 2, &xfer_bytes );
  if( rv == SNS_RC_SUCCESS && xfer_bytes == 2) {
    if(NULL != instance)
    {
      DBG_INST_PRINTF_EX(LOW, instance, "MXDS :0x%02x%02x", buffer[0], buffer[1]);
    }
  } else {
    if(NULL != instance)
    {
      DBG_INST_PRINTF(ERROR, instance, "Get MXDS failed! rv:%d xfer_bytes:%d", rv, xfer_bytes);
    }
  }
#endif /* LSM6DSO_DEBUG_I3C */
  return rv;
#else
  UNUSED_VAR(instance);
  UNUSED_VAR(com_port);
  UNUSED_VAR(scp_service);

  return SNS_RC_FAILED;
#endif

}

/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_device_sw_reset(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  lsm6dso_instance_state *inst_state = (lsm6dso_instance_state*)instance->state->state;
  lsm6dso_com_port_info *com_port = &inst_state->com_port_info;
  uint8_t buffer = 0x01;
  int8_t num_attempts = 5;
  sns_rc rc = SNS_RC_FAILED;

  DBG_INST_PRINTF_EX(LOW, instance, "sw_rst");

  while(num_attempts-- > 0 && SNS_RC_SUCCESS != rc)
  {
    lsm6dso_enter_i3c_mode(instance, com_port, inst_state->scp_service);
    rc = lsm6dso_write_regs_scp(instance, STM_LSM6DSO_REG_CTRL3, 1, &buffer);
    if(SNS_RC_SUCCESS != rc)
    {
      DBG_INST_PRINTF(ERROR, instance, "sw_rst: failed; wait and try again");
      sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
    }
  }

  if(num_attempts <= 0)
  {
    DBG_INST_PRINTF(ERROR, instance, "sw_rst: failed all attempts");
  }

  num_attempts = 10;
  do
  {
    if(num_attempts-- <= 0)
    {
      DBG_INST_PRINTF(LOW, instance, "sw_rst: failed due to timeout- attempts:%d", 10-num_attempts);
      // Sensor HW has not recovered from SW reset.
      return SNS_RC_FAILED;
    }
    else
    {
      //0.1ms wait
      sns_busy_wait(sns_convert_ns_to_ticks(100*1000));
      lsm6dso_enter_i3c_mode(instance, com_port, inst_state->scp_service);
      lsm6dso_read_regs_scp(instance, STM_LSM6DSO_REG_CTRL3, 1, &buffer);
    }

  } while((buffer & 0x01));

  DBG_INST_PRINTF_EX(HIGH, instance, "sw_rst success! attempts %d", 10-num_attempts);
  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6dso_hal.h
 */
static sns_rc lsm6dso_device_set_default_state(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type sensor)
{
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  lsm6dso_instance_state *state =
    (lsm6dso_instance_state*)instance->state->state;
  bool force_int_w_i3c = 0;
  DBG_INST_PRINTF_EX(LOW, instance, "set_default_state: sensor=0x%x", sensor);
#if LSM6DSO_USE_I3C
  lsm6dso_com_port_info *com_port = &state->com_port_info;
  if( com_port->com_config.bus_type == SNS_BUS_I3C_SDR ||
      com_port->com_config.bus_type == SNS_BUS_I3C_HDR_DDR )
  {
    force_int_w_i3c = 1;
  }
#endif
  if(sensor == LSM6DSO_ACCEL)
  {
    // reset Accel state only
  }
  else if(sensor == LSM6DSO_GYRO)
  {
    // reset Gyro state only
  }
  else if(sensor == (LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_MOTION_DETECT | LSM6DSO_SENSOR_TEMP))
  {

     // Configure control register 3
     buffer[0] = 0x0
       | (0<<7)           // BOOT bit
       | (1<<6)           // BDU bit
       | (0<<5)           // H_LACTIVE bit
       | (0<<4)           // PP_OD bit
       | (0<<3)           // SIM bit
       | (1<<2)           // IF_INC
       | (0<<1)           // BLE
       | 0;               // SW_RESET

     rv = lsm6dso_read_modify_write(instance,
                            STM_LSM6DSO_REG_CTRL3,
                            &buffer[0],
                            1,
                            &xfer_bytes,
                            false,
                            0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     //workaround enable HPF for XL here
     //initialize with high threshold
     buffer[0] = 0x3F;
     rv = lsm6dso_read_modify_write(instance,
                         STM_LSM6DSO_REG_WAKE_THS,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0x3F);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     buffer[0] = 0
       | (0<<7)            // HPLPF2_XL_BW2
       | (1<<6)            // HPLPF2_XL_BW1
       | (1<<5)            // HPLPF2_XL_BW0:1-HP Filter=ODR/10
       | (0<<4)            // HP_REF_MODE
       | (1<<3)            // XL_FASTSETTL_MODE
       | (0<<2)            // FDS
       | 0;                // LOW_PASS_ON_6D:
     rv = lsm6dso_read_modify_write(instance,
                         STM_LSM6DSO_REG_CTRL8_XL,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     //Enable interrupt for MD
     rv = lsm6dso_set_interrupts(instance);
     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     buffer[0] = 0
       | (0<<7)            // DEN_X
       | (0<<6)            // DEN_Y
       | (0<<5)            // DEN_Z
       | (0<<4)            // DEN_XL_G
       | (0<<3)            // DEN_XL_GEN
       | (0<<2)            // DEN_LH
       | (0<<1)            // I3C_disable
       | force_int_w_i3c;  // Force enable INT when I3C is active

     rv = lsm6dso_read_modify_write(instance,
                         STM_LSM6DSO_REG_CTRL9_XL,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     lsm6dso_set_accel_config(instance,
                            LSM6DSO_ACCEL_ODR_OFF,
                            state->accel_info.sstvt,
                            state->accel_info.range,
                            LSM6DSO_ACCEL_BW50);

     lsm6dso_set_gyro_config(instance,
                             LSM6DSO_GYRO_ODR_OFF,
                             state->gyro_info.sstvt,
                             state->gyro_info.range);
  }

  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_reset_device(
    sns_sensor_instance *const instance,
    lsm6dso_sensor_type sensor)
{
  sns_rc rv = SNS_RC_SUCCESS;

  DBG_INST_PRINTF_EX(HIGH, instance, "reset_device");
  /** HW reset only when both Accel and Gyro are requested for
   *  reset. */
  if( sensor == (LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_MOTION_DETECT | LSM6DSO_SENSOR_TEMP))
  {
     rv = lsm6dso_device_sw_reset(instance, sensor);
  }
  else
  {
    lsm6dso_instance_state *inst_state =
      (lsm6dso_instance_state*)instance->state->state;
    lsm6dso_com_port_info *com_port = &inst_state->com_port_info;
    lsm6dso_enter_i3c_mode(instance, com_port, inst_state->scp_service);
  }

  if(rv == SNS_RC_SUCCESS)
  {
    rv = lsm6dso_device_set_default_state(instance, sensor);
  }

  if(rv != SNS_RC_SUCCESS)
  {
    DBG_INST_PRINTF(ERROR, instance, "reset_device failed!");
  }
  return rv;
}

sns_rc lsm6dso_recover_device(sns_sensor_instance *const this)
{
  lsm6dso_instance_state *state = (lsm6dso_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  struct group_read {
    uint32_t first_reg;
    uint8_t  num_regs;
  } groups[] = { /* must fit within state->reg_status[] */
    { STM_LSM6DSO_REG_FIFO_CTRL1, 4 },
    { STM_LSM6DSO_REG_INT1_CTRL, 12 },
    { STM_LSM6DSO_REG_TAP_CFG0, 10  }
  };

  //Save Context
  {
    uint8_t *dest = state->reg_status;

    for(uint32_t i=0; i<ARR_SIZE(groups); i++)
    {
      lsm6dso_read_regs_scp(this, groups[i].first_reg, groups[i].num_regs, dest);
#if 1 //LSM6DSO_DUMP_REG
      for(uint32_t j=0; j<groups[i].num_regs; j++)
      {
        DBG_INST_PRINTF(LOW, this, "dump: 0x%02x=0x%02x",
                        groups[i].first_reg+j, dest[j]);
      }
#endif
      dest += groups[i].num_regs;
    }
    DBG_INST_PRINTF(MED, this, "Context saved");
  }

  //Gyro set power down mode
  lsm6dso_set_gyro_config(this,
                          LSM6DSO_GYRO_ODR_OFF,
                          state->gyro_info.sstvt,
                          state->gyro_info.range);

  //Set Accel in High Performance mode
  lsm6dso_set_acc_lpmode(this, false);

  // Reset Sensor
  rv = lsm6dso_reset_device(this,
      LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_MOTION_DETECT | LSM6DSO_SENSOR_TEMP);

  //Recover ESP and XSESNOR after device reset
  lsm6dso_recover_esp(this);

  //Power up Accel if needed. It was powered down during reset_device
  lsm6dso_set_accel_config(this,
      state->accel_info.desired_odr,
      state->accel_info.sstvt,
      state->accel_info.range,
      state->accel_info.bw);

  //Power up gyro if needed
  lsm6dso_set_gyro_config(this,
      state->gyro_info.desired_odr,
      state->gyro_info.sstvt,
      state->gyro_info.range);

  //Restore context
  {
    uint8_t *src = state->reg_status;

    for(uint32_t i=0; i<ARR_SIZE(groups); i++)
    {
      lsm6dso_write_regs_scp(this, groups[i].first_reg, groups[i].num_regs, src);
      src += groups[i].num_regs;
    }
    DBG_INST_PRINTF(MED, this, "Context restored");
  }

  //Resrt flags
  state->ascp_req_count = 0;
  state->fifo_info.reconfig_req = true;

  if(state->fifo_info.reconfig_req)
    lsm6dso_reconfig_fifo(this, false);

  state->fifo_info.last_timestamp = sns_get_system_time();

  return rv;
}

/**
 * see sns_lsm6dso_hal.h
 */
sns_rc lsm6dso_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = lsm6dso_com_read_wrapper(scp_service,
                                port_handle,
                                STM_LSM6DSO_REG_WHO_AM_I,
                                buffer,
                                1,
                                &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}


