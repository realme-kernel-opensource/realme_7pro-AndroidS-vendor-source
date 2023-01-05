/** ======================================================================================
  @file lsm6dsm.c

  @brief lsm6dsm data acquisition HAL

  Copyright (c) 2018 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

====================================================================================== **/

/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/
#include <stdbool.h>
#include "sns_dd_if.h"
#include "sns_macros.h"


/**
*****************************************************************************************
                               Constants/Macros
*****************************************************************************************
*/
#define STM_LSM6DSM_REG_CTRL1_XL          (0x10)
#define STM_LSM6DSM_REG_WAKE_SRC          (0x1B)
#define STM_LSM6DSM_REG_OUT_X_L_G         (0x22)

/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/
static uint8_t lsm6dsm_wake_src = 0;

/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dsm_get_data( sns_dd_handle_s*    dd_handle,
                  read_sensor_data    data_read_fptr,
                  notify_interrupt    notify_int_fptr,
                  int32_t             acc_delay,
                  int32_t*            delay_us,
                  bool*               call_again,
                  int32_t*            num_samples )
{
  sns_com_port_status_e status;
  uint8_t wake_src = 0;

  /* Status registers for use in this function */
  sns_com_port_vector_s state_vectors[] =
  {
    { .reg_addr = STM_LSM6DSM_REG_WAKE_SRC,
      .buf_sz   = 1,
      .buf      = &wake_src },
  };

  /* Read the status registers */
  status = sns_com_port_read_reg_v( dd_handle, state_vectors, ARR_SIZE(state_vectors) );
      
  if( status == SNS_COM_PORT_STATUS_SUCCESS && wake_src != 0xff )
  {
    sns_com_port_data_vector_s data_vectors[] = 
    {
      { .reg_addr = STM_LSM6DSM_REG_CTRL1_XL,
        .buf_sz   = 2 },
      { .reg_addr = STM_LSM6DSM_REG_OUT_X_L_G,
        .buf_sz   = 12 },
    };
    *num_samples = 1;
    status = data_read_fptr( dd_handle, data_vectors, ARR_SIZE(data_vectors) );
    *delay_us = 0;
    *call_again = false;

    wake_src &= ~0x7;
    if( wake_src != 0 )
    {
      uint8_t regvals[8] = {0};
      regvals[0] = wake_src;
      notify_int_fptr(dd_handle, regvals);
    }

    {
      /* debug code; entire block can be removed */
      lsm6dsm_wake_src  = wake_src;
    }
  }
  return status;
}


/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dsm_parse_accel_data( sns_dd_handle_s*        dd_handle,
                          uint8_t const*          data_ptr,
                          uint16_t                data_bytes,
                          notify_accel_data       notify_data_fptr,
                          notify_accel_sample_cnt notify_cnt_fptr )
{
  sns_com_port_status_e rv = SNS_COM_PORT_STATUS_ERROR;

  if( data_bytes >= 14 && (data_ptr[0] & 0xF0) != 0 )
  {
    int i = 8; /* first 2 bytes are REG_CTRL1_XL and REG_CTRL2_G, next 6 are Gyro data */
    int16_t x = ((data_ptr[i+1] << 8) | data_ptr[i+0]);
    int16_t y = ((data_ptr[i+3] << 8) | data_ptr[i+2]);
    int16_t z = ((data_ptr[i+5] << 8) | data_ptr[i+4]);

    notify_cnt_fptr( dd_handle, 1 );
    notify_data_fptr( dd_handle, x, y, z );
    rv = SNS_COM_PORT_STATUS_SUCCESS;
  }
  return  rv;
}

/**
*****************************************************************************************
                            Global Function Pointer Table
*****************************************************************************************
*/

sns_dd_if_s lsm6dsm_hal_table =
  { .get_data = lsm6dsm_get_data,
    .parse_accel_data = lsm6dsm_parse_accel_data };

