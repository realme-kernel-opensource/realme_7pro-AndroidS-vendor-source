/** ======================================================================================
  @file lsm6dso_temperature.c

  @brief lsm6dso data acquisition HAL
  Copyright (c) 2018, STMicroelectronics.
  All rights reserved.

  Copyright (c) 2018 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

====================================================================================== **/

/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/
#include <stdlib.h>
#include "sns_dd_if.h"
#include "sns_macros.h"


/**
*****************************************************************************************
                               Constants/Macros
*****************************************************************************************
*/
#define STM_LSM6DSO_REG_STATUS            (0x1E)
#define STM_LSM6DSO_REG_OUT_TEMP_L        (0x20)

/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/

static sns_com_port_status_e
get_temperature_data( sns_dd_handle_s*    dd_handle,
                      read_sensor_data    data_read_fptr,
                      notify_interrupt    notify_int_fptr,
                      int32_t             acc_delay,
                      int32_t*            delay_us,
                      bool*               call_again,
                      int32_t*            num_samples )
{
  sns_com_port_status_e status;
  (void)notify_int_fptr;

  /* Registers to read into Data Acquisition buffer */
  const sns_com_port_data_vector_s data_vectors[] = 
  {
    { .reg_addr = STM_LSM6DSO_REG_STATUS,
      .buf_sz   = 1 },
    { .reg_addr = STM_LSM6DSO_REG_OUT_TEMP_L,
      .buf_sz   = 2 },
  };

  status = data_read_fptr( dd_handle, data_vectors, ARR_SIZE(data_vectors) );
  if( status == SNS_COM_PORT_STATUS_SUCCESS )
  {
    *num_samples = 1;
    *delay_us = 0;
    *call_again = false;
  }
  return status;
}

/**
*****************************************************************************************
                            Global Function Pointer Table
*****************************************************************************************
*/

sns_dd_if_s lsm6dso_temperature_hal_table =
  { .get_data         = get_temperature_data,
    .parse_accel_data = NULL };

sns_dd_if_s lsm6dso_temperature_hal_table2 =
  { .get_data         = get_temperature_data,
    .parse_accel_data = NULL };

