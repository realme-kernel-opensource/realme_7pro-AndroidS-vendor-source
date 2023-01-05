/** ======================================================================================
  @file lsm6dsm_fifo.c

  @brief lsm6dsm data acquisition HAL in FIFO mode

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
#define STM_LSM6DSM_REG_FIFO_CTRL1        (0x06)
#define STM_LSM6DSM_REG_FIFO_CTRL5        (0x0A)
#define STM_LSM6DSM_REG_FIFO_CTRL3        (0x08)
#define STM_LSM6DSM_REG_CTRL1_XL          (0x10)
#define STM_LSM6DSM_REG_WAKE_SRC          (0x1B)
#define STM_LSM6DSM_REG_OUT_X_L_XL        (0x28)
#define STM_LSM6DSM_REG_FIFO_STATUS1      (0x3A)
#define STM_LSM6DSM_REG_FIFO_STATUS2      (0x3B)
#define STM_LSM6DSM_REG_FIFO_STATUS3      (0x3C)
#define STM_LSM6DSM_REG_FIFO_DATA_OUT_L   (0x3E)
#define STM_LSM6DSM_REG_FUNC_SRC          (0x53)
#define STM_LSM6DSM_FIFO_DIFF1_MASK       (0xFF)
#define STM_LSM6DSM_FIFO_DIFF2_MASK       (0x07)
#define STM_LSM6DSM_FIFO_SAMPLE_SIZE      (6)

#define STM_LSM6DSM_ACC_TAG	              (0x02)
#define META_DATA_SIZE                     8 /* first 8 bytes are meta data */
/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/
static uint8_t lsm6dsm_fifo_wake_src = 0;
static uint8_t lsm6dsm_fifo_status[2] = {0};


/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dsm_fifo_get_data( sns_dd_handle_s*    dd_handle,
                       read_sensor_data    data_read_fptr,
                       notify_interrupt    notify_int_fptr,
                       int32_t             acc_delay,
                       int32_t*            delay_us,
                       bool*               call_again,
                       int32_t*            num_samples )
{
  sns_com_port_status_e status;
  uint8_t wake_src = 0;
  uint8_t func_src = 0;
  uint8_t fifo_mode[3] = {0};
  uint8_t fifo_status[4] = {0};

  /* Status registers for use in this function */
  sns_com_port_vector_s state_vectors[] =
  {
    { .reg_addr = STM_LSM6DSM_REG_WAKE_SRC,
      .buf_sz   = 1,
      .buf      = &wake_src },
    { .reg_addr = STM_LSM6DSM_REG_FIFO_STATUS1,
      .buf_sz   = 4,
      .buf      = fifo_status },
    { .reg_addr = STM_LSM6DSM_REG_FIFO_CTRL3,
      .buf_sz = 3,
      .buf = fifo_mode },
    { .reg_addr = STM_LSM6DSM_REG_FUNC_SRC,
      .buf_sz   = 1,
      .buf      = &func_src },
  };

  status = sns_com_port_read_reg_v( dd_handle, state_vectors, ARR_SIZE(state_vectors) );

  if( status == SNS_COM_PORT_STATUS_SUCCESS && wake_src != 0xff )
  {
    uint16_t count_h = fifo_status[1] & STM_LSM6DSM_FIFO_DIFF2_MASK;
    uint16_t count_l = fifo_status[0] & STM_LSM6DSM_FIFO_DIFF1_MASK;
    uint16_t num_bytes = (((count_h << 8) & 0xFF00) | count_l) * 2;

    // Check for empty/overflow
    if(fifo_status[1] & 0x10)
    { //Check empty
        num_bytes = 0;
    }
    if((!num_bytes) && (fifo_status[1] & 0x40))
    { //Check overflow
      num_bytes = 4096;
    }

    uint8_t bytes_to_discard = 0;

    bool gyro_data_in_buffer = ((fifo_mode[0] & 0x38) != 0);
    int32_t sample_size = STM_LSM6DSM_FIFO_SAMPLE_SIZE * (gyro_data_in_buffer ? 2 : 1);

    uint16_t pattern_pos = (((fifo_status[3] & 0x03) << 8) | (fifo_status[2] & 0xFF));
    pattern_pos  %= (sample_size /2);

    if(pattern_pos)
    {
      bytes_to_discard += sample_size - pattern_pos*2;
    }

    *num_samples = (num_bytes - bytes_to_discard) / sample_size;

    if( *num_samples > 0 )
    {
      sns_com_port_data_vector_s data_vectors[] =
      {
        { .reg_addr = STM_LSM6DSM_REG_FIFO_CTRL1,
          .buf_sz   = 3 },
        { .mem_addr = &fifo_status[1], .reg_addr = 0,
          .buf_sz   = 3 },
        { .reg_addr = STM_LSM6DSM_REG_CTRL1_XL,
          .buf_sz   = 2 },
#if 1
        { .reg_addr = STM_LSM6DSM_REG_FIFO_DATA_OUT_L,
          .buf_sz   = bytes_to_discard + (*num_samples * sample_size)},
#else     //Also support polling
        if(fifo_mode[2] != 0)
        {
        { .reg_addr = STM_LSM6DSM_REG_FIFO_DATA_OUT_L;
          .buf_sz   = num_bytes},
        }
        else
        {
          /* FIFO disabled, read from accel registers */
          .reg_addr = STM_LSM6DSM_REG_OUT_X_L_XL;
          .buf_sz = STM_LSM6DSM_FIFO_SAMPLE_SIZE;
          *num_samples = 1;
        }
#endif
      };
      *num_samples *= (gyro_data_in_buffer ? 2 : 1);
      if( *num_samples > 0 )
      {
        status = data_read_fptr(dd_handle, data_vectors, ARR_SIZE(data_vectors));
        *delay_us = 0;
        *call_again = false;
      }
    }

    {
      /* debug code; entire block can be removed */
      lsm6dsm_fifo_wake_src  = wake_src;
      lsm6dsm_fifo_status[0] = fifo_status[0];
      lsm6dsm_fifo_status[1] = fifo_status[1];
    }

    uint8_t regvals[8] = {0};
    wake_src &= ~0xD0;
    if( wake_src != 0 )
    {
      regvals[0] = wake_src;
    }
    func_src &= ~0xEF;
    if( func_src != 0 )
    {
      regvals[1] = func_src;
    }
    if(( func_src != 0 ) || ( wake_src != 0 ))
      notify_int_fptr( dd_handle, regvals );

  }
  return status;
}


/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dsm_fifo_parse_accel_data( sns_dd_handle_s*        dd_handle,
                               uint8_t const*          data_ptr,
                               uint16_t                data_bytes,
                               notify_accel_data       notify_data_fptr,
                               notify_accel_sample_cnt notify_cnt_fptr )
{
  sns_com_port_status_e rv = SNS_COM_PORT_STATUS_ERROR;
  int i;
  bool gyro_data_in_buffer = ((data_ptr[2] & 0x38) != 0);
  int32_t sample_size = STM_LSM6DSM_FIFO_SAMPLE_SIZE * (gyro_data_in_buffer ? 2 : 1);
  int32_t sample_count = (data_bytes - META_DATA_SIZE)/sample_size;
  int32_t loop_start = META_DATA_SIZE;

  uint16_t pattern_pos = (((data_ptr[5] & 0x03) << 8) | (data_ptr[4] & 0xFF));
  pattern_pos  %= (sample_size /2);

  if((pattern_pos) && (data_bytes >= (loop_start + sample_size)))
  {
    uint8_t bytes_to_discard = sample_size - pattern_pos*2;
    loop_start += bytes_to_discard;
  }
  
  /*
   Idea is to point loop_start variable to Accel data. 
   If we update this variable before the condition check, 
   then the check may fail if only one A+G sample is read (incase of gyro enabled, sample size would be 12 bytes)
   */
  if( data_bytes >= (loop_start + sample_size))
  {
    notify_cnt_fptr( dd_handle, sample_count );

    /*The total packets are META_DATA(8) + Gyro(6) +Accel(6)
      updating loop_start to point to accel data when gyro is enabled*/
    if(gyro_data_in_buffer)
      loop_start += STM_LSM6DSM_FIFO_SAMPLE_SIZE;

    for(i = loop_start; i < data_bytes; i += sample_size)
    {
      int16_t x,y,z;
      x = ((data_ptr[i+1] << 8) | data_ptr[i+0]);
      y = ((data_ptr[i+3] << 8) | data_ptr[i+2]);
      z = ((data_ptr[i+5] << 8) | data_ptr[i+4]);
      notify_data_fptr( dd_handle, x, y, z );
    }
    rv = SNS_COM_PORT_STATUS_SUCCESS;
  }
  return  rv;
}
/**
*****************************************************************************************
                            Global Function Pointer Table
*****************************************************************************************
*/

sns_dd_if_s lsm6dsm_fifo_hal_table =
  { .get_data = lsm6dsm_fifo_get_data,
    .parse_accel_data = lsm6dsm_fifo_parse_accel_data };

sns_dd_if_s lsm6dsm_fifo_hal_table2 =
  { .get_data = lsm6dsm_fifo_get_data,
    .parse_accel_data = lsm6dsm_fifo_parse_accel_data };

