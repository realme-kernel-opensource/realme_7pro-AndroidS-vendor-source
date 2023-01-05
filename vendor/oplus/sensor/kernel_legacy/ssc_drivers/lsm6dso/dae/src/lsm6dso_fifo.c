/** ======================================================================================
  @file lsm6dso_fifo.c

  @brief lsm6dso data acquisition HAL in FIFO mode

  Copyright (c) 2018-2020, STMicroelectronics.
  All rights reserved.
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
#include "sns_time.h"

/**
*****************************************************************************************
                               Constants/Macros
*****************************************************************************************
*/
#define STM_LSM6DSO_REG_FIFO_CTRL1          (0x07)
#define STM_LSM6DSO_REG_CNT_BDR1            (0x0B)
#define STM_LSM6DSO_REG_CTRL1_XL            (0x10)
#define STM_LSM6DSO_REG_WAKE_SRC            (0x1B)
#define STM_LSM6DSO_REG_TAP_SRC             (0x1C)
#define STM_LSM6DSO_REG_FIFO_STATUS1        (0x3A)
#define STM_LSM6DSO_REG_FIFO_STATUS2        (0x3B)
#define STM_LSM6DSO_REG_FIFO_OUT_TAG        (0x78)

#define STM_LSM6DSO_FIFO_DIFF1_MASK         (0xFF)
#define STM_LSM6DSO_FIFO_DIFF2_MASK         (0x03)
#define STM_LSM6DSO_FIFO_OVERFLOW_MASK      (0x40)
#define STM_LSM6DSO_FIFO_SAMPLE_SIZE	    (1 + 6)
#define STM_LSM6DSO_SINGLE_TAP_MASK         (0x20)
#define STM_LSM6DSO_DOUBLE_TAP_MASK         (0x10)

#define STM_LSM6DSO_FIFO_MAX_SIZE           (3072)
#define STM_FIFO_OVERFLOW_SAMPLE_CNT        STM_LSM6DSO_FIFO_MAX_SIZE/STM_LSM6DSO_FIFO_SAMPLE_SIZE
#define STM_LSM6DSO_ACC_TAG			         (0x02)
#define STM_LSM6DSO_REG_MLC_STATUS_MAINPAGE (0x38)
#define STM_LSM6DSO_REG_MLC_STATUS_EMB      (0x15)
#define STM_LSM6DSO_REG_MLC_SRC             (0x70)
#define STM_LSM6DSO_REG_FSM_STATUS_MAINPAGE (0x36)
#define STM_LSM6DSO_REG_FSM_LONG_COUNTER_L  (0x48)
#define STM_LSM6DSO_REG_FSM_OUTS_1          (0x4C)
#define STM_LSM6DSO_REG_FUNC_CFG            (0x01)
#define LSM6DSO_FSM_ENABLED                 0
#define LSM6DSO_MLC_ENABLED                 0
#define LSM6DSO_FSM_OUTS_ENABLED            0
#define LSM6DSO_FSM_LC_ENABLED              0
#define LSM6DSO_ESP_DOUBLE_TAP              0
#define LSM6DSO_EX_TAP_TUNING_ENABLED       0
#define LSM6DSO_TS_IN_META_DATA             0
#if LSM6DSO_TS_IN_META_DATA
#define META_DATA_SIZE                      5
#else
#define META_DATA_SIZE                      4
#endif

/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/

/* ------------------------------------------------------------------------------------ */
static void lsm6dso_check_other_interrupts(
  sns_dd_handle_s*  dd_handle,
  uint8_t           fsm_reg_vals[8],  /* init to zeroes by caller */
  uint8_t           mlc_reg_vals[8],
  uint8_t           tap_reg_vals[2])  /* init to zeroes by caller */
{
#if LSM6DSO_FSM_ENABLED || LSM6DSO_MLC_ENABLED
  uint8_t fsm_mlc_status[3] = {0};
  uint8_t fsm_outs[3]       = {0};
  uint8_t fsm_lc[2]         = {0};
  uint8_t mlc_src[8]        = {0};

  sns_com_port_vector_s state_vectors[] =
  {
    { .reg_addr = STM_LSM6DSO_REG_FSM_STATUS_MAINPAGE,
      .buf_sz   = 3,
      .buf      = fsm_mlc_status  },
  };
  sns_com_port_read_reg_v( dd_handle, state_vectors, ARR_SIZE(state_vectors) );

#if (LSM6DSO_FSM_OUTS_ENABLED || LSM6DSO_MLC_ENABLED)
  if( (fsm_mlc_status[0] != 0) || (fsm_mlc_status[1] != 0) || (fsm_mlc_status[2] != 0))
  {
    /* Enable embedded src */
    uint8_t emb_mask = 0x80;
    sns_com_port_vector_s vectors_emb_w_1[] =
    {
      { .reg_addr = STM_LSM6DSO_REG_FUNC_CFG,
        .buf_sz   = 1,
        .buf      = &emb_mask },
    };
    sns_com_port_write_reg_v( dd_handle, vectors_emb_w_1, ARR_SIZE(vectors_emb_w_1) );

    if( (fsm_mlc_status[0] != 0) || (fsm_mlc_status[1] != 0) )
    {
      /* Read fsm_lc from embedded space */
      sns_com_port_vector_s state_vectors_fsm_mlc[] =
      {
#if LSM6DSO_FSM_LC_ENABLED
        { .reg_addr = STM_LSM6DSO_REG_FSM_LONG_COUNTER_L,
          .buf_sz   = 2,
          .buf      = fsm_lc },
#endif
#if LSM6DSO_FSM_OUTS_ENABLED
        { .reg_addr = STM_LSM6DSO_REG_FSM_OUTS_1,
          .buf_sz   = 3,
          .buf      = fsm_outs },
#endif
      };
      sns_com_port_read_reg_v( dd_handle, state_vectors_fsm_mlc, ARR_SIZE(state_vectors_fsm_mlc) );
    }

#if LSM6DSO_MLC_ENABLED
    if( fsm_mlc_status[2] != 0 )
    {
      /* Read mlc_src from embedded space */
      sns_com_port_vector_s state_vectors_mlc[] =
      {
        { .reg_addr = STM_LSM6DSO_REG_MLC_SRC,
          .buf_sz   = 8,
          .buf      = mlc_src },
      };
      sns_com_port_read_reg_v( dd_handle, state_vectors_mlc, ARR_SIZE(state_vectors_mlc) );
    }
#endif

    /* Disable embedded src */
    emb_mask = 0x00;
    sns_com_port_vector_s vectors_emb_w_2[] =
    {
      { .reg_addr = STM_LSM6DSO_REG_FUNC_CFG,
        .buf_sz   = 1,
        .buf      = &emb_mask },
    };
    sns_com_port_write_reg_v( dd_handle, vectors_emb_w_2, ARR_SIZE(vectors_emb_w_2) );
  }
#endif

#if LSM6DSO_FSM_ENABLED
  if( (fsm_mlc_status[0] != 0) || (fsm_mlc_status[1] != 0) )
  {
    fsm_reg_vals[0] = STM_LSM6DSO_REG_FSM_STATUS_MAINPAGE;
    fsm_reg_vals[1] = fsm_mlc_status[0];
    fsm_reg_vals[2] = fsm_mlc_status[1];

    /* Pass FSM_OUTS value for upto 3 FSMs */
    fsm_reg_vals[3] = fsm_outs[0];
    fsm_reg_vals[4] = fsm_outs[1];
    fsm_reg_vals[5] = fsm_outs[2];

    /* Pass FSM Long Counter value */
    fsm_reg_vals[6] = fsm_lc[0];
    fsm_reg_vals[7] = fsm_lc[1];
  }
#endif

#if LSM6DSO_MLC_ENABLED
  if( fsm_mlc_status[2] != 0 )
  {
    mlc_reg_vals[0] = STM_LSM6DSO_REG_MLC_STATUS_MAINPAGE;
    mlc_reg_vals[1] = fsm_mlc_status[2];
    mlc_reg_vals[2] = mlc_src[0];
    mlc_reg_vals[3] = mlc_src[1];
    mlc_reg_vals[4] = mlc_src[2];
    mlc_reg_vals[5] = mlc_src[3];
    mlc_reg_vals[6] = mlc_src[4];
    mlc_reg_vals[7] = mlc_src[5];
  }
#endif
#endif //LSM6DSO_FSM_ENABLED || LSM6DSO_MLC_ENABLED

#if LSM6DSO_ESP_DOUBLE_TAP
  uint8_t tap_src = 0;
  sns_com_port_vector_s state_vectors_dbt[] =
  {
    { .reg_addr = STM_LSM6DSO_REG_TAP_SRC,
      .buf_sz   = 1,
      .buf      = &tap_src},
  };
  sns_com_port_read_reg_v( dd_handle, state_vectors_dbt, ARR_SIZE(state_vectors_dbt) );

#if LSM6DSO_EX_TAP_TUNING_ENABLED
    tap_src &= STM_LSM6DSO_SINGLE_TAP_MASK;
#else
    tap_src &= STM_LSM6DSO_DOUBLE_TAP_MASK;
#endif
  if(tap_src != 0)
  {
    tap_reg_vals[0] = STM_LSM6DSO_REG_TAP_SRC;
    tap_reg_vals[1] = tap_src;
  }
#endif //LSM6DSO_ESP_DOUBLE_TAP
}

/* ------------------------------------------------------------------------------------ */
static void lsm6dso_notify_other_interrupts(
  sns_dd_handle_s*  dd_handle,
  notify_interrupt  notify_int_fptr,
  uint8_t           fsm_reg_vals[8],
  uint8_t           mlc_reg_vals[8],
  uint8_t           tap_reg_vals[2])
{
#if LSM6DSO_FSM_ENABLED || LSM6DSO_MLC_ENABLED
  if( fsm_reg_vals[0] != 0 )
  {
    notify_int_fptr( dd_handle, fsm_reg_vals );
  }
  if( mlc_reg_vals[0] != 0 )
  {
    notify_int_fptr( dd_handle, mlc_reg_vals );
  }
#endif

#if LSM6DSO_ESP_DOUBLE_TAP
  if(tap_reg_vals[0] != 0 )
  {
    notify_int_fptr( dd_handle, tap_reg_vals);
  }
#endif
}

/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dso_fifo_get_data( sns_dd_handle_s*    dd_handle,
                       read_sensor_data    data_read_fptr,
                       notify_interrupt    notify_int_fptr,
                       int32_t             acc_delay,
                       int32_t*            delay_us,
                       bool*               call_again,
                       int32_t*            num_samples )
{
  sns_com_port_status_e status;
  uint8_t wake_src       = 0;
  uint8_t fifo_status[2] = {0};
  uint8_t fifo_ctrl[3]   = {0};
#if LSM6DSO_TS_IN_META_DATA
  /**Send current time 2nd last byte to be used for flush request*/
  uint8_t time = (sns_get_system_time() >> 8) & 0xFF;
#endif
  /* Status registers for use in this function */
  sns_com_port_vector_s state_vectors[] =
  {
    { .reg_addr = STM_LSM6DSO_REG_FIFO_CTRL1,
      .buf_sz   = 3,
      .buf      = fifo_ctrl       },
    { .reg_addr = STM_LSM6DSO_REG_WAKE_SRC,
      .buf_sz   = 1,
      .buf      = &wake_src       },
    { .reg_addr = STM_LSM6DSO_REG_FIFO_STATUS1,
      .buf_sz   = 2,
      .buf      = fifo_status     },
  };
  status = sns_com_port_read_reg_v( dd_handle, state_vectors, ARR_SIZE(state_vectors) );
      
  if( status == SNS_COM_PORT_STATUS_SUCCESS && (wake_src != 0xff))
  {
    uint8_t fsm_reg_vals[8] = {0};
    uint8_t mlc_reg_vals[8] = {0};
    uint8_t tap_reg_vals[2] = {0};
    uint16_t count_h = fifo_status[1] & STM_LSM6DSO_FIFO_DIFF2_MASK;
    uint16_t count_l = fifo_status[0] & STM_LSM6DSO_FIFO_DIFF1_MASK;
    *num_samples = ((count_h << 8) & 0xFF00) | count_l;

    if( !*num_samples && (fifo_status[1] & STM_LSM6DSO_FIFO_OVERFLOW_MASK))
      *num_samples = STM_FIFO_OVERFLOW_SAMPLE_CNT;
    if( *num_samples > 0 && (fifo_status[1] & 0x10) == 0)
    {
      uint8_t bdr_src = 0x40;
      sns_com_port_vector_s vectors_w[] =
      {
        { .reg_addr = STM_LSM6DSO_REG_CNT_BDR1,
          .buf_sz   = 1,
          .buf      = &bdr_src },
      };
      status = sns_com_port_write_reg_v( dd_handle, vectors_w, ARR_SIZE(vectors_w) );
      if((fifo_ctrl[2] & 0xF0) != 0)
      {
        // If flush and if gyro_enabled, read only even number of samples
        *num_samples &= ~1;
      }
    }

    lsm6dso_check_other_interrupts(dd_handle, fsm_reg_vals, mlc_reg_vals, tap_reg_vals);

    if( *num_samples > 0 )
    {
      sns_com_port_data_vector_s data_vectors[] = 
      {
        { .mem_addr = fifo_ctrl, .reg_addr = 0,
          .buf_sz   = 3 },
        { .mem_addr = &fifo_status[1], .reg_addr = 0,
          .buf_sz   = 1 },
#if LSM6DSO_TS_IN_META_DATA
        { .mem_addr = &time, .reg_addr = 0,
          .buf_sz   = 1 },
#endif
        { .reg_addr = STM_LSM6DSO_REG_FIFO_OUT_TAG,
          .buf_sz   = *num_samples * STM_LSM6DSO_FIFO_SAMPLE_SIZE },
      };

      status = data_read_fptr( dd_handle, data_vectors, ARR_SIZE(data_vectors) );
      *delay_us = 0;
      *call_again = false;
    }

    wake_src &= ~0xD0;

    if( wake_src != 0 )
    {
      /* regvals[0] = STM_LSM6DSO_REG_WAKE_SRC; -> wake_src(MD, FF, HS) */
      uint8_t regvals[8] = {0};
      regvals[0] = STM_LSM6DSO_REG_WAKE_SRC;
      regvals[1] = wake_src;
      notify_int_fptr( dd_handle, regvals );
    }

    lsm6dso_notify_other_interrupts(dd_handle, notify_int_fptr, fsm_reg_vals, mlc_reg_vals, tap_reg_vals);
  }
  return status;
}


/* ------------------------------------------------------------------------------------ */
static sns_com_port_status_e
lsm6dso_fifo_parse_accel_data( sns_dd_handle_s*        dd_handle,
                               uint8_t const*          data_ptr,
                               uint16_t                data_bytes,
                               notify_accel_data       notify_data_fptr,
                               notify_accel_sample_cnt notify_cnt_fptr )
{
  sns_com_port_status_e rv = SNS_COM_PORT_STATUS_ERROR;

  if( data_bytes >= STM_LSM6DSO_FIFO_SAMPLE_SIZE )
  {
    int i;
    int32_t accel_sample_count = 0;
    int32_t loop_start = META_DATA_SIZE; /* first 4 bytes are meta data */

    /* counts up number of Accel samples */
    for( i = loop_start; i < data_bytes; i += STM_LSM6DSO_FIFO_SAMPLE_SIZE )
    {
      uint8_t tag = data_ptr[i] >> 3;
      if( tag == STM_LSM6DSO_ACC_TAG )
      {
        accel_sample_count++;
      }
    }

    if( accel_sample_count > 0 )
    {
      notify_cnt_fptr( dd_handle, accel_sample_count );
      for( i = loop_start; i < data_bytes; i += STM_LSM6DSO_FIFO_SAMPLE_SIZE )
      {
        uint8_t tag = data_ptr[i] >> 3;
        if( tag == STM_LSM6DSO_ACC_TAG )
        {
          int16_t x,y,z;
          x = ((data_ptr[i+2] << 8) | data_ptr[i+1]);
          y = ((data_ptr[i+4] << 8) | data_ptr[i+3]);
          z = ((data_ptr[i+6] << 8) | data_ptr[i+5]);
          notify_data_fptr( dd_handle, x, y, z );
        }
      }
      rv = SNS_COM_PORT_STATUS_SUCCESS;
    }
  }
  return  rv;
}

/**
*****************************************************************************************
                            Global Function Pointer Table
*****************************************************************************************
*/

sns_dd_if_s lsm6dso_fifo_hal_table =
  { .get_data = lsm6dso_fifo_get_data,
    .parse_accel_data = lsm6dso_fifo_parse_accel_data };

sns_dd_if_s lsm6dso_fifo_hal_table2 =
  { .get_data = lsm6dso_fifo_get_data,
    .parse_accel_data = lsm6dso_fifo_parse_accel_data };

