/***************************************************************
** Copyright (C), 2010-2020, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - alsps_stk3a6x.h
** Description: Source file for oplus alsps.
** Version: 1.0
** Date : 2020/07/20
**
** ---------------------Revision History: ---------------------
** <author>                     <date>      <version>    <desc>
****************************************************************/
#ifdef SUPPORT_STK32600
#pragma once

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_gpio_service.h"

/**
 * Address registers
 */
#define STK3A6X_REG_STATE                   0x00
#define STK3A6X_REG_PSCTRL                  0x01
#define STK3A6X_REG_ALSCTRL1                0x02
#define STK3A6X_REG_LEDCTRL                 0x03
#define STK3A6X_REG_INTCTRL1                0x04
#define STK3A6X_REG_WAIT                    0x05
#define STK3A6X_REG_THDH1_PS                0x06
#define STK3A6X_REG_THDH2_PS                0x07
#define STK3A6X_REG_THDL1_PS                0x08
#define STK3A6X_REG_THDL2_PS                0x09
#define STK3A6X_REG_THDH1_ALS               0x0A
#define STK3A6X_REG_THDH2_ALS               0x0B
#define STK3A6X_REG_THDL1_ALS               0x0C
#define STK3A6X_REG_THDL2_ALS               0x0D
#define STK3A6X_REG_FLAG                    0x10
#define STK3A6X_REG_DATA1_PS                0x11
#define STK3A6X_REG_DATA2_PS                0x12
#define STK3A6X_REG_DATA1_ALS               0x13
#define STK3A6X_REG_DATA2_ALS               0x14
#define STK3A6X_REG_DATA1_R                 0x15
#define STK3A6X_REG_DATA2_R                 0x16
#define STK3A6X_REG_DATA1_ALS1              0x17
#define STK3A6X_REG_DATA2_ALS1              0x18
#define STK3A6X_REG_DATA1_B                 0x19
#define STK3A6X_REG_DATA2_B                 0x1A
#define STK3A6X_REG_DATA1_C                 0x1B
#define STK3A6X_REG_DATA2_C                 0x1C
#define STK3A6X_REG_DATA1_PS_OFFSET         0x1D
#define STK3A6X_REG_DATA2_PS_OFFSET         0x1E
#define STK3A6X_REG_PDT_ID                  0x3E
#define STK3A6X_REG_RSRVD                   0x3F
#define STK3A6X_REG_GAINCTRL                0x4E
#define STK3A6X_REG_AGCTRL                  0xDB
#define STK3A6X_REG_ORIGINAL_DATA1_PS       0x38
#define STK3A6X_REG_ORIGINAL_DATA2_PS       0x39

/* Define state reg */
#define  STK3A6X_STATE_EN_CT_AUTOK_SHIFT        4
#define  STK3A6X_STATE_EN_INTELL_PRST_SHIFT     3
#define  STK3A6X_STATE_EN_WAIT_SHIFT            2
#define  STK3A6X_STATE_EN_ALS_SHIFT             1
#define  STK3A6X_STATE_EN_PS_SHIFT              0
#define  STK3A6X_STATE_EN_CT_AUTOK_MASK         0x10
#define  STK3A6X_STATE_EN_INTELL_PRST_MASK      0x08
#define  STK3A6X_STATE_EN_WAIT_MASK             0x04
#define  STK3A6X_STATE_EN_ALS_MASK              0x02
#define  STK3A6X_STATE_EN_PS_MASK               0x01

/* Define PS ctrl reg */
#define  STK3A6X_PS_PRS_SHIFT            6
#define  STK3A6X_PS_GAIN_SHIFT           4
#define  STK3A6X_PS_IT_SHIFT             0
#define  STK3A6X_PS_PRS_MASK             0xC0
#define  STK3A6X_PS_GAIN_MASK            0x30
#define  STK3A6X_PS_IT_MASK              0x0F

/* Define ALS ctrl reg */
#define  STK3A6X_ALS_PRS_SHIFT           6
#define  STK3A6X_ALS_GAIN_SHIFT          4
#define  STK3A6X_ALS_IT_SHIFT            0
#define  STK3A6X_ALS_PRS_MASK            0xC0
#define  STK3A6X_ALS_GAIN_MASK           0x30
#define  STK3A6X_ALS_IT_MASK             0x0F

/* Define LED ctrl reg */
#define  STK3A6X_EN_CTIR_SHIFT           0
#define  STK3A6X_EN_CTIRFC_SHIFT         1
#define  STK3A6X_LED_IRDR_SHIFT          6
#define  STK3A6X_EN_CTIR_MASK            0x01
#define  STK3A6X_EN_CTIRFC_MASK          0x02
#define  STK3A6X_LED_IRDR_MASK           0xC0

/* Define interrupt reg */
#define  STK3A6X_INT_CTRL_SHIFT          7
#define  STK3A6X_INT_INVALID_PS_SHIFT    5
#define  STK3A6X_INT_ALS_SHIFT           3
#define  STK3A6X_INT_PS_SHIFT            0

#define  STK3A6X_INT_CTRL_MASK           0x80
#define  STK3A6X_INT_INVALID_PS_MASK     0x20
#define  STK3A6X_INT_ALS_MASK            0x08
#define  STK3A6X_INT_PS_MASK             0x07

#define  STK3A6X_INT_PS_MODE1            0x01
#define  STK3A6X_INT_ALS                 0x08
#define  STK3A6X_INT_PS                  0x01

/* Define flag reg */
#define  STK3A6X_FLG_ALSDR_SHIFT            7
#define  STK3A6X_FLG_PSDR_SHIFT             6
#define  STK3A6X_FLG_ALSINT_SHIFT           5
#define  STK3A6X_FLG_PSINT_SHIFT            4
#define  STK3A6X_FLG_ALSSAT_SHIFT           2
#define  STK3A6X_FLG_INVALID_PSINT_SHIFT    1
#define  STK3A6X_FLG_NF_SHIFT               0

#define  STK3A6X_FLG_ALSDR_MASK          0x80
#define  STK3A6X_FLG_PSDR_MASK           0x40
#define  STK3A6X_FLG_ALSINT_MASK         0x20
#define  STK3A6X_FLG_PSINT_MASK          0x10
#define  STK3A6X_FLG_ALSSAT_MASK         0x04
#define  STK3A6X_FLG_INVALID_PSINT_MASK  0x02
#define  STK3A6X_FLG_NF_MASK             0x01

/* Define ALS CTRL-2 reg */
#define  STK3A6X_ALSC_GAIN_SHIFT         0x04
#define  STK3A6X_ALSC_GAIN_MASK          0x30
#define  STK3A6X_ALSC2_DGAIN_X128        0x04
#define  STK3A6X_C1_DGAIN_X128           0x02
#define  STK3A6X_PS_DGAIN_X16            0x01

/* Define INT-2 reg */
#define  STK3A6X_INT_ALS_DR_SHIFT        0x01
#define  STK3A6X_INT_PS_DR_SHIFT         0x00
#define  STK3A6X_INT_ALS_DR_MASK         0x02
#define  STK3A6X_INT_PS_DR_MASK          0x01

/* Define ALS/PS parameters */
#define  STK3A6X_PS_PRS1                0x00
#define  STK3A6X_PS_PRS2                0x40
#define  STK3A6X_PS_PRS4                0x80
#define  STK3A6X_PS_PRS8                0xC0

#define  STK3A6X_PS_GAIN1               0x00
#define  STK3A6X_PS_GAIN2               0x10
#define  STK3A6X_PS_GAIN4               0x20
#define  STK3A6X_PS_GAIN8               0x30

#define  STK3A6X_PS_IT100               0x02
#define  STK3A6X_PS_IT200               0x03
#define  STK3A6X_PS_IT400               0x04
#define  STK3A6X_PS_IT800               0x05
#define  STK3A6X_PS_IT1600              0x06

#define  STK3A6X_ALS_PRS1               0x00
#define  STK3A6X_ALS_PRS2               0x40
#define  STK3A6X_ALS_PRS4               0x80
#define  STK3A6X_ALS_PRS8               0xC0

#define  STK3A6X_ALS_GAIN1              0x00
#define  STK3A6X_ALS_GAIN4              0x10
#define  STK3A6X_ALS_GAIN16             0x20
#define  STK3A6X_ALS_GAIN64             0x30

#define  STK3A6X_ALS_IT25               0x03
#define  STK3A6X_ALS_IT50               0x04
#define  STK3A6X_ALS_IT12               0x02
#define  STK3A6X_ALS_IT100              0x05
#define  STK3A6X_ALS_IT200              0x06

#define  STK3A6X_ALSC_GAIN1             0x00
#define  STK3A6X_ALSC_GAIN4             0x10
#define  STK3A6X_ALSC_GAIN16            0x20
#define  STK3A6X_ALSC_GAIN64            0x30

#define  STK3A6X_LED_3_125MA            0x00
#define  STK3A6X_LED_6_25MA             0x04
#define  STK3A6X_LED_12_5MA             0x08
#define  STK3A6X_LED_25MA               0x10
#define  STK3A6X_LED_37MA               0x18
#define  STK3A6X_LED_43MA               0x1C
#define  STK3A6X_LED_50MA               0x20
#define  STK3A6X_LED_100MA              0x40
#define  STK3A6X_LED_150MA              0x60
#define  STK3A6X_LED_200MA              0x80

#define  STK3A6X_WAIT6                  0x03
#define  STK3A6X_WAIT20                 0x0C
#define  STK3A6X_WAIT50                 0x20
#define  STK3A6X_WAIT100                0x40

#define  STK3A6X_INTELL_20              0x32
#define  STK3A6X_BGIR_PS_INVALID        0x28

/* PID */
#define STK3A6X_PID_LIST_NUM            2

/* sw reset value */
#define STK_STK3A6X_SWRESET             0x00

/* Off to idle time */
#define STK3A6X_OFF_TO_IDLE_MS          10 //ms

/* ALS threshold */
#define STK3A6X_ALS_THD_ADJ             0.05
#define STK3A6X_NUM_AXES                3
#define STK3A6X_MAX_MIN_DIFF            250
#define STK3A6X_LT_N_CT                 150
#define STK3A6X_HT_N_CT                 250

#define STK3A6X_PRX_THD_NEAR            1000
#define STK3A6X_PRX_THD_FAR             900

#define STK3A6X_ALS_DATA_READY_TIME     60
#define STK3A6X_PRX_DATA_READY_TIME     10
#define STK3A6X_PRX_TUNE0_TIME          75

#define STK3A6X_ALS_THRESHOLD           30

#define STK3A6X_ALS_CALI_DATA_READY_US  55000000
#define STK3A6X_ALS_CALI_TARGET_LUX     500.0

#define STK3A6X_PS_CALI_TIMES           5
#define STK3A6X_PS_CALI_DATA_NUM        3
#define STK3A6X_PS_CALI_DATA_READY_US   55000000
#define STK3A6X_PS_CALI_MAX_CROSSTALK   3000
#define STK3A6X_PS_CALI_DIFF            40

#define STK3A6X_PS_OFF_THRESHOLD        0xFFFF
#define STK3A6X_PS_BGIR_THRESHOLD       0x64
#define STK3A6X_BOOT_CALI_ERROR_TIMES   2
#define ALS_FLAG_TIME                   100

#define STK3A6X_FIFO_COUNT1  0x64
#define STK3A6X_FIFO_COUNT2  0x65
#define STK3A6X_REG_FIFO_OUT 0x66
//#define STK_3A6X_ALS_GAP_TIME           25

sns_rc stk3a6x_get_who_am_i(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle  *port_handle);
void stk3a6x_dump_reg(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle);

typedef enum {
    STK3A6X_ALS             = 0x01,
    STK3A6X_PROX            = 0x02,
} stk3x6a_sensor_type;


typedef struct stk3x6a_driver_state {
    bool            first_ps;
    bool            first_als;
    uint8_t         als_gain;
    uint32_t        last_data_c;
    uint32_t        last_als;
    uint32_t        last_data_g;
    uint8_t         als_gain_level;
    uint32_t        last_c1;
    uint32_t        last_c2;
    uint32_t        last_ir;
    uint8_t         publish_sensors;
    int             offset;
}stk3x6a_driver_state;
#endif
