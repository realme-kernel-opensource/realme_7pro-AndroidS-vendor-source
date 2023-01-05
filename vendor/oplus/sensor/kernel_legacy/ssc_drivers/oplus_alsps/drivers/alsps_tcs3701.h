/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/
#ifdef SUPPORT_TCS3701
#pragma once

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_gpio_service.h"
#include "oplus_alsps.h"


/* Register map */
#define TCS3701_ENABLE_REG              0x80
#define TCS3701_ATIME_REG               0x81
#define TCS3701_PTIME_REG               0x82
#define TCS3701_WTIME_REG               0x83
#define TCS3701_AILTL_REG               0x84
#define TCS3701_AILTH_REG               0x85
#define TCS3701_AIHTL_REG               0x86
#define TCS3701_AIHTH_REG               0x87
#define TCS3701_PILT0L_REG              0x88
#define TCS3701_PILT0H_REG              0x89
#define TCS3701_PILT1L_REG              0x8A
#define TCS3701_PILT1H_REG              0x8B
#define TCS3701_PIHT0L_REG              0x8C
#define TCS3701_PIHT0H_REG              0x8D
#define TCS3701_PIHT1L_REG              0x8E
#define TCS3701_PIHT1H_REG              0x8F
#define TCS3701_AUXID_REG               0x90
#define TCS3701_REVID_REG               0x91
#define TCS3701_ID_REG                  0x92
#define TCS3701_STATUS_REG              0x93
#define TCS3701_ASTATUS_REG             0x94
#define TCS3701_ADATA0L_REG             0x95
#define TCS3701_ADATA0H_REG             0x96
#define TCS3701_ADATA1L_REG             0x97
#define TCS3701_ADATA1H_REG             0x98
#define TCS3701_ADATA2L_REG             0x99
#define TCS3701_ADATA2H_REG             0x9A
#define TCS3701_ADATA3L_REG             0x9B
#define TCS3701_ADATA3H_REG             0x9C
#define TCS3701_ADATA4L_REG             0x9D
#define TCS3701_ADATA4H_REG             0x9E
#define TCS3701_ADATA5L_REG             0x9F
#define TCS3701_ADATA5H_REG             0xA0
#define TCS3701_PDATAL_REG              0xA1
#define TCS3701_PDATAH_REG              0xA2
#define TCS3701_STATUS2_REG             0xA3
#define TCS3701_STATUS3_REG             0xA4
#define TCS3701_STATUS5_REG             0xA6
#define TCS3701_STATUS6_REG             0xA7
#define TCS3701_CFG0_REG                0xA9
#define TCS3701_CFG1_REG                0xAA
#define TCS3701_CFG3_REG                0xAC
#define TCS3701_CFG4_REG                0xAD
#define TCS3701_CFG8_REG                0xB1
#define TCS3701_CFG10_REG               0xB3
#define TCS3701_CFG11_REG               0xB4
#define TCS3701_CFG12_REG               0xB5
#define TCS3701_CFG14_REG               0xB7
#define TCS3701_PCFG1_REG               0xB8
#define TCS3701_PCFG2_REG               0xB9
#define TCS3701_PCFG4_REG               0xBB
#define TCS3701_PCFG5_REG               0xBC
#define TCS3701_PERS_REG                0xBD
#define TCS3701_GPIO_REG                0xBE
#define TCS3701_POFFSETL_REG            0xC7
#define TCS3701_POFFSETH_REG            0xC8
#define TCS3701_ASTEPL_REG              0xCA
#define TCS3701_ASTEPH_REG              0xCB
#define TCS3701_AGC_GAIN_MAX_REG        0xCF
#define TCS3701_PXAVGL_REG              0xD0
#define TCS3701_PXAVGH_REG              0xD1
#define TCS3701_PBSLNL_REG              0xD2
#define TCS3701_PBSLNH_REG              0xD3
#define TCS3701_AZ_CONFIG_REG           0xD6
#define TCS3701_CALIB_REG               0xEA
#define TCS3701_CALIBCFG0_REG           0xEB
#define TCS3701_CALIBCFG1_REG           0xEC
#define TCS3701_CALIBCFG2_REG           0xED
#define TCS3701_CALIBSTAT_REG           0xEE
#define TCS3701_INTENAB_REG             0xF9
#define TCS3701_CONTROL_REG             0xFA
#define TCS3701_RVED1_REG               0xFC
#define TCS3701_RVED2_REG               0xFD
#define TCS3701_RVED3_REG               0xFE
#define TCS3701_RVED4_REG               0xFF

/* Register bits map */
//ENABLE @ 0x80
#define PON                             (0x01 << 0)
#define AEN                             (0x01 << 1)
#define PEN                             (0x01 << 2)
#define WEN                             (0x01 << 3)

//AUXID @ 0x90
#define AUXID_MASK                      (0x0F << 0)

//REVID @ 0x91
#define REVID_MASK                      (0x07 << 0)

//ID_MASK @ 0x92
#define ID_MASK                         (0x3F << 2)

//STATUS @ 0x93
#define SINT                            (0x01 << 0)
#define CINT                            (0x01 << 1)
#define AINT                            (0x01 << 3)
#define PINT0                           (0x01 << 4)
#define PINT1                           (0x01 << 5)
#define PSAT                            (0x01 << 6)
#define ASAT                            (0x01 << 7)

//ASTATUS @0x94
#define AGAIN_STATUS_SHIFT              0
#define AGAIN_STATUS_MASK               (0x0F << AGAIN_STATUS_SHIFT)
#define ASAT_STATUS                     (0x01 << 7)

//STATUS2 @0xA3
#define ASAT_ANALOG                     (0x01 << 3)
#define ASAT_DIGITAL                    (0x01 << 4)
#define PVALID                          (0x01 << 5)
#define AVALID                          (0x01 << 6)

//STATUS3 @0xA4
#define PSAT_AMBIENT                    (0x01 << 0)
#define PSAT_REFLECTIVE                 (0x01 << 1)
#define PSAT_ADC                        (0x01 << 2)
#define STATUS3_RVED                    (0x01 << 3)
#define AINT_AILT                       (0x01 << 4)
#define AINT_AIHT                       (0x01 << 5)

//STATUS4 @0xA5
#define PINT0_PILT                      (0x01 << 0)
#define PINT0_PIHT                      (0x01 << 1)
#define PINT1_PILT                      (0x01 << 2)
#define PINT1_PIHT                      (0x01 << 3)

//STATUS6 @0xA7
#define INIT_BUSY                       (0x01 << 0)
#define SAI_ACTIVE                      (0x01 << 1)
#define ALS_TRIGGER_ERROR               (0x01 << 2)
#define PROX_TRIGGER_ERROR              (0x01 << 3)
#define OVTEMP_DETECTED                 (0x01 << 5)

//CFG0 @0xA9
#define ALS_TRIGGER_LONG                (0x01 << 2)
#define PROX_TRIGGER_LONG               (0x01 << 3)
#define LOWPOWER_IDLE                   (0x01 << 5)

//CFG1 @0xAA
#define AGAIN_SHIFT                     0
#define AGAIN_MASK                      (0x1F << AGAIN_SHIFT)
#define AGAIN_0_5X                      (0x00 << AGAIN_SHIFT)
#define AGAIN_1X                        (0x01 << AGAIN_SHIFT)
#define AGAIN_2X                        (0x02 << AGAIN_SHIFT)
#define AGAIN_4X                        (0x03 << AGAIN_SHIFT)
#define AGAIN_8X                        (0x04 << AGAIN_SHIFT)
#define AGAIN_16X                       (0x05 << AGAIN_SHIFT)
#define AGAIN_32X                       (0x06 << AGAIN_SHIFT)
#define AGAIN_64X                       (0x07 << AGAIN_SHIFT)
#define AGAIN_128X                      (0x08 << AGAIN_SHIFT)
#define AGAIN_256X                      (0x09 << AGAIN_SHIFT)
#define AGAIN_512X                      (0x0A << AGAIN_SHIFT)
#define AGAIN_1024X                     (0x0B << AGAIN_SHIFT)
#define AGAIN_2048X                     (0x0C << AGAIN_SHIFT)

//CFG3 @0xAC
#define CFG3_RVED                       (0x0C << 0)
#define SAI                             (0x01 << 4)
#define HXTALK_MODE1                    (0x01 << 5)


//CFG8_REG @0xB1
#define SWAP_PROX_ALS5                  (0x01 << 0)
#define ALS_AGC_ENABLE                  (0x01 << 2)
#define CONCURRENT_PROX_AND_ALS         (0x01 << 4)

//CFG10_REG @0xB3
#define ALS_AGC_LOW_HYST_MASK           (0x03 << 4)
#define ALS_AGC_LOW_HYST_12_5           (0x00 << 4)
#define ALS_AGC_LOW_HYST_25             (0x01 << 4)
#define ALS_AGC_LOW_HYST_37_5           (0x02 << 4)
#define ALS_AGC_LOW_HYST_50             (0x03 << 4)
#define ALS_AGC_HIGH_HYST_MASK          (0x03 << 6)
#define ALS_AGC_HIGH_HYST_50            (0x00 << 6)
#define ALS_AGC_HIGH_HYST_62_5          (0x01 << 6)
#define ALS_AGC_HIGH_HYST_75            (0x02 << 6)
#define ALS_AGC_HIGH_HYST_87_5          (0x03 << 6)

//CFG11_REG @0xB4
#define PINT_DIRECT                     (0x01 << 6)
#define AINT_DIRECT                     (0x01 << 7)

//CFG12_REG @0xB5
#define ALS_TH_CHANNEL_MASK             (0x07 << 0)
#define ALS_TH_CHANNEL_0                (0x00 << 0)
#define ALS_TH_CHANNEL_1                (0x01 << 0)
#define ALS_TH_CHANNEL_2                (0x02 << 0)
#define ALS_TH_CHANNEL_3                (0x03 << 0)
#define ALS_TH_CHANNEL_4                (0x04 << 0)

//CFG14_REG @0xB7
#define PROX_OFFSET_COARSE_MASK         (0x1F << 0)
#define EN_PROX_OFFSET_RANGE            (0x01 << 5)
#define AUTO_CO_CAL_EN                  (0x01 << 6)

//PCFG1_REG @0xB8
#define PROX_FILTER_MASK                (0x03 << 0)
#define PROX_FILTER_1                   (0x00 << 0)
#define PROX_FILTER_2                   (0x01 << 0)
#define PROX_FILTER_4                   (0x02 << 0)
#define PROX_FILTER_8                   (0x03 << 0)
#define PROX_FILTER_DOWNSAMPLE          (0x01 << 2)
#define PCFG1_RVED                      (0x01 << 3)
#define HXTALK_MODE2                    (0x01 << 7)

//PCFG2_REG @0xB9
#define PLDRIVE0_SHIFT                  0
#define PLDRIVE0_MASK                   (0x7F << PLDRIVE0_SHIFT)//2xPLDRIVE0 + 4mA

//PCFG4_REG @0xBB
#define PGAIN_SHIFT                     0
#define PGAIN_MASK                      (0x03 << PGAIN_SHIFT)
#define PGAIN_1X                        (0x00 << PGAIN_SHIFT)
#define PGAIN_2X                        (0x01 << PGAIN_SHIFT)
#define PGAIN_4X                        (0x02 << PGAIN_SHIFT)
#define PGAIN_8X                        (0x03 << PGAIN_SHIFT)

//PCFG5_REG @0xBC
#define PPULSE_SHIFT                    0
#define PPULSE_MASK                     (0x3F << PPULSE_SHIFT)
#define PPULSE_LEN_SHIFT                6
#define PPULSE_LEN_MASK                 (0x03 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_4US                  (0x00 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_8US                  (0x01 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_16US                 (0x02 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_32US                 (0x03 << PPULSE_LEN_SHIFT)

//PERS_REG @0xBD
#define APERS_SHIFT                     0
#define APERS_MASK                      (0x0F << APERS_SHIFT)
#define PPERS_SHIFT                     4
#define PPERS_MASK                      (0x0F << PPERS_SHIFT)

//GPIO_REG @0xBE
#define GPIO_IN                         (0x01 << 0)
#define GPIO_OUT                        (0x01 << 1)
#define GPIO_IN_EN                      (0x01 << 2)
#define GPIO_INVERT                     (0x01 << 3)

//GAIN_MAX_REG @0xCF
#define AGC_AGAIN_MAX_MASK              (0x0F << 0) //2^(AGC_AGAIN_MAX)

//CALIB_REG @0xEA
#define START_OFFSET_CALIB              (0x01 << 0)

//CALIBCFG0_REG @0xEB
#define DCAVG_ITERATIONS_MASK           (0x07 << 0)//0 is skip, 2^(ITERATIONS)
#define BINSRCH_SKIP                    (0x01 << 3)
#define DCAVG_AUTO_OFFSET_ADJUST        (0x01 << 6)
#define DCAVG_AUTO_BSLN                 (0x01 << 7)

//CALIBCFG1_REG @0xEC
#define PXAVG_ITERATIONS_MASK           (0x07 << 0)//0 is skip, 2^(ITERATIONS)
#define PXAVG_AUTO_BSLN                 (0x01 << 3)
#define PROX_AUTO_OFFSET_ADJUST         (0x01 << 6)

//CALIBCFG1_REG @0xED
#define BINSRCH_TARGET_SHIFT            5
#define BINSRCH_TARGET_MASK             (0x07 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_3                (0x00 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_7                (0x01 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_15               (0x02 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_31               (0x03 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_63               (0x04 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_127              (0x05 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_255              (0x06 << BINSRCH_TARGET_SHIFT)
#define BINSRCH_TARGET_511              (0x07 << BINSRCH_TARGET_SHIFT)

//CALIBSTAT_REG @0xEE
#define CALIB_FINISHED                  (0x01 << 0)
#define OFFSET_ADJUSTED                 (0x01 << 1)
#define BASELINE_ADJUSTED               (0x01 << 2)

//INTENAB_REG @0xF9
#define SIEN                            (0x01 << 0)
#define CIEN                            (0x01 << 1)
#define AIEN                            (0x01 << 3)
#define PIEN0                           (0x01 << 4)
#define PIEN1                           (0x01 << 5)
#define PSIEN                           (0x01 << 6)
#define ASIEN                           (0x01 << 7)

//CONTROL_REG @0xFA
#define CLEAR_SAI_ACTIVE                (0x01 << 0)
#define ALS_MANUAL_AZ                   (0x01 << 2)

//Configration calculations
#define ASTEP_US_PER_100                278
#define ASTEP_US(us)                    (uint16_t)(((uint32_t)us*100 + (ASTEP_US_PER_100 >> 1)) / ASTEP_US_PER_100 - 1)
#define PTIME_MS_PER_100                278
#define PTIME_MS(ms)                    (uint8_t)(((uint32_t)ms*100 + (PTIME_MS_PER_100 >> 1)) / PTIME_MS_PER_100 - 1)
#define WTIME_MS_PER_100                278
#define WTIME_MS(ms)                    (uint8_t)(((uint32_t)ms*100 + (WTIME_MS_PER_100 >> 1)) / WTIME_MS_PER_100 - 1)
#define PLDRIVE_MA(ma)                  (uint8_t)(((ma-4) >> 1) << PLDRIVE0_SHIFT)
#define PPULSES(c)                      (uint8_t)((c - 1) << PPULSE_SHIFT)
#define ALS_PERSIST(p)                  (uint8_t)(((p) & 0x0F) << APERS_SHIFT)
#define PROX_PERSIST(p)                 (uint8_t)(((p) & 0x0F) << PPERS_SHIFT)

/* ALS channel and buffer control */
#define ALS_1CH                         1
#define ALS_2CH                         2
#define ALS_3CH                         3
#define ALS_4CH                         4


/*oplus customization*/
#define ALSPS_PERSIST_CONFIG             1
#define ALS_TIMER_MS                    20

#define MAX_ALS_VALUEMAX_ALS_VALUE 0xffff

typedef struct tcs3701_reg_setting {
    uint8_t reg;
    uint8_t value;
} tcs3701_reg_setting;

typedef enum {
    TCS3701_ALS             = 0x01,
    TCS3701_PROX            = 0x02,
} tcs3701_sensor_type;

sns_rc tcs3701_recover_device(sns_sync_com_port_service *scp_service,
    sns_sync_com_port_handle *port_handle);

typedef struct tcs3701_instance_state {
    uint8_t                 publish_sensors;
    bool                    first_als;
    float                   ir_ratio;
    float                   atime;
    uint32_t                again;
    float                   lux;
    float                   last_lux;
    uint16_t    far_thd;
    uint16_t    near_thd;
    bool        first_prox;
    float    ch_raw[4];
    int                offset;
    uint32_t saturation;
    uint32_t low_saturation;
    bool use_fifo;
    uint16_t brightness;
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    als_algo_state algo_state;
#endif
} tcs3701_instance_state;
#endif
