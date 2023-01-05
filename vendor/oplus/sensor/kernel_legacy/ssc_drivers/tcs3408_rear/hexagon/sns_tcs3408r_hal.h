#pragma once
/*
 * Copyright (c) 2018, ams AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_gpio_service.h"

#include "sns_tcs3408r_sensor_instance.h"

/* Default ALS/Color equation coefficients */
#define CALIB_TARGET_LUX                1000.0

#define CCT_COEF_ROW 3
#define CCT_COEF_COL 5
#define H_L_IR_Devers 0.5
#define RAW_NUM 5

#define CCT_DIV_POINT                 0.204
#define IR_DIV_POINT                  0.0766
#define CALIB_TARGET_CCT              3500.0

#define ATIME_PER_STEP_X100            278

/* Register map */
#define TCS3408_ENABLE_REG              0x80
#define TCS3408_ATIME_REG               0x81
#define TCS3408_PTIME_REG               0x82
#define TCS3408_WTIME_REG               0x83
#define TCS3408_AILTL_REG               0x84
#define TCS3408_AILTH_REG               0x85
#define TCS3408_AIHTL_REG               0x86
#define TCS3408_AIHTH_REG               0x87
#define TCS3408_PILT0L_REG              0x88
#define TCS3408_PILT0H_REG              0x89
#define TCS3408_PILT1L_REG              0x8A
#define TCS3408_PILT1H_REG              0x8B
#define TCS3408_PIHT0L_REG              0x8C
#define TCS3408_PIHT0H_REG              0x8D
#define TCS3408_PIHT1L_REG              0x8E
#define TCS3408_PIHT1H_REG              0x8F
#define TCS3408_AUXID_REG               0x90
#define TCS3408_REVID_REG               0x91
#define TCS3408_ID_REG                  0x92
#define TCS3408_STATUS_REG              0x93
#define TCS3408_ASTATUS_REG             0x94
#define TCS3408_ADATA0L_REG             0x95
#define TCS3408_ADATA0H_REG             0x96
#define TCS3408_ADATA1L_REG             0x97
#define TCS3408_ADATA1H_REG             0x98
#define TCS3408_ADATA2L_REG             0x99
#define TCS3408_ADATA2H_REG             0x9A
#define TCS3408_ADATA3L_REG             0x9B
#define TCS3408_ADATA3H_REG             0x9C
#define TCS3408_ADATA4L_REG             0x9D
#define TCS3408_ADATA4H_REG             0x9E
#define TCS3408_ADATA5L_REG             0x9F
#define TCS3408_ADATA5H_REG             0xA0
#define TCS3408_PDATAL_REG              0xA1
#define TCS3408_PDATAH_REG              0xA2
#define TCS3408_STATUS2_REG             0xA3
#define TCS3408_STATUS3_REG             0xA4
#define TCS3408_STATUS5_REG             0xA6
#define TCS3408_STATUS6_REG             0xA7
#define TCS3408_CFG0_REG                0xA9
#define TCS3408_CFG1_REG                0xAA
#define TCS3408_CFG3_REG                0xAC
#define TCS3408_CFG4_REG                0xAD
#define TCS3408_CFG8_REG                0xB1
#define TCS3408_CFG10_REG               0xB3
#define TCS3408_CFG11_REG               0xB4
#define TCS3408_CFG12_REG               0xB5
#define TCS3408_CFG14_REG               0xB7
#define TCS3408_PCFG1_REG               0xB8
#define TCS3408_PCFG2_REG               0xB9
#define TCS3408_PCFG4_REG               0xBB
#define TCS3408_PCFG5_REG               0xBC
#define TCS3408_PERS_REG                0xBD
#define TCS3408_GPIO_REG                0xBE
#define TCS3408_POFFSETL_REG            0xC7
#define TCS3408_POFFSETH_REG            0xC8
#define TCS3408_ASTEPL_REG              0xCA
#define TCS3408_ASTEPH_REG              0xCB
#define TCS3408_AGC_GAIN_MAX_REG        0xCF
#define TCS3408_PXAVGL_REG              0xD0
#define TCS3408_PXAVGH_REG              0xD1
#define TCS3408_PBSLNL_REG              0xD2
#define TCS3408_PBSLNH_REG              0xD3
#define TCS3408_AZ_CONFIG_REG           0xD6
#define TCS3408_FD_CFG0                 0xD7
#define TCS3408_FD_CFG1                 0xD8
#define TCS3408_FD_CFG3                 0xDA
#define TCS3408_CALIB_REG               0xEA
#define TCS3408_CALIBCFG0_REG           0xEB
#define TCS3408_CALIBCFG1_REG           0xEC
#define TCS3408_CALIBCFG2_REG           0xED
#define TCS3408_CALIBSTAT_REG           0xEE
#define TCS3408_INTENAB_REG             0xF9
#define TCS3408_CONTROL_REG             0xFA
#define TCS3408_FIFO_MAP                0xFC
#define TCS3408_FIFO_STATUS             0xFD
#define TCS3408_FDATAL                  0xFE
#define TCS3408_FDATAH                  0xFF

/* Register bits map */
//ENABLE @ 0x80
#define PON                             (0x01 << 0)
#define AEN                             (0x01 << 1)
#define PEN                             (0x01 << 2)
#define WEN                             (0x01 << 3)
#define FDEN                            (0x01 << 6)
#define POFF                             0x00
#define FD_MASK                         (0x01 << 6)

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

//CFG4 @0xAD
#define GPIO_PINMAP_DEFAULT             (0x00 << 0)
#define GPIO_PINMAP_RVED                (0x01 << 0)
#define GPIO_PINMAP_AINT                (0x02 << 0)
#define GPIO_PINMAP_PINT0               (0x03 << 0)
#define GPIO_PINMAP_PINT1               (0x04 << 0)
#define GPIO_PINMAP_MASK                (0x07 << 0)
#define INT_INVERT                      (0x01 << 3)
#define INT_PINMAP_NORMAL               (0x00 << 4)
#define INT_PINMAP_RVED                 (0x01 << 4)
#define INT_PINMAP_AINT                 (0x02 << 4)
#define INT_PINMAP_PINT0                (0x03 << 4)
#define INT_PINMAP_PINT1                (0x04 << 4)
#define INT_PINMAP_MASK                 (0x07 << 4)

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
#define FD_TIME_US_PER_100              278
#define FD_TIME_US(us)                  (uint16_t)(((uint32_t)us*100 + (FD_TIME_US_PER_100 >> 1)) / FD_TIME_US_PER_100 - 1)
#define PLDRIVE_MA(ma)                  (uint8_t)(((ma-4) >> 1) << PLDRIVE0_SHIFT)
#define PPULSES(c)                      (uint8_t)((c - 1) << PPULSE_SHIFT)
#define ALS_PERSIST(p)                  (uint8_t)(((p) & 0x0F) << APERS_SHIFT)
#define PROX_PERSIST(p)                 (uint8_t)(((p) & 0x0F) << PPERS_SHIFT)

/* For ALS/Color auto gain setting */
#define MAX_ALS_VALUE                   0xFFFF
#define	MIN_ALS_VALUE                   3
#define	GAIN_SWITCH_LEVEL               200

/* Proximity thresholds */
#define AMS_PROX_THRESH_NEAR            200 /* unit of ADC count */
#define AMS_PROX_THRESH_FAR             100 /* unit of ADC count */
#define AMS_PROX_THRESH_VERY_NEAR       500 /* unit of ADC count */
#define AMS_PROX_THRESH_CONTAMINATED    300 /* unit of ADC count */
#define AMS_PROX_MAX_VALUE              1023
#define AMS_CONTAMINATED_COUNT          16
#define AMS_VERY_NEAR_COUNT             4
#define PDATA_JITTER_TH                 3

/* Time */
#define TCS3408_OFF_TO_IDLE_MS          10 //ms
#define TCS3408_POLLING_MS              20
#define TCS3408_DELAY_AFTER_PON         1
#define TCS3408_MAX_CALIB_TIME          30

#define TCS3408_WHOAMI_VALUE            0x18
#define TCS3408_PARTNO_MASK             0xFC

/* If greater than this value, we think it is unreasonable */
#define TCS3408_MAX_POFFSET             150

//TCS3408_CONTROL_REG @ 0xFA
#define FIFO_CLEAR                  	0x02

//TCS3408_FD_CFG3 @0xDA
#define FD_GAIN_0_5X                      (0x00 << 3)
#define FD_GAIN_1X                        (0x01 << 3)
#define FD_GAIN_2X                        (0x02 << 3)
#define FD_GAIN_4X                        (0x03 << 3)
#define FD_GAIN_8X                        (0x04 << 3)
#define FD_GAIN_16X                       (0x05 << 3)
#define FD_GAIN_32X                       (0x06 << 3)
#define FD_GAIN_64X                       (0x07 << 3)
#define FD_GAIN_128X                      (0x08 << 3)
#define FD_GAIN_256X                      (0x09 << 3)
#define FD_GAIN_512X                      (0x0A << 3)
#define FD_GAIN_1024X                     (0x0B << 3)
#define FD_GAIN_2048X                     (0x0C << 3)
#define FD_GAIN_MASK                      (0x1F << 3)
#define FD_TIME_MASK                      (0x07)

#define AMS_DEBUG

#ifdef AMS_DEBUG
#include "msg.h"
#ifndef DBG_LOW_PRIO
#define DBG_LOW_PRIO   MSG_LEGACY_LOW
#endif
#ifndef DBG_HIGH_PRIO
#define DBG_HIGH_PRIO  MSG_LEGACY_HIGH
#endif
#ifndef DBG_MED_PRIO
#define DBG_MED_PRIO   MSG_LEGACY_MED
#endif
#ifndef DBG_ERROR_PRIO
#define DBG_ERROR_PRIO MSG_LEGACY_ERROR
#endif

/* The priority param, x, is ignored; always use DBG_HIGH_PRIO to insure mini-dm shows it */
#define TCS3408_PORT_log_Msg(x, a)								UMSG(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">")
#define TCS3408_PORT_log_Msg_1(x, a, b)							UMSG_1(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b)
#define TCS3408_PORT_log_Msg_2(x, a, b, c)						UMSG_2(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c)
#define TCS3408_PORT_log_Msg_3(x, a, b, c, d)					UMSG_3(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d)
#define TCS3408_PORT_log_Msg_4(x, a, b, c, d, e)				UMSG_4(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d, e)
#define TCS3408_PORT_log_Msg_5(x, a, b, c, d, e, f)				UMSG_5(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d, e, f)
#define TCS3408_PORT_log_Msg_6(x, a, b, c, d, e, f, g)			UMSG_6(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d, e, f, g)
#define TCS3408_PORT_log_Msg_7(x, a, b, c, d, e, f, g, h)		UMSG_7(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d, e, f, g, h)
#define TCS3408_PORT_log_Msg_8(x, a, b, c, d, e, f, g, h, i)	UMSG_8(MSG_SSID_QDSP6, DBG_HIGH_PRIO, "TCS3408: - <" a ">", b, c, d, e, f, g, h, i)

#endif

typedef struct tcs3408_reg_setting {
	uint8_t reg;
	uint8_t value;
} tcs3408_reg_setting;

/******************* Function Declarations ***********************************/

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] scp_service   handle to synch COM port service
 * @param[i] port_handle   handle to synch COM port
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc tcs3408r_get_who_am_i(sns_sync_com_port_service *scp_service,
                                    sns_sync_com_port_handle *port_handle,
                                    uint8_t *buffer);

/**
 * Reset TCS3408 sensor to default.
 *
 * @param[i] state         state of sensor instance
 * @param[i] sensor        bit mask for sensors to handle
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 * @return none
 */
sns_rc tcs3408r_reset_device(tcs3408_instance_state *state,
                                    tcs3408_sensor_type sensor);


/**
 * Modify TCS3408 ENABLE register.
 *
 * @param[i] state         state of sensor instance
 * @param[i] mask          mask of the bits which will be modified
 * @param[i] val           the value to be written into the mask bits
 *
 * @return none
 */
sns_rc tcs3408r_modify_control(tcs3408_instance_state *state,
                                     uint8_t mask,
                                     uint8_t val);
/**
 * Modify TCS3408 ENABLE register.
 *
 * @param[i] state         state of sensor instance
 * @param[i] mask          mask of the bits which will be modified
 * @param[i] val           the value to be written into the mask bits
 *
 * @return none
 */
sns_rc tcs3408r_modify_enable(tcs3408_instance_state *state,
                                     uint8_t mask,
                                     uint8_t val);

/**
 * Modify TCS3408 INTENABLE register.
 *
 * @param[i] state         state of sensor instance
 * @param[i] mask          mask of the bits which will be modified
 * @param[i] val           the value to be written into the mask bits
 *
 * @return none
 */
sns_rc tcs3408r_modify_intenab(tcs3408_instance_state *state,
                                      uint8_t mask,
                                      uint8_t val);

/**
 * TCS3408 clear fifo.
 *
 * @param[i] state         state of sensor instance
 *
 * @return none
 */
sns_rc tcs3408r_clear_fifo(tcs3408_instance_state *state);

/**
 * TCS3408 config start calibration.
 *
 * @param[i] state         state of sensor instance
 *
 * @return none
 */
sns_rc tcs3408r_start_calibration(tcs3408_instance_state *state);

/**
 * Get ALS interrupt persistence from register.
 *
 * @param[i] state         state of sensor instance
 * @param[i] persistence   store the interrupt persistence of ALS sensor
 *
 * @return none
 */
sns_rc tcs3408r_get_als_pers(tcs3408_instance_state *state, uint8_t *persistence);

/**
 * Write ALS interrupt persistence setting to register.
 *
 * @param[i] state         state of sensor instance
 * @param[i] persistence   interrupt persistence for ALS sensor
 *
 * @return none
 */
sns_rc tcs3408r_set_als_pers(tcs3408_instance_state *state, uint8_t persistence);




/**
 * Update ALS interrupt threshold registers.
 *
 * @param[i] state                 Instance state
 * @param[i] high_thresh           High threshold
 * @param[i] low_thresh            Low threshold
 *
 * @return none
 */
sns_rc tcs3408r_update_als_threshold(tcs3408_instance_state *state,
                                               uint16_t high_thresh, uint16_t low_thresh);


/**
 * Write ALS gain setting to register.
 *
 * @param[i] state                 Instance state
 * @param[i] gain                  Gain value
 *
 * @return none
 */
sns_rc tcs3408r_set_als_gain(tcs3408_instance_state *state, uint32_t gain);

/**
 * Write ALS time setting to register.
 *
 * @param[i] state                 Instance state
 * @param[i] gain                  Gain value
 *
 * @return none
 */
sns_rc tcs3408r_set_als_time(tcs3408_instance_state *state, uint32_t time_us);

/**
 * Procee sensor data.
 *
 * @param[i] instance   Sensor Instance
 */
void tcs3408r_process_sensor_data(sns_sensor_instance *const instance);

/**
 * Write Flicker gain setting to register.
 *
 * @param[i] state                 Instance state
 * @param[i] gain                  Gain value
 *
 * @return none
 */

//static int tcs3408_fd_enable(sns_sensor_instance *instance, int enable);
 
sns_rc tcs3408r_set_fd_gain(tcs3408_instance_state *state, uint16_t fd_gain);

/**
 * Write Flicker time setting to register.
 *
 * @param[i] state                 Instance state
 * @param[i] time                  Time value
 *
 * @return none
 */
sns_rc tcs3408r_set_fd_time_us(tcs3408_instance_state *state, uint32_t fd_time_us);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @param[i] instance    reference to this Instance
 */
void tcs3408r_send_config_event(sns_sensor_instance *const instance);

/**
 * Sends sensor ALS event.
 *
 * @param[i] instance   Sensor Instance
 * @param[i] timeout_time timestamp in ticks
 */
void tcs3408r_handle_sensor_als_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks);


/**
 * Sends sensor RGB event.
 *
 * @param[i] instance   Sensor Instance
 * @param[i] timeout_time timestamp in ticks
 */
void tcs3408r_handle_sensor_rgb_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks);

/**
 * Sends sensor flicker event.
 *
 * @param[i] instance   Sensor Instance
 * @param[i] timeout_time timestamp in ticks
 */
void tcs3408r_handle_sensor_flicker_sample(sns_sensor_instance *const instance, uint64_t timeout_ticks);

sns_rc tcs3408r_channel_cali(sns_sensor_instance *const instance);
#ifdef TCS3408R_GET_PARAMETER_FROM_SMEM
void tcs3408r_set_algo_para();
#endif
