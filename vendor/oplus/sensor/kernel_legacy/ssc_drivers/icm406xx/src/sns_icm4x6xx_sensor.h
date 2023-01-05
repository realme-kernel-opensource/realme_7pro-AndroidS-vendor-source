#pragma once
/*
*
* Copyright @ 2014-2020 InvenSense Inc.  All rights reserved.
*
* INVENSENSE, INC. ("LICENSOR") SOFTWARE LICENSE AGREEMENT ("Agreement")
* BY DOWNLOADING, INSTALLING, COPYING OR OTHERWISE USING THE ENCLOSED SOFTWARE AND OTHER AVAILABLE MATERIALS ("LICENSED
* MATERIALS"), YOU ("LICENSEE") AGREE TO BE BOUND BY THE FOLLOWING TERMS AND CONDITIONS OF THIS AGREEMENT.  IF LICENSEE DOES NOT
* AGREE TO THE TERMS AND CONDITION OF THIS AGREEMENT, THEN DO NOT DOWNLOAD, INSTALL, COPY OR OTHERWISE USE THE ENCLOSED SOFTWARE
* AND MATERIALS.
*
* The Licensed Materials may include open source and proprietary software in both source code ("Source Code") and object code
* ("Object Code") versions, documentation and other materials.  Except as expressly indicated herein, all terms and conditions of
* this Agreement apply to all of the Licensed Materials.  In the event that any file contained within the Licensed Materials
* incorporates a different software license, such other license shall apply to such file.
* Except as expressly set forth herein, LICENSOR owns all of the Licensed Materials and makes them available to Licensee only
* under the terms and conditions set forth in this Agreement.
*
* 1. License:  Subject to the terms of this Agreement, LICENSOR hereby grants to Licensee a royalty-free, non-exclusive license to
* possess and to use the Licensed Materials in a secure location in accordance with the applicable terms herein.  Licensee may
* make back-up copies and install and use multiple copies of the Licensed Materials on a shared computer or concurrently on
* different computers, solely for Licensee's use within Licensee's Enterprise. "Enterprise" shall mean individual use by Licensee
* or any legal entity (such as a corporation or university) and the subsidiaries it owns by more than 50 percent.  The following
* terms apply to the specified type of Licensed Material:
*
* 1.1 Source Code:  Licensee shall have the right to use, copy, modify and prepare and have prepared derivative works of the
* Source Code for internal development and testing purposes only.  Licensee shall own any modification it makes directly to the
* Source Code that optimizes quality or performance with Licensee's product ("Modification(s)") subject to LICENSOR's ownership of
* the underlying Source Code and all intellectual property rights therein.
*
* 1.2 Object Code:  Licensee may distribute the Object Code (in object code form only) and compiled Source Code (in object code
* form only) with Modifications solely for execution or operation with applicable LICENSOR products for which Source Code and
* Object Code was designed and as incorporated in Licensee's software products. Licensee agrees not to disassemble, decompile or
* reverse engineer the Object Code.
*
* 2. Notices:  Licensee agrees to comply with and reproduce any and all copyright and other attribution notices/instructions of
* LICENSOR as included in the Licensed Materials in connection with Licensee's distribution rights pursuant to the Agreement.
*
* 3. Subcontractors:  Licensee may engage subcontractors to exercise Licensee's rights hereunder. Licensee is responsible to
* ensure that Licensee subcontractors comply with the terms and conditions of this Agreement.  Any act by a subcontractor that
* would be a breach of this Agreement by Licensee if Licensee performed the act will be deemed a breach of this Agreement by
* Licensee.
*
* 4. License Grant Back: Licensee hereby grants to LICENSOR and its Affiliates an exclusive, worldwide, irrevocable, perpetual,
* sublicenseable (through multiple tiers of sublicensees), royalty-free and fully paid-up right and license to the Modification(s)
* (in object code form) created by or on behalf of Licensee so that LICENSOR may copy, modify, create derivatives works thereof,
* to use, have used, import, make, have made, sell, offer to sell, sublicense (through multiple tiers of sublicensees), distribute
* (through multiple tiers of distributors) such derivative work(s) on a stand-alone basis or as incorporated into the Licensed
* Materials or other related technologies.  For the sake of clarity, LICENSOR is not prohibited or otherwise restricted from
* independently developing new features or functionality with respect to the Licensed Materials.
*
* 5. No Other License: No rights or licenses with respect to any proprietary information or patent, copyright, trade secret or
* other intellectual property right owned or controlled by LICENSOR are granted by LICENSOR to Licensee under this Agreement,
* expressly or by implication, except as expressly provided in this Agreement.
* 6. Intellectual Property Ownership: Except as expressly licensed to Licensee under this Agreement, LICENSOR reserves all right,
* title and interest, including but not limited to all intellectual property rights, in and to the Licensed Materials and any
* derivative work(s) made thereto.
*
* 7. Term of Agreement:  This Agreement is effective until (i) automatically terminated if Licensee fails to comply with any of
* the terms and conditions of this Agreement; or (ii) terminated by LICENSOR.  LICENSOR may terminate this Agreement (and with it,
* all of Licensee's right to the Licensed Materials) immediately upon written notice (which may include email) to Licensee, with
* or without cause; provided however, that sublicenses of Derivatives, to the extent validly granted to its customers prior to
* termination of this Agreement, shall survive such termination.
*
* 8. Confidential Information. "Confidential Information" means (i) the Licensed Materials; (ii) the structure, sequence and
* organization of the Licensed Materials and the concepts, methods of operations and ideas disclosed therein; and (iii) any trade
* secrets of LICENSOR or its affiliates or its or their suppliers relating to the Licensed Materials. Licensee will not disclose
* any Confidential Information to any third party (except subcontractors, as permitted herein) or use Confidential Information
* except as expressly permitted in this Agreement.  Licensee agrees to take all reasonable measures to protect Confidential
* Information and prevent its unauthorized disclosure, including measures at least as stringent as those measures Licensee takes
* to protect Licensee's own most sensitive confidential information.  Licensee agrees to restrict access to Confidential
* Information to Licensee employees and subcontractors who are under obligations to protect Confidential Information in accordance
* with this Agreement and who have a "need to know" the Confidential Information to exercise Licensee license rights in this
* Agreement.  All Confidential Information, and any documents and other tangible objects containing or representing Confidential
* Information, and all copies of Confidential Information, are and will remain the exclusive property of LICENSOR.
*
* 9. Defensive Suspension: If Licensee commences or participates in any legal proceeding against LICENSOR, then LICENSOR may, in
* its sole discretion, suspend or terminate all license grants and any other rights provided under this Agreement during the
* pendency of such legal proceedings.
*
* 10. No Support:  LICENSOR has no obligation to support or to continue providing or updating any of the Licensed Materials.
*
* 11. No Warranty:  THE LICENSED MATERIALS PROVIDED BY LICENSOR TO LICENSEE HEREUNDER ARE PROVIDED "AS IS."  LICENSOR DISCLAIMS
* ALL WARRANTIES, EXPRESS, IMPLIED OR STATUTORY, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*
* 12. Limitation of Liability: LICENSOR SHALL NOT BE LIABLE TO LICENSEE, LICENSEE'S CUSTOMERS, OR ANY OTHER PERSON OR ENTITY
* CLAIMING THROUGH OR UNDER LICENSEE FOR ANY LOSS OF PROFITS, INCOME, SAVINGS, OR ANY OTHER CONSEQUENTIAL, INCIDENTAL, SPECIAL,
* PUNITIVE, DIRECT OR INDIRECT DAMAGES (WHETHER IN AN ACTION IN CONTRACT, TORT OR BASED ON A WARRANTY), EVEN IF LICENSOR HAS BEEN
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF THE ESSENTIAL PURPOSE
* OF ANY LIMITED REMEDY.  IN NO EVENT SHALL LICENSOR'S AGGREGATE LIABILITY TO LICENSEE OR ANY OTHER PERSON OR ENTITY CLAIMING
* THROUGH OR UNDER LICENSEE EXCEED THE AMOUNT OF MONEY ACTUALLY PAID BY LICENSEE TO LICENSOR FOR THE LICENSED MATERIALS.
*
* 13. Applicable Law and Jurisdiction: This Agreement shall be deemed to have been made in, and shall be construed pursuant to,
* the laws of the State of California. The state and/or federal courts residing in Santa Clara County, California shall have
* exclusive jurisdiction over any dispute or claim arising out of this Agreement. The United Nations Convention on Contracts for
* the International Sale of Goods is specifically disclaimed.
*
* 14. Feedback: Licensee may, but is not obligated to, provide to LICENSOR any suggestions, comments and feedback regarding the
* Licensed Materials that are delivered by LICENSOR to Licensee under this Agreement (collectively, "Licensee Feedback").
* LICENSOR may use and include any Licensee Feedback that Licensee voluntarily provides to improve the Licensed Materials or other
* related LICENSOR technologies.  Accordingly, if Licensee provides Licensee Feedback, Licensee grants LICENSOR and its licensees
* a perpetual, irrevocable, worldwide, royalty-free, fully paid-up license grant to freely use, have used, sell, modify,
* reproduce, transmit, license, sublicense (through multiple tiers of sublicensees), distribute (through multiple tiers of
* distributors), and otherwise commercialize the Licensee Feedback in the Licensed Materials or other related technologies.
*
* 15. RESTRICTED RIGHTS NOTICE: Licensed Materials has been developed entirely at private expense and is commercial computer
* software provided with RESTRICTED RIGHTS. Use, duplication or disclosure by the U.S. Government or a U.S. Government
* subcontractor is subject to the restrictions set forth in the license agreement under which Licensed Materials was obtained
* pursuant to DFARS 227.7202-3(a) or as set forth in subparagraphs (c)(1) and (2) of the Commercial Computer Software - Restricted
* Rights clause at FAR 52.227-19, as applicable.
*
* 16. Miscellaneous: If any provision of this Agreement is inconsistent with, or cannot be fully enforced under, the law, such
* provision will be construed as limited to the extent necessary to be consistent with and fully enforceable under the law. This
* Agreement is the final, complete and exclusive agreement between the parties relating to the subject matter hereof, and
* supersedes all prior or contemporaneous understandings and agreements relating to such subject matter, whether oral or written.
* This Agreement is solely between LICENSOR and Licensee.  There are no third party beneficiaries, express or implied, to this
* Agreement. This Agreement may only be modified in writing signed by an authorized officer of LICENSOR.  Licensee agrees that it
* will not ship, transfer or export the Licensed Materials into any country, or use the Licensed Materials in any manner,
* prohibited by the United States Bureau of Industry and Security or any export laws, restrictions or regulations. This Agreement,
* and Licensee's rights and obligations herein, may not be assigned, subcontracted, delegated, or otherwise transferred by
* Licensee without LICENSOR's prior written consent, and any attempted assignment, subcontract, delegation, or transfer in
* violation of the foregoing will be null and void.   The terms of this Agreement shall be binding upon assignees.
*
*
*/

#include "sns_data_stream.h"
#include "sns_icm4x6xx_hal.h"
#include "sns_island_service.h"
#include "sns_pwr_rail_service.h"
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_suid_util.h"

#include "sns_diag_service.h"
#include "sns_icm4x6xx_sensor_instance.h"
#include "sns_math_util.h"

#include "sns_registry_util.h"
#include "sns_icm4x6xx_ver.h"

/**
 * Code review comment from Qualcomm: UUID's shouldn't be similar to each other
 * you can use Online UUID generator like https://uuidgenerator.net 
 **/
 

#define ICM_ACCEL_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x16, 0xd3, 0xac, 0x47, 0xde, 0x2e, 0x44, 0x58,  \
        0xaa, 0xc9, 0xa5, 0xb4, 0x76, 0x53, 0xab, 0x61  \
      }  \
  }

#define ICM_GYRO_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x0f, 0xa3, 0x60, 0x17, 0x50, 0xde, 0x4a, 0x45,  \
        0xa1, 0x86, 0xfc, 0x3e, 0x42, 0xec, 0x9a, 0x14  \
      }  \
  }

#define ICM_MOTION_DETECT_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x52, 0x02, 0x99, 0xf9, 0x00, 0xb7, 0x45, 0x87,  \
        0xbb, 0xd9, 0xd2, 0xcc, 0xd6, 0xbc, 0xda, 0x82  \
      }  \
  }

#define ICM_SENSOR_TEMPERATURE_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x95, 0x77, 0xc4, 0x8c, 0x69, 0x5f, 0x40, 0x59,  \
        0x9f, 0x7d, 0xd0, 0x9d, 0x8e, 0x70, 0xd1, 0xb6   \
      }  \
  }

#ifdef ICM4X6XX_ENABLE_LOWG
#define ICM_FREE_FALL_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0xa7, 0xd4, 0x96, 0xde, 0x4f, 0x04, 0x4a, 0x1b,  \
        0x82, 0x44, 0x3d, 0x57, 0x3b, 0xa1, 0x5b, 0x09  \
      }  \
  }
#endif

#ifdef ICM4X6XX_ENABLE_HIGHG
#define ICM_HIGH_SHOCK_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x20, 0xe9, 0x22, 0x4f, 0x47, 0xbd, 0x44, 0x65,  \
        0xb0, 0x57, 0x0b, 0x52, 0x96, 0xf1, 0xa9, 0x1c  \
      }  \
  }
#endif

#ifdef ICM4X6XX_ENABLE_PEDOMETER
#define ICM_STEP_COUNTER_SUID \
  {  \
    .sensor_uid =  \
      {  \
        0x5e, 0xc2, 0x0f, 0xe4, 0x07, 0xb6, 0x45, 0x84,  \
        0xb5, 0x20, 0x30, 0xb5, 0xb1, 0x2d, 0xb4, 0x9c  \
      }  \
  }
#endif /* ICM4X6XX_ENABLE_PEDOMETER */

#ifdef ICM4X6XX_ENABLE_DUAL_SENSOR
#define ICM_ACCEL_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x38, 0x15, 0xcf, 0x16, 0x1c, 0xcf, 0x44, 0xc7,  \
        0xae, 0xae, 0x38, 0xfa, 0xe1, 0x50, 0x2f, 0x16  \
      }  \
  }

#define ICM_GYRO_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0xf8, 0x94, 0xe1, 0xd4, 0x90, 0x1b, 0x4a, 0x9d,  \
        0x92, 0x3f, 0xf0, 0x20, 0xb2, 0x3a, 0x3f, 0x2a  \
      }  \
  }

#define ICM_MOTION_DETECT_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x8d, 0xaf, 0xed, 0x6c, 0x47, 0xe7, 0x4e, 0x46,  \
        0xa3, 0xf2, 0xb9, 0xdc, 0x22, 0x72, 0x9f, 0x0d  \
      }  \
  }

#define ICM_SENSOR_TEMPERATURE_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x28, 0x26, 0xf2, 0x7d, 0x7c, 0x14, 0x43, 0x62, \
        0x83, 0xa7, 0xe6, 0x7a, 0x4e, 0x52, 0x74, 0x9f  \
      }  \
  }

#ifdef ICM4X6XX_ENABLE_LOWG
#define ICM_FREE_FALL_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0x91, 0xc0, 0x9d, 0xa3, 0xd4, 0x0c, 0x4a, 0x67,  \
        0x95, 0xc9, 0x17, 0xd2, 0xca, 0x6d, 0x10, 0xbe  \
      }  \
  }
#endif

#ifdef ICM4X6XX_ENABLE_HIGHG
#define ICM_HIGH_SHOCK_SUID_1 \
  {  \
    .sensor_uid =  \
      {  \
        0xc2, 0x96, 0x99, 0x2a, 0x35, 0x1d, 0x4c, 0xff,  \
        0xa0, 0x31, 0x62, 0x4e, 0x59, 0x57, 0x31, 0xa7  \
      }  \
  }
#endif
#endif

#define ICM4X6XX_HAVANA_MAX_FIFO        128
#define ICM4X6XX_YOKOHAMA_MAX_FIFO      256

#ifdef ICM4X6XX_USE_INT_TS
#define FIFO_USE_PROPORTION             (0.78125f)  //150(QC's Request) < 200 < 256
#define FIFO_HAVANA_PROPORTION          (0.9f)
#else
/* Always 16 bytes, max fifo size is 64 sample,
 * we add some margin to protect overflow */
#define FIFO_USE_PROPORTION             (0.40625f)
#define FIFO_HAVANA_PROPORTION          (0.40625f)
#endif

#define ICM4X6XX_OFF_TO_IDLE_MS         100

/** Forward Declaration of Accel Sensor API */
sns_sensor_api icm4x6xx_accel_sensor_api;

/** Forward Declaration of Gyro Sensor API */
sns_sensor_api icm4x6xx_gyro_sensor_api;

/** Forward Declaration of Gyro Sensor API */
sns_sensor_api icm4x6xx_motion_detect_sensor_api;

#ifdef ICM4X6XX_ENABLE_LOWG
sns_sensor_api icm4x6xx_free_fall_sensor_api;
#endif

#ifdef ICM4X6XX_ENABLE_HIGHG
sns_sensor_api icm4x6xx_high_shock_sensor_api;
#endif

#ifdef ICM4X6XX_ENABLE_PEDOMETER
sns_sensor_api icm4x6xx_step_counter_sensor_api;
#endif

sns_sensor_api icm4x6xx_sensor_temp_sensor_api;

/**
 * ICM4X6XX Sensor Temperature ODR (Hz) definitions
 */
#define ICM4X6XX_SENSOR_TEMP_ODR_1        1.0f
#define ICM4X6XX_SENSOR_TEMP_ODR_5        5.0f

#ifdef ICM4X6XX_ENABLE_PEDOMETER
/**
 * ICM4X6XX Step Counter ODR (Hz) definitions
 */
#define ICM4X6XX_STEP_COUNTER_ODR_1       1.0f
#define ICM4X6XX_STEP_COUNTER_ODR_5       5.0f
#endif /* ICM4X6XX_ENABLE_PEDOMETER */

/**
 * Sensor Temperature range in deg Celsius
 */
#define ICM4X6XX_SENSOR_TEMPERATURE_RANGE_MIN  (-40.0f)
#define ICM4X6XX_SENSOR_TEMPERATURE_RANGE_MAX  (85.0f)

#define ICM4X6XX_SENSOR_TEMPERATURE_RESOLUTION (1/132.48f) // From the new DS, updated from email
#define ICM4X6XX_SENSOR_TEMPERATURE_RES_HAVANA (1/115.49f) // From DS


/** Power rail timeout States for the icm4x6xx Sensors.*/
#define ICM4X6XX_POWER_RAIL_OFF_TIMEOUT_NS 1000000000ULL /* 1 second */
typedef enum
{
  ICM4X6XX_POWER_RAIL_PENDING_NONE,
  ICM4X6XX_POWER_RAIL_PENDING_INIT,
  ICM4X6XX_POWER_RAIL_PENDING_SET_CLIENT_REQ,
  ICM4X6XX_POWER_RAIL_PENDING_OFF,
} icm4x6xx_power_rail_pending_state;

typedef struct _calibrationData {
    int16_t x;
    int16_t y;
    int16_t z;
} icm4x6xx_calibrationData_t;

/** Interrupt Sensor State. */

typedef struct icm4x6xx_common_state
{
  uint8_t                 a_res_idx;
  uint8_t                 g_res_idx;

  bool                    hw_is_present;
  bool                    registry_cfg_received;
  bool                    registry_pf_cfg_received;
  bool                    registry_orient_received;

  // placement
  bool                    registry_placement_received;
  float                   placement[12];

  sns_sensor_uid          a_suid;
  sns_sensor_uid          g_suid;
  sns_sensor_uid          md_suid;
  sns_sensor_uid          st_suid;

#ifdef ICM4X6XX_ENABLE_LOWG
  sns_sensor_uid          free_fall_suid;
#endif
#ifdef ICM4X6XX_ENABLE_HIGHG
  sns_sensor_uid          high_shock_suid;
#endif
#ifdef ICM4X6XX_ENABLE_PEDOMETER
  sns_sensor_uid          step_counter_suid;
#endif

  // Registry, IRQ, Timer, ASCP, DAE
  SNS_SUID_LOOKUP_DATA(5) suid_lookup_data;

  icm4x6xx_com_port_info  com_port_info;
  sns_interrupt_req       irq_config;

  sns_rail_config         rail_config;
  sns_power_rail_state    registry_rail_on_state;
  icm4x6xx_product_info   product_info;

  sns_registry_phy_sensor_cfg registry_cfg;

  sns_registry_phy_sensor_pf_cfg registry_pf_cfg;

  triaxis_conversion axis_map[TRIAXIS_NUM];

#ifndef ICM4X6XX_DISABLE_DAE
  /**--------DAE interface---------*/
  icm4x6xx_dae_if_info       dae_if;
#endif

} icm4x6xx_common_state;

typedef struct icm4x6xx_state
{
  icm4x6xx_sensor_type     sensor;

  icm4x6xx_power_rail_pending_state    power_rail_pend_state;

  sns_data_stream         *reg_data_stream;
  sns_data_stream         *fw_stream;
  sns_data_stream         *timer_stream;
  sns_pwr_rail_service    *pwr_rail_service;
  sns_diag_service        *diag_service;
  sns_island_service      *island_service;
  sns_sync_com_port_service *scp_service;

  bool                    sensor_client_present;
  bool                    new_stream;
  bool                    is_dri;
  bool                    supports_sync_stream;
  bool                    registry_md_config_received;
  bool                    registry_fac_cal_received;
  float                   fac_cal_bias[TRIAXIS_NUM];
  float                   fac_cal_scale[TRIAXIS_NUM];
  uint32_t                fac_cal_version;

  uint32_t                resolution_idx;
#ifdef ICM4X6XX_ENABLE_DUAL_SENSOR
  uint32_t                registration_idx;
#endif

  int64_t                 hardware_id;

  size_t                  encoded_event_len;

  sns_registry_md_cfg     md_config;

  matrix3                 fac_cal_corr_mat;

  sns_sensor_uid          my_suid;

  icm4x6xx_common_state   common;

} icm4x6xx_state;

/** Functions shared by all icm4x6xx Sensors */
/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 * @param[i] sensor_type   Sensor type
 *
 * @return none
 */
void icm4x6xx_reval_instance_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   icm4x6xx_sensor_type sensor_type);

/**
 * notify_event() Sensor API common between all icm4x6xx Sensors.
 *
 * @param this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc icm4x6xx_sensor_notify_event(sns_sensor *const this);

/**
 * set_client_request() Sensor API common between all icm4x6xx
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
struct sns_sensor_instance* icm4x6xx_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove);
sns_rc icm4x6xx_accel_init(sns_sensor *const this);
sns_rc icm4x6xx_gyro_init(sns_sensor *const this);
sns_rc icm4x6xx_motion_detect_init(sns_sensor *const this);
sns_rc icm4x6xx_sensor_temp_init(sns_sensor *const this);
#ifdef ICM4X6XX_ENABLE_LOWG
sns_rc icm4x6xx_free_fall_init(sns_sensor *const this);
sns_rc icm4x6xx_free_fall_deinit(sns_sensor *const this);
#endif
#ifdef ICM4X6XX_ENABLE_HIGHG
sns_rc icm4x6xx_high_shock_init(sns_sensor *const this);
sns_rc icm4x6xx_high_shock_deinit(sns_sensor *const this);
#endif
#ifdef ICM4X6XX_ENABLE_PEDOMETER
sns_rc icm4x6xx_step_counter_init(sns_sensor *const this);
sns_rc icm4x6xx_step_counter_deinit(sns_sensor *const this);
#endif
sns_rc icm4x6xx_accel_deinit(sns_sensor *const this);
sns_rc icm4x6xx_gyro_deinit(sns_sensor *const this);
sns_rc icm4x6xx_motion_detect_deinit(sns_sensor *const this);
sns_rc icm4x6xx_sensor_temp_deinit(sns_sensor *const this);
void icm4x6xx_sensor_process_registry_event(sns_sensor *const this,
                                           sns_sensor_event *event);
void icm4x6xx_request_registry(sns_sensor *const this);
void icm4x6xx_update_registry(sns_sensor *const this,
        sns_sensor_instance *const instance, icm4x6xx_sensor_type sensor);
void icm4x6xx_update_sensor_state(sns_sensor *const this,
        sns_sensor_instance *const instance);
bool icm4x6xx_discover_hw(sns_sensor *const this);
void icm4x6xx_publish_ag_commom_attrs(sns_sensor *const this);
void icm4x6xx_publish_available(sns_sensor *const this);
void icm4x6xx_update_sibling_sensors(sns_sensor *const this);
void icm4x6xx_start_hw_detect_sequence(sns_sensor *const this);
void icm4x6xx_start_power_rail_timer(sns_sensor *const this,
                    sns_time timeout_ticks,
                    icm4x6xx_power_rail_pending_state pwr_rail_pend_state);
void icm4x6xx_common_init(sns_sensor *const this);
