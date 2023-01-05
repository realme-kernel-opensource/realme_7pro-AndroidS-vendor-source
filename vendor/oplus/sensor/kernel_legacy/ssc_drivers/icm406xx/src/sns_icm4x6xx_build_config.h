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
*/

/* Enable or disable all debug MSG */
#define ICM4X6XX_ENABLE_DEBUG_MSG

/* Enable or disable verbose MSG */
//#define ICM4X6XX_DEBUG_VERBOSE

/* Enable or disable dump FIFO */
//#define ICM4X6XX_DEBUG_DUMP_FIFO

/* Enable logs of registers read/write operations */
//#define ICM4X6XX_DEBUG_REG_OPERATION

// time limiT WA for A/G-044/045, TOTO: remove this and CHOOSE BETTER sotlution
#define ICM4X6XX_ODR_CHANGING_SAMPLE_MISSING_SAMPLE_GAP

#define ICM4X6XX_TEMPERATURE_ONE_UNRELIABLE_SAMPLE

/* Enable or disable DAE automatically */
#ifndef SNS_ENABLE_DAE
#define ICM4X6XX_DISABLE_DAE
#endif

#ifndef ICM4X6XX_DISABLE_DAE
#include "sns_dae.pb.h"
/* workaround for DAE not response 604 issue, TODO: remove after issue fixed */
//#define ICM4X6XX_DAE_MISSING_604_WA
#endif

#define ICM4X6XX_DETECT_DAE_SENSOR

//define to USE I3C, otherwise will use SPI/I2C
//#define ICM4X6XX_ENABLE_I3C
#ifdef ICM4X6XX_ENABLE_I3C
/* Enable or disable I3C debug MSG */
//#define ICM4X6XX_ENABLE_DEBUG_I3C

/* Choose IBI or INT1 PIN for I3C */
#define ICM4X6XX_I3C_USE_INT1      (1)
#endif

//#define ICM4X6XX_PASSIVE_SENSOR_SUPPORT

/* Yoko C1 has built-in freefall(replace highg) and not support highg */
//#define ICM4X6XX_YOKOHAMA_C1
//#define ICM4X6XX_ENABLE_LOWG
#ifdef ICM4X6XX_YOKOHAMA_C1
/* defined when minimum DMP ODR is 500Hz*/
#define ICM4X6XX_YOKO_C1_DMP_MIN_500HZ
#else
//#define ICM4X6XX_ENABLE_HIGHG
#endif
#ifdef ICM4X6XX_DISABLE_DAE
#ifndef ICM4X6XX_YOKOHAMA_C1
/* SM855/865 has enough resource, will use platform pedometer,
   for different platform, maybe will use HW pedometer */
//#define ICM4X6XX_ENABLE_PEDOMETER
//#define ICM4X6XX_ENABLE_PEDOMETER_DEBUG_LOG
#endif
#endif

/* DMP ODR will be 100Hz when marcro defined, otherwise 50Hz */
//#define ICM4X6XX_HIGHG_LOWG_HIGH_PERFORMANCE

// must be open
#define ICM4X6XX_W_BIAS_TO_REG_IMMEDIATELY

/* Uncommnet to enable RTC MODE (Only for ICM42622/42688) */
//#define ICM4X6XX_ENABLE_RTC_MODE

#ifdef ICM4X6XX_ENABLE_RTC_MODE
/* RTC(CLKIN) frequency in Hz (Only for ICM42622/42688) */
#define ICM4X6XX_RTC_FREQ_HZ  32768
#define ICM4X6XX_ODR_ADJUST_RATE ((float)(ICM4X6XX_RTC_FREQ_HZ/32000.f))
#endif


//#define ICM4X6XX_VERIFY_SRAM_FW_DL
//#define ICM4X6XX_ONESHOT_SIMPLE_RW_DL_SRAM_FW
//#define ICM4X6XX_ONESHOT_ONE_VEC_DL_SRAM_FW
#define ICM4X6XX_SEVERAL_TIMES_WRITE_DL_STACK_MEM
//#define ICM4X6XX_SEVERAL_TIMES_WRITE_DL_STATIC_MEM
//#define ICM4X6XX_ONESHOT_MULTI_VEC_DL_SRAM_FW

/* Choose TS cal method */
#define ICM4X6XX_USE_INT_TS

/* read out all interrupt events to fix */
//#define INT_TS_SAME_WORKAROUND

#define ICM4X6XX_READ_TMST_IN_INT

/* only used for Havana, which does not have 500hz ODR*/
//#define ICM4X6XX_1K_2_500HZ

/* IKR: Havana A_LPM = 25Hz/50Hz ODR && only 1x averaging filter */
//#define ICM406XX_ENABLE_A_LPM

/*
* [HAVANA/YOKOHAMA] Disable aux pads(pin10&pin11) which are typically connected to OIS controller if applicable.
* 1. If the pin10 and pin11 are floating in customer design, need define below macro to disable aux pads to avoid current leak.
* 2. If the pin10 and pin11 are used/connected in customer design(e.g. OIS), need comment below macro to enable aux pads.
* [YOKO_C1] the aux pads were trimmed to enabled/disable aux, but need to comment below macro to config pads to avoid current leak.
* 3. [ICM42631][triple interface mode] pin 2/3/10/7/11 set to pull-up and pin9(AUX2) pull-down
* 4. [ICM42631][single/dual interface mode] pin 7/9/9(AUX2) must be set to 0.

*/
#define ICM4X6XX_DISABLE_AUX_PADS

/* Periodic reset mode "should be" better than Continuous mode */
#define ICM4X6XX_GYRO_STALL_WORKAROUND
/* workaround for gyro ring down issue */
#define ICM4X6XX_GYRO_RING_DOWN_WA

/*
* Use INT2 for all interrupts to replace INT1.
* If pin9 is used for RTC input, disable INT2.
*/
#ifndef ICM4X6XX_ENABLE_RTC_MODE
//#define ICM4X6XX_USE_INT2
#endif

#ifdef ICM4X6XX_ENABLE_DUAL_SENSOR
#define SENSOR_INST_CNT 2
#else
#define SENSOR_INST_CNT 1
#endif

//#define ICM4X6XX_REMOVE_FILTER_DELAY

#ifdef ICM4X6XX_ENABLE_DEBUG_MSG
#define ICM4X6XX_PRINTF(prio, sensor, ...) SNS_PRINTF(prio, sensor, "[INVN]"__VA_ARGS__);
#define ICM4X6XX_INST_PRINTF(prio, inst, ...) SNS_INST_PRINTF(prio, inst, "[INVN]"__VA_ARGS__);
#else  //ICM4X6XX_ENABLE_DEBUG_MSG
#define ICM4X6XX_PRINTF(prio, sensor,...)   UNUSED_VAR(sensor)
#define ICM4X6XX_INST_PRINTF(prio, inst,...) UNUSED_VAR(inst)
#endif  //ICM4X6XX_ENABLE_DEBUG_MSG

#ifdef ICM4X6XX_DEBUG_VERBOSE
#define ICM4X6XX_VERB_PRINTF(prio, sensor, ...) SNS_PRINTF(prio, sensor, "[INVN]"__VA_ARGS__);
#define ICM4X6XX_VERB_INST_PRINTF(prio, inst, ...) SNS_INST_PRINTF(prio, inst, "[INVN]"__VA_ARGS__);
#else  //ICM4X6XX_DEBUG_VERBOSE
#define ICM4X6XX_VERB_PRINTF(prio, sensor,...)   UNUSED_VAR(sensor)
#define ICM4X6XX_VERB_INST_PRINTF(prio, inst,...) UNUSED_VAR(inst)
#endif  //ICM4X6XX_DEBUG_VERBOSE

