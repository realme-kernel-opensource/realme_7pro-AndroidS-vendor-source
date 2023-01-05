/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#ifndef INCLUDE_AKM_CUSTOMERSPEC_H
#define INCLUDE_AKM_CUSTOMERSPEC_H

#define AKM_CUSTOM_FOOTPRINT "20171221"

/*****Choose a function*****/
//#define AKM_ENABLE_PDC
//#define AKM_ENABLE_DATA_SORT

/*****Choose a D9D version*****/
//#define AKM_USE_D9DV2_ALPHA

#include "sns_mem_util.h"
#include "../../../inc/internal/sns_memmgr.h"
#define AKM_MALLOC(sz)  sns_malloc(SNS_HEAP_MAIN, sz)
#define AKM_FREE(sz)  sns_free(sz)
#define AKM_MEMSET(block,val,size)  sns_memset(block,val,size)

#ifndef UNUSED_VAR
#define UNUSED_VAR(var) ((void)(var));
#endif


/*! \defgroup CUSTOMER_SPEC Customer specific parameters.
 * SmartCompass library parameters. <b> Please change these parameters
 * according to the directions from AKM.</b>
 @{*/
/*! A string of licenser. Don't change this string. */
#define AKM_CUSTOM_LICENSER  "ASAHIKASEI"
/*! A string of licensee. This string should be changed. */
#define AKM_CUSTOM_LICENSEE	"ONEP_X_SEE"


/*! The number of formation. */
#define AKM_CUSTOM_NUM_FORM  1


/*! \defgroup CSPEC_AXIS The axis conversion
 * Axis conversion parameters.
 */
#define AKM_CUSTOM_MAG_AXIS_ORDER_X  0
#define AKM_CUSTOM_MAG_AXIS_ORDER_Y  1
#define AKM_CUSTOM_MAG_AXIS_ORDER_Z  2
#define AKM_CUSTOM_MAG_AXIS_SIGN_X   0
#define AKM_CUSTOM_MAG_AXIS_SIGN_Y   0
#define AKM_CUSTOM_MAG_AXIS_SIGN_Z   0

#define AKM_CUSTOM_ACC_AXIS_ORDER_X  0
#define AKM_CUSTOM_ACC_AXIS_ORDER_Y  1
#define AKM_CUSTOM_ACC_AXIS_ORDER_Z  2
#define AKM_CUSTOM_ACC_AXIS_SIGN_X   0
#define AKM_CUSTOM_ACC_AXIS_SIGN_Y   0
#define AKM_CUSTOM_ACC_AXIS_SIGN_Z   0

#define AKM_CUSTOM_GYR_AXIS_ORDER_X  0
#define AKM_CUSTOM_GYR_AXIS_ORDER_Y  1
#define AKM_CUSTOM_GYR_AXIS_ORDER_Z  2
#define AKM_CUSTOM_GYR_AXIS_SIGN_X   0
#define AKM_CUSTOM_GYR_AXIS_SIGN_Y   0
#define AKM_CUSTOM_GYR_AXIS_SIGN_Z   0

#endif /* INCLUDE_AKM_CUSTOMERSPEC_H */
