#pragma once
/**
 * @file sns_ak0991x_lite.h
 *
 * compile switches for SEE-Lite.
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2019 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

/*--------------------------------------------------------------------------
 *
 * COMPILE CONDITION
 *
 *-------------------------------------------------------------------------*/
// Target board HDK845
//#define AK0991X_BOARD_HDK845

//#define AK0991X_ENABLE_TS_DEBUG           // Enable timestamp debug messages
//#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
#ifdef SNS_ENABLE_DAE
#define AK0991X_ENABLE_DAE                // Enable DAE
#endif
#ifndef SSC_TARGET_NO_I3C_SUPPORT
#define AK0991X_ENABLE_I3C_SUPPORT        // Enable support for I3C bus
#endif
//#define AK0991X_ENABLE_I3C_DEBUG
//#define AK0991X_ENABLE_DUAL_SENSOR        // Enable to set dual sensor support mode
//#define AK0991X_ENABLE_DEVICE_MODE_SENSOR // Enable devise_mode_sensor
#define AK0991X_ENABLE_TIMER_FILTER       // Enable polling timer filter
//#define AK0991X_PATCH_FOR_DAE_S4S_DT_EVENT_FIELDS_ON_704

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
#define MAX_DEVICE_MODE_SUPPORTED 2       // change number 2/4/8 in order to match the faccal num in the registry
#else
#define MAX_DEVICE_MODE_SUPPORTED 1
#endif

#define AK0991X_UNKNOWN_DEVICE_MODE ((uint32_t)-1) //Unknown device mode used for transitional state

#if defined(AK0991X_ENABLE_DAE) && defined(AK0991X_ENABLE_DEVICE_MODE_SENSOR) && defined(AK0991X_ENABLE_REGISTRY_ACCESS)
#define SUID_NUM 6
#elif defined(AK0991X_ENABLE_DAE) && defined(AK0991X_ENABLE_DEVICE_MODE_SENSOR)
#define SUID_NUM 5
#elif defined(AK0991X_ENABLE_DAE) && defined(AK0991X_ENABLE_REGISTRY_ACCESS)
#define SUID_NUM 5
#elif defined(AK0991X_ENABLE_DEVICE_MODE_SENSOR) && defined(AK0991X_ENABLE_REGISTRY_ACCESS)
#define SUID_NUM 5
#elif defined(AK0991X_ENABLE_DAE)
#define SUID_NUM 4
#elif defined(AK0991X_ENABLE_DEVICE_MODE_SENSOR)
#define SUID_NUM 4
#elif defined(AK0991X_ENABLE_REGISTRY_ACCESS)
#define SUID_NUM 4
#else
#define SUID_NUM 3
#endif
