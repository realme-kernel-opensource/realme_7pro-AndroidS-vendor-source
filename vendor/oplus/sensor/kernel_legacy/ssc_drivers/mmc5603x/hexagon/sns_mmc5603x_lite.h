#pragma once
/**
 * @file sns_mmc5603x_lite.h
 *
 * compile switches for SEE-Lite.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

/*--------------------------------------------------------------------------
 *
 * COMPILE CONDITION
 *
 *-------------------------------------------------------------------------*/

// Standard SEE Mode. Enabled all features.
#define MMC5603X_ENABLE_REGISTRY_ACCESS    // Enable registry access
#define MMC5603X_ENABLE_ALL_ATTRIBUTES     // Enable all attribute service
#define MMC5603X_ENABLE_DEBUG_MSG          // Enable debug messages
//#define MMC5603X_ENABLE_DAE                // Enable DAE
#define MMC5603X_ENABLE_DIAG_LOGGING       // Enable diagnostic logging
#define MMC5603X_ENABLE_POWER_RAIL         // Enable power rail reference
#define MMC5603X_ENABLE_DEINIT             // Enable deinit call
//#define MMC5603X_ENABLE_S4S                // Enable S4S parts
#define MMC5603X_ENABLE_ALL_DEVICES        // Enable MEMSIC all sensors
//#define MMC5603X_ENABLE_I3C_SUPPORT
//#define MMC5603X_ENABLE_DRI                // Enable DRI
#define MMC5603X_MODE_SWITCH



#define MAX_DEVICE_MODE_SUPPORTED 1


