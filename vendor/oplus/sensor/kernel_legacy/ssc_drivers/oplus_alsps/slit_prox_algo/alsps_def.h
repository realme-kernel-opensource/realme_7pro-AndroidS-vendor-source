/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - alsps_def.h
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>    	<author>              		<desc>
*******************************************************************/
#ifndef _ALSPS_DEF_H_
#define _ALSPS_DEF_H_
#define CFG_MSM_SEE_PLATFORM
#if defined(CFG_MSM_SEE_PLATFORM)
// qcom see platform
#include <math.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_event_service.h"
#include "sns_rc.h"
#include "sns_diag_service.h"
#include "sns_printf.h"
#include "sns_printf_int.h"
#elif defined(CFG_MTK_SCP_PLATFORM)
// mtk scp platform
#include <math.h>
#include <stdint.h>
#include <mt_printf.h>
#include <plat/inc/rtc.h>
#include <i2c.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#endif

#define STAG           "[oplus_alsps] "

#define CFG_ALSPS_ALGORITHM_LOG
#ifdef CFG_MSM_SEE_PLATFORM//Qcom
#ifdef CFG_ALSPS_ALGORITHM_LOG
//enable log
#define ALSPS_LOG(fmt, args...)       SNS_SPRINTF(ERROR, sns_fw_printf, STAG" %s : "fmt, __FUNCTION__, ##args)
#else
#define ALSPS_LOG(fmt, args...)
#endif
//err log should aways on
#define ALSPS_ERR(fmt, args...)       SNS_SPRINTF(ERROR, sns_fw_printf, STAG" %s : "fmt, __FUNCTION__, ##args)

#elif defined(CFG_MTK_SCP_PLATFORM)//MTK
#ifdef CFG_ALSPS_ALGORITHM_LOG
//enable log
#define ALSPS_LOG(fmt, args...)       osLog(LOG_ERROR, STAG" %s : "fmt, __FUNCTION__, ##args)
#else
#define ALSPS_LOG(fmt, args...)
#endif
//err log should aways on
#define ALSPS_ERR(fmt, args...)       osLog(LOG_ERROR, STAG" %s : "fmt, __FUNCTION__, ##args)
#endif


struct alsps_data {
    uint8_t sensType;
    union {
        struct {
            int als_data;// unit: lux
        };
        struct {
            int ps_data;// unit: cm
            int ps_state;
        };
        struct {
            int lux;
            int time_dealta;
            float classify_param;//use to classify light
        };
    };
};

#endif //_ALSPS_DEF_H_
