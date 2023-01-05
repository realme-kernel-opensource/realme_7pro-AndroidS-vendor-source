#ifndef _INCLUDE_AKL_LIB_INTERFACE_H
#define _INCLUDE_AKL_LIB_INTERFACE_H

#include "../Platform/AKM_CustomerSpec.h"
#include "akl_smart_compass.h"
#include "../AKM_Common.h"

//Convert form AKSC(Q14) to unit 1
//D9DV2 do not need to convert
#define QUAT_AKSC_TO_UNIT(x)    ((float32_t)(x)/ 16384.0f)
//Convert form us to AKSC(Q4)
//Only dt use this macro, dt is small, must not overflow
#define TIME_USEC_TO_AKSC(x)  ((x) * 16 / 1000.0f)

//Maximum time [usec] in Q4
#define MAXTIME_USEC_IN_Q4  (2047000000)
/*!
 * The minimum interval of the offset estimation.
 * The unit is msec in 12Q4 format.
 */

int16 AKL_DOEAG_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEAG_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_DOEEX_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOEEX_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_DOE_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_Calibrate(struct AKL_SCL_PRMS *prms);
int16 AKL_DOE_SetLevel(struct AKL_SCL_PRMS *prms, AKSC_HDST hdst);


int16 AKL_D9D_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_D9D_CalcFusion(struct AKL_SCL_PRMS *prms);


int16 AKL_D6D_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_D6D_CalcFusion(struct AKL_SCL_PRMS *prms);

int16 AKL_PG_SetPointer(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_FreePointer(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_Init(struct AKL_SCL_PRMS *prms);
int16 AKL_PG_CalcAngularRate(struct AKL_SCL_PRMS *prms);

int16 AKL_NULL_Function(struct AKL_SCL_PRMS *prms);

#endif
