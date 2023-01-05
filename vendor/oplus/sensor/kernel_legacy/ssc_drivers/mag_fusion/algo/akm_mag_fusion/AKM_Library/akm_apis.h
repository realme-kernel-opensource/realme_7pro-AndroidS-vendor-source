
#ifndef INCLUDE_AKM_APIS_H
#define INCLUDE_AKM_APIS_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../AKM_Common.h"
#include "akl_apis.h"
#include "akl_smart_compass.h"

int16_t AKM_LibraryInit(uint32_t device, uint32_t layout_pat, uint32_t mode);
int16_t AKM_LoadAndStart(void);
int16_t AKM_StopAndSave(void);
/*------WARNING------
 *   SET DATA UNIT
 *	MAG: uT
 *	ACC: m/s2
 *	GYR: rad/s
 *	timestamp: us
 */
int16_t AKM_SetData(struct AKM_SENSOR_DATA *data);
/*------WARNING------
*	GET DATA UNIT
*  MAG: uT
*  ACC: m/s2
*  GYR: rad/s
*  ORI: deg
*  GRAVITY: m/s2
*  Linear ACC: m/s2
*  QUAT: 1
*  timestamp: us
*/
int16_t AKM_GetData(float32_t result[6], int senor_type, int32_t *accuracy);

extern AKM_DEVICE_TYPE AKL_GetDeviceType(int device_id);
#endif
