#ifndef AKM_WRAPPER_INCLUDE_H
#define AKM_WRAPPER_INCLUDE_H

#include "sns_mag_fusion_sensor_instance.h"
#include "fusion_api.h"

#define AKM_ALGO_9D 0x84 //AKL_MODE_FUSION_9D | AKL_MODE_CALIB_DOEAG
#define AKM_ALGO_6D 0x22 //AKL_MODE_FUSION_6D_PG_ON | AKL_MODE_CALIB_DOEEX

typedef enum {
    INPUT_TYPE_ACCEL = 0,
    INPUT_TYPE_MAG,
    INPUT_TYPE_GYRO
} INPUT_TYPE;

typedef struct {
    float data_output[9];
    int accuracy;
} OUTPUT_DATA;

void akm_wrapper_get_mag(OUTPUT_DATA *data);
void akm_wrapper_get_ori(OUTPUT_DATA *data);
void akm_wrapper_get_gravity(OUTPUT_DATA *data);
void akm_wrapper_get_lacc(OUTPUT_DATA *data);
void akm_wrapper_get_quat(OUTPUT_DATA *data);

#endif


