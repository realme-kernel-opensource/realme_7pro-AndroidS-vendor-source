#ifndef AKM_FILTER_INCLUDE_H
#define AKM_FILTER_INCLUDE_H
#include "../Platform/AKM_CustomerSpec.h"
#include "../AKM_Common.h"

/*******These must be defined in AKM_CustomerSpec.h if filter is used***********
//pg mag filter
#define AKM_MAG_AVE_BUF_SIZE 16

//PG acc filter
#define AKM_ACC_AVE_BUF_SIZE 16

//D6D yaw average
#define AKM_YAW_RC_GAIN 0.01f


//Detect phone state
#define AKM_MOVING_BUF_SIZE 16

#define AKM_MOVING_VAR_THRESHOLD_X 6.0f
#define AKM_MOVING_VAR_THRESHOLD_Y 6.0f
#define AKM_MOVING_VAR_THRESHOLD_Z 7.5f

#define COS_20D_SQUARE 0.88302
#define COS_10D_SQUARE 0.96985
#define AKM_HORIZONTAL_THRESHOLD  COS_10D_SQUARE

#define AKM_STABLE_COUNT 150
#define AKM_STEBLE_COUNT_EXTRA 50
#define AKM_STABLE_FLUC_TIMES  10

**********************************************************************/

#ifdef AKM_ENABLE_PG_MAG_FILTER
void akm_pg_mag_filter_init(void);
int16_t akm_pg_mag_filter(float32_t *mx, float32_t *my, float32_t *mz);
#endif

#ifdef AKM_ENABLE_PG_ACC_FILTER
void akm_pg_acc_filter_init(void);
int16_t akm_pg_acc_filter(float32_t *ax, float32_t *ay, float32_t *az);
#endif

#ifdef AKM_ENABLE_D6D_YAW_FILTER
void akm_d6d_yaw_filter_init(void);
float32_t akm_d6d_yaw_filter(float32_t data);
#endif

#ifdef AKM_ENABLE_DETECT_PHONE_STATE
void akm_detect_phone_state_init(void);
int16_t akm_detect_phone_state(float32_t ax, float32_t ay, float32_t az);
#endif

#endif
