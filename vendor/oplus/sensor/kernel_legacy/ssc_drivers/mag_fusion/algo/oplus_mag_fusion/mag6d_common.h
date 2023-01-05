#pragma once

#include "fusion_api.h"
#include "stdint.h"
#include "stdbool.h"
#include "rotation.h"

#include "sns_mag_fusion_sensor_instance.h"

// #define MAG6D_ENABLE_DEBUG
#define MAG6D_ENABLE_LOG_INFO
#define MAG6D_LOG_STR "MAG6D_CAL -- "

#ifdef MAG6D_ENABLE_DEBUG
#define MAG6D_DEBUG(fmt, ...) do { \
	SNS_SPRINTF(HIGH, sns_fw_printf, MAG6D_LOG_STR "" fmt, ##__VA_ARGS__); \
} while (0)
#else
#define MAG6D_DEBUG(fmt, ...)
#endif

#ifdef MAG6D_ENABLE_LOG_INFO
#define MAG6D_INFO(fmt, ...) do { \
	SNS_SPRINTF(HIGH, sns_fw_printf, MAG6D_LOG_STR "" fmt, ##__VA_ARGS__); \
} while (0)
#else
#define MAG6D_INFO(fmt, ...)
#endif

#define MAG6D_ERROR(fmt, ...) do { \
	SNS_SPRINTF(HIGH, sns_fw_printf, MAG6D_LOG_STR "" fmt, ##__VA_ARGS__); \
} while (0)

#define f1000(f) ((int)((f)*1000))
#define abs_for_fp1000(f) ((f)<0?-f:f)

// e.g: printf("value = %d.%03d",fp1000(-2.05f))  ===>  "value = -2.050"
// caution: argument 'f' may be called times
#define fp1000(f) ((int)(f)),(((int)(abs_for_fp1000(f)*1000))%1000)
#define rad(f) ((f)*PI/180)
#define radi(f) ((int)(f)*PI/180)
#define deg(f) ((f)*180.0f/PI)


///////////////////////////////////////////////////////////////////////////////
//  physic constants
#define MAG6D_MIN_MAG_FIELD_RADIUS                  elem_val(25.0)
#define MAG6D_MAX_MAG_FIELD_RADIUS                  elem_val(65.0)
#define MAG6D_DEFAULT_MAG_FIELD_RADIUS              elem_val(45.0)

#define MAG6D_DYNAMIC_MAG_LENGTH_LIMIT              elem_val(150.0)
#define MAG6D_ABSOLUTE_MAG_LENGTH_LIMIT             elem_val(800.0)

#define RAD(degree) elem_mul((degree), elem_val(0.017453293f)) // (degree)*3.141592653/180.0
#define DEG(radian) elem_mul((radian), elem_val(57.295779524)) // (radian)*180.0/3.141592653

///////////////////////////////////////////////////////////////////////////////
//  mag6d function switch
//   when inserting sample data, additional precalculated data will be
// calculated and saved into mag6d_sample_data with the sample
#define MAG6D_SAMPLE_DATA_PRE_CALC_LOSS_GRAD        true

//   smartly choose the old sample data to be replaced by the incoming
// new sample data.
//
//   strategy (orderly judging):
//   1.if there is a sample point too close, replace it
//   2.if queue is full, replace the oldest data
//   3.if queue is not full, insert to a new position
//
//   note: this switch must be enabled now
// #define MAG6D_SAMPLE_DATA_SMART_INSERT              true

///////////////////////////////////////////////////////////////////////////////
//  mag6d mag_track configure
//   when the square of the length of mag_track is larger than this, the
// algorithm will insert sample data(including magdata and the rotation)
// to a queue, reset mag_track and gyro_rotation, and trigger calculation
#define MAG6D_MAG_TRACK_LENGTH_SQR_INPUT_TRIGGER    elem_val(12.5f)

//   maximum allowed mag value for each axis, samples over this value will
// be dropped and leads to the clearing of the whole sample queue
#define MAG6D_MAG_TRACK_INTERFERE_TRIGGER           elem_val(300.0f)

//   maximum allowed mag value for each axis, samples over this value will
// be dropped and leads to the clearing of the whole sample queue
#define MAG6D_ABSOLUTE_MAG_INTERFERE_TRIGGER        elem_val(1500.0f)

//   if mag_track includes too much points, accumulated deviation of gyro
// will be too big.
#define MAG6D_MAG_TRACK_MAX_POINT_NUM               100

//   prevent sampling on one mag_track for too long time
#define MAG6D_MAG_TRACK_TIMEOUT_NS                  3*1000000000LL // *sec

///////////////////////////////////////////////////////////////////////////////
//  mag6d gyro_rotation configure
//   Maximum gyro value. If gyro data received is over this value, it may
// not be so accurate, so clear current mag_track and gyro_rotation
#define MAG6D_GYRO_ROTATION_MAX_GYRO_VALUE          elem_val(20.0f) // not used now


///////////////////////////////////////////////////////////////////////////////
//  mag6d sample_queue configure
//   minimum num of samples to start iteration calculation
#define MAG6D_SAMPLE_QUEUE_MIN_CALC_LENGTH          6

//   maximum length of the queue of sample data
#define MAG6D_SAMPLE_QUEUE_MAX_LENGTH               20

//   samples older than this value will be removed first
#define MAG6D_SAMPLE_MAX_AGE_NS                     10*1000000000LL // *sec

///////////////////////////////////////////////////////////////////////////////
//  mag6d gradient-descent algorithm configure
//   initial step length when starting a gradient-descendent calculation
#define MAG6D_ITERATION_INIT_STEP_LENGTH            elem_val(0.1f)
#define MAG6D_ITERATION_STEP_INCREASE_FACTOR        elem_val(1.618f)
#define MAG6D_ITERATION_STEP_DECREASE_FACTOR        elem_val(0.3f)

//   gradient-descendent calculation iteration times
#define MAG6D_ITERATION_MAX_TIMES_PER_EPOCH         10

//   if the cost gradient length is less than this, the iteration in the
// procedure of gradient-descendent calculation will be stopped
#define MAG6D_COST_GRAD_THD_FOR_STOPPING_ITERATION  elem_val(1.0f)

///////////////////////////////////////////////////////////////////////////////
//  mag6d accuracy calculation configure
//   thresholds to judge the upper limit of accuracy by sample range
// (bigger is better)
#define MAG6D_YAW_RANGE_ACCURACY_HIGH_THRESHOLD     elem_mul(PI,elem_val(0.2f))
#define MAG6D_YAW_RANGE_ACCURACY_MED_THRESHOLD      elem_mul(PI,elem_val(0.1f))
#define MAG6D_YAW_RANGE_ACCURACY_LOW_THRESHOLD      elem_mul(PI,elem_val(0.05f))
#define MAG6D_PITCH_RANGE_ACCURACY_HIGH_THRESHOLD   elem_mul(PI,elem_val(0.1f))
#define MAG6D_PITCH_RANGE_ACCURACY_MED_THRESHOLD    elem_mul(PI,elem_val(0.05f))
#define MAG6D_PITCH_RANGE_ACCURACY_LOW_THRESHOLD    elem_mul(PI,elem_val(0.01f))

//   thresholds to judge the upper limit of accuracy by the square of cost
// gradient len (smaller is better)
#define MAG6D_COST_GRADIENT_ACCURACY_HIGH_THRESHOLD elem_val(0.5f)
#define MAG6D_COST_GRADIENT_ACCURACY_MED_THRESHOLD  elem_val(1.0f)
#define MAG6D_COST_GRADIENT_ACCURACY_LOW_THRESHOLD  elem_val(4.0f)

//   thresholds to judge the upper limit of accuracy by the variance rate
// to radius of the samples (smaller is better)
#define MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_HIGH_THRESHOLD elem_val(0.10f)
#define MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_MED_THRESHOLD  elem_val(0.20f)
#define MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_LOW_THRESHOLD  elem_val(1.00f)

///////////////////////////////////////////////////////////////////////////////
//  mag3d config
//   maximum length of the sample data queue
#define MAG3D_SAMPLE_QUEUE_MAX_LENGTH               40

//   minimum length of the sample data queue to start calculation
#define MAG3D_SAMPLE_QUEUE_MIN_LENGTH               10

//   new samples nearer to last sample data than this value will
// be ignored.
#define MAG3D_MINIMUM_INPUT_MAG_DISTANCE_FAST       elem_val(3.0f)
#define MAG3D_MINIMUM_INPUT_MAG_DISTANCE_SLOW       elem_val(30.0f)

//   maximum allowed deviation rate of the radius of any sample
// comparing to average radius
#define MAG3D_MAX_RADIUS_DEVIATION_RATE_THD         elem_val(0.1f)

//   thresholds to judge the upper limit of accuracy by sample range
// (bigger is better)
#define MAG3D_YAW_RANGE_ACCURACY_HIGH_THRESHOLD     RAD(elem_val(90))
#define MAG3D_YAW_RANGE_ACCURACY_MED_THRESHOLD      RAD(elem_val(30))
#define MAG3D_YAW_RANGE_ACCURACY_LOW_THRESHOLD      RAD(elem_val(10))
#define MAG3D_PITCH_RANGE_ACCURACY_HIGH_THRESHOLD   RAD(elem_val(50))
#define MAG3D_PITCH_RANGE_ACCURACY_MED_THRESHOLD    RAD(elem_val(20))
#define MAG3D_PITCH_RANGE_ACCURACY_LOW_THRESHOLD    RAD(elem_val(10))
#define MAG3D_XY_RANGE_REQUIREMENT                  elem_val(35)
#define MAG3D_XY_RANGE_REQUIREMENT_LOW              elem_val(25)
#define MAG3D_Z_RANGE_REQUIREMENT                   elem_val(20)

//   thresholds to judge the upper limit of accuracy by the variance
// rate to radius of the samples (smaller is better)
#define MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_HIGH_THRESHOLD elem_val(0.10f)
#define MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_MED_THRESHOLD  elem_val(0.25f)
#define MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_LOW_THRESHOLD  elem_val(0.50f)

//   thresholds to judge the upper limit of accuracy by sample num
// (bigger is better)
#define MAG3D_SAMPLE_NUM_ACCURACY_HIGH_THRESHOLD    (int)(MAG3D_SAMPLE_QUEUE_MAX_LENGTH * 0.6)
#define MAG3D_SAMPLE_NUM_ACCURACY_MED_THRESHOLD     (int)(MAG3D_SAMPLE_QUEUE_MAX_LENGTH * 0.3)
#define MAG3D_SAMPLE_NUM_ACCURACY_LOW_THRESHOLD     (int)(MAG3D_SAMPLE_QUEUE_MAX_LENGTH * 0.1)

///////////////////////////////////////////////////////////////////////////////
//  utils
void mag6d_qsort_elems(elem *arr, int len);

bool mag6d_check_input_interfered(vector *umag, vector *current_bias, bool bias_reliable);
