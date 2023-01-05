#pragma once
/*********************************************************************
 *
 * file: mag6d_cal.h
 * magnetometer 6D calibration algorithm
 *
 * 1.assume that the difference between uncal mag and calib mag is "bias":
 *       U -> uncal mag data
 *       M -> calib mag data
 *       bias = U - M
 *
 * 2.before the rotation, we get:
 *       U1 -> uncal mag data from mag sensor
 *   and we have:
 *       M1 = U1 - bias ------------------(1)
 *
 * 3.after the rotation, we get:
 *       U2 -> uncal mag data from mag sensor
 *       R -> rotation matrix from gyro sensor
 *   and we have:
 *       M2 = U2 - bias ------------------(2)
 *   and we expect M2 is equal to:
 *       EM2 = R x M1 --------------------(3)
 *
 * 4.but since bias may not be accurate, there is a difference:
 *       diff = EM2 - M2 -----------------(4)
 *
 * 5.combine the equations(1)(2)(3)(4) above:
 *       diff = R * (U1 - bias) - (U2 - bias)
 *
 * 6.using the module of vector 'diff' as loss function:
 *       loss(bias) = module(diff)
 *   and simplify the gradient of loss function:
 *       loss_grad(bias) = 2 x (E-R)^T x diff
 *
 * 7.use gradient descent algorithm to get the bias which leads to smallest loss
 *********************************************************************/

#ifndef __MAG6AXIS_H__
#define __MAG6AXIS_H__

#include "rotation.h"
#include "stdbool.h"
#include "stdint.h"

#include "mag6d_common.h"
#include "mag3d_cal.h"
#include "mag6d_mag_track.h"
#include "mag6d_gyro_rotation.h"
#include "mag6d_sample.h"

///////////////////////////////////////////////////////////////////////////////
// mag6d_cal_algo
//   the algorithm to calculate the bias of field magnet data, receives magnet
// data and gyro data.

//   states of mag6d algorithm
typedef enum mag6d_algo_state {
    MAG6D_STATE_INITIAL,        // first time initialiazed (unused)
    MAG6D_STATE_RESUMED,        // confirming if last bias value is correct
    MAG6D_STATE_INTERFERED,     // interfered by strong magnetic field
    MAG6D_STATE_RELIABLE,       // reached highest accuracy
} mag6d_algo_state;

typedef struct mag6d_cal_algo_s {
    mag_track            curr_mag_track;
    gyro_rotation        curr_gyro_rotation;
    mag6d_sample_queue   queue;
    mag6d_algo_state     algo_state;
    vector               curr_bias;
    elem                 curr_grad_len_sqr;
    elem                 curr_radius;
    int                  realtime_accuracy;
    bool                 output_ready;
    bool                 interfered;
    elem                 debug_info[16];
} mag6d_cal_algo_t;

int mag6d_cal_algo_init(mag6d_cal_algo_t *algo, bool first, float bias[3], int accuracy);

int mag6d_cal_algo_deinit(mag6d_cal_algo_t *algo);

int mag6d_cal_algo_set_mag(mag6d_cal_algo_t *algo, elem *data, uint64_t ts_ns);

int mag6d_cal_algo_set_gyro(mag6d_cal_algo_t *algo, elem *data, uint64_t ts_ns);

int mag6d_cal_algo_get_bias(mag6d_cal_algo_t *algo, elem *out_bias, int *out_accuracy, bool *result_changed, bool *out_interfered);

int mag6d_cal_algo_get_debug_info(mag6d_cal_algo_t *algo, elem *out);

#endif