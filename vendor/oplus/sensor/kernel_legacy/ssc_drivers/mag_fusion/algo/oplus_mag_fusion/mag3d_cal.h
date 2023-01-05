#pragma once
/*********************************************************************
 *
 * file: mag3d_cal.h
 * magnetometer 3d calibration algorithm
 *
 *********************************************************************/
#include "rotation.h"
#include "stdbool.h"
#include "stdint.h"
#include "mag6d_common.h"

typedef struct mag3d_sample_data {
    vector mag;
    uint64_t ts_ns;
    struct mag3d_sample_data *prev;
    struct mag3d_sample_data *next;
    bool in_use;
} mag3d_sample_data;

typedef struct mag3d_sample_queue {
    mag3d_sample_data data[MAG3D_SAMPLE_QUEUE_MAX_LENGTH];
    struct mag3d_sample_data *head;
    struct mag3d_sample_data *tail;
    int size;
    int input_count;
} mag3d_sample_queue;

// int mag3d_sample_queue_init(mag3d_sample_queue *queue);

// mag3d_sample_data * mag3d_sample_queue_input(mag3d_sample_queue *queue, vector *data, uint64_t ts_ns, elem distance_thd);

// #define MAG3D_SAMPLE_QUEUE_FOREACH_ALL(_data, _queue, action) \
//     for(int __index=0; __index<MAG3D_SAMPLE_QUEUE_MAX_LENGTH; __index++){ \
//         mag3d_sample_data *_data = &(_queue)->data[__index]; \
//         action \
//     }

// #define MAG3D_SAMPLE_QUEUE_FOREACH_VALID(_data, _queue, action) \
//     MAG3D_SAMPLE_QUEUE_FOREACH_ALL(_data, _queue, {if(_data->in_use){action}})

// #define MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(_data, _queue, action) \
//     for(mag3d_sample_data *_data = (_queue)->head; _data; _data = _data->next){ action }


//   states of mag3d algorithm
typedef enum mag3d_algo_state {
    MAG3D_STATE_INITIAL = 0,        // first time initialiazed (unused)
    MAG3D_STATE_RESUMED = 1,        // confirming if last bias value is suitable
    MAG3D_STATE_SLIGHTLY_INTERFERED = 2, // interfered by slight magnetic field
    MAG3D_STATE_INTERFERED = 3,     // interfered by strong magnetic field
    MAG3D_STATE_RELIABLE = 4,       // reached highest accuracy
} mag3d_algo_state;


//   mag3d algorithm
typedef struct mag3d_cal_algo_s {
    mag3d_sample_queue   queue;
    mag3d_algo_state     state;
    vector               last_mag;
    vector               last_reliable_bias;
    uint64_t             last_reliable_ts_ns;
    vector               curr_bias;
    elem                 curr_radius;
    elem                 curr_variance;
    int                  accuracy;
    bool                 output_ready;
    bool                 interfered;
    float                debug_info[16];
} mag3d_cal_algo_t;

int mag3d_cal_algo_init(mag3d_cal_algo_t *algo, bool first, float bias[3], int accuracy);

int mag3d_cal_algo_deinit(mag3d_cal_algo_t *algo);

int mag3d_cal_algo_set_mag(mag3d_cal_algo_t *algo, elem *data, uint64_t ts_ns);

int mag3d_cal_algo_get_bias(mag3d_cal_algo_t *algo, elem *out, int *accuracy, bool *result_changed, bool *interfered);
