#pragma once
#include "mag6d_common.h"
#include "mag6d_mag_track.h"
#include "mag6d_gyro_rotation.h"

//   buffered mag&rotation data used for mag bias calculation algorithm
typedef struct mag6d_sample_data {
    // sampled data
    vector umag1;    // U1, start of the track
    vector umag2;    // U2, end of the track
    rotmat rotation; // R,  rotation matrix
    uint64_t ts_ns;
#if MAG6D_SAMPLE_DATA_PRE_CALC_LOSS_GRAD
    // precalculated for loss and grad
    vector __precalc_vec1; // vec1 = R x U1 - U2
    rotmat __precalc_mat1; // mat1 = E - R
    rotmat __precalc_mat2; // mat2 = (E - R)^T x 2
#endif
    elem last_loss;
    struct mag6d_sample_data *prev;
    struct mag6d_sample_data *next;
    bool in_use;     // if this sample is in use
} mag6d_sample_data;

// make sample data from mag_track and gyro_rotation
void mag6d_sample_data_create(mag6d_sample_data *data, mag_track *mt, gyro_rotation *gr);

typedef struct mag6d_sample_queue {
    struct mag6d_sample_data data[MAG6D_SAMPLE_QUEUE_MAX_LENGTH];
    struct mag6d_sample_data *head;
    struct mag6d_sample_data *tail;
    int size;
} mag6d_sample_queue;

// reset sample queue to the initial state of no sample
void mag6d_sample_queue_reset(mag6d_sample_queue *queue);

// push mag6d_sample_data into tail of mag6d_sample_queue
void mag6d_sample_queue_append(mag6d_sample_queue *queue, mag6d_sample_data *data);

// remove and re-alloc space for new mag6d_sample_data
void mag6d_sample_queue_remove(mag6d_sample_queue *queue, mag6d_sample_data *data);

// input sample data from mag_track and gyro_rotation
void mag6d_sample_queue_input(mag6d_sample_queue *queue, mag_track *mt, gyro_rotation *gr);

// dump all data in mag6d_sample_queue
// void mag6d_sample_queue_dump_print(mag6d_sample_queue *queue);

// traverse every sample data in sample queue
#define MAG6D_SAMPLE_QUEUE_FOREACH_ALL(_data, _queue, action) \
    for (int __foreach_idx=0; __foreach_idx<MAG6D_SAMPLE_QUEUE_MAX_LENGTH; __foreach_idx++) \
    { \
        mag6d_sample_data *_data = &_queue->data[__foreach_idx]; \
        action \
    }

// traverse every valid sample data in sample queue
#define MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(_data, _queue, action) \
    for (int __foreach_idx=0; __foreach_idx<MAG6D_SAMPLE_QUEUE_MAX_LENGTH; __foreach_idx++) \
    { \
        mag6d_sample_data *_data = &_queue->data[__foreach_idx]; \
        if (_data->in_use) \
        { \
            action \
        } \
    }

// traverse every valid sample data in sample queue, ordered by 'next' pointer
#define MAG6D_SAMPLE_QUEUE_FOREACH_VALID_ORDERED_BY_NEXT(_data, _begin, _end, action) \
    for (mag6d_sample_data * _data = _begin; _data!=_end && _data; _data = _data->next) \
    { \
        action \
    }

// traverse every valid sample data in sample queue, ordered by 'prev' pointer
#define MAG6D_SAMPLE_QUEUE_FOREACH_VALID_ORDERED_BY_PREV(_data, _begin, _end, action) \
    for (mag6d_sample_data * _data = _begin; _data!=_end && _data; _data = _data->prev) \
    { \
        action \
    }

// indicates whether queue is full
#define MAG6D_SAMPLE_QUEUE_IS_EMPTY(_queue) ((_queue)->size == 0)

// indicates whether queue is full
#define MAG6D_SAMPLE_QUEUE_IS_FULL(_queue) ((_queue)->size == MAG6D_SAMPLE_QUEUE_MAX_LENGTH)

