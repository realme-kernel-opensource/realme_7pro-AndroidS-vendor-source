#include "mag6d_sample.h"

void mag6d_sample_queue_reset(mag6d_sample_queue *queue)
{
    queue->size = 0;
    queue->head = queue->data;
    queue->head->prev = NULL;
    queue->tail = queue->data;
    queue->tail->next = NULL;
    MAG6D_SAMPLE_QUEUE_FOREACH_ALL(data, queue, {
        data->in_use = false;
    });
}


void mag6d_sample_queue_append(mag6d_sample_queue *queue, mag6d_sample_data *data)
{
    if (!queue->head || !queue->head->in_use) {
        queue->head = data;
    }

    if (queue->tail && queue->tail->in_use) {
        queue->tail->next = data;
    }

    data->prev = queue->tail;
    data->next = NULL;
    data->in_use = true;
    queue->tail = data;
    queue->size++;
}

void mag6d_sample_queue_remove(mag6d_sample_queue *queue, mag6d_sample_data *data)
{
    if (data && data->in_use) {
        if (data == queue->head && data == queue->tail) {
            queue->head = NULL;
            queue->tail = NULL;
        } else if (data == queue->head) {
            queue->head = data->next;
            queue->head->prev = NULL;
        } else if (data == queue->tail) {
            queue->tail = data->prev;
            queue->tail->next = NULL;
        } else {
            data->next->prev = data->prev;
            data->prev->next = data->next;
        }

        data->in_use = false;
        data->next = data->prev = NULL;
        queue->size--;
    }
}

static mag6d_sample_data *mag6d_sample_queue_find_nearest(mag6d_sample_queue *queue, vector *umag, float *ret_distance)
{
    elem min_distance_sqr = elem_max;
    mag6d_sample_data *min_distance_data = NULL;
    MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
        elem distance_sqr = vector_distance_sqr(umag, &data->umag2);

        if(min_distance_sqr > distance_sqr)
        {
            min_distance_sqr = distance_sqr;
            min_distance_data = data;
        }
    });
    *ret_distance = elem_sqrt(min_distance_sqr);
    return min_distance_data;
}

static mag6d_sample_data *mag6d_sample_queue_find_earliest(mag6d_sample_queue *queue, uint64_t *age_to_latest)
{
    uint64_t earliest_sample_ts_ns = 0xFFFFFFFFFFFFFFFFLL;
    uint64_t latest_sample_ts_ns = 0LL;
    mag6d_sample_data *earliest_data = NULL;
    MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
        if(data->ts_ns < earliest_sample_ts_ns || earliest_data == NULL)
        {
            earliest_sample_ts_ns = data->ts_ns;
            earliest_data = data;
        }
        if(data->ts_ns > latest_sample_ts_ns || earliest_data == NULL)
        {
            latest_sample_ts_ns = data->ts_ns;
        }
    });

    if (age_to_latest != NULL) {
        *age_to_latest = latest_sample_ts_ns - earliest_data->ts_ns;
    }

    return earliest_data;
}

static mag6d_sample_data *mag6d_sample_queue_auto_remove_and_alloc(mag6d_sample_queue *queue, mag_track *mt)
{
    // check if a empty slot exists
    MAG6D_SAMPLE_QUEUE_FOREACH_ALL(data, queue, {
        if (!data->in_use)
        {
            MAG6D_DEBUG("mag6d_sample_queue_auto_remove_and_alloc return by idle");
            return data;
        }
    });

    // if there is some sample close enough to the incoming sample data, replace the existing point
    elem nearest_distance;
    mag6d_sample_data * nearest_data = mag6d_sample_queue_find_nearest(queue, &mt->umag2, &nearest_distance);

    if(nearest_data != NULL && nearest_distance * nearest_distance < MAG6D_MAG_TRACK_LENGTH_SQR_INPUT_TRIGGER) {
        MAG6D_DEBUG("mag6d_sample_queue_auto_remove_and_alloc return by nearest: distance=%d/1000", f1000(nearest_distance));
        mag6d_sample_queue_remove(queue, nearest_data);
        return nearest_data;
    }

    uint64_t age_ns = 0L;
    mag6d_sample_data * earliest_data = mag6d_sample_queue_find_earliest(queue, &age_ns);
    MAG6D_DEBUG("mag6d_sample_queue_auto_remove_and_alloc return by earliest: ts=%lu", earliest_data->ts_ns);
    mag6d_sample_queue_remove(queue, earliest_data);
    return earliest_data;
}

void mag6d_sample_queue_input(mag6d_sample_queue *queue, mag_track *mt, gyro_rotation *gr)
{
    // MAG6D_DEBUG("mag6d_sample_queue adopt umag1=[%d %d %d]/1000",f1000(mt->umag1[0]),f1000(mt->umag1[1]),f1000(mt->umag1[2]));
    // MAG6D_DEBUG("mag6d_sample_queue adopt umag2=[%d %d %d]/1000",f1000(mt->umag2[0]),f1000(mt->umag2[1]),f1000(mt->umag2[2]));
    // MAG6D_DEBUG("mag6d_sample_queue adopt rotation=[%d %d %d]/1000",f1000(gr->rotation[0]),f1000(gr->rotation[1]),f1000(gr->rotation[2]));
    // MAG6D_DEBUG("mag6d_sample_queue adopt rotation=[%d %d %d]/1000",f1000(gr->rotation[3]),f1000(gr->rotation[4]),f1000(gr->rotation[5]));
    // MAG6D_DEBUG("mag6d_sample_queue adopt rotation=[%d %d %d]/1000",f1000(gr->rotation[6]),f1000(gr->rotation[7]),f1000(gr->rotation[8]));

    // find a place to save new sample data
    mag6d_sample_data *data = mag6d_sample_queue_auto_remove_and_alloc(queue, mt);

    if (data == NULL) {
        MAG6D_ERROR("mag6d_sample_queue_auto_remove_and_alloc returned NULL!!");
        return;
    }

    mag6d_sample_data_create(data, mt, gr);
    mag6d_sample_queue_append(queue, data);
}

// void mag6d_sample_queue_dump_print(mag6d_sample_queue *queue)
// {
//     MAG6D_DEBUG("mag6d_sample_queue_dump_print: dump mag6d_sample_queue size=%d", queue->size);
//     MAG6D_SAMPLE_QUEUE_FOREACH_ALL(data, queue,
//     {
//         MAG6D_DEBUG("data[%02d]: 0x%08x -> [0x%08x] -> 0x%08x: in_use=%d, last_loss=%d/1000",
//             __foreach_idx,
//             data->prev, data, data->next,
//             data->in_use, f1000(data->last_loss));
//     });
//     MAG6D_DEBUG("mag6d_sample_queue_dump_print: foreach valid ordered");
//     MAG6D_SAMPLE_QUEUE_FOREACH_VALID_ORDERED_BY_NEXT(data, queue->head, NULL,
//     {
//         MAG6D_DEBUG("data[%02d]: 0x%08x -> [0x%08x] -> 0x%08x: in_use=%d, last_loss=%d/1000",
//             data - queue->data,
//             data->prev, data, data->next,
//             data->in_use, f1000(data->last_loss));
//     });
// }