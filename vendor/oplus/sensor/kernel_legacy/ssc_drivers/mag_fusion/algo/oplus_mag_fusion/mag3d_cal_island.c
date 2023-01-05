/*********************************************************************
 *
 * file: mag3d_cal_island.c
 * magnetometer 3d calibration algorithm
 *
 *********************************************************************/
#include "mag3d_cal.h"

#define MAG3D_SAMPLE_QUEUE_FOREACH_ALL(_data, _queue, action) \
    for(int __index=0; __index<MAG3D_SAMPLE_QUEUE_MAX_LENGTH; __index++){ \
        mag3d_sample_data *_data = &(_queue)->data[__index]; \
        action \
    }

#define MAG3D_SAMPLE_QUEUE_FOREACH_VALID(_data, _queue, action) \
    MAG3D_SAMPLE_QUEUE_FOREACH_ALL(_data, _queue, {if(_data->in_use){action}})

#define MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(_data, _queue, action) \
    for(mag3d_sample_data *_data = (_queue)->head; _data; _data = _data->next){ action }

int mag3d_sample_queue_init(mag3d_sample_queue *queue)
{
    for (int i = 0; i < MAG3D_SAMPLE_QUEUE_MAX_LENGTH; i++) {
        queue->data[i].in_use = false;
    }

    queue->head = queue->data;
    queue->tail = queue->data;
    queue->size = 0;
    queue->input_count = 0;

    return 0;
}

static mag3d_sample_data *mag3d_sample_queue_alloc_node(mag3d_sample_queue *queue, vector *mag, elem distance_thd)
{
    // find an idle one
    MAG3D_SAMPLE_QUEUE_FOREACH_ALL(data, queue, {
        if (!data->in_use)
        {
            return data;
        }
    })

    // find the one near enough
    elem distance_thd_sqr = elem_sqr(distance_thd);
    MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(data, queue, {
        elem distance_sqr = vector_distance_sqr(mag, &data->mag);

        if (distance_sqr < distance_thd_sqr)
        {
            return data;
        }
    })

    // find the earliest one
    mag3d_sample_data *earliest_data = queue->head;
    return earliest_data;
}

static void mag3d_sample_queue_remove_node(mag3d_sample_queue *queue, mag3d_sample_data *data)
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

static void mag3d_sample_queue_append_node(mag3d_sample_queue *queue, mag3d_sample_data *data)
{
    if (!queue->head) {
        queue->head = data;
    }

    if (queue->tail) {
        queue->tail->next = data;
    }

    data->prev = queue->tail;
    data->next = NULL;
    data->in_use = true;
    queue->tail = data;
    queue->size++;
}

mag3d_sample_data * mag3d_sample_queue_input(mag3d_sample_queue *queue, vector *data, uint64_t ts_ns, elem distance_thd)
{
    mag3d_sample_data *insert_data = mag3d_sample_queue_alloc_node(queue, data, distance_thd);
    mag3d_sample_queue_remove_node(queue, insert_data);
    VECTOR_COPY(insert_data->mag, *data);
    insert_data->ts_ns = ts_ns;
    mag3d_sample_queue_append_node(queue, insert_data);
    queue->input_count++;
    return insert_data;
}

// static mag3d_sample_data * mag3d_queue_find_most_inaccurate_sample(mag3d_sample_queue *queue, vector *bias, elem radius, elem deviation_rate_thd)
// {
//     elem                max_deviation = elem_zero;
//     mag3d_sample_data * most_deviated_data = NULL;
//     int search_count = queue->size / 2;
//     MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(data, queue, {
//         elem this_radius = vector_distance(&data->mag, bias);
//         elem deviation = elem_abs(this_radius - radius);
//         if (max_deviation < deviation)
//         {
//             max_deviation = deviation;
//             most_deviated_data = data;
//         }
//         if (--search_count <= 0)
//         {
//             break;
//         }
//     })
//     if (most_deviated_data && elem_div(max_deviation, radius) > deviation_rate_thd)
//     {
//         return most_deviated_data;
//     }
//     return NULL;
// }

static mag3d_sample_data * mag3d_queue_find_first_inaccurate_sample(mag3d_sample_queue *queue, vector *bias, elem radius, elem deviation_rate_thd)
{
    int search_count = queue->size / 2;
    MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(data, queue, {
        elem this_radius = vector_distance(&data->mag, bias);
        elem deviation = elem_abs(this_radius - radius);

        if (elem_div(deviation, radius) > deviation_rate_thd)
        {
            return data;
        }
        if (--search_count <= 0)
        {
            break;
        }
    })
    return NULL;
}

#ifdef SNS_PRINTF_AP
#include "string.h"
static void mag3d_sample_queue_print_ap(mag3d_sample_queue *queue, mag3d_sample_data *remark_data, vector *bias, elem radius)
{
    char str[MAG3D_SAMPLE_QUEUE_MAX_LENGTH + 20] = "queue  =";
    int index = strlen(str);
    MAG3D_SAMPLE_QUEUE_FOREACH_ALL(data, queue, {
        str[index++] = (data == remark_data) ? '+' : data->in_use ? 'o' : '_' ;
    })
    str[index] = '\0';
    SNS_PRINTF_AP(HIGH, NULL, str);

    char str2[MAG3D_SAMPLE_QUEUE_MAX_LENGTH + 20] = "radius =";
    int index2 = strlen(str2);
    MAG3D_SAMPLE_QUEUE_FOREACH_ALL(data, queue, {
        if (data->in_use)
        {
            elem this_radius = vector_distance(&data->mag, bias);
            elem deviation_rate = elem_div(elem_abs(this_radius - radius), radius);

            switch((int)(deviation_rate * 10)) {
            case 0:
                str2[index2++] = '0';
                break;

            case 1:
                str2[index2++] = '1';
                break;

            case 2:
                str2[index2++] = '2';
                break;

            case 3:
                str2[index2++] = '3';
                break;

            case 4:
                str2[index2++] = '4';
                break;

            case 5:
                str2[index2++] = '5';
                break;

            case 6:
                str2[index2++] = '6';
                break;

            case 7:
                str2[index2++] = '7';
                break;

            case 8:
                str2[index2++] = '8';
                break;

            case 9:
                str2[index2++] = '9';
                break;

            default:
                str2[index2++] = '*';
                break;
            }
        } else
        {
            str2[index2++] = '_';
        }
    })
    str2[index2] = '\0';
    SNS_PRINTF_AP(HIGH, NULL, str2);
}
#else
#define mag3d_sample_queue_print_ap(...)
#endif

/* python prototype code:
import numpy as np
def get_bias_least_square(dataset):
    '''solve bias by mag data list

    Arguments:
        dataset {list<vector>} -- mag data set

    Returns:
        bias -- vector
    '''

    matmul = np.matmul
    inv = np.linalg.inv

    dataset_a = np.array(dataset[:-1])
    dataset_b = np.array(dataset[1:])

    # assume bias is B(x,y,z), there're equations:
    # (x1 - x)^2 + (y1 - y)^2 + (z1 - z)^2 = r^2
    # (x2 - x)^2 + (y2 - y)^2 + (z2 - z)^2 = r^2
    # (x3 - x)^2 + (y3 - y)^2 + (z3 - z)^2 = r^2
    # ...

    # subtraction between neighbouring equations:
    # 2(x2 - x1)x + 2(y2 - y1)y + 2(z2 - z1)z = x2^2 - x1^2 + y2^2 - y1^2 + z2^2 - z1^2
    # 2(x3 - x2)x + 2(y3 - y2)y + 2(z3 - z2)z = x3^2 - x2^2 + y3^2 - y2^2 + z3^2 - z2^2
    # 2(x4 - x3)x + 2(y4 - y3)y + 2(z4 - z3)z = x4^2 - x3^2 + y4^2 - y3^2 + z4^2 - z3^2
    # ...

    # they're A*B=Y formed:

    # A = [
    #   [2*(x2 - x1), 2*(y2 - y1), 2*(z2 - z1)],
    #   [2*(x3 - x2), 2*(y3 - y2), 2*(z3 - z2)],
    #   [2*(x4 - x3), 2*(y4 - y3), 2*(z4 - z3)],
    #   ...
    # ]
    A = 2*(dataset_b - dataset_a)

    # Y = [
    #   x2^2 - x1^2 + y2^2 - y1^2 + z2^2 - z1^2,
    #   x3^2 - x2^2 + y3^2 - y2^2 + z3^2 - z2^2,
    #   x4^2 - x1^2 + y4^2 - y1^2 + z4^2 - z1^2,
    #   ...
    # ]
    Y = np.sum(dataset_b*dataset_b - dataset_a*dataset_a, axis=1)

    # using least square method:
    # B = (A^T * A)^-1 * A^T * Y
    B = matmul(matmul(inv(matmul(A.T, A)), A.T), Y)

    return B
*/
static void mag3d_solve(mag3d_sample_queue *queue, vector *out_bias)
{
    static vector A[MAG3D_SAMPLE_QUEUE_MAX_LENGTH];
    static elem Y[MAG3D_SAMPLE_QUEUE_MAX_LENGTH];
    static rotmat ATA;
    static vector temp[MAG3D_SAMPLE_QUEUE_MAX_LENGTH];

    mag3d_sample_data *last_data = NULL;
    int arr_len = 0;
    // gather all mag data
    // the order is not necessary, using MAG3D_SAMPLE_QUEUE_FOREACH to decrease logic branches
    MAG3D_SAMPLE_QUEUE_FOREACH_ORDER(data, queue, {
        if (last_data != NULL)
        {
            vector *mag_a = &last_data->mag;
            vector *mag_b = &data->mag;
            vector_sub(&A[arr_len], mag_b, mag_a);
            vector_scale(&A[arr_len], &A[arr_len], elem_val(2));
            Y[arr_len] = vector_dot_prod(mag_b, mag_b) - vector_dot_prod(mag_a, mag_a);
            arr_len++;
        }
        last_data = data;
    })

    // ATA = A^T * A
    for (int i = 0; i < 9; i++) {
        ATA[i] = elem_zero;
    }

    for (int i = 0; i < arr_len; i++) {
        ATA[0] += elem_mul(A[i][0], A[i][0]);
        ATA[1] += elem_mul(A[i][0], A[i][1]);
        ATA[2] += elem_mul(A[i][0], A[i][2]);
        ATA[3] += elem_mul(A[i][1], A[i][0]);
        ATA[4] += elem_mul(A[i][1], A[i][1]);
        ATA[5] += elem_mul(A[i][1], A[i][2]);
        ATA[6] += elem_mul(A[i][2], A[i][0]);
        ATA[7] += elem_mul(A[i][2], A[i][1]);
        ATA[8] += elem_mul(A[i][2], A[i][2]);
    }

    rotmat_inverse(&ATA, &ATA, NULL, 0);

    // temp = ATA^-1 * A^T
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < arr_len; col++) {
            temp[col][row] = elem_mul(ATA[row * 3 + 0], A[col][0]) + elem_mul(ATA[row * 3 + 1], A[col][1]) + elem_mul(ATA[row * 3 + 2], A[col][2]);
        }
    }

    // bias = temp * Y
    (*out_bias)[0] = elem_zero;
    (*out_bias)[1] = elem_zero;
    (*out_bias)[2] = elem_zero;

    for (int i = 0; i < arr_len; i++) {
        (*out_bias)[0] += elem_mul(temp[i][0], Y[i]);
        (*out_bias)[1] += elem_mul(temp[i][1], Y[i]);
        (*out_bias)[2] += elem_mul(temp[i][2], Y[i]);
    }
}

static elem mag3d_calc_radius_mean(mag3d_sample_queue *queue, vector *bias)
{
    elem radius = elem_zero;
    MAG3D_SAMPLE_QUEUE_FOREACH_VALID(data, queue, {
        radius += vector_distance(bias, &data->mag);
    })
    radius = elem_div(radius, elem_val(queue->size));
    return radius < MAG6D_MIN_MAG_FIELD_RADIUS ? MAG6D_MIN_MAG_FIELD_RADIUS :
        radius > MAG6D_MAX_MAG_FIELD_RADIUS ? MAG6D_MAX_MAG_FIELD_RADIUS :
        radius;
}

static elem mag3d_calc_radius_variance(mag3d_sample_queue *queue, vector *bias, elem radius_mean)
{
    elem variance = elem_zero;
    MAG3D_SAMPLE_QUEUE_FOREACH_VALID(data, queue, {
        elem radius = vector_distance(bias, &data->mag);
        variance += elem_mul(radius_mean - radius, radius_mean - radius);
    })
    variance = elem_div(variance, elem_val(queue->size));
    return variance;
}

static elem mag3d_stat_yaw_range(mag3d_sample_queue *queue, vector *bias)
{
    // the range of yaw is complementary to the biggest interval about 2*pi

    // calc yaw angle for every mag
    elem yaw_list[queue->size];
    int idx = 0;
    MAG3D_SAMPLE_QUEUE_FOREACH_VALID(data, queue, {
        vector temp_cmag;
        yaw_list[idx++] = vector_yaw(vector_sub(&temp_cmag, &data->mag, bias));
    })

    // find biggest interval in yaw list
    mag6d_qsort_elems(yaw_list, queue->size);
    elem max_yaw_interval = yaw_list[0] - yaw_list[queue->size - 1] + TWO_PI;

    for(int i = 1; i < queue->size; i++) {
        elem interval = yaw_list[i] - yaw_list[i - 1];

        if(max_yaw_interval < interval) {
            max_yaw_interval = interval;
        }
    }

    // the range of yaw is complementary to the biggest interval about 2*pi
    return TWO_PI - max_yaw_interval;
}

static elem mag3d_stat_pitch_range(mag3d_sample_queue *queue, vector *bias)
{
    // calc max&min z axis
    vector *max_z_umag = NULL;
    vector *min_z_umag = NULL;
    MAG3D_SAMPLE_QUEUE_FOREACH_VALID(data, queue, {
        if(max_z_umag == NULL || (*max_z_umag)[2] < data->mag[2])
        {
            max_z_umag = &data->mag;
        }
        if(min_z_umag == NULL || (*min_z_umag)[2] > data->mag[2])
        {
            min_z_umag = &data->mag;
        }
    });

    // the range of pitch depends on max&min pitch angle
    vector temp_cmag;
    elem max_pitch = vector_pitch(vector_sub(&temp_cmag, max_z_umag, bias));
    elem min_pitch = vector_pitch(vector_sub(&temp_cmag, min_z_umag, bias));
    return max_pitch - min_pitch;
}

static void mag3d_stat_xyz_range(mag3d_sample_queue *queue, elem *x_range, elem *y_range, elem *z_range)
{
    elem x_max = elem_min;
    elem y_max = elem_min;
    elem z_max = elem_min;
    elem x_min = elem_max;
    elem y_min = elem_max;
    elem z_min = elem_max;
    MAG3D_SAMPLE_QUEUE_FOREACH_VALID(data, queue, {
        if(x_min > data->mag[0]) x_min = data->mag[0];
        if(x_max < data->mag[0]) x_max = data->mag[0];
        if(y_min > data->mag[1]) y_min = data->mag[1];
        if(y_max < data->mag[1]) y_max = data->mag[1];
        if(z_min > data->mag[2]) z_min = data->mag[2];
        if(z_max < data->mag[2]) z_max = data->mag[2];
    })
    *x_range = x_max - x_min;
    *y_range = y_max - y_min;
    *z_range = z_max - z_min;
}

static int mag3d_estimate_accuracy(mag3d_sample_queue *queue, vector *bias, elem radius, elem radius_variance)
{
    int yaw_accuracy = 3;
    int pitch_accuracy = 3;
    int range_accuracy = 3;
    int variance_accuracy = 3;
    int sample_num_accuracy = 3;

    // judge by yaw range (bigger is better)
    {
        elem yaw_range = mag3d_stat_yaw_range(queue, bias);

        // judge accuracy
        yaw_accuracy = yaw_range > MAG3D_YAW_RANGE_ACCURACY_HIGH_THRESHOLD ? 3 :
            yaw_range > MAG3D_YAW_RANGE_ACCURACY_MED_THRESHOLD  ? 2 :
            yaw_range > MAG3D_YAW_RANGE_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;

        MAG6D_DEBUG("accuracy: [%d] yaw_range = %d deg", yaw_accuracy, (int)DEG(yaw_range));
    }

    // judge by pitch range (bigger is better)
    {
        elem pitch_range = mag3d_stat_pitch_range(queue, bias);

        // judge accuracy
        pitch_accuracy = pitch_range > MAG3D_PITCH_RANGE_ACCURACY_HIGH_THRESHOLD ? 3 :
            pitch_range > MAG3D_PITCH_RANGE_ACCURACY_MED_THRESHOLD  ? 2 :
            pitch_range > MAG3D_PITCH_RANGE_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;

        MAG6D_DEBUG("accuracy: [%d] pitch_range = %d deg", pitch_accuracy, (int)DEG(pitch_range));
    }

    // judge by box range
    {
        elem x_range, y_range, z_range;
        mag3d_stat_xyz_range(queue, &x_range, &y_range, &z_range);

        range_accuracy = 3;

        if ((x_range * x_range + y_range * y_range) < MAG3D_XY_RANGE_REQUIREMENT * MAG3D_XY_RANGE_REQUIREMENT)
            range_accuracy -= 1;

        if ((x_range * x_range + y_range * y_range) < MAG3D_XY_RANGE_REQUIREMENT_LOW * MAG3D_XY_RANGE_REQUIREMENT_LOW)
            range_accuracy -= 1;

        if (z_range < MAG3D_Z_RANGE_REQUIREMENT)
            range_accuracy -= 1;

        MAG6D_DEBUG("accuracy: [%d] xy_range=%d uT, z_range=%d uT",
            range_accuracy, (int)sqrtf(x_range * x_range + y_range * y_range), (int)z_range);
    }

    // judge by sample radius variance rate (smaller is better)
    {
        elem radius_variance_rate = elem_div(radius_variance, radius);
        variance_accuracy = radius_variance_rate < MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_HIGH_THRESHOLD ? 3 :
            radius_variance_rate < MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_MED_THRESHOLD  ? 2 :
            radius_variance_rate < MAG3D_RADIUS_VARIANCE_RATE_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;
        MAG6D_DEBUG("accuracy: [%d] radius_variance_rate = %d/1000", variance_accuracy, (int)(radius_variance_rate * 1000));
    }

    // judge by sample num (more is better)
    {
        sample_num_accuracy = queue->size > MAG3D_SAMPLE_NUM_ACCURACY_HIGH_THRESHOLD ? 3 :
            queue->size > MAG3D_SAMPLE_NUM_ACCURACY_MED_THRESHOLD  ? 2 :
            queue->size > MAG3D_SAMPLE_NUM_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;
        MAG6D_DEBUG("accuracy: [%d] queue->size = %d", sample_num_accuracy, queue->size);
    }

#define UPPER_LIMIT(var, value) if(var>value)var=value;
    int accuracy = 3;
    UPPER_LIMIT(accuracy, yaw_accuracy)
    UPPER_LIMIT(accuracy, pitch_accuracy)
    UPPER_LIMIT(accuracy, range_accuracy)
    UPPER_LIMIT(accuracy, variance_accuracy)
    UPPER_LIMIT(accuracy, sample_num_accuracy)
#undef UPPER_LIMIT

    if (accuracy == 0) {
        MAG6D_ERROR("calibration not ok. yaw:%d, pitch:%d, range:%d, variance:%d, sample_num:%d", yaw_accuracy, pitch_accuracy, range_accuracy, variance_accuracy,
            sample_num_accuracy);
    }

    return accuracy;
}

static void mag3d_update_bias(mag3d_cal_algo_t *algo)
{
    mag3d_solve(&algo->queue, &algo->curr_bias);
    algo->curr_radius = mag3d_calc_radius_mean(&algo->queue, &algo->curr_bias);

    // remove interfered sample when slightly interfered
    if (algo->queue.size >= MAG3D_SAMPLE_QUEUE_MIN_LENGTH * 2 && algo->state == MAG3D_STATE_SLIGHTLY_INTERFERED) {
        mag3d_sample_data * inaccurate_sample = mag3d_queue_find_first_inaccurate_sample(
                &algo->queue, &algo->curr_bias, algo->curr_radius, MAG3D_MAX_RADIUS_DEVIATION_RATE_THD);
        mag3d_sample_queue_remove_node(&algo->queue, inaccurate_sample);
        mag3d_solve(&algo->queue, &algo->curr_bias);
        algo->curr_radius = mag3d_calc_radius_mean(&algo->queue, &algo->curr_bias);
    }

    algo->curr_variance = mag3d_calc_radius_variance(&algo->queue, &algo->curr_bias, algo->curr_radius);
}

static void mag3d_process_state_transition(mag3d_cal_algo_t *algo, uint64_t ts_ns, int last_accuracy)
{
    (void)ts_ns;

    switch(algo->state) {
    case MAG3D_STATE_SLIGHTLY_INTERFERED: {
        // // if accuracy keeps lower than 2 for too long
        // if (algo->accuracy <= 1 && algo->queue.input_count > 100)
        // {
        //     MAG6D_DEBUG("set INTERFERED state because accuracy kept lower than"
        //         " 2 during over 100 samples on state SLIGHTLY_INTERFERED");
        //     algo->state = MAG3D_STATE_INTERFERED;
        //     // algo->interfered = true; // this flag will be catched below
        // }
        if (algo->accuracy == 2) {
            elem distance_to_last_bias = vector_distance(
                    &algo->last_reliable_bias, &algo->curr_bias);

            if (distance_to_last_bias > 10.0f) {
                MAG6D_DEBUG("set INTERFERED state because bias changed a lot");
                algo->state = MAG3D_STATE_INTERFERED;
            }
        }

        // FALL THROUGH...
    }

    case MAG3D_STATE_INITIAL:
    case MAG3D_STATE_RESUMED:
    case MAG3D_STATE_INTERFERED: {
        if (algo->interfered) {
            algo->state = MAG3D_STATE_INTERFERED;
            mag3d_sample_queue_init(&algo->queue);
            MAG6D_DEBUG("clear queue because interfered from state %d", algo->state);
            algo->output_ready = true;
        } else if (algo->accuracy == 3) {
            algo->state = MAG3D_STATE_RELIABLE;
            algo->output_ready = true;
        } else if (last_accuracy != algo->accuracy) {
            algo->output_ready = true;
        }

        break;
    }

    case MAG3D_STATE_RELIABLE: {
        if (algo->interfered) {
            algo->state = MAG3D_STATE_INTERFERED;
            mag3d_sample_queue_init(&algo->queue);
            MAG6D_DEBUG("clear queue because interfered when reliable");
            algo->output_ready = true;
        } else if (algo->accuracy == 3) {
            algo->state = MAG3D_STATE_RELIABLE;
            algo->output_ready = true;
        } else if (algo->accuracy == 2) {
            algo->state = MAG3D_STATE_SLIGHTLY_INTERFERED;
            MAG6D_DEBUG("slightly interfered from reliable");
        } else if (algo->accuracy <= 1) {
            algo->state = MAG3D_STATE_SLIGHTLY_INTERFERED;
            mag3d_sample_queue_init(&algo->queue);
            MAG6D_DEBUG("clear queue because slightly interfered from reliable");
        }

        // else if (algo->accuracy == 0)
        // {
        //     algo->state = MAG3D_STATE_INTERFERED;
        //     mag3d_sample_queue_init(&algo->queue);
        //     MAG6D_DEBUG("clear queue because interfered from reliable");
        //     algo->interfered = true;
        //     algo->output_ready = true;
        // }
        break;
    }
    }

#ifdef SNS_PRINTF_AP
    const char* states[] = {
        "MAG3D_STATE_INITIAL",
        "MAG3D_STATE_RESUMED",
        "MAG3D_STATE_SLIGHTLY_INTERFERED",
        "MAG3D_STATE_INTERFERED",
        "MAG3D_STATE_RELIABLE",
    };
    SNS_PRINTF_AP(HIGH, NULL, "%c bias=[%f, %f, %f] accuracy=%d, state=%s",
        algo->output_ready ? '*' : ' ', algo->curr_bias[0], algo->curr_bias[1], algo->curr_bias[2], algo->accuracy, states[algo->state]);
#endif
}

int mag3d_cal_algo_init(mag3d_cal_algo_t *algo, bool first, float bias[3], int accuracy)
{
    MAG6D_DEBUG("mag3d_cal_algo_init first=%d", first);
    algo->last_mag[0] = elem_zero;
    algo->last_mag[1] = elem_zero;
    algo->last_mag[2] = elem_zero;

    if (first) {
        mag3d_sample_queue_init(&algo->queue);
        algo->last_reliable_bias[0] = elem_zero;
        algo->last_reliable_bias[1] = elem_zero;
        algo->last_reliable_bias[2] = elem_zero;
        algo->last_reliable_ts_ns = 0LL;
        algo->curr_bias[0] = bias[0];
        algo->curr_bias[1] = bias[1];
        algo->curr_bias[2] = bias[2];
        algo->accuracy = accuracy;
        algo->curr_variance = elem_max;
        algo->curr_radius = elem_val(35);
    }

    algo->state = MAG3D_STATE_RESUMED;
    algo->output_ready = true;
    algo->interfered = false;

    for (int i = 0; i < 16; i++)
        algo->debug_info[i] = 0.0f;

    return 0;
}

int mag3d_cal_algo_deinit(mag3d_cal_algo_t *algo)
{
    (void)algo;
    return 0;
}

int mag3d_cal_algo_set_mag(mag3d_cal_algo_t *algo, elem *data, uint64_t ts_ns)
{
    // ignore continuous similar mag data
    elem distance_sqr = vector_distance_sqr(&algo->last_mag, (vector*)data);
    elem distance_thd = (algo->state == MAG3D_STATE_RELIABLE) ?
        MAG3D_MINIMUM_INPUT_MAG_DISTANCE_SLOW :
        MAG3D_MINIMUM_INPUT_MAG_DISTANCE_FAST;

    if (distance_sqr < elem_sqr(distance_thd)) {
        return 0;
    }

    VECTOR_COPY(algo->last_mag, data);
    MAG6D_DEBUG("|input=[%f, %f, %f]", data[0], data[1], data[2]);

    int last_accuracy = algo->accuracy;

    switch (algo->state) {
    case MAG3D_STATE_RELIABLE:
    case MAG3D_STATE_SLIGHTLY_INTERFERED:
        algo->interfered = mag6d_check_input_interfered(
                (vector*)data,
                &algo->last_reliable_bias,
                true);
        break;

    default:
        algo->interfered = mag6d_check_input_interfered(
                (vector*)data,
                &algo->curr_bias,
                false);
        break;
    }

    if (!algo->interfered) {
        // insert data into queue
        mag3d_sample_data *insert_data =
            mag3d_sample_queue_input(&algo->queue, (vector*)data, ts_ns, distance_thd);
        (void)insert_data;
        mag3d_sample_queue_print_ap(&algo->queue, insert_data, &algo->curr_bias, algo->curr_radius);

        // skip calculation if sample num is not enough
        if (algo->queue.size < MAG3D_SAMPLE_QUEUE_MIN_LENGTH) {
            return 0;
        }

        // do calculation
        mag3d_update_bias(algo);

        // estimate accuracy
        algo->accuracy = mag3d_estimate_accuracy(
                &algo->queue,
                &algo->curr_bias,
                algo->curr_radius,
                algo->curr_variance);

        // remember last reliable bias values
        if (algo->accuracy == 3) {
            VECTOR_COPY(algo->last_reliable_bias, algo->curr_bias);
            algo->last_reliable_ts_ns = ts_ns;
        }
    }

    // judge state, decide output_ready or interfered
    mag3d_process_state_transition(algo, ts_ns, last_accuracy);

    return 0;
}

int mag3d_cal_algo_get_bias(mag3d_cal_algo_t *algo, elem *out, int *accuracy, bool *result_changed, bool *interfered)
{
    switch(algo->state) {
    case MAG3D_STATE_RELIABLE:
        VECTOR_COPY(out, algo->curr_bias);
        *accuracy = algo->accuracy;
        break;

    case MAG3D_STATE_SLIGHTLY_INTERFERED:
        VECTOR_COPY(out, algo->last_reliable_bias);
        *accuracy = 3;
        break;

    case MAG3D_STATE_INTERFERED:
        VECTOR_COPY(out, algo->last_reliable_bias);
        *accuracy = algo->accuracy;
        break;

    default:
        VECTOR_COPY(out, algo->curr_bias);
        *accuracy = algo->accuracy;
        break;
    }

    if (result_changed != NULL) {
        *result_changed = algo->output_ready;
        algo->output_ready = false;
    }

    if (interfered != NULL) {
        *interfered = algo->interfered;
    }

    return 0;
}
