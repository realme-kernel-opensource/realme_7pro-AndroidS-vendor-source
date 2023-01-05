#include "mag6d_cal.h"

void mag6d_sample_data_create(mag6d_sample_data *data, mag_track *mt, gyro_rotation *gr)
{
    VECTOR_COPY(data->umag1, mt->umag1);
    VECTOR_COPY(data->umag2, mt->umag2);
    rotmat_copy(&data->rotation, &gr->rotation);
    data->ts_ns = mt->start_ts_ns;
#if MAG6D_SAMPLE_DATA_PRE_CALC_LOSS_GRAD
    {
        // precalc:
        //  vec1 = R x U1 - U2
        vector temp_vec;
        rotmat_rotate(&temp_vec, &data->rotation, &data->umag1);
        vector_sub(&data->__precalc_vec1, &temp_vec, &data->umag2);
        MAG6D_DEBUG("vec1=[%d %d %d]/1000", f1000(data->__precalc_vec1[0]), f1000(data->__precalc_vec1[1]), f1000(data->__precalc_vec1[2]));
    }
    {
        // precalc:
        //  mat1 = E - R
        //  mat2 = mat1^T x 2
        rotmat unit_mat = rotmat_init_unit;
        rotmat_sub(&data->__precalc_mat1, &unit_mat, &data->rotation);
        rotmat_transpose(&data->__precalc_mat2, &data->__precalc_mat1);
        rotmat_scale(&data->__precalc_mat2, &data->__precalc_mat2, elem_val(2));
    }
#endif
    data->last_loss = elem_zero;
    data->prev = NULL;
    data->next = NULL;
    data->in_use = true;
}

// calculate loss function and the gradient of loss function at inputed bias for certain sample data
static void mag6d_loss_and_loss_gradient(vector *bias, mag6d_sample_data *data, elem *loss, vector *grad)
{
    vector diff;

#if MAG6D_SAMPLE_DATA_PRE_CALC_LOSS_GRAD
    // precalc:
    //  mat1 = E - R
    //  vec1 = R x U1 - U2

    // diff = mat1 x bias + vec1
    rotmat_rotate(&diff, &data->__precalc_mat1, bias);
    vector_add(&diff, &data->__precalc_vec1, &diff);
#else
    // calibrated mag1 = uncalibrated mag1 - bias
    vector cmag1;
    vector_sub(&cmag1, &data->umag1, bias);

    // expected calibrated mag2 = rotation x uncalibrated mag1
    vector ecmag2;
    rotmat_rotate(&ecmag2, &data->rotation, &cmag1);

    // calibrated mag2 = uncalibrated mag2 - bias
    vector cmag2;
    vector_sub(&cmag2, &data->umag2, bias);

    // diff = expected calibrated mag2 - calibrated mag2
    vector_sub(&diff, &ecmag2, &cmag2);
#endif

    if(loss) {
        // loss = module(diff)
        *loss = vector_len_sqr(&diff);
        data->last_loss = *loss;
    }

    if(!grad) {
        return;
    }

#if MAG6D_SAMPLE_DATA_PRE_CALC_LOSS_GRAD
    // precalc:
    //  mat2 = (E - R)^T x 2

    // grad = mat2 x diff
    rotmat_rotate(grad, &data->__precalc_mat2, &diff);
#else
    // grad = (E - R)^T * diff * 2

    // mat1 = E - R
    rotmat unit = rotmat_init_unit;
    rotmat mat1;
    rotmat_sub(&mat1, &unit, &data->rotation);

    // mat2 = mat1^T
    rotmat mat2;
    rotmat_transpose(&mat2, &mat1);

    // grad = mat2 * diff
    rotmat_rotate(grad, &mat2, &diff);

    // grad = grad * 2
    vector_scale(grad, grad, 2);
#endif
}

// calculate cost function and the gradient of cost function at inputed bias for the sample dataset
static void mag6d_cost_and_cost_gradient(vector *bias, mag6d_sample_queue* queue, elem *cost, vector *cost_grad)
{
    elem ret_cost = 0;
    vector ret_grad = vector_init_zero;
    elem loss;
    vector loss_grad;

    if(cost_grad) {
        MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
            mag6d_loss_and_loss_gradient(bias, data, &loss, &loss_grad);
            ret_cost += loss;
            ret_grad[0] += loss_grad[0];
            ret_grad[1] += loss_grad[1];
            ret_grad[2] += loss_grad[2];
        });
        VECTOR_COPY(*cost_grad, ret_grad);
    } else {
        MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
            mag6d_loss_and_loss_gradient(bias, data, &loss, NULL);
            ret_cost += loss;
        });
    }

    if(cost) {
        *cost = ret_cost;
    }
}

// process a set of gradient-descent calculation
// return the vector length of current gradiant
static elem mag6d_gradient_descent_update_bias(
    vector *bias,
    mag6d_sample_queue *queue,
    elem init_step_len,
    elem max_step_num,
    elem min_grad_len_sqr)
{
    elem step_len = init_step_len;
    elem cost;
    vector cost_grad;
    elem grad_len_sqr;
    bool moved = true;
    int step_num = 0;

    for(step_num = 0; step_num < max_step_num; step_num++) {
        if(moved) {
            mag6d_cost_and_cost_gradient(bias, queue, &cost, &cost_grad);
            grad_len_sqr = vector_len_sqr(&cost_grad);

            if(grad_len_sqr <= min_grad_len_sqr) {
                break;
            }
        }

        vector next_bias = {
            (*bias)[0] - elem_mul(cost_grad[0], step_len),
            (*bias)[1] - elem_mul(cost_grad[1], step_len),
            (*bias)[2] - elem_mul(cost_grad[2], step_len)
        };
        elem next_cost;
        mag6d_cost_and_cost_gradient(&next_bias, queue, &next_cost, NULL);

        if(next_cost < cost) {
            VECTOR_COPY(*bias, next_bias);
            step_len = elem_mul(step_len, MAG6D_ITERATION_STEP_INCREASE_FACTOR);
            moved = true;
        } else {
            step_len = elem_mul(step_len, MAG6D_ITERATION_STEP_DECREASE_FACTOR);
            moved = false;
        }

        cost = next_cost;
    }

    MAG6D_DEBUG("gradient-descent times:%d", step_num);
    return grad_len_sqr;
}

int mag6d_remove_inaccurate_samples(mag6d_sample_queue *queue)
{
    elem sum_loss = elem_zero;
    MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
        sum_loss += data->last_loss;
    });
    elem average_loss = elem_div(sum_loss, elem_val(queue->size));
    elem fine_loss_thd = elem_mul(average_loss, elem_val(2));

    MAG6D_SAMPLE_QUEUE_FOREACH_VALID_ORDERED_BY_NEXT(data, queue->head, NULL, {
        if(data->last_loss > fine_loss_thd && queue->size > MAG6D_SAMPLE_QUEUE_MIN_CALC_LENGTH)
        {
            mag6d_sample_queue_remove(queue, data);
            return 1;
        }
    });
    return 0;

    // int remove_count = 0;
    // MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
    //     if(data->last_loss > fine_loss_thd && queue->size > MAG6D_SAMPLE_QUEUE_MIN_CALC_LENGTH){
    //         mag6d_sample_queue_remove(queue, data);
    //         remove_count += 1;
    //     }
    // });
    // return remove_count;
}

// calc the accuracy of bias by sample data range, BGD cost, and radiuses variance
static int mag6d_calc_accuracy(mag6d_cal_algo_t *algo, vector *bias, elem grad_len_sqr, elem radius_variance_rate)
{
    mag6d_sample_queue *queue = &algo->queue;
    int yaw_accuracy = -1;
    int pitch_accuracy = -1;
    int algo_accuracy = -1;
    int variance_accuracy = -1;
    MAG6D_DEBUG("mag6d_calc_accuracy: enter");

    // get accuracy by range of yaw
    {
        // calc yaw for every umag1
        elem yaw_list[MAG6D_SAMPLE_QUEUE_MAX_LENGTH];
        int idx = 0;
        MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
            vector cmag;
            vector_sub(&cmag, &data->umag1, bias);
            yaw_list[idx++] = vector_yaw(&cmag);
            // MAG6D_DEBUG("mag6d_calc_accuracy: yaw[%d]=%d.%03d", __foreach_idx, fp1000(yaw_list[__foreach_idx]*180/PI));
        });

        // find biggest interval in yaw list
        mag6d_qsort_elems(yaw_list, queue->size);
        elem max_yaw_interval = yaw_list[0] - yaw_list[queue->size - 1] + TWO_PI;

        for(int i = 1; i < queue->size; i++) {
            elem interval = yaw_list[i] - yaw_list[i - 1];

            if(max_yaw_interval < interval) {
                max_yaw_interval = interval;
            }
        }

        // the range of yaw is complementary to the biggest interval
        elem yaw_range = TWO_PI - max_yaw_interval;
        algo->debug_info[0] = yaw_range;

        // judge accuracy
        yaw_accuracy = yaw_range > MAG6D_YAW_RANGE_ACCURACY_HIGH_THRESHOLD ? 3 :
            yaw_range > MAG6D_YAW_RANGE_ACCURACY_MED_THRESHOLD  ? 2 :
            yaw_range > MAG6D_YAW_RANGE_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;
        algo->debug_info[1] = yaw_accuracy;

        MAG6D_DEBUG("mag6d_calc_accuracy: yaw_range=%d.%03d, yaw_accuracy=%d", fp1000(yaw_range * 180 / PI), yaw_accuracy);
    }

    // get accuracy by range of pitch
    {
        // calc max&min z axis
        vector *max_z_umag = &queue->data[0].umag1;
        vector *min_z_umag = &queue->data[0].umag1;
        MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
            // vector temp_cmag;
            // elem pitch = vector_pitch(vector_sub(&temp_cmag, &data->umag1, bias));
            // MAG6D_DEBUG("mag6d_calc_accuracy: pitch[%d]=%d.%d, z=%d.%d", __foreach_idx, fp1000(pitch*180/PI), fp1000(data->umag1[2]));
            if((*max_z_umag)[2] < data->umag1[2])
            {
                max_z_umag = &data->umag1;
            } else if((*min_z_umag)[2] > data->umag1[2])
            {
                min_z_umag = &data->umag1;
            }
        });

        // the range of pitch depends on max&min pitch angle
        vector temp_cmag;
        elem max_pitch = vector_pitch(vector_sub(&temp_cmag, max_z_umag, bias));
        // MAG6D_DEBUG("mag6d_calc_accuracy: max_pitch=%d.%d", fp1000(max_pitch*180/PI));
        elem min_pitch = vector_pitch(vector_sub(&temp_cmag, min_z_umag, bias));
        // MAG6D_DEBUG("mag6d_calc_accuracy: min_pitch=%d.%d", fp1000(min_pitch*180/PI));
        elem pitch_range = max_pitch - min_pitch;
        algo->debug_info[2] = pitch_range;

        // judge accuracy
        pitch_accuracy = pitch_range > MAG6D_PITCH_RANGE_ACCURACY_HIGH_THRESHOLD ? 3 :
            pitch_range > MAG6D_PITCH_RANGE_ACCURACY_MED_THRESHOLD  ? 2 :
            pitch_range > MAG6D_PITCH_RANGE_ACCURACY_LOW_THRESHOLD  ? 1 :
            0;
        algo->debug_info[3] = pitch_accuracy;

        MAG6D_DEBUG("mag6d_calc_accuracy: pitch_range=%d.%03d, pitch_accuracy=%d", fp1000(pitch_range * 180 / PI), pitch_accuracy);
    }

    // get accuracy by the square of cost gradient length
    algo_accuracy = grad_len_sqr < MAG6D_COST_GRADIENT_ACCURACY_HIGH_THRESHOLD ? 3 :
        grad_len_sqr < MAG6D_COST_GRADIENT_ACCURACY_MED_THRESHOLD  ? 2 :
        grad_len_sqr < MAG6D_COST_GRADIENT_ACCURACY_LOW_THRESHOLD  ? 1 :
        0;
    algo->debug_info[4] = grad_len_sqr;
    algo->debug_info[5] = algo_accuracy;
    MAG6D_DEBUG("mag6d_calc_accuracy: grad_len_sqr=%d.%03d, algo_accuracy=%d", fp1000(grad_len_sqr), algo_accuracy);

    // get accuracy by the variance of radiuses
    variance_accuracy = radius_variance_rate < MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_HIGH_THRESHOLD ? 3 :
        radius_variance_rate < MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_MED_THRESHOLD  ? 2 :
        radius_variance_rate < MAG6D_RADIUS_VARIANCE_RATE_ACCURACY_LOW_THRESHOLD  ? 1 :
        0;
    algo->debug_info[6] = radius_variance_rate;
    algo->debug_info[7] = variance_accuracy;
    MAG6D_DEBUG("mag6d_calc_accuracy: radius_variance_rate=%d.%03d, variance_accuracy=%d", fp1000(radius_variance_rate), variance_accuracy);

    // return the lowest value of the accuracies above
    {
        int accuracies[] = {yaw_accuracy, pitch_accuracy, algo_accuracy, variance_accuracy};
        const int arr_size = sizeof(accuracies) / sizeof(accuracies[0]);
        int min = 3;

        for(int i = 0; i < arr_size; i++) {
            if(accuracies[i] < min) {
                min = accuracies[i];
            }
        }

        return min;
    }
}

// calc the average radius and the variance of the radiuses calculated at inputed bias for each sample data in the dataset
static void mag6d_average_radius_and_variance(vector *bias, mag6d_sample_queue *queue, elem *out_radius, elem *out_variance)
{
    elem radiuses[MAG6D_SAMPLE_QUEUE_MAX_LENGTH * 2];
    elem average_radius;
    int size = queue->size * 2;
    memset((void *)radiuses, 0, sizeof(radiuses));
    memset((void *)&average_radius, 0, sizeof(average_radius));
    // calc average radius
    {
        elem radius_sum = elem_zero;
        int i = 0;
        MAG6D_SAMPLE_QUEUE_FOREACH_VALID_DATA(data, queue, {
            elem radius;
            radius = vector_distance(bias, &data->umag1);
            radiuses[i++] = radius;
            radius_sum += radius;
            radius = vector_distance(bias, &data->umag2);
            radiuses[i++] = radius;
            radius_sum += radius;
        })
        average_radius = elem_div(radius_sum, elem_val(size));
    }

    if (out_radius) {
        *out_radius = average_radius;
    }

    if (out_variance) {
        elem sqr_sum = elem_zero;

        for (int i = 0; i < size; i++) {
            elem diff = radiuses[i] - average_radius;
            sqr_sum += elem_mul(diff, diff);
        }

        *out_variance = elem_div(sqr_sum, elem_val(size));
    }
}

//   detect if external magnet field is static.
//   idea: respectively calculate rotation angle of this input from mag_track and gyro_rotation,
// and check the difference between the two angles.
//   normally the rotation angle of mag shouldn't be bigger than gyro(imagine there's a person on
// equator comparing with another on somewhere else on the earth, the former must have traveled
// further than the other during one same period of time)
//   if the angle from mag is bigger than 3 times the one from gyro, this sample will be supposed
// to be invalid.
static bool mag6d_check_mag_and_gyro_angle_is_matched(mag6d_cal_algo_t *algo)
{
    elem mag_length_sqr = mag_track_length_square(&algo->curr_mag_track);

    // calculate restricted mag radius
    elem mag_radius = algo->curr_radius == elem_zero ?                 MAG6D_DEFAULT_MAG_FIELD_RADIUS :
        algo->curr_radius < MAG6D_MIN_MAG_FIELD_RADIUS ? MAG6D_MIN_MAG_FIELD_RADIUS :
        algo->curr_radius > MAG6D_MAX_MAG_FIELD_RADIUS ? MAG6D_MAX_MAG_FIELD_RADIUS :
        algo->curr_radius;

    // calculate rotation angle by mag
    // cos(theta)=(a^2 + b^2 - c^2)/(2ab)
    elem mag_radius_sqr = elem_sqr(mag_radius);
    elem cos_val = elem_div(
            mag_radius_sqr + mag_radius_sqr - mag_length_sqr,
            elem_mul(elem_val(2), mag_radius_sqr)
        );

    // avoid acos(-1.***) or acos(1.***) exception
    cos_val = cos_val < elem_val(-1) ? elem_val(-1) :
        cos_val > elem_val(1)  ? elem_val(1) :
        cos_val;
    elem mag_angle = elem_acos(cos_val);

    // calculate rotation angle by gyro
    elem gyro_angle = gyro_rotation_angle(&algo->curr_gyro_rotation);
    // MAG6D_DEBUG("gyro_rotation=[0]%d.%03d [1]%d.%03d [2]%d.%03d", fp1000(algo->curr_gyro_rotation.rotation[0]), fp1000(algo->curr_gyro_rotation.rotation[1]), fp1000(algo->curr_gyro_rotation.rotation[2]));
    // MAG6D_DEBUG("gyro_rotation=[3]%d.%03d [4]%d.%03d [5]%d.%03d", fp1000(algo->curr_gyro_rotation.rotation[3]), fp1000(algo->curr_gyro_rotation.rotation[4]), fp1000(algo->curr_gyro_rotation.rotation[5]));
    // MAG6D_DEBUG("gyro_rotation=[6]%d.%03d [7]%d.%03d [8]%d.%03d", fp1000(algo->curr_gyro_rotation.rotation[6]), fp1000(algo->curr_gyro_rotation.rotation[7]), fp1000(algo->curr_gyro_rotation.rotation[8]));
    MAG6D_DEBUG("try inserting sample data: mag_length_sqr=%d.%03d, mag_radius=%d.%03d, mag_angle=%d.%03d gyro_angle=%d.%03d",
        fp1000(mag_length_sqr), fp1000(mag_radius), fp1000(mag_angle), fp1000(gyro_angle));

    bool matches = mag_angle <= elem_mul(gyro_angle, elem_val(3));

    if (!matches) {
        MAG6D_DEBUG("judging as interfered: mag_angle=%d.%03d gyro_angle=%d.%03d", fp1000(mag_angle), fp1000(gyro_angle));
    }

    return matches;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int mag6d_cal_algo_init(mag6d_cal_algo_t *algo, bool first, float bias[3], int accuracy)
{
    if(algo == NULL) {
        return -1;
    }

    MAG6D_DEBUG("started: first=%d", first);
    mag_track_reset(&algo->curr_mag_track);
    gyro_rotation_reset(&algo->curr_gyro_rotation);
    mag6d_sample_queue_reset(&algo->queue);
    algo->curr_grad_len_sqr = elem_max;
    algo->curr_radius = elem_zero;

    if(first) {
        algo->curr_bias[0] = bias[0];
        algo->curr_bias[1] = bias[1];
        algo->curr_bias[2] = bias[2];
        algo->realtime_accuracy = accuracy;

        for(int i = 0; i < 16; i++) {
            algo->debug_info[i] = elem_zero;
        }
    }

    algo->algo_state = MAG6D_STATE_RESUMED;

    // report value instantly on enabled
    algo->output_ready = true;
    return 0;
}

int mag6d_cal_algo_deinit(mag6d_cal_algo_t *algo)
{
    (void)algo;
    return 0;
}

int mag6d_cal_algo_set_mag(mag6d_cal_algo_t *algo, elem *data, uint64_t ts_ns)
{
    vector *umag = (vector*)data;

    algo->interfered = mag6d_check_input_interfered(
            umag,
            &algo->curr_bias,
            algo->algo_state == MAG6D_STATE_RELIABLE);

    if (algo->interfered && algo->algo_state != MAG6D_STATE_INTERFERED) {
        //   abnormal mag data means that external interference sources may have magnetize
        // some metal parts inside the phone, which leads to the change of mag sensor bias,
        // so stored samples may be useless.
        MAG6D_ERROR("reset mag6d because interfered. algo->interfered=%d, algo->algo_state=%d", algo->interfered, algo->algo_state);
        mag6d_cal_algo_init(algo, false, NULL, 0);
        algo->algo_state = MAG6D_STATE_INTERFERED;
        algo->realtime_accuracy = 0;
        algo->output_ready = true;
        return 0;
    }

    //   if the tracking has lasted for too long, reset gyro_rotation with
    // mag_track to avoid accumulated gyro deviation
    //   note: when curr_mag_track is just initialized, start_ts_ns will be
    // 0 and there shouldn't be reset action to mag_track or gyro_rotation.
    bool tracking_for_too_long = ts_ns - algo->curr_mag_track.start_ts_ns > MAG6D_MAG_TRACK_TIMEOUT_NS;

    if(tracking_for_too_long && !algo->curr_mag_track.waiting_for_first_input) {
        MAG6D_DEBUG("mag_track initial or tracking_for_too_long");
        mag_track_reset(&algo->curr_mag_track);
        gyro_rotation_reset(&algo->curr_gyro_rotation);
    }

    // input umag data to mag_track
    mag_track_input(&algo->curr_mag_track, data, ts_ns);

    // check if mag_track is long enough for a calculation
    elem mag_length_sqr = mag_track_length_square(&algo->curr_mag_track);
    // MAG6D_DEBUG("mag_length_sqr = %d/1000", (int)(1000*mag_length_sqr));
    // MAG6D_DEBUG("mag1=[ %d %d %d ]/1000",
    //     (int)(1000 * algo->curr_mag_track.umag1[0]),
    //     (int)(1000 * algo->curr_mag_track.umag1[1]),
    //     (int)(1000 * algo->curr_mag_track.umag1[2]));
    // MAG6D_DEBUG("mag2=[ %d %d %d ]/1000",
    //     (int)(1000 * algo->curr_mag_track.umag2[0]),
    //     (int)(1000 * algo->curr_mag_track.umag2[1]),
    //     (int)(1000 * algo->curr_mag_track.umag2[2]));
    bool mag_track_long_enough = mag_length_sqr > MAG6D_MAG_TRACK_LENGTH_SQR_INPUT_TRIGGER;

    // check if mag_track sample amount is enough for a calculation
    bool mag_sample_num_too_many = algo->curr_mag_track.count >= MAG6D_MAG_TRACK_MAX_POINT_NUM;

    // insert sample into sample_queue and execute calculation
    //   when mag_track has counted too many samples but too short, gyro_rotation
    // may have accumulated too much deviation by floating point calculation error.
    if (mag_sample_num_too_many) {
        mag_track_reset(&algo->curr_mag_track);
        gyro_rotation_reset(&algo->curr_gyro_rotation);
    }
    //   if there's a long mag_track without a period of time that is too lont, it
    // can be recognized as a valid input.
    else if(mag_track_long_enough) {
        if (!mag6d_check_mag_and_gyro_angle_is_matched(algo)) {
            // reset current track and rotation
            mag_track_reset(&algo->curr_mag_track);
            gyro_rotation_reset(&algo->curr_gyro_rotation);
            return 0;
        }

        // save sample data and precalc(if precalc enabled)
        mag6d_sample_queue_input(&algo->queue, &algo->curr_mag_track, &algo->curr_gyro_rotation);

        // reset current track
        mag_track_reset(&algo->curr_mag_track);
        gyro_rotation_reset(&algo->curr_gyro_rotation);

        // do calibration calculation if sample count is big enough
        if(algo->queue.size >= MAG6D_SAMPLE_QUEUE_MIN_CALC_LENGTH) {
            // this function will print tons of log lines. only for debug when necessary
            // mag6d_sample_queue_dump_print(&algo->queue);

            // do BGD calculation
            algo->curr_grad_len_sqr = mag6d_gradient_descent_update_bias(
                    &algo->curr_bias, &algo->queue,
                    MAG6D_ITERATION_INIT_STEP_LENGTH,
                    MAG6D_ITERATION_MAX_TIMES_PER_EPOCH,
                    MAG6D_COST_GRAD_THD_FOR_STOPPING_ITERATION);

            // do sample check & auto remove for the situation that algo is recovering from interfered state
            if(algo->queue.size >= MAG6D_SAMPLE_QUEUE_MIN_CALC_LENGTH) {
                int remove_count = mag6d_remove_inaccurate_samples(&algo->queue);
                MAG6D_DEBUG("---- removed %d inaccurate samples, remain %d ----", remove_count, algo->queue.size);

                if(remove_count > 0) {
                    // do another BGD after mag6d_sample_queue changed
                    algo->curr_grad_len_sqr = mag6d_gradient_descent_update_bias(
                            &algo->curr_bias, &algo->queue,
                            MAG6D_ITERATION_INIT_STEP_LENGTH,
                            MAG6D_ITERATION_MAX_TIMES_PER_EPOCH,
                            MAG6D_COST_GRAD_THD_FOR_STOPPING_ITERATION);
                }
            }

            elem variance;
            mag6d_average_radius_and_variance(
                &algo->curr_bias, &algo->queue,
                &algo->curr_radius, &variance);
            MAG6D_DEBUG("updated bias=[%d %d %d]/1000, radius=%d/1000, variance=%d/1000, variance_norm=%d/1000",
                f1000(algo->curr_bias[0]),
                f1000(algo->curr_bias[1]),
                f1000(algo->curr_bias[2]),
                f1000(algo->curr_radius),
                f1000(variance),
                f1000(elem_div(variance, algo->curr_radius)));

            int accuracy = mag6d_calc_accuracy(
                    algo,
                    &algo->curr_bias,
                    algo->curr_grad_len_sqr,
                    elem_div(variance, algo->curr_radius));

            if (algo->realtime_accuracy < 3 && accuracy == 3) {
                algo->algo_state = MAG6D_STATE_RELIABLE;
            }

            algo->realtime_accuracy = accuracy;
            MAG6D_DEBUG("mag6d_calc_accuracy() -> %d, algo->algo_state=%d", accuracy, algo->algo_state);
        }

        algo->output_ready = true;
    }

    return 0;
}

int mag6d_cal_algo_set_gyro(mag6d_cal_algo_t *algo, elem *data, uint64_t ts_ns)
{
    //   gyro value should be in a limited range. If gyro value is too large(like a rapid rotation in hand)
    // the rotation may accumulate large error.
    gyro_rotation_input(&algo->curr_gyro_rotation, data, ts_ns);
    return 0;
}

int mag6d_cal_algo_get_bias(mag6d_cal_algo_t *algo, elem *out_bias, int *out_accuracy, bool *result_changed, bool *out_interfered)
{
    if (out_bias != NULL) {
        out_bias[0] = algo->curr_bias[0];
        out_bias[1] = algo->curr_bias[1];
        out_bias[2] = algo->curr_bias[2];
    }

    if (out_accuracy != NULL) {
        if(algo->algo_state == MAG6D_STATE_RELIABLE) {
            *out_accuracy = 3;
        } else {
            *out_accuracy = algo->realtime_accuracy;
        }
    }

    if (result_changed != NULL) {
        *result_changed = algo->output_ready;
        algo->output_ready = false;
    }

    if (out_interfered != NULL) {
        *out_interfered = algo->interfered;
    }

    return 0;
}

int mag6d_cal_algo_get_debug_info(mag6d_cal_algo_t *algo, elem *out)
{
    for(int i = 0; i < 16; i++) {
        out[i] = algo->debug_info[i];
    }

    return 0;
}
