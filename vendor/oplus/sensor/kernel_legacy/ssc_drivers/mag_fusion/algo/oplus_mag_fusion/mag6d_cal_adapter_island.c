#include "fusion_api.h"
#include "mag6d_cal.h"
#include "mag3d_cal.h"

typedef struct hybrid_algo_s {
    mag6d_cal_algo_t   mag6d_algo;
    mag3d_cal_algo_t   mag3d_algo;
    // elem               last_output[3];
    // int                last_accuracy;
    // bool               last_interfered;
} hybrid_algo_t;

#define MAG6D(hybrid_algo) (&((hybrid_algo_t*)hybrid_algo)->mag6d_algo)
#define MAG3D(hybrid_algo) (&((hybrid_algo_t*)hybrid_algo)->mag3d_algo)

static hybrid_algo_t mag_fusion_algo;
//bool mag6d_cal_initialized=false;

int oplus_fusion_adapter_init(bool has_gyro, mag_fusion_output initial_result)
{
    UNUSED_VAR(has_gyro);
    void * algo = (void *)&mag_fusion_algo;

    mag3d_cal_algo_init(MAG3D(algo), true, initial_result.bias, initial_result.accuracy);
    mag6d_cal_algo_init(MAG6D(algo), true, initial_result.bias, initial_result.accuracy);

    if (initial_result.result_changed) {
        MAG3D(algo)->curr_bias[0] = initial_result.bias[0];
        MAG3D(algo)->curr_bias[1] = initial_result.bias[1];
        MAG3D(algo)->curr_bias[2] = initial_result.bias[2];
        MAG3D(algo)->accuracy = initial_result.accuracy;
        MAG3D(algo)->state = MAG3D_STATE_RESUMED;

        MAG6D(algo)->curr_bias[0] = initial_result.bias[0];
        MAG6D(algo)->curr_bias[1] = initial_result.bias[1];
        MAG6D(algo)->curr_bias[2] = initial_result.bias[2];
        MAG6D(algo)->realtime_accuracy = initial_result.accuracy;
        MAG6D(algo)->algo_state = MAG6D_STATE_RESUMED;
    }

    // ((hybrid_algo_t*)algo)->last_output[0] = initial_result.bias[0];
    // ((hybrid_algo_t*)algo)->last_output[1] = initial_result.bias[1];
    // ((hybrid_algo_t*)algo)->last_output[2] = initial_result.bias[2];
    // ((hybrid_algo_t*)algo)->last_accuracy = initial_result.accuracy;
    // ((hybrid_algo_t*)algo)->last_interfered = false;
    return 0;
}

int oplus_fusion_adapter_deinit()
{
    void * algo = (void *)&mag_fusion_algo;

    mag3d_cal_algo_deinit(MAG3D(algo));
    return mag6d_cal_algo_deinit(MAG6D(algo));
}

int oplus_fusion_adapter_start()
{
    void * algo = (void *)&mag_fusion_algo;

    mag3d_cal_algo_init(MAG3D(algo), false, NULL, 0);
    return mag6d_cal_algo_init(MAG6D(algo), false, NULL, 0);
}

int oplus_fusion_adapter_stop()
{
    void * algo = (void *)&mag_fusion_algo;

    mag3d_cal_algo_deinit(MAG3D(algo));
    return mag6d_cal_algo_deinit(MAG6D(algo));
}

int oplus_fusion_adapter_data_update(
    mag_fusion_data_input *input,
    mag_fusion_output *output)
{
    void * algo = (void *)&mag_fusion_algo;

    uint64_t ts_ns = SNS_TIME_CONVERT_TO_NS(input->timestamp);
    output->result_changed = 0;

    switch(input->type) {
    case MF_INPUT_TYPE_ACC:
        // do nothing
        return 0;

    case MF_INPUT_TYPE_MAG:
        mag3d_cal_algo_set_mag(MAG3D(algo), input->data, ts_ns);

        if (MAG3D(algo)->state == MAG3D_STATE_RELIABLE) {
            mag_track_reset(&MAG6D(algo)->curr_mag_track);
            gyro_rotation_reset(&MAG6D(algo)->curr_gyro_rotation);
        } else {
            mag6d_cal_algo_set_mag(MAG6D(algo), input->data, ts_ns);
        }

        break;

    case MF_INPUT_TYPE_GYR:
        if (MAG3D(algo)->state == MAG3D_STATE_RELIABLE) {
            // no gyro for mag3d
        } else {
            mag6d_cal_algo_set_gyro(MAG6D(algo), input->data, ts_ns);
        }

        break;

    default:
        return -1;
    }

    int accuracy = 0;
    bool result_changed = false;

    // [Combine 3d & 6d algo]
    //
    // There're two facts:
    //   1. 6d algo calibrates faster
    //   2. 3d algo is more accurate & steady at accuracy 3
    //
    // So there're rules:
    //   1. if 3d algo is at accuracy 3, use 3d bias
    //   2. if all 3d & 6d algo haven't reached accuracy 3 but already 2, use
    //      the average of the two algo. but if the deviation is too large,
    //      use 3d bias and report accuracy 0.
    //   3. if neither 3d nor 6d algo is ready (accuracy < 2), use 6d bias
    {
        float bias3d[3], bias6d[3];
        int accu3d, accu6d;
        bool valid3d, valid6d;
        bool interfered3d, interfered6d;
        mag3d_cal_algo_get_bias(MAG3D(algo), bias3d, &accu3d, &valid3d, &interfered3d);
        mag6d_cal_algo_get_bias(MAG6D(algo), bias6d, &accu6d, &valid6d, &interfered6d);

        if (valid3d || valid6d) {
            MAG6D_ERROR("mag3d (valid=%d, accuracy=%d, interfered=%d), mag6d (valid=%d, accuracy=%d, interfered=%d)",
                valid3d, accu3d, interfered3d, valid6d, accu6d, interfered6d);
        }

        if (accu3d == 3) {
            output->bias[0] = bias3d[0];
            output->bias[1] = bias3d[1];
            output->bias[2] = bias3d[2];
            accuracy = accu3d;
            result_changed = valid3d;

            if (result_changed) {
                MAG6D_INFO("mag3d bias [%f, %f, %f] accuracy %d valid %d",
                    output->bias[0], output->bias[1], output->bias[2], accuracy, result_changed);
            }

        } else if (accu6d >= 2 && accu3d >= 2) {
            if (vector_distance_sqr((vector*)bias3d, (vector*)bias6d) > elem_sqr(MAG6D_MIN_MAG_FIELD_RADIUS)) {
                // there must be something wrong
                output->bias[0] = bias3d[0];
                output->bias[1] = bias3d[1];
                output->bias[2] = bias3d[2];
                accuracy = 0;
                result_changed = valid3d;

                if (result_changed) {
                    MAG6D_INFO("mag3d bias [%f, %f, %f] accuracy %d valid %d",
                        output->bias[0], output->bias[1], output->bias[2], accuracy, result_changed);
                }

            } else {
                output->bias[0] = (bias3d[0] + bias6d[0]) / 2;
                output->bias[1] = (bias3d[1] + bias6d[1]) / 2;
                output->bias[2] = (bias3d[2] + bias6d[2]) / 2;
                accuracy = accu6d <= accu3d ? accu6d : accu3d;
                result_changed = valid3d || valid6d;

                if (result_changed) {
                    MAG6D_INFO("combined bias [%f, %f, %f] accuracy %d valid %d",
                        output->bias[0], output->bias[1], output->bias[2], accuracy, result_changed);
                }

            }
        } else {
            output->bias[0] = bias6d[0];
            output->bias[1] = bias6d[1];
            output->bias[2] = bias6d[2];
            accuracy = accu6d <= 2 ? accu6d : 2;
            result_changed = valid6d;

            if (result_changed) {
                MAG6D_INFO("mag6d bias [%f, %f, %f] accuracy %d valid %d",
                    output->bias[0], output->bias[1], output->bias[2], accuracy, result_changed);
            }

        }

        // hayden@ASTI add for GCET-6214 sometimes no accuracy 0 message sent when interfered by strong mag
        //   When strong mag has just approached, mag3d accuracy became 0 and then adapter choose mag6d result
        // as final result immediately, during which the accuracy 0 message was lost.
        //   And at this time mag6d was just enabled(mag6d was disabled for power saving when mag3d is accurate),
        // if mag6d has not accumulate any valid sample, it won't report any event, so there won't be any
        // accuracy 0 event reported.
        //   This change is totally for passing QE test, because mag6d will soon recover the accuracy with a
        // little rotation so the issue won't take effect on user experience
        if (interfered3d || interfered6d) {
            //   It is a workaround to check if mag3d is interfered and force set accuracy 0. It won't bring
            // more power consumption because mag3d will not notify 'interfered' after its state is already
            // MAG3D_STATE_INTERFERED.
            accuracy = 0;
            result_changed = true;
            MAG6D_ERROR("reset mag6d because interfered. interfered3d=%d, interfered6d=%d", interfered3d, interfered6d);
            mag6d_cal_algo_init(MAG6D(algo), false, NULL, 0);

            MAG6D_INFO("mag interfered bias [%f, %f, %f] accuracy %d valid %d",
                output->bias[0], output->bias[1], output->bias[2], accuracy, result_changed);

        }
    }

    switch(accuracy) {
    case 0:
        output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
        break;

    case 1:
        output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_LOW;
        break;

    case 2:
        output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_MEDIUM;
        break;

    case 3:
        output->accuracy = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
        break;
    }

    output->result_changed = result_changed;

    return 0;
}

int oplus_fusion_adapter_get_debug_info(elem * out)
{
    void * algo = (void *)&mag_fusion_algo;

    return mag6d_cal_algo_get_debug_info(MAG6D(algo), out);
}

mag_fusion_api oplus_fusion_api = {
    .algo_init = oplus_fusion_adapter_init,
    .algo_deinit = oplus_fusion_adapter_deinit,
    .algo_start = oplus_fusion_adapter_start,
    .algo_stop = oplus_fusion_adapter_stop,
    .algo_update = oplus_fusion_adapter_data_update,
    .algo_get_debug_info = oplus_fusion_adapter_get_debug_info,
};

