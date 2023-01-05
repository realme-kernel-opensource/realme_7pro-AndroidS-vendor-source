#include "mag6d_gyro_rotation.h"

void gyro_rotation_reset(gyro_rotation *gr)
{
    gr->waiting_for_first_input = true;
}

void gyro_rotation_input(gyro_rotation *gr, elem *gyrodata, uint64_t ts_ns)
{
    if(gr->waiting_for_first_input) {
        rotmat_assign_eye(&gr->rotation);
        gr->last_ts_ns = ts_ns;
        gr->waiting_for_first_input = false;
        return;
    }

    int32_t delta_ts_ns = (int32_t)(ts_ns - gr->last_ts_ns);
    elem delta_ts_sec = elem_div(elem_val((int32_t)(delta_ts_ns / 1000)), elem_val(1000000));
    euler e = {
        elem_mul(gyrodata[0], -delta_ts_sec),
        elem_mul(gyrodata[1], -delta_ts_sec),
        elem_mul(gyrodata[2], -delta_ts_sec),
    };
    rotmat d;
    euler_to_rotmat(&e, &d);
    // rotation = rotation * delta
    rotmat_mul(&gr->rotation, &d, &gr->rotation);
    gr->last_ts_ns = ts_ns;
}

elem gyro_rotation_angle(gyro_rotation *gr)
{
    return rotmat_angle(&gr->rotation);
}