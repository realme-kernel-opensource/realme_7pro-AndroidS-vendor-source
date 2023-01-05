#pragma once
#include "mag6d_common.h"

//   accumulated rotation of inputed gyro data, rotates in opposite direction in
// order to simulate the rotation of mag data
typedef struct gyro_rotation {
    bool waiting_for_first_input;
    rotmat rotation;
    uint64_t last_ts_ns;
} gyro_rotation;

// reset gyro rotation to initial state(rotation to unit matrix)
void gyro_rotation_reset(gyro_rotation *gr);

// rotate the rotation matrix at opposite direction of gyro data
void gyro_rotation_input(gyro_rotation *gr, elem *gyrodata, uint64_t ts_ns);

// get angle of the rotation
elem gyro_rotation_angle(gyro_rotation *gr);

