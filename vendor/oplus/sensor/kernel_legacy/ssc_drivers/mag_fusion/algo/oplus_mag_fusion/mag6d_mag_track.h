#pragma once
#include "mag6d_common.h"

//   a track of magnet data point, recording the start point and end point
typedef struct mag_track {
    bool waiting_for_first_input;
    vector umag1; // start
    vector umag2; // end
    int count; // inputed data count
    uint64_t start_ts_ns; // start timestamp
} mag_track;

// reset mag_track to zero values
void mag_track_reset(mag_track *mt);

// input mag data to mag2
void mag_track_input(mag_track *mt, elem *magdata, uint64_t ts_ns);

// calc the square of the length between umag2 and mag2
elem mag_track_length_square(mag_track *mt);
