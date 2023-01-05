#include "mag6d_mag_track.h"

void mag_track_reset(mag_track *mt)
{
    mt->waiting_for_first_input = true;
    mt->count = 0;
}

void mag_track_input(mag_track *mt, elem *magdata, uint64_t ts_ns)
{
    // if the tracking has just begun, set initial values
    if(mt->waiting_for_first_input) {
        mt->waiting_for_first_input = false;
        VECTOR_COPY(mt->umag1, magdata);
        mt->start_ts_ns = ts_ns;
    }

    // update umag2 (end of track)
    VECTOR_COPY(mt->umag2, magdata);
    mt->count++;
}

elem mag_track_length_square(mag_track *mt)
{
    vector diff;
    vector_sub(&diff, &mt->umag1, &mt->umag2);
    return vector_len_sqr(&diff);
}

