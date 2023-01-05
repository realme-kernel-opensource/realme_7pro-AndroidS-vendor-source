#include "mag6d_common.h"

static void mag6d_insert_sort_elems(elem *arr, int len)
{
    for (int i = 1; i < len; i++) {
        for (int j = i; j > 0; j--) {
            if (arr[j] < arr[j - 1]) {
                elem temp = arr[j];
                arr[j] = arr[j - 1];
                arr[j - 1] = temp;
            } else {
                break;
            }
        }
    }
}

static int mag6d_qsort_partition_elems(elem *arr, int len)
{
    int l = 0;
    int r = len - 1;
    elem pivot = arr[l];

    while(l != r) {
        for(; l < r && arr[r] >= pivot; r--);

        if(l != r) {
            arr[l] = arr[r];
        }

        for(; l < r && arr[l] <= pivot; l++);

        if(l != r) {
            arr[r] = arr[l];
        }
    }

    arr[l] = pivot;
    return l;
}

void mag6d_qsort_elems(elem *arr, int len)
{
    if(len <= 8) {
        mag6d_insert_sort_elems(arr, len);
        return;
    }

    int middle = mag6d_qsort_partition_elems(arr, len);

    // sort left of selected middle element
    mag6d_qsort_elems(arr, middle);

    // sort right of selected middle element
    mag6d_qsort_elems(arr + middle + 1, len - middle - 1);
}

bool mag6d_check_input_interfered(vector *umag, vector *current_bias, bool bias_reliable)
{
    // there're two situations when interference detection is activated:

    // 1. when algorithm has given a approximate bias (realtime_accuracy > 0),
    //    check if the magnetic field strength is extremely high (threshold is
    //    1500uT, when a magnet comes there should be a field higher than
    //    2000uT)
    elem mag_field_strength_sqr = vector_len_sqr(umag);
    bool absolutely_interfered = mag_field_strength_sqr > elem_sqr(MAG6D_ABSOLUTE_MAG_INTERFERE_TRIGGER);

    if (absolutely_interfered) {
        MAG6D_ERROR("absolutely_interfered. mag_field = %d/1000, bias=[%d, %d, %d]/1000",
            (int)(sqrtf(mag_field_strength_sqr) * 1000),
            (int)((*current_bias)[0] * 1000),
            (int)((*current_bias)[1] * 1000),
            (int)((*current_bias)[2] * 1000));
        return true;
    }

    // 2. when algorithm is reliable (state is RELIABLE), check if the magnetic
    //    field strength is much higher than normal (normally 25~45uT, set
    //    threshold to 300uT)
    elem radius_sqr = vector_distance_sqr(current_bias, umag);
    bool relatively_interfered = bias_reliable &&
        (radius_sqr > elem_sqr(MAG6D_MAG_TRACK_INTERFERE_TRIGGER));

    if (relatively_interfered) {
        MAG6D_ERROR("relatively_interfered. radius = %d/1000, bias=[%d, %d, %d]/1000",
            (int)(sqrtf(radius_sqr) * 1000),
            (int)((*current_bias)[0] * 1000),
            (int)((*current_bias)[1] * 1000),
            (int)((*current_bias)[2] * 1000));
        return true;
    }

    return false;
}
