#include "math_util.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "model_parameter.h"
#include "kiss_fft.h"

#include "model_parameter.h"

#include "sns_island_util.h"
#include "sns_island.h"
/************************************************************************/
/*                           Matrix Operations                          */
/************************************************************************/

matrix *motion_recognition_matrix_create(struct motion_recognition_persistent_state *persistent, matrix *mat, int row, int col)
{
    int i;

    mat->element = persistent->malloc(row * sizeof(float *));
    for (i = 0; i < row; i++) {
        if (!sns_island_is_island_ptr((intptr_t)mat->element)) {
            SNS_ISLAND_EXIT();
            mat->element[i] = persistent->malloc( col * sizeof(float));
        } else {
            mat->element[i] = persistent->malloc( col * sizeof(float));
        }
    }

    mat->row = row;
    mat->col = col;

    return mat;
}

void motion_recognition_matrix_delete(struct motion_recognition_persistent_state *persistent, matrix *mat)
{
    int i;

    for (i = 0; i < mat->row; i++) {
        persistent->free(mat->element[i]);
    }
    persistent->free(mat->element);
}

float motion_recognition_matrix_get(matrix *mat, int pos_r, int pos_c)
{
    float value = 0;
    if (pos_r < mat->row && pos_c < mat->col)
    {
        if (!sns_island_is_island_ptr((intptr_t)mat->element) ||
            !sns_island_is_island_ptr((intptr_t)mat->element[pos_r])) {
            SNS_ISLAND_EXIT();
            value = mat->element[pos_r][pos_c];
        } else {
            value = mat->element[pos_r][pos_c];
        }
    }
    return value;
}

void motion_recognition_matrix_set(matrix *mat, int pos_r, int pos_c, float value)
{
    if (pos_r < mat->row && pos_c < mat->col) {
        if (!sns_island_is_island_ptr((intptr_t)mat->element) ||
            !sns_island_is_island_ptr((intptr_t)mat->element[pos_r])) {
            SNS_ISLAND_EXIT();
            mat->element[pos_r][pos_c] = value;
        } else {
            mat->element[pos_r][pos_c] = value;
        }
    }
}

matrix *motion_recognition_matrix_zeros(matrix *mat)
{
    int row, col;

    for (row = 0; row < mat->row; row++) {
        for (col = 0; col < mat->col; col++) {
            motion_recognition_matrix_set(mat, row, col, 0.0f);
        }
    }

    return mat;
}

/* dst = src1 - src2 */
matrix *motion_recognition_matrix_sub(matrix *src1, matrix *src2, matrix *dst)
{
    int row, col;

    for (row = 0; row < src1->row; row++) {
        for (col = 0; col < src1->col; col++) {
            float matrix_value1 = motion_recognition_matrix_get(src1, row, col);
            float matrix_value2 = motion_recognition_matrix_get(src2, row, col);
            motion_recognition_matrix_set(dst, row, col, matrix_value1 - matrix_value2);
        }
    }
    return dst;
}

/* dst = src1 * src2 */

int motion_recognition_mat_max(matrix *src, int *index)
{
    float result = -FLT_MAX;
    for (int i = 0; i < src->row; i++) {
        for (int j = 0; j < src->col; j++) {
            float matrix_value = motion_recognition_matrix_get(src, i, j);
            if (matrix_value > result) {
                result = matrix_value;
                index[0] = i;
                index[1] = j;
            }
        }
    }
    return result;
}


/************************************************************************/
/*                          Array Operations                            */
/************************************************************************/
void motion_recognition_array_zero(float *src, unsigned size)
{
    for (int i = 0; i < size; i++) {
        // *(src + i) = 0.0;
        motion_recognition_array_set(src, i, 0.0);
    }
}

float motion_recognition_array_get(float *src, int pos)
{
    float value = 0;
    if (!sns_island_is_island_ptr((intptr_t)src)) {
        SNS_ISLAND_EXIT();
        value = src[pos];
    } else {
        value = src[pos];
    }
    return value;
}

void motion_recognition_array_set(float *src, int pos, float value)
{
    if (!sns_island_is_island_ptr((intptr_t)src)) {
        SNS_ISLAND_EXIT();
        src[pos] = value;
    } else {
        src[pos] = value;
    }
}

float motion_recognition_segment_sum(float *src, int start, int length)
{
    float sum = 0.0;
    for (int i = start; i < start + length; i++) {
        sum += motion_recognition_array_get(src, i);
    }
    return sum;
}

float motion_recognition_array_max(float *src, int start, int length)
{
    float max = -FLT_MAX;
    for (int i = start; i < start + length; i++) {
        float array_value = motion_recognition_array_get(src, i);
        if (array_value > max) {
            max = array_value;
        }
    }
    return max;
}

float motion_recognition_array_mean(float *src, int start, int length)
{
    return motion_recognition_segment_sum(src, start, length) / length;
}

int motion_recognition_arg_max(float *src, int start, int length)
{
    float max = -FLT_MAX;
    int index = start;
    for (int i = start; i < start + length; i++) {
        float array_value = motion_recognition_array_get(src, i);
        if (array_value > max) {
            max = array_value;
            index = i;
        }
    }
    return index - start;
}

void motion_recognition_sort(float *src, int start, int length)
{
    float tmp;
    for (int i = start; i < start + length - 1; i++) {
        for (int j = start; j < start + length - 1 - i; j++) {
            float array_value1 = motion_recognition_array_get(src, j + 1);
            float array_value0 = motion_recognition_array_get(src, j);
            if (array_value1 < array_value0) {
                tmp = array_value0;
                motion_recognition_array_set(src, j, array_value1);
                motion_recognition_array_set(src, j + 1, tmp);
            }
        }
    }
}

float motion_recognition_array_std(float *src, int start, int length)
{
    float mean_value = motion_recognition_array_mean(src, start, length);
    float variance = 0;
    for (int i = start; i < start + length; i++) {
        float array_value = motion_recognition_array_get(src, i);
        variance += powf(array_value - mean_value, 2.0);
    }
    variance /= length - 1;
    return sqrtf(variance);
}


void motion_recognition_fft(struct motion_recognition_persistent_state *persistent, float *input, int length)
{
    kiss_fft_cpx* buffer = NULL;

    buffer = persistent->malloc(sizeof(kiss_fft_cpx) * length);

    for (int i = 0; i < length; i++) {
        float array_value = motion_recognition_array_get(input, i); //input in heap
        if (!sns_island_is_island_ptr((intptr_t)buffer)) {
            SNS_ISLAND_EXIT();
            buffer[i].r = array_value;
            buffer[i].i = 0;
        } else {
            buffer[i].r = array_value;
            buffer[i].i = 0;
        }
    }

    kiss_fft_cfg cfg = kiss_fft_alloc(length , 0 , 0 , 0);

    kiss_fft(cfg, buffer, buffer);

    for (int i = 0; i < length; i++) {
         motion_recognition_array_set(input, i, //input in heap
            sqrtf(powf(buffer[i].r, 2.0) + powf(buffer[i].i, 2.0)));
    }

    persistent->free(buffer);
    persistent->free(cfg);
}
