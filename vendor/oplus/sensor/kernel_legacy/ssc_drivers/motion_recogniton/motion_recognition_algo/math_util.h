#ifndef __OPLUS_MATHUTL__
#define __OPLUS_MATHUTL__

#include "model_parameter.h"

#define equal(a, b)    ((a-b)<1e-7 && (a-b)>-(1e-7))

matrix *motion_recognition_matrix_create(struct motion_recognition_persistent_state *persistent, matrix *mat, int row, int col);

void motion_recognition_matrix_delete(struct motion_recognition_persistent_state *persistent, matrix *mat);

float motion_recognition_matrix_get(matrix *mat, int pos_r, int pos_c);

void motion_recognition_matrix_set(matrix *mat, int pos_r, int pos_c, float value);

matrix *motion_recognition_matrix_zeros(matrix *mat);

int motion_recognition_mat_max(matrix *src, int *index);

matrix *motion_recognition_matrix_sub(matrix *src1, matrix *src2, matrix *dst);

void motion_recognition_array_zero(float *src, unsigned size);

float motion_recognition_array_get(float *src, int pos);

void motion_recognition_array_set(float *src, int pos, float value);

float motion_recognition_segment_sum(float *src, int start, int length);

float motion_recognition_array_std(float *src, int start, int length);

float motion_recognition_array_mean(float *src, int start, int length);

float motion_recognition_array_max(float *src, int start, int length);

int motion_recognition_arg_max(float *src, int start, int length);

void motion_recognition_sort(float *src, int start, int length);

void motion_recognition_fft(struct motion_recognition_persistent_state *persistent, float *input, int length);

#endif //__OPLUS_MATHUTL__
