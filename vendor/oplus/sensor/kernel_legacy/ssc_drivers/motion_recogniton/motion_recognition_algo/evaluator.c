#include <assert.h>
#include <complex.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "math_util.h"
#include "model_parameter.h"

#define NFFT 256
#define FS 20
#define DRIVE_COMMUTE_SIZE 180
#define BIKE_COMMUTE_SIZE 150
#define CHEBYSHEV_DIM 5
#define LR_DRIVE_IDX  1
#define STEP_SIZE  20

#define USE_MAX_FOR_GMM
#define USE_DRIVE_ENTRY_CONDITION

typedef enum {
    CMC_UNKNOWN_IDX,
    CMC_WALK_IDX = 0,
    CMC_RUN_IDX,
    CMC_STATIONARY_IDX,
    CMC_DRIVE_IDX,
    CMC_BIKE_IDX,
} ACTIVITIES_IDX;


struct item
{
    int index;
    float value;
};


static void item_initialize(struct item *item, int index, float value)
{
    item->index = index;
    item->value = value;
}

static void custom_sort_vector_1d(struct item *item, int size)
{
    struct item tmp;
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - 1 - i; j++) {
            if (item[j + 1].value > item[j].value) {
                tmp = item[j];
                item[j] = item[j + 1];
                item[j + 1] = tmp;
            }
        }
    }
}

static int8_t replaceLabel_0_with_NULL_ID(float in, int8_t NULL_ID)
{
    if (in == 0) {
        return (int8_t) (NULL_ID);
    } else {
        return (int8_t) (in);
    }
}

static int8_t replaceLabel_NULL_ID_with_0(float in, int8_t NULL_ID)
{
    if (in == (float) NULL_ID) {
        return (int8_t) 0;
    } else {
        return (int8_t) in;
    }
}


static void apply_commute_logic(struct motion_recognition_persistent_state *_persistent, int buffer_size)
{
    if (_persistent->heu->initialize) {
        _persistent->heu->stationary_count = 0;
        _persistent->heu->exit = 0;
        return;
    }
    if (_persistent->heu->stationary_count < 0){
        _persistent->heu->stationary_count = 0;
    }
    _persistent->heu->stationary_count += 1;

    int st_timer = 0;

    if (_persistent->heu->bike_vs_drive == ACTIVITY_IN_MOTOR_TRANSIT) {
        float count = 0;
        for (int i = 0; i < buffer_size; i++) {
            if (_persistent->drive_buffer[i] > _persistent->drive_logic->prob_threshold) {
                    count = count + 1;
            }
        }

        if (count < _persistent->drive_logic->X1) {
            st_timer = 0;
        } else if (count > _persistent->drive_logic->X2) {
            st_timer = _persistent->drive_logic->max_timer;
        } else {
            st_timer = roundf(_persistent->drive_logic->max_timer *
                       (count - _persistent->drive_logic->X1) / (_persistent->drive_logic->X2 - _persistent->drive_logic->X1));
        }
    }

    if (_persistent->heu->bike_vs_drive == ACTIVITY_BIKE) {
        float count = 0;
        for (int i = 0; i < buffer_size; i++) {
            if (_persistent->bike_buffer[i] >
                _persistent->bike_logic->prob_threshold) {
                count = count + 1;
            }
        }

        if (count < _persistent->bike_logic->X1) {
            st_timer = 0;
        } else if (count > _persistent->bike_logic->X2) {
            st_timer = _persistent->bike_logic->max_timer;
        } else {
            st_timer = roundf(_persistent->bike_logic->max_timer *(count - _persistent->bike_logic->X1)
                    / (_persistent->bike_logic->X2 - _persistent->bike_logic->X1));
        }
    }

    // % Check if timer has elapsed
    if (_persistent->heu->stationary_count >= st_timer) {
        _persistent->heu->exit = 1;
    } else {
        _persistent->heu->exit = 0;
    }
}

static void apply_hysteresis_filter(struct motion_recognition_persistent_state *_persistent,
                           float (*MAT)[6], float conf_thresh[],
                           float filtered_activity_probs[], int8_t probs_size,
                           int8_t cmclabels, int8_t *labels_post_hyst)
{

    int8_t NULL_ID = 6;
    _persistent->hyst->stable_state = replaceLabel_0_with_NULL_ID(_persistent->hyst->stable_state, NULL_ID);
    _persistent->hyst->trans_state = replaceLabel_0_with_NULL_ID(_persistent->hyst->trans_state, NULL_ID);
    cmclabels = replaceLabel_0_with_NULL_ID(cmclabels, NULL_ID);

    if (cmclabels != NULL_ID) {
        struct item item_init;
        item_initialize(&item_init, -1, -1);

        const size_t arr_size = probs_size;
        struct item idx[arr_size];
        for (int i = 0; i < probs_size; i++) {
            idx[i].index = i;
            idx[i].value = filtered_activity_probs[i];
        }

        custom_sort_vector_1d(idx, probs_size);
        if (filtered_activity_probs[cmclabels - 1] - filtered_activity_probs[idx[1].index] > conf_thresh[cmclabels - 1]) {
            *labels_post_hyst = cmclabels;
            _persistent->hyst->hmode = 0;
            _persistent->hyst->stable_state = cmclabels;
            _persistent->hyst->trans_state = 0;
            _persistent->hyst->count = 0;
            return;
        }
    }

    if (_persistent->hyst->hmode == 0) {
        if (cmclabels == _persistent->hyst->stable_state) {
            *labels_post_hyst = cmclabels;
        } else {
            if (MAT[_persistent->hyst->stable_state - 1][cmclabels - 1] <= 1) {
                *labels_post_hyst = cmclabels;
                _persistent->hyst->stable_state = cmclabels;
            } else {
                *labels_post_hyst = _persistent->hyst->stable_state;
                _persistent->hyst->hmode = 1;
                _persistent->hyst->trans_state = cmclabels;
                _persistent->hyst->count = 1;
            }
        }
    } else {
        if (cmclabels == _persistent->hyst->trans_state) {
            _persistent->hyst->count += 1;
            if (_persistent->hyst->count >= MAT[_persistent->hyst->stable_state - 1][_persistent->hyst->trans_state - 1]) {
                *labels_post_hyst = cmclabels;
                _persistent->hyst->hmode = 0;
                _persistent->hyst->stable_state = cmclabels;
                _persistent->hyst->trans_state = 0;
                _persistent->hyst->count = 0;
            } else {
                *labels_post_hyst = _persistent->hyst->stable_state;
            }
        } else if (cmclabels == _persistent->hyst->stable_state) {
            _persistent->hyst->hmode = 0;
            _persistent->hyst->trans_state = 0;
            _persistent->hyst->count = 0;
            *labels_post_hyst = cmclabels;
        } else {
            if (MAT[_persistent->hyst->stable_state - 1][cmclabels - 1] <= 1) {
                _persistent->hyst->hmode = 0;
                *labels_post_hyst = cmclabels;
                _persistent->hyst->count = 0;
                _persistent->hyst->trans_state = 0;
                _persistent->hyst->stable_state = cmclabels;
            } else {
                _persistent->hyst->count = 1;
                *labels_post_hyst = _persistent->hyst->stable_state;
                _persistent->hyst->trans_state = cmclabels;
            }
        }
    }

    _persistent->hyst->stable_state = replaceLabel_NULL_ID_with_0(_persistent->hyst->stable_state, NULL_ID);
    _persistent->hyst->trans_state = replaceLabel_NULL_ID_with_0(_persistent->hyst->trans_state, NULL_ID);
    *labels_post_hyst = replaceLabel_NULL_ID_with_0(*labels_post_hyst, NULL_ID);
}

static void cmc_compute_probabilities(struct motion_recognition_persistent_state *_persistent)
{
    float likelihood[5][9][5];
    memset(likelihood, 0, sizeof(likelihood));

    for (int a = 0; a < NUM_ACTIVITIES; a++) {
        for (int p = 0; p < NUM_DEVICE_POSITIONS; p++) {
            if (_persistent->models->cmc_priors[a][p] > 0) {
                float Likelihoods = 1.0;
                for (int g = 0; g < CMC_FEATURE_DIM; g++) {
                    float factor = 1 / sqrtf((2 * PI) / _persistent->models->cmc_cov[a][p][g]);
                    likelihood[a][p][g] = _persistent->models->cmc_pcomponent[a][p][g] * factor *
                                                expf(-0.5 * powf(_persistent->cmc_features[g] - _persistent->models->cmc_mean[a][p][g], 2)
                                                        * _persistent->models->cmc_cov[a][p][g]);
                    likelihood[a][p][g] = likelihood[a][p][g] + _persistent->models->likelihood_regularization;
                    Likelihoods = Likelihoods * likelihood[a][p][g];
                }
                motion_recognition_matrix_set(&_persistent->prob_matrix,
                    a, p, _persistent->models->cmc_priors[a][p] * Likelihoods);
            }
        }
    }

    float sum = 0.0;

    for (int i = 0; i < _persistent->prob_matrix.row; i++) {
        for (int j = 0; j < _persistent->prob_matrix.col; j++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->prob_matrix, i, j);
            sum += matrix_value;
        }
    }

    for (int i = 0; i < _persistent->prob_matrix.row; i++) {
        for (int j = 0; j < _persistent->prob_matrix.col; j++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->prob_matrix, i, j);
            motion_recognition_matrix_set(&_persistent->prob_matrix, i, j, matrix_value / sum);
        }
    }
}


/*IQR*/
/* ||a(n)|| = sqrt(x^2 + y^2 +z^2)
 * normdata(n) = motion_recognition_sort(||a(n)||)
 * iqr = normdata(round(0.75N)) - normdata(round(0.25N))
 * */
static void compute_cmc_feature_iqr(struct motion_recognition_persistent_state *_persistent)
{
    int rows = SENSOR_DIM;
    int cols = _persistent->models->cmc_win[_persistent->feature_cnt];
    float normdata[20] = {0.0};

    for (int i = 0; i < cols; i++) {
        for (int j = 0; j < rows; j++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                j, i + _persistent->accel_buffer3X200.col - cols);
            normdata[i] += powf(matrix_value, 2.0);
        }
        normdata[i] = sqrtf(normdata[i]);
    }

    motion_recognition_sort(normdata, 0, cols);
    _persistent->cmc_features[_persistent->feature_cnt] = normdata[(int) (roundf(0.75 * cols)) - 1] - normdata[(int) (roundf(0.25 * cols)) - 1];
    _persistent->feature_cnt++;
}

/*Modified RM*/
static void compute_cmc_feature_modrm(struct motion_recognition_persistent_state *_persistent)
{
    int rows = SENSOR_DIM;
    int cols = _persistent->models->cmc_win[_persistent->feature_cnt];

    float norm_of_means = 0.0;
    float mean_of_norms = 0.0;
    float rm;
    float temp_value = 0.0;

    for (int i = 0; i < rows; ++i) {
        temp_value = 0.0;
        for (int j = 0; j < cols; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                i, j + _persistent->accel_buffer3X200.col - cols);
            temp_value += matrix_value;
        }
        norm_of_means += powf(temp_value / cols, 2.0);
    }
    norm_of_means = sqrtf(norm_of_means);

    for (int i = 0; i < cols; ++i) {
        temp_value = 0.0;
        for (int j = 0; j < rows; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                j, i + _persistent->accel_buffer3X200.col - cols);
            temp_value += powf(matrix_value, 2.0);
        }
        mean_of_norms += sqrtf(temp_value);
    }
    mean_of_norms = mean_of_norms / cols;

    if (!equal(mean_of_norms, 0.0)) {
        rm = norm_of_means / mean_of_norms;
    } else {
        rm = 0.0;
    }
    rm = rm - 1e-5;
    _persistent->cmc_features[_persistent->feature_cnt] = -logf(1.0 - rm);
    _persistent->feature_cnt++;
}

/*MM3*/
static void compute_cmc_feature_mm3(struct motion_recognition_persistent_state *_persistent)
{
    int rows = SENSOR_DIM;
    int cols = _persistent->models->cmc_win[_persistent->feature_cnt];
    float mean_of_axis[3] = {0.0};
    float sum = 0.0;
    float temp_value = 0.0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                i, j + _persistent->accel_buffer3X200.col - cols);
            mean_of_axis[i] += matrix_value;
        }
        mean_of_axis[i] /= cols;
    }

    for (int i = 0; i < cols; ++i) {
        temp_value = 0.0;
        for (int j = 0; j < rows; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                j, i + _persistent->accel_buffer3X200.col - cols);
            temp_value += powf(matrix_value - mean_of_axis[j], 2.0);
        }
        sum += sqrtf(temp_value);
    }

    _persistent->cmc_features[_persistent->feature_cnt] = sum / cols;
    _persistent->feature_cnt++;
}

/*DIFFSUM DIFF10P*/
static void compute_cmc_feature_diffsum_diff10p(struct motion_recognition_persistent_state *_persistent)
{
    float diff_sum = 0.0;
    float norm_of_diff = 0.0;
    float *norm_data = _persistent->malloc(sizeof(float) * (_persistent->accel_buffer3X200.col - 1));

    for (int col = 0; col < _persistent->accel_buffer3X200.col - 1; col++) {
        norm_of_diff = 0;
        for (int row = 0; row < _persistent->accel_buffer3X200.row; row++) {
            float matrix_value_l = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col + 1);
            float matrix_value_r = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col);
            norm_of_diff = norm_of_diff + powf(matrix_value_l - matrix_value_r, 2.0);
        }
        norm_of_diff = sqrtf(norm_of_diff);
        diff_sum = diff_sum + norm_of_diff;
        motion_recognition_array_set(norm_data, col, norm_of_diff);
    }

    motion_recognition_sort(norm_data, 0, _persistent->accel_buffer3X200.col - 1);
    _persistent->cmc_features[_persistent->feature_cnt++] = diff_sum;
    _persistent->cmc_features[_persistent->feature_cnt++] = motion_recognition_segment_sum(norm_data, 0, roundf(0.1 * _persistent->accel_buffer3X200.col)) / diff_sum;
    _persistent->free(norm_data);
}

static void compute_cmc_features(struct motion_recognition_persistent_state *_persistent)
{
    _persistent->feature_cnt = 0;
    compute_cmc_feature_iqr(_persistent);
    compute_cmc_feature_modrm(_persistent);
    compute_cmc_feature_mm3(_persistent);
    compute_cmc_feature_diffsum_diff10p(_persistent);
}

/* Standard deviation of accelerometer norm */
static void compute_lr_feature_sa(struct motion_recognition_persistent_state *_persistent)
{
    float *norm_array = _persistent->malloc(sizeof(float) * _persistent->accel_buffer3X200.col);
    float accel_norm = 0;

    for (int col = 0; col < _persistent->accel_buffer3X200.col; col++) {
        accel_norm = 0;
        for (int row = 0; row < _persistent->accel_buffer3X200.row; row++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col);
            accel_norm = accel_norm + powf(matrix_value, 2.0);
        }
        motion_recognition_array_set(norm_array, col, sqrtf(accel_norm));
    }

    _persistent->lr_features[_persistent->feature_cnt] = motion_recognition_array_std(norm_array, 0, _persistent->accel_buffer3X200.col);
    _persistent->free(norm_array);
    _persistent->feature_cnt++;
}

/* Standard deviation of the accel diff norms */
static void compute_lr_feature_sadiff(struct motion_recognition_persistent_state *_persistent)
{
    float *norm_diff_array = _persistent->malloc(sizeof(float) * _persistent->accel_buffer3X200.col);
    float accel_diff_norm = 0;

    for (int col = 0; col < _persistent->accel_buffer3X200.col - 1; col++) {
        accel_diff_norm = 0;
        for (int row = 0; row < _persistent->accel_buffer3X200.row; row++) {
            float matrix_value_l = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col + 1);
            float matrix_value_r = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col);
            accel_diff_norm += powf(matrix_value_l - matrix_value_r, 2.0);
        }
        motion_recognition_array_set(norm_diff_array, col, sqrtf(accel_diff_norm));
    }

    _persistent->lr_features[_persistent->feature_cnt] = motion_recognition_array_std(norm_diff_array, 0, _persistent->accel_buffer3X200.col - 1);
    _persistent->free(norm_diff_array);
    _persistent->feature_cnt++;
}

/* Standard deviation of the accel diff norms */
static void compute_lr_feature_linear_accel_and_diff(struct motion_recognition_persistent_state *_persistent)
{
    float *linear_acc_array = _persistent->malloc(sizeof(float) * _persistent->accel_buffer3X200.col);
    for (int row = 0; row < _persistent->accel_buffer3X200.row; row++) {
        for (int col = 0; col < _persistent->accel_buffer3X200.col; col++) {
            float matrix_value_l = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, row, col);
            float matrix_value_r = motion_recognition_matrix_get(&_persistent->filtered_accel_buffer3X200, row, col);
            motion_recognition_array_set(linear_acc_array, col, matrix_value_l - matrix_value_r);
        }
        _persistent->lr_features[row + _persistent->feature_cnt] = motion_recognition_array_std(linear_acc_array, 0, _persistent->filtered_accel_buffer3X200.col);

        for (int i = 0; i < _persistent->accel_buffer3X200.col - 1; i++) {
            float array_value1 = motion_recognition_array_get(linear_acc_array, i + 1);
            float array_value = motion_recognition_array_get(linear_acc_array, i);
            motion_recognition_array_set(linear_acc_array, i, array_value1 - array_value);
        }
        _persistent->lr_features[3 + row + _persistent->feature_cnt] = motion_recognition_array_std(linear_acc_array, 0, _persistent->filtered_accel_buffer3X200.col - 1);
    }
    _persistent->free(linear_acc_array);
    _persistent->feature_cnt += 6;
}

static void compute_lr_features(struct motion_recognition_persistent_state *_persistent)
{
    _persistent->feature_cnt = 0;

    compute_lr_feature_sa(_persistent);
    compute_lr_feature_sadiff(_persistent);

    for (int i = 0; i < _persistent->accel_buffer3X200.row; i++) {
        for (int j = 0; j < _persistent->accel_buffer3X200.col - STEP_SIZE; j++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->filtered_accel_buffer3X200, i, j + STEP_SIZE);
            motion_recognition_matrix_set(&_persistent->filtered_accel_buffer3X200, i, j, matrix_value);
        }
    }

    for (int i = 0; i < _persistent->accel_buffer3X200.row; i++) {
        for (int j = 0; j < STEP_SIZE; j++) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200,
                i, j + _persistent->accel_buffer3X200.col - STEP_SIZE);
            motion_recognition_matrix_set(&_persistent->filtered_accel_buffer3X200,
                i, j + _persistent->accel_buffer3X200.col - STEP_SIZE,
                _persistent->linear_accel_alpha * matrix_value + _persistent->filter_condition[i]);
            matrix_value = motion_recognition_matrix_get(&_persistent->filtered_accel_buffer3X200,
                i, j + _persistent->accel_buffer3X200.col - STEP_SIZE);
            _persistent->filter_condition[i] = (1 - _persistent->linear_accel_alpha) * matrix_value;
        }
    }

    // %linear accel
    compute_lr_feature_linear_accel_and_diff(_persistent);
}

// compute the Frequence Domain feature
static void compute_lr2_features(float *inputs, int length, int feature_cnt,
    struct motion_recognition_persistent_state *_persistent)
{
    float *inputs_zeros_padding = _persistent->malloc(sizeof(float) * NFFT);

    for (int i = 0; i < NFFT; i++) {
        if (i < length) {
            motion_recognition_array_set(inputs_zeros_padding, i, inputs[i]);
        } else {
            motion_recognition_array_set(inputs_zeros_padding, i, 0);
        }
    }

    motion_recognition_fft(_persistent, inputs_zeros_padding, NFFT);

    // Compute spectral energy between 0-2 Hz
    float sum_numerator = 0;
    float sum_denominator = 0;
    for (int i = 1; i < roundf(2 * NFFT / FS) + 1; i++) {
        float array_value = motion_recognition_array_get(inputs_zeros_padding, i);
        sum_numerator = sum_numerator + array_value;
    }
    for (int i = 0; i < NFFT / 2; i++) {
        float array_value = motion_recognition_array_get(inputs_zeros_padding, i);
        sum_denominator = sum_denominator + array_value;
    }

    float se0_2 = 0.0;
    if (!equal(sum_denominator, 0.0)) {
        se0_2 = sum_numerator / sum_denominator;
    }

    // Compute spectral energy between 2-4 Hz
    sum_numerator = 0;
    for (int i = roundf(2 * NFFT / FS) + 1; i < roundf(4 * NFFT / FS); i++) {
        float array_value = motion_recognition_array_get(inputs_zeros_padding, i);
        sum_numerator = sum_numerator + array_value;
    }

    float se2_4 = 0.0;
    if (!equal(sum_denominator, 0.0)) {
        se2_4 = sum_numerator / sum_denominator;
    }

    // Compute spectral energy between 5-10 Hz
    sum_numerator = 0;
    for (int i = roundf(5 * NFFT / FS); i < roundf(10 * NFFT / FS); i++) {
        float array_value = motion_recognition_array_get(inputs_zeros_padding, i);
        sum_numerator = sum_numerator + array_value;
    }

    float se5_10 = 0.0;
    if (!equal(sum_denominator, 0.0)) {
        se5_10 = sum_numerator / sum_denominator;
    }

    // Compute magnitude of highest peak after 0
    float pmag = 0.0;
    float array_value = motion_recognition_array_get(inputs_zeros_padding, 0);
    if (!equal(array_value, 0.0)) {
        pmag = motion_recognition_array_max(inputs_zeros_padding, 1, NFFT / 2) / array_value;
    }

    float ploc = (float) (motion_recognition_arg_max(inputs_zeros_padding, 1, NFFT / 2) + 1) * FS / NFFT;

    // Compute spectral entropy
    float *p = _persistent->malloc(sizeof(float) * (NFFT / 2 - 1));

    float max_value = motion_recognition_array_max(inputs_zeros_padding, 1, NFFT / 2 - 1);
    for (int i = 0; i < NFFT / 2 - 1; i++) {
        float array_value_t = motion_recognition_array_get(inputs_zeros_padding, i + 1);
        motion_recognition_array_set(p, i, array_value_t / max_value);
    }
    _persistent->free(inputs_zeros_padding);

    float sent = 0;
    float temp = 0;
    for (int i = 0; i < NFFT / 2 - 1; i++) {
        float array_value = motion_recognition_array_get(p, i);
        if (array_value > 0.0) {
             temp = log2f(array_value);
             if (!isnan(temp) && isfinite(temp)) {
                 sent = sent - array_value * temp;
            }
        }
    }
    _persistent->free(p);

    // Compute zero-crossings
    float mean_value = motion_recognition_array_mean(inputs, 0, length);
    float *inputs_ = _persistent->malloc(sizeof(float) * length);
    for (int i = 0; i < length; i++) {
        motion_recognition_array_set(inputs_, i, inputs[i] - mean_value);
    }

    float zc = 0;
    for (int i = 0; i < length - 1; i++) {
        float value0 = motion_recognition_array_get(inputs_, i);
        float value1 = motion_recognition_array_get(inputs_, i + 1);
        if (value0 * value1 < 0.0) {
            zc = zc + 1;
        }
    }

    _persistent->free(inputs_);

    _persistent->lr2_features[feature_cnt++] = se0_2;
    _persistent->lr2_features[feature_cnt++] = se2_4;
    _persistent->lr2_features[feature_cnt++] = se5_10;
    _persistent->lr2_features[feature_cnt++] = pmag;
    _persistent->lr2_features[feature_cnt++] = ploc;
    _persistent->lr2_features[feature_cnt++] = sent;
    _persistent->lr2_features[feature_cnt++] = zc;
}

static float *lr_compute_probabilities(float *features, float *weights, float *currprob, int dimension)
{
    float prob = 0;
    for (int i = 0; i < dimension; i++) {
        prob += *(features + i) * *(weights + 1 + i);
    }
    prob += *weights;

    *currprob = 0.0;
    *(currprob + 1) = 1.0;

    if (prob < -50) {
        return currprob;
    } else if (prob > 50) {
        *currprob = 1;
        *(currprob + 1) = 0;
        return currprob;
    }

    prob = expf(prob);
    *currprob = prob / (1 + prob);
    *(currprob + 1) = 1 - *currprob;
    return currprob;
}

static void cmc_lr_compute_features(struct motion_recognition_persistent_state *_persistent)
{
    if (_persistent->filtered_accel_buffer3X200_empty) {
        _persistent->filtered_accel_buffer3X200_empty = false;
        //  MatrixXf tmp_diff_debug=tmp3X180_filter-tmp3X180_filter;
        for (int i = 0; i < _persistent->accel_buffer3X200.row; i++) {
            for (int j = 0; j < _persistent->accel_buffer3X200.col - STEP_SIZE; j++) {
                float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, i, j);
                motion_recognition_matrix_set(&_persistent->filtered_accel_buffer3X200, i, j + STEP_SIZE,
                    _persistent->linear_accel_alpha * matrix_value + _persistent->filter_condition[i]);
                matrix_value = motion_recognition_matrix_get(&_persistent->filtered_accel_buffer3X200, i, j + STEP_SIZE);
                _persistent->filter_condition[i] = (1 - _persistent->linear_accel_alpha) * matrix_value;
            }
        }
    }

    compute_cmc_features(_persistent);
    compute_lr_features(_persistent);

    float mean_array[3] = {0.0};
    for (int i = 0; i < _persistent->accel_buffer3X200.row; ++i) {
        for (int j = 0; j < _persistent->models->lr2_win[0]; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, i, j);
            mean_array[i] += matrix_value;
        }
        mean_array[i] = powf(mean_array[i] / _persistent->models->lr2_win[0], 2.0);
    }

    float *norm_grav = _persistent->malloc(sizeof(float) * _persistent->models->lr2_win[0]);
    float *norm_nongrav = _persistent->malloc(sizeof(float) * _persistent->models->lr2_win[0]);
    float *norm_comb = _persistent->malloc(sizeof(float) * _persistent->models->lr2_win[0]);
    int indmax = motion_recognition_arg_max(mean_array, 0, 3);
    for (int i = 0; i < _persistent->models->lr2_win[0]; ++i) {
        for (int j = 0; j < _persistent->accel_buffer3X200.row; ++j) {
            float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, j, i);
            if (j == indmax) {
                float array_value = motion_recognition_array_get(norm_grav, i);
                motion_recognition_array_set(norm_grav, i, array_value + powf(matrix_value, 2.0));
            } else {
                float array_value = motion_recognition_array_get(norm_nongrav, i);
                motion_recognition_array_set(norm_nongrav, i, array_value + powf(matrix_value, 2.0));
            }
            float array_value = motion_recognition_array_get(norm_comb, i);
            motion_recognition_array_set(norm_comb, i, array_value + powf(matrix_value, 2.0));
        }
        motion_recognition_array_set(norm_grav, i, sqrtf(*(norm_grav + i)));
        motion_recognition_array_set(norm_nongrav, i, sqrtf(*(norm_nongrav + i)));
        motion_recognition_array_set(norm_comb, i, sqrtf(*(norm_comb + i)));
    }

    compute_lr2_features(norm_grav, _persistent->models->lr2_win[0], 0, _persistent);
    _persistent->free(norm_grav);
    compute_lr2_features(norm_nongrav, _persistent->models->lr2_win[0], 7, _persistent);
    _persistent->free(norm_nongrav);
    compute_lr2_features(norm_comb, _persistent->models->lr2_win[0], 14, _persistent);
    _persistent->free(norm_comb);
    for (int i = 0; i < CMC_FEATURE_DIM; i++) {
        if (isnan(_persistent->cmc_features[i]) || !isfinite(_persistent->cmc_features[i])) {
                _persistent->cmc_features[i] = 0;
        }
    }

    for (int i = 0; i < LR_FEATURE_DIM; i++) {
        if (isnan(_persistent->lr_features[i]) || !isfinite(_persistent->lr_features[i])) {
                _persistent->lr_features[i] = 0;
        }
    }

    for (int i = 0; i < LR2_FEATURE_DIM; i++) {
        if (isnan(_persistent->lr2_features[i]) || !isfinite(_persistent->lr2_features[i])) {
                _persistent->lr2_features[i] = 0;
        }
    }
}

static void add_accel_data(struct motion_recognition_persistent_state *_persistent)
{
    if ((_persistent->classify_data_in.action[0] == 2) && _persistent->classify_data_in.action[1] == 1) {
        if (_persistent->acmc_ready) {
            for (int i = 0; i < ACMC_START - ACMC_FREQ; i++) {
                for (int j = 0; j < SENSOR_DIM; j++) {
                    float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, j, i + ACMC_FREQ);
                    motion_recognition_matrix_set(&_persistent->accel_buffer3X200, j, i, matrix_value);
                }
            }
            for (int i = 0; i < ACMC_FREQ; i++) {
                for (int j = 0; j < SENSOR_DIM; j++) {
                    float matrix_value = motion_recognition_matrix_get(&_persistent->downsampled_data20X3, i, j);
                    motion_recognition_matrix_set(&_persistent->accel_buffer3X200, j, ACMC_START - ACMC_FREQ + i, matrix_value);
                }
            }
        } else {
            for (int i = 0; i < ACMC_FREQ; i++) {
                for (int j = 0; j < SENSOR_DIM; j++) {
                    float matrix_value = motion_recognition_matrix_get(&_persistent->downsampled_data20X3, i, j);
                    motion_recognition_matrix_set(&_persistent->accel_buffer3X200, j, _persistent->accel_buffer_ptr, matrix_value);
                }
                _persistent->accel_buffer_ptr++;
            }
        }
        if (_persistent->accel_buffer_ptr == ACMC_START) {
            _persistent->accel_buffer_ptr = 0;
            _persistent->acmc_ready = true;
        }
    }
}

void motion_recognition_classifier(struct motion_recognition_persistent_state *_persistent)
{
    float unfiltered_activity_probs[5] = {0.0};
    float filtered_activity_probs[5] = {0.0};

    int8_t labels_pre_hyst = 0;
    int8_t labels_post_hyst = 0;

    float features[34] = {0.0};
    float labels[LABELS_NUM_ROWS][LABELS_NUM_COLUMNS] = {
            {0, ACTIVITY_WALK, ACTIVITY_RUN, ACTIVITY_STATIONARY, ACTIVITY_IN_MOTOR_TRANSIT, ACTIVITY_BIKE},
            {MOTION_STATE_OUT, MOTION_STATE_OUT, MOTION_STATE_OUT, MOTION_STATE_OUT, MOTION_STATE_OUT, MOTION_STATE_OUT},
    };

    add_accel_data(_persistent);

    if ((_persistent->classify_data_in.action[0] == 3) && _persistent->classify_data_in.action[1] == 1) {
        if (_persistent->acmc_ready == false) {
            return;
        }

        if (_persistent->filter_condition_empty) {
            _persistent->filter_condition_empty = false;
            for (int i = 0; i < 3; i++) {
                float matrix_value = motion_recognition_matrix_get(&_persistent->accel_buffer3X200, i, 0);
                _persistent->filter_condition[i] = (1 - _persistent->linear_accel_alpha) * matrix_value;
            }
        }

        cmc_lr_compute_features(_persistent);

        for (int i = 0; i < CMC_FEATURE_DIM; i++) {
            _persistent->lr1vall_features[i] = _persistent->cmc_features[i];
            features[i] = _persistent->cmc_features[i];
        }

        for (int i = 0; i < LR_FEATURE_DIM; i++) {
            _persistent->lr1vall_features[i + CMC_FEATURE_DIM] = _persistent->lr_features[i];
            features[i + CMC_FEATURE_DIM] = _persistent->lr_features[i];
        }

        for (int i = 0; i < LR2_FEATURE_DIM; i++) {
            _persistent->lr1vall_features[i + CMC_FEATURE_DIM + LR_FEATURE_DIM] = _persistent->lr2_features[i];
            features[i + CMC_FEATURE_DIM + LR_FEATURE_DIM] = _persistent->lr2_features[i];
        }
    }

    if ((_persistent->classify_data_in.action[0] == 4) && _persistent->classify_data_in.action[1] == 1) {
        if (_persistent->acmc_ready == false) {
            return;
        }

        cmc_compute_probabilities(_persistent);
        float unfilt_probs[5] = {0.0};

#ifdef USE_MAX_FOR_GMM
        int max_index[2] = {0};
        motion_recognition_mat_max(&_persistent->prob_matrix, max_index);
        float unfilt_probs_sum = 0.0;
        if (_persistent->models->use_max_for_components[max_index[0]][max_index[1]]) {
            for (int i = 0; i < NUM_ACTIVITIES; ++i) {
                for (int j = 0; j < NUM_DEVICE_POSITIONS; ++j) {
                    float matrix_value = motion_recognition_matrix_get(&_persistent->prob_matrix, i, j);
                    if (matrix_value > unfilt_probs[i]) {
                        unfilt_probs[i] = matrix_value;
                    }
                }
                unfilt_probs_sum += unfilt_probs[i];
            }
            for (int i = 0; i < NUM_ACTIVITIES; i++) {
                unfilt_probs[i] = (float) unfilt_probs[i] / unfilt_probs_sum;
            }
        } else {
            for (int i = 0; i < NUM_ACTIVITIES; ++i) {
                for (int j = 0; j < NUM_DEVICE_POSITIONS; ++j) {
                    float matrix_value = motion_recognition_matrix_get(&_persistent->prob_matrix, i, j);
                    unfilt_probs[i] += matrix_value;
                }
            }
        }
#else
        for (int i = 0; i < NUM_ACTIVITIES; ++i) {
            for (int j = 0; j < NUM_DEVICE_POSITIONS; ++j) {
                float matrix_value = motion_recognition_matrix_get(&_persistent->prob_matrix, i, j);
                unfilt_probs[i] += matrix_value;
            }
        }

#endif

        float gmm_probs[5];
        for (int i = 0; i < 5; i++) {
            gmm_probs[i] = unfilt_probs[i];
        }
        float lr_probs[5] = {0.0};

        // still vs drive
#ifndef LOGISTIC_REGRESSION_DISABLE
        float currprob[2] = {0.0};
        lr_compute_probabilities(_persistent->lr_features, _persistent->models->lr_weights, currprob, LR_FEATURE_DIM); // 8

        lr_probs[CMC_STATIONARY_IDX] = currprob[0];
        lr_probs[CMC_DRIVE_IDX] = currprob[LR_DRIVE_IDX];
        unfilt_probs[CMC_STATIONARY_IDX] =
                unfilt_probs[CMC_STATIONARY_IDX] + unfilt_probs[CMC_DRIVE_IDX] * (1 - currprob[LR_DRIVE_IDX]);
        unfilt_probs[CMC_DRIVE_IDX] = unfilt_probs[CMC_DRIVE_IDX] * currprob[LR_DRIVE_IDX];

        lr_probs[CMC_STATIONARY_IDX] = currprob[0];
        lr_probs[CMC_DRIVE_IDX] = currprob[LR_DRIVE_IDX];
#endif

        // walk vs bike
#ifndef LOGISTIC_REGRESSION2_DISABLE
        float currprob2[2] = {0.0};
        lr_compute_probabilities(_persistent->lr2_features, _persistent->models->lr2_weights, currprob2, LR2_FEATURE_DIM); // 21

        float walk_bike_gmm_sum = unfilt_probs[CMC_WALK_IDX] + unfilt_probs[CMC_BIKE_IDX];
        float walk_bike_lr2_sum = currprob2[0] + currprob2[1];
        float normalized_gmm_probs[2] = {0.0};
        normalized_gmm_probs[0] = unfilt_probs[CMC_WALK_IDX] / walk_bike_gmm_sum;
        normalized_gmm_probs[1] = unfilt_probs[CMC_BIKE_IDX] / walk_bike_gmm_sum;

        float conf_gmm = fabsf(normalized_gmm_probs[0] - normalized_gmm_probs[1]);
        float conf_lr2 = fabsf(currprob2[0] - currprob2[1]);

        if (conf_lr2 > conf_gmm) {
            unfilt_probs[CMC_WALK_IDX] = currprob2[0] * walk_bike_gmm_sum / walk_bike_lr2_sum;
            unfilt_probs[CMC_BIKE_IDX] = currprob2[1] * walk_bike_gmm_sum / walk_bike_lr2_sum;
        }

        lr_probs[CMC_WALK_IDX] = currprob2[0];
        lr_probs[CMC_BIKE_IDX] = currprob2[1];
#endif

#ifndef LR1V_ALL_DISABLE
        float lr1vall_probs[5] = {0.0};
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            float lrp[2] = {0.0};
            lr_compute_probabilities(_persistent->lr1vall_features, _persistent->models->lr1vall_weights[i],
                    lrp, LR1VALL_FEATURE_DIM); // 34
            lr1vall_probs[i] = lrp[0] / _persistent->models->lr1vall_priors[i];
        }

        float lr1vall_probs_sum =
                    motion_recognition_segment_sum(lr1vall_probs, 0, NUM_ACTIVITIES);
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            lr1vall_probs[i] = lr1vall_probs[i] / lr1vall_probs_sum;
        }

        int cmc_prediction = motion_recognition_arg_max(unfilt_probs, 0, NUM_ACTIVITIES);
        int lr_prediction =  motion_recognition_arg_max(lr1vall_probs, 0, NUM_ACTIVITIES);

        if ((cmc_prediction == CMC_STATIONARY_IDX) || ((cmc_prediction == CMC_DRIVE_IDX)
                    && (lr_prediction != CMC_BIKE_IDX))) {
        } else {
            for (int i = 0; i < NUM_ACTIVITIES; i++) {
                unfilt_probs[i] = lr1vall_probs[i];
            }
        }
#endif

        float filt_probs[5] = {0.0};
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            for (int j = 0; j < NUM_ACTIVITIES; j++) {
                if (i == j) {
                    filt_probs[i] += (1.0 - _persistent->models->filter_probabilities_iir[i][j]) * _persistent->models->alphas[j]
                        + _persistent->models->filter_probabilities_iir[i][j] * unfilt_probs[j];
                } else {
                    filt_probs[i] += (-_persistent->models->filter_probabilities_iir[i][j]) * _persistent->models->alphas[j]
                        + _persistent->models->filter_probabilities_iir[i][j] * unfilt_probs[j];
                }
            }
        }
        for (int i = 0; i < NUM_ACTIVITIES; ++i) {
            _persistent->models->alphas[i] = filt_probs[i];
        }
        float filt_sum = motion_recognition_segment_sum(filt_probs, 0, 5);
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            filt_probs[i] = filt_probs[i] / filt_sum;
        }

        struct item item_init;
        item_initialize(&item_init, -1, -1);
        struct item sort_probs[5];

        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            sort_probs[i].index = i;
            sort_probs[i].value = filt_probs[i];
        }
        custom_sort_vector_1d(sort_probs, NUM_ACTIVITIES);

        int8_t cmclabels = -100;

        cmclabels = sort_probs[0].index + 1;
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            unfiltered_activity_probs[i] = unfilt_probs[i];
            filtered_activity_probs[i] = filt_probs[i];
        }

        float confidence = sort_probs[0].value - sort_probs[1].value;
        if (confidence < _persistent->models->unknown_confidence_threshold[sort_probs[0].index][sort_probs[1].index] &&
                !_persistent->in_drive && !_persistent->in_bike) {
            cmclabels = ACTIVITY_UNKNOWN;
        }

        bool flag = true;
        for (int i = 0; i < NUM_ACTIVITIES; i++) {
            _persistent->drive_entry_unfiltprob_buffer[i] = _persistent->drive_entry_unfiltprob_buffer[i + 1];
        }
        _persistent->drive_entry_unfiltprob_buffer[5] = unfilt_probs[CMC_DRIVE_IDX];

#ifdef USE_DRIVE_ENTRY_CONDITION
        if ( !_persistent->in_drive && !_persistent->in_bike) {
            for (int i = 0; i < NUM_ACTIVITIES + 1; i++) {
                if (_persistent->drive_entry_unfiltprob_buffer[i] <= _persistent->models->drive_entry_threshold) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                cmclabels = ACTIVITY_VEHICLE;
                _persistent->hyst->hmode = 0;
                _persistent->hyst->stable_state = cmclabels;
                _persistent->hyst->trans_state = 0;
                _persistent->hyst->count = 0;
            }
        }
#endif
        if (cmclabels == ACTIVITY_VEHICLE) {
            labels_pre_hyst = ACTIVITY_IN_MOTOR_TRANSIT;
        } else if (cmclabels == ACTIVITY_BIKE_TRANSIT) {
            labels_pre_hyst = ACTIVITY_BIKE;
        } else {
            labels_pre_hyst = cmclabels;
        }

#ifndef HYSTERES_IS_DISABLE
        if (!_persistent->in_drive && !_persistent->in_bike) {
            apply_hysteresis_filter(_persistent, _persistent->hyst->_mat.basic, _persistent->hyst->conf_thresh.basic,
                                  filtered_activity_probs, NUM_ACTIVITIES, cmclabels, &labels_post_hyst);
        } else if (!_persistent->in_bike) {
            apply_hysteresis_filter(_persistent, _persistent->hyst->_mat.drive, _persistent->hyst->conf_thresh.drive,
                                  filtered_activity_probs, NUM_ACTIVITIES, cmclabels, &labels_post_hyst);
        } else {
            apply_hysteresis_filter(_persistent, _persistent->hyst->_mat.bike, _persistent->hyst->conf_thresh.bike,
                                  filtered_activity_probs, NUM_ACTIVITIES, cmclabels, &labels_post_hyst);
        }

        if (labels_post_hyst == ACTIVITY_VEHICLE) {
            labels_post_hyst = ACTIVITY_IN_MOTOR_TRANSIT;
        } else if (labels_post_hyst == ACTIVITY_BIKE_TRANSIT) {
            labels_post_hyst = ACTIVITY_BIKE;
        }
#else
        labels_post_hyst = labels_pre_hyst;
#endif

        if (labels_post_hyst != _persistent->hyst->prev_state) {
            for (int j = 0; j < LABELS_NUM_COLUMNS; j++) {
                if (labels[0][j] == labels_post_hyst) {
                    labels[1][j] = MOTION_STATE_ENTRY;
                }
                if (labels[0][j] == _persistent->hyst->prev_state) {
                    labels[1][j] = MOTION_STATE_EXIT;
                }
            }

            _persistent->hyst->prev_state = labels_post_hyst;
        } else { //% No transition
            for (int j = 0; j < LABELS_NUM_COLUMNS; j++) {
                if (labels[0][j] == labels_post_hyst) {
                    labels[1][j] = MOTION_STATE_IN;
                }
            }
        }

        float probabilities[PROBABILITIES_NUM_ROWS][PROBABILITIES_NUM_COLUMNS];
        for (size_t i = 0; i < PROBABILITIES_NUM_ROWS; i++) {
            for (size_t j = 0; j < PROBABILITIES_NUM_COLUMNS; j++) {
                probabilities[i][j] = 0;
            }
        }
        for (size_t i = 0; i < 5; i++) {
            probabilities[0][i + 1] = gmm_probs[i];
            probabilities[1][i + 1] = unfiltered_activity_probs[i];
            probabilities[2][i + 1] = lr_probs[i];
            probabilities[3][i + 1] = filtered_activity_probs[i];
        }

        for (int i = 0; i < DRIVE_COMMUTE_SIZE - 1; i++) {
            _persistent->drive_buffer[i] = _persistent->drive_buffer[i + 1];
        }
        _persistent->drive_buffer[DRIVE_COMMUTE_SIZE - 1] = filtered_activity_probs[CMC_DRIVE_IDX];
        if (!_persistent->in_drive && labels_post_hyst == labels[0][CMC_DRIVE_IDX + 1]) { //% first entry to drive
            _persistent->in_drive = 1;
            _persistent->heu->initialize = 1;
            _persistent->heu->bike_vs_drive = ACTIVITY_IN_MOTOR_TRANSIT;
            apply_commute_logic(_persistent, DRIVE_COMMUTE_SIZE);
        } else if (_persistent->in_drive) { // % currently in drive
            if (labels_post_hyst == labels[0][CMC_STATIONARY_IDX + 1]) { //   % instantaneous output is stationary
                _persistent->heu->initialize = 0;
                _persistent->heu->bike_vs_drive = ACTIVITY_IN_MOTOR_TRANSIT;
                apply_commute_logic(_persistent, DRIVE_COMMUTE_SIZE);

                if (_persistent->heu->exit) { //     % Timer has elapsed, exit
                    labels[1][CMC_DRIVE_IDX + 1] = MOTION_STATE_EXIT;
                    _persistent->in_drive = 0;
                } else { //  % Timer has not elapsed
                    labels[1][CMC_DRIVE_IDX + 1] = MOTION_STATE_IN;
                    labels[1][CMC_STATIONARY_IDX + 1] = MOTION_STATE_OUT;
                }
            } else if (labels_post_hyst == labels[0][CMC_DRIVE_IDX + 1]) { //   % instantaneous outputs is drive
                labels[1][CMC_DRIVE_IDX + 1] = MOTION_STATE_IN;
                _persistent->heu->initialize = 1;
                _persistent->heu->bike_vs_drive = ACTIVITY_IN_MOTOR_TRANSIT;
                apply_commute_logic(_persistent, DRIVE_COMMUTE_SIZE);
            } else { //  % instantaneous output neither stationary nor drive, exit
                _persistent->in_drive = 0;
                labels[1][CMC_DRIVE_IDX + 1] = MOTION_STATE_EXIT;
                if (labels[1][CMC_STATIONARY_IDX + 1] != 0) { //  % Exit stationary if in it
                    labels[1][CMC_STATIONARY_IDX + 1] = MOTION_STATE_EXIT;
                }
            }
        }

        // bike logic
        for (int i = 0; i < BIKE_COMMUTE_SIZE - 1; i++) {
            _persistent->bike_buffer[i] = _persistent->bike_buffer[i + 1];
        }
        _persistent->bike_buffer[BIKE_COMMUTE_SIZE - 1] = filtered_activity_probs[CMC_BIKE_IDX];
        if (!_persistent->in_bike && labels_post_hyst == labels[0][CMC_BIKE_IDX + 1]) { //% first entry to bike
            _persistent->in_bike = 1;
            _persistent->heu->initialize = 1;
            _persistent->heu->bike_vs_drive = ACTIVITY_BIKE;
            apply_commute_logic(_persistent, BIKE_COMMUTE_SIZE);
        } else if (_persistent->in_bike) { // % currently in bike
            if (labels_post_hyst == labels[0][CMC_STATIONARY_IDX + 1]) { //   % instantaneous output is stationary
                _persistent->heu->initialize = 0;
                _persistent->heu->bike_vs_drive = ACTIVITY_BIKE;
                apply_commute_logic(_persistent, BIKE_COMMUTE_SIZE);
                if (_persistent->heu->exit) { //     % Timer has elapsed, exit
                    labels[1][CMC_BIKE_IDX + 1] = MOTION_STATE_EXIT;
                    _persistent->in_bike = 0;
                } else { //  % Timer has not elapsed
                    labels[1][CMC_BIKE_IDX + 1] = MOTION_STATE_IN;
                    labels[1][CMC_STATIONARY_IDX + 1] = MOTION_STATE_OUT;
                }
            } else if (labels_post_hyst == labels[0][CMC_BIKE_IDX + 1]) { //   % instantaneous outputs is bike
                labels[1][CMC_BIKE_IDX + 1] = MOTION_STATE_IN;
                _persistent->heu->initialize = 1;
                _persistent->heu->bike_vs_drive = ACTIVITY_BIKE;
                apply_commute_logic(_persistent, BIKE_COMMUTE_SIZE);
            } else { //  % instantaneous output neither stationary nor bike, exit
                _persistent->in_bike = 0;
                labels[1][CMC_BIKE_IDX + 1] = MOTION_STATE_EXIT;
                if (labels[1][CMC_STATIONARY_IDX + 1] != 0) { //  % Exit stationary if in it
                    labels[1][CMC_STATIONARY_IDX + 1] = MOTION_STATE_EXIT;
                }
            }
        }

        for (int i = 0; i < LABELS_NUM_ROWS; i++) {
            for (int j = 0; j < LABELS_NUM_COLUMNS; j++) {
                _persistent->classify_data_out.labels[i + j * LABELS_NUM_ROWS] = labels[i][j];
            }
        }

        for (int i = 0; i < PROBABILITIES_NUM_ROWS; i++) {
            for (int j = 0; j < PROBABILITIES_NUM_COLUMNS; j++) {
                _persistent->classify_data_out.probabilities[i * PROBABILITIES_NUM_COLUMNS + j] = probabilities[i][j];
            }
        }
    }
}
