#ifndef INIT_H
#define INIT_H

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
//#include "math_util.h"

#define LABELS_NUM_ROWS 2
#define LABELS_NUM_COLUMNS 6
#define PROBABILITIES_NUM_ROWS  4
#define PROBABILITIES_NUM_COLUMNS  6
#define MAX_NUM_SAMPLES  1000
#define NUM_DATA_COLUMNS  3
#define NUM_ACTIONS 2
#define NaN FLT_MIN


#define SENSOR_DIM  3
#define ACMC_FREQ  20   //sample rate
#define ACMC_START 200
#define NUM_ACTIVITIES  5
#define NUM_DEVICE_POSITIONS  9
#define CMC_FEATURE_DIM 5
#define LR_FEATURE_DIM 8
#define LR2_FEATURE_DIM  21
#define LR1VALL_FEATURE_DIM  34

#define PI acosf(-1)

typedef enum
{
    ACTIVITY_UNKNOWN,
    ACTIVITY_WALK,
    ACTIVITY_RUN,
    ACTIVITY_STATIONARY,
    ACTIVITY_VEHICLE,
    ACTIVITY_BIKE_TRANSIT,
    ACTIVITY_IN_MOTOR_TRANSIT = 7,
    ACTIVITY_BIKE,
} ACTIVITY_MODE;

typedef enum
{
    MOTION_STATE_OUT,
    MOTION_STATE_ENTRY,
    MOTION_STATE_EXIT,
    MOTION_STATE_IN,
} MOTION_STATE;

typedef signed char int8_t;
typedef struct m_conf_thresh
{
    float basic[5];
    float drive[5];
    float bike[5];
} m_conf_thresh;

typedef struct hyst_mat
{
    float basic[6][6];
    float drive[6][6];
    float bike[6][6];
} hyst_mat;

struct hyst
{
    int8_t hmode;
    int8_t stable_state; //% initially in NULL state (make sure this matches with the NULL_ID variable inside applyHysteresisFilter.m
    int8_t trans_state; //
    int8_t count;
    int8_t prev_state;
    m_conf_thresh conf_thresh;
    hyst_mat _mat;
};

struct heu
{
    int8_t initialize;
    int8_t bike_vs_drive;
    int8_t exit;
    int8_t stationary_count;
};

struct activity_logic
{
    float prob_threshold;
    int history_length;
    int X1;
    int X2;
    int max_timer;
    int state_id;
};  // load

struct motion_recognition_model
{
    float cmc_mean[5][9][5];
    float cmc_cov[5][9][5];
    float cmc_pcomponent[5][9][5];
    float cmc_priors[5][9];
    int cmc_win[5];
    float lr_bias;
    int lr_win[1];
    float lr_weights[9];
    int lr2_win[1];
    float lr2_weights[22];
    float alphas[5];
    float filter_probabilities_iir[5][5];
    float likelihood_regularization;
    float linear_accel_time_constant;
    float unknown_confidence_threshold[5][5];
    float use_max_for_components[5][9];
    int drive_entry_window;
    float drive_entry_threshold;
    float chebyshev_a[5];
    float chebyshev_b[5];
    float lr1vall_weights[5][35];
    float lr1vall_priors[5];
};

struct classify_data_in
{
    int8_t action[NUM_ACTIONS];
};

struct classify_data_out
{
    float probabilities[PROBABILITIES_NUM_ROWS * PROBABILITIES_NUM_COLUMNS];
    int labels[LABELS_NUM_ROWS * LABELS_NUM_COLUMNS];
};

typedef struct matrix
{
    int row, col;
    float **element;
} matrix;

struct motion_recognition_persistent_state
{
    bool acmc_ready;
    // bool lrdrivevsubwayDisable

    struct hyst *hyst;

    struct heu *heu;
    int8_t in_drive;
    int8_t in_bike;

    bool filter_condition_empty;
    bool filtered_accel_buffer3X200_empty;

    struct activity_logic *bike_logic;  // load
    struct activity_logic *drive_logic;

    struct motion_recognition_model *models;

    void* (*malloc)(size_t size);   //malloc heap
    void (*free)(void*);            //free heap

    matrix accel_buffer;
    int accel_buffer_ptr;
    matrix accel_buffer3X200;
    matrix filtered_accel_buffer3X200;

    float filter_condition[3];

    float linear_accel_alpha;
    float drive_entry_unfiltprob_buffer[6];
    float drive_buffer[180];
    // (X 180  DRIVE_LOGIC.history)
    float bike_buffer[150];
    // (bike_logic.history_length X180)

    float cmc_features[5]; // 5X1  (cmc_feature_dim)
    float lr_features[8];  //  8X1  ( lr_feature_dim)
    float lr2_features[21];   // 21 X (lr2_feature_dim)
    float lr1vall_features[34];  // 34X  (lr1vall_feature_dim
    int feature_cnt;
    matrix prob_matrix;
    matrix downsampled_data20X3;

    struct classify_data_in classify_data_in;
    struct classify_data_out classify_data_out;
};

void motion_recognition_persistent_init(struct motion_recognition_persistent_state *persistent);
void motion_recognition_persistent_deinit(struct motion_recognition_persistent_state *persistent);
void motion_recognition_persistent_reset_state(struct motion_recognition_persistent_state *persistent);

#endif // MODEL_LOAD_H //INIT_H
