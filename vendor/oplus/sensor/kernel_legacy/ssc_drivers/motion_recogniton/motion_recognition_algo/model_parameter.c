#include "model_parameter.h"
#include "math_util.h"
#include <string.h>

int motion_recognition_labels[LABELS_NUM_ROWS * LABELS_NUM_COLUMNS] = {0, 0, 1, 0, 2, 0, 3, 0, 7, 0, 8, 0};
struct hyst motion_recognition_hyst = {
        .hmode = 0,
        .stable_state = 6,
        .trans_state = 0,
        .count = 0,
        .prev_state = 0,
        .conf_thresh = {
                .basic = {0.5000, 0.6000, 0.3500, 0.8000, 0.8000},
                .drive = {0.5000, 0.6000, 0.6000, 0.9000, 0.9000},
                .bike = {0.7500, 0.6000, 0.6000, 0.9000, 0.9000}
        },
        ._mat = {
                .basic = {{1,  5,  5,  15,  25,  5},
                          {5,  1,  5,  15,  25,  5},
                          {6,  5,  1,  15,  20,  8},
                          {5,  5,  10, 1,   120, 20},
                          {15, 10, 10, 120, 1,   15},
                          {3,  4,  4,  15,  25,  1}
                },
                .drive = {
                        {1,  5,  5,  15,  25,  5},
                        {5,  1,  5,  15,  25,  5},
                        {6,  5,  1,  3,   120, 8},
                        {5,  5,  10, 1,   120, 20},
                        {15, 10, 5,  120, 1,   15},
                        {3,  4,  4,  15,  25,  1}
                },
                .bike = {
                        {1,  5,  5,  15,  25,  5},
                        {5,  1,  5,  15,  25,  5},
                        {6,  5,  1,  120, 3,   8},
                        {5,  5,  10, 1,   120, 20},
                        {20, 10, 5,  120, 1,   15},
                        {3,  4,  4,  15,  25,  1},
                },

        },

};

struct heu motion_recognition_heu = {
        .initialize = -1,
        .bike_vs_drive = -1,
        .exit = -1,
        .stationary_count = 0,
};

struct activity_logic motion_recognition_bike_logic = {
        .prob_threshold = 0.5000,
        .history_length = 150,
        .X1 = 6,
        .X2 = 85,
        .max_timer = 120,
        .state_id = 8,
};

struct activity_logic motion_recognition_drive_logic = {
        .prob_threshold = 0.5000,
        .history_length = 180,
        .X1 = 6,
        .X2 = 50,
        .max_timer = 150,
        .state_id = 7,
};


struct motion_recognition_model motion_recognition_models = {
        .cmc_mean = {
                {{4.854772, 3.524711, 3.739142, 654.769104, 0.022034}, {5.883017,  2.321516, 6.194081,  1251.258423, 0.019369}, {3.938234,  4.717549, 2.576207,  371.690277,  0.028654}, {4.809341,  3.821678, 3.385241, 552.002747,  0.016088}, {4.401734,  3.481113, 3.687108, 678.320679,  0.016588}, {NaN, NaN, NaN, NaN, NaN},                             {4.918518,  3.487828, 3.767523,  627.609131,  0.017441}, {NaN, NaN, NaN, NaN, NaN},                            {2.968322, 4.526753, 2.347062, 383.703583, 0.025767}},
                {{NaN, NaN, NaN, NaN, NaN},                            {11.468448, 1.328080, 11.318364, 2445.872070, 0.021021}, {12.547316, 1.136692, 14.960858, 2177.664795, 0.011376}, {13.114182, 1.797396, 9.687081, 1851.992065, 0.012616}, {11.190663, 1.764033, 8.708049, 1801.729980, 0.021041}, {NaN, NaN, NaN, NaN, NaN},                             {13.879852, 1.677247, 10.570487, 2042.744873, 0.009056}, {NaN, NaN, NaN, NaN, NaN},                            {NaN, NaN, NaN, NaN, NaN}},
                {{NaN, NaN, NaN, NaN, NaN},                            {NaN, NaN, NaN, NaN, NaN},                               {0.852451,  6.517031, 1.122217,  156.416946,  0.020306}, {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN},                              {0.055200, 10.823608, 0.055390, 12.345848,  0.029220}, {NaN, NaN, NaN, NaN, NaN},                               {NaN, NaN, NaN, NaN, NaN},                            {NaN, NaN, NaN, NaN, NaN}},
                {{0.536870, 7.309655, 0.547783, 102.362724, 0.024735}, {0.403314,  7.817362, 0.432425,  76.689987,   0.024001}, {0.887601,  6.583887, 0.998404,  193.707413,  0.024179}, {0.491153,  8.376285, 0.437805, 93.768578,   0.026392}, {0.494975,  7.937241, 0.429965, 81.972610,   0.026533}, {0.528885, 8.317327,  0.413183, 94.579575,  0.026638}, {0.460796,  7.705909, 0.465116,  80.239807,   0.024594}, {NaN, NaN, NaN, NaN, NaN},                            {NaN, NaN, NaN, NaN, NaN}},
                {{1.621962, 4.858696, 1.706692, 366.968414, 0.028728}, {3.545390,  4.016984, 2.965037,  512.414062,  0.027156}, {NaN, NaN, NaN, NaN, NaN},                               {2.079077,  5.012655, 1.809147, 390.280518,  0.026613}, {NaN, NaN, NaN, NaN, NaN},                              {0.825935, 5.307390,  1.198076, 298.988464, 0.026858}, {2.288171,  3.837270, 2.648639,  474.765656,  0.028673}, {1.834844, 4.066715, 2.218115, 526.048767, 0.026836}, {NaN, NaN, NaN, NaN, NaN}}
        },
// 5X9X5   (num_activities=5,num_device_positions=9, cmc_feature_dim=5;)

        .cmc_cov = {
                {{0.296106, 2.326230, 0.871082, 0.000017, 12939.944336}, {0.263292, 0.923491, 0.289786, 0.000006, 20555.404297}, {0.393392, 1.179149, 1.137247, 0.000066, 10739.690430}, {0.374440, 2.493021, 1.873367, 0.000068, 18235.009766}, {0.492629, 5.363084, 1.770644,  0.000052, 21343.763672}, {NaN, NaN, NaN, NaN, NaN},                                    {0.324244, 4.101271, 1.063676, 0.000043, 16392.281250}, {NaN, NaN, NaN, NaN, NaN},                              {0.497765, 1.732615, 1.486946, 0.000076, 24060.537109}},
                {{NaN, NaN, NaN, NaN, NaN},                              {0.075499, 6.735893, 0.102828, 0.000002, 12170.102539}, {0.023661, 8.829761, 0.032838, 0.000002, 18865.669922}, {0.055397, 5.340536, 0.170859, 0.000004, 19715.054688}, {0.148014, 7.254674, 0.426312,  0.000009, 19284.451172}, {NaN, NaN, NaN, NaN, NaN},                                    {0.030062, 3.440040, 0.072474, 0.000002, 25136.941406}, {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN}},
                {{NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN},                              {0.493344, 0.196314, 0.383740, 0.000032, 25958.800781}, {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN},                               {1037.489258, 3.934417, 1166.955444, 0.024032, 24837.619141}, {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN}},
                {{4.427688, 0.619220, 7.621622, 0.000307, 25708.201172}, {6.639340, 0.536509, 8.981606, 0.000453, 21245.865234}, {1.112780, 0.297246, 0.757399, 0.000050, 21809.167969}, {3.351889, 0.378634, 4.215653, 0.000143, 18658.970703}, {9.409203, 1.115231, 19.403515, 0.001286, 35419.511719}, {7.015355,    0.944568, 16.648371,   0.000376, 27608.707031}, {5.726044, 0.582432, 8.133102, 0.000527, 24286.740234}, {NaN, NaN, NaN, NaN, NaN},                              {NaN, NaN, NaN, NaN, NaN}},
                {{0.796910, 1.037762, 1.100837, 0.000028, 30704.783203}, {0.158650, 1.130558, 0.549122, 0.000015, 27733.564453}, {NaN, NaN, NaN, NaN, NaN},                              {0.532770, 1.033826, 1.113798, 0.000034, 26122.263672}, {NaN, NaN, NaN, NaN, NaN},                               {2.294380,    0.974554, 1.885798,    0.000040, 44659.082031}, {0.378369, 0.961384, 0.672104, 0.000019, 24337.011719}, {0.897372, 1.198789, 1.007742, 0.000021, 30375.169922}, {NaN, NaN, NaN, NaN, NaN}}
        },

        .cmc_pcomponent = {
                {{1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}},
                {{NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN}},
                {{NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN}},
                {{1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {NaN, NaN, NaN, NaN, NaN}},
                {{1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN},                          {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {1.000000, 1.000000, 1.000000, 1.000000, 1.000000}, {NaN, NaN, NaN, NaN, NaN}}
        },

        .cmc_priors = {
                {0.037037, 0.037037, 0.037037, 0.037037, 0.037037, 0.000000, 0.037037, 0.000000, 0.037037},
                {0.000000, 0.037037, 0.037037, 0.037037, 0.037037, 0.000000, 0.037037, 0.000000, 0.000000},
                {0.000000, 0.000000, 0.037037, 0.000000, 0.000000, 0.037037, 0.000000, 0.000000, 0.000000},
                {0.037037, 0.037037, 0.037037, 0.037037, 0.037037, 0.037037, 0.037037, 0.000000, 0.000000},
                {0.037037, 0.037037, 0.000000, 0.037037, 0.000000, 0.037037, 0.037037, 0.037037, 0.000000}
        },

// 5X9 (num_activities=5,num_device_positions=9)
        . cmc_win = {
                20,
                20,
                20,
                200,
                200
        },
        .lr_win = {200},
        .lr_weights = {-0.0436,
                       -23.4450,
                       23.3643,
                       16.2856,
                       5.8593,
                       5.4389,
                       -22.9893,
                       -8.7583,
                       1.7517},  //9X1 lr_feature_dim=8; (1 bias)
        .lr2_win = {200},
        .lr2_weights = {1.6103,
                        -11.7477,
                        -27.3008,
                        -34.7778,
                        1.9607,
                        -0.5382,
                        0.5459,
                        0.0144,
                        -3.8637,
                        -21.1773,
                        -26.9580,
                        13.0379,
                        0.8251,
                        0.3716,
                        0.0231,
                        20.5378,
                        39.5409,
                        59.1935,
                        -15.8615,
                        -0.4092,
                        -0.7803,
                        -0.1426},
// 22 ( 1 bias)

//  %add consistent gravity back if want to save power consumption
//  %persistent gravity_buffer;
//persistent alphas;    // %alphas for filters (HMM / IIR)

        .alphas = {0, 0, 0.6000, 0.2000, 0.2000},

// load activity (num_activities)  5 X 1
        .filter_probabilities_iir = {
                {0.5000, 0.1500, 0.1500, 0.2000, 0.1875},
                {0.1250, 0.4000, 0.1500, 0.2000, 0.1875},
                {0.1250, 0.1500, 0.4000, 0.2000, 0.1875},
                {0.1250, 0.1500, 0.1500, 0.4000, 0.1875},
                {0.1250, 0.1500, 0.1500, 0.2000, 0.2500}

        },
        .unknown_confidence_threshold = {
                {0,          0.10000000, 0.15000001,  0.15000001,  0.15000001},
                {0.10000000, 0,          0.15000001,  0.15000001,  0.15000001},
                {0.15000001, 0.15000001, 0,           0.01500000,  0.15000001},
                {0.15000001, 0.15000001, 0.01500000,  0,           0.15000001},
                {0.15000001, 0.15000001, 0.15000001,  0.15000001,  0}},  // load  5 activity  X  5 activity
        .likelihood_regularization = 1.0000e-7,

        .linear_accel_time_constant = 307,
        .use_max_for_components = {
                {0, 0, 0, 0, 0, 0, 0, 0, 1},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0}

        }, // load 5 activity   X  9 device position

        .drive_entry_window = 6,    // load
        .drive_entry_threshold = 0.9, //load
        .chebyshev_a = {
                1.0000, -0.7498, 1.0725, -0.5598, 0.2337
        }, // 1X5   fengmu (chebyshev_dim)
        .chebyshev_b = {
                0.0555, 0.2221, 0.3331, 0.2221, 0.0555
        },
        .lr1vall_weights = {
                {10.7697,  0.2143,  -1.0926, -0.8591, 0.0204,  -4.9599,  0.2758,   2.2455,  0.1654,  -0.9757, 1.0367,  -2.9299, -2.1619, -2.7542, 4.6173,  -3.1331, 1.8927,  -12.3731, -1.8251, -0.0493, 0.0116,  4.5128,  0.5936,  1.6656,   -2.5058, 0.6691,  0.0593,  0.0007,  0.7853,  0.1202,  6.9516,  -3.1132, -1.1009, -0.1200, -0.1043},
                {-4.2935,  -0.1408, 0.2213,  0.2722,  0.0248,  -0.6738,  -1.6015,  3.4872,  0.3358,  0.9191,  0.9168,  -2.4283, -3.0369, -4.1147, -4.2721, 2.0041,  0.0246,  1.1481,   1.7858,  -0.2600, -0.0935, -1.3602, -1.3926, 0.3405,   -0.6774, -0.3631, 0.0562,  -0.0039, -3.0094, 0.7305,  -0.0697, -0.7265, 0.4843,  0.1064,  0.0085},
                {-7.2083,  -1.0260, 0.1198,  0.6405,  -0.0152, -10.2193, -10.1487, 5.3541,  0.7841,  2.2300,  2.6741,  0.5559,  -2.9722, 0.5728,  -6.3944, 6.6352,  -9.7208, -8.1473,  -3.8151, -0.0179, -0.0010, 0.8271,  14.6541, -22.5269, -1.5648, -4.2852, 0.1207,  -0.0071, 2.9615,  6.0128,  -9.9600, 0.3950,  -0.2832, 0.6580,  -0.0030},
                {4.9397,   0.2575,  -0.6271, -2.0659, 0.0046,  0.5840,   3.8134,   1.1534,  0.0306,  -3.8705, -3.3676, -4.0262, -0.4168, 1.6051,  4.3959,  -2.1837, 1.5610,  2.2438,   0.4670,  -0.2147, 0.0117,  1.9003,  -3.1113, 6.1941,   6.3566,  -0.2518, -0.0350, 0.0119,  3.4423,  -1.7194, 1.1330,  1.2808,  -0.0153, -0.0243, -0.0144},
                {-10.1313, 0.4291,  -1.4091, -1.6740, -0.0405, 19.7026,  0.2392,   -7.8014, -0.4017, 0.9046,  -0.4127, 6.7076,  6.3638,  4.9506,  -7.3903, 2.6417,  -5.4078, 19.3399,  -0.1145, 0.4291,  -0.0160, -5.5870, 7.7822,  -4.2826,  -0.1421, -0.2915, -0.0156, 0.0023,  -4.0857, -2.1453, -8.1819, 19.6222, 0.6073,  0.2601,  0.0695}

        },  // 5X35 ( bias)

        .lr1vall_priors = {0.1001,
                           0.0161,
                           0.1435,
                           0.5091,
                           0.2312},  // 5X activiry
};

void motion_recognition_persistent_init(struct motion_recognition_persistent_state *persistent)
{
    motion_recognition_matrix_create(persistent, &persistent->accel_buffer3X200, SENSOR_DIM, ACMC_START);
    motion_recognition_matrix_create(persistent, &persistent->filtered_accel_buffer3X200, SENSOR_DIM, ACMC_START);
    motion_recognition_matrix_create(persistent, &persistent->prob_matrix, NUM_ACTIVITIES, NUM_DEVICE_POSITIONS);
    motion_recognition_matrix_create(persistent, &persistent->downsampled_data20X3, ACMC_FREQ, SENSOR_DIM);
    motion_recognition_persistent_reset_state(persistent);
    persistent->linear_accel_alpha =
        1 - expf(-1000 / ACMC_FREQ / persistent->models->linear_accel_time_constant);
}

void motion_recognition_persistent_reset_state(struct motion_recognition_persistent_state *persistent)
{
    persistent->acmc_ready = false;
    persistent->hyst = &motion_recognition_hyst;
    persistent->hyst->hmode = 0,
    persistent->hyst->stable_state = 6,
    persistent->hyst->trans_state = 0,
    persistent->hyst->count = 0,
    persistent->hyst->prev_state = 0,

    persistent->heu = &motion_recognition_heu;
    persistent->heu->initialize = -1;
    persistent->heu->bike_vs_drive = -1;
    persistent->heu->exit = -1;
    persistent->heu->stationary_count = 0;

    persistent->in_drive = 0;
    persistent->in_bike = 0;
    persistent->filter_condition_empty = true;
    persistent->filtered_accel_buffer3X200_empty = true;
    persistent->bike_logic = &motion_recognition_bike_logic;
    persistent->drive_logic = &motion_recognition_drive_logic;
    persistent->models = &motion_recognition_models;
    persistent->accel_buffer_ptr = 0;
    persistent->feature_cnt = 0;
    motion_recognition_matrix_zeros(&persistent->accel_buffer3X200);
    motion_recognition_matrix_zeros(&persistent->filtered_accel_buffer3X200);
    motion_recognition_matrix_zeros(&persistent->prob_matrix);
    motion_recognition_matrix_zeros(&persistent->downsampled_data20X3);
    motion_recognition_array_zero(persistent->cmc_features, CMC_FEATURE_DIM);
    motion_recognition_array_zero(persistent->lr_features, LR_FEATURE_DIM);
    motion_recognition_array_zero(persistent->lr2_features, LR2_FEATURE_DIM);
    motion_recognition_array_zero(persistent->lr1vall_features, LR1VALL_FEATURE_DIM);

    for (int i = 0; i < LABELS_NUM_ROWS * LABELS_NUM_COLUMNS; i++) {
        persistent->classify_data_out.labels[i] = motion_recognition_labels[i];
    }
    for (int i = 0; i < PROBABILITIES_NUM_ROWS * PROBABILITIES_NUM_COLUMNS; i++) {
        persistent->classify_data_out.probabilities[i] = 0.0;
    }
}

void motion_recognition_persistent_deinit(struct motion_recognition_persistent_state *persistent)
{
    motion_recognition_matrix_delete(persistent, &persistent->accel_buffer3X200);
    motion_recognition_matrix_delete(persistent, &persistent->filtered_accel_buffer3X200);
    motion_recognition_matrix_delete(persistent, &persistent->prob_matrix);
    motion_recognition_matrix_delete(persistent, &persistent->downsampled_data20X3);
}



