/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - light_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>                      <desc>
*******************************************************************/
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "light_algo.h"
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
static lb_algo_state g_lb_state;

lb_dbv_atime_coefs lb_dbv_coefs = {
    .atime_1 = 1248, //20000,
    .atime_2 = 864,
    .atime_3 = 480,
    .atime_4 = 380,
    .atime_h = 2962,
    .atime_50ms = 50000,
    .atime_comp_algo = 20000,
    .dbv_1 = 180,
    .dbv_2 = 250,
    .dbv_3 = 320,
    .dbv_4 = 319,
    .dbv_l2h = 350,
    .dbv_h2l = 320,
    .hba_dbv_thres = 270
};

#ifdef GOERTZEL_COMPENSATION
lb_goertzel_coefs lb_gor_coefs[3] = {
    {   //m100
        .sine_1 = 916922531,
        .cos_1 = 558725851,
        .gor_len_1 = 62,
        .sine_2 = 756861128,
        .cos_2 = 761631629,
        .gor_len_2 = 80,
        .sine_3 = 552953666,
        .cos_3 = 920414987,
        .gor_len_3 = 116,
        .sine_4 = 552953666,
        .cos_4 = 920414987,
        .gor_len_4 = 116,
        .sine_h = 992008094,
        .cos_h = -410903207,
        .gor_len_h = 32
    },
    {   //m150
        .sine_1 = 1073065304,
        .cos_1 = 38109826,
        .gor_len_1 = 42,
        .sine_2 = 990060751,
        .cos_2 = 415573355,
        .gor_len_2 = 54,
        .sine_3 = 778787396,
        .cos_3 = 739196656,
        .gor_len_3 = 78,
        .sine_4 = 778787396,
        .cos_4 = 739196656,
        .gor_len_4 = 78,
        .sine_h = 209476638,
        .cos_h = -1053110176,
        .gor_len_h = 32
    },
    {   //m200
        .sine_1 = 954248610,
        .cos_1 = -492271363,
        .gor_len_1 = 30,
        .sine_2 = 1073720629,
        .cos_2 = 6746474,
        .gor_len_2 = 40,
        .sine_3 = 947987365,
        .cos_3 = 504223621,
        .gor_len_3 = 58,
        .sine_4 = 947987365,
        .cos_4 = 504223621,
        .gor_len_4 = 58,
        .sine_h = -759250125,
        .cos_h = -759250125,
        .gor_len_h = 32
    }


};

static int lb_algo_average_count_mag = 0;
static float lb_algo_average_m100 = 0.0;
static float lb_algo_average_m150 = 0.0;
static float lb_algo_average_m200 = 0.0;
static float lb_algo_average_k1 = 0.0;
static float lb_algo_m100_history_values[LB_MEDIAN_SIZE_MAG];
static float lb_algo_m150_history_values[LB_MEDIAN_SIZE_MAG];
static float lb_algo_m200_history_values[LB_MEDIAN_SIZE_MAG];
static float lb_algo_m100_history_ages[LB_MEDIAN_SIZE_MAG];
static float lb_algo_m150_history_ages[LB_MEDIAN_SIZE_MAG];
static float lb_algo_m200_history_ages[LB_MEDIAN_SIZE_MAG];
#endif

lb_display_coefs lb_dis_coefs = {
    .lb_channels = CHANNEL_COUNT,
    .coef_a = 1.6,
    .coef_b = 0.972,
    .coef_c = 1.35,
    .coef_d = 3.661,
    .coef_50ms =  0.368,
    .coef_k = 0.0222,
    .coef_k1 = 0.2001,
    .dgf = 0.528,
    .dgf_50ms = 2.097379,
    .dbv_coefs = &lb_dbv_coefs,
#ifdef GOERTZEL_COMPENSATION
    .mag_lb_k3 = 1.0,
    .mag_lb_k4 = 0.6,
    .mag_hb_k3 = 1.0,
    .mag_hb_k4 = 0.6,
    .mag_lb_k1_thres = 4,
    .mag_lb_k2_thres = 20,
    .mag_hb_k1_thres = 0.5,
    .mag_hb_k2_thres = 0.8,
    .gor_coefs = {lb_gor_coefs}
#endif
};

#ifdef GOERTZEL_COMPENSATION
static int lb_algo_rounded_divide(long long a, int b)
{
    return (a > 0 ? a + (abs(b) >> 1) : a - (abs(b) >> 1)) / b;
}

static int lb_algo_sqrt_long_long(long long x)
{
    long long seed = 1, rr = x >> 2;
    if (x == 0) {
        return 0;
    }
    while (rr) {
        rr >>= 2;
        seed <<= 1;
    }
    seed = (seed + x / seed) >> 1;
    seed = (seed + x / seed) >> 1;
    seed = (seed + x / seed) >> 1;
    seed = (seed + x / seed) >> 1;
    return seed;
}

static int lb_algo_goertzel_mag(struct lb_sliding_goertzel *sg, int new_value)
{
    int real;
    int imag;
    int magnitude;

    int q0 = lb_algo_rounded_divide((long long)2 * sg->cosine * sg->q1, 1 << sg->frac_bits) - sg->q2 + new_value;

    sg->q2 = sg->q1;
    sg->q1 = q0;

    real = (sg->q1 - lb_algo_rounded_divide((long long)sg->q2 * sg->cosine, 1 << sg->frac_bits)) * 2.0 / sg->length;
    imag = lb_algo_rounded_divide((long long)sg->q2 * sg->sine, 1 << sg->frac_bits) * 2.0 / sg->length;
    magnitude = lb_algo_sqrt_long_long((long long)real * real + (long long)imag * imag);
    return magnitude;
}

static int lb_comb_filter(struct lb_sliding_goertzel *sg, int new_value)
{
    int ret = new_value - sg->comb_history[0];    // Return value of the function
    int i;

    for (i = 0; i < sg->length - 1; i++) {    // A circular buffer would be more efficient than this
        sg->comb_history[i] = sg->comb_history[i + 1];    // Shift all the values
    }
    sg->comb_history[sg->length - 1] = new_value;    // Store the new value
    return ret;
}

static int lb_algo_goertzel_step(struct lb_sliding_goertzel *sg, int new_value)
{
    return lb_algo_goertzel_mag(sg, lb_comb_filter(sg, new_value));
}

int lb_algo_get_insertion_index(float *ages, int len) {
    int i;
    for (i = 0; i < len; i++) {
        // Replace an invalid value index or the oldest value index
        if ((0 == ages[i]) || (len+1 == ages[i])) {
            return i;
        }
    }
    return 0; // We should never reach this return statement
}

void lb_algo_swap(float *a, float *b) {
    int temp = *a; // We are about to overwrite 'a'
    *a = *b;
    *b = temp;
}

int lb_algo_goertze_lmedian(float *data, int len) {
    if (len & 1) { // There is an odd number of elements
        return data[len >> 1];
    } else { // Average of the 2 middle elements
        return (data[len >> 1] + data[(len >> 1) - 1] + 1) / 2;
    }
}

float lb_algo_streaming_median(float new_val, float *values, float *ages, int len) {
    int i;
    int num_valid = len; // Initially assume that all stored values are valid
    int index;
    for (i = 0; i < len; i++) {
        if (0 != ages[i]) ages[i]++; // An age of '0' means there is not a valid value for that index
        else num_valid--; // We have one fewer valid values than initially assumed
    } // All of the values are now older by one sample

    index = lb_algo_get_insertion_index(ages, len); // We want to replace the oldest value with our new value
    ages[index] = 1; // Indicate that the value at index 'index' is new - '1' is the lowest valid age
    values[index] = new_val; // Store the new value in it's proper location
    if (num_valid < len) num_valid++; // We just replaced an invalid value index

    while ((index > 0) && (values[index] > values[index - 1])) {
        lb_algo_swap(&values[index], &values[index - 1]); // Swap the values
        lb_algo_swap(&ages[index], &ages[index - 1]); // Swap the ages corresponding to each value
        index--; // Now check the next value down
    }

    while ((index < len - 1) && (values[index] < values[index + 1])) {
        lb_algo_swap(&values[index], &values[index + 1]); // Swap the values
        lb_algo_swap(&ages[index], &ages[index + 1]); // Swap the ages corresponding to each value
        index++; // Now check the next value up
    }

    return lb_algo_goertze_lmedian(values, num_valid);
}

float lb_algo_streaming_average(float average, float new_val, int count) {
    return (average * (count - 1) + new_val) / count;
}

float lb_algo_get_magnitude(uint32_t *data, int len, uint32_t atime, int target_freq)
{
    struct lb_sliding_goertzel sg_struct;    // Store details about the Goertzel filter
    int comb_history[128] = {
        0
    }; // Comb filter history for the Goertzel filter
/*  the way to calculate sg_struct.sine and sg_struct.cosine
    sg_struct.sine = GOERTZEL_SINE;    // sin((2.0*PI*(int)(0.5 + ((length * target_freq) / sampling_freq))) / length)
    sg_struct.cosine = GOERTZEL_COSINE;    // cos((2.0*PI*(int)(0.5 + ((length * target_freq) / sampling_freq))) / length)
*/
    int i;
    float magnitude;
    int goertzel_len = 64;

    if (FIT_ATIME(atime, lb_dis_coefs.dbv_coefs->atime_1)) {
        sg_struct.sine = lb_gor_coefs[target_freq].sine_1;
        sg_struct.cosine = lb_gor_coefs[target_freq].cos_1;
        goertzel_len = lb_gor_coefs[target_freq].gor_len_1;
    } else if (FIT_ATIME(atime, lb_dis_coefs.dbv_coefs->atime_2)) {
        sg_struct.sine = lb_gor_coefs[target_freq].sine_2;
        sg_struct.cosine = lb_gor_coefs[target_freq].cos_2;
        goertzel_len = lb_gor_coefs[target_freq].gor_len_2;
    } else if (FIT_ATIME(atime, lb_dis_coefs.dbv_coefs->atime_3)) {
        sg_struct.sine = lb_gor_coefs[target_freq].sine_3;
        sg_struct.cosine = lb_gor_coefs[target_freq].cos_3;
        goertzel_len = lb_gor_coefs[target_freq].gor_len_3;
    } else if (FIT_ATIME(atime, lb_dis_coefs.dbv_coefs->atime_4)) {
        sg_struct.sine = lb_gor_coefs[target_freq].sine_4;
        sg_struct.cosine = lb_gor_coefs[target_freq].cos_4;
        goertzel_len = lb_gor_coefs[target_freq].gor_len_4;
    } else if (FIT_ATIME(atime, lb_dis_coefs.dbv_coefs->atime_h)) {
        sg_struct.sine = lb_gor_coefs[target_freq].sine_h;
        sg_struct.cosine = lb_gor_coefs[target_freq].cos_h;
        goertzel_len = lb_gor_coefs[target_freq].gor_len_h;
    }

    //SNS_INST_PRINTF(HIGH, instance, " goertzel_len: %d", goertzel_len);

    sg_struct.length = goertzel_len;
    sg_struct.q1 = 0;    // Clear intermediate variable 1
    sg_struct.q2 = 0;    // Clear intermediate variable 2
    sg_struct.frac_bits = LB_GOERTZEL_FRAC_BITS;

    sg_struct.comb_history = comb_history;

    for (i = 0; i < len; i++) {
        magnitude = lb_algo_goertzel_step(&sg_struct, data[i] * (1 << LB_ALG_FRAC_BITS));
        magnitude /= (1 << LB_ALG_FRAC_BITS);    // Remove the fixed point fractional multiplier
    }
    return magnitude;
}

bool lb_algo_flicker_detection(lb_algo_state *state, uint16_t *data, int len, uint16_t ch0_low)
{
    static float k1 = 0.0, k2_100 = 0.0, k2_150 = 0.0, k2_200 = 0.0;
    static uint32_t raw_ch0_raw[LB_GOERTZEL_DATA_SIZE] = {0};
    bool algo_updated = false;
    float k1_thres = 0.0, k2_thres = 0.0;
    float median_m100, median_m150, median_m200;
    int i;
    float temp_ch = 0;

    //calculate the average value of c/r/g/b channel
    for (int i = 0;i < CHANNEL_COUNT; i++)
        state->gor_info.lb_ch_avg[i] = state->gor_info.ch_sum[i] / len;
    //SNS_INST_PRINTF(HIGH, instance, "TCS3701 LB FIFO avg Ch0: %d, Ch1: %d, Ch2: %d, Ch3: %d",
    //    state->als_gor_info.lb_ch0_avg, state->als_gor_info.lb_ch1_avg, state->als_gor_info.lb_ch2_avg, state->als_gor_info.lb_ch3_avg);

    for (i = 0; i < len; i++) {
        raw_ch0_raw[state->gor_info.gor_samples] = (uint32_t)data[i];// * 1000 * 549 / (state->atime_us * state->again);    //normalize to same 549x again and 1000us atime
        state->gor_info.gor_samples++;
        if (state->gor_info.gor_samples == LB_GOERTZEL_DATA_SIZE) {
            //for (i = 0; i < state->gor_info.gor_samples ; i++) {
            //    ALSPS_LOG("raw_ch0_raw[%d]=%d",i,raw_ch0_raw[i]);
            //}
            state->gor_info.gor_samples = 0;
            state->gor_info.magnitude_100hz =lb_algo_get_magnitude(raw_ch0_raw, LB_GOERTZEL_DATA_SIZE, state->atime_us, LB_FREQ_100HZ);
            state->gor_info.magnitude_150hz = lb_algo_get_magnitude(raw_ch0_raw, LB_GOERTZEL_DATA_SIZE, state->atime_us, LB_FREQ_150HZ);
            state->gor_info.magnitude_200hz = lb_algo_get_magnitude(raw_ch0_raw, LB_GOERTZEL_DATA_SIZE, state->atime_us, LB_FREQ_200HZ);

            median_m100 = lb_algo_streaming_median(state->gor_info.magnitude_100hz * 100,
                    lb_algo_m100_history_values,
                    lb_algo_m100_history_ages,
                    LB_MEDIAN_SIZE_MAG);
            median_m150 = lb_algo_streaming_median(state->gor_info.magnitude_150hz * 100,
                    lb_algo_m150_history_values,
                    lb_algo_m150_history_ages,
                    LB_MEDIAN_SIZE_MAG);
            median_m200 = lb_algo_streaming_median(state->gor_info.magnitude_200hz * 100,
                    lb_algo_m200_history_values,
                    lb_algo_m200_history_ages,
                    LB_MEDIAN_SIZE_MAG);
            //ALSPS_LOG("median_m100*100 = %d, median_m150*100 = %d, median_m200*100 = %d",
            //        (uint32_t)(median_m100*100), (uint32_t)(median_m150*100), (uint32_t)(median_m200*100));
            if (lb_algo_average_count_mag < LB_AVERAGE_LEN_MAG) {
                lb_algo_average_count_mag++;
            }
            lb_algo_average_m100 = lb_algo_streaming_average(lb_algo_average_m100, median_m100, lb_algo_average_count_mag);
            state->gor_info.magnitude_100hz = lb_algo_average_m100/100;
            lb_algo_average_m150 = lb_algo_streaming_average(lb_algo_average_m150, median_m150, lb_algo_average_count_mag);
            state->gor_info.magnitude_150hz = lb_algo_average_m150/100;
            lb_algo_average_m200 = lb_algo_streaming_average(lb_algo_average_m200, median_m200, lb_algo_average_count_mag);
            state->gor_info.magnitude_200hz = lb_algo_average_m200/100;

            //ALSPS_LOG("lb_algo_average_m100 = %d, lb_algo_average_m150 = %d, lb_algo_average_m200 = %d",
            //        (uint32_t)(lb_algo_average_m100), (uint32_t)(lb_algo_average_m150), (uint32_t)(lb_algo_average_m200));
            //condition 1 AC/DC > ? AC = average - lowest   DC = lowest
            k1 = ((float)(state->gor_info.lb_ch_avg[0]) - (float)ch0_low)/((float)ch0_low + 1.0);

            lb_algo_average_k1 = lb_algo_streaming_average(lb_algo_average_k1, k1*100, lb_algo_average_count_mag);
            k1 = lb_algo_average_k1/100;
            //condition 2
            //k2 = state->als_gor_info.magnitude_100hz/((float)state->als_gor_info.lb_ch0_avg - (float)ch0_low);
            k2_100 = state->gor_info.magnitude_100hz;
            k2_150 = state->gor_info.magnitude_150hz;
            k2_200 = state->gor_info.magnitude_200hz;
            ALSPS_LOG("k1*100 = %d, k2_100*100 = %d, k2_150*100 = %d, k2_200*100 = %d",
                    (uint32_t)(k1*100), (uint32_t)(k2_100*100), (uint32_t)(k2_150*100), (uint32_t)(k2_200*100));

            algo_updated = true;
            k1_thres = lb_dis_coefs.mag_lb_k1_thres;
            k2_thres = lb_dis_coefs.mag_lb_k2_thres;
	    temp_ch = ch0_low * 1000 * 1057 /(state->again * state->atime_us);
	    if (temp_ch < 5) {
                k1_thres = lb_dis_coefs.mag_lb_k1_thres/2;
                k2_thres = lb_dis_coefs.mag_lb_k2_thres/2;
                if (k1 >= lb_dis_coefs.mag_lb_k1_thres) {
                    k2_thres = lb_dis_coefs.mag_lb_k2_thres/4;
                }
	    }

            if ((k1 > k1_thres) && ((k2_100 > k2_thres) | (k2_150 > k2_thres) | (k2_200 > k2_thres))) {//this value has to be adjusted based on the actual value
                if (!state->gor_info.use_average) {
                    ALSPS_LOG("use average");
                    state->gor_info.use_average = true;
                } else {
                    algo_updated = false;
                }
            } else if (state->gor_info.use_average){
                ALSPS_LOG("use raw channel counts");
                state->gor_info.use_average = false;
            } else {
                algo_updated = false;
            }
        }
    }
    if (algo_updated) {
        ALSPS_LOG("lb algo updated");
    }
    return algo_updated;
}

#endif

uint16_t lb_algo_median_filter(uint16_t data[], uint8_t count)
{
    int8_t i, j;
    uint16_t temp;
    for (i = 1; i < count; i++)
    {
        temp = data[i];
        j = i-1;

        while(data[j] > temp && j >= 0)
        {
            data[j+1] = data[j];
            j--;
        }

        if (j != (i-1))
        {
            data[j+1] = temp;
        }
    }

    return data[(count-1)/2];
}

void lb_algo_median(uint16_t* lb_data, uint8_t length)
{
    uint16_t raw_data[GROUP_CNT];
    uint16_t data[LB_ALGO_MIDIAN_LENGTH];
    uint8_t half = (LB_ALGO_MIDIAN_LENGTH-1)/2;
    uint8_t i = 1;

    memcpy(raw_data, lb_data, sizeof(raw_data));
    memcpy(&data[0], &raw_data[0], sizeof(data));
    lb_data[0] = lb_algo_median_filter(data, LB_ALGO_MIDIAN_LENGTH);
    for (i = 1; i <= half; i++) {
        lb_data[i] = lb_data[0];
    }
    for (i = (half+1); i < (length-half-1); i++) {
        memcpy(&data[0], &raw_data[i-half], sizeof(data));
        lb_data[i] = lb_algo_median_filter(data, LB_ALGO_MIDIAN_LENGTH);
    }
    memcpy(&data[0], &raw_data[length-LB_ALGO_MIDIAN_LENGTH], sizeof(data));
    lb_data[length-1] = lb_algo_median_filter(data, LB_ALGO_MIDIAN_LENGTH);
    for (i = 1; i <= half; i++) {
        lb_data[length-1-i] = lb_data[length-1];
    }
}

bool als_lb_algo(uint8_t* data_buf, uint16_t length, uint8_t* algo_adjust,
    uint16_t atime_us, uint16_t again, uint16_t ch_atg[], float ch_raw_min[], uint8_t* get_data)
{
    lb_algo_state *state = &g_lb_state;

    static uint16_t raw_data[GROUP_BUF_SIZE] = {0};
    uint8_t group_buf_len = 0;
    static uint16_t ch_min_buffer[CHANNEL_COUNT][GROUP_CNT];
    static uint16_t ch_max_buffer[CHANNEL_COUNT][GROUP_CNT];
    float ch0_max_tmp = 0.0, lux_coef = 0.0;
    static float ch_diff[CHANNEL_COUNT] = {0};
    uint16_t ch[CHANNEL_COUNT] = {0}, ch_min[CHANNEL_COUNT] = {0}, ch_max[CHANNEL_COUNT] = {0};
    uint32_t sum = 0, sum_max = 0, sum_min = 0;
    bool group_buf_full = false, get_sample = false;

#ifdef GOERTZEL_COMPENSATION
    uint16_t raw_als[MAX_DATA_FIFO] = {0};
    bool lb_algorithm_adjusted = false;
    static float k3 = 0.0, k4 = 0.0, k5 = 0.0;
#endif

    ALSPS_LOG("fifo length=%d", length);
    if (length > MAX_DATA_FIFO) {
        length = MAX_DATA_FIFO;
    }
    *algo_adjust = 0;

    group_buf_len = (uint8_t)((15000+(atime_us>>1)) / atime_us);//15ms

#ifdef GOERTZEL_COMPENSATION
    memset(state->gor_info.ch_sum, 0, sizeof(state->gor_info.ch_sum));
#endif

    sum_min = 0xFFFFFFFF;
    sum_max = 0;

    for (int i = 0; i < length; i++) {
        raw_data[state->group_buf_cnt] = (uint16_t)(((uint32_t)data_buf[i*2+1] << 8) | data_buf[i*2]);
        state->group_buf_cnt++;

        if (state->group_buf_cnt == (group_buf_len * CHANNEL_COUNT)) {
            state->group_buf_cnt = 0;
            group_buf_full = true;
            *get_data = 1;
            for (int j = 0; j < group_buf_len; j++) {
                sum = 0;
                memcpy(ch, &raw_data[j * CHANNEL_COUNT],sizeof(ch));
                ALSPS_LOG("raw_data, ch_als %d, ch1 %d, ch2 %d", ch[0], ch[1], ch[2]);

#ifdef GOERTZEL_COMPENSATION
                raw_als[j] = ch[0];
                for (int k = 0;k < CHANNEL_COUNT;k++) {
                    state->gor_info.ch_sum[k] += ch[k];
                }
#endif
        //for (int k = 0;k < CHANNEL_COUNT;k++)
                //    sum += ch[k];
                sum = ch[0];
                if (sum < sum_min) {
                    memcpy(ch_min, ch,sizeof(ch_min));
                    sum_min = sum;
                }
                if (sum > sum_max) {
                    memcpy(ch_max, ch,sizeof(ch_max));
                    sum_max = sum;
                }
            }
        }
    }
    if (!group_buf_full) {
        return get_sample;
    }

    memcpy(ch_atg, ch_min, sizeof(CHANNEL_COUNT));

#ifdef GOERTZEL_COMPENSATION
    if (state->gor_info.use_average) {
        memcpy(ch_atg, ch_max, sizeof(CHANNEL_COUNT));
    }
#endif

    state->again = again;
#ifdef GOERTZEL_COMPENSATION
    for (int i = 0; i < group_buf_len; i++) {
        raw_als[i] = (uint32_t)((float)raw_als[i] * 1000 * 1057/(float)(atime_us * again));
    }
    state->atime_us = atime_us;
    lb_algorithm_adjusted = lb_algo_flicker_detection(state, raw_als, group_buf_len, ch_min[0]);
    if (lb_algorithm_adjusted) {
        *algo_adjust = 1;
        return get_sample;
    }

    if (state->gor_info.use_average) {
            k3 = lb_dis_coefs.mag_lb_k3;
            k4 = lb_dis_coefs.mag_lb_k4;

#ifdef SOFT_SCREEN
            if (!FIT_ATIME(atime_us, lb_dis_coefs.dbv_coefs->atime_1)) {
                k3 = 1.0;
                k4 = 1.0;
            }
#endif

            //k5 = (k3*state->gor_info.magnitude_100hz)/(k3*state->gor_info.magnitude_100hz + k4*state->gor_info.magnitude_240hz);
            k5 = 1;
            for (int i = 0;i < CHANNEL_COUNT;i++) {
                ch_min[i] = (uint16_t)((float)state->gor_info.lb_ch_avg[i]* k5);
            }
    }
#endif //GOERTZEL_COMPENSATION

    for (int i = 0;i < CHANNEL_COUNT;i++) {
        ch_min_buffer[i][state->samples] = ch_min[i];
        ch_max_buffer[i][state->samples] = ch_max[i];
    }
    state->samples++;
    ALSPS_LOG("sample count, %d / 20, ch_min_buf %d, %d, %d",
        state->samples, ch_min[0], ch_min[1], ch_min[2]);
    /* Calcualte the Lux */
    if (state->samples == GROUP_CNT) {
        get_sample = true;

        for (int i = 0;i < CHANNEL_COUNT;i++) {
            lb_algo_median((uint16_t *)(&ch_min_buffer[i][0]), GROUP_CNT);
            lb_algo_median((uint16_t *)(&ch_max_buffer[i][0]), GROUP_CNT);
        }

        for (int i = 0;i < GROUP_CNT;i++){
            ALSPS_LOG("sample after median, sample %d, ch_min_buf %d, %d, %d",
                    i, ch_min_buffer[0][i], ch_min_buffer[1][i], ch_min_buffer[2][i]);
        }

        for (int i = 0; i < GROUP_CNT; i++) {
            for (int j = 0;j < CHANNEL_COUNT;j++) {
                state->rgbc_raw.ch_min[j] += (float)ch_min_buffer[j][i];
                state->rgbc_raw.ch_max[j] += (float)ch_max_buffer[j][i];
            }
        }

        {

            ALSPS_LOG("ch_min average, ch1 %d, ch2 %d, ch3 %d",
                    (int)(state->rgbc_raw.ch_min[0] / GROUP_CNT),
                    (int)(state->rgbc_raw.ch_min[1] / GROUP_CNT),
                    (int)(state->rgbc_raw.ch_min[2] / GROUP_CNT));
            ALSPS_LOG("ch_max average, ch1 %d, ch2 %d, ch3 %d",
                    (int)(state->rgbc_raw.ch_max[0] / GROUP_CNT),
                    (int)(state->rgbc_raw.ch_max[1] / GROUP_CNT),
                    (int)(state->rgbc_raw.ch_max[2] / GROUP_CNT));
        }

        for (int i = 0;i < CHANNEL_COUNT;i++)
            ch_diff[i] = (state->rgbc_raw.ch_max[i] - state->rgbc_raw.ch_min[i]) / GROUP_CNT;

#ifdef GOERTZEL_COMPENSATION
        if (state->gor_info.use_average)
#else
        if (0)
#endif
        {
            lux_coef = 0;
        }
        else
        {
            ALSPS_LOG("ch_diff %d, atime_us %d",(int)ch_diff[0], (int)atime_us);
            ch0_max_tmp = state->rgbc_raw.ch_max[0]/GROUP_CNT;
            ch0_max_tmp = ch0_max_tmp/atime_us* 1301;
            ch_diff[0] = ch_diff[0]/atime_us * 1301;
            ALSPS_LOG("before LB algo lux_coef %d, ch0_max_tmp %d",(int)lux_coef, (int)ch0_max_tmp);
            lux_coef = ch_diff[0]*0.025 * 15;
            //if (state->sensor_ops->get_brightness() < 88)
            lux_coef = lux_coef *0.83;
            if (ch0_max_tmp < 650 )
                lux_coef = lux_coef * ch0_max_tmp / 550;
        }

        ALSPS_LOG("LB algo cal lux, ch0 %d, ch1 %d, ch2 %d, lux_coef*1000 %d",
            (int)state->rgbc_raw.ch_min[0], (int)state->rgbc_raw.ch_min[1], (int)state->rgbc_raw.ch_min[2], (int)(lux_coef *1000));
        ALSPS_LOG("LB algo cal max lux, ch0 %d, ch1 %d, ch2 %d",
            (int)state->rgbc_raw.ch_max[0], (int)state->rgbc_raw.ch_max[1], (int)state->rgbc_raw.ch_max[2]);
        memcpy(ch_raw_min, state->rgbc_raw.ch_min, sizeof(CHANNEL_COUNT));
        ch_raw_min[3] = lux_coef;
#ifdef LB_ALGO_DEBUG
        log_data_info als_log_data;
        memset(&als_log_data, 0 , sizeof(als_log_data));
        als_log_data.sensor_id = 6;
        als_log_data.string_id = 1;
        als_log_data.argu2 = atime_us;
        als_log_data.argu3 = state->rgbc_raw.ch_min[0];
        als_log_data.argu4 = state->rgbc_raw.ch_max[0];
        als_log_data.argu5 = lux_coef * 1000;
        sensors_log_report(als_log_data);
#endif
        state->samples = 0;
        state->group_buf_cnt = 0;
        memset(state->rgbc_raw.ch_min, 0, sizeof(state->rgbc_raw.ch_min));
        memset(state->rgbc_raw.ch_max, 0, sizeof(state->rgbc_raw.ch_max));
    }

    return get_sample;
}

void lb_algo_clear_fifo()
{
#ifdef GOERTZEL_COMPENSATION
    lb_algo_average_count_mag = 0;
    lb_algo_average_m100 = 0;
    lb_algo_average_m150 = 0;
    lb_algo_average_m200 = 0;
    lb_algo_average_k1 = 0;
    for (int i = 0; i < LB_MEDIAN_SIZE_MAG; i++) {
        lb_algo_m100_history_values[i] = 0;
        lb_algo_m100_history_ages[i] = 0;
        lb_algo_m150_history_values[i] = 0;
        lb_algo_m150_history_ages[i] = 0;
        lb_algo_m200_history_values[i] = 0;
        lb_algo_m200_history_ages[i] = 0;
    }
    g_lb_state.gor_info.gor_samples = 0;
#endif
    g_lb_state.group_buf_cnt = 0;
    g_lb_state.samples = 0;
    memset(g_lb_state.rgbc_raw.ch_min, 0, sizeof(g_lb_state.rgbc_raw.ch_min));
    memset(g_lb_state.rgbc_raw.ch_max, 0, sizeof(g_lb_state.rgbc_raw.ch_max));
}
#endif//SUPPORT_LOW_BRIGHTNESS_ALGO
static int get_max_count_index(struct alsps_data *rgbw_p, int n)
{
    int count = 1;
    int max_count = 1;
    int index = 0;
    struct alsps_data rgbw[LIGHT_DATA_COUNT];

    if (n > LIGHT_DATA_COUNT) {
        return -1;
    } else {
        memcpy(rgbw, rgbw_p, sizeof(struct alsps_data) * n);
    }

    index = n - 1;

    for (int i = 0; i < n ; i++) {
        if (rgbw[i].lux < 0) {
            continue;
        }

        for (int j = i + 1; j < n; j++) {
            if (rgbw[i].lux == rgbw[j].lux) {
                count++;
                rgbw[j].lux = -rgbw[j].lux;
            }
        }

        if (count > max_count) {
            max_count = count;
            index = i;
        }

        count = 1;
    }

    ALSPS_ERR("count %d,index %d,value %d\n", count, index, rgbw_p[index].lux);
    return index;
}

struct alsps_data *get_proper_als_data(struct alsps_data *rgbw, uint8_t len)
{
    int index = 0;
    int time_delta = 0;
    struct alsps_data *rgbw_data = NULL;

    if (!rgbw || len > LIGHT_DATA_COUNT) {
        return NULL;
    }

    index = get_max_count_index(rgbw, len);

    if (index < -1) {
        return NULL;
    }

    rgbw_data = &rgbw[index];
    time_delta = (len - index) * ALS_TIME_DELTA;
    rgbw_data->time_dealta = time_delta;

    return rgbw_data;
}

