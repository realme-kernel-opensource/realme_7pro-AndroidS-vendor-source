/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - light_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>	<date>		<author>              		<desc>
*******************************************************************/
#ifndef _LIGHT_ALGO_H_
#define _LIGHT_ALGO_H_
#include "alsps_def.h"
#define LIGHT_DATA_COUNT 16
#define ALS_TIME_DELTA 12
#ifdef  SUPPORT_LOW_BRIGHTNESS_ALGO
#define EVENT_ALGO_STATE(event) (*(event + ALS_EVENT_SIZE))
#define SET_EVENT_VALID(event) (*(event + ALS_EVENT_SIZE + 1) = 1)
#define SET_EVENT_UNVALID(event) (*(event + ALS_EVENT_SIZE + 1) = 0)
#define IF_EVENT_VALID(event) (*(event + ALS_EVENT_SIZE + 1) == 1)

#define GOERTZEL_COMPENSATION
#define FIT_ATIME(actual_atime, aim_atime) ((actual_atime > (aim_atime - 5)) && (actual_atime < (aim_atime + 5)))

#define MAX_DATA_FIFO 128
#define MAX_ALS_VALUE 0xFFFF
#define CHANNEL_COUNT 3
#define ASTEP_US_PER_100 278
#define MINIMUM_ATIME_US    380
#define GROUP_BUF_SIZE      120  //(15000/MINIMUM_ATIME_US)
#define GROUP_CNT    	20
#define HB_ALGO_ATIME    	20000

#define LB_AVERAGE_LEN_MAG    	32
#define LB_FREQ_100HZ    		0
#define LB_FREQ_150HZ    		1
#define LB_FREQ_200HZ    		2
#define LB_MEDIAN_SIZE_MAG    	16
#define LB_GOERTZEL_FRAC_BITS    30	// The sine and cosine factors are multipled by (1 << GOERTZEL_FRAC_BITS)
#define LB_ALG_FRAC_BITS    	10  // The output of the Goertzel algorithm will have this number of fractional bits
#define LB_GOERTZEL_DATA_SIZE    256
#define LB_ALGO_MIDIAN_LENGTH   9
#define ALS_TIMER_MS            20
#define HB_ALS_BUF_LEN          10


enum DISPLAY_FREQUENT {
    DISPLAY_60HZ = 0,
    DISPLAY_90HZ,
    DISPLAY_120HZ
};

typedef enum ALS_ALGO {
    ALS_ALGO_UNKNOWN,
    LOW_BRIGHTNESS_ALGO,
    COMPENSATION_ALGO
} als_algo_state;

typedef enum
{
    ALGO_REGISTER,
    ALGO_RESET,
    ALGO_CLOSE,
}algo_status;

typedef struct lb_dbv_atime_coefs {
    uint16_t atime_1;
    uint16_t atime_2;
    uint16_t atime_3;
    uint16_t atime_4;
    uint16_t atime_h;
    uint16_t atime_50ms;
    uint16_t atime_comp_algo;
    int32_t dbv_1;
    int32_t dbv_2;
    int32_t dbv_3;
    int32_t dbv_4;
    int32_t dbv_l2h;
    int32_t dbv_h2l;
    int32_t dbv_240hz;
    int32_t hba_dbv_thres;
} lb_dbv_atime_coefs;

#ifdef GOERTZEL_COMPENSATION
typedef struct lb_goertzel_coefs {
    int sine_1;
    int cos_1;
    int gor_len_1;
    int sine_2;
    int cos_2;
    int gor_len_2;
    int sine_3;
    int cos_3;
    int gor_len_3;
    int sine_4;
    int cos_4;
    int gor_len_4;
    int sine_h;
    int cos_h;
    int gor_len_h;
} lb_goertzel_coefs;

typedef struct lb_algo_gor_info
{
    uint16_t    			gor_samples;
    float    				magnitude_100hz;
    float    				magnitude_150hz;
    float    				magnitude_200hz;
    uint32_t				lb_ch_avg[CHANNEL_COUNT];
    uint32_t    			ch_sum[CHANNEL_COUNT];
    bool    				use_average;
} lb_algo_gor_info;

struct lb_sliding_goertzel {
    int length;
    int *comb_history;
    int sine;
    int cosine;
    int q1;    		// Intermediate value 1
    int q2;    		// Intermediate value 2
    int frac_bits;    	// Fractional bits
};

#endif

typedef struct lb_display_coefs {
    int8_t lb_channels;
    float coef_a;
    float coef_b;
    float coef_c;
    float coef_d;
    float coef_50ms;
    float coef_k;
    float coef_k1;
    float dgf;
    float dgf_50ms;
    lb_dbv_atime_coefs *dbv_coefs;
#ifdef GOERTZEL_COMPENSATION
    float mag_lb_k3;
    float mag_lb_k4;
    float mag_hb_k3;
    float mag_hb_k4;
    float mag_lb_k1_thres;
    float mag_lb_k2_thres;
    float mag_hb_k1_thres;
    float mag_hb_k2_thres;
    lb_goertzel_coefs *gor_coefs[2];
#endif
} lb_display_coefs;

typedef struct lb_algo_rgb_raw_info
{
    float                   ch_min[CHANNEL_COUNT];
    float                   ch_max[CHANNEL_COUNT];
} lb_algo_rgb_raw_info;


typedef struct
{
    algo_status algo_status;
#ifdef GOERTZEL_COMPENSATION
    lb_algo_gor_info gor_info;
#endif
    lb_algo_rgb_raw_info rgbc_raw;
    uint8_t samples;
    uint8_t group_buf_cnt;
    uint16_t atime_us;
    uint16_t again;

} lb_algo_state;

void lb_algo_register(lb_algo_state **state);
void lb_algo_close(void);
void lb_algo_reset(void);
bool als_lb_algo(uint8_t* data_buf, uint16_t length, uint8_t* algo_adjust,
	uint16_t atime_us, uint16_t again, uint16_t ch_atg[], float ch_raw_min[], uint8_t* get_data);
void lb_algo_clear_fifo();
#endif//SUPPORT_LOW_BRIGHTNESS_ALGO
struct alsps_data *get_proper_als_data(struct alsps_data *rgbw, uint8_t len);

#endif
