/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - Psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>	<date>		<author>              		<desc>
*******************************************************************/
#ifndef _PSENSOR_ALGO_H_
#define _PSENSOR_ALGO_H_
#include "alsps_def.h"

struct psensor_data {
    int data;
    int ir_data;
    int original_data;
    int state;
    int offset;
};

struct gsensor_data {
    float acc_x;
    float acc_y;
    float acc_z;
};

struct ps_algo_operation {
    void (*set_offset)(int offset);
    void (*set_threshold)(int low, int high);
    void (*get_ps_data)(struct psensor_data* ps_data);
    void (*get_gsensor_data)(struct gsensor_data* acc_data);
    bool (*is_check_ir)(void);
    bool (*is_support_hw_cali)(void);
    void (*do_hw_cali)(void);
};

struct dynamic_thrd_parameter {
    int low_step;
    int high_step;
    int low_limit;
    int high_limit;
    int dirty_low_step;
    int dirty_high_step;
    int ps_dirty_limit;
    int ps_ir_limit;
    int ps_adjust_min;
    int ps_adjust_max;
    int sampling_count;
    int step_max;
    int step_min;
    int step_div;
    int anti_shake_delta;
};

struct factory_cali_parameter {
    int delta;
    int offset;
};

struct dynamic_cali_parameter {
    int dynamic_cali_max;
    int raw2offset_radio;
    int offset_max;
    int offset_range_min;
    int offset_range_max;
    int force_cali_limit;
    int cali_jitter_limit;
    int cal_offset_margin;//use to limit offset max
};

struct ps_algo {
    struct ps_algo_operation *ops;
    struct dynamic_thrd_parameter *dt_parameter;
    struct dynamic_cali_parameter *dc_parameter;
    struct factory_cali_parameter *fc_parameter;
};

void alsps_ps_algo_init(struct ps_algo *algo);
void alsps_ps_dynamic_force_cali(int last_ps_value);
int alsps_ps_dynamic_algo_run(bool enable);
#endif //_PSENSOR_ALGO_H_
