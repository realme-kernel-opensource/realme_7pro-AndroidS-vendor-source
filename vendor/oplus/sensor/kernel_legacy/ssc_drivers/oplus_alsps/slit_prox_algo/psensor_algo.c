/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>       <date>    	 <author>			   <desc>
*******************************************************************/
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "psensor_algo.h"

#define RADIANS_TO_DEGREES        (180.0f / M_PI)
#define M_PI    3.1415926535897932384
#define CALIBRATION_SIZE        10
#define DYNAMIC_CALI_GOAL        50
#define DYNAMIC_CALI_FLAT_MIN_PS    100
#define HIGHT_LIHT_DEALA        200
#define CMP_ZERO 0.0001

static struct ps_algo *g_chip = NULL;

enum prox_cali_state {
    CALI_NONE = 0,
    CALI_BOOTUP,
    CALI_FORCE,
    CALI_FLAT,
    CALI_OVER_ZERO,
    CALI_TYPE_NUM,
};

enum {
    INIT_STATE,
    COMMON_STATE,
    HIGH_IR_STATE,
    DIRTY_STATE,
};

struct device_info {
    bool is_flat;
    bool is_vertical;
    uint8_t device_change_count;
};

struct ps_info {
    int ps_value;
    int ps_avg;
    int ps_min;
    int ps_max;
    int ps_sum;
    int ps_last;
    int ps_count;
    int ir_data;
    int original_data;
    bool is_far;
    bool is_ir_high;
};

struct dynamic_thrd_info {
    struct device_info d_info;
    struct ps_info p_info;
    int low_threshold;
    int high_threshold;
    int algo_state;
};

struct dynamic_cali_info {
    struct device_info d_info;
    struct ps_info p_info;
    uint8_t action;
    uint8_t action_count;
    uint8_t buff_index;
    uint8_t buff_len;
    bool collect_finished;
    int buff[CALIBRATION_SIZE];
};

static struct dynamic_thrd_info dt_info;
static struct dynamic_cali_info dc_info;

static bool ps_check_ir(void)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->is_check_ir) {
        return false;
    }

    return g_chip->ops->is_check_ir();
}

static bool ps_support_hw_cali(void)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->is_support_hw_cali) {
        return false;
    }

    return g_chip->ops->is_support_hw_cali();
}

static void get_ps_data(struct psensor_data * data)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->get_ps_data) {
        return;
    }

    g_chip->ops->get_ps_data(data);
}

static void ps_get_gsensor_data(struct gsensor_data *data)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->get_gsensor_data) {
        return;
    }

    g_chip->ops->get_gsensor_data(data);
}

static void ps_set_threshold(int low, int high)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->set_threshold) {
        return;
    }

    g_chip->ops->set_threshold(low, high);
}

static void ps_set_offset(int offset)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->set_offset) {
        return;
    }

    g_chip->ops->set_offset(offset);
}

static void ps_do_hw_cali(void)
{
    if (!g_chip || !g_chip->ops ||
        !g_chip->ops->do_hw_cali) {
        return;
    }

    g_chip->ops->do_hw_cali();
}


static bool device_is_vertical(struct gsensor_data *data)
{
    bool ret = false;
    int angle = 0;
    float norm = 0;

    norm = sqrtf((data->acc_x / 1000) * (data->acc_x / 1000) +
            (data->acc_y / 1000) * (data->acc_y / 1000) +
            (data->acc_z / 1000) * (data->acc_z / 1000));

    if ((data->acc_y / 1000 >= 8.5f) || (data->acc_y / 1000 <= -8.5f)) {
        angle = (int)(asinf(fabs(data->acc_y / 1000) / (double)norm) * RADIANS_TO_DEGREES);
    } else if ((data->acc_x / 1000 >= 8.5f) || (data->acc_x / 1000 <= -8.5f)) {
        angle = (int)(asinf(fabs(data->acc_x / 1000) / (double)norm) * RADIANS_TO_DEGREES);
    }

    ret = angle >= 70 ? true : false;

    ALSPS_LOG("[x y z]:[%f %f %f],angle=%d\n",
        (double)data->acc_x,
        (double)data->acc_y,
        (double)data->acc_z, angle);

    return ret;
}

static bool device_is_flat(struct gsensor_data *data)
{
    bool is_flat = false;
    float norm_acc = 0;
    int tilt_angle = 0;

    if ((data->acc_z > 6500) && (data->acc_z <= 10500)) {
        norm_acc = sqrtf((data->acc_x / 1000) * (data->acc_x / 1000) +
                (data->acc_y / 1000) * (data->acc_y / 1000) +
                (data->acc_z / 1000) * (data->acc_z / 1000));

        tilt_angle = (int)(asinf(data->acc_z / 1000 / norm_acc) * RADIANS_TO_DEGREES);

        if (tilt_angle >= 40) {
            is_flat = true;
        }
    }

    if (is_flat != dc_info.d_info.is_flat) {
        dc_info.d_info.device_change_count++;
    } else {
        dc_info.d_info.device_change_count = 0;
    }

    ALSPS_LOG("[%d,%d],angle = %d count =%d", is_flat,
        dc_info.d_info.is_flat,
        tilt_angle,
        dc_info.d_info.device_change_count);

    if (dc_info.d_info.device_change_count >= 5) {
        return !dc_info.d_info.is_flat;
    } else {
        return dc_info.d_info.is_flat;
    }
}

static int update_ps_threshold (struct dynamic_thrd_parameter *para, int state)
{
    int low_threshold = 0;
    int high_threshold = 0;
    bool is_vertical = false;
    struct gsensor_data acc_data;

    ALSPS_LOG("ps_value = %d, ps_avg = %d, ps_min = %d, state = %d \n",
        dt_info.p_info.ps_value,
        dt_info.p_info.ps_avg,
        dt_info.p_info.ps_min,
        state);

    switch (state) {
    case INIT_STATE:
        ps_get_gsensor_data(&acc_data);

        is_vertical = device_is_vertical(&acc_data);
        dt_info.d_info.is_vertical = is_vertical;

        if ((is_vertical) && ((dt_info.low_threshold != 0) &&
                (dt_info.high_threshold != 0))) {
            low_threshold = dt_info.low_threshold;
            high_threshold = dt_info.high_threshold;
        } else {
            low_threshold = para->low_limit;
            high_threshold = para->high_limit;
        }

        break;

    case COMMON_STATE:
        low_threshold = dt_info.p_info.ps_avg + para->low_step;
        high_threshold = dt_info.p_info.ps_avg + para->high_step;

        if (low_threshold > (dt_info.p_info.ps_min + para->dirty_low_step)) {
            low_threshold = dt_info.p_info.ps_min + para->dirty_high_step;
        }

        break;

    case HIGH_IR_STATE:
        if (dt_info.algo_state == HIGH_IR_STATE) {
            /*already in HIGHLIGHT_STATE*/
            goto not_need_adjust;
        }

        if ((dt_info.low_threshold != 0) &&
            (dt_info.high_threshold != 0)) {
            low_threshold = dt_info.low_threshold + HIGHT_LIHT_DEALA;
            high_threshold = dt_info.high_threshold + HIGHT_LIHT_DEALA;
        } else {
            low_threshold = para->low_limit;
            high_threshold = para->high_limit;
        }

        break;

    case DIRTY_STATE:
        if (dt_info.low_threshold >= dt_info.p_info.ps_min + para->dirty_low_step) {
            /*threshold is large, no need to avoid Dirty Problem*/
            goto not_need_adjust;
        }

        low_threshold = dt_info.p_info.ps_min + para->dirty_low_step;
        high_threshold = dt_info.p_info.ps_min + para->dirty_high_step;
        break;

    default:
        ALSPS_LOG("Not support this algo state\n");
        break;
    }

    /*skip some condition*/
    if (high_threshold > para->ps_adjust_max ||
        low_threshold < para->ps_adjust_min) {
        /*no need to adjust*/
        goto not_need_adjust;
    } else if (((dt_info.p_info.ps_value <= low_threshold) && !dt_info.p_info.is_far)
        || ((dt_info.p_info.ps_value >= high_threshold) && dt_info.p_info.is_far)) {
        if (dt_info.algo_state != HIGH_IR_STATE) {
            /*no need to adjust*/
            goto not_need_adjust;
        }
    }

    if (low_threshold > para->low_limit) {
        low_threshold = para->low_limit;
    }

    if (high_threshold > para->high_limit) {
        high_threshold = para->high_limit;
    }

    dt_info.low_threshold = low_threshold;
    dt_info.high_threshold = high_threshold;
    dt_info.algo_state = state;

    ALSPS_LOG("ps_value %d,low_threshold %d, high_threshold %d\n",
        dt_info.p_info.ps_value,
        low_threshold,
        high_threshold);

    ps_set_threshold(low_threshold, high_threshold);
    return 0;

not_need_adjust:
    return -1;
}

static void do_dynamic_threshold(struct psensor_data *ps_data)
{
    if (!g_chip) {
        ALSPS_ERR("NULL \n");
        return;
    }

    if (ps_data->data < 0) {
        ALSPS_ERR("invalid ps_data \n");
        return;
    }

    dt_info.p_info.ps_value = ps_data->data;
    dt_info.p_info.ir_data = ps_data->ir_data;
    dt_info.p_info.is_far = !(!!ps_data->state);

    /*check ir condition*/
    if (ps_check_ir()) {
        dt_info.p_info.is_ir_high = false;
    } else {
        if (ps_data->ir_data > g_chip->dt_parameter->ps_ir_limit) {
            dt_info.p_info.is_ir_high = true;
        } else {
            dt_info.p_info.is_ir_high = false;
        }
    }

    ALSPS_LOG("ps_value %d, ir_data %d, is_ir_high %d, ps_min %d\n",
        dt_info.p_info.ps_value,
        dt_info.p_info.ir_data,
        dt_info.p_info.is_ir_high,
        dt_info.p_info.ps_min);

    /*if ps_min = ps_adjust_max means need to do initialization*/
    if (dt_info.p_info.ps_min >= g_chip->dt_parameter->ps_adjust_max) {
        update_ps_threshold(g_chip->dt_parameter, INIT_STATE);
    }

    if (dt_info.p_info.is_far) {
        if (dt_info.p_info.is_ir_high) {
            update_ps_threshold(g_chip->dt_parameter, HIGH_IR_STATE);
            dt_info.p_info.ps_last = g_chip->dt_parameter->ps_adjust_max;
        } else {
            if (dt_info.p_info.ps_count < g_chip->dt_parameter->sampling_count) {
                dt_info.p_info.ps_sum = dt_info.p_info.ps_sum +
                    dt_info.p_info.ps_value;
                dt_info.p_info.ps_count++;
                /* we need to collect sampling_count data*/
                return;
            } else {
                /*  get ps average when sampling_count*/
                dt_info.p_info.ps_avg = dt_info.p_info.ps_sum /
                    g_chip->dt_parameter->sampling_count;

                /*  only adjust from adjust_min to adjust_max */
                dt_info.p_info.ps_avg = (dt_info.p_info.ps_avg > g_chip->dt_parameter->ps_adjust_max) ?
                    g_chip->dt_parameter->ps_adjust_max : dt_info.p_info.ps_avg;
                dt_info.p_info.ps_avg = (dt_info.p_info.ps_avg < g_chip->dt_parameter->ps_adjust_min) ?
                    g_chip->dt_parameter->ps_adjust_min : dt_info.p_info.ps_avg;

                /*  guess min crosstalk when lowlight */
                dt_info.p_info.ps_min = (dt_info.p_info.ps_min > dt_info.p_info.ps_avg) ?
                    dt_info.p_info.ps_avg : dt_info.p_info.ps_min;

                /* adjust ps threshold */
                update_ps_threshold(g_chip->dt_parameter, COMMON_STATE);

                dt_info.p_info.ps_last = dt_info.p_info.ps_avg;
            }
        }
    } else {
        //To avoid a dirty problem when near with a large ps
        if (dt_info.p_info.ps_value > g_chip->dt_parameter->ps_dirty_limit) {
            update_ps_threshold(g_chip->dt_parameter, DIRTY_STATE);
        }

        dt_info.p_info.ps_last = g_chip->dt_parameter->ps_adjust_max;
    }

    dt_info.p_info.ps_sum = 0;
    dt_info.p_info.ps_count = 0;
}

static uint8_t ps_get_cali_action(void)
{
    uint8_t action = CALI_NONE;

    if (dc_info.action == CALI_FORCE) {
        action = CALI_FORCE;
    } else if (((dc_info.p_info.ps_value == 0) && (dc_info.p_info.original_data > DYNAMIC_CALI_GOAL))
        && (dc_info.p_info.ir_data < g_chip->dt_parameter->ps_ir_limit)
        && (dc_info.p_info.is_far)) {
        action = CALI_OVER_ZERO;
    } else if ((dc_info.p_info.ps_value >= DYNAMIC_CALI_FLAT_MIN_PS)
        && (dc_info.p_info.ps_value <= g_chip->dc_parameter->dynamic_cali_max)
        && (dc_info.d_info.is_flat)) {
        action = CALI_FLAT;
    }

    ALSPS_LOG(":action = %d is_far = %d is_flat= %d ps_data = %d\n",
        action,
        dc_info.p_info.is_far,
        dc_info.d_info.is_flat,
        dc_info.p_info.ps_value);

    return action;
}

static void clear_ps_dc_info(void)
{
    ALSPS_LOG("call \n");
    memset(&dc_info, 0, sizeof(struct dynamic_cali_info));
}

static void clear_ps_dc_buff(void)
{
    ALSPS_LOG("call \n");
    uint32_t offset = (uint32_t)&dc_info.p_info - (uint32_t)&dc_info.d_info;
    memset(&dc_info.p_info, 0,
        sizeof(struct dynamic_cali_info) - offset);
}

static void collect_ps_data(int ps_value)
{
    dc_info.buff[dc_info.buff_index] = ps_value;

    if ((dc_info.p_info.ps_max == 0) && (dc_info.p_info.ps_min == 0)) {
        dc_info.p_info.ps_max = dc_info.buff[dc_info.buff_index];
        dc_info.p_info.ps_min = dc_info.buff[dc_info.buff_index];
    }

    dc_info.p_info.ps_max = (dc_info.p_info.ps_max >= dc_info.buff[dc_info.buff_index] ?
            dc_info.p_info.ps_max : dc_info.buff[dc_info.buff_index]);
    dc_info.p_info.ps_min = (dc_info.p_info.ps_min <= dc_info.buff[dc_info.buff_index] ?
            dc_info.p_info.ps_min : dc_info.buff[dc_info.buff_index]);

    ALSPS_LOG("index %d ,buff[%d] = %d\n", dc_info.buff_index, dc_info.buff_index,
        dc_info.buff[dc_info.buff_index]);

    dc_info.p_info.ps_sum += dc_info.buff[dc_info.buff_index];
    dc_info.buff_index++;

    if (dc_info.buff_index >= dc_info.buff_len) {
        dc_info.collect_finished = true;
    } else {
        dc_info.collect_finished = false;
    }
}

static int calculate_ps_offset(void)
{
    int temp_cali = -1;
    int ps_average = 0;

    if (!dc_info.collect_finished) {
        return temp_cali;
    } else {
        ALSPS_LOG("max = %d min = %d cali_jitter_limit %d\n",
            dc_info.p_info.ps_max,
            dc_info.p_info.ps_min,
            g_chip->dc_parameter->cali_jitter_limit);

        if ((dc_info.p_info.ps_max - dc_info.p_info.ps_min) <=
            g_chip->dc_parameter->cali_jitter_limit) {
            ps_average = dc_info.p_info.ps_sum / dc_info.buff_index;
            temp_cali = ps_average - DYNAMIC_CALI_GOAL;

            if (temp_cali < 0) {
                temp_cali = 0;
            }

            ALSPS_LOG("ps_average = %d, temp_cali = %d, sum = %d, index = %d\n",
                ps_average,
                temp_cali,
                dc_info.p_info.ps_sum,
                dc_info.buff_index);
        } else {
            clear_ps_dc_buff();
        }

        return temp_cali;
    }
}

static void do_dynamic_calibration(int ps_value, int ps_offset)
{
    bool is_hw_cali = ps_support_hw_cali();
    int cali_offset = 0;

    if (!g_chip) {
        ALSPS_ERR("g_chip null \n");
        return;
    }

    ALSPS_LOG("ps_value = %d ps_offset= %d is_hw_cali %d\n",
        ps_value, ps_offset, is_hw_cali);


    if (!is_hw_cali && ps_value != 0) {
        collect_ps_data(ps_value);
        cali_offset = calculate_ps_offset();

        if (cali_offset >= 0) {
            if ((cali_offset < ps_offset) && (dc_info.action == CALI_FLAT)) {
                //invalid result
            } else {
                ps_set_offset(cali_offset);
            }

            clear_ps_dc_buff();
        }

    } else if (is_hw_cali && (ps_offset != g_chip->dc_parameter->offset_range_min)) {
        collect_ps_data(ps_value);
        cali_offset = calculate_ps_offset();

        if (cali_offset >= 0) {
            if (dc_info.action != CALI_OVER_ZERO) {
                cali_offset = (int)(cali_offset *
                        1000 / g_chip->dc_parameter->raw2offset_radio +
                        ps_offset);

                if ((cali_offset < g_chip->dc_parameter->offset_max) &&
                    (ps_offset > g_chip->dc_parameter->offset_range_min)) {
                    ps_set_offset(cali_offset);
                } else {
                    ALSPS_LOG("offset %d is too large", cali_offset);
                }
            } else {
                ALSPS_LOG(" start hard cali\n");
                ps_do_hw_cali();
            }

            clear_ps_dc_buff();
        }
    }
}

static void start_dynamic_calibration(struct psensor_data *ps_data,
    struct gsensor_data *acc_data)
{
    uint8_t action = CALI_NONE;
    bool is_ir_check = false;
    bool need_original_data = false;

    dc_info.d_info.is_flat = device_is_flat(acc_data);
    is_ir_check = ps_check_ir();

    if (ps_support_hw_cali()) {
        need_original_data = false;
    } else {
        need_original_data = true;
    }

    dc_info.p_info.ps_value = ps_data->data;
    dc_info.p_info.original_data = ps_data->original_data;
    dc_info.p_info.ir_data = ps_data->ir_data;
    dc_info.p_info.is_far = !(!!ps_data->state);

    if ((is_ir_check) && (need_original_data)) {
        action = ps_get_cali_action();
    } else if ((!is_ir_check) && (!need_original_data)) {
        dc_info.p_info.ir_data = 0;
        dc_info.p_info.original_data = DYNAMIC_CALI_GOAL + 1;
        action = ps_get_cali_action();
    } else if ((!is_ir_check) && (need_original_data)) {
        dc_info.p_info.ir_data = 0;
        action = ps_get_cali_action();
    } else if ((is_ir_check) && (!need_original_data)) {
        dc_info.p_info.original_data = DYNAMIC_CALI_GOAL + 1;
        action = ps_get_cali_action();
    }

    ALSPS_LOG("action %d,dc_info.action %d, is_ir_check %d, need_original_data %d\n",
        action, dc_info.action,
        is_ir_check, need_original_data);

    if (dc_info.action == CALI_NONE) {
        uint8_t count = dc_info.action_count;

        clear_ps_dc_buff();

        switch (action) {
        case CALI_FLAT:
            count++;
            ALSPS_LOG("action = %d count =%d\n", dc_info.action, count);

            if (count >= 5) {
                dc_info.action = CALI_FLAT;
                dc_info.buff_len = 10;
            }

            dc_info.action_count = count;
            break;

        case CALI_OVER_ZERO:
            dc_info.action = CALI_OVER_ZERO;
            dc_info.buff_len = 2;
            ALSPS_LOG("cross cali original_data =%d\n", dc_info.p_info.original_data);
            break;

        default:
            break;
        }
    } else {
        if (action == dc_info.action) {
            if (need_original_data) {
                do_dynamic_calibration(ps_data->original_data, ps_data->offset);
            } else {
                do_dynamic_calibration(ps_data->data, ps_data->offset);
            }
        } else {
            clear_ps_dc_buff();
        }
    }
}

void alsps_ps_dynamic_force_cali(int last_ps_value)
{
    ALSPS_LOG("alsps_set_dynamic_force_enable\n");

    clear_ps_dc_buff();

    if (last_ps_value < g_chip->dc_parameter->force_cali_limit) {
        dc_info.action = CALI_FORCE;
        dc_info.buff_len = 5;
    }
}

void alsps_ps_algo_init(struct ps_algo *algo)
{
    if (!algo || !algo->ops || !algo->fc_parameter ||
        !algo->dt_parameter || !algo->dc_parameter)
        return;

    g_chip = algo;
    dt_info.p_info.ps_count = 0;
    dt_info.p_info.ps_value = 0;
    dt_info.p_info.ps_avg = 0;
    dt_info.p_info.ps_sum = 0;
    dt_info.p_info.is_far = true;
    dt_info.p_info.is_ir_high = false;
    dt_info.d_info.is_vertical = false;
    dc_info.d_info.is_flat = false;

    if (g_chip->dt_parameter != NULL) {
        dt_info.p_info.ps_min = g_chip->dt_parameter->ps_adjust_max;
        dt_info.p_info.ps_last = g_chip->dt_parameter->ps_adjust_max;
    } else {
        dt_info.p_info.ps_last = 1023;
        dt_info.p_info.ps_min = 1023;
    }

    /* calculate dynamic low high step*/
    g_chip->dt_parameter->low_step = g_chip->fc_parameter->delta *
        1000 / g_chip->dt_parameter->step_div;

    if (g_chip->dt_parameter->low_step < g_chip->dt_parameter->step_min) {
        g_chip->dt_parameter->low_step = g_chip->dt_parameter->step_min;
    } else if (g_chip->dt_parameter->low_step > g_chip->dt_parameter->step_max) {
        g_chip->dt_parameter->low_step = g_chip->dt_parameter->step_max;
    }

    g_chip->dt_parameter->high_step = g_chip->dt_parameter->low_step +
        g_chip->dt_parameter->anti_shake_delta;


    /*calculate the max_offset*/
    if (g_chip->dc_parameter->cal_offset_margin + g_chip->fc_parameter->offset < g_chip->dc_parameter->offset_max) {
        g_chip->dc_parameter->offset_max = g_chip->dc_parameter->cal_offset_margin + g_chip->fc_parameter->offset;
    }

    ALSPS_LOG("low_step %d, high_step %d max_ofset = %d\n",
        g_chip->dt_parameter->low_step,
        g_chip->dt_parameter->high_step,
        g_chip->dc_parameter->offset_max);
}

int alsps_ps_dynamic_algo_run(bool enable)
{
    struct psensor_data ps_data;
    struct gsensor_data acc_data;

    if ((!g_chip) || (!g_chip->ops) ||
        (!g_chip->dt_parameter) ||
        (!g_chip->dc_parameter) ||
        (!g_chip->fc_parameter)) {
        ALSPS_LOG("g_chip is NULL !\n");
        return -1;
    }

    if (enable) {
        get_ps_data(&ps_data);
        ps_get_gsensor_data(&acc_data);

        do_dynamic_threshold(&ps_data);
        start_dynamic_calibration(&ps_data, &acc_data);
    } else {
        ALSPS_LOG("algo disable\n");

        dt_info.p_info.is_far = true;
        dt_info.p_info.ps_last = g_chip->dt_parameter->ps_adjust_max;
        dt_info.p_info.ps_min = g_chip->dt_parameter->ps_adjust_max;
        dt_info.p_info.ps_avg = 0;
        dt_info.p_info.ps_value = 0;

        update_ps_threshold(g_chip->dt_parameter, HIGH_IR_STATE);

        dt_info.p_info.ps_sum = 0;
        dt_info.p_info.ps_count = 0;

        clear_ps_dc_info();
    }

    return 0;
}
