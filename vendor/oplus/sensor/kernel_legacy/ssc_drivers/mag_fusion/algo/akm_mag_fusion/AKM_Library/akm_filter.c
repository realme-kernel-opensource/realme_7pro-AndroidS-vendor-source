#include "akm_filter.h"

/***********************************************************
 *			Circle buffer defination
 *     And its operation: Init/Update/Average/Variance
 ***********************************************************/
struct AKM_CIRCLE_BUF {
    float32_t  *m_buffer;
    int16_t    m_size;
    float32_t  m_max;
    float32_t  m_min;
    float32_t  m_sum;
    float32_t  m_last_data;
    float32_t  m_new_data;
    float32_t  m_ave;
    int16_t    m_pos;//point to next data
    int16_t    m_full_flag;
    int16_t    m_init_flag;
};

int16_t akm_cbuf_init(struct AKM_CIRCLE_BUF *cbuf, float32_t init_data)
{
    int i;

    if(cbuf->m_size <= 0)
        return AKM_ERROR;

    for (i = 0; i < cbuf->m_size; i++) {
        cbuf->m_buffer[i] = init_data;
    }

    cbuf->m_max = init_data;
    cbuf->m_min = init_data;
    cbuf->m_new_data = init_data;
    cbuf->m_last_data = 0;
    cbuf->m_sum = 0;
    cbuf->m_ave = 0;
    cbuf->m_pos = 0;
    cbuf->m_full_flag = 0;
    cbuf->m_init_flag = 1;
    return AKM_SUCCESS;
}

void akm_cbuf_update(struct AKM_CIRCLE_BUF *filter, float32_t data)
{
    int16_t ret = AKM_SUCCESS;
    int16_t i = 0, t = 0;

    if(filter->m_init_flag == 0) {
        ret = akm_cbuf_init(filter, data);

        if(ret != AKM_SUCCESS)
            return;
    }

    if(filter->m_full_flag) {
        filter->m_last_data = filter->m_buffer[filter->m_pos];
    } else {
        filter->m_last_data = 0;
    }

    filter->m_new_data = data;
    filter->m_buffer[filter->m_pos] = data;
    filter->m_pos++;

    if(filter->m_pos >= filter->m_size) {
        filter->m_pos = 0;
        filter->m_full_flag = 1;
    }

    if(filter->m_full_flag) {
        t = filter->m_size;
    } else {
        t = filter->m_pos;
    }

    filter->m_max = filter->m_min = filter->m_buffer[0];

    for(i = 1; i < t; i++) {
        if(filter->m_max < filter->m_buffer[i]) {
            filter->m_max = filter->m_buffer[i];
        }

        if(filter->m_min > filter->m_buffer[i]) {
            filter->m_min = filter->m_buffer[i];
        }
    }

}
/*                   WARNING
 *   akm_cbuf_calc_ave|akm_cbuf_calc_mid_ave|akm_cbuf_calc_var
 *   must not be called at the same time
 */
float32_t akm_cbuf_calc_ave(struct AKM_CIRCLE_BUF *filter)
{
    float32_t data_avg = 0;
    int16_t   ave_num = 0;

    if(filter->m_full_flag) {
        ave_num = filter->m_size;
    } else {
        ave_num = filter->m_pos;
    }

    filter->m_sum = filter->m_sum - filter->m_last_data + filter->m_new_data;

    if(ave_num > 0) {
        data_avg = filter->m_sum / ave_num;
    }

    filter->m_ave = data_avg;
    return data_avg;
}
float32_t akm_cbuf_calc_mid_ave(struct AKM_CIRCLE_BUF *filter)
{
    float32_t data_avg = 0;
    float32_t tmp_sum = 0;
    int16_t   ave_num = 0;

    if(filter->m_full_flag) {
        ave_num = filter->m_size;
    } else {
        ave_num = filter->m_pos;
    }

    filter->m_sum = filter->m_sum - filter->m_last_data + filter->m_new_data;

    //add mid-value filter only when we have more than 2 data
    if(ave_num > 2) {
        tmp_sum = filter->m_sum - filter->m_max - filter->m_min;
        data_avg = tmp_sum / (ave_num - 2);
    } else if(ave_num > 0) {
        data_avg = filter->m_sum / ave_num;
    }

    filter->m_ave = data_avg;
    return data_avg;

}


float32_t akm_cbuf_calc_var(struct AKM_CIRCLE_BUF *filter)
{
    int16_t   var_num, i = 0;
    float32_t sum = 0.0f;
    float32_t ave = 0.0f;
    float32_t result = 0.0f;

    ave = akm_cbuf_calc_ave(filter);

    if(filter->m_full_flag)
        var_num = filter->m_size;
    else
        var_num = filter->m_pos + 1;


    for (i = 0; i < var_num; i++) {
        sum = sum + (filter->m_buffer[i] - ave) * (filter->m_buffer[i] - ave);
    }

    result = sum / var_num;

    return result;
}

/**********************Circle buffer defination End*************************/

static int16_t g_akm_phone_state = 1;

void akm_func_for_avoid_compile_err(void)
{
    int16_t tmp_int;

    tmp_int = (int16_t)g_akm_phone_state;

    if(tmp_int)
        tmp_int = 0;
}

#ifdef AKM_ENABLE_PG_MAG_FILTER

static float32_t _mag_ave_bufx[AKM_MAG_AVE_BUF_SIZE];
static float32_t _mag_ave_bufy[AKM_MAG_AVE_BUF_SIZE];
static float32_t _mag_ave_bufz[AKM_MAG_AVE_BUF_SIZE];

static struct AKM_CIRCLE_BUF mag_ave_bufx = {_mag_ave_bufx, AKM_MAG_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF mag_ave_bufy = {_mag_ave_bufy, AKM_MAG_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF mag_ave_bufz = {_mag_ave_bufz, AKM_MAG_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void akm_pg_mag_filter_init(void)
{
    mag_ave_bufx.m_init_flag = 0;
    mag_ave_bufy.m_init_flag = 0;
    mag_ave_bufz.m_init_flag = 0;
}
int16_t akm_pg_mag_filter(float32_t *mx, float32_t *my, float32_t *mz)
{
    if(g_akm_phone_state) {
        akm_cbuf_init(&mag_ave_bufx, *mx);
        akm_cbuf_init(&mag_ave_bufy, *my);
        akm_cbuf_init(&mag_ave_bufz, *mz);
        return AKM_SUCCESS;
    }

    akm_cbuf_update(&mag_ave_bufx, *mx);
    *mx = akm_cbuf_calc_mid_ave(&mag_ave_bufx);

    akm_cbuf_update(&mag_ave_bufy, *my);
    *my = akm_cbuf_calc_mid_ave(&mag_ave_bufy);

    akm_cbuf_update(&mag_ave_bufz, *mz);
    *mz = akm_cbuf_calc_mid_ave(&mag_ave_bufz);


    return AKM_SUCCESS;
}

#endif


#ifdef AKM_ENABLE_PG_ACC_FILTER

static float32_t _acc_bufx[AKM_ACC_AVE_BUF_SIZE];
static float32_t _acc_bufy[AKM_ACC_AVE_BUF_SIZE];
static float32_t _acc_bufz[AKM_ACC_AVE_BUF_SIZE];

static struct AKM_CIRCLE_BUF acc_ave_bufx = {_acc_bufx, AKM_ACC_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF acc_ave_bufy = {_acc_bufy, AKM_ACC_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF acc_ave_bufz = {_acc_bufz, AKM_ACC_AVE_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};


void akm_pg_acc_filter_init(void)
{
    acc_ave_bufx.m_init_flag = 0;
    acc_ave_bufy.m_init_flag = 0;
    acc_ave_bufz.m_init_flag = 0;
}
int16_t akm_pg_acc_filter(float32_t *ax, float32_t *ay, float32_t *az)
{
    akm_cbuf_update(&acc_ave_bufx, *ax);
    *ax = akm_cbuf_calc_mid_ave(&acc_ave_bufx);

    akm_cbuf_update(&acc_ave_bufy, *ay);
    *ay = akm_cbuf_calc_mid_ave(&acc_ave_bufy);

    akm_cbuf_update(&acc_ave_bufz, *az);
    *az = akm_cbuf_calc_mid_ave(&acc_ave_bufz);

    return AKM_SUCCESS;
}

#endif


#ifdef AKM_ENABLE_D6D_YAW_FILTER
#define AKM_AKSC_180DEG 11520.0f//64*180.0f
#define AKM_AKSC_360DEG 23040.0f//64*360.0f

float32_t pre_data = 0.0f;

void akm_d6d_yaw_filter_init(void)
{
    pre_data = 0.0f;
}

float32_t akm_d6d_yaw_filter(float32_t data)
{
    float32_t diff = 0.0f;

    if(g_akm_phone_state) {
        pre_data = data;
        return data;
    }

    diff = data - pre_data;

    if(diff < 0)
        diff = -diff;

    if (diff > AKM_AKSC_180DEG) {
        if(pre_data < AKM_AKSC_180DEG) {
            data = data - AKM_AKSC_360DEG;
        } else {
            data = data + AKM_AKSC_360DEG;
        }
    }

    pre_data = (1 - AKM_YAW_RC_GAIN) * pre_data + AKM_YAW_RC_GAIN * data;

    if (diff > AKM_AKSC_180DEG) {
        if(pre_data < 0) {
            pre_data = pre_data + AKM_AKSC_360DEG;
        } else if(pre_data >= AKM_AKSC_360DEG) {
            pre_data = pre_data - AKM_AKSC_360DEG;
        }
    }

    return pre_data;
}


#endif


#ifdef AKM_ENABLE_DETECT_PHONE_STATE

static int16_t stable_count = 0;
static int16_t moving_count = 0;

static float32_t _acc_state_bufx[AKM_MOVING_BUF_SIZE];
static float32_t _acc_state_bufy[AKM_MOVING_BUF_SIZE];
static float32_t _acc_state_bufz[AKM_MOVING_BUF_SIZE];

static struct AKM_CIRCLE_BUF acc_state_bufx = {_acc_state_bufx, AKM_MOVING_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF acc_state_bufy = {_acc_state_bufy, AKM_MOVING_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static struct AKM_CIRCLE_BUF acc_state_bufz = {_acc_state_bufz, AKM_MOVING_BUF_SIZE, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void akm_detect_phone_state_init(void)
{

    stable_count = 0;
    moving_count = 0;
    g_akm_phone_state = 1;
    acc_state_bufx.m_init_flag = 0;
    acc_state_bufy.m_init_flag = 0;
    acc_state_bufz.m_init_flag = 0;
}

int16_t akm_detect_moving(float32_t ax, float32_t ay, float32_t az)
{
    float32_t var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;

    akm_cbuf_update(&acc_state_bufx, ax);
    var_x = akm_cbuf_calc_var(&acc_state_bufx);

    if(var_x > AKM_MOVING_VAR_THRESHOLD_X)
        return 1;

    akm_cbuf_update(&acc_state_bufy, ay);
    var_y = akm_cbuf_calc_var(&acc_state_bufy);

    if(var_y > AKM_MOVING_VAR_THRESHOLD_Y)
        return 1;

    akm_cbuf_update(&acc_state_bufz, az);
    var_z = akm_cbuf_calc_var(&acc_state_bufz);

    if(var_z > AKM_MOVING_VAR_THRESHOLD_Z)
        return 1;

    return 0;
}

int16_t akm_detect_horizontal(float32_t ax, float32_t ay, float32_t az)
{
    float32_t angle_cos_square = 1.0f;

    angle_cos_square = az * az / (ax * ax + ay * ay + az * az);

    if(angle_cos_square > AKM_HORIZONTAL_THRESHOLD) {

        return 1;
    } else {

        return 0;
    }

    return 0;
}
int16_t akm_detect_phone_state(float32_t ax, float32_t ay, float32_t az)
{
    int16_t moving_state = 0, horiz_state = 0;
    int16_t state;
    moving_state = akm_detect_moving(ax, ay, az);

    if(moving_state == 0)
        horiz_state = akm_detect_horizontal(ax, ay, az);

    AKM_MSG_INFO_2("akm_log detect_phone_state m:%d, h:%d", moving_state, horiz_state);

    if (moving_state || !horiz_state) {
        if(stable_count >= AKM_STABLE_COUNT) {
            moving_count++;

            if(moving_count <= AKM_STABLE_FLUC_TIMES) {
                stable_count++;
                state = 0;
                goto DETECT_EXIT;
            } else {
                moving_count = 0;
                stable_count = 0;
                state = 1;
                goto DETECT_EXIT;
            }
        } else {
            moving_count = 0;
            stable_count = 0;
            state = 1;
            goto DETECT_EXIT;
        }
    } else {
        stable_count++;

        if(stable_count >= AKM_STABLE_COUNT) {
            if(stable_count >= (AKM_STABLE_COUNT + AKM_STEBLE_COUNT_EXTRA)) {
                stable_count = AKM_STABLE_COUNT;
                moving_count = 0;
            }

            state = 0;
            goto DETECT_EXIT;
        }

        state = 1;
        goto DETECT_EXIT;
    }

DETECT_EXIT:
    g_akm_phone_state = state;
    return state;
}

#endif



