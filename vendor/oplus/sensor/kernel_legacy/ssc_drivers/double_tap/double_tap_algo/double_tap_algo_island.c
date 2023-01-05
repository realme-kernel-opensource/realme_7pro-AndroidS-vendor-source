#include "double_tap_algo.h"
static double_tap_state g_double_tap_state;

static void report_event_handle(double_tap_state *state)
{
    static uint16_t report_count = 0;

    report_count ++;
    report_count %= 0xFFFF;   // just double_tap report count to check slpi and sensor hal

    DOUBLE_TAP_LOGI("--------report_count = %d", report_count);
    state->sensor_ops->report(report_count);
}

static bool check_data_valid(double_tap_state *state, uint64_t timestamp, SENSOR_TYPE type)
{
/*
    if (timestamp < state->last_data_ts[type]) {
        DOUBLE_TAP_LOGI("abnormal timestamp, last_ts %lu",    *****);
        return false;
    }
*/
    state->last_data_ts[type] = timestamp;
    return true;
}

static bool judge_acc_history_state(double_tap_state *state)
{
    int i = 0;
    int temp = 0;
    int plat_count = 0;
    double_tap_sensor_data average;
    double_tap_sensor_data variance;
    double_tap_sensor_data acc_max;
    double_tap_sensor_data acc_min;
    state->is_check_history = true;
    memset(&average, 0, sizeof(double_tap_sensor_data));
    memset(&variance, 0, sizeof(double_tap_sensor_data));
    temp = (state->acc_point - 1 + ACC_CHECK_LEN) % ACC_CHECK_LEN;
    memcpy(&acc_max, &state->acc_buf[temp], sizeof(double_tap_sensor_data));
    memcpy(&acc_min, &state->acc_buf[temp], sizeof(double_tap_sensor_data));

    for (i = 0; i < HISTORY_DATA_LEN; i++) {
        temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
        average.x += state->gravity_buf[temp].x;
        average.y += state->gravity_buf[temp].y;
        average.z += state->gravity_buf[temp].z;
        DOUBLE_TAP_LOGI("grav data x:%d, y:%d, z:%d",
                        (int)(state->gravity_buf[temp].x *1000),
                        (int)(state->gravity_buf[temp].y * 1000),
                        (int)(state->gravity_buf[temp].z *1000));
    }
    average.x = average.x / HISTORY_DATA_LEN;
    average.y = average.y / HISTORY_DATA_LEN;
    average.z = average.z / HISTORY_DATA_LEN;

    for (i = 0; i < HISTORY_DATA_LEN; i++) {
        temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
        variance.x += powf(state->gravity_buf[temp].x - average.x, 2);
        variance.y += powf(state->gravity_buf[temp].y - average.y, 2);
        variance.z += powf(state->gravity_buf[temp].z - average.z, 2);
        if (state->gravity_buf[temp].z >= 9.75 && state->gravity_buf[temp].z <= 9.85) {
            plat_count++;
            if (plat_count >= 5) {
                plat_count = 0;
                DOUBLE_TAP_LOGI("trigger gravity plat");
                return false;
            }
        }
    }
    DOUBLE_TAP_LOGI("grav variance x:%d, y:%d, z:%d", (int)(variance.x *1000), (int)(variance.y * 1000), (int)(variance.z *1000));
    if (variance.x > 10 || variance.y > 10 || variance.z > 10) {
        return false;
    }

    for (i = 0; i < HISTORY_DATA_LEN; i++) {
        temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
        if (acc_max.x < state->acc_buf[temp].x)
            acc_max.x = state->acc_buf[temp].x;
        if (acc_max.y < state->acc_buf[temp].y)
            acc_max.y = state->acc_buf[temp].y;
        if (acc_max.z < state->acc_buf[temp].z)
            acc_max.z = state->acc_buf[temp].z;
        if (acc_min.x > state->acc_buf[temp].x)
            acc_min.x = state->acc_buf[temp].x;
        if (acc_min.y > state->acc_buf[temp].y)
            acc_min.y = state->acc_buf[temp].y;
        if (acc_min.z > state->acc_buf[temp].z)
            acc_min.z = state->acc_buf[temp].z;
        if (fabs(state->acc_buf[temp].x) <= 1 && fabs(state->acc_buf[temp].y) <= 1 && fabs(state->acc_buf[temp].z) <= 1) {
           DOUBLE_TAP_LOGI("trigger free fall");
           return false;
        }

        DOUBLE_TAP_LOGI("history acc x:%d, y:%d, z:%d", (int)(state->acc_buf[temp].x *1000), (int)(state->acc_buf[temp].y * 1000), (int)(state->acc_buf[temp].z *1000));
    }
    if (fabs(acc_max.x - acc_min.x) > 0.6 * fabs(acc_max.z - acc_min.z) || fabs(acc_max.y - acc_min.y) > 0.6 *fabs(acc_max.z - acc_min.z)) {
        DOUBLE_TAP_LOGI("trigger XYside tap");
        return false;
    }

    return true;
}

static void handle_dtp_event(double_tap_state *state)
{
    if (2 != state->dbt_state || 1 == state->prx_state) {
        return;
    }
    state->dbt_fired = 1;
}

static bool judge_is_plat_status(double_tap_state *state)
{
    int temp = 0;
    int i = 0;
    double_tap_sensor_data acc_max;
    double_tap_sensor_data acc_min;
    temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
    memcpy(&acc_max, &state->acc_buf[temp], sizeof(double_tap_sensor_data));
    memcpy(&acc_min, &state->acc_buf[temp], sizeof(double_tap_sensor_data));
    for (i = 0; i < 4; i++) {
        temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
        if (acc_max.x < state->acc_buf[temp].x)
            acc_max.x = state->acc_buf[temp].x;
        if (acc_max.y < state->acc_buf[temp].y)
            acc_max.y = state->acc_buf[temp].y;
        if (acc_max.z < state->acc_buf[temp].z)
            acc_max.z = state->acc_buf[temp].z;
        if (acc_min.x > state->acc_buf[temp].x)
            acc_min.x = state->acc_buf[temp].x;
        if (acc_min.y > state->acc_buf[temp].y)
            acc_min.y = state->acc_buf[temp].y;
        if (acc_min.z > state->acc_buf[temp].z)
            acc_min.z = state->acc_buf[temp].z;

    }
    if (fabs(acc_max.x - acc_min.x) < 0.5 && fabs(acc_max.y - acc_min.y) < 0.5 && fabs(acc_max.z - acc_min.z) <= 0.1) {
        DOUBLE_TAP_LOGI("trigger plat status");
        return true;
    }
    return false;
}

static void judge_amd_status(double_tap_state *state)
{
    double_tap_sensor_data last_gdata;
    double_tap_sensor_data cur_gdata;
    double_tap_sensor_data sum_gdata;
    int temp = 0;
    temp = (state->acc_point - 1 + ACC_CHECK_LEN) % ACC_CHECK_LEN;
    memcpy(&cur_gdata, &state->acc_buf[temp], sizeof(double_tap_sensor_data));
    if (!state->first_acc) {
        temp = (state->acc_point - 2 + ACC_CHECK_LEN) % ACC_CHECK_LEN;
        memcpy(&last_gdata, &state->acc_buf[temp], sizeof(double_tap_sensor_data));
	if (fabs(cur_gdata.x - last_gdata.x) <  STATIC_BIAS
			&& fabs(cur_gdata.y - last_gdata.y) <  STATIC_BIAS
			&& fabs(cur_gdata.z - last_gdata.z) <  STATIC_BIAS) {
	    state->static_flag_count++;
	    if (state->static_flag_count >= AMD_BUFFER_SIZE) {
                state->static_flag_count = AMD_BUFFER_SIZE;
		state->current_status = AMD_STATIC;
		memset(&sum_gdata, 0, sizeof(double_tap_sensor_data));
		for (int i = 0; i < G_FILTER_NUM; i++) {
                    temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
		    sum_gdata.x += state->acc_buf[temp].x;
		    sum_gdata.y += state->acc_buf[temp].y;
		    sum_gdata.z += state->acc_buf[temp].z;
		}
		memcpy(&state->static_gdata, &sum_gdata, sizeof(double_tap_sensor_data));
	    }
	} else {
            memset(&sum_gdata, 0, sizeof(double_tap_sensor_data));
	    for (int i = 0; i < G_FILTER_NUM; i++) {
                temp = (state->acc_point - 1 -i + ACC_CHECK_LEN) % ACC_CHECK_LEN;
		sum_gdata.x += state->acc_buf[temp].x;
		sum_gdata.y += state->acc_buf[temp].y;
		sum_gdata.z += state->acc_buf[temp].z;
	    }
	    if (((float)fabs(state->static_gdata.x - sum_gdata.x) / G_FILTER_NUM > STATIC_BIAS) ||
                ((float)fabs(state->static_gdata.y - sum_gdata.y) / G_FILTER_NUM > STATIC_BIAS) ||
		((float)fabs(state->static_gdata.z - sum_gdata.z) / G_FILTER_NUM > STATIC_BIAS)) {
                state->static_flag_count = 0;
		state->current_status = AMD_MOVE;
	    }
	}
    }
}

static void change_sensor_state(double_tap_state *state)
{
   if (AMD_MOVE != state->current_status)  {
       state->sensor_ops->sensor_change_state(GRAVITY_TYPE, false);
       state->sensor_ops->sensor_change_state(PROX_TYPE, false);
   } else {
       state->sensor_ops->sensor_change_state(GRAVITY_TYPE, true);
       state->sensor_ops->sensor_change_state(PROX_TYPE, true);
   }
}
static void copy_input_to_buffer(double_tap_state *state, double_tap_sensor_data *input)
{
    bool is_report = false;
    memcpy(&state->acc_buf[state->acc_point], input, sizeof(double_tap_sensor_data));
    DOUBLE_TAP_LOGI("acc_x:%d, acc_y:%d, acc_z:%d",(int)(input->x * 1000), (int)(input->y * 1000), (int)(input->z * 1000));
    state->acc_point++;
    state->acc_point %= ACC_CHECK_LEN;

    judge_amd_status(state);
    if (state->first_acc) {
        state->first_acc = false;
    }
    if (state->current_status != state->last_status) {
        state->last_status = state->current_status;
        DOUBLE_TAP_LOGI("phone current status %d",state->current_status);
	change_sensor_state(state);
    }
    if (state->current_status == AMD_STATIC) {
        state->dbt_fired = 0;
    }
    if (state->dbt_fired) {
        if (!state->is_check_history) {
            is_report = judge_acc_history_state(state);
            DOUBLE_TAP_LOGI("is_report = %d",is_report);
            if (!is_report) {
                state->dbt_fired = 0;
                state->is_check_history = false;
            }
        } else {
            state->count++;
            if (state->count == 10) {
                state->count = 0;
                state->dbt_fired = 0;
                state->is_check_history = false;
                if (!judge_is_plat_status(state)) {
                    report_event_handle(state);
                }
            }
        }
    }
}

static void copy_input_to_gravity_buffer(double_tap_state *state, double_tap_sensor_data *input)
{
    memcpy(&state->gravity_buf[state->gravity_count], input, sizeof(double_tap_sensor_data));
    state->gravity_count++;
    state->gravity_count %= GRAV_CHECK_LEN;
}

void double_tap_process_sensor_data(double_tap_sensor_data *input)
{
    double_tap_state *state = &g_double_tap_state;

    if (!(check_data_valid(state, input->timestamp, input->type))) {
        DOUBLE_TAP_LOGI("double_tap algo data ");
        return;
    }

    switch (input->type) {
    case ACCEL_TYPE:
        copy_input_to_buffer(state, input);

        break;

    case PROX_TYPE:
        DOUBLE_TAP_LOGI("prx_state = %d, prox_data = %d", (int)input->x, (int)input->y);
        state->prx_state = input->x;

        break;

    case IC_DOUBLE_TAP_TYPE:

        DOUBLE_TAP_LOGI("double_tap = %d", (int)input->x);
        state->dbt_state = input->x;
        handle_dtp_event(state);
        break;
    case GRAVITY_TYPE:
        copy_input_to_gravity_buffer(state, input);
        break;

    default:
        break;
    }
}
void double_tap_Reset()
{
    double_tap_state *state = &g_double_tap_state;

    state->sampleRate = ACC_SAMPLE_RATE;
    memset(state->last_data_ts, 0, sizeof(state->last_data_ts));
    state->prx_state = 0;
    state->acc_point = 0;
    state->gravity_count = 0;
    state->count = 0;
    state->first_acc = true;
    state->is_check_history = false;
    state->current_status = AMD_UNKNOWN;
    state->last_status = AMD_UNKNOWN;

    state->sensor_ops->sensor_change_state(ACCEL_TYPE, true);
    //state->sensor_ops->sensor_change_state(PROX_TYPE, true);
    state->sensor_ops->sensor_change_state(IC_DOUBLE_TAP_TYPE, true);
    //state->sensor_ops->sensor_change_state(GRAVITY_TYPE, true);
}
void double_tap_close()
{
    double_tap_state *state = &g_double_tap_state;

    state->algo_state = ALGO_CLOSE;

    state->sensor_ops->sensor_change_state(ACCEL_TYPE, false);
    state->sensor_ops->sensor_change_state(PROX_TYPE, false);
    state->sensor_ops->sensor_change_state(IC_DOUBLE_TAP_TYPE, false);
    state->sensor_ops->sensor_change_state(GRAVITY_TYPE, false);

    DOUBLE_TAP_LOGI("double_tap close.");
}
void double_tap_algo_register(double_tap_state **state)
{
    DOUBLE_TAP_LOGI("double_tap algo register.");

    memset(&g_double_tap_state, 0, sizeof(double_tap_state));

    g_double_tap_state.algo_state = ALGO_REGISTER;

    *state = &g_double_tap_state;
}
