
#include "near_ear_detect_algo.h"

static near_ear_detect_state g_near_ear_detect_state;

static const float typical_position[SS_NUM][7] = {
    {-4.5,  4.5,    -5,     5,      -EG,    -5,     SS_BACK_SLEEP},
    {-5,    5,      -1,     EG,     -6,     EG,     SS_NEAR_EAR_WAKEUP},
    {-5,    5,      -2,     EG,     0.5,    EG,     SS_ROTATE_FLAT_WAKEUP},
    {-4,    4,       2,     EG,     3.5,    9.3,    SS_STATIC_NEAR_EAR_WAKEUP},
    {-5,    5,     0.5,     EG,     3,      EG,     SS_POCKET_WAKEUP},
};

static void near_ear_detect_set_zero(near_ear_detect_sensor_data *data)
{
    data->x = 0;
    data->y = 0;
    data->z = 0;
    data->timestamp = 0;
}

static void near_ear_detect_copy_data(near_ear_detect_sensor_data *from, near_ear_detect_sensor_data *to)
{
    to->x = from->x;
    to->y = from->y;
    to->z = from->z;
    to->timestamp = from->timestamp;
}

static void near_ear_detect_add_data(near_ear_detect_sensor_data *from, near_ear_detect_sensor_data *to)
{
    to->x += from->x;
    to->y += from->y;
    to->z += from->z;
}

static void near_ear_detect_add_to_buffer(near_ear_detect_sensor_data *input,
    near_ear_detect_sensor_data *buffer, int *sample_num)
{
    int idx = (*sample_num) % BUFFER_SIZE;

    near_ear_detect_copy_data(input, &(buffer[idx]));

    (*sample_num)++;
    if ((*sample_num) >= (BUFFER_SIZE << 6)) {
        (*sample_num) = BUFFER_SIZE;
    }
}

static void rad_to_degree(near_ear_detect_sensor_data *data)
{
    data->x = data->x / PIE * (180.0 / NEAR_EAR_DETECT_STABLE_CHECK_HZ);
    data->y = data->y / PIE * (180.0 / NEAR_EAR_DETECT_STABLE_CHECK_HZ);
    data->z = data->z / PIE * (180.0 / NEAR_EAR_DETECT_STABLE_CHECK_HZ);
}

static int get_position(near_ear_detect_sensor_data *dev_posture)
{
    uint32_t ret = 0;
    int i = 0;

    for (i = 0; i < SS_NUM; i++) {
        if (dev_posture->x > typical_position[i][0] &&
            dev_posture->x < typical_position[i][1] &&
            dev_posture->y > typical_position[i][2] &&
            dev_posture->y < typical_position[i][3] &&
            dev_posture->z > typical_position[i][4] &&
            dev_posture->z < typical_position[i][5]
        ) {
            ret |= SS_MASK((int)typical_position[i][6]);
        }
    }

    if (ret == 0) {
        ret |= SS_MASK((int)SS_MOVING);
    }

    return ret;
}

static void calc_gyro_rad(near_ear_detect_state *state)
{
    int i = 0;
    int count = 0;

    near_ear_detect_set_zero(&(state->gyro_rad));

    count = (state->gyro_count > BUFFER_SIZE) ? BUFFER_SIZE : state->gyro_count;

    for (i = 0; i < count; i++) {
        near_ear_detect_add_data(&(state->gyro_buffer[i]), &(state->gyro_rad));
    }
    rad_to_degree(&(state->gyro_rad));

    NEAR_EAR_DETECT_LOG_4("gyro_rad[%d] %d %d %d", state->gyro_count, (int)state->gyro_rad.x,
        (int)state->gyro_rad.y, (int)state->gyro_rad.z);
    //NEAR_EAR_DETECT_LOG_4("gyro_rad(fabs)[%d] %d %d %d",state->gyro_count, (int)fabs(state->gyro_rad.x), (int)fabs(state->gyro_rad.y), (int)fabs(state->gyro_rad.z));
}

static void report_event_handle(near_ear_detect_state *state, device_status ret, uint64_t report_ts)
{
    static uint16_t report_count = 0;
    uint64_t delta_ms_from_lastreport;

    if ((state->sensor_ops->need_check_prox()) && (state->prx_state != 0) && (ret == DEVICE_WAKEUP)) {
        NEAR_EAR_DETECT_LOG_0("prx_state is near, don't report.");
    } else {
        state->dev_status = ret;

        delta_ms_from_lastreport = state->sensor_ops->get_delta_time_ms(report_ts -
                state->last_wakeup_timestamp);

        NEAR_EAR_DETECT_LOG_4("prx_state = %d, cur status = %d, last status = %d, event_delta_ms = %llu",
            state->prx_state, state->dev_status, state->last_dev_status, delta_ms_from_lastreport);

        if ((state->dev_status != state->last_dev_status) || (delta_ms_from_lastreport > 500)) {
            state->last_dev_status = state->dev_status;

            report_count ++;
            report_count %= 0xFFFF;   // just motion report count to check slpi and sensor hal

            NEAR_EAR_DETECT_LOG_2("Report motion, status = %d, report_count = %d", state->dev_status,
                report_count);

            state->sensor_ops->report(state->dev_status, report_count);
        }

        if (state->dev_status == DEVICE_WAKEUP) {
            state->last_wakeup_timestamp = report_ts;
        }

        memset(state->gyro_buffer, 0, sizeof(state->gyro_buffer));
        near_ear_detect_set_zero(&(state->gyro_rad));
        state->gyro_last_rad_x = 0;
    }
}


static bool near_ear_detect_judge_near_ear(near_ear_detect_state *state)
{
    int i = 0;

    if (state->gravity_count < BUFFER_SIZE) {
        NEAR_EAR_DETECT_LOG_1("near_ear_detect_judge_near_ear not enough count in buffer %d",
            state->gravity_count);
        return false;
    }

    i = state->gravity_count - NEAR_EAR_CHECK_NUM;
    near_ear_detect_sensor_data data;

    state->gyro_last_rad_x = state->gyro_rad.x;

    for (; i < state->gravity_count; i++) {
        near_ear_detect_copy_data(&state->gravity_buffer[i % BUFFER_SIZE], &data);

        if ((get_position(&data) & SS_MASK(SS_NEAR_EAR_WAKEUP)) == 0) {
            NEAR_EAR_DETECT_LOG_2("near_ear_detect_judge_near_ear[%d], wrong pos %d", i % BUFFER_SIZE,
                get_position(&data));
            return false;
        }
    }

    float miny = state->gravity_buffer[0].y;
    float maxy = state->gravity_buffer[0].y;
    float maxz = state->gravity_buffer[0].z;

    for (i = 0; i < BUFFER_SIZE; i++) {
        miny = (miny > state->gravity_buffer[i].y) ? state->gravity_buffer[i].y : miny;
        maxy = (maxy < state->gravity_buffer[i].y) ? state->gravity_buffer[i].y : maxy;
        maxz = (maxz < state->gravity_buffer[i].z) ? state->gravity_buffer[i].z : maxz;
    }

    if ((miny > 6) || (maxy - miny < 4) || (maxz < 7.5)) {
        NEAR_EAR_DETECT_LOG_3("near_ear_detect_judge_near_ear incorrect miny = %d, maxy = %d, maxz = %d",
            (int)miny, (int)maxy, (int)maxz);
        return false;
    }

    near_ear_detect_sensor_data avg;
    near_ear_detect_set_zero(&avg);
    i = state->gravity_count - NEAR_EAR_CHECK_NUM;

    for (; i < state->gravity_count; i++) {
        near_ear_detect_add_data(&(state->gravity_buffer[i % BUFFER_SIZE]), &avg);
    }

    avg.x /= NEAR_EAR_CHECK_NUM;
    avg.y /= NEAR_EAR_CHECK_NUM;
    avg.z /= NEAR_EAR_CHECK_NUM;

    if (avg.y < 1) {
        NEAR_EAR_DETECT_LOG_3("near_ear_detect_judge_near_ear incorrect avg %d[%d], cur %d", (int)(avg.y * 1000),
            (int)fabs(avg.y * 1000), (int)(data.y * 1000));
        return false;
    }

    return true;
}

static bool near_ear_detect_is_rotate_to_flat(near_ear_detect_state *state)
{
    if (state->gravity_count < BUFFER_SIZE) {
        NEAR_EAR_DETECT_LOG_1("rotate_to_flat not enough count in buffer %d", state->gravity_count);
        return false;
    }

    near_ear_detect_sensor_data data;
    int i = state->gravity_count - NEAR_EAR_CHECK_NUM;

    for (; i < state->gravity_count; i++) {
        near_ear_detect_copy_data(&state->gravity_buffer[i % BUFFER_SIZE], &data);

        NEAR_EAR_DETECT_LOG_4("rotate_to_flat [%d], %d %d %d", i % BUFFER_SIZE, (int)(data.x * 1000),
            (int)(data.y * 1000), (int)(data.z * 1000));
        NEAR_EAR_DETECT_LOG_4("rotate_to_flat_abs [%d], %d %d %d", i % BUFFER_SIZE, (int)fabs(data.x * 1000),
            (int)fabs(data.y * 1000), (int)fabs(data.z * 1000));

        if ((get_position(&data) & SS_MASK(SS_ROTATE_FLAT_WAKEUP)) == 0) {
            NEAR_EAR_DETECT_LOG_2("wrong rotate to flat data, y = %d[%d]", (int)data.y, (int)fabs(data.y));

            return false;
        }
    }

    float minz = state->gravity_buffer[0].z;
    float miny = state->gravity_buffer[0].y;

    for (i = 0; i < BUFFER_SIZE; i++) {
        minz = (minz > state->gravity_buffer[i].z) ? state->gravity_buffer[i].z : minz;
        miny = (miny > state->gravity_buffer[i].y) ? state->gravity_buffer[i].y : miny;
    }

    if ((minz > -4) || (miny > 5)) {
        NEAR_EAR_DETECT_LOG_4("rotate_to_flat incorrect, minz = %d[%d], miny = %d[%d]", (int)(minz * 1000),
            (int)fabs(minz * 1000), (int)(miny * 1000),
            (int)fabs(miny * 1000));
        return false;
    }

    return true;
}

static bool near_ear_detect_pocket_check(near_ear_detect_state *state)
{
    if (state->gravity_count < BUFFER_SIZE) {
        NEAR_EAR_DETECT_LOG_1("not enough count in buffer %d", state->gravity_count);
        return false;
    }

    near_ear_detect_sensor_data data;
    int i;

    near_ear_detect_copy_data(&state->gravity_buffer[0], &data);

    float miny = data.y;
    float minz = data.z;

    for (i = 0; i < BUFFER_SIZE; i++) {
        miny = (miny > state->gravity_buffer[i].y) ? state->gravity_buffer[i].y : miny;
        minz = (minz > state->gravity_buffer[i].z) ? state->gravity_buffer[i].z : minz;
    }

    if ((miny > 2) || (minz > 4)) {
        NEAR_EAR_DETECT_LOG_2("picket_check incorrect miny = %d, minz = %d", (int)miny, (int)minz);
        return false;
    }

    i = state->gravity_count - NEAR_EAR_CHECK_NUM;

    for (; i < state->gravity_count; i++) {
        near_ear_detect_copy_data(&state->gravity_buffer[i % BUFFER_SIZE], &data);

        NEAR_EAR_DETECT_LOG_4("pocket[%d], %d %d %d", i % BUFFER_SIZE, (int)(data.x * 1000),
            (int)(data.y * 1000), (int)(data.z * 1000));

        if ((get_position(&data) & SS_MASK(SS_POCKET_WAKEUP)) == 0) {
            NEAR_EAR_DETECT_LOG_0("wrong pocket position");

            return false;
        }
    }

    return true;
}

static int near_ear_detect_walk_wakup_judge(near_ear_detect_state *state)
{
    if (state->gyro_rad.x > 30) {
        //NEAR_EAR_DETECT_LOG_0("is near_ear check");
        if (near_ear_detect_judge_near_ear(state)) {
            NEAR_EAR_DETECT_LOG_0("is near_ear, wakeup");
            return DEVICE_WAKEUP;
        }
    }

    if ((float)fabs(state->gyro_rad.y) > 70) {
        //NEAR_EAR_DETECT_LOG_0("rotate to flat check");
        if (near_ear_detect_is_rotate_to_flat(state)) {
            NEAR_EAR_DETECT_LOG_0("rotate to flat, wakeup");
            return DEVICE_WAKEUP;
        }
    }

    if (((float)fabs(state->gyro_rad.z) > 40) && ((float)fabs(state->gyro_rad.y) > 20)) {
        //NEAR_EAR_DETECT_LOG_0("pocket check");
        if (near_ear_detect_pocket_check(state)) {
            NEAR_EAR_DETECT_LOG_0("pocket, wakeup");
            return DEVICE_WAKEUP;
        }
    }

    return DEVICE_POSTURE_RESET;
}

static void near_ear_detect_check_sleep(near_ear_detect_state *state)
{
    uint32_t cur_pos = 0;
    uint32_t last_pos = 0;

    uint64_t cur_ts = 0;
    uint64_t delta_ms_from_last_wakeup = 0;

    device_status ret = DEVICE_POSTURE_RESET;

    near_ear_detect_sensor_data cur_gravity_data;
    near_ear_detect_sensor_data last_gravity_data;

    near_ear_detect_copy_data(&(state->gravity_buffer[(state->gravity_count - 1) % BUFFER_SIZE]),
        &cur_gravity_data);
    near_ear_detect_copy_data(&(state->gravity_buffer[(state->gravity_count - 2) % BUFFER_SIZE]),
        &last_gravity_data);

    ret = near_ear_detect_walk_wakup_judge(state);

    if (state->dev_status == DEVICE_WAKEUP) {  // will sleep
        cur_pos = get_position(&cur_gravity_data);
        last_pos = get_position(&last_gravity_data);

        if (
            ((state->gyro_rad.x < -30) && ((float)fabs(state->gyro_rad.x) > (float)fabs(state->gyro_rad.y))
                && ((float)fabs(state->gyro_rad.x) > (float)fabs(state->gyro_rad.z)) && (cur_gravity_data.y < 1.5)
                && (last_gravity_data.y < 1.5))
            || // case 1: negative direction roate x axis after pick up
            ((((float)fabs(state->gyro_rad.y) > 30) && ((float)fabs(state->gyro_rad.z) > 40))
                && (cur_gravity_data.y < 1) && (last_gravity_data.y < 1))
            ||   // case 2: put down the phone after take out from pocket
            (((float)fabs(state->gyro_rad.y) > 30) && (cur_pos & SS_MASK(SS_BACK_SLEEP))
                && (last_pos & SS_MASK(SS_BACK_SLEEP))) // case 3: backsleep state then turn over
        ) {
            cur_ts = cur_gravity_data.timestamp;

            delta_ms_from_last_wakeup = state->sensor_ops->get_delta_time_ms(cur_ts -
                    state->last_wakeup_timestamp);

            if (delta_ms_from_last_wakeup < 60000) { // will response within 3s
                ret = DEVICE_SLEEP_FROM_WAKE;
                NEAR_EAR_DETECT_LOG_0("go to sleep");
            }
        }
    }

    if (ret != DEVICE_POSTURE_RESET) {
        report_event_handle(state, ret, cur_gravity_data.timestamp);
    }
}

static void change_sensor_state(near_ear_detect_state *state)
{
    if (AMD_MOVE != state->current_Amdstate) {
        state->sensor_ops->sensor_change_state(GYRO_TYPE, false);
        state->sensor_ops->sensor_change_state(GRAVITY_TYPE, false);
    } else {
        state->sensor_ops->sensor_change_state(GYRO_TYPE, true);
        state->sensor_ops->sensor_change_state(GRAVITY_TYPE, true);
    }
}

static void near_ear_detect_judge_amdstate(near_ear_detect_state *state)
{
    near_ear_detect_sensor_data last_gdata;
    near_ear_detect_sensor_data cur_gdata;
    near_ear_detect_sensor_data sum_gdata;
    int i;

    near_ear_detect_copy_data(&(state->acc_buffer[(state->acc_count - 1) % BUFFER_SIZE]), &cur_gdata);

    if (state->acc_count > 1) {
        near_ear_detect_copy_data(&(state->acc_buffer[(state->acc_count - 2) % BUFFER_SIZE]), &last_gdata);

        if (((float)fabs(cur_gdata.x - last_gdata.x) <  STATIC_BIAS) &&
            ((float)fabs(cur_gdata.y - last_gdata.y) <  STATIC_BIAS) &&
            ((float)fabs(cur_gdata.z - last_gdata.z) <  STATIC_BIAS)) {
            state->static_flag_count ++;

            if (state->static_flag_count >= BUFFER_SIZE) { // 50Hz, 20ms, about (BUFFER_SIZE * 20ms)
                state->static_flag_count = BUFFER_SIZE;
                state->current_Amdstate = AMD_STATIC;

                near_ear_detect_set_zero(&sum_gdata);

                i = state->acc_count - G_FILTER_NUM;
                for (; i < state->acc_count; i ++) {
                    near_ear_detect_add_data(&(state->acc_buffer[i % BUFFER_SIZE]), &sum_gdata);
                }

                near_ear_detect_copy_data(&sum_gdata, &(state->static_gdata));  // get the static gdata

                //NEAR_EAR_DETECT_LOG_3("static position [%5d][%5d][%5d]", fabs(state->static_gdata.x/G_FILTER_NUM*1000), fabs(state->static_gdata.y/G_FILTER_NUM*1000), fabs(state->static_gdata.z/G_FILTER_NUM*1000));
            }
        } else {
            if (state->acc_count > G_FILTER_NUM) {
                near_ear_detect_set_zero(&sum_gdata);

                i = state->acc_count - G_FILTER_NUM;
                for (; i < state->acc_count; i ++) {
                    near_ear_detect_add_data(&(state->acc_buffer[i % BUFFER_SIZE]), &sum_gdata);
                }

                if (((float)fabs(state->static_gdata.x - sum_gdata.x) / G_FILTER_NUM > STATIC_BIAS) ||
                    ((float)fabs(state->static_gdata.y - sum_gdata.y) / G_FILTER_NUM > STATIC_BIAS) ||
                    ((float)fabs(state->static_gdata.z - sum_gdata.z) / G_FILTER_NUM > STATIC_BIAS)) {
                    state->static_flag_count = 0;
                    state->current_Amdstate = AMD_MOVE;   // phone is moving
                }

                //NEAR_EAR_DETECT_LOG_3("sum position [%5d][%5d][%5d]", fabs(sum_gdata.x/G_FILTER_NUM*1000), fabs(sum_gdata.y/G_FILTER_NUM*1000), fabs(sum_gdata.z/G_FILTER_NUM*1000));
            }
        }
    }

    //NEAR_EAR_DETECT_LOG_3("curs position [%5d][%5d][%5d]",fabs(cur_gdata.x*1000), fabs(cur_gdata.y*1000), fabs(cur_gdata.z*1000));
    //NEAR_EAR_DETECT_LOG_3("gcount = %d, flaacc_count = [%d], ret = %d", state->acc_count, state->static_flag_count, state->current_Amdstate);
}

static void near_ear_detect_judge_wakeup(near_ear_detect_state *state)
{
    uint64_t cur_ts = 0;
    uint64_t delta_ms_from_md = 0;

    near_ear_detect_sensor_data cur_g_data;
    near_ear_detect_sensor_data last_g_data;

    device_status ret = DEVICE_POSTURE_RESET;

    near_ear_detect_copy_data(&(state->acc_buffer[(state->acc_count - 1) % BUFFER_SIZE]), &cur_g_data);

    if (state->acc_count == 1) {
        state->md_ts = cur_g_data.timestamp;               // get the first gsensor ts as the md ts
        near_ear_detect_copy_data(&cur_g_data,
            &state->md_data);    // get the first gsensor data as the md data

        near_ear_detect_copy_data(&cur_g_data, &last_g_data);
    } else {
        near_ear_detect_copy_data(&(state->acc_buffer[(state->acc_count - 2) % BUFFER_SIZE]), &last_g_data);
    }

    near_ear_detect_judge_amdstate(state);

    // the phone is moving, enable gyro
    // if phone is static, close gyro, and record the gsensor data as md data
    if (state->current_Amdstate == AMD_STATIC) {
        state->md_ts = cur_g_data.timestamp;
        near_ear_detect_copy_data(&cur_g_data, &state->md_data);
    }

    if (state->current_Amdstate != state->last_Amdstate) { // amd changed, close or enable gyro
        NEAR_EAR_DETECT_LOG_2("phone current state = %d, last state= %d", state->current_Amdstate,
            state->last_Amdstate);

        near_ear_detect_set_zero(&(state->gyro_rad));

        state->last_Amdstate = state->current_Amdstate;

        change_sensor_state(state);
    }

    cur_ts = cur_g_data.timestamp;

    delta_ms_from_md = state->sensor_ops->get_delta_time_ms(cur_ts - state->md_ts);

    NEAR_EAR_DETECT_LOG_4("delta_ms_from_md = %llu, cur_g %d, %d, %d", delta_ms_from_md,
        (int)(cur_g_data.x * 1000), (int)(cur_g_data.y * 1000),
        (int)(cur_g_data.z * 1000));

    if (delta_ms_from_md < 1000) { //in the first 1s, use md to judge the position
        uint32_t md_pos = get_position(&(state->md_data));
        uint32_t cur_pos = get_position(&cur_g_data);
        uint32_t last_pos = get_position(&last_g_data);

        if (md_pos & SS_MASK(SS_BACK_SLEEP)) {
            NEAR_EAR_DETECT_LOG_0("check, SS_BACK_SLEEP");

            // judge continuous two data for debounce
            if ((cur_pos & SS_MASK(SS_NEAR_EAR_WAKEUP)) && (last_pos & SS_MASK(SS_NEAR_EAR_WAKEUP))) {
                NEAR_EAR_DETECT_LOG_0("Back sleep, wakeup");
                ret = DEVICE_WAKEUP;
            }
        } else {
            NEAR_EAR_DETECT_LOG_2("cur_data.y = %d, md_y = %d", (int)cur_g_data.y, (int)state->md_data.y);

            // judge continuous two data for debounce
            if (((cur_g_data.y - state->md_data.y) >= WAKEUP_ANGLE_BIAS)
                && (cur_pos & SS_MASK(SS_STATIC_NEAR_EAR_WAKEUP)) &&
                ((last_g_data.y - state->md_data.y) >= WAKEUP_ANGLE_BIAS)
                && (last_pos & SS_MASK(SS_STATIC_NEAR_EAR_WAKEUP))
            ) {
                NEAR_EAR_DETECT_LOG_0("Static near_ear, wakeup");
                ret = DEVICE_WAKEUP;
            }
        }
    }

    if (ret != DEVICE_POSTURE_RESET) {
        report_event_handle(state, ret, cur_ts);
    }
}

static bool check_data_avail(near_ear_detect_state *state, uint64_t timestamp, SENSOR_TYPE type)
{
    uint64_t *pre_ts;
    uint64_t delta_ms;

    pre_ts = &state->last_data_ts[type];

    delta_ms = state->sensor_ops->get_delta_time_ms(timestamp - *pre_ts);

    if(delta_ms < (1000 / state->sampleRate / 2)) {
        return false;
    }

    *pre_ts = timestamp;

    return true;
}

void near_ear_detect_process_sensor_data(near_ear_detect_sensor_data *input)
{
    near_ear_detect_sensor_data cur_data;
    near_ear_detect_state *state = &g_near_ear_detect_state;

    if ((check_data_avail(state, input->timestamp, input->type) == false)
        && (input->type != PROX_TYPE)) {
        return;
    }

    switch (input->type) {
    case ACC_TYPE:
        near_ear_detect_add_to_buffer(input, state->acc_buffer, &(state->acc_count));

        near_ear_detect_copy_data(&(state->acc_buffer[(state->acc_count - 1) % BUFFER_SIZE]), &cur_data);

        NEAR_EAR_DETECT_LOG_3("g_[%d][%d][%d]", (int)(input->x * 1000), (int)(input->y * 1000),
            (int)(input->z * 1000));

        near_ear_detect_judge_wakeup(state);
        break;

    case GYRO_TYPE:
        near_ear_detect_add_to_buffer(input, state->gyro_buffer, &(state->gyro_count));

        calc_gyro_rad(state);
        break;

    case GRAVITY_TYPE:
        near_ear_detect_add_to_buffer(input, state->gravity_buffer, &(state->gravity_count));

        near_ear_detect_copy_data(&(state->gravity_buffer[(state->gravity_count - 1) % BUFFER_SIZE]),
            &cur_data);

        NEAR_EAR_DETECT_LOG_4("gravity_data[%d] %d %d %d", state->gravity_count, (int)(cur_data.x * 1000),
            (int)(cur_data.y * 1000), (int)(cur_data.z * 1000));

        near_ear_detect_check_sleep(state);
        break;

    case PROX_TYPE:

        NEAR_EAR_DETECT_LOG_2("prx_state = %d, prox_data = %d", (int)input->x, (int)input->y);
        state->prx_state = input->x;

    default:
        break;
    }
}

void near_ear_detect_Reset(void)
{
    near_ear_detect_state *state = &g_near_ear_detect_state;

    state->dev_status = DEVICE_POSTURE_RESET;
    state->last_dev_status = DEVICE_POSTURE_RESET;
    state->prx_state = 0;
    state->current_Amdstate = AMD_UNKNOWN;
    state->last_Amdstate = AMD_UNKNOWN;

    state->sampleRate = NEAR_EAR_DETECT_STABLE_CHECK_HZ;
    state->latency = NEAR_EAR_DETECT_NORMAL_LATENCY;

    state->algo_state = ALGO_RESET;

    NEAR_EAR_DETECT_LOG_0("Motion Reset.");

    state->sensor_ops->sensor_change_state(ACC_TYPE, true);

    if (state->sensor_ops->need_check_prox()) {
        state->sensor_ops->sensor_change_state(PROX_TYPE, true);
    }
}

void near_ear_detect_close(void)
{
    near_ear_detect_state *state = &g_near_ear_detect_state;

    state->algo_state = ALGO_CLOSE;

    state->sensor_ops->sensor_change_state(ACC_TYPE, false);
    state->sensor_ops->sensor_change_state(GYRO_TYPE, false);
    state->sensor_ops->sensor_change_state(GRAVITY_TYPE, false);

    if (state->sensor_ops->need_check_prox()) {
        state->sensor_ops->sensor_change_state(PROX_TYPE, false);
    }

    NEAR_EAR_DETECT_LOG_0("Motion close.");
}

void near_ear_detect_algo_register(near_ear_detect_state **state)
{
    NEAR_EAR_DETECT_LOG_0("Motion algo register.");

    memset(&g_near_ear_detect_state, 0, sizeof(near_ear_detect_state));

    g_near_ear_detect_state.algo_state = ALGO_REGISTER;

    *state = &g_near_ear_detect_state;
}

