/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ELEVATOR_DETECT
** File: elevator_detect_algo.c
**
** Description:
**      The specific algorithm for how to detect elevator.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "elevator_detect_algo.h"

static elevator_detect_state g_elevator_detect_state;

static void elevator_detect_copy_data(elevator_detect_sensor_data *from, elevator_detect_sensor_data *to)
{
    to->x_value = from->x_value;
    to->y_value = from->y_value;
    to->z_value = from->z_value;
    to->timestamp = from->timestamp;
}

static void QuatToRotMat(float rot_mat[9], float quat[4])
{
    float X_value = quat[0];
    float Y_value = quat[1];
    float Z_value = quat[2];
    float W_value = quat[3];

    float xx = X_value * X_value;
    float xy = X_value * Y_value;
    float xz = X_value * Z_value;
    float xw = X_value * W_value;
    float yy = Y_value * Y_value;
    float yz = Y_value * Z_value;
    float yw = Y_value * W_value;
    float zz = Z_value * Z_value;
    float zw = Z_value * W_value;

    rot_mat[0]  = 1 - 2 * (yy + zz);
    rot_mat[1]  =     2 * (xy - zw);
    rot_mat[2]  =     2 * (xz + yw);
    rot_mat[3]  =     2 * (xy + zw);
    rot_mat[4]  = 1 - 2 * (xx + zz);
    rot_mat[5]  =     2 * (yz - xw);
    rot_mat[6]  =     2 * (xz - yw);
    rot_mat[7]  =     2 * (yz + xw);
    rot_mat[8]  = 1 - 2 * (xx + yy);
}

static elevator_detect_sensor_data low_pass_filter(
            elevator_detect_sensor_data new_sensor_data,
            elevator_detect_sensor_data filtered_sensor_data,
            float filter_coefficient)
{
    filtered_sensor_data.x_value = new_sensor_data.x_value * filter_coefficient + (1 - filter_coefficient) * filtered_sensor_data.x_value;
    filtered_sensor_data.y_value = new_sensor_data.y_value * filter_coefficient + (1 - filter_coefficient) * filtered_sensor_data.y_value;
    filtered_sensor_data.z_value = new_sensor_data.z_value * filter_coefficient + (1 - filter_coefficient) * filtered_sensor_data.z_value;
    return filtered_sensor_data;
}

static void report_event_handle(elevator_detect_state *state)
{
    static uint16_t report_count = 0;

    report_count++;
    report_count %= 0xFFFF; // just elevator_detect report count to check slpi and sensor hal

    ELEVATOR_DETECT_LOG_2("Report elevator_detect, elevator event state = %d, report_count = %d", state->current_event_state, report_count);

    state->sensor_ops->report(state->current_event_state, report_count);
}

static SIGN_STATUS transform_to_sign(float value_of_z)
{
    float diff = value_of_z - GRAVITY_EARTH;
    if (fabsf(diff) <= THRESHOLD_BOTTOM || fabsf(diff) >= THRESHOLD_TOP) {
        return SIGN_UNKNOWN;
    }

    if (diff > 0) {
        return SIGN_POSITIVE;
    } else {
        return SIGN_NEGTIVE;
    }
}

static SIGN_STATUS transform(float value_of_z)
{
    float diff = value_of_z - GRAVITY_EARTH;
    if (fabsf(diff) < STEP_COUNTER_SYMBOL_THRESHOLD) {
        return SIGN_UNKNOWN;
    }

    if (diff > 0) {
        return SIGN_POSITIVE;
    } else {
        return SIGN_NEGTIVE;
    }
}

static void enable_step_detector(elevator_detect_state *state)
{
    //Enable step_count if satisfy the condition
    if (state->current_event_state == EVENT_STATE_UNKNOWN) {
        state->current_event_state = EVENT_STATE_ENTER;
        // only report when first detect!
        report_event_handle(state);
    }
    ELEVATOR_DETECT_LOG_0("detect the pair of hill and valley!");
}

static void elevator_detect(elevator_detect_sensor_data sensor_data, elevator_detect_sensor_data *input, elevator_detect_state *state)
{
    SIGN_STATUS current_sign = transform_to_sign(sensor_data.z_value);

    if ((current_sign != SIGN_UNKNOWN) && (state->last_sign_state == current_sign)) {
        state->same_sign_count++;
        //ELEVATOR_DETECT_LOG_1("Same sign count increases to %d", state->same_sign_count);
        state->speed += DELTA_TIME * (sensor_data.z_value - GRAVITY_EARTH);
        state->distance += DELTA_TIME * state->speed;

        if ((fabsf(sensor_data.x_value) < THRESHOLD_BOTTOM) && (fabsf(sensor_data.y_value) < THRESHOLD_BOTTOM)) {
            state->stationary_count++;
        }
    } else {
        //There is effient wave, Let's make pair to detect elevator.
        if (state->same_sign_count > THRESHOLD_WAVE_STATE_NUM) {
            // The horizontal direction is relatively stable.
            if (((float)state->stationary_count / state->same_sign_count) > MINIMUM_STATIONARY_RATIO) {
                //Current waveform is HILL.
                if (state->last_sign_state == SIGN_POSITIVE) {
                    state->distance_peak = state->distance;
                    //make pair the waveform.
                    if (state->last_wave_state == WAVE_VALLEY) {
                        //ensure the time interval of two wave is close.
                        if ((state->sensor_ops->get_delta_time_ms(input->timestamp - state->last_wave_end_ts) < INTERVAL_MAKE_PAIR_WAVEFORM) &&
                            ((state->distance_peak + state->distance_valley) > MINIMUM_CHANGED_DISTANCE)) {
                            state->current_elevator_state = ELEVATOR_DOWN;
                            state->last_wave_state = WAVE_UNKNOWN;
                            state->last_wave_end_ts = 0;
                            ELEVATOR_DETECT_LOG_0("ELEVATOR_DOWN.");
                            //reset the current elevator state
                            state->current_elevator_state = ELEVATOR_UNKNOWN;
                            //Enable timer and cmc if satisfy the condition
                            enable_step_detector(state);
                        } else {
                            // can not make pair.
                            state->last_wave_state = WAVE_HILL;
                            state->last_wave_end_ts = input->timestamp;
                        }
                    } else {
                        // can not make pair.
                        state->last_wave_state = WAVE_HILL;
                        state->last_wave_end_ts = input->timestamp;
                    }
                } else {
                    //Current waveform is VALLEY.
                    state->distance_valley = -state->distance;
                    if (state->last_wave_state == WAVE_HILL) {
                        if ((state->sensor_ops->get_delta_time_ms(input->timestamp - state->last_wave_end_ts) < INTERVAL_MAKE_PAIR_WAVEFORM) &&
                                ((state->distance_peak + state->distance_valley) > MINIMUM_CHANGED_DISTANCE)) {
                            state->current_elevator_state = ELEVATOR_UP;
                            state->last_wave_state = WAVE_UNKNOWN;
                            state->last_wave_end_ts = 0;
                            ELEVATOR_DETECT_LOG_0("ELEVATOR_UP.");
                            //Enable timer and cmc if satisfy the condition
                            state->current_elevator_state = ELEVATOR_UNKNOWN;
                            enable_step_detector(state);
                        } else {
                            // can not make pair.
                            state->last_wave_state = WAVE_VALLEY;
                            state->last_wave_end_ts = input->timestamp;
                        }
                    } else {
                        // can not make pair.
                        state->last_wave_state = WAVE_VALLEY;
                        state->last_wave_end_ts = input->timestamp;
                    }
                }
            }
        }
        // update the sign flag
        state->last_sign_state = current_sign;
        state->same_sign_count = 0;
        state->stationary_count = 0;
        state->speed = 0;
        state->distance = 0;
    }
}

static bool walke_detect(float value, elevator_detect_sensor_data *input, elevator_detect_state *state)
{
    SIGN_STATUS current_sign = transform(value);
    bool result = false;

    //First Enter The Hill
    if (current_sign == SIGN_POSITIVE && state->walk_detect_last_sign == SIGN_UNKNOWN) {
        state->walk_detect_enter_hill_ts = input->timestamp;
        //Exit the Valley
    } else if (current_sign == SIGN_UNKNOWN && state->walk_detect_last_sign == SIGN_NEGTIVE) {
        if (state->walk_detect_enter_hill_ts != 0) {
            // Catch one step
            if ((state->sensor_ops->get_delta_time_ms(input->timestamp - state->walk_detect_enter_hill_ts) > MAKE_STEP_TIME_BOTTOM)
                && (state->sensor_ops->get_delta_time_ms(input->timestamp - state->walk_detect_enter_hill_ts) < MAKE_STEP_TIME_TOP)) {
                    //Latest Step Count is Effective!
                if (state->sensor_ops->get_delta_time_ms(input->timestamp - state->walk_detect_last_step_ts) < STEP_EFFECTIVE_TIME) {
                    state->step_count++;
                    // Step Detect Successfully!
                    if (state->step_count == STEP_DETECT_THRESHOLD) {
                        state->step_count = 0;
                        result = true;
                    }
                } else {
                    state->step_count = 1;
                }
                state->walk_detect_last_step_ts = input->timestamp;
            }
            // Clear the enter Hill State
            state->walk_detect_enter_hill_ts = 0;
        }
    }

    state->walk_detect_last_sign = current_sign;
    return result;
}

void elevator_detect_check_sensor_data(elevator_detect_sensor_data *input)
{
    // elevator_detect_sensor_data cur_data;
    elevator_detect_state *state = &g_elevator_detect_state;
    float quat[4];
    float rot_mat[9];

    switch (input->type) {
    case ACC_TYPE:
        elevator_detect_copy_data(input, &state->current_acc);
        break;
    case GAMEROTATION_VECTOR_TYPE:
        quat[0] = input->x_value;
        quat[1] = input->y_value;
        quat[2] = input->z_value;
        quat[3] = (1 - quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2]);
        quat[3] = (quat[3] > 0.0f) ? sqrtf(quat[3]) : 0.0f;

        QuatToRotMat(rot_mat, quat);

        //calculate the z axix value of the world acc!
        elevator_detect_sensor_data sensor_data;
        sensor_data.x_value = state->current_acc.x_value * rot_mat[0] + state->current_acc.y_value * rot_mat[1] + state->current_acc.z_value * rot_mat[2];
        sensor_data.y_value = state->current_acc.x_value * rot_mat[3] + state->current_acc.y_value * rot_mat[4] + state->current_acc.z_value * rot_mat[5];
        sensor_data.z_value = state->current_acc.x_value * rot_mat[6] + state->current_acc.y_value * rot_mat[7] + state->current_acc.z_value * rot_mat[8];
        sensor_data.timestamp = input->timestamp;
        state->filtered_sensor_data = low_pass_filter(sensor_data, state->filtered_sensor_data, LOW_PASS_COEFFICIENT);
        elevator_detect(state->filtered_sensor_data, input, state);

        //WALK DETECT
        if (state->current_event_state == EVENT_STATE_ENTER) {
            if (walke_detect(state->filtered_sensor_data.z_value, input, state)) {
                state->current_event_state = EVENT_STATE_EXIT;
                report_event_handle(state);

                state->current_event_state = EVENT_STATE_UNKNOWN;
                report_event_handle(state);
            }
        }
        break;
    case AMD_TYPE:
        if (input->x_value == AMD_STATIONARY) {
            if (state->algo_state == ALGO_RUNNING) { // If algorithm is running, set a timer
                ELEVATOR_DETECT_LOG_0("enter amd , start timer !");
                state->sensor_ops->sensor_change_state(TIMER_TYPE, true);
            }
        } else {
            if (state->algo_state == ALGO_RUNNING) {
                // If algorithm is running, cancel the timer
                ELEVATOR_DETECT_LOG_0("exit amd , cancel timer !");
                state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
            } else if (state->algo_state == ALGO_CLOSE)  { // If algorithm is running, enable the sensor
                elevator_detect_Reset();
            }
        }
        break;
    case TIMER_TYPE:
        ELEVATOR_DETECT_LOG_0("timer done, close acc, report event, reset intermediate variables !");
        if (state->current_event_state != EVENT_STATE_UNKNOWN) {
            state->current_event_state = EVENT_STATE_UNKNOWN;
            report_event_handle(state);
        }
        // close the algorithm, and enable the amd!
        elevator_detect_close();
        state->sensor_ops->sensor_change_state(AMD_TYPE, true);
        break;
    default:
        break;
    }
}

void elevator_detect_Reset(void)
{
    elevator_detect_state *state = &g_elevator_detect_state;

    if (state->algo_state == ALGO_RUNNING) {
        report_event_handle(state);// report current state to new client
    } else {
        state->algo_state = ALGO_RUNNING;
        state->last_sign_state = SIGN_UNKNOWN;
        state->walk_detect_last_sign = SIGN_UNKNOWN;
        state->last_wave_state = WAVE_UNKNOWN;
        state->current_elevator_state = ELEVATOR_UNKNOWN;
        state->current_event_state = EVENT_STATE_UNKNOWN;

        state->acc_sampleRate = ELEVATOR_DETECT_SENSOR_COLLECT_HZ;
        state->game_rv_sampleRate = ELEVATOR_DETECT_SENSOR_COLLECT_HZ;

        state->last_wave_end_ts = 0;
        state->walk_detect_enter_hill_ts = 0;
        state->walk_detect_last_step_ts = 0;
        state->moving_count = 0;

        state->same_sign_count = 0;
        state->stationary_count = 0;
        state->speed = 0;
        state->distance = 0;
        state->distance_peak = 0;
        state->distance_valley = 0;
        state->step_count = 0;

        ELEVATOR_DETECT_LOG_0("elevator_detect Reset.");

        state->sensor_ops->sensor_change_state(ACC_TYPE, true);
        state->sensor_ops->sensor_change_state(GAMEROTATION_VECTOR_TYPE, true);
        state->sensor_ops->sensor_change_state(AMD_TYPE, true);
        state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
    }
}

void elevator_detect_close(void)
{
    elevator_detect_state *state = &g_elevator_detect_state;

    if (state->algo_state != ALGO_CLOSE) {
        state->algo_state = ALGO_CLOSE;
        state->sensor_ops->sensor_change_state(ACC_TYPE, false);
        state->sensor_ops->sensor_change_state(GAMEROTATION_VECTOR_TYPE, false);
        state->sensor_ops->sensor_change_state(AMD_TYPE, false);
        state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
        ELEVATOR_DETECT_LOG_0("elevator_detect close.");
    }
}

void elevator_detect_algo_register(elevator_detect_state **state)
{
    ELEVATOR_DETECT_LOG_0("elevator_detect algo register.");

    memset(&g_elevator_detect_state, 0, sizeof(elevator_detect_state));

    g_elevator_detect_state.algo_state = ALGO_REGISTER;

    *state = &g_elevator_detect_state;
}

