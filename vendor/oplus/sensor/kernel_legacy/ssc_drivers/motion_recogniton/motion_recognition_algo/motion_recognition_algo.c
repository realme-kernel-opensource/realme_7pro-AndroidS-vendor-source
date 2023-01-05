/************************************************************************************
** Copyright (C), 2008-2020, OPPO Mobile Comm Corp., Ltd
** OPLUS_FEATURE_ACTIVITY_RECOGNITION
** File: motion_recognition_algo.c
**
** Description:
**      The specific algorithm for how to detect motion state.
**
** Version: 1.0
** Date created: 2019/05/27
**
** --------------------------- Revision History: ------------------------------------
*  <version>        <date>         <author>                   <desc>
**************************************************************************************/

#include "motion_recognition_algo.h"
#include "model_parameter.h"
#include "math_util.h"

static motion_recognition_state g_motion_recognition_state;
extern void motion_recognition_classifier(struct motion_recognition_persistent_state *);

static void report_event_handle(motion_recognition_state *state)
{
    static uint16_t report_count = 0;
    report_count++;
    report_count %= 0xFFFF;
    MOTION_RECOGNITION_LOG_4("report_event_handle: count= %d, in_car= %d, activity= %d, timestamp= %lu",
                report_count,
                state->in_car_state,
                state->activity_mode,
                state->sensor_ops->get_current_time_tick());

    state->sensor_ops->report(report_count, state->in_car_state, state->activity_mode, state->sensor_ops->get_current_time_tick());
}

static void state_machine(motion_recognition_state *state, int activity_mode)
{
    MOTION_RECOGNITION_LOG_0("motion_recognition start machine.");
    if (state->in_car_temp == INCAR_OUT) {
        if (activity_mode == ACTIVITY_IN_MOTOR_TRANSIT) {
            state->in_car_state = INCAR_DRIVE;
            state->in_car_temp = INCAR_IN;
        }
        if (state->end_drive_detected) {
            if (state->end_drive_counter == 0) {
                state->in_car_state = INCAR_OUT;
                state->end_drive_detected = false;
            } else {
                state->end_drive_counter--;
            }
        }
    } else {
        if (activity_mode == ACTIVITY_IN_MOTOR_TRANSIT) {
            state->in_car_state = INCAR_DRIVE;
        } else if (activity_mode == ACTIVITY_WALK || activity_mode == ACTIVITY_RUN) {
            state->in_car_state = INCAR_END;
            state->in_car_temp = INCAR_OUT;
            state->end_drive_counter = END_DRIVE_COUNTER;
            state->end_drive_detected = true;
        }
    }

    if (state->activity_mode != activity_mode) {
        state->activity_mode = activity_mode;
        report_event_handle(state);
    }

    if (state->in_car_pre_state != state->in_car_state) {
        state->in_car_pre_state = state->in_car_state;
        report_event_handle(state);
    }
}

static void states_process(motion_recognition_state *state)
{
    for (int index = 0; index < MAX_MOTION_STATES; index++) {
        int activity_mode = state->_persistent.classify_data_out.labels[index * 2];
        int motion_state = state->_persistent.classify_data_out.labels[index * 2 + 1];
        if (motion_state == MOTION_STATE_ENTRY || motion_state == MOTION_STATE_IN) {
            MOTION_RECOGNITION_LOG_2("current activity_mode is %d , motion_state is %d", activity_mode, motion_state);
            state_machine(state, activity_mode);
        }
    }
}

static void process_accels(motion_recognition_state *state)
{
    state->_persistent.classify_data_in.action[0] = 2;
    state->_persistent.classify_data_in.action[1] = 1;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        motion_recognition_matrix_set(&state->_persistent.downsampled_data20X3,
            i, 0, state->acc_buffer[i].x_value);
        motion_recognition_matrix_set(&state->_persistent.downsampled_data20X3,
            i, 1, state->acc_buffer[i].y_value);
        motion_recognition_matrix_set(&state->_persistent.downsampled_data20X3,
            i, 2, state->acc_buffer[i].z_value);
    }

    motion_recognition_classifier(&state->_persistent);
    state->_persistent.classify_data_in.action[0] = 3;
    state->_persistent.classify_data_in.action[1] = 1;
    motion_recognition_classifier(&state->_persistent);

    state->_persistent.classify_data_in.action[0] = 4;
    state->_persistent.classify_data_in.action[1] = 1;

    motion_recognition_classifier(&state->_persistent);

    states_process(state);
}

static void motion_recognition_pause(void) {
    motion_recognition_state *state = &g_motion_recognition_state;
    state->activity_mode = ACTIVITY_STATIONARY;
    state->in_car_state = INCAR_OUT;
    state->in_car_temp = INCAR_OUT;
    state->in_car_pre_state = INCAR_OUT;
    state->end_drive_detected = false;
    state->end_drive_counter = 0;
    state->acc_count = 0;
    memset(&state->acc_buffer, 0 , sizeof(state->acc_buffer));
    motion_recognition_persistent_reset_state(&state->_persistent);
    state->sensor_ops->sensor_change_state(ACC_20_TYPE, false);
    state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
    state->algo_state = ALGO_PAUSE;
}

void motion_recognition_check_sensor_data(motion_recognition_sensor_data *input)
{
    motion_recognition_state *state = &g_motion_recognition_state;
    switch (input->type) {
    case ACC_20_TYPE:
        state->acc_buffer[state->acc_count % BUFFER_SIZE].x_value = input->x_value;
        state->acc_buffer[state->acc_count % BUFFER_SIZE].y_value = input->y_value;
        state->acc_buffer[state->acc_count % BUFFER_SIZE].z_value = input->z_value;
        state->acc_count++;
        if (state->acc_count == BUFFER_SIZE) {
            MOTION_RECOGNITION_LOG_0("motion_recognition start classify_data.");
            state->acc_count = 0;
            process_accels(state);
            memset(&state->acc_buffer, 0, sizeof(state->acc_buffer));
        }
        break;
    case AMD_TYPE:
        if (input->x_value == AMD_STATIONARY) {
            if (state->algo_state == ALGO_RUNNING) { // If algorithm is running, set a timer
                MOTION_RECOGNITION_LOG_0("enter amd , start timer !");
                state->sensor_ops->sensor_change_state(TIMER_TYPE, true);
            }
        } else {
            if (state->algo_state == ALGO_RUNNING) {
                // If algorithm is running, cancel the timer
                MOTION_RECOGNITION_LOG_0("exit amd , cancel timer !");
                state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
            } else if (state->algo_state == ALGO_PAUSE) { // If algorithm is pause, enable the accel
                MOTION_RECOGNITION_LOG_0("exit amd , restart the algo !");
                state->sensor_ops->sensor_change_state(ACC_20_TYPE, true);
                state->algo_state = ALGO_RUNNING;
            }
        }
        break;
    case TIMER_TYPE:
        MOTION_RECOGNITION_LOG_0("timer done, close acc, report event, pause the algo !");
        if (state->activity_mode != ACTIVITY_STATIONARY) {
            state->activity_mode = ACTIVITY_STATIONARY;
            report_event_handle(state);
        }
        // pause the algorithm
        motion_recognition_pause();
        break;
    default:
        break;
    }
}

void motion_recognition_Reset(void)
{
    motion_recognition_state *state = &g_motion_recognition_state;
    MOTION_RECOGNITION_LOG_1("motion_recognition Reset. algo state = %d", state->algo_state);

    if (state->algo_state == ALGO_RUNNING || state->algo_state == ALGO_PAUSE) {
        report_event_handle(state);// report current state to new client
    } else {
        state->acc_sample_rate = MOTION_RECOGNITION_ACC_CHECK_HZ;
        state->algo_state = ALGO_RUNNING;

        state->activity_mode = ACTIVITY_UNKNOWN;
        state->in_car_state = INCAR_OUT;
        state->in_car_temp = INCAR_OUT;
        state->in_car_pre_state = INCAR_OUT;
        state->end_drive_detected = false;
        state->end_drive_counter = 0;

        state->acc_count = 0;
        memset(&state->acc_buffer, 0 , sizeof(state->acc_buffer));
        state->_persistent.malloc = state->sensor_ops->malloc;
        state->_persistent.free = state->sensor_ops->free;
        motion_recognition_persistent_init(&state->_persistent);

        state->sensor_ops->sensor_change_state(ACC_20_TYPE, true);
        state->sensor_ops->sensor_change_state(AMD_TYPE, true);
        state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
    }
}

void motion_recognition_close(void)
{
    motion_recognition_state *state = &g_motion_recognition_state;
    state->algo_state = ALGO_CLOSE;
    state->sensor_ops->sensor_change_state(ACC_20_TYPE, false);
    state->sensor_ops->sensor_change_state(AMD_TYPE, false);
    state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
    motion_recognition_persistent_deinit(&state->_persistent);
    MOTION_RECOGNITION_LOG_0("motion_recognition close.");
}

void motion_recognition_algo_register(motion_recognition_state **state)
{
    MOTION_RECOGNITION_LOG_0("motion_recognition algo register.");

    memset(&g_motion_recognition_state, 0, sizeof(motion_recognition_state));

    g_motion_recognition_state.algo_state = ALGO_REGISTER;

    *state = &g_motion_recognition_state;
}

