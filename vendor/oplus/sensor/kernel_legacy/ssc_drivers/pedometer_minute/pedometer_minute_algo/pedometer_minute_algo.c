#include "pedometer_minute_algo.h"

static uint16_t report_count = 0;
static uint32_t last_report_value = 0;
#define SNS_CMC_STATIONARY_STEP  20
#define SNS_CMC_WALK_STEP  60
#define SNS_CMC_RUN_STEP  120

static pedometer_minute_state g_pedometer_minute_state;

#ifdef REGISTRY_CMC
static void check_move_status(uint32_t current_report_value,uint8_t* move_state){
    uint32_t step_minute = 0;
    if(last_report_value == 0){
        //just use the cmc status
    }else {
        step_minute = current_report_value - last_report_value;
        if(*move_state == MOTION_STATIONARY)//stationary
        {
            if (step_minute > SNS_CMC_RUN_STEP)
            {
                *move_state = MOTION_RUN;//run
            } else if (step_minute > SNS_CMC_STATIONARY_STEP) {
                *move_state = MOTION_STATIONARY;//stationary
            }
        } else if (*move_state == MOTION_WALK) {//walk
            if (step_minute > SNS_CMC_RUN_STEP)
            {
                *move_state = MOTION_RUN;//run
            }else if (step_minute < SNS_CMC_STATIONARY_STEP) {
                *move_state = MOTION_STATIONARY;//stationary
            }
        } else if (*move_state == MOTION_RUN ) {
            if (step_minute < SNS_CMC_WALK_STEP) {
                *move_state = MOTION_WALK;//walk
            } else if (step_minute < SNS_CMC_STATIONARY_STEP)
            {
                *move_state = MOTION_STATIONARY;//stationary
            }
        } else {
            if (step_minute > SNS_CMC_RUN_STEP) {
                *move_state = MOTION_RUN;//run
            } else if (step_minute < SNS_CMC_STATIONARY_STEP){
                *move_state = MOTION_STATIONARY;
            } else {
                *move_state = MOTION_WALK;
            }
        }
    }
    PEDEMETER_LOG_3("check_move_status,  move_status = %d,step_minut = %d last_report_value = %d",*move_state ,step_minute,last_report_value);
    last_report_value = current_report_value;
}
#endif

static void report_event_handle(pedometer_minute_state *state, uint64_t cur_time)
{
    report_count ++;
    report_count %= 0xFFFF;   // just pedometer_minute report count to check slpi and sensor hal

#ifdef REGISTRY_CMC
    check_move_status(state->current_step_counter,&state->current_move_state);
#endif
    PEDEMETER_LOG_3("Report pedometer_minute,  count = %d, report_count = %d move_state = %d",
	    state->current_step_counter,report_count,state->current_move_state);
    PEDEMETER_LOG_2("Report pedometer_minute, run_count = %d, walk_count = %d",
	    state->current_run_step_counter, state->current_walk_step_counter);

    state->sensor_ops->report(state->current_step_counter, state->current_run_step_counter,
	    state->current_walk_step_counter,report_count, state->current_move_state, 0, cur_time);
}
static void report_pre_event_handle(pedometer_minute_state *state, uint32_t pre_step, uint32_t pre_run_step,
	uint32_t pre_walk_step, uint8_t move_state, int time_gap, uint64_t cur_time)
{
    report_count ++;
    report_count %= 0xFFFF;   // just pedometer_minute report count to check slpi and sensor hal

#ifdef REGISTRY_CMC
    check_move_status(pre_step,&move_state);
#endif
    PEDEMETER_LOG_4("Report pre pedometer_minute,  count = %d, report_count = %d move_state = %d time_gap = %d", pre_step, report_count,move_state,time_gap);

    PEDEMETER_LOG_2("Report pre pedometer_minute, run_count = %d, walk_count = %d", pre_run_step, pre_walk_step);
    state->sensor_ops->report(pre_step, pre_run_step, pre_walk_step,report_count, move_state, time_gap, cur_time);
}

static bool check_data_avail(pedometer_minute_state *state, uint64_t timestamp, SENSOR_TYPE type)
{
    uint64_t *pre_ts;
    uint64_t delta_ms;

    pre_ts = &state->last_data_ts[type];

    if( *pre_ts > timestamp)
    {
        return false;
    }
    if(type == TIMER_TYPE)
    {
        PEDEMETER_LOG_2("pedometer_minute pre_ts = %llu,current_ts = %llu",*pre_ts,timestamp);
        delta_ms = state->sensor_ops->get_delta_time_ms(timestamp - *pre_ts);
        if(delta_ms < ( 60*1000 * TIMESTAMP_RATIO))
        {
            return false;
        }
    }
    *pre_ts = timestamp;

    return true;
}
static void pedometer_minute_handle_timer_event(pedometer_minute_state *state, bool ap_state)
{
    int i = 0;

    for(;i < state->timer_count;i++){
        if(ap_state){
            report_pre_event_handle(state,state->pre_move_info[i].pre_step, state->pre_move_info[i].pre_run_step,
		   state->pre_move_info[i].pre_walk_step,state->pre_move_info[i].move_status,
                (state->timer_count - i), state->current_timer_count_stamp + (uint64_t)i);
        } else {
            report_pre_event_handle(state,state->pre_move_info[i].pre_step, state->pre_move_info[i].pre_run_step,
		    state->pre_move_info[i].pre_walk_step, state->pre_move_info[i].move_status,
                (state->timer_count - (i+1)), state->current_timer_count_stamp + (uint64_t)i);
        }
    }
    if(ap_state){
        report_event_handle(state, state->current_timer_count_stamp + (uint64_t)i);
    }
}
static void pedometer_minute_handle_step_event(pedometer_minute_state *state)
{

    if (state->current_step_counter >= state->last_step_counter)
    {
        //
    } else {
        state->current_step_counter = state->last_step_counter + state->current_step_counter;
    }
    if(state->current_move_state == MOTION_WALK) {
	state->current_walk_step_counter += state->current_step_counter - state->last_step_counter;
    } else if (state->current_move_state == MOTION_RUN) {
	state->current_run_step_counter += state->current_step_counter - state->last_step_counter;
    } else {
	//do nothing
    }

    state->last_step_counter = state->current_step_counter;
}

void pedometer_minute_check_sensor_data(pedometer_minute_sensor_data *input)
{
    pedometer_minute_state *state = &g_pedometer_minute_state;

    if ((check_data_avail(state, input->timestamp, input->type) == false))
    {
        PEDEMETER_LOG_0("pedometer_minute algo data not correct");
        return;
    }

    switch (input->type)
    {
        case STEP_COUNTER_TYPE:
            state->current_step_counter = input->x;
            pedometer_minute_handle_step_event(state);
            PEDEMETER_LOG_2("pedometer_minute algo input = %d, cur_step = %d", (uint32_t)input->x, state->current_step_counter);
            break;
        case TIMER_TYPE:
            PEDEMETER_LOG_1("pedometer_minute algo data timer event, timer_count = %d", state->timer_count);
            state->current_timer_count_stamp = input->timestamp;
            if(AP_SUSPEND == state->current_ap_state){
                state->pre_move_info[state->timer_count].pre_step = state->current_step_counter;
		state->pre_move_info[state->timer_count].pre_run_step = state->current_run_step_counter;
		state->pre_move_info[state->timer_count].pre_walk_step = state->current_walk_step_counter;
                state->pre_move_info[state->timer_count].move_status = state->current_move_state;
                state->timer_count++;
                if(state->timer_count == 119)
                {
                    pedometer_minute_handle_timer_event(state,0);
                    state->timer_count = 0;
                }
            }
            else {
                pedometer_minute_handle_timer_event(state,1);
                state->timer_count = 0;
            }

            break;

#ifdef REGISTRY_CMC
        case CMC_TYPE  :
            PEDEMETER_LOG_1("pedometer_minute cmc sensor event %d",(int)input->x);
            state->current_move_state = (uint8_t)input->x;
#endif
        default:
            break;
    }
}
void pedometer_minute_Reset(float samplerate)
{
    pedometer_minute_state *state = &g_pedometer_minute_state;
	PEDEMETER_LOG_1("pedometer_minute sampletate %d",(int)samplerate);

	if(((samplerate - 50) < 0.01) && ((samplerate - 50) > -0.01))
	{
		state->current_ap_state = AP_AWAKE;
	} else {
		state->current_ap_state = AP_SUSPEND;
	}

	if (state->algo_state == ALGO_RESET)
	{
        if (AP_AWAKE == state->current_ap_state) {
            pedometer_minute_handle_timer_event(state,0);
            state->timer_count = 0;
        }
	} else {
	    state->sensor_ops->sensor_change_state(STEP_COUNTER_TYPE, true);
		state->sensor_ops->sensor_change_state(TIMER_TYPE,true);
#ifdef REGISTRY_CMC
	    state->sensor_ops->sensor_change_state(CMC_TYPE,true);
#endif
	    state->algo_state = ALGO_RESET;
	}	
}
void pedometer_minute_close()
{
    pedometer_minute_state *state = &g_pedometer_minute_state;

    state->algo_state = ALGO_CLOSE;
    state->timer_count = 0;

    state->sensor_ops->sensor_change_state(STEP_COUNTER_TYPE, false);
	state->sensor_ops->sensor_change_state(TIMER_TYPE, false);
#ifdef REGISTRY_CMC
    state->sensor_ops->sensor_change_state(CMC_TYPE, false);
#endif

    PEDEMETER_LOG_0("pedometer_minute close.");
}
void pedometer_minute_algo_register(pedometer_minute_state **state)
{
    PEDEMETER_LOG_0("pedometer_minute algo register.");

    memset(&g_pedometer_minute_state, 0, sizeof(pedometer_minute_state));

    g_pedometer_minute_state.algo_state = ALGO_REGISTER;
    g_pedometer_minute_state.last_step_counter = 0;
    g_pedometer_minute_state.current_step_counter = 0;
    g_pedometer_minute_state.current_walk_step_counter = 0;
    g_pedometer_minute_state.current_run_step_counter = 0;
    g_pedometer_minute_state.timer_count = 0;
    g_pedometer_minute_state.current_move_state = 0;
    g_pedometer_minute_state.current_ap_state = AP_SUSPEND;
    last_report_value = 0;

    *state = &g_pedometer_minute_state;
}
