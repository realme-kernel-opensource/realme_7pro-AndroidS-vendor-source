#include "oplus_devorient_algo.h"
static devorient_algo_state g_devorient_state;

//50HZ, alpfa= 0.25
#define FILTER_TIME_CONSTANT_MS         60
//#define FILTER_TIME_CONSTANT_MS         200
#define MIN_ACCEL_INTERVAL_MS           10
#define MAX_FILTER_DELTA_TIME_MS        1000
#define NEAR_ZERO_MAGNITUDE             1.0f    // m/s^2
#define FLAT_ANGLE                      70
#define FLAT_TIME                       800     // 800ms

#define ACCELERATION_TOLERANCE          4.0f
#define STANDARD_GRAVITY                9.8f
#define MIN_ACCELERATION_MAGNITUDE      (STANDARD_GRAVITY - ACCELERATION_TOLERANCE)
#define MAX_ACCELERATION_MAGNITUDE      (STANDARD_GRAVITY + ACCELERATION_TOLERANCE)

#define TILT_REFERENCE_PERIOD           1800000 // 30 min
#define TILT_REFERENCE_BACKOFF          300000   // 5 min

#define SWING_AWAY_ANGLE_DELTA          20
#define SWING_TIME                      300     // 300 ms

#define MAX_TILT                        70
#define TILT_OVERHEAD_ENTER             -80     // MTK: origin(-40)
#define TILT_OVERHEAD_EXIT              -70     // MTK: origin(-15)
#define ADJACENT_ORIENTATION_ANGLE_GAP  35      // MTK: origin(45)


#define PROPOSAL_SETTLE_TIME                            40      // 40 ms
#define PROPOSAL_MIN_TIME_SINCE_FLAT_ENDED              500     // 500 ms
#define PROPOSAL_MIN_TIME_SINCE_SWING_ENDED             300     // 300 ms
#define PROPOSAL_MIN_TIME_SINCE_ACCELERATION_ENDED      500     // 500 ms



static int8_t tilt_tolerance[4][2] = {
    /* ROTATION_0    { -70, 70 },*/{ -70, 70 },
    /* ROTATION_90   { -65, 65 },*/{-65, 55},
    /* ROTATION_180  { -60, 60 },*/{ -60, 60 },
    /* ROTATION_270  { -65, 65 }*/{-65, 55}
};


void clear_predicted_rotation()
{
    devorient_algo_state *state = &g_devorient_state;

    state->predicted_rotation = -1;
    state->predicted_rotation_time = 0;
}

void clearTiltHistory()
{
    devorient_algo_state *state = &g_devorient_state;

    state->tilt_history_time[0] = 0;
    state->tilt_history_index = 1;
    state->tilt_reference_time = 0;
}

#ifdef OPLUS_FEATURE_DEVORIENT_SWING
void clearxyzHistory(void)
{
    devorient_algo_state *state = &g_devorient_state;
    state->history_index = 0;
    memset(state->history_x, 0 ,sizeof(state->history_x));
    memset(state->history_y, 0 ,sizeof(state->history_y));
    memset(state->history_z, 0 ,sizeof(state->history_z));
}

static float variance(float *pdata, int32_t num, float *pavg)
{
    float aver,s;
    float sum=0,e=0;
    int i;

    for (i=0; i<num; i++)
    {
        sum += pdata[i];
    }

    aver = sum/num;
    *pavg = aver;

    for(i=0;i<num;i++)
    {
        e += (pdata[i]-aver)*(pdata[i]-aver);
    }

    e/=num;
    s=sqrt(e);
    return s;
}
//STEADY_ADJUST_NUM
static bool is_device_swingwing(void)
{
    devorient_algo_state *state = &g_devorient_state;
    bool x_swinging = false;
    bool y_swinging = false;
    bool z_swinging = false;
    bool flag = false;
    float var[3] = {0.0f};
    float pavg =0;
    float *data_x = state->history_x;
    float *data_y = state->history_y;
    float *data_z = state->history_z;

    /*******************x*******************/
    var[0] = variance(data_x, STEADY_ADJUST_NUM, &pavg);
    if (var[0] > X_SWINGING_VARIANCE_THRESHOLD)
    {
        x_swinging = true;
    }

    /*******************y*******************/

    var[1] = variance(data_y, STEADY_ADJUST_NUM, &pavg);

    if (var[1] > Y_SWINGING_VARIANCE_THRESHOLD)
    {
        //SNS_SMGR_PRINTF0(ERROR, "data is steady");
        y_swinging = true;
    }

    /*******************z*******************/
    var[2] = variance(data_z, STEADY_ADJUST_NUM, &pavg);

    if (var[2] > Z_SWINGING_VARIANCE_THRESHOLD)
    {
        //SNS_SMGR_PRINTF0(ERROR, "data is steady");
        z_swinging =  true;
    }

    if (x_swinging || y_swinging || z_swinging)
        flag = true;
    else
        flag = false;

    //DEVICE_ORIENT_INST_LOG(ERROR, this, "x_swinging :
    // %d,y_swinging : %d,z_swinging : %d",x_swinging,y_swinging,z_swinging);
    return flag;
}

void add_accdata_to_history_xyz(devorient_sensor_data *input) {
    devorient_algo_state *state = &g_devorient_state;
    if(state->history_index  >= STEADY_ADJUST_NUM) {
        state->history_index = 0;
    }
    state->history_x[state->history_index] = input->x;
    state->history_y[state->history_index] = input->y;
    state->history_z[state->history_index] = input->z;
    state->history_index++;
}
#endif



static void report_event_handle(devorient_algo_state *state)
{
    static uint16_t report_count = 0;

    report_count ++;
    report_count %= 0xFFFF;   // just count to check slpi and sensor hal

    DEVORIENT_LOG_INFO("Report event, status = %d,report_count = %d", state->proposed_rotation, report_count);

    state->sensor_ops->report(state->proposed_rotation, report_count);
}

static bool is_accelerating(float magnitude)
{
    return ((magnitude < MIN_ACCELERATION_MAGNITUDE)
            || (magnitude > MAX_ACCELERATION_MAGNITUDE));
}

void reset_status()
{
    devorient_algo_state *state = &g_devorient_state;

    state->last_filtered_data.timestamp = 0;
    state->need_refilter = true;
    state->proposed_rotation = -1;

    state->flat_time = 0;
    state->flat = false;

    state->swinging_time = 0;
    state->swinging = false;

    state->accelerating_time = 0;
    state->accelerating = false;

    state->overhead = false;

    clear_predicted_rotation();
    clearTiltHistory();
}

// ms
static void add_tilt_history_entry(uint64_t now, int8_t tilt)
{
    devorient_algo_state *state = &g_devorient_state;

    uint64_t old_reference_time_ms, delta_ms;
    size_t i;
    int index;

    if (state->tilt_reference_time == 0) {
        // set reference_time after reset()

        state->tilt_reference_time = now - 1;
    } else if ((now - state->tilt_reference_time) > TILT_REFERENCE_PERIOD ) {
        // uint32_t tilt_history_time[] is good up to 71 min (2^32 * 1e-6 sec).
        // proactively shift reference_time every 30 min,
        // all history entries are within 5 min interval (15Hz x 200 samples)

        old_reference_time_ms = state->tilt_reference_time;
        state->tilt_reference_time = now - TILT_REFERENCE_BACKOFF;

        delta_ms = state->tilt_reference_time - old_reference_time_ms;

        for (i = 0; i < TILT_HISTORY_SIZE; ++i) {
            state->tilt_history_time[i] = (state->tilt_history_time[i] > delta_ms)
                ? (state->tilt_history_time[i] - delta_ms) : 0;
        }
    }

    index = state->tilt_history_index;
    state->tilt_history[index] = tilt;
    state->tilt_history_time[index] = now - state->tilt_reference_time;

    index = ((index + 1) == TILT_HISTORY_SIZE) ? 0 : (index + 1);
    state->tilt_history_index = index;
    state->tilt_history_time[index] = 0;
}

static int next_tilt_history_index(int index)
{
    devorient_algo_state *state = &g_devorient_state;
    int next = (index == 0) ? (TILT_HISTORY_SIZE - 1) : (index - 1);
    return ((state->tilt_history_time[next] != 0) ? next : -1);
}

static bool is_flat(uint64_t now)
{
    devorient_algo_state *state = &g_devorient_state;

    int i = state->tilt_history_index;

    for (; (i = next_tilt_history_index(i)) >= 0;) {
        // MTK: consider overhead
        // if (mTask.tilt_history[i] < FLAT_ANGLE) {
        if (state->tilt_history[i] < FLAT_ANGLE && state->tilt_history[i] > -FLAT_ANGLE) {
            break;
        }

        if (state->tilt_reference_time + state->tilt_history_time[i] + FLAT_TIME <= now) {
            // Tilt has remained greater than FLAT_ANGLE for FLAT_TIME.
            return true;
        }
    }

    return false;
}

// 200ms tilt change  > 20 isswing
static bool is_swinging(uint64_t now, int8_t tilt)
{
    devorient_algo_state *state = &g_devorient_state;

    int i = state->tilt_history_index;

    for (; (i = next_tilt_history_index(i)) >= 0;) {
        if (state->tilt_reference_time + state->tilt_history_time[i] + SWING_TIME
            < now) {
            break;
        }

        if (state->tilt_history[i] + SWING_AWAY_ANGLE_DELTA <= tilt) {
            // Tilted away by SWING_AWAY_ANGLE_DELTA within SWING_TIME.
            return true;
        }
    }

    return false;
}

static void update_predicted_rotation(uint64_t now, int rotation)
{
    devorient_algo_state *state = &g_devorient_state;

    if (state->predicted_rotation != rotation) {
        state->predicted_rotation = rotation;
        state->predicted_rotation_time = now;
        DEVORIENT_LOG_DEBUG("update_predicted_rotation rotation %d time = %llu", state->predicted_rotation, state->predicted_rotation_time);
    }
}

static bool is_tilt_angle_acceptable(int rotation, int8_t tilt_angle)
{
    return ((tilt_angle >= tilt_tolerance[rotation][0]) && (tilt_angle <= tilt_tolerance[rotation][1]));
}

static bool is_orientation_angle_acceptable(int current_rotation, int rotation,
    int orientation_angle)
{
    // If there is no current rotation, then there is no gap.
    // The gap is used only to introduce hysteresis among advertised orientation
    // changes to avoid flapping.
    int lower_bound, upper_bound;

    if (current_rotation >= 0) {
        // If the specified rotation is the same or is counter-clockwise
        // adjacent to the current rotation, then we set a lower bound on the
        // orientation angle.
        // For example, if currentRotation is ROTATION_0 and proposed is
        // ROTATION_90, then we want to check orientationAngle > 45 + GAP / 2.
        if ((rotation == current_rotation) || (rotation == (current_rotation + 1) % 4)) {
            lower_bound = rotation * 90 - 45 + ADJACENT_ORIENTATION_ANGLE_GAP / 2;

            if (rotation == 0) {
                if ((orientation_angle >= 315) && (orientation_angle < lower_bound + 360)) {
                    return false;
                }
            } else {
                if (orientation_angle < lower_bound) {
                    return false;
                }
            }
        }

        // If the specified rotation is the same or is clockwise adjacent,
        // then we set an upper bound on the orientation angle.
        // For example, if currentRotation is ROTATION_0 and rotation is
        // ROTATION_270, then we want to check orientationAngle < 315 - GAP / 2.
        if ((rotation == current_rotation) || (rotation == (current_rotation + 3) % 4)) {
            upper_bound = rotation * 90 + 45 - ADJACENT_ORIENTATION_ANGLE_GAP / 2;

            if (rotation == 0) {
                if ((orientation_angle <= 45) && (orientation_angle > upper_bound)) {
                    return false;
                }
            } else {
                if (orientation_angle > upper_bound) {
                    return false;
                }
            }
        }
    }

    return true;
}

static bool is_predicted_rotation_acceptable(uint64_t now)
{
    devorient_algo_state *state = &g_devorient_state;

    // The predicted rotation must have settled long enough.
    if (now < state->predicted_rotation_time + PROPOSAL_SETTLE_TIME) {
        DEVORIENT_LOG_DEBUG("wait the status hold for 40ms");
        return false;
    }

    // The last flat state (time since picked up) must have been sufficiently
    // long ago.
    /*if (now < state->flat_time + PROPOSAL_MIN_TIME_SINCE_FLAT_ENDED) {
        DEVORIENT_LOG_0("just flat");
        return false;
    }*/

    // The last swing state (time since last movement to put down) must have
    // been sufficiently long ago.
    if (now < state->swinging_time + PROPOSAL_MIN_TIME_SINCE_SWING_ENDED) {
        DEVORIENT_LOG_DEBUG("just swing");
        return false;
    }

    // The last acceleration state must have been sufficiently long ago.
    if (now < state->accelerating_time
        + PROPOSAL_MIN_TIME_SINCE_ACCELERATION_ENDED) {
        DEVORIENT_LOG_DEBUG("just accelera");
        return false;
    }

    #ifdef OPLUS_FEATURE_DEVORIENT_SWING
    if(is_device_swingwing()) {
        DEVORIENT_LOG_DEBUG("device is swingwing");
        return false;
    }
    #endif

    // Looks good!
    return true;
}


bool process_acc_sample(devorient_sensor_data *input)
{
    devorient_algo_state *state = &g_devorient_state;
    int64_t delta_ms;
    uint64_t now_ms;

    float alpha;
    float x, y, z, magnitude;
    int tilt_tmp;
    int8_t tilt_angle;
    int orientation_angle, nearest_rotation;
    int8_t old_proposed_rotation = -1;
    int8_t proposed_rotation = -1;
    bool change_detected = 0;
    bool skip_sample = true;

    //check timestamp and get the delta
    now_ms = state->sensor_ops->get_time_ms(input->timestamp);

    if (state->need_refilter) {
        state->last_filtered_data.timestamp  = now_ms;
        state->last_filtered_data.x = input->x;
        state->last_filtered_data.y = input->y;
        state->last_filtered_data.z = input->z;
        state->need_refilter = false;
        skip_sample = false;
    }

    delta_ms = now_ms - state->last_filtered_data.timestamp;

    if( delta_ms <= 0 && skip_sample) {
        DEVORIENT_LOG_DEBUG("acc data timestamp now < last");
        return false;
    } else if (delta_ms > MAX_FILTER_DELTA_TIME_MS) {
        DEVORIENT_LOG_DEBUG("acc data timestamp now %d > last %d + 1s", (uint32_t)now_ms, (int32_t)delta_ms);
        reset_status();
        return false;
    } else if (delta_ms < MIN_ACCEL_INTERVAL_MS && skip_sample) {
        return false;
    }

    alpha = ((float)delta_ms) / (float)(FILTER_TIME_CONSTANT_MS + delta_ms);

    x = alpha * (input->x - state->last_filtered_data.x) + state->last_filtered_data.x;
    y = alpha * (input->y - state->last_filtered_data.y) + state->last_filtered_data.y;
    z = alpha * (input->z - state->last_filtered_data.z) + state->last_filtered_data.z;

    state->last_filtered_data.x = x;
    state->last_filtered_data.y = y;
    state->last_filtered_data.z = z;
    state->last_filtered_data.timestamp = now_ms;

    DEVORIENT_LOG_INFO("x = %f y = %f z %f alpha = %f", x * 1000, y * 1000, z * 1000, alpha);

    magnitude = sqrtf(x * x + y * y + z * z);

    if(magnitude < NEAR_ZERO_MAGNITUDE) {
        DEVORIENT_LOG_DEBUG("magnitude < 1");
        clear_predicted_rotation();
    } else {
        //if linner acc > 4 set it to untrusted
        if (is_accelerating(magnitude)) {
            state->accelerating = true;
            state->accelerating_time = now_ms;
            DEVORIENT_LOG_DEBUG(" isAcceletating time = %llu magnitude = %f", state->accelerating_time, magnitude);
        } else {
            state->accelerating = false;
        }

        // Calculate the tilt angle.
        // This is the angle between the up vector and the x-y plane
        // (the plane of the screen) in a range of [-90, 90] degrees.
        //  -90 degrees: screen horizontal and facing the ground (overhead)
        //    0 degrees: screen vertical
        //   90 degrees: screen horizontal and facing the sky (on table)
        tilt_tmp = (int)(asinf(z / magnitude) * RADIANS_TO_DEGREES);
        tilt_tmp = (tilt_tmp > 127) ? 127 : tilt_tmp;
        tilt_tmp = (tilt_tmp < -128) ? -128 : tilt_tmp;
        tilt_angle = tilt_tmp;
        DEVORIENT_LOG_INFO("tilt_angle =  %d", tilt_angle);
        add_tilt_history_entry(now_ms, tilt_angle);

        if (is_flat(now_ms)) {
            state->flat_time = now_ms;
            state->flat = true;
            DEVORIENT_LOG_DEBUG(" is_flat time = %llu now_ms = %llu", state->flat_time, now_ms);
        } else {
            state->flat = false;
        }

        if (is_swinging(now_ms, tilt_angle)) {
            state->swinging_time = now_ms;
            state->swinging = true;
            DEVORIENT_LOG_DEBUG(" is_swinging time = %llu", state->swinging_time);
        } else {
            state->swinging = false;
        }

        // If the tilt angle is too close to horizontal then we cannot
        // determine the orientation angle of the screen.
        if (tilt_angle <= TILT_OVERHEAD_ENTER) {
            state->overhead = true;
            DEVORIENT_LOG_DEBUG(" overhead time = %llu", now_ms);
        } else if (tilt_angle >= TILT_OVERHEAD_EXIT) {
            state->overhead = false;
        }

        if (now_ms < (state->flat_time + PROPOSAL_MIN_TIME_SINCE_FLAT_ENDED)) {
            update_predicted_rotation(now_ms, 4);
        } else {
            if (state->overhead) {
                clear_predicted_rotation();
            } else if ((tilt_angle > MAX_TILT) || (tilt_angle < -MAX_TILT)) {
                clear_predicted_rotation();
            } else {
                // Calculate the orientation angle.
                // This is the angle between the x-y projection of the up
                // vector onto the +y-axis, increasing clockwise in a range
                // of [0, 360] degrees.
                orientation_angle = (int)(-atan2f(-x, y) * RADIANS_TO_DEGREES);

                if (orientation_angle < 0) {
                    // atan2 returns [-180, 180]; normalize to [0, 360]
                    orientation_angle += 360;
                }

                // Find the nearest rotation.
                nearest_rotation = (orientation_angle + 45) / 90;

                if (nearest_rotation == 4) {
                    nearest_rotation = 0;
                }

                DEVORIENT_LOG_DEBUG("nearest_rotation = %d orientation_angle = %d tilt_angle %d", nearest_rotation, orientation_angle, tilt_angle);

                // Determine the predicted orientation.
                if (is_tilt_angle_acceptable(nearest_rotation, tilt_angle)
                    && is_orientation_angle_acceptable(state->current_rotation,
                        nearest_rotation,
                        orientation_angle)) {
                    update_predicted_rotation(now_ms, nearest_rotation);
                } else {
                    clear_predicted_rotation();
                }
            }
        }

    }

    // Determine new proposed rotation.
    old_proposed_rotation = state->proposed_rotation;

    if ((state->predicted_rotation < 0)
        || is_predicted_rotation_acceptable(now_ms)) {

        state->proposed_rotation = state->predicted_rotation;
    }

    proposed_rotation = state->proposed_rotation;

    DEVORIENT_LOG_DEBUG("old_proposed_rotation = %d proposed_rotation = %d prevalid_rotation %d", old_proposed_rotation, proposed_rotation, state->prev_valid_rotation);

    if ((proposed_rotation != old_proposed_rotation)
        && (proposed_rotation >= 0)) {
        state->current_rotation = proposed_rotation;

        change_detected = (proposed_rotation != state->prev_valid_rotation);
        state->prev_valid_rotation = proposed_rotation;

        if (change_detected) {
            return true;
        }
    }

    return false;
}

void devorient_algo_check_sensor_data(devorient_sensor_data *input)
{
    devorient_algo_state *state = &g_devorient_state;
    bool rotation_changed;

    switch (input->type) {
    case ACC_TYPE:
        #ifdef OPLUS_FEATURE_DEVORIENT_SWING
        add_accdata_to_history_xyz(input);
        #endif

        rotation_changed = process_acc_sample(input);

        if(rotation_changed) {
            report_event_handle(state);
        }

        break;

    case AMD_TYPE:
#ifdef OPLUS_FEATURE_DEVORIENT_USE_AMD
        if (input->x == AMD_STATIONARY) {
            state->sensor_ops->sensor_change_state(ACC_TYPE, false);
        } else {
            state->sensor_ops->sensor_change_state(ACC_TYPE, true);
        }

#endif
        DEVORIENT_LOG_INFO("amd_event = %d", (uint32_t)input->x);

    default:
        break;
    }
}

void devorient_algo_reset()
{
    devorient_algo_state *state = &g_devorient_state;

    if (g_devorient_state.algo_state == ALGO_RESET) {
        //renotify and return
        report_event_handle(state);
        return;
    }

    g_devorient_state.algo_state = ALGO_RESET;
    state->current_rotation = -1;
    state->prev_valid_rotation = -1;

    reset_status();
    clear_predicted_rotation();
    clearTiltHistory();
    #ifdef OPLUS_FEATURE_DEVORIENT_SWING
    clearxyzHistory();
    #endif

    state->first_init = true;
    state->acc_sampleRate = 50;
    state->sensor_ops->sensor_change_state(ACC_TYPE, true);
    state->sensor_ops->sensor_change_state(AMD_TYPE, true);
}
void devorient_algo_close()
{
    devorient_algo_state *state = &g_devorient_state;

    state->algo_state = ALGO_CLOSE;

    state->sensor_ops->sensor_change_state(ACC_TYPE, false);
    state->sensor_ops->sensor_change_state(AMD_TYPE, false);

    DEVORIENT_LOG_DEBUG("algo_close.");
}
void devorient_algo_register(devorient_algo_state **state)
{
    DEVORIENT_LOG_DEBUG("algo register.");

    memset(&g_devorient_state, 0, sizeof(devorient_algo_state));

    g_devorient_state.algo_state = ALGO_REGISTER;

    *state = &g_devorient_state;
}
