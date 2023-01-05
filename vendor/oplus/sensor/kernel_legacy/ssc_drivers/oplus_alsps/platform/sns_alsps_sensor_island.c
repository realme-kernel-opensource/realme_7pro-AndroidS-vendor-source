/******************************************************************
** Copyright (C), 2004-2020 OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - psensor_algo.c
** Description: Source file for oplus alsps new arch.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version>    <date>        <author>              <desc>
*******************************************************************/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_diag_service.h"
#include "sns_attribute_util.h"
#include "sns_sync_com_port_service.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_printf.h"
#include "sns_physical_sensor_test.pb.h"

#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"
#include "oplus_alsps.h"
#include <math.h>


sns_sensor_uid const* alsps_als_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = ALSPS_ALS_SUID;

    return &sensor_uid;
}

sns_sensor_uid const* alsps_ps_get_sensor_uid(sns_sensor const *const this)
{
    UNUSED_VAR(this);
    static const sns_sensor_uid sensor_uid = ALSPS_PS_SUID;

    return &sensor_uid;
}

sns_sensor_instance* alsps_sensor_util_find_instance_match (struct sns_sensor *this)
{
    sns_sensor_instance *instance;
    alsps_instance_state *inst_state = NULL;
    alsps_state *state = (alsps_state*)this->state->state;

    for (instance = this->cb->get_sensor_instance(this, true);
        NULL != instance;
        instance = this->cb->get_sensor_instance(this, false)) {
        inst_state = (alsps_instance_state*)instance->state->state;

        if (inst_state->rgt_sensor_type == state->sensor) {
            return instance;
        }
    }

    return NULL;
}

sns_sensor_instance * alsps_get_instance(struct sns_sensor *this,
    struct sns_request const *request)
{
    UNUSED_VAR(request);
    alsps_state *state = (alsps_state*)this->state->state;
    sns_sensor_instance *instance = NULL;

    if (state->is_unit_device) {
        instance = sns_sensor_util_get_shared_instance(this);
    } else {
        //try to find the instance before
        instance = alsps_sensor_util_find_instance_match(this);
    }

    return instance;
}

sns_rc alsps_sensor_notify_event(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;
    sns_sensor_event *event;

    OPLUS_ALS_PS_LOG("ALSPS alsps_sensor_notify_event 0x%x", this);

    if (state->fw_stream) {
        if (sns_sensor_uid_compare(&state->irq_suid, &((sns_sensor_uid) {{0}}))
            || sns_sensor_uid_compare(&state->timer_suid, &((sns_sensor_uid) {{0}}))
            || sns_sensor_uid_compare(&state->accel_suid, &((sns_sensor_uid) {{0}}))
            || sns_sensor_uid_compare(&state->resampler_suid, &((sns_sensor_uid) {{0}}))
            || sns_sensor_uid_compare(&state->reg_suid, &((sns_sensor_uid) {{0}}))) {
            /* All SUID events can be handled in normal mode. */
            state->island_service->api->sensor_island_exit(state->island_service, this);
            alsps_process_suid_events(this);

            if (!sns_sensor_uid_compare(&state->reg_suid, &((sns_sensor_uid) {{0}}))) {
                state->island_service->api->sensor_island_exit(state->island_service, this);

                if (!state->hw_is_present) {
                    alsps_request_registry(this);
                }
            }
        }

        /* Check if the SUID sensor stream can be removed. */
        if (!sns_sensor_uid_compare(&state->irq_suid, &((sns_sensor_uid) {{0}}))
            && !sns_sensor_uid_compare(&state->timer_suid, &((sns_sensor_uid) {{0}}))
            && !sns_sensor_uid_compare(&state->accel_suid, &((sns_sensor_uid) {{0}}))
            && !sns_sensor_uid_compare(&state->resampler_suid, &((sns_sensor_uid) {{0}}))
            && !sns_sensor_uid_compare(&state->reg_suid, &((sns_sensor_uid) {{0}}))) {
            sns_sensor_util_remove_sensor_stream(this, &state->fw_stream);
        }
    }

    /* Handle a Timer Sensor event */
    if (NULL != state->timer_stream) {
        OPLUS_ALS_PS_LOG("start handle timer stream\n");
        event = state->timer_stream->api->peek_input(state->timer_stream);

        while (NULL != event) {
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);
            sns_timer_sensor_event timer_event;

            if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id) {
                if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                    if (ALSPS_POWER_RAIL_PENDING_INIT == state->power_rail_pend_state) {
                        /* Initial HW discovery is OK to run in normal mode. */
                        state->island_service->api->sensor_island_exit(state->island_service, this);
                        state->hw_is_present = alsps_discover_hw(this);

                        if (state->hw_is_present) {
                            alsps_publish_available(this);
                            alsps_publish_registry_attributes(this);

                            /** More clean up. */
                            sns_sensor_util_remove_sensor_stream(this, &state->reg_data_stream);
                            OPLUS_ALS_PS_LOG("%d HW present \n", state->sensor);
                        } else {
                            #ifdef OPLUS_FEATURE_SENSOR_FB
                            struct fb_event fb_event;
                            memset(&fb_event, 0, sizeof(struct fb_event));
                            fb_event.event_id = PS_INIT_FAIL_ID;
                            oplus_add_fd_event(&fb_event);
                            #endif
                            rv = SNS_RC_INVALID_LIBRARY_STATE;
                            OPLUS_ALS_PS_LOG("%d HW absent \n", state->sensor);
                        }

                        state->power_rail_pend_state = ALSPS_POWER_RAIL_PENDING_NONE;
                    } else if (ALSPS_POWER_RAIL_PENDING_SET_CLIENT_REQ == state->power_rail_pend_state) {
                        sns_sensor_instance *instance = alsps_get_instance(this, NULL);

                        if (NULL != instance) {
                            alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;

                            inst_state->instance_is_ready_to_configure = true;
                            alsps_reval_instance_config(this, instance, state->sensor);

                            OPLUS_ALS_PS_LOG("ALSPS notify event ALSPS_POWER_RAIL_PENDING_SET_CLIENT_REQ\n");
                        }

                        state->power_rail_pend_state = ALSPS_POWER_RAIL_PENDING_NONE;
                    } else if (ALSPS_POWER_RAIL_PENDING_OFF == state->power_rail_pend_state) {
                        sns_sensor_instance *instance = alsps_get_instance(this, NULL);

                        if (NULL == instance) {
                            if (SNS_RAIL_OFF != state->rail_config.rail_vote) {
                                state->rail_config.rail_vote = SNS_RAIL_OFF;
                                state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                    this,
                                    &state->rail_config,
                                    NULL);
                                OPLUS_ALS_PS_LOG("ALSPS notify event instace null turn of rail\n");
                            }
                        }

                        state->power_rail_pend_state = ALSPS_POWER_RAIL_PENDING_NONE;
                    }
                } else {
                    OPLUS_ALS_PS_LOG("ALSPS pb_decode error");
                }
            }

            event = state->timer_stream->api->get_next_input(state->timer_stream);
        }

        /* Free up timer stream if not needed anymore */
        if (ALSPS_POWER_RAIL_PENDING_NONE == state->power_rail_pend_state) {
            sns_sensor_util_remove_sensor_stream(this, &state->timer_stream);
        }
    }

    if (NULL != state->reg_data_stream) {
        event = state->reg_data_stream->api->peek_input(state->reg_data_stream);

        while (NULL != event) {
            /* All registry events can be handled in normal mode. */
            state->island_service->api->sensor_island_exit(state->island_service, this);
            alsps_sensor_process_registry_event(this, event);

            event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
        }
    }

    if (!sns_sensor_uid_compare(&state->timer_suid, &((sns_sensor_uid) {{0}})) &&
        state->registry_pf_cfg_received && !state->start_hw_detect) {
        /* Initial HW detection sequence is OK to run in normal mode. */
        state->island_service->api->sensor_island_exit(state->island_service, this);
        alsps_start_hw_detect_sequence(this);
    }
    return rv;
}
//0 means clientless  1 means als avaliable 2 means ps avaliable 3 means als+ps avaliable
static int is_instance_clientless(sns_sensor *this, sns_sensor_instance *instance)
{
    alsps_state *state = (alsps_state*)this->state->state;
    int result = 0;

    if (state->is_unit_device) {
        if (NULL != instance->cb->get_client_request(instance, &(sns_sensor_uid)ALSPS_ALS_SUID, true)) {
            result += ALS;
        }

        if (NULL != instance->cb->get_client_request(instance, &(sns_sensor_uid)ALSPS_PS_SUID, true)) {
            result += PS;
        }
    } else {
        if (state->sensor == ALS) {
            if (NULL != instance->cb->get_client_request(instance,
                    &(sns_sensor_uid)ALSPS_ALS_SUID, true)) {
                result += ALS;
            }
        } else if (state->sensor == PS) {
            if (NULL != instance->cb->get_client_request(instance,
                    &(sns_sensor_uid)ALSPS_PS_SUID, true)) {
                result += PS;
            }
        }
    }

    return result;
}

static bool alsps_get_decoded_request(sns_sensor const *this, sns_request const *in_request,
    sns_std_request *decoded_request,
    sns_std_sensor_config *decoded_payload)
{
    UNUSED_VAR(this);
    pb_istream_t stream;

    pb_simple_cb_arg arg = {
        .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields
    };

    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb, .arg = &arg
    };

    stream = pb_istream_from_buffer(in_request->request,
            in_request->request_len);

    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        OPLUS_ALS_PS_LOG("decode error");
        return false;
    }

    return true;
}

static void alsps_get_config(sns_sensor *this,
    sns_sensor_instance *instance,
    alsps_sensor_type sensor_type,
    float *chosen_sample_rate,
    float *chosen_report_rate)
{
    sns_sensor_uid suid;
    sns_request const *request;
    alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;

    if (sensor_type == ALS) {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ALSPS_ALS_SUID), sizeof(sns_sensor_uid));
        inst_state->als_info.is_factory_mode = false;
        inst_state->als_info.enable_count = 0;
    } else if (sensor_type == PS) {
        sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ALSPS_PS_SUID), sizeof(sns_sensor_uid));
        inst_state->ps_info.enable_count = 0;
    }

    *chosen_report_rate = 0.0f;
    *chosen_sample_rate = 0.0f;

    for (request = instance->cb->get_client_request(instance, &suid, true);
        NULL != request;
        request = instance->cb->get_client_request(instance, &suid, false)) {
        sns_std_request decoded_request;
        sns_std_sensor_config decoded_payload = {0};

        if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG ||
            request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG) {
            if (alsps_get_decoded_request(this, request, &decoded_request, &decoded_payload)) {
                float report_rate = 0.0f;

                if (fabs(*chosen_sample_rate - 0) < 0.01) {
                    *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                            decoded_payload.sample_rate);
                } else {
                    *chosen_sample_rate = SNS_MIN(*chosen_sample_rate,
                            decoded_payload.sample_rate);
                }

                if (decoded_request.has_batching
                    &&
                    decoded_request.batching.batch_period > 0) {
                    report_rate = (1000000.0f / (float)decoded_request.batching.batch_period);
                } else {
                    report_rate = decoded_payload.sample_rate;
                }

                if (fabs(report_rate - 10) <= 0.001 && (sensor_type == ALS)) {
                    inst_state->als_info.is_factory_mode = true;
                }

                *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                        report_rate);
            }
        }

        if (request->message_id != SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG
                && request->message_id != SNS_OPLUS_IFACE_MSGID_LIGHT_LEVEL) {
            if (sensor_type == ALS) {
                inst_state->als_info.enable_count++;
            }

            if (sensor_type == PS) {
                inst_state->ps_info.enable_count++;
            }
        }
    }

    if (sensor_type == ALS) {
        if (inst_state->als_info.enable_count > 0) {
            inst_state->als_info.is_als_enable = true;
        } else {
            inst_state->als_info.is_als_enable = false;
        }
    } else if (sensor_type == PS) {
        if (inst_state->ps_info.enable_count > 0) {
            inst_state->ps_info.is_ps_enable = true;
        } else {
            inst_state->ps_info.is_ps_enable = false;
        }
    }

    OPLUS_ALS_PS_LOG("alsps_get_config::SensorType = %d, sample rate = %d, report rate = %d,als_enable = %d, ps_enable = %d is_factory = %d",
        sensor_type, (int32_t)*chosen_sample_rate,
        (int32_t)*chosen_report_rate, inst_state->als_info.is_als_enable,
        inst_state->ps_info.is_ps_enable, inst_state->als_info.is_factory_mode);
}

static void alsps_set_inst_config(sns_sensor *this,
    sns_sensor_instance *instance,
    float chosen_report_rate,
    float chosen_sample_rate,
    alsps_sensor_type sensor_type,
    uint32_t message_id)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_alsps_req new_client_config;
    sns_request config;
    struct vendor_node *s_list = NULL;
    new_client_config.desired_report_rate = chosen_report_rate;
    new_client_config.desired_sample_rate = chosen_sample_rate;
    new_client_config.sensor_type = sensor_type;
    config.message_id = message_id;
    config.request_len = sizeof(sns_alsps_req);
    config.request = &new_client_config;

    s_list = alsps_get_s_list();

    if (state->sensor == ALS &&
        state->vendor_id >= 0 &&
        s_list[state->vendor_id].als_ops) {
        alsps_register_als(instance, s_list[state->vendor_id].als_ops);
    } else if (state->sensor == PS &&
        state->vendor_id >= 0 &&
        s_list[state->vendor_id].ps_ops) {
        alsps_register_ps(instance, s_list[state->vendor_id].ps_ops);
    }

    this->instance_api->set_client_config(instance, &config);
}

void alsps_reval_instance_config(sns_sensor *this,
    sns_sensor_instance *instance,
    alsps_sensor_type sensor_type)
{
    float chosen_sample_rate = 0.0f;
    float chosen_report_rate = 0.0f;
    float sample_rate = 0.0f;
    float report_rate = 0.0f;
    uint32_t message_id = 0;
    alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;

    alsps_get_config(this,
        instance,
        ALS,
        &chosen_sample_rate,
        &chosen_report_rate);
    alsps_get_config(this,
        instance,
        PS,
        &sample_rate,
        &report_rate);

    chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
    chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

    if (inst_state->als_info.test_info.test_client_present || inst_state->ps_info.test_info.test_client_present) {
        message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
    } else {
        message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
    }

    alsps_set_inst_config(this,
        instance,
        chosen_report_rate,
        chosen_sample_rate,
        sensor_type,
        message_id);
}

static bool alsps_get_decoded_sensor_test_request(sns_sensor const *this,
    sns_request const *request,
    sns_std_request *decoded_request,
    sns_physical_sensor_test_config *test_config)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg = {
        .decoded_struct = test_config,
        .fields = sns_physical_sensor_test_config_fields
    };

    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb, .arg = &arg
    };

    stream = pb_istream_from_buffer(request->request,
            request->request_len);

    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        OPLUS_ALS_PS_LOG("decode error");
        return false;
    }

    return true;
}

static bool alsps_decode_sensor_test_info(sns_sensor *this,
    sns_sensor_instance *instance,
    struct sns_request const *new_request)
{
    sns_std_request decoded_request;
    sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
    alsps_state *state = (alsps_state*)this->state->state;
    alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;
    alsps_test_info *self_test_info;

    if (state->sensor == ALS) {
        self_test_info = &inst_state->als_info.test_info;
    } else if (state->sensor == PS) {
        self_test_info = &inst_state->ps_info.test_info;
    } else {
        return false;
    }

    if (alsps_get_decoded_sensor_test_request(this, new_request,
            &decoded_request, &test_config)) {
        self_test_info->test_type = test_config.test_type;
        self_test_info->test_client_present = true;
        return true;
    } else {
        return false;
    }
}
static bool alsps_get_decoded_light_level_request(sns_sensor const *this,
        sns_request const *request,
        sns_std_request *decoded_request,
        sns_light_level_data *data)
{
    UNUSED_VAR(this);
    pb_istream_t stream;
    pb_simple_cb_arg arg = {
        .decoded_struct = data,
        .fields = sns_light_level_data_fields
    };

    decoded_request->payload = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_simple_cb, .arg = &arg
    };

    stream = pb_istream_from_buffer(request->request,
            request->request_len);

    if (!pb_decode(&stream, sns_std_request_fields, decoded_request)) {
        OPLUS_ALS_PS_LOG("decode error");
        return false;
    }

    return true;


}
static bool alsps_get_light_level_info(sns_sensor *this,
        sns_sensor_instance *instance,
        struct sns_request const *new_request)
{
    sns_std_request decoded_request;
    sns_light_level_data data = sns_light_level_data_init_default;
    alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;
    int backlight_info;
    if (alsps_get_decoded_light_level_request(this, new_request, &decoded_request, &data)) {
        backlight_info = (int)data.level;
        inst_state->real_brightness = backlight_info & 0xffff;
        inst_state->dc_mode = backlight_info >> 16;
        if (false == inst_state->als_info.test_info.test_client_present) {
            alsps_set_brightness(inst_state, inst_state->real_brightness);
        }
        OPLUS_ALS_PS_LOG("get_light_level_info: backlight_info %d, brightness %d, dc %d",
                backlight_info, (int)inst_state->real_brightness, (int)inst_state->dc_mode);
        return true;
    } else {
        return false;
    }

}
static void alsps_turn_rails_off(sns_sensor *const this)
{
    sns_sensor *sensor;

    for (sensor = this->cb->get_library_sensor(this, true);
        NULL != sensor;
        sensor = this->cb->get_library_sensor(this, false)) {
        alsps_state *sensor_state = (alsps_state*)sensor->state->state;

        if (sensor_state->rail_config.rail_vote != SNS_RAIL_OFF) {
            sns_time timeout = sns_convert_ns_to_ticks(ALS_PS_RAIL_OFF_TIMEOUT_NS);
            alsps_start_power_rail_timer(sensor, timeout, ALSPS_POWER_RAIL_PENDING_OFF);
        }
    }
}

static void send_fifo_flush_done(sns_sensor_instance *const instance,
    sns_sensor_uid *suid)
{
    sns_service_manager *mgr = instance->cb->get_service_manager(instance);
    sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
    sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);

    if (NULL != event) {
        event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
        event->event_len = 0;
        event->timestamp = sns_get_system_time();
        e_service->api->publish_event(e_service, instance, event, suid);
    }
}
sns_sensor_instance* alsps_set_client_request(sns_sensor *const this,
    struct sns_request const *exist_request,
    struct sns_request const *new_request,
    bool remove)
{
    sns_sensor_instance *instance = alsps_get_instance(this, new_request);
    alsps_state *state = (alsps_state*)this->state->state;
    alsps_instance_state *inst_state = NULL;

    sns_time on_timestamp;
    sns_time delta;
    bool reval_config = false;

    if (instance) {
        inst_state = (alsps_instance_state*)instance->state->state;
    }

    if (remove) {
        if (NULL != instance) {
            instance->cb->remove_client_request(instance, exist_request);
            if (SNS_OPLUS_IFACE_MSGID_LIGHT_LEVEL != exist_request->message_id) {
                alsps_reval_instance_config(this, instance, state->sensor);
            }
            int result = is_instance_clientless(this, instance);

            if (inst_state->update_cal_registry) {
                OPLUS_ALS_PS_LOG("Update reg \n");
                /** Registry write can be handled in normal mode. */
                state->island_service->api->sensor_island_exit(state->island_service, this);
                alsps_update_sensor_state(this, instance);
                alsps_update_registry(this, instance, ALS);
                alsps_update_registry(this, instance, PS);
                inst_state->update_cal_registry = false;
                inst_state->update_dynamic_cal_registry = false;
            }

            if ((result & PS) == 0) {
                if (inst_state->update_dynamic_cal_registry) {
                    OPLUS_ALS_PS_LOG("Update reg \n");
                    state->island_service->api->sensor_island_exit(state->island_service, this);
                    alsps_update_sensor_state(this, instance);
                    alsps_update_registry(this, instance, PS);
                    inst_state->update_dynamic_cal_registry = false;
                }
            }

            if (result == 0) {
                this->cb->remove_instance(instance);
                alsps_turn_rails_off(this);
            }
        }
    } else {
        if (NULL == instance) {//off2idle
            state->rail_config.rail_vote = state->registry_rail_on_state;
            state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                this,
                &state->rail_config,
                &on_timestamp);
            delta = sns_get_system_time() - on_timestamp;

            // Use on_timestamp to determine correct Timer value.
            if (delta < sns_convert_ns_to_ticks(ALSPS_OFF_TO_IDLE_MS * 1000 * 1000)) {
                OPLUS_ALS_PS_LOG("start power rail timer \n");
                alsps_start_power_rail_timer(this,
                    sns_convert_ns_to_ticks(ALSPS_OFF_TO_IDLE_MS * 1000 * 1000) - delta,
                    ALSPS_POWER_RAIL_PENDING_SET_CLIENT_REQ);
            } else {
                // rail is already ON
                reval_config = true;
            }

            //create_instance
            instance = this->cb->create_instance(this, sizeof(alsps_instance_state));

            if (instance) {
                inst_state = (alsps_instance_state*)instance->state->state;

                if (reval_config) {
                    inst_state->instance_is_ready_to_configure = true;
                }
            }
        } else {
            inst_state = (alsps_instance_state*)instance->state->state;

            if (!inst_state->instance_is_ready_to_configure &&
                    SNS_OPLUS_IFACE_MSGID_LIGHT_LEVEL != new_request->message_id) {
                alsps_start_power_rail_timer(this,
                    sns_convert_ns_to_ticks(ALSPS_OFF_TO_IDLE_MS * 1000 * 1000),
                    ALSPS_POWER_RAIL_PENDING_SET_CLIENT_REQ);
            }

            if (NULL != exist_request && NULL != new_request &&
                new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ) {
                send_fifo_flush_done(instance, &state->my_suid);
                return instance;
            } else {
                reval_config = true;

                // An existing client is changing request
                if ((NULL != exist_request) && (NULL != new_request)) {
                    instance->cb->remove_client_request(instance, exist_request);
                }
                //  A new client sent new_request
                else if (NULL != new_request) {
                    if (state->sensor == PS) {
                        inst_state->ps_info.is_new_request = true;
                    }

                    if (state->sensor == ALS) {
                        inst_state->als_info.is_new_request = true;
                    }
                }
            }
        }
    }

    //Add the new request to instance
    if (NULL != instance) {
        alsps_instance_state *inst_state = (alsps_instance_state*)instance->state->state;

        if (NULL != new_request) {
            instance->cb->add_client_request(instance, new_request);

            if (new_request->message_id == SNS_CAL_MSGID_SNS_CAL_RESET) {
                OPLUS_ALS_PS_LOG("cal reset");
            }

            if (new_request->message_id ==
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG) {
                OPLUS_ALS_PS_LOG("test config \n");

                if (alsps_decode_sensor_test_info(this, instance, new_request)) {
                    OPLUS_ALS_PS_LOG("test config decode success \n");
                }
            } else if (SNS_OPLUS_IFACE_MSGID_LIGHT_LEVEL == new_request->message_id) {
                OPLUS_ALS_PS_LOG("lcd info config");
                alsps_get_light_level_info(this, instance, new_request);
            }
        }

        if (reval_config && inst_state->instance_is_ready_to_configure) {
            OPLUS_ALS_PS_LOG("reval_config");
            if (NULL != new_request && SNS_OPLUS_IFACE_MSGID_LIGHT_LEVEL == new_request->message_id) {
                return instance;
            }
            alsps_reval_instance_config(this, instance, state->sensor);
        }
    }

    return instance;

}

sns_sensor_api als_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &alsps_als_init,
    .deinit             = &alsps_als_deinit,
    .get_sensor_uid     = &alsps_als_get_sensor_uid,
    .notify_event        = &alsps_sensor_notify_event,
    .set_client_request = &alsps_set_client_request,
};

sns_sensor_api ps_sensor_api = {
    .struct_len         = sizeof(sns_sensor_api),
    .init               = &alsps_ps_init,
    .deinit             = &alsps_ps_deinit,
    .get_sensor_uid     = &alsps_ps_get_sensor_uid,
    .notify_event        = &alsps_sensor_notify_event,
    .set_client_request = &alsps_set_client_request,
};
