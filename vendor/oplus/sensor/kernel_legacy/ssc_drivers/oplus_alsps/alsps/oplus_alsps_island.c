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

#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"
#include "sns_cal.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "oplus_alsps.h"
#include "sns_time.h"
#include "oplus_list.h"
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif
#include "sns_alsps_feature.h"

sns_sensor_instance *prox_mTask;
sns_sensor_instance *als_mTask;
int  normal_value = 1057;
int  ps_offset_cal = 0;
bool is_ps_far = false;
static bool need_boot_cali = true;
#define IRQ_EROOR_REST 10

static sns_sensor_instance *get_ps_instance()
{
    if (prox_mTask != NULL) {
        return prox_mTask;
    }

    if (als_mTask != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)als_mTask->state->state;

        if (state->is_unit_device) {
            return als_mTask;
        }
    }

    return NULL;
}
static void slit_prox_set_thrd(int th_L, int th_H)
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if ( (NULL != state->ps_info.ops) && (NULL != state->ps_info.ops->ps_set_thd) ) {
            state->ps_info.ops->ps_set_thd(state->scp_service, state->port_handle,
                th_H, th_L, state->ps_info.status);
            state->ps_info.thd_far = th_L;
            state->ps_info.thd_near = th_H;
        }
    }
}

static void alsps_ps_update_cali_params(alsps_instance_state *state, int mode, int value)
{
    switch (mode) {
    case PS_CAIL_FACTORY_UNCOVERED:
        state->ps_fac_cal_data[0] = value;
        state->ps_cal_version[0]++;
        state->ps_info.offset = value;
        state->ps_fac_cal_data[5] = state->ps_fac_cal_data[0];
        state->ps_cal_version[5]++;
        break;

    case PS_CAIL_FACTORY_3CM:
        state->ps_fac_cal_data[1] = state->ps_info.cali_data;
        state->ps_fac_cal_data[2] = value;
        state->ps_cal_version[1]++;
        state->ps_cal_version[2]++;
        break;

    case PS_CAIL_DYNAMIC_PARAMS:
        state->ps_info.offset = value;
        state->ps_fac_cal_data[5] = value;
        state->ps_cal_version[5]++;
        break;
    }
}

static void alsps_ps_update_offset_calibration_params(alsps_instance_state *state, int value)
{
    if (value >= 0xFF00){
        state->ps_fac_cal_data[0] = (value - 0xFF00) - 0xFF;
    } else {
        state->ps_fac_cal_data[0] = value;
    }
    state->ps_cal_version[0]++;
    state->ps_info.offset = value;
    state->ps_fac_cal_data[5] = state->ps_fac_cal_data[0];
    state->ps_cal_version[5]++;
}

static void slit_prox_set_offset(int offset)
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if ( (NULL != state->ps_info.ops) && (NULL != state->ps_info.ops->set_offset) ) {
            state->ps_info.ops->set_offset(state->scp_service, state->port_handle, offset);
            alsps_ps_update_cali_params(state, PS_CAIL_DYNAMIC_PARAMS, offset);
            state->update_dynamic_cal_registry = true;
            OPLUS_ALS_PS_LOG("do offset do registry update fac_offset= %d  version = %d\n",
                (int)state->ps_fac_cal_data[5], state->ps_cal_version[5]);
        }
    }
}

static void slit_prox_get_sensor_data(struct psensor_data *psensor_data_info)
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        psensor_data_info->data = state->ps_info.last_ps;
        psensor_data_info->state = state->ps_info.status;
        psensor_data_info->offset = state->ps_info.offset;

        if (NULL != state->ps_info.ops) {
            if (NULL != state->ps_info.ops->get_ps_original_data) {
                state->ps_info.ops->get_ps_original_data(state->scp_service, state->port_handle,
                    &psensor_data_info->original_data);
            } else {
                psensor_data_info->original_data = 0;
            }

            if ((NULL != state->ps_info.ops) && (state->ps_info.ops->prox_need_ir_info != NULL)
                && (state->ps_info.ops->prox_need_ir_info())) {
                state->ps_info.ops->get_ir_data(state->scp_service,
                    state->port_handle, &psensor_data_info->ir_data);
            } else {
                psensor_data_info->ir_data = 0;
            }
        }
    }
}

static void slit_prox_get_acc_data(struct gsensor_data *accdata)
{
    sns_sensor_instance * inst = get_ps_instance();
    accdata->acc_x = 0;
    accdata->acc_y = 0;
    accdata->acc_z = 0;

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;
        accdata->acc_x = state->ps_info.last_acc_x;
        accdata->acc_y = state->ps_info.last_acc_y;
        accdata->acc_z = state->ps_info.last_acc_z;
    }
}

static bool slit_prox_need_ir_info()
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if ((NULL != state->ps_info.ops) && (NULL != state->ps_info.ops->prox_need_ir_info)) {
            return state->ps_info.ops->prox_need_ir_info();
        }
    }

    return false;
}

static bool is_ps_support_hardware_cali()
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if ((NULL != state->ps_info.ops) && (NULL != state->ps_info.ops->hardware_cali)) {
            return true;
        }
    }

    return false;
}

static void slit_prox_hard_cali()
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if ((NULL != state->ps_info.ops) && (NULL != state->ps_info.ops->hardware_cali)
            && (state->ps_info.in_hard_cali != 1)) {
            OPLUS_ALS_PS_LOG("do hard cali \n");
            state->ps_info.ops->hardware_cali(state->scp_service, state->port_handle);
            state->ps_info.in_hard_cali = 1;
            state->ps_info.hard_cali_time = sns_get_system_time();
        }
    }
}

bool ps_need_gesture()
{
    sns_sensor_instance * inst = get_ps_instance();

    if (inst != NULL) {
        alsps_instance_state *state = (alsps_instance_state*)inst->state->state;

        if (state->ps_type == PS_TYPE_Y) {
            OPLUS_ALS_PS_LOG("prox_need_gesture \n");
            return true;
        }
    }

    return false;
}

struct ps_algo_operation x_ops = {
    .set_offset = slit_prox_set_offset,
    .set_threshold = slit_prox_set_thrd,
    .get_ps_data = slit_prox_get_sensor_data,
    .get_gsensor_data = slit_prox_get_acc_data,
    .is_check_ir = slit_prox_need_ir_info,
    .is_support_hw_cali = is_ps_support_hardware_cali,
    .do_hw_cali = slit_prox_hard_cali,
};

struct dynamic_thrd_parameter default_dt_parameter = {
    .low_step = 300,
    .high_step = 400,
    .low_limit = 650,
    .high_limit = 1050,
    .dirty_low_step = 300,
    .dirty_high_step = 400,
    .ps_dirty_limit = 1800,
    .ps_ir_limit = 2000,
    .ps_adjust_min = 0,
    .ps_adjust_max = 1850,
    .sampling_count = 5,
    .step_max = 400,
    .step_min = 100,
    .step_div = 2000,
    .anti_shake_delta = 70,
};
struct dynamic_cali_parameter default_dc_parameter = {
    .dynamic_cali_max = 2000,
    .raw2offset_radio = 1000,
    .offset_max = 60000,
    .offset_range_min = 0,
    .offset_range_max = 65535,
    .force_cali_limit = 2000,
    .cali_jitter_limit = 20,
    .cal_offset_margin = 3000,//use to limit offset max
};
struct factory_cali_parameter default_fc_parameter = {
    .delta = 0,
    .offset = 0,
};



struct ps_algo default_slit_prox_cali = {
    .ops = &x_ops,
    .dt_parameter = &default_dt_parameter,
    .dc_parameter = &default_dc_parameter,
    .fc_parameter = &default_fc_parameter,
};

static struct stream_node s_head = {
    .stream_id = S_HEAD,
    .type = UNKNOWN,
    .data_stream = NULL,
};

static bool s_head_init = false;

struct vendor_node s_list[] =  {
    {STK3A5X, 0x46, 0x3E, &stk3a5x_als_ops, &stk3a5x_ps_ops, &default_slit_prox_cali, "stk33502"},
    {TCS3701, 0x39, 0x92, &tcs3701_als_ops, &tcs3701_ps_ops, &default_slit_prox_cali, "tcs3701"},
    {STK3A6X, 0X48, 0x3E, &stk3a6x_als_ops, &stk3a6x_ps_ops, &default_slit_prox_cali, "stk32600"},
    {MN78911, 0x41, 0x06, &mn78911_als_ops, &mn78911_ps_ops, &default_slit_prox_cali, "MN78911"}
};

struct vendor_node * alsps_get_s_list(void)
{
    return &s_list[0];
}

int alsps_register_stream(sns_sensor_instance *this, int stream_id,
    enum stream_type type)
{
    struct stream_node* pos = NULL;
    struct stream_node* new_s = NULL;
    bool find = false;

    if (!s_head_init) {
        s_head_init = true;
        INIT_LIST_HEAD(&s_head.list);
    }

    list_for_each_entry(pos, struct stream_node, &(s_head.list), list) {
        if (stream_id == pos->stream_id) {
            find = true;
            break;
        }
    }

    if (!find) {
        new_s = (struct stream_node*)ALSPS_ISLAND_MALLOC(sizeof(struct stream_node));

        if (!new_s) {
            OPLUS_ALS_PS_LOG("malloc fail \n");
            return -EINVAL;
        }

        new_s->stream_id = stream_id;
        new_s->type = type;
        new_s->instance = this;
        new_s->is_vilid = false;
        new_s->data_stream = NULL;

        list_add(&(new_s->list), &(s_head.list));
    } else { //update stream
        pos->stream_id = stream_id;
        pos->type = type;
        pos->instance = this;

        if (pos->data_stream) {
            pos->is_vilid = true;
        } else {
            pos->is_vilid = false;
        }
    }

    return 0;
}

struct stream_node* alsps_get_stream_node(int stream_id)
{
    struct stream_node* pos = NULL;
    struct stream_node* n = NULL;
    bool find = false;

    if (!s_head_init) {
        OPLUS_ALS_PS_LOG("s_head did not init \n");
        return NULL;
    }

    list_for_each_entry_safe(pos, n, struct stream_node, &(s_head.list), list) {
        if (stream_id == pos->stream_id) {
            find = true;
            break;
        }
    }

    if (find) {
        return pos;
    } else {
        return NULL;
    }
}

int alsps_set_stream_invalid(int stream_id)
{
    struct stream_node* pos = NULL;
    struct stream_node* n = NULL;

    if (!s_head_init) {
        OPLUS_ALS_PS_LOG("s_head did not init \n");
        return -EINVAL;
    }

    list_for_each_entry_safe(pos, n, struct stream_node, &(s_head.list), list) {
        if (stream_id == pos->stream_id) {
            pos->is_vilid = false;
            break;
        }
    }

    return 0;
}

int alsps_set_stream_valid(int stream_id)
{
    struct stream_node* pos = NULL;
    struct stream_node* n = NULL;

    if (!s_head_init) {
        OPLUS_ALS_PS_LOG("s_head did not init \n");
        return -EINVAL;
    }

    list_for_each_entry_safe(pos, n, struct stream_node, &(s_head.list), list) {
        if (stream_id == pos->stream_id) {
            pos->is_vilid = true;
            break;
        }
    }

    return 0;
}

int alsps_unregister_stream(int stream_id)
{
    struct stream_node* pos = NULL;
    struct stream_node* n = NULL;

    if (!s_head_init) {
        OPLUS_ALS_PS_LOG("s_head did not init \n");
        return -EINVAL;
    }

    list_for_each_entry_safe(pos, n, struct stream_node, &(s_head.list), list) {
        if (stream_id == pos->stream_id) {
            list_del(&(pos->list));
            sns_free(pos);
            break;
        }
    }

    return 0;
}

int alsps_register_als(sns_sensor_instance *const this, struct alsps_als_operations *ops)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (!state->als_info.ops) {
        if (ops) {
            state->als_info.ops = ops;
        }  else {
            OPLUS_ALS_PS_LOG("ops NULL \n");
            return -EINVAL;
        }
    } else {
        OPLUS_ALS_PS_LOG("als _ops has been register \n");
        return -EINVAL;
    }

    return 0;
}

int alsps_register_ps(sns_sensor_instance *const this, struct alsps_ps_operations *ops)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (!state->ps_info.ops) {
        if (ops) {
            state->ps_info.ops = ops;
        }  else {
            OPLUS_ALS_PS_LOG("ops NULL \n");
            return -EINVAL;
        }
    } else {
        OPLUS_ALS_PS_LOG("als _ops has been register \n");
        return -EINVAL;
    }

    return 0;
}

int alsps_register_timer(sns_sensor_instance *this, int stream_id, sns_time period, bool periodic)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    struct stream_node* node = NULL;

    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    uint8_t buffer[50];
    sns_request timer_req = {
        .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
        .request    = buffer
    };
    sns_rc rc = SNS_RC_SUCCESS;

    alsps_register_stream(this, stream_id, TIMER_STREAM);

    node = alsps_get_stream_node(stream_id);

    if (node) {
        if (!node->data_stream) {//need create new data_stream
            sns_service_manager *smgr = this->cb->get_service_manager(this);
            sns_stream_service  *strm_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);
            rc = strm_svc->api->create_sensor_instance_stream(strm_svc,
                    this, state->timer_suid, &node->data_stream);

            if ((rc != SNS_RC_SUCCESS) || (NULL == node->data_stream)) {
                OPLUS_ALS_PS_LOG("error creating stream %d 0x%x", rc, node->data_stream);
                alsps_unregister_stream(stream_id);
                return -EINVAL;
            }

            alsps_set_stream_valid(stream_id);
        }
    } else {
        OPLUS_ALS_PS_LOG("node NULL \n");
        return -EINVAL;
    }

    req_payload.is_periodic    = periodic;
    req_payload.start_time     = sns_get_system_time();
    req_payload.timeout_period = period;
    timer_req.request_len = pb_encode_request(buffer,
            sizeof(buffer),
            &req_payload,
            sns_timer_sensor_config_fields,
            NULL);

    if (timer_req.request_len > 0) {
        node->data_stream->api->send_request(node->data_stream, &timer_req);
    } else {
        return -EINVAL;
    }

    return 0;
}

int alsps_register_accel_by_resampler(sns_sensor_instance *this, int stream_id, uint64_t batch_period, uint64_t resample_rate)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    struct stream_node* node = NULL;
    uint8_t buffer[100];
    size_t encoded_len = 0;
    sns_rc rc = SNS_RC_SUCCESS;
    sns_resampler_config resampler_config = sns_resampler_config_init_default;
    sns_std_request std_req = sns_std_request_init_default;

    //init data
    sns_memscpy(&resampler_config.sensor_uid,
        sizeof(resampler_config.sensor_uid),
        &state->accel_suid,
        sizeof(state->accel_suid));
    std_req.has_batching = false;

    if (batch_period > (1000 / resample_rate)) {
        std_req.has_batching = true;
        std_req.batching.batch_period = batch_period * 1000;
    }

    resampler_config.resampled_rate = resample_rate;// 50hz;
    resampler_config.rate_type = SNS_RESAMPLER_RATE_MINIMUM;
    resampler_config.filter = false;

    alsps_register_stream(this, stream_id, REASMPLER_STREAM);

    node = alsps_get_stream_node(stream_id);

    if (node) {
        if (!node->data_stream) {//need create new data_stream
            sns_service_manager *smgr     = this->cb->get_service_manager(this);
            sns_stream_service  *strm_svc = (sns_stream_service*)
                smgr->get_service(smgr, SNS_STREAM_SERVICE);
            strm_svc->api->create_sensor_instance_stream(strm_svc,
                this,
                state->resampler_suid,
                &node->data_stream);

            if ((rc != SNS_RC_SUCCESS) || (NULL == node->data_stream)) {
                OPLUS_ALS_PS_LOG("error creating stream %d 0x%x", rc, node->data_stream);
                alsps_unregister_stream(stream_id);
                return -EINVAL;
            }
        }
    } else {
        OPLUS_ALS_PS_LOG("node NULL \n");
        return -EINVAL;
    }

    OPLUS_ALS_PS_LOG("std_req.has_batching = %d \n", std_req.has_batching);

    if (std_req.has_batching) {
        encoded_len = pb_encode_request(buffer,
                sizeof(buffer),
                &resampler_config,
                sns_resampler_config_fields, &std_req);
    } else {
        encoded_len = pb_encode_request(buffer,
                sizeof(buffer),
                &resampler_config,
                sns_resampler_config_fields, NULL);
    }

    if (0 < encoded_len) {
        sns_request request = (sns_request) {
            .message_id = SNS_RESAMPLER_MSGID_SNS_RESAMPLER_CONFIG,
            .request_len = encoded_len, .request = buffer
        };
        rc = node->data_stream->api->send_request(node->data_stream, &request);

        if (rc == SNS_RC_SUCCESS) {
            alsps_set_stream_valid(stream_id);
            OPLUS_ALS_PS_LOG("success to request accel \n");
        } else {
            OPLUS_ALS_PS_LOG("fail to request accel \n");
        }
    } else {
        OPLUS_ALS_PS_LOG("failed to encode resampler config \n");
    }

    return 0;
}

int alsps_register_interrupt(sns_sensor_instance *this, int stream_id)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    struct stream_node* node = NULL;
    uint8_t buffer[20];
    sns_rc rc = SNS_RC_SUCCESS;

    alsps_register_stream(this, stream_id, IRQ_STREAM);

    node = alsps_get_stream_node(stream_id);

    if (node) {//need create new data_stream
        if (!node->data_stream) {//need create new data_stream
            sns_service_manager *smgr     = this->cb->get_service_manager(this);
            sns_stream_service  *strm_svc = (sns_stream_service*)
                smgr->get_service(smgr, SNS_STREAM_SERVICE);
            strm_svc->api->create_sensor_instance_stream(strm_svc,
                this,
                state->irq_suid,
                &node->data_stream);

            if ((rc != SNS_RC_SUCCESS) || (NULL == node->data_stream)) {
                OPLUS_ALS_PS_LOG("error creating stream %d 0x%x", rc, node->data_stream);
                alsps_unregister_stream(stream_id);
                return -EINVAL;
            }

            alsps_set_stream_valid(stream_id);
        } else {
            OPLUS_ALS_PS_LOG("irq  has been registered\n");
            return -EINVAL;
        }
    } else {
        OPLUS_ALS_PS_LOG("node NULL \n");
        return -EINVAL;
    }

    sns_request irq_req = {
        .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
        .request    = buffer
    };

    irq_req.request_len = pb_encode_request(buffer,
            sizeof(buffer),
            &state->irq_config,
            sns_interrupt_req_fields,
            NULL);

    if (irq_req.request_len > 0) {
        node->data_stream->api->send_request(node->data_stream, &irq_req);
    } else {
        return -EINVAL;
    }

    return 0;
}

void alsps_remove_stream_safe(int stream_id)
{
    struct stream_node * node = NULL;

    alsps_set_stream_invalid(stream_id);
    node = alsps_get_stream_node(stream_id);

    if (node) {
        sns_sensor_util_remove_sensor_instance_stream(node->instance, &node->data_stream);
        node->data_stream = NULL;
    }
}

void alsps_remove_ps_data_stream(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
    alsps_remove_stream_safe(PS_MONITOR_TIMRER);
    alsps_remove_stream_safe(ACCEL_DATA);
}

void alsps_remove_als_data_stream(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
    alsps_remove_stream_safe(ALS_REPORT_DATA);
}

void alsps_remove_als_heartbeat_stream(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
    alsps_remove_stream_safe(ALS_HEARTBEAT_TIMER);
}


int alsps_clear_ps_int(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if (!state->ps_info.ops || !state->ps_info.ops->clear_ps_int) {
        return -EINVAL;
    }

    return state->ps_info.ops->clear_ps_int(scp_service, port_handle);
}

int alsps_get_ps_data(sns_sensor_instance *this, uint16_t *raw_data)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if (!state->ps_info.ops || !state->ps_info.ops->get_ps_data) {
        return -EINVAL;
    }

    return state->ps_info.ops->get_ps_data(scp_service, port_handle, raw_data);
}

void  alsps_recover_device(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;
    sns_rc rv  = SNS_RC_SUCCESS;

    if (!state->ps_info.ops || !state->ps_info.ops->recover_device) {
        //do nothing
    } else {
        rv = state->ps_info.ops->recover_device(scp_service, port_handle);

        if (rv == SNS_RC_SUCCESS) {
            state->ps_info.is_alsps_enable_error = false;
        }

        OPLUS_ALS_PS_LOG("irq time to quick try reset\n");
    }
}

sns_rc alsps_poweroff_com_port(sns_sensor * this)
{
    alsps_state *state = (alsps_state*)this->state->state;

    //Power Down and Close COM Port
    state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle, false);
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);

    return 0;
}

sns_rc alsps_reconfig_com_port(sns_sensor * this, uint8_t addr)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    state->com_port_info.com_config.slave_control = addr;
    state->com_port_info.com_config.bus_instance = state->bus_number;
    state->irq_config.interrupt_num = state->irq_number;

    //Register and Open COM Port
    if (NULL == state->com_port_info.port_handle) {
        rv = state->scp_service->api->sns_scp_register_com_port(
                &state->com_port_info.com_config,
                &state->com_port_info.port_handle);

        if (rv == SNS_RC_SUCCESS) {
            rv = state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
        }
    }

    return rv;
}

#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
static void alsps_tranfer_dts_param(alsps_state *state, struct sensor_hw *ps_hw)
{
    struct dynamic_thrd_parameter* dy_th_param;
    struct dynamic_cali_parameter *dc_parm;
    int dy_th_param_length = sizeof(struct dynamic_thrd_parameter) / sizeof(int);

    dy_th_param = s_list[state->vendor_id].ps_cali_parm->dt_parameter;
    dc_parm = s_list[state->vendor_id].ps_cali_parm->dc_parameter;

    memcpy(dy_th_param, &ps_hw->feature.parameter[0], sizeof(struct dynamic_thrd_parameter));
    memcpy(dc_parm, &ps_hw->feature.parameter[dy_th_param_length], sizeof(struct dynamic_cali_parameter));
    OPLUS_ALS_PS_SLOG("low_step %d high_step %d low_limit %d high_limit %d dirty_low_step %d"
        "dirty_high_step %d ps_dirty_limit %d ps_ir_limit %d ps_adjust_min %d"
        "ps_adjust_max %d sampling_count %d step_max %d step_min %d step_div %d anti_shake_delta %d",
        dy_th_param->low_step, dy_th_param->high_step, dy_th_param->low_limit, dy_th_param->high_limit,
        dy_th_param->dirty_low_step, dy_th_param->dirty_high_step, dy_th_param->ps_dirty_limit,
        dy_th_param->ps_ir_limit, dy_th_param->ps_adjust_min, dy_th_param->ps_adjust_max,
        dy_th_param->sampling_count, dy_th_param->step_max, dy_th_param->step_min, dy_th_param->step_div,
        dy_th_param->anti_shake_delta);
    OPLUS_ALS_PS_LOG("dynamic_cali_max %d raw2offset_radio %d offset_max %d offset_range_min %d"
        "offset_range_max %d force_cali_limit %d cali_jitter_limit %d cal_offset_margin %d",
        dc_parm->dynamic_cali_max, dc_parm->raw2offset_radio, dc_parm->offset_max, dc_parm->offset_range_min,
        dc_parm->offset_range_max, dc_parm->force_cali_limit, dc_parm->cali_jitter_limit, dc_parm->cal_offset_margin);
}


static void alsps_get_info_from_smem(alsps_state *state)
{
    struct sensor_hw *ps_hw = NULL;
    struct sensor_hw *als_hw = NULL;
    oppo_get_sensor_hw(OPPO_LIGHT, s_list[state->vendor_id].id, &als_hw);
    oppo_get_sensor_hw(OPPO_PROXIMITY, s_list[state->vendor_id].id, &ps_hw);

    if (ps_hw != NULL) {
        state->ps_factory_cali_max = ps_hw->feature.feature[3];

        ps_offset_cal =  ps_hw->feature.feature[4];
        OPLUS_ALS_PS_LOG("ps_offset_cal:%d\n",ps_offset_cal);

        if (ps_hw->feature.parameter[0] != 0) {/*ps parameter num 19*/
            alsps_tranfer_dts_param(state, ps_hw);
        }

        OPLUS_ALS_PS_LOG("prox %d sucess to probe. ps type %d parameter[0] %d"
            "is_need_check_pd %d ps_saturation %d ps_factory_cali_max %d",
            ps_hw->sensor_name, state->ps_type, ps_hw->feature.parameter[0],
            state->is_need_check_pd, state->ps_saturation, state->ps_factory_cali_max);
    }

    if (als_hw != NULL) {
        state->als_factor = als_hw->feature.feature[3];
        state->is_als_initialed = als_hw->feature.feature[4];
        state->als_buffer_length = als_hw->feature.feature[5];
        normal_value = als_hw->feature.feature[6];
        OPLUS_ALS_PS_LOG("als %d sucess to probe. als type %d is unit device %d "
            "is als dri %d als factor %d is als initialed %d als buffer length %d normal_value %d",
            als_hw->sensor_name, state->als_type, state->is_unit_device,
            state->is_als_dri, state->als_factor, state->is_als_initialed, state->als_buffer_length, normal_value);
    }
}
int get_als_normal_value(void)
{
    return normal_value;
}
bool select_ps_offset_cal(void)
{
    if (ps_offset_cal == 1) {
        return true;
    } else {
        return false;
    }
}
#endif


static void ps_special_process_before_avaliable(sns_sensor * this)
{
    alsps_state *state = (alsps_state*)this->state->state;

    if (s_list[state->vendor_id].ps_ops != NULL &&
        s_list[state->vendor_id].ps_ops->special_process_before_avaliable != NULL) {
        s_list[state->vendor_id].ps_ops->special_process_before_avaliable(state);
    }
}

sns_rc alsps_get_who_am_i(sns_sensor * this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = NULL;
    int ii = 0;
    sns_rc rv = SNS_RC_SUCCESS;
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
    int jj = 0;
    struct sensor_hw *ps_hw = NULL;
    struct sensor_hw *als_hw = NULL;
#endif

    for (ii = 0; ii < sizeof(s_list) / sizeof(struct vendor_node); ii++) {
        if (state->sensor == PS) {
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
            oppo_get_sensor_hw(OPPO_PROXIMITY, s_list[ii].id, &ps_hw);

            if (ps_hw != NULL) {
                state->irq_number = ps_hw->irq_number;
                state->bus_number = ps_hw->bus_number;
#endif
                OPLUS_ALS_PS_LOG("prox irq number: %d bus number: %d", state->irq_number, state->bus_number);
                rv = alsps_reconfig_com_port(this, s_list[ii].slave_addr);

                if (rv == SNS_RC_SUCCESS) {
                    port_handle = state->com_port_info.port_handle;

                    if ((s_list[ii].ps_ops != NULL) && (s_list[ii].ps_ops->get_who_am_i != NULL)) {
                        rv = s_list[ii].ps_ops->get_who_am_i(scp_service, port_handle);
                    } else {
                        rv = SNS_RC_FAILED;
                    }

                    if (rv == SNS_RC_SUCCESS) {
                        state->vendor_id = ii;
                        alsps_publish_name(this, s_list[ii].name);
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
                        state->reg_num = ps_hw->feature.reg[0];

                        for (jj = 0; jj < state->reg_num; jj++) {
                            state->reg_table[jj] = ps_hw->feature.reg[jj + 1];
                        }

                        state->is_need_check_pd = ps_hw->feature.feature[2];
                        state->ps_saturation = ps_hw->feature.feature[1];
                        alsps_reconifg_reg_table(this);
#endif
                        ps_special_process_before_avaliable(this);
                        alsps_poweroff_com_port(this);
                        break;
                    }
                }

                alsps_poweroff_com_port(this);
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
                ps_hw = NULL;
            }

#endif
        } else if (state->sensor == ALS) {
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
            oppo_get_sensor_hw(OPPO_LIGHT, s_list[ii].id, &als_hw);

            if (als_hw != NULL) {
                state->irq_number = als_hw->irq_number;
                state->bus_number = als_hw->bus_number;
#endif
                OPLUS_ALS_PS_LOG("als irq number: %d bus number: %d",
                    state->irq_number, state->bus_number);
                rv = alsps_reconfig_com_port(this, s_list[ii].slave_addr);

                if (rv == SNS_RC_SUCCESS) {
                    port_handle = state->com_port_info.port_handle;

                    if ((s_list[ii].als_ops != NULL) && (s_list[ii].als_ops->get_who_am_i != NULL)) {
                        rv = s_list[ii].als_ops->get_who_am_i(scp_service, port_handle);
                    } else {
                        rv = SNS_RC_FAILED;
                    }

                    if (rv == SNS_RC_SUCCESS) {
                        alsps_poweroff_com_port(this);
                        state->vendor_id = ii;
                        alsps_publish_name(this, s_list[ii].name);
                        break;
                    }
                }

                alsps_poweroff_com_port(this);
            }

#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
            als_hw = NULL;
        }

#endif
    }

    if (state->vendor_id >= 0) {
#ifdef ALSPS_GET_PARAMETER_FROM_SMEM
        alsps_get_info_from_smem(state);
#endif
    } else {
        rv = SNS_RC_FAILED;
        OPLUS_ALS_PS_LOG("fail to probe any devices \n");
    }

    return rv;
}

void alsps_ps_notify(sns_sensor_instance *this, uint16_t raw_data, sns_time curr_time)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    static alsps_ps_state status = PRX_NEAR_BY_UNKNOWN;

    if (raw_data > state->ps_info.thd_near) {
        status = RX_NEAR_BY;
    } else if (raw_data < state->ps_info.thd_far) {
        status = PRX_FAR_AWAY;
    } else {
        status = PRX_FAR_AWAY;
    }

    if (get_phone_status(this) && ps_need_gesture() && (status == RX_NEAR_BY) && (!state->amode_flag.first_near_by)) {
        state->amode_flag.first_near_by = true;
        state->amode_flag.is_delay_report = true;
        OPLUS_ALS_PS_LOG("set delay report true");
    }

    if (status != state->ps_info.status) {
        state->ps_info.status = status;
        state->ps_info.last_irq_time = curr_time;
        state->ps_info.data_stream_event.proximity_event_type = (sns_proximity_event_type)state->ps_info.status;
        state->ps_info.data_stream_event.raw_adc = (uint32_t)raw_data;
        state->ps_info.data_stream_event.status  = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

        if (state->amode_flag.is_delay_report && (status == PRX_FAR_AWAY)) {
            state->ps_info.delay_data_stream_event = state->ps_info.data_stream_event;
            state->amode_flag.need_report_far = true;
            OPLUS_ALS_PS_LOG("delay report wait the gesture");
        } else {
            state->amode_flag.need_report_far = false;
            pb_send_event(this,
                sns_proximity_event_fields,
                &state->ps_info.data_stream_event,
                curr_time,
                SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT,
                &state->ps_info.prox_suid);
            OPLUS_ALS_PS_SLOG("ps_notify:near = %d, far = %d, raw_data = %d, prx_type = %d ps_timestamp = %llu",
                state->ps_info.thd_near,
                state->ps_info.thd_far,
                raw_data,
                state->ps_info.status,
                curr_time);
        }

        if (status == PRX_FAR_AWAY) {
            is_ps_far = true;
        }
    }
}

void alsps_re_ps_notify(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    pb_send_event(this,
        sns_proximity_event_fields,
        &state->ps_info.data_stream_event,
        state->ps_info.last_irq_time,
        SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT,
        &state->ps_info.prox_suid);

    if (state->ps_info.status == PRX_FAR_AWAY) {
        is_ps_far = true;
    }
    OPLUS_ALS_PS_LOG("ps_notify:near = %d, far = %d, raw_data = %d, prx_type = %d",
        state->ps_info.thd_near,
        state->ps_info.thd_far,
        state->ps_info.data_stream_event.raw_adc,
        state->ps_info.status);
}
int alsps_get_als_data(sns_sensor_instance *this, float *raw_data, int len)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if (!state->als_info.ops || !state->als_info.ops->get_als_data) {
        return -EINVAL;
    }

    if (state->use_lb_algo && state->als_info.ops->get_als_fifo_data) {
        return state->als_info.ops->get_als_fifo_data(scp_service, port_handle, raw_data, len);
    } else {
        return state->als_info.ops->get_als_data(scp_service, port_handle, raw_data, len,
                      state->als_type, state->als_info.is_dri);
    }
}

void alsps_als_notify(sns_sensor_instance *this, float *raw_data, int len, sns_time curr_time)
{
    UNUSED_VAR(len);
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    float report_data[ALS_EVENT_SIZE];

    report_data[1] = (float)raw_data[1];

    if ((state->als_type == UNDER_LCD) || (state->als_type == VIRTUAL_UNDER_LCD)) {
        report_data[2] = (float)raw_data[2];
        report_data[0] = (float)raw_data[0];
    } else {
        report_data[0] = (float)raw_data[0] * state->als_info.scale / 1000 / (1000 / state->als_info.als_factor);
    }
    state->last_report_time = curr_time;
    pb_send_sensor_stream_event(this,
        &state->als_info.als_suid,
        curr_time,
        SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
        SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
        &report_data[0],
        ALS_EVENT_SIZE,
        state->encoded_als_event_len);

    OPLUS_ALS_PS_LOG("als_notify: raw_data = %d, report_data = %d ,scale = %d",
        (int)raw_data[0],
        (int)report_data[0],
        (int)state->als_info.scale);
}
void alsps_ps_irq_handle(struct stream_node *pos, sns_time curr_time)
{
    int rv = 0;
    uint16_t raw_data;

    //clear int
    alsps_clear_ps_int(pos->instance);

    //alsps_get_ps_data
    rv = alsps_get_ps_data(pos->instance, &raw_data);

    if (rv) {
        return;
    }

    alsps_ps_notify(pos->instance, raw_data, curr_time);
}
void alsps_clean_int(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if ((state->ps_info.ops != NULL) && (state->ps_info.ops->clear_ps_int != NULL)) {
        state->ps_info.ops->clear_ps_int(scp_service, port_handle);
    } else if ((state->als_info.ops != NULL)  && (state->als_info.ops->clear_als_int != NULL)) {
        state->als_info.ops->clear_als_int(scp_service, port_handle);
    }
}


void under_lcd_clean_state(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    state->als_under_info.data_full = false;
    state->als_info.first_data = false;
    state->als_info.is_new_request = false;
    is_ps_far = false;
    state->als_under_info.data_count = 0;
}
void handle_virtual_undler_lcd_als_sample(sns_sensor_instance *this, float * als_data, sns_time curr_time)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    als_data[1] = 0;
    state->als_under_info.data_count %= state->als_under_info.buffer_lenth;
    state->als_under_info.data_count++;
    SNS_PRINTF(ERROR, sns_fw_printf, "data_count = %d,buffer_lenth = %d",
        state->als_under_info.data_count, state->als_under_info.buffer_lenth);

    if (state->als_under_info.data_count >= state->als_under_info.buffer_lenth) {
        state->als_under_info.data_full = true;
    }

    if (state->als_info.first_data || state->als_info.is_new_request || is_ps_far) {
        alsps_als_notify(this, als_data, ALS_EVENT_SIZE, curr_time);
        SNS_PRINTF(ERROR, sns_fw_printf, "alsps_virtual_under_lce_handle_als, als_value = %d",
            (int)(als_data[0]));
        under_lcd_clean_state(this);
    }

    if (state->als_under_info.data_full) {
        alsps_als_notify(this, als_data, ALS_EVENT_SIZE, curr_time);
        SNS_PRINTF(ERROR, sns_fw_printf, "alsps_virtual_under_lce_handle_als, als_value = %d",
            (int)(als_data[0]));
        under_lcd_clean_state(this);
    }
}

void handle_undler_lcd_als_sample(sns_sensor_instance *this, float * als_data, sns_time curr_time)
{
    struct alsps_data* als_report_data = NULL;
    int temp_gap = 0;
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    state->als_under_info.data_count %= state->als_under_info.buffer_lenth;

    state->als_under_info.als_data[state->als_under_info.data_count].lux = als_data[0];
    state->als_under_info.als_data[state->als_under_info.data_count].classify_param = als_data[1];
    state->als_under_info.data_count++;
    SNS_PRINTF(ERROR, sns_fw_printf, "data_count = %d,buffer_lenth = %d",
        state->als_under_info.data_count, state->als_under_info.buffer_lenth);

    if (state->als_under_info.data_count >= state->als_under_info.buffer_lenth) {
        state->als_under_info.data_full = true;
    }

    if (state->als_info.first_data || state->als_info.is_new_request || is_ps_far) {
        als_data[2] = (float)(als_data[1]);
        als_data[1] = (float)0.0;
        temp_gap = (int)sns_get_ms_time_from_tick(sns_get_system_time() -state->last_report_time);
        if (is_ps_far && temp_gap <= 120) {
            SNS_PRINTF(ERROR, sns_fw_printf, "time gap %d", temp_gap);
            return;
        }
        alsps_als_notify(this, als_data, ALS_EVENT_SIZE, curr_time);
        under_lcd_clean_state(this);
        if (state->als_info.cali_status == ALSPS_CALI_IDLE && !state->als_info.is_factory_mode && !state->ps_info.is_ps_enable) {
            alsps_set_brightness(state, state->real_brightness);
        }
        SNS_PRINTF(ERROR, sns_fw_printf, "tcs3701_hb_report, als_value = %d, brightness = %d", (int)(als_data[0]),
                (int)state->real_brightness);
    }

    if (state->als_under_info.data_full) {
        als_report_data = get_proper_als_data(state->als_under_info.als_data,
                state->als_under_info.buffer_lenth);

        als_data[0] = als_report_data->lux;
        als_data[1] = als_report_data->time_dealta;
        als_data[2] = als_report_data->classify_param;

        alsps_als_notify(this, als_data, ALS_EVENT_SIZE, curr_time);
        under_lcd_clean_state(this);
        SNS_PRINTF(ERROR, sns_fw_printf, "tcs3701_hb_report, als_value = %d", (int)(als_data[0]));
    }
}

void alsps_get_irq_mask(struct stream_node *pos, sns_time curr_time, uint8_t* mask )
{
    UNUSED_VAR(curr_time);
    alsps_instance_state *state = (alsps_instance_state*)pos->instance->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if ((state->ps_info.ops != NULL) && (state->ps_info.ops->get_ps_device_irq_mask != NULL)) {
        state->ps_info.ops->get_ps_device_irq_mask(scp_service, port_handle, mask);
    } else if ((state->als_info.ops != NULL) && (state->als_info.ops->get_als_device_irq_mask != NULL)) {
        state->als_info.ops->get_als_device_irq_mask(scp_service, port_handle, mask);
    }
}

void alsps_alsps_irq_handle(struct stream_node *pos, sns_time curr_time)
{
    uint8_t mask = 0;
    int rv = 0;
    uint16_t raw_data;
    alsps_instance_state *state = (alsps_instance_state*)pos->instance->state->state;
    OPLUS_ALS_PS_LOG("alsps_alsps_irq_handle \n");

    if (state->ps_info.hw_cali_started) {
        // if in ps_fac_mode  not to solve intr
        OPLUS_ALS_PS_LOG("ps_fac_cali return");
        return;
    }

    alsps_get_irq_mask(pos, curr_time, &mask);

    if (state->als_info.is_dri && (mask & ALS_INT) != 0) {
        float als_data[ALS_EVENT_SIZE];
        state->als_info.last_irq_timestamp = curr_time;
        rv = alsps_get_als_data(pos->instance, als_data, ALS_EVENT_SIZE);
        if (rv) {
            return;
        }
        if (state->als_type == UNDER_LCD) {
            handle_undler_lcd_als_sample(pos->instance, als_data, curr_time);
        } else if (state->als_type == VIRTUAL_UNDER_LCD) {
            handle_virtual_undler_lcd_als_sample(pos->instance, als_data, curr_time);
        } else {
            alsps_als_notify(pos->instance, als_data, ALS_EVENT_SIZE, curr_time);
        }
    }

    if ((mask & PS_INT) != 0) {
        //alsps_get_ps_data
        if ( state->ps_info.last_irq_check_time == 0 ) {
            state->ps_info.last_irq_check_time = curr_time;
        } else {
            OPLUS_ALS_PS_LOG("ps_irq_time_gap = %d \n",
                (int)sns_get_ms_time_from_tick(curr_time - state->ps_info.last_irq_check_time));

            if (sns_get_ms_time_from_tick(curr_time - state->ps_info.last_irq_check_time) < 20) {
                state->ps_info.irq_error_count++;
            } else {
                state->ps_info.irq_error_count = 0;
            }

            if (state->ps_info.irq_error_count > IRQ_EROOR_REST) {
                state->ps_info.irq_error_count = 0;
                alsps_recover_device(pos->instance);
            }

            state->ps_info.last_irq_check_time = curr_time;
        }

        OPLUS_ALS_PS_LOG("ps_irq_error_count = %d \n", state->ps_info.irq_error_count);
        rv = alsps_get_ps_data(pos->instance, &raw_data);

        if (rv) {
            return;
        }

        alsps_ps_notify(pos->instance, raw_data, curr_time);
    }

    if ((mask & PS_CALI)) {
        if (!state->ps_info.ops || !state->ps_info.ops->get_offset) {
            OPLUS_ALS_PS_LOG("get_ofset  null \n");
        } else {
            state->ps_info.in_hard_cali = 0;
            state->ps_info.ops->get_offset(state->scp_service, state->port_handle, &state->ps_info.offset);
            alsps_ps_update_cali_params(state, PS_CAIL_DYNAMIC_PARAMS, state->ps_info.offset);
            state->update_dynamic_cal_registry = true;
            OPLUS_ALS_PS_LOG("get_ofset offset = %d \n", state->ps_info.offset);
        }
    }
}

void alsps_process_irq_events(struct stream_node *pos, sns_sensor_event *event)
{
    bool need_handle_irq = false;
    sns_time last_irq_time = 0;
    uint32_t count = 0;

    while (event) {
        if (!pos || !pos->is_vilid) {
            break;
        }

        if (event->message_id == SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT) {
            sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                    event->event_len);

            if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event)) {
                need_handle_irq = true;
                last_irq_time = irq_event.timestamp;
                count++;
                OPLUS_ALS_PS_LOG("ps_irq_count = %d \n", count);
            }
        }

        if (pos && pos->data_stream && pos->is_vilid) {
            event = pos->data_stream->api->get_next_input(pos->data_stream);
        } else {
            break;
        }
    }

    if (need_handle_irq) {
        if ((pos) && (pos->is_vilid)) {
            switch (pos->stream_id) {
            case PS_IRQ:
                OPLUS_ALS_PS_LOG("ps_irq enter\n");
                alsps_alsps_irq_handle(pos, last_irq_time);
                break;

            default:
                OPLUS_ALS_PS_LOG("not support irq \n");
                break;
            }
        }
    }
}

void alsps_avage_als(sns_sensor_instance *this, float *raw_data, int len, sns_time curr_time)
{
    UNUSED_VAR(len);
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    OPLUS_ALS_PS_LOG("als_count = %d als_sum= %d \n",
        state->als_info.als_count, state->als_info.als_sum);

    if (state->als_info.first_data) {
        alsps_als_notify(this, raw_data, ALS_EVENT_SIZE, curr_time);
        state->als_info.first_data = false;
    } else {
        state->als_info.als_count++;
        state->als_info.als_sum += raw_data[0];

        if (state->als_info.als_count >= 3) {
            raw_data[0] = state->als_info.als_sum / state->als_info.als_count;
            alsps_als_notify(this, raw_data, ALS_EVENT_SIZE, curr_time);
            state->als_info.als_count = 0;
            state->als_info.als_sum = 0;
        }
    }
}

void alsps_als_heatbeat_thread(sns_sensor_instance *this)
{
    sns_time curr_time = sns_get_system_time();
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    int tem_gap = (int)sns_get_ms_time_from_tick(curr_time - state->als_info.last_irq_timestamp);
    OPLUS_ALS_PS_LOG("heart beat timer event tem_gap = %d\n", tem_gap);
    if (tem_gap > 3000) {
        //6s no als data try recover
        #ifdef OPLUS_FEATURE_SENSOR_FB
        struct fb_event fb_event;
        memset(&fb_event, 0, sizeof(struct fb_event));
        fb_event.event_id = ALS_NO_INTERRUPT_ID;
        oplus_add_fd_event(&fb_event);
        #endif
        alsps_recover_device(this);
    }
}

void alsps_process_data_timer(struct stream_node* pos,sns_time curr_time)
{
    alsps_instance_state *state = (alsps_instance_state*)pos->instance->state->state;

    int rv = 0;
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    als_algo_state als_algo;
    static als_algo_state last_als_algo = ALS_ALGO_UNKNOWN;
#endif

    float als_data[ALS_EVENT_SIZE + 2] = {0}; // ALS_EVENT_SIZE -- als_algo_state  -- event_valid

    rv = alsps_get_als_data(pos->instance, als_data, ALS_EVENT_SIZE);
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    als_algo = (als_algo_state)EVENT_ALGO_STATE(als_data);
#endif
    if (rv) {
        SNS_PRINTF(ERROR, sns_fw_printf, "als read data error, rv %d", rv);
        return;
    }

    if (!state->use_lb_algo) {
        alsps_avage_als(pos->instance,als_data,ALS_EVENT_SIZE, curr_time);
    }
#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
    else {
        if(state->als_type == UNDER_LCD) {
            if (als_algo == ALS_ALGO_UNKNOWN || als_algo == COMPENSATION_ALGO) {
                if (IF_EVENT_VALID(als_data))
                    handle_undler_lcd_als_sample(pos->instance,als_data,curr_time);
            } else {
                if (IF_EVENT_VALID(als_data))
                    alsps_als_notify(pos->instance,als_data,ALS_EVENT_SIZE,curr_time);

                if (last_als_algo == COMPENSATION_ALGO)
                    under_lcd_clean_state(pos->instance);
            }

            last_als_algo = als_algo;
        } else if (state->als_type == VIRTUAL_UNDER_LCD){
            handle_virtual_undler_lcd_als_sample(pos->instance,als_data,curr_time);
        } else {
            alsps_als_notify(pos->instance,als_data,ALS_EVENT_SIZE,curr_time);
        }
    }
#endif

}

void alsps_process_timer_events(struct stream_node *pos, sns_sensor_event *event)
{
    alsps_instance_state *state = NULL;

    while (event) {
        if (!pos || !pos->is_vilid) {
            break;
        }

        if (event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT) {
            sns_timer_sensor_event timer_event;
            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                    event->event_len);

            if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event)) {
                if (pos) {
                    switch (pos->stream_id) {
                    case ALS_REPORT_DATA:
                        SNS_INST_PRINTF(ERROR, pos->instance, "als_timer \n");
                        alsps_process_data_timer(pos, timer_event.timeout_time);
                        break;

                    case ALS_FACTORY_CALI:
                        alsps_als_factory_cali(pos->instance);
                        break;

                    case PS_FACTORY_CALI:
                        state = (alsps_instance_state*)pos->instance->state->state;

                        if (state->ps_info.cali_type == PS_CALI_CROSSTALK && is_ps_support_hardware_cali()) {
                            alsps_ps_hardware_cali_handle(pos->instance);
                        } else {
                            alsps_ps_factory_cali(pos->instance, state->ps_info.cali_type);
                        }

                        break;

                    case PS_MONITOR_TIMRER:
                        alsps_ps_monitor_thread(pos->instance);
                        break;
                    case ALS_HEARTBEAT_TIMER:
                        alsps_als_heatbeat_thread(pos->instance);
                        break;
                    default:
                        OPLUS_ALS_PS_LOG("not support timer \n");
                        break;
                    }
                }
            }
        }

        if (pos && pos->data_stream && pos->is_vilid) {
            event = pos->data_stream->api->get_next_input(pos->data_stream);
        } else {
            break;
        }
    }
}
static void check_phone_call_state(struct stream_node *pos, int* data)
{
    alsps_instance_state *state = (alsps_instance_state*)pos->instance->state->state;

    int acc_x = data[0];
    int acc_y = data[1];
    int acc_z = data[2];

    static int x_last = 0;
    static int y_last = 0;
    static int z_last = 0;

    static int count = 0;
    static int debounce_count = 0;

    if (state->amode_flag.first_acc_check) {
        x_last = acc_x;
        y_last = acc_y;
        z_last = acc_z;

        count = 0;
        debounce_count = 0;
        state->amode_flag.first_acc_check = false;
    }

    /*
       enter delay report default true in call mode (to avoid the prox sensor detection distance < 2.5 cm)
       (x,y,z) in call mode for 500ms (500ms to avoid put the phone quickly but the (x,y,z) stll in call mode)

       exit delay report

       (x,y,z) not in call mode for 60ms (for the speed of lighting screen)
       or (x,y,z) change > 2000 in  200ms (considerig a action of puting the phone down)
       */
    if (((acc_z > -7000) && (acc_z < 2000)) || ((acc_z > 2000) && (acc_x < -5000)) || ((acc_z > 2000) && (acc_x > 5000))) {
        if (!state->amode_flag.is_delay_report) {
            debounce_count++;
        } else {
            debounce_count = 0;
        }

        OPLUS_ALS_PS_LOG("ir_prox delay report = %d,debounce_count = %d",
            state->amode_flag.is_delay_report, debounce_count);

        if (debounce_count >= 25) {
            debounce_count = 0;
            state->amode_flag.is_delay_report = true;

            OPLUS_ALS_PS_LOG("ir_prox enter delay report");
        }
    } else {
        if (state->amode_flag.is_delay_report) {
            debounce_count++;
        } else {
            debounce_count = 0;
        }

        OPLUS_ALS_PS_LOG("ir_prox delay report = %d,debounce_count = %d",
            state->amode_flag.is_delay_report, debounce_count);

        if (debounce_count >= 3) {
            debounce_count = 0;
            state->amode_flag.is_delay_report = false;

            OPLUS_ALS_PS_LOG("ir_prox exit delay report becase of position");
        }
    }

    //considerig a action of puting the phone down

    if (state->amode_flag.is_delay_report) {
        count++;
        OPLUS_ALS_PS_LOG("ir_prox delay report = %d,count = %d", state->amode_flag.is_delay_report, count);

        if (count == 9) {
            count = 0;

            if (((abs(acc_x - x_last) > 1500) && (abs(acc_y - y_last) > 1500))
                || ((abs(acc_x - x_last) > 1800) && (abs(acc_z - z_last) > 1800))
                || ((abs(acc_y - y_last) > 1800) && (abs(acc_z - z_last) > 1800))) {
                state->amode_flag.is_delay_report = false;

                OPLUS_ALS_PS_LOG("ir_prox exit delay report becase of puting downing");
            }

            x_last = acc_x;
            y_last = acc_y;
            z_last = acc_z;
        }
    } else {
        count = 0;
        x_last = acc_x;
        y_last = acc_y;
        z_last = acc_z;
    }
}
void alsps_process_resampler_events(struct stream_node *pos, sns_sensor_event *event)
{
    while (event) {
        if (!pos || !pos->is_vilid) {
            break;
        }

        if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == event->message_id) {
            float data[3] = {0};
            int input_data[3] = {0};
            uint8_t arr_index = 0;
            sns_std_sensor_event resampler_data = sns_std_sensor_event_init_default;
            pb_float_arr_arg arg = {
                .arr = data,
                .arr_len = ARR_SIZE(data),
                .arr_index = &arr_index
            };

            resampler_data.data = (struct pb_callback_s) {
                .funcs.decode = &pb_decode_float_arr_cb,
                .arg = &arg
            };

            pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

            if (pb_decode(&stream, sns_std_sensor_event_fields, &resampler_data)) {
                if (pos) {
                    switch (pos->stream_id) {
                    case ACCEL_DATA:
                        if (SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE == resampler_data.status) {
                            OPLUS_ALS_PS_LOG("received unreliable accel data \n");
                            break;
                        }

                        input_data[0] = (int)(1000 * data[0]);
                        input_data[1] = (int)(1000 * data[1]);
                        input_data[2] = (int)(1000 * data[2]);

                        if ((input_data[0] == 0) || (input_data[1] == 0) || (input_data[2] == 0)) {
                            //skip the invalid acc data
                        } else {
                            alsps_instance_state *state =
                                (alsps_instance_state*)pos->instance->state->state;
                            state->ps_info.last_acc_x = input_data[0];
                            state->ps_info.last_acc_y = input_data[1];
                            state->ps_info.last_acc_z = input_data[2];

                            if (state->ps_info.first_data && (state->ps_info.need_check_vertical)) {
                                OPLUS_ALS_PS_LOG("check phone status  reconfig acc \n");
                                alsps_ps_dynamic_algo_run(1);
                                state->ps_info.need_check_vertical = false;
                            }

                            if (ps_need_gesture() && get_phone_status(pos->instance) && state->amode_flag.first_near_by) {
                                check_phone_call_state(pos, input_data);
                            }

                            if (state->amode_flag.need_report_far && (!state->amode_flag.is_delay_report)) {
                                state->amode_flag.need_report_far = false;
                                pb_send_event(pos->instance,
                                    sns_proximity_event_fields,
                                    &state->ps_info.delay_data_stream_event,
                                    sns_get_system_time(),
                                    SNS_PROXIMITY_MSGID_SNS_PROXIMITY_EVENT,
                                    &state->ps_info.prox_suid);
                                OPLUS_ALS_PS_LOG("ps_delay notify:near = %d, far = %d, raw_data = %d, prx_type = %d",
                                    state->ps_info.thd_near,
                                    state->ps_info.thd_far,
                                    state->ps_info.delay_data_stream_event.raw_adc,
                                    state->ps_info.delay_data_stream_event.status);
                            }
                        }

                        OPLUS_ALS_PS_LOG("accl x:%d y:%d z:%d status %d\n",
                            input_data[0],
                            input_data[1],
                            input_data[2],
                            resampler_data.status);
                        break;

                    default:
                        OPLUS_ALS_PS_LOG("not support resampler \n");
                        break;
                    }
                }
            }
        }

        if (pos && pos->data_stream && pos->is_vilid) {
            event = pos->data_stream->api->get_next_input(pos->data_stream);
        } else {
            break;
        }
    }
}

void alsps_process_events(sns_sensor_instance *this)
{
    UNUSED_VAR(this);
    struct stream_node* pos = NULL;
    struct stream_node* n = NULL;
    sns_sensor_event *event = NULL;

    list_for_each_entry_safe(pos, n, struct stream_node, &(s_head.list), list) {
        if (!pos->is_vilid) {
            alsps_unregister_stream(pos->stream_id);
            continue;
        }

        if (pos && pos->data_stream && pos->is_vilid) {
            event = pos->data_stream->api->peek_input(pos->data_stream);

            switch (pos->type) {
            case TIMER_STREAM:
                alsps_process_timer_events(pos, event);
                break;

            case IRQ_STREAM:
                alsps_process_irq_events(pos, event);
                break;

            case REASMPLER_STREAM:
                alsps_process_resampler_events(pos, event);
                break;

            default:
                break;
            }
        }
    }
}

void alsps_send_config_event(sns_sensor_instance *const instance, sns_time timestamp, alsps_sensor_type sensor)
{
    alsps_instance_state *state =
        (alsps_instance_state*)instance->state->state;
    sns_std_sensor_physical_config_event phy_sensor_config =
        sns_std_sensor_physical_config_event_init_default;
    char operating_mode[] = "NORMAL";
    pb_buffer_arg op_mode_args;
    op_mode_args.buf = &operating_mode[0];
    op_mode_args.buf_len = sizeof(operating_mode);


    if (sensor == ALS && state->als_info.is_als_enable) {
        if (state->als_registry_cfg.is_dri) {
            phy_sensor_config.has_sample_rate = false;
        } else {
            phy_sensor_config.has_sample_rate = true;
            phy_sensor_config.sample_rate = DEFAULT_SAMPLING_RATE_HZ;
        }

        phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 90;
        phy_sensor_config.has_resolution = true;
        phy_sensor_config.resolution = ALSPS_ALS_RESOLUTION;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = ALSPS_ALS_RANGE_MIN;
        phy_sensor_config.range[1] = ALSPS_ALS_RANGE_MAX;
        phy_sensor_config.has_stream_is_synchronous = true;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = state->als_registry_cfg.is_dri;
        pb_send_event(instance,
            sns_std_sensor_physical_config_event_fields,
            &phy_sensor_config,
            timestamp,
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
            &state->als_info.als_suid);
    }

    if (sensor == PS && state->ps_info.is_ps_enable) {
        if (state->prox_registry_cfg.is_dri) {
            phy_sensor_config.has_sample_rate = false;
        } else {
            phy_sensor_config.has_sample_rate = true;
            phy_sensor_config.sample_rate = DEFAULT_SAMPLING_RATE_HZ;
        }

        phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
        phy_sensor_config.operation_mode.arg = &op_mode_args;
        phy_sensor_config.has_active_current = true;
        phy_sensor_config.active_current = 100;
        phy_sensor_config.range_count = 2;
        phy_sensor_config.range[0] = ALSPS_PROX_RANGE_MIN;
        phy_sensor_config.range[1] = ALSPS_PROX_RANGE_MAX;
        phy_sensor_config.has_stream_is_synchronous = true;
        phy_sensor_config.stream_is_synchronous = false;
        phy_sensor_config.has_dri_enabled = true;
        phy_sensor_config.dri_enabled = state->prox_registry_cfg.is_dri;
        pb_send_event(instance,
            sns_std_sensor_physical_config_event_fields,
            &phy_sensor_config,
            timestamp,
            SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
            &state->ps_info.prox_suid);
    }
}

sns_rc alsps_init_driver(sns_sensor_instance *const this, alsps_sensor_type sensor)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (sensor == ALS) {
        if (state->als_info.ops && state->als_info.ops->init_driver) {
            if (state->als_info.ops->enable_fifo) {
                state->als_info.ops->enable_fifo(state->use_lb_algo);
            }
            state->als_info.ops->init_driver(state->scp_service, state->port_handle);
            alsps_set_brightness(state, 1023);
        }
    } else if (sensor == PS) {
        if (state->ps_info.ops && state->ps_info.ops->init_driver) {
            state->ps_info.ops->init_driver(state->scp_service, state->port_handle);
        }
    }

    return 0;
}

sns_rc alsps_reconifg_reg_table(sns_sensor * this)
{
    alsps_state *sensor_state = (alsps_state*)this->state->state;

    if (sensor_state->reg_num < 2) {
        OPLUS_ALS_PS_LOG("no need to reconfig reg table");
        return 0;
    }

    //ps support, later add if als need
    if (sensor_state->sensor == PS) {
        if (s_list[sensor_state->vendor_id].ps_ops != NULL &&
            s_list[sensor_state->vendor_id].ps_ops->reconfig_reg_table != NULL) {
            s_list[sensor_state->vendor_id].ps_ops->reconfig_reg_table(sensor_state->reg_num,
                sensor_state->reg_table);
        }
    }

    return 0;
}


sns_rc alsps_deinit_driver(sns_sensor_instance *const this, alsps_sensor_type sensor)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (sensor == ALS) {
        if (state->als_info.ops && state->als_info.ops->deinit_driver) {
            state->als_info.ops->deinit_driver();
        }
    } else if (sensor == PS) {
        if (state->ps_info.ops && state->ps_info.ops->deinit_driver) {
            state->ps_info.ops->deinit_driver();
        }
    }

    return 0;
}

void alsps_als_enable(sns_sensor_instance *const this, bool enable)
{
    UNUSED_VAR(enable);
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_time timer_sample_rate = 0;
    sns_rc rv = SNS_RC_SUCCESS;

    if (!state->als_info.ops || !state->als_info.ops->als_enable) {
        return;
    }

    if (state->als_info.first_init) {
        state->als_info.first_init = false;
    }

    state->als_info.enable = state->als_info.is_als_enable;

    if (state->als_info.is_als_enable) {
        if ( state->als_info.is_dri ) {
            //tcs3701 prox and als use the same interrupt number
            if (!state->als_info.int_registered) {
                state->als_info.int_registered = true;
                alsps_init_irq(this, ALS);

                if (is_unit_device(this)) {
                    alsps_register_interrupt(this, PS_IRQ);
                    timer_sample_rate = sns_convert_ns_to_ticks((sns_time)((sns_time)6000* 1000 * 1000));
                    OPLUS_ALS_PS_LOG("register als heat beat timer");
                    alsps_register_timer(this, ALS_HEARTBEAT_TIMER, timer_sample_rate, true);
                } else {
                    //implement in future
                }
            }
        } else {
            OPLUS_ALS_PS_LOG("als odr = %d, ps odr = %d als_type = %d \n",
                (uint16_t)state->als_info.sampling_rate_hz,
                (uint16_t)state->ps_info.sampling_rate_hz,
                (int)state->als_type);


#ifdef SUPPORT_LOW_BRIGHTNESS_ALGO
            timer_sample_rate = sns_convert_ns_to_ticks((sns_time)(20 * 1000 * 1000));//20ms
#else
            timer_sample_rate = sns_convert_ns_to_ticks((sns_time)(1000000000.0 / 15.0));//15hz
#endif
            alsps_register_timer(this, ALS_REPORT_DATA, timer_sample_rate, true);
        }

        rv = state->als_info.ops->als_enable(state->scp_service, state->port_handle, true);

        if (rv != SNS_RC_SUCCESS) {
            //add count
            state->ps_info.is_alsps_enable_error = true;
        }

        is_ps_far = false;//after als enable to check the ps status
        state->als_info.first_data = true;
        state->als_info.als_count = 0;
        state->als_info.als_sum = 0;
        state->als_under_info.data_count = 0;
        alsps_set_brightness(state, 1023);
        //#if OPPO_ALSPS_ENABLE_DUMP_REG
        alsps_dump_reg(this, ALS);
        //#endif
    } else {
        state->als_info.ops->als_enable(state->scp_service, state->port_handle, false);
        alsps_remove_als_data_stream(this);
        if (state->als_info.is_dri && is_unit_device(this)) {
            OPLUS_ALS_PS_LOG("unregister als heat beat timer");
            alsps_remove_als_heartbeat_stream(this);
        }
        state->als_info.int_registered = false;
        state->als_under_info.data_count = 0;
        state->als_info.first_data = true;
    }
}

sns_rc alsps_ps_set_thd(sns_sensor_instance *const this,
    uint16_t ps_thd_near,
    uint16_t ps_thd_far)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (!state->ps_info.ops || !state->ps_info.ops->ps_set_thd) {
        return SNS_RC_FAILED;
    }

    state->ps_info.ops->ps_set_thd(state->scp_service, state->port_handle,
        ps_thd_near, ps_thd_far, state->ps_info.status);

    return 0;
}

static void alsps_ps_config_offset(alsps_instance_state *state)
{
    if (state->ps_fac_cal_data[0] > state->ps_fac_cal_data[5]) {
        state->ps_info.offset = state->ps_fac_cal_data[0];
    } else {
        state->ps_info.offset = state->ps_fac_cal_data[5];
    }
}

sns_rc alsps_ps_set_offset(sns_sensor_instance *const this, int val)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (!state->ps_info.ops || !state->ps_info.ops->set_offset) {
        return SNS_RC_FAILED;
    }

    state->ps_info.ops->set_offset(state->scp_service, state->port_handle, val);

    return 0;
}

static void alsps_ps_config_thrd(alsps_instance_state *state)
{
    if ((NULL != state->ps_info.ps_parms)) {
        state->ps_info.thd_near = state->ps_info.ps_parms->dt_parameter->high_limit;
        state->ps_info.thd_far = state->ps_info.ps_parms->dt_parameter->low_limit;
    }
}

/*config ps mode interface*/
static void alsps_ps_config_mode(alsps_instance_state *state)
{
    UNUSED_VAR(state);
    return;
}
void alsps_ps_enable(sns_sensor_instance *const this, bool enable)
{
    UNUSED_VAR(enable);
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    if (!state->ps_info.ops || !state->ps_info.ops->ps_enable) {
        return;
    }

    if (state->ps_info.first_init) {
        alsps_ps_config_thrd(state);
        state->ps_info.first_init = false;
    }

    state->ps_info.enable = state->ps_info.is_ps_enable;


    if (state->ps_info.is_ps_enable) {
        OPLUS_ALS_PS_LOG("status = %d  offset = %d", state->ps_info.status, state->ps_info.offset);

        alsps_ps_config_offset(state);
        alsps_ps_set_offset(this, state->ps_info.offset);
        alsps_ps_set_thd(this, state->ps_info.thd_near, state->ps_info.thd_far);
        alsps_ps_config_mode(state);

        if (state->prox_registry_cfg.is_dri) {
            if (!state->ps_info.int_registered) {
                state->ps_info.int_registered = true;
                alsps_init_irq(this, PS);
                alsps_register_interrupt(this, PS_IRQ);
            }

            //ps monitor timer
            alsps_register_timer(this, PS_MONITOR_TIMRER, sns_convert_ns_to_ticks(100 * 1000 * 1000), true);
            state->ps_info.need_check_vertical = true;
        } else {
            //do nothing
        }

        rv = state->ps_info.ops->ps_enable(state->scp_service, state->port_handle, true);

        if (rv != SNS_RC_SUCCESS) {
            //add count
            state->ps_info.is_alsps_enable_error = true;
        }
        alsps_set_brightness(state, 1023);

        state->ps_info.first_data = true;
//#if ALSPS_ALSPS_ENABLE_DUMP_REG
        alsps_dump_reg(this, PS);
//#endif
    } else {
        state->ps_info.ops->ps_enable(state->scp_service, state->port_handle, false);
        alsps_ps_dynamic_algo_run(0);
        alsps_remove_ps_data_stream(this);
        state->ps_info.first_init = true;
        state->ps_info.int_registered = false;
        state->ps_info.last_irq_check_time = 0;
        state->ps_info.irq_error_count = 0;
        state->ps_info.in_hard_cali = 0;
        state->ps_info.status = PRX_NEAR_BY_UNKNOWN;
        alsps_set_brightness(state, state->real_brightness);
    }
}


sns_rc alsps_dump_reg(sns_sensor_instance *const this, alsps_sensor_type sensor)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (sensor == ALS) {
        if (state->als_info.ops && state->als_info.ops->dump_reg) {
            state->als_info.ops->dump_reg(state->scp_service, state->port_handle);
        }
    } else if (sensor == PS) {
        if (state->ps_info.ops && state->ps_info.ops->dump_reg) {
            state->ps_info.ops->dump_reg(state->scp_service, state->port_handle);
        }
    }

    return 0;
}

sns_rc alsps_init_irq(sns_sensor_instance *const this, alsps_sensor_type sensor)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    if (sensor == ALS) {
        if (state->als_info.ops && state->als_info.ops->init_irq) {
            state->als_info.ops->init_irq(state->scp_service, state->port_handle);
        }
    } else if (sensor == PS) {
        if (state->ps_info.ops && state->ps_info.ops->init_irq) {
            state->ps_info.ops->init_irq(state->scp_service, state->port_handle);
        }
    }

    return 0;
}

void alsps_send_cal_event(sns_sensor_instance *const instance, alsps_sensor_type sensor_type)
{
    alsps_instance_state *state = (alsps_instance_state*)instance->state->state;
    sns_cal_event new_calibration_event = sns_cal_event_init_default;
    float bias_data[] = {0, 0};

    if (sensor_type == ALS) {
        bias_data[0] = state->als_fac_cal_data[0];
        bias_data[1] = state->als_fac_cal_data[1];
    } else if (sensor_type == PS) {
        return;
    }

    pb_buffer_arg buff_arg_bias = (pb_buffer_arg) {
        .buf = &bias_data,
        .buf_len = ARR_SIZE(bias_data)
    };
    sns_sensor_uid *suid_current;

    //update suid
    if (sensor_type == ALS) {
        suid_current = &state->als_info.als_suid;
    } else {
        suid_current = &state->ps_info.prox_suid;
    }

    new_calibration_event.bias.funcs.encode = &pb_encode_float_arr_cb;
    new_calibration_event.bias.arg = &buff_arg_bias;
    new_calibration_event.status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    OPLUS_ALS_PS_LOG("Calibration event");

    pb_send_event(instance, sns_cal_event_fields,
        &new_calibration_event,
        sns_get_system_time(),
        SNS_CAL_MSGID_SNS_CAL_EVENT,
        suid_current);
}

void alsps_send_sensor_test_event(sns_sensor_instance *instance,
    sns_sensor_uid *uid, bool test_result,
    sns_physical_sensor_test_type type)
{
    uint8_t data[1] = {0};
    pb_buffer_arg buff_arg = (pb_buffer_arg) {
        .buf = &data, .buf_len = sizeof(data)
    };

    sns_physical_sensor_test_event test_event =
        sns_physical_sensor_test_event_init_default;
    test_event.test_passed = test_result;
    test_event.test_type = type;
    test_event.test_data.funcs.encode = &pb_encode_string_cb;
    test_event.test_data.arg = &buff_arg;
    pb_send_event(instance,
        sns_physical_sensor_test_event_fields,
        &test_event,
        sns_get_system_time(),
        SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
        uid);
}

sns_rc alsps_als_factory_cali(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    float als_data[ALS_EVENT_SIZE];
    sns_rc rv = SNS_RC_SUCCESS;
    static int als_skip_count = 0;

    if (!state->als_info.ops || !state->als_info.ops->als_enable) {
        return rv;
    }

    switch (state->als_info.cali_status) {
    case ALSPS_CALI_IDLE:
        OPLUS_ALS_PS_LOG("alsps_als_factory_cali start\n");
        state->als_info.ops->als_enable(state->scp_service, state->port_handle, true);
        state->als_info.cali_status = ALSPS_CALI_RUNNING;
        state->als_info.cali_count = 0;
        state->als_info.cali_data = 0;
        als_skip_count = 0;
        alsps_set_brightness(state, 1023);
        alsps_register_timer(this, ALS_FACTORY_CALI, sns_convert_ns_to_ticks(100 * 1000 * 1000), true);

        break;

    case ALSPS_CALI_RUNNING:
        als_skip_count++;
        state->als_info.ops->get_als_data(state->scp_service,
            state->port_handle, als_data, ALS_EVENT_SIZE,
            state->als_type, state->als_info.is_dri);

        if (als_skip_count <= 2) {
            return rv;
        }

        state->als_info.cali_count++;
        state->als_info.cali_data += als_data[0];

        OPLUS_ALS_PS_LOG("alsps_als_factory_cali running data %d\n", (int)als_data[0]);

        if (state->als_info.cali_count >= ALS_CALI_NUM) {
            state->als_info.cali_data /= ALS_CALI_NUM;

            state->als_info.scale = ((float)ALS_CALI_TARGET_LUX / state->als_info.cali_data) * 1000 * 1000 / (state->als_info.als_factor);
            state->als_info.cali_status = ALSPS_CALI_DONE;

            if((state->als_info.scale >= 1000.00000f) && (state->als_info.scale <= 1000.99999f)) {
                state->als_info.scale += 1.0f;
                OPLUS_ALS_PS_LOG("ALS: start scale=%d\n", (uint32_t)state->als_info.scale);
            }
            state->als_fac_cal_data[0] = state->als_info.scale;
            state->als_cal_version[0]++;

            OPLUS_ALS_PS_LOG("ALS: Scale = %d, cal_ver[0] = %d  als_factory = %d \n",
                (uint32_t)(state->als_fac_cal_data[0]),
                state->als_cal_version[0], state->als_info.als_factor);

            if (!state->als_info.enable) {
                state->als_info.ops->als_enable(state->scp_service, state->port_handle, false);
            }
        }

        break;

    case ALSPS_CALI_DONE:
        OPLUS_ALS_PS_LOG("als_factory_cali:done(Success)");
        alsps_remove_stream_safe(ALS_FACTORY_CALI);
        alsps_set_brightness(state, state->real_brightness);
        state->update_cal_registry = true;

        alsps_send_cal_event(this, ALS);
        alsps_send_sensor_test_event(this,
            &state->als_info.als_suid,
            true,
            SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY);

        state->als_info.test_info.test_client_present = false;
        state->als_info.cali_status = ALSPS_CALI_IDLE;

        break;

    default:
        break;
    }

    return rv;
}



void alsps_ps_hardware_cali_handle(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;
    int offset = 0;
    bool test_result = false;

    if (!state->ps_info.ops || !state->ps_info.ops->get_hwcali_result ||
        !state->ps_info.ops->get_offset) {
        OPLUS_ALS_PS_LOG("get_hwcali_result null \n");
        return;
    }

    if (!state->ps_info.hw_cali_started) {
        OPLUS_ALS_PS_LOG("hw_cali no started \n");
        return;
    }

    state->ps_info.hw_cali_started = false;
    state->ps_info.ops->get_hwcali_result(scp_service, port_handle, &test_result);

    if (test_result) {
        state->ps_info.ops->get_offset(scp_service, port_handle, &offset);
        if (select_ps_offset_cal()) {
            alsps_ps_update_offset_calibration_params(state, offset);
        } else {
            alsps_ps_update_cali_params(state, PS_CAIL_FACTORY_UNCOVERED, offset);
        }
        state->update_dynamic_cal_registry = true;
        state->update_cal_registry = true;
        OPLUS_ALS_PS_LOG("ps_factory_cali offset = %d ver = %d",
            (int)state->ps_fac_cal_data[0],
            (uint32_t)state->ps_cal_version[0]);
    }

    OPLUS_ALS_PS_LOG("offset = %d test_result = %d", offset, test_result);

    alsps_send_sensor_test_event(this,
        &state->ps_info.prox_suid,
        test_result,
        SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY);

    state->ps_info.test_info.test_client_present = false;
    alsps_remove_stream_safe(PS_FACTORY_CALI);
}

void alsps_start_ps_hardware_cali(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    sns_sync_com_port_service *scp_service = state->scp_service;
    sns_sync_com_port_handle *port_handle = state->port_handle;

    if (!state->ps_info.ops || !state->ps_info.ops->hardware_cali) {
        OPLUS_ALS_PS_LOG("hardware_cali null \n");
        return;
    }

    state->ps_info.hw_cali_started = true;

    state->ps_info.ops->ps_enable(state->scp_service, state->port_handle, true);

    sns_busy_wait(sns_convert_ns_to_ticks(5 * 1000 * 1000)); //wait for pon ok
    sns_busy_wait(sns_convert_ns_to_ticks(5 * 1000 * 1000)); //wait for pon ok

    state->ps_info.ops->hardware_cali(scp_service, port_handle);


    alsps_register_timer(this, PS_FACTORY_CALI, sns_convert_ns_to_ticks(200 * 1000 * 1000), true);
}

sns_rc alsps_ps_factory_cali(sns_sensor_instance *this, uint8_t type)
{
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;
    uint16_t raw_data = 0;
    int offset = 0;
    bool cali_result = false;
    static int skip_count = 0;
    static sns_physical_sensor_test_type test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;

    if (!state->ps_info.ops || !state->ps_info.ops->ps_enable) {
        return rv;
    }

    //check if need hardware cali
    if (type == PS_CALI_CROSSTALK && is_ps_support_hardware_cali()) {
        OPLUS_ALS_PS_LOG("alsps_ps_hardware_cali start \n");
        alsps_start_ps_hardware_cali(this);
        return rv;
    }

    switch (state->ps_info.cali_status) {
    case ALSPS_CALI_IDLE:
        OPLUS_ALS_PS_LOG("alsps_ps_factory_cali start \n");
        state->ps_info.ops->ps_enable(state->scp_service, state->port_handle, true);

        if (type == PS_CALI_CROSSTALK) {
            alsps_ps_set_offset(this, 0);
            test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
        } else if (type == PS_CALI_3CM) {
            alsps_ps_set_offset(this, state->ps_info.offset);
            test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
        }

        state->ps_info.cali_status = ALSPS_CALI_RUNNING;
        skip_count = 0;
        state->ps_info.cali_count = 0;
        state->ps_info.cali_data = 0;
        alsps_register_timer(this, PS_FACTORY_CALI, sns_convert_ns_to_ticks(100 * 1000 * 1000), true);

        break;

    case ALSPS_CALI_RUNNING:
        state->ps_info.ops->get_ps_data(state->scp_service, state->port_handle, &raw_data);
        skip_count++;

        if (skip_count <= 2) {
            return rv;
        }

        state->ps_info.cali_data += raw_data;
        state->ps_info.cali_count++;

        OPLUS_ALS_PS_LOG("alsps_ps_factory_cali running data %d\n", raw_data);

        if (state->ps_info.cali_count >= PS_CALI_NUM) {
            state->ps_info.cali_status = ALSPS_CALI_DONE;

            if (!state->ps_info.enable) {
                state->ps_info.ops->ps_enable(state->scp_service, state->port_handle, false);
            }
        }

        break;

    case ALSPS_CALI_DONE:
        OPLUS_ALS_PS_LOG("alsps_ps_factory_cali done");
        alsps_remove_stream_safe(PS_FACTORY_CALI);
        state->ps_info.cali_status = ALSPS_CALI_IDLE;
        state->ps_info.cali_data /= PS_CALI_NUM;
        skip_count = 0;

        if (state->ps_info.cali_data < state->ps_info.ps_cali_goal) {
            // no need to cali
            offset = 0;
            cali_result = true;
            OPLUS_ALS_PS_LOG("no need to cali type %d ps_cali_data_avg %d",
                type, state->ps_info.cali_data);
        } else if (state->ps_info.cali_data > state->ps_info.ps_cali_up_thrd) {
            //fail
            cali_result = false;
            OPLUS_ALS_PS_LOG("fail to cali type %d up_thrd %d",
                type, state->ps_info.ps_cali_up_thrd);
        } else {
            //success
            cali_result = true;
            offset = state->ps_info.cali_data - state->ps_info.ps_cali_goal;
        }

        if (cali_result) {
            if (type == PS_CALI_CROSSTALK) {
                alsps_ps_update_cali_params(state, PS_CAIL_FACTORY_UNCOVERED, offset);
                state->update_dynamic_cal_registry = true;
                OPLUS_ALS_PS_LOG("ps_factory_cali offset = %d ver = %d",
                    (int)state->ps_fac_cal_data[0],
                    (uint32_t)state->ps_cal_version[0]);

            } else if (type == PS_CALI_3CM) {
                alsps_ps_update_cali_params(state, PS_CAIL_FACTORY_3CM, offset);
                OPLUS_ALS_PS_LOG("ps_factory_cali 3cm =  %d offset = %d ver = %d",
                    (uint32_t)state->ps_fac_cal_data[1],
                    (uint32_t)state->ps_fac_cal_data[2],
                    (uint32_t)state->ps_cal_version[1]);
            }

            state->update_cal_registry = true;
        }

        alsps_send_sensor_test_event(this,
            &state->ps_info.prox_suid,
            cali_result,
            test_type);

        state->ps_info.test_info.test_client_present = false;
        OPLUS_ALS_PS_LOG("ps_factory_cali: Calibration already finished");
        break;

    default:
        break;
    }

    return rv;
}


static void alsps_ps_workaround(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    int tem_gap = 0;
    sns_time curr_time;

    if (state->ps_info.is_alsps_enable_error) {
        alsps_recover_device(this);
    }

    if (state->ps_info.in_hard_cali) {
        //count
        tem_gap = (int)sns_get_ms_time_from_tick(sns_get_system_time() - state->ps_info.hard_cali_time);

        if (tem_gap > 600) {
            alsps_recover_device(this);
            state->ps_info.in_hard_cali = 0;
        }
    }

    if ((state->ps_info.last_ps < state->ps_info.thd_far && state->ps_info.status != PRX_FAR_AWAY)
        || (state->ps_info.last_ps > state->ps_info.thd_near && state->ps_info.status != RX_NEAR_BY)) {
        state->ps_info.lost_count++;

        if (state->ps_info.lost_count >= 4) {
            curr_time = sns_get_system_time();
            state->ps_info.lost_count = 0;
            alsps_ps_notify(this, state->ps_info.last_ps, curr_time);
            alsps_clear_ps_int(this);
        }
    } else {
        state->ps_info.lost_count  = 0;
    }
}

void alsps_ps_monitor_thread(sns_sensor_instance *this)
{
    sns_time curr_time;
    int tmp_offset = 0;
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;

    alsps_get_ps_data(this, &state->ps_info.last_ps);

    //monitor

    alsps_ps_logger_report_raw_data(this);
    if ((state->ps_info.last_ps < state->ps_info.ps_parms->dc_parameter->force_cali_limit)
        && (false != need_boot_cali)) {
        SNS_PRINTF(ERROR, sns_fw_printf, "no need boot cali ps = %d", state->ps_info.last_ps);
        need_boot_cali = false;
    }

    if (need_boot_cali) {
        if (state->ps_info.boot_cali_parms.count == 0) {
            state->ps_info.boot_cali_parms.max = state->ps_info.last_ps;
            state->ps_info.boot_cali_parms.min = state->ps_info.last_ps;
        } else {
            if (state->ps_info.last_ps > state->ps_info.boot_cali_parms.max) {
                state->ps_info.boot_cali_parms.max = state->ps_info.last_ps;
            }
            if (state->ps_info.last_ps < state->ps_info.boot_cali_parms.min) {
                state->ps_info.boot_cali_parms.min = state->ps_info.last_ps;
            }
        }
        state->ps_info.boot_cali_parms.count++;
        if (state->ps_info.boot_cali_parms.count >= 5) {
            if ((state->ps_info.boot_cali_parms.max - state->ps_info.boot_cali_parms.min) < state->ps_info.boot_cali_parms.delta) {
                need_boot_cali = false;
                if (is_ps_support_hardware_cali()) {
                    // no need boot cali
                } else {
                    tmp_offset = state->ps_info.offset + state->ps_info.boot_cali_parms.min;
                    if (tmp_offset > 65535) {
                        tmp_offset = 65535;
                    }
                    slit_prox_set_offset(tmp_offset);
                }
            } else {
                state->ps_info.boot_cali_parms.count = 0;
            }
        }
        SNS_PRINTF(ERROR, sns_fw_printf, "boot cali max= %d, min=%d, count = %d",
                    state->ps_info.boot_cali_parms.max,
                    state->ps_info.boot_cali_parms.min,
                    state->ps_info.boot_cali_parms.count);
    }
    //try to report
    if (state->ps_info.first_data) {
        state->ps_info.first_data = false;
        curr_time = sns_get_system_time();
        alsps_ps_notify(this, state->ps_info.last_ps, curr_time);
        alsps_ps_dynamic_force_cali(state->ps_info.last_ps);
    } else {
        alsps_ps_dynamic_algo_run(1);
    }

    if (state->ps_info.is_new_request) {
        state->ps_info.is_new_request = false;
        alsps_re_ps_notify(this);
    }

    alsps_ps_workaround(this);
}

void alsps_ps_logger_report_raw_data(sns_sensor_instance *this)
{
    log_data_info ps_raw_data;
    log_data_info pro_algo_log_data;
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    memset(&ps_raw_data, 0, sizeof(ps_raw_data));

    ps_raw_data.sensor_id = SENSOR_TYPE_PROXIMITY;
    ps_raw_data.string_id = ALSPS_LOG_ALGO_GET;
    ps_raw_data.argu2 = state->ps_info.last_acc_x;
    ps_raw_data.argu3 = state->ps_info.status;
    ps_raw_data.argu4 = state->ps_info.last_ps;
    ps_raw_data.argu5 = state->ps_info.thd_near;
    ps_raw_data.argu6 = state->ps_info.thd_far;
    sensors_log_report(ps_raw_data);


    memset(&pro_algo_log_data, 0, sizeof(pro_algo_log_data));
    pro_algo_log_data.sensor_id = SENSOR_TYPE_PROXIMITY;
    pro_algo_log_data.string_id = ALSPS_LOG_PROX_CONDITION;
    pro_algo_log_data.argu2 = state->ps_info.is_phone_mode;
    pro_algo_log_data.argu3 = state->ps_info.last_acc_y;
    pro_algo_log_data.argu4 = state->amode_flag.is_delay_report;
    pro_algo_log_data.argu5 = state->ps_info.offset;
    pro_algo_log_data.argu6 = state->ps_info.last_acc_z;
    sensors_log_report(pro_algo_log_data);


    SNS_INST_PRINTF(ERROR, this, "ps %d ,near %d ,far %d \n", state->ps_info.last_ps,
        state->ps_info.thd_near, state->ps_info.thd_far);
}
bool get_als_fac_mode(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    OPLUS_ALS_PS_LOG(" als_fac_mode = %d \n", state->als_info.is_factory_mode);
    return state->als_info.is_factory_mode;
}
bool get_phone_status(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state *)this->state->state;
    // remove the gesture first

    OPLUS_ALS_PS_LOG("is_phone_mode = %d", state->ps_info.is_phone_mode);

    return state->ps_info.is_phone_mode;
}

void alsps_set_brightness(alsps_instance_state *state, uint16_t brightness)
{
    if (state->use_lb_algo && state->als_info.ops && state->als_info.ops->set_brightness) {
        if (state->als_info.first_data || state->dc_mode || state->als_info.is_factory_mode
                  || state->ps_info.is_ps_enable) {
            brightness = 1023;
        }
        state->als_info.ops->set_brightness(brightness);
    }
}
