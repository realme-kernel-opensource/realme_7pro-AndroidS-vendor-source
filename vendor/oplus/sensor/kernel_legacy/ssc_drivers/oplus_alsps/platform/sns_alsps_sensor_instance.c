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
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_types.h"

#include "oplus_alsps.h"
#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"
#include "sns_alsps_feature.h"

extern sns_sensor_instance *als_mTask;
extern sns_sensor_instance *prox_mTask;

static void inst_cleanup(sns_sensor_instance *const this, sns_stream_service *stream_mgr)
{
    UNUSED_VAR(stream_mgr);
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;

    alsps_deinit_driver(this, state->rgt_sensor_type);

    if (is_unit_device(this)) {
        alsps_remove_stream_safe(PS_IRQ);
        alsps_remove_stream_safe(PS_FACTORY_CALI);
        alsps_remove_stream_safe(ALS_FACTORY_CALI);
        if (state->als_info.is_dri) {
            alsps_remove_stream_safe(ALS_HEARTBEAT_TIMER);
        }
    } else {
        if (state->rgt_sensor_type == ALS) {
            alsps_remove_stream_safe(ALS_FACTORY_CALI);
        } else {
            alsps_remove_stream_safe(PS_FACTORY_CALI);
            alsps_remove_stream_safe(PS_IRQ);
        }
    }

    if (NULL != state->scp_service) {
        state->scp_service->api->sns_scp_close(state->port_handle);
        state->scp_service->api->sns_scp_deregister_com_port(&state->port_handle);
        state->scp_service = NULL;
    }

    als_mTask = NULL;
    prox_mTask = NULL;
}


static void alsps_init_ps_params(alsps_instance_state *state, alsps_state *sensor_state,float offset)
{
    struct vendor_node *s_list = NULL;
    s_list = alsps_get_s_list();

    if (sensor_state->vendor_id >= 0 && s_list[sensor_state->vendor_id].ps_cali_parm) {
        state->ps_info.ps_parms = s_list[sensor_state->vendor_id].ps_cali_parm;
        state->ps_info.ps_parms->fc_parameter->delta = state->ps_fac_cal_data[2];
        state->ps_info.ps_parms->fc_parameter->offset = offset;
        alsps_ps_algo_init(state->ps_info.ps_parms);
    }
}

sns_rc alsps_inst_init(sns_sensor_instance* const this,
    sns_sensor_state const *sstate)
{
    sns_rc rv;
    float als_data[ALS_EVENT_SIZE];
    float prox_data[PROX_EVENT_SIZE];
    float temp_offset = 0.0;
    struct vendor_node *s_list = NULL;

    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    alsps_state *sensor_state   = (alsps_state*)sstate->state;

    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr = (sns_stream_service*)service_mgr->get_service(service_mgr,
            SNS_STREAM_SERVICE);
    state->scp_service = (sns_sync_com_port_service*)
        service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
    state->island_service = (sns_island_service*)
        service_mgr->get_service(service_mgr, SNS_ISLAND_SERVICE);

    /*1. Get size of sensor stream event*/
    state->encoded_als_event_len = pb_get_encoded_size_sensor_stream_event(als_data, ALS_EVENT_SIZE);
    state->encoded_ps_event_len = pb_get_encoded_size_sensor_stream_event(prox_data, PROX_EVENT_SIZE);

    OPLUS_ALS_PS_LOG("sensor %d, alsps_inst_init is_dri = %d,",
        sensor_state->sensor, sensor_state->is_dri);

    /* 2. Initialize COM port to be used by the Instance */
    sns_memscpy(&state->com_config,
        sizeof(state->com_config),
        &sensor_state->com_port_info.com_config,
        sizeof(sensor_state->com_port_info.com_config));

    rv = state->scp_service->api->sns_scp_register_com_port(&state->com_config, &state->port_handle);

    if (rv == SNS_RC_SUCCESS) {
        rv = state->scp_service->api->sns_scp_open(state->port_handle);

        if (rv != SNS_RC_SUCCESS) {
            inst_cleanup(this, stream_mgr);
            SNS_INST_PRINTF(ERROR, this, "fail to open com port %d", rv);
            return SNS_RC_FAILED;
        }
    } else {
        inst_cleanup(this, stream_mgr);
        SNS_INST_PRINTF(ERROR, this, "fail to register com port %d", rv);
        return SNS_RC_FAILED;
    }

    /* 3. Copy all Sensor UIDs in instance state */
    sns_memscpy(&state->als_info.als_suid,
        sizeof(state->als_info.als_suid),
        &((sns_sensor_uid)ALSPS_ALS_SUID),
        sizeof(state->als_info.als_suid));
    sns_memscpy(&state->ps_info.prox_suid,
        sizeof(state->ps_info.prox_suid),
        &((sns_sensor_uid)ALSPS_PS_SUID),
        sizeof(state->ps_info.prox_suid));
    sns_memscpy(&state->timer_suid,
        sizeof(state->timer_suid),
        &sensor_state->timer_suid,
        sizeof(sensor_state->timer_suid));
    sns_memscpy(&state->irq_suid,
        sizeof(state->irq_suid),
        &sensor_state->irq_suid,
        sizeof(sensor_state->irq_suid));
    sns_memscpy(&state->accel_suid,
        sizeof(state->accel_suid),
        &sensor_state->accel_suid,
        sizeof(sensor_state->accel_suid));
    sns_memscpy(&state->resampler_suid,
        sizeof(state->resampler_suid),
        &sensor_state->resampler_suid,
        sizeof(sensor_state->resampler_suid));

    /*4. transfer sensor state info to instance*/
    sns_memscpy(&state->irq_config,
        sizeof(sns_interrupt_req),
        &sensor_state->irq_config,
        sizeof(sns_interrupt_req));

    s_list = alsps_get_s_list();

    if (sensor_state->registry_fac_cal_received) {
        OPLUS_ALS_PS_LOG("alsps_inst_init: Update fac");

        state->als_fac_cal_data[0] = sensor_state->als_fac_cal_data[0];
        state->als_fac_cal_data[1] = sensor_state->als_fac_cal_data[1];
        state->als_cal_version[0]  = sensor_state->als_cal_version[0];
        state->als_cal_version[1]  = sensor_state->als_cal_version[1];
        sns_memscpy(&state->als_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg),
            &sensor_state->als_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg));

        for (int i = 0; i < sizeof(sensor_state->ps_fac_cal_data) / sizeof(float); i++) {
            state->ps_fac_cal_data[i] = sensor_state->ps_fac_cal_data[i];
            state->ps_cal_version[i] = sensor_state->ps_cal_version[i];
        }

        sns_memscpy(&state->prox_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg),
            &sensor_state->prox_registry_cfg,
            sizeof(sns_registry_phy_sensor_cfg));

        //need to sync this info for fac cali
        state->als_info.scale = state->als_fac_cal_data[0];
        state->als_info.bias  = state->als_fac_cal_data[1];

        if ( sensor_state->vendor_id >= 0 ) {
            if (s_list[sensor_state->vendor_id].ps_ops &&
                s_list[sensor_state->vendor_id].ps_ops->ps_offset_cali) {
                s_list[sensor_state->vendor_id].ps_ops->ps_offset_cali(state->ps_fac_cal_data[0],
                    state->ps_fac_cal_data[5],&(state->ps_info.offset),&temp_offset);
            } else {
                if (state->ps_fac_cal_data[0] > state->ps_fac_cal_data[5]) {
                    state->ps_info.offset = state->ps_fac_cal_data[0];
                } else {
                    state->ps_info.offset = state->ps_fac_cal_data[5];
                }
                temp_offset = state->ps_info.offset;
            }
        }

        OPLUS_ALS_PS_LOG("alsps_inst_init:fac_data[0] %d,fac_data[1] =%d,offset= %d,temp_offset= %d,vendor_id = %d",
            (int)state->ps_fac_cal_data[0], (int)state->ps_fac_cal_data[5],
            state->ps_info.offset,(int)temp_offset,sensor_state->vendor_id);
        state->ps_info.ps_cali_goal = state->ps_fac_cal_data[3];
        state->ps_info.ps_cali_up_thrd = state->ps_fac_cal_data[4];
    }

    state->als_type = sensor_state->als_type;
    state->use_lb_algo = sensor_state->use_lb_algo;
    state->ps_type = sensor_state->ps_type;
    state->als_under_info.buffer_lenth = sensor_state->als_buffer_length;
    state->als_under_info.persist_length = sensor_state->als_buffer_length;
    state->als_info.als_factor = sensor_state->als_factor;
    state->rgt_sensor_type = sensor_state->sensor;
    state->als_info.is_dri = sensor_state->is_als_dri;
    state->is_unit_device = sensor_state->is_unit_device;
    state->rgt_sensor_type = sensor_state->sensor;

    /*5 init ps als info*/
    state->ps_info.cali_status = ALSPS_CALI_IDLE;
    state->ps_info.cali_count = 0;
    state->ps_info.int_registered = false;
    state->ps_info.first_init = true;
    state->ps_info.first_data = false;
    state->ps_info.is_new_request = false;
    state->ps_info.enable = false;
    state->ps_info.status = PRX_NEAR_BY_UNKNOWN;//force to report
    state->ps_info.hw_cali_started = false;
    state->ps_info.is_phone_mode = false;
    state->amode_flag.first_acc_check = true;
    state->amode_flag.is_delay_report = false;
    state->amode_flag.first_near_by = false;
    state->amode_flag.need_report_far = false;
    state->ps_info.last_irq_check_time = 0;
    state->ps_info.irq_error_count = 0;
    state->ps_info.in_hard_cali = 0;
    state->ps_info.is_alsps_enable_error = false;
    state->ps_info.boot_cali_parms.count = 0;
    state->ps_info.boot_cali_parms.max = 0;
    state->ps_info.boot_cali_parms.min = 0;
    state->ps_info.boot_cali_parms.delta = 50;

    state->als_info.cali_status = ALSPS_CALI_IDLE;
    state->als_info.cali_count = 0;
    state->als_info.int_registered = false;
    state->als_info.first_data = false;
    state->als_info.enable = false;
    state->als_info.first_init = true;
    state->als_info.is_new_request = false;

    state->update_cal_registry = false;
    state->update_dynamic_cal_registry = false;
    state->is_first_init = true;
    state->real_brightness = 1023;

    /*6. init ps dynamic cali paramers*/
    if (is_unit_device(this) || sensor_state->sensor == PS) {
            alsps_init_ps_params(state, sensor_state,temp_offset);
    }

    /*7. register als ps ops*/
    if (sensor_state->sensor == ALS &&
        sensor_state->vendor_id >= 0 &&
        s_list[sensor_state->vendor_id].als_ops) {
        alsps_register_als(this, s_list[sensor_state->vendor_id].als_ops);
        als_mTask = this;
    } else if (sensor_state->sensor == PS &&
        sensor_state->vendor_id >= 0 &&
        s_list[sensor_state->vendor_id].ps_ops) {
        alsps_register_ps(this, s_list[sensor_state->vendor_id].ps_ops);
        prox_mTask = this;
    }

    /*8. close the COM port*/
    state->scp_service->api->sns_scp_update_bus_power(state->port_handle, false);
    state->als_info.last_irq_timestamp = 0;
    return SNS_RC_SUCCESS;
}

void alsps_run_sensor_test(sns_sensor_instance *instance)
{
    alsps_instance_state *state =
        (alsps_instance_state*)instance->state->state;

    if (state->als_info.test_info.test_client_present) {
        state->als_info.test_info.test_client_present = false;

        switch (state->als_info.test_info.test_type) {
        case SNS_PHYSICAL_SENSOR_TEST_TYPE_COM:
        case SNS_PHYSICAL_SENSOR_TEST_TYPE_SW:
        case SNS_PHYSICAL_SENSOR_TEST_TYPE_HW:
            break;

        case SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY:
            alsps_als_factory_cali(instance);
            break;

        default:
            break;
        }

    } else if (state->ps_info.test_info.test_client_present) {
        state->ps_info.test_info.test_client_present = false;

        switch (state->ps_info.test_info.test_type) {
        case SNS_PHYSICAL_SENSOR_TEST_TYPE_COM:
            state->ps_info.cali_type = PS_CALI_3CM;
            alsps_ps_factory_cali(instance, PS_CALI_3CM);
            break;

        case SNS_PHYSICAL_SENSOR_TEST_TYPE_SW:
        case SNS_PHYSICAL_SENSOR_TEST_TYPE_HW:
            break;

        case SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY:
            state->ps_info.cali_type = PS_CALI_CROSSTALK;
            alsps_ps_factory_cali(instance, PS_CALI_CROSSTALK);
            break;

        default:
            break;
        }
    }
}

sns_rc alsps_inst_deinit(sns_sensor_instance* const this)
{
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
        (sns_stream_service*)service_mgr->get_service(service_mgr,
            SNS_STREAM_SERVICE);

    inst_cleanup(this, stream_mgr);

    OPLUS_ALS_PS_LOG("alsps_inst_deinit");
    return SNS_RC_SUCCESS;
}
