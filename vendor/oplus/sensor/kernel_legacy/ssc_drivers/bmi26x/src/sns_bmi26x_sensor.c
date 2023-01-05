/*******************************************************************************
 * Copyright (c) 2017-2020, Bosch Sensortec GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of Bosch Sensortec GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file sns_bmi26x_sensor.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"

#include "sns_bmi26x_sensor.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_cal_util.h"
#include "sns_registry.pb.h"
#include "sns_devinfo_utils.h"

#ifdef BMI26X_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif


#define BMI26X_REG_NN_ACCEL                     "bmi26x_0.accel.config"
#define BMI26X_REG_NN_GYRO                      "bmi26x_0.gyro.config"
#define BMI26X_REG_NN_TEMP                      "bmi26x_0.temp.config"
#define BMI26X_REG_NN_MD                        "bmi26x_0.md.config"
#define BMI26X_REG_NN_PLATFORM_CONFIG           "bmi26x_0_platform.config"
#define BMI26X_REG_NN_PLATFORM_PLACEMENT        "bmi26x_0_platform.placement"
#define BMI26X_REG_NN_PLATFORM_ORIENT           "bmi26x_0_platform.orient"
#define BMI26X_REG_NN_PLATFORM_FAC_CAL_ACCEL    "bmi26x_0_platform.accel.fac_cal"
#define BMI26X_REG_NN_PLATFORM_FAC_CAL_GYRO     "bmi26x_0_platform.gyro.fac_cal"
#define BMI26X_REG_NN_PLATFORM_FAC_CAL_TEMP     "bmi26x_0_platform.temp.fac_cal"
#define BMI26X_REG_NN_PLATFORM_MD_CONFIG        "bmi26x_0_platform.md.config"
#define BMI26X_FAC_CAL_BIAS 	".bias"

#if BMI26X_CONFIG_ENABLE_CRT
#define BMI26X_REG_NN_PLATFORM_CRT_CONFIG        "bmi26x_crt_state"
#define BMI26X_REG_NN_CRT_GAIN_GROUP_NAME        "crt_gain"
#define BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_X "gain_x"
#define BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_Y "gain_y"
#define BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_Z "gain_z"


#define BMI26X_REG_NN_MULTIPLE_CRT_CONFIG          "bmi26x_crt_config"
#define BMI26X_REG_NN_MULTIPLE_CRT_ROUP_NAME       "crt_cfg"
#define BMI26X_REG_NN_MULTIPLE_CRT_INTERVAL        "crt_itvl"
#define BMI26X_REG_NN_MULTIPLE_CRT_REPEAT_ON_ERROR "repeate_on_error"
#define BMI26X_REG_NN_MULTIPLE_CRT_XXX             "reserved"

#endif

#if BMI26X_CONFIG_ENABLE_OIS
#define BMI26X_REG_OIS                          "bmi26x_ois_cfg"
#define BMI26X_REG_GROUP_NAME                   "ois_cfg"
#define BMI26X_REG_EN                           "enable"
#define BMI26X_REG_RANGE_IDX                    "range_idx"
#define BMI26X_REG_SPI4                         "spi4"
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
#define BMI26X_REG_NN_PLATFORM_LOWG_CONFIG      "bmi26x_0_platform.lowg"
//#define BMI26X_REG_GROUP_NAME                  "ois_cfg"
#define BMI26X_REG_FREE_FALL_EN                  "enable"
#define BMI26X_REG_FREE_FALL_MODE                "mode"
#define BMI26X_REG_FREE_FALL_DEBUG               "debug"
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
#define BMI26X_REG_NN_PLATFORM_DOUBLE_TAP_CONFIG      "bmi26x_double_tap"
#define BMI26X_REG_DOUBLE_TAP_GROUP_NAME              "dbtap_cfg"
#define BMI26X_REG_DOUBLE_TAP_EN                      "enable"
#define BMI26X_REG_DOUBLE_TAP_SENS_THRES              "sens_thres"
#define BMI26X_REG_DOUBLE_TAP_GEST_DURATION           "max_gest_dur"
#define BMI26X_REG_DOUBLE_TAP_QUITE_TIME_AFTER_GEST   "quite_time_after_gest"
#endif


typedef struct pb_arg_reg_group_arg {
    sns_sensor_instance* instance;
    const char*         name;
    bmi26x_sensor_type  sensor;
    uint32_t            version;
} pb_arg_reg_group_arg;


extern const range_attr_t bmi26x_accel_ranges[];
extern const float bmi26x_accel_resolutions[];
extern const range_attr_t bmi26x_gyro_ranges[];
extern const float bmi26x_gyro_resolutions[];


// <power rail>
#if BMI26X_CONFIG_POWER_RAIL
static sns_rc bmi26x_register_power_rail(sns_sensor *const this)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    sns_service_manager *smgr = this->cb->get_service_manager(this);
    sns_rc rv = SNS_RC_SUCCESS;

    sstate->common.rail_config.rail_vote = SNS_RAIL_OFF;

    if (NULL == sstate->pwr_rail_service) {
        sstate->pwr_rail_service =
            (sns_pwr_rail_service*)smgr->get_service(smgr, SNS_POWER_RAIL_SERVICE);

        sstate->pwr_rail_service->api->sns_register_power_rails(sstate->pwr_rail_service, &sstate->common.rail_config);
    }

    if (NULL == sstate->pwr_rail_service) {
        rv = SNS_RC_FAILED;
    }
    return rv;
}
#endif



// <registry>
#if BMI26X_CONFIG_ENABLE_REGISTRY

#if BMI26X_CONFIG_ENABLE_CRT
static bool
bmi26x_sns_registry_parse_self_define_cfg_4_crt(sns_registry_data_item *reg_item,
        pb_buffer_arg *item_name,
        pb_buffer_arg *item_str_val,
        void *parsed_buffer)
{
    bool rc = true;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
            NULL == parsed_buffer) {
        rc = false;
    } else {
        bmi26x_self_define_state_t  *reg_arg  = (bmi26x_self_define_state_t *)parsed_buffer;
        bmi26x_state *sstate =  (bmi26x_state *) reg_arg->priv;

        BMI26X_SENSOR_LOG(LOW, sstate->owner, "start get crt value, %d %d %d",
                          reg_item->has_sint, reg_item->has_subgroup, reg_item->has_version);

        if (reg_item->has_sint) {
            if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_X,
                             item_name->buf_len)) {
                reg_arg->crt_buffer[0] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "crt.x: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_Y,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[1] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "crt.y: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_PLATFORM_CRT_CONFIG_GAIN_Z,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[2] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "crt.z: %d", reg_item->sint);
            } else  if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_MULTIPLE_CRT_INTERVAL,
                                     item_name->buf_len)) {
                reg_arg->crt_buffer[0] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "itvl: %d/min", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_MULTIPLE_CRT_REPEAT_ON_ERROR,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[1] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "repeat on error: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_NN_MULTIPLE_CRT_XXX,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[2] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "reserved: %d", reg_item->sint);
            }
        }
    }

    return rc;
}


static bool
bmi26x_crt_encode_registry_group_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
                                    void *const *arg)
{
    pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
    bmi26x_instance_state *istate =
        (bmi26x_instance_state*)pb_arg->instance->state->state;

    if (0 == strncmp(pb_arg->name, BMI26X_REG_NN_CRT_GAIN_GROUP_NAME, strlen("bias"))) {
        char const *names[] = {"gain_x", "gain_y", "gain_z"};

        for (uint32_t i = 0; i < ARR_SIZE(names); i++) {
            pb_buffer_arg name_data = (pb_buffer_arg) {
                .buf = names[i], .buf_len = strlen(names[i]) + 1
            };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;

            pb_item.name.funcs.encode = pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_sint = true;
            pb_item.has_version = true;

            if (pb_arg->sensor == BMI26X_GYRO) {
                if (i == 0) {
                    pb_item.sint = istate->gyro_info.sstate->crt_gain.gain_x;
                } else if (i == 1) {
                    pb_item.sint = istate->gyro_info.sstate->crt_gain.gain_y;
                } else if (i == 2) {
                    pb_item.sint = istate->gyro_info.sstate->crt_gain.gain_z;
                } else {
                }
                pb_item.version = istate->gyro_info.sstate->crt_version;
            }
            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
                BMI26X_INST_LOG(ERROR, pb_arg->instance, "ERROR!!! encoding sns_registry_data_item_fields");
                return false;
            }
        }
    }

    return true;
}


static bool
bmi26x_crt_encode_registry_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
                              void *const *arg)
{
    pb_arg_reg_group_arg *reg_arg = (pb_arg_reg_group_arg*)*arg;
    sns_sensor_instance *instance = reg_arg->instance;
    char const *names[] = {BMI26X_REG_NN_CRT_GAIN_GROUP_NAME};

    for (uint32_t i = 0; i < ARR_SIZE(names); i ++) {
        pb_buffer_arg name_data = (pb_buffer_arg) {
            .buf = names[i], .buf_len = strlen(names[i]) + 1
        };
        sns_registry_data_item pb_item = sns_registry_data_item_init_default;
        pb_arg_reg_group_arg pb_arg = (pb_arg_reg_group_arg) {
            .name = NULL, .instance = instance, .sensor = reg_arg->sensor
        };

        pb_item.has_version = true;
        pb_item.version = reg_arg->version;
        pb_item.name.arg = &name_data;
        pb_item.name.funcs.encode = pb_encode_string_cb;

        if (0 == strncmp(names[i], BMI26X_REG_NN_CRT_GAIN_GROUP_NAME, name_data.buf_len)) {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;
            pb_item.subgroup.items.funcs.encode = bmi26x_crt_encode_registry_group_cb;
            pb_item.subgroup.items.arg = &pb_arg;

        }

        if (!pb_encode_tag_for_field(stream, field)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! encoding corr_mat");

            return false;
        }

        if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! encoding sns_registry_data_item_fields");
            return false;
        }
    }

    return true;
}


void bmi26x_crt_update_registry(sns_sensor *const this,
                                sns_sensor_instance *const instance,
                                bmi26x_sensor_type sensor)
{

    bmi26x_state *sstate = (bmi26x_state *)this->state->state;
    bmi26x_instance_state *istate = (bmi26x_instance_state *)instance->state->state;

    if (NULL == this || NULL == instance)

        if (sensor != BMI26X_GYRO) {
            BMI26X_SENSOR_LOG(HIGH, this, "ignore on sensor:%d", sstate->sensor);
            return ;
        }

    pb_arg_reg_group_arg arg = {.instance = instance };

    uint8_t buffer[350];
    int32_t encoded_len;
    char crt_gain_name[] = BMI26X_REG_NN_PLATFORM_CRT_CONFIG;
    pb_buffer_arg name_data;
    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    name_data = (pb_buffer_arg) {
        .buf = crt_gain_name, .buf_len = strlen(crt_gain_name) + 1
    };

    arg.sensor = BMI26X_GYRO;
    arg.version = istate->gyro_info.sstate->crt_version;

    write_req.name.funcs.encode = pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = bmi26x_crt_encode_registry_cb;
    write_req.data.items.arg = &arg;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                    &write_req, sns_registry_write_req_fields, NULL);
    if (0 < encoded_len) {
        if (NULL == sstate->reg_data_stream) {
            sns_service_manager *smgr = this->cb->get_service_manager(this);
            sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

            sns_sensor_uid suid;
            sns_suid_lookup_get(&sstate->common.suid_lookup_data, "registry", &suid);


            stream_svc->api->create_sensor_stream(stream_svc, this, suid, &sstate->reg_data_stream);
        }

        if (NULL != sstate->reg_data_stream) {
            sns_request request = (sns_request) {
                .request_len = encoded_len, .request = buffer,
                .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ
            };
            sstate->reg_data_stream->api->send_request(sstate->reg_data_stream, &request);

            BMI26X_SENSOR_LOG(MED, this, "req_update_registry for %d len:%d version: %d", sensor, encoded_len, arg.version);
        }
    }
}

#endif


#if BMI26X_CONFIG_ENABLE_OIS
static bool
bmi26x_sns_registry_parse_self_define_cfg_4_ois(sns_registry_data_item *reg_item,
        pb_buffer_arg *item_name,
        pb_buffer_arg *item_str_val,
        void *parsed_buffer)
{
    bool rc = true;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
            NULL == parsed_buffer) {
        rc = false;
    } else {
        bmi26x_self_define_state_t  *reg_arg  = (bmi26x_self_define_state_t *)parsed_buffer;
        bmi26x_state *sstate =  (bmi26x_state *) reg_arg->priv;

        BMI26X_SENSOR_LOG(LOW, sstate->owner, "start crt cfg value, %d %d %d",
                          reg_item->has_sint, reg_item->has_subgroup, reg_item->has_version);

        if (reg_item->has_sint) {
            if (0 == strncmp((char*)item_name->buf, BMI26X_REG_EN,
                             item_name->buf_len)) {
                reg_arg->crt_buffer[0] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "ois.en: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_RANGE_IDX,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[1] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "ois.range_index: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_SPI4,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[2] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "spi4: %d", reg_item->sint);
            }
        }
    }

    return rc;
}
#endif


#if BMI26X_CONFIG_ENABLE_LOWG
static bool
bmi26x_sns_registry_parse_self_define_cfg_4_free_fall(sns_registry_data_item *reg_item,
        pb_buffer_arg *item_name,
        pb_buffer_arg *item_str_val,
        void *parsed_buffer)
{
    bool rc = true;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
            NULL == parsed_buffer) {
        rc = false;
    } else {
        bmi26x_self_define_state_t  *reg_arg  = (bmi26x_self_define_state_t *)parsed_buffer;
        bmi26x_state *sstate =  (bmi26x_state *) reg_arg->priv;

        BMI26X_SENSOR_LOG(LOW, sstate->owner, "start free.fall cfg, %d %d %d",
                          reg_item->has_sint, reg_item->has_subgroup, reg_item->has_version);

        if (reg_item->has_sint) {
            if (0 == strncmp((char*)item_name->buf, BMI26X_REG_FREE_FALL_EN,
                             item_name->buf_len)) {
                reg_arg->crt_buffer[0] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "lowg.en: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_FREE_FALL_MODE,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[1] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "lowg.mode: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_FREE_FALL_DEBUG,
                                    item_name->buf_len)) {
                reg_arg->crt_buffer[2] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "lowg.debug: %d", reg_item->sint);
            }
        }
    }

    return rc;
}
#endif


#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
static bool
bmi26x_sns_registry_parse_self_define_cfg_4_double_tap(sns_registry_data_item *reg_item,
        pb_buffer_arg *item_name,
        pb_buffer_arg *item_str_val,
        void *parsed_buffer)
{
    bool rc = true;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
            NULL == parsed_buffer) {
        rc = false;
    } else {
        bmi26x_self_define_cfg_int_type_t *reg_arg  = (bmi26x_self_define_cfg_int_type_t *)parsed_buffer;
        bmi26x_state *sstate =  (bmi26x_state *) reg_arg->priv;

        BMI26X_SENSOR_LOG(LOW, sstate->owner, "start to parse dbtap.cfg, %d %d %d",
                          reg_item->has_sint, reg_item->has_subgroup, reg_item->has_version);

        if (reg_item->has_sint) {
            if (0 == strncmp((char*)item_name->buf, BMI26X_REG_DOUBLE_TAP_EN,
                             item_name->buf_len)) {
                reg_arg->cfg_buffer[0] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "dbtap.en: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_DOUBLE_TAP_SENS_THRES,
                                    item_name->buf_len)) {
                reg_arg->cfg_buffer[1] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "dbtap.sens.thres: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_DOUBLE_TAP_GEST_DURATION,
                                    item_name->buf_len)) {
                reg_arg->cfg_buffer[2] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "dbtap.gest.dur: %d", reg_item->sint);
            } else if (0 == strncmp((char*)item_name->buf, BMI26X_REG_DOUBLE_TAP_QUITE_TIME_AFTER_GEST,
                                    item_name->buf_len)) {
                reg_arg->cfg_buffer[3] = reg_item->sint;
                BMI26X_SENSOR_LOG(LOW, sstate->owner, "dbtap.quite.time.afer.gest: %d", reg_item->sint);
            }
        }
    }

    return rc;
}
#endif



static const char ss_name[] = SENSOR_NAME;
static const char ss_temp_name[] = SENSOR_TEMP_NAME;
static const char ss_vendor[] = VENDOR_NAME;

void bmi26x_sensor_publish_default_registry_attributes(sns_sensor *const this, bmi26x_sensor_type ss_type)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
#if BMI26X_CONFIG_ENABLE_SEE_LITE
        UNUSED_VAR(this);
#else

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        if (ss_type & (BMI26X_ACCEL | BMI26X_GYRO)) {
            value.str.arg = &((pb_buffer_arg )
                        { .buf = ss_name, .buf_len = sizeof(ss_name) } );
        } else {
            // MUST BE Sensor Temperature
            value.str.arg = &((pb_buffer_arg )
                        { .buf = ss_temp_name, .buf_len = sizeof(ss_temp_name) } );
        }
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_NAME,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg )
                        { .buf = ss_vendor, .buf_len = sizeof(ss_vendor) } );
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_VENDOR,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI26X_SEE_DD_ATTRIB_VERSION;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_VERSION,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI26X_FF_MAX_FRAMES_IMU; // samples
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_FIFO_SIZE,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data values[] = { SNS_ATTR, SNS_ATTR };
        static char const op_mode1[] = BMI26X_LPM;
        static char const op_mode2[] = BMI26X_NORMAL;

        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg )
                        { .buf = op_mode1, .buf_len = sizeof(op_mode1) } );
        values[1].str.funcs.encode = pb_encode_string_cb;
        values[1].str.arg = &((pb_buffer_arg )
                        { .buf = op_mode2, .buf_len = sizeof(op_mode2) } );
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_OP_MODES,
                              values,
                              ARR_SIZE(values),
                              false);
    }

    {
        float data[3] = { 0 };
        sstate->encoded_event_len =
                        pb_get_encoded_size_sensor_stream_event(data, 3);
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->encoded_event_len;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_EVENT_SIZE,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data values[] = { SNS_ATTR };
        values[0].has_sint = true;
        values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
                              values,
                              ARR_SIZE(values),
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_DYNAMIC,
                              &value,
                              1,
                              false);
    }

    {
        const sns_std_sensor_rigid_body_type rigid_body =
                        SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = rigid_body;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_RIGID_BODY,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data values[] = { SNS_ATTR, SNS_ATTR, SNS_ATTR,
                        SNS_ATTR,
        SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
                        SNS_ATTR };
        for (uint8_t i = 0; i < 12; i ++) {
            values[i].has_flt = true;
            values[i].flt = 0;
        }
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_PLACEMENT,
                              values,
                              ARR_SIZE(values),
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = 0;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_HW_ID,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_DRI,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_STREAM_SYNC,
                              &value,
                              1,
                              false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(this,
                              SNS_STD_SENSOR_ATTRID_AVAILABLE,
                              &value,
                              1,
                              false);
    }

        //@@@@@@
#endif
}

/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void bmi26x_publish_registry_attributes(sns_sensor *const this)
{
#if !BMI26X_CONFIG_ENABLE_SEE_LITE
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = sstate->is_dri;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = sstate->supports_sync_stream;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->hardware_id;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
                                            SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR
                                           };
        for (uint8_t i = 0; i < 12; i ++) {
            values[i].has_flt = true;
            values[i].flt = sstate->common.placement[i];
        }
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
                              values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = sstate->common.registry_pf_cfg.rigid_body_type;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
    }

    if (sstate->sensor == BMI26X_ACCEL ||
            sstate->sensor == BMI26X_GYRO) {
        /** Only accel and gyro use registry information for selected resolution. */
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_flt = true;
        value.flt = (sstate->sensor == BMI26X_ACCEL) ?
                    bmi26x_accel_resolutions[sstate->resolution_idx] :
                    bmi26x_gyro_resolutions[sstate->resolution_idx];
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);
    }

    /** Only accel and gyro use registry information for selected range. */
    if (sstate->sensor == BMI26X_ACCEL ||
            sstate->sensor == BMI26X_GYRO) {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        sns_std_attr_value_data rangeMinMax[] = {SNS_ATTR, SNS_ATTR};
        rangeMinMax[0].has_flt = true;
        rangeMinMax[0].flt = (sstate->sensor == BMI26X_ACCEL) ?
                             bmi26x_accel_ranges[sstate->resolution_idx].min :
                             bmi26x_gyro_ranges[sstate->resolution_idx].min;
        rangeMinMax[1].has_flt = true;
        rangeMinMax[1].flt = (sstate->sensor == BMI26X_ACCEL) ?
                             bmi26x_accel_ranges[sstate->resolution_idx].max :
                             bmi26x_gyro_ranges[sstate->resolution_idx].max;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = rangeMinMax, .buf_len = ARR_SIZE(rangeMinMax)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0], ARR_SIZE(values), true);
    }
#else
    UNUSED_VAR(this);
#endif
}


void bmi26x_sensor_check_registry_col_progress(sns_sensor *const this)

{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    bool registry_collection_done = false;

    if (sstate->common.hw_is_present) {

        if (sstate->registry_cfg_received && !sstate->registry_attr_published) {
            sstate->registry_attr_published = 1;
            bmi26x_publish_registry_attributes(this);
            BMI26X_SENSOR_LOG(LOW, this, "registry attr published: %d", sstate->sensor);
        }

        registry_collection_done =
            sstate->registry_cfg_received &&
            sstate->common.registry_pf_cfg_received &&
            sstate->common.registry_orient_received &&
            sstate->common.registry_placement_received &&
            sstate->registry_md_config_received
#if BMI26X_CONFIG_ENABLE_CRT
            && sstate->registry_cfg_crt_received
#endif
#if BMI26X_CONFIG_ENABLE_OIS
            && sstate->registry_cfg_ois_received
#endif

            ;
    }

    /** More clean up. */
    switch (sstate->sensor) {
        case BMI26X_ACCEL:
        case BMI26X_GYRO:
            //TODOMAG
            //case BMI26X_SENSOR_TEMP:
            registry_collection_done = registry_collection_done && sstate->registry_fac_cal_received;
            break;
        default:
            break;
    }

    if (registry_collection_done) {
        sns_sensor_util_remove_sensor_stream(this, &sstate->reg_data_stream);
        BMI26X_SENSOR_LOG(LOW, this, "registry_collection_done: %d", sstate->sensor);
    }
}

bool
bmi26x_sns_registry_parse_self_define_cfg(sns_registry_data_item *reg_item,
        pb_buffer_arg *item_name,
        pb_buffer_arg *item_str_val,
        void *parsed_buffer)
{
    bool rv = true;

    if (NULL == reg_item || NULL == item_name || NULL == item_str_val ||
            NULL == parsed_buffer) {
        rv = false;
    } else if (reg_item->has_sint) {
        uint8_t *cfg =
            (uint8_t *)parsed_buffer;

        if (0 == strncmp((char*)item_name->buf, "ois_range", item_name->buf_len)) {
            *cfg = (reg_item->sint) ;
        } else {
            rv = false;
        }
    }

    return rv;
}


void bmi26x_sensor_process_registry_event(sns_sensor *const this,
        sns_sensor_event *event)
{
    bool rv = true;
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;

    pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
                          event->event_len);


    BMI26X_SENSOR_LOG(LOW, this, "process_registry_event snr: %d", sstate->sensor);

    if (SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id) {
        sns_registry_read_event read_event = sns_registry_read_event_init_default;
        pb_buffer_arg group_name = {0, 0};
        read_event.name.arg = &group_name;
        read_event.name.funcs.decode = pb_decode_string_cb;

        if (!pb_decode(&stream, sns_registry_read_event_fields, &read_event)) {
            BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! Error decoding registry event");
        } else {
            stream = pb_istream_from_buffer((void*)event->event, event->event_len);

            if (0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_ACCEL,
                             group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_GYRO,
                                 group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_TEMP,
                                 group_name.buf_len) ||
                    0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_MD,
                                 group_name.buf_len))
            {
#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_cfg,
                            .parsed_buffer = &sstate->common.registry_cfg
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    sstate->registry_cfg_received = true;
                    sstate->is_dri = sstate->common.registry_cfg.is_dri;
                    sstate->hardware_id = sstate->common.registry_cfg.hw_id;
                    sstate->resolution_idx = sstate->common.registry_cfg.res_idx;
                    sstate->supports_sync_stream = sstate->common.registry_cfg.sync_stream;

                    if (BMI26X_ACCEL == sstate->sensor) {
                        if (sstate->resolution_idx > BMI26X_RANGE_ACC_PM16G) {
                            sstate->resolution_idx = BMI26X_RANGE_ACC_PM16G;
                        }
                    } else if (BMI26X_GYRO == sstate->sensor) {
                        if (sstate->resolution_idx > BMI26X_RANGE_GYR_PM2000DPS) {
                            sstate->resolution_idx = BMI26X_RANGE_GYR_PM2000DPS;
                        }
                    }

                    BMI26X_SENSOR_LOG(LOW, this, "sensor: %d is_dri:%d, hardware_id:%d ",
                                      sstate->sensor,
                                      sstate->is_dri,
                                      (int)sstate->hardware_id);
                    BMI26X_SENSOR_LOG(LOW, this, "sensor: %d resolution_idx:%d, supports_sync_stream:%d ",
                                      sstate->sensor,
                                      sstate->common.registry_cfg.res_idx,
                                      sstate->supports_sync_stream);
                }
#endif
            }
#if BMI26X_CONFIG_ENABLE_OIS
            if (0 == strncmp((char*)group_name.buf, BMI26X_REG_OIS,
                             group_name.buf_len)) {
                bmi26x_self_define_state_t ois_cfg = {.crt_buffer = {0}};
                ois_cfg.priv = sstate;
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "ois_cfg",
                            .parse_func = bmi26x_sns_registry_parse_self_define_cfg_4_ois,
                            .parsed_buffer = &ois_cfg,
                        },
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    sstate->registry_cfg_ois_received = 1;
                    sstate->ois_cfg.enable = ois_cfg.crt_buffer[0];
                    sstate->ois_cfg.range_idx = ois_cfg.crt_buffer[1];
                    sstate->ois_cfg.spi4 = ois_cfg.crt_buffer[2];

                    BMI26X_SENSOR_LOG(LOW, this, "ois received: rc:%d <%d %d %d>",
                                      rv,
                                      sstate->ois_cfg.enable,
                                      sstate->ois_cfg.range_idx,
                                      sstate->ois_cfg.spi4);
                }
            }
#endif
#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            else if (0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_PLATFORM_CONFIG,
                                  group_name.buf_len))
            {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "config",
                            .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
                            .parsed_buffer = &sstate->common.registry_pf_cfg
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    sstate->common.registry_pf_cfg_received = true;

                    sstate->common.com_port_info.com_config.bus_type = (sns_bus_type)sstate->common.registry_pf_cfg.bus_type;
                    sstate->common.com_port_info.com_config.bus_instance = sstate->common.registry_pf_cfg.bus_instance;
                    sstate->common.com_port_info.com_config.slave_control = sstate->common.registry_pf_cfg.slave_config;
                    sstate->common.com_port_info.com_config.min_bus_speed_KHz = sstate->common.registry_pf_cfg.min_bus_speed_khz;
                    sstate->common.com_port_info.com_config.max_bus_speed_KHz = sstate->common.registry_pf_cfg.max_bus_speed_khz;
                    sstate->common.com_port_info.com_config.reg_addr_type = (sns_reg_addr_type)sstate->common.registry_pf_cfg.reg_addr_type;
                    sstate->common.irq_config.interrupt_num = sstate->common.registry_pf_cfg.dri_irq_num;
                    sstate->common.irq_config.interrupt_pull_type = (sns_interrupt_pull_type)sstate->common.registry_pf_cfg.irq_pull_type;
                    sstate->common.irq_config.is_chip_pin = sstate->common.registry_pf_cfg.irq_is_chip_pin;
                    sstate->common.irq_config.interrupt_drive_strength = (sns_interrupt_drive_strength)
                            sstate->common.registry_pf_cfg.irq_drive_strength;
                    sstate->common.irq_config.interrupt_trigger_type = (sns_interrupt_trigger_type)
                            sstate->common.registry_pf_cfg.irq_trigger_type;
                    sstate->common.rail_config.num_of_rails = sstate->common.registry_pf_cfg.num_rail;
                    sstate->common.registry_rail_on_state = (sns_power_rail_state)sstate->common.registry_pf_cfg.rail_on_state;
                    sns_strlcpy(sstate->common.rail_config.rails[0].name,
                                sstate->common.registry_pf_cfg.vddio_rail,
                                sizeof(sstate->common.rail_config.rails[0].name));
                    sns_strlcpy(sstate->common.rail_config.rails[1].name,
                                sstate->common.registry_pf_cfg.vdd_rail,
                                sizeof(sstate->common.rail_config.rails[1].name));

                    BMI26X_SENSOR_LOG(LOW, this, "bus_type:%d bus_instance:%d slave_control:%d",
                                      sstate->common.com_port_info.com_config.bus_type,
                                      sstate->common.com_port_info.com_config.bus_instance,
                                      sstate->common.com_port_info.com_config.slave_control);

                    BMI26X_SENSOR_LOG(LOW, this, "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                                      sstate->common.com_port_info.com_config.min_bus_speed_KHz,
                                      sstate->common.com_port_info.com_config.max_bus_speed_KHz,
                                      sstate->common.com_port_info.com_config.reg_addr_type);

                    BMI26X_SENSOR_LOG(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                                      sstate->common.irq_config.interrupt_num,
                                      sstate->common.irq_config.interrupt_pull_type,
                                      sstate->common.irq_config.is_chip_pin);

                    BMI26X_SENSOR_LOG(LOW, this, "interrupt_drive_strength:%d interrupt_trigger_type:%d"
                                      " rigid body type:%d",
                                      sstate->common.irq_config.interrupt_drive_strength,
                                      sstate->common.irq_config.interrupt_trigger_type,
                                      sstate->common.registry_pf_cfg.rigid_body_type);

                }
            }

            //OPTIM3
            else if (0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_PLATFORM_PLACEMENT,
                                  group_name.buf_len)) {
                {
                    uint8_t arr_index = 0;
                    pb_float_arr_arg arr_arg = {
                        .arr = sstate->common.placement,
                        .arr_index = &arr_index,
                        .arr_len = 12
                    };

                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "placement",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &arr_arg
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    sstate->common.registry_placement_received = true;
                }
            }
#endif
            //OPTIM3
            else if (0 == strncmp((char*)group_name.buf, BMI26X_REG_NN_PLATFORM_ORIENT,
                                  group_name.buf_len)) {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "orient",
                            .parse_func = sns_registry_parse_axis_orientation,
                            .parsed_buffer = sstate->common.axis_map
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    sstate->common.registry_orient_received = true;
                    //SENSOR_PRINTF_LOW_FULL(this, "Registry read event for group %s received ", (char*)group_name.buf);
                    #ifdef BMI26X_GET_PARAMETER_FROM_SMEM
                    {
                        uint8_t gsensor_dir_index;
                        struct sensor_direction gsensor_dir;
                        struct sensor_hw *acc_hw;
                        triaxis_conversion *axis_map = &sstate->common.axis_map[0];
                        oppo_get_sensor_hw(OPPO_ACCEL, BMI260, &acc_hw);
                        if(acc_hw != NULL && acc_hw->direction != DEFAULT_CONFIG) {
                            gsensor_dir_index = acc_hw->direction;
                            get_direction(gsensor_dir_index,&gsensor_dir);
                            axis_map[0].opaxis = gsensor_dir.map[0];
                            axis_map[0].invert = gsensor_dir.sign[0];
                            axis_map[1].opaxis = gsensor_dir.map[1];
                            axis_map[1].invert = gsensor_dir.sign[1];
                            axis_map[2].opaxis = gsensor_dir.map[2];
                            axis_map[2].invert = gsensor_dir.sign[2];
                            BMI26X_SENSOR_LOG(HIGH, this, "gsensor_dir: %d", gsensor_dir_index);
                        }
                    }
                    #endif

                    BMI26X_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                                      sstate->common.axis_map[0].ipaxis,
                                      sstate->common.axis_map[0].opaxis, sstate->common.axis_map[0].invert);

                    BMI26X_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                                      sstate->common.axis_map[1].ipaxis, sstate->common.axis_map[1].opaxis,
                                      sstate->common.axis_map[1].invert);

                    BMI26X_SENSOR_LOG(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                                      sstate->common.axis_map[2].ipaxis, sstate->common.axis_map[2].opaxis,
                                      sstate->common.axis_map[2].invert);
                }
            } else if (0 == strncmp((char*)group_name.buf,
                                    BMI26X_REG_NN_PLATFORM_FAC_CAL_ACCEL,
                                    group_name.buf_len) ||
                       0 == strncmp((char*)group_name.buf,
                                    BMI26X_REG_NN_PLATFORM_FAC_CAL_GYRO,
                                    group_name.buf_len) ||
                       0 == strncmp((char*)group_name.buf,
                                    BMI26X_REG_NN_PLATFORM_FAC_CAL_TEMP,
                                    group_name.buf_len)) {
                uint32_t fac_cal_version = 0;
                float fac_cal_bias[TRIAXIS_NUM];
                {
                    uint8_t bias_arr_index = 0, scale_arr_index = 0;
                    pb_float_arr_arg bias_arr_arg = {
                        .arr = fac_cal_bias,
                        .arr_index = &bias_arr_index,
                        .arr_len = TRIAXIS_NUM
                    };

                    pb_float_arr_arg scale_arr_arg = {
                        .arr = sstate->fac_cal_scale,
                        .arr_index = &scale_arr_index,
                        .arr_len = TRIAXIS_NUM
                    };

                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 3,
                        .parse_info[0] = {
                            .group_name = "bias",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &bias_arr_arg
                        },
                        .parse_info[1] = {
                            .group_name = "scale",
                            .parse_func = sns_registry_parse_float_arr,
                            .parsed_buffer = &scale_arr_arg
                        },
                        .parse_info[2] = {
                            .group_name = "corr_mat",
                            .parse_func = sns_registry_parse_corr_matrix_3,
                            .parsed_buffer = &sstate->fac_cal_corr_mat
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

                    fac_cal_version = arg.version;
                }

                if (rv) {
                    sstate->registry_fac_cal_received = true;
                    sstate->fac_cal_version = fac_cal_version;
                    BMI26X_SENSOR_LOG(LOW, this, "fac_cal received: %d", sstate->sensor);
                    uint8_t i;
                    for (i = 0; i < TRIAXIS_NUM; i++) {
                        sstate->fac_cal_bias[i] = roundf(fac_cal_bias[i] * sstate->scale_factor);
                    }
                    if (sstate->fac_cal_scale[0] != 0.0) {
                        sstate->fac_cal_corr_mat.e00 = sstate->fac_cal_scale[0];
                        sstate->fac_cal_corr_mat.e11 = sstate->fac_cal_scale[1];
                        sstate->fac_cal_corr_mat.e22 = sstate->fac_cal_scale[2];
                    }
                } else {
                    BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! fac_cal error: %d", sstate->sensor);
                }
            } else if (0 == strncmp((char*)group_name.buf,
                                    BMI26X_REG_NN_PLATFORM_MD_CONFIG,
                                    group_name.buf_len)) {
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = BMI26X_REG_NN_PLATFORM_MD_CONFIG,
                            .parse_func = sns_registry_parse_md_cfg,
                            .parsed_buffer = &sstate->md_config
                        }
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                }

                if (rv) {
                    BMI26X_SENSOR_LOG(MED, this, "md_config received: %d %d %d %d",
                                      sstate->sensor, (int)(sstate->md_config.thresh * 1e6),
                                      (int)(sstate->md_config.win * 1e6), sstate->md_config.disable);
                    sstate->registry_md_config_received = true;
                }
            }
#if BMI26X_CONFIG_ENABLE_CRT
            else if (0 == strncmp((char*)group_name.buf,
                                     BMI26X_REG_NN_PLATFORM_CRT_CONFIG,
                                     group_name.buf_len)) {
                uint32_t crt_version = 0;
                bmi26x_self_define_state_t crt_gain = {.crt_buffer = {0}};
                crt_gain.priv = sstate;
                {
                    sns_registry_decode_arg arg = {
                        .item_group_name = &group_name,
                        .parse_info_len = 1,
                        .parse_info[0] = {
                            .group_name = "crt_gain",
                            .parse_func = bmi26x_sns_registry_parse_self_define_cfg_4_crt,
                            .parsed_buffer = &crt_gain,
                        },
                    };

                    read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                    read_event.data.items.arg = &arg;

                    rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

                    crt_version = arg.version;
                }

                if (rv) {
                    sstate->registry_cfg_crt_received = true;
                    sstate->crt_version = crt_version;
                    sstate->crt_gain.gain_x = crt_gain.crt_buffer[0];
                    sstate->crt_gain.gain_y = crt_gain.crt_buffer[1];
                    sstate->crt_gain.gain_z = crt_gain.crt_buffer[2];

                    BMI26X_SENSOR_LOG(MED, this, "crt received: %d %d %d <%d %d %d>",
                                      rv, sstate->sensor,
                                      sstate->crt_version,
                                      sstate->crt_gain.gain_x,
                                      sstate->crt_gain.gain_y,
                                      sstate->crt_gain.gain_z);
                }
            } else if (0 == strncmp((char*)group_name.buf,
                                    BMI26X_REG_NN_MULTIPLE_CRT_CONFIG,
                                    group_name.buf_len)) {
               uint32_t crt_version = 0;
               bmi26x_self_define_state_t crt_cfg = {.crt_buffer = {0}};
               crt_cfg.priv = sstate;

               {
                   sns_registry_decode_arg arg = {
                       .item_group_name = &group_name,
                       .parse_info_len = 1,
                       .parse_info[0] = {
                           .group_name = BMI26X_REG_NN_MULTIPLE_CRT_ROUP_NAME,
                           .parse_func = bmi26x_sns_registry_parse_self_define_cfg_4_crt,
                           .parsed_buffer = &crt_cfg,
                       },
                   };

                   read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                   read_event.data.items.arg = &arg;

                   rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

                   crt_version = arg.version;
               }

               if (rv) {
                   sstate->registry_cfg_crt_received = true;
                   sstate->crt_cfg.version = crt_version;
                   sstate->crt_cfg.crt_execution_win = crt_cfg.crt_buffer[0];
                   sstate->crt_cfg.crt_repeate_on_error = crt_cfg.crt_buffer[1];

                   BMI26X_SENSOR_LOG(MED, this, "crt received: %d %d<%d %d %d>",
                                     rv, sstate->sensor,
                                     sstate->crt_cfg.version,
                                     sstate->crt_cfg.crt_execution_win,
                                     sstate->crt_cfg.crt_repeate_on_error,
                                     crt_cfg.crt_buffer[2]);
               }
            }
#endif

#if BMI26X_CONFIG_ENABLE_LOWG
            else if (0 == strncmp((char*)group_name.buf,
                    BMI26X_REG_NN_PLATFORM_LOWG_CONFIG,
                        group_name.buf_len)) {
                {

                    bmi26x_self_define_state_t free_fall_cfg = {.crt_buffer = {0}};
                    free_fall_cfg.priv = sstate;
                    {
                        sns_registry_decode_arg arg = {
                            .item_group_name = &group_name,
                            .parse_info_len = 1,
                            .parse_info[0] = {
                                .group_name = "config",
                                .parse_func = bmi26x_sns_registry_parse_self_define_cfg_4_free_fall,
                                .parsed_buffer = &free_fall_cfg,
                            },
                        };

                        read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                        read_event.data.items.arg = &arg;

                        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                    }

                    if (rv) {
                        sstate->registry_cfg_ois_received = 1;
                        sstate->lowg_config.enable = free_fall_cfg.crt_buffer[0];
                        sstate->lowg_config.mode = free_fall_cfg.crt_buffer[1];
                        sstate->lowg_config.debug = free_fall_cfg.crt_buffer[2];

                        BMI26X_SENSOR_LOG(LOW, this, "free.fall: rc:%d <%d %d %d>",
                                          rv,
                                          sstate->lowg_config.enable,
                                          sstate->lowg_config.mode,
                                          sstate->lowg_config.debug);
                    }
                }
            }
#endif

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP
            else if (0 == strncmp((char*)group_name.buf,
                    BMI26X_REG_NN_PLATFORM_DOUBLE_TAP_CONFIG,
                        group_name.buf_len)) {
                {

                    bmi26x_self_define_cfg_int_type_t db_tap_cfg = {.cfg_buffer = {0}};
                    db_tap_cfg.priv = sstate;
                    {
                        sns_registry_decode_arg arg = {
                            .item_group_name = &group_name,
                            .parse_info_len = 1,
                            .parse_info[0] = {
                                .group_name = "dbtap_cfg",
                                .parse_func = bmi26x_sns_registry_parse_self_define_cfg_4_double_tap,
                                .parsed_buffer = &db_tap_cfg,
                            },
                        };

                        read_event.data.items.funcs.decode = sns_registry_item_decode_cb;
                        read_event.data.items.arg = &arg;

                        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
                    }

                    if (rv) {
                        sstate->registry_cfg_ois_received = 1;
                        sstate->dbtap_cfg.enable = db_tap_cfg.cfg_buffer[0];
                        sstate->dbtap_cfg.tap_sens_thres = db_tap_cfg.cfg_buffer[1];
                        sstate->dbtap_cfg.max_gest_dur = db_tap_cfg.cfg_buffer[2];
                        sstate->dbtap_cfg.quite_time_after_gest = db_tap_cfg.cfg_buffer[3];

                        BMI26X_SENSOR_LOG(LOW, this, "free.fall: rc:%d <%d %d %d %d>",
                                          rv,
                                          sstate->dbtap_cfg.enable,
                                          sstate->dbtap_cfg.tap_sens_thres,
                                          sstate->dbtap_cfg.max_gest_dur,
                                          sstate->dbtap_cfg.quite_time_after_gest);
                    }
                }
            }
#endif

            else {
                rv = false;
            }

            if (!rv) {
                BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! Error decoding registry");
            }
        }
    } else {
        BMI26X_SENSOR_LOG(HIGH, this, "WARN!!! Received registry event msg id %u",
                          event->message_id);
    }
}


/**
 * Registry request send function
 * sensor send the getting registry configuration request to the Framework
 *
 * @param this             the sensor handler
 * @param reg_group_name   the registry group name
 *
 * @return none
 */
static void bmi26x_sensor_send_registry_request(sns_sensor *const this,
        char *reg_group_name)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    uint8_t buffer[100];
    int32_t encoded_len;
    sns_memset(buffer, 0, sizeof(buffer));
    sns_rc rc = SNS_RC_SUCCESS;

    sns_registry_read_req read_request;
    pb_buffer_arg data = (pb_buffer_arg) {
        .buf = reg_group_name,
        .buf_len = (strlen(reg_group_name) + 1)
    };

    read_request.name.arg = &data;
    read_request.name.funcs.encode = pb_encode_string_cb;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                    &read_request, sns_registry_read_req_fields, NULL);
    if (0 < encoded_len) {
        sns_request request = (sns_request) {
            .request_len = encoded_len,
            .request = buffer,
            .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ
        };
        rc = sstate->reg_data_stream->api->send_request(sstate->reg_data_stream, &request);

        BMI26X_SENSOR_LOG(LOW, this, "req_registry sent %d %d", sstate->sensor, rc);
    }

    //SENSOR_PRINTF_LOW_FULL(this, "Sending registry request for group name:%s", reg_group_name);
}


void bmi26x_request_registry(sns_sensor *const this)
{
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
                                     service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    BMI26X_SENSOR_LOG(LOW, this, "req_registry: %d 0x%x %d", sstate->sensor, sstate->reg_data_stream,
                      sstate->common.who_am_i);

    if ((NULL == sstate->reg_data_stream)
            && (0 == sstate->common.who_am_i)) {
        sns_sensor_uid suid;

        sns_suid_lookup_get(&sstate->common.suid_lookup_data, "registry", &suid);


        stream_svc->api->create_sensor_stream(stream_svc,
                                              this, suid, &sstate->reg_data_stream);

#if BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        sns_bmi26x_registry_def_config(this);
#endif

#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_CONFIG);
        bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_PLACEMENT);
        bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_MD_CONFIG);
#endif
        bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_ORIENT);

#if BMI26X_CONFIG_ENABLE_LOWG
        if (BMI26X_FREE_FALL == sstate->sensor) {
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_LOWG_CONFIG);
        }
#endif

        if (BMI26X_ACCEL == sstate->sensor) {
#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_ACCEL);
#endif

            bmi26x_sensor_send_registry_request(
                this, BMI26X_REG_NN_PLATFORM_FAC_CAL_ACCEL);
        } else if (BMI26X_GYRO == sstate->sensor) {
#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_GYRO);
#endif
            bmi26x_sensor_send_registry_request(
                this, BMI26X_REG_NN_PLATFORM_FAC_CAL_GYRO);
#if BMI26X_CONFIG_ENABLE_CRT
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_PLATFORM_CRT_CONFIG);
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_NN_MULTIPLE_CRT_CONFIG);
#endif
#if BMI26X_CONFIG_ENABLE_OIS
            bmi26x_sensor_send_registry_request(this, BMI26X_REG_OIS);
#endif
        }
#if !BMI26X_CONFIG_ENABLE_REGISTRY_LOAD_SPLIT
        else if (BMI26X_SENSOR_TEMP == sstate->sensor) {
            bmi26x_sensor_send_registry_request(
                this, BMI26X_REG_NN_TEMP);
            //todo: BMI26X_REG_NN_PLATFORM_FAC_CAL_TEMP
        } else if (BMI26X_MOTION_DETECT == sstate->sensor) {
            bmi26x_sensor_send_registry_request(
                this, BMI26X_REG_NN_MD);
        } else if (BMI26X_DOUBLE_TAP == sstate->sensor) {
            bmi26x_sensor_send_registry_request(
                this, BMI26X_REG_NN_PLATFORM_DOUBLE_TAP_CONFIG);
        }
#endif
    }
}


static bool
bmi26x_encode_registry_group_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
                                void *const *arg)
{
    pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
    bmi26x_instance_state *state =
        (bmi26x_instance_state*)pb_arg->instance->state->state;

    if (0 == strncmp(pb_arg->name, "bias", strlen("bias"))) {
        char const *names[] = {"x", "y", "z"};

        for (uint32_t i = 0; i < ARR_SIZE(names); i++) {
            pb_buffer_arg name_data = (pb_buffer_arg) {
                .buf = names[i], .buf_len = strlen(names[i]) + 1
            };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;

            pb_item.name.funcs.encode = pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;

            if (pb_arg->sensor == BMI26X_ACCEL) {
                pb_item.flt = state->accel_info.sstate->fac_cal_bias[i] / BMI26X_SCALE_FACTOR_DATA_ACCEL;
                pb_item.version = state->accel_info.sstate->fac_cal_version;
            } else {
                pb_item.flt = state->gyro_info.sstate->fac_cal_bias[i] / BMI26X_SCALE_FACTOR_DATA_DFT;
                pb_item.version = state->gyro_info.sstate->fac_cal_version;
            }

            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
                BMI26X_INST_LOG(ERROR, pb_arg->instance, "ERROR!!! Error encoding sns_registry_data_item_fields");
                return false;
            }
        }
    } else if (0 == strncmp(pb_arg->name, "corr_mat", strlen("corr_mat"))) {
        char const *names[] = {"0_0", "0_1", "0_2",
                               "1_0", "1_1", "1_2",
                               "2_0", "2_1", "2_2",
                              };

        for (uint32_t i = 0; i < ARR_SIZE(names); i++) {
            pb_buffer_arg name_data = (pb_buffer_arg) {
                .buf = names[i], .buf_len = strlen(names[i]) + 1
            };
            sns_registry_data_item pb_item = sns_registry_data_item_init_default;

            pb_item.name.funcs.encode = pb_encode_string_cb;
            pb_item.name.arg = &name_data;
            pb_item.has_flt = true;
            pb_item.has_version = true;
            if (pb_arg->sensor == BMI26X_ACCEL) {
                pb_item.flt = state->accel_info.sstate->fac_cal_corr_mat.data[i];
                pb_item.version = state->accel_info.sstate->fac_cal_version;
            } else {
                pb_item.flt = state->gyro_info.sstate->fac_cal_corr_mat.data[i];
                pb_item.version = state->gyro_info.sstate->fac_cal_version;
            }

            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }

            if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
                BMI26X_INST_LOG(ERROR, pb_arg->instance, "ERROR!!! Error encoding sns_registry_data_item_fields");
                return false;
            }
        }
    }
    return true;
}

static bool
bmi26x_encode_registry_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
                          void *const *arg)
{
    pb_arg_reg_group_arg *reg_arg = (pb_arg_reg_group_arg*)*arg;
    sns_sensor_instance *instance = reg_arg->instance;
    char const *names[] = {"bias", "corr_mat"};

    for (uint32_t i = 0; i < ARR_SIZE(names); i++) {
        pb_buffer_arg name_data = (pb_buffer_arg) {
            .buf = names[i], .buf_len = strlen(names[i]) + 1
        };
        sns_registry_data_item pb_item = sns_registry_data_item_init_default;
        pb_arg_reg_group_arg pb_arg = (pb_arg_reg_group_arg) {
            .name = NULL, .instance = instance, .sensor = reg_arg->sensor
        };

        pb_item.has_version = true;
        pb_item.version = reg_arg->version;
        pb_item.name.arg = &name_data;
        pb_item.name.funcs.encode = pb_encode_string_cb;

        if (0 == strncmp(names[i], "bias", name_data.buf_len)) {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;
            pb_item.subgroup.items.funcs.encode = bmi26x_encode_registry_group_cb;
            pb_item.subgroup.items.arg = &pb_arg;

        } else if (0 == strncmp(names[i], "corr_mat", name_data.buf_len)) {
            pb_arg.name = names[i];
            pb_item.has_subgroup = true;
            pb_item.subgroup.items.funcs.encode = bmi26x_encode_registry_group_cb;
            pb_item.subgroup.items.arg = &pb_arg;
        }
        if (!pb_encode_tag_for_field(stream, field)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! Error encoding corr_mat");

            return false;
        }

        if (!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item)) {
            BMI26X_INST_LOG(ERROR, instance, "ERROR!!! Error encoding sns_registry_data_item_fields");
            return false;
        }
    }

    return true;
}



void bmi26x_update_registry(sns_sensor *const this,
                            sns_sensor_instance *const instance, bmi26x_sensor_type sensor)
{
    bmi26x_state *state = (bmi26x_state*)this->state->state;
    pb_arg_reg_group_arg arg = {.instance = instance };

    uint8_t buffer[350];
    int32_t encoded_len;
    char accel_name[] = BMI26X_REG_NN_PLATFORM_FAC_CAL_ACCEL;
    char gyro_name[] = BMI26X_REG_NN_PLATFORM_FAC_CAL_GYRO;

    bmi26x_instance_state *istate = (bmi26x_instance_state *)instance->state->state;
    pb_buffer_arg name_data;
    sns_registry_write_req write_req = sns_registry_write_req_init_default;

    if (sensor == BMI26X_ACCEL) {
        name_data = (pb_buffer_arg) {
            .buf = accel_name, .buf_len = strlen(accel_name) + 1
        };
        arg.sensor = BMI26X_ACCEL;

        arg.version = istate->accel_info.sstate->fac_cal_version;
    } else if (sensor == BMI26X_GYRO) {
        name_data = (pb_buffer_arg) {
            .buf = gyro_name, .buf_len = strlen(gyro_name) + 1
        };
        arg.sensor = BMI26X_GYRO;

        arg.version = istate->gyro_info.sstate->fac_cal_version;
    } else {
        BMI26X_SENSOR_LOG(ERROR, this, "ERROR!!! Unsupported sensor %d", sensor);
        return;
    }

    write_req.name.funcs.encode = pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = bmi26x_encode_registry_cb;
    write_req.data.items.arg = &arg;

    encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                    &write_req, sns_registry_write_req_fields, NULL);
    if (0 < encoded_len) {
        if (NULL == state->reg_data_stream) {
            sns_service_manager *smgr = this->cb->get_service_manager(this);
            sns_stream_service *stream_svc = (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

            sns_sensor_uid suid;
            sns_suid_lookup_get(&state->common.suid_lookup_data, "registry", &suid);


            stream_svc->api->create_sensor_stream(stream_svc, this, suid, &state->reg_data_stream);
        }


        if (NULL != state->reg_data_stream) {
            sns_request request = (sns_request) {
                .request_len = encoded_len, .request = buffer,
                .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ
            };
            state->reg_data_stream->api->send_request(state->reg_data_stream, &request);

            BMI26X_SENSOR_LOG(MED, this, "req_update_registry for ss:%d len:%d on version: %d",
                    sensor, encoded_len, arg.version);
        }
    }

}
#endif

bool bmi26x_discover_hw(sns_sensor *const this)
{
    uint8_t chip_id;
    uint8_t dummy_byte;
    bool hw_is_present = false;
    bmi26x_state *state = (bmi26x_state*)this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

    if (NULL == state->common.com_port_info.port_handle) {
        rv = state->scp_service->api->sns_scp_register_com_port(
             &state->common.com_port_info.com_config,
             &state->common.com_port_info.port_handle);

        if ((SNS_RC_SUCCESS == rv) && (NULL != state->common.com_port_info.port_handle)) {
            rv = state->scp_service->api->sns_scp_open(state->common.com_port_info.port_handle);
        }
    }

    state->common.init_flags |= state->sensor;

    BMI26X_SENSOR_LOG(MED, this, "dhw comport handle:%p, init flags:0x%x",
                      state->common.com_port_info.port_handle,
                      state->common.init_flags);
    // CHECK prepare for SPI
    if (state->common.com_port_info.com_config.bus_type == SNS_BUS_SPI) {
        if (bmi26x_hal_sensor_prepare_spi_if(this) != SNS_RC_SUCCESS) {
        }
    }
    /**-------------------Read and Confirm WHO-AM-I------------------------*/
    dummy_byte = 0x00;
    chip_id = 0x00;
    rv = bmi26x_hal_get_who_am_i(state->scp_service, &state->common.com_port_info, &chip_id, &dummy_byte);
    BMI26X_SENSOR_LOG(MED, this, "@sensor:%d get chip id:0x%x 0x%x", state->sensor, chip_id, dummy_byte);
    if (rv == SNS_RC_SUCCESS
            &&
            (BMI26X_REGV_CHIP_ID_MAJOR == (chip_id & BMI26X_REGV_CHIP_ID_MAJOR))) {
        // Reset Sensor only if an instance is not already running
        if (NULL == sns_sensor_util_get_shared_instance(this)) {
        }

        if (rv == SNS_RC_SUCCESS) {
            if (BMI16X_REGV_CHIP_ID_MAJOR != (dummy_byte & BMI16X_REGV_CHIP_ID_MAJOR)) {
                hw_is_present = true;
            } else {
                BMI26X_SENSOR_LOG(MED, this, "BMI16X dev suspected");
            }
        }
    }
    state->common.who_am_i = chip_id;


    /**------------------Power Down and Close COM Port--------------------*/
    state->scp_service->api->sns_scp_update_bus_power(
        state->common.com_port_info.port_handle,
        false);

    state->scp_service->api->sns_scp_close(state->common.com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->common.com_port_info.port_handle);

#if BMI26X_CONFIG_POWER_RAIL
    /**----------------------Turn Power Rail OFF--------------------------*/
    state->common.rail_config.rail_vote = SNS_RAIL_OFF;
    state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
            this,
            &state->common.rail_config,
            NULL);
#endif

    state->common.hw_is_finish_detection = 1;
#ifdef OPLUS_FEATURE_SENSOR
    if (hw_is_present) {
        static bool register_deinfo = false;
        struct devinfo info_acc = {
                .cal_path = BMI26X_REG_NN_PLATFORM_FAC_CAL_ACCEL "" BMI26X_FAC_CAL_BIAS,
        };
        struct devinfo info_gyro = {
                .cal_path = BMI26X_REG_NN_PLATFORM_FAC_CAL_GYRO "" BMI26X_FAC_CAL_BIAS,
        };
        if (!register_deinfo) {
            register_deinfo = true;
            register_sensor_devinfo(OPLUS_GSENSOR,&info_acc);
            register_sensor_devinfo(OPLUS_GYRO,&info_gyro);
            SNS_PRINTF(HIGH, this, "bmi26X egister_sensor_devinfo");
        }
    }
#endif
    return hw_is_present;
}





void bmi26x_publish_available(sns_sensor * const this)
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}




void bmi26x_start_hw_detect_sequence(sns_sensor *this)
{
    bmi26x_state *sstate = (bmi26x_state *) this->state->state;
    sns_rc rv = SNS_RC_SUCCESS;

#if BMI26X_CONFIG_POWER_RAIL
    /**---------------------Register Power Rails --------------------------*/
    if (sns_suid_lookup_get(&sstate->common.suid_lookup_data, "timer", NULL)
            && NULL == sstate->pwr_rail_service
            && rv == SNS_RC_SUCCESS) {
        rv = bmi26x_register_power_rail(this);
        if (rv == SNS_RC_SUCCESS) {
        }

        /**---------------------Turn Power Rails ON----------------------------*/
        sstate->common.rail_config.rail_vote = sstate->common.registry_rail_on_state;

        if (rv == SNS_RC_SUCCESS) {
            sstate->pwr_rail_service->api->sns_vote_power_rail_update(sstate->pwr_rail_service,
                    this,
                    &sstate->common.rail_config,
                    NULL);
        }

        /**-------------Create a Timer stream for Power Rail ON timeout.---------*/
        if (rv == SNS_RC_SUCCESS) {
            bmi26x_start_power_rail_timer(this,
                                          sns_convert_ns_to_ticks(BMI26X_OFF_TO_IDLE_MS * 1000 * 1000),
                                          BMI26X_POWER_RAIL_PENDING_INIT);
        }
    }
#else
    sstate->common.hw_is_present = bmi26x_discover_hw(this);

    if (sstate->common.hw_is_present) {
        BMI26X_SENSOR_LOG(MED, this, "sensor:%d init finished", sstate->sensor);
        bmi26x_publish_available(this);
        bmi26x_update_sibling_sensors(this);
    } else {
        rv = SNS_RC_INVALID_LIBRARY_STATE;
        BMI26X_SENSOR_LOG(HIGH, this, "WARN!!! BMI26X HW absent");
    }
#endif
}

void bmi26x_update_sibling_sensors(sns_sensor * const this)
{
    sns_sensor *sensor = NULL;
    bmi26x_state *sstate;

    bmi26x_state *sstate_this = (bmi26x_state*) this->state->state;

    for (sensor = this->cb->get_library_sensor(this, true);
            sensor != NULL;
            sensor = this->cb->get_library_sensor(this, false)) {
        sstate = (bmi26x_state*) sensor->state->state;

        if (sstate->sensor != sstate_this->sensor) {
            sns_memscpy(&sstate->common, sizeof(sstate->common),
                        &sstate_this->common, sizeof(sstate_this->common));
#if BMI26X_CONFIG_POWER_RAIL
            if (bmi26x_register_power_rail(sensor) != SNS_RC_SUCCESS) {
            }
#endif
            bmi26x_publish_available(sensor);
        }

#if BMI26X_CONFIG_ENABLE_REGISTRY
        bmi26x_sensor_check_registry_col_progress(sensor);

        BMI26X_SENSOR_LOG(LOW, this, "update_siblings: %d %d", sstate->sensor, sstate->registry_cfg_received);
#else
        if (sstate->sensor != sstate_this->sensor) {
            sns_bmi26x_registry_def_config(sensor);
        }
#endif
    }
}

/**

 * Sensor common initialization functions
 * @param this   sensor handler
 */
void bmi26x_common_init(sns_sensor * const this)
{
    bmi26x_state *sstate = (bmi26x_state*) this->state->state;
    uint8_t i;

    struct sns_service_manager *smgr = this->cb->get_service_manager(this);
    sstate->diag_service = (sns_diag_service *) smgr->get_service(smgr, SNS_DIAG_SERVICE);
    sstate->scp_service = (sns_sync_com_port_service *) smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

    sstate->sensor_client_present = false;
    sstate->owner = this;

    if ((sstate->sensor == BMI26X_ACCEL) ||
            (sstate->sensor == BMI26X_GYRO)) {
        // initialize axis conversion settings
        for (i = 0; i < TRIAXIS_NUM; i ++) {
            sstate->common.axis_map[i].opaxis = (triaxis) i;
            sstate->common.axis_map[i].ipaxis = (triaxis) i;
            sstate->common.axis_map[i].invert = false;
        }
    }

    // initialize fac cal correction matrix to identity
    sstate->fac_cal_corr_mat.e00 = 1.0;
    sstate->fac_cal_corr_mat.e11 = 1.0;
    sstate->fac_cal_corr_mat.e22 = 1.0;

    SNS_SUID_LOOKUP_INIT(sstate->common.suid_lookup_data, NULL);
    if (BMI26X_ACCEL == sstate->sensor) {
#if BMI26X_CONFIG_ENABLE_DAE
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "data_acquisition_engine");
#endif
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "interrupt");
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "async_com_port");
        sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "timer");
    }

#if BMI26X_CONFIG_ENABLE_REGISTRY
    sns_suid_lookup_add(this, &sstate->common.suid_lookup_data, "registry");
#endif
}

