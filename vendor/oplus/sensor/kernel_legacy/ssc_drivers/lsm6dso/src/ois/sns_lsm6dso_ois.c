/**
 * @file sns_lsm6dso_ois.c
 *
 * LSM6DSO OIS Sensor implementation.
 *
 * Copyright (c) 2020, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
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
 *
 **/

#include <string.h>
#include "sns_register.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_lsm6dso_sensor.h"
#include "sns_lsm6dso_ois.h"
#include "pb_encode.h"
#include "sns_service_manager.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"

#if LSM6DSO_OIS_ENABLED
extern sns_sensor_api lsm6dso_ois_sensor_api;
#define CONFIG_OIS             ".ois.config"
#define PLATFORM_FAC_CAL_OIS   "_platform.ois.fac_cal"
#define LSM6DSO_GEN_GROUP(x,group) NAME "_"#x group

enum {
  REG_CONFIG_OIS,
  REG_PLATFORM_FAC_CAL_OIS,
  REG_MAX_CONFIGS,
};

static char lsm6dso_ois_reg_config[SENSOR_INST_CNT][REG_MAX_CONFIGS][SNS_REGISTRY_MAX_NAME_LEN] = {
  {
    LSM6DSO_GEN_GROUP(0, CONFIG_OIS),
  },
#if LSM6DSO_DUAL_SENSOR_ENABLED
  {
    LSM6DSO_GEN_GROUP(1, CONFIG_OIS),
  }
#endif
};
/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static sns_rc
lsm6dso_publish_ois_attributes(sns_sensor *const this)
{
  {
    sns_std_attr_value_data rates = SNS_ATTR;

    rates.has_flt = true;
    rates.flt = LSM6DSO_OIS_RATE;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
        &rates, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    int i;
    for(i = 0; i < ARR_SIZE(values); i++)
    {
      values[i].has_flt = true;
      values[i].flt = lsm6dso_ois_resolutions[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
        values, i, false);
  }
  {
    sns_std_attr_value_data values = SNS_ATTR;
    {
      values.has_sint = true;
      values.sint = 550;
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
        &values, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 3; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const proto1[] = "sns_gyro.proto";
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,SNS_ATTR};
    sns_std_attr_value_data ranges[ARR_SIZE(values)][2] ={{SNS_ATTR}, {SNS_ATTR}};
    int i, j;
    for(i = 0; i < ARR_SIZE(values); i++)
    {
      values[i].has_flt = true;
      values[i].flt = lsm6dso_ois_resolutions[i];
      for(j = 0; j < ARR_SIZE(ranges[i]); j++) {
        ranges[i][j].has_flt = true;
        ranges[i][j].flt = (j) ? lsm6dso_gyro_range_max[i] :  lsm6dso_gyro_range_min[i];
      }
      values[i].has_subtype = true;
      values[i].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
      values[i].subtype.values.arg =
        &((pb_buffer_arg){ .buf = ranges[i], .buf_len = ARR_SIZE(ranges[i]) });
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
        values, ARR_SIZE(values), true);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const op_mode1[] = LSM6DSO_HIGH_PERF;

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }
  {
    const char type[] = "ois_sensor";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, true); //last attribute
  }
  return SNS_RC_SUCCESS;
}

void lsm6dso_process_ois_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);

  pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
      event->event_len);

  if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
  {
    sns_registry_read_event read_event = sns_registry_read_event_init_default;
    pb_buffer_arg group_name = {0,0};
    read_event.name.arg = &group_name;
    read_event.name.funcs.decode = pb_decode_string_cb;

    if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
    {
      DBG_PRINTF(HIGH, this,  "Error decoding registry event");
    }
    else
    {
      bool rv = true;
      uint32_t hw_id = shared_state->hw_idx;
      stream = pb_istream_from_buffer((void*)event->event, event->event_len);
      if(0 == strncmp((char*)group_name.buf, lsm6dso_ois_reg_config[hw_id][REG_CONFIG_OIS],
                      group_name.buf_len))
      {
        lsm6dso_decode_sensor_config_registry_data(this, &stream, &group_name, &read_event);
      }
#if 0
      else if(0 == strncmp((char*)group_name.buf,
            lsm6dso_ois_reg_config[hw_id][REG_PLATFORM_FAC_CAL_OIS],
            group_name.buf_len))
      {
        lsm6dso_decode_fac_cal_registry_data(this, &stream, &group_name, &read_event, shared_state->inst_cfg.ois_reg_cfg.cal);
      }
#endif
      else
      {
        rv = false;
      }
      if(!rv)
      {
        DBG_PRINTF(HIGH, this,  "Error decoding registry group ");
      }
    }
    state->outstanding_reg_requests--;
  }
}

void lsm6dso_update_ois_resolution_idx(sns_sensor *const this, uint8_t res_idx)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  shared_state->inst_cfg.ois_reg_cfg.resolution_idx = res_idx;
}

void lsm6dso_send_ois_registry_requests(sns_sensor *const this, uint8_t hw_id)
{
  //fac calibration
  //lsm6dso_send_registry_request(this, lsm6dso_ois_reg_config[hw_id][REG_PLATFORM_FAC_CAL_OIS]);
  lsm6dso_send_registry_request(this, lsm6dso_ois_reg_config[hw_id][REG_CONFIG_OIS]);
}


sns_rc lsm6dso_ois_init(sns_sensor *const this)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  shared_state->hw_idx = 0;
  sns_sensor_uid* suid = &((sns_sensor_uid)OIS_SUID_0);
#if LSM6DSO_DUAL_SENSOR_ENABLED
  shared_state->hw_idx = this->cb->get_registration_index(this);
  suid = (shared_state->hw_idx) ? &((sns_sensor_uid)OIS_SUID_1) : suid;
#endif

  lsm6dso_init_sensor_info(this, suid, LSM6DSO_OIS);
  lsm6dso_publish_def_attributes(this);
  lsm6dso_publish_ois_attributes(this);

  DBG_PRINTF_EX(HIGH, this, "ois sensor init ");
  return SNS_RC_SUCCESS;
}

sns_rc lsm6dso_ois_deinit(sns_sensor *const this)
{
  DBG_PRINTF_EX(HIGH, this,  "OIS: Deinit sensor");
  return SNS_RC_SUCCESS;
}

void lsm6dso_ois_register(sns_register_cb const *register_api)
{
  register_api->init_sensor(sizeof(lsm6dso_state), &lsm6dso_ois_sensor_api,
      &lsm6dso_sensor_instance_api);
}

#endif //LSM6DSO_OIS_ENABLED
