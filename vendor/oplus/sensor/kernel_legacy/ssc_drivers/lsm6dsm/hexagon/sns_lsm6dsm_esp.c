/**
 * @file sns_lsm6dsm_esp.c
 *
 * LSM6DSM Double Tap Sensor implementation.
 *
 * Copyright (c) 2018, STMicroelectronics.
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
#include "sns_types.h"
#include "sns_lsm6dsm_sensor.h"

#if LSM6DSM_ESP_ENABLED
#include "pb_encode.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"

#define PLATFORM_CONFIG_FREE_FALL      "_platform.ff.config"
#define PLATFORM_CONFIG_HIGH_SHOCK     "_platform.hs.config"
#define PLATFORM_CONFIG_ACTIVITY       "_platform.act.config"
#define LSM6DSM_GEN_GROUP(x,group) NAME "_"#x group

enum {
  REG_PLATFORM_CONFIG_FREE_FALL,
  REG_PLATFORM_CONFIG_HIGH_SHOCK,
  REG_PLATFORM_CONFIG_ACTIVITY,
  REG_MAX_CONFIGS,
};

static char lsm6dsm_esp_reg_config[SENSOR_INST_CNT][REG_MAX_CONFIGS][SNS_REGISTRY_MAX_NAME_LEN] = {
  {
    LSM6DSM_GEN_GROUP(0, PLATFORM_CONFIG_FREE_FALL),
    LSM6DSM_GEN_GROUP(0, PLATFORM_CONFIG_HIGH_SHOCK),
    LSM6DSM_GEN_GROUP(0, PLATFORM_CONFIG_ACTIVITY),
  },
#if LSM6DSM_DUAL_SENSOR_ENABLED
  {
    LSM6DSM_GEN_GROUP(1, PLATFORM_CONFIG_FREE_FALL),
    LSM6DSM_GEN_GROUP(1, PLATFORM_CONFIG_HIGH_SHOCK),
    LSM6DSM_GEN_GROUP(1, PLATFORM_CONFIG_ACTIVITY),
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
lsm6dsm_publish_esp_attributes(sns_sensor *const this, lsm6dsm_sensor_type sensor)
{
  lsm6dsm_state *state = (lsm6dsm_state*)this->state->state;
  int i;
#if LSM6DSM_ATTRIBUTE_DISABLED
  lsm6dsm_state *state = (lsm6dsm_state*)this->state->state;
  for(i = 0; i < ARR_SIZE(lsm6dsm_supported_esp_sensors); i++)
    if(lsm6dsm_supported_esp_sensors[i].sensor == sensor)
      break;
  if(i >= ARR_SIZE(lsm6dsm_supported_esp_sensors))
    return SNS_RC_FAILED;
  {
    char const* type = lsm6dsm_supported_esp_sensors[i].name;
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = strlen(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->available;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
  }
#else
  for(i = 0; i < ARR_SIZE(lsm6dsm_supported_esp_sensors); i++)
    if(lsm6dsm_supported_esp_sensors[i].sensor == sensor)
      break;
  if(i >= ARR_SIZE(lsm6dsm_supported_esp_sensors))
    return SNS_RC_FAILED;
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = lsm6dsm_supported_esp_sensors[i].stream_type;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    if(sensor == LSM6DSM_STEP_COUNTER)
    {
      float data[3] = {0};
      state->encoded_event_len =
          pb_get_encoded_size_sensor_stream_event(data, 1);
      sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
      value.has_sint = true;
      value.sint = state->encoded_event_len;
      sns_publish_attribute(
          this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
    }
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = ESP_ACTIVE_CURRENT;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
        values, ARR_SIZE(values), false);
  }

  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const op_mode1[] = LSM6DSM_HIGH_PERF;

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const* proto1 = lsm6dsm_supported_esp_sensors[i].proto;
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = strlen(proto1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }
  {
    char const name[] = NAME;
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    char const* type = lsm6dsm_supported_esp_sensors[i].name;
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = strlen(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    char const vendor[] = VENDOR;
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = vendor, .buf_len = sizeof(vendor) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
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
    value.sint = SNS_VERSION_LSM6DSM;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 6; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, true);
  }
#endif // LSM6DSM_ATTRIBUTE_DISABLED
  return SNS_RC_SUCCESS;
}

static sns_rc lsm6dsm_esp_init(sns_sensor *const this, lsm6dsm_sensor_type sensor)
{
  sns_rc rc = SNS_RC_SUCCESS;

  lsm6dsm_shared_state *shared_state = lsm6dsm_get_shared_state(this);
  shared_state->hw_idx = 0;

  shared_state->hw_idx = this->cb->get_registration_index(this);
  sns_sensor_uid* suid = NULL;
  if(sensor == LSM6DSM_FREE_FALL) {
    suid = &((sns_sensor_uid)FREE_FALL_SUID_0);
#if LSM6DSM_DUAL_SENSOR_ENABLED
    suid = (shared_state->hw_idx) ? &((sns_sensor_uid)FREE_FALL_SUID_1) : suid;
#endif
  }
  else if(sensor == LSM6DSM_HIGH_SHOCK) {
    suid = &((sns_sensor_uid)HIGH_SHOCK_SUID_0);
#if LSM6DSM_DUAL_SENSOR_ENABLED
    suid = (shared_state->hw_idx) ? &((sns_sensor_uid)HIGH_SHOCK_SUID_1) : suid;
#endif
  }
  else if(sensor == LSM6DSM_ACTIVITY) {
    suid = &((sns_sensor_uid)ACTIVITY_SUID_0);
#if LSM6DSM_DUAL_SENSOR_ENABLED
    suid = (shared_state->hw_idx) ? &((sns_sensor_uid)ACTIVITY_SUID_1) : suid;
#endif
  }
  else if(sensor == LSM6DSM_INACTIVITY) {
    suid = &((sns_sensor_uid)INACTIVITY_SUID_0);
#if LSM6DSM_DUAL_SENSOR_ENABLED
    suid = (shared_state->hw_idx) ? &((sns_sensor_uid)INACTIVITY_SUID_1) : suid;
#endif
  }
  else if(sensor == LSM6DSM_STEP_COUNTER) {
    suid = &((sns_sensor_uid)STEP_COUNTER_SUID_0);
#if LSM6DSM_DUAL_SENSOR_ENABLED
    suid = (shared_state->hw_idx) ? &((sns_sensor_uid)STEP_COUNTER_SUID_1) : suid;
#endif
  }

  if(!suid)
    return SNS_RC_FAILED;

  rc = lsm6dsm_publish_esp_attributes(this, sensor);

  DBG_PRINTF_EX(HIGH, this,
      "esp attrbitues published rc=%d sensor=0x%x",rc, sensor);
  if(rc == SNS_RC_SUCCESS) {
    lsm6dsm_init_sensor_info(this, suid, sensor);

    DBG_PRINTF_EX(HIGH, this,
        "esp initializing sensor 0x%x", sensor);
  }
  return rc;
}

static sns_rc lsm6dsm_esp_deinit(sns_sensor *const this, lsm6dsm_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  DBG_PRINTF_EX(HIGH, this, "Deinit sensor: %d", sensor);
  return SNS_RC_SUCCESS;
}

/* See sns_sensor::init */
sns_rc lsm6dsm_free_fall_init(sns_sensor *const this)
{
  return lsm6dsm_esp_init(this, LSM6DSM_FREE_FALL);
}

sns_rc lsm6dsm_free_fall_deinit(sns_sensor *const this)
{
  return lsm6dsm_esp_deinit(this, LSM6DSM_FREE_FALL);
}
/* See sns_sensor::init */
sns_rc lsm6dsm_high_shock_init(sns_sensor *const this)
{
  return lsm6dsm_esp_init(this, LSM6DSM_HIGH_SHOCK);
}

sns_rc lsm6dsm_high_shock_deinit(sns_sensor *const this)
{
  return lsm6dsm_esp_deinit(this, LSM6DSM_HIGH_SHOCK);
}
sns_rc lsm6dsm_activity_init(sns_sensor *const this)
{
  return lsm6dsm_esp_init(this, LSM6DSM_ACTIVITY);
}

sns_rc lsm6dsm_activity_deinit(sns_sensor *const this)
{
  return lsm6dsm_esp_deinit(this, LSM6DSM_ACTIVITY);
}
sns_rc lsm6dsm_inactivity_init(sns_sensor *const this)
{
  return lsm6dsm_esp_init(this, LSM6DSM_INACTIVITY);
}

sns_rc lsm6dsm_inactivity_deinit(sns_sensor *const this)
{
  return lsm6dsm_esp_deinit(this, LSM6DSM_INACTIVITY);
}

/* See sns_sensor::init */
sns_rc lsm6dsm_step_counter_init(sns_sensor *const this)
{
  return lsm6dsm_esp_init(this, LSM6DSM_STEP_COUNTER);
}

sns_rc lsm6dsm_step_counter_deinit(sns_sensor *const this)
{
  return lsm6dsm_esp_deinit(this, LSM6DSM_STEP_COUNTER);
}

void lsm6dsm_esp_register(sns_register_cb const *register_api)
{
  for(int i = 0; i< ARR_SIZE(lsm6dsm_supported_esp_sensors) ; i++) {
    register_api->init_sensor(sizeof(lsm6dsm_state),
                              lsm6dsm_supported_esp_sensors[i].sensor_api,
                              &lsm6dsm_sensor_instance_api);
  }
}

#if !LSM6DSM_REGISTRY_DISABLED
void lsm6dsm_send_esp_registry_requests(sns_sensor *const this, uint8_t hw_id)
{
  lsm6dsm_state *state = (lsm6dsm_state*)this->state->state;
  if(LSM6DSM_FREE_FALL == state->sensor)
  {
    //lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_CONFIG_FREE_FALL]);
    lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_FREE_FALL]);
  }
  else if(LSM6DSM_HIGH_SHOCK == state->sensor)
  {
//    lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_CONFIG_HIGH_SHOCK]);
    lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_HIGH_SHOCK]);
  }
  else if(LSM6DSM_ACTIVITY == state->sensor)
  {
//    lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_CONFIG_HIGH_SHOCK]);
    lsm6dsm_send_registry_request(this, lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_ACTIVITY]);
  }

}
#if LSM6DSM_ESP_ACTIVITY
static bool
lsm6dsm_reg_parse_activity_cfg(sns_registry_data_item *reg_item,
                          struct pb_buffer_arg  *item_name,
                          struct pb_buffer_arg  *item_str_val,
                          void *parsed_buffer)
{
  lsm6dsm_act_registry_cfg *data_ptr = (lsm6dsm_act_registry_cfg *)parsed_buffer;
  UNUSED_VAR(item_str_val);
  if(!reg_item->has_flt)
    return false;
  if(0 == strncmp((char*)item_name->buf,
        "thresh",
        item_name->buf_len)) {
    data_ptr->thresh = reg_item->flt;
  } else if(0 == strncmp((char*)item_name->buf,
        "win",
        item_name->buf_len)) {
    data_ptr->dur = reg_item->flt;
  } else if(0 == strncmp((char*)item_name->buf,
        "inact_timeout",
        item_name->buf_len)) {
    data_ptr->inact_timeout = reg_item->flt;
  }

  return true;
}
#endif
#endif // !LSM6DSM_REGISTRY_DISABLED

#if LSM6DSM_ESP_FREE_FALL
static bool
lsm6dsm_reg_parse_free_fall_cfg(sns_registry_data_item *reg_item,
                          struct pb_buffer_arg  *item_name,
                          struct pb_buffer_arg  *item_str_val,
                          void *parsed_buffer)
{
  lsm6dsm_ff_registry_cfg *data_ptr = (lsm6dsm_ff_registry_cfg *)parsed_buffer;
  UNUSED_VAR(item_str_val);

  if(0 == strncmp((char*)item_name->buf,
        "thresh",
        item_name->buf_len)) {
    data_ptr->thresh = reg_item->flt;
  } else if(0 == strncmp((char*)item_name->buf,
        "win",
        item_name->buf_len)) {
    data_ptr->dur = reg_item->flt;
  }
  return true;
}
#endif

#if (LSM6DSM_ESP_ACTIVITY || LSM6DSM_ESP_FREE_FALL || LSM6DSM_ESP_HIGH_SHOCK)
void lsm6dsm_process_esp_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  lsm6dsm_state *state = (lsm6dsm_state*)this->state->state;
  lsm6dsm_shared_state *shared_state = lsm6dsm_get_shared_state(this);

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
      if(0 == strncmp((char*)group_name.buf,
            lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_FREE_FALL],
            group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .parse_func = lsm6dsm_reg_parse_free_fall_cfg,
              .parsed_buffer = &shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf }
          };

          sns_strlcpy(arg.parse_info[0].group_name,lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_FREE_FALL],sizeof(arg.parse_info[0].group_name));
          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          DBG_PRINTF_EX(HIGH, this,
              "FREE_FALL Threshold(*1000):%d m/s2 FREE_FALL Window(*1000):%d sec ",
              (int)(shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf.thresh*1000),
              (int) (shared_state->inst_cfg.esp_reg_cfg.ff_reg_conf.dur*1000));
        }
      }
      else if(0 == strncmp((char*)group_name.buf,
            lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_HIGH_SHOCK],
            group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .parse_func = sns_registry_parse_md_cfg,
              .parsed_buffer = &shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf }
          };

          sns_strlcpy(arg.parse_info[0].group_name,lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_HIGH_SHOCK],sizeof(arg.parse_info[0].group_name));
          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          DBG_PRINTF_EX(HIGH, this,
              "High-Shock Threshold(*1000):%d m/s2 Window(*1000):%d sec Disable :%d",
              (int)(shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf.thresh*1000),
              (int) (shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf.win*1000),
              (int) (shared_state->inst_cfg.esp_reg_cfg.hs_reg_conf.disable));
        }
      }
     else if(0 == strncmp((char*)group_name.buf,
            lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_ACTIVITY],
            group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .parse_func = lsm6dsm_reg_parse_activity_cfg,
              .parsed_buffer = &shared_state->inst_cfg.esp_reg_cfg.act_reg_conf }
          };

          sns_strlcpy(arg.parse_info[0].group_name,lsm6dsm_esp_reg_config[hw_id][REG_PLATFORM_CONFIG_ACTIVITY],sizeof(arg.parse_info[0].group_name));
          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          DBG_PRINTF_EX(HIGH, this,
              "Activity Threshold(*1000):%d m/s2 Window(*1000):%d sec timeout(*1000) :%d sec",
              (int)(shared_state->inst_cfg.esp_reg_cfg.act_reg_conf.thresh*1000),
              (int) (shared_state->inst_cfg.esp_reg_cfg.act_reg_conf.dur*1000),
              (int) (shared_state->inst_cfg.esp_reg_cfg.act_reg_conf.inact_timeout*1000));
        }
      }
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
#else
void lsm6dsm_process_esp_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  UNUSED_VAR(this);
  UNUSED_VAR(event);
}
#endif

#if LSM6DSM_ESP_STEP_COUNTER
static size_t lsm6dsm_get_encoded_log_len(uint16_t sensor)
{
  size_t len= 0;
  if(sensor == LSM6DSM_STEP_COUNTER)
  {
    uint64_t buffer[10];
    pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
    sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
    uint8_t arr_index = 0;
    float diag_temp[1];
    pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = 1,
      .arr_index = &arr_index};
    batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
    batch_sample.sample.arg = &arg;

    if(pb_encode_tag(&stream, PB_WT_STRING,
          sns_diag_sensor_state_raw_sample_tag))
    {
      if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
            &batch_sample))
      {
        len = stream.bytes_written;
      }
    }
  }
  return len;
}

void lsm6dsm_esp_handle_reset_device(sns_sensor_instance *const instance, sns_sensor_state const *sstate)
{
  lsm6dsm_instance_state *inst_state = (lsm6dsm_instance_state*)instance->state->state;
  uint16_t sc_reg_value = read_step_count(instance);
  if(sstate != NULL)
  {
    lsm6dsm_shared_state *shared_state = lsm6dsm_get_shared_state_from_state(sstate);
    if(shared_state->esp_shared_info.stepCount_base >= sc_reg_value)
    {
      inst_state->esp_info.sc_info.stepCount_base = shared_state->esp_shared_info.stepCount_base - sc_reg_value;
      inst_state->esp_info.sc_info.stepCount_prev = sc_reg_value;
#if LSM6DSM_OEM_FACTORY_CONFIG
      if(shared_state->esp_shared_info.oem_config_on)
      {
        lsm6dsm_oem_factory_test_config(instance, false);
      }
#endif
    }
    else
    {
      DBG_INST_PRINTF(ERROR, instance, "sc record err");
    }
  }
  else if(sc_reg_value == 0)
  {
    inst_state->esp_info.sc_info.stepCount_base += inst_state->esp_info.sc_info.stepCount_prev;
    inst_state->esp_info.sc_info.stepCount_prev = 0;
  }
}
#else
static size_t lsm6dsm_get_encoded_log_len(uint16_t sensor)
{
  UNUSED_VAR(sensor);
  return 0;
}
void lsm6dsm_esp_handle_reset_device(sns_sensor_instance *const instance, sns_sensor_state const *sstate)
{
  UNUSED_VAR(instance);
  UNUSED_VAR(sstate);
}
#endif

void init_esp_shared_state(sns_sensor *const this)
{
  lsm6dsm_shared_state *shared_state = lsm6dsm_get_shared_state(this);
  shared_state->esp_shared_info.stepCount_base = 0;
#if LSM6DSM_OEM_FACTORY_CONFIG
  shared_state->esp_shared_info.oem_config_on = false;
#endif
}

void lsm6dsm_init_esp_instance(sns_sensor_instance *instance, sns_sensor_state const *this)
{
  UNUSED_VAR(this);
  lsm6dsm_instance_state *inst_state =
    (lsm6dsm_instance_state*)instance->state->state;

  float data[1];
  inst_state->esp_info.ff_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);
  inst_state->esp_info.hs_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);
  inst_state->esp_info.act_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);
  inst_state->esp_info.sc_info.encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 1);

  inst_state->esp_info.sc_info.encoded_raw_log_len = lsm6dsm_get_encoded_log_len(LSM6DSM_STEP_COUNTER);

  inst_state->esp_info.desired_sensors = 0;
  inst_state->esp_info.update_int = 0;
  inst_state->esp_info.hs_info.config_md.thresh = inst_state->md_info.md_config.thresh;
  inst_state->esp_info.hs_info.config_md.win = inst_state->md_info.md_config.win;
  inst_state->esp_info.hs_info.timer_hs_data_stream = NULL;
  inst_state->esp_info.act_info.timer_inact_data_stream = NULL;
#if LSM6DSM_DUAL_SENSOR_ENABLED
  if(inst_state->hw_idx) {
    sns_memscpy(&inst_state->esp_info.suid[FF_INDX],
        sizeof(inst_state->esp_info.suid[FF_INDX]),
        &((sns_sensor_uid)FREE_FALL_SUID_1),
        sizeof(inst_state->esp_info.suid[FF_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[HS_INDX],
        sizeof(inst_state->esp_info.suid[HS_INDX]),
        &((sns_sensor_uid)HIGH_SHOCK_SUID_1),
        sizeof(inst_state->esp_info.suid[HS_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[ACT_INDX],
        sizeof(inst_state->esp_info.suid[ACT_INDX]),
        &((sns_sensor_uid)ACTIVITY_SUID_1),
        sizeof(inst_state->esp_info.suid[ACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[INACT_INDX],
        sizeof(inst_state->esp_info.suid[INACT_INDX]),
        &((sns_sensor_uid)INACTIVITY_SUID_1),
        sizeof(inst_state->esp_info.suid[INACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[SC_INDX],
        sizeof(inst_state->esp_info.suid[SC_INDX]),
        &((sns_sensor_uid)STEP_COUNTER_SUID_1),
        sizeof(inst_state->esp_info.suid[SC_INDX]));
  }
  else
#endif
  {
    sns_memscpy(&inst_state->esp_info.suid[FF_INDX],
        sizeof(inst_state->esp_info.suid[FF_INDX]),
        &((sns_sensor_uid)FREE_FALL_SUID_0),
        sizeof(inst_state->esp_info.suid[FF_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[HS_INDX],
        sizeof(inst_state->esp_info.suid[HS_INDX]),
        &((sns_sensor_uid)HIGH_SHOCK_SUID_0),
        sizeof(inst_state->esp_info.suid[HS_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[ACT_INDX],
        sizeof(inst_state->esp_info.suid[ACT_INDX]),
        &((sns_sensor_uid)ACTIVITY_SUID_0),
        sizeof(inst_state->esp_info.suid[ACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[INACT_INDX],
        sizeof(inst_state->esp_info.suid[INACT_INDX]),
        &((sns_sensor_uid)INACTIVITY_SUID_0),
        sizeof(inst_state->esp_info.suid[INACT_INDX]));
    sns_memscpy(&inst_state->esp_info.suid[SC_INDX],
        sizeof(inst_state->esp_info.suid[SC_INDX]),
        &((sns_sensor_uid)STEP_COUNTER_SUID_0),
        sizeof(inst_state->esp_info.suid[SC_INDX]));
  }
}

#else

sns_rc lsm6dsm_free_fall_init(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_high_shock_init(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_activity_init(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_inactivity_init(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_free_fall_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_high_shock_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_activity_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

sns_rc lsm6dsm_inactivity_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_NOT_SUPPORTED;
}

void lsm6dsm_esp_register(sns_register_cb const *register_api)
{
  UNUSED_VAR(register_api);
}

void lsm6dsm_send_esp_registry_requests(sns_sensor *const this, uint8_t hw_id)
{
  UNUSED_VAR(this);
  UNUSED_VAR(hw_id);
}

void lsm6dsm_process_esp_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  UNUSED_VAR(this);
  UNUSED_VAR(event);
}

#endif

