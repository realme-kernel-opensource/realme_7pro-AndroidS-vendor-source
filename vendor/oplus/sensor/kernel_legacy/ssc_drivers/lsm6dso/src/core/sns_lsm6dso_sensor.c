/**
 * @file sns_lsm6dso_sensor.c
 *
 * Common implementation for LSM6DSO Sensors.
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
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"

#include "sns_lsm6dso_sensor.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"
#ifdef LSM6DSO_GET_PARAMETER_FROM_SMEM
#include "oppo_sensor.h"
#endif

#ifndef OPLUS_SENSOR_LITE_DRIVER
#include "sns_devinfo_utils.h"
#endif

#define CONFIG_ACCEL            ".accel.config"
#define CONFIG_GYRO             ".gyro.config"
#define CONFIG_TEMP             ".temp.config"
#define CONFIG_MD               ".md.config"
#define PLATFORM_CONFIG         "_platform.config"
#define PLATFORM_PLACEMENT      "_platform.placement"
#define PLATFORM_ORIENT         "_platform.orient"
#define PLATFORM_FAC_CAL_ACCEL  "_platform.accel.fac_cal"
#define PLATFORM_FAC_CAL_GYRO  "_platform.gyro.fac_cal"
#define PLATFORM_FAC_CAL_TEMP   "_platform.temp.fac_cal"
#define PLATFORM_CONFIG_MD      "_platform.md.config"
#define REG_GROUP_ACCEL_THERMAL_SCALE "dynamic.fac_cal.accel.thermal.scale"
#define REG_GROUP_GYRO_THERMAL_SCALE  "dynamic.fac_cal.gyro.thermal.scale"

#define LSM6DSO_GEN_GROUP(x,group) NAME "_"#x group
#ifndef OPLUS_SENSOR_LITE_DRIVER
#define LSM6DSO_FAC_CAL_BIAS 	".bias"
#endif
#define MAX_DEP_LENGTH 30

// temp structure for pb arg
typedef struct pb_arg_reg_group_arg
{
  sns_sensor* this;
  const char*          name;
  lsm6dso_sensor_type sensor;
  uint32_t version;
}pb_arg_reg_group_arg;

#if !LSM6DSO_REGISTRY_DISABLED
enum {
  REG_CONFIG_ACCEL,
  REG_CONFIG_GYRO,
  REG_CONFIG_TEMP,
  REG_CONFIG_MD,
  REG_PLATFORM_CONFIG,
  REG_PLATFORM_PLACEMENT,
  REG_PLATFORM_ORIENT,
  REG_PLATFORM_FAC_CAL_ACCEL,
  REG_PLATFORM_FAC_CAL_GYRO,
  REG_PLATFORM_FAC_CAL_TEMP,
  REG_PLATFORM_CONFIG_MD,
  REG_MAX_CONFIGS,
};

static char lsm6dso_reg_config[SENSOR_INST_CNT][REG_MAX_CONFIGS][40] = {
  {
    LSM6DSO_GEN_GROUP(0, CONFIG_ACCEL),
    LSM6DSO_GEN_GROUP(0, CONFIG_GYRO),
    LSM6DSO_GEN_GROUP(0, CONFIG_TEMP),
    LSM6DSO_GEN_GROUP(0, CONFIG_MD),
    LSM6DSO_GEN_GROUP(0, PLATFORM_CONFIG),
    LSM6DSO_GEN_GROUP(0, PLATFORM_PLACEMENT),
    LSM6DSO_GEN_GROUP(0, PLATFORM_ORIENT),
    LSM6DSO_GEN_GROUP(0, PLATFORM_FAC_CAL_ACCEL),
    LSM6DSO_GEN_GROUP(0, PLATFORM_FAC_CAL_GYRO),
    LSM6DSO_GEN_GROUP(0, PLATFORM_FAC_CAL_TEMP),
    LSM6DSO_GEN_GROUP(0, PLATFORM_CONFIG_MD),
  },
#if LSM6DSO_DUAL_SENSOR_ENABLED
  {
    LSM6DSO_GEN_GROUP(1, CONFIG_ACCEL),
    LSM6DSO_GEN_GROUP(1, CONFIG_GYRO),
    LSM6DSO_GEN_GROUP(1, CONFIG_TEMP),
    LSM6DSO_GEN_GROUP(1, CONFIG_MD),
    LSM6DSO_GEN_GROUP(1, PLATFORM_CONFIG),
    LSM6DSO_GEN_GROUP(1, PLATFORM_PLACEMENT),
    LSM6DSO_GEN_GROUP(1, PLATFORM_ORIENT),
    LSM6DSO_GEN_GROUP(1, PLATFORM_FAC_CAL_ACCEL),
    LSM6DSO_GEN_GROUP(1, PLATFORM_FAC_CAL_GYRO),
    LSM6DSO_GEN_GROUP(1, PLATFORM_FAC_CAL_TEMP),
    LSM6DSO_GEN_GROUP(1, PLATFORM_CONFIG_MD),
  }
#endif
};
#endif

extern const odr_reg_map lsm6dso_odr_map[];
extern const uint32_t lsm6dso_odr_map_len;

static char def_dependency[][MAX_DEP_LENGTH] =  {
  "interrupt", "async_com_port", "timer",
#if LSM6DSO_DAE_ENABLED
  "data_acquisition_engine",
#endif
#if !LSM6DSO_REGISTRY_DISABLED
  // Only depend registry if registry support is enabled
   "registry"
#endif
};

#if !LSM6DSO_ATTRIBUTE_DISABLED
static const char name[] = NAME;
static const char vendor[] = VENDOR;
static const uint32_t version = SNS_VERSION_LSM6DSO; // major[31:16].minor[15:0]

static const uint32_t max_fifo_depth = LSM6DSO_MAX_FIFO; // samples

static const sns_std_sensor_stream_type stream_type = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
static const bool is_dynamic = false;
static const sns_std_sensor_rigid_body_type rigid_body = SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
static const uint32_t hardware_id = 0;
static const bool supports_dri = true;
static const bool supports_sync_stream = true;

void lsm6dso_publish_def_attributes(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = vendor, .buf_len = sizeof(vendor) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = version;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = max_fifo_depth;
    if(state->sensor == LSM6DSO_GYRO && (value.sint > (LSM6DSO_HW_MAX_FIFO >> 1)))
    {
      value.sint = max_fifo_depth >> 1; // Always has to share with Accel
    }
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    char const op_mode1[] = LSM6DSO_LPM;
    char const op_mode2[] = LSM6DSO_NORMAL;
    char const op_mode3[] = LSM6DSO_HIGH_PERF;
    char const op_mode4[] = LSM6DSO_OFF;
	
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    values[1].str.funcs.encode = pb_encode_string_cb;
    values[1].str.arg = &((pb_buffer_arg)
        { .buf = op_mode2, .buf_len = sizeof(op_mode2) });
    values[2].str.funcs.encode = pb_encode_string_cb;
    values[2].str.arg = &((pb_buffer_arg)
        { .buf = op_mode3, .buf_len = sizeof(op_mode3) });
    values[3].str.funcs.encode = pb_encode_string_cb;
    values[3].str.arg = &((pb_buffer_arg)
        { .buf = op_mode4, .buf_len = sizeof(op_mode4) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }
  {
    float data[3] = {0};
    state->encoded_event_len =
        pb_get_encoded_size_sensor_stream_event(data, 3);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = stream_type;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = is_dynamic;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
  }
{
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = rigid_body;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = 0;
    values[1].has_flt = true;
    values[1].flt = 0;
    values[2].has_flt = true;
    values[2].flt = 0;
    values[3].has_flt = true;
    values[3].flt = 0;
    values[4].has_flt = true;
    values[4].flt = 0;
    values[5].has_flt = true;
    values[5].flt = 0;
    values[6].has_flt = true;
    values[6].flt = 0;
    values[7].has_flt = true;
    values[7].flt = 0;
    values[8].has_flt = true;
    values[8].flt = 0;
    values[9].has_flt = true;
    values[9].flt = 0;
    values[10].has_flt = true;
    values[10].flt = 0;
    values[11].has_flt = true;
    values[11].flt = 0;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = hardware_id;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = supports_dri;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = supports_sync_stream;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
#if LSM6DSO_PASSIVE_SENSOR_SUPPORT
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PASSIVE_REQUEST, &value, 1, false);
  }
#endif  
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->available;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
  }
}
#endif
/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void publish_registry_attributes(
  sns_sensor *const this,
  lsm6dso_shared_state *shared_state)
{
#if !LSM6DSO_ATTRIBUTE_DISABLED
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->is_dri;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->supports_sync_stream;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->hardware_id;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    for(uint8_t i = 0; i < 12; i++)
    {
      values[i].has_flt = true;
      values[i].flt = shared_state->placement[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = shared_state->rigid_body_type;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }

  if(state->sensor == LSM6DSO_ACCEL ||
     state->sensor == LSM6DSO_GYRO)
  {
    /** Only accel and gyro use registry information for min and max ODRs */
    {
      int i, j;
      int num_odrs = lsm6dso_odr_map_len - 1;
      sns_std_attr_value_data rates[] = {
        SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
        SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};

      for(i=0, j=shared_state->inst_cfg.min_odr_idx;
          i<ARR_SIZE(rates) && j<num_odrs && j<=shared_state->inst_cfg.max_odr_idx;
          i++, j++)
      {
        rates[i].has_flt = true;
        rates[i].flt = lsm6dso_odr_map[j].odr;
      }
      sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES, rates, i, false);
    }

    /** Only accel and gyro use registry information for selected resolution. */
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_flt = true;
    value.flt = (state->sensor == LSM6DSO_ACCEL) ?
       lsm6dso_accel_resolutions[shared_state->inst_cfg.accel_resolution_idx] * ACC_RES_CONVERSION:
       lsm6dso_gyro_resolutions[shared_state->inst_cfg.gyro_resolution_idx] * GYRO_CONVERSION;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);

    /** Only accel and gyro use registry information for selected range. */
    sns_std_attr_value_data values[] = {SNS_ATTR};
    sns_std_attr_value_data rangeMinMax[] = {SNS_ATTR, SNS_ATTR};
    rangeMinMax[0].has_flt = true;
    rangeMinMax[1].has_flt = true;
    if(state->sensor == LSM6DSO_ACCEL) {
      rangeMinMax[0].flt = lsm6dso_accel_range_min[shared_state->inst_cfg.accel_resolution_idx] * ACC_CONVERSION;
      rangeMinMax[1].flt = lsm6dso_accel_range_max[shared_state->inst_cfg.accel_resolution_idx] * ACC_CONVERSION;
    } else {
      rangeMinMax[0].flt = lsm6dso_gyro_range_min[shared_state->inst_cfg.gyro_resolution_idx] * GYRO_CONVERSION;
      rangeMinMax[1].flt = lsm6dso_gyro_range_max[shared_state->inst_cfg.gyro_resolution_idx] * GYRO_CONVERSION;
    }
    values[0].has_subtype = true;
    values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[0].subtype.values.arg =
      &((pb_buffer_arg){ .buf = rangeMinMax, .buf_len = ARR_SIZE(rangeMinMax) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0], ARR_SIZE(values), true);
  }
#else
  UNUSED_VAR(this);
  UNUSED_VAR(shared_state);
#endif
}

static void publish_available(sns_sensor *const this)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
  value.has_boolean = true;
  value.boolean = true;
  sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
  state->available = true;
}

#if !LSM6DSO_REGISTRY_DISABLED
void lsm6dso_send_registry_request(sns_sensor *const this, char *reg_group_name)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  uint8_t buffer[100];
  int32_t encoded_len;

  sns_registry_read_req read_request;
  pb_buffer_arg data = (pb_buffer_arg){
    .buf = reg_group_name,
    .buf_len = (strlen(reg_group_name) + 1) };

  read_request.name.arg = &data;
  read_request.name.funcs.encode = pb_encode_string_cb;

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                  &read_request, sns_registry_read_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer,
      .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ };
    sns_rc rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
    if(SNS_RC_SUCCESS == rc)
    {
      state->outstanding_reg_requests++;
    }
  }
  else
  {
    SNS_PRINTF(ERROR, this, "send_reg_req: group not sent");
  }
}

static void send_registry_requests(sns_sensor *const this, uint8_t hw_id)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  if(LSM6DSO_ACCEL == state->sensor)
  {
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_CONFIG]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_PLACEMENT]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_ORIENT]);

    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_CONFIG_ACCEL]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_ACCEL]);
    lsm6dso_send_registry_request(this, REG_GROUP_ACCEL_THERMAL_SCALE);
  }
  else if(LSM6DSO_GYRO == state->sensor)
  {
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_CONFIG_GYRO]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_GYRO]);
    lsm6dso_send_registry_request(this, REG_GROUP_GYRO_THERMAL_SCALE);
  }
  else if(LSM6DSO_SENSOR_TEMP == state->sensor)
  {
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_CONFIG_TEMP]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_TEMP]);
  }
  else if(LSM6DSO_MOTION_DETECT == state->sensor)
  {
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_CONFIG_MD]);
    lsm6dso_send_registry_request(this, lsm6dso_reg_config[hw_id][REG_PLATFORM_CONFIG_MD]);
  }
  else if(LSM6DSO_IS_ESP_SENSOR(state->sensor))
  {
    lsm6dso_send_esp_registry_requests(this, hw_id);
  }
  else if(LSM6DSO_IS_OIS_SENSOR(state->sensor))
  {
    lsm6dso_send_ois_registry_requests(this, hw_id);
  }
}

static void process_registry_suid(sns_sensor *const this)
{
  sns_sensor *sensor;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_svc  =
    (sns_stream_service*) service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);

  for(sensor = this->cb->get_library_sensor(this, true);
      NULL != sensor;
      sensor = this->cb->get_library_sensor(this, false))
  {
    lsm6dso_state *state = (lsm6dso_state*)sensor->state->state;

    stream_svc->api->create_sensor_stream(stream_svc, sensor,
                                          shared_state->inst_cfg.reg_suid,
                                          &state->reg_data_stream);
    if(NULL != state->reg_data_stream)
    {
      send_registry_requests(sensor, shared_state->hw_idx);

      DBG_PRINTF_EX(HIGH, sensor, "process_registry_suid: sensor=%u reg_stream=%x #req=%u",
                 state->sensor, state->reg_data_stream, state->outstanding_reg_requests);
    }
    else
    {
      SNS_PRINTF(ERROR, sensor, "Failed to create registry stream");
    }
  }
}
#endif

static void sensor_save_registry_pf_cfg(
  sns_sensor *const this,
  sns_registry_phy_sensor_pf_cfg const * phy_sensor_pf_cfg)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);

  shared_state->rigid_body_type = phy_sensor_pf_cfg->rigid_body_type;

  {
    lsm6dso_instance_config *inst_cfg = &shared_state->inst_cfg;
    inst_cfg->min_odr_idx = (LSM6DSO_ACCEL_ODR26 >> 4);
    inst_cfg->max_odr_idx = (LSM6DSO_ACCEL_ODR416 >> 4); //QC - limiting to 416 Hz
#if LSM6DSO_ODR_REGISTRY_FEATURE_ENABLE
    if(phy_sensor_pf_cfg->max_odr != 0 &&
       phy_sensor_pf_cfg->max_odr >= phy_sensor_pf_cfg->min_odr &&
       phy_sensor_pf_cfg->max_odr < (unsigned int)lsm6dso_odr_map[lsm6dso_odr_map_len-1].odr)
    {
      inst_cfg->min_odr_idx = (LSM6DSO_ACCEL_ODR13 >> 4);
      while(phy_sensor_pf_cfg->min_odr >
            (unsigned int)lsm6dso_odr_map[inst_cfg->min_odr_idx].odr)
      {
        inst_cfg->min_odr_idx++;
      }
      DBG_PRINTF_EX(MED, this, "min_odr=%u --> idx=%u odr=%u",
                    phy_sensor_pf_cfg->min_odr, inst_cfg->min_odr_idx,
                    (unsigned int)lsm6dso_odr_map[inst_cfg->min_odr_idx].odr);

      inst_cfg->max_odr_idx = (LSM6DSO_ACCEL_ODR6656 >> 4);
      while(phy_sensor_pf_cfg->max_odr <
            (unsigned int)lsm6dso_odr_map[inst_cfg->max_odr_idx].odr &&
            inst_cfg->max_odr_idx > inst_cfg->min_odr_idx)
      {
        inst_cfg->max_odr_idx--;
      }
      DBG_PRINTF_EX(MED, this, "max_odr=%u --> idx=%u odr=%u",
                    phy_sensor_pf_cfg->max_odr, inst_cfg->max_odr_idx,
                    (unsigned int)lsm6dso_odr_map[inst_cfg->max_odr_idx].odr);
    }
#endif
  }
  {
    lsm6dso_com_port_info *com_port        = &shared_state->inst_cfg.com_port_info;
    com_port->com_config.bus_type          = phy_sensor_pf_cfg->bus_type;
    com_port->com_config.bus_instance      = phy_sensor_pf_cfg->bus_instance;
    com_port->com_config.slave_control     = phy_sensor_pf_cfg->slave_config;
    com_port->i2c_address                  = phy_sensor_pf_cfg->slave_config;
    com_port->com_config.min_bus_speed_KHz = phy_sensor_pf_cfg->min_bus_speed_khz;
    com_port->com_config.max_bus_speed_KHz = phy_sensor_pf_cfg->max_bus_speed_khz;
    com_port->com_config.reg_addr_type     = phy_sensor_pf_cfg->reg_addr_type;
    DBG_PRINTF_EX(MED, this, "min_bus_speed_KHz:%d max_bus_speed_KHz:%d reg_addr_type:%d",
               com_port->com_config.min_bus_speed_KHz,
               com_port->com_config.max_bus_speed_KHz,
               com_port->com_config.reg_addr_type);
#if LSM6DSO_USE_I3C
    // --- if I3C mode, set up the com port to always use the I3C address
    com_port->i3c_address                  = phy_sensor_pf_cfg->i3c_address;
    if(com_port->com_config.bus_type == SNS_BUS_I3C_SDR ||
       com_port->com_config.bus_type == SNS_BUS_I3C_HDR_DDR )
    {
      com_port->com_config.slave_control   = com_port->i3c_address;
    }
#endif
  }
  {
    sns_interrupt_req *irq_config        = &shared_state->inst_cfg.irq_config;
    sns_interrupt_req *irq2_config       = &shared_state->inst_cfg.irq2_config;
    // QC - might be better to add a second interrupt number to registry
    // rather than overloading dri_irq_num
    irq_config->interrupt_num            = (phy_sensor_pf_cfg->dri_irq_num & 0xFFFF);
    irq_config->interrupt_pull_type      = phy_sensor_pf_cfg->irq_pull_type;
    irq_config->is_chip_pin              = phy_sensor_pf_cfg->irq_is_chip_pin;
    irq_config->interrupt_drive_strength = phy_sensor_pf_cfg->irq_drive_strength;
    irq_config->interrupt_trigger_type   = phy_sensor_pf_cfg->irq_trigger_type;
    irq2_config->interrupt_num = (phy_sensor_pf_cfg->dri_irq_num>>16);
    if(irq2_config->interrupt_num)
    {
      shared_state->irq2_enabled = true;
      irq2_config->interrupt_pull_type = phy_sensor_pf_cfg->irq_pull_type;
      irq2_config->is_chip_pin = phy_sensor_pf_cfg->irq_is_chip_pin;
      irq2_config->interrupt_drive_strength = phy_sensor_pf_cfg->irq_drive_strength;
      irq2_config->interrupt_trigger_type = phy_sensor_pf_cfg->irq_trigger_type;
    }
    else
    {
      shared_state->irq2_enabled = false;
    }
    DBG_PRINTF_EX(MED, this, "interrupt_num:%d pull_type:%d is_chip_pin:%d",
               irq_config->interrupt_num,
               irq_config->interrupt_pull_type,
               irq_config->is_chip_pin);
    DBG_PRINTF_EX(MED, this, "drive_strength:%d trigger_type:%d",
               irq_config->interrupt_drive_strength,
               irq_config->interrupt_trigger_type);
    if(shared_state->irq2_enabled)
    {
      DBG_PRINTF_EX(MED, this, "interrupt2_num:%d", irq2_config->interrupt_num);
    }
  }
  {
#if !LSM6DSO_POWERRAIL_DISABLED
    shared_state->rail_config.num_of_rails = phy_sensor_pf_cfg->num_rail;
    shared_state->registry_rail_on_state = phy_sensor_pf_cfg->rail_on_state;

    sns_strlcpy(shared_state->rail_config.rails[0].name,
                phy_sensor_pf_cfg->vddio_rail,
                sizeof(shared_state->rail_config.rails[0].name));
    sns_strlcpy(shared_state->rail_config.rails[1].name,
                phy_sensor_pf_cfg->vdd_rail,
                sizeof(shared_state->rail_config.rails[1].name));

    /**---------------------Register Power Rails --------------------------*/
    if(NULL == shared_state->pwr_rail_service)
    {
      shared_state->rail_config.rail_vote = SNS_RAIL_OFF;

      shared_state->pwr_rail_service =
        (sns_pwr_rail_service*)service_mgr->get_service(service_mgr,
                                                        SNS_POWER_RAIL_SERVICE);

      shared_state->pwr_rail_service->api->
        sns_register_power_rails(shared_state->pwr_rail_service,
                                 &shared_state->rail_config);
    }
#endif
  }
}

#if !LSM6DSO_REGISTRY_DISABLED
static bool lsm6dso_fac_cal_mean_temp_reader(
    sns_registry_data_item *reg_item,
    pb_buffer_arg *item_name,
    pb_buffer_arg *item_str_val,
    void *parsed_buffer)
{
  if (!reg_item || !item_name || !item_str_val || !parsed_buffer) {
    return false;
  }

  if (strncmp((char *)item_name->buf, "mean_temp", item_name->buf_len) == 0 && reg_item->has_flt)
  {
    *(float *)parsed_buffer = reg_item->flt;
    return true;
  }

  return false;
}

static bool lsm6dso_decode_fac_cal_registry_data(
    sns_sensor *const this,
    pb_istream_t* stream,
    pb_buffer_arg* group_name,
    sns_registry_read_event* read_event,
    sns_lsm6dso_registry_cfg *cal)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  bool rv = false;
  float fac_cal_scale[3] = {0.0f, 0.0f, 0.0f};
  uint32_t version = 0;
  {
    uint8_t bias_arr_index = 0;

    pb_float_arr_arg bias_arr_arg = {
      .arr = cal->fac_cal_bias,
      .arr_index = &bias_arr_index,
      .arr_len = TRIAXIS_NUM
    };

    sns_registry_decode_arg arg = {
      .item_group_name = group_name,
      .parse_info_len = 3,
      .parse_info[0] = {
        .group_name = "bias",
        .parse_func = sns_registry_parse_float_arr,
        .parsed_buffer = &bias_arr_arg
      },
      .parse_info[1] = {
        .group_name = "fac_cal",
        .parse_func = lsm6dso_fac_cal_mean_temp_reader,
        .parsed_buffer = &cal->mean_temp,
      },
      .parse_info[2] = {
        .group_name = "corr_mat",
        .parse_func = sns_registry_parse_corr_matrix_3,
        .parsed_buffer = &cal->fac_cal_corr_mat
      }
    };

    read_event->data.items.funcs.decode = &sns_registry_item_decode_cb;
    read_event->data.items.arg = &arg;

    rv = pb_decode(stream, sns_registry_read_event_fields, read_event);
    version = arg.version;
  }

  if(rv)
  {
    cal->ts = sns_get_system_time();
    cal->sensor_type = state->sensor;
    cal->registry_persist_version = version;
    DBG_PRINTF_EX(MED, this, "Version: %d, Sensor: %d",version, state->sensor);
    if(fac_cal_scale[0] != 0.0f)
    {
      cal->fac_cal_corr_mat.e00 = fac_cal_scale[0];
      cal->fac_cal_corr_mat.e11 = fac_cal_scale[1];
      cal->fac_cal_corr_mat.e22 = fac_cal_scale[2];
    }

    DBG_PRINTF_EX(MED, this, "Fac Cal Corr Matrix e00:%d e01:%d e02:%d",
        (int)cal->fac_cal_corr_mat.e00, (int)cal->fac_cal_corr_mat.e01,
        (int)cal->fac_cal_corr_mat.e02);
    DBG_PRINTF_EX(MED, this, "Fac Cal Corr Matrix e10:%d e11:%d e12:%d",
        (int)cal->fac_cal_corr_mat.e10, (int)cal->fac_cal_corr_mat.e11,
        (int)cal->fac_cal_corr_mat.e12);
    DBG_PRINTF_EX(MED, this, "Fac Cal Corr Matrix e20:%d e21:%d e22:%d",
        (int)cal->fac_cal_corr_mat.e20, (int)cal->fac_cal_corr_mat.e21,
        (int)cal->fac_cal_corr_mat.e22);
    DBG_PRINTF_EX(MED, this, "Fac Cal Bias x:%d y:%d z:%d",
        cal->fac_cal_bias[0], cal->fac_cal_bias[1],
        cal->fac_cal_bias[2]);
    DBG_PRINTF(MED, this, "fac_cal.mean_temp = %d.%d",
        (int)cal->mean_temp, (int)(cal->mean_temp * 100) % 100);
  }
  return rv;
}

static bool lsm6dso_decode_sensor_config_registry_data(
    sns_sensor *const this,
    pb_istream_t* stream,
    pb_buffer_arg* group_name,
    sns_registry_read_event* read_event)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  bool rv = false;
  sns_registry_phy_sensor_cfg sensor_cfg = {0,0,0,0};
  sns_registry_decode_arg arg = {
    .item_group_name = group_name,
    .parse_info_len = 1,
    .parse_info[0] = {
      .group_name = "config",
      .parse_func = sns_registry_parse_phy_sensor_cfg,
      .parsed_buffer = &sensor_cfg
    }
  };

  read_event->data.items.funcs.decode = &sns_registry_item_decode_cb;
  read_event->data.items.arg = &arg;

  rv = pb_decode(stream, sns_registry_read_event_fields, read_event);

  if(rv)
  {
    state->is_dri               = sensor_cfg.is_dri;
    state->hardware_id          = sensor_cfg.hw_id;
    state->supports_sync_stream = sensor_cfg.sync_stream;
    if(state->sensor == LSM6DSO_ACCEL) {
      shared_state->inst_cfg.ag_stream_mode = (state->supports_sync_stream) ? S4S_SYNC : state->is_dri;
      shared_state->inst_cfg.accel_resolution_idx = sensor_cfg.res_idx;
    } else if(state->sensor == LSM6DSO_GYRO) {
      shared_state->inst_cfg.gyro_resolution_idx = sensor_cfg.res_idx;
    }
    else if(LSM6DSO_IS_OIS_SENSOR(state->sensor)) {
      lsm6dso_update_ois_resolution_idx(this, sensor_cfg.res_idx);
    }
  }
  DBG_PRINTF_EX(MED, this, "resolution_idx:%d, supports_sync_stream:%d ",
                sensor_cfg.res_idx, state->supports_sync_stream);
  return rv;
}

static void process_registry_event(sns_sensor *const this, sns_sensor_event *event)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  uint8_t hw_id = shared_state->hw_idx;

  pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);

  if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
  {
    sns_registry_read_event read_event = sns_registry_read_event_init_default;
    pb_buffer_arg group_name = {0,0};
    read_event.name.arg = &group_name;
    read_event.name.funcs.decode = pb_decode_string_cb;

    if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
    {
      SNS_PRINTF(ERROR, this, "Error decoding registry event");
    }
    else
    {
      bool rv = false;
      stream = pb_istream_from_buffer((void*)event->event, event->event_len);

      if(0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_CONFIG_ACCEL],
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_CONFIG_GYRO],
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_CONFIG_TEMP],
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_CONFIG_MD],
                      group_name.buf_len))
      {
        rv = lsm6dso_decode_sensor_config_registry_data(this, &stream, &group_name, &read_event);
      }
      else if(0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_PLATFORM_CONFIG],
                           group_name.buf_len))
      {
        sns_registry_phy_sensor_pf_cfg phy_sensor_pf_cfg;
        memset(&phy_sensor_pf_cfg, 0, sizeof(phy_sensor_pf_cfg));
        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
              .group_name = "config",
              .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
              .parsed_buffer = &phy_sensor_pf_cfg
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

        if(rv)
        {
          sensor_save_registry_pf_cfg(this, &phy_sensor_pf_cfg);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_PLATFORM_PLACEMENT],
                         group_name.buf_len))
      {
        uint8_t arr_index = 0;
        pb_float_arr_arg arr_arg = {
          .arr = shared_state->placement,
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

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
      }
      else if(0 == strncmp((char*)group_name.buf, lsm6dso_reg_config[hw_id][REG_PLATFORM_ORIENT],
                             group_name.buf_len))
      {
        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .group_name = "orient",
            .parse_func = sns_registry_parse_axis_orientation,
            .parsed_buffer = shared_state->inst_cfg.axis_map
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

        if(rv)
        {
#ifdef LSM6DSO_GET_PARAMETER_FROM_SMEM
          uint8_t st_gsensor_dir;
          struct sensor_direction gsensor_dir;
          struct sensor_hw *acc_hw;

          oppo_get_sensor_hw(OPPO_ACCEL, LSM6DSO, &acc_hw);
          if(acc_hw != NULL && acc_hw->direction != DEFAULT_CONFIG) {
            st_gsensor_dir = acc_hw->direction;
            get_direction(st_gsensor_dir,&gsensor_dir);
            shared_state->inst_cfg.axis_map[0].opaxis = gsensor_dir.map[0];
            shared_state->inst_cfg.axis_map[0].invert = gsensor_dir.sign[0];
            shared_state->inst_cfg.axis_map[1].opaxis = gsensor_dir.map[1];
            shared_state->inst_cfg.axis_map[1].invert = gsensor_dir.sign[1];
            shared_state->inst_cfg.axis_map[2].opaxis = gsensor_dir.map[2];
            shared_state->inst_cfg.axis_map[2].invert = gsensor_dir.sign[2];
            SNS_PRINTF(HIGH, this, "st_gsensor_dir: %d", st_gsensor_dir);
          }
#endif

          DBG_PRINTF_EX(MED, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                     shared_state->inst_cfg.axis_map[0].ipaxis,
                     shared_state->inst_cfg.axis_map[0].opaxis,
                     shared_state->inst_cfg.axis_map[0].invert);

          DBG_PRINTF_EX(MED, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                     shared_state->inst_cfg.axis_map[1].ipaxis,
                     shared_state->inst_cfg.axis_map[1].opaxis,
                     shared_state->inst_cfg.axis_map[1].invert);

          DBG_PRINTF_EX(MED, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                     shared_state->inst_cfg.axis_map[2].ipaxis,
                     shared_state->inst_cfg.axis_map[2].opaxis,
                     shared_state->inst_cfg.axis_map[2].invert);
        }
      }
      else if(0 == strncmp((char*)group_name.buf,
                           lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_ACCEL],
                           group_name.buf_len) ||
              0 == strncmp((char*)group_name.buf,
                           lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_GYRO],
                           group_name.buf_len) ||
              0 == strncmp((char*)group_name.buf,
                           lsm6dso_reg_config[hw_id][REG_PLATFORM_FAC_CAL_TEMP],
                           group_name.buf_len))
      {
        lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
        sns_lsm6dso_registry_cfg *cal =
          (state->sensor == LSM6DSO_ACCEL) ? &shared_state->inst_cfg.accel_cal :
          (state->sensor == LSM6DSO_GYRO)  ? &shared_state->inst_cfg.gyro_cal :
          &shared_state->inst_cfg.temper_cal;

        rv = lsm6dso_decode_fac_cal_registry_data(this, &stream, &group_name, &read_event, cal);
      }
      else if(0 == strncmp((char*)group_name.buf,
                           REG_GROUP_ACCEL_THERMAL_SCALE,
                           group_name.buf_len))
      {
        float *data = shared_state->inst_cfg.accel_cal.thermal_scale.data;

        uint8_t arr_index = 0;
        pb_float_arr_arg arr_arg = {
          .arr = data,
          .arr_index = &arr_index,
          .arr_len = TRIAXIS_NUM,
        };

        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .group_name = "scale",
            .parse_func = sns_registry_parse_float_arr,
            .parsed_buffer = &arr_arg,
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        if (!rv) {
          SNS_PRINTF(ERROR, this, "failed to decode thermal scale from registry");
        }

        SNS_PRINTF(MED, this, "accel thermal_scale = [%d/10000, %d/10000, %d/10000]",
                   (int)(data[0] * 10000),
                   (int)(data[1] * 10000),
                   (int)(data[2] * 10000));
      }
      else if(0 == strncmp((char*)group_name.buf,
                           REG_GROUP_GYRO_THERMAL_SCALE,
                           group_name.buf_len))
      {
        float *data = shared_state->inst_cfg.gyro_cal.thermal_scale.data;

        uint8_t arr_index = 0;
        pb_float_arr_arg arr_arg = {
          .arr = data,
          .arr_index = &arr_index,
          .arr_len = TRIAXIS_NUM,
        };

        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .group_name = "scale",
            .parse_func = sns_registry_parse_float_arr,
            .parsed_buffer = &arr_arg,
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        if (!rv) {
          SNS_PRINTF(ERROR, this, "failed to decode thermal scale from registry");
        }

        SNS_PRINTF(MED, this, "gyro thermal_scale = [%d/10000, %d/10000, %d/10000]",
                   (int)(data[0] * 10000),
                   (int)(data[1] * 10000),
                   (int)(data[2] * 10000));
      }
      else if(0 == strncmp((char*)group_name.buf,
                           lsm6dso_reg_config[hw_id][REG_PLATFORM_CONFIG_MD],
                           group_name.buf_len))
      {
        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .parse_func = sns_registry_parse_md_cfg,
            .parsed_buffer = &shared_state->inst_cfg.md_config }
        };

        sns_strlcpy(arg.parse_info[0].group_name,
                    lsm6dso_reg_config[shared_state->hw_idx][REG_PLATFORM_CONFIG_MD],
                    sizeof(arg.parse_info[0].group_name));
        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

        if(rv)
        {
          DBG_PRINTF_EX(
            MED, this, "MD Threshold(*1000):%d m/s2 MD Window(*1000):%d sec MD Disable :%d",
            (int)(shared_state->inst_cfg.md_config.thresh*1000),
            (int)(shared_state->inst_cfg.md_config.win*1000),
            shared_state->inst_cfg.md_config.disable);
        }
      }
      else
      {
        rv = false;
      }

      if(!rv)
      {
        SNS_PRINTF(ERROR, this, "err decoding registry group");
      }
    }
    state->outstanding_reg_requests--;
  }
#if LSM6DSO_REGISTRY_WRITE_EVENT
  else if(SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_EVENT == event->message_id)
  {
    sns_registry_write_event write_event = sns_registry_write_event_init_default;

    if(pb_decode(&stream, sns_registry_write_event_fields, &write_event))
    {
      if (write_event.status == SNS_REGISTRY_WRITE_STATUS_SUCCESS)
      {
        SNS_PRINTF(HIGH, this, "Registry updated");
      }
      else
      {
        SNS_PRINTF(ERROR, this, "Registry update failed");
      }
    }
    else
    {
      SNS_PRINTF(ERROR, this, "Error decoding REGISTRY_WRITE_EVENT");
    }
  }
#endif
  else
  {
    DBG_PRINTF(ERROR, this, "Received unsupported registry event msg id %u",
               event->message_id);
  }
}

static bool
encode_group_for_registry(struct pb_ostream_s *stream, struct pb_field_s const *field,
    void *const *arg)
{
  pb_arg_reg_group_arg* pb_arg = (pb_arg_reg_group_arg*)*arg;
//  lsm6dso_state *state = (lsm6dso_state*)pb_arg->this->state->state;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(pb_arg->this);
  sns_lsm6dso_registry_cfg const *cal = (pb_arg->sensor == LSM6DSO_ACCEL) ?
    &shared_state->inst_cfg.accel_cal : &shared_state->inst_cfg.gyro_cal;

  if (0 == strncmp(pb_arg->name,"bias",strlen("bias")))
  {
    char const *names[] = {"x", "y", "z"};

    for (int i = 0; i < ARR_SIZE(names); i++)
    {
      pb_buffer_arg name_data = (pb_buffer_arg)
                      { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
      sns_registry_data_item pb_item = sns_registry_data_item_init_default;

      pb_item.name.funcs.encode = &pb_encode_string_cb;
      pb_item.name.arg = &name_data;
      pb_item.has_flt = true;
      pb_item.has_version = true;
      pb_item.flt = cal->fac_cal_bias[i];
      pb_item.version = cal->registry_persist_version;

      if(!pb_encode_tag_for_field(stream, field))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding tag");
        return false;
      }

      if(!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding sns_registry_data_item_fields");
        return false;
      }
    }
  }
  #if SUPPORT_SCALE_FACTOR
  else if (0 == strncmp(pb_arg->name,"scale",strlen("scale")))
  {
    char const *names[] = {"sf_0", "sf_1", "sf_2"};
    for (int i = 0; i < ARR_SIZE(names); i++)
    {
      pb_buffer_arg name_data = (pb_buffer_arg)
                           { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
      sns_registry_data_item pb_item = sns_registry_data_item_init_default;

      pb_item.name.funcs.encode = &pb_encode_string_cb;
      pb_item.name.arg = &name_data;
      pb_item.has_flt = true;
      pb_item.flt = cal->fac_cal_scale[i];
      pb_item.has_version = true;
      pb_item.version = cal->registry_persist_version;

      if(!pb_encode_tag_for_field(stream, field))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding tag");
        return false;
      }

      if(!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding sns_registry_data_item_fields");
        return false;
      }
    }
  }
  #endif
  else if (0 == strncmp(pb_arg->name,"corr_mat",strlen("corr_mat")))
  {
    char const *names[] = {"0_0", "0_1", "0_2",
                           "1_0", "1_1", "1_2",
                           "2_0", "2_1", "2_2",};

    for (int i = 0; i < ARR_SIZE(names); i++)
    {
      pb_buffer_arg name_data = (pb_buffer_arg)
                             { .buf = names[i], .buf_len = strlen(names[i]) + 1 };
      sns_registry_data_item pb_item = sns_registry_data_item_init_default;

      pb_item.name.funcs.encode = &pb_encode_string_cb;
      pb_item.name.arg = &name_data;
      pb_item.has_flt = true;
//      pb_item.flt = state->persist_state.output.compensation_matrix[i];
      pb_item.has_version = true;
      pb_item.flt = cal->fac_cal_corr_mat.data[i];
      pb_item.version = cal->registry_persist_version;

      if(!pb_encode_tag_for_field(stream, field))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding tag");
        return false;
      }

      if(!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
      {
        SNS_PRINTF(ERROR, pb_arg->this, "err encoding sns_registry_data_item_fields");
        return false;
      }
    }
  }
  return true;
}

static bool
sns_send_to_registry_persist_cb(struct pb_ostream_s *stream, struct pb_field_s const *field,
    void *const *arg)
{
   pb_arg_reg_group_arg *reg_arg = (pb_arg_reg_group_arg*)*arg;
   sns_sensor *this = reg_arg->this;

#if SUPPORT_SCALE_FACTOR
  char const *names[] = {"bias", "scale", "corr_mat"};
#else
  char const *names[] = {"bias", "corr_mat"};
#endif

  for(int i = 0; i < ARR_SIZE(names); i++)
  {
    sns_registry_data_item pb_item = sns_registry_data_item_init_default;
    pb_buffer_arg name_data = (pb_buffer_arg)
    {
      .buf = names[i], .buf_len = strlen(names[i]) + 1
    };
    pb_arg_reg_group_arg pb_arg= (pb_arg_reg_group_arg){
      .name = NULL,.this = this, .sensor = reg_arg->sensor
    };

    pb_item.has_version = true;
    pb_item.version = reg_arg->version;

    pb_item.name.arg = &name_data;
    pb_item.name.funcs.encode = &pb_encode_string_cb;
    pb_arg.name = names[i];
    pb_item.has_subgroup = true;
    pb_item.subgroup.items.funcs.encode = &encode_group_for_registry;
    pb_item.subgroup.items.arg = &pb_arg;

    if(!pb_encode_tag_for_field(stream, field))
    {
      SNS_PRINTF(ERROR, this,"reg_persist_cb: Error encoding tag for item %u (of %u)",
                 i, ARR_SIZE(names));

      return false;
    }

    if(!pb_encode_submessage(stream, sns_registry_data_item_fields, &pb_item))
    {
      SNS_PRINTF(ERROR, this, "reg_persist_cb: err encoding submsg");
      return false;
    }
  }

  return true;
}
#endif

static void send_suid_req(sns_sensor *this, char *const data_type, uint32_t data_type_len)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);

  if(shared_state->suid_stream == NULL)
  {
    sns_service_manager *manager = this->cb->get_service_manager(this);
    sns_stream_service *stream_service =
      (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);
     stream_service->api->create_sensor_stream(stream_service, this, sns_get_suid_lookup(),
                                               &shared_state->suid_stream);
  }

  if(shared_state->suid_stream != NULL)
  {
    size_t encoded_len;
    pb_buffer_arg data = (pb_buffer_arg){ .buf = data_type, .buf_len = data_type_len };
    uint8_t buffer[50];

    sns_suid_req suid_req = sns_suid_req_init_default;
    suid_req.has_register_updates = true;
    suid_req.register_updates = true;
    suid_req.data_type.funcs.encode = &pb_encode_string_cb;
    suid_req.data_type.arg = &data;
    sns_rc rc = SNS_RC_SUCCESS;

    encoded_len = pb_encode_request(buffer, sizeof(buffer), &suid_req, sns_suid_req_fields, NULL);
    if(0 < encoded_len)
    {
      sns_request request = (sns_request){
        .request_len = encoded_len, .request = buffer, .message_id = SNS_SUID_MSGID_SNS_SUID_REQ };
      rc = shared_state->suid_stream->api->
             send_request(shared_state->suid_stream, &request);
    }
    if(0 >= encoded_len || SNS_RC_SUCCESS != rc)
    {
      SNS_PRINTF(ERROR, this, "encoded_len=%d rc=%u", encoded_len, rc);
    }
  }
}

static void init_dependencies(sns_sensor *const this)
{
  DBG_PRINTF_EX(LOW, this, "init_dependencies sensor");

  for(int i=0;i<ARR_SIZE(def_dependency);i++)
  {
    send_suid_req(this, def_dependency[i], strlen(def_dependency[i]));
  }
}

void lsm6dso_init_sensor_info(sns_sensor *const this,
                              sns_sensor_uid const *suid,
                              lsm6dso_sensor_type sensor_type)
{
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;

  state->sensor = sensor_type;
  sns_memscpy(&state->my_suid, sizeof(state->my_suid), suid, sizeof(sns_sensor_uid));

  DBG_PRINTF_EX(LOW, this, "init_sensor_info: sensor=%u", sensor_type);

  if(LSM6DSO_ACCEL == sensor_type)
  {
    init_dependencies(this);
#if LSM6DSO_REGISTRY_DISABLED
    sns_registry_phy_sensor_pf_cfg cfg;
    sns_lsm6dso_registry_def_config(this, &cfg);
    sensor_save_registry_pf_cfg(this, &cfg);
#endif
  }
}

void lsm6dso_process_suid_events(sns_sensor *const this)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  sns_data_stream *stream = shared_state->suid_stream;
  sns_service_manager *service_mgr;
  sns_stream_service  *stream_svc;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return;
  }

  service_mgr = this->cb->get_service_manager(this);
  stream_svc = (sns_stream_service*) service_mgr->get_service(service_mgr,
                                                              SNS_STREAM_SERVICE);
  for(sns_sensor_event *event = stream->api->peek_input(stream);
      NULL != event;
      event = stream->api->get_next_input(stream))
  {
    if(SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
    {
      pb_istream_t pbstream = pb_istream_from_buffer((void*)event->event, event->event_len);
      sns_suid_event suid_event = sns_suid_event_init_default;
      pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
      sns_sensor_uid uid_list;
      sns_suid_search suid_search;
      suid_search.suid = &uid_list;
      suid_search.num_of_suids = 0;

      suid_event.data_type.funcs.decode = &pb_decode_string_cb;
      suid_event.data_type.arg = &data_type_arg;
      suid_event.suid.funcs.decode = &pb_decode_suid_event;
      suid_event.suid.arg = &suid_search;

      if(!pb_decode(&pbstream, sns_suid_event_fields, &suid_event))
      {
         SNS_PRINTF(ERROR, this, "pb_decode() failed");
         continue;
       }

      /* if no suids found, ignore the event */
      if(suid_search.num_of_suids == 0)
      {
        continue;
      }

      /* save suid based on incoming data type name */
      if(0 == strncmp(data_type_arg.buf, "interrupt", data_type_arg.buf_len))
      {
        shared_state->inst_cfg.irq_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len))
      {
        shared_state->inst_cfg.timer_suid = uid_list;
#if !LSM6DSO_POWERRAIL_DISABLED
        stream_svc->api->create_sensor_stream(stream_svc, this,
                                              shared_state->inst_cfg.timer_suid,
                                              &shared_state->timer_stream);
        if(NULL == shared_state->timer_stream)
        {
          DBG_PRINTF(ERROR, this, "process_suid_events: Failed to create timer stream");
        }
#endif
      }
      else if (0 == strncmp(data_type_arg.buf, "async_com_port",
                            data_type_arg.buf_len))
      {
        shared_state->inst_cfg.acp_suid = uid_list;
      }
#if !LSM6DSO_REGISTRY_DISABLED
      else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len))
      {
        shared_state->inst_cfg.reg_suid = uid_list;
        process_registry_suid(this);
      }
#endif
#if LSM6DSO_DAE_ENABLED
      else if (0 == strncmp(data_type_arg.buf, "data_acquisition_engine",
                            data_type_arg.buf_len))
      {
        shared_state->inst_cfg.dae_suid = uid_list;
      }
#endif
      else
      {
        SNS_PRINTF(ERROR, this, "process_suid_events: invalid datatype_name");
      }
    }
  }
  return;
}

void lsm6dso_process_registry_events(sns_sensor *const this)
{
#if !LSM6DSO_REGISTRY_DISABLED
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  sns_data_stream *stream = state->reg_data_stream;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return;
  }

  DBG_PRINTF_EX(HIGH, this, "registry_event: sensor=%u stream=%x", state->sensor, stream);

  for(; 0 != stream->api->get_input_cnt(stream); stream->api->get_next_input(stream))
  {
    sns_sensor_event *event = stream->api->peek_input(stream);
    if(LSM6DSO_IS_OIS_SENSOR(state->sensor)) {
      lsm6dso_process_ois_registry_event(this, event);
      continue;
    }
    process_registry_event(this, event);
  }
  if (state->outstanding_reg_requests == 0)
  {
    sns_sensor_util_remove_sensor_stream(this, &state->reg_data_stream);
  }
#else
  UNUSED_VAR(this);
#endif
}

bool lsm6dso_sensor_write_output_to_registry(sns_sensor *const this)
{
  bool registry_updated = false;
#if !LSM6DSO_REGISTRY_DISABLED
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;
  lsm6dso_shared_state* shared_state = lsm6dso_get_shared_state(this);
  pb_arg_reg_group_arg arg = {.this = this };

  pb_buffer_arg name_data = {0, 0};
  sns_registry_write_req write_req = sns_registry_write_req_init_default;
  char* accel_name = lsm6dso_reg_config[shared_state->hw_idx][REG_PLATFORM_FAC_CAL_ACCEL];
  char* gyro_name = lsm6dso_reg_config[shared_state->hw_idx][REG_PLATFORM_FAC_CAL_GYRO];
  sns_lsm6dso_registry_cfg *cal =
    (state->sensor == LSM6DSO_ACCEL) ?
    &shared_state->inst_cfg.accel_cal : &shared_state->inst_cfg.gyro_cal;

  if(LSM6DSO_ACCEL == state->sensor)
  {
    name_data.buf = (void *)accel_name;
    //name_data.buf = LSM6DSO_REG_PLATFORM_FAC_CAL_ACCEL;
  }
  else if(LSM6DSO_GYRO == state->sensor)
  {
    name_data.buf = (void *)gyro_name;
    //name_data.buf = LSM6DSO_REG_PLATFORM_FAC_CAL_GYRO;
  }

  if(NULL != name_data.buf)
  {
    arg.sensor = state->sensor;
    arg.version = cal->registry_persist_version;
    name_data.buf_len = strlen(name_data.buf) + 1;
    write_req.name.funcs.encode = &pb_encode_string_cb;
    write_req.name.arg = &name_data;
    write_req.data.items.funcs.encode = &sns_send_to_registry_persist_cb;
    write_req.data.items.arg = &arg;

    if(NULL == state->reg_data_stream)
    {
      sns_service_manager *service_mgr = this->cb->get_service_manager(this);
      sns_stream_service *stream_svc = (sns_stream_service*)
        service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

      stream_svc->api->create_sensor_stream(stream_svc, this,
                                            shared_state->inst_cfg.reg_suid,
                                            &state->reg_data_stream);
    }
    if(NULL != state->reg_data_stream)
    {
      uint8_t buffer[1000];
      int32_t encoded_len = pb_encode_request(buffer, sizeof(buffer), &write_req,
                                              sns_registry_write_req_fields, NULL);
      if(0 < encoded_len)
      {
        sns_request request = {
          .request = buffer,
          .request_len = encoded_len,
          .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_WRITE_REQ };
        sns_rc rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
        if(rc == SNS_RC_SUCCESS)
        {
          registry_updated = true;
          DBG_PRINTF_EX(HIGH, this, "write_to_reg: done");
        }
        else
        {
          SNS_PRINTF(ERROR, this, "write_to_reg:rc=%d", rc);
        }
      }
      else
      {
        SNS_PRINTF(ERROR, this, "write_to_reg: failed encoding");
      }
    }
  }
#else
  UNUSED_VAR(this);
#endif
  return registry_updated;
}

/**
 * Provides odr drift based on STM_LSM6DSO_REG_FREQ_FINE.
 *
 * @param[i] this              Current LSM6DSO sns_sensor.
 *
 * @return void
 */
static void lsm6dso_read_sample_odr_drift(sns_sensor *const this)
{
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_com_port_info *com_port = &shared_state->inst_cfg.com_port_info;
  uint32_t xfer_bytes;
  uint8_t  freq_drift;
  lsm6dso_com_read_wrapper(shared_state->scp_service,
        com_port->port_handle,
        STM_LSM6DSO_REG_FREQ_FINE,
        &freq_drift,
        1,
        &xfer_bytes);

  shared_state->clock_trim_factor = (1 + ((0.0015f * (int8_t)freq_drift)));
  DBG_PRINTF_EX(LOW, this, "freq_drift=%d, clock_trim_factor=%d",
                freq_drift, (int)shared_state->clock_trim_factor);
  return;
}

sns_rc lsm6dso_discover_hw(sns_sensor *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lsm6dso_shared_state *shared_state = lsm6dso_get_shared_state(this);
  lsm6dso_com_port_info *com_port = &shared_state->inst_cfg.com_port_info;
  uint8_t buffer[1];

  /**-----------------Register and Open COM Port-------------------------*/
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  shared_state->scp_service =  (sns_sync_com_port_service *)
	  service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  rv = shared_state->scp_service->api->
	sns_scp_register_com_port(&com_port->com_config, &com_port->port_handle);

  if(rv == SNS_RC_SUCCESS)
  {
    rv = shared_state->scp_service->api->sns_scp_open(com_port->port_handle);
  }
  else
  {
    SNS_PRINTF(ERROR, this, "sns_scp_register_com_port fail rv:%u",rv);
  }

  /**-------------------Read and Confirm WHO-AM-I------------------------*/
  buffer[0] = 0x0;
  rv = lsm6dso_get_who_am_i(shared_state->scp_service, com_port->port_handle, &buffer[0]);

  if(rv == SNS_RC_SUCCESS && buffer[0] == LSM6DSO_WHOAMI_VALUE)
  {
    DBG_PRINTF_EX(HIGH, this, "[%u] Got 0x%x", shared_state->hw_idx, buffer[0]);
    sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
    if(instance)
    {
      DBG_PRINTF_EX(HIGH, this, "Instance available resetting device");
      // Reset Sensor
      rv = lsm6dso_reset_device(instance,
          LSM6DSO_ACCEL | LSM6DSO_GYRO | LSM6DSO_MOTION_DETECT | LSM6DSO_SENSOR_TEMP);
      if(rv == SNS_RC_SUCCESS)
      {
        shared_state->hw_is_present = true;
      }
    }
    else
    {
      shared_state->hw_is_present = true;
    }
  }
  else
  {
    SNS_PRINTF(ERROR, this, "[%u] err=0x%x WhoAmI=0x%x", shared_state->hw_idx, rv, buffer[0]);
  }
  shared_state->who_am_i = buffer[0];

  /**-----Read clock frequency drift----*/
  //Use STM_LSM6DSO_REG_FREQ_FINE to read initial sample
  lsm6dso_read_sample_odr_drift(this);

  /**------------------Power Down and Close COM Port--------------------*/
  shared_state->scp_service->api->sns_scp_update_bus_power(com_port->port_handle, false);

  shared_state->scp_service->api->sns_scp_close(com_port->port_handle);
  shared_state->scp_service->api->sns_scp_deregister_com_port(&com_port->port_handle);

  /**----------------------Turn Power Rail OFF--------------------------*/
  lsm6dso_update_rail_vote(this, shared_state, SNS_RAIL_OFF, NULL);

  if(rv != SNS_RC_SUCCESS)
  {
    SNS_PRINTF(HIGH, this, "discover_hw failed %d", rv);
    rv = SNS_RC_INVALID_LIBRARY_STATE;
  }

#ifndef OPLUS_SENSOR_LITE_DRIVER
  if (shared_state->hw_is_present && SNS_RC_SUCCESS == rv) {
    static bool register_deinfo = false;
    struct devinfo info_acc = {
      .cal_path = LSM6DSO_GEN_GROUP(0, PLATFORM_FAC_CAL_ACCEL) "" LSM6DSO_FAC_CAL_BIAS,
    };
    struct devinfo info_gyro = {
      .cal_path = LSM6DSO_GEN_GROUP(0, PLATFORM_FAC_CAL_GYRO) "" LSM6DSO_FAC_CAL_BIAS,
    };
    if (!register_deinfo) {
      register_deinfo = true;
      register_sensor_devinfo(OPLUS_GSENSOR,&info_acc);
      register_sensor_devinfo(OPLUS_GYRO,&info_gyro);
      SNS_PRINTF(HIGH, this, "lsm6dsO gsensor and gyro register_sensor_devinfo");
    }
  }
#endif

  return rv;
}

void lsm6dso_update_siblings(sns_sensor *const this, lsm6dso_shared_state *shared_state)
{
  bool all_suids_available = true;
  lsm6dso_state *state = (lsm6dso_state*)this->state->state;

  if(   SUID_IS_NULL(&shared_state->inst_cfg.irq_suid)
     || SUID_IS_NULL(&shared_state->inst_cfg.acp_suid)
     || SUID_IS_NULL(&shared_state->inst_cfg.timer_suid)
#if !LSM6DSO_REGISTRY_DISABLED
     || SUID_IS_NULL(&shared_state->inst_cfg.reg_suid)
#endif
#if LSM6DSO_DAE_ENABLED
    || SUID_IS_NULL(&shared_state->inst_cfg.dae_suid)
#endif
    )
  {
    all_suids_available = false;
  }

  if(all_suids_available)
  {
    sns_sensor *lib_sensor;

    DBG_PRINTF_EX(HIGH, this, "siblings: publishing registry attributes and avail");

    for(lib_sensor = this->cb->get_library_sensor(this, true);
        NULL != lib_sensor;
        lib_sensor = this->cb->get_library_sensor(this, false))
    {
      DBG_PRINTF(HIGH, this, "[%u] siblings: sensor=0x%x",
                 shared_state->hw_idx, ((lsm6dso_state*)lib_sensor->state->state)->sensor);
      publish_registry_attributes(lib_sensor, shared_state);
      publish_available(lib_sensor);
    }
    //Remove registry data stream
    sns_sensor_util_remove_sensor_stream(this, &state->reg_data_stream);
    sns_sensor_util_remove_sensor_stream(this, &shared_state->suid_stream);
  }
}

void lsm6dso_handle_selftest_request_removal(
  sns_sensor          *const this,
  sns_sensor_instance *const instance,
  lsm6dso_shared_state *shared_state)
{
  lsm6dso_instance_state *inst_state   = (lsm6dso_instance_state*)instance->state->state;

  DBG_PRINTF_EX(MED, this, "selftest_removal: config=0x%x update=%u",
             shared_state->inst_cfg.config_sensors,
             inst_state->self_test_info.update_registry);

  //if reconfigure hw has been postponed due to a remove request during self test. Do it now
  if(inst_state->self_test_info.reconfig_postpone)
  {
    DBG_PRINTF_EX(MED, this, "Reconfiguring HW for request received during self-test");
    lsm6dso_set_client_config(this, instance, shared_state);
    inst_state->self_test_info.reconfig_postpone = false;
  }

  if(inst_state->self_test_info.update_registry) //factory self test ran to completion
  {
    sns_lsm6dso_registry_cfg *dest_cal =
      (LSM6DSO_ACCEL == inst_state->self_test_info.sensor) ?
      &shared_state->inst_cfg.accel_cal : &shared_state->inst_cfg.gyro_cal;
    sns_lsm6dso_registry_cfg *src_cal =
      (LSM6DSO_ACCEL == inst_state->self_test_info.sensor) ?
      &inst_state->accel_registry_cfg : &inst_state->gyro_registry_cfg;

    dest_cal->ts = src_cal->ts;
    dest_cal->registry_persist_version++;
    sns_memscpy(dest_cal->fac_cal_bias, sizeof(dest_cal->fac_cal_bias),
                src_cal->fac_cal_bias, sizeof(src_cal->fac_cal_bias));

    DBG_PRINTF(MED, this, "selftest_removal: version=%u",
               dest_cal->registry_persist_version);
    lsm6dso_sensor_write_output_to_registry(this);
    inst_state->self_test_info.update_registry = false;
  }
}

