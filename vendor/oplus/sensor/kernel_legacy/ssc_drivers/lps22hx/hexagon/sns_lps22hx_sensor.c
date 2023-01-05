/**
 * @file sns_lps22hx_sensor.c
 *
 * Common implementation for LPS22HX Sensors.
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
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"

#include "sns_lps22hx_sensor.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"
#include "sns_devinfo_utils.h"

#if LPS22HX_DEVICE_LPS22HB
#define LPS22HX_REG_CONFIG_PRESS            "lps22hb_0.pressure.config"
#define LPS22HX_REG_CONFIG_TEMP             "lps22hb_0.temp.config"
#define LPS22HX_REG_PLATFORM_CONFIG         "lps22hb_0_platform.config"
#define LPS22HX_REG_PLATFORM_PLACEMENT      "lps22hb_0_platform.placement"
#define LPS22HX_REG_PLATFORM_FAC_CAL_PRESS  "lps22hb_0_platform.pressure.fac_cal"
#define LPS22HX_REG_PLATFORM_FAC_CAL_TEMP   "lps22hb_0_platform.temp.fac_cal"
#else
#define LPS22HX_REG_CONFIG_PRESS            "lps22hh_0.pressure.config"
#define LPS22HX_REG_CONFIG_TEMP             "lps22hh_0.temp.config"
#define LPS22HX_REG_PLATFORM_CONFIG         "lps22hh_0_platform.config"
#define LPS22HX_REG_PLATFORM_PLACEMENT      "lps22hh_0_platform.placement"
#define LPS22HX_REG_PLATFORM_FAC_CAL_PRESS  "lps22hh_0_platform.pressure.fac_cal"
#define LPS22HX_REG_PLATFORM_FAC_CAL_TEMP   "lps22hh_0_platform.temp.fac_cal"
#endif

#define MAX_DEP_LENGTH 20

#if !(LPS22HX_POLLING)
static char def_dependency[][MAX_DEP_LENGTH] =  {"interrupt", "async_com_port", "timer"};
#else
static char def_dependency[][MAX_DEP_LENGTH] =  {"async_com_port", "timer"};
#endif

static const char name[] = PRESS_SENSOR_NAME;
static const char vendor[] = PRESS_SENSOR_VENDOR;
static const uint32_t version = SNS_VERSION_LPS22HX; // major[31:16].minor[15:0]

static const uint32_t max_fifo_depth = 0; // samples

static const sns_std_sensor_stream_type stream_type = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
static const bool is_dynamic = false;
static const sns_std_sensor_rigid_body_type rigid_body = SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
static const uint32_t hardware_id = 0;
static const bool supports_dri = true;
static const bool supports_sync_stream = false;

void lps22hx_publish_def_attributes(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;

  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->available;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
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
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
  }

  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
    char const op_mode1[] = LPS22HX_MODE_PWRDWN;
    char const op_mode2[] = LPS22HX_ONE_SHOT;
    char const op_mode3[] = LPS22HX_CONTINUOUS;

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    values[1].str.funcs.encode = pb_encode_string_cb;
    values[1].str.arg = &((pb_buffer_arg)
        { .buf = op_mode2, .buf_len = sizeof(op_mode2) });
    values[2].str.funcs.encode = pb_encode_string_cb;
    values[2].str.arg = &((pb_buffer_arg)
        { .buf = op_mode3, .buf_len = sizeof(op_mode3) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }

  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
    char const proto1[] = "sns_physical_sensor_test.proto";
    char const proto2[] = "sns_std_sensor.proto";

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
    values[1].str.funcs.encode = pb_encode_string_cb;
    values[1].str.arg = &((pb_buffer_arg)
        { .buf = proto2, .buf_len = sizeof(proto2) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }

  {
    float data[1] = {0};
    state->encoded_event_len =
      pb_get_encoded_size_sensor_stream_event(data, 1);
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
    value.boolean = supports_sync_stream;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, false);
  }  
}

/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void publish_registry_attributes(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  const float resolutions[] =	{LPS22HX_PRESS_RESOLUTION };  // 1/4096 lsb/hPa
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
      values[i].flt = state->placement[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->registry_pf_cfg.rigid_body_type;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
  if(state->sensor == LPS22HX_PRESS)
  {
    {
      /** Only pressure may use registry information for selected resolution. */
      sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
      value.has_flt = true;
      value.flt = resolutions[0];
      sns_publish_attribute(
          this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);
    }
    {
      sns_std_attr_value_data values[] = {SNS_ATTR};

      sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
      range1[0].has_flt = true;
      range1[0].flt = LPS22HX_PRESS_RANGE_MIN;
      range1[1].has_flt = true;
      range1[1].flt = LPS22HX_PRESS_RANGE_MAX;
      values[0].has_subtype = true;
      values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
      values[0].subtype.values.arg =
        &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

      sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE,
          values, ARR_SIZE(values), false);
    }
  }
}

static void publish_available(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
  value.has_boolean = true;
  value.boolean = true;
  sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
  state->available = true;
}

static void lps22hx_send_registry_request(sns_sensor *const this, char *reg_group_name)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  uint8_t buffer[100];
  int32_t encoded_len;
  sns_memset(buffer, 0, sizeof(buffer));

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
    DBG_PRINT(state->diag_service, this, ERROR, __FILENAME__, __LINE__,
        "lps22hx_send_registry_request:: group not sent");
  }
}

static void send_registry_requests(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;

  if(LPS22HX_PRESS == state->sensor)
  {
    lps22hx_send_registry_request(this, LPS22HX_REG_PLATFORM_CONFIG);
    lps22hx_send_registry_request(this, LPS22HX_REG_PLATFORM_PLACEMENT);
    lps22hx_send_registry_request(this, LPS22HX_REG_CONFIG_PRESS);
    lps22hx_send_registry_request(this, LPS22HX_REG_PLATFORM_FAC_CAL_PRESS);
  }
  else if(LPS22HX_SENSOR_TEMP == state->sensor)
  {
    lps22hx_send_registry_request(this, LPS22HX_REG_CONFIG_TEMP);
    lps22hx_send_registry_request(this, LPS22HX_REG_PLATFORM_FAC_CAL_TEMP);
  }
}


/** See sns_lps22hx_sensor.h */
void lps22hx_send_suid_req(sns_sensor *this, char *const data_type,
    uint32_t data_type_len)
{
  uint8_t buffer[50];
  sns_memset(buffer, 0, sizeof(buffer));
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_service_manager *manager = this->cb->get_service_manager(this);
  sns_stream_service *stream_service =
    (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);
  size_t encoded_len;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = data_type, .buf_len = data_type_len };
  sns_diag_service* diag = state->diag_service;
  DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
      "Entering lps22hx_send_suid_req");
  sns_suid_req suid_req = sns_suid_req_init_default;
  suid_req.has_register_updates = true;
  suid_req.register_updates = true;
  suid_req.data_type.funcs.encode = &pb_encode_string_cb;
  suid_req.data_type.arg = &data;

  if(state->fw_stream == NULL)
  {
    stream_service->api->create_sensor_stream(stream_service,
        this, sns_get_suid_lookup(), &state->fw_stream);
  }

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
      &suid_req, sns_suid_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer, .message_id = SNS_SUID_MSGID_SNS_SUID_REQ };
    state->fw_stream->api->send_request(state->fw_stream, &request);
  }
}

void lps22hx_init_dependencies(sns_sensor *const this)
{
  int i = 0;
  UNUSED_VAR(this);
  for(i=0;i<ARR_SIZE(def_dependency);i++)
    lps22hx_send_suid_req(this, def_dependency[i], strlen(def_dependency[i]));
}

static void process_registry_suid(sns_sensor *const this, sns_sensor_uid *reg_suid)
{
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_svc  =
    (sns_stream_service*) service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  lps22hx_state *state = (lps22hx_state*)this->state->state;

  state->reg_suid = *reg_suid;

  stream_svc->api->create_sensor_stream(stream_svc, this, *reg_suid,
                                          &state->reg_data_stream);
  if(NULL != state->reg_data_stream)
  {
    send_registry_requests(this);

    DBG_PRINT(state->diag_service, this, HIGH, __FILENAME__, __LINE__,
              "process_registry_suid:: sensor=%u reg_stream=%x #req=%u",
              state->sensor, state->reg_data_stream, state->outstanding_reg_requests);
  }
  else
  {
    DBG_PRINT(state->diag_service, this, ERROR, __FILENAME__, __LINE__,
              "Failed to create registry stream");
  }
}

void sns_lps22hx_save_registry_pf_cfg(
  sns_sensor *const this,
  sns_registry_phy_sensor_pf_cfg const * phy_sensor_pf_cfg)
{
  sns_rc rc;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_diag_service *diag = state->diag_service;

  state->rigid_body_type = phy_sensor_pf_cfg->rigid_body_type;
  state->com_port_info.com_config.bus_type = phy_sensor_pf_cfg->bus_type;
  state->com_port_info.com_config.bus_instance = phy_sensor_pf_cfg->bus_instance;
  state->com_port_info.com_config.slave_control = phy_sensor_pf_cfg->slave_config;
  state->com_port_info.com_config.min_bus_speed_KHz = phy_sensor_pf_cfg->min_bus_speed_khz;
  state->com_port_info.com_config.max_bus_speed_KHz = phy_sensor_pf_cfg->max_bus_speed_khz;
  state->com_port_info.com_config.reg_addr_type = phy_sensor_pf_cfg->reg_addr_type;
#if !(LPS22HX_POLLING)
  state->irq_config.interrupt_num = phy_sensor_pf_cfg->dri_irq_num;
  state->irq_config.interrupt_pull_type = phy_sensor_pf_cfg->irq_pull_type;
  state->irq_config.is_chip_pin = phy_sensor_pf_cfg->irq_is_chip_pin;
  state->irq_config.interrupt_drive_strength = phy_sensor_pf_cfg->irq_drive_strength;
  state->irq_config.interrupt_trigger_type = phy_sensor_pf_cfg->irq_trigger_type;
#endif
  state->rail_config.num_of_rails = phy_sensor_pf_cfg->num_rail;
  state->registry_rail_on_state = phy_sensor_pf_cfg->rail_on_state;

  sns_strlcpy(state->rail_config.rails[0].name,
              phy_sensor_pf_cfg->vddio_rail,
              sizeof(state->rail_config.rails[0].name));
  sns_strlcpy(state->rail_config.rails[1].name,
              phy_sensor_pf_cfg->vdd_rail,
              sizeof(state->rail_config.rails[1].name));

  DBG_PRINT(diag, this, MED, __FILENAME__, __LINE__,
            "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
            state->com_port_info.com_config.min_bus_speed_KHz,
            state->com_port_info.com_config.max_bus_speed_KHz,
            state->com_port_info.com_config.reg_addr_type);

#if !(LPS22HX_POLLING)
  DBG_PRINT(diag, this, MED, __FILENAME__, __LINE__,
            "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
            state->irq_config.interrupt_num,
            state->irq_config.interrupt_pull_type,
            state->irq_config.is_chip_pin);

  DBG_PRINT(diag, this, MED, __FILENAME__, __LINE__,
            "interrupt_drive_strength:%d interrupt_trigger_type:%d"
            " rigid body type:%d", state->irq_config.interrupt_drive_strength,
            state->irq_config.interrupt_trigger_type,
            phy_sensor_pf_cfg->rigid_body_type);
#endif

  /**-----------------Register and Open COM Port-------------------------*/
  state->scp_service =  (sns_sync_com_port_service *)
      service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  rc = state->scp_service->api->sns_scp_register_com_port(
    &state->com_port_info.com_config,
    &state->com_port_info.port_handle);

  if(rc == SNS_RC_SUCCESS)
  {
    rc = state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
  }
  else
  {
    DBG_PRINT(diag, this, ERROR, "sns_scp_register_com_port fail rc:%u",rc);
  }

  /**---------------------Register Power Rails --------------------------*/
  if(NULL == state->pwr_rail_service && rc == SNS_RC_SUCCESS)
  {
    state->rail_config.rail_vote = SNS_RAIL_OFF;

    state->pwr_rail_service =
      (sns_pwr_rail_service*)service_mgr->get_service(service_mgr,
                                                      SNS_POWER_RAIL_SERVICE);

    state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
                                                           &state->rail_config);
  }
}

static void lps22hx_sensor_process_registry_event(sns_sensor *const this,
    sns_sensor_event *event)
{
  bool rv = true;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;

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
      DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
        "Error decoding registry event");
    }
    else
    {
      stream = pb_istream_from_buffer((void*)event->event, event->event_len);

      if(0 == strncmp((char*)group_name.buf, LPS22HX_REG_CONFIG_PRESS,
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, LPS22HX_REG_CONFIG_TEMP,
                      group_name.buf_len))
      {
        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .group_name = "config",
            .parse_func = sns_registry_parse_phy_sensor_cfg,
            .parsed_buffer = &state->registry_cfg
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

        if(rv)
        {
          state->is_dri = state->registry_cfg.is_dri;
          state->hardware_id = state->registry_cfg.hw_id;
          state->resolution_idx = state->registry_cfg.res_idx;
          state->supports_sync_stream = state->registry_cfg.sync_stream;

          DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
              "resolution_idx:%d, supports_sync_stream:%d ",
              state->resolution_idx,
              state->supports_sync_stream);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, LPS22HX_REG_PLATFORM_CONFIG,
                           group_name.buf_len))
      {
        sns_registry_decode_arg arg = {
          .item_group_name = &group_name,
          .parse_info_len = 1,
          .parse_info[0] = {
            .group_name = "config",
            .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
            .parsed_buffer = &state->registry_pf_cfg
          }
        };

        read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
        read_event.data.items.arg = &arg;

        rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);

        if(rv)
        {
          sns_lps22hx_save_registry_pf_cfg(this, &state->registry_pf_cfg);
        }
      }
      else if(0 == strncmp((char*)group_name.buf,
                           LPS22HX_REG_PLATFORM_FAC_CAL_PRESS,
                           group_name.buf_len) ||
              0 == strncmp((char*)group_name.buf,
                           LPS22HX_REG_PLATFORM_FAC_CAL_TEMP,
                           group_name.buf_len))
      {
        {
          uint8_t bias_arr_index = 0, scale_arr_index = 0;
          pb_float_arr_arg bias_arr_arg = {
            .arr = state->fac_cal_bias,
            .arr_index = &bias_arr_index,
            .arr_len = 1
          };

          pb_float_arr_arg scale_arr_arg = {
            .arr = state->fac_cal_scale,
            .arr_index = &scale_arr_index,
            .arr_len = 1
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
              .parsed_buffer = &state->fac_cal_corr_mat
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_fac_cal_received = true;
          // TODO: implement the below logic once facility is available from Qualcomm framework
          //          state->registry_persist_version = <version_from_registry>;
          if(state->fac_cal_scale[0] != 0.0)
          {
            state->fac_cal_corr_mat.e00 = state->fac_cal_scale[0];
            state->fac_cal_corr_mat.e11 = state->fac_cal_scale[1];
            state->fac_cal_corr_mat.e22 = state->fac_cal_scale[2];
          }

          DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__, "Fac Cal Corr Matrix e00:%d e01:%d e02:%d",
              state->fac_cal_corr_mat.e00, state->fac_cal_corr_mat.e01, state->fac_cal_corr_mat.e02);
          DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__, "Fac Cal Corr Matrix e10:%d e11:%d e12:%d",
              state->fac_cal_corr_mat.e10, state->fac_cal_corr_mat.e11, state->fac_cal_corr_mat.e12);
          DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__, "Fac Cal Corr Matrix e20:%d e21:%d e22:%d",
              state->fac_cal_corr_mat.e20, state->fac_cal_corr_mat.e21, state->fac_cal_corr_mat.e22);
          DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__, "Fac Cal Bias x:%d y:%d z:%d",
              state->fac_cal_bias[0], state->fac_cal_bias[1], state->fac_cal_bias[2]);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, LPS22HX_REG_PLATFORM_PLACEMENT,
                           group_name.buf_len))
      {
        {
          uint8_t arr_index = 0;
          pb_float_arr_arg arr_arg = {
            .arr = state->placement,
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
      }
      else
      {
        rv = false;
      }

      if(!rv)
      {
        DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__, "Error decoding registry group ");
      }
    }
    state->outstanding_reg_requests--;
  }
  else
  {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
      "Received unsupported registry event msg id %u",
        event->message_id);
  }
}

/** See sns_lps22hx_sensor.h */
void lps22hx_process_suid_events(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_service_manager *service_mgr;
  sns_stream_service  *stream_svc;

  service_mgr = this->cb->get_service_manager(this);
  stream_svc = (sns_stream_service*) service_mgr->get_service(service_mgr,
      SNS_STREAM_SERVICE);

  sns_diag_service* diag = state->diag_service;
  DBG_PRINT_EX(diag, this, ERROR, __FILENAME__, __LINE__,
      "Entering lps22hx_process_suid_events");
  for(;
      0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
      state->fw_stream->api->get_next_input(state->fw_stream))
  {
    sns_sensor_event *event =
      state->fw_stream->api->peek_input(state->fw_stream);

    if(SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
    {
      pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
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

      if(!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
        DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
            "SUID Decode failed");
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
        state->irq_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len))
      {
        state->timer_suid = uid_list;
        stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid,
                                              &state->timer_stream);
        if(NULL == state->timer_stream)
        {
          DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
                    "process_suid_events:: Failed to create timer stream");
        }
      }
      else if (0 == strncmp(data_type_arg.buf, "async_com_port",
            data_type_arg.buf_len))
      {
        state->acp_suid = uid_list;
      }
      else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len))
      {
        process_registry_suid(this, &uid_list);
      }
      else
      {
        DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
                  "invalid datatype_name %s", data_type_arg.buf);
      }
    }
  }
}

sns_rc lps22hx_process_registry_events(sns_sensor *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  //uint32_t num_events;
  sns_sensor_event *event;

  if(NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
    while(NULL != event)
    {
      lps22hx_sensor_process_registry_event(this, event);
      event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }
  if (state->outstanding_reg_requests == 0)
  {
    sns_sensor_util_remove_sensor_stream(this, &state->reg_data_stream);
  }
  return rv;
}

void lps22hx_discover_hw(sns_sensor *const this)
{
  sns_rc rv = SNS_RC_SUCCESS;
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  sns_diag_service *diag = state->diag_service;
  uint8_t buffer[1];

  /**-------------------Read and Confirm WHO-AM-I------------------------*/
  buffer[0] = 0x0;
  DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
      "Calling who am i");
  rv = lps22hx_get_who_am_i(state->scp_service,state->com_port_info.port_handle, &buffer[0]);
  DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,"Error = 0x%x : WhoAmI=0x%x",rv,buffer[0]);

  if(rv == SNS_RC_SUCCESS
      &&
      buffer[0] == LPS22HX_WHOAMI_VALUE)
  {
#if LPS22HX_DEVICE_LPS22HB
    DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
        "Got LPS22HB");
#else
    DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
        "Got LPS22HH");
#endif
    sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
    if(instance)
    {
      DBG_PRINT_EX(diag, this, MED, __FILENAME__, __LINE__,
                "Instance available resetting device");
      // Reset Sensor
      rv = lps22hx_reset_device(instance, LPS22HX_PRESS | LPS22HX_SENSOR_TEMP);
      if(rv == SNS_RC_SUCCESS)
      {
        state->hw_is_present = true;
      }
    }
    else
    {
      DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
                "Instance Not available");
      state->hw_is_present = true;
    }
  }
  else
  {
    DBG_PRINT(diag, this, ERROR, __FILENAME__, __LINE__,
              "Error = 0x%x : WhoAmI=0x%x", rv, buffer[0]);
  }
  state->who_am_i = buffer[0];

  /**------------------Power Down and Close COM Port--------------------*/
  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                    false);

  state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
  state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);

  /**----------------------Turn Power Rail OFF--------------------------*/
  state->rail_config.rail_vote = SNS_RAIL_OFF;
  state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                           this,
                                                           &state->rail_config,
                                                           NULL);
  if (state->hw_is_present) {
	static bool register_deinfo = false;
	struct devinfo info_press = {
		.cal_path = "lps22hh_0_platform.pressure.fac_cal.bias",
	};
	if (!register_deinfo) {
		register_deinfo = true;
		register_sensor_devinfo(OPLUS_PRESS,&info_press);
		SNS_PRINTF(HIGH, this, "lps22hh pressure register_sensor_devinfo");
	}
  }
}

void lps22hx_update_siblings(sns_sensor *const this)
{
  lps22hx_state *state = (lps22hx_state*)this->state->state;
  bool all_suids_available = true;

  if(   SUID_IS_NULL(&state->acp_suid)
#if !(LPS22HX_POLLING)
     || SUID_IS_NULL(&state->irq_suid)
#endif
     || SUID_IS_NULL(&state->timer_suid)
#if LPS22HX_ENABLE_REGISTRY_ACCESS
     || SUID_IS_NULL(&state->reg_suid)
#endif
    )
  {
    all_suids_available = false;
  }

  if(all_suids_available)
  {
    sns_sensor *lib_sensor;
    sns_service_manager *smgr= this->cb->get_service_manager(this);

    for(lib_sensor = this->cb->get_library_sensor(this, true);
        NULL != lib_sensor;
        lib_sensor = this->cb->get_library_sensor(this, false))
    {
      lps22hx_state *lib_state = (lps22hx_state*)lib_sensor->state->state;

      DBG_PRINT(lib_state->diag_service, lib_sensor, HIGH, __FILENAME__, __LINE__,
                "update_siblings:: sensor=%u", lib_state->sensor);

      if(lib_sensor != this)
      {
        sns_sync_com_port_service *scp_service;

#if !(LPS22HX_POLLING)
        lib_state->irq_suid              = state->irq_suid;
        lib_state->irq_config            = state->irq_config;
#endif
        lib_state->acp_suid              = state->acp_suid;
        lib_state->timer_suid            = state->timer_suid;

        lib_state->sensor_client_present = false;
        lib_state->hw_is_present         = true;
        lib_state->who_am_i              = state->who_am_i;
        lib_state->rigid_body_type       = state->rigid_body_type;
        lib_state->com_port_info         = state->com_port_info;
        lib_state->rail_config           = state->rail_config;

        //sns_memscpy(lib_state->axis_map, sizeof(lib_state->axis_map),
        //            state->axis_map, sizeof(state->axis_map));
        sns_memscpy(lib_state->placement, sizeof(lib_state->placement),
                    state->placement, sizeof(state->placement));
        lib_state->scp_service = scp_service =
          (sns_sync_com_port_service*)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);
        lib_state->pwr_rail_service =
          (sns_pwr_rail_service*)smgr->get_service(smgr, SNS_POWER_RAIL_SERVICE);
        scp_service->api->sns_scp_register_com_port(&lib_state->com_port_info.com_config,
                                                    &lib_state->com_port_info.port_handle);
      }
      publish_registry_attributes(lib_sensor);
      publish_available(lib_sensor);
    }
    sns_sensor_util_remove_sensor_stream(this, &state->fw_stream);
  }
}

