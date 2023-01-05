/**
 * @file sns_lsm6ds3c_accel_sensor.c
 *
 * LSM6DS3C Accel virtual Sensor implementation.
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
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_lsm6ds3c_sensor.h"
#include "pb_encode.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"

/**
 * Publish all sensor-specific attributes.
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void lsm6ds3c_acc_publish_attributes(sns_sensor *const this)
{
#if LSM6DS3C_ATTRIBUTE_DISABLED
  lsm6ds3c_state *state = (lsm6ds3c_state*)this->state->state;
  const char type[] = "accel";
  {
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
    value.boolean = state->available;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
  }
#else
  const char type[] = "accel";
  const uint32_t active_current[3] = {ACCEL_LP_MODE_CURRENT, ACCEL_NORMAL_MODE_CURRENT, ACCEL_HIPRF_MODE_CURRENT};
  const uint32_t sleep_current = ACCEL_SLEEP_CURRENT;


  lsm6ds3c_publish_def_attributes(this);
  {
    sns_std_attr_value_data values[] = {SNS_ATTR/*, SNS_ATTR, SNS_ATTR, SNS_ATTR*/};
    values[0].has_flt = true;
    values[0].flt = LSM6DS3C_ODR_832;
    //QC currently we are limiting to 832
    /*values[1].has_flt = true;
    values[1].flt = LSM6DS3C_ODR_1664;
    values[2].has_flt = true;
    values[2].flt = LSM6DS3C_ODR_3328;
    values[3].has_flt = true;
    values[3].flt = LSM6DS3C_ODR_6656;
    */
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ADDITIONAL_LOW_LATENCY_RATES,
       values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    int i;
    for(i = 0; i < ARR_SIZE(values); i++)
    {
      values[i].has_flt = true;
      values[i].flt = lsm6ds3c_accel_resolutions[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
        values, i, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
    int i;
    for(i = 0; i < ARR_SIZE(active_current); i++)
    {
      values[i].has_sint = true;
      values[i].sint = active_current[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
        values, i, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = sleep_current; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const proto1[] = "sns_accel.proto";
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
    values[1].has_sint = true;
    values[1].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
    values[2].has_sint = true;
    values[2].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};

    sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
    range1[0].has_flt = true;
    range1[0].flt = LSM6DS3C_ACCEL_RANGE_2G_MIN;
    range1[1].has_flt = true;
    range1[1].flt = LSM6DS3C_ACCEL_RANGE_2G_MAX;
    values[0].has_subtype = true;
    values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[0].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

    sns_std_attr_value_data range2[] = {SNS_ATTR, SNS_ATTR};
    range2[0].has_flt = true;
    range2[0].flt = LSM6DS3C_ACCEL_RANGE_4G_MIN;
    range2[1].has_flt = true;
    range2[1].flt = LSM6DS3C_ACCEL_RANGE_4G_MAX;
    values[1].has_subtype = true;
    values[1].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[1].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range2, .buf_len = ARR_SIZE(range2) });

    sns_std_attr_value_data range3[] = {SNS_ATTR, SNS_ATTR};
    range3[0].has_flt = true;
    range3[0].flt = LSM6DS3C_ACCEL_RANGE_8G_MIN;
    range3[1].has_flt = true;
    range3[1].flt = LSM6DS3C_ACCEL_RANGE_8G_MIN;
    values[2].has_subtype = true;
    values[2].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[2].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range3, .buf_len = ARR_SIZE(range3) });

    sns_std_attr_value_data range4[] = {SNS_ATTR, SNS_ATTR};
    range4[0].has_flt = true;
    range4[0].flt = LSM6DS3C_ACCEL_RANGE_16G_MIN;
    range4[1].has_flt = true;
    range4[1].flt = LSM6DS3C_ACCEL_RANGE_16G_MAX;
    values[3].has_subtype = true;
    values[3].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[3].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range4, .buf_len = ARR_SIZE(range4) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
        values, ARR_SIZE(values), true);
  }
#endif // LSM6DS3C_ATTRIBUTE_DISABLED
}

/* See sns_sensor::init */
sns_rc lsm6ds3c_accel_init(sns_sensor *const this)
{
  lsm6ds3c_shared_state* shared_state = lsm6ds3c_get_shared_state_from_state(this->state);
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  shared_state->scp_service = (sns_sync_com_port_service*)
    smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);
  lsm6ds3c_instance_config *inst_cfg = &shared_state->inst_cfg;

  sns_sensor_uid* suid = &((sns_sensor_uid)ACCEL_SUID_0);
#if LSM6DS3C_DUAL_SENSOR_ENABLED
  shared_state->hw_idx = this->cb->get_registration_index(this);
  if(shared_state->hw_idx)
    suid = &((sns_sensor_uid)ACCEL_SUID_1);
#endif

  for(uint32_t i = 0; i < ARR_SIZE(inst_cfg->axis_map); i++)
  {
    inst_cfg->axis_map[i].ipaxis = i;
    inst_cfg->axis_map[i].opaxis = i;
    inst_cfg->axis_map[i].invert = false;
  }

  inst_cfg->accel_cal.sensor_type = LSM6DS3C_ACCEL;
  inst_cfg->accel_cal.fac_cal_corr_mat.e00     = 1.0f;
  inst_cfg->accel_cal.fac_cal_corr_mat.e11     = 1.0f;
  inst_cfg->accel_cal.fac_cal_corr_mat.e22     = 1.0f;
  inst_cfg->accel_cal.registry_persist_version = 0;

  inst_cfg->gyro_cal.sensor_type = LSM6DS3C_GYRO;
  inst_cfg->gyro_cal.fac_cal_corr_mat.e00     = 1.0f;
  inst_cfg->gyro_cal.fac_cal_corr_mat.e11     = 1.0f;
  inst_cfg->gyro_cal.fac_cal_corr_mat.e22     = 1.0f;
  inst_cfg->gyro_cal.registry_persist_version = 0;

  // sensor temperature only uses fac_cal_corr_mat.e00 for scaling and
  // fac_cal_bias[0] for bias correction of its single axis output
  inst_cfg->temper_cal.sensor_type = LSM6DS3C_SENSOR_TEMP;
  inst_cfg->temper_cal.fac_cal_corr_mat.e00     = 1.0f;
  inst_cfg->temper_cal.fac_cal_corr_mat.e11     = 1.0f;
  inst_cfg->temper_cal.fac_cal_corr_mat.e22     = 1.0f;
  inst_cfg->temper_cal.registry_persist_version = 0;

  // initialize md defaults
  inst_cfg->md_config.thresh = LSM6DS3C_MD_THRESH;
  inst_cfg->md_config.win    = LSM6DS3C_MD_DUR;

  // initialize avg_to_nominal on power ON
  inst_cfg->avg_to_nominal_ratio    = 1.0f;
  inst_cfg->avg_to_nominal_ratio_g  = 1.0f;

  lsm6ds3c_init_sensor_info(this, suid, LSM6DS3C_ACCEL);
  lsm6ds3c_acc_publish_attributes(this);
  DBG_PRINTF_EX(LOW, this, "accel init");
  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3c_accel_deinit(sns_sensor *const this)
{
  // Turn Sensor OFF.
  // Close COM port.
  // Turn Power Rails OFF.
  // No need to clear lsm6ds3c_state because it will get freed anyway.

  DBG_PRINTF_EX(LOW, this, "accel deinit");
  return SNS_RC_SUCCESS;
}

