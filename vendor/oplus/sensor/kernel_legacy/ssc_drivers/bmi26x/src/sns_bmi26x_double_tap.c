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
 * @file sns_bmi26x_lowg.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_bmi26x_config.h"

#if BMI26X_CONFIG_ENABLE_DOUBLE_TAP

#include <string.h>
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_bmi26x_sensor.h"
#include "pb_encode.h"
#include "sns_service_manager.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"
#include "sns_bmi26x_ver.h"

/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void
bmi26x_double_tap_publish_attributes(sns_sensor *const this)
{
#if !BMI26X_CONFIG_ENABLE_SEE_LITE
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
                              values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = BMI26X_DOUBLE_TAP_ACTIVE_CURRENT;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
                              values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        static char const op_mode1[] = BMI26X_HIGH_PERF;

        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = op_mode1, .buf_len = sizeof(op_mode1)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
                              values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        static char const proto1[] = "sns_double_tap.proto";

        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = proto1, .buf_len = sizeof(proto1)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
                              values, ARR_SIZE(values), false);
    }

    {
        static char const name[] = "BMI26X_DOUBLE_TAP";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = name, .buf_len = sizeof(name)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
    }
#endif

    {
        static char const type[] = "double_tap";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = type, .buf_len = sizeof(type)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
    }

#if !BMI26X_CONFIG_ENABLE_SEE_LITE
    {
        static char const vendor[] = "BOSCH";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = vendor, .buf_len = sizeof(vendor)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
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
        value.has_sint = true;
        value.sint = BMI26X_SEE_DD_ATTRIB_VERSION;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
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
        value.sint = BMI26X_DOUBLE_TAP_LOWPOER_CURRENT; //uA
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        sns_double_tap_event stream_event =
            sns_double_tap_event_init_default;
        pb_get_encoded_size(&sstate->encoded_event_len,
                            sns_double_tap_event_fields,
                            &stream_event);
        value.has_sint = true;
        value.sint = sstate->encoded_event_len;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
                              values, ARR_SIZE(values), true);
    }
#endif
}

/* See sns_sensor::init */
sns_rc bmi26x_double_tap_init(sns_sensor * const this)
{
    bmi26x_state *sstate = (bmi26x_state *) this->state->state;

    sstate->sensor = BMI26X_DOUBLE_TAP;
    sns_memscpy(&sstate->my_suid,
                sizeof(sstate->my_suid),
                &((sns_sensor_uid) DOUBLE_TAP_SENSOR_SUID),
                sizeof(sns_sensor_uid));

    bmi26x_common_init(this);
    bmi26x_double_tap_publish_attributes(this);

    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_if_ lowg init");

    return SNS_RC_SUCCESS;
}

sns_rc bmi26x_double_tap_deinit(sns_sensor * const this)
{
    UNUSED_VAR(this);
#if !BMI26X_CONFIG_ENABLE_SEE_LITE
    bmi26x_state *sstate = (bmi26x_state*)this->state->state;
    UNUSED_VAR(sstate);
#endif
    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_if_ lowg_deinit>");
    return SNS_RC_SUCCESS;
}

#endif
