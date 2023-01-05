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
 * @file sns_bmi160.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_bmi160_sensor.h"
#include "pb_encode.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"
#include "sns_bmi160_ver.h"
/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void
bmi160_accel_publish_attributes(sns_sensor *const this)
{
    static char const type[] = "accel";
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg)
                        { .buf = type, .buf_len = sizeof(type) });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }

#if BMI160_CONFIG_ENABLE_SEE_LITE
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
                        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
    }
#else

    bmi160_publish_default_registry_attributes(this);

#if BMI160_CONFIG_ENABLE_LOW_LATENCY_RATE
    {
        // Low latency channel use same ODR as traditional channel, no additional
        // rate to publish, if any additional rate to be supported by low latency
        // channel, please publish here.

        // Following is for example only and exercising low latency path via QC example is TBD
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = BMI160_ODR_800;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ADDITIONAL_LOW_LATENCY_RATES,
                values, ARR_SIZE(values), false);
    }
#endif

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = BMI160_ACCEL_RESOLUTION_2G;
        values[1].has_flt = true;
        values[1].flt = BMI160_ACCEL_RESOLUTION_4G;
        values[2].has_flt = true;
        values[2].flt = BMI160_ACCEL_RESOLUTION_8G;
        values[3].has_flt = true;
        values[3].flt = BMI160_ACCEL_RESOLUTION_16G; //mg/LSB
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = BMI160_ACC_LOWPOWER_CURRENT; //uA
        values[1].has_sint = true;
        values[1].sint = BMI160_ACC_ACTIVE_CURRENT; //uA

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI160_ACC_LOWPOWER_CURRENT; //uA
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        static char const proto1[] = "sns_accel.proto";
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
                values, ARR_SIZE(values), true);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};

        sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
        range1[0].has_flt = true;
        range1[0].flt = BMI160_ACCEL_RANGE_2G_MIN;
        range1[1].has_flt = true;
        range1[1].flt = BMI160_ACCEL_RANGE_2G_MAX;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg =
            &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

        sns_std_attr_value_data range2[] = {SNS_ATTR, SNS_ATTR};
        range2[0].has_flt = true;
        range2[0].flt = BMI160_ACCEL_RANGE_4G_MIN;
        range2[1].has_flt = true;
        range2[1].flt = BMI160_ACCEL_RANGE_4G_MAX;
        values[1].has_subtype = true;
        values[1].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[1].subtype.values.arg =
            &((pb_buffer_arg){ .buf = range2, .buf_len = ARR_SIZE(range2) });

        sns_std_attr_value_data range3[] = {SNS_ATTR, SNS_ATTR};
        range3[0].has_flt = true;
        range3[0].flt = BMI160_ACCEL_RANGE_8G_MIN;
        range3[1].has_flt = true;
        range3[1].flt = BMI160_ACCEL_RANGE_8G_MAX;
        values[2].has_subtype = true;
        values[2].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[2].subtype.values.arg =
            &((pb_buffer_arg){ .buf = range3, .buf_len = ARR_SIZE(range3) });

        sns_std_attr_value_data range4[] = {SNS_ATTR, SNS_ATTR};
        range4[0].has_flt = true;
        range4[0].flt = BMI160_ACCEL_RANGE_16G_MIN;
        range4[1].has_flt = true;
        range4[1].flt = BMI160_ACCEL_RANGE_16G_MAX;
        values[3].has_subtype = true;
        values[3].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[3].subtype.values.arg =
            &((pb_buffer_arg){ .buf = range4, .buf_len = ARR_SIZE(range4) });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR
#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_50)
        , SNS_ATTR
#endif

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_100)
        , SNS_ATTR
#endif
#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_200)
        , SNS_ATTR
#endif

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_400)
        , SNS_ATTR
#endif
        };

        values[0].has_flt = true;
        values[0].flt = BMI160_ODR_12P5;

        values[1].has_flt = true;
        values[1].flt = BMI160_ODR_25;

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_50)
        values[2].has_flt = true;
        values[2].flt = BMI160_ODR_50;
#endif

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_100)
        values[3].has_flt = true;
        values[3].flt = BMI160_ODR_100;
#endif

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_200)
        values[4].has_flt = true;
        values[4].flt = BMI160_ODR_200;
#endif

#if (BMI160_CONFIG_ACC_FASTEST_ODR >= BMI160_ODR_400)
        values[5].has_flt = true;
        values[5].flt = BMI160_ODR_400;
#endif
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
                values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
                this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
    }

//@see-lite
#endif
}

/* See sns_sensor::init */
sns_rc bmi160_accel_init(sns_sensor * const this)
{
    bmi160_state *sstate = (bmi160_state *) this->state->state;

    sstate->sensor = BMI160_ACCEL;
    sns_memscpy(&sstate->my_suid,
            sizeof(sstate->my_suid),
            &((sns_sensor_uid)ACCEL_SUID ),
            sizeof(sns_sensor_uid));

    sstate->resolution_idx = BMI160_RANGE_ACC_PM8G;
    sstate->scale_factor = (uint32_t)(BMI160_SCALE_FACTOR_DATA_ACCEL);

    bmi160_common_init(this);
    bmi160_accel_publish_attributes(this);

    BMI160_SENSOR_LOG(MED, this, "<bmi160_if_ accel_init>");

    return SNS_RC_SUCCESS;
}

sns_rc bmi160_accel_deinit(sns_sensor *const this)
{
    UNUSED_VAR(this);
#if !BMI160_CONFIG_ENABLE_SEE_LITE
    bmi160_state *sstate = (bmi160_state *)this->state->state;
    UNUSED_VAR(sstate);
    BMI160_SENSOR_LOG(MED, this, "<bmi160_if_ accel_deinit>");
#endif

    return SNS_RC_SUCCESS;
}

