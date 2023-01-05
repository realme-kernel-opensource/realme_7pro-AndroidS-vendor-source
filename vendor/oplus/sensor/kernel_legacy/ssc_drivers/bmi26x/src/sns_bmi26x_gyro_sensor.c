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
 * @file sns_bmi26x_gyro_sensor.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_bmi26x_sensor.h"
#include "pb_encode.h"
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
bmi26x_gyro_publish_attributes(sns_sensor *const this)
{
    {
        static char const type[] = "gyro";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = type, .buf_len = sizeof(type)
        });
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
    }

#if BMI26X_CONFIG_ENABLE_SEE_LITE
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
    }
#else
    bmi26x_sensor_publish_default_registry_attributes(this, BMI26X_GYRO);

#if BMI26X_CONFIG_ENABLE_LOW_LATENCY_RATE
    {
        // Low latency channel use same ODR as traditional channel, no additional
        // rate to publish, if any additional rate to be supported by low latency
        // channel, please publish here.

        // Following is for example only and exercising low latency path via Qualcomm is TBD
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = BMI26X_ODR_800;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ADDITIONAL_LOW_LATENCY_RATES,
                              values, ARR_SIZE(values), false);
    }
#endif


    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = BMI26X_GYRO_RESOLUTION_250DPS;
        values[1].has_flt = true;
        values[1].flt = BMI26X_GYRO_RESOLUTION_500DPS;
        values[2].has_flt = true;
        values[2].flt = BMI26X_GYRO_RESOLUTION_1000DPS;
        values[3].has_flt = true;
        values[3].flt = BMI26X_GYRO_RESOLUTION_2000DPS; //mdps/LSB
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
                              values, ARR_SIZE(values), false);
    }


    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = BMI26X_GYRO_LOWPOWER_CURRENT;
        values[1].has_sint = true;
        values[1].sint = BMI26X_GYRO_ACTIVE_CURRENT;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
                              values, ARR_SIZE(values), false);
    }

    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = BMI26X_GYRO_LOWPOWER_CURRENT; //uA
        sns_publish_attribute(
            this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
    }

    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        static char const proto1[] = "sns_gyro.proto";
        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = proto1, .buf_len = sizeof(proto1)
        });
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
        range1[0].flt = BMI26X_GYRO_RANGE_250_MIN;
        range1[1].has_flt = true;
        range1[1].flt = BMI26X_GYRO_RANGE_250_MAX;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = range1, .buf_len = ARR_SIZE(range1)
        });

        sns_std_attr_value_data range2[] = {SNS_ATTR, SNS_ATTR};
        range2[0].has_flt = true;
        range2[0].flt = BMI26X_GYRO_RANGE_500_MIN;
        range2[1].has_flt = true;
        range2[1].flt = BMI26X_GYRO_RANGE_500_MAX;
        values[1].has_subtype = true;
        values[1].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[1].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = range2, .buf_len = ARR_SIZE(range2)
        });

        sns_std_attr_value_data range3[] = {SNS_ATTR, SNS_ATTR};
        range3[0].has_flt = true;
        range3[0].flt = BMI26X_GYRO_RANGE_1000_MIN;
        range3[1].has_flt = true;
        range3[1].flt = BMI26X_GYRO_RANGE_1000_MAX;
        values[2].has_subtype = true;
        values[2].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[2].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = range3, .buf_len = ARR_SIZE(range3)
        });

        sns_std_attr_value_data range4[] = {SNS_ATTR, SNS_ATTR};
        range4[0].has_flt = true;
        range4[0].flt = BMI26X_GYRO_RANGE_2000_MIN;
        range4[1].has_flt = true;
        range4[1].flt = BMI26X_GYRO_RANGE_2000_MAX;
        values[3].has_subtype = true;
        values[3].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[3].subtype.values.arg =
        &((pb_buffer_arg) {
            .buf = range4, .buf_len = ARR_SIZE(range4)
        });
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
                              values, ARR_SIZE(values), false);
    }


    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR
#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
                                            , SNS_ATTR
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_400)
                                            , SNS_ATTR
#endif
#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_800)
                                            , SNS_ATTR
#endif
                                           };

#if BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
        values[0].has_flt = true;
        values[0].flt = BMI26X_ODR_12P5;

        values[1].has_flt = true;
        values[1].flt = BMI26X_ODR_25;

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_50)
        values[2].has_flt = true;
        values[2].flt = BMI26X_ODR_50;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_100)
        values[3].has_flt = true;
        values[3].flt = BMI26X_ODR_100;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_200)
        values[4].has_flt = true;
        values[4].flt = BMI26X_ODR_200;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_400)
        values[5].has_flt = true;
        values[5].flt = BMI26X_ODR_400;
#endif


#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_800)
        values[6].has_flt = true;
        values[6].flt = BMI26X_ODR_800;
#endif

#else   //BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW
        values[0].has_flt = true;
        values[0].flt = BMI26X_ODR_25;

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_50)
        values[1].has_flt = true;
        values[1].flt = BMI26X_ODR_50;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_100)
        values[2].has_flt = true;
        values[2].flt = BMI26X_ODR_100;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_200)
        values[3].has_flt = true;
        values[3].flt = BMI26X_ODR_200;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_400)
        values[4].has_flt = true;
        values[4].flt = BMI26X_ODR_400;
#endif

#if (BMI26X_CONFIG_GYR_FASTEST_ODR >= BMI26X_ODR_800)
        values[5].has_flt = true;
        values[5].flt = BMI26X_ODR_800;
#endif
#endif  //BMI26X_CONFIG_ENABLE_GYRO_DOWNSAMPLING_SW

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
                              values, ARR_SIZE(values), true);
    }

    //@@@
#endif
}

/* See sns_sensor::init */
sns_rc bmi26x_gyro_init(sns_sensor * const this)
{
    bmi26x_state *sstate = (bmi26x_state *) this->state->state;

    sstate->sensor = BMI26X_GYRO;
    sns_memscpy(&sstate->my_suid,
                sizeof(sstate->my_suid),
                &((sns_sensor_uid) GYRO_SUID ),
                sizeof(sns_sensor_uid));

    sstate->resolution_idx = BMI26X_RANGE_GYR_PM2000DPS;
    sstate->scale_factor = ((uint32_t)BMI26X_SCALE_FACTOR_DATA_GYRO);

    bmi26x_common_init(this);
    bmi26x_gyro_publish_attributes(this);

    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_if_ gyro_init>");

    return SNS_RC_SUCCESS;
}

/** See sns_sensor.h */
sns_rc bmi26x_gyro_deinit(sns_sensor * const this)
{
    UNUSED_VAR(this);
#if !BMI26X_CONFIG_ENABLE_SEE_LITE
    bmi26x_state *sstate = (bmi26x_state*) this->state->state;
    UNUSED_VAR(sstate);
    BMI26X_SENSOR_LOG(MED, this, "<bmi26x_if_ gyro_deinit>");
#endif
    return SNS_RC_SUCCESS;
}

