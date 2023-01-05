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

#include <string.h>
#include "pb_encode.h"
#include "sns_attribute_util.h"
#include "sns_rc.h"
#include "sns_types.h"
#include "sns_register.h"
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_timer.pb.h"
#include "sns_suid.pb.h"
#include "sns_island_service.h"
#include "sns_alsps_sensor.h"

static void alsps_als_publish_attributes(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    {
        char const name[] = "alsps";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = name, .buf_len = sizeof(name)
        });

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_NAME,
            &value, 1, false);
    }
    {
        char const vendor[] = "oplus";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;
        value.str.arg = &((pb_buffer_arg) {
            .buf = vendor, .buf_len = sizeof(vendor)
        });

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VENDOR,
            &value, 1, false);
    }
    {
        char const type[] = "ambient_light";
        char const type_wise[] = "wise_light";
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.str.funcs.encode = pb_encode_string_cb;

        if ((state->als_type == UNDER_LCD) || (state->als_type == VIRTUAL_UNDER_LCD)) {
            value.str.arg = &((pb_buffer_arg) {
                .buf = type_wise, .buf_len = sizeof(type_wise)
            });
        } else {
            value.str.arg = &((pb_buffer_arg) {
                .buf = type, .buf_len = sizeof(type)
            });
        }

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_TYPE,
            &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = ALSPS_DRIVER_VERSION;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_VERSION,
            &value, 1, false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        char const proto1[] = "sns_ambient_light.proto";
        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = proto1, .buf_len = sizeof(proto1)
        });

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
            values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_flt = true;
        values[0].flt = ALSPS_ALS_RESOLUTION;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
            values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = ALSPS_ALS_ACTIVE_CURRENT;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
            values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = ALSPS_ALS_SLEEP_CURRENT;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT,
            values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR};
        sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
        range1[0].has_flt = true;
        range1[0].flt = ALSPS_ALS_RANGE_MIN;
        range1[1].has_flt = true;
        range1[1].flt = ALSPS_ALS_RANGE_MAX;
        values[0].has_subtype = true;
        values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
        values[0].subtype.values.arg = &((pb_buffer_arg) {
            .buf = range1, .buf_len = ARR_SIZE(range1)
        });

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
            values, ARR_SIZE(values), false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        char const op_mode0[] = ALSPS_LPM;
        char const op_mode1[] = ALSPS_NORMAL;
        values[0].str.funcs.encode = pb_encode_string_cb;
        values[0].str.arg = &((pb_buffer_arg) {
            .buf = op_mode0, .buf_len = sizeof(op_mode0)
        });
        values[1].str.funcs.encode = pb_encode_string_cb;
        values[1].str.arg = &((pb_buffer_arg) {
            .buf = op_mode1, .buf_len = sizeof(op_mode1)
        });

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
            values, ARR_SIZE(values), false);
    }
    {
        float data[ALS_EVENT_SIZE] = {0};
        state->als_encoded_event_len =
            pb_get_encoded_size_sensor_stream_event(data, ALS_EVENT_SIZE);
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_sint = true;
        value.sint = state->als_encoded_event_len;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE,
            &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = false;
        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_DYNAMIC,
            &value, 1, false);
    }
    {
        sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
        value.has_boolean = true;
        value.boolean = true;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR,
            &value, 1, false);
    }
    {
        sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
        values[0].has_sint = true;
        values[0].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
        values[1].has_sint = true;
        values[1].sint = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;

        sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR_TESTS,
            values, ARR_SIZE(values), true);
    }
}

sns_rc alsps_als_init(sns_sensor *const this)
{
    alsps_state *state = (alsps_state*)this->state->state;
    sns_memzero(state, sizeof(alsps_state));

    state->sensor = ALS;
    state->vendor_id = -1;
    state->start_hw_detect = false;
    sns_memscpy(&state->my_suid,
        sizeof(state->my_suid),
        &((sns_sensor_uid)ALSPS_ALS_SUID),
        sizeof(sns_sensor_uid));

    alsps_common_init(this);
    alsps_als_publish_attributes(this);

    return SNS_RC_SUCCESS;
}

sns_rc alsps_als_deinit(sns_sensor *const this)
{
    UNUSED_VAR(this);

    return SNS_RC_SUCCESS;
}

