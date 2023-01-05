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

#include "sns_alsps_feature.h"

bool is_unit_device(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    return state->is_unit_device;
}

static int get_als_type(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    return state->als_type;
}

bool is_als_under_lcd(sns_sensor_instance *inst)
{
    return get_als_type(inst) == UNDER_LCD;
}

bool is_als_virtual_under_lcd(sns_sensor_instance *inst)
{
    return get_als_type(inst) == VIRTUAL_UNDER_LCD;
}

bool is_als_normal(sns_sensor_instance *inst)
{
    return get_als_type(inst) == NORMAL;
}

static int get_ps_type(sns_sensor_instance *this)
{
    alsps_instance_state *state = (alsps_instance_state*)this->state->state;
    return state->ps_type;
}

bool is_ps_type_y(sns_sensor_instance *inst)
{
    return get_ps_type(inst) == PS_TYPE_Y;
}

bool is_ps_normal(sns_sensor_instance *inst)
{
    return get_ps_type(inst) == PS_NORMAL;
}

bool is_ps_under_lcd(sns_sensor_instance *inst)
{
    return get_ps_type(inst) == PS_UNDER_LCD;
}

