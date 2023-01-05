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

#include "sns_alsps_sensor.h"
#include "sns_alsps_sensor_instance.h"

bool is_unit_device(sns_sensor_instance *this);

bool is_als_under_lcd(sns_sensor_instance *inst);


bool is_als_virtual_under_lcd(sns_sensor_instance *inst);

bool is_als_normal(sns_sensor_instance *inst);


bool is_ps_type_y(sns_sensor_instance *inst);

bool is_ps_normal(sns_sensor_instance *inst);

bool is_ps_under_lcd(sns_sensor_instance *inst);

