/******************************************************************
** Copyright (C), 2004-2020, OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_ALGORITHM
** File: - oppo_sensor.c
** Description: Source file for oppo sensor.
** Version: 1.0
** Date : 2019/03/15
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#include "ChipInfoImage.h"
#include "PlatformInfoDefs.h"
#include "smem.h"
#include "DALSys.h"

#include "oppo_sensor.h"
#define SMEM_PROJECT    135
#define SMEM_SENSOR 130

struct sensor_direction dirction_axie[] = {
    { { 0, 0, 0}, {0, 1, 2} },
    { { 1, 0, 0}, {1, 0, 2} },
    { { 1, 1, 0}, {0, 1, 2} },
    { { 0, 1, 1}, {0, 1, 2} },
    { { 0, 1, 0}, {1, 0, 2} },
    { { 1, 0, 1}, {0, 1, 2} },
    { { 0, 0, 1}, {1, 0, 2} },
    { { 1, 1, 1}, {1, 0, 2} },
    { { 1, 1, 0}, {1, 0, 2} },
    { { 0, 0, 1}, {0, 1, 2} },
    { { 1, 0, 0}, {0, 1, 2} },
};

//static ProjectInfoCDTType *g_project = NULL;
static struct sensor_data *g_info = NULL;

void init_sensor_info(void)
{
    uint32 nSize = 0;
    /*
        g_project = (ProjectInfoCDTType *)smem_get_addr(SMEM_PROJECT, &nSize);
        if (g_project == NULL || nSize == 0) {
            g_project = NULL;
        }
    */
    g_info = (struct sensor_data *)smem_get_addr(SMEM_SENSOR, &nSize);
    if (g_info == NULL || nSize == 0) {
        g_info = NULL;
    }
}

/*get_project is not available*/
/*******************************
static unsigned int get_project(void)
{
    if (!g_project) {
        init_sensor_info();
    }

    if (g_project)
        return g_project->nProject;
    else
        return -1;
}

static unsigned int is_project(OPPO_PROJECT project)
{
    return (get_project() == project?1:0);
}

static unsigned char get_Oppo_Boot_Mode(void)
{
    if (!g_project) {
        init_sensor_info();
    }

    if (g_project)
        return g_project->nOppoBootMode;
    else
        return -1;
}
*********************************/

bool oppo_get_sensor_hw(int sensor_id, int sensor_name, struct sensor_hw **hw)
{
    if (sensor_id < 0 || sensor_id >= SENSORS_NUM) {
        return false;
    }
    if (!g_info) {
        init_sensor_info();
    }
    if (g_info && g_info->s_vector[sensor_id].sensor_id == sensor_id) {
        /*return source 0 hw info*/
        if (sensor_name == -1) {
            *hw = &g_info->s_vector[sensor_id].hw[0];
            return true;
        }
        for (int i = 0; i < SOURCE_NUM; i++) {
            if (g_info->s_vector[sensor_id].hw[i].sensor_name & sensor_name) {
                *hw = &g_info->s_vector[sensor_id].hw[i];
                return true;
            }
        }
    }
    return false;
}

bool oppo_get_sensor_feature(int sensor_id, int sensor_name, struct sensor_feature **feature)
{
    bool found = false;
    struct sensor_hw *hw = NULL;
    found = oppo_get_sensor_hw(sensor_id, sensor_name, &hw);
    if (found && hw != NULL) {
        *feature = &hw->feature;
        return true;
    }
    return false;
}

bool oppo_get_virtual_sensor(int sensor_id, struct sensor_algorithm **algo_vect)
{
    if (sensor_id < 0 || sensor_id >= SENSOR_ALGO_NUM) {
        return false;
    }
    if (!g_info) {
        init_sensor_info();
    }
    if (g_info && g_info->a_vector[sensor_id].sensor_id == sensor_id) {
        *algo_vect = &g_info->a_vector[sensor_id];
        return true;
    }
    return false;
}

/*compatible old interface, will be remove later*/
/*
uint32 get_oppo_feature(enum f_index index)
{
    if (!g_project) {
        init_sensor_info();
    }

    if (index < 1 || index > FEATURE_COUNT)
        return 0;

    return g_project?g_project->nFeature[index-1]:0;
}
*/
/*compatible old interface, will be remove later*/
int get_als_type(void)
{
    struct sensor_feature *feature = NULL;
    if (oppo_get_sensor_feature(OPPO_LIGHT, -1, &feature) && feature != NULL) {
        return feature->feature[0];
    }
    return 0;
}



int get_direction(uint8 num, struct sensor_direction *s_dir)
{
    struct sensor_direction *src;
    if (num >= sizeof(dirction_axie) / sizeof(dirction_axie[0])) {
        return -1;
    }

    src = &dirction_axie[num];
    memcpy(s_dir, src, sizeof(struct sensor_direction));
    return 0;
}
