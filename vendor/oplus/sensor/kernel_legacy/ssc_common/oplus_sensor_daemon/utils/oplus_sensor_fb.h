/******************************************************************
** Copyright (C), 2004-2020, OPLUS Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR
** File: - oplus_sensor_fb.cpp
** Description: Source file for oplus sensor feedback.
** Version: 1.0
** Date : 2020/03/31
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
*******************************************************************/
#pragma once
#include "oplus_list.h"
#include "oplus_fb_utils.h"

#ifndef EINVAL
#define EINVAL 13
#endif

#define EVNET_NUM_MAX 109

struct fb_event_node {
    struct list_head list;
    struct fb_event event;
};

struct fb_event_smem {
    struct fb_event event[EVNET_NUM_MAX];
};

extern void oplus_feedback_init(void);
extern int oplus_copy_to_smem_fifo(void);
extern int oplus_smem_fifo_show(void);
extern int oplus_get_smem_fifo_len(void);
extern void oplus_init_fw_sensor(void);
extern void oplus_show_fw_sensor(void);