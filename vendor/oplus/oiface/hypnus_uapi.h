/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HYPNUS_UAPI_H__
#define _HYPNUS_UAPI_H__

#include <asm-generic/ioctl.h>

/*
 * This File contains all the types used by usrspace
 */
#define NR_CLUSTERS     4
#define NR_GPUS		1
#define HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN         97

enum check_mask_shift {
	FPS_SHIFT = 0,
	THERMAL_SHIFT,
	LOAD_SHIFT,
	RQ_SHIFT,
	VPOWER_SHIFT,
	GPU_SHIFT,//5
	BOOST_SHIFT,
	AFFINITY_SHIFT,
	LPM_GOV_SHIFT,
	SCHED_FULL_BOOST_SHIFT,
	SCHED_CONS_BOOST_SHIFT, //10
	PREFER_IDLE_SHIFT,
	DIS_PACKING_SHIFT,
	PREFER_SMALL_SHIFT,
	STORAGE_SCALING_SHIFT,
	PREFER_BIG_SHIFT, //15
	DISABLE_FPSGO_SHIFT,
	DDR_BOOST_SHIFT,
	IGNORE_DOZEMODE_SHIFT,
	DDR_POWERSAVE_SHIFT,
	DEEPDOZE_DDR_POWERSAVE_SHIFT,//20
	SCHED_REST_BOOST_SHIFT,
	SCHED_UPDOWN_MIGRATE_SHIFT,
};

enum control_type {
	C_LIMIT = 0,
	C_OVERRIDE,
	C_TYPE_MAX,
};

struct uint_range {
	unsigned int max;
	unsigned int min;
};

struct hypnus_cpu_control_data {
	struct uint_range cpufreq;
	struct uint_range cpu_online;
	unsigned int control_type;
};

struct hypnus_gpu_control_data {
	struct uint_range gpufreq;
	struct uint_range gpu_online;
	unsigned int control_type;
};

#define HYP_CONTROL_DATA_VERSION 010

struct hypnus_control_data {
	unsigned int version;
	unsigned int size;
	struct hypnus_cpu_control_data cpu_data[NR_CLUSTERS];// sorted by capacity
	struct hypnus_gpu_control_data gpu_data[NR_GPUS];
	unsigned int control_mask;
};

#define HYPNUS_IOC_MAGIC	0xF4
/*
 * Allow usrspace to change scene inside app dynamically
 * notice that the scene will be override by ActivityManager
 * calling hypnusSetScene
 */
#define HYPNUS_SET_SCENE_INSIDE_APP	_IOWR(HYPNUS_IOC_MAGIC, 1, struct hypnus_control_data)
/*
 * Allow usrspace to send the limit data to hypnus inside APP.
 */
#define HYPNUS_SET_CONTROL_DATA_INSIDE_APP	_IOWR(HYPNUS_IOC_MAGIC, 2, struct hypnus_control_data)
#define HYPNUS_GET_CONTROL_DATA_INSIDE_APP	_IOWR(HYPNUS_IOC_MAGIC, 3, struct hypnus_control_data)
#define HYPNUS_RESTORE_DEFAULT_SETTING _IOWR(HYPNUS_IOC_MAGIC, 4, struct hypnus_control_data)
#define HYPNUS_SET_UPDOWN_MIGRATE_INSIDE_APP _IOWR(HYPNUS_IOC_MAGIC, 5, struct uint_range)
#endif /* _HYPNUS_UAPI_H__ */
