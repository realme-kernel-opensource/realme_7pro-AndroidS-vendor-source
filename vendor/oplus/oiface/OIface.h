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
 *
 * ------------------------------- Revision History: ----------------------------
 * <author>                   <date>       <version>   <desc>
 * ------------------------------------------------------------------------------
 */

#ifndef __RESOURCE_GOVERNOR_H__
#define __RESOURCE_GOVERNOR_H__

#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete;      \
    void operator=(const TypeName&) = delete
#endif

#ifdef USE_NEO
#include <log_base.h>
#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "oiface"
#endif
#else
#include <cutils/log.h>
extern int gLogLevel;
#define DEBUG(...)   ALOGD_IF(gLogLevel >= 2, __VA_ARGS__)
#define INFO(...)    ALOGI_IF(gLogLevel >= 1, __VA_ARGS__)
#define ERROR(...)   ALOGE_IF(gLogLevel >= 0, __VA_ARGS__)
#endif

#include "OIfaceModule.h"
#include "api/OIfaceApi.h"

#define OIFACE_PROPERTY                         "persist.sys.oiface.enable"

// change to default on
#define OIFACE_PROPERTY_DEFAULT_VALUE           "2"
#define OIFACE_FEATURE_PROPERTY                 "persist.sys.oiface.feature"
#define OIFACE_CONTROL_PROPERTY                 "persist.sys.oiface.control"
#define OIFACE_DATA_COLLECT_PROPERTY            "persist.sys.oiface.dc"
#define OIFACE_SHOW_CONF_PROPERTY               "persist.sys.oiface.showconf"
#define OIFACE_FEATURE_DEFAULT_VALUE            ((1 << OIFACE_ENABLE_HYPNUS_ACTION) |\
                                                (1 << OIFACE_ENABLE_HYPNUS_SCENE) |\
                                                (1 << OIFACE_ENABLE_WIFI_NETWORK) |\
                                                (1 << OIFACE_ENABLE_AFFINITY) |\
                                                (1 << OIFACE_ENABLE_IO))
#define OIFACE_FEATURE_PROPERTY_PREFIX          "oiface"

#define HYPNUSD_PROPERTY "persist.sys.hypnus.daemon.enable"

#define HQV_PROPERTY                  "debug.vps.disable"
#define HQV_HALF_MODE_PROPERTY        "debug.media.vpp.demo.enable"

#define OIFACE_JSON_SERVER            "@resmon"
#define OIFACE_LOG_SERVER             "@reslog"
#define OIFACE_OIM_SERVER             "@oiface"
#define OIFACE_ENGINE_SERVER          "@engine_boost"
#define OIFACE_CONNECTION_LESS        "oiface.connetionless.client"

#define LPM_DISABLED_PATH           "/sys/module/lpm_levels/parameters/sleep_disabled"
#define CPU_INPUT_BOOST             "/sys/devices/system/cpu/cpu_boost/input_boost_ms"
#define CPU_INPUT_FREQ              "/sys/devices/system/cpu/cpu_boost/input_boost_freq"

#define OIFACE_LOCATION_KEY     "OIFACE_LOCATION"
#define LMH_LIMIT_CNT_PATH_0  "/sys/devices/platform/soc/17d41000.qcom,cpucc/17d41000.qcom,cpucc:qcom,limits-dcvs@0/lmh_limit_cnt"
#define LMH_LIMIT_CNT_PATH_1  "/sys/devices/platform/soc/17d41000.qcom,cpucc/17d41000.qcom,cpucc:qcom,limits-dcvs@1/lmh_limit_cnt"

#define LMH_FREQ_LIMIT_PATH_0  "/sys/devices/platform/soc/17d41000.qcom,cpucc/17d41000.qcom,cpucc:qcom,limits-dcvs@0/lmh_freq_limit"
#define LMH_FREQ_LIMIT_PATH_1  "/sys/devices/platform/soc/17d41000.qcom,cpucc/17d41000.qcom,cpucc:qcom,limits-dcvs@1/lmh_freq_limit"

#define MMC_PATH "/sys/class/devfreq/mmc0/min_freq"
#define SCSI_PATH "/sys/class/scsi_host/host0/../../../clkscale_enable"


#define HYPNUS_SCHEDTUNE_FOREGROUND_BOOST_PATH  "/dev/stune/foreground/schedtune.boost"
#define SCHEDTUNE_TOPAPP_TASKS_PATH      "/dev/stune/top-app/tasks"
#define SCHEDTUNE_TOPAPP_BOOST_PATH      "/dev/stune/top-app/schedtune.boost"
#define SCHEDTUNE_TOPAPP_DEFERED_PATH    "/dev/stune/top-app/schedtune.defered"
#define SCHEDTUNE_FB0_BOOST_PATH      "/dev/stune/fb0/schedtune.boost"
#define SCHEDTUNE_FB0_DEFERED_PATH    "/dev/stune/fb0/schedtune.defered"
#define SCHEDTUNE_FB1_BOOST_PATH      "/dev/stune/fb1/schedtune.boost"
#define SCHEDTUNE_FB1_DEFERED_PATH    "/dev/stune/fb1/schedtune.defered"
#define SCHEDTUNE_TOPAPP_CPUCTL_BOOST_PATH      "/dev/cpuctl/top-app/cpu.uclamp.min"

//fpsgo variance
#define FPSGO_PATH_TARGET_FPS                        "/sys/kernel/fpsgo/fstb/fstb_fps_list"
#define FPSGO_PATH_BHR_OPP                           "/sys/module/fbt_cpu/parameters/bhr_opp"
#define FPSGO_PATH_FLOOR_BOUND                       "/sys/module/fbt_cpu/parameters/floor_bound"
#define FPSGO_PATH_KMIN                              "/sys/module/fbt_cpu/parameters/kmin"
#define FPSGO_PATH_VARIANCE                          "/sys/module/fbt_cpu/parameters/variance"
#define FPSGO_PATH_BHR                               "/sys/module/fbt_cpu/parameters/bhr"
//fpsgo close or on
#define FPSGO_PATH_FPSGO_FORCE_ON_OFF                "/sys/kernel/fpsgo/common/force_onoff"
//fpsgo drm gear
#define FPSGO_PATH_DRAM_CM_MGR                       "/proc/cm_mgr/dbg_cm_mgr"
#define FPSGO_PATH_DRAM_OPP_MIN                      "/proc/perfmgr/boost_ctrl/dram_ctrl/ddr"
//gpu dynamic frequency
#define FPSGO_PATH_GPU_GED_MARGIN_MODE               "/sys/kernel/ged/hal/dvfs_margin_value"
#define FPSGO_PATH_GPU_GED_TIMER_BASE_DVFS_MARGIN    "/sys/kernel/ged/hal/timer_base_dvfs_margin"
#define FPSGO_PATH_GPU_GED_LOADING_BASE_DVFS_STEP    "/sys/kernel/ged/hal/loading_base_dvfs_step"
//fpsgo boost probability
#define FPSGO_PATH_FPSGO_MARGIN_MODE                 "/sys/kernel/fpsgo/fstb/margin_mode"
#define FPSGO_PATH_FPSGO_MARGIN_MODE_DBNC_A          "/sys/kernel/fpsgo/fstb/margin_mode_dbnc_a"
#define FPSGO_PATH_FPSGO_MARGIN_MODE_DBNC_B          "/sys/kernel/fpsgo/fstb/margin_mode_dbnc_b"
//fpsgo sched boost
#define FPSGO_PATH_SCHED_BOOST                       "/sys/devices/system/cpu/sched/sched_boost"
#define FPSGO_PATH_SCHED_UCLAMP_MIN_TA               "/proc/perfmgr/boost_ctrl/eas_ctrl/perfserv_ta_uclamp_min"
#define FPSGO_PATH_FPSGO_LLF_TH                      "/sys/module/fbt_cpu/parameters/loading_th"
#define FPSGO_PATH_FBT_BOOST_TA                      "/sys/kernel/fpsgo/fbt/boost_ta"
//fpsgo rescue
#define FPSGO_PATH_RESCUE_PERCENT                    "/sys/module/fbt_cpu/parameters/rescue_percent"
#define FPSGO_PATH_MIN_RESCUE_PERCENT                "/sys/module/fbt_cpu/parameters/min_rescue_percent"
#define FPSGO_PATH_SHORT_RESCUE_NS                   "/sys/module/fbt_cpu/parameters/short_rescue_ns"
#define FPSGO_PATH_RESCUE_F                          "/sys/module/fbt_cpu/parameters/rescue_enhance_f"
#define FPSGO_PATH_RESCUE_C                          "/sys/module/fbt_cpu/parameters/rescue_opp_c"
#define FPSGO_PATH_ULTRA_RESCUE                      "/sys/kernel/fpsgo/fbt/ultra_rescue"
//fpsgo enable/disable scheduler idle prefer
#define FPSGO_PATH_FPSGO_IDLEPREFER                  "/sys/kernel/fpsgo/fbt/switch_idleprefer"

#define NAND_SWAP_CTL_PATH                  "/proc/nandswap/swap_ctl"
#define OIFACE_DATA_PATH "/data/oiface/liboiface.so"
#define OIFACE_DATA_OPLUS_LIB_PATH   "/data/oplus_lib/liboiface.so"
#if defined(__LP64__)
#define OIFACE_SYSTEM_PATH    "/system_ext/lib64/liboiface.so"
#else
#define OIFACE_SYSTEM_PATH    "/system_ext/lib/liboiface.so"
#endif

// Binder Call To SF
#define REGISTER_LAYER 30000
#define UNREGISTER_LAYER 30001
#define SET_FRAME_BOOST_LAYER 30002

#define KERNEL_GRIP_HANDLE_NODE 8
#define TOUCH_SMOOTH_LEVEL_NODE 24
#define TOUCH_SENSITIVE_LEVEL_NODE 25
#define GAME_SWITCH_ENABLE_NODE 26
#define REPORT_POINT_FIRST_NODE 29

enum {
    OIFACE_ENABLE_HYPNUS_ACTION = 0,
    OIFACE_ENABLE_HYPNUS_SCENE,
    OIFACE_ENABLE_WIFI_NETWORK,
    OIFACE_ENABLE_AFFINITY,
    OIFACE_ENABLE_IO,
    OIFACE_ENABLE_REMOTE_LOG,
    OIFACE_ENABLE_LOG_TO_FILE,
    OIFACE_ENABLE_MODEM_LOG,
    OIFACE_ENABLE_TOUCH,
    OIFACE_ENABLE_TRIGGER_MARK,
    OIFACE_ENABLE_INTERNAL_SCENE,
    OIFACE_ENABLE_REPORT_NETWORK,
    OIFACE_ENABLE_HYPNUS_CONTROL,
    OIFACE_ENABLE_SCHED,
    OIFACE_ENABLE_COUNT,
    OIFACE_ENABLE_SCHEDTUNE,
};

enum {
    FPSGO_COMMAND_SET_TARGETFPS = 0,
    FPSGO_COMMAND_SET_BHR_OPP = 1,
    FPSGO_COMMAND_SET_FLOOR_BOUND = 2,
    FPSGO_COMMAND_SET_KMIN = 3,
    FPSGO_COMMAND_VARIANCE = 4,
    FPSGO_COMMAND_BHR = 5,
    FPSGO_COMMAND_FPSGO_FORCE_ON_OFF = 6,
    FPSGO_COMMAND_DRAM_CM_MGR = 7,
    FPSGO_COMMAND_DRAM_OPP_MIN = 8,
    FPSGO_COMMAND_GPU_GED_MARGIN_MODE = 9,
    FPSGO_COMMAND_GPU_GED_TIMER_BASE_DVFS_MARGIN = 10,
    FPSGO_COMMAND_GPU_GED_LOADING_BASE_DVFS_STEP = 11,
    FPSGO_COMMAND_FPSGO_MARGIN_MODE = 12,
    FPSGO_COMMAND_FPSGO_MARGIN_MODE_DBNC_A = 13,
    FPSGO_COMMAND_FPSGO_MARGIN_MODE_DBNC_B = 14,
    FPSGO_COMMAND_SCHED_BOOST = 15,
    FPSGO_COMMAND_SCHED_UCLAMP_MIN_TA = 16,
    FPSGO_COMMAND_FPSGO_LLF_TH = 17,
    FPSGO_COMMAND_FBT_BOOST_TA = 18,
    FPSGO_COMMAND_RESCUE_PERCENT = 19,
    FPSGO_COMMAND_MIN_RESCUE_PERCENT = 20,
    FPSGO_COMMAND_SHORT_RESCUE_NS = 21,
    FPSGO_COMMAND_RESCUE_F = 22,
    FPSGO_COMMAND_RESCUE_C = 23,
    FPSGO_COMMAND_ULTRA_RESCUE = 24,
    FPSGO_COMMAND_FPSGO_IDLEPREFER = 25,
};

enum SET_HQV_TYPE {
    HQV_TYPE_DEL_CONFIG = 0,
    HQV_TYPE_SET_CONFIG = 1,
    HQV_TYPE_GET_CONFIG = 2,
};

enum DEVICE_OUT_TYPE {
    DEVICE_OUT_WIRED      = 0x01,
    DEVICE_OUT_BLUETOOTH  = 0x02,
};

enum GENERAL_CLIENT_TYPE {
    GENERAL_CLIENT_APP = 0,
    GENERAL_CLIENT_GAME,
    GENERAL_CLIENT_ENGINE,
    GENERAL_CLIENT_ENGINE_GAME,
    GENERAL_CLIENT_COUNT,
};

struct GeneralConfig {
    int general_type;
    int config_enable;
    int load_time;
    int burst_time;
    int load_interval;
    int burst_interval;
    int acc_ratio;
    GeneralConfig() {
        general_type = -1;
        config_enable = 0;
        load_time = 0;
        burst_time = 0;
        load_interval = 0;
        burst_interval = 0;
        acc_ratio = 0;
    }
};

typedef struct decision Decision;
typedef struct json_key JsonKey;
typedef union json_value JsonValue;
typedef struct oim_message OimMessage;

struct FrameBoost {
    int type;
    Decision decision;
};

#define ARRAY_SIZE(x)    (sizeof(x)/sizeof((x)[0]))

extern "C" int oiface_main(void* resp);
extern "C" int oiface_exit(void *);
extern "C" int oiface_receive_data_from_ns(oiface_sys_info* info);

extern "C" struct OifaceVersion {
    uint32_t sdk; /* android platform sdk version, eg: 27 */
    uint32_t reserved[4];
} __attribute__((packed));

#endif
