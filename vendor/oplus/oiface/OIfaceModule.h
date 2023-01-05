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

#ifndef __SAMPLE_ACTION_BOOSTER_H__
#define __SAMPLE_ACTION_BOOSTER_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>

#include "hypnus_uapi.h"

/* Supproted events maybe for all apps */
#define EVENT_MAJOR               "1"
#define EVENT_MINOR               "2"
#define EVENT_TIMESTAMP           "3" /* deprecated! Should not use. */
#define EVENT_SCENE               "4"
#define EVENT_FPS                 "5"
#define EVENT_FRAME_DROP          "6"
#define EVENT_REFRESH_RATE        "7"
#define EVENT_MODEL_QUALITY       "8"
#define EVENT_EFFECT_QUALITY      "9"
#define EVENT_RESOLUTION          "10"
#define EVENT_PLAYER              "11"
#define EVENT_NETWORK_LATENCY     "12"
#define EVENT_RECORDING           "13"
#define EVENT_URGENT_SIGNAL       "14"
#define EVENT_MULTITHREAD         "43"
#define EVENT_CONTOUR             "52"
#define EVENT_SCENE_EFFECTS       "44"
#define EVENT_TID                 "51"
#define LOW_BATTERY               "low_battery"


#define MAX_DECISIONS      16
#define MAX_KEY_NAME       32
#define MAX_BOARD_NAME     16
#define LOW_BATTERY_LEVEL  20

#define OIFACE_MODULE_TAG       (('O' << 24) | ('I' << 16) | ('M' << 8) | 'N')
#define OIFACE_INTERFACE_TAG    (('O') << 16 | ('I' << 8) | 'I')
#define OIM_TAG                 (('H' << 24) | ('D' << 16) | ('C' << 8) | 'R')

#define DEFAULT_NETWORK_POWER_MODE      0
#define MAX_IM_CALLBACK_DECSION_LEN     1024

#define NR_CLUSTERS     4
#define NR_MIGRATION    3
#define NR_GPUS         1

enum {
    TYPE_STRING = 0, /* maximum 64 bytes */
    TYPE_UINT32,
    TYPE_INT32,
    TYPE_UINT64,
    TYPE_INT64,
    /* TYPE_VECTOR, not supported in interface version 1.0 */
};

/* Add to tail to keep sync with every project */
enum {
    PLAT_MSM8953 = 0,
    PLAT_MSM8976,
    PLAT_MSM8976PRO,
    PLAT_SDM660,
    PLAT_MT6750,
    PLAT_MT6771,
    PLAT_MT6779,
    PLAT_SDM845,
    PLAT_SDM450,
    PLAT_SDM670,
    PLAT_SDM710,
    PLAT_SDM712,
    PLAT_SDM855,
    PLAT_SDM7150,
    PLAT_SDM6125,
    PLAT_SDM7250,
    PLAT_MT6885,
    PLAT_SDM8250,
    PLAT_MT6873,
    PLAT_SDM7125,
    PLAT_SDM7225,
    PLAT_MT6889,
    PLAT_SDM8350,
    PLAT_MT6893,
    PLAT_MT6853,
    PLAT_SDM6115,
    PLAT_MT6833,
    PLAT_SDM4350,
    PLAT_MT6877,
    PLAT_SDM8450,
    PLAT_SDM6225,
    PLAT_MT6983,
    PLAT_UNKNOWN,
    PLAT_NUM,
};

/* max 8 level for smart freq */
#define SMART_FREQ_LEVELS  16
enum {
    SMART_FREQ_BEGIN = 1,           // do not use this enum value

    /* scene definition begin */
    SMART_FREQ_LOW = 2,
    SMART_FREQ_NORMAL,
    SMART_FREQ_HIGH,
    SMART_FREQ_LOW_BATTERY,
    /* scene definition end */

    SMART_FREQ_END = SMART_FREQ_LEVELS - 1,
};

enum {
    HYPNUS_MAGIC_SMART_FREQ_LOW = HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN + SMART_FREQ_LOW,
    HYPNUS_MAGIC_SMART_FREQ_NORMAL = HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN + SMART_FREQ_NORMAL,
    HYPNUS_MAGIC_SMART_FREQ_HIGH = HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN + SMART_FREQ_HIGH,
    HYPNUS_MAGIC_SMART_FREQ_LOW_BATTERY = HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN + SMART_FREQ_LOW_BATTERY,
};

inline int is_mask_for_smart_freq(int mask) {
    if (mask > HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN
        && mask < (HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN + SMART_FREQ_LEVELS))
        return 1;
    return 0;
}

#define mask_to_level(mask) (mask - HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN)
#define level_to_mask(level) (level + HYPNUS_MAGIC_FOR_SMART_FREQ_BEGIN)

/* 30 opp level for cpu should be enough */
#define MAX_CPUFREQ_NUM 32

/* Each module is configured via a 32bit mask. Modules should handle its mask except the MSB bit */
#define MODULE_ENABLE_MASK  (1 << 31)

/* a genenral buffer length definition */
#define MAX_BUFFER_LEN 500
/* XXX: keep sync with that in hypnus */
enum {
    PERF_MODE_UNKNOWN = -1,
    PERF_MODE_NORMAL = 0,
    PERF_MODE_POWER_SAVE = 1,
    PERF_MODE_HIGH_PERF = 2,
};

/* Scene IDs for new hardcoder */
const static uint32_t HARDCODER_SCENE_STARTUP = 1;
const static uint32_t HARDCODER_SCENE_WINDOW_SWITCH = 2;
const static uint32_t HARDCODER_SCENE_WINDOW_SCROLL = 3;
const static uint32_t HARDCODER_SCENE_DATA_LOADING_AND_PROCESS = 4;
const static uint32_t HARDCODER_SCENE_MULTI_MEDIA_PROCESS = 5;
const static uint32_t HARDCODER_SCENE_COMMUNICATE = 6;
const static uint32_t HARDCODER_SCENE_SYSTEM_DEVICE = 7;

/* Scene IDs for old hardcoder */
const static uint32_t HARDCODER_SCENE_BOOT = 101;
const static uint32_t HARDCODER_SCENE_RECEVE_MSG = 201;
const static uint32_t HARDCODER_SCENE_SEND_MSG = 202;
const static uint32_t HARDCODER_SCENE_SEND_PIC_MSG = 203;
const static uint32_t HARDCODER_SCENE_ENTER_CHATTING = 301;
const static uint32_t HARDCODER_SCENE_QUIT_CHATTING = 302;
const static uint32_t HARDCODER_SCENE_UPDATE_CHATROOM = 401;
const static uint32_t HARDCODER_SCENE_SCENE_DB = 501;
const static uint32_t HARDCODER_SCENE_DECODE_PIC = 601;
const static uint32_t HARDCODER_SCENE_GIF = 602;
const static uint32_t HARDCODER_SCENE_ENCODE_VIDEO = 603;
const static uint32_t HARDCODER_SCENE_SNS_SCROLL = 701;
const static uint32_t HARDCODER_SCENE_ALBUM_SCROLL = 702;
const static uint32_t HARDCODER_SCENE_MEDIA_GALLERY_SCROLL = 703;
const static uint32_t HARDCODER_SCENE_AIO_MSG_SLIDE = 704;

/*************************************************************************************************/
/*                                      Protocol Defination                                      */
/*************************************************************************************************/

enum {
    OIFACE_PROTOCOL_JSON = 0, /* Using json as protocol between app and oiface */
    OIFACE_PROTOCOL_OIM, /* Using OIM(oplus interface message) as protocol between app and oiface */
};

/* a list of keys & type need to be parsed by framework */
union json_value {
    int32_t        val_int32;
    uint32_t    val_uint32;
    int64_t        val_int64;
    uint64_t    val_uint64;

    char        val_string[64];
};

struct json_key {
    char key_name[MAX_KEY_NAME];
    int type;
    int oneshot;
    union json_value default_value;
};

/* describe a key:value map */
struct key_value_map {
    struct json_key key;
    union json_value value;
};

/* this prototype is parsed/checked by framework. Strategy module only need to take care of body */
/* framework ensures message 'body' is full and correctly received */
struct oim_message {
    uint32_t    tag;
    uint16_t    version;
    uint16_t    func_id;
    uint32_t    body_len;
    int64_t     request_id;
    uint32_t    caller_tid;
    int64_t     timestamp;
    uint8_t     body[0]; /* body has length of 'body_len' */
} __attribute__((packed));

/* oim function */
enum {
    OIM_FUNC_CHECK_PERMISSION = 1001,       /* 1001 */
    OIM_FUNC_CPU_HIGH_FREQ,                 /* 1002 */
    OIM_FUNC_CANCEL_CPU_HIGH_FREQ,          /* 1003 */
    OIM_FUNC_CPU_CORE_FOR_THREAD,           /* 1004 */
    OIM_FUNC_CANCEL_CPU_CORE_FOR_THREAD,    /* 1005 */
    OIM_FUNC_HIGH_IO_FREQ,                  /* 1006 */
    OIM_FUNC_CANCEL_HIGH_IO_FREQ,           /* 1007 */
    OIM_FUNC_SET_SCREEN_RESOLUTION,         /* 1008 */
    OIM_FUNC_RESET_SCREEN_RESOLUTION,       /* 1009 */
    OIM_FUNC_REG_ANR_CALLBACK,              /* 1010 */
    OIM_FUNC_REG_PRELOAD_BOOT_RESOURCE,     /* 1011 */
    OIM_FUNC_TERMINATE_APP,                 /* 1012 */
    OIM_FUNC_UNIFY_CPU_IO_THREAD_CORE,      /* 1013 */
    FUNC_CANCEL_UNIFY_CPU_IO_THREAD_CORE,   /* 1014 */
    OIM_FUNC_GET_PARAMETERS    = 1019,                /* 1019 */
    OIM_FUNC_NUM,
};

#define OIM_VERSION 4

/* oim response return code */
#define ERR_RET_OK      0
#define RET_LEVEL_1     1
#define RET_LEVEL_2     2
#define RET_LEVEL_3     3
#define RET_CPU_HIGH_FREQ     (1 << 3)
#define RET_HIGH_IO_FREQ      (1 << 4)
#define RET_GPU_HIGH_FREQ     (1 << 5)

#define RET_CPU_HIGH_FREQ_LEVEL_1     (RET_CPU_HIGH_FREQ | RET_LEVEL_1)//0x9
#define RET_CPU_HIGH_FREQ_LEVEL_2     (RET_CPU_HIGH_FREQ | RET_LEVEL_2)
#define RET_CPU_HIGH_FREQ_LEVEL_3     (RET_CPU_HIGH_FREQ | RET_LEVEL_3)
#define RET_HIGH_IO_FREQ_LEVEL_1      (RET_HIGH_IO_FREQ | RET_LEVEL_1)//0x10
#define RET_HIGH_IO_FREQ_LEVEL_2      (RET_HIGH_IO_FREQ | RET_LEVEL_2)
#define RET_HIGH_IO_FREQ_LEVEL_3      (RET_HIGH_IO_FREQ | RET_LEVEL_3)

/* oim response error code */
#define ERR_UNAUTHORIZED (-10001)
#define ERR_FUNCTION_NOT_SUPPORT (-10002)
#define ERR_SERVICE_UNAVAILABLE (-10003)
#define ERR_FAILED_DEPENDENCY (-10004)
#define ERR_PACKAGE_DECODE_FAILED (-10005)
#define ERR_PARAMETERS_WRONG (-10006)
#define ERR_CLIENT_UPGRADE_REQUIRED (-10007)
#define ERR_CLIENT_DISCONNECT (-20001)
#define ERR_CLIENT_RESPONSE (-20002)

/*************************************************************************************************/
/*                                          Decision defination                                  */
/*************************************************************************************************/

/* Decision type correspondence a fixed number, please be careful to modify  */

enum {
    DECISION_TYPE_NONE                = 0,  /* no operation */
    DECISION_TYPE_ACTION              = 1,  /* set action to hypnus */
    DECISION_TYPE_NETWORK             = 2,  /* set network optimize */
    DECISION_TYPE_SCENE               = 3,  /* set scene to hypnus */
    DECISION_TYPE_NORMALIZED_LOAD     = 4,  /* This type is not supported in version 1.0 */
    DECISION_TYPE_AFFINITY            = 5,  /* affinity task to cpu */
    DECISION_TYPE_IO                  = 6,  /* io optimization */
    DECISION_TYPE_MODEM_LOG           = 7,  /* set log */
    DECISION_TYPE_TOUCH               = 8,  /* touch optimization */
    DECISION_TYPE_MARK                = 9,  /* mark start end signal */
    DECISION_TYPE_INTERNAL_SCENE      = 10, /* FIXME: DONT USE simutaneously with hypnus control */
    DECISION_TYPE_REPORT_NETWORK      = 11, /* report network */
    DECISION_TYPE_HYPNUS_CONTROL      = 12, /* set cpu and gpu control to hypnus */
    DECISION_TYPE_SCHED_CONTROL       = 13, /* set sched control */
    DECISION_TYPE_CPUSET              = 14, /* use cgroup to set task */
    DECISION_TYPE_REPORT_GAME_STATUS  = 15, /* report game status*/
    DECISION_TYPE_APP_EVENT           = 16, /* app event*/
    DECISION_TYPE_SCHEDTUNE           = 17, /* sched tune*/
    DECISION_TYPE_FOR_USE             = 18, /* you can use this number for another feature*/
    DECISION_TYPE_LPM                 = 19, /* low power mode */
    DECISION_TYPE_FRAME_BOOST         = 20, /* frame boost feature*/
    DECISION_TYPE_TOUCH_REPORT        = 21, /* touch report optimization*/
    DECISION_TYPE_STRING              = 22, /* hardcoder callback string*/
    DECISION_TYPE_TOP_APP_CTL         = 23, /* protect top app*/
    DECISION_TYPE_KEY_THREAD_TID      = 24, /* notify oiface key thread tid */
    DECISION_TYPE_INPUT_BOOST         = 25, /* CPU input boost*/
    DECISION_TYPE_SET_FPS             = 26, /* set fixed fps to surfaceflinger*/
    DECISION_TYPE_ORMS_CONTROL        = 27, /* set cpu and gpu control to orms */
    DECISION_TYPE_ORMS_ACTION         = 28, /* set action to orms*/
};

struct network_action {
    int32_t power_mode;
    int32_t timeout;
    int32_t reserved[0];
} __attribute__((packed));

struct orms_action {
    const char* scene;
    const char* action_type;
    int32_t timeout;
    int32_t resultCode;
    int32_t reserved[0];
} __attribute__((packed));

struct hypnus_action {
    int32_t action_type;
    int32_t timeout;
    int32_t reserved[0];
} __attribute__((packed));

#define HYPNUS_SCENE_18         18
#define HYPNUS_SCENE_19         19
#define HYPNUS_SCENE_18_STR     "com.tencent.tmgp.sgame"
#define HYPNUS_SCENE_19_STR     "com.tencent.tmgp.sgame60"
#define SGAME_PACKAGE_NAME      "com.tencent.tmgp.sgame"

struct hypnus_scene {
    int32_t pid;
    int32_t sceneNeedRestore;
    const char* name;
    int32_t reserved[0];
} __attribute__((packed));


struct normalized_load_action {
    int32_t cpu_load;
    int32_t gpu_load;
    int32_t io_load;
    int32_t time;
    int32_t reserved[0];
} __attribute__((packed));

struct window_action {
    int32_t action_type;
    int32_t width;
    int32_t height;
} __attribute__((packed));

/* save modem log for debugging */
struct modem_log_action {
    int32_t latency;
    int32_t network_type;
    int32_t reserved[0];
} __attribute__((packed));

enum cluster_type {
    CLUSTER_ALL = 0,
    CLUSTER_BIG,
    CLUSTER_LITTLE,
    CLUSTER_BIG_PLUS,
    CLUSTER_FG_LITTLE,
    CLUSTER_NUM,
};

/* action to lock pid to CPU */
struct affinity_action {
    int32_t cluster_type; /* set one of cluster type */
    int32_t pid;
    int32_t timeout; /* timeout for this affinity action.*/
    int32_t reserved[0];
} __attribute__((packed));

enum {
    NORMAL_IO = 0,
    HIGH_IO = 1,
};

struct io_action {
    int32_t io_type;
    int32_t timeout; /* timeout for this io action */
    int32_t reserved[0];
} __attribute__((packed));

enum {
    TP_REPORT_NORMAL = 0,
    TP_REPORT_FIRST = 1,
    TP_HIGH_SENSITIVITY = 'a',
    TP_NORMAL_SENSITIVITY = '0',
};

enum {
    TP_AREA_ALL = 0,
};

enum lowbattery_status {
    lowbattery_off = 2,
    lowbattery_on = 3,
    lowbattery_tp_on = 4,
    lowbattery_tp_off = 5,
};

struct tpr_action {
    int32_t type;
    int32_t reserved[0];
};

struct tp_action {
    int32_t sensitivity;
    int32_t area_type;
    int32_t area[4];
    int32_t reserved[0];
} __attribute__((packed));

struct set_fps_action {
    int32_t type;
    int32_t mode;
    int32_t reserved[0];
};

enum {
    SOFT_VSYNC = 0,
    HARD_VSYNC = 1,
    GAME_VSYNC = 2,
};

enum {
    MARK_START = 0,
    MARK_STOP,
    MARK_LOAD_START,
    MARK_LOAD_STOP,
};

struct mark_action {
    int32_t  mark_type;
    int32_t reserved[0];
}__attribute__((packed));

struct scene_action {
    int32_t scene;
    int32_t reserved[0];
}__attribute__((packed));

struct scene_effects_action {
    int32_t scene;
    int32_t reserved[0];
}__attribute__((packed));

struct network_report_action {
    int32_t latency;
    int32_t reserved[0];
}__attribute__((packed));

struct sched_action{
    int32_t up_migrate;
    int32_t down_migrate;
    int32_t reserved[0];
}__attribute__((packed));

enum {
    BIND_TYPE_DEFAULT   = 0,
    BIND_TYPE_DIRECT    = 2,
    BIND_TYPE_TGPA   = 3,
};

struct cpuset_action {
    int32_t cluster_type;
    int32_t delay;
    int32_t bind_type;
    int32_t bind_tid;
}__attribute__((packed));

struct top_app_ctl_action
{
    int32_t top_app_cluster;
    int32_t others_app_cluster;
    int32_t reserved[0];
}__attribute__((packed));

struct key_thread_notification
{
    int32_t key_thread_tid;
    int32_t reserved[0];
}__attribute__((packed));

struct im_callback_action {
    int32_t len;
    uint8_t protocolinfo[MAX_IM_CALLBACK_DECSION_LEN];;
}__attribute__((packed));

struct boost_task {
    int id;
    int64_t duration;
};

enum {
    SCHEDTUNE_BG_INDEX_TOP_APP = -1,
    SCHEDTUNE_BG_INDEX_FB0 = 0,
    SCHEDTUNE_BG_INDEX_FB1 = 1,
    SCHEDTUNE_BG_MAX_INDEX,
};

struct schedtune_boost_action {
    int32_t index;      /* index of boost group: -1,0,1 */
    int32_t boost;      /* boost value */
    int32_t defered;    /* defered time */
    int32_t clear_tasks;      /* clear current index(>0) bg tasks */
    int32_t add_tasks;  /* add tasks to bg tasks */
    int32_t count;
    struct boost_task *tasks;
};

enum {
    APP_EVENT_START = 0,
    APP_EVENT_STOP,
};

struct app_event_trigger {
    int32_t type;
    int32_t scene;
}__attribute__((packed));

struct lpm_action {
    int32_t lpm_disabled;
    int32_t reserved[0];
} __attribute__((packed));

enum {
    FRAME_BOOST_STOP = 0,
    FRAME_BOOST_START,
    FRAME_BOOST_SMART_FREQ,
    FRAME_BOOST_FB_AND_SMART_FREQ,
    FRAME_BOOST_WITH_FPS_TRACK,
};

struct frame_boost_action {
    int32_t type;
    int32_t action_type; /* decision type */
    int32_t param[2];
    int32_t reserved[0];
};

struct input_boost_action {
    int32_t input_boost_ms;
    int32_t input_boost_freq[3];
    int32_t reserved[0];
};

struct orms_data_range {
    int32_t max;
    int32_t min;
};

struct orms_cpu_control_data {
    struct orms_data_range core;
    struct orms_data_range freq;
    int32_t control_type;
};

struct orms_gpu_control_data {
    struct orms_data_range core;
    struct orms_data_range freq;
    int32_t control_type;
};

struct orms_cpu_mig_data {
    int32_t migUp;
    int32_t migDown;
};

struct orms_control_info {
    int32_t cpu_cluster_num;
    int32_t gpu_cluster_num;
    struct orms_cpu_control_data cpu_data[NR_CLUSTERS]; // sorted by capacity
    struct orms_gpu_control_data gpu_data[NR_GPUS];
    struct orms_cpu_mig_data cpu_mig_data[NR_MIGRATION];
    int32_t control_mask;
};

struct decision {
    /* should be set to DECISION_TYPE_ACTION \DECISION_TYPE_NETWORK\DECISION_TYPE_SCENE  */
    int type;

    union {
        /* action name and timeout */
        struct hypnus_action action;

        struct orms_action orms;

        struct network_action network;

        struct hypnus_scene scene;

        struct affinity_action affinity;

        struct io_action io;

        /* normalized resource requirement and need to be done in 'time'
         * millisecond. XXX: not supported in interface version 1.0 */
        struct normalized_load_action normalized_load;

        /* trigger a modem log broadcast to capture modem log */
        struct modem_log_action modem_log;

        /* trigger touchpanel sensitivity change */
        struct tp_action tp;

        /* trigger touchpanel report sequence change */
        struct tpr_action tpr;

        /* mark start playing game */
        struct mark_action mark;

        /* scene inside app */
        struct scene_action internal_scene;

        /* network report action */
        struct network_report_action network_report;

        /* hypnus control data */
        struct hypnus_control_data hypnus_ctl;

        /*sched control data */
        struct sched_action sched;

        /* cpuset action */
        struct cpuset_action cpuset;

        /* top app protect action */
        struct top_app_ctl_action top_app_ctl;

        /* key thread notification */
        struct key_thread_notification key_thread;

        /* schedtune boost */
        struct schedtune_boost_action schedtune_boost;

        /* set fps*/
        struct set_fps_action set_fps;

        /* network report action */
        struct scene_effects_action scene_effects;

        /* event trigger for IM and etc. */
        struct app_event_trigger app_event;

        /* lpm enable/disable action */
        struct lpm_action lpm;

        /* frame boost action */
        struct frame_boost_action frame_boost;

        /* CPU input boost action*/
        struct input_boost_action input_boost;

        /* reserved structure */
        void* data[8];

        /* int32_t accesing index */
        int32_t idata[0];
        /* string action of json or something else */
        struct im_callback_action im_callback;

        /* orms control data */
        struct orms_control_info orms_ctl;
    };
};

struct OIfaceInterface;

/* Global structure to describe this module.
 * Every Oiface module should define such a structure globally and set
 * its vairable name to RGMS_NAME.
 * */
struct OIfaceModuleInfo {
    /* corresponding to OIFACE_MODULE_TAG. (corresponding to 'OIMI') */
    uint32_t tag;

    /* version of this interface. Set to 1. */
    uint32_t interface_version;

    /* version of this module. Maintained by module. */
    uint32_t module_version;

    /* name of this module. Pointed memory should not be release during life time */
    const char* name;

    /* supported package names. The last one should be NULL */
    const char** package_name;

    /* Modules can do initializations here. Modules can also store private data to
     * private_data. Module should return a list of json keys to key_list, which
     * needed to be parsed by framework. */
    union {
        int (*initialize)(struct OIfaceInterface* interface, struct json_key** key_list);
        int (*oim_init)(struct OIfaceInterface* interface, void *data);
    };

    /* Called by framework to make a decision. private_data point to the memory
     * stored by 'intialize'. Return numer of decisions made. At most MAX_DECISIONS
     * decisions can be made. */
    union {
        int (*make_decision)(struct OIfaceInterface* interface, const struct key_value_map* list,
                struct decision decision[], int max_decisions);
        int (*oim_make_decision)(struct OIfaceInterface* interface, const struct oim_message *msg,
                struct decision decision[], int max_decisions);
    };

    int protocol;

    int (*deinit)(struct OIfaceInterface* interface);

    int (*is_package_enabled)(uint32_t mask, const char *package_name);

    int (*oim_check_permission)(struct OIfaceInterface* interface, const struct oim_message *msg);
    /* Reserved data */
    int32_t reserved[16];
};

struct OIfaceInterface {
    /* Tag of framework interface. Set to OIFACE_INTERFACE_TAG, corresponding to 'OII'. */
    uint32_t tag;
    /* It's set to 1. */
    uint32_t interface_version;
    /* platform */
    int platform;
    /* stores private_data stored by module. */
    void* private_data;
    /* permission id for checkPermission*/
    const char* permission_id;
    /* package name looking for */
    const char* package_name;
    /* package PID */
    pid_t pid;
    /* package uid */
    uid_t uid;
    /* configuration mask passed to modules */
    uint32_t config_mask;
    /* send log interface, modules can send log to remote logger. */
    int (*send_log)(const char *buf);
    /* board name */
    char board[MAX_BOARD_NAME];
    /* policies parsed from config file */
#define SCENES_NUM 32
#define DECISION_LENG 5
    int32_t parsedDecisionsMap[SCENES_NUM][DECISION_LENG+1];
    /* reserved */
    int32_t reserved[16];
};

#if defined(__cplusplus)
}
#endif

#endif
