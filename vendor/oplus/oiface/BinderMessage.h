#ifndef __BINDER_MESSAGE_H__
#define __BINDER_MESSAGE_H__

#include <binder/IBinder.h>
#include <string>

#include "OIfaceClient.h"
#include "util/FrameStatsTracer.h"

/* BinderMessage is processed by BinderHandler */
enum {
        BINDER_MESSAGE_NEW_CLIENT = 0,
        BINDER_MESSAGE_UPDATE_GAME_INFO,     /* 1 */
        BINDER_MESSAGE_REQUEST_RESOURCE,     /* 2 */
        BINDER_MESSAGE_REGISTER_CLIENT,      /* 3 */
        BINDER_MESSAGE_REGISTER_NOTIFIER,    /* 4 */
        BINDER_MESSAGE_BINDER_DIED,          /* 5 */
        BINDER_MESSAGE_NOTIFY_CALLBACK,      /* 6 inner use, not binder message */
        BINDER_MESSAGE_RUN_PROGRAM,          /* 7 */
        BINDER_MESSAGE_RESTORE_STATE,        /* 8 not a binder message */
        BINDER_MESSAGE_CANCEL_STATE,         /* 9 not a binder message */
        BINDER_MESSAGE_GET_SUPPORT_PACKAGE,  /* 10 */
        BINDER_MESSAGE_FORGROUND,            /* 11 */
        BINDER_MESSAGE_BACKGROUND,           /* 12 */
        BINDER_MESSAGE_HYPNUS_POLL_EVENT,    /* 13 */
        BINDER_MESSAGE_SCREEN_ON,            /* 14 */
        BINDER_MESSAGE_SCREEN_OFF,           /* 15 */
        BINDER_MESSAGE_ENGINE_CLIENT,        /* 16 */
        BINDER_MESSAGE_UPDATE_FRAME_STATS,   /* not a binder message */
        BINDER_MESSAGE_CHARGER,
        BINDER_MESSAGE_SKIN_TEMP,
        BINDER_MESSAGE_BATTERY_TEMP,
        BINDER_MESSAGE_BATTERY_LEVEL,
        BINDER_MESSAGE_UPDATE_ENGINE_INFO,
        BINDER_MESSAGE_FRAME_BOOST,
        BINDER_MESSAGE_SCHEDTUNE_UPDATE_BOOST_TASKS,
        BINDER_MESSAGE_RESTORE_CONNECTIONLESS_STATE,
        BINDER_MESSAGE_FPS_CHANGE,
        BINDER_MESSAGE_BATTERY_CURRENT,
        BINDER_MESSAGE_NOTIFY_TEMP_LEVEL,
        BINDER_MESSAGE_NOTIFY_NORMALIZED_EXEC_TIME,
        BINDER_MESSAGE_NOTIFY_THERMAL_STATUS, /* 30 */
        BINDER_MESSAGE_TASK_MONITOR_TIMEOUT,
        BINDER_MESSAGE_PICK_TASK_TIMEOUT,
        BINDER_MESSAGE_PICK_KEY_TASK_FINISHED,
        BINDER_MESSAGE_SET_LIGHTNING_START_PACKAGE,
        BINDER_MESSAGE_STOP_LIGHTNING_START_PACKAGE,
        BINDER_MESSAGE_COLLECT_EVENT_DATA,
};

struct BinderMessage {
    int what;
    int uid;
    int pid;
    int value;
    android::sp<android::IBinder> binder;
    std::string json;
    union {
        struct FrameStats frameStats;
        struct FrameBoost frameboost;
        TemperatureLevelData tempLevelData;
        int thermalStatusCode;
    } data;

    BinderMessage() {
        what = -1;
        uid = -1;
        pid = -1;
        value = 0;
    }
};

#endif
