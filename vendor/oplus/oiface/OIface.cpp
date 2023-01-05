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

#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <pthread.h>
#include <cutils/properties.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <sys/stat.h>

#ifndef USE_NEO
#include <binder/BinderService.h>
int gLogLevel = 0;
#endif

#include "Utils.h"
#include "ThreadState.h"
#include "PlatformAdaptor.h"
#include "ActivityServiceProxy.h"
#include "OIfaceService.h"
#include "OplusOIfaceService.h"
#include "FrameRescuer.h"
#include "GlobalConfig.h"
#include "feature/gpa/GpaReader.h"
#include "OplusTouchService.h"

using namespace std;
using namespace android;

static void* loop_thread(void*) {
    PlatformAdaptor& adaptor(PlatformAdaptor::getInstance());
    if (adaptor.getPlatform() == PLAT_UNKNOWN) {
        ERROR("platform is unknown");
        return NULL;
    }
    GpaReader& reader(GpaReader::getInstance());
    if(false == reader.isSystemReady()) {
        ERROR("GpaReader system not ready");
        return NULL;
    }
    OIfaceServer& mgr(OIfaceServer::getInstance());
    mgr.joinSocketLoop();
    ERROR("thread exited!!!\n");

    return NULL;

}

#ifdef USE_NEO /* NEO sends wifi/forgroud app information to oiface */
int oiface_receive_data_from_ns(oiface_sys_info* info) {
    ThreadState::getInstance().updateSysInfo(info);

    return 0;
}

int oiface_exit(void *) {
    return 0;
}
#endif

#ifndef USE_NEO
void __attribute__((constructor)) globalInitialize() {
    gLogLevel = property_get_int32(OIFACE_LOG_PROPERTY, 0);
}
#endif

struct OifaceVersion gVersion = {
    ANDROID_VERSION,
    {},
};


#define MTK_GPU_TUNER_CUSTOM_HINT_SET           "/sys/kernel/debug/ged/gpu_tuner/custom_hint_set"
#define MTK_GPU_TUNER_ANISOTROPIC_DISABLE_ALL   "-1 anisotropic_disable 1"
#define MTK_GPU_TUNER_TRILINEAR_DISABLE_ALL     "-1 trilinear_disable 1"
#define MTK_GPU_TUNER_ANISOTROPIC_RESTORE_ALL   "-1 anisotropic_disable 0"
#define MTK_GPU_TUNER_TRILINEAR_RESTORE_ALL     "-1 trilinear_disable 0"
#define MTK_GPU_TUNER_ANISOTROPIC_FILTER_LEVEL_PROPERTY     "debug.anisotropic_filter_level"
static int set_gpu_tuner(const char *setting_buf) {
    DEBUG("set_gpu_tuner: %s", setting_buf);
    int ret = writeFile(MTK_GPU_TUNER_CUSTOM_HINT_SET, string(setting_buf));
    if (ret < 0) {
        ERROR("set_gpu_tuner: failed to write (%s)", setting_buf);
        return -1;
    }
    return 0;
}

static int mtk_gpu_tuner_init() {
    uint32_t gpu_enable = 0, pkg_enabe = 0;
    int platform = PlatformAdaptor::getInstance().getPlatform();

    if (platform != PLAT_MT6771 && platform != PLAT_MT6779 && platform != PLAT_MT6873) {
        INFO("gpu tunner init, platform(%s) is not match, skip!",
                PlatformAdaptor::getInstance().getPlatformName().c_str());
        return -1;
    }

    //set anisotropic and trilinear.
    set_gpu_tuner(MTK_GPU_TUNER_ANISOTROPIC_RESTORE_ALL);
    set_gpu_tuner(MTK_GPU_TUNER_TRILINEAR_RESTORE_ALL);
    //property_set(MTK_GPU_TUNER_ANISOTROPIC_FILTER_LEVEL_PROPERTY, "16");
    if (GlobalConfig::getInstance().getConfig(&gpu_enable, {"gputuner", "enable"}) == 0
            && gpu_enable == 1) {
        if (GlobalConfig::getInstance().getConfig(&pkg_enabe, {"gputuner", "default"}) == 0) {
            if ((pkg_enabe & 1) == 1)
                set_gpu_tuner(MTK_GPU_TUNER_ANISOTROPIC_DISABLE_ALL);
            if ((pkg_enabe & 2) == 2)
                set_gpu_tuner(MTK_GPU_TUNER_TRILINEAR_DISABLE_ALL);
        }
        Json::Value v;
        char buf[256];
        if (GlobalConfig::getInstance().getConfig(&v, {"gputuner", "pkgsettings"}) == 0) {
            for (auto& iter: v.getMemberNames()) {
                if (v[iter].isUInt()) {
                    pkg_enabe = v[iter].asUInt();
                    if ((pkg_enabe & 1) == 1) {
                        snprintf(buf, sizeof(buf), "%s anisotropic_disable 1", iter.c_str());
                        set_gpu_tuner(buf);
                    }
                    if ((pkg_enabe & 2) == 2) {
                        snprintf(buf, sizeof(buf), "%s trilinear_disable 1", iter.c_str());
                        set_gpu_tuner(buf);
                    }
                }
            }
        }
        //set anisotropic level
        // comment out the setting of anisotropic level as MTK recommands.
        // uint32_t aniso_level = 0;
        // if (GlobalConfig::getInstance().getConfig(&aniso_level, {"gputuner", "anisolevel"}) == 0
        //         && aniso_level > 0 && aniso_level <= 16) {
        //     snprintf(buf, sizeof(buf), "%d", aniso_level);
        //     property_set(MTK_GPU_TUNER_ANISOTROPIC_FILTER_LEVEL_PROPERTY, buf);
        // }
    }

    return 0;
}

static int misc_init() {
    FrameRescuer::setTimerSlack();
    intTargetFPSTrace();
    mtk_gpu_tuner_init();

    /* make sure LPM is enabled */
    writeVendorFile(LPM_DISABLED_PATH, "N");
    string content = OplusTouchService::getInstance().readTouchAdjusterNode(0, GAME_SWITCH_ENABLE_NODE);
    if (content.size() <= 0) {
        DEBUG("unable to get %d content", GAME_SWITCH_ENABLE_NODE);
        return 0;
    }

    if (stoi(content) > 0) {
        DEBUG("last oiface exit is abnormal, trying to recover...");
        OplusTouchService::getInstance().writeTouchAdjusterNode(0, GAME_SWITCH_ENABLE_NODE, "0");
        DEBUG("recover done");
    }

    return 0;
}

int oiface_main(void* resp) {
    int ret;
    char buf[PROPERTY_VALUE_MAX];
    char *path = getenv(OIFACE_LOCATION_KEY);
    bool bDup = false;

    ERROR("starting oiface_version:%s from %s\n", GIT_VERSION, path);
    if (path == NULL) { /* very old one. since it can execute here, which is very strange, keep it running */
        path = OIFACE_DATA_PATH;
    } else {
        if (strcmp(path, "data") == 0) { /* be compatible with new one */
            path = OIFACE_DATA_PATH;
        } else if (strcmp(path, "system") == 0) {
            path = OIFACE_SYSTEM_PATH;
        } else {
            path = strdup(path);
            bDup = true;
        }
    }
    setenv(OIFACE_LOCATION_KEY, path, 1);
    DEBUG("Oiface location is %s", path);
    if (bDup) {
        free(path);
        path = NULL;
    }

    misc_init();

    property_get(OIFACE_PROPERTY, buf, OIFACE_PROPERTY_DEFAULT_VALUE);
    if ((strcmp(buf, "1") != 0) && (strcmp(buf, "2") != 0)) {
        ERROR("oiface is not enabled\n");
        return -1;
    }

    /* suppress warning */
    DEBUG("caller provided resp:%p", resp);

#ifdef USE_NEO
    ThreadState::getInstance().setOifaceResponse(OIFACE_SEND_DATA_TO_NS(resp));
#endif

#ifdef USE_OIFACE_SERVICE
    sp<IServiceManager> sm(defaultServiceManager());

    sp<OplusOIfaceService> oplusOifaceService = new OplusOIfaceService();
    status_t code = sm->addService(String16("oplusoiface"), oplusOifaceService, false);
    if (code != NO_ERROR) {
        ERROR("add oplusoiface service failed(%d)", code);
        return -1;
    }

    INFO("add oplusoiface service success");

#ifdef OPLUS_FEATURE_OPPO_OIFACE
    sp<OIfaceService> oifaceService = new OIfaceService();
    status_t code = sm->addService(String16("oiface"), oifaceService, false);
    if (code != NO_ERROR) {
        ERROR("add oiface service failed(%d)", code);
        return -1;
    }

    INFO("add oiface service success");
#endif

#endif

    pthread_t thread;

    ret = pthread_create(&thread, NULL, loop_thread, NULL);
    if (ret) {
        ERROR("create thread failed(%d)\n", ret);
        return -1;
    } else {
        DEBUG("pthread create success");
    }

    pthread_setname_np(thread, "oiface");

#ifndef USE_NEO /* binder thread loop in main */

    ProcessState::self()->setThreadPoolMaxThreadCount(4);
    sp<IServiceManager> sm(defaultServiceManager());
    status_t service_status = android::OK;

#ifdef OPLUS_FEATURE_OPPO_OIFACE
    service_status = sm->addService(String16(OIfaceService::getServiceName()),
                                    new OIfaceService(),
                                    false,
                                    IServiceManager::DUMP_FLAG_PRIORITY_DEFAULT);
    if (service_status != android::OK) {
        ERROR("add oiface service failed(%d)", service_status);
        return -1;
    }
#endif

    service_status = sm->addService(String16(OplusOIfaceService::getServiceName()),
                                    new OplusOIfaceService(),
                                    false,
                                    IServiceManager::DUMP_FLAG_PRIORITY_DEFAULT);
    if (service_status != android::OK) {
        ERROR("add oplusoiface service failed(%d)", service_status);
        return -1;
    }

    ProcessState::self()->startThreadPool();
    ProcessState::self()->giveThreadPoolName();
    IPCThreadState::self()->joinThreadPool();

    ERROR("binder thread exited");
#endif

    return 0;
}
