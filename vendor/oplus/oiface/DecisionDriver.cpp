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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <time.h>
#include <dirent.h>
#include <unistd.h>

#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include <utils/Mutex.h>

#include <thread>
#include <chrono>

#include "OIfaceModule.h"
#include "DecisionDriver.h"
#include "Utils.h"
#include "ThreadState.h"
#include <cutils/properties.h>
#include "OIfaceServer.h"
#include "AffinityService.h"
#include "IOService.h"
#include "ActivityServiceProxy.h"
#include "GlobalConfig.h"
#include "OIfaceClient.h"
#include "DataCollector.h"
#include "NetworkLatencyManager.h"
#include "GameStatusObserverManager.h"
#include "hypnus_uapi.h"
#include "PlatformAdaptor.h"
#include "SurfaceFlingerProxyManager.h"
#include "ScreenModeProxy.h"
#include "OifaceCallbackManager.h"
#include "TaskMonitor.h"
#include "TaskManager.h"
#include "OplusTouchService.h"

using namespace android;

#define HYPNUS_SERVICE                  "hypnus"
#define HYPNUSD_SERVICE                 "hypnusd"
#define ACTIVITY_SERVICE                "activity"
#define MODEM_LOG_BROADCAST_ACTION      "android.intent.action.QxdmStopAndPostLog"

// JSHash
constexpr int hashCompileTime(const char *str) {
    unsigned int hash = 1315423911 ;

    if (str != NULL) {
        while(*str) {
            hash^=((hash << 5) + (*str++) + (hash >> 2));
        }
    }
    return(hash % HASH_M);
}

DecisionDriver* DecisionDriver::driver = NULL;

int DecisionDriver::setOrmsAction(const struct orms_action& action) {
    Mutex::Autolock _l(mOrmsLock);
    return OrmsProxy::getInstance().setAction(action);
}

int DecisionDriver::setOrmsMigrate(int up, int down) {
    Mutex::Autolock _l(mOrmsLock);
    orms_control_info ctl_info;
    ctl_info.cpu_cluster_num = PlatformAdaptor::getInstance().getClusterNum();
    ctl_info.gpu_cluster_num = 1;
    ctl_info.cpu_mig_data[0].migUp = up;
    ctl_info.cpu_mig_data[0].migDown = down;
    OrmsProxy::getInstance().setCtrlData(ctl_info);
    return 0;
}

int DecisionDriver::setOrmsCtrlData(const struct orms_control_info &ormsData) {
    Mutex::Autolock _l(mOrmsLock);
    if (ormsData.control_mask == UINT_MAX) {
        DEBUG("restore default settings...");
        OrmsProxy::getInstance().resetCtrlData();
    } else {
        DEBUG("setting orms control...");
        OrmsProxy::getInstance().setCtrlData(ormsData);
    }
    return 0;
}

int DecisionDriver::connectActivityService() {
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->checkService(String16(ACTIVITY_SERVICE));
    if (binder == 0) {
        ERROR("Cannot get activity service");
        return -1;
    }

    mActivity = interface_cast<IActivityServiceProxy>(binder);
    if (mActivity == NULL) {
        ERROR("activity service is not valid\n");
        return -1;
    }

    return 0;
}

DecisionDriver::DecisionDriver() : mActivity(NULL), mEnableMask(0LL), mFd(-1) {
    mLastDecision.type = DECISION_TYPE_NONE;
    mLastUpdateTime = 0LL;
    mFpBoost = NULL;
    mFpDefered = NULL;

    AffinityService::getInstance();

    if (GlobalConfig::getInstance().getConfig(&mEnableMask, {OIFACE_FEATURE_PROPERTY_PREFIX, "enable"}) < 0) {
        ERROR("getConfig for %s failed", OIFACE_FEATURE_PROPERTY_PREFIX);
        return;
    }
    int control = property_get_int32(OIFACE_CONTROL_PROPERTY, -1);
    if (control < 0) {
        DEBUG("oiface feature control is not set");
    } else {
        mEnableMask = control;
    }
    DEBUG("enable mask for oiface is 0x%x", mEnableMask);

    if (mEnableMask & (1 << OIFACE_ENABLE_MODEM_LOG)) {
        if (connectActivityService() < 0) {
            INFO("connect activity service failed. Will retry...\n");
        } else {
            DEBUG("connect activity service success");
        }
    }

    mFgCpus = getSystemFile(CPUSET_FG_CPUS);
}

int DecisionDriver::talkToDriver(const struct hypnus_action& action) {
    INFO("talkToDriver hypnus_action, should never touch here!!!\n");
    return 0;
}

int DecisionDriver::talkToDriver(const struct orms_action& action) {
    if (!isSupportOrms()) {
        ERROR("orms is not available");
        return -1;
    }
    return setOrmsAction(action);
}

int DecisionDriver::talkToDriver(const struct hypnus_scene& scene) {
    INFO("talkToDriver hypnus_scene, should never touch here!!!\n");
    return 0;
}

int DecisionDriver::talkToDriver(const struct network_action& action) {

    if (!(mEnableMask & (1 << OIFACE_ENABLE_WIFI_NETWORK))) {
        ERROR("OIface wifi network not enabled and escaped...\n");
        return -1;
    }

    if (!ThreadState::getInstance().isCurrentWifi()) {
        INFO("current network is not wifi, escaped\n");
        return -1;
    }

    INFO("setting wifi status(%d) to hypnus...\n", action.power_mode);
//    mHypnus->hypnusSetWifiStatus(action.power_mode, action.timeout);
    return 0;
}

int DecisionDriver::talkToDriver(const struct affinity_action &action) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_AFFINITY))) {
        ERROR("OIface affinity not enabled and escaped...\n");
        return -1;
    }

    if (action.pid == 0) {
        ERROR("pid 0 is not currently supported");
        return 0;
    }

    AffinityService &svc(AffinityService::getInstance());
    if (svc.affineTask(action.pid, action.cluster_type, action.timeout)) {
        ERROR("affine task failed\n");
        return -1;
    }

    return 0;
}

int DecisionDriver::talkToDriver(const struct top_app_ctl_action &action) {
    if ((action.top_app_cluster == -1) && (action.others_app_cluster == -1)) {
        //recover default value
        writeFile(CPUSET_FG_CPUS, mFgCpus);
        return 0;
    }

    AffinityService &svc(AffinityService::getInstance());
    if (svc.setAffinityCpuset(action.others_app_cluster, false) < 0) {
        ERROR("set others app cluster failed");
        return -1;
    }

    return 0;
}

int DecisionDriver::talkToDriver(const struct io_action &action) {

    if (!(mEnableMask & (1 << OIFACE_ENABLE_IO))) {
        ERROR("OIface IO not enabled and escaped...\n");
        return -1;
    }
    IOService &svc(IOService::getInstance());
    if (svc.boost(action.timeout) < 0) {
        ERROR("set io boost with timeout:%d failed", action.timeout);
        return -1;
    }
    return 0;
}

int DecisionDriver::talkToDriver(const struct modem_log_action& action) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_MODEM_LOG))) {
        ERROR("OIface modem log not enabled and escaped...\n");
        return -1;
    }

    if (mActivity == NULL) {
        if (connectActivityService() < 0) {
            return -1;
        }
    }

    if (!ThreadState::getInstance().isCurrentData()) {
        INFO("current network is not data, escaped\n");
        return -1;
    }

    string latency = "latency_" + to_string(action.latency);
    sendBroadcast(String16(MODEM_LOG_BROADCAST_ACTION), String16(), String16(), String16(),
        4, "logtype", "440", "logdesc", latency.c_str());

    DEBUG("sent broadcast %s with argument(440, %s)", MODEM_LOG_BROADCAST_ACTION, latency.c_str());

    return 0;
}

int DecisionDriver::talkToDriver(const struct tpr_action& action) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_TOUCH))) {
        ERROR("OIface touch report action not enabled and escaped...\n");
        return -1;
    }

    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, REPORT_POINT_FIRST_NODE, to_string(action.type));
}

int DecisionDriver::talkToDriver(const struct tp_action& action) {
    DEBUG("talkToDriver tp_action");

    if (!(mEnableMask & (1 << OIFACE_ENABLE_TOUCH))) {
        ERROR("OIface touch action not enabled and escaped...\n");
        return -1;
    }

    if (action.area_type != TP_AREA_ALL) {
        ERROR("tp area %d is currently not supported", action.area_type);
        return -1;
    }

    /* may be a 4 byte null-terminated string */
    char buf[5];
    buf[0] = uint32_t(action.sensitivity) & 0xff;
    buf[1] = ((uint32_t(action.sensitivity)) >> 8) & 0xff;
    buf[2] = ((uint32_t(action.sensitivity)) >> 16) & 0xff;
    buf[3] = ((uint32_t(action.sensitivity)) >> 24) & 0xff;
    buf[4] = '\0';
    string plat, prj;
    plat = PlatformAdaptor::getInstance().getPlatformName();
    prj = PlatformAdaptor::getInstance().getProject();
    DEBUG("talkToDriver project %s, platform %s", prj.c_str(), plat.c_str());

    if (strcmp(plat.c_str(),"MT6779") == 0){
        if(prj!=""&&(strcmp(prj.c_str(),"19551") != 0)&&(strcmp(prj.c_str(),"19553") != 0)
        &&(strcmp(prj.c_str(),"19357") != 0)&&(strcmp(prj.c_str(),"19358") != 0)
        &&(strcmp(prj.c_str(),"19359") != 0)&&(strcmp(prj.c_str(),"19354") != 0)
        &&(strcmp(prj.c_str(),"19550") != 0)&&(strcmp(prj.c_str(),"19556") != 0)
        &&(strcmp(prj.c_str(),"19597") != 0)){
            ERROR("project (%s) touch action not support...\n", prj.c_str());
            return -1;
        }
    }
    DEBUG("set project (%s) touch action\n", prj.c_str());

    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, GAME_SWITCH_ENABLE_NODE, buf);
}

int DecisionDriver::talkToDriver(const struct hypnus_control_data& action) {
    INFO("talkToDriver hypnus_control_data, should never touch here!!!\n");
    return -1;
}

int DecisionDriver::talkToDriver(const struct orms_control_info& control_info) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_HYPNUS_CONTROL))) {
        ERROR("OIface hypnus control action not enabled and escaped...\n");
        return -1;
    }

    if (!isSupportOrms()) {
        ERROR("orms is not support");
        return -1;
    }

    return setOrmsCtrlData(control_info);
}


int DecisionDriver::talkToDriver(const struct sched_action& action) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_SCHED))) {
        ERROR("OIface sched action not enabled and escaped...\n");
        return -1;
    }

    if (isSupportOrms()) {
        return setOrmsMigrate(action.up_migrate, action.down_migrate);
    }

    return 0;
}

int clearSchedtuneTasks(int index) {
    char buf[256];
    char currentTask[64];
    const char* parentTask = "/dev/stune/tasks";

    if (index < 0 || index >= SCHEDTUNE_BG_MAX_INDEX)
        return -1;

    snprintf(currentTask, sizeof(currentTask), "/dev/stune/fb%d/tasks", index);

    FILE* fp1 = fopen(currentTask, "r");
    if (fp1 == NULL) {
        ERROR("open %s failed.(%s)", currentTask, strerror(errno));
        return -1;
    }

    vector<int> tasklist;
    while(fgets(buf, sizeof(buf), fp1) != NULL) {
        int pid = atoi(buf);
        if (pid <= 0)
            continue;
        tasklist.push_back(pid);
    }
    fclose(fp1);

    string rootSchetune(parentTask);
    for (int task : tasklist) {
        writeSystemFile(rootSchetune, to_string(task));
        DEBUG("clearSchedtuneTasks(%d) %d", index, task);
    }

    return 0;
}

int addSchedtuneTask(int index, int task) {
    char path[64];

    if (index >= SCHEDTUNE_BG_MAX_INDEX)
        return -1;
    if (index < 0)
        snprintf(path, sizeof(path), SCHEDTUNE_TOPAPP_TASKS_PATH);
    else
        snprintf(path, sizeof(path), "/dev/stune/fb%d/tasks", index);

    DEBUG("addSchedtuneTask(%d) %d", index, task);

    return writeSystemFile(path, to_string(task));
}

int DecisionDriver::setSchedtuneDefered(int index, int defered) {
    char path[64], buf[128];

    if (index >= SCHEDTUNE_BG_MAX_INDEX)
        return -1;

    // FIXME: only support top_app boostgroup
    if (index < 0)
        snprintf(path, sizeof(path), SCHEDTUNE_TOPAPP_DEFERED_PATH);
    else
        snprintf(path, sizeof(path), "/dev/stune/fb%d/schedtune.defered", index);

    if (mFpDefered == NULL) {
        mFpDefered = fopen(path, "w");
        if (mFpDefered == NULL) {
            ERROR("fopen %s failed.(%s)", path, strerror(errno));
            return -1;
        }
    }
    if (fseek(mFpDefered, 0, SEEK_SET) < 0) {
        ERROR("fseek failed.(%s)\n", strerror(errno));
        fclose(mFpDefered);
        mFpDefered = NULL;
        return -1;
    }
    snprintf(buf, sizeof(buf), "%d", defered);
    if (fwrite(buf, sizeof(char), strlen(buf), mFpDefered) == 0) {
        ERROR("fwrite failed.(%s)\n", strerror(errno));
        fclose(mFpDefered);
        mFpDefered = NULL;
        return -1;
    }
    fflush(mFpDefered);

    DEBUG("set schedtune defered %d", defered);

    return 0;
}

int DecisionDriver::setSchedtuneBoost(int index, int boost) {
    char path[64], buf[128];

    if (index >= SCHEDTUNE_BG_MAX_INDEX)
        return -1;
    if(isFileExist(SCHEDTUNE_TOPAPP_CPUCTL_BOOST_PATH))
    {
          DEBUG("set cpuctl enter");
          snprintf(path, sizeof(path), SCHEDTUNE_TOPAPP_CPUCTL_BOOST_PATH);
    }
    // FIXME: only support top_app boostgroup
    else if (index < 0)
        snprintf(path, sizeof(path), SCHEDTUNE_TOPAPP_BOOST_PATH);
    else
        snprintf(path, sizeof(path), "/dev/stune/fb%d/schedtune.boost", index);

    if (mFpBoost == NULL) {
        mFpBoost = fopen(path, "w");
        if (mFpBoost == NULL) {
            ERROR("fopen %s failed.(%s)", path, strerror(errno));
            return -1;
        }
    }
    if (fseek(mFpBoost, 0, SEEK_SET) < 0) {
        ERROR("fseek failed.(%s)\n", strerror(errno));
        fclose(mFpBoost);
        mFpBoost = NULL;
        return -1;
    }
    snprintf(buf, sizeof(buf), "%d", boost);
    if (fwrite(buf, sizeof(char), strlen(buf), mFpBoost) == 0) {
        ERROR("fwrite failed.(%s)\n", strerror(errno));
        fclose(mFpBoost);
        mFpBoost = NULL;
        return -1;
    }
    fflush(mFpBoost);

    DEBUG("set schedtune boost %d", boost);

    return 0;
}

int DecisionDriver::talkToDriver(const struct schedtune_boost_action& action) {
    if (!(mEnableMask & (1 << OIFACE_ENABLE_SCHEDTUNE))) {
        ERROR("OIface schedtune action not enabled and escaped...\n");
        return -1;
    }
    // FIXME: only support top_app boostgroup as fb0/fb1 need to configure with cgroup
    int index = -1; // action.index

    /* update task */
    if (action.clear_tasks) {
        clearSchedtuneTasks(index);
    }
    if (action.add_tasks) {
        for (int i = 0; i < action.count && action.tasks; i++)
            addSchedtuneTask(index, action.tasks[i].id);
    }
    /* set defered */
    if (action.defered != SCHEDTUNE_DEFERED_SKIP_SETTING) {
        int defered = action.defered;
        if (defered > 100)
            defered = 100;
        else if (defered < 0)
            defered = -1;
        setSchedtuneDefered(index, defered);
    }
    /* set boost */
    if (action.boost != SCHEDTUNE_BOOST_SKIP_SETTING) {
        int boost = action.boost;
        if (boost > 100)
            boost = 100;
        else if (boost < 0)
            boost = 0;
        setSchedtuneBoost(index, boost);
    }

    DEBUG("schedtune boost set to driver");

    return 0;
}

/* bind_type: 0:scan forground task 1: scan all task 2:bind specified tid */
void DecisionDriver::setTask(int delay_ms, int cluster_type, int bind_type, std::string task) {
    std::this_thread::sleep_for (std::chrono::milliseconds(delay_ms));
    string cpusetPath;
    vector<int> pidList;

    pidList = getPidByName(task.c_str(), bind_type, true);
    if (pidList.size() == 0) {
        ERROR("affine task %s failed(not found)", task.c_str());
        return;
    }

    if (AffinityService::getInstance().getCpusetInfo(cluster_type, cpusetPath) < 0) {
        ERROR("setTask: get cpuset path failed");
        return;
    }

    for (auto &pid: pidList) {
        DEBUG("tring to affine %s(%d) to %d", task.c_str(), pid, cluster_type);
        if (writeSystemFile(cpusetPath, to_string(pid)) < 0) {
            ERROR("set affinity by cpuset failed");
            return;
        }
    }

    DEBUG("affine %s success", task.c_str());
}
void DecisionDriver::setTaskByCpuFlag(int delay_ms, int cluster_type, int bind_type, std::string task) {
    std::this_thread::sleep_for (std::chrono::milliseconds(delay_ms));
    string cpusetPath;
    vector<int> pidList;
    pidList = getPidByName(task.c_str(), bind_type, true);
    if (pidList.size() == 0) {
        ERROR("affine task %s failed(not found)", task.c_str());
        return;
    }
    for (auto &pid: pidList) {
        DEBUG("tring to affine %s(%d) to %d", task.c_str(), pid, cluster_type);
        if (AffinityService::getInstance().setSchedAffinityByCpu(pid, cluster_type) < 0) {
            ERROR("setTask: get cpuset path failed");
            return;
        }
    }

    DEBUG("affine %s success", task.c_str());
}

void DecisionDriver::setTaskByFgLittle(int delay_ms, int cluster_type, std::string fgCpus, std::string task) {
    std::this_thread::sleep_for (std::chrono::milliseconds(delay_ms));
    if( cluster_type != CLUSTER_FG_LITTLE ){
        return;
    }
    if(task.compare("fg_little")){//recopy mFgCpus.
        writeFile(CPUSET_FG_CPUS,fgCpus);
        return;
    }
    if (AffinityService::getInstance().setAffinityCpuset(4, false) < 0) { // cpu_little is 2 but bit is (100) it is 4;
       ERROR("set others app cluster failed");
       return;
    }
}

void DecisionDriver::bindTGPATask(int delay_ms, int cluster_type, int tid) {
    string cpusetPath;

    if (!AffinityService::getInstance().getTGPABindTaskEnable()) {
        ERROR("bindTGPATask: disabled by cosa");
        return;
    }

    std::this_thread::sleep_for (std::chrono::milliseconds(delay_ms));

    if (AffinityService::getInstance().getCpusetInfo(cluster_type, cpusetPath) < 0) {
        ERROR("bindTGPATask: get cpuset path failed");
        return;
    }

    DEBUG("tring to affine %d to %d", tid, cluster_type);
    if (writeSystemFile(cpusetPath, to_string(tid)) < 0) {
        ERROR("set affinity by cpuset failed");
        return;
    }

    DEBUG("bindTGPATask %d success", tid);
}

void DecisionDriver::directSetTask(int bind_tid, int cluster_type) {
    string cpusetPath;
    DEBUG("tring to set tid %d to %d", bind_tid, cluster_type);
    if (AffinityService::getInstance().getCpusetInfo(cluster_type, cpusetPath) < 0) {
        ERROR("directSetTask: get cpuset path failed");
        return;
    }
    if (writeSystemFile(cpusetPath, to_string(bind_tid)) < 0) {
        ERROR("set affinity by cpuset failed");
        return;
    }
    DEBUG("affine %d success", bind_tid);
}

void DecisionDriver::setKeyTask(int gamePid, int taskTid, int bindValue) {
    DEBUG("trying to set key task, pid: %d, tid: %d, value: %d", gamePid, taskTid, bindValue);
    string taskDir = string("/proc/") + to_string(gamePid) + string("/task/") + to_string(taskTid) + string("/static_ux");
    if (!isFileExist(taskDir.c_str())) {
        DEBUG("%s is not exist", taskDir.c_str());
        taskDir = string("/proc/") + to_string(gamePid) + string("/task/") + to_string(taskTid) + string("/ux_state");
        bindValue = bindValue == 1 ? 129 : bindValue;
    }
    DEBUG("key thread path: %s, value: %d", taskDir.c_str(), bindValue);
    if (writeSystemFile(taskDir, to_string(bindValue)) < 0) {
        ERROR("set affinity by cpuset failed");
        return;
    }
    DEBUG("set key thread %d success.", taskTid);
}

int DecisionDriver::talkToDriver(const struct lpm_action& action) {
    const char *buf = action.lpm_disabled ? "Y" : "N";

    DEBUG("write %s to lpm level", buf);

    return writeFile(LPM_DISABLED_PATH, buf);
}

int DecisionDriver::talkToDriver(const struct frame_boost_action& action,
    const string& clientName) {
    sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
    if (client == NULL) {
        ERROR("fb: invalid client %s", clientName.c_str());
        return -1;
    }

    // Here, frame_boost_action contains another decision which
    // can be hypnus_action, schedtune and others.
    Decision decision = { .type = 0, .data = { 0 } };
    switch (action.action_type) {
        case DECISION_TYPE_ACTION:
            decision.type = DECISION_TYPE_ACTION;
            decision.action.action_type = action.param[0];
            decision.action.timeout = action.param[1];
            break;
        case DECISION_TYPE_SCHEDTUNE:
            decision.type = DECISION_TYPE_SCHEDTUNE;
            decision.schedtune_boost.index = action.param[0];
            decision.schedtune_boost.boost = action.param[1];
            break;
        default:
            ERROR("fb has not support boost with action(%d), use default instead.",
                    action.action_type);
            break;
    }

    // trigger frame boost
    BinderMessage msg;
    msg.what = BINDER_MESSAGE_FRAME_BOOST;
    msg.json = clientName;
    msg.data.frameboost.type = action.type;
    if (action.type & (FB_TYPE_SMART_FREQ_WITH_TIMER | FB_TYPE_SMART_FREQ_WITHOUT_TIMER)) {
        msg.data.frameboost.decision.frame_boost = action;
    } else{
        msg.data.frameboost.decision = decision;
    }
    OIfaceServer::getInstance().sendMessage(msg);

    return 0;
}

int DecisionDriver::talkToDriver(const struct input_boost_action& action, const string& clientName) {
    sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
    if (client == NULL) {
        ERROR("input boost action : invalid client %s", clientName.c_str());
        return -1;
    }
    if (action.input_boost_ms == -1) { //reset to default
        Json::Value state = client->getState("cpu_input_boost");
        if (state.isInt()) {
            DEBUG("reset cpu_input_boost to %dms", state.asInt());
            if (writeFile(CPU_INPUT_BOOST, to_string(state.asInt())) != 0){
                return -1;
            }
        } else{
            return -1;
        }

        state = client->getState("input_boost_freq");
        if(state.isString()) {
            DEBUG("reset input_boost_freq to [%s]", state.asCString());
            if (writeFile(CPU_INPUT_FREQ, state.asCString()) != 0) {
                return -1;
            }
        } else {
            return -1;
        }
        return 0;
    } else {
        string content = getFile(CPU_INPUT_BOOST);
        int cpu_input_boost_ms = std::stoi(content);
        if (cpu_input_boost_ms == action.input_boost_ms) {
            DEBUG("cpu_input_boost same value %d need not set", cpu_input_boost_ms);
            return 0;
        }
        client->setState("cpu_input_boost",cpu_input_boost_ms);
        DEBUG("set cpu_input_boost to %dms and backup the preValue %d", action.input_boost_ms, cpu_input_boost_ms);
        if (writeFile(CPU_INPUT_BOOST, to_string(action.input_boost_ms)) !=0 )
            return -1;

        content = getFile(CPU_INPUT_FREQ);
        client->setState("input_boost_freq", content.c_str());
        int input_freq_mp[8] ={0,};
        string new_content = "";
        map<int, int> clusterLeadingCpu = AffinityService::getInstance().getClusterLeadingCpu();
        input_freq_mp[clusterLeadingCpu[CLUSTER_LITTLE]] = action.input_boost_freq[0];
        input_freq_mp[clusterLeadingCpu[CLUSTER_BIG]] = action.input_boost_freq[1];
        input_freq_mp[clusterLeadingCpu[CLUSTER_BIG_PLUS]] = action.input_boost_freq[2];
        for (int i = 0;i < 8; i++) {
            new_content = new_content + to_string(i) + ":" + to_string(input_freq_mp[i]);
            if (i != 7)
                new_content = new_content + " ";
        }
        DEBUG("set cpu input freq to %s and preValue %s", new_content.c_str(), content.c_str());
        if (writeFile(CPU_INPUT_FREQ, new_content) != 0 )
            return -1;

        return 0;
    }
}

int DecisionDriver::talkToDriver(const struct key_thread_notification& key_thread,
    const string& clientName) {
    sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
    if (client == NULL) {
        ERROR("key thread: invalid client %s", clientName.c_str());
        return -1;
    }

    client->addKeyThreadTid(key_thread.key_thread_tid);
    return 0;
}

int DecisionDriver::talkToDriver(const Decision& decision, const string& clientName, bool needToThrottle) {
    struct timespec res;
    int64_t now;
    android::Mutex::Autolock _l(mLock);

    DEBUG("client %s talking to driver, type:%d size:%d", clientName.c_str(), decision.type,
            (int)sizeof(decision));

#define MAX_TRACE_NAME  256
    char tag[MAX_TRACE_NAME];
    snprintf(tag, MAX_TRACE_NAME, "talkToDriver:%d,%d,%d,%d,%d",
            decision.type, decision.action.action_type,
            decision.action.timeout, decision.action.reserved[0],
            decision.action.reserved[1]);

    ATRACE_NAME(tag);

    clock_gettime(CLOCK_MONOTONIC, &res);
    now = 1000000000LL * res.tv_sec + res.tv_nsec;

    if (needToThrottle && (mLastDecision == decision) &&
            (now - mLastUpdateTime < TALK_TO_DRIVER_INTERVAL)) {
        DEBUG("throttling... talk to driver escaped\n");
        return -1;
    } else {
        DEBUG("not throttling");
    }

    sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
    if (client == NULL) {
        ERROR("invalid client %s", clientName.c_str());
        return -1;
    }

    if (decision.type == DECISION_TYPE_NETWORK) {
        DEBUG("socket %s requested wifi state(%d)\n", clientName.c_str(),
                decision.network.power_mode);

        /* update wifi decision if changes */
        if (talkToDriver(decision.network)) {
            ERROR("talk to driver error\n");
            return -1;
        }

        if (decision.network.power_mode == 1)
            client->setDecisionState("wifi", decision.network.power_mode);
        else
            client->setDecisionState("wifi", Json::nullValue);

    } else if (decision.type == DECISION_TYPE_ACTION) {
        Decision newDecision = decision;

        /* cancel last action  */
        if ((decision.action.timeout == 0) && (decision.action.action_type == -1)) {
            Json::Value state = client->getDecisionState("action_type");
            if (!state.isUInt()) {
                ERROR("unknown state get");
                return -1;
            }

            newDecision.action.action_type = state.asUInt();

            DEBUG("oldDecision type:%d timeout:%d", decision.action.action_type,
                    decision.action.timeout);

            DEBUG("newDecision type:%d timeout:%d", newDecision.action.action_type,
                    newDecision.action.timeout);
        }

        // if (talkToDriver(newDecision.orms) < 0) {
        //     ERROR("talk to driver error\n");
        //     return -1;
        // }
        DEBUG("escape hypnus action");

        // if (talkToDriver(newDecision.action) < 0) {
        //     ERROR("talk to driver error\n");
        //     return -1;
        // }

        /* record last action in order to cancel it later */
        if ((decision.action.action_type != -1) && (decision.action.timeout != 0)) {
            client->setDecisionState("action_type", decision.action.action_type);
        }

    } else if (decision.type == DECISION_TYPE_ORMS_ACTION) {
        Decision ormsDecision = decision;

        // cancel last action
        if (decision.orms.timeout == 0) {
            Json::Value state = client->getDecisionState("orms_action_type");
            Json::Value resultCode = client->getDecisionState("orms_result_code");
            if (!resultCode.isUInt()) {
                ERROR("unknown state get");
                return -1;
            }

            ormsDecision.orms.resultCode = resultCode.asUInt();

            DEBUG("last orms decision type:%s resultCode:%d", state.asString().c_str(), ormsDecision.orms.resultCode);
        }

        DEBUG("DecisionDriver orms final decision time: %d, orms decision time %d.", ormsDecision.orms.timeout, decision.orms.timeout);
        int actionResultCode = talkToDriver(ormsDecision.orms);
        if (actionResultCode < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }

        // record last action in order to cancel it later
        if ((decision.orms.action_type != NULL) && (decision.orms.timeout != 0)) {
            client->setDecisionState("orms_action_type", decision.orms.action_type);
            client->setDecisionState("orms_result_code", actionResultCode);
        }
    }
    else if (decision.type == DECISION_TYPE_SCENE) {
        /* FIXME: fix pid information */
        if (talkToDriver(decision.scene) < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }

        if (ThreadState::getInstance().getForgroundPackage() != decision.scene.name)
            client->setDecisionState("scene", decision.scene.name);
        else
            client->setDecisionState("scene", Json::nullValue);

    } else if (decision.type == DECISION_TYPE_AFFINITY) {
        /* Check if pid allowed */
        OIfaceServer &mgr(OIfaceServer::getInstance());
        sp<OIfaceClient> client = mgr.asOIfaceClient(clientName.c_str());
        if (client == NULL) {
            ERROR("get client for client %s failed", clientName.c_str());
            return -1;
        }

        int callerUid = client->getUid();
        int requestUid = getUidByPid(decision.affinity.pid);
        if (callerUid != requestUid) {
            ERROR("caller uid %d doesn't match request uid %d", callerUid, requestUid);
            return -1;
        }

        if (talkToDriver(decision.affinity) < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_IO) {
        if (talkToDriver(decision.io) < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_MODEM_LOG) {
        if (talkToDriver(decision.modem_log) < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_TOUCH) {
        if (talkToDriver(decision.tp) < 0) {
            ERROR("talk to tp driver error\n");
            return -1;
        }
        if (decision.tp.sensitivity == TP_HIGH_SENSITIVITY)
            client->setDecisionState("touch", TP_HIGH_SENSITIVITY);
        else
            client->setDecisionState("touch", Json::nullValue);

    } else if (decision.type == DECISION_TYPE_TOUCH_REPORT) {
        if (talkToDriver(decision.tpr) < 0) {
            ERROR("talk to tpr driver error\n");
            return -1;
        }
        if (decision.tpr.type == TP_REPORT_FIRST)
            client->setDecisionState("touch_report", TP_REPORT_FIRST);
        else
            client->setDecisionState("touch_report", Json::nullValue);
    } else if (decision.type == DECISION_TYPE_MARK) {
        if (!(mEnableMask & (1 << OIFACE_ENABLE_TRIGGER_MARK))) {
            ERROR("OIface mark type not enabled and escaped...\n");
            return -1;
        }
        /* callback game end info to OS */
		// FIXME map: change map to json; DONE
        Json::Value pkgStat;
        pkgStat["app_status"] = decision.mark.mark_type;
        std::string pkgStatResult = pkgStat.toStyledString();
        DEBUG("json string: %s", pkgStatResult.c_str());
        GameStatusObserverManager::getInstance().reportGameStatus(client->getPackageName(), pkgStatResult);
        OifaceCallbackManager::getInstance().reportGameStatus(client->getPackageName(), pkgStatResult);
#if 0
        binder::Map pkgStat;
        pkgStat["app_status"] = decision.mark.mark_type;
        GameStatusObserverManager::getInstance().reportGameStatus(client->getPackageName(),pkgStat);
#endif

        client->triggerMark(decision.mark.mark_type);
    } else if (decision.type == DECISION_TYPE_APP_EVENT) {
        client->triggerAppEvent(decision.app_event.type, decision.app_event.scene);
    } else if (decision.type == DECISION_TYPE_INTERNAL_SCENE) {
        if (decision.internal_scene.scene <= 0)
            client->setDecisionState("internal_scene", Json::nullValue);
        else
            client->setDecisionState("internal_scene", decision.internal_scene.scene);
    } else if (decision.type == DECISION_TYPE_REPORT_NETWORK) {
        if (!(mEnableMask & (1 << OIFACE_ENABLE_REPORT_NETWORK))) {
            ERROR("OIface modem log not enabled and escaped...\n");
            return -1;
        }

        if (client->isGameStarted()) {
            Json::Value lastNetworkReportTime = client->getState("networkReportTime");
            if (lastNetworkReportTime == Json::nullValue)
                lastNetworkReportTime = 0;

            Json::Value lastNetworkLatency = client->getState("networkLatency");
            if (lastNetworkLatency == Json::nullValue)
                lastNetworkLatency = 0;

            NetworkLatencyManager::getInstance().reportNetworkLatency(client->getPackageName(),
                    decision.network_report.latency, lastNetworkReportTime.asInt(),
                    lastNetworkLatency.asInt());

            client->setState("networkLatency", decision.network_report.latency);
            client->setState("networkReportTime", (int32_t)(systemTime() / 1000000));
        }
    } else if (decision.type == DECISION_TYPE_REPORT_GAME_STATUS) {
        if (client->isGameStarted()) {
            //Todo maybe will add more parameters in the future
			// FIXME map: change map to json; DONE
            Json::Value pkgStat;
            // Json::FastWriter writer;
            pkgStat["scene"] = to_string(DECISION_TYPE_REPORT_GAME_STATUS);
            pkgStat["action_value"] = to_string(decision.scene_effects.scene);
            std::string pkgStatResult = pkgStat.toStyledString();
            DEBUG("json string: %s", pkgStatResult.c_str());
            GameStatusObserverManager::getInstance().reportGameStatus(client->getPackageName(), pkgStatResult);
#if 0
            binder::Map pkgStat;
            pkgStat["scene"] = String16((to_string(DECISION_TYPE_REPORT_GAME_STATUS)).c_str());
            pkgStat["action_value"] = String16((to_string(decision.scene_effects.scene)).c_str());
            GameStatusObserverManager::getInstance().reportGameStatus(client->getPackageName(),pkgStat);
#endif
        }
    } else if (decision.type == DECISION_TYPE_HYPNUS_CONTROL) {
        if (is_mask_for_smart_freq(decision.hypnus_ctl.control_mask)) {
            DEBUG("Set control for smart freq");
            unsigned int magic = decision.hypnus_ctl.control_mask;

            client->setSmartFreqDecision(mask_to_level(magic), decision);
        } else {
            if (talkToDriver(decision.hypnus_ctl) < 0) {
                ERROR("talk to driver failed for hypnus control data");
                return -1;
            }

            Json::Value v(Json::arrayValue);
            int32_t *ptr = (int32_t *)&decision.hypnus_ctl;
            for (int i = 0; i < (int)(sizeof(decision.hypnus_ctl)/sizeof(int32_t)); i++) {
                v.append(ptr[i]);
            }
            if (decision.hypnus_ctl.control_mask == HYPNUS_IOC_MAGIC)
                client->setDecisionState("hypnus_control", Json::nullValue);
            else
                client->setDecisionState("hypnus_control", v);
        }
    } else if (decision.type == DECISION_TYPE_ORMS_CONTROL) {
        if (is_mask_for_smart_freq(decision.orms_ctl.control_mask)) {
            DEBUG("Set control for smart freq");
            unsigned int magic = decision.orms_ctl.control_mask;

            client->setSmartFreqDecision(mask_to_level(magic), decision);
        } else {
            DEBUG("JK-talk to orms control driver!");
            if (talkToDriver(decision.orms_ctl) < 0) {
                ERROR("talk to driver failed for orms control data");
                return -1;
            }

            Json::Value v(Json::arrayValue);
            int32_t *ptr = (int32_t *)&decision.orms_ctl;
            for (int i = 0; i < (int)(sizeof(decision.orms_ctl)/sizeof(int32_t)); i++) {
                v.append(ptr[i]);
            }
            if (decision.orms_ctl.control_mask == ORMS_IOC_MAGIC)
                client->setDecisionState("orms_control", Json::nullValue);
            else
                client->setDecisionState("orms_control", v);
        }
    } else if (decision.type == DECISION_TYPE_SCHED_CONTROL) {
        if(decision.sched.up_migrate >= 0 && decision.sched.up_migrate <= 100) {
            client->setDecisionState("up_migrate", decision.sched.up_migrate);
        } else {
            ERROR("set up_migrate error");
            return -1;
        }
        if(decision.sched.down_migrate >= 0 && decision.sched.down_migrate <= 100) {
            client->setDecisionState("down_migrate", decision.sched.down_migrate);
        } else {
            ERROR("set down_migrate error");
            return -1;
        }
        if (talkToDriver(decision.sched) < 0) {
            ERROR("talk to driver error\n");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_CPUSET) {
        if (decision.cpuset.bind_type == BIND_TYPE_DIRECT) {
            directSetTask(decision.cpuset.bind_tid, decision.cpuset.cluster_type);
        } else if (decision.cpuset.bind_type == BIND_TYPE_TGPA) {
            DEBUG("Bind TGPA tid to big core: %d", decision.cpuset.bind_tid);
            std::thread(bindTGPATask, decision.cpuset.delay, decision.cpuset.cluster_type,
                    decision.cpuset.bind_tid).detach();
        } else  {
            std::string task;
            if (GlobalConfig::getInstance().getConfig(&task,{"sdk", "connectionless", client->getPackageName(), "cluster"}) < 0) {
                DEBUG("no cluster set in %s", client->getPackageName().c_str());
                return -1;
            }
            std::thread(setTask, decision.cpuset.delay, decision.cpuset.cluster_type,
                    decision.cpuset.bind_type, task).detach();
        }
        DEBUG("fired settask for %s", client->getPackageName().c_str());
    } else if (decision.type == DECISION_TYPE_SCHEDTUNE) {
        if (decision.schedtune_boost.boost > 100 || decision.schedtune_boost.defered > 100
                || decision.schedtune_boost.index >= SCHEDTUNE_BG_MAX_INDEX) {
            ERROR("set schedtune boost params are wrong: boost %d, defered %d, index %d",
                    decision.schedtune_boost.boost, decision.schedtune_boost.defered,
                    decision.schedtune_boost.index);
            return -1;
        }
        if (talkToDriver(decision.schedtune_boost) < 0) {
            ERROR("set schedtune boost action error\n");
            return -1;
        }
        /* record status of schedtune to recover if need */
        if (!needToThrottle)
            return 0;
        Decision action = decision;
        action.schedtune_boost.count = 0;
        action.schedtune_boost.tasks = NULL;
        action.schedtune_boost.add_tasks = 0;
        action.schedtune_boost.clear_tasks = 0;
        Json::Value v(Json::arrayValue);
        int *ptr = (int *)&action.schedtune_boost;
        for (int i = 0; i < (int)(sizeof(action.schedtune_boost)/sizeof(int)); i++) {
            v.append(ptr[i]);
        }
        client->setDecisionState("schedtune", v);
    } else if (decision.type == DECISION_TYPE_SET_FPS) {
        DEBUG("set PackageName %s, type: %d, fps: %d", client->getPackageName().c_str(),
            decision.set_fps.type, decision.set_fps.mode);

        /****
        string platform, project;
        platform = PlatformAdaptor::getInstance().getPlatformName();
        project = PlatformAdaptor::getInstance().getProject();
        if ((platform == "") || (strcmp(platform.c_str(), "SDM8250") != 0)
            || (project == "") || (strcmp(project.c_str(),"19161") != 0)) {

            ERROR("project (%s) change screen mode not support...\n", project.c_str());
            return -1;
        }
        ***/

        if (decision.set_fps.type == GAME_VSYNC) {
            ScreenModeProxy::getInstance().setScreenMode(client->getPackageName(), decision.set_fps.mode);
        } else {
            ERROR("unknow set fps type");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_LPM) {
        if (talkToDriver(decision.lpm) < 0) {
            ERROR("set lpm action failed");
            return -1;
        }
        client->setDecisionState("lpm", decision.lpm.lpm_disabled);
    } else if (decision.type == DECISION_TYPE_FRAME_BOOST) {
        if (talkToDriver(decision.frame_boost, clientName) < 0) {
            ERROR("set frame boost action failed");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_TOP_APP_CTL) {
        if (talkToDriver(decision.top_app_ctl) < 0) {
            ERROR("set top app control action failed");
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_KEY_THREAD_TID) {
        if (talkToDriver(decision.key_thread, clientName) < 0) {
            ERROR("Failed to notify key thread tid %d", decision.key_thread.key_thread_tid);
            return -1;
        }
    } else if (decision.type == DECISION_TYPE_INPUT_BOOST) {
        if (talkToDriver(decision.input_boost, clientName) < 0) {
            ERROR("Failed to set cpu input boost action");
            return -1;
        }
    } else {
        ERROR("decision type %d currently not supported\n", decision.type);
        return -1;
    }

    ThreadState::getInstance().sendDecisionInfo(decision);

    snprintf(tag, MAX_TRACE_NAME, "{\"action\":{\"type\":%d, \"action_type\":%d, \"timeout\":%d, \"reserved0\":%d, \"reserved1\":%d, \"timestamp\":%lld}}\n",
            decision.type, decision.action.action_type,
            decision.action.timeout, decision.action.reserved[0],
            decision.action.reserved[1],
            (long long)systemTime(CLOCK_MONOTONIC));

    OIfaceServer::getInstance().sendRemoteLog(tag);
    /* statistics */
    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, client->getPackageName(), "action");

    /* copy last decision */
    mLastDecision = decision;
    mLastUpdateTime = now;

    return 0;
}

int DecisionDriver::setControlData(const Decision& decision) {
    if (talkToDriver(decision.hypnus_ctl) < 0) {
        ERROR("failed for hypnus control data");
        return -1;
    }
    return 0;
}

void DecisionDriver::oifaceDecisionSingle(Json::Value &d) {
    DEBUG("Decision: %s", d.toStyledString().c_str());

    string type = d.get("type", "unknown_decision").asString();

    switch (hashCompileTime(type.c_str())) {
        case (hashCompileTime("game_switch_enable")):
            DEBUG("game switch enable decision");
            oifaceDecisionGameSwitchEnable(d);
            break;
        case (hashCompileTime("schedtune")):
            DEBUG("schedtune decision");
            oifaceDecisionSchedtune(d);
            break;
        case (hashCompileTime("game_status")):
            DEBUG("game status decision");
            oifaceDecisionGameStatus(d);
            break;
        case (hashCompileTime("screen_refresh_rate")):
            DEBUG("Screen refresh rate");
            oifaceDecisionScreenRefreshRate(d);
            break;
        case (hashCompileTime("frame_boost")):
            DEBUG("Frame boost");
            oifaceDecisionFrameBoost(d);
            break;
        case (hashCompileTime("bind_core")):
            DEBUG("Bind core");
            oifaceDecisionBindCore(d);
            break;
        case (hashCompileTime("task_monitor")):
            DEBUG("set task monitor");
            oifaceDecisionSetTaskMonitor(d);
            break;
        case (hashCompileTime("key_thread")):
            DEBUG("set key thread");
            oifaceDecisionSetKeyThread(d);
            break;
        case (hashCompileTime("app_fps")):
            DEBUG("set app fps");
            oifaceDecisionSetAppFps(d);
            break;
        case (hashCompileTime("set_proc_cpuset")):
            DEBUG("set_proc_cpuset");
            oifaceDecisionSetProcCpuset(d);
            break;
        case (hashCompileTime("game_auth_status")):
            DEBUG("set the game auth status");
            oifaceDecisionSetAuthStatus(d);
            break;
        case (hashCompileTime("set_fpsgo")):
            DEBUG("set_fpsgo");
            oifaceDecisionSetFpsgo(d);
            break;
        case (hashCompileTime("cpu_policy")):
            DEBUG("set the game cpu_policy");
            oifaceDecisionSetTargetLoads(d);
            break;
        default:
            ERROR("Unknown decision %s", type.c_str());
            break;
    }
}

void DecisionDriver::oifaceDecisionSetFpsgo(Json::Value v) {
    int task_code = stoi(v.get("task_code", "-1").asString());
    int delayMs = stoi(v.get("delayMs", "0").asString());
    std::string value = v.get("value", "invalid").asString();
    if (task_code == -1) {
        ERROR("Unknown task_code %d", task_code);
        return;
    }
    if (!value.compare("invalid")) {
        ERROR("value  parameter err");
        return;
    }
    std::thread(setFpsgoTask, delayMs, task_code, value).detach();
}

void DecisionDriver::setFpsgoTask(int delay_ms, int task_code, std::string task) {
    DEBUG("setFpsgoTask(%d %d) task:%s", delay_ms, task_code, task.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    switch (task_code) {
        case FPSGO_COMMAND_SET_TARGETFPS:
            DEBUG("set target_fps enter");
            writeFile(FPSGO_PATH_TARGET_FPS, task);
            break;
        case FPSGO_COMMAND_SET_BHR_OPP:
            DEBUG("set bhr_opp enter");
            writeFile(FPSGO_PATH_BHR_OPP, task);
            break;
        case FPSGO_COMMAND_SET_FLOOR_BOUND:
            DEBUG("set floor_bound enter");
            writeFile(FPSGO_PATH_FLOOR_BOUND, task);
            break;
        case FPSGO_COMMAND_SET_KMIN:
            DEBUG("set kmin enter");
            writeFile(FPSGO_PATH_KMIN, task);
            break;
        case FPSGO_COMMAND_VARIANCE:
            DEBUG("set variance enter");
            writeFile(FPSGO_PATH_VARIANCE, task);
            break;
        case FPSGO_COMMAND_BHR:
            DEBUG("set bhr enter");
            writeFile(FPSGO_PATH_BHR, task);
            break;
        case FPSGO_COMMAND_FPSGO_FORCE_ON_OFF:
            DEBUG("set fpsgo onoff enter");
            writeFile(FPSGO_PATH_FPSGO_FORCE_ON_OFF, task);
            break;
        case FPSGO_COMMAND_DRAM_CM_MGR:
            DEBUG("set dram mgr enter");
            writeFile(FPSGO_PATH_DRAM_CM_MGR, task);
            break;
        case FPSGO_COMMAND_DRAM_OPP_MIN:
            DEBUG("set dram opp enter");
            writeFile(FPSGO_PATH_DRAM_OPP_MIN, task);
            break;
        case FPSGO_COMMAND_GPU_GED_MARGIN_MODE:
            DEBUG("set gpu margin mode enter");
            writeFile(FPSGO_PATH_GPU_GED_MARGIN_MODE, task);
            break;
        case FPSGO_COMMAND_GPU_GED_TIMER_BASE_DVFS_MARGIN:
            DEBUG("set dvfs margin enter");
            writeFile(FPSGO_PATH_GPU_GED_TIMER_BASE_DVFS_MARGIN, task);
            break;
        case FPSGO_COMMAND_GPU_GED_LOADING_BASE_DVFS_STEP:
            DEBUG("set dvfs step enter");
            writeFile(FPSGO_PATH_GPU_GED_LOADING_BASE_DVFS_STEP, task);
            break;
        case FPSGO_COMMAND_FPSGO_MARGIN_MODE:
            DEBUG("set fpsgo margin mode enter");
            writeFile(FPSGO_PATH_FPSGO_MARGIN_MODE, task);
            break;
        case FPSGO_COMMAND_FPSGO_MARGIN_MODE_DBNC_A:
            DEBUG("set bdnc a enter");
            writeFile(FPSGO_PATH_FPSGO_MARGIN_MODE_DBNC_A, task);
            break;
        case FPSGO_COMMAND_FPSGO_MARGIN_MODE_DBNC_B:
            DEBUG("set dbnc b enter");
            writeFile(FPSGO_PATH_FPSGO_MARGIN_MODE_DBNC_B, task);
            break;
        case FPSGO_COMMAND_SCHED_BOOST:
            DEBUG("set sched boost enter");
            writeFile(FPSGO_PATH_SCHED_BOOST, task);
            break;
        case FPSGO_COMMAND_SCHED_UCLAMP_MIN_TA:
            DEBUG("set clamp min enter");
            writeFile(FPSGO_PATH_SCHED_UCLAMP_MIN_TA, task);
            break;
        case FPSGO_COMMAND_FPSGO_LLF_TH:
            DEBUG("set loading enter");
            writeFile(FPSGO_PATH_FPSGO_LLF_TH, task);
            break;
        case FPSGO_COMMAND_FBT_BOOST_TA:
            DEBUG("set boot top cground enter");
            writeFile(FPSGO_PATH_FBT_BOOST_TA, task);
            break;
        case FPSGO_COMMAND_RESCUE_PERCENT:
            DEBUG("set rescue percent enter");
            writeFile(FPSGO_PATH_RESCUE_PERCENT, task);
            break;
        case FPSGO_COMMAND_MIN_RESCUE_PERCENT:
            DEBUG("set rescue min percent enter");
            writeFile(FPSGO_PATH_MIN_RESCUE_PERCENT, task);
            break;
        case FPSGO_COMMAND_SHORT_RESCUE_NS:
            DEBUG("set vysnc period enter");
            writeFile(FPSGO_PATH_SHORT_RESCUE_NS, task);
            break;
        case FPSGO_COMMAND_RESCUE_F:
            DEBUG("set rescue floor enter");
            writeFile(FPSGO_PATH_RESCUE_F, task);
            break;
        case FPSGO_COMMAND_RESCUE_C:
            DEBUG("set rescue celing enter");
            writeFile(FPSGO_PATH_RESCUE_C, task);
            break;
        case FPSGO_COMMAND_ULTRA_RESCUE:
            DEBUG("set ultra recue enter");
            writeFile(FPSGO_PATH_ULTRA_RESCUE, task);
            break;
        case FPSGO_COMMAND_FPSGO_IDLEPREFER:
            DEBUG("set idle prefer enter");
            writeFile(FPSGO_PATH_FPSGO_IDLEPREFER, task);
            break;
        default:
            ERROR("Unknown task_code %d", task_code);
            break;
    }
}

void DecisionDriver::oifaceDecisionArray(Json::Value &decisions) {
    for (int i = 0; i < decisions.size(); i++) {
        oifaceDecisionSingle(decisions[i]);
    }
}

void DecisionDriver::oifaceDecision(const string &decisionJson) {
    Json::Value root;
    Json::Reader reader;

    if (!reader.parse(decisionJson, root) || root.isNull()) {
        ERROR("Parse json %s fail", decisionJson.c_str());
        return;
    }
    Json::Value decisions = root.get("decision", Json::Value::null);

    if (decisions.size() > 0) {
        DEBUG("total %d decision found", decisions.size());
        oifaceDecisionArray(decisions);
    } else {
        DEBUG("single decision");
        oifaceDecisionSingle(root);
    }
}

int DecisionDriver::oifaceDecisionBindCore(Json::Value v) {
    string taskName = v.get("task_name", "invalid").asString();
    int clusterType = stoi(v.get("cluster_type", "-1").asString());// 1 big     2 little   3 big plus  >10.signleCore
    int delayMs = stoi(v.get("delayMs", "0").asString());
    int gamePid = stoi(v.get("pid", "0").asString());

    if( (!taskName.compare("invalid")) || clusterType < 0 ||
    (clusterType >= CLUSTER_NUM && clusterType < CPUTYPE_MAX_NUM) ||
    clusterType > 0xff + CPUTYPE_MAX_NUM ){
         ERROR("parameter err");
         return -1;
    }
    if(clusterType <= CLUSTER_BIG_PLUS){
        std::thread(setTask, delayMs, clusterType, gamePid, taskName).detach();
    }else if(clusterType == CLUSTER_FG_LITTLE ){
        std::thread(setTaskByFgLittle,delayMs, clusterType, mFgCpus, taskName).detach();
    }else if(gamePid != 0 && clusterType >= CPUTYPE_MAX_NUM && clusterType <= clusterType + 0xff){
        std::thread(setTaskByCpuFlag, delayMs, clusterType, gamePid, taskName).detach();
    }else{
         ERROR("parameter err");
         return -1;
    }
    return 0;
}

void DecisionDriver::oifaceDecisionSetProcCpuset(Json::Value v) {
    int clusterType = stoi(v.get("cluster_type", "-1").asString());// 1 big     2 little   3 big plus
    int pid = stoi(v.get("pid", "-1").asString());
    string filePath;

    if (clusterType < 0 || pid <= 0 ) {
        ERROR("parameter err");
        return;
    }
    if (AffinityService::getInstance().getCpusetCgroupProcs(clusterType, filePath) < 0) {
        ERROR("SetProcCpuset: get cpuset path failed");
        return;
    }
    if (writeSystemFile(filePath, to_string(pid)) < 0) {
        ERROR("set proc cpuset, write %d to %s failed", pid, filePath.c_str());
        return;
    }
    ERROR("set proc cpuset, write %d to %s success", pid, filePath.c_str());
    return;
}

void DecisionDriver::oifaceDecisionSetAuthStatus(Json::Value v) {
    int authOff = stoi(v.get("game_auth_off", "0").asString());
    if (authOff < 0 || authOff > 1) {
        DEBUG("auth status is invalid");
        return;
    }
    OIfaceServer::getInstance().setAuthOff(authOff);
}

void DecisionDriver::delayedSetTaskMonitor(int gamePid, int timeDuration, int timeThreshold, int fpsThreshold) {
    if (gamePid == 0) {
        std::this_thread::sleep_for (std::chrono::milliseconds(1000)); //wait for 1 second to get gamepid
        gamePid = ThreadState::getInstance().getForgroundPid();
    }
    if (gamePid > 0) {
        TaskMonitor::getInstance().start(gamePid, timeDuration, timeThreshold, fpsThreshold);
    }
}

void DecisionDriver::oifaceDecisionSetKeyThread(Json::Value v) {
    int status = stoi(v.get("status", "0").asString());
    int gamePid = stoi(v.get("pid", "0").asString());
    if (status == 1) {
        int keyWorkerValue = stoi(v.get("key_worker_value", "-1").asString());
        Json::Value keyWorkerArray = v.get("key_worker", Json::nullValue);
        if (keyWorkerArray != Json::nullValue && keyWorkerArray.isArray()) { //record key thread, while list
            for (unsigned int i = 0; keyWorkerValue >= 0 && i < keyWorkerArray.size(); i++) {
                if (keyWorkerArray[i].isString()) {
                    vector<int> taskTidList = getPidByName(keyWorkerArray[i].asString().c_str(), gamePid, true);
                    for (auto taskTid : taskTidList) {
                        TaskManager::getInstance().setWhiteListTask(taskTid, keyWorkerValue);
                        TaskManager::getInstance().setKeyThreadReportInfo(taskTid, keyWorkerValue);
                        DecisionDriver::getInstance().setKeyTask(gamePid, taskTid, keyWorkerValue);
                        DEBUG("set tid %d name %s as key thread.", taskTid, keyWorkerArray[i].asString().c_str());
                    }
                }
            }
        }
        double ratio = stod(v.get("ratio", "-1").asString());
        if (ratio >= 0 && keyWorkerValue >= 0) {
            double descentRate = stod(v.get("descent_rate", "-1").asString());
            int keyWorkerValue = stoi(v.get("key_worker_value", "-1").asString());
            TaskManager::getInstance().setPid(gamePid);
            TaskManager::getInstance().setRatio(ratio);
            TaskManager::getInstance().setDescendRate(descentRate);
            TaskManager::getInstance().setKeyWorkerBindValue(keyWorkerValue);
        }

        int timeDuration = stoi(v.get("time_duration", "-1").asString());
        int timeThreshold = stoi(v.get("time_threshold", "-1").asString());
        int fpsThreshold = stoi(v.get("fps_threshold", "-1").asString());
        int taskCount = stoi(v.get("task_count", "-1").asString());
        int keyThreadValue = stoi(v.get("key_thread_value", "-1").asString());
        TaskManager::getInstance().setKeyThreadBindValue(keyThreadValue);
        if (timeDuration < 0 || timeThreshold < 0 || fpsThreshold < 0 || fpsThreshold > 120 || taskCount < 0 || keyThreadValue < 0) {
            ERROR("parameter is invalid, not set heavy task");
            return;
        }
        std::thread(delayedSetTaskMonitor, gamePid, timeDuration, timeThreshold, fpsThreshold).detach();

        // try to pick heavy thread and set it as key thread
        BinderMessage msg;
        msg.what = BINDER_MESSAGE_TASK_MONITOR_TIMEOUT;
        msg.pid = gamePid;
        msg.json = string("taskCount") + ":" + to_string(taskCount);
        OIfaceServer::getInstance().sendMessageDelayed(msg, TASK_MONITOR_TIMEOUT);
        TaskManager::getInstance().setMonitorRunning(true);
    } else {
        TaskMonitor::getInstance().stop();
        BinderMessage msg;
        msg.what = BINDER_MESSAGE_PICK_TASK_TIMEOUT;
        OIfaceServer::getInstance().removeMessage(msg);
        TaskManager::getInstance().clearAll();
    }
}

int DecisionDriver::oifaceDecisionSetTaskMonitor(Json::Value v) {
    int status = stoi(v.get("status", "0").asString());
    int gamePid = stoi(v.get("pid", "0").asString());
    if (status == 1) {
        int timeDuration = stoi(v.get("time_duration", "-1").asString());
        int timeThreshold = stoi(v.get("time_threshold", "-1").asString());
        int fpsThreshold = stoi(v.get("fps_threshold", "-1").asString());
        if (timeDuration < 0 || timeThreshold < 0 || fpsThreshold < 0 || fpsThreshold > 120) {
            ERROR("parameter err");
            return -1;
        }
        std::thread(delayedSetTaskMonitor, gamePid, timeDuration, timeThreshold, fpsThreshold).detach();
    } else {
        TaskMonitor::getInstance().stop();
    }
    return 0;
}

int DecisionDriver::oifaceDecisionFrameBoost(Json::Value v) {
    std::string packageName = v.get("package", "").asString();
    std::string status = v.get("status", "").asString();

    if (packageName.empty()) {
        DEBUG("empty package");
        return -1;
    }

    if (status.empty()) {
        DEBUG("status unknown");
        return -1;
    }

    BinderMessage msg;
    msg.what = BINDER_MESSAGE_FRAME_BOOST;
    msg.json = packageName;

    switch(status[0]) {
        case '0':
            msg.data.frameboost.type = 0;
            break;
        case '1':
            msg.data.frameboost.type = 1;
            break;
        case '4':
            msg.data.frameboost.type = 4;
            break;
        default:
            return -1;
    }

    OIfaceServer::getInstance().sendMessage(msg);
    return 0;
}


int DecisionDriver::oifaceDecisionScreenRefreshRate(Json::Value v) {
    std::string vsyncType = v.get("vsync_type", "-1").asString();
    std::string fpsMode = v.get("fps_mode", "-1").asString();
    std::string packageName = v.get("package", "com.oplus.oiface").asString();
    DEBUG("vsync type: %s", vsyncType.c_str());
    DEBUG("fps mode: %s", fpsMode.c_str());
    DEBUG("package name: %s", packageName.c_str());

    if (vsyncType[0] < '0' || vsyncType[0] > '2') {
        ERROR("vsync type error %s", vsyncType.c_str());
        return -1;
    }

    if (fpsMode[0] < '0' || fpsMode[0] > '3') {
        ERROR("fps mode error %s", fpsMode.c_str());
        return -1;
    }

    if (!packageName.compare("com.oplus.oiface")) {
        ERROR("packageName empty");
        return -1;
    }

    int mode = fpsMode[0] - '0';
    int vsync = vsyncType[0] - '0';

    DEBUG("set PackageName %s, fps mode: %d", packageName.c_str(), mode);

    if (vsync == GAME_VSYNC) {
        ScreenModeProxy::getInstance().setScreenMode(packageName, mode);
    } else {
        ERROR("unknow set fps type");
        return -1;
    }

    return 0;
}


int DecisionDriver::oifaceDecisionGameStatus(Json::Value v) {
    std::string status = v.get("status", "-1").asString();
    std::string packageName = v.get("package", "com.oplus.oiface").asString();

    int gameStatus = atoi(status.c_str());
    if (gameStatus < 0 || gameStatus > 3) {
        ERROR("oifaceDecisionGameStatus input error: %s", status.c_str());
        return -1;
    }

    // game start stop notify callback
    Json::Value pkgStatus;
    pkgStatus["app_status"] = gameStatus;
    std::string pkgStatResult = pkgStatus.toStyledString();
    DEBUG("json string: %s", pkgStatResult.c_str());

    // we need to notify 2 devices
    GameStatusObserverManager::getInstance().reportGameStatus(packageName, pkgStatResult);
    OifaceCallbackManager::getInstance().reportGameStatus(packageName, pkgStatResult);

    return 0;
}

int DecisionDriver::oifaceDecisionSchedtune(Json::Value v) {
    std::string boost = v.get("boost", "0").asString();
    int b = stoi(boost);
    DEBUG("oifaceDecision Schedtune boost: %d", b);

    if ((b < -100) || (b > 100)) {
        ERROR("Invalid argument");
        return -1;
    }

    setSchedtuneBoost(-1, b);
    return 0;
}

int DecisionDriver::oifaceDecisionGameSwitchEnable(Json::Value v) {
    std::string enable = v.get("value", "-1").asString();
    char buf[5] = {0, };

    DEBUG("oifaceDecisionGameSwitchEnable enable: %s", enable.c_str());

    if (enable[0] == '1')
        buf[0] = 'a';
    else if (enable[0] == '0')
        buf[0] = '0';
    else {
        ERROR("oifaceDecisionGameSwitchEnable input error: %s", enable.c_str());
        return -1;
    }
    DEBUG("Write touchpanel value: %s", buf);

    /* TODO: keep this code, but it shoule be moved to cosa config file */
    string plat, prj;
    plat = PlatformAdaptor::getInstance().getPlatformName();
    prj = PlatformAdaptor::getInstance().getProject();
    DEBUG("talkToDriver project %s, platform %s", prj.c_str(), plat.c_str());

    if (strcmp(plat.c_str(),"MT6779") == 0){
        if(prj!=""&&(strcmp(prj.c_str(),"19551") != 0)&&(strcmp(prj.c_str(),"19553") != 0)
        &&(strcmp(prj.c_str(),"19357") != 0)&&(strcmp(prj.c_str(),"19358") != 0)
        &&(strcmp(prj.c_str(),"19359") != 0)&&(strcmp(prj.c_str(),"19354") != 0)
        &&(strcmp(prj.c_str(),"19550") != 0)&&(strcmp(prj.c_str(),"19556") != 0)
        &&(strcmp(prj.c_str(),"19536") != 0)&&(strcmp(prj.c_str(),"19537") != 0)
        &&(strcmp(prj.c_str(),"19538") != 0)&&(strcmp(prj.c_str(),"19539") != 0)
        &&(strcmp(prj.c_str(),"19597") != 0)){
            ERROR("project (%s) touch action not support...\n", prj.c_str());
            return -1;
        }
    }
    /* TODO: keep this code, but it shoule be moved to cosa config file */

    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, GAME_SWITCH_ENABLE_NODE, buf);
}

int DecisionDriver::oifaceDecisionSetAppFps(Json::Value v) {
    std::string packageName = v.get("package", "invalid").asString();
    std::string temp = v.get("temperature", "unknow").asString();
    int fps = stoi(v.get("fps", "-1").asString());
    INFO("set appFps %s, temp[%s],fps [%d]", packageName.c_str(), temp.c_str(), fps);
    if (fps < 0 || (!packageName.compare("invalid"))) {
        ERROR("invalid param fps %d, pkgName %s", fps, packageName.c_str());
    }
    return SurfaceFlingerProxyManager::getInstance().setSoftVsync(packageName, fps);
}

int DecisionDriver::setTouchSensibility(int level) {
    DEBUG("setTouchSensibility: %d", level);

    std::string buf = std::to_string(level);
    DEBUG("Write game_switch_enable value: %s", buf.c_str());

    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, TOUCH_SENSITIVE_LEVEL_NODE, buf);
}

int DecisionDriver::setTouchSmoothly(int level) {
    DEBUG("setTouchSmoothly: %d", level);

    std::string buf = std::to_string(level);
    DEBUG("Write smooth_level value: %s", buf.c_str());

    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, TOUCH_SMOOTH_LEVEL_NODE, buf);
}

int DecisionDriver::setTouchProtection(int status) {
    int ret = false;
    DEBUG("setTouchProtection: %d", status);

    std::string buf = " ";
    if(0 == status) {
        buf = "mod para grip_disable_level:0";
        DEBUG("Write kernel_grip_handle value: %s", buf.c_str());
        ret = setCommand(KERNEL_GRIP_HANDLE_NODE, buf);
        buf = "mod para grip_disable_level:1";
        DEBUG("Write kernel_grip_handle value: %s", buf.c_str());
        ret = setCommand(KERNEL_GRIP_HANDLE_NODE, buf);
    } else {
        buf = "mod para grip_enable_level:0";
        DEBUG("Write kernel_grip_handle value: %s", buf.c_str());
        ret = setCommand(KERNEL_GRIP_HANDLE_NODE, buf);
        buf = "mod para grip_enable_level:1";
        DEBUG("Write kernel_grip_handle value: %s", buf.c_str());
        ret = setCommand(KERNEL_GRIP_HANDLE_NODE, buf);
    }

    return ret;
}

int DecisionDriver::setCommand(int32_t nodeFlag, const std::string& str) {
    return OplusTouchService::getInstance().writeTouchAdjusterNode(0, nodeFlag, str);
}


int DecisionDriver::oifaceDecisionSetTargetLoads(Json::Value v) {
    int cluster = stoi(v.get("cluster", "-1").asString());
    DEBUG("oifaceDecisionSetTargetLoads = %d", cluster);

    if (cluster < 0 || cluster > 4) {
        ERROR("cluster = %d is out of limit", cluster);
        return 0;
    }

    vector<int> sPolicy;
    string cPolicy = v.get("policy_num", "-1").asString();
    if (cPolicy != "-1") {
        for (int i = 0;i < cPolicy.size();i++) {
            if (cPolicy[i] == ']') {
                break;
            }
            if (cPolicy[i] >= '0'&&cPolicy[i] <= '4') {
                sPolicy.push_back(cPolicy[i]-'0');
                DEBUG("cPolicy[%d] = %d", i, cPolicy[i]-'0');
            }
        }
    }

    string delay = v.get("delayMs", "-1").asString();
    int delayTime = 0;
    if (delay == "-1") {
        delayTime = 10;
        DEBUG("delayTime == -1 and then delay 10ms");
    } else {
        delayTime = stoi(delay);
        DEBUG("delayTime == %d", delayTime);
    }


    int reset = stoi(v.get("reset", "-1").asString());
    if (reset == 1) {
        std::lock_guard<std::mutex> lockGuard(lock1);
        ERROR("AffinityService: it will into resetAllPolicy");
        if (AffinityService::getInstance().resetAllPolicy(cluster, ans, delayTime) < 0) {
            ERROR("AffinityService: reset all policy data error");
            return 0;
        }
        return 1;
    }

    std::vector<std::vector<std::string> > allPolicyPath;
    if (AffinityService::getInstance().getCpuAllPolicyPath(allPolicyPath) < 0) {
        ERROR("DecisionDriver: get all cpuset policy path failed");
        return 0;
    }
    ans.resize(3);
    for (int i = 0;i < cluster;i++) {
        ans[i].resize(cluster);
    }
    {
        std::lock_guard<std::mutex> lockGuard(lock1);
        if (AffinityService::getInstance().getAllPolicy(cluster) < 0) {
            ERROR("DecisionDriver: get all policy data error");
            return 0;
        }
    }
    for (auto pType : sPolicy) {
        if (pType <= 0) {
            ERROR("policy_num can not smaller than 0");
            return 0;
        } else if (pType > 0 && pType <= 4 && pType <= cluster) {
            std::lock_guard<std::mutex> lockGuard(lock1);
            string target_loads_cur = v.get("target_loads_"+to_string(pType), "-1").asString();
            string up_time_cur = v.get("up_limit_"+to_string(pType), "-1").asString();
            string down_time_cur = v.get("down_limit_"+to_string(pType), "-1").asString();
            DEBUG("oifaceDecisionSetTargetLoads: target_loads_cur = %s", target_loads_cur.c_str());
            DEBUG("oifaceDecisionSetTargetLoads: up_time_cur = %s", up_time_cur.c_str());
            DEBUG("oifaceDecisionSetTargetLoads: down_time_cur = %s", down_time_cur.c_str());
            DEBUG("oifaceDecisionSetTargetLoads: allPolicyPath[0][pType-1] = %s", allPolicyPath[0][pType-1].c_str());
            DEBUG("oifaceDecisionSetTargetLoads: allPolicyPath[1][pType-1] = %s", allPolicyPath[1][pType-1].c_str());
            DEBUG("oifaceDecisionSetTargetLoads: allPolicyPath[2][pType-1] = %s", allPolicyPath[2][pType-1].c_str());

            if (target_loads_cur == "-1") {
                ERROR("oifaceDecisionSetTargetLoads:target_loads_cur == -1");
            } else if (target_loads_cur != "-1") {
                ans[0][pType-1] = true;
                DEBUG("oifaceDecisionSetTargetLoads:target_loads_cur has value");
            }
            if (up_time_cur == "-1") {
                ERROR("oifaceDecisionSetTargetLoads:up_time_cur == -1");
            } else if (up_time_cur != "-1") {
                ans[1][pType-1] = true;
                DEBUG("oifaceDecisionSetTargetLoads:up_time_cur has value");
            }
            if (down_time_cur == "-1") {
                ERROR("oifaceDecisionSetTargetLoads:down_time_cur == -1");
            } else if (down_time_cur != "-1") {
                ans[2][pType-1] = true;
                DEBUG("oifaceDecisionSetTargetLoads:down_time_cur has value");
            }

            DEBUG("oifaceDecisionSetTargetLoads: start write target_loads----------------------");
            if (target_loads_cur != "-1") {
                DEBUG("!target_loads_cur.empty() and start a thread");
                std::thread(AffinityService::getInstance().writeTime, delayTime, allPolicyPath[0][pType-1], target_loads_cur).detach();
            }
            DEBUG("oifaceDecisionSetTargetLoads: start write up_time_cur---------------");
            if (up_time_cur != "-1") {
                DEBUG("!up_time_cur.empty() and start a thread");
                std::thread(AffinityService::getInstance().writeTime, delayTime, allPolicyPath[1][pType-1], up_time_cur).detach();
            }
            DEBUG("oifaceDecisionSetTargetLoads: start write down_time_cur-------------------");
            if (down_time_cur != "-1") {
                DEBUG("!down_time_cur.empty() and start a thread");
                std::thread(AffinityService::getInstance().writeTime, delayTime, allPolicyPath[2][pType-1], down_time_cur).detach();
            }
        } else {
            ERROR("policy_num out of limit");
            return 0;
        }
    }
    DEBUG("oifaceDecisionSetTargetLoads:write TargetLoads success");
    return 1;
}

int DecisionDriver::updateDecisionState(const std::string& clientName, bool isRestore) {
    sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
    if (client == NULL) {
        ERROR("client %s not found, skipped", clientName.c_str());
        return -1;
    }

    /* DEBUG */
    client->dumpDecisionState();

    Json::Value state = client->getDecisionState("wifi");
    if (state.isUInt()) {
        DEBUG("wifi state found");
        network_action network;
        network.timeout = 0;
        if (isRestore)
            network.power_mode = state.asUInt();
        else
            network.power_mode = 0;
        talkToDriver(network);
    }

    state = client->getDecisionState("scene");
    if (state.isString()) {
        DEBUG("scene state found");
        hypnus_scene scene;
        if (isRestore) { /* framework will handle scene change if leaving a tracked app */
            scene.name = state.asCString();
            /* FIXME: should set to correct pid */
            scene.pid = 1;
            talkToDriver(scene);
        }
    }

    state = client->getDecisionState("touch");
    if (state.isInt()) {
        DEBUG("touch state found");
        tp_action action;
        action.area_type = TP_AREA_ALL;
        memset(action.area, 0, sizeof(action.area));
        if (isRestore)
            action.sensitivity = state.asInt();
        else
            action.sensitivity = TP_NORMAL_SENSITIVITY;

        talkToDriver(action);
    }

    state = client->getDecisionState("touch_report");
    if (state.isInt()) {
        DEBUG("touch report state found");
        tpr_action action;
        if (isRestore)
            action.type = state.asInt();
        else
            action.type = TP_REPORT_NORMAL;

        talkToDriver(action);
    }

    state = client->getDecisionState("up_migrate");
    if (state.isUInt()) {
        sched_action action;
        action.up_migrate = state.asInt();
        state = client->getDecisionState("down_migrate");
        if(state.isUInt()) {
            action.down_migrate = state.asInt();
            if(isRestore) {
                DEBUG("Decision up_migrate:%d down_migrate:%d", action.up_migrate, action.down_migrate);
                talkToDriver(action);
            }
        }
    }

    state = client->getDecisionState("internal_scene");
    if (state.isInt()) {
        DEBUG("internal scene state found");
        scene_action action;
        if (isRestore) {
            action.scene = state.asInt();
        } else {
            action.scene = -1;
        }
    }

    state = client->getDecisionState("hypnus_control");
    if (state.isArray()) {
        DEBUG("hypnus control found");
        hypnus_control_data data;
        int32_t *ptr = (int32_t *)&data;
        if (isRestore) {
            for (int i = 0; i < (int)(sizeof(data)/sizeof(int32_t)); i++) {
                ptr[i] = state[i].asInt();
            }
        } else {
            data.control_mask = UINT_MAX;
        }

        talkToDriver(data);
    }

    state = client->getDecisionState("orms_control");
    if (state.isArray()) {
        DEBUG("orms control found");
        orms_control_info info;
        int32_t *ptr = (int32_t *)&info;
        if (isRestore) {
            for (int i = 0; i < (int)(sizeof(info)/sizeof(int32_t)); i++) {
                ptr[i] = state[i].asInt();
            }
        } else {
            info.control_mask = UINT_MAX;
        }

        talkToDriver(info);
    }

    state = client->getDecisionState("schedtune");
    if (state.isArray()) {
        DEBUG("schedtune state found");
        /* FIXME: support multi-boostgroup */
        struct schedtune_boost_action action;
        int *ptr = (int *)&action;
        for (int i = 0; i < (int)(sizeof(action)/sizeof(int)); i++) {
            ptr[i] = state[i].asInt();
        }
        if (!isRestore) {
            action.boost = 0;
        }
        talkToDriver(action);
    }

    state = client->getDecisionState("lpm");
    if (state.isInt()) {
        DEBUG("lpm action state found. isRestore:%d", isRestore);
        struct lpm_action action;

        if (isRestore)
            action.lpm_disabled = state.asInt();
        else
            action.lpm_disabled = 0;

        talkToDriver(action);
    }

    return 0;
}
