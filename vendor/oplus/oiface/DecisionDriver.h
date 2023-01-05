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

#ifndef __DECISION_DRIVER__
#define __DECISION_DRIVER__

#include <stdio.h>
#include <utils/Mutex.h>

#include <utils/RefBase.h>
#include <utils/Unicode.h>
#include <binder/IServiceManager.h>
#include <json/value.h>

#include "OIface.h"
#include "ActivityServiceProxy.h"
#include "util/OrmsProxy.h"
#include "GlobalConfig.h"

#include <thread>
#include <chrono>
#include <utility>
#include <mutex>
#include <atomic>

using namespace std;
using namespace android;

#define TIME_ONE_SEC    1000000000LL
#define TALK_TO_DRIVER_INTERVAL (1 * TIME_ONE_SEC)
#define SCHEDTUNE_DEFERED_SKIP_SETTING      -99
#define SCHEDTUNE_BOOST_SKIP_SETTING        -99
#define HASH_M 249997
#define TASK_MONITOR_TIMEOUT 10000000000LL

class DecisionDriver {
private:
    static DecisionDriver* driver;
    android::sp<IActivityServiceProxy> mActivity;
    mutable android::Mutex mOrmsLock;
    uint32_t mEnableMask;
    int64_t  mLastUpdateTime;
    int mFd;
    mutable android::Mutex mLock;
    Decision mLastDecision;
    FILE* mFpBoost;
    FILE* mFpDefered;
    std::string mFgCpus;
    std::string mRestrictedCpus;
    std::string mTopCpus;
    std::string mSysBgCpus;

    /* packages request performance wifi */
    DecisionDriver(const DecisionDriver&);
    DecisionDriver& operator = (const DecisionDriver&);
    ~DecisionDriver() {
        if (mFd > 0) close(mFd);
        if (mFpBoost) fclose(mFpBoost);
        if (mFpDefered) fclose(mFpDefered);
    };
    DecisionDriver();
    int talkToDriver(const struct hypnus_action& action);
    int talkToDriver(const struct orms_action& action);
    int talkToDriver(const struct network_action& action);
    int talkToDriver(const struct hypnus_scene& scene);
    int talkToDriver(const struct affinity_action& action);
    int talkToDriver(const struct top_app_ctl_action &action);
    int talkToDriver(const struct io_action& action);
    int talkToDriver(const struct modem_log_action& action);
    // int talkToDriver(const struct tp_action& action);
    int talkToDriver(const struct tpr_action& action);
    int talkToDriver(const struct sched_action& action);
    int talkToDriver(const struct hypnus_control_data& action);
    int talkToDriver(const struct orms_control_info& control_info);
    int talkToDriver(const struct schedtune_boost_action& action);
    int talkToDriver(const struct lpm_action& action);
    int talkToDriver(const struct frame_boost_action& action, const string& clientName);
    int talkToDriver(const struct input_boost_action& action, const string& clientName);
    int talkToDriver(const struct key_thread_notification& key_thread, const string& clientName);
    int setOrmsAction(const struct orms_action& action);
    int setOrmsMigrate(int up, int down);
    int setOrmsCtrlData(const struct orms_control_info &ormsData);
    int connectActivityService();
    int setSchedtuneBoost(int index, int boost);
    int setSchedtuneDefered(int index, int defered);
    static void setTask(int delay_ms, int cluster_type, int bind_type, std::string task);
    static void setTaskByCpuFlag(int delay_ms, int cluster_flag, int bind_type, std::string task);
    static void setTaskByFgLittle(int delay_ms, int cluster_flag, std::string fgCpus, std::string task);
    static void bindTGPATask(int delay_ms, int cluster_type, int tid);
    static void delayedSetTaskMonitor(int gamePid, int timeDuration, int timeThreshold, int fpsThreshold);
    static void setFpsgoTask(int delay_ms, int task_code, std::string task);

    int oifaceDecisionGameSwitchEnable(Json::Value v);
    int oifaceDecisionSchedtune(Json::Value v);
    int oifaceDecisionGameStatus(Json::Value v);
    int oifaceDecisionFrameBoost(Json::Value v);
    int oifaceDecisionScreenRefreshRate(Json::Value v);
    int oifaceDecisionBindCore(Json::Value v);
    int oifaceDecisionSetTaskMonitor(Json::Value v);
    int oifaceDecisionSetAppFps(Json::Value v);
    void oifaceDecisionSetKeyThread(Json::Value v);
    void oifaceDecisionSetProcCpuset(Json::Value v);
    void oifaceDecisionSetAuthStatus(Json::Value v);
    void oifaceDecisionSetFpsgo(Json::Value v);
public:

    static DecisionDriver& getInstance() {
        DecisionDriver* instance = driver;

        if (instance == NULL) {
            instance = new DecisionDriver();
            driver = instance;
        }

        return *instance;
    }
    bool isSupportOrms() {return true;};  //need to add feature toggle
    void directSetTask(int bind_tid, int cluster_type);
    int talkToDriver(const Decision& decision, const string& clientName, bool needToThrottle = true);
    int updateDecisionState(const std::string& clientName, bool isRestore);
    void handleHypnusdDied();
    int talkToDriver(const struct tp_action& action);

    int setControlData(const Decision& decision);
    void setKeyTask(int gamePid, int taskTid, int bindValue);

    // new interface for cosa decision interface
    void oifaceDecision(const string &decisionJson);
    void oifaceDecisionArray(Json::Value &decisions);
    void oifaceDecisionSingle(Json::Value &d);
    // constexpr int hashCompileTime(const char *str);

    int setTouchSensibility(int level);
    int setTouchSmoothly(int level);
    int setTouchProtection(int status);

    int setCommand(int32_t nodeFlag, const std::string& str);

    std::mutex lock1;
    std::vector<std::vector<bool>> ans;
    int oifaceDecisionSetTargetLoads(Json::Value v);
};

#endif
