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
 * huangzhigen     2019/09/30   1.0         Add Oiface feature
 * ------------------------------------------------------------------------------
 */

#ifndef __FRAME_RESCUER__
#define __FRAME_RESCUER__

#include <vector>
#include <string>
#include <list>
#include <utils/Mutex.h>
#include <utils/Singleton.h>
#include <utils/RefBase.h>

#include "OIface.h"
#include "util/OrmsProxy.h"
#include "FrameStateManager.h"

using namespace std;
using namespace android;
class FRFrameStateMgrObserver;

class FrameRescuer : public android::Singleton<FrameRescuer> {
private:
    friend class android::Singleton<FrameRescuer>;
    friend class FRFrameStateMgrObserver;
    FrameRescuer();
    ~FrameRescuer();

    struct frame_info {
        int64_t time;
        int64_t delay;
    };

    struct timer_params {
        int uid;
        int pid;
        FrameRescuer *fr;
        int tti;
    };

    static const int FRAME_RESCUER_TIMER_SLACK_NS = 1000000;
    static const int FRAME_RESCUER_HYPNUS_ACTION_TIMEOUT_MS = 200;
    static const int MAX_SAMPLE_FRAMES = 60;
    static const int ONE_SEC_NS = 1000000000;
    static const int64_t ONE_MSEC_NS = 1000000;
    static const int FRAME_INTERVAL_16_MS = 16666666;
    static const int FRAME_INTERVAL_33_MS = 33333333;
    static const int FRAME_INTERVAL_50_MS = 50000000;
    static const int FRAME_RESCUER_REDUNDANT_TIME = 2000000;
    static constexpr float FRAME_ALPHA = 1.50f;
    static constexpr float FRAME_ALPHA_MIN = 0.50f;
    int mTargetFps;
    int mUid;
    int mPid;
    int mFrameBoostType;
    float mFrameAlp;
    timer_t mTimerId;
    volatile bool mBHypnusActing;
    android::Mutex mLock;
    std::list<struct frame_info> mFrameTimeList;
    struct timer_params mTimerParam;
    std::string mClientName;
    Decision mDecision, mDecisionStop;
    volatile bool mBoostReady, mBoosting;
    //statistics
    std::atomic_int mDirectlyBoost;
    std::atomic_int mTotalBoost;
    std::atomic_int64_t mBoostingTime;
    std::atomic_int64_t mBoostTotalTime;
    std::atomic_int mLags;
    bool mFrameRescueStarted;

    static void timerCallBack(union sigval val);

    void addFrameInterval(int64_t timestamp);
    int caclTimerTimeInterval(); /* ns */
    int calcFrameInterval(int64_t mean);
    int setupTimer(int bcNum);
    void cancelTimer();
    int redundantFrameBoostTime(int elapsed, int mdelay, int cdelay);

    int setBoostAction(const Decision& decision);
    void boost();
    void stopBoost();
    void cleanup();

    // statistics for boost time
    void doStatistics(int64_t now);

    // boost with hypnus action
    void actionBoost();
    void stopBoostAction();

    // default boost
    void stopDefaultFrameBoost();
    void defaultFrameBoost();

    //boost with schedtune
    void schedtuneBoost();
    void stopSchedtuneBoost();

    void deleteTimer();
    struct orms_action mBoostAction;
    bool mTraceTargetFPS = false;
    android::sp<FRFrameStateMgrObserver> mFrameStateMgrObserver;
public:
    enum {
        FR_DEQUEUE_START = 1,
        FR_DEQUEUE_END,
        FR_QUEUE_START,
        FR_QUEUE_END,
        FR_SF_CONSUME_BUFFER_SZ,
    };

    enum {
        FR_FPS_ULTIMATE = 120,
        FR_FPS_EXTREME = 90,
        FR_FPS_HIGH = 60,
        FR_FPS_MEDIUM = 40,
        FR_FPS_LOW = 30,
        FR_FPS_LOWEST = 20,
    };

    static int setTimerSlack();

    void setTargetFps(int targetFps);
    int getTargetFps();
    void setFrameAlp(float frameAlp);
    float getFrameAlp();
    int produceFrame(int bcNumber, int64_t timestamp);
    int getDiretlyBoostCount();
    int getBoostCount();
    int64_t getBoostTime();
    int getLags();
    void stopAndClean();
    void setFrameBoostType(int type) {mFrameBoostType = type;}
    void setTraceTargetFPS(bool trace) { mTraceTargetFPS = trace; }
    int getFrameBoostType() {return mFrameBoostType;}
    int updateBoostDecision(const std::string& clientName, const Decision& decision);
    std::string getClientName() { return mClientName; }
    int getFrameBoostStatus() { return mFrameRescueStarted; }

    int triggerFrameBoost(std::string package, const struct FrameBoost& fb);
    void triggerFrameBoostStart(std::string package, const Decision& decision);
    void triggerFrameBoostStart(std::string package);
    void triggerFrameBoostStop();
    void triggerFrameInfoStop();

    bool recordStatu = false;
    void recordFI(int now);
    int64_t lastRecordTime = 0;
};

class FRFrameStateMgrObserver: public IFrameStateManagerObserver {
public:
    FRFrameStateMgrObserver(FrameStateManager &frameStateManager) : frameStateManager_(frameStateManager) {
    }

    virtual ~FRFrameStateMgrObserver() {
    }

    int onFrameProduce(int bcNumber, int64_t timestamp) override {
        return FrameRescuer::getInstance().produceFrame(bcNumber, timestamp);
    }

    void removeFrameStateListener() {
        frameStateManager_.removeFrameStateListener("frameBooster", this);
    }

    void addFrameStateListener() {
        frameStateManager_.addFrameStateListener("frameBooster", this);
    }

private:
    std::string message_from_subject_;
    FrameStateManager &frameStateManager_;
};
#endif
