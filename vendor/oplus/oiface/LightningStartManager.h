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
 * shanghaoqiang               2021/07/05   1.0      Add Lightning Start Manager
 * ------------------------------------------------------------------------------
 */

#ifndef __LIGHTNING_START_MANAGER__
#define __LIGHTNING_START_MANAGER__

#include <vector>
#include <string>
#include <list>
#include <utils/Mutex.h>
#include <utils/Singleton.h>
#include <utils/RefBase.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>

#include "OIface.h"
#include "util/OrmsProxy.h"
#include "FrameStateManager.h"
#define LIGHTNING_START_FRAME_THRESHOLD    1

using namespace std;
using namespace android;
class LSFrameStateMgrObserver;

class LightningStartManager : public android::Singleton<LightningStartManager> {
private:
    friend class android::Singleton<LightningStartManager>;
    friend class LSFrameStateMgrObserver;
    LightningStartManager();
    ~LightningStartManager();

    struct timer_params {
        int uid;
        int pid;
        LightningStartManager *lightningStMan;
        android::sp<LSFrameStateMgrObserver> frameStateMgrObserver;
        int tti;
    };

    enum {
        LIGHTNING_START_DEFAULT = 0,
        LS_FRAME_PRODUCED,
        LS_FRAME_PRODUCED_FAILED,
        LS_SWAP_OUT,
        LS_SWAP_OUT_FAILED
    };

    void deleteTimer();
    int setupGetLayerNameTimer();
    static void timerCallBack(union sigval val);
    static std::string curLightningStartStateStr(int curState, std::string packageName);

    int mUid;
    int mPid;
    int mLastBufNum;
    timer_t mTimerId;
    std::string mLightningStartPackage;
    std::atomic_int mGetLayerRetry = 0;
    std::atomic_int mFrameProduceNum = 0;
    int mSwapVersion;
    bool mFrameRescueStarted;
    struct timer_params mTimerParam;
    android::sp< LSFrameStateMgrObserver > mFrameStateMgrObserver;
    Mutex mLightenStrManMutex;
public:
    void cancelGetLayerNameTimer();
    int getLayerRetryCount();
    void decGetLayerRetry();
    std::string getLightningStartPackage();
    void setLightningStartPackage(std::string packageName);
    int produceFrame(int bcNumber, int64_t timestamp);
    int setNandSwapIn(int pid, std::string packageName);
    int setNandSwapOut(int pid, std::string packageName);
};

class LSFrameStateMgrObserver: public IFrameStateManagerObserver {
public:
    LSFrameStateMgrObserver(FrameStateManager &frameStateManager) : frameStateManager_(frameStateManager) {
    }
    virtual ~LSFrameStateMgrObserver() {
    }

    int onFrameProduce(int bcNumber, int64_t timestamp) override {
        return LightningStartManager::getInstance().produceFrame(bcNumber, timestamp);
    }

    void removeFrameStateListener() {
        frameStateManager_.removeFrameStateListener("lightningStart", this);
    }

    void addFrameStateListener() {
        frameStateManager_.addFrameStateListener("lightningStart", this);
    }

private:
    std::string message_from_subject_;
    FrameStateManager &frameStateManager_;
};

#endif
