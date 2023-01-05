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
 * shanghaoqiang             2021/07/05      1.0     Add FrameStateManager
 * ------------------------------------------------------------------------------
 */

#ifndef __FRAME_STATE_MANAGER_H__
#define __FRAME_STATE_MANAGER_H__

#include <string>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <list>
#include <utils/RefBase.h>
#include <map>

class IFrameStateManagerObserver;

class FrameStateManager : public android::Singleton<FrameStateManager> {
public:
    void notifyFrameProduce(int bcNumber, int64_t timestamp);
    void addFrameStateListener(const std::string & name, android::sp<IFrameStateManagerObserver> observer);
    void removeFrameStateListener(const std::string & name, android::sp<IFrameStateManagerObserver> observer);
    bool startSurfaceFlingerFrameNotification(const std::string &pkgName, const std::string &layerName);
    bool stopSurfaceFlingerFrameNotification(const std::string & name);

private:
    friend class android::Singleton<FrameStateManager>;

    friend class IFrameStateManagerObserver;
    FrameStateManager();
    ~FrameStateManager() {}

    void clearFrameStateListener();
    android::Mutex mLock;
    std::string mPackageName;
    std::string mLayerName;
    std::atomic_int mFBEnableCount;
    std::atomic_int mTotalBoost;
    std::atomic_int64_t mBoostingTime;
    std::atomic_int64_t mBoostTotalTime;
    std::atomic_int mLags;

    std::map<std::string, android::sp<IFrameStateManagerObserver>> mFrameStateManagerObserverMap;
    std::list< android::sp<IFrameStateManagerObserver>> mFrameStateManagerObserverList;
};

class IFrameStateManagerObserver:public virtual android::RefBase {
public:
    virtual ~IFrameStateManagerObserver(){};
    virtual int onFrameProduce(int bcNumber, int64_t timestamp) = 0;
};

#endif
