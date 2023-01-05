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


#include "OIface.h"
#include "FrameRescuer.h"
#include "FrameStateManager.h"
#include "SurfaceFlingerProxyManager.h"
#include "OifaceCallbackManager.h"

ANDROID_SINGLETON_STATIC_INSTANCE(FrameStateManager);
using namespace std;
using namespace android;

FrameStateManager::FrameStateManager() {
    mFBEnableCount = 0;
    mPackageName = "none";
    mLayerName = "none";
    mFrameStateManagerObserverMap.clear();
    clearFrameStateListener();
}

void FrameStateManager::notifyFrameProduce(int bcNumber, int64_t timestamp) {
    //DEBUG("notifyFrameProduce to %d features", mFrameStateManagerObserverMap.size());
    android::Mutex::Autolock _l(mLock);

    for (map<std::string, android::sp<IFrameStateManagerObserver>>::iterator iter = mFrameStateManagerObserverMap.begin();
    iter != mFrameStateManagerObserverMap.end(); ) {
        if (iter->second == NULL) {
            iter++;
            continue;
            ERROR("notifyFrameProduce: failed");
        }

        if (iter->second->onFrameProduce(bcNumber, timestamp) == 2) {
            DEBUG("remove : IFrameStateManagerObserver");
            mFrameStateManagerObserverMap.erase(iter++);
            continue;
        }
        iter++;
    }
}

void FrameStateManager::addFrameStateListener(const std::string & name, android::sp<IFrameStateManagerObserver> observer) {
    DEBUG("addFrameStateListener: %d", mFrameStateManagerObserverList.size());
    android::Mutex::Autolock _l(mLock);

    auto iter = mFrameStateManagerObserverMap.find(name);
    if (iter == mFrameStateManagerObserverMap.end()) {
        mFrameStateManagerObserverMap[name] = observer;
    }
}

void FrameStateManager::removeFrameStateListener(const std::string & name, android::sp<IFrameStateManagerObserver> observer) {
    android::Mutex::Autolock _l(mLock);

    auto iter = mFrameStateManagerObserverMap.find(name);
    if (iter != mFrameStateManagerObserverMap.end()) {
        mFrameStateManagerObserverMap.erase(name);
    }

    if (mFrameStateManagerObserverMap.size() == 0) {
        mPackageName = "none";
        mLayerName = "none";
    }
}

void FrameStateManager::clearFrameStateListener() {
    mFrameStateManagerObserverMap.clear();
}

bool FrameStateManager::startSurfaceFlingerFrameNotification(const std::string &pkgName, const std::string &layerName) {
    android::Mutex::Autolock _l(mLock);

    DEBUG("startSurfaceFlingerFrameNotification: layerName %s! mLayerName:%s, mFBEnableCount= %d",
    layerName.c_str(), mLayerName.c_str(), mFBEnableCount.load());

    if (layerName.empty() || layerName == "none") {
        return false;
    }

    if (layerName == mLayerName) {
        mFBEnableCount++;
        return true;
    }

    if (mLayerName != "none") {
        mFBEnableCount = 0;
        clearFrameStateListener();
    }

    if (SurfaceFlingerProxyManager::getInstance().startSurfaceFlingerFrameNotification(layerName.c_str())) {
        mFBEnableCount++;
        mLayerName = layerName;
        mPackageName = pkgName;
        return true;
    }
    return false;
}

bool FrameStateManager::stopSurfaceFlingerFrameNotification(const std::string &name) {
    android::Mutex::Autolock _l(mLock);
    DEBUG("stopSurfaceFlingerFrameNotification: pkgName %s! mFBEnableCount= %d", mPackageName.c_str(), mFBEnableCount.load());
    mFBEnableCount--;
    auto iter = mFrameStateManagerObserverMap.find(name);
    if (iter != mFrameStateManagerObserverMap.end()) {
        ERROR("stopSurfaceFlingerFrameNotification: delete listener delayed!");
    }

    if ((mFrameStateManagerObserverMap.size() == 0) || ((mFrameStateManagerObserverMap.size() == 1) && (iter != mFrameStateManagerObserverMap.end()))) {
        SurfaceFlingerProxyManager::getInstance().stopSurfaceFlingerFrameNotification();
        mPackageName = "none";
        mLayerName = "none";
        mFBEnableCount = 0;
        return true;
    }

    DEBUG("stopSurfaceFlingerFrameNotification:  %lu!", mFrameStateManagerObserverMap.size());
    return false;
}
