 /************************************************************
  * Copyright (C) 2020 Oplus. All rights reserved.
  *
  * Description     :display service manager
  * History         :( ID, Date, Author, Description)
  * v1.0, 2019-06-18,  wangmingyuan, create
  ************************************************************/

#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <utils/Singleton.h>
#include "ThreadState.h"
#include "GlobalConfig.h"
#include "SurfaceFlingerProxyManager.h"

using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE( SurfaceFlingerProxyManager );

#define RETURN_ON_ERROR(X) \
    do { \
        status_t res = (X); \
        if (res != NO_ERROR) { \
            return resultJsonString(0); \
        } \
    } while (false)

#define THREE_HUNDRED_MS 300000000

SurfaceFlingerProxyManager::SurfaceFlingerProxyManager():mLayerName(""),mLastPkgName(""){
    DEBUG("New SurfaceFlingerProxyManager\n");
    Mutex::Autolock _l(mSFLock);
    getSfService();
}

// SurfaceFlingerProxyManager::~SurfaceFlingerProxyManager(){}

int SurfaceFlingerProxyManager::getSfService() {
    if (mSFBinder == NULL) {
        mSFBinder = defaultServiceManager()->checkService(String16("SurfaceFlinger"));

        if (mSFBinder == NULL) {
            ERROR("unable to get surfaceflinger");
            return -1;
        }

        // Create the death listener.
        class DeathObserver : public IBinder::DeathRecipient {
            SurfaceFlingerProxyManager& mSurfaceFlingerProxy;
            virtual void binderDied(const wp<IBinder>& who) {
                ALOGW("SurfaceFlingerProxy remote (surfaceflinger) died [%p]",
                    who.unsafe_get());
                mSurfaceFlingerProxy.handleSFBinderDied();
            }
        public:
            explicit DeathObserver(SurfaceFlingerProxyManager& mgr) : mSurfaceFlingerProxy(mgr) { }
        };
        mDeathObserver = new DeathObserver(*const_cast<SurfaceFlingerProxyManager*>(this));
        mSFBinder->linkToDeath(mDeathObserver);
    }

    return mSFBinder == NULL ? -1 : 0;
}

void SurfaceFlingerProxyManager::handleSFBinderDied() {
    Mutex::Autolock _l(mSFLock);
    mSFBinder = NULL;
    mDeathObserver = NULL;
}

std::string SurfaceFlingerProxyManager::getLayerName(const std::string& pkgName,const vector<string>& item) {
    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return string("");
        }
        data.writeInterfaceToken(String16(SURFACEFLINGER_DESCRIPTOR));
        data.writeString16(String16(pkgName.c_str()));
        status_t result = mSFBinder->transact(GET_LAYER_NAME, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
    }
    std::string layerName = String8(reply.readString16()).string();
    //DEBUG("package Name is: %s, Layer Name is: %s", pkgName.c_str(), layerName.c_str());
    return layerName;
}

int32_t SurfaceFlingerProxyManager::getTotalFpsByLayerName(const std::string& layerName) {
    if (layerName.empty()) return 0;
    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return 0;
        }
        data.writeInterfaceToken(String16(SURFACEFLINGER_DESCRIPTOR));
        data.writeString16(String16(mLayerName.c_str()));
        status_t result = mSFBinder->transact(GET_LAYER_FRAMES, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
    }

    return reply.readInt32();
}

int32_t SurfaceFlingerProxyManager::getTotalFlipsFromSf() {
    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return 0;
        }
        data.writeInterfaceToken(String16(SURFACEFLINGER_DESCRIPTOR));
        status_t result = mSFBinder->transact(1013, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
    }

    return reply.readInt32();
}

int32_t SurfaceFlingerProxyManager::getTotalFlipsFromSf(std::string packageName) {
    if(!mLayerName.empty() && packageName == mLastPkgName) {
        mLayerName = getLayerName(packageName,{"sdk",packageName});
        int32_t nowTotalFps = getTotalFpsByLayerName(mLayerName);
        DEBUG("%s[%s] current frames is: %d ", packageName.c_str(), mLayerName.c_str(), nowTotalFps);
        return nowTotalFps;
    } else {
        mLayerName = getLayerName(packageName,{"sdk",packageName});
        DEBUG("mLastPkgName[%s] pkgName[%s], layerName[%s]", mLastPkgName.c_str(), packageName.c_str(), mLayerName.c_str());
        mLastPkgName = packageName;
        return 0;
    }
}

float SurfaceFlingerProxyManager::getFps(const std::string pkgName) {
    if(!mLayerName.empty() && pkgName == mLastPkgName) {
        int64_t now = systemTime(SYSTEM_TIME_MONOTONIC);
        mLayerName = getLayerName(pkgName,{"sdk",pkgName});
        int32_t nowTotalFps = getTotalFpsByLayerName(mLayerName);
        int64_t timeDurationNs = now - mLastTime;
        float fps = (float)(nowTotalFps - mLastTotalFps) * NS_PER_SEC / timeDurationNs;
        if (timeDurationNs > THREE_HUNDRED_MS) {
            DEBUG("%s[%s] and update last, current fps is: %f ", pkgName.c_str(), mLayerName.c_str(), fps);
            mLastTime = now;
            mLastTotalFps = nowTotalFps;
            mLastFps = fps;
        } else {
            DEBUG("%s[%s] and timeDuration(%ldms) do not update last, fps is: %f ~use lastFps %f",
                pkgName.c_str(), mLayerName.c_str(), timeDurationNs / 1000000, fps, mLastFps);
            fps = mLastFps;
        }
        /* FIXME: Filter out the wrong fps calculated by the wrong sample */
        if(fps < 0.1 || fps > 125){
            DEBUG("Abnormal! %s[%s] current fps is: %f abnormal, return 0", pkgName.c_str(), mLayerName.c_str(), fps);
            return 0;
        }
        return fps;
    } else {
        mLayerName = getLayerName(pkgName,{"sdk",pkgName});
        DEBUG("mLastPkgName[%s] pkgName[%s], layerName[%s]", mLastPkgName.c_str(), pkgName.c_str(), mLayerName.c_str());
        mLastTotalFps = getTotalFpsByLayerName(mLayerName);
        mLastTime = systemTime(SYSTEM_TIME_MONOTONIC);
        mLastPkgName = pkgName;
        mLastFps = 0.0f;
        return 0;
    }
}

int32_t SurfaceFlingerProxyManager::setGCPEffectMode(int32_t mode)
{
    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);
        if (getSfService() < 0) {
            return 0;
        }
        data.writeInterfaceToken(String16(SURFACEFLINGER_DESCRIPTOR));
        data.writeInt32(mode);
        status_t result = mSFBinder->transact(1023, data, &reply);
        if (result != NO_ERROR) {
            ERROR("%s, failed to transact: %d", __func__, result);
        }
    }
    return 0;
}

float SurfaceFlingerProxyManager::getFps() {
    std::string currentPkgName = ThreadState::getInstance().getForgroundPackage();
    if((!mLayerName.empty()) && !ThreadState::getInstance().isActivityChanged()) {
        int64_t now = systemTime(SYSTEM_TIME_MONOTONIC);
        int32_t nowTotalFps = getTotalFpsByLayerName(mLayerName);
        int64_t timeDurationNs = now - mLastTime;
        float fps = (float)(nowTotalFps - mLastTotalFps) * NS_PER_SEC / timeDurationNs;
        if (timeDurationNs > THREE_HUNDRED_MS) {
            DEBUG("[%s] and update last, current fps is: %f ", mLayerName.c_str(), fps);
            mLastTime = now;
            mLastTotalFps = nowTotalFps;
            mLastFps = fps;
        } else {
            DEBUG("[%s] and timeDuration(%ldms) do not update last, fps is: %f use lastFps %f", mLayerName.c_str(), timeDurationNs/1000000, fps, mLastFps);
            fps = mLastFps;
        }
        /* FIXME: Filter out the wrong fps calculated by the wrong sample */
        if (fps < 0.1 || fps > 125) {
            DEBUG("Abnormal! [%s] current fps is: %f abnormal, return 0", mLayerName.c_str(), fps);
            return 0;
        }
        return fps;
    } else {
        //it means foreground app has switched ,the current layerName is changed
        mLayerName = getLayerName(currentPkgName,{"sdk",currentPkgName});
        mLastTotalFps = getTotalFpsByLayerName(mLayerName);
        mLastTime = systemTime(SYSTEM_TIME_MONOTONIC);
        mLastFps = 0.0f;
        ThreadState::getInstance().resetActivityChanged();

        DEBUG("current layerName is: %s", mLayerName.c_str());
        return 0;
    }
}

float SurfaceFlingerProxyManager::getFramesCommon() {
    int64_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    int32_t nowTotalFlips = getTotalFlipsFromSf();
    float fps = (float)(nowTotalFlips - mLastTotalFlips) * NS_PER_SEC / (now - mLastGetTime);
    mLastGetTime = now;
    mLastTotalFlips = nowTotalFlips;
    DEBUG("%s, current fps is: %f", __func__, fps);
    /* FIXME: Filter out the wrong fps calculated by the wrong sample */
    if(fps < 0 || fps > 125){
        return 0;
    }
    return fps;
}

bool SurfaceFlingerProxyManager::startSurfaceFlingerFrameNotification(const char *layerName) {
    android::Mutex::Autolock _l(mSFLock);
    if (getSfService() < 0) {
        return false;
    }
    Parcel data, reply;
    data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
    data.writeString8(String8(layerName));
    DEBUG("Start SF notification for layer %s", layerName);

    status_t result = mSFBinder->transact(SET_FRAME_BOOST_LAYER, data, &reply);
    if (result == NO_ERROR) {
        return true;
    }

    ERROR("Fail to start SF notification: %d", result);
    return false;
}

void SurfaceFlingerProxyManager::stopSurfaceFlingerFrameNotification() {
    android::Mutex::Autolock _l(mSFLock);

    if (getSfService() < 0) {
        return;
    }
    DEBUG("Stop SF notification");
    Parcel data, reply;
    data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
    data.writeString8(String8());
    status_t result = mSFBinder->transact(SET_FRAME_BOOST_LAYER, data, &reply);
    if (result != NO_ERROR)
        ERROR("%s, failed to transact: %d", __func__, result);
}

std::string SurfaceFlingerProxyManager::startFrameStat(const std::string &pkgName) {
    if (pkgName.empty()) {
        ERROR("invalid package name");
        return resultJsonString(0);
    }

    vector<string> empty;
    std::string layerName = getLayerName(pkgName, empty);

    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return resultJsonString(0);
        }
        data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        data.writeString16(String16(layerName.c_str()));
        RETURN_ON_ERROR(data.writeBool(true));
        RETURN_ON_ERROR(data.writeBool(true));

        status_t ret = mSFBinder->transact(0x6f696664, data, &reply);
        if (ret != NO_ERROR) {
            return resultJsonString(0);
        }
    }
    return resultJsonString(1);
}

std::string SurfaceFlingerProxyManager::getAndStopFrameStat(const std::string &pkgName, struct FrameStats &frameStatsOut) {
    if (pkgName.size() == 0) {
        ERROR("invalid package name");
        return resultJsonString(0);
    }

    vector<string> empty;
    std::string layerName = getLayerName(pkgName, empty);

    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return resultJsonString(0);
        }
        data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        data.writeString16(String16(layerName.c_str()));
        RETURN_ON_ERROR(data.writeBool(true));
        RETURN_ON_ERROR(data.writeBool(false));
        status_t ret = mSFBinder->transact(0x6f696664, data, &reply);
        if (ret != NO_ERROR) {
            ERROR("error on transaction: %d\n", reply.readExceptionCode());
            return resultJsonString(0);
        }
    }

    int fsize = 0;
    status_t ret = reply.readInt32(&fsize);
    if (ret != 0) {
        return resultJsonString(0);
    }

    if (fsize < (int32_t)sizeof(frameStatsOut)) {
        ERROR("version not matched");
        return resultJsonString(0);
    }
    frameStatsOut.frameCount = reply.readInt32();
    frameStatsOut.displayPeriod = reply.readInt64();
    frameStatsOut.startTime = reply.readInt64();
    frameStatsOut.stopTime = reply.readInt64();
    frameStatsOut.meanInterval = reply.readFloat();
    frameStatsOut.stdev = reply.readFloat();
    if (NUM_INTERVAL_STATS != reply.readInt32()) {
        ERROR("interval stats length not matched");
        return resultJsonString(0);
    }
    for (int i = 0; i < NUM_INTERVAL_STATS; i++) {
        frameStatsOut.intervalStat[i] = reply.readInt32();
    }
    frameStatsOut.statTime = reply.readInt64();

    //stop frame statistics
    Parcel transData, transReply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            return resultJsonString(0);
        }
        transData.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        transData.writeString16(String16(layerName.c_str()));
        RETURN_ON_ERROR(transData.writeBool(false));
        RETURN_ON_ERROR(transData.writeBool(false));
        status_t ret = mSFBinder->transact(0x6f696664, transData, &transReply);
        if (ret != NO_ERROR) {
            ERROR("error on transaction: %d\n", transReply.readExceptionCode());
            return resultJsonString(0);
        }
    }

    return resultJsonString(1);
}

int SurfaceFlingerProxyManager::setSoftVsync(const std::string &pkgName, int fps) {
    int useLess_oifaceFlags = fps > 0 ? 1 : 0;
    Parcel data, transReply;
    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSfService() < 0) {
            ERROR("setSoftVsync getSfService failed");
            return -1;
        }
        data.writeInterfaceToken(String16(SURFACEFLINGER_DESCRIPTOR));
        data.writeInt32(fps);
        data.writeInt32(useLess_oifaceFlags);
        data.writeString16(String16(pkgName.c_str()));
        status_t ret = mSFBinder->transact(SET_SOFT_VSYNC, data, &transReply);
        if (ret != NO_ERROR) {
            ERROR("setSoftVsync error on transaction: %d\n", transReply.readExceptionCode());
            return -1;
        }
    }
    return 0;
}

bool SurfaceFlingerProxyManager::startSurfaceFlingerPackageNotification(const std::string &pkgName) {
    std::string layer;
    int getLayerTry = 3;
    vector<string> empty;
    DEBUG("startSurfaceFlingerPackageNotification: %s", pkgName.c_str());

    for (int i = 0; i < getLayerTry; i++) {
        layer = getLayerName(pkgName, empty);
        if (layer.empty()) {
            usleep(100000);
        } else {
            DEBUG("Get layer name: %s", layer.c_str());
            break;
        }
    }

    if (layer.empty()) {
        return false;
    }

    // notify sf
    if (startSurfaceFlingerFrameNotification(layer.c_str())) {
        INFO("trigger SF notification start[%s]", layer.c_str());
    }

    return true;
}
