 /************************************************************
  * Copyright (C) 2020 Oplus. All rights reserved.
  *
  * Description     :display service manager
  * History         :( ID, Date, Author, Description)
  * v1.0, 2019-06-21, wangmingyuan, create
  ************************************************************/

#ifndef SURFACEFLINGER_SERVICE_MANAGER_H
#define SURFACEFLINGER_SERVICE_MANAGER_H

#include <vector>
#include <string>
#include <binder/IBinder.h>
#include <utils/Log.h>
#include <json/value.h>
#include <utils/Mutex.h>
#include <utils/Singleton.h>
#include "util/FrameStatsTracer.h"

#define SURFACEFLINGER_DESCRIPTOR     "android.ui.ISurfaceComposer"
#define GET_LAYER_FRAMES     0x6f696663
#define GET_LAYER_NAME       0x6f696665
#define SET_SOFT_VSYNC      21002
using namespace android;

class SurfaceFlingerProxyManager: public Singleton<SurfaceFlingerProxyManager> {
    public:
        float getFps(const std::string pkgName);
        float getFps();
        float getFramesCommon();
        std::string getLayerName(const std::string& pkgName,const std::vector<std::string>& item);
        int32_t getTotalFpsByLayerName(const std::string& layerName);
        int32_t getTotalFlipsFromSf();
        int32_t getTotalFlipsFromSf(std::string packageName);
        int32_t setGCPEffectMode(int32_t mode);
        void stopSurfaceFlingerFrameNotification();
        int setSoftVsync(const std::string &pkgName, int fps);
        std::string startFrameStat(const std::string &pkgName);
        std::string getAndStopFrameStat(const std::string &pkgName, struct FrameStats &frameStatsIn);

        bool startSurfaceFlingerPackageNotification(const std::string &pkgName);
        bool startSurfaceFlingerFrameNotification(const char *layerName);
    private:
        sp<android::IBinder> mSFBinder;
        std::string mLayerName;
        std::string mLastPkgName;
        android::Mutex mSFLock;
        sp<IBinder::DeathRecipient> mDeathObserver;
        //for get current's fps
        uint32_t mLastTotalFps = 0;
        int64_t mLastTime = 0;
        float mLastFps = 0.0f;

        uint32_t mLastTotalFlips = 0;
        int64_t mLastGetTime = 0 ;

        int getSfService();
        void handleSFBinderDied();
        friend class Singleton<SurfaceFlingerProxyManager>;
         SurfaceFlingerProxyManager();
         ~SurfaceFlingerProxyManager(){};
};

#endif /*SURFACEFLINGER_SERVICE_MANAGER_H*/
