#ifndef SCREEN_MODE_SERVICE_MANAGER_H
#define SCREEN_MODE_SERVICE_MANAGER_H

#include <string>
#include <binder/IBinder.h>
#include <utils/Log.h>
#include <utils/Mutex.h>
#include <utils/Singleton.h>

#define SCREENMODE_DESCRIPTOR     "com.oplus.screenmode.IOplusScreenMode"

using namespace android;

class ScreenModeProxy: public Singleton<ScreenModeProxy> {
    public:
        enum {
            GAME_SET_SCREEN_MODE = android::IBinder::FIRST_CALL_TRANSACTION + 10,
        };

        void setScreenMode(const std::string& pkgName, int32_t mode);

    private:
        sp<android::IBinder> mSmpBinder;
        sp<IBinder::DeathRecipient> mDeathObserver;
        android::Mutex mSmpLock;

        int getSmpService();
        void handleSmpBinderDied();
        friend class Singleton<ScreenModeProxy>;
        ScreenModeProxy();
        ~ScreenModeProxy(){};
};

#endif /*SCREEN_MODE_SERVICE_MANAGER_H*/
