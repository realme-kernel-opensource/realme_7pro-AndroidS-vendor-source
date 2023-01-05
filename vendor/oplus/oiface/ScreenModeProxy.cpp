#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <utils/Singleton.h>
#include "ScreenModeProxy.h"
#include "OIface.h"


using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE( ScreenModeProxy );

ScreenModeProxy::ScreenModeProxy()
{
    DEBUG("New ScreenModeProxy \n");
    Mutex::Autolock _l(mSmpLock);
    getSmpService();
}

int ScreenModeProxy::getSmpService() {
    if (mSmpBinder == NULL) {
        mSmpBinder = defaultServiceManager()->checkService(String16("oplusscreenmode"));

        if (mSmpBinder == NULL) {
            ERROR("unable to get oplusscreenmode service");
            return -1;
        }

         // Create the death listener.
        class DeathObserver : public IBinder::DeathRecipient {
            ScreenModeProxy& mScreenModeProxy;
            virtual void binderDied(const wp<IBinder>& who) {
                ALOGW("ScreenModeProxy remote (oplusscreenmode) died [%p]",
                    who.unsafe_get());
                mScreenModeProxy.handleSmpBinderDied();
            }
        public:
            explicit DeathObserver(ScreenModeProxy& mgr) : mScreenModeProxy(mgr) { }
        };
        mDeathObserver = new DeathObserver(*const_cast<ScreenModeProxy*>(this));
        mSmpBinder->linkToDeath(mDeathObserver);
    }

        return mSmpBinder == NULL ? -1 : 0;
}

void ScreenModeProxy::handleSmpBinderDied() {
    Mutex::Autolock _l(mSmpLock);
    mSmpBinder = NULL;
    mDeathObserver = NULL;
}

void ScreenModeProxy::setScreenMode(const std::string& pkgName, int32_t mode) {
    Parcel data, reply;
    {
        android::Mutex::Autolock _l(mSmpLock);

        if (getSmpService() < 0) {
            return;
        }

        data.writeInterfaceToken(String16(SCREENMODE_DESCRIPTOR));
        data.writeString16(String16(pkgName.c_str()));
        data.writeInt32(mode);
        status_t result = mSmpBinder->transact(GAME_SET_SCREEN_MODE, data, &reply);
        if (result != NO_ERROR) {
            int32_t err = reply.readExceptionCode();
            if (err) {
                ERROR("Server exepction code: %d\n", err);
                return;
            }
        }
    }
    DEBUG("ScreenModeProxy set screen mode: %d", mode);
}
