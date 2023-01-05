#include <utils/RefBase.h>
//#include <binder/Map.h>
//#include <binder/Value.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <HoraeServiceProxy.h>

#include "OIface.h"
#include "OifaceCallbackManager.h"
#include "OIfaceServer.h"
#include "BinderMessage.h"

using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE( HoraeServiceProxy );

#define THERMAL_STATUS_FOR_NOTIFY_LIGHT_RESTRICT 7
#define THERMAL_STATUS_FOR_NOTIFY_HEAVY_RESTRICT 9

IMPLEMENT_META_INTERFACE(HoraeThermalStatusListener, "com.oplus.thermalcontrol.IThermalStatusListener");
android::status_t HoraeThermalStatusListener::onTransact(uint32_t code, const android::Parcel& data,
                                    android::Parcel* reply, uint32_t flags) {
    if (!data.checkInterface(this))
        return android::PERMISSION_DENIED;
    if (code == NOTIFY_THERMAL_STATUS_CODE) {
        int status = data.readInt32();
        // notify cosa
        OifaceCallbackManager::getInstance().reportThermalStatus(status);
        // notify client
        int code = 0;
        if(status >= THERMAL_STATUS_FOR_NOTIFY_HEAVY_RESTRICT) {
            code = 2;
        }else{
            if(status >= THERMAL_STATUS_FOR_NOTIFY_LIGHT_RESTRICT) {
                code = 1;
            }
        }
        if(code != HoraeServiceProxy::getInstance().previousThermalStatusCode){
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_NOTIFY_THERMAL_STATUS;
            msg.data.thermalStatusCode = code;
            DEBUG("Notify thermal status code: %d", code);
            OIfaceServer::getInstance().sendMessage(msg);
            HoraeServiceProxy::getInstance().previousThermalStatusCode = code;
        }else{
            DEBUG("Thermal status code no change");
        }
        DEBUG("Horae Thermal status: [%d]", status);
    } else {
        return BBinder::onTransact(code, data, reply, flags);
    }
    return android::NO_ERROR;
}

HoraeServiceProxy::HoraeServiceProxy() {
    DEBUG("New HoraeServiceProxy \n");
    Mutex::Autolock _l(mHSLock);
    mListener = new HoraeThermalStatusListener();
    getHoraeService();
}

int HoraeServiceProxy::getHoraeService() {
    if (mHSBinder == NULL) {
        mHSBinder = defaultServiceManager()->checkService(String16("horae"));

        if (mHSBinder == NULL) {
            ERROR("unable to get HoraeService");
            return -1;
        }

        // Create the death listener.
        class DeathObserver : public IBinder::DeathRecipient {
            HoraeServiceProxy& mHoraeServiceProxy;
            virtual void binderDied(const wp<IBinder>& who) {
                ERROR("HoraeServiceProxy remote (HoraeServiceProxy) died [%p]",
                    who.unsafe_get());
                mHoraeServiceProxy.handleHSBinderDied();
            }
        public:
            explicit DeathObserver(HoraeServiceProxy& mgr) : mHoraeServiceProxy(mgr) { }
        };
        mDeathObserver = new DeathObserver(*const_cast<HoraeServiceProxy*>(this));
        mHSBinder->linkToDeath(mDeathObserver);
    }

    return mHSBinder == NULL ? -1 : 0;
}

void HoraeServiceProxy::handleHSBinderDied() {
    Mutex::Autolock _l(mHSLock);
    mHSBinder = NULL;
    mDeathObserver = NULL;
}

void HoraeServiceProxy::notifyGameTargetFps(const std::string pkgName, int targetFps) {
    Parcel data, reply;
    {
        Mutex::Autolock _l(mHSLock);
        if (getHoraeService() < 0) {
            return;
        }
        data.writeInterfaceToken(String16(HORAESERVICE_DESCRIPTOR));
        data.writeString8(String8(pkgName.c_str()));
        data.writeInt32(targetFps);
        status_t result = mHSBinder->transact(HoraeServiceProxy::HORAE_GAME_TARGET_FPS, data, &reply);
        if (result != NO_ERROR) {
            ERROR("%s, failed to transact: %d", __func__, result);
        } else {
            ERROR("%s, notify horae %s targetFps %d success", __func__, pkgName.c_str(), targetFps);
        }
    }
}

void HoraeServiceProxy::registerThermalListener() {
    Parcel data, reply;
    {
        Mutex::Autolock _l(mHSLock);
        if (getHoraeService() < 0) {
            return;
        }
        data.writeInterfaceToken(String16(HORAESERVICE_DESCRIPTOR));
        data.writeStrongBinder(android::IInterface::asBinder(mListener));
        status_t result = mHSBinder->transact(HoraeServiceProxy::HORAE_THERMAL_STATUS, data, &reply);
        if (result != NO_ERROR) {
            ERROR("%s, failed to transact: %d", __func__, result);
        } else {
            ERROR("register horae listener success");
        }
    }
}

float HoraeServiceProxy::getCurrentThermal() {
    Parcel data, reply;
    {
        Mutex::Autolock _l(mHSLock);
        if (getHoraeService() < 0) {
            ERROR("%s, failed to get horae service", __func__);
            return -1.0;
        }
        data.writeInterfaceToken(String16(HORAESERVICE_DESCRIPTOR));
        status_t result = mHSBinder->transact(HoraeServiceProxy::HORAE_CURRENT_THERMAL, data, &reply);
        if (result != NO_ERROR) {
            ERROR("%s, failed to transact: %d", __func__, result);
            return -1.0;
        } else {
            float thermal = reply.readFloat();
            ERROR("%s, get current thermal from horae success : %f", __func__, thermal);
            return thermal;
        }
    }
}
