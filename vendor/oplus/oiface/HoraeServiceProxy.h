#ifndef HORAE_SERVICE_PROXY_H
#define HORAE_SERVICE_PROXY_H

#include <utils/RefBase.h>
#include <utils/Mutex.h>
#include <utils/Log.h>
#include <utils/Singleton.h>
#include <utils/StrongPointer.h>
#include <string>
#include <binder/IBinder.h>
#include <binder/Status.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <cstdint>

#define HORAESERVICE_DESCRIPTOR  "com.oplus.horae.IHoraeService"

using namespace android;

class IHoraeThermalStatusListener: public ::android::IInterface {
    public:
        DECLARE_META_INTERFACE(HoraeThermalStatusListener)
        const int NOTIFY_THERMAL_STATUS_CODE = android::IBinder::FIRST_CALL_TRANSACTION + 2;
};

class HoraeThermalStatusListener: public android::BnInterface<IHoraeThermalStatusListener> {
    protected:
        virtual android::status_t onTransact(uint32_t code, const android::Parcel& data,
                                            android::Parcel* reply, uint32_t flags) override;
};

class BpHoraeThermalStatusListener: public android::BpInterface<IHoraeThermalStatusListener> {
    public:
        BpHoraeThermalStatusListener(const android::sp<IBinder> &impl): android::BpInterface<IHoraeThermalStatusListener>(impl) {}
};

class HoraeServiceProxy: public Singleton<HoraeServiceProxy> {
    public:
        void notifyGameTargetFps(const std::string pkgName, int targetFps);
        void registerThermalListener();
        float getCurrentThermal();
        int HORAE_THERMAL_STATUS = android::IBinder::FIRST_CALL_TRANSACTION;
        int HORAE_GAME_TARGET_FPS = android::IBinder::FIRST_CALL_TRANSACTION + 14;
        int HORAE_CURRENT_THERMAL = android::IBinder::FIRST_CALL_TRANSACTION + 16;
        int previousThermalStatusCode = 0;

    private:
        sp<android::IBinder> mHSBinder;
        sp<IBinder::DeathRecipient> mDeathObserver;
        android::sp<HoraeThermalStatusListener> mListener;
        android::Mutex mHSLock;
        int getHoraeService();
        void handleHSBinderDied();
        friend class Singleton<HoraeServiceProxy>;
        HoraeServiceProxy();
        ~HoraeServiceProxy(){};
};

#endif