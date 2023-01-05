#ifndef __OIFACE_SERVICE_H__
#define __OIFACE_SERVICE_H__

#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <cutils/compiler.h>
// #include <binder/Map.h>
// #include <binder/Value.h>
#include "HealthListener.h"

class IOIfaceService: public android::IInterface {
    public:
        static char const* getServiceName() ANDROID_API {
            return "oiface";
        }

        DECLARE_META_INTERFACE(OIfaceService);
        enum {
            /* application */
            REGISTER_CLIENT              = android::IBinder::FIRST_CALL_TRANSACTION + 100,
            UPDATE_GAME_INFO             = android::IBinder::FIRST_CALL_TRANSACTION + 101,
            REQUEST_RESOURCE             = android::IBinder::FIRST_CALL_TRANSACTION + 102,
            REGISTER_NOTIFIER            = android::IBinder::FIRST_CALL_TRANSACTION + 103,
            GET_VERSION                  = android::IBinder::FIRST_CALL_TRANSACTION + 104,
            GET_SYSTEM_TEMP              = android::IBinder::FIRST_CALL_TRANSACTION + 106,
            GET_SYSTEM_MODE              = android::IBinder::FIRST_CALL_TRANSACTION + 107,
            GET_MEMORY_USAGE             = android::IBinder::FIRST_CALL_TRANSACTION + 108,
            SET_GENERAL_SINGAL           = android::IBinder::FIRST_CALL_TRANSACTION + 109,
            REGISTER_ENGINE_CLIENT       = android::IBinder::FIRST_CALL_TRANSACTION + 153,
            UPDATE_ENGINE_INFO           = android::IBinder::FIRST_CALL_TRANSACTION + 154,
            REGISTER_CLIENT_FROM_COSA    = android::IBinder::FIRST_CALL_TRANSACTION + 160,
            SET_GENERAL_SINGAL_FROM_COSA = android::IBinder::FIRST_CALL_TRANSACTION + 161,
            DUMP_PKG_INFO                = android::IBinder::FIRST_CALL_TRANSACTION + 199,
            GAME_SDK_END                 = android::IBinder::FIRST_CALL_TRANSACTION + 200,
            /* MISC */
            GET_DEVICE_ID                = android::IBinder::FIRST_CALL_TRANSACTION + 500,
            GET_DEVICE_ID_NEW            = android::IBinder::FIRST_CALL_TRANSACTION + 503,

            NOTIFY_GET_DATA = android::IBinder::FIRST_CALL_TRANSACTION + 3001,
        };
};

class OIfaceService: public android::BnInterface<IOIfaceService> {
    public:
        OIfaceService() {ALOGE("OIfaceService contructor");}
        virtual ~OIfaceService() {ALOGE("OIfaceService destructor");}
    protected:
        virtual android::status_t onTransact(uint32_t code, const android::Parcel& data,
                android::Parcel *reply, uint32_t flags = 0) override;
};


class IOIfaceNotifier: public android::IInterface {
    public:
        DECLARE_META_INTERFACE(OIfaceNotifier);
        virtual void onSystemNotify(const android::String16& result) = 0;
        enum {
            SYSTEM_NOTIFY = android::IBinder::FIRST_CALL_TRANSACTION,
        };
};

class BpOIfaceNotifier: public android::BpInterface<IOIfaceNotifier> {
    public:
        BpOIfaceNotifier(const android::sp<android::IBinder>& impl):
            BpInterface<IOIfaceNotifier>(impl) {};
        virtual void onSystemNotify(const android::String16& result);
};

#endif
