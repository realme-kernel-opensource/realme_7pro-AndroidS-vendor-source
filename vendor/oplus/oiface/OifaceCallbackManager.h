#ifndef __OIFACE_CALLBCK_MANAGER_H__
#define __OIFACE_CALLBCK_MANAGER_H__
#include <utils/Singleton.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>

const int BIG_DATA_TYPE_DYNAMIC_RESOLUTION = 1;

class OifaceCallbackManager: public android::Singleton<OifaceCallbackManager> {
    public:
        struct OifaceCallback {
            android::sp<android::IBinder> callback;
        };

        OifaceCallbackManager() {};
        virtual ~OifaceCallbackManager() {};
        int registerOifaceCallback(android::sp<android::IBinder> binder);
        void reportGameStatus(const std::string& pkgName, const std::string &gameStatusInfo);
        void reportTGPAInfo(const int uid, const int pid, const std::string &gameStatusInfo);
        void reportFB(const int status);
        void reportThermalStatus(const int status);
        void reportGPANotification(const std::string &notification);
        int getColorXStatusFromCOSA(const std::string &packageName);
        android::String16 getCoolExDataFromCOSA(const std::string &packageName);
        int getDynamicResSwitchFromCOSA(const std::string &packageName);
        int get4DSupportStatusFromCOSA(const std::string &packageName);
        void reportWeaponName(const std::string &weaponName);
        void reportHeavyTaskInfo(const std::string& heavyTaskInfo);
        void reportDataForTempPredict(const std::string& infoForTempPredict);
        void reportStartFrameProduce(const std::string& fastStartState);
        void reportEvent2DataCollecter(const int type, const std::string &data);

    private:
        void onSystemNotifyData(const std::string &data);
        std::map<android::wp<android::IBinder>, OifaceCallback> mCallbackList;
        mutable android::Mutex mLock;

        std::string connectedString();
};

#endif
