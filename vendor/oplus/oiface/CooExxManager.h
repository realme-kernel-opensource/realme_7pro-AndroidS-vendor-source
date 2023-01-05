#ifndef __COOEXX_MANAGER_H__
#define __COOEXX_MANAGER_H__
#include <utils/Singleton.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>

using namespace std;

class CooExxManager: public android::Singleton<CooExxManager> {
    public:
        struct CoolExCallback {
            android::sp<android::IBinder> callback;
        };

        CooExxManager() {};
        virtual ~CooExxManager() {};
        int registerCoolExCallback(android::sp<android::IBinder> binder);
        void setCoolExState(int ces);
        void setCoolExConfig(std::string cec);
        void setCoolExFilterType(int cee);
        int getCoolExState();
        std::string getCoolExConfig();
        int getCoolExFilterType();
        void reportCoolExFilterType(int type, std::string config);

    private:
        int coolExState = 0;
        std::string coolExConfig;
        int coolExFilterType = 0;
        std::map<android::wp<android::IBinder>, CoolExCallback> mCallbackList;
        mutable android::Mutex mLock;

        std::string connectedString();
};

#endif

