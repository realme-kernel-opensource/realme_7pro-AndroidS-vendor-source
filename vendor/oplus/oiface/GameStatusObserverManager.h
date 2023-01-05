#ifndef __GAME_STATUS_OBSERVER_MANAGER_H__
#define __GAME_STATUS_OBSERVER_MANAGER_H__
#include <utils/Singleton.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
// #include <binder/Map.h>

class GameStatusObserverManager: public android::Singleton<GameStatusObserverManager> {
    public:
        GameStatusObserverManager() {};
        virtual ~GameStatusObserverManager() {};
        int registerObserverListener(int32_t observerType,const std::string& observerconfig, android::sp<android::IBinder> binder);
        int unRegisterObserverListener(android::sp<android::IBinder> binder);
        // void reportGameStatus(const std::string& pkgName,  android::binder::Map gameStatusInfo);
        void reportGameStatus(const std::string& pkgName,  std::string &gameStatusInfo);
        enum {
            OBSERVER_GAME_NONE = 0,
            OBSERVER_GAME_SCENE,
            OBSERVER_GAME_STATUS,
        };

        struct ObserverCallback {
            int32_t observerType;
            std::string observerconfig;
            android::sp<android::IBinder> callback;
        };

    private:
        /* Use vector is enough?? */
        std::map<android::wp<android::IBinder>, ObserverCallback> mCallbackList;
        mutable android::Mutex mLock;
};

#endif
