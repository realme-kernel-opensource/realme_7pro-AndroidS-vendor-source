#include "GameStatusObserverManager.h"
#include "OplusOIfaceService.h"
#include "OIface.h"
#include "DataCollector.h"

using namespace android;
using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE(GameStatusObserverManager);

int GameStatusObserverManager::registerObserverListener(int32_t observerType,const std::string& observerconfig,
        sp<IBinder> binder) {
    DEBUG("registering game status observer listener:%d", observerType);

    android::Mutex::Autolock _l(mLock);
    ObserverCallback callback;
    callback.observerType = observerType;
    callback.observerconfig = observerconfig;
    callback.callback = binder;
    mCallbackList[binder] = callback;

    return 0;
}

int GameStatusObserverManager::unRegisterObserverListener(android::sp<android::IBinder> binder){
    DEBUG("trying to unRegisterObserverListener ");
    android::Mutex::Autolock _l(mLock);
    std::map<android::wp<android::IBinder>, ObserverCallback>::iterator iter = mCallbackList.find(binder);
    if(iter != mCallbackList.end()){
        mCallbackList.erase(iter);
        return 0;
    }
    return -1;
}

void GameStatusObserverManager::reportGameStatus(const std::string& pkgName, string &gameStatusInfo) {
// void GameStatusObserverManager::reportGameStatus(const std::string& pkgName,  binder::Map gameStatusInfo) {
    DEBUG("trying to call reportGameStatus :%s", pkgName.c_str());
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        switch(iter->second.observerType){
            case OBSERVER_GAME_STATUS: {
                    // FIXME map: change map to json; DONE
#if 0
                    auto viter = gameStatusInfo.find("app_status");
                    if (viter == gameStatusInfo.end()){
                        DEBUG("calling game status notifier..,this is not app_status");
                        break;
                    }
#else
                    Json::Value root;
                    Json::Reader reader;
                    bool success = reader.parse(gameStatusInfo, root);
                    if (!success || root.isNull()) {
                        ERROR("parse json failed(%s)", gameStatusInfo.c_str());
                        break;
                    }
                    if (root.isMember("app_status")) {
                        DEBUG("calling game status notifier..,this is not app_status");
                        break;
                    }
#endif
                }
            case OBSERVER_GAME_SCENE:
                DEBUG("calling game status notifier...");
                android::sp<IGameStatusNotifier> mNotifier =
                    interface_cast<IGameStatusNotifier>(iter->second.callback);
                if (mNotifier == NULL) {
                    ERROR("unable to cast remote binder");
                    mCallbackList.erase(iter++);
                    continue;
                }

                if (mNotifier->onGameStatusNotify(pkgName, gameStatusInfo) < 0) {
                    mCallbackList.erase(iter++);
                    continue;
                }
            break;
        }
        ++iter;
    }
}
