#include "CooExxManager.h"
#include "OplusOIfaceService.h"
#include "OIface.h"
#include "DataCollector.h"

using namespace android;
using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE(CooExxManager);

int CooExxManager::registerCoolExCallback(sp<IBinder> binder) {
    IBinder *t = binder.get();
    DEBUG("Registering Cool_Ex Callback, binder: %p", t);
    if (t == NULL) {
        ERROR("No Cool_Ex Callback");
        return -1;
    }

    android::Mutex::Autolock _l(mLock);
    CoolExCallback coolExCallback;
    coolExCallback.callback = binder;
    mCallbackList[binder] = coolExCallback;

    DEBUG("Register a new client %s", connectedString().c_str());
    return 0;
}

void CooExxManager::setCoolExState(int ces){
    coolExState = ces;
}

void CooExxManager::setCoolExConfig(std::string cec){
    coolExConfig = cec;
}

void CooExxManager::setCoolExFilterType(int ceft){
    coolExFilterType = ceft;
}

int CooExxManager::getCoolExState(){
    return coolExState;
}

std::string CooExxManager::getCoolExConfig(){
    return coolExConfig;
}

int CooExxManager::getCoolExFilterType(){
    return coolExFilterType;
}


std::string CooExxManager::connectedString() {
    Json::Value root;
    Json::FastWriter  writer;
    root["Cool_Ex"] = "connected";
    std::string data = writer.write(root);
    return data;
}

void CooExxManager::reportCoolExFilterType(int type, std::string config) {
    DEBUG("trying to call reportCoolExFilterType");
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<ICoolExCallback> mCallback
            = interface_cast<ICoolExCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onReportCoolExFilterType(type, config) == false) {
            ERROR("report coolex filter type error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}


