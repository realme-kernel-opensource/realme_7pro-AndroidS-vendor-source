#include "OifaceCallbackManager.h"
#include "OplusOIfaceService.h"
#include "OIface.h"
#include "DataCollector.h"

using namespace android;
using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE(OifaceCallbackManager);

int OifaceCallbackManager::registerOifaceCallback(sp<IBinder> binder) {
    IBinder *t = binder.get();
    DEBUG("registering oiface callback, binder: %p", t);
    if (t == NULL) {
        ERROR("No oiface callback");
        return -1;
    }

    android::Mutex::Autolock _l(mLock);
    OifaceCallback oifaceCallback;
    oifaceCallback.callback = binder;
    mCallbackList[binder] = oifaceCallback;

    android::sp<IOIfaceCallback> cb = interface_cast<IOIfaceCallback>(oifaceCallback.callback);

    DEBUG("Register a new client %s", connectedString().c_str());
    cb->onSystemNotify(connectedString());

    return 0;
}

std::string OifaceCallbackManager::connectedString() {
    Json::Value root;
    Json::FastWriter  writer;
    root["oiface"] = "connected";
    std::string data = writer.write(root);
    return data;
}

void OifaceCallbackManager::reportGameStatus(const std::string& pkgName, const std::string &gameStatusInfo) {
    DEBUG("trying to call reportGameStatus :%s gameStatusInfo %s", pkgName.c_str(), gameStatusInfo.c_str());
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onGameStatusChanged(pkgName, gameStatusInfo) < 0) {
            ERROR("notify game status error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}

void OifaceCallbackManager::reportTGPAInfo(const int uid, const int pid, const std::string &gameStatusInfo) {
    DEBUG("trying to call reportTGPAInfo, uid: %d, pid: %d, status info: %s", uid, pid, gameStatusInfo.c_str());
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onTGPAInfo(gameStatusInfo, uid, pid) < 0) {
            ERROR("notify game status error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}

void OifaceCallbackManager::reportFB(const int status) {
    DEBUG("trying to call reportFB, status: %d", status);
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        //use gameJitter to avoid binder timeout, no need to wait reply
        //structure need to be modified in future version
        std::string game = " ";
        if (mCallback->onGameJitter(game, status) < 0) {
            ERROR("notify frame boost error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}

void OifaceCallbackManager::reportGPANotification(const std::string &notification) {
    DEBUG("trying to call reportGPANotification: %s", notification.c_str());

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onGPANotification(notification).size() <= 0) {
            ERROR("notify GPA error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}

int OifaceCallbackManager::getColorXStatusFromCOSA(const std::string &packageName){
    DEBUG("Try to get ColorX Status From COSA by calling reportGPANotification: %s", packageName.c_str());
    Json::Value value;
    std::string colorx_switch = "colorx_switch";
    value[colorx_switch.c_str()] = packageName;
    Json::FastWriter writer;
    std::string data = writer.write(value);

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }
        Json::Reader reader;
        std::string reply = String8(mCallback->onGPANotification(data)).string();
        DEBUG("colorX reply: %s", reply.c_str());
        if(reply.size() <= 0){
            DEBUG("reply size <= 0");
            ++iter;
            continue;
        }else{
            Json::Value root;
            reader.parse(reply, root);
            std::string status = root.get("colorx_switch", "-1").asString();
            DEBUG("Get ColorX Status: [%s]", status.c_str());
            return std::atoi(status.c_str());
        }
    }
    return -1;
}

android::String16 OifaceCallbackManager::getCoolExDataFromCOSA(const std::string &packageName){
    DEBUG("Try to get CoolEx Data From COSA by calling reportGPANotification: %s", packageName.c_str());
    Json::Value value;
    std::string coolex_data = "coolex_data";
    value[coolex_data.c_str()] = packageName;
    Json::FastWriter writer;
    std::string data = writer.write(value);

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }
        android::String16 reply = mCallback->onGPANotification(data);
        if(reply.size() <= 0){
            DEBUG("reply size <= 0");
            ++iter;
            continue;
        }else{
            DEBUG("Get coolEx data size: %d\n", reply.size());
            return reply;
        }
    }
    std::string defaultData = "default data";
    return android::String16(defaultData.c_str());
}

void OifaceCallbackManager::reportThermalStatus(const int status) {
    DEBUG("trying to call reportThermalStatus, status: %d", status);
    android::Mutex::Autolock _l(mLock);
    std::string status_str = std::to_string(status);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback
            = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onThermalStatusChanged(status_str) < 0) {
            ERROR("notify frame boost error!");
            mCallbackList.erase(iter++);
            continue;
        }
        ++iter;
    }
}

void OifaceCallbackManager::reportHeavyTaskInfo(const std::string& heavyTaskInfo) {
    DEBUG("trying to call reportHeavyTaskInfo,  taskinfo %s", heavyTaskInfo.c_str());
    onSystemNotifyData(heavyTaskInfo);
}

void OifaceCallbackManager::reportStartFrameProduce(const std::string& fastStartState) {
    DEBUG("trying to call reportStartFrameProduce, status: %s ,mCallbackList %d", fastStartState.c_str(),
        mCallbackList.size());
    onSystemNotifyData(fastStartState);
}

#if 0
void OifaceCallbackManager::reportNetworkLatency(const std::string& pkgName, int32_t latency,
        int32_t lastReportTime, int32_t lastLatency) {
    int32_t now = int32_t(systemTime() / 1000000LL) ;
    DEBUG("trying to call network notifier:%s %d", pkgName.c_str(), latency);
    DataCollector::getInstance().addData(DataCollector::DC_TYPE_INTERNAL, pkgName,
            "network_latency", latency);
    DataCollector::getInstance().incData(DataCollector::DC_TYPE_INTERNAL, pkgName,
            "network_jank_times");

    /* ever reported */
    if (lastReportTime != 0) {
        int32_t duration = (systemTime() / 1000000) - lastReportTime;
        for (int i = 0; i < (int)ARRAY_SIZE(kInterval); i++) {
            if (lastLatency < kInterval[i]) {
                string keyName = "nl_" + to_string(kInterval[i]);
                DataCollector::getInstance().addData(DataCollector::DC_TYPE_INTERNAL, pkgName,
                        keyName, duration);
                break;
            }
        }
    }

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        if ((latency > iter->second.thresholdMs) && ((iter->second.lastReportMs < 0) ||
                    ((iter->second.lastReportMs > 0) &&
                    (now - iter->second.lastReportMs >= iter->second.minReportMs)))) {
            /* call binder callback */
            DEBUG("calling network notifier...");
            android::sp<IOIfaceNetworkNotifier> mNotifier =
                interface_cast<IOIfaceNetworkNotifier>(iter->second.callback);
            if (mNotifier == NULL) {
                ERROR("unable to cast remote binder");
                mCallbackList.erase(iter++);
                continue;
            }

            if (mNotifier->onNetworkNotify(pkgName, latency) < 0) {
                mCallbackList.erase(iter++);
                continue;
            }

            iter->second.lastReportMs = now;
        }

        ++iter;
    }
}
#endif

void OifaceCallbackManager::reportDataForTempPredict(const std::string& infoForTempPredict) {
    onSystemNotifyData(infoForTempPredict);
}

int OifaceCallbackManager::get4DSupportStatusFromCOSA(const std::string &packageName){
    DEBUG("Try to get 4D Support Status From COSA by calling reportGPANotification: %s", packageName.c_str());
    Json::Value value;
    std::string support4d = "4dsupport";
    value[support4d.c_str()] = packageName;
    Json::FastWriter writer;
    std::string data = writer.write(value);

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }
        Json::Reader reader;
        std::string reply = String8(mCallback->onGPANotification(data)).string();
        DEBUG("4D support status reply: %s", reply.c_str());
        if(reply.size() <= 0) {
            DEBUG("reply size <= 0");
            ++iter;
            continue;
        } else {
            Json::Value root;
            reader.parse(reply, root);
            std::string status = root.get("4dsupport", "0").asString();
            DEBUG("Get 4dsupport Status: %s", status.c_str());
            return std::atoi(status.c_str());
        }
    }
    return 1;
}

void OifaceCallbackManager::reportWeaponName(const std::string &weaponName){
    DEBUG("Try to repport weapon name to COSA by calling reportGPANotification: %s", weaponName.c_str());
    Json::Value value;
    std::string signal4d = "4dsignal";
    value[signal4d.c_str()] = weaponName;
    Json::FastWriter writer;
    std::string data = writer.write(value);

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }
        std::string reply = String8(mCallback->onGPANotification(data)).string();
        if(reply.compare("ok") == 0){
            DEBUG("Repport weapon name successed");
            return;
        }
        ++iter;
    }
    DEBUG("Repport weapon name failed");
}


int OifaceCallbackManager::getDynamicResSwitchFromCOSA(const std::string &packageName) {
    DEBUG("Try to get Dynamic Res Switch From COSA by calling reportGPANotification: %s", packageName.c_str());
    Json::Value value;
    std::string dynamicResSwitch = "dynamic_res_switch";
    value[dynamicResSwitch.c_str()] = packageName;
    Json::FastWriter writer;
    std::string data = writer.write(value);

    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }
        std::string reply = String8(mCallback->onGPANotification(data)).string();
        if(reply.size() <= 0){
            DEBUG("reply size <= 0");
            ++iter;
            continue;
        }else{
            DEBUG("Get Dynamic Res Switch size: %s\n", reply.c_str());
            return std::atoi(reply.c_str());
        }
        ++iter;
    }
    DEBUG("should not reach here and return default data");
    return 0;
}

void OifaceCallbackManager::reportEvent2DataCollecter(const int type, const std::string &data) {
    DEBUG("Try to get report event 2 data collect calling onSystemNotify,type %d, data %s", type, data.c_str());
    Json::Value value;
    if (type == BIG_DATA_TYPE_DYNAMIC_RESOLUTION) {
        value["dynamic_resolution_event"] = data;
    } else {
        ERROR("unknow type: %d", type);
        return;
    }
    Json::FastWriter writer;
    std::string sendData = writer.write(value);
    onSystemNotifyData(sendData);
}

void OifaceCallbackManager::onSystemNotifyData(const std::string &data) {
    android::Mutex::Autolock _l(mLock);
    for (auto iter = mCallbackList.begin(); iter != mCallbackList.end();) {
        android::sp<IOIfaceCallback> mCallback = interface_cast<IOIfaceCallback>(iter->second.callback);
        if (mCallback == NULL) {
            ERROR("unable to get remote binder");
            mCallbackList.erase(iter++);
            continue;
        }

        if (mCallback->onSystemNotify(data) < 0) {
            ERROR("failed onSystemNotify %s!", data.c_str());
            mCallbackList.erase(iter++);
            continue;
        } else {
            DEBUG("onSystemNotify %s", data.c_str());
        }
        ++iter;
    }
}