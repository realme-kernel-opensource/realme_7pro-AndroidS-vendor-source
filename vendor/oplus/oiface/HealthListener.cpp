#include <unistd.h>
#include <binder/IServiceManager.h>
#include <healthhalutils/HealthHalUtils.h>
#include <android/hidl/manager/1.0/IServiceManager.h>
#include <hidl/HidlTransportSupport.h>
#include <pthread.h>
#include <algorithm>

#include "HealthListener.h"
#include "OIface.h"
#include "OIfaceServer.h"
#include "Utils.h"
#include "GlobalConfig.h"
#include "PlatformAdaptor.h"
#include "ThermalService.h"

using namespace android;
using namespace std;
using android::hardware::interfacesEqual;
using android::hardware::Return;
using android::hardware::health::V1_0::BatteryStatus;
using android::hardware::health::V1_0::toString;
using android::hardware::health::V2_0::get_health_service;
using android::hardware::health::V2_0::HealthInfo;
using android::hardware::health::V2_0::IHealth;
using android::hardware::health::V2_0::Result;
using android::hidl::manager::V1_0::IServiceManager;

ANDROID_SINGLETON_STATIC_INSTANCE(HealthListener);
android::Mutex HealthListener::sLock;
pthread_cond_t HealthListener::sCondition;
pthread_mutex_t HealthListener::sCondLock;
bool HealthListener::sShouldPoll = true;


const int HealthListener::ERROR_NODE_NOT_FOUND = -2;
const int HealthListener::ERROR_INVALID_VALUE = -3;

#define SLEEP_POLL_SEC      10
#define SLEEP_NOPOLL_SEC    120
/* Onetime initializer */
HealthListener::HealthListener() {
    /* get all interested thermal list */
    Json::Value v;
    GlobalConfig::getInstance().getConfig(&v, {"thermal_list"});
    if (!v.isObject()) {
        ERROR("thermal list should be an object");
        return;
    }

    /* scan thermal list first */
    vector<string> path;

    if (PlatformAdaptor::getInstance().isQcom()) {
        getFileList("/sys/class/thermal/", path);
    } else {
        getVendorFileList("/sys/class/thermal/", path);
    }

    for (auto& item:path) {
        if (item.find("thermal_zone") == string::npos)
            continue;

        string type;
        if (PlatformAdaptor::getInstance().isQcom())
            type = getSystemFile(item + "/type");
        else
            type = getVendorFile(item + "/type");

        if (type.size() == 0) {
            DEBUG("get file %s failed", (item + "/type").c_str());
            continue;
        }

        type.erase(std::find_if(type.rbegin(), type.rend(), [](int ch) {
                    return !std::isspace(ch);
                    }).base(), type.end());

        for (auto& interest: v.getMemberNames()) {
            if (!v[interest].isString())
                continue;

            DEBUG("type:%s interested:%s", type.c_str(), v[interest].asString().c_str());
            if (v[interest].asString().find(type) == string::npos)
                continue;

            INFO("HealthListener init, matched a thermal(%s), type(%s), with interest(%s)",
                    item.c_str(), type.c_str(), v[interest].asString().c_str());

            mThermalList[interest] = item + "/temp";
        }
    }

    for (auto &item: mThermalList) {
        DEBUG("type:%s path:%s", item.first.c_str(), item.second.c_str());
    }

    initHealthService();

    if (mHealth == NULL) {
        ERROR("init health service failed");
        return;
    }
    /* get intial health information */
    Result r = mHealth->update();
    if (r != Result::SUCCESS) {
        ERROR("health update failed(ret=%d).", r);
        mHealth = NULL;
    }

    pthread_t thread;

    pthread_cond_init(&sCondition, NULL);
    pthread_mutex_init(&sCondLock, NULL);

    if (pthread_create(&thread, NULL, monitor_thread, NULL)) {
        ERROR("create moinitor thread failed");
        return;
    }
}

int32_t HealthListener::getSkinTemperature() {
    if (mThermalList.find("skin") == mThermalList.end()) {
        return ERROR_NODE_NOT_FOUND;
    }

    string value = getVendorFile(mThermalList["skin"]);
    if (value.size() == 0) {
        ERROR("get skin thermal failed");
        return ERROR_INVALID_VALUE;
    }

    return std::stoi(value);
}

int32_t HealthListener::getCpuTemperature() {
    if (mThermalList.find("cpu") == mThermalList.end()) {
        return -1;
    }

    string value = getVendorFile(mThermalList["cpu"]);
    if (value.size() == 0) {
        ERROR("get cpu thermal failed");
        return -1;
    }

    return std::stoi(value);
}

void HealthListener::updateThermalStats() {
    /* dummy code here */
}

void * HealthListener::monitor_thread(void *) {
    pthread_setname_np(pthread_self(), "oiface_monitor_thread");
    int mLastTemp = -1;
    while (1) {
        sLock.lock();

        int temp = HealthListener::getInstance().getSkinTemperature();
        if ((temp != -1) && (temp != mLastTemp)) {
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_SKIN_TEMP;
            msg.value = temp;
            OIfaceServer::getInstance().sendMessage(msg);
            mLastTemp = temp;
        }

        /* updateThermalStats */
        HealthListener::getInstance().updateThermalStats();

        /* handle healthd died case */
        HealthListener::getInstance().kickHealthdIfNeeded();

        sLock.unlock();

        pthread_mutex_lock(&sCondLock);

        struct timespec ts;

        if (clock_gettime(CLOCK_REALTIME, &ts) < 0) {
            ERROR("clock_gettime failed.(%s)", strerror(errno));
            pthread_mutex_unlock(&sCondLock);
            usleep(30000000LL); /* sleep 30 second */
            continue;
        }

        ts.tv_sec += sShouldPoll ? SLEEP_POLL_SEC : SLEEP_NOPOLL_SEC;
        DEBUG("shouldPoll:%d", sShouldPoll);
        pthread_cond_timedwait(&sCondition, &sCondLock, &ts);
        pthread_mutex_unlock(&sCondLock);
    }

    ERROR("monitor_looper exit");
    return NULL;
}

void HealthListener::initHealthService() {
    mHealth = get_health_service();
    if (mHealth == NULL) {
        ERROR("get health service failed");
        return;
    }

    mHealth->registerCallback(this);
    mHealth->linkToDeath(this, 0);

}

void HealthListener::kickHealthdIfNeeded() {
    if (mHealth == NULL)
        kickHealthd();
}

void HealthListener::kickHealthd() {
    if (mHealth == NULL)
        initHealthService();

    /* failed anyway */
    if (mHealth == NULL)
        return;

    /* update health information */
    Result r = mHealth->update();
    if (r != Result::SUCCESS) {
        ERROR("health update failed(ret=%d).", r);
        mHealth = NULL;
    }
}

void HealthListener::updatePollInterval(bool shouldPoll) {
    pthread_mutex_lock(&sCondLock);
    if (shouldPoll != sShouldPoll) {
        DEBUG("shouldPoll changed:%d->%d", sShouldPoll, shouldPoll);
        sShouldPoll = shouldPoll;
        pthread_cond_signal(&sCondition);
    }
    pthread_mutex_unlock(&sCondLock);
}

/* Referring to android/hardware/interfaces/health/1.0/types.hal */
Return<void> HealthListener::healthInfoChanged(const HealthInfo& props) {
    Mutex::Autolock _l(sLock);
    DEBUG("healthInfoChanged,"
            "chargerAcOnline=%d,"
            "chargerUsbOnline=%d,"
            "chargerWirelessOnline=%d,"
            "maxChargingCurrent=%d,"
            "maxChargingVoltage=%d,"
            "batteryStatus=%d,"
            "batteryHealth=%d,"
            "batteryPresent=%d,"
            "batteryLevel=%d,"
            "batteryVoltage=%d,"
            "batteryTemperature=%d,"
            "batteryCurrent=%d,"
            "batteryCycleCount=%d,"
            "batteryFullCharge=%d,"
            "batteryChargeCounter=%d,"
            "batteryTechnology=%s",
            props.legacy.chargerAcOnline,
            props.legacy.chargerUsbOnline,
            props.legacy.chargerWirelessOnline,
            props.legacy.maxChargingCurrent,
            props.legacy.maxChargingVoltage,
            props.legacy.batteryStatus,
            props.legacy.batteryHealth,
            props.legacy.batteryPresent,
            props.legacy.batteryLevel,
            props.legacy.batteryVoltage,
            props.legacy.batteryTemperature,
            props.legacy.batteryCurrent,
            props.legacy.batteryCycleCount,
            props.legacy.batteryFullCharge,
            props.legacy.batteryChargeCounter,
            props.legacy.batteryTechnology.c_str());

    BinderMessage msg;

    msg.what = BINDER_MESSAGE_BATTERY_TEMP;
    msg.value = props.legacy.batteryTemperature;
    OIfaceServer::getInstance().sendMessage(msg);

    msg.what = BINDER_MESSAGE_BATTERY_LEVEL;
    msg.value = props.legacy.batteryLevel;
    OIfaceServer::getInstance().sendMessage(msg);

    msg.what = BINDER_MESSAGE_BATTERY_CURRENT;
    msg.value = props.legacy.batteryCurrent;
    OIfaceServer::getInstance().sendMessage(msg);

    msg.what = BINDER_MESSAGE_CHARGER;
    msg.value = props.legacy.chargerAcOnline;
    OIfaceServer::getInstance().sendMessage(msg);

    return android::hardware::Void();
}

void HealthListener::serviceDied(uint64_t cookie, const wp<::android::hidl::base::V1_0::IBase>& who) {
    Mutex::Autolock _l(sLock);

    if (mHealth != NULL && interfacesEqual(mHealth, who.promote())) {
        ERROR("hidl health service died");
    }

    mHealth = NULL;
}
