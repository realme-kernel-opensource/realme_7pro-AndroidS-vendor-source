#include "NetworkLatencyManager.h"
#include "OplusOIfaceService.h"
#include "OIface.h"
#include "DataCollector.h"

using namespace android;
using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE(NetworkLatencyManager);
const int32_t NetworkLatencyManager::kInterval[] = {10, 50, 90, 110, 130, 150, 190, 230,
    300, 400, INT_MAX};

int NetworkLatencyManager::registerNetworkListener(int32_t thresholdMs, int32_t minReportMs,
        sp<IBinder> binder) {
    DEBUG("registering network latency listener:%d %d", thresholdMs, minReportMs);

    android::Mutex::Autolock _l(mLock);
    NetworkCallback callback;
    callback.thresholdMs = thresholdMs;
    callback.minReportMs = minReportMs;
    callback.callback = binder;
    callback.lastReportMs = -1;
    mCallbackList[binder] = callback;

    return 0;
}

void NetworkLatencyManager::reportNetworkLatency(const std::string& pkgName, int32_t latency,
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
