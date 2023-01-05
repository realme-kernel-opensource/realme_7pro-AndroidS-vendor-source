#ifndef __NETWORK_LATENCY_MANAGER_H__
#define __NETWORK_LATENCY_MANAGER_H__
#include <utils/Singleton.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>

class NetworkLatencyManager: public android::Singleton<NetworkLatencyManager> {
    public:
        NetworkLatencyManager() {};
        virtual ~NetworkLatencyManager() {};
        int registerNetworkListener(int32_t thresholdMs, int32_t minReportMs, android::sp<android::IBinder> binder);
        void reportNetworkLatency(const std::string& pkgName, int32_t latency, int32_t lastReportTime, int32_t lastLatecy);

        struct NetworkCallback {
            int32_t thresholdMs;
            int32_t minReportMs;
            int32_t lastReportMs;
            android::sp<android::IBinder> callback;
        };

    private:
        /* Use vector is enough?? */
        std::map<android::wp<android::IBinder>, NetworkCallback> mCallbackList;
        mutable android::Mutex mLock;
        static const int32_t kInterval[];
};

#endif
