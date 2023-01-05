#ifndef __BATTERRY_LISTENER_H__
#define __BATTERRY_LISTENER_H__

#include <utils/Singleton.h>
#include <android/hardware/health/2.0/IHealth.h>

class HealthListener: public android::hardware::health::V2_0::IHealthInfoCallback,
        public android::hardware::hidl_death_recipient,
        public android::Singleton<HealthListener> {
    public:
        HealthListener();
        virtual ~HealthListener() {};
        void updatePollInterval(bool shouldPoll);
        int32_t getSkinTemperature();
        int32_t getCpuTemperature();

    protected:
        virtual ::android::hardware::Return<void> healthInfoChanged(
                const ::android::hardware::health::V2_0::HealthInfo& info);
        virtual void serviceDied(uint64_t cookie, const android::wp<::android::hidl::base::V1_0::IBase>& who);
    private:
        static const int ERROR_NODE_NOT_FOUND;
        static const int ERROR_INVALID_VALUE;
        android::sp<android::hardware::health::V2_0::IHealth> mHealth;
        std::map<std::string, std::string> mThermalList;
        static bool sShouldPoll;
        static pthread_cond_t sCondition;
        static pthread_mutex_t sCondLock;

        void initHealthService();
        void kickHealthd();
        void kickHealthdIfNeeded();
        void updateThermalStats();

        static android::Mutex sLock;
        static void * monitor_thread(void *cookie);
};

#endif
