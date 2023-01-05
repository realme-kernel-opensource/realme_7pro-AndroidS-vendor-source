//
// Created by 80302108 on 2021/04/07.
//

#ifndef ANDROIDR_OPLUSTOUCHSERVICE_H
#define ANDROIDR_OPLUSTOUCHSERVICE_H

#include <string>
#include <vector>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <vendor/oplus/hardware/touch/1.0/IOplusTouch.h>

class OplusTouchServiceDeathRecipient;

class OplusTouchService : public android::Singleton<OplusTouchService> {
public:
    std::string getTouchAdjusterSupportStatus(int32_t deviceId, int32_t nodeFlag);
    int32_t writeTouchAdjusterNode(int32_t deviceId, int32_t nodeFlag, std::string info);
    std::string readTouchAdjusterNode(int32_t deviceId, int32_t nodeFlag);

    void handleServiceDied();

private:
    friend class android::Singleton<OplusTouchService>;
    OplusTouchService();
    ~OplusTouchService();
    bool checkService();

    android::Mutex mLock;
    android::sp<::vendor::oplus::hardware::touch::V1_0::IOplusTouch> mService = nullptr;
    android::sp<OplusTouchServiceDeathRecipient> mDeathRecipient = nullptr;
};

class OplusTouchServiceDeathRecipient : public android::hardware::hidl_death_recipient {
public:
    OplusTouchServiceDeathRecipient(OplusTouchService& Service): mOplusTouchService(Service) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) override;
private:
    OplusTouchService& mOplusTouchService;
};

#endif //ANDROIDR_OPLUSTOUCHSERVICE_H
