#ifndef __CHARGER_SERVICE_H__
#define __CHARGER_SERVICE_H__

#include <string>
#include <vector>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <vendor/oplus/hardware/charger/1.0/ICharger.h>
#include <map>

using namespace std;


class ChargerServiceDeathRecipient;
class ChargerService : public android::Singleton<ChargerService> {
public:
    int getBatteryRemain();
    float getBatteryCurrentNow();
    int getBatteryFCC();
    int getAcOnline();
    // int getBatteryLevel();
    void handleServiceDied();
    int getSuperVOOCStatus(std::string projectName);

private:
    friend class android::Singleton<ChargerService>;
    friend class ChargerServiceDeathHandler;
    ChargerService();
    ~ChargerService();
    bool checkService();

    android::Mutex mLock;
    android::sp<::vendor::oplus::hardware::charger::V1_0::ICharger> mService = nullptr;
    android::sp<ChargerServiceDeathRecipient> mDeathRecipient = nullptr;

    std::map<std::string, int> superVOOCMap;
    int superVOOCStatus = -1;
};

class ChargerServiceDeathRecipient : public android::hardware::hidl_death_recipient {
public:
    ChargerServiceDeathRecipient(ChargerService& Service): mChargerService(Service) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) override;
private:
    ChargerService& mChargerService;
};


#endif

