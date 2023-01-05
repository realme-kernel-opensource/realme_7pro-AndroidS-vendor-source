#ifndef __GAMEOPT_SERVICE_H__
#define __GAMEOPT_SERVICE_H__

#include <string>
#include <vector>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <vendor/oplus/hardware/gameopt/1.0/IGameOptHalService.h>
#include <map>

using namespace std;


class GameOptServiceDeathRecipient;
class GameOptService : public android::Singleton<GameOptService> {
public:
    void init();
    void handleServiceDied();
    GameOptService();
    ~GameOptService();

private:
    bool checkService();
    friend class android::Singleton<GameOptService>;
    friend class GameOptServiceDeathHandler;

    android::Mutex mLock;
    android::sp<::vendor::oplus::hardware::gameopt::V1_0::IGameOptHalService> mService = nullptr;
    android::sp<GameOptServiceDeathRecipient> mDeathRecipient = nullptr;
};

class GameOptServiceDeathRecipient : public android::hardware::hidl_death_recipient {
public:
    GameOptServiceDeathRecipient(GameOptService& Service): mGameOptService(Service) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) override;
private:
    GameOptService& mGameOptService;
};

#endif

