#include "OIface.h"
#include "GameOptService.h"

#include <math.h>
#include <stdlib.h>

#include <utils/Log.h>
#include <utils/String8.h>
#include "OplusProjectService.h"

using namespace std;
using namespace android;

using android::hidl::base::V1_0::IBase;
using hardware::hidl_death_recipient;

using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Void;
using ::android::sp;
using ::vendor::oplus::hardware::gameopt::V1_0::IGameOptHalService;

template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(GameOptService);

GameOptService::GameOptService() {
    ERROR("GameOptService init");
}

GameOptService::~GameOptService() {
}

void GameOptService::init() {
    ERROR("GameOpt service init");
    checkService();
}

bool GameOptService::checkService() {
    if (mService != nullptr) {
        return true;
    } else {
        mService = IGameOptHalService::getService();
        if (mService == nullptr) {
            ERROR("unable to get GameOpt service");
        } else {
            if (mDeathRecipient == nullptr) {
                mDeathRecipient = new GameOptServiceDeathRecipient(*this);
            }
            hardware::Return<bool> linked = mService->linkToDeath(mDeathRecipient, 0x456F);
            if (!linked.isOk()) {
                ERROR("Transaction error in linking to GameOpt Service death: %s", linked.description().c_str());
            } else if (!linked) {
                ERROR("Unable to link to GameOpt Service death notifications");
            } else {
                DEBUG("Link to death notification successful");
                return true;
            }
        }
    }
    mService = nullptr;
    return false;
}

void GameOptService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
}

void GameOptServiceDeathRecipient::serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) {
    ERROR("GameOptService just died");
    mGameOptService.handleServiceDied();
}
