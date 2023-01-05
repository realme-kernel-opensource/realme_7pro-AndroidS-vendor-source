//
// Created by 80302108 on 2021/04/07.
//

#include "OplusTouchService.h"
#include "OIface.h"

#include <math.h>
#include <stdlib.h>

#include <utils/Log.h>
#include <utils/String8.h>

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
using ::vendor::oplus::hardware::touch::V1_0::IOplusTouch;
using android::hardware::hidl_string;

template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(OplusTouchService);

OplusTouchService::OplusTouchService() {
}

OplusTouchService::~OplusTouchService() {
}

bool OplusTouchService::checkService() {
    if (mService != nullptr) {
        return true;
    } else {
        mService = IOplusTouch::getService();
        if (mService == nullptr) {
            ERROR("unable to get OplusTouch service");
        } else {
            if (mDeathRecipient == nullptr) {
                mDeathRecipient = new OplusTouchServiceDeathRecipient(*this);
            }
            hardware::Return<bool> linked = mService->linkToDeath(mDeathRecipient, 0x456F);
            if (!linked.isOk()) {
                ERROR("Transaction error in linking to OplusTouch Service death: %s", linked.description().c_str());
            } else if (!linked) {
                ERROR("Unable to link to OplusTouch Service death notifications");
            } else {
                DEBUG("Link to death notification successful");
                return true;
            }
        }
    }
    mService = nullptr;
    return false;
}

void OplusTouchService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
}

void OplusTouchServiceDeathRecipient::serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) {
    ERROR("OplusTouchService just died");
    mOplusTouchService.handleServiceDied();
}

std::string OplusTouchService::getTouchAdjusterSupportStatus(int32_t deviceId, int32_t nodeFlag){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get OplusTouch service in query mode.");
        return "-1";
    }
    std::string ret;
    int32_t result = mService->isTouchNodeSupport(deviceId, nodeFlag);

    DEBUG("result: %d\n", result);
    ret = to_string(result);

    return ret;
}

int32_t OplusTouchService::writeTouchAdjusterNode(int32_t deviceId, int32_t nodeFlag, std::string info) {
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get OplusTouch service in write mode.");
        return -1;
    }
    int32_t ret = mService->touchWriteNodeFile(deviceId, nodeFlag, info);

    DEBUG("ret: %d\n", ret);

    return ret;
}

std::string OplusTouchService::readTouchAdjusterNode(int32_t deviceId, int32_t nodeFlag) {
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get OplusTouch service in read mode.");
        return "-1";
    }
    std::string ret = "";
    mService->touchReadNodeFile(deviceId, nodeFlag, [&](std::string resultValue) {
	    ret = resultValue;
    });

    DEBUG("ret: %s\n", ret.c_str());

    return ret;
}