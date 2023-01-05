#include "OIface.h"
#include "OplusProjectService.h"

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
using ::vendor::oplus::hardware::stability::oplus_project::V1_0::IOplusProject;
using android::hardware::hidl_string;

template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(OplusProjectService);

OplusProjectService::OplusProjectService() {
}

OplusProjectService::~OplusProjectService() {
}

bool OplusProjectService::checkService() {
    if (mService != nullptr) {
        return true;
    } else {
        mService = IOplusProject::getService();
        if (mService == nullptr) {
            ERROR("unable to get OplusProject service");
        } else {
            if (mDeathRecipient == nullptr) {
                mDeathRecipient = new OplusProjectServiceDeathRecipient(*this);
            }
            hardware::Return<bool> linked = mService->linkToDeath(mDeathRecipient, 0x456F);
            if (!linked.isOk()) {
                ERROR("Transaction error in linking to OplusProject Service death: %s", linked.description().c_str());
            } else if (!linked) {
                ERROR("Unable to link to OplusProject Service death notifications");
            } else {
                DEBUG("Link to death notification successful");
                return true;
            }
        }
    }
    mService = nullptr;
    return false;
}

void OplusProjectService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
}

void OplusProjectServiceDeathRecipient::serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) {
    ERROR("OplusProjectService just died");
    mOplusProjectService.handleServiceDied();
}

uint32_t OplusProjectService::getProject(){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get project because of OplusProject error.");
        return -1;
    }
    return mService->get_project();
}

std::string OplusProjectService::getProjectString(){
    uint32_t projectName = getProject();
    DEBUG("HAL return %d", (int)projectName);
    return std::to_string(projectName);
}


std::string OplusProjectService::getSerialID(){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get project because of OplusProject error.");
        return "-1";
    }
    std::string serialID;
    mService->get_serial_ID([&](hidl_string result) {
        DEBUG("serial_ID: %s\n", result.c_str());
        serialID = std::string(result);
    });
    return serialID;
}

