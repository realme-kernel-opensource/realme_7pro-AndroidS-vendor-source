#include "OIface.h"
#include "ChargerService.h"

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
using ::vendor::oplus::hardware::charger::V1_0::ICharger;

template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(ChargerService);

ChargerService::ChargerService() {
    //Reno6 series
    superVOOCMap["20171"] = 1;
    superVOOCMap["20172"] = 1;
    superVOOCMap["20353"] = 1;
    superVOOCMap["20181"] = 1;
    superVOOCMap["20355"] = 1;
    //Reno5 series
    superVOOCMap["20261"] = 1;
    superVOOCMap["20262"] = 1;
    superVOOCMap["20263"] = 1;
    superVOOCMap["20111"] = 1;
    superVOOCMap["20113"] = 1;
    superVOOCMap["20251"] = 1;
    superVOOCMap["20131"] = 1;
    superVOOCMap["20133"] = 1;
    superVOOCMap["20255"] = 1;
    superVOOCMap["20235"] = 1;
    superVOOCMap["20135"] = 1;
    superVOOCMap["20137"] = 1;
    //Find X2 series
    superVOOCMap["19063"] = 1;
    superVOOCMap["19065"] = 1;
    superVOOCMap["19066"] = 1;
    superVOOCMap["19361"] = 1;
    superVOOCMap["19362"] = 1;
    //Reno4 series
    superVOOCMap["19191"] = 1;
    superVOOCMap["19192"] = 1;
    superVOOCMap["19015"] = 1;
    superVOOCMap["19016"] = 1;
    superVOOCMap["19591"] = 1;
    superVOOCMap["19525"] = 1;
    //Ace2
    superVOOCMap["19161"] = 1;
    //Reno Ace
    superVOOCMap["19081"] = 1;
}

ChargerService::~ChargerService() {
}

bool ChargerService::checkService() {
    if (mService != nullptr) {
        return true;
    } else {
        mService = ICharger::getService();
        if (mService == nullptr) {
            ERROR("unable to get Charger service");
        } else {
            if (mDeathRecipient == nullptr) {
                mDeathRecipient = new ChargerServiceDeathRecipient(*this);
            }
            hardware::Return<bool> linked = mService->linkToDeath(mDeathRecipient, 0x456F);
            if (!linked.isOk()) {
                ERROR("Transaction error in linking to Charger Service death: %s", linked.description().c_str());
            } else if (!linked) {
                ERROR("Unable to link to Charger Service death notifications");
            } else {
                DEBUG("Link to death notification successful");
                return true;
            }
        }
    }
    mService = nullptr;
    return false;
}

void ChargerService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
}

void ChargerServiceDeathRecipient::serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) {
    ERROR("ChargerService just died");
    mChargerService.handleServiceDied();
}


int ChargerService::getBatteryRemain(){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get battery remain because of ChargerService error.");
        return -1;
    }
    return (int)mService->getPsyBatteryRm();
}
float ChargerService::getBatteryCurrentNow(){
    if(superVOOCStatus == -1){
        std::string peojectName = OplusProjectService::getInstance().getProjectString();
        DEBUG("project name: %s\n", peojectName.c_str());
        superVOOCStatus = getSuperVOOCStatus(peojectName);
    }
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get battery remain because of ChargerService error.");
        return -1;
    }
    int originalBatteryCurrentNow = (int)mService->getPsyBatteryCurrentNow();
    if(superVOOCStatus == 1){
        return ((float)originalBatteryCurrentNow - 4.0) * 1.95;
    }
    return (float)originalBatteryCurrentNow;
}

int ChargerService::getBatteryFCC(){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get battery Fcc because of ChargerService error.");
        return -1;
    }
    return (int)mService->getPsyBatteryFcc();
}

int ChargerService::getSuperVOOCStatus(std::string projectName){
    map<std::string, int>::iterator iter = superVOOCMap.find(projectName);
    if (iter == superVOOCMap.end()) {
        return 0;
    } else {
        return superVOOCMap[projectName];
    }
}

int ChargerService::getAcOnline(){
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get ac online because of ChargerService error.");
        return -1;
    }
    return (int)mService->getPsyAcOnline();
}
// int ChargerService::getBatteryLevel(){
//     float batteryLevel = (float)getBatteryRemain() / getBatteryFcc() * 100.0;
//     DEBUG("batteryLevel_float: %f", batteryLevel);
//     return (int)(batteryLevel + 0.5);
// }
