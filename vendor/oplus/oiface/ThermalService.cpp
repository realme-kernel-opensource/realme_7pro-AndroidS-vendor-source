/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ------------------------------- Revision History: ----------------------------
 * <author>                   <date>       <version>   <desc>
 * zhoubugang     2019/05/30        1.0      add ThermalService
 * ------------------------------------------------------------------------------
 */


#include "OIface.h"
#include "ThermalService.h"

#include <math.h>
#include <stdlib.h>

#include <utils/Log.h>
#include <utils/String8.h>

using namespace std;
using namespace android;

using android::hidl::base::V1_0::IBase;
using hardware::hidl_death_recipient;
using hardware::hidl_vec;
using hardware::thermal::V1_0::CoolingDevice;
using hardware::thermal::V1_0::CpuUsage;
using hardware::thermal::V1_0::IThermal;
using hardware::thermal::V1_0::Temperature;
using hardware::thermal::V1_0::ThermalStatus;
using hardware::thermal::V1_0::ThermalStatusCode;
template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(ThermalService);

ThermalService::ThermalService() {
    mTemperatures.resize(TEMPERATURE_COUNT, -1);
}

ThermalService::~ThermalService() {

}

bool ThermalService::checkService() {
    if (mService != nullptr) {
        return true;
    } else {
        mService = IThermal::getService();
        if (mService == nullptr) {
            ERROR("unable to get Thermal service");
        } else {
            if (mDeathRecipient == nullptr) {
                mDeathRecipient = new ThermalServiceDeathRecipient(*this);
            }
            hardware::Return<bool> linked = mService->linkToDeath(mDeathRecipient, 0x456F /* cookie */);
            if (!linked.isOk()) {
                ERROR("Transaction error in linking to Thermal Service death: %s", linked.description().c_str());
            } else if (!linked) {
                ERROR("Unable to link to Thermal Service death notifications");
            } else {
                DEBUG("Link to death notification successful");
                return true;
            }
        }
    }
    mService = nullptr;
    return false;
}

void ThermalService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
}

void ThermalServiceDeathRecipient::serviceDied(uint64_t /*cookie*/, const wp<IBase>& /*who*/) {
    ERROR("ThermalService just died");
    mThermalService.handleServiceDied();
}

int ThermalService::getDeviceTemperature() {
    Mutex::Autolock _l(mLock);
    if(!checkService()) {
        DEBUG("Couldn't get device temperatures because of ThermalService error.");
        return -1;
    }
    hidl_vec<Temperature> list;
    Return<void> ret = mService->getTemperatures(
            [&list](ThermalStatus status, hidl_vec<Temperature> temperatures) {
                if (status.code == ThermalStatusCode::SUCCESS) {
                    list = std::move(temperatures);
                } else {
                    ERROR("Couldn't get temperatures because of HAL error: %s",
                          status.debugMessage.c_str());
                }
            });

    if (!ret.isOk()) {
        ERROR("getDeviceTemperature failed status: %s", ret.description().c_str());
        return -1;
    }
    if(list.size() < TEMPERATURE_COUNT) {
        ERROR("getDeviceTemperature failed size: %d", int(list.size()));
        return -1;
    }
    float value;
    for (size_t i = 0; i < list.size(); ++i) {
        DEBUG("list %zu name: %s", i, list[i].name.c_str());
        value =  float(list[i].currentValue);
        switch (int(list[i].type)) {
            case TEMPERATURE_CPU:
                if(value > mTemperatures[TEMPERATURE_CPU]) {
                    mTemperatures[TEMPERATURE_CPU] = value;
                    DEBUG("Cpu temerature is :%f", value);
                }
                break;
            case TEMPERATURE_GPU:
                mTemperatures[TEMPERATURE_GPU] = value;
                DEBUG("Gpu temerature is :%f", value);
                break;
            case TEMPERATURE_BATTERY:
                mTemperatures[TEMPERATURE_BATTERY] = value;
                DEBUG("Battery temerature is :%f", value);
                break;
            case TEMPERATURE_SKIN:
                if((0 != strcmp(list[i].name.c_str(), "skin")) && (value > mTemperatures[TEMPERATURE_SKIN])) {
                    mTemperatures[TEMPERATURE_SKIN] = value;
                    DEBUG("shell temerature is :%f", value);
                }
                break;
            default:
                DEBUG("Unknown temperature %s type", list[i].name.c_str());
                break;
        }
    }
    return 0;
}

int ThermalService::getCpuTemperature() {
    if (getDeviceTemperature() == 0) {
        return mTemperatures[TEMPERATURE_CPU];
    } else {
        ERROR("getCpuTemperature failed status");
        return -1;
    }
}

int ThermalService::getGpuTemperature() {
    if (getDeviceTemperature() == 0) {
        return mTemperatures[TEMPERATURE_GPU];
    } else {
        ERROR("getCpuTemperature failed status");
        return -1;
    }
}

int ThermalService::getBatteryTemperature() {
    if (getDeviceTemperature() == 0) {
        return mTemperatures[TEMPERATURE_BATTERY];
    } else {
        ERROR("getCpuTemperature failed status");
        return -1;
    }
}

int ThermalService::getSkinTemperature() {
    if (getDeviceTemperature() == 0) {
        return mTemperatures[TEMPERATURE_SKIN];
    } else {
        ERROR("getCpuTemperature failed status");
        return -1;
    }
}

int ThermalService::getAllTemperature(vector<float> &temp) {
    if (getDeviceTemperature() == 0) {
        temp = mTemperatures;
        return 0;
    } else {
        ERROR("getAllTemperature failed status");
        return -1;
    }
}
