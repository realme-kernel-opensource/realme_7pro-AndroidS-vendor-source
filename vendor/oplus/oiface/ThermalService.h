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
 * zhoubugang       2019/5/30        1.0       Add ThermalService
 * ------------------------------------------------------------------------------
 */

#ifndef __THERMAL_SERVICE_H__
#define __THERMAL_SERVICE_H__

#include <string>
#include <vector>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <android/hardware/thermal/1.0/IThermal.h>

class ThermalServiceDeathRecipient;
class ThermalService : public android::Singleton<ThermalService> {
public:
    int getCpuTemperature();
    int getGpuTemperature();
    int getBatteryTemperature();
    int getSkinTemperature();
    int getAllTemperature(std::vector<float> &temp);
    void handleServiceDied();
    // These values must be kept in sync with the temperature source constants
    enum {
        TEMPERATURE_CPU = 0,
        TEMPERATURE_GPU = 1,
        TEMPERATURE_BATTERY = 2,
        TEMPERATURE_SKIN = 3,
        TEMPERATURE_COUNT = 4
    };
    enum {
        TEMPERATURE_CURRENT = 0,
        TEMPERATURE_THROTTLING = 1,
        TEMPERATURE_SHUTDOWN = 2,
        TEMPERATURE_THROTTLING_BELOW_VR_MIN = 3
    };
private:
    friend class android::Singleton<ThermalService>;
    friend class ThermalServiceDeathHandler;
    ThermalService();
    ~ThermalService();
    bool checkService();
    int getDeviceTemperature();
    std::vector<float> mTemperatures;
    android::Mutex mLock;
    android::sp<android::hardware::thermal::V1_0::IThermal> mService = nullptr;
    android::sp<ThermalServiceDeathRecipient> mDeathRecipient = nullptr;
};

class ThermalServiceDeathRecipient : public android::hardware::hidl_death_recipient {
public:
    ThermalServiceDeathRecipient(ThermalService& Service)
          : mThermalService(Service) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) override;
private:
    ThermalService& mThermalService;
};


#endif
