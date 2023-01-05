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
 * huangzhigen     2019/03/06   1.0         Add OIfaceHalService
 * ------------------------------------------------------------------------------
 */

#ifndef __OIFACE_HAL_SERVICE_H__
#define __OIFACE_HAL_SERVICE_H__

#include <string>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
// #include <vendor/oplus/hardware/oiface/1.0/IOiface.h>

class OIfaceHalServiceDeathHandler;
class OIfaceHalService : public android::Singleton<OIfaceHalService> {
public:
    int readFile(const std::string& path, std::string& out);
    int writeFile(const std::string& path, const std::string& str);

private:
    friend class android::Singleton<OIfaceHalService>;
    friend class OIfaceHalServiceDeathHandler;
    OIfaceHalService();
    ~OIfaceHalService() {}
    bool checkService();
    void handleServiceDied();

    android::Mutex mLock;

    // FIXME
    
};

class OIfaceHalServiceDeathHandler : public android::hardware::hidl_death_recipient {
public:
    OIfaceHalServiceDeathHandler(OIfaceHalService& halService)
          : mHalService(halService) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who);

private:
    OIfaceHalService& mHalService;
};


#endif
