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


#include "OIface.h"
#include "OIfaceHalService.h"

using namespace std;
using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE(OIfaceHalService);

// FIXME 
#if 0
OIfaceHalService::OIfaceHalService() : mService(0) {
}
#else
OIfaceHalService::OIfaceHalService() {
}
#endif

bool OIfaceHalService::checkService() {
    if (mService == NULL) {
        mService = vendor::oplus::hardware::oiface::V1_0::IOiface::getService();
        if (mService != NULL) {
            mOHSDeathHandler = new OIfaceHalServiceDeathHandler(*this);
            mService->linkToDeath(mOHSDeathHandler, 0x6f696663);
        } else {
            ERROR("OIfaceHalService get failed!");
        }
    }
    return mService != NULL;
}

int OIfaceHalService::readFile(const string& path, string& out) {
    Mutex::Autolock _l(mLock);
    if (!checkService())
        return -1;

    mService->readFile(path, [&](android::hardware::hidl_string result) {
        out = result.c_str();
    });

    return 0;
}

int OIfaceHalService::writeFile(const std::string& path, const std::string& str) {
    Mutex::Autolock _l(mLock);
    if (!checkService())
        return -1;

    return mService->writeFile(path, str);
}

void OIfaceHalService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
    ERROR("OIfaceHalService died!");
}

void OIfaceHalServiceDeathHandler::serviceDied(uint64_t /*cookie*/,
        const android::wp<::android::hidl::base::V1_0::IBase>& /*who*/) {
    mHalService.handleServiceDied();
}

