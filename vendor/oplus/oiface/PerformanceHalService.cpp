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
 * shanghaoqiang             2021/07/05      1.0     Add PerformanceHalService
 * ------------------------------------------------------------------------------
 */


#include "OIface.h"
#include "PerformanceHalService.h"

using namespace std;
using namespace android;
using ::android::hardware::hidl_string;
using ::android::hardware::Void;

template<typename T>
using Return = hardware::Return<T>;

ANDROID_SINGLETON_STATIC_INSTANCE(PerformanceHalService);

PerformanceHalService::PerformanceHalService() {
}

bool PerformanceHalService::checkService() {
    if (mService == NULL) {
        mService = vendor::oplus::hardware::performance::V1_0::IPerformance::getService();
        if (mService != NULL) {
            mOHSDeathHandler = new PerformanceHalServiceDeathHandler(*this);
            mService->linkToDeath(mOHSDeathHandler, 0x6f696663);
        } else {
            ERROR("PerformanceHalService get failed!");
        }
    }
    return mService != NULL;
}

int PerformanceHalService::readNandswapProc(const string& path, string& out) {
    Mutex::Autolock _l(mLock);
    if (!checkService())
        return -1;

    Return<void> ret = mService->readNandswapProc("swap_ctl", [&](const hidl_string &buf) {
        DEBUG("buf=%s", buf.c_str());
         out = buf;
    });

    if (!ret.isOk()) {
        ERROR("readNandswapProc failed status: %s", ret.description().c_str());
        return -1;
    }

    DEBUG("readNandswapProc result! %s", out.c_str());
    return 0;
}

int PerformanceHalService::writeNandswapProc(const std::string& path, const std::string& str) {
    Mutex::Autolock _l(mLock);
    if (!checkService())
        return -1;
    int32_t ret = mService->writeNandswapProc("swap_ctl", str);

    DEBUG("writeNandswapProc result! %d", ret);
    return ret;
}

int PerformanceHalService::getNandswapVersion() {
    Mutex::Autolock _l(mLock);
    if (!checkService()) {
        return -1;
    }

    int32_t ret = mService->getKernelVersion();
    if (ret >= 510) {
        return 2;
    } else {
        return 1;
    }
}

int PerformanceHalService::writeNandswapProc(int32_t uid, int32_t pid, int32_t type, int swapType) {
    Mutex::Autolock _l(mLock);
    if (!checkService()) {
        return -1;
    }

    int32_t ret = 0;
    if (swapType == 0) {
        ret = mService->perProcessMemReadahead(uid, pid, RECLAIM_ALL);
    } else if (swapType == 1) {
        ret = mService->perProcessMemReclaim(uid, pid, RECLAIM_ALL);
    }

    DEBUG("writeNandswapProc result! %d, swapType :%d", ret, swapType);
    return ret;
}

void PerformanceHalService::handleServiceDied() {
    Mutex::Autolock _l(mLock);
    mService = NULL;
    DEBUG("PerformanceHalService died!");
}

void PerformanceHalServiceDeathHandler::serviceDied(uint64_t /*cookie*/,
        const android::wp<::android::hidl::base::V1_0::IBase>& /*who*/) {
    mHalService.handleServiceDied();
}

