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
 * ------------------------------------------------------------------------------
 */

#ifndef __THREAD_STATE_H__
#define __THREAD_STATE_H__

#include <pthread.h>

#include <string>
#include <map>

#include <utils/Mutex.h>

#include "OIface.h"
using namespace std;

class ThreadState {
private:
    static ThreadState* sInstance;
    string mCurrentPackage;
    string mLastPackage;
    int mPid;
    /* response callback in NEO */
    int (*mRespCallback)(oiface_data_response* resp);
    int mWifiStatus;
    int mDataStatus;
    bool mActivityChanged;
    mutable android::Mutex mLock;
    map<int, uint32_t> mUidConfig;
    ~ThreadState() {};
    ThreadState();
public:
    static ThreadState& getInstance() {
        ThreadState* instance = sInstance;

        if (instance == NULL) {
            instance = new ThreadState();
            sInstance = instance;
        }

        return *instance;
    }

    /* interact with neo */
    int sendDecisionInfo(const Decision& decision);
    bool isPackageForground(string app);
    std::string getForgroundPackage();
    int getForgroundPid();
    void updateSysInfo(oiface_sys_info* info);
    void setOifaceResponse(OIFACE_SEND_DATA_TO_NS resp) {
        mRespCallback = resp;
    };
    bool isCurrentWifi() {
        return (mWifiStatus == 1);
    };
    bool isCurrentData() {
        return (mDataStatus == 1);
    };
    int getConfig(int uid, uint32_t *value);
    int setConfig(int uid, uint32_t value);
    bool isActivityChanged() {
        return mActivityChanged;
    }
    void resetActivityChanged() {
         mActivityChanged = false;
    }

    char *dumpNetworkStatus();
};

#endif
