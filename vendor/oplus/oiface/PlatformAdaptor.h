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

#ifndef __PLATFORM_ADAPTOR__
#define __PLATFORM_ADAPTOR__

#include <stdio.h>
#include <string>
#include <vector>
#include "OIfaceModule.h"

using namespace std;

class PlatformAdaptor {
//Properties List
public:
    struct CpuInfo {
        int cpuCluster;
        //int coresInCluster[CLUSTER_NUM][MAX_CLUSTER_CORE];
        std::vector<std::vector<int>> coresInCluster;
        //int cpuFreqInfo[CLUSTER_NUM][MAX_CPUFREQ_NUM];
        std::vector<std::vector<uint64_t>> cpuFreqInfo;
    };

    struct GpuInfo {
        bool faster_way;
        std::string gpu_load;
    };

private:
    static PlatformAdaptor* adaptor;
    bool mIsQcom;
    int mPlatform;
    std::string mProject;

    struct PlatformInfo {
        int platform;
        const char name[20];
    };

    typedef struct tag_chipid {
	    unsigned int size;	
	    unsigned int hw_code;	
	    unsigned int hw_subcode;	
	    unsigned int hw_ver;	
	    unsigned int sw_ver;
    } mtkChipInfo;

    CpuInfo mCpuInfo;
    GpuInfo mGpuInfo;

    static PlatformInfo sPlatformList[PLAT_NUM];


//Method List
public:
    static PlatformAdaptor& getInstance() {
        PlatformAdaptor* instance = adaptor;

        if (instance == NULL) {
            instance = new PlatformAdaptor();
            adaptor = instance;
        }

        return *instance;
    }

    bool isQcom() { return mIsQcom; }
    bool isMTK() { return !mIsQcom; }

    int getPlatform() { return mPlatform; };
    std::string getPlatformName() { return sPlatformList[mPlatform].name; }

    std::string getBoardName() { return mProject; }
    std::string getProject() { return mProject; }

    void dumpCpufreqInfo();
    int getClusterNum();

    const struct CpuInfo getCpuInfo();
    const struct GpuInfo getGpuInfo();

private:
    PlatformAdaptor(const PlatformAdaptor&);
    PlatformAdaptor& operator = (const PlatformAdaptor&);
    ~PlatformAdaptor() {};
    PlatformAdaptor();
    void readPlatformCpufreqInfo();
    void readPlatformGpuInfo();
    void parseCpufreqInfo(string &freq, int cluster);
    void parseCoresPerCluster(string &cores, int cluster);
    void getMtkChipVersion(const std::string& path, mtkChipInfo& mtkinfo);
};

#endif
