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

#include <PlatformAdaptor.h>
#include <utils/Log.h>
#include <algorithm>
#include "Utils.h"
#include "OIface.h"
#include <string.h>
#include "OplusProjectService.h"

using namespace std;

//QCOM soc id
#define SOC_ID_PATH            "/sys/devices/soc0/soc_id"
#define RAW_ID_PATH            "/sys/devices/soc0/raw_version"
//MTK soc id
#define PROC_CHIP_INFO_V1      "/proc/device-tree/chosen/atag,chipid"

#define CPUFREQ_BASE_DIR       "/sys/devices/system/cpu/cpufreq/"

PlatformAdaptor* PlatformAdaptor::adaptor = NULL;

/* keep the same order with the defination of enumerate PLATFORM */
PlatformAdaptor::PlatformInfo PlatformAdaptor::sPlatformList[PLAT_NUM] = {
    {PLAT_MSM8953,     "MSM8953"},
    {PLAT_MSM8976,     "MSM8976"},
    {PLAT_MSM8976PRO,  "MSM8976PRO"},
    {PLAT_SDM660,      "SDM660"},
    {PLAT_MT6750,      "MT6750"},
    {PLAT_MT6771,      "MT6771"},
    {PLAT_MT6779,      "MT6779"},
    {PLAT_SDM845,      "SDM845"},
    {PLAT_SDM450,      "SDM450"},
    {PLAT_SDM670,      "SDM670"},
    {PLAT_SDM710,      "SDM710"},
    {PLAT_SDM712,      "SDM712"},
    {PLAT_SDM855,      "SDM855"},
    {PLAT_SDM7150,      "SDM7150"},
    {PLAT_SDM6125,      "SDM6125"},
    {PLAT_SDM7250,      "SDM7250"},
    {PLAT_MT6885,      "MT6885"},
    {PLAT_SDM8250,      "SDM8250"},
    {PLAT_MT6873,      "MT6873"},
    {PLAT_SDM7125,      "SDM7125"},
    {PLAT_SDM7225,      "SDM7225"},
    {PLAT_MT6889,      "MT6889"},
    {PLAT_SDM8350,      "SDM8350"},
    {PLAT_MT6893,      "MT6893"},
    {PLAT_MT6853,       "MT6853"},
    {PLAT_SDM6115,      "SDM6115"},
    {PLAT_MT6833,       "MT6833"},
    {PLAT_SDM4350,      "SDM4350"},
    {PLAT_MT6877,      "MT6877"},
    {PLAT_SDM8450,     "SDM8450"},
    {PLAT_SDM6225,     "SDM6225"},
    {PLAT_MT6983,       "MT6983"},
    {PLAT_UNKNOWN,     "UNKNOWN"},
};

void PlatformAdaptor::getMtkChipVersion(const std::string& path, mtkChipInfo& mtkinfo) {
    FILE *chip_info;
    if ( (chip_info = fopen(path.c_str(), "rb")) ) {
        size_t s = sizeof(mtkChipInfo);
        if (s == fread(&mtkinfo, s, 1, chip_info)) {
            INFO("hw_code 1 0x%x  0x%x V1 node\n", mtkinfo.hw_code, mtkinfo.hw_subcode);
        }
        fclose(chip_info);
    }
}

PlatformAdaptor::PlatformAdaptor(): mPlatform(PLAT_UNKNOWN) {
    string id;
    int socId;
    int rawId;

    mIsQcom = isFileExist(PROC_CHIP_INFO_V1)? false : true;

    mtkChipInfo mtkinfo;
    getMtkChipVersion(PROC_CHIP_INFO_V1, mtkinfo);

    if(isQcom()){
        id = getFile(SOC_ID_PATH);
    }else{
        char hwcode[25];
        sprintf(hwcode,"%x",mtkinfo.hw_code);
        id = hwcode;
    }
    if (id == "") {
        return;
    }

    socId = strtol(id.c_str(), NULL, isQcom() ? 10:16);

    if(isQcom()){
        id = getFile(RAW_ID_PATH);
    }else{
        char hwsubcode[25];
        sprintf(hwsubcode,"%x",mtkinfo.hw_subcode);
        id = hwsubcode;
    }
    if (id == "") {
        return;
    }

    rawId = strtol(id.c_str(), NULL, isQcom() ? 10:16);

    INFO("Soc id:%d, Sub soc id:%d\n", socId, rawId);

    switch (socId) {
    case 293:
    case 304:
        mPlatform = PLAT_MSM8953;
        break;

    case 266:
    case 274:
    case 277:
    case 278:
        if (rawId == 2)
            mPlatform = PLAT_MSM8976PRO;
        else
            mPlatform = PLAT_MSM8976;
        break;
    case 317:
        mPlatform = PLAT_SDM660;
        break;
    case 336:
        mPlatform= PLAT_SDM670;
        break;
    case 338:
        mPlatform = PLAT_SDM450;
        break;
    case 360:
        mPlatform = PLAT_SDM710;
        break;
    case 393:
        mPlatform = PLAT_SDM712;
        break;
    case 394:
        mPlatform = PLAT_SDM6125;
        break;
    case 806:
        mPlatform = PLAT_MT6750;
        break;
    case 1928:
        mPlatform = PLAT_MT6771;
        break;
    case 1829:
        mPlatform = PLAT_MT6779;
        break;
    case 321:
        mPlatform = PLAT_SDM845;
        break;
    case 339:
        mPlatform = PLAT_SDM855;
        break;
    case 365:
        mPlatform = PLAT_SDM7150;
        break;
    case 400:
        mPlatform = PLAT_SDM7250;
        break;
    case 459:
        mPlatform = PLAT_SDM7225;
        break;
    case 356:
        mPlatform = PLAT_SDM8250;
        break;
    case 2182:
        mPlatform = PLAT_MT6873;
        break;
    case 2070:
        mPlatform = PLAT_MT6889;
        break;
    case 443:
    case 999:
        mPlatform = PLAT_SDM7125;
        break;
    case 415:
        mPlatform = PLAT_SDM8350;
        break;
    case 457:
        mPlatform = PLAT_SDM8450;
        break;
    case 2311:
        mPlatform = PLAT_MT6983;
        break;
    case 2384:
        mPlatform = PLAT_MT6893;
        break;
    case 2454:
        mPlatform = PLAT_MT6853;
        break;
    case 444:
        mPlatform = PLAT_SDM6115;
        break;
    case 2441:
        mPlatform = PLAT_MT6833;
        break;
    case 454:
        mPlatform = PLAT_SDM4350;
        break;
    case 2393:
        mPlatform = PLAT_MT6877;
        break;
    case 518:
        mPlatform = PLAT_SDM6225;
        break;
    default:
        mPlatform = PLAT_UNKNOWN;
        break;
    }

    INFO("Platform is %s\n", getPlatformName().c_str());

    mProject = OplusProjectService::getInstance().getProjectString();
    if (mProject == "")
        return;

    INFO("Project is %s\n", getProject().c_str());

    readPlatformCpufreqInfo();

    readPlatformGpuInfo();
}

void PlatformAdaptor::readPlatformCpufreqInfo() {
    memset(&mCpuInfo, 0, sizeof(mCpuInfo));

    vector<string> policyDir;
    if (getFileList(CPUFREQ_BASE_DIR, policyDir) < 0) {
        ERROR("Failed to load cpufreq info");
        return;
    }

    int cluster = 0;

    sort(policyDir.begin(), policyDir.end());
    // getFileList return descent order
    for (vector<string>::iterator it = policyDir.begin(); it < policyDir.end(); it++) {
            {
                std::string temp = *it + "/scaling_available_frequencies";
                //*it += "/scaling_available_frequencies";

                DEBUG("Read %s", temp.c_str());
                string result = getFile(temp);
                if (result.empty()) {
                    continue;
                }
                parseCpufreqInfo(result, cluster);
            }
            {
                std::string temp = *it + "/related_cpus";
                //*it += "/related_cpus";
                DEBUG("Read %s", temp.c_str());
                string result = getFile(temp);
                if(result.empty()) {
                    continue;
                }
                parseCoresPerCluster(result, cluster);
            }

            cluster++;
    }

    mCpuInfo.cpuCluster = cluster;

    dumpCpufreqInfo();
}

void PlatformAdaptor::readPlatformGpuInfo() {
    memset(&mGpuInfo, 0, sizeof(GpuInfo));
    switch (mPlatform) {
        case PLAT_SDM6115:
        case PLAT_SDM7125:
        case PLAT_SDM7225:
        case PLAT_SDM8250:
        case PLAT_SDM7250:
        case PLAT_SDM8350:
        case PLAT_SDM8450:
        case PLAT_SDM6225:
        case PLAT_SDM4350: {
            const std::string path = "/sys/class/kgsl/kgsl-3d0/gpubusy";
            const std::string pathBetter = "/sys/class/kgsl/kgsl-3d0/devfreq/gpu_load";
            string result = getFile(pathBetter);
            if (!result.empty()) {
                mGpuInfo.gpu_load = pathBetter;
                mGpuInfo.faster_way = true;
            } else {
                mGpuInfo.gpu_load = path;
                mGpuInfo.faster_way = false;
            }
        } break;
        case PLAT_MT6833:
        case PLAT_MT6885:
        case PLAT_MT6889:
        case PLAT_MT6877:
        case PLAT_MT6983:
        case PLAT_MT6893: {
            const std::string path = "/sys/kernel/ged/hal/gpu_utilization";
            string result = getFile(path);
            if (!result.empty()) {
                mGpuInfo.gpu_load = path;
            } else {
                mGpuInfo.gpu_load = path;
            }
            mGpuInfo.faster_way = false;
        } break;
    }

}

void PlatformAdaptor::dumpCpufreqInfo() {
    DEBUG("----------- CPU Freq Info -----------");

    for (int i = 0; i < mCpuInfo.cpuCluster; i++) {
        DEBUG("cluster %d, table length %d", i, mCpuInfo.cpuFreqInfo[i].size());
        //while (mCpuInfo.cpuFreqInfo[i][j] > 0) {
        //    DEBUG("  %lu ", mCpuInfo.cpuFreqInfo[i][j]);
        //    j++;
        //}
        for (int j = 0; j < mCpuInfo.cpuFreqInfo[i].size(); j++) {
            DEBUG("  %lu ", mCpuInfo.cpuFreqInfo[i][j]);
        }
    }
}

int PlatformAdaptor::getClusterNum() {
    return mCpuInfo.cpuCluster;
}

void PlatformAdaptor::parseCpufreqInfo(string &freq, int cluster) {
    char freqRaw[MAX_BUFFER_LEN];
    strcpy(freqRaw, freq.c_str());

    vector<uint64_t> originalFreq;
    DEBUG("Input freq: %s", freqRaw);

    char *nextFreq = strtok(freqRaw, " ");
    while (nextFreq != NULL) {
        uint64_t f = atoi(nextFreq);
        DEBUG("%lu", f);
        if (f > 0) {
            originalFreq.push_back((f * 1000)); //convert kHz to Hz
        }
        nextFreq = strtok(NULL, " ");
    }
    DEBUG("sort begin");
    DEBUG("cluster: %d", cluster);
    for(auto& i : originalFreq) {
        DEBUG("table: %lu", i);
    }
    // sort freq list to ascending order
    sort(originalFreq.begin(), originalFreq.end());
    mCpuInfo.cpuFreqInfo.push_back(originalFreq);
}

void PlatformAdaptor::parseCoresPerCluster(string &cores, int cluster) {
    char CoreRaw[MAX_BUFFER_LEN];
    strcpy(CoreRaw, cores.c_str());

    vector<int> originalCore;
    DEBUG("Input cores: %s", CoreRaw);

    char *nextFreq = strtok(CoreRaw, " ");
    while (nextFreq != NULL) {
        int f = atoi(nextFreq);

        if (f >= 0)
            originalCore.push_back(f);
        nextFreq = strtok(NULL, " ");
    }

    // sort freq list to ascending order
    sort(originalCore.begin(), originalCore.end());

    mCpuInfo.coresInCluster.push_back(originalCore);
}

const PlatformAdaptor::CpuInfo PlatformAdaptor::getCpuInfo() {
    return mCpuInfo;
}

const PlatformAdaptor::GpuInfo PlatformAdaptor::getGpuInfo() {
    return mGpuInfo;
}
