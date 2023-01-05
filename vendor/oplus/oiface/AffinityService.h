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

#ifndef __AFFINITY_SERVICE_H__
#define __AFFINITY_SERVICE_H__

#include <utils/Singleton.h>

#include "TimedJob.h"
#include <string>
#include <vector>

class AffinityService: public TimedJob, public android::Singleton<AffinityService> {
public:
    AffinityService();
    int affineTask(pid_t pid, int32_t type, int timeoutMs);
    virtual ~AffinityService() {};
    static int getCpuset(int platform, int type, cpu_set_t *set);
    /* in predictable future, 32bit is enough */
    static uint32_t sClusterMap[PLAT_NUM][CLUSTER_NUM];
    std::map<int, int> getClusterLeadingCpu();
    int getCpusetInfo(int clusterType, std::string& cpusetPath);
    static void clearAffinityForSingleThread(pid_t pid, int cluster_type, int delay_ms);
    int getCpusetCgroupProcs(int clusterType, std::string& filePpath);
    void setAffinityForSingleThread(pid_t pid, int cluster_type);
    void setAffinityForSingleThreadTimeout(pid_t pid, int cluster_type, int timeout);
    int getSchedAffinity(pid_t pid, int cluster_type);
    int setSchedAffinity(pid_t pid, int cluster_type);
    void setTGPABindTaskEnable(int32_t enable);
    bool getTGPABindTaskEnable() { return mTGPABindTaskEnable; }
    int setAffinityCpuset(int cluster_bit, bool isTop);
    int setSchedAffinityByCpu(pid_t pid,int cpu_flag);
    std::map<int, std::vector<int> > getClusterFreq();

    std::vector<std::string> pre_load;
    std::vector<std::string> pre_up_time;
    std::vector<std::string> pre_down_time;

    void readPathFile(std::string path, std::string &result);
    static void writeTime(int delay_ms, std::string path, std::string res);
    int getCpuPolicy(std::vector<std::string> &res, std::string path = "/sys/devices/system/cpu/cpufreq");
    int getCpuAllPolicyPath(std::vector<std::vector<std::string> > &allPolicyPath);
    int setAllPolicy(int cluster, std::string paraLoad, std::string paraUpTime, std::string paraDownTime);
    int getAllPolicy(int cluster);
    int resetAllPolicy(int cluster, std::vector<std::vector<bool>> &ans, int delayTime = 0);



protected:
    virtual int handleTimeout(int32_t *data, int size);

private:
    static void bitmaskToCpuset(uint32_t bitmask, cpu_set_t *set);
    static std::string bitmaskToString(uint32_t bitmask);
    std::string getSpecifiedCpuset(int cluster_bit);
    int mPlatform;
    bool mUseCpuset;
    bool mTGPABindTaskEnable;
};

#endif
