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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <strings.h>

#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>

#include "OIface.h"
#include "AffinityService.h"
#include "PlatformAdaptor.h"
#include "Utils.h"

using namespace android;

#define CPUSET_OIFACE_FG                "/dev/cpuset/oiface_fg"
#define CPUSET_OIFACE_BG                "/dev/cpuset/oiface_bg"
#define CPUSET_OIFACE_FG_CPUS           "/dev/cpuset/oiface_fg/cpus"
#define CPUSET_OIFACE_BG_CPUS           "/dev/cpuset/oiface_bg/cpus"
#define CPUSET_OIFACE_FG_MEM            "/dev/cpuset/oiface_fg/mems"
#define CPUSET_OIFACE_BG_MEM            "/dev/cpuset/oiface_bg/mems"
#define CPUSET_OIFACE_FG_TASKS          "/dev/cpuset/oiface_fg/tasks"
#define CPUSET_OIFACE_FG_CGROUP_PROCS   "/dev/cpuset/oiface_fg/cgroup.procs"
#define CPUSET_OIFACE_BG_TASKS          "/dev/cpuset/oiface_bg/tasks"
#define CPUSET_OIFACE_BG_CGROUP_PROCS   "/dev/cpuset/oiface_bg/cgroup.procs"
#define CPUSET_OIFACE_FG_PLUS           "/dev/cpuset/oiface_fg+"
#define CPUSET_OIFACE_FG_PLUS_CPUS      "/dev/cpuset/oiface_fg+/cpus"
#define CPUSET_OIFACE_FG_PLUS_MEM       "/dev/cpuset/oiface_fg+/mems"
#define CPUSET_OIFACE_FG_PLUS_TASKS     "/dev/cpuset/oiface_fg+/tasks"
#define CPUSET_ALL_TASKS                "/dev/cpuset/tasks"
#define CPUSET_BG_TASKS                 "/dev/cpuset/background/tasks"

#define CPUSET_OIFACE_FG_PLUS_CGROUP_PROCS   "/dev/cpuset/oiface_fg+/cgroup.procs"
#define CPUSET_ALL_PROCS                "/dev/cpuset/cgroup.procs"

#define CPUPOLICY_OIFACE                   "/sys/devices/system/cpu/cpufreq/"
#define CPUPOLICY_OIFACE_TARGET_LOADS      "/schedutil/target_loads"
#define CPUPOLICY_OIFACE_UP_TIME           "/schedutil/up_rate_limit_us"
#define CPUPOLICY_OIFACE_DOWN_TIME         "/schedutil/down_rate_limit_us"
ANDROID_SINGLETON_STATIC_INSTANCE(AffinityService);

uint32_t AffinityService::sClusterMap[PLAT_NUM][CLUSTER_NUM] = {
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MSM8953*/
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MSM8976*/
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MSM8976PRO */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_SDM660 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MT6750 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MT6771 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_MT6779 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_SDM845 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_SDM450 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM670 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM710 */
    { 0xff, 0x70, 0x0f, 0x80 }, /* PLAT_SDM855 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM7150 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_SDM6125 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM712 */
    { 0xff, 0xc0, 0x3f, 0x0 }, /* PLAT_SDM7250 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MT6885 */
    { 0xff, 0x70, 0x0f, 0x80  }, /* PLAT_SDM8250 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MT6873 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM7125 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM7225 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MT6889 */
    { 0xff, 0x70, 0x0f, 0x80  }, /* PLAT_SDM8350 */
    { 0xff, 0x70, 0x0f, 0x80  }, /* PLAT_MT6893 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_MT6853 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_SDM6115 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_MT6833 */
    { 0xff, 0xc0, 0x3f, 0x0  }, /* PLAT_SDM4350 */
    { 0xff, 0xc0, 0x3f, 0x0 }, /* PLAT_MT6877 */
    { 0xff, 0x70, 0x0f, 0x80  }, /* PLAT_SDM8450 */
    { 0xff, 0xf0, 0x0f, 0x0  }, /* PLAT_MSM8953*/
    { 0xff, 0x70, 0x0f, 0x80  }, /* PLAT_MT6983 */
};

AffinityService::AffinityService(): mUseCpuset(false) {
    mPlatform = PlatformAdaptor::getInstance().getPlatform();

    // bind task enable feature default on
    DEBUG("Default enable mTGPABindTaskEnable");
    mTGPABindTaskEnable = true;

    if (mkdir(CPUSET_OIFACE_FG, S_IRWXU) < 0) {
        if (errno != EEXIST) {
            ERROR("create %s failed(%s)", CPUSET_OIFACE_FG, strerror(errno));
            return;
        }
    }

    if (writeFile(CPUSET_OIFACE_FG_MEM, "0") < 0) {
        ERROR("write cpuset fg mem failed.");
        return;
    }

    if (writeFile(CPUSET_OIFACE_FG_CPUS, bitmaskToString(sClusterMap[mPlatform][CLUSTER_BIG]).c_str()) < 0) {
        ERROR("write cpuset fg cpus failed.");
        return;
    }

    if (mkdir(CPUSET_OIFACE_BG, S_IRWXU) < 0) {
        if (errno != EEXIST) {
            ERROR("create %s failed(%s)", CPUSET_OIFACE_BG, strerror(errno));
            return;
        }
    }
    if (writeFile(CPUSET_OIFACE_BG_MEM, "0") < 0) {
        ERROR("write cpuset bg mem failed.");
        return;
    }
    if (writeFile(CPUSET_OIFACE_BG_CPUS, bitmaskToString(sClusterMap[mPlatform][CLUSTER_LITTLE]).c_str()) < 0) {
        ERROR("write cpuset bg cpus failed.");
        return;
    }

    // settings for cpu big plus cores
    if (sClusterMap[mPlatform][CLUSTER_BIG_PLUS] != 0) {
        if (mkdir(CPUSET_OIFACE_FG_PLUS, S_IRWXU) < 0) {
            if (errno != EEXIST) {
                ERROR("create %s failed(%s)", CPUSET_OIFACE_FG_PLUS, strerror(errno));
                return;
            }
        }
        if (writeFile(CPUSET_OIFACE_FG_PLUS_MEM, "0") < 0) {
            ERROR("write cpuset fg+ mem failed.");
            return;
        }
        if (writeFile(CPUSET_OIFACE_FG_PLUS_CPUS,
                bitmaskToString(sClusterMap[mPlatform][CLUSTER_BIG_PLUS]).c_str()) < 0) {
            ERROR("write cpuset fg+ cpus failed.");
            return;
        }
    }

    mUseCpuset = true;

}

int AffinityService::getCpusetInfo(int clusterType, std::string& taskPath) {
    if (clusterType < CLUSTER_ALL || clusterType >= CLUSTER_NUM)
        return -1;

    if (!mUseCpuset)
        return -1;

    switch(clusterType) {
        case CLUSTER_BIG:
            taskPath = CPUSET_OIFACE_FG_TASKS;
            break;
        case CLUSTER_LITTLE:
            taskPath = CPUSET_OIFACE_BG_TASKS;
            break;
        case CLUSTER_BIG_PLUS:
            taskPath = CPUSET_OIFACE_FG_PLUS_TASKS;
            break;
        default:
            taskPath = CPUSET_ALL_TASKS;
            break;
    }

    return 0;
}

int AffinityService::getCpusetCgroupProcs(int clusterType, std::string& filePpath) {
    if (clusterType < CLUSTER_ALL || clusterType >= CLUSTER_NUM)
        return -1;

    if (!mUseCpuset)
        return -1;

    switch(clusterType) {
        case CLUSTER_BIG:
            filePpath = CPUSET_OIFACE_FG_CGROUP_PROCS;
            break;
        case CLUSTER_LITTLE:
            filePpath = CPUSET_OIFACE_BG_CGROUP_PROCS;
            break;
        case CLUSTER_BIG_PLUS:
            filePpath = CPUSET_OIFACE_FG_PLUS_TASKS;
            break;
        default:
            filePpath = CPUSET_ALL_PROCS;
            break;
    }

    return 0;
}

int AffinityService::getSchedAffinity(pid_t pid, int cluster_type) {
    /* Todo */
    return 0;
}

int AffinityService::setSchedAffinity(pid_t pid, int cluster_type) {
    cpu_set_t set;
    uint32_t mask = 0;

    switch(cluster_type) {
        case CLUSTER_BIG_PLUS:
            mask |= sClusterMap[mPlatform][CLUSTER_BIG_PLUS];
            break;
        case CLUSTER_BIG:
            mask |= sClusterMap[mPlatform][CLUSTER_BIG];
            break;
        case CLUSTER_LITTLE:
            mask |= sClusterMap[mPlatform][CLUSTER_LITTLE];
            break;
        default:
            mask |= sClusterMap[mPlatform][CLUSTER_ALL];
            break;
    }

    DEBUG("setSchedAffinity mask 0x%x", mask);

    bitmaskToCpuset(mask, &set);

    if (sched_setaffinity(pid, sizeof(set), &set) < 0) {
        ERROR("sched_setaffinity failed\n");
        return -1;
    }

    return 0;
}

int AffinityService::setAffinityCpuset(int cluster_bit, bool isTop) {
    if (cluster_bit < 0x1 || cluster_bit > 0x0e) {
        ERROR("cluster type error");
        return -1;
    }
    string set = getSpecifiedCpuset(cluster_bit);
    ERROR("set affinity cpuset: %s ", set.c_str());
    writeFile(CPUSET_FG_CPUS, set);

    return 0;
}
int AffinityService::setSchedAffinityByCpu(pid_t pid,int cpu_flag) {
    cpu_set_t cpuSet;
    if (cpu_flag < 0 || cpu_flag > 0xff+10) {
        ERROR("cluster cpu num error");
        return -1;
    }
    DEBUG("adding %d to set..", cpu_flag);
    CPU_ZERO(&cpuSet);
    bitmaskToCpuset((cpu_flag-CPUTYPE_MAX_NUM) & 0xff,&cpuSet);
    sched_setaffinity(pid, sizeof(cpuSet), &cpuSet);
    return 0;
}


/* clearAffinity - move all the task in cgroup to root cgroup */
void AffinityService::clearAffinityForSingleThread(pid_t pid, int cluster_type, int delay_ms) {
    std::this_thread::sleep_for (std::chrono::milliseconds(delay_ms));

    if (AffinityService::getInstance().setSchedAffinity(pid, 0) < 0) {
        ERROR("clearAffinityForSingleThread: clear sched affinity failed");
        return;
    }
}

void AffinityService::setAffinityForSingleThread(pid_t pid, int cluster_type) {

    DEBUG("Tring to set tid %d to cluster %d", pid, cluster_type);
    if (AffinityService::getInstance().setSchedAffinity(pid, cluster_type) < 0) {
        ERROR("setAffinityForSingleThreadTimeout %d to cluster %d failed", pid, cluster_type);
        return;
    }

    DEBUG("setAffinityForSingleThread %d success", pid);
}

void AffinityService::setAffinityForSingleThreadTimeout(pid_t pid, int cluster_type, int timeout) {

    DEBUG("Tring to set tid %d to cluster %d timeout %d", pid, cluster_type, timeout);
    if (AffinityService::getInstance().setSchedAffinity(pid, cluster_type) < 0) {
        ERROR("setAffinityForSingleThreadTimeout %d to cluster %d failed", pid, cluster_type);
        return;
    }

    DEBUG("setAffinityForSingleThreadTimeout %d to cluster %d success", pid, cluster_type);

    /* Due to power concern, clear task affinity after timeout */
    std::thread(clearAffinityForSingleThread, pid, cluster_type, timeout).detach();
}

int AffinityService::handleTimeout(int32_t *data, int) {
    cpu_set_t set;
    int pid;

    if (data == NULL) {
        ERROR("invalid argument");
        return -1;
    }

    pid = data[0];

    if (getCpuset(mPlatform, CLUSTER_ALL, &set) < 0) {
        ERROR("get cpu set failed\n");
        return -1;
    }

    if (mUseCpuset) {
        if (writeFile(CPUSET_ALL_TASKS, to_string(pid)) < 0) {
            ERROR("set affinity by cpuset failed");
            return -1;
        }
    } else {
        if (sched_setaffinity(pid, sizeof(set), &set) < 0) {
            ERROR("set affinity failed(%s)", strerror(errno));
            return -1;
        }
    }

    return 0;
}

int AffinityService::affineTask(pid_t pid, int32_t type, int timeoutMs) {
    cpu_set_t set;

    if ((pid <= 0) || (type >= CLUSTER_NUM) || (type < 0))
        return -1;

    DEBUG("start to affine:%d cluster:%d timeout:%d", pid, type, timeoutMs);

    if (timeoutMs <= 0) {
        if (cancelJob("affine_" + to_string(pid)) < 0) {
            ERROR("cancel job failed");
            return -1;
        }
        return 0;
    }

    if (getCpuset(mPlatform, type, &set) < 0) {
        ERROR("get cpu set failed\n");
        return -1;
    }

    /* set affinity first */
    if (mUseCpuset) {
        std::string path;
        if (getCpusetInfo(type, path) < 0)
            return -1;

        if (writeFile(path, to_string(pid)) < 0) {
            ERROR("set affinity by cpuset failed");
            return -1;
        }
    } else {
        if (sched_setaffinity(pid, sizeof(set), &set) < 0) {
            ERROR("set affinity failed(%s)", strerror(errno));
            return -1;
        }
    }

    TimedJob::Job job;
    job.timeout = timeoutMs * 1000000LL;
    job.name = "affine_"  + to_string(pid);
    job.instance = this;
    job.data[0] = pid;
    job.data[1] = type;

    if (addJob(job) < 0) {
        ERROR("add new job failed");
        return -1;
    }

    DEBUG("set affinity for pid %d done, timeout:%d", pid, timeoutMs);

    return 0;
}

int AffinityService::getCpuset(int platform, int type, cpu_set_t *set) {
    if ((platform >= PLAT_NUM) || (platform < 0))
        return -1;

    if ((type >= CLUSTER_NUM) || (type < 0))
        return -1;

    bitmaskToCpuset(sClusterMap[platform][type], set);

    return 0;
}

void AffinityService::bitmaskToCpuset(uint32_t bitmask, cpu_set_t *set) {
    int i = 0;

    CPU_ZERO(set);

    while (bitmask) {
        if (bitmask & 0x1) {
            DEBUG("adding %d to set..", i);
            CPU_SET(i, set);
        }

        i++;
        bitmask >>= 1;
    }
};

void AffinityService::setTGPABindTaskEnable(int32_t enable) {
    if (enable == 1)
        mTGPABindTaskEnable = true;
    else {
        DEBUG("Disable mTGPABindTaskEnable");
        mTGPABindTaskEnable = false;
    }
}

string AffinityService::bitmaskToString(uint32_t bitmask) {
    int i = 0;
    string set;

    while (bitmask) {
        if (bitmask & 0x1) {
            DEBUG("adding %d to set..", i);
            set += to_string(i) + ",";
        }

        i++;
        bitmask >>= 1;
    }

    return set;
}

map<int, int> AffinityService::getClusterLeadingCpu() {
    map<int, int> cpuMap;
    int bit;

    if ((mPlatform < 0) || (mPlatform >= PLAT_NUM))
        return cpuMap;

    if ((bit = ffs(sClusterMap[mPlatform][CLUSTER_BIG])))
        cpuMap[CLUSTER_BIG] = bit - 1;

    if ((bit = ffs(sClusterMap[mPlatform][CLUSTER_LITTLE])))
        cpuMap[CLUSTER_LITTLE] = bit - 1;

    if ((bit = ffs(sClusterMap[mPlatform][CLUSTER_BIG_PLUS])))
        cpuMap[CLUSTER_BIG_PLUS] = bit - 1;

    return cpuMap;
}

map<int, vector<int> > AffinityService::getClusterFreq() {
    map<int, vector<int> > cpuMap;
    int cluster_num = 0;
    char path[128];
    for (int i=0; i<8; i++){
        snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpufreq/policy%d/scaling_available_frequencies", i);
        if (access(path, F_OK)==0) {
            int fd=open(path, O_RDONLY);
            if (fd < 0) {
                ERROR("open %s failed(%s)\n", path, strerror(errno));
            } else {
                char buf[512] = {0,};
                int n=read(fd, buf, (size_t)(sizeof(buf) - 1));
                //DEBUG("reading %d policy%d available_frequencies:%s", n, i, buf);

                char *saveptr, *str, *token;
                int j=0;
                for (j = 0, str = buf; ;j++, str = NULL) {
                    token = strtok_r(str, " \t\n", &saveptr);
                    if (token == NULL)
                        break;
                    int freq=atoi(token);
                    if (freq > 0) {
                        cpuMap[cluster_num].push_back(freq);
                    }
                }
                sort(cpuMap[cluster_num].begin(), cpuMap[cluster_num].end());
                close(fd);
                cluster_num++;
            }
        }
    }
    return cpuMap;
}

string AffinityService::getSpecifiedCpuset(int cluster_bit) {
    string set;

    for (int i = 0; i < 4 && cluster_bit; i++) {
        if (cluster_bit & 0x1) {
            DEBUG("get cluster %d", i);
            set += bitmaskToString(sClusterMap[mPlatform][i]);
        }
        cluster_bit >>= 1;
    }

    return set;
}

void AffinityService::readPathFile(std::string path, std::string &result) {
    if (path.empty()) {
        ERROR("readPathFile:path is empty");
        return;
    }
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        ERROR("readPathFile:open path %s failed, %s", path.c_str(), strerror(errno));
        return;
    }
    char buff[1024] = {0, };
    int k = read(fd, buff, (size_t)(sizeof(buff)-1));
    if (k >= 0) {
        result = buff;
    }
    close(fd);

    DEBUG("readPathFile:read information: %s", result.c_str());
    return;
}

int AffinityService::getCpuPolicy(std::vector<std::string> &res, std::string path) {
    if (path.empty()) {
        ERROR("getCpuPolicy:path is empty");
        return -1;
    }
    string path_mid = "policy";
    vector<int> mid;
    getClusterLeadingCpuNo(mid);
    for (int i = 0; i < mid.size(); i++) {
        res.push_back(path_mid+to_string(mid[i]));
        DEBUG("getCpuPolicy:policy = %s", res[i].c_str());
    }
    DEBUG("AffinityService::getCpuPolicy finished");
    return 0;
}

int AffinityService::getCpuAllPolicyPath(std::vector<std::vector<std::string> > &allPolicyPath) {
    allPolicyPath.resize(3);                                     //target_load , up_time , down_time
    vector<string> mid;
    if (getCpuPolicy(mid) < 0) {
        ERROR("AffinityService::getCpuAllPolicyPath: getCpuPolicy error");
        return -1;
    }
        for (int i = 0;i < mid.size();i++) {
            string target_load = CPUPOLICY_OIFACE+ mid[i] +CPUPOLICY_OIFACE_TARGET_LOADS;
            string up_rate_time = CPUPOLICY_OIFACE+ mid[i] +CPUPOLICY_OIFACE_UP_TIME;
            string down_rate_time = CPUPOLICY_OIFACE+ mid[i] +CPUPOLICY_OIFACE_DOWN_TIME;

            allPolicyPath[0].push_back(target_load);
            allPolicyPath[1].push_back(up_rate_time);
            allPolicyPath[2].push_back(down_rate_time);
            DEBUG("AffinityService::getCpuAllPolicyPath :target_load is  %s", target_load.c_str());
            DEBUG("AffinityService::getCpuAllPolicyPath :up_rate_time is  %s", up_rate_time.c_str());
            DEBUG("AffinityService::getCpuAllPolicyPath :down_rate_time is  %s", down_rate_time.c_str());
        }
    DEBUG("AffinityService::getCpuAllPolicyPath finished");
    return 0;
}

int AffinityService::setAllPolicy(int cluster, string paraLoad, string paraUpTime, string paraDownTime) {
    if (!cluster) {
        ERROR("it's wrong with cluster smaller than 0");
        return -1;
    }
    std::vector<std::string> pString;
    if (getCpuPolicy(pString) != -1) {
        for (int i = 0; i < pString.size(); i++) {
            string target_load = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_TARGET_LOADS;
            string up_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_UP_TIME;
            string down_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_DOWN_TIME;
                if (writeSystemFile(target_load, paraLoad) < 0) {
                    ERROR("write paraLoad failed");
                    return -1;
                }
                if (writeSystemFile(up_rate_time, paraUpTime) < 0) {
                    ERROR("write paraUpTime failed");
                    return -1;
                }
                if (writeSystemFile(down_rate_time, paraDownTime) < 0) {
                    ERROR("write paraDownTime failed");
                    return -1;
                }
        }
    } else {
        ERROR("AffinityService::setAllPolicy error");
        return -1;
    }
    DEBUG("AffinityService::setAllPolicy finished");
    return 0;
}
int AffinityService::getAllPolicy(int cluster) {
    pre_load.resize(cluster);
    pre_up_time.resize(cluster);
    pre_down_time.resize(cluster);
    std::vector<std::string> pString;
    if (getCpuPolicy(pString) != -1) {
            for (int i = 0; i < cluster; i++) {
                string target_load = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_TARGET_LOADS;
                string up_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_UP_TIME;
                string down_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_DOWN_TIME;
                readPathFile(target_load, pre_load[i]);
                readPathFile(up_rate_time, pre_up_time[i]);
                readPathFile(down_rate_time, pre_down_time[i]);
                DEBUG("AffinityService::getAllPolicy :pre_load[%d] = %s", i, pre_load[i].c_str());
                DEBUG("AffinityService::getAllPolicy :pre_up_time[%d] = %s", i, pre_up_time[i].c_str());
                DEBUG("AffinityService::getAllPolicy :pre_down_time[%d] = %s", i, pre_down_time[i].c_str());
            }
    } else {
        ERROR("AffinityService::getAllPolicy error");
        return -1;
        }
    DEBUG("AffinityService::getAllPolicy finished");
    return 0;
}
void AffinityService::writeTime(int delay_ms, string path, string res) {
    DEBUG("AffinityService::writeTime ------------------path = %s,res = %s ", path.c_str(), res.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    if (!path.empty()&&!res.empty()) {
        DEBUG("AffinityService::writeTime : it will write ---path = %s,res = %s ", path.c_str(), res.c_str());
            if (writeSystemFile(path, res) < 0) {
                ERROR("AffinityService::writeTime :write failed");
                return;
            }
    }
    else {
        ERROR("AffinityService::writeTime : path.empty() or res.empty()");
    }
}

int AffinityService::resetAllPolicy(int cluster, std::vector< std::vector<bool>> &ans, int delayTime) {
    if (cluster < 0) {
        return -1;
    }
    std::vector<std::string> pString;
    if (getCpuPolicy(pString) != -1) {
        for (int i = 0;i < cluster;i++) {
            string target_load = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_TARGET_LOADS;
            string up_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_UP_TIME;
            string down_rate_time = CPUPOLICY_OIFACE+ pString[i] +CPUPOLICY_OIFACE_DOWN_TIME;
            DEBUG("AffinityService::resetAllPolicy-------------------------------- ");
            DEBUG("AffinityService::resetAllPolicy target_load = %s", target_load.c_str());
            DEBUG("AffinityService::resetAllPolicy up_rate_time = %s", up_rate_time.c_str());
            DEBUG("AffinityService::resetAllPolicy down_rate_time = %s", down_rate_time.c_str());
            DEBUG("AffinityService::resetAllPolicy pre_load[%d] = %s", i, pre_load[i].c_str());
            DEBUG("AffinityService::resetAllPolicy pre_up_time[%d] = %s", i, pre_up_time[i].c_str());
            DEBUG("AffinityService::resetAllPolicy down_rate_time[%d] = %s", i, pre_down_time[i].c_str());
            DEBUG("AffinityService::resetAllPolicy-------------------------------- ");
            if (!pre_load[i].empty()&&ans[0][i]) {
                DEBUG("AffinityService::resetAllPolicy pre_load[%d] = %s", i, pre_load[i].c_str());
                std::thread(writeTime, delayTime, target_load, pre_load[i]).detach();
                ans[0][i] = false;
            }
            if (!pre_up_time[i].empty()&&ans[1][i]) {
                DEBUG("AffinityService::resetAllPolicy pre_up_time[%d] = %s", i, pre_up_time[i].c_str());
                std::thread(writeTime, delayTime, up_rate_time, pre_up_time[i]).detach();
                ans[1][i] = false;
            }
            if (!pre_down_time[i].empty()&&ans[2][i]) {
                DEBUG("AffinityService::resetAllPolicy down_rate_time[%d] = %s", i, pre_down_time[i].c_str());
                std::thread(writeTime, delayTime, down_rate_time, pre_down_time[i]).detach();
                ans[2][i] = false;
            }
        }
    } else {
        ERROR("AffinityService::resetAllPolicy error");
            return -1;
    }
    pre_load.clear();
    pre_up_time.clear();
    pre_down_time.clear();
    DEBUG("AffinityService::resetAllPolicy finished");
    return 0;
}
