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

#ifndef __UTILS_H__
#define __UTILS_H__
#include <map>
#include <vector>
#include <string>
#include "OIface.h"
#include "OIfaceModule.h"
#include "OIfaceServer.h"
#include "util/OrmsProxy.h"

using namespace android;

#define SYS_USB            "/sys/class/power_supply/usb/online"
#define SYS_PC_PORT        "/sys/class/power_supply/pc_port/online"
#define SYS_CAPACITY       "/sys/class/power_supply/battery/capacity"
#define SYS_BAT_REMAIN     "/sys/class/power_supply/battery/batt_rm"
#define SYS_BAT_FCC        "/sys/class/power_supply/battery/batt_fcc"
#define SYS_BRIGHTNESS     "/sys/class/leds/lcd-backlight/brightness"
#define SYS_CPUTEMP        "/sys/class/thermal/thermal_zone10/temp"

#define CPUSET_FG_CPUS                 "/dev/cpuset/foreground/cpus"
#define CPUSET_RESTRICTED_CPUS         "/dev/cpuset/restricted/cpus"
#define CPUSET_TOP_CPUS                "/dev/cpuset/top-app/cpus"
#define CPUSET_BACKGROUND              "/dev/cpuset/system-background/cpus"
#define CPUTYPE_MAX_NUM 10
int getPackageForUid(int uid, std::vector<std::string>& package);
bool isAppDebuggable(android::String16 packageName);
void hypnusBroadcast(android::String16 package);
void sendBroadcast(android::String16 action, android::String16 packageName,
    android::String16 componentName, android::String16 permission, int intentExtrasCounts,
    /*Intent extras*/...);
int makeDecision(const std::string& name, int type, ...);
std::map<std::string, int> jsonToMap(const Json::Value& value);
int32_t getGlobalFrames();
int64_t getStartTime();
int32_t getBrightness();
int32_t getSoundLevel();
int32_t getHeadsetState();
int32_t getLmhCnt(const char *path);
int getUidByPid(pid_t pid);
int getTimeInState(const char *path, std::map<int32_t, int64_t> &timeInState);
int getCpuTimeInState(int cpu, std::map<int32_t, int64_t> &timeInState);
void timeInStateToAverageFreq(map<int32_t, int64_t> &timeInState);
int64_t getCpuIdleTime(int cpu);
int getGpuStats(std::map<int32_t, int32_t> &stat);
/*Date operations*/
bool isEvenDay();
/* File operations are all here*/
bool isFileExist(const char *file_path);
bool isVendorFile(const std::string& path);
std::string getFile(const std::string& path);
std::string getVendorFile(const std::string& path);
int getVendorFileList(const char *p, std::vector<std::string>& str);
std::string getSystemFile(const std::string& path);
int writeFile(const std::string& path, const std::string& str);
int writeVendorFile(const std::string& path, const std::string& str);
int writeSystemFile(const std::string& path, const std::string& str);
int getFileList(const char *p, std::vector<std::string>& str);
/* scan forground app by default */
std::vector<int> getPidByName(const char* task_name, int gamePid = 0, bool scan_all = 0);
float getCpuUsage(int type, int uid = -1, int pid = -1);
int getUidByPid(pid_t pid);
int getAllProcessCpuTime(std::map<int, float> &cpuTime, std::map<int, std::string> &pidComm);
std::string getCpuLoad();
std::string getCpuLoadFromProcStat();
std::string getGpuLoad();
std::string getGpuFreq();
std::string getHashByUID(int uid);
bool isSpecialPermited(int uid);
int getConfigTaskList(int index, const std::string& package_name, std::vector<int>& taskList);
int getTasks(int pid, std::vector<struct boost_task>& tasks);
std::string getTaskNameByTid(int pid, int tid);
void splitString(const string& s, vector<string>& v, const string& c);
void getClusterLeadingCpuNo(std::vector<int>& leading_cpu);
void getCpuCapacity(std::vector<int>& cpu_capacity);
void getTidsByPid(int pid, std::vector<int>& tidList);
void getKeyThreadStatus(pid_t pid, const vector<int>& tidList, vector<string>& outStaticUxInfo);
char *getOpenID();
void setOpenID(std::string &id);
inline bool compareBoostTask(const struct boost_task& task1, const struct boost_task& task2) {
    return task1.duration > task2.duration;
}

inline bool operator!=(const Decision& d1, const Decision& d2) {
    if (d1.type != d2.type)
        return true;

    if (d1.type == DECISION_TYPE_ACTION) {
        return ((d1.action.action_type != d2.action.action_type) ||
                        (d1.action.timeout != d2.action.timeout));
    }

    if (d1.type == DECISION_TYPE_NETWORK) {
        return ((d1.network.power_mode != d2.network.power_mode) ||
                        (d1.network.timeout != d2.network.timeout));

    }

    return true;
}

inline bool operator== (const Decision& d1, const Decision& d2) {
    return (!(d1 != d2));
}

extern "C" {
inline int sendLog(const char *buf) {
    return OIfaceServer::getInstance().sendRemoteLog(buf);
}
}

inline int64_t getCurrentTimestamp() {
    struct timespec now;

    if (clock_gettime(CLOCK_BOOTTIME, &now) < 0) {
        ERROR("get current time failed\n");
        return -1;
    }

    return (now.tv_sec * 1000000000LL + now.tv_nsec);
}

void setSgameSignatureMD5(std::string &md5);
bool sgameSignatureMatch();

int getTimeInState(const char *path, std::map<int32_t, std::vector<int64_t>> &timeInState);
int getCpuTimeInState(int cpu, std::map<int32_t, std::vector<int64_t> > &timeInState);
vector<float> getCpuUsage(std::vector<int64_t>& preUse);

#endif
