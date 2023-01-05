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

#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include <Utils.h>
#include <dirent.h>
#include <string.h>
#include <fstream>
#include <streambuf>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <cutils/properties.h>
#include <algorithm>
#include <numeric>

#include "OIface.h"
#include <utils/RefBase.h>
#include <utils/Unicode.h>
#include <binder/IServiceManager.h>
#include <binder/IPermissionController.h>
#include <private/android_filesystem_config.h>
#include "OIfaceModule.h"
#include "DecisionDriver.h"
#include "ThreadState.h"
#include "GlobalConfig.h"
#include "PlatformAdaptor.h"
#include "AffinityService.h"
#include "OIfaceHalService.h"
#include "ThermalService.h"

using namespace android;
#undef OIFACE_HIDL

int getPackageForUid(int uid, std::vector<std::string>& outPackages) {
    int len;
    Vector<String16> packages;

#if !defined(PREM)
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->checkService(String16("permission"));
    if (binder == 0) {
        ERROR("Cannot get permission service");
        return -1;
    }

    sp<IPermissionController> permCtrl = interface_cast<IPermissionController>(binder);
    permCtrl->getPackagesForUid(uid, packages);
#else
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->checkService(String16("package"));
    if (binder == 0) {
        ERROR("Cannot get package service");
        return -1;
    }

    Parcel data, reply;
    int32_t size;
    data.writeInterfaceToken(String16("android.content.pm.IPackageManager"));
    data.writeInt32(uid);
    if (binder->transact(26, data, &reply) != NO_ERROR) {
        ERROR("transact to package manager failed");
        return -1;
    }

    if (reply.readExceptionCode() != 0) {
        ERROR("exception noticed from reply");
        return -1;
    }

    if (reply.readInt32(&size) != NO_ERROR) {
        ERROR("read parcel faield");
        return -1;
    }

    DEBUG("uid package number:%d", size);

    for (int i = 0; i < size; i++) {
        packages.push_back(reply.readString16());
    }

#endif

    if (packages.isEmpty()) {
        ERROR("No packages for calling UID:%d\n", uid);
        return -1;
    }

    for (Vector<String16>::iterator iter = packages.begin();
            iter != packages.end(); iter++) {

        len = utf16_to_utf8_length(iter->string(), iter->size());
        char* utf8 = (char *)malloc(len + 1);
        utf16_to_utf8(iter->string(), iter->size(), utf8, len + 1);
        utf8[len] = '\0';

        outPackages.push_back(utf8);
        free(utf8);
    }

    return packages.size();
}

#define OIFACE_DEBUGGABLE_PROPERTY "sys.oplus.oiface.debuggable"
#define TRANSACT_GET_APPLICATIONINFO 13
#define BNDL_MAGIC 0x4C444E42
#define FLAGS_DEBUGGABLE (1<<1)
#define DEBUG_SWTICH_ON true
bool isAppDebuggable(String16 pkgName) {
    if (!property_get_bool(OIFACE_DEBUGGABLE_PROPERTY, false)) {
        ERROR("App debuggable feature is off, skip check!");
        return false;
    }

    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->checkService(String16("package"));
    if (binder == 0) {
        ERROR("Cannot get package service");
        return false;
    }

    Parcel data, reply;
    data.writeInterfaceToken(String16("android.content.pm.IPackageManager"));
    data.writeString16(pkgName);
    data.writeInt32(0);//flags
    data.writeInt32(0);//uid
    if (binder->transact(TRANSACT_GET_APPLICATIONINFO, data, &reply) != NO_ERROR) {
        ERROR("transact to package manager failed");
        return false;
    }

    if (reply.readExceptionCode() != 0) {
        ERROR("exception noticed from reply");
        return false;
    }

    int result = reply.readInt32();
    if (result == 0) {
        ERROR("No ApplicationInfo object return.");
        return false;
    }

    /******** Parsing PackageItemInfo ********/
    String16 name = reply.readString16();
    String16 packageName = reply.readString16();
    int labelRes = reply.readInt32();
    /* TextUtils parsing... */
    int kind = reply.readInt32();
    String16 nonLocalizedLabel = reply.readString16();
    if (!String8(nonLocalizedLabel).isEmpty()) {
        INFO("nonLocalizedLabel is not empty");
        /* TODO : here we need to parse more informations*/
        return false;
    }
    int icon = reply.readInt32();
    int logo = reply.readInt32();
    /* metaData Bundle parsing... */
    int length = reply.readInt32();
    int magic = 0;
    if (length > 0) {
        magic = reply.readInt32();
        if (magic == BNDL_MAGIC) {
            INFO("start reading metaData Bundle!");
            /* TODO : here we need to parse more informations*/
            return false;
        }
    }
    int banner = reply.readInt32();
    int showUserIcon = reply.readInt32();

    /******** Parsing ApplicationInfo ********/
    String16 taskAffinity = reply.readString16();
    String16 permission = reply.readString16();
    String16 processName = reply.readString16();
    String16 className = reply.readString16();
    int theme = reply.readInt32();
    int flags = reply.readInt32();
    DEBUG("flags : 0x%x", flags);

    if (DEBUG_SWTICH_ON) {
        INFO("packageName : %s", String8(pkgName).string());
        INFO("name : %s", String8(name).string());
        INFO("packageName : %s", String8(packageName).string());
        INFO("labelRes    : %d", labelRes);
        INFO("kind : %d", kind);
        INFO("nonLocalizedLabel : %s", String8(nonLocalizedLabel).string());
        INFO("icon : %d", icon);
        INFO("logo : %d", logo);
        INFO("length : %d", length);
        INFO("magic  : 0x%x", magic);
        INFO("banner : %d", banner);
        INFO("showUserIcon : %d", showUserIcon);
        INFO("taskAffinity : %s", String8(taskAffinity).string());
        INFO("permission   : %s", String8(permission).string());
        INFO("processName  : %s", String8(processName).string());
        INFO("className    : %s", String8(className).string());
        INFO("theme : %d", theme);
        INFO("flags : %d", flags);
    }

    if ((flags & FLAGS_DEBUGGABLE) == 0) {
        return false;
    }

    return true;
}

#define MAX_INTENT_EXTRAS_CNTS  (20)
void sendBroadcast(String16 action, String16 packageName, String16 componentName, String16 permission,
    int intentExtrasCounts, /*Intent extras*/...) {
#if 0
    char *serviceName = "activity";
    bool useHypnus = false;

    /* Use hypnus service as a proxy for Android P */
    if (property_get_int32("ro.build.version.sdk", -1) >= 28) {
        serviceName = "hypnus";
        DEBUG("use hypnus as broadcast proxy");
        useHypnus = true;
    }
    if (intentExtrasCounts > MAX_INTENT_EXTRAS_CNTS) {
        ERROR("Cannot hold so much intent extras! The limit is %d.", MAX_INTENT_EXTRAS_CNTS/2);
        return;
    }

    if (intentExtrasCounts % 2 != 0) {
        ERROR("intentExtrasCounts should be even!");
        return;
    }

    va_list ap;
    va_start(ap, intentExtrasCounts);
    const char *extras[MAX_INTENT_EXTRAS_CNTS];
    for (int i = 0; i < intentExtrasCounts; i++) {
        extras[i] = va_arg(ap, const char *);
    }
    va_end(ap);

    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->checkService(String16(serviceName));
    if (binder == 0) {
        ERROR("Cannot get %s service", serviceName);
        return;
    }

    if (!useHypnus) {
        android::sp<IActivityServiceProxy> act = interface_cast<IActivityServiceProxy>(binder);
        if (act == NULL) {
            ERROR("%s service is not valid\n", serviceName);
            return;
        }

        act->broadcast(action, packageName, componentName, permission, intentExtrasCounts, extras);
    } else {
        android::sp<IHypnusServiceProxy> act = interface_cast<IHypnusServiceProxy>(binder);
        if (act == NULL) {
            ERROR("%s service is not valid\n", serviceName);
            return;
        }

        act->broadcast(action, packageName, componentName, permission, intentExtrasCounts, extras);
    }
#endif
    INFO("Sending broadcast %s", String8(action).c_str());
}

#define CPU_CONTROL_ARG_MAX         20
#define CPU_CONTROL_ARG_COUNT       15
#define GPU_CONTROL_ARG_COUNT       5
#define SCHEDTUNE_DECISION_ARG_COUNT    5
#define DEFAULT_DECISION_ARG_COUNT      4

int makeDecision(const std::string& name, int type, ...) {
    va_list ap;
    va_start(ap, type);

    Decision decision;

    static std::string pkgName;
    pkgName = ThreadState::getInstance().getForgroundPackage();

    decision.type = type;
    if (type == DECISION_TYPE_SCENE) {
        decision.scene.pid = 1;
        decision.scene.sceneNeedRestore = 1;
        int scene = va_arg(ap, int32_t);

        if (scene == HYPNUS_SCENE_18) {
            decision.scene.name = HYPNUS_SCENE_18_STR;
        } else if (scene == HYPNUS_SCENE_19) {
            decision.scene.name = HYPNUS_SCENE_19_STR;
        } else {
            /* FIXME: package name may be changed during making decision. */
            decision.scene.name = pkgName.c_str();
        }

    } else if (type == DECISION_TYPE_HYPNUS_CONTROL) {
        int32_t *cpu_ptr = (int32_t*)((char*)&decision.hypnus_ctl +
                offsetof(struct hypnus_control_data, cpu_data));

        int32_t *gpu_ptr = (int32_t*)((char*)&decision.hypnus_ctl +
                offsetof(struct hypnus_control_data, gpu_data));

        decision.hypnus_ctl.version = HYP_CONTROL_DATA_VERSION;
        decision.hypnus_ctl.size = sizeof(struct hypnus_control_data);
        int i = 0;
        for (; i < CPU_CONTROL_ARG_COUNT; i++) {
            cpu_ptr[i] = va_arg(ap, int32_t);
        }

        for (; i < CPU_CONTROL_ARG_MAX; i++) {
            cpu_ptr[i] = 0;
        }

        for (int j = 0; j < GPU_CONTROL_ARG_COUNT; j++) {
            gpu_ptr[j] = va_arg(ap, int32_t);
        }

        decision.hypnus_ctl.control_mask = va_arg(ap, int32_t);
        DEBUG("hypnus control mask is %d", decision.hypnus_ctl.control_mask);

    } else if (type == DECISION_TYPE_SCHEDTUNE) {
        int32_t *ptr = &(decision.idata[0]);
        // FIXME: add task has not enable now...
        // and only support setting for top-app
        for (int i = 0; i < DEFAULT_DECISION_ARG_COUNT; i++) {
            ptr[i] = va_arg(ap, int32_t);
        }
        decision.schedtune_boost.index = SCHEDTUNE_BG_INDEX_TOP_APP;
        decision.schedtune_boost.add_tasks = 0;
        decision.schedtune_boost.count = 0;
        decision.schedtune_boost.tasks = 0;
    } else {
        int32_t *ptr = &(decision.idata[0]);

        for (int i = 0; i < DEFAULT_DECISION_ARG_COUNT; i++) {
            ptr[i] = va_arg(ap, int32_t);
        }
    }

    va_end(ap);

    return DecisionDriver::getInstance().talkToDriver(decision, name);
}

std::map<std::string, int> jsonToMap(const Json::Value& value) {
    std::map<std::string, int> m;
    for (Json::ValueConstIterator iter = value.begin(); iter != value.end(); iter++) {
        if ((*iter).isNumeric()) {
            m[iter.key().asCString()] = atoi((*iter).toStyledString().c_str());
        } else if ((*iter).isString()) {
            char *endptr;
            const char *str = iter->asCString();

            errno = 0;
            long val = strtol(str, &endptr, 0);

            /* Check for various possible errors */
            if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
                    || (errno != 0 && val == 0)) {
                DEBUG("strtol failed for (%s:%s):%s", iter.key().asCString(),
                        str, value.toStyledString().c_str());
                continue;
            }

            if (endptr == str) {
                DEBUG("No digits were found\n");
                continue;
            }

            m[iter.key().asCString()] = val;
        }
    }

    return m;
}

int32_t getGlobalFrames() {
    sp<IServiceManager> sm = defaultServiceManager();
    if (sm == NULL) {
        ALOGE("Unable to get default service manager!");
        return -1;
    }

    sp<IBinder> service = sm->checkService(String16("SurfaceFlinger"));
    if (service == NULL) {
        ALOGE("unable to get surfaceflinger");
        return -1;
    }

    Parcel data, reply;
    data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));

    status_t result = service->transact(1013, data, &reply);
    if (result != NO_ERROR)
        ERROR("%s, failed to transact: %d", __func__, result);

    return reply.readInt32();
}

int32_t getSoundLevel() {
    sp<IServiceManager> sm = defaultServiceManager();
    if (sm == NULL) {
        ALOGE("Unable to get default service manager!");
        return -1;
    }

    sp<IBinder> service = sm->checkService(String16("audio"));
    if (service == NULL) {
        ALOGE("unable to get audio service");
        return -1;
    }

    Parcel data, reply;
    data.writeInterfaceToken(String16("android.media.IAudioService"));

#if(ANDROID_VERSION >= 28)
    #define GET_STREAM_VOLUME   12
#else
    #define GET_STREAM_VOLUME   8
#endif

#define STREAM_MUSIC        3

    data.writeInt32(STREAM_MUSIC);

    status_t result = service->transact(GET_STREAM_VOLUME, data, &reply);
    if (result != NO_ERROR)
        ERROR("%s, failed to transact: %d", __func__, result);

    if (reply.readInt32() != 0) {
        ERROR("exception returned by audio manager");
        return -1;
    }
    return reply.readInt32();
}

int32_t getHeadsetState() {
    int inserted = 0;

    sp<IServiceManager> sm = defaultServiceManager();
    if (sm == NULL) {
        ALOGE("Unable to get default service manager!");
        return -1;
    }

    sp<IBinder> service = sm->checkService(String16("media.audio_policy"));
    if (service == NULL) {
        ALOGE("unable to get audio_policy");
        return -1;
    }

#define GET_DEVICE_CONNECTION_STATE 2

#define DEVICE_OUT_WIRED_HEADSET    0x4
#define DEVICE_OUT_WIRED_HEADPHONE  0x8
#define DEVICE_OUT_BLUETOOTH_A2DP   0x80

#define AUDIO_POLICY_DEVICE_STATE_UNAVAILABLE   0
#define AUDIO_POLICY_DEVICE_STATE_AVAILABLE     1

    int32_t deviceList[] = {DEVICE_OUT_WIRED_HEADSET, DEVICE_OUT_WIRED_HEADPHONE,
        DEVICE_OUT_BLUETOOTH_A2DP};

    for (int i = 0; i < (int)ARRAY_SIZE(deviceList); i++) {
        Parcel data, reply;
        data.writeInterfaceToken(String16("android.media.IAudioPolicyService"));
        data.writeInt32(deviceList[i]);
        data.writeCString("");

        status_t result = service->transact(GET_DEVICE_CONNECTION_STATE, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
        if (reply.readInt32() == AUDIO_POLICY_DEVICE_STATE_AVAILABLE) {
            DEBUG("headset 0x%x inserted", deviceList[i]);
            if(deviceList[i] == DEVICE_OUT_BLUETOOTH_A2DP)
                inserted = DEVICE_OUT_WIRED;
            else
                inserted = DEVICE_OUT_BLUETOOTH;
            break;
        }
    }
    return inserted;
}

bool isFileExist(const char *file_path)
{
    if (file_path == NULL)
        return false;
    if (access(file_path, F_OK) == 0)
        return true;
    return false;
}

bool isVendorFile(const std::string& path) {
    return path.compare(1,4,"proc") == 0 ||
           path.compare(1,3, "sys") == 0 ||
           path.compare(1,3, "dev") == 0 ;
}

std::string getFile(const std::string& path) {
    string result;
    DEBUG("Read file by orms: %s", path.c_str());

    if (OrmsProxy::getInstance().readFile(path, result) != 0)
        return string("");
    else
        return result;
    // return string("");
}

std::string getVendorFile(const std::string& path) {
#ifdef OIFACE_HIDL
    std::string out;
    OIfaceHalService::getInstance().readFile(path, out);
    return out;
#else
    return "";
#endif
}

std::string getSystemFile(const std::string& path) {
    std::ifstream ifs(path);
    return string((std::istreambuf_iterator<char>(ifs)),
                  (std::istreambuf_iterator<char>()));
}

int writeFile(const std::string& path, const std::string& str) {
#ifdef OIFACE_HIDL
    if(isVendorFile(path))
        return writeVendorFile(path, str);
    else
        return writeSystemFile(path, str);
#else
    return writeSystemFile(path, str);
#endif
}

int writeVendorFile(const std::string& path, const std::string& str) {
#ifdef OIFACE_HIDL
    return OIfaceHalService::getInstance().writeFile(path, str);
#else
    return 0;
#endif
}

int writeSystemFile(const std::string& path, const std::string& str) {
    int fd = open(path.c_str(), O_WRONLY);
    if (fd < 0) {
        ERROR("open %s failed(%s)", path.c_str(), strerror(errno));
        return -1;
    }

    ssize_t len = write(fd, str.c_str(), str.size());
    if (len <= 0) {
        ERROR("write file %s failed.(%s)", path.c_str(), strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

/* return start time of current process */
int64_t getStartTime() {
    string stime = getFile("/proc/self/stat");
    if (stime.size() <= 0) {
        ERROR("get time from proc file failed");
        return -1;
    }

    size_t pos = 0;
    string token;
    int count = 0;

    while ((pos = stime.find(" ")) != string::npos) {
        token = stime.substr(0, pos);
        stime.erase(0, pos + strlen("\t"));
        count++;

        if (count == 22)
            break;
    }

    if (count != 22) {
        ERROR("unable to find start time\n");
        return -1;
    }

    int64_t curTime = systemTime(SYSTEM_TIME_BOOTTIME) / 1000000000LL;
    int64_t t = curTime - stoll(token)/sysconf(_SC_CLK_TCK);
    DEBUG("conf:%ld curTime:%ld alread started:%ld", sysconf(_SC_CLK_TCK), curTime, t);

    DEBUG("oiface already start %ld seconds", t);
    return t;
}

int32_t getBrightness() {
    string path;
    if (GlobalConfig::getInstance().getConfig(&path, {"brightness_path"}) < 0) {
        path = SYS_BRIGHTNESS;
    }

    std::string bright = getFile(path);
    if (bright.size() != 0) {
        DEBUG("brightness level:%s len:%d", bright.c_str(), (int)bright.size());
        return std::stoi(bright);
    }

    return -1;
}

int32_t getLmhCnt(const char *path) {
    string content = getFile(path);
    if (content.size() == 0)
        return -1;

    return atoi(content.c_str());
}

int getVendorFileList(const char *p, vector<string>& str) {
	// FIXME: use orms instead
	string path = string(p);
    if (OrmsProxy::getInstance().readFileList(path, str) != 0)
        return -1;
#if 0
    sp<IOiface> hal_service = IOiface::getService();
    if (hal_service == nullptr) {
        ERROR("Can't find oiface hardware service...");
        return -1;
    }

    hal_service->getFileList(p, [&](hidl_vec<hidl_string> result) {
        for (auto &item: result)
            str.push_back(item);
    });
#endif

    return 0;
}

int getFileList(const char *p, vector<string>& str) {
    struct dirent** namelist;
    int n;
    char path[PATH_MAX];
    char file[PATH_MAX];
    snprintf(path, sizeof(path), "%s", p);
    n = scandir(p, &namelist, 0, alphasort);
    if (n < 0) {
        DEBUG("scandir %s failed. (%s)\n", p, strerror(errno));
        return -1;
    } else {
        while(n--) {
            if (namelist[n]->d_name[0] != '.') {
                snprintf(file, sizeof(file), "%s%s", path, namelist[n]->d_name);
                str.push_back(file);
            }
            free(namelist[n]);
        }
        free(namelist);
    }
    return 0;
}

vector<int> getPidByName(const char* task_name, int gamePid/* = 0 */, bool scan_all /* = 0 */) {
    int pid;
    vector<string> fileList;
    string cur_task_name;
    string taskDir;
    vector<int> pid_list;

    if (gamePid != 0) {
        taskDir = string("/proc/") + to_string(gamePid) + string("/task/");
        DEBUG("scaning dir %s", taskDir.c_str());
    } else {
        DEBUG("scaning /proc dir");
        taskDir = "/proc/";
    }

    if(getFileList(taskDir.c_str(), fileList) == -1) {
        ERROR("can not get proc file list");
        return pid_list;
    }

    if(fileList.size() == 0) {
        ERROR("return zero file list");
        return pid_list;
    }
    for(auto &item : fileList) {
        string base = basename(item.c_str());
        if (!std::all_of(base.begin(), base.end(), ::isdigit))
            continue;

        string filepath = item+"/comm";
        cur_task_name = getSystemFile(filepath);
        /* FIXME: may be there's no priviledge. */
        if(cur_task_name.empty()) {
            continue;
        }

        if (cur_task_name.back() == '\n')
            cur_task_name.pop_back();

        if (cur_task_name.find(task_name) != std::string::npos) {
            int pos = item.find_last_of('/');
            std::string string_pid = item.substr(pos+1);
            DEBUG("find thread %s, PID: %s\n", task_name, string_pid.c_str());
            pid = atoi(string_pid.c_str());
            pid_list.push_back(pid);

            if (!scan_all)
                break;
        }
    }

    if (pid_list.size() == 0)
        ERROR("can not find task:%s", task_name);

    return pid_list;
}

#define ACCT_PATH  "/acct"
#define ACCT_USAGE      "cpuacct.usage_percpu"
#define MAX_CORES       8
/* Get cpu usage in seconds */
float getCpuUsage(int type, int uid/* = -1*/, int pid /* = -1*/) {
    char buf[1024];
    char path[PATH_MAX];

    /* FIXME: currently support maximum 8 CPU cores */
    unsigned long long usage[MAX_CORES];
    unsigned long long  allUsage = 0; /* in seconds */

    if (uid != -1) {
        if (pid != -1) {
            snprintf(path, sizeof(path), "%s/uid_%d/pid_%d/", ACCT_PATH, uid, pid);
        } else {
            snprintf(path, sizeof(path), "%s/uid_%d/", ACCT_PATH, uid);
        }
    } else {
        snprintf(path, sizeof(path), "%s/", ACCT_PATH);
    }

    strcat(path, ACCT_USAGE);

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        ERROR("open %s failed(%s)", path, strerror(errno));
        return -1;
    }

    int len = read(fd, buf, sizeof(buf) - 1);
    if (len <= 0) {
        ERROR("read failed(%s)", strerror(errno));
        close(fd);
        return -1;
    }

    buf[len] = '\0';

    close(fd);

    len = sscanf(buf, "%llu %llu %llu %llu %llu %llu %llu %llu", &usage[0], &usage[1], &usage[2],
            &usage[3], &usage[4], &usage[5], &usage[6], &usage[7]);

    if (len < MAX_CORES) {
        ERROR(ACCT_USAGE " format not expected" );
        return -1;
    }

    int platform = PlatformAdaptor::getInstance().getPlatform();
    if (platform == PLAT_UNKNOWN) {
        ERROR("unable to get cpu usage due to unknown platform");
        return -1;
    }

    cpu_set_t set;
    CPU_ZERO(&set);
    AffinityService::getCpuset(platform, type, &set);

    for (int i = 0; i < MAX_CORES; i++) {
        if (CPU_ISSET(i, &set)) {
            allUsage += usage[i];
        }
    }

    DEBUG("CPU time for type %d is %.3f seconds", type, float(allUsage)/1000000000.0f);
    return float(allUsage) / 1000000000.0f;
}

int getUidByPid(pid_t pid) {
    struct stat st;
    char buf[PATH_MAX];

    snprintf(buf, sizeof(buf), "/proc/%d/task", pid);
    if (stat(buf, &st) < 0) {
        ERROR("stat %s failed.(%s)", buf, strerror(errno));
        return -1;
    }

    DEBUG("stat %s sucess, uid:%d", buf, st.st_uid);

    return st.st_uid;
}

#define SCHEDX_PROCESS_CPUTIME  "/proc/schedx/process_cputime"
int getAllProcessCpuTime(map<int, float> &cpuTime, map<int, string> &pidComm) {
    /* Use schedx interface */

    string get_file = getFile(SCHEDX_PROCESS_CPUTIME);
    if(get_file.empty()) {
        ERROR("file %s get failed\n", SCHEDX_PROCESS_CPUTIME);
        return -1;
    }
    stringstream infile;
    infile << get_file;
    string line;

    while (std::getline(infile, line)) {
        int pid;
        uint64_t time;
        char buf[16];
        int len;
        len = sscanf(line.c_str(), "%d %lu %16[^\t\n]", &pid, &time, buf);
        if (len < 3) {
            DEBUG("scanf returns %d for %s", len, line.c_str());
            continue;
        }

        cpuTime[pid] = (float)(time) / 1000000000.0f;
        pidComm[pid] = buf;
    }

    return 0;
}

int getTimeInState(const char *path, map<int32_t, int64_t> &timeInState) {
    ATRACE_CALL();
    if (access(path, R_OK) < 0) {
        ERROR("access %s failed(%s)\n", path, strerror(errno));
        return -1;
    }
    ifstream infile(path);
    string line;
    const int timeinstateMaxLineCnt = 150;
    int readLineCnt = 0;
    while (std::getline(infile, line) && readLineCnt++ < timeinstateMaxLineCnt) {
        unsigned int key;
        unsigned long long value;
        int len;

        len = sscanf(line.c_str(), "%u %llu\n", &key, &value);
        if (len < 2) {
            DEBUG("sscanf returns %d for %s", len, line.c_str());
            return -1;
        }

        timeInState[key] = value;

    }

    return 0;
}

int getCpuTimeInState(int cpu, map<int32_t, int64_t> &timeInState) {
#define CPU_TIME_IN_STATE_PATH  "/sys/devices/system/cpu/cpu%d/cpufreq/stats/time_in_state"
    char path[PATH_MAX];
    snprintf(path, sizeof(path), CPU_TIME_IN_STATE_PATH, cpu);

    return getTimeInState(path, timeInState);
}

int getTimeInState(const char *path, std::map<int32_t, std::vector<int64_t> > &timeInState) {
    ATRACE_CALL();
    if (access(path, R_OK) < 0) {
        ERROR("access %s failed(%s)\n", path, strerror(errno));
        return -1;
    }
    ifstream infile(path);
    string line;
    const int maxTimeinstateLine = 150;
    int lineCnt = 0;
    while (std::getline(infile, line) && lineCnt++ < maxTimeinstateLine) {
        unsigned int key;
        unsigned long long value;
        unsigned long long idleValue[6];    //The max cpu nums of cluster is 6
        std::vector<int64_t> vec;
        int len;

        vec.clear();
        len = sscanf(line.c_str(), "%u %llu %llu %llu %llu %llu %llu %llu", &key, &value,
            &idleValue[0], &idleValue[1], &idleValue[2], &idleValue[3], &idleValue[4], &idleValue[5]);
        if (len < 2) {
            DEBUG("sscanf returns %d for %s", len, line.c_str());
            return -1;
        }

        vec.push_back(value);
        for (int i = 0; i < len-2; i++) {
            vec.push_back(idleValue[i]);
        }

        timeInState[key] = vec;
    }

    return 0;
}

int getCpuTimeInState(int cpu, std::map<int32_t, std::vector<int64_t> > &timeInState) {
#define CPU_TIME_IN_STATE_PATH  "/sys/devices/system/cpu/cpu%d/cpufreq/stats/time_in_state"
    char path[PATH_MAX];
    snprintf(path, sizeof(path), CPU_TIME_IN_STATE_PATH, cpu);

    return getTimeInState(path, timeInState);
}

#define CPU_IDLE_PATH    "/sys/devices/system/cpu/cpu%d/cpuidle/"
/* return CPU idle time in ms */
int64_t getCpuIdleTime(int cpu) {
    ATRACE_CALL();
    char buf[PATH_MAX];
    vector<string> fileList;
    int64_t time = 0;

    snprintf(buf, sizeof(buf), CPU_IDLE_PATH, cpu);
    getFileList(buf, fileList);

    if (fileList.size() <= 0) {
        ERROR("get file list failed");
        return -1;
    }

    for (auto &fileName: fileList) {
        string content;

        if (strncmp(basename(fileName.c_str()), "state", strlen("state")) != 0) {
            continue;
        }

        content = getFile(fileName + "/time");
        if (content.size() <= 0)
            continue;

        time += std::stoll(content);
    }

    return time / 1000;
}

/* return gpu stats in millisecond */
int getGpuStats(map<int32_t, int32_t> &stat) {
#define GPU_FREQ_TABLE_PATH "/sys/class/kgsl/kgsl-3d0/freq_table_mhz"
#define GPU_FREQ_STAT_PATH  "/sys/class/kgsl/kgsl-3d0/gpu_clock_stats"
    char *saveptr;
    char *str, *token;
    vector<int> freq_list;
    vector<int> stat_list;
    int i;
    char buf[4096]; /* maximum page size */
    string path;

    if (GlobalConfig::getInstance().getConfig(&path, {"gpu_freq_table_path"}) < 0) {
        ERROR("get gpu config path failed. Fallback to default.");
        path = GPU_FREQ_TABLE_PATH;
    }

    string table = getFile(path);
    if (table.size() <= 0) {
        ERROR("get gpu freq table failed");
        return -1;
    }

    /* TODO: optmize out strcpy */
    strncpy(buf, table.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    for (i = 0, str = buf;;i++, str = NULL) {
        token = strtok_r(str, " \t\n", &saveptr);
        if (token == NULL || i > 80) {
            if (i > 80) {
                ERROR("get wrong file content please check path %s's content", path.c_str());
            }
            break;
        }

        freq_list.push_back(atoi(token));
    }

    if (freq_list.size() == 0) {
        ERROR("get freq list failed");
        return -1;
    }

    if (GlobalConfig::getInstance().getConfig(&path, {"gpu_freq_stat_path"}) < 0) {
        path = GPU_FREQ_STAT_PATH;
    }

    /* TODO: optmize out strcpy */
    table = getFile(path);
    strncpy(buf, table.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    for (i = 0, str = buf; ;i++, str = NULL) {
        token = strtok_r(str, " \t\n,", &saveptr);
        if (token == NULL || i > 80) {
            if (i > 80) {
                ERROR("get wrong file content please check path %s's content", path.c_str());
            }
            break;
        }
        if ((strlen(token) > 0) && (::isdigit(token[0])))
                stat_list.push_back((int32_t)(atoll(token) / 1000));
    }

    if (freq_list.size() != stat_list.size()) {
        ERROR("stat list size (%ld) is not match freq list (%ld)",
                        stat_list.size(), freq_list.size());
        return -1;
    }

    for (i = 0; i < (int)(freq_list.size()); i++) {
        stat[freq_list[i]] = stat_list[i];
    }

    return 0;
}

std::string getHashByUID(int uid) {
    string hash_value("");
    return hash_value;
}

#define GAME_SPACE_UI_HASH  "88F953D309D5CEF6BD862F6FDB5D2200EC7131DDD0862132B1926E3EB7133EB0"
#define GAME_SPACE_UI_NEW_HASH  "AF0AD9B2BDDE15CD76CD793BFD004011F674AC398EAD053608ACDC10BEEC1D31"
#define GAME_SPACE_UI_REALME_HASH  "84D42C16AEE15EB2CC8324DF55B1A3171AA55C20AAD3C0835DFE1011B65B53D0"
#define GAME_SPACE_UI_HASHKEY "760457E4B5B85F00EC031E9A753B8D415089849518C64F715F8F596D98973B89"
static const char* specialPermitedApps[] = {
        GAME_SPACE_UI_HASH,GAME_SPACE_UI_NEW_HASH,GAME_SPACE_UI_REALME_HASH,GAME_SPACE_UI_HASHKEY
};

bool isSpecialPermited(int uid) {
    if ((uid != AID_SYSTEM) && (uid != AID_ROOT)) {
        vector<string> pkgList;
        getPackageForUid(uid, pkgList);
        if (pkgList.size() <= 0) {
            DEBUG("uid not permited:%d", uid);
            return false;
        } else  {
            string originHashValue = getHashByUID(uid);
            DEBUG("uid(%d), hash(%s)", uid, originHashValue.c_str());
            for (int i = 0; i < (int)ARRAY_SIZE(specialPermitedApps); i++) {
                if(strcmp(originHashValue.c_str(), specialPermitedApps[i]) == 0) {
                    return true;
                }
            }
            return false;
        }
    }
    return true;
}


bool isEvenDay() {
    struct tm *local;
    time_t t;
    t = time(NULL);
    local = localtime(&t);
    DEBUG("Local day is: %d\n", local->tm_mday);
    if (local->tm_mday % 2 == 0) {
        return true;
    }
    return false;
}

int getTasks(int pid, vector<struct boost_task>& tasks) {
    ATRACE_CALL();
    char path[256];
    char task_file[256];
    struct dirent **tasklist;
    int n, count = 0;

    snprintf(path, sizeof(path), "/proc/%d/task", pid);
    n = scandir(path, &tasklist, 0, alphasort);
    if (n < 0) {
        ERROR("getTasks scandir %s failed(%s)", path, strerror(errno));
        return -1;
    }
    DEBUG("getTasks pid %d has %d tasks", pid, n);
    while (n--) {
        snprintf(task_file, sizeof(task_file), "%s/%s/schedstat",
                path, tasklist[n]->d_name);
        int task_id = atoi(tasklist[n]->d_name);
        free(tasklist[n]);

        if (task_id <= 0)
            continue;

        string out = getSystemFile(task_file);
        if (out.empty()) {
            ERROR("getTasks error: file(%s) is empty", task_file);
            continue;
        }

        long long sum_exec_runtime = atoll(out.c_str());
        if (sum_exec_runtime <= 0) {
            ERROR("getTasks error, runtime %lld, task_id %d",
                    sum_exec_runtime, task_id);
            continue;
        }
        struct boost_task task;
        task.id = task_id;
        task.duration = sum_exec_runtime;
        tasks.push_back(task);
        count++;
    }
    free(tasklist);

    // sort
    sort(tasks.begin(), tasks.end(), compareBoostTask);

    return count;
}

static inline int getConfigTaskNameList(const string& package_name,
        const string& boost_group, vector<string>& configTaskList) {
    if (GlobalConfig::getInstance().getConfig(configTaskList,
            {"sdk", package_name, boost_group}) < 0
            && GlobalConfig::getInstance().getConfig(configTaskList,
            {"oifacegame", package_name, boost_group}) < 0
            && GlobalConfig::getInstance().getConfig(configTaskList,
            {"sdk", "connectionless", package_name, boost_group}) < 0) {
        INFO("getConfigTaskNameList: can not get task list (%s) for (%s) !",
                boost_group.c_str(), package_name.c_str());
        return -1;
    }
    return 0;
}

int getConfigTaskList(int index, const string& package_name, vector<int>& taskList) {
    ATRACE_CALL();
    int ret = -1;
    string boost_group = "boost_tasks";
    vector<string> configTaskList;

    if (index >= 0) {
        string boost_group_idx = boost_group + to_string(index);
        ret = getConfigTaskNameList(package_name, boost_group_idx, configTaskList);
    }
    if (ret < 0) {
        ret = getConfigTaskNameList(package_name, boost_group, configTaskList);
    }
    if (ret < 0) {
        return -1;
    }

    string out;
    for (auto& task : configTaskList) {
        vector<int> pidList = getPidByName(task.c_str(), 0 , true);
        for (auto pid : pidList) {
            taskList.push_back(pid);
            out += task + " ";
        }
    }

    DEBUG("getConfigTaskList: %s", out.c_str());

    return 0;
}

void split(const string& s, vector<string>& sv, const char flag) {
    sv.clear();
    istringstream iss(s);
    string temp;
    while (getline(iss, temp, flag)) {
        sv.push_back(temp);
    }
    return;
}

std::string getCpuLoadFromProcStat() {
    char cpu[5];
    char buf[128];
    int64_t user, nice, sys, idle, iowait, irq, softirq;
    int64_t total;

    static int64_t previous_total = 0;
    static int64_t previous_idle = 0;

    FILE *fp = fopen("/proc/stat", "r");
    if (fp == NULL) {
        ERROR("Failed to open /proc/stat");
        return "";
    }

    fgets(buf, sizeof(buf), fp);
    sscanf(buf, "%s%ld%ld%ld%ld%ld%ld%ld", cpu, &user, &nice, &sys, &idle,
        &iowait, &irq, &softirq);
    fclose(fp);

    total = user + nice + sys + idle + iowait + irq + softirq;

    int load = 0;
    if (total - previous_total == 0) {
        load = 100 * (idle - previous_idle) / (total - previous_total + 1);
    } else {
        load = 100 * (idle - previous_idle) / (total - previous_total);
    }
    load = 100 - load;
    previous_idle = idle;
    previous_total = total;

    return std::to_string(load);
}

#define CPU_LOAD_PATH   "/proc/schedx/cpustat"
std::string getCpuLoad() {
    string loadInfo = getFile(CPU_LOAD_PATH);
    vector<string> loadRaw;
    split(loadInfo, loadRaw, ' ');
    if(loadRaw.empty()) return "-1";
    int tmpSum = 0;
    int tempy = 0;
    for (string val : loadRaw) {
        if(val.size() <= 1){
            continue;
        }
        vector<string> singleCpuInfo;
        split(val, singleCpuInfo, ':');
        if(singleCpuInfo.size() < 2) {
            ERROR("cpu config is error!");
            continue;
        }
        tmpSum += stoi(singleCpuInfo[0])*stoi(singleCpuInfo[1]);
        tempy += stoi(singleCpuInfo[1]);
    }
    if(tempy != 0){
        DEBUG("cpu load is : %d", (tmpSum / tempy));
        return to_string(tmpSum / tempy);
    }

    return "-1";
}

#define GPU_NORMALIZATION_LOAD   "/sys/class/kgsl/kgsl-3d0/devfreq/gpu_load"
#define GPU_LOAD_PATH   "/sys/kernel/gpu/gpu_busy"
#define MTK_GPU_LOAD_PATH   "/sys/kernel/ged/hal/gpu_utilization"

std::string getGpuLoad() {
    std::string gpuStr = getSystemFile(GPU_NORMALIZATION_LOAD);
    DEBUG("gpu normalization load, gpuStr: %s", gpuStr.c_str());
    if (gpuStr.empty()) {
        gpuStr = getSystemFile(GPU_LOAD_PATH);
    }
    DEBUG("gpu load, gpuStr: %s", gpuStr.c_str());

    if(PlatformAdaptor::getInstance().isMTK()){
        gpuStr = getSystemFile(MTK_GPU_LOAD_PATH);
        DEBUG("MTK gpu load, gpuStr: %s", gpuStr.c_str());
    }
    if(!gpuStr.empty()){
        size_t pos = gpuStr.find(" ");
        string gpuOut = gpuStr.substr(0, pos);
        DEBUG("gpu load is: %s", gpuOut.c_str());
        return gpuOut;
    } else  {
        return string("-1");
    }
}

#define GPU_FREQUENCY_PATH  "/sys/kernel/gpu/gpu_clock"
#define MTK_GPU_FREQUENCY_PATH  "/sys/kernel/ged/hal/current_freqency"

std::string getGpuFreq() {
    std::string gpuStr = getSystemFile(GPU_FREQUENCY_PATH);
    //DEBUG("gpu freq, gpuStr: %s", gpuStr.c_str());

    if (PlatformAdaptor::getInstance().isMTK()) {
        gpuStr = getSystemFile(MTK_GPU_FREQUENCY_PATH);
        //DEBUG("MTK gpu freq, gpuStr: %s", gpuStr.c_str());
    }
    if (!gpuStr.empty()) {
        size_t pos = gpuStr.find(" ");
        string gpuFreq = gpuStr.substr(pos + 1);
        DEBUG("gpu freq is: %s", gpuFreq.c_str());
        return gpuFreq;
    } else {
        return string("0");
    }
}

void hypnusBroadcast(android::String16 package) {
    // TODO need to remove all hypnus function
}

void timeInStateToAverageFreq(map<int32_t, int64_t> &timeInState) {
    int64_t total_time = 1;
    int64_t total_time_freq = 0;

    for(map<int32_t, int64_t>::iterator it = timeInState.begin();
            it != timeInState.end();
            it++){
        total_time_freq += it->first * it->second;
        total_time += it->second;
    }
    DEBUG("Average freq %ld for %lds.", total_time_freq / total_time,
        total_time / 100);
}

std::string getTaskNameByTid(int pid, int tid) {
    std::string path = "/proc/" + to_string(pid) + "/task/" + to_string(tid) + "/comm";
    if (isFileExist(path.c_str())){
        std::string str = getFile(path);
        str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
        return str;
    }else{
        ERROR("%s is not exist.",path.c_str());
    }
    return "";
}


void splitString(const string& s, vector<string>& v, const string& c) {
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2) {
        v.push_back(s.substr(pos1, pos2-pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length()) {
        v.push_back(s.substr(pos1));
    }
}

void getCpuCapacity(std::vector<int>& cpu_capacity){
    for (int i = 0; i < 8; i++) {
        string cpuCapaFile = string("/sys/devices/system/cpu/cpu") + to_string(i) + string("/cpu_capacity");
        if (access(cpuCapaFile.c_str(), R_OK) < 0) {
            ERROR("access %s failed(%s)\n", cpuCapaFile.c_str(), strerror(errno));
            continue;
        }
        string out = getSystemFile(cpuCapaFile);
        if (out.empty()) {
            ERROR("getSystemFile error: file(%s) is empty", cpuCapaFile.c_str());
            continue;
        }
        int cpuCapa = atoi(out.c_str());
        cpu_capacity.push_back(cpuCapa);
    }

    return;
}

#define CPU_FREQ_PATH  "/sys/devices/system/cpu/cpufreq"
void getClusterLeadingCpuNo(std::vector<int>& leading_cpu) {
    struct dirent** namelist;
    int n = scandir(CPU_FREQ_PATH, &namelist, 0, alphasort);
    if (n < 0) {
        DEBUG("getClusterLeadingCpuNo scandir %s failed. (%s)\n", CPU_FREQ_PATH, strerror(errno));
        return;
    } else {
        for(int i=0; i<n; i++) {
            //INFO("scandir  get %d and %s", i, namelist[i]->d_name);
            if (strncmp("policy", namelist[i]->d_name, strlen("policy")) == 0) {
                //snprintf(file, sizeof(file), "cpu%c", namelist[n]->d_name[6]);
                leading_cpu.push_back(namelist[i]->d_name[6] - '0');
                //INFO("leading_cpu pushed %d", namelist[i]->d_name[6] - '0');
            }
            free(namelist[i]);
        }
        free(namelist);
    }
    return;
}

void getTidsByPid(int pid, vector<int>& tidList) {
    tidList.clear();
    char path[256];
    struct dirent **tasklist;
    int n = 0;

    snprintf(path, sizeof(path), "/proc/%d/task", pid);
    n = scandir(path, &tasklist, 0, alphasort);
    if (n < 0) {
        ERROR("getTasks scandir %s failed(%s)", path, strerror(errno));
        return;
    }
    DEBUG("getTasks pid %d has %d tasks", pid, n);
    while (n--) {
        int task_id = atoi(tasklist[n]->d_name);
        tidList.push_back(task_id);
        free(tasklist[n]);
    }
    free(tasklist);
    return;
}

void getKeyThreadStatus(pid_t pid, const vector<int>& tidList, vector<string>& outStaticUxInfo) {
    outStaticUxInfo.clear();
    if (pid <= 0) {
        return;
    }

    string staticUxPath(" ");
    string staticUx(" ");
    string taskCommPath(" ");
    string taskComm(" ");
    for (auto& taskTid : tidList) {
        staticUxPath = string("/proc/") + to_string(pid) + string("/task/") + to_string(taskTid) + string("/static_ux");
        if (!isFileExist(staticUxPath.c_str())) {
            DEBUG("static_ux is not exist");
            staticUxPath = string("/proc/") + to_string(pid) + string("/task/") + to_string(taskTid) + string("/ux_state");
        }
        taskCommPath = string("/proc/") + to_string(pid) + string("/task/") + to_string(taskTid) + string("/comm");
        staticUx = getSystemFile(staticUxPath);
        taskComm = getSystemFile(taskCommPath);
        if (staticUx.empty()) {
            ERROR("process %d task %d static_ux content is empty.", pid, taskTid);
            continue;
        } else if (taskComm.empty()) {
            ERROR("process %d task %d comm content is empty.", pid, taskTid);
            continue;
        }
        string info = string("process:") + to_string(pid) + string(" task name:") + taskComm.c_str() + string(" task id:") + to_string(taskTid)
            + string(" static_ux:") + staticUx.c_str();
        outStaticUxInfo.push_back(info);
    }
}


// keep same with PackageInfo.GET_SIGNING_CERTIFICATES
#define SIGNATURE_LENGTH 32
char *runtimeMD5 = NULL;
void setSgameSignatureMD5(std::string &md5) {
    if (runtimeMD5 == NULL) {
        runtimeMD5 = new char[SIGNATURE_LENGTH + 1];
    }
    if (runtimeMD5 != NULL) {
        memset(runtimeMD5, 0, SIGNATURE_LENGTH + 1);
        strcpy(runtimeMD5, md5.c_str());
    }
}

bool sgameSignatureMatch() {
    if (runtimeMD5 == NULL) {
        return false;
    }
    const char *sgameSignatureMD5 = "f82e9b8d123a424d9aeb45ea7ce2cc76";
    DEBUG("runtime md5 %s", runtimeMD5);
    DEBUG("should be %s", sgameSignatureMD5);
    return !strncmp(runtimeMD5, sgameSignatureMD5, SIGNATURE_LENGTH);
}

#define MAX_OPENID_LENGTH 100
char *openID = NULL;
void setOpenID(std::string &id) {
    if (openID == NULL) {
        openID = new char[MAX_OPENID_LENGTH];
    }
    if (openID != NULL) {
        memset(openID, 0, MAX_OPENID_LENGTH);
        strcpy(openID, id.c_str());
    }
}

char *getOpenID() {
    if (openID == NULL) {
        ERROR("openID is NULL");
        return "";
    }
    return openID;
}

#define CPU_STAT_PATH    "/proc/stat"
std::vector<float> getCpuUsage(std::vector<int64_t>& preUse) {
    std::ifstream ifs(CPU_STAT_PATH);
    if (!ifs.is_open()) {
        ERROR("Fail to open %s", CPU_STAT_PATH);
        return {};
    }
    std::string line;

    std::vector<float> totalUsage;
    bool firstLine = true;
    int count = 0;
    while (std::getline(ifs, line) && count < 8) {
        if (firstLine) {
            firstLine = false;
            continue;
        }
        const int size = 10;
        const int idlePos = 3;
        std::string cpuName;
        std::vector<int64_t> cpuCurUse(size);

        //user nice system idle iowait irq softirq steal guest guest_nice
        int len = sscanf(line.c_str(), "%s %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
                                     &cpuName, &cpuCurUse[0], &cpuCurUse[1],
                                     &cpuCurUse[2], &cpuCurUse[3], &cpuCurUse[4],
                                     &cpuCurUse[5], &cpuCurUse[6], &cpuCurUse[7],
                                     &cpuCurUse[8], &cpuCurUse[9]);

        if (size + 1 != len) {
            ERROR("got wrong size %d", len);
            return {};
        }

        int64_t totalTime = std::accumulate(cpuCurUse.begin(), cpuCurUse.end(), 0);
        int64_t runningTime = totalTime - cpuCurUse[idlePos];
        if (preUse.size() >= 16) {
            float up = (float)(runningTime - preUse[2*count + 1]);
            float down = (float)(totalTime - preUse[2*count]);
            totalUsage.push_back(up/down);
            preUse[2*count] = totalTime;
            preUse[2*count + 1] = runningTime;
        }
        else {
            preUse.push_back(totalTime);
            preUse.push_back(runningTime);
        }
        count++;
    }
    return totalUsage;
}
