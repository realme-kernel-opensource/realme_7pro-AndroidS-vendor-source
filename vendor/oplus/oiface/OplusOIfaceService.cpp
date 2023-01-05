/*
 * Copyright(C) 2020 OPlus. All rights reserved.
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

#include <binder/IPCThreadState.h>
#include <binder/Parcel.h>
#include <private/android_filesystem_config.h>
#include <cutils/properties.h>
#include <openssl/sha.h>
#include <dlfcn.h>

#include "OplusOIfaceService.h"
#include "ThreadState.h"
#include "OIface.h"
#include "OIfaceServer.h"
#include "BinderClient.h"
#include "GeneralCheck.h"
#include "GlobalConfig.h"
#include "OIfaceModuleLoader.h"
#include "DataCollector.h"
#include "Utils.h"
#include "MemoryInfo.h"
#include "NetworkLatencyManager.h"
#include "GameStatusObserverManager.h"
#include "SurfaceFlingerProxyManager.h"
#include "PlatformAdaptor.h"
#include "FrameRescuer.h"
#include "DecisionDriver.h"
#include "AffinityService.h"
#include "ThermalService.h"
#include "VpsClient.h"
#include "AffinityService.h"
#include "OifaceCallbackManager.h"
#include "feature/gpa/GpaReader.h"
#include "HoraeServiceProxy.h"
#include "FrameRescuer.h"
#include <cutils/properties.h>
#include "ChargerService.h"
#include "CooExxManager.h"
#include "TaskMonitor.h"
#include "util/FrameStatsTracer.h"
#include "OplusProjectService.h"
#include "GameListManager.h"
#include "TaskManager.h"
#include "OplusTouchService.h"
#include "util/DataReader.h"
#include "FrameStateManager.h"
#include "LightningStartManager.h"
#include "GameOptService.h"
#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>
#include <gui/TraceUtils.h>

#define CONFIDENTIAL_PROP   "ro.version.confidential"
#define OTA_VERSION_PROP    "ro.build.version.ota"
#define OIFACE_PERMISSION   "com.oplus.permission.safe.GAME"

using namespace android;
using namespace std;


// JSHash
constexpr int hashCompileTime(const char *str) {
    unsigned int hash = 1315423911 ;

    if (str != NULL) {
        while(*str) {
            hash^=((hash << 5) + (*str++) + (hash >> 2));
        }
    }
    return(hash % HASH_M);
}


class BpOplusOIfaceService: public BpInterface<IOplusOIfaceService> {
    public:
        BpOplusOIfaceService(const sp<IBinder> &impl): BpInterface<IOplusOIfaceService>(impl) {}

        virtual void currentNetwork(int32_t status) override;
        virtual void currentPackage(const android::String16 &package,
                int32_t uid, int32_t pid) override;
};

void BpOplusOIfaceService::currentNetwork(int32_t status) {
    Parcel data, reply;

    data.writeInterfaceToken(IOplusOIfaceService::getInterfaceDescriptor());
    data.writeInt32(status);

    status_t result = remote()->transact(IOplusOIfaceService::CURRENT_NETWORK, data, &reply);
    if (result != NO_ERROR)
        ERROR("%s, failed to transact: %d", __func__, result);
}

void BpOplusOIfaceService::currentPackage(const android::String16 &package,
        int32_t uid, int32_t pid) {
    Parcel data, reply;

    data.writeInterfaceToken(IOplusOIfaceService::getInterfaceDescriptor());
    data.writeString16(package);
    data.writeInt32(uid);
    data.writeInt32(pid);

    status_t result = remote()->transact(IOplusOIfaceService::CURRENT_PACKAGE, data, &reply);
    if (result != NO_ERROR)
        ERROR("%s, failed to transact: %d", __func__, result);
}

IMPLEMENT_META_INTERFACE(OplusOIfaceService, "com.oplus.oiface.IOIfaceService");
IMPLEMENT_META_INTERFACE(OIfaceNetworkNotifier, "com.oplus.oiface.IOIfaceNetworkNotifier");
IMPLEMENT_META_INTERFACE(GameStatusNotifier, "com.oplus.oiface.IGameStatusNotifier");
IMPLEMENT_META_INTERFACE(OIfaceCallback, "com.oplus.oiface.IOIfaceCallback");
IMPLEMENT_META_INTERFACE(CoolExCallback, "com.oplus.oiface.ICoolExCallback");


OplusOIfaceService::OplusOIfaceService() {
    // HealthListener::getInstance();
    HoraeServiceProxy::getInstance().registerThermalListener();
    old_mode = 0;
    openID = "";
    permissionedUidList.clear();
}

status_t OplusOIfaceService::dump(int fd, const Vector<String16>& args) {
    const int uid = IPCThreadState::self()->getCallingUid();

    String8 result;

    result.appendFormat("Start time:%ld\n", getStartTime());
    result.appendFormat("Oiface version:%s\n", OIFACE_VERSION);
    result.appendFormat("Git version:%s\n\n", GIT_VERSION);
    //permission check
    if ((uid != AID_SYSTEM) && (uid != AID_ROOT)) {
        write(fd, result.string(), result.size());
        return NO_ERROR;
    }

    bool all = false;
    static const String16 helpOption("--help");
    static const String16 allOption("--all");
    static const String16 clientOption("--client");
    static const String16 keyThreadOption("--key-thread");
    static const String16 taskMonitorOption("--task-monitor");
    static const String16 memoryInfoOption("--memory");
    static const String16 appOption("--app");
    static const String16 coolExOption("--coolex");
    static const String16 bigDataOption("--big-data");
    static const String16 configOption("--config");

    /* Privileged user can get more information */
    for (size_t i = 0; i < args.size(); i++) {
        if (args[i] == helpOption) {
            result.appendFormat("Recognized parameters:\n");
            result.appendFormat("--all              show all records.\n");
            result.appendFormat("--help             display help.\n");
            result.appendFormat("--client           show all client info.\n");
            result.appendFormat("--task-monitor     heavy task info.\n");
            result.appendFormat("--key-thread       game key threads, protected by game zone.\n");
            result.appendFormat("--memory           oiface and system memory info.\n");
            result.appendFormat("--app              current app base info, include cpu, network, frameboost status etc.\n");
            result.appendFormat("--coolex           coolEx status.\n");
            result.appendFormat("--big-data         big data info.\n");
            result.appendFormat("--config           show oiface.config file, \"persist.sys.oiface.showconf\" must be set.\n");
            write(fd, result.string(), result.size());
            return NO_ERROR;
        }
        if (args[i] == allOption) {
            all = true;
        }

        if (all || args[i] == memoryInfoOption) {
            vector<MemoryInfo> info;
            MemoryInfo::getProcessMemoryInfo(&info);

            if (info.size() > 0) {
                map<string, string> stats = info[0].getMemoryStats();
                for (auto& iter : stats) {
                    result.appendFormat("%s:%s.\n", iter.first.c_str(), iter.second.c_str());
                }
            }
            long totalMem, availMem;
            if (SystemMemInfo::getSystemMem(totalMem, availMem) == 0) {
                result.appendFormat("totalMem:%ld.\n", totalMem);
                result.appendFormat("availMem:%ld.\n", availMem);
            }

            if (!all) {continue;}
        }

        if (all || args[i] == appOption) {
            result.appendFormat("cpu time all:%.3f seconds\n", getCpuUsage(CLUSTER_ALL));
            result.appendFormat("cpu time big cluster:%.3f seconds\n", getCpuUsage(CLUSTER_BIG));
            result.appendFormat("cpu time little cluster:%.3f seconds\n", getCpuUsage(CLUSTER_LITTLE));
            result.appendFormat("cpu time of oiface:%.3f seconds\n",
                    getCpuUsage(CLUSTER_LITTLE, getuid(), getpid()));

            result.appendFormat("Current network status: %s\n",
                ThreadState::getInstance().dumpNetworkStatus());

            Json::Value v;
            GlobalConfig::getInstance().getConfig(&v, {"oiface", "perfmode"});
            result.appendFormat("Performance mode: %d\n", v.asInt());
            result.appendFormat("Foreground package: %s\n",
                ThreadState::getInstance().getForgroundPackage().c_str());
            result.appendFormat("Frame boost status: %d package: %s\n",
                FrameRescuer::getInstance().getFrameBoostStatus(),
                FrameRescuer::getInstance().getClientName().c_str());

            if (!all) {continue;}
        }

        if (all || args[i] == clientOption) {
            vector<sp<OIfaceClient>> clients;
            OIfaceServer::getInstance().getClientList(&clients);
            for (auto &iter : clients) {
                if (OIfaceServer::isServerSocket(iter->getType())) {continue;}
                result.appendFormat("client:%s package name:%s.\n",
                        iter->getClientName().c_str(), iter->getPackageName().c_str());
                result.appendFormat("key thread: %s.\n",
                        iter->dumpKeyThreads().c_str());
                result.appendFormat("DecisionState: %s.\n",
                        iter->getDecisionState().toStyledString().c_str());
            }

            if (!all) {continue;}
        }

        if (all || args[i] == taskMonitorOption) {
            vector<struct task_normalized_rate> normalizedRates;
            TaskMonitor::getInstance().getNormalizedLoadRate(normalizedRates);
            for (auto &iter : normalizedRates) {
                if (iter.rate > 0.015) {// if rate is too low, ignore
                    result.appendFormat("TaskMonitor: normalized rate %f tid:%6d, %s.\n",  iter.rate, iter.tid, iter.name.c_str());
                }
            }

            if (!all) {continue;}
        }

        if (all || args[i] == keyThreadOption) {
            //for gameopt test
            GameOptService::getInstance().init();
            vector<int> tidList;
            map<int, int>& keyThreadInfo = TaskManager::getInstance().getKeyThreadReportInfo();
            result.appendFormat("key thread Info:\nsize: %u.\n", keyThreadInfo.size());
            if (keyThreadInfo.size() > 0) {
                for (auto& info : keyThreadInfo) {
                    result.appendFormat("taskId: %d, value: %d.\n", info.first, info.second);
                    tidList.push_back(info.first);
                }

                int currentPid = ThreadState::getInstance().getForgroundPid();
                vector<string> taskInfo;
                getKeyThreadStatus(currentPid, tidList, taskInfo);
                for (auto& task : taskInfo) {
                    result.appendFormat("%s.\n", task.c_str());
                }
            }

            if (!all) {continue;}
        }

        if (all || args[i] == coolExOption) {
            result.appendFormat("CoolEx filter type: %d\n", CooExxManager::getInstance().getCoolExFilterType());

            if (!all) {continue;}
        }

        if (all || args[i] == bigDataOption) {
            if (PlatformAdaptor::getInstance().isQcom()) {
                result.appendFormat("lmn_limit_cnt0:%s", getFile(LMH_LIMIT_CNT_PATH_0).c_str());
                result.appendFormat("lmn_limit_cnt1:%s", getFile(LMH_LIMIT_CNT_PATH_1).c_str());
                result.appendFormat("lmn_freq_limit0:%s", getFile(LMH_FREQ_LIMIT_PATH_0).c_str());
                result.appendFormat("lmn_freq_limit1:%s", getFile(LMH_FREQ_LIMIT_PATH_1).c_str());
            }
            result.append((string("DataCollector:\n") + DataCollector::getInstance().asJson()).c_str());

            if (!all) {continue;}
        }

        if (all || args[i] == configOption) {
            if (property_get_int32(OIFACE_SHOW_CONF_PROPERTY, 0)) {
            result.appendFormat("config file:\n%s\n", GlobalConfig::getInstance().getConfig().c_str());
            }

            if (!all) {continue;}
        }
    }

    write(fd, result.string(), result.size());

    return NO_ERROR;
}

#define CPU_CONTROL_ARG_MAX         20
#define CPU_CONTROL_ARG_COUNT       15
#define GPU_CONTROL_ARG_COUNT       5

status_t OplusOIfaceService::onTransact(uint32_t code, const Parcel& data, Parcel* reply,
        uint32_t flags) {
    int pid = IPCThreadState::self()->getCallingPid();
    int uid = IPCThreadState::self()->getCallingUid();
    ATRACE_FORMAT("callerUid %d, pid %d, code %d", uid, pid, code);
    if (!(code > 400 && code <= 410)) {
        if (code > COSA_GET_THERMAL_TEMPS || code < COSA_GET_FPS)
            DEBUG("got transaction code:%d flags:%d from pid:%d uid:%d", code, flags, pid, uid);
    }

    switch (code) {
        case GAMESPACE_PERFMODE_EVENT:
        case HYPNUSD_PERFMODE_EVENT: {

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            int type = data.readInt32();
            DEBUG("received perfmode message : %d ", type);
            Json::Value v;
            v["oiface"]["perfmode"] = type;
            GlobalConfig::getInstance().setConfig(v);
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_HYPNUS_POLL_EVENT;
            msg.json = v["oiface"].toStyledString();
            OIfaceServer::getInstance().sendMessage(msg);
        } break;
        case HYPNUSD_SCREEN_EVENT:
        case GAMESPACE_SCREEN_EVENT: {

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            int type = data.readInt32();
            DEBUG("received screen message : %d", type);
            BinderMessage msg;
            msg.what = type == 1 ? BINDER_MESSAGE_SCREEN_ON : BINDER_MESSAGE_SCREEN_OFF;
            msg.json = "";
            OIfaceServer::getInstance().sendMessage(msg);
        } break;
        case NOTIFY_BUFFER_PRODUCER: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game or system permission!");
                break;
            }

            /* sent from SF */
            int64_t timestamp = data.readInt64();
            int bufnum = data.readInt32();
            FrameStateManager::getInstance().notifyFrameProduce(bufnum, timestamp);
        } break;
        case CURRENT_NETWORK: {
            CHECK_INTERFACE(IOplusOIfaceService, data,reply);
            DEBUG("CURRENT_NETWORK");
            return NO_ERROR;
        } break;
        case CURRENT_PACKAGE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("CURRENT_PACKAGE");
            return NO_ERROR;
        } break;

        case RUN_PROGRAM: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            if (uid != AID_ROOT) {
                return PERMISSION_DENIED;
            }

            String16 program = data.readString16();
            DEBUG("geting parse result:\n%s", String8(program).string());
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_RUN_PROGRAM;
            msg.uid = uid;
            msg.pid = pid;
            msg.json = String8(program).string();

            OIfaceServer::getInstance().sendMessage(msg);

        } break;

        case COLLECT_DATA: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            reply->writeNoException();
            DEBUG("COLLECT_DATA");
            return NO_ERROR;
        } break;

        case GET_SUPPORT_PACKAGE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            // FIXME map: rewrite;  done
            std::string pkgStat;
            getSupportPackage(pkgStat);

            reply->writeNoException();
            reply->writeString16(String16(pkgStat.c_str()));
#if 0
            binder::Map pkgstat;
            getSupportPackage(pkgstat);

            reply->writeNoException();
            reply->writeMap(pkgstat);
#endif

        } break;
        case GET_SUPPORT_GAME_START_PACKAGE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            // FIXME map: rewrite; done
            std::string pkgStat;
            getSupportGameStartPackage(pkgStat);

            reply->writeNoException();
            reply->writeString16(String16(pkgStat.c_str()));
#if 0
            binder::Map pkgstat;
            getSupportGameStartPackage(pkgstat);

            reply->writeNoException();
            reply->writeMap(pkgstat);
#endif

        } break;
        case GET_SYSTEM_THERMAL: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game or system permission!");
                reply->writeFloat(0.0);
                reply->writeFloat(0.0);
                reply->writeFloat(0.0);
                break;
            }

            std::vector<float> temp;
            if(ThermalService::getInstance().getAllTemperature(temp) == -1) {
                ERROR("get Thermal error!\n");
                break;
            }
            reply->writeFloat(temp[ThermalService::TEMPERATURE_CPU]);
            reply->writeFloat(temp[ThermalService::TEMPERATURE_GPU]);
            reply->writeFloat(temp[ThermalService::TEMPERATURE_SKIN]);
        } break;
        case BIND_TASK: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            int32_t clusterType  = data.readInt32();
            int32_t task = data.readInt32();
            DecisionDriver::getInstance().directSetTask(task, clusterType);
        } break;
        case BIND_SINGLE_TASK: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            int32_t clusterType  = data.readInt32();
            int32_t taskId = data.readInt32();
            AffinityService::getInstance().setAffinityForSingleThread(
                taskId, clusterType);
        } break;
        case BIND_SINGLE_TASK_TIMEOUT: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            int32_t clusterType  = data.readInt32();
            int32_t taskId = data.readInt32();
            int32_t timeout = data.readInt32();
            AffinityService::getInstance().setAffinityForSingleThreadTimeout(
                taskId, clusterType, timeout);
        } break;
        case ENABLE_HQV: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }
                std::string value = to_string(data.readInt32());
                property_set(HQV_PROPERTY, value.c_str());
        } break;
        case SET_HALF_HQV: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }
                std::string value = to_string(data.readInt32());
                property_set(HQV_HALF_MODE_PROPERTY, value.c_str());
        } break;
        case SET_LOG_LEVEL: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }
            int32_t level  = data.readInt32();
            std::string ret = setOifaceLogLevel(level, uid);
            DEBUG("set oiface log level %s", ret.c_str());
        } break;
        case REGISTER_HQV: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            bool (*Register)(const std::string&, const std::vector<VpsConfigure>& configs);
            bool (*Unregister)(const std::string&);
            bool (*GetConfigs)(std::vector<VpsConfigure>* configs);
            reply->writeNoException();

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeString16(String16(""));
                break;
            }

            void *handle = dlopen("/system/lib64/libvpsclient.so", RTLD_NOW);
            if (handle == NULL) {
                ERROR("dlopen failed(%s)\n", dlerror());
                reply->writeString16(String16("{}"));
                break;
            }
            Register = (bool(*)(const std::string&,
                                const std::vector<VpsConfigure>&))dlsym(handle, "Register");
            if (Register == NULL) {
                ERROR("dlsym Register error(%s)\n", dlerror());
                dlclose(handle);
                break;
            }
            Unregister = (bool(*)(const std::string&))dlsym(handle, "Unregister");
            if (Unregister == NULL) {
                ERROR("dlsym Unregister error(%s)\n", dlerror());
                dlclose(handle);
                break;
            }
            GetConfigs = (bool(*)(std::vector<VpsConfigure>*))dlsym(handle, "GetConfigs");
            if (GetConfigs == NULL) {
                ERROR("dlsym GetConfigs error(%s)\n", dlerror());
                dlclose(handle);
                break;
            }
            string package = String8(data.readString16()).string();
            switch(data.readInt32()) {
                case HQV_TYPE_DEL_CONFIG: {
                    if (Unregister(package) != true) {
                        ERROR("Unregister %s failed", package.c_str());
                        reply->writeString16(String16("-1"));
                    } else {
                        DEBUG("unregistered %s sucess", package.c_str());
                        if (ThreadState::getInstance().isPackageForground(package)) {
                            DEBUG("registered package:%s is in forground, need to refresh", package.c_str());
                            hypnusBroadcast(String16(package.c_str()));
                        }
                        reply->writeString16(String16("0"));
                    }
                } break;
                case HQV_TYPE_SET_CONFIG: {
                    // FIXME map:
#if 1
                    std::vector<VpsConfigure> configs;
                    // binder::Map sets;
                    // data.readMap(&sets);
                    const char* setsJson = data.readCString();
                    Json::Value setsRoot;
                    Json::Reader reader;
                    setsRoot = reader.parse(setsJson, setsRoot);

                    if (setsRoot.empty()) {
                        DEBUG("get nothing config, just to enable:%s", package.c_str());
                        if (GetConfigs(&configs) != true) {
                            ERROR("get origin configs failed\n");
                            reply->writeString16(String16("-1"));
                            break;
                        }
                    } else {
                        VpsConfigure config("", 0);
                        /* {
                            for (auto &item: sets) {
                                config.key = item.first;
                                item.second.getInt(&(config.value));
                                configs.push_back(config);
                                DEBUG("setting config: %s:%d\n", config.key.c_str(), config.value);
                            }
                        } */

                        Json::Value::Members mem = setsRoot.getMemberNames();
                        for (auto iter = mem.begin(); iter != mem.end(); iter++) {
                            config.key = *iter;
                            std::string value = setsRoot[config.key.c_str()].asString();
                            configs.push_back(config);
                            DEBUG("setting config: %s:%d\n", config.key.c_str(), config.value);
                        }
                    }
                    if (Register(package, configs) != true) {
                        ERROR("register %s failed", package.c_str());
                        reply->writeString16(String16("-1"));
                    } else {
                        DEBUG("registered %s", package.c_str());
                        if (ThreadState::getInstance().isPackageForground(package)) {
                            DEBUG("registered package:%s is in forground, need to refresh", package.c_str());
                            hypnusBroadcast(String16(package.c_str()));
                        }
                        reply->writeString16(String16("0"));
                    }
#endif
                } break;
                case HQV_TYPE_GET_CONFIG: {
#if 0
                    std::vector<VpsConfigure> configs;
                    if (GetConfigs(&configs) != true) {
                        ERROR("get origin configs failed\n");
                        break;
                    }
                    binder::Map tmp;
                    for (auto &config:configs) {
                        tmp[config.key] = config.value;
                        DEBUG("hqv config: %s:%d\n", config.key.c_str(), config.value);
                    }
                    reply->writeMap(tmp);
#endif
                } break;
                default: {
                    ERROR("unknow HQV type input, expect 1,2,3 ");
                } break;
            }
        dlclose(handle);
        } break;
        case REGISTER_NETWORK_LISTENER: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("REGISTER_NETWORK_LISTENER");
            return NO_ERROR;
        } break;
        case REGISTER_OIFACE_CALLBACK: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            sp<IBinder> binder = data.readStrongBinder();
            OifaceCallbackManager::getInstance().registerOifaceCallback(binder);
        } break;
        case REGISTER_GAME_ROLE_EVENT_LISTENER: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            /*if ((uid != AID_SYSTEM) && (uid != AID_ROOT) && (uid != AID_RADIO)) {
                DEBUG("uid not permited:%d", uid);
                return PERMISSION_DENIED;
            }*/

            int32_t observerType = data.readInt32();
            std::string observerconfig = String8(data.readString16()).string();
            DEBUG("geting REGISTER_GAME_ROLE_EVENT_LISTENER observerconfig:\n %s", observerconfig.c_str());
            sp<IBinder> binder = data.readStrongBinder();

            GameStatusObserverManager::getInstance().registerObserverListener(observerType,
                    observerconfig, binder);

        } break;
        case UNREGISTER_GAME_ROLE_EVENT_LISTENER: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            Json::Value v;
            GlobalConfig::getInstance().getConfig(&v, {"oiface", "enable"});
            /*if (!v.isUInt()) {
                DEBUG("enable key not found");
                return PERMISSION_DENIED;
            }*/
            sp<IBinder> binder = data.readStrongBinder();

            GameStatusObserverManager::getInstance().unRegisterObserverListener(binder);

        } break;
        case LAYER_FRAME_STATS_DATA: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            String8 layerName;
            status_t ret_code = data.readString8(&layerName);
            if (ret_code != NO_ERROR || layerName.isEmpty())
                return BAD_VALUE;

            // get layer real name without "#number"
            string str(layerName.string());
            size_t pos = str.find_last_of('#');
            if (pos != string::npos && (pos + 1) < str.size()) {
                char c = *(str.c_str() + pos + 1);
                if (c >= '0' && c <= '9') {
                    str = str.substr(0, pos);
                }
            }
            DEBUG("recved frame stats, layer(%s)", str.c_str());
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_UPDATE_FRAME_STATS;
            msg.uid = 0;
            msg.pid = 0;
            msg.json = str;

            // copy data: struct FrameStats
            struct FrameStats& frameStats = msg.data.frameStats;
            int size = data.readInt32();
            if (size < (int)sizeof(frameStats)) {
                return BAD_VALUE;
            }
            frameStats.frameCount = data.readInt32();
            frameStats.displayPeriod = data.readInt64();
            frameStats.startTime = data.readInt64();
            frameStats.stopTime = data.readInt64();
            frameStats.meanInterval = data.readFloat();
            frameStats.stdev = data.readFloat();
            if (NUM_INTERVAL_STATS != data.readInt32()) {
                return BAD_VALUE;
            }
            for (int i = 0; i < NUM_INTERVAL_STATS; i++) {
                frameStats.intervalStat[i] = data.readInt32();
            }
            frameStats.statTime = data.readInt64();

            OIfaceServer::getInstance().sendMessage(msg);
        } break;

        case REGISTERED_LAYER_DELAY_BOOST: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("REGISTERED_LAYER_DELAY_BOOST");
            return NO_ERROR;
        } break;
        case FAKE_SYSTEM_NOTIFY: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            if ((uid != AID_ROOT) && (uid != AID_SYSTEM) && (uid != AID_SHELL)) {
                ERROR("No game permission!");
                break;
            }

            String16 level = data.readString16();
            DEBUG("geting parse result:\n%s", String8(level).string());

            BinderMessage msg;
            msg.what = BINDER_MESSAGE_NOTIFY_CALLBACK;
            msg.uid = uid;
            msg.pid = pid;
            msg.json = String8(level).string();

            OIfaceServer::getInstance().sendMessage(msg);

        } break;
        case GET_DEVICE_ID_NEW:
        case GET_DEVICE_ID: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            vector<string> pkgList;

            getPackageForUid(uid, pkgList);
            if (pkgList.size() <= 0) {
                ERROR("unable to get package name");
                reply->writeInt32(-1);
                break;
            }

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                INFO("No game permission for get device id, check package");

                if (strcmp(SGAME_PACKAGE_NAME, pkgList[0].c_str())
                    || !sgameSignatureMatch()) {
                    ERROR("Package check fail");
                    reply->writeInt32(-1);
                    break;
                }
            }

            reply->writeNoException();
            DEBUG("Return open ID: %s\n", getOpenID());
            reply->writeString16(String16(getOpenID()));
        } break;
        case GET_ALL_LOAD_INFO_R:
        case GET_ALL_LOAD_INFO: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            /*if (!isSpecialPermited(uid)) {
                return PERMISSION_DENIED;
            }*/
            // std::string packageName = String8(data.readString16()).string();

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game or system permission!");
                if (GET_ALL_LOAD_INFO_R == code) {
                    reply->writeNoException();
                }
                reply->writeString16(String16(""));
                break;
            }

            bool useGlobalFps = false;
            std::string packageName = String8(data.readString16()).string();
            std::string caller = getTaskNameByTid(pid, pid);
            DEBUG("caller is %s", caller.c_str());

            if (caller == ".oplus.onetrace") {
                DEBUG("use global sf fps");
                useGlobalFps = true;
            }

            DEBUG("input package name is %s", packageName.c_str());

            float fps = 0.0;
            if (useGlobalFps) {
                fps = SurfaceFlingerProxyManager::getInstance().getFramesCommon();
            } else if (packageName.empty()) {
                fps = SurfaceFlingerProxyManager::getInstance().getFps();
            } else {
                fps = SurfaceFlingerProxyManager::getInstance().getFps(packageName);
            }
            DEBUG("GET_ALL_LOAD_INFO fps %f", fps);
            std::string result = (to_string(fps) +
                    ":" + getCpuLoadFromProcStat() + ":" + getGpuLoad() + ":" + getGpuFreq());

            if (GET_ALL_LOAD_INFO_R == code)
                reply->writeNoException();
            reply->writeString16(String16(result.c_str()));
        } break;
        /*
        case GPA_GET_SYSTEM_INFO: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            //TODO: add more info to the reply  Important!!!
            std::string CpuLoad = getCpuLoad();
            std::string GpuLoad = getGpuLoad();
            std::string GpuFreq = getGpuFreq();
            std::vector<int> temp;
            if(ThermalService::getInstance().getAllTemperature(temp) == -1) {
                ERROR("get Thermal error!\n");
                break;
            }

            DEBUG("GPA Get CPU load: %s, GPU load: %s, GPU freq: %s", CpuLoad.c_str(), GpuLoad.c_str(), GpuFreq.c_str());
            DEBUG("GPA Get CPU temp: %d, GPU temp: %d, Skin Temp: %d", temp[ThermalService::TEMPERATURE_CPU], temp[ThermalService::TEMPERATURE_GPU], temp[ThermalService::TEMPERATURE_SKIN]);

            reply->writeString16(String16((getCpuLoad() + ":" + getGpuLoad() + ":" + getGpuFreq() +
                    ":" + to_string(temp[ThermalService::TEMPERATURE_CPU]) +
                    ":" + to_string(temp[ThermalService::TEMPERATURE_GPU]) +
                    ":" + to_string(temp[ThermalService::TEMPERATURE_SKIN])).c_str()));
        } break;

        case GPA_GET_FPS_COMMON: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            float fps = 0.0;
            fps = SurfaceFlingerProxyManager::getInstance().getFramesCommon();
            DEBUG("GPA Get FPS common: %f", fps);
            reply->writeString16(String16((to_string(fps)).c_str()));
        } break;
        case GPA_GET_FPS_SPECIFIED_PACKAGENAME: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            std::string packageName = String8(data.readString16()).string();
            float fps = 0.0;
            if (packageName.empty()){
                fps = SurfaceFlingerProxyManager::getInstance().getFrames();
            } else {
                fps = SurfaceFlingerProxyManager::getInstance().getFrames(packageName);
            }
            DEBUG("GPA Get FPS specified package name: %f", fps);
            reply->writeString16(String16((to_string(fps)).c_str()));
        } break;
        */
        case COSA_ENABLE_TGPA_BIND_TASK: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            int32_t enable = data.readInt32();
            AffinityService::getInstance().setTGPABindTaskEnable(enable);
        } break;
        case NOTIFY_TOUCH_TRACE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            int32_t state = data.readInt32();
            String8 pkgName = state ? String8(data.readString16()) : String8("");
            DEBUG("notify touch trace cmd: %s, package name = %s\n", state ? "start" : "stop", pkgName.c_str());
            property_set("debug.oiface.trace.pkg_name", pkgName.c_str());
            property_set("debug.oiface.trace.switch", state ? "1" : "0");

        } break;

        // FIXME map: NOTIFY_GAME_JITTER should be deprecated done
        case NOTIFY_GAME_JITTER: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("NOTIFY_GAME_JITTER");
            return NO_ERROR;
        } break;
        case SET_GCP_EFFECT_MODE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game or system permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            int mode = data.readInt32();
            int set_mode;
            char value[PROPERTY_VALUE_MAX];
            property_get("ro.oplus.oiface.gcp.extmode", value, NULL);
            DEBUG("hinlock get prop is %s", value);

            if (!strncmp(value, "true", 4)) {
                if (old_mode == 263) {
                    if (mode == 20003) {
                        set_mode = 258;
                    } else if (mode == 20006) {
                        set_mode = 259;
                    } else if (mode == 20008) {
                        set_mode = 264;
                    } else {
                        set_mode = mode;
                    }
                } else {
                    set_mode = mode + 2;
                }
                old_mode = set_mode;
                DEBUG("hinlock set_mode is %d old_mode is %d", set_mode, old_mode);
            } else {
                set_mode = mode;
            }
            int result = SurfaceFlingerProxyManager::getInstance().setGCPEffectMode(set_mode);
            DEBUG("Gcp result %d", result);
            reply->writeNoException();
            reply->writeString16(String16(to_string(result).c_str()));
        } break;
        case NOTIFY_TEMP_LEVEL:{
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("NOTIFY_TEMP_LEVEL");
            return NO_ERROR;
        } break;
        case COSA_ENABLE_TGPA_LIMTING_NOTIFY: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            int32_t enable = data.readInt32();
            std::string packageName = String8(data.readString16()).string();
            INFO(" packageName[%s] TGPA_LIMTING_NOTIFY feature %s", packageName.c_str(), enable ? "enable" : "disabled");
            std::string clientName = OIfaceServer::getInstance().getClientName(packageName);
            sp<OIfaceClient> client = OIfaceServer::getInstance().asOIfaceClient(clientName.c_str());
            if (client == NULL) {
                INFO("can not get client packageName[%s],clientName[%s]",packageName.c_str(), clientName.c_str());
                break;
            }
            client->setTGPANotifyEnable(enable);
        } break;
        case COSA_CURRENT_PKG_STATUS:{
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            packageSwitch(data);
        } break;

        case COSA_GET_FPS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            std::string packageName = String8(data.readString16()).string();
            int type = data.readInt32();
            DEBUG("type: %d\n", type);
            reply->writeNoException();

            int count = GpaReader::getInstance().getFPS(packageName, type);
            reply->writeInt32(count);
        } break;

        case COSA_GENERAL_OIFACE_SIGNAL: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            std::string signal = String8(data.readString16()).string();
            std::string result = generalOIfaceSignal(signal, uid);
            reply->writeNoException();
            reply->writeString16(String16(result.c_str()));
        } break;

        case COSA_OIFACE_DECISION: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            std::string decision = String8(data.readString16()).string();
            DEBUG("Oiface decision: %s\n", decision.c_str());
            DecisionDriver::getInstance().oifaceDecision(decision);
        } break;

        case COSA_OIFACE_CONTROL: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            std::string control = String8(data.readString16()).string();
            DEBUG("Oiface control: %s\n", control.c_str());
        } break;
        case COSA_GET_CPU_CLUSTER_NUM: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            reply->writeNoException();
            reply->writeInt32(0);
        } break;

        case COSA_GET_CPU_AVAILABLE_FREQ_TABLE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<uint64_t> freq;
                freq.push_back(0);
                reply->writeUint64Vector(freq);
                break;
            }

            int type = data.readInt32();
            //DEBUG("type: %d\n", type);
            reply->writeNoException();

            //std::vector<int64_t> freq = {100000, 200000};
            std::vector<uint64_t> freq = GpaReader::getInstance().getCpuAvailableFreqTable(type);
            reply->writeUint64Vector(freq);

        } break;

        case COSA_GET_CPU_LIMITED_FREQS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<uint64_t> freq;
                freq.push_back(0);
                reply->writeUint64Vector(freq);
                break;
            }

            int type = data.readInt32();
            //DEBUG("type: %d\n", type);
            reply->writeNoException();

            //std::vector<int64_t> freq = {100000, 200000};
            std::vector<uint64_t> freq = GpaReader::getInstance().getCpuLimitedFreqs(type);
            reply->writeUint64Vector(freq);

        } break;

        case COSA_GET_CPU_CURRENT_FREQ: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<uint64_t> freq;
                freq.push_back(0);
                reply->writeUint64Vector(freq);
                break;
            }

            int type = data.readInt32();
            //DEBUG("type: %d\n", type);
            reply->writeNoException();

            //std::vector<int64_t> freq = {100000, 200000};
            std::vector<uint64_t> freq = GpaReader::getInstance().getCpuCurrentFreqs(type);
            reply->writeUint64Vector(freq);

        } break;

        case COSA_GET_CPU_LOADS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<float> freq;
                freq.push_back(0.0);
                reply->writeFloatVector(freq);
                break;
            }

            int type = data.readInt32();
            //DEBUG("type: %d\n", type);
            reply->writeNoException();
            //std::vector<float> freq = {12, 22};
            std::vector<float> freq = GpaReader::getInstance().getCpuLoads(type);
            reply->writeFloatVector(freq);

        } break;

        case COSA_GET_GPU_AVAILABLE_FREQ_TABLE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<uint64_t> freq;
                freq.push_back(0);
                reply->writeUint64Vector(freq);
                break;
            }

            reply->writeNoException();

            //std::vector<int64_t> freq = {100000, 200000};
            std::vector<uint64_t> freq = GpaReader::getInstance().getGpuAvailableFreqTable();
            reply->writeUint64Vector(freq);

        } break;

        case COSA_GET_GPU_LIMITED_FREQS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<uint64_t> freq;
                freq.push_back(0);
                reply->writeUint64Vector(freq);
                break;
            }

            reply->writeNoException();

            //std::vector<int64_t> freq = {100000, 200000};
            std::vector<uint64_t> freq = GpaReader::getInstance().getGpuLimitedFreqs();
            reply->writeUint64Vector(freq);

        } break;

        case COSA_GET_GPU_CURRENT_FREQ: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeUint64(0);
                break;
            }

            reply->writeNoException();
            uint64_t ret = GpaReader::getInstance().getGpuCurrentFreqs();
            reply->writeUint64(ret);
        } break;

        case COSA_GET_GPU_LOADS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeFloat(0.0);
                break;
            }

            reply->writeNoException();
            float ret = GpaReader::getInstance().getGpuLoad();
            reply->writeFloat(ret);
        } break;

        case COSA_GET_THERMAL_TEMPS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<float> temp;
                temp.push_back(0.0);
                reply->writeFloatVector(temp);
                break;
            }

            int type = data.readInt32();
            //DEBUG("COSA_GET_THERMAL_TEMPS: %d\n", type);
            reply->writeNoException();

            std::vector<float> temp;
            switch (type) {
                case 100: {//util
                    temp = DataReader::getInstance().getData();
                    temp.emplace_back(std::stof(getGpuLoad()) / 100);
                } break;
                case 3: {//util + frame count
                    temp = DataReader::getInstance().getData();
                    temp.emplace_back(std::stof(getGpuLoad()) / 100);
                    temp.emplace_back(GpaReader::getInstance().getFPS("", 1));
                } break;
                case 4: {//util + ratio
                    float ratio = DataReader::getInstance().mFrameIntervalCollector.getIntervals();
                    temp = DataReader::getInstance().getData();
                    temp.emplace_back(std::stof(getGpuLoad()) / 100);
                    temp.emplace_back(ratio);
                } break;
                case 5: {//util + frame count + ratio
                    float ratio = DataReader::getInstance().mFrameIntervalCollector.getIntervals();
                    temp = DataReader::getInstance().getData();
                    temp.emplace_back(std::stof(getGpuLoad()) / 100);
                    temp.emplace_back(GpaReader::getInstance().getFPS("", 1));
                    temp.emplace_back(ratio);
                } break;
                default: {
                    temp = GpaReader::getInstance().getSystemInfo(type);
                } break;
            }
            reply->writeFloatVector(temp);
        } break;

        case SET_TOUCH_SENSIBILITY: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            int value = data.readInt32();
            DEBUG("SET_TOUCH_SENSIBILITY: %d\n", value);
            reply->writeNoException();
            int result = DecisionDriver::getInstance().setTouchSensibility(value);
            reply->writeInt32(result);
        } break;

        case SET_TOUCH_RESPONSIVENESS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            int value = data.readInt32();
            DEBUG("SET_TOUCH_RESPONSIVENESS: %d\n", value);
            reply->writeNoException();
            int result = DecisionDriver::getInstance().setTouchSmoothly(value);
            reply->writeInt32(result);
        } break;

        case SET_GYROSCOPE_LEVEL: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            int value = data.readInt32();
            DEBUG("SET_GYROSCOPE_LEVEL: %d\n", value);
            reply->writeNoException();

            reply->writeInt32(0);
        } break;

        case SET_TOUCH_PROTECTION: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            int value = data.readInt32();
            DEBUG("SET_TOUCH_PROTECTION: %d\n", value);
            reply->writeNoException();
            int result = DecisionDriver::getInstance().setTouchProtection(value);
            reply->writeInt32(result);
        } break;

        case ENABLE_HAPTIC_VABRATION: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                break;
            }

            int haptic_enable = data.readInt32();
            if(haptic_enable == 1){
                   DEBUG("start hapticscreencap");
                   property_set("ctl.start","hapticImgScreencap");
            }else if(haptic_enable == 0){
                   DEBUG("stop hapticscreencap");
                   property_set("ctl.stop","hapticImgScreencap");
            }
        } break;
        case REGISTER_COOLEX_CALLBACK: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            sp<IBinder> binder = data.readStrongBinder();
            CooExxManager::getInstance().registerCoolExCallback(binder);
            int type = CooExxManager::getInstance().getCoolExFilterType();
            CooExxManager::getInstance().reportCoolExFilterType(type, "");
            reply->writeNoException();
        } break;
        case TASK_MONITOR: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game or system permission!");
                break;
            }

            int32_t enabled = data.readInt32();
            if (enabled == 1) {
                int32_t pid = data.readInt32();
                int32_t timeDuration = data.readInt32();
                int32_t timeThreshold = data.readInt32();
                int32_t fpsThreshold = data.readInt32();
                TaskMonitor::getInstance().start(pid, timeDuration, timeThreshold, fpsThreshold);
            } else {
                TaskMonitor::getInstance().stop();
            }
        } break;
        case GET_BATTERY_REMAIN: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            reply->writeNoException();
            int batteryRemain = ChargerService::getInstance().getBatteryRemain();
            DEBUG("GET_BATTERY_REMAIN: %d\n", batteryRemain);
            reply->writeInt32(batteryRemain);
        } break;
        case GET_BATTERY_CURRENT_NOW: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeFloat(0.0);
                break;
            }

            reply->writeNoException();
            float batteryCurrentNow = ChargerService::getInstance().getBatteryCurrentNow();
            DEBUG("GET_BATTERY_CURRENT_NOW: %f\n", batteryCurrentNow);
            reply->writeFloat(batteryCurrentNow);
        } break;
        case GET_SUPER_VOOC_STATUS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            reply->writeNoException();
            std::string peojectName = OplusProjectService::getInstance().getProjectString();
            DEBUG("project name: %s\n", peojectName.c_str());
            int status = ChargerService::getInstance().getSuperVOOCStatus(peojectName);
            DEBUG("GET_SUPER_VOOC_STATUS: %d\n", status);
            reply->writeInt32(status);
        } break;
        case GET_BATTERY_FCC: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            reply->writeNoException();
            int batteryFCC = ChargerService::getInstance().getBatteryFCC();
            DEBUG("GET_BATTERY_FCC: %d\n", batteryFCC);
            reply->writeInt32(batteryFCC);
        } break;
        case GET_GPA_SYSTEM_INFO: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<float> value;
                value.push_back(0.0);
                reply->writeFloatVector(value);
                break;
            }

            reply->writeNoException();
            std::vector<float> value;
            value.push_back(HoraeServiceProxy::getInstance().getCurrentThermal());
            reply->writeFloatVector(value);
        } break;
        case SET_COOLEX_FILETER_TYPE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                break;
            }

            int type = data.readInt32();
            std::string config = String8(data.readString16()).string();
            reply->writeNoException();
            CooExxManager::getInstance().setCoolExFilterType(type);
            DEBUG("filter_type: %d\n", type);
            CooExxManager::getInstance().reportCoolExFilterType(type, config);
        } break;
        case SET_GAME_MODE_STATSUS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            DEBUG("set game mode status\n");
            int mode = data.readInt32();
            std::string package = String8(data.readString16()).string();
            reply->writeNoException();
            vector<string> pkg;
            getPackageForUid(uid, pkg);
            for(auto &p : pkg) {
                DEBUG("package name: %s\n", p.c_str());
                if (p == "com.oplus.cosa") {
                    GameListManager::getInstance().setGameMode(mode);
                    GameListManager::getInstance().setCurrentGamePackage(package);
                    break;
                }
            }
        } break;
        case GET_GAME_MODE_STATSUS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeInt32(0);
                break;
            }

            reply->writeNoException();
            int mode = GameListManager::getInstance().getGameMode();
            DEBUG("get game mode status: %d\n", mode);
            reply->writeInt32(mode);
        } break;
        case GET_CURRENT_GAME_PACKAGE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            reply->writeNoException();
            std::string package = GameListManager::getInstance().getCurrentGamePackage();
            DEBUG("get current game package: %s\n", package.c_str());
            reply->writeString16(String16(package.c_str()));
        } break;
        case SET_INSTALLED_GAME_LIST: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            std::vector<String16> list;
            data.readString16Vector(&list);
            DEBUG("game list size: %d\n", list.size());
            reply->writeNoException();
            vector<string> pkg;
            getPackageForUid(uid, pkg);
            for(auto &p : pkg) {
                DEBUG("package name: %s\n", p.c_str());
                if (p == "com.oplus.cosa") {
                    std::vector<std::string> gameList;
                    for(auto &l : list){
                        gameList.push_back(String8(l).string());
                    }
                    GameListManager::getInstance().setInstalledGameList(gameList);
                    break;
                }
            }
        } break;
        case GET_INSTALLED_GAME_LIST: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<String16> list;
                list.push_back(String16(""));
                reply->writeString16Vector(list);
                break;
            }

            std::vector<std::string> gameList;
            GameListManager::getInstance().getInstalledGameList(gameList);
            std::vector<String16> list;
            reply->writeNoException();
            for(auto &l : gameList){
                list.push_back(String16(l.c_str()));
            }
            DEBUG("game list size: %d\n", list.size());
            reply->writeString16Vector(list);
        } break;
        case GET_CPU_TIME_IN_STATE: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            reply->writeNoException();
            std::string state = getTimeInState();
            DEBUG("Get CPU time state\n");
            reply->writeString16(String16(state.c_str()));
        } break;
        case TRIGGER_FRAME_STAT: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            std::string status = String8(data.readString16()).string();
            std::string packageName = String8(data.readString16()).string();
            std::string result;
            reply->writeNoException();
            if (packageName != "") {
                if (!strcmp("start", status.c_str())) {
                    result = SurfaceFlingerProxyManager::getInstance().startFrameStat(packageName);
                } else {
                    struct FrameStats stat;
                    SurfaceFlingerProxyManager::getInstance().getAndStopFrameStat(packageName, stat);
                    result = frameStatsToJson(stat);
                }
            } else {
                result = std::string("");
            }
            DEBUG("Trigger freame stat\n");
            reply->writeString16(String16(result.c_str()));
        } break;
        case GET_CHIP_NAME: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                reply->writeString16(String16(""));
                break;
            }

            reply->writeNoException();
            std::string chipName = PlatformAdaptor::getInstance().getPlatformName();
            DEBUG("Get chip name: %s\n", chipName.c_str());
            reply->writeString16(String16(chipName.c_str()));
        } break;
        case GET_CPU_CLUSTER_INFO: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);

            if (!isGameSafePermissionCheckedOK(pid, uid)) {
                ERROR("No game permission!");
                reply->writeNoException();
                std::vector<int32_t> clusterInfo;
                clusterInfo.push_back(0);
                reply->writeInt32Vector(clusterInfo);
                break;
            }

            reply->writeNoException();
            PlatformAdaptor::CpuInfo cpuInfo = PlatformAdaptor::getInstance().getCpuInfo();
            std::vector<int32_t> clusterInfo;
            DEBUG("coresInCluster size: %d\n", cpuInfo.coresInCluster.size());
            for(int i = 0; i< cpuInfo.coresInCluster.size(); ++i){
                clusterInfo.push_back(int32_t(cpuInfo.coresInCluster[i].size()));
            }
            DEBUG("Get cpu cluster info\n");
            DEBUG("clusterInfo size: %d\n", clusterInfo.size());
            reply->writeInt32Vector(clusterInfo);
        } break;
        case GET_COLORX_STATUS: {
            CHECK_INTERFACE(IOplusOIfaceService, data, reply);
            std::string packageName = String8(data.readString16()).string();
            reply->writeNoException();
            int colorXStatus = OifaceCallbackManager::getInstance().getColorXStatusFromCOSA(packageName);
            DEBUG("Get colorX status: %d\n", colorXStatus);
            reply->writeInt32(colorXStatus);
        } break;
        case NOTIFY_GL_THREADS: {
            CHECK_INTERFACE(IOIfaceService, data, reply);

            int gamePid = data.readInt32();
            int glThread = data.readInt32();

            if (OIfaceServer::getInstance().isClientRunning(gamePid) == false) {
                DEBUG("this process pid %d is not a game process", gamePid);
                return NO_ERROR;
            }

            // foregroundPid just for debug
            int foregroundPid = ThreadState::getInstance().getForgroundPid();
            DEBUG("notify GL thread pid: %d, tid: %d, foreground app pid: %d.", gamePid, glThread, foregroundPid);
            // always set the GL thread, noly check when heavy thread need to set
            TaskManager::getInstance().setGLThread(glThread);
        } break;
        case GET_COOLEX_RUS_DATA: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            std::string packageName = String8(data.readString16()).string();
            reply->writeNoException();
            android::String16 rusData = OifaceCallbackManager::getInstance().getCoolExDataFromCOSA(packageName);
            DEBUG("CoolEx RUS data size: %d\n", rusData.size());
            reply->writeString16(rusData);
        } break;
        case GET_4D_SUPPORT_STATUS: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            std::string packageName = String8(data.readString16()).string();
            reply->writeNoException();
            int supportStatus = OifaceCallbackManager::getInstance().get4DSupportStatusFromCOSA(packageName);
            DEBUG("4D Support Status: %d\n", supportStatus);
            reply->writeInt32(supportStatus);
        } break;
        case GET_DYNAMIC_RESOLUTION_SWITCH: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            std::string packageName = String8(data.readString16()).string();
            reply->writeNoException();
            int switchData = OifaceCallbackManager::getInstance().getDynamicResSwitchFromCOSA(packageName);
            DEBUG("get Dynamic resolution switch : %d\n", switchData);
            reply->writeInt32(switchData);
        } break;
        case DYNAMIC_RESOLUTION_EVENT_TRACE: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_COLLECT_EVENT_DATA;
            msg.uid = 0;
            msg.pid = 0;
            msg.value = BIG_DATA_TYPE_DYNAMIC_RESOLUTION;
            msg.json = String8(data.readString16()).string();
            OIfaceServer::getInstance().sendMessage(msg);
            reply->writeNoException();
        } break;
        case REPPORT_WEAPON_NAME: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            std::string weaponName = String8(data.readString16()).string();
            reply->writeNoException();
            OifaceCallbackManager::getInstance().reportWeaponName(weaponName);
            //DEBUG("Repport weapon name feedback size: %s\n", feedback.size());
            // reply->writeString16(feedback);
        } break;
        case NOTIFY_GET_DATA: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            int switchOnOff = data.readInt32();
            ERROR("chenxinfu NOTIFY_GET_DATA swtich: %d", switchOnOff);
            if (switchOnOff) {
                DataReader::getInstance().start(2000);
            }
            else
                DataReader::getInstance().stop();
        } break;
        default:
            return BBinder::onTransact(code, data, reply, flags);
    }

    return NO_ERROR;
}

void OplusOIfaceService::currentNetwork(int32_t status) {
    oiface_sys_info info;

    info.cmd = OIFACE_REQUEST_CMD_NETWORK;
    info.data.network.status = status;

    ThreadState::getInstance().updateSysInfo(&info);
}

static Json::Value map2JsonValue(map<int32_t, int64_t> &timeInState) {
    Json::Value obj;

    for (map<int32_t, int64_t>::const_iterator iter = timeInState.begin();
            iter != timeInState.end();
            iter++) {
        // jsoncpp bug cannot handle double, workaround
        Json::Int64 temp = iter->second;
        obj[std::to_string(iter->first)] = temp;
    }

    DEBUG("map2JsonValue result: %s", obj.toStyledString().c_str());
    DEBUG("--------------------------------");
    return obj;
}

std::string OplusOIfaceService::getTimeInState() {
    std::map<int32_t, int64_t> timeInState;
    Json::Value root;

    std::string cpu = "cpu";
    for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
        timeInState.clear();
        getCpuTimeInState(cpuMap.second, timeInState);
        root[(cpu + std::to_string(cpuMap.second)).c_str()] = map2JsonValue(timeInState);
    }

    DEBUG("getTimeInState return: %s", root.toStyledString().c_str());
    return root.toStyledString();
}


std::string OplusOIfaceService::generalOIfaceSignal(std::string &signal, int uid) {
    Json::Value root;
    Json::Reader reader;
    DEBUG("generalOIfaceSignal: %s", signal.c_str());

    if (!reader.parse(signal, root) || root.isNull()) {
        ERROR("Parse signal json fail");
        return std::string("");
    }

    string signalType = root.get("signal", "unknown_signal").asString();
    Json::Value result;
    switch(hashCompileTime(signalType.c_str())) {
        case (hashCompileTime("log_level")) :{
            DEBUG("set oiface log level");
            int level = stoi(root.get("level", "-1").asString());
            return setOifaceLogLevel(level, uid);
        } break;
        case (hashCompileTime("time_in_state")):{
            DEBUG("Signal get time_in_state");
            return getTimeInState();
            }break;
        case (hashCompileTime("battery_remain")):{
            DEBUG("Signal get battery_remain");
            std::string battery_remain = "battery_remain";
            int batteryRemain = ChargerService::getInstance().getBatteryRemain();
            result[battery_remain.c_str()] = std::to_string(batteryRemain);
            return result.toStyledString();
            }break;
        case (hashCompileTime("battery_current_now")):{
            DEBUG("Signal get battery_current_now");
            std::string battery_current_now = "battery_current_now";
            float batteryCurrentNow = ChargerService::getInstance().getBatteryCurrentNow();
            result[battery_current_now.c_str()] = std::to_string(batteryCurrentNow);
            return result.toStyledString();
            }break;
        case (hashCompileTime("battery_FCC")):{
            DEBUG("Signal get battery_FCC");
            std::string battery_FCC = "battery_FCC";
            int batteryFCC = ChargerService::getInstance().getBatteryFCC();
            result[battery_FCC.c_str()] = std::to_string(batteryFCC);
            return result.toStyledString();
            }break;
        // case (hashCompileTime("device_temperature")):{
        //     DEBUG("Signal get device_temperature");
        //     std::string device_temperature = "device_temperature";
        //     std::vector<float> devicetemperature;
        //     std::string devicetemperature_string = "";
        //     char temp[20];
        //     if(ThermalService::getInstance().getAllTemperature(devicetemperature) == -1){
        //         ERROR("Get temperature error!\n");
        //         result[device_temperature.c_str()] = std::string("");
        //         return result.toStyledString();
        //         }else{
        //             for(auto &t : devicetemperature){
        //                 memset(temp, '\0', sizeof(temp));
        //                 if(t != devicetemperature[devicetemperature.size() - 1]){
        //                     snprintf(temp, sizeof(temp), "%.2f ", t);
        //                 }else{
        //                     snprintf(temp, sizeof(temp), "%.2f", t);
        //                 }
        //             devicetemperature_string += std::string(temp);
        //         }
        //         DEBUG("Get temperature succeed!\n");
        //         result[device_temperature.c_str()] = devicetemperature_string;
        //         return result.toStyledString();
        //     }
        // } break;
        case (hashCompileTime("current_thermal")):{
            DEBUG("Signal get current_thermal");
            std::string current_thermal = "current_thermal";
            float currentThermal = HoraeServiceProxy::getInstance().getCurrentThermal();
            DEBUG("current_thermal: %f\n", currentThermal);
            result[current_thermal.c_str()] = std::to_string(currentThermal);
            return result.toStyledString();
        } break;
        case (hashCompileTime("coolex_filter_type")):{
            DEBUG("Signal set coolex_filter_type");
            std::string coolex_filter_type = "coolex_filter_type";
            int type = std::atoi(root.get("filter_type", "unknown_type").asCString());
            CooExxManager::getInstance().setCoolExFilterType(type);
            DEBUG("filter_type: %d\n", type);
            CooExxManager::getInstance().reportCoolExFilterType(type, "");
            result[coolex_filter_type.c_str()] = std::to_string(type);
            return result.toStyledString();
        } break;
        case (hashCompileTime("frame_stat")):{
            DEBUG("Start frame stat");
            std::string package = root.get("package", "unknown_package").asString();
            if (package != "unknown_package") {
                if (!strcmp("start", root.get("status", "unknown_status").asCString())) {
                    return SurfaceFlingerProxyManager::getInstance().startFrameStat(package);
                }
                struct FrameStats stat;
                SurfaceFlingerProxyManager::getInstance().getAndStopFrameStat(package, stat);
                return frameStatsToJson(stat);
            } else {
                return std::string("");
            }
        } break;
        case (hashCompileTime("serial_ID")):{
            DEBUG("Signal get serial_ID");
            std::string serial_ID = "serial_ID";
            std::string serialID = OplusProjectService::getInstance().getSerialID();
            DEBUG("serial_ID: %s\n", serialID.c_str());
            result[serial_ID.c_str()] = serialID;
            return result.toStyledString();
        } break;
        case (hashCompileTime("set_open_ID")):{
            DEBUG("Signal set open ID");
            openID = root.get("open_ID", "unknown_open_ID").asString();
            setOpenID(openID);
            DEBUG("open_ID: %s\n", openID.c_str());
            return std::string("");
        } break;
        case (hashCompileTime("set_TargetFps")): {
            int fps = root.get("target_fps", "-1").asInt();
            //setTargetTgpaFps(TFps);
            FrameRescuer::getInstance().setTargetFps(fps);
            DEBUG(" FrameRescuer getTargetFps:  %d", FrameRescuer::getInstance().getTargetFps());
            DEBUG("cosa->oiface Signal set set_TargetFps: %d", fps);
            return std::string("");
        } break;
        case (hashCompileTime("set_FrameAlp")): {
            float frameAlp = root.get("frame_alpla", "-1").asFloat();
            FrameRescuer::getInstance().setFrameAlp(frameAlp);
            DEBUG(" FrameRescuer getFrameAlp:  %f", FrameRescuer::getInstance().getFrameAlp());
            DEBUG("cosa->oiface Signal set set_FrameAlp: %f", frameAlp);
            return std::string("");
        } break;
        case (hashCompileTime("super_VOOC_status")):{
            DEBUG("Signal get super VOOC status");
            std::string super_VOOC_status = "super_VOOC_status";
            std::string peojectName = OplusProjectService::getInstance().getProjectString();
            DEBUG("project name: %s\n", peojectName.c_str());
            int status = ChargerService::getInstance().getSuperVOOCStatus(peojectName);
            DEBUG("GET_SUPER_VOOC_STATUS: %d\n", status);
            result[super_VOOC_status.c_str()] = std::to_string(status);
            return result.toStyledString();
        } break;
        case (hashCompileTime("set_signature")): {
            std::string md5 = root.get("signature", "sgame").asString();
            setSgameSignatureMD5(md5);
            return std::string("");
        } break;
        case (hashCompileTime("touch_adjuster")): {
            DEBUG("Signal get touchAdjuster Support");
            std::string touch_adjuster = "touch_adjuster";
            std::string status = OplusTouchService::getInstance().getTouchAdjusterSupportStatus(0, TOUCH_SMOOTH_LEVEL_NODE);
            DEBUG("touch_adjuster: %s\n", status.c_str());
            result[touch_adjuster.c_str()] = status;
            return result.toStyledString();
        } break;
        case (hashCompileTime("cpu_policy")): {
            DEBUG("Signal get cpu_policy");
            int status = DecisionDriver::getInstance().oifaceDecisionSetTargetLoads(root);
            DEBUG("cpu_policy: %d\n", status);
            return to_string(status);
        } break;
        case (hashCompileTime("fast_start")): {
            DEBUG("Signal get fast_start");

            string package = root.get("swap_pkg", "none").asString();
            int pid = stoi(root.get("swap_pid", "0").asString());
            int swapType = stoi(root.get("swap_type", "-1").asString());
            if (swapType == 1) {
                LightningStartManager::getInstance().setNandSwapIn(pid, package);
                BinderMessage msg;
                msg.what = BINDER_MESSAGE_SET_LIGHTNING_START_PACKAGE;
                msg.uid = 0;
                msg.pid = 0;
                msg.json = package;
                OIfaceServer::getInstance().sendMessage(msg);
            } else if (swapType == 0) {
                DEBUG("generalOIfaceSignal   setLightning swapOut package: %s \n", package.c_str());
                LightningStartManager::getInstance().cancelGetLayerNameTimer();
                LightningStartManager::getInstance().setNandSwapOut(pid, package);
            }
            //int status = 0;//DecisionDriver::getInstance().oifaceDecisionSetTargetLoads(root);
            DEBUG("Signal fast_start  : pid:%d, package:%s , swapType:%d \n", pid, package.c_str(), swapType);
            return std::string("");
        } break;
        case (hashCompileTime("frame_interval")): {
            DEBUG("start to colloect frame interval info");
            int type = std::atoi(root.get("type", "0").asCString());
            std::string packageName = root.get("pkg", "unknown_type").asString();
            float upbound = (float)std::atof(root.get("upbound", "20").asCString());
            float downbound = (float)std::atof(root.get("downbound", "5").asCString());
            if (type == 1) {
                FrameRescuer::getInstance().recordStatu = true;
                DataReader::getInstance().mFrameIntervalCollector.setBound(upbound, downbound);
                FrameRescuer::getInstance().triggerFrameBoostStart(packageName);
            }
            else if (type == 0) {
                FrameRescuer::getInstance().recordStatu = false;
                FrameRescuer::getInstance().triggerFrameInfoStop();
            }
            else if (type == 2) {
                //update bounds
                DataReader::getInstance().mFrameIntervalCollector.setBound(upbound, downbound);
            }
            return std::string("");
        } break;
        case (hashCompileTime("get_file_content")): {
            std::string filePath = root.get("file_path", "invalid file").asString();
            if (filePath != "invalid file") {
                std::string filecontent = getFile(filePath);
                return filecontent;
            } else {
                return std::string("");
            }
        } break;
        default:{
            ERROR("Unknown decision %s", signalType.c_str());
            return std::string("");
        }break;
    }
}

bool OplusOIfaceService::checkOifaceWhiteList(int type, int uid) {
    vector<string> pkgList;

    getPackageForUid(uid, pkgList);
    if (pkgList.size() <= 0) {
        ERROR("unable to get package name");
        return false;
    }

    Json::Value whiteList;
    GlobalConfig::getInstance().getConfig(&whiteList, {"callback_whitelist"});

    if (whiteList.isArray()) {
        for (auto &pkgListItem : pkgList) {
            for (int i = 0; i < (int)whiteList.size(); i++) {
                if ((pkgListItem == std::string(whiteList[i].asCString()))) {
                    return true;
                }
            }
        }
    }

    DEBUG("pkg not found in whitelist");
    for (vector<string>::iterator iter = pkgList.begin();
        iter != pkgList.end();
        iter++) {
        DEBUG(">> %s", iter->c_str());
    }

    DEBUG("Whitelist packages:");
    for (int i = 0; i < (int)whiteList.size(); i++) {
        DEBUG(" >> %s", whiteList[i].asCString());
    }
    return false;
}

void OplusOIfaceService::packageSwitch(const Parcel& data) {
    vector<std::string> splitedStr;
    int32_t  pkgStatus = data.readInt32();
    std::string packageWithPid = String8(data.readString16()).string();
    splitString(packageWithPid, splitedStr, ":");
    std::string package = splitedStr.front();
    int pid = -1;
    if (splitedStr.size() > 1 ){
        pid = atoi(splitedStr.back().c_str());
    }
    INFO("packageSwitch package %s[pid=%d], pkgStatus %d ",package.c_str(), pid, pkgStatus);

    oiface_sys_info info;
    switch (pkgStatus) {
        case OIFACE_APP_ENTER_APP:
        case OIFACE_APP_ENTER_GAME: {
            DEBUG("packageSwitch pkg %s, enter", package.c_str());
        } break;
        case OIFACE_APP_PIP_OPENED:
        case OIFACE_APP_PIP_OPEN:
        case OIFACE_APP_PIP_CLOSED:
        case OIFACE_APP_PIP_CLOSE: {
            std::string zoomPackage = String8(data.readString16()).string();
            DEBUG("packageSwitch package %s, ZoomPkg %s, pip %s",
                  package.c_str(),
                  zoomPackage.c_str(),
                  pkgStatus == OIFACE_APP_PIP_OPEN? "opened":"closed" );
        } break;
        case OIFACE_APP_PIP_SWITCH:
        case OIFACE_APP_PIP_CHANGE: {
            std::string toZoomPkg = String8(data.readString16()).string();
            std::string fromZoomPkg = String8(data.readString16()).string();
            DEBUG("packageSwitch pip switch from %s to %s", fromZoomPkg.c_str(),toZoomPkg.c_str());
        } break;
        default:
            DEBUG("ignore pkgstatus %d", pkgStatus);
    }

    strncpy(info.data.app.app_name, package.c_str(), sizeof(oiface_app_status::app_name) - 1);
    info.data.app.app_name[sizeof(oiface_app_status::app_name) - 1] = '\0';
    info.cmd = OIFACE_REQUEST_CMD_APP_SWITCH;
    info.data.app.status = pkgStatus;
    info.data.app.app_pid = pid;
    if (pkgStatus > 0 && pkgStatus <= OIFACE_APP_PIP_SWITCH) {
        ThreadState::getInstance().updateSysInfo(&info);
    }
}

std::string OplusOIfaceService::setOifaceLogLevel(int level, int uid) {
    bool permission_enable = false;
    ERROR("setOifaceLogLevel %d %d", level, uid);
    if (level > 2 || level < 0)
        return std::string("wrong param");

    if ((uid == AID_SYSTEM) || (uid == AID_ROOT)) {
        permission_enable = true;
    } else {
        std::vector<std::string> packages;
        getPackageForUid(uid, packages);
        if (packages.size() > 0) {
            Json::Value whiteList;
            GlobalConfig::getInstance().getConfig(&whiteList, {"loglevel_whitelist"});
            if (whiteList.isArray()) {
                for (int i = 0; i < (int)whiteList.size(); i++) {
                    if (whiteList[i].asString().compare(packages[0]) == 0) {
                        permission_enable = true;
                        break;
                    }
                }
            }
        }
    }

    if (permission_enable) {
        ERROR("set log level %d", level);
        gLogLevel = level;
    } else {
        return std::string("permission denied");
    }
    return std::string("set Log level OK");
}

void OplusOIfaceService::currentPackage(const android::String16 &package,
        __attribute__((unused)) int32_t uid, __attribute__((unused)) int32_t pid) {
    oiface_sys_info info;
    int len = utf16_to_utf8_length(package.string(), package.size());
    char* utf8 = new char[len + 1];

    utf16_to_utf8(package.string(), package.size(), utf8, len + 1);
    utf8[len] = '\0';
    strncpy(info.data.app.app_name, utf8, sizeof(oiface_app_status::app_name) - 1);
    info.data.app.app_name[sizeof(oiface_app_status::app_name) - 1] = '\0';
    delete[] utf8;

    info.cmd = OIFACE_REQUEST_CMD_APP;
#if defined(USE_OIFACE_SERVICE) && !defined(USE_NEO)
    info.data.app.uid = uid;
    info.data.app.pid = pid;
#endif

    info.data.app.status = 1;
    ThreadState::getInstance().updateSysInfo(&info);
}

int OplusOIfaceService::getSupportPackage(std::string &pkgStat) {
// int OplusOIfaceService::getSupportPackage(binder::Map &pkgStat) {
    pkgStat.clear();

    // FIXME map; done
    Json::Value pkgStatRoot;
    Json::Reader reader;
#if 1
    /* error ignored */
    OIfaceModuleLoader::getSupportPackage(pkgStat);

    if (!reader.parse(pkgStat, pkgStatRoot)) {
        return -1;
    }

    Json::Value v;
    GlobalConfig::getInstance().getConfig(&v, {"sdk"});
    for (auto& iter: v.getMemberNames()) {
        if (v[iter].isObject()) {
            if (v[iter]["enable"].isUInt())
                pkgStatRoot[iter] = v[iter]["enable"].asUInt() ?
                std::string("enabled"):std::string("disabled");
            else
                pkgStatRoot[iter] = std::string("disabled");
        }
    }
#endif

    return 0;
}

int OplusOIfaceService::getSupportGameStartPackage(std::string &pkgStat) {
// int OplusOIfaceService::getSupportGameStartPackage(binder::Map &pkgStat) {

    // FIXME map: done
    Json::Value pkgStatRoot;
#if 1
    Json::Value v;
    GlobalConfig::getInstance().getConfig(&v, {"sdk"});
    for (auto& iter: v.getMemberNames()) {
        if (v[iter].isObject()) {
            if (v[iter]["has_start_signal"].isUInt()) {
                pkgStatRoot[iter] =  std::string("1");
                DEBUG("getSupportGameStartPackage item is %s", iter.c_str());
            }
        }
    }

    Json::Value v1;
    GlobalConfig::getInstance().getConfig(&v1, {"oifacegame"});
    for (auto& iter: v1.getMemberNames()) {
        if (v1[iter].isObject()) {
            if (v1[iter]["has_start_signal"].isUInt()) {
                pkgStatRoot[iter] =  std::string("1");
                DEBUG("getSupportGameStartPackage oifacegame item is %s", iter.c_str());
            }
        }
    }
#endif

    pkgStat.append(pkgStatRoot.toStyledString());

    return 0;
}


bool OplusOIfaceService::isGameSafePermissionCheckedOK(int pid, int uid) {
    if (uid == 1000) {
        return true;
    }

    android::Mutex::Autolock _l(mMutex);
    map<int, bool>::iterator iter = permissionedUidList.find(uid);
    if (iter != permissionedUidList.end()) {
        //ERROR("GameSafe permission CheckedOK uid %d has already checked and ok", uid);
        return iter->second;
    } else {
        if (checkPermission(String16(OIFACE_PERMISSION), pid, uid)) {
            //ERROR("GameSafe permission uid %d checked ok for first time", uid);
            permissionedUidList.insert(pair<int, bool>(uid, true));
            return true;
        } else {
            ERROR("GameSafe permission uid %d checked faild for first time", uid);
            permissionedUidList.insert(pair<int, bool>(uid, false));
            return false;
        }
    }
    return false;
}


android::String16 BpOIfaceCallback::onFBNotification(int status) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;

    DEBUG("onFBNotification called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeInt32(status);

    rv = remote()->transact(FB_NOTIFICATION, data, NULL, 0);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return android::String16("");
    }

    return android::String16("0");
}

android::String16 BpOIfaceCallback::onGPANotification(const std::string &info) {
    android::Parcel data, reply;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onGPANotification called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(info.c_str()));

    rv = remote()->transact(GPA_NOTIFICATION, data, &reply, 0);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return android::String16("");
    }
    int32_t err = reply.readExceptionCode();
    if (err < 0) {
        ERROR("%s: remote exception: %d", __func__, err);
        return android::String16("");;
    }
    return reply.readString16();
}

int BpOIfaceCallback::onTGPAInfo(const std::string &info, int uid, int pid) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(info.c_str()));
    data.writeInt32(uid);
    data.writeInt32(pid);

    rv = remote()->transact(TGPA_INFO, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onHyperBoostInfo(const std::string &info, int uid, int pid) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onHyperBoostInfo called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(info.c_str()));
    data.writeInt32(uid);
    data.writeInt32(pid);

    rv = remote()->transact(HYPERBOOST_INFO, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onEngineBoostINfo(const std::string &info, int uid, int pid) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onEngineBoostInfo called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(info.c_str()));
    data.writeInt32(uid);
    data.writeInt32(pid);

    rv = remote()->transact(ENGINEBOOST_INFO, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onOifaceGeneralInfo(const std::string &info, int type, int uid, int pid) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onOifaceGeneralInfo called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(info.c_str()));
    data.writeInt32(type);
    data.writeInt32(uid);
    data.writeInt32(pid);

    rv = remote()->transact(OIFACE_GENERAL_INFO, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onGameStatusChanged(const std::string &packageName,
                                               const std::string &gamestat) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onGameStatusChanged called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(packageName.c_str()));
    data.writeString16(String16(gamestat.c_str()));

    rv = remote()->transact(GAME_STATUS_CHANGED, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onNetworkChanged(const std::string &packageName, int latency) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onNetworkChanged called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(packageName.c_str()));
    data.writeInt32(latency);

    rv = remote()->transact(NETWORK_CHANGED, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onSystemNotify(const std::string &result) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onSystemNotify called: %s", result.c_str());

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(result.c_str()));

    rv = remote()->transact(SYSTEM_NOTIFY, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onThermalStatusChanged(const std::string &status) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onThermalStatusChanged called: %s", status.c_str());

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(status.c_str()));

    rv = remote()->transact(THERMAL_STATUS_CHANGED, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpOIfaceCallback::onGameJitter(const std::string &packageName, int fps) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;
    DEBUG("onGameJitter called");

    data.writeInterfaceToken(BpOIfaceCallback::getInterfaceDescriptor());
    data.writeString16(String16(packageName.c_str()));
    data.writeInt32(fps);

    rv = remote()->transact(GAME_JITTER, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

bool BpCoolExCallback::onReportCoolExFilterType(int type, std::string &config) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;

    DEBUG("onReportCoolExFilterType called: %d", type);

    data.writeInterfaceToken(BpCoolExCallback::getInterfaceDescriptor());
    data.writeInt32(type);
    data.writeString16(String16(config.c_str()));

    rv = remote()->transact(REPORT_COOLEX_FILTER_TYPE, data, NULL, 0);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return false;
    }
    return true;
}


int BpOIfaceNetworkNotifier::onNetworkNotify(const string& pkgName, int32_t latency) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;

    data.writeInterfaceToken(BpOIfaceNetworkNotifier::getInterfaceDescriptor());
    data.writeString16(String16(pkgName.c_str()));
    data.writeInt32(latency);

    rv = remote()->transact(NETWORK_NOTIFY, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}

int BpGameStatusNotifier::onGameStatusNotify(const std::string& pkgName,
                                                  const std::string &gameStats) {
    android::Parcel data;
    android::status_t rv = android::NO_ERROR;

    data.writeInterfaceToken(BpGameStatusNotifier::getInterfaceDescriptor());
    data.writeString16(String16(pkgName.c_str()));

    // FIXME map: write gameStats here; done
    // data.writeMap(gameStats);
    data.writeString16(String16(gameStats.c_str()));

    rv = remote()->transact(GAME_STATUS_NOTIFY, data, NULL, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ERROR("transact to remote binder failed(%d)", rv);
        return -1;
    }

    return 0;
}
