#include <private/android_filesystem_config.h>
#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>
#include <cutils/properties.h>
#include <string.h>

#include "OIface.h"
#include "BinderMessage.h"
#include "BinderManager.h"
#include "OIfaceServer.h"
#include "Utils.h"
#include "GlobalConfig.h"
#include "ThreadState.h"
#include "comp.h"
#include "DataCollector.h"
#include "AffinityService.h"
#include "GeneralCheck.h"
#include "ThermalService.h"
#include "DecisionDriver.h"
#include "OifaceCallbackManager.h"
#include "ChargerService.h"
#include "SurfaceFlingerProxyManager.h"
#include "unistd.h"
#include "TaskManager.h"

using namespace std;
using namespace android;

/* called in binder thread */
void BinderDeathRecipient::binderDied(const wp<IBinder>& who) {
    DEBUG("binder died called");

    BinderMessage msg;
    msg.what = BINDER_MESSAGE_BINDER_DIED;
    msg.uid = mUid;

    OIfaceServer::getInstance().sendMessage(msg);
}

BinderHandler::BinderHandler(const BinderMessage& msg): mMsg(msg) {
    //DEBUG("binder handler constructor for message:%d called", msg.what);
}

/* msg unused */
void BinderHandler::handleMessage(const Message& /* msg */) {
    OIfaceServer::getInstance().handleMessage(mMsg);
}

BinderHandler::~BinderHandler() {
    //DEBUG("binder handler destructor for message:%d called", mMsg.what);
}

BinderManager::BinderManager():
    mConnectionLessClient(new ConnectionLessClient(OIFACE_CONNECTION_LESS)),
    mLastBatteryRemain(-1), mLastStartTime(-1) {
    DEBUG("binder manager constructor called");
    mForgroundCpus = getSystemFile(CPUSET_FG_CPUS);
    Json::Value v;
    if (GlobalConfig::getInstance().getConfig(&v, {"sdk", "connectionless"}) < 0) {
        DEBUG("no connectionless clients");
        return;
    }

    DEBUG("processing members");
    for (auto &iter:v.getMemberNames()) {
        DEBUG("processing member %s", iter.c_str());
        if (v[iter].isObject()) {
            if (v[iter]["enable"].isUInt() && (v[iter]["enable"].asUInt() > 0)) {
                /* package enabled */
                DEBUG("package %s enabled", iter.c_str());
            } else {
                DEBUG("package %s disabled", iter.c_str());
                continue;
            }

            /* parse rules */
            Json::Value &rules(v[iter]["rules"]);

            if (rules.isNull() || (!rules.isString())) {
                DEBUG("rules config value for %s is invalid", iter.c_str());
                continue;
            }

            /*
            vector<nodeType*> syntaxTree;
            if (compileProgram(rules.asString().c_str(), &syntaxTree) < 0) {
                ERROR("compile program for %s failed", iter.c_str());
                continue;
            }
            mSyntaxTreeList[iter] = syntaxTree;
            */
        }
    }

    /* TODO: handle screen off case */
    string forgroundPackage = ThreadState::getInstance().getForgroundPackage();
    if (forgroundPackage.size() > 0) {
        /* Initialize statistics */
        for (int i = 0; i < MAX_CPU_CORES; i++)
            mIdleTime[i] = getCpuIdleTime(i);

        for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
            DEBUG("cluster:%d cpu:%d", cpuMap.first, cpuMap.second);
            getCpuTimeInState(cpuMap.second, mTimeInState[cpuMap.first]);
        }

        // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
        //         string("s_") + forgroundPackage, "start_time",
        //         (int32_t)(systemTime(SYSTEM_TIME_REALTIME) / 1000000000LL));
    }

    getGpuStats(mGpuTimeInState);
}

/* all binder messages are handled here */
void BinderManager::handleMessage(const BinderMessage& msg) {
    DEBUG("handling message(what:%d uid:%d pid:%d json:%s)", msg.what, msg.uid, msg.pid,
            msg.json.c_str());
    switch (msg.what) {
        case BINDER_MESSAGE_ENGINE_CLIENT:
        case BINDER_MESSAGE_NEW_CLIENT: {
            createNewClient(msg);
        } break;

        case BINDER_MESSAGE_UPDATE_GAME_INFO:
        case BINDER_MESSAGE_UPDATE_ENGINE_INFO: {
            // whether the last one is none register client, if so, kill it.
            if (mCurrentUid != 0 && mCurrentUid != msg.uid) {
                map<int, sp<BinderClient>>::iterator itr = mBinderClients.find(mCurrentUid);
                if (itr != mBinderClients.end()) {
                    Json::Value statusJson;
                    statusJson[CONNECT_STATUS_KEY] = CLIENT_DIED;
                    OifaceCallbackManager::getInstance().reportTGPAInfo(mCurrentUid, itr->second->getPid(), statusJson.toStyledString());
                    mBinderClients.erase(mCurrentUid);
                }
                mCurrentUid = 0;
            }

            map<int, sp<BinderClient>>::iterator iter = mBinderClients.find(msg.uid);
            if (iter != mBinderClients.end()) {
                iter->second->handleEvent(msg);
                break;
            }
            int isAuthOff = OIfaceServer::getInstance().getAuthOff();
            DEBUG("client(uid:%d pid:%d) is not registered, is auth off: %d", msg.uid, msg.pid, isAuthOff);
            if (isAuthOff == 1 && createNewClient(msg) == 0) {
                iter = mBinderClients.find(msg.uid);
                if (iter != mBinderClients.end()) {
                    iter->second->handleEvent(msg);
                    break;
                }
            }
        } break;

        case BINDER_MESSAGE_REQUEST_RESOURCE: {
            map<int, sp<BinderClient>>::iterator iter = mBinderClients.find(msg.uid);
            if (iter == mBinderClients.end()) {
                ERROR("client(uid:%d pid:%d) is not registered", msg.uid, msg.pid);
                break;
            }
            iter->second->handleEvent(msg);
        } break;

        case BINDER_MESSAGE_BINDER_DIED: {
            DEBUG("received binder died message from %d", msg.uid);
            map<int, sp<BinderClient>>::iterator iter = mBinderClients.find(msg.uid);
            if (iter == mBinderClients.end()) {
                ERROR("unable to find binder client record for uid:%d", msg.uid);
            } else {
                /* restore state before instance is destoryed */
                DecisionDriver::getInstance().updateDecisionState(iter->second->getClientName(), false);

                Json::Value statusJson;
                statusJson[CONNECT_STATUS_KEY] = CLIENT_DIED;
                OifaceCallbackManager::getInstance().reportTGPAInfo(msg.uid, msg.pid, statusJson.toStyledString());
                mBinderClients.erase(iter);
                GeneralCheck::getInstance().removeConfig(msg.uid);
            }

            map<int, sp<BinderDeathRecipient>>::iterator diter = mDeathRecipients.find(msg.uid);
            if (diter == mDeathRecipients.end()) {
                ERROR("unable to find client record for uid:%d", msg.uid);
                break;
            }

            mDeathRecipients.erase(diter);
        } break;

        case BINDER_MESSAGE_REGISTER_NOTIFIER: {
            DEBUG("regsiter notifier received(uid:%d pid:%d)", msg.uid, msg.pid);
            map<int, sp<BinderClient>>::iterator iter = mBinderClients.find(msg.uid);
            if (iter == mBinderClients.end()) {
                ERROR("unable to find binder client record for uid:%d", msg.uid);
            } else {
                iter->second->enableNotifier(true);
            }
        } break;

        case BINDER_MESSAGE_RUN_PROGRAM: {
            DEBUG("running program");

            ATRACE_BEGIN("running program");
            vector<nodeType*> syntaxTree;
            map<string, int> pre1, pre2, output;

            if (compileProgram(msg.json.c_str(), &syntaxTree) == 0) {
                string pkg = ThreadState::getInstance().getForgroundPackage();
                runProgram(OIfaceServer::getInstance().getClientName(pkg).c_str(),
                        syntaxTree, pre1, pre2, &output);
            } else {
                ERROR("compile program failed\n");
            }
            destroyProgram(syntaxTree);
            ATRACE_END();

        } break;

        /* msg.json is package name */
        case BINDER_MESSAGE_FORGROUND:
        case BINDER_MESSAGE_BACKGROUND:{
            const char *event = msg.what == BINDER_MESSAGE_FORGROUND ? "fg" : "bg";
            // int val = msg.what == BINDER_MESSAGE_FORGROUND ? 1:0;
            DEBUG("received %s %s event", msg.json.c_str(), event);

            /* handle background/forground message */
            if (msg.what == BINDER_MESSAGE_FORGROUND) {
                handleForground(msg.json);
            } else {
                handleBackground(msg.json);
            }

            /* handle background/forground message if client exist */
            for (auto &iter: mBinderClients) {
                if (msg.json == iter.second->getPackageName()) {
                    iter.second->handleEvent(msg);
                }
            }

        } break;
        case BINDER_MESSAGE_FPS_CHANGE:{
            int val = msg.value;
            /* handle connectionless rules */
            auto iter = mSyntaxTreeList.find(msg.json);
            if (iter != mSyntaxTreeList.end()) {
                ATRACE_BEGIN("running program");
                mConnectionLessClient->setPackageName(msg.json);
                runProgram(OIFACE_CONNECTION_LESS, iter->second, mStateList,
                        {{"fps_change", val}}, &mGlobalVariable[msg.json]);
                ATRACE_END();
                mStateList["fps_change"] = val;
            }
            for (auto &iter: mBinderClients) {
                if (msg.json == iter.second->getPackageName()) {
                    iter.second->handleEvent(msg);
                }
            }
        } break;
        /* msg.json is {"key":value} */
        case BINDER_MESSAGE_HYPNUS_POLL_EVENT: {
            Json::Reader reader;
            Json::Value root;
            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed");
                break;
            }

            string pkgName = mConnectionLessClient->getPackageName();
            auto syntax = mSyntaxTreeList.find(pkgName);

            /* only handle integer value */
            for (auto &member: root.getMemberNames()) {
                if (root[member].isInt()) {
                    /* only handle forground connectionless client */
                    if (syntax != mSyntaxTreeList.end()) {
                        DEBUG("connectionless received hypnus poll message:%s:%d",
                                member.c_str(), root[member].asInt());

                        ATRACE_BEGIN("running program");
                        runProgram(OIFACE_CONNECTION_LESS, syntax->second, mStateList,
                                {{member, root[member].asInt()}}, &mGlobalVariable[pkgName]);
                        ATRACE_END();
                    }
                    mStateList[member] = root[member].asInt();

                    /* handle oiface client */
                    for (auto &iter: mBinderClients) {
                        iter.second->handleEvent(msg);
                    }
                }
            }
        } break;

        case BINDER_MESSAGE_NOTIFY_CALLBACK: {
            notifyCallback(msg.json);
        } break;

        case BINDER_MESSAGE_SCREEN_ON: {
            collectionStop("oplus.screen.off", CLUSTER_NUM, MAX_CPU_CORES,
                    mScreenOffTimeInState, mScreenOffGpuTimeInState, mScreeOffIdleTime);
        } break;
        case BINDER_MESSAGE_SCREEN_OFF: {
            collectionStart("oplus.screen.off", CLUSTER_NUM, MAX_CPU_CORES,
                    mScreenOffTimeInState, mScreenOffGpuTimeInState, mScreeOffIdleTime);
        } break;
        case BINDER_MESSAGE_NOTIFY_TEMP_LEVEL:{
            TemperatureLevelData tempLevelData = msg.data.tempLevelData;
            int val = handleTemperatureNotify(tempLevelData);
            string notifyStr = "{\"4\":\"" + to_string(val) + "\"}";
            INFO("NOTIFY_TEMP_LEVEL: %s",notifyStr.c_str());
            for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();iter != mBinderClients.end(); iter++) {
                if (iter->second->getTGPANotifyEnable()) {
                    INFO("NOTIFY_TEMP_LEVEL Notify to package %s", iter->second->getPackageName().c_str());
                    iter->second->notifyCallback(notifyStr);
                } else {
                    INFO("NOTIFY_TEMP_LEVEL TGPA notify %s disabled", iter->second->getPackageName().c_str());
                }
            }
        } break;
        case BINDER_MESSAGE_NOTIFY_THERMAL_STATUS:{
            int val = msg.data.thermalStatusCode;
            string notifyStr = "{\"4\":\"" + to_string(val) + "\"}";
            INFO("NOTIFY_THERMAL_LEVEL: %s",notifyStr.c_str());
            for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin(); iter != mBinderClients.end(); iter++) {
                if (iter->second->getTGPANotifyEnable()) {
                    INFO("NOTIFY_THERMAL_LEVEL Notify to package %s", iter->second->getPackageName().c_str());
                    iter->second->notifyCallback(notifyStr);
                } else {
                    INFO("NOTIFY_THERMAL_LEVEL TGPA notify %s disabled", iter->second->getPackageName().c_str());
                }
            }
        } break;
        default:
            break;
    }
}

void BinderManager::notifyCallback(const string& str) {
    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        iter->second->notifyCallback(str);
    }
}

sp<OIfaceClient> BinderManager::asOIfaceClient(const char *clientName) {
    if(strcmp(clientName, OIFACE_CONNECTION_LESS) == 0) {
        return getConnectionLessClient();
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getClientName() == clientName)
            return iter->second;
    }
    return NULL;
}

sp<OIfaceClient> BinderManager::asOIfaceClientWithLayerName(const string& layerName) {
    if (getConnectionLessClient() != NULL &&
            getConnectionLessClient()->getLayerName().find(layerName) != string::npos) {
        return getConnectionLessClient();
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getLayerName().find(layerName) != string::npos)
            return iter->second;
    }

    return NULL;
}

string BinderManager::getClientName(const string& packageName) {
    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getPackageName() == packageName)
            return iter->second->getClientName();
    }
    if (getConnectionLessClient()->getPackageName() == packageName)
        return getConnectionLessClient()->getClientName();

    return "";
}

bool BinderManager::isClientRunning(const pid_t pid) {
    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getPid() == pid)
            return true;
    }

    return false;
}

void BinderManager::setHeavytask(const pid_t pid, const int tid, const int normalizedTime) {
    if (pid <= 0 || tid <= 0 || normalizedTime <= 0) {
        DEBUG("the client pid %d, tid %d, normalizedTime %d is invalid.", pid, tid, normalizedTime);
        return;
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("binder client pid %d, set task tid %d, load %d.", pid, tid, normalizedTime);
            iter->second->setHeavyTaskLoad(tid, normalizedTime);
            return;
        }
    }
}

void BinderManager::clearHeavyTask(pid_t pid) {
    if (pid <= 0) {
        DEBUG("the client pid %d is invalid.", pid);
        return;
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("clear binder client pid %d vip thread.", pid);
            std::map<int, int>& keyTask = TaskManager::getInstance().getKeyTaskInfo();
            if (keyTask.size() > 0) {
                for (auto itr = keyTask.begin(); itr != keyTask.end(); itr++) {
                    DecisionDriver::getInstance().setKeyTask(pid, itr->first, 0);
                }
                TaskManager::getInstance().clearAll();
                iter->second->clearHeavyTaskLoad();
            }
            return;
        }
    }
}

 void BinderManager::getHeavyTask(pid_t pid, map<int, int>& outTaskLoad) {
    if (pid <= 0) {
        DEBUG("the client pid %d is invalid.", pid);
        return;
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("get binder client pid %d task load info.", pid);
            iter->second->getHeavyTaskLoad(outTaskLoad);
            return;
        }
    }
}

int BinderManager::getClientList(vector<sp<OIfaceClient>> *list) {
    if (list == NULL) {
        DEBUG("invalid argument passed to get client list");
        return -1;
    }
    for (auto &iter: mBinderClients) {
        list->push_back(iter.second);
    }
    return 0;
}

bool BinderManager::hasAnyClientStartedFrameBoost() {
    if (getConnectionLessClient() != NULL &&
            getConnectionLessClient()->isFrameBooted()) {
        return true;
    }

    for (map<int, sp<BinderClient>>::iterator iter = mBinderClients.begin();
            iter != mBinderClients.end(); iter++) {
        if (iter->second->isFrameBooted())
            return true;
    }

    return false;
}

void BinderManager::collectionStart(const std::string& loggerName, int clusterCount, int cpuCoreCount,
        std::map<int32_t, int64_t> *cpuTimeInState, std::map<int32_t, int32_t> &gpuTimeInState,
        int64_t *idleTime) {

    /* Initialize statistics */
    for (int i = 0; i < cpuCoreCount; i++)
        idleTime[i] = getCpuIdleTime(i);

    for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
        DEBUG("cluster:%d cpu:%d", cpuMap.first, cpuMap.second);
        getCpuTimeInState(cpuMap.second, cpuTimeInState[cpuMap.first]);
    }

    getGpuStats(gpuTimeInState);

    /* skin temp begin */
    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
    //         loggerName, "skin_temp_begin",
    //         ThermalService::getInstance().getSkinTemperature());

    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL, loggerName,
    //         "start_time", (int32_t)(systemTime(SYSTEM_TIME_REALTIME)/ 1000000000LL));
}

void BinderManager::handleForground(const std::string& pkgName) {
    DEBUG("forground package is %s", pkgName.c_str());
    handleCpuSet(pkgName, 1);
    collectionStart(string("s_") + pkgName, CLUSTER_NUM, MAX_CPU_CORES,
            mTimeInState, mGpuTimeInState, mIdleTime);
}

void BinderManager::handleBackground(const std::string& pkgName) {
    handleCpuSet(pkgName, 0);
    collectionStop(string("s_") + pkgName, CLUSTER_NUM, MAX_CPU_CORES,
            mTimeInState, mGpuTimeInState, mIdleTime);
}

void BinderManager::handleCpuSet(const std::string& pkgName, int foreground) {
    uint32_t enable = 0;
    GlobalConfig::getInstance().getConfig(&enable, {"foreground", "enable"});
    if(!enable) {
        DEBUG("foreground config is disabled");
        return;
    }
    bool found = false;
    Json::Value whiteList;
    GlobalConfig::getInstance().getConfig(&whiteList, {"foreground", "pkg_list"});
    if (whiteList.isArray()) {
        for (int i = 0; i < (int)whiteList.size(); i++) {
            if(whiteList[i].asString().compare(pkgName) == 0){
                found = true;
                DEBUG("find ground: %d package name: %s", foreground, pkgName.c_str());
                break;
            }
        }
    }
    if(!found) {
        DEBUG("can not find package name:%s", pkgName.c_str());
        return;
    }
    if(!foreground) {
        DEBUG("set packageName:%s, cpus is ;%s", pkgName.c_str(), mForgroundCpus.c_str());
        writeSystemFile(CPUSET_FG_CPUS, mForgroundCpus);
    } else {
        std::string cpus;
        if (GlobalConfig::getInstance().getConfig(&cpus, {"foreground", "cpus"}) == 0) {
            if(!cpus.empty()) {
                DEBUG("set packageName:%s, cpus is ;%s", pkgName.c_str(), cpus.c_str());
            } else {
                DEBUG("cpus is not set!");
                return;
            }
        }
        writeSystemFile(CPUSET_FG_CPUS, cpus);
    }
}

int BinderManager::handleTemperatureNotify(TemperatureLevelData tempLevelData) {
    map<int, vector<int> > cpuMap = AffinityService::getInstance().getClusterFreq();
    for (int i=3; i>=0; i--) {/*only check the biggest cluster*/
        map <int, vector<int> > :: iterator itf;
        itf = cpuMap.find(i);
        if(itf != cpuMap.end()) {
            vector<int> freqTable = itf->second;
            int maxFreq = freqTable.back();
            float levelRate = (tempLevelData.c_freq[i] + 0.01)/maxFreq;
            DEBUG("cluster %d freq[%d]: MaxFreq[%d]freqTableSize:%ld levelRate:%f; ", i, tempLevelData.c_freq[i],
                        maxFreq, freqTable.size(), levelRate);
            if (levelRate >= 0.95)
                return 0;
            else if (levelRate >= 0.65)
                return 1;
            else
                return 2;
        }
    }
    return 0;
}

void BinderManager::collectionStop(const std::string& loggerName, int clusterCount, int cpuCoreCount,
        std::map<int32_t, int64_t> *lastCpuTimeInState, std::map<int32_t, int32_t> &lastGpuTimeInState,
        int64_t *lastIdleTime) {
    char keyName[64];

    for (int i = 0; i < cpuCoreCount; i++) {
        int64_t idleTime;

        idleTime = getCpuIdleTime(i);

        // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
        //         loggerName, "end_time", (int32_t)(systemTime(SYSTEM_TIME_REALTIME) / 1000000000LL));

        snprintf(keyName, sizeof(keyName), "cpu%d_idle", i);
        // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
        //         loggerName, keyName, (float)(idleTime - lastIdleTime[i]));
    }

    for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
        map<int32_t, int64_t> timeInState[CLUSTER_NUM];
        map<int32_t, int64_t> deltaTimeInState;
        DEBUG("cluster:%d cpu:%d", cpuMap.first, cpuMap.second);
        getCpuTimeInState(cpuMap.second, timeInState[cpuMap.first]);

        for (auto &oneState: timeInState[cpuMap.first]) {
            snprintf(keyName, sizeof(keyName), "cpu%d_freq_%d", cpuMap.second, oneState.first);

            deltaTimeInState[oneState.first] = oneState.second - mTimeInState[cpuMap.first][oneState.first];

            // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
            //         loggerName, keyName, (float)((oneState.second - lastCpuTimeInState[cpuMap.first][oneState.first]) *
            //             1000 / sysconf(_SC_CLK_TCK)));
        }
        timeInStateToAverageFreq(deltaTimeInState);
    }

    map<int32_t, int32_t> gpuStat;
    getGpuStats(gpuStat);

    if ((gpuStat.size() > 0) && (gpuStat.size() == lastGpuTimeInState.size())) {
        for (auto &stat: lastGpuTimeInState) {
            snprintf(keyName, sizeof(keyName), "gpu_freq_%d", stat.first);
            // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
            //         loggerName, keyName, (float)(gpuStat[stat.first] - stat.second));
        }
    }

    /* Battery Level and avg current*/
    int32_t nowRemain = ChargerService::getInstance().getBatteryRemain();
    int64_t now = systemTime();
    // if (mLastBatteryRemain != -1 && nowRemain != -1) {
    //     double timeInterval = (double)(now - mLastStartTime);
    //     if (mLastBatteryRemain > 0 && (timeInterval > 30000000000LL)) {
    //         double current = (double)(mLastBatteryRemain - nowRemain) * 3600000000000LL /
    //             (double)(now - mLastStartTime);

    //         if ((nowRemain > 0) && (mLastBatteryRemain - nowRemain >= 0)) {
    //             DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL, loggerName, "current_app", current);
    //         }
    //     }
    // }

    /* skin temp end */
    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
    //         loggerName, "skin_temp_end",
    //         ThermalService::getInstance().getSkinTemperature());

    mLastBatteryRemain = nowRemain;
    mLastStartTime = now;

    // DataCollector::getInstance().finishOne(DataCollector::DC_TYPE_INTERNAL, loggerName);

}

/* handle connectionless rules */
void BinderManager::updateConnectionLessClient(const std::string& package, int value) {
    DEBUG("update ConnectionLess Client: %s, value: %d", package.c_str(), value);
    auto iter = mSyntaxTreeList.find(package);
    if (iter != mSyntaxTreeList.end()) {
        DEBUG("find package to update");
        ATRACE_BEGIN("running program");
        mConnectionLessClient->setPackageName(package);
        runProgram(OIFACE_CONNECTION_LESS, iter->second, mStateList,
            {{"fg", value}}, &mGlobalVariable[package]);
        ATRACE_END();
        mStateList["fg"] = value;
    }
}

int BinderManager::createNewClient(const BinderMessage& msg) {
    struct GeneralConfig config;
    uint32_t general = -1;
    vector<string> pkg;
    if (getPackageForUid(msg.uid, pkg) <= 0) {
        ERROR("get package for uid %d failed", msg.uid);
        return -1;
    }
    if (!isAppDebuggable(String16(pkg[0].c_str()))) {
        uint32_t enable = 0;
        GlobalConfig::getInstance().getConfig(&enable, {"sdk", pkg[0].c_str(), "enable"});
        ThreadState::getInstance().setConfig(msg.uid, enable);
        if (enable == 0) {
            //can not find packageName in whiteList, start to check general whiteList
            GlobalConfig::getInstance().getConfig(&enable, {"general", pkg[0].c_str(), "enable"});
            if(enable != 0) {
                //get general client type in whiteList
                GlobalConfig::getInstance().getConfig(&general, {"general", pkg[0].c_str(), "type"});
                config.general_type = general;
                DEBUG("General client package name :%s, type: %d, is defined in white list", pkg[0].c_str(), (int)general);
            } else {
                //can not find packageName in general whiteList, start to check permission
                if(GeneralCheck::getInstance().getConfig(msg.uid, config) == -1) {
                    ERROR("PermissionID %s is error, please confirm!", msg.json.c_str());
                } else {
                    enable = 1;
                }
                // disable engine client for battery consumer risk
                //if(msg.what == BINDER_MESSAGE_ENGINE_CLIENT) {
                    //DEBUG("Engine client connect ,default to permitted");
                    //config.general_type = 2;
                    //enable = 1;
                //}
            }
            ThreadState::getInstance().setConfig(msg.uid, enable);
            if (enable == 0) {
                ERROR("package name %s is not supported", pkg[0].c_str());
                return -1;
            }
        }
    }

    map<int, sp<BinderClient>>::iterator iter = mBinderClients.find(msg.uid);
    if (iter != mBinderClients.end()) {
        ERROR("binder client(name:%s uid:%d pid:%d) already added",
                iter->second->getClientName().c_str(),
                iter->second->getUid(), iter->second->getPid());
        return 0;
    }
    sp<BinderClient> client;
    if(config.general_type != -1) {
        client = new GeneralClient(msg.uid, msg.pid, config);
    } else {
        client = new BinderClient(msg.uid, msg.pid);
        Json::Value statusJson;
        statusJson[CONNECT_STATUS_KEY] = CLIENT_CONNECTED;
        OifaceCallbackManager::getInstance().reportTGPAInfo(msg.uid, msg.pid, statusJson.toStyledString());
    }
    if (msg.binder != nullptr) {
        if (client->registerNotifier(msg.binder) < 0) {
            return -1;
        }
        sp<BinderDeathRecipient> recipient = new BinderDeathRecipient(msg.uid);
        msg.binder->linkToDeath(recipient);
        mDeathRecipients[msg.uid] = recipient;
    } else {
        // none register client
        mCurrentUid = msg.uid;
    }

    mBinderClients[msg.uid] = client;
    return 0;
}
