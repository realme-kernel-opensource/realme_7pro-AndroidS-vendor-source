#include <sys/socket.h>
#include <sys/un.h>
#include <vector>
#include <thread>
#include <time.h>
#include <chrono>

#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <cutils/properties.h>
#include <utils/Trace.h>

#include <utils/Unicode.h>
#include <utils/RefBase.h>
#include <binder/IServiceManager.h>

#include "OIfaceClient.h"
#include "Utils.h"
#include "GlobalConfig.h"
#include "DataCollector.h"
#include "ThreadState.h"
#include "PlatformAdaptor.h"
#include "AffinityService.h"
#include "ThermalService.h"
#include "DecisionDriver.h"
#include "ChargerService.h"

#define FRAME_BOOST_DEFAULT_ACTION  { .type = DECISION_TYPE_ACTION, .data = { 0 } }

#define RETURN_ON_ERROR(X) \
    do { \
        status_t res = (X); \
        if (res != NO_ERROR) \
            return -1; \
    } while(false)

using namespace std;
using namespace android;

FrameRescuer* OIfaceClient::mFrameRescuer = NULL;

OIfaceClient::OIfaceClient(const std::string &clientName, int type,
        int uid /* =-1 */, int pid /* =-1 */, int fd /* = -1 */): mClientName(clientName) {
    mType = type;
    mUid = uid;
    mPid = pid;
    mFd = fd;
    mPackageName = "client.fake.package";
    mTriggerTime = 0;
    mFrameCount = 0;
    mInvalidated = false;
    mTriggerStarted = false;
    mBatteryRemain = 0;
    mLmhCnt0 = -1;
    mLmhCnt1 = -1;
    mLoadingStart = 0;
    mAppSceneId = 0;
    mAppEventStartNanoSec = 0;
    mFbAction = FRAME_BOOST_DEFAULT_ACTION;
    mFrameBoosted = false;
    mEnableTGPANotify = 1;
    mTemperature.resize(ThermalService::TEMPERATURE_COUNT, -1);
    if (uid > 0) {
        std::vector<std::string> packages;
        getPackageForUid(uid, packages);
        if (packages.size() > 0)
            mPackageName = packages[0];
        DEBUG("uid:%d package_size:%d", uid, (int)(packages.size()));
    } else {
        DEBUG("an invalid uid:%d pid:%d passed to OIfaceClient", uid, pid);
    }
    mUseFrameStats = 0;
    setLayerName();
    if (mFrameRescuer == NULL) {
        mFrameRescuer = &FrameRescuer::getInstance();
    }
}

int OIfaceClient::getUid(int fd) {
    struct ucred ucred;
    socklen_t len;

    len = sizeof(struct ucred);
    if (getsockopt(fd, SOL_SOCKET,SO_PEERCRED, &ucred,
                &len) == -1) {
        ERROR("getsockopt failed(%s)\n",
                strerror(errno));
        return -1;
    }

    INFO("pid:%d uid:%d gid:%d\n",
            ucred.pid, ucred.uid,
            ucred.gid);

    return ucred.uid;
}

int OIfaceClient::getPid(int fd) {
    struct ucred ucred;
    socklen_t len;

    len = sizeof(struct ucred);
    if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &ucred,
                &len) == -1) {
        ERROR("getsockopt failed(%s)\n",
                strerror(errno));
        return -1;
    }

    INFO("pid:%d uid:%d gid:%d\n",
            ucred.pid, ucred.uid,
            ucred.gid);

    return ucred.pid;
}

Json::Value OIfaceClient::getState(const std::string& key) {
    return mState.get(key, Json::nullValue);
}

Json::Value OIfaceClient::getState() {
    return mState;
}

void OIfaceClient::setState(const std::string& key, const Json::Value &value) {
    mState[key] = value;
}

Json::Value OIfaceClient::getDecisionState(const std::string& key) {
    return mDecisionState[key];
}

Json::Value OIfaceClient::getDecisionState() {
    return mDecisionState;
}

void OIfaceClient::setDecisionState(const std::string& key, const Json::Value &value) {
    if (value == Json::nullValue) {
        if (mDecisionState.isMember(key))
            mDecisionState.removeMember(key);
    } else
        mDecisionState[key] = value;
}

void OIfaceClient::dumpDecisionState() {
    DEBUG("dump state for %s\n%s", getClientName().c_str(),
            mDecisionState.toStyledString().c_str());
}

string OIfaceClient::getTypeAsString(int type) {
    switch (type) {
        case OIFACE_TYPE_JSON_CLIENT:
            return "oifacegame";
        case OIFACE_TYPE_OIM_CLIENT:
            return "oifaceim";
        case OIFACE_TYPE_LOG_CLIENT:
            return "reslog";
        case OIFACE_TYPE_BINDER_CLIENT:
            return "sdk";
        case OIFACE_TYPE_ENGINE_CLIENT:
            return "engine_boost";
        // case OIFACE_TYPE_CONNECTION_LESS:
        //     return "oifacegame";
        default:
            DEBUG("unknown type %d", type);
            return "";
    }
}

int OIfaceClient::getConfigCategory(vector<string>& category) {
    int type = getType();

    switch (type) {
        case OIFACE_TYPE_CONNECTION_LESS:
            category.push_back("sdk");
            category.push_back("connectionless");
            break;
        default:
            string stype = getTypeAsString(type);
            if (stype == "")
                return -1;
            category.push_back(stype);
            break;
    }

    return 0;
}

int OIfaceClient::getConfig(const std::string& key, Json::Value *val) {
    if (val == NULL) {
        ERROR("val is null");
        return -1;
    }

    vector<string> category;
    if (getConfigCategory(category) < 0)
        return -1;

    category.push_back(getPackageName());

    Json::Value value;
    if (GlobalConfig::getInstance().getConfig(&value, category) == 0) {
        if (value.isNull() || (!value.isObject())) {
            DEBUG("get config value for %s:%s failed",
                    getTypeAsString(getType()).c_str(), getPackageName().c_str());
            return -1;
        }

        if (!value.isMember(key)) {
            DEBUG("unable to find key %s", key.c_str());
            return -1;
        }

        *val = value[key];
    }
    return 0;
}

/* invalid frames if needed */
void OIfaceClient::invalidateLayer() {
    DEBUG("invalidating layer frame counter");
    mInvalidated = true;
}

android::Mutex OIfaceClient::mSFLock;
sp<IBinder> OIfaceClient::mSFBinder = NULL;
sp<SFDeathRecipient> OIfaceClient::mSFDeathRect = NULL;
void OIfaceClient::handleSFBinderDied() {
    android::Mutex::Autolock _l(mSFLock);
    mSFBinder = NULL;
    DEBUG("surfaceflinger died!");
}


int OIfaceClient::getSFService() {
    if (mSFBinder == NULL) {
        mSFBinder = defaultServiceManager()->checkService(String16("SurfaceFlinger"));
        if (mSFBinder == NULL) {
            ERROR("unable to get surfaceflinger");
            return -1;
        }
        if (mSFDeathRect == NULL) {
            mSFDeathRect = new SFDeathRecipient();
        }
        mSFBinder->linkToDeath(mSFDeathRect);
    }
    return mSFBinder == NULL ? -1 : 0;
}

void OIfaceClient::handleLayerRegistered(int32_t type) {
    Json::Value layer, delay;

    if (getConfig("layer", &layer) < 0) {
        ERROR("unable to get layer name\n");
        return;
    }
    if (!(layer.isString())) {
        ERROR("layer is not a string");
        return;
    }

    if (getConfig("delay", &delay) < 0) {
        ERROR("unable to get delay\n");
        return;
    }
    if (!(delay.isInt())) {
        ERROR("delay is not a int");
        return;
    }

    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSFService() < 0) {
            return;
        }
        Parcel data, reply;
        data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        /*TODO: data.writeStrongBinder(xxx) passing a binder to SF maybe a better idea*/
        data.writeString8(String8(layer.asCString()));
        if (type == MARK_START) {
            data.writeInt32(delay.asInt());
            status_t result = mSFBinder->transact(REGISTER_LAYER, data, &reply);
            if (result != NO_ERROR)
                ERROR("%s, failed to transact: %d", __func__, result);
        } else if (type == MARK_STOP) {
            status_t result = mSFBinder->transact(UNREGISTER_LAYER, data, &reply);
            if (result != NO_ERROR)
                ERROR("%s, failed to transact: %d", __func__, result);
        }
    }
}

void OIfaceClient::updateFrameBoostAction(const Decision& decision) {
    // currently, just support hypnus action and schedtune
    if (decision.type != DECISION_TYPE_ACTION
            && decision.type != DECISION_TYPE_SCHEDTUNE) {
        mFbAction = FRAME_BOOST_DEFAULT_ACTION;
    } else {
        mFbAction = decision;
    }
    mFrameRescuer->updateBoostDecision(mClientName, mFbAction);
    INFO("Update frame boost action: type %d, boost %d",
            mFbAction.type, mFbAction.type == DECISION_TYPE_ACTION ?
            mFbAction.action.action_type : mFbAction.schedtune_boost.boost);
}

int OIfaceClient::getFBStatistics(struct FrameBoostStats& stats) {
    if (!mFrameBoosted)
        return -1;

    stats.time = mFrameRescuer->getBoostTime();
    stats.count = mFrameRescuer->getBoostCount();
    stats.dcount = mFrameRescuer->getDiretlyBoostCount();
    stats.lags = mFrameRescuer->getLags();

    return 0;
}

void OIfaceClient::handleTriggerStart() {
    mInvalidated = false;
    mTriggerTime = systemTime();
    sendBroadcast(String16("oplus.intent.action.gamestart"), String16(), String16(), String16(),
        4, "packageName", getPackageName().c_str(), "startTs", std::to_string(mTriggerTime).c_str());
    mFrameCount = getFrames();
    mBatteryRemain = ChargerService::getInstance().getBatteryRemain();

    /* ThermalService information */
    ThermalService::getInstance().getAllTemperature(mTemperature);

    /* FrameStats contains fps information */
    if (startFrameStats() == 0)
        mUseFrameStats = 1;
    else
        mUseFrameStats = 0;

    /* virtual on trigger start */
    onTriggerStart();
    mTriggerStarted = true;
    if(PlatformAdaptor::getInstance().isQcom()) {
        mLmhCnt0 = getLmhCnt(LMH_LIMIT_CNT_PATH_0);
        mLmhCnt1 = getLmhCnt(LMH_LIMIT_CNT_PATH_1);
    }
    map<int, string> dummy;
    getAllProcessCpuTime(mCpuTime, dummy);

    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(),
    //         "start_time", (int32_t)(systemTime(SYSTEM_TIME_REALTIME) / 1000000000LL));

    for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
        getCpuTimeInState(cpuMap.second, mTimeInState[cpuMap.first]);
    }

    for (int i = 0; i < MAX_CPU_CORES; i++) {
            mIdleTime[i] = getCpuIdleTime(i);
    }

    getGpuStats(mGpuTimeInState);

    /* clear network latency state */
    DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(),
            "networkReportTime", 0);
    DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(),
            "networkLatency", 0);

    /* fb statistics */
    getFBStatistics(mFbStats);
}

void OIfaceClient::handleTriggerStop() {
    DataCollector& dc(DataCollector::getInstance());

    int64_t now = systemTime();
    if ((now <= 0) || (now == mTriggerTime)) {
        ERROR("invalid current time");
        return;
    }

    sendBroadcast(String16("oplus.intent.action.gamestop"), String16(), String16(), String16(),
        4, "packageName", getPackageName().c_str(), "stopTs", std::to_string(now).c_str());

    if (mInvalidated) {
        DEBUG("discarding previous information due to invalidted state");
        return;
    }

    if (!mTriggerStarted) {
        ERROR("trigger not started");
        return;
    }

    if (mTriggerTime <= 0) {
        ERROR("trigger time invalid");
        return;
    }

    /* FPS */
    // ATRACE_BEGIN("get frame stats");
    // if (mFrameCount > 0) {
    //     int32_t frames = getFrames();
    //     double fps = (double)(frames - mFrameCount) * 1000000000LL /
    //         (now - mTriggerTime);
    //     DEBUG("trigger time:%ld now:%ld last frames:%d current frames:%d fps:%f",
    //             mTriggerTime, now, mFrameCount, frames, fps);
    //     if (fps > 0.1f) {
    //         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "avg_fps", fps);
    //     }
    // }

//     if (mUseFrameStats) {
//         stopFrameStats();
//         double fps = 0;
//         if (mFrameStats.statTime > 0) {
//             fps = (double)(mFrameStats.frameCount * 1000000000LL) / mFrameStats.statTime;
//         } else if (mFrameStats.stopTime > mFrameStats.startTime) {
//             fps = (double)(mFrameStats.frameCount * 1000000000LL) /
//                     (mFrameStats.stopTime - mFrameStats.startTime);
//         } else if (mFrameStats.meanInterval != 0) {
//             fps = (double)1000 / mFrameStats.meanInterval;
//         }
//         if (fps > 0.1f) {
//             dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "stat_avg_fps", fps);
//             ERROR("%s fps:%f", getPackageName().c_str(), fps);
//         }
//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "stat_time",
//                 (int32_t)(mFrameStats.statTime / 1000000000LL));

//         int32_t jank_cnt = 0;
//         for (int i = 0; i < NUM_INTERVAL_STATS; i++) {
//             dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(),
//                     (string("interval") + to_string(i)).c_str(),
//                     mFrameStats.intervalStat[i]);
// #define MINIMAL_JANK_FRAMES	6
//             /* Nasty: innereye maintainer all quit... */
//             if (i > MINIMAL_JANK_FRAMES)
//                     jank_cnt += mFrameStats.intervalStat[i];
//         }

//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "variance",
//                         (float)jank_cnt / mFrameStats.statTime);

//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "stat_start_time",
//                 (int32_t)(mFrameStats.startTime / 1000000000LL));
//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "stat_stop_time",
//                 (int32_t)(mFrameStats.stopTime / 1000000000LL));
//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "mean_interval",
//                 mFrameStats.meanInterval);
//         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "stdev",
//                 mFrameStats.stdev);

//         DEBUG("frame stats: fps %f, %f, %f, stat time %lld",
//                 (double)(mFrameStats.frameCount * 1000000000LL) / mFrameStats.statTime,
//                 (double)(mFrameStats.frameCount * 1000000000LL) / (mFrameStats.stopTime - mFrameStats.startTime),
//                 (double)1000 / mFrameStats.meanInterval,
//                 (long long)mFrameStats.statTime);
//     }
//     ATRACE_END();

    /* switch to background */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "switch_bg", mSwitchToBackground);
    // DEBUG("switch_bg: %d", mSwitchToBackground);

    /* Frame Boost */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "frameboost",
    //         isFrameBooted() ? 1 : 0);
    struct FrameBoostStats fbstats;
    if (getFBStatistics(fbstats) == 0) {
        int time = (int)((fbstats.time - mFbStats.time) / 1000000);
        int count = fbstats.count - mFbStats.count;
        int dcount =  fbstats.dcount - mFbStats.dcount;
        int lags = fbstats.lags - mFbStats.lags;
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "fb_time", time);
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "fb_count", count);
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "fb_dcount", dcount);
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "fb_lag", lags);
        DEBUG("fb stats: fb_time %d ms, fb_lag %d, fb_count %d, fb_dcount %d, play_time %d s",
                time, lags, count, dcount, (int32_t)((now - mTriggerTime) / 1000000000LL));
    }

    /* Play Time*/
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "play_time",
    //         (int32_t)((now - mTriggerTime) / 1000000000LL));

    // if (mBatteryRemain > 0) {
    //     int nowRemain = ChargerService::getInstance().getBatteryRemain();
    //     double current = (double)(mBatteryRemain - nowRemain) * 3600000000000LL /
    //         (double)(now - mTriggerTime);

    //     if ((nowRemain > 0) && (mBatteryRemain - nowRemain >= 0)) {
    //         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "current", current);
    //         ERROR("%s current:%f", getPackageName().c_str(), current);
    //     }
    // }

    /* brightness */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "brightness", getBrightness());

    /* sound level */
    int32_t sound = getSoundLevel();
    if (sound >= 0) {
        int32_t soundType = getHeadsetState();
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "soundType", soundType);
        if (soundType > 0) /* headset inserted */
            sound = -sound;

        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "sound", sound);
    }

    /* Thermal Service information */
    std::vector<float> temperature(ThermalService::TEMPERATURE_COUNT);
    ThermalService::getInstance().getAllTemperature(temperature);
    /* cpu temperature */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "cpu_temp_begin", mTemperature[ThermalService::TEMPERATURE_CPU]);
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "cpu_temp_end", temperature[ThermalService::TEMPERATURE_CPU]);
    /* gpu temperature */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "gpu_temp_begin", mTemperature[ThermalService::TEMPERATURE_GPU]);
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "gpu_temp_end", temperature[ThermalService::TEMPERATURE_GPU]);
    /* skin temperature */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "skin_temp_begin", mTemperature[ThermalService::TEMPERATURE_SKIN]);
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "skin_temp_end", temperature[ThermalService::TEMPERATURE_SKIN]);
    DEBUG("%s temp: %f", getPackageName().c_str(), temperature[ThermalService::TEMPERATURE_SKIN]);
    /* battery temperature */
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "battery_temp_begin", mTemperature[ThermalService::TEMPERATURE_BATTERY]);
    // dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "battery_temp_end", temperature[ThermalService::TEMPERATURE_BATTERY]);

    /* network type */
    ThreadState &ts(ThreadState::getInstance());
    dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "network",
            (ts.isCurrentWifi() ? 1:0) | (ts.isCurrentData() ? 2: 0));

    /* delay boost times */
    mDelayBoostTimes = 0;

    if (mLmhCnt0 >= 0) {
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "lmh_cnt0",
                getLmhCnt(LMH_LIMIT_CNT_PATH_0) - mLmhCnt0);
    }

    if (mLmhCnt1 >= 0) {
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "lmh_cnt1",
                getLmhCnt(LMH_LIMIT_CNT_PATH_1) - mLmhCnt1);
    }

    /* cpuTime */
    ATRACE_BEGIN("GetAndSort");
    map<int, float> diff;
    multimap<float, int> sort;
    map<int, string> pidComm;

    if (mCpuTime.size() > 0) {
        map<int, float> cpuTimeEnd;

        ATRACE_BEGIN("getCpuTime");
        getAllProcessCpuTime(cpuTimeEnd, pidComm);
        ATRACE_END();

        for (auto& iter: mCpuTime) {
            if (cpuTimeEnd.find(iter.first) == cpuTimeEnd.end())
                continue;

            diff[iter.first] = cpuTimeEnd[iter.first] - iter.second;
        }
    }

    ATRACE_BEGIN("sort");
    if (diff.size() > 0) {
        std::transform(diff.begin(), diff.end(), std::inserter(sort, sort.begin()),
                    [&](std::pair<int, float> a)
                    {return std::pair<float, int>(a.second, a.first);});
    }
    ATRACE_END();

    ATRACE_BEGIN("set");
#define TOP_NUMBER  5
    string topCpu;
    int count = 0;
    for (auto iter = sort.rbegin(); iter != sort.rend(); iter++) {
        string path;

        if (pidComm.size() > 0) {
            path = pidComm[iter->second];
        } else {
            path = getFile(string("/proc/") + to_string(iter->second) + "/comm");
            if (path.size() > 0)
                path.pop_back();
        }
        topCpu += path + ":" + to_string(iter->first) + ",";

        count++;
        if (count >= TOP_NUMBER)
            break;
    }

    if (topCpu.back() == ',')
        topCpu.pop_back();

    dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "top_cpu",
            topCpu);

    ATRACE_END();
    ATRACE_END();

    char keyName[64];

    for (int i = 0; i < MAX_CPU_CORES; i++) {
        int64_t idleTime = 0;

        idleTime = getCpuIdleTime(i);

        snprintf(keyName, sizeof(keyName), "cpu%d_idle", i);
        if ((mIdleTime[i] >= 0) && (idleTime >= 0))
            DEBUG("idle time:%ld, last idle time:%ld", idleTime, mIdleTime[i]);
        dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), keyName,
                (float)(idleTime - mIdleTime[i]));
    }

    for (auto &cpuMap: AffinityService::getInstance().getClusterLeadingCpu()) {
        map<int32_t, int64_t> timeInState[CLUSTER_NUM];
        DEBUG("cluster:%d cpu:%d", cpuMap.first, cpuMap.second);
        map<int32_t, int64_t> deltaTimeInState;
        getCpuTimeInState(cpuMap.second, timeInState[cpuMap.first]);

        for (auto &oneState: timeInState[cpuMap.first]) {
            snprintf(keyName, sizeof(keyName), "cpu%d_freq_%d", cpuMap.second, oneState.first);

            deltaTimeInState[oneState.first] = oneState.second - mTimeInState[cpuMap.first][oneState.first];

            // DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
            //         getPackageName(), keyName, (float)((oneState.second - mTimeInState[cpuMap.first][oneState.first]) *
            //             1000 / sysconf(_SC_CLK_TCK)));
        }
        timeInStateToAverageFreq(deltaTimeInState);
    }

    map<int32_t, int32_t> gpuStat;
    getGpuStats(gpuStat);

    // if ((gpuStat.size() > 0) && (gpuStat.size() == mGpuTimeInState.size())) {
    //     DEBUG("triggering gpu freq stat");
    //     for (auto &stat: mGpuTimeInState) {
    //         snprintf(keyName, sizeof(keyName), "gpu_freq_%d", stat.first);
    //         DataCollector::getInstance().setData(DataCollector::DC_TYPE_INTERNAL,
    //                         getPackageName(), keyName, (float)(gpuStat[stat.first] - stat.second));
    //     }
    // }
}

void OIfaceClient::triggerMark(int32_t type) {
    DEBUG("client %s triggering mark:%d", getClientName().c_str(), type);

    // DataCollector& dc(DataCollector::getInstance());

    switch (type) {
    case MARK_START:
        /* virtual on trigger start */
        ATRACE_BEGIN("mark_start");
        handleTriggerStart();
        onTriggerStart();
        mTriggerStarted = true;
        ATRACE_END();
        break;
    case MARK_STOP:
        ATRACE_BEGIN("mark_stop");
        handleTriggerStop();

        /* virtual on trigger stop */
        onTriggerStop();

        // DEBUG("%s", dc.asJson().c_str());
        mTriggerStarted = false;
        ATRACE_END();
        break;
    case MARK_LOAD_START:
        mLoadingStart = systemTime();
        DEBUG("%s loading start", getPackageName().c_str());
        onLoadingStart();
        break;
    case MARK_LOAD_STOP:
        // if (mLoadingStart > 0) {
        //     dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), "loading",
        //             (int32_t)((systemTime() - mLoadingStart)/ 1000000000LL));
        // }
        DEBUG("%s loading stop", getPackageName().c_str());
        onLoadingStop();
        break;
    default:
        break;
    }
}

void OIfaceClient::triggerAppEvent(int32_t type, int32_t sceneId) {
    DEBUG("client %s triggering app event:%d, sceneId = %d", getClientName().c_str(), type, sceneId);

    switch (type) {
    case APP_EVENT_START:
        mAppSceneId = sceneId;
        mAppEventStartNanoSec = systemTime();
        break;
    case APP_EVENT_STOP:
        if (mAppSceneId > 0) {
            writeAppEvent2Db();
#ifdef PREQ
            checkAppEventTime();
#endif
            mAppEventStartNanoSec = 0;
            mAppSceneId = 0;
        }
        break;
    default:
        break;
    }
}

inline void OIfaceClient::writeAppEvent2Db() {
    DataCollector& dc(DataCollector::getInstance());
    DataCollector::DcType dcType = DataCollector::DC_TYPE_CONNECTION;

    switch(mAppSceneId) {
        case HARDCODER_SCENE_BOOT:
        case HARDCODER_SCENE_SEND_MSG:
        case HARDCODER_SCENE_SEND_PIC_MSG:
        case HARDCODER_SCENE_ENTER_CHATTING:
        case HARDCODER_SCENE_QUIT_CHATTING:
        case HARDCODER_SCENE_ENCODE_VIDEO:
        case HARDCODER_SCENE_SNS_SCROLL:
        case HARDCODER_SCENE_ALBUM_SCROLL:
        case HARDCODER_SCENE_AIO_MSG_SLIDE:
        {
            const int KEY_MAX_LEN = 20;
            char cntKeyStr[KEY_MAX_LEN] = {0};
            char timeKeyStr[KEY_MAX_LEN] = {0};
            snprintf(cntKeyStr, sizeof(cntKeyStr), "scene_%d_cnt", mAppSceneId);
            snprintf(timeKeyStr, sizeof(timeKeyStr), "scene_%d_time", mAppSceneId);

            dc.addData(dcType, getPackageName(), cntKeyStr, 1);
            dc.addData(dcType, getPackageName(), timeKeyStr,
                    (int32_t)((systemTime() - mAppEventStartNanoSec) / 1E6));
            break;
        }
        default:
            DEBUG("No need to write big data db, scene id = %d", mAppSceneId);
            return;
    }
    dc.finishOne(dcType, getPackageName());
}

#ifdef PREQ
inline void OIfaceClient::checkAppEventTime() {
    static const int PROP_NAME_MAX_LEN = 40;
    char propName[PROP_NAME_MAX_LEN] = {0};
    switch(mAppSceneId) {
        case HARDCODER_SCENE_BOOT:
        case HARDCODER_SCENE_SEND_MSG:
        case HARDCODER_SCENE_SEND_PIC_MSG:
        case HARDCODER_SCENE_ENTER_CHATTING:
        case HARDCODER_SCENE_QUIT_CHATTING:
        case HARDCODER_SCENE_ENCODE_VIDEO:
        case HARDCODER_SCENE_SNS_SCROLL:
        case HARDCODER_SCENE_ALBUM_SCROLL:
        case HARDCODER_SCENE_AIO_MSG_SLIDE:
            snprintf(propName, sizeof(propName), "persist.sys.oiface.scene_%d_time", mAppSceneId);
            break;

        default:
            DEBUG("No need to check app event, scene id = %d", mAppSceneId);
            return;
    }

    int32_t threshold = property_get_int32(propName, -1);
    DEBUG("getprop %s: %d", propName, threshold);

    if (threshold < 0) {
        DEBUG("Cannot get property: %s", propName);
        return;
    }

    int32_t appEventTime = (systemTime() - mAppEventStartNanoSec) / 1E6;
    if (appEventTime > threshold) {
        DEBUG("HM_APP_AVG", "oiface.scene_%d_time %d", mAppSceneId, appEventTime);
    }
}
#endif

int32_t OIfaceClient::setLayerName() {
    int inWhiteList = -1;
    Json::Value layer;
    inWhiteList = getConfig("layer", &layer);
    if(inWhiteList < 0) {
        DEBUG("unable to get layer name int white list\n");
    } else {
        if(layer.isString()) {
            mLayerName = layer.asString();
            DEBUG("find layer name in white list is: %s", mLayerName.c_str());
            return 0;
        } else {
            DEBUG("layer name in white list is not a string");
        }
    }

    Parcel data, reply;

    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSFService() < 0) {
            return -1;
        }
        data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        data.writeString16(String16(getPackageName().c_str()));
        status_t result = mSFBinder->transact(0x6f696665, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
    }

    mLayerName = String8(reply.readString16()).string();
    if(mLayerName.empty()) {
        ERROR("cannot get layer name by package name: %s", getPackageName().c_str());
        return -1;
    }
    DEBUG("package Name is: %s, Layer Name is: %s", getPackageName().c_str(), mLayerName.c_str());
    return 0;
}

int32_t OIfaceClient::getFrames() {
    if (mLayerName.empty() && setLayerName() == -1) {
        ERROR("invalid layer name");
        return -1;
    }

    Parcel data, reply;

    {
        android::Mutex::Autolock _l(mSFLock);

        if (getSFService() < 0) {
            return -1;
        }
        data.writeInterfaceToken(String16("android.ui.ISurfaceComposer"));
        data.writeString16(String16(mLayerName.c_str()));
        status_t result = mSFBinder->transact(0x6f696663, data, &reply);
        if (result != NO_ERROR)
            ERROR("%s, failed to transact: %d", __func__, result);
    }

    return reply.readInt32();
}

void OIfaceClient::onTriggerStart() {
}

void OIfaceClient::onTriggerStop() {
    DataCollector::getInstance().finishOne(DataCollector::DC_TYPE_INTERNAL, getPackageName());
}

int OIfaceClient::restoreFrameStats() {
    // if (isGameStarted()) {
    //     return uploadFrameStats();
    // }
    return 0;
}

void OIfaceClient::freshBackgroundStats() {
    if (!ThreadState::getInstance().isPackageForground(mPackageName))
        mSwitchToBackground++;
}

int OIfaceClient::startFrameStats() {
    // DEBUG("start frame stats");
    // memset(&mFrameStats, 0 , sizeof(mFrameStats));
    // mSwitchToBackground = 0;
    // return syncFrameStats(mLayerName, &mFrameStats, NULL, true, true);
    return 0;
}

void OIfaceClient::stopFrameStats() {
    // DEBUG("stop frame stats");
    // struct FrameStats frameStats = { .frameCount = 0, .statTime = 0 };
    // syncFrameStats(mLayerName, NULL, &frameStats, true, false);
    // updateFrameStats(frameStats);
    // syncFrameStats(mLayerName, NULL, NULL, false, false);
}

int OIfaceClient::getFrameStats() {
    // if (!isGameStarted())
    //     return -1;

    // struct FrameStats frameStats = { .frameCount = 0, .statTime = 0 };
    // syncFrameStats(mLayerName, NULL, &frameStats, true, false);
    // updateFrameStats(frameStats);
    return 0;
}

int OIfaceClient::uploadFrameStats() {
    // DEBUG("upload frame stats");
    // return syncFrameStats(mLayerName, &mFrameStats, NULL, true, false);
    return 0;
}

int OIfaceClient::updateFrameStats(const struct FrameStats& frameStats) {
    DEBUG("updateFrameStats start: count %d, stat time %lld",
            mFrameStats.frameCount, (long long)mFrameStats.statTime);

    if (mFrameStats.frameCount < frameStats.frameCount ||
            mFrameStats.statTime < frameStats.statTime) {
        memcpy(&mFrameStats, &frameStats, sizeof(frameStats));
        DEBUG("frame stats update suceess!");
    } else {
        DEBUG("frame stats has been updated in time, skip!");
    }

    DEBUG("updateFrameStats end: count %d, stat time %lld",
            mFrameStats.frameCount, (long long)mFrameStats.statTime);

    return 0;
}

const struct decision & OIfaceClient::getSmartFreqDecision(int level) {
    return smartFreqDecisionMap[level];
}

void OIfaceClient::setSmartFreqDecision(int level, const struct decision &decision) {
    DEBUG("Set level %d smart freq decision", level);
    smartFreqDecisionMap[level] = decision;
    smartFreqDecisionMap[level].hypnus_ctl.control_mask = 0;
}

#define FB_MAX_BOOST_TASKS     5
#define FB_MAX_BOOST_TASKS_CONFIG   FB_MAX_BOOST_TASKS + 5
int OIfaceClient::updateFrameBoostTaskList() {
    ATRACE_CALL();

    if (!isFrameBooted() || mFbAction.type != DECISION_TYPE_SCHEDTUNE)
        return -1;

    vector<struct boost_task> tasks;
    if (getTasks(mPid, tasks) < 0)
        return -1;

    for (auto& task : tasks) {
        map<int, struct boost_task>::iterator it = mBoostTasks.find(task.id);
        if (it != mBoostTasks.end()) {
            task.duration -= it->second.duration;
        }
    }
    sort(tasks.begin(), tasks.end(), compareBoostTask);

    struct boost_task topTasks[FB_MAX_BOOST_TASKS];
    int n = 0;
    for (auto& task : tasks) {
        topTasks[n].id = task.id;
        topTasks[n].duration = task.duration;
        DEBUG("fb update boost tasks: %d, %ld", task.id, task.duration);
        if (++n >= FB_MAX_BOOST_TASKS)
            break;
    }

    Decision decision = mFbAction;
    decision.schedtune_boost.clear_tasks = 0;
    decision.schedtune_boost.add_tasks = 1;
    decision.schedtune_boost.count = n;
    decision.schedtune_boost.tasks = topTasks;
    decision.schedtune_boost.boost = SCHEDTUNE_BOOST_SKIP_SETTING;
    decision.schedtune_boost.defered = SCHEDTUNE_DEFERED_SKIP_SETTING;
    DecisionDriver::getInstance().talkToDriver(decision, mClientName, false);
    return 0;
}

#define SCHEDTUNE_UPDATE_BOOST_TASK_DELAY 30000000000LL
int OIfaceClient::setFrameBoostTaskList() {
    ATRACE_CALL();

    if (mFbAction.type != DECISION_TYPE_SCHEDTUNE)
        return -1;

    int n = 0;
    struct boost_task topTasks[FB_MAX_BOOST_TASKS_CONFIG];
    vector<int> taskList;
    vector<struct boost_task> tasks;

    // init mBoostTasks, used by update task list later
    mBoostTasks.clear();
    getTasks(mPid, tasks);
    for (auto& task : tasks) {
        mBoostTasks[task.id] = task;
    }
    // firstly, get tasks with config file
    getConfigTaskList(mFbAction.schedtune_boost.index, mClientName, taskList);
    if (!taskList.empty()) {
        for (auto& task : taskList) {
            topTasks[n].id = task;
            DEBUG("fb set boost tasks: %d", task);
            if (++n >= FB_MAX_BOOST_TASKS_CONFIG)
                break;
        }
    } else {
        // failed to get tasks with config, then get its top tasks
        for (auto& task : tasks) {
            topTasks[n].id = task.id;
            DEBUG("fb set boost tasks: %d, %ld", task.id, task.duration);
            if (++n >= FB_MAX_BOOST_TASKS)
                break;
        }
    }
    // set boost task list
    Decision decision = mFbAction;
    decision.schedtune_boost.clear_tasks = 1;
    decision.schedtune_boost.add_tasks = 1;
    decision.schedtune_boost.count = n;
    decision.schedtune_boost.tasks = topTasks;
    decision.schedtune_boost.boost = 0;
    decision.schedtune_boost.defered = SCHEDTUNE_DEFERED_SKIP_SETTING;
    DecisionDriver::getInstance().talkToDriver(decision, mClientName, false);
    // update boost task list later
    BinderMessage msg;
    msg.what = BINDER_MESSAGE_SCHEDTUNE_UPDATE_BOOST_TASKS;
    msg.json = mClientName;
    OIfaceServer::getInstance().sendMessageDelayed(msg, SCHEDTUNE_UPDATE_BOOST_TASK_DELAY);
    return 0;
}

OIfaceClient::~OIfaceClient() {
    if (mTriggerStarted) {
        /* fake a stop trigger, MUST NOT call virutal functions  */
        handleTriggerStop();
        // triggerFrameBoostStop();
    }
}

ConnectionLessClient::ConnectionLessClient(const std::string &clientName)
    :OIfaceClient(clientName, OIFACE_TYPE_CONNECTION_LESS) {
}

ConnectionLessClient::~ConnectionLessClient(){}

Json::Value ConnectionLessClient::getDecisionState(const std::string& key) {
    return Json::nullValue;
}
Json::Value ConnectionLessClient::getDecisionState() {
    return Json::nullValue;
}
void ConnectionLessClient::setDecisionState(const std::string& key, const Json::Value& value) {}

void SFDeathRecipient::binderDied(const android::wp<android::IBinder>& who) {
    OIfaceClient::handleSFBinderDied();
};
