#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include <string>
#include <json/value.h>
#include <json/reader.h>
#include <stdlib.h>

#include "GeneralClient.h"
#include "OIfaceServer.h"
#include "DecisionDriver.h"
#include "GlobalConfig.h"
#include "Utils.h"
#include "comp.h"
#include "DataCollector.h"
#include "ThreadState.h"

using namespace std;
using namespace android;

GeneralClient::GeneralClient(int uid, int pid, struct GeneralConfig config):
    BinderClient(uid, pid),
    mConfig(config) {
    DEBUG("GeneralClient %s constructor called", getClientName().c_str());
    mStartTime = getSystemTime();
    mExtra = 0;
    mLastLoadTime = 0;
    mLastBurstTime = 0;
    if (!mConfig.config_enable) {
        if (isApp()) {
            mConfig.load_interval = 30000;
            mConfig.load_time = 10000;
        } else if (isGame()) {
            mConfig.load_interval = 60000;
            mConfig.load_time = 30000;
        }
        mConfig.burst_interval = 3000;
        mConfig.burst_time = 1000;
        mConfig.acc_ratio = 20;
    }

    if (isApp() || onlyEngine()) {
      triggerMark(0);
    }

    DataCollector& dc(DataCollector::getInstance());
    DataCollector::DcType dcType = DataCollector::DC_TYPE_CONNECTION;
    string pkgName = getPackageName();
    dc.setData(dcType, pkgName, "type", (int32_t)getType());
}

GeneralClient::~GeneralClient() {
    mActionTime = 0;
    setAction();
    if (isApp() || onlyEngine()) {
      triggerMark(1);
    }
    dumpData();
    DEBUG("GeneralClient %s destructor called", getClientName().c_str());
}

void GeneralClient::dumpData() {
    int64_t duration = getSystemTime() - mStartTime - mAllBackgroundTime;
    DataCollector& dc(DataCollector::getInstance());
    DataCollector::DcType dcType = DataCollector::DC_TYPE_CONNECTION;
    string pkgName = getPackageName();
    dc.setData(dcType, pkgName, "type", (int32_t)getType());
    dc.setData(dcType, pkgName, "allTime", (int32_t)duration);
    dc.setData(dcType, pkgName, "accTimes", (int32_t)mAccTimes);
    dc.setData(dcType, pkgName, "accAllTime", (int32_t)(mAccAllTime));
    dc.setData(dcType, pkgName, "invalidTimes", (int32_t)mInvalidTimes);
    dc.setData(dcType, pkgName, "accRatio", (int32_t)(100*mAccAllTime/duration));
}

int GeneralClient::setAction () {

    int64_t start = getSystemTime();
    int64_t end = start + mActionTime;

    int64_t duration = start - mStartTime - mAllBackgroundTime;
    if (duration < 0) {
        DEBUG("GeneralClient internal error (start:%ld, mStart:%ld, end:%ld, duration:%ld)", start, mStartTime, end, duration);
        return -1;
    }
    //Calculate the proportion of total acceleration time to total time
    int64_t ratio = 100*mAccAllTime/duration;
    if (ratio > getAccRatio()) {
        mInvalidTimes++;
        DEBUG("GeneralClient Acc Alltime: %ld, ratio :%ld reach the limit ratio : %d", mAccAllTime, ratio, getAccRatio());
        return -1;
    }

    mAccTimes++;
    if (start >= mAccEndTime) {
        //The last acceleration has been completed and need to be record
        mAccAllTime += (mAccEndTime - mAccStartTime);
        mAccEndTime = end;
    } else {
        //The last acceleration has been interrupted and need to record the time of the interruption
        mAccAllTime += (start - mAccStartTime);
        if (mActionTime == 0) {
            mAccEndTime = end;
        } else {
            if(end >= mAccEndTime) {
                mAccEndTime = end;
            } else {
                //do nothing
            }
        }
    }
    mAccStartTime = start;
    DEBUG("Generalclient type: %d, mAccAllTime:%ld, actiontype: %d, actiontime: %d", getType(), mAccAllTime, mActionType, mActionTime);
    if (isActionEnable()) {
        Decision decision;
        decision.type = DECISION_TYPE_ACTION;
        decision.action.action_type = 15;
        decision.action.timeout = mActionTime;
        DecisionDriver::getInstance().talkToDriver(decision, getClientName());
    }
    return 0;
}

int GeneralClient::handleEvent(const BinderMessage& msg) {

    DEBUG("GeneralClient %s handling event:%d json:%s", getClientName().c_str(), msg.what, msg.json.c_str());

    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(), "request");

    Json::Value root;
    Json::Reader reader;

    switch (msg.what) {
        case BINDER_MESSAGE_UPDATE_GAME_INFO: {
            ATRACE_CALL();

            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                return -1;
            }
            if (root["actionType"].isUInt()) {
                mActionType = root["actionType"].asUInt();
            }
            mActionTime = root["actionTime"].isUInt() ? root["actionTime"].asUInt() : 0;
            if (isGame()) {
                switch (mActionType) {
                    case ACTION_GAME_BURST:
                        if (!checkBurst())
                            break;
                        setAction();
                        break;
                    case ACTION_GAME_LOAD:
                        if (!checkLoad())
                            break;
                        setAction();
                        break;
                    case ACTION_GAME_CANCEL:
                        mActionTime = 0;
                        setAction();
                        break;
                    case ACTION_GAME_START: {
                        triggerMark(0);
                        mLastStartTime = getSystemTime();
                        mLastAccTimes  = mAccTimes;
                        mLastAccAllTime = mAccAllTime;
                        mLastInvalidTimes = mInvalidTimes;
                        break;
                    }
                    case  ACTION_GAME_END:{
                        DEBUG("ACTION_GAME_END triggerMark(1) end");
                        if(isGameStarted()) {
                            int64_t duration = getSystemTime() - mLastStartTime - mAllBackgroundTime;
                            DataCollector& dc(DataCollector::getInstance());
                            DataCollector::DcType dcType = DataCollector::DC_TYPE_INTERNAL;
                            string pkgName = getPackageName();
                            dc.setData(dcType, pkgName, "allTime", (int32_t)duration);
                            dc.setData(dcType, pkgName, "accTimes", (int32_t)(mAccTimes - mLastAccTimes));
                            dc.setData(dcType, pkgName, "accAllTime", (int32_t)(mAccAllTime - mLastAccAllTime));
                            dc.setData(dcType, pkgName, "invalidTimes", (int32_t)(mInvalidTimes - mLastInvalidTimes));
                            dc.setData(dcType, pkgName, "accRatio", (int32_t)(100*(mAccAllTime - mLastAccAllTime)/duration));
                            dumpData();
                            triggerMark(1);
                        }
                        break;
                    }
                    case  ACTION_GAME_MAIN:
                        mActionTime = 0;
                        setAction();
                        break;
                }
            } else {
                switch (mActionType) {
                    case ACTION_APP_MAIN:
                        mActionTime = 0;
                        setAction();
                        break;
                    case ACTION_APP_BURST:
                        if(!checkBurst())
                            break;
                        setAction();
                        break;
                    case ACTION_APP_LOAD:
                        if(!checkLoad())
                            break;
                        setAction();
                        break;
                    case ACTION_APP_CANCEL:
                        mActionTime = 0;
                        setAction();
                        break;
                }
            }
        } break;

        case BINDER_MESSAGE_UPDATE_ENGINE_INFO: {
            if (!isEngine()) {
                DEBUG("Only engine client can call this binder");
                break;
            }
            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                return -1;
            }
            int shaderCompile = root["shader_compile"].isUInt() ? root["shader_compile"].asUInt() : -1;
            int loadScene = root["load_scene"].isUInt() ? root["load_scene"].asUInt() : -1;
            bool update = false;

            if ((shaderCompile != -1) && (shaderCompile != mShaderCompile)) {
                update = true;
                mShaderCompile = shaderCompile;
            }
            if ((loadScene != -1) && (loadScene != mLoadScene)) {
                update = true;
                mLoadScene = loadScene;
            }
            if (update) {
                DEBUG("shaderCompile:%d, loadScene:%d", mShaderCompile, mLoadScene);
                if (mShaderCompile == 1 || mLoadScene == 1) {
                    mActionTime = mMaxEngineTime;
                } else {
                    mActionTime = 0;
                }
                setAction();
            }
        } break;

        case BINDER_MESSAGE_FORGROUND:
        case BINDER_MESSAGE_BACKGROUND:{
            int forground  = msg.what == BINDER_MESSAGE_FORGROUND ? 1:0;
            if (forground == 1) {
                enableAction();
                //need to set back to app/game mode when switch to forgound
                mAllBackgroundTime += (getSystemTime() - mLastBackgroundTime);
                DEBUG("General Client: %s changed to forgroud, all background time is: %ld", getClientName().c_str(), mAllBackgroundTime);
            } else {
                disableAction();
                mLastBackgroundTime = getSystemTime();
                //TODO
                //need to switch to normal scene
            }
        } break;

        case BINDER_MESSAGE_HYPNUS_POLL_EVENT: {
            DEBUG("client received perfmode message:%s", msg.json.c_str());
            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                break;
            }
            int perfmode = -1;
            if (root["perfmode"].isUInt()) {
                perfmode = root["perfmode"].asUInt();
            }
            if (perfmode == 0) {
                //TODO
                //normal scene
            } else if (perfmode == 1) {
                //TODO
                //power save
            } else if (perfmode == 2) {
                //TODO
                //performance
            } else {
                //TODO
                ERROR("get permode(%d) error ", perfmode);
            }
        } break;

        default: {
            DEBUG("BUG: unable to handle event:%d", msg.what);
        } break;
    }
    return 0;
}

void GeneralClient::onTriggerStop() {
    OIfaceClient::onTriggerStop();
}

bool GeneralClient::checkBurst() {
    int64_t timeInterval = getSystemTime() - mLastBurstTime;
    if (timeInterval < getBurstInterval()) {
        mInvalidTimes++;
        DEBUG("received BurstTime is too frequently(last:%ld, interval:%ld, config:%ld)", mLastBurstTime, timeInterval, getBurstInterval());
        return false;
    }
    mActionTime = (mActionTime < getBurstTime()) ? mActionTime : getBurstTime();
    mLastBurstTime = getSystemTime();
    return true;
}


bool GeneralClient::checkLoad() {
    int64_t timeInterval = getSystemTime() - mLastLoadTime;
    if (timeInterval < getLoadInterval()) {
        mInvalidTimes++;
        DEBUG("received LoadTime is too frequently(last:%ld, interval:%ld, config:%ld)", mLastLoadTime, timeInterval, getLoadInterval());
        return false;
    }
    mActionTime = (mActionTime < getLoadTime()) ? mActionTime : getLoadTime();
    mLastLoadTime = getSystemTime();
    return true;
}

uint64_t GeneralClient::getSystemTime() {
    struct timeval t;
    t.tv_sec = t.tv_usec = 0;
    gettimeofday(&t, NULL);
    return nsecs_t(t.tv_sec)*1000LL + nsecs_t(t.tv_usec)/1000LL;
}

bool GeneralClient::isApp() {
    return mConfig.general_type == GENERAL_CLIENT_APP;
}

bool GeneralClient::isGame() {
    return mConfig.general_type == GENERAL_CLIENT_GAME ||
           mConfig.general_type == GENERAL_CLIENT_ENGINE_GAME;
}

bool GeneralClient::isEngine() {
    return mConfig.general_type  == GENERAL_CLIENT_ENGINE ||
           mConfig.general_type == GENERAL_CLIENT_ENGINE_GAME;
}

bool GeneralClient::onlyEngine() {
    return mConfig.general_type  == GENERAL_CLIENT_ENGINE;
}

int GeneralClient::isActionEnable() {
    return mEnableAction;
}
