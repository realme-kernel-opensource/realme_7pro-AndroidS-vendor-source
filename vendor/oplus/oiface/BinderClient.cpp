#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include <string>
#include <json/value.h>
#include <json/reader.h>
#include <stdlib.h>

#include "BinderClient.h"
#include "OIfaceServer.h"
#include "DecisionDriver.h"
#include "GlobalConfig.h"
#include "Utils.h"
#include "comp.h"
#include "DataCollector.h"
#include "ThreadState.h"
#include "OifaceCallbackManager.h"

using namespace std;
using namespace android;

enum {
        CPU_LEVEL_1 = 1,
        CPU_LEVEL_2,
        CPU_LEVEL_3,
};

BinderClient::BinderClient(int uid, int pid): OIfaceClient(string("binder_uid_") + to_string(uid) +
        "_pid_" + to_string(pid), OIFACE_TYPE_BINDER_CLIENT, uid, pid), mNotifierEnabled(false) {
    DEBUG("BinderClient %s constructor called", getClientName().c_str());

    mStartTime = systemTime();

    /* getPackageName is virtual function and should return mPackageName */
    GlobalConfig::getInstance().getConfig(&mConfig, {getTypeAsString(getType()), getPackageName()});
    DEBUG("config for %s is type:%s package_name:%s", getClientName().c_str(),
            getTypeAsString(getType()).c_str(), getPackageName().c_str());

    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(), "connect");

    /* parse rules */
    if (mConfig["enable"].isInt() && (mConfig["enable"].asInt() == 1)) {
        string catogory = getTypeAsString(getType());
        if (catogory == "")
            return;

        Json::Value root(mConfig["rules"]);

        if (root.isNull() || (!root.isString())) {
            DEBUG("rules config value for %s:%s is invalid", catogory.c_str(),
                    getPackageName().c_str());
            return;
        }

        /* split key to expression */
        string rule(root.asString());

        if (compileProgram(rule.c_str(), &mSyntaxTree) < 0) {
            ERROR("compile program failed\n");
            mSyntaxTree.clear();
        }

        /* setup data collector keys */
    }

    setState("fg", (int)ThreadState::getInstance().isPackageForground(getPackageName()));
}

BinderClient::~BinderClient() {
    DEBUG("BinderClient %s destructor called", getClientName().c_str());

    /* release resources */
    destroyProgram(mSyntaxTree);

    /* data statistics */
    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "disconnect");
    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "duration", (int32_t)((systemTime() - mStartTime) / 1000000000L));
}

int BinderClient::registerNotifier(const sp<IBinder>& binder) {
#ifdef OPLUS_FEATURE_OPPO_OIFACE
    mNotifier = interface_cast<IOIfaceNotifier>(binder);
#endif
    return 0;
}

void BinderClient::notifyCallback(const std::string& msg) {
#ifdef OPLUS_FEATURE_OPPO_OIFACE
    if ((mNotifierEnabled) && (mNotifier != NULL))
        mNotifier->onSystemNotify(String16(msg.c_str()));
    else if (!mNotifierEnabled) {
        INFO("BinderClient %s notifyCallback disabled", getPackageName().c_str());
    } else {
        INFO("mNotifier is null %s", getPackageName().c_str());
    }
#endif
}

int BinderClient::handleEvent(const BinderMessage& msg) {

    DEBUG("BinderClient %s handling event:%d json:%s", getClientName().c_str(), msg.what, msg.json.c_str());

    if (mConfig["enable"].isInt()) {
        if (mConfig["enable"].asInt() == 0) {
            ERROR("config is disabled for %s\n", getPackageName().c_str());
            return -1;
        }
    }

    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "request");

    Json::Value root;
    Json::Reader reader;

    switch (msg.what) {
        case BINDER_MESSAGE_UPDATE_GAME_INFO: {
            ATRACE_CALL();

            /* json format as follows, should be format-free :
               {
               "sceneId": xx,
               "loadLevel": xx,
               "cpuLevel": xx,
               "gpuLevel": xx
               }
            */
            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                return -1;
            }

            DEBUG("received json: %s", root.toStyledString().c_str());
            if (!isInterested(root)) {
                DEBUG("received json is not interested");
                break;
            }
            // matchAndApply(root);
            OifaceCallbackManager::getInstance().reportTGPAInfo(getUid(), getPid(), root.toStyledString());
            /* TODO: Only update interested keys */
            for (Json::ValueConstIterator iter = root.begin(); iter != root.end(); iter++) {
                setState(iter.key().asCString(), *iter);
            }
        } break;

        case BINDER_MESSAGE_REQUEST_RESOURCE: {
            Decision decision;

            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                return -1;
            }

            decision.type = DECISION_TYPE_ACTION;
            if (root["cpuLevel"].isUInt()) {
                if (root["cpuLevel"].asUInt() <= CPU_LEVEL_1)
                    decision.action.action_type = 15;
                else if (root["cpuLevel"].asUInt() >= CPU_LEVEL_3)
                    decision.action.action_type = 7;
                else
                    decision.action.action_type = 12;

                decision.action.timeout = root["timeout"].isUInt() ? root["timeout"].asUInt() : 5000;
                if (decision.action.timeout > 10000)
                    decision.action.timeout = 10000;

                DecisionDriver::getInstance().talkToDriver(decision, getClientName());
            }

        } break;

        case BINDER_MESSAGE_FORGROUND:
        case BINDER_MESSAGE_BACKGROUND:{
            const char *event = msg.what == BINDER_MESSAGE_FORGROUND ? "fg" : "bg";
            Json::Value v;

            DEBUG("%s handling event %s event", getClientName().c_str(), event);
            v["fg"] = msg.what == BINDER_MESSAGE_FORGROUND ? 1:0;

            if (!isInterested(v)) {
                DEBUG("perfMode is not interested");
                break;
            }
            matchAndApply(v);
            /* TODO: Only update interested keys */
            setState("fg", v["fg"]);
        } break;
        case BINDER_MESSAGE_HYPNUS_POLL_EVENT: {
            DEBUG("client received perfmode message:%s", msg.json.c_str());
            bool success = reader.parse(msg.json, root);
            if (!success || root.isNull()) {
                ERROR("parse json failed(%s)", msg.json.c_str());
                break;
            }

            if (!isInterested(root)) {
                DEBUG("perfMode is not interested");
                break;
            }
            matchAndApply(root);
            /* TODO: Only update interested keys */
            for (auto &item: root.getMemberNames()) {
                if (root[item].isInt())
                    setState(item, root[item]);
            }

        } break;

        default: {
            DEBUG("BUG: unable to handle event:%d", msg.what);
        } break;
    }

    return 0;
}

void BinderClient::matchAndApply(const Json::Value& value) {
    ATRACE_BEGIN("running program");
    runProgram(getClientName().c_str(), mSyntaxTree, jsonToMap(getState()),
            jsonToMap(value), &mClientVariable);
    ATRACE_END();
}

/* bypass getParseResult if all items is not instested */
bool BinderClient::isInterested(const Json::Value& value) {
    /* all items is interested */
    if (!mConfig.isMember("items"))
        return true;

    if (!mConfig["items"].isArray()) {
        ERROR("invalid config type for items");
        return true;
    }

    for (Json::ValueConstIterator iter = value.begin(); iter != value.end(); iter++) {
        for (unsigned int i = 0; i < mConfig["items"].size(); i++) {
            if (mConfig["items"][i] == iter.key()) {
                DEBUG("interested in %s", iter.key().asCString());
                return true;
            }
        }
    }

    return false;
}

void BinderClient::onTriggerStop() {
    /* log configuration data */
    if (!mConfig["dc_config"].isObject()) {
        OIfaceClient::onTriggerStop();
        return;
    }

    // DataCollector& dc(DataCollector::getInstance());

    // for (auto& item: mConfig["dc_config"].getMemberNames()) {
    //     if (!mConfig["dc_config"][item].isString())
    //         continue;

    //     /* have this state */
    //     Json::Value v = getState(item);
    //     if (v.isString()) {
    //         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), mConfig["dc_config"][item].asString(), v.asString());
    //     } else if (v.isInt()) {
    //         dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), mConfig["dc_config"][item].asString(), to_string(v.asInt()));
    //     }
    // }

    OIfaceClient::onTriggerStop();
}
