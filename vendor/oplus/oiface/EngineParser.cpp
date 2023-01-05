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
 * zhoubugang      2018/11/16       1.0       Add Oiface feature
 * ------------------------------------------------------------------------------
 */

#include <time.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>
#include <stdlib.h>
#include <errno.h>
#include <utils/Trace.h>
#include "OIface.h"
#include "EngineParser.h"
#include "ThreadState.h"
#include "DataCollector.h"
#include "OIfaceServer.h"
#include "DecisionDriver.h"
#include "GlobalConfig.h"
#include "Utils.h"
#include "comp.h"

using namespace android;
using namespace std;

EngineParser::EngineParser(const char *socketName, int fd, int type, int pid, int uid):
        SocketParser(socketName, fd, type, pid, uid),
        mRawBuffer(""), mJsonBuffer("") {

    DEBUG("EngineParser %s constructor", socketName);

    mStartTime = systemTime();

    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),  "connect");

    /* getPackageName is virtual function and should return mPackageName */
    GlobalConfig::getInstance().getConfig(&mConfig, {getTypeAsString(getType())});
    DEBUG("config for %s is type:%s", getClientName().c_str(),  getTypeAsString(getType()).c_str());
    if (mConfig["enable"].isInt() && (mConfig["enable"].asInt() == 1)) {
        string catogory = getTypeAsString(getType());
        if (catogory == "")
            return;
        Json::Value root(mConfig["rules"]);
        if (root.isNull() || (!root.isString())) {
            DEBUG("rules config value for %s is invalid", catogory.c_str());
            return;
        }
        string rule(root.asString());
        if (compileProgram(rule.c_str(), &mSyntaxTree) < 0) {
            ERROR("compile program failed\n");
             mSyntaxTree.clear();
        }
    }
    setState("fg", (int)ThreadState::getInstance().isPackageForground(getPackageName()));
}

int EngineParser::formatJson() {
    int i;
    int state = -1;
    int start = 0, end = 0;
    if (mRawBuffer.size() == 0)
         return -1;
    for (i = 0; i < (int)mRawBuffer.size(); i++) {
         if (state == -1) {
             if (mRawBuffer[i] == '{') {
                  state = 0;
                  start = i;
                  //DEBUG("%d, found left", i);
             }
             continue;
         }
         if (state == 0) {
             if (mRawBuffer[i] == '}') {
                  state = 1;
                  end = i;
                 // DEBUG("%d, found right", i);
                  break;
             }
         }
    }
    if (state == 1) { /* start & end are initialized */
         mJsonBuffer = mRawBuffer.substr(start, end - start + 1);
         mRawBuffer = mRawBuffer.substr(end + 1);
    } else {
         DEBUG("not contain a full json:%s", mRawBuffer.c_str());
         return -1;
    }
    return 0;
}

/* return -2 on close */
int EngineParser::handlePollIn() {
    string str;

    ATRACE_CALL();

    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(), "request");
    if (readRemoteMessage(str) < 0) {
        ERROR("read from remote message failed");
        return -1;
    }

    /* XXX: May affect performance?? */
    OIfaceServer::getInstance().sendRemoteLog("{\"json\":" + str + "}\n");

    if (str.size() == 0) {
        INFO("may be closed");
        return -2;
    }

    DEBUG("EngineParser handling message(json:%s)", str.c_str());

    Json::Value root;
    Json::Reader reader;
    mRawBuffer += str;
    while(mRawBuffer.size() > 0) {
        if(formatJson() < 0) {
            ERROR("formatJson failed\n");
            return -1;
        }
        bool success = reader.parse(mJsonBuffer, root);
        if (!success || root.isNull()) {
            ERROR("parse json failed(%s)", str.c_str());
            return -1;
        }
        // DEBUG("received json: %s", root.toStyledString().c_str());
        if (!isInterested(root)) {
             DEBUG("received json is not interested");
             return -1;
        }
        matchAndApply(root);
        for (Json::ValueConstIterator iter = root.begin(); iter != root.end(); iter++) {
            setState(iter.key().asCString(), *iter);
        }
    }
    return 0;
}

void EngineParser::matchAndApply(const Json::Value& value) {
    ATRACE_BEGIN("running program");
    runProgram(getClientName().c_str(), mSyntaxTree, jsonToMap(getState()), jsonToMap(value), &mClientVariable);
    ATRACE_END();
}

bool EngineParser::isInterested(const Json::Value& value) {
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

void EngineParser::onTriggerStop() {
    OIfaceClient::onTriggerStop();
}

EngineParser::~EngineParser() {
    /* FIXME:wifi is per-app decision??? */
    ThreadState::getInstance();
    DEBUG("EngineParser destructor called\n");


    destroyProgram(mSyntaxTree);
    /* Delete record */
    DecisionDriver::getInstance().updateDecisionState(getClientName(), false);

    /* data statistics */
    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "disconnect");
    DataCollector::getInstance().setData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "duration", (int32_t)((systemTime() - mStartTime) / 1000000000L));

}

