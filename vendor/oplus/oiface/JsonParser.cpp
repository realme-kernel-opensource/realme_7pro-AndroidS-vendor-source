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

#include <time.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>
#include <stdlib.h>
#include <errno.h>

#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include "OIface.h"
#include "JsonParser.h"
#include "ThreadState.h"
#include "DataCollector.h"
#include "OifaceCallbackManager.h"
#include "BinderMessage.h"

using namespace android;
using namespace std;

JsonParser::JsonParser(const char *socketName, int fd, int type, int pid, int uid):
        SocketParser(socketName, fd, type, pid, uid),
        mRawBuffer(""), mJsonBuffer(""), dirty(false),
        driver(DecisionDriver::getInstance()),
        mLoader(getPid(), getUid(), "oifacegame") {

    DEBUG("JsonParser %s constructor", socketName);

    for (int i = 0; i < (int)ARRAY_SIZE(mDecision); i++) {
        mDecision[i].type = DECISION_TYPE_NONE;
    }

    /* set json keys to parse */
    if (mLoader.isInitialized()) {
        setJsonKey(mLoader.getJsonKey());
    }
    mStartTime = systemTime();

    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "connect");
    Json::Value statusJson;
    statusJson[CONNECT_STATUS_KEY] = CLIENT_CONNECTED;
    OifaceCallbackManager::getInstance().reportTGPAInfo(getUid(), getPid(), statusJson.toStyledString());
}

/* check mRawBuffer and put full json string to mJsonBuffer */
int JsonParser::formatJson() {
    int i;
    int state = -1;
    int start = 0, end = 0;

    if (mRawBuffer.size() == 0)
        return -1;

    /* state:
     * -1: begin state
     * 0: found {
     * 1: found }
     */

    for (i = 0; i < (int)mRawBuffer.size(); i++) {
        if (state == -1) {
            if (mRawBuffer[i] == '{') {
                state = 0;
                start = i;
                DEBUG("%d, found left", i);
            }
            continue;
        }

        if (state == 0) {
            if (mRawBuffer[i] == '}') {
                state = 1;
                end = i;
                DEBUG("%d, found right", i);
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

bool JsonParser::compareJsonValue(int type, JsonValue& v1, JsonValue& v2) {
    if (type == TYPE_UINT32)
        return (v1.val_uint32 == v2.val_uint32);

    if (type == TYPE_INT32)
        return (v1.val_int32 == v2.val_int32);

    if (type == TYPE_UINT64)
        return (v1.val_uint64 == v2.val_uint64);

    if (type == TYPE_INT64)
        return (v1.val_int64 == v2.val_int64);

    if (type == TYPE_STRING)
        return (strcmp(v1.val_string, v2.val_string) == 0);

    return false;
}

int JsonParser::assignValue(int type, JsonValue& value, const char* str) {
    long int lval;
    long long llval;

    errno = 0;
    if ((type == TYPE_INT32) || (type == TYPE_UINT32)) {
        lval = strtol(str, NULL, 0);

        if ((errno == ERANGE && (lval == LONG_MAX || lval == LONG_MIN))
                || (errno != 0 && lval == 0)) {
            ERROR("strtol %s failed(%s)\n", str,
                    strerror(errno));
            return -1;
        }

        if (type == TYPE_INT32)
            value.val_int32 = lval;
        else
            value.val_uint32 = lval;

        return 0;
    }

    if ((type == TYPE_INT64) || (type == TYPE_UINT64)) {
        llval = strtoll(str, NULL, 0);

        DEBUG("changing string %s", str);
        if ((errno == ERANGE && (llval == LLONG_MAX || llval == LLONG_MIN))
                || (errno != 0 && llval == 0)) {
            ERROR("strtol %s failed(%s)\n", str,
                    strerror(errno));
            return -1;
        }
        DEBUG("value change to %lld", (long long)llval);

        if (type == TYPE_INT64)
            value.val_int64 = llval;
        else
            value.val_uint64 = llval;

        return 0;
    }

    if (type == TYPE_STRING) {
        snprintf(value.val_string, sizeof(value.val_string), "%s", str);
        return 0;
    }

    return -1;
}

int JsonParser::parse() {
    bool success;

    ATRACE_CALL();

    Json::Value root;
    Json::Reader reader;

    if (formatJson() < 0) {
        ERROR("formatJson failed\n");
        return -1;
    }

    success = reader.parse(mJsonBuffer, root);

    INFO("JsonParser parsing string:%s\n", mJsonBuffer.c_str());

    if (!success || root.isNull()) {
        ERROR("parse failed:\n%s", mJsonBuffer.c_str());
        return -1;
    }

    for (vector<JsonKey>::iterator iter = mJsonKey.begin();
            iter != mJsonKey.end(); iter++) {

        JsonValue oldValue = mJsonValue[iter->key_name];
        JsonValue newValue;

        Json::Value value = root.get(iter->key_name, "-1");
        if (!value.isString()) {
            ERROR("value is not string\n");
            continue;
        }

        if (value.asString() == "-1") {
            continue;
        }

        errno = 0;

        if (assignValue(iter->type, newValue, value.asCString())) {
            ERROR("assign value failed.(type:%d key:%s value:%s)\n",
                    iter->type, iter->key_name,
                    value.asCString());
            continue;
        }

        if (!compareJsonValue(iter->type, oldValue, newValue)) {
            dirty = true;
            mJsonValue[iter->key_name] = newValue;
            DEBUG("new value(%s:%s) assigned\n",
                    iter->key_name, value.asCString());
        }
    }

    return 0;
}

/* return -2 on close */
int JsonParser::handlePollIn() {
    string str;

    ATRACE_CALL();

    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "request");
    if (readRemoteMessage(str) < 0) {
        ERROR("read from remote message failed");
        return -1;
    }

    OifaceCallbackManager::getInstance().reportTGPAInfo(getUid(), getPid(), str + "\n");
    /* XXX: May affect performance?? */
    OIfaceServer::getInstance().sendRemoteLog("{\"json\":" + str + "}\n");

/** game strategy was defined in cosa
    if (str.size() == 0) {
        INFO("may be closed");
        return -2;
    }

    mRawBuffer += str;
    while (mRawBuffer.size() > 0) {
        if (parse() != 0) {
            return -1;
        }

        if (performAction() < 0) {
            ERROR("perform action failed");
            return -1;
        }
    }
*/
    return 0;
}

void JsonParser::dumpKeys() {
    for (vector<JsonKey>::const_iterator iter = mJsonKey.begin();
            iter != mJsonKey.end(); iter++) {
        DEBUG("key_name:%s type:%d oneshot:%d\n",
                iter->key_name, iter->type, iter->oneshot);
    }
}

void JsonParser::resetOneshotValue() {
    JsonValue val;

    val.val_uint64 = 0LL;
    for (vector<JsonKey>::const_iterator iter = mJsonKey.begin();
            iter != mJsonKey.end(); iter++) {
        if (!iter->oneshot)
            continue;

        /* XXX: tricky here!! Set it to 0 for all union vals . */
        if (mJsonValue.find(iter->key_name) != mJsonValue.end()) {
            DEBUG("reset key:%s\n", iter->key_name);
            setJsonValue(iter->key_name, val);
        }
    }
}

int JsonParser::performAction() {
    int num;
    int i;
    JsonValue lowBatteryValue;

    /* strlen(action) + strlen(timeout) + MAX_INT_IN_BYTES*2 + etc.. */
#define MAX_BUFFER      80

    ThreadState& state(ThreadState::getInstance());

    if (!state.isPackageForground(getPackageName())) {
        INFO("package %s action during background skipped\n", getPackageName().c_str());
        return 0;
    }

    if (!isDirty())
        return 0;

    if(OIfaceServer::getInstance().getCurrentTemp() <= LOW_BATTERY_LEVEL) {
        lowBatteryValue.val_int64 = 1;
        mJsonValue["low_battery"] = lowBatteryValue;
    } else{
        lowBatteryValue.val_int64 = 0;
        mJsonValue["low_battery"] = lowBatteryValue;
    }

    num = mLoader.getDecision(mDecision, MAX_DECISIONS, mJsonValue);
    if (num < 0) {
        ERROR("got %d decisions\n", num);
        return -1;
    }

    if (num == 0)
        return 0;

    resetOneshotValue();
    clearDirty();

    for (i = 0; i < num; i++) {
        /* sync decision type to driver */
        DEBUG("decision[%d] type:%d", i, mDecision[i].type);
        if (driver.talkToDriver(mDecision[i], getClientName()) < 0)
            continue;
    }

    return 0;
}

const map<string, JsonValue>& JsonParser::getJsonValue() {
    return mJsonValue;
}

const vector<JsonKey>& JsonParser::getJsonKey() {
    return mJsonKey;
}

int JsonParser::getJsonValue(const string& str, JsonValue* val) {
    const map<string, JsonValue>::const_iterator& iter = mJsonValue.find(str);
    if (iter != mJsonValue.end()) {
        *val = mJsonValue[str];
        return 0;
    }

    return -1;
}

void JsonParser::setJsonValue(const string& str, JsonValue& val) {
    map<string, JsonValue>::iterator iter = mJsonValue.find(str);
    if (iter != mJsonValue.end()) {
        mJsonValue[str] = val;
    } else {
        ERROR("unexpected set in %s:%d\n", __func__, __LINE__);
    }
}

int JsonParser::setJsonKey(const struct json_key* keyList) {
    if (keyList == NULL) {
        ERROR("set json key failed with invalid argument\n");
        return -1;
    }

    while (strlen(keyList->key_name) != 0) {
        mJsonKey.push_back(*keyList);
        mJsonValue[keyList->key_name] = keyList->default_value;
        keyList++;
    }

    dumpKeys();
    return 0;
}

void JsonParser::onTriggerStop() {

    // DataCollector &dc(DataCollector::getInstance());
    // for (auto& iter: mJsonKey) {
    //     string val = iter.type == TYPE_STRING ? mJsonValue[iter.key_name].val_string :
    //         to_string(mJsonValue[iter.key_name].val_int64);
    //     dc.setData(DataCollector::DC_TYPE_INTERNAL, getPackageName(), string("key") + iter.key_name,
    //             val) ;
    // }

    OIfaceClient::onTriggerStop();
}

JsonParser::~JsonParser() {
    /* FIXME:wifi is per-app decision??? */
    ThreadState::getInstance();
    DEBUG("JsonParser destructor called\n");

    /* Delete record */
    DecisionDriver::getInstance().updateDecisionState(getClientName(), false);

    /* data statistics */
    // DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "disconnect");
    // DataCollector::getInstance().setData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
    //         "duration", (int32_t)((systemTime() - mStartTime) / 1000000000L));
    Json::Value statusJson;
    statusJson[CONNECT_STATUS_KEY] = CLIENT_DIED;
    OifaceCallbackManager::getInstance().reportTGPAInfo(getUid(), getPid(), statusJson.toStyledString());
}

