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

#include "OimParser.h"
#include "ThreadState.h"
#include "Utils.h"
#include "DataCollector.h"

using namespace android;

OimParser::OimParser(const char *socketName, int fd, int type, int pid, int uid):
    SocketParser(socketName, fd, type, pid, uid),
    mLoader(NULL),
    mDriver(DecisionDriver::getInstance())
{
    DEBUG("OimParser %s constructor", socketName);

    for (int i = 0; i < (int)ARRAY_SIZE(mDecision); i++) {
        mDecision[i].type = DECISION_TYPE_NONE;
    }

    mStartTime = systemTime();

    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "connect");
}

bool OimParser::isInitialized() {
    if (mLoader == NULL)
        return false;

    return ((PlatformAdaptor::getInstance().getPlatform() != PLAT_UNKNOWN) &&
            (mLoader->isInitialized()));
}

int OimParser::handleOneMessage()
{
    const OimMessage *msg;
    OimResponse resp;
    memset(&resp, 0, sizeof(resp));
    resp.func_id = -1;
    int ret = 0, respRetCode = 0;

    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "request");

    if (mRawBuffer.size() < sizeof(OimMessage)) { /* at least header is full */
        ERROR("partial OIM header(expected %d, got %d)", (int)sizeof(OimMessage), (int)mRawBuffer.size());
        return 0; /* return 0 anyway and handle it next time */
    }

    msg = reinterpret_cast<const OimMessage *>(mRawBuffer.data());

    if (msg->tag != OIM_TAG) {
        ERROR("protocol header %d is not recognized", msg->tag);
        resp.ret_code = ERR_UNAUTHORIZED;
        ret = -2;
        goto fail_resp;
    }

    if (msg->version != OIM_VERSION) {
        ERROR("protocol version %d is unable to handle", msg->version);
        resp.ret_code = ERR_CLIENT_UPGRADE_REQUIRED;
        ret = -2;
        goto fail_resp;
    }

    /* don't need to send response */
    if (mRawBuffer.size() < sizeof(OimMessage) + msg->body_len) {
        ERROR("received partial OIM message body"); /* save and return */
        return 0;
    }

    /* response initialization */
    resp.tag = msg->tag;
    resp.version = msg->version;
    resp.func_id = msg->func_id;
    resp.request_id = msg->request_id;
    resp.ret_code = 0;
    resp.body_len = 0;

    INFO("Message received(tag:%d version:%d func_id:%d size:%d body_len:%d tid:%d timestamp:%ld)",
            msg->tag, msg->version, msg->func_id, (int)mRawBuffer.size(), msg->body_len,
            msg->caller_tid, msg->timestamp);

#define MAX_LOG_BUFFER  256
    char buf[MAX_LOG_BUFFER];
    snprintf(buf, sizeof(buf), "{\"oim\":{\"tag\":\"%d\", \"version\":\"%d\", \"func_id\":\"%d\",\"size\":%d, \"body_len\":%d, \"tid\":%d, \"timestamp_remote\":%ld, \"timestamp\":%lld}}\n",
            msg->tag, msg->version, msg->func_id, (int)mRawBuffer.size(), msg->body_len,
            msg->caller_tid, msg->timestamp, (long long)systemTime(CLOCK_MONOTONIC));

    OIfaceServer::getInstance().sendRemoteLog(buf);

    /* check permission when first message arrived */
    if (mLoader == NULL) {
        mLoader = new OIfaceModuleLoader(getPid(), getUid(), "oifaceim");
        if (!mLoader->isInitialized()) { /* not in white list */
            DEBUG("cannot initialize module, come here to check permission");
            if (checkPermission() < 0) {
                DEBUG("check permission errror, connection refused");
                /* free resouce & close connection */
                ret = -2;
                resp.ret_code = ERR_UNAUTHORIZED;
                delete mLoader;
                mLoader = NULL;
                goto fail_resp;
            }
        }
        /* check successful, fall through */
    }

    respRetCode = performAction(resp);
    DEBUG("OIM respRetCode is 0x%x", respRetCode);
    if (respRetCode < 0) {
        ERROR("perform action use OIM failed");
        resp.ret_code = ERR_SERVICE_UNAVAILABLE;
        ret = -1;
        /* don't return here */
    } else if (respRetCode > 0) {
        resp.ret_code = respRetCode;
    }

    /* erase already handled message */
    mRawBuffer.erase(0, sizeof(OimMessage) + msg->body_len);

    if (mRawBuffer.size() > 0) {
        DEBUG("still have message to handle");
        ret = 1;
    }

fail_resp:
    /* add timestamp to each response call */
    resp.timestamp = getCurrentTimestamp();
    if (sendResponse(resp) < 0) {
        ERROR("send response failed");
    }

    return ret;
}

/* return -2 on close */
int OimParser::handlePollIn() {
    string str;
    int ret = 0;

    ATRACE_CALL();

    DEBUG("new message arrrived");

    if (readRemoteMessage(str) < 0) {
        ERROR("read from remote message failed");
        return -1;
    }

    if (str.size() == 0) {
        INFO("may be closed");
        return -2;
    }

    mRawBuffer += str;

    do {
        ret = handleOneMessage();
        DEBUG("message left:%d", (int)mRawBuffer.size());
    } while (ret > 0);


    return ret;
}

int OimParser::checkPermission() {
    return mLoader->checkPermission(reinterpret_cast<const OimMessage*>(mRawBuffer.data()));
}

int OimParser::performAction(OimResponse& resp)
{
    int num;
    int ret = 0;

    ThreadState& state(ThreadState::getInstance());

    if (mLoader == NULL)
        return -1;

    if (!state.isPackageForground(getPackageName())) {
        ERROR("action during background forbidden\n");
        return -1;
    }

    num = mLoader->getDecision(mDecision, MAX_DECISIONS,
            reinterpret_cast<const OimMessage*>(mRawBuffer.data()));
    if (num < 0) {
        ERROR("got %d decisions\n", num);
        return -1;
    }

    if (num == 0)
        return 0;

    for (int i = 0; i < num; i++) {
        if (mDecision[i].type == DECISION_TYPE_STRING) {
            memcpy(resp.str, (const char *)mDecision[i].im_callback.protocolinfo, mDecision[i].im_callback.len);
            resp.body_len = mDecision[i].im_callback.len;
            DEBUG("return protocolinfo len %d, info: %s \n", resp.body_len,resp.str);
            continue;
        }
        DEBUG("OimParser hypnus timeout: %d, orms timeout %d.", mDecision[i].action.timeout, mDecision[i].orms.timeout );
        if (mDriver.talkToDriver(mDecision[i], getClientName()) < 0) {
            ERROR("talk to driver failed");
            continue;
        }

        /* Firstly check if there is any CPU HIGH FREQ actions, if not check IO HIGH FREQ.*/
        if (mDecision[i].type == DECISION_TYPE_ACTION) {
            if (mDecision[i].action.action_type == 15) {
                DEBUG("IM perform action CPU HIGH FREQ LEVEL 1");
                ret = RET_CPU_HIGH_FREQ_LEVEL_1;
            } else if (mDecision[i].action.action_type == 12) {
                DEBUG("IM perform action CPU HIGH FREQ LEVEL 2");
                ret = RET_CPU_HIGH_FREQ_LEVEL_2;
            } else if (mDecision[i].action.action_type == 7) {
                DEBUG("IM perform action CPU HIGH FREQ LEVEL 3");
                ret = RET_CPU_HIGH_FREQ_LEVEL_3;
            }
        } else if (ret == 0 && mDecision[i].type == DECISION_TYPE_IO) {
            if (mDecision[i].io.io_type == HIGH_IO) {
                DEBUG("IM perform action HIGH IO");
                ret = RET_HIGH_IO_FREQ;
            }
        }
    }

    return ret;
}

int OimParser::sendResponse(const OimResponse &resp) {
    string str;

#define MAX_TRACE_NAME 128
    char tag[MAX_TRACE_NAME];

    snprintf(tag, MAX_TRACE_NAME, "sendResponse:%d,%d",
            resp.func_id, resp.ret_code);

    ATRACE_NAME(tag);

    snprintf(tag, MAX_TRACE_NAME, "{\"oim_response\":{\"func_id\":\"%d\", \"ret_code\":%d, \"timestamp\":%lld}\n",
            resp.func_id, resp.ret_code, (long long)systemTime(CLOCK_MONOTONIC));
    OIfaceServer::getInstance().sendRemoteLog(tag);

    str.append((const char *)&resp, (sizeof(OimResponse) - sizeof(resp.str) + resp.body_len));

    return sendRemoteMessage(str);
}

OimParser::~OimParser() {
    DataCollector::getInstance().incData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "disconnect");
    DataCollector::getInstance().setData(DataCollector::DC_TYPE_CONNECTION, getPackageName(),
            "duration", (int32_t)((systemTime() - mStartTime) / 1000000000L));

    /* free resource */
    if (mLoader != NULL) {
        delete mLoader;
    }

}
