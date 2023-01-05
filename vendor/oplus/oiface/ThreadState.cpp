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
#include <cutils/properties.h>

#include "ThreadState.h"
#include "DecisionDriver.h"
#include "PlatformAdaptor.h"
#include "OIfaceServer.h"

#define MAX_PACKAGE_NAME    4096

ThreadState* ThreadState::sInstance = NULL;

ThreadState::ThreadState(): mCurrentPackage(""),
        mRespCallback(NULL), mWifiStatus(-1), mPid(-1), mDataStatus(-1) {
    char pkg[MAX_PACKAGE_NAME];
    char buf[MAX_PACKAGE_NAME];
    unsigned int scene, pid;

    buf[MAX_PACKAGE_NAME - 1] = '\0';

    mCurrentPackage = pkg;
}

int ThreadState::sendDecisionInfo(const Decision& decision) {
    oiface_data_response resp;

    if (mRespCallback == NULL) {
        DEBUG("oiface response data not ready\n");
        return -1;
    }

    resp.cmd = OIFACE_RESPONSE_CMD_ACTION;

    snprintf(resp.data.action.desp, OIFACE_ACTION_DATA_MAX_LEN,
            "type:%d action:%d timeout:%d",
            decision.type, decision.action.action_type,
            decision.action.timeout);

    return mRespCallback(&resp);

}

bool ThreadState::isPackageForground(string package) {
    bool ret;
    android::Mutex::Autolock _l(mLock);

#if defined(PREM)
    ret = (package == getForgroundPackage());
#else
    ret = (package == mCurrentPackage);
#endif

    DEBUG("current package is %s\n", mCurrentPackage.c_str());

    return ret;
}

std::string ThreadState::getForgroundPackage() {
    android::Mutex::Autolock _l(mLock);

    return mCurrentPackage;
}

int ThreadState::getForgroundPid() {
    android::Mutex::Autolock _l(mLock);
    if (mPid > 0)
        return mPid;
    return -1;
}

char *ThreadState::dumpNetworkStatus() {
    if (isCurrentWifi())
        return "wifi";
    if (isCurrentData())
        return "data";
    return "none";
}


/* XXX: called from NEO thread */
void ThreadState::updateSysInfo(oiface_sys_info* info) {
    DecisionDriver::getInstance();

    switch (info->cmd) {
    case OIFACE_REQUEST_CMD_SCREEN_ON: {
        BinderMessage msg;
        msg.what = BINDER_MESSAGE_SCREEN_ON;
        OIfaceServer::getInstance().sendMessage(msg);
        break;
    }
    case OIFACE_REQUEST_CMD_SCREEN_OFF: {
        BinderMessage msg;
        msg.what = BINDER_MESSAGE_SCREEN_ON;
        OIfaceServer::getInstance().sendMessage(msg);
        break;
    }

    case OIFACE_REQUEST_CMD_APP_SWITCH: {
        DEBUG("packageSwitch received package:%s status:%d\n",
                info->data.app.app_name,
                info->data.app.status);

        OIfaceServer &mgr(OIfaceServer::getInstance());
        BinderMessage msg;

        if (info->data.app.status == OIFACE_APP_PIP_OPENED || info->data.app.status == OIFACE_APP_PIP_OPEN) {
            //mLastPackage = mCurrentPackage;
            mCurrentPackage = info->data.app.app_name,
            mPid = info->data.app.app_pid;
            msg.what = BINDER_MESSAGE_BACKGROUND;
            msg.json = mCurrentPackage;
            mgr.sendMessage(msg);

            msg.what = BINDER_MESSAGE_CANCEL_STATE;
            msg.json = mgr.getClientName(mCurrentPackage);
            if (msg.json.size() > 0)
                mgr.sendMessage(msg);
        } else if (info->data.app.status == OIFACE_APP_ENTER_APP || info->data.app.status == OIFACE_APP_ENTER_GAME
        || info->data.app.status == OIFACE_APP_PIP_CLOSED || info->data.app.status == OIFACE_APP_PIP_CLOSE) {
            if (info->data.app.status == OIFACE_APP_ENTER_APP || info->data.app.status == OIFACE_APP_ENTER_GAME) {
                mLastPackage = mCurrentPackage;
                msg.what = BINDER_MESSAGE_BACKGROUND;
                msg.json = mLastPackage;
                mgr.sendMessage(msg);

                msg.what = BINDER_MESSAGE_CANCEL_STATE;
                msg.json = mgr.getClientName(mLastPackage);
                if (msg.json.size() > 0)
                    mgr.sendMessage(msg);
            }

            mCurrentPackage = info->data.app.app_name;
            mPid = info->data.app.app_pid;
            msg.what = BINDER_MESSAGE_RESTORE_STATE;
            msg.json = mgr.getClientName(mCurrentPackage);
            mgr.sendMessage(msg);

            msg.what = BINDER_MESSAGE_FORGROUND;
            msg.json = mCurrentPackage;
            mgr.sendMessage(msg);

            msg.what = BINDER_MESSAGE_RESTORE_CONNECTIONLESS_STATE;
            msg.json = mCurrentPackage;
            mgr.sendMessage(msg);
        } else if (info->data.app.status == OIFACE_APP_PIP_SWITCH || info->data.app.status == OIFACE_APP_PIP_CHANGE) {
            DEBUG("PIP switch, do nothing");
            mCurrentPackage = info->data.app.app_name;
            mPid = info->data.app.app_pid;
        }
        DEBUG("current package is %s and pid is %d", mCurrentPackage.c_str(), mPid);
        break;
    }

    /* FIXME: should remove oiface_request_cmd_app related code */
    case OIFACE_REQUEST_CMD_APP:
        DEBUG("package:%s cmd:%d status:%d\n",
                info->data.app.app_name,
                info->cmd,
                info->data.app.status);

        if (getForgroundPackage() == info->data.app.app_name) {

            OIfaceServer &mgr(OIfaceServer::getInstance());
            BinderMessage msg1;
            msg1.what = BINDER_MESSAGE_RESTORE_CONNECTIONLESS_STATE;
            msg1.json = info->data.app.app_name;
            mgr.sendMessage(msg1);
#if 0
            string clientName = mgr.getClientName(info->data.app.app_name);
            if (clientName == "") {
                DEBUG("same untracked package, ignored");
                break;
            }

            sp<OIfaceClient> client = mgr.asOIfaceClient(clientName.c_str());
            if (client == NULL) {
                ERROR("tracked package but not client, race condition??");
                break;
            }

            DEBUG("tracked package maybe screen off/on");
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_RESTORE_STATE;
            msg.json = clientName.c_str();
            mgr.sendMessage(msg);
#endif
            break;

        }

        /* forground app changes, change wifi state accordingly */
        if (info->data.app.status == OIFACE_APP_ENTER_FORGROUND) {
#if 0
            mLock.lock();
            mLastPackage = mCurrentPackage;
            mCurrentPackage = info->data.app.app_name;
            mActivityChanged = true;
            mLock.unlock();

            OIfaceServer &mgr(OIfaceServer::getInstance());

            DEBUG("last client name:%s current client name:%s",
                    mgr.getClientName(mLastPackage).c_str(),
                    mgr.getClientName(mCurrentPackage).c_str());


            BinderMessage msg;
            /* background event */
            msg.what = BINDER_MESSAGE_BACKGROUND;
            msg.json = mLastPackage;
            if (msg.json.size() > 0)
                mgr.sendMessage(msg);
            /* cancel state handler */
            msg.what = BINDER_MESSAGE_CANCEL_STATE;
            msg.json = mgr.getClientName(mLastPackage);
            if (msg.json.size() > 0)
                mgr.sendMessage(msg);

            /* restore state handler */
            msg.what = BINDER_MESSAGE_RESTORE_STATE;
            msg.json = mgr.getClientName(mCurrentPackage);
            if (msg.json.size() > 0)
                mgr.sendMessage(msg);

            /* forgroud message handler */
            msg.what = BINDER_MESSAGE_FORGROUND;
            msg.json = mCurrentPackage;
            mgr.sendMessage(msg);
#endif
        } else if (info->data.app.status == OIFACE_APP_EXIT_FORGROUND) {
            mLock.lock();
            mCurrentPackage = "";
            mPid = -1;
            mLock.unlock();
        } else {
            ERROR("unexpected status:%d\n",
                    info->data.app.status);
        }

        break;

    case OIFACE_REQUEST_CMD_NETWORK:
        /* change back to wifi, restore wifi state accordingly */
        DEBUG("network status:%d\n", info->data.network.status);
        if (info->data.network.status == OIFACE_NETWORK_DATA_ON_WLAN) {
            mWifiStatus = 1;
            OIfaceServer &mgr(OIfaceServer::getInstance());

            /* ever tracked package */
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_RESTORE_STATE;
            msg.json = mgr.getClientName(mCurrentPackage);
            mgr.sendMessage(msg);

        } else if (info->data.network.status == OIFACE_NETWORK_DATA_OFF_WLAN)
            mWifiStatus = 0;
        else if (info->data.network.status == OIFACE_NETWORK_DATA_ON_WWAN)
            mDataStatus = 1;
        else if (info->data.network.status == OIFACE_NETWORK_DATA_OFF_WWAN)
            mDataStatus = 0;

        break;
    }
}

int ThreadState::getConfig(int uid, uint32_t *value) {
    if (value == NULL) {
        INFO("uid %d is not cached", uid);
        return -1;
    }

    map<int, uint32_t>::iterator iter = mUidConfig.find(uid);

    if (iter != mUidConfig.end()) {
        *value = iter->second;
        return 0;
    }

    INFO("config value for uid %d not found", uid);
    return -1;
}

int ThreadState::setConfig(int uid, uint32_t value) {
        mUidConfig[uid] = value;
            return 0;
}
