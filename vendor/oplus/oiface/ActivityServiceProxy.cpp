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

#include <utils/Unicode.h>
#include <utils/RefBase.h>
#include <ActivityServiceProxy.h>
#include "OIface.h"

using namespace std;

#define BROADCAST_DUMP_SWITCH true

class BpActivityServiceProxy: public BpInterface<IActivityServiceProxy> {
public:
    BpActivityServiceProxy(const sp<IBinder>& impl) : BpInterface<IActivityServiceProxy>(impl) {}
    virtual void broadcast(String16 action, String16 packageName, String16 componentName,
        String16 permission, int intentExtrasCounts, const char ** extras) {
        if (BROADCAST_DUMP_SWITCH) {
            DEBUG("action : %s", String8(action).c_str());
            DEBUG("packageName : %s", String8(packageName).c_str());
            DEBUG("componentName : %s", String8(componentName).c_str());
            DEBUG("permission : %s", String8(permission).c_str());
            DEBUG("intentExtrasCounts : %d", intentExtrasCounts);
            for (int i = 0; i < intentExtrasCounts; i += 2) {
                DEBUG("key   : %s", extras[i]);
                DEBUG("value : %s", extras[i+1]);
            }
        }

        Parcel data;
        data.writeInterfaceToken(IActivityServiceProxy::getInterfaceDescriptor());
        data.writeStrongBinder(NULL);
        data.writeInt32(1); /* check whether intent is null or not */
        data.writeString16(action);
        data.writeInt32(0); /* Uri - type */
        data.writeString16(NULL, 0); /* type */
        data.writeInt32(0); /* flags */
        if (packageName != String16()) {
            data.writeString16(packageName);
        } else {
            data.writeString16(NULL, 0);
            INFO("packageName is NULL");
        }
        if (componentName != String16()) {
            data.writeString16(packageName);
            data.writeString16(componentName);
        } else {
            data.writeString16(NULL, 0);
            INFO("componentName is NULL");
        }
        data.writeInt32(0); /* source bound - size */
        data.writeInt32(0); /* Categories - size */
        data.writeInt32(0); /* selector - size */
        data.writeInt32(0); /* ClipData */
        data.writeInt32(0); /* contentUserHint */
        data.writeInt32(0); /*  OplusUserId  */
        data.writeInt32(0); /*  IsFromGameSpace  */
        data.writeInt32(0); /*  IsForFreeForm  */
        data.writeInt32(0); /*  StackId  */

        /* Intent extras */
        if (intentExtrasCounts != 0) {
            int lengthPos = data.dataPosition();
            data.writeInt32(intentExtrasCounts/2); /* length */
            data.writeInt32(0x4C444E42); // 'B' 'N' 'D' 'L'

            int startPos = data.dataPosition();
            data.writeInt32(intentExtrasCounts/2); /* size */
            for (int i = 0; i < intentExtrasCounts; i += 2) {
                data.writeString16(String16(extras[i]));
                data.writeInt32(0);
                data.writeString16(String16(extras[i+1]));
            }
            int endPos = data.dataPosition();

            data.setDataPosition(lengthPos);
            data.writeInt32(endPos - startPos); /* length */
            data.setDataPosition(endPos);
        } else {
            data.writeInt32(-1);
            INFO("intentExtrasCounts is 0");
        }

        /* Others */
        data.writeString16(NULL, 0); /* resolvedType */
        data.writeStrongBinder(NULL); /* resultTo */
        data.writeInt32(-1); /* result code */
        data.writeString16(NULL, 0); /* result data */
        data.writeInt32(0); /* no result extra */
        if (permission != String16()) {
            data.writeInt32(1); /* permission length */
            data.writeString16(permission);
        } else {
            data.writeInt32(-1);
            INFO("permission is NULL");
        }
        data.writeInt32(0); /* app operation in AppOpsManager */
        data.writeInt32(0); /* options */
        data.writeInt32(0); /* serialized */
        data.writeInt32(0); /* sticky */
        data.writeInt32(-1); /* user id */

        status_t result = remote()->transact(BROADCAST_INTENT_TRANSACTION, data, nullptr,
                IBinder::FLAG_ONEWAY);
        if (result != NO_ERROR) {
            ERROR("%s, failed to transact: %d", __func__, result);
        }
    }

    virtual void broadcast(String16 action, String16 packageName) {
        return;
#if 0
        Parcel data;

        data.writeInterfaceToken(IHypnusServiceProxy::getInterfaceDescriptor());
        data.writeString16(action);
        data.writeString16(packageName);

        if (remote()->transact(HYPNUS_SEND_BROADCAST, data, nullptr, IBinder::FLAG_ONEWAY) < 0) {
            ERROR("transact broadcast failed");
        }
#endif
    }
};

IMPLEMENT_META_INTERFACE(ActivityServiceProxy, "android.app.IActivityManager");
