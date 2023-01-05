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

#ifndef __OIFACE_MODULE_LOADER_H__
#define __OIFACE_MODULE_LOADER_H__

#include <string>
#include <map>
#include <vector>
#include <binder/Parcel.h>

#include "OIface.h"

class OIfaceModuleLoader {
public:
    static int getSupportPackage(std::string &packages);
    // static int getSupportPackage(android::binder::Map *packages);
    virtual int getDecision(Decision decision[], int maxDecisions,
            const std::map<std::string, JsonValue> &jsonValueMap);
    virtual int getDecision(Decision decision[], int maxDecisions,
            const OimMessage *msg);
    int checkPermission(const OimMessage *msg);
    /* name to search for a compatible module */
    OIfaceModuleLoader(pid_t pid, uid_t uid, std::string typeName);
    OIfaceModuleLoader();
    virtual bool isInitialized() {
        return mIsInitialized;
    }
    virtual ~OIfaceModuleLoader();

    const struct json_key* getJsonKey() {return mKeyList;};
    std::string getPackageName();
    static int getFileList(const char *p, std::vector<std::string>& str);
private:
    DISALLOW_COPY_AND_ASSIGN(OIfaceModuleLoader);
    bool mIsInitialized;
    void* mModule;
    char* mPackageName;
    std::string mPermissionID;
    struct OIfaceModuleInfo* mModuleInfo;
    struct OIfaceInterface mOIfaceInterface;
    struct json_key* mKeyList;
};

#endif
