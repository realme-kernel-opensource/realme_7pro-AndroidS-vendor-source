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

#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <libgen.h>

#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include "OIfaceModuleLoader.h"
#include "ThreadState.h"
#include "PlatformAdaptor.h"
#include "Utils.h"
#include "GlobalConfig.h"
#include "GeneralCheck.h"

#define OIFACE_MODULE_PREFIX    "liboiface"
#define DECISION_LENG 5

int OIfaceModuleLoader::getFileList(const char *p, vector<string>& str) {
    struct dirent** namelist;
    int n;
    char path[PATH_MAX];
    char file[PATH_MAX];
    int len;

    snprintf(path, sizeof(path), "%s", p);
    strcat(path, "/");
    len = strlen(path);

    n = scandir(p, &namelist, 0, alphasort);
    if (n < 0) {
        ERROR("scandir %s failed.(%s)\n", p, strerror(errno));
        return -1;
    } else {
        while (n--) {
            DEBUG("file name:%s\n", namelist[n]->d_name);
            if (namelist[n]->d_type == DT_REG) {
                int l = strlen(OIFACE_MODULE_PREFIX);
                if ((strncmp(namelist[n]->d_name, OIFACE_MODULE_PREFIX, l) == 0) &&
                        isalpha(*(namelist[n]->d_name + l))) { /* omit liboiface.so */
                    snprintf(file, sizeof(file), "%s%s", path, namelist[n]->d_name);
                    str.push_back(file);
                }
            }
            free(namelist[n]);
        }
        free(namelist);
    }

    return 0;
}

OIfaceModuleLoader::OIfaceModuleLoader()
    : mIsInitialized(false)
    , mModule(NULL)
    , mPackageName(NULL)
    , mModuleInfo(NULL)
    , mKeyList(NULL) {

}

OIfaceModuleLoader::OIfaceModuleLoader(pid_t pid, uid_t uid, std::string typeName):
    mIsInitialized(false), mModule(NULL), mPackageName(NULL), mModuleInfo(NULL), mKeyList(NULL) {
    void* module = NULL;
    struct OIfaceModuleInfo* info = NULL;
    int ret;
    int i;
    vector<std::string> clientPackageName;
    vector<std::string> files;
    bool isPkgDebuggable = false;
    bool isInWhiteList = false;
    string curPackageName;

    memset(&mOIfaceInterface, 0, sizeof(mOIfaceInterface));

    const char *path = getenv(OIFACE_LOCATION_KEY);
    if (path == NULL) {
        ERROR("getenv failed");
        return;
    }

    if (getPackageForUid(uid, clientPackageName) < 0) {
        ERROR("get package for uid:%d failed", uid);
        return;
    }

    /* scan data&system partition for libraries and append to files . data partition ones apears first */
    if (getFileList(dirname(path), files) < 0) {
        DEBUG("getFilesList from %s failed\n", dirname(path));
    }

    if (files.size() == 0) {
        ERROR("get module files faield\n");
        return;
    } else {
        for (const auto& file: files) {
            DEBUG("scaned library files:%s", file.c_str());
        }
    }

    /* call initializer in library */
    for (vector<string>::iterator iter = files.begin();
            iter != files.end(); iter++) {

        DEBUG("opening %s\n", iter->c_str());
        module = dlopen(iter->c_str(), RTLD_NOW);
        if (module == NULL) {
            ERROR("dlopen %s failed.(%s)\n",
                    iter->c_str(), dlerror());
            continue;
        }

        info = (struct OIfaceModuleInfo*)dlsym(module, "OIMN");
        if (info == NULL) {
            ERROR("dlsym %s failed(%s)\n", "OIMN",
                    dlerror());
            dlclose(module);
            continue;
        }

        INFO("info get success\n");
        INFO("tag:0x%x\n", info->tag);
        INFO("interface_version:0x%x\n", info->interface_version);
        INFO("module_version:0x%x\n", info->module_version);
        INFO("name:%s\n", info->name);

        if (typeName != info->name) {
            ERROR("type name not match(need:%s got:%s)", typeName.c_str(), info->name);
            dlclose(module);
            continue;
        }

        for (vector<string>::iterator iter = clientPackageName.begin(); iter != clientPackageName.end(); iter++) {
            if (isAppDebuggable(android::String16((*iter).c_str()))) {
                isPkgDebuggable = true;
                curPackageName = *iter;
                break;
            }
        }

        if (!isPkgDebuggable) {
            i = 0;
            while (info->package_name[i] != NULL) {
                if (find(clientPackageName.begin(), clientPackageName.end(), info->package_name[i]) ==
                        clientPackageName.end()) {
                    INFO("skip package %s\n", info->package_name[i]);
                } else {
                    isInWhiteList = true;
                    curPackageName = info->package_name[i];
                    INFO("found package %s\n", info->package_name[i]);
                    break;
                }
                i++;
            }

            if (info->package_name[i] == NULL) {
                curPackageName = clientPackageName[0];
                ERROR("unable to find package %s, start to check permission", curPackageName.c_str());
            }
        }

        /* get configuration mask for this module */
        if (GlobalConfig::getInstance().getConfig(&mOIfaceInterface.config_mask,
                    {info->name, "enable"}) < 0) {
            mOIfaceInterface.config_mask = UINT_MAX;
        }

        memset(mOIfaceInterface.parsedDecisionsMap, -1, sizeof(mOIfaceInterface.parsedDecisionsMap));

        /* Get decisions of scenes from config file.
         * the formats of configs in the config file need to as follows:
         *  "com.xx.mobileqq": {
         *      "decisions": {
         *         "101": [ 1, 15, 888, 0, 0],
         *         "301": [ 1, 15, 666, 0, 0]
         * }
        */
        Json::Value configs;
        GlobalConfig::getInstance().getConfig(&configs, {info->name, curPackageName.c_str(), "decisions"});
        int32_t sceneIdx = 0;
        for (Json::ValueConstIterator iter = configs.begin(); iter != configs.end(); iter++, sceneIdx++) {
            mOIfaceInterface.parsedDecisionsMap[sceneIdx][0] = atoi(iter.key().asCString());
            for (int32_t decisonIdx = 0; decisonIdx < DECISION_LENG; decisonIdx++) {
                mOIfaceInterface.parsedDecisionsMap[sceneIdx][decisonIdx+1]
                    = configs[iter.key().asCString()][decisonIdx].asInt();
            }
        }

        /* cache configuration for uid */
        ThreadState::getInstance().setConfig(uid, mOIfaceInterface.config_mask);

        if ((mOIfaceInterface.config_mask & MODULE_ENABLE_MASK) == 0) {
            ERROR("module %s is disabled", info->name);
            dlclose(module);
            continue;
        }
        mPackageName = strdup(curPackageName.c_str());
        if(mPackageName == NULL) {
            ERROR("low memory");
            dlclose(module);
            continue;
        }
        mOIfaceInterface.tag = OIFACE_INTERFACE_TAG;
        mOIfaceInterface.interface_version = 1;
        mOIfaceInterface.private_data = NULL;
        mOIfaceInterface.permission_id = NULL;
        /* tell strategy which strategy should use */
        mOIfaceInterface.package_name = mPackageName;
        mOIfaceInterface.platform = PlatformAdaptor::getInstance().getPlatform();
        mOIfaceInterface.pid = pid;
        mOIfaceInterface.uid = uid;
        mOIfaceInterface.send_log = sendLog;

        /* fill board suffix to interface in case of need it */
        mOIfaceInterface.board[MAX_BOARD_NAME - 1] = '\0';
        strncpy(mOIfaceInterface.board, PlatformAdaptor::getInstance().getBoardName().c_str(),
            MAX_BOARD_NAME - 1);

        memset(mOIfaceInterface.reserved, 0,
                ARRAY_SIZE(mOIfaceInterface.reserved));

        if (info->protocol == OIFACE_PROTOCOL_OIM) {
            ret = info->oim_init(&mOIfaceInterface, NULL);
            if (ret < 0) {
                ERROR("initialize module %s failed.\n",
                        info->name);
                dlclose(module);
                continue;
            }
        } else {
            DEBUG("calling initialize");
            ret = info->initialize(&mOIfaceInterface, &mKeyList);
            if ((ret < 0) || (mKeyList == NULL)) {
                ERROR("initialize module %s failed.\n",
                        info->name);
                dlclose(module);
                continue;
            }
        }

        mModule = module;
        mModuleInfo = info;

        INFO("done initialize module %s\n", info->name);
        if(isInWhiteList)
            mIsInitialized = true;
        break;
    }
}

string OIfaceModuleLoader::getPackageName() {
    if (mIsInitialized)
        return mOIfaceInterface.package_name;

    return "";
}

int OIfaceModuleLoader::getDecision(Decision decision[], int maxDecisions,
        const map<string, JsonValue> &jsonValueMap) {

    struct key_value_map* list;
    struct json_key *key = NULL;
    int i;
    int num;

    ATRACE_CALL();

    if (!mIsInitialized) {
        ERROR("module is not initialized");
        return 0;
    }

    decision[0].type = DECISION_TYPE_NONE;

    if (jsonValueMap.size() <= 0) {
        return 0;
    }

    list = (struct key_value_map*)malloc(sizeof(struct key_value_map) *
                    (jsonValueMap.size() + 1));
    if (list == NULL) {
        ERROR("new key_value_map failed(%s)\n", strerror(errno));
        return 0;
    }

    for (key = mKeyList, i = 0; strlen(key->key_name) != 0; key++) {
        const map<string, JsonValue>::const_iterator viter =
                jsonValueMap.find(key->key_name);
        if (viter != jsonValueMap.end()) {
            list[i].value = viter->second;
            list[i].key = *key;
            i++;
        }
    }

    /* Last value */
    strcpy(list[i].key.key_name, "");

    DEBUG("calling make decision");
    num = mModuleInfo->make_decision(&mOIfaceInterface, list,
                    decision, maxDecisions);

    free(list);

    return num;
}

int OIfaceModuleLoader::getDecision(Decision decision[], int maxDecisions,
            const OimMessage *msg) {
    int num;

    ATRACE_CALL();

    if (!mIsInitialized) {
        ERROR("module is not initialized");
        return 0;
    }

    decision[0].type = DECISION_TYPE_NONE;

    num = mModuleInfo->oim_make_decision(&mOIfaceInterface, msg, decision, maxDecisions);

    return num;
}

int OIfaceModuleLoader::getSupportPackage(std::string &packages) {
// int OIfaceModuleLoader::getSupportPackage(android::binder::Map *packages) {

    // FIXME map: fix this code, change package to json string; DONE
    Json::Value packagesRoot;

    void *module = NULL;
    struct OIfaceModuleInfo *info = NULL;

#if 0
    if (packages == NULL) {
        ERROR("invalid package parameter");
        return -1;
    }
#endif

    const char *path = getenv(OIFACE_LOCATION_KEY);
    if (path == NULL) {
        ERROR("getenv failed");
        return -1;
    }

    vector<std::string> files;

    /* scan data&system partition for libraries and append to files . data
    partition ones apears first */
    if (getFileList(dirname(path), files) < 0) {
        DEBUG("getFilesList from %s failed\n", dirname(path));
    }

    if (files.size() == 0) {
        ERROR("get module files faield\n");
        return -1;
    } else {
        for (const auto& file: files) {
            DEBUG("scaned library files:%s", file.c_str());
        }
    }

    /* call initializer in library */
    for (vector<string>::iterator iter = files.begin();
            iter != files.end(); iter++) {
        DEBUG("opening %s\n", iter->c_str());
        module = dlopen(iter->c_str(), RTLD_NOW);
        if (module == NULL) {
            ERROR("dlopen %s failed.(%s)\n",
                    iter->c_str(), dlerror());
            continue;
        }

        info = (struct OIfaceModuleInfo*)dlsym(module, "OIMN");
        if (info == NULL) {
            ERROR("dlsym %s failed(%s)\n", "OIMN",
                    dlerror());
            dlclose(module);
            continue;
        }

        uint32_t mask;

        if (GlobalConfig::getInstance().getConfig(&mask, {info->name, "enable"}) < 0) {
            mask = UINT_MAX;
        }

        for (int i = 0; info->package_name[i] != NULL; i++) {
            if (info->is_package_enabled == NULL) /* incompatible version */
                continue;
            // (*packages)[info->package_name[i]] = info->is_package_enabled(mask, info->package_name[i]) ?
            //     android::String16("enabled"):android::String16("disabled");
            packagesRoot[info->package_name[i]] = info->is_package_enabled(mask, info->package_name[i]) ?
                std::string("enabled") : std::string("disabled");
        }

        dlclose(module);
    }

    packages.append(packagesRoot.toStyledString());
    DEBUG("json string: %s", packages.c_str());

    return 0;
}

int OIfaceModuleLoader::checkPermission(const OimMessage *msg) {
    ATRACE_CALL();
    if (mModuleInfo->oim_check_permission(&mOIfaceInterface, msg) < 0)
        return -1;
    if(mOIfaceInterface.permission_id != NULL) {
        mPermissionID = mOIfaceInterface.permission_id;
        mOIfaceInterface.permission_id = NULL;
    }
    if(GeneralCheck::getInstance().checkPermission(mPermissionID, mOIfaceInterface.uid) < 0) {
        return -1;
    } else {
        mIsInitialized = true;
        return 0;
    }
}

OIfaceModuleLoader::~OIfaceModuleLoader() {
    DEBUG("module loader dectructor called\n");
    if (mModule != NULL) {
        if (mModuleInfo->deinit != NULL) {
            mModuleInfo->deinit(&mOIfaceInterface);
        }
        DEBUG("close module\n");
        dlclose(mModule);
    }
    if(mPackageName != NULL)
        free(mPackageName);
}
