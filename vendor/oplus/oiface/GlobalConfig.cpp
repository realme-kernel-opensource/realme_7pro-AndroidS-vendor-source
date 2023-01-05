#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>

#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>
#include <utils/Mutex.h>
#include <utils/Singleton.h>

#include <cutils/properties.h>

#include "GlobalConfig.h"
#include "OIface.h"
#include "PlatformAdaptor.h"

using namespace std;
using namespace android;

#define CONFIG_NAME     "oiface.config"

// From Android R, oiface.config change to system_ext partition
#define SYSTEM_CONFIG_PATH  "/system_ext/etc/oiface/oiface.config"

/*
const uint8_t GlobalConfig::gKey[AES_BLOCK_SIZE * 32] = {
#include "ocrypto/ocrypt_key_in.h"
};
*/

ANDROID_SINGLETON_STATIC_INSTANCE(GlobalConfig);

// remove crypto
GlobalConfig::GlobalConfig() {
    Json::Reader reader;
    Json::Value root;
    string str; // , out;

    /* parse done */
    parseProperties();

    if (getFileContents(&str) < 0) {
        ERROR("get file contents failed\n");
        return;
    }

    /*
    if (decryptConfig(str, &out) < 0) {
        ERROR("read config failed");
        return;
    }
    */

    // if (!reader.parse(out, root, false)) {
    if (!reader.parse(str, root, false)) {
        ERROR("parse config file failed");
    } else {
        /* tried my best to get config value */
        getGlobalConfig(root);
    }
}

int GlobalConfig::getFileContents(string *output) {
    struct stat st;
    ssize_t len;
    string realPath;

    const char *path = getenv(OIFACE_LOCATION_KEY);
    if (path == NULL) {
        ERROR("get env failed");
        return -1;
    }
    DEBUG("OIFACE_LOCATION_KEY: %s", path);

    if (strncmp(path, "/system", strlen("/system"))) { /* not system */
        realPath = realPath + dirname(path) + "/" + CONFIG_NAME;
    } else { /* system */
        realPath = SYSTEM_CONFIG_PATH;
    }

    int fd = open(realPath.c_str(), O_RDONLY);
    if (fd < 0) {
        ERROR("open %s failed(%s)", realPath.c_str(),
                strerror(errno));
        return -1;
    }

    if (fstat(fd, &st) < 0) {
        ERROR("stat %s failed(%s)", CONFIG_NAME, strerror(errno));
        close(fd);
        return -1;
    }

    DEBUG("file size: %d", (int)st.st_size);
    char *buffer = (char *)malloc(st.st_size);
    if (buffer == NULL) {
        ERROR("malloc failed(%s)", strerror(errno));
        close(fd);
        return -1;
    }

    if ((len = read(fd, buffer, st.st_size)) < st.st_size) {
        ERROR("read of %ld bytes instead of %ld bytes(%s)", len, st.st_size, strerror(errno));
        free(buffer);
        close(fd);
        return -1;
    }

    output->assign(buffer, st.st_size);

    close(fd);
    free(buffer);

    return 0;
}

#if 0
int GlobalConfig::decryptConfig(const string& in, string *str) {

    AES_KEY aesKey;
    uint32_t size;
    int blkLen;

    memset(&aesKey, 0, sizeof(AES_KEY));

    uint8_t key[AES_BLOCK_SIZE];
    for (int i = 0; i < AES_BLOCK_SIZE; i++) { /* hard code to mess up */
        key[i] = gKey[i] ^ gKey[i * 2 + AES_BLOCK_SIZE * 2 + 5]; /* maximum value is 4*AES_BLOCK_SIZE + 5*/
    }

    if (AES_set_decrypt_key(key, AES_BLOCK_SIZE * 8, &aesKey) < 0) {
        ERROR("Unable to set encryption key in AES...");
        return -1;
    }
    uint8_t ivec[AES_BLOCK_SIZE];
    memset(ivec, 0, sizeof(ivec));

    blkLen = (in.size() + AES_BLOCK_SIZE - 1) & ~(AES_BLOCK_SIZE - 1);
    DEBUG("in size:%ld, alligned to %d bytes", in.size(), blkLen);

    uint8_t *out = (uint8_t *)malloc(blkLen);
    if (out == NULL) {
        ERROR("malloc %d bytes faield(%s)", blkLen, strerror(errno));
        return -1;
    }

    AES_cbc_encrypt((uint8_t *)in.data(), out, blkLen, &aesKey, ivec, AES_DECRYPT);

    size = *(uint32_t*)&out[0];
    DEBUG("size is %d\n", size);
    /* 'blkLen' is filled size. 'size' is original file size, plus sizeof(uint32_t), should not cross block boundary. */
    if (((blkLen - (size + 4)) > AES_BLOCK_SIZE) || (out[sizeof(uint32_t)] != '{')) {
        ERROR("size in file is %d while block size is %d\n", size, blkLen);
        free(out);
        return -1;
    }

    str->assign((const char *)&out[sizeof(uint32_t)], size);

    free(out);

    return 0;
}
#endif


/* XXX Note:
  There're two properties: persist.sys.oiface.enable & persist.sys.oiface.feature.
  The first one with value 0,1 or 2 is to control oiface completely off, partialy on or
  completely on.
  The second one is to control oiface feature seperately. So there're following combinations:
  +-------------------------------------------------------------------------+
  | persist.sys.oiface.enable | persist.sys.oiface.feature | mConfig result |
  |           None            |               *            |oiface all off  |
  |           0               |               *            |oiface all off  |
  |           1               |               *            |       0        |
  |           2               |               X            |       X        |
  +-------------------------------------------------------------------------+
*/
/* back compatible FIXME:add oiface property control logic */
void GlobalConfig::parseProperties() {
    char def[PROPERTY_VALUE_MAX];
    string propName;
    char buf[PROPERTY_VALUE_MAX];
    char *str1, *str2, *saveptr1, *saveptr2;
    char *token, *subtoken;
    int i;

    buf[0] = '\0';

    snprintf(def, sizeof(def), OIFACE_FEATURE_PROPERTY_PREFIX":%x",
            OIFACE_FEATURE_DEFAULT_VALUE);

    property_get(OIFACE_FEATURE_PROPERTY, buf, def);

    INFO("feature config:%s", buf);

    if (strncmp(OIFACE_FEATURE_PROPERTY_PREFIX, buf,
                strlen(OIFACE_FEATURE_PROPERTY_PREFIX)) == 0) {
        string key, value;
        for (str1 = buf;; str1 = NULL) {
            token = strtok_r(str1, ",", &saveptr1);
            if (token == NULL)
                break;

            for (i = 0, str2 = token; ; str2 = NULL, i++) {
                subtoken = strtok_r(str2, ":", &saveptr2);
                if (subtoken == NULL) {
                    if (i == 2) {
                        if (mJsonRoot[key]["enable"] == Json::nullValue)
                            mJsonRoot[key]["enable"] = (uint32_t)strtoul(value.c_str(), NULL, 16);
                    }
                    break;
                }

                if (i == 0)
                    key = subtoken;

                if (i == 1)
                    value = subtoken;
            }
        }
    }

}

int GlobalConfig::getGlobalConfig(Json::Value& root) {
    string plat, prj;
    Json::Value& defval = root["default"];

    /* update with platform config */
    plat = PlatformAdaptor::getInstance().getPlatformName();
    if (root[plat] != Json::Value::null) {
        updateJson(defval, root[plat]);
    }
    /* update with project config */
    prj = PlatformAdaptor::getInstance().getProject();
    if (root[prj] != Json::Value::null) {
        updateJson(defval, root[prj]);
    }
    mJsonRoot = defval;

    return 0;
}

int GlobalConfig::updateJson(Json::Value& dst, const Json::Value& src) {
    string key;
    Json::Value::Members members;
    Json::Value::Members::iterator it;

    members = src.getMemberNames();

    for (it = members.begin(); it != members.end(); it++) {
        key = *it;
        const Json::Value& srcvalue = src[key];
        if (srcvalue == Json::Value::null)
            continue;
        Json::Value& dstvalue = dst[key];
        if (dstvalue == Json::Value::null || dstvalue.type() != srcvalue.type()) {
            dstvalue = srcvalue;
            continue;
        }
        if (srcvalue.isObject()) {
            updateJson(dstvalue, srcvalue);
        } else {
            dstvalue = srcvalue;
        }
    }

    return 0;
}

int GlobalConfig::getConfig(Json::Value *value, vector<string> item) {
    Json::Value next = mJsonRoot;

    for (vector<string>::iterator iter = item.begin(); iter != item.end(); iter++) {
        next = next[*iter];
        if (next == Json::nullValue) {
            return -1;
        }
    }

    *value = next;
    return 0;
}

int GlobalConfig::setConfig(const Json::Value &v) {
    /* TODO: check if valid */
    return updateJson(mJsonRoot, v);
}

int GlobalConfig::getConfig(string *value, vector<string> item) {
    Json::Value v;

    if (getConfig(&v, item) < 0) {
        return -1;
    }

    if (!v.isString()) {
        ERROR("%s is not string", v.toStyledString().c_str());
        return -1;
    }

    *value = v.asString();

    return 0;
}

int GlobalConfig::getConfig(std::vector<std::string> &value, std::vector<std::string> item) {
    Json::Value v;

    if (getConfig(&v, item) < 0) {
        return -1;
    }
    if (v.isString()) {
        value.push_back(v.asString());
    } else if (v.isArray()) {
        for (int i = 0; i < (int)v.size(); i++) {
            value.push_back(v[i].asString());
        }
    } else {
        ERROR("%s is not string or array", v.toStyledString().c_str());
        return -1;
    }

    return 0;
}

int GlobalConfig::getConfig(uint32_t *value, vector<string> item) {
    Json::Value v;

    if (getConfig(&v, item) < 0) {
        return -1;
    }

    if (!v.isUInt()) {
        ERROR("%s is not uint", v.toStyledString().c_str());
        return -1;
    }

    *value = v.asUInt();

    return 0;
}

int GlobalConfig::getConfig(bool *value, vector<string> item) {
    Json::Value v;

    if (getConfig(&v, item) < 0) {
        return -1;
    }

    if (!v.isBool()) {
        ERROR("%s is not bool", v.toStyledString().c_str());
        return -1;
    }

    *value = v.asBool();

    return 0;
}

string GlobalConfig::getConfig() {
    return mJsonRoot.toStyledString();
}
