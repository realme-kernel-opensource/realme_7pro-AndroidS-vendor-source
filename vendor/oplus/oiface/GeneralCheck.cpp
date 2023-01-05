#include <time.h>
#include <vector>
#include <sstream>
#include "OIface.h"
#include "GeneralCheck.h"
#include "GlobalConfig.h"
#include <binder/IServiceManager.h>
#include "Utils.h"

using namespace std;
using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE(GeneralCheck);

GeneralCheck::GeneralCheck() {
}

GeneralCheck::~GeneralCheck() {
}

int GeneralCheck::checkPermission(const string &permissionID, int uid) {
    //this function is just for test rsa and base64
    //testGeneralCheck();
    if (findConfig(uid)) {
        DEBUG("check Permission is already sucess: %d", uid);
        return 1;
    }
    int ret = -1;
    uint32_t enable = 0;
    GlobalConfig::getInstance().getConfig(&enable, {"general", "enable"});
    if (enable == 0) {
        ERROR("Oiface general feature is not supported!");
        return ret;
    }
    DEBUG("RSA current input permissionID is %s", permissionID.c_str());
    string decoded = base64Decode(permissionID);
    if (decoded.empty()) {
        ERROR("RSA failed to decode base64");
        return ret;
    }
    string decrypted = rsaPublicDecrypt(decoded, mPublicKey);
    if (decrypted.empty()) {
        ERROR("RSA failed to public decrypt");
        return ret;
    }
    int nDate;
    int curDate = (int)(ns2s(systemTime(CLOCK_REALTIME))/(3600*24));
    int version;
    int type = GENERAL_CLIENT_COUNT;
    size_t position = decrypted.find(mSeparator);
    size_t start = 0;
    vector<string> vec;
    for (int ii = 0; ii < PERMISSIONID_COUNT; ii++) {
        DEBUG("index[%d], start[%d], position[%d]", ii, (int)start, (int)position);
        if (string::npos != position) {
            vec.push_back(decrypted.substr(start, position - start));
            start = position + mSeparator.size();
            position = decrypted.find(mSeparator, start);
        } else {
            break;
        }
    }
    if (start != decrypted.length())
        vec.push_back(decrypted.substr(start));
    if (vec.size() < PERMISSIONID_HASH) {
        ERROR("RSA permissionID need at least %d element input", PERMISSIONID_CONF);
        return ret;
    }
    for (auto val : vec) {
        DEBUG("RSA permissionID context[%s]", val.c_str());
    }
    stringstream ss;
    for (int ii = PERMISSIONID_DATE; ii <= PERMISSIONID_TYPE; ii++) {
        ss << vec[ii];
        if (ii == PERMISSIONID_DATE) {
            ss >> nDate;
        } else if (ii == PERMISSIONID_VERSION) {
            ss >> version;
        } else {
            ss >> type;
        }
        ss.clear();
    }
    if (nDate < curDate) {
        ERROR("RSA PermissionID date:%d(current:%d) is out of date", nDate, curDate);
        return ret;
    } else {
        DEBUG("RSA PermissionID date:%d(current:%d)is in date", nDate, curDate);
    }
    if (type >= GENERAL_CLIENT_COUNT) {
        ERROR("RSA do not support client type :%d", type);
        return ret;
    }
    string originHashValue = getHashByUID(uid);
    DEBUG("RSA uid(%d), hash(%s)", uid, originHashValue.c_str());
    if (vec[PERMISSIONID_HASH] != originHashValue) {
        ERROR("RSA permission check is failed");
        return ret;
    }
    struct GeneralConfig generalConfig;
    if (vec.size() == PERMISSIONID_COUNT) {
        DEBUG("RAS permissionID has config input");
        vector<int> config;
        split(vec[PERMISSIONID_CONF], config, ':');
        if(config.size() < CONFIG_COUNT) {
            ERROR("RSA CONFIG need at %d element input", CONFIG_COUNT);
            return ret;
        }
        for (auto val : config) {
            DEBUG("RSA config context[%d]", val);
        }
        generalConfig.config_enable = 1;
        generalConfig.load_time = config[CONFIG_LOAD_TIME];
        generalConfig.burst_time = config[CONFIG_BURST_TIME];
        generalConfig.load_interval = config[CONFIG_LOAD_INTERVAL];
        generalConfig.burst_interval = config[CONFIG_BURST_INTERVAL];
        generalConfig.acc_ratio = config[CONFIG_ACC_RATIO];
    }
    generalConfig.general_type = type;
    setConfig(uid, generalConfig);
    return 1;
}

std::string GeneralCheck::decryptPkgName(const std::string &pkg,std::string &errorInfo) {
    string decoded = base64Decode(pkg);
    string ret("");
    if (decoded.empty()) {
        ERROR("RSA failed to decode PkgName base64");
        errorInfo = "-1";
        return ret;
    }
    string decrypted = rsaPublicDecrypt(decoded, mPublicKey);
    if (decrypted.empty()) {
        ERROR("RSA failed to PkgName public decrypt");
        errorInfo = "-1";
        return ret;
    }
    DEBUG("RSA Decrypt General DumpTool is: %s", decrypted.c_str());

    int startDate = 0;
    size_t position = decrypted.find(mSeparator);
    size_t start = 0;
    if (string::npos != position) {
        ret = decrypted.substr(start, position);
        start = position + mSeparator.size();
        position = decrypted.find(mSeparator, start);
        if (string::npos != position) {
            stringstream ss;
            ss << decrypted.substr(start, position);
            ss >> startDate;
            DEBUG("RSA Decrypt General DumpTool startDate: %d", startDate);
        } else {
            errorInfo = "-2";
            return string("");
        }
    } else {
        errorInfo = "-3";
        return ret;
    }

    int validityPeriod = 2;
    int curDate = (int)(ns2s(systemTime(CLOCK_REALTIME))/(3600));
    if ((((int)(startDate/3600))+validityPeriod) < curDate) {
         errorInfo = "-4";
         DEBUG("RSA Decrypt General DumpTool is out of Date!");
         return string("");
    }
    return ret;
}

bool GeneralCheck::testGeneralCheck() {
    string encrypted = rsaPrivateEncrypt(mTest, mTestPrivateKey);
    if (encrypted.empty()) {
        ERROR("Private Encrypt failed");
        return false;
    }
    string testEncode = base64Encode(encrypted);
    if (testEncode.empty()) {
        ERROR("base64 encode failed");
        return false;
    }
    string testDecode = base64Decode(testEncode);
    if (testDecode.empty()) {
        ERROR("base64 decode failed");
        return false;
    }
    string testBase64 = base64Encode(testDecode);
    string testBase64Decode = base64Decode(testBase64);
    string decrypted = rsaPublicDecrypt(testDecode, mTestPublicKey);
    if (decrypted.empty()) {
        DEBUG("Public Decrypt failed");
        decrypted = rsaPublicDecrypt(encrypted, mTestPublicKey);
        if(decrypted.empty()) {
            DEBUG("Origin public decrypt failed");
            return false;
        }
    }
    DEBUG("Test Sucess!");
    return true;
}

string GeneralCheck::base64Encode(const string &text) {
    int outLength = 0;
    int inLength = text.length();
    char *result = (char*)malloc(inLength*2);
    outLength = EVP_EncodeBlock((uint8_t*)result, (uint8_t*)text.data(), inLength);
    if (outLength < inLength) {
        ERROR("RSA base64 encode error");
        free(result);
        return string("");
    }
    DEBUG("RSA base64 encode input %s(len:%d)", text.c_str(), inLength);
    DEBUG("RSA base64 decode output %s(len:%d)", result, outLength);
    string encode = string(result, outLength);
    free(result);
    return encode;
}

string GeneralCheck::base64Decode(const string &text) {
    int outLength = 0;
    int inLength = text.length();
    char *result = (char*)malloc(inLength);
    outLength = EVP_DecodeBlock((uint8_t*)result, (uint8_t*)text.data(), inLength);
    if (outLength < 0) {
        ERROR("RSA base64 decode error");
        free(result);
        return string("");
    }
    outLength = outLength - outLength % 128;
    DEBUG("RSA base64 decode input %s(len:%d)", text.c_str(), inLength);
    DEBUG("RSA base64 decode output %s(len:%d)", result, outLength);
    string decode = string(result, outLength);
    free(result);
    return decode;
}

RSA* GeneralCheck::createRSA(const std::string &key, int rsaType) {
    RSA *rsa = NULL;
    BIO *bio = NULL;
    bio = BIO_new_mem_buf((uint8_t*)key.data(), -1);
    if (bio == NULL) {
        ERROR( "RSA failed to create key bio");
        return NULL;
    }
    if (rsaType == RSA_PUBLIC_DECRYPT) {
        rsa = PEM_read_bio_RSA_PUBKEY(bio, &rsa, NULL, NULL);
    } else {
        rsa = PEM_read_bio_RSAPrivateKey(bio, &rsa, NULL, NULL);
    }
    BIO_free_all(bio);
    return rsa;
}

string GeneralCheck::rsaPrivateEncrypt(const string &text, const string &key) {
    string result("");
    RSA *rsa = createRSA(key, RSA_PRIVATE_ENCRYPT);
    if (rsa == NULL) {
        ERROR("RSA failed to create private key");
        return result;
    }
    int rsaSize = RSA_size(rsa);
    int inLength = text.length();
    int block = rsaSize - 11;
    int count = inLength / block;
    if(inLength % block != 0) {
        count += 1;
    }
    char *outText = (char*)malloc(rsaSize + 1);
    std::string inText;
    int ret = 0;
    for (int i = 0; i < count; i++) {
        inText = text.substr(i * block, block);
        memset(outText, 0, rsaSize + 1);
        ret = RSA_private_encrypt(inText.length(), (uint8_t*)inText.c_str(),
                                  (uint8_t*)outText, rsa, RSA_PKCS1_PADDING );
        if (ret == rsaSize) {
            result.append(string(outText, ret));
            DEBUG("RSA cout:%d, result:%s", i, outText);
        } else {
            ERROR("RSA failed to private encrypt");
            break;
        }
    }
    free(outText);
    RSA_free(rsa);
    return result;
}

string GeneralCheck::rsaPublicDecrypt(const string &text, const string &key) {
    string result("");
    RSA *rsa = createRSA(key, RSA_PUBLIC_DECRYPT);
    if (rsa == NULL) {
        ERROR("RSA failed to create public key");
        return result;
    }
    int inLength = text.length();
    int block = RSA_size(rsa);
    int count = inLength / block;
    char *outText = (char*)malloc(block + 1);
    std::string inText;
    int ret = 0;
    for (int i = 0; i < count; i++) {
        inText = text.substr(i * block, block);
        memset(outText, 0, block + 1);
        ret = RSA_public_decrypt(inText.length(), (uint8_t*)inText.c_str(),
                                (uint8_t*)outText, rsa, RSA_PKCS1_PADDING );
        if (ret > 0) {
            result.append(string(outText, ret));
            DEBUG("RSA cout:%d, result:%s", i, outText);
        } else {
            ERROR("RSA failed to public decrypt");
            break;
        }
    }
    free(outText);
    RSA_free(rsa);
    return result;
}

void GeneralCheck::split(const string& s, vector<int>& sv, const char flag) {
    sv.clear();
    istringstream iss(s);
    string temp;
    while (getline(iss, temp, flag)) {
        sv.push_back(stoi(temp));
    }
    return;
}

int GeneralCheck::getConfig(int uid, struct GeneralConfig &config) {

    map<int, struct GeneralConfig>::iterator iter = mConfig.find(uid);

    if (iter != mConfig.end()) {
        config = iter->second;
        return 0;
    }

    INFO("config value for uid %d not found", uid);
    return -1;
}

void GeneralCheck::setConfig(int uid, const struct GeneralConfig &config) {
    mConfig[uid] = config;
}

void GeneralCheck::removeConfig(int uid) {
    mConfig.erase(uid);
    DEBUG("GeneralCheck config is removed because of binder died");
}

int GeneralCheck::findConfig(int uid) {
    return (int)(mConfig.count(uid));
}
