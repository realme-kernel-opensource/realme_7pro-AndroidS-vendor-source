#ifndef __GLOBAL_CONFIG_H__
#define __GLOBAL_CONFIG_H__

#include <utils/Singleton.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>
#include <openssl/aes.h>

#include <vector>
#include <string>

/* maintain a runtime configuration for oiface, settings may dynamically changed  */
/* FIXME: don't use it in multi-thread context */
class GlobalConfig: public android::Singleton<GlobalConfig> {
    public:
        GlobalConfig();
        ~GlobalConfig();

        /* static configration from Json config file, return default value if not exist */
        int getConfig(Json::Value *value, std::vector<std::string> item);
        int getConfig(std::string *value, std::vector<std::string> item);
        int getConfig(std::vector<std::string> &value, std::vector<std::string> item);
        int getConfig(uint32_t *value, std::vector<std::string> item);
        int getConfig(bool *value, std::vector<std::string> item);
        int setConfig(const Json::Value &v);
        std::string getConfig();

    private:
        Json::Value mJsonRoot;
        static const uint8_t gKey[AES_BLOCK_SIZE * 32];

        void parseProperties();
        // int decryptConfig(const std::string& input, std::string *output);
        int getFileContents(std::string *output);
        int getGlobalConfig(Json::Value& root);
        int updateJson(Json::Value& dst, const Json::Value& src);
};

#endif
