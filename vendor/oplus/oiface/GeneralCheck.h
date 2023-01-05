#ifndef __GENERAL_CHECK_H__
#define __GENERAL_CHECK_H__

#include <utils/Singleton.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/rsa.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <stdio.h>
#include <string>
#include <map>

using namespace std;

class GeneralCheck: public android::Singleton<GeneralCheck> {
    public:
        GeneralCheck();
        ~GeneralCheck();
        bool testGeneralCheck();
        int checkPermission(const std::string & permissionID, int uid);
        std::string decryptPkgName(const std::string &text,string &errorInfo);
        void removeConfig(int uid);
        int findConfig(int uid);
        int getConfig(int uid, struct GeneralConfig &config);
        void setConfig(int uid, const struct GeneralConfig &config);
    private:
        enum RSA_TYPE {
            RSA_PRIVATE_ENCRYPT = 0,
            RSA_PUBLIC_DECRYPT = 1,
        };
        enum PERMISSIONID_TYPE {
            PERMISSIONID_DATE = 0,
            PERMISSIONID_VERSION,
            PERMISSIONID_TYPE,
            PERMISSIONID_HASH,
            PERMISSIONID_CONF,
            PERMISSIONID_COUNT,
        };

       enum GENERAL_DUMP_TYPE {
            GENERAL_DUMP_PKG = 0,
            GENERAL_DUMP_DATE,
            GENERAL_DUMP_OTHER,
            GENERAL_DUMP_COUNT,
        };

        enum CONFIG_TYPE {
            CONFIG_BURST_INTERVAL = 0,
            CONFIG_BURST_TIME,
            CONFIG_LOAD_INTERVAL,
            CONFIG_LOAD_TIME,
            CONFIG_ACC_RATIO,
            CONFIG_COUNT,
        };

        std::map<int, struct GeneralConfig> mConfig;
        std::string base64Encode(const std::string &text);
        std::string base64Decode(const std::string &text);
        void split(const string& s,vector<int>& sv,const char flag = ' ');
        std::string rsaPrivateEncrypt(const std::string &text, const std::string &key);
        std::string rsaPublicDecrypt(const std::string &text, const std::string &key);
        RSA* createRSA(const std::string &key, int rsaType);
        std::string mSeparator = "--";
        std::string mTest = "99999--10--0--4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9--"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END"\
                            "START4177B29126A6D22F097C50F2C71A17BA47281E6BD0C482631D67E91B4C3C3FB9END";
        std::string mTestPrivateKey = "-----BEGIN RSA PRIVATE KEY-----\n"\
                "MIICXQIBAAKBgQDbRLK7ucTlLdezySSLLka/YvFbuMjnFV1ZcXtw2VVFE8+HbST8\n"\
                "hBhGYCIguI5d877lpS/yP8RgeinIf+OV8H7VqGCuUzYod9znZZ5kAEY4MJ4Q5mRK\n"\
                "oKtFtPawVQkporVOZBgh2hFID1AcWowXcC0+q03rjGhMZdMCiL2AnKZ7qwIDAQAB\n"\
                "AoGBAJTx14+Zo1wihdHEoqRArSI4rccmgKIoax7k7Xs8xqWDzYcq5uL5QBweFVCw\n"\
                "zVSYzGXmjek608TNdzCRXyFtGln6GRcodbliD0u1pbPqm8tUexlxlIsvdoAKBaSu\n"\
                "3YkUVCD42JdaQM7sGUOOAEKwES9VbTa3OOanFvGO70I0VnMRAkEA+UTRMSyBWXTC\n"\
                "6GInIXg+OXJPzZ6GSxIpJFrbYeblg4l36B1Wtfgmz9wNcM0cs3vreOTcHMEJxRgY\n"\
                "AAP+2qBXfwJBAOEwfUN0VjAOkBY5euSr9+qwnSb/0cR4HkCk8rMnGMlylZQoPoOL\n"\
                "M4GVNL51GtKpBlUbDGW68GtDtHUNpDGx0dUCQQCR9G+hIXQj0zCFLWPIP0YAESUV\n"\
                "OutbbajVdrXaX36oRIgHS5aIOmNPdoQQNqnFlpvMUAVqeMWluHPlHnHNxfE1AkBQ\n"\
                "3GWOjD7KoOASau97H1k1l9fGgkPjxbIvSkuwExDufUvC2LSh7aFAHcDDxy8rh/Az\n"\
                "LTRFb1wXWEd7ZLuZ8J1FAkB/pZ5zFXlk5sYxXMpUmIzV4QnMEyxu6+6C8b/xgiH3\n"\
                "GQamQjOq3ff8mCNrQni4KCpkqMsxguCSjic/BEC94At+\n"\
                "-----END RSA PRIVATE KEY-----\n";
        std::string mTestPublicKey = "-----BEGIN PUBLIC KEY-----\n"\
                "MIGfMA0GCSqGSIb3DQEBAQUAA4GNADCBiQKBgQDbRLK7ucTlLdezySSLLka/YvFb\n"\
                "uMjnFV1ZcXtw2VVFE8+HbST8hBhGYCIguI5d877lpS/yP8RgeinIf+OV8H7VqGCu\n"\
                "UzYod9znZZ5kAEY4MJ4Q5mRKoKtFtPawVQkporVOZBgh2hFID1AcWowXcC0+q03r\n"\
                "jGhMZdMCiL2AnKZ7qwIDAQAB\n"\
                "-----END PUBLIC KEY-----\n";
        std::string  mPublicKey = "-----BEGIN PUBLIC KEY-----\n"\
                "MIGfMA0GCSqGSIb3DQEBAQUAA4GNADCBiQKBgQDWcMFTSLmCII9/B73pfHCrLKbm\n"\
                "usPp0pVo/MNH9JoDpVuyDRmbPyLdq18pADqu90++p1HvIzTOfjLZ2VetMshbEb14\n"\
                "+lClKjicSYM5G8pw9vNlVVjWkLlyL2IJTWaQtL4DGQMDohpGWNDgM0xRIiZQBkqP\n"\
                "gCSgBIGNuE2XYt7p/QIDAQAB\n"\
                "-----END PUBLIC KEY-----\n";
};

#endif
