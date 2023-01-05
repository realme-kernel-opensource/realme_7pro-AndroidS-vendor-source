#ifndef __DATA_COLLECTOR_H__
#define __DATA_COLLECTOR_H__

#include <utils/Singleton.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>

#define MAX_DC_INTERNAL_NUM 20
#define MAX_TRACKED_NORMAL_APK  10
/* store data in a json structure */
class DataCollector: public android::Singleton<DataCollector> {
    public:
        DataCollector();
        ~DataCollector();

        enum DcType {
            DC_TYPE_CONNECTION = 0,
            DC_TYPE_INTERNAL,
        };

        int setData(DcType type, const std::string& pkg, const std::string& key, int32_t value,
                bool isAdd = false);
        int setData(DcType type, const std::string& pkg, const std::string& key, double value, bool isAdd = false);
        int setData(DcType type, const std::string& pkg, const std::string& key, const std::string& value);
        int addData(DcType type, const std::string& pkg, const std::string& key, int32_t value = 1);
        int incData(DcType type, const std::string& pkg, const std::string& key) { return addData(type, pkg, key, 1); };
        int decData(DcType type, const std::string& pkg, const std::string& key) { return addData(type, pkg, key, -1); };
        void finishOne(DcType type, const std::string& pkg);

        std::string asJson();
        Json::Value getData();
        void reset();

    private:
        Json::Value mData;
        mutable android::Mutex mLock;
        std::string getOldestPackage(const std::string& prefix = "s_", int *pkgCnt = NULL);

        Json::Value *getRequiredKey(DcType type, const std::string& pkg, const std::string &key);
        void startOneIfNeeded( const std::string& pkg);

};


#endif

