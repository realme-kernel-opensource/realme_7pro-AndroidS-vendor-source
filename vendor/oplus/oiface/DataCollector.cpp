#include "OIface.h"
#include "DataCollector.h"

using namespace std;
using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE(DataCollector);

DataCollector::DataCollector() {
}
Json::Value *DataCollector::getRequiredKey(DcType type, const std::string& pkg,
        const std::string &key) {
    Json::Value *next = NULL;

    if (type == DC_TYPE_CONNECTION) {
        next = &mData[pkg][key];
    } else { /* create required key */
        if (!mData[pkg]["index"].isInt())
            mData[pkg]["index"] = 0;

        int index = mData[pkg]["index"].asInt();
        if (!mData[pkg]["internal"][index].isObject())
            mData[pkg]["internal"][index] = Json::Value(Json::objectValue);
        next = &mData[pkg]["internal"][index][key];
    }

    return next;
}

string DataCollector::getOldestPackage(const std::string& prefix /* = "s_" */,
        int *pkgCnt /* = NULL*/) {
    string oldestPkg;
    int32_t oldestTime = -1;
    int32_t tmpTime;

    if (pkgCnt != NULL)
        *pkgCnt = 0;

    for (auto &pkg: mData.getMemberNames()) {
        if ((prefix.size() > 0) && (pkg.substr(0, prefix.size()) != prefix)) {
            continue;
        }

        if (mData[pkg]["last_time"].isInt()) {
            tmpTime = mData[pkg]["last_time"].asInt();
            if ((oldestTime > tmpTime) || (oldestTime == -1)) {
                oldestTime = tmpTime;
                oldestPkg = pkg;
            }
        }

        if (pkgCnt != NULL)
            (*pkgCnt)++;
    }

    return oldestPkg;
}

void DataCollector::startOneIfNeeded(const std::string& pkg) {
    string oldestPkg;
    int pkgCnt = 0;

    oldestPkg = getOldestPackage("s_", &pkgCnt);
    if ((oldestPkg.size() > 0) && (pkgCnt > MAX_TRACKED_NORMAL_APK)) {
        mData.removeMember(oldestPkg);
        DEBUG("removed %s", oldestPkg.c_str());
    }
    /* create index value if not exist */
    if (!mData[pkg]["index"].isInt())
        mData[pkg]["index"] = 0;

    Json::Value *next = getRequiredKey(DC_TYPE_INTERNAL, pkg, "done");
    if (next->isInt() && (next->asInt() == 1)) {
        int index = mData[pkg]["index"].asInt() + 1;
        if (index >= MAX_DC_INTERNAL_NUM)
            index = 0;

        mData[pkg]["index"] = index;

        /* clear object data */
        mData[pkg]["internal"][index] = Json::Value(Json::objectValue);
    }
}

void DataCollector::finishOne(DcType type, const std::string& pkg) {
    Mutex::Autolock _l(mLock);
    Json::Value *next = getRequiredKey(type, pkg, "done");
    *next = 1;
    mData[pkg]["last_time"] = (int)(systemTime() / 1000000LL);
}

int DataCollector::setData(DcType type, const std::string& pkg, const std::string& key, int32_t value,
        bool isAdd /* = false */) {
    Mutex::Autolock _l(mLock);

    startOneIfNeeded(pkg);

    Json::Value *next = getRequiredKey(type, pkg, key);
    if (isAdd && (!next->isInt())) {
        *next = 0;
    }

    *next = isAdd ? next->asInt() + value : value;

    return 0;
}

int DataCollector::setData(DcType type, const std::string& pkg, const std::string& key, double value,
        bool isAdd /* = false */) {
    Mutex::Autolock _l(mLock);

    startOneIfNeeded(pkg);

    Json::Value *next = getRequiredKey(type, pkg, key);
    if (isAdd && (!next->isDouble())) {
        *next = 0.0f;
    }

    *next = isAdd ? next->asDouble() + value : value;

    return 0;
}

int DataCollector::setData(DcType type, const std::string& pkg, const std::string& key,
        const std::string& value) {
    Mutex::Autolock _l(mLock);

    startOneIfNeeded(pkg);

    Json::Value *next = getRequiredKey(type, pkg, key);

    if (!next->isString()) {
        *next = "";
    }

    *next = value;

    return 0;
}

int DataCollector::addData(DcType type, const std::string& pkg, const std::string& key,
        int32_t value /* = 1*/) {
    return setData(type, pkg, key, value, true);
}

string DataCollector::asJson() {
    Mutex::Autolock _l(mLock);
    return mData.toStyledString();
}

Json::Value DataCollector::getData() {
    Mutex::Autolock _l(mLock);
    return mData;
}

void DataCollector::reset() {
    Mutex::Autolock _l(mLock);
    for (auto &iter: mData.getMemberNames()) { /* package level */
        if (mData[iter]["internal"].isArray()) {
            for (auto& obj: mData[iter]["internal"]) {
                if (!obj["done"].isInt())
                    continue;

                if (obj["done"].asInt() != 1)
                    continue;

                obj = Json::Value(Json::nullValue);
            }
        }
    }
}
