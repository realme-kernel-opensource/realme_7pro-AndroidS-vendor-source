#ifndef __BINDER_MANAGER_H__
#define __BINDER_MANAGER_H__

#include <map>
#include <string>

#include <binder/IBinder.h>
#include "BinderClient.h"
#include "GeneralClient.h"

#define CONNECT_STATUS_KEY    "80000"
#define CLIENT_CONNECTED      "1"
#define CLIENT_DIED           "0"

/* BinderHandler is sent via sendMessage */
class BinderHandler: public android::MessageHandler {
    public:
        BinderHandler(const BinderMessage& msg);
        virtual void  handleMessage(const android::Message& message);
        virtual ~BinderHandler();
    private:
        BinderMessage mMsg;
};

class BinderDeathRecipient: public android::IBinder::DeathRecipient {
    public:
        BinderDeathRecipient(int uid):mUid(uid) {}
        virtual void binderDied(const android::wp<android::IBinder>& who);
        virtual ~BinderDeathRecipient(){}
    private:
        /* to track which binder client died */
        int mUid;
};


class BinderManager {
    public:
        BinderManager();
        ~BinderManager() {};
        void handleMessage(const BinderMessage& msg);
        void notifyCallback(const std::string& str);
        void updateConnectionLessClient(const std::string& package, int value);
        android::sp<OIfaceClient> asOIfaceClient(const char *clientName);
        android::sp<OIfaceClient> asOIfaceClientWithLayerName(const std::string& layerName);
        android::sp<OIfaceClient> getConnectionLessClient() { return mConnectionLessClient; };
        std::string getClientName(const std::string& packageName);
        int getClientList(std::vector<android::sp<OIfaceClient>> *list);
        bool hasAnyClientStartedFrameBoost();
        bool isClientRunning(const pid_t pid);
        void setHeavytask(const pid_t pid, const int tid, const int normalizedTime);
        void clearHeavyTask(pid_t pid);
        void getHeavyTask(pid_t pid, map<int, int>& outTaskLoad);

    private:

        android::sp<ConnectionLessClient> mConnectionLessClient;
        /* hold reference to all binder clients */
        std::map<int, android::sp<BinderClient>> mBinderClients;

        /* hold reference to all death recipients */
        std::map<int, android::sp<BinderDeathRecipient>> mDeathRecipients;

        /* connectless oiface client syntax tree */
        std::map<std::string, std::vector<nodeType*>> mSyntaxTreeList;
        std::map<std::string, std::map<std::string, int>> mGlobalVariable;

        /* connectionless client state list */
        std::map<std::string, int> mStateList;

        int mCurrentUid = 0;

        void handleForground(const std::string& pkgName);
        void handleBackground(const std::string& pkgName);

        /*handle cpuset config when foreground and background is chaged*/
        void handleCpuSet(const std::string& pkgName, int foreground);
        int handleTemperatureNotify(TemperatureLevelData tempLevelData);

        void collectionStart(const std::string& loggerName, int clusterCount, int cpuCoreCount,
                std::map<int32_t, int64_t> *cpuTimeInState, std::map<int32_t, int32_t> &gpuTimeInState,
                int64_t *idleTime);

        void collectionStop(const std::string& loggerName, int clusterCount, int cpuCoreCount,
                std::map<int32_t, int64_t> *cpuTimeInState, std::map<int32_t, int32_t> &gpuTimeInState,
                int64_t *idleTime);

        int createNewClient(const BinderMessage& msg);

        /* TODO: an index not used */
        std::map<int32_t, int64_t> mTimeInState[CLUSTER_NUM];
        int64_t mIdleTime[MAX_CPU_CORES];
        std::map<int32_t, int32_t> mGpuTimeInState;

        /* TODO: an index not used */
        std::map<int32_t, int64_t> mScreenOffTimeInState[CLUSTER_NUM];
        int64_t mScreeOffIdleTime[MAX_CPU_CORES];
        std::map<int32_t, int32_t> mScreenOffGpuTimeInState;

        std::string mForgroundCpus;

        int32_t mLastBatteryRemain;
        int64_t mLastStartTime;
};

#endif
