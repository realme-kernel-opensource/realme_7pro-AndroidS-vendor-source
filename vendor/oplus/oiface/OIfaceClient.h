#ifndef __OIFACE_CLIENT_H__
#define __OIFACE_CLIENT_H__

#include <string>
#include <utils/Mutex.h>
#include <json/value.h>
#include <utils/RefBase.h>
#include <binder/IBinder.h>

#include "FrameRescuer.h"
#include "OIface.h"
#include "util/KeyThreadTracer.h"
#include "util/FrameStatsTracer.h"

typedef struct {
    int32_t temperature;
    int32_t temperatureLevel;
    int32_t gpu_freq;
    int32_t c_freq[4];
}TemperatureLevelData;

enum {
    /* FIXME!!! */
    FB_TYPE_NONE = 0,
    FB_TYPE_FRAME_BOOST = 1,
    FB_TYPE_SMART_FREQ_WITHOUT_TIMER = 2, /*set low level freq when 2 buffer were produced; set normal level when the only buffer consumed */
    FB_TYPE_SMART_FREQ_WITH_TIMER = 4, /*set low level freq when 2 buffer were produced; set normal level when the only buffer consumed and timeout */
    FB_TYPE_LOW_LATENCY = 8,
};

class SFDeathRecipient: public android::IBinder::DeathRecipient {
    public:
        virtual void binderDied(const android::wp<android::IBinder>& who);
};

/* describe an OIface client. Socket type should have fd, whereas binder client may not  */
class OIfaceClient: public virtual android::RefBase {
    private:
        int mType;
        int mFd;
        int mUid;
        int mPid;
        std::string mClientName;
        std::string mPackageName;
        std::map<int, int> mTaskLoad;

        /* Every client have a state associate with it */
        Json::Value mState;
        Json::Value mDecisionState;

        int64_t mTriggerTime;
        int32_t mFrameCount;
        int32_t mBatteryRemain; /* remain battery in mAh*/
        /* temperature */
        std::vector<float> mTemperature;
        bool    mInvalidated;
        bool    mTriggerStarted;
        int     mLmhCnt0;
        int     mLmhCnt1;
        int64_t mLoadingStart;
        int32_t mAppSceneId;
        int64_t mAppEventStartNanoSec;
        int32_t mEnableTGPANotify;
        std::map<int, float> mCpuTime;

        void handleTriggerStart();
        void handleTriggerStop();
        void writeAppEvent2Db();
#ifdef PREQ
        void checkAppEventTime();
#endif
        struct FrameStats mFrameStats;

        std::string mLayerName;
        int     mUseFrameStats;
        int mSwitchToBackground;

        Decision mFbAction;
        bool mFrameBoosted;
        std::map<int, struct boost_task> mBoostTasks;
        static FrameRescuer* mFrameRescuer;
        struct FrameBoostStats {
            int64_t time;
            int count;
            int dcount;
            int lags;
        } mFbStats;

        int getFBStatistics(struct FrameBoostStats& stats);

        static int getSFService();
        static android::Mutex mSFLock;
        static android::sp<android::IBinder> mSFBinder;
        static android::sp<SFDeathRecipient> mSFDeathRect;

        KeyThreadTracer mKeyThread;

#define MAX_CPU_CORES   8

        std::map<int32_t, int64_t> mTimeInState[MAX_CPU_CORES];
        std::map<int32_t, int32_t> mGpuTimeInState;
        int64_t mIdleTime[MAX_CPU_CORES];

        void updateFrameBoostAction(const Decision& decision);
        void triggerFrameBoostStart(const Decision& decision);
        void triggerFrameBoostStop();
        void triggerFrameBoostSmartFreq();
        int setFrameBoostTaskList();

        struct decision smartFreqDecisionMap[SMART_FREQ_LEVELS];

    public:
        enum {
            /* FIXME!!! */
            OIFACE_TYPE_JSON_SERVER = 0,
            OIFACE_TYPE_JSON_CLIENT,
            OIFACE_TYPE_OIM_SERVER,
            OIFACE_TYPE_OIM_CLIENT,
            OIFACE_TYPE_LOG_SERVER,
            OIFACE_TYPE_LOG_CLIENT,
            OIFACE_TYPE_BINDER_CLIENT,
            OIFACE_TYPE_CONNECTION_LESS,
            OIFACE_TYPE_ENGINE_SERVER,
            OIFACE_TYPE_ENGINE_CLIENT,
        };

        enum {
            FRAME_IPC_VERSION_1 = 1,
        };

        /* setter & getter */
        void setFd(int fd) {mFd = fd;}
        int getFd() {return mFd;}
        void setType(int type) {mType = type;}
        int getType() {return mType;}
        void setUid(int uid) {mUid = uid;}
        int getUid() {return mUid;}
        void setPid(int pid) {mPid = pid;}
        int getPid() {return mPid;}
        void setHeavyTaskLoad(int tid, int loadValue) {
            if (tid > 0 && loadValue > 0) {
                std::map<int, int>::iterator itr = mTaskLoad.find(tid);
                if (itr == mTaskLoad.end()) {
                    mTaskLoad[tid] = loadValue;
                } else {
                    mTaskLoad[tid] = itr->second + loadValue;
                }
            }
        }
        void getHeavyTaskLoad(std::map<int, int>& outTaskLoad) {
            outTaskLoad = mTaskLoad;
        }
        void clearHeavyTaskLoad() {
            mTaskLoad.clear();
        }
        void setClientName(const std::string &clientName) {mClientName = clientName;}
        virtual void setTGPANotifyEnable(int32_t enabled) {mEnableTGPANotify = enabled;}
        virtual int32_t getTGPANotifyEnable() {return mEnableTGPANotify;}
        std::string getClientName() {return mClientName;}
        std::string getPackageName() {return mPackageName;}
        std::string getLayerName() { return mLayerName; }
        void addKeyThreadTid(int tid) { mKeyThread.addKeyThreadTid(tid); }
        string dumpKeyThreads() { return mKeyThread.dumpKeyThreads(); }
        static std::string getTypeAsString(int type);

        /* only valid for socket client */
        static int getUid(int fd);
        static int getPid(int fd);

        /* take careful when updating members */
        Json::Value getState(const std::string& key);
        Json::Value getState();
        void setState(const std::string& key, const Json::Value& value);
        void dumpState();

        Json::Value getDecisionState(const std::string& key);
        Json::Value getDecisionState();
        void setDecisionState(const std::string& key, const Json::Value& value);
        void dumpDecisionState();

        int getConfig(const std::string& key, Json::Value *val);
        int getConfigCategory(std::vector<std::string>& catogory);

        OIfaceClient(const std::string &clientName, int type, int uid = -1, int pid = -1, int fd = -1);

        virtual ~OIfaceClient();

        void triggerMark(int32_t type);
        void triggerAppEvent(int32_t type, int32_t sceneId);
        int32_t setLayerName();
        int32_t getFrames();
        void invalidateLayer();
        bool isGameStarted() {return mTriggerStarted;}

        // Notice: all frame stats' functions should be called in the same thread
        int startFrameStats();
        void stopFrameStats();
        int getFrameStats();
        int updateFrameStats(const struct FrameStats& frameStats);
        int uploadFrameStats();
        int restoreFrameStats();
        void freshBackgroundStats();

        void setSmartFreqDecision(int level, const struct decision &decision);
        const struct decision & getSmartFreqDecision(int level);

        int32_t mDelayBoostTimes = 0;
        void handleLayerRegistered(int32_t type);
        bool isFrameBooted() {return mFrameBoosted;}
        Decision getFrameBoostAction() {return mFbAction;}
        int triggerFrameBoost(const struct FrameBoost& fb);
        int updateFrameBoostTaskList();

        static void handleSFBinderDied();

protected:
        virtual void onTriggerStart();
        virtual void onTriggerStop();
        virtual void onLoadingStart() {};
        virtual void onLoadingStop() {};
        void setPackageName(const std::string &packageName) {mPackageName = packageName;};
        static int syncFrameStats(const std::string &layer, struct FrameStats *frameStatsIn,
                struct FrameStats *frameStatsOut, bool startStat = false, bool resetStat = false);
};

class ConnectionLessClient : public OIfaceClient{
public:

    ConnectionLessClient(const std::string &clientName);
    virtual ~ConnectionLessClient();

    virtual Json::Value getDecisionState(const std::string& key);
    virtual Json::Value getDecisionState();
    virtual void setDecisionState(const std::string& key, const Json::Value& value);
    void setPackageName(const std::string &packageName) {OIfaceClient::setPackageName(packageName);}
};

#endif
