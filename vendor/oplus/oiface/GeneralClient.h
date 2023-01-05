#ifndef __GENERAL_CLIENT_H__
#define __GENERAL_CLIENT_H__

#include <utils/Looper.h>
#include <map>
#include <string>

#include <utils/String8.h>
#include "OIfaceService.h"
#include "BinderMessage.h"
#include "BinderClient.h"
#include "comp.h"

class GeneralClient: public BinderClient {
    public:
        GeneralClient(int uid, int pid, struct GeneralConfig config);
        virtual ~GeneralClient();

        virtual int handleEvent(const BinderMessage &msg);
    private:
        enum ACTION_TYPE {
            ACTION_GAME_BURST = 1,
            ACTION_GAME_LOAD,
            ACTION_GAME_CANCEL,
            ACTION_GAME_START,
            ACTION_GAME_END,
            ACTION_GAME_MAIN,
            ACTION_APP_MAIN,
            ACTION_APP_BURST,
            ACTION_APP_LOAD,
            ACTION_APP_CANCEL,
        };

        int mEnableAction = 1;
        struct GeneralConfig mConfig;
        int mActionType;
        int mActionTime;
        int mExtra;
        int64_t mStartTime;
        int64_t mLastLoadTime;
        int64_t mLastBurstTime;
        int64_t mAllBackgroundTime = 0;
        int64_t mLastBackgroundTime = 0;
        //general client Accelerate time
        int64_t mAccStartTime = 0;
        int64_t mAccEndTime = 0;
        int64_t mAccAllTime = 0;
        int mAccTimes = 0;
        int mInvalidTimes = 0;
        //for data collect
        int64_t mLastStartTime;
        int mLastAccTimes = 0;
        int64_t mLastAccAllTime = 0;
        int mLastInvalidTimes = 0;

         //general engine default max time
        int64_t mMaxEngineTime  = 10000;           //10s

        //record engine status to make decesions
        int mShaderCompile = 0;
        int mLoadScene = 0;

        uint64_t getSystemTime();
        //set action
        int setAction();
        int getAccRatio() { return mConfig.acc_ratio; }
        int getType() { return mConfig.general_type; }
        int64_t getBurstInterval() { return mConfig.burst_interval; }
        int64_t getLoadInterval() { return mConfig.load_interval; }
        int getBurstTime() { return mConfig.burst_time; }
        int getLoadTime() { return mConfig.load_time; }
        bool isApp();
        bool isGame();
        bool isEngine();
        bool onlyEngine();
        //check burst interval and max time
        bool checkBurst();
        //check load interval and max time
        bool checkLoad();
        //for dump  this time data collect
        void dumpData();

        void enableAction() { mEnableAction = 1; }
        void disableAction() { mEnableAction = 0; }
        int isActionEnable();

    protected:
        virtual void onTriggerStop();
};

#endif
