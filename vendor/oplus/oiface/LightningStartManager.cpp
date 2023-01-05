#include <signal.h>
#include <time.h>
#include <sys/prctl.h>
#include <unistd.h>
#include "OIfaceModule.h"
#include "OIfaceServer.h"
#include "ThreadState.h"
#include "Utils.h"

#include "LightningStartManager.h"
#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>
#include <utils/Mutex.h>
#include "OifaceCallbackManager.h"
#include "SurfaceFlingerProxyManager.h"
#include "util/FrameStatsTracer.h"
#include "PerformanceHalService.h"

ANDROID_SINGLETON_STATIC_INSTANCE(LightningStartManager);
LightningStartManager::LightningStartManager() {
    mUid = -1;
    mPid = -1;
    mTimerId = 0;
    mLastBufNum = 0;
    mLightningStartPackage = "none";
    mFrameRescueStarted = 0;
    mFrameProduceNum = 0;
    mFrameStateMgrObserver = new LSFrameStateMgrObserver(FrameStateManager::getInstance());
    mSwapVersion = PerformanceHalService::getInstance().getNandswapVersion();
}

LightningStartManager::~LightningStartManager() {
    deleteTimer();
}

void LightningStartManager::deleteTimer() {
    if (mTimerId) {
        if (timer_delete(mTimerId) < 0) {
            ERROR("timer_delete failed.(%s)", strerror(errno));
            return;
        }
        mTimerId = 0;
    }
}

int LightningStartManager::produceFrame(int bcNumber, int64_t timestamp) {
    mFrameProduceNum ++;
    DEBUG("LightningStart produceFrame:mFrameProduceNum  %d", mFrameProduceNum.load());
    if (mFrameProduceNum < LIGHTNING_START_FRAME_THRESHOLD) {
        return 0;
    }

    DEBUG("LightningStart produceFrame:mFrameProduceNum  %d", mFrameProduceNum.load());
    OifaceCallbackManager::getInstance().reportStartFrameProduce(curLightningStartStateStr(LS_FRAME_PRODUCED, mLightningStartPackage));

    BinderMessage msg;
    msg.what = BINDER_MESSAGE_STOP_LIGHTNING_START_PACKAGE;
    msg.uid = 0;
    msg.pid = 0;
    msg.json = "lightningStart";
    OIfaceServer::getInstance().sendMessage(msg);
    mFrameProduceNum = 0;
    return 2;
}

void LightningStartManager::setLightningStartPackage(std::string packageName) {
    DEBUG("setLightningStartPackage enter packageName: %s\n", packageName.c_str());

    if(packageName.empty()) {
        OifaceCallbackManager::getInstance().reportStartFrameProduce(curLightningStartStateStr(LS_FRAME_PRODUCED_FAILED, packageName));
        cancelGetLayerNameTimer();
        return;
    }
    mLightningStartPackage = packageName;
    setupGetLayerNameTimer();
}

int LightningStartManager::getLayerRetryCount() {
    return mGetLayerRetry.load();
}

string LightningStartManager::getLightningStartPackage() {
    return mLightningStartPackage;
}

void LightningStartManager::decGetLayerRetry() {
    mGetLayerRetry--;
}

void LightningStartManager:: cancelGetLayerNameTimer() {
    mGetLayerRetry = 0;
    if (mTimerId) {
        struct itimerspec its;
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = 0;
        its.it_value.tv_sec = 0;
        its.it_value.tv_nsec = 0;
        if (timer_settime(mTimerId, 0, &its, NULL) < 0) {
            ERROR("set timer failed.(%s)", strerror(errno));
            return;
        }
    }

    DEBUG("cancelGetLayerNameTimer \n");
}

void LightningStartManager::timerCallBack(union sigval val) {
    struct timer_params *tp = static_cast<struct timer_params*>(val.sival_ptr);
    vector<string> empty;
    std::string layer;
    string package;
    package = tp->lightningStMan->getLightningStartPackage();

    layer = SurfaceFlingerProxyManager::getInstance().getLayerName(tp->lightningStMan->getLightningStartPackage(), empty);

    if(!layer.empty()) {
        int frameNum = SurfaceFlingerProxyManager::getInstance().getTotalFpsByLayerName(layer);
        DEBUG("LightningStartManager frameNum: %d \n", frameNum);
        if (frameNum > 0) {
            OifaceCallbackManager::getInstance().reportStartFrameProduce(curLightningStartStateStr(LS_FRAME_PRODUCED, package));
            tp->lightningStMan->cancelGetLayerNameTimer();
            return;
       }
    }
    DEBUG("LightningStartManager package : %s , getLayerName:  mGetLayerRetry: %d \n", package.c_str(), tp->lightningStMan->getLayerRetryCount());
    if(layer.find(tp->lightningStMan->getLightningStartPackage()) == string::npos) {
        DEBUG("getLayerName: %s, package : %s\n", layer.c_str(), tp->lightningStMan->getLightningStartPackage().c_str());
    } else if (!layer.empty() && ((layer.find("Splash") == string::npos) || LightningStartManager::getInstance().getLayerRetryCount() < 5)) {
        if (FrameStateManager::getInstance().startSurfaceFlingerFrameNotification(package, layer)) {
            tp->frameStateMgrObserver->addFrameStateListener();
            tp->lightningStMan->cancelGetLayerNameTimer();
            DEBUG("set Lightning Start mark start layer[%s]", layer.c_str());
            return;
        }
        ERROR("LightningStartManager StartFB:failed \n");
    }

    if(tp->lightningStMan->getLayerRetryCount() == 0) {
        OifaceCallbackManager::getInstance().reportStartFrameProduce(curLightningStartStateStr(LS_FRAME_PRODUCED_FAILED, package));
        tp->lightningStMan->cancelGetLayerNameTimer();
    }
    tp->lightningStMan->decGetLayerRetry();
}

int LightningStartManager:: setupGetLayerNameTimer() {
    DEBUG("setupGetLayerNameTimer enter \n");

    struct sigevent se;
    struct itimerspec its;
    Mutex::Autolock lock(mLightenStrManMutex);

    if (mTimerId == 0) {
        mTimerParam.uid = mUid;
        mTimerParam.pid = mPid;
        mTimerParam.lightningStMan = this;
        mTimerParam.frameStateMgrObserver = mFrameStateMgrObserver;

        se.sigev_notify = SIGEV_THREAD;
        se.sigev_signo = SIGALRM;
        se.sigev_value.sival_ptr = &mTimerParam;
        se.sigev_notify_function = timerCallBack;
        se.sigev_notify_attributes = NULL;

        if (timer_create(SYSTEM_TIME_MONOTONIC, &se, &mTimerId) < 0) {
            ERROR("create timer failed.(%s)", strerror(errno));
            return -1;
        }
    }
    mGetLayerRetry = 10;

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 200000000;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 200000000;

    if (timer_settime(mTimerId, 0, &its, NULL) < 0) {
        ERROR("set timer failed.(%s)", strerror(errno));
        return -1;
    }
    DEBUG("setupGetLayerNameTimer");

    return 0;
}

int LightningStartManager:: setNandSwapOut(int pid, std::string packageName) {
    int res = -1;

    if (mSwapVersion == 1) {
        string swapOutStr = to_string(pid);
        swapOutStr.append(" 1");
        res = PerformanceHalService::getInstance().writeNandswapProc(NAND_SWAP_CTL_PATH, swapOutStr);
        DEBUG("setNandSwapOut : %s, res = %d",swapOutStr.c_str(), res);
    } else if (mSwapVersion == 2) {
        int uid = getUidByPid(pid);
        res = PerformanceHalService::getInstance().writeNandswapProc(uid, pid, 0, 1);
        DEBUG("setNandSwapOut newVersion uid: %d, pid: %d, res = %d", uid, pid, res);
    }

    int curState = res == 0 ? LS_SWAP_OUT: LS_SWAP_OUT_FAILED;
    OifaceCallbackManager::getInstance().reportStartFrameProduce(curLightningStartStateStr(curState, packageName));
    return res;
}

int LightningStartManager:: setNandSwapIn(int pid, std::string packageName) {
    int res = -1;

    if (mSwapVersion == 1) {
        string swapInStr = to_string(pid);
        swapInStr.append(" 0");
        int res = PerformanceHalService::getInstance().writeNandswapProc(NAND_SWAP_CTL_PATH, swapInStr);
        DEBUG("setNandSwapIn : %s , res = %d",swapInStr.c_str(), res);
    } else if (mSwapVersion == 2) {
        int uid = getUidByPid(pid);
        res = PerformanceHalService::getInstance().writeNandswapProc(uid, pid, 0, 0);
        DEBUG("setNandSwapIn newVersion uid: %d, pid: %d, res = %d", uid, pid, res);
    }

    return res;
}

std::string LightningStartManager::curLightningStartStateStr(int curState, std::string packageName) {
    Json::Value LSState;
    LSState["notify"] = std::string("lightning_start");
    LSState["pkg_name"] =  std::string(packageName);
    LSState["cur_state"] = Json::Value(curState);

    Json::FastWriter writer;
    std::string data = writer.write(LSState);
    return data;
}
