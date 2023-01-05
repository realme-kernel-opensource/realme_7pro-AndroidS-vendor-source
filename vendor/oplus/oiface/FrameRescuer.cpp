#include <signal.h>
#include <time.h>
#include <sys/prctl.h>
#include <unistd.h>

#include "DecisionDriver.h"
#include "OIfaceModule.h"
#include "OIfaceServer.h"
#include "ThreadState.h"

#include "FrameRescuer.h"
#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>
#include "OifaceCallbackManager.h"
#include "SurfaceFlingerProxyManager.h"
#include "util/FrameStatsTracer.h"
#include "util/DataReader.h"

ANDROID_SINGLETON_STATIC_INSTANCE(FrameRescuer);
FrameRescuer::FrameRescuer() {
    mUid = -1;
    mPid = -1;
    mTimerId = 0;
    mBoostingTime = 0;
    mBoostTotalTime = 0;
    mDirectlyBoost = 0;
    mTotalBoost = 0;
    mLags = 0;
    mBoostReady = false;
    mBoosting = false;
    mFrameBoostType = 0;
    mClientName = "none";
    mFrameRescueStarted = 0;
    mTargetFps = -1;
    mFrameAlp = FRAME_ALPHA;
    mFrameStateMgrObserver = new FRFrameStateMgrObserver(FrameStateManager::getInstance());
}

FrameRescuer::~FrameRescuer() {
    deleteTimer();
}

void FrameRescuer::deleteTimer() {
    if (mTimerId) {
        timer_delete(mTimerId);
        mTimerId = 0;
    }
}

int FrameRescuer::setTimerSlack() {
    int timerslack = prctl(PR_GET_TIMERSLACK);

    if (timerslack < 0) {
        ERROR("FrameRescuer get timerslack error: %s", strerror(errno));
        return -1;
    }
    if (timerslack <= FRAME_RESCUER_TIMER_SLACK_NS) {
        DEBUG("FrameRescuer current timer slack: %d ns", timerslack);
        return 0;
    }
    if (prctl(PR_SET_TIMERSLACK, FRAME_RESCUER_TIMER_SLACK_NS) < 0) {
        ERROR("FrameRescuer set timerslack error: %s", strerror(errno));
        return -1;
    }
    DEBUG("FrameRescuer set timerslack: %d ns", FRAME_RESCUER_TIMER_SLACK_NS);

    return 0;
}

void FrameRescuer::addFrameInterval(int64_t timestamp) {
    struct frame_info frame;
    struct frame_info lastFrame = mFrameTimeList.back();
    int interval = (timestamp - lastFrame.time)/1000000;
    ATRACE_INT("frameIntervalMs", interval);

    frame.time = timestamp;
    frame.delay = systemTime(SYSTEM_TIME_MONOTONIC) - timestamp;
    mFrameTimeList.push_back(frame);
    if (mFrameTimeList.size() > MAX_SAMPLE_FRAMES) {
        mFrameTimeList.pop_front();
    }
}

int FrameRescuer::getDiretlyBoostCount() {
    return mDirectlyBoost.load();
}

int FrameRescuer::getBoostCount() {
    return mTotalBoost.load();
}

int64_t FrameRescuer::getBoostTime() {
    return mBoostTotalTime.load();
}

int FrameRescuer::getLags() {
    return mLags.load();
}

int FrameRescuer::redundantFrameBoostTime(int elapsed, int mdelay, int cdelay) {
    //1. binder thread wakeup costs variable/delay time, and it is different every frame,
    //2. it's necessary to add a redundant time to avoid frequently accidently boost system
    //performance with hypnus.
    //At first, calc delay with weight(50% for the latest frame, 50% for the mean value),
    //int delay = cdelay / 2 + mdelay / 2;
    //but accutally, the binder wakeup time is more random regard as the statistics data...
    //so here use the mean value.
    int delay = mdelay;
    return delay - elapsed + FRAME_RESCUER_REDUNDANT_TIME;
}

int FrameRescuer::caclTimerTimeInterval() {
    static int lastFI = 0;
    int fi, tti;
    int64_t mean, mdelay, cdelay;
    int64_t biggest = 0, bigger = 0, sum = 0, delay = 0;
    int64_t last = mFrameTimeList.front().time;

    for (auto& frame : mFrameTimeList) {
        int64_t intval = frame.time - last;
        if (intval < 0) {
            ERROR("FrameRescuer timestamp list order(%ld > %ld) was wrong!", last, frame.time);
            continue;
        }
        last = frame.time;
        sum += intval;
        if (intval > biggest) {
            bigger = biggest;
            biggest = intval;
        } else if (intval > bigger) {
            bigger = intval;
        }
        delay += frame.delay;
    }

    if (mFrameTimeList.size() > 3) {
        //calculate mean value without the two biggest one that may be result from laggy
        sum = sum - biggest - bigger;
        mean = sum / (mFrameTimeList.size() - 3);
    } else if (mFrameTimeList.size() > 1) {
        mean = sum / (mFrameTimeList.size() - 1);
    } else {
        mean = 0;
    }
    mdelay = delay / mFrameTimeList.size();
    fi = calcFrameInterval(mean);
    cdelay = mFrameTimeList.back().delay;
    if (fi != lastFI) {
        INFO("FrameRescuer frame interval changed %d -> %d.(m %ld, md %ld, cd %ld)",
            lastFI, fi, mean, mdelay, cdelay);
        lastFI = fi;
    }

    tti = fi + redundantFrameBoostTime(cdelay, mdelay, cdelay);

    return tti;
}

int FrameRescuer::calcFrameInterval(int64_t mean) {
    int rate = FR_FPS_HIGH;
    int interval = ONE_SEC_NS / rate;
    int range = (ONE_SEC_NS / FR_FPS_LOWEST + interval / 2) / interval;
    int64_t fi;

    for (int i = 0; i <= range; i++) {
        fi = interval * i;
        if (mean <= (fi * 104 / 100))
            break;
    }
    if (fi == 0) {
        fi = ONE_SEC_NS / FR_FPS_LOWEST;
    }

    return fi;
}

void FrameRescuer::recordFI(int now) {
    if (lastRecordTime < now) {
        DataReader::getInstance().mFrameIntervalCollector.addToVec(now - lastRecordTime);
    }
    lastRecordTime = now;
}

int FrameRescuer::produceFrame(int bcNumber, int64_t timestamp) {
    if (bcNumber <= 0 || timestamp >= systemTime(SYSTEM_TIME_MONOTONIC))
        return -1;

    ATRACE_INT("produceFrame", bcNumber);
    stopBoost();
    setupTimer(bcNumber);
    if (recordStatu) {
        recordFI(timestamp);
    }
    addFrameTime(timestamp);
    addFrameInterval(timestamp);
    ATRACE_INT("produceFrame", 0);

    if (!mFrameRescueStarted && !mTraceTargetFPS) {
        BinderMessage msg;
        msg.what = BINDER_MESSAGE_STOP_LIGHTNING_START_PACKAGE;
        msg.uid = 0;
        msg.pid = 0;
        msg.json = "frameBooster";
        OIfaceServer::getInstance().sendMessage(msg);
        return 2;
    }
    return 0;
}

int FrameRescuer::setupTimer(int bcNum) {
    struct sigevent se;
    struct itimerspec its;
    int tti = 0;
    ATRACE_INT("targetFps", mTargetFps);

    if (mTargetFps == -1) {
        tti = caclTimerTimeInterval();
    } else {
        tti = ONE_SEC_NS * mFrameAlp / mTargetFps ;
    }

    if (tti <= TTI_MIN_TEN_MS || tti >= ONE_SEC_NS) {
        ERROR("FrameRescuer: tti(%d) is abnormal", tti);
        return -1;
    }

    //if bcnum is 2, timer can be longer to avoid unnecessary boost.
    if (bcNum > 1)
        tti = tti * 1.7;

    if (mTimerId == 0) {
        mTimerParam.uid = mUid;
        mTimerParam.pid = mPid;
        mTimerParam.fr = this;
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

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = tti;
    mTimerParam.tti = tti;
    if (timer_settime(mTimerId, 0, &its, NULL) < 0) {
        ERROR("set timer failed.(%s)", strerror(errno));
        return -1;
    }

    return 0;
}

int FrameRescuer::getTargetFps(){
    return mTargetFps;
}

void FrameRescuer::setTargetFps(int targetFps) {
    mTargetFps = targetFps;
}

float FrameRescuer::getFrameAlp(){
    return mFrameAlp;
}

void FrameRescuer::setFrameAlp(float frameAlp) {
    if(frameAlp < FRAME_ALPHA_MIN){
        mFrameAlp = FRAME_ALPHA;
    }else{
        mFrameAlp = frameAlp;
    }
}

void FrameRescuer::cancelTimer() {
    if (mTimerId) {
        struct itimerspec its;
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = 0;
        its.it_value.tv_sec = 3600;
        its.it_value.tv_nsec = 0;
        if (timer_settime(mTimerId, 0, &its, NULL) < 0) {
            ERROR("set timer failed.(%s)", strerror(errno));
            return;
        }
        //DEBUG("reset timer to one hour");
    }
}

void FrameRescuer::timerCallBack(union sigval val) {
    struct timer_params *tp = static_cast<struct timer_params*>(val.sival_ptr);
    DEBUG("%dms no frame produced boost boost boost", tp->tti/1000000);
    tp->fr->boost();
    // ATRACE_INT("TTI", tp->tti/1000000);
    // print statistics
    static int lcount = 0;
    static int64_t lstime = 0;
    static int64_t ltime = systemTime(SYSTEM_TIME_MONOTONIC);
    int64_t curtime = systemTime(SYSTEM_TIME_MONOTONIC);
    if (curtime >= ltime + ONE_SEC_NS) {
        int count = tp->fr->getBoostCount();
        int64_t time = tp->fr->getBoostTime();
        INFO("FrameRescuer stats: "
                "total %ld ms, count %d (D%d), %d/%ld => %.2lf bps, %ld mspb",
                time / ONE_MSEC_NS,
                count,
                tp->fr->getDiretlyBoostCount(),
                count - lcount,
                (curtime - ltime) / ONE_MSEC_NS,
                (count - lcount) / ((double)(curtime - ltime) / ONE_SEC_NS),
                (time - lstime) / ONE_MSEC_NS / (count - lcount));
        // update
        lcount = count;
        lstime = time;
        ltime = curtime;
    }
}

#define FRAME_BOOST_DEF_ACTION  12
#define FRAME_BOOST_DEF_BOOST   100
int FrameRescuer::setBoostAction(const Decision& decision) {
    // 1. convert to local data struct
    // 2. format to be used exclusivly for fb
    mBoostReady = false;
    mDecision = decision;

    if (mFrameBoostType & (FB_TYPE_SMART_FREQ_WITH_TIMER | FB_TYPE_SMART_FREQ_WITHOUT_TIMER)) {
        // When smart freq is on, fb action is hypnus action 1,12,200
        mDecision.type = DECISION_TYPE_ACTION;
        mDecision.action.action_type = FRAME_BOOST_DEF_ACTION;
        mDecision.action.timeout = FRAME_RESCUER_HYPNUS_ACTION_TIMEOUT_MS;
        mDecisionStop = mDecision;
        mDecisionStop.action.timeout = 0;
    } else if (mDecision.type == DECISION_TYPE_ACTION) {
        struct hypnus_action& action = mDecision.action;
        if (action.action_type <= 0) {
            action.action_type = FRAME_BOOST_DEF_ACTION;
        }
        if (action.timeout <= 0
                || action.timeout > FRAME_RESCUER_HYPNUS_ACTION_TIMEOUT_MS) {
            action.timeout = FRAME_RESCUER_HYPNUS_ACTION_TIMEOUT_MS;
        }
        // set mDecisionStop
        mDecisionStop = mDecision;
        mDecisionStop.action.timeout = 0;
    } else if (mDecision.type == DECISION_TYPE_SCHEDTUNE) {
        struct schedtune_boost_action& action = mDecision.schedtune_boost;
        if (action.index < 0 || action.index >= SCHEDTUNE_BG_MAX_INDEX) {
            action.index = SCHEDTUNE_BG_INDEX_TOP_APP;
        }
        if (action.boost < 0 || action.boost > FRAME_BOOST_DEF_BOOST) {
            action.boost = FRAME_BOOST_DEF_BOOST;
        }
        action.defered = -1; // instantly boost
        action.clear_tasks = 0;
        action.add_tasks = 0,
        action.count = 0;
        action.tasks = 0;
        // set mDecisionStop
        mDecisionStop = mDecision;
        mDecisionStop.schedtune_boost.defered = SCHEDTUNE_DEFERED_SKIP_SETTING;
        mDecisionStop.schedtune_boost.boost = 0;
    } else {
        return -1;
    }

    // set boost flag
    mBoostReady = true;

    return 0;
}

int FrameRescuer::updateBoostDecision(const string& clientName,
                                    const Decision& decision) {
    return 0;
}

void FrameRescuer::defaultFrameBoost() {
    // Default frame boost, using orms action
    // ERROR("defaultFrameBoost");
    /*
    mBoostAction.scene = "oiface";
    mBoostAction.action_type = "ORMS_ACTION_OIFACE_GAME_BOOST_L3";
    mBoostAction.timeout = 100;
    mBoostAction.resultCode = 0;

    OrmsProxy::getInstance().setAction(mBoostAction);
    */
}

void FrameRescuer::stopDefaultFrameBoost() {
    // Default frame boost, using orms action
    /*
    ERROR("stopDefaultFrameBoost");
    mBoostAction.scene = "oiface";
    mBoostAction.action_type = "ORMS_ACTION_OIFACE_GAME_BOOST_L3";
    mBoostAction.timeout = 0;
    mBoostAction.resultCode = -1;

    OrmsProxy::getInstance().setAction(mBoostAction);
    */
}

void FrameRescuer::boost() {
    ATRACE_INT("FrameBoost", 1);
    OifaceCallbackManager::getInstance().reportFB(1);

    if (mDecision.type == DECISION_TYPE_ACTION)
        actionBoost();
    else if (mDecision.type == DECISION_TYPE_SCHEDTUNE)
        schedtuneBoost();
    else
        defaultFrameBoost();

    int64_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    if (mBoosting && mBoostingTime > 0) {
        doStatistics(now);
    }
    mBoostingTime = now;
    mBoosting = true;
    mTotalBoost++;
}

void FrameRescuer::stopBoost() {
    ATRACE_INT("FrameBoost", 0);
    if (!mBoosting)
        return;

    OifaceCallbackManager::getInstance().reportFB(0);

    if (mDecision.type == DECISION_TYPE_ACTION) {
        stopBoostAction();
    } else if (mDecision.type == DECISION_TYPE_SCHEDTUNE) {
        stopSchedtuneBoost();
    } else {
        stopDefaultFrameBoost();
    }

    if (mBoostingTime > 0) {
        doStatistics(systemTime(SYSTEM_TIME_MONOTONIC));
        mBoostingTime = 0;
    }

    mBoosting = false;
}

void FrameRescuer::doStatistics(int64_t now) {
    // statistics boosting time
    int64_t delta = now - mBoostingTime;
    if (delta > 100 * ONE_MSEC_NS) {
        mLags++;
    }
    if (mDecision.type == DECISION_TYPE_ACTION
            && delta > mDecision.action.timeout * ONE_MSEC_NS) {
        delta = mDecision.action.timeout * ONE_MSEC_NS;
    }
    mBoostTotalTime += delta;
}

static inline bool isSchedtuneBoost(const Decision& decision) {
    return decision.type == DECISION_TYPE_SCHEDTUNE;
}

void FrameRescuer::stopAndClean() {
    setTargetFps(-1);
    stopBoost();
    cleanup();
}

void FrameRescuer::cleanup() {
    int clean = 0;
    string clientName;
    Decision decision = { .type = 0, .data = { 0 } };
    deleteTimer();

    mLock.lock();
    mBoostReady = false;
    clientName = mClientName;
    if (isSchedtuneBoost(mDecisionStop)) {
        decision = mDecisionStop;
        clean = 1;
    }
    mLock.unlock();

    if (!clean)
        return;

    if (isSchedtuneBoost(decision)) {
        decision.schedtune_boost.defered = 0;
        decision.schedtune_boost.boost = 0;
        decision.schedtune_boost.clear_tasks = 1;
        decision.schedtune_boost.add_tasks = 0;
    }
    DecisionDriver::getInstance().talkToDriver(decision, clientName, true);

    DEBUG("FrameRescuer: clean done");
}

void FrameRescuer::actionBoost() {
    // DecisionDriver::getInstance().talkToDriver(mDecision, mClientName, false);
    ERROR("FrameRescuer::actionBoost: Should never come here");
}

void FrameRescuer::stopBoostAction() {
    // DecisionDriver::getInstance().talkToDriver(mDecisionStop, mClientName, false);
    ERROR("FrameRescuer::stopBoostAction: Should never come here");
}

void FrameRescuer::schedtuneBoost() {
    DecisionDriver::getInstance().talkToDriver(mDecision, mClientName, false);
}

void FrameRescuer::stopSchedtuneBoost() {
    DecisionDriver::getInstance().talkToDriver(mDecisionStop, mClientName, false);
}

int FrameRescuer::triggerFrameBoost(std::string package, const struct FrameBoost& fb) {
    switch (fb.type) {
        case FRAME_BOOST_START:
            setFrameBoostType(FB_TYPE_FRAME_BOOST);
            triggerFrameBoostStart(package, fb.decision);
            break;
        case FRAME_BOOST_STOP:
            setFrameBoostType(FB_TYPE_NONE);
            triggerFrameBoostStop();
            if (mTraceTargetFPS) {
                stopFPSTrace();
                setTraceTargetFPS(false);
            }
            break;
        case FRAME_BOOST_WITH_FPS_TRACK:
            DEBUG("Enable FPS trace and frame boost");
            setFrameBoostType(FB_TYPE_FRAME_BOOST);
            setTraceTargetFPS(true);
            startFPSTrace();
            triggerFrameBoostStart(package, fb.decision);
            break;
        default:
            ERROR("Get unknow frame boost type %d!", fb.type);
            return -1;
    }
    return 0;
}

void FrameRescuer::triggerFrameBoostStart(std::string package, const Decision& decision) {
    DEBUG("Start frame boost  Alp:%f",mFrameAlp);
    if (mFrameRescueStarted || recordStatu) {
        ERROR("Frame Rescue has started for package: %s", mClientName.c_str());
        mFrameRescueStarted = true;
        if (package == mClientName) {
            mFrameStateMgrObserver->addFrameStateListener();
            return;
        } else {
            ERROR("stop current frame boost");
            triggerFrameBoostStop();
        }
    }

    vector<string> empty;
    std::string layer;
    int getLayerTry = 3;

    for (int i = 0; i < getLayerTry; i++) {
        layer = SurfaceFlingerProxyManager::getInstance().getLayerName(package, empty);
        if (layer.empty()) {
            usleep(100000);
        } else {
            DEBUG("Get layer name: %s", layer.c_str());
            break;
        }
    }

    if (layer.empty()) {
        ERROR("!!!!layer is empty");
        return;
    }

    // notify sf
    if (FrameStateManager::getInstance().startSurfaceFlingerFrameNotification(package, layer)) {
        mFrameRescueStarted = true;
        mClientName = package;
        mFrameStateMgrObserver->addFrameStateListener();
        INFO("triggerFrameBoost mark start layer[%s]", layer.c_str());
    }
}

void FrameRescuer::triggerFrameBoostStart(std::string package) {
    DEBUG("Start frame info collection");
    // if (mFrameRescueStarted) {
    //     ERROR("Frame Rescue has started for package: %s", mClientName.c_str());
    //     recordStatu = true;
    //     if (package != mClientName) {
    //         ERROR("stop current frame boost");
    //         triggerFrameBoostStop();
    //     }
    //     return;
    // }
    recordStatu = true;

    vector<string> empty;
    std::string layer;
    int getLayerTry = 5;

    for (int i = 0; i < getLayerTry; i++) {
        layer = SurfaceFlingerProxyManager::getInstance().getLayerName(package, empty);
        if (layer.empty()) {
            usleep(200000);
        } else {
            DEBUG("Get layer name: %s", layer.c_str());
            break;
        }
    }

    // notify sf
    if (SurfaceFlingerProxyManager::getInstance().startSurfaceFlingerFrameNotification(layer.c_str())) {
        mClientName = package;
    }
}

void FrameRescuer::triggerFrameBoostStop() {
    DEBUG("Stop frame boost");
    if (!mFrameRescueStarted) {
        INFO("Frame boost has stoped.");
        return;
    }

    mFrameStateMgrObserver->removeFrameStateListener();
    if(!recordStatu){
        mClientName = "none";
        FrameStateManager::getInstance().stopSurfaceFlingerFrameNotification("frameBooster");
    }
    mFrameRescueStarted = false;
    stopAndClean();
    INFO("triggerFrameBoost mark stop");
}

void FrameRescuer::triggerFrameInfoStop() {
    DEBUG("Stop frame info collection");
    recordStatu = false;
    if (mFrameRescueStarted) {

        return;
    }

    if(!mFrameRescueStarted && recordStatu){
        mClientName = "none";
        SurfaceFlingerProxyManager::getInstance().stopSurfaceFlingerFrameNotification();
        mFrameRescueStarted = false;
        stopAndClean();
        INFO("triggerFrameInfo collection mark stop");
    }
}
