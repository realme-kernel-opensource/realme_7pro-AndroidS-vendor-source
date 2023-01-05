#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <utils/Mutex.h>
#include "ThreadState.h"
#include "OIface.h"
#include "TaskManager.h"
#include "Utils.h"
#include "BinderMessage.h"
#include "OIfaceServer.h"
#include "DecisionDriver.h"

ANDROID_SINGLETON_STATIC_INSTANCE( TaskManager );

#define CHECK_WAKE_UP_DELAY_TIME 8000000000LL
#define TASK_MONITOR_PATH "/proc/game_opt/render_thread_info"

Mutex mTmLock;

TaskManager::TaskManager() {
    INFO("initialize TaskManager feature");
    Mutex::Autolock _l(mTmLock);
}

void TaskManager::setHeavyTask(const pid_t tid, const int normalizedLoad) {
    if (tid > 0 && normalizedLoad > 0) {
        std::map<int, int>::iterator itr = mTaskLoad.find(tid);
        if (itr == mTaskLoad.end()) {
            mTaskLoad[tid] = normalizedLoad;
        } else {
            mTaskLoad[tid] = itr->second + normalizedLoad;
        }
    }
}

void TaskManager::getHeavyTask(std::map<int, int>& outTaskLoad) {
    outTaskLoad = mTaskLoad;
}

// get top "pickNum" heavy task
void TaskManager::pickHeavyTask(pid_t gamePid, int pickNum) {
    DEBUG("pick game pid: %d, count: %d.", gamePid, pickNum);
    if (gamePid <= 0 || pickNum <= 0) {
        DEBUG("cant pick heavy task, param is invalid.");
        return;
    }

    if (mTaskLoad.empty()) {
        DEBUG("no task load info!");
        return;
    }

    vector<pair<int, int>> vtMap;
    for (auto iter = mTaskLoad.begin(); iter != mTaskLoad.end(); iter++){
        vtMap.push_back(make_pair(iter->first, iter->second));
        DEBUG("heavy task id: %d load: %d.", iter->first, iter->second);
    }
    sort(vtMap.begin(), vtMap.end(),
        [](const pair<int, int> &item1, const pair<int, int> &item2)-> int {
            return item1.second > item2.second;
        });

    pid_t glThread = getGLThread();
    if (glThread == 0) {
        DEBUG("no GL Thread from SF");
        for (int i = 0; i < vtMap.size() && i < pickNum; i++) {
            keyTaskInfo[vtMap[i].first] = mKeyThreadBindValue;
        }
    } else {
        bool setGLThread = false;
        for (int i = 0; i < vtMap.size() && i < pickNum; i++) {
            if (glThread == vtMap[i].first) {
                setGLThread = true;
            }
            // ensure to set the GL thread
            if (((i == vtMap.size() - 1) || (i == pickNum - 1)) && (setGLThread == false)) {
                keyTaskInfo[glThread] = mKeyThreadBindValue;
                break;
            }
            DEBUG("set pick thread %d as key task.", vtMap[i].first);
            keyTaskInfo[vtMap[i].first] = mKeyThreadBindValue;
        }
    }
}

void TaskManager::setMonitorTask(vector<int> tids) {
    DEBUG("setMonitorTask");
    if (tids.size() == 0) {
        DEBUG("empty arrays");
        return;
    }

    string tidInfo = "";
    for (int i = 0; i < tids.size(); i++) {
        tidInfo += to_string(tids[i]);
        if (i == tids.size() - 1) { break; }
        tidInfo += " ";
    }
    DEBUG("write to monitor path: %s, tids: %s.", TASK_MONITOR_PATH, tidInfo.c_str());
    writeFile(TASK_MONITOR_PATH, tidInfo);
    BinderMessage msg;
    msg.what = BINDER_MESSAGE_PICK_TASK_TIMEOUT;
    OIfaceServer::getInstance().sendMessageDelayed(msg, CHECK_WAKE_UP_DELAY_TIME);
}

void TaskManager::findKeyWorker() {
    Mutex::Autolock _l(mTmLock);
    if (!isMonitorRunning()) {
        return;
    }

    DEBUG("find waker task");
    string taskInfo = getSystemFile(TASK_MONITOR_PATH);
    DEBUG("get wake up info: %s", taskInfo.c_str());
    int max = 1;
    // cache tid -> wakeupCount map
    map<int, int> wakeupCount;
    // vector item -> waker wakee wakeupCount
    vector<string> wakeupInfo;
    splitString(taskInfo, wakeupInfo, "\n");
    for (auto item : wakeupInfo) {
        // DEBUG("wakeupInfo: %s", item.c_str());
        vector<string> info;
        splitString(item, info, " ");
        //for debug
        int foregroundPid = ThreadState::getInstance().getForgroundPid();
        DEBUG("wakeupInfo waker: %s, wakerName: %s, wakee: %s, wakeeName: %s, count: %s", info[0].c_str(),
        getTaskNameByTid(foregroundPid, stoi(info[0])).c_str(), info[1].c_str(), getTaskNameByTid(foregroundPid, stoi(info[1])).c_str(), info[2].c_str());
        vector<pid_t>::iterator itr = find(mMainTask.begin(), mMainTask.end(), stoi(info[0]));
        if (itr == mMainTask.end()) {
            int count = stoi(info[2]);
            // consider more then two threads wake up by the same thread
            if (wakeupCount.find(stoi(info[0])) == wakeupCount.end()) {
                wakeupCount[stoi(info[0])] = count;
            } else {
                int oldCount = wakeupCount[stoi(info[0])];
                wakeupCount[stoi(info[0])] = oldCount < count ? count : oldCount;
            }
            max = max < count ? count : max;
        }
    }

    DEBUG("wakeup count size: %d, max count value: %d", wakeupCount.size(), max);
    bool change = false;
    for (auto subTid : wakeupCount) {
        float curRatio = (float)subTid.second / max;
        DEBUG("calculate tid: %d, count %d, calRatio %f", subTid.first, subTid.second, curRatio);
        if (curRatio < mRatio) {
            if (keyTaskInfo.find(subTid.first) != keyTaskInfo.end()) {
                DEBUG("remove keyThread: %d", subTid.first);
                keyTaskInfo.erase(subTid.first);
                keyThreadReportInfo.erase(subTid.first);
                DecisionDriver::getInstance().setKeyTask(mPid, subTid.first, 0);
                change = true;
            }
            continue;
        }

        if (keyTaskInfo.find(subTid.first) == keyTaskInfo.end()) {
            DEBUG("add keyThread: %d", subTid.first);
            keyTaskInfo[subTid.first] = mKeyWorkerBindValue;
            TaskManager::getInstance().setKeyThreadReportInfo(subTid.first, mKeyWorkerBindValue);
            DecisionDriver::getInstance().setKeyTask(mPid, subTid.first, mKeyWorkerBindValue);
            change = true;
        }
    }

    // if (change && isMonitorRunning()) { // stop when the keyworker is stable
    if(isMonitorRunning()) { // stop when the game is over or switch to BG
        setMonitorTask(mMainTask);
    }
}

void TaskManager::pickKeyWorker() {
    DEBUG("pick waker task");

    string taskInfo = getSystemFile(TASK_MONITOR_PATH);
    DEBUG("get wake up info: %s", taskInfo.c_str());
    int max = 1;
    // cache tid -> wakeupCount map
    map<int, int> wakeupCount;
    // vector item -> waker wakee wakeupCount
    vector<string> wakeupInfo;
    splitString(taskInfo, wakeupInfo, "\n");
    for (auto item : wakeupInfo) {
        DEBUG("wakeupInfo: %s", item.c_str());
        vector<string> info;
        splitString(item, info, " ");
        DEBUG("wakeupInfo waker: %s, wakee: %s, count: %s", info[0].c_str(), info[1].c_str(), info[2].c_str());
        int foregroundPid = ThreadState::getInstance().getForgroundPid();
        DEBUG("wakeupInfo waker: %s, wakerName: %s, wakee: %s, wakeeName: %s, count: %s", info[0].c_str(),
        getTaskNameByTid(foregroundPid, stoi(info[0])).c_str(), info[1].c_str(), getTaskNameByTid(foregroundPid, stoi(info[1])).c_str(), info[2].c_str());
        // consider threads wake up each other
        if (keyTaskInfo.find(stoi(info[0])) == keyTaskInfo.end()) {
            int count = stoi(info[2]);
            // consider more then two threads wake up by the same thread
            if (wakeupCount.find(stoi(info[0])) == wakeupCount.end()) {
                wakeupCount[stoi(info[0])] = count;
            } else {
                int oldCount = wakeupCount[stoi(info[0])];
                wakeupCount[stoi(info[0])] = oldCount < count ? count : oldCount;
            }
            max = max < count ? count : max;
        }
    }

    DEBUG("wakeup count size: %d, max count value: %d", wakeupCount.size(), max);
    // try to find more tids as wakee
    vector<int> wakerTids;
    for (auto subTid : wakeupCount) {
        float curRatio = (float)subTid.second / max;
        DEBUG("calculate tid: %d, count %d, calRatio %f", subTid.first, subTid.second, curRatio);
        if (curRatio < mRatio) {
            continue;
        }

        keyTaskInfo[subTid.first] = mKeyWorkerBindValue;
        DEBUG("add thread: %d", subTid.first);
        wakerTids.push_back(subTid.first);
    }

    DEBUG("current ratio: %f. key Task size: %d, waker size: %d ", mRatio, keyTaskInfo.size(), wakerTids.size());
    // more tids need to find waker
    if (wakerTids.size() > 0) {
        setMonitorTask(wakerTids);
        mRatio += mRatio * mDescendRate;
        return;
    }

    BinderMessage msg;
    msg.what = BINDER_MESSAGE_PICK_KEY_TASK_FINISHED;
    msg.pid = mPid;
    OIfaceServer::getInstance().sendMessage(msg);
}

void TaskManager::clearAll() {
    Mutex::Autolock _l(mTmLock);
    mRatio = 0;
    mDescendRate = 0;
    mPid = 0;
    mKeyThreadBindValue = 0;
    mKeyWorkerBindValue = 0;
    mGLThread = 0;
    keyThreadReportInfo.clear();
    keyTaskInfo.clear();
    mTaskLoad.clear();
    whiteListTask.clear();
    mMainTask.clear();
    monitorRunning = false;
}
