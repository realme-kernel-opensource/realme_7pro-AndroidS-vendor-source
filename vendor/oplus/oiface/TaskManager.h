#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <utils/Singleton.h>
#include <utils/Mutex.h>

using namespace android;
using namespace std;

class TaskManager: public Singleton<TaskManager> {
public:
    TaskManager();
    virtual ~TaskManager() {};
    void pickHeavyTask(pid_t gamePid, int pickNum);
    void pickKeyWorker();
    void findKeyWorker();
    void setMonitorTask(vector<int> tids);
    void clearAll();

    void setRatio(double ratio) { mRatio = ratio > 0 ? ratio : 0; }
    double getRatio() {return mRatio; }
    void setDescendRate(double descendRate) { mDescendRate = descendRate > 0 ? descendRate : 0; }
    double getDescendRate() { return mDescendRate; }
    void setKeyThreadBindValue(int bindValue) { mKeyThreadBindValue = bindValue > 0 ? bindValue : 0; }
    int getKeyThreadBindValue() { return mKeyThreadBindValue; }
    void setKeyWorkerBindValue(int bindValue) { mKeyWorkerBindValue = bindValue > 0 ? bindValue : 0; }
    int getKeyWorkerBindValue() { return mKeyWorkerBindValue; }
    void setPid(pid_t pid) { mPid = pid > 0 ? pid : 0; }
    pid_t getPid() { return mPid; }
    void setGLThread(pid_t tid) { mGLThread = tid > 0 ? tid : 0; }
    void setKeyThreadReportInfo(pid_t tid, int bindValue) {
        if (tid < 0 || bindValue < 0) {
            DEBUG("record key thread failed, tid: %d bindValue: %d is invalid.", tid, bindValue);
            return;
        }
        keyThreadReportInfo[tid] = bindValue;
    }
    map<int, int>& getKeyThreadReportInfo() {
        return keyThreadReportInfo;
    }
    pid_t getGLThread() { return mGLThread; }
    void setKeyTaskInfo(int taskTid, int bindValue) {
        if (taskTid < 0 || bindValue < 0) {
            DEBUG("record task info failed, tid: %d bindValue: %d is invalid.", taskTid, bindValue);
            return;
        }
        keyTaskInfo[taskTid] = bindValue;
    }
    map<int, int>& getKeyTaskInfo() {
        return keyTaskInfo;
    }

    void setHeavyTask(const pid_t tid, const int normalizedTime);
    void getHeavyTask(map<int, int>& outTaskLoad);
    void setWhiteListTask(pid_t tid, int bindValue) {
        if (tid < 0 || bindValue < 0) {
            DEBUG("record white list task failed, tid: %d bindValue: %d is invalid.", tid, bindValue);
            return;
        }
        whiteListTask[tid] = bindValue;
    }
    map<int, int>& getWhiteListTask() {
        return whiteListTask;
    }
    void setMainTask(const pid_t tid) {
        if (tid < 0) {
            DEBUG("main task tid: %d is invalid.", tid);
            return;
        }
        mMainTask.push_back(tid);
    }
    vector<pid_t>& getMainTask() {
        return mMainTask;
    }
    void setMonitorRunning(bool running) {
        monitorRunning = running;
    }
    bool isMonitorRunning() {
        return monitorRunning;
    }

private:
    android::Mutex mTmLock;
    double mRatio = 0;
    double mDescendRate = 0;
    pid_t mPid = 0;
    int mKeyThreadBindValue = 0;
    int mKeyWorkerBindValue = 0;
    pid_t mGLThread = 0;
    // cache tid -> uifirst value map, for big data
    map<int, int> keyThreadReportInfo;
    // cache tid -> key worker value map, for white list record manually
    map<int, int> whiteListTask;
    // cache tid -> key thread map, for recognition automatically
    map<int, int> keyTaskInfo;
    // cache tid -> normalized load map, for recording load of each thread
    map<int, int> mTaskLoad;
    // record the threads use to find the waker thread
    vector<int> mMainTask;
    // monitor game running status, start or stop
    bool monitorRunning = false;
};

#endif
