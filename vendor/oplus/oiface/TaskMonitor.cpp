
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <signal.h>
#include <streambuf>
#include <fstream>
#include <sys/types.h>
#include <utils/Mutex.h>
#include "OIfaceModule.h"
#include "Utils.h"
#include "TaskMonitor.h"
#include "AffinityService.h"
#include "SurfaceFlingerProxyManager.h"
#include "OifaceCallbackManager.h"
#include "DecisionDriver.h"

using namespace android;
using namespace std;

ANDROID_SINGLETON_STATIC_INSTANCE( TaskMonitor );
int TaskMonitor::monitorPid = 0;
int TaskMonitor::monitorCycle = 0;
int TaskMonitor::exectimeThreshold = 0;
bool TaskMonitor::sShouldPoll = false;
Mutex mMonitorMutex;
vector<int> TaskMonitor::leadingCpu;
vector<int> TaskMonitor::cpuCapa;
vector<struct task_normalized_duration> TaskMonitor::taskNormalizedDuration;
int TaskMonitor::fpsThreshold = 180;
map<int, vector<int> > TaskMonitor::cpuFreqMap;

TaskMonitor::TaskMonitor() {
    INFO("inittialize TaskMonitor feature");
}

void TaskMonitor::start(int pid, int cycle, int threshold, int fpsTh) {
    if(!sShouldPoll) {
        leadingCpu.clear();
        getClusterLeadingCpuNo(leadingCpu);

        cpuCapa.clear();
        getCpuCapacity(cpuCapa);

        monitorPid = pid;
        monitorCycle = cycle;
        exectimeThreshold = threshold;
        fpsThreshold = fpsTh;

        cpuFreqMap.clear();
        cpuFreqMap = AffinityService::getInstance().getClusterFreq();
        pthread_t thread;
        if (pthread_create(&thread, NULL, monitior_thread, NULL)) {
            ERROR("TaskMonitor create sync monitior_thread failed");
            sShouldPoll = false;
        } else {
            INFO("start TaskMonitor cycle[%dms],threshold[%dms],pid[%d]", monitorCycle, threshold, pid);
            sShouldPoll = true;
        }
    } else {
        INFO("TaskMonitor thread has already started");
    }
}

void TaskMonitor::stopMonitor() {
    INFO("stop TaskMonitor pid[%d]", monitorPid);
    monitorPid = 0;
    monitorCycle = 0;
    exectimeThreshold = 0;
    sShouldPoll = false;
    fpsThreshold = 180;
    Mutex::Autolock lock(mMonitorMutex);
    taskNormalizedDuration.clear();
    cpuCapa.clear();
    leadingCpu.clear();
}

void TaskMonitor::stop() {
    if (sShouldPoll) {
        stopMonitor();
    } else {
        INFO("TaskMonitor thread has already stopped");
    }
}

void TaskMonitor::getNormalizedLoadRate(vector<struct task_normalized_rate>& normalizedRates) {
    Mutex::Autolock lock(mMonitorMutex);
    normalizedRates.clear();
    for(vector<task_normalized_duration>::iterator iter = taskNormalizedDuration.begin(); iter != taskNormalizedDuration.end(); iter++){
        string taskName = getTaskNameByTid(monitorPid, (*iter).tid);
        normalizedRates.push_back({
            .tid = (*iter).tid,
            .name = taskName,
            .rate = (*iter).taskNormalizedDuration / (monitorCycle + 0.0001),
        });
    }
    sort(normalizedRates.begin(), normalizedRates.end(), compareNormalizedRate);
}

int TaskMonitor::getPid() {
    return monitorPid;
}

int TaskMonitor::getFpsThreshold() {
    return fpsThreshold;
}

int TaskMonitor::getExectimeThreshold() {
    return exectimeThreshold;
}


void * TaskMonitor::monitior_thread(void *) {
    pthread_setname_np(pthread_self(), "TaskMonitor");
    vector<struct task_normalized_time> last_tasks;
    vector<struct task_normalized_time> cur_tasks;
    float curFps = 180.5;
    float uselessFps = SurfaceFlingerProxyManager::getInstance().getFps(); //first time the value is 0.00
    while (sShouldPoll) {
        {
            Mutex::Autolock lock(mMonitorMutex);
            taskNormalizedDuration.clear();
            cur_tasks.clear();
            getTaskNormalizedTimeByPid(monitorPid, cur_tasks);
            for (auto& task : cur_tasks) {
                bool found = false;
                unsigned long long normalizedDuration = 0;
                for (auto& last_task : last_tasks) {
                    if (last_task.tid == task.tid) {
                        found = true;
                        normalizedDuration = task.taskNormalizedTimeAll - last_task.taskNormalizedTimeAll;
                        break;
                    }
                }

                //found no task in last_tasks, but the task now is in cur_tasks ====>> a new task was created, should als be monitored
                if (false == found && last_tasks.size() > 0) {
                    normalizedDuration = task.taskNormalizedTimeAll;
                }

                taskNormalizedDuration.push_back({.tid =task.tid, .taskNormalizedDuration = normalizedDuration,});
                if (normalizedDuration > exectimeThreshold) {
                    string taskName = getTaskNameByTid(monitorPid, task.tid);
                    INFO("monitor thread task %lldms %s:%d  and the task was %s", normalizedDuration, taskName.c_str(), task.tid, found? "existing":"new");
                    BinderMessage msg;
                    msg.what = BINDER_MESSAGE_NOTIFY_NORMALIZED_EXEC_TIME;
                    msg.pid = monitorPid;
                    msg.value = stoi(to_string(normalizedDuration));
                    msg.json = taskName + ":" + to_string(task.tid);
                    OIfaceServer::getInstance().sendMessage(msg);
                }
            }
        }

        last_tasks.clear();
        last_tasks.swap(cur_tasks);
        curFps = SurfaceFlingerProxyManager::getInstance().getFps();
        usleep(monitorCycle*1000);
    }
    INFO("TaskMonitor oiface_monitior_thread exit never arrivied here");
    return NULL;
}

void TaskMonitor::getTaskNormalizedTimeByPid(int pid, vector<struct task_normalized_time>& tasks) {
    vector<int> monitorTids;
    monitorTids.clear();
    getTidsByPid(pid, monitorTids);
    for(vector<int>::iterator iter = monitorTids.begin(); iter != monitorTids.end(); iter++) {
        struct task_normalized_time item;
        getTaskNormalizedTime(pid, *iter, item);
        if (item.tid != -1) {
            tasks.push_back(item);
        }
    }
    return;
}

void TaskMonitor::getTaskNormalizedTime(int pid, int tid, struct task_normalized_time& taskNormalizedTime) {
    if (pid <= 0 || tid <= 0) {
        //ERROR("invalid pid %d tid %d", pid, tid);
        return;
    }

    struct task_time_in_state taskTimeInState;
    getTaskTimeInStateByTid(pid, tid, taskTimeInState);
    taskNormalizedTime.tid = taskTimeInState.tid;

    if (taskTimeInState.tid != tid) {
        // found no task whose id is tid
        taskNormalizedTime.tid = -1;
        return;
    }

    unsigned long long ret = 0;
    for(vector<int>::iterator iter = leadingCpu.begin(); iter != leadingCpu.end(); iter++) {
        int clusterNum = 0;
        int cpuno = *iter;
        int capa = cpuCapa[cpuno];
        int maxCpuFreq = *(cpuFreqMap[clusterNum].rbegin());
        auto it_find = taskTimeInState.execTimeOnFreq.find(cpuno);
        if (it_find != taskTimeInState.execTimeOnFreq.end()) {
            map<unsigned int, unsigned long long> execTimeForEachFreq = it_find->second;
            auto it = execTimeForEachFreq.begin();
            unsigned long long addOn = 0;
            for (; it != execTimeForEachFreq.end(); ) {
                addOn = addOn + it->second  * capa * it->first;
                //ERROR("cpuNo[%d,%d] [%d,%lld] addon %lld,", cpuno, capa, it->first, it->second, it->second * capa * it->first);
                it++;
            }
            ret = ret + addOn / maxCpuFreq;
            clusterNum++;
        }
    }
    int maxCpuCapa = cpuCapa[*(leadingCpu.rbegin())];
    // time_in_state 的单位是10ms�? 归一化为最大核、最大算�?1024的运行时间ms
    ret = ret * 10 / maxCpuCapa;

    taskNormalizedTime.taskNormalizedTimeAll = ret;
}

void TaskMonitor::getTaskTimeInStateByTid(int pid, int tid, struct task_time_in_state& taskTimeInState) {
    char path[256];
    snprintf(path, sizeof(path), "/proc/%d/task/%d/time_in_state", pid, tid);

    if (access(path, R_OK) < 0) {
        ERROR("access %s failed(%s)\n", path, strerror(errno));
        taskTimeInState.tid = -1;
        return;
    }
    taskTimeInState.tid = tid;

    string line;
    int cpuNo = -1;
    int readLineCnt = 0;
    const int maxTimeInStateLine = 150; // in case of std::getline repeat forever
    map<unsigned int, unsigned long long> timeInState;
    timeInState.clear();
    ifstream infile(path);
    while (std::getline(infile, line) && readLineCnt++ < maxTimeInStateLine) {
        unsigned int key;
        unsigned long long value;
        int len;
        if (strncmp("cpu", line.c_str(), strlen("cpu")) == 0) {
            if (cpuNo != -1) {
                taskTimeInState.execTimeOnFreq[cpuNo] = timeInState;
            }
            cpuNo = line.at(3) - '0';
            // init the time_in_stats value to 0
            timeInState.clear();
            vector<int> freqtab = cpuFreqMap[cpuNo];
            for (vector<int>::iterator iter = freqtab.begin(); iter != freqtab.end(); iter++) {
                timeInState[*iter] = 0;
            }
        } else {
            //value is 10ms
            len = sscanf(line.c_str(), "%d %llu\n", &key, &value);
            if (len < 2) {
                DEBUG("sscanf returns %d for %s", len, line.c_str());
                continue;
            }
            timeInState[key] = value;
            //INFO("timeInState pushed [%d:%llu]", key, value);
        }
    }
    //assign the last value
    if (cpuNo > 0) {
        taskTimeInState.execTimeOnFreq[cpuNo] = timeInState;
    }

    return;
}

unsigned long long TaskMonitor::calcTaskTimeInState(const struct task_time_in_state& previous, const struct task_time_in_state& current) {
    if ((previous.tid != current.tid) || (current.tid == 0)) {
        ERROR("calcTaskTimeInState pre tid %d, cur tid %d", previous.tid, current.tid);
        return 0;
    }

    unsigned long long ret = 0;
    int maxCpuCapa = 1024;
    //map <int, map<unsigned int, unsigned long long> > :: iterator it_find;
    // time_in_state 的单位是10ms
    for(vector<int>::iterator iter = leadingCpu.begin(); iter != leadingCpu.end(); iter++) {
        int clusterNum = 0;
        int cpuno = *iter;
        int capa = cpuCapa[*iter];
        int maxCpuFreq = *(cpuFreqMap[clusterNum].rbegin());
        auto it_find1 = previous.execTimeOnFreq.find(cpuno);
        auto it_find2 = current.execTimeOnFreq.find(cpuno);
        if (it_find1 == previous.execTimeOnFreq.end() || it_find2 == current.execTimeOnFreq.end()) {
            ERROR("cpuno %d has no time stats info", cpuno);
            return 0;
        }
        map<unsigned int, unsigned long long> pre = it_find1->second;
        map<unsigned int, unsigned long long> cur = it_find2->second;
        auto it_prev = pre.begin();
        auto it_cur = cur.begin();
        unsigned long long addOn = 0;
        for (; it_prev != pre.end(); ) {
            addOn = addOn + (it_cur->second - it_prev->second) * capa * it_prev->first;
            it_prev++;
            it_cur++;
            //int realCpuCapa = capa * it_prev->first / maxCpuFreq ;
            //ret = addOn + realCpuCapa * (it_cur->second - it_prev->second);
            //这样写是为例减少计算�?
        }
        ret = ret + addOn / maxCpuFreq;
        clusterNum++;
    }

    // time_in_state 的单位是10ms�? 归一化为最大核、最大算�?1024的运行时间ms
    ret = ret * 10 / maxCpuCapa;

    DEBUG("TaskTimeInState during latest time, normalized time is %lld", ret);
    return ret;
}

