#ifndef __TASK_MONITOR_H__
#define __TASK_MONITOR_H__

#include <vector>
#include <string>
#include <utils/Singleton.h>

struct task_time_in_state {
    int tid;
    std::map<int, std::map<unsigned int, unsigned long long> > execTimeOnFreq;
};

struct task_normalized_time{
    int tid;
    unsigned long long taskNormalizedTimeAll;
};

struct task_normalized_duration{
    int tid;
    unsigned long long taskNormalizedDuration;
};

struct task_normalized_rate{
    int tid;
    std::string name;
    double rate;
};

class TaskMonitor: public android::Singleton<TaskMonitor> {
public:
    TaskMonitor();
    virtual ~TaskMonitor() {};
    void start(int pid, int cycle, int threshold, int fpsTh);
    void stop();
    void stopMonitor();
    int getFpsThreshold();
    int getExectimeThreshold();
    void getNormalizedLoadRate(vector<struct task_normalized_rate>& normalizedRates);
    int getPid();
    static unsigned long long calcTaskTimeInState(const struct task_time_in_state& previous, const struct task_time_in_state& current);
    static void getTaskTimeInStateByTid(int pid, int tid, struct task_time_in_state& taskTimeInState);
    static void getTaskNormalizedTimeByPid(int pid, vector<struct task_normalized_time>& tasks);
    static void getTaskNormalizedTime(int pid, int tid, struct task_normalized_time& taskNormalizedTime);

    static bool sShouldPoll;
    static int monitorPid;

    static int monitorCycle;
    static int exectimeThreshold;
    static int fpsThreshold;
    static vector<struct task_normalized_duration> taskNormalizedDuration;

    static vector<int> leadingCpu;
    static vector<int> cpuCapa;
    static map<int, vector<int> > cpuFreqMap;

    static void * monitior_thread(void *cookie);

    static bool compareNormalizedRate(const struct task_normalized_rate& task1, const struct task_normalized_rate& task2) {
        return task1.rate > task2.rate;
    }
};

#endif

