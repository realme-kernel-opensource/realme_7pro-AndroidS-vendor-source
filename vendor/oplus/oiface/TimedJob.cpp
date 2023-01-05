/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ------------------------------- Revision History: ----------------------------
 * <author>                   <date>       <version>   <desc>
 * ------------------------------------------------------------------------------
 */

#include <signal.h>

#include "TimedJob.h"
#include "OIface.h"

using namespace std;

android::Mutex TimedJob::sLock;
map<const string, TimedJob::Job> TimedJob::sJobs;

#define SEC_TO_NS   (1000000000LL)

TimedJob::TimedJob() {
}

int TimedJob::createTimer(Job *pJob) {
    struct sigevent se;
    timer_t timer;
    struct itimerspec its;

    /* modify value */
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = pJob->timeout / SEC_TO_NS;
    its.it_value.tv_nsec = pJob->timeout % SEC_TO_NS;

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_signo = SIGALRM;
    se.sigev_value.sival_ptr = pJob;
    se.sigev_notify_function = timerHandler;
    se.sigev_notify_attributes = NULL;

    if (timer_create(CLOCK_MONOTONIC, &se, &timer) < 0) {
        ERROR("create timer failed.(%s)", strerror(errno));
        return -1;
    }

    pJob->timer = timer;

    if (timer_settime(timer, 0, &its, NULL) < 0) {
        ERROR("set time failed.(%s)", strerror(errno));
        return -1;
    }

    DEBUG("timer created");

    return 0;
}

int TimedJob::addJob(const Job& job) {
    struct itimerspec its;

    /* modify value */
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = job.timeout / SEC_TO_NS;
    its.it_value.tv_nsec = job.timeout % SEC_TO_NS;

    /* Lock sJobs */
    android::Mutex::Autolock _l(sLock);

    /* modify job if already exist */
    map<const string, Job>::iterator iter = sJobs.find(job.name);
    if (iter != sJobs.end()) {
        INFO("timer %s is already added\n", job.name.c_str());

        memcpy(iter->second.data, job.data, sizeof(job.data));

        if (timer_settime(iter->second.timer, 0, &its, NULL) < 0) {
            ERROR("set time failed.(%s)", strerror(errno));
            return -1;
        }

        return 0;
    }

    /* add a new job */
    sJobs[job.name] = job;

    /* point to job instance in container */
    if (createTimer(&sJobs[job.name])) {
        ERROR("create timer failed");
        sJobs.erase(job.name);
        return -1;
    }

    dumpJobs();

    return 0;
}

void TimedJob::timerHandler(union sigval val) {
    Job *pJob = (Job *)val.sival_ptr;
    INFO("job %s armed", pJob->name.c_str());

    if (cancelJob(pJob->name) < 0) {
        ERROR("cancel %s in timer handler failed", pJob->name.c_str());
        return;
    }
}

/* called in timer thread or main thread */
int TimedJob::cancelJob(const string& name) {
    sLock.lock();
    map<const string, Job>::iterator iter = sJobs.find(name);
    if (iter == sJobs.end()) {
        INFO("timer %s is not found\n", name.c_str());
        sLock.unlock();
        return 0;
    }

    if (timer_delete(iter->second.timer) < 0) { /* can't do anything if it fails */
        ERROR("delete timer failed(%s)", strerror(errno));
    }

    /* sThis should already be correctly set since this method is protected */
    /* Job dereferenced memory is released in handleTimeout */
    if (iter->second.instance->handleTimeout(iter->second.data, sizeof(Job::data)) < 0) {
        ERROR("handle timeout failed");
    }

    sJobs.erase(iter);
    sLock.unlock();

    INFO("job %s canceled", name.c_str());

    dumpJobs();
    return 0;
}

void TimedJob::dumpJobs() {

    DEBUG("***dump jobs***");

    for (map<const std::string, Job>::iterator iter = sJobs.begin(); iter != sJobs.end();
            iter++) {
        DEBUG("job:%s", iter->second.name.c_str());
    }
}
