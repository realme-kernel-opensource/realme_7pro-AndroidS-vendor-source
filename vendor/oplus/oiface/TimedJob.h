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

#ifndef __TIMED_JOB_H__
#define __TIMED_JOB_H__

#include <signal.h>
#include <time.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <pthread.h>
#include <utils/SortedVector.h>
#include <map>
#include <string>

class TimedJob {
public:
    TimedJob();
    /* singlton should never be called */
    virtual ~TimedJob() {};

    static const int kPayloadSize = 4;
    /* Job pieces */
    struct Job {
        int64_t timeout;
        /* XXX:name is identifier */
        std::string name;
        timer_t timer;
        TimedJob *instance;
        int32_t data[kPayloadSize];
    };

    /* XXX: job may be modified */
    int addJob(const Job& job);

protected:
    virtual int handleTimeout(int32_t *data, int size) = 0;
    static int cancelJob(const std::string& name);

private:
    /* timer handler */
    static void timerHandler (union sigval val);
    /* all the timed jobs stored in this structure. XXX: string keys performance? */
    static std::map<const std::string, Job> sJobs;
    static android::Mutex sLock;

    int createTimer(Job *pJob);
    static void dumpJobs();
};

#endif
