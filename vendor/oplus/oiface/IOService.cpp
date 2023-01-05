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

#include "Utils.h"
#include "IOService.h"

#define MMC_HIGH_FREQ   "200000000"
#define MMC_NORMAL_FREQ "50000000"
#define SCSI_HIGH_FREQ   "0"
#define SCSI_NORMAL_FREQ "1"


using namespace android;

ANDROID_SINGLETON_STATIC_INSTANCE(IOService);

int IOService::boost(int timeoutMs) {

    TimedJob::Job job;

    DEBUG("boost with timeout %d", timeoutMs);
    if (timeoutMs != 0) {
        if(isFileExist(MMC_PATH)) {
            mIOType = IO_MMC;
            if (writeFile(MMC_PATH, MMC_HIGH_FREQ) < 0) {
                ERROR("set to MMC high IO frequency failed");
                return -1;
            }
        } else if (isFileExist(SCSI_PATH)) {
            mIOType = IO_SCSI;
            if (writeFile(SCSI_PATH, SCSI_HIGH_FREQ) < 0) {
                ERROR("set to SCSI high IO frequency failed");
                return -1;
            }
        } else {
            ERROR("MMC and SCSI path is not exist");
            return -1;
        }
    }

    job.timeout = timeoutMs * 1000000LL;
    job.name = "io";
    job.instance = this;
    if (job.timeout != 0LL) {
        if (addJob(job) < 0) { /* use 0 as job id */
            ERROR("add job failed");
            return -1;
        }
    } else {
        if (cancelJob(job.name) < 0) {
            ERROR("cancel job failed");
            return -1;
        }
    }

    return 0;
}

int IOService::handleTimeout(int32_t*, int) {
    if(mIOType == IO_MMC) {
        return writeFile(MMC_PATH, MMC_NORMAL_FREQ);
    } else {
        return writeFile(SCSI_PATH, SCSI_NORMAL_FREQ);
    }
}

