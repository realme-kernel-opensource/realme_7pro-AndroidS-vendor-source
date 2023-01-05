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

#ifndef __OIMPARSER_H__
#define __OIMPARSER_H__

#include "OIface.h"
#include "OIfaceServer.h"
#include "PlatformAdaptor.h"
#include "OIfaceModuleLoader.h"
#include "DecisionDriver.h"

class OimParser: public SocketParser
{
public:
    OimParser(const char *socketName, int fd, int type, int pid, int uid);
    virtual ~OimParser();

    virtual bool isInitialized();

    virtual int handlePollIn();

protected:
    /* oplus Interface Message prototype */
    struct OimResponse {
        uint32_t    tag;
        uint16_t    version;
        uint16_t    func_id;
        uint32_t    ret_code;
        uint32_t    body_len;
        int64_t     request_id;
        int64_t     timestamp;
        char        str[MAX_IM_CALLBACK_DECSION_LEN];
    } __attribute__((packed));

    /* consume one message from mRawBuffe
     * return > 0 on still have message.
     * return 0 on all message are handled(include partial message).
     * return -1 on error.
     */
    virtual int handleOneMessage();
    virtual int performAction(OimResponse& resp);

private:
    DISALLOW_COPY_AND_ASSIGN(OimParser);

    OIfaceModuleLoader *mLoader;
    string mRawBuffer;
    DecisionDriver& mDriver;
    Decision mDecision[MAX_DECISIONS];
    int64_t mStartTime;
    int checkPermission();
    int sendResponse(const OimResponse &resp);
};

#endif

