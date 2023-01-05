/*!
 * @file VpsClient.h
 *
 * @cr
 * Copyright (c) 2019 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * @services
 */

#ifndef _VPS_CLIENT_H_
#define _VPS_CLIENT_H_


#include <string>
#include <vector>

namespace android {
#ifdef __cplusplus
extern "C" {
#endif

struct VpsConfigure {
    std::string key;
    int32_t     value;
public:
    VpsConfigure(const char* o, int32_t v) : key(o), value(v) {};
};

bool GetConfigs(std::vector<VpsConfigure>* configs);
bool Register(const std::string& name, const std::vector<VpsConfigure>& config);
bool Unregister(const std::string& name);

#ifdef __cplusplus
}  // extern "C"
#endif

}  // namespace android

#endif  // _VPS_CLIENT_H_