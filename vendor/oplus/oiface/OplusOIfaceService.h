/*
 * Copyright(C) 2020 OPlus. All rights reserved.
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
#ifndef __OPLUS_OIFACE_SERVICE_H__
#define __OPLUS_OIFACE_SERVICE_H__

#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <cutils/compiler.h>
#include "HealthListener.h"

class IOplusOIfaceService: public android::IInterface {
    public:
        static char const* getServiceName() ANDROID_API {
            return "oplusoiface";
        }

        DECLARE_META_INTERFACE(OplusOIfaceService);
        enum {
            CURRENT_NETWORK                     = android::IBinder::FIRST_CALL_TRANSACTION,
            CURRENT_PACKAGE                     = android::IBinder::FIRST_CALL_TRANSACTION + 1,
            RUN_PROGRAM                         = android::IBinder::FIRST_CALL_TRANSACTION + 2,
            COLLECT_DATA                        = android::IBinder::FIRST_CALL_TRANSACTION + 3,
            /* fake notifier */
            FAKE_SYSTEM_NOTIFY                  = android::IBinder::FIRST_CALL_TRANSACTION + 4,
            /* bind task */
            BIND_TASK                           = android::IBinder::FIRST_CALL_TRANSACTION + 6,
            BIND_SINGLE_TASK                    = android::IBinder::FIRST_CALL_TRANSACTION + 7,
            BIND_SINGLE_TASK_TIMEOUT            = android::IBinder::FIRST_CALL_TRANSACTION + 8,

            /* register hqv */
            ENABLE_HQV                          = android::IBinder::FIRST_CALL_TRANSACTION + 10,
            REGISTER_HQV                        = android::IBinder::FIRST_CALL_TRANSACTION + 11,
            SET_HALF_HQV                        = android::IBinder::FIRST_CALL_TRANSACTION + 12,

            /*Change the log level*/
            SET_LOG_LEVEL                       = android::IBinder::FIRST_CALL_TRANSACTION + 13,

            /* system */
            GET_SUPPORT_PACKAGE                  = android::IBinder::FIRST_CALL_TRANSACTION + 200,
            REGISTER_NETWORK_LISTENER            = android::IBinder::FIRST_CALL_TRANSACTION + 201,
            LAYER_FRAME_STATS_DATA               = android::IBinder::FIRST_CALL_TRANSACTION + 202,
            REGISTER_GAME_ROLE_EVENT_LISTENER    = android::IBinder::FIRST_CALL_TRANSACTION + 203,
            UNREGISTER_GAME_ROLE_EVENT_LISTENER  = android::IBinder::FIRST_CALL_TRANSACTION + 204,
            GET_SUPPORT_GAME_START_PACKAGE       = android::IBinder::FIRST_CALL_TRANSACTION + 205,
            GET_ALL_LOAD_INFO                    = android::IBinder::FIRST_CALL_TRANSACTION + 206,
            NOTIFY_TOUCH_TRACE                   = android::IBinder::FIRST_CALL_TRANSACTION + 207,
            GET_SYSTEM_THERMAL                   = android::IBinder::FIRST_CALL_TRANSACTION + 208,
            NOTIFY_TEMP_LEVEL                    = android::IBinder::FIRST_CALL_TRANSACTION + 209,
            SET_GCP_EFFECT_MODE                  = android::IBinder::FIRST_CALL_TRANSACTION + 210,
            GET_ALL_LOAD_INFO_R                  = android::IBinder::FIRST_CALL_TRANSACTION + 211,
            /* SF */
            REGISTERED_LAYER_DELAY_BOOST        = android::IBinder::FIRST_CALL_TRANSACTION + 300,
            NOTIFY_GAME_JITTER                  = android::IBinder::FIRST_CALL_TRANSACTION + 301,
            /* GUI */
            NOTIFY_BUFFER_PRODUCER              = android::IBinder::FIRST_CALL_TRANSACTION + 403,
            NOTIFY_BUFFER_CONSUMER              = android::IBinder::FIRST_CALL_TRANSACTION + 404,
            /* MISC */
            GET_DEVICE_ID                       = android::IBinder::FIRST_CALL_TRANSACTION + 500,
            GAMESPACE_PERFMODE_EVENT            = android::IBinder::FIRST_CALL_TRANSACTION + 501,
            GAMESPACE_SCREEN_EVENT              = android::IBinder::FIRST_CALL_TRANSACTION + 502,
            GET_DEVICE_ID_NEW                   = android::IBinder::FIRST_CALL_TRANSACTION + 503,
            /* HYPNUSD */
            HYPNUSD_PERFMODE_EVENT              = android::IBinder::FIRST_CALL_TRANSACTION + 600,
            HYPNUSD_SCREEN_EVENT                = android::IBinder::FIRST_CALL_TRANSACTION + 601,

            /* code 701 is used by com.oplus.persist.system */

            /* COSA Interface */
            COSA_ENABLE_TGPA_BIND_TASK          = android::IBinder::FIRST_CALL_TRANSACTION + 800,
            COSA_ENABLE_TGPA_LIMTING_NOTIFY     = android::IBinder::FIRST_CALL_TRANSACTION + 801,
            COSA_CURRENT_PKG_STATUS             = android::IBinder::FIRST_CALL_TRANSACTION + 802,

            COSA_GET_FPS                        = android::IBinder::FIRST_CALL_TRANSACTION + 850,
            COSA_GENERAL_OIFACE_SIGNAL          = android::IBinder::FIRST_CALL_TRANSACTION + 851,
            COSA_OIFACE_DECISION                = android::IBinder::FIRST_CALL_TRANSACTION + 852,
            COSA_OIFACE_CONTROL                 = android::IBinder::FIRST_CALL_TRANSACTION + 853,
            COSA_GET_CPU_CLUSTER_NUM            = android::IBinder::FIRST_CALL_TRANSACTION + 854,
            COSA_GET_CPU_AVAILABLE_FREQ_TABLE   = android::IBinder::FIRST_CALL_TRANSACTION + 855,
            COSA_GET_CPU_LIMITED_FREQS          = android::IBinder::FIRST_CALL_TRANSACTION + 856,
            COSA_GET_CPU_CURRENT_FREQ           = android::IBinder::FIRST_CALL_TRANSACTION + 857,
            COSA_GET_CPU_LOADS                  = android::IBinder::FIRST_CALL_TRANSACTION + 858,
            COSA_GET_GPU_AVAILABLE_FREQ_TABLE   = android::IBinder::FIRST_CALL_TRANSACTION + 859,
            COSA_GET_GPU_LIMITED_FREQS          = android::IBinder::FIRST_CALL_TRANSACTION + 860,
            COSA_GET_GPU_CURRENT_FREQ           = android::IBinder::FIRST_CALL_TRANSACTION + 861,
            COSA_GET_GPU_LOADS                  = android::IBinder::FIRST_CALL_TRANSACTION + 862,
            COSA_GET_THERMAL_TEMPS              = android::IBinder::FIRST_CALL_TRANSACTION + 863,

            /*code 900 for enable/disable hapticscreencap use fo fourd haptic vibration */
            ENABLE_HAPTIC_VABRATION             = android::IBinder::FIRST_CALL_TRANSACTION + 900,

            SET_TOUCH_SENSIBILITY               = android::IBinder::FIRST_CALL_TRANSACTION + 901,
            SET_TOUCH_RESPONSIVENESS            = android::IBinder::FIRST_CALL_TRANSACTION + 902,
            SET_GYROSCOPE_LEVEL                 = android::IBinder::FIRST_CALL_TRANSACTION + 903,
            SET_TOUCH_PROTECTION                = android::IBinder::FIRST_CALL_TRANSACTION + 904,

            /* callback register since R */
            REGISTER_OIFACE_CALLBACK            = android::IBinder::FIRST_CALL_TRANSACTION + 1000,
            GET_BATTERY_REMAIN                  = android::IBinder::FIRST_CALL_TRANSACTION + 1001,
            GET_BATTERY_CURRENT_NOW             = android::IBinder::FIRST_CALL_TRANSACTION + 1002,
            GET_SUPER_VOOC_STATUS               = android::IBinder::FIRST_CALL_TRANSACTION + 1003,
            GET_GPA_SYSTEM_INFO                 = android::IBinder::FIRST_CALL_TRANSACTION + 1004,
            SET_COOLEX_FILETER_TYPE             = android::IBinder::FIRST_CALL_TRANSACTION + 1005,
            SET_GAME_MODE_STATSUS               = android::IBinder::FIRST_CALL_TRANSACTION + 1006,
            GET_GAME_MODE_STATSUS               = android::IBinder::FIRST_CALL_TRANSACTION + 1007,
            GET_CURRENT_GAME_PACKAGE            = android::IBinder::FIRST_CALL_TRANSACTION + 1008,
            SET_INSTALLED_GAME_LIST             = android::IBinder::FIRST_CALL_TRANSACTION + 1009,
            GET_INSTALLED_GAME_LIST             = android::IBinder::FIRST_CALL_TRANSACTION + 1010,
            GET_CPU_TIME_IN_STATE               = android::IBinder::FIRST_CALL_TRANSACTION + 1011,
            TRIGGER_FRAME_STAT                  = android::IBinder::FIRST_CALL_TRANSACTION + 1012,
            GET_CHIP_NAME                       = android::IBinder::FIRST_CALL_TRANSACTION + 1013,
            GET_CPU_CLUSTER_INFO                = android::IBinder::FIRST_CALL_TRANSACTION + 1014,
            GET_BATTERY_FCC                     = android::IBinder::FIRST_CALL_TRANSACTION + 1015,

            TASK_MONITOR                        = android::IBinder::FIRST_CALL_TRANSACTION + 1020,

            REGISTER_COOLEX_CALLBACK            = android::IBinder::FIRST_CALL_TRANSACTION + 2000,
            GET_COLORX_STATUS                   = android::IBinder::FIRST_CALL_TRANSACTION + 2001,
            GET_COOLEX_RUS_DATA                 = android::IBinder::FIRST_CALL_TRANSACTION + 2002,
            GET_4D_SUPPORT_STATUS               = android::IBinder::FIRST_CALL_TRANSACTION + 2003,
            REPPORT_WEAPON_NAME                 = android::IBinder::FIRST_CALL_TRANSACTION + 2004,
            GET_DYNAMIC_RESOLUTION_SWITCH       = android::IBinder::FIRST_CALL_TRANSACTION + 2005,
            DYNAMIC_RESOLUTION_EVENT_TRACE      = android::IBinder::FIRST_CALL_TRANSACTION + 2006,

            NOTIFY_GL_THREADS                   = android::IBinder::FIRST_CALL_TRANSACTION + 3000,
            NOTIFY_GET_DATA                     = android::IBinder::FIRST_CALL_TRANSACTION + 3001,
        };

        virtual void currentNetwork(int32_t status) = 0;
        virtual void currentPackage(const android::String16 &package, int32_t uid, int32_t pid) = 0;
};

class OplusOIfaceService: public android::BnInterface<IOplusOIfaceService> {
    public:
        virtual ~OplusOIfaceService() {ALOGE("OplusOIfaceService destructor");};
        OplusOIfaceService();
    protected:
        virtual android::status_t onTransact(uint32_t code,
                                             const android::Parcel& data,
                                             android::Parcel *reply,
                                             uint32_t flags = 0) override;

        virtual void currentNetwork(int32_t status) override;
        virtual void currentPackage(const android::String16 &package,
                                    int32_t uid,
                                    int32_t pid) override;
        virtual android::status_t dump(int fd, const android::Vector<android::String16>& args);

    private:
        int getSupportPackage(std::string &pkgStatJson);            // pkgStat);
        // int getSupportPackage(Map &pkgStat);
        int getSupportGameStartPackage(std::string &pkgStatJson);    // &pkgStat);
        // int getSupportGameStartPackage(Map &pkgStat);
        void packageSwitch(const android::Parcel& data);
        bool checkOifaceWhiteList(int type, int uid);
        std::string generalOIfaceSignal(std::string &signal, int uid);
        std::string getTimeInState();
        std::string setOifaceLogLevel(int level, int uid);
        int old_mode;
        std::string openID;
        std::map<int, bool> permissionedUidList;
        android::Mutex mMutex;
        bool isGameSafePermissionCheckedOK(int pid, int uid);
};

class IOIfaceCallback: public android::IInterface {
    public:
        DECLARE_META_INTERFACE(OIfaceCallback);

        virtual android::String16 onFBNotification(int status) = 0;
        virtual android::String16 onGPANotification(const std::string &info) = 0;
        virtual int onTGPAInfo(const std::string &info, int uid, int pid) = 0;
        virtual int onHyperBoostInfo(const std::string &info, int uid, int pid) = 0;
        virtual int onEngineBoostINfo(const std::string &info, int uid, int pid) = 0;
        virtual int onOifaceGeneralInfo(const std::string &info, int type, int uid, int pid) = 0;
        virtual int onGameStatusChanged(const std::string &packageName, const std::string &gamestat) = 0;
        virtual int onNetworkChanged(const std::string &packageName, int latency) = 0;
        virtual int onSystemNotify(const std::string &result) = 0;
        virtual int onThermalStatusChanged(const std::string &status) = 0;
        virtual int onGameJitter(const std::string &packageName, int fps) = 0;

        enum {
            FB_NOTIFICATION = android::IBinder::FIRST_CALL_TRANSACTION,
            GPA_NOTIFICATION,
            TGPA_INFO,
            HYPERBOOST_INFO,
            ENGINEBOOST_INFO,
            OIFACE_GENERAL_INFO,
            GAME_STATUS_CHANGED,
            NETWORK_CHANGED,
            SYSTEM_NOTIFY,
            THERMAL_STATUS_CHANGED,
            GAME_JITTER,
        };
};

class BpOIfaceCallback: public android::BpInterface<IOIfaceCallback> {
    public:
        BpOIfaceCallback(const android::sp<android::IBinder>& impl)
            : BpInterface<IOIfaceCallback>(impl) {}

        virtual android::String16 onFBNotification(int status);
        virtual android::String16 onGPANotification(const std::string &info);
        virtual int onTGPAInfo(const std::string &info, int uid, int pid);
        virtual int onHyperBoostInfo(const std::string &info, int uid, int pid);
        virtual int onEngineBoostINfo(const std::string &info, int uid, int pid);
        virtual int onOifaceGeneralInfo(const std::string &info, int type, int uid, int pid);
        virtual int onGameStatusChanged(const std::string &packageName, const std::string &gamestat);
        virtual int onNetworkChanged(const std::string &packageName, int latency);
        virtual int onSystemNotify(const std::string &result);
        virtual int onThermalStatusChanged(const std::string &status);
        virtual int onGameJitter(const std::string &packageName, int fps);
};

class ICoolExCallback: public android::IInterface {
    public:
        DECLARE_META_INTERFACE(CoolExCallback);
        virtual bool onReportCoolExFilterType(int type, std::string &config) = 0;
        enum {
            REPORT_COOLEX_FILTER_TYPE = android::IBinder::FIRST_CALL_TRANSACTION,
        };
};

class BpCoolExCallback: public android::BpInterface<ICoolExCallback> {
    public:
        BpCoolExCallback(const android::sp<android::IBinder>& impl)
            : BpInterface<ICoolExCallback>(impl) {}
        virtual bool onReportCoolExFilterType(int type, std::string &config);
};

class IOIfaceNetworkNotifier: public android::IInterface {
    public:
        DECLARE_META_INTERFACE(OIfaceNetworkNotifier);
        virtual int onNetworkNotify(const std::string&, int32_t ) = 0;
        enum {
            NETWORK_NOTIFY = android::IBinder::FIRST_CALL_TRANSACTION,
        };
};

class BpOIfaceNetworkNotifier: public android::BpInterface<IOIfaceNetworkNotifier> {
    public:
        BpOIfaceNetworkNotifier(const android::sp<android::IBinder>& impl)
            : BpInterface<IOIfaceNetworkNotifier>(impl) {}
        virtual int onNetworkNotify(const std::string& pkgName, int32_t latency);
};

class IGameStatusNotifier: public android::IInterface {
    public:
        DECLARE_META_INTERFACE(GameStatusNotifier);
        virtual int onGameStatusNotify(const std::string&, const std::string&) = 0;
        // virtual int onGameStatusNotify(const std::string&, const Map&) = 0;
        enum {
            GAME_STATUS_NOTIFY = android::IBinder::FIRST_CALL_TRANSACTION,
        };
};

class BpGameStatusNotifier: public android::BpInterface<IGameStatusNotifier> {
    public:
        BpGameStatusNotifier(const android::sp<android::IBinder>& impl)
            : BpInterface<IGameStatusNotifier>(impl) {}
        virtual int onGameStatusNotify(const std::string& pkgName, const std::string&);
};

#endif // __OPLUS_OIFACE_SERVICE_H__