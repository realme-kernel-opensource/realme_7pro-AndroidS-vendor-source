#include <binder/IPCThreadState.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <private/android_filesystem_config.h>
#include <cutils/properties.h>
#include <openssl/sha.h>
#include <dlfcn.h>

#include "OIfaceService.h"
#include "ThreadState.h"
#include "OIface.h"
#include "OIfaceServer.h"
#include "BinderClient.h"
#include "GeneralCheck.h"
#include "GlobalConfig.h"
#include "OIfaceModuleLoader.h"
#include "DataCollector.h"
#include "Utils.h"
#include "MemoryInfo.h"
#include "PlatformAdaptor.h"
#include "ThermalService.h"
#include "OifaceCallbackManager.h"
#include <cutils/properties.h>

#define OIFACE_PERMISSION   "com.oplus.permission.safe.GAME"

using namespace android;
using namespace std;

class BpOIfaceService: public BpInterface<IOIfaceService> {
    public:
        BpOIfaceService(const sp<IBinder> &impl): BpInterface<IOIfaceService>(impl) {}
};

IMPLEMENT_META_INTERFACE(OIfaceService, "com.oppo.oiface.IOIfaceService");
IMPLEMENT_META_INTERFACE(OIfaceNotifier, "com.oppo.oiface.IOIfaceNotifier");

status_t OIfaceService::onTransact(uint32_t code, const Parcel& data, Parcel* reply,
        uint32_t flags) {
    int pid = IPCThreadState::self()->getCallingPid();
    int uid = IPCThreadState::self()->getCallingUid();

    BinderMessage cosaMsg;
    if (code >= REGISTER_CLIENT_FROM_COSA && code <= SET_GENERAL_SINGAL_FROM_COSA) {
        switch (code) {
            case REGISTER_CLIENT_FROM_COSA:{
                CHECK_INTERFACE(IOIfaceService, data, reply);
                cosaMsg.binder  = data.readStrongBinder();
                cosaMsg.json = String8(data.readString16()).string();
                cosaMsg.uid = uid = data.readInt32();
                cosaMsg.pid = pid = data.readInt32();
            } break;
            case SET_GENERAL_SINGAL_FROM_COSA:{
                CHECK_INTERFACE(IOIfaceService, data, reply);
                cosaMsg.json = String8(data.readString16()).string();
                cosaMsg.uid = uid = data.readInt32();
                cosaMsg.pid = pid = data.readInt32();
            } break;
            default:
              break;
        }
        DEBUG("do the handle after cached cofig check");
    }

    if ((code >= REGISTER_CLIENT) && (code < GAME_SDK_END)) {
        uint32_t enable = 0;
        GlobalConfig::getInstance().getConfig(&enable, {"sdk", "enable"});
        if (enable == 0) {
            ERROR("game sdk is not enabled");
            return BBinder::onTransact(code, data, reply, flags);
        }

        // TODO: persmission check
        /*
        if (ThreadState::getInstance().getConfig(uid, &enable) == 0) {
            if (enable == 0) {
                DEBUG("gamesdk uid %d is disabled according to cached config", uid);

                return BBinder::onTransact(code, data, reply, flags);
            }
        }*/
    }

    switch (code) {
        case REGISTER_ENGINE_CLIENT:
        case REGISTER_CLIENT: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            BinderMessage msg;
            msg.what = (code == REGISTER_CLIENT) ? BINDER_MESSAGE_NEW_CLIENT : BINDER_MESSAGE_ENGINE_CLIENT;
            msg.uid = uid;
            msg.pid = pid;
            msg.binder  = data.readStrongBinder();
            msg.json = String8(data.readString16()).string();
            if (!msg.json.empty()) {
                vector<string> pkg;
                if (getPackageForUid(msg.uid, pkg) > 0) {
                    uint32_t enable = 0;
                    GlobalConfig::getInstance().getConfig(&enable, {"sdk", pkg[0].c_str(), "enable"});
                    if (enable == 1) {
                        reply->writeInt32(1);
                    } else {
                        reply->writeInt32(GeneralCheck::getInstance().checkPermission(msg.json, msg.uid));
                    }
                }
            }
            OIfaceServer::getInstance().sendMessage(msg);
        } break;

        case REGISTER_CLIENT_FROM_COSA: {
            cosaMsg.what = BINDER_MESSAGE_NEW_CLIENT;
            reply->writeNoException();
            if (!cosaMsg.json.empty()) {
                vector<string> pkg;
                if (getPackageForUid(cosaMsg.uid, pkg) > 0) {
                    uint32_t enable = 0;
                    GlobalConfig::getInstance().getConfig(&enable, {"sdk", pkg[0].c_str(), "enable"});
                    if (enable == 1) {
                        reply->writeInt32(1);
                    } else {
                        // reply->writeInt32(GeneralCheck::getInstance().checkPermission(cosaMsg.json, cosaMsg.uid));
                        reply->writeInt32(1);
                    }
                } else {
                    reply->writeInt32(0);
                }
            }
            OIfaceServer::getInstance().sendMessage(cosaMsg);
        } break;

        case SET_GENERAL_SINGAL_FROM_COSA: {
            cosaMsg.what = BINDER_MESSAGE_UPDATE_GAME_INFO;
            OIfaceServer::getInstance().sendMessage(cosaMsg);

            reply->writeNoException();
            DEBUG("reply for cosa general signal");
            reply->writeString16(String16("{}"));
        } break;

        case REGISTER_NOTIFIER: {
            CHECK_INTERFACE(IOIfaceService, data, reply);

            BinderMessage msg;
            msg.what = BINDER_MESSAGE_REGISTER_NOTIFIER;
            msg.uid = uid;
            msg.pid = pid;

            OIfaceServer::getInstance().sendMessage(msg);
        } break;

        case UPDATE_ENGINE_INFO:
        case UPDATE_GAME_INFO: {
            CHECK_INTERFACE(IOIfaceService, data, reply);

            BinderMessage msg;
            msg.what = (code == UPDATE_GAME_INFO) ? BINDER_MESSAGE_UPDATE_GAME_INFO : BINDER_MESSAGE_UPDATE_ENGINE_INFO;
            msg.uid = uid;
            msg.pid = pid;
            msg.json = String8(data.readString16()).string();

            OIfaceServer::getInstance().sendRemoteLog("{\"json\":" + msg.json + "}\n");

            OIfaceServer::getInstance().sendMessage(msg);

        } break;

        case REQUEST_RESOURCE: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            BinderMessage msg;
            msg.what = BINDER_MESSAGE_REQUEST_RESOURCE;
            msg.uid = uid;
            msg.pid = pid;
            msg.json = String8(data.readString16()).string();

            OIfaceServer::getInstance().sendRemoteLog("{\"json\":" + msg.json + "}\n");

            OIfaceServer::getInstance().sendMessage(msg);
        } break;
        case GET_MEMORY_USAGE: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            vector<MemoryInfo> info;
            MemoryInfo::getProcessMemoryInfo(&info,pid);
            if (info.size() > 0) {
                reply->writeInt32(info[0].getTotalPss());
            } else {
                reply->writeInt32(-1);
            }
        } break;
        case GET_SYSTEM_TEMP: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            int32_t tempTypeId = data.readInt32();
            if (tempTypeId ==0){
                reply->writeInt32(ThermalService::getInstance().getCpuTemperature());
            } else {
                reply->writeInt32(-1);
            }
        } break;
        case SET_GENERAL_SINGAL: {
            CHECK_INTERFACE(IOIfaceService, data, reply);
            string singalInfo = String8(data.readString16()).string();
            BinderMessage msg;
            msg.json = singalInfo;
            msg.what = BINDER_MESSAGE_UPDATE_GAME_INFO;
            msg.uid = uid;
            msg.pid = pid;
            OIfaceServer::getInstance().sendMessage(msg);
        } break;
        case GET_VERSION: {
            /* XXX: check tag from server */
            CHECK_INTERFACE(IOIfaceService, data, reply);
            String16 version(OIFACE_VERSION);
            if (version.size() == 0)
                version = String16("2.0");
            reply->writeNoException();
            reply->writeString16(version);
        } break;
        case GET_DEVICE_ID_NEW:
        case GET_DEVICE_ID: {
            CHECK_INTERFACE(IOIfaceService, data, reply);

            vector<string> pkgList;

            getPackageForUid(uid, pkgList);
            if (pkgList.size() <= 0) {
                ERROR("unable to get package name");
                reply->writeInt32(-1);
                break;
            }

            if (!checkPermission(String16(OIFACE_PERMISSION), pid, uid)) {
                INFO("No game permission for get device id, check package");

                if (strcmp(SGAME_PACKAGE_NAME, pkgList[0].c_str())
                    || !sgameSignatureMatch()) {
                    ERROR("Package check fail");
                    reply->writeInt32(-1);
                    break;
                }
            }

            reply->writeNoException();
            DEBUG("Return open ID: %s\n", getOpenID());
            reply->writeString16(String16(getOpenID()));
        } break;
        case DUMP_PKG_INFO:{
            CHECK_INTERFACE(IOIfaceService, data, reply);

             /* 当前package name需要rsa加密，后续应该修改成Game Permission */
            string rsaPackage = String8(data.readString16()).string();
            Json::Value outputData;
            DEBUG("rsaPakcage: %s", rsaPackage.c_str());
            if(rsaPackage.empty() || rsaPackage.length() == 0){
                reply->writeString16(String16("-1"));
                break;
            }
            string errorInfo;
            string pkg_name = GeneralCheck::getInstance().decryptPkgName(rsaPackage,errorInfo);
            if (pkg_name.empty()) {
                ERROR("RSA Dumptool decrypt PkgName error %s",errorInfo.c_str());
                reply->writeString16(String16(errorInfo.c_str()));
                break;
            }

            outputData["pkgName"] = pkg_name;
            const Json::Value &allData(DataCollector::getInstance().getData());
            if (!allData[pkg_name].isNull() && allData[pkg_name].isObject()) {
                string array[] = {"type","accRatio","accTimes","accAllTime","invalidTimes","connect","disconnect",
                                    "duration","action","request"};
                int numbers = (int)ARRAY_SIZE(array);
                for(int i = 0; i < numbers; i++){
                    outputData[array[i]] = allData[pkg_name][array[i]];
                }
                if (!allData[pkg_name]["internal"].isNull() && allData[pkg_name]["internal"].isArray()) {
                    int index = 0;
                    for (auto& obj: allData[pkg_name]["internal"]) {
                        if (!obj["done"].isInt())
                            continue;
                        if (obj["done"].asInt() != 1)
                            continue;
                        outputData["internal"][index] = Json::Value(Json::objectValue);
                        string internalArray[] = {"start_time","current","sound","brightness","play_time","avg_fps",
                        "accRatio","accTimes","accAllTime","invalidTimes"};
                        int internalSize = (int)ARRAY_SIZE(internalArray);
                        for(int j = 0; j < internalSize; j++){
                            outputData["internal"][index][internalArray[j]] = obj[internalArray[j]];
                        }
                        index++;
                    }
                }
            }

            String8 result;
            result.append(outputData.toStyledString().c_str());
            reply->writeString16(String16(result));

        } break;
        default:
            return BBinder::onTransact(code, data, reply, flags);
    }
    return NO_ERROR;
}

void BpOIfaceNotifier::onSystemNotify(const String16& result) {
    android::Parcel data, reply;
    android::status_t rv = android::NO_ERROR;
    data.writeInterfaceToken(BpOIfaceNotifier::getInterfaceDescriptor());
    data.writeString16(result);
    // binder call
    rv = remote()->transact(SYSTEM_NOTIFY, data, &reply, IBinder::FLAG_ONEWAY);
    if (rv != android::NO_ERROR) {
        ALOGE("%s : Couldn't contact remote: %d",__func__, rv);
        return;
    }

    int32_t err = reply.readExceptionCode();
    if (err < 0) {
        ALOGE("%s: remote exception: %d",__func__, err);
        return;
    }
}
