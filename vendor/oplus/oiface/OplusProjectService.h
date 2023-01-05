#ifndef __oplus_project_SERVICE_H__
#define __oplus_project_SERVICE_H__

#include <string>
#include <vector>
#include <hidl/HidlSupport.h>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
#include <vendor/oplus/hardware/stability/oplus_project/1.0/IOplusProject.h>


class OplusProjectServiceDeathRecipient;
class OplusProjectService : public android::Singleton<OplusProjectService> {
public:
    uint32_t getProject();
    std::string getProjectString();
    std::string getSerialID();
    void handleServiceDied();


private:
    friend class android::Singleton<OplusProjectService>;
//    friend class OplusProjectServiceDeathHandler;
    OplusProjectService();
    ~OplusProjectService();
    bool checkService();

    android::Mutex mLock;
    android::sp<::vendor::oplus::hardware::stability::oplus_project::V1_0::IOplusProject> mService = nullptr;
    android::sp<OplusProjectServiceDeathRecipient> mDeathRecipient = nullptr;
};

class OplusProjectServiceDeathRecipient : public android::hardware::hidl_death_recipient {
public:
    OplusProjectServiceDeathRecipient(OplusProjectService& Service): mOplusProjectService(Service) {}
    virtual void serviceDied(uint64_t cookie,
            const android::wp<::android::hidl::base::V1_0::IBase>& who) override;
private:
    OplusProjectService& mOplusProjectService;
};


#endif

